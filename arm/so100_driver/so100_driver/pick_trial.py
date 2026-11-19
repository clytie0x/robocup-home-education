"""One JSON file per bag-pick trial. Pure: no ROS, no camera.

Written under ~/hmc-follow/runs (bag_trial_dir) by bag_camera.CameraBag.pick.
A trial that cannot be written is logged and does not fail the pick.
"""
import hashlib
import json
import os
import time


ICT_OFFSET_S = 7 * 3600  # Indochina Time, UTC+7, no daylight saving


def ict_timestamp(epoch):
    """ISO-8601 in Indochina Time (UTC+7) for a Unix epoch second."""
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(float(epoch) + ICT_OFFSET_S)) + "+07:00"


def new_trial_id(epoch, pid, n):
    """Id of one trial: ICT time to the second, process id, and a counter. Safe as a file name."""
    stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime(float(epoch) + ICT_OFFSET_S))
    return "%s-%s-%s" % (stamp, int(pid), int(n))


def camera_params_hash(params):
    """First 16 hex chars of sha256 over the calibration dict (key order does not matter)."""
    raw = json.dumps(params, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()[:16]


def read_commit(version_path=None, environ=None, run=None):
    """Git commit of the code on this machine: HMC_GIT_COMMIT, else VERSION.json, else `git rev-parse HEAD`.

    `run(args) -> str` replaces the git call in tests. Anything else, including a missing git, is "unknown".
    """
    environ = os.environ if environ is None else environ
    forced = (environ.get("HMC_GIT_COMMIT") or "").strip()
    if forced:
        return forced
    path = version_path or os.path.expanduser("~/hmc-follow/VERSION.json")
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        commit = data.get("commit") if isinstance(data, dict) else None
        if isinstance(commit, str) and commit.strip():
            return commit.strip()
    except (OSError, ValueError):
        pass
    if run is None:
        import subprocess

        def run(args):
            return subprocess.run(args, capture_output=True, text=True, timeout=2).stdout
    try:
        line = (run(["git", "rev-parse", "HEAD"]) or "").strip()
    except (OSError, ValueError):
        return "unknown"
    if len(line) >= 7 and all(c in "0123456789abcdef" for c in line):
        return line
    return "unknown"


def build_trial(trial_id, ict, commit, camera_hash, requested, measured, phases, confidences,
                success, failure, frames):
    """The one record written for a trial. `failure` is None when the pick held the bag."""
    return {
        "trial_id": trial_id,
        "ict": ict,
        "commit": commit,
        "camera_params_hash": camera_hash,
        "requested": requested,
        "measured": measured,
        "phases_s": dict(phases),
        "confidences": dict(confidences),
        "success": bool(success),
        "failure": failure,
        "frames": list(frames),
    }


def write_trial(record, directory):
    """Write `record` as one JSON file named by its trial_id. Returns the path."""
    os.makedirs(directory, exist_ok=True)
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(record["trial_id"]))
    path = os.path.join(directory, safe + ".json")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, path)
    return path


class PhaseClock:
    """Durations (seconds) of named phases. `phase(name)` ends the previous phase and starts `name`.

    `close()` ends the phase that is still open. Time before the first phase, when nothing has started,
    is stored as "tail".
    """

    def __init__(self, now=None):
        self._now = now or time.monotonic
        self._mark = self._now()
        self._open = None
        self.phases = {}
        self.closed = False

    def phase(self, name):
        if self.closed:
            return
        self._add(self._open, self._now())
        self._open = name

    def close(self, name="tail"):
        """End the open phase (or store the whole interval as `name` when none was started), once."""
        if self.closed:
            return
        self._add(self._open or name, self._now())
        self._open = None
        self.closed = True

    def _add(self, name, t):
        if name is None:
            self._mark = t
            return
        self.phases[name] = round(self.phases.get(name, 0.0) + (t - self._mark), 3)
        self._mark = t
