"""Tools that move the arm run only when the robot console started them (design 2026-11-16, "Console gate in
our tools"; a copy of base/pi/console_gate.py, which this package cannot import). A repair by hand sets
HMC_MAINTENANCE=<reason>, which is logged and let through. Needs no ROS."""
import json
import os
import sys
import time
from pathlib import Path

ENV = "HMC_CONSOLE_JOB"  # set by the console for every job it starts
CONSOLE_ENV = "HMC_CONSOLE"  # "1" for the console's own processes
MAINTENANCE_ENV = "HMC_MAINTENANCE"
LOG_ENV = "HMC_COMMAND_LOG"
LOG = Path.home() / "hmc-follow" / "logs" / "commands.jsonl"  # the Jetson's command log (agent GET /status)
NOT_CONSOLE = ("start this through the console (robotctl on the Pi / the console page), or set"
               " HMC_MAINTENANCE=<reason> for a logged repair")


def require(what, environ=None, clock=time.time, log=None):
    """Returns when `what` (a tool, e.g. "arm_cli") may move the arm here; raises RuntimeError otherwise."""
    environ = os.environ if environ is None else environ
    if environ.get(ENV) or environ.get(CONSOLE_ENV) == "1":
        return
    reason = environ.get(MAINTENANCE_ENV, "").strip()
    if not reason:
        raise RuntimeError("REFUSED: %s: %s" % (what, NOT_CONSOLE))
    line = {"time": clock(), "what": what, "reason": reason, "pid": os.getpid(),
            "user": environ.get("USER") or environ.get("USERNAME"), "ssh": environ.get("SSH_CLIENT")}
    log = Path(log or environ.get(LOG_ENV) or LOG)
    try:
        log.parent.mkdir(parents=True, exist_ok=True)
        with open(log, "a", encoding="utf-8") as f:
            f.write(json.dumps(line) + "\n")
    except OSError as exc:  # a repair must not fail because the log is unwritable
        print("WARNING: %s: could not write the command log %s: %s" % (what, log, exc), file=sys.stderr)
