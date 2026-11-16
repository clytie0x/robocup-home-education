"""The LiDAR, the Xtion and the wheel port open only in a program the robot console started (design
2026-11-16, "Console gate in our tools"): one owner for each device, checked here rather than by OS
permissions. A repair by hand sets HMC_MAINTENANCE=<reason>, which is logged and let through."""
import json
import os
import time
from pathlib import Path

ENV = "HMC_CONSOLE_JOB"  # set by console.py for every job it starts: "<start time>-<kind>", its log's name
CONSOLE_ENV = "HMC_CONSOLE"  # "1", set by console.py for its own process (its live view, camera_pipe, S#)
MAINTENANCE_ENV = "HMC_MAINTENANCE"
LOG = Path(__file__).resolve().parent / "logs" / "console" / "commands.jsonl"
NOT_CONSOLE = ("start this through the console (robotctl check ... / the console page), or set"
               " HMC_MAINTENANCE=<reason> for a logged repair")


def require(what, environ=None, clock=time.time, log=LOG):
    """Returns when `what` (a device, e.g. "lidar") may open here; raises RuntimeError otherwise."""
    environ = os.environ if environ is None else environ
    if environ.get(ENV) or environ.get(CONSOLE_ENV) == "1":
        return
    reason = environ.get(MAINTENANCE_ENV, "").strip()
    if not reason:
        raise RuntimeError("REFUSED: %s: %s" % (what, NOT_CONSOLE))
    line = {"time": clock(), "what": what, "reason": reason, "pid": os.getpid(),
            "user": environ.get("USER") or environ.get("USERNAME"), "ssh": environ.get("SSH_CLIENT")}
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    with open(log, "a", encoding="utf-8") as f:
        f.write(json.dumps(line) + "\n")
