"""The arm moves from a shell only for a logged repair (design 2026-11-16, "Console gate in our tools"; a copy of
base/pi/console_gate.py, which the Jetson cannot import). The console moves the arm through the agent (every
program the agent starts has HMC_CONSOLE_JOB=agent:<name>); bag_arm.py --pick/--place and floor_pick.py are repair
tools: HMC_MAINTENANCE=<reason> lets them through and is logged in ~/hmc-follow/logs/commands.jsonl."""
import json
import os
import time
from pathlib import Path

ENV = "HMC_CONSOLE_JOB"  # set by agent.py for every program it starts: "agent:<name>"
MAINTENANCE_ENV = "HMC_MAINTENANCE"
LOG = Path(__file__).resolve().parent / "logs" / "commands.jsonl"
NOT_CONSOLE = ("the arm moves through the console (its Arm card, robotctl arm ..., the task flow); for a repair"
               " from this shell set HMC_MAINTENANCE=<reason>, which is logged")


def require(what, environ=None, clock=time.time, log=LOG):
    """Returns when `what` (e.g. "arm") may move from here; raises RuntimeError otherwise."""
    environ = os.environ if environ is None else environ
    if environ.get(ENV):
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
