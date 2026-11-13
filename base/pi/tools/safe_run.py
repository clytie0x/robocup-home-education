"""Run a drive program, then send S# however it ended: also kill -9, the OOM killer or a crash.

  python tools/safe_run.py .venv/bin/python run_follow.py --drive --seconds 60
  python tools/safe_run.py .venv/bin/python tools/base_check.py --line 0.08 --yes-owner-at-robot

The firmware has no command timeout (measured 2026-11-13: still driving 1.5 s after the last
packet), so a drive program that dies before its own S# leaves the base on its last command.
This parent outlives the child and stops the base. It cannot help when the Pi loses power or
the USB cable comes out: keep a hand near the motor cut-off button.
"""
import signal
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # params.py, wheels.py
import params  # noqa: E402
import wheels  # noqa: E402

STOP_SIGNALS = [s for s in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)) if s is not None]


def open_wheels():
    return wheels.Wheels(params.load(), start_thread=False)  # opening already sends S#


def run(cmd, open_wheels=open_wheels):
    """The child's exit code, after S# went to the base."""
    child = subprocess.Popen(cmd)  # started first, so the child keeps the normal Ctrl+C handling
    for sig in STOP_SIGNALS:
        signal.signal(sig, signal.SIG_IGN)  # Ctrl+C and a dropped ssh stop the child; we stay for S#
    code = child.wait()
    try:
        w = open_wheels()
        try:
            w.stop()
        finally:
            w.close()
        print("safe_run: the program ended (code %s); S# sent" % code, flush=True)
    except Exception as error:  # the port is gone: say so loudly, nothing else can be done here
        print("safe_run: COULD NOT SEND S# (%s). Press the motor cut-off button." % error, flush=True)
        return code if code else 1
    return code


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sys.exit(run(sys.argv[1:]))
