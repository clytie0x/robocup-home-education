"""Run one console job and stop it when the console that started it is gone.

  python tools/job_guard.py <program> <arguments>

console.py starts every job through this, as the leader of a new process group. The guard starts
the program in the same group and checks its own parent every POLL s. When the parent changes
(the console was killed, even with kill -9, so nobody else will stop the job), it sends SIGTERM
to the whole group: run_follow and the check tools stop on it, tools/safe_run.py ignores it and
sends S# once its child has ended. Whatever is left after GRACE s gets SIGKILL (the firmware stops
the wheels by itself 0.3 s after the last drive command). The guard ignores SIGINT and SIGTERM
itself, so it ends only after the program does, and exits with the program's code.
"""
import os
import signal
import subprocess
import sys
import time

POLL = 0.2  # s between parent checks
GRACE = 5.0  # s from SIGTERM to SIGKILL


def stop_group(child, sleep=time.sleep, clock=time.monotonic):
    """SIGTERM to our group (we ignore it), then SIGKILL to all of it if the child is still there."""
    if os.name != "posix":
        child.terminate()
        return
    try:
        os.killpg(os.getpgrp(), signal.SIGTERM)
    except OSError:
        pass
    end = clock() + GRACE
    while child.poll() is None and clock() < end:
        sleep(0.05)
    if child.poll() is None:
        os.killpg(os.getpgrp(), signal.SIGKILL)  # ends this guard too


def guard(cmd, parent=None, getppid=os.getppid, popen=subprocess.Popen, sleep=time.sleep, log=print, stop=stop_group):
    """Run cmd; stop it when getppid() is no longer parent. Returns the exit code to use."""
    parent = getppid() if parent is None else parent
    child = popen(cmd)  # started before the ignores below, so the program keeps the normal signals
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.SIG_IGN)
    stopping = False
    while child.poll() is None:
        if not stopping and getppid() != parent:
            log("job_guard: the console is gone, stopping the job", flush=True)
            stopping = True
            stop(child, sleep)
        sleep(POLL)
    code = child.returncode
    return 128 - code if code < 0 else code  # killed by signal n: 128 + n, as a shell says


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sys.exit(guard(sys.argv[1:]))
