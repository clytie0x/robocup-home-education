"""Wheels-raised check: where is the 0.12 m/s cap, and does the firmware stop by itself?

  python tools/air_check.py --yes-wheels-raised

Every wheel must be off the floor, the owner at the robot with a hand on the power switch.
Part 1 sends raw packets F008 to F037 and a turn for 4 s each (past params.json's clamps,
on purpose) and reads the firmware odometry at 20 Hz. If F037 spins faster than F015 in the
air, the cap comes from the load or the supply, not a firmware clamp.
Part 2 sends one F008 packet and then nothing for 1.5 s. If the wheels still turn, the
firmware has no command timeout and only the Pi's watchdog stops the base.
S# is sent after every step and on any exit. The report goes to logs/air-check/.
"""

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # params.py, wheels.py
import params  # noqa: E402
import wheels as wheels_module  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "logs" / "air-check"
STEPS = [b"F008|+000#", b"F012|+000#", b"F015|+000#", b"F020|+000#", b"F030|+000#", b"F037|+000#",
         b"F000|+050#"]
STEP_S = 4.0  # s per packet; the speed comes from the last 2 s, after the ramp
PERIOD = 0.05  # s, 20 Hz odometry, well inside the 0.3 s watchdog
REST_S = 2.0  # s of S# between steps
SILENCE_S = 1.5  # s without any byte: longer than any sensible firmware timeout
GAP_S = 0.5  # s between the two odometry reads after the silence


def rates(log, p):
    """Speeds over the last 2 s of rows (time, x, y, th) in firmware units."""
    tail = [r for r in log if r[0] >= log[-1][0] - 2.0]
    dt = tail[-1][0] - tail[0][0]
    if dt <= 0:
        raise ValueError("not enough odometry rows")
    dist = math.hypot(tail[-1][1] - tail[0][1], tail[-1][2] - tail[0][2])
    yaw = tail[-1][3] - tail[0][3]
    return {"fw_speed": round(dist / dt, 3), "true_speed": round(dist / dt / p["odom_distance_scale"], 3),
            "fw_yaw_rate": round(yaw / dt, 3), "true_yaw_rate": round(yaw / dt / p["odom_yaw_scale"], 3)}


def moved(a, b, metres=0.005, radians=0.01):
    """Did the odometry change between two reads (x, y, th)?"""
    return math.hypot(b[0] - a[0], b[1] - a[1]) > metres or abs(b[2] - a[2]) > radians


def hold(w, packet, seconds, clock=time.monotonic, sleep=time.sleep):
    """Repeat a raw packet for `seconds`, reading odometry each period; S# afterwards."""
    log = []
    t0 = clock()
    try:
        while clock() - t0 < seconds:
            with w.lock:  # raw write: past packet()'s max_speed clamp, the watchdog repeats it
                w.last, w.sent_at = packet, w.clock()
                w.port.write(packet)
            log.append((clock(),) + w.read_odom())
            sleep(max(0.0, PERIOD - (clock() - log[-1][0])))
    finally:
        w.stop()
    return log


def silence(w, packet=b"F008|+000#", clock=time.monotonic, sleep=time.sleep):
    """One packet, then nothing. w.last stays S#, so the watchdog neither repeats nor stops it."""
    try:
        with w.lock:
            w.last = b"S#"
            w.port.write(packet)
        sleep(SILENCE_S)
        a = w.read_odom()
        sleep(GAP_S)
        b = w.read_odom()
    finally:
        w.stop()
    return {"packet": packet.decode(), "silence_s": SILENCE_S, "odom_a": a, "odom_b": b,
            "still_moving_after_silence": moved(a, b)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yes-wheels-raised", action="store_true", help="every wheel is off the floor")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="folder for the JSON report")
    args = parser.parse_args()
    if not args.yes_wheels_raised:
        sys.exit("REFUSED: raise every wheel off the floor, then pass --yes-wheels-raised")
    if not wheels_module.under_safe_run():
        sys.exit("REFUSED: " + wheels_module.NOT_SAFE_RUN)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit("STOPPED: SIGTERM"))  # so finally sends S#

    p = params.load()
    raw = dict(p, odom_distance_scale=1.0, odom_yaw_scale=1.0)  # read firmware units
    w = wheels_module.Wheels(raw)
    report = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "params": p, "steps": [], "silence": None}
    try:
        for packet in STEPS:
            log = hold(w, packet, STEP_S)
            rec = dict(packet=packet.decode(), samples=len(log), **rates(log, p))
            report["steps"].append(rec)
            print(json.dumps(rec), flush=True)
            for _ in range(int(REST_S / 0.1)):  # S# again and again, in case one is lost
                w.stop()
                time.sleep(0.1)
        report["silence"] = silence(w)
        print(json.dumps(report["silence"]), flush=True)
    finally:
        w.close()
        args.out.mkdir(parents=True, exist_ok=True)
        path = args.out / time.strftime("%Y%m%d-%H%M%S-air.json")
        path.write_text(json.dumps(report, indent=1))
        print("closed, last packet S#. Report:", path)


if __name__ == "__main__":
    main()
