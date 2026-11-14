"""Task 5: command latency, from base_check logs.

  python tools/latency.py logs/base-check/*-line-*.json logs/base-check/*-turn-*.json

For each log: seconds from drive_start to the first odometry sample that moved MOVED or turned
TURNED from where the base stood. Odometry steps by 0.01 m and 0.01 rad, so this is an upper
bound: it includes the time to roll that far (about 0.1 s at 0.12 m/s). Prints one line per
log, then the median and the worst.
"""
import argparse
import json
import math
import statistics
from pathlib import Path

MOVED = 0.015  # m, more than one 0.01 m odometry step after scaling
TURNED = math.radians(1.0)


def first_motion(log):
    """Seconds from drive_start to the first sample that moved MOVED or turned TURNED, or None."""
    t0 = log["drive_start"]
    odom = log["odom"]  # [time, x, y, yaw]
    before = [o for o in odom if o[0] <= t0] or odom[:1]
    _, x0, y0, th0 = before[-1]
    for t, x, y, th in odom:
        if t > t0 and (math.hypot(x - x0, y - y0) >= MOVED or abs(th - th0) >= TURNED):
            return t - t0
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    args = ap.parse_args()
    found = []
    for name in args.logs:
        log = json.loads(Path(name).read_text())
        if "drive_start" not in log or not log.get("odom"):
            continue  # reports and silence checks
        lag = first_motion(log)
        print("%-45s %s" % (Path(name).name, "no motion" if lag is None else "%.3f s" % lag))
        if lag is not None:
            found.append(lag)
    if found:
        print("median %.3f s, worst %.3f s, best %.3f s over %d logs" % (
            statistics.median(found), max(found), min(found), len(found)))


if __name__ == "__main__":
    main()
