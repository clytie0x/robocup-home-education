"""Turn to the nearest object ahead and creep up to it until it is `--to` metres from the base centre.

  python tools/safe_run.py .venv/bin/python tools/approach.py --to 0.37 [--yes-close-approach]

For handing an object to the arm: the follower's last-resort stop keeps 0.30 m free ahead, which
is further than the arm reaches. This tool goes closer, but only at CREEP (0.03 m/s, under 1.5 cm
of run-on after S#) and only towards the one object it locked onto; anything else inside
OTHER_CLEAR of the front strip, a lost target, more than MAX_TRAVEL, or MAX_SECONDS stops it.
The target is the nearest LiDAR cluster within AHEAD_DEG of straight ahead and FIND_RANGE.
Prints one JSON line with where the target ended up.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import base_check  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402
import run_follow  # noqa: E402
import safety  # noqa: E402
import wheels as wheels_module  # noqa: E402

CREEP = 0.03  # m/s
TURN = 0.2  # rad/s
AHEAD_DEG = 30.0
FIND_RANGE = 1.0  # m from the base centre
CLUSTER = 0.06  # m: points this close to the target's nearest point belong to it
TRACK_JUMP = 0.08  # m: the target may move this much between scans (we move under 0.01)
OTHER_CLEAR = 0.08  # m: anything else this close ahead of the front edge stops the creep
MAX_TRAVEL = 0.35  # m
MAX_SECONDS = 25.0
TURN_DONE = math.radians(2)


def target_of(pts, last=None):
    """(x, y) of the target: near `last` when given, else the nearest point within AHEAD_DEG and FIND_RANGE."""
    if not len(pts):
        return None
    if last is not None:
        d = numpy.hypot(pts[:, 0] - last[0], pts[:, 1] - last[1])
        if d.min() > TRACK_JUMP:
            return None
        near = pts[d < d.min() + CLUSTER]
    else:
        r = numpy.hypot(pts[:, 0], pts[:, 1])
        ahead = (numpy.abs(numpy.degrees(numpy.arctan2(pts[:, 1], pts[:, 0]))) <= AHEAD_DEG) & (r <= FIND_RANGE) & (pts[:, 0] > safety.FRONT)
        if not ahead.any():
            return None
        first = pts[ahead][numpy.argmin(r[ahead])]
        near = pts[numpy.hypot(pts[:, 0] - first[0], pts[:, 1] - first[1]) < CLUSTER]
    k = numpy.argmin(numpy.hypot(near[:, 0], near[:, 1]))
    return float(near[k, 0]), float(near[k, 1])


def blocked_by_other(pts, target):
    """True when a point that is not the target is within OTHER_CLEAR ahead of the front edge, in the body's width."""
    if not len(pts):
        return False
    other = numpy.hypot(pts[:, 0] - target[0], pts[:, 1] - target[1]) > 2 * CLUSTER
    ahead = (pts[:, 0] > safety.CORNERS) & (pts[:, 0] - safety.FRONT < OTHER_CLEAR) & (numpy.abs(pts[:, 1]) < 0.22)
    return bool((other & ahead).any())


def creep_speed(target_x, to):
    """Forward speed for the distance left (target x in the base frame), 0 when there."""
    left = target_x - to
    return 0.0 if left <= 0.003 else min(CREEP, max(0.01, left))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--to", type=float, required=True, help="end with the target this far ahead of the base centre (m)")
    ap.add_argument("--yes-close-approach", action="store_true", help="I know this goes closer than the 0.30 m rule")
    args = ap.parse_args()
    if not args.yes_close_approach:
        sys.exit("REFUSED: pass --yes-close-approach (this tool goes closer than the follower's 0.30 m clearance)")
    if not wheels_module.under_safe_run():
        sys.exit("REFUSED: " + wheels_module.NOT_SAFE_RUN)
    if not safety.FRONT + 0.05 <= args.to <= 1.0:
        sys.exit("REFUSED: --to must be between %.2f and 1.0 m" % (safety.FRONT + 0.05))
    p = params.load()
    session = lidar.LidarSession()
    w = None
    result = {"stop": None}
    try:
        scan = base_check.first_full_scan(session, {"scans": []}, time.monotonic, time.sleep)
        target = target_of(run_follow.scan_points(scan, p))
        if target is None:
            sys.exit("REFUSED: nothing within %.0f deg and %.1f m ahead" % (AHEAD_DEG, FIND_RANGE))
        result["found"] = [round(target[0], 3), round(target[1], 3)]
        print("target at (%.3f, %.3f)" % target, flush=True)
        w = wheels_module.Wheels(p)
        t0 = time.monotonic()
        # 1. turn to face it (two passes: turn, look again)
        for _ in range(2):
            bearing = math.atan2(target[1], target[0])
            if abs(bearing) < TURN_DONE:
                break
            yaw0 = w.read_odom()[2]
            while abs(w.read_odom()[2] - yaw0) < abs(bearing) - math.radians(0.5):
                pts = run_follow.scan_points(session.latest() or scan, p)
                if safety.must_stop(pts, 0.0, math.copysign(TURN, bearing), p):
                    result["stop"] = "turn blocked"
                    break
                w.send(0.0, math.copysign(TURN, bearing))
                if time.monotonic() - t0 > MAX_SECONDS:
                    result["stop"] = "time"
                    break
                time.sleep(0.05)
            w.stop()
            time.sleep(0.6)  # the base rolls on, then a fresh scan
            target = target_of(run_follow.scan_points(session.latest(), p), None)
            if target is None or result["stop"]:
                result["stop"] = result["stop"] or "lost the target after turning"
                break
        # 2. creep straight at it
        start = w.read_odom()
        last = None
        while result["stop"] is None:
            scan = session.latest()
            pts = run_follow.scan_points(scan, p)
            target = target_of(pts, target)
            if target is None:
                result["stop"] = "lost the target"
                break
            v = creep_speed(target[0], args.to)
            if v == 0.0:
                result["stop"] = "there"
                break
            if blocked_by_other(pts, target):
                result["stop"] = "something else ahead"
                break
            pose = w.read_odom()
            if math.dist(pose[:2], start[:2]) > MAX_TRAVEL:
                result["stop"] = "max travel"
                break
            if time.monotonic() - t0 > MAX_SECONDS:
                result["stop"] = "time"
                break
            w.send(v, 0.0)
            last = target
            time.sleep(0.05)
        w.stop()
        time.sleep(1.0)  # let it roll to a stop, then measure where the target is
        final = target_of(run_follow.scan_points(session.latest(), p), last or target)
        result["target"] = None if final is None else [round(final[0], 3), round(final[1], 3)]
        result["travelled"] = round(math.dist(w.read_odom()[:2], start[:2]), 3)
    finally:
        if w is not None:
            w.stop()
            w.close()
        session.close()
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
