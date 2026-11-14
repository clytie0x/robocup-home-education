"""Back the base up a little, straight, when ahead is blocked but behind is free (owner's request).

  python tools/safe_run.py .venv/bin/python tools/back_up.py --metres 0.2 --yes-back-up [--avoid trail.json]

The only program that drives the base backward (wheels.Wheels.send_reverse, B packets); everything
else stays forward-only (wheels.packet, safety.must_stop). Straight back at SPEED, at most MAX_METRES.
Before the first packet and on every scan after it, for the distance still left plus the run-on
after S#: the real outline must stay clear of the LiDAR points (safety.rear_clear) and AVOID_MARGIN
from every --avoid point, by the rule of footprint.sweep_clear (a point already nearer only blocks
getting closer). --avoid is a JSON list of [x, y] in the odometry frame, or a file written by
tether.Tether.save (its cable points, without the part attached to the robot).
Stops at the distance (odometry), on any check failing, on no scan, on odometry going forward or not
moving for STALL_S while backing, or after time_limit(metres); S# at the end. Prints one JSON line.
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
import footprint  # noqa: E402
import geometry  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402
import run_follow  # noqa: E402
import safety  # noqa: E402
import tether  # noqa: E402
import wheels as wheels_module  # noqa: E402

SPEED = 0.04  # m/s backward
MAX_METRES = 0.3
AVOID_MARGIN = 0.20  # m between the outline and any --avoid point, anywhere on the path
FORWARD_LIMIT = 0.02  # m: odometry going this far forward means B does not back up; stop
STALL_S, STALL_M = 1.0, 0.005  # backing commanded this long without the odometry moving this far: stop
PERIOD = 0.05


def time_limit(metres):
    """Seconds allowed for backing `metres`: half as long again as at SPEED, plus 1 s."""
    return metres / SPEED * 1.5 + 1.0


def load_avoid(path, p=None):
    """(N, 2) odometry-frame points from a JSON list of [x, y], or from a tether.Tether.save file."""
    data = json.loads(Path(path).read_text())
    if isinstance(data, dict):
        t = tether.Tether.load(path, p or params.load())
        data = [geometry.base_to_odom(q, t.pose) for q in t.points(t.pose)]
    pts = numpy.asarray(data, dtype=float)
    if pts.size == 0:
        return numpy.empty((0, 2))
    if pts.ndim != 2 or pts.shape[1] != 2 or not numpy.isfinite(pts).all():
        raise ValueError("not a list of finite [x, y]")
    return pts


def blocked(pts, pose, left, avoid, p):
    """Why backing `left` more metres from odometry `pose` is not safe now, else None."""
    if not len(pts):
        return "no LiDAR points"
    if not safety.rear_clear(pts, SPEED, p, left):
        return "rear blocked"
    if len(avoid):
        local = numpy.array([geometry.odom_to_base(a, pose) for a in avoid])
        if not footprint.sweep_clear(local, safety.rear_poses(SPEED, p, left), AVOID_MARGIN):
            return "avoid point within %.2f m of the path" % AVOID_MARGIN
    return None


def drive(w, session, p, metres, avoid, clock, sleep):
    """Back up `metres`, checking every scan; S# however it ends. The result dict."""
    start = w.read_odom()
    c, s = math.cos(start[2]), math.sin(start[2])
    t0 = clock()
    moved_pose, moved_at = start, None  # moved_at: when the odometry last moved STALL_M while backing
    result = {"metres": metres, "stop": None}
    try:
        while True:
            pose = w.read_odom()
            if math.dist(pose[:2], moved_pose[:2]) > STALL_M:
                moved_pose, moved_at = pose, clock()
            elif moved_at is not None and clock() - moved_at > STALL_S:
                result["stop"] = "odometry not moving"
                break
            back = -((pose[0] - start[0]) * c + (pose[1] - start[1]) * s)
            if back < -FORWARD_LIMIT:
                result["stop"] = "odometry went forward"
                break
            if back >= metres:
                result["stop"] = "there"
                break
            if clock() - t0 > time_limit(metres):
                result["stop"] = "time"
                break
            reason = blocked(run_follow.scan_points(session.latest(), p), pose, metres - max(back, 0.0), avoid, p)
            if reason:
                result["stop"] = reason
                break
            w.send_reverse(SPEED, 0.0)
            if moved_at is None:
                moved_at = clock()  # the first packet starts the stall clock
            sleep(PERIOD)
    finally:
        w.stop()
    sleep(1.0)  # let it roll to a stop
    result["travelled"] = round(math.dist(w.read_odom()[:2], start[:2]), 3)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--metres", type=float, required=True, help="how far to back up (m), at most %.1f" % MAX_METRES)
    ap.add_argument("--yes-back-up", action="store_true", help="I know the base drives backward")
    ap.add_argument("--avoid", type=Path, help="JSON list of [x, y] odometry-frame points to keep %.2f m from" % AVOID_MARGIN)
    args = ap.parse_args()
    if not args.yes_back_up:
        sys.exit("REFUSED: pass --yes-back-up (this tool drives the base backward)")
    if not 0 < args.metres <= MAX_METRES:
        sys.exit("REFUSED: --metres must be in (0, %.1f]" % MAX_METRES)
    if not wheels_module.under_safe_run():
        sys.exit("REFUSED: " + wheels_module.NOT_SAFE_RUN)
    p = params.load()
    avoid = numpy.empty((0, 2))
    if args.avoid is not None:
        try:
            avoid = load_avoid(args.avoid, p)
        except (OSError, ValueError, TypeError, KeyError) as error:
            sys.exit("REFUSED: --avoid %s: %s" % (args.avoid, error))
    session = lidar.LidarSession()
    w = None
    try:
        scan = base_check.first_full_scan(session, {"scans": []}, time.monotonic, time.sleep)
        w = wheels_module.Wheels(p)
        reason = blocked(run_follow.scan_points(scan, p), w.read_odom(), args.metres, avoid, p)
        if reason:
            sys.exit("REFUSED: " + reason)
        result = drive(w, session, p, args.metres, avoid, time.monotonic, time.sleep)
    finally:
        if w is not None:
            w.stop()
            w.close()
        session.close()
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
