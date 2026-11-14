"""The base tests itself unattended in the lab while the power cable trails from it (owner's rules).

  python tools/safe_run.py .venv/bin/python tools/tether_run.py init [--length 10]
  python tools/safe_run.py .venv/bin/python tools/tether_run.py turn DEG
  python tools/safe_run.py .venv/bin/python tools/tether_run.py line METRES [--speed V] [--back-up-if-blocked]
  python tools/safe_run.py .venv/bin/python tools/tether_run.py uturn [right|left]
  python tools/safe_run.py .venv/bin/python tools/tether_run.py precheck-line METRES V
  python tools/tether_run.py after-base-check LOG.json

One motion per call. Where the cable lies (tether.Tether) is kept in --state (default /tmp/tether.json):
`init` lays it out straight behind the robot at the current odometry pose and surveys the floor with
the camera; every other call loads it and refuses when the odometry is not where it was saved.
Owner's rules: keep track of the cable, never run over it (its points are obstacles), never wind it
(net turn within +-180 deg), never pull it taut (tether.slack_ok), at most MAX_SPEED, every drive
under safe_run, and when driving forward: if the camera sees no floor, stop; if then the LiDAR says
ahead is blocked, back up when behind is free (only `line --back-up-if-blocked`, via tools/back_up.py).
A motion is refused up front when the cable model says any part of it would meet the cable.
`uturn` turns on two quarter arcs (UTURN_RADIUS) with a straight leg between: turning on the spot
past about 74 deg meets the model's own trail (the cable dragged round the back corner).
`after-base-check` feeds a tools/base_check.py log's odometry into the trail (base_check knows
nothing of the cable); `precheck-line` exits 0 only when that line may be driven.
Prints one JSON line; the exit code is 0 only when the motion or check fully succeeded.
"""
import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

import numpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import back_up  # noqa: E402
import base_check  # noqa: E402
import camera  # noqa: E402
import floor_cable  # noqa: E402
import geometry  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402
import run_follow  # noqa: E402
import safety  # noqa: E402
import tether  # noqa: E402
import wheels as wheels_module  # noqa: E402

STATE = "/tmp/tether.json"
SPEED = 0.10  # m/s, line speed by default
MAX_SPEED = 0.12  # m/s, owner's cap for every motion here
MAX_LINE = 2.0  # m per call
TURN_RATE = 0.3  # rad/s
CAMERA_HEIGHT = 0.105  # m: fits the real frames (params.json's 0.07 does not)
PERIOD = 0.05  # s per control cycle
FRAME_WAIT = 5.0  # s for a colour frame before driving forward or at a survey stop
CAMERA_STALE = 0.5  # s without a new frame while driving forward: stop
POSE_TOL, YAW_TOL = 0.05, math.radians(5)  # odometry this far from the saved pose: refuse
PLAN_STEP = 0.02  # m or rad between the poses a plan is checked at
BACK_METRES = 0.2  # backed at back_up.SPEED (0.04 m/s)
UTURN_SIDE = 0.6  # m between the way out and the way back
UTURN_RADIUS = 0.15  # m, the quarter arcs: 0.045 m/s at TURN_RATE
SURVEY = (-60, -30, 0, 30, 60, 0)  # deg from the start heading, in order (past ~74 the model's own trail blocks)
clock, sleep = time.monotonic, time.sleep


class Rig:
    def __init__(self, w, session, cam, p):
        self.w, self.session, self.cam, self.p = w, session, cam, p


def summary(t):
    return {"net_turn_deg": round(math.degrees(t.net_turn), 1), "used_length": round(t.used_length(), 3),
            "cable_points": len(t.points(t.pose)), "pose": [round(q, 3) for q in t.pose]}


def plan_check(t, parts, p):
    """Why the motions [(v, w, seconds), ...] from the tether's pose are not allowed by the cable, else None.

    On a copy of the tether moved along them: a straight part must be tether.clear (0.20 m) where it
    starts, and on every part the per-cycle cable stop (safety.must_stop on the tether's points) must
    not fire, as the trail a turn lays down can come round the body. A turning part gets only the
    second: tether.clear does not model the near cable swinging back, so it would forbid unwinding.
    """
    dw = sum(w * s for _, w, s in parts)
    if not t.turn_allowed(dw) and abs(t.net_turn + dw) > abs(t.net_turn):  # a hair past 180 may still unwind or go straight
        return "would wind the cable: net turn past 180 deg"
    if not t.slack_ok(sum(v * s for v, _, s in parts)):
        return "not enough slack: %.2f m of cable used" % t.used_length()
    sim = copy.deepcopy(t)
    x, y, th = sim.pose
    for v, w, s in parts:
        if not w and not sim.clear((x, y, th), v, w, p, seconds=s):
            return "the cable is in the way"
        n = max(1, math.ceil(max(v * s, abs(w) * s) / PLAN_STEP))
        for _ in range(n):
            th += w * s / n
            x, y = x + v * math.cos(th) * s / n, y + v * math.sin(th) * s / n
            sim.update((x, y, th))
            if safety.must_stop(sim.points((x, y, th)), v, w, p):
                return "the cable would be in the way at (%.2f, %.2f, %.0f deg)" % (x, y, math.degrees(th))
    return None


def look(frame, t, pose, p):
    """(why the floor cannot be judged or None, cable points added). Points within TRAIL_ATTACHED of
    the post are the part that swings with the robot, not kept."""
    if not floor_cable.floor_visible(frame, p, height=CAMERA_HEIGHT):
        return "camera blind", 0
    post = geometry.base_to_odom(tether.ATTACH, pose)
    seen = [q for q in (geometry.base_to_odom(c, pose) for c in floor_cable.cable_points(frame, p, height=CAMERA_HEIGHT))
            if math.dist(q, post) > tether.TRAIL_ATTACHED]
    before = len(t.seen)
    t.add_seen(seen)
    return None, len(t.seen) - before


def frame_check(rig, t):
    """look() at the next colour frame, waiting up to FRAME_WAIT for one."""
    deadline = clock() + FRAME_WAIT
    while True:
        frame = rig.cam.latest()
        if frame is not None:
            return look(frame, t, t.pose, rig.p)
        if rig.cam.error is not None or clock() > deadline:
            return "camera silent", 0
        sleep(PERIOD)


def settle(rig, t, v, w):
    """Follow the run-on after S# into the tether until the base must have stopped."""
    p = rig.p
    end = clock() + p["stop_delay"] + max(v / p["decel"], abs(w) / p["turn_accel"]) + 0.5
    while True:
        try:
            t.update(rig.w.read_odom())
        except (ValueError, OSError):
            pass  # the next call refuses if the saved pose is off
        if clock() >= end:
            return
        sleep(0.1)


def move(rig, t, v, w, amount):
    """`amount` m straight at v (w = 0), else `amount` rad at w (on the spot, or an arc with v > 0),
    checked every cycle. The stop reason, "there" when done; it stops early by the run-on."""
    p, wh = rig.p, rig.w
    start = last = wh.read_odom()
    t.update(start)
    run_on = safety.run_on_poses(v, abs(w), p, cycle=0.0)[-1]
    early, rate = (run_on[2], abs(w)) if w else (run_on[0], v)
    limit = clock() + 2 * amount / rate + 5.0
    turned, framed, reason = 0.0, clock(), None
    try:
        while reason is None:
            pose = wh.read_odom()
            t.update(pose)
            turned += math.remainder(pose[2] - last[2], math.tau)
            last = pose
            done = abs(turned) if w else (pose[0] - start[0]) * math.cos(start[2]) + (pose[1] - start[1]) * math.sin(start[2])
            if done >= amount - early:
                reason = "there"
                break
            if clock() > limit:
                reason = "time"
                break
            pts = run_follow.scan_points(rig.session.latest(), p)
            if not len(pts):
                reason = "no LiDAR points"
                break
            if v > 0:  # driving forward: the camera must see the floor ahead
                frame = rig.cam.latest()
                if frame is not None:
                    framed = clock()
                    reason, added = look(frame, t, pose, p)
                    if reason is None and added and plan_check(t, [(v, w, max(amount - done, 0.0) / rate)], p):
                        reason = "cable ahead"
                elif rig.cam.error is not None or clock() - framed > CAMERA_STALE:
                    reason = "camera silent"
                if reason:
                    break
            if safety.must_stop(pts, v, w, p):
                reason = "blocked ahead" if v else "turn blocked"
                break
            if safety.must_stop(t.points(pose), v, w, p):
                reason = "cable"
                break
            wh.send(v, w)
            sleep(PERIOD)
    except (ValueError, OSError) as error:
        reason = "odometry: %s" % error
    finally:
        wh.stop()
    settle(rig, t, v, w)
    return reason


def ready(rig, t, v, w, amount):
    """("refused", why) by the cable model, ("stop", why) by the camera when driving forward, or None."""
    seconds = amount / (abs(w) if w else v)
    why = plan_check(t, [(v, w, seconds)], rig.p)
    if why:
        return "refused", why
    if v > 0:
        why, added = frame_check(rig, t)
        if why is None and added and plan_check(t, [(v, w, seconds)], rig.p):
            why = "cable ahead"
        if why:
            return "stop", why
    return None


def go(rig, t, v, w, amount):
    """(kind, why) of one checked motion, as move()."""
    return ready(rig, t, v, w, amount) or ("stop", move(rig, t, v, w, amount))


def turn(rig, t, rad, v=0.0):
    """Turn `rad` (left positive) on the spot, or along an arc with v > 0."""
    return go(rig, t, v, math.copysign(TURN_RATE, rad), abs(rad))


def whole_cable(t, pose):
    """(N, 2) odometry points of the whole cable, the part hanging from the post included, and the
    camera's points: for backing, which can run over the attached part too."""
    poly = t._cable(pose)
    along = numpy.r_[0.0, numpy.cumsum(numpy.hypot(*numpy.diff(poly, axis=0).T))]
    at = numpy.arange(0.0, along[-1] + 1e-9, tether.STEP)
    return numpy.vstack([numpy.stack([numpy.interp(at, along, poly[:, 0]), numpy.interp(at, along, poly[:, 1])], axis=1),
                         numpy.array(t.seen).reshape(-1, 2)])


def back(rig, t):
    """Back up BACK_METRES with tools/back_up.py's checks, the whole cable as its avoid points."""
    pose = rig.w.read_odom()
    t.update(pose)
    avoid = whole_cable(t, pose)
    why = back_up.blocked(run_follow.scan_points(rig.session.latest(), rig.p), pose, BACK_METRES, avoid, rig.p)
    if why:
        return {"back_up": "refused: " + why}
    result = back_up.drive(rig.w, rig.session, rig.p, BACK_METRES, avoid, clock, sleep)
    settle(rig, t, back_up.SPEED, 0.0)
    return {"back_up": result["stop"], "backed": result["travelled"]}


def survey(rig, t):
    """Turn to each SURVEY heading, look at the floor there; back to the start heading however it ends."""
    stops, reason = [], None
    for deg in SURVEY:
        rad = math.radians(deg) - t.net_turn
        if abs(rad) > math.radians(1):
            kind, why = turn(rig, t, rad)
            if why != "there":
                reason = "%s at %d deg: %s" % (kind, deg, why)
                break
        blind, added = frame_check(rig, t)
        stops.append({"deg": deg, "floor_visible": blind is None, "cable_points": added})
    if abs(t.net_turn) > math.radians(1):
        kind, why = turn(rig, t, -t.net_turn)
        if why != "there":
            reason = (reason + "; " if reason else "") + "return %s: %s" % (kind, why)
    return {"survey": stops, "stop": reason or "there"}


def uturn(rig, t, side):
    """Quarter arc, straight leg, quarter arc: UTURN_SIDE to that side, facing back."""
    rad = math.pi / 2 if side == "left" else -math.pi / 2
    v, w, leg = UTURN_RADIUS * TURN_RATE, math.copysign(TURN_RATE, rad), UTURN_SIDE - 2 * UTURN_RADIUS
    quarter = (v, w, abs(rad) / TURN_RATE)
    why = plan_check(t, [quarter, (SPEED, 0.0, leg / SPEED), quarter], rig.p)
    if why:
        return {"refused": why}
    end = t.net_turn + 2 * rad  # the second arc turns what is left, so the first one's error does not wind past it
    for part, step in (("first arc", lambda: turn(rig, t, rad, v)), ("leg", lambda: go(rig, t, SPEED, 0.0, leg)),
                       ("second arc", lambda: turn(rig, t, end - t.net_turn, v))):
        kind, why = step()
        if why != "there":
            return {kind: why, "part": part}
    return {"stop": "there"}


def after_base_check(t, log_path):
    odom = json.loads(Path(log_path).read_text())["odom"]
    why = pose_off(t, odom[0][1:4]) if odom else "no odometry in the log"
    if why:
        return {"refused": why}
    for _, x, y, yaw in odom:
        t.update((x, y, yaw))
    return {"stop": "there", "samples": len(odom)}


def pose_off(t, pose):
    if math.dist(pose[:2], t.pose[:2]) > POSE_TOL or abs(math.remainder(pose[2] - t.pose[2], math.tau)) > YAW_TOL:
        return ("odometry %s is not the saved pose %s: something moved the base (run after-base-check) or the"
                " firmware restarted (run init)" % ([round(q, 3) for q in pose], [round(q, 3) for q in t.pose]))
    return None


def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", type=Path, default=Path(STATE), help="the tether state file")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("init", help="lay the cable straight behind and survey the floor")
    s.add_argument("--length", type=float, default=10.0, help="cable length (m)")
    s = sub.add_parser("turn", help="turn on the spot, left positive")
    s.add_argument("deg", type=float)
    s = sub.add_parser("line", help="drive straight forward")
    s.add_argument("metres", type=float)
    s.add_argument("--speed", type=float, default=SPEED)
    s.add_argument("--back-up-if-blocked", action="store_true", help="back up %.1f m when ahead is blocked" % BACK_METRES)
    s = sub.add_parser("uturn", help="to the side %.1f m and facing back" % UTURN_SIDE)
    s.add_argument("side", nargs="?", choices=("right", "left"), default="right")
    s = sub.add_parser("after-base-check", help="feed a base_check log's odometry into the trail")
    s.add_argument("log", type=Path)
    s = sub.add_parser("precheck-line", help="exit 0 only if this line may be driven")
    s.add_argument("metres", type=float)
    s.add_argument("speed", type=float)
    args = ap.parse_args(argv)
    if args.cmd in ("line", "precheck-line") and not (0 < args.metres <= MAX_LINE and 0 < args.speed <= MAX_SPEED):
        sys.exit("REFUSED: a line is (0, %.1f] m at (0, %.2f] m/s" % (MAX_LINE, MAX_SPEED))
    if args.cmd == "turn" and not 0 < abs(args.deg) <= 180:
        sys.exit("REFUSED: a turn is (0, 180] deg either way")
    if args.cmd == "init" and not (math.isfinite(args.length) and args.length > tether.SLACK_MARGIN):
        sys.exit("REFUSED: --length must be more than %.1f m" % tether.SLACK_MARGIN)
    return args


def run(args, t, rig):
    """The result dict of the command."""
    if args.cmd == "init":
        return survey(rig, t)
    if args.cmd == "turn":
        return dict([turn(rig, t, math.radians(args.deg))])
    if args.cmd == "uturn":
        return uturn(rig, t, args.side)
    if args.cmd == "precheck-line":
        return dict([ready(rig, t, args.speed, 0.0, args.metres) or ("stop", "clear")])
    kind, why = go(rig, t, args.speed, 0.0, args.metres)
    result = {kind: why}
    if why in ("blocked ahead", "camera blind"):  # the owner's rule: then the LiDAR decides
        result["blocked_ahead"] = bool(safety.must_stop(run_follow.scan_points(rig.session.latest(), rig.p),
                                                        args.speed, 0.0, rig.p))
        if result["blocked_ahead"] and args.back_up_if_blocked:
            result.update(back(rig, t))
    return result


def main(argv=None):
    args = parse(argv)
    p = params.load()
    if args.cmd != "init" and not args.state.exists():
        sys.exit("REFUSED: no tether state %s: run init first" % args.state)
    if args.cmd == "after-base-check":  # no port: the base_check run is over
        t = tether.Tether.load(args.state, p)
        result = after_base_check(t, args.log)
        if "refused" not in result:
            t.save(args.state)
    else:
        if not wheels_module.under_safe_run():
            sys.exit("REFUSED: " + wheels_module.NOT_SAFE_RUN)
        t = None if args.cmd == "init" else tether.Tether.load(args.state, p)
        session = lidar.LidarSession()
        w = cam = None
        try:
            base_check.first_full_scan(session, {"scans": []}, clock, sleep)
            w = wheels_module.Wheels(p)
            if args.cmd != "turn":
                cam = camera.Camera()
                cam.start()
            pose = w.read_odom()
            if t is None:
                t = tether.Tether(p, pose, args.length)
                t.save(args.state)
            why = pose_off(t, pose)
            if why:
                result = {"refused": why}
            else:
                t.update(pose)
                try:
                    result = run(args, t, Rig(w, session, cam, p))
                finally:
                    t.save(args.state)
        finally:
            if cam is not None:
                cam.stop()
            if w is not None:
                w.stop()
                w.close()
            session.close()
    result = dict({"cmd": args.cmd}, **result, **summary(t))
    print(json.dumps(result), flush=True)
    return 0 if result.get("stop") in ("there", "clear") else 1


if __name__ == "__main__":
    sys.exit(main())
