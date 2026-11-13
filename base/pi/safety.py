"""Last-resort stop, independent of the brain. Points are base-frame (x forward, y left)."""
import math

import numpy

import footprint

FRONT = 0.20  # m, the front edge of the body from the base centre
CORNERS = 0.16  # m, where the cut front corners start
SIDE_MARGIN = 0.05  # m, extra width each side of the body
CYCLE = 0.1  # s, one control cycle: the command now may run this long before the next check
SWEEP_DT = 0.05  # s, between two outline poses of the sweep
BLIND_CAP = 0.3  # m/s, top speed when there is no depth image
CLEARANCE = 0.30  # m, left ahead after a full stop at any speed (spec criterion 8: never closer than 0.3 m)


def braking_distance(v, p, margin=0.15):
    """How far the base runs on after S# from speed v, plus `margin` still free after it stops."""
    if p["decel"] is None or p["stop_delay"] is None:
        raise ValueError("decel and stop_delay are not measured yet in params.json")
    return v * v / (2 * p["decel"]) + v * p["stop_delay"] + margin


def run_on_poses(v, w, p, cycle=CYCLE):
    """Poses (x, y, th) the base passes if (v, w) runs one more cycle, then S# is sent.

    Full speed for `cycle` + stop_delay, then v falls at decel and w at turn_accel (measured).
    """
    if p["decel"] is None or p["stop_delay"] is None or p.get("turn_accel") is None:
        raise ValueError("decel, stop_delay and turn_accel must be measured in params.json")
    t0 = cycle + p["stop_delay"]
    end = t0 + max(v / p["decel"], abs(w) / p["turn_accel"])
    poses, x, y, th, t = [], 0.0, 0.0, 0.0, 0.0
    while t < end:
        late = max(0.0, t - t0)  # the speed at the start of the step, the larger one
        vt = max(0.0, v - p["decel"] * late)
        wt = math.copysign(max(0.0, abs(w) - p["turn_accel"] * late), w)
        t += SWEEP_DT
        th += wt * SWEEP_DT
        x += vt * math.cos(th) * SWEEP_DT
        y += vt * math.sin(th) * SWEEP_DT
        poses.append((x, y, th))
    return poses


class Coast:
    """What the base may still be doing after the commands it got (review 2026-11-14).

    The brain sets its own speed to 0 at a stop, but the base rolls on: each part (forward speed,
    left turn, right turn) keeps the largest speed it was asked for during stop_delay, then
    slows at the measured decel or turn_accel. Left and right are kept apart, so a turn that
    flips direction still checks the old one until it has died away.
    """

    def __init__(self, p):
        if p["decel"] is None or p["stop_delay"] is None or p.get("turn_accel") is None:
            raise ValueError("decel, stop_delay and turn_accel must be measured in params.json")  # at the start, not mid-run
        self.p = p
        self.parts = {"v": (0.0, 0.0), "left": (0.0, 0.0), "right": (0.0, 0.0)}  # (speed, when asked)

    def _now(self, part, t):
        speed, since = self.parts[part]
        rate = self.p["decel"] if part == "v" else self.p["turn_accel"]
        return max(0.0, speed - rate * max(0.0, t - since - self.p["stop_delay"]))

    def motions(self, v, w, t):
        """The (v, w) pairs must_stop should check at time t for the command (v, w)."""
        cv = self._now("v", t)
        forward = v if not v < cv else cv  # NaN stays NaN, so must_stop stops
        checks = [(forward, w)]
        for part, sign in (("left", 1.0), ("right", -1.0)):
            still = self._now(part, t)
            if still > 0 and not (w * sign >= still):  # turning that way faster than asked: check it
                checks.append((forward, sign * still))
        return checks

    def sent(self, v, w, t):
        """The wheels got (v, w) at time t."""
        for part, asked in (("v", v), ("left", max(w, 0.0)), ("right", max(-w, 0.0))):
            if asked > 0 and asked >= self._now(part, t):
                self.parts[part] = (asked, t)


def must_stop(points_base, v, w, p, cycle=CYCLE):
    """True when a point is in the path the body sweeps before it could stop.

    Moving forward (v > 0): the strip 0 < x - FRONT < run-on distance + CLEARANCE, |y| < half width
    + margin. So the base always stops with CLEARANCE left, also when creeping, when the run-on
    distance alone would let it get within a few centimetres.
    Turning (w != 0): the real outline swept along the run-on arc (run_on_poses) must stay
    footprint.MARGIN from every point, or not come closer to a point already nearer than that.
    The strip also covers the cut corners in front (x > the corners' start, outside the outline).
    `cycle` is how long this command may run before the next check (the measured cycle).
    Any NaN in v, w or the points means stop.
    """
    if math.isnan(v) or math.isnan(w):
        return True
    if v < 0:
        raise ValueError("the follower never drives backward")
    pts = numpy.asarray(points_base, dtype=float).reshape(-1, 2)
    if numpy.isnan(pts).any():
        return True
    x, y = pts[:, 0], pts[:, 1]
    if v > 0:
        ahead = x - FRONT
        corner = (x > CORNERS) & (numpy.abs(y) < p["robot_half_width"]) & (footprint.body_gap(pts, 0.0, 0.0, 0.0) > 0)
        front = (ahead > 0) | corner  # ahead of the front edge, or in a cut corner in front
        if numpy.any(front & (ahead < braking_distance(v, p, margin=CLEARANCE))
                     & (numpy.abs(y) < p["robot_half_width"] + SIDE_MARGIN)):
            return True
    return bool(w != 0 and not footprint.sweep_clear(pts, run_on_poses(v, w, p, cycle)))


def speed_cap(depth_points, p):
    """A blind depth camera (None) is never 'nothing there': slow down. An empty array is a real view."""
    return p["max_speed"] if depth_points is not None else min(p["max_speed"], BLIND_CAP)
