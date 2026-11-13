"""Last-resort stop, independent of the brain. Points are base-frame (x forward, y left)."""
import math

import numpy

FRONT = 0.20  # m, the front edge of the body from the base centre
SIDE_MARGIN = 0.05  # m, extra width each side of the body
SPIN_RADIUS = 0.33  # m, how far the front corner reaches when turning on the spot
BLIND_CAP = 0.3  # m/s, top speed when there is no depth image
CLEARANCE = 0.30  # m, left ahead after a full stop at any speed (spec criterion 8: never closer than 0.3 m)


def braking_distance(v, p, margin=0.15):
    """How far the base runs on after S# from speed v, plus `margin` still free after it stops."""
    if p["decel"] is None or p["stop_delay"] is None:
        raise ValueError("decel and stop_delay are not measured yet in params.json")
    return v * v / (2 * p["decel"]) + v * p["stop_delay"] + margin


def must_stop(points_base, v, w, p):
    """True when a point is in the path the body sweeps before it could stop.

    Moving forward (v > 0): the strip 0 < x - FRONT < run-on distance + CLEARANCE, |y| < half width
    + margin. So the base always stops with CLEARANCE left, also when creeping, when the run-on
    distance alone would let it get within a few centimetres.
    Turning (w != 0, with or without v): any point inside SPIN_RADIUS on either side, because
    the front corner swings one way and the rear corner the other.
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
        if numpy.any((ahead > 0) & (ahead < braking_distance(v, p, margin=CLEARANCE))
                     & (numpy.abs(y) < p["robot_half_width"] + SIDE_MARGIN)):
            return True
    return bool(w != 0 and numpy.any(numpy.hypot(x, y) < SPIN_RADIUS))


def speed_cap(depth_points, p):
    """A blind depth camera (None) is never 'nothing there': slow down. An empty array is a real view."""
    return p["max_speed"] if depth_points is not None else min(p["max_speed"], BLIND_CAP)
