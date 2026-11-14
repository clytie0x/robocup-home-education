"""The robot's real outline (hmc.world.FOOTPRINT, a 0.40 m square with 0.04 m corners cut) against points."""
import math

import numpy

from hmc.world import FOOTPRINT

MARGIN = 0.03  # m, kept between the outline and any point (spec goal 1)
TOL = 1e-9  # m, float noise only: any more would let a slow turn creep closer every cycle
GRID = 0.01  # m, points are merged to cells this size before the sweep (time on the Pi; margin grows 7 mm)
_A = numpy.array(FOOTPRINT, dtype=float)  # corners, counter-clockwise
_AB = numpy.roll(_A, -1, axis=0) - _A
REACH = float(numpy.hypot(_A[:, 0], _A[:, 1]).max())  # 0.256 m, the farthest corner from the centre


def body_gap(points, x, y, th):
    """Distance (m) from each point to the outline of the robot at pose (x, y, th); negative inside."""
    pts = numpy.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    local = numpy.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)[:, None, :]  # (N, 1, 2)
    rel = local - _A  # (N, 8, 2)
    t = numpy.clip((rel * _AB).sum(axis=2) / (_AB * _AB).sum(axis=1), 0.0, 1.0)
    dist = numpy.hypot(*(rel - t[..., None] * _AB).transpose(2, 0, 1)).min(axis=1)
    inside = ((_AB[:, 0] * rel[..., 1] - _AB[:, 1] * rel[..., 0]) >= 0).all(axis=1)
    return numpy.where(inside, -dist, dist)


def sweep_clear(points, poses, margin=MARGIN):
    """True when the outline, placed at every pose, stays `margin` from every point.

    A point already closer than `margin` at the start pose (0, 0, 0) blocks only the poses that
    bring it closer still, so the robot can always move away from or along a wall it stands at.
    """
    pts = numpy.asarray(points, dtype=float).reshape(-1, 2)
    if not len(pts) or not len(poses):
        return True
    far = max(math.hypot(px, py) for px, py, _ in poses) + REACH + margin
    pts = pts[numpy.hypot(pts[:, 0], pts[:, 1]) < far]  # only what the outline can reach
    if not len(pts):
        return True
    # one point per GRID cell, at its centre; the margin grows by the farthest a point moved
    pts = (numpy.unique(numpy.floor(pts / GRID), axis=0) + 0.5) * GRID
    margin += GRID * math.sqrt(0.5)
    allowed = numpy.minimum(body_gap(pts, 0.0, 0.0, 0.0), margin) - TOL
    return all(not (body_gap(pts, px, py, pth) < allowed).any() for px, py, pth in poses)
