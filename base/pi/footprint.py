"""The robot's real outline (hmc.world.FOOTPRINT, a 0.40 m square with 0.04 m corners cut) against points."""
import math

import numpy

from hmc.world import FOOTPRINT

MARGIN = 0.03  # m, kept between the outline and any point (spec goal 1)
TOL = 1e-9  # m, float noise only: any more would let a slow turn creep closer every cycle
GRID = 0.01  # m, points are merged to cells this size before the sweep (time on the Pi; margin grows 7 mm)
# A return on the skin of the outline. The farthest corner is 0.256 m from the centre; the LiDAR body
# box only drops what is inside 0.18 m, so a return at ~0.26 m (the 18 Nov stops) is the robot itself.
# 1.5 cm covers that shell. 2 cm beside the side (the geometry test) stays an obstacle.
SKIN = 0.015
# Bearings are base-frame degrees, 0 straight ahead, positive left. Ranges are from the base centre.
# The front corners of the outline sit near ±39 deg. The chest wedge is only the hanging bag and cable:
# a return at 0.325 m straight ahead (the braking-strip test) stays outside it.
DEFAULT_SECTORS = (
    {"name": "arm", "min_deg": 32.0, "max_deg": 55.0, "max_m": 0.36},
    {"name": "cable", "min_deg": -55.0, "max_deg": -32.0, "max_m": 0.36},
    {"name": "bag", "min_deg": -15.0, "max_deg": 15.0, "max_m": 0.30},
)
LAST = {}  # sensor -> {count, nearest} from the last drop_self, for the cycle log
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


def _mask_config(p):
    """(skin, sectors, extra polygons). Missing self_mask uses the defaults; an empty sectors list means none."""
    if not p or "self_mask" not in p:
        return SKIN, DEFAULT_SECTORS, []
    cfg = p.get("self_mask") or {}
    skin = float(cfg.get("skin_m", SKIN))
    sectors = cfg["sectors"] if "sectors" in cfg else DEFAULT_SECTORS
    return skin, sectors, cfg.get("polygons") or []


def _inside(points, poly):
    """Even-odd ray cast: True where each point sits inside the polygon (base frame, metres)."""
    x, y = points[:, 0], points[:, 1]
    inside = numpy.zeros(len(points), dtype=bool)
    count = len(poly)
    for i in range(count):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % count]
        crosses = (y1 > y) != (y2 > y)
        # a horizontal edge never crosses; the tiny term keeps the division defined
        edge = x < (x2 - x1) * (y - y1) / ((y2 - y1) + 1e-15) + x1
        inside ^= crosses & edge
    return inside


def describe(sensor, xy):
    """One base-frame point as the log wants it: which sensor, and the bearing it was seen on."""
    x, y = float(xy[0]), float(xy[1])
    return {"sensor": sensor, "bearing_deg": round(math.degrees(math.atan2(y, x)), 1),
            "range_m": round(math.hypot(x, y), 3), "x": round(x, 3), "y": round(y, 3)}


def own_mask(points, p=None):
    """True for each base-frame point that is the robot: the outline's skin, a configured sector, or an extra polygon.

    Sectors are the arm, the cable and a carried bag, which stick out past the 0.18 m LiDAR body box and
    read as an obstacle about 0.26 m from the centre (18 Nov: every stop was that close).
    """
    pts = numpy.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return numpy.zeros(0, dtype=bool)
    skin, sectors, polygons = _mask_config(p)
    own = body_gap(pts, 0.0, 0.0, 0.0) <= skin
    x, y = pts[:, 0], pts[:, 1]
    bearing = numpy.degrees(numpy.arctan2(y, x))
    rng = numpy.hypot(x, y)
    for sector in sectors:
        lo, hi = float(sector["min_deg"]), float(sector["max_deg"])
        angle = (bearing >= lo) & (bearing <= hi) if lo <= hi else (bearing >= lo) | (bearing <= hi)
        own |= angle & (rng >= float(sector.get("min_m", 0.0))) & (rng <= float(sector.get("max_m", 0.36)))
    for poly in polygons:
        ring = numpy.asarray(poly, dtype=float).reshape(-1, 2)
        if len(ring) >= 3:
            own |= _inside(pts, ring)
    return own


def drop_self(points, p=None, sensor=None):
    """The points that are not the robot. With sensor, LAST[sensor] records how many were dropped and the nearest."""
    pts = numpy.asarray(points, dtype=float).reshape(-1, 2)
    own = own_mask(pts, p)
    dropped = pts[own] if len(pts) else pts
    nearest = None
    if len(dropped):
        i = int(numpy.hypot(dropped[:, 0], dropped[:, 1]).argmin())
        nearest = describe(sensor, dropped[i])
    if sensor is not None:
        LAST[sensor] = {"sensor": sensor, "count": int(own.sum()) if len(own) else 0, "nearest": nearest}
    return pts[~own] if len(pts) else pts
