"""The bag measured from a distance on the Pi: its top height from the Xtion depth image and its long-axis yaw from
its LiDAR points. Pure (numpy only); bag.py asks for it in the far stage, while the bag is 0.6 to 1.5 m away (the
Xtion gives no depth nearer than about 0.5 m, and at the pick distance nothing sees the bag at all:
base/docs/hardware-assessment-2026-11-13.md, last section). See base/docs/bag-sensing.md.

Top height. The camera pose in params.json is not trusted for absolute heights: camera_height 0.07 is known to be
about 0.105, camera_pitch_deg 12.3 comes from a wall fit. Every point of one depth frame carries the same pose error,
and a wrong height or pitch is a rigid move of the whole cloud (an offset, a rotation about the camera). So the floor
round the bag's base is fitted as a plane in the SAME frame, and the top is the highest bag point's distance from
that plane, measured square to it: a rigid move changes neither, so the height error and the pitch error cancel
(a wrong field of view is no rigid move and does not cancel).

Yaw. The LiDAR cuts the bag at 0.245 m and sees the side or the two sides facing it; the longer one is taken as the
bag's long axis (the handles of a gift bag run along it). One side seen end-on gives the short axis instead: the
confidence then is low only when that side is short.

Frames: base (x ahead, y left, z up from the floor, origin at the turning centre). The depth image is mirrored
(image right is the robot's left), as depth_obstacles.py reads it.
"""
import math

import numpy as np

import depth_obstacles

FOOT_R = 0.15  # m round the bag centre: its footprint (a gift bag about 0.25 m long, so 0.125 m plus a margin)
FLOOR_R = (0.20, 0.40)  # m round the bag centre: the floor ring the local plane is fitted to
MIN_UP = 0.03  # m above the local floor: lower is floor (noise, the bag's own foot)
MAX_UP = 0.80  # m: higher is not the bag
DEPTH_RANGE = (0.4, 2.0)  # m along the optical axis: no depth nearer than about 0.5 m, too noisy past 2 m
TOP_K = 3  # the top is the 3rd highest bag point: a single flying pixel at an edge does not set it
TOP_RANGE = (0.10, 0.60)  # m, a top outside this is a wrong measurement
MIN_BAG_POINTS = 40  # bag points for a full confidence
MIN_FLOOR_POINTS = 100  # floor points for a full confidence
FLOOR_RMS = 0.01  # m, a floor fit worse than this lowers the confidence (zero at twice this)
VALID_FULL = 0.6  # share of valid depth pixels in the bag's image box for a full confidence (zero at 0.2)
SHARE_TOP = 0.245  # m: that box reaches up to the LiDAR plane, where the LiDAR saw the bag (bag or floor behind it)
OK_CONF = 0.5  # a measurement under this confidence is not used
YAW_EXTENT = 0.20  # m, a LiDAR side this long gives a full yaw confidence
SIDE_RMS = 0.01  # m, points farther than this from one line are two sides (an L)
ONE_SIDE_MIN = 0.18  # m, one straight side shorter than this may be the bag's narrow end (seen end on: 90 deg off)
SIDE_RATIO = 1.3  # of an L, the longer side must be this much longer than the other to be the long axis


def _pose(p):
    return p["camera_x"], p["camera_y"], p["camera_height"], math.radians(p["camera_pitch_deg"])


def cloud(depth, p):
    """(N, 3) base-frame points (x, y, z) of the valid pixels in DEPTH_RANGE, and their image rows (N,).

    Same conversion as depth_obstacles.points: the camera tilted UP by camera_pitch_deg, the image mirrored."""
    depth = np.asarray(depth)
    valid = (depth >= DEPTH_RANGE[0] * 1000) & (depth <= DEPTH_RANGE[1] * 1000)
    right, down = depth_obstacles._rays(depth.shape, p["camera_hfov_deg"])
    cx, cy, h, pitch = _pose(p)
    z = depth[valid] / 1000.0
    x_cam, y_cam = right[valid] * z, down[valid] * z
    ahead = cx + z * math.cos(pitch) + y_cam * math.sin(pitch)
    left = cy + x_cam
    up = h + z * math.sin(pitch) - y_cam * math.cos(pitch)
    rows = np.nonzero(valid)[0]
    return np.column_stack([ahead, left, up]), rows


def project(points, p, shape):
    """Base-frame (N, 3) points -> image (u, v) (N, 2) in the depth image (the inverse of cloud); nan behind."""
    q = np.asarray(points, float).reshape(-1, 3)
    cx, cy, h, pitch = _pose(p)
    dx, dz = q[:, 0] - cx, q[:, 2] - h
    z = dx * math.cos(pitch) + dz * math.sin(pitch)  # along the optical axis
    y_cam = dx * math.sin(pitch) - dz * math.cos(pitch)
    x_cam = q[:, 1] - cy
    height, width = shape
    f = (width / 2) / math.tan(math.radians(p["camera_hfov_deg"]) / 2)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(z > 1e-6, x_cam / z * f + (width - 1) / 2, np.nan)
        v = np.where(z > 1e-6, y_cam / z * f + (height - 1) / 2, np.nan)
    return np.column_stack([u, v])


def fit_floor(points):
    """Plane z = a x + b y + c through the floor points, robust to things standing on it.
    Returns ((a, b, c), rms, inlier count) or None with fewer than 10 points."""
    q = np.asarray(points, float).reshape(-1, 3)
    if len(q) < 10:
        return None
    keep = np.abs(q[:, 2] - np.median(q[:, 2])) < 0.06
    coef = None
    for _ in range(4):
        if keep.sum() < 10:
            return None
        a = np.column_stack([q[keep, 0], q[keep, 1], np.ones(keep.sum())])
        coef = np.linalg.lstsq(a, q[keep, 2], rcond=None)[0]
        res = q[:, 2] - (coef[0] * q[:, 0] + coef[1] * q[:, 1] + coef[2])
        s = max(0.004, 1.4826 * float(np.median(np.abs(res[keep]))))
        keep = np.abs(res) < 3 * s
    res = q[keep, 2] - (coef[0] * q[keep, 0] + coef[1] * q[keep, 1] + coef[2])
    return tuple(float(c) for c in coef), float(np.sqrt(np.mean(res ** 2))), int(keep.sum())


def above(points, plane):
    """Distance of each point above the plane z = a x + b y + c, square to the plane."""
    a, b, c = plane
    q = np.asarray(points, float).reshape(-1, 3)
    return (q[:, 2] - (a * q[:, 0] + b * q[:, 1] + c)) / math.sqrt(1 + a * a + b * b)


def _ramp(x, zero, full):
    return float(min(1.0, max(0.0, (x - zero) / (full - zero))))


def measure_top(depth, centre, p):
    """The bag's top height above the local floor from one depth frame.

    centre: the bag centre (x, y), base frame, the same moment as the frame (bag.py's LiDAR centre).
    Returns {"ok", "top" (m or None), "conf" (0 to 1), "valid_share", "bag_points", "floor_points", "floor_rms",
    "why"}: ok only with conf >= OK_CONF and the top in TOP_RANGE. Dark, light-absorbing surfaces give no depth:
    then the valid share and the bag points are low, ok is false and the caller falls back."""
    out = {"ok": False, "top": None, "conf": 0.0, "valid_share": None, "bag_points": 0, "floor_points": 0,
           "floor_rms": None, "why": None}
    if depth is None:
        out["why"] = "no depth frame"
        return out
    depth = np.asarray(depth)
    bx, by = float(centre[0]), float(centre[1])
    ring = [(bx + FOOT_R * math.cos(a), by + FOOT_R * math.sin(a), z)
            for a in np.linspace(0, 2 * math.pi, 12, endpoint=False) for z in (0.0, SHARE_TOP)]
    uv = project(ring, p, depth.shape)
    uv = uv[np.isfinite(uv).all(axis=1)]
    height, width = depth.shape
    if not len(uv):
        out["why"] = "the bag is behind the depth camera"
        return out
    u0, v0 = int(max(0, np.floor(uv[:, 0].min()))), int(max(0, np.floor(uv[:, 1].min())))
    u1, v1 = int(min(width, np.ceil(uv[:, 0].max()) + 1)), int(min(height, np.ceil(uv[:, 1].max()) + 1))
    if u1 <= u0 or v1 <= v0:
        out["why"] = "the bag is out of the depth camera's view"
        return out
    out["valid_share"] = round(float((depth[v0:v1, u0:u1] > 0).mean()), 3)
    pts, rows = cloud(depth, p)
    r = np.hypot(pts[:, 0] - bx, pts[:, 1] - by)
    floor = pts[(r >= FLOOR_R[0]) & (r <= FLOOR_R[1])]
    fit = fit_floor(floor)
    if fit is None:
        out["why"] = "too few floor points round the bag (%d)" % len(floor)
        return out
    plane, rms, n_floor = fit
    out["floor_points"], out["floor_rms"] = n_floor, round(rms, 4)
    inside = r <= FOOT_R
    up = above(pts[inside], plane)
    bag = (up > MIN_UP) & (up < MAX_UP)
    out["bag_points"] = int(bag.sum())
    if bag.sum() < TOP_K:
        out["why"] = "no depth on the bag (%d points; valid share %.2f)" % (bag.sum(), out["valid_share"])
        return out
    heights = np.sort(up[bag])
    top = float(heights[-TOP_K])
    out["top"] = round(top, 3)
    clipped = bool((rows[inside][bag] <= 1).any())  # bag pixels in the image's top rows: its top may be out of view
    conf = min(_ramp(out["valid_share"], 0.2, VALID_FULL), _ramp(bag.sum(), 0, MIN_BAG_POINTS),
               _ramp(n_floor, 0, MIN_FLOOR_POINTS), 1.0 - _ramp(rms, FLOOR_RMS, 2 * FLOOR_RMS))
    if clipped:
        conf, out["why"] = 0.0, "the bag reaches the top of the depth image: its top may be higher"
    out["conf"] = round(conf, 2)
    out["ok"] = conf >= OK_CONF and TOP_RANGE[0] <= top <= TOP_RANGE[1]
    if not out["ok"] and out["why"] is None:
        out["why"] = ("top %.3f m outside %.2f to %.2f m" % (top, *TOP_RANGE) if conf >= OK_CONF
                      else "low confidence %.2f" % conf)
    return out


def wrap_line(a):
    """A line's direction in [-pi/2, pi/2) (a line has no front or back)."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def _line(pts):
    """Direction (unit), extent along it and the rms off it, of a total least squares line through pts."""
    m = pts.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((pts - m).T))
    d = v[:, 1]
    t = (pts - m) @ d
    off = (pts - m) @ v[:, 0]
    return d, float(t.max() - t.min()), float(np.sqrt(np.mean(off ** 2)))


def cluster_yaw(points):
    """The bag's long-axis yaw (rad, base frame, a line in [-pi/2, pi/2)) from its LiDAR points, with a confidence.

    One straight side: its direction. Two (an L: the points stray more than SIDE_RMS off one line): split at the
    corner, the point farthest off the line between the two ends, and take the longer side. No yaw when it may
    be the short axis: one side under ONE_SIDE_MIN (the narrow end seen end on), or an L whose sides differ by
    less than SIDE_RATIO. Returns (yaw or None, conf 0 to 1, why)."""
    pts = np.asarray(points, float).reshape(-1, 2)
    if len(pts) < 3:
        return None, 0.0, "fewer than 3 LiDAR points"
    d, extent, rms = _line(pts)
    why = "one side %.2f m" % extent
    if rms > SIDE_RMS and len(pts) >= 6:
        t = (pts - pts.mean(axis=0)) @ d
        order = np.argsort(t)
        a, b = pts[order[0]], pts[order[-1]]
        chord = (b - a) / (np.linalg.norm(b - a) or 1e-9)
        off = np.abs((pts - a) @ np.array([-chord[1], chord[0]]))
        corner = pts[np.argmax(off)]
        tc = (corner - pts.mean(axis=0)) @ d
        sides = [pts[t <= tc], pts[t >= tc]]
        fits = sorted((_line(s) for s in sides if len(s) >= 3), key=lambda f: f[1])
        if len(fits) == 2 and fits[1][1] < SIDE_RATIO * fits[0][1]:
            return None, 0.0, "two sides of about the same length (%.2f, %.2f m)" % (fits[0][1], fits[1][1])
        if fits:
            d, extent, rms = fits[-1]
            why = "two sides, the longer %.2f m" % extent
    elif extent < ONE_SIDE_MIN:
        return None, 0.0, "one side %.2f m: may be the narrow end" % extent
    straight = 1.0 - _ramp(rms, SIDE_RMS, 3 * SIDE_RMS)
    conf = round(min(1.0, extent / YAW_EXTENT) * straight, 2)
    return wrap_line(math.atan2(d[1], d[0])), conf, why
