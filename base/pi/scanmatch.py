"""How far the base moved between two LiDAR scans, by point-to-point ICP.

The result (dx, dy, dyaw) is the pose of the base at the second scan, seen from the base at
the first scan: dx forward, dy left (metres), dyaw left (radians). Only lidar.usable points
are used, in the base frame, so the motion is that of the turning centre.
"""
import math

import numpy as np

import geometry
import lidar
import params

AGREE = 0.05  # m, a point "agrees" when its nearest neighbour is this close after the match
SURFACE_GAP = 0.10  # m, neighbouring returns closer than this are taken to be on one surface


def points(scan, p):
    """lidar.usable points of a scan as an (N, 2) array in the base frame."""
    xy = [geometry.lidar_to_base((m * math.cos(math.radians(deg)), m * math.sin(math.radians(deg))), p) for deg, m in lidar.usable(scan)]
    return np.array(xy, dtype=float).reshape(-1, 2)


def tangents(pts):
    """Unit direction of the surface at each point, from its neighbours in the scan (0 when alone)."""
    prev, nxt = np.roll(pts, 1, axis=0), np.roll(pts, -1, axis=0)  # scans are in angle order
    a = np.where((np.hypot(*(pts - prev).T) < SURFACE_GAP)[:, None], prev, pts)
    b = np.where((np.hypot(*(nxt - pts).T) < SURFACE_GAP)[:, None], nxt, pts)
    length = np.hypot(*(b - a).T)[:, None]
    return np.divide(b - a, length, out=np.zeros_like(pts), where=length > 0)


def reach(pts, keep):
    """How far along its surface each kept return stands for: to the farthest of the returns it
    replaces (half a stride either side) that are joined to it by gaps under SURFACE_GAP, at least
    SURFACE_GAP / 2. A thinned return must stand for the wall between it and the next one kept."""
    n = len(pts)
    half = max(1, math.ceil((keep[1] - keep[0]) / 2)) if len(keep) > 1 else 1
    gap = np.hypot(*(pts[1:] - pts[:-1]).T)  # gap[i]: from return i to i + 1
    far = np.full(len(keep), SURFACE_GAP / 2)
    for direction in (1, -1):
        joined = np.ones(len(keep), dtype=bool)
        for s in range(1, half + 1):
            j = keep + direction * s
            inside = (j >= 0) & (j < n)
            jc = np.clip(j, 0, n - 1)
            step = np.clip(np.minimum(jc, jc - direction), 0, n - 2)  # the gap crossed by this step
            joined &= inside & (gap[step] < SURFACE_GAP)
            far = np.where(joined, np.maximum(far, np.hypot(*(pts[jc] - pts[keep]).T)), far)
    return far


def _closest(moved, fixed, along, half=SURFACE_GAP / 2):
    """For every moved point, the closest point on the fixed surface, and the distance to it.

    The surface near a return is the short line through it along its tangent, `half` long either
    side (one value or one per fixed return). Matching to the returns themselves (3.5 cm apart at
    2 m) pulls small steps toward zero along walls.
    """
    d2 = ((moved[:, None, :] - fixed[None, :, :]) ** 2).sum(axis=2)
    index = d2.argmin(axis=1)
    q, u = fixed[index], along[index]
    h = half[index] if np.ndim(half) else half
    s = np.clip(((moved - q) * u).sum(axis=1), -h, h)
    target = q + s[:, None] * u
    return target, np.hypot(*(moved - target).T)


def _best_rotation(src, dst):
    """Rotation R and shift t that best map src onto dst (least squares, Kabsch)."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    h = (src - cs).T @ (dst - cd)
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return r, cd - r @ cs


def match(before, after, p=None, guess=(0.0, 0.0, 0.0), iterations=60, max_points=None):
    """(dx, dy, dyaw, agree): the motion, and the share of points that agree within AGREE.

    guess is a starting motion; ICP only finds the nearest fit, so a big turn needs one.
    dyaw is returned within 180 deg of the guessed yaw, not wrapped to +-180 deg.
    max_points: match at most this many returns of each scan (every k-th), for speed. The surface
    lines of `before` come from all its returns first, and each kept return's line reaches over the
    returns it replaces: thinned returns stand too far apart to be one surface, and matching to bare
    returns pulls the motion toward zero (all the more the further the walls).
    Raises ValueError when a scan has fewer than 20 usable points or they do not overlap.
    """
    p = p or params.load()
    fixed, moving = points(before, p), points(after, p)
    if len(fixed) < 20 or len(moving) < 20:
        raise ValueError("too few usable points: %d and %d" % (len(fixed), len(moving)))
    along = tangents(fixed)
    half = SURFACE_GAP / 2
    if max_points:
        keep = np.arange(0, len(fixed), math.ceil(len(fixed) / max_points))
        half = reach(fixed, keep)
        fixed, along = fixed[keep], along[keep]
        moving = moving[::math.ceil(len(moving) / max_points)]
    x, y, yaw = guess
    r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t = np.array([x, y], dtype=float)
    reject = 0.5  # m, pairs farther apart are ignored; the band narrows as the fit improves
    for _ in range(iterations):
        moved = moving @ r.T + t
        target, dist = _closest(moved, fixed, along, half)
        keep = dist < reject
        if keep.sum() < 10:
            raise ValueError("the scans do not overlap")
        step_r, step_t = _best_rotation(moved[keep], target[keep])
        r, t = step_r @ r, step_r @ t + step_t
        # 3e-5 m / rad: 1e-5 took twice the iterations with the longer thinned surface lines (37 ms median
        # on the Pi); 5e-5 left a 0.012 m step over 1.5 % short (test_scanmatch)
        small = np.hypot(*step_t) < 3e-5 and abs(math.atan2(step_r[1, 0], step_r[0, 0])) < 3e-5
        if small and reject <= 0.1:
            break
        reject = max(0.1, reject * 0.7)
    _, dist = _closest(moving @ r.T + t, fixed, along, half)
    turned = math.atan2(r[1, 0], r[0, 0])
    turned = yaw + math.atan2(math.sin(turned - yaw), math.cos(turned - yaw))  # near the guess, so 190 deg stays 190
    return float(t[0]), float(t[1]), turned, float((dist < AGREE).mean())


def scan_shift(before, after, p=None, guess=(0.0, 0.0, 0.0)):
    """(dx, dy, dyaw) of the base from scan `before` to scan `after`; see match()."""
    return match(before, after, p, guess)[:3]


def scan_match(reference, scan, guess=(0.0, 0.0, 0.0), p=None):
    """(dx, dy, dyaw, share, slide): the pose `scan` was taken from in the frame of `reference`,
    the share of its points agreeing within AGREE (see match()), and slide, 0 to 1: how freely the
    match can slide in one direction. slide is 1 when every agreeing point lies on surfaces of one
    direction (two parallel walls: the match says nothing along them), near 0 when the directions
    spread evenly (a room with corners)."""
    p = p or params.load()
    dx, dy, dyaw, share = match(reference, scan, p, guess, max_points=120)
    fixed, moving = points(reference, p), points(scan, p)
    along = tangents(fixed)
    c, s = math.cos(dyaw), math.sin(dyaw)
    moved = moving[::math.ceil(len(moving) / 120)] @ np.array([[c, -s], [s, c]]).T + (dx, dy)
    d2 = ((moved[:, None, :] - fixed[None, :, :]) ** 2).sum(axis=2)
    index = d2.argmin(axis=1)
    u = along[index[d2[np.arange(len(moved)), index] < AGREE ** 2]]
    u = u[np.hypot(*u.T) > 0]
    if len(u) == 0:
        return dx, dy, dyaw, share, 1.0
    low, high = np.linalg.eigvalsh(u.T @ u / len(u))  # spread of the surface directions
    return dx, dy, dyaw, share, float(1.0 - low / high)
