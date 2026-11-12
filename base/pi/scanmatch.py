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


def _closest(moved, fixed, along):
    """For every moved point, the closest point on the fixed surface, and the distance to it.

    The surface near a return is the short line through it along its tangent. Matching to the
    returns themselves (3.5 cm apart at 2 m) pulls small steps toward zero along walls.
    """
    d2 = ((moved[:, None, :] - fixed[None, :, :]) ** 2).sum(axis=2)
    index = d2.argmin(axis=1)
    q, u = fixed[index], along[index]
    s = np.clip(((moved - q) * u).sum(axis=1), -SURFACE_GAP / 2, SURFACE_GAP / 2)
    target = q + s[:, None] * u
    return target, np.hypot(*(moved - target).T)


def _best_rotation(src, dst):
    """Rotation R and shift t that best map src onto dst (least squares, Kabsch)."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    h = (src - cs).T @ (dst - cd)
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    return r, cd - r @ cs


def match(before, after, p=None, guess=(0.0, 0.0, 0.0), iterations=60):
    """(dx, dy, dyaw, agree): the motion, and the share of points that agree within AGREE.

    guess is a starting motion; ICP only finds the nearest fit, so a big turn needs one.
    dyaw is returned within 180 deg of the guessed yaw, not wrapped to +-180 deg.
    Raises ValueError when a scan has fewer than 20 usable points or they do not overlap.
    """
    p = p or params.load()
    fixed, moving = points(before, p), points(after, p)
    if len(fixed) < 20 or len(moving) < 20:
        raise ValueError("too few usable points: %d and %d" % (len(fixed), len(moving)))
    along = tangents(fixed)
    x, y, yaw = guess
    r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    t = np.array([x, y], dtype=float)
    reject = 0.5  # m, pairs farther apart are ignored; the band narrows as the fit improves
    for _ in range(iterations):
        moved = moving @ r.T + t
        target, dist = _closest(moved, fixed, along)
        keep = dist < reject
        if keep.sum() < 10:
            raise ValueError("the scans do not overlap")
        step_r, step_t = _best_rotation(moved[keep], target[keep])
        r, t = step_r @ r, step_r @ t + step_t
        small = np.hypot(*step_t) < 1e-5 and abs(math.atan2(step_r[1, 0], step_r[0, 0])) < 1e-5
        if small and reject <= 0.1:
            break
        reject = max(0.1, reject * 0.7)
    _, dist = _closest(moving @ r.T + t, fixed, along)
    turned = math.atan2(r[1, 0], r[0, 0])
    turned = yaw + math.atan2(math.sin(turned - yaw), math.cos(turned - yaw))  # near the guess, so 190 deg stays 190
    return float(t[0]), float(t[1]), turned, float((dist < AGREE).mean())


def scan_shift(before, after, p=None, guess=(0.0, 0.0, 0.0)):
    """(dx, dy, dyaw) of the base from scan `before` to scan `after`; see match()."""
    return match(before, after, p, guess)[:3]
