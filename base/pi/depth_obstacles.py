"""Obstacles from the depth image: base-frame points that stick up above the floor."""
import functools
import math

import numpy as np

import footprint

MIN_HEIGHT = 0.05  # m above the floor: lower than this is floor, carpet edge or noise
MAX_RANGE = 3.0  # m, depth is too noisy beyond this
CELL = 0.05  # m, one point per cell
MIN_CELL_PIXELS = 3  # drop 1-2 flying depth pixels in a cell (a 5 cm belt at 3 m is ~22 px)
MIN_VALID_SHARE = 0.05  # below this the image is blind (sunlight), not empty
NEEDS = ("camera_x", "camera_y", "camera_height", "camera_pitch_deg", "robot_height", "camera_hfov_deg")


@functools.lru_cache(maxsize=4)
def _rays(shape, hfov_deg):
    """Pinhole ray slopes (right, down) per pixel; the same every frame, so built once.

    Same convention as tools/sensor_check.pixel_rays (reimplemented: that file lives in tools/
    and pulls in camera and lidar on import).
    """
    height, width = shape
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    return (u - (width - 1) / 2) / f, (v - (height - 1) / 2) / f


def points(depth, p, heights=False):
    """(N, 2) base-frame x, y of obstacle pixels, or None when the depth image is blind. heights: (N, 3), the
    third column each cell's highest point above the floor (m), for the memory of what leaves the view.

    depth is uint16 millimetres, 0 = no reading. None means "no depth"; an empty array
    means "looked, nothing there".
    """
    missing = [k for k in NEEDS if p.get(k) is None]
    if missing:
        raise ValueError("camera pose not measured yet: %s" % ", ".join(missing))
    points.last = {"sensor": "depth", "count": 0, "nearest": None}
    valid = depth > 0
    # Any blind third (a window, sunlight on one side) makes the whole image "no depth":
    # half a frame blinded must not read as an empty room.
    if any(third.mean() < MIN_VALID_SHARE for third in np.array_split(valid, 3, axis=1)):
        return None

    near = valid & (depth < MAX_RANGE * 1000)  # farther pixels are dropped anyway: skip their maths
    right, down = _rays(depth.shape, p["camera_hfov_deg"])
    z = depth[near] / 1000.0
    x_cam = right[near] * z
    y_cam = down[near] * z

    # Camera tilted UP by pitch: forward = (cos, sin), down = (sin, -cos) in (x ahead, up).
    pitch = math.radians(p["camera_pitch_deg"])
    ahead = p["camera_x"] + z * math.cos(pitch) + y_cam * math.sin(pitch)
    left = p["camera_y"] + x_cam  # image is mirrored: image right is the robot's left (hardware assessment 2026-11-13)
    up = p["camera_height"] + z * math.sin(pitch) - y_cam * math.cos(pitch)

    keep = (up > MIN_HEIGHT) & (up < p["robot_height"])
    xy = np.column_stack([ahead[keep], left[keep]])
    if len(xy) == 0:
        return np.empty((0, 3)) if heights else xy
    cells = np.floor(xy / CELL).astype(np.int64)
    _, first, inverse, counts = np.unique(cells, axis=0, return_index=True, return_inverse=True, return_counts=True)
    top = np.full(len(first), -np.inf)
    np.maximum.at(top, inverse.ravel(), up[keep])  # each cell's highest pixel
    good = np.flatnonzero(counts >= MIN_CELL_PIXELS)
    good = good[np.argsort(first[good])]  # one point per cell, in pixel order as before
    xy, top = xy[first[good]], top[good]
    # The arm, cable and a carried bag are rigid in the base frame. robot_height 1.05 m lets the
    # upward-tilted Xtion turn them into an obstacle a fixed ~0.26 m away (18 Nov).
    kept = footprint.drop_self(xy, p, sensor="depth")
    points.last = dict(footprint.LAST.get("depth") or points.last)
    if not heights:
        return kept
    return np.column_stack([kept, top[~footprint.own_mask(xy, p)]]) if len(xy) else np.empty((0, 3))
