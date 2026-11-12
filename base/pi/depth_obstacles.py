"""Obstacles from the depth image: base-frame points that stick up above the floor."""
import math

import numpy as np

MIN_HEIGHT = 0.05  # m above the floor: lower than this is floor, carpet edge or noise
MAX_RANGE = 3.0  # m, depth is too noisy beyond this
CELL = 0.05  # m, one point per cell
MIN_VALID_SHARE = 0.05  # below this the image is blind (sunlight), not empty
NEEDS = ("camera_x", "camera_y", "camera_height", "camera_pitch_deg", "robot_height", "camera_hfov_deg")


def points(depth, p):
    """(N, 2) base-frame x, y of obstacle pixels, or None when the depth image is blind.

    depth is uint16 millimetres, 0 = no reading. None means "no depth"; an empty array
    means "looked, nothing there".
    """
    missing = [k for k in NEEDS if p.get(k) is None]
    if missing:
        raise ValueError("camera pose not measured yet: %s" % ", ".join(missing))
    valid = depth > 0
    # Any blind third (a window, sunlight on one side) makes the whole image "no depth":
    # half a frame blinded must not read as an empty room.
    if any(third.mean() < MIN_VALID_SHARE for third in np.array_split(valid, 3, axis=1)):
        return None

    # Pinhole rays, same convention as tools/sensor_check.pixel_rays (reimplemented: that
    # file lives in tools/ and pulls in camera and lidar on import).
    height, width = depth.shape
    f = (width / 2) / math.tan(math.radians(p["camera_hfov_deg"]) / 2)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    z = depth[valid] / 1000.0
    x_cam = (u[valid] - (width - 1) / 2) / f * z  # right
    y_cam = (v[valid] - (height - 1) / 2) / f * z  # down

    # Camera tilted UP by pitch: forward = (cos, sin), down = (sin, -cos) in (x ahead, up).
    pitch = math.radians(p["camera_pitch_deg"])
    ahead = p["camera_x"] + z * math.cos(pitch) + y_cam * math.sin(pitch)
    left = p["camera_y"] + x_cam  # image is mirrored: image right is the robot's left (hardware assessment 2026-11-13)
    up = p["camera_height"] + z * math.sin(pitch) - y_cam * math.cos(pitch)

    keep = (up > MIN_HEIGHT) & (up < p["robot_height"]) & (z < MAX_RANGE)
    xy = np.column_stack([ahead[keep], left[keep]])
    if len(xy) == 0:
        return xy
    cells = np.floor(xy / CELL).astype(np.int64)
    _, first = np.unique(cells, axis=0, return_index=True)
    return xy[np.sort(first)]
