"""A black cable lying on the floor, from the colour image: base-frame floor points."""
import functools
import math

import numpy as np

from depth_obstacles import _rays

MAX_RANGE = 1.2  # m ahead: farther rows show furniture bottoms, not floor
DARK_RATIO = 0.5  # a cable pixel is under half the floor brightness of its row
DARK_MAX = 80  # and black in absolute terms (grout lines are about 100)
CHROMA_MAX = 40  # and grey, not blue
MIN_LENGTH = 20  # px, bounding-box diagonal of a cable piece
CABLE_RADIUS = 0.0035  # m: rays are met with the cable's axis height, the floor plane overshoots by ~2 cm
MAX_THICKNESS = 6  # px, a 7 mm cable is 2-4 px at 0.6-1.2 m; furniture reflections are wider
SEEN = (15, 250)  # value range of a pixel that shows something
MIN_SEEN_SHARE = 0.6


def _rgb(frame_or_rgb):
    if hasattr(frame_or_rgb, "rgb"):
        return np.frombuffer(frame_or_rgb.rgb, np.uint8).reshape(frame_or_rgb.height, frame_or_rgb.width, 3)
    return np.asarray(frame_or_rgb)


@functools.lru_cache(maxsize=8)
def _floor(shape, hfov_deg, pitch_deg, height, cam_x, cam_y):
    """(first floor row, ahead, left) for the rows within MAX_RANGE: each pixel's ray met with the floor."""
    right, down = _rays(shape, hfov_deg)
    pitch = math.radians(pitch_deg)
    dz = math.sin(pitch) - down[:, 0] * math.cos(pitch)  # rays tilt down below the horizon row
    with np.errstate(divide="ignore"):
        t = np.where(dz < 0, height / -dz, np.inf)
    ahead = cam_x + t * (math.cos(pitch) + down[:, 0] * math.sin(pitch))
    top = int(np.argmax(ahead <= MAX_RANGE)) if (ahead <= MAX_RANGE).any() else shape[0]
    t = t[top:, None]
    return top, np.broadcast_to(ahead[top:, None], (shape[0] - top, shape[1])), cam_y + t * right[top:]  # image right is the robot's left


def _band(frame_or_rgb, p, height):
    rgb = _rgb(frame_or_rgb)
    h = p["camera_height"] if height is None else height
    top, ahead, left = _floor(rgb.shape[:2], p["camera_hfov_deg"], p["camera_pitch_deg"], h - CABLE_RADIUS,
                              p["camera_x"], p["camera_y"])
    return rgb[top:], ahead, left


def floor_visible(frame_or_rgb, p, height=None):
    """False when the floor rows are too dark or blown out to judge (the camera is blind)."""
    band = _band(frame_or_rgb, p, height)[0]
    if band.size == 0:
        return False
    value = band.max(axis=2)
    return ((value > SEEN[0]) & (value < SEEN[1])).mean() >= MIN_SEEN_SHARE


def _find(parent, i):
    while parent[i] != i:
        parent[i] = parent[parent[i]]
        i = parent[i]
    return i


def _label(mask):
    """8-connected component ids (0 = background), from row runs and union-find: cheap for sparse masks."""
    labels = np.zeros(mask.shape, np.int32)
    parent = [0]
    prev = []
    for r, row in enumerate(mask):
        edges = np.flatnonzero(np.diff(np.concatenate(([0], row.view(np.int8), [0]))))
        cur = []
        for s, e in zip(edges[::2], edges[1::2]):
            lab = None
            for ps, pe, pl in prev:
                if ps <= e and s <= pe:  # runs [s, e) and [ps, pe) touch, diagonals included
                    root = _find(parent, pl)
                    if lab is None:
                        lab = root
                    elif root != lab:
                        parent[root] = lab
            if lab is None:
                lab = len(parent)
                parent.append(lab)
            cur.append((s, e, lab))
            labels[r, s:e] = lab
        prev = cur
    return np.array([_find(parent, i) for i in range(len(parent))])[labels]


def cable_points(frame_or_rgb, p, height=None):
    """(N, 2) base-frame x, y on the floor of black thin curved pixels; height overrides camera_height."""
    band, ahead, left = _band(frame_or_rgb, p, height)
    if band.size == 0:
        return np.empty((0, 2))
    value = band.max(axis=2).astype(np.int16)
    chroma = value - band.min(axis=2)
    floor = np.median(value, axis=1, keepdims=True)  # the cable is thin: the row median is the floor
    dark = (value < DARK_RATIO * floor) & (value < DARK_MAX) & (chroma < CHROMA_MAX)
    if not dark.any():
        return np.empty((0, 2))

    labels = _label(dark)
    padded = np.pad(dark, 1, mode="edge")
    inner = padded[:-2, 1:-1] & padded[2:, 1:-1] & padded[1:-1, :-2] & padded[1:-1, 2:]
    ids = labels[dark]
    rows, cols = np.nonzero(dark)
    n = np.bincount(ids)
    edge = np.bincount(ids, weights=~inner[dark], minlength=len(n))
    span = []
    for coord in (rows, cols):
        lo = np.full(len(n), 1 << 30)
        hi = np.full(len(n), -1)
        np.minimum.at(lo, ids, coord)
        np.maximum.at(hi, ids, coord)
        span.append(hi - lo + 1)
    with np.errstate(divide="ignore", invalid="ignore"):
        thickness = 2 * n / edge  # area over half the outline: the width of a thin band
    cable = (np.hypot(*span) >= MIN_LENGTH) & (thickness <= MAX_THICKNESS)
    keep = cable[labels] & dark
    return np.column_stack([ahead[keep], left[keep]])
