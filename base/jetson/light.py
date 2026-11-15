"""Venue lighting: judge each camera frame's light, even it out before the networks, and the lighting changes
tools/light_bench.py applies to real frames to measure what spotlights, dimmed or flickering lights do.

FrameGate says when a frame must not be trusted: too dark, too much of it saturated, or a flash (its brightness
far from the frames just before). The Pi then treats that answer as no answer for identity: no look learned
or compared, no re-lock, no enrol, and no "the camera saw nobody there" evidence; the LiDAR track goes on.

normalise() evens out the light. Which mode the server uses, and the gate's numbers, come from light_bench on
real frames (see NORMALISE and the gate constants for the measurements).
"""

import math

import cv2
import numpy as np

# Gate. Numbers from tools/light_bench.py on 2026-11-15, on the PC: two real 320 x 240 robot frames with a person
# (scratch depth30.npz, Xtion colour plus 30 depth frames; floor1.jpg), yolo11n-pose_256x320.onnx as the person
# detector (the Jetson's yolo11n engine not reachable from the PC), a colour histogram standing in for the OSNet look.
# Two frames are few: rerun on the Jetson with the engines and more frames before trusting the numbers further.
# Dark: at 0.2 of the light (frame means 15.1 and 17.1) both people were found (recall 1.0) and the stand-in look
# stayed >= 0.70; at 0.1 (means 7.7 and 8.8) still found, but the look fell to 0.53. Refused below 10.
DARK_MEAN = 10.0  # mean luma (0-255) below this: too dark to trust
SATURATED = 250  # luma at or above this is clipped
# Saturated: overexposed x3 (31 % and 27 % clipped; one frame already has 15 % clipped from a bright floor and
# window) kept recall 1.0 and looks >= 0.85; x6 dropped the look to 0.45. Refused above 35 %.
SATURATED_SHARE = 0.35  # more of the frame than this clipped: too bright to trust
# Flash: a light switching between full and 0.3 at 2 to 25 Hz lost nothing (recall 1.0, looks >= 0.80 on every
# frame), so those frames must not be refused (a ratio of 1.8 refused 32 to 50 % of them for nothing); 4 only
# catches jumps the dark and saturated rules miss.
FLASH_RATIO = 4.0  # mean luma this many times above or below the median of the last frames: a flash
FLASH_HISTORY = 6  # frames in that median (about 0.2 s at 29 answers/s)
# Light rule for looks (applied on the Pi, senses.LIGHT_RATIO): a spotlight on the person put their box at 2.4 to
# 3.4 times its light and the look at 0.33 and 0.35; of the 9 changes that put a look under 0.65, a ratio of 2
# kept 6 out.
LIGHT_RATIO = 2.0  # a look is compared only when the box is lit within this ratio of when the operator was learned
# Normalise: every lighting change but a black frame kept recall 1.0 without it; CLAHE lowered the IoU on the
# unaltered frames to 0.967 and the stand-in look at 0.3 of the light from 0.85 to 0.66; grayworld lifted the
# look under a warm or cold cast (0.72 to 0.92) but dropped it under a spotlight (0.34 to 0.17). None until the
# Jetson bench with OSNet says otherwise (server.py --normalise to try one).
NORMALISE = "none"  # the mode the server applies before the detector and the look network


def luma(bgr):
    """Rec. 601 luma (OpenCV's gray), uint8, same height and width."""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


def box_light(bgr, box):
    """Mean luma inside a person's box [x1, y1, x2, y2] (clipped to the frame; never an empty box)."""
    h, w = bgr.shape[:2]
    x1, y1 = min(w - 1, max(0, int(box[0]))), min(h - 1, max(0, int(box[1])))
    x2, y2 = min(w, max(x1 + 1, int(round(box[2])))), min(h, max(y1 + 1, int(round(box[3]))))
    return float(luma(bgr[y1:y2, x1:x2]).mean())


def light_ok(light, reference, ratio=LIGHT_RATIO):
    """Was this look taken in about the light of the reference (both box mean luma)? Unknown counts as yes."""
    if light is None or reference is None or reference <= 0:
        return True
    return 1 / ratio <= max(light, 1e-6) / reference <= ratio


def light_of(bgr):
    """(mean luma, share of pixels clipped at SATURATED) of a BGR frame."""
    y = luma(bgr)
    return float(y.mean()), float(np.count_nonzero(y >= SATURATED)) / y.size


class FrameGate:
    """Judges a stream of frames, one at a time: check(bgr) -> (ok, why) with why "dark", "saturated" or "flash".

    A flash compares the frame's mean luma with the median of the last FLASH_HISTORY frames (bad ones too, so a
    lasting change of the lights becomes the new normal after about half the history instead of being refused
    for ever). A level the gate already refuses as dark or saturated is never a flash, it already says why.
    """

    def __init__(self, dark=DARK_MEAN, saturated_share=SATURATED_SHARE, flash=FLASH_RATIO, history=FLASH_HISTORY):
        self.dark, self.saturated_share, self.flash, self.history = dark, saturated_share, flash, history
        self._means = []
        self.mean = None  # mean luma of the last frame checked

    def check(self, bgr):
        mean, clipped = light_of(bgr)
        self.mean = mean
        before, self._means = self._means, (self._means + [mean])[-self.history:]
        if mean < self.dark:
            return False, "dark"
        if clipped > self.saturated_share:
            return False, "saturated"
        if len(before) >= self.history // 2 and self.flash > 0:
            ref = float(np.median(before))
            if ref > 0 and not 1 / self.flash <= mean / ref <= self.flash:
                return False, "flash"
        return True, None


def normalise(bgr, mode):
    """Even out the light: "none", "clahe" (contrast-limited equalisation of luma only, colours kept),
    "grayworld" (each colour channel scaled to the same mean: removes a warm or cold cast) or "clahe+grayworld"."""
    if mode == "none":
        return bgr
    out = bgr
    if "grayworld" in mode:
        means = out.reshape(-1, 3).mean(axis=0)
        gains = means.mean() / np.maximum(means, 1.0)
        out = np.clip(out.astype(np.float32) * gains, 0, 255).astype(np.uint8)
    if "clahe" in mode:
        ycc = cv2.cvtColor(out, cv2.COLOR_BGR2YCrCb)
        ycc[:, :, 0] = _CLAHE.apply(ycc[:, :, 0])
        out = cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR)
    if out is bgr:
        raise ValueError("unknown normalise mode %r" % mode)
    return out


_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))  # 4 x 4 tiles: 80 x 60 pixels on a 320 x 240 frame
MODES = ("none", "clahe", "grayworld", "clahe+grayworld")


# ---- Lighting changes for the bench. Every one takes and returns a BGR uint8 frame. ----

def _clip(x):
    return np.clip(np.rint(x), 0, 255).astype(np.uint8)


def dim(bgr, gain, noise=3.0, seed=0):
    """Less light, the camera's exposure at its limit: darker, plus sensor noise (sigma in 0-255 levels)."""
    rng = np.random.default_rng(seed)
    return _clip(bgr.astype(np.float32) * gain + rng.normal(0.0, noise, bgr.shape))


def dim_auto(bgr, gain, noise=3.0, seed=0):
    """Less light with the camera's auto exposure making up for it: normal brightness, noise amplified 1 / gain."""
    rng = np.random.default_rng(seed)
    return _clip((bgr.astype(np.float32) * gain + rng.normal(0.0, noise, bgr.shape)) / gain)


def gamma(bgr, g):
    return _clip(255.0 * (bgr.astype(np.float32) / 255.0) ** g)


def overexpose(bgr, gain):
    return _clip(bgr.astype(np.float32) * gain)


def spotlight(bgr, centre, radius, peak=220.0):
    """A spotlight: a radial bright spot (Gaussian, sigma = radius / 2) adding up to `peak` levels, saturating
    where it is strongest."""
    h, w = bgr.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    blob = np.exp(-((xx - centre[0]) ** 2 + (yy - centre[1]) ** 2) / (2 * (radius / 2) ** 2))
    return _clip(bgr.astype(np.float32) + peak * blob[:, :, None])


def shadow(bgr, gain=0.3, angle_deg=70.0):
    """A hard shadow: everything on one side of a straight line through the middle at `gain`."""
    h, w = bgr.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    a = math.radians(angle_deg)
    side = (xx - w / 2) * math.sin(a) - (yy - h / 2) * math.cos(a) > 0
    out = bgr.astype(np.float32)
    out[side] *= gain
    return _clip(out)


def cast(bgr, blue, green, red):
    """A colour cast: each channel times its gain (warm: red up, blue down; cold: the other way)."""
    return _clip(bgr.astype(np.float32) * np.array([blue, green, red], np.float32))


def bands(bgr, depth=0.5, period_rows=40.0, phase=0.0):
    """Rolling-shutter flicker: rows read at different moments of a flickering light, horizontal bands."""
    rows = np.arange(bgr.shape[0], dtype=np.float32)
    g = 1.0 - depth * (0.5 + 0.5 * np.sin(2 * math.pi * rows / period_rows + phase))
    return _clip(bgr.astype(np.float32) * g[:, None, None])


def flicker_gains(hz, fps, count, low=0.3):
    """Frame gains of a light switching between full and `low` at `hz` (square wave), sampled at `fps`."""
    return [1.0 if math.sin(2 * math.pi * hz * (i / fps) + 0.1) >= 0 else low for i in range(count)]


# ---- Depth ----

def person_patch(box, shape):
    """The rows and columns senses.person_range reads in a person's box: the middle half of its width, from half
    to nine tenths of its height. ((r0, r1), (c0, c1)), or None when empty."""
    height, width = shape
    x1, y1, x2, y2 = box
    cols = int(max(0, math.floor(x1 + (x2 - x1) / 4))), int(min(width, math.ceil(x2 - (x2 - x1) / 4)))
    rows = int(max(0, math.floor(y1 + (y2 - y1) / 2))), int(min(height, math.ceil(y1 + (y2 - y1) * 0.9)))
    if cols[1] <= cols[0] or rows[1] <= rows[0]:
        return None
    return rows, cols


def valid_share(depth, box):
    """Share of the person_patch pixels with a depth reading (0 = no reading), or 0.0 for an empty patch."""
    patch = person_patch(box, depth.shape)
    if patch is None:
        return 0.0
    (r0, r1), (c0, c1) = patch
    return float(np.count_nonzero(depth[r0:r1, c0:c1])) / ((r1 - r0) * (c1 - c0))


def wash_depth(depth, centre, radius, dropout=0.0, seed=0):
    """Depth under a strong infrared light (halogen spotlight, sunlight): the Xtion's projected pattern drowns
    and readings go to 0 (holes), in a disc of `radius` pixels at `centre` and at random over `dropout` of
    the rest. Values that stay are unchanged: washout makes holes, not wrong numbers."""
    h, w = depth.shape
    yy, xx = np.mgrid[0:h, 0:w]
    out = depth.copy()
    out[(xx - centre[0]) ** 2 + (yy - centre[1]) ** 2 <= radius ** 2] = 0
    if dropout > 0:
        out[np.random.default_rng(seed).random((h, w)) < dropout] = 0
    return out
