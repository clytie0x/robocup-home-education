"""Every bag in one front camera frame, with its colour, its side and where it stands: so the operator can say
which one ("the blue one", "the one on the left") when the bag and the operator are both close to the robot.

  list_bags(image, cam, bv, bc)  every bag their bag-trained YOLO11-seg (models/bag*-seg*.engine) and the COCO one
                                 (their bag_vision segmenters, as find_bag uses them) see, each with
                                 {"colour", "x", "y", "width", "ok", "why", "box", "side"}; x, y in bag_arm.py's
                                 robot frame (origin under the LiDAR), from their locate() as tools/pick_bag_cam.py
                                 does it. ok: about a bag's width and inside the arm's reach. side: "left", "middle"
                                 or "right" among the ok bags (None when there is only one).
  describe(bags)                 "a blue bag on the left and a red bag on the right"
  choose(bags, colour, side)     (index, why): the one ok bag that fits the words, or None and why not
  look(snap_url, camera_file)    one frame from webcam_view (8082) -> {"ok", "bags", "why"} for bag_arm's look_bags

  python3 bag_choice.py [--image f.jpg]   print the bags in the camera now (nothing moves)
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "tools"))

import pick_bag_cam as pbc  # noqa: E402 (arm_modules, robot_xy and the same limits as the one-bag pick)

COLOURS = ("red", "orange", "yellow", "green", "blue", "purple", "pink", "brown", "white", "grey", "black")
# words people say -> the colour name above
COLOUR_WORDS = {"navy": "blue", "dark blue": "blue", "light blue": "blue", "sky blue": "blue", "violet": "purple",
                "magenta": "pink", "gray": "grey", "silver": "grey", "gold": "yellow", "beige": "brown",
                "tan": "brown", "cream": "white"}
SIDES = ("left", "middle", "right")
SIDE_WORDS = {"centre": "middle", "center": "middle"}
SAME_BAG_IOU = 0.5  # two masks overlapping this much are one bag
MIN_SCORE = 0.4  # a listed bag's segmenter score (2026-11-19, new camera: real bags 0.63-0.83, the floor's best 0.33)


def colour_name(bgr, mask):
    """The bag's colour from its own pixels: the most common hue of its coloured pixels, or white / grey / black
    when most of it has no colour (logos and print are a minority, so the hue mode is the bag's)."""
    import cv2  # noqa: PLC0415
    m = mask.astype(bool)
    inner = cv2.erode(m.astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)
    if inner.sum() > 200:  # off the edges, where floor and shadow mix in
        m = inner
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[m]
    if not len(hsv):
        return "unknown"
    h, s, v = hsv[:, 0].astype(int), hsv[:, 1].astype(int), hsv[:, 2].astype(int)
    coloured = (s >= 60) & (v >= 50)
    if coloured.mean() < 0.35:
        mv = float(np.median(v))
        return "white" if mv >= 150 else "black" if mv < 60 else "grey"
    hue = int(np.bincount(h[coloured] // 5, minlength=36).argmax()) * 5 + 2  # 5-degree bins (OpenCV hue 0-179)
    sat, val = float(np.median(s[coloured])), float(np.median(v[coloured]))
    if hue < 8 or hue >= 170:
        return "pink" if sat < 130 and val > 170 else "brown" if val < 110 else "red"
    if hue < 20:
        return "brown" if val < 130 else "orange"
    if hue < 34:
        return "yellow"
    if hue < 85:
        return "green"
    if hue < 130:
        return "blue"
    if hue < 150:
        return "purple"
    return "pink"


def _iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return inter / union if union else 0.0


def tuned_engine(bv):
    """The arm team's bag-trained segmenter (class 0 bag, 1 handle): $SO100_BAG_TUNED_ENGINE, their default, else
    the newest models/bag*-seg*.engine. None when there is none. On 2026-11-19 it found both paper bags in the new
    camera's frame where the COCO model found none."""
    import glob  # noqa: PLC0415
    for path in (os.environ.get("SO100_BAG_TUNED_ENGINE"), getattr(bv, "DEFAULT_TUNED_ENGINE", None)):
        if path and os.path.isfile(os.path.expanduser(path)):
            return os.path.expanduser(path)
    found = sorted(glob.glob(os.path.join(HERE, "models", "bag*-seg*.engine")), key=os.path.getmtime)
    return found[-1] if found else None


def _segmenters(bv):
    out = []
    tuned = tuned_engine(bv)
    if tuned:
        out.append(bv._get_segmenter("tuned", tuned))
    out.append(bv._get_segmenter("coco", None))
    return [s for s in out if s is not None]


def _masks(image, bv, segmenters):
    """Bag masks, one per bag: the tuned model's first, then the COCO model's that are not the same bag."""
    kept = []
    for seg in segmenters:
        try:
            bags, _handles = bv._instances(seg.segment(image))
        except Exception:  # noqa: BLE001 (a bad frame is no bag, not a crash of bag_arm)
            continue
        for score, _box, mask in bags:
            if float(score) < MIN_SCORE:
                continue
            m = mask > 0
            if m.sum() and not any(_iou(m, k) >= SAME_BAG_IOU for k in kept):
                kept.append(m)
    return kept


def _sides(bags):
    """left / middle / right among the ok bags, by y in the robot frame (left is +y). One ok bag: no side."""
    ok = sorted((b for b in bags if b["ok"]), key=lambda b: -b["y"])
    for b in bags:
        b["side"] = None
    if len(ok) < 2:
        return
    for i, b in enumerate(ok):
        b["side"] = "left" if i == 0 else "right" if i == len(ok) - 1 else "middle"


def list_bags(image, cam, bv, bc, segmenters=None, pan=pbc.ARM_PAN, half_depth=pbc.HALF_DEPTH, rim_h=pbc.RIM_H,
              laser_x=pbc.LASER_X, width=pbc.BAG_WIDTH, width_tol=pbc.WIDTH_TOL, reach=pbc.REACH_X):
    """Every bag in the frame, left to right in the robot (ok ones first), see the module note."""
    orange = bv.orange_mask(image)
    out = []
    segs = _segmenters(bv) if segmenters is None else segmenters
    if segs:  # nothing found is no bag: their floor-difference fallback took the empty floor for a white bag
        blobs = [(bv._blob_from_mask(m & (orange == 0), orange, "yolo-seg"), m) for m in _masks(image, bv, segs)]
    else:  # no segmenter at all: the arm team's own single-bag fallbacks (floor difference, pink/red)
        b = bv.find_bag(image)
        blobs = [(b, None)] if b is not None else []
    cam_pan = (pan[0] + laser_x, pan[1])
    for blob, m in blobs:
        if blob is None:
            continue
        if m is None:
            m = np.zeros(image.shape[:2], bool)
            m[int(blob["y0"]):int(blob["y1"]) + 1, int(blob["x0"]):int(blob["x1"]) + 1] = True
        bag = {"colour": colour_name(image, m), "box": [int(blob["x0"]), int(blob["y0"]), int(blob["x1"]),
                                                          int(blob["y1"])], "ok": False, "why": "", "x": None,
               "y": None, "width": None}
        where = bc.locate(cam, blob, cam_pan, half_depth, rim_h, foot=bv.foot_y(image, blob))
        if where is None:
            bag["why"] = "its rays do not reach the floor"
            out.append(bag)
            continue
        x, y = pbc.robot_xy(where["centre"], pan)
        w = where.get("width")
        bag.update(x=round(x, 3), y=round(y, 3), width=None if w is None else round(float(w), 3))
        if w is None or abs(w - width) > width_tol:
            bag["why"] = "%s m wide, not a %.2f m bag" % ("?" if w is None else "%.2f" % w, width)
        elif not reach[0] <= x <= reach[1]:
            bag["why"] = "%.2f m ahead, outside %.2f to %.2f m" % (x, *reach)
        else:
            bag["ok"] = True
        out.append(bag)
    _sides(out)
    out.sort(key=lambda b: (not b["ok"], -(b["y"] if b["y"] is not None else 0.0)))
    return out


def normal_colour(word):
    w = (word or "").strip().lower()
    w = COLOUR_WORDS.get(w, w)
    return w if w in COLOURS else None


def normal_side(word):
    w = (word or "").strip().lower()
    w = SIDE_WORDS.get(w, w)
    return w if w in SIDES else None


def describe(bags):
    """The ok bags in words, left to right: 'a blue bag on the left and a red bag on the right'."""
    ok = [b for b in bags if b["ok"]]
    parts = ["a %s bag%s" % (b["colour"], " on the %s" % b["side"] if b["side"] == "left" or b["side"] == "right"
                             else " in the middle" if b["side"] == "middle" else "") for b in ok]
    if len(parts) <= 1:
        return "".join(parts)
    return ", ".join(parts[:-1]) + " and " + parts[-1]


# colours a person and the camera may name differently (a violet-blue bag, a pink-red one): matched only when no bag
# has the very colour that was said
FAMILY = {"purple": "blue", "pink": "red", "orange": "red", "grey": "white"}


def _family(colour):
    return FAMILY.get(colour, colour)


def choose(bags, colour=None, side=None):
    """(index into bags, why). The one ok bag with that colour and/or side; one ok bag and no words: that one.
    why is "which one" (several, no words), "no blue bag", "more than one blue bag" or "no bag within reach"."""
    colour, side = normal_colour(colour), normal_side(side)
    ok = [i for i, b in enumerate(bags) if b["ok"]]
    if not ok:
        return None, "no bag within reach"
    if colour is None and side is None:
        return (ok[0], "the only bag") if len(ok) == 1 else (None, "which one")
    what = " ".join(w for w in (colour, "bag", "on the %s" % side if side else None) if w)
    on_side = [i for i in ok if side is None or bags[i]["side"] == side]
    fit = [i for i in on_side if colour is None or bags[i]["colour"] == colour]
    if not fit and colour is not None:
        fit = [i for i in on_side if _family(bags[i]["colour"]) == _family(colour)]
    if len(fit) == 1:
        return fit[0], "the " + what
    return None, ("no " + what) if not fit else ("more than one " + what)


def look(snap_url=pbc.SNAP_URL, camera_file=pbc.CAMERA_FILE):
    """One frame -> {"ok": True, "bags": [...]} or {"ok": False, "why": ...} (JSON-safe, for /robot/event)."""
    try:
        bv, bc = pbc.arm_modules()
        cam = bc.FrontCamera.load(camera_file)
        image = pbc.fetch(snap_url)
        return {"ok": True, "bags": list_bags(image, cam, bv, bc)}
    except Exception as exc:  # noqa: BLE001 (any failure is "could not look", the flow falls back to pointing)
        return {"ok": False, "why": str(exc)[:200], "bags": []}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--image", help="a saved frame instead of the camera")
    ap.add_argument("--snap", default=pbc.SNAP_URL)
    ap.add_argument("--camera", default=pbc.CAMERA_FILE)
    a = ap.parse_args(argv)
    if a.image:
        import cv2  # noqa: PLC0415
        bv, bc = pbc.arm_modules()
        res = {"ok": True, "bags": list_bags(cv2.imread(a.image), bc.FrontCamera.load(a.camera), bv, bc)}
    else:
        res = look(a.snap, a.camera)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    if res["ok"]:
        print("I see " + (describe(res["bags"]) or "no bag within reach") + ".")
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
