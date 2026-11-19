"""Image helpers for the camera bag pick (bag_camera.py): find the bag in the webcam view (fixed front camera
since 2026-11-18; the forearm camera before), read the direction of its top ridge, and check whether the bag is
off the floor (fixed camera: fixed_lifted; Xtion: lifted)."""
import math

import cv2
import numpy as np


def masks(im):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    orange = ((H >= 3) & (H <= 22) & (S > 110) & (V > 150)).astype(np.uint8)
    orange = cv2.dilate(orange, np.ones((9, 9), np.uint8))
    bag = (((H >= 135) | (H <= 2)) & (S > 50) & (V > 60)).astype(np.uint8)
    bag[orange > 0] = 0
    bag = cv2.morphologyEx(bag, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    bag = cv2.morphologyEx(bag, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return bag, orange


def gripper_x(orange):
    cols = np.where(orange[:orange.shape[0] * 2 // 3].any(0))[0]
    return float(np.median(cols)) if len(cols) else orange.shape[1] / 2


def find_bag(im):
    """Bag blob: dict(cx, x0, x1, tops (columns the gripper does not hide), all_tops, y0, y1, h, area, grip_x)
    or None."""
    bag, orange = masks(im)
    n, lab, st, _ = cv2.connectedComponentsWithStats(bag)
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, 4]))
    if st[i, 4] < 1500:
        return None
    m = lab == i
    cols = np.where(m.any(0))[0]
    rows = np.where(m.any(1))[0]
    tops = np.array([[c, np.nonzero(m[:, c])[0].min()] for c in cols], float)
    all_tops = tops.copy()        # every column (the fixed camera with the arm out of view: bag_camera.locate)
    # drop columns where the top is hidden by the gripper (orange right above it)
    keep = [not orange[max(0, int(y) - 12):int(y) + 1, int(x)].any() for x, y in tops]
    tops = tops[np.array(keep, bool)] if any(keep) else tops
    return {"cx": float(np.mean(cols)), "x0": float(cols.min()), "x1": float(cols.max()), "tops": tops,
            "all_tops": all_tops,
            "y0": float(rows.min()), "y1": float(rows.max()), "h": int(m.shape[0]),
            "area": int(st[i, 4]), "grip_x": gripper_x(orange)}


def ridge(b):
    """Classify the top edge: ('across', 0) broad side facing, ('away', 90) narrow side facing (peak),
    ('diag', +-45). Also returns the fitted image slope (deg, + = edge rising to the right)."""
    t = b["tops"]
    if len(t) < 20:
        return "across", 0, 0.0
    x, y = t[:, 0], t[:, 1]
    k = int(np.argmin(y))
    w = x.max() - x.min()
    left, right = t[x < x[k] - 15], t[x > x[k] + 15]
    # peak: highest point well inside, both sides fall away from it
    if (0.3 * w < x[k] - x.min() < 0.7 * w and len(left) > 5 and len(right) > 5):
        sl = np.polyfit(left[:, 0], left[:, 1], 1)[0]
        sr = np.polyfit(right[:, 0], right[:, 1], 1)[0]
        if sl < -0.35 and sr > 0.35:
            return "away", 90, 0.0
    s = np.polyfit(x, y, 1)[0]
    ang = -math.degrees(math.atan(s))           # image y grows down: + = rising to the right
    if abs(ang) < 22:
        return "across", 0, ang
    return ("diag", 45, ang) if ang > 0 else ("diag", -45, ang)


def floor_mask(xim):
    hsv = cv2.cvtColor(xim, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    f = (H >= 75) & (H <= 105) & (S > 25) & (V > 80)
    return f[int(xim.shape[0] * 0.9):]


def bag_bottom(xim):
    """Bag-hued pixels (purple / magenta / red, not skin or dark clothes) in the bottom rows of an Xtion frame."""
    hsv = cv2.cvtColor(xim, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    return (((H >= 120) | (H <= 3)) & (S > 40) & (V > 30))[int(xim.shape[0] * 0.88):]


def lifted(before, after):
    """Share of the bottom-row columns that showed the bag before the lift and show floor after it.
    > 0.5 = the bag is off the floor. Falls back to plain floor columns when the bag's bottom is not seen."""
    cols = bag_bottom(before).mean(0) > 0.5
    if cols.sum() < 8:
        return floor_lifted(before, after)
    return float(floor_mask(after)[:, cols].mean()), int(cols.sum())


def cam_lifted(b0, b1, grow=1.25, shift=30.0):
    """Arm-camera lift check: the bag seen from the same arm pose before the grasp (b0) and after lifting
    10 cm (b1). A lifted bag comes 10 cm nearer the camera: bigger and moved in the image. Returns a dict with
    lifted True / False / None (cannot tell) and the numbers, for the log."""
    if b0 is None:
        return {"lifted": None, "why": "no bag before"}
    if b1 is None:
        return {"lifted": False, "why": "no bag after"}
    ratio = b1["area"] / max(b0["area"], 1)
    c0 = ((b0["x0"] + b0["x1"]) / 2, float(b0["tops"][:, 1].min()))
    c1 = ((b1["x0"] + b1["x1"]) / 2, float(b1["tops"][:, 1].min()))
    move = float(np.hypot(c1[0] - c0[0], c1[1] - c0[1]))
    return {"lifted": bool(ratio > grow or move > shift), "area_ratio": round(ratio, 2), "move_px": round(move, 1)}


def fixed_lifted(b0, b1, min_up=12.0, min_cols=15):
    """Fixed front camera lift check: the bag before (b0) and after (b1) a lift, the arm at the same pose in both
    frames. A lifted bag moves UP in the image (a 10 cm lift is ~40 px at the pick distance, model of
    front_camera.json). Compared: the top edge over the columns seen in both, and the bottom row when it is not
    cut off by the frame. Returns dict(lifted True / False / None = cannot tell, numbers for the log)."""
    if b0 is None:
        return {"lifted": None, "why": "no bag before"}
    if b1 is None:
        return {"lifted": None, "why": "no bag after (hidden by the arm?)"}
    t0 = {int(x): y for x, y in b0["tops"]}
    t1 = {int(x): y for x, y in b1["tops"]}
    common = sorted(set(t0) & set(t1))
    out = {"cols": len(common)}
    ups = []
    if len(common) >= min_cols:
        out["top_up_px"] = round(float(np.median([t0[c] - t1[c] for c in common])), 1)
        ups.append(out["top_up_px"])
    if b0["y1"] < b0["h"] - 3 and b1["y1"] < b1["h"] - 3:      # bottom edge in view in both frames
        out["bottom_up_px"] = round(b0["y1"] - b1["y1"], 1)
        ups.append(out["bottom_up_px"])
    if not ups:
        out.update(lifted=None, why="nothing to compare")
        return out
    out["lifted"] = bool(max(ups) > min_up)
    return out


def foot_y(im, b, run=6):
    """Image row of the bag's foot on the floor in the fixed front camera: below the bag blob, the first row
    where the cyan floor shows again (the test bag's lower part is white with dots, not in the bag mask, so
    the blob's own bottom is not the foot). Median over the blob's middle columns; None when the floor below
    is out of the picture (the bag too near) in most of them."""
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    floor = (H >= 75) & (H <= 105) & (S > 25) & (V > 80)
    x0, x1 = int(b["x0"]), int(b["x1"])
    cols = range(x0 + (x1 - x0) // 4, x1 - (x1 - x0) // 4 + 1, 2)
    rows = []
    for c in cols:
        f = floor[int(b["y1"]):, c]
        hit = None
        for r in range(0, len(f) - run + 1):        # first run of floor rows (not a fleck)
            if f[r:r + run].all():
                hit = int(b["y1"]) + r
                break
        rows.append(hit)
    found = [r for r in rows if r is not None]
    if len(found) < max(3, len(rows) // 2):
        return None
    return float(np.median(found))


def edge_y(b, half=40):
    """Image row of the bag's top edge around the gripper column (lower row = nearer bag)."""
    t = b["tops"]
    near = t[np.abs(t[:, 0] - b["grip_x"]) < half]
    if len(near) < 3:
        near = t[np.argsort(np.abs(t[:, 0] - b["grip_x"]))[:10]]
    return float(np.median(near[:, 1]))


def floor_lifted(before, after):
    """Xtion frames before/after lifting: share of the columns the bag covered at the bottom that now show
    floor. Returns (share, n_cols); share > 0.5 = the bag is off the floor."""
    fb, fa = floor_mask(before), floor_mask(after)
    cols = fb.mean(0) < 0.3                    # bag (not floor) at the bottom before the lift
    if cols.sum() < 10:
        return 0.0, int(cols.sum())
    return float(fa[:, cols].mean()), int(cols.sum())


def floor_under_bag(xim):
    """Xtion frame: fraction of floor (cyan) pixels in the bottom rows. High = the bag is off the floor."""
    hsv = cv2.cvtColor(xim, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    floor = (H >= 75) & (H <= 105) & (S > 25) & (V > 80)
    h = xim.shape[0]
    return float(floor[int(h * 0.9):].mean())


def draw(im, b, label):
    o = im.copy()
    if b:
        for x, y in b["tops"][::4]:
            cv2.circle(o, (int(x), int(y)), 2, (0, 255, 0), -1)
        cv2.line(o, (int(b["cx"]), 0), (int(b["cx"]), o.shape[0]), (255, 0, 0), 1)
        cv2.line(o, (int(b["grip_x"]), 0), (int(b["grip_x"]), o.shape[0]), (0, 0, 255), 1)
    cv2.putText(o, label, (8, 24), 0, 0.6, (0, 0, 255), 2)
    return o


if __name__ == "__main__":
    import sys
    for f in sys.argv[1:]:
        im = cv2.imread(f)
        if "xtion" in f:
            print(f, "floor under bag", round(floor_under_bag(im), 2))
            continue
        b = find_bag(im)
        r = ridge(b) if b else None
        print(f, None if not b else {k: round(v, 1) for k, v in b.items() if k in ("cx", "grip_x", "area")}, r)
        cv2.imwrite(f.replace(".jpg", "_bv.jpg"), draw(im, b, str(r)))
