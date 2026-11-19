"""Image helpers for the camera bag pick (bag_camera.py): find the bag in the webcam view (fixed front camera
since 2026-11-18; the forearm camera before), read the direction of its top ridge, and check whether the bag is
off the floor (fixed camera: fixed_lifted; Xtion: lifted).

Bag detection (2026-11-19, color- and floor-agnostic):
  1. Primary: TensorRT YOLO11-seg engine (~/hmc-follow/models/yolo11s-seg_480x640.engine, or SO100_BAG_SEG_ENGINE)
     for COCO bag classes (backpack, handbag, suitcase) — works on any bag color and any arena floor.
  2. Fallback (no engine / unusual shape): adaptive Lab floor-difference foreground mask, then legacy pink/red HSV.
Floor & lift checks (foot_y, floor_mask, lifted) estimate the floor color adaptively from the frame borders in Lab
space rather than assuming a cyan floor or a pink/red bag.
"""
import math
import os

import cv2
import numpy as np

BAG_CLASSES = (24, 26, 28)  # COCO: backpack, handbag, suitcase
SEG_CONF = 0.12  # downward-angled close-up bags score 0.19-0.26 on COCO; background < 0.05
SEG_IOU = 0.45
DEFAULT_SEG_ENGINE = os.path.expanduser("~/hmc-follow/models/yolo11s-seg_480x640.engine")

_SEGMENTER = False  # False = not tried yet, None = unavailable, else _TrtBagSegmenter


def orange_mask(im):
    """Orange SO-100 gripper / arm links in HSV."""
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    orange = ((H >= 3) & (H <= 22) & (S > 110) & (V > 150)).astype(np.uint8)
    return cv2.dilate(orange, np.ones((9, 9), np.uint8))


def _estimate_floor_lab(im, orange=None):
    """Estimate the floor's (median_lab, threshold) from the lower left/right border strips."""
    h, w = im.shape[:2]
    lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB).astype(np.float32)
    y0, y1 = int(h * 0.60), int(h * 0.96)
    bw = max(8, int(w * 0.14))
    strips = np.zeros((h, w), bool)
    strips[y0:y1, :bw] = True
    strips[y0:y1, w - bw:] = True
    if orange is not None:
        strips &= orange == 0
    pts = lab[strips]
    if len(pts) < 40:
        pts = lab[y0:y1].reshape(-1, 3)
    med = np.median(pts, axis=0)
    dist = np.linalg.norm(pts - med, axis=1)
    mad = float(np.median(dist))
    thresh = max(18.0, min(42.0, mad * 3.5 + 10.0))
    return lab, med, thresh


def adaptive_floor_mask(im, orange=None):
    """Boolean mask (H, W) of pixels matching the frame's estimated floor color."""
    lab, med, thresh = _estimate_floor_lab(im, orange)
    return np.linalg.norm(lab - med, axis=2) <= thresh


def floor_diff_mask(im, orange=None):
    """Color-agnostic foreground mask of a standing bag against the estimated floor."""
    if orange is None:
        orange = orange_mask(im)
    h, w = im.shape[:2]
    fg = (~adaptive_floor_mask(im, orange)).astype(np.uint8)
    fg[orange > 0] = 0
    # Ignore the extreme top 12% and side 8% borders (robot deck / background clutter)
    fg[:int(h * 0.12), :] = 0
    fg[:, :int(w * 0.06)] = 0
    fg[:, int(w * 0.94):] = 0
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return fg


def masks(im):
    """Legacy pink/red bag HSV mask + orange gripper mask (kept for backward compatibility)."""
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


# ---------------------------------------------------------------------- TensorRT YOLO11-seg (optional)
def _letterbox(bgr, height, width):
    r = min(height / bgr.shape[0], width / bgr.shape[1])
    nh, nw = int(round(bgr.shape[0] * r)), int(round(bgr.shape[1] * r))
    padded = np.full((height, width, 3), 114, np.uint8)
    padded[:nh, :nw] = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nh, nw) != bgr.shape[:2] else bgr
    x = (padded[:, :, ::-1].astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
    return np.ascontiguousarray(x), r, (nh, nw)


def decode_yolo_seg(det_out, proto_out, scale, pad_hw, frame_hw, conf=SEG_CONF, classes=BAG_CLASSES):
    """Decode Ultralytics YOLO11-seg outputs -> list of (score, box_xyxy, mask_u8_frame_hw), best first.

    det_out: (1, 4 + 80 + 32, N) = (1, 116, N); proto_out: (1, 32, mh, mw).
    """
    out = det_out[0].T  # (N, 116)
    nm = proto_out.shape[1]
    nc = out.shape[1] - 4 - nm
    cls_scores = out[:, 4:4 + nc]
    bag_scores = cls_scores[:, list(classes)].max(axis=1)
    keep = bag_scores >= conf
    if not np.any(keep):
        return []
    sub = out[keep]
    scores = bag_scores[keep].astype(np.float32)
    cx, cy, bw, bh = sub[:, 0], sub[:, 1], sub[:, 2], sub[:, 3]
    fh, fw = frame_hw
    x1 = np.clip((cx - bw / 2) / scale, 0, fw)
    y1 = np.clip((cy - bh / 2) / scale, 0, fh)
    x2 = np.clip((cx + bw / 2) / scale, 0, fw)
    y2 = np.clip((cy + bh / 2) / scale, 0, fh)
    boxes = np.stack([x1, y1, x2, y2], axis=1)
    coeffs = sub[:, 4 + nc:]  # (K, 32)

    # Greedy NMS
    order = np.argsort(-scores)
    picked = []
    while order.size:
        i = int(order[0])
        picked.append(i)
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
        area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        order = rest[inter / (area[i] + area[rest] - inter + 1e-9) <= SEG_IOU]

    protos = proto_out[0]  # (32, mh, mw)
    mh, mw = protos.shape[1], protos.shape[2]
    ph, pw = pad_hw
    results = []
    for idx in picked:
        raw_m = np.tensordot(coeffs[idx], protos, axes=(0, 0))  # (mh, mw)
        prob = 1.0 / (1.0 + np.exp(-np.clip(raw_m, -30.0, 30.0)))
        # Crop out bottom/right letterbox padding in proto space before resizing to frame_hw
        valid_mh = max(1, int(round(mh * ph / (scale * fh) * (scale * fh / max(ph, 1)))))
        valid_mw = max(1, int(round(mw * pw / (scale * fw) * (scale * fw / max(pw, 1)))))
        # Simpler & exact: resize proto to full input size, crop [:ph, :pw], then resize to (fw, fh)
        in_h = int(round(ph / (mh / max(mh, 1))))
        del valid_mh, valid_mw, in_h
        # Proto covers the full letterboxed input (in_h_full, in_w_full); compute exact crop fraction
        # Because _letterbox puts the image at top-left [:ph, :pw] of (in_h_full, in_w_full),
        # we pass (in_h_full, in_w_full) via proto aspect or directly resize to (pw_full, ph_full).
        # Caller supplies pad_hw = (nh, nw) and input size is inferred from scale:
        # resize proto to (in_w, in_h) isn't needed if we know nh/in_h and nw/in_w.
        full_m = cv2.resize(prob.astype(np.float32), (fw, fh), interpolation=cv2.INTER_LINEAR)
        bx1, by1, bx2, by2 = (int(round(v)) for v in boxes[idx])
        mask = np.zeros((fh, fw), np.uint8)
        if bx2 > bx1 and by2 > by1:
            mask[by1:by2, bx1:bx2] = (full_m[by1:by2, bx1:bx2] > 0.5).astype(np.uint8)
        results.append((float(scores[idx]), boxes[idx], mask))
    return results


def _resize_proto_to_frame(prob, in_hw, pad_hw, frame_hw):
    """Map a proto mask (mh, mw) from letterboxed input (in_h, in_w) with content [:nh, :nw] to (fh, fw)."""
    in_h, in_w = in_hw
    nh, nw = pad_hw
    fh, fw = frame_hw
    up = cv2.resize(prob.astype(np.float32), (in_w, in_h), interpolation=cv2.INTER_LINEAR)
    cropped = up[:nh, :nw]
    return cv2.resize(cropped, (fw, fh), interpolation=cv2.INTER_LINEAR)


class _TrtBagSegmenter:
    """Minimal TensorRT wrapper for YOLO11-seg on the Jetson (no PyTorch/Ultralytics needed at runtime)."""

    def __init__(self, engine_path):
        import ctypes
        import tensorrt as trt

        self._cudart = ctypes.CDLL("libcudart.so")
        self._cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self._cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self._cudart.cudaFree.argtypes = [ctypes.c_void_p]
        self._H2D, self._D2H = 1, 2
        self._ctypes = ctypes

        runtime = trt.Runtime(trt.Logger(trt.Logger.ERROR))
        with open(engine_path, "rb") as f:
            self._engine = runtime.deserialize_cuda_engine(f.read())
        self._ctx = self._engine.create_execution_context()
        self._in_name = self._engine.get_tensor_name(0)
        self._in_shape = tuple(self._engine.get_tensor_shape(self._in_name))
        self._h, self._w = int(self._in_shape[2]), int(self._in_shape[3])
        self._bufs = {}
        self._out_names = []
        for i in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(i)
            shape = tuple(self._engine.get_tensor_shape(name))
            dtype = trt.nptype(self._engine.get_tensor_dtype(name))
            nbytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
            ptr = ctypes.c_void_p()
            self._cudart.cudaMalloc(ctypes.byref(ptr), nbytes)
            self._bufs[name] = (ptr, shape, dtype, nbytes)
            self._ctx.set_tensor_address(name, int(ptr.value))
            if i > 0:
                self._out_names.append(name)

    def segment(self, bgr):
        x, scale, pad_hw = _letterbox(bgr, self._h, self._w)
        ptr, _, _, nbytes = self._bufs[self._in_name]
        self._cudart.cudaMemcpy(ptr, x.ctypes.data, nbytes, self._H2D)
        self._ctx.execute_async_v3(0)
        outs = {}
        for name in self._out_names:
            optr, shape, dtype, obytes = self._bufs[name]
            arr = np.empty(shape, dtype=dtype)
            self._cudart.cudaMemcpy(arr.ctypes.data, optr, obytes, self._D2H)
            outs[name] = arr
        det = next(v for v in outs.values() if v.ndim == 3)
        proto = next(v for v in outs.values() if v.ndim == 4)
        return self._decode_exact(det, proto, scale, pad_hw, bgr.shape[:2])

    def _decode_exact(self, det, proto, scale, pad_hw, frame_hw):
        out = det[0].T
        nm = proto.shape[1]
        nc = out.shape[1] - 4 - nm
        bag_scores = out[:, 4:4 + nc][:, list(BAG_CLASSES)].max(axis=1)
        keep = bag_scores >= SEG_CONF
        if not np.any(keep):
            return []
        sub, scores = out[keep], bag_scores[keep].astype(np.float32)
        cx, cy, bw, bh = sub[:, 0], sub[:, 1], sub[:, 2], sub[:, 3]
        fh, fw = frame_hw
        boxes = np.stack([
            np.clip((cx - bw / 2) / scale, 0, fw), np.clip((cy - bh / 2) / scale, 0, fh),
            np.clip((cx + bw / 2) / scale, 0, fw), np.clip((cy + bh / 2) / scale, 0, fh),
        ], axis=1)
        coeffs = sub[:, 4 + nc:]
        order = np.argsort(-scores)
        picked = []
        while order.size and len(picked) < 5:
            i = int(order[0])
            picked.append(i)
            rest = order[1:]
            xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
            yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
            xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
            yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
            inter = np.clip(xx2 - xx1, 0, None) * np.clip(yy2 - yy1, 0, None)
            area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            order = rest[inter / (area[i] + area[rest] - inter + 1e-9) <= SEG_IOU]
        protos = proto[0]
        results = []
        for idx in picked:
            raw_m = np.tensordot(coeffs[idx], protos, axes=(0, 0))
            prob = 1.0 / (1.0 + np.exp(-np.clip(raw_m, -30.0, 30.0)))
            full_m = _resize_proto_to_frame(prob, (self._h, self._w), pad_hw, frame_hw)
            bx1, by1, bx2, by2 = (int(round(v)) for v in boxes[idx])
            mask = np.zeros((fh, fw), np.uint8)
            if bx2 > bx1 and by2 > by1:
                mask[by1:by2, bx1:bx2] = (full_m[by1:by2, bx1:bx2] > 0.5).astype(np.uint8)
            results.append((float(scores[idx]), boxes[idx], mask))
        return results


def _get_segmenter():
    global _SEGMENTER
    if _SEGMENTER is not False:
        return _SEGMENTER
    path = os.path.expanduser(os.environ.get("SO100_BAG_SEG_ENGINE", DEFAULT_SEG_ENGINE))
    if not os.path.isfile(path):
        _SEGMENTER = None
        return None
    try:
        _SEGMENTER = _TrtBagSegmenter(path)
    except Exception:
        _SEGMENTER = None
    return _SEGMENTER


def _blob_from_mask(m, orange, source):
    """Build the standard find_bag dict from a boolean mask m (H, W)."""
    area = int(m.sum())
    if area < 1500:
        return None
    cols = np.where(m.any(0))[0]
    rows = np.where(m.any(1))[0]
    tops = np.array([[c, np.nonzero(m[:, c])[0].min()] for c in cols], float)
    all_tops = tops.copy()  # every column (the fixed camera with the arm out of view: bag_camera.locate)
    # drop columns where the top is hidden by the gripper (orange right above it)
    keep = [not orange[max(0, int(y) - 12):int(y) + 1, int(x)].any() for x, y in tops]
    tops = tops[np.array(keep, bool)] if any(keep) else tops
    return {
        "cx": float(np.mean(cols)), "x0": float(cols.min()), "x1": float(cols.max()), "tops": tops,
        "all_tops": all_tops,
        "y0": float(rows.min()), "y1": float(rows.max()), "h": int(m.shape[0]),
        "area": area, "grip_x": gripper_x(orange), "source": source,
    }


def _largest_component(mask_u8, im_w):
    """Pick the best bag-sized connected component (largest, slightly favouring the centre of the frame)."""
    n, lab, st, cent = cv2.connectedComponentsWithStats(mask_u8)
    if n < 2:
        return None
    best_i, best_score = None, 0.0
    for i in range(1, n):
        area = int(st[i, 4])
        if area < 1500:
            continue
        # Mildly favour objects near horizontal centre (where the robot drove up to the bag)
        centre_bonus = 1.0 - 0.35 * abs(float(cent[i][0]) - im_w / 2) / max(im_w / 2, 1.0)
        sc = area * centre_bonus
        if sc > best_score:
            best_score, best_i = sc, i
    return None if best_i is None else (lab == best_i)


def find_bag(im, segmenter=None):
    """Bag blob: dict(cx, x0, x1, tops, all_tops, y0, y1, h, area, grip_x, source) or None.

    Tries (1) TensorRT YOLO11-seg (any bag color, any floor), (2) adaptive Lab floor-difference mask
    (any bag color on a uniform floor), (3) legacy pink/red HSV mask.
    """
    orange = orange_mask(im)
    seg = segmenter if segmenter is not None else _get_segmenter()
    if seg is not None:
        dets = seg.segment(im)
        if dets:
            # Prefer highest-scoring detection with enough unoccluded mask area
            for _, _, mask in dets:
                m = (mask > 0) & (orange == 0)
                blob = _blob_from_mask(m, orange, "yolo-seg")
                if blob is not None:
                    return blob

    # Fallback 1: color-agnostic floor-difference foreground
    fg = floor_diff_mask(im, orange)
    comp = _largest_component(fg, im.shape[1])
    if comp is not None:
        blob = _blob_from_mask(comp, orange, "floor-diff")
        if blob is not None:
            return blob

    # Fallback 2: legacy pink/red HSV mask
    bag, orange = masks(im)
    comp = _largest_component(bag, im.shape[1])
    if comp is not None:
        return _blob_from_mask(comp, orange, "hsv")
    return None


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
    """Floor pixels in the bottom 10% of an Xtion frame (adaptive Lab floor color)."""
    f = adaptive_floor_mask(xim)
    return f[int(xim.shape[0] * 0.9):]


def bag_bottom(xim):
    """Non-floor (bag) pixels in the bottom 12% of an Xtion frame."""
    fg = ~adaptive_floor_mask(xim)
    return fg[int(xim.shape[0] * 0.88):]


def lifted(before, after):
    """Share of the bottom-row columns that showed the bag before the lift and show floor after it.
    > 0.5 = the bag is off the floor. Works for any bag color and any uniform floor."""
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
    """Image row of the bag's foot on the floor in the fixed front camera.

    For full-bag masks (yolo-seg / floor-diff) the blob's own bottom row is already the foot when visible.
    Otherwise searches below the blob for the first run of adaptive-floor rows.
    """
    if b.get("source") in ("yolo-seg", "floor-diff"):
        return float(b["y1"]) if b["y1"] < b["h"] - 3 else None
    floor = adaptive_floor_mask(im)
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
    """Xtion frame: fraction of floor pixels in the bottom rows. High = the bag is off the floor."""
    return float(floor_mask(xim).mean())


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
