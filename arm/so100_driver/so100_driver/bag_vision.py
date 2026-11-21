"""Image helpers for the camera bag pick (bag_camera.py): find the bag in the webcam view (fixed front camera
since 2026-11-18; the forearm camera before), read the direction of its top ridge, and check whether the bag is
off the floor (fixed camera: fixed_lifted; Xtion: lifted).

The frame is the AX1080P JPEG from webcam_view (MJPG 640x480, the calibration size). This module
does not open the camera and does not change that size.

Bag detection (2026-11-19, color- and floor-agnostic):
  1. Primary: TensorRT YOLO11-seg. bag_model 'coco' (the default) reads COCO backpack / handbag / suitcase
     from ~/hmc-follow/models/yolo11s-seg_480x640.engine (or SO100_BAG_SEG_ENGINE). bag_model 'tuned' reads
     class 0 = bag and class 1 = handle from the fine-tune (bag11s-seg-v2, same 480x640 input). A missing
     or empty tuned engine is logged and the coco engine is used.
  2. Fallback, coco only: adaptive Lab floor-difference, then legacy pink/red HSV. The tuned model leaves
     these off (they mark empty frames and merge blobs).
The orange gripper colour is removed only inside the gripper region. A red bag shares that hue, and masking
the whole frame was taking 35-48% of it.
Floor & lift checks (foot_y, floor_mask, lifted) estimate the floor color adaptively from the frame borders in Lab
space rather than assuming a cyan floor or a pink/red bag.
"""
import logging
import math
import os

import cv2
import numpy as np

BAG_CLASSES = (24, 26, 28)  # COCO: backpack, handbag, suitcase
TUNED_BAG_CLASS = 0
TUNED_HANDLE_CLASS = 1
SEG_CONF = 0.12  # downward-angled close-up bags score 0.19-0.26 on COCO; background < 0.05
SEG_IOU = 0.45
DEFAULT_SEG_ENGINE = os.path.expanduser("~/hmc-follow/models/yolo11s-seg_480x640.engine")
DEFAULT_TUNED_ENGINE = os.path.expanduser("~/hmc-follow/models/bag11s-seg-v2_480x640.engine")
# A gripper is a compact orange part. A red bag's pixels in the same hue are wider than this.
GRIPPER_MIN_PX = 400
GRIPPER_MAX_PX = 20000
_LOG = logging.getLogger("so100_driver.bag_vision")
_WARNED = set()
_SEGMENTERS = {}  # (model, path) -> _TrtBagSegmenter or None


def _warn_once(reason):
    if reason not in _WARNED:
        _WARNED.add(reason)
        _LOG.warning(reason)
    return reason


def resolve_bag_model(model, tuned_engine=None):
    """(model, engine path or None, reason).

    'tuned' with a missing or empty engine file becomes 'coco'. The reason is empty when the
    requested model is the one that will run. Anything other than 'tuned' is coco.
    """
    name = str(model or "coco").strip().lower()
    if name != "tuned":
        return "coco", None, ""
    path = os.path.expanduser(tuned_engine or os.environ.get("SO100_BAG_TUNED_ENGINE", DEFAULT_TUNED_ENGINE))
    if not os.path.isfile(path):
        return "coco", None, _warn_once("tuned bag engine missing (%s); using coco" % path)
    if os.path.getsize(path) == 0:
        return "coco", None, _warn_once("tuned bag engine is empty (%s); using coco" % path)
    return "tuned", path, ""


def _orange_colour(im):
    """Undilated orange pixels (SO-100 gripper hue). A red bag shares part of this range."""
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    return ((H >= 3) & (H <= 22) & (S > 110) & (V > 150)).astype(np.uint8)


def gripper_region(colour):
    """Dilated ROI of the orange gripper, or zeros when the orange is a bag rather than an arm.

    The arm is a narrow component, touching the image border when it enters the frame. A floating
    gripper (the arm tip alone) is a small box. A bag-width orange blob is left out.
    """
    if colour is None or not np.any(colour):
        return np.zeros(colour.shape if colour is not None else (0, 0), np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(colour, connectivity=8)
    h, w = colour.shape
    border, floating = [], []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < GRIPPER_MIN_PX or area > GRIPPER_MAX_PX:
            continue
        x, y, bw, bh = (int(stats[i, k]) for k in range(4))
        touches = x <= 1 or y <= 1 or x + bw >= w - 1 or y + bh >= h - 1
        if touches and bw <= 0.35 * w:
            border.append((area, i))
        elif not touches and bw <= 0.22 * w and bh <= 0.40 * h:
            floating.append((area, i))
    chosen = border or floating
    if not chosen:
        return np.zeros_like(colour)
    _area, index = max(chosen)
    roi = (lab == index).astype(np.uint8)
    return cv2.dilate(roi, np.ones((9, 9), np.uint8))


def orange_mask(im):
    """Orange SO-100 gripper, only inside the gripper region (not the red bag that shares the hue)."""
    colour = _orange_colour(im)
    roi = gripper_region(colour)
    if not np.any(roi):
        return np.zeros_like(colour)
    dilated = cv2.dilate(colour, np.ones((9, 9), np.uint8))
    return ((dilated > 0) & (roi > 0)).astype(np.uint8)


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
    lab, med, thresh = _estimate_floor_lab(im, orange)
    fg = (np.linalg.norm(lab - med, axis=2) > thresh).astype(np.uint8)
    fg[orange > 0] = 0
    # On bright glossy floors, exclude washed-out ceiling-light glare (high V, very low S)
    if med[0] > 140.0:
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        fg[(hsv[..., 2] > 228) & (hsv[..., 1] < 38)] = 0
    # Ignore the extreme top 12% and side 6% borders (robot deck / background clutter)
    fg[:int(h * 0.12), :] = 0
    fg[:, :int(w * 0.06)] = 0
    fg[:, int(w * 0.94):] = 0
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    return fg


def masks(im):
    """Legacy pink/red bag HSV mask + the gripper-region orange mask (kept for the coco fallback)."""
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    orange = orange_mask(im)
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


def decode_yolo_instances(det_out, proto_out, scale, pad_hw, frame_hw, in_hw, classes, conf=SEG_CONF):
    """(score, class_id, box_xyxy, mask) for each class, best first within that class.

    NMS is per class: a handle sits on the bag, so one NMS over both would drop it.
    classes are read directly (0 = bag, 1 = handle on the tuned head; COCO ids on the coco head).
    A class index the head does not have is skipped.
    """
    out = det_out[0].T
    nm = proto_out.shape[1]
    nc = out.shape[1] - 4 - nm
    fh, fw = frame_hw
    protos = proto_out[0]
    results = []
    for class_id in classes:
        if class_id < 0 or class_id >= nc:
            continue
        scores_all = out[:, 4 + class_id]
        keep = np.flatnonzero(scores_all >= conf)
        if keep.size == 0:
            continue
        sub = out[keep]
        scores = scores_all[keep].astype(np.float32)
        cx, cy, bw, bh = sub[:, 0], sub[:, 1], sub[:, 2], sub[:, 3]
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
        for idx in picked:
            raw_m = np.tensordot(coeffs[idx], protos, axes=(0, 0))
            prob = 1.0 / (1.0 + np.exp(-np.clip(raw_m, -30.0, 30.0)))
            full_m = _resize_proto_to_frame(prob, in_hw, pad_hw, frame_hw)
            bx1, by1, bx2, by2 = (int(round(v)) for v in boxes[idx])
            mask = np.zeros((fh, fw), np.uint8)
            if bx2 > bx1 and by2 > by1:
                mask[by1:by2, bx1:bx2] = (full_m[by1:by2, bx1:bx2] > 0.5).astype(np.uint8)
            results.append((float(scores[idx]), int(class_id), boxes[idx], mask))
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
    """Minimal TensorRT wrapper for YOLO11-seg on the Jetson (no PyTorch/Ultralytics needed at runtime).

    classes are the bag class ids (COCO, or 0 on the tuned head). handle_class is 1 on the tuned head
    and None on the coco head, which has no handle class.
    """

    def __init__(self, engine_path, classes=BAG_CLASSES, handle_class=None):
        self._bag_classes = tuple(classes)
        self._handle_class = handle_class
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
        if self._handle_class is None:
            return self._decode_coco(det, proto, scale, pad_hw, frame_hw)
        classes = self._bag_classes + (self._handle_class,)
        return decode_yolo_instances(
            det, proto, scale, pad_hw, frame_hw, (self._h, self._w), classes)

    def _decode_coco(self, det, proto, scale, pad_hw, frame_hw):
        """COCO bag classes, one NMS on the best class score. (score, box, mask), as before."""
        out = det[0].T
        nm = proto.shape[1]
        nc = out.shape[1] - 4 - nm
        ids = [c for c in self._bag_classes if 0 <= c < nc]
        if not ids:
            return []
        bag_scores = out[:, 4:4 + nc][:, ids].max(axis=1)
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


def _get_segmenter(model, tuned_path):
    """The TensorRT segmenter for this model, or None when it cannot be loaded."""
    if model == "tuned":
        path, classes, handle = tuned_path, (TUNED_BAG_CLASS,), TUNED_HANDLE_CLASS
    else:
        path = os.path.expanduser(os.environ.get("SO100_BAG_SEG_ENGINE", DEFAULT_SEG_ENGINE))
        classes, handle = BAG_CLASSES, None
    key = (model, path)
    if key in _SEGMENTERS:
        return _SEGMENTERS[key]
    seg = None
    if path and os.path.isfile(path) and os.path.getsize(path) > 0:
        try:
            seg = _TrtBagSegmenter(path, classes, handle)
        except Exception as exc:  # noqa: BLE001 (a bad engine must not stop the pick)
            _warn_once("bag engine failed to load (%s): %s; using coco" % (path, exc))
    _SEGMENTERS[key] = seg
    return seg


def _instances(dets):
    """(bag detections, handle detections) from a segmenter.

    A 3-tuple is a bag (the coco head). A 4-tuple is (score, class_id, box, mask): class 1 is the
    handle, anything else is a bag. A dict with "bags" and "handles" is accepted too.
    """
    if isinstance(dets, dict):
        return list(dets.get("bags") or []), list(dets.get("handles") or [])
    bags, handles = [], []
    for item in dets or []:
        if len(item) >= 4:
            score, cls, box, mask = item[0], int(item[1]), item[2], item[3]
            (handles if cls == TUNED_HANDLE_CLASS else bags).append((score, box, mask))
        else:
            bags.append(item)
    return bags, handles


def _handle_on_bag(handles, bag_mask, orange):
    """{cx, cy, area} of the best handle whose centroid sits on the bag, else None.

    Gripper pixels are taken off the handle first. A handle away from the bag (a sock, a sleeve)
    is not an aim point.
    """
    ys, xs = np.nonzero(bag_mask)
    if len(xs) == 0:
        return None
    x0, x1, y0, y1 = float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())
    best = None
    for score, _box, mask in handles:
        m = (mask > 0) & (orange == 0)
        if int(m.sum()) < 12:
            continue
        hys, hxs = np.nonzero(m)
        cx, cy = float(hxs.mean()), float(hys.mean())
        if not (x0 - 8 <= cx <= x1 + 8 and y0 - 8 <= cy <= y1 + 8):
            continue
        if best is None or score > best[0]:
            best = (score, {"cx": cx, "cy": cy, "area": int(m.sum())})
    return None if best is None else best[1]


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


def find_bag(im, segmenter=None, model="coco", tuned_engine=None):
    """Bag blob: dict(cx, x0, x1, tops, all_tops, y0, y1, h, area, grip_x, source, model) or None.

    model 'coco' (the default): TensorRT YOLO11-seg on the COCO bag classes, then the floor-difference
    mask, then the pink/red HSV mask. model 'tuned': class 0 is the bag and class 1 is the handle, and
    the two fallbacks stay off. A handle on the bag is blob["handle"] = {cx, cy, area}. A missing or
    empty tuned engine is logged and coco runs instead. segmenter, when given, is used as-is (tests).
    """
    if segmenter is None:
        model, tuned_engine, _reason = resolve_bag_model(model, tuned_engine)
    else:
        model = "tuned" if str(model or "").strip().lower() == "tuned" else "coco"
    orange = orange_mask(im)
    seg = segmenter if segmenter is not None else _get_segmenter(model, tuned_engine)
    if seg is None and model == "tuned" and segmenter is None:
        model = "coco"
        seg = _get_segmenter("coco", None)
    if seg is not None:
        try:
            dets = seg.segment(im)
        except Exception as exc:  # noqa: BLE001 (a bad frame or engine must not stop the pick)
            _warn_once("bag segmenter failed: %s; using coco" % exc)
            dets = []
            if model == "tuned" and segmenter is None:
                model = "coco"
                coco = _get_segmenter("coco", None)
                if coco is not None:
                    try:
                        dets = coco.segment(im)
                    except Exception as exc2:  # noqa: BLE001
                        _warn_once("coco bag segmenter failed: %s" % exc2)
                        dets = []
        bags, handles = _instances(dets)
        for _score, _box, mask in bags:
            m = (mask > 0) & (orange == 0)
            blob = _blob_from_mask(m, orange, "yolo-seg")
            if blob is None:
                continue
            handle = _handle_on_bag(handles, m, orange)
            if handle is not None:
                blob["handle"] = handle
            blob["model"] = model
            return blob
    if model == "tuned":
        return None

    # Fallback 1, coco only: color-agnostic floor-difference foreground (reject huge background blobs)
    fg = floor_diff_mask(im, orange)
    comp = _largest_component(fg, im.shape[1])
    if comp is not None:
        blob = _blob_from_mask(comp, orange, "floor-diff")
        if blob is not None and (blob["x1"] - blob["x0"]) <= im.shape[1] * 0.72 and blob["area"] <= im.shape[0] * im.shape[1] * 0.45:
            blob["model"] = "coco"
            return blob

    # Fallback 2, coco only: legacy pink/red HSV mask
    bag, orange = masks(im)
    comp = _largest_component(bag, im.shape[1])
    if comp is not None:
        blob = _blob_from_mask(comp, orange, "hsv")
        if blob is not None:
            blob["model"] = "coco"
        return blob
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
    """Image row of the bag's foot on the floor in the fixed front camera: below the blob, the first row
    where the adaptive floor shows again (a patterned/white lower bag half may sit below b["y1"], so b["y1"]
    alone is not necessarily the foot). Median over the blob's middle columns; None when the floor below is
    out of the picture."""
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


def spot_floor(before, after, box=None, band=0.12, min_bin=0.01):
    """Share of the spot where the bag stood that no longer shows the bag in `after`: its pixels whose colour is
    none of the spot's colours in `before` (Lab in 8 x 8 x 8 bins; a bin is the bag's when it holds min_bin of the
    spot). box = (x0, y0, x1, y1) pixels of `before`; None = the bottom `band` of the frame, where the Xtion under
    the deck sees the bag's foot. No floor colour is guessed: adaptive_floor_mask took a frame full of bag for
    floor (Xtion, robot 2026-11-19 15:17: 0.86 "floor" before the close, then lifted() 0.93 with the bag still on
    the floor). Those frames give 0.00 (15:17, not lifted) and 0.99 (17:29, lifted). None when a frame is missing,
    the sizes differ or the spot is empty."""
    if before is None or after is None or before.shape != after.shape:
        return None
    h, w = before.shape[:2]
    x0, y0, x1, y1 = (0, int(h * (1 - band)), w, h) if box is None else (int(v) for v in box)
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1), min(h, y1)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None

    def bins(im):
        return ((cv2.cvtColor(im[y0:y1, x0:x1], cv2.COLOR_BGR2LAB) // 32).astype(int) @ (64, 8, 1)).ravel()

    k0, k1 = bins(before), bins(after)
    bag = np.bincount(k0, minlength=512) / k0.size >= min_bin
    return float(1.0 - bag[k1].mean())


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
