"""Person detector on a TensorRT engine. Reads both output layouts used here:

- Ultralytics YOLO (YOLO11n, YOLOv8n): output (1, 84, N) = cx, cy, w, h, 80 class scores; input RGB / 255.
- YOLOX 0.1.1rc0 weights (nano, tiny), exported with decoding: output (1, N, 85) = cx, cy, w, h,
  objectness, 80 class scores; input BGR 0-255, no scaling (measured: RGB / 255 with mean and std,
  as that tag's demo code does, finds nobody; raw BGR gives person scores of 0.86).

Both: letterbox to the engine input, image at the top-left, padding 114. Boxes are in input-frame pixels.
"""

import cv2
import numpy as np

CONF = 0.4
IOU = 0.45
PERSON = 0  # COCO class index
BAGS = {24: "backpack", 26: "handbag", 28: "suitcase"}  # COCO class index -> name, for the bag to carry
BAG_CONF = 0.25


def letterbox(bgr, height, width, yolox):
    """Image -> (1, 3, height, width) float32 and the scale from frame to input pixels."""
    r = min(height / bgr.shape[0], width / bgr.shape[1])
    h, w = int(round(bgr.shape[0] * r)), int(round(bgr.shape[1] * r))
    padded = np.full((height, width, 3), 114, np.uint8)
    padded[:h, :w] = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_LINEAR) if (h, w) != bgr.shape[:2] else bgr
    x = padded.astype(np.float32) if yolox else padded[:, :, ::-1].astype(np.float32) / 255.0
    return x.transpose(2, 0, 1)[None], r


def nms(boxes, scores, iou=IOU):
    """Greedy non-maximum suppression; returns kept indices, best score first."""
    order = np.argsort(-scores)
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        rest = order[1:]
        x1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        y1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        x2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        y2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
        area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        order = rest[inter / (area[i] + area[rest] - inter + 1e-9) <= iou]
    return keep


def empty_reason(output, conf=CONF):
    """Why decode kept no person, for the log. The low Xtion often returns no box while the LiDAR still has legs."""
    out = output[0]
    if out.size == 0 or (out.ndim == 2 and min(out.shape) == 0):
        return "the network returned no rows"
    if out.shape[0] == 84:  # Ultralytics (84, N)
        score = out[4].astype(np.float32) if out.shape[1] else np.zeros(0, np.float32)
    else:
        score = (out[:, 4] * out[:, 5]).astype(np.float32) if out.shape[0] else np.zeros(0, np.float32)
    if score.size == 0:
        return "the network returned no rows"
    best = float(score.max())
    if best < conf:
        return "best person score %.2f is under %.2f" % (best, conf)
    return "every person box was clipped away"


def decode(output, scale, frame_w, frame_h, conf=CONF, cls=PERSON):
    """Raw engine output -> [(x1, y1, x2, y2, conf)] for one class (people by default), in frame pixels."""
    out = output[0]
    if out.shape[0] == 84:  # Ultralytics: (84, N)
        out = out.T
        cxcywh, score = out[:, :4], out[:, 4 + cls]
    else:  # YOLOX: (N, 85)
        cxcywh, score = out[:, :4], out[:, 4] * out[:, 5 + cls]
    mask = score >= conf
    cxcywh, score = cxcywh[mask].astype(np.float32), score[mask].astype(np.float32)
    boxes = np.concatenate([cxcywh[:, :2] - cxcywh[:, 2:] / 2, cxcywh[:, :2] + cxcywh[:, 2:] / 2], axis=1) / scale
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, frame_w)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, frame_h)
    seen = (boxes[:, 2] - boxes[:, 0] >= 2) & (boxes[:, 3] - boxes[:, 1] >= 2)  # drop boxes clipped to nothing
    boxes, score = boxes[seen], score[seen]
    return [(*map(float, boxes[i]), float(score[i])) for i in nms(boxes, score)]


def decode_objects(output, scale, frame_w, frame_h, conf=BAG_CONF):
    """Raw engine output -> [(x1, y1, x2, y2, conf, name)] for the bag classes, best first.

    One bag often scores as two classes (handbag and backpack); NMS across the classes keeps the best name.
    """
    found = [(*b, name) for cls, name in BAGS.items() for b in decode(output, scale, frame_w, frame_h, conf, cls)]
    if not found:
        return []
    boxes = np.array([b[:4] for b in found], np.float32)
    return [found[i] for i in nms(boxes, np.array([b[4] for b in found], np.float32))]


class Detector:
    def __init__(self, engine_path):
        from engine import Engine

        self._engine = Engine(engine_path)
        _, _, self._h, self._w = self._engine.input_shape
        self._yolox = self._engine.output_shape[-1] == 85
        self.empty = None  # why the last detect returned no people; None when it returned some
        self._said = None

    def _note(self, text):
        self.empty = text
        if text != self._said:
            print("detector: 0 people: %s" % text, flush=True)
            self._said = text

    def _execute(self, bgr):
        try:
            x, scale = letterbox(bgr, self._h, self._w, self._yolox)
            return self._engine.run(x)[0], scale
        except RuntimeError as exc:
            self._note("TensorRT execute failed: %s" % exc)
            return None, None

    def detect(self, bgr):
        out, scale = self._execute(bgr)
        if out is None:
            return []
        boxes = decode(out, scale, bgr.shape[1], bgr.shape[0])
        if boxes:
            self.empty = None
        else:
            mean = float(bgr.mean()) if getattr(bgr, "size", 0) else 0.0
            self._note("%s; frame mean %.0f" % (empty_reason(out), mean))
        return boxes

    def detect_all(self, bgr):
        """People and bags from one engine run: ([(x1, y1, x2, y2, conf)], [(x1, y1, x2, y2, conf, name)])."""
        out, scale = self._execute(bgr)
        if out is None:
            return [], []
        people = decode(out, scale, bgr.shape[1], bgr.shape[0])
        if people:
            self.empty = None
        else:
            mean = float(bgr.mean()) if getattr(bgr, "size", 0) else 0.0
            self._note("%s; frame mean %.0f" % (empty_reason(out), mean))
        return people, decode_objects(out, scale, bgr.shape[1], bgr.shape[0])
