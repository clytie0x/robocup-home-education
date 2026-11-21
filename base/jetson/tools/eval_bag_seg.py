"""Score bag segmentation models (YOLO11-seg ONNX, classes bag and handle) on a folder of frames and draw contact
sheets: to compare a new bag model with the one on the Jetson (bag11n-seg-v3) on the same held-out frames before it
goes to the robot. Laptop CPU, onnxruntime; a model inside a zip is read from the zip, nothing is unpacked.

  python tools/eval_bag_seg.py FRAMES_DIR --out SHEETS_DIR --model A.onnx --model B.zip:path/in/zip/B.onnx
                               [--labels LABELS_DIR]
The split zips in robocup-models/parts/ are joined into one .zip first (copy /b part00+part01+... v3.zip).

The preprocessing the models were exported with: the frame resized to 640 x 480, RGB / 255, input (1, 3, 480, 640);
outputs (1, 38, 6300) = cx, cy, w, h, 2 class scores (0 bag, 1 handle), 32 mask coefficients, and the mask
prototypes (1, 32, 120, 160). A detection has a class score >= CONF, NMS per class at IOU; its mask is
sigmoid(coefficients . prototypes) > 0.5 at 480 x 640, cut to its box.

Per frame and model one line with the bag and handle scores. With --labels (YOLO-seg: LABELS_DIR/<frame name>.txt,
a line "class x1 y1 x2 y2 ..." per object, polygon normalised to 0-1; no file or no bag line = a frame with no bag),
per model:
  recall    labelled bags that a predicted bag mask matches at mask IoU >= MATCH_IOU (one prediction per bag)
  mean IoU  mean mask IoU of each labelled bag with its match (0 when missed)
  false     predicted bags on the frames with no bag label
Contact sheets: SHEETS_DIR/sheet_N.jpg, a row per frame, a tile per model, SHEET_ROWS frames a sheet.
"""
import argparse
import glob
import os
import sys
import zipfile

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detector import nms  # noqa: E402

INPUT_HW = (480, 640)  # px (h, w): the models' input
CLASSES = ("bag", "handle")
CONF = 0.25       # class score for a detection
IOU = 0.5         # box IoU above which NMS drops the lower score, per class
MATCH_IOU = 0.5   # mask IoU for a predicted bag to count as the labelled one
TILE = (320, 240)  # px (w, h): one model's picture of one frame on a contact sheet
SHEET_ROWS = 4    # frames a contact sheet
COLOURS = {0: (0, 0, 255), 1: (0, 255, 0)}  # BGR: bag red, handle green


def load_model(spec):
    """(name, onnxruntime session) for "model.onnx" or "archive.zip:member.onnx"."""
    import onnxruntime as ort  # noqa: PLC0415 (only main needs it; the tests run without a model)
    head, sep, member = spec.partition(".zip:")
    if sep:
        with zipfile.ZipFile(head + ".zip") as z:
            source = z.read(member)
    else:
        source = member = spec
    name = os.path.splitext(os.path.basename(member))[0]
    return name, ort.InferenceSession(source, providers=["CPUExecutionProvider"])


def preprocess(bgr):
    """Frame -> (the 480 x 640 BGR image the masks refer to, the (1, 3, 480, 640) RGB / 255 float32 input)."""
    img = cv2.resize(bgr, INPUT_HW[::-1])
    return img, cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32).transpose(2, 0, 1)[None] / 255.0


def decode(out, protos, conf=CONF, iou=IOU):
    """Outputs (1, 38, N) and (1, 32, 120, 160) -> [(class, score, box x1 y1 x2 y2 int, bool mask 480 x 640)]."""
    nm = protos.shape[1]
    if out.shape[1] != 4 + len(CLASSES) + nm:
        raise ValueError("output %s is not 4 box + %d classes + %d mask values" % (out.shape, len(CLASSES), nm))
    p = out[0].T  # (N, 38)
    scores = p[:, 4:4 + len(CLASSES)]
    cls, score = scores.argmax(1), scores.max(1)
    keep = score >= conf
    p, cls, score = p[keep], cls[keep], score[keep]
    boxes = np.stack([p[:, 0] - p[:, 2] / 2, p[:, 1] - p[:, 3] / 2, p[:, 0] + p[:, 2] / 2, p[:, 1] + p[:, 3] / 2], 1)
    flat = protos[0].reshape(nm, -1)
    h, w = INPUT_HW
    dets = []
    for c in range(len(CLASSES)):
        idx = np.where(cls == c)[0]
        for j in nms(boxes[idx], score[idx], iou):
            i = idx[j]
            logit = np.clip((p[i, -nm:] @ flat).reshape(protos.shape[2:]), -30.0, 30.0)
            m = cv2.resize((1 / (1 + np.exp(-logit))).astype(np.float32), (w, h)) > 0.5
            b = boxes[i].astype(int)
            inbox = np.zeros_like(m)
            inbox[max(b[1], 0):max(b[3], 0), max(b[0], 0):max(b[2], 0)] = True
            dets.append((c, float(score[i]), b, m & inbox))
    return dets


def read_labels(path, hw=INPUT_HW):
    """YOLO-seg label file -> [(class, bool mask h x w)]; [] when there is no file (a frame with no bag)."""
    if not os.path.isfile(path):
        return []
    h, w = hw
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            v = line.split()
            if not v:
                continue
            if len(v) < 7 or len(v) % 2 == 0:
                raise ValueError("%s line %d: not 'class x1 y1 x2 y2 x3 y3 ...'" % (path, n))
            pts = (np.array(v[1:], float).reshape(-1, 2) * (w, h)).round().astype(np.int32)
            m = np.zeros(hw, np.uint8)
            cv2.fillPoly(m, [pts], 1)
            out.append((int(v[0]), m.astype(bool)))
    return out


def mask_iou(a, b):
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.0


def match_ious(truth, preds):
    """Mask IoU of each labelled mask with its own predicted mask: best pairs first, a prediction used once, 0 for
    a labelled mask left without one."""
    m = np.array([[mask_iou(t, p) for p in preds] for t in truth]).reshape(len(truth), len(preds))
    best = np.zeros(len(truth))
    while m.size and m.max() > 0:
        i, j = np.unravel_index(m.argmax(), m.shape)
        best[i] = m[i, j]
        m[i, :] = -1
        m[:, j] = -1
    return best


class Score:
    """One model's bag recall, mean mask IoU and false bags over the labelled frames."""

    def __init__(self):
        self.ious, self.false, self.empty = [], 0, 0

    def add(self, labels, dets):
        truth = [m for c, m in labels if c == 0]
        preds = [d[3] for d in dets if d[0] == 0]
        if truth:
            self.ious.extend(match_ious(truth, preds))
        else:
            self.false += len(preds)
            self.empty += 1

    def line(self, name):
        ious = np.array(self.ious)
        return "%s: bag recall %d/%d, mean mask IoU %.2f, false bags %d on %d frames with no bag" % (
            name, int((ious >= MATCH_IOU).sum()), len(ious), ious.mean() if len(ious) else 0.0, self.false, self.empty)


def draw(img, dets, label):
    out = img.copy()
    for c, conf, b, m in dets:
        out[m] = (0.5 * out[m] + 0.5 * np.array(COLOURS[c])).astype(np.uint8)
        cv2.rectangle(out, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), COLOURS[c], 2)
        cv2.putText(out, "%s %.2f" % (CLASSES[c], conf), (int(b[0]), max(int(b[1]) - 4, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOURS[c], 2)
    cv2.putText(out, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", help="folder of .jpg / .png frames")
    ap.add_argument("--model", action="append", required=True, help="model.onnx or archive.zip:member.onnx; repeat")
    ap.add_argument("--labels", help="folder of YOLO-seg .txt labels named like the frames")
    ap.add_argument("--out", required=True, help="folder for the contact sheets (made when missing)")
    a = ap.parse_args(argv)
    frames = sorted(f for f in glob.glob(os.path.join(a.frames, "*")) if f.lower().endswith((".jpg", ".jpeg", ".png")))
    if not frames:
        sys.exit("no .jpg / .png frames in %s" % a.frames)
    models = [(*load_model(s), Score()) for s in a.model]
    rows = []
    for f in frames:
        bgr = cv2.imread(f)
        if bgr is None:
            print("%s: not readable, skipped" % os.path.basename(f))
            continue
        img, x = preprocess(bgr)
        labels = None
        if a.labels:
            labels = read_labels(os.path.join(a.labels, os.path.splitext(os.path.basename(f))[0] + ".txt"))
        line, tiles = [os.path.basename(f)], []
        for name, sess, score in models:
            out, protos = sorted(sess.run(None, {sess.get_inputs()[0].name: x}), key=np.ndim)
            dets = decode(out, protos)
            bags, handles = ([round(d[1], 2) for d in dets if d[0] == c] for c in (0, 1))
            line.append("%s bag %s handle %s" % (name, bags, handles))
            if labels is not None:
                score.add(labels, dets)
            tiles.append(cv2.resize(draw(img, dets, name), TILE))
        print(" | ".join(line), flush=True)
        rows.append(np.hstack(tiles))
    os.makedirs(a.out, exist_ok=True)
    for i in range(0, len(rows), SHEET_ROWS):
        cv2.imwrite(os.path.join(a.out, "sheet_%d.jpg" % (i // SHEET_ROWS)), np.vstack(rows[i:i + SHEET_ROWS]))
    print("%d contact sheets in %s" % ((len(rows) + SHEET_ROWS - 1) // SHEET_ROWS, a.out))
    if a.labels:
        for name, _sess, score in models:
            print(score.line(name))


if __name__ == "__main__":
    main()
