"""Light bench: what venue lighting (dimmed lights, spotlights, shadows, colour casts, flicker, black frames) does
to person detection, the operator's look and the depth, on real frames, with each normalise mode of light.py.

Run on the Jetson (TensorRT engines), over a folder of frames:
    python3 tools/light_bench.py FRAMES_DIR --detector models/yolo11n_256x320.engine \
        --reid models/osnet_x0_25_msmt17_256x128.engine [--json out.json]
On a PC any Ultralytics ONNX works with onnxruntime (a pose model counts as a person detector):
    python tools/light_bench.py FRAMES_DIR --detector yolo11n-pose_256x320.onnx --reid hist
FRAMES_DIR holds .jpg / .png frames (BGR as saved) and .npz files with "rgb" (H, W, 3, RGB order) and optionally
"depth" (uint16 mm, (H, W) or (N, H, W)). Frames are resized to 320 x 240, the Xtion size the Pi sends.

Reference: the people the detector finds in the unaltered frame without normalisation (conf >= detector.CONF);
the operator is the biggest of them, and their look in the unaltered frame (same normalise mode) is the enrolled
look. For each lighting change and mode:
  recall   share of reference people found again (IoU >= 0.5)
  iou      mean IoU of each reference box with its best match (0 when missed)
  look     mean cosine of the operator's look to the enrolled one, over the frames where they were found
  look_ok  share of frames where the operator was found and the look scores >= 0.65 (senses.LOOK_MATCH)
  gate     share of frames light.FrameGate refuses, by reason (a fresh gate per frame: dark and saturated only;
           the flicker runs feed it the sequence and so test the flash rule too)
--reid hist uses a hue-saturation histogram as a stand-in look (not OSNet: only shows how colour shifts).
Depth (npz with depth): valid share in the person_range patch and how often the range must fall back (valid share
< 0.3, as senses.person_range) under a washed-out disc and random dropout, and how far the 30th percentile moves
when it does not fall back.
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import light  # noqa: E402
from detector import CONF, decode, letterbox  # noqa: E402

IOU_FOUND = 0.5
LOOK_OK = 0.65  # senses.LOOK_MATCH, hmc.follower.OPERATOR_MATCH
RANGE_VALID = 0.3  # senses.person_range: fewer valid pixels than this share and the range is None
FPS = 30.0  # the Xtion colour stream
SEQUENCE = 60  # frames per flicker run (2 s)


def load_frames(folder):
    """[(name, bgr 240 x 320, depth stack (N, 240, 320) or None)]"""
    found = []
    for path in sorted(glob.glob(os.path.join(folder, "*"))):
        ext = os.path.splitext(path)[1].lower()
        depth = None
        if ext in (".jpg", ".jpeg", ".png"):
            bgr = cv2.imread(path)
        elif ext == ".npz":
            data = np.load(path)
            bgr = np.ascontiguousarray(data["rgb"][:, :, ::-1])
            if "depth" in data.files:
                depth = data["depth"] if data["depth"].ndim == 3 else data["depth"][None]
        else:
            continue
        if bgr is None:
            continue
        if bgr.shape[:2] != (240, 320):
            bgr = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_AREA)
            depth = None  # depth only kept at its own size
        found.append((os.path.basename(path), bgr, depth))
    return found


class OnnxDetector:
    """An Ultralytics ONNX detector (1, 84, N) or pose model (1, 56, N) on onnxruntime: people only."""

    def __init__(self, path):
        import onnxruntime as ort

        self._session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        inp = self._session.get_inputs()[0]
        self._name, (_, _, self._h, self._w) = inp.name, inp.shape

    def detect(self, bgr):
        x, scale = letterbox(bgr, self._h, self._w, yolox=False)
        out = self._session.run(None, {self._name: x})[0]
        if out.shape[1] == 56:
            from server import decode_pose
            return [(*box, score) for box, score, _ in decode_pose(out, scale, bgr.shape[1], bgr.shape[0], conf=CONF)]
        return decode(out, scale, bgr.shape[1], bgr.shape[0])


class OnnxReId:
    def __init__(self, path):
        import onnxruntime as ort

        self._session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self._name = self._session.get_inputs()[0].name

    def looks(self, bgr, boxes):
        from reid import crops, unit_rows
        if not len(boxes):
            return []
        return [tuple(r) for r in unit_rows(self._session.run(None, {self._name: crops(bgr, boxes)})[0])]


class HistLook:
    """Stand-in look: 16 x 8 hue-saturation histogram of the box, unit length. Not OSNet."""

    def looks(self, bgr, boxes):
        out = []
        for b in boxes:
            x1, y1 = max(0, int(b[0])), max(0, int(b[1]))
            x2, y2 = max(x1 + 1, int(round(b[2]))), max(y1 + 1, int(round(b[3])))
            hsv = cv2.cvtColor(bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
            h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).ravel()
            out.append(tuple(h / max(np.linalg.norm(h), 1e-12)))
        return out


def make_detector(path):
    if path.endswith(".onnx"):
        return OnnxDetector(path)
    from detector import Detector
    return Detector(path)


def make_reid(path):
    if path is None:
        return None
    if path == "hist":
        return HistLook()
    if path.endswith(".onnx"):
        return OnnxReId(path)
    from reid import ReId
    return ReId(path)


def iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def conditions(bgr, operator):
    """name -> altered frame, every lighting change of the bench for one frame (operator: their reference box)."""
    cx, cy = (operator[0] + operator[2]) / 2, (operator[1] + operator[3]) / 2
    size = max(operator[2] - operator[0], operator[3] - operator[1])
    side = (min(319.0, operator[2]), operator[1] + (operator[3] - operator[1]) * 0.3)
    return {
        "original": bgr,
        "dim 0.5": light.dim(bgr, 0.5), "dim 0.3": light.dim(bgr, 0.3),
        "dim 0.2": light.dim(bgr, 0.2), "dim 0.1": light.dim(bgr, 0.1),
        "dim 0.3 + AE": light.dim_auto(bgr, 0.3), "dim 0.1 + AE": light.dim_auto(bgr, 0.1),
        "gamma 0.5": light.gamma(bgr, 0.5), "gamma 2.0": light.gamma(bgr, 2.0),
        "over x1.8": light.overexpose(bgr, 1.8), "over x3": light.overexpose(bgr, 3.0),
        "over x6": light.overexpose(bgr, 6.0),
        "spot on body": light.spotlight(bgr, (cx, cy), 0.6 * size),
        "spot at edge": light.spotlight(bgr, side, 0.6 * size),
        "shadow": light.shadow(bgr),
        "warm": light.cast(bgr, 0.65, 0.9, 1.2), "cold": light.cast(bgr, 1.2, 0.95, 0.7),
        "bands": light.bands(bgr),
        "black": np.zeros_like(bgr),
    }


def flicker_runs(bgr):
    """name -> list of frames: a light switching at 2 to 25 Hz, black frames, and lights dimmed for good."""
    runs = {}
    for hz in (2, 5, 10, 25):
        runs["flicker %d Hz" % hz] = [light.dim(bgr, g, seed=i) for i, g in
                                      enumerate(light.flicker_gains(hz, FPS, SEQUENCE))]
    runs["flicker 10 Hz to 0.1"] = [light.dim(bgr, g, seed=i) for i, g in
                                    enumerate(light.flicker_gains(10, FPS, SEQUENCE, low=0.1))]
    runs["black every 5th"] = [np.zeros_like(bgr) if i % 5 == 4 else light.dim(bgr, 1.0, seed=i)
                               for i in range(SEQUENCE)]
    runs["dimmed to 0.3, stays"] = [light.dim(bgr, 1.0 if i < 10 else 0.3, seed=i) for i in range(SEQUENCE)]
    return runs


class Tally:
    def __init__(self, has_look):
        self.has_look = has_look
        self.refs = self.found = self.frames = self.look_ok = 0
        self.ious, self.cos, self.refused = [], [], {}
        self.passed, self.low_passed, self.low = 0, 0, 0

    def add(self, refs, boxes, op_look_fn, enrolled, refused, light_fn=None, reference=None):
        """refs: reference boxes (operator first); boxes: found; op_look_fn(box) -> look; refused: why or None."""
        self.frames += 1
        if refused:
            self.refused[refused] = self.refused.get(refused, 0) + 1
        best_op = None
        for k, r in enumerate(refs):
            scores = [iou(r, b) for b in boxes]
            best = max(scores, default=0.0)
            self.refs += 1
            self.ious.append(best)
            if best >= IOU_FOUND:
                self.found += 1
                if k == 0:
                    best_op = boxes[int(np.argmax(scores))]
        if best_op is not None and enrolled is not None:
            c = float(np.dot(op_look_fn(best_op), enrolled))
            self.cos.append(c)
            self.look_ok += c >= LOOK_OK
            self.low += c < LOOK_OK
            if light_fn is None or light.light_ok(light_fn(best_op), reference):
                self.passed += 1
                self.low_passed += c < LOOK_OK

    def row(self):
        r = {"frames": self.frames, "recall": round(self.found / self.refs, 3) if self.refs else None,
             "iou": round(float(np.mean(self.ious)), 3) if self.ious else None,
             "look": round(float(np.mean(self.cos)), 3) if self.cos else None,
             "look_min": round(float(np.min(self.cos)), 3) if self.cos else None,
             "look_ok": round(self.look_ok / self.frames, 3) if self.frames and self.has_look else None,
             "gate": round(sum(self.refused.values()) / self.frames, 3) if self.frames else None}
        if self.cos:
            # the light rule (light.light_ok): looks it lets through, and of the looks under 0.65, how many it lets
            # through (each one could make the follower drop the operator for that step)
            r["light_passed"] = round(self.passed / len(self.cos), 3)
            r["low_looks"] = "%d, %d passed" % (self.low, self.low_passed)
        if self.refused:
            r["refused"] = dict(self.refused)
        return r


def bench(frames, detector, reid, modes, flicker=True):
    rows = []
    refs_of = {}
    for name, bgr, _ in frames:
        refs = sorted(detector.detect(bgr), key=lambda b: -(b[2] - b[0]) * (b[3] - b[1]))
        if refs:
            refs_of[name] = [b[:4] for b in refs]
    print("frames with people: %d of %d" % (len(refs_of), len(frames)), file=sys.stderr)
    usable = [(n, b, d) for n, b, d in frames if n in refs_of]

    def look_of(img, box):
        return np.array(reid.looks(img, [box])[0]) if reid else None

    for mode in modes:
        tallies = {}
        for name, bgr, _ in usable:
            refs = refs_of[name]
            enrolled = look_of(light.normalise(bgr, mode), refs[0])
            reference = light.box_light(bgr, refs[0])
            for cond, img in conditions(bgr, refs[0]).items():
                ok, why = light.FrameGate().check(img)
                norm = light.normalise(img, mode)
                tallies.setdefault(cond, Tally(reid is not None)).add(
                    refs, detector.detect(norm), lambda b: look_of(norm, b), enrolled, None if ok else why,
                    lambda b: light.box_light(img, b), reference)
            if not flicker:
                continue
            for run, seq in flicker_runs(bgr).items():
                gate = light.FrameGate()
                every = tallies.setdefault(run, Tally(reid is not None))
                kept = tallies.setdefault(run + " (gate kept)", Tally(reid is not None))
                for img in seq:
                    ok, why = gate.check(img)
                    norm = light.normalise(img, mode)
                    boxes = detector.detect(norm)
                    every.add(refs, boxes, lambda b: look_of(norm, b), enrolled, None if ok else why,
                              lambda b: light.box_light(img, b), reference)
                    if ok:
                        kept.add(refs, boxes, lambda b: look_of(norm, b), enrolled, None,
                                 lambda b: light.box_light(img, b), reference)
        for cond, t in tallies.items():
            rows.append({"kind": "light", "condition": cond, "mode": mode, **t.row()})
    rows += depth_rows(usable, refs_of)
    rows += timing_rows(usable, modes)
    return rows


def depth_rows(usable, refs_of):
    rows = []
    for name, _, stack in usable:
        if stack is None:
            continue
        box = refs_of[name][0]
        cx, cy = (box[0] + box[2]) / 2, box[1] + (box[3] - box[1]) * 0.7  # the middle of the range patch
        h = box[3] - box[1]
        for radius, dropout in [(0, 0.0), (0.15, 0.0), (0.3, 0.0), (0, 0.3), (0, 0.6), (0.15, 0.3)]:
            shares, fallback, moved = [], 0, []
            for i, depth in enumerate(stack):
                washed = light.wash_depth(depth, (cx, cy), radius * h, dropout, seed=i)
                s = light.valid_share(washed, box)
                shares.append(s)
                if s < RANGE_VALID:
                    fallback += 1
                    continue
                (r0, r1), (c0, c1) = light.person_patch(box, depth.shape)
                before, after = depth[r0:r1, c0:c1], washed[r0:r1, c0:c1]
                if np.count_nonzero(before):
                    moved.append(abs(float(np.percentile(after[after > 0], 30))
                                     - float(np.percentile(before[before > 0], 30))))
            rows.append({"kind": "depth", "frame": name, "wash_radius_of_box_height": radius, "dropout": dropout,
                         "depth_frames": len(stack), "valid_share": round(float(np.mean(shares)), 3),
                         "falls_back": round(fallback / len(stack), 3),
                         "p30_moved_mm_max": round(max(moved), 1) if moved else None})
    return rows


def timing_rows(usable, modes, repeats=200):
    """Cost of normalise plus the gate on one 320 x 240 frame, median over `repeats` (this machine's CPU)."""
    if not usable:
        return []
    bgr = usable[0][1]
    rows = []
    for mode in modes:
        gate, times = light.FrameGate(), []
        for i in range(repeats):
            img = light.dim(bgr, 0.3) if i % 2 else bgr
            t = time.perf_counter()
            gate.check(img)
            light.normalise(img, mode)
            times.append(time.perf_counter() - t)
        rows.append({"kind": "timing", "mode": mode, "ms_median": round(1000 * float(np.median(times)), 3),
                     "ms_p95": round(1000 * float(np.percentile(times, 95)), 3)})
    return rows


def table(rows):
    lines = []
    for r in rows:
        if r["kind"] == "light":
            lines.append("%-30s %-15s recall %-5s iou %-5s look %-5s min %-6s ok %-5s light %-5s low %-10s gate %-5s %s"
                         % (r["condition"], r["mode"], r["recall"], r["iou"], r["look"], r["look_min"], r["look_ok"],
                            r.get("light_passed", ""), r.get("low_looks", ""), r["gate"], r.get("refused", "")))
        else:
            lines.append(json.dumps(r))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames", help="folder of .jpg / .png / .npz frames")
    ap.add_argument("--detector", required=True, help=".engine (TensorRT) or .onnx (onnxruntime)")
    ap.add_argument("--reid", help=".engine, .onnx, or hist (stand-in); none: no look numbers")
    ap.add_argument("--modes", default=",".join(light.MODES), help="normalise modes, comma separated")
    ap.add_argument("--no-flicker", action="store_true")
    ap.add_argument("--json", help="also write the rows here")
    args = ap.parse_args()
    frames = load_frames(args.frames)
    if not frames:
        raise SystemExit("no frames in %s" % args.frames)
    rows = bench(frames, make_detector(args.detector), make_reid(args.reid), args.modes.split(","),
                 flicker=not args.no_flicker)
    print(table(rows))
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"detector": args.detector, "reid": args.reid, "frames": [n for n, _, _ in frames],
                       "rows": rows}, f, indent=1)


if __name__ == "__main__":
    main()
