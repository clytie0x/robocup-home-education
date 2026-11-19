"""Look vectors (person re-identification): one unit vector per box.

The live engine is OSNet ain x1_0 (server.REID). On TensorRT 10.14+ (this Jetson: 10.16.2.10) a dynamic
batch plus InstanceNorm crashes in Myelin __myl_Inor with CUDA 720 (514 times from 19 Nov 12:04). Crops
are run one at a time, and on that crash the plan is rebuilt here with a static batch of 1 when the ONNX
file and trtexec are present. Otherwise an existing OSNet x1_0 is tried, then the fallback engine
(server.REID_FALLBACK, OSNet x0_25, loaded once — the same loader as before). If none of those run,
looks() returns a clothing histogram, still without raising. An empty look is not enrolled.
"""

import os

import cv2
import numpy as np

HEIGHT, WIDTH = 256, 128
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def crops(bgr, boxes):
    """Boxes -> (n, 3, 256, 128) float32, RGB with ImageNet mean and std (torchreid's test transform)."""
    batch = np.empty((len(boxes), 3, HEIGHT, WIDTH), np.float32)
    h, w = bgr.shape[:2]
    for i, box in enumerate(boxes):
        x1, y1 = min(w - 1, max(0, int(box[0]))), min(h - 1, max(0, int(box[1])))  # never an empty crop
        x2, y2 = min(w, max(x1 + 1, int(round(box[2])))), min(h, max(y1 + 1, int(round(box[3]))))
        crop = cv2.resize(bgr[y1:y2, x1:x2], (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
        batch[i] = ((crop[:, :, ::-1].astype(np.float32) / 255.0 - MEAN) / STD).transpose(2, 0, 1)
    return batch


def unit_rows(features):
    norms = np.linalg.norm(features, axis=1, keepdims=True)
    return features / np.maximum(norms, 1e-12)


HIST_H, HIST_S = 16, 8  # hue x saturation of the torso
X1_NAME = "osnet_x1_0_msmt17_256x128.engine"  # an existing plan, tried before the x0_25 fallback


def clothing_histogram(bgr, boxes):
    """Unit hue-saturation histogram of the torso of each box. Same length for every box."""
    if not len(boxes):
        return []
    frame_h, frame_w = bgr.shape[:2]
    out = []
    for box in boxes:
        x1 = min(frame_w - 1, max(0, int(box[0])))
        y1 = min(frame_h - 1, max(0, int(box[1])))
        x2 = min(frame_w, max(x1 + 1, int(round(box[2]))))
        y2 = min(frame_h, max(y1 + 1, int(round(box[3]))))
        top = y1 + (y2 - y1) // 4
        bot = y1 + 3 * (y2 - y1) // 4
        if bot <= top:
            top, bot = y1, y2
        crop = bgr[top:bot, x1:x2]
        if crop.size == 0:
            vec = np.zeros(HIST_H * HIST_S, np.float32)
            vec[0] = 1.0
            out.append(tuple(float(v) for v in vec))
            continue
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [HIST_H, HIST_S], [0, 180, 0, 256]).reshape(-1)
        norm = float(np.linalg.norm(hist))
        if norm:
            vec = hist / norm
        else:
            vec = np.zeros_like(hist)
            vec[0] = 1.0
        out.append(tuple(vec.astype(float)))
    return out


class ReId:
    def __init__(self, engine_path, fallback_path=None, runner=None, rebuild=None, trtexec=None):
        from engine import EngineError

        self.path = engine_path
        self.backend = "tensorrt"
        self.why = None
        self._EngineError = EngineError
        self._gave_up = False
        self._rebuilt = False
        self._rebuild = rebuild
        self._trtexec = trtexec
        if fallback_path == engine_path:
            fallback_path = None
        self._fallback_path = fallback_path
        if runner is not None:
            self._engine = runner
            self._EngineError = RuntimeError
            if getattr(runner, "device_mismatch", False):
                self._recover(RuntimeError("engine plan was built for a different device"))
            return
        from engine import Engine
        try:
            self._engine = Engine(engine_path)
        except EngineError as exc:
            self._recover(exc)

    def _load_fallback(self, exc):
        """Load the x0_25 fallback once. None when there is none, or it will not load."""
        path = self._fallback_path
        self._fallback_path = None
        if not path:
            print("re-id engine failed and there is no fallback: %s" % exc, flush=True)
            return None
        print("re-id engine failed (%s); trying %s" % (exc, path), flush=True)
        from engine import Engine, EngineError
        try:
            return Engine(path)
        except (EngineError, OSError, RuntimeError, ImportError) as exc2:
            print("re-id fallback failed: %s" % exc2, flush=True)
            return None

    def _try_onnx(self):
        """CPU ONNX of the same file, when onnxruntime is installed. False otherwise (it is not, here)."""
        onnx = os.path.splitext(self.path)[0] + ".onnx"
        if not os.path.exists(onnx):
            return False
        try:
            import onnxruntime as ort
            self._ort = ort.InferenceSession(onnx, providers=["CPUExecutionProvider"])
            self._ort_name = self._ort.get_inputs()[0].name
        except Exception as exc:  # noqa: BLE001 — missing package or a plan that will not load
            self.why = "%s; onnx failed (%s)" % (self.why, exc)
            return False
        self.backend = "onnx"
        self.why = None
        print("reid: onnxruntime CPU (%s)" % onnx, flush=True)
        return True

    def _onnx_vectors(self, batch):
        rows = [self._row(self._ort.run(None, {self._ort_name: batch[i:i + 1]})[0]) for i in range(len(batch))]
        return [tuple(map(float, row)) for row in unit_rows(np.stack(rows))]

    def _row(self, out):
        out = np.asarray(out)
        return np.reshape(out, (out.shape[0], -1))[0] if out.ndim >= 2 else out.reshape(-1)

    def _vectors(self, batch):
        """One crop per execute. A dynamic batch of the ain plan is the CUDA 720 crash."""
        rows = [self._row(self._engine.run(batch[i:i + 1])[0]) for i in range(len(batch))]
        return [tuple(map(float, row)) for row in unit_rows(np.stack(rows))]

    def looks(self, bgr, boxes):
        if not len(boxes):
            return []
        if self.backend == "histogram" or (self._engine is None and self.backend != "onnx"):
            return clothing_histogram(bgr, boxes)
        batch = crops(bgr, boxes)
        if self.backend == "onnx":
            try:
                return self._onnx_vectors(batch)
            except Exception as exc:  # noqa: BLE001 — a bad session must not drop the link
                self.why = "onnx failed (%s)" % exc
                self.backend = "histogram"
                return clothing_histogram(bgr, boxes)
        try:
            return self._vectors(batch)
        except self._EngineError as exc:
            self._recover(exc)
            got = self._again(bgr, boxes)
            if got is not None:
                return got
            if self.backend == "tensorrt":
                # the rebuilt plan still crashed: try x1_0, the x0_25 fallback, then ONNX
                self._engine = None
                self._rebuilt = True
                self._recover(exc)
                got = self._again(bgr, boxes)
                if got is not None:
                    return got
            self.backend = "histogram"
            self._engine = None
            return clothing_histogram(bgr, boxes)

    def _again(self, bgr, boxes):
        """Retry the crops on whatever _recover just selected. None when that also fails."""
        batch = crops(bgr, boxes)
        try:
            if self.backend == "onnx":
                return self._onnx_vectors(batch)
            if self.backend == "tensorrt" and self._engine is not None:
                return self._vectors(batch)
        except Exception:  # noqa: BLE001 — EngineError and a bad ONNX session both land here
            return None
        return None

    def _recover(self, exc):
        """Static batch-1 rebuild, then an existing x1_0, then the x0_25 fallback, then a histogram."""
        self.why = str(exc)
        from engine import myelin_dynamic_batch, rebuild_static, trtexec_path
        text = str(exc)
        onnx = os.path.splitext(self.path)[0] + ".onnx"
        has_trtexec = bool(self._trtexec) if self._trtexec is not None else bool(trtexec_path())
        if not self._rebuilt and os.path.exists(onnx) and has_trtexec and (
            myelin_dynamic_batch(text) or "execute failed" in text.lower() or "different device" in text.lower()
        ):
            self._rebuilt = True
            rebuild = self._rebuild or rebuild_static
            if rebuild(onnx, self.path):
                try:
                    from engine import Engine
                    self._engine = Engine(self.path)
                    self.backend = "tensorrt"
                    print("reid: static batch 1 (%s)" % self.path, flush=True)
                    self.why = None
                    return
                except Exception as exc2:  # noqa: BLE001
                    self.why = "%s; static engine did not load (%s)" % (exc, exc2)
        x1 = os.path.join(os.path.dirname(self.path) or ".", X1_NAME)
        if os.path.exists(x1) and os.path.abspath(x1) != os.path.abspath(self.path):
            from engine import Engine, EngineError
            try:
                self._engine = Engine(x1)
                self.path = x1
                self.backend = "tensorrt"
                print("reid: %s (%s)" % (X1_NAME, self.why), flush=True)
                self.why = None
                return
            except (EngineError, OSError, RuntimeError, ImportError) as exc2:
                self.why = "%s; %s failed (%s)" % (self.why, X1_NAME, exc2)
        engine = self._load_fallback(exc)
        if engine is not None:
            self._engine = engine
            self.backend = "tensorrt"
            self.why = None
            return
        if self._try_onnx():
            return
        self._engine = None
        self.backend = "histogram"
        if not self._gave_up:
            print("reid: histogram (%s)" % self.why, flush=True)
            self._gave_up = True
