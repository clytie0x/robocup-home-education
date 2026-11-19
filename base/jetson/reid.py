"""Look vectors (person re-identification): one 512-number unit vector per box.

The live engine is OSNet ain x1_0 (server.REID). A plan is built for one GPU. "TensorRT execute failed"
with "engine built for a different device" means that file was not built on this Jetson (19 Nov 2026: the
ain plan failed on every re-id, 65 times, and the detector dropped the Pi). Rebuild it here:

    trtexec --onnx=<the matching .onnx> --saveEngine=models/osnet_ain_x1_0_msmt17_256x128.engine --fp16

Until that file runs, ReId loads the fallback (server.REID_FALLBACK, OSNet x0_25) once and retries.
If that also fails, looks() returns None per box. The reply then carries "look": [] and the link stays
up. An empty look must not be enrolled: LookMemory.add of an empty vector drops the next real 512-d look.
"""

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


class ReId:
    def __init__(self, engine_path, fallback_path=None):
        from engine import Engine, EngineError

        self._EngineError = EngineError
        self._gave_up = False
        if fallback_path == engine_path:
            fallback_path = None
        self._fallback_path = fallback_path
        try:
            self._engine = Engine(engine_path)
        except EngineError as exc:
            self._engine = self._load_fallback(exc)
        if self._engine is not None:
            self._max_batch = self._engine.max_input_shape[0]

    def _load_fallback(self, exc):
        """Load the fallback engine once. None when there is none, or it will not load."""
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

    def _vectors(self, batch):
        feats = [self._engine.run(batch[i:i + self._max_batch])[0] for i in range(0, len(batch), self._max_batch)]
        return [tuple(map(float, row)) for row in unit_rows(np.concatenate(feats))]

    def looks(self, bgr, boxes):
        if not len(boxes):
            return []
        if self._engine is None:
            return [None] * len(boxes)
        batch = crops(bgr, boxes)
        try:
            return self._vectors(batch)
        except self._EngineError as exc:
            self._engine = self._load_fallback(exc)
            if self._engine is None:
                if not self._gave_up:
                    print("re-id off for this process: %s" % exc, flush=True)
                    self._gave_up = True
                return [None] * len(boxes)
            self._max_batch = self._engine.max_input_shape[0]
            try:
                return self._vectors(batch)
            except (self._EngineError, RuntimeError) as exc2:
                self._engine = None
                if not self._gave_up:
                    print("re-id off for this process: %s" % exc2, flush=True)
                    self._gave_up = True
                return [None] * len(boxes)
