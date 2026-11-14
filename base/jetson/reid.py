"""Look vectors (person re-identification) on a TensorRT engine of OSNet x0_25: one 512-number unit vector per box."""

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
    def __init__(self, engine_path):
        from engine import Engine

        self._engine = Engine(engine_path)
        self._max_batch = self._engine.max_input_shape[0]

    def looks(self, bgr, boxes):
        if not len(boxes):
            return []
        batch = crops(bgr, boxes)
        feats = [self._engine.run(batch[i:i + self._max_batch])[0] for i in range(0, len(batch), self._max_batch)]
        return [tuple(map(float, row)) for row in unit_rows(np.concatenate(feats))]
