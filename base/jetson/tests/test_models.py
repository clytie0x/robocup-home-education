"""Box decoding, NMS and look-vector maths on fake outputs (no GPU); the engine tests skip without tensorrt."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import detector  # noqa: E402
import reid  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "..", "models")


def yolo_output(rows):
    """rows of (cx, cy, w, h, person score) -> fake Ultralytics output (1, 84, N)."""
    out = np.zeros((1, 84, len(rows)), np.float32)
    for i, (cx, cy, w, h, s) in enumerate(rows):
        out[0, :5, i] = (cx, cy, w, h, s)
        out[0, 5, i] = 0.99  # a confident bicycle must not count
    return out


class DecodeTests(unittest.TestCase):
    def test_yolo_people_only_with_threshold_and_nms(self):
        out = yolo_output([(100, 100, 40, 120, 0.9), (102, 101, 40, 120, 0.8), (250, 120, 30, 90, 0.5),
                           (50, 50, 20, 20, 0.39)])
        boxes = detector.decode(out, 1.0, 320, 240)
        self.assertEqual(len(boxes), 2)  # duplicate suppressed, 0.39 dropped
        self.assertEqual([round(v, 3) for v in boxes[0]], [80.0, 40.0, 120.0, 160.0, 0.9])
        self.assertAlmostEqual(boxes[1][4], 0.5, places=5)

    def test_yolox_score_is_objectness_times_person(self):
        out = np.zeros((1, 2, 85), np.float32)
        out[0, 0, :6] = (100, 100, 40, 120, 0.9, 0.9)  # 0.81
        out[0, 1, :6] = (200, 100, 40, 120, 0.9, 0.4)  # 0.36, below 0.4
        boxes = detector.decode(out, 1.0, 320, 240)
        self.assertEqual(len(boxes), 1)
        self.assertAlmostEqual(boxes[0][4], 0.81, places=5)

    def test_scale_back_and_clip_to_frame(self):
        out = yolo_output([(10, 10, 40, 40, 0.9)])
        x1, y1, x2, y2, _ = detector.decode(out, 0.5, 320, 240)[0]
        self.assertEqual((x1, y1, x2, y2), (0.0, 0.0, 60.0, 60.0))

    def test_box_off_the_edge_is_dropped(self):
        out = yolo_output([(335, 100, 30, 120, 0.9), (100, 250, 40, 20, 0.9), (100, 100, 40, 120, 0.8)])
        boxes = detector.decode(out, 1.0, 320, 240)
        self.assertEqual(len(boxes), 1)
        self.assertAlmostEqual(boxes[0][4], 0.8, places=5)

    def test_letterbox_pads_bottom_and_keeps_scale(self):
        bgr = np.full((240, 320, 3), (10, 20, 30), np.uint8)
        x, r = detector.letterbox(bgr, 256, 320, yolox=False)
        self.assertEqual((x.shape, r), ((1, 3, 256, 320), 1.0))
        self.assertAlmostEqual(float(x[0, 0, 0, 0]), 30 / 255, places=5)  # RGB order
        self.assertAlmostEqual(float(x[0, 0, 255, 0]), 114 / 255, places=5)
        x, _ = detector.letterbox(bgr, 256, 320, yolox=True)
        self.assertEqual(float(x[0, 0, 0, 0]), 10.0)  # BGR, not scaled

    def test_nms_keeps_separate_boxes(self):
        boxes = np.array([[0, 0, 10, 10], [20, 20, 30, 30], [1, 1, 10, 10]], np.float32)
        self.assertEqual(detector.nms(boxes, np.array([0.5, 0.9, 0.6], np.float32)), [1, 2])


class LookTests(unittest.TestCase):
    def test_unit_rows(self):
        rows = reid.unit_rows(np.array([[3.0, 4.0], [0.0, 0.0], [-2.0, 0.0]]))
        np.testing.assert_allclose(np.linalg.norm(rows[[0, 2]], axis=1), [1.0, 1.0])
        np.testing.assert_allclose(rows[0], [0.6, 0.8])
        self.assertTrue(np.all(np.isfinite(rows)))

    def test_crops_shape_and_box_at_edge(self):
        bgr = np.zeros((240, 320, 3), np.uint8)
        batch = reid.crops(bgr, [(-5, -5, 50, 100), (300, 200, 400, 300), (10, 10, 10, 10), (320, 100, 320, 200),
                                 (100, 240, 140, 240)])
        self.assertEqual(batch.shape, (5, 3, 256, 128))


try:
    import tensorrt  # noqa: F401
    HAVE_TRT = True
except ImportError:
    HAVE_TRT = False


@unittest.skipUnless(HAVE_TRT and os.path.exists(os.path.join(MODELS, "two_people_forest.jpg")), "needs tensorrt and models")
class EngineTests(unittest.TestCase):
    def test_two_people_found_and_unit_looks(self):
        import cv2

        img = cv2.resize(cv2.imread(os.path.join(MODELS, "two_people_forest.jpg")), (320, 240))
        boxes = detector.Detector(os.path.join(MODELS, "yolo11n_256x320.engine")).detect(img)
        self.assertEqual(len(boxes), 2)
        self.assertTrue(all(b[4] >= 0.4 for b in boxes))
        looks = reid.ReId(os.path.join(MODELS, "osnet_x0_25_msmt17_256x128.engine")).looks(img, [b[:4] for b in boxes])
        self.assertEqual([len(v) for v in looks], [512, 512])
        for v in looks:
            self.assertAlmostEqual(float(np.dot(v, v)), 1.0, places=4)


if __name__ == "__main__":
    unittest.main()
