"""Box decoding, NMS and look-vector maths on fake outputs (no GPU); the engine tests skip without tensorrt."""

import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import detector  # noqa: E402
import engine  # noqa: E402
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

    def test_bags_handbag_backpack_suitcase_from_their_own_scores(self):
        out = np.zeros((1, 84, 5), np.float32)
        out[0, :4, 0], out[0, 4 + 26, 0] = (100, 200, 30, 30), 0.6  # handbag
        out[0, :4, 1], out[0, 4 + 24, 1] = (200, 200, 40, 50), 0.3  # backpack, above 0.25
        out[0, :4, 2], out[0, 4 + 28, 2] = (260, 190, 50, 60), 0.2  # suitcase below 0.25
        out[0, :4, 3], out[0, 4 + 0, 3] = (150, 120, 40, 200), 0.9  # a person is not a bag
        out[0, :4, 4], out[0, 4 + 24, 4] = (101, 201, 30, 30), 0.5  # the handbag again, as a backpack
        bags = detector.decode_objects(out, 1.0, 320, 240)
        self.assertEqual([(b[5], round(b[4], 2)) for b in bags], [("handbag", 0.6), ("backpack", 0.3)])
        self.assertEqual([round(v, 3) for v in bags[0][:4]], [85.0, 185.0, 115.0, 215.0])
        self.assertEqual(len(detector.decode(out, 1.0, 320, 240)), 1)  # people unchanged

    def test_bags_from_yolox_output(self):
        out = np.zeros((1, 1, 85), np.float32)
        out[0, 0, :5], out[0, 0, 5 + 28] = (100, 100, 40, 60, 0.8), 0.5  # 0.4
        self.assertEqual([b[5] for b in detector.decode_objects(out, 1.0, 320, 240)], ["suitcase"])

    def test_nms_keeps_separate_boxes(self):
        boxes = np.array([[0, 0, 10, 10], [20, 20, 30, 30], [1, 1, 10, 10]], np.float32)
        self.assertEqual(detector.nms(boxes, np.array([0.5, 0.9, 0.6], np.float32)), [1, 2])

    def test_empty_reason_names_a_score_under_the_threshold(self):
        out = yolo_output([(100, 100, 40, 120, 0.2)])
        self.assertIn("0.20", detector.empty_reason(out))
        self.assertIn("0.40", detector.empty_reason(out))
        self.assertIn("no rows", detector.empty_reason(np.zeros((1, 84, 0), np.float32)))


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

    def test_red_clothes_match_red_and_not_blue(self):
        red = np.zeros((80, 40, 3), np.uint8)
        red[:, :] = (0, 0, 220)
        blue = np.zeros((80, 40, 3), np.uint8)
        blue[:, :] = (220, 0, 0)
        box = [(0, 0, 40, 80)]
        same = np.dot(reid.clothing_histogram(red, box)[0], reid.clothing_histogram(red, box)[0])
        other = np.dot(reid.clothing_histogram(red, box)[0], reid.clothing_histogram(blue, box)[0])
        self.assertGreater(same, 0.99)
        self.assertLess(other, same)


class EnginePlanTests(unittest.TestCase):
    def test_device_mismatch_and_the_myelin_crash(self):
        self.assertTrue(engine.device_mismatch("engine was built for a different device"))
        self.assertFalse(engine.device_mismatch("ok"))
        self.assertTrue(engine.myelin_dynamic_batch("Myelin __myl_Inor CUDA 720"))
        self.assertEqual(engine.choose_backend(False, False, True, True), "tensorrt")
        self.assertEqual(engine.choose_backend(True, True, True, True), "rebuild")
        self.assertEqual(engine.choose_backend(True, True, True, False), "onnx")
        self.assertEqual(engine.choose_backend(True, True, False, False), "histogram")

    def test_static_rebuild_pins_the_batch_at_one(self):
        seen = []
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        live = os.path.join(folder.name, "a.engine")

        def run(cmd):
            seen.append(cmd)
            dest = next(part.split("=", 1)[1] for part in cmd if str(part).startswith("--saveEngine="))
            with open(dest, "wb") as handle:
                handle.write(b"plan")
            return 0

        # The shape flags are what this test pins. Deserialization needs a real plan and TensorRT.
        original = engine._plan_loads
        engine._plan_loads = engine.engine_present
        try:
            self.assertTrue(engine.rebuild_static(os.path.join(folder.name, "a.onnx"), live,
                                                   run=run, free_bytes=0))
        finally:
            engine._plan_loads = original
        self.assertIn("--minShapes=input:1x3x256x128", seen[0])
        self.assertIn("--optShapes=input:1x3x256x128", seen[0])
        self.assertIn("--maxShapes=input:1x3x256x128", seen[0])
        self.assertIn("--fp16", seen[0])
        self.assertTrue(engine.engine_present(live))

    def test_crops_run_one_at_a_time_and_a_myelin_crash_does_not_raise(self):
        class Counter:
            def __init__(self):
                self.calls = []
                self.max_input_shape = (8, 3, 256, 128)

            def run(self, batch):
                self.calls.append(len(batch))
                return [np.ones((len(batch), 4), np.float32)]

        counter = Counter()
        model = reid.ReId("osnet_ain.engine", runner=counter, trtexec=False)
        bgr = np.zeros((40, 40, 3), np.uint8)
        looks = model.looks(bgr, [(0, 0, 10, 30), (5, 5, 20, 35)])
        self.assertEqual(counter.calls, [1, 1])
        self.assertEqual(len(looks), 2)

        class Boom:
            max_input_shape = (4, 3, 256, 128)
            device_mismatch = False

            def run(self, batch):
                raise RuntimeError("TensorRT execute failed: Myelin __myl_Inor CUDA 720")

        failed = reid.ReId("missing.engine", runner=Boom(), trtexec=False)
        again = failed.looks(bgr, [(0, 0, 10, 30)])
        self.assertEqual(failed.backend, "histogram")
        self.assertEqual(len(again[0]), 128)
        self.assertAlmostEqual(float(np.dot(again[0], again[0])), 1.0, places=5)

    def test_a_different_device_rebuilds_once_then_uses_the_histogram(self):
        import os
        import tempfile

        folder = tempfile.mkdtemp()
        path = os.path.join(folder, "osnet_ain_x1_0_msmt17_256x128.engine")
        open(path, "wb").close()
        open(os.path.splitext(path)[0] + ".onnx", "wb").close()
        calls = []

        class Mismatch:
            max_input_shape = (1, 3, 256, 128)
            device_mismatch = True

            def run(self, batch):
                raise AssertionError("the mismatched plan must not execute")

        def rebuild(onnx, engine_path):
            calls.append((onnx, engine_path))
            return False

        model = reid.ReId(path, runner=Mismatch(), rebuild=rebuild, trtexec=True)
        self.assertEqual(calls, [(os.path.splitext(path)[0] + ".onnx", path)])
        self.assertEqual(model.backend, "histogram")
        self.assertIn("different device", model.why)
        bgr = np.zeros((40, 40, 3), np.uint8)
        got = model.looks(bgr, [(0, 0, 8, 20)])
        self.assertEqual(len(got[0]), 128)

    def test_a_detector_execute_failure_returns_no_people_and_says_why(self):
        det = detector.Detector.__new__(detector.Detector)
        det._h, det._w, det._yolox = 32, 32, False
        det.empty, det._said = None, None

        class Boom:
            def run(self, array):
                raise RuntimeError("TensorRT execute failed: Myelin")

        det._engine = Boom()
        frame = np.zeros((16, 16, 3), np.uint8)
        self.assertEqual(det.detect(frame), [])
        self.assertIn("TensorRT", det.empty)
        self.assertEqual(det.detect_all(frame), ([], []))


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
