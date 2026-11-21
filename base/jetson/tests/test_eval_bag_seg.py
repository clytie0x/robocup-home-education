"""tools/eval_bag_seg.py: decoding, NMS, masks, labels and the scores on synthetic model outputs (no model file)."""
import contextlib
import io
import os
import sys
import tempfile
import types
import unittest
from unittest import mock

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

import eval_bag_seg as ev  # noqa: E402

N = 6300
A = (160, 120, 320, 240)  # x1 y1 x2 y2 in input px: prototype 0 is on here
B = (400, 320, 600, 440)  # prototype 1


def protos():
    """(1, 32, 120, 160): prototype k is +10 inside its box (a quarter of the input px), -10 elsewhere."""
    p = np.full((1, 32, 120, 160), -10.0, np.float32)
    for k, (x1, y1, x2, y2) in enumerate((A, B)):
        p[0, k, y1 // 4:y2 // 4, x1 // 4:x2 // 4] = 10.0
    return p


def output(rows):
    """(1, 38, N) with rows of (class, score, box x1 y1 x2 y2, prototype); every other column scores 0."""
    out = np.zeros((1, 38, N), np.float32)
    for n, (c, score, (x1, y1, x2, y2), k) in enumerate(rows):
        out[0, :4, n] = ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)
        out[0, 4 + c, n] = score
        out[0, 6 + k, n] = 1.0
    return out


def rect(box, hw=ev.INPUT_HW):
    m = np.zeros(hw, bool)
    m[box[1]:box[3], box[0]:box[2]] = True
    return m


class DecodeTest(unittest.TestCase):
    def test_nms_runs_per_class_and_low_scores_are_dropped(self):
        rows = [(0, 0.9, A, 0), (0, 0.8, (165, 125, 325, 245), 0), (0, 0.7, B, 1), (1, 0.6, A, 0),
                (0, 0.2, (0, 0, 50, 50), 0)]
        dets = ev.decode(output(rows), protos())
        self.assertEqual([(c, round(s, 2)) for c, s, _b, _m in dets], [(0, 0.9), (0, 0.7), (1, 0.6)])
        np.testing.assert_array_equal(dets[0][2], A)

    def test_the_mask_comes_from_the_prototypes_at_input_size(self):
        (_c, _s, _b, m), = ev.decode(output([(0, 0.9, A, 0)]), protos())
        self.assertEqual(m.shape, ev.INPUT_HW)
        self.assertGreater(ev.mask_iou(m, rect(A)), 0.95)

    def test_the_mask_is_cut_to_its_box(self):
        half = (160, 120, 240, 240)
        (_c, _s, _b, m), = ev.decode(output([(0, 0.9, half, 0)]), protos())  # prototype 0 reaches x 320
        self.assertEqual(m[:, 240:].sum(), 0)
        self.assertGreater(ev.mask_iou(m, rect(half)), 0.95)

    def test_nothing_above_the_threshold_gives_no_detection(self):
        self.assertEqual(ev.decode(output([(0, 0.1, A, 0)]), protos()), [])

    def test_a_model_with_another_layout_is_refused(self):
        with self.assertRaises(ValueError):
            ev.decode(np.zeros((1, 116, N), np.float32), protos())  # a COCO seg model: 80 classes

    def test_preprocess_is_rgb_over_255_at_480_by_640(self):
        blue = np.zeros((240, 320, 3), np.uint8)
        blue[..., 0] = 255
        img, x = ev.preprocess(blue)
        self.assertEqual(img.shape, (480, 640, 3))
        self.assertEqual((x.shape, x.dtype), ((1, 3, 480, 640), np.float32))
        self.assertEqual((x[0, 0].max(), x[0, 2].min()), (0.0, 1.0))


class LabelTest(unittest.TestCase):
    def label(self, text):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        path = os.path.join(d.name, "f.txt")
        with open(path, "w") as f:
            f.write(text)
        return path

    def test_a_polygon_becomes_a_mask_at_input_size(self):
        (c, m), = ev.read_labels(self.label("0 0.25 0.25 0.5 0.25 0.5 0.5 0.25 0.5\n\n"))
        self.assertEqual(c, 0)
        self.assertGreater(ev.mask_iou(m, rect(A)), 0.97)

    def test_no_file_is_a_frame_with_no_bag(self):
        self.assertEqual(ev.read_labels(os.path.join(HERE, "no_such_label.txt")), [])

    def test_a_box_label_is_refused(self):
        with self.assertRaises(ValueError):
            ev.read_labels(self.label("0 0.5 0.5 0.2 0.2\n"))  # YOLO detect format, not a polygon


class ScoreTest(unittest.TestCase):
    def test_a_prediction_matches_one_labelled_bag_the_best_pair_first(self):
        near = (170, 120, 330, 240)
        np.testing.assert_allclose(ev.match_ious([rect(near), rect(A)], [rect(A)]), [0.0, 1.0])
        np.testing.assert_allclose(ev.match_ious([rect(A)], []), [0.0])
        self.assertEqual(len(ev.match_ious([], [rect(A)])), 0)

    def test_recall_mean_iou_and_false_bags(self):
        s = ev.Score()
        s.add([(0, rect(A)), (1, rect(B))], [(0, 0.9, None, rect(A)), (1, 0.6, None, rect(B))])
        s.add([], [(0, 0.5, None, rect(A)), (0, 0.4, None, rect(B)), (1, 0.4, None, rect(B))])
        s.add([(0, rect(B))], [(1, 0.6, None, rect(B))])  # only a handle where the bag is
        self.assertEqual(s.line("m"), "m: bag recall 1/2, mean mask IoU 0.50, false bags 2 on 1 frames with no bag")


class FakeSession:
    def get_inputs(self):
        return [types.SimpleNamespace(name="images")]

    def run(self, _names, feed):
        assert feed["images"].shape == (1, 3, 480, 640)
        return [protos(), output([(0, 0.9, A, 0)])]  # protos first: main must not rely on the order


class MainTest(unittest.TestCase):
    def test_frames_labels_and_sheets(self):
        with tempfile.TemporaryDirectory() as d:
            frames, labels, out = (os.path.join(d, n) for n in ("frames", "labels", "out"))
            os.makedirs(frames)
            os.makedirs(labels)
            for n in ("a", "b"):
                cv2.imwrite(os.path.join(frames, n + ".jpg"), np.zeros((480, 640, 3), np.uint8))
            with open(os.path.join(labels, "a.txt"), "w") as f:
                f.write("0 0.25 0.25 0.5 0.25 0.5 0.5 0.25 0.5\n")
            text = io.StringIO()
            with mock.patch.object(ev, "load_model", return_value=("fake", FakeSession())), \
                    contextlib.redirect_stdout(text):
                ev.main([frames, "--model", "x.zip:fake.onnx", "--labels", labels, "--out", out])
            sheet = cv2.imread(os.path.join(out, "sheet_0.jpg"))
        self.assertIn("a.jpg | fake bag [0.9] handle []", text.getvalue())
        self.assertIn("fake: bag recall 1/1, mean mask IoU 0.9", text.getvalue())
        self.assertIn("false bags 1 on 1 frames with no bag", text.getvalue())
        self.assertEqual(sheet.shape, (2 * ev.TILE[1], ev.TILE[0], 3))


if __name__ == "__main__":
    unittest.main()
