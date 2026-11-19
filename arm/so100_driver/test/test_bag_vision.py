"""Unit tests for color- and floor-agnostic bag_vision.py."""
import os
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so100_driver import bag_vision as bv  # noqa: E402
from so100_driver.bag_camera import FrontCamera, locate  # noqa: E402


def _make_scene(floor_bgr, bag_bgr=None, rect=(220, 140, 420, 400), h=480, w=640):
    im = np.full((h, w, 3), floor_bgr, np.uint8)
    if bag_bgr is not None:
        x0, y0, x1, y1 = rect
        cv2.rectangle(im, (x0, y0), (x1, y1), bag_bgr, -1)
    return im


def _bgr(h, s, v):
    pix = np.uint8([[[h, s, v]]])
    return tuple(int(c) for c in cv2.cvtColor(pix, cv2.COLOR_HSV2BGR)[0, 0])


class _FakeSegmenter:
    def __init__(self, rect, h=480, w=640, handle=None, extra=None):
        self.rect = rect
        self.h, self.w = h, w
        self.handle = handle
        self.extra = extra

    def segment(self, bgr):
        x0, y0, x1, y1 = self.rect
        mask = np.zeros((self.h, self.w), np.uint8)
        mask[y0:y1, x0:x1] = 1
        dets = [(0.92, 0, np.array([x0, y0, x1, y1], float), mask)]
        if self.handle is not None:
            hx0, hy0, hx1, hy1 = self.handle
            hm = np.zeros_like(mask)
            hm[hy0:hy1, hx0:hx1] = 1
            dets.append((0.80, 1, np.array([hx0, hy0, hx1, hy1], float), hm))
        if self.extra is not None:
            dets.append(self.extra)
        return dets


class _EmptySegmenter:
    def segment(self, bgr):
        return []


class BagVisionTest(unittest.TestCase):
    def test_any_bag_color_on_any_floor(self):
        cases = [
            ("brown_on_grey", (150, 150, 150), (60, 100, 145)),
            ("black_on_wood", (130, 170, 200), (20, 20, 20)),
            ("white_on_dark_carpet", (55, 55, 60), (235, 235, 235)),
            ("blue_on_beige", (180, 200, 210), (170, 70, 30)),
            ("pink_on_cyan", (190, 190, 90), (120, 50, 210)),
        ]
        for name, floor_bgr, bag_bgr in cases:
            with self.subTest(name=name):
                im = _make_scene(floor_bgr, bag_bgr, rect=(240, 150, 400, 390))
                b = bv.find_bag(im)
                self.assertIsNotNone(b, f"missed {name}")
                self.assertAlmostEqual(b["cx"], 320.0, delta=6.0)
                self.assertAlmostEqual(b["y0"], 150.0, delta=6.0)
                self.assertAlmostEqual(b["y1"], 389.0, delta=6.0)
                self.assertAlmostEqual(bv.foot_y(im, b), 389.0, delta=6.0)

    def test_empty_floor_returns_none(self):
        im = _make_scene((150, 150, 150), bag_bgr=None)
        self.assertIsNone(bv.find_bag(im))

    def test_lifted_detects_non_pink_bag_on_non_cyan_floor(self):
        before = _make_scene((155, 155, 155), (50, 90, 135), rect=(200, 260, 440, 479))
        after = _make_scene((155, 155, 155), (50, 90, 135), rect=(200, 120, 440, 340))
        share, cols = bv.lifted(before, after)
        self.assertGreater(cols, 100)
        self.assertGreater(share, 0.8)

        still_share, _ = bv.lifted(before, before)
        self.assertLess(still_share, 0.2)

    def test_segmenter_backend_takes_priority_and_excludes_orange_gripper(self):
        im = _make_scene((140, 140, 140), (138, 138, 138), rect=(220, 160, 420, 410))  # low-contrast bag
        # Draw orange gripper bar above middle columns
        cv2.rectangle(im, (300, 100), (340, 165), (0, 120, 255), -1)
        b = bv.find_bag(im, segmenter=_FakeSegmenter((220, 160, 420, 410)))
        self.assertIsNotNone(b)
        self.assertEqual(b["source"], "yolo-seg")
        self.assertAlmostEqual(b["y1"], 409.0, delta=2.0)
        self.assertAlmostEqual(bv.foot_y(im, b), 409.0, delta=2.0)
        # Orange-occluded columns dropped from tops, kept in all_tops
        self.assertLess(len(b["tops"]), len(b["all_tops"]))

    def test_decode_yolo_seg_extracts_bag_mask(self):
        det = np.zeros((1, 116, 10), np.float32)
        proto = np.zeros((1, 32, 120, 160), np.float32)
        det[0, 0, 3] = 320.0  # cx
        det[0, 1, 3] = 240.0  # cy
        det[0, 2, 3] = 160.0  # w
        det[0, 3, 3] = 200.0  # h
        det[0, 4 + 26, 3] = 0.85  # COCO handbag
        det[0, 84, 3] = 5.0  # positive coeff on proto channel 0
        proto[0, 0, 35:85, 60:100] = 2.0
        res = bv.decode_yolo_seg(det, proto, scale=1.0, pad_hw=(480, 640), frame_hw=(480, 640))
        self.assertEqual(len(res), 1)
        score, box, mask = res[0]
        self.assertAlmostEqual(score, 0.85, places=2)
        self.assertGreater(int(mask.sum()), 10000)

    def test_orange_mask_stays_on_the_gripper_not_the_red_bag(self):
        bag_bgr = _bgr(8, 180, 210)  # inside the gripper hue, the colour that used to eat the bag
        im = _make_scene((170, 175, 175), bag_bgr, rect=(160, 150, 480, 430))
        cv2.rectangle(im, (300, 0), (340, 70), _bgr(14, 230, 240), -1)  # arm, touching the top edge
        orange = bv.orange_mask(im)
        bag = np.zeros(orange.shape, bool)
        bag[150:430, 160:480] = True
        bag[:80, :] = False
        removed = float(orange[bag].mean()) if bag.any() else 1.0
        self.assertLess(removed, 0.05)
        self.assertGreater(int(orange[:70, 300:340].sum()), 100)
        # The same wide bag with no arm is not a gripper, even where it touches the frame.
        wide = _make_scene((170, 175, 175), bag_bgr, rect=(80, 200, 560, 479))
        self.assertEqual(int(bv.orange_mask(wide).sum()), 0)

    def test_tuned_model_reads_the_handle_and_skips_fallbacks(self):
        im = _make_scene((190, 190, 90), (120, 50, 210), rect=(220, 160, 420, 410))
        missed = bv.find_bag(im, segmenter=_EmptySegmenter(), model="tuned")
        self.assertIsNone(missed)
        coco = bv.find_bag(im, segmenter=_EmptySegmenter(), model="coco")
        self.assertIsNotNone(coco)
        self.assertIn(coco["source"], ("floor-diff", "hsv"))
        self.assertEqual(coco["model"], "coco")

        aimed = bv.find_bag(
            im, segmenter=_FakeSegmenter((220, 160, 420, 410), handle=(300, 150, 340, 176)), model="tuned")
        self.assertEqual(aimed["source"], "yolo-seg")
        self.assertEqual(aimed["model"], "tuned")
        self.assertAlmostEqual(aimed["handle"]["cx"], 319.5, delta=1.0)
        self.assertAlmostEqual(aimed["handle"]["cy"], 162.5, delta=1.0)
        # a higher-scoring handle off the bag is not the aim
        far = np.zeros((480, 640), np.uint8)
        far[10:40, 10:40] = 1
        off = bv.find_bag(im, segmenter=_FakeSegmenter(
            (220, 160, 420, 410), extra=(0.99, 1, np.array([10, 10, 40, 40], float), far)), model="tuned")
        self.assertNotIn("handle", off)

    def test_tuned_head_reads_bag_and_handle_classes(self):
        det = np.zeros((1, 38, 4), np.float32)
        proto = np.zeros((1, 32, 8, 10), np.float32)
        det[0, 0, 0], det[0, 1, 0], det[0, 2, 0], det[0, 3, 0] = 20, 16, 12, 12
        det[0, 4, 0] = 0.91  # class 0, bag
        det[0, 6, 0] = 4.0
        det[0, 0, 1], det[0, 1, 1], det[0, 2, 1], det[0, 3, 1] = 20, 8, 8, 6
        det[0, 5, 1] = 0.77  # class 1, handle
        det[0, 7, 1] = 4.0
        proto[0, 0] = 2.0
        proto[0, 1] = 2.0
        frame = (32, 40)
        res = bv.decode_yolo_instances(det, proto, 1.0, frame, frame, frame, (0, 1))
        classes = sorted(item[1] for item in res)
        self.assertEqual(classes, [0, 1])
        for _score, _cls, _box, mask in res:
            self.assertGreater(int(mask.sum()), 10)
        self.assertEqual(bv.decode_yolo_instances(det, proto, 1.0, frame, frame, frame, (24, 26, 28)), [])

    def test_a_missing_or_empty_tuned_engine_uses_coco(self):
        model, path, reason = bv.resolve_bag_model("tuned", "/no/such/bag11s-seg-v2.engine")
        self.assertEqual((model, path), ("coco", None))
        self.assertIn("missing", reason)
        self.assertIn("coco", reason)
        with tempfile.NamedTemporaryFile(suffix=".engine") as empty:
            model, path, reason = bv.resolve_bag_model("tuned", empty.name)
        self.assertEqual((model, path), ("coco", None))
        self.assertIn("empty", reason)
        self.assertEqual(bv.resolve_bag_model("coco", "/no/such/bag11s-seg-v2.engine")[2], "")
        im = _make_scene((150, 150, 150), (60, 100, 145), rect=(240, 150, 400, 390))
        blob = bv.find_bag(im, model="tuned", tuned_engine="/no/such/bag11s-seg-v2.engine")
        self.assertIsNotNone(blob)
        self.assertEqual(blob["model"], "coco")

        class Boom:
            def __init__(self, *args, **kwargs):
                raise RuntimeError("no tensorrt")

        real = bv._TrtBagSegmenter
        bv._SEGMENTERS.clear()
        bv._TrtBagSegmenter = Boom
        try:
            with tempfile.NamedTemporaryFile(suffix=".engine") as bad:
                bad.write(b"not-an-engine")
                bad.flush()
                blob = bv.find_bag(im, model="tuned", tuned_engine=bad.name)
            self.assertIsNotNone(blob)
            self.assertEqual(blob["model"], "coco")
        finally:
            bv._TrtBagSegmenter = real
            bv._SEGMENTERS.clear()

    def test_the_grasp_aims_at_the_handle_centroid(self):
        cam = FrontCamera({
            "calibrated": True, "fx": 500.0, "cx": 320.0, "cy": 240.0,
            "x": 0.05, "y": 0.0, "z": 0.80, "pitch_deg": 40.0,
            "width": 640, "height": 480,
        })
        cols = np.arange(200, 441, 4)
        tops = np.stack([cols, np.full(cols.shape, 320.0)], axis=1)
        blob = {"x0": 200.0, "x1": 440.0, "tops": tops, "all_tops": tops, "y0": 220.0, "y1": 430.0}
        rim = locate(cam, blob, (0.0, 0.0), 0.05, 0.27)
        same = locate(cam, dict(blob, handle={"cx": 320.0, "cy": 320.0, "area": 80}), (0.0, 0.0), 0.05, 0.27)
        aimed = locate(cam, dict(blob, handle={"cx": 400.0, "cy": 320.0, "area": 80}), (0.0, 0.0), 0.05, 0.27)
        self.assertEqual(rim["aim"], "rim")
        self.assertEqual(same["aim"], "handle")
        self.assertAlmostEqual(same["centre"][0], rim["centre"][0], delta=0.01)
        self.assertAlmostEqual(same["centre"][1], rim["centre"][1], delta=0.01)
        self.assertEqual(aimed["from"], "handle")
        self.assertGreater(abs(aimed["centre"][1] - rim["centre"][1]), 0.01)


if __name__ == "__main__":
    unittest.main()
