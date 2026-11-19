"""Unit tests for color- and floor-agnostic bag_vision.py."""
import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so100_driver import bag_vision as bv  # noqa: E402


def _make_scene(floor_bgr, bag_bgr=None, rect=(220, 140, 420, 400), h=480, w=640):
    im = np.full((h, w, 3), floor_bgr, np.uint8)
    if bag_bgr is not None:
        x0, y0, x1, y1 = rect
        cv2.rectangle(im, (x0, y0), (x1, y1), bag_bgr, -1)
    return im


class _FakeSegmenter:
    def __init__(self, rect, h=480, w=640):
        self.rect = rect
        self.h, self.w = h, w

    def segment(self, bgr):
        x0, y0, x1, y1 = self.rect
        mask = np.zeros((self.h, self.w), np.uint8)
        mask[y0:y1, x0:x1] = 1
        return [(0.92, np.array([x0, y0, x1, y1], float), mask)]


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


if __name__ == "__main__":
    unittest.main()
