"""tools/pick_bag_cam.py: front camera frame -> bag centre in bag_arm.py's robot frame (synthetic frames, no arm)."""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

import pick_bag_cam as pbc  # noqa: E402

try:
    BV, BC = pbc.arm_modules()
except ImportError:  # the arm team's tree is not on this machine
    BV = BC = None

CAM = {"calibrated": True, "fx": 805.66, "cx": 319.5, "cy": 239.5, "x": 0.33, "y": 0.0, "z": 0.89,
       "yaw_deg": 0.0, "pitch_deg": 61.37, "roll_deg": 0.0, "mirror": False}


def frame(x0=240, x1=420, y0=340, y1=470):
    img = np.full((480, 640, 3), (200, 190, 120), np.uint8)   # cyan-ish floor (BGR)
    img[y0:y1, x0:x1] = (90, 30, 210)                          # pink-red bag
    return img


@unittest.skipIf(BV is None, "no arm team source tree")
class PickBagCamTest(unittest.TestCase):
    def setUp(self):
        self.cam = BC.FrontCamera(CAM)

    def test_robot_frame_is_the_pan_axis_plus_the_mount(self):
        self.assertEqual(pbc.robot_xy((0.30, -0.02)), (0.30 + 0.012, -0.02 + 0.0145))

    def test_the_bag_is_found_where_the_arm_code_finds_it(self):
        img = frame()
        res = pbc.find(img, self.cam, BV, BC)
        self.assertTrue(res["ok"], res)
        b = BV.find_bag(img)
        where = BC.locate(self.cam, b, (0.255, 0.0), 0.055, 0.27, foot=BV.foot_y(img, b))
        self.assertAlmostEqual(res["x"], where["centre"][0] + 0.012, places=3)
        self.assertAlmostEqual(res["y"], where["centre"][1] + 0.0145, places=3)

    def test_the_measured_pan_axis_puts_the_bag_nearer_than_the_old_sum(self):
        """2026-11-24: the pan axis is 0.255 m in the camera's frame, not LASER_X 0.175 + 0.012 = 0.187."""
        img = frame()
        new = pbc.find(img, self.cam, BV, BC)
        old = pbc.find(img, self.cam, BV, BC, cam_pan=(0.187, 0.0145))
        self.assertAlmostEqual(old["x"] - new["x"], 0.255 - 0.187, delta=0.002)  # x, y are rounded to 1 mm
        self.assertAlmostEqual(old["y"] - new["y"], 0.0 - 0.0145, delta=0.002)

    def test_a_bag_on_the_robots_right_has_negative_y(self):
        res = pbc.find(frame(430, 600), self.cam, BV, BC)
        self.assertTrue(res["ok"], res)
        self.assertLess(res["y"], -0.05)

    def test_a_blob_much_wider_than_a_bag_is_refused(self):
        res = pbc.find(frame(40, 600, 60, 470), self.cam, BV, BC)
        self.assertFalse(res["ok"])
        self.assertIn("wide", res["why"])

    def test_a_centre_out_of_reach_is_refused(self):
        res = pbc.find(frame(), self.cam, BV, BC, reach=(0.20, 0.25))
        self.assertFalse(res["ok"])
        self.assertIn("outside", res["why"])

    def test_no_bag_is_refused(self):
        res = pbc.find(np.full((480, 640, 3), (200, 190, 120), np.uint8), self.cam, BV, BC)
        self.assertFalse(res["ok"])
        self.assertIn("no bag", res["why"])


if __name__ == "__main__":
    unittest.main()
