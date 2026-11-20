"""Bag pick posture from the 2026-11-19 simulation (bag_camera.py): the small-tilt raise, the outward close first,
and the hold check from the carry side. No ROS, no arm."""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so100_driver import bag_camera as bc  # noqa: E402
from so100_driver.task_rules import Failed  # noqa: E402


class RaiseLadder(unittest.TestCase):
    def ladder(self, top=0.05, pitch=-60, want=0.16):
        cam = bc.CameraBag(NS(kin=NS(heading0=0)))
        return cam.raise_ladder(top, pitch, want)

    def test_tilts_only_to_hold_pitch_and_ends_at_the_height(self):
        steps = self.ladder()
        self.assertEqual(len(steps), bc.RAISE_STEPS)
        self.assertAlmostEqual(steps[-1][0], bc.HOLD_PITCH)
        self.assertAlmostEqual(steps[-1][1], 0.16)
        pitches = [p for p, _z in steps]
        self.assertTrue(all(-60 < p <= bc.HOLD_PITCH for p in pitches))    # never flatter than HOLD_PITCH
        self.assertEqual(pitches, sorted(pitches))                          # a little more tilt each step

    def test_never_goes_down(self):
        steps = self.ladder(top=0.20, want=0.16)                            # already high: stays there
        self.assertTrue(all(z >= 0.20 for _p, z in steps))

    def test_bag_bottom_clears_the_lidar_at_the_default_height(self):
        # tips at bag_carry_bottom + grip height: the bag's bottom (tips - grip height) over LIDAR_Z + 2 cm
        self.assertGreater(0.28, bc.LIDAR_Z + 0.02)


class OutwardCloseFirst(unittest.TestCase):
    """grasp_pose takes a pose whose closing axis points away from the arm before one pointing back at it,
    at any pitch; grasp_outward moves the gap nearer before giving that up."""

    def camera(self, outward_ok):
        cam = bc.CameraBag(NS(kin=NS(heading0=0, limits={"wrist_roll": (-1.48, 4.74)}), table_z=-0.331,
                              say=lambda s: None))
        cam.p = lambda k: {"bag_cam_min_align": 0.85}[k]
        cam.roll_for = lambda r, z, pt, d: (0.0, 0.9)
        cam.closing_sign = lambda r, z, pt, rl, d: 1.0 if abs(rl - 180.0) < 1 else -1.0
        cam.feasible = lambda r, z, pt, rl, pre=None: (abs(rl - 180.0) >= 1) or outward_ok(r, pt)
        return cam

    def test_outward_at_another_pitch_wins_over_inward_at_the_first(self):
        cam = self.camera(lambda r, pt: pt == -70)
        pt, rl, _al = cam.grasp_pose(0.30, -0.12, -60, (1, 0, 0))
        self.assertEqual((pt, round(rl)), (-70, 180))

    def test_gap_moved_nearer_before_closing_inward(self):
        cam = self.camera(lambda r, pt: r <= 0.29)
        pt, rl, _al, r = cam.grasp_outward(0.31, -0.12, -60, (1, 0, 0))
        self.assertEqual(round(rl), 180)
        self.assertAlmostEqual(r, 0.28)

    def test_inward_only_when_nothing_else(self):
        cam = self.camera(lambda r, pt: False)
        _pt, rl, _al, r = cam.grasp_outward(0.31, -0.12, -60, (1, 0, 0))
        self.assertNotEqual(round(rl), 180)
        self.assertAlmostEqual(r, 0.31)

    def test_nothing_at_all_is_unreachable(self):
        cam = self.camera(lambda r, pt: False)
        cam.feasible = lambda *a, **k: False
        with self.assertRaises(Failed) as ctx:
            cam.grasp_outward(0.31, -0.12, -60, (1, 0, 0))
        self.assertEqual(ctx.exception.code, "UNREACHABLE")


class HoldCheckFromTheSide(unittest.TestCase):
    START = {"x0": 220.0, "x1": 448.0, "y0": 410.0}

    def test_spot_left(self):
        self.assertFalse(bc.spot_left(self.START, {"x0": 225.0, "x1": 450.0, "y0": 402.0}))   # still there
        self.assertTrue(bc.spot_left(self.START, None))                                       # place empty
        self.assertTrue(bc.spot_left(self.START, {"x0": 500.0, "x1": 640.0, "y0": 300.0}))   # seen elsewhere
        self.assertTrue(bc.spot_left(self.START, {"x0": 230.0, "x1": 440.0, "y0": 330.0}))   # up in the picture

    def check(self, frame, blob, in_view=False):
        cam = bc.CameraBag(NS(kin=NS(heading0=0), joints={}))
        cam.p = lambda key: "calibration"
        cam.in_view = lambda *a: in_view
        cam.web = lambda: frame
        cam.vision_model = lambda: ("coco", None)
        load, sleep, find = bc.FrontCamera.__dict__["load"], bc.time.sleep, bc.bv.find_bag
        bc.FrontCamera.load = classmethod(lambda cls, path: object())
        bc.time.sleep = lambda _: None
        bc.bv.find_bag = lambda im, **k: blob
        try:
            return cam.place_left(self.START)
        finally:
            bc.FrontCamera.load, bc.time.sleep, bc.bv.find_bag = load, sleep, find

    def test_no_frame_is_no_evidence(self):
        self.assertIsNone(self.check(None, None))

    def test_arm_still_in_the_picture_cannot_tell(self):
        self.assertIsNone(self.check(object(), None, in_view=True))

    def test_place_empty_and_bag_still_there(self):
        self.assertTrue(self.check(object(), None))
        self.assertFalse(self.check(object(), {"x0": 222.0, "x1": 446.0, "y0": 405.0}))


if __name__ == "__main__":
    unittest.main()
