"""Bag pick posture (bag_camera.py): the one-move carry, the grip depth, the outward close first and the hold check
from the carry side (simulation 2026-11-19/20, robot 2026-11-20). No ROS, no arm."""
import math
import os
import sys
import unittest

import numpy as np
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from so100_driver import bag_camera as bc  # noqa: E402
from so100_driver.task_rules import Failed  # noqa: E402


class CarryOneMove(unittest.TestCase):
    """The held move after the close (2026-11-20 simulation and robot): one move, the bag's bottom over the LiDAR,
    the least tilt, never dragged on the floor or swung into the deck."""

    @classmethod
    def setUpClass(cls):
        from test_bag_pick_continuous import FLOOR, Node
        from so100_driver.ik import deck_box
        cls.FLOOR = FLOOR
        cls.node = Node(bc.LOOK)
        cls.node.deck = deck_box(cls.node.kin, 0.18, 0.22, 0.18, 0.22, -0.068, FLOOR)
        cls.cam = bc.CameraBag(cls.node)
        cls.cam.theta = 0.0
        cls.cz = FLOOR + 0.22                      # tall paper bag: rim 0.27, 5 cm of wall in the jaws
        cls.close_q = {j: float(bc.LOOK[j]) for j in bc.ARM}
        cls.cam.tip(0.26, cls.cz, -60, 180.0, "", sim=cls.close_q)

    def test_one_move_to_the_left_side_with_the_bottom_over_the_lidar(self):
        path, theta, pitch, reach = self.cam.carry_plan(0.26, self.cz, -60, 180.0, sim=self.close_q)
        self.assertAlmostEqual(math.degrees(theta), bc.CARRY_SIDES[0])
        self.assertGreaterEqual(pitch, -60)                          # never steeper than the grasp
        self.assertLessEqual(abs(pitch - -60), 35)                   # a small tilt (robot: 25 deg)
        g = self.node.kin.gripper_point(path[-1], (bc.GAP_MID, bc.TIP_Y, 0.0))
        bottom = g[2] - self.cz                                      # tips minus the grip height, over the floor
        self.assertGreaterEqual(bottom, bc.LIDAR_Z + 0.02)
        self.assertEqual(self.cam.theta, 0.0)                        # the bearing of the bag is kept

    def test_dragging_and_the_deck_are_refused(self):
        grasp = self.node.kin.gripper_point(self.close_q, (bc.GAP_MID, bc.TIP_Y, 0.0))[:2]
        grip_h = self.cz - self.FLOOR
        self.assertTrue(self.cam.held_clear([self.close_q], grip_h, grasp))
        moved = grasp + [0.0, 0.05]                                  # the grip 5 cm away, the bottom still down
        self.assertFalse(self.cam.held_clear([self.close_q], grip_h, moved))
        # a deck under the whole hanging bag: a move from where it stood clear (high) into it is refused; a bag that
        # already stood there (the deck parameter is too wide) is not refused for staying there
        high = {j: float(bc.LOOK[j]) for j in bc.ARM}
        self.cam.tip(0.26, self.cz + 0.30, -15, 180.0, "", sim=high)   # bag bottom over the deck top
        real, self.node.deck = self.node.deck, (np.array([-1.0, -1.0, self.FLOOR]), np.array([1.0, 1.0, -0.068]))
        try:
            self.assertFalse(self.cam.held_clear([high, self.close_q], grip_h, grasp))
            self.assertTrue(self.cam.held_clear([self.close_q, self.close_q], grip_h, grasp))
        finally:
            self.node.deck = real

    def test_a_back_wall_grip_hangs_the_bag_towards_the_robot(self):
        # the same pose: the bag behind the wall (front grip) vs in front of it (back grip), a deck just in front of
        # the gripped wall: only the back grip puts the bag into it
        roll, _align = self.cam.roll_for(0.26, self.cz, -60, self.cam.user_dir((1.0, 0.0)))   # jaws across the wall
        close_q = {j: float(bc.LOOK[j]) for j in bc.ARM}
        self.cam.tip(0.26, self.cz, -60, roll, "", sim=close_q)
        g = self.node.kin.gripper_point(close_q, (bc.GAP_MID, bc.TIP_Y, 0.0))
        grip_h = self.cz - self.FLOOR
        high = {j: float(bc.LOOK[j]) for j in bc.ARM}
        self.cam.tip(0.26, self.cz + 0.30, -15, roll, "", sim=high)   # bag bottom over the deck top
        real = self.node.deck
        # base frame: the arm's forward is heading0; the deck covers everything nearer the pan axis than 2 cm in front
        # of the gripped wall
        fwd = np.array([math.cos(self.node.kin.heading0), math.sin(self.node.kin.heading0)])
        near = g[:2] - 0.02 * fwd
        lo = np.where(fwd > 0.5, -1.0, np.where(fwd < -0.5, near, -1.0))
        hi = np.where(fwd > 0.5, near, 1.0)
        self.node.deck = (np.array([*lo, self.FLOOR]), np.array([*hi, -0.068]))
        try:
            # side is along the closing axis, which a real grasp points away from the arm (grasp_outward); this
            # test pose's roll may point it back, so the sign of "away" comes from the pose
            a = self.node.kin.close_axis(close_q)[:2]
            away = 1.0 if float(a @ (g[:2] - self.node.kin.pan_point)) > 0 else -1.0
            self.assertTrue(self.cam.held_clear([high, close_q], grip_h, g[:2], away))
            self.assertFalse(self.cam.held_clear([high, close_q], grip_h, g[:2], -away))
        finally:
            self.node.deck = real

    def test_a_low_bag_must_be_nearer(self):
        self.assertAlmostEqual(bc.wall_reach(0.13), 0.24)
        self.assertAlmostEqual(bc.wall_reach(0.16), 0.29)
        self.assertAlmostEqual(bc.wall_reach(0.22), bc.REACH_WALL)
        self.assertAlmostEqual(bc.wall_reach(0.10), 0.24)

    def test_the_put_down_lets_go_with_the_bottom_on_the_floor(self):
        self.assertEqual(bc.PUT_DROP, 0.0)


class GripDepth(unittest.TestCase):
    def cam(self, depth):
        values = {"bag_cam_grip_depth": depth, "bag_cam_rim_h": 0.27, "bag_cam_close_h": 0.21}
        cam = bc.CameraBag(NS(kin=NS(heading0=0)))
        cam.p = lambda k: values[k]
        return cam

    def test_under_the_rim(self):
        self.assertAlmostEqual(self.cam(0.05).close_h(NS(size=(0.1, 0.1, float("nan")))), 0.22)
        self.assertAlmostEqual(self.cam(0.05).close_h(NS(size=(0.1, 0.1, 0.20))), 0.15)    # the Pi's top
        self.assertAlmostEqual(self.cam(0.0).close_h(NS(size=(0.1, 0.1, float("nan")))), 0.21)   # old fixed

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
