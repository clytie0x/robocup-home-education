"""bag_arm: the Help Me Carry arm side over a fake arm (no ROS)."""
import json
import math
import os
import sys
import time
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bag_arm  # noqa: E402
from floor_pick import JAW_OPEN, READY  # noqa: E402
from test_floor_pick import KIN, PLANNER, FakeArm, ProjectingDetector, true_camera  # noqa: E402


class ApiArm(FakeArm):
    """FakeArm plus the arm team's API tasks."""

    def __init__(self, results=None, holding=False, **kw):
        super().__init__(**kw)
        self.tasks = []
        self.results = dict(results or {})
        self.api_holding = holding
        self.sent = []

    def arm_state(self):
        st = super().arm_state()
        st["holding"] = self.api_holding
        return st

    def arm_task(self, name, timeout, **goal):
        self.tasks.append((name, goal))
        if name == "home":
            self.q.update(READY)
        return self.results.get(name, (True, "SUCCESS", "done"))

    def event(self, text):
        self.sent.append(text)


def make(arm, camera=None, detector=None):
    return bag_arm.BagArm(KIN, arm, camera, detector, log=lambda _s: None)


class PickTest(unittest.TestCase):
    def test_without_a_camera_it_homes_then_picks_at_the_default_top(self):
        arm = ApiArm()
        res = make(arm).pick(0.31, 0.0)
        self.assertTrue(res.ok, res)
        self.assertEqual([n for n, _g in arm.tasks], ["home", "pick_bag"])
        grasp = np.array(arm.tasks[1][1]["grasp"])
        np.testing.assert_allclose(grasp, KIN.mount.to_arm([0.31, 0.0, bag_arm.DEFAULT_TOP]), atol=1e-9)
        # arm frame: forward is -y from the pan axis (0, -0.0452), floor 0.331 below the arm base
        self.assertAlmostEqual(grasp[1], -0.0452 - (0.31 - 0.012), places=6)
        self.assertAlmostEqual(grasp[2], bag_arm.DEFAULT_TOP - 0.331, places=6)
        self.assertEqual(arm.sent, ["grab_success"])

    def test_the_camera_measures_the_top_and_the_grasp_moves_to_it(self):
        cam = true_camera()
        handle = np.array([0.325, 0.02, 0.33])
        arm = ApiArm()
        det = ProjectingDetector(arm, cam, handle, noise=0.3)
        res = make(arm, cam, det).pick(0.31, 0.0)
        self.assertTrue(res.ok, res)
        grasp = np.array(arm.tasks[-1][1]["grasp"])
        np.testing.assert_allclose(grasp, KIN.mount.to_arm(handle), atol=0.01)
        looks = [pts for pts, _t in arm.moves]
        self.assertGreaterEqual(len(looks), 3)        # at least 2 looks and back to ready
        for pts in looks:                             # never closer than LOOK_CLEAR over the bag
            for _t, q in pts:
                g = KIN.grasp_point(q)
                if math.hypot(g[0] - 0.31, g[1]) < 0.10:
                    self.assertGreater(g[2], bag_arm.DEFAULT_TOP + 0.05)

    def test_a_far_off_camera_point_is_not_trusted(self):
        cam = true_camera()
        arm = ApiArm()
        det = ProjectingDetector(arm, cam, np.array([0.45, 0.15, 0.30]), noise=0.3)   # another thing
        res = make(arm, cam, det).pick(0.31, 0.0)
        self.assertTrue(res.ok, res)
        np.testing.assert_allclose(arm.tasks[-1][1]["grasp"],
                                   KIN.mount.to_arm([0.31, 0.0, bag_arm.DEFAULT_TOP]), atol=1e-9)

    def test_a_given_top_skips_the_camera(self):
        arm = ApiArm()
        make(arm, true_camera(), ProjectingDetector(arm, true_camera(), (0.31, 0, 0.3))).pick(0.31, 0.0, top=0.30)
        self.assertEqual(arm.moves, [])
        self.assertAlmostEqual(arm.tasks[-1][1]["grasp"][2], 0.30 - 0.331, places=6)

    def test_their_failure_code_goes_out(self):
        arm = ApiArm(results={"pick_bag": (False, "MISSED", "closed on air 3 times")})
        res = make(arm).pick(0.31, 0.0)
        self.assertEqual(res.code, "MISSED")
        self.assertEqual(arm.sent, ["grab_failed|MISSED|/arm/pick_bag: closed on air 3 times"])

    def test_someone_else_moving_the_arm_refuses_before_anything(self):
        arm = ApiArm()
        arm.others_moving = True
        res = make(arm).pick(0.31, 0.0)
        self.assertEqual(res.code, "BUSY")
        self.assertEqual(arm.tasks, [])

    def test_holding_already_says_holding(self):
        arm = ApiArm(holding=True)
        res = make(arm).pick(0.31, 0.0)
        self.assertEqual(res.code, "HOLDING")
        self.assertEqual(arm.sent[0].split("|")[:2], ["grab_failed", "HOLDING"])

    def test_look_poses_exist_for_a_bag_in_the_pick_zone(self):
        poses = bag_arm.look_poses(PLANNER, KIN, (0.31, 0.0, 0.31))
        self.assertGreaterEqual(len(poses), 2)
        g = [KIN.grasp_point(q) for q in poses]
        self.assertGreater(max(np.linalg.norm(a - b) for a in g for b in g), 0.05)   # a stereo baseline


class PlaceTest(unittest.TestCase):
    def test_place_sends_the_floor_point_in_the_arm_frame(self):
        arm = ApiArm()
        res = make(arm).place(0.31, 0.0)
        self.assertTrue(res.ok, res)
        self.assertEqual(arm.tasks[0][0], "place")
        np.testing.assert_allclose(arm.tasks[0][1]["target"], KIN.mount.to_arm([0.31, 0.0, 0.0])[:2])
        self.assertEqual(arm.sent, ["drop_done"])

    def test_place_failure(self):
        arm = ApiArm(results={"place": (False, "NOT_HOLDING", "nothing held")})
        make(arm).place(0.31, 0.0)
        self.assertEqual(arm.sent, ["drop_failed|NOT_HOLDING|/arm/place: nothing held"])


class JobsTest(unittest.TestCase):
    def run_job(self, jobs, text):
        jobs.on_command(text)
        if jobs.thread is not None:
            jobs.thread.join(10)

    def test_pick_and_drop_commands_run_jobs(self):
        arm = ApiArm()
        jobs = bag_arm.Jobs(make(arm), log=lambda _s: None)
        self.run_job(jobs, json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0}}))
        self.run_job(jobs, json.dumps({"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}))
        self.assertEqual(arm.sent, ["grab_success", "drop_done"])

    def test_bad_params_and_other_actions(self):
        arm = ApiArm()
        jobs = bag_arm.Jobs(make(arm), log=lambda _s: None)
        self.run_job(jobs, json.dumps({"action": "pick_bag", "params": {"x": "a"}}))
        self.run_job(jobs, json.dumps({"action": "drop_bag", "params": {"x": float("nan"), "y": 0}}))
        self.run_job(jobs, json.dumps({"action": "follow_me", "params": {}}))
        self.run_job(jobs, "not json")
        self.assertEqual([s.split("|")[:2] for s in arm.sent], [["grab_failed", "FAILED"], ["drop_failed", "FAILED"]])
        self.assertEqual(arm.tasks, [])

    def test_a_second_job_while_busy_and_stop(self):
        arm = ApiArm()
        gate = []

        def slow(name, timeout, **goal):
            arm.tasks.append((name, goal))
            while not gate:
                time.sleep(0.01)
            return True, "SUCCESS", ""
        arm.arm_task = slow
        jobs = bag_arm.Jobs(make(arm), log=lambda _s: None)
        jobs.on_command(json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0}}))
        jobs.on_command(json.dumps({"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}))
        jobs.on_command(json.dumps({"action": "stop"}))
        gate.append(1)
        jobs.thread.join(10)
        self.assertEqual(arm.sent[0], "drop_failed|BUSY|another arm job is running")
        self.assertEqual(arm.stops, 1)


if __name__ == "__main__":
    unittest.main()
