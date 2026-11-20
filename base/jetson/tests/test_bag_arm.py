"""bag_arm: the Help Me Carry arm side over a fake arm (no ROS)."""
import io
import json
import math
import os
import tempfile
import signal
import socket
import subprocess
import sys
import threading
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bag_arm  # noqa: E402
import console_gate  # noqa: E402
import floor_pick  # noqa: E402
from floor_pick import JAW_OPEN, READY  # noqa: E402
from so100_ik import Kinematics, Mount  # noqa: E402
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


def make(arm, camera=None, detector=None, probe=False):
    return bag_arm.BagArm(KIN, arm, camera, detector, log=lambda _s: None, probe=probe)


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

    def test_an_unknown_top_is_found_by_touch_by_default(self):
        """Nothing measured the top and no gripper camera: z NaN, so their pick_bag probes (arm-system def69dc)."""
        self.assertTrue(bag_arm.PROBE_UNKNOWN)
        arm = ApiArm()
        res = bag_arm.BagArm(KIN, arm, log=lambda _s: None).pick(0.31, 0.0)
        self.assertTrue(res.ok, res)
        grasp = np.array(arm.tasks[-1][1]["grasp"])
        self.assertTrue(math.isnan(grasp[2]))
        np.testing.assert_allclose(grasp[:2], KIN.mount.to_arm([0.31, 0.0, 0.0])[:2], atol=1e-9)

    def test_a_measured_top_never_probes(self):
        arm = ApiArm()
        bag_arm.BagArm(KIN, arm, log=lambda _s: None).pick(0.31, 0.0, top=0.30)
        self.assertAlmostEqual(arm.tasks[-1][1]["grasp"][2], 0.30 - 0.331, places=6)

    def test_the_top_goes_in_the_arm_frame_of_their_floor_not_our_mount_offset(self):
        # their launch floor_z for mount:=robot is -0.331 (arm.launch.py); our Mount's z_offset (camera work)
        # must not shift the height we send
        kin = Kinematics(Mount(z_offset=0.0), KIN.limits)
        arm = ApiArm()
        bag_arm.BagArm(kin, arm, log=lambda _s: None).pick(0.31, 0.0, top=0.30)
        grasp = np.array(arm.tasks[-1][1]["grasp"])
        self.assertAlmostEqual(bag_arm.ARM_FLOOR_Z, -0.331)
        self.assertAlmostEqual(grasp[2], 0.30 - 0.331, places=6)
        np.testing.assert_allclose(grasp[:2], kin.mount.to_arm([0.31, 0.0, 0.0])[:2], atol=1e-9)

    def test_the_grasp_reading_goes_into_the_event(self):
        arm = ApiArm(results={"pick_bag": (True, "SUCCESS", "holding", {"gripper_pos": 0.123, "gripper_load": 25.0})})
        make(arm).pick(0.31, 0.0, top=0.30)
        self.assertEqual(arm.sent, ["grab_success|SUCCESS|gripper_pos=0.123 gripper_load=25.0"])

    def test_an_unverified_hold_stays_a_grab_and_names_the_code(self):
        arm = ApiArm(results={"pick_bag": (True, "SUCCESS_UNVERIFIED", "closed, camera did not confirm",
                                           {"gripper_pos": 0.1, "gripper_load": 10.0})})
        res = make(arm).pick(0.31, 0.0, top=0.30)
        self.assertTrue(res.ok)
        self.assertEqual(res.code, "SUCCESS_UNVERIFIED")
        self.assertEqual(arm.sent, ["grab_success|SUCCESS_UNVERIFIED|gripper_pos=0.100 gripper_load=10.0"])
        arm = ApiArm(results={"pick_bag": (False, "MISSED", "closed on air 3 times",
                                           {"gripper_pos": 0.041, "gripper_load": 3.0})})
        make(arm).pick(0.31, 0.0, top=0.30)
        self.assertEqual(arm.sent, ["grab_failed|MISSED|/arm/pick_bag: closed on air 3 times "
                                    "[gripper_pos=0.041 gripper_load=3.0]"])
        arm = ApiArm(results={"pick_bag": (True, "SUCCESS", "holding", {"gripper_pos": float("nan"),
                                                                         "gripper_load": float("nan")})})
        make(arm).pick(0.31, 0.0, top=0.30)
        self.assertEqual(arm.sent, ["grab_success"])        # no grasp reading: the plain event

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


class YawTest(unittest.TestCase):
    """The bag's long axis comes in the robot frame (base/pi/bag.py); the arm team's pick_bag wants it in the arm
    frame, where forward is -y and left is +x (API.md)."""

    def test_robot_directions_in_the_arm_frame(self):
        m = Mount()
        self.assertAlmostEqual(bag_arm.arm_yaw(math.pi / 2, m), 0.0)  # across the robot = along the arm's +x (left)
        self.assertAlmostEqual(bag_arm.arm_yaw(0.0, m), -math.pi / 2)  # along the robot's forward = the arm's -y
        self.assertAlmostEqual(bag_arm.arm_yaw(0.3, m), 0.3 - math.pi / 2)
        self.assertAlmostEqual(bag_arm.arm_yaw(0.3 + math.pi, m), 0.3 - math.pi / 2)  # a line: no front or back

    def test_it_matches_the_mount_for_any_direction_and_mount_yaw(self):
        for mount_yaw in (0.0, 0.2, -0.5):
            m = Mount(yaw=mount_yaw)
            for phi in np.linspace(-3.0, 3.0, 13):
                p = np.array([0.31, 0.02, 0.0])
                d = m.to_arm(p + 0.1 * np.array([math.cos(phi), math.sin(phi), 0.0])) - m.to_arm(p)
                want = math.atan2(d[1], d[0])
                got = bag_arm.arm_yaw(phi, m)
                self.assertAlmostEqual(math.sin(2 * (got - want)), 0.0, places=9)
                self.assertTrue(-math.pi / 2 <= got < math.pi / 2)

    def test_their_default_is_across_the_line_from_the_arm(self):
        # all-zero quaternion: "the part runs across the arm's view"; for a bag straight ahead that is the arm's
        # x axis (yaw 0 in the arm frame), the same as a measured yaw across the robot
        pan = (0.0, -0.0452)
        target = Mount().to_arm([0.31, 0.0145, 0.0])
        default = math.atan2(target[1] - pan[1], target[0] - pan[0]) + math.pi / 2
        self.assertAlmostEqual(math.sin(2 * (default - bag_arm.arm_yaw(math.pi / 2, Mount()))), 0.0, places=6)

    def test_pick_sends_the_arm_frame_yaw_and_none_without_one(self):
        arm = ApiArm()
        make(arm).pick(0.31, 0.0, top=0.28, yaw=0.3)
        self.assertAlmostEqual(arm.tasks[-1][1]["yaw"], 0.3 - math.pi / 2)
        arm = ApiArm()
        make(arm).pick(0.31, 0.0, top=0.28)
        self.assertIsNone(arm.tasks[-1][1]["yaw"])

    def test_the_command_carries_top_and_yaw(self):
        arm = ApiArm()
        jobs = bag_arm.Jobs(make(arm), log=lambda _s: None)
        jobs.on_command(json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0, "top": 0.29, "yaw": 1.0}}))
        jobs.thread.join(10)
        goal = arm.tasks[-1][1]
        self.assertAlmostEqual(goal["grasp"][2], 0.29 - 0.331, places=6)
        self.assertAlmostEqual(goal["yaw"], 1.0 - math.pi / 2)
        arm = ApiArm()
        jobs = bag_arm.Jobs(make(arm), log=lambda _s: None)
        jobs.on_command(json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0, "yaw": "x"}}))
        if jobs.thread is not None:
            jobs.thread.join(10)
        self.assertEqual(arm.sent[0].split("|")[:2], ["grab_failed", "FAILED"])

    def test_bag_miss_below_and_the_default_top_stay(self):
        self.assertEqual(bag_arm.DEFAULT_TOP, 0.26)
        self.assertFalse(hasattr(bag_arm, "bag_miss_below"))


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


# the real limits on 2026-11-16 (deg), as the robot description gives them (rad)
LIMITS_DEG = {"shoulder_pan": (-121, 120), "shoulder_lift": (1, 200), "elbow_flex": (-191, -2),
              "wrist_flex": (-144, 32), "wrist_roll": (-85, 271), "gripper": (0, 125)}
URDF = "<robot name='so'>%s<joint name='fixed_base' type='fixed'/></robot>" % "".join(
    "<joint name='%s' type='revolute'><limit lower='%r' upper='%r'/></joint>" % (n, math.radians(lo), math.radians(hi))
    for n, (lo, hi) in LIMITS_DEG.items())


class FakeApi:
    """ArmCommands' api without ROS: records what it was asked."""

    def __init__(self, state="idle", rest_after=120.0):
        self.calls = []
        self.params = {"rest_after_s": rest_after, "robot_description": URDF}
        self.arm_state = {"state": state, "holding": False, "gripper_load": float("nan")}
        self.results = {}
        self.gate = None  # a threading.Event the move waits for

    def task(self, name, timeout):
        self.calls.append(name)
        return self.results.get(name, (True, "SUCCESS", "done"))

    def stop(self):
        self.calls.append("stop")

    def torque(self, on):
        self.calls.append(("torque", on))
        return True, "ok"

    def state(self):
        return dict(self.arm_state)

    def joints(self):
        return {"shoulder_pan": 0.1, "gripper": float("nan")}

    def limits(self):
        return bag_arm.urdf_limits(self.params["robot_description"])

    def move(self, q, timeout):
        self.calls.append(("move", q))
        if self.gate is not None:
            self.gate.wait(10)
        return self.results.get("move", (True, "MoveIt code 1"))

    def grip(self, position, timeout):
        self.calls.append(("grip", position))
        return True, "position"

    def get_param(self, name):
        return self.params[name]

    def set_param(self, name, value):
        self.calls.append(("set", name, value))
        self.params[name] = value


class ArmCommandsTest(unittest.TestCase):
    def setUp(self):
        self.arm = ApiArm()
        self.jobs = bag_arm.Jobs(make(self.arm), log=lambda _s: None)
        self.api = FakeApi()
        self.cmds = bag_arm.ArmCommands(self.jobs, self.api, log=lambda _s: None)
        self.jobs.before = self.cmds.release

    def send(self, cmd):
        return self.cmds.handle(json.dumps(cmd))

    def test_mid_pose_is_the_middle_of_each_limit(self):
        q = bag_arm.mid_pose(bag_arm.urdf_limits(URDF))
        want = {"shoulder_pan": -0.5, "shoulder_lift": 100.5, "elbow_flex": -96.5, "wrist_flex": -56.0,
                "wrist_roll": 93.0, "gripper": 62.5}
        self.assertEqual(set(q), set(want))
        for n, deg in want.items():
            self.assertAlmostEqual(math.degrees(q[n]), deg, places=6)
        self.assertEqual(bag_arm.mid_pose({"a": (0.0, 1.0), **{n: (-1.0, 3.0) for n in LIMITS_DEG}})["gripper"], 1.0)
        with self.assertRaises(ValueError):
            bag_arm.mid_pose({"shoulder_pan": (0.0, 1.0)})

    def test_center_homes_moves_the_arm_then_the_gripper_and_holds_the_pose(self):
        out = self.send({"cmd": "center"})
        self.assertTrue(out["ok"], out)
        self.assertEqual([c if isinstance(c, str) else c[0] for c in self.api.calls], ["home", "move", "grip", "set"])
        q = self.api.calls[1][1]
        self.assertEqual(set(q), set(bag_arm.ARM))   # the gripper goes by its own action
        self.assertAlmostEqual(math.degrees(q["shoulder_lift"]), 100.5)
        self.assertAlmostEqual(math.degrees(self.api.calls[2][1]), 62.5)
        self.assertEqual(self.api.params["rest_after_s"], 0.0)
        self.assertEqual(self.cmds.saved_rest, 120.0)

    def test_the_saved_rest_after_comes_back_on_rest_home_and_a_bag_job(self):
        self.send({"cmd": "center"})
        self.send({"cmd": "center"})                      # a second hold keeps the first saved value
        self.assertEqual(self.cmds.saved_rest, 120.0)
        self.assertTrue(self.send({"cmd": "rest"})["ok"])
        self.assertEqual(self.api.params["rest_after_s"], 120.0)
        self.assertIsNone(self.cmds.saved_rest)
        self.assertEqual(self.api.calls[-1], "rest")      # put back before it moves
        self.send({"cmd": "torque", "on": True})
        self.assertEqual(self.api.params["rest_after_s"], 0.0)
        self.send({"cmd": "home"})
        self.assertEqual(self.api.params["rest_after_s"], 120.0)
        self.send({"cmd": "center"})
        self.jobs.on_command(json.dumps({"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}))
        self.jobs.thread.join(10)
        self.assertEqual(self.api.params["rest_after_s"], 120.0)
        self.assertEqual(self.arm.sent, ["drop_done"])
        n = len(self.api.calls)
        self.send({"cmd": "rest"})                        # nothing held: nothing set
        self.assertEqual(self.api.calls[n:], ["rest"])

    def test_a_failed_center_does_not_hold(self):
        self.api.results["move"] = (False, "MoveIt code 99999")
        out = self.send({"cmd": "center"})
        self.assertFalse(out["ok"])
        self.assertIn("99999", out["error"])
        self.assertEqual(self.api.params["rest_after_s"], 120.0)
        self.api.results = {"home": (False, "BUSY", "busy")}
        out = self.send({"cmd": "center"})
        self.assertEqual(out["code"], "BUSY")
        self.assertEqual(self.api.calls[-1], "home")       # nothing moved after a refused home

    def test_moves_are_refused_while_a_bag_job_runs_but_stop_and_state_answer(self):
        gate = threading.Event()

        def slow(name, timeout, **goal):
            gate.wait(10)
            return True, "SUCCESS", ""
        self.arm.arm_task = slow
        self.jobs.on_command(json.dumps({"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}))
        for cmd in ({"cmd": "home"}, {"cmd": "rest"}, {"cmd": "center"}, {"cmd": "torque", "on": True}):
            out = self.send(cmd)
            self.assertFalse(out["ok"])
            self.assertIn("bag job", out["error"])
        self.assertEqual(self.api.calls, [])
        state = self.send({"cmd": "state"})
        self.assertTrue(state["ok"] and state["busy"])
        self.assertTrue(self.send({"cmd": "stop"})["ok"])
        self.assertEqual(self.api.calls, ["stop"])
        gate.set()
        self.jobs.thread.join(10)
        self.assertFalse(self.send({"cmd": "state"})["busy"])

    def test_a_bag_job_is_refused_while_the_console_moves_and_stop_ends_a_center(self):
        self.api.gate = threading.Event()
        out = []
        th = threading.Thread(target=lambda: out.append(self.send({"cmd": "center"})))
        th.start()
        self.assertTrue(wait_until(lambda: any(isinstance(c, tuple) and c[0] == "move" for c in self.api.calls)))
        self.assertTrue(self.send({"cmd": "state"})["busy"])
        self.assertIn("busy with center", self.send({"cmd": "home"})["error"])
        self.jobs.on_command(json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0}}))
        self.assertEqual(self.arm.sent, ["grab_failed|BUSY|the console is moving the arm (center)"])
        self.send({"cmd": "stop"})
        self.api.gate.set()
        th.join(10)
        self.assertEqual(out[0]["code"], "STOPPED")       # the gripper never moved
        self.assertNotIn("grip", [c[0] for c in self.api.calls if isinstance(c, tuple)])
        self.assertEqual(self.api.params["rest_after_s"], 120.0)

    def test_pick_and_drop_from_the_console_answer_when_the_bag_job_is_done(self):
        self.arm.results["pick_bag"] = (True, "SUCCESS", "picked", {"gripper_pos": 0.5, "gripper_load": 30.0})
        out = self.send({"cmd": "pick_bag", "x": 0.31, "y": 0.0, "top": 0.27, "yaw": 0.0})
        self.assertEqual((out["ok"], out["code"], out["reading"]), (True, "SUCCESS", "gripper_pos=0.500 gripper_load=30.0"))
        self.assertEqual([t[0] for t in self.arm.tasks], ["home", "pick_bag"])
        self.assertTrue(math.isfinite(self.arm.tasks[1][1]["grasp"][2]))  # the top given is used
        self.assertTrue(self.send({"cmd": "drop_bag", "x": 0.31, "y": 0.0})["ok"])
        self.assertEqual(self.arm.sent, ["grab_success|SUCCESS|gripper_pos=0.500 gripper_load=30.0", "drop_done"])
        self.arm.results["pick_bag"] = (False, "MISSED", "closed on air")
        out = self.send({"cmd": "pick_bag", "x": 0.31, "y": 0.0, "top": 0.27})
        self.assertEqual((out["ok"], out["code"]), (False, "MISSED"))
        self.assertIn("closed on air", out["error"])
        out = self.send({"cmd": "drop_bag", "x": 0.31})
        self.assertEqual((out["ok"], out["code"]), (False, "FAILED"))
        self.assertIn("needs x and y", out["error"])
        self.assertFalse(self.send({"cmd": "state"})["busy"])

    def test_one_bag_job_at_a_time_from_the_console_or_the_flow(self):
        gate = threading.Event()

        def slow(name, timeout, **goal):
            self.arm.tasks.append((name, goal))
            gate.wait(10)
            return True, "SUCCESS", ""
        self.arm.arm_task = slow
        self.jobs.on_command(json.dumps({"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}))
        out = self.send({"cmd": "pick_bag", "x": 0.31, "y": 0.0})
        self.assertEqual((out["ok"], out["code"]), (False, "BUSY"))
        gate.set()
        self.jobs.thread.join(10)
        gate.clear()
        got = []
        th = threading.Thread(target=lambda: got.append(self.send({"cmd": "drop_bag", "x": 0.31, "y": 0.0})))
        th.start()
        self.assertTrue(wait_until(lambda: len(self.arm.tasks) == 2))
        self.assertTrue(self.send({"cmd": "state"})["busy"])
        self.assertIn("bag job", self.send({"cmd": "home"})["error"])
        self.jobs.on_command(json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0}}))
        self.assertEqual(self.arm.sent[-1], "grab_failed|BUSY|another arm job is running")
        gate.set()
        th.join(10)
        self.assertTrue(got[0]["ok"], got)

    def test_torque_off_only_at_rest(self):
        for st in ("idle", "busy", "holding", "torque_off"):
            self.api.arm_state["state"] = st
            out = self.send({"cmd": "torque", "on": False})
            self.assertFalse(out["ok"])
            self.assertIn("only at rest", out["error"])
        self.assertEqual(self.api.calls, [])
        self.api.arm_state["state"] = "resting"
        self.assertTrue(self.send({"cmd": "torque", "on": False})["ok"])
        self.assertEqual(self.api.calls, [("torque", False)])
        self.assertIsNone(self.cmds.saved_rest)             # torque off holds nothing

    def test_state_and_bad_commands(self):
        out = self.send({"cmd": "state"})
        self.assertEqual(out["state"]["state"], "idle")
        self.assertIsNone(out["state"]["gripper_load"])      # NaN is no JSON
        self.assertEqual(out["joints"], {"shoulder_pan": 0.1, "gripper": None})
        self.assertFalse(out["busy"])
        json.loads(json.dumps(out, allow_nan=False))
        self.assertIn("one of", self.send({"cmd": "dance"})["error"])
        self.assertIn("torque needs", self.send({"cmd": "torque"})["error"])
        self.assertEqual(self.cmds.handle(b"not json")["error"], "not JSON")
        self.assertFalse(self.cmds.handle("[1]")["ok"])
        self.assertEqual(self.api.calls, [])

    def test_the_tcp_server_answers_line_by_line(self):
        server = bag_arm.serve_commands(self.cmds, port=0)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with socket.create_connection(server.server_address, 5) as s:
            f = s.makefile("rwb")
            f.write(b'{"cmd": "state"}\n{"cmd": "center"}\n')
            f.flush()
            self.assertTrue(json.loads(f.readline())["ok"])
            self.assertTrue(json.loads(f.readline())["ok"])
        self.assertEqual(bag_arm.ARM_CMD_PORT, 5607)


class FakeProc:
    """bagpick.py's process: its output lines, then its end (hang: only on a signal, after the first `ignore`)."""

    def __init__(self, lines=(), code=0, hang=False, ignore=0):
        self.pid, self.code, self.ignore, self.signals = 4242, code, ignore, []
        self.ended = threading.Event()
        if not hang:
            self.ended.set()
        self.stdout = self._out([ln + "\n" for ln in lines])

    def _out(self, lines):
        yield from lines
        self.ended.wait(10)

    def wait(self, timeout=None):
        if not self.ended.wait(timeout):
            raise subprocess.TimeoutExpired("bagpick.py", timeout)
        return self.code

    def signal(self, sig):
        self.signals.append(sig)
        if len(self.signals) > self.ignore:
            self.code = -int(sig)
            self.ended.set()


HOLDING = 'RESULT holding the bag {"left": 0.0, "tips": 0.33}'


class BagpickTest(unittest.TestCase):
    def setUp(self):
        self.arm = ApiArm()
        self.jobs = bag_arm.Jobs(make(self.arm), log=lambda _s: None)
        self.api = FakeApi()
        self.proc, self.started = FakeProc([HOLDING]), []

        def popen(argv, **kw):
            self.started.append((argv, kw))
            if isinstance(self.proc, OSError):
                raise self.proc
            return self.proc
        self.cmds = bag_arm.ArmCommands(self.jobs, self.api, log=lambda _s: None, popen=popen,
                                        signal_group=lambda proc, sig: proc.signal(sig))

    def send(self, cmd):
        return self.cmds.handle(json.dumps(cmd))

    def run_hanging(self, cmd=None, **kw):
        """A bagpick on a thread that runs until a signal ends it; the list gets its answer."""
        self.proc, out = FakeProc(["scan left +0.00: bag area 900"], hang=True, **kw), []
        th = threading.Thread(target=lambda: out.append(self.send(cmd or {"cmd": "bagpick"})), daemon=True)
        th.start()
        self.assertTrue(wait_until(lambda: self.cmds.bagpick is not None))
        return th, out

    def test_the_result_line_gives_ok_and_the_code(self):
        cases = [(HOLDING, True, "HOLDING_BAG"),
                 ('RESULT holding the bag (lift NOT verified by the Xtion) {"left": 0.1}', True, "HOLDING_BAG"),
                 ("RESULT missed 4 times (bag not off the floor): it may be too far", False, "MISSED"),
                 ("RESULT too far: the bag is beyond reach of the arm, move the robot closer", False, "TOO_FAR"),
                 ("RESULT no bag in view", False, "NO_BAG"),
                 ("RESULT lost the bag after a try (knocked over or pushed away?)", False, "LOST"),
                 ("RESULT dancing", False, "FAILED")]
        for line, ok, code in cases:
            self.assertEqual(bag_arm.bagpick_result(["try 1: bearing +0.0 deg", line], 0), (ok, code, line), line)
        self.assertEqual(bag_arm.bagpick_result([HOLDING, "put back"], 0), (True, "HOLDING_BAG", HOLDING))  # --putback
        ok, code, msg = bag_arm.bagpick_result(["Traceback (most recent call last):", "RuntimeError: no webcam frame"], 1)
        self.assertEqual((ok, code), (False, "FAILED"))
        self.assertIn("exit 1", msg)
        self.assertIn("no webcam frame", msg)
        self.assertEqual(bag_arm.bagpick_result([], None)[:2], (False, "FAILED"))
        self.assertEqual(bag_arm.bagpick_result(["  RESULT holding (indented, not its line)"], 0)[:2], (False, "FAILED"))
        # robot 2026-11-16: the bag fell while raising, yet bagpick.py still printed "RESULT holding"
        dropped = ["high check: floor under the bag 0.25  -> DROPPED while raising", HOLDING]
        self.assertEqual(bag_arm.bagpick_result(dropped, 0)[:2], (False, "DROPPED"))
        # and its put-back crashed after the RESULT line (the arm was taken): not a clean hold
        ok, code, msg = bag_arm.bagpick_result([HOLDING, "Traceback (most recent call last):",
                                                "RuntimeError: move failed: arm busy: another program has it"], 1)
        self.assertEqual((ok, code), (False, "FAILED"))
        self.assertIn("arm busy", msg)

    def test_it_runs_bagpick_in_the_arm_workspace_with_this_environment(self):
        self.proc = FakeProc(["line %d" % i for i in range(25)] + [HOLDING, "put back"])
        out = self.send({"cmd": "bagpick", "putback": True})
        self.assertEqual((out["ok"], out["code"], out["message"], out["error"]), (True, "HOLDING_BAG", HOLDING, None))
        self.assertEqual(len(out["tail"]), bag_arm.BAGPICK_TAIL)
        self.assertEqual(out["tail"][-2:], [HOLDING, "put back"])
        argv, kw = self.started[0]
        self.assertEqual(argv, ["python3", "bagpick.py", "--putback"])
        self.assertEqual(kw["cwd"], os.path.expanduser(bag_arm.BAGPICK_DIR))
        self.assertEqual((kw["stderr"], kw["start_new_session"]), (subprocess.STDOUT, True))  # RESULT is on stderr
        self.assertNotIn("env", kw)                    # HMC_CONSOLE_JOB from the agent passes its gate
        self.proc = FakeProc(["RESULT no bag in view"], code=1)
        out = self.send({"cmd": "bagpick"})
        self.assertEqual(self.started[1][0], ["python3", "bagpick.py"])
        self.assertEqual((out["ok"], out["code"]), (False, "NO_BAG"))
        self.assertIn("RESULT no bag in view", out["error"])
        self.assertFalse(self.send({"cmd": "state"})["busy"])
        # their auto rest (120 s idle) took the arm mid-bagpick on the robot: off while it runs, then back
        sets = [c for c in self.api.calls if c[0] == "set"]
        self.assertEqual(sets, [("set", "rest_after_s", 0.0), ("set", "rest_after_s", 120.0)] * 2)
        self.assertIsNone(self.cmds.saved_rest)

    def test_bad_putback_and_a_missing_workspace(self):
        self.assertIn("putback", self.send({"cmd": "bagpick", "putback": "yes"})["error"])
        self.proc = FileNotFoundError("no such directory")
        out = self.send({"cmd": "bagpick"})
        self.assertEqual((out["ok"], out["code"]), (False, "FAILED"))
        self.assertIn("not started", out["error"])
        self.assertEqual(len(self.started), 1)
        self.assertIsNone(self.jobs.console)           # the claim is given back

    def test_a_bag_job_and_bagpick_are_busy_both_ways_and_stop_ends_it(self):
        th, out = self.run_hanging()
        self.assertTrue(self.send({"cmd": "state"})["busy"])
        self.assertEqual(self.send({"cmd": "pick_bag", "x": 0.31, "y": 0.0})["code"], "BUSY")
        self.jobs.on_command(json.dumps({"action": "pick_bag", "params": {"x": 0.31, "y": 0.0}}))
        self.assertEqual(self.arm.sent, ["grab_failed|BUSY|the console is moving the arm (bagpick)"])
        self.assertIn("busy with bagpick", self.send({"cmd": "home"})["error"])
        self.assertEqual(self.send({"cmd": "bagpick"})["code"], "BUSY")
        self.assertTrue(self.send({"cmd": "stop"})["ok"])
        th.join(10)
        self.assertEqual(self.proc.signals, [signal.SIGINT])
        self.assertEqual(self.api.calls, [("set", "rest_after_s", 0.0), "stop",     # and /arm/stop, as before
                                          ("set", "rest_after_s", 120.0)])          # their auto rest back after
        self.assertEqual((out[0]["ok"], out[0]["code"]), (False, "STOPPED"))
        self.assertEqual(out[0]["tail"], ["scan left +0.00: bag area 900"])
        self.assertFalse(self.send({"cmd": "state"})["busy"])
        self.assertEqual(len(self.started), 1)

        gate = threading.Event()

        def slow(name, timeout, **goal):
            gate.wait(10)
            return True, "SUCCESS", ""
        self.arm.arm_task = slow
        self.jobs.on_command(json.dumps({"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}))
        out = self.send({"cmd": "bagpick"})
        self.assertEqual((out["ok"], out["code"]), (False, "BUSY"))
        self.assertIn("bag job", out["error"])
        self.assertEqual(len(self.started), 1)         # never started
        gate.set()
        self.jobs.thread.join(10)

    def test_stop_goes_on_to_sigterm_and_sigkill(self):
        with mock.patch.object(bag_arm, "BAGPICK_STOP_STEPS", ((signal.SIGINT, 0.05), (signal.SIGTERM, 0.05))):
            th, out = self.run_hanging(ignore=2)
            self.send({"cmd": "stop"})
            th.join(10)
        self.assertEqual(self.proc.signals, [signal.SIGINT, signal.SIGTERM, bag_arm.KILL])
        self.assertEqual(out[0]["code"], "STOPPED")

    def test_a_run_past_its_timeout_is_killed(self):
        with mock.patch.object(bag_arm, "BAGPICK_TIMEOUT", 0.2):
            th, out = self.run_hanging()
            th.join(10)
        self.assertEqual(self.proc.signals, [bag_arm.KILL])
        self.assertEqual((out[0]["ok"], out[0]["code"]), (False, "FAILED"))
        self.assertIn("longer than", out[0]["error"])
        self.assertEqual(bag_arm.BAGPICK_TIMEOUT, 300.0)
        self.assertEqual(bag_arm.BAGPICK_DIR, "~/so100_ws")


class GateTest(unittest.TestCase):
    def setUp(self):
        self.env = mock.patch.dict(os.environ)
        self.env.start()
        for name in (console_gate.ENV, console_gate.MAINTENANCE_ENV):
            os.environ.pop(name, None)

    def tearDown(self):
        self.env.stop()

    def test_a_pick_or_place_from_the_shell_is_refused(self):
        with mock.patch.object(floor_pick, "RosArm", side_effect=AssertionError("moved")):
            for argv in (["--pick", "0.31", "0.0"], ["--place", "0.31", "0.0"]):
                with self.assertRaisesRegex(RuntimeError, "REFUSED: arm: bag_arm.py"):
                    bag_arm.main(argv)

    def test_the_agents_node_is_not_gated(self):
        class Reached(Exception):
            pass
        with mock.patch.object(floor_pick, "RosArm", side_effect=Reached), redirect_stdout(io.StringIO()):
            with self.assertRaises(Reached):  # past the gate, at the ROS transport
                bag_arm.main([])


class Blind:
    def detect(self, _frame):
        return None


class ProfileTest(unittest.TestCase):
    def test_a_calibrated_camera_that_sees_nothing_does_not_probe(self):
        arm = ApiArm()
        ba = bag_arm.BagArm(KIN, arm, true_camera(), Blind(), log=lambda _s: None)
        res = ba.pick(0.31, 0.0)
        self.assertTrue(res.ok, res)
        grasp = arm.tasks[-1][1]["grasp"]
        self.assertTrue(math.isfinite(grasp[2]))
        self.assertAlmostEqual(grasp[2], bag_arm.ARM_FLOOR_Z + bag_arm.DEFAULT_TOP, places=6)
        names = [row["phase"] for row in ba.last_pick["phases"]]
        self.assertIn("home", names)
        self.assertIn("pick_bag", names)
        self.assertFalse(ba.last_pick["probe"])
        self.assertEqual(ba.last_pick["profile"], "safe")

    def test_fast_is_handed_to_the_arm_and_written_with_the_pick(self):
        class Profiling(ApiArm):
            def set_speed_profile(self, name):
                self.asked = name

        arm = Profiling()
        with tempfile.TemporaryDirectory() as folder:
            ba = bag_arm.BagArm(KIN, arm, log=lambda _s: None, profile="fast", pick_log_dir=folder)
            ba.pick(0.31, 0.0, top=0.30)
            self.assertEqual(arm.asked, "fast")
            self.assertEqual(ba.last_pick["speeds"]["bag_speed"], 0.12)
            self.assertEqual(ba.last_pick["speeds"]["joint_scale"], 0.45)
            written = os.listdir(folder)
            self.assertEqual(len(written), 1)
            self.assertTrue(written[0].endswith("-bag.json"))


def wait_until(check, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.01)
    return check()


if __name__ == "__main__":
    unittest.main()


class LookBagsTest(unittest.TestCase):
    """look_bags: every bag in the front camera for the flow's 'which one?', without moving the arm."""

    def run_look(self, look):
        arm = ApiArm()
        jobs = bag_arm.Jobs(make(arm), log=lambda _s: None, look=look)
        jobs.on_command(json.dumps({"action": "look_bags", "params": {}}))
        if getattr(jobs, "look_thread", None) is not None:
            jobs.look_thread.join(10)
        return arm

    def test_the_bags_go_out_as_one_event_and_the_arm_stays(self):
        bags = [{"colour": "blue", "side": "left", "x": 0.3, "y": 0.1, "ok": True, "box": [1, 2, 3, 4], "why": ""}]
        arm = self.run_look(lambda: {"ok": True, "bags": bags})
        name, code, data = arm.sent[-1].split("|", 2)
        self.assertEqual((name, code), ("bags_seen", "OK"))
        self.assertEqual(json.loads(data), {"bags": [{"colour": "blue", "side": "left", "x": 0.3, "y": 0.1, "ok": True}]})
        self.assertEqual(arm.tasks, [])

    def test_a_failed_look_or_none_is_failed(self):
        self.assertEqual(self.run_look(lambda: {"ok": False, "why": "no frame | camera"}).sent[-1],
                         "bags_seen|FAILED|no frame / camera")
        self.assertTrue(self.run_look(lambda: 1 / 0).sent[-1].startswith("bags_seen|FAILED|"))
        self.assertTrue(self.run_look(None).sent[-1].startswith("bags_seen|FAILED|"))
