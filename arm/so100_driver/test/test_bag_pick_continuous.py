"""Continuous camera-pick plan, the LiDAR outlier gate, the rim height and the motor-temperature wait.

The plan is checked against tip(sim=): the same joint targets as the old per-step moves, sent as one
path down to the handle and one path for the lift. The gripper close is the only pause between them.
"""
import inspect
import unittest
from pathlib import Path

from so100_driver.bag_camera import (
    CARRY_THETA,
    DRIVER_TEMP_C,
    FREE_SPEED,
    HELD_SPEED,
    JOINT_STEP,
    LIFT_CONTINUOUS,
    LIFT_SPEED,
    LOOK,
    PATH_BOW,
    PICK_TEMP_C,
    REACH_WALL,
    CameraBag,
    bag_fallen,
    cool_reason,
    line_seconds,
    pick_result,
    pose_outlier,
    resolve_grasp,
    rim_height,
    tip_overshoot,
    wait_until_cool,
)
from so100_driver.ik import ARM, GraspPlanner, Kinematics
from so100_driver.task_rules import Failed

CONFIG = Path(__file__).resolve().parents[2] / "so100_moveit_config"
URDF = (CONFIG / "urdf" / "so100.urdf").read_text()
MESHES = str(CONFIG / "meshes")
REAL = {"shoulder_pan": (-2.11, 1.90), "shoulder_lift": (0.0, 3.50), "elbow_flex": (-3.34, 0.0),
        "wrist_flex": (-2.52, 0.56), "wrist_roll": (-1.48, 4.74)}
# Floor in the arm frame when the arm is mounted on the robot (launch floor_z).
FLOOR = -0.331


class Node:
    """Joints, a planner and a line() that records one call. No ROS."""

    def __init__(self, joints):
        self.joints = {j: float(joints[j]) for j in ARM}
        self.kin = Kinematics(URDF, REAL, MESHES)
        self.planner = GraspPlanner(self.kin, FLOOR, [])
        self.table_z = FLOOR
        self.calls = []

    def get_parameter(self, _name):
        raise KeyError("no override")

    def line(self, qs, what, speed, step):
        self.calls.append((what, qs, speed, step))
        self.joints = {j: float(qs[-1][j]) for j in ARM}

    def say(self, _text):
        pass

    def get_logger(self):
        class Log:
            def info(self, _text):
                pass

            def warning(self, _text):
                pass

        return Log()


def same_path(a, b):
    if len(a) != len(b):
        return False
    for left, right in zip(a, b):
        for joint in ARM:
            if abs(float(left[joint]) - float(right[joint])) > 1e-9:
                return False
    return True


class Gates(unittest.TestCase):
    def test_rim_height_uses_the_pi_measurement_inside_the_band(self):
        self.assertEqual(rim_height(0.28, 0.27), (0.28, "measured"))
        self.assertEqual(rim_height(0.15, 0.27)[0], 0.15)
        self.assertEqual(rim_height(0.50, 0.27)[1], "measured")
        self.assertEqual(rim_height(0.14, 0.27), (0.27, "fallback"))
        self.assertEqual(rim_height(0.51, 0.27), (0.27, "fallback"))
        self.assertEqual(rim_height(float("nan"), 0.27), (0.27, "fallback"))
        self.assertEqual(rim_height(None, 0.27), (0.27, "fallback"))

    def test_outlier_is_more_than_10_cm_or_wider_than_30_cm(self):
        self.assertFalse(pose_outlier(0.10, 0.18))
        self.assertTrue(pose_outlier(0.11, 0.18))
        self.assertFalse(pose_outlier(0.0, 0.30))
        self.assertTrue(pose_outlier(0.0, 0.31))
        self.assertFalse(pose_outlier(None, None))

    def test_a_disagreement_relooks_once_then_uses_lidar(self):
        goal = (0.35, 0.0)
        good = {"centre": (0.36, 0.01), "width": 0.18, "standing": True, "confidence": 0.8, "from": "rim"}
        far = {"centre": (0.50, 0.0), "width": 0.18, "standing": True, "confidence": 0.9, "from": "rim"}
        wide = {"centre": (0.36, 0.0), "width": 0.40, "standing": False, "confidence": 0.9, "from": "rim"}
        fallen = {"centre": (0.36, 0.0), "width": 0.10, "standing": False, "confidence": 0.8, "from": "foot"}
        used = resolve_grasp([good], goal, 0.18)
        self.assertEqual(used["source"], "camera")
        self.assertFalse(used["relook"])
        again = resolve_grasp([far, good], goal, 0.18)
        self.assertTrue(again["relook"])
        self.assertEqual(again["source"], "camera")
        self.assertEqual(again["xy"], (0.36, 0.01))
        fallback = resolve_grasp([far, far], goal, 0.18)
        self.assertEqual(fallback["source"], "lidar")
        self.assertEqual(fallback["xy"], goal)
        self.assertIsNone(fallback["failure"])
        wide_fallback = resolve_grasp([wide, wide], goal, 0.18)
        self.assertEqual(wide_fallback["source"], "lidar")
        self.assertTrue(bag_fallen(fallen, 0.18))
        self.assertEqual(resolve_grasp([fallen], goal, 0.18)["failure"], "fallen")

    def test_a_pick_waits_at_55_c_and_does_not_start_at_the_driver_limit(self):
        self.assertEqual(PICK_TEMP_C, 55.0)
        self.assertEqual(DRIVER_TEMP_C, 60.0)
        self.assertEqual(cool_reason(54.9), "")
        self.assertEqual(cool_reason(None), "")
        reason = cool_reason(60.0)
        self.assertIn("waiting", reason)
        self.assertIn("60", reason)
        clock = {"t": 0.0}
        readings = iter([58.0, 57.0, 54.0])

        def read():
            return next(readings)

        def sleep(dt):
            clock["t"] += dt

        waited, why, end = wait_until_cool(read, sleep, lambda: clock["t"])
        self.assertEqual(end, 54.0)
        self.assertGreater(waited, 0.0)
        self.assertIn("waiting", why)

        def still_hot():
            return 61.0

        with self.assertRaises(Failed) as caught:
            wait_until_cool(still_hot, lambda dt: clock.__setitem__("t", clock["t"] + dt),
                            lambda: clock["t"], limit_s=0.0)
        self.assertEqual(caught.exception.code, "HOT")
        self.assertIn("waiting", str(caught.exception))

    def test_an_unverified_hold_is_not_success(self):
        self.assertEqual(pick_result("ถือถุง"), ("SUCCESS", "ถือถุง"))
        code, detail = pick_result(("SUCCESS_UNVERIFIED", "ถือถุง (ยังไม่ได้ยืนยันด้วยกล้อง)"))
        self.assertEqual(code, "SUCCESS_UNVERIFIED")
        self.assertNotEqual(code, "SUCCESS")
        self.assertIn("กล้อง", detail)

    def test_move_closer_is_the_far_wall_reason(self):
        self.assertEqual(REACH_WALL, 0.32)
        src = inspect.getsource(CameraBag._pick_impl)
        self.assertIn("move closer", src)
        self.assertLess(src.index("self.n.line(down_path"), src.index('self.n.gripper(0.0, "หนีบ")'))
        self.assertLess(src.index('self.n.gripper(0.0, "หนีบ")'), src.index("self.n.line(lift_path"))
        self.assertIn("_down_stepwise", src)
        self.assertIn("_swing_stepwise", src)


class ContinuousPlan(unittest.TestCase):
    def test_one_plan_matches_the_stepwise_targets_and_fits_in_30s(self):
        node = Node(LOOK)
        cam = CameraBag(node)
        cam.theta = 0.0
        close_z = FLOOR + 0.19
        found = None
        for pitch, roll in ((-60, 200.0), (-60, 0.0), (-50, 200.0), (-60, 90.0)):
            for reach in (0.30, 0.28, 0.26):
                plan = cam.plan_handle_lift(reach, close_z, pitch, roll)
                if plan is not None:
                    found = (reach, pitch, roll, plan)
                    break
            if found:
                break
        self.assertIsNotNone(found, "no handle pose in reach from LOOK; the dry run cannot time the phases")
        reach, pitch, roll, plan = found
        down, lift, lifted, seconds = plan
        self.assertGreater(len(down), 0)
        self.assertAlmostEqual(lifted, LIFT_CONTINUOUS[-1])
        # The same tip(sim=) targets: above the handle, one descent that does not bulge, then the
        # straight 5 cm lift. A descent that bulges past PATH_BOW keeps the old 2 cm steps.
        start = {j: float(LOOK[j]) for j in ARM}
        pre = cam._approach(reach, close_z, pitch, roll)
        self.assertIsNotNone(pre)
        sim = dict(start)
        steps = [(pre[0], close_z + pre[1], pitch, roll, 0.0), (reach, close_z + 0.10, pitch, roll, 0.0)]
        segs = cam.dry_steps(steps, sim)
        direct = cam.tip(reach, close_z, pitch, roll, "", sim=dict(sim))
        if tip_overshoot(cam.kin, direct) <= PATH_BOW:
            segs.append(direct)
            sim.update(direct[-1])
        else:
            segs.extend(cam.dry_steps(cam.descend_steps(reach, close_z, pitch, roll), sim))
        lift_steps = [(reach, close_z + dz, pitch, roll, 0.03) for dz in LIFT_CONTINUOUS]
        lift_segs = cam.dry_steps(lift_steps, sim)
        expected_down = [q for seg in segs for q in seg]
        expected_lift = [q for seg in lift_segs for q in seg]
        self.assertTrue(same_path(down, expected_down))
        self.assertTrue(same_path(lift, expected_lift))
        self.assertLessEqual(tip_overshoot(cam.kin, direct), PATH_BOW)
        self.assertAlmostEqual(seconds["down_s"], line_seconds(len(down), FREE_SPEED, JOINT_STEP))
        self.assertAlmostEqual(seconds["lift_s"], line_seconds(len(lift), LIFT_SPEED, JOINT_STEP))
        # The arm is at the top of the lift when the raise and the swing are planned.
        node.joints = {j: float(lift[-1][j]) for j in ARM}
        # raise to the carry height (the bag's bottom 0.28 m up, over the LiDAR), the jaws tilting to HOLD_PITCH
        want_tz = FLOOR + 0.28 + (close_z - FLOOR)
        raise_path, top, raise_pitch = cam.raise_path(reach, roll, close_z + lifted, pitch, want_tz)
        raise_s = line_seconds(len(raise_path), HELD_SPEED)
        if raise_path:
            node.joints = {j: float(raise_path[-1][j]) for j in ARM}
        hold_pitch = pitch if raise_pitch is None else raise_pitch
        swing_s = 0.0
        for sign in (1.0, -1.0):
            path, _end, full = cam.swing_plan(reach, top, hold_pitch, roll, 0.0, sign * CARRY_THETA)
            if full and path:
                swing_s = line_seconds(len(path), HELD_SPEED)
                break
        total = seconds["down_s"] + seconds["lift_s"] + raise_s + swing_s
        report = ("down %.2fs (%d pts) lift %.2fs (%d pts, %.0f cm) raise %.2fs swing %.2fs total %.2fs "
                  "at free %.2f / lift %.2f / held %.2f rad/s, reach %.2f pitch %s roll %s"
                  % (seconds["down_s"], len(down), seconds["lift_s"], len(lift), lifted * 100, raise_s, swing_s,
                     total, FREE_SPEED, LIFT_SPEED, HELD_SPEED, reach, pitch, roll))
        print(report)
        self.assertLess(total, 30.0, report)
        # One line() each for the way down and the lift. Nothing else moves between them.
        fresh = Node(LOOK)
        mover = CameraBag(fresh)
        mover.theta = 0.0
        again = mover.plan_handle_lift(reach, close_z, pitch, roll)
        self.assertIsNotNone(again)
        fresh.line(again[0], "ลงไปที่ถุง", FREE_SPEED, JOINT_STEP)
        fresh.line(again[1], "ยกถุงตรงขึ้น", LIFT_SPEED, JOINT_STEP)
        self.assertEqual(len(fresh.calls), 2)
        self.assertEqual([call[0] for call in fresh.calls], ["ลงไปที่ถุง", "ยกถุงตรงขึ้น"])

    def test_an_unreachable_handle_has_no_plan(self):
        cam = CameraBag(Node(LOOK))
        cam.theta = 0.0
        self.assertIsNone(cam.plan_handle_lift(1.20, FLOOR + 0.19, -60, 0.0))


if __name__ == "__main__":
    unittest.main()
