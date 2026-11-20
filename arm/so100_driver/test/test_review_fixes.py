"""Code review of 2026-11-19, arm findings 1, 2 and 5: the reviewer's offline cases, now asserting the fixed
behaviour. No ROS, no servos, no camera."""
import ast
import os
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so100_driver import bag_camera as bc  # noqa: E402
from so100_driver.calib import JOINTS, Calibration  # noqa: E402
from so100_driver.hardware import SimArm  # noqa: E402
from so100_driver.motion import Motion, ids_to_names, names_to_ids  # noqa: E402
from so100_driver.task_rules import Failed  # noqa: E402

PKG = Path(__file__).resolve().parents[1] / "so100_driver"


def methods_of(filename, class_name, names, namespace):
    """Compile only the named methods of a ROS node class (the module itself imports rclpy)."""
    tree = ast.parse((PKG / filename).read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    assert {n.name for n in body} == set(names)
    mod = ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
        name="Methods", bases=[], keywords=[], body=body, decorator_list=[])], type_ignores=[]))
    env = dict(namespace)
    exec(compile(mod, str(PKG / filename), "exec"), env)
    return env["Methods"]


def node_constant(filename, name, default):
    """A module-level constant of a node file (its action_msgs GoalStatus value when the file has none yet)."""
    tree = ast.parse((PKG / filename).read_text(encoding="utf-8"))
    for n in tree.body:
        if isinstance(n, ast.Assign):
            for t in n.targets:
                if isinstance(t, ast.Tuple) and name in [e.id for e in t.elts]:
                    return ast.literal_eval(n.value)[[e.id for e in t.elts].index(name)]
    return default


class StopEndsTheBagPick(unittest.TestCase):
    """Finding 2: a STOPPED (or driver fault, too hot, collision) failure in a real move ends the pick; only
    "out of reach" lets the loop stop early or try another way."""

    def camera(self, fail_at, code):
        node = NS(kin=NS(heading0=0), joints={j: 0.0 for j in bc.ARM}, table_z=-0.331)
        cam = bc.CameraBag(node)
        cam.speed = lambda kind: 0.5
        cam.feasible = lambda *a, **k: True
        moves = []

        def tip(*args, **kwargs):
            moves.append(args[4])
            if args[4] == fail_at and kwargs.get("sim") is None:
                raise Failed("stopped", code)

        cam.tip = tip
        return cam, moves

    def setUp(self):
        self.sleep, bc.time.sleep = bc.time.sleep, lambda _: None

    def tearDown(self):
        bc.time.sleep = self.sleep

    def test_stop_in_the_lift_ends_it(self):
        for code in ("STOPPED", "FAILED", "HOT", "COLLISION"):
            cam, moves = self.camera("ยกถุงตรงขึ้น", code)
            with self.assertRaises(Failed) as ctx:
                cam._lift_stepwise(0.3, -0.12, -60, 180)
            self.assertEqual(ctx.exception.code, code)
            self.assertEqual(moves, ["ยกถุงตรงขึ้น"])        # no move after the failing one

    def test_out_of_reach_in_the_lift_stops_it_early(self):
        cam, moves = self.camera("ยกถุงตรงขึ้น", "UNREACHABLE")
        self.assertEqual(cam._lift_stepwise(0.3, -0.12, -60, 180), 0.0)

    def test_stop_in_the_swing_does_not_try_the_other_side(self):
        cam, moves = self.camera("เก็บถุงไว้ด้านข้าง (หลบ LiDAR)", "STOPPED")
        cam.theta = 0.0
        with self.assertRaises(Failed):
            cam._swing_stepwise(0.3, 0.1, -35, 180, {})
        self.assertEqual(len(moves), 1)

    def test_stop_on_the_way_above_the_bag_ends_it(self):
        cam, moves = self.camera("ไปเหนือถุง", "STOPPED")
        with self.assertRaises(Failed) as ctx:
            cam.above(0.3, -0.12, -60, 180)
        self.assertEqual(ctx.exception.code, "STOPPED")
        self.assertEqual(moves, ["ไปเหนือถุง"])


class ActionEndsAreChecked(unittest.TestCase):
    """Finding 5: an ABORTED gripper goal is a failure; a stop before sending sends nothing."""

    def task(self):
        ns = {"Failed": Failed, "GOAL_SUCCEEDED": node_constant("pick_place_node.py", "GOAL_SUCCEEDED", 4),
              "GOAL_CANCELED": node_constant("pick_place_node.py", "GOAL_CANCELED", 5),
              "GripperCommand": NS(Goal=lambda: NS(command=NS(position=0.0, max_effort=0.0)))}
        t = methods_of("pick_place_node.py", "PickPlace", {"run_action", "gripper"}, ns)()
        t.preempt, t.cancel_requested, t.grip_max, t.grip = False, False, 1.0, None
        t.wait = lambda future, timeout: future
        return t

    @staticmethod
    def client(status, result, sent):
        handle = NS(accepted=True, get_result_async=lambda: NS(status=status, result=result),
                    cancel_goal_async=lambda: None)
        return NS(wait_for_server=lambda **k: True, send_goal_async=lambda g: (sent.append(g), handle)[1])

    def test_aborted_gripper_fails(self):
        t, sent = self.task(), []
        t.grip = self.client(6, NS(position=0.0, effort=0.0, reached_goal=False, stalled=False), sent)
        with self.assertRaises(Failed) as ctx:
            t.gripper(0.0, "close")
        self.assertEqual(ctx.exception.code, "FAILED")

    def test_gripper_that_got_there_or_holds_is_fine(self):
        for reached, stalled in ((True, False), (False, True)):
            t, sent = self.task(), []
            t.grip = self.client(4, NS(position=0.05, effort=12.0, reached_goal=reached, stalled=stalled), sent)
            self.assertEqual(t.gripper(0.0, "close").position, 0.05)

    def test_succeeded_gripper_neither_there_nor_holding_fails(self):
        t, sent = self.task(), []
        t.grip = self.client(4, NS(position=0.3, effort=0.0, reached_goal=False, stalled=False), sent)
        with self.assertRaises(Failed):
            t.gripper(0.0, "close")

    def test_aborted_trajectory_returns_its_error_code_to_the_caller(self):
        t, sent = self.task(), []
        res = NS(error_code=-4, error_string="path")
        self.assertIs(t.run_action(self.client(6, res, sent), object(), 10, "move"), res)

    def test_stop_before_sending_sends_nothing(self):
        t, sent = self.task(), []
        t.cancel_requested = True
        with self.assertRaises(Failed) as ctx:
            t.run_action(self.client(4, NS(), sent), object(), 10, "move")
        self.assertEqual(ctx.exception.code, "STOPPED")
        self.assertEqual(sent, [])

    def test_stop_during_the_goal_is_stopped(self):
        t, sent = self.task(), []
        client = self.client(4, NS(error_code=0), sent)
        orig = client.send_goal_async

        def send(goal):
            t.cancel_requested = True                       # /arm/stop arrives while the goal runs
            return orig(goal)

        client.send_goal_async = send
        with self.assertRaises(Failed) as ctx:
            t.run_action(client, object(), 10, "move")
        self.assertEqual(ctx.exception.code, "STOPPED")


class FaultHoldReachesTheServos(unittest.TestCase):
    """Finding 1: every fault path sends the hold to the servos (tick() writes nothing while faulted)."""

    def driver(self):
        class JointState:
            def __init__(self):
                self.header = NS()

        d = methods_of("driver_node.py", "Driver", {"tick", "set_fault", "end_traj"}, {
            "time": time, "ids_to_names": ids_to_names, "names_to_ids": names_to_ids, "ARM_FAULT": -100,
            "PATH_TOLERANCE_VIOLATED": -4, "JointState": JointState, "JOINTS": JOINTS})()
        d.calib = Calibration.from_zero({j: 2048 for j in JOINTS})
        for j in JOINTS:
            d.calib.data[j].update(min=100, max=4000)
        d.motion = Motion(d.calib, {})
        d.present = {j: 2048 for j in JOINTS}
        d.motion.hold(d.present)
        d.arm = SimArm(names_to_ids(d.present))
        d.arm.set_torque(True)
        old = names_to_ids(d.present)
        old[2] += 130                                       # the shoulder still going to an old goal
        d.arm.write_goals(old)
        d.lock = threading.RLock()
        d.last_tick = time.monotonic()
        d.read_errors, d.enabled, d.fault = 0, True, None
        d.errors = []
        d.get_logger = lambda: NS(error=d.errors.append)
        d.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
        d.js_pub = NS(publish=lambda _: None)
        return d

    def test_slow_tick_fault_stops_the_shoulder(self):
        d = self.driver()
        d.set_fault("servo over-temperature", -101)
        self.assertEqual(d.arm.goal[2], 2048)               # the hold went to the servo
        for _ in range(3):
            d.tick()
        self.assertEqual(d.present["shoulder_lift"], 2048)

    def test_hold_that_cannot_be_sent_is_reported(self):
        d = self.driver()

        def broken(*a, **k):
            raise OSError("no reply")

        d.arm.write_goals = broken
        d.set_fault("lost contact with servos", -100)
        self.assertIn("hold NOT sent", d.fault)
        self.assertIn("hold NOT sent", d.errors[-1])


if __name__ == "__main__":
    unittest.main()
