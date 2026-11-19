"""Speed profiles and the Pilz LIN request. No ROS."""
import ast
import importlib.util
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from so100_driver import motion_profile as mp  # noqa: E402

PKG = Path(__file__).resolve().parents[1] / "so100_driver"
JETSON = Path(__file__).resolve().parents[3] / "base" / "jetson" / "speed_profile.py"


def jetson_profile():
    spec = importlib.util.spec_from_file_location("jetson_speed_profile", JETSON)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Resolve(unittest.TestCase):
    def test_safe_keeps_the_live_parameters(self):
        got = mp.resolve("safe", {"joint_scale": 0.30, "approach_speed": 0.03, "bag_speed": 0.08})
        self.assertEqual(got["joint_scale"], 0.30)
        self.assertEqual(got["approach_speed"], 0.03)
        self.assertEqual(got["bag_speed"], 0.08)
        self.assertEqual(got["planning_time"], 5.0)
        self.assertEqual(got["planning_attempts"], 3)

    def test_fast_is_the_bench_step(self):
        self.assertEqual(mp.resolve("fast"), mp.FAST_PROFILE)
        self.assertEqual(mp.FAST_PROFILE["joint_scale"], 0.45)
        self.assertEqual(mp.FAST_PROFILE["approach_speed"], 0.08)
        self.assertEqual(mp.FAST_PROFILE["bag_speed"], 0.12)
        self.assertEqual(mp.FAST_PROFILE["line_speed"], 0.08)
        self.assertEqual(mp.FAST_PROFILE["planning_time"], 1.0)
        self.assertEqual(mp.FAST_PROFILE["planning_attempts"], 1)

    def test_an_unknown_name_is_refused(self):
        with self.assertRaises(ValueError):
            mp.resolve("faster")

    def test_the_jetson_copy_matches(self):
        other = jetson_profile()
        for key in mp.FAST_PROFILE:
            self.assertEqual(other.FAST_PROFILE[key], mp.FAST_PROFILE[key], key)
            self.assertEqual(other.SAFE_PROFILE[key], mp.SAFE_DEFAULTS[key], key)


class Lin(unittest.TestCase):
    def test_velocity_scale_is_the_line_speed_over_the_pilz_cap(self):
        self.assertAlmostEqual(mp.velocity_scale(0.03), 0.1)
        self.assertAlmostEqual(mp.velocity_scale(0.08), 0.08 / 0.3)
        self.assertEqual(mp.velocity_scale(1.0), 1.0)

    def test_identity_quaternion(self):
        eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        x, y, z, w = mp.quat_xyzw(eye)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 0.0)
        self.assertAlmostEqual(w, 1.0)

    def test_lin_request_asks_pilz_for_a_straight_line(self):
        matrix = [[1, 0, 0, 0.2], [0, 1, 0, 0.1], [0, 0, 1, 0.3], [0, 0, 0, 1]]
        req = mp.lin_request(matrix, 0.03, "tcp")
        self.assertEqual(req["pipeline_id"], mp.PILZ_PIPELINE)
        self.assertEqual(req["planner_id"], "LIN")
        self.assertEqual(req["frame_id"], "base")
        self.assertEqual(req["link"], "tcp")
        self.assertEqual(req["position"], (0.2, 0.1, 0.3))
        self.assertAlmostEqual(req["velocity_scale"], 0.1)

    def test_tip_frame_prefers_tcp(self):
        frames = {"gripper": "g", "tcp": "t"}
        self.assertEqual(mp.tip_frame(frames), ("tcp", "t"))
        self.assertEqual(mp.tip_frame({"gripper": "g"}), ("gripper", "g"))

    def test_ompl_request_uses_the_profile(self):
        req = mp.ompl_request(mp.FAST_PROFILE)
        self.assertEqual(req["pipeline_id"], "ompl")
        self.assertEqual(req["allowed_planning_time"], 1.0)
        self.assertEqual(req["num_planning_attempts"], 1)
        self.assertEqual(req["max_velocity_scaling_factor"], 0.45)
        safe = mp.ompl_request(mp.SAFE_DEFAULTS)
        self.assertEqual(safe["allowed_planning_time"], 5.0)
        self.assertEqual(safe["num_planning_attempts"], 3)
        self.assertEqual(safe["max_velocity_scaling_factor"], 0.30)


class NodeDefaults(unittest.TestCase):
    def test_the_declared_safe_speeds_are_unchanged(self):
        tree = ast.parse((PKG / "pick_place_node.py").read_text(encoding="utf-8"))
        defaults = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "declare_parameter":
                name, value = node.args[0], node.args[1]
                if isinstance(name, ast.Constant) and isinstance(value, ast.Constant):
                    defaults[name.value] = value.value
        self.assertEqual(defaults["velocity_scaling"], 0.3)
        self.assertEqual(defaults["line_speed"], 0.03)
        self.assertEqual(defaults["bag_speed"], 0.08)
        self.assertEqual(defaults["speed_profile"], "safe")
        pick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "pick")
        src = ast.unparse(pick)
        self.assertIn("self.straight(s['approach'], 'ลงไปที่ของ')", src)
        self.assertIn("self.straight(s['lift'], 'ยกขึ้น')", src)
        self.assertIn("joint_path", src)  # the by-weight lift stays a joint line


if __name__ == "__main__":
    unittest.main()
