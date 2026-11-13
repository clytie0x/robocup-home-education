import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import slam_replay  # noqa: E402


def make_log(folder, cycles=25, scans=(0, 10, 20)):
    lines = [json.dumps({"time": 100.0 + 0.1 * n, "pose": [0.01 * n, 0.0, 0.0]}) for n in range(cycles)]
    lines.insert(3, json.dumps({"time": 100.3, "say": "hello"}))  # speech is not a cycle
    (Path(folder) / "log.jsonl").write_text("\n".join(lines) + "\n")
    for n in scans:
        scan = [] if n == 0 else [[a, 2.0] for a in range(-170, 170, 10)]
        (Path(folder) / ("scan-%06d.json" % n)).write_text(json.dumps(scan))


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, t, pose, scan):
        self.sent.append((t, pose, scan))


class ReplayPlanTests(unittest.TestCase):
    def test_one_record_per_saved_scan_skipping_the_empty_first(self):
        with tempfile.TemporaryDirectory() as folder:
            make_log(folder)
            plan = slam_replay.load_plan(folder)
        self.assertEqual([round(p[0], 1) for p in plan], [1.0, 2.0])
        self.assertEqual(plan[0][1], [0.1, 0.0, 0.0])  # the pose of cycle 10, not of the scan before it
        self.assertEqual(len(plan[0][2]), 34)

    def test_fill_sends_every_cycle_once_a_scan_exists(self):
        with tempfile.TemporaryDirectory() as folder:
            make_log(folder)
            plan = slam_replay.load_plan(folder, fill=True)
        self.assertEqual(len(plan), 15)  # cycles 10 to 24
        self.assertEqual(plan[0][0], 1.0)

    def test_a_log_without_scans_says_so(self):
        with tempfile.TemporaryDirectory() as folder:
            make_log(folder, scans=())
            with self.assertRaises(ValueError):
                slam_replay.load_plan(folder)

    def test_repair_turns_motion_along_the_unscaled_heading_back_into_the_robot_frame(self):
        scale = 0.862
        raw = 20.0  # the firmware heading after many turns: x and y are integrated along this
        # the robot drives 1 m straight ahead, 0.1 m a step, then 1 m after a 90 degree left turn
        poses = [[0.1 * k * math.cos(raw), 0.1 * k * math.sin(raw), raw / scale] for k in range(11)]
        turned = raw + math.pi / 2
        poses += [[poses[-1][0] + 0.1 * k * math.cos(turned), poses[-1][1] + 0.1 * k * math.sin(turned), turned / scale] for k in range(1, 11)]
        fixed = slam_replay.repaired_poses(poses, scale)
        self.assertEqual(fixed[0], [0.0, 0.0, 0.0])
        self.assertAlmostEqual(fixed[10][0], 1.0, places=6)
        self.assertAlmostEqual(fixed[10][1], 0.0, places=6)
        # the logged heading step across the turn is scaled by 1/yaw_scale, so the repaired turn is too
        self.assertAlmostEqual(fixed[-1][2], (math.pi / 2) / scale, places=6)

    def test_run_keeps_real_time_and_the_speed_factor(self):
        now = [0.0]
        slept = []

        def sleep(seconds):
            slept.append(seconds)
            now[0] += seconds

        link = FakeLink()
        plan = [(0.0, [0, 0, 0], []), (1.0, [1, 0, 0], []), (3.0, [2, 0, 0], [])]
        self.assertEqual(slam_replay.run(plan, link, speed=2.0, clock=lambda: now[0], sleep=sleep), 3)
        self.assertEqual([t for t, _, _ in link.sent], [0.0, 0.5, 1.5])
        self.assertAlmostEqual(sum(slept), 1.5)


if __name__ == "__main__":
    unittest.main()
