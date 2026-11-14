"""The power cable's winding: the net turn since the owner last straightened it (cable.py)."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cable
import params

P = params.load()


class Said:
    def __init__(self):
        self.lines = []

    def say(self, time, text):
        self.lines.append(text)


class Folder(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.path = Path(folder.name) / "data" / "cable.json"


class File(Folder):
    def test_a_missing_file_is_a_straight_cable_of_unknown_age(self):
        self.assertEqual(cable.load(self.path), {"net_deg": 0.0, "set_at": None})
        self.assertEqual(cable.Cable(P, self.path).net_deg, 0.0)

    def test_reset_writes_zero_and_the_time_atomically(self):
        cable.save(self.path, 170.0, 5.0)
        with mock.patch("time.time", return_value=1234.5):
            self.assertEqual(cable.reset(self.path), {"net_deg": 0.0, "set_at": 1234.5})
        self.assertEqual(json.loads(self.path.read_text()), {"net_deg": 0.0, "set_at": 1234.5})
        self.assertEqual([f.name for f in self.path.parent.iterdir()], ["cable.json"])  # no temporary file left

    def test_the_net_follows_the_odometry_yaw_and_is_saved_at_most_once_a_second(self):
        c = cable.Cable(P, self.path)
        yaw, now = 7.07, 0.0  # the odometry yaw is not wrapped (2026-11-15: 7.07 to 17.87 rad)
        c.update(yaw, now)
        for _ in range(25):  # 2.5 s at 10 Hz, +0.3 rad/s
            yaw, now = yaw + 0.03, now + 0.1
            c.update(yaw, now)
        self.assertAlmostEqual(c.net_deg, math.degrees(0.75), places=6)
        saved = json.loads(self.path.read_text())["net_deg"]
        self.assertLess(saved, c.net_deg)  # the last cycles are not written yet
        self.assertGreaterEqual(saved, c.net_deg - math.degrees(0.03) * 10)  # but at most a second old
        writes = []
        with mock.patch.object(cable, "save", side_effect=lambda *a: writes.append(a)):
            for i in range(10):
                c.update(yaw, now + 0.05 * (i + 1))
        self.assertLessEqual(len(writes), 1)
        c.save()
        self.assertAlmostEqual(json.loads(self.path.read_text())["net_deg"], c.net_deg, places=2)

    def test_a_wrapped_yaw_counts_the_short_step(self):
        c = cable.Cable(P, None)  # no file: tests and dry tools keep it in memory
        c.update(math.pi - 0.05, 0.0)
        c.update(-math.pi + 0.05, 0.1)
        self.assertAlmostEqual(c.net_deg, math.degrees(0.1), places=6)

    def test_a_reset_while_running_is_taken_at_the_next_save(self):
        c = cable.Cable(P, self.path)
        c.update(0.0, 0.0)
        c.update(3.0, 0.1)
        c.save()
        self.assertGreater(c.net_deg, 150.0)
        cable.reset(self.path)  # the console's button while run_follow runs
        c.update(3.0, 2.0)  # the next save, a second later, sees the reset
        self.assertEqual(c.net_deg, 0.0)
        self.assertEqual(json.loads(self.path.read_text())["net_deg"], 0.0)

    def test_the_cli_resets(self):
        cable.save(self.path, -160.0, None)
        cable.main(["--reset", "--path", str(self.path)])
        self.assertEqual(json.loads(self.path.read_text())["net_deg"], 0.0)


class Steer(unittest.TestCase):
    """Owner, 2026-11-15: warn at 200 deg, never past 240; a turn free to go either way unwinds."""

    def wound(self, net):
        c = cable.Cable(P, None, Said())
        c.net_deg = net
        return c

    def test_the_limits(self):
        self.assertEqual((cable.WARN_DEG, cable.LIMIT_DEG), (200.0, 240.0))

    def test_below_the_warning_a_turn_passes(self):
        c = self.wound(150.0)
        self.assertEqual(c.steer(0.0, 0.3, math.radians(40), 0.0), 0.3)  # ends at 190
        self.assertIsNone(c.why)

    def test_a_turn_that_would_end_past_240_goes_the_long_way_round(self):
        c = self.wound(160.0)
        self.assertEqual(c.steer(0.0, 0.4, math.radians(90), 0.0), -0.4)  # +90 ends at 250: -270 ends at -110
        self.assertEqual(c.why, "the other way round")
        c.net_deg = 100.0
        self.assertEqual(c.steer(0.0, 0.4, math.radians(170), 0.1), -0.4)  # latched while the brain turns on
        self.assertEqual(c.steer(0.0, -0.4, math.radians(-170), 0.2), -0.4)  # the brain agrees past 180 deg
        self.assertEqual(c.steer(0.0, 0.0, 0.0, 0.3), 0.0)  # at the heading: the latch is gone
        self.assertEqual(c.steer(0.0, 0.3, math.radians(20), 0.4), 0.3)

    def test_a_goal_behind_turns_the_way_that_unwinds(self):
        c = self.wound(40.0)
        self.assertEqual(c.steer(0.0, 0.4, math.radians(160), 0.0), -0.4)  # +160 ends at 200, -200 at -160
        c = self.wound(40.0)
        self.assertEqual(c.steer(0.0, 0.4, math.radians(100), 0.0), 0.4)  # not behind: the short way

    def test_past_200_a_turn_that_winds_more_goes_round(self):
        c = self.wound(205.0)
        self.assertEqual(c.steer(0.0, 0.3, math.radians(10), 0.0), -0.3)  # +10 is refused; -350 ends at -145
        c = self.wound(-210.0)
        self.assertEqual(c.steer(0.0, -0.3, math.radians(-60), 0.0), 0.3)  # +300 ends at 90
        self.assertEqual(c.speech.lines, [])

    def test_the_long_way_round_is_kept_until_the_heading_is_reached(self):
        # review 2026-11-15: the long way was dropped at once (|error| measured the short way < KEEP_WAY),
        # so the robot shook on the spot at net 200
        for net0, err0 in ((200.0, 30.0), (185.0, 40.0), (205.0, 10.0)):
            c = self.wound(net0)
            heading = math.radians(err0)            # the goal heading, relative to the start
            yaw, ws = 0.0, []
            for i in range(400):                    # 40 s of 0.1 s cycles
                error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
                if abs(error) < math.radians(3):
                    break
                w = c.steer(0.0, math.copysign(0.3, error), error, 0.1 * i)
                ws.append(w)
                yaw += w * 0.1
                c.net_deg += math.degrees(w * 0.1)
                self.assertLessEqual(abs(c.net_deg), cable.LIMIT_DEG)
            else:
                self.fail("from net %.0f the heading %+.0f was never reached (net %.1f)" % (net0, err0, c.net_deg))
            flips = sum(1 for a, b in zip(ws, ws[1:]) if a * b < 0)
            self.assertLessEqual(flips, 1, (net0, err0, ws[:20]))

    def test_the_search_spin_unwinds_first_turns_round_at_200_and_never_passes_240(self):
        c = self.wound(50.0)
        self.assertEqual(c.steer(0.0, 0.3, None, 0.0), -0.3)  # no heading of its own: the way that unwinds
        c, net, ws = self.wound(0.0), 0.0, []
        for i in range(600):  # 60 s of the search's 0.3 rad/s to the left, no goal heading
            w = c.steer(0.0, 0.3, None, 0.1 * i)
            ws.append(w)
            c.net_deg += math.degrees(w * 0.1)
            net = max(net, abs(c.net_deg))
        self.assertLessEqual(net, cable.WARN_DEG + 2.0)
        self.assertIn(-0.3, ws)
        self.assertIn(0.3, ws[200:])  # round again at -200
        self.assertEqual(c.speech.lines, [])

    def test_an_arc_that_winds_more_past_200_drives_straight(self):
        c = self.wound(202.0)
        self.assertEqual(c.steer(0.1, 0.2, math.radians(5), 0.0), 0.0)
        self.assertEqual(c.steer(0.1, -0.2, math.radians(-5), 0.0), -0.2)

    def test_no_command_takes_the_net_past_240_and_it_says_so_once(self):
        c = self.wound(236.0)
        self.assertEqual(c.steer(0.0, 0.3, math.radians(-200), 0.0, planned=True), 0.0)  # its run-on would pass 240
        self.assertEqual((c.why, c.holding), ("wound", True))
        c.steer(0.0, 0.3, None, 0.1, planned=True)
        self.assertEqual(c.speech.lines, [cable.TEXT])
        self.assertEqual(c.steer(0.0, -0.3, math.radians(-10), 0.2), -0.3)  # unwinding is always fine
        self.assertFalse(c.holding)

    def test_turns_lists_the_ways_the_cable_allows_the_unwinding_first_when_free(self):
        c = self.wound(160.0)
        self.assertEqual([round(math.degrees(a)) for a in c.turns(math.radians(40))], [40, -320])
        self.assertEqual([round(math.degrees(a)) for a in c.turns(math.radians(170))], [-190])
        c.net_deg = 40.0
        self.assertEqual([round(math.degrees(a)) for a in c.turns(math.radians(150))], [-210, 150])
        c.net_deg = 205.0
        self.assertEqual([round(math.degrees(a)) for a in c.turns(math.radians(10))], [-350])


def drive(lay, poses):
    for pose in poses:
        lay.update(pose)


def line(x0, x1, y=0.0, th=0.0, step=0.02):
    n = int(round(abs(x1 - x0) / step))
    return [(x0 + (x1 - x0) * k / n, y, th) for k in range(n + 1)]


class Lay(unittest.TestCase):
    """Where the dragged cable lies: the attach post's path since the cable was straightened."""

    def test_the_lay_is_the_posts_path_without_the_near_part(self):
        lay = cable.Lay(P)
        drive(lay, line(0.0, 3.0))
        circles = lay.circles((3.0, 0.0, 0.0), lambda q: q)
        self.assertAlmostEqual(lay.length_m(), 3.0, delta=0.05)
        self.assertTrue(circles)
        self.assertTrue(all(abs(c.y - cable.ATTACH[1]) < 1e-6 and c.r == cable.RADIUS for c in circles))
        post_x = 3.0 + cable.ATTACH[0]
        self.assertGreaterEqual(post_x - max(c.x for c in circles), cable.NEAR - 1e-6)  # the near 0.6 m is left out
        xs = sorted(c.x for c in circles)
        self.assertTrue(all(abs(b - a - cable.SPACING) < 0.02 for a, b in zip(xs, xs[1:])))

    def test_it_is_capped_at_the_free_length_and_moved_to_the_brains_frame(self):
        lay = cable.Lay(dict(P, cable_free_m=4.0))
        drive(lay, line(0.0, 10.0))
        self.assertAlmostEqual(lay.length_m(), 4.0, delta=0.06)
        circles = lay.circles((10.0, 0.0, 0.0), lambda q: (q[0], q[1] + 1.0))
        self.assertGreater(min(c.x for c in circles), 10.0 + cable.ATTACH[0] - 4.0 - 0.06)
        self.assertTrue(all(abs(c.y - cable.ATTACH[1] - 1.0) < 1e-6 for c in circles))

    def test_douglas_peucker_keeps_the_corners(self):
        poly = [(0.0, 0.0), (1.0, 0.01), (2.0, 0.0), (2.0, 1.0)]
        self.assertEqual(cable.simplify(poly, 0.05), [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)])

    def test_a_spin_on_the_spot_lays_no_cable_round_the_robot(self):
        lay = cable.Lay(P)
        drive(lay, line(0.0, 2.0))
        drive(lay, [(2.0, 0.0, k * 0.05) for k in range(126)])  # a full turn
        circles = lay.circles((2.0, 0.0, 2 * math.pi), lambda q: q)
        self.assertTrue(all(math.dist((c.x, c.y), (2.0, 0.0)) > 0.4 for c in circles))

    def test_a_straightened_cable_starts_a_new_lay(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name) / "cable.json"
        c = cable.Cable(P, path)
        for i, pose in enumerate(line(0.0, 2.0, step=0.05)):
            c.update(pose[2], 0.1 * i, pose)
        self.assertGreater(c.lay.length_m(), 1.9)
        cable.reset(path)
        c.update(0.0, 10.0, (2.0, 0.0, 0.0))
        self.assertLess(c.lay.length_m(), 0.01)

    def test_no_way_beside_the_cable_plans_over_it_slowly(self):
        lay = cable.Lay(P)
        drive(lay, line(0.0, 3.0))
        lay.refresh((3.0, 0.0, 0.0), lambda q: q, now=5.0)
        asked = []

        def planner(start, goal, shapes, near_side=False, heading=None):
            asked.append(len(shapes))
            return None if any(getattr(s, "r", None) == cable.RADIUS for s in shapes) else [start, goal]

        self.assertEqual(lay.plan(planner, (3.0, 0.0), (1.0, 0.5), [], waiting=lambda: False), [(3.0, 0.0), (1.0, 0.5)])
        self.assertEqual(asked[1], 0)  # the second plan, without the cable
        self.assertEqual(lay.crossing, {"since": 5.0})
        self.assertEqual(lay.cap((2.0, 0.1)), cable.CROSS_SPEED)  # near the cable: slow
        self.assertEqual(lay.cap((2.0, -3.0)), math.inf)
        lay.refresh((3.0, 0.0, 0.0), lambda q: q, now=5.0 + cable.CROSS_HOLD + 0.1)
        self.assertIsNone(lay.crossing)
        self.assertIsNone(lay.plan(lambda *a, **k: None, (3.0, 0.0), (1.0, 0.5), [], waiting=lambda: True))
        self.assertIsNone(lay.crossing)  # no answer yet is not no way


if __name__ == "__main__":
    unittest.main()
