import math
import os
import tempfile
import unittest

import numpy

import geometry
import params
import tether

P = params.load()


def drive(t, pose, v, w, seconds, dt=0.1):
    """Integrate (v, w) for `seconds`, calling t.update each step; returns the end pose."""
    x, y, th = pose
    for _ in range(round(seconds / dt)):
        th += w * dt
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        t.update((x, y, th))
    return (x, y, th)


def to_odom(points, pose):
    return numpy.array([geometry.base_to_odom(q, pose) for q in points])


class TrailTest(unittest.TestCase):
    def test_straight_drive_leaves_the_cable_behind_and_ahead_is_clear(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.2, 0.0, 5.0)  # 1 m forward
        pts = t.points(pose)
        self.assertGreater(len(pts), 20)  # the assumed 1 m and most of the 1 m driven
        self.assertTrue((pts[:, 0] < -0.4).all())  # all behind, past the attached part
        self.assertTrue(t.clear(pose, 0.2, 0.0, P))

    def test_the_points_are_spaced_one_step_apart(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        gaps = numpy.hypot(*numpy.diff(t.points((0.0, 0.0, 0.0)), axis=0).T)
        numpy.testing.assert_allclose(gaps, tether.STEP, atol=1e-9)

    def test_a_turn_on_the_spot_keeps_the_swinging_part_attached(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.0, 0.5, math.pi)  # 90 deg left
        odom = to_odom(t.points(pose), pose)
        # only the assumed straight lay behind the start, nothing from the arc the attach point swept
        numpy.testing.assert_allclose(odom[:, 1], tether.ATTACH[1], atol=1e-9)
        self.assertTrue((odom[:, 0] < tether.ATTACH[0] - 0.05).all())

    def test_turning_left_and_back_swings_the_cable_back(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        before = t.points((0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.0, 0.5, math.pi)
        pose = drive(t, pose, 0.0, -0.5, math.pi)
        numpy.testing.assert_allclose(t.points(pose), before, atol=1e-6)
        self.assertAlmostEqual(t.used_length(), 1.0, places=6)

    def test_driving_back_along_the_same_line_is_not_clear(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.2, 0.0, 5.0)
        pose = drive(t, pose, 0.0, 0.5, 2 * math.pi)  # 180 deg on the spot
        self.assertFalse(t.clear(pose, 0.2, 0.0, P))

    def test_a_u_turn_with_a_side_step_then_back_parallel_is_clear(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.2, 0.0, 5.0)
        pose = drive(t, pose, 0.15, -0.5, 2 * math.pi)  # right U-turn, radius 0.3: 0.6 m to the side
        pose = drive(t, pose, 0.2, 0.0, 2.5)
        self.assertAlmostEqual(pose[1], -0.6, delta=0.02)
        self.assertTrue(t.clear(pose, 0.2, 0.0, P))

    def test_backing_up_straight_onto_the_trail_is_not_clear(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.2, 0.0, 5.0)
        self.assertFalse(t.clear(pose, -0.1, 0.0, P))


class LimitTest(unittest.TestCase):
    def test_net_turn_is_unwrapped_and_limited_both_ways(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.0, 0.5, 6.0)  # 3 rad left
        self.assertAlmostEqual(t.net_turn, 3.0, places=6)
        self.assertFalse(t.turn_allowed(0.2))
        self.assertTrue(t.turn_allowed(-6.0))  # back the unwinding way, past zero
        drive(t, pose, 0.0, -0.5, 12.0)  # 6 rad right: -3 rad net
        self.assertAlmostEqual(t.net_turn, -3.0, places=6)
        self.assertFalse(t.turn_allowed(-0.2))
        self.assertTrue(t.turn_allowed(0.2))

    def test_slack_counts_the_assumed_lay_and_the_path(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0), length=3.5)
        drive(t, (0.0, 0.0, 0.0), 0.2, 0.0, 5.0)
        self.assertAlmostEqual(t.used_length(), 2.0, places=6)
        self.assertTrue(t.slack_ok(0.4))
        self.assertFalse(t.slack_ok(0.6))  # 2.6 > 3.5 - 1.0 margin


class StateTest(unittest.TestCase):
    def test_camera_points_are_kept_and_merged(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        t.add_seen([(1.0, 0.0), (1.02, 0.0)])
        t.add_seen([(1.01, 0.01), (1.5, 0.3)])
        pose = (0.5, 0.0, 0.0)
        pts = t.points(pose)
        seen = pts[pts[:, 0] > 0]
        numpy.testing.assert_allclose(seen, [(0.5, 0.0), (1.0, 0.3)], atol=1e-9)
        self.assertFalse(t.clear(pose, 0.2, 0.0, P))

    def test_save_and_load_round_trip(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0), length=8.0)
        pose = drive(t, (0.0, 0.0, 0.0), 0.2, 0.3, 4.0)
        t.add_seen([(2.0, 1.0)])
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "tether.json")
            t.save(path)
            u = tether.Tether.load(path, P)
        numpy.testing.assert_allclose(u.points(pose), t.points(pose))
        self.assertEqual((u.net_turn, u.used_length(), u.length), (t.net_turn, t.used_length(), 8.0))
        drive(u, pose, 0.0, 0.5, 1.0)  # keeps counting from the saved pose
        self.assertAlmostEqual(u.net_turn, t.net_turn + 0.5, places=6)

    def test_reset_lays_the_cable_out_again(self):
        t = tether.Tether(P, (0.0, 0.0, 0.0))
        pose = drive(t, (0.0, 0.0, 0.0), 0.2, 0.5, 4.0)
        t.add_seen([(2.0, 1.0)])
        t.reset(pose)
        self.assertEqual((t.net_turn, t.used_length()), (0.0, 1.0))
        numpy.testing.assert_allclose(t.points(pose), tether.Tether(P, pose).points(pose))


if __name__ == "__main__":
    unittest.main()
