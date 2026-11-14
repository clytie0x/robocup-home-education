import math
import unittest

import body
import params
from hmc import world

P = params.load()


class BodyTest(unittest.TestCase):
    def test_limits_come_from_params(self):
        b = body.Body(P)
        self.assertEqual(b.robot.max_speed, P["max_speed"])
        self.assertEqual(b.robot.max_turn_rate, P["max_turn_rate"])
        self.assertEqual(world.MAX_ACCEL, P["accel"])
        self.assertEqual(world.MAX_TURN_ACCEL, P["turn_accel"])
        self.assertEqual(world.ROBOT_RADIUS, P["robot_radius"])

    def test_body_sets_decel_delay_and_deadband(self):
        body.Body(P)
        self.assertEqual(world.MAX_DECEL, P["decel"])
        self.assertEqual(world.STOP_DELAY, P["stop_delay"])
        self.assertAlmostEqual(world.HEADING_DEADBAND, math.radians(4))
        from hmc import navigation
        self.assertTrue(navigation.CLOSE_PLAN)
        self.assertTrue(navigation.TOWARD_UNREACHABLE)  # the real base comes as close as it can
        self.assertFalse(navigation.REPLAN_ON_NEW_SHAPES)

    def test_command_comes_from_the_brain_step(self):
        b = body.Body(P)
        b.sync((0, 0, 0))
        b.robot.drive_toward((2, 0), 0.1)
        v, w = b.command()
        self.assertGreater(v, 0)
        self.assertLessEqual(v, P["accel"] * 0.1 + 1e-9)

    def test_sync_keeps_speed_and_takes_the_measured_pose(self):
        b = body.Body(P)
        b.robot.speed, b.robot.turn_speed = 0.1, 0.2
        b.sync((1.0, 2.0, 0.5))
        r = b.robot
        self.assertEqual((r.x, r.y, r.heading), (1.0, 2.0, 0.5))
        self.assertEqual((r.speed, r.turn_speed), (0.1, 0.2))

    def test_limit_lowers_the_cap_for_one_cycle_only(self):
        b = body.Body(P)
        b.limit(0.05)
        self.assertEqual(b.robot.max_speed, 0.05)
        b.limit(10.0)  # never above the params cap
        self.assertEqual(b.robot.max_speed, P["max_speed"])

    def test_the_body_keeps_where_the_brains_turn_goes_until_the_next_sync(self):
        # cable.Cable needs the heading a turn goes for, to know whether the long way round still gets there
        b = body.Body(P)
        b.sync((0.0, 0.0, 0.0))
        self.assertIsNone(b.robot.turn_error)
        b.robot.turn_toward((0.0, 1.0), 0.1)  # a quarter turn to the left
        self.assertAlmostEqual(b.robot.turn_error, math.pi / 2)
        b.sync((0.0, 0.0, 0.1))
        self.assertIsNone(b.robot.turn_error)

    def test_limit_refuses_a_bad_cap(self):
        b = body.Body(P)
        for bad in (None, float("nan"), -0.1):
            with self.assertRaises(ValueError):
                b.limit(bad)


class BrainMemoryTest(unittest.TestCase):
    def test_a_circle_seen_once_leaves_the_brain_with_the_senses(self):
        # 2026-11-15: the brain kept every circle MEMORY_KEEP (10 s) after RealSenses dropped it, so a
        # stray depth point blocked the way for 20 s. On the real base the senses' memory is the only one.
        import run_follow  # noqa: F401  (its import sets the real base's brain settings)
        from hmc.follower import Follower, Senses
        from hmc.world import Circle
        b = body.Body(P)
        brain = Follower(b.robot, speech=None)
        stray = Circle(1.0, 0.0, 0.05)
        self.assertIn(stray, brain.shapes(Senses(0.0, None, None, {}, [stray])))
        self.assertNotIn(stray, brain.shapes(Senses(1.2, None, None, {}, [])))  # 10 s before the fix


if __name__ == "__main__":
    unittest.main()
