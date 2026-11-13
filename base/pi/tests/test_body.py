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

    def test_limit_refuses_a_bad_cap(self):
        b = body.Body(P)
        for bad in (None, float("nan"), -0.1):
            with self.assertRaises(ValueError):
                b.limit(bad)


if __name__ == "__main__":
    unittest.main()
