"""The follow cap stays 0.12 until a brake measurement says 0.15 is still outside the strip."""
import unittest

import follow_speed
import params

P = params.load()


def coast(speed, stop_m):
    return {"speed": speed, "odom_stop_m": stop_m, "lidar_stop_m": stop_m * 0.5}


class Cap(unittest.TestCase):
    def test_the_committed_cap_is_still_012(self):
        self.assertEqual(P["max_speed"], 0.12)
        self.assertEqual(follow_speed.follow_max_speed(P), 0.12)
        self.assertIn("0.15", follow_speed.SWITCH_HOW)
        self.assertTrue(follow_speed.gate_holds(0.12, P))

    def test_a_stop_inside_the_delay_is_not_a_decel(self):
        self.assertIsNone(follow_speed.decel_from_coast(0.15, 0.15 * P["stop_delay"], P["stop_delay"]))
        self.assertIsNone(follow_speed.decel_from_coast(0.15, 0.02, P["stop_delay"]))

    def test_short_coasts_unlock_015_without_writing_the_file(self):
        runs = [coast(0.08, 0.05), coast(0.12, 0.08), coast(0.15, 0.10)]
        before = dict(P)
        got = follow_speed.recommendation(runs, P)
        self.assertTrue(got["ok"], got)
        self.assertEqual(got["max_speed"], 0.15)
        self.assertGreater(got["decel"], 0.0)
        self.assertEqual(P["max_speed"], before["max_speed"])
        self.assertEqual(P["decel"], before["decel"])
        self.assertEqual(params.load()["max_speed"], 0.12)

    def test_a_long_stop_at_015_keeps_the_cap(self):
        runs = [coast(0.08, 0.05), coast(0.12, 0.08), coast(0.15, 1.5)]
        got = follow_speed.recommendation(runs, P)
        self.assertFalse(got["ok"])
        self.assertEqual(got["max_speed"], 0.12)
        self.assertIsNotNone(got["decel"])

    def test_a_missing_speed_does_not_unlock(self):
        got = follow_speed.recommendation([coast(0.08, 0.05), coast(0.12, 0.08)], P)
        self.assertFalse(got["ok"])
        self.assertEqual(got["max_speed"], 0.12)
        self.assertIsNone(got["decel"])


if __name__ == "__main__":
    unittest.main()
