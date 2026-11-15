import unittest

import params


class Gates(unittest.TestCase):
    def test_committed_file_has_speed_and_depth_open(self):
        p = params.load()
        self.assertTrue(params.speed_gates_open(p))  # measured in Task 2, 2026-11-13
        self.assertTrue(params.depth_gates_open(p))  # camera pitch from a wall at 1.29 m, 2026-11-13

    def test_braking_and_latency_are_the_online_estimates(self):
        # calib.BaseEstimates, trusted in all four runs of 2026-11-15: decel 0.135 to 0.157 m/s^2,
        # latency 0.393 to 0.421 s (the 2026-11-13 values 0.064 and 0.17 were far off).
        p = params.load()
        # decel: the cautious end of the corrected coast (0.09 to 0.11), below the online 0.135 to 0.157
        self.assertEqual((p["decel"], p["command_latency"]), (0.10, 0.40))
        self.assertIn("0.135 to 0.157", params.__doc__)
        self.assertIn("counted the delay a second", params.__doc__)
        self.assertIn("0.393 to 0.421", params.__doc__)

    def test_speed_gate_opens_when_the_three_values_are_measured(self):
        p = dict(params.load(), linear_cmd_scale=0.9, decel=0.5, stop_delay=0.2)
        self.assertTrue(params.speed_gates_open(p))


if __name__ == "__main__":
    unittest.main()
