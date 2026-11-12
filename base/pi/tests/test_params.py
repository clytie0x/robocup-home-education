import unittest

import params


class Gates(unittest.TestCase):
    def test_committed_file_has_speed_open_and_depth_closed(self):
        p = params.load()
        self.assertTrue(params.speed_gates_open(p))  # measured in Task 2, 2026-11-13
        self.assertFalse(params.depth_gates_open(p))  # camera pitch not measured yet

    def test_speed_gate_opens_when_the_three_values_are_measured(self):
        p = dict(params.load(), linear_cmd_scale=0.9, decel=0.5, stop_delay=0.2)
        self.assertTrue(params.speed_gates_open(p))


if __name__ == "__main__":
    unittest.main()
