import unittest

import params


class Gates(unittest.TestCase):
    def test_gates_are_closed_in_the_committed_file(self):
        p = params.load()
        self.assertFalse(params.speed_gates_open(p))
        self.assertFalse(params.depth_gates_open(p))

    def test_speed_gate_opens_when_the_three_values_are_measured(self):
        p = dict(params.load(), linear_cmd_scale=0.9, decel=0.5, stop_delay=0.2)
        self.assertTrue(params.speed_gates_open(p))


if __name__ == "__main__":
    unittest.main()
