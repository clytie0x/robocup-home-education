import unittest

import numpy

import approach


class ApproachTest(unittest.TestCase):
    def test_finds_the_nearest_thing_ahead_and_follows_it(self):
        pts = numpy.array([[0.53, -0.13], [0.54, -0.12], [0.9, 0.5], [-0.5, 0.0], [0.6, 0.6]])
        self.assertEqual(approach.target_of(pts), (0.53, -0.13))
        moved = pts - [0.02, 0.0]
        self.assertAlmostEqual(approach.target_of(moved, (0.53, -0.13))[0], 0.51)
        self.assertIsNone(approach.target_of(pts + [0.5, 0.0], (0.53, -0.13)))  # jumped: lost

    def test_nothing_ahead(self):
        self.assertIsNone(approach.target_of(numpy.array([[0.3, 0.6], [-0.4, 0.0]])))

    def test_something_else_close_ahead_stops_it(self):
        target = (0.40, 0.0)
        self.assertFalse(approach.blocked_by_other(numpy.array([[0.40, 0.0], [0.41, 0.02]]), target))
        self.assertTrue(approach.blocked_by_other(numpy.array([[0.40, 0.0], [0.25, 0.15]]), target))
        self.assertFalse(approach.blocked_by_other(numpy.array([[0.40, 0.0], [0.25, 0.30]]), target))  # beside

    def test_creep_speed_slows_and_stops_at_the_distance(self):
        self.assertEqual(approach.creep_speed(0.60, 0.37), approach.CREEP)
        self.assertAlmostEqual(approach.creep_speed(0.385, 0.37), 0.015)
        self.assertEqual(approach.creep_speed(0.372, 0.37), 0.0)


if __name__ == "__main__":
    unittest.main()
