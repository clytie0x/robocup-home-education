import sys
import unittest
from pathlib import Path

import numpy

import params

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import lap_check  # noqa: E402

P = params.load()


def wall_along(y, x0=-1.0, x1=2.0):
    """Points every 1 cm on the line y = const (base frame, x forward, y left)."""
    xs = numpy.arange(x0, x1, 0.01)
    return numpy.stack([xs, numpy.full_like(xs, y)], axis=1)


def wall_across(x, y0=-1.0, y1=1.0):
    ys = numpy.arange(y0, y1, 0.01)
    return numpy.stack([numpy.full_like(ys, x), ys], axis=1)


def run(points):
    v, w = lap_check.control(*lap_check.zones(points))
    return (v, w), lap_check.choose(points, v, w, P)


class ControlAndChoose(unittest.TestCase):
    def test_wall_ahead_turns_left_on_the_spot(self):
        wanted, chosen = run(wall_across(0.50))  # 0.30 m ahead of the front edge
        self.assertEqual(wanted, (0.0, lap_check.TURN))
        self.assertEqual(chosen, (0.0, lap_check.TURN, None))

    def test_corridor_goes_straight_at_top_speed(self):
        wanted, chosen = run(numpy.vstack([wall_along(-0.45), wall_along(0.45)]))  # 0.25 m each side
        self.assertEqual(wanted, (lap_check.TOP, 0.0))
        self.assertEqual(chosen, (lap_check.TOP, 0.0, None))

    def test_open_curves_right_to_find_the_wall(self):
        wanted, chosen = run(numpy.zeros((0, 2)))
        self.assertEqual(wanted, (0.08, -0.35))
        self.assertEqual(chosen, (0.08, -0.35, None))

    def test_wall_3_cm_right_holds(self):
        # The wall is inside the front strip (|y| < 0.25), so control turns left on the spot. Every
        # turn swings a corner (reach 0.256 m) into it, and must_stop or footprint.sweep_clear refuses
        # even a straight move along it: points facing a cut corner come closer than they started.
        wanted, chosen = run(wall_along(-0.23, -0.4, 0.6))
        self.assertEqual(wanted, (0.0, lap_check.TURN))
        self.assertEqual(chosen, (0.0, 0.0, "hold: no safe move"))

    def test_wall_3_cm_ahead_holds_it_never_backs_off(self):
        # backing up is tools/back_up.py's job alone (rear_clear, cable check); the lap holds
        _, chosen = run(wall_across(0.23))
        self.assertEqual(chosen, (0.0, 0.0, "hold: no safe move"))

    def test_no_path_can_produce_a_backward_command(self):
        self.assertTrue(all(v >= 0 for v, _ in lap_check.ESCAPES))
        scenes = [wall_across(x) for x in (0.21, 0.23, 0.3, 0.5)] + [wall_along(y) for y in (-0.22, -0.3, 0.22)]
        scenes += [numpy.vstack([wall_across(0.23), wall_along(-0.23), wall_along(0.23)]), numpy.zeros((0, 2))]
        for points in scenes:
            for v, w in [lap_check.control(*lap_check.zones(points)), (0.0, 0.3), (0.1, 0.0)]:
                self.assertGreaterEqual(lap_check.choose(points, v, w, P)[0], 0.0)
        with self.assertRaises(ValueError):  # a negative wanted speed reaches must_stop, which refuses it
            lap_check.sweep_ok(numpy.zeros((0, 2)), -0.05, 0.0, P)
        source = (Path(lap_check.__file__)).read_text()
        for raw in ("port.write", "send_reverse", "reverse_packet", '"B'):
            self.assertNotIn(raw, source)  # every command goes through Wheels.send, whose packet() refuses v < 0

    def test_closest_body_gap(self):
        self.assertAlmostEqual(lap_check.closest(wall_along(-0.45)), 0.25)
        self.assertEqual(lap_check.closest(numpy.zeros((0, 2))), float("inf"))


if __name__ == "__main__":
    unittest.main()
