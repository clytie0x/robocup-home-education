import unittest

import params
import wheels

P = dict(params.load(), linear_cmd_scale=None, max_speed=0.15, odom_distance_scale=0.902)  # the plan's test values, not the measured ones


class FakePort:
    def __init__(self, odom=b"Ox:+000.90,y:+000.00,th:+000.86#"):
        self.written = []
        self.odom = odom

    def write(self, data):
        self.written.append(data)

    def read_until(self, _end):
        return self.odom

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Packet(unittest.TestCase):
    def test_packet_scales_and_formats(self):
        p = dict(P, linear_cmd_scale=0.9, max_speed=0.6)
        self.assertEqual(wheels.packet(0.30, 0.0, p), b"F027|+000#")
        self.assertEqual(wheels.packet(0.0, -0.40, p), b"F000|-034#")  # 0.40 * 0.862
        self.assertEqual(wheels.packet(0.0, 0.0, p), b"S#")

    def test_packet_is_capped_and_never_backward(self):
        self.assertEqual(wheels.packet(2.0, 0.0, P), b"F015|+000#")  # max_speed 0.15, scale null
        with self.assertRaises(ValueError):
            wheels.packet(-0.1, 0.0, P)

    def test_non_finite_command_is_an_error(self):
        for v, w in ((0.0, float("nan")), (float("nan"), 0.0), (float("inf"), 0.0), (0.0, float("-inf"))):
            with self.assertRaises(ValueError):
                wheels.packet(v, w, P)

    def test_command_that_rounds_to_zero_is_a_stop(self):
        self.assertEqual(wheels.packet(0.005, 0.0, P), b"S#")
        self.assertEqual(wheels.packet(0.0, -0.005, P), b"S#")


class Odom(unittest.TestCase):
    def test_parse_odom_applies_the_measured_scales(self):
        x, y, yaw = wheels.parse_odom("Ox:+000.90,y:+000.00,th:+000.86#", P)
        self.assertAlmostEqual(x, 0.90 / 0.902, places=3)
        self.assertAlmostEqual(yaw, 0.86 / 0.862, places=3)

    def test_broken_odom_line_is_an_error(self):
        for line in ("", "Ox:+000.90,y:+000.00#", "garbage", "Ox:a,y:+0,th:+0#", "x:+0,y:+0,th:+0#", "Ox:+0,y:+0,th:+0"):
            with self.assertRaises(ValueError):
                wheels.parse_odom(line, P)

    def test_non_finite_odom_is_an_error(self):
        for line in ("Ox:nan,y:+0,th:+0#", "Ox:+0,y:inf,th:+0#", "Ox:+0,y:+0,th:-inf#"):
            with self.assertRaises(ValueError):
                wheels.parse_odom(line, P)


class Watchdog(unittest.TestCase):
    def test_silence_becomes_a_stop(self):
        port, clock = FakePort(), Clock()
        w = wheels.Wheels(P, port=port, clock=clock, start_thread=False)
        w.send(0.1, 0.0)
        clock.now = 0.1
        w.tick()
        self.assertEqual(port.written[-1], b"F010|+000#")
        clock.now = 0.35
        w.tick()
        self.assertEqual(port.written[-1], b"S#")

    def test_read_odom_asks_and_parses(self):
        port = FakePort()
        w = wheels.Wheels(P, port=port, start_thread=False)
        self.assertAlmostEqual(w.read_odom()[0], 0.90 / 0.902, places=3)
        self.assertEqual(port.written, [b"O#"])

    def test_read_odom_gives_up_after_four_bad_lines(self):
        port = FakePort(odom=b"junk#")
        w = wheels.Wheels(P, port=port, start_thread=False)
        with self.assertRaises(ValueError):
            w.read_odom()
        self.assertEqual(port.written.count(b"O#"), 4)


if __name__ == "__main__":
    unittest.main()
