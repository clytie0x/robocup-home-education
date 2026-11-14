import math
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
        p = dict(P, linear_cmd_scale=0.9, angular_cmd_scale=0.862, max_speed=0.6)  # the old firmware's turn scale
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


class ReversePacket(unittest.TestCase):
    def test_reverse_packet_formats_like_forward_with_b(self):
        self.assertEqual(wheels.reverse_packet(0.04, 0.0, P), b"B004|+000#")
        self.assertEqual(wheels.reverse_packet(0.06, -0.30, P), b"B006|-030#")
        p = dict(P, linear_cmd_scale=0.9, angular_cmd_scale=0.862)  # the same scaling as packet()
        self.assertEqual(wheels.reverse_packet(0.06, 0.40, p), b"B005|+034#")

    def test_reverse_packet_caps_the_turn_and_rounds_nothing_to_a_stop(self):
        self.assertEqual(wheels.reverse_packet(0.05, 2.0, P), b"B005|+049#")  # max_turn_rate 0.49
        self.assertEqual(wheels.reverse_packet(0.004, 0.0, P), b"S#")

    def test_reverse_packet_refuses_above_the_cap_at_zero_and_non_finite(self):
        self.assertEqual(wheels.REVERSE_MAX, 0.06)
        for speed, w in ((0.061, 0.0), (0.12, 0.0), (0.0, 0.0), (-0.04, 0.0), (float("nan"), 0.0),
                         (float("inf"), 0.0), (0.04, float("nan")), (0.04, float("-inf"))):
            with self.assertRaises(ValueError):
                wheels.reverse_packet(speed, w, P)

    def test_packet_still_refuses_backward(self):
        for v in (-0.04, -0.06, -1e-6):
            with self.assertRaises(ValueError):
                wheels.packet(v, 0.0, P)

    def test_send_reverse_writes_b_and_the_watchdog_repeats_then_stops_it(self):
        port, clock = FakePort(), Clock()
        w = wheels.Wheels(P, port=port, clock=clock, start_thread=False)
        self.assertEqual(w.send_reverse(0.04, 0.0), b"B004|+000#")
        clock.now = 0.1
        w.tick()
        self.assertEqual(port.written[-1], b"B004|+000#")
        clock.now = 0.35
        w.tick()
        self.assertEqual(port.written[-wheels.STOP_REPEATS:], [b"S#"] * wheels.STOP_REPEATS)
        with self.assertRaises(ValueError):
            w.send_reverse(0.07, 0.0)

    def test_send_reverse_backs_straight_only(self):
        port = FakePort()
        w = wheels.Wheels(P, port=port, start_thread=False)
        self.assertEqual(w.send_reverse(0.04, -wheels.REVERSE_TURN_MAX), b"B004|-005#")
        for turn in (0.06, -0.06, 0.25, float("nan")):  # rear_clear checks a straight path only
            with self.assertRaises(ValueError):
                w.send_reverse(0.04, turn)
        self.assertEqual(w.last, b"B004|-005#")

    def test_not_safe_run_text_matches_the_new_firmware(self):
        self.assertNotIn("no command timeout", wheels.NOT_SAFE_RUN)
        self.assertIn("0.3 s", wheels.NOT_SAFE_RUN)


class Odom(unittest.TestCase):
    def test_parse_odom_reads_the_firmware_units(self):
        self.assertEqual(wheels.parse_odom("Ox:+000.90,y:-000.10,th:+000.86#"), (0.90, -0.10, 0.86))

    def test_a_step_is_scaled_and_follows_the_true_heading(self):
        d, k = P["odom_distance_scale"], P["odom_yaw_scale"]
        step = lambda pose, a, b: wheels.true_step(pose, a, b, P)
        # straight, with the firmware's heading far from zero (it never resets during a run)
        x, y, th = step((0, 0, 0), (10, 10, 23.4), (10 + 0.9 * math.cos(23.4), 10 + 0.9 * math.sin(23.4), 23.4))
        self.assertAlmostEqual(x, 0.9 / d, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(th, 0.0, places=9)
        # a turn on the spot, then straight on: the drive goes where the true heading points
        pose = step((0, 0, 0), (0, 0, 0), (0, 0, k * 1.0))
        self.assertAlmostEqual(pose[2], 1.0, places=9)
        x, y, th = step(pose, (0, 0, k), (0.9 * math.cos(k), 0.9 * math.sin(k), k))
        self.assertAlmostEqual(x, 0.9 / d * math.cos(1.0), places=9)
        self.assertAlmostEqual(y, 0.9 / d * math.sin(1.0), places=9)

    def test_broken_odom_line_is_an_error(self):
        for line in ("", "Ox:+000.90,y:+000.00#", "garbage", "Ox:a,y:+0,th:+0#", "x:+0,y:+0,th:+0#", "Ox:+0,y:+0,th:+0"):
            with self.assertRaises(ValueError):
                wheels.parse_odom(line)

    def test_non_finite_odom_is_an_error(self):
        for line in ("Ox:nan,y:+0,th:+0#", "Ox:+0,y:inf,th:+0#", "Ox:+0,y:+0,th:-inf#"):
            with self.assertRaises(ValueError):
                wheels.parse_odom(line)


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
        self.assertEqual(port.written[-wheels.STOP_REPEATS:], [b"S#"] * wheels.STOP_REPEATS)
        quiet = len(port.written)
        w.tick()  # stopped already: the port stays quiet, so a silence test means silence
        self.assertEqual(len(port.written), quiet)

    def test_opening_the_port_and_every_stop_send_s_more_than_once(self):
        port = FakePort()
        w = wheels.Wheels(P, port=port, start_thread=False)
        self.assertEqual(port.written, [b"S#"] * wheels.STOP_REPEATS)  # whatever the base was doing
        port.written.clear()
        w.stop()
        self.assertEqual(port.written, [b"S#"] * wheels.STOP_REPEATS)

    def test_read_odom_asks_and_parses(self):
        port = FakePort()
        w = wheels.Wheels(P, port=port, start_thread=False)
        self.assertAlmostEqual(w.read_odom()[0], 0.90 / 0.902, places=3)
        self.assertEqual([d for d in port.written if d != b"S#"], [b"O#"])  # opening S# aside

    def test_an_impossible_step_is_refused_and_a_reset_ends_the_run(self):
        port = FakePort()
        w = wheels.Wheels(P, port=port, start_thread=False)
        first = w.read_odom()
        port.odom = b"Ox:+00090,y:+000.00,th:+000.86#"  # the decimal point lost: x jumps to 90
        with self.assertRaises(ValueError):
            w.read_odom()  # four tries, all the same jump
        port.odom = b"Ox:+000.91,y:+000.00,th:+000.86#"
        self.assertAlmostEqual(math.dist(w.read_odom()[:2], first[:2]), 0.01 / P["odom_distance_scale"], places=9)
        port.odom = b"Ox:+000.00,y:+000.00,th:+000.00#"  # the firmware restarted
        with self.assertRaises(ValueError):
            w.read_odom()

    def test_read_odom_gives_up_after_four_bad_lines(self):
        port = FakePort(odom=b"junk#")
        w = wheels.Wheels(P, port=port, start_thread=False)
        with self.assertRaises(ValueError):
            w.read_odom()
        self.assertEqual(port.written.count(b"O#"), 4)


if __name__ == "__main__":
    unittest.main()
