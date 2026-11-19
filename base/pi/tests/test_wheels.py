import math
import unittest

import params
import wheels

P = dict(params.load(), linear_cmd_scale=None, max_speed=0.15, odom_distance_scale=0.902)  # the plan's test values, not the measured ones


class FakePort:
    timeout = 0.2  # as wheels.Wheels opens the real port

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


class StatusPort(FakePort):
    """The new firmware (0c020e4): E# answers with the encoder counts, the PWM and the fault flags."""

    def __init__(self, status=b"Eleft:+0000120,right:-0000007,pwm:+076,+076,flags:03#"):
        super().__init__()
        self.status = status
        self.timeouts = []  # the read timeout at each read

    def read_until(self, _end):
        self.timeouts.append(self.timeout)
        return self.status if self.written[-1] == b"E#" else self.odom


class FirmwareStatus(unittest.TestCase):
    def test_parse_status_reads_counts_pwm_and_hex_flags(self):
        self.assertEqual(wheels.parse_status("Eleft:+0000120,right:-0000007,pwm:+076,-012,flags:03#"),
                         {"left": 120, "right": -7, "pwm": [76, -12], "flags": 3})
        self.assertEqual(wheels.parse_status("Eleft:+0000000,right:+0000000,pwm:+000,+000,flags:0c#")["flags"], 12)

    def test_an_unparsable_status_is_an_error(self):
        # the old firmware has no E#: nothing, or an odometry line, comes back
        for line in ("", "Ox:+000.90,y:+000.00,th:+000.86#", "Eleft:+1,right:+2,pwm:+3,+4#",
                     "Eleft:+1,right:+2,pwm:+3,+4,flags:zz#", "Eleft:+1,right:+2,pwm:+3,+4,flags:03", "garbage#"):
            with self.assertRaises(ValueError, msg=line):
                wheels.parse_status(line)

    def test_read_status_asks_e_with_a_short_timeout_and_puts_the_timeout_back(self):
        port = StatusPort()
        w = wheels.Wheels(P, port=port, start_thread=False)
        self.assertEqual(w.read_status()["flags"], 3)
        self.assertEqual([d for d in port.written if d != b"S#"], [b"E#"])
        self.assertEqual(port.timeouts, [wheels.STATUS_TIMEOUT])
        self.assertEqual(port.timeout, 0.2)
        port.status = b""  # the old firmware: no answer before the timeout
        with self.assertRaises(ValueError):
            w.read_status()
        self.assertEqual(port.timeout, 0.2)
        self.assertAlmostEqual(w.read_odom()[0], 0.90 / 0.902, places=3)  # odometry is unaffected


class StuckWrite(StatusPort):
    """A USB write that times out on E#."""

    def write(self, data):
        if data == b"E#":
            raise OSError("write timeout")
        super().write(data)


class StatusTimeout(unittest.TestCase):
    def test_a_failed_e_write_still_puts_the_timeout_back(self):
        port = StuckWrite()
        w = wheels.Wheels(P, port=port, start_thread=False)
        with self.assertRaises(OSError):
            w.read_status()
        self.assertEqual(port.timeout, 0.2)


class Stall(unittest.TestCase):
    """The Pi's own check that the wheels turn: works with any firmware (StallWatch)."""

    def feed(self, watch, sent, poses, start=0.0):
        """One update per 0.1 s; returns the times at which the watch said "not turning"."""
        return [round(start + 0.1 * i, 1) for i, pose in enumerate(poses)
                if watch.update(start + 0.1 * i, sent, pose)]

    def test_a_command_that_does_not_move_the_odometry_for_a_second_is_not_turning(self):
        watch = wheels.StallWatch()
        self.assertEqual(self.feed(watch, (0.114, 0.0), [(0.0, 0.0, 0.0)] * 15), [1.0, 1.1, 1.2, 1.3, 1.4])

    def test_a_turn_on_the_spot_counts_too(self):
        watch = wheels.StallWatch()
        self.assertEqual(self.feed(watch, (0.0, 0.3), [(0.0, 0.0, 0.0)] * 11), [1.0])

    def test_a_base_that_moves_is_turning(self):
        watch = wheels.StallWatch()
        slow = [(0.03 * 0.1 * i, 0.0, 0.0) for i in range(30)]  # the smallest watched speed, really driven
        self.assertEqual(self.feed(watch, (0.03, 0.0), slow), [])
        turning = [(0.0, 0.0, 0.15 * 0.1 * i) for i in range(30)]
        self.assertEqual(self.feed(wheels.StallWatch(), (0.0, 0.15), turning), [])

    def test_a_turn_that_reverses_inside_the_window_is_turning(self):
        # Lane D, 2026-11-15: the search spin turns back at +-200 deg; over the 1 s window the net yaw was near 0
        # and the watch said "not turning" (and held the robot) while the wheels turned all the time.
        poses, yaw = [], 0.0
        for i in range(30):  # 0.3 rad/s left for 0.5 s, right for 0.5 s, and so on
            yaw += 0.03 if (i // 5) % 2 == 0 else -0.03
            poses.append((0.0, 0.0, yaw))
        self.assertEqual(self.feed(wheels.StallWatch(), (0.0, 0.3), poses), [])

    def test_a_drive_that_comes_back_inside_the_window_is_moving(self):
        poses, x = [], 0.0
        for i in range(30):  # 0.05 m/s forward and back (an escape), 0.4 s each way
            x += 0.005 if (i // 4) % 2 == 0 else -0.005
            poses.append((x, 0.0, 0.0))
        self.assertEqual(self.feed(wheels.StallWatch(), (0.05, 0.0), poses), [])

    def test_small_commands_are_not_watched(self):
        watch = wheels.StallWatch()
        self.assertEqual(self.feed(watch, (0.02, 0.1), [(0.0, 0.0, 0.0)] * 30), [])

    def test_the_second_must_be_continuous(self):
        watch = wheels.StallWatch()
        self.assertEqual(self.feed(watch, (0.1, 0.0), [(0.0, 0.0, 0.0)] * 9), [])
        self.assertFalse(watch.update(0.9, (0.0, 0.0), (0.0, 0.0, 0.0)))  # a stop between
        self.assertEqual(self.feed(watch, (0.1, 0.0), [(0.0, 0.0, 0.0)] * 10, start=1.0), [1.9])

    def test_wheels_that_stop_turning_mid_drive_are_found_a_second_later(self):
        watch = wheels.StallWatch()
        driving = [(0.2 * 0.1 * i, 0.0, 0.0) for i in range(20)]  # 2 s at 0.2 m/s, then the power dies
        dead = [driving[-1]] * 15
        said = self.feed(watch, (0.2, 0.0), driving + dead)
        self.assertEqual(said[0], 2.9)  # at 2.8 the pose 1 s before was still 2 cm back: not yet
        watch.reset()
        self.assertFalse(watch.update(10.0, (0.2, 0.0), dead[0]))  # reset: a new window


class Least(unittest.TestCase):
    """The smallest commands that move the wheels (hardware assessment, "Lowest speeds that turn the wheels")."""

    def test_measured_numbers(self):
        self.assertEqual((wheels.MIN_V, wheels.MIN_TURN), (0.01, 0.10))
        self.assertAlmostEqual(math.degrees(wheels.TURN_DEADBAND), 2.0)

    def test_a_turn_on_the_spot_is_0_or_at_least_min_turn(self):
        self.assertEqual(wheels.least(0.0, 0.07, math.radians(44)), (0.0, 0.10))
        self.assertEqual(wheels.least(0.0, -0.05, None), (0.0, -0.10))  # no heading of its own: still turns
        self.assertEqual(wheels.least(0.0, 0.3, math.radians(44)), (0.0, 0.3))
        self.assertEqual(wheels.least(0.0, 0.0, math.radians(44)), (0.0, 0.0))

    def test_a_small_heading_error_on_the_spot_turns_nothing(self):
        self.assertEqual(wheels.least(0.0, 0.07, math.radians(1.9)), (0.0, 0.0))
        self.assertEqual(wheels.least(0.0, 0.4, math.radians(-1.0)), (0.0, 0.0))
        self.assertEqual(wheels.least(0.0, 0.07, math.radians(2.1)), (0.0, 0.10))

    def test_forward_is_0_or_at_least_min_v_and_an_arc_keeps_its_turn(self):
        self.assertEqual(wheels.least(0.004, 0.0, None), (0.01, 0.0))
        self.assertEqual(wheels.least(0.004, 0.03, math.radians(1.0)), (0.01, 0.03))  # driving: an arc
        self.assertEqual(wheels.least(0.05, 0.02, None), (0.05, 0.02))
        self.assertEqual(wheels.least(-0.06, 0.0, None), (-0.06, 0.0))  # backing (an escape) is left alone

    def test_nan_stays_nan_for_the_stops(self):
        v, w = wheels.least(float("nan"), 0.05, None)
        self.assertTrue(math.isnan(v))
        v, w = wheels.least(0.0, float("nan"), None)
        self.assertTrue(math.isnan(w))


class CurvatureAndBgOdom(unittest.TestCase):
    def test_clamp_curvature_preserves_turning_radius(self):
        p = dict(P, max_speed=0.12, max_turn_rate=0.48)
        v, w = wheels.clamp_curvature(0.12, 0.96, p)  # w is 2x limit -> both halve
        self.assertAlmostEqual(v, 0.06, places=6)
        self.assertAlmostEqual(w, 0.48, places=6)
        self.assertAlmostEqual(v / w, 0.12 / 0.96, places=6)
        self.assertEqual(wheels.clamp_curvature(0.10, -0.20, p), (0.10, -0.20))

    def test_background_odom_returns_cached_and_rejects_stale(self):
        port, clock = FakePort(), Clock()
        w = wheels.Wheels(P, port=port, clock=clock, start_thread=False, poll_odom=True)
        pose = w.poll_odom_once()
        self.assertIsNotNone(pose)
        clock.now = 0.10
        written_before = len(port.written)
        self.assertEqual(w.cached_odom(), pose)
        self.assertEqual(len(port.written), written_before)  # zero serial wait
        clock.now = 0.40
        with self.assertRaises(ValueError):
            w.cached_odom(max_age=0.25)


if __name__ == "__main__":
    unittest.main()
