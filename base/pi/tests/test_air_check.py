import sys
import unittest
from pathlib import Path

import params
import wheels

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import air_check  # noqa: E402

P = params.load()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(seconds, 0.001)


class FakePort:
    """Records writes; each O# answers the next odometry line, the last one repeats."""

    def __init__(self, lines):
        self.lines, self.writes = list(lines), []

    def write(self, data):
        self.writes.append(data)

    def reset_input_buffer(self):
        pass

    def read_until(self, _end):
        return (self.lines.pop(0) if len(self.lines) > 1 else self.lines[0]).encode()

    def close(self):
        pass


def odom(x, th=0.0):
    return "Ox:%+07.2f,y:+000.00,th:%+07.2f#" % (x, th)


class AirCheckTests(unittest.TestCase):
    def test_rates_use_the_last_two_seconds_in_true_units(self):
        log = [(t * 0.5, 0.1 * t * 0.5, 0.0, 0.0) for t in range(9)]  # 0.1 m/s for 4 s
        r = air_check.rates(log, P)
        self.assertAlmostEqual(r["fw_speed"], 0.1, places=3)
        self.assertAlmostEqual(r["true_speed"], round(0.1 / P["odom_distance_scale"], 3), places=3)
        self.assertEqual(r["fw_yaw_rate"], 0.0)

    def test_moved(self):
        self.assertFalse(air_check.moved((1.0, 0.0, 0.0), (1.001, 0.0, 0.0)))
        self.assertTrue(air_check.moved((1.0, 0.0, 0.0), (1.05, 0.0, 0.0)))
        self.assertTrue(air_check.moved((0.0, 0.0, 0.0), (0.0, 0.0, 0.2)))

    def test_hold_sends_past_the_clamp_and_ends_with_stop(self):
        clock = Clock()
        port = FakePort([odom(0.0)])
        w = wheels.Wheels(dict(P, odom_distance_scale=1.0, odom_yaw_scale=1.0), port=port, clock=clock,
                          start_thread=False)
        log = air_check.hold(w, b"F037|+000#", 1.0, clock=clock, sleep=clock.sleep)
        self.assertGreater(len(log), 5)
        self.assertIn(b"F037|+000#", port.writes)  # not clamped to max_speed 0.12
        self.assertEqual(port.writes[-1], b"S#")
        self.assertEqual(w.last, b"S#")

    def test_silence_sends_one_packet_and_the_watchdog_stays_quiet(self):
        clock = Clock()
        port = FakePort([odom(1.00), odom(1.05)])
        w = wheels.Wheels(P, port=port, clock=clock, start_thread=False)
        w.last = b"F010|+000#"  # a previous command must not be repeated during the silence
        real_sleep = clock.sleep

        def sleep(seconds):  # the watchdog ticks during the silence, as its thread would
            for _ in range(int(seconds / wheels.PERIOD)):
                w.tick()
            real_sleep(seconds)

        result = air_check.silence(w, clock=clock, sleep=sleep)
        drive = [d for d in port.writes if d.startswith(b"F")]
        self.assertEqual(drive, [b"F008|+000#"])
        self.assertEqual(port.writes[-1], b"S#")
        self.assertTrue(result["still_moving_after_silence"])

    def test_silence_stops_when_odometry_fails(self):
        clock = Clock()
        port = FakePort(["garbage#"])
        w = wheels.Wheels(P, port=port, clock=clock, start_thread=False)
        with self.assertRaises(ValueError):
            air_check.silence(w, clock=clock, sleep=clock.sleep)
        self.assertEqual(port.writes[-1], b"S#")


if __name__ == "__main__":
    unittest.main()
