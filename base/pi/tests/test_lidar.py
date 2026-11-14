# Copied from xohm@pi:~/pi-drive/test_lidar.py on 2026-11-13.
import math
import time
import unittest
from pathlib import Path

from lidar import (START, STOP, FRONT_LASER_DEG, LidarSession, ScanStream, checksum_ok, complete,
                   parse_revolutions, parse_samples)

HERE = Path(__file__).resolve().parent
REAL = bytes.fromhex((HERE / "fixtures" / "lidar-4-packets.hex").read_text().strip())
ODD = bytes.fromhex((HERE / "fixtures" / "lidar-odd-packets.hex").read_text().strip())


def packet(first_deg, last_deg, raws, ct=0):
    count = len(raws)
    first = (int(round(first_deg * 64)) << 1) | 1
    last = (int(round(last_deg * 64)) << 1) | 1
    check = 0x55AA ^ (ct | (count << 8)) ^ first ^ last
    body = b""
    for raw in raws:
        check ^= 60 ^ raw
        body += bytes([60, raw & 255, raw >> 8])
    head = bytes([0xAA, 0x55, ct, count])
    return head + first.to_bytes(2, "little") + last.to_bytes(2, "little") + check.to_bytes(2, "little") + body


def sweep(start_deg, degrees, raw=4000):
    out = b""
    angle = start_deg
    left = degrees
    while left > 0:
        count = min(24, left)
        out += packet(angle % 360, (angle + count - 1) % 360, [raw] * count)
        angle += count
        left -= count
    return out


class RealPacketTest(unittest.TestCase):
    def test_every_real_packet_passes_the_checksum(self):
        for start in range(0, len(REAL), 85):
            self.assertTrue(checksum_ok(REAL[start : start + 85]))

    def test_one_flipped_byte_fails_the_checksum(self):
        bad = bytearray(REAL[:85])
        bad[40] ^= 0x10
        self.assertFalse(checksum_ok(bytes(bad)))

    def test_a_single_sample_packet_is_a_packet(self):
        self.assertTrue(checksum_ok(ODD))

    def test_four_packets_give_one_hundred_samples_with_smooth_ranges(self):
        samples = parse_samples(REAL)
        self.assertEqual(len(samples), 100)
        steps = sorted(abs(a[1] - b[1]) for a, b in zip(samples, samples[1:]) if a[1] and b[1])
        self.assertLess(steps[len(steps) // 2], 0.05)

    def test_sample_angles_advance_through_each_packet(self):
        samples = parse_samples(REAL[:85])
        angles = [deg for deg, _ in samples]
        self.assertTrue(all(0 < (b - a) % 360 < 5 for a, b in zip(angles, angles[1:])))


class StreamTest(unittest.TestCase):
    def test_garbage_and_a_corrupt_packet_are_skipped(self):
        good = packet(10, 33, [4000] * 24)
        bad = bytearray(packet(40, 63, [4000] * 24))
        bad[20] ^= 0xFF
        samples = parse_samples(b"\x01\x02\xaa\x55\x99" + good + bytes(bad) + good)
        self.assertEqual(len(samples), 48)

    def test_distance_is_a_quarter_millimetre_count_in_metres(self):
        samples = parse_samples(packet(10, 33, [4000] * 24))
        self.assertAlmostEqual(samples[0][1], 1.0)

    def test_the_longest_revolution_is_returned_without_a_wrap_jump(self):
        scan = parse_revolutions(sweep(0, 360) + sweep(0, 90))
        self.assertEqual(len(scan), 360)

    def test_the_measured_front_is_the_laser_angle_of_base_zero(self):
        self.assertAlmostEqual(FRONT_LASER_DEG, 178.7)

    def test_laser_angles_run_clockwise_so_robot_left_is_a_smaller_laser_angle(self):
        def base_at(laser):
            scan = parse_revolutions(packet(laser, laser, [4000]))
            return scan[0][0]

        self.assertAlmostEqual(base_at(178.7), 0.0, places=1)
        self.assertAlmostEqual(base_at(148.7), 30.0, places=1)
        self.assertAlmostEqual(base_at(208.7), -30.0, places=1)

    def test_front_of_the_base_follows_the_forward_offset(self):
        scan = parse_revolutions(sweep(0, 360))
        angles = sorted(deg for deg, _ in scan)
        self.assertTrue(all(-180 < deg <= 180 for deg in angles))
        self.assertEqual(len({round(deg) for deg in angles}), 360)


class StreamingTest(unittest.TestCase):
    def feed_in_chunks(self, stream, data, size):
        for start in range(0, len(data), size):
            stream.feed(data[start : start + size])

    def test_chunks_of_any_size_give_the_same_revolution_as_one_pass(self):
        data = sweep(0, 360) + sweep(0, 90)
        expected = parse_revolutions(data)
        for size in (1, 7, 37, 85, 500, len(data)):
            stream = ScanStream(clock=lambda: 0.0)
            self.feed_in_chunks(stream, data, size)
            self.assertEqual(stream.latest(), expected, size)

    def test_a_corrupt_packet_between_good_ones_is_skipped(self):
        bad = bytearray(packet(40, 63, [4000] * 24))
        bad[20] ^= 0xFF
        data = sweep(0, 360) + bytes(bad) + sweep(0, 90)
        stream = ScanStream(clock=lambda: 0.0)
        self.feed_in_chunks(stream, data, 50)
        self.assertEqual(len(stream.latest()), 360)

    def test_no_complete_revolution_yet_gives_an_empty_scan(self):
        stream = ScanStream(clock=lambda: 0.0)
        stream.feed(sweep(0, 100))
        self.assertEqual(stream.latest(), [])

    def test_a_scan_that_stopped_updating_goes_stale(self):
        now = [0.0]
        stream = ScanStream(clock=lambda: now[0])
        stream.feed(sweep(0, 360) + sweep(0, 30))
        self.assertEqual(len(stream.latest()), 360)
        now[0] = 0.4
        self.assertEqual(len(stream.latest()), 360)
        now[0] = 0.6
        self.assertEqual(stream.latest(), [])

    def test_a_revolution_with_a_hole_does_not_replace_the_last_good_one(self):
        now = [0.0]
        stream = ScanStream(clock=lambda: now[0])
        stream.feed(sweep(0, 360))                    # one good revolution
        stream.feed(sweep(0, 100) + sweep(140, 220))  # the next one loses 40 deg of packets
        stream.feed(sweep(0, 5))                      # the wrap that would promote it
        self.assertEqual(len(stream.latest()), 360)   # the good one is kept, the holed one dropped
        now[0] = 0.6
        self.assertEqual(stream.latest(), [])         # and it goes stale, so the robot is told nothing

    def test_each_feed_only_parses_the_new_bytes(self):
        stream = ScanStream(clock=lambda: 0.0)
        data = sweep(0, 360) + sweep(0, 30)
        stream.feed(data)
        before = stream.parsed_bytes
        stream.feed(sweep(30, 24))
        self.assertEqual(stream.parsed_bytes - before, len(sweep(30, 24)))


class FakeHandle:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.written = []
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(bytes(data))

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, count):
        if not self.chunks:
            time.sleep(0.05)
            return b""
        return self.chunks.pop(0)

    def close(self):
        self.closed = True


class SessionTest(unittest.TestCase):
    def test_the_session_starts_the_lidar_and_returns_a_complete_scan(self):
        data = sweep(0, 360) + sweep(0, 30)
        handle = FakeHandle([data[:5000], data[5000:]])
        session = LidarSession(handle=handle)
        self.assertEqual(handle.written[0], START)
        scan = []
        for _ in range(2):
            scan = session.latest()
        self.assertEqual(len(scan), 360)

    def test_close_stops_the_lidar_and_the_port(self):
        handle = FakeHandle([])
        LidarSession(handle=handle).close()
        self.assertEqual(handle.written[-1], STOP)
        self.assertTrue(handle.closed)

    def test_nothing_waiting_does_not_block_the_loop(self):
        session = LidarSession(handle=FakeHandle([]))
        started = time.monotonic()
        session.latest()
        self.assertLess(time.monotonic() - started, 0.02)

    def test_a_dead_lidar_returns_no_scan(self):
        handle = FakeHandle([])
        self.assertEqual(LidarSession(handle=handle).latest(), [])


if __name__ == "__main__":
    unittest.main()


class CompleteTest(unittest.TestCase):
    def test_a_full_circle_is_complete_and_a_hole_in_it_is_not(self):
        full = [(float(deg) - 180.0, 1.0) for deg in range(360)]
        self.assertTrue(complete(full))
        self.assertFalse(complete([point for point in full if not 0.0 <= point[0] < 40.0]))
        self.assertFalse(complete([]))
        self.assertFalse(complete([(0.0, 1.0)]))

    def test_a_hole_behind_the_robot_counts_too(self):
        self.assertFalse(complete([(float(deg), 1.0) for deg in range(-170, 171)]))  # 20 deg across the back
