import json
import math
import os
import socket
import struct
import sys
import tempfile
import time
import unittest
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import slam_bridge  # noqa: E402


def ring(count=400, meters=2.0):
    """One revolution as the Pi saves it: (degrees, metres), 0 = ahead, positive left."""
    return [[-180.0 + 360.0 * i / count, meters] for i in range(count)]


def framed(obj):
    body = json.dumps(obj).encode()
    return struct.pack(">I", len(body)) + body


def wait_for(condition, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


class ToMessagesTests(unittest.TestCase):
    def test_pose_becomes_odometry_with_a_yaw_quaternion(self):
        _, odom = slam_bridge.to_messages({"t": 1.0, "pose": [1.5, -2.0, math.pi / 2], "scan": ring()})
        self.assertEqual((odom["frame_id"], odom["child_frame_id"]), ("odom", "base_link"))
        self.assertEqual((odom["x"], odom["y"]), (1.5, -2.0))
        self.assertAlmostEqual(slam_bridge.yaw_of(odom["qz"], odom["qw"]), math.pi / 2)

    def test_scan_is_a_regular_grid_in_the_laser_frame(self):
        laser, _ = slam_bridge.to_messages({"t": 1.0, "pose": [0, 0, 0], "scan": ring()})
        self.assertEqual(laser["frame_id"], "laser")
        self.assertEqual(len(laser["ranges"]), 360)
        self.assertAlmostEqual(laser["angle_min"], -math.pi)
        self.assertAlmostEqual(laser["angle_max"], laser["angle_min"] + 359 * laser["angle_increment"])
        self.assertTrue(all(r == 2.0 for r in laser["ranges"]))

    def test_beam_directions_keep_ahead_zero_and_left_positive(self):
        scan = [[deg, 1.0 if abs(deg) < 0.5 else 3.0 if abs(deg - 90) < 0.5 else 2.0] for deg, _ in ring()]
        laser, _ = slam_bridge.to_messages({"t": 1.0, "pose": [0, 0, 0], "scan": scan})
        self.assertEqual(laser["ranges"][180], 1.0)  # beam 180 is 0 degrees, straight ahead
        self.assertEqual(laser["ranges"][270], 3.0)  # beam 270 is +90 degrees, to the left
        self.assertEqual(laser["ranges"][90], 2.0)  # and -90 is to the right

    def test_a_real_gap_stays_nan_and_is_not_filled(self):
        scan = [s for s in ring() if not 40 < s[0] < 60]  # a 20 degree hole
        ranges = slam_bridge.regular_ranges(scan)
        self.assertTrue(math.isnan(ranges[180 + 50]))
        self.assertEqual(ranges[180 + 20], 2.0)

    def test_nearest_sample_is_found_across_the_seam(self):
        scan = [s for s in ring() if abs(s[0]) < 170]
        scan.append([179.8, 1.2])
        ranges = slam_bridge.regular_ranges(scan)
        self.assertEqual(ranges[0], 1.2)  # the beam at -180 is 0.2 degrees from the sample at 179.8

    def test_no_return_samples_are_dropped_and_far_ones_become_inf(self):
        scan = ring()
        scan[10][1] = 0.0
        scan[300][1] = 20.0
        ranges = slam_bridge.regular_ranges(scan)
        self.assertEqual(ranges[int(round(scan[10][0] + 180))], 2.0)  # a neighbour covers the dropped sample
        self.assertTrue(math.isinf(ranges[int(round(scan[300][0] + 180))]))

    def test_malformed_records_raise_value_error(self):
        for record in (
            {},
            {"pose": [0, 0], "scan": ring()},
            {"pose": [0, 0, float("nan")], "scan": ring()},
            {"pose": [0, 0, 0], "scan": ring()[:5]},
            {"pose": [0, 0, 0], "scan": "abc"},
            {"pose": [0, 0, 0], "scan": [[1.0]] * 100},
        ):
            with self.assertRaises(ValueError, msg=repr(record)[:60]):
                slam_bridge.to_messages(record)


class PngTests(unittest.TestCase):
    def read_png(self, data):
        self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
        at, chunks = 8, {}
        while at < len(data):
            size = struct.unpack(">I", data[at : at + 4])[0]
            kind, body = data[at + 4 : at + 8], data[at + 8 : at + 8 + size]
            self.assertEqual(struct.unpack(">I", data[at + 8 + size : at + 12 + size])[0], zlib.crc32(kind + body))
            chunks[kind] = body
            at += 12 + size
        return chunks

    def test_png_round_trip(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "m.png")
            slam_bridge.write_png(path, 3, 2, bytes([0, 1, 2, 3, 4, 5]))
            chunks = self.read_png(__import__("pathlib").Path(path).read_bytes())
        self.assertEqual(struct.unpack(">IIBBBBB", chunks[b"IHDR"]), (3, 2, 8, 0, 0, 0, 0))
        rows = zlib.decompress(chunks[b"IDAT"])
        self.assertEqual(rows, b"\x00\x00\x01\x02" + b"\x00\x03\x04\x05")

    def test_map_pixels_put_north_up_and_colour_the_cells(self):
        # 2 wide, 2 tall; row 0 is the bottom of the map
        pixels = slam_bridge.map_pixels([0, 100, -1, 0], 2, 2)
        self.assertEqual(list(pixels), [205, 254, 254, 0])  # top row is map row 1: unknown, free; then free, occupied


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.records = []
        self.server = slam_bridge.SlamServer(self.on_record, "127.0.0.1", 0, idle=1.0)
        self.server.start()
        self.addCleanup(self.server.stop)

    def on_record(self, record):
        slam_bridge.to_messages(record)  # a bad record raises, as in the real node
        self.records.append(record)

    def connect(self):
        sock = socket.create_connection(("127.0.0.1", self.server.port), timeout=2.0)
        self.addCleanup(sock.close)
        return sock

    def test_records_arrive_in_order_even_when_sent_in_pieces(self):
        sock = self.connect()
        data = b"".join(framed({"t": i, "pose": [i, 0, 0], "scan": ring()}) for i in range(3))
        for i in range(0, len(data), 1000):
            sock.sendall(data[i : i + 1000])
        self.assertTrue(wait_for(lambda: len(self.records) == 3))
        self.assertEqual([r["t"] for r in self.records], [0, 1, 2])

    def test_a_bad_record_is_counted_and_the_connection_goes_on(self):
        sock = self.connect()
        sock.sendall(framed({"t": 1, "pose": [0, 0], "scan": []}))
        sock.sendall(framed({"t": 2, "pose": [0, 0, 0], "scan": ring()}))
        self.assertTrue(wait_for(lambda: len(self.records) == 1))
        self.assertEqual(self.server.bad, 1)

    def test_correction_goes_back_to_the_connected_pi(self):
        sock = self.connect()
        sock.sendall(framed({"t": 1, "pose": [0, 0, 0], "scan": ring()}))
        self.assertTrue(wait_for(lambda: self.records))
        self.server.send_correction(5.0, 0.1, 0.2, 0.3)
        size = struct.unpack(">I", sock.recv(4))[0]
        self.assertEqual(json.loads(sock.recv(size)), {"t": 5.0, "map_to_odom": [0.1, 0.2, 0.3]})

    def test_correction_with_no_pi_connected_is_a_no_op(self):
        self.server.send_correction(1.0, 0.0, 0.0, 0.0)

    def test_oversized_length_drops_the_connection(self):
        sock = self.connect()
        sock.sendall(struct.pack(">I", 4_000_000_000))
        self.assertEqual(sock.recv(10), b"")

    def test_a_new_connection_replaces_the_old_one(self):
        first = self.connect()
        first.sendall(framed({"t": 1, "pose": [0, 0, 0], "scan": ring()}))
        self.assertTrue(wait_for(lambda: self.records))
        second = self.connect()
        second.sendall(framed({"t": 2, "pose": [0, 0, 0], "scan": ring()}))
        self.assertTrue(wait_for(lambda: len(self.records) == 2))
        self.server.send_correction(1.0, 1.0, 0.0, 0.0)
        size = struct.unpack(">I", second.recv(4))[0]
        self.assertEqual(json.loads(second.recv(size))["map_to_odom"], [1.0, 0.0, 0.0])


@unittest.skipUnless(__import__("importlib").util.find_spec("rclpy"), "rclpy is not installed")
class RosTests(unittest.TestCase):
    def test_the_node_can_be_built(self):
        import rclpy

        with tempfile.TemporaryDirectory() as folder:
            node = slam_bridge.make_bridge(0, folder, 0.175)
            try:
                self.assertGreater(node.server.port, 0)
            finally:
                node.server.stop()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
