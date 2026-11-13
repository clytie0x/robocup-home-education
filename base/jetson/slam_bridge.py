"""Jetson side of the SLAM stream: takes the Pi's LiDAR revolutions and poses, feeds slam_toolbox.

Wire format both ways: 4-byte big-endian length, then a UTF-8 JSON body (see base/pi/slam_link.py).
Pi sends {"t", "pose": [x, y, th], "scan": [[deg, m], ...]}; the Jetson sends back
{"t", "map_to_odom": [x, y, th]} twice a second. Angles on the wire: scan in degrees, th in radians.

ROS2 side: /scan, /odom, the odom -> base_link transform and a static base_link -> laser one go
out, stamped with the Jetson's receive time (the Pi's clock is not synchronised with ours).
slam_toolbox answers with map -> odom, which is looked up and sent back. /map is saved as a PNG.

Run: source /opt/ros/jazzy/setup.bash && python3 slam_bridge.py   (see slam/README.md)
"""

import argparse
import json
import math
import socket
import struct
import threading
import time
import zlib
from bisect import bisect_left
from pathlib import Path

LASER_X = 0.175  # metres, the laser sits ahead of the turning centre (params.json lidar_x)
RANGE_MIN = 0.12  # metres; lidar.is_body treats anything closer as the robot itself
RANGE_MAX = 8.0  # metres; the same as max_laser_range in slam/params.yaml
NO_RETURN = 0.05  # metres; the LDS-03 reports a missing return as 0
BEAM_DEG = 1.0  # LaserScan wants a regular grid; the LiDAR gives about 400 uneven samples per turn
MAX_GAP_DEG = 1.5  # a beam with no sample this close is left empty (NaN), never filled with a guess
MAX_SAMPLES = 5000
MIN_SAMPLES = 20
MAX_BODY = 1_000_000
CORRECTION_HZ = 2.0
MAP_PNG_EVERY = 10.0  # seconds


def _wrap_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def regular_ranges(scan):
    """One range per beam from -180 deg up in BEAM_DEG steps: the nearest sample, or NaN in a gap."""
    samples = sorted((_wrap_deg(deg), m) for deg, m in scan if m > NO_RETURN)
    if len(samples) < MIN_SAMPLES:
        raise ValueError("only %d usable samples" % len(samples))
    # Copies one turn either side, so the nearest sample is found across the +-180 seam too.
    around = [(a + k, m) for k in (-360.0, 0.0, 360.0) for a, m in samples]
    angles = [a for a, _ in around]
    ranges = []
    for i in range(int(round(360.0 / BEAM_DEG))):
        deg = -180.0 + i * BEAM_DEG
        k = bisect_left(angles, deg)
        best = min(around[max(k - 1, 0) : k + 1], key=lambda s: abs(s[0] - deg))
        if abs(best[0] - deg) > MAX_GAP_DEG:
            ranges.append(float("nan"))
        else:
            ranges.append(best[1] if best[1] <= RANGE_MAX else float("inf"))
    return ranges


def to_messages(record):
    """(LaserScan fields, Odometry fields) from one wire record. ValueError on a malformed one."""
    try:
        x, y, th = (float(v) for v in record["pose"])
        scan = [(float(deg), float(m)) for deg, m in record["scan"]]
    except (KeyError, TypeError) as error:
        raise ValueError("malformed record: %r" % error)
    if not all(math.isfinite(v) for v in (x, y, th)) or len(scan) > MAX_SAMPLES:
        raise ValueError("pose not finite or scan too long")
    ranges = regular_ranges([(d, m) for d, m in scan if math.isfinite(d) and math.isfinite(m)])
    step = math.radians(BEAM_DEG)
    laser = {
        "frame_id": "laser",
        "angle_min": -math.pi,
        "angle_max": -math.pi + step * (len(ranges) - 1),
        "angle_increment": step,
        "range_min": RANGE_MIN,
        "range_max": RANGE_MAX,
        "ranges": ranges,
    }
    odom = {  # the yaw as a quaternion, since the robot only turns about the vertical axis
        "frame_id": "odom",
        "child_frame_id": "base_link",
        "x": x,
        "y": y,
        "qz": math.sin(th / 2),
        "qw": math.cos(th / 2),
    }
    return laser, odom


def yaw_of(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def _png_chunk(kind, data):
    body = kind + data
    return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))


def write_png(path, width, height, grey):
    """An 8-bit greyscale PNG from width*height bytes, top row first. Standard library only."""
    if len(grey) != width * height:
        raise ValueError("pixel count does not match the size")
    rows = b"".join(b"\x00" + bytes(grey[r * width : (r + 1) * width]) for r in range(height))  # filter 0
    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", header) + _png_chunk(b"IDAT", zlib.compress(rows)) + _png_chunk(b"IEND", b"")
    Path(path).write_bytes(png)


def map_pixels(data, width, height):
    """OccupancyGrid cells (-1 unknown, 0 free, 100 occupied; row 0 at the bottom) to image bytes, north up."""
    grey = bytearray()
    for row in range(height - 1, -1, -1):
        for v in data[row * width : (row + 1) * width]:
            grey.append(205 if v < 0 else 254 - min(v, 100) * 254 // 100)
    return bytes(grey)


class SlamServer:
    """One Pi at a time (a new connection replaces the old one). Hands each record to on_record."""

    def __init__(self, on_record, host="0.0.0.0", port=5602, idle=5.0):
        self._on_record = on_record
        self._idle = idle
        self._current = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._listener = socket.socket()
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(2)
        self._listener.settimeout(0.2)
        self.port = self._listener.getsockname()[1]
        self._threads = []
        self.bad = 0  # records that failed to convert

    def start(self):
        self._spawn(self._accept_loop)

    def _spawn(self, target, *args):
        thread = threading.Thread(target=target, args=args, daemon=True)
        thread.start()
        self._threads = [t for t in self._threads if t.is_alive()] + [thread]

    def stop(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(5)
        self._listener.close()

    def send_correction(self, t, x, y, th):
        body = json.dumps({"t": t, "map_to_odom": [x, y, th]}).encode("utf-8")
        with self._send_lock:
            conn = self._current
            if conn is None:
                return
            try:
                conn.sendall(struct.pack(">I", len(body)) + body)
            except OSError:
                pass  # the reader thread notices the dead connection

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._listener.accept()
            except socket.timeout:
                continue
            conn.settimeout(0.2)
            with self._send_lock:
                old, self._current = self._current, conn
            if old is not None:
                try:
                    old.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            self._spawn(self._serve, conn)

    def _serve(self, conn):
        buffer = b""
        last = time.monotonic()
        with conn:
            try:
                while not self._stop.is_set():
                    try:
                        data = conn.recv(65536)
                    except socket.timeout:
                        if time.monotonic() - last > self._idle:
                            return
                        continue
                    if not data:
                        return
                    last = time.monotonic()
                    buffer = self._handle(buffer + data)
            except (OSError, ValueError):
                return  # bad or closed connection: drop it, the Pi reconnects
            finally:
                with self._send_lock:
                    if self._current is conn:
                        self._current = None

    def _handle(self, buffer):
        while len(buffer) >= 4:
            size = struct.unpack(">I", buffer[:4])[0]
            if size > MAX_BODY:
                raise ValueError("message too big")
            if len(buffer) < 4 + size:
                break
            body, buffer = buffer[4 : 4 + size], buffer[4 + size :]
            try:
                self._on_record(json.loads(body.decode("utf-8")))
            except (ValueError, KeyError, TypeError):
                self.bad += 1
        return buffer


def make_bridge(port, log_dir, laser_x):
    """The rclpy node, built here so the rest of the file imports without ROS."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener

    class Bridge(Node):
        def __init__(self):
            super().__init__("slam_bridge")
            self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
            self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self.tf_pub = TransformBroadcaster(self)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
            self.map = None  # the newest /map not yet written to a PNG
            self.saved = None  # the cells of the last PNG written
            self.log_dir = Path(log_dir)
            self.create_timer(1.0 / CORRECTION_HZ, self.send_correction)
            self.create_timer(MAP_PNG_EVERY, self.save_map)
            self.static = StaticTransformBroadcaster(self)  # kept, or the transform vanishes with it
            self.static.sendTransform(self.transform("base_link", "laser", laser_x, 0.0, 0.0, 1.0, self.get_clock().now().to_msg()))
            self.records = 0
            self.server = SlamServer(self.on_record, port=port)
            self.server.start()

        def transform(self, parent, child, x, y, qz, qw, stamp):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            return t

        def on_record(self, record):
            laser, odom = to_messages(record)  # raises ValueError for a bad record, which the server counts
            stamp = self.get_clock().now().to_msg()  # receive time, the same for all three messages
            self.tf_pub.sendTransform(self.transform("odom", "base_link", odom["x"], odom["y"], odom["qz"], odom["qw"], stamp))
            odometry = Odometry()
            odometry.header.stamp, odometry.header.frame_id, odometry.child_frame_id = stamp, "odom", "base_link"
            odometry.pose.pose.position.x, odometry.pose.pose.position.y = odom["x"], odom["y"]
            odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w = odom["qz"], odom["qw"]
            self.odom_pub.publish(odometry)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, laser["frame_id"]
            scan.angle_min, scan.angle_max, scan.angle_increment = laser["angle_min"], laser["angle_max"], laser["angle_increment"]
            scan.range_min, scan.range_max = laser["range_min"], laser["range_max"]
            scan.ranges = laser["ranges"]
            self.scan_pub.publish(scan)
            self.records += 1

        def send_correction(self):
            try:
                found = self.buffer.lookup_transform("map", "odom", rclpy.time.Time())  # the latest one
            except Exception:  # no map yet, or slam_toolbox is not running
                return
            r, v = found.transform.rotation, found.transform.translation
            self.server.send_correction(time.time(), v.x, v.y, yaw_of(r.z, r.w))

        def on_map(self, message):
            self.map = message

        def save_map(self):
            message, self.map = self.map, None
            if message is None or list(message.data) == self.saved:  # slam_toolbox republishes an unchanged map
                return
            self.saved = list(message.data)
            self.log_dir.mkdir(parents=True, exist_ok=True)
            path = self.log_dir / time.strftime("map-%Y%m%d-%H%M%S.png")
            write_png(path, message.info.width, message.info.height, map_pixels(message.data, message.info.width, message.info.height))
            self.get_logger().info("wrote %s" % path)

    rclpy.init()
    return Bridge()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5602)
    parser.add_argument("--log-dir", default=str(Path.home() / "hmc-follow" / "logs"))
    parser.add_argument("--laser-x", type=float, default=LASER_X)
    args = parser.parse_args()
    import rclpy

    node = make_bridge(args.port, args.log_dir, args.laser_x)
    print("listening on port", node.server.port, flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
