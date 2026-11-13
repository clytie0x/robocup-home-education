"""Streams every LiDAR revolution and the odometry pose to the Jetson, and collects the map correction.

Wire format both ways: 4-byte big-endian length, then a UTF-8 JSON body (like link.py).
Pi to Jetson {"t", "pose": [x, y, th], "scan": [[deg, m], ...]}: t is the Pi clock, pose is the
base in the odometry frame (metres, radians), scan is one revolution (0 deg = ahead, positive left).
Jetson to Pi {"t", "map_to_odom": [x, y, th]}: where the odometry frame sits in the map.
"""

import json
import math
import select
import socket
import struct
import threading
from collections import deque

MAX_BODY = 1_000_000  # bytes; a bigger message means a broken or hostile peer
MAX_JUMP_M = 0.5  # a correction that moves this far in one message is refused
MAX_JUMP_RAD = math.radians(20)
REBASE_AFTER = 10  # refused corrections in a row that agree with each other: 5 s at the bridge's 2 Hz
BACKLOG = 20  # records waiting to go out; a longer Wi-Fi stall drops the oldest, never blocks the caller
IDENTITY = (0.0, 0.0, 0.0)


def _wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _frame(obj):
    body = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    return struct.pack(">I", len(body)) + body


def parse_frames(buffer):
    """(bodies, rest) from the bytes received so far. Raises ValueError on an oversized length."""
    bodies = []
    while len(buffer) >= 4:
        size = struct.unpack(">I", buffer[:4])[0]
        if size > MAX_BODY:
            raise ValueError("message too big: %d bytes" % size)
        if len(buffer) < 4 + size:
            break
        bodies.append(buffer[4 : 4 + size])
        buffer = buffer[4 + size :]
    return bodies, buffer


class SlamLink:
    """send() only queues, correction() only reads, so neither ever blocks the drive loop."""

    def __init__(self, host, port=5602, connect_timeout=1.0, send_timeout=1.0, retry=1.0):
        self._host, self._port = host, port
        self._connect_timeout, self._send_timeout, self._retry = connect_timeout, send_timeout, retry
        self._lock = threading.Lock()
        self._queue = deque(maxlen=BACKLOG)
        self._correction = IDENTITY
        self._stop = threading.Event()
        self._thread = None
        self.refused = 0  # corrections thrown away for jumping too far
        self.rebased = 0  # times a jump stayed put long enough to be taken as the new truth
        self._candidate = None  # the refused correction that later ones are compared with
        self._agree = 0
        self.connected = False
        self.error = None  # text of the last problem, for logs

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, t, pose, scan):
        # Rounded: the saved scans show 3 mm and 0.01 degrees are far below the LiDAR's noise.
        record = {
            "t": t,
            "pose": [round(v, 5) for v in pose],
            "scan": [[round(deg, 2), round(m, 3)] for deg, m in scan],
        }
        with self._lock:
            self._queue.append(_frame(record))

    def correction(self):
        with self._lock:
            return self._correction

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(5)

    def _run(self):
        while not self._stop.is_set():
            try:
                with socket.create_connection((self._host, self._port), timeout=self._connect_timeout) as sock:
                    sock.settimeout(self._send_timeout)
                    self.connected = True
                    self._talk(sock)
            except Exception as error:  # the thread must live until stop()
                self.error = "%s: %s" % (type(error).__name__, error)
            self.connected = False
            self._stop.wait(self._retry)

    def _talk(self, sock):
        buffer = b""
        while not self._stop.is_set():
            readable, _, _ = select.select([sock], [], [], 0.05)
            if readable:
                data = sock.recv(65536)
                if not data:
                    raise ConnectionError("closed by the Jetson")
                bodies, buffer = parse_frames(buffer + data)
                for body in bodies:
                    self._accept(body)
            with self._lock:
                message = self._queue.popleft() if self._queue else None
            if message is not None:
                try:
                    sock.sendall(message)
                except OSError:
                    with self._lock:
                        self._queue.appendleft(message)  # sent whole or not at all after the reconnect
                    raise

    def _accept(self, body):
        try:
            x, y, th = (float(v) for v in json.loads(body.decode("utf-8"))["map_to_odom"])
            if not all(math.isfinite(v) for v in (x, y, th)):
                raise ValueError("not finite")
        except (ValueError, KeyError, TypeError) as error:  # one bad message must not end the link
            self.error = "bad correction: %s" % error
            return
        with self._lock:
            if not self._jumped((x, y, th), self._correction):
                self._candidate, self._agree = None, 0
                self._correction = (x, y, th)
                return
            self.refused += 1
            self.error = "correction refused: jumped to %.2f %.2f %.2f" % (x, y, th)
            # One bad message is a glitch. The same new value again and again is real: slam_toolbox
            # started a new map (the map origin is the first pose it sees, and the firmware odometry
            # is not zeroed between runs) or a loop closure moved it. Take that one.
            if self._candidate is not None and not self._jumped((x, y, th), self._candidate):
                self._agree += 1
            else:
                self._agree = 1
            self._candidate = (x, y, th)
            if self._agree >= REBASE_AFTER:
                self.rebased += 1
                self._candidate, self._agree = None, 0
                self._correction = (x, y, th)

    @staticmethod
    def _jumped(new, old):
        return math.hypot(new[0] - old[0], new[1] - old[1]) > MAX_JUMP_M or abs(_wrap(new[2] - old[2])) > MAX_JUMP_RAD
