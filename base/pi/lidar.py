# Copied from xohm@pi:~/pi-drive/lidar.py on 2026-11-13.
"""One revolution from the LDS-03 on /dev/rct_ahr_lidar. Angles are base-frame degrees."""

import math
import time

import params

LIDAR_PORT = "/dev/rct_ahr_lidar"
LIDAR_X = params.load()["lidar_x"]  # laser ahead of the turning centre, metres
# Half the 0.40 m square body, 2 cm in from its outline: a return inside it is part of the robot.
# Seen 2026-11-13: laser 132 deg at 0.20 m, 3 of 726 logged scans, stopped a turn.
BODY_INSIDE = 0.18
# Measured with the camera depth columns and the lidar ranges, and by rotating the base: the
# raw laser angle runs clockwise, and the front of the base is at this laser angle.
FRONT_LASER_DEG = 178.7
START = bytes([0xAA, 0x55, 0xF0, 0x0F])
STOP = bytes([0xAA, 0x55, 0xF5, 0x0A])
# The widest angular gap one revolution may have. In 549 revolutions recorded on the robot
# (logs/base-check, 2026-11-13) the largest gap was 1.03 deg median, 1.47 at the 95th
# percentile; two revolutions had lost packets, the worst leaving a 252 deg hole.
GAP_DEG = 5.0


def _wrap(deg):
    while deg > 180:
        deg -= 360
    while deg <= -180:
        deg += 360
    return deg


def _word(buf, at):
    return buf[at] | (buf[at + 1] << 8)


def checksum_ok(packet):
    count = packet[3]
    if len(packet) != 10 + 3 * count:
        return False
    check = 0x55AA ^ (packet[2] | (count << 8)) ^ _word(packet, 4) ^ _word(packet, 6)
    for sample in range(count):
        at = 10 + 3 * sample
        check ^= packet[at] ^ _word(packet, at + 1)
    return check == _word(packet, 8)


def _extract(buf):
    """Samples as (laser degrees, metres) from checked packets, and the unparsed tail."""
    samples = []
    index = 0
    while index + 10 < len(buf):
        if buf[index] != 0xAA or buf[index + 1] != 0x55:
            index += 1
            continue
        count = buf[index + 3]
        need = 10 + 3 * count
        if not 1 <= count <= 40:
            index += 1
            continue
        if index + need > len(buf):
            break
        packet = bytes(buf[index : index + need])
        if not checksum_ok(packet):
            index += 1
            continue
        first = (_word(packet, 4) >> 1) / 64.0
        last = (_word(packet, 6) >> 1) / 64.0
        span = (last - first) % 360 if count > 1 else 0.0
        for sample in range(count):
            angle = (first + span * sample / max(count - 1, 1)) % 360
            raw = _word(packet, 10 + 3 * sample + 1)
            samples.append((angle, raw / 4000.0))
        index += need
    return samples, bytes(buf[index:])


def parse_samples(buf):
    """Samples as (laser degrees, metres) from checked packets: 10 header bytes, then 3 bytes each."""
    return _extract(buf)[0]


def parse_revolutions(buf):
    revolutions = []
    current = []
    previous = None
    for angle, meters in parse_samples(buf):
        if previous is not None and angle < previous - 180:
            revolutions.append(current)
            current = []
        previous = angle
        current.append((_wrap(FRONT_LASER_DEG - angle), meters))
    if current:
        revolutions.append(current)
    if not revolutions:
        return []
    return max(revolutions, key=len)


def complete(scan, gap_deg=GAP_DEG):
    """True when one revolution covers the whole circle with no gap wider than gap_deg.

    Where packets were lost the revolution simply has no samples, and a missing sample
    cannot be told from clear air: the obstacle in that direction would be invisible.
    """
    if len(scan) < 2:
        return False
    degrees = sorted(deg for deg, _ in scan)
    gaps = [b - a for a, b in zip(degrees, degrees[1:])]
    gaps.append(degrees[0] + 360.0 - degrees[-1])  # across the back of the circle
    return max(gaps) <= gap_deg


class ScanStream:
    """Parses only the new bytes and keeps the last complete revolution in the base frame."""

    def __init__(self, clock=time.monotonic, max_age=0.5):
        self.clock = clock
        self.max_age = max_age
        self.rest = b""
        self.current = []
        self.last = []
        self.updated_at = None
        self.previous = None
        self.parsed_bytes = 0

    def feed(self, data):
        carried = len(self.rest)
        buf = self.rest + data
        samples, self.rest = _extract(buf)
        self.parsed_bytes += len(buf) - carried
        for angle, meters in samples:
            if self.previous is not None and angle < self.previous - 180:
                if complete(self.current):  # an incomplete revolution is not a view of free space
                    self.last = self.current
                    self.updated_at = self.clock()
                self.current = []
            self.previous = angle
            self.current.append((_wrap(FRONT_LASER_DEG - angle), meters))

    def latest(self):
        if self.updated_at is None or self.clock() - self.updated_at > self.max_age:
            return []
        return self.last


class LidarSession:
    def __init__(self, port=LIDAR_PORT, handle=None):
        if handle is None:
            import serial

            handle = serial.Serial(port, 230400, timeout=0.05)
        self.handle = handle
        self.stream = ScanStream()
        self.handle.reset_input_buffer()
        self.handle.write(START)
        self.handle.flush()

    def latest(self):
        waiting = self.handle.in_waiting
        if waiting:
            self.stream.feed(self.handle.read(min(waiting, 65536)))
        return self.stream.latest()

    def close(self):
        try:
            self.handle.write(STOP)
            self.handle.flush()
        finally:
            self.handle.close()


def read_scan(port=LIDAR_PORT, seconds=1.2):
    import serial

    handle = serial.Serial(port, 230400, timeout=0.05)
    try:
        handle.reset_input_buffer()
        handle.write(START)
        handle.flush()
        buffer = bytearray()
        started = time.monotonic()
        while time.monotonic() - started < seconds and len(buffer) < 180000:
            buffer += handle.read(8192)
    finally:
        try:
            handle.write(STOP)
            handle.flush()
        finally:
            handle.close()
    return parse_revolutions(buffer)


def is_body(deg, meters):
    """The four aluminium posts seen in 11 scans: 0.18 m at about +-111 deg and 0.31 m at +-146 deg."""
    if meters <= 0.12:
        return True
    if meters <= 0.25 and 100 <= abs(deg) <= 120:
        return True
    if meters <= 0.40 and 138 <= abs(deg) <= 155:
        return True
    x = LIDAR_X + meters * math.cos(math.radians(deg))
    y = meters * math.sin(math.radians(deg))
    return abs(x) < BODY_INSIDE and abs(y) < BODY_INSIDE  # a cable or the camera stand, never an obstacle


def usable(scan):
    return [(deg, meters) for deg, meters in scan if meters > 0.05 and not is_body(deg, meters)]
