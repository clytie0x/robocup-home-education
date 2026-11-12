# Copied from xohm@pi:~/pi-drive/lidar.py on 2026-11-13.
"""One revolution from the LDS-03 on /dev/rct_ahr_lidar. Angles are base-frame degrees."""

import time

LIDAR_PORT = "/dev/rct_ahr_lidar"
# Measured with the camera depth columns and the lidar ranges, and by rotating the base: the
# raw laser angle runs clockwise, and the front of the base is at this laser angle.
FRONT_LASER_DEG = 178.7
START = bytes([0xAA, 0x55, 0xF0, 0x0F])
STOP = bytes([0xAA, 0x55, 0xF5, 0x0A])


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
                self.last = self.current
                self.current = []
                self.updated_at = self.clock()
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
    return False


def usable(scan):
    return [(deg, meters) for deg, meters in scan if meters > 0.05 and not is_body(deg, meters)]
