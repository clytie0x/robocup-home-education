"""Minimal Feetech STS3215 protocol over pyserial (same packet format LeRobot uses).

Packet: FF FF ID LEN INSTR PARAMS... CHECKSUM, little-endian 16-bit values.
"""
import threading
import time

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE

# STS3215 control table (addresses used by this driver)
P_GAIN, D_GAIN, I_GAIN = 21, 22, 23
CW_DEAD_ZONE, CCW_DEAD_ZONE = 26, 27
TORQUE_ENABLE = 40
ACCELERATION = 41
GOAL_POSITION = 42          # 2 bytes; followed by goal time (44) and goal speed (46)
TORQUE_LIMIT = 48           # RAM, 0..1000 (x0.1 %)
LOCK = 55
PRESENT_POSITION = 56       # 2 bytes
PRESENT_LOAD = 60           # 2 bytes, bit10 = direction
PRESENT_VOLTAGE = 62        # 0.1 V
PRESENT_TEMPERATURE = 63    # deg C
SERVO_STATUS = 65           # error bits: 0 voltage, 1 sensor, 2 overheat, 3 current, 5 overload
PRESENT_CURRENT = 69        # 2 bytes


class FeetechError(IOError):
    pass


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(servo_id, instr, params=()):
    body = [servo_id, len(params) + 2, instr, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def le16(value):
    return [value & 0xFF, (value >> 8) & 0xFF]


def decode_signed(value, sign_bit=15):
    """STS encodes negative numbers as magnitude + sign bit."""
    if value & (1 << sign_bit):
        return -(value & ((1 << sign_bit) - 1))
    return value


class Bus:
    def __init__(self, port, baudrate=1_000_000, timeout=0.02, serial_factory=None):
        if serial_factory is None:
            import serial
            # exclusive: a second program (old app, calibrate, driver) cannot grab the bus at the same time
            self.ser = serial.Serial(port, baudrate, timeout=timeout, exclusive=True)
        else:
            self.ser = serial_factory(port, baudrate, timeout=timeout)
        self.lock = threading.Lock()

    def close(self):
        self.ser.close()

    # -- low level -------------------------------------------------------
    def _read_exact(self, n, deadline):
        buf = bytearray()
        while len(buf) < n:
            if time.monotonic() > deadline:
                raise FeetechError('timeout waiting for servo reply')
            buf += self.ser.read(n - len(buf))
        return bytes(buf)

    def _read_status(self, sent=b''):
        """Return (id, error, params) of one status packet; skips an echo of `sent`."""
        deadline = time.monotonic() + 0.05
        while True:
            prev = None
            while True:
                b = self._read_exact(1, deadline)[0]
                if prev == 0xFF and b == 0xFF:
                    break
                prev = b
            sid, length = self._read_exact(2, deadline)
            if length < 2:
                raise FeetechError('bad packet length')
            rest = self._read_exact(length, deadline)
            if checksum(bytes([sid, length]) + rest[:-1]) != rest[-1]:
                raise FeetechError('bad checksum')
            if sent and b'\xff\xff' + bytes([sid, length]) + rest == sent:
                continue  # half-duplex adapter echoed our own request
            return sid, rest[0], rest[1:-1]

    def _txrx(self, servo_id, instr, params=(), retries=2):
        with self.lock:
            for attempt in range(retries + 1):
                try:
                    self.ser.reset_input_buffer()
                    sent = packet(servo_id, instr, params)
                    self.ser.write(sent)
                    sid, err, data = self._read_status(sent)
                    if sid != servo_id:
                        raise FeetechError(f'reply from ID{sid}, expected ID{servo_id}')
                    return err, data
                except FeetechError:
                    if attempt == retries:
                        raise
        raise AssertionError

    # -- public API ------------------------------------------------------
    def ping(self, servo_id):
        try:
            self._txrx(servo_id, PING, retries=1)
            return True
        except FeetechError:
            return False

    def read(self, servo_id, address, length):
        _, data = self._txrx(servo_id, READ, [address, length])
        if len(data) != length:
            raise FeetechError(f'ID{servo_id}: short read')
        return data

    def read1(self, servo_id, address):
        return self.read(servo_id, address, 1)[0]

    def read2(self, servo_id, address):
        d = self.read(servo_id, address, 2)
        return d[0] | (d[1] << 8)

    def write(self, servo_id, address, data):
        err, _ = self._txrx(servo_id, WRITE, [address, *data])
        return err

    def write1(self, servo_id, address, value):
        return self.write(servo_id, address, [value & 0xFF])

    def write2(self, servo_id, address, value):
        return self.write(servo_id, address, le16(value))

    def sync_read(self, ids, address, length, retries=2):
        """Read `length` bytes at `address` from every id. Returns {id: bytes}."""
        params = [address, length, *ids]
        with self.lock:
            for attempt in range(retries + 1):
                try:
                    self.ser.reset_input_buffer()
                    sent = packet(BROADCAST, SYNC_READ, params)
                    self.ser.write(sent)
                    out = {}
                    for _ in ids:
                        sid, _err, data = self._read_status(sent)
                        if len(data) != length:
                            raise FeetechError(f'ID{sid}: short sync read')
                        out[sid] = data
                    if set(out) != set(ids):
                        raise FeetechError('sync read: missing servo')
                    return out
                except FeetechError:
                    if attempt == retries:
                        raise
        raise AssertionError

    def sync_read_positions(self, ids):
        raw = self.sync_read(ids, PRESENT_POSITION, 2)
        return {i: decode_signed(d[0] | (d[1] << 8)) for i, d in raw.items()}

    def sync_write(self, address, values, length):
        """values: {id: int} written as `length`-byte little endian. No reply."""
        params = [address, length]
        for sid, v in values.items():
            params += [sid] + [(v >> (8 * k)) & 0xFF for k in range(length)]
        with self.lock:
            self.ser.write(packet(BROADCAST, SYNC_WRITE, params))

    def sync_write_goals(self, goals):
        self.sync_write(GOAL_POSITION, {i: max(0, min(4095, int(round(g)))) for i, g in goals.items()}, 2)

    def sync_write_goals_speed(self, goals, speeds):
        """Goal position + goal speed in one packet (addr 42..47: pos, time=0, speed)."""
        values = {}
        for i, g in goals.items():
            pos = max(0, min(4095, int(round(g))))
            spd = max(0, min(3400, int(speeds.get(i, 0))))
            values[i] = pos | (spd << 32)
        self.sync_write(GOAL_POSITION, values, 6)
