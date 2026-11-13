"""Wheel packets for the base firmware, odometry parsing and a watchdog that stops the base."""
import math
import threading
import time

WHEEL_PORT = "/dev/rct_ahr_robot"
BAUD = 115200
TIMEOUT = 0.3  # seconds of silence before the watchdog sends S#
PERIOD = 0.04
STOP_REPEATS = 3  # S# writes per stop: one byte lost to a parity error must not leave the base driving


def _digits(value):
    # the epsilon keeps 0.27 * 100 from truncating to 26
    n = int(abs(value) * 100 + 1e-9)
    return f"{n // 100 % 10}{n // 10 % 10}{n % 10}"


def packet(v, w, p):
    """True m/s and rad/s in, firmware packet out. Never backward."""
    if not (math.isfinite(v) and math.isfinite(w)):
        raise ValueError(f"non-finite command {v}, {w}")
    if v < 0:
        raise ValueError(f"negative speed {v}")
    v = min(v, p["max_speed"]) * (p["linear_cmd_scale"] or 1.0)
    w = max(-p["max_turn_rate"], min(p["max_turn_rate"], w)) * p["angular_cmd_scale"]
    dv, dw = _digits(v), _digits(w)
    if dv == dw == "000":  # rounds to nothing, so stop instead of F000|+000#
        return b"S#"
    return f"F{dv}|{'+' if w >= 0 else '-'}{dw}#".encode("ascii")


def parse_odom(line, p):
    """'Ox:+001.00,y:+000.50,th:+000.86#' to true (x, y, yaw)."""
    line = line.strip()
    if not (line.startswith("O") and line.endswith("#")):
        raise ValueError(f"bad odom {line!r}")
    parts = line[1:-1].split(",")
    if len(parts) != 3 or [s.split(":")[0] for s in parts] != ["x", "y", "th"]:
        raise ValueError(f"bad odom {line!r}")
    try:
        x, y, th = (float(s.split(":", 1)[1]) for s in parts)
    except (IndexError, ValueError):
        raise ValueError(f"bad odom {line!r}") from None
    if not all(math.isfinite(n) for n in (x, y, th)):
        raise ValueError(f"bad odom {line!r}")
    d = p["odom_distance_scale"]
    return (x / d, y / d, th / p["odom_yaw_scale"])


class Wheels:
    """Repeats the latest packet and stops the base when send() goes quiet."""

    def __init__(self, p, port=None, clock=time.monotonic, start_thread=True):
        if port is None:
            import serial

            # write_timeout: a stuck USB write raises instead of holding the lock, so the watchdog keeps running
            port = serial.Serial(WHEEL_PORT, BAUD, timeout=0.2, write_timeout=0.1, parity=serial.PARITY_EVEN,
                                 dsrdtr=False, rtscts=False)
        self.p, self.port, self.clock = p, port, clock
        self.lock = threading.Lock()
        self.last = b"S#"
        self.sent_at = clock()
        self._write_stop()  # a base left driving by a crashed run stops as soon as we open the port
        self.done = threading.Event()
        self.thread = None
        if start_thread:
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def _run(self):
        while not self.done.wait(PERIOD):
            self.tick()

    def _write_stop(self):
        """S# several times. The caller holds the lock, except in __init__ where nobody else can."""
        self.last = b"S#"  # first, so a failed write still leaves us stopped
        for _ in range(STOP_REPEATS):
            self.port.write(b"S#")

    def tick(self):
        with self.lock:
            if self.last == b"S#":
                return
            try:
                if self.clock() - self.sent_at >= TIMEOUT:
                    self._write_stop()
                else:
                    self.port.write(self.last)
            except Exception:
                self.last = b"S#"

    def send(self, v, w):
        data = packet(v, w, self.p)
        with self.lock:
            self.last, self.sent_at = data, self.clock()
            self.port.write(data)
        return data

    def read_odom(self):
        line = ""
        for _ in range(4):
            with self.lock:
                self.port.reset_input_buffer()
                self.port.write(b"O#")
            line = self.port.read_until(b"#").decode("ascii", "replace")
            try:
                return parse_odom(line, self.p)
            except ValueError:
                pass
        raise ValueError(f"bad odom {line!r}")

    def stop(self):
        with self.lock:
            self._write_stop()

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(1)
        try:
            self.stop()
        finally:
            self.port.close()
