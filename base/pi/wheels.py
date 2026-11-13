"""Wheel packets for the base firmware, odometry parsing and a watchdog that stops the base."""
import math
import os
import threading
import time

WHEEL_PORT = "/dev/rct_ahr_robot"
BAUD = 115200
TIMEOUT = 0.3  # seconds of silence before the watchdog sends S#
PERIOD = 0.04
# One odometry step longer than this cannot be real (0.12 m/s top speed, reads every 0.1 s): a garbled
# line that still parses, or a firmware reset (raw back to 0). It is refused like a bad line (review 2026-11-14).
STEP_MAX = 0.5  # m
TURN_MAX = 1.0  # rad
STOP_REPEATS = 3  # S# writes per stop: one byte lost to a parity error must not leave the base driving
SAFE_RUN_ENV = "HMC_SAFE_RUN"  # set by tools/safe_run.py for the program it starts
NOT_SAFE_RUN = ("start it through tools/safe_run.py, which sends S# however it ends (the firmware has no"
                " command timeout): python tools/safe_run.py .venv/bin/python <program> <arguments>")


def under_safe_run():
    """True when tools/safe_run.py started this program."""
    return os.environ.get(SAFE_RUN_ENV) == "1"


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


def parse_odom(line):
    """'Ox:+001.00,y:+000.50,th:+000.86#' to the firmware's own (x, y, th), unscaled."""
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
    return (x, y, th)


def true_step(pose, before, after, p):
    """Add one firmware odometry step to the true pose.

    The firmware sums x and y along its own, unscaled heading. Scaling its absolute heading
    (th / odom_yaw_scale) turned the pose away from the way x and y grow, further with every
    turn: the SLAM replay of 2026-11-13 found the logged pose about 170 degrees off its motion.
    So the step is read in the firmware's frame (along its heading halfway through the step),
    then scaled and added along the true heading.
    """
    (x0, y0, t0), (x1, y1, t1) = before, after
    mid = (t0 + t1) / 2
    dx, dy = x1 - x0, y1 - y0
    forward = (dx * math.cos(mid) + dy * math.sin(mid)) / p["odom_distance_scale"]
    side = (-dx * math.sin(mid) + dy * math.cos(mid)) / p["odom_distance_scale"]
    x, y, th = pose
    turn = (t1 - t0) / p["odom_yaw_scale"]
    h = th + turn / 2
    return (x + forward * math.cos(h) - side * math.sin(h), y + forward * math.sin(h) + side * math.cos(h), th + turn)


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
        self._raw = None  # the firmware's last (x, y, th)
        self._pose = None  # the true pose built from it, step by step (true_step)
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
                raw = parse_odom(line)
            except ValueError:
                continue
            if self._raw is not None and (math.hypot(raw[0] - self._raw[0], raw[1] - self._raw[1]) > STEP_MAX
                                          or abs(raw[2] - self._raw[2]) > TURN_MAX):
                line = "jump from %s to %s" % (self._raw, raw)
                continue
            if self._pose is None:  # the first reading: start where the old scaling put it
                d = self.p["odom_distance_scale"]
                self._pose = (raw[0] / d, raw[1] / d, raw[2] / self.p["odom_yaw_scale"])
            else:
                self._pose = true_step(self._pose, self._raw, raw, self.p)
            self._raw = raw
            return self._pose
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
