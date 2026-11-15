"""Wheel packets for the base firmware, odometry and status parsing, a watchdog that stops the base, and
a check that the wheels really turn."""
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
REVERSE_MAX = 0.06  # m/s, the top backward speed; only tools/back_up.py and run_follow's escape (escape.py) send it, after safety.rear_clear
REVERSE_TURN_MAX = 0.05  # rad/s: send_reverse backs straight, the only path rear_clear checks
# E# (base firmware 0c020e4 on). It answers in a few ms; the old firmware has no E# and answers nothing,
# so the read waits only this long, not the port's 0.2 s (a cycle is 0.1 s).
STATUS_TIMEOUT = 0.05  # s
FAULT_LEFT, FAULT_RIGHT, NO_COMMAND = 0x01, 0x02, 0x04  # E#'s flags: an encoder fault cuts the motors
# Wheels that do not turn (2026-11-14: the motor supply died, the firmware cut the motors and run_follow
# commanded 0.114 m/s for 60 s). A command at least STALL_V or STALL_W for STALL_TIME moves the base well
# past STALL_MOVE or STALL_TURN (3 cm or 8.6 deg at the smallest watched command), start-up delay included.
STALL_V = 0.03  # m/s
STALL_W = 0.15  # rad/s
STALL_TIME = 1.0  # s
STALL_MOVE = 0.01  # m
STALL_TURN = math.radians(2.0)
# The smallest commands that move the wheels (base/docs/hardware-assessment-2026-11-13.md, "Lowest speeds that
# turn the wheels", 2026-11-15): forward they move from 0.01 m/s; on the spot 0.05 rad/s did nothing, 0.10 turned.
# In the four logs of 2026-11-15, 19 to 58 % of the turns on the spot were below 0.10: they never moved.
MIN_V = 0.01  # m/s
MIN_TURN = 0.10  # rad/s, on the spot
TURN_DEADBAND = math.radians(2.0)  # a heading error this small on the spot turns nothing, rather than crawl
NOT_SAFE_RUN = ("start it through tools/safe_run.py, which sends S# however it ends (the new firmware also stops by"
                " itself 0.3 s after the last command; the old one did not): python tools/safe_run.py .venv/bin/python"
                " <program> <arguments>")


def under_safe_run():
    """True when tools/safe_run.py started this program."""
    return os.environ.get(SAFE_RUN_ENV) == "1"


def _digits(value):
    # the epsilon keeps 0.27 * 100 from truncating to 26
    n = int(abs(value) * 100 + 1e-9)
    return f"{n // 100 % 10}{n // 10 % 10}{n % 10}"


def least(v, w, error=None):
    """(v, w) with a command too small to move the wheels made one that moves them, or 0 (run_follow, just before
    its stops: they judge what is sent).

    Forward, 0 < v < MIN_V becomes MIN_V; an arc (v at least MIN_V) keeps its w. On the spot (v 0, or backing),
    a heading error within TURN_DEADBAND (rad; None: the turn has no heading of its own) turns nothing, and any
    other turn below MIN_TURN becomes MIN_TURN its way. A NaN stays NaN: the stops stop for it.
    """
    if 0 < v < MIN_V:
        v = MIN_V
    if not v >= MIN_V and w:  # on the spot or backing (a NaN v too: it stays NaN, the stops stop for it)
        if error is not None and abs(error) < TURN_DEADBAND:
            return v, 0.0
        if abs(w) < MIN_TURN:
            w = math.copysign(MIN_TURN, w)
    return v, w


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


def reverse_packet(speed, w, p):
    """Backward at `speed` m/s (0 < speed <= REVERSE_MAX), turn and scaling as packet(): B<vvv>|<s><www>#."""
    if not (math.isfinite(speed) and math.isfinite(w)):
        raise ValueError(f"non-finite command {speed}, {w}")
    if not 0 < speed <= REVERSE_MAX:
        raise ValueError(f"backward speed {speed} outside (0, {REVERSE_MAX}]")
    data = packet(speed, w, p)
    return data if data == b"S#" else b"B" + data[1:]


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


def parse_status(line):
    """'Eleft:+0000120,right:-0000007,pwm:+076,+076,flags:03#' to {"left", "right", "pwm": [l, r], "flags"}.
    The flags are hex digits (FAULT_LEFT, FAULT_RIGHT, NO_COMMAND)."""
    line = line.strip()
    if not (line.startswith("E") and line.endswith("#")):
        raise ValueError(f"bad status {line!r}")
    parts = line[1:-1].split(",")
    if len(parts) != 5 or [s.split(":")[0] for s in parts] != ["left", "right", "pwm", parts[3], "flags"]:
        raise ValueError(f"bad status {line!r}")
    try:
        left, right, pwm_l = (int(s.split(":", 1)[1]) for s in parts[:3])
        return {"left": left, "right": right, "pwm": [pwm_l, int(parts[3])], "flags": int(parts[4].split(":", 1)[1], 16)}
    except ValueError:
        raise ValueError(f"bad status {line!r}") from None


class StallWatch:
    """The Pi's own check that the wheels turn, with any firmware: a command of at least STALL_V or STALL_W
    sent for STALL_TIME without a break, while the odometry moved less than STALL_MOVE and STALL_TURN."""

    def __init__(self):
        self.samples = []  # (time, pose) since the command became big enough; the first is the window start

    def reset(self):
        self.samples = []

    def update(self, now, sent, pose):
        """sent: the command the wheels had since the previous update. True: the wheels are not turning."""
        if abs(sent[0]) < STALL_V and abs(sent[1]) < STALL_W:
            self.samples = [(now, pose)]  # a big command sent now starts its window here
            return False
        if not self.samples:
            self.samples = [(now, pose)]  # nothing known from before
            return False
        self.samples.append((now, pose))
        while len(self.samples) > 1 and now - self.samples[1][0] >= STALL_TIME - 1e-6:
            self.samples.pop(0)  # the window: from the newest sample at least STALL_TIME old
        since, start = self.samples[0]
        if now - since < STALL_TIME - 1e-6:
            return False
        turned = abs(math.atan2(math.sin(pose[2] - start[2]), math.cos(pose[2] - start[2])))
        return math.dist(pose[:2], start[:2]) < STALL_MOVE and turned < STALL_TURN


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

    def send_reverse(self, speed, w):
        """The only backward command (reverse_packet); tools/back_up.py and run_follow's escape (escape.py) call it."""
        if not abs(w) <= REVERSE_TURN_MAX:  # NaN too
            raise ValueError(f"backing turn {w} outside +-{REVERSE_TURN_MAX}")
        data = reverse_packet(speed, w, self.p)
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

    def read_status(self):
        """E#: parse_status of the firmware's answer, ValueError when it does not parse (the old firmware
        has no E#). One try, at STATUS_TIMEOUT; the timeout is set and put back under the lock, as writes are."""
        timeout = self.port.timeout
        try:  # a write timeout must not leave the short read timeout for the rest of the run
            with self.lock:
                self.port.reset_input_buffer()
                self.port.timeout = STATUS_TIMEOUT
                self.port.write(b"E#")
            line = self.port.read_until(b"#").decode("ascii", "replace")
        finally:
            with self.lock:
                self.port.timeout = timeout
        return parse_status(line)

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
