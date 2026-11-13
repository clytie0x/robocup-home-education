"""Online estimates of how the base really moves, measured while it drives (Task M2).

The measured motion (motion.MotionState) is compared with the commands sent. Each estimate keeps
an exponential average with a sample count; below MIN_SAMPLES it is not trusted and the
params.json value is reported instead. Every value stays inside its bounds. The braking used for
safety, safe_braking(), is never better than params.json's: the estimates may only make the robot
more careful.
"""
import math

GAIN_BOUNDS = (0.5, 1.5)
LATENCY_BOUNDS = (0.0, 1.0)  # s
DECEL_BOUNDS = (0.02, 1.0)  # m/s^2
DELAY_BOUNDS = (0.0, 1.0)  # s
DRIFT_BOUNDS = (-0.2, 0.2)  # rad per metre
ALPHA = 0.05  # weight of a new sample in the averages
MIN_SAMPLES = {"speed_gain": 30, "turn_gain": 30, "drift_per_m": 30, "latency": 3, "decel": 3, "stop_delay": 3}
STEADY = 0.3  # s a command must be unchanged (after the latency) before its speed counts
MIN_V, MIN_W = 0.03, 0.1  # commands this small say too little about the gain
WINDOW = 10.0  # s of history for the latency
LAG_STEP = 0.02  # s
STOP_FROM = 0.03  # m/s: a stop counts when the command falls to 0 from at least this
SAME = 0.01  # m/s or rad/s: commands closer than this are one command (the velocity loop moves it a little)
STRAIGHT = 0.02  # rad/s: a smaller turn command counts as driving straight
CLEAR_FIT = 0.7  # the best latency must fit at least this much better than the median lag


def _clamp(x, bounds):
    return min(max(x, bounds[0]), bounds[1])


class _Average:
    def __init__(self, default, bounds):
        self.default, self.bounds, self.value, self.samples = default, bounds, None, 0

    def add(self, x):
        x = _clamp(x, self.bounds)
        weight = max(ALPHA, 1.0 / (self.samples + 1))  # a plain mean until 1/ALPHA samples, then an average that forgets
        self.value = x if self.value is None else self.value + weight * (x - self.value)
        self.samples += 1


class BaseEstimates:
    def __init__(self, p):
        self.p = p
        self._avg = {
            "speed_gain": _Average(1.0, GAIN_BOUNDS), "turn_gain": _Average(1.0, GAIN_BOUNDS),
            "drift_per_m": _Average(0.0, DRIFT_BOUNDS),
            "latency": _Average(p.get("command_latency", 0.0), LATENCY_BOUNDS),
            "decel": _Average(p["decel"], DECEL_BOUNDS), "stop_delay": _Average(p["stop_delay"], DELAY_BOUNDS),
        }
        self.sent = []  # (time, v, w) of every command, the last WINDOW + 1 s
        self.seen = []  # (time, v, w) measured, the last WINDOW seconds
        self._stop = None  # (time the command fell to 0, speed then) while a stop is being watched
        self._last_latency_at = -math.inf

    def _report(self, name):
        a = self._avg[name]
        trusted = a.samples >= MIN_SAMPLES[name] and a.value is not None
        return (a.value if trusted else a.default, a.samples, trusted)

    speed_gain = property(lambda self: self._report("speed_gain"))
    turn_gain = property(lambda self: self._report("turn_gain"))
    drift_per_m = property(lambda self: self._report("drift_per_m"))
    latency = property(lambda self: self._report("latency"))
    decel = property(lambda self: self._report("decel"))
    stop_delay = property(lambda self: self._report("stop_delay"))

    def safe_braking(self):
        """(decel, stop_delay) for safety: the worse of the estimate and params.json, each."""
        return min(self.decel[0], self.p["decel"]), max(self.stop_delay[0], self.p["stop_delay"])

    def _command_at(self, t):
        before = [s for s in self.sent if s[0] <= t + 1e-9]
        return before[-1] if before else None

    def observe(self, state, command, sent_at):
        """One cycle: the measured state (motion.MotionState), and the command (v, w) sent at sent_at."""
        changed = not self.sent or max(abs(command[0] - self.sent[-1][1]), abs(command[1] - self.sent[-1][2])) > SAME
        if changed:
            self.sent.append((sent_at, float(command[0]), float(command[1])))
        self.sent = [s for s in self.sent if s[0] > state.t - WINDOW - 1.0] or self.sent[-1:]
        self.seen = [s for s in self.seen if s[0] > state.t - WINDOW] + [(state.t, state.v, state.w)]
        self._stops(state, changed)
        if not state.trusted:
            return  # gains only from motion the LiDAR confirmed
        lag = self.latency[0]
        acting = self._command_at(state.t - lag)
        if acting is None or state.t - lag - acting[0] < STEADY:
            return
        _, v_cmd, w_cmd = acting
        if v_cmd >= MIN_V and abs(w_cmd) < STRAIGHT:
            self._avg["speed_gain"].add(state.v / v_cmd)
            if state.v > MIN_V:
                self._avg["drift_per_m"].add(state.w / state.v)
        if abs(w_cmd) >= MIN_W and v_cmd < MIN_V:
            self._avg["turn_gain"].add(state.w / w_cmd)
        if state.t - self._last_latency_at >= 1.0:
            self._last_latency_at = state.t  # at most once a second, found or not
            self._latency(state.t)

    def _latency(self, now):
        """The lag that best lines the turn commands up with the measured turn, over WINDOW."""
        seen = [s for s in self.seen if s[0] > now - WINDOW]
        if len(seen) < 30 or max(abs(s[2]) for s in seen) < MIN_W:
            return
        turns = [s for s in self.sent if s[0] > now - WINDOW]
        if sum(1 for a, b in zip(turns, turns[1:]) if abs(b[2] - a[2]) >= MIN_W) < 2:
            return  # the turn command must change in the window, or every lag fits the same
        best, errors = None, []
        lag = 0.0
        while lag <= LATENCY_BOUNDS[1] + 1e-9:
            pairs = [(c[2], s[2]) for s in seen if (c := self._command_at(s[0] - lag)) is not None]
            if len(pairs) >= 20:
                err = sum((m - c) ** 2 for c, m in pairs) / len(pairs)
                errors.append(err)
                if best is None or err < best[0] - 1e-12:
                    best = (err, lag)
            lag = round(lag + LAG_STEP, 6)
        sign_changes = sum(1 for a, b in zip(seen, seen[1:]) if a[2] * b[2] < 0)
        clear = errors and best is not None and best[0] <= CLEAR_FIT * sorted(errors)[len(errors) // 2]
        if clear and sign_changes >= 2:  # a lag is only seen when the turn changes, and one lag must fit clearly best
            self._avg["latency"].add(best[1])

    def _stops(self, state, changed):
        """Watch each stop: how long the base kept its speed, then how fast it slowed down."""
        cmd = self.sent[-1] if self.sent else None
        if self._stop is None:
            if changed and cmd is not None and cmd[1] == 0.0 and len(self.sent) >= 2 and self.sent[-2][1] >= STOP_FROM:
                before = [s for s in self.seen if s[0] < cmd[0]]
                if before:
                    self._stop = (cmd[0], before[-1][1], [])
            return
        t0, v0, trace = self._stop
        if cmd is None or cmd[1] != 0.0:
            self._stop = None  # driving again before it stood: no clean stop
            return
        trace.append((state.t, state.v))
        if state.v > 0.02 * v0:
            return
        # it stands: the delay is until it fell below 90 % of v0; the decel is the slope from 90 % to 10 %
        high = next((t for t, v in trace if v < 0.9 * v0), None)
        low = next((t for t, v in trace if v < 0.1 * v0), None)
        self._stop = None
        if high is None or low is None or low <= high:
            return
        decel = 0.8 * v0 / (low - high)
        delay = high - t0 - 0.1 * v0 / decel  # back from the 90 % point to where the slowing began
        self._avg["decel"].add(decel)
        self._avg["stop_delay"].add(max(0.0, delay))
