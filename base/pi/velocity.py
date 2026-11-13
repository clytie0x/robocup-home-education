"""The velocity loop on the Pi (Task M3): send what makes the base drive what the brain wants.

The firmware drives open loop, so its speed is off by a gain that changes with load and battery.
The command is the wanted speed divided by the trusted gain (feed-forward), plus a PI term on
the error the LiDAR-checked motion shows against what was wanted one latency ago (what the base
can be doing by now), plus a turn that cancels the measured drift. It is
clamped to params.json's max_speed and max_turn_rate; while the base sits at its cap the integral
stops growing; a wanted 0 is sent as 0 at once.
"""
KP_V, KI_V = 0.5, 1.0  # per second
KP_W, KI_W = 0.3, 0.6
I_LIMIT = 0.05  # m/s (and rad/s) the integral may add at most
DT_MAX = 0.4  # s; a longer gap between calls resets nothing but counts as this much


class VelocityLoop:
    def __init__(self, p, estimates):
        self.p, self.estimates = p, estimates
        self.integral_v = self.integral_w = 0.0
        self.t = None
        self.wanted = []  # (time, v, w) wanted, the last 2 s

    def _wanted_at(self, t):
        before = [x for x in self.wanted if x[0] <= t + 1e-9]
        return before[-1][1:] if before else (0.0, 0.0)

    def command(self, v_wanted, w_wanted, state):
        """(v, w) to send for the wanted (v, w), given the measured motion.MotionState."""
        self.wanted = [x for x in self.wanted if x[0] > state.t - 2.0] + [(state.t, v_wanted, w_wanted)]
        if v_wanted == 0.0 and w_wanted == 0.0:
            self.integral_v = self.integral_w = 0.0
            self.t = state.t
            return (0.0, 0.0)
        dt = 0.0 if self.t is None else min(max(state.t - self.t, 0.0), DT_MAX)
        self.t = state.t
        gain_v = self.estimates.speed_gain[0] if self.estimates.speed_gain[2] else 1.0
        gain_w = self.estimates.turn_gain[0] if self.estimates.turn_gain[2] else 1.0
        drift = self.estimates.drift_per_m[0] if self.estimates.drift_per_m[2] else 0.0
        top, turn_top = self.p["max_speed"], self.p["max_turn_rate"]
        v = v_wanted / gain_v
        w = w_wanted / gain_w - drift * v_wanted  # the base turns drift rad per metre by itself
        if state.trusted:
            then_v, then_w = self._wanted_at(state.t - self.estimates.latency[0])
            err_v, err_w = then_v - state.v, then_w - state.w
            at_cap_v = v + KP_V * err_v + self.integral_v >= top and err_v > 0
            if not at_cap_v:  # no wind-up while the command is already at the cap
                self.integral_v = max(-I_LIMIT, min(I_LIMIT, self.integral_v + KI_V * err_v * dt))
            if err_v < 0 and self.integral_v > 0:
                self.integral_v = max(0.0, self.integral_v + KI_V * err_v * dt)  # come down at once when asked less
            self.integral_w = max(-I_LIMIT, min(I_LIMIT, self.integral_w + KI_W * err_w * dt))
            v += KP_V * err_v + self.integral_v
            w += KP_W * err_w + self.integral_w
        v = 0.0 if v_wanted == 0.0 else max(0.0, min(top, v))
        w = max(-turn_top, min(turn_top, w))
        return (v, w)
