"""Follow speed cap, and whether a brake measurement allows the next step (0.15 m/s).

The cap the follower uses is params.json "max_speed" (run_follow.State, body.limit, wheels.packet).
It is 0.12 m/s. 0.15 m/s is the next step (CLOSED_GATE_SPEED in run_follow.py) and stays off
until tools/brake_distance.py has measured the stop at 0.08, 0.12 and 0.15 m/s and the safety
gate still holds with that decel: an operator standing at follow_gap is outside the braking strip.

This module does not write params.json. After a supervised run whose JSON has "gate"."ok" true:

  1. Set "decel" to the JSON's gate.decel (the smallest decel implied by the three stops).
  2. Set "max_speed" to 0.15.

Leave both as they are when gate.ok is false. Do not raise max_speed first: the follower
would then plan stops with a decel the brake test has not confirmed.
"""
import safety

NEXT_SPEED = 0.15  # m/s, the only step this measurement may unlock
RUN_SPEEDS = (0.08, 0.12, 0.15)  # m/s, short straight runs, slowest first


def follow_max_speed(p):
    """The follow cap, m/s. It is params.json max_speed."""
    return float(p["max_speed"])


def decel_from_coast(speed, stop_m, stop_delay):
    """Deceleration (m/s^2) if `stop_m` is the distance from S# to a standstill.

    The coast is full speed for stop_delay, then constant decel: d = v*delay + v^2/(2a).
    None when the distance is not longer than the delay coast (nothing left to call decel).
    """
    if not (speed > 0 and stop_m >= 0 and stop_delay >= 0):
        return None
    pure = stop_m - speed * stop_delay
    if pure <= 1e-4:
        return None
    return speed * speed / (2.0 * pure)


def cautious_stop(run):
    """The longer of the odometry and LiDAR stopping distances, or None if both are missing."""
    vals = []
    for key in ("odom_stop_m", "lidar_stop_m"):
        value = run.get(key)
        if value is not None:
            vals.append(float(value))
    return max(vals) if vals else None


def measured_decel(runs, stop_delay):
    """The smallest decel implied by the runs. None if any run has no usable stop distance."""
    decels = []
    for run in runs:
        distance = cautious_stop(run)
        if distance is None:
            return None
        decel = decel_from_coast(float(run["speed"]), distance, stop_delay)
        if decel is None or not decel > 0:
            return None
        decels.append(decel)
    return min(decels) if decels else None


def gate_holds(speed, p):
    """True when an operator at follow_gap is outside the braking strip at `speed`."""
    operator = safety.FRONT + float(p["follow_gap"])
    return not safety.must_stop([(operator, 0.0)], float(speed), 0.0, p)


def recommendation(runs, p):
    """Whether the measured stops allow max_speed 0.15. Does not change `p`.

    ok is true only when every run speed in RUN_SPEEDS is present and the cautious decel
    still leaves the operator outside the strip at NEXT_SPEED. max_speed in the result is
    the value to write; it stays at p["max_speed"] when ok is false.
    """
    have = {}
    for run in runs:
        have[round(float(run["speed"]), 2)] = run
    missing = [v for v in RUN_SPEEDS if round(v, 2) not in have]
    current = follow_max_speed(p)
    if missing:
        return {"ok": False, "max_speed": current, "decel": None,
                "why": "need a stop at %s m/s" % ", ".join("%.2f" % v for v in missing)}
    ordered = [have[round(v, 2)] for v in RUN_SPEEDS]
    decel = measured_decel(ordered, float(p["stop_delay"]))
    if decel is None:
        return {"ok": False, "max_speed": current, "decel": None,
                "why": "a stop distance was missing or no longer than the command delay"}
    trial = dict(p, decel=decel, max_speed=NEXT_SPEED)
    if gate_holds(NEXT_SPEED, trial):
        return {"ok": True, "max_speed": NEXT_SPEED, "decel": decel,
                "why": "measured decel keeps the follow gap outside the braking strip at %.2f m/s" % NEXT_SPEED}
    return {"ok": False, "max_speed": current, "decel": decel,
            "why": "measured decel would stop behind an operator standing at the follow gap"}


SWITCH_HOW = (
    "If gate.ok is true, set params.json max_speed to gate.max_speed (0.15) and decel to "
    "gate.decel. If gate.ok is false, leave max_speed at 0.12. This tool does not edit params.json."
)
