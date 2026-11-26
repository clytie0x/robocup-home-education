"""Checks for text that arrives on the voice topics, and for wheel commands.

The ROS nodes import rclpy, so the checks live here and the tests call them directly.
A bad message is ignored. It must not raise, and a move or turn must not run the wheels
for longer than max_duration_s.
"""
import json
import math


def finite_seconds(value, fallback, allow_zero=True):
    """A finite number of seconds, or `fallback` when value is missing, negative, or not finite.

    0 is kept when allow_zero is set (a timeout of 0 means "already due"). It is replaced
    when allow_zero is false, because a speech limit of 0 would skip the announcement.
    """
    if isinstance(value, bool):
        return fallback
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(number) or number < 0 or (number == 0.0 and not allow_zero):
        return fallback
    return number


def object_from_json(data):
    """A JSON object, or None when the text is empty, not JSON, or not an object.

    `json.loads("null")` and `json.loads("[]")` succeed and then crash any caller that
    does `.get` on the result. Those are not commands.
    """
    if isinstance(data, dict):
        return data
    if not isinstance(data, (str, bytes, bytearray)):
        return None
    try:
        out = json.loads(data)
    except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
        return None
    return out if isinstance(out, dict) else None


def accepted_command(data):
    """(action, params) from a /robot/command payload.

    None means the message is not a JSON object.
    params is None when they are not an object and the action is not stop: the caller
    ignores it. Stop is kept even then, so a messy "stop" still stops the robot.
    """
    cmd = object_from_json(data)
    if cmd is None:
        return None
    action = cmd.get("action") or "none"
    if not isinstance(action, str) or not action.strip():
        action = "none"
    else:
        action = action.strip()
    raw = cmd.get("params", None)
    if raw is None or action == "stop":
        return action, {} if not isinstance(raw, dict) else raw
    if isinstance(raw, dict):
        return action, raw
    return action, None


def llm_fields(out, allowed):
    """(reply, action, params) from one model message, or None if it is not an object.

    The worker used to call `.get` on whatever `json.loads` returned. A reply of
    `null` or `[]` killed that thread, and the robot then ignored every later sentence.
    """
    if not isinstance(out, dict):
        return None
    reply = out.get("reply")
    reply = reply.strip() if isinstance(reply, str) else ""
    action = out.get("action", "none")
    if not isinstance(action, str):
        action = "none"
    else:
        action = action.strip()
        if action not in allowed:
            action = "none"
    raw = out.get("params", None)
    if action == "stop" or raw is None:
        params = {} if not isinstance(raw, dict) else raw
    elif isinstance(raw, dict):
        params = raw
    else:
        # don't drive on a command whose params are a list or a string
        action, params = "none", {}
    return reply, action, params


def _positive(value, fallback):
    """A finite number above 0, or fallback. 0 and below are not a usable speed or limit."""
    number = finite_seconds(value, fallback, allow_zero=False)
    return number


class MotionLimits:
    """Safety clamps for one open-loop move or turn. Defaults match config/params.yaml."""

    def __init__(self, default_speed=0.15, max_speed=0.3, turn_speed=0.6,
                 max_distance=2.0, max_turn_deg=360.0, max_duration_s=20.0):
        self.max_speed = _positive(max_speed, 0.3)
        self.default_speed = min(_positive(default_speed, 0.15), self.max_speed)
        self.turn_speed = _positive(turn_speed, 0.6)
        self.max_distance = _positive(max_distance, 2.0)
        self.max_turn_deg = _positive(max_turn_deg, 360.0)
        self.max_duration_s = _positive(max_duration_s, 20.0)


def _number(value):
    """('missing'|'bad'|'ok', float). 0 is ok. JSON true is bad: it is not 1 metre."""
    if value is None or value == "":
        return "missing", 0.0
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        return "bad", 0.0
    try:
        number = float(value.strip() if isinstance(value, str) else value)
    except ValueError:
        return "bad", 0.0
    if not math.isfinite(number):
        return "bad", 0.0
    return "ok", number


def plan_motion(action, params, limits=None):
    """(linear_x, angular_z, duration_s, description), or None when the robot should not move.

    Distance and speed are clamped. A turn is at most one full circle. Nothing runs longer
    than max_duration_s, so a tiny speed or "turn 3600 degrees" cannot drive for minutes.
    A value that is present but not a finite number is refused, so "speed: fast" does not move.
    """
    if not isinstance(params, dict):
        return None
    lim = limits or MotionLimits()
    if action in ("move_forward", "move_backward"):
        speed_kind, speed = _number(params.get("speed"))
        dist_kind, dist = _number(params.get("distance"))
        if speed_kind == "bad" or dist_kind == "bad":
            return None
        speed = lim.default_speed if speed_kind == "missing" else abs(speed)
        if speed == 0:
            return None
        if speed > lim.max_speed:
            speed = lim.max_speed
        dist = 0.5 if dist_kind == "missing" else abs(dist)
        if dist == 0:
            return None
        if dist > lim.max_distance:
            dist = lim.max_distance
        duration = dist / speed
        capped = duration > lim.max_duration_s
        if capped:
            duration = lim.max_duration_s
        linear = speed if action == "move_forward" else -speed
        desc = f"{action} {dist:.2f} m"
        if capped:
            desc += f", stopped at {lim.max_duration_s:.0f}s"
        return linear, 0.0, duration, desc
    if action == "turn":
        kind, angle = _number(params.get("angle"))
        if kind == "bad":
            return None
        if kind == "missing":
            angle = 90.0
        if angle == 0:
            return None
        if abs(angle) > lim.max_turn_deg:
            angle = math.copysign(lim.max_turn_deg, angle)
        radians = math.radians(angle)
        duration = abs(radians) / lim.turn_speed
        capped = duration > lim.max_duration_s
        if capped:
            duration = lim.max_duration_s
        angular = math.copysign(lim.turn_speed, angle)
        desc = f"turning {math.degrees(radians):.0f} deg"
        if capped:
            desc += f", stopped at {lim.max_duration_s:.0f}s"
        return 0.0, angular, duration, desc
    return None
