"""Malformed topic payloads and wheel-command limits. No ROS."""
import math

import pytest

from voice_assistant.input_safety import (MotionLimits, accepted_command, finite_seconds, llm_fields,
                                          plan_motion)


def test_json_that_is_not_an_object_is_not_a_command():
    for text in ("", "   ", "null", "[]", "42", '"stop"', "{", "not json"):
        assert accepted_command(text) is None
    assert accepted_command(None) is None


def test_stop_survives_garbage_params():
    assert accepted_command('{"action": "stop"}') == ("stop", {})
    assert accepted_command('{"action": " stop ", "params": [1]}') == ("stop", {})
    assert accepted_command('{"action": "stop", "params": {"arm": false}}') == ("stop", {"arm": False})


def test_a_motion_whose_params_are_not_an_object_is_refused():
    assert accepted_command('{"action": "move_forward", "params": "fast"}') == ("move_forward", None)
    assert accepted_command('{"action": "turn", "params": []}') == ("turn", None)


def test_a_normal_move_and_turn_keep_their_old_timing():
    linear, angular, duration, desc = plan_motion("move_forward", {"distance": 1.0})
    assert linear == pytest.approx(0.15)
    assert angular == 0.0
    assert duration == pytest.approx(1.0 / 0.15)
    assert desc == "move_forward 1.00 m"

    _, angular, duration, desc = plan_motion("turn", {"angle": -20})
    assert angular == pytest.approx(-0.6)
    assert duration == pytest.approx(abs(math.radians(-20)) / 0.6)
    assert desc == "turning -20 deg"

    # a missing angle used to mean 90 degrees; 0 must not
    _, angular, duration, _ = plan_motion("turn", {})
    assert angular == pytest.approx(0.6)
    assert duration == pytest.approx(abs(math.radians(90)) / 0.6)
    assert plan_motion("turn", {"angle": 0}) is None
    assert plan_motion("move_forward", {"distance": 0}) is None
    assert plan_motion("move_forward", {"speed": 0, "distance": 1}) is None


def test_distance_and_speed_stay_inside_the_limits():
    linear, _, duration, _ = plan_motion("move_backward", {"distance": 9, "speed": 5})
    assert linear == pytest.approx(-0.3)
    assert duration == pytest.approx(2.0 / 0.3)


def test_a_huge_turn_and_a_tiny_speed_cannot_run_for_minutes():
    _, angular, duration, desc = plan_motion("turn", {"angle": 3600})
    assert angular == pytest.approx(0.6)
    assert duration == pytest.approx(2 * math.pi / 0.6)   # 360 degrees, not 3600
    assert "360 deg" in desc and "3600" not in desc

    linear, _, duration, desc = plan_motion("move_forward", {"distance": 2, "speed": 0.001})
    assert linear == pytest.approx(0.001)
    assert duration == pytest.approx(20.0)
    assert "stopped at 20s" in desc


def test_a_present_value_that_is_not_a_number_does_not_move():
    for params in ({"speed": "fast"}, {"distance": "two"}, {"distance": float("nan")},
                   {"speed": float("inf")}, {"speed": True}, {"distance": [1]}):
        assert plan_motion("move_forward", params) is None
    assert plan_motion("turn", {"angle": "left"}) is None
    assert plan_motion("turn", {"angle": float("nan")}) is None
    # a numeric string is still a number
    linear, _, _, _ = plan_motion("move_forward", {"distance": "1.5", "speed": "0.2"})
    assert linear == pytest.approx(0.2)


def test_bad_limits_fall_back_instead_of_dividing_by_zero():
    limits = MotionLimits(default_speed=0, max_speed=float("nan"), turn_speed=-1,
                          max_distance="far", max_turn_deg=0, max_duration_s=float("inf"))
    assert limits.max_speed == pytest.approx(0.3)
    assert limits.default_speed == pytest.approx(0.15)
    assert limits.turn_speed == pytest.approx(0.6)
    assert limits.max_turn_deg == pytest.approx(360.0)
    assert limits.max_duration_s == pytest.approx(20.0)
    # a real lower cap is kept, and the default speed cannot sit above it
    slow = MotionLimits(default_speed=0.15, max_speed=0.12)
    assert slow.max_speed == pytest.approx(0.12)
    assert slow.default_speed == pytest.approx(0.12)


def test_timeouts_that_are_not_a_wait_use_the_fallback():
    assert finite_seconds(float("nan"), 30.0) == 30.0
    assert finite_seconds(float("inf"), 30.0) == 30.0
    assert finite_seconds(-5, 30.0) == 30.0
    assert finite_seconds("fast", 30.0) == 30.0
    assert finite_seconds(True, 30.0) == 30.0
    assert finite_seconds(0, 30.0) == 0.0
    assert finite_seconds(0, 15.0, allow_zero=False) == 15.0
    assert finite_seconds("12.5", 30.0) == 12.5


def test_an_llm_reply_that_is_not_an_object_is_refused():
    allowed = ["none", "stop", "move_forward"]
    for bad in (None, [], "stop", 5, "null"):
        assert llm_fields(bad, allowed) is None
    assert llm_fields({"reply": "  Stopping.  ", "action": " stop ", "params": None}, allowed) == (
        "Stopping.", "stop", {})
    assert llm_fields({"reply": 3, "action": "dance", "params": {}}, allowed) == ("", "none", {})
    assert llm_fields({"reply": "go", "action": "move_forward", "params": ["x"]}, allowed) == (
        "go", "none", {})
    reply, action, params = llm_fields(
        {"reply": "Moving.", "action": "move_forward", "params": {"distance": 1}}, allowed)
    assert (reply, action, params) == ("Moving.", "move_forward", {"distance": 1})
