"""Hand-made success and reward cases. No simulator."""

from rl_grasp import constants as C
from rl_grasp.reward import judge, outcome_reward


def test_success_needs_a_held_lift_without_a_collision():
    success, dropped = judge(grasped=True, bottom=0.03, held_s=3.0, slip=0.0, collision=False)
    assert success and not dropped


def test_two_seconds_is_not_yet_a_hold():
    success, _dropped = judge(grasped=True, bottom=0.05, held_s=2.0, slip=0.0, collision=False)
    assert not success


def test_a_centimetre_of_slip_fails():
    success, _dropped = judge(grasped=True, bottom=0.05, held_s=3.0, slip=0.011, collision=False)
    assert not success


def test_a_collision_cancels_the_lift():
    success, _dropped = judge(grasped=True, bottom=0.05, held_s=3.0, slip=0.0, collision=True)
    assert not success


def test_closing_and_then_dropping_is_not_a_plain_miss():
    success, dropped = judge(grasped=True, bottom=0.0, held_s=0.2, slip=0.0, collision=False)
    assert not success and dropped


def test_a_miss_that_never_closed_is_not_a_drop():
    success, dropped = judge(grasped=False, bottom=0.0, held_s=0.0, slip=0.0, collision=False)
    assert not success and not dropped


def test_reward_uses_the_named_terms():
    # +10 success, -5 collision, -2 drop, -0.1 per second.
    assert outcome_reward(True, False, False, 10.0) == 10.0 - 1.0
    assert outcome_reward(True, False, False, 10.0) > outcome_reward(True, False, False, 12.0)


def test_quitting_early_does_not_beat_a_real_miss():
    # A rejected reach used to return time 0 and reward 0, which beat a full miss.
    instant = outcome_reward(False, False, False, 0.0)
    full = outcome_reward(False, False, False, C.MINIMUM_TRY_S)
    assert instant == full
    assert instant < outcome_reward(True, False, False, C.MINIMUM_TRY_S)
