"""Episode outcome for one grasp try.

Tier 1 uses the outcome, not the tier-2 shaping terms. Success is the PLAN.md
rule: both jaws on the handle or the rim, bag bottom at least 2 cm off the floor,
held 3 s, slip under 1 cm, and no collision.
"""

from . import constants as C


def judge(grasped, bottom, held_s, slip, collision):
    """Return success, dropped. `grasped` means both finger pads held the rim or a handle."""
    success = (
        grasped
        and bottom >= C.LIFT_CLEAR
        and held_s >= C.HOLD_S - 1e-9
        and slip <= C.SLIP_M
        and not collision
    )
    dropped = bool(grasped and not success and bottom < C.LIFT_CLEAR)
    return success, dropped


def outcome_reward(success, collision, dropped, sim_time_s):
    """Score one try.

    Time is -0.1 per second (the tier-2 -0.01 per 10 Hz step). On a miss the
    clock is at least MINIMUM_TRY_S, because a rejected reach returns in one
    step and would otherwise score 0, which is better than a real miss.
    """
    elapsed = float(sim_time_s)
    if not success:
        elapsed = max(elapsed, C.MINIMUM_TRY_S)
    reward = -C.TIME_PENALTY_PER_S * elapsed
    if success:
        reward += C.SUCCESS_BONUS
    if collision:
        reward -= C.COLLISION_PENALTY
    if dropped:
        reward -= C.DROP_PENALTY
    return reward
