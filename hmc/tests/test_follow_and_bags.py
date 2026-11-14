import math

import pytest

from hmc.perception import MotionTracker, Ray, choose_pointed_bag
from hmc.run import run_scenario
from hmc.navigation import FOLLOW_DISTANCE
from hmc.task import State
from hmc.world import build_world, distance

BAGS = {"red": (13.5, 7.5), "blue": (13.5, 4.5), "green": (12.0, 8.5)}
OPERATOR_AT_CAR = (12.0, 6.0)


def test_robot_stays_behind_operator_while_following():
    gaps = []

    def record(world, task):
        if task.state == State.FOLLOW and world.time > 5:
            gaps.append(distance(world.robot.pos, world.operator.pos))

    result = run_scenario(on_step=record)
    assert gaps, "never followed"
    # Going around obstacles makes the gap grow for a while, but we never fall far behind.
    assert max(gaps) < 4.5
    assert min(gaps) > FOLLOW_DISTANCE - 0.5  # did not run into them
    assert result.sheet.counts["follow_to_car"] == 1


def test_robot_never_reads_the_car_position():
    # The rulebook says the car location is unknown, so the task must not use it.
    import inspect
    import re

    import hmc.task

    # \b so that ".carried_bag" does not count as ".car"
    assert re.search(r"\.car\b", inspect.getsource(hmc.task)) is None


def test_robot_waits_until_ready():
    world = build_world()
    assert world.operator.walking is False


@pytest.mark.parametrize("target", ["red", "blue", "green"])
def test_picks_the_bag_the_operator_points_at(target):
    result = run_scenario(target_bag=target)
    assert result.task.perceived_bag == target
    assert result.sheet.counts["perceive_bag"] == 1
    assert result.sheet.counts["pick_bag"] == 1
    assert any(f"{target} bag" in text for _, text in result.speech.messages)


@pytest.mark.parametrize("target", ["red", "blue", "green"])
def test_ray_selects_exact_bag(target):
    bx, by = BAGS[target]
    angle = math.atan2(by - OPERATOR_AT_CAR[1], bx - OPERATOR_AT_CAR[0])
    assert choose_pointed_bag(Ray(OPERATOR_AT_CAR, angle), BAGS) == target


def test_ray_pointing_at_nothing_selects_no_bag():
    ray = Ray(OPERATOR_AT_CAR, math.pi)  # points away from every bag
    assert choose_pointed_bag(ray, BAGS) is None


def test_ray_tolerates_small_error():
    bx, by = BAGS["blue"]
    angle = math.atan2(by - 6.0, bx - 12.0) + math.radians(8)
    assert choose_pointed_bag(Ray(OPERATOR_AT_CAR, angle), BAGS) == "blue"


def test_motion_tracker():
    tracker = MotionTracker(window=2.0)
    for i in range(30):
        tracker.update(i * 0.1, (i * 0.1, 0.0))  # moving
    assert not tracker.is_stationary()
    for i in range(30, 60):
        tracker.update(i * 0.1, (3.0, 0.0))  # standing still
    assert tracker.is_stationary()
