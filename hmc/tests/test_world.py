"""The robot's motion model: braking like the real base, and the heading deadband."""

import math

import pytest

from hmc import world
from hmc.world import Robot, braking_speed


@pytest.fixture
def real_base():
    saved = world.MAX_DECEL, world.STOP_DELAY, world.HEADING_DEADBAND
    world.MAX_DECEL, world.STOP_DELAY, world.HEADING_DEADBAND = 0.064, 0.27, math.radians(4)
    yield
    world.MAX_DECEL, world.STOP_DELAY, world.HEADING_DEADBAND = saved


def test_braking_speed_with_the_real_base(real_base):
    for room in (0.05, 0.3, 1.0):
        v = braking_speed(room, 0.1)
        assert v * 0.27 + v * v / (2 * 0.064) <= room + 1e-9, room


def test_the_simulation_brakes_as_before():
    assert world.MAX_DECEL is None and world.STOP_DELAY == 0.0
    a = world.MAX_ACCEL
    assert braking_speed(1.0, 0.1) == max(math.sqrt(2 * a * 1.0) - a * 0.1, a * 0.1)


def test_brake_slows_at_the_real_decel(real_base):
    robot = Robot(speed=0.12)
    robot.brake(0.1)
    assert robot.speed == pytest.approx(0.12 - 0.0064)


def test_no_turn_inside_the_deadband(real_base):
    for degrees, turns in ((3, False), (6, True)):
        robot = Robot()
        target = (math.cos(math.radians(degrees)), math.sin(math.radians(degrees)))
        robot.turn_toward(target, 0.1)
        assert (robot.heading != 0.0) is turns, degrees
        assert (robot.turn_speed != 0.0) is turns, degrees


def test_a_moving_robot_still_corrects_a_small_error(real_base):
    robot = Robot(speed=0.1)
    robot.drive_toward((5.0, 5.0 * math.tan(math.radians(3))), 0.1)
    assert robot.heading > 0


def test_driving_corrects_even_a_small_error(real_base):
    robot = Robot(speed=0.006)
    robot.drive_toward((0.5 * math.cos(math.radians(2)), 0.5 * math.sin(math.radians(2))), 0.1, stop_in=0.001)
    assert robot.heading > 0.0
