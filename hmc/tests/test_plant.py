"""The measured base as a plant model: command delay, dead zones, speed cap and decel."""

import unittest

import pytest

from hmc.run import run_scenario
from hmc.world import Robot


def plant(**options):
    """A plant in front of an ordinary robot, so the shadow's limits are the usual ones."""
    from hmc.plant import Plant

    return Plant(Robot(max_speed=1.0, max_turn_rate=2.0), **options)


def test_a_command_reaches_the_wheels_after_the_delay():
    base = plant(delay=0.40)
    for _ in range(4):  # 0.0, 0.1, 0.2 and 0.3 s
        base.command(0.10, 0.0, 0.1)
    assert base.x == 0.0  # nothing has arrived yet
    base.command(0.10, 0.0, 0.1)  # this cycle starts at 0.4 s: the command from 0.0 s arrives
    assert base.x == pytest.approx(0.01)  # 0.10 m/s for 0.1 s


def test_a_turn_below_the_dead_zone_does_not_move():
    base = plant(delay=0.0, turn_deadzone=0.10)
    base.command(0.0, 0.05, 0.1)  # measured: 0.05 rad/s on the spot does nothing
    assert (base.x, base.y, base.heading) == (0.0, 0.0, 0.0)


def test_a_turn_at_the_dead_zone_turns():
    base = plant(delay=0.0, turn_deadzone=0.10)
    base.command(0.0, 0.10, 0.1)  # measured: 0.10 rad/s turns
    assert base.heading == pytest.approx(0.01)  # 0.10 rad/s for 0.1 s


def test_a_forward_command_below_the_dead_zone_does_not_move():
    base = plant(delay=0.0, drive_deadzone=0.01)
    base.command(0.005, 0.0, 0.1)
    assert base.x == 0.0


def test_a_forward_command_from_the_dead_zone_moves():
    base = plant(delay=0.0, drive_deadzone=0.01)
    base.command(0.01, 0.0, 0.1)  # measured: forward motion starts at 0.01 m/s
    assert base.x == pytest.approx(0.001)  # 0.01 m/s for 0.1 s


def test_the_speed_is_capped_at_the_measured_top_speed():
    base = plant(delay=0.0, max_speed=0.12)
    base.command(1.0, 0.0, 0.1)
    assert base.speed == 0.12
    assert base.x == pytest.approx(0.012)  # 0.12 m/s for 0.1 s


def test_it_slows_down_at_the_measured_deceleration():
    base = plant(delay=0.0, max_speed=0.12, decel=0.14)
    base.command(1.0, 0.0, 0.1)  # at the cap: 0.12 m/s
    base.command(0.0, 0.0, 0.1)
    assert base.speed == pytest.approx(0.106)  # 0.12 - 0.14 * 0.1
    for _ in range(7):
        base.command(0.0, 0.0, 0.1)
    assert base.speed == pytest.approx(0.008)  # 0.12 - 0.14 * 0.8
    base.command(0.0, 0.0, 0.1)
    assert base.speed == 0.0  # it does not roll backwards past zero


class RealBaseGuard(unittest.TestCase):
    """The number lane D must make zero. It is not a pass/fail of the plant itself yet."""

    @unittest.expectedFailure  # lane D removes this marker: its fix makes the count 0
    def test_the_real_base_plant_never_asks_for_a_slow_turn_on_the_spot(self):
        """The guard for lane D. Now the brain asks for small turns the base cannot do."""
        result = run_scenario(base="real", plant="real-base")
        slow = [(v, w) for v, w in result.world.robot.requests
                if v == 0.0 and 0.0 < abs(w) < 0.10]
        print("cycles asking for 0 < |w| < 0.10 rad/s on the spot: %d" % len(slow))
        self.assertEqual(slow, [], "%d cycles asked to turn on the spot below 0.10 rad/s" % len(slow))
