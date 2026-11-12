import pytest

from hmc import hri
from hmc.manipulation import slip
from hmc.run import run_scenario
from hmc.task import ALLOWED, State, Task
from hmc.world import TIME_LIMIT, build_world


def test_full_run_visits_states_in_order():
    result = run_scenario()
    states = [state for _, state in result.task.history]
    assert states == [
        State.ANNOUNCE_READY,
        State.FOLLOW,
        State.PERCEIVE_BAG,
        State.GO_TO_BAG,
        State.PICK,
        State.RETURN,
        State.DONE,
    ]


def test_every_recorded_transition_is_allowed():
    result = run_scenario()
    states = [state for _, state in result.task.history]
    for before, after in zip(states, states[1:]):
        assert after in ALLOWED[before]


def test_illegal_transition_raises():
    task = Task(build_world(), hri.Speech())
    with pytest.raises(ValueError):
        task._go(State.PICK)  # cannot jump from ANNOUNCE_READY to PICK


def test_robot_announces_ready_first():
    result = run_scenario()
    assert result.speech.messages[0][1] == hri.READY_TEXT


def test_finished_states_do_not_change():
    result = run_scenario()
    result.task.step(0.1)
    assert result.task.state == State.DONE


def test_timeout_at_five_minutes():
    result = run_scenario(robot_speed=0.05)  # far too slow to finish
    assert result.task.state == State.TIMEOUT
    assert result.world.time == TIME_LIMIT
    assert result.sheet.counts.get("re_reach_start", 0) == 0


def test_drop_costs_50_and_robot_recovers():
    dropped = []

    def drop_once(world, task):
        if task.state == State.RETURN and not dropped:
            dropped.append(slip(world))

    result = run_scenario(on_step=drop_once)
    assert dropped == [True]
    assert result.sheet.counts["drop_bag"] == 1
    assert result.task.state == State.DONE  # it picked the bag up again
    # 300 + 15 + 100 + 200 - 50
    assert result.sheet.total() == 565
