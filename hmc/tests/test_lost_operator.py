"""Losing the operator while following, finding them again, and the -150 for it."""

from hmc import hri, perception
from hmc.run import run_scenario
from hmc.scoresheet import Referee, Scoresheet
from hmc.task import State
from hmc.world import build_world

HIDE_FROM = 8.0  # seconds; the operator is walking by then


def hide_operator(monkeypatch, until_back: bool):
    """Make the sensors miss the operator from HIDE_FROM on.

    With until_back the operator is seen again once they came back and stand in front
    of the robot, because it asked. Without it they stay hidden for good.
    """
    real_frame = perception._frame
    back = [False]

    def frame(world):
        seen = real_frame(world)
        if world.time < HIDE_FROM or (until_back and back[0]):
            return seen
        return dict(seen, operator=None, operator_look=None, ray=None)

    def watch(world, task):
        back[0] = world.operator.came_back > 0

    monkeypatch.setattr(perception, "_frame", frame)
    return watch


def test_robot_searches_asks_and_recovers(monkeypatch):
    on_step = hide_operator(monkeypatch, until_back=True)
    result = run_scenario(on_step=on_step)
    states = [state for _, state in result.task.history]
    assert State.SEARCH in states
    assert states[states.index(State.SEARCH) + 1] == State.FOLLOW  # found again
    assert result.speech.has_said(hri.COME_BACK_TEXT)
    assert result.world.operator.came_back == 1
    assert result.task.state == State.DONE
    assert result.sheet.counts.get("dem_come_back") == 1
    assert result.sheet.points("dem_come_back") == -150
    assert result.sheet.total() == 1115 - 150


def test_no_penalty_when_the_robot_never_scores_again(monkeypatch):
    on_step = hide_operator(monkeypatch, until_back=False)
    result = run_scenario(on_step=on_step)
    assert result.world.operator.came_back == 1  # they came back, but stay unseen
    assert result.task.state == State.TIMEOUT
    assert result.sheet.counts.get("dem_come_back", 0) == 0
    assert result.sheet.total() == 0


def test_come_back_penalty_waits_for_a_score_and_is_capped():
    world = build_world()
    sheet = Scoresheet()
    referee = Referee(sheet)
    world.operator.came_back = 2  # asked back twice: -300 before the cap
    referee.update(world, None)
    assert sheet.counts.get("dem_come_back", 0) == 0  # nothing scored yet
    world.robot.x, world.robot.y = world.car  # now the robot scores again
    referee.update(world, None)
    assert sheet.counts["dem_come_back"] == 2
    assert sheet.total() == 300 - 250  # follow_to_car, and the -250 cap
