import inspect
import re

import pytest

import hmc.navigation
import hmc.task
from hmc.navigation import INFLATE, plan_path
from hmc.perception import see_obstacles
from hmc.run import main, run_scenario
from hmc.scoresheet import Referee, Scoresheet
from hmc.task import State
from hmc.world import (
    PERSON_RADIUS, ROBOT_RADIUS, TIME_LIMIT, Box, Circle, build_world, distance,
)  # fmt: skip

AVOID_ROWS = [
    "avoid_small_object",
    "avoid_hard_to_see",
    "avoid_crowd",
    "avoid_barriers",
]


# Planner


def test_planner_goes_around_a_wall():
    wall = Box(2.0, -2.0, 2.4, 2.0)
    path = plan_path((0.0, 0.0), (4.0, 0.0), [wall])
    assert path is not None
    assert path[-1] == pytest.approx((4.0, 0.0), abs=0.15)
    assert all(wall.clearance(p) >= INFLATE for p in path)


def test_planner_reports_no_path_when_boxed_in():
    walls = [Box(-1.2, -1.2, 1.2, -1.0), Box(-1.2, 1.0, 1.2, 1.2),
             Box(-1.2, -1.2, -1.0, 1.2), Box(1.0, -1.2, 1.2, 1.2)]  # fmt: skip
    assert plan_path((0.0, 0.0), (4.0, 0.0), walls) is None


def test_planner_stops_next_to_a_blocked_goal():
    person = Circle(3.0, 0.0, PERSON_RADIUS)  # we want to walk up to a person
    path = plan_path((0.0, 0.0), (3.0, 0.0), [person])
    assert path is not None
    assert person.clearance(path[-1]) >= INFLATE - 1e-9


def test_task_code_never_reads_obstacles_or_people_from_the_world():
    # The robot may only learn about them through perception.
    for module in (hmc.task, hmc.navigation):
        source = inspect.getsource(module)
        assert re.search(r"\.obstacles\b", source) is None
        assert re.search(r"\.people\b", source) is None


# Perception


def test_hard_to_see_object_is_seen_only_up_close():
    world = build_world()
    hidden = next(o for o in world.obstacles if o.kind == "hard_to_see")
    world.robot.x, world.robot.y = 3.0, 0.0  # 2.85 m from its edge
    assert hidden.shape not in see_obstacles(world)
    world.robot.x, world.robot.y = 5.0, 0.0  # about 1 m from its edge
    assert hidden.shape in see_obstacles(world)


# The scripted run


def test_every_avoid_row_is_awarded():
    result = run_scenario()
    for key in AVOID_ROWS:
        assert result.sheet.counts.get(key) == 1, key


def test_robot_never_touches_an_obstacle_or_a_person():
    closest = []

    def record(world, task):
        robot = world.robot.pos
        closest.extend(o.shape.clearance(robot) for o in world.obstacles)
        closest.extend(distance(robot, p.pos) - PERSON_RADIUS for p in world.people)

    run_scenario(on_step=record)
    assert min(closest) > ROBOT_RADIUS


def test_robot_joins_and_stays_in_the_queue():
    result = run_scenario()
    assert State.JOIN_QUEUE in [state for _, state in result.task.history]
    assert result.sheet.counts.get("queue") == 1


def test_without_a_crowd_there_is_no_queue_and_no_crowd_points():
    def no_crowd(world, task):
        world.people.clear()

    result = run_scenario(on_step=no_crowd)
    assert result.task.state == State.DONE
    assert result.sheet.counts.get("queue", 0) == 0
    assert result.sheet.counts.get("avoid_crowd", 0) == 0
    assert result.sheet.total() == 1115 - 300 - 50


# Referee


def test_touching_an_obstacle_loses_its_row():
    world = build_world()
    sheet = Scoresheet()
    referee = Referee(sheet)
    small = next(o for o in world.obstacles if o.kind == "small_object")
    world.robot.x, world.robot.y = small.shape.x, small.shape.y  # right on top of it
    referee.update(world, None)
    world.robot.x, world.robot.y = world.car  # now arrive at the car
    referee.update(world, None)
    assert sheet.counts.get("follow_to_car") == 1
    assert sheet.counts.get("avoid_small_object", 0) == 0


def test_an_obstacle_never_faced_earns_nothing():
    world = build_world()
    sheet = Scoresheet()
    referee = Referee(sheet)
    world.robot.x, world.robot.y = world.car  # teleport: nothing was passed on the way
    referee.update(world, None)
    for key in AVOID_ROWS:
        assert sheet.counts.get(key, 0) == 0


def test_falling_back_from_the_queue_loses_the_queue_row():
    def fall_back(world, task):
        # Stand still for 6 s while the queue walks on (it gets 3 m further away).
        world.robot.max_speed = 0.0 if 40 < world.time < 46 else 1.0

    result = run_scenario(on_step=fall_back)
    assert result.task.state == State.DONE
    assert result.sheet.counts.get("queue", 0) == 0


# The Eindhoven 2024 map


def test_eindhoven2024_map_earns_everything_without_touching_anything():
    closest = []

    def record(world, task):
        robot = world.robot.pos
        closest.extend(o.shape.clearance(robot) for o in world.obstacles)
        closest.extend(distance(robot, p.pos) - PERSON_RADIUS for p in world.people)

    result = run_scenario(map_name="eindhoven2024", on_step=record)
    assert result.task.state == State.DONE
    assert result.sheet.total() == 1115
    assert result.world.time < TIME_LIMIT
    assert min(closest) > ROBOT_RADIUS


def test_cli_can_pick_the_map(tmp_path, capsys):
    assert main(["--map", "eindhoven2024", "--out", str(tmp_path)]) == 0
    assert "Awarded 1115 of 1115" in capsys.readouterr().out
