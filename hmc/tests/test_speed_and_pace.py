import math

from hmc import hri
from hmc.run import run_scenario
from hmc.task import PACE_SHARE, State
from hmc.world import DT, MAX_ACCEL, MAX_TURN_ACCEL, Robot, distance

# The acceleration limit


def test_robot_speeds_up_no_faster_than_the_limit():
    robot = Robot()
    speeds = [0.0]
    for _ in range(40):
        robot.drive_toward((100.0, 0.0), DT)
        speeds.append(robot.speed)
    for before, after in zip(speeds, speeds[1:]):
        assert after - before <= MAX_ACCEL * DT + 1e-9
    assert speeds[1] < 0.1  # no jump to top speed
    assert math.isclose(speeds[-1], robot.max_speed)  # it does get there


def test_robot_brakes_in_time_for_the_end_of_the_path():
    robot = Robot(speed=1.0)
    speeds = [robot.speed]
    for _ in range(100):
        robot.drive_toward((3.0, 0.0), DT, stop_in=3.0 - robot.x)
        speeds.append(robot.speed)
    for before, after in zip(speeds, speeds[1:]):
        assert before - after <= MAX_ACCEL * DT + 1e-9  # no sudden stop
    assert 2.95 < robot.x <= 3.0 + 1e-9  # stopped at the end, not past it


def test_robot_turns_with_the_turn_limit_and_does_not_swing_past():
    robot = Robot()
    turn_speeds = [0.0]
    for _ in range(50):
        robot.turn_toward((0.0, 5.0), DT)  # a quarter turn to the left
        turn_speeds.append(robot.turn_speed)
        assert robot.heading <= math.pi / 2 + 1e-9
    for before, after in zip(turn_speeds, turn_speeds[1:]):
        assert after - before <= MAX_TURN_ACCEL * DT + 1e-9  # no sudden spin-up
    assert math.isclose(robot.heading, math.pi / 2)


def test_slow_robot_still_finishes_in_time():
    result = run_scenario(robot_speed=0.5)
    assert result.task.state == State.DONE


# Asking the operator to walk slowly


def test_robot_asks_operator_to_walk_at_a_pace_it_can_follow():
    result = run_scenario(robot_speed=0.5)
    texts = [text for _, text in result.speech.messages]
    asked = [hri.asked_pace(text) for text in texts if hri.asked_pace(text)]
    assert asked == [PACE_SHARE * 0.5]
    assert asked[0] < result.world.robot.max_speed
    assert texts.index(hri.READY_TEXT) < texts.index(hri.walk_slowly_text(asked[0]))
    assert math.isclose(result.world.operator.speed, asked[0])


def test_operator_who_walks_slower_than_asked_keeps_their_own_pace():
    result = run_scenario(operator_speed=0.6)  # asked for 0.8 m/s
    assert math.isclose(result.world.operator.speed, 0.6)


def test_walk_slowly_sentence_round_trip():
    assert hri.asked_pace(hri.walk_slowly_text(0.12)) == 0.12
    assert hri.asked_pace(hri.READY_TEXT) is None


# Asking the operator to wait


def test_robot_asks_operator_to_wait_when_it_falls_behind():
    steps_while_waiting = []  # how far the operator moved in each step they were waiting
    last = {}

    def hold_robot(world, task):
        world.robot.max_speed = 0.0 if 3 < world.time < 12 else 1.0
        if world.operator.waiting and "pos" in last:
            steps_while_waiting.append(distance(world.operator.pos, last["pos"]))
        last["pos"] = world.operator.pos

    result = run_scenario(on_step=hold_robot)
    wait = [time for time, text in result.speech.messages if text == hri.WAIT_TEXT]
    go_on = [time for time, text in result.speech.messages if text == hri.GO_ON_TEXT]
    assert wait and 3 < wait[0] < 12
    assert go_on and go_on[0] > wait[0]
    assert steps_while_waiting and max(steps_while_waiting) == 0.0  # stood still
    assert result.task.state == State.DONE


def test_robot_never_asks_a_slow_operator_to_wait():
    result = run_scenario(operator_speed=0.4)  # much slower than the robot
    assert not result.speech.has_said(hri.WAIT_TEXT)
    assert result.task.state == State.DONE


def test_real_base_limits_are_applied_and_reset():
    import hmc.world as hmc_world
    from hmc.run import BASES, use_base
    from hmc.world import build_world

    world = build_world()
    use_base(world, "real")
    assert hmc_world.MAX_ACCEL == BASES["real"]["accel"]
    assert world.robot.max_turn_rate == BASES["real"]["turn_rate"]
    use_base(world, "sim")  # the next run must not inherit the real limits
    assert hmc_world.MAX_ACCEL == BASES["sim"]["accel"]
