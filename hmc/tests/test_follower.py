"""The Follower on its own: a body, a speech log and one Senses record per step."""

import math
import random

import pytest

from hmc import hri
from hmc.appearance import LookMemory, random_look
from hmc.follower import LOST_AFTER, MEMORY_KEEP, OPERATOR_LEARN, Follower, Mode, Senses
from hmc.navigation import FOLLOW_DISTANCE
from hmc.world import DT, Box, Robot, distance

LOOK = random_look(random.Random(1))  # the operator's look
STRANGER = random_look(random.Random(2))  # someone else's look
OPERATOR = (4.0, 0.0)  # 4 m in front of the robot


def make(**options) -> tuple[Follower, Robot, hri.Speech]:
    body = Robot()  # at (0, 0), facing +x
    speech = hri.Speech()
    return Follower(body, speech, **options), body, speech


def seen(time: float, operator=OPERATOR, look=LOOK, obstacles=None) -> Senses:
    """The operator (and their look) seen this step, nobody else around."""
    return Senses(time, operator, look, {}, obstacles or [])


def unseen(time: float) -> Senses:
    return Senses(time, None, None, {}, [])


def cycle(follower: Follower, senses: Senses) -> None:
    follower.watch(senses)
    follower.step(senses, DT)


def learn(follower: Follower, operator=OPERATOR) -> float:
    """Learn the operator's look while they stand still. Returns the time after it."""
    for i in range(OPERATOR_LEARN):
        follower.watch(seen(i * DT, operator))
    return OPERATOR_LEARN * DT


def lose(follower: Follower, start: float) -> float:
    """Steps without the operator until we start to search. Returns the time after it."""
    time = start
    while follower.mode != Mode.SEARCH:
        cycle(follower, unseen(time))
        time += DT
    return time


def test_follows_to_the_follow_distance_and_holds():
    follower, body, _ = make()
    for i in range(200):
        cycle(follower, seen(i * DT))
    assert distance(body.pos, OPERATOR) == pytest.approx(FOLLOW_DISTANCE, abs=0.1)


def test_a_look_that_does_not_match_is_not_the_operator():
    follower, _, _ = make()
    time = learn(follower)
    before = follower.last_operator_pos
    follower.watch(seen(time, (4.1, 0.0), STRANGER))  # close by, but a different look
    assert follower.last_operator_pos == before


def test_search_after_two_seconds_without_the_operator():
    follower, _, _ = make()
    time = learn(follower)
    cycle(follower, unseen(time + LOST_AFTER + DT))
    assert follower.mode == Mode.SEARCH
    cycle(follower, seen(time + LOST_AFTER + 2 * DT))
    assert follower.mode == Mode.FOLLOW


def test_a_stranger_does_not_end_the_search():
    follower, _, _ = make()
    time = lose(follower, learn(follower))
    for i in range(10):
        cycle(follower, seen(time + i * DT, look=STRANGER))
    assert follower.mode == Mode.SEARCH


def test_remembered_obstacle_still_blocks_after_it_leaves_the_view():
    follower, _, _ = make()
    box = Box(2.0, -0.5, 2.4, 0.5)
    follower.shapes(seen(0.0, obstacles=[box]))
    assert box in follower.shapes(seen(5.0))
    assert box not in follower.shapes(seen(MEMORY_KEEP + 1.0))


def test_look_memory_takes_a_512_number_look():
    rng = random.Random(3)
    look512 = tuple(rng.gauss(0.0, 1.0) for _ in range(512))
    memory = LookMemory()
    memory.add(look512)
    assert memory.similarity(look512) == pytest.approx(1.0)


def test_follow_distance_is_a_parameter():
    follower, body, _ = make(follow_distance=0.8)
    for i in range(200):
        cycle(follower, seen(i * DT))
    assert distance(body.pos, OPERATOR) == pytest.approx(0.8, abs=0.1)


def after_come_back(**options) -> tuple[float, Robot, float]:
    """Search until the robot asks the operator back, then wait 12 s.

    Returns how much the body turned from 11 s to 12 s after it asked, the body, and
    its turn speed just before it asked (it was turning on the spot then).
    """
    follower, body, speech = make(**options)
    time = lose(follower, learn(follower, operator=(1.0, 0.0)))
    turning = body.turn_speed
    while not speech.has_said(hri.COME_BACK_TEXT):
        turning = body.turn_speed
        cycle(follower, unseen(time))
        time += DT
    asked_at = speech.messages[-1][0]
    while time < asked_at + 11.0:
        cycle(follower, unseen(time))
        time += DT
    heading = body.heading
    while time < asked_at + 12.0:
        cycle(follower, unseen(time))
        time += DT
    return abs(math.remainder(body.heading - heading, 2 * math.pi)), body, turning


def test_search_turns_again_every_ten_seconds_when_asked_to():
    turned, _, _ = after_come_back(search_again_every=10.0)
    assert turned > 0.1
    turned, body, turning = after_come_back()  # the simulation: stand still
    assert turned == 0.0
    # Standing still is commanded: the wheels get 0, not the turn from before.
    assert turning != 0.0
    assert body.turn_speed == 0.0 and body.speed == 0.0


def test_robot_keeps_up_with_an_operator_who_walks_away(monkeypatch):
    # The real base: wider, slow, and it stops only 0.8 m from the operator (centre to
    # centre). There the path once ended on a random side of the operator's ring and
    # the robot turned on the spot instead of following.
    from hmc import world
    monkeypatch.setattr(world, "ROBOT_RADIUS", 0.29)
    monkeypatch.setattr(world, "MAX_ACCEL", 0.25)
    monkeypatch.setattr(world, "MAX_TURN_ACCEL", 0.7)
    body = Robot(max_speed=0.12, max_turn_rate=0.49)
    follower = Follower(body, hri.Speech(), follow_distance=0.8)
    dt = 0.1  # the robot's cycle
    for i in range(300):
        operator = (0.9 + 0.1 * i * dt, 0.0)  # walks away at 0.1 m/s
        senses = seen(i * dt, operator=operator)
        follower.watch(senses)
        follower.step(senses, dt)
    assert distance(body.pos, operator) < 0.8 + 0.15
