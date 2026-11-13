"""The QueueFollower on its own: a body, people trackers and one Senses record per step."""

import math
import random

from hmc.appearance import random_look
from hmc.follower import Senses
from hmc.navigation import Navigator
from hmc.perception import MotionTracker
from hmc.queue import (
    CROWD_STANDING,
    LOST_TAIL_WAIT,
    QUEUE_GAP,
    QueueFollower,
)
from hmc.scoresheet import QUEUE_JOIN
from hmc.world import DT, PERSON_RADIUS, Circle, Robot, distance

START = (0.0, 0.0)
CROWD_A = random_look(random.Random(11))
CROWD_B = random_look(random.Random(12))
STRANGER = random_look(random.Random(13))
WALK = 0.5  # m/s, the queue walks toward the start along -x


def make(robot_at=(0.0, 0.0)) -> tuple[QueueFollower, Robot, dict]:
    body = Robot(x=robot_at[0], y=robot_at[1])
    trackers: dict[int, MotionTracker] = {}
    return QueueFollower(body, START, trackers), body, trackers


def senses(time, people, looks) -> Senses:
    return Senses(time, None, None, dict(people), [], looks=dict(looks))


def stand(trackers, pid, pos, until):
    """Two seconds of a person standing still at pos, ending at `until`."""
    tracker = trackers.setdefault(pid, MotionTracker())
    for i in range(21):
        tracker.update(until - 2.0 + i * DT, pos)


def walk(trackers, pid, x0, until):
    """Two seconds of a person walking toward the start along y = 0. Returns where they are."""
    tracker = trackers.setdefault(pid, MotionTracker())
    for i in range(21):
        t = until - 2.0 + i * DT
        tracker.update(t, (x0 - WALK * (t - (until - 2.0)), 0.0))
    return trackers[pid].last()


def remember(queue, trackers, looks: dict) -> None:
    """Two people of the crowd standing together for long enough, 2 m from the robot."""
    places = {1: (2.0, 0.0), 2: (2.0, 1.0)}
    now = CROWD_STANDING + 1.0
    for pid, pos in places.items():
        stand(trackers, pid, pos, now)
    standing = {pid: (0.0, pos) for pid, pos in places.items()}
    queue.remember_crowd(senses(now, places, looks), standing)


def test_people_standing_in_a_group_are_remembered_as_the_crowd():
    queue, _, trackers = make()
    remember(queue, trackers, {1: CROWD_A, 2: CROWD_B})
    assert len(queue.crowd) == 2


def test_a_single_standing_person_is_not_the_crowd():
    queue, _, trackers = make()
    now = CROWD_STANDING + 1.0
    stand(trackers, 1, (2.0, 0.0), now)
    queue.remember_crowd(
        senses(now, {1: (2.0, 0.0)}, {1: CROWD_A}), {1: (0.0, (2.0, 0.0))}
    )
    assert queue.crowd == []


def crowd_known(robot_at):
    queue, body, trackers = make()
    remember(queue, trackers, {1: CROWD_A, 2: CROWD_B})
    body.x, body.y = robot_at
    trackers.clear()  # the way back: new tracks
    return queue, body, trackers


def test_tail_is_the_nearest_walking_crowd_member_closer_to_the_start():
    queue, _, trackers = crowd_known((6.0, 0.0))
    now = 20.0
    people = {
        3: walk(trackers, 3, 5.0, now),  # nearest to us, ahead: the tail
        4: walk(trackers, 4, 3.0, now),  # further ahead in the queue
        5: walk(trackers, 5, 9.0, now),  # behind us: not ahead
    }
    stand(trackers, 6, (5.5, 0.5), now)  # standing, not walking with the queue
    people[6] = (5.5, 0.5)
    looks = {3: CROWD_A, 4: CROWD_B, 5: CROWD_A, 6: CROWD_B}
    assert queue.tail(senses(now, people, looks)) == 3


def test_a_passer_by_with_another_look_is_not_the_queue():
    queue, _, trackers = crowd_known((6.0, 0.0))
    now = 20.0
    people = {3: walk(trackers, 3, 5.0, now)}
    assert queue.tail(senses(now, people, {3: STRANGER})) is None


def test_no_looks_means_no_queue():
    queue, _, trackers = crowd_known((6.0, 0.0))
    now = 20.0
    people = {3: walk(trackers, 3, 5.0, now), 4: walk(trackers, 4, 3.0, now)}
    assert queue.tail(senses(now, people, {})) is None


def drive_behind(queue, body, trackers, now, x0, seconds, stop_at=None):
    """The tail (id 3) walks on toward the start, and stops at stop_at. Steps the queue.

    Returns (time, tail position, what step() returned last)."""
    navigator = Navigator()
    x = walk(trackers, 3, x0, now)[0]
    going = True
    for _ in range(round(seconds / DT)):
        now += DT
        if stop_at is None or x > stop_at:
            x -= WALK * DT
        trackers[3].update(now, (x, 0.0))
        seen = senses(now, {3: (x, 0.0)}, {3: CROWD_A})
        going = queue.step(seen, [Circle(x, 0.0, PERSON_RADIUS)], navigator, DT)
        if not going:
            break
    return now, (x, 0.0), going


def test_robot_stands_one_metre_behind_the_tail_in_their_walking_direction():
    queue, body, trackers = crowd_known((8.0, 0.0))
    body.heading = math.pi  # facing the start, like the queue
    _, tail, going = drive_behind(queue, body, trackers, 20.0, 7.0, 6.0)
    assert going and not queue.done
    # They walk along -x, so behind is +x: in the line, not next to it. The robot stops
    # within QUEUE_SPOT of its spot, and the smoothed track lags a walking person, so
    # the gap is a little more than QUEUE_GAP, but within what the referee counts as joined.
    assert abs(body.y) < 0.1
    assert QUEUE_GAP <= body.x - tail[0] <= QUEUE_JOIN


def test_leaves_when_the_followed_person_stops():
    # Review Focus 4: the queue arrives and stops while we are behind it.
    queue, body, trackers = crowd_known((7.5, 0.0))
    now, _, going = drive_behind(queue, body, trackers, 20.0, 6.0, 15.0, stop_at=5.0)
    assert not going and queue.done
    # About 2 s until the tail looks stationary, then LOST_TAIL_WAIT more.
    assert now - 20.0 < 2.0 + 2.0 + LOST_TAIL_WAIT + 1.0
    assert (
        queue.tail(senses(now, {3: (5.0, 0.0)}, {3: CROWD_A})) is None
    )  # never joined again


def test_leaves_two_metres_from_the_start():
    queue, body, trackers = crowd_known((1.8, 0.0))
    now = 20.0
    seen = senses(now, {3: walk(trackers, 3, 2.0, now)}, {3: CROWD_A})
    assert queue.step(seen, [], Navigator(), DT) is False
    assert queue.done


def test_keeps_going_three_seconds_after_losing_sight_of_the_tail():
    queue, body, trackers = crowd_known((6.5, 0.0))
    navigator = Navigator()
    now = 20.0
    seen = senses(now, {3: walk(trackers, 3, 5.0, now)}, {3: CROWD_A})
    assert queue.step(seen, [], navigator, DT)
    lost = now
    results = []
    while now < lost + LOST_TAIL_WAIT + 0.5:
        now += DT
        results.append((now - lost, queue.step(senses(now, {}, {}), [], navigator, DT)))
        if not results[-1][1]:
            break
    assert all(going for since, going in results if since <= LOST_TAIL_WAIT - 1e-9)
    assert results[-1][1] is False
    assert LOST_TAIL_WAIT - 1e-6 <= results[-1][0] <= LOST_TAIL_WAIT + DT + 1e-6
