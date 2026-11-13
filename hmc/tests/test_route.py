"""Breadcrumbs on the way out (Route) and driving them back (Returner)."""

import math

from hmc import hri
from hmc.follower import Follower, Mode, Senses
from hmc.navigation import Navigator
from hmc.route import ARRIVE, ARRIVE_BLOCKED, ASK_AFTER, FACE, REACHED, SKIP_AFTER, Returner, Route
from hmc.world import (
    DT, PERSON_RADIUS, ROBOT_RADIUS, Box, Circle, Robot, distance, wrap_angle,
)

START = (0.0, 0.0)


def line(route: Route, a, b, step=0.1) -> None:
    """Add positions every `step` metres from a to b, as a driving robot would."""
    n = round(distance(a, b) / step)
    for i in range(1, n + 1):
        route.add((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))


def straight(length=4.0) -> Route:
    route = Route(START)
    line(route, START, (length, 0.0))
    return route


def drive(returner, body, shapes, seconds=60.0, on_step=None):
    """Step the returner until it is done. Returns the time it took."""
    now = 0.0
    while not returner.done and now < seconds:
        if on_step:
            on_step(now)
        returner.step(shapes, now, DT)
        now += DT
    return now


def make(route, at, heading=math.pi, start_heading=0.0):
    body = Robot(x=at[0], y=at[1], heading=heading)
    speech = hri.Speech()
    return Returner(body, speech, Navigator(), route, start_heading), body, speech


def test_a_breadcrumb_every_half_metre():
    r = Route((0.0, 0.0))
    for i in range(1, 21):
        r.add((0.1 * i, 0.0))
    assert [round(x, 1) for x, _ in r.points] == [0.0, 0.5, 1.0, 1.5, 2.0]


def test_a_loop_is_cut_out():
    # Out 3 m, a circle of 2 m across back onto the line, then on to 5 m.
    r = Route(START)
    line(r, START, (3.0, 0.0))
    for i in range(1, 64):
        t = 2 * math.pi * i / 63
        r.add((3.0 + math.sin(t), 1.0 - math.cos(t)))
    line(r, (3.0, 0.0), (5.0, 0.0))
    back = r.back()
    # The circle reaches 2 m off the line; none of it is left. Only where it meets the
    # line again a breadcrumb may lie a little off it, like any breadcrumb on a curve.
    assert all(abs(y) < 0.2 for _, y in back)
    xs = [x for x, _ in back]
    assert xs == sorted(xs, reverse=True) and back[-1] == START


def test_standing_still_adds_nothing():
    r = Route((1.0, 2.0))
    assert not any(r.add((1.0 + 0.01 * (i % 2), 2.0)) for i in range(100))
    assert r.points == [(1.0, 2.0)]


def test_drives_the_breadcrumbs_in_reverse_and_stops_at_the_start():
    # An L: 4 m along x, then 4 m along y. A wall fills the inside of the L, across the
    # straight line from the end back to the start.
    route = Route(START)
    line(route, START, (4.0, 0.0))
    line(route, (4.0, 0.0), (4.0, 4.0))
    wall = Box(0.5, 0.6, 3.4, 4.5)
    returner, body, speech = make(route, (4.0, 4.0), heading=math.pi / 2)
    closest, corner = [math.inf], [math.inf]

    def record(now):
        closest[0] = min(closest[0], wall.clearance(body.pos))
        corner[0] = min(corner[0], distance(body.pos, (4.0, 0.0)))

    drive(returner, body, [wall], on_step=record)
    assert returner.done
    assert distance(body.pos, START) <= ARRIVE
    assert closest[0] > ROBOT_RADIUS  # never inside the wall
    # It went back the way it came: round the corner of the L (it steers a little ahead,
    # so it may cut the corner), not round the far side of the wall, 4 m away.
    assert corner[0] <= 2 * REACHED
    assert speech.has_said(hri.ARRIVED_TEXT)


def test_turns_to_the_start_heading_at_the_end():
    returner, body, _ = make(
        straight(3.0), (3.0, 0.0), heading=0.0, start_heading=math.pi / 2
    )
    drive(returner, body, [])
    assert returner.done
    assert abs(wrap_angle(body.heading - math.pi / 2)) <= FACE


def test_one_breadcrumb_means_already_there():
    # Review Focus 3: Enter before the robot has moved.
    returner, body, speech = make(Route(START), START, heading=0.0)
    returner.step([], 0.0, DT)
    assert returner.done
    assert speech.has_said(hri.ARRIVED_TEXT)


def test_return_from_a_search_starts_where_the_robot_stands():
    # Review Focus 3: the robot follows the operator out, loses them and searches. Enter
    # then starts the way back from where the robot stands, not from where it searched to.
    body = Robot()
    speech = hri.Speech()
    follower = Follower(body, speech)
    assert follower.route.points == [START]
    look = (1.0,) + (0.0,) * 15
    now, operator = 0.0, (2.0, 0.0)
    while operator[0] < 6.0 or follower.mode == Mode.LEARN:
        if follower.mode != Mode.LEARN:
            operator = (operator[0] + 0.5 * DT, 0.0)  # walks on along x
        senses = Senses(now, operator, look, {}, [])
        follower.watch(senses)
        follower.step(senses, DT)
        now += DT
    while follower.mode != Mode.SEARCH:  # the operator is gone
        senses = Senses(now, None, None, {}, [])
        follower.watch(senses)
        follower.step(senses, DT)
        now += DT
    assert len(follower.route.points) > 5  # breadcrumbs dropped while following
    here = body.pos
    returner = Returner(body, speech, follower.navigator, follower.route, 0.0)
    furthest = [here[0]]
    drive(
        returner,
        body,
        [],
        on_step=lambda t: furthest.__setitem__(0, max(furthest[0], body.x)),
    )
    assert returner.done and distance(body.pos, START) <= ARRIVE
    assert (
        furthest[0] <= here[0] + 0.3
    )  # it did not drive on to the operator's last spot


def test_skips_a_blocked_breadcrumb_after_five_seconds():
    # Something long lies along the route from 1.4 m to 3.1 m: the breadcrumb at 3 m and
    # the ones we steer at behind it cannot be reached within REACHED.
    returner, body, _ = make(straight(), (4.0, 0.0))
    chair = Box(1.4, -0.1, 3.1, 0.1)
    targets = []
    drive(
        returner, body, [chair], on_step=lambda t: targets.append((t, returner.target))
    )
    assert returner.done and distance(body.pos, START) <= ARRIVE
    aimed = [
        t
        for t, target in targets
        if target is not None and distance(target, (3.0, 0.0)) < 1e-9
    ]
    after = [
        t
        for t, target in targets
        if target is not None and distance(target, (2.5, 0.0)) < 1e-9
    ]
    assert aimed and after
    assert after[0] - aimed[0] >= SKIP_AFTER - 1e-6


def test_asks_to_pass_after_twenty_seconds_blocked_and_goes_on_when_free():
    returner, body, speech = make(straight(), (4.0, 0.0))
    shapes = [Box(2.9, -20.0, 3.1, 20.0)]  # a wall across the whole route
    said_at = []

    def watch(now):
        if not said_at and speech.has_said(hri.PASS_TEXT):
            said_at.append(now)
        if now >= ASK_AFTER + 5.0:
            shapes.clear()  # cleared

    drive(returner, body, shapes, on_step=watch)
    # It first reaches the breadcrumb in front of the wall (about 1 s), then waits there.
    assert said_at and ASK_AFTER <= said_at[0] <= ASK_AFTER + 2.0
    assert [text for _, text in speech.messages].count(hri.PASS_TEXT) == 1
    assert returner.done and distance(body.pos, START) <= ARRIVE


def test_goes_around_a_chair_put_on_the_route():
    returner, body, _ = make(straight(), (4.0, 0.0))
    chair = Box(1.8, -0.3, 2.2, 0.3)
    closest = [math.inf]
    drive(
        returner,
        body,
        [chair],
        on_step=lambda t: closest.__setitem__(
            0, min(closest[0], chair.clearance(body.pos))
        ),
    )
    assert returner.done and distance(body.pos, START) <= ARRIVE
    assert closest[0] > ROBOT_RADIUS


def test_steers_at_a_free_breadcrumb_not_at_one_a_person_stands_on():
    # A person stands on the breadcrumb at 2 m. Steering at it would make the planner aim
    # beside them, on a side that flips as they move; steer at the last free one before.
    returner, body, _ = make(straight(), (4.0, 0.0))
    person = Circle(2.0, 0.0, PERSON_RADIUS)
    returner.step([person], 0.0, DT)
    assert distance(returner.navigator.path[-1], (3.0, 0.0)) < 0.11


def test_someone_standing_at_the_start_still_lets_us_arrive_close_by():
    # The queue waits in its line next to the start: a person 0.3 m from it keeps us
    # more than ARRIVE away. After SKIP_AFTER seconds ARRIVE_BLOCKED will do.
    returner, body, speech = make(straight(2.0), (2.0, 0.0))
    person = Circle(-0.3, 0.0, PERSON_RADIUS)
    drive(returner, body, [person], seconds=30.0)
    assert returner.done and speech.has_said(hri.ARRIVED_TEXT)
    assert ARRIVE < distance(body.pos, START) <= ARRIVE_BLOCKED
