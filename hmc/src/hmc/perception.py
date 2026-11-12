"""What the robot can sense. The robot code uses only these functions."""

import math
from collections import deque
from typing import NamedTuple

from hmc.world import Point, Shape, World, distance, wrap_angle

SENSOR_RANGE = 10.0  # metres
MAX_POINTING_ERROR = math.radians(15)  # a bag must be this close to the ray


class Ray(NamedTuple):
    origin: Point
    angle: float  # radians


def can_see(world: World, p: Point) -> bool:
    """In sensor range, and no wall in between. Only walls block the view."""
    if distance(world.robot.pos, p) > SENSOR_RANGE:
        return False
    return not any(
        o.shape.blocks(world.robot.pos, p) for o in world.obstacles if o.kind == "wall"
    )


def _frame(world: World) -> dict:
    """What the sensors report in this time step: operator, people, bags, pointing.

    Without noise this is the truth. With noise (see noise.py) it is computed once per
    time step, so every function below sees the same, slightly wrong, picture.
    """
    noise = world.noise
    if noise is not None and noise.frame_time == world.time:
        return noise.frame
    robot = world.robot.pos
    op = world.operator
    operator = op.pos if can_see(world, op.pos) else None
    everyone = list(world.people) + list(world.bystanders)
    visible = {p.id: p for p in everyone if can_see(world, p.pos)}
    people = {pid: p.pos for pid, p in visible.items()}
    looks = {pid: getattr(p, "look", None) for pid, p in visible.items()}
    operator_look = getattr(op, "look", None) if operator is not None else None
    bags = {n: b.pos for n, b in world.bags.items() if can_see(world, b.pos)}
    ray = world.pointing_ray() if operator is not None else None
    if noise is None:
        return {
            "operator": operator, "operator_look": operator_look, "people": people,
            "looks": looks, "bags": bags, "ray": ray,
        }  # fmt: skip

    # Tracker id swaps between two people passing close to each other. They stay.
    for pid in people:
        noise.ids.setdefault(pid, pid)
    close = sorted(people)
    for i, a in enumerate(close):
        for b in close[i + 1 :]:
            near = distance(people[a], people[b]) <= noise.swap_radius
            if near and noise.rng.random() < noise.swap_rate:
                noise.ids[a], noise.ids[b] = noise.ids[b], noise.ids[a]

    def seen_look(look):
        if look is None:
            return None
        return tuple(v + noise.rng.gauss(0.0, noise.look_sigma) for v in look)

    seen_people, seen_looks = {}, {}
    for pid, pos in people.items():
        if not noise.missed(distance(robot, pos)):
            seen_people[noise.ids[pid]] = noise.jitter(pos, noise.position_sigma)
            # The look belongs to the real person, even when the tracker id was swapped.
            seen_looks[noise.ids[pid]] = seen_look(looks[pid])
    if operator is not None:
        others = [
            p
            for p in visible.values()
            if distance(p.pos, operator) <= noise.confuse_radius
        ]
        if others and noise.rng.random() < noise.confuse_rate:
            # Someone else is taken for the operator this frame. Their look comes too.
            operator, operator_look = others[0].pos, getattr(others[0], "look", None)
        if noise.missed(distance(robot, operator)):
            operator, operator_look = None, None
        else:
            operator = noise.jitter(operator, noise.position_sigma)
            operator_look = seen_look(operator_look)
    seen_bags = {
        n: noise.jitter(pos, noise.bag_sigma)
        for n, pos in bags.items()
        if not noise.missed(distance(robot, pos))
    }
    if ray is not None and operator is not None:
        ray = (
            ray[0],
            ray[1] + math.radians(noise.rng.gauss(0.0, noise.pointing_sigma_deg)),
        )
    else:
        ray = None
    noise.frame_time = world.time
    noise.frame = {
        "operator": operator, "operator_look": operator_look, "people": seen_people,
        "looks": seen_looks, "bags": seen_bags, "ray": ray,
    }  # fmt: skip
    return noise.frame


def see_operator(world: World) -> Point | None:
    """Operator position, or None when we cannot see them."""
    return _frame(world)["operator"]


def see_operator_look(world: World):
    """The look (re-ID vector) of whoever is reported as the operator, or None."""
    return _frame(world)["operator_look"]


def see_looks(world: World) -> dict:
    """The look (re-ID vector) of every person in see_people, by tracker id."""
    return dict(_frame(world)["looks"])


def see_bags(world: World) -> dict[str, Point]:
    """Positions of the bags we can see, by name."""
    return dict(_frame(world)["bags"])


def see_obstacles(world: World) -> list[Shape]:
    """Shapes of the obstacles we can see. We do not learn what kind of obstacle it is."""
    seen = []
    for obstacle in world.obstacles:
        reach = obstacle.seen_within or SENSOR_RANGE
        if obstacle.shape.clearance(world.robot.pos) <= reach:
            seen.append(obstacle.shape)
    return seen


def see_people(world: World) -> dict[int, Point]:
    """Positions of the other people (not the operator) we can see, by tracker id.

    The crowd, the queue and the passers-by all look the same to the robot.
    """
    return dict(_frame(world)["people"])


def see_pointing(world: World) -> Ray | None:
    """The pointing ray, only if we can see the operator."""
    ray = _frame(world)["ray"]
    return Ray(*ray) if ray else None


def choose_pointed_bag(ray: Ray, bags: dict[str, Point]) -> str | None:
    """Return the bag closest to the ray direction, or None if none is close enough."""
    best_name, best_error = None, MAX_POINTING_ERROR
    for name, pos in bags.items():
        to_bag = math.atan2(pos[1] - ray.origin[1], pos[0] - ray.origin[0])
        error = abs(wrap_angle(to_bag - ray.angle))
        if error <= best_error:
            best_name, best_error = name, error
    return best_name


class MotionTracker:
    """Remembers recent positions of one person (or bag) and smooths out the noise.

    Every answer uses averages over SMOOTH seconds, never a single noisy sample.
    """

    SMOOTH = 0.5  # seconds averaged for "where is it now" and "where was it then"

    def __init__(self, window: float = 2.0, tolerance: float = 0.2):
        self.window = window  # seconds of history we keep
        self.tolerance = tolerance  # metres; moving less than this is standing still
        self._samples: deque[tuple[float, Point]] = deque()

    def update(self, time: float, pos: Point) -> None:
        self._samples.append((time, pos))
        while self._samples and time - self._samples[0][0] > self.window + 1e-9:
            self._samples.popleft()

    @property
    def seen(self) -> bool:
        return bool(self._samples)

    @property
    def last_time(self) -> float:
        """When we last saw it."""
        return self._samples[-1][0]

    def _mean(self, samples) -> Point:
        return (
            sum(p[0] for _, p in samples) / len(samples),
            sum(p[1] for _, p in samples) / len(samples),
        )

    def last(self) -> Point:
        """Where it is now: the average of the last SMOOTH seconds."""
        end = self._samples[-1][0]
        return self._mean([s for s in self._samples if s[0] >= end - self.SMOOTH])

    def _first(self) -> Point:
        """Where it was at the start of the window: the average of the first samples."""
        begin = self._samples[0][0]
        return self._mean([s for s in self._samples if s[0] <= begin + self.SMOOTH])

    def _covered(self) -> float:
        return self._samples[-1][0] - self._samples[0][0] if self._samples else 0.0

    def has_moved(self, min_distance: float = 0.3) -> bool:
        """True when it moved at least min_distance inside the window."""
        if self._covered() < 2 * self.SMOOTH:
            return False
        return distance(self._first(), self.last()) >= min_distance

    def direction(self) -> Point | None:
        """Unit vector of the walking direction inside the window, or None if standing."""
        if not self.has_moved():
            return None
        (x0, y0), (x1, y1) = self._first(), self.last()
        length = math.hypot(x1 - x0, y1 - y0)
        return ((x1 - x0) / length, (y1 - y0) / length)

    def is_stationary(self) -> bool:
        """True when it stood still for the whole window."""
        if self._covered() < self.window - 0.2 - 1e-9:
            return False  # not enough history yet (a few frames may be missed)
        return distance(self._first(), self.last()) <= self.tolerance
