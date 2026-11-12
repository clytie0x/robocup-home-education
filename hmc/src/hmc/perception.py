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


def _blocks(box, a: Point, b: Point) -> bool:
    """True if the straight line from a to b passes through the box (slab method)."""
    t_in, t_out = 0.0, 1.0
    for start, delta, low, high in (
        (a[0], b[0] - a[0], box.x_min, box.x_max),
        (a[1], b[1] - a[1], box.y_min, box.y_max),
    ):
        if abs(delta) < 1e-12:
            if not low <= start <= high:
                return False
            continue
        t1, t2 = (low - start) / delta, (high - start) / delta
        t_in, t_out = max(t_in, min(t1, t2)), min(t_out, max(t1, t2))
        if t_in > t_out:
            return False
    return True


def can_see(world: World, p: Point) -> bool:
    """In sensor range, and no wall in between. Only walls block the view."""
    if distance(world.robot.pos, p) > SENSOR_RANGE:
        return False
    return not any(
        _blocks(o.shape, world.robot.pos, p)
        for o in world.obstacles
        if o.kind == "wall"
    )


def see_operator(world: World) -> Point | None:
    """Operator position, or None when we cannot see them."""
    if not can_see(world, world.operator.pos):
        return None
    return world.operator.pos


def see_bags(world: World) -> dict[str, Point]:
    """Positions of the bags inside sensor range, by name."""
    return {
        name: bag.pos for name, bag in world.bags.items() if can_see(world, bag.pos)
    }


def see_obstacles(world: World) -> list[Shape]:
    """Shapes of the obstacles we can see. We do not learn what kind of obstacle it is."""
    seen = []
    for obstacle in world.obstacles:
        reach = obstacle.seen_within or SENSOR_RANGE
        if obstacle.shape.clearance(world.robot.pos) <= reach:
            seen.append(obstacle.shape)
    return seen


def see_people(world: World) -> dict[int, Point]:
    """Positions of the other people (not the operator) in range, by tracker id."""
    return {
        person.id: person.pos for person in world.people if can_see(world, person.pos)
    }


def see_pointing(world: World) -> Ray | None:
    """The pointing ray, only if we can see the operator."""
    if see_operator(world) is None:
        return None
    ray = world.pointing_ray()
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
    """Remembers recent positions so we can tell when a person stood still."""

    def __init__(self, window: float = 2.0, tolerance: float = 0.1):
        self.window = window
        self.tolerance = tolerance
        self._samples: deque[tuple[float, Point]] = deque()

    def update(self, time: float, pos: Point) -> None:
        self._samples.append((time, pos))
        while self._samples and time - self._samples[0][0] > self.window + 1e-9:
            self._samples.popleft()

    def has_moved(self, min_distance: float = 0.3) -> bool:
        """True when the person moved at least min_distance inside the window."""
        if len(self._samples) < 2:
            return False
        return distance(self._samples[0][1], self._samples[-1][1]) >= min_distance

    def last(self) -> Point:
        """The last position we saw."""
        return self._samples[-1][1]

    def direction(self) -> Point | None:
        """Unit vector of the walking direction inside the window, or None if standing."""
        if not self.has_moved():
            return None
        (x0, y0), (x1, y1) = self._samples[0][1], self._samples[-1][1]
        length = math.hypot(x1 - x0, y1 - y0)
        return ((x1 - x0) / length, (y1 - y0) / length)

    def is_stationary(self) -> bool:
        if len(self._samples) < 2:
            return False
        first_time, first_pos = self._samples[0]
        last_time, last_pos = self._samples[-1]
        if last_time - first_time < self.window - 1e-9:
            return False  # not enough history yet
        return all(
            distance(first_pos, pos) <= self.tolerance for _, pos in self._samples
        )
