"""A tiny 2D kinematic world: one robot, one scripted operator, a few bags.

Everything here is "ground truth". The robot code must not read the world
directly. It asks the perception module instead.
"""

import math
from dataclasses import dataclass, field

Point = tuple[float, float]

TIME_LIMIT = 300.0  # seconds, the 5:00 limit from the score sheet
DT = 0.1  # seconds per simulation step


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the range -pi .. pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Robot:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians, 0 = facing +x
    max_speed: float = 1.0  # m/s
    max_turn_rate: float = 2.0  # rad/s

    @property
    def pos(self) -> Point:
        return (self.x, self.y)

    def drive_toward(
        self, target: Point, dt: float, stop_distance: float = 0.0
    ) -> None:
        """Turn toward the target and drive until stop_distance away from it."""
        remaining = distance(self.pos, target) - stop_distance
        if remaining <= 0:
            return
        wanted = math.atan2(target[1] - self.y, target[0] - self.x)
        error = wrap_angle(wanted - self.heading)
        turn = max(-self.max_turn_rate * dt, min(self.max_turn_rate * dt, error))
        self.heading = wrap_angle(self.heading + turn)
        # Drive slower while still turning, and never drive backwards.
        speed = self.max_speed * max(0.0, math.cos(error - turn))
        step = min(speed * dt, remaining)
        self.x += math.cos(self.heading) * step
        self.y += math.sin(self.heading) * step


# Bag states
ON_FLOOR = "on_floor"
CARRIED = "carried"
DROPPED = "dropped"  # fell by accident, costs points
PLACED = "placed"  # put down on purpose, free


@dataclass
class Bag:
    name: str
    x: float
    y: float
    state: str = ON_FLOOR

    @property
    def pos(self) -> Point:
        return (self.x, self.y)


@dataclass
class Operator:
    """A volunteer that follows a fixed script.

    Waits until told to start, walks along the route, stops at the last
    point (the car), waits a moment, then points at the target bag.
    """

    x: float
    y: float
    route: list[Point]
    target_bag: str
    speed: float = 0.8  # m/s, a calm walking pace
    point_delay: float = 2.0  # seconds between stopping and pointing
    walking: bool = False
    pointing: bool = False
    _stopped_for: float = 0.0
    _target_pos: Point = (0.0, 0.0)  # filled in by World

    @property
    def pos(self) -> Point:
        return (self.x, self.y)

    def start_walking(self) -> None:
        self.walking = True

    def step(self, dt: float) -> None:
        if not self.walking:
            return
        if self.route:
            goal = self.route[0]
            left = distance(self.pos, goal)
            move = min(self.speed * dt, left)
            if left > 0:
                self.x += (goal[0] - self.x) / left * move
                self.y += (goal[1] - self.y) / left * move
            if distance(self.pos, goal) < 1e-9:
                self.route.pop(0)
        else:
            self._stopped_for += dt
            if self._stopped_for >= self.point_delay:
                self.pointing = True


@dataclass
class World:
    robot: Robot
    operator: Operator
    bags: dict[str, Bag]
    start: Point
    car: Point  # the robot is never told this
    time: float = 0.0
    drop_count: int = 0
    trail: list[Point] = field(default_factory=list)

    def pointing_ray(self) -> tuple[Point, float] | None:
        """The operator's pointing ray (origin, angle) or None if not pointing."""
        if not self.operator.pointing:
            return None
        target = self.bags[self.operator.target_bag]
        angle = math.atan2(target.y - self.operator.y, target.x - self.operator.x)
        return (self.operator.pos, angle)

    def carried_bag(self) -> Bag | None:
        for bag in self.bags.values():
            if bag.state == CARRIED:
                return bag
        return None

    def step(self, dt: float = DT) -> None:
        self.operator.step(dt)
        bag = self.carried_bag()
        if bag is not None:  # a carried bag moves with the robot
            bag.x, bag.y = self.robot.pos
        self.time = round(self.time + dt, 6)


def build_world(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    """The default scenario. The car is at (12, 6), three bags lie around it."""
    start = (0.0, 0.0)
    car = (12.0, 6.0)
    bags = {
        "red": Bag("red", 13.5, 7.5),
        "blue": Bag("blue", 13.5, 4.5),
        "green": Bag("green", 12.0, 8.5),
    }
    operator = Operator(
        x=1.5,  # standing in front of the robot
        y=0.0,
        route=[(8.0, 0.0), (8.0, 6.0), car],
        target_bag=target_bag,
    )
    robot = Robot(x=start[0], y=start[1], max_speed=robot_speed)
    return World(robot=robot, operator=operator, bags=bags, start=start, car=car)
