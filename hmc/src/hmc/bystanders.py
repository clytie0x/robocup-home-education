"""Passers-by: other people who walk around outside during the whole test.

The rulebook (5.1, setup) says: "There are no restrictions on other people walking by or
standing around throughout the complete task." These people are not the crowd and not
the queue, and they give no points. They only make the task harder.

Each passer-by walks from one random free spot to the next, sometimes waits, and stops
to let the robot pass when it comes close, as polite people do. A fixed seed makes every
run the same.
"""

import math
import random
from dataclasses import dataclass, field

from hmc.world import PERSON_RADIUS, Box, Point, World, distance, walk_route

YIELD_DISTANCE = 1.2  # metres between centres; closer than this a passer-by steps away
STEP_AWAY_SPEED = 0.4  # m/s
SPOT_CLEARANCE = 0.8  # metres; random spots are at least this far from any obstacle
FIRST_ID = 100  # passers-by get tracker ids from 100 up


@dataclass
class Bystander:
    id: int
    x: float
    y: float
    area: tuple[float, float, float, float]  # x_min, y_min, x_max, y_max to walk in
    rng: random.Random
    speed: float = 1.2
    route: list[Point] = field(default_factory=list)
    wait: float = 0.0
    walking: bool = True

    @property
    def pos(self) -> Point:
        return (self.x, self.y)

    def step(self, dt: float, world: World) -> None:
        robot = world.robot.pos
        gap = distance(self.pos, robot)
        if gap < YIELD_DISTANCE:
            # Step away from the robot to let it pass. Standing still is not enough:
            # two people standing next to the robot can box it in.
            self.route = []
            if gap > 1e-9:
                # Step to the side of the robot's way, the side we already stand on.
                # Stepping straight away from it kept people walking just ahead of a
                # robot in a corridor, so it could never pass.
                h = world.robot.heading
                side = (-math.sin(h), math.cos(h))
                ax, ay = self.x - robot[0], self.y - robot[1]
                if ax * side[0] + ay * side[1] < 0:
                    side = (-side[0], -side[1])
                spot = (
                    self.x + side[0] * STEP_AWAY_SPEED * dt,
                    self.y + side[1] * STEP_AWAY_SPEED * dt,
                )
                if all(
                    o.shape.clearance(spot) > PERSON_RADIUS for o in world.obstacles
                ):
                    self.x, self.y = spot
            return
        if self.wait > 0:
            self.wait -= dt
            return
        if not self.route:
            spot = _free_spot(world, self.area, self.rng, start=self.pos)
            if spot is None:
                return
            self.route = [spot]
            self.wait = self.rng.uniform(0.0, 3.0)  # stand around for a moment first
            return
        walk_route(self, dt)


def _free_spot(world, area, rng, start=None) -> Point | None:
    """A random spot away from obstacles, that we can walk to in a straight line."""
    for _ in range(50):
        spot = (rng.uniform(area[0], area[2]), rng.uniform(area[1], area[3]))
        if any(o.shape.clearance(spot) < SPOT_CLEARANCE for o in world.obstacles):
            continue
        if start is not None and any(
            isinstance(o.shape, Box) and o.shape.blocks(start, spot)
            for o in world.obstacles
        ):
            continue
        if distance(spot, world.start) < 3.0:
            continue  # keep the start area free
        return spot
    return None


def add_bystanders(world: World, count: int, seed: int = 0) -> None:
    """Put `count` passers-by around the outside part of the course."""
    if count <= 0:
        return
    # The area: around the car, the obstacles (not the walls) and the crowd.
    points = [world.car] + [p.pos for p in world.people]
    for o in world.obstacles:
        if o.kind != "wall":
            x0, y0, x1, y1 = o.shape.bounds()
            points += [(x0, y0), (x1, y1)]
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    area = (min(xs) - 2.0, min(ys) - 2.0, max(xs) + 2.0, max(ys) + 2.0)
    rng = random.Random(seed)
    for i in range(count):
        spot = _free_spot(world, area, rng)
        if spot is None:
            continue
        speed = rng.uniform(1.0, 1.4)  # people walk about 1.0 to 1.4 m/s
        world.bystanders.append(
            Bystander(
                FIRST_ID + i, spot[0], spot[1], area, random.Random(rng.random()), speed
            )
        )
