"""Scenario maps you can pick with  python -m hmc.run --map NAME

"default" is the small scenario in world.build_world().

"eindhoven2024" copies the layout of the official Carry My Luggage map of RoboCup 2024
in Eindhoven (github.com/RoboCupAtHome/Eindhoven2024, maps/stage_one/map_carry_my_luggage.png):
a team area with 3 rows of 7 tables, arenas A, B and C, audience stands, and starting
point A on the left of arena A. That map has no scale and no car, as the rulebook keeps
the car location secret. So these choices are ours:

- 1 pixel of the official map is 5 cm. That makes an arena 9.5 m wide and the aisle
  between the arenas and the team area 2.65 m wide.
- The car stands to the right of the team area, at (37, 22).
- The four obstacles of rulebook 5.1 are on the operator's way: a small object and a
  hard-to-see object on the way up from the start, then a crowd and retractable
  barriers in the aisle. Both leave a passage along the tables that is wide enough
  for the robot.

"bangkok2022", "bordeaux2023", "germanopen2024" and "portugalopen2024" copy the other
official Carry My Luggage maps (github.com/RoboCupAtHome/<event>/maps). In Bangkok,
German Open and Portugal Open the robot starts inside the arena and must first leave
through the exit door. Bordeaux 2023 starts outside, in the entering area. Only German
Open 2024 gives a scale (the arena is 10 m wide); for the others we also take 10 m. The
walls are traced by eye, and only the furniture near the path is kept. Doors are 1.0 m
wide; the rulebook recommends 0.915 m. German Open 2025 is left out: its official map is
a placeholder.

None of these maps shows the outside. So outside the exit we put the course of the
default map (obstacles, crowd, car, bags), turned to face away from the door. The robot
is never given any of this. It plans only from what it can see, and walls block its view.

All values are in metres. x goes right, y goes up (the official image is flipped).
"""

import math

from hmc.world import (
    Bag, Box, Circle, Obstacle, Operator, Person, Point, Robot, World, build_world,
    distance,
)  # fmt: skip

PX = 0.05  # metres per pixel of the official map
IMAGE_HEIGHT = 550  # pixels


def _box(x0: int, y0: int, x1: int, y1: int) -> Box:
    """A box from pixel corners of the official image (y down) to metres (y up)."""
    return Box(x0 * PX, (IMAGE_HEIGHT - y1) * PX, x1 * PX, (IMAGE_HEIGHT - y0) * PX)


def eindhoven2024(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    walls = []
    # The three rows of team tables
    for top, bottom in [(40, 70), (97, 127), (153, 183)]:
        for left in [53, 148, 243, 338, 432, 527, 621]:
            walls.append(_box(left, top, left + 75, bottom))
    walls.append(_box(53, 236, 243, 425))  # arena A
    walls.append(_box(284, 236, 473, 425))  # arena B
    walls.append(_box(510, 236, 698, 425))  # arena C
    walls.append(_box(53, 476, 698, 547))  # audience stands
    obstacles = [Obstacle("wall", box) for box in walls]

    obstacles += [
        Obstacle("small_object", Circle(0.95, 11.0, 0.12)),
        Obstacle("hard_to_see", Circle(1.3, 13.5, 0.15), seen_within=1.5),
        Obstacle("barriers", Box(27.0, 15.7, 28.5, 17.0)),
    ]

    start = (0.95, 7.6)  # starting point A
    car = (37.0, 22.0)
    bags = {
        "red": Bag("red", 38.5, 23.5),
        "blue": Bag("blue", 38.5, 20.5),
        "green": Bag("green", 37.0, 24.5),
    }
    operator = Operator(
        x=0.95,  # standing in front of the robot, which faces up the left side
        y=9.1,
        route=[(0.95, 17.0), (19.2, 17.0), (19.6, 17.7), (36.0, 17.7), car],
        target_bag=target_bag,
    )
    # The crowd stands in the aisle. The queue walks back along the aisle, down the left
    # side, and stops in a line on the start's left (as on every map), each person 0.7 m
    # behind the one in front, so it never walks into the robot there.
    path = [(19.0, 16.4), (0.95, 16.4), (0.95, 9.6), (-0.2, 9.0)]
    people = [
        Person(i + 1, x, y, path + [(-0.2, 7.0 + 0.7 * i)])
        for i, (x, y) in enumerate(((20.0, 16.0), (20.0, 16.75), (20.75, 16.4)))
    ]
    robot = Robot(x=start[0], y=start[1], heading=1.5708, max_speed=robot_speed)
    return World(
        robot=robot, operator=operator, bags=bags, start=start, car=car,
        obstacles=obstacles, people=people,
    )  # fmt: skip


def _walls(*boxes: tuple[float, float, float, float]) -> list[Obstacle]:
    return [Obstacle("wall", Box(*box)) for box in boxes]


def _with_outside_course(
    walls: list[Obstacle],
    start: Point,
    way_out: list[Point],
    facing: Point,
    way_back: list[Point],
    target_bag: str,
    robot_speed: float,
) -> World:
    """A map: the given walls, plus the default course placed outside the exit.

    way_out: the operator's waypoints from the start to just outside the exit.
    facing: the unit direction the course goes in, away from the exit.
    way_back: where the queue walks after coming back in. It ends next to the start.
    """
    origin = way_out[-1] if way_out else start
    side = (-facing[1], facing[0])  # the course's "left"

    def turn(p: Point) -> Point:
        u, v = p
        return (
            origin[0] + u * facing[0] + v * side[0],
            origin[1] + u * facing[1] + v * side[1],
        )

    def turn_shape(shape):
        if isinstance(shape, Circle):
            x, y = turn((shape.x, shape.y))
            return Circle(x, y, shape.r)
        x0, y0 = turn((shape.x_min, shape.y_min))
        x1, y1 = turn((shape.x_max, shape.y_max))
        return Box(min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))

    course = build_world(target_bag=target_bag)
    obstacles = walls + [
        Obstacle(o.kind, turn_shape(o.shape), o.seen_within) for o in course.obstacles
    ]
    bags = {name: Bag(name, *turn(bag.pos)) for name, bag in course.bags.items()}

    first = way_out[0] if way_out else turn((1.5, 0.0))
    heading = math.atan2(first[1] - start[1], first[0] - start[0])
    gap = 1.5 / max(distance(start, first), 1e-9)  # the operator stands 1.5 m in front
    operator = Operator(
        x=start[0] + (first[0] - start[0]) * gap,
        y=start[1] + (first[1] - start[1]) * gap,
        route=list(way_out) + [turn(p) for p in course.operator.route],
        target_bag=target_bag,
    )

    queue_route = [turn((8.0, -1.0)), turn((1.0, -1.0))] + list(reversed(way_out))
    people = []
    for i, person in enumerate(course.people):
        # Each person stops 0.7 m behind the one in front, so they end up in a line.
        *path, end = queue_route + way_back
        before = path[-1]
        back = 0.7 * i / max(distance(before, end), 1e-9)
        end = (
            end[0] + (before[0] - end[0]) * back,
            end[1] + (before[1] - end[1]) * back,
        )
        people.append(Person(person.id, *turn(person.pos), path + [end]))

    robot = Robot(x=start[0], y=start[1], heading=heading, max_speed=robot_speed)
    return World(
        robot=robot, operator=operator, bags=bags, start=start, car=turn(course.car),
        obstacles=obstacles, people=people,
    )  # fmt: skip


def germanopen2024(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    """RoboCup German Open 2024, Kassel. A 10 m x 10 m flat. Start in the living room."""
    walls = _walls(
        (-0.1, -0.1, 10.1, 0.0),  # bottom
        (-0.1, 0.0, 0.0, 2.6), (-0.1, 3.6, 0.0, 10.0),  # left, with the entrance
        (10.0, 0.0, 10.1, 10.0),  # right
        (-0.1, 10.0, 4.2, 10.1), (5.2, 10.0, 10.1, 10.1),  # top, with the exit
        (0.0, 6.0, 4.0, 6.1),  # living room / bedroom
        (3.95, 6.0, 4.05, 7.0), (3.95, 8.0, 4.05, 10.0),  # bedroom / corridor
        (5.95, 5.0, 6.05, 6.6), (5.95, 7.6, 6.05, 10.0),  # corridor / office
        (6.0, 4.95, 7.0, 5.05), (8.0, 4.95, 10.0, 5.05),  # office / kitchen
        (5.95, 0.0, 6.05, 2.0), (5.95, 3.0, 6.05, 5.0),  # living room / kitchen
        (5.53, 4.66, 5.93, 5.66),  # bookshelf
    )  # fmt: skip
    return _with_outside_course(
        walls,
        start=(4.72, 3.85),
        way_out=[(4.9, 6.6), (4.7, 9.6), (4.7, 10.7)],
        facing=(0.0, 1.0),
        way_back=[(3.0, 4.6)],
        target_bag=target_bag,
        robot_speed=robot_speed,
    )


def bangkok2022(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    """RoboCup 2022, Bangkok. A 10 m x 10 m flat with thick walls. Start next to the exit."""
    walls = _walls(
        (-0.3, -0.3, 2.15, 0.0), (3.15, -0.3, 6.9, 0.0), (7.8, -0.3, 10.3, 0.0),  # bottom
        (-0.3, 0.0, 0.0, 10.0), (10.0, 0.0, 10.3, 10.0), (-0.3, 10.0, 10.3, 10.3),
        (4.9, 0.0, 5.15, 2.7), (4.9, 3.5, 5.15, 7.8), (4.9, 8.8, 5.15, 10.0),  # middle
        (0.0, 4.87, 0.8, 5.04), (1.8, 4.87, 7.4, 5.04), (8.4, 4.87, 10.0, 5.04),
        (0.42, 0.44, 1.26, 3.14),  # 20, a cupboard
        (4.43, 0.44, 4.89, 1.67),  # 22, a shelf
        (2.7, 4.0, 4.47, 4.86),  # 19, a desk
    )  # fmt: skip
    return _with_outside_course(
        walls,
        start=(2.65, 2.17),
        way_out=[(2.65, 0.4), (2.65, -0.7)],
        facing=(0.0, -1.0),
        way_back=[(3.8, 0.9), (3.8, 3.4)],
        target_bag=target_bag,
        robot_speed=robot_speed,
    )


def portugalopen2024(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    """RoboCup Portugal Open 2024. A 10 m x 10 m flat. Start in the office, by the exit."""
    walls = _walls(
        (-0.15, -0.15, 2.2, 0.0), (3.2, -0.15, 7.1, 0.0), (8.05, -0.15, 10.15, 0.0),
        (-0.15, 0.0, 0.0, 10.0), (10.0, 0.0, 10.15, 10.0), (-0.15, 10.0, 10.15, 10.15),
        (4.87, 0.0, 5.04, 2.04), (4.87, 2.96, 5.04, 7.0), (4.87, 7.93, 5.04, 10.0),
        (0.0, 4.87, 1.39, 5.03), (2.33, 4.87, 7.58, 5.03), (8.55, 4.87, 10.0, 5.03),
        (0.14, 0.77, 0.71, 2.17),  # 7, the desk
        (4.32, 3.38, 4.77, 4.05),  # 5, the bookshelf
    )  # fmt: skip
    return _with_outside_course(
        walls,
        start=(2.58, 3.69),
        way_out=[(2.7, 0.4), (2.7, -0.65)],
        facing=(0.0, -1.0),
        way_back=[(1.4, 0.9), (1.4, 3.0)],
        target_bag=target_bag,
        robot_speed=robot_speed,
    )


def bordeaux2023(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    """RoboCup 2023, Bordeaux. The OPL starting point is outside, above the arenas."""
    walls = _walls(
        (0.38, 10.2, 21.1, 10.27),  # the long wall between the entering area and arenas
        (0.35, 0.03, 10.35, 9.8),  # OPL arena (the robot never goes in)
        (11.1, 0.03, 21.1, 9.8),  # SSPL/OPL arena
    )  # fmt: skip
    return _with_outside_course(
        walls,
        start=(1.16, 11.18),
        way_out=[],
        facing=(0.0, 1.0),
        way_back=[(3.3, 11.4)],
        target_bag=target_bag,
        robot_speed=robot_speed,
    )


MAPS = {
    "default": build_world,
    "bangkok2022": bangkok2022,
    "bordeaux2023": bordeaux2023,
    "eindhoven2024": eindhoven2024,
    "germanopen2024": germanopen2024,
    "portugalopen2024": portugalopen2024,
}
