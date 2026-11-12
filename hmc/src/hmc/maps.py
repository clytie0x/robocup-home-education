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

All values are in metres. x goes right, y goes up (the official image is flipped).
"""

from hmc.world import (
    Bag, Box, Circle, Obstacle, Operator, Person, Robot, World, build_world,
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
    # The crowd stands in the aisle. The queue walks back along the aisle and down the left side.
    queue_route = [(19.0, 16.4), (0.95, 16.4), (0.95, 4.0)]
    people = [
        Person(1, 20.0, 16.0, list(queue_route)),
        Person(2, 20.0, 16.75, list(queue_route)),
        Person(3, 20.75, 16.4, list(queue_route)),
    ]
    robot = Robot(x=start[0], y=start[1], heading=1.5708, max_speed=robot_speed)
    return World(
        robot=robot, operator=operator, bags=bags, start=start, car=car,
        obstacles=obstacles, people=people,
    )  # fmt: skip


MAPS = {"default": build_world, "eindhoven2024": eindhoven2024}
