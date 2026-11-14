"""Where the real robot's sensors can see. Used by perception when --sensors robot.

The robot has three ways to see:
- an RGB camera with a person/bag detector (YOLO). It looks forward only.
- the depth image of the same camera (ASUS Xtion). Forward only, and only from 0.6 m
  to 3.5 m. It is the only sensor that sees things lower than the lidar.
- a 2D lidar (LDS-03), 360 degrees, but only in one flat plane 0.22 m above the floor.
  It sees walls, the legs of people and the posts of retractable barriers, but not the
  small object on the ground, nor who a person is.

The numbers are typical values for these sensors, not measurements on our robot.
"""

import math

from hmc.world import Box, Circle, Point, World, distance, wrap_angle

CAMERA_FOV = math.radians(58)  # horizontal field of view of the Xtion
CAMERA_RANGE = 6.0  # metres; YOLO finds people and bags up to here
POINTING_RANGE = 4.0  # metres; arm keypoints are good enough up to here
DEPTH_MIN, DEPTH_MAX = 0.6, 3.5  # metres; the depth image only works in this range
LIDAR_RANGE = 8.0  # metres
LEG_TRACK_GAP = (
    0.5  # seconds; the lidar keeps a person's track if it misses less than this
)
POST_RADIUS = 0.05  # metres; a barrier post as the lidar sees it


def _in_view(world: World, p: Point) -> bool:
    robot = world.robot
    bearing = math.atan2(p[1] - robot.y, p[0] - robot.x) - robot.heading
    return abs(wrap_angle(bearing)) <= CAMERA_FOV / 2


def _clear(world: World, p: Point) -> bool:
    """No wall between the robot and p."""
    a = world.robot.pos
    return not any(o.shape.blocks(a, p) for o in world.obstacles if o.kind == "wall")


def camera_sees(world: World, p: Point, max_range: float = CAMERA_RANGE) -> bool:
    return (
        distance(world.robot.pos, p) <= max_range
        and _in_view(world, p)
        and _clear(world, p)
    )


def lidar_sees(world: World, p: Point) -> bool:
    return distance(world.robot.pos, p) <= LIDAR_RANGE and _clear(world, p)


def _nearest_point(shape, p: Point) -> Point:
    if isinstance(shape, Circle):
        return (shape.x, shape.y)
    return (
        min(max(p[0], shape.x_min), shape.x_max),
        min(max(p[1], shape.y_min), shape.y_max),
    )


def depth_sees(world: World, shape, max_range: float = DEPTH_MAX) -> bool:
    gap = shape.clearance(world.robot.pos)
    near = _nearest_point(shape, world.robot.pos)
    return DEPTH_MIN <= gap <= max_range and _in_view(world, near)


def barrier_posts(box: Box) -> list[Circle]:
    """What the lidar sees of retractable barriers: the posts at the corners."""
    corners = [(box.x_min, box.y_min), (box.x_min, box.y_max),
               (box.x_max, box.y_min), (box.x_max, box.y_max)]  # fmt: skip
    return [Circle(x, y, POST_RADIUS) for x, y in corners]
