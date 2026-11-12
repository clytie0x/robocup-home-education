"""Moving the robot: follow a person, or go to a point."""

from hmc.world import Point, Robot, distance

FOLLOW_DISTANCE = 1.5  # metres to keep behind the operator
GOAL_TOLERANCE = 0.5  # metres, "close enough" for go_to


def follow_step(robot: Robot, operator_pos: Point, dt: float) -> None:
    """One step of following: close the gap, but stop FOLLOW_DISTANCE away."""
    robot.drive_toward(operator_pos, dt, stop_distance=FOLLOW_DISTANCE)


def go_to_step(
    robot: Robot, goal: Point, dt: float, tolerance: float = GOAL_TOLERANCE
) -> bool:
    """One step toward a goal. Returns True once we are within tolerance."""
    robot.drive_toward(goal, dt, stop_distance=tolerance)
    return distance(robot.pos, goal) <= tolerance + 1e-9
