"""The simulation as close to the real robot as we can make it (2026-11-13)."""

import math

from hmc.perception import MotionTracker
from hmc.world import Circle, Robot, body_clearance


def test_touching_is_judged_with_the_real_outline():
    # AT-56: a post 0.22 m beside the centre misses the 0.40 m square but is inside the
    # planner's 0.26 m circle; ahead of the corner it touches.
    robot = Robot(x=0.0, y=0.0, heading=0.0)
    beside = Circle(0.0, 0.25, 0.03)  # edge 0.22 m from the centre
    assert body_clearance(robot, beside.clearance) > 0
    corner = Circle(0.19, 0.19, 0.03)  # on the cut corner
    assert body_clearance(robot, corner.clearance) < 0


def test_the_outline_turns_with_the_robot():
    side = Circle(0.0, 0.24, 0.02)  # just beside the robot, outside the square
    assert body_clearance(Robot(heading=0.0), side.clearance) > 0
    # Turned by 45 degrees, a corner points that way and reaches it.
    assert body_clearance(Robot(heading=math.radians(45)), side.clearance) < 0


def test_a_small_object_under_the_robot_counts_as_touching():
    small = Circle(0.14, 0.0, 0.05)  # inside the outline, touching no edge of it
    assert body_clearance(Robot(), small.clearance) < 0


def test_a_walking_person_is_predicted_ahead_of_the_smoothed_spot():
    tracker = MotionTracker()
    for i in range(21):  # 1 m/s along x for 2 s
        tracker.update(i * 0.1, (i * 0.1, 0.0))
    smoothed = tracker.last()[0]
    predicted = tracker.predict(2.0)[0]
    assert predicted > smoothed + 0.2  # the smoothed spot lags by about 0.25 m
    assert abs(predicted - 2.0) < 0.05


def test_a_standing_person_is_not_moved():
    tracker = MotionTracker()
    for i in range(21):
        tracker.update(i * 0.1, (1.0, 1.0))
    assert tracker.predict(5.0) == tracker.last()
