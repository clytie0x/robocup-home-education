"""Planning with the robot's real outline: narrow gaps that the old 0.36 m circle closed."""

import math

import pytest

from hmc import navigation
from hmc.navigation import OUTLINE_MARGIN, outline_gap, path_clear, plan_path
from hmc.world import Circle, Robot


@pytest.fixture(autouse=True)
def close_plan():
    """These tests are about the close plan, which only the real base uses."""
    navigation.CLOSE_PLAN, navigation._tight_skip = True, 0
    yield
    navigation.CLOSE_PLAN, navigation._tight_skip = False, 0


def walls(gap: float, y_mid: float = 0.0) -> list[Circle]:
    """Two walls across the way at x = 1.5, made of 0.05 m posts, `gap` metres apart between surfaces."""
    edge = gap / 2 + 0.05
    posts = []
    for k in range(25):
        posts.append(Circle(1.5, y_mid + edge + 0.1 * k, 0.05))
        posts.append(Circle(1.5, y_mid - edge - 0.1 * k, 0.05))
    return posts


def l_corridor(gap: float) -> list[Circle]:
    """An L-shaped corridor `gap` wide: east along y = 0 to x = 2, then north to y = 2."""
    h = gap / 2 + 0.05
    posts = []
    x = -0.5
    while x <= 2 + h:
        posts.append(Circle(x, -h, 0.05))  # south wall, past the corner
        if x <= 2 - h:
            posts.append(Circle(x, h, 0.05))  # north wall up to the inner corner
        x += 0.05
    y = h
    while y <= 3.0:
        posts.append(Circle(2 - h, y, 0.05))  # west wall of the north leg
        y += 0.05
    y = -h
    while y <= 3.0:
        posts.append(Circle(2 + h, y, 0.05))  # east wall of the north leg
        y += 0.05
    return posts


def test_a_0_65_m_gap_is_planned_through_wherever_it_sits_on_the_grid():
    for y_mid in (0.0, 0.03, 0.05, 0.07):
        path = plan_path((0, y_mid), (3, y_mid), walls(0.65, y_mid))
        assert path is not None, y_mid
        assert path[-1][0] > 2.5, y_mid  # through the gap, not stopped in front of it


def test_a_0_50_m_gap_is_not():
    path = plan_path((0, 0), (3, 0), walls(0.50))
    assert (
        path is None or path[-1][0] < 1.5
    )  # at best the free cell in front of the wall


def test_wide_open_space_keeps_the_old_wide_berth():
    # one post beside the straight line: the old circle (0.36 m) still decides the path
    post = [Circle(1.5, 0.3, 0.05)]
    path = plan_path((0, 0), (3, 0), post)
    assert path is not None
    assert min(post[0].clearance(p) for p in path) >= navigation.SAFETY + 0.26 - 1e-9


def test_no_corner_turn_inside_a_tight_corridor():
    # 0.70 m: turning at the corner leaves 0.094 m, the path goes round it
    shapes = l_corridor(0.70)
    path = plan_path((0, 0), (2, 2.5), shapes, heading=0.0)
    assert path is not None and path[-1][1] > 2.0
    assert path_clear(path, (0, 0), 0.0, shapes, OUTLINE_MARGIN) is None
    # 0.65 m: the corner square leaves 0.069 m for the turn, under the margin, so no path once the
    # corner is within the outline horizon (from further away the next plans refuse it)
    assert plan_path((1.0, 0), (2, 2.5), l_corridor(0.65), heading=0.0) is None


def test_outline_gap_faces_the_heading():
    wall = [Circle(0.0, 0.30, 0.05)]  # 0.25 m from the centre, to the left
    assert math.isclose(
        outline_gap((0, 0), 0.0, wall), 0.05, abs_tol=0.005
    )  # side towards it
    assert outline_gap((0, 0), math.radians(45), wall) < 0.05  # a corner towards it


def test_free_ahead_lets_the_robot_drive_off_centre_through_a_0_65_m_gap():
    assert navigation.free_ahead(Robot(x=0.6, y=0.05), walls(0.65), 0.0) < 1.0  # the circle alone stops it
    robot = Robot(x=0.6, y=0.05, heading=0.0)
    assert (
        navigation.free_ahead(robot, walls(0.65), 0.0, outline=True) >= navigation.LOOK_AHEAD - 1e-9
    )


def test_the_close_plan_walks_up_to_the_operator_it_follows():
    # The wide berth finds no way through the 0.65 m gap; the goal is the operator beyond it.
    operator = Circle(3.0, 0.0, 0.25)
    path = plan_path((0, 0), (3.0, 0.0), walls(0.65) + [operator], near_side=True, heading=0.0)
    assert path is not None and path[-1][0] > 2.0


def test_the_exact_outline_distance_matches_the_sampled_outline():
    import random
    rng = random.Random(1)
    for _ in range(200):
        c = Circle(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), 0.05)
        pos, heading = (rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1)), rng.uniform(-3, 3)
        sampled = min(c.clearance(p) for p in navigation.hmc_world.outline(pos[0], pos[1], heading, step=0.002))
        exact = navigation._outline_distance(c.x, c.y, pos, heading) - c.r
        if exact > 0:  # the sampled outline only knows the outside
            assert abs(exact - sampled) < 0.002


def test_a_failed_close_plan_is_not_retried_for_a_few_calls():
    plan_path((0, 0), (3, 0), walls(0.50))  # the close plan finds nothing
    assert navigation._tight_skip == navigation.TIGHT_COOLDOWN
    assert plan_path((0, 0), (3, 0), walls(0.65)) is None  # skipped
    navigation._tight_skip = 0
    assert plan_path((0, 0), (3, 0), walls(0.65)) is not None


def test_the_close_plan_gives_up_when_its_checks_run_out():
    shapes = walls(0.65)
    budget = [3]
    path = navigation._grid_path((0, 0), (3, 0), shapes, False, 0.25, [])
    assert path_clear(path, (0, 0), 0.0, shapes, OUTLINE_MARGIN, budget) == -1


def test_a_tight_spot_far_along_the_path_does_not_refuse_it():
    # a 0.50 m gap (0.05 m each side) 2.5 m away along the path: beyond the outline horizon
    path = [(0.1 * k, 0.0) for k in range(1, 31)]
    shapes = walls(0.50)
    shapes = [Circle(c.x + 1.0, c.y, c.r) for c in shapes]  # the gap at x = 2.5
    assert path_clear(path, (0, 0), 0.0, shapes, OUTLINE_MARGIN) is None
    near = [Circle(c.x - 1.5, c.y, c.r) for c in shapes]  # the same gap at x = 1.0: checked
    assert path_clear(path, (0, 0), 0.0, near, OUTLINE_MARGIN) is not None
