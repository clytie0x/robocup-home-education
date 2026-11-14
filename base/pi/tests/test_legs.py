# Copied from xohm@pi:~/pi-drive/test_legs.py on 2026-11-13, adapted: chase tests and camera_at dropped, lock point is a lidar-frame (x, y) passed as lock_at.
import math
import unittest

from legs import LegTracker, candidates, moved


def ray_circle(deg, cx, cy, r):
    dx, dy = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    b = dx * cx + dy * cy
    c = cx * cx + cy * cy - r * r
    disc = b * b - c
    if disc < 0 or b <= 0:
        return None
    return b - math.sqrt(disc)


def scan_of(legs=(), wall=3.0, step=0.8):
    """A 360-degree scan in a round room of radius `wall`, with 0.05 m legs at the given points."""
    scan = []
    deg = -180.0
    while deg < 180.0:
        hits = [d for x, y in legs if (d := ray_circle(deg, x, y, 0.05)) is not None]
        scan.append((deg, min(hits) if hits else wall))
        deg += step
    return scan


def person(x, y):
    return [(x, y + 0.1), (x, y - 0.1)]


class Candidates(unittest.TestCase):
    def test_two_legs_are_one_person(self):
        found = candidates(scan_of(person(1.5, 0.3)))
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0][0], 1.5, delta=0.1)
        self.assertAlmostEqual(found[0][1], 0.3, delta=0.05)

    def test_a_wall_is_not_a_person(self):
        self.assertEqual(candidates(scan_of()), [])


class Tracker(unittest.TestCase):
    def test_keeps_the_person_after_they_leave_the_camera_view(self):
        t = LegTracker()
        self.assertIsNotNone(t.update(scan_of(person(1.5, 0.0)), 0.0, lock_at=(1.5, 0.0)))
        x, y, now = 1.5, 0.0, 0.0
        for _ in range(20):  # walks to the robot's left side and behind, 0.1 m per scan
            now += 0.1
            y += 0.1
            x -= 0.08
            pos = t.update(scan_of(person(x, y)), now)
        self.assertIsNotNone(pos)
        # Side-on, one leg hides the other, so the centre is the visible leg: up to 0.1 m off.
        self.assertAlmostEqual(pos[0], x, delta=0.2)
        self.assertAlmostEqual(pos[1], y, delta=0.2)
        self.assertGreater(math.degrees(math.atan2(pos[1], pos[0])), 60)  # far outside 58 deg view

    def test_does_not_lock_without_the_camera(self):
        self.assertIsNone(LegTracker().update(scan_of(person(1.5, 0.0)), 0.0))

    def test_stays_on_the_first_person_when_a_second_one_appears(self):
        t = LegTracker()
        t.update(scan_of(person(1.5, 0.0)), 0.0, lock_at=(1.5, 0.0))
        pos = t.update(scan_of(person(1.6, 0.0) + person(1.5, -1.2)), 0.1)
        self.assertAlmostEqual(pos[1], 0.0, delta=0.1)

    def test_two_people_side_by_side_then_apart_does_not_switch(self):
        t = LegTracker(); t.update(scan_of(person(1.5, 0.0)), 0.0, lock_at=(1.5, 0.0))
        t.update(scan_of(person(1.5, 0.0) + person(1.5, -0.45)), 0.1)        # shoulder to shoulder
        for i in range(1, 15):                                                # the other walks off
            pos = t.update(scan_of(person(1.5, 0.0) + person(1.5, -0.45 - 0.1 * i)), 0.1 + 0.1 * i)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos[1], 0.0, delta=0.15)

    def test_other_person_at_arm_spacing_then_apart_does_not_switch(self):
        for side in (-1, 1):  # 0.25 m is where the leg pairing can mix the two people up
            with self.subTest(side=side):
                t = LegTracker(); t.update(scan_of(person(1.5, 0.0)), 0.0, lock_at=(1.5, 0.0))
                t.update(scan_of(person(1.5, 0.0) + person(1.5, side * 0.25)), 0.1)
                for i in range(1, 15):
                    other = person(1.5, side * (0.25 + 0.1 * i))
                    pos = t.update(scan_of(person(1.5, 0.0) + other), 0.1 + 0.1 * i)
                self.assertIsNotNone(pos)
                self.assertAlmostEqual(pos[1], 0.0, delta=0.15)

    def test_lost_after_the_person_is_gone(self):
        t = LegTracker()
        t.update(scan_of(person(1.5, 0.0)), 0.0, lock_at=(1.5, 0.0))
        self.assertIsNotNone(t.update(scan_of(), 1.0))
        self.assertIsNone(t.update(scan_of(), 2.0))

    def test_predict_turning_left_moves_a_point_to_the_right(self):
        x, y = moved((1.0, 0.0), 0.0, 0.5, 1.0)
        self.assertLess(y, 0.0)
        x, _ = moved((1.0, 0.0), 0.2, 0.0, 1.0)
        self.assertAlmostEqual(x, 0.8)


if __name__ == "__main__":
    unittest.main()
