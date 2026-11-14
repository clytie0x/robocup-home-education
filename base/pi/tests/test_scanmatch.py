import math
import unittest

import geometry
import params
import scanmatch

P = params.load()


def room_scan(pose, half=2.0, step_deg=1.0):
    """What the LiDAR sees from base pose (x, y, yaw) inside a square room of side 2 * half.

    Ray casting, so each pose samples different wall points, like a real scan.
    """
    x, y, yaw = pose
    sx, sy = geometry.base_to_odom((P["lidar_x"], 0.0), pose)  # the LiDAR sits ahead of the centre
    scan = []
    for i in range(int(360 / step_deg)):
        deg = -180 + i * step_deg
        a = yaw + math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        hits = [(w - sx) / c for w in (-half, half) if abs(c) > 1e-9] + [(w - sy) / s for w in (-half, half) if abs(s) > 1e-9]
        scan.append((deg, min(t for t in hits if t > 0)))
    return scan


def offset_scan(pose, half, step_deg, start_deg):
    """room_scan with the first ray at start_deg: each revolution of the LDS-03 starts somewhere else."""
    x, y, yaw = pose
    sx, sy = geometry.base_to_odom((P["lidar_x"], 0.0), pose)
    scan = []
    for i in range(int(360 / step_deg)):
        deg = -180 + start_deg + i * step_deg
        a = yaw + math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        hits = [(w - sx) / c for w in (-half, half) if abs(c) > 1e-9] + [(w - sy) / s for w in (-half, half) if abs(s) > 1e-9]
        scan.append((deg, min(t for t in hits if t > 0)))
    return scan


def world_scan(pose, walls, legs=(), step_deg=0.9, start_deg=0.0, reach=8.0):
    """What the LiDAR sees from base pose (x, y, yaw) among wall segments ((x0, y0), (x1, y1)) and
    0.05 m legs at (x, y); 0 (no return) where a ray meets nothing within reach. 400 rays as on the robot."""
    sx, sy = geometry.base_to_odom((P["lidar_x"], 0.0), pose)
    scan = []
    for i in range(int(360 / step_deg)):
        deg = -180 + start_deg + i * step_deg
        a = pose[2] + math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        hits = []
        for (x0, y0), (x1, y1) in walls:
            ex, ey, wx, wy = x1 - x0, y1 - y0, x0 - sx, y0 - sy
            den = c * ey - s * ex
            if abs(den) > 1e-12:
                t, u = (wx * ey - wy * ex) / den, (wx * s - wy * c) / den
                if t > 0 and 0 <= u <= 1:
                    hits.append(t)
        for lx, ly in legs:
            bx, by = lx - sx, ly - sy
            along = bx * c + by * s
            off2 = bx * bx + by * by - along * along
            if along > 0 and off2 < 0.05 ** 2:
                hits.append(along - math.sqrt(0.05 ** 2 - off2))
        t = min(hits, default=0.0)
        scan.append((deg, t if t <= reach else 0.0))
    return scan


def box(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


ROOM = box(-1.5, -2.0, 3.0, 2.5)  # 4.5 m by 4.5 m, the base off centre, four corners
CORRIDOR = [((-30.0, -0.8), (30.0, -0.8)), ((-30.0, 0.8), (30.0, 0.8))]  # ends beyond the LiDAR's reach


def crowd(x, y, n=8, radius=0.5):
    """Legs of n people standing in a ring around (x, y)."""
    legs = []
    for i in range(n):
        a = 2 * math.pi * i / n
        px, py = x + radius * math.cos(a), y + radius * math.sin(a)
        legs += [(px - 0.1 * math.sin(a), py + 0.1 * math.cos(a)), (px + 0.1 * math.sin(a), py - 0.1 * math.cos(a))]
    return legs


class Thinned(unittest.TestCase):
    def test_thinned_match_keeps_the_surfaces_of_the_full_scan(self):
        # 400 returns a revolution, as on the robot, each revolution's rays starting somewhere else.
        # Thinning the scans first left too few neighbours for the surface lines and turned a
        # right drift into a left one on the robot (base_check log 2026-11-14 05:13); thinned
        # surface lines no longer than SURFACE_GAP read 0.5 s at 0.12 m/s 9 % short in a 5 m room.
        for dt in (0.1, 0.5):  # one revolution, and M1's BASELINE
            moved = (0.12 * dt, 0.0, -0.0071 * dt)
            for phase in (0.1, 0.45, 0.8):
                a = offset_scan((0, 0, 0), 2.5, 0.9, 0.0)
                b = offset_scan(moved, 2.5, 0.9, 0.9 * phase)
                dx, dy, dyaw, agree = scanmatch.match(a, b, P, max_points=120)
                self.assertAlmostEqual(dx / moved[0], 1.0, delta=0.015)
                self.assertAlmostEqual(dy, 0.0, delta=0.002)
                self.assertAlmostEqual(math.degrees(dyaw - moved[2]), 0.0, delta=0.035)
                self.assertGreater(agree, 0.9)

    def test_max_points_bounds_the_work(self):
        seen = []
        real = scanmatch._closest

        def counting(moved, fixed, along, half):
            seen.append((len(moved), len(fixed)))
            return real(moved, fixed, along, half)
        scanmatch._closest = counting
        try:
            scanmatch.match(offset_scan((0, 0, 0), 2.5, 0.9, 0.0), offset_scan((0.01, 0, 0), 2.5, 0.9, 0.3), P, max_points=120)
        finally:
            scanmatch._closest = real
        self.assertLessEqual(max(max(s) for s in seen), 120)


class ScanShift(unittest.TestCase):
    def test_square_room_shift_and_turn(self):
        moved = (0.30, 0.0, math.radians(10))
        dx, dy, dyaw = scanmatch.scan_shift(room_scan((0, 0, 0)), room_scan(moved))
        self.assertAlmostEqual(dx, 0.30, delta=0.01)
        self.assertAlmostEqual(dy, 0.0, delta=0.01)
        self.assertAlmostEqual(math.degrees(dyaw), 10.0, delta=0.5)

    def test_sideways_and_agreement(self):
        dx, dy, dyaw, agree = scanmatch.match(room_scan((0, 0, 0)), room_scan((0.1, -0.2, math.radians(-5))))
        self.assertAlmostEqual(dx, 0.1, delta=0.01)
        self.assertAlmostEqual(dy, -0.2, delta=0.01)
        self.assertAlmostEqual(math.degrees(dyaw), -5.0, delta=0.5)
        self.assertGreater(agree, 0.9)

    def test_guess_makes_a_half_turn_work(self):
        turned = (0.0, 0.0, math.radians(175))
        _, _, dyaw, _ = scanmatch.match(room_scan((0, 0, 0)), room_scan(turned), guess=(0, 0, math.radians(165)))
        self.assertAlmostEqual(math.degrees(dyaw), 175.0, delta=0.5)

    def test_too_few_points(self):
        with self.assertRaises(ValueError):
            scanmatch.scan_shift([(0.0, 1.0)] * 5, room_scan((0, 0, 0)))


class ScanMatch(unittest.TestCase):
    def test_recovers_a_known_shift_in_a_room_with_corners(self):
        moved = (0.16, -0.12, math.radians(5))  # 0.20 m and 5 deg
        dx, dy, dyaw, share, slide = scanmatch.scan_match(world_scan((0, 0, 0), ROOM), world_scan(moved, ROOM, start_deg=0.4), p=P)
        self.assertAlmostEqual(dx, moved[0], delta=0.02)
        self.assertAlmostEqual(dy, moved[1], delta=0.02)
        self.assertAlmostEqual(math.degrees(dyaw), 5.0, delta=0.5)
        self.assertGreater(share, 0.9)
        self.assertLess(slide, 0.2)

    def test_reversed_heading_matches_the_same_place(self):
        moved = (0.10, 0.05, math.pi + math.radians(3))
        dx, dy, dyaw, share, _ = scanmatch.scan_match(world_scan((0, 0, 0), ROOM), world_scan(moved, ROOM, start_deg=0.4), (0.0, 0.0, math.pi), P)
        self.assertAlmostEqual(dx, moved[0], delta=0.02)
        self.assertAlmostEqual(dy, moved[1], delta=0.02)
        self.assertAlmostEqual(math.degrees(dyaw), 183.0, delta=0.5)
        self.assertGreater(share, 0.9)

    def test_a_plain_corridor_slides(self):
        # Review Focus 1: two parallel walls fix sideways and heading only
        moved = (0.3, 0.1, math.radians(3))
        _, dy, dyaw, share, slide = scanmatch.scan_match(world_scan((0, 0, 0), CORRIDOR), world_scan(moved, CORRIDOR, start_deg=0.4), p=P)
        self.assertAlmostEqual(dy, 0.1, delta=0.02)
        self.assertAlmostEqual(math.degrees(dyaw), 3.0, delta=0.5)
        self.assertGreater(share, 0.6)  # far wall returns stand too far apart to agree within 5 cm: 0.85
        self.assertGreater(slide, 0.8)

    def test_people_in_the_scan_lower_the_share(self):
        # Review Focus 5: the crowd hides the walls the stored scan saw
        moved = (0.05, 0.0, 0.0)
        *_, share, _ = scanmatch.scan_match(world_scan((0, 0, 0), ROOM), world_scan(moved, ROOM, crowd(0.05, 0.0), start_deg=0.4), p=P)
        self.assertLess(share, 0.6)


if __name__ == "__main__":
    unittest.main()
