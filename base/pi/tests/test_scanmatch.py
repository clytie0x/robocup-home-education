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


if __name__ == "__main__":
    unittest.main()
