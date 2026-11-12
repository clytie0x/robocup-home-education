import math
import unittest

import geometry
import params
import shapes

P = params.load()


def wall_scan(pose, wall_x=2.0):
    """LiDAR scan (degrees, metres) of a straight wall at odometry x = wall_x, from the given pose."""
    scan = []
    for i in range(-20, 21):
        base = geometry.odom_to_base((wall_x, i * 0.05), pose)
        x, y = base[0] - P["lidar_x"], base[1]
        deg, metres = geometry.polar((x, y))
        scan.append((deg, metres))
    return scan


class ObstacleMemoryTest(unittest.TestCase):
    def test_depth_obstacle_is_remembered_ten_seconds(self):
        memory = shapes.ObstacleMemory(P)
        pose = (0.0, 0.0, 0.0)
        memory.update(0.0, pose, [], [(1.0, 0.0)], [])
        self.assertEqual(len(memory.update(9.9, pose, [], None, [])), 1)
        circle = memory.update(9.9, pose, [], None, [])[0]
        self.assertAlmostEqual((circle.x, circle.y, circle.r), (1.0, 0.0, 0.05))
        self.assertEqual(memory.update(10.1, pose, [], None, []), [])

    def test_lidar_points_near_a_person_are_left_out(self):
        memory = shapes.ObstacleMemory(P)
        pose = (0.0, 0.0, 0.0)
        scan = wall_scan(pose)
        everything = memory.update(0.0, pose, scan, None, [])
        self.assertGreater(len(everything), 5)
        person = (2.0, 0.0)
        left = shapes.ObstacleMemory(P).update(0.0, pose, scan, None, [person])
        self.assertLess(len(left), len(everything))
        self.assertTrue(all(math.dist((c.x, c.y), person) > 0.45 for c in left))
        # depth points near a person are left out as well
        self.assertEqual(shapes.ObstacleMemory(P).update(0.0, pose, [], [(2.1, 0.0)], [person]), [])

    def test_a_person_passing_does_not_erase_a_remembered_obstacle(self):
        memory = shapes.ObstacleMemory(P)
        pose = (0.0, 0.0, 0.0)
        memory.update(0.0, pose, [], [(1.0, 0.0)], [])
        self.assertEqual(memory.update(1.0, pose, [], None, [(1.2, 0.0)]), [])
        self.assertEqual(len(memory.update(2.0, pose, [], None, [])), 1)

    def test_depth_points_follow_a_moved_pose(self):
        circle = shapes.ObstacleMemory(P).update(0.0, (1.0, 2.0, math.pi / 2), [], [(1.0, 0.5)], [])[0]
        self.assertAlmostEqual((circle.x, circle.y), (0.5, 3.0))

    def test_lidar_circles_last_for_one_scan_only(self):
        memory = shapes.ObstacleMemory(P)
        pose = (0.0, 0.0, 0.0)
        memory.update(0.0, pose, wall_scan(pose), None, [])
        self.assertEqual(memory.update(0.1, pose, [], None, []), [])

    def test_shapes_are_in_the_odometry_frame(self):
        for pose in [(0.0, 0.0, 0.0), (0.5, 0.3, 0.2)]:
            circles = shapes.ObstacleMemory(P).update(0.0, pose, wall_scan(pose), None, [])
            self.assertGreater(len(circles), 5)
            for c in circles:
                self.assertLess(abs(c.x - 2.0), 0.05)
                self.assertEqual(c.r, 0.05)


if __name__ == "__main__":
    unittest.main()
