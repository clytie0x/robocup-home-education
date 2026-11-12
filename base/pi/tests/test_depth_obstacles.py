import math
import unittest

import numpy as np

import depth_obstacles
import params

P = dict(params.load(), camera_height=0.07, camera_pitch_deg=20.0, camera_x=0.205, robot_height=0.24)
W, H = 320, 240


def scene_depth(p, faces=(), wall_x=3.5):
    """Depth in mm of a flat floor, a far wall at base x = wall_x, and upright faces.

    A face is (x, y0, y1, z0, z1): a rectangle at base x, seen from the front.
    The camera is tilted up by camera_pitch_deg (the same pinhole convention as sensor_check).
    """
    pitch = math.radians(p["camera_pitch_deg"])
    f = (W / 2) / math.tan(math.radians(p["camera_hfov_deg"]) / 2)
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rx, ry = (u - (W - 1) / 2) / f, (v - (H - 1) / 2) / f
    # ray = forward + rx * right + ry * down, in (x forward, y left, z up); depth is the forward share, so t = depth
    dx = math.cos(pitch) + ry * math.sin(pitch)
    dy = rx  # image is mirrored: image right is the robot's left
    dz = math.sin(pitch) - ry * math.cos(pitch)
    cam = (p["camera_x"], p["camera_y"], p["camera_height"])
    best = np.full(rx.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -cam[2] / dz
        best = np.where((dz < 0) & (t > 0), t, best)
        for x, y0, y1, z0, z1 in [(wall_x, -9, 9, 0, 9)] + list(faces):
            t = (x - cam[0]) / dx
            y = cam[1] + t * dy
            z = cam[2] + t * dz
            hit = (dx > 0) & (t > 0) & (y >= y0) & (y <= y1) & (z >= z0) & (z <= z1)
            best = np.where(hit & (t < best), t, best)
    return np.where(np.isfinite(best), best * 1000, 0).astype(np.uint16)


class DepthObstacleTest(unittest.TestCase):
    def test_floor_is_not_an_obstacle_and_a_box_is(self):
        floor = depth_obstacles.points(scene_depth(P), P)
        self.assertEqual(floor.shape, (0, 2))  # the wall at 3.5 m is out of range, the floor is flat
        box = depth_obstacles.points(scene_depth(P, [(1.2, -0.15, 0.15, 0, 0.12)]), P)
        self.assertGreater(len(box), 5)
        self.assertTrue(np.all(np.abs(box[:, 0] - 1.2) < 0.05), box[:, 0])
        self.assertTrue(np.all(np.abs(box[:, 1]) < 0.2))

    def test_a_box_on_the_left_appears_on_the_left(self):
        depth = scene_depth(P, [(1.2, 0.2, 0.6, 0, 0.12)])
        box = depth_obstacles.points(depth, P)
        self.assertGreater(len(box), 5)
        self.assertTrue(np.all(np.abs(box[:, 1] - 0.4) < 0.25), box[:, 1])
        # the box's pixels sit on the image's right half
        near = (depth > 0) & (depth < 1100)  # the box face; floor starts further out
        self.assertGreater(np.nonzero(near)[1].min(), W // 2)

    def test_one_blind_third_makes_the_image_blind(self):
        depth = scene_depth(P)
        depth[:, : W // 3] = 0
        self.assertIsNone(depth_obstacles.points(depth, P))

    def test_level_camera_sees_the_box_too(self):
        level = dict(P, camera_pitch_deg=0.0)
        self.assertEqual(depth_obstacles.points(scene_depth(level), level).shape, (0, 2))
        box = depth_obstacles.points(scene_depth(level, [(1.2, -0.15, 0.15, 0, 0.12)]), level)
        self.assertGreater(len(box), 5)
        self.assertTrue(np.all(np.abs(box[:, 0] - 1.2) < 0.05))

    def test_things_above_the_robot_are_ignored(self):
        shelf = depth_obstacles.points(scene_depth(P, [(1.2, -0.3, 0.3, 0.5, 0.6)]), P)
        self.assertIsNotNone(shelf)
        self.assertEqual(len(shelf), 0)

    def test_a_blind_depth_image_is_none_not_empty(self):
        self.assertIsNone(depth_obstacles.points(np.zeros((240, 320), np.uint16), P))

    def test_unmeasured_camera_pose_is_an_error(self):
        with self.assertRaises(ValueError):
            depth_obstacles.points(scene_depth(P), dict(P, camera_height=None))


if __name__ == "__main__":
    unittest.main()
