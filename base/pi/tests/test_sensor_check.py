import math
import sys
import unittest
from pathlib import Path

import numpy as np

import params

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import sensor_check  # noqa: E402

P = params.load()
W, H = 640, 480


def rays(hfov_deg, w=W, h=H):
    """Pinhole ray (x, y, 1) per pixel, camera frame: x right, y down, z forward."""
    f = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    return (u - (w - 1) / 2) / f, (v - (h - 1) / 2) / f, f


def scene_depth(height, pitch_up_deg, wall=None, max_range=5.0, w=W, h=H):
    """Depth in mm of a flat floor `height` below the lens, camera tilted up by pitch_up_deg."""
    x, y, _ = rays(P["camera_hfov_deg"], w, h)
    p = math.radians(pitch_up_deg)
    up = -y * math.cos(p) + math.sin(p)  # ray component along world up
    fwd = y * math.sin(p) + math.cos(p)  # ray component along world forward
    with np.errstate(divide="ignore"):
        t = np.where(up < 0, height / -up, np.inf)
        if wall is not None:
            t = np.minimum(t, np.where(fwd > 0, wall / fwd, np.inf))
    t[t > max_range] = 0  # out of sensor range reads as 0, like the Xtion
    return np.round(t * 1000).astype(np.uint16)


class FitFloor(unittest.TestCase):
    def test_low_camera_pitched_down(self):
        depth = scene_depth(0.20, -8.0)
        # Pitched down the floor is near, so the 1.6 m default would hide it.
        height, pitch, share = sensor_check.fit_floor(depth, P["camera_hfov_deg"], min_range=0.3)
        self.assertAlmostEqual(height, 0.20, delta=0.01)
        self.assertAlmostEqual(pitch, -8.0, delta=0.5)
        self.assertGreater(share, 0.9)

    def test_floor_camera_tilted_up_sees_only_far_floor_and_a_wall(self):
        depth = scene_depth(0.07, 20.0, wall=3.0)
        height, pitch, share = sensor_check.fit_floor(depth, P["camera_hfov_deg"])
        self.assertAlmostEqual(height, 0.07, delta=0.01)
        self.assertAlmostEqual(pitch, 20.0, delta=0.5)
        self.assertLess(share, 0.5)  # the wall is most of the view, yet the floor is the answer

    def test_tilted_up_at_320x240_passes_the_scaled_inlier_minimum(self):
        depth = scene_depth(0.07, 20.0, wall=3.0, w=320, h=240)
        height, pitch, _ = sensor_check.fit_floor(depth, P["camera_hfov_deg"])
        self.assertAlmostEqual(height, 0.07, delta=0.01)
        self.assertAlmostEqual(pitch, 20.0, delta=0.5)

    def test_noisy_wall_without_floor_raises(self):
        # A wall at 3 m, 15 mm depth noise, no floor: used to give a plausible-looking wrong fit.
        for pitch in (0.0, 20.0):
            for seed in range(3):
                depth = scene_depth(100.0, pitch, wall=3.0).astype(float)
                noise = np.random.default_rng(seed).normal(0, 15, depth.shape)
                depth = np.where(depth > 0, depth + noise, 0).astype(np.uint16)
                with self.assertRaises(ValueError):
                    sensor_check.fit_floor(depth, P["camera_hfov_deg"])

    def test_no_floor_raises(self):
        with self.assertRaises(ValueError):
            sensor_check.fit_floor(np.zeros((H, W), np.uint16), P["camera_hfov_deg"])


REAL_DEPTH = Path("C:/Users/teera/AppData/Local/Temp/claude/dcd73c56e0776980/18334c49-fd54-4647-a087-ce61289ceb23/scratchpad/verify/depth10.npy")


class Pitch(unittest.TestCase):
    def test_vertical_wall_gives_the_pitch(self):
        depth = scene_depth(100.0, 17.0, wall=3.0, w=320, h=240)  # floor 100 m down: wall only
        pitch, share, yaw = sensor_check.fit_wall(depth, P["camera_hfov_deg"])
        self.assertAlmostEqual(pitch, 17.0, delta=0.5)
        self.assertAlmostEqual(yaw, 0.0, delta=0.5)

    def test_floor_alone_is_not_a_wall(self):
        with self.assertRaises(ValueError):
            sensor_check.fit_wall(scene_depth(0.07, 17.0, w=320, h=240), P["camera_hfov_deg"])

    @unittest.skipUnless(REAL_DEPTH.exists(), "real Xtion sample is only in the laptop session scratchpad")
    def test_real_frames_from_the_robot(self):
        frames = np.load(REAL_DEPTH)
        pitches = [sensor_check.fit_wall(f, P["camera_hfov_deg"])[0] for f in frames]
        # Controller's box fit gave 16.5-17.5; this picks a different plane, see the task-3 report.
        self.assertTrue(15.0 <= float(np.median(pitches)) <= 18.0, pitches)

    def test_object_top_edge_gives_the_pitch(self):
        camera_height, pitch_up, camera_dx, ahead = 0.07, 17.0, 0.03, 1.5
        x, y, _ = rays(P["camera_hfov_deg"], 320, 240)
        p = math.radians(pitch_up)
        up = -y * math.cos(p) + math.sin(p)
        fwd = y * math.sin(p) + math.cos(p)
        distance = ahead - camera_dx  # camera to the object's front face
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(fwd > 0, distance / fwd, np.inf)
        hit = (camera_height + t * up >= 0) & (camera_height + t * up <= 0.25) & (np.abs(t * x) <= 0.05)
        depth = np.where(hit, np.round(t * 1000), 0).astype(np.uint16)
        pitch = sensor_check.object_pitch(depth, P["camera_hfov_deg"], 0.25, ahead, camera_height, camera_dx)
        self.assertAlmostEqual(pitch, 17.0, delta=1.0)


class Yaw(unittest.TestCase):
    def test_scan_bearing_picks_the_cluster_nearest_1_5_m(self):
        bottle = [(2.0 + d, 1.5) for d in (-1.0, -0.5, 0.0, 0.5, 1.0)]
        clutter = [(-15.0 + d, 1.05) for d in (0.0, 0.5, 1.0)]
        wall = [(d, 3.0) for d in range(-20, 21)]
        bearing, metres = sensor_check.scan_bearing(bottle + clutter + wall)
        self.assertAlmostEqual(bearing, 2.0, delta=0.1)
        self.assertAlmostEqual(metres, 1.5, delta=0.01)

    def test_scan_bearing_none_without_a_target(self):
        self.assertIsNone(sensor_check.scan_bearing([(0.0, 3.0), (90.0, 1.5)]))

    def test_suggested_front_cancels_the_scan_error(self):
        # lidar.py reports base_deg = front - laser_deg, so a +2 deg reading needs front - 2.
        self.assertAlmostEqual(sensor_check.suggested_front(178.7, 2.0), 176.7)

    def bottle_image(self, offset_left, camera_y=0.045, camera_dx=0.03, ahead=1.5):
        """Depth image of a bottle `ahead` m in front of the LiDAR and offset_left to its left."""
        theta_cam = math.atan2(offset_left - camera_y, ahead - camera_dx)
        x, y, f = rays(P["camera_hfov_deg"])
        u0 = round((W - 1) / 2 + f * math.tan(theta_cam))  # mirrored image: robot left is image right
        r = math.hypot(ahead, offset_left) - camera_dx
        depth = np.zeros((H, W), np.uint16)
        depth[470:, :] = int(r * 1000)  # floor rows at the same range across the image
        depth[380:470, u0 - 10 : u0 + 11] = int(r * 1000)
        return depth, math.hypot(ahead, offset_left)

    def test_image_bearing_corrects_for_camera_y(self):
        depth, scan_range = self.bottle_image(0.0)
        bearing = sensor_check.image_bearing(depth, P["camera_hfov_deg"], scan_range, 0.045, 0.03)
        self.assertAlmostEqual(bearing, 0.0, delta=0.2)

    def test_bottle_on_the_left_reads_left_with_the_mirrored_image(self):
        depth, scan_range = self.bottle_image(0.3)
        bearing = sensor_check.image_bearing(depth, P["camera_hfov_deg"], scan_range, 0.045, 0.03)
        self.assertAlmostEqual(bearing, math.degrees(math.atan2(0.3, 1.5)), delta=0.5)
        self.assertEqual(sensor_check.side(bearing), "left")

    def test_suggested_front_with_an_off_centre_bottle(self):
        self.assertAlmostEqual(sensor_check.suggested_front(178.7, 13.0, true_deg=11.3), 177.0)

    def test_image_bearing_none_when_nothing_at_range(self):
        depth = np.full((H, W), 4000, np.uint16)
        self.assertIsNone(sensor_check.image_bearing(depth, P["camera_hfov_deg"], 1.5, 0.045, 0.03))


if __name__ == "__main__":
    unittest.main()
