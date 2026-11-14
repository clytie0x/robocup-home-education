import math
import time
import unittest
from pathlib import Path

import numpy as np

import camera
import floor_cable
import params

P = params.load()
HEIGHT = 0.105  # m, the height that fits the box seen at row ~215 at 1.20 m (params says 0.07)
W, H = 320, 240
FIXTURE = Path(__file__).parent / "fixtures" / "floor-no-cable.npz"


def wiggle(y):
    """The test cable: crosses the path about 0.85 m ahead, curving by +-4 cm."""
    return 0.85 + 0.04 * np.sin(8 * y)


def rays():
    """Per-pixel ray direction (ahead, left, up) in the base frame, as in depth_obstacles."""
    pitch = math.radians(P["camera_pitch_deg"])
    f = (W / 2) / math.tan(math.radians(P["camera_hfov_deg"]) / 2)
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rx, ry = (u - (W - 1) / 2) / f, (v - (H - 1) / 2) / f
    dx = math.cos(pitch) + ry * math.sin(pitch)
    dy = rx  # image is mirrored: image right is the robot's left
    dz = math.sin(pitch) - ry * math.cos(pitch)
    return dx, dy, dz


def floor_at(z, h=HEIGHT):
    """Base-frame x, y where each pixel's ray meets the plane z (NaN above the horizon)."""
    dx, dy, dz = rays()
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(dz < 0, (h - z) / -dz, np.nan)
    return P["camera_x"] + t * dx, P["camera_y"] + t * dy


def scene(cable=None, glare=False, blob=False, seed=0):
    """RGB of light blue glossy tiles (0.3 m, dark grout), a brown wall above, and extras."""
    rng = np.random.default_rng(seed)
    fx, fy = floor_at(0.0)
    floor = np.isfinite(fx)
    img = np.empty((H, W, 3))
    img[:] = (120, 80, 50)
    img[floor] = (110, 150, 165)
    gx, gy = np.abs((np.nan_to_num(fx) + 0.15) % 0.3 - 0.15), np.abs((np.nan_to_num(fy) + 0.15) % 0.3 - 0.15)
    img[floor & (np.minimum(gx, gy) < 0.004)] = (60, 85, 100)
    if glare:  # a ceiling light mirrored in the tiles
        img[floor & ((fx - 1.0) ** 2 + (fy + 0.1) ** 2 < 0.08 ** 2)] = (245, 250, 255)
    if blob:  # a dark cabinet mirrored in the tiles: wide, not thin
        img[floor & (fx > 0.8) & (fx < 1.2) & (fy > -0.3) & (fy < -0.05)] = (30, 35, 42)
    if cable is not None:
        y0, y1 = cable
        # a 7 mm round cable: the pixel ray passes within 3.5 mm of its axis, 3.5 mm up
        cx, cy = floor_at(0.0035)
        slope = 0.32 * np.cos(8 * np.nan_to_num(cy))
        across = np.abs(cx - wiggle(cy)) / np.sqrt(1 + slope ** 2)  # on the floor, square to the cable
        dx, dy, dz = rays()
        flat = np.abs(dx - slope * dy) / np.sqrt(1 + slope ** 2)
        near = across * np.abs(dz) / np.hypot(flat, dz) < 0.0035
        img[np.isfinite(cx) & near & (cy > y0) & (cy < y1)] = (15, 15, 18)
    img += rng.normal(0, 4, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


class CablePointsTest(unittest.TestCase):
    def test_a_curved_cable_is_found_where_it_lies(self):
        pts = floor_cable.cable_points(scene(cable=(-0.3, 0.3)), P, height=HEIGHT)
        self.assertGreater(len(pts), 30)
        err = np.abs(pts[:, 0] - wiggle(pts[:, 1]))
        self.assertLess(err.max(), 0.05, err.max())
        self.assertLess(pts[:, 1].min(), -0.2)
        self.assertGreater(pts[:, 1].max(), 0.2)

    def test_a_cable_on_the_left_appears_on_the_left(self):
        pts = floor_cable.cable_points(scene(cable=(0.1, 0.35)), P, height=HEIGHT)
        self.assertGreater(len(pts), 10)
        self.assertGreater(pts[:, 1].min(), 0.05)

    def test_a_frame_works_like_an_rgb_array(self):
        rgb = scene(cable=(-0.3, 0.3))
        frame = camera.Frame(0.0, W, H, rgb.tobytes(), np.zeros((H, W), np.uint16))
        np.testing.assert_array_equal(floor_cable.cable_points(frame, P, height=HEIGHT),
                                      floor_cable.cable_points(rgb, P, height=HEIGHT))

    def test_grout_lines_are_not_a_cable(self):
        self.assertEqual(floor_cable.cable_points(scene(), P, height=HEIGHT).shape, (0, 2))

    def test_a_bright_reflection_is_not_a_cable(self):
        self.assertEqual(floor_cable.cable_points(scene(glare=True), P, height=HEIGHT).shape, (0, 2))

    def test_a_dark_furniture_reflection_is_not_a_cable(self):
        self.assertEqual(floor_cable.cable_points(scene(blob=True), P, height=HEIGHT).shape, (0, 2))

    def test_height_defaults_to_params(self):
        rgb = scene(cable=(-0.3, 0.3))
        np.testing.assert_array_equal(floor_cable.cable_points(rgb, P),
                                      floor_cable.cable_points(rgb, P, height=P["camera_height"]))

    @unittest.skipUnless(FIXTURE.exists(), "real frame fixture missing")
    def test_the_real_floor_has_no_cable(self):
        saved = np.load(FIXTURE)
        rgb = np.full((H, W, 3), 128, np.uint8)
        rgb[int(saved["top"]):] = saved["rgb"]  # only the rows below the horizon were kept
        self.assertTrue(floor_cable.floor_visible(rgb, P, height=HEIGHT))
        self.assertEqual(floor_cable.cable_points(rgb, P, height=HEIGHT).shape, (0, 2))

    def test_fast_enough_for_the_pi(self):
        rgb = scene(cable=(-0.3, 0.3), glare=True)
        floor_cable.cable_points(rgb, P, height=HEIGHT)
        start = time.perf_counter()
        for _ in range(20):
            floor_cable.cable_points(rgb, P, height=HEIGHT)
        self.assertLess((time.perf_counter() - start) / 20, 0.01)


class FloorVisibleTest(unittest.TestCase):
    def test_a_tiled_floor_is_visible(self):
        self.assertTrue(floor_cable.floor_visible(scene(cable=(-0.3, 0.3)), P, height=HEIGHT))

    def test_a_black_image_is_blind(self):
        self.assertFalse(floor_cable.floor_visible(np.zeros((H, W, 3), np.uint8), P, height=HEIGHT))

    def test_a_blown_out_image_is_blind(self):
        self.assertFalse(floor_cable.floor_visible(np.full((H, W, 3), 255, np.uint8), P, height=HEIGHT))


if __name__ == "__main__":
    unittest.main()
