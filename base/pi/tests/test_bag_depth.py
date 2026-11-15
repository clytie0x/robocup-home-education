"""bag_depth: the bag's top from synthetic Xtion depth images (a box on the floor, the camera's real pose differing
from params.json the way it is known to: height 0.105 against 0.07), and its yaw from LiDAR points."""
import math
import unittest

import numpy as np

import bag_depth
import depth_obstacles
import params

P = params.load()
SHAPE = (240, 320)


def render(box, cam_height=0.105, pitch_deg=12.0, p=P, shape=SHAPE, dark=False, noise=0.0, seed=0):
    """uint16 mm depth image of a floor (z = 0) and a box {"centre", "yaw", "size": (long, deep, high)} seen by a
    camera at camera_x, camera_y but at cam_height, tilted up by pitch_deg (the true pose), as the Xtion gives it.
    dark: the box gives no depth (a black, light-absorbing bag)."""
    right, down = depth_obstacles._rays(shape, p["camera_hfov_deg"])
    pitch = math.radians(pitch_deg)
    # the ray for unit depth along the optical axis (the inverse of bag_depth.cloud)
    d = np.stack([math.cos(pitch) + down * math.sin(pitch), right, math.sin(pitch) - down * math.cos(pitch)], -1)
    o = np.array([p["camera_x"], p["camera_y"], cam_height])
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(d[..., 2] < -1e-9, -o[2] / d[..., 2], np.inf)
    # the box: slabs in its own frame
    c, s = math.cos(box["yaw"]), math.sin(box["yaw"])
    rot = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])  # base -> box
    lo = np.array([-box["size"][0] / 2, -box["size"][1] / 2, 0.0])
    hi = np.array([box["size"][0] / 2, box["size"][1] / 2, box["size"][2]])
    ol = rot @ (o - np.array([box["centre"][0], box["centre"][1], 0.0]))
    dl = d @ rot.T
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - ol) / dl, (hi - ol) / dl
    tn = np.nanmax(np.minimum(t1, t2), axis=-1)
    tf = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = (tf >= tn) & (tf > 0)
    t_box = np.where(hit, tn, np.inf)
    t = np.minimum(t_floor, t_box)
    if noise:
        grow = np.minimum(t, 9) ** 2  # Xtion noise grows with depth squared
        t = np.where(np.isfinite(t), t + np.random.default_rng(seed).normal(0, noise, t.shape) * grow, t)
    mm = np.where(np.isfinite(t), np.round(t * 1000), 0)
    if dark:
        mm = np.where(t_box <= t_floor, 0, mm)
    return np.clip(mm, 0, 65535).astype(np.uint16)


BAG = {"centre": (1.0, 0.05), "yaw": math.radians(20), "size": (0.26, 0.11, 0.30)}


class Top(unittest.TestCase):
    def test_params_pose_is_the_one_known_to_be_wrong(self):
        self.assertEqual((P["camera_height"], P["camera_pitch_deg"]), (0.07, 12.3))

    def test_the_top_comes_out_within_1_5_cm_although_the_camera_height_is_wrong(self):
        for centre, high in (((1.0, 0.05), 0.30), ((0.7, -0.05), 0.26), ((1.4, 0.15), 0.36)):
            box = dict(BAG, centre=centre, size=(0.26, 0.11, high))
            got = bag_depth.measure_top(render(box), centre, P)
            self.assertTrue(got["ok"], got)
            self.assertLess(abs(got["top"] - high), 0.015, (centre, got))
            self.assertGreater(got["conf"], 0.5)
            self.assertGreaterEqual(got["valid_share"], bag_depth.VALID_FULL)  # no wall behind: the sky part is empty

    def test_the_absolute_height_would_be_3_5_cm_off(self):
        # what the local floor cancels: the floor itself comes out at 0.07 - 0.105 = -0.035 m with params' pose
        pts, _ = bag_depth.cloud(render(BAG), P)
        r = np.hypot(pts[:, 0] - 1.0, pts[:, 1] - 0.05)
        floor_z = np.median(pts[(r > 0.2) & (r < 0.4), 2])
        self.assertAlmostEqual(floor_z, -0.035, delta=0.01)

    def test_a_pitch_error_cancels_too(self):
        got = bag_depth.measure_top(render(BAG, pitch_deg=10.0), BAG["centre"], P)  # 2.3 deg off params
        self.assertTrue(got["ok"], got)
        self.assertLess(abs(got["top"] - 0.30), 0.015, got)

    def test_with_depth_noise(self):
        got = bag_depth.measure_top(render(BAG, noise=0.003), BAG["centre"], P)  # 3 mm at 1 m
        self.assertTrue(got["ok"], got)
        self.assertLess(abs(got["top"] - 0.30), 0.015, got)

    def test_a_dark_bag_gives_no_depth_and_is_not_used(self):
        got = bag_depth.measure_top(render(BAG, dark=True), BAG["centre"], P)
        self.assertFalse(got["ok"])
        self.assertIsNone(got["top"])
        self.assertLess(got["valid_share"], 0.8)
        self.assertIn("no depth on the bag", got["why"])

    def test_a_bag_too_tall_for_the_image_is_not_trusted(self):
        box = dict(BAG, centre=(0.6, 0.05), size=(0.26, 0.11, 0.55))
        got = bag_depth.measure_top(render(box), (0.6, 0.05), P)
        self.assertFalse(got["ok"], got)
        self.assertEqual(got["conf"], 0.0)

    def test_no_frame_or_bare_floor(self):
        self.assertFalse(bag_depth.measure_top(None, (1.0, 0.0), P)["ok"])
        bare = dict(BAG, size=(0.26, 0.11, 0.0))
        got = bag_depth.measure_top(render(bare), BAG["centre"], P)
        self.assertFalse(got["ok"])
        self.assertIsNone(got["top"])

    def test_project_is_the_inverse_of_cloud(self):
        depth = render(BAG)
        pts, rows = bag_depth.cloud(depth, P)
        uv = bag_depth.project(pts[::97], P, SHAPE)
        self.assertTrue(np.allclose(uv[:, 1], rows[::97], atol=1e-6))
        v, u = np.nonzero((depth >= 400) & (depth <= 2000))
        self.assertTrue(np.allclose(uv[:, 0], u[::97], atol=1e-6))


def side(a, b, n):
    return np.linspace(a, b, n)


class Yaw(unittest.TestCase):
    def test_one_side_gives_its_direction(self):
        for deg in (0, 25, -40, 89):
            a = math.radians(deg)
            pts = side((1.0, 0.0), (1.0 + 0.25 * math.cos(a), 0.25 * math.sin(a)), 15)
            yaw, conf, _ = bag_depth.cluster_yaw(pts)
            self.assertAlmostEqual(math.degrees(bag_depth.wrap_line(yaw - a)), 0.0, delta=0.5)
            self.assertGreater(conf, 0.9)

    def test_of_two_sides_the_longer_one(self):
        a = math.radians(30)
        corner = np.array([1.0, 0.0])
        long_end = corner + 0.26 * np.array([math.cos(a), math.sin(a)])
        short_end = corner + 0.11 * np.array([-math.sin(a), math.cos(a)])
        pts = np.vstack([side(short_end, corner, 6), side(corner, long_end, 14)[1:]])
        yaw, conf, why = bag_depth.cluster_yaw(pts)
        self.assertAlmostEqual(math.degrees(bag_depth.wrap_line(yaw - a)), 0.0, delta=1.0)
        self.assertIn("two sides", why)

    def test_a_short_or_tiny_cluster_has_low_confidence(self):
        yaw, conf, _ = bag_depth.cluster_yaw(side((1.0, 0.0), (1.0, 0.05), 5))
        self.assertLess(conf, 0.3)
        self.assertEqual(bag_depth.cluster_yaw([(1.0, 0.0), (1.0, 0.01)])[0], None)

    def test_one_short_side_may_be_the_narrow_end_so_no_yaw(self):
        """Seen end on, the LiDAR gets only the bag's narrow side (0.12 m): its direction is the bag's SHORT axis,
        90 deg off. A single side shorter than ONE_SIDE_MIN gives no yaw (the arm keeps its default)."""
        yaw, conf, why = bag_depth.cluster_yaw(side((1.0, -0.06), (1.0, 0.06), 9))
        self.assertIsNone(yaw)
        self.assertEqual(conf, 0.0)
        self.assertIn("narrow end", why)

    def test_two_sides_of_about_the_same_length_give_no_yaw(self):
        a = math.radians(30)
        corner = np.array([1.0, 0.0])
        one = corner + 0.20 * np.array([math.cos(a), math.sin(a)])
        other = corner + 0.19 * np.array([-math.sin(a), math.cos(a)])
        pts = np.vstack([side(other, corner, 12), side(corner, one, 12)[1:]])
        yaw, conf, why = bag_depth.cluster_yaw(pts)
        self.assertIsNone(yaw)
        self.assertIn("same length", why)

    def test_the_yaw_is_a_line_in_minus_90_to_90_deg(self):
        for a in (-3.0, -1.6, 0.0, 1.5708, 2.0, 3.1):
            w = bag_depth.wrap_line(a)
            self.assertTrue(-math.pi / 2 <= w < math.pi / 2)
            self.assertAlmostEqual(math.sin(2 * (w - a)), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
