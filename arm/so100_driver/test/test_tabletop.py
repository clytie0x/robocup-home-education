"""Camera-test maths on a synthetic cloud: table plane, calibration from two spots, object boxes."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from so100_driver.tabletop import CameraPose, calibrate_2pt, find_objects, fit_plane, fuse, rot_z  # noqa: E402

TABLE_Z = -0.075


def box_surface(cx, cy, yaw, size, step=0.004, cam=None):
    """Top and side faces of a box standing on the table (base frame); with cam, only the faces it sees."""
    d, w, h = size
    faces = [((0, 0, 1), [(a, b, h) for a in np.arange(-d / 2, d / 2 + 1e-9, step)
                          for b in np.arange(-w / 2, w / 2 + 1e-9, step)])]
    zs = np.arange(0.0, h, step)
    for sgn in (-1, 1):
        faces.append(((0, sgn, 0), [(a, sgn * w / 2, z) for z in zs for a in np.arange(-d / 2, d / 2 + 1e-9, step)]))
        faces.append(((sgn, 0, 0), [(sgn * d / 2, b, z) for z in zs for b in np.arange(-w / 2, w / 2 + 1e-9, step)]))
    r = rot_z(yaw)
    out = []
    for n, pts in faces:
        p = np.array(pts) @ r.T + (cx, cy, TABLE_Z)
        if cam is not None and (np.asarray(cam) - p.mean(axis=0)) @ (r @ np.array(n, float)) <= 0:
            continue
        out.append(p)
    return np.vstack(out)


def camera(tilt_deg=50.0, yaw=1.2, pos=(0.35, -0.30, 0.45)):
    """True cam -> base pose: optical z looks along `yaw`, tilted down by tilt_deg."""
    t = math.radians(tilt_deg)
    fwd = np.array([math.cos(t) * math.cos(yaw), math.cos(t) * math.sin(yaw), -math.sin(t)])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    down = np.cross(fwd, right)
    r = np.column_stack([right, down, fwd])        # columns = cam axes in base
    return r, np.array([pos[0], pos[1], TABLE_Z + pos[2]])


def scene(objects, noise=0.002, seed=1, cam=None):
    rng = np.random.default_rng(seed)
    xs, ys = np.meshgrid(np.arange(-0.4, 0.4, 0.008), np.arange(-0.6, 0.15, 0.008))
    table = np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, TABLE_Z)])
    pts = np.vstack([table] + [box_surface(*o, cam=cam) for o in objects])
    return pts + rng.normal(0, noise, pts.shape)


class TableTop(unittest.TestCase):
    def setUp(self):
        self.R, self.t = camera()
        self.objs = [(0.05, -0.25, 0.4, (0.04, 0.03, 0.06)), (-0.10, -0.20, -0.2, (0.06, 0.05, 0.12))]
        base = scene(self.objs)
        self.cam = (base - self.t) @ self.R          # base -> cam  (R orthonormal)

    def calibrated(self):
        n, d, _ = fit_plane(self.cam)
        guess = CameraPose(n, d)
        spots = [(0.05, -0.25), (-0.10, -0.20)]
        seen = [min(find_objects(guess.to_base(self.cam), 0.0), key=lambda o: o['size'][2]),
                max(find_objects(guess.to_base(self.cam), 0.0), key=lambda o: o['size'][2])]
        yaw, txy, err = calibrate_2pt((seen[0]['x'], seen[0]['y']), (seen[1]['x'], seen[1]['y']), *spots)
        self.assertLess(err, 0.005)
        return CameraPose(n, d, yaw, txy, TABLE_Z)

    def test_wall_is_not_the_table(self):
        wall = np.column_stack([np.linspace(-0.4, 0.4, 6000), np.full(6000, 0.2), np.random.default_rng(3).uniform(-0.07, 0.6, 6000)])
        cam = np.vstack([self.cam, (wall - self.t) @ self.R, (wall - self.t) @ self.R])
        n, d, _ = fit_plane(cam, up=(0, -1, 0))
        self.assertAlmostEqual(d, 0.45, delta=0.005)

    def test_plane_gives_height(self):
        n, d, inl = fit_plane(self.cam)
        self.assertAlmostEqual(d, 0.45, delta=0.004)
        self.assertGreater(inl.mean(), 0.8)

    def test_calibrated_pose_matches_truth(self):
        pose = self.calibrated()
        np.testing.assert_allclose(pose.R, self.R, atol=0.02)
        np.testing.assert_allclose(pose.t, self.t, atol=0.01)

    def test_objects_in_base(self):
        pose = self.calibrated()
        found = sorted(find_objects(pose.to_base(self.cam), TABLE_Z), key=lambda o: o['x'], reverse=True)
        self.assertEqual(len(found), 2)
        for o, (x, y, yaw, size) in zip(found, self.objs):
            self.assertAlmostEqual(o['x'], x, delta=0.006)
            self.assertAlmostEqual(o['y'], y, delta=0.006)
            self.assertAlmostEqual(o['size'][2], size[2], delta=0.008)
            got = sorted(o['size'][:2])
            for g, w in zip(got, sorted(size[:2])):
                self.assertAlmostEqual(g, w, delta=0.009)

    def test_fuse_median(self):
        pose = self.calibrated()
        frames = [find_objects(pose.to_base(self.cam), TABLE_Z) for _ in range(3)]
        f = fuse(frames)
        self.assertEqual(len(f), 2)
        self.assertEqual(f[0]['frames'], 3)

    def test_quaternion_roundtrip(self):
        pose = self.calibrated()
        x, y, z, w = pose.quaternion()
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(r, pose.R, atol=1e-6)


class LowCamera(unittest.TestCase):
    """Camera on a low box ~1 m in front of the arm, looking back at it: sees only the near faces."""

    def test_round_objects_completed(self):
        r, t = camera(tilt_deg=12, yaw=math.pi / 2, pos=(0.0, -1.05, 0.18))   # looks toward +y (the arm)
        objs = [(0.06, -0.25, 0.0, (0.05, 0.05, 0.10)), (-0.08, -0.30, 0.6, (0.04, 0.04, 0.12))]
        cam_pts = (scene(objs, cam=t) - t) @ r
        n, d, _ = fit_plane(cam_pts)
        self.assertAlmostEqual(d, 0.18, delta=0.004)
        pose = CameraPose(n, d, 0.0, (0.0, 0.0), 0.0)        # table frame; calibrate from the two objects
        seen = sorted(find_objects(pose.to_base(cam_pts), 0.0, camera=(0.0, 0.0, d)), key=lambda o: o['size'][2])
        yaw, txy, err = calibrate_2pt((seen[0]['x'], seen[0]['y']), (seen[1]['x'], seen[1]['y']),
                                      objs[0][:2], objs[1][:2])
        pose = CameraPose(n, d, yaw, txy, TABLE_Z)
        found = sorted(find_objects(pose.to_base(cam_pts), TABLE_Z, camera=pose.t), key=lambda o: o['size'][2])
        for o, (x, y, _, size) in zip(found, objs):
            self.assertTrue(o['completed'])
            self.assertAlmostEqual(o['x'], x, delta=0.008)
            self.assertAlmostEqual(o['y'], y, delta=0.008)
            # seen width = the object's extent across the line of sight (a turned square looks wider)
            self.assertGreater(o['size'][1], size[1] - 0.005)
            self.assertLess(o['size'][1], size[1] * math.sqrt(2) + 0.006)


    def test_known_box(self):
        """A long thin box (15 x 4 x 4 cm) seen low from the side: only the near long face and a bit of top."""
        r, t = camera(tilt_deg=14, yaw=math.pi / 2 + 0.4, pos=(-0.25, -0.95, 0.20))
        size = (0.15, 0.04, 0.04)
        objs = [(0.04, -0.26, 0.3, size)]
        cam_pts = (scene(objs, cam=t) - t) @ r
        n, d, _ = fit_plane(cam_pts, up=(0, -1, 0))
        true = CameraPose(n, d)
        true.R, true.t = r, t                           # use the true pose: this checks the box fit only
        found = find_objects(true.to_base(cam_pts), TABLE_Z, camera=t, known_size=size)
        self.assertEqual(len(found), 1)
        o = found[0]
        self.assertAlmostEqual(o['x'], 0.04, delta=0.008)
        self.assertAlmostEqual(o['y'], -0.26, delta=0.008)
        self.assertAlmostEqual(math.remainder(o['yaw'] - 0.3, math.pi), 0.0, delta=0.08)
        self.assertEqual(o['size'], size)


if __name__ == '__main__':
    unittest.main()
