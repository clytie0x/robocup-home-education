"""A front frame that is not the calibration size is scaled only when both axes match."""
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
try:
    import cv2  # noqa: F401, E402  (real OpenCV when the vision tests share this process)
except ImportError:
    sys.modules.setdefault("cv2", types.ModuleType("cv2"))

import numpy as np  # noqa: E402

from so100_driver.bag_camera import FrontCamera  # noqa: E402

CAL = {
    "calibrated": True, "fx": 600.0, "cx": 319.5, "cy": 239.5,
    "x": 0.2, "y": 0.0, "z": 0.25, "width": 640, "height": 480,
}


class Scale(unittest.TestCase):
    def test_the_calibration_size_is_the_same_camera(self):
        cam = FrontCamera(CAL)
        self.assertIs(cam.for_image(np.zeros((480, 640, 3), np.uint8)), cam)

    def test_a_uniform_scale_moves_the_optical_centre_with_the_pixels(self):
        cam = FrontCamera(CAL)
        scaled = cam.for_image(np.zeros((960, 1280, 3), np.uint8))
        self.assertAlmostEqual(scaled.fx, 1200.0)
        self.assertAlmostEqual(scaled.cx, (319.5 + 0.5) * 2 - 0.5)
        self.assertAlmostEqual(scaled.cy, (239.5 + 0.5) * 2 - 0.5)
        self.assertEqual((scaled.width, scaled.height), (1280, 960))

    def test_a_16_by_9_frame_is_refused(self):
        cam = FrontCamera(CAL)
        self.assertIsNone(cam.for_image(np.zeros((720, 1280, 3), np.uint8)))


# AX1080P file of 2026-11-18, without dist_model. The new lens is the 2026-11-20 candidate.
OLD = {
    "calibrated": True, "fx": 805.6566516199323, "cx": 319.5, "cy": 239.5,
    "x": 0.33, "y": 0.0, "z": 0.89, "width": 640, "height": 480,
    "yaw_deg": 1.4024275508991533, "pitch_deg": 61.36732280365156, "roll_deg": 0.0, "mirror": False,
}
FISH_K = (-0.12000174022745945, 0.05392880504627527, 0.0, 0.0)
FISH = dict(OLD, fx=522.7004622679294, cx=296.70882730620366, cy=241.09751641118947, z=0.7933,
            yaw_deg=0.7606571023798528, pitch_deg=46.954749902416516, roll_deg=0.9615972633301483,
            dist_model="fisheye", dist=list(FISH_K))


class Fisheye(unittest.TestCase):
    def test_an_unknown_lens_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            FrontCamera(dict(FISH, dist_model="brown"))
        self.assertIn("unknown dist_model", str(caught.exception))
        self.assertIn("brown", str(caught.exception))

    def test_an_old_pinhole_dict_matches_the_plain_projection(self):
        cam = FrontCamera(OLD)
        self.assertEqual(cam.dist_model, "pinhole")
        self.assertEqual(cam.dist, (0.0, 0.0, 0.0, 0.0))
        for p in ((0.8, -0.2, 0.0), (1.2, 0.15, 0.05), (1.6, 0.0, 0.3)):
            pc = (np.asarray(p, float) - cam.pos) @ cam.rot
            want = (cam.cx + cam.fx * pc[0] / pc[2], cam.cy + cam.fx * pc[1] / pc[2])
            self.assertEqual(cam.project(p), want)
        for u, v in ((10.0, 10.0), (320.0, 240.0), (600.0, 400.0)):
            d = np.array([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fx, 1.0])
            d = cam.rot @ d
            d = d / np.linalg.norm(d)
            self.assertTrue(np.array_equal(cam.ray(u, v), d))
        mirrored = FrontCamera(dict(OLD, mirror=True))
        p = (1.2, 0.15, 0.05)
        pc = (np.asarray(p, float) - mirrored.pos) @ mirrored.rot
        u = mirrored.fx * pc[0] / pc[2]
        self.assertEqual(mirrored.project(p), (mirrored.cx - u, mirrored.cy + mirrored.fx * pc[1] / pc[2]))

    def test_for_image_keeps_the_lens(self):
        cam = FrontCamera(FISH)
        scaled = cam.for_image(np.zeros((960, 1280, 3), np.uint8))
        self.assertEqual(scaled.dist_model, "fisheye")
        self.assertEqual(scaled.dist, FISH_K)
        u, v = 120.0, 360.0
        uu, vv = (u + 0.5) * 2 - 0.5, (v + 0.5) * 2 - 0.5
        self.assertTrue(np.allclose(cam.ray(u, v), scaled.ray(uu, vv), atol=1e-12))

    def test_pixels_round_trip_across_the_frame(self):
        for mirror in (False, True):
            cam = FrontCamera(dict(FISH, mirror=mirror))
            err = 0.0
            for v in np.linspace(0, 479, 25):
                for u in np.linspace(0, 639, 33):
                    uu, vv = cam.project(cam.pos + cam.ray(float(u), float(v)))
                    err = max(err, abs(uu - u), abs(vv - v))
            self.assertLess(err, 1e-6)

    def test_the_projection_matches_opencv_fisheye(self):
        import cv2
        cam = FrontCamera(FISH)
        pts = np.array([[x, y, z] for x in np.linspace(0.45, 2.2, 7) for y in np.linspace(-0.8, 0.8, 7)
                        for z in (0.0, 0.25)])
        got = np.array([cam.project(p) for p in pts])
        rvec, _ = cv2.Rodrigues(cam.rot.T)
        tvec = -cam.rot.T @ cam.pos
        k = np.array([[cam.fx, 0.0, cam.cx], [0.0, cam.fx, cam.cy], [0.0, 0.0, 1.0]])
        cvp = cv2.fisheye.projectPoints(pts.reshape(-1, 1, 3), rvec, tvec.reshape(3, 1), k,
                                        np.array(cam.dist, float))[0].reshape(-1, 2)
        self.assertLess(float(np.abs(got - cvp).max()), 1e-9)
        pixels = np.array([[0.0, 0.0], [639.0, 479.0], [320.0, 240.0], [100.0, 400.0], [500.0, 50.0]])
        und = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), k, np.array(cam.dist, float)).reshape(-1, 2)
        for (u, v), (x, y) in zip(pixels, und):
            d = cam.rot.T @ cam.ray(float(u), float(v))
            self.assertAlmostEqual(d[0] / d[2], x, delta=1e-9)
            self.assertAlmostEqual(d[1] / d[2], y, delta=1e-9)

    def test_a_harsh_lens_still_gives_a_finite_ray(self):
        cam = FrontCamera(dict(FISH, dist=[-0.9, 0.8, -0.5, 0.2]))
        for u, v in ((0.0, 0.0), (639.0, 479.0), (320.0, 240.0)):
            d = cam.ray(u, v)
            self.assertTrue(np.isfinite(d).all())
            self.assertAlmostEqual(float(np.linalg.norm(d)), 1.0, places=12)


if __name__ == "__main__":
    unittest.main()
