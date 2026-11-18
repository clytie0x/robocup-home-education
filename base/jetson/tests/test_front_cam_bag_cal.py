"""tools/front_cam_bag_cal.py: bag corners on the floor -> the front camera (synthetic frames, no camera)."""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

import front_cam_bag_cal as bc  # noqa: E402
import front_camera as fc  # noqa: E402

TRUE = fc.FrontCamera(fx=470.0, x=0.39, y=0.01, z=0.95, yaw=math.radians(1.5), pitch=math.radians(45.0),
                      roll=math.radians(-2.0), calibrated=True)
PLACES = [(20, 0, 30), (35, 0, 30), (50, 0, 30), (30, 15, 30), (30, -15, 30), (45, 12, 30), (45, -12, 30)]


def frames_from(cam, places, noise=0.3, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for i, (front, left, width) in enumerate(places):
        pl, pr = bc.floor_corners(front / 100, left / 100, width / 100)
        a, b = cam._project(np.array([pl, pr]))
        if cam.mirror:
            a, b = b, a
        out.append({"name": "b%d" % i, "points": (pl, pr), "left": a + rng.normal(0, noise, 2),
                    "right": b + rng.normal(0, noise, 2)})
    return out


class BagCalTest(unittest.TestCase):
    def test_floor_corners_are_in_the_turning_centre_frame(self):
        left, right = bc.floor_corners(0.24, 0.0, 0.30)
        np.testing.assert_allclose(left, [0.24 + 0.20 + 0.175, 0.15, 0.0])
        np.testing.assert_allclose(right, [0.24 + 0.20 + 0.175, -0.15, 0.0])

    def test_fit_recovers_the_camera_from_seven_bag_places(self):
        out = bc.fit_both_mirrors(frames_from(TRUE, PLACES), fc.FrontCamera(x=0.375))
        cam = out["camera"]
        self.assertFalse(cam.mirror)
        self.assertLess(out["rms_px"], 1.0)
        self.assertAlmostEqual(cam.fx, TRUE.fx, delta=0.03 * TRUE.fx)
        self.assertAlmostEqual(cam.z, TRUE.z, delta=0.03)
        for k in ("yaw", "pitch", "roll"):
            self.assertAlmostEqual(math.degrees(getattr(cam, k)), math.degrees(getattr(TRUE, k)), delta=1.0)

    def test_a_mirrored_image_is_found(self):
        true = TRUE.copy(mirror=True)
        out = bc.fit_both_mirrors(frames_from(true, PLACES), fc.FrontCamera(x=0.375))
        self.assertTrue(out["camera"].mirror)
        self.assertLess(out["rms_px"], 1.0)

    def test_too_few_places_are_refused(self):
        with self.assertRaises(ValueError):
            bc.fit(frames_from(TRUE, PLACES[:1]), fc.FrontCamera(x=0.375))

    def test_one_distance_only_leaves_something_undetermined(self):
        # all at the same distance and side: the floor homography is not pinned, nothing may be written
        out = bc.fit_both_mirrors(frames_from(TRUE, [(30, 0, 30)] * 3), fc.FrontCamera(x=0.375))
        self.assertTrue(out["undetermined"])
        self.assertFalse(out["camera"].calibrated)

    def test_corners_of_a_synthetic_bag_blob(self):
        img = np.full((480, 640, 3), (200, 200, 120), np.uint8)   # pale floor (BGR)
        img[250:400, 300:460] = (60, 30, 220)                     # red face
        det = bc.detect(img)
        self.assertEqual(det["left"], [300.0, 399.0])
        self.assertEqual(det["right"], [459.0, 399.0])

    def test_a_bag_cut_by_the_image_bottom_is_not_used(self):
        img = np.full((480, 640, 3), (200, 200, 120), np.uint8)
        img[300:480, 300:460] = (60, 30, 220)
        self.assertIsNone(bc.detect(img))


class DiscTest(unittest.TestCase):
    LAYOUT = "25:15,-15;40:20,-20;60:25,-25;90:0"

    def frame(self, cam, layout):
        import cv2
        img = np.full((480, 640, 3), (220, 215, 170), np.uint8)
        for front, lefts in bc.parse_layout(layout):
            for left in lefts:
                p = np.array([front / 100 + 0.375, left / 100, 0.0])
                u, v = cam._project(p.reshape(1, 3))[0]
                cv2.circle(img, (int(round(u)), int(round(v))), 7, (60, 40, 200), -1)
        return img

    def test_discs_are_found_matched_and_fit_the_camera(self):
        cam = fc.FrontCamera(fx=480.0, x=0.33, y=0.0, z=0.90, pitch=math.radians(55), calibrated=True)
        discs = bc.detect_discs(self.frame(cam, self.LAYOUT))
        self.assertEqual(len(discs), 7)
        matched = bc.match_discs(discs, bc.parse_layout(self.LAYOUT))
        self.assertIsNotNone(matched)
        far = [d for d in matched if d["front_cm"] == 90]
        self.assertEqual([d["left_cm"] for d in far], [0.0])
        near = [d for d in matched if d["front_cm"] == 25]
        self.assertLess(near[0]["uv"][0], near[1]["uv"][0])   # robot-left (+15) is image-left
        self.assertEqual([d["left_cm"] for d in near], [15.0, -15.0])
        frames = bc.frames_of({"frames": [{"name": "d", "discs": matched}]})
        out = bc.fit_best(frames, fc.FrontCamera(x=0.33, z=0.90), ("fx", "yaw", "pitch", "roll"))
        self.assertLess(out["rms_px"], 1.0)
        self.assertAlmostEqual(math.degrees(out["camera"].pitch), 55.0, delta=0.5)
        self.assertAlmostEqual(out["camera"].fx, 480.0, delta=10.0)

    def test_a_missing_disc_is_not_matched(self):
        cam = fc.FrontCamera(fx=480.0, x=0.33, z=0.90, pitch=math.radians(55), calibrated=True)
        discs = bc.detect_discs(self.frame(cam, self.LAYOUT))[:-1]
        self.assertIsNone(bc.match_discs(discs, bc.parse_layout(self.LAYOUT)))


if __name__ == "__main__":
    unittest.main()
