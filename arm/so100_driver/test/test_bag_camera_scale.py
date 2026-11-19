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


if __name__ == "__main__":
    unittest.main()
