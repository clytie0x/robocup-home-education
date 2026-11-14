import time
import unittest

import numpy as np

import camera


def fake_capture(pairs):
    """A capture function that hands out `pairs` frames, then waits to be stopped."""

    def capture(seconds, on_pair, should_stop):
        for n in range(pairs):
            color = {"width": 4, "height": 3, "rgb": bytes([n]) * 36}
            depth = {"width": 4, "height": 3, "mm": np.full((3, 4), 1000 + n, np.uint16)}
            on_pair(color, depth)
        while not should_stop():
            time.sleep(0.01)

    return capture


def wait_for(get, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        value = get()
        if value is not None:
            return value
        time.sleep(0.01)
    return None


class CameraTests(unittest.TestCase):
    def test_frame_has_the_depth_shape_and_dtype_and_comes_once(self):
        cam = camera.Camera(capture=fake_capture(1))
        cam.start()
        frame = wait_for(cam.latest)
        cam.stop()
        self.assertIsNone(cam.error)
        self.assertEqual((frame.width, frame.height), (4, 3))
        self.assertEqual(frame.depth.shape, (3, 4))
        self.assertEqual(frame.depth.dtype, np.uint16)
        self.assertEqual(len(frame.rgb), 36)
        self.assertIsNone(cam.latest())  # the same frame is never handed out twice

    def test_capture_error_is_kept_in_error(self):
        def broken(seconds, on_pair, should_stop):
            raise RuntimeError("camera_open_failed")

        cam = camera.Camera(capture=broken)
        cam.start()
        cam.stop()
        self.assertEqual(str(cam.error), "camera_open_failed")
        self.assertIsNone(cam.latest())

    def test_mismatched_sizes_are_an_error(self):
        def odd(seconds, on_pair, should_stop):
            on_pair({"width": 8, "height": 6, "rgb": b""}, {"width": 4, "height": 3, "mm": np.zeros((3, 4), np.uint16)})

        cam = camera.Camera(capture=odd)
        cam.start()
        cam.stop()
        self.assertIsInstance(cam.error, ValueError)


if __name__ == "__main__":
    unittest.main()
