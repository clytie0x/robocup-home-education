"""tools/capture_front.py: the change gate and the file names, on fake JPEG bytes (no camera, no network)."""
import os
import sys
import tempfile
import time
import unittest
from unittest import mock

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))

import capture_front as cf  # noqa: E402


def jpeg(grey, size=(640, 480)):
    """A flat grey frame as JPEG bytes."""
    return cv2.imencode(".jpg", np.full((size[1], size[0], 3), grey, np.uint8))[1].tobytes()


class GateTest(unittest.TestCase):
    def test_the_first_frame_is_kept_and_the_same_view_again_is_not(self):
        g = cf.Gate(6.0)
        self.assertEqual(g.offer(jpeg(100)), float("inf"))
        self.assertIsNone(g.offer(jpeg(102)))

    def test_a_changed_view_is_kept_and_becomes_the_reference(self):
        g = cf.Gate(6.0)
        g.offer(jpeg(100))
        self.assertAlmostEqual(g.offer(jpeg(120)), 20.0, delta=1.0)
        self.assertIsNone(g.offer(jpeg(123)))  # 3 from 120 now, though 23 from 100

    def test_a_slow_drift_is_measured_from_the_last_kept_frame(self):
        g = cf.Gate(5.0)
        g.offer(jpeg(100))
        kept = [g.offer(jpeg(100 + 2 * i)) is not None for i in range(1, 8)]
        self.assertEqual(kept, [False, False, True, False, False, True, False])  # at 106 and at 112

    def test_a_change_in_part_of_the_view_counts_by_its_share(self):
        img = np.full((480, 640, 3), 100, np.uint8)
        g = cf.Gate(6.0)
        g.offer(cv2.imencode(".jpg", img)[1].tobytes())
        img[:240, :320] = 200  # a quarter of the view, 100 levels brighter: about 25 on average
        self.assertAlmostEqual(g.offer(cv2.imencode(".jpg", img)[1].tobytes()), 25.0, delta=2.0)

    def test_bytes_that_are_no_image_are_skipped(self):
        g = cf.Gate()
        self.assertIsNone(g.offer(b""))
        self.assertIsNone(g.offer(b"<html>camera busy</html>"))
        self.assertIsNone(g.last)


class NameTest(unittest.TestCase):
    T = time.mktime((2026, 11, 21, 14, 5, 9, 0, 0, -1))

    def test_the_name_is_the_local_time_to_the_second(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(cf.frame_path(d, self.T), os.path.join(d, "front_20261121-140509.jpg"))

    def test_a_second_frame_in_the_same_second_does_not_overwrite_the_first(self):
        with tempfile.TemporaryDirectory() as d:
            open(os.path.join(d, "front_20261121-140509.jpg"), "wb").close()
            self.assertEqual(cf.frame_path(d, self.T), os.path.join(d, "front_20261121-140509-2.jpg"))

    def test_only_the_jetsons_snapshot_url(self):
        self.assertEqual(cf.snap_url("192.168.31.137"), "http://192.168.31.137:8082/snap.jpg")
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit):
            cf.main(["out", "--host", "10.0.0.1"])


class LoopTest(unittest.TestCase):
    def test_the_loop_writes_the_changed_frames_as_sent_and_survives_a_failed_get(self):
        frames = [jpeg(100), jpeg(101), OSError("timed out"), jpeg(160), KeyboardInterrupt()]
        with tempfile.TemporaryDirectory() as d, mock.patch.object(cf, "fetch", side_effect=frames) as fetch, \
                mock.patch.object(cf.time, "sleep"), mock.patch("sys.stdout"):
            self.assertEqual(cf.main([d]), 2)
            urls = {c.args[0] for c in fetch.call_args_list}
            saved = sorted(os.listdir(d))
            data = [open(os.path.join(d, n), "rb").read() for n in saved]
        self.assertEqual(urls, {"http://100.108.202.48:8082/snap.jpg"})
        self.assertEqual(len(saved), 2)
        self.assertTrue(all(n.startswith("front_") and n.endswith(".jpg") for n in saved))
        self.assertEqual(sorted(data), sorted([frames[0], frames[3]]))


if __name__ == "__main__":
    unittest.main()
