"""live_feed.LiveFeed: what run_follow puts there each cycle, and what the web side takes out."""
import json
import math
import threading
import time
import unittest

import numpy

import live_feed


class Clock:
    """A fake clock; sleep() moves it on."""

    def __init__(self):
        self.now, self.slept = 0.0, 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.slept += seconds


class Encoder:
    def __init__(self):
        self.calls = []

    def __call__(self, frame):
        self.calls.append(frame)
        return b"jpeg-%s" % str(frame).encode()


RECORD = {"operator": [2.0, 1.0], "operator_kept": [2.0, 1.0], "locked": True, "people": [[1.0, 3.0]],
          "stop": "obstacle", "phase": "follow", "believed": [1.0, 1.0, math.pi / 2]}


class LiveFeedTest(unittest.TestCase):
    def feed(self):
        self.clock, self.encode = Clock(), Encoder()
        return live_feed.LiveFeed(encode=self.encode, clock=self.clock, sleep=self.clock.sleep)

    def test_scan_json_has_the_camera_view_shape_and_the_overlay(self):
        feed = self.feed()
        empty = json.loads(feed.scan_json(0.0))
        self.assertEqual(empty, {"points": [], "older": [], "nearest": None, "age": None, "operator": None,
                                 "people": [], "stop": None, "phase": None, "pose": None})
        for i in range(6):  # six scans: the newest and the 4 before it
            feed.put(None, numpy.array([[1.0 + i, 0.0], [0.0, -0.5], [9.0, 0.0]]), RECORD, 10.0 + i)
        got = json.loads(feed.scan_json(15.5))
        self.assertEqual(got["points"], [[0.0, -0.5]])  # 6 m and 9 m are past the 4 m drawn
        self.assertEqual(len(got["older"]), 4)
        self.assertEqual(got["older"][0], [[0.0, -0.5]])  # 5 m: left out too
        self.assertEqual(got["older"][-1], [[2.0, 0.0], [0.0, -0.5]])
        self.assertEqual((got["nearest"], got["age"]), (0.5, 0.5))
        # the overlay comes in the robot frame: the robot at (1, 1) facing +y, so (2, 1) is 1 m to its right
        self.assertEqual([round(v, 6) for v in got["operator"]], [0.0, -1.0])
        self.assertEqual([[round(v, 6) for v in q] for q in got["people"]], [[2.0, 0.0]])
        self.assertEqual((got["stop"], got["phase"]), ("obstacle", "follow"))
        self.assertEqual(json.loads(feed.scan_json(15.5))["pose"], None)  # RECORD has no odometry pose
        feed.put(None, numpy.zeros((0, 2)), dict(RECORD, pose=[0.5, 0.25, 0.1]), 16.0)
        self.assertEqual(json.loads(feed.scan_json(16.0))["pose"], [0.5, 0.25, 0.1])  # the points' own cycle

    def test_an_operator_not_seen_this_cycle_but_still_locked_is_kept(self):
        feed = self.feed()
        feed.put(None, numpy.zeros((0, 2)), dict(RECORD, operator=None, believed=[0.0, 0.0, 0.0]), 1.0)
        self.assertEqual(json.loads(feed.scan_json(1.0))["operator"], [2.0, 1.0])
        feed.put(None, numpy.zeros((0, 2)), dict(RECORD, operator=None, locked=False), 1.1)
        self.assertIsNone(json.loads(feed.scan_json(1.1))["operator"])

    def test_put_only_keeps_references(self):
        feed = self.feed()
        frame, points = object(), numpy.zeros((720, 2))
        feed.put(frame, points, RECORD, 1.0)
        self.assertEqual(self.encode.calls, [])  # no encoding inside the control cycle
        self.assertIs(feed.frame, frame)
        self.assertIs(feed.points, points)  # not copied
        feed.put(None, points, RECORD, 1.1)
        self.assertIs(feed.frame, frame)  # a cycle without a frame keeps the last one
        self.assertTrue(feed.has_frame())

    def test_the_stream_encodes_only_when_asked_and_at_most_live_fps(self):
        feed = self.feed()
        self.assertFalse(feed.has_frame())
        frames = feed.stream()
        feed.put("f0", None, {}, 0.0)
        self.assertEqual(self.encode.calls, [])  # no viewer asked yet
        self.assertEqual(next(frames), b"jpeg-f0")
        got = [b"jpeg-f0"]
        for i in range(1, 30):  # the camera far faster than LIVE_FPS: every take sees a new frame
            feed.put("f%d" % i, None, {}, self.clock.now)
            got.append(next(frames))
        self.assertEqual(len(self.encode.calls), 30)
        self.assertGreaterEqual(self.clock.now, 29 / live_feed.LIVE_FPS - 1e-9)  # 30 images took 2.9 s at least
        self.assertEqual(got[-1], b"jpeg-f29")

    def test_the_same_frame_is_encoded_once_for_every_viewer(self):
        feed = self.feed()
        feed.put("f0", None, {}, 0.0)
        one, two = feed.stream(), feed.stream()
        self.assertEqual((next(one), next(two)), (b"jpeg-f0", b"jpeg-f0"))
        self.assertEqual(self.encode.calls, ["f0"])

    def test_viewers_at_the_same_moment_still_encode_the_frame_once(self):
        calls = []

        def slow(frame):
            calls.append(frame)
            time.sleep(0.05)  # three viewers ask while the first encode runs
            return b"jpeg"

        feed = live_feed.LiveFeed(encode=slow)
        feed.put("f0", None, {}, 0.0)
        got = []
        viewers = [threading.Thread(target=lambda: got.append(feed._jpeg_of("f0"))) for _ in range(3)]
        for viewer in viewers:
            viewer.start()
        for viewer in viewers:
            viewer.join(5)
        self.assertEqual((calls, got), (["f0"], [b"jpeg"] * 3))

    def test_a_stream_without_a_new_frame_ends_and_frees_its_viewer(self):
        feed = self.feed()
        feed.put("f0", None, {}, 0.0)
        frames = feed.stream()
        self.assertEqual(next(frames), b"jpeg-f0")
        with self.assertRaises(StopIteration):  # the camera stopped: no new frame for STREAM_IDLE
            next(frames)
        self.assertGreaterEqual(self.clock.now, live_feed.STREAM_IDLE)
        self.assertLess(self.clock.now, live_feed.STREAM_IDLE + 0.5)

    def test_an_unchanged_frame_is_not_sent_again(self):
        feed = self.feed()
        feed.put("f0", None, {}, 0.0)
        frames = feed.stream()
        next(frames)
        waits = []
        real_sleep = self.clock.sleep

        def sleep(seconds):  # the camera gives a new frame only after the viewer waited a while
            waits.append(seconds)
            real_sleep(seconds)
            if len(waits) == 3:
                feed.put("f1", None, {}, self.clock.now)
        feed.sleep = sleep
        self.assertEqual(next(frames), b"jpeg-f1")
        self.assertEqual(self.encode.calls, ["f0", "f1"])


if __name__ == "__main__":
    unittest.main()
