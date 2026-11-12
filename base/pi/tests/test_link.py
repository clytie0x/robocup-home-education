import json
import os
import socket
import struct
import sys
import threading
import time
import unittest

import numpy as np

for jetson_dir in ("..", "jetson"), ("..", "..", "jetson"):  # for server.py: ../jetson on the Pi, ../../jetson on the laptop
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), *jetson_dir))

import link  # noqa: E402
import server  # noqa: E402
from camera import Frame  # noqa: E402


def make_frame(t, width=320, height=240):
    return Frame(t, width, height, bytes(width * height * 3), np.zeros((height, width), dtype=np.uint16))


def wait_for(condition, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


def raw_server(replies, stop):
    """Listens once per entry in replies; for each connection answers every request with the next raw reply bytes."""
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(2)
    listener.settimeout(0.2)

    def run():
        queue = list(replies)
        while not stop.is_set():
            try:
                conn, _ = listener.accept()
            except socket.timeout:
                continue
            conn.settimeout(0.5)
            try:
                while queue and not stop.is_set():
                    head = struct.unpack(">I", conn.recv(4))[0]
                    header = json.loads(conn.recv(head))
                    conn.recv(header["jpeg"])
                    conn.sendall(queue.pop(0))
            except (OSError, struct.error, ValueError):  # the link hung up
                pass
            conn.close()
        listener.close()

    threading.Thread(target=run, daemon=True).start()
    return listener.getsockname()[1]


def framed(obj):
    body = json.dumps(obj).encode()
    return struct.pack(">I", len(body)) + body


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.seen_frames = []
        self.delay = 0.0
        self.srv = None
        self.jl = None
        self.start_server(0)

    def start_server(self, port):
        def detect(jpeg, width, height):
            self.seen_frames.append(jpeg)
            time.sleep(self.delay)
            return server.fake_people(jpeg, width, height)

        self.srv = server.Server(detect, "127.0.0.1", port)
        self.srv.start()

    def make_link(self, port=None):
        self.jl = link.JetsonLink("127.0.0.1", port or self.srv.port, 58.59, encode=lambda f: b"jpg", reply_timeout=1.0)
        self.jl.start()
        return self.jl

    def tearDown(self):
        if self.jl:
            self.jl.stop()
        self.srv.stop()

    def test_round_trip_keeps_the_capture_time(self):
        jl = self.make_link()
        jl.send(make_frame(123.456))
        self.assertTrue(wait_for(lambda: jl.alive(time.monotonic(), 5)))
        answer = jl.latest()
        self.assertEqual(answer.t, 123.456)
        self.assertEqual(len(answer.people), 1)
        self.assertAlmostEqual(answer.people[0].bearing_deg, 0.0, places=3)
        self.assertEqual(answer.people[0].look, (0.1, 0.2, 0.3, 0.4))
        self.assertIsNone(jl.latest())  # each answer once

    def test_only_the_newest_frame_is_sent(self):
        self.delay = 0.3
        jl = self.make_link()
        for i in range(5):
            jl.send(make_frame(float(i)))
        self.assertTrue(wait_for(lambda: len(self.seen_frames) >= 1))
        time.sleep(1.0)
        self.assertIn(len(self.seen_frames), (1, 2))
        self.assertEqual(jl.latest().t, 4.0)

    def test_alive_goes_false_after_the_silence_limit(self):
        jl = self.make_link()
        self.assertFalse(jl.alive(time.monotonic(), 0.5))  # nothing heard yet
        jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: jl.alive(time.monotonic(), 0.5)))
        self.assertFalse(jl.alive(time.monotonic() + 0.6, 0.5))

    def test_link_reconnects_after_the_server_restarts(self):
        port = self.srv.port
        jl = self.make_link()
        jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: jl.latest() is not None))
        self.srv.stop()
        self.start_server(port)
        # keep sending until an answer from the new server arrives
        def got_answer():
            jl.send(make_frame(2.0))
            return jl.latest() is not None
        self.assertTrue(wait_for(got_answer, 5.0))

    def test_bearing_sign_image_right_is_robot_left(self):
        self.assertGreater(link.bearing_deg([300, 0, 320, 240], 320, 58.59), 20)
        self.assertLess(link.bearing_deg([0, 0, 20, 240], 320, 58.59), -20)
        self.assertAlmostEqual(link.bearing_deg([140, 0, 180, 240], 320, 58.59), 0.0)

    def test_send_never_blocks_without_a_server(self):
        self.srv.stop()
        self.srv = server.Server(server.fake_people, "127.0.0.1", 0)  # so tearDown has something to stop
        self.srv.start()
        jl = link.JetsonLink("127.0.0.1", 1, 58.59, encode=lambda f: b"jpg")  # nothing listens on port 1
        jl.start()
        self.jl = jl
        frame = make_frame(1.0)
        begin = time.monotonic()
        for _ in range(100):
            jl.send(frame)
        self.assertLess(time.monotonic() - begin, 0.1)

    def test_link_recovers_from_a_reply_with_a_missing_key(self):
        stop = threading.Event()
        self.addCleanup(stop.set)
        good = framed({"seq": 2, "t": 9.0, "people": []})
        port = raw_server([framed({"seq": 1}), good], stop)
        jl = self.make_link(port)
        def got_answer():
            jl.send(make_frame(9.0))
            return jl.latest() is not None
        self.assertTrue(wait_for(got_answer, 5.0))
        self.assertIn("KeyError", jl.error)

    def test_oversized_reply_length_drops_the_connection(self):
        stop = threading.Event()
        self.addCleanup(stop.set)
        port = raw_server([struct.pack(">I", 4_000_000_000)], stop)
        jl = self.make_link(port)
        jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: jl.error is not None))
        self.assertIn("too big", jl.error)
        self.assertIsNone(jl.latest())

    def test_encoder_error_skips_that_frame_only(self):
        calls = []
        def flaky(frame):
            calls.append(frame.time)
            if len(calls) == 1:
                raise RuntimeError("bad frame")
            return b"jpg"
        self.jl = link.JetsonLink("127.0.0.1", self.srv.port, 58.59, encode=flaky, reply_timeout=1.0)
        self.jl.start()
        self.jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: self.jl.error is not None))
        self.jl.send(make_frame(2.0))
        self.assertTrue(wait_for(lambda: self.jl.latest() is not None))
        self.assertEqual(len(self.seen_frames), 1)

    def test_default_encoder_makes_a_jpeg(self):
        jpeg = link._encode_jpeg(make_frame(1.0))
        self.assertEqual(jpeg[:2], b"\xff\xd8")


if __name__ == "__main__":
    unittest.main()
