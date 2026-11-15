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

    def test_replies_per_second_counts_the_answers_of_the_last_second(self):
        now = [100.0]
        self.jl = link.JetsonLink("127.0.0.1", self.srv.port, 58.59, encode=lambda f: b"jpg",
                                  reply_timeout=1.0, clock=lambda: now[0])
        self.jl.start()
        self.assertEqual(self.jl.replies_per_s(100.0), 0)  # nothing heard yet
        self.jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: self.jl.replies_per_s(100.0) == 1))
        now[0] = 100.8
        self.jl.send(make_frame(2.0))
        self.assertTrue(wait_for(lambda: self.jl.replies_per_s(100.8) == 2))
        self.assertEqual(self.jl.replies_per_s(102.0), 0)  # both answers are older than 1 s

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

    def test_bags_and_keypoints_are_parsed(self):
        stop = threading.Event()
        self.addCleanup(stop.set)
        kp = [[float(i), 2.0 * i, 0.9] for i in range(17)]
        reply = {"seq": 1, "t": 4.0, "people": [{"box": [140, 0, 180, 240], "conf": 0.9, "look": [0.5], "kp": kp}],
                 "objects": [{"cls": "handbag", "box": [300, 200, 320, 230], "conf": 0.6}]}
        jl = self.make_link(raw_server([framed(reply)], stop))
        jl.send(make_frame(4.0))
        self.assertTrue(wait_for(lambda: jl.alive(time.monotonic(), 5)))
        answer = jl.latest()
        self.assertEqual(answer.people[0].kp, kp)
        bag = answer.objects[0]
        self.assertEqual((bag.cls, bag.conf, bag.box), ("handbag", 0.6, [300, 200, 320, 230]))
        self.assertGreater(bag.bearing_deg, 20)  # image right is robot left, as for people

    def light_reply(self, quality):
        stop = threading.Event()
        self.addCleanup(stop.set)
        kp = [[float(i), 2.0 * i, 0.9] for i in range(17)]
        reply = {"seq": 1, "t": 4.0, "quality": quality,
                 "people": [{"box": [140, 0, 180, 240], "conf": 0.9, "look": [0.5], "kp": kp, "light": 182.5}]}
        jl = self.make_link(raw_server([framed(reply)], stop))
        jl.send(make_frame(4.0))
        self.assertTrue(wait_for(lambda: jl.alive(time.monotonic(), 5)))
        return jl.latest(), kp

    def test_a_frame_the_jetson_judged_too_dark_is_marked_and_has_no_keypoints(self):
        answer, _ = self.light_reply({"ok": False, "why": "dark", "luma": 3.1})
        self.assertFalse(answer.good)
        self.assertEqual(answer.why, "dark")
        self.assertEqual(len(answer.people), 1)  # still there, the senses decide
        self.assertIsNone(answer.people[0].kp)  # pointing never reads keypoints from a frame like that
        self.assertEqual(answer.people[0].light, 182.5)

    def test_a_good_frame_keeps_its_keypoints_and_box_light(self):
        answer, kp = self.light_reply({"ok": True, "why": None, "luma": 96.0})
        self.assertTrue(answer.good)
        self.assertIsNone(answer.why)
        self.assertEqual(answer.people[0].kp, kp)
        self.assertEqual(answer.people[0].light, 182.5)

    def test_an_old_reply_without_bags_or_keypoints_still_works(self):
        jl = self.make_link()
        jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: jl.alive(time.monotonic(), 5)))
        answer = jl.latest()
        self.assertEqual(answer.objects, [])
        self.assertIsNone(answer.people[0].kp)
        self.assertTrue(answer.good)  # no "quality": a server without the light gate, every frame counts
        self.assertIsNone(answer.people[0].light)
        self.assertEqual(link.Answer(1.0, []).objects, [])  # positional use elsewhere is unchanged
        self.assertTrue(link.Answer(1.0, []).good)
        self.assertIsNone(link.Seen(0.0, 0.9, (), [0, 0, 1, 1]).kp)
        self.assertIsNone(link.Seen(0.0, 0.9, (), [0, 0, 1, 1]).light)

    def test_pose_is_asked_only_for_frames_sent_with_pose(self):
        self.srv.stop()
        asked = []

        def detect(jpeg, width, height, pose=False):
            asked.append(pose)
            return server.fake_people(jpeg, width, height, pose)

        self.srv = server.Server(detect, "127.0.0.1", 0)
        self.srv.start()
        jl = self.make_link()
        jl.send(make_frame(1.0))
        self.assertTrue(wait_for(lambda: jl.alive(time.monotonic(), 5)))
        self.assertIsNone(jl.latest().people[0].kp)
        jl.send(make_frame(2.0), pose=True)
        self.assertTrue(wait_for(lambda: len(asked) == 2))
        self.assertTrue(wait_for(lambda: (jl._answer is not None and jl._answer.t == 2.0)))
        self.assertEqual(len(jl.latest().people[0].kp), 17)
        self.assertEqual(asked, [False, True])

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
        errors = set()  # .error is overwritten by later problems, so note each value as it appears

        def got_answer():
            errors.add(jl.error)
            jl.send(make_frame(9.0))
            return jl.latest() is not None
        self.assertTrue(wait_for(got_answer, 5.0))
        self.assertTrue(any(e and "KeyError" in e for e in errors), errors)

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


def free_port():
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


class BackupHostTests(unittest.TestCase):
    """Host names stand for ports on 127.0.0.1, through a fake connect."""

    def setUp(self):
        self.servers = {}
        self.ports = {"cable": free_port(), "wifi": free_port()}
        self.jl = None

    def tearDown(self):
        if self.jl:
            self.jl.stop()
        for srv in self.servers.values():
            srv.stop()

    def serve(self, name):
        self.servers[name] = server.Server(server.fake_people, "127.0.0.1", self.ports[name])
        self.servers[name].start()

    def connect(self, address, timeout):
        return socket.create_connection(("127.0.0.1", self.ports[address[0]]), timeout=timeout)

    def make_link(self, hosts, retry_primary=0.3):
        self.jl = link.JetsonLink(hosts, None, 58.59, encode=lambda f: b"jpg", reply_timeout=1.0,
                                  connect=self.connect, retry_primary=retry_primary)
        self.jl.start()
        return self.jl

    def answered(self):
        self.jl.send(make_frame(1.0))
        return self.jl.latest() is not None

    def test_primary_refused_uses_the_backup(self):
        self.serve("wifi")
        jl = self.make_link(["cable", "wifi"], retry_primary=60)
        self.assertTrue(wait_for(self.answered, 5.0))
        self.assertEqual(jl.host, "wifi")

    def test_link_goes_back_to_the_primary_when_it_returns(self):
        self.serve("wifi")
        jl = self.make_link(["cable", "wifi"])
        self.assertTrue(wait_for(self.answered, 5.0))
        self.assertEqual(jl.host, "wifi")
        self.serve("cable")
        self.assertTrue(wait_for(lambda: jl.host == "cable", 5.0))
        self.servers.pop("wifi").stop()  # answers must now come over the primary
        self.assertTrue(wait_for(self.answered, 5.0))
        self.assertEqual(jl.host, "cable")

    def test_primary_up_is_used_first(self):
        self.serve("cable")
        self.serve("wifi")
        jl = self.make_link(["cable", "wifi"])
        self.assertTrue(wait_for(self.answered, 5.0))
        self.assertEqual(jl.host, "cable")

    def black_hole(self, name):
        """Accepts connections and reads, never answers: TCP up, data not passing. Returns the accept count."""
        listener = socket.socket()
        listener.bind(("127.0.0.1", self.ports[name]))
        listener.listen(2)
        listener.settimeout(0.2)
        accepted, stop = [], threading.Event()
        self.addCleanup(stop.set)

        def run():
            conns = []
            while not stop.is_set():
                try:
                    conn, _ = listener.accept()
                    accepted.append(1)
                    conns.append(conn)
                except socket.timeout:
                    continue
            for conn in conns:
                conn.close()
            listener.close()
        threading.Thread(target=run, daemon=True).start()
        return accepted

    def test_a_host_that_connects_but_never_answers_is_skipped(self):
        accepted = self.black_hole("cable")
        self.serve("wifi")
        jl = self.make_link(["cable", "wifi"], retry_primary=0.3)
        self.assertTrue(wait_for(self.answered, 6.0))
        self.assertEqual(jl.host, "wifi")
        end = time.monotonic() + 2.0  # several retry_primary periods: the cable is not tried again
        while time.monotonic() < end:
            self.answered()
            time.sleep(0.05)
        self.assertEqual(jl.host, "wifi")
        self.assertEqual(len(accepted), 1)

    def test_a_single_host_string_still_works(self):
        self.serve("cable")
        jl = self.make_link("cable")
        self.assertTrue(wait_for(self.answered, 5.0))
        self.assertEqual(jl.host, "cable")


class HostsTests(unittest.TestCase):
    def test_primary_then_backups_without_duplicates(self):
        p = {"jetson_host": "10.0.0.2", "jetson_hosts_backup": ["192.168.31.137", "10.0.0.2", "100.108.202.48", "192.168.31.137"]}
        self.assertEqual(link.hosts(p), ["10.0.0.2", "192.168.31.137", "100.108.202.48"])

    def test_no_backups_is_the_primary_alone(self):
        self.assertEqual(link.hosts({"jetson_host": "10.0.0.2"}), ["10.0.0.2"])

    def test_committed_params_list_the_cable_first(self):
        import params
        self.assertEqual(link.hosts(params.load()), ["10.0.0.2", "192.168.31.137", "100.108.202.48"])


if __name__ == "__main__":
    unittest.main()
