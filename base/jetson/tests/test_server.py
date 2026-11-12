import json
import os
import socket
import struct
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import server  # noqa: E402


def ask(port, header, jpeg=b"jpg"):
    with socket.create_connection(("127.0.0.1", port), timeout=2) as sock:
        sock.settimeout(2)
        head = json.dumps(header).encode()
        sock.sendall(struct.pack(">I", len(head)) + head + jpeg)
        size = struct.unpack(">I", sock.recv(4))[0]
        data = b""
        while len(data) < size:
            data += sock.recv(size - len(data))
        return json.loads(data)


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.srv = server.Server(server.fake_people, "127.0.0.1", 0)
        self.srv.start()

    def tearDown(self):
        self.srv.stop()

    def test_fake_reply_echoes_seq_and_time_with_one_centred_person(self):
        reply = ask(self.srv.port, {"seq": 7, "t": 12.5, "width": 320, "height": 240, "jpeg": 3})
        self.assertEqual((reply["seq"], reply["t"]), (7, 12.5))
        self.assertEqual(len(reply["people"]), 1)
        x1, _, x2, _ = reply["people"][0]["box"]
        self.assertAlmostEqual((x1 + x2) / 2, 160)

    def test_several_frames_on_one_connection(self):
        with socket.create_connection(("127.0.0.1", self.srv.port), timeout=2) as sock:
            for seq in (1, 2):
                head = json.dumps({"seq": seq, "t": 0.0, "width": 10, "height": 10, "jpeg": 0}).encode()
                sock.sendall(struct.pack(">I", len(head)) + head)
                size = struct.unpack(">I", sock.recv(4))[0]
                self.assertEqual(json.loads(sock.recv(size))["seq"], seq)

    def test_oversized_header_closes_the_connection(self):
        with socket.create_connection(("127.0.0.1", self.srv.port), timeout=2) as sock:
            sock.sendall(struct.pack(">I", 10**9))
            self.assertEqual(sock.recv(4), b"")

    def test_a_malformed_header_closes_the_connection(self):
        with socket.create_connection(("127.0.0.1", self.srv.port), timeout=2) as sock:
            head = b"[1, 2]"
            sock.sendall(struct.pack(">I", len(head)) + head)
            self.assertEqual(sock.recv(4), b"")

    def test_a_second_client_replaces_the_first(self):
        first = socket.create_connection(("127.0.0.1", self.srv.port), timeout=2)
        self.addCleanup(first.close)
        head = json.dumps({"seq": 1, "t": 0.0, "width": 10, "height": 10, "jpeg": 0}).encode()
        first.sendall(struct.pack(">I", len(head)) + head)
        first.recv(1000)  # answered, so the server is serving it
        reply = ask(self.srv.port, {"seq": 2, "t": 0.0, "width": 10, "height": 10, "jpeg": 0}, b"")
        self.assertEqual(reply["seq"], 2)
        self.assertEqual(first.recv(1000), b"")  # first connection was closed by the server

    def test_an_idle_client_is_dropped(self):
        self.srv.stop()
        self.srv = server.Server(server.fake_people, "127.0.0.1", 0, idle=0.3)
        self.srv.start()
        with socket.create_connection(("127.0.0.1", self.srv.port), timeout=3) as sock:
            self.assertEqual(sock.recv(4), b"")

    def test_detect_calls_never_overlap(self):
        state = {"running": 0, "overlap": False}

        def slow_detect(jpeg, width, height):
            state["running"] += 1
            state["overlap"] |= state["running"] > 1
            time.sleep(0.3)
            state["running"] -= 1
            return []

        self.srv.stop()
        self.srv = server.Server(slow_detect, "127.0.0.1", 0)
        self.srv.start()
        header = {"seq": 1, "t": 0.0, "width": 10, "height": 10, "jpeg": 0}
        first = socket.create_connection(("127.0.0.1", self.srv.port), timeout=3)
        self.addCleanup(first.close)
        head = json.dumps(header).encode()
        first.sendall(struct.pack(">I", len(head)) + head)
        time.sleep(0.1)  # first is now inside slow_detect
        self.assertEqual(ask(self.srv.port, header, b"")["seq"], 1)  # replaces first, must wait its turn
        self.assertFalse(state["overlap"])

    def test_the_detector_seam_is_not_built_yet(self):
        with self.assertRaises(NotImplementedError):
            server.find_people(b"", 1, 1)


if __name__ == "__main__":
    unittest.main()
