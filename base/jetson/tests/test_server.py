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

    def test_people_get_their_box_confidence_and_look(self):
        import cv2
        import numpy as np

        class Detector:
            def detect(self, bgr):
                self.shape = bgr.shape
                return [(10.0, 20.0, 60.0, 200.0, 0.8)]

        class ReId:
            def looks(self, bgr, boxes):
                return [(0.6, 0.8) for _ in boxes]

        jpeg = cv2.imencode(".jpg", np.zeros((240, 320, 3), np.uint8))[1].tobytes()
        det = Detector()
        people = server.people_in(jpeg, det, ReId())
        self.assertEqual(det.shape, (240, 320, 3))  # the frame the Pi sent, not resized
        self.assertEqual(people, [{"box": [10.0, 20.0, 60.0, 200.0], "conf": 0.8, "look": [0.6, 0.8]}])
        self.assertEqual(server.people_in(b"not a jpeg", det, ReId()), [])


def jpeg_320x240():
    import cv2
    import numpy as np

    return cv2.imencode(".jpg", np.zeros((240, 320, 3), np.uint8))[1].tobytes()


class FakeDetector:
    def detect_all(self, bgr):
        return [(10.0, 20.0, 60.0, 200.0, 0.8), (200.0, 30.0, 260.0, 230.0, 0.7)], [(100.0, 200.0, 130.0, 230.0, 0.6, "handbag")]


class FakeReId:
    def looks(self, bgr, boxes):
        return [(0.6, 0.8) for _ in boxes]


def skeleton(x, y):
    return [[x + i, y + 2 * i, 0.9] for i in range(17)]


class PoseAndBagTests(unittest.TestCase):
    def test_bags_come_from_the_same_detector_run_and_people_are_unchanged(self):
        seen = server.seen_in(jpeg_320x240(), FakeDetector(), FakeReId())
        self.assertEqual(seen["objects"], [{"cls": "handbag", "box": [100.0, 200.0, 130.0, 230.0], "conf": 0.6}])
        self.assertEqual(seen["people"][0], {"box": [10.0, 20.0, 60.0, 200.0], "conf": 0.8, "look": [0.6, 0.8]})
        self.assertNotIn("kp", seen["people"][0])  # no pose asked
        self.assertEqual(server.seen_in(b"broken", FakeDetector(), FakeReId()), {"people": [], "objects": []})

    def test_keypoints_go_to_the_person_box_they_overlap(self):
        class Pose:
            def poses(self, bgr):
                return [((198.0, 32.0, 258.0, 228.0), 0.9, skeleton(220, 40)),
                        ((90.0, 10.0, 120.0, 60.0), 0.5, skeleton(95, 12))]  # overlaps nobody

        people = server.seen_in(jpeg_320x240(), FakeDetector(), FakeReId(), Pose())["people"]
        self.assertNotIn("kp", people[0])
        self.assertEqual(people[1]["kp"], skeleton(220, 40))

    def test_attach_matches_by_best_iou_one_pose_each(self):
        people = [{"box": [0, 0, 100, 200]}, {"box": [90, 0, 190, 200]}]
        poses = [((88, 0, 188, 200), 0.9, skeleton(1, 1)), ((2, 0, 102, 200), 0.9, skeleton(2, 2))]
        server.attach_keypoints(people, poses)
        self.assertEqual([p["kp"][0][0] for p in people], [2, 1])

    def test_decode_pose_rows(self):
        import numpy as np

        out = np.zeros((1, 56, 3), np.float32)
        out[0, :5, 0] = (160, 120, 60, 200, 0.9)
        out[0, 5:, 0] = np.tile([150.0, 50.0, 0.8], 17)
        out[0, :5, 1] = (161, 121, 60, 200, 0.7)  # the same person again
        out[0, :5, 2] = (40, 120, 30, 100, 0.2)  # too weak
        found = server.decode_pose(out, 0.5, 640, 480)
        self.assertEqual(len(found), 1)
        box, conf, kp = found[0]
        self.assertEqual([round(v) for v in box], [260, 40, 380, 440])
        self.assertAlmostEqual(conf, 0.9, places=5)
        self.assertEqual(len(kp), 17)
        self.assertEqual([round(v, 3) for v in kp[0]], [300.0, 100.0, 0.8])

    def test_fake_server_gives_keypoints_only_when_asked(self):
        self.assertNotIn("kp", server.fake_people(b"", 320, 240)[0])
        self.assertEqual(len(server.fake_people(b"", 320, 240, pose=True)[0]["kp"]), 17)


class PoseRequestTests(unittest.TestCase):
    def serve(self, **kw):
        calls = []

        def detect(jpeg, width, height, **extra):
            calls.append(extra.get("pose", False))
            return {"people": [], "objects": [{"cls": "suitcase", "box": [1, 2, 3, 4], "conf": 0.5}]}

        srv = server.Server(detect, "127.0.0.1", 0, **kw)
        srv.start()
        self.addCleanup(srv.stop)
        return srv, calls

    def test_pose_only_when_the_header_asks(self):
        srv, calls = self.serve()
        base = {"t": 0.0, "width": 10, "height": 10, "jpeg": 0}
        reply = ask(srv.port, dict(base, seq=1), b"")
        self.assertEqual(reply["objects"][0]["cls"], "suitcase")
        self.assertEqual(reply["people"], [])
        ask(srv.port, dict(base, seq=2, pose=True), b"")
        self.assertEqual(calls, [False, True])

    def test_pose_every_nth_frame_when_configured(self):
        srv, calls = self.serve(pose_every=3)
        with socket.create_connection(("127.0.0.1", srv.port), timeout=2) as sock:
            for seq in range(1, 7):
                head = json.dumps({"seq": seq, "t": 0.0, "width": 10, "height": 10, "jpeg": 0}).encode()
                sock.sendall(struct.pack(">I", len(head)) + head)
                size = struct.unpack(">I", sock.recv(4))[0]
                sock.recv(size)
        self.assertEqual(calls, [False, False, True, False, False, True])

if __name__ == "__main__":
    unittest.main()
