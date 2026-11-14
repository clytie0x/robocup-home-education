import math
import os
import socket
import sys
import threading
import time
import unittest

for jetson_dir in ("..", "jetson"), ("..", "..", "jetson"):  # for slam_bridge.py: ../jetson on the Pi, ../../jetson on the laptop
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), *jetson_dir))

import slam_bridge  # noqa: E402
import slam_link  # noqa: E402


def ring(count=400, meters=2.0):
    return [[-180.0 + 360.0 * i / count, meters] for i in range(count)]


def wait_for(condition, seconds=3.0):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


class SlamLinkTests(unittest.TestCase):
    def setUp(self):
        self.records = []
        self.link = None
        self.start_server(0)

    def start_server(self, port):
        self.server = slam_bridge.SlamServer(self.records.append, "127.0.0.1", port, idle=2.0)
        self.server.start()

    def make_link(self, port=None):
        self.link = slam_link.SlamLink("127.0.0.1", port or self.server.port, retry=0.1)
        self.link.start()
        return self.link

    def tearDown(self):
        if self.link:
            self.link.stop()
        self.server.stop()

    def reply(self, x, y, th):
        self.server.send_correction(1.0, x, y, th)

    def test_one_record_round_trip(self):
        link = self.make_link()
        link.send(12.5, (1.0, -2.0, 0.5), [(10.0, 1.234), (-20.0, 3.0)])
        self.assertTrue(wait_for(lambda: self.records))
        self.assertEqual(self.records[0], {"t": 12.5, "pose": [1.0, -2.0, 0.5], "scan": [[10.0, 1.234], [-20.0, 3.0]]})

    def test_every_revolution_arrives_in_order(self):
        link = self.make_link()
        for i in range(10):
            link.send(float(i), (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: len(self.records) == 10))
        self.assertEqual([r["t"] for r in self.records], [float(i) for i in range(10)])

    def test_correction_is_identity_before_the_first(self):
        self.assertEqual(self.make_link().correction(), (0.0, 0.0, 0.0))

    def test_a_correction_comes_back(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.reply(0.3, -0.2, 0.1)
        self.assertTrue(wait_for(lambda: link.correction() == (0.3, -0.2, 0.1)))

    def test_a_jump_over_half_a_metre_is_refused_and_counted(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.reply(0.3, 0.0, 0.0)
        self.assertTrue(wait_for(lambda: link.correction() == (0.3, 0.0, 0.0)))
        self.reply(0.9, 0.0, 0.0)  # 0.6 m from the last one
        self.assertTrue(wait_for(lambda: link.refused == 1))
        self.assertEqual(link.correction(), (0.3, 0.0, 0.0))
        self.reply(0.7, 0.0, 0.0)  # 0.4 m from the last one, fine
        self.assertTrue(wait_for(lambda: link.correction() == (0.7, 0.0, 0.0)))

    def test_a_jump_that_keeps_coming_is_taken_after_REBASE_AFTER_messages(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        for _ in range(slam_link.REBASE_AFTER - 1):
            self.reply(2.0, -3.4, 1.0)  # a new map far from identity, as after a slam_toolbox restart
        self.assertTrue(wait_for(lambda: link.refused == slam_link.REBASE_AFTER - 1))
        self.assertEqual(link.correction(), (0.0, 0.0, 0.0))
        self.reply(2.0, -3.4, 1.0)
        self.assertTrue(wait_for(lambda: link.correction() == (2.0, -3.4, 1.0)))
        self.assertEqual((link.refused, link.rebased), (slam_link.REBASE_AFTER, 1))

    def test_one_glitch_between_good_values_never_gets_through(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.reply(0.1, 0.0, 0.0)
        self.assertTrue(wait_for(lambda: link.correction() == (0.1, 0.0, 0.0)))
        for _ in range(3):
            self.reply(5.0, 0.0, 0.0)  # glitch
            self.reply(0.1, 0.0, 0.0)  # and back to the good value: the count of agreeing glitches restarts
        self.assertTrue(wait_for(lambda: link.refused == 3))
        self.assertEqual((link.correction(), link.rebased), ((0.1, 0.0, 0.0), 0))

    def test_jumping_values_that_do_not_agree_are_never_taken(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        for i in range(2 * slam_link.REBASE_AFTER):
            self.reply(1.0 + 0.7 * (i % 3), 0.0, 0.0)  # each 0.7 m from the last: no two in a row agree
        self.assertTrue(wait_for(lambda: link.refused == 2 * slam_link.REBASE_AFTER))
        self.assertEqual((link.correction(), link.rebased), ((0.0, 0.0, 0.0), 0))

    def test_a_jump_over_20_degrees_is_refused_across_the_wrap(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.reply(0.0, 0.0, math.radians(15))  # within 20 degrees of identity
        self.assertTrue(wait_for(lambda: link.correction()[2] == math.radians(15)))
        self.reply(0.0, 0.0, math.radians(-170))  # 175 degrees away, though only 10 from -180
        self.assertTrue(wait_for(lambda: link.refused == 1))
        self.reply(0.0, 0.0, math.radians(30))  # 15 degrees on, fine
        self.assertTrue(wait_for(lambda: link.correction()[2] == math.radians(30)))
        self.reply(0.0, 0.0, math.radians(60))  # 30 degrees on, refused
        self.assertTrue(wait_for(lambda: link.refused == 2))

    def test_a_bad_correction_is_ignored_and_the_link_goes_on(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.server.send_correction(1.0, float("nan"), 0.0, 0.0)
        self.reply(0.1, 0.0, 0.0)
        self.assertTrue(wait_for(lambda: link.correction() == (0.1, 0.0, 0.0)))
        self.assertEqual(link.refused, 0)

    def test_link_down_keeps_the_last_value_and_never_raises(self):
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.reply(0.2, 0.1, 0.05)
        self.assertTrue(wait_for(lambda: link.correction() == (0.2, 0.1, 0.05)))
        self.server.stop()
        self.assertTrue(wait_for(lambda: not link.connected))
        for _ in range(100):
            link.send(2.0, (0, 0, 0), ring())  # nobody listens
        self.assertEqual(link.correction(), (0.2, 0.1, 0.05))

    def test_reconnects_by_itself_after_the_jetson_restarts(self):
        port = self.server.port
        link = self.make_link()
        link.send(1.0, (0, 0, 0), ring())
        self.assertTrue(wait_for(lambda: self.records))
        self.server.stop()
        self.records.clear()
        self.start_server(port)

        def arrived():
            link.send(2.0, (0, 0, 0), ring())
            return bool(self.records)

        self.assertTrue(wait_for(arrived, 6.0))

    def test_send_never_blocks_without_a_server(self):
        link = slam_link.SlamLink("127.0.0.1", 1)  # nothing listens on port 1
        link.start()
        self.addCleanup(link.stop)
        scan = ring()
        begin = time.monotonic()
        for _ in range(100):
            link.send(1.0, (0, 0, 0), scan)
        self.assertLess(time.monotonic() - begin, 0.5)

    def test_the_backlog_is_bounded(self):
        link = slam_link.SlamLink("127.0.0.1", 1)  # never started: nothing drains the queue
        for i in range(slam_link.BACKLOG + 30):
            link.send(float(i), (0, 0, 0), [])
        self.assertEqual(len(link._queue), slam_link.BACKLOG)

    def test_oversized_length_from_the_jetson_drops_the_connection(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        self.addCleanup(listener.close)

        def serve():
            conn, _ = listener.accept()
            conn.sendall(b"\xff\xff\xff\xff")
            time.sleep(0.5)
            conn.close()

        threading.Thread(target=serve, daemon=True).start()
        link = slam_link.SlamLink("127.0.0.1", listener.getsockname()[1])
        link.start()
        self.addCleanup(link.stop)
        self.assertTrue(wait_for(lambda: link.error is not None and "too big" in link.error))
        self.assertEqual(link.correction(), (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
