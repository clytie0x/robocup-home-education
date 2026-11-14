"""command_server.py on localhost: what a client sends, line by line, and what it gets back."""
import json
import select
import socket
import threading
import time
import unittest

from command_server import MAX_LINE, MAX_READ, CommandServer

PING = b'{"cmd": "ping"}\n'


class Client:
    """The Jetson's end of --remote, on localhost: sends command lines, reads the Pi's lines."""

    def __init__(self, port, sock=None):
        self.sock = sock or socket.create_connection(("127.0.0.1", port), timeout=2.0)
        self.buffer, self.closed = b"", False

    def send(self, raw):
        self.sock.sendall(raw)
        time.sleep(0.05)  # arrived before the next cycle polls

    def cmd(self, name):
        self.send(json.dumps({"cmd": name}).encode("utf-8") + b"\n")

    def read(self, wait=0.05):
        """Every line the Pi has sent so far, as objects."""
        while not self.closed and select.select([self.sock], [], [], wait)[0]:
            try:
                chunk = self.sock.recv(65536)
            except ConnectionResetError:
                chunk = b""
            self.closed = not chunk
            self.buffer += chunk
        lines = self.buffer.split(b"\n")
        self.buffer = lines.pop()
        return [json.loads(line) for line in lines]

    def errors(self):
        return [r["error"] for r in self.read() if "error" in r]

    def close(self):
        self.sock.close()


class Broken:
    """A client socket that fails in a way no handler expects."""

    def recv(self, size):
        raise RuntimeError("recv broke")

    def send(self, data):
        raise RuntimeError("send broke")

    def close(self):
        pass


class Server(unittest.TestCase):
    def setUp(self):
        self.server = CommandServer("127.0.0.1", 0)
        self.server.start()
        self.addCleanup(self.server.stop)

    def client(self, sock=None):
        client = Client(self.server.port, sock)
        self.addCleanup(client.close)
        return client

    def test_a_line_split_across_polls_is_one_command(self):
        jetson = self.client()
        jetson.send(b'{"cmd": "fo')
        self.assertEqual(self.server.poll(0.0), [])
        jetson.send(b'llow"}\n')
        self.assertEqual(self.server.poll(0.1), ["follow"])
        self.assertEqual(jetson.errors(), [])

    def test_a_complete_line_longer_than_max_line_is_rejected_whole(self):
        jetson = self.client()
        jetson.send(json.dumps({"cmd": "wait", "pad": "x" * MAX_LINE}).encode("utf-8") + b"\n" + PING)
        self.assertEqual(self.server.poll(0.0), ["ping"])
        self.assertEqual(len(jetson.errors()), 1)

    def test_the_tail_of_a_cut_line_is_not_read_as_a_new_line(self):
        jetson = self.client()
        jetson.send(b'{"pad": "' + b"x" * (MAX_LINE + 10))
        self.assertEqual(self.server.poll(0.0), [])
        self.assertEqual(len(jetson.errors()), 1)
        jetson.send(b'xxx", "cmd": "stop"}\n{"cmd": "wait"}\n')
        self.assertEqual(self.server.poll(0.1), ["wait"])
        self.assertEqual(jetson.errors(), [])

    def test_one_poll_takes_at_most_max_read(self):
        jetson = self.client()
        count = 6000  # 96 000 bytes, more than MAX_READ
        sender = threading.Thread(target=jetson.sock.sendall, args=(PING * count,))
        sender.start()
        time.sleep(0.2)
        first = self.server.poll(0.0)
        self.assertLess(len(first), count)
        self.assertLessEqual(len(first) * len(PING), MAX_READ + 4096)
        got = len(first)
        for _ in range(100):
            if got == count:
                break
            time.sleep(0.02)
            got += len(self.server.poll(0.0))
        sender.join(5)
        self.assertEqual(got, count)

    def test_a_client_that_never_reads_is_dropped(self):
        sock = socket.socket()
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        sock.connect(("127.0.0.1", self.server.port))
        self.client(sock)
        self.server.poll(0.0)
        self.server.client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        for _ in range(5000):  # up to 5 MB
            self.server.send({"pad": "x" * 1000})
            if self.server.client is None:
                break
        self.assertIsNone(self.server.client)
        self.assertIn("not reading its status", self.server.events)

    def test_the_last_line_of_a_client_that_just_closed_counts(self):
        jetson = self.client()
        self.server.poll(0.0)
        jetson.send(b'{"cmd": "stop"}\n')
        jetson.close()
        time.sleep(0.05)
        self.assertEqual(self.server.poll(0.1), ["stop"])
        self.assertIsNone(self.server.client)
        self.assertIn("closed by the client", self.server.events)
        self.assertEqual(self.server.heard, 0.1)

    def test_a_replaced_clients_buffered_lines_still_count(self):
        first = self.client()
        self.server.poll(0.0)
        first.cmd("wait")  # sent just before its Wi-Fi dropped
        second = self.client()
        second.cmd("ping")
        self.assertEqual(self.server.poll(0.1), ["wait", "ping"])
        self.assertTrue(any(e.startswith("replaced") for e in self.server.events))
        first.read()
        self.assertTrue(first.closed)

    def test_two_clients_waiting_to_be_accepted_both_count(self):
        first = self.client()
        first.cmd("wait")
        second = self.client()
        second.cmd("follow")
        self.assertEqual(self.server.poll(0.0), ["wait", "follow"])

    def test_deeply_nested_json_is_an_error_reply(self):
        jetson = self.client()
        jetson.send(b"[" * 3000 + b"\n" + PING)
        self.assertEqual(self.server.poll(0.0), ["ping"])
        self.assertEqual(len(jetson.errors()), 1)

    def test_a_broken_socket_is_dropped_and_logged_not_raised(self):
        self.client()
        self.server.poll(0.0)
        self.server.client.close()
        self.server.client = Broken()
        self.assertEqual(self.server.poll(0.1), [])
        self.assertIsNone(self.server.client)
        self.assertTrue(any("recv broke" in e for e in self.server.events))
        self.server.client = Broken()
        self.server.send({"t": 0.2})  # no exception
        self.assertIsNone(self.server.client)
        self.assertTrue(any("send broke" in e for e in self.server.events))

    def test_a_sender_not_on_the_allow_list_is_closed_at_once_and_logged_once_a_minute(self):
        self.server.allow = ("10.9.9.9",)  # the Jetson only: this test's 127.0.0.1 is someone else on the LAN
        stranger = self.client()
        stranger.cmd("follow")
        self.assertEqual(self.server.poll(0.0), [])
        self.assertIsNone(self.server.client)
        stranger.read()
        self.assertTrue(stranger.closed)
        self.assertEqual([e for e in self.server.events if "refused" in e], ["refused 127.0.0.1: not on the allow list"])
        again = self.client()
        again.cmd("follow")
        self.assertEqual(self.server.poll(30.0), [])
        self.assertEqual(len([e for e in self.server.events if "refused" in e]), 1)  # once a minute per address
        third = self.client()
        third.cmd("ping")
        self.assertEqual(self.server.poll(61.0), [])
        self.assertEqual(len([e for e in self.server.events if "refused" in e]), 2)

    def test_a_refused_sender_does_not_replace_the_jetson(self):
        jetson = self.client()
        jetson.cmd("ping")
        self.assertEqual(self.server.poll(0.0), ["ping"])
        kept = self.server.client
        self.server.allow = ("10.9.9.9",)  # whoever connects now is not the Jetson
        stranger = self.client()
        stranger.cmd("ping")  # its pings must not keep the silence hold away
        self.assertEqual(self.server.poll(0.1), [])
        self.assertIs(self.server.client, kept)
        self.assertEqual(self.server.heard, 0.0)
        jetson.cmd("wait")
        self.assertEqual(self.server.poll(0.2), ["wait"])

    def test_by_default_only_this_computer_may_send(self):
        self.assertEqual(CommandServer().allow, ("127.0.0.1",))

    def test_silent_counts_from_the_first_poll_and_the_last_command(self):
        self.assertEqual(self.server.silent(5.0), 0.0)  # never polled
        self.server.poll(1.0)
        self.assertEqual(self.server.silent(3.5), 2.5)  # no client at all
        jetson = self.client()
        jetson.send(b"junk\n")
        self.server.poll(4.0)
        self.assertEqual(self.server.silent(4.0), 3.0)  # a malformed line is not heard
        jetson.cmd("ping")
        self.server.poll(4.5)
        self.assertEqual(self.server.silent(4.5), 0.0)


if __name__ == "__main__":
    unittest.main()
