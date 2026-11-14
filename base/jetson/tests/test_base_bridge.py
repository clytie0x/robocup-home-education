import importlib.util
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import base_bridge  # noqa: E402
from base_bridge import BridgeLogic, PiLink  # noqa: E402


def sent(out):
    return [v["cmd"] for k, v in out if k == "send"]


def events(out):
    return [v for k, v in out if k == "event"]


def statuses(out):
    return [v for k, v in out if k == "status"]


def status(phase="FOLLOW", locked=True, at_start=False, **kw):
    s = {"t": 1.0, "phase": phase, "waiting": False, "locked": locked, "operator_m": 1.2,
         "breadcrumbs": 3, "queue_tail": False, "at_start": at_start, "stop": "", "ack": ""}
    s.update(kw)
    return s


class FakeSocket:
    def __init__(self):
        self.incoming = []  # byte chunks recv() will hand out
        self.lines = []  # what the bridge sent, decoded
        self.closed = False
        self.fail_send = False
        self.eof = False

    def recv(self, n):
        if self.closed:
            raise OSError("closed")
        if self.incoming:
            return self.incoming.pop(0)
        if self.eof:
            return b""
        raise BlockingIOError()

    def sendall(self, data):
        if self.fail_send or self.closed:
            raise OSError("broken pipe")
        for line in data.decode().splitlines():
            self.lines.append(json.loads(line))

    def close(self):
        self.closed = True

    def feed(self, obj):
        self.incoming.append((json.dumps(obj) + "\n").encode())

    def cmds(self):
        return [m["cmd"] for m in self.lines if m["cmd"] != "ping"]

    def pings(self):
        return [m for m in self.lines if m["cmd"] == "ping"]


class FakeConnector:
    """Hands out sockets from a list; None in the list = refuse (OSError)."""

    def __init__(self, plan):
        self.plan = list(plan)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        item = self.plan.pop(0) if self.plan else None
        if item is None:
            raise OSError("connection refused")
        return item


def connected(now=0.0, phase=None):
    logic = BridgeLogic()
    logic.on_connected(now)
    if phase:
        logic.on_status(status(phase=phase, locked=False), now)
    return logic


class MappingTests(unittest.TestCase):
    def test_actions_map_to_pi_commands(self):
        for action, cmd in [("follow_me", "follow"), ("go_back", "return"), ("wait", "wait"),
                            ("resume", "resume"), ("end", "stop")]:
            logic = connected()
            self.assertEqual(sent(logic.on_command({"action": action}, 1.0)), [cmd], action)

    def test_stop_while_moving_only_waits(self):
        for phase in ("FOLLOW", "RETURN"):
            logic = connected(phase=phase)
            self.assertEqual(sent(logic.on_command({"action": "stop"}, 1.0)), ["wait"], phase)

    def test_stop_when_idle_or_done_stops(self):
        for phase in ("IDLE", "DONE", None):
            logic = connected(phase=phase)
            self.assertEqual(sent(logic.on_command({"action": "stop"}, 1.0)), ["stop"], phase)

    def test_other_actions_are_ignored(self):
        logic = connected()
        for action in ("carry_bag", "drop_bag", "move_forward", "turn", None):
            self.assertEqual(sent(logic.on_command({"action": action}, 1.0)), [], action)

    def test_command_while_disconnected_is_sent_on_connect_latest_only(self):
        logic = BridgeLogic()
        self.assertEqual(sent(logic.on_command({"action": "follow_me"}, 0.0)), [])
        self.assertEqual(sent(logic.on_command({"action": "end"}, 0.1)), [])
        self.assertEqual(sent(logic.on_connected(1.0)), ["stop"])
        self.assertEqual(sent(logic.on_connected(2.0)), [])  # only once


class EventTests(unittest.TestCase):
    def test_lock_edges_give_see_and_lost_person_with_debounce(self):
        logic = connected()
        self.assertEqual(events(logic.on_status(status(locked=True), 0.0)), ["base_link_ok", "see_person"])
        self.assertEqual(events(logic.on_status(status(locked=True), 0.2)), [])
        self.assertEqual(events(logic.on_status(status(locked=False), 1.0)), [])
        self.assertEqual(events(logic.on_tick(1.3)), [])
        self.assertEqual(events(logic.on_status(status(locked=True), 1.4)), [])  # blip: nothing
        self.assertEqual(events(logic.on_status(status(locked=False), 2.0)), [])
        self.assertEqual(events(logic.on_status(status(locked=False), 2.4)), [])
        self.assertEqual(events(logic.on_tick(2.5)), ["lost_person"])
        self.assertEqual(events(logic.on_tick(3.0)), [])
        self.assertEqual(events(logic.on_status(status(locked=True), 3.1)), ["see_person"])

    def test_unlocked_at_first_says_nothing(self):
        logic = connected()
        logic.on_status(status(locked=False), 0.0)
        self.assertEqual(events(logic.on_tick(1.0)), [])

    def test_at_start_rising_edge_once(self):
        logic = connected()
        self.assertEqual(events(logic.on_status(status(locked=False, at_start=False), 0.0)), ["base_link_ok"])
        self.assertEqual(events(logic.on_status(status(locked=False, at_start=True), 0.2)), ["at_start"])
        self.assertEqual(events(logic.on_status(status(locked=False, at_start=True), 0.4)), [])
        logic.on_status(status(locked=False, at_start=False), 0.6)
        self.assertEqual(events(logic.on_status(status(locked=False, at_start=True), 0.8)), ["at_start"])

    def test_status_is_published_as_json_ready_dict(self):
        logic = connected()
        out = statuses(logic.on_status(status(phase="RETURN", locked=False), 0.0))
        self.assertEqual(out[0]["phase"], "RETURN")
        self.assertTrue(out[0]["link"])
        json.dumps(out[0])

    def test_pi_error_is_published_and_logged(self):
        logic = connected()
        out = logic.on_status({"error": "unknown command"}, 0.0)
        self.assertEqual(statuses(out)[0]["error"], "unknown command")
        self.assertIn("log", [k for k, _ in out])


class LinkTests(unittest.TestCase):
    def test_first_status_is_link_ok(self):
        logic = connected()
        self.assertIn("base_link_ok", events(logic.on_status(status(), 0.1)))
        self.assertNotIn("base_link_ok", events(logic.on_status(status(), 0.3)))

    def test_silence_for_1_5_s_is_link_lost_and_drops_the_socket(self):
        logic = connected()
        logic.on_status(status(locked=False), 0.0)
        self.assertEqual(events(logic.on_tick(1.4)), [])
        out = logic.on_tick(1.6)
        self.assertEqual(events(out), ["base_link_lost"])
        self.assertIn("drop", [k for k, _ in out])
        self.assertFalse(statuses(out)[0]["link"])
        self.assertEqual(events(logic.on_disconnected(1.6)), [])  # not twice
        logic.on_connected(2.5)
        self.assertEqual(events(logic.on_status(status(locked=False), 2.6)), ["base_link_ok"])

    def test_socket_drop_is_link_lost(self):
        logic = connected()
        logic.on_status(status(locked=False), 0.0)
        self.assertEqual(events(logic.on_disconnected(0.5)), ["base_link_lost"])

    def test_connected_but_never_a_status_is_link_lost_once(self):
        logic = BridgeLogic()
        logic.on_connected(0.0)
        out = logic.on_tick(1.6)
        self.assertEqual(events(out), ["base_link_lost"])
        self.assertIn("drop", [k for k, _ in out])
        logic.on_disconnected(1.6)
        self.assertEqual(events(logic.on_connect_failed(3.0)), [])

    def test_lock_after_link_loss_is_seen_again(self):
        logic = connected()
        logic.on_status(status(locked=True), 0.0)
        logic.on_disconnected(0.5)
        logic.on_connected(1.0)
        self.assertEqual(events(logic.on_status(status(locked=True), 1.1)), ["base_link_ok", "see_person"])


class PingTests(unittest.TestCase):
    def test_ping_every_half_second_while_connected(self):
        logic = connected(0.0)
        pings = []
        for i in range(21):  # 0.0 .. 2.0 s in 0.1 s steps
            t = i / 10
            logic.on_status(status(), t)  # keep the link alive
            if sent(logic.on_tick(t)) == ["ping"]:
                pings.append(t)
        self.assertEqual(pings, [0.0, 0.5, 1.0, 1.5, 2.0])

    def test_no_ping_while_disconnected(self):
        logic = BridgeLogic()
        self.assertEqual(sent(logic.on_tick(1.0)), [])


class PiLinkTests(unittest.TestCase):
    def test_reconnects_with_backoff_and_resets_after_a_status(self):
        sock = FakeSocket()
        conn = FakeConnector([None, None, None, sock])
        link = PiLink(BridgeLogic(), conn)
        attempts = []
        t = 0.0
        while t < 4.0 and link.sock is None:
            before = conn.calls
            link.step(t)
            if conn.calls > before:
                attempts.append(round(t, 2))
            t = round(t + 0.05, 2)
        self.assertEqual(attempts, [0.0, 0.5, 1.5, 3.5])  # waits 0.5, 1, 2 s
        sock.feed(status())
        out = link.step(t)
        self.assertIn("base_link_ok", events(out))
        sock.eof = True
        out = link.step(t + 0.05)
        self.assertIn("base_link_lost", events(out))
        self.assertIsNone(link.sock)
        self.assertTrue(sock.closed)
        # reset after the status: the next try is 0.5 s later, not 4 s
        sock2 = FakeSocket()
        conn.plan = [sock2]
        link.step(t + 0.3)
        self.assertIsNone(link.sock)
        link.step(t + 0.6)
        self.assertIs(link.sock, sock2)

    def test_backoff_is_capped(self):
        logic = BridgeLogic()
        t = 0.0
        for _ in range(10):
            logic.on_connect_failed(t)
            t = logic.next_connect_at
        self.assertLessEqual(logic.retry_delay, base_bridge.MAX_BACKOFF_S)

    def test_lines_split_across_reads_and_bad_json(self):
        sock = FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock]))
        link.step(0.0)
        line = (json.dumps(status(locked=True)) + "\n").encode()
        sock.incoming = [b"not json\n" + line[:10], line[10:]]
        out = link.step(0.1)
        self.assertEqual(events(out), ["base_link_ok", "see_person"])
        self.assertTrue(any(k == "log" and "bad line" in v for k, v in out))

    def test_commands_and_pings_go_out_as_json_lines(self):
        sock = FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock]))
        link.step(0.0)
        link.command({"action": "follow_me"}, 0.1)
        self.assertEqual(sock.cmds(), ["follow"])
        self.assertEqual(len(sock.pings()), 1)

    def test_send_failure_drops_and_reconnects(self):
        sock = FakeSocket()
        sock2 = FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock, sock2]))
        link.step(0.0)
        sock.feed(status())
        link.step(0.1)
        sock.fail_send = True
        out = link.command({"action": "go_back"}, 0.2)
        self.assertIn("base_link_lost", events(out))
        self.assertIsNone(link.sock)
        link.step(0.8)
        self.assertIs(link.sock, sock2)
        self.assertEqual(sock2.cmds(), ["return"])  # the failed command is retried on the new link

    def test_silent_pi_is_dropped(self):
        sock = FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock]))
        link.step(0.0)
        sock.feed(status())
        link.step(0.1)
        out = link.step(1.7)
        self.assertIn("base_link_lost", events(out))
        self.assertTrue(sock.closed)
        self.assertIsNone(link.sock)

    def test_overlong_line_drops_the_connection(self):
        sock = FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock]))
        link.step(0.0)
        sock.incoming = [b"x" * (base_bridge.MAX_LINE + 1)]
        link.step(0.1)
        self.assertIsNone(link.sock)


class HostTests(unittest.TestCase):
    def test_argument_wins_then_params(self):
        self.assertEqual(base_bridge.pi_host("10.0.0.2", {"pi_host": "10.0.0.9"}), "10.0.0.2")
        self.assertEqual(base_bridge.pi_host(None, {"pi_host": "10.0.0.9"}), "10.0.0.9")
        with self.assertRaises(SystemExit):
            base_bridge.pi_host(None, {})


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "rclpy is not installed")
class RosTests(unittest.TestCase):
    def test_the_node_can_be_built(self):
        import rclpy

        node = base_bridge.make_bridge("127.0.0.1", 1)
        try:
            self.assertEqual(node.get_name(), "base_bridge")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
