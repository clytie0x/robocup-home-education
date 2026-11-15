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

    def __call__(self, timeout=None):
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

    def test_stop_when_idle_done_or_unknown_sends_nothing(self):
        for phase in ("IDLE", "DONE", None):
            logic = connected(phase=phase)
            out = logic.on_command({"action": "stop"}, 1.0)
            self.assertEqual(sent(out), [], phase)
            self.assertIn("log", [k for k, _ in out])

    def test_other_actions_are_ignored(self):
        logic = connected()
        for action in ("carry_bag", "pick_bag", "drop_bag", "move_forward", "turn", None):
            self.assertEqual(sent(logic.on_command({"action": action}, 1.0)), [], action)

    def test_follow_asked_while_disconnected_is_not_sent_to_an_idle_pi(self):
        logic = BridgeLogic()
        self.assertEqual(sent(logic.on_command({"action": "follow_me"}, 0.0)), [])
        self.assertEqual(sent(logic.on_connected(1.0)), [])  # nothing blind: wait for the Pi's state
        self.assertEqual(sent(logic.on_status(status(phase="IDLE", locked=False), 1.1)), [])  # may be a new run
        self.assertEqual(sent(logic.on_status(status(phase="IDLE", locked=False), 2.5)), [])

    def test_end_while_disconnected_is_never_sent_later(self):
        logic = BridgeLogic()
        logic.on_command({"action": "follow_me"}, 0.0)
        logic.on_command({"action": "end"}, 0.1)
        self.assertEqual(sent(logic.on_connected(1.0)), [])
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", locked=False, held="silent"), 1.1)), [])


class RestoreTests(unittest.TestCase):
    """BridgeLogic keeps what the flow last asked for and puts the Pi back into it."""

    def at(self, action, phase, now=0.0):
        logic = connected(now, phase=phase)
        logic.on_command({"action": action}, now)
        return logic

    def restore(self, logic, now, **kw):
        kw.setdefault("locked", False)
        return sent(logic.on_status(status(**kw), now))

    def test_following_but_held_by_silence_resumes(self):
        logic = self.at("follow_me", "IDLE")
        self.assertEqual(self.restore(logic, 2.0, phase="FOLLOW", held="silent"), ["resume"])
        self.assertEqual(self.restore(logic, 2.2, phase="FOLLOW", held="silent"), [])  # not at 5 Hz
        self.assertEqual(self.restore(logic, 3.1, phase="FOLLOW", held="silent"), ["resume"])
        self.assertEqual(self.restore(logic, 4.5, phase="FOLLOW", held=None), [])

    def test_following_and_running_needs_nothing(self):
        logic = self.at("follow_me", "IDLE")
        self.assertEqual(self.restore(logic, 2.0, phase="FOLLOW", held=None, waiting=False), [])

    def test_following_but_waiting_resumes(self):
        logic = self.at("follow_me", "IDLE")
        self.assertEqual(self.restore(logic, 2.0, phase="FOLLOW", waiting=True, held="wait"), ["resume"])

    def test_following_but_idle_on_the_same_link_follows(self):
        logic = self.at("follow_me", "IDLE")
        self.assertEqual(self.restore(logic, 2.0, phase="IDLE"), ["follow"])  # the follow was lost on the way

    def test_following_but_done_is_a_run_end_not_a_follow(self):
        logic = self.at("follow_me", "IDLE")
        out = logic.on_status(status(phase="DONE", locked=False), 2.0)
        self.assertEqual((sent(out), events(out), logic.intent), ([], ["base_run_ended"], "idle"))

    def test_a_fresh_command_is_not_repeated_at_once(self):
        logic = self.at("follow_me", "IDLE", now=1.0)
        self.assertEqual(self.restore(logic, 1.2, phase="IDLE"), [])  # the Pi has not read it yet

    def test_waiting_but_not_waiting_waits(self):
        logic = self.at("follow_me", "IDLE")
        logic.on_command({"action": "wait"}, 1.0)
        self.assertEqual(self.restore(logic, 2.5, phase="FOLLOW", waiting=False, held=None), ["wait"])
        self.assertEqual(self.restore(logic, 4.0, phase="FOLLOW", waiting=False, held="silent"), ["wait"])
        self.assertEqual(self.restore(logic, 5.5, phase="FOLLOW", waiting=True, held="wait"), [])

    def test_flow_stop_while_following_is_a_wait_that_is_kept(self):
        logic = self.at("follow_me", "FOLLOW")
        self.assertEqual(sent(logic.on_command({"action": "stop"}, 1.0)), ["wait"])
        self.assertEqual(self.restore(logic, 2.5, phase="FOLLOW", waiting=False), ["wait"])

    def test_returning_restores_return_or_resume(self):
        logic = self.at("go_back", "FOLLOW")
        self.assertEqual(self.restore(logic, 2.0, phase="FOLLOW"), ["return"])
        self.assertEqual(self.restore(logic, 3.5, phase="RETURN", held="silent"), ["resume"])
        self.assertEqual(self.restore(logic, 5.0, phase="RETURN", held=None), [])
        self.assertEqual(self.restore(logic, 6.5, phase="DONE"), [])  # the return finished
        self.assertEqual(self.restore(logic, 8.0, phase="IDLE"), [])  # a new run: the Pi refuses return in IDLE

    def test_resume_goes_back_to_what_was_moving(self):
        logic = self.at("go_back", "RETURN")
        logic.on_command({"action": "stop"}, 1.0)  # -> wait
        self.assertEqual(sent(logic.on_command({"action": "resume"}, 2.0)), ["resume"])
        self.assertEqual(self.restore(logic, 3.5, phase="RETURN", waiting=False, held=None), [])
        self.assertEqual(self.restore(logic, 5.0, phase="RETURN", waiting=True, held="wait"), ["resume"])

    def test_after_end_nothing_is_restored_and_stop_never_resent(self):
        logic = self.at("follow_me", "FOLLOW")
        self.assertEqual(sent(logic.on_command({"action": "end"}, 1.0)), ["stop"])
        for t in (2.5, 4.0, 6.0):
            self.assertEqual(self.restore(logic, t, phase="FOLLOW", held="silent"), [])

    def test_nothing_asked_nothing_sent(self):
        logic = connected(0.0)
        self.assertEqual(self.restore(logic, 2.0, phase="IDLE"), [])
        self.assertEqual(self.restore(logic, 3.0, phase="FOLLOW", held="silent"), [])


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


class SearchStatusTests(unittest.TestCase):
    """/base/status "search": the Pi's brain is looking for a lost operator (hmc_flow asks them back after it)."""

    def search(self, logic, now, **kw):
        return statuses(logic.on_status(status(**kw), now))[0]["search"]

    def test_following_unlocked_for_the_brains_lost_after_is_a_search(self):
        logic = connected()
        self.assertFalse(self.search(logic, 0.0, locked=True))
        self.assertFalse(self.search(logic, 1.0, locked=False))
        self.assertFalse(self.search(logic, 2.9, locked=False))
        self.assertTrue(self.search(logic, 3.0, locked=False))  # 2 s, hmc follower.LOST_AFTER
        self.assertFalse(self.search(logic, 4.0, locked=True))

    def test_waiting_held_or_not_following_is_no_search(self):
        logic = connected()
        self.search(logic, 0.0, locked=False)
        self.assertFalse(self.search(logic, 5.0, locked=False, waiting=True))
        self.assertFalse(self.search(logic, 5.2, locked=False, held="silent"))
        self.assertFalse(self.search(logic, 5.4, locked=False, phase="RETURN"))
        self.assertTrue(self.search(logic, 5.6, locked=False))

    def test_the_pis_own_mode_wins_when_it_sends_one(self):
        logic = connected()
        self.assertTrue(self.search(logic, 0.0, locked=False, mode="search"))
        self.assertFalse(self.search(logic, 5.0, locked=False, mode="follow"))


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
        self.assertEqual(sock2.cmds(), [])
        sock2.feed(status(phase="FOLLOW"))
        link.step(0.9)
        self.assertEqual(sock2.cmds(), ["return"])  # the failed command is restored on the new link

    def test_wait_lost_in_a_dying_link_is_restored_after_reconnect(self):
        sock, sock2 = FakeSocket(), FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock, sock2]))
        link.step(0.0)
        link.command({"action": "follow_me"}, 0.0)
        sock.feed(status(phase="FOLLOW"))
        link.step(0.1)
        link.command({"action": "wait"}, 0.2)  # sendall succeeds, but the Pi never reads it
        self.assertEqual(sock.cmds(), ["follow", "wait"])
        sock.eof = True
        link.step(0.3)
        self.assertIsNone(link.sock)
        link.step(0.9)
        self.assertIs(link.sock, sock2)
        sock2.feed(status(phase="FOLLOW", waiting=False, held="silent"))  # latched by silence, not waiting
        link.step(1.0)
        self.assertEqual(sock2.cmds(), ["wait"])  # never "resume": the flow wants it to wait

    def test_following_resumed_after_reconnect(self):
        sock, sock2 = FakeSocket(), FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock, sock2]))
        link.step(0.0)
        link.command({"action": "follow_me"}, 0.0)
        sock.feed(status(phase="FOLLOW"))
        link.step(0.1)
        sock.eof = True
        link.step(0.3)
        link.step(0.9)
        sock2.feed(status(phase="FOLLOW", held="silent"))
        link.step(1.0)
        self.assertEqual(sock2.cmds(), ["resume"])

    def test_error_reply_from_the_pi_is_logged_and_the_link_stays(self):
        sock = FakeSocket()
        link = PiLink(BridgeLogic(), FakeConnector([sock]))
        link.step(0.0)
        sock.feed(status(phase="IDLE", locked=False))
        link.step(0.1)
        link.command({"action": "go_back"}, 0.2)
        sock.feed({"error": "return refused in IDLE"})
        out = link.step(0.3)
        self.assertIs(link.sock, sock)
        self.assertTrue(any(k == "log" and "return refused in IDLE" in v for k, v in out))
        self.assertEqual(statuses(out)[0]["error"], "return refused in IDLE")
        sock.feed(status(phase="IDLE", locked=False))
        link.step(0.4)
        self.assertEqual(sock.cmds(), ["return"])  # refused in IDLE: not hammered again

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


class HoldTests(unittest.TestCase):
    """A hold from any source wins: the bridge never sends resume or follow over one it did not cause."""

    def test_page_wait_while_following_is_never_resumed(self):
        logic = connected(0.0, phase="IDLE")
        logic.on_command({"action": "follow_me"}, 0.0)
        out = logic.on_status(status(phase="FOLLOW", waiting=True, held="page"), 2.0)
        self.assertEqual(sent(out), [])
        self.assertTrue(any(k == "log" and "page" in v for k, v in out))
        out = logic.on_status(status(phase="FOLLOW", waiting=True, held="page"), 4.0)
        self.assertEqual(out[-1:], [("status", dict(status(phase="FOLLOW", waiting=True, held="page"), link=True,
                                                    search=False))])  # logged once
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW"), 6.0)), [])  # page resume: agrees

    def test_page_wait_while_returning_or_idle_is_never_resumed_or_followed(self):
        logic = connected(0.0, phase="IDLE")
        logic.on_command({"action": "go_back"}, 0.0)
        self.assertEqual(sent(logic.on_status(status(phase="RETURN", waiting=True, held="page"), 2.0)), [])
        logic = connected(0.0, phase="IDLE")
        logic.on_command({"action": "follow_me"}, 0.0)
        self.assertEqual(sent(logic.on_status(status(phase="IDLE", held="page"), 2.0)), [])

    def test_page_resume_while_waiting_is_adopted_not_corrected(self):
        logic = connected(0.0, phase="IDLE")
        logic.on_command({"action": "follow_me"}, 0.0)
        logic.on_command({"action": "wait"}, 0.5)
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=True, held="wait"), 1.0)), [])
        out = logic.on_status(status(phase="FOLLOW", waiting=False, held=None), 3.0)
        self.assertEqual(sent(out), [])
        self.assertEqual(logic.intent, "following")
        self.assertTrue(any(k == "log" and "page" in v for k, v in out))
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=True, held="page"), 5.0)), [])

    def test_a_wait_not_seen_yet_is_resent_not_adopted(self):
        logic = connected(0.0, phase="IDLE")
        logic.on_command({"action": "follow_me"}, 0.0)
        logic.on_command({"action": "wait"}, 0.5)
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=False), 2.0)), ["wait"])
        self.assertEqual(logic.intent, "waiting")

    def test_after_a_reconnect_a_moving_pi_is_put_back_to_wait(self):
        logic = connected(0.0, phase="IDLE")
        logic.on_command({"action": "wait"}, 0.0)
        logic.on_status(status(phase="FOLLOW", waiting=True, held="wait"), 0.5)
        logic.on_disconnected(1.0)
        logic.on_connected(2.0)
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=False), 2.1)), ["wait"])

    def test_stop_right_after_follow_me_is_a_wait(self):
        logic = connected(0.0, phase="IDLE")
        cmds = sent(logic.on_command({"action": "follow_me"}, 0.0))
        cmds += sent(logic.on_command({"action": "stop"}, 0.1))
        for t, s in ((0.3, status(phase="IDLE")), (1.5, status(phase="IDLE")), (3.0, status(phase="FOLLOW", waiting=True))):
            cmds += sent(logic.on_status(s, t))
        self.assertEqual(cmds, ["follow", "wait"])
        self.assertEqual(logic.intent, "waiting")

    def test_stop_while_link_down_after_follow_me_never_follows(self):
        logic = BridgeLogic()
        cmds = sent(logic.on_command({"action": "follow_me"}, 0.0))
        cmds += sent(logic.on_command({"action": "stop"}, 0.1))
        logic.on_connected(1.0)
        for t in (1.1, 2.5, 4.0):
            cmds += sent(logic.on_status(status(phase="IDLE"), t))
        self.assertEqual(cmds, [])
        self.assertEqual(logic.intent, "waiting")


class RunEndTests(unittest.TestCase):
    """Nothing automatic starts motion the operator did not ask for in this run of the Pi."""

    def follow(self):
        logic = connected(0.0, phase="IDLE")
        self.assertEqual(sent(logic.on_command({"action": "follow_me"}, 0.0)), ["follow"])
        for t in (0.5, 1.0):
            logic.on_status(status(phase="FOLLOW", locked=False), t)
        return logic

    def test_done_then_a_new_run_on_a_new_connection_never_follows(self):
        logic = self.follow()
        out = logic.on_status(status(phase="DONE", locked=False), 2.0)
        self.assertEqual((sent(out), events(out)), ([], ["base_run_ended"]))
        logic.on_disconnected(3.0)
        logic.on_connected(4.0)
        cmds, evs = [], []
        for t in (4.1, 5.5, 7.0):
            out = logic.on_status(status(phase="IDLE", locked=False), t)
            cmds, evs = cmds + sent(out), evs + events(out)
        self.assertEqual((cmds, logic.intent), ([], "idle"))
        self.assertNotIn("base_run_ended", evs)  # said once

    def test_run_ended_by_time_while_the_link_was_down_never_follows(self):
        logic = self.follow()
        logic.on_disconnected(1.5)
        logic.on_connected(5.0)
        out = logic.on_status(status(phase="IDLE", locked=False), 5.1)
        self.assertEqual((sent(out), "base_run_ended" in events(out)), ([], True))
        out = logic.on_status(status(phase="IDLE", locked=False), 6.5)
        self.assertEqual((sent(out), events(out)), ([], []))

    def test_idle_after_moving_on_the_same_link_is_a_new_run(self):
        logic = self.follow()
        out = logic.on_status(status(phase="IDLE", locked=False), 2.0)
        self.assertEqual((sent(out), events(out), logic.intent), ([], ["base_run_ended"], "idle"))

    def test_a_new_follow_me_after_a_run_end_follows(self):
        logic = self.follow()
        logic.on_status(status(phase="DONE", locked=False), 2.0)
        self.assertEqual(sent(logic.on_command({"action": "follow_me"}, 3.0)), ["follow"])
        self.assertEqual(sent(logic.on_status(status(phase="IDLE", locked=False), 4.5)), ["follow"])  # lost: same link

    def test_stop_before_any_status_on_this_link_is_kept_as_wait(self):
        for connected_first in (True, False):
            logic = BridgeLogic()
            if connected_first:
                logic.on_connected(0.0)
            out = logic.on_command({"action": "stop"}, 0.1)
            self.assertEqual((sent(out), logic.intent), ([], "waiting"), connected_first)
            logic.on_connected(1.0)
            self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=False), 1.5)), ["wait"])

    def test_stop_before_any_status_is_nothing_for_an_idle_pi(self):
        logic = BridgeLogic()
        logic.on_command({"action": "stop"}, 0.0)
        logic.on_connected(1.0)
        self.assertEqual(sent(logic.on_status(status(phase="IDLE"), 1.5)), [])

    def test_a_page_hold_before_the_flow_wait_is_not_its_ack(self):
        logic = self.follow()
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=True, held="page"), 1.5)), [])
        self.assertEqual(sent(logic.on_command({"action": "stop"}, 1.6)), ["wait"])
        self.assertEqual(sent(logic.on_status(status(phase="FOLLOW", waiting=True, held="page"), 1.8)), [])
        out = logic.on_status(status(phase="FOLLOW", waiting=False, held=None), 2.7)  # Resume on the page
        self.assertEqual((sent(out), logic.intent), (["wait"], "waiting"))

    def test_the_wait_ack_counts_as_seen(self):
        logic = self.follow()
        logic.on_command({"action": "wait"}, 1.5)
        logic.on_status(status(phase="FOLLOW", waiting=True, held="page", ack="wait"), 1.7)
        out = logic.on_status(status(phase="FOLLOW", waiting=False, held=None), 3.0)
        self.assertEqual((sent(out), logic.intent), ([], "following"))

    def test_a_wait_ack_counts_only_from_the_client_when_ack_by_is_given(self):
        for ack_by, cmds, intent in (("page", ["wait"], "waiting"), ("client", [], "following")):
            logic = self.follow()
            logic.on_command({"action": "wait"}, 1.5)
            logic.on_status(status(phase="FOLLOW", waiting=True, held="page", ack="wait", ack_by=ack_by), 1.7)
            out = logic.on_status(status(phase="FOLLOW", waiting=False, held=None), 3.0)
            self.assertEqual((sent(out), logic.intent), (cmds, intent), ack_by)

    def test_resume_without_a_run_sends_resume_only_and_never_follow(self):
        logic = connected(0.0, phase="IDLE")  # the bridge just started: nothing was moving
        self.assertEqual(sent(logic.on_command({"action": "resume"}, 1.0)), ["resume"])  # the Pi refuses it in IDLE
        self.assertIsNone(logic.intent)
        for t in (2.5, 4.0):
            self.assertEqual(sent(logic.on_status(status(phase="IDLE"), t)), [])
        logic = self.follow()
        logic.on_status(status(phase="DONE", locked=False), 2.0)  # base_run_ended
        self.assertEqual(sent(logic.on_command({"action": "resume"}, 3.0)), ["resume"])
        self.assertEqual(logic.intent, "idle")
        for t in (4.5, 6.0):
            self.assertEqual(sent(logic.on_status(status(phase="IDLE"), t)), [])

    def test_resume_while_the_pi_is_idle_keeps_the_intent(self):
        logic = self.follow()
        logic.on_command({"action": "wait"}, 1.2)
        logic.on_disconnected(1.5)
        logic.on_connected(2.0)
        logic.on_status(status(phase="IDLE"), 2.1)  # a new run, never seen moving on this link
        self.assertEqual(sent(logic.on_command({"action": "resume"}, 2.5)), ["resume"])
        self.assertNotEqual(logic.intent, "following")
        self.assertEqual(sent(logic.on_status(status(phase="IDLE"), 4.0)), [])


class FakeHold:
    def __init__(self):
        self.asked = 0

    def poll(self):
        asked, self.asked = self.asked, 0
        return asked


class HoldRequestTests(unittest.TestCase):
    """The agent asks the bridge to hold a moving Pi before it stops the flow or the bridge."""

    def make(self, phase):
        sock, hold = FakeSocket(), FakeHold()
        link = PiLink(BridgeLogic(), FakeConnector([sock]), hold=hold)
        link.step(0.0)
        link.command({"action": "follow_me"}, 0.0)
        sock.feed(status(phase=phase, locked=False))
        link.step(0.1)
        return link, sock, hold

    def test_hold_while_moving_sends_wait(self):
        for phase in ("FOLLOW", "RETURN"):
            link, sock, hold = self.make(phase)
            hold.asked = 1
            out = link.step(0.2)
            self.assertEqual(sock.cmds(), ["follow", "wait"], phase)
            self.assertEqual(link.logic.intent, "waiting")
            self.assertTrue(any(k == "log" and "hold" in v for k, v in out))

    def test_hold_while_idle_sends_nothing(self):
        link, sock, hold = self.make("DONE")
        hold.asked = 1
        out = link.step(0.2)
        self.assertEqual(sock.cmds(), ["follow"])
        self.assertTrue(any(k == "log" and "hold" in v for k, v in out))

    def test_hold_listener_takes_datagrams_on_localhost(self):
        listener = base_bridge.HoldListener(0)
        self.addCleanup(listener.close)
        self.assertEqual(listener.poll(), 0)
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(b"hold", ("127.0.0.1", listener.port))
            s.sendto(b"other", ("127.0.0.1", listener.port))
        self.assertTrue(wait_until(lambda: listener.poll() == 1))
        self.assertEqual(listener.poll(), 0)


def wait_until(check, timeout=2.0):
    import time
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.01)
    return False


class BackupLinkTests(unittest.TestCase):
    """PiLink with connectors in priority order: [cable, wifi]."""

    def make(self, cable_plan, wifi_plan):
        self.cable, self.wifi = FakeConnector(cable_plan), FakeConnector(wifi_plan)
        return PiLink(BridgeLogic(), [self.cable, self.wifi])

    def run_link(self, link, start, end, watch):
        """Steps every 0.5 s with a status from whichever link is up; the times `watch` was called."""
        tries, out = [], []
        for i in range(int(round((end - start) / 0.5)) + 1):
            t = round(start + i * 0.5, 2)
            if link.sock is not None:
                link.sock.feed(status())
            before = watch.calls
            out += link.step(t)
            if watch.calls > before:
                tries.append(t)
        return tries, out

    def test_primary_refused_uses_the_backup_one_host_per_step(self):
        wifi = FakeSocket()
        link = self.make([None], [wifi])
        link.step(0.0)
        self.assertEqual((self.cable.calls, self.wifi.calls), (1, 0))  # one connect per step
        link.step(0.05)
        self.assertIs(link.sock, wifi)
        self.assertEqual((self.cable.calls, self.wifi.calls), (1, 1))

    def test_primary_up_is_used_first(self):
        cable = FakeSocket()
        link = self.make([cable], [FakeSocket()])
        link.step(0.0)
        self.assertIs(link.sock, cable)
        self.assertEqual(self.wifi.calls, 0)

    def test_all_refused_is_one_link_lost_and_backoff(self):
        link = self.make([None, None], [None, None])
        self.assertEqual(events(link.step(0.0)), [])
        self.assertEqual(events(link.step(0.05)), ["base_link_lost"])
        self.assertEqual((self.cable.calls, self.wifi.calls), (1, 1))
        link.step(0.3)  # inside the 0.5 s backoff
        self.assertEqual((self.cable.calls, self.wifi.calls), (1, 1))

    def test_connect_timeouts_keep_one_step_short(self):
        self.assertLessEqual(base_bridge.CONNECT_TIMEOUT_S, 0.3)
        self.assertLessEqual(base_bridge.PROBE_TIMEOUT_S, 0.3)

    def test_primary_is_tried_again_every_10_s_and_taken_back(self):
        wifi, cable = FakeSocket(), FakeSocket()
        link = self.make([None, None, cable], [wifi])
        tries, out = self.run_link(link, 0.0, 20.5, self.cable)
        self.assertIs(link.sock, cable)
        self.assertTrue(wifi.closed)
        self.assertEqual(len(cable.pings()), 1)  # the Pi hears the new link at once
        more, out2 = self.run_link(link, 21.0, 40.0, self.cable)
        self.assertEqual(tries + more, [0.0, 10.5, 20.5])  # refused at 10.5 s, taken at 20.5 s, then no more tries
        self.assertEqual(events(out + out2), ["base_link_ok", "see_person"])  # a switch is not a link loss, nor a new lock
        self.assertEqual(self.wifi.calls, 1)

    def test_switch_back_restores_the_intent_on_the_primary(self):
        wifi, cable = FakeSocket(), FakeSocket()
        link = self.make([None, cable], [wifi])
        link.step(0.0)
        link.step(0.05)
        link.command({"action": "follow_me"}, 0.1)
        self.assertEqual(wifi.cmds(), ["follow"])
        wifi.feed(status(phase="FOLLOW"))
        link.step(9.9)
        link.command({"action": "wait"}, 9.95)  # may be lost when the Pi drops the wifi client
        link.step(10.1)
        self.assertIs(link.sock, cable)
        self.assertEqual(cable.cmds(), [])  # nothing blind: the first status decides
        cable.feed(status(phase="FOLLOW", waiting=False))
        link.step(10.15)
        self.assertEqual(cable.cmds(), ["wait"])

    def test_reconnect_to_a_backup_restores_the_intent(self):
        cable, wifi = FakeSocket(), FakeSocket()
        link = self.make([cable, None], [wifi])
        link.step(0.0)
        self.assertIs(link.sock, cable)
        link.command({"action": "follow_me"}, 0.0)
        cable.feed(status(phase="FOLLOW"))
        link.step(0.1)
        cable.eof = True  # the cable is pulled
        out = link.step(0.2)
        self.assertIn("base_link_lost", events(out))
        link.step(0.8)
        self.assertIs(link.sock, wifi)
        wifi.feed(status(phase="FOLLOW", held="silent"))
        out = link.step(0.9)
        self.assertIn("base_link_ok", events(out))
        self.assertEqual(wifi.cmds(), ["resume"])

    def test_a_host_lost_soon_after_connecting_is_skipped_for_30_s(self):
        cable, cable2, wifi = FakeSocket(), FakeSocket(), FakeSocket()
        link = self.make([cable, cable2], [wifi])
        link.step(0.0)
        self.assertIs(link.sock, cable)
        cable.eof = True  # connects, then the data does not pass
        link.step(1.0)
        link.step(1.6)
        self.assertIs(link.sock, wifi)  # the cable is skipped, not tried first
        self.assertEqual(self.cable.calls, 1)
        tries, out = self.run_link(link, 2.0, 35.0, self.cable)
        self.assertEqual(len(tries), 1)
        self.assertGreaterEqual(tries[0], 31.0)  # not before the 30 s are over
        self.assertIs(link.sock, cable2)

    def test_a_silent_link_is_skipped_however_long_it_lasted(self):
        cable, wifi = FakeSocket(), FakeSocket()
        link = self.make([cable, FakeSocket()], [wifi])
        self.run_link(link, 0.0, 10.0, self.cable)
        self.assertIs(link.sock, cable)
        out = link.step(11.6)  # no status for 1.6 s
        self.assertIn("base_link_lost", events(out))
        link.step(12.2)
        self.assertIs(link.sock, wifi)
        self.assertEqual(self.cable.calls, 1)

    def test_a_skipped_host_is_still_tried_when_nothing_else_answers(self):
        cable, cable2 = FakeSocket(), FakeSocket()
        link = self.make([cable, cable2], [None])
        link.step(0.0)
        cable.eof = True
        link.step(0.5)
        link.step(1.0)  # wifi refused
        link.step(1.05)  # then the skipped cable
        self.assertIs(link.sock, cable2)


class HostTests(unittest.TestCase):
    def test_argument_wins_then_params(self):
        self.assertEqual(base_bridge.pi_host("10.0.0.2", {"pi_host": "10.0.0.9"}), ["10.0.0.2"])
        self.assertEqual(base_bridge.pi_host(None, {"pi_host": "10.0.0.9"}), ["10.0.0.9"])
        with self.assertRaises(SystemExit):
            base_bridge.pi_host(None, {})

    def test_argument_may_be_a_comma_list(self):
        self.assertEqual(base_bridge.pi_host("10.0.0.1, 192.168.31.21,,100.73.128.36", {"pi_host": "10.0.0.9"}),
                         ["10.0.0.1", "192.168.31.21", "100.73.128.36"])

    def test_params_primary_then_backups_without_duplicates(self):
        p = {"pi_host": "10.0.0.1", "pi_hosts_backup": ["192.168.31.21", "10.0.0.1", "100.73.128.36"]}
        self.assertEqual(base_bridge.pi_host(None, p), ["10.0.0.1", "192.168.31.21", "100.73.128.36"])
        self.assertEqual(base_bridge.pi_host("10.0.0.1,10.0.0.1", {}), ["10.0.0.1"])

    def test_committed_params_list_the_cable_first(self):
        path = os.path.join(os.path.dirname(__file__), "..", "..", "params.json")
        if not os.path.exists(path):  # the Jetson's copy of this folder has no base/params.json next to it
            self.skipTest("checks the repository's base/params.json, which is not here (%s)" % os.path.abspath(path))
        with open(path) as f:
            p = json.load(f)
        self.assertEqual(base_bridge.pi_host(None, p), ["10.0.0.1", "192.168.31.21", "100.73.128.36"])


def bag(id_, state="pointing", chosen=None, result=None):
    return {"id": id_, "seq": 1, "mode": "point", "state": state, "chosen": chosen, "result": result}


CHOSEN = {"x": 2.0, "y": 0.8, "side": "right", "say": "I will take the right bag, on your right."}


class BagTests(unittest.TestCase):
    """find_bag -> the Pi's BAG phase; its status fields -> bag_chosen, bag_reached, bag_failed."""

    def waiting(self):
        """At the car: the flow's wait taken by the Pi (FOLLOW, waiting), an earlier bag task on the status."""
        logic = connected(0.0, phase="FOLLOW")
        logic.on_command({"action": "follow_me"}, 0.0)
        logic.on_command({"action": "wait"}, 1.0)
        logic.on_status(status(phase="FOLLOW", waiting=True, held="wait", ack="wait", bag=bag(0.5, "failed",
                        result={"ok": False, "code": "NO_BAG", "message": "old"})), 1.1)
        return logic

    def test_find_bag_sends_bag_or_bag_again(self):
        logic = self.waiting()
        self.assertEqual(sent(logic.on_command({"action": "find_bag", "params": {}}, 2.0)), ["bag"])
        self.assertEqual(sent(logic.on_command({"action": "find_bag", "params": {"again": True}}, 2.1)), ["bag_again"])
        self.assertEqual(logic.intent, "waiting")  # no intent of its own

    def test_chosen_then_reached_once_each(self):
        logic = self.waiting()
        logic.on_command({"action": "find_bag", "params": {}}, 2.0)
        self.assertEqual(events(logic.on_status(status(phase="FOLLOW", waiting=True, held="wait", bag=bag(0.5, "failed",
                         result={"ok": False, "code": "NO_BAG", "message": "old"})), 2.1)), [])  # the old task
        out = logic.on_status(status(phase="BAG", waiting=True, held=None, bag=bag(2.2, "turn", CHOSEN)), 2.2)
        self.assertEqual(events(out), ["bag_chosen|RIGHT|I will take the right bag, on your right."])
        self.assertEqual(events(logic.on_status(status(phase="BAG", waiting=True, bag=bag(2.2, "far", CHOSEN)), 2.4)), [])
        done = bag(2.2, "reached", CHOSEN, {"ok": True, "code": "OK", "x": 0.3104, "y": 0.0151, "odometry_m": 0.07})
        out = logic.on_status(status(phase="FOLLOW", waiting=True, held="wait", bag=done), 30.0)
        self.assertEqual(events(out), ["bag_reached|OK|0.310,0.015"])
        self.assertEqual(events(logic.on_status(status(phase="FOLLOW", waiting=True, held="wait", bag=done), 30.2)), [])
        self.assertEqual(sent(out), [])  # BAG and after: nothing restored

    def test_a_result_in_the_first_status_and_a_single_bag(self):
        logic = self.waiting()
        logic.on_command({"action": "find_bag", "params": {}}, 2.0)
        one = dict(CHOSEN, side=None, say="I will take the bag.")
        out = logic.on_status(status(phase="FOLLOW", waiting=True, held="wait", bag=bag(
            2.1, "failed", one, {"ok": False, "code": "BLOCKED", "message": "something is in my way"})), 2.3)
        self.assertEqual(events(out), ["bag_chosen|NONE|I will take the bag.",
                                       "bag_failed|BLOCKED|something is in my way"])

    def test_no_events_for_a_bag_task_nobody_here_asked_for(self):
        logic = connected(0.0, phase="FOLLOW")
        out = logic.on_status(status(phase="BAG", locked=False, bag=bag(3.0, "turn", CHOSEN)), 3.0)
        self.assertEqual(events(out), [])

    def test_a_refused_bag_is_bag_failed_refused(self):
        logic = connected(0.0, phase="FOLLOW")
        logic.on_command({"action": "find_bag", "params": {}}, 1.0)
        out = logic.on_status({"error": "bag: only while waiting at the car", "refused": "bag"}, 1.1)
        self.assertEqual(events(out), ["bag_failed|REFUSED|bag: only while waiting at the car"])
        self.assertEqual(events(logic.on_status({"error": "not a command", "refused": "bag"}, 1.2)), [])  # once
        self.assertEqual(events(logic.on_status({"error": "not a command"}, 1.3)), [])

    def test_find_bag_with_the_link_down_fails_at_once(self):
        logic = BridgeLogic()
        out = logic.on_command({"action": "find_bag", "params": {}}, 1.0)
        self.assertEqual(sent(out), [])
        self.assertEqual(events(out), ["bag_failed|NO_LINK|the link to the base is down"])

    def test_bag_is_halted_by_stop_and_never_restored(self):
        logic = self.waiting()
        logic.on_command({"action": "find_bag", "params": {}}, 2.0)
        out = logic.on_status(status(phase="BAG", waiting=True, held=None, ack="bag", bag=bag(2.2, "far", CHOSEN)), 4.0)
        self.assertEqual(sent(out), [])  # waiting intent, the Pi moving in BAG: not a wait to restore
        self.assertEqual(sent(logic.on_command({"action": "stop"}, 4.1)), ["wait"])  # but a flow stop halts it

    def test_a_result_that_came_while_the_link_was_down_is_reported_after_it(self):
        logic = self.waiting()
        logic.on_command({"action": "find_bag", "params": {}}, 2.0)
        logic.on_status(status(phase="BAG", waiting=True, bag=bag(2.2, "far", CHOSEN)), 2.2)
        logic.on_disconnected(3.0)
        logic.on_connected(4.0)
        done = bag(2.2, "reached", CHOSEN, {"ok": True, "code": "OK", "x": 0.31, "y": 0.02})
        out = logic.on_status(status(phase="FOLLOW", locked=False, waiting=True, held="wait", bag=done), 4.1)
        self.assertEqual(events(out), ["base_link_ok", "bag_reached|OK|0.310,0.020"])

    def test_a_hold_request_halts_the_bag_phase(self):
        sock, hold = FakeSocket(), FakeHold()
        link = PiLink(BridgeLogic(), FakeConnector([sock]), hold=hold)
        link.step(0.0)
        sock.feed(status(phase="BAG", locked=False))
        link.step(0.1)
        hold.asked = 1
        link.step(0.2)
        self.assertEqual(sock.cmds(), ["wait"])


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
