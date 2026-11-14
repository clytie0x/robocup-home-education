"""Jetson <-> Pi command bridge for Help Me Carry (decision D3 of
docs/superpowers/specs/2026-11-14-hmc-integration-design.md). A ROS 2 Jazzy node (domain 42) that is a thin
shell around BridgeLogic (pure, no sockets, no ROS) and PiLink (the socket, no ROS).

Pi link: TCP to the Pi (run_follow.py --remote), port 5604, one JSON object per line.
  bridge -> Pi  {"cmd": "follow"|"wait"|"resume"|"return"|"stop"|"ping"|"bag"|"bag_again"}; "ping" every 0.5 s
                (the Pi holds still after 2 s of silence)
  Pi -> bridge  {"t","phase": "IDLE|FOLLOW|RETURN|BAG|DONE","waiting","held": "silent"|"wait"|null,"locked",
                 "operator_m","breadcrumbs","queue_tail","at_start","stop","ack","bag"} up to 5 Hz, or {"error": ...}
                (2 s of silence latches a hold on the Pi until resume, follow or return); "bag": null or the
                running or last bag task {"id","state","chosen": {"x","y","side","say"}|null,"result"|null}
                (base/pi/bag.py); a refused bag command is answered {"error", "refused": "bag"|"bag_again"}

ROS side:
  /robot/command (JSON {"action",...})  follow_me -> follow, go_back -> return, wait -> wait,
      resume -> resume, end -> stop (the only way to send Pi "stop", which ends its run);
      stop -> wait while the Pi is in FOLLOW or RETURN, or a follow or return is asked, or the Pi's
      state on this link is not known yet (then wait goes out only if the Pi turns out to move);
      else nothing (logged). The Pi's BAG phase counts as moving for stop (and for a hold request).
      find_bag {"again": bool} -> bag (or bag_again: the last bag measured and approached once more); the
      Pi takes it only while it waits at the car (FOLLOW after wait). The intent is not changed: it stays
      waiting, BAG is no phase any intent restores, and the bag task ends in FOLLOW still waiting, so
      nothing is restored during it or after it. A lost link is handled as in the other phases (the
      flow's error); a bag command is never re-sent by the bridge. With the link down it is answered at
      once with bag_failed|NO_LINK.
      Other actions (pick_bag, drop_bag, ...) are not ours.
  /robot/event  see_person when locked becomes true, lost_person when it has been false for 0.5 s,
      at_start when at_start becomes true, base_link_lost when no status for 1.5 s or the socket
      drops, base_link_ok when statuses come back, base_run_ended when the Pi's run ended while the
      flow wanted it following, returning or waiting (DONE, or IDLE after it moved: a new run); the
      intent is then idle and nothing is restored until the flow asks again.
      For a find_bag sent by this bridge, once per bag task (the status "bag" id differs from the one
      seen when it was sent): bag_chosen|SIDE|say text (SIDE LEFT, RIGHT, MIDDLE or NONE for one bag),
      then bag_reached|CODE|x,y (CODE OK, or ESTIMATED when the LiDAR never saw the bag; robot frame,
      metres, 3 decimals) or bag_failed|CODE|message (NOT_POINTED, NO_BAG, BLOCKED, TIMEOUT, LOST_BAG,
      CANCELLED; REFUSED when the Pi refused the command).
  /base/status  every Pi status as JSON, plus "link": true; {"link": false} when the link is lost.

The bridge keeps the intent, what the flow last asked (following, waiting, returning, idle after end).
A sent line is not proof it arrived, so every status is compared with the intent; after each
(re)connect and whenever they disagree the restoring command goes out, at most once a second:
following -> follow (Pi IDLE, only for a follow asked on this link and not seen moving yet) or
resume (held or waiting); returning -> return (Pi FOLLOW) or resume (held or waiting); waiting ->
wait (moving, not waiting). "stop" is never re-sent, and
nothing is sent blind on connect: the first status decides. A Pi {"error"} reply is logged only.
A hold from any source wins: resume or follow is never sent while the Pi is held by its own page
(held "page"), and when the Pi took the flow's wait and then moves with nothing holding it (Resume
on the page), the bridge adopts that as the intent instead of sending wait again. The wait counts
as taken only from held "wait" or ack "wait", never from a page hold that was there before it. Flow "stop"
while a follow or return is asked becomes wait even before the Pi reports moving. Only follow_me
starts a run: resume with nothing moving (bridge just started, after base_run_ended, Pi IDLE or
DONE) is sent as it is and never becomes a following intent. An ack counts as the wait taken only
when it came from the client (ack_by "client", or no ack_by from an older Pi).
Hold port: UDP 127.0.0.1:5605, b"hold" from agent.py before it stops the flow or the bridge: a Pi
last seen in FOLLOW or RETURN gets wait (a flow wait), any other is left alone.
Reconnects after 0.5, 1, 2, 4, 5, 5 ... s; a received status resets the wait to 0.5 s.

Pi addresses in priority order (cable, Wi-Fi, Tailscale): one address is tried per step (at most
0.3 s on the ROS timer), the first that answers is used. An address whose link failed within 5 s
of connecting, or went silent, is tried last for 30 s. On a backup, every 10 s a short connect to
the primary is tried; when it answers it becomes the link and the backup is closed. The Pi's
command server takes the new client after reading the old one's lines, and the switch counts as a reconnect for the intent (restored at the first status), so
a command lost on the way is restored. It is not a link loss: no event goes to ROS.

Run: source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 base_bridge.py [--pi-host IP[,IP...]]
     (the hosts default to "pi_host" then "pi_hosts_backup" in base/params.json)
"""

import argparse
import json
import socket
import time
from pathlib import Path

PORT = 5604
HOLD_PORT = 5605  # UDP on 127.0.0.1: agent.py asks for a hold here (see HoldListener)
PING_S = 0.5
LOST_DEBOUNCE_S = 0.5
LINK_TIMEOUT_S = 1.5
MIN_BACKOFF_S = 0.5
MAX_BACKOFF_S = 5.0
CONNECT_TIMEOUT_S = 0.3  # one address per step, so a step blocks the ROS timer at most this long
RETRY_PRIMARY_S = 10.0  # while on a backup address, how often the primary is tried again
PROBE_TIMEOUT_S = 0.2  # that try blocks the ROS timer at most this long (the cable answers in ~1 ms)
BAD_WITHIN_S = 5.0  # a link lost this soon after connecting (or gone silent) marks its address bad
BAD_FOR_S = 30.0  # a bad address is tried only after the others, and not tried again from a backup
MAX_LINE = 65536
EPS = 1e-6
RESEND_S = 1.0  # a restoring command is sent at most once a second (statuses come at 5 Hz)
ACTION_TO_CMD = {"follow_me": "follow", "go_back": "return", "wait": "wait", "resume": "resume", "end": "stop"}
MOVING = ("FOLLOW", "RETURN")
HALTABLE = MOVING + ("BAG",)  # phases a flow stop (or a hold request) halts with wait; BAG restores nothing
BAG_CMDS = ("bag", "bag_again")


class BridgeLogic:
    """Pure link logic driven by a clock passed in. Every method returns a list of
    ("send", {"cmd": ...}) | ("event", name) | ("status", dict) | ("drop", reason) | ("log", text)."""

    def __init__(self, ping_s=PING_S, lost_debounce_s=LOST_DEBOUNCE_S, link_timeout_s=LINK_TIMEOUT_S):
        self.ping_s, self.lost_debounce_s, self.link_timeout_s = ping_s, lost_debounce_s, link_timeout_s
        self.connected = False
        self.link_ok = None  # None = not known yet
        self.last_heard = 0.0
        self.last_ping = None
        self.intent = None  # what the flow last asked: following | waiting | returning | idle (after end)
        self.moving = None  # following | returning: what "resume" goes back to
        self.wait_seen = False  # the Pi showed the wait on this link (waiting, or held by wait or page)
        self.page_logged = False  # "held by its page" said once per page hold
        self.intent_at = None  # when the flow last set the intent
        self.link_since = None  # when this link connected
        self.status_seen = False  # a status arrived on this link
        self.moved = False  # the Pi was seen in FOLLOW or RETURN since the flow's last follow_me or go_back
        self.last_sent_at = None  # last follow/wait/resume/return/stop on this link, for the resend gap
        self.phase = None
        self.locked = False  # what the Pi said last
        self.reported_locked = False  # what we told ROS last
        self.unlocked_since = None
        self.at_start = False
        self.retry_delay = MIN_BACKOFF_S
        self.next_connect_at = 0.0
        self.bag_id = None  # the id of the newest bag task seen in a status
        self.bag_asked = None  # (bag_id seen when a find_bag went out,) while its result is not reported yet
        self.bag_chosen_said = None  # the bag id whose bag_chosen went out

    # ---------------- connection ----------------
    def should_connect(self, now):
        return not self.connected and now >= self.next_connect_at - EPS

    def on_connected(self, now):
        self.connected = True
        self.last_heard = now  # the link timeout counts from here until the first status
        self.last_ping = None
        self.last_sent_at = None  # what went into the old link may be lost: restore at the first status
        self.wait_seen = False  # a Pi moving on a new link is put back to wait, never taken as a page resume
        self.link_since, self.status_seen = now, False
        return [("log", "connected to the Pi")]

    def _backoff(self, now):
        self.next_connect_at = now + self.retry_delay
        self.retry_delay = min(self.retry_delay * 2, MAX_BACKOFF_S)

    def _link_lost(self, why):
        self.reported_locked = False
        self.unlocked_since = None
        if self.link_ok is False:
            return []
        self.link_ok = False
        return [("event", "base_link_lost"), ("status", {"link": False}), ("log", "link lost: " + why)]

    def on_connect_failed(self, now, why="connection refused"):
        self._backoff(now)
        return self._link_lost(why)

    def on_disconnected(self, now, why="socket closed"):
        self.connected = False
        self.last_sent_at = None
        self._backoff(now)
        return self._link_lost(why)

    def on_send_failed(self, message, now):
        """The socket broke while sending. The intent is kept and restored on the next link."""
        return self.on_disconnected(now, "send failed")

    # ---------------- inputs ----------------
    def on_command(self, command, now):
        action = command.get("action")
        if action == "find_bag":
            return self._find_bag(command)
        if action == "stop":
            # IDLE or DONE seen on this link and nothing asked to move: Pi "stop" would end its run. A
            # follow or return asked but not seen yet, or a Pi not heard yet on this link, is halted all
            # the same (a wait the restore sends only if the Pi turns out to move).
            known = self.connected and self.status_seen
            if known and self.phase not in HALTABLE and self.intent not in ("following", "returning"):
                return [("log", "stop with the Pi in %s: nothing to halt" % self.phase)]
            if self.connected and not known:
                self.intent, self.wait_seen, self.intent_at = "waiting", False, now
                return [("log", "stop before the Pi's state is known: wait goes out if it moves")]
            action = "wait"
        cmd = ACTION_TO_CMD.get(action)
        if cmd is None:
            return []
        if action == "resume" and (self.moving is None or self.phase in ("IDLE", "DONE")):
            # Only follow_me starts a run: resume goes out as asked (the Pi refuses it in IDLE), and
            # the intent and its age stay, so nothing restores a follow from it.
            if not self.connected:
                return [("log", "link down, resume not sent (nothing to resume)")]
            return [("send", {"cmd": "resume"}), ("log", "resume -> resume (Pi in %s, nothing moving: intent kept)" % self.phase)]
        self.intent_at = now
        if action == "follow_me":
            self.intent = self.moving = "following"
            self.moved = False
        elif action == "go_back":
            self.intent = self.moving = "returning"
            self.moved = False
        elif action == "wait":
            self.intent = "waiting"
            self.wait_seen = False
        elif action == "resume":
            self.intent = self.moving
        else:  # end
            self.intent = "idle"
        if not self.connected:
            note = {"stop": "not sent", "follow": "never sent to an IDLE Pi later: ask again",
                    "return": "sent later only to a Pi in FOLLOW"}.get(cmd, "restored when it is back")
            return [("log", "link down, %s %s" % (cmd, note))]
        self.last_sent_at = now
        return [("send", {"cmd": cmd}), ("log", "%s -> %s" % (command.get("action"), cmd))]

    def _find_bag(self, command):
        """find_bag -> bag or bag_again as it is: no intent, never restored (a lost one: the flow times out)."""
        params = command.get("params") or {}
        cmd = "bag_again" if params.get("again") else "bag"
        if not self.connected:
            return [("event", "bag_failed|NO_LINK|the link to the base is down"),
                    ("log", "link down, %s not sent" % cmd)]
        self.bag_asked = (self.bag_id,)
        return [("send", {"cmd": cmd}), ("log", "find_bag -> %s" % cmd)]

    def _bag_events(self, status):
        """bag_chosen, then bag_reached or bag_failed, once per bag task that a find_bag of ours asked for."""
        if "error" in status:
            if status.get("refused") in BAG_CMDS and self.bag_asked is not None:
                self.bag_asked = None
                return [("event", "bag_failed|REFUSED|%s" % status["error"])]
            return []
        bag = status.get("bag")
        if not isinstance(bag, dict):
            return []
        self.bag_id = bag.get("id")
        if self.bag_asked is None or bag.get("id") == self.bag_asked[0]:
            return []
        out = []
        chosen, result = bag.get("chosen"), bag.get("result")
        if isinstance(chosen, dict) and self.bag_chosen_said != bag.get("id"):
            self.bag_chosen_said = bag.get("id")
            out.append(("event", "bag_chosen|%s|%s" % (str(chosen.get("side") or "none").upper(), chosen.get("say", ""))))
        if isinstance(result, dict):
            self.bag_asked = None
            if result.get("ok"):
                out.append(("event", "bag_reached|%s|%.3f,%.3f" % (result.get("code", "OK"), result["x"], result["y"])))
            else:
                out.append(("event", "bag_failed|%s|%s" % (result.get("code", "FAILED"), result.get("message", ""))))
        return out

    def _restore_cmd(self, status):
        """The command that puts the Pi back into the intent, or None when it agrees (never "stop")."""
        phase, held = status.get("phase"), status.get("held")
        waiting = bool(status.get("waiting"))
        if self.intent == "following":
            if phase == "IDLE" and not self.moved and self.intent_at is not None                     and self.intent_at >= self.link_since - EPS:
                return "follow"  # a follow lost on this link; an older one may belong to another run
            if phase == "FOLLOW" and (waiting or held):
                return "resume"
        elif self.intent == "returning":
            if phase == "FOLLOW":
                return "return"
            if phase == "RETURN" and (waiting or held):
                return "resume"  # IDLE refuses return; DONE means the return is over
        elif self.intent == "waiting":
            if phase in MOVING and not waiting and held != "wait":
                return "wait"
        return None

    def _restore(self, status, now):
        phase, held, waiting = status.get("phase"), status.get("held"), bool(status.get("waiting"))
        if held != "page":
            self.page_logged = False
        if self.intent == "waiting":
            # A page hold may predate the flow's wait; an ack counts only from the client (no ack_by: old Pi).
            if held == "wait" or (status.get("ack") == "wait" and status.get("ack_by", "client") == "client"):
                self.wait_seen = True
            elif phase in MOVING and not held and self.wait_seen:
                # The Pi took the wait and now moves with nothing holding it: Resume on the Pi's page.
                self.intent = self.moving = "following" if phase == "FOLLOW" else "returning"
                return [("log", "Pi resumed from its page in %s: flow intent is now %s" % (phase, self.intent))]
        cmd = self._restore_cmd(status)
        if cmd in ("resume", "follow") and held == "page":  # a hold the bridge did not cause wins
            if self.page_logged:
                return []
            self.page_logged = True
            return [("log", "Pi held by its page in %s, flow wants %s: %s not sent" % (phase, self.intent, cmd))]
        if cmd is None or (self.last_sent_at is not None and now - self.last_sent_at < RESEND_S - EPS):
            return []
        self.last_sent_at = now
        return [("send", {"cmd": cmd}),
                ("log", "Pi in %s (waiting %s, held %s), flow wants %s: %s" % (
                    status.get("phase"), status.get("waiting"), status.get("held"), self.intent, cmd))]

    def on_status(self, status, now):
        self.last_heard = now
        self.retry_delay = MIN_BACKOFF_S
        out = []
        if self.link_ok is not True:
            self.link_ok = True
            out.append(("event", "base_link_ok"))
        if "error" in status:
            return out + [("status", dict(status, link=True)), ("log", "Pi error: %s" % status["error"])] + \
                self._bag_events(status)
        self.phase = status.get("phase")
        self.locked = bool(status.get("locked"))
        if self.locked:
            self.unlocked_since = None
            if not self.reported_locked:
                self.reported_locked = True
                out.append(("event", "see_person"))
        elif self.unlocked_since is None:
            self.unlocked_since = now
        out += self._debounce(now)
        at_start = bool(status.get("at_start"))
        if at_start and not self.at_start:
            out.append(("event", "at_start"))
        self.at_start = at_start
        out.append(("status", dict(status, link=True)))
        self.status_seen = True
        return out + self._bag_events(status) + self._run_end(self.phase) + self._restore(status, now)

    def _run_end(self, phase):
        """DONE, or IDLE after moving: the Pi's run is over, so the flow's intent is dropped."""
        if phase in MOVING:
            self.moved = True
            return []
        if not (phase == "DONE" or (phase == "IDLE" and self.moved)) or self.intent not in (
                "following", "returning", "waiting"):
            return []
        was = self.intent
        self.intent, self.moving, self.moved = "idle", None, False
        return [("event", "base_run_ended"),
                ("log", "Pi run ended (%s) while the flow wanted %s: nothing restored until asked" % (phase, was))]

    def _debounce(self, now):
        if self.reported_locked and not self.locked and self.unlocked_since is not None \
                and now - self.unlocked_since >= self.lost_debounce_s - EPS:
            self.reported_locked = False
            return [("event", "lost_person")]
        return []

    def on_tick(self, now):
        if not self.connected:
            return []
        if now - self.last_heard > self.link_timeout_s:
            return [("drop", "no status for %.1f s" % (now - self.last_heard))] + self._link_lost("Pi silent")
        out = self._debounce(now)
        if self.last_ping is None or now - self.last_ping >= self.ping_s - EPS:
            self.last_ping = now
            out.append(("send", {"cmd": "ping"}))
        return out


class PiLink:
    """The socket around BridgeLogic, stepped by the caller (the ROS timer), no threads.
    `connect` is one connector or a list in priority order; a connector returns a connected socket
    or raises OSError, and the primary's is called with timeout=PROBE_TIMEOUT_S when it is tried
    again from a backup. step() and command() return the outputs that are not sends:
    ("event", ...), ("status", ...), ("log", ...). `hold` (a HoldListener) is polled every step: a
    hold request is a flow wait when the Pi was last seen in FOLLOW or RETURN, else only logged."""

    def __init__(self, logic, connect, retry_primary_s=RETRY_PRIMARY_S, hold=None):
        self.logic = logic
        self.hold = hold
        self.connects = list(connect) if isinstance(connect, (list, tuple)) else [connect]
        self.retry_primary_s = retry_primary_s
        self.using = None  # index in connects of the link that is up
        self.connected_at = None
        self.next_primary_try = 0.0
        self.bad_until = {}  # index -> time: connected, then failed soon (TCP up, data not passing)
        self.round = []  # indices still to try in this connect round, one per step
        self.round_errors = []
        self.sock = None
        self.buffer = b""

    def command(self, command, now):
        return self._run(self.logic.on_command(command, now), now)

    def step(self, now):
        out = []
        if self.sock is None and (self.round or self.logic.should_connect(now)):
            if not self.round:  # good addresses in priority order, then the bad ones
                good = [i for i in range(len(self.connects)) if not self._bad(i, now)]
                self.round = good + [i for i in range(len(self.connects)) if i not in good]
                self.round_errors = []
            index = self.round.pop(0)
            try:
                sock = self.connects[index]()
            except OSError as error:
                self.round_errors.append(str(error))
                if not self.round:
                    out += self._run(self.logic.on_connect_failed(now, "; ".join(self.round_errors)), now)
            else:
                self.round = []
                out += self._use(sock, index, now)
        elif self.sock is not None and self.using != 0 and now >= self.next_primary_try - EPS \
                and not self._bad(0, now):
            out += self._back_to_primary(now)
        if self.sock is not None:
            out += self._read(now)
        if self.hold is not None and self.hold.poll():
            out += self._hold(now)
        if self.sock is not None:
            out += self._run(self.logic.on_tick(now), now)
        return out

    def _hold(self, now):
        phase = self.logic.phase
        if phase not in HALTABLE:
            return [("log", "hold asked (the agent stops a program): Pi in %s, nothing to do" % phase)]
        return [("log", "hold asked (the agent stops a program): Pi in %s, wait" % phase)] + \
            self.command({"action": "wait"}, now)

    def _bad(self, index, now):
        return now < self.bad_until.get(index, 0.0) - EPS

    def _use(self, sock, index, now):
        self.sock, self.using, self.buffer, self.connected_at = sock, index, b"", now
        self.next_primary_try = now + self.retry_primary_s
        out = self._run(self.logic.on_connected(now), now)
        return out if index == 0 else out + [("log", "on backup address %d" % index)]

    def _back_to_primary(self, now):
        """A short try of the primary; when it answers the backup socket is closed without a link loss."""
        self.next_primary_try = now + self.retry_primary_s
        try:
            sock = self.connects[0](timeout=PROBE_TIMEOUT_S)
        except OSError:
            return []
        try:
            self.sock.close()
        except OSError:
            pass
        return self._use(sock, 0, now) + [("log", "back on the primary address")]

    def _read(self, now):
        out = []
        while self.sock is not None:
            try:
                data = self.sock.recv(4096)
            except (BlockingIOError, socket.timeout):
                break
            except OSError as error:
                return out + self._drop(now, str(error))
            if not data:
                return out + self._drop(now, "closed by the Pi")
            self.buffer += data
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    status = json.loads(line.decode("utf-8"))
                    if not isinstance(status, dict):
                        raise ValueError("not an object")
                except (ValueError, UnicodeDecodeError):
                    out.append(("log", "bad line from the Pi: %r" % line[:80]))
                    continue
                out += self._run(self.logic.on_status(status, now), now)
            if len(self.buffer) > MAX_LINE:
                return out + self._drop(now, "line too long")
        return out

    def _drop(self, now, why, notify=True, silent=False):
        out = []
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            if silent or now - self.connected_at < BAD_WITHIN_S - EPS:
                self.bad_until[self.using] = now + BAD_FOR_S
                out.append(("log", "address %d failed after connecting: tried last for %.0f s" % (self.using, BAD_FOR_S)))
        self.sock = self.using = None
        self.buffer = b""
        return out + (self._run(self.logic.on_disconnected(now, why), now) if notify else [])

    def _run(self, outputs, now):
        out = []
        for kind, val in outputs:
            if kind == "send":
                if self.sock is None:
                    continue  # the link broke earlier in this batch; the intent is restored later
                try:
                    self.sock.sendall((json.dumps(val) + "\n").encode("utf-8"))
                except OSError:
                    out += self._drop(now, "send failed", notify=False)
                    out += self._run(self.logic.on_send_failed(val, now), now)
            elif kind == "drop":
                out.append(("log", "dropping the link: " + val))
                out += self._drop(now, val, notify=False, silent=True)
                self.logic.on_disconnected(now, val)  # already reported lost by on_tick
            else:
                out.append((kind, val))
        return out


class HoldListener:
    """UDP on 127.0.0.1:HOLD_PORT, never blocks: agent.py sends b"hold" before it stops the flow or the
    bridge, so a moving Pi gets wait at once instead of after the bridge's silence. Only a hold can be
    asked this way, so a local sender can stop the robot but never move it."""

    def __init__(self, port=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", HOLD_PORT if port is None else port))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]

    def poll(self):
        """How many hold requests arrived since the last poll."""
        asked = 0
        while True:
            try:
                data = self.sock.recv(64)
            except (BlockingIOError, InterruptedError):
                return asked
            except OSError:  # e.g. a port-unreachable report on Windows
                continue
            asked += data.strip() == b"hold"

    def close(self):
        self.sock.close()


def tcp_connector(host, port=PORT, timeout=CONNECT_TIMEOUT_S):
    def connect(timeout=timeout):
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setblocking(False)
        return sock

    return connect


def load_params():
    path = Path(__file__).resolve().parent.parent / "params.json"
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def pi_host(argument, params):
    """The Pi's addresses in priority order, each once: --pi-host (comma-separated) if given, else
    "pi_host" then "pi_hosts_backup" from base/params.json."""
    if argument:
        candidates = argument.split(",")
    else:
        candidates = [params.get("pi_host")] + list(params.get("pi_hosts_backup", []))
    hosts = []
    for host in candidates:
        host = (host or "").strip()
        if host and host not in hosts:
            hosts.append(host)
    if not hosts:
        raise SystemExit('no Pi address: pass --pi-host or add "pi_host" to base/params.json')
    return hosts


def make_bridge(host, port=PORT):
    """The rclpy node, built here so the rest of the file imports without ROS. host: one address or a
    list in priority order."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    hosts = [host] if isinstance(host, str) else list(host)

    class Bridge(Node):
        def __init__(self):
            super().__init__("base_bridge")
            try:
                hold = HoldListener()
            except OSError as error:  # the port is taken (another bridge?): run without it
                hold = None
                self.get_logger().warn("no hold port %d: %s" % (HOLD_PORT, error))
            self.link = PiLink(BridgeLogic(), [tcp_connector(h, port) for h in hosts], hold=hold)
            self.pub_event = self.create_publisher(String, "/robot/event", 10)
            self.pub_status = self.create_publisher(String, "/base/status", 10)
            self.create_subscription(String, "/robot/command", self.on_command, 10)
            self.create_timer(0.05, self.on_tick)
            self.get_logger().info("Pi link %s port %d" % (", ".join(hosts), port))

        def emit(self, outputs):
            for kind, val in outputs:
                if kind == "event":
                    self.get_logger().info("event -> %s" % val)
                    self.pub_event.publish(String(data=val))
                elif kind == "status":
                    self.pub_status.publish(String(data=json.dumps(val)))
                else:
                    self.get_logger().info(str(val))

        def on_command(self, msg):
            try:
                command = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().warn("bad command: %s" % msg.data)
                return
            if isinstance(command, dict):
                self.emit(self.link.command(command, time.monotonic()))

        def on_tick(self):
            self.emit(self.link.step(time.monotonic()))

    rclpy.init()
    return Bridge()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pi-host", default=None)
    parser.add_argument("--port", type=int, default=PORT)
    args, _ = parser.parse_known_args()  # leaves --ros-args to rclpy
    import rclpy

    node = make_bridge(pi_host(args.pi_host, load_params()), args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
