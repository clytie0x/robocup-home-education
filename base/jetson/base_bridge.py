"""Jetson <-> Pi command bridge for Help Me Carry (decision D3 of
docs/superpowers/specs/2026-11-14-hmc-integration-design.md). A ROS 2 Jazzy node (domain 42) that is a thin
shell around BridgeLogic (pure, no sockets, no ROS) and PiLink (the socket, no ROS).

Pi link: TCP to the Pi (run_follow.py --remote), port 5604, one JSON object per line.
  bridge -> Pi  {"cmd": "follow"|"wait"|"resume"|"return"|"stop"|"ping"}; "ping" every 0.5 s
                (the Pi holds still after 2 s of silence)
  Pi -> bridge  {"t","phase": "IDLE|FOLLOW|RETURN|DONE","waiting","held": "silent"|"wait"|null,"locked",
                 "operator_m","breadcrumbs","queue_tail","at_start","stop","ack"} up to 5 Hz, or {"error": ...}
                (2 s of silence latches a hold on the Pi until resume, follow or return)

ROS side:
  /robot/command (JSON {"action",...})  follow_me -> follow, go_back -> return, wait -> wait,
      resume -> resume, end -> stop (the only way to send Pi "stop", which ends its run);
      stop -> wait while the Pi is in FOLLOW or RETURN, else nothing (logged).
      Other actions (carry_bag, drop_bag, ...) are not ours.
  /robot/event  see_person when locked becomes true, lost_person when it has been false for 0.5 s,
      at_start when at_start becomes true, base_link_lost when no status for 1.5 s or the socket
      drops, base_link_ok when statuses come back.
  /base/status  every Pi status as JSON, plus "link": true; {"link": false} when the link is lost.

The bridge keeps the intent, what the flow last asked (following, waiting, returning, idle after end).
A sent line is not proof it arrived, so every status is compared with the intent; after each
(re)connect and whenever they disagree the restoring command goes out, at most once a second:
following -> follow (Pi IDLE/DONE) or resume (held or waiting); returning -> return (Pi FOLLOW) or
resume (held or waiting); waiting -> wait (moving, not waiting). "stop" is never re-sent, and
nothing is sent blind on connect: the first status decides. A Pi {"error"} reply is logged only.
Reconnects after 0.5, 1, 2, 4, 5, 5 ... s; a received status resets the wait to 0.5 s.

Run: source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 base_bridge.py [--pi-host IP]
     (the host defaults to "pi_host" in base/params.json)
"""

import argparse
import json
import socket
import time
from pathlib import Path

PORT = 5604
PING_S = 0.5
LOST_DEBOUNCE_S = 0.5
LINK_TIMEOUT_S = 1.5
MIN_BACKOFF_S = 0.5
MAX_BACKOFF_S = 5.0
CONNECT_TIMEOUT_S = 0.5
MAX_LINE = 65536
EPS = 1e-6
RESEND_S = 1.0  # a restoring command is sent at most once a second (statuses come at 5 Hz)
ACTION_TO_CMD = {"follow_me": "follow", "go_back": "return", "wait": "wait", "resume": "resume", "end": "stop"}
MOVING = ("FOLLOW", "RETURN")


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
        self.last_sent_at = None  # last follow/wait/resume/return/stop on this link, for the resend gap
        self.phase = None
        self.locked = False  # what the Pi said last
        self.reported_locked = False  # what we told ROS last
        self.unlocked_since = None
        self.at_start = False
        self.retry_delay = MIN_BACKOFF_S
        self.next_connect_at = 0.0

    # ---------------- connection ----------------
    def should_connect(self, now):
        return not self.connected and now >= self.next_connect_at - EPS

    def on_connected(self, now):
        self.connected = True
        self.last_heard = now  # the link timeout counts from here until the first status
        self.last_ping = None
        self.last_sent_at = None  # what went into the old link may be lost: restore at the first status
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
        if action == "stop":
            if self.phase not in MOVING:  # IDLE, DONE or not known: Pi "stop" would end its run
                return [("log", "stop with the Pi in %s: nothing to halt" % self.phase)]
            action = "wait"
        cmd = ACTION_TO_CMD.get(action)
        if cmd is None:
            return []
        if action == "follow_me":
            self.intent = self.moving = "following"
        elif action == "go_back":
            self.intent = self.moving = "returning"
        elif action == "wait":
            self.intent = "waiting"
        elif action == "resume":
            self.intent = self.moving or "following"
        else:  # end
            self.intent = "idle"
        if not self.connected:
            note = "not sent" if cmd == "stop" else "restored when it is back"
            return [("log", "link down, %s %s" % (cmd, note))]
        self.last_sent_at = now
        return [("send", {"cmd": cmd}), ("log", "%s -> %s" % (command.get("action"), cmd))]

    def _restore_cmd(self, status):
        """The command that puts the Pi back into the intent, or None when it agrees (never "stop")."""
        phase, held = status.get("phase"), status.get("held")
        waiting = bool(status.get("waiting"))
        if self.intent == "following":
            if phase in ("IDLE", "DONE"):
                return "follow"
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
        cmd = self._restore_cmd(status)
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
            return out + [("status", dict(status, link=True)), ("log", "Pi error: %s" % status["error"])]
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
        return out + self._restore(status, now)

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
    `connect()` returns a connected socket or raises OSError. step() and command() return the outputs
    that are not sends: ("event", ...), ("status", ...), ("log", ...)."""

    def __init__(self, logic, connect):
        self.logic = logic
        self.connect = connect
        self.sock = None
        self.buffer = b""

    def command(self, command, now):
        return self._run(self.logic.on_command(command, now), now)

    def step(self, now):
        out = []
        if self.sock is None and self.logic.should_connect(now):
            try:
                self.sock = self.connect()
            except OSError as error:
                out += self._run(self.logic.on_connect_failed(now, str(error)), now)
            else:
                self.buffer = b""
                out += self._run(self.logic.on_connected(now), now)
        if self.sock is not None:
            out += self._read(now)
        if self.sock is not None:
            out += self._run(self.logic.on_tick(now), now)
        return out

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

    def _drop(self, now, why, notify=True):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.buffer = b""
        return self._run(self.logic.on_disconnected(now, why), now) if notify else []

    def _run(self, outputs, now):
        out = []
        for kind, val in outputs:
            if kind == "send":
                if self.sock is None:
                    continue  # the link broke earlier in this batch; the intent is restored later
                try:
                    self.sock.sendall((json.dumps(val) + "\n").encode("utf-8"))
                except OSError:
                    self._drop(now, "send failed", notify=False)
                    out += self._run(self.logic.on_send_failed(val, now), now)
            elif kind == "drop":
                out.append(("log", "dropping the link: " + val))
                self._drop(now, val, notify=False)
                self.logic.on_disconnected(now, val)  # already reported lost by on_tick
            else:
                out.append((kind, val))
        return out


def tcp_connector(host, port=PORT, timeout=CONNECT_TIMEOUT_S):
    def connect():
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
    """--pi-host if given, else "pi_host" from base/params.json."""
    host = argument or params.get("pi_host")
    if not host:
        raise SystemExit('no Pi address: pass --pi-host or add "pi_host" to base/params.json')
    return host


def make_bridge(host, port=PORT):
    """The rclpy node, built here so the rest of the file imports without ROS."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class Bridge(Node):
        def __init__(self):
            super().__init__("base_bridge")
            self.link = PiLink(BridgeLogic(), tcp_connector(host, port))
            self.pub_event = self.create_publisher(String, "/robot/event", 10)
            self.pub_status = self.create_publisher(String, "/base/status", 10)
            self.create_subscription(String, "/robot/command", self.on_command, 10)
            self.create_timer(0.05, self.on_tick)
            self.get_logger().info("Pi link %s:%d" % (host, port))

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
