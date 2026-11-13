"""The run_follow side of plan_server.py: plans in the background, never blocks the cycle.

plan() hands the newest request to a thread and returns at once with the newest answer. That
answer is for a request from a cycle or two ago (0.1 to 0.3 s): the robot moves under 0.04 m
meanwhile. When no answer has come for MAX_AGE, alive() is False and the caller plans itself.
"""
import contextlib
import json
import socket
import threading
import time

MAX_AGE = 1.0  # s; an older answer means the planner is gone or stuck
CONNECT_TIMEOUT = 0.5  # s
READ_TIMEOUT = 1.0  # s for one answer
RETRY = 1.0  # s between connection attempts


class RemotePlanner:
    def __init__(self, host, port, robot_radius, clock=time.monotonic, connect=socket.create_connection):
        self.host, self.port, self.robot_radius = host, port, robot_radius
        self.clock, self.connect = clock, connect
        self.lock = threading.Lock()
        self.wanted = None  # the newest request, not sent yet
        self.wake = threading.Event()
        self.answered_at = None  # clock time of the newest answer
        self.path = None  # the newest answer's path (None: no way found)
        self.seconds = None  # how long the server took for it
        self.next_id = 1
        self.done = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.done.set()
        self.wake.set()

    def alive(self):
        with self.lock:
            return self.answered_at is not None and self.clock() - self.answered_at <= MAX_AGE

    def plan(self, start, goal, shapes, near_side=False, heading=None):
        """Ask for a plan and return the newest answer (a list of points, or None)."""
        request = {"start": list(start), "goal": list(goal), "near_side": near_side, "heading": heading,
                   "robot_radius": self.robot_radius,
                   "shapes": [[type(s).__name__] + list(vars(s).values()) for s in shapes]}
        with self.lock:
            request["id"], self.next_id = self.next_id, self.next_id + 1
            self.wanted = request
            path = self.path
        self.wake.set()
        return None if path is None else [tuple(p) for p in path]

    def _run(self):
        while not self.done.is_set():
            try:
                conn = self.connect((self.host, self.port), timeout=CONNECT_TIMEOUT)
            except OSError:
                self.done.wait(RETRY)
                continue
            try:
                conn.settimeout(READ_TIMEOUT)
                with contextlib.suppress(OSError):  # not a TCP socket (a test's socketpair on Linux)
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._talk(conn.makefile("rb"), conn)
            except (OSError, ValueError):
                pass  # the server went away or timed out: connect again
            finally:
                conn.close()
            self.done.wait(RETRY)

    def _talk(self, reader, conn):
        while not self.done.is_set():
            self.wake.wait(0.5)
            self.wake.clear()
            with self.lock:
                request, self.wanted = self.wanted, None
            if request is None:
                continue
            conn.sendall((json.dumps(request) + "\n").encode())
            line = reader.readline()
            if not line:
                raise OSError("plan server closed")
            got = json.loads(line)
            with self.lock:
                self.path, self.seconds, self.answered_at = got["path"], got["seconds"], self.clock()
