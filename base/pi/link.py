"""Sends camera frames to the Jetson and collects its answers, on a background thread.

Wire format both ways: 4-byte big-endian length of the header, then the UTF-8 JSON header, then
(requests only) the JPEG bytes. The request header's "jpeg" field says how many JPEG bytes follow.
"""

import json
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field


@dataclass
class Seen:
    bearing_deg: float  # positive = robot's left
    conf: float
    look: tuple  # appearance numbers used to recognise the same person again
    box: list  # [x1, y1, x2, y2] in image pixels
    kp: list = None  # 17 COCO keypoints [x, y, visibility] in image pixels; only on frames sent with pose=True


@dataclass
class Thing:
    cls: str  # "handbag", "backpack" or "suitcase"
    bearing_deg: float  # positive = robot's left
    conf: float
    box: list  # [x1, y1, x2, y2] in image pixels


@dataclass
class Answer:
    t: float  # the Pi capture time of the frame this answers
    people: list
    objects: list = field(default_factory=list)  # Thing per bag seen; empty from a server that sends none


def bearing_deg(box, width, hfov_deg):
    centre_x = (box[0] + box[2]) / 2
    # The image is mirrored, so image right is robot left (hence positive).
    return (centre_x - width / 2) / (width / 2) * hfov_deg / 2


def _encode_jpeg(frame):
    import cv2
    import numpy as np

    rgb = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)
    ok, jpeg = cv2.imencode(".jpg", rgb[:, :, ::-1])  # cv2 wants BGR
    if not ok:
        raise ValueError("jpeg encode failed")
    return jpeg.tobytes()


MAX_REPLY = 1_000_000  # bytes; a bigger reply means a broken or hostile peer
RETRY_PRIMARY_S = 10.0  # while on a backup host, how often to try the primary again
PROBE_TIMEOUT_S = 0.2  # that try: the cable answers in ~1 ms, and a frame waits at most this long
BAD_WITHIN_S = 5.0  # a link lost this soon after connecting (or by a reply timeout) marks its host bad
BAD_FOR_S = 30.0  # a bad host is tried only after the others, and not tried again from a backup
REPLY_WINDOW_S = 1.0  # answers counted by replies_per_s: those of the last second


def hosts(p):
    """The Jetson's addresses in priority order: "jetson_host", then "jetson_hosts_backup", each once."""
    found = []
    for host in [p["jetson_host"]] + list(p.get("jetson_hosts_backup", [])):
        if host not in found:
            found.append(host)
    return found


def _read_exactly(sock, count, deadline):
    data = b""
    while len(data) < count:
        if time.monotonic() > deadline:
            raise TimeoutError("reply took too long")
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError("closed by the Jetson")
        data += chunk
    return data


class JetsonLink:
    """One frame in flight at a time. send() only stores the frame, so it never blocks.

    host is one address or a list in priority order (the cable first). Each (re)connect takes the
    first that answers. On a backup, every retry_primary seconds and only between frames, a short
    connect to the primary is tried; when it answers it becomes the link and the backup is closed.
    A host whose link failed within BAD_WITHIN_S of connecting, or by a reply timeout, is tried
    last for BAD_FOR_S (TCP up but data not passing would otherwise flip the link forever).
    The Jetson server serves one client and a new connection replaces the old one, so the try is
    never a bare probe: a connection made is a connection used, and no reply is in flight then.
    """

    def __init__(self, host, port, hfov_deg, encode=_encode_jpeg, reply_timeout=2.0, clock=time.monotonic,
                 connect=socket.create_connection, retry_primary=RETRY_PRIMARY_S):
        self._hosts = [host] if isinstance(host, str) else list(host)
        self._port, self._hfov = port, hfov_deg
        self._connect = connect
        self._retry_primary = retry_primary
        self._next_primary_try = 0.0
        self._bad_until = {}  # host -> time.monotonic() until which it is tried last
        self._connected_at = None
        self._sock = None
        self.host = None  # the address in use, None while not connected
        self._encode = encode
        self._reply_timeout = reply_timeout
        self._clock = clock
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._pending = None
        self._pending_pose = False
        self._answer = None
        self._last_reply = None
        self.reply_times = deque()  # clock times of the answers kept for replies_per_s
        self._seq = 0
        self._thread = None
        self.error = None  # text of the last problem, for logs

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, frame, pose=False):
        """pose=True asks the Jetson for keypoints on this frame (costs it a pose run: only while pointing)."""
        with self._lock:
            self._pending = frame  # replaces a frame that was still waiting
            self._pending_pose = pose
        self._wake.set()

    def latest(self):
        with self._lock:
            answer, self._answer = self._answer, None
        return answer

    def alive(self, now, silence):
        with self._lock:
            return self._last_reply is not None and now - self._last_reply <= silence

    def replies_per_s(self, now):
        """How many answers arrived in the last REPLY_WINDOW_S seconds (one second)."""
        with self._lock:
            return sum(1 for t in self.reply_times if now - t < REPLY_WINDOW_S)

    def stop(self):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(5)

    def _run(self):
        while not self._stop.is_set():
            try:
                self._open_first()
                self._talk()
            except Exception as error:  # the thread must live until stop()
                self.error = "%s: %s" % (type(error).__name__, error)
                if self.host is not None and (isinstance(error, (TimeoutError, socket.timeout))
                                              or time.monotonic() - self._connected_at < BAD_WITHIN_S):
                    self._bad_until[self.host] = time.monotonic() + BAD_FOR_S
            finally:
                if self._sock is not None:
                    self._sock.close()
                self._sock = self.host = None
            self._stop.wait(1.0)  # reconnect every second

    def _open_first(self):
        """Connect to the first host that answers, in priority order, bad hosts last; raises the last error."""
        good = [h for h in self._hosts if not self._bad(h)]
        for host in good + [h for h in self._hosts if h not in good]:
            try:
                self._use(self._connect((host, self._port), timeout=1.0), host)
                return
            except Exception as error:
                self.error = "%s %s: %s" % (host, type(error).__name__, error)
                last = error
        raise last

    def _bad(self, host):
        return time.monotonic() < self._bad_until.get(host, 0.0)

    def _use(self, sock, host):
        sock.settimeout(self._reply_timeout)
        old, self._sock, self.host = self._sock, sock, host
        self._connected_at = time.monotonic()
        if old is not None:
            old.close()
        self._next_primary_try = time.monotonic() + self._retry_primary

    def _back_to_primary(self):
        """On a backup: now and then try the primary, between frames. A failed try keeps the backup."""
        primary = self._hosts[0]
        if self.host == primary or time.monotonic() < self._next_primary_try or self._bad(primary):
            return
        self._next_primary_try = time.monotonic() + self._retry_primary
        try:
            sock = self._connect((primary, self._port), timeout=PROBE_TIMEOUT_S)
        except Exception:
            return
        self._use(sock, primary)

    def _talk(self):
        while not self._stop.is_set():
            self._back_to_primary()
            sock = self._sock
            if not self._wake.wait(0.2):
                continue
            with self._lock:
                frame, self._pending = self._pending, None
                pose, self._pending_pose = self._pending_pose, False
                self._wake.clear()
            if frame is None:
                continue
            try:
                jpeg = self._encode(frame)
            except Exception as error:
                self.error = "encode %s: %s" % (type(error).__name__, error)
                continue  # skip this frame, keep the connection
            self._seq += 1
            header = {"seq": self._seq, "t": frame.time, "width": frame.width, "height": frame.height, "jpeg": len(jpeg)}
            if pose:
                header["pose"] = True
            head = json.dumps(header).encode("utf-8")
            sock.sendall(struct.pack(">I", len(head)) + head + jpeg)
            deadline = time.monotonic() + self._reply_timeout
            size = struct.unpack(">I", _read_exactly(sock, 4, deadline))[0]
            if size > MAX_REPLY:
                raise ValueError("reply too big: %d bytes" % size)
            reply = _read_exactly(sock, size, deadline)
            self._store(json.loads(reply.decode("utf-8")), frame.width)

    def _store(self, reply, width):
        people = [
            Seen(bearing_deg(p["box"], width, self._hfov), p["conf"], tuple(p["look"]), p["box"], p.get("kp"))
            for p in reply["people"]
        ]
        objects = [Thing(o["cls"], bearing_deg(o["box"], width, self._hfov), o["conf"], o["box"])
                   for o in reply.get("objects", [])]
        with self._lock:
            self._answer = Answer(reply["t"], people, objects)
            self._last_reply = self._clock()
            self.reply_times.append(self._last_reply)
            while self.reply_times and self.reply_times[0] <= self._last_reply - REPLY_WINDOW_S:
                self.reply_times.popleft()  # older answers are not counted any more
