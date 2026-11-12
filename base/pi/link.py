"""Sends camera frames to the Jetson and collects its answers, on a background thread.

Wire format both ways: 4-byte big-endian length of the header, then the UTF-8 JSON header, then
(requests only) the JPEG bytes. The request header's "jpeg" field says how many JPEG bytes follow.
"""

import json
import socket
import struct
import threading
import time
from dataclasses import dataclass


@dataclass
class Seen:
    bearing_deg: float  # positive = robot's left
    conf: float
    look: tuple  # appearance numbers used to recognise the same person again
    box: list  # [x1, y1, x2, y2] in image pixels


@dataclass
class Answer:
    t: float  # the Pi capture time of the frame this answers
    people: list


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
    """One frame in flight at a time. send() only stores the frame, so it never blocks."""

    def __init__(self, host, port, hfov_deg, encode=_encode_jpeg, reply_timeout=2.0, clock=time.monotonic):
        self._host, self._port, self._hfov = host, port, hfov_deg
        self._encode = encode
        self._reply_timeout = reply_timeout
        self._clock = clock
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._pending = None
        self._answer = None
        self._last_reply = None
        self._seq = 0
        self._thread = None
        self.error = None  # text of the last problem, for logs

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, frame):
        with self._lock:
            self._pending = frame  # replaces a frame that was still waiting
        self._wake.set()

    def latest(self):
        with self._lock:
            answer, self._answer = self._answer, None
        return answer

    def alive(self, now, silence):
        with self._lock:
            return self._last_reply is not None and now - self._last_reply <= silence

    def stop(self):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(5)

    def _run(self):
        while not self._stop.is_set():
            try:
                with socket.create_connection((self._host, self._port), timeout=1.0) as sock:
                    sock.settimeout(self._reply_timeout)
                    self._talk(sock)
            except Exception as error:  # the thread must live until stop()
                self.error = "%s: %s" % (type(error).__name__, error)
            self._stop.wait(1.0)  # reconnect every second

    def _talk(self, sock):
        while not self._stop.is_set():
            if not self._wake.wait(0.2):
                continue
            with self._lock:
                frame, self._pending = self._pending, None
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
            Seen(bearing_deg(p["box"], width, self._hfov), p["conf"], tuple(p["look"]), p["box"])
            for p in reply["people"]
        ]
        with self._lock:
            self._answer = Answer(reply["t"], people)
            self._last_reply = self._clock()
