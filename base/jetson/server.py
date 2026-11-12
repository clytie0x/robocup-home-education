"""Jetson side of the link: receives JPEG frames from the Pi and answers with the people in them.

Wire format both ways: 4-byte big-endian length of the header, then the UTF-8 JSON header, then
(requests only) the JPEG bytes. Request header {"seq", "t", "width", "height", "jpeg"}; reply
{"seq", "t", "people": [{"box": [x1, y1, x2, y2], "conf": float, "look": [float, ...]}]}.

Run: python3 server.py --fake      (answers one person in the middle, for testing the link)
"""

import argparse
import json
import socket
import struct
import threading
import time

MAX_HEADER = 4096
MAX_JPEG = 10_000_000


def find_people(jpeg, width, height):
    """The seam for the real detector: turn one JPEG image into a list of people dicts."""
    raise NotImplementedError("the real detector comes in a later task, use --fake for now")


def fake_people(jpeg, width, height):
    box = [width * 0.4, height * 0.1, width * 0.6, height * 0.9]
    return [{"box": box, "conf": 0.9, "look": [0.1, 0.2, 0.3, 0.4]}]


def _read_exactly(sock, count, stop, idle):
    data = b""
    deadline = time.monotonic() + idle
    while len(data) < count:
        if time.monotonic() > deadline:
            raise ConnectionError("idle too long")
        try:
            chunk = sock.recv(count - len(data))
        except socket.timeout:
            if stop.is_set():
                raise ConnectionError("server stopping")
            continue
        if not chunk:
            raise ConnectionError("closed by the Pi")
        data += chunk
    return data


class Server:
    def __init__(self, detect, host="0.0.0.0", port=5601, idle=5.0):
        self._detect = detect
        self._idle = idle
        self._current = None  # the one client being served
        self._detect_lock = threading.Lock()  # a replaced client may still be mid-detect
        self._stop = threading.Event()
        self._listener = socket.socket()
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(2)
        self._listener.settimeout(0.2)
        self.port = self._listener.getsockname()[1]
        self._threads = []

    def start(self):
        self._spawn(self._accept_loop)

    def _spawn(self, target, *args):
        thread = threading.Thread(target=target, args=args, daemon=True)
        thread.start()
        self._threads = [t for t in self._threads if t.is_alive()] + [thread]

    def stop(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(5)
        self._listener.close()

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._listener.accept()
            except socket.timeout:
                continue
            conn.settimeout(0.2)
            if self._current is not None:  # a new client replaces the old one
                try:
                    self._current.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            self._current = conn
            self._spawn(self._serve, conn)

    def _serve(self, conn):
        with conn:
            try:
                while True:
                    size = struct.unpack(">I", _read_exactly(conn, 4, self._stop, self._idle))[0]
                    if size > MAX_HEADER:
                        return
                    header = json.loads(_read_exactly(conn, size, self._stop, self._idle).decode("utf-8"))
                    if not 0 <= header["jpeg"] <= MAX_JPEG:
                        return
                    jpeg = _read_exactly(conn, header["jpeg"], self._stop, self._idle)
                    with self._detect_lock:
                        people = self._detect(jpeg, header["width"], header["height"])
                    reply = json.dumps({"seq": header["seq"], "t": header["t"], "people": people}).encode("utf-8")
                    conn.sendall(struct.pack(">I", len(reply)) + reply)
            except (OSError, ValueError, KeyError, TypeError):
                return  # bad or closed connection: drop it, the Pi reconnects


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--port", type=int, default=5601)
    args = parser.parse_args()
    server = Server(fake_people if args.fake else find_people, port=args.port)
    server.start()
    print("listening on port", server.port, flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
