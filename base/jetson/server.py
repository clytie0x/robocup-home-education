"""Jetson side of the link: receives JPEG frames from the Pi and answers with the people in them.

Wire format both ways: 4-byte big-endian length of the header, then the UTF-8 JSON header, then
(requests only) the JPEG bytes. Request header {"seq", "t", "width", "height", "jpeg"}, optional "pose": true;
reply {"seq", "t", "people": [{"box": [x1, y1, x2, y2], "conf": float, "look": [float, ...]}],
"objects": [{"cls": "handbag" | "backpack" | "suitcase", "box": [x1, y1, x2, y2], "conf": float}]}.
With pose (asked in the header, or every --pose-every frames) a person whose box overlaps a pose box also
has "kp": 17 COCO keypoints [x, y, visibility] in the frame's pixels. Old clients ignore the new keys.
The real server also judges each frame's light (light.FrameGate): "quality": {"ok", "why", "luma"}, ok False for a
frame too dark, too saturated or a flash, and gives each person "light", the mean luma of their box (light.box_light),
so the Pi compares a look only in about the light the operator was learned in.
"reid": "<backend> <engine file>" names the model the looks come from; reid.py's fallbacks can switch it
mid-run, and the Pi then leaves out looks from a model other than the one the operator was learned with.

Run: python3 server.py --fake      (answers one person in the middle, for testing the link)
     python3 server.py             (TensorRT detector and OSNet looks, see DETECTOR and REID; pose with POSE)

Pose cost, not measured: yolo11n-pose is 2.9 M parameters and 7.6 GFLOPs at 640 x 640 against 2.6 M and
6.5 GFLOPs for yolo11n (Ultralytics' tables), so at 256 x 320 about 1.2 times the detector. Detector plus
OSNet measured 94.7 frames/s (about 10.6 ms) on 2026-11-13, so pose should add roughly 5 to 8 ms a frame,
inside the 100 ms of 10 Hz. It runs only when asked all the same, and the Pi asks only while the operator
points at the bag.
"""

import argparse
import json
import os
import socket
import struct
import threading
import time

import ports

MAX_HEADER = 4096
MAX_JPEG = 10_000_000


DETECTOR = "models/yolo11s_480x640.engine"
DETECTOR_FALLBACK = "models/yolo11n_256x320.engine"
REID = "models/osnet_ain_x1_0_msmt17_256x128.engine"
REID_FALLBACK = "models/osnet_x0_25_msmt17_256x128.engine"
POSE = "models/yolo11s-pose_480x640.engine"
POSE_FALLBACK = "models/yolo11n-pose_256x320.engine"
POSE_CONF = 0.4
KP_IOU = 0.3  # a pose box must overlap a person box at least this much to give it keypoints


def seen_in(jpeg, detector, reid, pose=None, gate=None, normalise="none"):
    """One JPEG to {"people", "objects"}: person and bag boxes from one detector run, a look vector per
    person, and with a pose engine the keypoints of each person it finds. With a gate (light.FrameGate, it keeps
    the history of the last frames) also the frame's "quality" and each person's box "light". normalise: the
    light.normalise mode applied before the detector, the look network and pose."""
    import cv2
    import numpy as np

    import light

    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        # a broken frame is "nobody seen", never an error that drops the link
        return {"people": [], "objects": [], "empty": "the frame could not be decoded"}
    raw, bgr = bgr, light.normalise(bgr, normalise)
    try:
        boxes, objects = detector.detect_all(bgr) if hasattr(detector, "detect_all") else (detector.detect(bgr), [])
    except RuntimeError as exc:
        print("detector: 0 people: TensorRT execute failed: %s" % exc, flush=True)
        return {"people": [], "objects": [], "empty": "TensorRT execute failed: %s" % exc}
    try:
        looks = reid.looks(bgr, [b[:4] for b in boxes])
    except RuntimeError as exc:
        print("reid failed, looks left empty: %s" % exc, flush=True)
        looks = [None] * len(boxes)
    from reid import clothing_histogram
    colours = clothing_histogram(bgr, [b[:4] for b in boxes]) if boxes else []
    # An empty look is [] (never JSON null: the Pi does tuple(look)). The Pi treats [] as no look,
    # so a failed re-id is not enrolled. A real vector is a list of floats.
    people = []
    for i, (b, look) in enumerate(zip(boxes, looks)):
        person = {"box": [float(v) for v in b[:4]], "conf": float(b[4]),
                  "look": [float(v) for v in look] if look else []}
        if i < len(colours):
            person["colour"] = [float(v) for v in colours[i]]
        people.append(person)
    if pose is not None:
        attach_keypoints(people, pose.poses(bgr))
    seen = {"people": people,
            "objects": [{"cls": o[5], "box": [float(v) for v in o[:4]], "conf": float(o[4])} for o in objects]}
    if getattr(reid, "backend", None):  # the model the looks come from: the Pi never compares two models' looks
        seen["reid"] = "%s %s" % (reid.backend, os.path.basename(getattr(reid, "path", "") or ""))
    if gate is not None:
        ok, why = gate.check(raw)
        seen["quality"] = {"ok": ok, "why": why, "luma": round(gate.mean, 1)}
        for person in people:  # the light the camera really had, before normalise
            person["light"] = round(light.box_light(raw, person["box"]), 1)
    if not people:
        seen["empty"] = getattr(detector, "empty", None) or "no person box kept"
    return seen


def people_in(jpeg, detector, reid):
    """One JPEG to people dicts: TensorRT person boxes, then one look vector per person."""
    return seen_in(jpeg, detector, reid)["people"]


def _iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def attach_keypoints(people, poses, min_iou=KP_IOU):
    """Give each person dict the "kp" of the pose box it overlaps best; each pose goes to one person at most."""
    pairs = sorted(((_iou(p["box"], q[0]), i, j) for i, p in enumerate(people) for j, q in enumerate(poses)),
                   reverse=True)
    used_people, used_poses = set(), set()
    for iou, i, j in pairs:
        if iou < min_iou:
            break
        if i in used_people or j in used_poses:
            continue
        used_people.add(i)
        used_poses.add(j)
        people[i]["kp"] = [[float(v) for v in k] for k in poses[j][2]]
    return people


def decode_pose(output, scale, frame_w, frame_h, conf=POSE_CONF):
    """yolo11n-pose output (1, 56, N): cx, cy, w, h, score, 17 x (x, y, visibility) in input pixels
    -> [((x1, y1, x2, y2), score, 17 x [x, y, v])] in frame pixels, best first."""
    import numpy as np
    from detector import nms

    out = output[0].T
    out = out[out[:, 4] >= conf].astype(np.float32)
    if not len(out):
        return []
    boxes = np.concatenate([out[:, :2] - out[:, 2:4] / 2, out[:, :2] + out[:, 2:4] / 2], axis=1) / scale
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, frame_w)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, frame_h)
    kps = out[:, 5:56].reshape(-1, 17, 3).copy()
    kps[:, :, :2] /= scale
    return [(tuple(map(float, boxes[i])), float(out[i, 4]), kps[i].tolist()) for i in nms(boxes, out[:, 4])]


class Pose:
    """The pose engine, letterboxed like the Ultralytics detector (RGB / 255, padding at the bottom)."""

    def __init__(self, engine_path):
        from engine import Engine

        self._engine = Engine(engine_path)
        _, _, self._h, self._w = self._engine.input_shape

    def poses(self, bgr):
        from detector import letterbox

        x, scale = letterbox(bgr, self._h, self._w, yolox=False)
        return decode_pose(self._engine.run(x)[0], scale, bgr.shape[1], bgr.shape[0])


def _pick_engine(path, fallback):
    from engine import engine_present

    if engine_present(path):
        return path
    return fallback if engine_present(fallback) else path


def real_people(detector_path=DETECTOR, reid_path=REID, pose_path=POSE, normalise=None):
    """find_people for the server, with the engines loaded once. Without the pose engine, pose asks get no "kp".
    Every frame goes through one light.FrameGate (the Server serves one frame at a time)."""
    import light
    from detector import Detector
    from engine import engine_present
    from reid import ReId

    det_p = _pick_engine(detector_path, DETECTOR_FALLBACK)
    reid_p = _pick_engine(reid_path, REID_FALLBACK)
    pose_p = _pick_engine(pose_path, POSE_FALLBACK)
    print(f"engines: det={det_p} reid={reid_p} pose={pose_p}", flush=True)
    reid_fallback = REID_FALLBACK if reid_p != REID_FALLBACK else None
    detector, reid = Detector(det_p), ReId(reid_p, reid_fallback)
    pose_engine = Pose(pose_p) if engine_present(pose_p) else None
    if pose_engine is None:
        print("no pose engine at", pose_path, "- answers carry no keypoints", flush=True)
    gate, mode = light.FrameGate(), normalise or light.NORMALISE
    return lambda jpeg, width, height, pose=False: seen_in(jpeg, detector, reid, pose_engine if pose else None,
                                                           gate, mode)


def fake_people(jpeg, width, height, pose=False):
    box = [width * 0.4, height * 0.1, width * 0.6, height * 0.9]
    person = {"box": box, "conf": 0.9, "look": [0.1, 0.2, 0.3, 0.4]}
    if pose:  # someone standing with both arms down, in the middle of the box
        cx, top, h = width * 0.5, height * 0.1, height * 0.8
        rows = [0.05, 0.03, 0.03, 0.04, 0.04, 0.18, 0.18, 0.35, 0.35, 0.5, 0.5, 0.52, 0.52, 0.72, 0.72, 0.95, 0.95]
        side = [0, -1, 1, -2, 2, -5, 5, -6, 6, -6, 6, -3, 3, -3, 3, -3, 3]
        person["kp"] = [[cx + s * width * 0.01, top + r * h, 0.9] for s, r in zip(side, rows)]
    return [person]


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
    def __init__(self, detect, host="0.0.0.0", port=ports.DETECTOR, idle=5.0, pose_every=0):
        self._detect = detect
        self._idle = idle
        self._pose_every = pose_every  # 0: pose only when a request header asks for it
        self._frames = 0
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
                    self._frames += 1
                    pose = header.get("pose") is True or (self._pose_every > 0 and self._frames % self._pose_every == 0)
                    try:
                        with self._detect_lock:  # pose passed only when wanted, so detect callables without it still work
                            found = self._detect(jpeg, header["width"], header["height"],
                                                 **({"pose": True} if pose else {}))
                    except RuntimeError as exc:
                        # A TensorRT execute failure used to drop the socket; the Pi reconnected and it failed
                        # again (65 times on 19 Nov). One empty answer keeps the link, like a broken JPEG.
                        print("detect failed, answering nobody: %s" % exc, flush=True)
                        found = {"people": [], "objects": []}
                    reply = {"seq": header["seq"], "t": header["t"]}
                    reply.update(found if isinstance(found, dict) else {"people": found})
                    reply = json.dumps(reply).encode("utf-8")
                    conn.sendall(struct.pack(">I", len(reply)) + reply)
            except (OSError, ValueError, KeyError, TypeError):
                return  # bad or closed connection: drop it, the Pi reconnects


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--port", type=int, default=ports.DETECTOR)
    parser.add_argument("--detector", default=DETECTOR, help="person detector engine")
    parser.add_argument("--reid", default=REID, help="look vector engine")
    parser.add_argument("--pose", default=POSE, help="pose engine (keypoints only when asked)")
    parser.add_argument("--pose-every", type=int, default=0, help="also run pose every N frames (0: only when asked)")
    parser.add_argument("--normalise", default=None,
                        help="light.normalise mode before the networks (default light.NORMALISE; tools/light_bench.py)")
    args = parser.parse_args()
    server = Server(fake_people if args.fake else real_people(args.detector, args.reid, args.pose, args.normalise),
                    port=args.port,
                    pose_every=args.pose_every)
    server.start()
    print("listening on port", server.port, flush=True)
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
