"""A virtual operator for run_follow --virtual-operator PORT: the real robot follows a simulated person.

The owner's PC (base/pc/virtual_operator/app.py) walks a 3D person through a map of the real room and
sends its state about 10 times a second as one UDP datagram of JSON:

    {"t": sender seconds, "x": m, "y": m, "th": rad, "speaking": null | "follow" | "stop" | "done" | "back",
     "pointing": null | [x, y], "bags": [[x, y], ...] (optional, at most MAX_BAGS)}

x, y, th are in the run's odometry frame (the frame of the record's "pose"). Each cycle, before the
senses read, apply() puts that person into what the senses see:

- LiDAR: two leg circles (LEG_RADIUS, LEG_APART across the heading, swinging with the distance walked)
  are cut into the real scan at the real scan's own bearings. A virtual return behind a real return on
  the same bearing is dropped (the real object hides the leg); a virtual leg in front of a real return
  replaces it (the leg hides the wall), as a real person would.
- Camera: the Jetson's answer is replaced by a made-up one: one Seen (conf 0.9, a look vector fixed for
  this run, a box of a 1.7 m person) when the person is inside the camera's field of view, within
  CAMERA_RANGE and not hidden behind real returns; else nobody. Real camera people are always dropped,
  or a real bystander would be enrolled.
- Pointing: the floor spot the person points at (the bag), in the same frame, goes to the report and so to
  the record as "virtual_pointing"; run_follow's BAG phase (bag.py) uses it in place of the camera's pointing.
- Bags: each virtual bag is a BAG_RADIUS circle cut into the scan the senses read, like the legs (a real return
  nearer on the same bearing wins), so BAG finds and approaches it with no real bag. Like the legs, the
  last-resort stop never sees it.
- Words: a new word in "speaking" is one command, as the voice flow would send it (COMMANDS), queued as
  the page's task buttons (dashboard.Controls); each fresh state is also the page's heartbeat.

No state for TIMEOUT: the person is gone. The scan is the real one, there is no camera answer, and the
robot behaves as with a lost operator.

Safety: run_follow keeps the real scan for safety.must_stop, the scan matching, the keyframes and the
logs. Only the senses (and so the brain) see the virtual legs: the brain keeps its follow gap to the
person as to a real one, while the last-resort stop decides on the real room alone, never made weaker
(no real return is removed for it) and never triggered by someone who is not there.

Only datagrams from 127.0.0.1, the addresses given (the PC's LAN address) and Tailscale
(100.64.0.0/10) are taken.
"""
import ipaddress
import json
import math
import random
import socket
import threading
import time

import geometry
from link import Answer, Seen

TIMEOUT = 0.5  # s without a state: the person vanishes
LEG_RADIUS = 0.06  # m
LEG_APART = 0.18  # m between the two leg centres, across the heading
SWING = 0.12  # m each leg swings ahead and behind at a full walk
FULL_SWING_SPEED = 0.5  # m/s and faster: the full swing; slower walks swing less
STRIDE = 0.8  # m walked per full gait cycle (left forward, back, and forward again)
SPEED_SMOOTH = 0.5  # weight of the newest packet in the walking speed
TELEPORT = 1.0  # m in one packet: put there, not walked (no swing, no speed)
CONF = 0.9
LOOK_SIZE = 512  # numbers in a look vector (the Jetson's OSNet x0_25)
CAMERA_RANGE = 5.0  # m from the camera; farther people are not detected
IMAGE = (640, 480)  # px of the camera frames the boxes are given in
PERSON_HEIGHT = 1.7  # m
PERSON_WIDTH = 0.45  # m
MIN_RANGE = 0.05  # m; a real sample at most this is no return (lidar.usable)
MAX_PACKET = 1024  # bytes
BAG_RADIUS = 0.065  # m, a virtual bag: its returns average about 0.05 m (pi / 4 of it) in front of its centre, as bag.BAG_HALF_DEPTH
MAX_BAGS = 4
# word -> command; "bag" stands for the flow's find_bag (look for the pointed bag and drive up to it)
COMMANDS = {"follow": "follow", "stop": "wait", "done": "return", "back": "return", "bag": "bag"}
TAILSCALE = ipaddress.ip_network("100.64.0.0/10")
LOOPBACK = "127.0.0.1"


def legs(person, walked, speed):
    """(left, right) leg centres of a person (x, y, th) in the same frame, after `walked` m at `speed` m/s."""
    x, y, th = person
    fx, fy = math.cos(th), math.sin(th)
    lx, ly = -fy, fx
    swing = SWING * min(1.0, max(0.0, speed) / FULL_SWING_SPEED) * math.sin(2 * math.pi * walked / STRIDE)
    half = LEG_APART / 2
    return ((x + half * lx + swing * fx, y + half * ly + swing * fy),
            (x - half * lx - swing * fx, y - half * ly - swing * fy))


def _ray_hit(deg, centres, radius=LEG_RADIUS):
    """The nearest range along the laser bearing `deg` to any circle (lidar frame), or None."""
    dx, dy = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    best = None
    for cx, cy in centres:
        b = dx * cx + dy * cy
        if b <= 0:
            continue
        disc = b * b - (cx * cx + cy * cy - radius * radius)
        if disc < 0:
            continue
        hit = b - math.sqrt(disc)
        if hit > 0 and (best is None or hit < best):
            best = hit
    return best


def inject(scan, legs_base, p, bags_base=()):
    """The scan with the legs (base-frame centres) cut in at its own bearings: (scan, shown, hidden).

    shown: samples that now return from a leg; hidden: leg samples dropped behind a nearer real return.
    bags_base: virtual bags (BAG_RADIUS circles), cut in the same way; they are not counted."""
    centres = [(x - p["lidar_x"], y) for x, y in legs_base]  # the scan's ranges start at the laser
    bags = [(x - p["lidar_x"], y) for x, y in bags_base]
    sensed, shown, hidden = [], 0, 0
    for deg, m in scan:
        hit = _ray_hit(deg, centres)
        bag = _ray_hit(deg, bags, BAG_RADIUS) if bags else None
        if bag is not None and (hit is None or bag < hit):
            sensed.append((deg, m) if m > MIN_RANGE and m <= bag else (deg, bag))
            continue
        if hit is None:
            sensed.append((deg, m))
        elif m > MIN_RANGE and m <= hit:
            sensed.append((deg, m))
            hidden += 1
        else:
            sensed.append((deg, hit))
            shown += 1
    return sensed, shown, hidden


def camera_bearing(point_base, p):
    """Degrees from the camera's axis to a base-frame point, positive left (as link.Seen.bearing_deg)."""
    return math.degrees(math.atan2(point_base[1] - p["camera_y"], point_base[0] - p["camera_x"]))


def box_of(bearing, distance, p):
    """A plausible [x1, y1, x2, y2] for a standing person at this camera bearing and distance."""
    w, h = IMAGE
    focal = (w / 2) / math.tan(math.radians(p["camera_hfov_deg"] / 2))
    cx = w / 2 + bearing / (p["camera_hfov_deg"] / 2) * (w / 2)  # link.bearing_deg backwards
    half = focal * PERSON_WIDTH / 2 / distance
    lift = p.get("camera_height", 0.0)
    top = h / 2 - focal * (PERSON_HEIGHT - lift) / distance
    bottom = h / 2 + focal * lift / distance
    x1, x2 = max(0.0, cx - half), min(float(w), cx + half)
    y1, y2 = max(0.0, top), min(float(h), bottom)
    if y2 - y1 < 1.0:
        y1, y2 = 0.0, float(h)
    return [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]


def random_look(rng):
    look = [rng.gauss(0.0, 1.0) for _ in range(LOOK_SIZE)]
    norm = math.sqrt(sum(v * v for v in look))
    return tuple(v / norm for v in look)


def parse_allow(text):
    """--virtual-allow's comma list as addresses; ValueError for anything but one computer's IPv4 address."""
    found = [a.strip() for a in (text or "").split(",") if a.strip()]
    for a in found:
        try:
            address = ipaddress.IPv4Address(a)
        except ValueError:
            raise ValueError("%r is not an IPv4 address" % a) from None
        if address.is_unspecified or address.is_multicast or a.endswith(".255"):
            raise ValueError("%r is not one computer" % a)
    return found


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class VirtualOperator:
    """The virtual person's newest state (from a UDP thread) and what it does to one cycle.

    controls: dashboard.Controls, which gets the spoken commands and the heartbeat; None in some tests.
    clock: the clock of run_follow's cycle `now`."""

    def __init__(self, p, port, allow=(), controls=None, host="0.0.0.0", clock=time.monotonic, seed=None):
        self.p, self.port, self.host = p, port, host
        self.allow = {a for a in allow if a}
        self.controls, self.clock = controls, clock
        self.look = random_look(random.Random(seed))
        self.lock = threading.Lock()
        self.state = None  # (x, y, th) of the newest accepted state
        self.at = None  # clock time it arrived
        self.sent_t = None  # its sender time
        self.sender = None  # its sender address
        self.speaking = None  # its word
        self.pointing = None  # the floor spot it points at, [x, y], or None
        self.bags = []  # virtual bags on the floor, [[x, y], ...]
        self.words = []  # new words since the last cycle took them
        self.walked = 0.0  # m walked so far (the leg swing)
        self.speed = 0.0  # m/s, smoothed
        self.packets = 0
        self.refused = 0
        self.report = {"person": None}
        self.sock = None
        self.thread = None
        self.stopping = threading.Event()

    def allowed(self, address):
        if address == LOOPBACK or address in self.allow:
            return True
        try:
            return ipaddress.ip_address(address) in TAILSCALE
        except ValueError:
            return False

    def _parse(self, data):
        if len(data) > MAX_PACKET:
            return None
        try:
            msg = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return None
        if not isinstance(msg, dict) or not all(_number(msg.get(k)) for k in ("t", "x", "y", "th")):
            return None
        if msg.get("speaking") is not None and msg.get("speaking") not in COMMANDS:
            return None
        pointing = msg.get("pointing")
        if pointing is not None and not (isinstance(pointing, list) and len(pointing) == 2 and all(map(_number, pointing))):
            return None
        bags = msg.get("bags")
        if bags is not None and not (isinstance(bags, list) and len(bags) <= MAX_BAGS and all(
                isinstance(b, list) and len(b) == 2 and all(map(_number, b)) for b in bags)):
            return None
        return msg

    def receive(self, data, address):
        """One datagram. True when taken; anything malformed, late or from elsewhere is counted and dropped."""
        now = self.clock()
        msg = self._parse(data) if self.allowed(address[0]) else None
        with self.lock:
            fresh = self.at is not None and now - self.at <= TIMEOUT
            if msg is None or (fresh and msg["t"] <= self.sent_t):  # a late datagram from the same sender run
                self.refused += 1
                return False
            state = (float(msg["x"]), float(msg["y"]), float(msg["th"]))
            if self.state is not None and fresh:
                step = math.dist(state[:2], self.state[:2])
                dt = msg["t"] - self.sent_t
                if step <= TELEPORT:
                    self.walked += step
                    self.speed = (1 - SPEED_SMOOTH) * self.speed + SPEED_SMOOTH * (step / dt if dt > 0 else 0.0)
                else:
                    self.speed = 0.0
            word = msg.get("speaking")
            if word is not None and word != self.speaking:
                self.words.append(word)
            self.speaking = word
            self.pointing = None if msg.get("pointing") is None else [float(v) for v in msg["pointing"]]
            self.bags = [[float(v) for v in b] for b in msg.get("bags") or []]
            self.state, self.at, self.sent_t, self.sender = state, now, msg["t"], address[0]
            self.packets += 1
        return True

    def apply(self, now, pose, scan, answer, link_alive):
        """(scan, answer, link_alive) for the senses this cycle. pose: the odometry pose of this cycle."""
        with self.lock:
            state, at, sender, speaking, pointing = self.state, self.at, self.sender, self.speaking, self.pointing
            bags = [list(b) for b in self.bags]
            words, self.words = self.words, []
            walked, speed, packets, refused = self.walked, self.speed, self.packets, self.refused
        fresh = state is not None and now - at <= TIMEOUT
        report = {"person": None, "age": None if at is None else round(now - at, 3), "from": sender,
                  "speaking": speaking, "pointing": None, "bags": [], "said": [], "seen": False, "returns": 0, "hidden": 0,
                  "real_people_dropped": len(answer.people) if answer is not None else 0,
                  "jetson_link": bool(link_alive), "packets": packets, "refused": refused}
        if not fresh:
            self.report = report
            return scan, None, False
        if self.controls is not None:
            self.controls.polled()  # the PC leads the task while it sends
            for word in words:
                self.controls.command(COMMANDS[word])
        report["said"] = words
        left, right = legs(state, walked, speed)
        sensed, shown, hidden = inject(scan, [geometry.odom_to_base(left, pose), geometry.odom_to_base(right, pose)],
                                       self.p, [geometry.odom_to_base(b, pose) for b in bags])
        centre = geometry.odom_to_base(state[:2], pose)
        bearing = camera_bearing(centre, self.p)
        distance = math.hypot(centre[0] - self.p["camera_x"], centre[1] - self.p["camera_y"])
        seen = (abs(bearing) <= self.p["camera_hfov_deg"] / 2 and 0.3 <= distance <= CAMERA_RANGE
                and shown > 0 and shown >= hidden)
        people = [Seen(bearing, CONF, self.look, box_of(bearing, distance, self.p))] if seen else []
        report.update(person=[round(v, 3) for v in state], pointing=pointing, bags=bags, seen=seen, returns=shown, hidden=hidden,
                      speed=round(speed, 3))
        self.report = report
        return sensed, Answer(now, people), True

    # ------------------------------------------------------------ the UDP thread

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopping.is_set():
            try:
                data, address = self.sock.recvfrom(MAX_PACKET + 1)
            except socket.timeout:
                continue
            except OSError:
                if self.stopping.is_set():
                    return
                continue
            try:
                self.receive(data, address)
            except Exception:  # the thread must live until stop()
                pass

    def stop(self):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(2)
        if self.sock is not None:
            self.sock.close()
