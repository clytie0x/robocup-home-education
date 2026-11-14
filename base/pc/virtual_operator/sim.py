"""The virtual person's world on the PC: a map of the real room from the robot's scans, A* walks, the auto loop.

Standard library only. Everything is in the run's odometry frame (the frame of run_follow's record "pose"):
x, y in metres, th in radians. The robot's LiDAR points come in its own frame (x forward, y left).
"""
import csv
import heapq
import math
import random
import time
from pathlib import Path

CELL = 0.05  # m, one map cell
HIT = 1.0  # added to a cell for each scan that returns from it
MISS = 0.4  # taken from a cell a ray passes through
CAP = 5.0  # the most a cell can hold
OCCUPIED = 1.5  # a cell with at least this is a wall (two scans in a row)
DECAY = 0.995  # each scan, every cell: what is no longer seen fades (a chair moved away)
SCAN_RANGE = 4.0  # m; farther points are not mapped (camera_view.SCAN_RANGE: the console sends no more)
CLEARANCE = 0.3  # m the person's centre keeps from every wall cell
ROBOT_KEEP = 0.6  # m the person's path keeps from the robot's centre
FRONT = 1.0  # m in front of the robot where the person stands to be locked
NEAR, FAR = 2.0, 4.0  # m: the auto loop walks to a random spot this far away
WALK_SPEED = 0.24  # m/s: the owner's median walking speed while followed (run-023855, see docs/virtual-operator.md)
SAY_S = 1.5  # s a spoken word stays in the state
POINT_S = 5.0  # s the person points at a floor spot (the bag), unless told otherwise
MAX_BAGS = 3  # virtual bags on the floor (the robot takes up to 4)
REACHED = 1.2  # m; the robot this close to the person at the goal: "stop"
HOLD_S = 5.0  # s between "stop" and "back"
TOO_CLOSE = 0.05  # m between the robot's body and anything: the auto loop stops
CABLE_LIMIT = 150.0  # degrees of net cable turn: the auto loop stops
BODY_HALF = 0.20  # m, half the robot's 0.40 m square body around its turning centre
WORDS = ("follow", "stop", "done", "back", "bag")  # "bag": the robot looks for the pointed bag (the flow's find_bag)
HELD = ("wait", "page", "idle", "silent", "paused")  # run_follow stop reasons where standing still is right
CSV_FIELDS = ["start", "start_x", "start_y", "goal_x", "goal_y", "walk_m", "time_to_reach_s", "nearest_m",
              "stuck_s", "result"]
# s each auto step may take before the loop gives up
LIMITS = {"starting": 45.0, "lock": 30.0, "follow": 8.0, "reach": 120.0, "return": 240.0}


def cell_of(x, y):
    return (math.floor(x / CELL), math.floor(y / CELL))


def centre(c):
    return ((c[0] + 0.5) * CELL, (c[1] + 0.5) * CELL)


def to_world(pose, point):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return (x + c * point[0] - s * point[1], y + s * point[0] + c * point[1])


def body_gap(points):
    """m from the robot's square body to the nearest robot-frame point, or None without points."""
    gaps = [math.hypot(max(abs(x) - BODY_HALF, 0.0), max(abs(y) - BODY_HALF, 0.0)) for x, y in points]
    return min(gaps) if gaps else None


def _disc(radius):
    r = math.ceil(radius / CELL)
    return [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1) if (i * i + j * j) * CELL * CELL <= radius * radius]


def inflate(cells, radius):
    offsets = _disc(radius)
    out = set()
    for i, j in cells:
        for di, dj in offsets:
            out.add((i + di, j + dj))
    return out


class Grid:
    """Occupancy from the robot's scans: cells hit grow, cells a ray crosses shrink, all fade a little."""

    def __init__(self):
        self.score = {}  # cell -> evidence of a wall
        self.free = set()  # cells a ray has crossed: seen, so the person may walk there
        self.version = 0

    def clear(self):
        self.score, self.free = {}, set()
        self.version += 1

    def add_scan(self, pose, points, rays_every=1):
        x0, y0 = pose[0], pose[1]
        hits, crossed = set(), set()
        for n, point in enumerate(points):
            r = math.hypot(point[0], point[1])
            if not 0.05 < r <= SCAN_RANGE:
                continue
            wx, wy = to_world(pose, point)
            hit = cell_of(wx, wy)
            hits.add(hit)
            if n % rays_every:
                continue
            steps = int(r / (CELL / 2))  # half a cell per sample, or a ray skips cells
            for k in range(1, steps):
                f = k / steps
                crossed.add(cell_of(x0 + (wx - x0) * f, y0 + (wy - y0) * f))
        # A bearing with no point within SCAN_RANGE: the LiDAR saw nothing that near (the console sends no
        # farther points), so the floor is open up to there. Half-degree bins, as the LDS-03's step.
        returned = {round(math.degrees(math.atan2(q[1], q[0])) * 2) % 720 for q in points}
        steps = int(SCAN_RANGE / (CELL / 2))
        for b in range(720):
            if b in returned or (b - 1) % 720 in returned and (b + 1) % 720 in returned:
                continue
            a = pose[2] + math.radians(b / 2)
            ex, ey = x0 + SCAN_RANGE * math.cos(a), y0 + SCAN_RANGE * math.sin(a)
            for k in range(1, steps):
                crossed.add(cell_of(x0 + (ex - x0) * k / steps, y0 + (ey - y0) * k / steps))
        crossed -= hits
        for c in self.robot_cells(pose):
            crossed.add(c)  # the robot stands there: it is floor
        score = self.score
        for c in list(score):
            v = score[c] * DECAY - (MISS if c in crossed else 0.0)
            if v < 0.05:
                del score[c]
            else:
                score[c] = v
        for c in hits:
            score[c] = min(CAP, score.get(c, 0.0) + HIT)
        self.free |= crossed
        self.version += 1

    @staticmethod
    def robot_cells(pose):
        i, j = cell_of(pose[0], pose[1])
        return [(i + di, j + dj) for di, dj in _disc(BODY_HALF)]

    def occupied(self):
        return {c for c, v in self.score.items() if v >= OCCUPIED}

    def blocked(self, robot_xy=None, clearance=CLEARANCE):
        cells = inflate(self.occupied(), clearance)
        if robot_xy is not None:
            i, j = cell_of(*robot_xy)
            cells |= {(i + di, j + dj) for di, dj in _disc(ROBOT_KEEP)}
        return cells


def _line_clear(a, b, ok):
    steps = max(1, int(math.dist(a, b) / (CELL / 2)))
    return all(ok(cell_of(a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)) for k in range(steps + 1))


def astar(grid, start, goal, robot_xy=None, blocked=None, limit=80000):
    """A walk [start, ..., goal] through seen floor keeping CLEARANCE from walls and ROBOT_KEEP from the
    robot, or None. The person may leave a start that is too close to something."""
    if blocked is None:
        blocked = grid.blocked(robot_xy)
    s, g = cell_of(*start), cell_of(*goal)
    escape = math.ceil(CLEARANCE / CELL) + 1

    def ok(c):
        return c in grid.free and c not in blocked

    def passable(c):
        return ok(c) or (abs(c[0] - s[0]) <= escape and abs(c[1] - s[1]) <= escape and c in grid.free)

    if not ok(g):
        return None
    came, cost = {s: None}, {s: 0.0}
    frontier = [(0.0, s)]
    steps = 0
    while frontier and steps < limit:
        _, c = heapq.heappop(frontier)
        steps += 1
        if c == g:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if not (di or dj):
                    continue
                n = (c[0] + di, c[1] + dj)
                if not passable(n) or (di and dj and not (passable((c[0] + di, c[1])) and passable((c[0], c[1] + dj)))):
                    continue
                new = cost[c] + (1.4142 if di and dj else 1.0)
                if new < cost.get(n, math.inf):
                    cost[n], came[n] = new, c
                    h = math.hypot(g[0] - n[0], g[1] - n[1])
                    heapq.heappush(frontier, (new + h, n))
    if g not in came:
        return None
    cells = []
    c = g
    while c is not None:
        cells.append(c)
        c = came[c]
    cells.reverse()  # start cell first
    points = [tuple(start)] + [centre(c) for c in cells[1:-1]] + [tuple(goal)]
    return _smooth(points, passable)


def _smooth(points, passable):
    """Drop the corners a straight line can cut (string pulling)."""
    out, i = [points[0]], 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not _line_clear(points[i], points[j], passable):
            j -= 1
        out.append(points[j])
        i = j
    return out


def path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def random_goal(grid, start, robot_xy, rng, near=NEAR, far=FAR, tries=40):
    """(goal, path) to a random reachable floor spot near to far metres from start, or None."""
    blocked = grid.blocked(robot_xy)
    spots = [c for c in grid.free if c not in blocked and near <= math.dist(centre(c), start) <= far]
    rng.shuffle(spots)
    for c in spots[:tries]:
        path = astar(grid, start, centre(c), blocked=blocked)
        if path is not None:
            return centre(c), path
    return None


class Person:
    """The virtual person: where they stand, the walk they are on, the word they are saying."""

    def __init__(self, x=1.0, y=0.0, th=math.pi, speed=WALK_SPEED):
        self.x, self.y, self.th, self.speed = x, y, th, speed
        self.path = []  # the points still to walk to
        self.goal = None
        self.word, self.word_until = None, -math.inf
        self.pointed, self.point_until = None, -math.inf  # the floor spot pointed at, and until when

    @property
    def walking(self):
        return bool(self.path)

    def place(self, x, y, th):
        self.x, self.y, self.th = x, y, th
        self.path, self.goal = [], None

    def go(self, path):
        self.path = [tuple(p) for p in path[1:]]
        self.goal = tuple(path[-1]) if path else None

    def halt(self):
        self.path, self.goal = [], None

    def face(self, x, y):
        self.th = math.atan2(y - self.y, x - self.x)

    def step(self, dt):
        left = self.speed * dt
        while left > 1e-9 and self.path:
            tx, ty = self.path[0]
            d = math.dist((self.x, self.y), (tx, ty))
            if d > 1e-9:
                self.th = math.atan2(ty - self.y, tx - self.x)
            if d <= left:
                self.x, self.y = tx, ty
                self.path.pop(0)
                left -= d
            else:
                self.x += (tx - self.x) * left / d
                self.y += (ty - self.y) * left / d
                left = 0.0
        if not self.path:
            self.goal = None

    def say(self, word, now):
        if word not in WORDS:
            raise ValueError("say one of %s" % ", ".join(WORDS))
        self.word, self.word_until = word, now + SAY_S

    def speaking(self, now):
        return self.word if now < self.word_until else None

    def point(self, spot, now, seconds=POINT_S):
        """Point at a floor spot (x, y) for `seconds`, facing it when standing; None stops pointing."""
        if spot is None:
            self.pointed, self.point_until = None, -math.inf
            return
        if not (len(spot) == 2 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in spot)):
            raise ValueError("point at [x, y]")
        self.pointed, self.point_until = (float(spot[0]), float(spot[1])), now + seconds
        if not self.walking:
            self.face(*self.pointed)

    def pointing(self, now):
        return list(self.pointed) if self.pointed is not None and now < self.point_until else None

    def state(self, now):
        """The datagram for the robot's virtual_operator.py."""
        return {"t": round(now, 3), "x": round(self.x, 4), "y": round(self.y, 4), "th": round(self.th, 4),
                "speaking": self.speaking(now), "pointing": self.pointing(now)}


class Robot:
    """What the console says about the run, as the auto loop reads it."""

    def __init__(self, running=False, virtual=False, record=None, gap=None, label=None):
        self.running, self.virtual, self.record, self.gap, self.label = running, virtual, record, gap, label

    def get(self, key, default=None):
        return (self.record or {}).get(key, default)

    @property
    def pose(self):
        return self.get("pose")

    @property
    def ended(self):
        return self.get("end")


def append_csv(path, row):
    path = Path(path)
    new = not path.exists()
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            writer.writeheader()
        writer.writerow({k: row.get(k) for k in CSV_FIELDS})


class Auto:
    """The auto loop: stand in front of the robot until it locks, say follow, walk 2 to 4 m, say stop when the
    robot has come close, wait, say back, wait until the way back is done, then again with a new run.

    actions: start_run() -> None or an error text (starts a virtual operator run on the console);
    stop_run() (the console's STOP). Any fault ends the loop with the console's STOP."""

    def __init__(self, person, grid, actions, csv_path, rng=None):
        self.person, self.grid, self.actions, self.csv_path = person, grid, actions, Path(csv_path)
        self.rng = rng or random.Random()
        self.step = "off"
        self.since = 0.0  # when the current step began
        self.message = None  # why the loop last stopped, or what it waits for
        self.episode = None
        self.episodes = []  # the rows written this session
        self.asked = None  # when start_run was last asked
        self.ended_at = None  # when the robot was last seen without a run
        self.history = []  # (t, x, y, th) of the robot for the stuck time
        self.link_lost = None  # since when the record's link is false
        self.last = None  # time of the previous tick

    @property
    def on(self):
        return self.step != "off"

    def start(self, now):
        self.step, self.since, self.message, self.asked = "starting", now, None, None
        self.episode = None

    def _go(self, step, now, message=None):
        self.step, self.since = step, now
        self.message = message

    def stop(self, now, result="หยุดโดยผู้ใช้", stop_robot=True):
        """End the loop: the person stops where they are, the run gets STOP, the episode is written."""
        if not self.on:
            return
        self.person.halt()
        if stop_robot:
            self.actions.stop_run()
        self._finish(now, result)
        self._go("off", now, result)

    def _finish(self, now, result):
        e = self.episode
        if e is None:
            return
        row = {"start": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(e["wall"])),
               "start_x": _r(e["start"][0]), "start_y": _r(e["start"][1]),
               "goal_x": _r(e["goal"][0]) if e["goal"] else None, "goal_y": _r(e["goal"][1]) if e["goal"] else None,
               "walk_m": _r(e["walk_m"]), "time_to_reach_s": _r(e["reach_s"], 1), "nearest_m": _r(e["nearest"]),
               "stuck_s": _r(e["stuck"], 1), "result": result}
        append_csv(self.csv_path, row)
        self.episodes.append(row)
        self.episode = None

    def fault(self, now, robot):
        """Why the loop must stop now, or None."""
        rec = robot.record or {}
        if robot.running and robot.record is not None:
            if rec.get("wheels"):
                return "ล้อมีปัญหา: %s" % rec["wheels"]
            if rec.get("link") is False and self.step in ("follow", "walk", "reach", "hold", "return"):
                self.link_lost = now if self.link_lost is None else self.link_lost
                if now - self.link_lost >= 2.0:
                    return "หุ่นไม่ได้รับคนเสมือนจาก PC (link ขาด)"
            else:
                self.link_lost = None
        if robot.gap is not None and robot.gap < TOO_CLOSE:
            return "หุ่นเข้าใกล้ของเกิน %.0f ซม. (%.3f m)" % (TOO_CLOSE * 100, robot.gap)
        cable = rec.get("cable_deg")
        if isinstance(cable, (int, float)) and abs(cable) > CABLE_LIMIT:
            return "สายไฟบิดเกิน %.0f องศา (%.0f)" % (CABLE_LIMIT, cable)
        if not robot.running and self.step not in ("starting", "return", "off"):
            return "งานจบเอง: %s" % (robot.ended or "ไม่ทราบเหตุ")
        return None

    def tick(self, now, robot):
        if not self.on:
            return
        dt = 0.0 if self.last is None else min(1.0, now - self.last)
        self.last = now
        why = self.fault(now, robot)
        if why:
            return self.stop(now, why)
        limit = LIMITS.get(self.step)
        if limit is not None and now - self.since > limit:
            return self.stop(now, "เกินเวลาที่ขั้น %s (%.0f s)" % (self.step, limit))
        self._watch(now, robot, dt)
        getattr(self, "_" + self.step)(now, robot)

    def _watch(self, now, robot, dt):
        """The episode's nearest gap and the seconds the robot stood still when it should have moved."""
        e, pose = self.episode, robot.pose
        if e is None or pose is None:
            return
        if robot.gap is not None:
            e["nearest"] = robot.gap if e["nearest"] is None else min(e["nearest"], robot.gap)
        self.history = [h for h in self.history if now - h[0] <= 1.0] + [(now, pose[0], pose[1], pose[2])]
        old = self.history[0]
        still = (now - old[0] >= 0.9 and math.dist(old[1:3], pose[:2]) < 0.02
                 and abs(math.atan2(math.sin(pose[2] - old[3]), math.cos(pose[2] - old[3]))) < 0.05)
        phase, stop = robot.get("phase"), robot.get("stop")
        far = math.dist(pose[:2], (self.person.x, self.person.y)) > REACHED
        should_move = ((self.step in ("walk", "reach") and phase == "follow" and far)
                       or (self.step == "return" and phase == "return"))
        if still and should_move and stop not in HELD:
            e["stuck"] += dt

    # ------------------------------------------------------------ the steps

    def _starting(self, now, robot):
        if robot.running:
            if not robot.virtual:
                return self.stop(now, "งานที่วิ่งอยู่ไม่ใช่งานคนเสมือน", stop_robot=False)
            if robot.ended is not None or robot.get("phase") == "done":
                return  # the last run is still closing
            if robot.pose is not None:
                self.episode = {"wall": time.time(), "start": (self.person.x, self.person.y), "goal": None,
                                "walk_m": None, "reach_s": None, "nearest": None, "stuck": 0.0, "t0": None}
                self._go("place", now)
            return
        self.ended_at = self.ended_at or now
        if now - self.ended_at < 4.0:  # the console frees the devices after a run; STOP blocks starts for 3 s
            return
        if self.asked is None or now - self.asked >= 10.0:
            self.asked = now
            error = self.actions.start_run()
            self.message = "กำลังเริ่มงาน" if not error else "เริ่มงานไม่ได้: %s" % error

    def _place(self, now, robot):
        x, y, th = robot.pose
        spot = (x + FRONT * math.cos(th), y + FRONT * math.sin(th))
        p = self.person
        if math.dist((p.x, p.y), spot) <= 0.15:
            p.halt()
            p.face(x, y)
            self.ended_at = None
            return self._go("lock", now, "รอหุ่นล็อก")
        if not p.walking:
            path = astar(self.grid, (p.x, p.y), spot, robot_xy=None)
            if path is None or path_length(path) > 8.0:
                p.place(spot[0], spot[1], th + math.pi)  # no way there on the map: put there
            else:
                p.go(path)

    def _lock(self, now, robot):
        if robot.get("locked"):
            self.person.say("follow", now)
            self._go("follow", now, "พูด follow")

    def _follow(self, now, robot):
        if robot.get("phase") != "follow":
            return
        p = self.person
        found = random_goal(self.grid, (p.x, p.y), robot.pose[:2], self.rng)
        if found is None:
            return self.stop(now, "ไม่มีที่ว่างให้เดิน 2 ถึง 4 m บนแผนที่")
        goal, path = found
        self.episode.update(start=(p.x, p.y), goal=goal, walk_m=path_length(path), t0=now)
        p.go(path)
        self._go("walk", now, "เดินไป (%.1f, %.1f)" % goal)

    def _walk(self, now, robot):
        if not self.person.walking:
            self._go("reach", now, "ถึงแล้ว รอหุ่นตามมา")

    def _reach(self, now, robot):
        x, y = robot.pose[:2]
        if math.dist((x, y), (self.person.x, self.person.y)) <= REACHED:
            self.episode["reach_s"] = now - self.episode["t0"]
            self.person.say("stop", now)
            self._go("hold", now, "พูด stop")

    def _hold(self, now, robot):
        if now - self.since >= HOLD_S:
            self.person.say("back", now)
            self._go("return", now, "พูด back รอหุ่นกลับ")

    def _return(self, now, robot):
        done = robot.ended is not None and str(robot.ended).startswith("done") or robot.get("phase") == "done"
        if done:
            self._finish(now, "ok")
            self.ended_at = None
            self.asked = None
            self._go("starting", now, "รอบต่อไป")
        elif not robot.running:
            self.stop(now, "งานจบระหว่างกลับ: %s" % (robot.ended or "ไม่ทราบเหตุ"), stop_robot=False)


def _r(value, digits=3):
    return None if value is None else round(value, digits)
