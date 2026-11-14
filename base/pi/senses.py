"""Fuse the LiDAR track, the camera people and the obstacles into the brain's Senses.

The LiDAR track says where the operator is; the camera only says who someone is. A camera
person can start a track, but only when the Jetson link is alive and their look matches.
"""
import math

from hmc.follower import Senses

import geometry
from legs import GATE, MAX_WIDTH, LegTracker, candidates
from shapes import ObstacleMemory

FRESH = 0.5  # s; an older camera answer is dropped, never matched
MATCH_DEG = 8.0  # a camera person and a LiDAR candidate this close in bearing are the same
LOOK_MATCH = 0.65  # score a look needs to start a track (hmc.follower.OPERATOR_MATCH)
CROWDED = 0.35  # m; another candidate this close to the operator makes the cycle ambiguous
ENROL_RANGE = 2.5  # m
PEOPLE_RANGE = (0.3, 4.0)  # m from the base
YAW_KEEP = 2.5  # s of yaw history, enough to correct any answer still used
LOOK_KEEP = 2.0  # s a camera look can still veto a fix that ends a LiDAR gap
UNSURE_LATCH = 1.0  # s ambiguous stays True after the last crowded cycle
LEARNED = 20  # looks the brain must have learned before the camera may re-lock (hmc.follower.OPERATOR_LEARN)
COMPANY = MAX_WIDTH  # m; someone this close to the operator can become one LiDAR object with them
# s an id must live before its vanishing or appearing next to the operator counts. In a review fuzz
# (600 runs, link down) 0.3 and 0.6 s both left 2 silent stranger follows, but a lone walker's own
# stride (a sine-leg model, not real data) dropped the track in 51 of 96 gaits at 0.3 s, 6 at 0.6 s.
COMPANY_MIN = 0.6
# Furniture is not a person (chair and table legs look like legs). A candidate counts as a person
# once it moved MOVED from where it was first seen, or the camera saw a person at it in the last
# CAMERA_KEEP, or it was first seen within COMPANY of the operator (it may have come out of them).
# Until then it is an obstacle for the brain and it cannot crowd the operator. It still counts for
# the company rules (vanished, split): a still stranger the operator walks into must drop the track
# (review 2026-11-14), and a track fix that lands where furniture stood is refused.
MOVED = 0.25  # m
CAMERA_KEEP = 2.0  # s
STATIC_NEAR = 0.1  # m; a candidate this close to last cycle's furniture is that furniture
# Something set down next to the operator (run 20261115-032329: the gift bag 0.31 m from their foot held
# the robot 'crowded' for 13.7 s) is first seen within COMPANY of them, so a person by the rule above.
# It stops being one once it stood still SET_DOWN_S while the camera, answering fresh with that spot
# in view, never saw a person there; its spot is remembered SET_DOWN_KEEP so that a flickering object
# with a new id is furniture at once. With the link down nothing is demoted: no camera to ask.
SET_DOWN_S = 2.0  # s
SET_DOWN_MOVED = 0.10  # m: it moved less than this from where it was first seen
SET_DOWN_KEEP = 10.0  # s
SET_DOWN_NEAR = 0.15  # m: a candidate this close to a remembered spot is that object
SPOT_GAP = 0.6  # s a spot may go unseen (the object flickers in the scan) and still collect its evidence


class RealSenses:
    def __init__(self, p):
        self.p = p
        self.tracker = self._new_tracker()
        self.memory = ObstacleMemory(p)
        self.ambiguous = False  # True: someone else is right at the operator, the caller should hold
        self.why = None
        self._unsure_at = -math.inf  # last cycle the operator was crowded
        self._answer = None  # newest camera answer, for the look check after a gap
        self._last_now = None  # time of the previous read (or enrol)
        self.learned = 0  # the loop sets this to Follower.operator_look.count every cycle
        self._pose = (0.0, 0.0, 0.0)
        self._yaws = []  # (time, yaw) from the poses passed to read
        self._people = {}  # id -> odometry point, last cycle
        self._next_id = 1
        self._born = {}  # id -> when that person was first seen
        self._first = {}  # id -> (odometry point, time) where and when that candidate was first seen
        self._person = set()  # ids that showed they are people (moved, seen by the camera, or came out of the operator)
        self._camera_at = {}  # id -> last time the camera saw a person at it
        self._furniture = []  # odometry points of the candidates that are not people, last cycle
        self._came_out = set()  # ids that are people only because they were first seen next to the operator
        self._spots = []  # where 'came out' people stand: {"at", "seen", "watched_since", "person"}
        self._camera_fresh = False  # this cycle has a fresh camera answer over a live link
        self._set_down = []  # (odometry point, time) of objects demoted as set down next to the operator
        self._operator_at = None  # odometry point of the operator at the last fix

    @property
    def locked(self):
        return self.tracker.pos is not None

    def enrol(self, now, scan, answer, accept=None):
        """Lock onto the one camera person within 2.5 m. Returns (odometry point, look) or a refusal.

        accept(look) -> bool, when given, must agree before the lock (a re-enrol checks the look).
        """
        if now - answer.t > FRESH:
            return "camera answer too old"
        if self.ambiguous:
            return "someone is right at the operator: wait"  # a fresh enrol has no look to check
        cands = candidates(scan)
        close = []
        for bearing, look in self._fresh(now, answer):
            c = self._match(bearing, cands)
            if c is not None and math.hypot(*self._base(c)) <= ENROL_RANGE:
                close.append((c, look))
        if len(close) > 1 or (close and self._crowded(close[0][0], cands)):
            return "more than one person within 2.5 m"
        if not close:
            return "nobody in view"
        c, look = close[0]
        if accept is not None and not accept(look):
            return "the person in view does not look like the operator"
        self.tracker = self._new_tracker()
        self.tracker.update(scan, now, lock_at=c)
        self._last_now = now
        return geometry.base_to_odom(self._base(c), self._pose), look

    def read(self, now, pose, v, w, dt, scan, depth_points, answer, link_alive, score):
        self.why = None  # which rule made this cycle unsure or dropped the track, for the log
        self._pose = pose
        self._yaws = [s for s in self._yaws if now - s[0] <= YAW_KEEP] + [(now, pose[2])]
        if link_alive and answer is not None:
            self._answer = answer
        seen = self._fresh(now, answer) if link_alive else []
        self._camera_fresh = link_alive and answer is not None and now - answer.t <= FRESH
        cands = candidates(scan)
        look = None

        self.tracker.predict(v, w, dt)
        if self.locked:
            predicted, last_fix = self.tracker.pos, self.tracker.seen_at
            pos = self.tracker.update(scan, now)  # coasts (no fix) up to track_coast_max
            fixed = pos is not None and self.tracker.seen_at == now
            if fixed and (not self._fix_ok(now, pos, predicted, last_fix, cands, score) or self._is_furniture(pos)):
                self.tracker = self._new_tracker()  # lost: only a camera look can lock again (rule 4)
                self.why = "fix rejected"
            elif fixed:
                at_track = [l for b, l in seen if abs(self._camera_bearing(pos) - b) <= MATCH_DEG]
                if self._crowded(pos, cands) or len(at_track) > 1:
                    self._unsure_at = now
                    self.why = "crowded" if self._crowded(pos, cands) else "two seen at the track"
                else:
                    look = next((l for b, l in seen if self._match(b, cands) == pos), None)
        unsure = now - self._unsure_at <= UNSURE_LATCH
        if not self.locked and not unsure and self.learned >= LEARNED:  # before that, only enrol may lock
            for bearing, l in sorted(seen, key=lambda s: -score(s[1])):
                c = self._match(bearing, cands)
                if c is None or score(l) < LOOK_MATCH:
                    continue
                two_seen = sum(abs(self._camera_bearing(c) - b) <= MATCH_DEG for b, _ in seen) > 1
                if self._crowded(c, cands) or two_seen:
                    self._unsure_at = now  # never lock onto a spot where two people stand (or look like one)
                    self.why = "no re-lock: crowded" if self._crowded(c, cands) else "no re-lock: two seen"
                    break
                self.tracker.update(scan, now, lock_at=c)
                look = l
                break

        fixed = self.locked and self.tracker.seen_at == now
        operator = geometry.base_to_odom(self._base(self.tracker.pos), pose) if fixed else None
        if operator is not None:
            self._operator_at = operator
        others = [c for c in cands if c != self.tracker.pos and PEOPLE_RANGE[0] <= math.hypot(*self._base(c)) <= PEOPLE_RANGE[1]]
        everybody = self._ids([geometry.base_to_odom(self._base(c), pose) for c in others])
        people = self._sort_out(now, dict(zip(everybody, others)), everybody, seen)
        pid_of = {c: pid for pid, c in zip(everybody, others) if pid in people}
        # A look goes to a person only when its bearing points at exactly one candidate (not _match's
        # nearest-range rule, made for the operator: it hands a passer-by's look to whoever stands at the
        # operator's range), and never a look that scores as the operator (the operator coasting off the track).
        paired = []
        for b, l in seen:
            near = [c for c in cands if abs(self._camera_bearing(c) - b) <= MATCH_DEG]
            if len(near) == 1 and near[0] in pid_of and score(l) < LOOK_MATCH:
                paired.append((near[0], l))
        twice = {c for c, _ in paired if sum(1 for d, _ in paired if d == c) > 1}
        looks = {pid_of[c]: l for c, l in paired if c not in twice}  # two camera people on one track: whose look is it?
        mixed = self._mixed_up(now, operator, everybody)  # furniture too: it may be a stranger who never moved
        if mixed:
            # They may now be one LiDAR object; when it splits, nobody can say which piece is the
            # operator (spec criterion 4). Drop the track and hold: only a camera look may lock again.
            self.tracker = self._new_tracker()
            self._unsure_at = now
            self.why = mixed
            operator = None

        self.ambiguous = now - self._unsure_at <= UNSURE_LATCH
        self._last_now = now
        if self.ambiguous:
            look, looks = None, {}  # never learn or score a look from a merged scene
        everyone = list(people.values()) + ([operator] if operator is not None else [])
        obstacles = self.memory.update(now, pose, scan, depth_points, everyone)
        return Senses(now, operator, look, people, obstacles, looks=looks)

    def _sort_out(self, now, cand_of, everybody, seen):
        """The people among all the candidates; the rest is furniture, kept for the next cycle."""
        track_bearing = self._camera_bearing(self.tracker.pos) if self.locked else None
        for pid, c in cand_of.items():
            if pid not in self._first:
                self._first[pid] = (everybody[pid], now)
                if self._operator_at is not None and math.dist(everybody[pid], self._operator_at) <= COMPANY                         and not self._at_set_down(everybody[pid], now):
                    self._person.add(pid)  # it may have come out of the operator
                    self._came_out.add(pid)
            for bearing, _ in seen:
                if track_bearing is not None and abs(bearing - track_bearing) <= MATCH_DEG:
                    continue  # that camera person is the operator
                if abs(self._camera_bearing(c) - bearing) <= MATCH_DEG:
                    self._camera_at[pid] = now  # every candidate at that bearing: the person may be the farther one
        for pid, c in cand_of.items():
            self._demote_if_set_down(pid, c, everybody[pid], now)
        people = {pid: pt for pid, pt in everybody.items() if self._is_person(pid, now)}
        self._furniture = [pt for pid, pt in everybody.items() if pid not in people]
        for gone in [pid for pid in self._first if pid not in everybody]:  # forget ids no longer seen
            self._first.pop(gone)
            self._person.discard(gone)
            self._camera_at.pop(gone, None)
            self._came_out.discard(gone)
        self._spots = [sp for sp in self._spots if now - sp["seen"] <= SPOT_GAP]
        return people

    def _demote_if_set_down(self, pid, c, at, now):
        """A 'came out of the operator' person that stood still while the camera watched its spot and saw
        nobody there is an object put down: furniture from now on, its spot remembered. The evidence is
        kept per spot, not per id: a flickering object gets a new id every time it comes back."""
        if pid not in self._came_out:
            return
        spot = next((sp for sp in self._spots if math.dist(sp["at"], at) <= SET_DOWN_NEAR), None)
        if spot is None:
            spot = {"at": at, "watched_since": None, "person": False}
            self._spots.append(spot)
        spot["seen"] = now
        start, _ = self._first[pid]
        if math.dist(start, at) >= SET_DOWN_MOVED or now - self._camera_at.get(pid, -math.inf) <= CAMERA_KEEP:
            spot["person"] = True  # it moved or the camera saw someone there: a person, for good
        if spot["person"]:
            self._came_out.discard(pid)
            return
        half = self.p["camera_hfov_deg"] / 2 - 2.0  # its spot must be inside the picture, with a margin
        if not (self._camera_fresh and abs(self._camera_bearing(c)) <= half):
            spot["watched_since"] = None
            return
        if spot["watched_since"] is None:
            spot["watched_since"] = now
        if now - spot["watched_since"] >= SET_DOWN_S:
            self._person.discard(pid)
            self._came_out.discard(pid)
            self._spots.remove(spot)
            self._set_down.append((spot["at"], now))

    def _at_set_down(self, at, now):
        self._set_down = [(pt, t) for pt, t in self._set_down if now - t <= SET_DOWN_KEEP]
        return any(math.dist(at, pt) <= SET_DOWN_NEAR for pt, _ in self._set_down)

    def _mixed_up(self, now, operator, people):
        """True when the LiDAR may have mixed the operator up with someone next to them.

        Vanished: someone within COMPANY of the operator is gone from the scan. They merged into
        the operator's object or hid behind them; walking away keeps the id (less than GATE a scan).
        Split: someone appeared within COMPANY of the operator and stayed. An object that was the
        two of them came apart, and the track may be on either piece.
        Both need an id alive COMPANY_MIN, so a leg the pairing missed for a scan or two does not count.
        """
        for pid in people:
            self._born.setdefault(pid, self._first[pid][1] if pid in self._first else now)
        doubt = None
        if operator is not None:  # no fix this cycle: keep what we knew, judge on the next fix
            near = {pid for pid, pt in people.items() if math.dist(pt, operator) <= COMPANY}
            if self._fixed_before:  # the first fix of a track has nothing to compare with
                self._born_near |= {pid for pid in near if self._born[pid] == now}
            old = [pid for pid, t in self._born.items() if now - t >= COMPANY_MIN]
            vanished = any(pid not in people for pid in self._company if pid in old)
            split = any(pid in people for pid in self._born_near if pid in old)
            doubt = "someone next to the operator vanished" if vanished else ("split next to the operator" if split else None)
            self._company, self._fixed_before = near, True
            self._born_near = {pid for pid in self._born_near if pid in people}
        self._born = {pid: t for pid, t in self._born.items() if pid in people or pid in self._company}
        return doubt

    def _new_tracker(self):
        self._company = set()  # ids within COMPANY of the operator at the last fix
        self._born_near = set()  # ids that first appeared within COMPANY of the operator
        self._fixed_before = False
        return LegTracker(lost_after=self.p["track_coast_max"])

    def _fix_ok(self, now, pos, predicted, last_fix, cands, score):
        """May the track take this fix? Any jump, or a doubtful fix after a gap, drops the track."""
        if math.dist(pos, predicted) > self.p["track_jump"]:
            return False
        if self._last_now is None or last_fix >= self._last_now:
            return True  # the previous cycle had a fix too: no gap
        if pos != min(cands, key=lambda c: math.dist(c, predicted)):
            return False
        for bearing, l in self._fresh(now, self._answer, LOOK_KEEP):
            if abs(self._camera_bearing(pos) - bearing) <= MATCH_DEG and score(l) < LOOK_MATCH:
                return False  # the camera says the one at the track now is someone else
        return True

    def _base(self, c):
        return geometry.lidar_to_base(c, self.p)

    def _camera_bearing(self, c):
        # Camera bearings are measured at the camera, which is not at the laser: compare in the
        # base frame, with the candidate's bearing as seen from the camera position.
        x, y = self._base(c)
        return math.degrees(math.atan2(y - self.p["camera_y"], x - self.p["camera_x"]))

    def _match(self, bearing, cands):
        """The LiDAR candidate for a camera bearing, or None."""
        near = [c for c in cands if abs(self._camera_bearing(c) - bearing) <= MATCH_DEG]
        if len(near) > 1 and self.tracker.pos is not None:
            track_range = math.hypot(*self.tracker.pos)
            return min(near, key=lambda c: abs(math.hypot(*c) - track_range))
        return min(near, key=lambda c: abs(self._camera_bearing(c) - bearing), default=None)

    def _crowded(self, point, cands):
        return any(0 < math.dist(c, point) <= CROWDED and not self._is_furniture(c) for c in cands)

    def _is_furniture(self, c):
        """A lidar-frame candidate standing where a non-person candidate stood last cycle."""
        at = geometry.base_to_odom(self._base(c), self._pose)
        return any(math.dist(at, f) <= STATIC_NEAR for f in self._furniture) or self._at_set_down(at, self._last_now or 0.0)

    def _is_person(self, pid, now):
        """Has the candidate with this id shown it is a person? Latched: once a person, always."""
        if pid not in self._person:
            (start, _), at = self._first[pid], self._people[pid]
            if math.dist(start, at) >= MOVED or now - self._camera_at.get(pid, -math.inf) <= CAMERA_KEEP:
                self._person.add(pid)
        return pid in self._person

    def _fresh(self, now, answer, max_age=FRESH):
        """(bearing now, look) for each camera person, or [] when the answer is missing or late."""
        if answer is None or now - answer.t > max_age:
            return []
        # shortcut: only the turn is corrected, not the drive; fine for 0.5 s answers, rough for the 2 s look veto.
        yaws = self._yaws or [(now, 0.0)]  # no read yet: no turn known
        then = [yaw for t, yaw in yaws if t <= answer.t] or [yaws[0][1]]
        turn = yaws[-1][1] - then[-1]
        turn = math.degrees(math.atan2(math.sin(turn), math.cos(turn)))
        return [(s.bearing_deg - turn, s.look) for s in answer.people]

    def _ids(self, points):
        """Give each point the id of the nearest person last cycle (within GATE), else a new id."""
        # shortcut: greedy in list order, not a best global assignment; fine for a few people.
        old, ids = dict(self._people), {}
        for pt in points:
            pid = min(old, key=lambda i: math.dist(old[i], pt), default=None)
            if pid is None or math.dist(old[pid], pt) > GATE:
                pid, self._next_id = self._next_id, self._next_id + 1
            else:
                del old[pid]
            ids[pid] = pt
        self._people = ids
        return ids
