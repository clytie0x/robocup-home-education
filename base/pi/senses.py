"""Fuse the LiDAR track, the camera people and the obstacles into the brain's Senses.

The LiDAR track says where the operator is; the camera only says who someone is. A camera
person can start a track, but only when the Jetson link is alive and their look matches.
Everyone else in the scan keeps an id from scan to scan (tracks.Tracks, a Kalman filter each).
A camera person is matched to the LiDAR by bearing, and by range where the depth image gives one.
"""
import math

import numpy as np

from hmc.follower import Senses

import geometry
from legs import MAX_WIDTH, LegTracker, candidates
from shapes import ObstacleMemory
from tracks import KEEP, Tracks

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
# The tracks keep an id through a missed scan. Split counts from the last missed scan (Track.run_since), as
# when every missed scan gave a new id, and so does vanished for an id first seen next to the operator (a
# leg of a long stride paired alone comes and goes there). Vanished for someone who walked up from WALKED_IN or
# farther counts their whole life (Track.born): one missed scan just before they merge must not hide them.
# Vanished for someone who walked up (moved MOVED) to within CROWDED of the operator at the last fix counts
# at any age: no leg of the operator's own stride stands that close to the track (legs.PAIR pairs legs
# nearer than 0.5 m), so it is someone else, and the track may just have taken their candidate.
# Scratch fuzz 2026-11-15 (senses only, robot still, a model, not real data): of 200 merge scenes with the
# link down, 10 rode the stranger before these tracks and 5 after; vanished counted from born for everyone
# gave 0, but held a lone walker with a 0.3 to 0.6 m stride in 21 of 150 walks (0 now, 0 before).
COMPANY_MIN = 0.6
# Revealed. An id born next to a still operator where the previous scan already had a return at its range is
# no one coming apart from them: the robot's own move (turning toward them on arrival) cut a leg-sized piece out
# of something that stood there all along. Run 20261115-023855 at 160 s: a cabinet edge 0.59 m from the operator
# became a new id that way, the split rule dropped the track and the robot searched for a person in front of it
# (tests/test_follow_scenes.py). A stranger coming apart from a still operator was their object before, so the
# vanished rule dropped the track when they merged; a split needs the operator to have moved.
STILL = 0.10  # m the operator's fix moved since the previous one
SEEN_THERE = 0.10  # m: a previous return this much past the new id's range showed its spot free
SEEN_DEG = 1.5  # degrees each side of the new id's bearing searched for such a return
WALKED_IN = 0.7  # m: first seen this far from the operator, it is someone else, not a leg of a long stride (up to
# 0.6 m apart in the scratch walker model, and the track may sit on either leg). It was 1.0, and the 5 scenes
# above were strangers first seen 0.64 to 0.99 m away. In the walker (2 x 150 walks) every id that came within
# COMPANY was first seen at most 0.56 m from the operator; WALKED_IN 0.6 held one walk of 150, 0.7 none.
# Furniture is not a person (chair and table legs look like legs). A candidate counts as a person
# once it moved MOVED from where it was first seen, or the camera saw a person at it in the last
# CAMERA_KEEP, or it was first seen within COMPANY of the operator (it may have come out of them).
# Until then it is an obstacle for the brain and it cannot crowd the operator. It still counts for
# the company rules (vanished, split): a still stranger the operator walks into must drop the track
# (review 2026-11-14), and a track fix that lands where furniture stood is refused.
MOVED = 0.25  # m
CAMERA_KEEP = 2.0  # s
STATIC_NEAR = 0.1  # m; a candidate this close to last cycle's furniture is that furniture
# Objects. Something set down next to the operator (run 20261115-032329: the gift bag 0.31 m from their
# foot held the robot 'crowded' for 13.7 s) is first seen within COMPANY of them, so a person by the rule
# above. Any candidate, that one or furniture, becomes an object once it stood still (never SET_DOWN_MOVED
# from where it was first seen) for SET_DOWN_S while the camera, answering fresh, had its spot in clear
# view (_in_clear_view) and never saw a person there. An object is furniture for good (until it moves or
# the camera sees someone there); while the link is alive it no longer counts for the company rules, so
# a bag that stays put does not drop the track. While that evidence is being collected, a company rule
# set off by it holds the robot instead of dropping the track. An object's spot is remembered
# SET_DOWN_KEEP, so the same thing under a new id (after a gap longer than tracks.KEEP) is an object at
# once. With the link down nothing becomes an object: no camera to ask.
SET_DOWN_S = 2.0  # s
SET_DOWN_MOVED = 0.10  # m: it moved less than this from where it was first seen
SET_DOWN_KEEP = 10.0  # s
SET_DOWN_NEAR = 0.15  # m: a candidate this close to a remembered spot is that object
# Clear view: a person standing there would be in the picture with their whole width (BODY_HALF each side
# of the bearing) and from the floor up to WATCH_HEIGHT (from camera_height and camera_pitch_deg in
# params.json: camera_height 0.07 there, about 0.105 measured, so the 0.07 asks for more range: 1.18 m),
# and no candidate more than OCCLUDE_GAP nearer the camera hides their middle (their bearing inside its
# BODY_HALF). Run 20261115-032329 at 180 s: the gift bag 11.6 deg beside the operator, 0.19 m farther, is
# in clear view (the operator's half width at 1.14 m is 10 deg); something 0.15 m behind them, 3 deg off, is not.
BODY_HALF = 0.2  # m, half a person's width at the legs and hips, what this camera sees of them
WATCH_HEIGHT = 0.9  # m: legs and hips in the picture
OCCLUDE_GAP = 0.05  # m: nearer than this is beside, not in front (LiDAR centres move a few cm scan to scan)
# Range. A camera person's range is the depth inside their box (camera frames carry the Xtion depth image):
# the NEAR_SHARE percentile of the valid depths in the middle half of the box width, between half and
# nine tenths of its height (legs and hips; the bottom tenth is floor). The low percentile reads the
# person, not the background between their legs; someone hidden behind a nearer person reads that
# person's range, which can only make two people look like one place, never one look like two.
NEAR_SHARE = 30  # percent
MIN_DEPTH_PIXELS = 20
DEPTH_MAX = 4.0  # m, the Xtion's depth is too rough beyond (PEOPLE_RANGE ends there too)
DEPTH_KEEP = 1.0  # s of depth images kept to find the one an answer was computed from (answers come in ~0.1 s)
# Two camera people within MATCH_DEG of the track are ambiguous, unless their ranges say clearly who
# is who: exactly one within RANGE_TOL of the track's range, every other one more than RANGE_FAR
# behind it (someone nearer than the track, or without a range, keeps it ambiguous).
RANGE_TOL = 0.3  # m
RANGE_FAR = 0.6  # m
# Merged. When a company rule drops the track, or a fix is refused with someone within COMPANY of the
# operator's last fix, the operator and someone else may be one LiDAR object there. Until the people seen
# there (the ids within COMPANY of it since) stand more than COMPANY apart, or nothing is there any more
# (tracks.KEEP), a camera re-lock onto a candidate there (or a re-enrol) needs the depth: the operator's
# range within RANGE_TOL of the candidate's, and nobody else the camera sees within COMPANY of it. By
# bearing alone the camera only says the operator is in that object, not that nobody else is. Scratch fuzz
# (200 merge scenes, camera on, the stranger not in its answer, 20 looks learned; a model, not real data):
# such re-locks drove on the stranger after the split in 42 scenes before, 0 now.
# Venue light (base/jetson/light.py, tools/light_bench.py). An answer whose frame the Jetson's light gate refused
# (link.Answer.good False: too dark, mostly saturated, or a flash) is no answer for who is who: no enrol, no
# re-lock, no look learned, compared or kept for the veto after a gap. It is not "the camera saw nobody there"
# either: an object's SET_DOWN_S evidence waits through such frames (it neither counts nor breaks), so a dark or
# flashing frame never turns a person into an object. The LiDAR track goes on as with the link down.
# A look counts only when the person's box was lit (link.Seen.light, its mean luma) within LIGHT_RATIO of the
# median light of the operator's learned looks (once LIGHT_MIN of them are in). light_bench, real frames, a
# colour-histogram stand-in for the look: a spotlight on the operator took the look from 1.0 to 0.33-0.35 at 2.4 to
# 3.4 times the light; of 9 lighting changes that put a look under LOOK_MATCH, this rule kept 6 out (the 3 it
# let in: a colour cast, noise of auto exposure at 0.1 of the light, a spotlight beside the body at 1.5 times).
# A look not counted is None: it never drops the operator in the follower nor vetoes a fix nor locks.
LIGHT_RATIO = 2.0  # light.LIGHT_RATIO: outside this, a look is not used to say who someone is
LIGHT_MIN = 5  # learned looks with a light before the rule applies
# Wider than LIGHT_RATIO. A box at 2.5 times the learned light (200 against 80) is the spotlight the
# identity tests keep out of a re-lock, but it is not "wait, wrong light" for enrolment: with the LiDAR
# track still continuous it can still count. Beyond this (a 5 times jump) the look is refused outright.
SOFT_RATIO = 3.5


def person_range(box, depth, p):
    """Metres from the camera to a camera person in the floor plane, from the depth in their box, or None.

    box is [x1, y1, x2, y2] in the pixels of the frame (link.Seen.box; mirrored, but the depth image is
    mirrored the same way), depth the uint16 millimetre image of that frame (0 = no reading).
    """
    height, width = depth.shape
    x1, y1, x2, y2 = box
    cols = int(max(0, math.floor(x1 + (x2 - x1) / 4))), int(min(width, math.ceil(x2 - (x2 - x1) / 4)))
    rows = int(max(0, math.floor(y1 + (y2 - y1) / 2))), int(min(height, math.ceil(y1 + (y2 - y1) * 0.9)))
    if cols[1] <= cols[0] or rows[1] <= rows[0]:
        return None
    patch = depth[rows[0]:rows[1], cols[0]:cols[1]]
    valid = patch[patch > 0]
    if valid.size < MIN_DEPTH_PIXELS or valid.size < 0.3 * patch.size:
        return None
    z = float(np.percentile(valid, NEAR_SHARE)) / 1000.0
    if z > DEPTH_MAX:
        return None
    f = (width / 2) / math.tan(math.radians(p["camera_hfov_deg"]) / 2)
    right = ((cols[0] + cols[1] - 1) / 2 - (width - 1) / 2) / f
    down = ((rows[0] + rows[1] - 1) / 2 - (height - 1) / 2) / f
    pitch = math.radians(p["camera_pitch_deg"])  # same camera model as depth_obstacles.points
    return math.hypot(z * math.cos(pitch) + down * z * math.sin(pitch), right * z)


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
        self.tracks = Tracks()  # everyone but the operator's fix, in the odometry frame
        self._person = set()  # ids that showed they are people (moved, seen by the camera, or came out of the operator)
        self._camera_at = {}  # id -> last time the camera saw a person at it
        self._furniture = []  # odometry points of the candidates that are not people, last cycle
        self._came_out = set()  # ids that are people only because they were first seen next to the operator
        self._walked_in = set()  # ids first seen farther than WALKED_IN from the operator's last fix and track
        self._watched = {}  # id -> since when the camera watched it stand still with nobody there
        self._camera_fresh = False  # this cycle has a fresh camera answer over a live link
        self._set_down = {}  # id -> (odometry point, time) where an object was last seen
        self._operator_at = None  # odometry point of the operator at the last fix
        self._depths = []  # (frame time, depth image) of the latest camera frames
        self._ranges = {}  # answer time -> range of each of its people (None where unknown)
        self._merged = None  # [odometry point, last time seen, ids seen there] where the operator may be merged
        self._lights = []  # box light of each look given while the brain learned (LIGHT_RATIO)
        self.camera_refused = None  # why the Jetson refused this cycle's fresh frame ("dark", ...), for the log
        self.enrol_how = None  # set when a lock used the LiDAR because the camera had no crop
        self._scan_before = None  # (scan, pose) of the previous read

    @property
    def locked(self):
        return self.tracker.pos is not None

    def enrol(self, now, scan, answer, accept=None):
        """Lock onto the one camera person within 2.5 m. Returns (odometry point, look) or a refusal.

        accept(look) -> bool, when given, must agree before the lock (a re-enrol checks the look).

        The low Xtion often returns no box while the LDS-03 still has legs. The first enrol (no accept)
        then locks the nearest leg inside the camera's view, and a look is stored only once a crop exists.
        """
        self.enrol_how = None
        if now - answer.t > FRESH:
            return "camera answer too old"
        if not getattr(answer, "good", True):
            return "camera frame refused (%s): wait" % getattr(answer, "why", None)
        if self.ambiguous:
            return "someone is right at the operator: wait"  # a fresh enrol has no look to check
        cands = candidates(scan)
        close = []
        fresh = self._fresh(now, answer)
        for person in fresh:
            c = self._match(person[0], person[2], cands)
            if c is not None and math.hypot(*self._base(c)) <= ENROL_RANGE:
                close.append((c, person))
        if len(close) > 1 or (close and self._crowded(close[0][0], cands)):
            return "more than one person within 2.5 m"
        if not close:
            # "nobody in view" on its own hid the cause: 0 detector boxes, or boxes that matched no LiDAR leg.
            if not answer.people and accept is None:
                locked = self._lidar_enrol(now, scan, cands)
                if locked is not None:
                    return locked
            if not answer.people:
                extra = getattr(answer, "empty", None)
                if not cands:
                    return "nobody in view" if not extra else "nobody in view: %s" % extra
                legs = "%d lidar leg%s" % (len(cands), "" if len(cands) == 1 else "s")
                return ("nobody in view: camera saw 0 (%s), %s, none ahead in the %.0f deg view"
                        % (extra or "no person box", legs, self.p.get("camera_hfov_deg", 58.0)))
            return ("nobody in view: camera saw %d, none within %.0f deg of a lidar person inside %.1f m"
                    % (len(answer.people), MATCH_DEG, ENROL_RANGE))
        c, person = close[0]
        _, look, _, light, _, raw, band = person
        if look is None and band == "soft" and raw:
            look = raw  # moderately different light: enrolment may take it, accept() still checks the look
        if self._merged_with(c) and not self._only_one_there(c, person, fresh):
            return "two people may be one object there: wait"
        if band == "hard":
            ref = float(np.median(self._lights)) if self._lights else 0.0
            box = 0.0 if light is None else light
            return "the operator is not in the light they were learned in: wait (box %.0f, learned %.0f)" % (box, ref)
        if accept is not None and look and not accept(look):
            return "the person in view does not look like the operator"
        self.tracker = self._new_tracker()
        self.tracker.update(scan, now, lock_at=c)
        self._last_now = now
        if look is None:
            self.enrol_how = "lidar track, camera crop had no look"
        return geometry.base_to_odom(self._base(c), self._pose), look

    def _lidar_enrol(self, now, scan, cands):
        """Lock the nearest leg in the Xtion's view when the detector returned no box.

        A re-enrol (accept is set) does not use this: without a look it could take a stranger's legs.
        """
        half = float(self.p.get("camera_hfov_deg", 58.0)) / 2.0
        ahead = [c for c in cands
                 if math.hypot(*self._base(c)) <= ENROL_RANGE and abs(self._camera_bearing(c)) <= half]
        if not ahead:
            return None
        ahead.sort(key=lambda c: math.hypot(*self._base(c)))
        c = ahead[0]
        if self._crowded(c, cands) or self._merged_with(c):
            return None
        self.enrol_how = "lidar track, camera saw 0 people, %d lidar leg%s, nearest ahead" % (
            len(cands), "" if len(cands) == 1 else "s")
        self.tracker = self._new_tracker()
        self.tracker.update(scan, now, lock_at=c)
        self._last_now = now
        return geometry.base_to_odom(self._base(c), self._pose), None

    def read(self, now, pose, v, w, dt, scan, depth_points, answer, link_alive, score, frame=None):
        """frame: this cycle's camera frame (camera.Frame), kept for the range of the people in its answer."""
        self.why = None  # which rule made this cycle unsure or dropped the track, for the log
        self._pose = pose
        self._yaws = [s for s in self._yaws if now - s[0] <= YAW_KEEP] + [(now, pose[2])]
        if frame is not None and getattr(frame, "depth", None) is not None:
            self._depths = [d for d in self._depths if frame.time - d[0] <= DEPTH_KEEP] + [(frame.time, frame.depth)]
        good = answer is None or getattr(answer, "good", True)
        if link_alive and answer is not None and good:
            self._answer = answer
        seen = self._fresh(now, answer) if link_alive and good else []
        self._camera_fresh = link_alive and answer is not None and good and now - answer.t <= FRESH
        refused = link_alive and answer is not None and not good and now - answer.t <= FRESH
        self.camera_refused = getattr(answer, "why", None) or "refused" if refused else None
        cands = candidates(scan)
        look = look_light = op_colour = op_soft = None

        self.tracker.predict(v, w, dt)
        if self.locked:
            predicted, last_fix = self.tracker.pos, self.tracker.seen_at
            pos = self.tracker.update(scan, now)  # coasts (no fix) up to track_coast_max
            fixed = pos is not None and self.tracker.seen_at == now
            if fixed and (not self._fix_ok(now, pos, predicted, last_fix, cands, score) or self._is_furniture(pos)):
                self.tracker = self._new_tracker()  # lost: only a camera look can lock again (rule 4)
                self.why = "fix rejected"
                if self._operator_at is not None and any(math.dist(geometry.base_to_odom(self._base(c), pose), self._operator_at) <= COMPANY for c in cands):
                    self._merged = [self._operator_at, now, set()]  # someone else stood where the operator was
            elif fixed:
                at_track = self._at_track(pos, seen)
                if self._crowded(pos, cands) or len(at_track) > 1:
                    self._unsure_at = now
                    self.why = "crowded" if self._crowded(pos, cands) else "two seen at the track"
                else:
                    hit = next((s for s in at_track if self._match(s[0], s[2], cands) == pos), None)
                    if hit is not None:
                        look, look_light, op_colour = hit[1], hit[3], hit[4]
                        if look is None and hit[6] == "soft":
                            op_soft = hit[5]  # not for identity: the follower may still count a continuous track
        unsure = now - self._unsure_at <= UNSURE_LATCH
        if not self.locked and not unsure and self.learned >= LEARNED:  # before that, only enrol may lock
            for person in sorted(seen, key=lambda s: -score(s[1]) if s[1] is not None else math.inf):
                bearing, l, rng = person[0], person[1], person[2]
                c = self._match(bearing, rng, cands)
                if c is None or l is None or score(l) < LOOK_MATCH:
                    continue
                if rng is not None and abs(self._camera_range(c) - rng) > RANGE_FAR:
                    continue  # the depth says that camera person stands nearer or farther than anyone there
                two_seen = len(self._at_track(c, seen)) > 1
                if self._crowded(c, cands) or two_seen:
                    self._unsure_at = now  # never lock onto a spot where two people stand (or look like one)
                    self.why = "no re-lock: crowded" if self._crowded(c, cands) else "no re-lock: two seen"
                    break
                if self._merged_with(c) and not self._only_one_there(c, person, seen):
                    self._unsure_at = now  # the operator is in that object, maybe not alone
                    self.why = "no re-lock: merged"
                    break
                self.tracker.update(scan, now, lock_at=c)
                look, look_light, op_colour = l, person[3], person[4]
                break

        fixed = self.locked and self.tracker.seen_at == now
        operator = geometry.base_to_odom(self._base(self.tracker.pos), pose) if fixed else None
        if operator is not None:
            self._operator_at = operator
        others = [c for c in cands if c != self.tracker.pos and PEOPLE_RANGE[0] <= math.hypot(*self._base(c)) <= PEOPLE_RANGE[1]]
        everybody = self.tracks.update(now, [geometry.base_to_odom(self._base(c), pose) for c in others])
        people = self._sort_out(now, dict(zip(everybody, others)), everybody, seen, cands)
        self._follow_merge(now, everybody, people)
        pid_of = {c: pid for pid, c in zip(everybody, others) if pid in people}
        # A look goes to a person only when its bearing points at exactly one candidate (not _match's
        # nearest-range rule, made for the operator: it hands a passer-by's look to whoever stands at the
        # operator's range), and never a look that scores as the operator (the operator coasting off the track).
        paired = []
        for person in seen:
            b, l = person[0], person[1]
            near = [c for c in cands if abs(self._camera_bearing(c) - b) <= MATCH_DEG]
            if len(near) == 1 and near[0] in pid_of and l is not None and score(l) < LOOK_MATCH:
                paired.append((near[0], l))
        twice = {c for c, _ in paired if sum(1 for d, _ in paired if d == c) > 1}
        looks = {pid_of[c]: l for c, l in paired if c not in twice}  # two camera people on one track: whose look is it?
        mixed, drop = self._mixed_up(now, operator, everybody)  # furniture too: it may be a stranger who never moved
        if mixed:
            # They may now be one LiDAR object; when it splits, nobody can say which piece is the
            # operator (spec criterion 4). Drop the track and hold: only a camera look may lock again.
            # A still thing the camera is still looking at only holds: it is an object or a person soon.
            if drop:
                self.tracker = self._new_tracker()
                self._merged = [operator, now, {pid for pid, pt in everybody.items() if math.dist(pt, operator) <= COMPANY}]
                operator = None
            self._unsure_at = now
            self.why = mixed

        self.ambiguous = now - self._unsure_at <= UNSURE_LATCH
        self._last_now = now
        self._scan_before = (scan, pose)  # for _was_there on the next cycle
        if self.ambiguous:
            look, looks = None, {}  # never learn or score a look from a merged scene
        if look is not None and look_light is not None:
            self._lights.append(look_light)  # rolling median, so a slow change of the room lights is learned too
            if len(self._lights) > LEARNED:
                del self._lights[0]
        everyone = list(people.values()) + ([operator] if operator is not None else [])
        obstacles = self.memory.update(now, pose, scan, depth_points, everyone)
        return Senses(now, operator, look, people, obstacles, looks=looks,
                      operator_colour=op_colour, soft_look=op_soft)

    def _sort_out(self, now, cand_of, everybody, seen, cands):
        """The people among the candidates measured this scan; the rest is furniture, kept for the next cycle."""
        at_operator = {id(s) for s in self._at_track(self.tracker.pos, seen)} if self.locked else set()
        operator_at = [self._operator_at] if self._operator_at is not None else []  # the last fix, and the track now
        if self.locked:
            operator_at.append(geometry.base_to_odom(self._base(self.tracker.pos), self._pose))
        for pid, c in cand_of.items():
            track = self.tracks[pid]
            if track.born == now:
                if all(math.dist(everybody[pid], at) > WALKED_IN for at in operator_at):
                    self._walked_in.add(pid)
                if self._at_set_down(everybody[pid], now):
                    track.object = True  # where an object stood a moment ago: that object again
                elif self._operator_at is not None and math.dist(everybody[pid], self._operator_at) <= COMPANY:
                    self._person.add(pid)  # it may have come out of the operator
                    self._came_out.add(pid)
            for s in seen:
                if id(s) in at_operator:
                    continue  # that camera person is the operator
                if self._at(c, s[0], s[2]):
                    self._camera_at[pid] = now  # every candidate there: the person may be the farther one
        for pid, c in cand_of.items():
            self._watch(pid, c, now, cands)
        people = {pid: pt for pid, pt in everybody.items() if self._is_person(pid, now)}
        alive = self.tracks.tracks
        for pid in [pid for pid in self._camera_at if pid not in alive]:  # forget ids no longer tracked
            self._camera_at.pop(pid)
        for pid in [pid for pid in self._watched if pid not in alive]:
            self._watched.pop(pid)
        self._person &= set(alive)
        self._came_out &= set(alive)
        self._walked_in &= set(alive)
        # Furniture seen this scan, and objects unseen this scan too (a bag flickers). Not other unseen
        # tracks: the operator's own candidate is left out of the tracks while the track holds it.
        self._furniture = [track.at for pid, track in alive.items()
                           if pid not in self._person and (track.seen_at == now or track.object)]
        return people

    def _watch(self, pid, c, now, cands):
        """A still candidate the camera watched SET_DOWN_S, its spot in clear view, and never saw a person
        at becomes an object (see SET_DOWN_S). Moving or a camera person there ends that for good."""
        track = self.tracks[pid]
        camera = now - self._camera_at.get(pid, -math.inf) <= CAMERA_KEEP
        if track.object:
            if track.far >= SET_DOWN_MOVED or camera:
                track.object = False  # it moved, or someone stands there: a person, never an object again
                self._person.add(pid)
                self._set_down.pop(pid, None)
            else:
                self._set_down[pid] = (track.at, now)
            return
        if track.far >= SET_DOWN_MOVED or camera or (pid in self._person and pid not in self._came_out):
            self._came_out.discard(pid)  # it moved, or the camera saw someone there: a person, for good
            self._watched.pop(pid, None)
            return
        if self.camera_refused:
            if pid in self._watched and self._last_now is not None:
                self._watched[pid] += now - self._last_now  # a refused frame: the evidence waits, no count, no break
            return
        if not self._in_clear_view(c, cands):
            self._watched.pop(pid, None)  # the evidence must be unbroken
            return
        since = self._watched.setdefault(pid, now)
        if now - since >= SET_DOWN_S:
            track.object = True
            self._person.discard(pid)
            self._came_out.discard(pid)
            self._watched.pop(pid)
            self._set_down[pid] = (track.at, now)

    def _in_clear_view(self, c, cands):
        """Would the camera, answering fresh now, see a person who stood at this lidar-frame candidate?"""
        if not self._camera_fresh:
            return False
        bearing, rng = self._camera_bearing(c), self._camera_range(c)
        half_v = math.atan(math.tan(math.radians(self.p["camera_hfov_deg"]) / 2) * 3 / 4)  # 4:3 frames
        top = math.radians(self.p["camera_pitch_deg"]) + half_v
        if rng > PEOPLE_RANGE[1] or rng * math.tan(top) < WATCH_HEIGHT - self.p["camera_height"]:
            return False
        if abs(bearing) > self.p["camera_hfov_deg"] / 2 - math.degrees(math.atan2(BODY_HALF, rng)):
            return False
        for d in cands:
            near = self._camera_range(d)
            if d != c and near < rng - OCCLUDE_GAP and abs(self._camera_bearing(d) - bearing) <= math.degrees(
                    math.atan2(BODY_HALF, near)):
                return False  # someone nearer stands in front of its middle
        return True

    def _at_set_down(self, at, now):
        self._set_down = {pid: (pt, t) for pid, (pt, t) in self._set_down.items() if now - t <= SET_DOWN_KEEP}
        return any(math.dist(at, pt) <= SET_DOWN_NEAR for pt, _ in self._set_down.values())

    def _mixed_up(self, now, operator, measured):
        """(why, drop): the LiDAR may have mixed the operator up with someone next to them.

        Vanished: someone within COMPANY of the operator is gone from the scan (or missed a scan since).
        They merged into the operator's object or hid behind them; walking away keeps the id (less than
        tracks.GATE a scan). Split: someone appeared within COMPANY of the operator and stayed. An object that was
        the two of them came apart, and the track may be on either piece.
        Both need an id COMPANY_MIN old (split: an unbroken run), so a leg the pairing missed for a scan or two does not count.
        An object (while the camera answers) never counts; a still thing the camera is watching to tell
        (SET_DOWN_S) only holds: drop is False then.
        """
        if operator is None:  # no fix this cycle: keep what we knew, judge on the next fix
            return None, False
        near = {}  # pid -> (start of its run, since when its vanishing counts)
        for pid, pt in measured.items():
            if math.dist(pt, operator) <= COMPANY:
                track = self.tracks[pid]
                walked_up = math.dist(pt, operator) <= CROWDED and track.far >= MOVED
                near[pid] = (track.run_since, -math.inf if walked_up else track.born if pid in self._walked_in else track.run_since)
        if self._fixed_before:  # the first fix of a track has nothing to compare with
            born = {pid for pid, (since, _) in near.items() if since == now}
            self._born_near |= born
            self._revealed |= {pid for pid in born if self._was_there(measured[pid], operator)}
        # a revealed piece goes as it came, with the view: no one merged or came apart. Its centre slides along
        # the thing it is cut from as the view changes, so `far` cannot tell; the camera can: once it saw a
        # person there, it counts like anyone.
        self._revealed -= set(self._camera_at)
        vanished = {pid for pid, (since, born) in self._company.items() if now - born >= COMPANY_MIN
                    and (pid not in measured or self.tracks[pid].run_since != since)} - self._revealed
        split = {pid for pid in self._born_near - self._revealed
                 if pid in measured and now - self.tracks[pid].run_since >= COMPANY_MIN}
        self._company, self._fixed_before, self._fix_before = near, True, operator
        self._born_near = {pid for pid in self._born_near if pid in measured}
        self._revealed &= self._born_near
        exempt = {pid for pid in vanished | split if self._camera_fresh and pid in self.tracks and self.tracks[pid].object}
        vanished, split = vanished - exempt, split - exempt
        if not vanished and not split:
            return None, False
        why = "someone next to the operator vanished" if vanished else "split next to the operator"
        if all(pid in self._watched for pid in vanished | split):
            return why + ": a still thing, the camera is looking", False
        return why, True

    def _follow_merge(self, now, measured, people):
        """Keep the merged spot on the people there as they move; forget it once they stand apart or left, or on a lock."""
        if self._merged is None:
            return
        if self.locked:
            self._merged = None
            return
        spot, seen_at, ids = self._merged
        near = {pid: pt for pid, pt in measured.items() if pid in people and math.dist(pt, spot) <= COMPANY}
        ids |= set(near)
        apart = [measured[pid] for pid in ids if pid in measured]
        if any(math.dist(a, b) > COMPANY for a in apart for b in apart):
            self._merged = None  # they came apart: the usual rules tell who is who
        elif near:
            self._merged = [min(near.values(), key=lambda pt: math.dist(pt, spot)), now, ids]
        elif now - seen_at > KEEP:
            self._merged = None  # gone from the scan

    def _merged_with(self, c):
        """Is lidar-frame candidate c the object the operator may be merged in?"""
        return self._merged is not None and math.dist(
            geometry.base_to_odom(self._base(c), self._pose), self._merged[0]) <= COMPANY

    def _only_one_there(self, c, person, seen):
        """Is this camera person (bearing, look, range) the only one the camera sees who could be in candidate
        c, and does the depth put them at its range?"""
        here, there = self._base(c), []
        for s in seen:
            r = s[2] if s[2] is not None else self._camera_range(c)  # no range: as far as the candidate
            at = (self.p["camera_x"] + r * math.cos(math.radians(s[0])), self.p["camera_y"] + r * math.sin(math.radians(s[0])))
            if math.dist(at, here) <= COMPANY:
                there.append(s)
        return (there == [person] and person[2] is not None
                and abs(person[2] - self._camera_range(c)) <= RANGE_TOL)

    def _was_there(self, at, operator):
        """With the operator standing still, did the previous scan show odometry point `at` free? Not free (a return
        there or nearer, or no return at all on those bearings) means whatever is there may have stood there."""
        if self._scan_before is None or self._fix_before is None or math.dist(operator, self._fix_before) > STILL:
            return False
        scan, pose = self._scan_before
        bx, by = geometry.odom_to_base(at, pose)
        lx = bx - self.p["lidar_x"]
        deg, rng = math.degrees(math.atan2(by, lx)), math.hypot(lx, by)
        return not any(abs((b - deg + 180.0) % 360.0 - 180.0) <= SEEN_DEG and m > rng + SEEN_THERE for b, m in scan)

    def _new_tracker(self):
        self._company = {}  # id -> (start of its run, start of its age) for the ids within COMPANY of the operator at the last fix
        self._born_near = set()  # ids that first appeared within COMPANY of the operator
        self._revealed = set()  # of those, the ones that stood there before the robot's move showed them (STILL)
        self._fixed_before = False
        self._fix_before = None  # the operator's odometry point at the previous fix
        return LegTracker(lost_after=self.p["track_coast_max"])

    def _fix_ok(self, now, pos, predicted, last_fix, cands, score):
        """May the track take this fix? Any jump, or a doubtful fix after a gap, drops the track."""
        if math.dist(pos, predicted) > self.p["track_jump"]:
            return False
        if self._last_now is None or last_fix >= self._last_now:
            return True  # the previous cycle had a fix too: no gap
        if pos != min(cands, key=lambda c: math.dist(c, predicted)):
            return False
        for person in self._fresh(now, self._answer, LOOK_KEEP):
            bearing, l = person[0], person[1]
            # bearing only: a range up to LOOK_KEEP old says little about where someone is now
            if l is not None and abs(self._camera_bearing(pos) - bearing) <= MATCH_DEG and score(l) < LOOK_MATCH:
                return False  # the camera says the one at the track now is someone else
        return True

    def _base(self, c):
        return geometry.lidar_to_base(c, self.p)

    def _camera_bearing(self, c):
        # Camera bearings are measured at the camera, which is not at the laser: compare in the
        # base frame, with the candidate's bearing as seen from the camera position.
        x, y = self._base(c)
        return math.degrees(math.atan2(y - self.p["camera_y"], x - self.p["camera_x"]))

    def _camera_range(self, c):
        x, y = self._base(c)
        return math.hypot(x - self.p["camera_x"], y - self.p["camera_y"])

    def _at(self, c, bearing, rng):
        """Can the camera person at this bearing and range (None: unknown) stand at lidar-frame candidate c?"""
        if abs(self._camera_bearing(c) - bearing) > MATCH_DEG:
            return False
        return rng is None or abs(self._camera_range(c) - rng) <= RANGE_FAR

    def _at_track(self, c, seen):
        """The camera people at lidar-frame point c: those within MATCH_DEG of its bearing, unless the
        ranges say clearly that only one of them stands there (RANGE_TOL, RANGE_FAR)."""
        window = [s for s in seen if abs(self._camera_bearing(c) - s[0]) <= MATCH_DEG]
        if len(window) > 1 and all(s[2] is not None for s in window):
            here = self._camera_range(c)
            at = [s for s in window if abs(s[2] - here) <= RANGE_TOL]
            behind = [s for s in window if s[2] > here + RANGE_FAR]
            if len(at) == 1 and len(at) + len(behind) == len(window):
                return at
        return window

    def _match(self, bearing, rng, cands):
        """The LiDAR candidate for a camera bearing (and range, None if unknown), or None."""
        near = [c for c in cands if abs(self._camera_bearing(c) - bearing) <= MATCH_DEG]
        if len(near) > 1 and rng is not None:
            return min(near, key=lambda c: abs(self._camera_range(c) - rng))
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
        if pid not in self._person and not self.tracks[pid].object:
            if self.tracks[pid].far >= MOVED or now - self._camera_at.get(pid, -math.inf) <= CAMERA_KEEP:
                self._person.add(pid)
        return pid in self._person

    def _light_band(self, light):
        """'ok' (compare the look), 'soft' (enrolment only, with a continuous track) or 'hard' (refuse).

        Unknown light, or fewer than LIGHT_MIN learned lights, is ok: there is nothing to compare with yet.
        """
        if light is None or len(self._lights) < LIGHT_MIN:
            return "ok"
        ref = float(np.median(self._lights))
        if ref <= 0:
            return "ok"
        ratio = max(float(light), 1e-6) / ref
        if 1 / LIGHT_RATIO <= ratio <= LIGHT_RATIO:
            return "ok"
        if 1 / SOFT_RATIO <= ratio <= SOFT_RATIO:
            return "soft"
        return "hard"

    def _light_ok(self, light):
        """Was a look taken in about the light of the operator's learned looks (LIGHT_RATIO)? Unknown: yes."""
        return self._light_band(light) == "ok"

    def _fresh(self, now, answer, max_age=FRESH):
        """(bearing now, look, range or None, box light or None) for each camera person, or [] when the answer is
        missing or late. The look is None when the box light is too far from the learned one (_light_ok)."""
        if answer is None or now - answer.t > max_age:
            return []
        # shortcut: only the turn is corrected, not the drive; fine for 0.5 s answers, rough for the 2 s look veto.
        yaws = self._yaws or [(now, 0.0)]  # no read yet: no turn known
        then = [yaw for t, yaw in yaws if t <= answer.t] or [yaws[0][1]]
        turn = yaws[-1][1] - then[-1]
        turn = math.degrees(math.atan2(math.sin(turn), math.cos(turn)))
        ranges = self._ranges_of(answer)
        out = []
        for s, r in zip(answer.people, ranges):
            light = getattr(s, "light", None)
            band = self._light_band(light)
            colour = getattr(s, "colour", None)
            raw = tuple(s.look) if s.look else None  # an empty look (re-id failed) is no look
            # identity look is None unless the light is close to the learned one: a spotlight must not
            # re-lock or veto. The raw look and the band stay, so enrolment can still use a soft one.
            out.append((s.bearing_deg - turn, raw if band == "ok" else None, r, light,
                        None if colour is None else tuple(colour), raw, band))
        return out

    def _ranges_of(self, answer):
        """The range of each camera person of this answer, from the depth image of its frame (None: unknown)."""
        if answer.t not in self._ranges:
            depth = next((d for t, d in self._depths if abs(t - answer.t) < 1e-6), None)
            ranges = [None] * len(answer.people)
            if depth is not None:
                ranges = [person_range(s.box, depth, self.p) if s.box is not None else None for s in answer.people]
            self._ranges = {t: r for t, r in self._ranges.items() if answer.t - t <= LOOK_KEEP}
            self._ranges[answer.t] = ranges
        return self._ranges[answer.t]
