"""Keep an id on every LiDAR candidate from scan to scan, in the odometry frame (it never jumps, so
a still thing stays where it is while the robot drives).

Each candidate gets a constant-velocity Kalman filter, one per axis (position and speed). A new scan's
points are matched to the tracks' predicted positions, nearest pairs first, inside a gate. A track
nobody matched coasts on its prediction and keeps its id up to KEEP: a leg pair or a bag that drops out
of a scan or a few (legs scissor, a dark bag returns no light now and then) comes back under the same id.

Each track also says how it moved: `far` is the farthest any of its measured points was from the
first one, `speed` the filter's speed. senses.py sorts people from furniture with these.
"""
import math

GATE = 0.40  # m a candidate may move between two scans (legs.GATE)
GATE_GROWTH = 0.8  # m added per second the track went unseen (legs.GATE_GROWTH)
OBJECT_GATE = 0.15  # m: a track the senses called an object only takes a point this close (senses.SET_DOWN_NEAR)
SCAN = 0.1  # s between scans (10 Hz)
KEEP = 0.6  # s a track may go unseen and keep its id (the old senses.SPOT_GAP: the bag's flicker in run 20261115-032329)
MEASURE_SD = 0.05  # m, one scan's leg-pair centre (side-on one leg hides the other: up to 0.1, legs.py tests)
ACCEL_SD = 1.5  # m/s^2: how hard a walking person speeds up, slows down or turns
SPEED_SD = 1.0  # m/s: a new track's speed is unknown, a person walks at most about this fast
MAX_SPEED = 2.0  # m/s: a faster filter speed is a mix-up of two candidates, not a person


class Track:
    def __init__(self, tid, point, now):
        self.id = tid
        self.axes = [[point[0], 0.0], [point[1], 0.0]]  # per axis: position, speed
        self.cov = [[[MEASURE_SD ** 2, 0.0], [0.0, SPEED_SD ** 2]] for _ in range(2)]
        self.t = now  # time of the filter state
        self.first = point  # where it was first seen
        self.born = now
        self.at = point  # its last measured point
        self.seen_at = now
        self.run_since = now  # start of its current run of scans without a miss
        self.far = 0.0  # farthest a measured point was from `first`
        self.object = False  # set by the senses: a still thing, not a person; it takes points only OBJECT_GATE away

    @property
    def pos(self):
        return self.axes[0][0], self.axes[1][0]

    @property
    def speed(self):
        return math.hypot(self.axes[0][1], self.axes[1][1])

    def predict(self, now):
        dt = now - self.t
        if dt <= 0:
            return
        q = ACCEL_SD ** 2
        for i in range(2):
            p, v = self.axes[i]
            (a, b), (_, d) = self.cov[i]
            self.axes[i] = [p + v * dt, v]
            a, b, d = a + 2 * b * dt + d * dt * dt, b + d * dt, d  # F P F'
            a, b, d = a + q * dt ** 4 / 4, b + q * dt ** 3 / 2, d + q * dt ** 2  # + Q (white acceleration)
            self.cov[i] = [[a, b], [b, d]]
        self.t = now

    def correct(self, point, now):
        r = MEASURE_SD ** 2
        for i in range(2):
            p, v = self.axes[i]
            (a, b), (_, d) = self.cov[i]
            s = a + r
            k0, k1 = a / s, b / s
            y = point[i] - p
            self.axes[i] = [p + k0 * y, v + k1 * y]
            self.cov[i] = [[(1 - k0) * a, (1 - k0) * b], [(1 - k0) * b, d - k1 * b]]
        if self.speed > MAX_SPEED:
            scale = MAX_SPEED / self.speed
            for axis in self.axes:
                axis[1] *= scale
        self.at = point
        self.seen_at = now
        self.far = max(self.far, math.dist(point, self.first))

    def gate(self, now):
        if self.object:
            return OBJECT_GATE
        return GATE + GATE_GROWTH * max(0.0, now - self.seen_at - SCAN)


class Tracks:
    def __init__(self):
        self.tracks = {}  # id -> Track, every track alive (seen this scan or coasting)
        self._next_id = 1
        self._last = None  # time of the previous update

    def __contains__(self, tid):
        return tid in self.tracks

    def __getitem__(self, tid):
        return self.tracks[tid]

    def update(self, now, points):
        """Match this scan's odometry-frame points to the tracks. Returns {id: point} in the order of `points`."""
        for track in self.tracks.values():
            track.predict(now)
        pairs = sorted((math.dist(track.pos, pt), tid, j) for tid, track in self.tracks.items()
                       for j, pt in enumerate(points) if math.dist(track.pos, pt) <= track.gate(now))
        given, used = {}, set()
        for _, tid, j in pairs:  # nearest pairs first: a global greedy assignment
            if j in given or tid in used:
                continue
            given[j] = tid
            used.add(tid)
        ids = {}
        for j, pt in enumerate(points):
            tid = given.get(j)
            if tid is None:
                tid, self._next_id = self._next_id, self._next_id + 1
                self.tracks[tid] = Track(tid, pt, now)
            else:
                track = self.tracks[tid]
                if self._last is None or track.seen_at < self._last:
                    track.run_since = now  # it missed a scan: a new run starts
                track.correct(pt, now)
            ids[tid] = pt
        for tid in [tid for tid, track in self.tracks.items() if now - track.seen_at > KEEP]:
            del self.tracks[tid]
        self._last = now
        return ids
