# Copied from xohm@pi:~/pi-drive/legs.py on 2026-11-13, adapted: chase and its constants dropped, usable from lidar, LIDAR_AHEAD from params, update takes lock_at (x, y) instead of a camera dict.
"""Keep hold of one person in the lidar scan, so the robot still follows when they leave the
camera view. The caller picks the person; the lidar (360 degrees, 10 Hz) keeps them.

Positions are metres in the lidar frame: x forward, y left, origin at the laser, which sits
LIDAR_AHEAD in front of the turning centre of the base.
"""

import math

import params
from lidar import usable

GAP = 0.10  # neighbouring points closer than this are one object
MIN_WIDTH = 0.03
MAX_WIDTH = 0.60  # one leg, two legs together, or a body
PAIR = 0.50  # two leg clusters closer than this are one person
MAX_RANGE = 4.0
GATE = 0.40  # how far the person may move between two scans
GATE_GROWTH = 0.8  # metres added per second without a match
LOCK = 0.60  # a lock point this close to a lidar candidate locks or re-locks onto it
LOST_AFTER = 1.5  # seconds without a lidar match before the person counts as lost
LIDAR_AHEAD = params.load()["lidar_x"]  # laser to turning centre, metres


def _xy(deg, meters):
    return meters * math.cos(math.radians(deg)), meters * math.sin(math.radians(deg))


def clusters(scan):
    """Small objects standing in front of what is behind them, as (x, y) centres."""
    points = sorted((deg, m) for deg, m in usable(scan))
    groups, current = [], []
    for deg, m in points:
        if current and math.dist(_xy(deg, m), _xy(*current[-1])) > GAP:
            groups.append(current)
            current = []
        current.append((deg, m))
    if current:
        groups.append(current)
    if len(groups) > 1 and math.dist(_xy(*groups[0][0]), _xy(*groups[-1][-1])) <= GAP:
        groups[0] = groups.pop() + groups[0]  # the object across the -180/180 seam
    centres = []
    for i, g in enumerate(groups):
        xy = [_xy(deg, m) for deg, m in g]
        length = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
        if len(g) < 2 or not MIN_WIDTH <= length <= MAX_WIDTH or min(m for _, m in g) > MAX_RANGE:
            continue
        # A piece of wall seen between two legs has nearer neighbours; a leg has further ones.
        before, after = groups[i - 1][-1][1], groups[(i + 1) % len(groups)][0][1]
        if len(groups) > 1 and (before < g[0][1] or after < g[-1][1]):
            continue
        centres.append((sum(x for x, _ in xy) / len(xy), sum(y for _, y in xy) / len(xy)))
    return centres


def candidates(scan):
    """People-sized objects: single clusters, with pairs of legs merged into one centre."""
    left = clusters(scan)
    people = []
    while left:
        a = left.pop(0)
        near = [b for b in left if math.dist(a, b) <= PAIR]
        if near:
            b = min(near, key=lambda c: math.dist(a, c))
            left.remove(b)
            a = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        people.append(a)
    return people


def moved(point, v, w, dt):
    """Where a still point appears after the base drove v m/s and turned w rad/s for dt s."""
    x, y = point[0] + LIDAR_AHEAD, point[1]  # to the turning centre
    x -= v * dt
    a = -w * dt
    x, y = x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)
    return x - LIDAR_AHEAD, y


class LegTracker:
    def __init__(self):
        self.pos = None
        self.seen_at = None

    def predict(self, v, w, dt):
        if self.pos is not None:
            self.pos = moved(self.pos, v, w, dt)

    def update(self, scan, now, lock_at=None):
        people = candidates(scan)
        if lock_at is not None:
            fix = lock_at
            near = [p for p in people if math.dist(p, fix) <= LOCK]
            if near and (self.pos is None or math.dist(self.pos, fix) > LOCK):
                self.pos, self.seen_at = min(near, key=lambda p: math.dist(p, fix)), now
                return self.pos
        if self.pos is None:
            return None
        gate = GATE + GATE_GROWTH * (now - self.seen_at)
        near = [p for p in people if math.dist(p, self.pos) <= gate]
        if near:
            self.pos, self.seen_at = min(near, key=lambda p: math.dist(p, self.pos)), now
        elif now - self.seen_at > LOST_AFTER:
            self.pos = self.seen_at = None
        return self.pos
