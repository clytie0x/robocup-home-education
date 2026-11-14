"""Which bag the operator points at: pose keypoints + the operator's LiDAR position -> a floor point -> a bag.

Pure functions, no camera or GPU. Frames: robot base frame, x ahead, y left, z up, metres, floor z = 0.
Camera: pinhole with the horizontal field of view camera_hfov_deg, at (camera_x, camera_y, camera_height),
tilted up by camera_pitch_deg; the same model as pi/depth_obstacles.py. The image is mirrored (image right
is robot left), so a pose model, trained on unmirrored people, names the operator's right arm "left".

Depth approximation: one frame gives no depth per keypoint, so every keypoint is put where its camera ray
meets the vertical plane through the operator's LiDAR position that faces the camera (all of the body at the
operator's range). Good for an arm that points to the side; an arm aimed toward or away from the camera loses
that part of its direction (docs/pointing.md).
"""

import math
from dataclasses import dataclass, replace

NEEDS = ("camera_x", "camera_y", "camera_height", "camera_pitch_deg", "camera_hfov_deg")

# COCO keypoint indices, as the pose model names them
NOSE, L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST, L_HIP, R_HIP = 0, 5, 6, 7, 8, 9, 10, 11, 12
ARMS = {"left": (L_SHOULDER, L_ELBOW, L_WRIST, L_HIP), "right": (R_SHOULDER, R_ELBOW, R_WRIST, R_HIP)}

MIN_VIS = 0.5  # keypoint visibility (the model's score) needed for shoulder, elbow, wrist and both hips
MIN_ARM_DEG = 30.0  # angle at the shoulder between upper arm and the torso side (shoulder -> hip)
MAX_REACH = 3.0  # m from the operator; a floor point farther than this is a ray nearly level with the floor
STEEP_DEG = 30.0  # a ray at least this far below level gets full confidence; shallower rays get less


@dataclass
class Pointing:
    point: tuple  # (x, y) on the floor, robot frame
    conf: float  # 0..1: lowest keypoint visibility times how steeply the arm points down
    arm: str  # "left" / "right": the operator's own arm
    shoulder: tuple  # (x, y, z) of that shoulder
    operator: tuple  # (x, y) from the LiDAR
    wrist: tuple = None  # (x, y, z)


@dataclass
class Choice:
    index: int  # into the candidates
    xy: tuple
    side: str  # "left" / "right" / "middle" relative to the operator, None when there is one candidate
    say: str


def _focal(p, width):
    return (width / 2) / math.tan(math.radians(p["camera_hfov_deg"]) / 2)


def _axes(p, mirrored):
    """Camera forward, down and image-right as base-frame unit vectors (camera tilted up by the pitch)."""
    pitch = math.radians(p["camera_pitch_deg"])
    side = 1.0 if mirrored else -1.0  # mirrored: image right is robot left
    return (math.cos(pitch), 0.0, math.sin(pitch)), (math.sin(pitch), 0.0, -math.cos(pitch)), (0.0, side, 0.0)


def _origin(p):
    missing = [k for k in NEEDS if p.get(k) is None]
    if missing:
        raise ValueError("camera pose not measured yet: %s" % ", ".join(missing))
    return (p["camera_x"], p["camera_y"], p["camera_height"])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def ray(u, v, p, width, height, mirrored=True):
    """Pixel -> (camera origin, direction) in the base frame. The direction has unit length along the optical axis."""
    f = _focal(p, width)
    right, down = (u - (width - 1) / 2) / f, (v - (height - 1) / 2) / f
    fwd, dn, rt = _axes(p, mirrored)
    return _origin(p), tuple(a + down * b + right * c for a, b, c in zip(fwd, dn, rt))


def project(point, p, width, height, mirrored=True):
    """Base-frame (x, y, z) -> pixel (u, v); the inverse of ray()."""
    d = [a - b for a, b in zip(point, _origin(p))]
    fwd, dn, rt = _axes(p, mirrored)
    z, f = _dot(d, fwd), _focal(p, width)
    return _dot(d, rt) / z * f + (width - 1) / 2, _dot(d, dn) / z * f + (height - 1) / 2


def top_visible_height(distance, p, height=240, width=320):
    """Height above the floor of the top image row at `distance` m (level) in front of the camera."""
    up = math.radians(p["camera_pitch_deg"]) + math.atan((height - 1) / 2 / _focal(p, width))
    return p["camera_height"] + distance * math.tan(up)


def _angle_deg(a, b):
    na, nb = math.hypot(*a), math.hypot(*b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return math.degrees(math.acos(max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (na * nb)))))


def _off_line(q, a, b):
    """Distance in pixels from q to the line through a and b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return math.hypot(q[0] - a[0], q[1] - a[1])
    return abs(dx * (q[1] - a[1]) - dy * (q[0] - a[0])) / n


def _arm_out(kp, label, min_vis, min_arm_deg):
    """Angle at the shoulder when this arm (model label) is held out from the body, else None. Image pixels only."""
    s, e, w, h = (kp[i] for i in ARMS[label])
    other = kp[R_SHOULDER if label == "left" else L_SHOULDER]
    if min(s[2], e[2], w[2], kp[L_HIP][2], kp[R_HIP][2]) <= min_vis:
        return None
    angle = _angle_deg((e[0] - s[0], e[1] - s[1]), (h[0] - s[0], h[1] - s[1]))
    if angle <= min_arm_deg:
        return None
    hip_mid = ((kp[L_HIP][0] + kp[R_HIP][0]) / 2, (kp[L_HIP][1] + kp[R_HIP][1]) / 2)
    top = ((s[0] + other[0]) / 2, (s[1] + other[1]) / 2) if other[2] > min_vis else s
    if _off_line(w, hip_mid, top) <= _off_line(e, hip_mid, top):  # the forearm must go on away from the body
        return None
    return angle


def _on_body_plane(u, v, operator, p, width, height, mirrored):
    """Keypoint pixel -> 3D point on the vertical plane through the operator that faces the camera."""
    origin, d = ray(u, v, p, width, height, mirrored)
    nx, ny = operator[0] - origin[0], operator[1] - origin[1]
    n = math.hypot(nx, ny)
    if n < 1e-6:
        return None
    along = (d[0] * nx + d[1] * ny) / n
    if along <= 1e-6:
        return None
    t = n / along
    return tuple(o + t * c for o, c in zip(origin, d))


def pointing(kp, range_m, bearing_deg, p, width=320, height=240, mirrored=True, min_vis=MIN_VIS,
             min_arm_deg=MIN_ARM_DEG, max_reach=MAX_REACH):
    """One person's 17 [u, v, vis] keypoints + LiDAR range and bearing (robot frame) -> Pointing, or None."""
    if kp is None or len(kp) < 17:
        return None
    b = math.radians(bearing_deg)
    operator = (range_m * math.cos(b), range_m * math.sin(b))
    arms = [(a, label) for label in ARMS for a in [_arm_out(kp, label, min_vis, min_arm_deg)] if a is not None]
    if not arms:
        return None
    _, label = max(arms)  # both out: the one held farther from the body
    s_i, e_i, w_i, _ = ARMS[label]
    shoulder = _on_body_plane(kp[s_i][0], kp[s_i][1], operator, p, width, height, mirrored)
    wrist = _on_body_plane(kp[w_i][0], kp[w_i][1], operator, p, width, height, mirrored)
    if shoulder is None or wrist is None or wrist[2] >= shoulder[2] - 0.02:
        return None  # level or raised arm: never reaches the floor
    s = shoulder[2] / (shoulder[2] - wrist[2])
    point = (shoulder[0] + s * (wrist[0] - shoulder[0]), shoulder[1] + s * (wrist[1] - shoulder[1]))
    if math.hypot(point[0] - operator[0], point[1] - operator[1]) > max_reach:
        return None
    level = math.hypot(wrist[0] - shoulder[0], wrist[1] - shoulder[1])
    steep = min(1.0, math.sin(math.atan2(shoulder[2] - wrist[2], level)) / math.sin(math.radians(STEEP_DEG)))
    conf = min(kp[s_i][2], kp[e_i][2], kp[w_i][2]) * steep
    own = {"left": "right", "right": "left"}[label] if mirrored else label
    return Pointing(point, float(conf), own, shoulder, operator, wrist)


def _median(values):
    v = sorted(values)
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2


class Steady:
    """Median of the last `frames` points, reported only once the point stayed within `moves_m` for `window_s`.

    A frame without a pointing arm (None) is skipped, not a reset; points older than window_s drop out.
    """

    def __init__(self, window_s=1.0, frames=5, moves_m=0.25):
        self.window_s, self.frames, self.moves_m = window_s, frames, moves_m
        self._seen = []  # (t, Pointing)

    def add(self, t, found):
        if found is not None:
            self._seen.append((t, found))
        self._seen = [(ts, f) for ts, f in self._seen if ts >= t - self.window_s - 1e-9]
        if len(self._seen) < self.frames or self._seen[-1][0] - self._seen[0][0] < 0.9 * self.window_s - 1e-6:
            return None
        pts = [f.point for _, f in self._seen]
        if max(math.hypot(a[0] - b[0], a[1] - b[1]) for a in pts for b in pts) >= self.moves_m:
            return None
        last = [f for _, f in self._seen[-self.frames:]]
        point = (_median([f.point[0] for f in last]), _median([f.point[1] for f in last]))
        return replace(last[-1], point=point, conf=_median([f.conf for f in last]))


def bag_candidates(boxes, p, width=320, height=240, clusters=(), max_deg=5.0, mirrored=True):
    """Bag boxes [x1, y1, x2, y2] -> floor positions (x, y), robot frame.

    Bearing from the box centre. Range from the LiDAR cluster (x, y) closest in bearing, seen from the camera,
    within max_deg; without one, from the box bottom row on the floor, which leans on camera_height and pitch
    (at 2 m a 1 cm height error moves it about 0.3 m: use it only as a fallback).
    """
    found = []
    for x1, y1, x2, y2 in (b[:4] for b in boxes):
        origin, d = ray((x1 + x2) / 2, (y1 + y2) / 2, p, width, height, mirrored)
        bearing = math.atan2(d[1], d[0])
        near = [(abs(math.degrees(math.atan2(c[1] - origin[1], c[0] - origin[0]) - bearing)),
                 math.hypot(c[0] - origin[0], c[1] - origin[1]), tuple(c)) for c in clusters]
        near = [n for n in near if n[0] <= max_deg]
        if near:
            found.append(min(near)[2])
            continue
        origin, d = ray((x1 + x2) / 2, y2, p, width, height, mirrored)
        if d[2] < -1e-6:
            t = -origin[2] / d[2]
            found.append((origin[0] + t * d[0], origin[1] + t * d[1]))
    return found


def choose_bag(found, candidates, near_m=0.6):
    """The candidate within near_m of the pointed floor point that lies nearest the shoulder -> point ray.

    Score: distance from the ray (a half-line from the shoulder, on the floor plane) plus a quarter of the
    distance along it from the point, so two bags in line with the arm go to the one nearer the point.
    Side: relative to the operator, who is taken to face the robot.
    """
    if found is None or not candidates:
        return None
    px, py = found.point
    sx, sy = found.shoulder[0], found.shoulder[1]
    dx, dy = px - sx, py - sy
    n = math.hypot(dx, dy) or 1e-9
    dx, dy = dx / n, dy / n

    def score(c):
        along = (c[0] - sx) * dx + (c[1] - sy) * dy
        off = abs((c[0] - sx) * dy - (c[1] - sy) * dx) if along > 0 else math.hypot(c[0] - sx, c[1] - sy)
        return off + 0.25 * abs(along - n)

    near = [i for i, c in enumerate(candidates) if math.hypot(c[0] - px, c[1] - py) <= near_m]
    if not near:
        return None
    best = min(near, key=lambda i: score(candidates[i]))
    side = None
    if len(candidates) > 1:
        ox, oy = found.operator
        r = math.hypot(ox, oy) or 1e-9
        fx, fy = -ox / r, -oy / r  # facing the robot
        lat = [(c[0] - ox) * -fy + (c[1] - oy) * fx for c in candidates]  # + = the operator's left
        side = "left" if lat[best] == max(lat) else "right" if lat[best] == min(lat) else "middle"
    say = "I will take the %s bag, on your %s." % (side, side) if side in ("left", "right") else \
        "I will take the middle bag." if side else "I will take the bag."
    return Choice(best, tuple(candidates[best]), side, say)
