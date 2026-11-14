"""The AX1080P webcam under the SO-100's jaws: camera model, camera-to-gripper calibration fit, and
pluggable detectors (bag handle, calibration marker, floor-tile grout crossing).

No ROS. Frames come from the arm team's webcam_view.py (http://<jetson>:8082/snap.jpg); this module
never opens /dev/video0 itself (that would fight their program).

Camera frame: OpenCV convention (z along the optical axis, x right, y down in the image).
The pose of the camera is T_gc: camera frame -> gripper link frame (so100.urdf `gripper`).
"""
import json
import math
import os
import time
import urllib.request

import numpy as np

DEFAULT_FILE = os.path.expanduser("~/hmc-follow/gripper_camera.json")
WIDTH, HEIGHT = 640, 480
F_GUESS = 480.0          # px, about 67 deg horizontal field of view: fitted by the calibration
JAW_ROI = 0.82           # the jaw fills the bottom ~15 % of the image (owner's frame, 2026-11-15)


def rotvec_to_matrix(r):
    r = np.asarray(r, float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3)
    k = r / th
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * kx + (1 - math.cos(th)) * kx @ kx


def matrix_to_rotvec(m):
    m = np.asarray(m, float)
    c = max(-1.0, min(1.0, (np.trace(m) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if abs(th - math.pi) < 1e-6:
        w, v = np.linalg.eigh((m + np.eye(3)) / 2)
        return v[:, np.argmax(w)] * th
    return th / (2 * math.sin(th)) * np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])


def nominal_rotation(spin=0.0):
    """Camera looking along the tool axis (gripper -y); `spin` turns it about that axis."""
    z = np.array([0.0, -1.0, 0.0])
    x = np.array([math.cos(spin), 0.0, math.sin(spin)])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class CameraModel:
    def __init__(self, fx=F_GUESS, fy=None, cx=WIDTH / 2, cy=HEIGHT / 2, T_gc=None, width=WIDTH, height=HEIGHT,
                 rms_px=None, info=None):
        self.fx, self.fy = float(fx), float(fy if fy is not None else fx)
        self.cx, self.cy = float(cx), float(cy)
        self.width, self.height = int(width), int(height)
        if T_gc is None:
            T_gc = np.eye(4)
            T_gc[:3, :3] = nominal_rotation()
            T_gc[:3, 3] = (0.0, -0.03, -0.035)
        self.T_gc = np.asarray(T_gc, float)
        self.rms_px = rms_px
        self.info = info or {}

    # -------------------------------------------------------------------------- geometry
    def project_cam(self, p_cam):
        p = np.asarray(p_cam, float)
        if p[2] <= 1e-6:
            return None
        return np.array([self.fx * p[0] / p[2] + self.cx, self.fy * p[1] / p[2] + self.cy])

    def ray_cam(self, u, v):
        d = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        return d / np.linalg.norm(d)

    def camera_in_arm(self, kin, q):
        """4x4 camera pose in the ARM frame for joint angles q."""
        return kin.fk(q)["gripper"] @ self.T_gc

    def project(self, kin, q, p_robot):
        """Pixel (u, v) of a robot-frame point, or None when it is behind the camera."""
        T = self.camera_in_arm(kin, q)
        p_arm = kin.mount.to_arm(p_robot)
        p_cam = T[:3, :3].T @ (p_arm - T[:3, 3])
        return self.project_cam(p_cam)

    def pixel_to_plane(self, kin, q, u, v, plane_z):
        """Robot-frame point where the pixel's ray meets the horizontal plane z = plane_z (robot frame)."""
        T = self.camera_in_arm(kin, q)
        o = kin.mount.to_robot(T[:3, 3])
        far = kin.mount.to_robot(T[:3, 3] + T[:3, :3] @ self.ray_cam(u, v))
        d = far - o
        if abs(d[2]) < 1e-9:
            return None
        s = (plane_z - o[2]) / d[2]
        if s <= 0:
            return None
        return o + s * d

    def in_view(self, uv, roi_bottom=JAW_ROI):
        return uv is not None and 0 <= uv[0] < self.width and 0 <= uv[1] < self.height * roi_bottom

    # ------------------------------------------------------------------------------- file
    def to_dict(self):
        return {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy, "width": self.width,
                "height": self.height, "T_gripper_camera": self.T_gc.tolist(), "rms_px": self.rms_px,
                **self.info}

    def save(self, path=DEFAULT_FILE):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path=DEFAULT_FILE):
        with open(path) as f:
            d = json.load(f)
        info = {k: v for k, v in d.items()
                if k not in ("fx", "fy", "cx", "cy", "width", "height", "T_gripper_camera", "rms_px")}
        return cls(d["fx"], d["fy"], d["cx"], d["cy"], np.array(d["T_gripper_camera"]), d["width"], d["height"],
                   d.get("rms_px"), info)


# ------------------------------------------------------------------------- calibration fit
def _model_from(params, base):
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(params[3:6])
    T[:3, 3] = params[:3]
    f = abs(params[6])      # a negative focal length would mirror the image: same pixels, camera turned 180 deg
    return CameraModel(f, f, base.cx, base.cy, T, base.width, base.height)


def _residuals(params, kin, obs, base, frames):
    cam = _model_from(params, base)
    out = []
    for (q, p, uv), g in zip(obs, frames):
        T = g @ cam.T_gc
        p_cam = T[:3, :3].T @ (kin.mount.to_arm(p) - T[:3, 3])
        if p_cam[2] <= 1e-4:
            out += [1e3, 1e3]
            continue
        out += list(cam.project_cam(p_cam) - np.asarray(uv, float))
    return np.array(out)


def fit_camera(kin, observations, base=None, iterations=60):
    """Fit the camera pose in the gripper frame and its focal length to observations
    [(joint angles q, marker position in the robot frame, observed pixel (u, v))].
    Tries several start rotations about the tool axis and offsets, keeps the lowest error.
    Returns (CameraModel, rms in px)."""
    if len(observations) < 4:
        raise ValueError("need at least 4 marker sightings, got %d" % len(observations))
    base = base or CameraModel()
    frames = [kin.fk(q)["gripper"] for q, _p, _uv in observations]
    best = None
    for spin in np.radians(np.arange(0, 360, 45)):
        for t0 in ((0.0, -0.03, -0.035), (0.0, -0.03, 0.035), (0.03, -0.03, 0.0), (-0.03, -0.03, 0.0)):
            x = np.r_[t0, matrix_to_rotvec(nominal_rotation(spin)), base.fx]
            x, cost = _lm(lambda y: _residuals(y, kin, observations, base, frames), x, iterations)
            if best is None or cost < best[1]:
                best = (x, cost)
    x, cost = best
    cam = _model_from(x, base)
    cam.rms_px = float(math.sqrt(cost / len(observations)))
    return cam, cam.rms_px


def fit_camera_free(kin, sightings, base=None, iterations=80):
    """Like fit_camera, but the markers are floor points (robot z = 0) at unknown places, e.g. tile grout
    crossings nobody measured: [(joint angles q, marker key, observed pixel (u, v))]. The arm's own motion
    fixes the scale, so each marker's x, y is fitted along with the camera.
    Returns (CameraModel, rms in px, {key: fitted (x, y)})."""
    keys = sorted({k for _q, k, _uv in sightings}, key=str)
    if len(sightings) < 3 + 2 * len(keys):
        raise ValueError("need at least %d sightings of %d markers, got %d"
                         % (3 + 2 * len(keys), len(keys), len(sightings)))
    base = base or CameraModel()
    frames = [kin.fk(q)["gripper"] for q, _k, _uv in sightings]
    index = {k: i for i, k in enumerate(keys)}

    def residuals(x):
        cam = _model_from(x[:7], base)
        out = []
        for (q, k, uv), g in zip(sightings, frames):
            j = 7 + 2 * index[k]
            p = np.array([x[j], x[j + 1], 0.0])
            T = g @ cam.T_gc
            p_cam = T[:3, :3].T @ (kin.mount.to_arm(p) - T[:3, 3])
            if p_cam[2] <= 1e-4:
                out += [1e3, 1e3]
                continue
            out += list(cam.project_cam(p_cam) - np.asarray(uv, float))
        return np.array(out)

    steps = [1e-6] * 6 + [1e-3] + [1e-6] * (2 * len(keys))
    best = None
    starts = [(spin, tilt, t0) for tilt in np.radians((0, -30, 30, -60, 60)) for spin in np.radians(np.arange(0, 360, 45))
              for t0 in ((0.0, -0.03, -0.035), (0.0, -0.03, 0.035), (0.03, -0.03, 0.0), (-0.03, -0.03, 0.0))]
    for spin, tilt, t0 in starts:      # tilt: the camera may look well off the tool axis
        x0 = np.r_[t0, matrix_to_rotvec(nominal_rotation(spin) @ rotvec_to_matrix(np.array([tilt, 0, 0]))),
                   base.fx]
        start = _model_from(x0, base)
        guess = {k: [] for k in keys}
        for q, k, uv in sightings:            # each marker where the start model's rays meet the floor
            p = start.pixel_to_plane(kin, q, uv[0], uv[1], 0.0)
            if p is not None and np.all(np.isfinite(p)) and np.hypot(p[0], p[1]) < 2.0:
                guess[k].append(p[:2])
        if any(not g for g in guess.values()):
            continue
        x0 = np.r_[x0, np.concatenate([np.mean(guess[k], axis=0) for k in keys])]
        x, cost = _lm(residuals, x0, iterations, steps)
        if best is None or cost < best[1]:
            best = (x, cost)
    if best is None:
        raise ValueError("no start model puts the markers on the floor in front")
    x, cost = best
    cam = _model_from(x[:7], base)
    cam.rms_px = float(math.sqrt(cost / len(sightings)))
    where = {k: x[7 + 2 * i: 9 + 2 * i].copy() for i, k in enumerate(keys)}
    return cam, cam.rms_px, where


def triangulate(kin, cam, views):
    """Robot-frame point nearest to the rays of pixels seen from several arm poses [(q, (u, v))].
    Returns (point, spread): spread is the RMS distance from the point to the rays (m)."""
    if len(views) < 2:
        raise ValueError("need two views or more, got %d" % len(views))
    a = np.zeros((3, 3))
    b = np.zeros(3)
    rays = []
    for q, uv in views:
        T = cam.camera_in_arm(kin, q)
        o = kin.mount.to_robot(T[:3, 3])
        d = kin.mount.to_robot(T[:3, 3] + T[:3, :3] @ cam.ray_cam(uv[0], uv[1])) - o
        d = d / np.linalg.norm(d)
        m = np.eye(3) - np.outer(d, d)
        a += m
        b += m @ o
        rays.append((o, m))
    p = np.linalg.solve(a, b)
    spread = math.sqrt(sum(float(np.sum((m @ (p - o)) ** 2)) for o, m in rays) / len(rays))
    return p, spread


def _lm(fun, x, iterations, steps=None):
    lam = 1e-3
    r = fun(x)
    cost = float(r @ r)
    for _ in range(iterations):
        jac = np.zeros((len(r), len(x)))
        for i in range(len(x)):
            h = steps[i] if steps is not None else (1e-6 if i < 6 else 1e-3)
            dx = x.copy()
            dx[i] += h
            jac[:, i] = (fun(dx) - r) / h
        a = jac.T @ jac
        g = jac.T @ r
        improved = False
        for _ in range(8):
            step = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-9), -g)
            xn = x + step
            rn = fun(xn)
            cn = float(rn @ rn)
            if cn < cost:
                x, r, cost, lam = xn, rn, cn, max(lam / 5, 1e-9)
                improved = True
                break
            lam *= 10
        if not improved or float(np.abs(step).max()) < 1e-9:
            break
    return x, cost


# -------------------------------------------------------------------------------- detection
class Detection:
    def __init__(self, u, v, area=0.0, angle=0.0, how=""):
        self.u, self.v, self.area, self.angle, self.how = float(u), float(v), float(area), float(angle), how

    def __repr__(self):
        return "Detection(u=%.1f, v=%.1f, area=%.0f, how=%s)" % (self.u, self.v, self.area, self.how)


def _cv2():
    import cv2  # noqa: PLC0415 (only the detectors need OpenCV)
    return cv2


def _components(mask, min_area):
    """[(area, cx, cy, angle, compactness, (x, y, w, h))] of the mask's connected parts, largest first."""
    cv2 = _cv2()
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        ys, xs = np.nonzero(labels[y:y + h, x:x + w] == i)
        cov = np.cov(np.vstack([xs, ys])) if len(xs) > 2 else np.eye(2)
        angle = 0.5 * math.atan2(2 * cov[0, 1], cov[0, 0] - cov[1, 1])
        out.append((float(area), float(cents[i][0]), float(cents[i][1]), angle, area / float(w * h), (x, y, w, h)))
    return sorted(out, key=lambda c: -c[0])


class ColourBlobDetector:
    """Largest blob in a hue range (OpenCV HSV, hue 0..180). Default: the pink/red gift bag.
    The jaw at the bottom of the image is cut off (roi_bottom)."""

    def __init__(self, hues=((0, 12), (150, 180)), s_min=70, v_min=50, roi_bottom=JAW_ROI, min_area=150,
                 point="centroid"):
        self.hues, self.s_min, self.v_min = hues, s_min, v_min
        self.roi_bottom, self.min_area, self.point = roi_bottom, min_area, point

    def mask(self, img):
        cv2 = _cv2()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        m = np.zeros(h.shape, bool)
        for lo, hi in self.hues:
            m |= (h >= lo) & (h <= hi)
        m &= (s >= self.s_min) & (v >= self.v_min)
        m[int(img.shape[0] * self.roi_bottom):] = False
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return m

    def detect(self, img):
        if img is None:
            return None
        comps = _components(self.mask(img), self.min_area)
        if not comps:
            return None
        area, u, v, angle, _c, (x, y, w, h) = comps[0]
        if self.point == "top":       # seen from the side: the neck is the top of the bag
            v = y + 0.15 * h
        return Detection(u, v, area, angle, "colour")


class CompactBlobDetector:
    """Fallback: the largest compact blob that differs from the floor colour, scored by closeness to
    the image centre (the arm is already roughly above the bag)."""

    def __init__(self, diff=45.0, roi_bottom=JAW_ROI, min_area=300, min_compact=0.3):
        self.diff, self.roi_bottom, self.min_area, self.min_compact = diff, roi_bottom, min_area, min_compact

    def detect(self, img):
        if img is None:
            return None
        cv2 = _cv2()
        cut = int(img.shape[0] * self.roi_bottom)
        roi = cv2.GaussianBlur(img[:cut], (5, 5), 0).astype(float)
        floor = np.median(roi.reshape(-1, 3), axis=0)
        m = np.linalg.norm(roi - floor, axis=2) > self.diff
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        best = None
        cx, cy = img.shape[1] / 2, cut / 2
        for area, u, v, angle, compact, _box in _components(m, self.min_area):
            if compact < self.min_compact:
                continue
            score = area / (1.0 + math.hypot(u - cx, v - cy) / 50.0)
            if best is None or score > best[0]:
                best = (score, Detection(u, v, area, angle, "compact"))
        return best[1] if best else None


class BagHandleDetector:
    """Bag neck/handles: the pink/red colour blob, else the compact-blob fallback."""

    def __init__(self, primary=None, fallback=None):
        self.primary = primary or ColourBlobDetector()
        self.fallback = fallback or CompactBlobDetector()

    def detect(self, img):
        return self.primary.detect(img) or self.fallback.detect(img)


class GroutCrossDetector:
    """Crossing of two floor-tile grout lines (dark thin lines, 0.60 m tiles) nearest the image centre:
    a free calibration marker when its floor position is measured with a tape."""

    def __init__(self, roi_bottom=JAW_ROI, min_len=120, dark=15):   # 15: the robot room's grout (25 found none)
        self.roi_bottom, self.min_len, self.dark = roi_bottom, min_len, dark

    def lines(self, img):
        cv2 = _cv2()
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cut = int(g.shape[0] * self.roi_bottom)
        g = g[:cut]
        blur = cv2.GaussianBlur(g, (0, 0), 3)
        m = ((blur.astype(int) - g.astype(int)) > self.dark // 2).astype(np.uint8) * 255   # thin dark lines
        segs = cv2.HoughLinesP(m, 1, np.pi / 180, 60, minLineLength=self.min_len, maxLineGap=12)
        return [] if segs is None else [tuple(map(float, s)) for s in np.asarray(segs).reshape(-1, 4)]

    NEAR_SEGMENT = 30.0   # px: a crossing must lie this close to both of its lines (not where two far lines meet)

    def crossings(self, img):
        """[(u, v)] where two grout lines at more than 50 deg meet, close to both segments."""
        segs = self.lines(img)
        angs = [math.atan2(y2 - y1, x2 - x1) % math.pi for x1, y1, x2, y2 in segs]
        out = []
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                d = abs(angs[i] - angs[j])
                if min(d, math.pi - d) < math.radians(50):
                    continue
                p = _intersect(segs[i], segs[j])
                if p is None or not (0 <= p[0] < img.shape[1] and 0 <= p[1] < img.shape[0] * self.roi_bottom):
                    continue
                if max(_to_segment(p, segs[i]), _to_segment(p, segs[j])) > self.NEAR_SEGMENT:
                    continue
                out.append((float(p[0]), float(p[1]), angs[i]))
        return out

    def detect(self, img, near=None):
        """The crossing nearest `near` (u, v), or nearest the image centre."""
        if img is None:
            return None
        cx, cy = near if near is not None else (img.shape[1] / 2, img.shape[0] * self.roi_bottom / 2)
        best = None
        for u, v, ang in self.crossings(img):
            dist = math.hypot(u - cx, v - cy)
            if best is None or dist < best[0]:
                best = (dist, Detection(u, v, 0.0, ang, "grout"))
        return best[1] if best else None


def _to_segment(p, seg):
    a, b = np.array(seg[:2]), np.array(seg[2:])
    t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-9), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


def _intersect(a, b):
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-9:
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return np.array([x1 + t * (x2 - x1), y1 + t * (y2 - y1)])


def load_detector(spec):
    """'module:Class' -> an instance (a learned detector with .detect(bgr_image) -> Detection|None)."""
    import importlib  # noqa: PLC0415
    mod, _, cls = spec.partition(":")
    return getattr(importlib.import_module(mod), cls)()


def fetch_snapshot(url, timeout=3.0):
    """One BGR frame from webcam_view.py's /snap.jpg (None when it fails)."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            data = np.frombuffer(r.read(), np.uint8)
        return _cv2().imdecode(data, 1)
    except Exception:  # noqa: BLE001 (no frame: the caller reports NOT_FOUND)
        return None


def wait_fresh_snapshot(url, settle=0.6, timeout=3.0):
    """Let the arm settle and the MJPEG stream catch up, then grab a frame."""
    time.sleep(settle)
    fetch_snapshot(url, timeout)          # drop one possibly stale frame
    return fetch_snapshot(url, timeout)
