"""The fixed front camera (front centre of the robot, under the top deck, looking straight ahead): its model, the
calibration fit, and the bag's top height measured from a distance with the room lit (the RGB path).

Why: at the pick distance nothing sees the bag (base/docs/hardware-assessment-2026-11-13.md, last section), so the
bag is measured while the robot still stands 0.7 to 1.5 m away (base/pi/bag.py's far stage). The Pi measures with
the Xtion depth first (base/pi/bag_depth.py); when that has no depth (a dark room is fine for depth, a dark bag is
not) it asks this module, which runs on the Jetson next to the arm team's webcam_view.py (frames from
http://127.0.0.1:8082/snap.jpg, 640x480; /dev/video0 is never opened). See base/docs/bag-sensing.md.

No ROS. bag_arm.py serves measurements on TCP PORT (one JSON line each way, see serve); tools/front_cam_cal.py
calibrates it (the fit is here).

Frames: robot = the base frame (x ahead, y left, z up from the floor, origin at the turning centre). Camera:
OpenCV (z along the optical axis, x right, y down in the image). With yaw = pitch = roll = 0 the camera looks along
robot +x, image right = robot -y. R = Rz(yaw) Ry(pitch) Rx(roll) R0: yaw + turns it left, pitch + tilts it down,
roll + puts its right side down (a level line then rises to the right in the image). mirror: the image is flipped
left-right (image right = robot left, as the Xtion's).
"""
import json
import math
import os
import socketserver
import threading
import urllib.request

import numpy as np

import ports

DEFAULT_FILE = os.path.expanduser("~/hmc-follow/front_camera.json")
SNAP_URL = "http://127.0.0.1:%d/snap.jpg" % ports.WEBCAM
PORT = ports.LOOK  # TCP: bag measurements for the Pi (base/pi/bag.py Look)
WIDTH, HEIGHT = 640, 480  # webcam_view opens the AX1080P at this size, MJPG. fx, cx, cy are for these pixels.
F_GUESS = 480.0  # px: the same AX1080P webcam as gripper_camera.F_GUESS (about 67 deg across); the fit sets it
X_GUESS = 0.20  # m ahead of the turning centre ("about 0.2 m", the lane brief); measure it with a tape
Z_GUESS = 0.25  # m above the floor: a guess, the fit sets it
LIDAR_Z = 0.245  # m, the LiDAR plane: the bag's points from the Pi are cut there
R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])  # camera axes -> robot, all angles zero
NAMES = ("fx", "x", "y", "z", "yaw", "pitch", "roll")
ANGLES = ("yaw", "pitch", "roll")


class NotCalibrated(RuntimeError):
    pass


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


class FrontCamera:
    """Pinhole (fx = fy, cx, cy) at (x, y, z) in the robot frame, turned by yaw, pitch, roll (rad).

    calibrated False (no file, or a file that says so): project and ray raise NotCalibrated, and the bag is not
    measured by this camera. The fit uses the same geometry without that check (_project, _ray)."""

    def __init__(self, fx=F_GUESS, x=X_GUESS, y=0.0, z=Z_GUESS, yaw=0.0, pitch=0.0, roll=0.0, cx=None, cy=None,
                 width=WIDTH, height=HEIGHT, mirror=False, calibrated=False, info=None):
        self.fx, self.x, self.y, self.z = float(fx), float(x), float(y), float(z)
        self.yaw, self.pitch, self.roll = float(yaw), float(pitch), float(roll)
        self.width, self.height = int(width), int(height)
        self.cx = (self.width - 1) / 2 if cx is None else float(cx)  # pixel centres at whole numbers (OpenCV)
        self.cy = (self.height - 1) / 2 if cy is None else float(cy)
        self.mirror, self.calibrated = bool(mirror), bool(calibrated)
        self.info = dict(info or {})

    def copy(self, **changes):
        d = {k: getattr(self, k) for k in NAMES + ("cx", "cy", "width", "height", "mirror", "calibrated")}
        d.update(changes)
        return FrontCamera(info=self.info, **d)

    def rotation(self):
        """Camera axes -> robot frame."""
        return _rz(self.yaw) @ _ry(self.pitch) @ _rx(self.roll) @ R0

    @property
    def position(self):
        return np.array([self.x, self.y, self.z])

    def _check(self):
        if not self.calibrated:
            raise NotCalibrated("the front camera is not calibrated (tools/front_cam_cal.py)")

    def _project(self, pts):
        """(N, 3) robot points -> (N, 2) pixels, nan for points not in front of the camera."""
        pc = (np.asarray(pts, float).reshape(-1, 3) - self.position) @ self.rotation()
        z = pc[:, 2]
        good = z > 1e-6
        zs = np.where(good, z, 1.0)
        u = self.fx * pc[:, 0] / zs
        u = self.cx - u if self.mirror else self.cx + u
        v = self.cy + self.fx * pc[:, 1] / zs
        return np.where(good[:, None], np.column_stack([u, v]), np.nan)

    def _ray(self, u, v):
        d = np.array([(u - self.cx) / self.fx * (-1.0 if self.mirror else 1.0), (v - self.cy) / self.fx, 1.0])
        d = self.rotation() @ d
        return d / np.linalg.norm(d)

    def project(self, p):
        """A robot-frame point -> (u, v), or None when it is not in front of the camera."""
        self._check()
        uv = self._project(p)[0]
        return None if not np.isfinite(uv).all() else (float(uv[0]), float(uv[1]))

    def project_many(self, pts):
        self._check()
        return self._project(pts)

    def ray(self, u, v):
        """Unit robot-frame direction of pixel (u, v), from self.position."""
        self._check()
        return self._ray(u, v)

    # ------------------------------------------------------------------ file
    def to_dict(self):
        return {"calibrated": self.calibrated, "fx": self.fx, "cx": self.cx, "cy": self.cy, "width": self.width,
                "height": self.height, "x": self.x, "y": self.y, "z": self.z, "yaw_deg": math.degrees(self.yaw),
                "pitch_deg": math.degrees(self.pitch), "roll_deg": math.degrees(self.roll), "mirror": self.mirror,
                "info": self.info}

    @classmethod
    def from_dict(cls, d):
        return cls(fx=d["fx"], x=d["x"], y=d["y"], z=d["z"], yaw=math.radians(d.get("yaw_deg", 0.0)),
                   pitch=math.radians(d.get("pitch_deg", 0.0)), roll=math.radians(d.get("roll_deg", 0.0)),
                   cx=d.get("cx"), cy=d.get("cy"), width=d.get("width", WIDTH), height=d.get("height", HEIGHT),
                   mirror=d.get("mirror", False), calibrated=d.get("calibrated", False), info=d.get("info"))

    def save(self, path=DEFAULT_FILE):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path=DEFAULT_FILE):
        """The calibration, or an uncalibrated camera when the file is missing or broken."""
        try:
            with open(path) as f:
                return cls.from_dict(json.load(f))
        except (OSError, ValueError, KeyError, TypeError):
            return cls(calibrated=False, info={"why": "no calibration at %s" % path})


def for_image(cam, image):
    """The camera for this frame's pixel size.

    The calibration (fx, cx, cy, width, height) is for WIDTH x HEIGHT. The same size is returned
    unchanged. A frame that is a uniform scale of that size (the same aspect) gets fx, cx and cy
    scaled about pixel centres, so a ray through a feature is the ray it had in the calibration
    image. A different aspect (1280x720 against 640x480) is refused: scaling one axis would bend
    the rays. Returns None in that case.
    """
    shape = np.asarray(image).shape
    height, width = int(shape[0]), int(shape[1])
    if width == cam.width and height == cam.height:
        return cam
    if cam.width <= 0 or cam.height <= 0:
        return None
    sx, sy = width / float(cam.width), height / float(cam.height)
    if abs(sx - sy) > 0.02:
        return None
    scale = (sx + sy) / 2.0
    return cam.copy(fx=cam.fx * scale, cx=(cam.cx + 0.5) * scale - 0.5, cy=(cam.cy + 0.5) * scale - 0.5,
                    width=width, height=height)


# ====================================================================== calibration fit
# Tolerances the fit must reach (the lane brief): 1 % of fx, 0.3 deg, 1 cm. A parameter whose standard error
# (at the measured pixel scatter, at least NOISE_FLOOR_PX) is over its tolerance is reported as not determined.
NEED = {"fx": 0.01, "x": 0.01, "y": 0.01, "z": 0.01, "yaw": math.radians(0.3), "pitch": math.radians(0.3),
        "roll": math.radians(0.3)}
NOISE_FLOOR_PX = 0.5
FIT_FREE = ("fx", "yaw", "pitch", "roll", "z")
EDGE_NEED = 0.02  # m, an edge position is only a helper unknown; reported with its error


def _to_robot(q_world, pose):
    """World (x, y) -> the robot frame of `pose` (x, y, yaw of the robot in the world)."""
    x, y, th = pose
    dx, dy = q_world[0] - x, q_world[1] - y
    c, s = math.cos(th), math.sin(th)
    return np.array([c * dx + s * dy, -s * dx + c * dy])


def _image_line(cam, p1, p2):
    """Image line (a, b, c), a u + b v + c = 0 with a^2 + b^2 = 1, through the projections of two robot points."""
    uv = cam._project(np.array([p1, p2]))
    if not np.isfinite(uv).all():
        return None
    (u1, v1), (u2, v2) = uv
    a, b = v2 - v1, u1 - u2
    n = math.hypot(a, b)
    if n < 1e-9:
        return None
    return a / n, b / n, -(a * u1 + b * v1) / n


def _line_dist(line, pts):
    if line is None:
        return np.full(len(pts), 1e3)
    a, b, c = line
    return a * pts[:, 0] + b * pts[:, 1] + c


def _floor_line_points(cam, wall):
    """Two floor points on the wall line (robot frame: n . q = d, n at angle `normal`), in front of the camera."""
    normal, d = wall
    n = np.array([math.cos(normal), math.sin(normal)])
    t = np.array([-n[1], n[0]])
    foot = cam.position[:2] + (d - n @ cam.position[:2]) * n
    return np.append(foot - 0.5 * t, 0.0), np.append(foot + 0.5 * t, 0.0)


def _residuals(cam, frames, edges):
    out = []
    for fr in frames:
        floor = fr.get("floor")
        if floor is not None and len(floor) and fr.get("wall") is not None:
            out.append(_line_dist(_image_line(cam, *_floor_line_points(cam, fr["wall"])), floor))
        for name, pts in (fr.get("edges") or {}).items():
            q = _to_robot(edges[name], fr["pose"])
            out.append(_line_dist(_image_line(cam, (q[0], q[1], 0.0), (q[0], q[1], 1.0)), pts))
    return np.concatenate(out) if out else np.zeros(0)


LOG = ("fx", "z")  # fitted as logarithms: fx > 0 and z > 0, so no mirrored or upside-down twin solves it


def _unpack(base, free, names_e, vec):
    ch = {k: math.exp(vec[i]) if k in LOG else vec[i] for i, k in enumerate(free)}
    cam = base.copy(**ch)
    edges = {n: vec[len(free) + 2 * i: len(free) + 2 * i + 2] for i, n in enumerate(names_e)}
    return cam, edges


def _edge_guess(cam, frames, name, wall_dist=2.0):
    """A start for an edge's world (x, y): its first sighting's ray, out to the wall when one is given."""
    for fr in frames:
        pts = (fr.get("edges") or {}).get(name)
        if pts is None or not len(pts):
            continue
        u, v = pts[0]
        d = cam._ray(u, cam.cy if not np.isfinite(v) else v)
        h = np.array([d[0], d[1]]) / (math.hypot(d[0], d[1]) or 1e-9)
        dist = wall_dist
        if fr.get("wall") is not None:
            normal, wd = fr["wall"]
            n = np.array([math.cos(normal), math.sin(normal)])
            k = h @ n
            if k > 0.2:
                dist = (wd - n @ cam.position[:2]) / k
        q = cam.position[:2] + dist * h
        x, y, th = fr["pose"]
        c, s = math.cos(th), math.sin(th)
        return np.array([x + c * q[0] - s * q[1], y + s * q[0] + c * q[1]])
    raise ValueError("edge %s is never seen" % name)


def fit(frames, guess, free=FIT_FREE, iterations=200):
    """Least squares camera fit from the wall-floor line and vertical edges seen at LiDAR-measured poses.

    frames: [{"pose": (x, y, yaw) of the robot in the world (the first frame's robot frame, from the LiDAR),
              "wall": (normal, dist) the wall line in THIS frame's robot frame (n . q = dist on the floor), or None,
              "floor": (N, 2) pixels on the wall-floor line, "edges": {name: (M, 2) pixels on a vertical edge, v nan
              when only the column is known}}]
    guess: a FrontCamera: start values of the free parameters, the values of the fixed ones (cx, cy stay).
    Unknown too: each edge's world (x, y). Returns a dict: camera (calibrated, with info), rms_px, residuals count,
    std (standard errors at max(rms, NOISE_FLOOR_PX)), determined / undetermined parameter names, edges, and why.
    Parameters the data cannot pin to NEED are listed as undetermined, never invented: the caller must fix them
    by measurement (or add frames) before it writes the calibration."""
    frames = [dict(fr, floor=None if fr.get("floor") is None else np.asarray(fr["floor"], float).reshape(-1, 2),
                   edges={k: np.asarray(v, float).reshape(-1, 2) for k, v in (fr.get("edges") or {}).items()})
              for fr in frames]
    for fr in frames:  # a column only: on the middle row
        for pts in fr["edges"].values():
            pts[~np.isfinite(pts[:, 1]), 1] = guess.cy
    names_e = sorted({n for fr in frames for n in fr["edges"]})
    free = tuple(free)
    vec = np.array([math.log(getattr(guess, k)) if k in LOG else getattr(guess, k) for k in free]
                   + [c for n in names_e for c in _edge_guess(guess, frames, n)])
    steps = np.array([1e-5] * len(free) + [1e-5] * (2 * len(names_e)))

    def fun(p):
        cam, edges = _unpack(guess, free, names_e, p)
        return _residuals(cam, frames, edges)

    r = fun(vec)
    m, n = len(r), len(vec)
    if m < n:
        raise ValueError("%d residuals for %d unknowns: more frames are needed" % (m, n))
    cost, lam = float(r @ r), 1e-3
    jac = None
    for _ in range(iterations):
        jac = np.column_stack([(fun(vec + np.eye(n)[i] * steps[i]) - fun(vec - np.eye(n)[i] * steps[i]))
                               / (2 * steps[i]) for i in range(n)])
        a, g = jac.T @ jac, jac.T @ r
        moved = False
        for _try in range(12):
            dp = -np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), g)
            r2 = fun(vec + dp)
            c2 = float(r2 @ r2)
            if c2 < cost:
                vec, r, lam, moved = vec + dp, r2, max(lam / 10, 1e-12), True
                done = cost - c2 < 1e-12 * max(cost, 1e-12) or np.all(np.abs(dp) < steps * 1e-3)
                cost = c2
                break
            lam *= 10
        if not moved or done:
            break
    jac = np.column_stack([(fun(vec + np.eye(n)[i] * steps[i]) - fun(vec - np.eye(n)[i] * steps[i]))
                           / (2 * steps[i]) for i in range(n)])
    rms = math.sqrt(cost / m)
    sigma = max(rms, NOISE_FLOOR_PX)
    w, v = np.linalg.eigh(jac.T @ jac)
    var = sigma ** 2 * (v ** 2 @ (1.0 / np.maximum(w, 1e-300)))
    std = np.sqrt(var)
    cam, edges = _unpack(guess, free, names_e, vec)
    report_std, determined, undetermined = {}, [], []
    for i, k in enumerate(free):
        value = math.exp(vec[i]) if k in LOG else vec[i]
        report_std[k] = float(std[i] * value if k in LOG else std[i])  # a log's error is relative: back in px, m
        need = NEED[k] * value if k == "fx" else NEED[k]  # NEED["fx"] is relative
        (determined if report_std[k] <= need and math.isfinite(report_std[k]) else undetermined).append(k)
    edge_out = {nm: {"x": float(edges[nm][0]), "y": float(edges[nm][1]),
                     "std": float(max(std[len(free) + 2 * i], std[len(free) + 2 * i + 1]))}
                for i, nm in enumerate(names_e)}
    cam.calibrated = not undetermined
    cam.info = {"rms_px": round(rms, 3), "residuals": m, "free": list(free), "undetermined": undetermined}
    return {"camera": cam, "rms_px": rms, "residuals": m, "std": report_std, "determined": determined,
            "undetermined": undetermined, "edges": edge_out,
            "why": None if not undetermined else "not determined by these frames: %s" % ", ".join(undetermined)}


def fit_both_mirrors(frames, guess, free=FIT_FREE):
    """fit with the image as it is and flipped; the lower rms wins. Both rms are in the result (mirror_rms)."""
    out = {m: fit(frames, guess.copy(mirror=m), free) for m in (False, True)}
    best = min(out.values(), key=lambda f: f["rms_px"])
    best["mirror_rms"] = {str(m).lower(): round(f["rms_px"], 3) for m, f in out.items()}
    return best


# ====================================================================== detection in the image
STEP_MIN = 40.0  # grey levels: the floor below the black baseboard is this much brighter than it, at least


def _gray(img):
    img = np.asarray(img, float)
    return img.mean(axis=2) if img.ndim == 3 else img


def fit_line_ransac(pts, tol=2.0, iterations=300, seed=0):
    """Robust image line through pts: ((a, b, c) normalised, inlier points) or (None, empty)."""
    pts = np.asarray(pts, float).reshape(-1, 2)
    if len(pts) < 2:
        return None, pts[:0]
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iterations):
        i, j = rng.choice(len(pts), 2, replace=False)
        (u1, v1), (u2, v2) = pts[i], pts[j]
        a, b = v2 - v1, u1 - u2
        n = math.hypot(a, b)
        if n < 1e-9:
            continue
        line = (a / n, b / n, -(a * u1 + b * v1) / n)
        k = int((np.abs(_line_dist(line, pts)) < tol).sum())
        if best is None or k > best[0]:
            best = (k, line)
    if best is None:
        return None, pts[:0]
    inl = pts[np.abs(_line_dist(best[1], pts)) < tol]
    for _ in range(3):  # total least squares on the inliers
        m = inl.mean(axis=0)
        w, v = np.linalg.eigh(np.cov((inl - m).T))
        a, b = v[:, 0]
        line = (a, b, -(a * m[0] + b * m[1]))
        inl = pts[np.abs(_line_dist(line, pts)) < tol]
        if len(inl) < 2:
            return None, pts[:0]
    return line, inl


def detect_floor_line(img, step=STEP_MIN, rows=None, every=2):
    """The wall-floor line: the bottom edge of the black baseboard over the pale floor. In each column (every
    `every`) the row where the image gets brightest going down (rows above dark, rows below bright) by more
    than `step`, to a tenth of a pixel; then a robust line through those points.
    Returns {"line", "points" (inliers), "count", "rms"} or None."""
    g = _gray(img)
    g = np.vstack([g[:1], (g[:-2] + 2 * g[1:-1] + g[2:]) / 4, g[-1:]])  # [1, 2, 1] down the columns
    h, w = g.shape
    d = g[1:, :] - g[:-1, :]  # d[i] = g[i + 1] - g[i]: the step between rows i and i + 1
    if rows is not None:
        lo, hi = max(0, rows[0]), max(0, min(h - 1, rows[1]))
        d = d.copy()
        d[:lo] = -1e9
        d[hi:] = -1e9
    pts = []
    for u in range(0, w, every):
        col = d[:, u]
        i = int(np.argmax(col))
        if col[i] < step:
            continue
        off = 0.0
        if 0 < i < len(col) - 1:
            den = col[i - 1] - 2 * col[i] + col[i + 1]
            off = 0.5 * (col[i - 1] - col[i + 1]) / den if abs(den) > 1e-9 else 0.0
        pts.append((u, i + 0.5 + off))  # the edge lies between rows i and i + 1
    line, inl = fit_line_ransac(pts)
    if line is None or len(inl) < 20:
        return None
    return {"line": line, "points": inl, "count": len(inl), "rms": float(np.sqrt(np.mean(_line_dist(line, inl) ** 2)))}


def detect_vertical_edge(img, u_guess, window=30, rows=(0, None), step=STEP_MIN):
    """A vertical edge near column u_guess: in each row the column of the strongest left-right step within
    +-window, then a robust line. Returns (M, 2) inlier pixels or an empty array."""
    g = _gray(img)
    h, w = g.shape
    r0, r1 = rows[0], h if rows[1] is None else rows[1]
    u0, u1 = max(1, int(u_guess - window)), min(w - 2, int(u_guess + window))
    pts = []
    for v in range(r0, r1, 2):
        diff = np.abs(g[v, u0 + 1:u1 + 2] - g[v, u0 - 1:u1])
        i = int(np.argmax(diff))
        if diff[i] >= step:
            pts.append((u0 + i, v))
    line, inl = fit_line_ransac(pts, tol=1.5)
    return inl if line is not None and len(inl) >= 10 else np.zeros((0, 2))


# ====================================================================== the bag (RGB path)
TOP_MAX = 0.70  # m: the region looked in reaches this high
SIDE_M = 0.15  # m beside the bag's LiDAR points: the strips there are the background of each row
BOTTOM_M = 0.10  # m in front of the bag's near side: the region reaches this far down the floor
COLOUR_T = 40.0  # BGR distance from the row's background that makes a bag pixel
EDGE_T = 60.0  # grey-level gradient that makes a bag pixel (fallback for a clear bag with little colour)
BAG_HALF_DEPTH = 0.05  # m from the side the LiDAR sees to the bag centre (base/pi/bag.py, an assumption)
AGREE = 0.03  # m: the range way and the camera height way agree this well for a full confidence (zero at 2x)
TOP_RANGE = (0.10, 0.60)
OK_CONF = 0.5


def _cv2():
    import cv2  # noqa: PLC0415 (only the segmenter needs OpenCV)
    return cv2


class ColourEdgeSegmenter:
    """Bag pixels inside a region: those whose colour differs from the background of their own row (the strips
    left and right of the bag see the floor, the baseboard and the wall at that row), and, when that finds
    little, strong edges (a clear plastic bag shows little colour but its creases and shine are edges); the part
    connected to the LiDAR seeds.

    Pluggable: anything with segment(image, roi, inner, seeds) -> bool mask over the roi can replace it (a learned
    model later). roi (u0, v0, u1, v1), inner (a, b) the bag's columns within the roi, seeds (K, 2) roi pixels
    known to be on the bag (its LiDAR points)."""

    def __init__(self, colour=COLOUR_T, edge=EDGE_T):
        self.colour, self.edge = colour, edge

    def segment(self, image, roi, inner, seeds):
        cv2 = _cv2()
        u0, v0, u1, v1 = roi
        sub = np.asarray(image, float)[v0:v1, u0:u1]
        a, b = inner
        side = np.zeros(sub.shape[1], bool)
        side[:max(0, a - 4)] = True
        side[min(sub.shape[1], b + 4):] = True
        if side.sum() < 4:
            return np.zeros(sub.shape[:2], bool)
        strips = sub[:, side]
        bg = np.median(strips, axis=1)
        spread = np.median(np.linalg.norm(strips - bg[:, None], axis=2), axis=1)
        diff = np.linalg.norm(sub - bg[:, None], axis=2)
        mask = diff > np.maximum(self.colour, 3 * spread)[:, None]
        if self._seeded(mask, seeds) < 0.5:  # little colour: try edges too
            g = _gray(sub)
            gy, gx = np.gradient(g)
            mag = np.hypot(gx, gy)
            mask |= mag > np.maximum(self.edge, 3 * np.median(mag[:, side], axis=1))[:, None]
        mask[:, side] = False
        m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        n, labels = cv2.connectedComponents(m, connectivity=8)
        keep = {int(labels[int(round(v)), int(round(u))]) for u, v in seeds
                if 0 <= int(round(v)) < m.shape[0] and 0 <= int(round(u)) < m.shape[1]} - {0}
        return np.isin(labels, list(keep)) if keep else np.zeros(m.shape, bool)

    @staticmethod
    def _seeded(mask, seeds):
        hit = [mask[int(round(v)), int(round(u))] for u, v in seeds
               if 0 <= int(round(v)) < mask.shape[0] and 0 <= int(round(u)) < mask.shape[1]]
        return float(np.mean(hit)) if hit else 0.0


def _elev_tan(d):
    return d[2] / math.hypot(d[0], d[1])


def _range_at(cam, d, m, e):
    """Horizontal distance from the camera along direction d to the bag's side (the line m + t e), or None."""
    h = np.array([d[0], d[1]]) / math.hypot(d[0], d[1])
    a = np.column_stack([h, -e])
    if abs(np.linalg.det(a)) < 0.2:
        return None
    s, _t = np.linalg.solve(a, m - cam.position[:2])
    return float(s) if s > 0 else None


def measure_bag(cam, image, points, segmenter=None):
    """The bag's top height and handle centre from one front camera frame and its LiDAR points.

    points: (N, 2) robot frame, the bag's LiDAR points (cut at LIDAR_Z) from the same moment (the robot stands).
    The top: height = range x (tan(top elevation) - tan(bottom elevation)), the top and the bottom (where the bag
    meets the floor) seen in the same image column, the range from the camera to the bag's side in that column.
    That is the height between the two, so the camera height drops out (base/docs/bag-sensing.md); the camera
    height way (z + range x tan(top elevation)) is the check. Without a bottom it is used alone, at lower
    confidence. Returns {"ok", "top", "top_by" ("range" | "height"), "top_height_way", "handle" [x, y],
    "conf", "range", "box", "why"}; ok only with conf >= OK_CONF and the top in TOP_RANGE."""
    out = {"ok": False, "top": None, "top_by": None, "top_height_way": None, "handle": None, "conf": 0.0,
           "range": None, "box": None, "why": None}
    if not cam.calibrated:
        out["why"] = "the front camera is not calibrated"
        return out
    if image is None:
        out["why"] = "no front camera frame"
        return out
    scaled = for_image(cam, image)
    if scaled is None:
        out["why"] = "front frame is not %dx%d and not a uniform scale of it" % (cam.width, cam.height)
        return out
    cam = scaled
    pts = np.asarray(points, float).reshape(-1, 2)
    if len(pts) < 3:
        out["why"] = "fewer than 3 LiDAR points on the bag"
        return out
    segmenter = segmenter or ColourEdgeSegmenter()
    h_img, w_img = np.asarray(image).shape[:2]
    m = pts.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((pts - m).T)) if len(pts) > 2 else (None, np.eye(2))
    e = v[:, 1]
    rng = np.hypot(pts[:, 0] - cam.x, pts[:, 1] - cam.y)
    r_face = float(np.median(rng))
    out["range"] = round(r_face, 3)
    view = (m - cam.position[:2]) / (np.linalg.norm(m - cam.position[:2]) or 1e-9)
    across = np.array([-view[1], view[0]])
    corners = []
    for q in pts:
        for s in (-SIDE_M, SIDE_M):
            for z in (0.0, TOP_MAX):
                corners.append((*(q + s * across), z))
        corners.append((*(q - BOTTOM_M * view), 0.0))
    uv = cam._project(np.array(corners))
    uv = uv[np.isfinite(uv).all(axis=1)]
    if not len(uv):
        out["why"] = "the bag is behind the front camera"
        return out
    u0, v0 = int(max(0, np.floor(uv[:, 0].min()))), int(max(0, np.floor(uv[:, 1].min())))
    u1, v1 = int(min(w_img, np.ceil(uv[:, 0].max()) + 1)), int(min(h_img, np.ceil(uv[:, 1].max()) + 1))
    if u1 - u0 < 10 or v1 - v0 < 10:
        out["why"] = "the bag is out of the front camera's view"
        return out
    out["box"] = [u0, v0, u1, v1]
    inner_uv = cam._project(np.array([(*q, z) for q in pts for z in (0.0, LIDAR_Z, TOP_MAX)]))
    a, b = int(np.nanmin(inner_uv[:, 0])) - u0, int(np.nanmax(inner_uv[:, 0])) + 1 - u0
    seeds = cam._project(np.array([(*q, LIDAR_Z) for q in pts])) - (u0, v0)
    mask = segmenter.segment(image, (u0, v0, u1, v1), (max(0, a), min(u1 - u0, b)), seeds)
    if not mask.any():
        out["why"] = "the bag is not told from the background in the image"
        return out
    rows = np.nonzero(mask.any(axis=1))[0]
    top_row = int(rows[0])
    if top_row == 0 and v0 == 0:
        out["why"] = "the bag reaches the top of the image"
        return out
    if top_row == 0:
        out["why"] = "the bag reaches the top of the region (taller than %.2f m?)" % TOP_MAX
        return out
    cols = np.nonzero(mask[top_row])[0]
    u_t = float(cols.mean())
    v_t = top_row - 0.5  # the top edge of the first bag row
    d_t = cam._ray(u_t + u0, v_t + v0)
    r_col = _range_at(cam, d_t, m, e) or r_face
    height_way = cam.z + r_col * _elev_tan(d_t)
    out["top_height_way"] = round(height_way, 3)
    # the bottom in the same column (a few columns round it)
    c0, c1 = max(0, int(round(u_t)) - 2), min(mask.shape[1], int(round(u_t)) + 3)
    col_rows = np.nonzero(mask[:, c0:c1].any(axis=1))[0]
    bottom = int(col_rows[-1]) if len(col_rows) else None
    if bottom is not None and bottom < mask.shape[0] - 1:
        d_b = cam._ray(u_t + u0, bottom + 0.5 + v0)
        top = r_col * (_elev_tan(d_t) - _elev_tan(d_b))
        out["top_by"] = "range"
        conf = 1.0 - min(1.0, max(0.0, (abs(top - height_way) - AGREE) / AGREE))
    else:
        top, out["top_by"], conf = height_way, "height", 0.5
    out["top"] = round(float(top), 3)
    # the handle: the middle of the top 3 cm of bag pixels, out at the bag centre's range
    band = max(1, int(round(cam.fx * 0.03 / r_col)))
    hc = np.nonzero(mask[top_row:top_row + band].any(axis=0))[0]
    d_h = cam._ray(float(hc.mean()) + u0, top_row + v0)
    hh = np.array([d_h[0], d_h[1]]) / math.hypot(d_h[0], d_h[1])
    handle = cam.position[:2] + (r_col + BAG_HALF_DEPTH) * hh
    out["handle"] = [round(float(handle[0]), 3), round(float(handle[1]), 3)]
    out["conf"] = round(conf, 2)
    out["ok"] = bool(conf >= OK_CONF and TOP_RANGE[0] <= top <= TOP_RANGE[1])
    if not out["ok"]:
        out["why"] = ("top %.3f m outside %.2f to %.2f m" % (top, *TOP_RANGE) if conf >= OK_CONF else
                      "the two ways disagree: %.3f against %.3f m" % (top, height_way))
    return out


# ====================================================================== the service
def fetch_snapshot(url=SNAP_URL, timeout=2.0):
    """One BGR frame from webcam_view.py's /snap.jpg, or None."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            data = np.frombuffer(r.read(), np.uint8)
        return _cv2().imdecode(data, 1)
    except Exception:  # noqa: BLE001 (no frame: the answer says so)
        return None


def answer(request, camera_file=DEFAULT_FILE, snap=fetch_snapshot):
    """One Pi request {"id", "points": [[x, y], ...]} -> the measurement dict with its id."""
    cam = FrontCamera.load(camera_file)
    try:
        pts = np.asarray(request.get("points") or [], float).reshape(-1, 2)
    except (ValueError, TypeError):
        return {"id": request.get("id"), "ok": False, "why": "bad points"}
    image = snap() if cam.calibrated else None
    out = measure_bag(cam, image, pts)
    out["id"] = request.get("id")
    return out


MAX_LINE = 65536


def serve(port=PORT, camera_file=DEFAULT_FILE, snap_url=SNAP_URL, host="0.0.0.0"):
    """TCP PORT: one JSON line in ({"id", "points"}), one JSON line out (answer), then the connection closes.
    Only reads: it never moves anything. Returns the server, running on a daemon thread."""

    class Handler(socketserver.StreamRequestHandler):
        timeout = 5.0

        def handle(self):
            try:
                line = self.rfile.readline(MAX_LINE)
                req = json.loads(line.decode("utf-8"))
                out = answer(req if isinstance(req, dict) else {}, camera_file,
                             lambda: fetch_snapshot(snap_url))
            except Exception as exc:  # noqa: BLE001 (a bad request gets an answer, never kills the server)
                out = {"ok": False, "why": "bad request: %s" % exc}
            self.wfile.write((json.dumps(out) + "\n").encode("utf-8"))

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    server = socketserver.ThreadingTCPServer((host, port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


if __name__ == "__main__":
    import argparse
    import time
    ap = argparse.ArgumentParser(description="the front camera's bag measurement service (bag_arm.py starts it too)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--camera-file", default=DEFAULT_FILE)
    ap.add_argument("--snap-url", default=SNAP_URL)
    a = ap.parse_args()
    serve(a.port, a.camera_file, a.snap_url)
    print("front_camera: bag measurements on TCP %d (%s)" % (a.port, "calibrated" if FrontCamera.load(
        a.camera_file).calibrated else "NOT calibrated: answers carry no top"))
    while True:
        time.sleep(1.0)
