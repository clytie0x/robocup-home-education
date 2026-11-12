"""Table-top object finding from a depth camera point cloud (numpy only, no ROS) - camera test for the arm.

Frames
  cam    the cloud's own frame (optical: x right, y down, z forward)
  table  z = up from the table top (table at z = 0), origin under the camera,
         x = the camera's viewing direction projected onto the table
  base   the arm base frame (table at z = table_z)

The table plane gives the camera's tilt and height by itself; the remaining turn (yaw) and shift on the
table come from two objects put at tape-measured spots (calibrate_2pt).
"""
import math

import numpy as np
from scipy import ndimage


def fit_plane(pts, iters=300, tol=0.008, seed=0, up=None, max_tilt=70.0, max_range=2.0):
    """RANSAC plane through the largest flat area. Returns (n, d, inliers) with n.p + d = 0 and the
    camera (origin) on the +n side (d > 0), so n points up out of the table.
    up: the camera's rough "up" direction (optical frame: (0, -1, 0)); planes tilted more than max_tilt
    degrees from it (walls, box sides) are skipped, and points further than max_range m are not used."""
    if up is not None:
        up = np.asarray(up, float) / np.linalg.norm(up)
        pts = pts[np.linalg.norm(pts, axis=1) < max_range]
    if len(pts) < 50:
        raise ValueError('not enough points for a table plane')
    min_cos = math.cos(math.radians(max_tilt))
    rng = np.random.default_rng(seed)
    best, best_n = None, -1
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if up is not None and (abs(n @ up) < min_cos or (n @ a) * (n @ up) > 0):
            continue                               # not a floor-like plane below the camera
        inl = np.abs(pts @ n - n @ a) < tol
        k = int(inl.sum())
        if k > best_n:
            best, best_n = inl, k
    if best is None:
        raise ValueError('no table-like plane below the camera')
    q = pts[best]
    centre = q.mean(axis=0)
    n = np.linalg.svd(q - centre)[2][2]          # least-squares refit on the inliers
    d = -n @ centre
    if d < 0:
        n, d = -n, -d
    inl = np.abs(pts @ n + d) < tol
    return n, float(d), inl


def table_frame(n, d):
    """Rotation R (cam -> table) and camera height h: p_table = R @ p_cam + [0, 0, h]."""
    z = n / np.linalg.norm(n)
    view = np.array([0.0, 0.0, 1.0])
    x = view - (view @ z) * z
    if np.linalg.norm(x) < 1e-6:                  # looking straight down: any x will do
        x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.vstack([x, y, z]), float(d)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def calibrate_2pt(a_tab, b_tab, a_base, b_base):
    """Turn + shift on the table that maps two object centres seen in the table frame onto where they
    were measured in the base frame. Returns (yaw, (tx, ty), length_error)."""
    a_tab, b_tab, a_base, b_base = (np.asarray(v, float)[:2] for v in (a_tab, b_tab, a_base, b_base))
    dt, db = b_tab - a_tab, b_base - a_base
    if np.linalg.norm(db) < 0.03:
        raise ValueError('the two spots must be at least 3 cm apart')
    yaw = math.atan2(db[1], db[0]) - math.atan2(dt[1], dt[0])
    r = rot_z(yaw)[:2, :2]
    t = ((a_base - r @ a_tab) + (b_base - r @ b_tab)) / 2
    return yaw, (float(t[0]), float(t[1])), float(abs(np.linalg.norm(db) - np.linalg.norm(dt)))


class CameraPose:
    """cam -> base: p_base = R @ p_cam + t."""

    def __init__(self, n, d, yaw=0.0, txy=(0.0, 0.0), table_z=0.0):
        self.n, self.d, self.yaw, self.txy, self.table_z = np.asarray(n, float), float(d), yaw, tuple(txy), table_z
        r_tab, h = table_frame(self.n, self.d)
        self.R = rot_z(yaw) @ r_tab
        self.t = np.array([txy[0], txy[1], table_z + h])

    def to_base(self, pts):
        return pts @ self.R.T + self.t

    def to_dict(self):
        return {'n': self.n.tolist(), 'd': self.d, 'yaw': self.yaw, 'txy': list(self.txy), 'table_z': self.table_z}

    @classmethod
    def from_dict(cls, v):
        return cls(v['n'], v['d'], v['yaw'], v['txy'], v['table_z'])

    def quaternion(self):
        """(x, y, z, w) of R."""
        m = self.R
        w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
        x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
        y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
        z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
        return x, y, z, w


LOW_VIEW = math.radians(35)   # camera looks at the object flatter than this: it sees mostly the near side


def find_objects(pts, table_z=0.0, keep=None, z_min=0.008, z_max=0.30, cell=0.01, min_points=30, pad=0.003,
                 camera=None, known_size=None):
    """Objects standing on the table. pts: N x 3 in a frame whose table top is z = table_z.
    keep: optional boolean mask of points allowed (e.g. not the arm). Returns a list of dicts
    {x, y, yaw, size: (depth, width, height), points} - depth is along yaw, the box stands on the table.
    camera: (x, y, z) of the camera in the same frame. A low camera only sees the near side of an object,
    so its depth away from the camera is taken as at least its width (round/square objects) and the
    centre is pushed back by the hidden part.
    known_size: (a, b, height) of the object when it is known (a box): only the seen side's position and
    turn come from the camera, the hidden part is filled in with the real size (needs camera)."""
    z = pts[:, 2] - table_z
    m = (z > z_min) & (z < z_max)
    if keep is not None:
        m &= keep
    p = pts[m]
    if len(p) < min_points:
        return []
    lo = p[:, :2].min(axis=0)
    ij = np.floor((p[:, :2] - lo) / cell).astype(int)
    grid = np.zeros(ij.max(axis=0) + 1, int)
    np.add.at(grid, (ij[:, 0], ij[:, 1]), 1)
    labels, count = ndimage.label(grid >= 2, structure=np.ones((3, 3)))
    lab = labels[ij[:, 0], ij[:, 1]]
    found = []
    for k in range(1, count + 1):
        q = p[lab == k]
        if len(q) < min_points:
            continue
        xy = q[:, :2]
        c = xy.mean(axis=0)
        w, v = np.linalg.eigh(np.cov((xy - c).T))
        u = v[:, 1]                                # major axis
        if u[0] < 0:
            u = -u
        vv = np.array([-u[1], u[0]])
        a, b = (xy - c) @ u, (xy - c) @ vv
        a0, a1 = np.percentile(a, [4, 96])
        b0, b1 = np.percentile(b, [4, 96])
        centre = c + u * (a0 + a1) / 2 + vv * (b0 + b1) / 2
        height = float(np.percentile(q[:, 2] - table_z, 98))
        obj = {'x': float(centre[0]), 'y': float(centre[1]), 'yaw': math.atan2(u[1], u[0]),
               'size': (float(a1 - a0 + pad), float(b1 - b0 + pad), height), 'points': len(q), 'completed': False}
        if camera is not None and known_size is not None:
            obj = fit_known_box(obj, xy, u, camera, known_size)
            if obj.get('ambiguous'):
                continue
        elif camera is not None:
            obj = complete_far_side(obj, xy, camera, table_z, pad)
        found.append(obj)
    return found


def fit_known_box(obj, xy, u, camera, size):
    """Box of known size: the seen face fixes one edge (position + turn), the rest lies behind it."""
    long_, short = max(size[0], size[1]), min(size[0], size[1])
    v = np.array([-u[1], u[0]])
    if (np.array([obj['x'], obj['y']]) - np.asarray(camera, float)[:2]) @ v < 0:
        v = -v                                     # v points away from the camera
    a, b = xy @ u, xy @ v
    a0, a1 = np.percentile(a, [4, 96])
    near = np.percentile(b, 4)
    seen = a1 - a0
    if 1.4 * short < seen < 0.6 * long_:
        return dict(obj, ambiguous=True, seen=(float(seen), obj['size'][2]))   # can't tell which side it sees
    along, depth = (long_, short) if seen > (long_ + short) / 2 else (short, long_)
    centre = u * (a0 + a1) / 2 + v * (near + depth / 2)
    return dict(obj, x=float(centre[0]), y=float(centre[1]), yaw=math.atan2(u[1], u[0]),
                size=(float(along), float(depth), float(size[2])), seen=(float(seen), obj['size'][2]),
                completed=True)


def complete_far_side(obj, xy, camera, table_z, pad):
    """Low view: measure along / across the line of sight; a shallow depth means the back is hidden."""
    cam = np.asarray(camera, float)
    d = np.array([obj['x'], obj['y']]) - cam[:2]
    dist = float(np.linalg.norm(d))
    if dist < 1e-6 or math.atan2(cam[2] - table_z - obj['size'][2], dist) > LOW_VIEW:
        return obj
    d /= dist
    perp = np.array([-d[1], d[0]])
    a, b = xy @ d, xy @ perp
    a0, a1 = np.percentile(a, [4, 96])
    b0, b1 = np.percentile(b, [4, 96])
    along, across = a1 - a0 + pad, b1 - b0 + pad
    if along >= across:
        return obj
    near = a0 * d + (b0 + b1) / 2 * perp               # the seen front, middle across
    centre = near + d * across / 2
    return dict(obj, x=float(centre[0]), y=float(centre[1]), yaw=math.atan2(d[1], d[0]),
                size=(float(across), float(across), obj['size'][2]), completed=True)


def fuse(history, near=0.03, min_frames=2):
    """Median of the same object over several frames: history = list of per-frame object lists.
    Objects are grouped by position across all frames; a group seen in at least min_frames frames
    (or in the only frame there is) is kept, so a frame that missed a far object does not lose it."""
    if not history:
        return []
    groups = []
    for k, frame in enumerate(history):
        for o in frame:
            g = min(groups, key=lambda g: math.hypot(g[0]['x'] - o['x'], g[0]['y'] - o['y']), default=None)
            if g is not None and math.hypot(g[0]['x'] - o['x'], g[0]['y'] - o['y']) < near and k not in g[1]:
                g[0].setdefault('_all', [g[0]]).append(o)
                g[1].add(k)
            else:
                groups.append([dict(o), {k}])
    need = min(min_frames, len(history))
    out = []
    for first, frames in groups:
        if len(frames) < need:
            continue
        same = first.get('_all', [first])
        yaw0 = same[-1]['yaw']
        yaws = [yaw0 + math.remainder(o['yaw'] - yaw0, math.pi) for o in same]   # boxes repeat every 180 deg
        out.append({'x': float(np.median([o['x'] for o in same])), 'y': float(np.median([o['y'] for o in same])),
                    'yaw': float(np.median(yaws)),
                    'size': tuple(float(np.median([o['size'][j] for o in same])) for j in range(3)),
                    'points': same[-1]['points'], 'frames': len(frames)})
    return out
