"""Calibrate the fixed front camera (front_camera.py, the webcam under the top deck, port 8082) with a bag on the floor.

No wall, no LiDAR, no driving (tools/front_cam_cal.py needs all three): the robot stands still and a bag is put at
measured places in front of it. Each frame gives the two bottom corners of the bag's front face (where it meets the
floor), at known robot-frame floor points; frames at several distances and sides pin down fx, the camera's place and
its yaw, pitch and roll (a floor homography, 8 numbers, for 7 unknowns).

Distances are measured on the floor from F, the point under the front edge of the lowest plate, on the robot's
centre line. The robot frame of front_camera.py starts at the turning centre: x = F distance + --f-from-lidar
(0.20 m, measured 2026-11-18) + --lidar-x (0.175 m, slam_bridge.LASER_X / the Pi's params.json lidar_x).

  python3 tools/front_cam_bag_cal.py detect FRAME.jpg [--out overlay.jpg]
  python3 tools/front_cam_bag_cal.py capture --session cal/bag.json --name b1 --front CM --left CM --width CM
        front: F to the bottom edge of the bag's front face, left: the face's middle left of the centre line (right
        negative), width: the face's width. The face square to the robot. Saves the frame next to the session.
        [--corner-z CM]: the red face starts this high above the floor (a pale band at the foot), kept for the session.
  python3 tools/front_cam_bag_cal.py fit cal/bag.json [--write ~/hmc-follow/front_camera.json]
        Both image mirrors are tried. Nothing is written while a parameter is not determined to front_camera.NEED.
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import front_camera as fc  # noqa: E402

F_FROM_LIDAR = 0.20   # m, the LiDAR centre to F (owner's tape, 2026-11-18)
LIDAR_X = 0.175       # m, the LiDAR ahead of the turning centre (slam_bridge.LASER_X)
FREE = ("fx", "x", "y", "z", "yaw", "pitch", "roll")
MIN_AREA = 3000       # px: the bag blob, at least
MAX_TILT_PX = 16      # px: a square face's two lower corners differ by 4-13 px in v here; 24 was a corner lost in glare


# ====================================================================== detection
def bag_mask(img):
    """The largest pink/red blob (the bag), as a bool mask, or None. The gripper camera's colour rule."""
    import gripper_camera as gc  # noqa: PLC0415
    cv2 = gc._cv2()
    det = gc.ColourBlobDetector(roi_bottom=1.0)
    m = det.mask(img)
    n, labels, stats, _c = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
    if n < 2:
        return None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[best, cv2.CC_STAT_AREA] < MIN_AREA:
        return None
    return labels == best


def bottom_corners(mask):
    """(left, right) bottom corners of the blob in the image: the pixels furthest down-left and down-right."""
    v, u = np.nonzero(mask)
    if len(u) == 0:
        return None
    left = int(np.argmax(v - u))
    right = int(np.argmax(v + u))
    return [float(u[left]), float(v[left])], [float(u[right]), float(v[right])]


def _is_bag(hsv_px):
    """Bag (red, pink, the pale pink band) rather than floor (cyan / grey-cyan, hue 30-140 in OpenCV's 0-180)."""
    h = hsv_px[..., 0].astype(int)
    return (h <= 25) | (h >= 150)


def foot_rows(img, mask, cols, run=5, reach=80):
    """Where the bag meets the floor in each column: from the red blob's lowest row, down through the pale band at
    the bag's foot, to the first row of `run` floor rows. None for a column that finds no floor within `reach`."""
    import cv2  # noqa: PLC0415
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = img.shape[0]
    out = []
    for u in cols:
        rows = np.nonzero(mask[:, u])[0]
        if len(rows) == 0:
            out.append(None)
            continue
        v = int(rows.max())
        found = None
        for vv in range(v, min(h - run, v + reach)):
            if not _is_bag(hsv[vv:vv + run, u]).any():
                found = vv
                break
        out.append(found)
    return out


def foot_corners(img, mask, inset=(6, 22)):
    """(left, right) [u, v] of the bag's foot (z = 0): the median foot row of the columns just inside each side of
    the red blob's lower part, at that side's column. None when a side finds no floor."""
    v, u = np.nonzero(mask)
    low = v >= np.percentile(v, 75)
    u_l, u_r = int(u[low].min()), int(u[low].max())
    res = []
    for side, edge in ((1, u_l), (-1, u_r)):
        cols = [edge + side * k for k in range(inset[0], inset[1])]
        rows = [r for r in foot_rows(img, mask, cols) if r is not None]
        if len(rows) < len(cols) // 2:
            return None
        res.append([float(edge), float(np.median(rows))])
    return res[0], res[1]


def detect(img, foot=False):
    """{"left": [u, v], "right": [u, v], "area"} of the bag's lower corners, or None. foot=False: the red face's
    lower corners (at the session's corner_z); foot=True: where the bag meets the floor (z = 0)."""
    m = bag_mask(img)
    if m is None:
        return None
    c = foot_corners(img, m) if foot else bottom_corners(m)
    if c is None or c[0][1] >= m.shape[0] - 2 or c[1][1] >= m.shape[0] - 2:
        return None  # cut by the image's bottom edge: the corners are not the bag's
    if abs(c[0][1] - c[1][1]) > MAX_TILT_PX:
        return None  # one corner lost (glare washes the bag's colour out, 2026-11-18): not a level face edge
    return {"left": c[0], "right": c[1], "area": int(m.sum())}


# ---------------------------------------------------------------------- discs
DISC_AREA = (25, 4000)  # px: one red disc on the floor, near (big) to far (small)
DISC_FILL = 0.5         # its area over its bounding box's: round or square, not a strip or a bag handle


def detect_discs(img):
    """[(u, v, area)] of the red discs (flat, round, a few cm) in the frame, nearest (lowest in the image) first."""
    import cv2  # noqa: PLC0415
    import gripper_camera as gc  # noqa: PLC0415
    m = gc.ColourBlobDetector(roi_bottom=1.0, min_area=1).mask(img)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
    out = []
    for i in range(1, n):
        area, w, h = stats[i, cv2.CC_STAT_AREA], stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        x0, y0 = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
        on_edge = x0 <= 1 or y0 <= 1 or x0 + w >= img.shape[1] - 1 or y0 + h >= img.shape[0] - 1
        if on_edge:
            continue  # cut by the frame (the jaw tip at the bottom edge, 2026-11-18): its centre is not a disc's
        if DISC_AREA[0] <= area <= DISC_AREA[1] and area >= DISC_FILL * w * h and max(w, h) <= 3 * min(w, h):
            out.append((float(cents[i][0]), float(cents[i][1]), int(area)))
    return sorted(out, key=lambda d: -d[1])


def parse_layout(text):
    """"25:15,-15;40:20,-20" -> [(25, [15, -15]), (40, [20, -20])]: cm from F, then each disc's cm left of the line."""
    rows = []
    for part in text.split(";"):
        front, lefts = part.split(":")
        rows.append((float(front), [float(x) for x in lefts.split(",")]))
    return sorted(rows)


def match_discs(discs, layout):
    """Detections -> layout, row by row: the nearest row is the lowest in the image; in a row, image left = robot left
    (the image is not mirrored: a bag put robot-left showed image-left, 2026-11-18). None unless the counts agree and
    every row sits wholly below the next one in the image."""
    want = sum(len(lefts) for _f, lefts in layout)
    if len(discs) != want:
        return None
    out, i, prev_top = [], 0, None
    for front, lefts in layout:
        row = sorted(discs[i:i + len(lefts)], key=lambda d: d[0])
        i += len(lefts)
        if prev_top is not None and max(d[1] for d in row) >= prev_top:
            return None  # rows overlap in v: the order is not safe
        prev_top = min(d[1] for d in row)
        for (u, v, _a), left in zip(row, sorted(lefts, reverse=True)):
            out.append({"front_cm": front, "left_cm": left, "uv": [u, v]})
    return out


# ====================================================================== geometry
def floor_corners(front_m, left_m, width_m, f_from_lidar=F_FROM_LIDAR, lidar_x=LIDAR_X, z_m=0.0):
    """Robot-frame points (turning centre origin) of the face's lower corners: (robot-left one, robot-right one).
    z_m: how high the detected corner is above the floor (a pale band at the bag's foot is not seen as red)."""
    x = front_m + f_from_lidar + lidar_x
    return np.array([x, left_m + width_m / 2, z_m]), np.array([x, left_m - width_m / 2, z_m])


def _project(cam, p):
    return cam._project(np.asarray(p, float).reshape(1, 3))[0]


def pairs_of(fr, mirror):
    """[(robot point, pixel)] of one frame. A bag frame: its two corners, the image-left one the robot-left one unless
    the image is mirrored. A disc frame ("pairs"): matched already (as seen, not mirrored)."""
    if "pairs" in fr:
        return fr["pairs"]
    p_left, p_right = fr["points"]
    if mirror:  # image left = robot right
        p_left, p_right = p_right, p_left
    return [(p_left, fr["left"]), (p_right, fr["right"])]


def residuals(cam, frames):
    """Pixel errors (u, v) of every point of every frame."""
    out = []
    for fr in frames:
        for p, uv in pairs_of(fr, cam.mirror):
            pr = _project(cam, p)
            out.extend(pr - np.asarray(uv, float) if np.isfinite(pr).all() else (1e3, 1e3))
    return np.array(out)


def fit(frames, guess, free=FREE):
    """Least squares fit of the free FrontCamera parameters; standard errors decide what is determined."""
    from scipy.optimize import least_squares  # noqa: PLC0415
    free = tuple(free)

    def unpack(p):
        ch = {}
        for k, val in zip(free, p):
            ch[k] = math.exp(val) if k in fc.LOG else val
        return guess.copy(**ch)

    p0 = np.array([math.log(getattr(guess, k)) if k in fc.LOG else getattr(guess, k) for k in free])
    m = sum(2 * len(pairs_of(fr, False)) for fr in frames)
    if m < len(free) + 1:
        raise ValueError("%d pixel numbers for %d unknowns: more bag places or discs are needed" % (m, len(free)))
    res = least_squares(lambda p: residuals(unpack(p), frames), p0, method="lm", x_scale="jac")
    cam = unpack(res.x)
    r = res.fun
    rms = math.sqrt(float(r @ r) / len(r))
    sigma = max(rms, fc.NOISE_FLOOR_PX)
    try:
        cov = np.linalg.inv(res.jac.T @ res.jac) * sigma ** 2
        std = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        std = np.full(len(free), np.inf)
    report, determined, undetermined = {}, [], []
    for i, k in enumerate(free):
        value = getattr(cam, k)
        s = float(std[i] * value) if k in fc.LOG else float(std[i])
        report[k] = s
        need = fc.NEED[k] * value if k == "fx" else fc.NEED[k]
        (determined if math.isfinite(s) and s <= need else undetermined).append(k)
    cam.calibrated = not undetermined
    cam.info = {"method": "bag corners on the floor (tools/front_cam_bag_cal.py)", "date": time.strftime("%Y-%m-%d"),
                "rms_px": round(rms, 3), "frames": len(frames), "free": list(free), "undetermined": undetermined,
                "f_from_lidar": frames[0].get("f_from_lidar") if frames else None}
    return {"camera": cam, "rms_px": rms, "std": report, "determined": determined, "undetermined": undetermined,
            "per_frame_px": _per_frame(r, frames), "per_point_px": [float(math.hypot(*r[i:i + 2]))
                                                                    for i in range(0, len(r), 2)]}


def _per_frame(r, frames):
    out, i = [], 0
    for fr in frames:
        n = 2 * len(pairs_of(fr, False))
        out.append(float(np.abs(r[i:i + n]).max()) if n else 0.0)
        i += n
    return out


def fit_best(frames, guess, free=FREE):
    """fit from several start heights and tilts (the fit is not convex from a poor start): the lowest rms."""
    zs = (0.6, 0.9, 1.2) if "z" in free else (guess.z,)      # a fixed (measured) value is never replaced
    pitches = (20, 40, 60) if "pitch" in free else (math.degrees(guess.pitch),)
    starts = [guess.copy(z=z, pitch=math.radians(p)) for z in zs for p in pitches]
    return min((fit(frames, st, free) for st in starts), key=lambda f: f["rms_px"])


def fit_both_mirrors(frames, guess, free=FREE):
    out = {mi: fit_best(frames, guess.copy(mirror=mi), free) for mi in (False, True)}
    best = min(out.values(), key=lambda f: f["rms_px"])
    best["mirror_rms"] = {str(mi).lower(): round(f["rms_px"], 3) for mi, f in out.items()}
    return best


# ====================================================================== session
def load_session(path):
    try:
        with open(path) as f:
            return json.load(f)
    except OSError:
        return {"frames": []}


def frames_of(session, f_from_lidar=F_FROM_LIDAR, lidar_x=LIDAR_X):
    out = []
    for fr in session["frames"]:
        if "discs" in fr:  # flat discs on the floor, z 0
            pairs = [(np.array([d["front_cm"] / 100 + f_from_lidar + lidar_x, d["left_cm"] / 100, 0.0]),
                      np.asarray(d["uv"], float)) for d in fr["discs"]]
            out.append({"name": fr["name"], "pairs": pairs, "f_from_lidar": f_from_lidar})
            continue
        z = 0.0 if fr.get("foot") else fr.get("corner_z_cm", session.get("corner_z_cm", 0.0)) / 100
        pts = floor_corners(fr["front_cm"] / 100, fr["left_cm"] / 100, fr["width_cm"] / 100, f_from_lidar, lidar_x, z)
        out.append({"name": fr["name"], "points": pts, "left": fr["left"], "right": fr["right"],
                    "f_from_lidar": f_from_lidar})
    return out


def _imread(path):
    import cv2  # noqa: PLC0415
    return cv2.imread(path)


def _overlay(img, d, path):
    import cv2  # noqa: PLC0415
    vis = img.copy()
    m = bag_mask(img)
    if m is not None:
        vis[m] = (0.5 * vis[m] + (127, 0, 127)).astype(np.uint8)
    if d:
        for key, col in (("left", (0, 255, 255)), ("right", (255, 255, 0))):
            cv2.circle(vis, tuple(int(c) for c in d[key]), 8, col, 2)
    cv2.imwrite(path, vis)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("detect")
    d.add_argument("image")
    d.add_argument("--out")
    d.add_argument("--foot", action="store_true")
    c = sub.add_parser("capture")
    c.add_argument("--session", required=True)
    c.add_argument("--name", required=True)
    c.add_argument("--front", type=float, required=True, help="cm from F to the face's bottom edge")
    c.add_argument("--left", type=float, required=True, help="cm the face's middle is left of the centre line")
    c.add_argument("--width", type=float, required=True, help="cm, the face's width")
    c.add_argument("--corner-z", type=float, help="cm the red face starts above the floor (the session's default)")
    c.add_argument("--foot", action="store_true", help="use where the bag meets the floor (z 0), not the red face")
    c.add_argument("--url", default=fc.SNAP_URL)
    k = sub.add_parser("discs", help="capture one frame of red discs laid out at measured floor points")
    k.add_argument("--session", required=True)
    k.add_argument("--name", required=True)
    k.add_argument("--layout", required=True, help='rows "CM:LEFT,LEFT;CM:LEFT": cm from F, cm left (right < 0)')
    k.add_argument("--image", help="a saved frame instead of the camera")
    k.add_argument("--url", default=fc.SNAP_URL)
    f = sub.add_parser("fit")
    f.add_argument("session")
    f.add_argument("--write")
    f.add_argument("--free", default=",".join(FREE))
    f.add_argument("--f-from-lidar", type=float, default=F_FROM_LIDAR)
    f.add_argument("--lidar-x", type=float, default=LIDAR_X)
    a = ap.parse_args(argv)

    if a.cmd == "detect":
        img = _imread(a.image)
        det = detect(img, foot=a.foot) if img is not None else None
        print(json.dumps(det))
        if a.out and img is not None:
            _overlay(img, det, a.out)
        return 0 if det else 1

    if a.cmd == "capture":
        import cv2  # noqa: PLC0415
        data = urllib.request.urlopen(a.url, timeout=5).read()
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        det = detect(img, foot=a.foot)
        folder = os.path.dirname(os.path.abspath(a.session))
        os.makedirs(folder, exist_ok=True)
        cv2.imwrite(os.path.join(folder, a.name + ".jpg"), img)
        _overlay(img, det, os.path.join(folder, a.name + "_overlay.jpg"))
        if det is None:
            print("no usable bag in the frame (whole, not cut by the image edge, both lower corners seen; see %s): "
                  "nothing added" % os.path.join(folder, a.name + "_overlay.jpg"))
            return 1
        s = load_session(a.session)
        if a.corner_z is not None:
            s["corner_z_cm"] = a.corner_z
        s["frames"] = [fr for fr in s["frames"] if fr["name"] != a.name] + [
            {"name": a.name, "image": a.name + ".jpg", "foot": a.foot, "front_cm": a.front, "left_cm": a.left, "width_cm": a.width,
             "left": det["left"], "right": det["right"], "area": det["area"]}]
        with open(a.session, "w") as fh:
            json.dump(s, fh, indent=2)
        print("%s: corners left (%.0f, %.0f) right (%.0f, %.0f) px, %d frames in %s"
              % (a.name, *det["left"], *det["right"], len(s["frames"]), a.session))
        return 0

    if a.cmd == "discs":
        import cv2  # noqa: PLC0415
        if a.image:
            img = cv2.imread(a.image)
        else:
            data = urllib.request.urlopen(a.url, timeout=5).read()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        folder = os.path.dirname(os.path.abspath(a.session))
        os.makedirs(folder, exist_ok=True)
        cv2.imwrite(os.path.join(folder, a.name + ".jpg"), img)
        discs = detect_discs(img)
        vis = img.copy()
        for u, v, _ar in discs:
            cv2.circle(vis, (int(u), int(v)), 10, (0, 255, 255), 2)
        cv2.imwrite(os.path.join(folder, a.name + "_overlay.jpg"), vis)
        layout = parse_layout(a.layout)
        matched = match_discs(discs, layout)
        print("found %d discs: %s" % (len(discs), ", ".join("(%.0f, %.0f) %dpx" % d for d in discs)))
        if matched is None:
            print("not matched to the layout (%d discs wanted, rows must not overlap in the image; see %s)"
                  % (sum(len(lf) for _f, lf in layout), os.path.join(folder, a.name + "_overlay.jpg")))
            return 1
        s = load_session(a.session)
        s["frames"] = [fr for fr in s["frames"] if fr["name"] != a.name] + [
            {"name": a.name, "image": a.name + ".jpg", "discs": matched}]
        with open(a.session, "w") as fh:
            json.dump(s, fh, indent=2)
        for d in matched:
            print("  F+%4.0f cm, left %4.0f cm -> (%.0f, %.0f) px" % (d["front_cm"], d["left_cm"], *d["uv"]))
        return 0

    s = load_session(a.session)
    frames = frames_of(s, a.f_from_lidar, a.lidar_x)
    guess = fc.FrontCamera(x=a.f_from_lidar + a.lidar_x)  # over the LiDAR, under the top deck: the fit moves it
    out = fit_both_mirrors(frames, guess, tuple(a.free.split(",")))
    cam = out["camera"]
    print("rms %.2f px (mirror rms %s), frames %d" % (out["rms_px"], out["mirror_rms"], len(frames)))
    for k in out["std"]:
        val = getattr(cam, k)
        shown = math.degrees(val) if k in fc.ANGLES else val
        err = math.degrees(out["std"][k]) if k in fc.ANGLES else out["std"][k]
        print("  %-5s %9.3f  +- %.3f%s" % (k, shown, err, "" if k in out["determined"] else "  NOT DETERMINED"))
    for fr, px in zip(frames, out["per_frame_px"]):
        print("  frame %-6s worst corner %.1f px" % (fr["name"], px))
    if a.write:
        if out["undetermined"]:
            print("not written: %s not determined (more bag places, farther apart)" % ", ".join(out["undetermined"]))
            return 1
        cam.save(os.path.expanduser(a.write))
        print("saved %s" % a.write)
    return 0


if __name__ == "__main__":
    sys.exit(main())
