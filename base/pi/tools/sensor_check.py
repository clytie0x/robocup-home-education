"""Measure the camera pose and the LiDAR yaw with the robot standing still.

  python tools/sensor_check.py --floor   empty floor ahead: camera height and pitch from 30 depth frames
  python tools/sensor_check.py --yaw     bottle 1.5 m straight ahead: LiDAR and image bearing of it
  python tools/sensor_check.py --wall    camera pitch from the largest vertical plane (no floor needed)
  python tools/sensor_check.py --object H  camera pitch from an object H m tall straight ahead

Each mode prints a summary and writes a JSON result to logs/sensor-check/ next to tools/.
"""

import argparse
import collections
import json
import math
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # camera.py, lidar.py, params.py
import camera  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "logs" / "sensor-check"


def pixel_rays(shape, hfov_deg):
    """Per pixel (x, y) of the ray (x, y, 1): pinhole, square pixels, x right, y down, z forward."""
    height, width = shape
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    return (u - (width - 1) / 2) / f, (v - (height - 1) / 2) / f


def _local_triples(rows, cols, valid, rng, count, reach=(10, 60)):
    """Random point triples drawn close together in the image, so most lie on one surface.

    Drawing three points from the whole image fails when the floor is only a thin strip
    (camera low and tilted up): almost every triple would mix floor and wall.
    """
    seed = rng.integers(0, len(rows), count)
    r = rows[seed][:, None] + rng.integers(-reach[0], reach[0] + 1, (count, 2))
    c = cols[seed][:, None] + rng.integers(-reach[1], reach[1] + 1, (count, 2))
    r = np.clip(r, 0, valid.shape[0] - 1)
    c = np.clip(c, 0, valid.shape[1] - 1)
    keep = valid[r, c].all(axis=1)
    return np.column_stack([rows[seed], r[:, 0], r[:, 1]])[keep], np.column_stack([cols[seed], c[:, 0], c[:, 1]])[keep]


def _plane_through(points):
    """Unit normal n pointing to the camera's up side (y < 0) and offset d with n . p = d."""
    centre = points.mean(axis=0)
    normal = np.linalg.svd(points - centre, full_matrices=False)[2][-1]  # least-squares plane normal
    if normal[1] > 0:
        normal = -normal
    return normal, float(normal @ centre)


# A floor fit outside these is a wall or noise, not the floor (lens is ~7 cm up, tilt 15-25 deg).
PLAUSIBLE_HEIGHT = (0.02, 0.50)
PLAUSIBLE_PITCH = (-45.0, 45.0)


def fit_floor(depth, hfov_deg, min_range=1.6, tries=400, tolerance=0.005, max_tilt_deg=45.0, seed=0, min_inliers=None):
    """Floor plane from one depth image: (camera_height m, pitch_deg, inlier_share).

    pitch_deg is positive when the camera is tilted UP, negative when tilted down.
    Only pixels in the lower half of the image with depth beyond min_range are used: the
    camera sits low and tilted up, so the near floor is out of view anyway, and the far
    floor is where the depth is good enough. RANSAC keeps the plane with the most inliers
    among planes that lie below the camera with a normal within max_tilt_deg of the
    camera's up direction (0, -1, 0); walls and boxes fail that test. inlier_share is the
    share of the used pixels on the floor plane; it is small by nature (about 5 % when tilted
    20 deg up), so the guard is a pixel count instead: fewer than min_inliers floor pixels
    (default 1.3 % of the image: 4000 at 640x480, 1000 at 320x240; a noisy wall alone gives
    up to ~3000 at 640x480, the 20 deg-up floor ~7700) raises
    ValueError, as do a height outside PLAUSIBLE_HEIGHT or a pitch outside PLAUSIBLE_PITCH.
    """
    z = depth.astype(float) / 1000.0
    x, y = pixel_rays(depth.shape, hfov_deg)
    valid = z > min_range
    valid[: depth.shape[0] // 2] = False
    rows, cols = np.nonzero(valid)
    if len(rows) < 3:
        raise ValueError("no depth beyond %.1f m in the lower half" % min_range)
    cloud = np.stack([x * z, y * z, z], axis=-1)  # metres, camera frame
    points = cloud[rows, cols]

    rng = np.random.default_rng(seed)
    sample = points[rng.choice(len(points), min(len(points), 20000), replace=False)]  # scoring speed
    best, best_count = None, 0
    for tr, tc in zip(*_local_triples(rows, cols, valid, rng, tries)):
        a, b, c = cloud[tr, tc]
        normal = np.cross(b - a, c - a)
        size = np.linalg.norm(normal)
        if size < 1e-6:
            continue
        normal = normal / size if normal[1] < 0 else -normal / size
        d = float(normal @ a)
        if d >= 0 or -normal[1] < math.cos(math.radians(max_tilt_deg)):
            continue  # above the camera (a ceiling) or too steep (a wall)
        count = int((np.abs(sample @ normal - d) < tolerance).sum())
        if count > best_count:
            best, best_count = (normal, d), count
    if best is None:
        raise ValueError("no floor-like plane found")

    # The band is tight (5 mm) because the far floor is seen at a grazing angle: a plane a
    # degree off would still hold the whole floor strip, plus the foot of the wall behind it.
    # Depth noise lies along the ray, so it barely moves floor points off the plane.
    normal, d = best
    for _ in range(2):
        normal, d = _plane_through(points[np.abs(points @ normal - d) < tolerance])
    if d >= 0 or -normal[1] < math.cos(math.radians(max_tilt_deg)):
        raise ValueError("refit left the floor-like range")  # e.g. a wall took over the inliers
    inliers = np.abs(points @ normal - d) < tolerance
    pitch = math.degrees(math.atan2(normal[2], -normal[1]))  # floor normal leans to +z when tilted up
    if min_inliers is None:
        min_inliers = round(0.013 * depth.size)  # frame size is only known at capture
    if inliers.sum() < min_inliers:
        raise ValueError("only %d floor pixels (need %d)" % (inliers.sum(), min_inliers))
    if not PLAUSIBLE_HEIGHT[0] <= -d <= PLAUSIBLE_HEIGHT[1] or not PLAUSIBLE_PITCH[0] <= pitch <= PLAUSIBLE_PITCH[1]:
        raise ValueError("implausible floor: height %.3f m, pitch %.1f deg" % (-d, pitch))
    return -d, pitch, float(inliers.mean())


def fit_wall(depth, hfov_deg, near=1.0, far=4.5, tries=400, tolerance=0.08, min_share=0.03, max_yaw_deg=60.0, seed=0, refit=(0.15, 0.1, 0.05)):
    """Camera pitch from the largest vertical plane in view: (pitch_deg, inlier_share, wall_yaw_deg).

    For the floor-free view of a low camera tilted up. A vertical wall's normal n is horizontal,
    so with no roll it is perpendicular to world up (0, -cos p, sin p) in camera coordinates
    (x right, y down, z forward): pitch_up p = atan2(n_y, n_z) with n_z > 0, whatever the wall's
    yaw. Bands are wide because Xtion depth at 3-4 m comes in steps of several cm; the refit
    narrows from 15 cm to 5 cm. RANSAC on valid depth near..far keeps the plane with most inliers whose implied pitch is
    within PLAUSIBLE_PITCH (the floor implies p - 90 deg and is skipped). Raises ValueError when
    the inliers are under min_share of the image (inlier_share is of the whole image too). wall_yaw_deg: the normal's horizontal direction,
    positive = it points to the robot's left (the image is mirrored: image right, x > 0, is the
    robot's left; mirroring does not change the pitch).
    """
    z = depth.astype(float) / 1000.0
    x, y = pixel_rays(depth.shape, hfov_deg)
    valid = (z >= near) & (z <= far)
    rows, cols = np.nonzero(valid)
    if len(rows) < 3:
        raise ValueError("no depth between %.1f and %.1f m" % (near, far))
    cloud = np.stack([x * z, y * z, z], axis=-1)
    points = cloud[rows, cols]

    def wall_pitch(normal):
        if normal[2] < 0:
            normal = -normal
        yaw = math.degrees(math.atan2(normal[0], math.hypot(normal[1], normal[2])))
        return normal, math.degrees(math.atan2(normal[1], normal[2])), yaw

    def wall_like(pitch, yaw):
        # A side wall seen edge-on has a nearly sideways normal and an undefined pitch.
        return PLAUSIBLE_PITCH[0] <= pitch <= PLAUSIBLE_PITCH[1] and abs(yaw) <= max_yaw_deg

    rng = np.random.default_rng(seed)
    sample = points[rng.choice(len(points), min(len(points), 20000), replace=False)]
    best, best_count = None, 0
    for tr, tc in zip(*_local_triples(rows, cols, valid, rng, tries)):
        a, b, c = cloud[tr, tc]
        normal = np.cross(b - a, c - a)
        size = np.linalg.norm(normal)
        if size < 1e-6:
            continue
        normal, pitch, yaw = wall_pitch(normal / size)
        if not wall_like(pitch, yaw):
            continue
        d = float(normal @ a)
        count = int((np.abs(sample @ normal - d) < tolerance).sum())
        if count > best_count:
            best, best_count = (normal, d), count
    if best is None:
        raise ValueError("no wall-like plane found")
    normal, d = best
    for band in refit:
        normal, d = _plane_through(points[np.abs(points @ normal - d) < band])  # |d| test, sign-free
    inliers = np.abs(points @ normal - d) < refit[-1]
    normal, pitch, yaw = wall_pitch(normal)
    if not wall_like(pitch, yaw):
        raise ValueError("refit left the wall-like range")
    if inliers.sum() < min_share * depth.size:
        raise ValueError("only %d wall pixels (need %d)" % (inliers.sum(), round(min_share * depth.size)))
    return pitch, float(inliers.sum() / depth.size), yaw


def object_pitch(depth, hfov_deg, object_height, scan_range, camera_height, camera_dx, window=0.15, min_pixels=3):
    """Camera pitch_up in deg from the top edge of an object of known height straight ahead, or None.

    Pixels in the middle quarter of the columns whose 3-D range is within `window` of the
    camera-to-object distance r = scan_range - camera_dx form the object; its top edge is the
    topmost row with at least min_pixels of them, taken half a row higher (the edge lies
    between that row and the empty one above). The edge is seen at elevation
    atan2(object_height - camera_height, r) above level and at -atan((v - cy) / f) above the
    optical axis, so pitch_up = the first minus the second.
    """
    r = scan_range - camera_dx
    x, y = pixel_rays(depth.shape, hfov_deg)
    ranges = depth.astype(float) / 1000.0 * np.sqrt(1 + x**2 + y**2)
    width = depth.shape[1]
    middle = slice(width * 3 // 8, width * 5 // 8)
    hits = ((depth > 0) & (np.abs(ranges - r) < window))[:, middle].sum(axis=1)
    rows = np.nonzero(hits >= min_pixels)[0]
    if len(rows) == 0:
        return None
    top = rows[0]
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    edge_above_axis = -math.atan((top - 0.5 - (depth.shape[0] - 1) / 2) / f)
    return math.degrees(math.atan2(object_height - camera_height, r) - edge_above_axis)


def scan_bearing(scan, near=1.5, half_width_deg=20.0, ranges=(1.0, 2.0), gap=0.1):
    """(bearing deg, range m) of the scan cluster nearest `near` metres ahead, or None.

    Points within +-half_width_deg and `ranges`, grouped where neighbours are within `gap`
    metres. The bearing is that of the cluster's centre point. Stand-in until legs.py.
    """
    points = sorted(
        (deg, m) for deg, m in lidar.usable(scan) if abs(deg) <= half_width_deg and ranges[0] <= m <= ranges[1]
    )
    xy = [(m * math.cos(math.radians(deg)), m * math.sin(math.radians(deg))) for deg, m in points]
    clusters = []
    for p in xy:
        if clusters and math.dist(clusters[-1][-1], p) <= gap:
            clusters[-1].append(p)
        else:
            clusters.append([p])
    if not clusters:
        return None
    centres = [(sum(p[0] for p in c) / len(c), sum(p[1] for p in c) / len(c)) for c in clusters]
    cx, cy = min(centres, key=lambda c: abs(math.hypot(*c) - near))
    return math.degrees(math.atan2(cy, cx)), math.hypot(cx, cy)


def suggested_front(current_front_deg, scan_deg, true_deg=0.0):
    """lidar_front_deg that makes the bottle read its true bearing (0 deg when straight ahead).

    lidar.py reports base_deg = front - laser_deg, so laser_deg = current - scan_deg, and the
    new front must give true_deg = new - laser_deg: new = current - (scan_deg - true_deg)
    (a bottle read too far left needs a smaller front angle).
    """
    return current_front_deg - (scan_deg - true_deg)


def side(deg):
    return "left" if deg > 0 else "right" if deg < 0 else "centre"


def image_bearing(depth, hfov_deg, scan_range, camera_y, camera_dx, window=0.3):
    """Bearing in deg of the bottle as seen from the LiDAR, taken from the depth image, or None.

    Pixels whose 3-D range is within `window` of the bottle's expected range from the camera
    (r = scan_range - camera_dx) are counted per column. The whole image is used, not a middle
    band: with the camera low and tilted up, a bottle on the floor sits near the bottom rows.
    The bottle is the run of columns holding at least half the best column's count (floor rows
    at the same range add only a few pixels per column). Its centre column u gives the camera
    bearing theta = atan((u - cx) / f), positive left: the Xtion image is mirrored, image right
    is the robot's left (hardware assessment, measured 2026-11-07; link.bearing_deg uses the
    same rule). Camera pitch is ignored, it is small near the centre column. The camera sits camera_y to the left and camera_dx ahead of the LiDAR, so
    the bearing from the LiDAR is atan2(camera_y + r sin(theta), camera_dx + r cos(theta)).
    """
    r = scan_range - camera_dx
    x, y = pixel_rays(depth.shape, hfov_deg)
    ranges = depth.astype(float) / 1000.0 * np.sqrt(1 + x**2 + y**2)
    per_column = ((depth > 0) & (np.abs(ranges - r) < window)).sum(axis=0)
    peak = int(per_column.argmax())
    if per_column[peak] < 10:
        return None
    strong = per_column >= per_column[peak] / 2
    left = right = peak
    while left > 0 and strong[left - 1]:
        left -= 1
    while right < len(strong) - 1 and strong[right + 1]:
        right += 1
    theta = math.atan((x[0, left] + x[0, right]) / 2)  # mirrored image: image right (x > 0) is robot left
    return math.degrees(math.atan2(camera_y + r * math.sin(theta), camera_dx + r * math.cos(theta)))


def _frames(cam, count, timeout=15.0):
    frames = []
    deadline = time.monotonic() + timeout
    while len(frames) < count and time.monotonic() < deadline:
        if cam.error is not None:
            raise cam.error
        frame = cam.latest()
        if frame is None:
            time.sleep(0.02)
        else:
            frames.append(frame)
    return frames


def _scans(session, count, timeout=10.0):
    scans, last = [], None
    deadline = time.monotonic() + timeout
    while len(scans) < count and time.monotonic() < deadline:
        scan = session.latest()
        if scan and scan is not last:  # ScanStream returns the same list until a new turn ends
            scans.append(scan)
            last = scan
        time.sleep(0.02)
    return scans


def _write(out_dir, name, result):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / (time.strftime("%Y%m%d-%H%M%S") + "-" + name + ".json")
    path.write_text(json.dumps(result, indent=2))
    print("wrote", path)


def check_floor(p, out_dir, count=30):
    cam = camera.Camera()
    cam.start()
    try:
        frames = _frames(cam, count)
    finally:
        cam.stop()
    fits, reasons = [], collections.Counter()
    for frame in frames:
        try:
            fits.append(fit_floor(frame.depth, p["camera_hfov_deg"]))
        except ValueError as error:
            reasons[str(error).split(":")[0].split(" (")[0]] += 1
    misses = len(frames) - len(fits)
    if not fits:
        sys.exit("FAILED: no floor plane in %d frames read (asked %d). Reasons: %s" % (len(frames), count, dict(reasons)))
    heights, pitches, shares = zip(*fits)
    print("frames %d, fitted %d, no floor %d %s" % (len(frames), len(fits), misses, dict(reasons) if reasons else ""))
    print("camera_height  median %.3f m   spread %.3f .. %.3f" % (statistics.median(heights), min(heights), max(heights)))
    print("camera_pitch   median %.2f deg spread %.2f .. %.2f (positive = tilted up)" % (statistics.median(pitches), min(pitches), max(pitches)))
    print("inlier share   median %.2f" % statistics.median(shares))
    warnings = []
    if len(fits) < 20:
        warnings.append("only %d of %d frames fit (want 20+)" % (len(fits), count))
    if max(heights) - min(heights) > 0.01:
        warnings.append("height spread %.3f m > 0.01 m" % (max(heights) - min(heights)))
    if max(pitches) - min(pitches) > 1.0:
        warnings.append("pitch spread %.2f deg > 1 deg" % (max(pitches) - min(pitches)))
    for warning in warnings:
        print("WARNING:", warning, "- do not trust this result")
    _write(out_dir, "floor", {
        "method": "RANSAC floor plane, lower half of the depth image, depth beyond 1.6 m",
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frames": len(frames), "misses": misses,
        "camera_height": statistics.median(heights), "camera_height_range": [min(heights), max(heights)],
        "camera_pitch_deg": statistics.median(pitches), "camera_pitch_range": [min(pitches), max(pitches)],
        "inlier_share": statistics.median(shares),
        "per_frame": [list(f) for f in fits], "miss_reasons": dict(reasons), "warnings": warnings,
    })


def check_yaw(p, out_dir, offset_left=0.0, count=10):
    session = lidar.LidarSession()
    try:
        scans = _scans(session, count)
    finally:
        session.close()
    cam = camera.Camera()
    cam.start()
    try:
        frames = _frames(cam, count)
    finally:
        cam.stop()

    seen = [b for b in (scan_bearing(s) for s in scans) if b is not None]
    if not seen:
        print("no bottle in the scan (%d scans): need a cluster within +-20 deg, 1.0-2.0 m" % len(scans))
        return
    scan_deg = statistics.median(b[0] for b in seen)
    scan_range = statistics.median(b[1] for b in seen)
    # camera_x is still a gate value (null until Task 3 step 4); until then assume the lens is at the LiDAR's x.
    camera_dx = (p["camera_x"] - p["lidar_x"]) if p["camera_x"] is not None else 0.0
    images = [b for b in (image_bearing(f.depth, p["camera_hfov_deg"], scan_range, p["camera_y"], camera_dx) for f in frames) if b is not None]
    image_deg = statistics.median(images) if images else None
    # True bearing of the bottle from the LiDAR, placed offset_left to the left at the scan's range.
    true_deg = math.degrees(math.asin(max(-1.0, min(1.0, offset_left / scan_range))))
    front = suggested_front(lidar.FRONT_LASER_DEG, scan_deg, true_deg)

    print("true   bearing %.2f deg %s (bottle placed %.3f m left)" % (true_deg, side(true_deg), offset_left))
    print("scan   bearing %.2f deg %s at %.3f m (%d of %d scans)" % (scan_deg, side(scan_deg), scan_range, len(seen), len(scans)))
    if lidar.FRONT_LASER_DEG != p["lidar_front_deg"]:
        print("note: lidar.py FRONT_LASER_DEG %.2f differs from params lidar_front_deg %.2f" % (lidar.FRONT_LASER_DEG, p["lidar_front_deg"]))
    print("suggested lidar_front_deg %.2f (now %.2f)" % (front, lidar.FRONT_LASER_DEG))
    agree = None
    if image_deg is None:
        print("image  no bottle found in %d frames" % len(frames))
    else:
        # With the suggested front the scan reads true_deg, so the image must be within 2 deg of it.
        agree = abs(image_deg - true_deg) <= 2.0
        print("image  bearing %.2f deg %s (%d of %d frames), raw image - scan %.2f deg" % (image_deg, side(image_deg), len(images), len(frames), image_deg - scan_deg))
        print("image agrees with the corrected scan within 2 deg:", "YES" if agree else "NO")
    sides_ok = None
    if offset_left != 0:
        # The sign proof: a bottle put on the left must read left in both sensors.
        sides_ok = side(scan_deg) == side(true_deg) and image_deg is not None and side(image_deg) == side(true_deg)
        print("scan and image both put it on the %s:" % side(true_deg), "YES" if sides_ok else "NO")
    _write(out_dir, "yaw", {
        "date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "scan_deg": scan_deg, "scan_range": scan_range, "scans": len(scans), "scans_seen": len(seen),
        "image_deg": image_deg, "frames": len(frames), "frames_seen": len(images),
        "front_now": lidar.FRONT_LASER_DEG, "front_suggested": front, "agree_within_2deg": agree,
        "camera_y": p["camera_y"], "camera_dx": camera_dx,
        "offset_left": offset_left, "true_deg": true_deg, "sides_ok": sides_ok,
    })


def _spread(values):
    return statistics.median(values), min(values), max(values)


def _pitch_warnings(pitches, count):
    warnings = []
    if len(pitches) < 20:
        warnings.append("only %d of %d frames fit (want 20+)" % (len(pitches), count))
    if max(pitches) - min(pitches) > 1.0:
        warnings.append("pitch spread %.2f deg > 1 deg" % (max(pitches) - min(pitches)))
    for warning in warnings:
        print("WARNING:", warning, "- do not trust this result")
    return warnings


def _read_frames(count):
    cam = camera.Camera()
    cam.start()
    try:
        return _frames(cam, count)
    finally:
        cam.stop()


def check_wall(p, out_dir, count=30):
    frames = _read_frames(count)
    fits, reasons = [], collections.Counter()
    for frame in frames:
        try:
            fits.append(fit_wall(frame.depth, p["camera_hfov_deg"]))
        except ValueError as error:
            reasons[str(error).split(" (")[0]] += 1
    if not fits:
        sys.exit("FAILED: no wall plane in %d frames read (asked %d). Reasons: %s" % (len(frames), count, dict(reasons)))
    pitches, shares, yaws = zip(*fits)
    print("frames %d, fitted %d %s" % (len(frames), len(fits), dict(reasons) if reasons else ""))
    print("camera_pitch  median %.2f deg spread %.2f .. %.2f (positive = tilted up)" % _spread(pitches))
    print("inlier share  median %.2f of the image" % statistics.median(shares))
    print("wall yaw      median %.1f deg spread %.1f .. %.1f (normal, positive = robot's left)" % _spread(yaws))
    warnings = _pitch_warnings(pitches, count)
    _write(out_dir, "wall", {
        "method": "RANSAC largest vertical plane, depth 1-4.5 m, pitch = atan2(n_y, n_z)",
        "date": time.strftime("%Y-%m-%d %H:%M:%S"), "frames": len(frames), "misses": len(frames) - len(fits),
        "camera_pitch_deg": statistics.median(pitches), "camera_pitch_range": [min(pitches), max(pitches)],
        "inlier_share": statistics.median(shares), "wall_yaw_deg": statistics.median(yaws),
        "per_frame": [list(f) for f in fits], "miss_reasons": dict(reasons), "warnings": warnings,
    })


def check_object(p, out_dir, object_height, camera_height, count=30):
    session = lidar.LidarSession()
    try:
        scans = _scans(session, 10)
    finally:
        session.close()
    seen = [b for b in (scan_bearing(s) for s in scans) if b is not None]
    if not seen:
        sys.exit("FAILED: no object in the scan (%d scans): need a cluster within +-20 deg, 1.0-2.0 m, taller than the LiDAR" % len(scans))
    scan_range = statistics.median(b[1] for b in seen)
    camera_dx = (p["camera_x"] - p["lidar_x"]) if p["camera_x"] is not None else 0.0
    frames = _read_frames(count)
    pitches = [v for v in (object_pitch(f.depth, p["camera_hfov_deg"], object_height, scan_range, camera_height, camera_dx) for f in frames) if v is not None]
    if not pitches:
        sys.exit("FAILED: object top edge not found in %d frames (range %.3f m)" % (len(frames), scan_range))
    print("object %.3f m tall at %.3f m (scan), camera_height %.3f m, camera_dx %.3f m" % (object_height, scan_range, camera_height, camera_dx))
    print("camera_pitch  median %.2f deg spread %.2f .. %.2f (positive = tilted up, %d of %d frames)" % (_spread(pitches) + (len(pitches), len(frames))))
    warnings = _pitch_warnings(pitches, count)
    _write(out_dir, "object", {
        "method": "top edge of a known-height object straight ahead, range from the LiDAR",
        "date": time.strftime("%Y-%m-%d %H:%M:%S"), "frames": len(frames), "misses": len(frames) - len(pitches),
        "object_height": object_height, "scan_range": scan_range, "camera_height": camera_height, "camera_dx": camera_dx,
        "camera_pitch_deg": statistics.median(pitches), "camera_pitch_range": [min(pitches), max(pitches)],
        "per_frame": pitches, "warnings": warnings,
    })


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--floor", action="store_true", help="camera height and pitch from an empty floor")
    mode.add_argument("--yaw", action="store_true", help="bearing of a bottle 1.5 m straight ahead")
    mode.add_argument("--wall", action="store_true", help="camera pitch from the largest vertical plane in view")
    mode.add_argument("--object", type=float, metavar="H",
                      help="camera pitch from an object H m tall straight ahead at 1-2 m (taller than the LiDAR, 0.22 m)")
    parser.add_argument("--camera-height", type=float, metavar="M",
                        help="--object: lens height (default params camera_height, else 0.07)")
    parser.add_argument("--offset-left", type=float, default=0.0, metavar="M",
                        help="--yaw: the bottle is M metres to the robot's left (negative = right), at 1.5 m")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="folder for the JSON result")
    args = parser.parse_args()
    p = params.load()
    if args.floor:
        check_floor(p, args.out)
    elif args.wall:
        check_wall(p, args.out)
    elif args.object is not None:
        height = args.camera_height
        if height is None:
            height = p["camera_height"] if p["camera_height"] is not None else 0.07
        check_object(p, args.out, args.object, height)
    else:
        check_yaw(p, args.out, args.offset_left)


if __name__ == "__main__":
    main()
