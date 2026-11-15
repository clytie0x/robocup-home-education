"""Calibrate the fixed front camera (front_camera.py) in front of a flat wall, with the room lit.

The procedure (base/docs/bag-sensing.md has it in full):
  1. The robot stands square to a flat wall with a black baseboard (the wall-floor line is its bottom edge), the
     floor in front clear. The LiDAR gives the wall line (a line fit: normal angle and distance, in the robot
     frame, from the turning centre). Best with a vertical edge (a cabinet side) in view.
  2. It turns on the spot by small LiDAR-measured angles (about -15, -7, 0, 7, 15 deg), then drives straight
     back about 0.6 m (LiDAR-measured) and turns the same again. After each move, with the robot standing:
        python3 tools/front_cam_cal.py capture --session cal/session.json --name t3 --pose X,Y,YAW_DEG \
            [--wall NORMAL_DEG,DIST] [--edge cabinet=U] [--url http://<jetson>:8082/snap.jpg]
     pose: the robot in the world frame (the first frame's robot frame) from the LiDAR; --wall: the LiDAR wall
     fit in this frame's robot frame (else the session's world wall moved by the pose); --edge NAME=U: a vertical
     edge near column U (the tool finds it there).
  3. python3 tools/front_cam_cal.py fit cal/session.json [--write ~/hmc-follow/front_camera.json]
     Least squares for fx, yaw, pitch, roll and the height (cx, cy at the image centre; the camera's x, y from the
     session's camera_guess unless --free adds them). Every parameter the frames cannot pin to 1 % (fx), 0.3 deg
     or 1 cm is printed as not determined, and then nothing is written: add frames or fix it by a measurement.
     The image may be mirrored: both are fitted and the lower rms wins (--mirror yes|no to force it).
  Offline: the session may name saved frames ("image") or give the pixels directly ("floor": [[u, v], ...],
  "edges": {"cabinet": [[u, v or null], ...]}).
  python3 tools/front_cam_cal.py detect FRAME.jpg [--out overlay.png]   shows what is found in one frame.

Session JSON: {"camera_guess": {"fx", "x", "y", "z", "yaw_deg", "pitch_deg", "roll_deg"}, "free": [...],
  "walls": [{"normal_deg", "dist"}] (world frame), "frames": [{"image" | "floor", "pose": [x, y, yaw_deg],
  "wall": index into walls (default 0) | {"normal_deg", "dist"} (robot frame), "floor_rows": [lo, hi],
  "edges": {name: {"u": U[, "rows": [lo, hi]]} | [[u, v], ...]}}]}. Image paths are relative to the session file.
"""
import argparse
import json
import math
import os
import sys
import urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import front_camera as fc  # noqa: E402

MAX_RMS = 1.5  # px: a fit worse than this is not written (something in the frames is not the wall or the edge)


def _cv2():
    import cv2  # noqa: PLC0415
    return cv2


def _load_session(path):
    with open(path) as f:
        return json.load(f)


def _guess(session):
    g = session.get("camera_guess") or {}
    return fc.FrontCamera(fx=g.get("fx", fc.F_GUESS), x=g.get("x", fc.X_GUESS), y=g.get("y", 0.0),
                          z=g.get("z", fc.Z_GUESS), yaw=math.radians(g.get("yaw_deg", 0.0)),
                          pitch=math.radians(g.get("pitch_deg", 0.0)), roll=math.radians(g.get("roll_deg", 0.0)),
                          width=g.get("width", fc.WIDTH), height=g.get("height", fc.HEIGHT))


def _robot_wall(session, fr, pose):
    w = fr.get("wall", 0)
    if isinstance(w, dict):
        return math.radians(w["normal_deg"]), float(w["dist"])
    walls = session.get("walls") or []
    if not walls:
        return None
    wall = walls[int(w)]
    nw = math.radians(wall["normal_deg"])
    n = np.array([math.cos(nw), math.sin(nw)])
    return nw - pose[2], float(wall["dist"]) - float(n @ np.array(pose[:2]))


def frames_from_session(session, base_dir, log=print):
    """The session's frames as front_camera.fit wants them (detecting lines and edges in saved images)."""
    out = []
    for i, fr in enumerate(session.get("frames") or []):
        x, y, yaw_deg = fr["pose"]
        pose = (float(x), float(y), math.radians(yaw_deg))
        item = {"pose": pose, "wall": _robot_wall(session, fr, pose), "floor": None, "edges": {}}
        img = None
        if fr.get("image"):
            img = _cv2().imread(os.path.join(base_dir, fr["image"]))
            if img is None:
                raise ValueError("frame %d: cannot read %s" % (i, fr["image"]))
        if fr.get("floor") is not None:
            item["floor"] = np.asarray(fr["floor"], float).reshape(-1, 2)
        elif img is not None:
            fl = fc.detect_floor_line(img, rows=fr.get("floor_rows"))
            if fl is None:
                log("frame %d (%s): no wall-floor line found" % (i, fr.get("image")))
            else:
                item["floor"] = fl["points"]
                log("frame %d (%s): wall-floor line from %d columns, rms %.2f px"
                    % (i, fr.get("image"), fl["count"], fl["rms"]))
        for name, e in (fr.get("edges") or {}).items():
            if isinstance(e, dict):
                if img is None:
                    pts = np.array([[e["u"], np.nan]])  # a column only
                else:
                    pts = fc.detect_vertical_edge(img, e["u"], rows=tuple(e.get("rows") or (0, None)))
                    if not len(pts):
                        log("frame %d: edge %s not found near column %.0f" % (i, name, e["u"]))
                        continue
            else:
                pts = np.array([[np.nan if c is None else c for c in p] for p in e], float).reshape(-1, 2)
            item["edges"][name] = pts
        out.append(item)
    return out


def report(res, log=print):
    cam = res["camera"]
    log("rms %.3f px over %d residuals%s" % (res["rms_px"], res["residuals"],
        ("; mirror rms %s" % res["mirror_rms"]) if "mirror_rms" in res else ""))
    shown = {"fx": (cam.fx, "px", 1.0), "x": (cam.x, "m", 1.0), "y": (cam.y, "m", 1.0), "z": (cam.z, "m", 1.0),
             "yaw": (math.degrees(cam.yaw), "deg", math.degrees(1)), "pitch": (math.degrees(cam.pitch), "deg",
             math.degrees(1)), "roll": (math.degrees(cam.roll), "deg", math.degrees(1))}
    for k, (value, unit, scale) in shown.items():
        if k in res["std"]:
            flag = "ok" if k in res["determined"] else "NOT DETERMINED"
            log("  %-5s %9.4f %-3s +- %.4f  %s" % (k, value, unit, res["std"][k] * scale, flag))
        else:
            log("  %-5s %9.4f %-3s (fixed)" % (k, value, unit))
    log("  mirror %s" % cam.mirror)
    for name, e in res["edges"].items():
        log("  edge %s at world (%.3f, %.3f) +- %.3f m" % (name, e["x"], e["y"], e["std"]))
    if res["undetermined"]:
        log(res["why"])


def cmd_fit(a, log=print):
    session = _load_session(a.session)
    frames = frames_from_session(session, os.path.dirname(os.path.abspath(a.session)), log)
    guess = _guess(session)
    free = tuple(a.free.split(",")) if a.free else tuple(session.get("free") or fc.FIT_FREE)
    try:
        if a.mirror == "auto":
            res = fc.fit_both_mirrors(frames, guess, free)
        else:
            res = fc.fit(frames, guess.copy(mirror=a.mirror == "yes"), free)
    except ValueError as exc:
        log("cannot fit: %s" % exc)
        return 1
    report(res, log)
    good = not res["undetermined"] and res["rms_px"] <= MAX_RMS
    if not good:
        log("not written: %s" % (res["why"] or "rms %.2f px over %.1f" % (res["rms_px"], MAX_RMS)))
        return 1
    if a.write:
        cam = res["camera"]
        cam.info.update({"session": os.path.abspath(a.session), "frames": len(frames)})
        cam.save(a.write)
        log("written %s" % a.write)
    return 0


def cmd_capture(a, log=print):
    path = a.session
    session = _load_session(path) if os.path.exists(path) else {"camera_guess": {}, "walls": [], "frames": []}
    folder = os.path.dirname(os.path.abspath(path))
    os.makedirs(folder, exist_ok=True)
    with urllib.request.urlopen(a.url, timeout=3.0) as r:
        data = r.read()
    if _cv2().imdecode(np.frombuffer(data, np.uint8), 1) is None:
        log("the snapshot at %s is no image" % a.url)
        return 1
    name = a.name + ".jpg"
    with open(os.path.join(folder, name), "wb") as f:
        f.write(data)
    x, y, yaw_deg = (float(v) for v in a.pose.split(","))
    fr = {"image": name, "pose": [x, y, yaw_deg]}
    if a.wall:
        nd, dist = (float(v) for v in a.wall.split(","))
        fr["wall"] = {"normal_deg": nd, "dist": dist}
    for e in a.edge or []:
        nm, u = e.split("=")
        fr.setdefault("edges", {})[nm] = {"u": float(u)}
    session["frames"] = [f for f in session.get("frames") or [] if f.get("image") != name] + [fr]
    with open(path, "w") as f:
        json.dump(session, f, indent=2)
    log("saved %s (%d bytes), %d frames in %s" % (name, len(data), len(session["frames"]), path))
    return 0


def cmd_detect(a, log=print):
    cv2 = _cv2()
    img = cv2.imread(a.image)
    if img is None:
        log("cannot read %s" % a.image)
        return 1
    fl = fc.detect_floor_line(img)
    if fl is None:
        log("no wall-floor line found")
    else:
        aa, bb, cc = fl["line"]
        log("wall-floor line from %d columns, rms %.2f px; at u=0 v=%.1f, at u=%d v=%.1f" % (
            fl["count"], fl["rms"], -cc / bb if abs(bb) > 1e-9 else float("nan"), img.shape[1] - 1,
            -(cc + aa * (img.shape[1] - 1)) / bb if abs(bb) > 1e-9 else float("nan")))
    if a.out:
        over = img.copy()
        for u, v in ([] if fl is None else fl["points"]):
            cv2.circle(over, (int(round(u)), int(round(v))), 1, (0, 0, 255), -1)
        cv2.imwrite(a.out, over)
        log("overlay %s" % a.out)
    return 0 if fl is not None else 1


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit")
    f.add_argument("session")
    f.add_argument("--write", help="calibration file to write when every parameter is determined")
    f.add_argument("--free", help="comma list of the parameters to fit (default: the session's, else %s)"
                   % ",".join(fc.FIT_FREE))
    f.add_argument("--mirror", choices=("auto", "yes", "no"), default="auto")
    c = sub.add_parser("capture")
    c.add_argument("--session", required=True)
    c.add_argument("--name", required=True)
    c.add_argument("--pose", required=True, help="x,y,yaw_deg of the robot in the world frame (LiDAR)")
    c.add_argument("--wall", help="normal_deg,dist: the LiDAR wall fit in this frame's robot frame")
    c.add_argument("--edge", action="append", help="NAME=U: a vertical edge near column U")
    c.add_argument("--url", default=fc.SNAP_URL)
    d = sub.add_parser("detect")
    d.add_argument("image")
    d.add_argument("--out")
    return ap.parse_args(argv)


def main(argv=None, log=print):
    a = parse(argv)
    return {"fit": cmd_fit, "capture": cmd_capture, "detect": cmd_detect}[a.cmd](a, log)


if __name__ == "__main__":
    sys.exit(main())
