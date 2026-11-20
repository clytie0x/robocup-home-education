"""Pick the bag in front of the robot with no position typed in: the front camera finds it, the arm picks it.

  1. one frame from the fixed front camera (webcam_view.py /snap.jpg, port 8082)
  2. the bag's centre from that frame with the ARM TEAM'S OWN code (so100_driver.bag_vision.find_bag / foot_y and
     bag_camera.locate, with front_camera.json), so the goal sent is what their camera pick would see itself
     (their pick re-looks and lets the image win when the goal is within GOAL_TOL)
  3. that centre in the robot frame of bag_arm.py (origin under the LiDAR) -> POST /arm pick_bag on agent.py

The centre comes from locate(), so it carries its errors too (2026-11-18: the blob's top edge is taken as the front
wall's rim; from this steep camera it may be the back rim or the handles, putting the bag several cm too far).

  python3 tools/pick_bag_cam.py --dry-run          # where the bag is, nothing moves (writes an overlay picture)
  python3 tools/pick_bag_cam.py                    # find it and pick it
  python3 tools/pick_bag_cam.py --image f.jpg --dry-run   # a saved frame instead of the camera
Refused (nothing moves): no red blob, a blob whose rim is not about 0.18 m wide, a centre outside 0.20-0.50 m ahead.
Options: --top M (handle top above the floor; left out = the arm's own rim height), --home (arm to ready first,
then the frame), --agent URL, --snap URL, --camera FILE, --pan X Y, --half-depth M, --rim-h M.
"""
import argparse
import json
import math
import os
import sys
import urllib.error
import urllib.request

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
ARM_SRC = os.path.expanduser("~/so100_ws/src/so100_driver")   # the arm team's package (bag_camera, bag_vision)

import ports  # noqa: E402

CAMERA_FILE = os.path.join(HERE, "front_camera.json")
SNAP_URL = "http://127.0.0.1:%d/snap.jpg" % ports.WEBCAM
AGENT_URL = "http://127.0.0.1:%d/arm" % ports.AGENT
# the arm team's parameter defaults (pick_place_node.py, 2026-11-18 22:09); --pan etc. override them
ARM_PAN = (0.012, 0.0145)  # arm pan axis from the LiDAR (bag_robot_pan; bag_arm.py Mount.pan_xy)
LASER_X = 0.175            # LiDAR ahead of the turning centre (slam_bridge.LASER_X), front_camera.json's origin
HALF_DEPTH = 0.055         # bag_cam_half_depth
RIM_H = 0.27               # bag_cam_rim_h
BAG_WIDTH = 0.18           # bag_cam_width: the rim must look about this wide, else it is not the bag (or it lies)
WIDTH_TOL = 0.05           # m either way
REACH_X = (0.20, 0.50)     # m, robot frame: a centre outside this is not sent (the deck edge is at 0.20)
PICK_WAIT = 250.0          # s for the HTTP answer (bag_arm PICK_TIMEOUT 150 s plus home)


def arm_modules(src=ARM_SRC):
    """(bag_vision, bag_camera) from the arm team's source tree."""
    if src not in sys.path:
        sys.path.insert(0, src)
    from so100_driver import bag_camera, bag_vision  # noqa: PLC0415
    return bag_vision, bag_camera


def robot_xy(centre, pan=ARM_PAN):
    """locate()'s centre (forward, left from the arm's pan axis) -> bag_arm.py's robot frame (origin under the LiDAR)."""
    return float(centre[0] + pan[0]), float(centre[1] + pan[1])


def find(image, cam, bv, bc, pan=ARM_PAN, half_depth=HALF_DEPTH, rim_h=RIM_H, laser_x=LASER_X,
         width=BAG_WIDTH, width_tol=WIDTH_TOL, reach=REACH_X):
    """One BGR frame -> dict(ok, x, y, from, width, blob) or dict(ok False, why). Refused: no blob, a blob whose
    rim is not about `width` wide (something else red, or a fallen bag), a centre outside `reach`."""
    out = {}
    b = bv.find_bag(image)
    if b is None:
        return dict(out, ok=False, why="no bag in the picture")
    out["blob"] = {k: float(b[k]) for k in ("x0", "x1", "y0", "y1")}
    foot = bv.foot_y(image, b)
    cam_pan = (pan[0] + laser_x, pan[1])          # the pan axis in front_camera.json's frame (turning centre)
    where = bc.locate(cam, b, cam_pan, half_depth, rim_h, foot=foot)
    if where is None:
        return dict(out, ok=False, why="the bag's rays do not reach the floor")
    x, y = robot_xy(where["centre"], pan)
    out.update(x=round(x, 3), y=round(y, 3), width=where.get("width"), foot_row=foot, **{"from": where["from"]})
    w = out["width"]
    if w is None or abs(w - width) > width_tol:
        return dict(out, ok=False, why="the red blob is %s m wide, not a %.2f m bag" %
                    ("?" if w is None else "%.2f" % w, width))
    if not reach[0] <= x <= reach[1]:
        return dict(out, ok=False, why="the bag centre x %.2f m is outside %.2f to %.2f m" % (x, *reach))
    return dict(out, ok=True)


def fetch(url, timeout=3.0):
    import cv2  # noqa: PLC0415
    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = np.frombuffer(r.read(), np.uint8)
    im = cv2.imdecode(data, 1)
    if im is None:
        raise ValueError("not a picture from %s" % url)
    return im


def post(url, body, timeout):
    req = urllib.request.Request(url, json.dumps(body).encode("utf-8"), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:          # agent.py answers a failed arm job with 409 and a JSON body
        try:
            return json.loads(exc.read().decode("utf-8"))
        except ValueError:
            return {"ok": False, "error": "HTTP %d" % exc.code}


def overlay(image, res, path):
    import cv2  # noqa: PLC0415
    im = image.copy()
    if "blob" in res:
        b = res["blob"]
        cv2.rectangle(im, (int(b["x0"]), int(b["y0"])), (int(b["x1"]), int(b["y1"])), (0, 255, 0), 2)
    text = ("bag x %.3f y %+.3f (%s)" % (res["x"], res["y"], res["from"])) if res["ok"] else res["why"][:60]
    cv2.putText(im, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
    cv2.putText(im, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
    cv2.imwrite(path, im)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dry-run", action="store_true", help="only say where the bag is; the arm does not move")
    ap.add_argument("--image", help="a saved frame instead of the camera")
    ap.add_argument("--top", type=float, help="handle top above the floor (m)")
    ap.add_argument("--home", action="store_true", help="arm to ready before the frame")
    ap.add_argument("--overlay", default=os.path.join(HERE, "logs", "pick_bag_cam.jpg"))
    ap.add_argument("--agent", default=AGENT_URL)
    ap.add_argument("--snap", default=SNAP_URL)
    ap.add_argument("--camera", default=CAMERA_FILE)
    ap.add_argument("--pan", type=float, nargs=2, default=ARM_PAN)
    ap.add_argument("--half-depth", type=float, default=HALF_DEPTH)
    ap.add_argument("--rim-h", type=float, default=RIM_H)
    a = ap.parse_args(argv)

    bv, bc = arm_modules()
    cam = bc.FrontCamera.load(a.camera)
    if a.home and not a.dry_run:
        r = post(a.agent, {"cmd": "home"}, 90.0)
        print("home:", json.dumps(r, ensure_ascii=False))
        if not r.get("ok"):
            return 1
    import cv2  # noqa: PLC0415
    image = cv2.imread(a.image) if a.image else fetch(a.snap)
    if image is None:
        print("cannot read", a.image)
        return 1
    res = find(image, cam, bv, bc, tuple(a.pan), a.half_depth, a.rim_h)
    try:
        overlay(image, res, a.overlay)
    except Exception as exc:  # noqa: BLE001 (the picture is only for looking)
        print("overlay not written:", exc)
    print(json.dumps(res, ensure_ascii=False))
    if not res["ok"]:
        print("not picking:", res["why"], "- picture", a.overlay)
        return 1
    print("bag centre x %.3f y %+.3f m (robot frame, origin under the LiDAR), from the %s; picture %s"
          % (res["x"], res["y"], res["from"], a.overlay))
    if a.dry_run:
        return 0
    body = {"cmd": "pick_bag", "x": res["x"], "y": res["y"]}
    if a.top is not None and math.isfinite(a.top):
        body["top"] = a.top
    print("sending", json.dumps(body))
    r = post(a.agent, body, PICK_WAIT)
    print(json.dumps(r, ensure_ascii=False))
    return 0 if r.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
