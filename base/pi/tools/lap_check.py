"""Drive one lap round the room keeping the wall on the right; map it; log every gap passed.

  python tools/safe_run.py .venv/bin/python tools/lap_check.py --yes-floor-run [--seconds 300]

The base drives on the floor round the whole room. Only run it with the owner at the robot.
Safety every cycle: the wanted motion must keep the real outline (footprint.sweep_clear) clear
along 0.9 s of it and pass safety.must_stop, else the safe escape that leaves the most room is
taken, else hold; stall check (asked to drive, odometry did not move); time limit. The lap ends
back within END_NEAR of the start after at least MIN_TRAVEL of driving and once AWAY from it.
Writes logs/lap-<time>/ next to tools/: log.jsonl, map.png, summary.json (with the closest body
gap, footprint.body_gap). Prints a progress line every 5 s.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # lidar.py, safety.py, ...
import base_check  # noqa: E402
import camera  # noqa: E402
import depth_obstacles  # noqa: E402
import footprint  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402
import safety  # noqa: E402
import scanmatch  # noqa: E402
import wheels  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "logs"
HALF = 0.20  # m, half the 0.40 m body: the side strips that steer (zones), not the safety check
WALL_GAP = 0.25  # m, wanted gap between the robot's right side and the wall
TOP, TURN = 0.12, 0.45
END_NEAR, AWAY, MIN_TRAVEL = 0.7, 2.5, 6.0
STALL_S, STALL_M = 2.0, 0.03
CELL, EXT = 0.05, 8.0
HORIZON = 0.9  # s simulated ahead (move plus run-on after S#)
ESCAPES = [(0.06, 0.0), (0.06, 0.25), (0.06, -0.25), (0.0, 0.35), (0.0, -0.35), (-0.05, 0.0), (-0.05, 0.25), (-0.05, -0.25)]


def zones(P):
    """Gaps (m) from the body: ahead of the front edge, beside the right and left sides."""
    def gap(sel, val):
        return float(val[sel].min()) if sel.any() else math.inf
    x, y = P[:, 0], P[:, 1]
    front = gap((x > HALF) & (np.abs(y) < HALF + 0.05), x - HALF)
    right = gap((x > -HALF - 0.05) & (x < HALF + 0.15) & (y < -HALF), -y - HALF)
    left = gap((x > -HALF - 0.05) & (x < HALF + 0.15) & (y > HALF), y - HALF)
    return front, right, left


def control(front, right, left):
    """Right-hand wall following. Returns (v, w), v >= 0, w positive = left."""
    if front < 0.35:
        return 0.0, TURN  # blocked ahead: turn left on the spot
    if right > 0.8:
        return (0.08, -0.35) if front > 0.6 else (0.0, TURN)  # lost the wall: curve right to find it
    v = min(TOP, 0.03 + TOP * (front - 0.35) / 0.5)
    w = max(-0.4, min(0.4, -1.5 * (right - WALL_GAP)))
    if left < 0.10:
        w -= 0.3  # too close on the left: steer away
    return max(0.0, v), max(-0.49, min(0.49, w))


def closest(P, x=0.0, y=0.0, th=0.0):
    """Smallest gap (m) from the real outline at pose (x, y, th) to any point; inf with no points."""
    return float(footprint.body_gap(P, x, y, th).min()) if len(P) else math.inf


def arc(v, w, horizon=HORIZON, dt=0.1):
    """Poses (x, y, th) every dt while (v, w) runs for horizon seconds from (0, 0, 0)."""
    poses, x, y, th = [], 0.0, 0.0, 0.0
    for _ in range(int(horizon / dt)):
        th += w * dt
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        poses.append((x, y, th))
    return poses


def sweep_ok(P, v, w, p):
    """(safe, body gap at the end): the outline along the arc stays clear, and forward passes must_stop."""
    poses = arc(v, w)
    if not footprint.sweep_clear(P, poses) or (v >= 0 and safety.must_stop(P, v, w, p)):
        return False, 0.0
    return True, closest(P, *poses[-1])


def choose(P, v, w, p, allow_back=True):
    """The wanted motion if it is safe, else the safe escape that leaves the most room, else hold."""
    if sweep_ok(P, v, w, p)[0]:
        return v, w, None
    best = None
    for ev, ew in ESCAPES:
        if ev < 0 and not allow_back:
            continue  # backed 0.3 m already in this escape
        ok, gap = sweep_ok(P, ev, ew, p)
        if ok:
            score = gap + (0.02 if ev > 0 else 0.0)  # prefer going on over backing off
            if best is None or score > best[0]:
                best = (score, ev, ew)
    if best is None:
        return 0.0, 0.0, "hold: no safe move"
    return best[1], best[2], "escape v %+.2f w %+.2f (gap after %.2f m)" % (best[1], best[2], best[0])


def send(w, v, wr, p):
    """Like Wheels.send, but a short backward escape is allowed (B packet, raw like air_check)."""
    if v >= 0:
        return w.send(v, wr)
    wr = max(-p["max_turn_rate"], min(p["max_turn_rate"], wr)) * p["angular_cmd_scale"]
    pk = ("B%03d|%s%03d#" % (int(abs(v) * 100 + 1e-9), "+" if wr >= 0 else "-", int(abs(wr) * 100 + 1e-9))).encode()
    with w.lock:
        w.last, w.sent_at = pk, w.clock()
        w.port.write(pk)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--yes-floor-run", action="store_true", help="the base drives round the room; the owner is at the robot")
    args = ap.parse_args()
    if not args.yes_floor_run:
        sys.exit("REFUSED: the base drives round the room. Pass --yes-floor-run only when the owner is at the robot.")
    if not wheels.under_safe_run():
        sys.exit("REFUSED: " + wheels.NOT_SAFE_RUN)

    p = params.load()
    out = DEFAULT_OUT / time.strftime("lap-%Y%m%d-%H%M%S"); out.mkdir(parents=True)
    logf = open(out / "log.jsonl", "w")
    n = int(2 * EXT / CELL); hits = np.zeros((n, n), np.uint16); free = np.zeros((n, n), np.uint16)
    ij = lambda X, Y: (np.clip(((X + EXT) / CELL).astype(int), 0, n - 1), np.clip(((Y + EXT) / CELL).astype(int), 0, n - 1))
    session = lidar.LidarSession(); log = {"scans": []}; cam = camera.Camera(); cam.start(); w = None
    end, traj, gaps, travelled, been_away, k = "time limit", [], [], 0.0, False, 0
    nearest = (math.inf, None, None, None)  # (closest body gap m, t, x, y)
    t0 = time.monotonic()
    try:
        scan = base_check.first_full_scan(session, log, time.monotonic, time.sleep)
        w = wheels.Wheels(p)
        x0, y0, th0 = w.read_odom(); start = (x0, y0); t0 = time.monotonic(); last_print = 0.0
        moving_since, moved_from, prev = None, None, (x0, y0)
        backed, held = 0.0, 0.0
        while time.monotonic() - t0 < args.seconds:
            now = time.monotonic() - t0
            scan = session.latest() or scan
            if not scan:
                w.stop(); end = "no scan"; break
            P = scanmatch.points(scan, p)
            f = cam.latest()
            dp = depth_obstacles.points(np.asarray(f.depth), p) if f is not None else None
            allp = P if dp is None or not len(dp) else np.vstack([P, dp])
            front, right, left = zones(allp)
            body = closest(allp)
            v, wr = control(front, right, left)
            v, wr, note = choose(allp, v, wr, p, allow_back=backed < 0.3)
            backed = backed + 0.005 if v < 0 else (0.0 if v > 0 else backed)
            if note and note.startswith("hold"):
                held += 0.1
                if held > 5.0:
                    w.stop(); end = "stuck: no safe move for 5 s"; break
            else:
                held = 0.0
            send(w, v, wr, p)
            x, y, th = w.read_odom()
            step = math.dist((x, y), prev); travelled += step; prev = (x, y)
            far = math.dist((x, y), start)
            been_away = been_away or far > AWAY
            if body < nearest[0]:
                nearest = (round(body, 3), round(now, 1), round(x, 2), round(y, 2))
            # stall: asked to drive for STALL_S, odometry did not move STALL_M
            if v > 0.05:
                if moving_since is None:
                    moving_since, moved_from = now, (x, y)
                elif now - moving_since > STALL_S and math.dist((x, y), moved_from) < STALL_M:
                    w.stop(); end = "stalled: asked to drive, the wheels did not move"; break
            else:
                moving_since = None
            if v > 0.05 and right < 0.5 and left < 0.5:
                gaps.append((round(right + left + 2 * HALF, 3), round(x, 2), round(y, 2), round(now, 1)))
            traj.append((x, y))
            if k % 3 == 0:  # map: hits and free space along the rays, in the odometry frame
                c, s = math.cos(th), math.sin(th)
                X = x + P[:, 0] * c - P[:, 1] * s; Y = y + P[:, 0] * s + P[:, 1] * c
                hi, hj = ij(X, Y); np.add.at(hits, (hi, hj), 1)
                for t in (0.25, 0.5, 0.75):
                    fi, fj = ij(x + (X - x) * t, y + (Y - y) * t); np.add.at(free, (fi, fj), 1)
            logf.write(json.dumps({"t": round(now, 2), "pose": [round(x, 3), round(y, 3), round(th, 3)], "v": v, "w": round(wr, 3),
                                   "front": round(front, 3), "right": round(right, 3), "left": round(left, 3),
                                   "body_gap": round(body, 3) if math.isfinite(body) else None,
                                   "note": note, "far": round(far, 2)}) + "\n")
            if now - last_print > 5.0:
                print("%5.1f s travelled %.1f m, %.1f m from start, front %.2f right %.2f left %.2f body gap %.2f %s"
                      % (now, travelled, far, front, right, left, body, note or ""), flush=True)
                last_print = now
            if been_away and travelled > MIN_TRAVEL and far < END_NEAR:
                w.stop(); end = "lap done: back at the start"; break
            k += 1
            time.sleep(max(0.0, 0.1 - ((time.monotonic() - t0) - now)))
    finally:
        if w is not None:
            w.stop(); w.close()
        cam.stop(); session.close(); logf.close()
    img = np.full((n, n, 3), 170, np.uint8); img[free > 0] = (255, 255, 255); img[hits > 1] = (0, 0, 0)
    for (ax, ay), (bx, by) in zip(traj, traj[1:]):
        a, b = ij(np.array([ax]), np.array([ay])), ij(np.array([bx]), np.array([by]))
        cv2.line(img, (int(a[1][0]), int(a[0][0])), (int(b[1][0]), int(b[0][0])), (0, 160, 0), 1)
    narrow = sorted(gaps)[:5]
    for g in narrow:
        a = ij(np.array([g[1]]), np.array([g[2]])); cv2.circle(img, (int(a[1][0]), int(a[0][0])), 4, (0, 0, 255), 1)
    ys, xs = np.where(img.min(axis=2) < 255); crop = img[max(0, ys.min() - 10):ys.max() + 10, max(0, xs.min() - 10):xs.max() + 10] if len(ys) else img
    cv2.imwrite(str(out / "map.png"), cv2.resize(cv2.flip(crop, 0), None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    summary = {"end": end, "travelled_m": round(travelled, 2), "seconds": round(time.monotonic() - t0, 1),
               "narrowest_gaps_passed": narrow,
               "closest_body_gap": dict(zip(("m", "t", "x", "y"), nearest)) if nearest[1] is not None else None,
               "folder": str(out)}
    (out / "summary.json").write_text(json.dumps(summary, indent=1)); print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
