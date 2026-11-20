"""Measure how far the base coasts after S# at 0.08, 0.12 and 0.15 m/s.

The base moves. Run it only with the owner at the motor switch, the battery charged, and at
least 2 m clear straight ahead. It does not change params.json. Follow stays at max_speed
0.12 until the JSON it writes says the safety gate holds.

  python tools/safe_run.py .venv/bin/python tools/brake_distance.py --yes-owner-at-robot

Each speed is a short straight run (--metres, default 0.40), then S#. The stopping distance
is recorded twice: odometry from the pose at S# to the pose after the settle, and LiDAR as
how much closer the nearest point ahead of the body got over that same interval. The JSON
is logs/brake-distance/<date>.json next to this file's parent (base/pi/logs/...).

Switch the follow cap only when "gate"."ok" is true (follow_speed.SWITCH_HOW):

  params.json  "decel"      = gate.decel
  params.json  "max_speed"  = 0.15

Leave max_speed at 0.12 when gate.ok is false.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import base_check  # noqa: E402
import follow_speed  # noqa: E402
import params as params_module  # noqa: E402
import wheels as wheels_module  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "logs" / "brake-distance"
METRES = 0.40  # m of driving before S#, enough to be at the asked speed, short enough to stay in the clear strip
CLEAR_AHEAD = 2.0  # m the owner is told to keep clear; the strip check uses metres + base_check.LINE_MARGIN


def ahead_min(scan, p):
    """Nearest distance ahead of the body (m), or None when that strip is empty."""
    near = base_check.ahead(scan, p)
    return float(near.min()) if len(near) else None


def stops_from_log(log, p):
    """Stopping distances of one run: odometry, and LiDAR (the wall ahead got this much closer)."""
    stop = log["stop_time"]
    odom = log["odom"]
    at = max((row for row in odom if row[0] <= stop), key=lambda row: row[0])
    end = odom[-1]
    odom_stop = math.hypot(end[1] - at[1], end[2] - at[2])
    before = [row for row in log["scans"] if row[0] <= stop]
    lidar_stop = None
    if before and log["scans"]:
        near_before = ahead_min(before[-1][1], p)
        near_after = ahead_min(log["scans"][-1][1], p)
        if near_before is not None and near_after is not None:
            lidar_stop = near_before - near_after
    return {"speed": log["rate"], "odom_stop_m": odom_stop, "lidar_stop_m": lidar_stop,
            "stop_reason": log.get("stop_reason")}


def run_params(p):
    """A copy whose max_speed allows the 0.15 m/s run. params.json is not written."""
    copy = dict(p)
    copy["max_speed"] = max(float(p["max_speed"]), follow_speed.NEXT_SPEED)
    return copy


def build_report(runs, committed, metres):
    gate = follow_speed.recommendation(runs, committed)
    return {
        "metres": metres,
        "speeds_m_s": list(follow_speed.RUN_SPEEDS),
        "runs": runs,
        "follow_max_speed_now": follow_speed.follow_max_speed(committed),
        "gate": gate,
        "switch": follow_speed.SWITCH_HOW,
    }


def measure_run(wheels, session, p, speed, metres, clock=time.monotonic, sleep=time.sleep):
    """One straight run at `speed`, then the settle. Returns stops_from_log. S# however it ends."""
    log = {"mode": "line", "rate": speed, "odom": [], "scans": []}
    scan = base_check.first_full_scan(session, log, clock, sleep)
    ok, nearest = base_check.clear_to_drive(scan, p, "line", metres)
    if not ok:
        where = "none" if nearest is None else "%.2f m" % nearest
        raise RuntimeError("not clear for %.2f m/s (nearest ahead %s)" % (speed, where))
    base_check.drive(wheels, session, p, "line", speed, metres, log, clock, sleep)
    base_check.settle(wheels, session, log, clock, sleep)
    return stops_from_log(log, p)


def refuse(owner, safe):
    """Why the tool must not move, or None."""
    if not owner:
        return "REFUSED: the base moves. Pass --yes-owner-at-robot only when the owner is at the robot."
    if not safe:
        return "REFUSED: " + wheels_module.NOT_SAFE_RUN
    return None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yes-owner-at-robot", action="store_true",
                        help="the owner is at the robot, at the motor switch, with the area clear")
    parser.add_argument("--metres", type=float, default=METRES, help="odometry distance before S# at each speed")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="folder for the JSON report")
    args = parser.parse_args(argv)
    why = refuse(args.yes_owner_at_robot, wheels_module.under_safe_run())
    if why:
        sys.exit(why)
    if not 0.2 <= args.metres <= 1.0:
        sys.exit("REFUSED: --metres %.2f is outside 0.20 .. 1.00" % args.metres)
    committed = params_module.load()
    p = run_params(committed)
    import lidar  # noqa: PLC0415 (the port is opened only after the refusals)
    session = lidar.LidarSession()
    runs = []
    wheels = None
    try:
        wheels = wheels_module.Wheels(p)
        for speed in follow_speed.RUN_SPEEDS:
            print("brake run at %.2f m/s for %.2f m, then S#" % (speed, args.metres), flush=True)
            runs.append(measure_run(wheels, session, p, speed, args.metres))
            print("  odometry %.3f m, lidar %s" % (
                runs[-1]["odom_stop_m"],
                "n/a" if runs[-1]["lidar_stop_m"] is None else "%.3f m" % runs[-1]["lidar_stop_m"]), flush=True)
    finally:
        if wheels is not None:
            with base_check.signals_ignored():
                wheels.close()
        session.close()
    report = build_report(runs, committed, args.metres)
    args.out.mkdir(parents=True, exist_ok=True)
    path = args.out / (time.strftime("%Y%m%d-%H%M%S") + "-brake.json")
    path.write_text(json.dumps(report, indent=2) + "\n")
    gate = report["gate"]
    print("wrote %s" % path)
    print("gate ok=%s decel=%s  %s" % (gate["ok"], gate["decel"], gate["why"]))
    print(follow_speed.SWITCH_HOW)
    return 0 if gate["ok"] else 2


if __name__ == "__main__":
    sys.exit(main())
