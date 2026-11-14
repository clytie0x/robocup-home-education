"""Feeds a recorded run_follow log to the SLAM bridge at real time, so the bridge and slam_toolbox can be
checked end to end with the robot standing still. It sends nothing to the wheels and opens no serial port.

    slam_replay.py LOGDIR [--host HOST] [--speed 1.0] [--fill] [--repair]

LOGDIR is ~/hmc-base/logs/<time>/ with log.jsonl (a "pose" per cycle) and scan-NNNNNN.json (the scan of
cycle NNNNNN, saved every 10th cycle). By default one record is sent per saved scan, 1 per second at the
robot's own pace. --fill sends every cycle (10 Hz) with the newest saved scan, only to check /scan and
/odom rates: the scan is then older than its pose, so the map it draws is smeared.
--repair rebuilds the poses of a log recorded before the yaw scale was applied to the increments (see
repaired_poses). The lap-* logs hold no scans and cannot be replayed.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # lidar.py, params.py, slam_link.py
import lidar  # noqa: E402
import params  # noqa: E402
import slam_link  # noqa: E402


def repaired_poses(poses, yaw_scale):
    """Poses in the frame of the first one, with the heading and the motion agreeing again.

    wheels.parse_odom divides the firmware's absolute heading by yaw_scale, but the firmware moved
    x and y along its own unscaled heading. After many turns (the firmware is not zeroed between
    runs, a log starts at 23 rad) the two differ by metres of wrong direction, and slam_toolbox
    is fed a robot that drives backwards. Here each step is turned back into the robot's own frame
    with the unscaled heading and replayed with the scaled yaw increments.
    """
    x, y, heading = 0.0, 0.0, 0.0
    out = [[x, y, heading]]
    for (x0, y0, t0), (x1, y1, t1) in zip(poses, poses[1:]):
        c, s = math.cos(t0 * yaw_scale), math.sin(t0 * yaw_scale)
        forward, left = c * (x1 - x0) + s * (y1 - y0), -s * (x1 - x0) + c * (y1 - y0)
        x += math.cos(heading) * forward - math.sin(heading) * left
        y += math.sin(heading) * forward + math.cos(heading) * left
        heading += t1 - t0
        out.append([x, y, heading])
    return out


def load_plan(folder, fill=False, repair=False):
    """[(seconds from the start, pose, scan)] in log order. Scan n belongs to the n-th cycle record."""
    folder = Path(folder)
    lines = (folder / "log.jsonl").read_text(encoding="utf-8").splitlines()
    cycles = [r for r in (json.loads(line) for line in lines if line.strip()) if "pose" in r]
    if repair:
        for record, pose in zip(cycles, repaired_poses([r["pose"] for r in cycles], params.load()["odom_yaw_scale"])):
            record["pose"] = pose
    scans = {}
    for path in folder.glob("scan-*.json"):
        scan = json.loads(path.read_text(encoding="utf-8"))
        if scan:  # the very first revolution of a run is saved empty
            scans[int(path.stem.split("-")[1])] = lidar.usable([tuple(s) for s in scan])
    if not scans:
        raise ValueError("no scan-*.json in %s: nothing to feed slam_toolbox" % folder)
    plan, newest = [], None
    for n, record in enumerate(cycles):
        fresh = n in scans
        if fresh:
            newest = scans[n]
        if newest is not None and (fresh or fill):
            plan.append((record["time"] - cycles[0]["time"], record["pose"], newest))
    return plan


def run(plan, link, speed=1.0, clock=time.monotonic, sleep=time.sleep):
    """Sends each record at its own time (scaled by speed). Returns how many were sent."""
    start = clock()
    for offset, pose, scan in plan:
        wait = start + offset / speed - clock()
        if wait > 0:
            sleep(wait)
        link.send(clock(), pose, scan)
    return len(plan)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logdir", type=Path)
    parser.add_argument("--host", default=params.load()["jetson_host"])
    parser.add_argument("--port", type=int, default=5602)
    parser.add_argument("--speed", type=float, default=1.0, help="1.0 is real time")
    parser.add_argument("--fill", action="store_true", help="a record every cycle, for rate checks")
    parser.add_argument("--repair", action="store_true", help="fix the poses of an old log (see repaired_poses)")
    args = parser.parse_args(argv)
    plan = load_plan(args.logdir, args.fill, args.repair)
    print("%d records over %.0f s to %s:%d" % (len(plan), plan[-1][0] / args.speed, args.host, args.port), flush=True)
    link = slam_link.SlamLink(args.host, args.port)
    link.start()
    try:
        run(plan, link, args.speed)
        time.sleep(2.0)  # let the last records and the last correction through
    finally:
        link.stop()
    print("sent %d, connected %s, last error %s" % (len(plan), link.connected, link.error))
    print("correction %.3f %.3f %.3f, refused %d" % (*link.correction(), link.refused))


if __name__ == "__main__":
    main()
