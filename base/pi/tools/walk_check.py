"""Task 5 step 3: does the LiDAR track hold one person walking round the standing robot?

  python tools/walk_check.py --seconds 60

The wheel port is never opened. The owner stands alone 0.4 to 1.6 m in front of the robot;
the tool locks onto them and then they walk loops round it. The Jetson is not used, so this
is the LiDAR alone with the identity rules of senses.py, as when the link is down. After a
drop the tool locks again on the one candidate near the last fix, so one run counts every
drop. Every scan and the track go to logs/walk-check/, so a scan that lost the track can
become a test fixture.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # lidar.py, senses.py, ...
import base_check  # noqa: E402
import geometry  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402
from legs import candidates  # noqa: E402
from senses import RealSenses  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "logs" / "walk-check"
LOCK_X, LOCK_Y = (0.4, 1.6), 0.5  # m, base frame: where the owner stands to be locked
RELOCK = 0.5  # m; after a drop, lock again on the one candidate this close to the last fix


def lock_on(senses, now, scan, p, near=None):
    """Lock onto the one candidate in the lock box, or the one within RELOCK of `near`."""
    cands = candidates(scan)
    if near is None:
        pick = [c for c in cands if LOCK_X[0] <= geometry.lidar_to_base(c, p)[0] <= LOCK_X[1]
                and abs(c[1]) <= LOCK_Y]
    else:
        pick = [c for c in cands if math.dist(c, near) <= RELOCK]
    if len(pick) != 1:
        return False
    senses.tracker = senses._new_tracker()  # as RealSenses.enrol does, without a camera look
    senses.tracker.update(scan, now, lock_at=pick[0])
    senses._last_now = now
    return True


def walk(scans, p):
    """scans: (time, scan) pairs; the first one that locks starts the count. Returns (records, summary)."""
    senses = RealSenses(p)
    records, last, last_fix, started = [], None, None, False
    for now, scan in scans:
        if not started:
            started = lock_on(senses, now, scan, p)
            last = now
            continue
        got = senses.read(now, (0.0, 0.0, 0.0), 0.0, 0.0, now - last, scan, None, None, False, lambda look: 0.0)
        last = now
        rec = {"t": round(now, 3), "held": got.operator is not None, "locked": senses.locked,
               "ambiguous": senses.ambiguous, "candidates": len(candidates(scan)),
               "track": None if senses.tracker.pos is None else [round(v, 3) for v in senses.tracker.pos]}
        if senses.tracker.pos is not None and rec["held"]:
            last_fix = senses.tracker.pos
        if not senses.locked:
            rec["relocked"] = lock_on(senses, now, scan, p, near=last_fix)
        records.append(rec)
    n = len(records)
    drops = sum(1 for a, b in zip(records, records[1:]) if a["locked"] and not b["locked"])
    drops += bool(records) and not records[0]["locked"]
    summary = {"scans": n, "held": sum(r["held"] for r in records), "locked": sum(r["locked"] for r in records),
               "ambiguous": sum(r["ambiguous"] for r in records), "drops": drops,
               "relocks": sum(r.get("relocked", False) for r in records)}
    summary["held_share"] = round(summary["held"] / n, 3) if n else 0.0
    return records, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to log after the lock")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="folder for the JSON log")
    args = parser.parse_args()
    p = params.load()
    session, log = lidar.LidarSession(), {"scans": []}
    try:
        base_check.first_full_scan(session, log, time.monotonic, time.sleep)
        print("stand alone 0.4 to 1.6 m in front of the robot", flush=True)
        probe, deadline = RealSenses(p), time.monotonic() + 30.0
        while True:  # lock live, so a failed lock costs no walk
            scan = base_check.wait_scan(session, log, time.monotonic, time.sleep)
            if lock_on(probe, log["scans"][-1][0], scan, p):
                break
            if time.monotonic() > deadline:
                sys.exit("REFUSED: nobody alone 0.4 to 1.6 m in front within 30 s")
        collected = [(log["scans"][-1][0], scan)]
        print("LOCKED: walk round the robot now, %.0f s" % args.seconds, flush=True)
        while collected[-1][0] - collected[0][0] < args.seconds:
            scan = base_check.wait_scan(session, log, time.monotonic, time.sleep)
            collected.append((log["scans"][-1][0], scan))
    finally:
        session.close()
    records, summary = walk(collected, p)
    args.out.mkdir(parents=True, exist_ok=True)
    path = args.out / time.strftime("%Y%m%d-%H%M%S-walk.json")
    path.write_text(json.dumps({"summary": summary, "records": records, "scans": collected}))
    print(json.dumps(summary))
    print("wrote", path)


if __name__ == "__main__":
    main()
