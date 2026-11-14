"""Summarise one run_follow.py log.

    replay.py LOGDIR

Prints the share of cycles locked, the longest time without a lock, the closest obstacle
(from the base centre), the stops by reason and the cycle time (cycle start to
wheel send) median and worst. For the way back: the breadcrumbs dropped, the keyframe
matches used and skipped by reason, the largest correction used, the time behind the
queue, and the seconds standing still on the way back (blocked).
"""
import argparse
import json
import statistics
from collections import Counter
from pathlib import Path


def read_log(folder):
    lines = (Path(folder) / "log.jsonl").read_text(encoding="utf-8").splitlines()
    return [json.loads(line) for line in lines if line.strip()]


def summary(records):
    cycles = [r for r in records if "pose" in r]  # speech and end records are not cycles
    longest, unlocked_since = 0.0, None
    for r in cycles:
        if not r["locked"] and unlocked_since is None:
            unlocked_since = r["time"]
        elif r["locked"] and unlocked_since is not None:
            longest = max(longest, r["time"] - unlocked_since)
            unlocked_since = None
    if unlocked_since is not None:  # the run ended without the lock
        longest = max(longest, cycles[-1]["time"] - unlocked_since)
    nearest = [r["nearest"] for r in cycles if r["nearest"] is not None]
    times = [r["cycle_time"] for r in cycles if r["cycle_time"] is not None]
    matches = [r["keyframe"] for r in cycles if r.get("keyframe")]
    crumbs = [r["breadcrumbs"] for r in cycles if r.get("breadcrumbs") is not None]
    return {
        "cycles": len(cycles),
        "locked_share": sum(r["locked"] for r in cycles) / len(cycles) if cycles else 0.0,
        "longest_gap": longest,
        "closest": min(nearest) if nearest else None,
        "stops": dict(Counter(r["stop"] for r in records if r.get("stop"))),
        "cycle_median": statistics.median(times) if times else None,
        "cycle_worst": max(times) if times else None,
        "breadcrumbs": max(crumbs) if crumbs else None,  # a loop cut shortens the route: the most it had
        "keyframes": dict(Counter(m["why"] for m in matches if m["why"] != "far")),  # "far": nothing to match
        "largest_correction": max((m["shift"] for m in matches if m["used"]), default=None),
        "queue_seconds": sum(r.get("period") or 0.0 for r in cycles if r.get("queue") is not None),
        "blocked_seconds": sum(r.get("period") or 0.0 for r in cycles
                               if r.get("phase") == "return" and r["v"] == 0 and r["w"] == 0),
    }


def _number(value, unit):
    return "none" if value is None else "%.3f %s" % (value, unit)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logdir", type=Path)
    s = summary(read_log(parser.parse_args(argv).logdir))
    print("cycles          %d" % s["cycles"])
    print("locked          %.1f %%" % (100 * s["locked_share"]))
    print("longest gap     %.2f s" % s["longest_gap"])
    print("closest         %s (from the base centre)" % _number(s["closest"], "m"))
    print("stops           %s" % (", ".join("%s %d" % kv for kv in sorted(s["stops"].items())) or "none"))
    print("cycle median    %s" % _number(s["cycle_median"], "s"))
    print("cycle worst     %s" % _number(s["cycle_worst"], "s"))
    print("breadcrumbs     %s" % ("none" if s["breadcrumbs"] is None else s["breadcrumbs"]))
    print("keyframes       %s" % (", ".join("%s %d" % kv for kv in sorted(s["keyframes"].items())) or "none"))
    print("largest fix     %s" % _number(s["largest_correction"], "m"))
    print("in the queue    %.1f s" % s["queue_seconds"])
    print("blocked         %.1f s (standing still on the way back)" % s["blocked_seconds"])


if __name__ == "__main__":
    main()
