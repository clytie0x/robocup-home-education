"""The realism ladder: add one piece of the real world at a time, and count the full runs.

Every step runs all maps with seeds 0 to N-1 on the real base (its measured limits and
follow distance) at the given top speed. A run counts only if it ends in DONE with all
1115 points and the robot's real outline never touched anything.

    python -m hmc.ladder --seeds 10 --speed 0.5

The runs are independent, so they use every CPU core.
"""

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor

from hmc.maps import MAPS
from hmc.robustness import check

STEPS = {
    "1 real base": dict(),
    "2 +noise": dict(noise="realistic"),
    "3 +passers-by": dict(noise="realistic", bystanders=10),
    "4 +robot sensors": dict(noise="realistic", bystanders=10, sensors="robot"),
    "5 +odometry, scan match": dict(
        noise="realistic", bystanders=10, sensors="robot", odometry="calibrated_slam"
    ),
}


def _one(job):
    step, map_name, seed, speed = job
    options = dict(STEPS[step])
    bystanders = options.pop("bystanders", 0)
    problems = check(
        map_name, bystanders, seed, base="real", robot_speed=speed, **options
    )
    return step, map_name, seed, problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="The realism ladder on the real base")
    parser.add_argument("--seeds", type=int, default=10, help="runs per map and step")
    parser.add_argument("--speed", type=float, default=0.5, help="top speed in m/s")
    args = parser.parse_args(argv)
    jobs = [
        (s, m, seed, args.speed)
        for s in STEPS
        for m in MAPS
        for seed in range(args.seeds)
    ]
    results: dict[str, list] = {s: [] for s in STEPS}
    with ProcessPoolExecutor() as pool:
        for step, map_name, seed, problems in pool.map(_one, jobs):
            results[step].append((map_name, seed, problems))
    print(f"real base, top speed {args.speed} m/s, {args.seeds} seeds per map")
    failed = 0
    for step, rows in results.items():
        ok = sum(1 for *_, p in rows if not p)
        failed += len(rows) - ok
        print(f"{step:<26} {ok:>4} / {len(rows)}  ({100 * ok / len(rows):.0f} %)")
        for map_name, seed, problems in rows:
            if problems:
                print(f"    {map_name} seed {seed}: {', '.join(problems)}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
