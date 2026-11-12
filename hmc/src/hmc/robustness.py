"""Run every map many times with random passers-by and count how often the robot succeeds.

    python -m hmc.robustness --seeds 20 --bystanders 10

A run succeeds when it ends in DONE with all 1115 points and the robot never touched an
obstacle or a person. The table shows, for each map, how many runs succeeded. Failed runs
are listed with the rows they lost, so you can rerun one with
    python -m hmc.run --map NAME --bystanders N --seed S
"""

import argparse
import sys

from hmc.maps import MAPS
from hmc.run import run_scenario
from hmc.scoresheet import MAX_TOTAL
from hmc.task import State
from hmc.world import PERSON_RADIUS, ROBOT_RADIUS, distance

SCORED_ROWS = [
    "follow_to_car", "perceive_bag", "pick_bag", "avoid_crowd", "avoid_small_object",
    "avoid_hard_to_see", "avoid_barriers", "re_reach_start", "queue",
]  # fmt: skip


def check(
    map_name: str,
    bystanders: int,
    seed: int,
    bag: str = "blue",
    noise: str = "off",
    sensors: str = "ideal",
    odometry: str = "perfect",
    base: str = "sim",
    robot_speed: float = 1.0,
    operator_speed: float = 1.2,
) -> list[str]:
    """Run once. Returns the problems found (an empty list means success)."""
    closest = [float("inf")]

    def record(world, task):
        robot = world.robot.pos
        for obstacle in world.obstacles:
            closest[0] = min(closest[0], obstacle.shape.clearance(robot))
        for person in world.people + world.bystanders + [world.operator]:
            closest[0] = min(closest[0], distance(robot, person.pos) - PERSON_RADIUS)

    result = run_scenario(
        map_name=map_name,
        bystanders=bystanders,
        seed=seed,
        target_bag=bag,
        noise=noise,
        sensors=sensors,
        odometry=odometry,
        base=base,
        robot_speed=robot_speed,
        operator_speed=operator_speed,
        on_step=record,
    )
    problems = [
        f"lost {row}" for row in SCORED_ROWS if not result.sheet.counts.get(row)
    ]
    if result.task.state != State.DONE:
        problems.append(f"ended in {result.task.state.value}")
    if closest[0] <= ROBOT_RADIUS:
        problems.append(f"touched something ({closest[0]:.2f} m)")
    if result.sheet.total() != MAX_TOTAL and not problems:
        problems.append(f"scored {result.sheet.total()}")
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Robustness check over all maps")
    parser.add_argument("--seeds", type=int, default=10, help="runs per map")
    parser.add_argument("--bystanders", type=int, default=10, help="passers-by per run")
    parser.add_argument(
        "--noise", default="off", help="sensor noise: off, realistic, hard"
    )
    parser.add_argument(
        "--sensors", default="ideal", help="sensor model: ideal or robot"
    )
    parser.add_argument("--odometry", default="perfect", help="perfect, wheel or slam")
    parser.add_argument(
        "--robot-speed", type=float, default=1.0, help="robot top speed in m/s"
    )
    parser.add_argument(
        "--operator-speed", type=float, default=1.2,
        help="operator's natural walking speed in m/s",
    )  # fmt: skip
    parser.add_argument("--base", default="sim", help="sim or real (measured limits)")
    args = parser.parse_args(argv)

    failures = []
    print(
        f"{'map':<18} {'ok':>4} / {args.seeds}   ({args.bystanders} passers-by, noise {args.noise}, sensors {args.sensors}, odometry {args.odometry}, robot {args.robot_speed} m/s, base {args.base})"
    )
    total_ok = 0
    for map_name in MAPS:
        ok = 0
        for seed in range(args.seeds):
            problems = check(
                map_name, args.bystanders, seed, noise=args.noise, sensors=args.sensors,
                odometry=args.odometry, robot_speed=args.robot_speed, base=args.base,
                operator_speed=args.operator_speed,
            )  # fmt: skip
            if problems:
                failures.append(f"{map_name} seed {seed}: " + ", ".join(problems))
            else:
                ok += 1
        total_ok += ok
        print(f"{map_name:<18} {ok:>4} / {args.seeds}")
    runs = args.seeds * len(MAPS)
    print(f"{'all':<18} {total_ok:>4} / {runs}  ({100 * total_ok / runs:.1f} %)")
    for line in failures:
        print("  failed:", line)
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
