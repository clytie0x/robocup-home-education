"""Scripted scenario. Run it with:  python -m hmc.run --out out"""

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from hmc import hri, plotting
from hmc.appearance import give_looks
from hmc.bystanders import add_bystanders
from hmc.maps import MAPS
from hmc.noise import PRESETS, make_noise
from hmc.odometry import PRESETS as ODOMETRY, make_odometry
from hmc.scoresheet import MAX_TOTAL, Referee, Scoresheet
from hmc.task import State, Task
from hmc.world import DT, World

LOG_EVERY = 0.5  # seconds between timeline rows (events always get a row)

COLUMNS = [
    "time", "state", "robot_x", "robot_y", "robot_heading",
    "operator_x", "operator_y", "carrying", "score", "event",
]  # fmt: skip


@dataclass
class Result:
    world: World
    task: Task
    sheet: Scoresheet
    speech: hri.Speech
    rows: list[dict]


def run_scenario(
    target_bag: str = "blue",
    robot_speed: float = 1.0,
    on_step: Callable[[World, Task], None] | None = None,
    map_name: str = "default",
    bystanders: int = 0,
    seed: int = 0,
    noise: str = "off",
    sensors: str = "ideal",
    odometry: str = "perfect",
    operator_speed: float = 1.2,
) -> Result:
    """Run the whole scenario. on_step lets tests poke the world (e.g. drop the bag)."""
    world = MAPS[map_name](target_bag=target_bag, robot_speed=robot_speed)
    add_bystanders(world, bystanders, seed)
    give_looks(world, seed)
    world.noise = make_noise(noise, seed)
    world.sensors = sensors
    world.odometry = make_odometry(odometry, seed)
    if world.odometry is not None:
        world.odometry.start(world.robot)
    world.operator.speed = operator_speed  # the natural walking speed
    speech = hri.Speech()
    task = Task(world, speech)
    sheet = Scoresheet()
    referee = Referee(sheet)
    rows: list[dict] = []
    seen_messages = 0
    heard_messages = 0  # what the operator has heard so far
    last_state = task.state
    last_logged = -LOG_EVERY

    while not task.finished:
        if on_step:
            on_step(world, task)
        task.step(DT)
        if hri.READY_TEXT in [text for _, text in speech.messages]:
            world.operator.start_walking()  # the operator waits for "ready"
        # The operator follows the robot's instructions (rulebook 5.1).
        for _, text in speech.messages[heard_messages:]:
            pace = hri.asked_pace(text)
            if pace is not None:  # walk slowly, but never faster than we normally do
                world.operator.speed = min(world.operator.speed, pace)
            if text == hri.WAIT_TEXT:
                world.operator.waiting = True
            if text == hri.GO_ON_TEXT:
                world.operator.waiting = False
        heard_messages = len(speech.messages)
        # Timing out does nothing, so it takes no sim time.
        if task.state != State.TIMEOUT:
            world.step(DT)
        referee.update(world, task.perceived_bag)

        events = []
        if task.state != last_state:
            events.append(f"state {task.state.value}")
            last_state = task.state
        for _, text in speech.messages[seen_messages:]:
            events.append(f"say: {text}")
        seen_messages = len(speech.messages)

        if events or world.time - last_logged >= LOG_EVERY - 1e-9:
            last_logged = world.time
            rows.append(
                {
                    "time": f"{world.time:.1f}",
                    "state": task.state.value,
                    "robot_x": f"{world.robot.x:.3f}",
                    "robot_y": f"{world.robot.y:.3f}",
                    "robot_heading": f"{world.robot.heading:.3f}",
                    "operator_x": f"{world.operator.x:.3f}",
                    "operator_y": f"{world.operator.y:.3f}",
                    "carrying": world.carried_bag().name if world.carried_bag() else "",
                    "score": sheet.total(),
                    "event": "; ".join(events),
                }
            )
    return Result(world, task, sheet, speech, rows)


def write_outputs(result: Result, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "timeline.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(result.rows)
    summary = f"{result.sheet.table()}\nSim time: {result.world.time:.1f} s\nEnd state: {result.task.state.value}\n"
    (out / "score.txt").write_text(summary, encoding="utf-8")
    if not plotting.save_trajectory(
        out / "trajectory.png", result.world, [_floats(r) for r in result.rows]
    ):
        print(
            "matplotlib not installed, skipping trajectory.png (pip install -e .[plot])"
        )


def _floats(row: dict) -> dict:
    return {
        k: float(v) if k.startswith(("robot_", "operator_")) else v
        for k, v in row.items()
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Help Me Carry slice 1 scenario")
    parser.add_argument("--out", default="out", help="output folder (default: out)")
    parser.add_argument(
        "--map", default="default", choices=sorted(MAPS), help="scenario map"
    )
    parser.add_argument("--bystanders", type=int, default=0, help="passers-by outside")
    parser.add_argument(
        "--seed", type=int, default=0, help="seed for passers-by and noise"
    )
    parser.add_argument(
        "--noise", default="off", choices=list(PRESETS), help="sensor noise"
    )
    parser.add_argument(
        "--odometry", default="perfect", choices=list(ODOMETRY), help="pose estimate"
    )
    parser.add_argument(
        "--sensors", default="ideal", choices=["ideal", "robot"], help="sensor model"
    )
    parser.add_argument(
        "--robot-speed", type=float, default=1.0, help="robot top speed in m/s"
    )
    parser.add_argument(
        "--operator-speed", type=float, default=1.2,
        help="operator's natural walking speed in m/s",
    )  # fmt: skip
    parser.add_argument("--bag", default="blue", choices=["red", "blue", "green"],
                        help="bag the operator points at")  # fmt: skip
    args = parser.parse_args(argv)

    result = run_scenario(
        target_bag=args.bag,
        robot_speed=args.robot_speed,
        operator_speed=args.operator_speed,
        map_name=args.map,
        bystanders=args.bystanders,
        seed=args.seed,
        noise=args.noise,
        sensors=args.sensors,
        odometry=args.odometry,
    )
    write_outputs(result, Path(args.out))
    print(result.sheet.table())
    print(f"Awarded {result.sheet.total()} of {MAX_TOTAL}")
    print(
        f"Sim time: {result.world.time:.1f} s (limit 300 s), end state: {result.task.state.value}"
    )
    return 0 if result.task.state == State.DONE else 1


if __name__ == "__main__":
    sys.exit(main())
