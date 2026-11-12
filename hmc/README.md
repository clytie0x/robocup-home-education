# Help Me Carry

A small Python simulation of the RoboCup@Home 2025.1 task **5.1 Help Me Carry**.

The robot follows an operator to a car. It does not know where the car is.
On the way it goes around a small object, a hard-to-see object, a crowd and retractable barriers.
The operator points at one of three bags. The robot picks that bag, joins the queue of people walking back, and carries the bag to the start.
Everything happens in a 2D simulation with a 5:00 time limit, and the scoresheet is filled in as the robot works.

It uses plain Python and the standard library. matplotlib is optional and only draws the plot.
There is no ROS and no hardware yet. Each module has a small interface, so it can be wrapped later as a ROS 2 Jazzy node.

## Run it on Windows (PowerShell)

From the repo root:

```powershell
cd hmc
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -e ".[dev,plot]"
pytest
python -m hmc.run
```

If `Activate.ps1` is blocked, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once.

## Run it on Ubuntu

From the repo root:

```bash
cd hmc
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev,plot]"
pytest
python -m hmc.run
```

`python -m hmc.run` prints the score table and writes these files to `out/`. Use `--out` to pick another folder and `--bag red|blue|green` to pick the bag.

Use `--map` to pick the scenario:

| Map | What it is |
|---|---|
| `default` | A small made-up scenario, about 14 m by 9 m |
| `bangkok2022` | Official Carry My Luggage map, RoboCup 2022 Bangkok. The robot starts inside the flat and leaves through the exit door |
| `bordeaux2023` | Official map, RoboCup 2023 Bordeaux. The robot starts outside, in the entering area |
| `eindhoven2024` | Official map, RoboCup 2024 Eindhoven: team tables, arenas A to C, audience stands and starting point A, about 40 m by 27 m |
| `germanopen2024` | Official map, German Open 2024 Kassel (a 10 m flat). The robot starts in the living room and leaves through the exit at the top |
| `portugalopen2024` | Official map, Portugal Open 2024. The robot starts in the office and leaves through the exit door |

The official maps come from github.com/RoboCupAtHome. They show the flat and the starting point, never the car or the outside, because the rulebook keeps the car location secret. So the scale (only German Open 2024 gives one), the outside part, the car and the obstacles are our own choice. Outside the exit we reuse the course of the `default` map. German Open 2025 is left out, because its official map is a placeholder. See `src/hmc/maps.py`.

The robot is never given a map. It plans only from what it can see, and walls block its view. So inside the flat and outside it works the same way.

| File | What it holds |
|---|---|
| `score.txt` | the score table |
| `timeline.csv` | one row every 0.5 s and one per event |
| `trajectory.png` | the paths of the robot and the operator, the obstacles and the crowd |

## Harder runs

The rulebook allows other people to walk around during the whole test, and real sensors make mistakes. Try:

```bash
python -m hmc.run --map eindhoven2024 --bystanders 10 --noise realistic --seed 3
python -m hmc.robustness --seeds 20 --bystanders 10 --noise realistic
```

`--bystanders N` adds N passers-by. `--noise off|realistic|hard` adds sensor noise. `--seed` changes both. `hmc.robustness` runs every map with seeds 0 to N-1 and prints how many runs ended in DONE with all 1115 points, without touching anything.

## Folder map

| Path | What |
|---|---|
| `src/hmc/world.py` | The 2D world: robot, operator, bags, car, obstacles, crowd and queue, clock |
| `src/hmc/perception.py` | What the robot can see: operator, pointing ray, bags, obstacle shapes, people |
| `src/hmc/navigation.py` | Follow a person or go to a point, planning around obstacles with A* |
| `src/hmc/manipulation.py` | Pick and place a bag (a state flag, no real grasp) |
| `src/hmc/hri.py` | What the robot says |
| `src/hmc/task.py` | The state machine that decides what to do next |
| `src/hmc/scoresheet.py` | The scoresheet and the referee that fills it in |
| `src/hmc/plotting.py` | Draws `trajectory.png` |
| `src/hmc/maps.py` | The scenario maps you can pick with `--map` |
| `src/hmc/bystanders.py` | Passers-by who walk around outside (`--bystanders N`) |
| `src/hmc/noise.py` | Sensor noise: misses, position error, id swaps (`--noise`) |
| `src/hmc/appearance.py` | Looks for re-identification (re-ID) of people |
| `src/hmc/robustness.py` | Runs every map many times and counts the successes |
| `src/hmc/run.py` | The scripted scenario CLI |
| `tests/` | pytest tests |
| `docs/tasks/5.1-help-me-carry.md` | The task spec, the scoresheet and the acceptance tests |
| `docs/reference/` | The rulebook and the other inputs |
| `docs/evidence/help-me-carry-slice1/` | Outputs of slice 1 (615 of 1115), with `SHA256SUMS` |
| `docs/evidence/help-me-carry-slice2/` | Outputs of slice 2 (1115 of 1115) for every map, with `SHA256SUMS` |
| `AGENTS.md` | Rules for coding agents |
| `WORKLOG.md` | Decisions and why we made them |

## Limits

- The scripted run scores **1115 of 1115** on all six maps, for each of the three bags. That is every main and bonus row. The world, the obstacles and the referee are all our own code, so this shows the logic works in our scenario. It does not show that the real robot can do the task.
- Deus ex Machina, the alternative start signal, restarts and the three tries are not simulated. Not attending and outstanding performance are referee decisions.
- The robot drives at up to 1.0 m/s in the simulation. The real base is limited to 0.15 m/s, which is too slow to follow a person walking at 0.8 m/s.
- By default perception is perfect, except that walls block the view. `--noise realistic` or `--noise hard` adds misses, position errors, pointing errors, tracker id swaps and wrong operator detections. The noise levels are our guess, not measurements from a real camera or lidar.
- Re-identification is simulated with a random "look" per person. A real robot needs a re-ID model for this, and real people can look much more alike.
- The walls of the official maps are traced by eye, and most furniture is left out.
- Picking a bag only sets a flag. There is no arm and no grasp.
- The operator follows one fixed route, so a passing test does not show that the robot can do the real task.
