# Help Me Carry (slice 1)

A small Python simulation of the RoboCup@Home 2025.1 task **5.1 Help Me Carry**.

The robot follows an operator to a car. It does not know where the car is.
The operator points at one of three bags. The robot picks that bag and carries it back to the start.
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

| File | What it holds |
|---|---|
| `score.txt` | the score table |
| `timeline.csv` | one row every 0.5 s and one per event |
| `trajectory.png` | the paths of the robot and the operator |

## Folder map

| Path | What |
|---|---|
| `src/hmc/world.py` | The 2D world: robot, operator, bags, car, clock |
| `src/hmc/perception.py` | What the robot can see: operator, pointing ray, bags |
| `src/hmc/navigation.py` | Follow a person, go to a point |
| `src/hmc/manipulation.py` | Pick and place a bag (a state flag, no real grasp) |
| `src/hmc/hri.py` | What the robot says |
| `src/hmc/task.py` | The state machine that decides what to do next |
| `src/hmc/scoresheet.py` | The scoresheet and the referee that fills it in |
| `src/hmc/plotting.py` | Draws `trajectory.png` |
| `src/hmc/run.py` | The scripted scenario CLI |
| `tests/` | pytest tests |
| `docs/tasks/5.1-help-me-carry.md` | The task spec, the scoresheet and the acceptance tests |
| `docs/reference/` | The rulebook and the other inputs |
| `docs/evidence/help-me-carry-slice1/` | Outputs from a real run, with `SHA256SUMS` |
| `AGENTS.md` | Rules for coding agents |
| `WORKLOG.md` | Decisions and why we made them |

## Limits

- The scripted run scores **615 of 1115**. The robot follows the operator to the car (300), perceives the correct bag (15), picks it (100) and re-reaches the start (200).
- The obstacles, the crowd, the queue, Deus ex Machina and the alternative start signal are not simulated.
- Perception is perfect and comes straight from the world. There is no camera, lidar or noise.
- Picking a bag only sets a flag. There is no arm and no grasp.
- The operator follows one fixed route, so a passing test does not show that the robot can do the real task.
