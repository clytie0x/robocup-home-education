# Tier-1 bag grasp, in simulation

One grasp decision per try. The policy picks six numbers. A scripted executor, the same order as `bag_camera.CameraBag`, moves the arm. There are no images in the policy.

Install from `sim/` (CPU PyTorch, not the default CUDA wheel):

```bash
pip install mujoco numpy pytest
pip install torch --index-url https://download.pytorch.org/whl/cpu
```

Run the tests:

```bash
cd sim
python -m pytest -q
```

Evaluate the scripted pick (today's offsets: 0, +1.5 cm, -1.5 cm at -60 deg, then +3 cm at -50 deg):

```bash
python -m rl_grasp evaluate --policy scripted --episodes 8 --seed 0 --tries 4 --nominal
```

Train. `--steps` is the number of grasp tries, not physics steps. `--nominal` keeps one paper bag in one place. Without it, pose, mass, friction, actuator gains, handle stiffness, floor height and the fisheye camera are randomised. `--workers` is CPU processes for PPO. SAC updates in this process.

```bash
python -m rl_grasp train --algo ppo --steps 200 --workers 4 --seed 0 --nominal --out checkpoints/ppo.pt
python -m rl_grasp evaluate --policy checkpoints/ppo.pt --episodes 8 --seed 0 --tries 1 --nominal
```

`--algo sac` is the same command with a SAC checkpoint.

The report is six lines: `policy`, `episodes`, `success_rate`, `median_time_s` (successes only, or `nan`), `collision_count` (episodes that hit the deck, the floor, or the bag body), `successes`.

## What the policy sees and does

Observation, 24 numbers, in the arm's user frame (forward, left, height above the floor):

- six joint positions and six joint velocities
- jaw angle and a load percent (actuator force over the placeholder force limit, times 100)
- bag centre, handle point and a valid flag (0 when the detector has no handle)
- rim width, standing, confidence, camera-versus-lidar

The handle and the rim are projected through a random wide-FOV fisheye and back-projected with one fixed nominal model. Nothing is read from `front_camera.json`.

Action, clipped to a fixed range: forward offset (m), sideways offset (m), close height (m), pitch (deg), roll offset (deg), speed scale. The scripted baseline uses `TRIES` from `bag_camera.py` for the first two of those, close height 0.19 m, roll offset 0 and speed scale 1.

Success is both finger pads on the rim or a handle, the bag bottom at least 2 cm off the floor, held 3 s, slip under 1 cm, and no collision. The reward is +10, -5 on a collision, -2 if the jaws had the bag and then lost it, and -0.1 per second (the tier-2 time term, -0.01 per 10 Hz step).

## Placeholders

Stall torque, bag mass, rope thickness, ribbon size, Young's modulus, camera intrinsics and the camera mount are not measured. They are named in `rl_grasp/constants.py` and marked PLACEHOLDER. The paper wall the jaws pinch is a plate `jaw_gap(0.10)` thick along the closing axis, so a jaw commanded to 0 rad stalls on it. That stands in for paper bunching in the jaws. The real bunched thickness was not measured.

MuJoCo 4.0.1 has no `<composite>` cable. Handles are `flexcomp dim="1"` rods: round for the paper bag, a flat box section for the gift bag.

## A full tier-1 run

On this machine, 200 nominal PPO tries with 4 workers took 40.0 s, about 5 tries/s. At that rate 200k tries is about 11 hours and the plan's 50k-try budget is about 2 hours 47 minutes. That is slower than the 10–20 tries/s guessed from the physics-only bench, because each try includes the 3 s hold and the Python executor. Train without `--nominal` for that run, and compare to the scripted policy on the same seeds. The numbers are in `WORKLOG.md`.
