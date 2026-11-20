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

Train. `--steps` is the number of grasp tries, not physics steps. `--nominal` keeps one paper bag in one place. Without it, the bag pose, a narrow mass and friction band, and a few centimetres of camera error are randomised. `--hard` is the wide camera and dynamics range. `--workers` is CPU processes for the physics. A network output of 0 is the scripted pick.

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

Action, clipped to a fixed range: forward offset (m), sideways offset (m), close height (m), pitch (deg), roll offset (deg), speed scale. The scripted baseline uses `TRIES` from `bag_camera.py` for the first two of those, close height 0.21 m, roll offset 0 and speed scale 1.

Success is both finger pads on the rim or a handle, the bag bottom at least 2 cm off the floor, held 3 s, slip under 1 cm, and no collision. The reward is +10, -5 on a collision, -2 if the jaws had the bag and then lost it, and -0.1 per second (the tier-2 time term, -0.01 per 10 Hz step).

## Placeholders

Stall torque, bag mass, rope thickness, ribbon size, Young's modulus, camera intrinsics and the camera mount are not measured. They are named in `rl_grasp/constants.py` and marked PLACEHOLDER. The paper wall the jaws pinch is a plate `jaw_gap(0.10)` thick along the closing axis, so a jaw commanded to 0 rad stalls on it. That stands in for paper bunching in the jaws. The real bunched thickness was not measured.

MuJoCo 4.0.1 has no `<composite>` cable. Handles are `flexcomp dim="1"` rods: round for the paper bag, a flat box section for the gift bag.

## A full tier-1 run

A full try is about 11 s of simulation, including the 3 s hold. A rejected reach used to return in one step and score 0, which was better than a real miss, so a long PPO run collapsed onto that and reported hundreds of tries per second. A miss is now charged at least the scripted try's duration.

The close height is 0.21 m, matching `bag_cam_close_h`. On the default ranges, 50k PPO tries took 5472 s and 50k SAC tries took 5872 s (4 workers). On the same 100 seeds, scripted was 71/100 and 77/100 (1 and 4 tries, median 11.200 s). PPO was 78/100 and 91/100 (median 8.920 s). SAC was 79/100 and 90/100 (median 9.800 s). The checkpoints are in `policies/`. Details are in `WORKLOG.md`.
