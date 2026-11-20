# Tier-1 work log

20 Nov 2026. Laptop/simulation only. No deploy scripts, robot runtime, Jetson, or Pi were changed.

## What landed

`sim/rl_grasp` builds a MuJoCo scene from `arm/so100_moveit_config` (compiler `meshdir`, package paths stripped). It adds placeholder position actuators, finger pads, the deck and the floor, a rigid paper bag and a rigid gift bag, each with two `flexcomp dim="1"` handles. MuJoCo 4.0.1 rejects `<composite>`. The executor follows `CameraBag`: look pose, aim at the handle when the detector has one and at the rim otherwise, close height and pitch, up to four tries, jaw command 0, lift, hold 3 s. PPO and SAC are in this package. Observations are numbers only, degraded by an uncalibrated wide-FOV fisheye.

## Tests

```text
cd sim && python3 -m pytest -q --tb=line -p no:cacheprovider
.......................                                                  [100%]
23 passed in 7.67s
```

`python3 -m ruff check sim` passed.

## Short PPO run

```text
cd sim && python3 -m rl_grasp train --algo ppo --steps 200 --seed 0 --workers 4 --tries 1 --nominal --out /tmp/ppo200.pt
```

Wall time 40.005 s. Output:

```text
wrote /tmp/ppo200.pt
episodes: 200
mean_return: 1.2001
```

That is 200/40.005 = 5.00 tries/s on 4 workers. At this rate, 200k tries would take 40005 s (11 h 6 min 45 s) and the plan's 50k-try budget would take 10001 s (2 h 46 min 41 s). A full run should drop `--nominal`.

Nominal bag, one try, 8 repeats of the same pose (seed 1000 does not move a nominal bag):

```text
python3 -m rl_grasp evaluate --policy scripted --episodes 8 --seed 1000 --tries 1 --nominal
policy: scripted
episodes: 8
success_rate: 1.000000
median_time_s: 11.560
collision_count: 0
successes: 8

python3 -m rl_grasp evaluate --policy /tmp/ppo200.pt --episodes 8 --seed 1000 --tries 1 --nominal
policy: ppo
episodes: 8
success_rate: 1.000000
median_time_s: 10.680
collision_count: 0
successes: 8
```

Randomised bags, seeds 1000–1007:

```text
python3 -m rl_grasp evaluate --policy scripted --episodes 8 --seed 1000 --tries 1
policy: scripted
episodes: 8
success_rate: 0.250000
median_time_s: 11.570
collision_count: 4
successes: 2

python3 -m rl_grasp evaluate --policy scripted --episodes 8 --seed 1000 --tries 4
policy: scripted
episodes: 8
success_rate: 0.375000
median_time_s: 11.600
collision_count: 1
successes: 3

python3 -m rl_grasp evaluate --policy /tmp/ppo200.pt --episodes 8 --seed 1000 --tries 1
policy: ppo
episodes: 8
success_rate: 0.250000
median_time_s: 11.090
collision_count: 4
successes: 2
```

On the one nominal bag the short policy matches the scripted pick and is 0.88 s faster. On 8 randomised episodes it ties the 1-try scripted pick and does not beat the 4-try scripted pick. Eight episodes is a small sample.

## Self-review

- The plate the jaws pinch is `jaw_gap(0.10)` thick along the closing axis so a jaw commanded to 0 rad stalls. That thickness was not measured. A real paper wall is thinner and bunches; this plate only stands in for that.
- Compiled flex bending coefficients include `-1` slots, and raising the positive ones segfaults MuJoCo 4.0.1. Stiffness randomisation only softens them, and the handle length is fixed when the model is compiled. The rope collides with the finger pads only. Arm-versus-rope is not scored, because that contact was unstable. Arm versus the bag body, the deck and the floor still counts.
- 200 tries on one bag is not a tier-1 result. The policy has not been shown to beat today's 4-try pick under randomisation, and the actuator torque, bag mass, rope radius and camera are still placeholders.

## Follow-up: the 50k run that did not grasp

20 Nov 2026, after PR #18. A 50k PPO run on the merged code (`--steps 50000 --workers 6`, no `--nominal`) finished in 164 s, about 300 tries/s, mean return -0.0089. On 100 seeds the scripted pick was 19/100 (1 try) and 32/100 (4 tries). The policy was 0/100 with 0 collisions.

### Causes, and the measurement

1. Speed. A nominal scripted try is 11.560 s of simulation and about 0.67 s of wall time here. An initial random policy on the old wide randomisation was instant (sim time 0, no `mj_step` of the arm) on 22 of 40 tries, because the reach check or the IK returned before any motion. After a few updates the learned action was always that reject, so the rest of the 50k steps were resets. Mean return -0.0089 is the time penalty on a clock that never starts. The training workers were not skipping physics; the actions never asked for any.

2. Reward. A rejected reach scored 0. A real miss scored about -1.16. A collision scored about -6.2. Doing nothing beat trying, so PPO learned to avoid collisions by never grasping. The zero network output was also not the scripted pick: the close-height bounds were 0.12–0.28, so 0 mapped to 0.20 m, not 0.19 m. Advantage normalisation divided by a tiny standard deviation when a batch of one-step tries all had nearly the same reward.

3. Randomisation. On 16 seeds with the old ranges, turning the camera off (true pose, rim aim) raised the scripted pick from 4/16 to 12/16. Pose, yaw and dynamics alone stayed at 14/16 or 16/16. The wide fisheye moved the estimated wall by up to 8 cm. The close collides when that estimate is under about 0.28 m and is rejected past 0.32 m, so an 8 cm error falls out of the only band that works. Handle-aim versus rim-aim did not change that 16-seed count.

### What changed

A miss is charged at least 11.56 s, the measured nominal scripted try. Success still uses the real clock, so a faster lift scores higher. Network output 0 is the scripted pick (height bounds 0.15–0.23, centred on 0.19). The mean head starts at 0. Advantages are not rescaled when their spread is under 1e-3. The parallel learning rate is 3e-4, the same as the single-process trainer.

The default camera is 1 cm of position, 2% of focal length, 4 px and no extra aim jitter (back-projection p90 about 1.5 cm). Mass and friction are ±20% of the placeholder. Actuators are ±10%. Bag walls are spawned at 0.305–0.312 m. `--hard` keeps the old wide ranges. The real default is still the rim aim; the handle shift is `tuned=True`.

`tests/test_rollout.py` checks that a training step of the zero action is a full try and matches `evaluate` on the same nominal seed. `tests/test_reward.py` checks that quitting early does not beat a real miss. `tests/test_camera.py` checks that the default camera stays within 4 cm and that `--hard` can move the point further.

### 50k rerun, this machine, 4 workers

```text
cd sim
python3 -u -m rl_grasp train --algo ppo --steps 50000 --seed 0 --workers 4 --tries 1 --out /tmp/ppo50k.pt
```

```text
episodes: 50000
mean_return: 5.3776
wall_s: 5454.969
tries_per_s: 9.17
median_time_s: 8.960
instant_fraction: 0.184
```

```text
python3 -u -m rl_grasp train --algo sac --steps 50000 --seed 0 --workers 4 --tries 1 --out /tmp/sac50k.pt
```

```text
episodes: 50000
mean_return: 6.7026
wall_s: 6307.326
tries_per_s: 7.93
median_time_s: 10.400
instant_fraction: 0.151
```

Same 100 seeds (1000–1099), default randomisation, not `--hard`:

| policy | tries | success | median time | collisions |
|---|---|---|---|---|
| scripted | 1 | 75/100 | 11.580 s | 0 |
| scripted | 4 | 80/100 | 11.580 s | 1 |
| PPO | 1 | 77/100 | 8.920 s | 0 |
| PPO | 4 | 89/100 | 8.920 s | 1 |
| SAC | 1 | 80/100 | 9.620 s | 1 |
| SAC | 4 | 94/100 | 9.620 s | 2 |

Both policies beat the scripted pick on these seeds: more successes, and a lower median time. The scripted 1-try rate is 75/100, above the real 2/3. The sim is still optimistic, and `--hard` is the wide draw that produced the 19–32% result.

### Self-review

- The 11.56 s floor is the nominal scripted duration in this sim, not a measured real pick. It stops the "do nothing" exploit. It does not make a fast miss cheaper than a slow one, except when the miss is also a success.
- The default camera is tight because the arm only closes cleanly in a few centimetres of wall distance. That is a property of this scene, not a calibration. `--hard` is still there, and it will look much worse.
- Flex warnings still show up during training. A few steps report a stiff inertia. They did not stop the runs, and they are not a measured rope modulus.
