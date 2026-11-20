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
