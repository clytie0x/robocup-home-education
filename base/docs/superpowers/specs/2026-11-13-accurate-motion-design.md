# Accurate motion for the real base: design

Date: 2026-11-13. Owner's request: plan and fix all three groups of motion errors found on
the robot, continuously, the best method first; optimise only if RAM or CPU runs short.
Evidence for every number: `base/docs/hardware-assessment-2026-11-13.md` (measured today).

## Why

The robot locks onto the operator and stops 0.6 m from them, but it does not follow
through the gaps of a furnished room, its track drops next to furniture, and the base does
not do quite what it is told. What was measured:

| Measured | Value |
|---|---|
| Planner gives up in a 0.8 m gap (plans the 0.40 m body as a 0.29 m circle plus 0.10 m) | path 0 in 406 of 528 following cycles, run 3 |
| The wall-following test passed a 0.62 m gap without stopping | lap run 2 |
| Speed actually driven while following, against the command | 0.63 to 1.03 times |
| Command to first motion | 0.3 to 0.6 s (from 10 Hz logs and 1 cm odometry, rough) |
| Brain's braking model against the base | brain 0.25 m/s^2, no delay; base 0.064 m/s^2 after 0.12 to 0.32 s |
| Turning commanded while standing still behind the operator | 343 of 855 cycles, 12 sign flips |
| Sideways drift driving straight | 1.3 to 2.7 cm per metre |
| Firmware odometry step | 0.01 m, no timestamp |
| Firmware speed control | open loop (top speed drops under load), no command timeout |
| Track lost next to furniture (legs counted as possible people) | locked 67.6 % and 80.8 % in runs 2 and 3 |

## Goals

1. The robot follows the operator through any gap of at least 0.65 m, and never touches
   anything (closest approach to the body at least 0.03 m from the LiDAR and depth points,
   at least 0.30 m straight ahead as today).
2. The base drives what the brain asks: straight-line speed within 10 %, turn rate within
   10 %, no turning back and forth while standing.
3. The robot knows where it is: after a 10 m route with two turns its pose error is at most
   0.10 m with SLAM (the return-trip criterion asks 0.5 m).
4. The simulation keeps running the same brain: every map still earns 1115 points at
   0.5 m/s (ladder last rung), and the evidence folder is regenerated.

## Principle: measure while driving, bound by fixed safety limits (owner, 2026-11-13)

The hardware is measured coarsely (odometry in 0.01 m steps, no firmware speed loop), so
the software carries the accuracy. Values in `params.json` are starting values, not
truth: while the robot drives, it measures what it really did and keeps its own estimates.
Safety limits stay fixed and the estimates may only make the robot more careful, never
less: top speed, the 0.30 m clearance ahead, the 0.03 m body margin, and a floor under
the braking distance (the worse of the estimate and `params.json`).

Layers, bottom first; each one is built on the one below:
1. Stopping for sure: `safe_run` now, a firmware timeout later.
2. Wheel speed and measured motion: a motion estimator on the Pi fuses wheel odometry with
   LiDAR scan-to-scan matching every revolution and gives the real (v, w); a velocity loop
   on the Pi corrects the command until the measured speed matches; new firmware later
   adds a wheel loop and finer odometry.
3. Pose: the same estimator gives the pose; slam_toolbox corrects it on the Jetson.
4. Collision checks with the real outline. 5. Planning. 6. Following and who is who.

Estimated online, each with bounds and a fallback to `params.json` when the estimate is
not trustworthy (too few samples, scan match below 60 % agreement, value outside bounds):
speed gain (driven / commanded), turn gain, straight-line drift (turn per metre),
command latency, braking (deceleration and stop delay, from every stop).

## Design, group B: the brain (code only, first)

- **Footprint, not a circle.** Planning inflates obstacles by the inscribed half-width
  (0.20 m) plus `SAFETY`, then checks the real outline (`world.FOOTPRINT`) along each path
  segment, facing the segment's direction. A segment whose outline comes within `SAFETY`
  of a shape is blocked and the path is planned again (at most 5 rounds). Turning on the
  spot needs the circumscribed radius (0.256 m) clear; the navigator only turns on the
  spot where that holds.
- **The last-resort stop uses the outline too.** `safety.must_stop` sweeps the outline
  along the commanded arc for the run-on time (stop delay plus braking) and stops when any
  point comes within 0.03 m; the 0.30 m clearance straight ahead stays. This replaces the
  0.33 m spin circle that froze the robot beside a wall.
- **A base model with the real numbers.** `world.MAX_DECEL` (braking) is separate from
  `world.MAX_ACCEL`; the brain brakes with the estimator's deceleration and stop delay
  (never better than the floor). The Pi predicts its pose ahead by the estimated latency
  before each brain step.
- **Heading deadband.** Within 4 degrees of the wanted heading while standing, the robot
  does not turn.
- **Furniture is not a person.** A LiDAR candidate counts as a person (for the crowd and
  company rules and the planner's person circles) only once it has moved 0.25 m from where
  it was first seen, or the camera matched a person at its bearing in the last 2 s. Until
  then it is an obstacle like any other.
- **Searching for the operator is part of the follower.** With nobody to lock onto, the
  robot turns slowly on the spot while every frame goes to the detector, and stops turning
  the moment a person is in the middle of the image.

## Design, group A: the base (measure, calibrate, then hardware)

- A1 and A2 (latency, drift) are no longer one-off calibrations: the motion estimator
  measures them all the time (layer 2), and the velocity loop corrects them.
- A3 new firmware (draft `base/docs/firmware-design.md`): wheel speed PI loop, odometry in
  millimetres with a timestamp, 0.3 s command timeout. Needs the owner at the board
  (photos, ST-Link, multimeter), and only after the backup of the old firmware.
- A4 an IMU (gyro) on the Pi's I2C, fused with wheel odometry. Needs a part.

## Design, group C: knowing where the robot is

- slam_toolbox (ROS2 Jazzy, online async) on the Jetson, fed from the Pi: every LiDAR
  revolution and the odometry pose go over a second TCP connection (port 5602); a ROS2 node
  on the Jetson publishes `/scan`, `/odom` and the `odom -> base_link` transform, and
  sends the `map -> odom` correction back. The Pi keeps driving on odometry when the link
  is down, and the follower never needs the map.
- The return-trip plan's breadcrumbs stay (they handle dead ends); its scan keyframes are
  replaced by the SLAM correction. The owner installs `ros-jazzy-slam-toolbox` (sudo) and
  tells the other teams.
- RAM on the Jetson is measured with the detector, SLAM, voice and arm running. If it does
  not fit, the fallback is the return-trip plan's keyframe matching on the Pi.

## Out of scope

The arm and the bag, voice, the queue (return-trip plan), outdoor sunlight on the depth
camera.
