# From the simulation to the real robot: what is still needed

Date: 2026-11-13. The simulation (`hmc/`) now earns 1115 of 1115 in every run of the
realism ladder's last step: real base limits at 0.5 m/s top speed, realistic noise,
10 passers-by, the robot's sensor model, and calibrated odometry with scan-match
corrections (`python -m hmc.ladder --seeds 10 --speed 0.5`, evidence in
`hmc/docs/evidence/help-me-carry-slice2/robustness/`). This page lists what the real robot
must have before that result can mean anything on the floor. A simulation result is not
a hardware result.

Rows of sections 1 to 3 updated 2026-11-21 from `base/docs/hardware-assessment-2026-11-13.md` and the code.

## 1. Wheeled base

| Need | Why (from the simulation) | State today | How to close it |
|---|---|---|---|
| Top speed 0.5 m/s | At 0.12 m/s no run finishes in 5:00; at 0.3 m/s about 1 run in 5 times out on the long maps; at 0.5 m/s none does | About 0.12 m/s at full PWM on the firmware flashed 2026-11-14 ("Top speed on the floor is a hardware limit", 2026-11-15); 0.12 to 0.13 m/s on the old firmware (Task 2) | The cap is the motor supply, the L298N and the motors, not the firmware. Not measured yet: the motor voltage under load and the motors' rated voltage |
| Braking 0.25 m/s² or more | The robot never drives faster than it can stop within 1.5 m, where it is sure to see the hard-to-see object | Old firmware: coast of 5 to 8.4 cm from up to 0.12 m/s; the 0.08 m/s run gives 0.09 to 0.11 m/s² with the stop delay counted once (`decel` 0.10 in `base/params.json`, 4e39afb). The flashed firmware ramps down at `BRAKE_MPS2` 0.5 m/s²; its floor stop is not measured | `base/pi/tools/brake_distance.py` (0.08, 0.12 and 0.15 m/s) |
| Turn rate 0.49 rad/s, turn accel 0.7 rad/s² | The simulation's real base uses these | Turn rate measured on the old firmware (evening 2026-11-13): 0.47 to 0.50 rad/s for 0.49 asked, about 0.70 rad/s at most on the spot. Turn accel not measured | Turn test of the speed ladder on the flashed firmware, with the turn accel |
| Odometry within about 1 % | Assumed by the `calibrated` odometry model | Old firmware: distance about 2 % short (scale 0.882), turns within 0.3 %. Flashed firmware, scales 1.0 (2026-11-14): one 1.0 m line 1.4 % short by odometry against the scan, one 180° turn within 0.2° | More lines with `base_check.py --line`: one line so far, and 1.4 % is above 1 % |

## 2. Sensors

| Need | State today | How to close it |
|---|---|---|
| Camera pitch known within 1° | Measured: 12.3° up on the evening of 2026-11-13 (`sensor_check.py --wall`, 30 of 30 frames fitted, spread 12.32 to 12.36; `camera_pitch_deg` in `base/params.json`; `base/docs/hardware-assessment-2026-11-13.md`, "Sensor geometry settled") | Done; measure again after any change to the camera mount |
| LiDAR front angle settled | 178.7° confirmed on the evening of 2026-11-13: a box on the centre line 0.564 m away read +0.03° ("Sensor geometry settled"); the 180.3° estimate assumed the posts are symmetric | Done; check again after any change to the LiDAR mount |
| Depth camera sees a 0.12 m object from 0.6 m | Seen 1.0 m ahead of the front edge (Task 6 step 3, 2026-11-13): 50 depth cells over 10 frames, 0 LiDAR points. Not checked at 0.6 m | The same check with the box 0.6 m ahead |
| Retractable barrier belt (about 0.9 m up) seen as blocked | The LiDAR sees only the posts. Since 2026-11-19 the depth layer keeps points up to `robot_height` 1.05 m, so a belt in the Xtion's view is an obstacle (9b8c128) | Check with a real barrier |
| Re-identification looks for every person | The crowd memory, the queue and the operator check all need them | Jetson detector and OSNet looks exist (Task 11). Since 2026-11-14 the Pi keeps a look for every person matched to a camera person by bearing (return-trip plan Task 4, a12dc95). On the robot: return-trip plan Task 6 |
| The Jetson on the robot, on a LAN cable, with its own power | Outside the room Wi-Fi drops; without looks the robot cannot tell the queue from passers-by | LAN cable since 2026-11-14 (Pi 10.0.0.1, Jetson 10.0.0.2, hardware assessment). On the robot with its own power: waiting for parts (owner) |

## 3. Software the robot does not run yet

The simulation's brain is in `hmc/src/hmc/`. The robot runs `base/pi/run_follow.py`, which
uses `hmc.follower.Follower` for following only. Still to bring over:

1. The changes from 2026-11-13 in the brain: people predicted from their walking speed,
   swept-heading braking, the "never outdrive what you surely see" speed cap, keeping
   small objects in memory while they are out of view. They live in `follower.py` and
   `navigation.py`, which the robot already imports, so a redeploy carries them; check
   them on the robot with `replay.py` logs.
2. The other states of `hmc/src/hmc/task.py`: perceiving the pointed bag, picking it,
   the way back, remembering the crowd and joining the queue. The way back is planned in
   `base/docs/superpowers/plans/2026-11-13-return-trip.md`. Pointing detection is in
   `base/jetson/pointing.py` (`base/docs/pointing.md`). The pick goes through `base/jetson/bag_arm.py`
   to the arm team's `/arm/pick_bag` (`base/docs/floor-pick.md`).
3. Voice: "ready", the pace request, "please wait for me", "please go on", "I lost you,
   please come back", "please let me pass". `hmc/src/hmc/hri.py` has the texts; the
   voice team's stack on the Jetson can say them.

## 4. Assumptions of the simulation to check in the real arena

These are the simulation's own choices, not the rulebook's. If the real test differs,
the result can differ too.

- The queue walks at 0.5 m/s and stops in a line next to the start. It starts 2 s after
  the robot reaches the car. Until 2026-11-13 the queue on the `default` and Eindhoven
  maps walked on past the start; no robot under about 1.5 m/s could join it there.
- Passers-by step to the side of the robot's way when it comes within 1.2 m.
- The operator walks round the robot instead of into it, waits when asked, and comes
  back when asked.
- The maps are traced by eye; the outside, the car and the obstacles are made up.

## 5. Order

1. Base speed and braking (section 1). Nothing else matters at 0.12 m/s.
2. Sensor geometry (section 2), then the person-follow plan's first drives.
3. Jetson on the robot.
4. Return trip and queue on the robot; then the arm and voice for the bag.
5. Field runs of every criterion, inside and outside, as both plans require.
