# From the simulation to the real robot: what is still needed

Date: 2026-11-13. The simulation (`hmc/`) now earns 1115 of 1115 in every run of the
realism ladder's last step: real base limits at 0.5 m/s top speed, realistic noise,
10 passers-by, the robot's sensor model, and calibrated odometry with scan-match
corrections (`python -m hmc.ladder --seeds 10 --speed 0.5`, evidence in
`hmc/docs/evidence/help-me-carry-slice2/robustness/`). This page lists what the real robot
must have before that result can mean anything on the floor. A simulation result is not
a hardware result.

## 1. Wheeled base

| Need | Why (from the simulation) | State today | How to close it |
|---|---|---|---|
| Top speed 0.5 m/s | At 0.12 m/s no run finishes in 5:00; at 0.3 m/s about 1 run in 5 times out on the long maps; at 0.5 m/s none does | 0.12 to 0.13 m/s whatever is commanded (`base/docs/hardware-assessment-2026-11-13.md`, Task 2) | Find whether the cap is in the STM32 firmware or the power supply (wheels-raised test), then new firmware or motor power. Session "Pi cleanup, modules and new STM32 firmware" owns this |
| Braking 0.25 m/s² or more | The robot never drives faster than it can stop within 1.5 m, where it is sure to see the hard-to-see object | Coast of 5 to 8 cm from 0.12 m/s, not measured above that | Speed ladder (`base/pi/tools/base_check.py`) once the base goes faster |
| Turn rate 0.49 rad/s, turn accel 0.7 rad/s² | The simulation's real base uses these | Configured, not measured | Turn test of the speed ladder |
| Odometry within about 1 % | Assumed by the `calibrated` odometry model | Measured: distance about 2 % short (scale 0.882), turns within 0.3 % | Done; recheck after a firmware change |

## 2. Sensors

| Need | State today | How to close it |
|---|---|---|
| Camera pitch known within 1° | Not measured (estimates 11 to 17° up) | `base/pi/tools/sensor_check.py --wall` with a flat board 1.2 to 1.5 m away |
| LiDAR front angle settled | 178.7° in use; the robot's own posts suggest 180.3° | `sensor_check.py --yaw` with an object taller than 0.35 m |
| Depth camera sees a 0.12 m object from 0.6 m | Assumed by `--sensors robot` | Person-follow plan, Task 6 step 3 |
| Re-identification looks for every person | The crowd memory, the queue and the operator check all need them | Jetson detector and OSNet looks exist (Task 11); the Pi must ask for looks of everyone (return-trip plan, Task 4) |
| The Jetson on the robot, on a LAN cable, with its own power | Outside the room Wi-Fi drops; without looks the robot cannot tell the queue from passers-by | Waiting for parts (owner) |

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
   `base/docs/superpowers/plans/2026-11-13-return-trip.md`. Pointing detection and the
   pick need the arm and a pose model, not planned yet.
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
