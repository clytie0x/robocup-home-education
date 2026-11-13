# Hardware assessment for person following, 2026-11-13

Every row says where the number comes from. "Measured today" means a command was run on
the robot on 2026-11-13 and its output was read. "Config" means a value read from a file,
not checked against the real robot. "Unverified" means nobody has measured it.

A row marked **GATE** must be measured before the follower may drive faster than
0.15 m/s. The plan (`superpowers/plans/2026-11-13-person-follow.md`) has one task per gate.

## Computers

| Item | Value | Source |
|---|---|---|
| Pi | Raspberry Pi 4 Model B Rev 1.5, 4 cores, 3.8 GB RAM (3.3 GB free), Debian 13, not throttled, 37 °C idle | Measured today |
| Pi Python | `~/pi-drive/.venv` has numpy 2.5.4, OpenCV, pyserial. No pytest (tests use `unittest`) | Measured today |
| Jetson | Orin Nano Super dev kit, 6 cores, 7.3 GB unified RAM, 25 W mode, JetPack r39.2, CUDA 13.2, NVMe 802 GB free | Measured today |
| Jetson load | Shared with the arm and voice teams. At the time of the check: 4.6 GB RAM used, CPU 65 to 80 % (rviz2 and MoveIt), GPU 0 % | Measured today |
| Jetson GPU inference | Nothing usable yet. No TensorRT, no ultralytics. `ros2_venv` has torch 2.15.0 but `import torch` fails on numpy. onnxruntime lists only the CPU provider. `apt` offers `tensorrt 10.16.2.10-1+cuda13.2` | Measured today |
| Jetson location | Not on the robot today. The owner expects a LAN cable at the competition | Owner, 2026-11-13 |
| Network | Pi `wlan0` 192.168.31.21 and Jetson `wlP1p1s0` 192.168.31.137, same Wi-Fi. Ping 2.5 / 5.8 / 10.0 ms (min / avg / max, 5 packets) | Measured today |

## Wheeled base (STM32 over `/dev/rct_ahr_robot`, CH340, 115200 baud)

| Item | Value | Source |
|---|---|---|
| Command | `F014\|+026#` = forward 0.14, turn +0.26 (centi-units of m/s and rad/s, firmware units). `B` = backward. `S#` = stop | Read from `rct_driver_watchdog.py` and `pi-drive/wheels.py` |
| Odometry | `O#` returns `Ox:+000.00,y:+000.00,th:+000.00#` (metres, metres, radians, firmware units) | Read from `driver.py` |
| Odometry scale | true yaw = firmware yaw / 0.862. true distance = firmware distance / 0.902 | Measured today with LiDAR scan matching: 360° turns within 0.3 %, 0.27 m lines within 1 % |
| Command scale, turn | The base turns 1 / 0.862 times faster than asked, so send `w * 0.862` | Measured today (same turns) |
| Command scale, linear | **GATE.** Two old numbers disagree: a request of 0.117 m/s gave 0.135 m/s on 2026-11-07, yet `pi-drive/wheels.py` multiplies every request by 1.28 | Unverified |
| Top speed | **GATE.** The ROS driver was capped at 0.15 m/s until today. The cap is now 0.30 m/s, but no run has reached it: the only logged maximum is 0.103 m/s | Measured today (log), not verified |
| Braking | Hard stop from the old cruise speed coasts 4.6 cm and 8.0°. A soft stop gives 2.4 cm and 2.0° | Measured today at 0.15 m/s and below. **GATE** above that |
| Turn odometry under `pi-drive` | **GATE.** In a 20 s follow run the command was `-026` for about 10 s while firmware yaw moved only 0.14 rad. Either the base did not turn as asked or `th` was read wrong | Measured today, cause unknown |
| Footprint | 0.40 m x 0.40 m, turning centre in the middle (`front_m` 0.2, `rear_m` 0.2, `half_width_m` 0.2) | Config (`robot.json`) |
| Height | **GATE.** Needed to decide what the robot can pass under | Unverified |
| Battery | **GATE.** Voltage, run time and whether it can also feed the Jetson are unknown | Unverified |
| Reverse | The follower never drives backward | Design choice |

## LiDAR (LDS-03 over `/dev/rct_ahr_lidar`, CP2102, 230400 baud)

| Item | Value | Source |
|---|---|---|
| Rate | 9.8 revolutions per second, 398 points per revolution | Measured today (49 revolutions in 5 s) |
| Pose | 0.175 m in front of the turning centre, 0.22 m above the floor, mounted backward: `yaw_deg` 183.5 in `robot.json`, front of the base at laser angle 178.7° in `pi-drive/lidar.py` | Config, and direction checked today with two bottles: front at 359.6°, right at 268.7° in the base frame |
| The two yaw numbers | The ROS stack and `pi-drive` each have their own mounting angle (183.5° and 178.7°, in different angle conventions). They disagree by 2° to 5°, depending on how the conventions map. **GATE**: at 3 m, 5° is 0.26 m sideways | Unverified |
| Body returns | Four posts of the robot itself: about 0.19 m at 58°, 117°, 244° and 302° in the base frame. `pi-drive/decide.py:is_body` masks them | Measured today |
| Blind spots | Sees one plane 0.22 m above the floor. It misses table tops, chair arms and anything lower than 0.22 m | Geometry |
| Sunlight | Not tested outdoors | Unverified |

## Camera (ASUS Xtion on the Pi, USB 2)

| Item | Value | Source |
|---|---|---|
| Stream | Colour and depth pairs, 320 x 240, 21.7 pairs per second through OpenNI2 | Measured today (240 pairs in 11.1 s) |
| JPEG | Quality 80: 2.0 ms and 15.8 kB per frame on the Pi, so about 350 kB/s at full rate | Measured today |
| Field of view | 58.59° horizontal. The image is mirrored: image right is the robot's left | Measured 2026-11-07 (`pi-drive/README.md`) |
| Depth range | Usable from about 0.5 m. `person.py` accepts 0.7 to 4.5 m | Read from code |
| Mount | **GATE.** The camera was moved to the bottom deck today. `robot.json` still says x 0.14, z 0.27, pitch 0. The real height and pitch are unknown | Unverified |
| Person detector on the Pi | SSD, about 6 detections per second | Measured 2026-11-07 |
| Follow with that detector | 5-minute run today: the operator was out of sight in 963 of 1430 cycles (67 %) | Measured today |
| Sunlight | The Xtion measures depth with infrared light. Direct sun usually blinds it. Outside the arena the depth obstacle layer may see nothing | Known limit of the sensor, not tested here |

## What this means

1. **The Pi alone cannot recognise the operator fast enough.** Six detections per second
   and a 58° view lose the operator two thirds of the time. The LiDAR must carry the
   track, and the Jetson must do the recognition.
2. **The Jetson link is a single point of failure outside the arena.** Today the Jetson is
   on a desk and talks over Wi-Fi. Once the robot leaves the room the link will drop. The
   follower must keep following on the LiDAR alone and must never pick a new person
   while the link is down. For the competition the Jetson should ride on the robot, on a
   LAN cable, with its own power. That needs the battery row above.
3. **Speed above 0.15 m/s is not proven.** Every number about braking and command scale
   was taken at or below 0.15 m/s. The speed ladder (0.3, 0.4, 0.5, 0.6 m/s) comes first.
4. **Low and overhanging objects need the depth image**, and the depth image needs the
   real camera pose. Until the mount is measured, the depth layer cannot be trusted.
5. **Outside the arena, sunlight can remove the depth layer.** The plan treats "no depth"
   as "drive slower", not as "nothing there".

## Files changed on the robot today

Recorded here because none of it is in git.

- Pi `~/backup-driver-20261113/` holds the original of every file listed below.
- ROS driver: odometry scales in `driver.py`; `rct_driver_watchdog.py` scales the turn
  command and caps linear speed at 0.30 and wheel speed at 0.35 m/s.
- `robot-deploy-kit`: `config/robot.json` (nav 0.30 m/s, 28°/s, accel 0.2, 40°/s²),
  `common/config.py`, `common/configure-robot.py`, `navigation_guard.py`,
  `command_guard.py`, `task_tree.py` (cruise 0.30 m/s).
- `~/pi-drive`: new `legs.py`, `chase.py`, `test_legs.py`; `main.py` gained
  `--keep-going`, `--no-recall` and ignores a second Ctrl+C while stopping
  (original: `main.py.before-keep-going-20261113`). 111 unit tests pass.
- 205 unused items moved from `~` to `~/archive-xohm-20261113/` (6.88 GB, nothing deleted).

## Task 3: sensor geometry, measured 2026-11-13 (wheels off)

The owner was at the robot. Camera stand hinge and base fixed firmly (owner). Camera on
the bottom deck, tilted up, in its present position; the owner may move it later, and
after any change to the mount the pitch must be measured again
(`tools/sensor_check.py --wall` or `--object H`) before the depth layer is trusted.

| Item | Value | Source |
|---|---|---|
| Camera lens to turning centre, x | 0.205 m (20 to 21 cm) | Owner, tape |
| Camera lens sideways, y | 0.045 m to the robot's left (lens pair 4 to 5 cm left of the Xtion stand, which is on the centre line) | Owner, tape |
| Camera lens height | 0.07 m | Owner, tape. Not cross-checked: the floor fit failed (below) |
| Robot height | 0.24 m, floor to the real top | Owner, tape, confirmed as the highest point |
| Depth stream | 320 x 240, 74.8 % of pixels valid in this room; rows 196 to 239 are zero in every frame | Measured |
| Camera pitch | **Not measured to the required precision.** Estimates from this room range from 11 to 17 degrees up: two near vertical surfaces 12.0 and 12.7, a 23 cm bottle top at 1.5 m about 14, the wall at 3.7 m 14.5 to 17 (`--wall`: median 15.76, spread 15.12 to 16.65, inlier share 0.10, warns "do not trust"). The far wall is quantised in about 4 cm depth steps. `camera_pitch_deg` stays null, so the depth layer stays off | Measured, inconclusive |
| Floor fit | Failed: about 300 floor pixels per frame (needs about 1000). With the camera tilted up, almost no floor is in view | Measured |
| LiDAR front | The robot's own four posts: front pair at +109.24 and -112.88 degrees, rear pair at +144.58 and -147.33 degrees (49 scans). If the posts are symmetric about the drive axis, the true front lies 1.6 degrees right of the present front, so `lidar_front_deg` would be about 180.3. The two-bottle check of 2026-11-07 put it 0.4 degrees the other way. Not resolved; `lidar_front_deg` stays 178.7 | Measured, inconclusive |
| LiDAR and a 23 cm bottle | The LiDAR at 0.22 m does not see a 23 cm bottle at 1.5 m. Objects for checks must be taller than about 0.35 m and opaque | Measured |
| Mirror | Image right is the robot's left; an object 15 to 25 degrees left at 1.4 to 1.75 m appears on the image's right half in both colour and depth | Measured |

What closes the two open values: a flat vertical board about 1.2 to 1.5 m in front of the
lens (pitch, `--wall`), and an opaque object taller than 0.35 m 1.5 m straight ahead on a
floor-tile line through the middle of the robot (LiDAR front, `--yaw`). About five minutes
with the owner.

Power (owner, 2026-11-13): one power bank, 145 W, 25,000 mAh, feeds everything on the
robot, Pi and motors. The Jetson stays off the robot until more parts arrive, because the
power is probably not enough; until then the Jetson link is Wi-Fi only.

Design risk (to decide with the owner once the pitch is known): with the lens 7 cm above
the floor and about 45 degrees of vertical view, a level camera sees a person only up to
about 0.9 m at 2 m range and 1.3 m at 3 m. Re-identification needs the torso. The owner
chose the bottom deck, tilted up, for now.

## Task 2: speed ladder, measured 2026-11-13

The owner was at the robot and approved every run in chat. ROS stopped (`~/robot-pi stop`).
`tools/base_check.py`, straight lines, real distance and speeds by LiDAR scan matching.
Logs: `~/hmc-base/logs/base-check/` on the Pi.

| Run (log) | Packet | Asked | Real cruise | Real dist | Odom dist | After S# | Drift | Yaw scan/odom | stop_delay | Stop |
|---|---|---|---|---|---|---|---|---|---|---|
| 053826-line-0.15 | F015 | 0.15 | 0.120 (s 2-10; 0.112 middle s) | 1.304 m | 1.277 m | 0.079 m | +2.0 cm/m | 2.4 / 1.3° | 0.27 s | emergency (object came 0.35 m ahead; correct) |
| 054734-line-0.15 | F015 | 0.15 | 0.122 | 1.232 m | 1.213 m | 0.081 m | +1.9 cm/m | 1.2 / 0.7° | 0.18 s | time limit |
| 054957-line-0.15 | F015 | 0.15 | 0.131 | 1.235 m | 1.202 m | 0.074 m | +1.3 cm/m | 1.0 / 0.7° | 0.12 s | time limit |
| 055346-line-0.3 | F037 | 0.3 (scale 1.25) | 0.119 | 0.795 m | 0.781 m | 0.084 m | +2.1 cm/m | 1.0 / 0.7° | 0.22 s | time limit |
| 055742-line-0.08 | F008 | 0.08 | 0.080 | 0.663 m | 0.645 m | 0.050 m | -2.7 cm/m | -3.2 / -3.3° | 0.22 s | target |

Run 1's numbers were recomputed after a tool fix (its first scan was a partial
revolution). Drift carries the unresolved `lidar_front_deg` error (about 3 cm per m).

Findings:
- **The base does not go faster than about 0.12 to 0.13 m/s.** `F015` and `F037` both
  give 0.12 to 0.13 m/s; `F008` gives 0.080. Below the cap the linear command is true
  (`linear_cmd_scale` 1.0). The cause is below the Pi (firmware limit or motor supply,
  the power bank feeds everything); the owner expects this is the base's speed. The
  ladder above 0.15 m/s was therefore not run: there is no faster rung to measure.
- Braking: 5 to 8.4 cm of coast after `S#`, well inside the 0.4 m limit; drift 1.3 to
  2.7 cm per m, inside 5 cm per m.
- Odometry reads 1.5 to 2.7 % short (mean 2.2 %) against the scans, so
  `odom_distance_scale` goes from 0.902 to 0.882.
- Turns not measured: the 180° turn at 0.26 rad/s was refused by its clearance check
  (an object 0.41 m from the turning centre). `odom_yaw_scale` 0.862 and
  `angular_cmd_scale` 0.862 stay as measured earlier today.

Values written to `base/params.json`: `linear_cmd_scale` 1.0, `max_speed` 0.12 (the real
cap), `decel` 0.064 m/s² (v² / (2 × coast), lowest run, the 0.08 m/s one), `stop_delay`
0.27 s (highest run), `odom_distance_scale` 0.882. Braking distance by the plan's formula
at 0.12 m/s: 0.113 + 0.032 + 0.15 = 0.295 m.

Top speed without a depth image (owner decision 5): the plan's 0.3 m/s cap is above what
the base can do, so it never acts; at 0.12 m/s the braking distance (0.295 m) is far inside
what the LiDAR alone sees ahead. Measured choice, 2026-11-13: no separate no-depth cap is
needed while `max_speed` is 0.12.

## Task 8: link to the Jetson over Wi-Fi, measured 2026-11-13

`base/jetson/server.py --fake` on the Jetson (192.168.31.137:5601), 60 s stream of real
Pi camera frames (320 x 240, JPEG) through `link.JetsonLink`: 29.42 frames sent per second,
29.42 answers per second (1765 answers), round trip median 16.5 ms, 95th percentile
22.3 ms, no link error. Pi to Jetson ping 1.7 to 4.9 ms. The fake server answers at once,
so this is the link alone; the detector's time adds to it (Task 11).

## Task 11: person detector and look vectors on the Jetson, measured 2026-11-13

TensorRT 10.16.2.10 installed by the owner (`apt`, 22 packages). FP16 engines built with
`trtexec`, input 256 x 320; look vectors from OSNet x0.25 (512 numbers). Sources, licences,
SHA-256 and exact commands: `base/jetson/models/README.md`. `bench.py`, 300 frames of
320 x 240 with two people, detector plus looks, while the other teams' processes ran as
found (load 5.5 to 6.1, rviz2 about 355 % CPU):

| Detector + OSNet | fps median | fps 5th pct | Engines + buffers (MiB, without the CUDA context) | RSS (MiB) | Pass |
|---|---|---|---|---|---|
| YOLO11n | 94.7 | 91.3 | 178 | 737 | yes |
| YOLOv8n | 102.2 | 96.0 | 198 | 736 | yes |
| YOLOX-nano | 106.9 | 99.2 | 160 | 727 | yes, but found only one of the two people in 36 of 300 frames |
| YOLOX-tiny | 109.8 | 107.1 | 163 | 742 | yes |

A 2000-frame check of YOLO11n gave median 96.9, 5th percentile 94.3. All four pass the
15 frames per second and 1.5 GB gates with a wide margin. The choice between them waits
for person recall on frames from this robot's low, tilted-up camera (Task 11 step 4).

## Evening checks, 2026-11-13 (nothing moved)

- Pi before the deploy: ROS off, no Python running, both serial ports free, not throttled.
  `~/hmc-base` matched `feature/person-follow` (01a5f3b) file for file, plus one file not
  in git: `~/hmc-base/air_check.py` (written 06:43, SHA-256 `a8a62ccf...2a15`, no saved
  output, not known whether it ran). It was left in place; `base/pi/tools/air_check.py`
  replaces it in git.
- Firmware odometry is not reset when a program opens the wheel port: the first `O#` of
  the five base-check runs reads x 5.28, 6.55, 7.76, 8.96, 9.73 m. So DTR does not reset
  the STM32, and the Pi probably cannot put it into its bootloader without a hand at the
  board (RTS not checked).
- No ST-Link and no STMicro USB device on the Pi (`lsusb`); `openocd` is installed,
  `st-flash`, `stm32flash`, `dfu-util` are not. No firmware project or board document
  was found on the Pi (`~`, the archive, `catkin_ws`, `robot-deploy-kit`).
- Deployed `sim/robustness-ladder` (e59e331) with `base/tools/deploy.sh`: the Pi now
  matches it in 59 of 60 files (the extra one is the old `air_check.py`). Tests on the
  Pi: 159 run, OK (1 skipped). On the Jetson `~/hmc-follow`: 17 run, OK.
- `hmc.navigation.plan_path` on the Pi, room scene of 189 circles (walls, a box, a chair),
  50 runs each: operator 2.2 m ahead median 27.9 ms, worst 28.8 ms; goal 3.5 m past the box
  and chair median 33.7 ms, worst 34.3 ms. Under the 150 ms limit, so no planner thread
  is needed. Synthetic scene, not the plan's real box and chair (Task 6 step 3 still open).
- Pi to Jetson ping over Wi-Fi: 5.2 / 23.5 / 59.0 ms (min / avg / max, 3 packets), worse
  than the morning. From the laptop the LAN addresses timed out later in the evening
  while Tailscale still worked.
- The ROS driver still uses `distance_scale` 0.902 (`driver.py` default); the follower
  uses 0.882. Only the ROS path (voice moves over `/cmd_vel`) is affected.

New risk for Help Me Carry: retractable barriers have a belt about 0.9 m above the floor.
The LiDAR plane (0.22 m) sees only the posts, and the robot (0.24 m high) fits under the
belt, so it may drive into the blocked area without touching anything. Only the depth
image can see the belt, and only once `camera_pitch_deg` is known.

## Safety review of the drive path, 2026-11-13 (before the first drive)

A review of `safety.py`, `wheels.py` and `run_follow.py`. Each item says whether it was
confirmed by reading the code here.

Fixed in this branch (`163 tests OK` on the laptop and on the Pi):

1. **An incomplete LiDAR revolution read as free space** (confirmed). `ScanStream.feed`
   promoted whatever had arrived between two angle wraps, so a revolution with lost
   packets had no samples where the packets were missing, and a missing sample cannot be
   told from clear air. `must_stop` saw nothing there and the robot would drive on. Real
   rate: in the 549 revolutions saved by the five base-check runs, 2 were broken (0.4 %),
   the worst with 135 samples and a 252 degree hole. Good revolutions have gaps of 1.03
   degrees median, 1.47 at the 95th percentile. Fix: `lidar.complete()` with a 5 degree
   limit; a holed revolution is dropped and the last good one is kept, so after 0.5 s
   (`max_age`) `latest()` returns nothing and `run_follow` stops with "no scan".
2. **`S#` was sent exactly once per stop** (confirmed). The watchdog wrote one `S#` and
   then returned early for ever; `stop()` and `close()` wrote one each. One byte lost to
   an 8E1 parity error left the base driving on the last command. Fix: every stop writes
   `S#` three times (`STOP_REPEATS`), including when the port is opened, which also stops
   a base left driving by a crashed run. The port still goes quiet after a stop, so
   `air_check.py`'s silence test still measures silence.

Open, and the reason the owner must keep a hand on the power switch:

3. **Process death leaves the last `F` packet in force** (confirmed by reading; the
   firmware's behaviour is unmeasured). The only watchdog is a thread in the same process.
   `kill -9`, the OOM killer, a crash in the OpenNI thread, a Pi brown-out or an unplugged
   CH340 run no `finally` and send no `S#`. `run_follow.py` claimed in a docstring that
   "the firmware then stops on its own silence"; that was never measured, and the docstring
   now says so. `tools/air_check.py` part 2 measures it. No Python change can remove this
   risk.
4. **Jetson link loss does not stop the robot** (confirmed). This is the design the owner
   chose (`specs/2026-11-13-person-follow-design.md`: keep following on the LiDAR track
   when the Jetson is gone), so it is not a fault. The real question behind it is whether
   the LiDAR leg track can jump to another person while no look arrives to contradict it.
   `legs.py` and `senses.py` were not reviewed in depth. Open for Task 9 and Task 13
   scenario 4.
5. **The 0.3 m closest approach was not enforced by any layer** (confirmed by arithmetic;
   fixed, owner's decision 2026-11-13). `must_stop` fired at the braking distance, whose
   last term is a fixed 0.15 m, so with the measured decel 0.064 and stop_delay 0.27 the
   base stopped with only 0.15 m left at every speed (trigger 0.295 m at 0.12 m/s, 0.153 m
   at 0.01 m/s). Fix: `safety.CLEARANCE` 0.30 m; the strip now reaches run-on distance plus
   0.30 m, so the base stops with 0.30 m left ahead at any speed. At 0.12 m/s the strip
   reaches 0.445 m from the front edge, inside the 0.60 m follow gap. That margin closes at
   0.18 m/s with today's braking: above it the robot would stop behind every operator, so
   braking must be measured again before `max_speed` rises (a test now fails if it is
   raised alone). Sideways and while turning this floor does not apply: `SIDE_MARGIN` is
   0.05 m and `SPIN_RADIUS` 0.33 m, and a 0.3 m floor there would forbid turning in a door.
   Side clearance stays the planner's job.
6. Smaller, reported and not fixed: the brain brakes with `accel` 0.25 m/s² while the base
   needs 0.064 m/s² plus 0.27 s, so a close approach ends about 0.07 m short of the
   0.60 m gap; no heading deadband, so a standing operator gives small turn packets back
   and forth (wheel chatter, not a hazard); `--max-speed` is enforced through the brain
   only, not in `packet()`; no `write_timeout` on the wheel port, so a blocked write could
   hold the lock; the scan timestamp is the parse time, allowing about 1 s of sensor age in
   the worst cycle; valid but frozen or reset odometry is not detected; `must_stop` checks
   a straight strip while a turning robot sweeps an arc.

Not a fault: `safety.speed_cap`'s blind-camera cap (0.3 m/s) is above `max_speed` (0.12),
so it never acts. That was the measured decision of Task 2, recorded above.

## Turns, the speed cap and the firmware timeout, evening 2026-11-13

The owner watched and, from this evening, lets the session drive without a per-run
approval (announce each run; report danger). ROS off. `tools/base_check.py`, wheels on the
floor; logs in `~/hmc-base/logs/base-check/`.

| Run | Asked | Real rate (middle 1 s) | Yaw scan / odom | Coast after S# | Stop |
|---|---|---|---|---|---|
| 203934 | +0.26 | 0.267 | 126.2 / 126.3 deg | 3.7 deg | emergency: a return inside the robot outline (fixed, see below) |
| 204400 | +0.26 | 0.252 | 184.7 / 184.1 deg | 3.9 deg | target (report recomputed: `timeout 60` killed the 60 s report) |
| 204747 | +0.26 | 0.275 | 183.9 / 183.5 deg | 3.7 deg | target |
| 204919 | -0.26 | 0.245 | -183.7 / -184.1 deg | 4.6 deg | target |
| 0.49 x3 | +0.49, -0.49, +0.49 | 0.472, 0.478, 0.504 | 191.1/191.4, -192.2/-192.8, 190.4/191.4 deg | 11.5, 12.6, 10.8 deg | target |
| 0.65 | +0.65 | 0.621 | 201.1 / 200.7 deg | 20.3 deg | target |
| 0.80 | +0.80 | 0.659 | 186.2 / 186.1 deg | 24.6 deg | time limit |
| 1.00 | +1.00 | 0.696 | 166.9 / 166.8 deg | 26.1 deg | time limit |

Also a line at 0.08 m/s to get clear of a wall behind the robot: 0.346 m real, 0.079 m/s,
5.1 cm after S#, stop_delay estimate 0.32 s (above the 0.27 s in `params.json`; one run,
not changed).

Findings:
- **Turn odometry gate closed.** Under `wheels.py` the odometry yaw agrees with the scan
  within 1.0 deg on all 9 turns that reached their end (0.5 % at most), both directions.
  `angular_cmd_scale` 0.862 gives 0.94 to 1.06 of the asked rate up to 0.49 rad/s.
- **The cap is per wheel.** Turning on the spot tops out at about 0.70 rad/s, each wheel at
  0.10 to 0.11 m/s (computed with the owner's 0.31 m track). A firmware clamp on forward
  speed alone would not limit an on-the-spot turn. The wheels run slower under the higher
  load of spinning than in straight lines (0.12 to 0.13 m/s), which fits full PWM (or a PWM
  clamp), not a closed-loop speed limit. Left to tell apart: a PWM clamp in the firmware or
  the motor supply. Needs a multimeter: motor terminal voltage against the supply at F015
  and at F037.
- **The firmware has no command timeout of 1.5 s or less.** One `F008|+000#`, then no byte
  for 1.5 s: the base was still driving at about 0.064 m/s (odometry moved 3.2 cm in the
  next 0.5 s; log `*-silence-floor.json`, the strip ahead was 3.89 m clear). Only the Pi's
  watchdog thread stops the base. A dead `run_follow` process (kill -9, OOM), a Pi power
  loss or an unplugged CH340 leaves the base on its last command. A physical cut-off within
  reach is required for every drive.
- A LiDAR return at laser 132 deg, 0.20 m (0.155 m from the turning centre, inside the
  body) stopped the first turn. It appeared in 3 of 726 logged scans, all tonight, always at
  that spot. `lidar.is_body` now drops any return inside the outline (2 cm in from the
  0.40 m square).

From the owner's photos (2026-11-13, 06:09 to 06:11): two DC gear motors with encoders, tyres
marked 6X1.5 (about 0.15 m; at 0.12 m/s that is about 15 rpm, computed), ball casters, a
"21V MAX LITHIUM" tool battery, a UGREEN power bank, a USB hub, a finned DC-DC converter on
the bottom plate (no label visible), and a 22 mm red push button with contact blocks 11 (NC)
and 23 (NO), wired on 11. The controller board is hidden between the motors. This disagrees
with the Task 3 note that one power bank feeds everything: to be confirmed by the owner,
with the converter's output voltage.

The red push button cuts the motor power (owner, 2026-11-13). It is the cut-off to keep within
reach on every drive, because the firmware does not stop by itself.

## Operator identity, review round 2, 2026-11-13

A second review of the merge rule (fuzz of 600 runs with leg geometry, link down) found two
holes; both are fixed and covered by tests that fail on the earlier code:
- a stranger walking up fast merged before 0.3 s in the 0.6 m zone had passed, and was
  forgotten: the age now counts from when the id was first seen;
- a stranger never seen as a separate object (in the operator's LiDAR shadow) had no id to
  vanish: an id that first appears within 0.6 m of the fixed operator and stays is a split.
- `enrol` refuses while the scene is ambiguous (a fresh enrol has no look to check).
Fuzz, rerun here with the reviewer's probes: silent stranger follows 156 of 600 before any
fix, 25 after the first, 2 now; runs that hold or drop the track 81, 294, 363. `COMPANY_MIN`
0.6 s instead of 0.3 s keeps the 2 and cuts a lone walker's false drops in a sine-leg gait
model from 51 to 6 of 96 gaits. The gait model is the reviewer's assumption: Task 5 step 3
(the owner walking a loop) must show the real rate.

Still open from that review: ordinary occlusions (a chair leg, a passer behind the operator,
someone crossing the 4.0 m range edge next to the operator) drop the track, for good while
the link is down; in a queue the camera can never re-lock (anyone behind the operator within
8 deg counts as "two seen"), so the robot holds; a camera re-lock uses the bearing only, and
can lock onto a stranger standing in front of the operator.

## safe_run: stopping the base when a drive program dies, 2026-11-13

`tools/safe_run.py` starts a drive program as its child and sends `S#` however the child
ends. `run_follow.py --drive`, `base_check.py` and `air_check.py` now refuse to open the wheel
port unless `safe_run` started them. Measured on the robot: `base_check --line 0.08` under
`safe_run`, `kill -9` of `base_check` while driving. `safe_run` sent `S#` 0.11 s after the
kill; odometry read every 0.11 s from 0.31 s on moved 11 mm and 16 mm (the tail of the coast)
and then nothing from 0.75 s to 1.74 s. Without `safe_run` the base would have kept driving
(see the timeout result above). It does not help when the Pi loses power or the CH340 comes
out: the red button stays within reach.

Firmware odometry has a 0.01 m step (11 mm after `odom_distance_scale`), so at 0.08 m/s and
10 Hz one read often shows no motion; speed from odometry needs at least about 0.3 s.

## Sensor geometry settled, evening 2026-11-13 (wheels still)

- **LiDAR front.** A box (face 7.9 cm wide) on the robot's centre line, placed by the owner,
  0.564 m from the laser: centre bearing +0.03 deg (median of 30 scans, spread -0.38 to
  +0.51) with `lidar_front_deg` 178.7, i.e. 0.3 mm off the centre line. With 180.3 it would
  have shown at about 1.6 deg, 16 mm off. 178.7 stays, valid if the box was within about
  8 mm of the centre line (the owner: "exactly in the middle"). This agrees with the
  two-bottle check of 2026-11-07 (0.4 deg); the 180.3 estimate rested on the assumption
  that the robot's posts are symmetric. A clear plastic bottle at 1.5 m first was not seen
  at all (the LiDAR passes through clear plastic).
- **Camera pitch: 12.3 deg up.** `sensor_check.py --wall` on a room wall 1.29 m from the lens
  (LiDAR: 1.32 m, flat and square within a few degrees): 30 of 30 frames fitted, median
  12.33 deg, spread 12.32 to 12.36, 74 % of the image on the plane, wall yaw -1.4 deg.
  Written to `params.json` (`camera_pitch_deg` 12.3), which opens the depth obstacle gate.
- Depth layer check on 30 live frames of that scene: no obstacle cell nearer than 1.30 m
  ahead (the bare floor stays clear) at pitch 11.3, 12.33 and 13.3 deg; the wall lands
  1.517 m from the turning centre against 1.495 m by LiDAR (2.2 cm). Not yet checked: a
  0.12 m box and a chair (Task 6 step 3), and a longer floor view.
- Depth layer time on the Pi (`depth_obstacles.points`, 320 x 240, 30 calls): an empty view
  (every pixel past 3 m) 16.6 ms before, 0.7 ms after building the pixel rays once and
  skipping pixels past 3 m; a view with every pixel near 42.4 ms before, 37.2 ms after
  (same points, checked on 50 random frames). The 0.1 s loop has room; striding pixels would
  cut it further if a busy scene ever pushes a cycle near `SLOW_CYCLE`.

## Task 6 step 3: box and chair, 2026-11-13 (wheels still)

Owner placed a low box (about 0.12 m, under the 0.22 m LiDAR plane) on the centre line 1.0 m
ahead of the front edge and a chair 1.5 m ahead, about 0.3 m left. Picture:
`~/hmc-base/logs/scene-20261113-215304.png` (LiDAR points, depth obstacle cells, the obstacle
circles the follower plans with, the robot, a planned path).
- **The box appears in the depth layer and not in the LiDAR**: in the box zone (x 1.05 to
  1.55 m, |y| < 0.25 m from the turning centre) 50 depth cells over 10 frames, 0 LiDAR points
  over 5 scans.
- **The bare floor stays clear**: 0 depth cells between 0.25 and 1.0 m ahead (|y| < 0.45 m).
- The chair shows in both, around x 1.8 m, y +0.55 m.
- `hmc.navigation.plan_path` on the Pi for this scene (231 obstacle circles, goal 2.6 m
  ahead): median 44.7 ms, worst 45.6 ms of 50, under the 150 ms limit, so no planner thread.
  The path goes round the back and along the far side of the wall on the right: the box, the
  chair and that wall, each grown by the robot radius plus 0.1 m, close every gap ahead. Real
  geometry, not a planner fault, but it shows how little room 0.39 m of inflation leaves.

## Task 12 step 1 dry run, and the simulation at the measured speed, 2026-11-13

Dry run, `run_follow.py --seconds 75` with the real Jetson server (YOLO11n + OSNet, wired in
this evening): 750 cycles, locked 98.4 %, longest gap 1.20 s, closest obstacle 0.720 m from
the base centre, 4 "no scan" stops, cycle median 0.068 s, worst 0.109 s (`replay.py`). The
enrolment locked and learned 20 looks, mode follow for 712 cycles. But the track stayed in
front (bearings -30 to 0 deg, 0.98 to 1.17 m, no net turn): the owner was not told when the
lock happened (the ssh output was buffered), so this run shows a standing operator, not a
walk round the robot. Not yet measured: answers per second and round trip with the real
detector (the log has no answer times).

Simulation at the real base's top speed (`hmc.run.run_scenario`, the ladder's last rung:
base real, realistic noise, 10 passers-by, robot sensors, calibrated odometry, operator
1.2 m/s; 6 maps x 10 seeds; script and CSV outside the repo, `sweep.py`):
| Top speed | Car reached (follow_to_car, 300) | Time to the car | Score median | Timeouts |
|---|---|---|---|---|
| 0.12 m/s | 50 of 60 (all but eindhoven2024) | 151 to 225 s | 615 (eindhoven 0) | 60 of 60 |
| 0.2 m/s | 59 of 60 | 96 to 287 s | 1115 on 3 maps, 615 to 915 on 2, 300 on eindhoven | 26 of 60 |
| 0.3 m/s | 60 of 60 | 64 to 192 s | 1115 on 5 maps, 615 on eindhoven | 10 of 60 |
| 0.5 m/s | 60 of 60 | 44 to 119 s | 1115 everywhere | 0 |
At 0.12 m/s the robot can earn the main 615 points on 5 of 6 maps but never the return and
the queue. Asking the operator to walk slower does not help (0.096 m/s is already the asked
pace). The simulation does not model the measured braking (0.064 m/s^2 plus 0.27 s), so these
numbers are optimistic.

## The board, from the owner's photos and the original team's code, 2026-11-14

Read from photos (not checked with a meter): carrier board "@Home Low Level Electronic Board",
"RobotCitiZens 2025", "Designed by NT+KS". Blue Pill (8 MHz crystal, RESET, yellow BOOT0/BOOT1
jumpers, 4-pin SWD header at its end; the chip marking is not readable yet). USB-to-serial is
a purple CH340E module with USB-C (pins GND, +5V, TXD, RXD, DTR, 3V3); the Pi's USB cable goes
there, the Blue Pill's own micro USB is unused. Red L298N module: 6 control wires (likely ENA,
IN1 to IN4, ENB, so PWM on the enable pins), 220 uF 35 V capacitor, 3-pin power terminal.
Green terminals "12V", "EStop", "Motor2", "Motor1"; headers ENCODER1, ENCODER2 (4 pin),
PWMEXP2, I2CExpansion, ADCExpansion; LEDs on PC14, PC15, 3V3. Motors marked "X" and "Y" by
hand, magnetic encoder discs on the back; finned DC-DC converter next to the tool battery.

The board's QR code decodes to https://www.robotcitizens.org/home (404); the site links to
https://gitlab.com/robotcitizens. Public there: `workshop/robotcitizens_athome_robot/
software-team/` (rct_ahr_driver, rct_ahr_description, rct_ahr_bringup, rct_ahr_wiki; robot
name CheeseCake-01). No firmware or hardware repository is public. From that code (their
configuration, not measured):
- driver.py speaks our protocol and adds `R#` (reset odometry); port 115200 8E1 with
  rtscts and dsrdtr on (ours has both off and works); sends at 30 Hz; MAX_VEL_X 0.25,
  MAX_VEL_TH 0.5. udev: CH340 1a86:7523 -> /dev/rct_ahr_robot.
- URDF: wheels at +-0.1295 m, so a 0.259 m wheel separation. bringup: LiDAR 0.27 m up,
  turned 180 deg (agrees with our lidar_front_deg 178.7), placed at the centre (the photo
  shows it at the front edge; our lidar_x is measured).
- velocity_smoother: speed_lim_v 0.2, accel_lim_v 0.4, speed_lim_w 1.0.

From the web (third-party datasheet copies, see sources in the chat): the L298 loses 1.8 V
typical, 3.2 V worst at 1 A and up to 4.9 V at 2 A (source plus sink), so a 12 V supply leaves
the motors about 8.8 to 10.2 V at 1 A. Blue Pill clones (CKS32, APM32, GD32) are common and
can carry an STM32 marking; a CKS32 answers SWD with id 0x2ba01477 (OpenOCD needs
`set CPUTAPID 0x2ba01477`). The STM32F103 ROM bootloader listens on USART1 (PA9/PA10) at 8E1
with BOOT0 high, the same framing as this protocol: if the CH340 is wired to PA9/PA10 the
Pi could flash with stm32flash over the existing cable. Not checked: which pins the CH340
is wired to, and whether DTR reaches RESET.
Close-up photos later the same day: the chip top shows no readable marking even with contrast
enhancement (blank or worn), so the chip must be identified electronically (SWD IDCODE, or
the ROM bootloader's GET_ID over serial). Level shifter: 4 channels LV1-4/HV1-4. CH340E
module "BTE17-06B" has a solder jumper marked 5 (logic level, read from the photo only).
Pi unreachable over ssh at the time (both addresses timed out).
A sharper photo (same day) shows the marking after all: ST logo, "STM32F 103C8T6" and lot and
date lines (not fully readable). The marking says genuine ST STM32F103C8T6; clones can carry
the same marking, so the SWD or bootloader id is still to be read before flashing.

## Old firmware backed up over the serial bootloader, 2026-11-14

Owner moved BOOT0 to 1 (BOOT1 left at 0) and pressed RESET; the motors were cut with the red
button. `stm32flash -b 115200 /dev/rct_ahr_robot` (installed by the owner) answered, so the
CH340 is wired to USART1 (PA9/PA10) and the Pi can flash without an ST-Link. Bootloader
version 0x22, option bytes 0x00 0x00, Device ID 0x0410 (STM32F10xxx medium density), flash
read 128 KiB with no read protection. Read twice, both
SHA-256 5924421ea558a965d5519a98514ddeb8c92a08ea88528dca0b71876d4b849ef9. 29 pages of 1 KiB
used (0x08000000 to 0x08007400), the rest erased; stack 0x20005000, reset 0x08004c6d, so the
application starts at 0x08000000 with no separate bootloader. Copies: Pi
`~/firmware-backup/old-1.bin` and `old-2.bin`; here
`base/firmware-original/stm32-old-firmware-2026-11-14.bin` (not committed).
