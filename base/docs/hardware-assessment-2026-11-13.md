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
