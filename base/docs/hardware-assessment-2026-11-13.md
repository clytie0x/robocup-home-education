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
