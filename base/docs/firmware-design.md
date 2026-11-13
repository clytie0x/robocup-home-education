# New firmware for the wheeled base (STM32), design draft

Status: **draft, not approved by the owner.** Nothing has been written to the board, and
nothing may be flashed until the owner approves this document and is at the robot.

Why: the previous owner left without handing over the firmware source. The current
firmware caps the base at about 0.12 m/s whatever is commanded (`F015` and `F037` both
give 0.12 to 0.13 m/s), turns 1 / 0.862 times faster than asked, and its odometry reads
short (the Pi corrects it with `odom_distance_scale` 0.882 and `odom_yaw_scale` 0.862).
Source: `hardware-assessment-2026-11-13.md`, "Task 2".

## 0. What is known and what blocks the work

| Item | State | Source |
|---|---|---|
| USB link | CH340 (`1a86:7523`), `/dev/rct_ahr_robot`, 115200 baud, **even parity** (8E1) | lsusb 2026-11-07; `base/pi/wheels.py` opens it with `PARITY_EVEN` |
| MCU | "STM32" from the udev rule name `00-stm32.rules` in `~/catkin_ws` (a `00-teensy.rules` is there too). Part number unknown | 2026-11-07 inventory, inferred |
| Board, motor driver, motors, encoders | Unknown | Needs photos (blocker 1) |
| Which MCU UART the CH340 is wired to | Unknown | Needs photos or a bootloader probe (blocker 2) |
| Flash method | Unknown: ST-Link on SWD, USB DFU, or the serial bootloader (BOOT0 high + reset) | Needs photos (blocker 3) |
| Read protection (RDP) | Unknown | Read when a programmer is connected (blocker 4) |
| Why 0.12 m/s | Unknown: firmware clamp, PWM limit, motor supply voltage, or the motors' own top speed | Section 5 test (blocker 5) |
| Wheel diameter, encoder counts per wheel turn | Unknown | Measure (section 4) |
| Track (wheel to wheel, centre of contact) | 0.31 m | Owner |
| `R#` (odometry reset) | In the task brief. Not used by any Pi code in this repo; check `~/catkin_ws/.../driver.py` | Not verified |

Blockers 1 to 3 need the owner. The Pi was offline (Tailscale "last seen 52 m ago", no
ping on the LAN) when this draft was written, so no USB or bootloader read was possible.

## 1. Back up the current firmware before anything else

Rule: no write to the board until a full read-out exists on the laptop and the Pi, with a
SHA-256 recorded here.

1. Connect a programmer (ST-Link on SWD if the board has an SWD header; otherwise the
   serial bootloader through BOOT0 on the UART that the CH340 uses, if that is USART1 or
   another bootloader UART for this part).
2. Read the chip ID and the option bytes first. **If RDP is level 1 the flash cannot be
   read.** Do not lower RDP: going from level 1 to 0 erases the whole flash. In that case
   stop and tell the owner: there is no way back to the old firmware, and the decision to
   replace it is theirs.
3. If RDP is level 0: read the whole flash (size from the chip ID) and the option bytes,
   twice, and compare the two reads byte for byte. Tools: `st-flash read` or
   `STM32_Programmer_CLI -r` for ST-Link, `stm32flash -r` for the serial bootloader.
4. Store as `stm32-backup-<date>/flash.bin`, `option-bytes.bin`, `chip-id.txt`,
   `SHA256SUMS` on the Pi (next to `~/archive-xohm-20261113/`) and on the laptop outside the
   repository (it is someone else's binary). Record the hashes in this section.
5. Restore check, only with the owner's approval: flash the backup back, run `O#` and one
   `F008|+000#` line with the wheels off the floor, confirm the old behaviour.

## 2. Serial protocol (kept, so `base/pi/wheels.py` keeps working)

115200 baud, 8E1. Every message ends with `#`. Unknown or malformed messages are ignored
and do not reset the timeout.

| Message | Meaning | Change from today |
|---|---|---|
| `F<vvv>\|<s><www>#` | Forward `vvv/100` m/s, turn `s www/100` rad/s (left positive) | Same text. Now exact: the base drives the asked speed in true m/s and rad/s |
| `B<vvv>\|<s><www>#` | Same, backward | Same |
| `S#` | Stop: ramp to zero at the brake deceleration, then hold zero | Same text; stopping behaviour defined (section 3) |
| `O#` | Reply `Ox:+000.00,y:+000.00,th:+000.00#` (m, m, rad, from the encoders) | Same format. Values now true, so the Pi scales become 1.0 |
| `R#` | Reset odometry to zero | Same (to be confirmed against the old driver) |
| `M<vvv>#` | New: set the top speed to `vvv/100` m/s, never above the compiled maximum. Not kept over a power cycle | New, optional. Old drivers never send it |
| `E#` | New: reply `Eleft:+0000000,right:+0000000,pwm:+000,+000,flags:00#` (raw counts, PWM in percent, fault flags) | New, for calibration and fault finding |

**Coordination point:** the day the new firmware goes in, `base/params.json` must change
in the same step: `linear_cmd_scale` 1.0, `angular_cmd_scale` 1.0, `odom_distance_scale`
1.0, `odom_yaw_scale` 1.0, `max_speed` to the new measured top speed. With the old scales
the base would turn at 0.862 of the asked rate and the odometry would read 13 % long.
The ROS driver in `~/catkin_ws` carries its own copies of these scales (changed
2026-11-13, originals in `~/backup-driver-20261113/`), so it needs the same change.

## 3. Control

- **Loop:** fixed 200 Hz timer interrupt (5 ms). Encoders read from hardware timers in
  encoder mode (no counts lost to interrupt load), speed per wheel from the count change
  over the last 4 ticks (20 ms window) to smooth the quantisation.
- **Body to wheels:** `v_left = v - w * track / 2`, `v_right = v + w * track / 2`,
  track 0.31 m. If one wheel would exceed the top speed, both are scaled down by the same
  factor, so the path curvature stays what was asked (today the Pi clamps `v` and `w`
  separately).
- **Ramp:** the target speed moves toward the command at most `accel` (default 0.25 m/s²,
  the value in `params.json`) per second, and at most `brake` (default 0.5 m/s²) when
  slowing. The Pi's brain already ramps; the firmware ramp is the floor that protects the
  gearbox and stops wheel slip if the Pi sends a step.
- **Per-wheel PI with feed-forward:** `pwm = kff * target + kp * error + ki * integral`.
  `kff` from the measured speed per PWM percent (section 5), so the PI only corrects.
  Integral clamped and frozen while the PWM is saturated (anti-windup). Gains tuned with
  the wheels off the floor, then on the floor, with step logs from `E#`.
- **Odometry:** from encoder count changes every tick, midpoint integration
  (`th += dth`, `x += ds * cos(th - dth/2)`). Float32 on the MCU is enough for a 10-minute
  run (errors far below the scan-match error). `O#` reports the latest pose.

## 4. Calibration (owner at the robot)

1. **Counts per wheel turn:** wheels off the floor, mark a wheel, turn it 10 times by hand,
   read `E#` before and after.
2. **Wheel diameter:** drive a straight line on the floor and compare the encoder distance
   with the LiDAR scan-match distance (`base/pi/tools/base_check.py` already does this).
3. **Track:** start with 0.31 m, then two 360° turns in place against scan matching; adjust
   the track until odometry yaw matches within 1 %.
4. Store the three numbers in `config.h` (section 6), rebuild, flash, repeat 2 and 3 once.

## 5. First find out if the base can go faster at all

Closed-loop control cannot exceed what the motors give at full PWM. Before writing the
control loop:

1. Read the motor label (rated voltage, no-load rpm, gear ratio) from the photos, and the
   motor supply voltage at the driver while driving (multimeter).
2. With the new firmware's test build: wheels off the floor, PWM 25, 50, 75, 100 % for
   2 s each, record wheel speed from `E#`.

If 100 % PWM off the floor is only a little above 0.12 m/s, the cap is the motors or the
power bank's output voltage, not the firmware, and new firmware buys accuracy and safety,
not speed. The owner should know this before spending time on the rest.

## 6. Safety

- **Command timeout:** if no valid `F`/`B` arrives for 0.3 s (the Pi resends every 40 ms,
  `wheels.TIMEOUT` is also 0.3 s), ramp to zero at `brake`. Valid `O#`/`E#` do not count.
- **Boot:** motors held at zero until the first valid drive command after power-up.
- **Hardware watchdog (IWDG):** 100 ms. If the control loop stops running, the MCU
  resets, and the boot rule holds the motors.
- **Encoder fault:** PWM above 30 % for 0.3 s with no counts on that wheel, or the wheel
  turning the wrong way, stops both motors and sets a flag in `E#`. Cleared by `S#`.
- **Speed limit:** `config.h` `MAX_SPEED` is the hard cap; `M###` can only lower it.
- **Brake vs coast:** stop with the ramp, then the driver's brake mode (if the driver
  supports it) at zero speed. Today `S#` coasts 5 to 8.4 cm from 0.12 m/s.

## 7. Code layout and tools

```
base/firmware/
  README.md        build, flash, restore commands for the identified board
  config.h         wheel diameter, counts per turn, track, MAX_SPEED, accel, brake, gains
  main.c           init, 200 Hz loop, watchdog
  protocol.c/.h    parse F/B/S/O/R/M/E, reply formats (unit-tested on the laptop)
  control.c/.h     body to wheels, ramp, PI (unit-tested on the laptop)
  odom.c/.h        integration (unit-tested on the laptop)
  board.c/.h       the only board-specific file: timers, PWM pins, UART
  tests/           host tests in plain C, built with gcc on the laptop
```

Build with `arm-none-eabi-gcc` and the vendor HAL or LL drivers for the identified part
(the exact choice waits for the part number). `protocol`, `control` and `odom` have no
hardware calls, so their tests run on the laptop with the same packets that
`base/pi/tests/test_wheels.py` uses.

## 8. Order of work, each step needs the owner's yes

1. Photos and the part number (blocker 1 to 3).
2. Back up the flash (section 1). Stop here if RDP is level 1 and ask.
3. Approve this design.
4. Test build: protocol + `E#` + open-loop PWM only, no control (section 5). Flash with the
   owner at the robot, wheels off the floor.
5. Decide whether the speed goal is reachable (section 5).
6. Full build: control, odometry, safety. Calibrate (section 4). Update `params.json` and
   the ROS driver scales in the same step (section 2).
7. Re-run the speed ladder and turn tests from the plan with the new firmware.
