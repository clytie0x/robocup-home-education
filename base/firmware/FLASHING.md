# Flash, check and roll back the base firmware

**Only with the owner at the robot and the owner's yes.** Wheels lifted off the floor, red
button within reach. Nothing here has been run yet; the commands are written from the
stm32flash manual and the 2026-11-14 bootloader read, not tested with this image.

The Pi flashes over the robot's own cable: the CH340E is wired to USART1 (PA9/PA10), where
the STM32's ROM bootloader listens at 8E1. No ST-Link is needed. DTR is not known to reach
RESET, so the owner moves the jumper and presses RESET by hand.

## 0. Before

1. On the laptop: `make test` and `make firmware` in `base/firmware`. Note the SHA-256 that
   `make firmware` prints for `build/base.bin`.
2. Copy `build/base.bin` and `tools/bringup.py` to the Pi, into `~/firmware-new/`. On the
   Pi, `sha256sum ~/firmware-new/base.bin` must print the same hash.
3. Check that the backup is still there and whole:
   `sha256sum ~/firmware-backup/old-1.bin` must print
   `5924421ea558a965d5519a98514ddeb8c92a08ea88528dca0b71876d4b849ef9`.
4. Nothing may hold the port: stop ROS and every drive program. `fuser /dev/rct_ahr_robot`
   must print nothing.

## 1. Flash

1. Owner: BOOT0 jumper to 1, BOOT1 stays 0, press RESET. The motors stay off while the
   bootloader runs.
2. Probe: `stm32flash -b 115200 /dev/rct_ahr_robot`. Expect device ID 0x0410 and
   bootloader version 0x22, as on 2026-11-14. If it does not answer, press RESET again
   and retry; do not go on without this.
3. Write and verify: `stm32flash -b 115200 -w ~/firmware-new/base.bin -v -S 0x08000000 /dev/rct_ahr_robot`.
   `-v` reads every page back after writing it. It must end with "Done." and no error.
4. Read back the whole image as a second check, and compare:
   `stm32flash -b 115200 -r ~/firmware-new/readback.bin -S 0x08000000:$(stat -c %s ~/firmware-new/base.bin) /dev/rct_ahr_robot`
   then `cmp ~/firmware-new/base.bin ~/firmware-new/readback.bin` must print nothing.
5. Owner: BOOT0 jumper back to 0, press RESET. The Blue Pill's LED (PC13) now blinks once
   a second: the control loop runs.

If step 3 or 4 fails, do not reset into the new image: go to section 3 (rollback) with
BOOT0 still at 1.

## 2. Bring-up test (wheels lifted, red button within reach)

`python tools/safe_run.py .venv/bin/python ~/firmware-new/bringup.py` from `~/hmc-base`
(safe_run sends S# however the test ends). The test asks the owner twice to turn a wheel
by hand and once to confirm by eye. It checks:

| Check | Pass rule |
|---|---|
| replies | `O#` and `P#` replies have the right format, PWM 0 after `S#` |
| left encoder by hand | left counts rise by more than a sixteenth of a turn, right stays nearly still |
| right encoder by hand | the same, the other way round |
| both wheels forward | `F005\|+000#` for 1.5 s: both counts rise by more than 1000; the owner confirms both wheels roll forward |
| turn left on the spot | `F000\|+030#`: left counts fall, right counts rise |
| S# stops | after `S#`, fewer than 50 counts in 0.3 s and PWM 0 |
| 0.3 s timeout stops | after the Pi goes silent, the same |

All must pass. If a wheel turns the wrong way, or an encoder counts the wrong way, the
pin map or a sign in `config.h` is wrong: roll back or fix and reflash; do not drive on
the floor.

## 3. Rollback to the old firmware

1. Owner: BOOT0 to 1, BOOT1 0, RESET.
2. `stm32flash -b 115200 -w ~/firmware-backup/old-1.bin -v -S 0x08000000 /dev/rct_ahr_robot`
   (writes all 128 KiB that were read, verified page by page).
3. Owner: BOOT0 to 0, RESET.
4. Check: `O#` answers in the old format, and one `F008|+000#` with the wheels lifted
   turns both wheels (send `S#` right after; the old firmware has no timeout).
5. Put the old scales back in `base/params.json` if they were changed (section 4).

## 4. After a good bring-up: the Pi's scales

The old firmware turned 1/0.862 too fast and read its odometry short, so the Pi corrects
it. The new firmware drives and reports true m/s, rad/s, m and rad, so **in the same step**:

- `base/params.json`: `linear_cmd_scale` 1.0, `angular_cmd_scale` 1.0,
  `odom_distance_scale` 1.0, `odom_yaw_scale` 1.0 (today 1.0, 0.862, 0.882, 0.862).
- The ROS driver in `~/catkin_ws` carries its own copies of these scales
  (originals in `~/backup-driver-20261113/`): the same change there.

With the old scales the base would turn at 0.862 of the asked rate and the odometry
would read 12 to 14 % long. Then repeat calibration steps 4.2 and 4.3 of the design on the
floor: they check `METERS_PER_COUNT` and `TRACK_M` in `config.h`.
