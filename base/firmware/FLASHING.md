# Flash, update and roll back the base firmware

**Only with the owner's yes.** Wheels lifted off the floor, red button within reach. None
of these commands has been run with this image yet; they are written from the stm32flash
source and manual, ST AN3155 and the 2026-11-14 bootloader read.

The Pi writes the chip over the robot's own cable: the CH340E is wired to USART1
(PA9/PA10), where the STM32's ROM bootloader listens at 8E1 (stm32flash's default). The
Pi cannot reset the chip (DTR and RTS do not reach RESET, measured 2026-11-14).

There are two kinds of write:

| When | File | Where | How |
|---|---|---|---|
| Once, with the BOOT0 jumper (section 1) | `build/full.bin` (stage1 + application) | 0x08000000 | stm32flash by hand |
| Every later update, no jumper (section 4) | `build/app.bin` (application only) | 0x08001000 | `tools/update.py` |

stage1 (0x08000000 to 0x08000FFF) is what makes the jumper-free path work, and only the
jumper can put it back. **After section 1, never run stm32flash with `-S 0x08000000` or
without `-S` for a write**: stm32flash then erases from page 0 to the end (a mass erase),
stage1 is gone, and the next change needs the jumper again. `update.py` always writes at
0x08001000 with the image's length, so it erases only the pages it writes.

## 0. Before

1. On the laptop, on a committed tree (otherwise the build id ends in `-dirty`):
   `make test` and `make firmware` in `base/firmware`. Note the two SHA-256 lines and the
   `Vversion:...` line that `make firmware` prints.
2. Copy `build/full.bin`, `build/app.bin`, `tools/update.py`, `tools/image.py` and
   `tools/bringup.py` to the Pi, into `~/firmware-new/`. On the Pi,
   `sha256sum ~/firmware-new/*.bin` must print the same hashes.
3. The backup is still there and whole: `sha256sum ~/firmware-backup/old-1.bin` must print
   `5924421ea558a965d5519a98514ddeb8c92a08ea88528dca0b71876d4b849ef9`.
4. Nothing holds the port: stop ROS and every drive program. `fuser /dev/rct_ahr_robot`
   must print nothing.

## 1. The one flash with the BOOT0 jumper (deck open)

1. Owner: BOOT0 jumper to 1, BOOT1 stays 0, press RESET.
2. Probe: `stm32flash -b 115200 /dev/rct_ahr_robot`. Expect device ID 0x0410 and
   bootloader version 0x22, as on 2026-11-14. If it does not answer, press RESET again and
   retry; do not go on without this.
3. Write and verify (this one erases the whole flash, on purpose):
   `stm32flash -b 115200 -w ~/firmware-new/full.bin -v -S 0x08000000 /dev/rct_ahr_robot`
   It must end with "Done." and no error.
4. Read back and compare:
   `stm32flash -b 115200 -r ~/firmware-new/readback-full.bin -S 0x08000000:$(stat -c %s ~/firmware-new/full.bin) /dev/rct_ahr_robot`
   then `cmp ~/firmware-new/full.bin ~/firmware-new/readback-full.bin` must print nothing.
5. Owner: BOOT0 jumper back to 0, press RESET. The Blue Pill's LED (PC13) blinks once a
   second: stage1 accepted the application and the control loop runs.

If step 3 or 4 fails, do not reset into the new image: roll back (section 5) with BOOT0
still at 1. **Keep the deck open until section 3 has passed.**

## 2. Bring-up test (wheels lifted, red button within reach)

`python tools/safe_run.py .venv/bin/python ~/firmware-new/bringup.py` from `~/hmc-base`
(safe_run sends S# however the test ends). The test asks the owner twice to turn a wheel
by hand (it sets the motors to coast for that) and once to confirm by eye:

| Check | Pass rule |
|---|---|
| replies | `O#`, `P#` and `V#` have the right format, PWM 0 after `S#` |
| left encoder by hand | left counts rise by more than a sixteenth of a turn, right stays nearly still |
| right encoder by hand | the same, the other way round |
| both wheels forward | `F005\|+000#` for 1.5 s: both counts rise by more than 1000; the owner confirms both wheels roll forward |
| turn left on the spot | `F000\|+030#`: left counts fall, right counts rise |
| U# refused while driving | `U#` right after a drive packet answers `U!busy#` |
| S# stops | after `S#`, fewer than 50 counts in 0.3 s and PWM 0 |
| 0.3 s timeout stops | after the Pi goes silent, the same |

All must pass. A wheel or an encoder the wrong way means the pin map or a sign in
`config.h` is wrong: fix, rebuild, and update with section 4 (no jumper needed for that,
once section 3 has passed).

## 3. Prove the jumper-free path before the deck closes (required)

The ROM bootloader jump, the backup register and the Go command can only be proven on the
board. Do it now, while the jumper can still be reached:

1. `python ~/firmware-new/update.py ~/firmware-new/app.bin` (rewrites the same application
   through `U#`, the ROM bootloader and Go). It must end with
   `OK: the base runs Vversion:1,build:<hash>#`, the hash from section 0.
2. Power the base off and on (not RESET), then run the `replies` step again, or
   `python ~/firmware-new/update.py ~/firmware-new/app.bin` once more: stage1 must start
   the application after a power-up.

If step 1 fails, the script says what state the chip is in. If the chip cannot be brought
back with `update.py --rescue` or a power cycle, use the jumper: section 1 again, or the
rollback in section 5. Close the deck only after both steps pass.

## 4. Every later update (no jumper)

1. Laptop: commit, `make test`, `make firmware`. Copy `build/app.bin`, `tools/update.py`
   and `tools/image.py` to `~/firmware-new/` and compare the SHA-256 on the Pi.
2. Pi: stop ROS and every drive program, wheels standing still (lifted or on the floor).
3. `python ~/firmware-new/update.py ~/firmware-new/app.bin`

What `update.py` does, and the same by hand if it is ever not available:

| Step | By hand |
|---|---|
| check the file (header, length, CRC-32) | `python ~/firmware-new/image.py check ~/firmware-new/app.bin` |
| no process holds the port | `fuser /dev/rct_ahr_robot` prints nothing |
| `V#`, three `S#`, wait 0.5 s, `U#` must answer `Uok#` | any serial terminal at 115200 8E1 |
| write and verify | `stm32flash -b 115200 -w ~/firmware-new/app.bin -v -S 0x08001000:$(stat -c %s ~/firmware-new/app.bin) /dev/rct_ahr_robot` |
| read back and compare | `stm32flash -b 115200 -r ~/firmware-new/readback.bin -S 0x08001000:$(stat -c %s ~/firmware-new/app.bin) /dev/rct_ahr_robot` then `cmp ~/firmware-new/app.bin ~/firmware-new/readback.bin` |
| start it (Go, through stage1) | `stm32flash -b 115200 -g 0x08000000 /dev/rct_ahr_robot` |
| check the version | `V#` must give the `Vversion:...` line `make firmware` printed |

When something fails, `update.py` prints the chip's state:

- Before `U#` was accepted: nothing was written, the old application still runs.
- `U#` answered `U!busy#`: a drive command is in force or a wheel turns. If the wheels
  really stand still and an encoder counts by itself, `--force` sends `Uforce#`, which
  skips only the wheel-speed check.
- After `U#`, if stm32flash cannot reach the bootloader: nothing was written; a power cycle
  starts the unchanged application.
- After the write started: the chip waits in the ROM bootloader (motors off). Run
  `update.py --rescue app.bin` (it skips `V#`/`U#`). A power cycle lands in the
  bootloader again if the application is broken (stage1 checks its CRC-32), or starts it
  if it is whole.
- Go failed after a good write: power the base off and on.

stage1 also sends the chip to the ROM bootloader by itself after three watchdog or fault
restarts in a row (each before the loop ran 2 s): a new application that crashes at once
can be replaced with `update.py --rescue`.

**Go from the ROM bootloader:** stm32flash's `-g <address>` sends the bootloader's Go
command with that address (stm32flash source, `stm32_go`); per AN3155 the bootloader then
resets the peripherals it used, loads the stack pointer from the address and jumps to the
reset vector at address + 4. `-g 0x08000000` therefore runs stage1, which checks and
starts the application. This is the usual way to leave the bootloader on an F103 with
stm32flash, but it is proven on this board only by section 3. The application does not
rely on the bootloader leaving the chip clean: it sets its own vector table, unmasks
interrupts, and switches the clock to the internal oscillator before it starts the PLL.

## 5. Rollback

- **To an earlier build of the new firmware:** keep its `app.bin` and run `update.py`
  with it (section 4). No jumper.
- **To the old firmware (`old-1.bin`): jumper only.** The old firmware has no `U#`, and
  `old-1.bin` covers 0x08000000, so writing it erases stage1. From then on every change
  needs the jumper again.
  1. Owner: BOOT0 to 1, BOOT1 0, RESET (the deck must be opened).
  2. `stm32flash -b 115200 -w ~/firmware-backup/old-1.bin -v -S 0x08000000 /dev/rct_ahr_robot`
     (writes all 128 KiB that were read, verified page by page).
  3. Owner: BOOT0 to 0, RESET.
  4. Check: `O#` answers in the old format, and one `F008|+000#` with the wheels lifted
     turns both wheels (send `S#` right after; the old firmware has no timeout).
  5. Put the old scales back in `base/params.json` if they were changed (section 6).

## 6. After a good bring-up: the Pi's scales

The old firmware turned 1/0.862 too fast and read its odometry short, so the Pi corrects
it. The new firmware drives and reports true m/s, rad/s, m and rad, so **in the same step**:

- `base/params.json`: `linear_cmd_scale` 1.0, `angular_cmd_scale` 1.0,
  `odom_distance_scale` 1.0, `odom_yaw_scale` 1.0 (today 1.0, 0.862, 0.882, 0.862).
- The ROS driver in `~/catkin_ws` carries its own copies of these scales
  (originals in `~/backup-driver-20261113/`): the same change there.

With the old scales the base would turn at 0.862 of the asked rate and the odometry
would read 12 to 14 % long. Then repeat calibration steps 4.2 and 4.3 of the design on the
floor, and set the results at run time first (`Kmpc_left=`, `Kmpc_right=`, `Ktrack=`, see
README "Protocol"); once they are right, put them in `config.h` and update (section 4).
