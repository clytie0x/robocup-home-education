# Base firmware (STM32F103C8T6)

New firmware for the wheeled base, following the draft in
[`../docs/firmware-design.md`](../docs/firmware-design.md). **Flashing approved by the
owner on 2026-11-14; not flashed yet.** The chip is an STM32F103C8T6 on a Blue Pill
driving an L298N. The pin map below was read from the old firmware's binary. Flash once
with the jumper, update without it, roll back: [`FLASHING.md`](FLASHING.md).

- Lifecycle state: `unconfirmed` (draft until the owner approves the design)
- Entry point on the laptop: `make test` in this folder
- Entry point on the board: `make firmware` gives `build/full.bin` (stage1 + application,
  for the one jumper flash) and `build/app.bin` (application only, for every update with
  `tools/update.py`); the application starts at `main.c`, stage1 at `stage1.c`

## What is here

| File | Job |
|---|---|
| `config.h` | Every constant and the defaults of the run-time values. `MEASURE ON THE BOARD` marks placeholders |
| `params.c/.h` | Run-time tuning values (`K<name>=<value>#`) with hard bounds; RAM only, defaults from `config.h` |
| `boot.c/.h` | stage1's decisions (application or ROM bootloader), the application check (header, CRC-32), the flash layout |
| `stage1.c`, `stage1.ld` | stage1: the first 4 KiB of flash, written once with the jumper, never by an update |
| `board.h` | The only hardware interface (time, tick, encoder counts, PWM, UART, watchdog) |
| `board.c` | `board.h` for this board: clock, TIM3 PWM, TIM2/TIM4 encoders, USART1 rings, watchdog, LED |
| `stm32f103.h` | The few registers `board.c` uses, with reference-manual sections |
| `startup.c`, `stm32f103c8.ld` | Application vector table, header, reset handler, memory layout (0x08001000 to the end of 64 KiB, 20 KiB RAM) |
| `protocol.c/.h` | Parse `F B S O R M E P U V D K`, format the replies |
| `control.c/.h` | Body speed to wheel speeds, accel and brake ramps, PI wheel-speed loop with anti-wind-up, speed from counts |
| `odom.c/.h` | Pose from encoder counts, midpoint rule |
| `safety.c/.h` | 0.3 s command timeout, encoder fault |
| `app.c/.h` | Ties them together: one control tick, one UART poll |
| `main.c` | Board loop: wait for the 5 ms tick, poll, tick, kick the watchdog |
| `tests/` | Plain-C tests; `board_stub.c` stands in for the board, `test_app.c` runs the whole firmware against a model motor |
| `tools/bringup.py` | Bring-up test on the Pi after flashing (wheels lifted); `test_bringup.py` checks it on the laptop |
| `tools/update.py` | Update from the Pi without the jumper: `U#`, stm32flash write, verify, read back, Go, `V#` |
| `tools/image.py` | Seal `app.bin` (length, CRC-32), join it with stage1 into `full.bin`, check an image; `test_update.py` checks both |

## Build and test

Needs `gcc` and `make` for the tests (MinGW gcc 14.2 and GNU make were used on Windows),
and `arm-none-eabi-gcc` (Arm GNU Toolchain) and `python` for the board. Run `make` from a
POSIX shell (Git Bash): the firmware rules use `mkdir -p`.

```
cd base/firmware
make test
make firmware
python -m unittest discover -s tools
```

The last line of the C test run is `<n> checks, <m> failed`; `make` fails if any check
fails. `make firmware` prints the sizes, the version string `V#` will report and the
SHA-256 of `build/app.bin` and `build/full.bin`. On the laptop the toolchain came from
`winget install Arm.GnuArmEmbeddedToolchain` (14.2.Rel1) and lives in
`C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin`; put that on `PATH`
if a new shell does not find `arm-none-eabi-gcc`. The build id inside the image is the
git hash of the sources (with `-dirty` for uncommitted changes), so build from a commit;
the same commit gives the same hashes.

## Flash layout

| Address | What | Written |
|---|---|---|
| 0x08000000 to 0x08000FFF | stage1 | once, with the BOOT0 jumper (`full.bin`) |
| 0x08001000 | application: vector table, header at +0x100 (magic, length), code, CRC-32 right after the image | by every update (`app.bin`, `tools/update.py`) |

On every start stage1 runs the application, or the ST ROM bootloader (which waits for
stm32flash on the robot's cable) when `U#` left a mark in backup register 1, when the
application restarted from a fault or the watchdog three times in a row before running
2 s, or when the application's header or CRC-32 is wrong. stage1 drives the L298N pins
low first, uses no RAM variables and no interrupts, and does not change the clock.

## Pin map, read from the old firmware

Source: the 2026-11-14 backup (SHA-256 `5924421e...49ef9`), disassembled with capstone.
The old firmware is an Arduino sketch on the STM32duino core (HAL underneath): its
`pinMode`, `digitalWrite`, `HardwareTimer` and HAL encoder calls name the pins through the
core's Blue Pill tables, which are in the image too.

| Signal | Pin | How sure | Evidence |
|---|---|---|---|
| Pi link | USART1, PA9 TX, PA10 RX, 115200 8E1 | high | `Serial.begin(115200, 0x26 = SERIAL_8E1)`; the bootloader answered on this cable |
| Right motor PWM (L298N enable) | PA6, TIM3_CH1 | high | `HardwareTimer(TIM3)`, channel from the core's PWM pin table for A6 |
| Left motor PWM | PA7, TIM3_CH2 | high | the same for A7 |
| Right motor direction | PB13 high = forward, PB12 high = backward | high | `digitalWrite` pair driven by the right wheel's PI output sign |
| Left motor direction | PB15 high = forward, PB14 high = backward | high | the same for the left wheel |
| Right encoder | PB6/PB7, TIM4 encoder mode (x4) | high | HAL encoder init on TIM4, counts feed the right wheel's speed |
| Left encoder | PA0/PA1, TIM2 encoder mode (x4) | high | the same on TIM2, for the left wheel |
| Encoder direction | right counts up going forward, left counts down | medium-high | the old odometry negates TIM2 and not TIM4; checked by the bring-up test |
| Which side is "right" | as above | high | the wheel fed `v + w * track / 2` and the yaw `(R - L) / track` |
| LEDs | PC13 (Blue Pill), PC14, PC15 (blinked at 1 Hz from TIM1) | high | `pinMode` and the TIM1 callback |
| PB0, PB1 | set as outputs, never written | medium | no `digitalWrite` found for them; the new firmware leaves them as inputs |

Old firmware facts: PWM 100 Hz, 12-bit duty, a deadband of 280 of 4095; a PI loop on
wheel speed in rad/s with doubles; a 10 ms `delay()` loop; odometry with wheel radius
0.0762 m, 0.0872222/596 rad per count, track 0.31 m; replies end with `#` and CR LF;
`th` "wraps" only beyond 360 rad (a degrees mix-up), so in practice it never wraps.

## Protocol

Old messages, same text, so `base/pi/wheels.py` works unchanged:

| Message | Meaning |
|---|---|
| `F<vvv>\|<s><www>#` | Forward `vvv/100` m/s, turn `s www/100` rad/s, left positive |
| `B<vvv>\|<s><www>#` | Backward, same turn convention |
| `S#` | Ramped stop at `BRAKE_MPS2`, then zero PWM. Also clears an encoder fault |
| `O#` | `Ox:+000.00,y:+000.00,th:+000.00#` (m, m, rad; `th` is not wrapped) |
| `R#` | Odometry back to zero |

New:

| Message | Reply or meaning |
|---|---|
| `P#` | `Px:+0000000,y:+0000000,th:+0000000,t:0000000000#`: mm, mm, milliradians, and the board's millisecond clock when that pose was computed |
| `E#` | `Eleft:+0000000,right:+0000000,pwm:+000,+000,flags:00#`: running counts (last 7 digits), PWM %, flags (01 left encoder fault, 02 right, 04 no drive command in the last 0.3 s) |
| `M<vvv>#` | Lower the per-wheel top speed to `vvv/100` m/s, never above `vmax` (below). Lost at power-off |
| `U#` | Update: answers `Uok#` and restarts into the ST ROM bootloader, only with no drive command in force, ramp and PWM at zero and both wheels still; else `U!busy#` |
| `Uforce#` | The same without the wheel-speed check, for an encoder that counts by itself (still needs no drive command and zero PWM) |
| `V#` | `Vversion:1,build:<git hash>#` |
| `D#` | `Dtarget:+0000,+0000,speed:+0000,+0000,pwm:+000,+000#`: per wheel (left, right) the ramped target and the measured speed in mm/s, and the PWM in % |
| `K<name>=<value>#` | Set a run-time value; answers `K<name>=<value in force>#` (6 decimals), or `K!range#` (out of bounds, nothing changed), `K!unknown#`, `K!syntax#` |
| `K<name>?#` | Read one: `K<name>=<value>#` |

Run-time values (RAM only: a reset brings back the `config.h` defaults, so the Pi sends
its values again after a start). Value syntax: optional sign, digits, optional decimals,
at most 9 digits, no exponent.

| Name | Unit | Default | Bounds |
|---|---|---|---|
| `kff` | PWM % per m/s | 770 | 0 to 3000 |
| `kp` | PWM % per m/s of error | 300 | 0 to 3000 |
| `ki` | PWM % per m of integrated error | 1500 | 0 to 20000 |
| `mpc_left`, `mpc_right` | micrometres per encoder count, each wheel | 12.36 | 5 to 30 |
| `track` | m | 0.296 | 0.20 to 0.40 |
| `accel` | m/s² | 0.25 | 0.05 to 2.0 |
| `brake` | m/s² (also `S#` and the timeout) | 0.5 | 0.10 to 3.0 |
| `vmax` | m/s, top speed of each wheel | 0.12 | 0 to 0.60 |
| `brake_at_zero` | 1 brake, 0 coast at zero PWM | 1 | 0 or 1 |

Not settable on purpose: the 0.3 s command timeout, the encoder-fault rule and the
watchdog.

Rules: a message is the bytes from a start letter (`F B S O R M E P U V D K`) to `#`. Malformed messages are dropped
and do not reset the timeout; only a valid `F` or `B` does. A start letter always begins a
new message, so a lost byte cannot glue the next `S#` onto a broken message. A byte with a
parity or framing error drops the message it is in. `\r` and `\n` are ignored.

## Open decisions for the owner

1. Approve or change the design draft (nothing gets flashed before that).
2. Letter and format of the fine odometry reply (`P#` above is a proposal). The old
   firmware answers only `F B S O R`, so `P`, `M` and `E` were free.
3. PWM frequency: **decided 2026-11-14, the owner chose 1 kHz** (`PWM_HZ`) instead of
   the old 100 Hz. Smoother at low speed; the speed per PWM percent may change, so the
   PWM ladder (design 5) sets `KFF`.
4. `METERS_PER_COUNT` and `TRACK_M` are derived from the old firmware and the LiDAR scales
   (0.902 distance, 0.862 yaw), not measured on their own: calibrate (design 4.2, 4.3).
5. `BRAKE_MPS2` 0.5 m/s²: **default chosen 2026-11-14, owner may change** (`Kbrake=`).
   Higher stops shorter but may slip on a smooth floor.
6. Brake or coast at zero PWM: **default chosen 2026-11-14: brake, owner may change**
   (`Kbrake_at_zero=0#` coasts). The L298N brakes with the enable high and both inputs
   low, and coasts with the enable low (the old firmware coasted).
7. One wheel scale for both wheels, or one each: **default chosen 2026-11-14: the same
   value for both, owner may change** with `Kmpc_left=` / `Kmpc_right=` after a
   straight-line test (no reflash).
8. The encoder fault also trips on a wheel blocked for 0.3 s at more than 30 % PWM, and
   the operator must send `S#` to drive again: **rule kept as the default on 2026-11-14,
   owner may change** (compile-time, like the timeout).

## Known from the owner's photos (2026-11-14)

- MCU: a Blue Pill module on a carrier board marked "Designed by NT+KS". Chip marking
  ST STM32F103C8T6; the ROM bootloader (version 0x22) reports device ID 0x0410 (medium
  density). 128 KiB answered the read, but the C8 is only specified for 64 KiB, so the new
  firmware must fit in 64 KiB.
- USB: the robot's cable goes to a purple CH340E module (USB-C; pins GND, +5V, TXD, RXD,
  DTR, 3V3) on the carrier, not to the Blue Pill's own USB.
- Motor driver: a red L298N module with 6 control wires (likely ENA, IN1 to IN4, ENB, so
  PWM on the enable pins). Motor supply terminal labelled 12V; other terminals "EStop",
  "Motor1", "Motor2". Datasheet drop 1.8 V typical, 3.2 V worst at 1 A, up to 4.9 V at
  2 A: the motors see about 8.8 to 10.2 V (one possible part of the 0.12 m/s cap, to check
  with a multimeter).
- A 4-pin SWD header at the Blue Pill's end, and yellow jumpers on BOOT0/BOOT1.
- Headers "ENCODER1" and "ENCODER2" (4 pin each), "PWMEXP2", "I2CExpansion",
  "ADCExpansion", "GND 5V", an HV/LV level shifter, LEDs on PC14 and PC15.
- The original ROS driver (gitlab.com/robotcitizens, `rct_ahr_driver`, `nodes/driver.py`)
  uses the same text protocol, including `R#` (reset odometry). No firmware source was
  found there. Their URDF puts the wheels at +-0.1295 m (track 0.259 m, a model value),
  which disagrees with the owner's 0.31 m: calibration step 4.3 of the design settles it.

### Flashing over the existing cable (checked 2026-11-14, nothing written)

The CH340E is wired to USART1 (PA9/PA10), where the ROM bootloader listens at 8E1, the
same framing as the protocol. So the Pi flashes with `stm32flash` (installed there) over
the robot's own cable, no ST-Link: the owner sets BOOT0=1, BOOT1=0 and presses RESET,
then sets BOOT0 back to 0 and presses RESET to run. The firmware's UART is therefore
USART1 too. Read protection is off (option bytes 0x00/0x00). Whether DTR reaches RESET
is not checked.

### Backup of the old firmware (design section 1, done 2026-11-14)

Read twice over the bootloader, both reads identical, SHA-256
`5924421ea558a965d5519a98514ddeb8c92a08ea88528dca0b71876d4b849ef9`. 29 KiB used
(0x08000000 to 0x08007400), initial stack 0x20005000, reset vector 0x08004c6d, the
application starts at 0x08000000 (no separate bootloader). Copies on the Pi at
`~/firmware-backup/old-1.bin` and `old-2.bin`, and one on the laptop in the sim-ladder
worktree (not committed: it is someone else's binary).

## Review: what could force a second jumper flash (2026-11-14)

Checked in the code, and how each is handled:

| Risk | Handling |
|---|---|
| An update stops half way (power, cable, Pi) | stage1 and its page are never erased by `update.py` (`-S 0x08001000:<length>`; stm32flash mass-erases only for a range from page 0 to the end, read in its source). stage1 finds the CRC-32 wrong and starts the ROM bootloader: `update.py --rescue` |
| A new application crashes or hangs at start | fault handler and watchdog restart the chip; the third restart in a row before 2 s of running goes to the ROM bootloader |
| No crystal (HSE does not start) | the application restarts into the ROM bootloader (it runs on the internal clock) instead of hanging |
| The ROM bootloader's Go leaves the PLL, vector table or USART set up | the application sets its vector table, unmasks interrupts, switches to the internal clock before starting the PLL, and writes every USART register it uses |
| The bootloader runs with VTOR pointing at stage1 | stage1 points VTOR at system memory before the jump, as with BOOT0 at 1 |
| The watchdog fires inside the ROM bootloader | `U#` restarts with a system reset, which stops the watchdog; the application starts it only after stage1 |
| An encoder that counts by itself blocks `U#` | `Uforce#` skips only the wheel-speed check |
| A bad run-time value | every value is bounded, nothing is stored in flash, a reset brings back the defaults |
| UART stuck after noise or overrun | the receive interrupt reads DR on every error (clears ORE/FE/NE/PE); a start letter always begins a new message, so `U#` parses after any garbage |
| Someone writes with `-S 0x08000000` or without `-S` | not preventable in firmware: FLASHING.md says so in bold; `update.py` refuses `full.bin` and the old image |
| Rolling back to `old-1.bin` | needs the jumper and removes stage1 (the old firmware has no `U#`) |

Not checked, because only the board can show it: that backup register 1 keeps the mark
across the system reset (VBAT is tied to 3.3 V on a Blue Pill), that the software jump
into the F1 ROM bootloader answers stm32flash, and that Go starts stage1. FLASHING.md
section 3 proves all three before the deck closes. A hang that starts only after 2 s of
running resets the crash count each time; the Pi then has those first 2 s of every start
to send `U#`.

## What is still to be checked at the robot

- The pin map above is read from the old firmware, not traced on the board. The bring-up
  test (`FLASHING.md`, section 2) checks every motor and encoder pin and sign with the
  wheels lifted before the base drives on the floor.
- The motor label (rated voltage, rpm, gear ratio) and the L298N output voltage under
  load (one possible cause of the 0.12 m/s cap).
