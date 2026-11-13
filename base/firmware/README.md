# Base firmware (STM32F103C8T6)

New firmware for the wheeled base, following the draft in
[`../docs/firmware-design.md`](../docs/firmware-design.md). **Not approved, never
flashed.** The chip is an STM32F103C8T6 on a Blue Pill driving an L298N. The pin map
below was read from the old firmware's binary. Flash, check and roll back:
[`FLASHING.md`](FLASHING.md).

- Lifecycle state: `unconfirmed` (draft until the owner approves the design)
- Entry point on the laptop: `make test` in this folder
- Entry point on the board: `make firmware` gives `build/base.bin` (starts at `main.c`)

## What is here

| File | Job |
|---|---|
| `config.h` | Every constant. `MEASURE ON THE BOARD` marks placeholders |
| `board.h` | The only hardware interface (time, tick, encoder counts, PWM, UART, watchdog) |
| `board.c` | `board.h` for this board: clock, TIM3 PWM, TIM2/TIM4 encoders, USART1 rings, watchdog, LED |
| `stm32f103.h` | The few registers `board.c` uses, with reference-manual sections |
| `startup.c`, `stm32f103c8.ld` | Vector table, reset handler, memory layout (64 KiB flash, 20 KiB RAM) |
| `protocol.c/.h` | Parse `F B S O R M E P`, format the `O`, `P`, `E` replies |
| `control.c/.h` | Body speed to wheel speeds, accel and brake ramps, PI wheel-speed loop with anti-wind-up, speed from counts |
| `odom.c/.h` | Pose from encoder counts, midpoint rule |
| `safety.c/.h` | 0.3 s command timeout, encoder fault |
| `app.c/.h` | Ties them together: one control tick, one UART poll |
| `main.c` | Board loop: wait for the 5 ms tick, poll, tick, kick the watchdog |
| `tests/` | Plain-C tests; `board_stub.c` stands in for the board, `test_app.c` runs the whole firmware against a model motor |
| `tools/bringup.py` | Bring-up test on the Pi after flashing (wheels lifted); `test_bringup.py` checks it on the laptop |

## Build and test

Needs `gcc` and `make` for the tests (MinGW gcc 14.2 and GNU make were used on Windows),
and `arm-none-eabi-gcc` (Arm GNU Toolchain) for the board.

```
cd base/firmware
make test
make firmware
python -m unittest discover -s tools
```

The last line of the C test run is `<n> checks, <m> failed`; `make` fails if any check
fails. `make firmware` prints the image size and the SHA-256 of `build/base.bin`. On the
laptop the toolchain came from `winget install Arm.GnuArmEmbeddedToolchain` (14.2.Rel1)
and lives in `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin`; put
that on `PATH` if a new shell does not find `arm-none-eabi-gcc`. The build is
reproducible: a clean rebuild gives the same hash.

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
| `M<vvv>#` | Lower the per-wheel top speed to `vvv/100` m/s, never above `MAX_WHEEL_SPEED_MPS`. Lost at power-off |

Rules: a message is the bytes from a start letter to `#`. Malformed messages are dropped
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
5. `BRAKE_MPS2` 0.5 m/s² (draft value). Higher stops shorter but may slip on a smooth
   floor.
6. Brake or coast at zero PWM. The L298N can do both: enable high with both inputs at the
   same level brakes, enable low coasts.
7. One wheel diameter for both wheels, or one each after calibration (a straight-line test
   that curves would show the need).
8. The encoder fault also trips on a wheel blocked for 0.3 s at more than 30 % PWM. That
   is intended (it saves the motor), but the operator must send `S#` to drive again.

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

## What is still to be checked at the robot

- The pin map above is read from the old firmware, not traced on the board. The bring-up
  test (`FLASHING.md`, section 2) checks every motor and encoder pin and sign with the
  wheels lifted before the base drives on the floor.
- The motor label (rated voltage, rpm, gear ratio) and the L298N output voltage under
  load (one possible cause of the 0.12 m/s cap).
