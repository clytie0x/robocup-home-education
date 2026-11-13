# Base firmware (STM32), hardware-free part

New firmware for the wheeled base, following the draft in
[`../docs/firmware-design.md`](../docs/firmware-design.md). **Not approved, never
flashed.** The board is a Blue Pill with an L298N (see below), but the chip and the
pin wiring are not confirmed yet, so only the parts with no hardware calls are written;
they run and are tested on the laptop.

- Lifecycle state: `unconfirmed` (draft until the owner approves the design)
- Entry point on the laptop: `make test` in this folder
- Entry point on the board: `main.c`, once `board.c` exists for the identified part

## What is here

| File | Job |
|---|---|
| `config.h` | Every constant. `MEASURE ON THE BOARD` marks placeholders |
| `board.h` | The only hardware interface (time, tick, encoder counts, PWM, UART, watchdog) |
| `protocol.c/.h` | Parse `F B S O R M E P`, format the `O`, `P`, `E` replies |
| `control.c/.h` | Body speed to wheel speeds, accel and brake ramps, PI wheel-speed loop with anti-wind-up, speed from counts |
| `odom.c/.h` | Pose from encoder counts, midpoint rule |
| `safety.c/.h` | 0.3 s command timeout, encoder fault |
| `app.c/.h` | Ties them together: one control tick, one UART poll |
| `main.c` | Board loop: wait for the 5 ms tick, poll, tick, kick the watchdog |
| `tests/` | Plain-C tests; `board_stub.c` stands in for the board, `test_app.c` runs the whole firmware against a model motor |

Not here yet: `board.c` (needs the part number), the `arm-none-eabi-gcc` build, the
flash and restore commands.

## Build and test on the laptop

Needs `gcc` and `make` (MinGW gcc 14.2 and GNU make were used on Windows).

```
cd base/firmware
make test
```

The last line of the test run is `<n> checks, <m> failed`; `make` fails if any check
fails. `main.c` is only compiled for syntax, since it needs `board.c`.

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
2. Letter and format of the fine odometry reply (`P#` above is a proposal). Check that the
   old firmware does not already use `P`, `M` or `E` for something else.
3. Does the old firmware's `B` turn the same way as `F` for a positive rate? The new code
   keeps "left positive" for both; the Pi never sends `B`.
4. Did the old firmware wrap `th` to plus or minus pi? The new one does not (two turns read
   12.57), which keeps more information; Pi code that wraps angles works with both.
5. `BRAKE_MPS2` 0.5 m/s² (draft value). Higher stops shorter but may slip on a smooth
   floor.
6. Brake or coast at zero PWM. The L298N can do both: enable high with both inputs at the
   same level brakes, enable low coasts.
7. One wheel diameter for both wheels, or one each after calibration (a straight-line test
   that curves would show the need).
8. The encoder fault also trips on a wheel blocked for 0.3 s at more than 30 % PWM. That
   is intended (it saves the motor), but the operator must send `S#` to drive again.

## Known from the owner's photos (2026-11-14)

- MCU: a Blue Pill module (STM32F103C8 class) on a carrier board marked "Designed by
  NT+KS". The chip marking is not read yet: it may be a GD32 or CKS32 clone.
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

### Flashing without an ST-Link, if the wiring allows

The STM32F103 ROM bootloader listens on USART1 (PA9 TX, PA10 RX) at 8E1 with BOOT0 high,
the same framing as the protocol. If the CH340E is wired to PA9/PA10, the Pi can read and
write the flash with `stm32flash` over the existing cable: the owner moves the BOOT0
jumper and presses RESET. Not checked yet: that wiring, and whether DTR reaches RESET.
The backup of the old firmware (design section 1) comes first either way.

### Clone check before any write

A chip marked STM32 can be a clone (CKS32 answers SWD id `0x2ba01477`, OpenOCD then needs
`set CPUTAPID`; GD32 and APM32 differ again). Read the SWD IDCODE and `DBGMCU_IDCODE` and
record them before flashing.

## What the board photos must still show

- The chip marking on the Blue Pill (STM32F103C8T6, a clone, or the 128 KB C8/CB part).
- The SWD header's pin labels (`SWDIO`, `SWCLK`, `GND`, `3V3`).
- Which Blue Pill pins the CH340E's TXD, RXD and DTR reach (PA9/PA10 and RESET would allow
  flashing over the existing cable, see above).
- The L298N wiring: which Blue Pill pins drive ENA/ENB (they must be timer PWM pins) and
  IN1 to IN4, and whether the ENA/ENB jumpers are still on (then PWM goes on the IN pins).
- The ENCODER1/ENCODER2 wiring: which Blue Pill pins the two signal wires reach (they must
  land on TIM2, TIM3 or TIM4 channels 1 and 2 for hardware encoder mode), and the encoder
  supply voltage (5 V encoders need 5 V tolerant pins or the level shifter).
- The motor label (rated voltage, rpm, gear ratio) and the L298N output voltage under
  load.
