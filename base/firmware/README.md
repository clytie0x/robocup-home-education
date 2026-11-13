# Base firmware (STM32), hardware-free part

New firmware for the wheeled base, following the draft in
[`../docs/firmware-design.md`](../docs/firmware-design.md). **Not approved, never
flashed.** The board is not identified yet, so only the parts with no hardware calls are
written; they run and are tested on the laptop.

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
6. Brake or coast at zero PWM (depends on the motor driver).
7. One wheel diameter for both wheels, or one each after calibration (a straight-line test
   that curves would show the need).
8. The encoder fault also trips on a wheel blocked for 0.3 s at more than 30 % PWM. That
   is intended (it saves the motor), but the operator must send `S#` to drive again.

## What the board photos must show

- The MCU marking (the full part number, for example `STM32F103C8T6`).
- The programming header: SWD pins (`SWDIO`, `SWCLK`, `GND`, `3V3`, maybe `NRST`), and
  the `BOOT0` pin or jumper.
- Where the CH340 connects: which MCU pins (the UART number decides whether the serial
  bootloader can be used).
- The motor driver chip or module and its marking (decides brake or coast, PWM pins and
  frequency, the current limit).
- The encoder wiring: how many wires per motor, which MCU pins they reach (they must land
  on timer channels that support encoder mode), and the supply voltage of the encoders.
- The motor label (rated voltage, rpm, gear ratio) and the motor supply voltage.
