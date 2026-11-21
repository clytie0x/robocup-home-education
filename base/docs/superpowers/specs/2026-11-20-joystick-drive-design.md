# Joystick drive: the DualShock 4 steers the base wheels

Status: approved by the owner, 2026-11-20 ("ให้ครบเลย": forward, turn, backward and the LiDAR guard all in).

## Why

The owner wants to move the base by hand with the DualShock 4. It was on the Jetson (`js0`) and is now plugged
into the Pi, where `js0` is readable by `xohm` (group `input`; 168 bytes of start-up events read on 2026-11-20).
Nothing in the repo uses a joystick today.

## Goals

1. A console check `joystick_drive` that drives the wheels from the controller: forward, turn on the spot, backward.
2. A deadman button: the base moves only while it is held, and every other reason to stop ends the drive.
3. The LiDAR guards every move (`safety.must_stop` forward and turning, `safety.rear_clear` backward).
4. A button on the console's Checks tab; a drive can only be started from the page: the presence flag `--yes-owner-at-robot` is page-only, so `robotctl check joystick_drive` is refused.
5. Tests and a short entry in `robot-console.md`.

## Non-goals

- Autonomous behaviour, arm control, Bluetooth pairing, a speed above `params.json` `max_speed`, the depth camera.
- Reading the joystick on the Jetson or relaying it over the network.

## Anti-goals

- A base that keeps moving after the controller, the program or the Pi's link dies.
- A second way to open the wheel port outside the console (`console_gate`, `safe_run`, PIN, `--yes-owner-at-robot`).

## Constraints (from the code)

- Wheels and LiDAR open only inside a console job (`console_gate.require`); wheels only under `tools/safe_run.py`.
- `wheels.packet` refuses a negative speed. Backward is only `Wheels.send_reverse`: at most `REVERSE_MAX` 0.06 m/s,
  turn at most `REVERSE_TURN_MAX` 0.05 rad/s, and only after `safety.rear_clear` (as `tools/back_up.py`).
- `max_speed` 0.12 m/s and `max_turn_rate` 0.49 rad/s from `params.json`; the console refuses a speed above `max_speed`.
- The firmware stops by itself 0.3 s after the last command; `Wheels` repeats the packet every 40 ms.
- The power cable (10 m, front left) must never be run over or wound; the operator watches it. The LiDAR does not see it.

## Approaches

| | What | Trade-off |
|---|---|---|
| **A (chosen)** | A tool in `base/pi/tools/` started by the console as a check, like `back_up` and `base_check` | Fits the one-way-to-command rule; no new port; one LiDAR reader per job already handled by the console |
| B | Controller stays on the Jetson, a program there sends UDP to the Pi | New port, a network hop and a second deadman on Wi-Fi; moot now that the controller is on the Pi |
| C | An always-on teleop service on the Pi | Opens the wheel port outside the console, which the 2026-11-16 design forbids |

## Design

- `base/pi/tools/joystick_drive.py [--max-speed M] [--seconds S] [--device PATH] [--dry] [--yes-owner-at-robot]`.
  Reads `/dev/input/js0` as raw 8-byte `js_event` records (`struct "IhBB"`), no new package. Axis and button numbers are
  constants at the top; their real values are confirmed with `--dry` (prints what it would send, opens no wheel or LiDAR).
- Nothing moves until the deadman has been seen released with both sticks centred (a trigger axis resting off centre, from a wrong axis number, never arms). SIGTERM from the console's STOP ends the tool through its finally blocks.
- A pure function `command(axes, buttons, p, max_speed)` returns `("stop"|"forward"|"reverse", v, w)`:
  deadman not held means stop; left stick up is forward and down is reverse; right stick sideways is the turn; a dead zone
  on both sticks; v and w scale with the stick up to `min(--max-speed, max_speed)`, reverse to 0.04 m/s (as `back_up`);
  `wheels.least` lifts a command too small to move the wheels.
- Every 50 ms the loop takes the newest LiDAR scan and asks the guard: no scan, a stale scan, `must_stop(...)` forward or
  turning, or `not rear_clear(...)` backward all send S#. A guard stop shows in the output and the base moves again
  when the stick asks for a command that is clear.
- Ends, with S# in `finally` (and `safe_run` after it), on: `--seconds` (default 300, at most 600), the controller going
  away (read error), a LiDAR error, SIGTERM from the console's STOP.
- Console: `TOOL_CHECKS["joystick_drive"] = (True, [], {"--max-speed": "speed", "--seconds": "number", "--yes-owner-at-robot": "flag"})`.
  `--dry`, `--device` stay out of the allow-list. The Checks tab gets a panel with a confirm box ("the base will move").
- Tests: `command()` table; the loop with a fake controller, fake wheels and fake scans (deadman release, blocked front,
  blocked rear, controller gone, time limit); `check_command` allow and refuse cases in `test_console_control.py`.

## Bluetooth (added 2026-11-20)

The owner wants the DS4 wireless. A js device sends an event only when a value changes, so a Bluetooth link that
dies with R1 held can leave the last state standing, and the kernel removes the device only after the link
supervision timeout (up to about 20 s). The heartbeat is the motion-sensor evdev device of the same controller
(`... Motion Sensors`): on USB it delivered about 168 reports a second at rest (measured on the Pi, kernel
6.18, `hid_playstation`, 2026-11-20).

- The tool finds the controller in `/proc/bus/input/devices` by name (`Wireless Controller`, with or without the
  vendor prefix) and takes the sensor device with the same `Uniq`; no controller found falls back to `js0`.
- `Pad.silent_for()` is the time since the last report of either device. Over `HEARTBEAT_S` (0.25 s, under the
  wheels' 0.3 s watchdog) the base stops and the tool needs R1 released again; the tool does not end.
- A Bluetooth controller with no sensor device is refused. `--device` skips the search and the heartbeat and stays out
  of the console's allow-list.
- Not verified: the kernel's names and report rate over Bluetooth (the test text for it is synthesized from the USB
  capture), the number of the js device after pairing, and the gap size under Wi-Fi interference on the Pi's shared
  2.4 GHz radio. A false stop is safe (R1 released and pressed again), a missed one is not, so 0.25 s errs to stopping.
- Pairing is a system change on the Pi: the owner does it (`bluetoothctl power on`, the DS4 in pairing mode with Share
  and PS held, then `scan on`, `pair`, `trust`, `connect` with its address) and runs `--dry` to see the line
  `controller /dev/input/jsN (bluetooth), heartbeat from /dev/input/eventM`.

## Test on the robot

1. `--dry` over ssh with the owner pressing buttons: confirm the numbers, print the commands, nothing moves.
2. Console start with the wheels off the ground or the owner holding the area clear, 0.05 m/s, a few seconds.
3. Then 0.08 and 0.12 m/s on open floor, forward, turn, backward, deadman release, controller unplugged mid-drive.

## Left to the builder

Exact button and axis numbers (found in step 1), the dead zone size, the wording of the page panel.
