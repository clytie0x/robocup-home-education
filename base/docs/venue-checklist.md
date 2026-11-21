# Venue checklist: Help Me Carry 5.1

One page for the team at the competition. Numbers are from this repo and their source is at
the end. The drive is on a cable and the rulebook limit is 5:00, so the order below matters.

Healthy in the console is a black dot, not green: a black dot is OK, a black ring is a check,
a red dot is a fault (`base/docs/robot-console.md`).

## Before leaving

- Charge the 21 V "21V MAX LITHIUM" tool battery. It feeds the motors through the DC-DC
  converter; with it disconnected the wheels get no power and the wheel test ends with code 2
  (hardware assessment 2026-11-13, power; 2026-11-15, motor power).
- Charge the UGREEN power bank, 145 W, 25,000 mAh. It feeds the Pi side, not the motors.
- Set the travel router to the lab SSID with WPA2 and test that a phone reaches
  http://192.168.31.21:8081/ (Pi console) and http://192.168.31.137:8090/status (Jetson
  agent). A crowded venue widens link times (hardware assessment 2026-11-14).
- Laptop with the repo, a clean `git status`, at the deployed commit or tag. Know that
  commit: `VERSION.json` on each robot and the console show it.
- Console PIN set and written down. The wheel test and a Drive run need it; without one only
  dry runs and the camera check start (`base/docs/robot-console.md`, "Who can use it").
- The robot's 10+ m power cable and the top-deck power strip. The cable is what limits turns
  (hardware assessment 2026-11-14, arm section).
- Get the arm to rest before packing: folded, gripper closed, torque on. The 2026-11-15 run
  showed `/arm/state` resting, torque on, floor z -0.331 (hardware assessment, M1 section).

## At the venue: power-on order

1. Place the robot at the start, wheels on the floor, arm folded to rest.
2. Run the cable straight from the wall socket to the front-left post. No loop, nothing under
   a wheel.
3. Connect the 21 V tool battery (motors). Check both connectors are seated.
4. Power the Pi side from the power bank. The console starts by itself: the crontab waits 20 s
   after boot (`base/docs/robot-console.md`, autostart). Open the console.
5. Power the Jetson. The agent starts by itself 30 s after boot on the crontab; nothing under
   it starts until Start all. The Jetson tab must show the agent.
6. Join the lab SSID on the phone and laptop. Check the console at 192.168.31.21:8081. If the
   laptop is on the robot's cable network, 10.0.0.1:8081 is the same page.
7. Press Start all on the Jetson tab. It starts detector, whisper, voice, bridge, armstack,
   webcam, arm, flow in that order (`base/jetson/agent.py`, `ORDER`). A program with a port gets up to
   60 s to listen; one without a port gets 3 s before the next.

The order is for the checks: the 21 V battery first so the wheel test can turn the wheels,
the Pi before the Jetson so the console can see the agent, Start all last so nothing runs
before the operator is ready. If `base/systemd/` was installed, both jobs start at boot
instead of after 20 s and 30 s; do not also keep the crontab lines.

## What healthy looks like

- **Overview**: no red fault block. The health line is `n/n` and every card's dot is black,
  none rings. Camera and LiDAR each say สด (live).
- **Live**: the camera picture moves and the LiDAR drawing updates. A missing camera or LiDAR
  is retried every 5 s but a run should not start without them.
- **Wheel test**: on Overview or Checks, press it once. It drives 4 cm at 0.05 m/s. Result ok
  and exit code 0. Code 2 no motor power (check the 21 V battery), 3 one encoder dead, 4 no
  answer from the base, 5 unclear, 6 not under safe_run. It needs the PIN, runs at most once
  every 10 s, and needs a LiDAR scan younger than 0.5 s with nothing within 0.35 m ahead.
- **Jetson tab**: 8/8 running, one black dot per program, no red rows, and the sequence line
  says "all started". The eight are detector, whisper, voice, bridge, armstack, webcam, arm, flow.
- **Version**: the lower line on the Checks, Logs or Help tab shows the Pi and the deployed
  commit; the Jetson tab shows `agent <host> <commit>`. Both are the commit you deployed and
  the same one in `VERSION.json`, which `deploy.sh` writes.
- **Arm**: folded at rest; the Live tab's arm webcam (port 8082, the webcam program) shows it. The
  console only stops programs the agent started. A copy the arm team started by hand is left
  alone, and Start all uses an arm stack started that way instead of starting a second one.

## The cable

- Lay it straight before every attempt. When it is straight, reset the count on the Pi:
  `cd ~/hmc-base && .venv/bin/python cable.py --reset`. The console has no reset button.
- The owner's limits are 200 deg to warn and 240 deg hard (`base/pi/cable.py`, `WARN_DEG`,
  `LIMIT_DEG`). From 200 deg a turn that would wind more goes the other way round or stops;
  no command takes the net past 240 deg. At the limit the robot says "My cable is wound.
  Please straighten it."
- After a run, read the net turn on the Pi with
  `cd ~/hmc-base && .venv/bin/python cable.py` and straighten before it reaches 200. The
  console does not show this number.
- The red push button on the robot cuts motor power. Keep it within reach on every drive; the
  cable and the console are not emergency stops.

## Arm at rest and the deck

- Before each run: arm folded, gripper closed, and nothing on the top deck where the arm or a
  carried bag goes. The deck holds the power strip and the Jetson board; the arm team's
  planner keeps a carried bag 8 cm off the deck footprint, or 20 cm above it when it passes
  over (`BAG_OVER_DECK`, hardware assessment 2026-11-14).
- The arm team's IK reaches 0.605 m at most, so a chair seat is out; the bag goes down on the
  floor at the start (hardware assessment 2026-11-14; `hmc/docs/tasks/5.1-help-me-carry.md`,
  procedure 4).
- The flow waits up to 240 s for a pick and 60 s for a place (`base/jetson/agent.py`,
  `FLOW_ARGS`). If the arm stalls, most of the 5:00 is gone; stop the run and, if a scoring
  restart is still allowed, take it rather than waiting for the timeout.
- A copy of the arm stack started by hand is used by Start all as it is (`SHARED` in
  `agent.py`). The console cannot stop a copy it did not start.

## Dry run, then drive

- Task tab, Dry run, 0.08 m/s, 2 min. The wheels do not move. Check that the run starts, the
  phase changes, and STOP works.
- Then Drive, only when the area is clear and the cable is safe. The console asks that
  question first. Speeds are 0.08 or 0.12 m/s and never above `max_speed` 0.12 in
  `base/params.json`.
- The base tops out at about 0.12 m/s at full PWM, a hardware limit (hardware assessment,
  2026-11-15). It coasted 5 to 8.4 cm after S# on the old firmware (Task 2). The floor stop on
  the firmware flashed 2026-11-14 is not measured yet (`base/pi/tools/brake_distance.py`). Leave
  that room.

## When something fails

- Robot heading for a person, cable taut, or a wheel fault: press the red push button on the
  robot. It cuts motor power. The console STOP is not an emergency stop: with the Wi-Fi gone
  the page can do nothing (`base/docs/robot-console.md`).
- Console STOP: asks the run to end, sends SIGTERM after 3 s, SIGKILL 3 s later, then S#.
  Pressing it again kills at once. It is the normal end of a bad run, not the emergency stop.
- The firmware flashed 2026-11-14 ramps to a stop when no valid F or B comes for 300 ms
  (`COMMAND_TIMEOUT_MS` in `base/firmware/config.h`). Checked with the wheels lifted only
  (hardware assessment, "New firmware flashed"). The red button stays in reach: it cuts the motor
  power whatever the board does.
- Scoring restart: one per attempt, only before a major scoring action. It subtracts 30 s from
  the total and points scored before it are dropped. Leave the arena with the robot and queue
  again (rulebook 3.7.9, restated in `hmc/docs/tasks/5.1-help-me-carry.md`).
- Console: `systemctl --user restart hmc-console` if the units are installed; otherwise
  `pkill -f 'console.py'`, and the crontab wrapper starts it again after 2 s if that wrapper
  runs. Its log is `~/hmc-base/logs/console/console.out`.
- Jetson: `systemctl --user restart hmc-agent`, or `pkill -f 'agent.py'`; a clean stop ends the
  programs it started. Wait for :8090 to answer, then Start all. Its log is
  `~/hmc-follow/logs/agent.out`.
- Start all says a program "is already running, started by hand": another copy runs outside
  the new agent, either started by hand or left from an agent that died. Stop that copy at a
  Jetson terminal, then Start all. The new agent does not own it, so the console cannot stop
  it.
- Rollback: from the laptop, `base/tools/deploy.sh --rollback` restores the newest
  `backups/deploy-<time>.tar.gz` on both hosts. Like a deploy it asks the console for the hold
  first (refused while a job runs or the arm moves) and releases it at the end. It does not restart anything, so restart the
  console and the agent as above afterwards.
- Jetson USB-A ports dead: only a full power cycle brings them back (hardware assessment
  2026-11-14). Do not replug USB near the running Jetson; power off, unplug, plug in.
- No console at all: check the SSID and router, then the Pi's power bank, then the console log.
  With the laptop on the robot's cable network, the console is also at 10.0.0.1:8081 and the
  agent at 10.0.0.2:8090.

## The five-minute task in order

1. Start signal. The robot says it is ready and asks the operator to walk at its pace:
   follow_to_car, 300 points.
2. The operator walks to the car. The robot follows and meets the four obstacles, 50 points
   each: small object, hard-to-see object, crowd, retractable barriers.
3. The operator points at one of the two bags. The robot says which bag it saw
   (perceive_bag, 15) and picks it (pick_bag, 100).
4. The robot returns to the start (re_reach_start, 200, optional) and puts the bag down.
5. If the crowd walks back, it may join the queue for the queue bonus, 300, optional.

The hard stop is 5:00. At 0.12 m/s the whole 300 s is 36 m if nothing else took time, so any
arm wait or restart comes straight out of the route. A scoring restart costs 30 s out of the
same 300 s.

## Where the numbers come from

| Number | Source |
|---|---|
| 5:00 limit, 30 s restart, one restart per attempt, points before it dropped | `hmc/docs/tasks/5.1-help-me-carry.md`, "Time limit" and "Tries and restart" (rulebook 3.7.9) |
| 300, 15, 100, 4 x 50, 200, 300 points | same file, "Score sheet" |
| 21 V tool battery to the motors, power bank 145 W 25,000 mAh to the Pi side | `base/docs/hardware-assessment-2026-11-13.md`, power sections (2026-11-13 and 2026-11-15) |
| 10+ m cable, 20 cm bag pass over the deck, arm reach 0.605 m | same file, arm section 2026-11-14 |
| 0.12 to 0.13 m/s real speed, 5 to 8.4 cm coast, no 1.5 s firmware timeout, red button | same file, Task 2 and safe_run sections 2026-11-13 |
| 200 deg warn, 240 deg hard, wound-cable text | `base/pi/cable.py` (`WARN_DEG`, `LIMIT_DEG`, `TEXT`; owner, 2026-11-15) |
| 4 cm wheel test at 0.05 m/s, exit codes 0 and 2 to 6, PIN and 0.35 m rule | `base/docs/robot-console.md`, "What the page does" and "Who can use it" |
| 20 s Pi wait, 30 s Jetson wait, URLs | same file, "Autostart" and "URLs" |
| Program order, port wait 60 s, settle 3 s, 240 s/60 s arm timeouts | `base/jetson/agent.py` (`ORDER`, `UP_WAIT`, `SETTLE`, `FLOW_ARGS`) |
| 0.12 m/s cap | `base/params.json` (`max_speed`), measured in the hardware assessment |
| Commit shown in the console and in `VERSION.json` | `base/tools/deploy.sh` |
