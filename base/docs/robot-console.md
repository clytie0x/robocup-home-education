# Robot console

One web page to run and check the whole robot from a phone or a laptop, without ssh or an AI assistant.

| What | Where |
|---|---|
| Console (Pi) | `base/pi/console.py`, page files in `base/pi/console/` (index.html, style.css, app.js) |
| Jetson agent | `base/jetson/agent.py` |
| Wheel test | `base/pi/tools/wheel_test.py` (run by the console under `tools/safe_run.py`) |
| Job guard | `base/pi/tools/job_guard.py` (stops a job when the console dies) |

Deployed flat like the rest: `base/pi/*` to `~/hmc-base/` on the Pi, `base/jetson/*` to `~/hmc-follow/` on the Jetson.

## URLs

- Phone or laptop on the lab Wi-Fi: **http://192.168.31.21:8081/**
- On the LAN cable: http://10.0.0.1:8081/
- Jetson agent (read only from outside the Pi): http://192.168.31.137:8090/status or http://10.0.0.2:8090/status

The console replaces `tools/camera_view.py` on port 8081. camera_view still works on its own (same port: run one or the other); the console reuses its stream code.

## What the page does

Look: style 4 "Swiss minimal" (white, black rules, no rounded corners; red only for STOP, faults that need action and the person the robot follows; numbers in a monospace font). No web fonts or CDN: the page works offline. Phone (< 900 px): a header per tab and five tabs at the bottom (ภาพรวม, ภาพสด, งาน, ข้อมูล, อื่นๆ with Jetson, checks, Log, help); while a job runs a red STOP bar sits above them. Desktop: tabs on top, STOP top right, a one-line status strip, and the Overview shows camera, LiDAR and the system tables side by side.

- **Overview**: the state in one word, the power bank estimate, Pi °C and Jetson W, Start and Wheel test, then each fault as a red-ruled block (what was measured, then what to do), then the health list for the Pi (temperature, load, disk, uptime), LiDAR, Xtion camera, wheels and motor power (last wheel test), Jetson link (TCP to 10.0.0.2 and 192.168.31.137, ms), Jetson programs, arm webcam: black dot OK, black ring check, red dot fault. The power bank block shows `telemetry.estimate` (percent, its range, hours since the last full mark, minutes driven, a warning under 30 %) with "charged to full" and "reading from the display" (0 to 100) buttons.
- **Live**: Xtion camera, LiDAR from above, the arm's webcam (relayed from the Jetson's port 8082; the agent's webcam program, `webcam_view.py`). During a task run the camera and LiDAR come from run_follow itself (relayed from its page, with the operator as a red ring and label, other people as hollow black circles, and the stop reason); during a check the page says the view is paused and shows the job's status instead. The camera stream has no detection box, so the camera shows the operator's distance as a red label instead of a box.
- **Robot** (the page that opens): Xtion `/stream`, the arm webcam `/usb-stream`, and one Jetson `:8082/snap.jpg`, the LiDAR, status cards, a timestamped event list, and pick/drop at the measured bag (or the reachable drop default). A large STOP calls `/api/stop`, `/api/arm/stop`, and `/api/jetson/stop-all`.
- **Task**: start run_follow (Dry run or Drive, 0.08 or 0.12 m/s, never above params.json `max_speed`, 2 to 20 min, always `--remote`; with "คนเสมือน" also `--virtual-operator 5610 --virtual-allow <PC>`, PIN needed, see [virtual-operator.md](virtual-operator.md)), then Follow, Wait, Resume, Go back, and STOP. A Drive run asks "The robot will move. Is the area clear and the cable safe?" first. Shows phase, lock, operator distance, nearest obstacle, Jetson link, wheels, stop reason from `/api/state` once a second, and the last 30 lines of the run's output. The mission executive on that same `/api/state` is documented in [api-state.md](api-state.md).
- **Jetson**: start, stop, restart each program the agent lists (detector, whisper, voice, bridge, armstack, webcam, arm, flow); Start all (in that order); uptime and last log lines. The Arm card: the state line from `GET /api/arm` every 2 s while the tab is open (`/arm/state`'s state, torque on or off, busy or free, the task and step, a fault in red; an error from the agent or bag_arm in red), and Home, Rest, Center, Bagpick, Torque on, Torque off (`POST /api/arm` with the PIN) and Stop (`POST /api/arm/stop`, no PIN). Center asks first: the arm reaches out about 0.5 m in front of the robot. Bagpick (`{"cmd": "bagpick", "putback": false}`, the arm team's camera pick) asks first too: the arm reaches out and down to the floor in front of the robot; a miss shows its RESULT line in red. Torque off is enabled only while the state is `resting`. While the console is held the moving buttons are off and the holder is named; Stop still works.
- **Checks**: Wheel test (4 cm at 0.05 m/s), Camera pitch check (`tools/sensor_check.py --floor`, needs 1.5 m of empty floor), Link test, Joystick drive (`tools/joystick_drive.py`, the DualShock 4 plugged into the Pi: hold R1 to move, left stick up and down is forward and backward (backward at 0.04 m/s only), right stick turns; from the page at most 0.08 m/s for 5 minutes; the LiDAR guards every move, front and back, and stops the base on no scan, but it does not see the power cable, so the person driving watches it; nothing moves until R1 has been seen released once; the page asks first and sends `--yes-owner-at-robot`, which a program cannot, so `robotctl check joystick_drive` is refused; `python tools/joystick_drive.py --dry` over ssh prints the buttons and what would be sent without opening wheels or LiDAR). The tool finds the controller by itself in `/proc/bus/input/devices` (USB or Bluetooth) and reads the motion-sensor stream of the same controller as a heartbeat (about 170 reports a second): no report for 0.25 s stops the base and R1 must be seen released again, so a lost Bluetooth link cannot leave the base driving; a Bluetooth controller without that stream is refused. Pairing is done on the Pi with `bluetoothctl` (see `base/docs/superpowers/specs/2026-11-20-joystick-drive-design.md`).
- **Data**: 1 hour, today, 7 days or one run; charts (power bank estimate with the 30 % line, Pi and Jetson temperature, Jetson VDD_IN W, link ms cable and Wi-Fi, commanded and measured speed of the picked run), drawn as SVG by the page itself; tap or hover for the value and time. A runs table (date, length, distance, end reason) and a CSV of the chosen window. Read only, from telemetry.py's `data/robot.db`; before the file exists the charts are empty, and without telemetry.py the tab says so.
- **Logs**: the newest `~/hmc-base/logs/<start time>/` runs with length and end reason; download `log.jsonl`. Below them the command list: the last 30 lines of `GET /api/commands`, newest first (time, who, path with the HTTP code when not 200, the error in red or ok; a maintenance line shows the reason).
- **Help**: a short how-to in Thai.

STOP is at the bottom of every tab while a job runs. It asks the run to end (`POST :8080/control {"end": true}`), sends SIGTERM to the job's process group after 3 s, SIGKILL after 3 s more, then sends S# itself. Pressing STOP again kills at once. It is not an emergency stop: with the Wi-Fi gone the page can do nothing. The motor cut-off on the robot is.

## Devices and jobs

The camera and the LiDAR can each be open in one program at a time. The console reads both for the Live tab and closes them (reader threads joined) before it starts any job, and opens them again when the job has ended. During a run, run_follow puts each cycle's frame and scan in `live_feed.py`; its page on port 8080 serves them as `/stream` (MJPEG, at most 10 a second, encoded only while someone watches) and `/scan`, and the console relays both under its own `/stream` and `/scan`. The console starts run_follow with `--dashboard-host 127.0.0.1`, so port 8080 (switches and task buttons, no PIN) answers only the console; STOP still needs no PIN. A missing camera or LiDAR is shown as not available and retried every 5 s; the console starts without them.

One job at a time: run_follow or one check. Each job is started as `tools/job_guard.py <program>` in its own process group, output to `~/hmc-base/logs/console/<time>-<kind>.log`. Anything that moves the wheels runs under `tools/safe_run.py`. When the console ends (Ctrl+C, SIGTERM) it stops the job as STOP does; when it is killed outright (kill -9, OOM), job_guard sees its parent change within 0.2 s and stops the group itself.

## Start by hand

Pi:

```bash
cd ~/hmc-base
.venv/bin/python console.py            # port 8081; --no-devices to never open the camera or LiDAR
```

Jetson:

```bash
cd ~/hmc-follow
python3 agent.py                       # port 8090; programs' logs in ~/hmc-follow/logs/<name>.log
```

The agent starts, from `~/hmc-follow`, `python3 server.py`, the whisper server (`/opt/robot_voice/src/whisper.cpp/build/bin/whisper-server` on 127.0.0.1:8080, as `voice/scripts/servers.sh start-stt` does, in the foreground), `ros2 launch voice_assistant hmc_voice.launch.py` (with `~/ros2_ws/install/setup.bash`), `python3 base_bridge.py --pi-host 10.0.0.1,192.168.31.21`, the arm stack (`ros2 launch so100_moveit_config arm.launch.py mount:=robot rviz:=false`), `python3 bag_arm.py` and `python3 hmc_flow.py --ros-args -p use_arm:=true -p grab_timeout:=240.0 -p drop_timeout:=60.0` (the last five with `/opt/ros/jazzy/setup.bash` and `ROS_DOMAIN_ID=42`; the arm stack and bag_arm also with `~/so100_ws/install/setup.bash`), each in its own process group. The two voice paths are `VOICE_ROOT` and `VOICE_WS` at the top of agent.py: check them on the robot before the first Start all. A whisper server or an arm stack started by hand is used by Start all as it is. It only ever signals what it started; copies started by hand are listed as "outside" and left alone. Stopping the agent stops what it started.

## Autostart (user crontab, no sudo)

Installed in each user's crontab. Both jobs start only the console and the agent: the agent waits for Start all, the console for Start task.

Pi:

```cron
@reboot sleep 20 && /home/xohm/hmc-base/tools/run_console.sh
```

`tools/run_console.sh` starts `console.py` from `~/hmc-base`, appends to `logs/console/console.out`, and starts it again 2 s after it ends. A lock file allows one copy.

Jetson (a login shell, so the CUDA and TensorRT paths of `~/.profile` are there as when started by hand):

```cron
@reboot sleep 30 && bash -lc 'cd "$HOME/hmc-follow" && mkdir -p logs && exec python3 agent.py >> logs/agent.out 2>&1'
```

Remove (on that machine):

```bash
crontab -l | grep -v 'run_console.sh' | crontab -     # Pi
crontab -l | grep -v 'agent.py' | crontab -           # Jetson
```

Then stop the running one: `pkill -f '[r]un_console.sh'` and then `pkill -f 'console.py'` on the Pi (the wrapper starts the console again until the wrapper is gone; it stops its job first), `pkill -f 'agent.py'` on the Jetson (it stops its programs first).

[`base/systemd/`](../systemd/README.md) holds user units for the same two jobs, with the same log files. Use one or the other, not both: the console's lock file keeps the second copy from running. `deploy.sh` does not copy that folder; install it once on the machine.

## Who can use it

Anyone who can reach the Pi's port 8081 can use the page, so anyone on the lab Wi-Fi could start a Drive run. Limits in place:

- PIN (off until set): put one in a file outside the repo, and every start and command needs it; STOP never does. The page asks once and remembers it in the browser. Without a PIN file a Drive run and the wheel test are refused ("ตั้ง PIN ก่อน"); dry runs and the camera check still start.
- Every POST except STOP must come from the console's own page: `Content-Type: application/json` and `X-HMC-Console: 1` (a page from another site cannot send that header without a preflight the console never answers), or from a program on the Pi itself: `X-HMC-Client: <name>` instead, taken only from 127.0.0.1 (see robotctl below). STOP takes anything.
- The wheel test (it drives 4 cm without looking) needs the PIN, runs at most once every 10 s, and needs a scan of the console's own LiDAR younger than 0.5 s showing nothing in front within 0.35 m (0 < x < 0.35, |y| < 0.25): without one (a job holds the LiDAR, `--no-devices`, the LiDAR not back yet) it is refused.
- 5 wrong PINs (from anyone) lock the PIN out for 60 s, then 120 s and so on up to 15 min (logged on stderr); STOP is never locked.
- At most 2 `/api/data/*` queries at once, 1 while a drive job runs (429 otherwise).

  ```bash
  mkdir -p ~/.config && printf '%s\n' 4821 > ~/.config/hmc-console-pin && chmod 600 ~/.config/hmc-console-pin
  ```

  Restart the console to take it. `rm ~/.config/hmc-console-pin` and a restart turn it off.
- The Jetson agent takes start and stop only from 127.0.0.1, 10.0.0.1 and 192.168.31.21 (`--allow`), so from the lab Wi-Fi it goes through the console (and its PIN); its `GET /status` is open.
- Speeds are only 0.08 and 0.12 m/s and never above params.json `max_speed`; every drive is under safe_run.
- Still open: run_follow's own page on port 8080 has no PIN (its `/control` and `/command` work for anyone on the network while a run is up).

## Programs and AI sessions: robotctl

Programs, scripts and AI sessions command the robot the way the page does, through the console, with `~/hmc-base/robotctl` (`base/pi/tools/robotctl.py`, standard library only; `deploy.sh` installs the wrapper). It runs on the Pi, so from a PC: `ssh xohm@192.168.31.21 hmc-base/robotctl arm center`. It reads the PIN from `~/.config/hmc-console-pin` on the Pi (no file, no PIN header; it is never printed, and never leaves the Pi), sends `X-HMC-Client: $USER@<first field of $SSH_CLIENT>` (or `--who NAME`), prints the console's JSON answer and exits 0 when taken, 1 when refused (the error on stderr), 2 when the console does not answer.

```bash
robotctl state | stop | hold | release | log [N]
robotctl check NAME [args...]           # wheel, floor, base_check, approach, back_up, lap_check, tether_run, walk_check
robotctl task dry|drive SPEED MINUTES [--virtual HOST]
robotctl run follow|wait|resume|return|stop      # stop is the console's STOP
robotctl run bag|bag_again|bag_near
robotctl arm home|rest|center|stop|state
robotctl arm torque on|off
robotctl arm pick [X Y [TOP YAW]] | arm drop X Y
robotctl arm bagpick [--putback]
robotctl fetch
robotctl jetson start|stop|restart NAME | jetson start-all|stop-all
```

Help Me Carry step by step (each one alone, from the page's task buttons where there is one, or robotctl; a task run must be up for the `run` steps, `robotctl task dry|drive ...`):

| Step | robotctl | What it does, and when it is taken |
|---|---|---|
| Follow the person | `run follow` | IDLE to FOLLOW (the way back starts here); after a wait it follows again |
| Stop at the car | `run wait` | stands still, the senses keep running; cancels a running bag task |
| Go on | `run resume` | ends a wait (and a hold for wheels that did not turn) |
| Go back | `run return` | the way back along the breadcrumbs; refused in IDLE and in a bag task started from IDLE |
| Go to the pointed bag | `run bag` | only in FOLLOW held by a wait: the operator points, the robot drives up to the bag (bag.py) |
| The last bag again | `run bag_again` | as bag, to the bag chosen last, measured once more |
| Go to the bag in front | `run bag_near` | nobody points: the nearest bag-sized LiDAR cluster within 2 m and 40 deg of straight ahead; also from IDLE (a step test), and it ends the silence hold; the run goes back to the phase it came from |
| Pick the bag | `arm pick [X Y [TOP YAW]]` | bag_arm's pick (robot frame m, rad). Without X Y: the running task's last reached bag (x, y, top and yaw when the look measured them). Refused while the bag task moves the base, and when the run's status does not answer. The answer comes when the pick is done (up to 240 s), with `code`, `message` and `reading` (gripper_pos, gripper_load) |
| Put it down | `arm drop X Y` | bag_arm's place at that floor point; the same rules |
| Pick the bag by camera | `arm bagpick [--putback]` | the arm team's `bagpick.py` (arm/tools/bagpick) in `~/so100_ws`, started by bag_arm: it finds the bag in front with the arm camera, grasps it and raises it (`--putback`: and puts it back, a test). The same rules as a pick, and one at a time with a pick or drop. The answer comes when it ended (up to 300 s, then it is killed): `ok` only for `RESULT holding`, `code` HOLDING_BAG, MISSED, TOO_FAR, NO_BAG, LOST or FAILED (STOPPED after the arm's stop), `message` its RESULT line, `tail` its last 20 output lines. The arm's stop sends it Ctrl+C, then SIGTERM, then SIGKILL |
| Bag in one go | `fetch` | `run bag_near`, then the run's status twice a second (its page heartbeat) until that bag task reached the bag or failed (at most 120 s), then `arm pick` with its result; prints each step; Ctrl+C sends STOP |

A refused `run` step (for example bag outside a wait) is only in the run's log: the console's answer means the run queued it. `fetch` says "no bag result" after 120 s then. A pick or drop from the console is the same bag job as the flow's `/robot/command`: one at a time from either side, and its `grab_success`/`grab_failed` event still goes out on `/robot/event`.

- Checks by name: each tool's options are on an allow-list in `console.TOOL_CHECKS` / `console.TETHER` (plain numbers only, no paths); a speed is never above params.json `max_speed`. The presence flags (`--yes-owner-at-robot`, `--yes-floor-run`, `--yes-back-up`, `--yes-close-approach`) are taken only from the page, never from a program. Checks that move need the PIN and run under safe_run. `air_check` (raw packets) is not a console check.
- Hold: `robotctl hold` is refused while a job runs or is starting or the agent's `GET /arm` says `busy`. While held, task and check starts, arm moves and Jetson start/stop are refused; STOP and the arm's stop are not. `robotctl release` or a console restart ends it. `deploy.sh` holds before it copies anything (and refuses to deploy without the hold, unless the console does not answer at all), releases at the end, also on a failure, and `--restart` restarts the console only while it holds it. `deploy.sh --rollback` asks for the hold the same way before it restores anything (not with `--dry-run`).
- Command log: every POST, from the page or a program, is one line in `~/hmc-base/logs/console/commands.jsonl` (`time`, `client`: the program's name or `page` with the first 6 characters of the tab's session, `address`, `path`, `body` without any `pin`, `code`, `error`). `console_gate.py` writes its maintenance lines (`HMC_MAINTENANCE=<reason>`) to the same file.
- Arm gate on the Jetson: `base/jetson/console_gate.py` (a copy; the Jetson cannot import `base/pi`) refuses `bag_arm.py --pick/--place` and `floor_pick.py` on the real arm (`--dry-run` is free) unless `HMC_CONSOLE_JOB` is set or `HMC_MAINTENANCE=<reason>` is, which is logged in `~/hmc-follow/logs/commands.jsonl`. Every program the agent starts gets `HMC_CONSOLE_JOB=agent:<name>`; bag_arm as the agent's node (no `--pick/--place`) is not gated.

## API

Console (`:8081`): `GET /api/state` (cards, devices, job and its last 30 lines, Jetson snapshot including the cached arm, last wheel test, `run`, `bag`, `drop`, `events`, `bank`), `GET /api/run` (run_follow's `/status` while a task runs; `{"state":"idle"}` with HTTP 200 when none is), `GET /api/logs`, `GET /api/linktest`, `GET /logs/<run>/log.jsonl`, `GET /stream`, `/scan` (during a task: relayed from run_follow's page, 503 until it answers), `/usb-stream`, `/usb-snap` (one JPEG from the arm webcam);
`GET /api/data/series?metrics=m1,m2&since=<unix>&until=<unix>&points=200` (each metric averaged into at most `points` buckets, at most 12 metrics, 1000 points), `GET /api/data/runs?limit=50`, `GET /api/data/run/<run_id>?points=400` (run_samples, every n-th row), `GET /api/data/export.csv?metrics=...&since=...&until=...`, `GET /api/bank` (telemetry.estimate plus `full_ts`, `warn_pct`); all read only (SQLite `mode=ro`), empty answers before the database exists, 503 without telemetry.py;
`POST /api/bank/full`, `POST /api/bank/reading {"percent": 0..100}` (no PIN);
`POST /api/task/start {"drive", "speed", "minutes"[, "virtual": true, "virtual_host": "<PC IPv4>"]}` (virtual: the PC that asked, or virtual_host, sends the person to UDP 5610; `/api/state` says `virtual_port`), `POST /api/run/command {"command": "follow"|"wait"|"resume"|"return"|"bag"|"bag_again"|"bag_near"}` (proxied to run_follow's `POST /command`), `POST /api/stop` (also the arm's stop, best effort: the answer's `arm` note says how it went, the base's stop never waits for it), `POST /api/check/start {"check": "wheel"|"floor"|<tool>[, "args": [...]]}`, `POST /api/jetson/<start|stop|restart|start-all|stop-all> {"name"}` (proxied to the agent on 10.0.0.2, then 192.168.31.137);
`POST /api/arm {"cmd": "home"|"rest"|"center"|"torque"[, "on": true|false]}` (relayed to the agent's `POST /arm`, its code and answer back; 409 during a task run or a hold), `POST /api/arm {"cmd": "pick_bag"[, "x", "y"[, "top", "yaw"]]}` and `{"cmd": "drop_bag", "x", "y"}` (also during a task run, 409 while its phase is `bag`, 503 when its status does not answer; a pick without x, y takes the run's `record.bag_task.result` when it is ok, else 409; the answer has `sent`, what went to bag_arm; waits up to 245 s), `POST /api/arm {"cmd": "bagpick"[, "putback": true|false]}` (the same rules; waits up to 335 s), `POST /api/arm/stop` (no PIN), `GET /api/arm` (the agent's `GET /arm`); a command is sent to the Wi-Fi address only when the cable one could not be reached, never again after a timeout;
`POST /api/hold {"who"}` (409 when busy or already held), `POST /api/release`; `/api/state` has `hold` (`{"who", "since"}` or null); `GET /api/commands?n=20` (the last lines of the command log, at most 200, from its last 16 KB). With a PIN, POSTs other than `/api/stop`, `/api/arm/stop` and the two `/api/bank` marks need the header `X-Console-Pin`. `/api/run/command` answers a `session` token (the tab keeps it in sessionStorage and sends it as `X-HMC-Session`); only the newest commanding tab's `GET /api/run?heartbeat=1` is forwarded as run_follow's heartbeat, other tabs get the status without it. Right after a command is taken the console sends one heartbeat for that tab itself, so the run's next cycle sees the page alive.

STOP never answers "nothing": with a job it runs one stop sequence (pressed again, the same sequence goes to SIGKILL, but never sooner than 0.3 s after asking the run to end, so it sends its own S#; when the run's page does not answer or answers an HTTP error, SIGTERM comes at once); without one it asks `:8080/control {"end": true}` (a run started by hand), ends what is left of the last job's process group (only if a process in that group still runs one of our job scripts, read from /proc), and sends S#. Job timing uses the monotonic clock. A STOP while a job is starting (the camera and LiDAR take up to 10 s to close) or within 3 s before keeps it from starting.

`GET /api/run?heartbeat=1` and `GET /api/state?heartbeat=1` ask run_follow's `GET /status?heartbeat=1`, its page-alive heartbeat; a plain read (and anything the console reads by itself) asks the plain `/status`, which is no heartbeat. The page adds `heartbeat=1` on its once-a-second `/api/state` poll while a task runs and only while the tab is in view, so a phone in a pocket or out of Wi-Fi stops the heartbeat. With no task running, both reads return HTTP 200 `{"state":"idle"}` and do not ask port 8080.

Jetson agent (`:8090`): see the docstring of `base/jetson/agent.py` for the `GET /status` shape (flat numeric metrics for telemetry plus the programs) and the POSTs.

Wheel test exit codes: 0 ok, 2 motors get no power, 3 one encoder dead, 4 no answer from the base, 5 unclear, 6 not under safe_run. It writes `~/hmc-base/logs/wheel-test/last.json`, which the Overview reads.
