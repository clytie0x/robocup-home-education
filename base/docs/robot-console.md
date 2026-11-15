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
- **Live**: Xtion camera, LiDAR from above, the arm team's webcam (relayed from the Jetson's port 8082; theirs, never started or stopped here). During a task run the camera and LiDAR come from run_follow itself (relayed from its page, with the operator as a red ring and label, other people as hollow black circles, and the stop reason); during a check the page says the view is paused and shows the job's status instead. The camera stream has no detection box, so the camera shows the operator's distance as a red label instead of a box.
- **Task**: start run_follow (Dry run or Drive, 0.08 or 0.12 m/s, never above params.json `max_speed`, 2 to 20 min, always `--remote`; with "คนเสมือน" also `--virtual-operator 5610 --virtual-allow <PC>`, PIN needed, see [virtual-operator.md](virtual-operator.md)), then Follow, Wait, Resume, Go back, and STOP. A Drive run asks "The robot will move. Is the area clear and the cable safe?" first. Shows phase, lock, operator distance, nearest obstacle, Jetson link, wheels, stop reason from run_follow's own `GET :8080/status` 3 times a second, and the last 30 lines of the run's output.
- **Jetson**: start, stop, restart each program the agent lists (detector, whisper, voice, bridge, armstack, arm, flow); Start all (in that order); uptime and last log lines.
- **Checks**: Wheel test (4 cm at 0.05 m/s), Camera pitch check (`tools/sensor_check.py --floor`, needs 1.5 m of empty floor), Link test.
- **Data**: 1 hour, today, 7 days or one run; charts (power bank estimate with the 30 % line, Pi and Jetson temperature, Jetson VDD_IN W, link ms cable and Wi-Fi, commanded and measured speed of the picked run), drawn as SVG by the page itself; tap or hover for the value and time. A runs table (date, length, distance, end reason) and a CSV of the chosen window. Read only, from telemetry.py's `data/robot.db`; before the file exists the charts are empty, and without telemetry.py the tab says so.
- **Logs**: the newest `~/hmc-base/logs/<start time>/` runs with length and end reason; download `log.jsonl`.
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
- Every POST except STOP must come from the console's own page: `Content-Type: application/json` and `X-HMC-Console: 1` (a page from another site cannot send that header without a preflight the console never answers). STOP takes anything.
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

## API

Console (`:8081`): `GET /api/state` (cards, devices, job and its last 30 lines, Jetson snapshot, last wheel test), `GET /api/run` (run_follow's `/status`, proxied), `GET /api/logs`, `GET /api/linktest`, `GET /logs/<run>/log.jsonl`, `GET /stream`, `/scan` (during a task: relayed from run_follow's page, 503 until it answers), `/usb-stream`;
`GET /api/data/series?metrics=m1,m2&since=<unix>&until=<unix>&points=200` (each metric averaged into at most `points` buckets, at most 12 metrics, 1000 points), `GET /api/data/runs?limit=50`, `GET /api/data/run/<run_id>?points=400` (run_samples, every n-th row), `GET /api/data/export.csv?metrics=...&since=...&until=...`, `GET /api/bank` (telemetry.estimate plus `full_ts`, `warn_pct`); all read only (SQLite `mode=ro`), empty answers before the database exists, 503 without telemetry.py;
`POST /api/bank/full`, `POST /api/bank/reading {"percent": 0..100}` (no PIN);
`POST /api/task/start {"drive", "speed", "minutes"[, "virtual": true, "virtual_host": "<PC IPv4>"]}` (virtual: the PC that asked, or virtual_host, sends the person to UDP 5610; `/api/state` says `virtual_port`), `POST /api/run/command {"command": "follow"|"wait"|"resume"|"return"}` (proxied to run_follow's `POST /command`), `POST /api/stop`, `POST /api/check/start {"check": "wheel"|"floor"}`, `POST /api/jetson/<start|stop|restart|start-all|stop-all> {"name"}` (proxied to the agent on 10.0.0.2, then 192.168.31.137). With a PIN, POSTs other than `/api/stop` and the two `/api/bank` marks need the header `X-Console-Pin`. `/api/run/command` answers a `session` token (the tab keeps it in sessionStorage and sends it as `X-HMC-Session`); only the newest commanding tab's `GET /api/run?heartbeat=1` is forwarded as run_follow's heartbeat, other tabs get the status without it. Right after a command is taken the console sends one heartbeat for that tab itself, so the run's next cycle sees the page alive.

STOP never answers "nothing": with a job it runs one stop sequence (pressed again, the same sequence goes to SIGKILL, but never sooner than 0.3 s after asking the run to end, so it sends its own S#; when the run's page does not answer or answers an HTTP error, SIGTERM comes at once); without one it asks `:8080/control {"end": true}` (a run started by hand), ends what is left of the last job's process group (only if a process in that group still runs one of our job scripts, read from /proc), and sends S#. Job timing uses the monotonic clock. A STOP while a job is starting (the camera and LiDAR take up to 10 s to close) or within 3 s before keeps it from starting.

`GET /api/run?heartbeat=1` asks run_follow's `GET /status?heartbeat=1`, its page-alive heartbeat; a plain `GET /api/run` (and anything the console reads by itself) asks the plain `/status`, which is no heartbeat. The page adds `heartbeat=1` 3 times a second while a task runs and only while its tab is in view, so a phone in a pocket or out of Wi-Fi stops the heartbeat.

Jetson agent (`:8090`): see the docstring of `base/jetson/agent.py` for the `GET /status` shape (flat numeric metrics for telemetry plus the programs) and the POSTs.

Wheel test exit codes: 0 ok, 2 motors get no power, 3 one encoder dead, 4 no answer from the base, 5 unclear, 6 not under safe_run. It writes `~/hmc-base/logs/wheel-test/last.json`, which the Overview reads.
