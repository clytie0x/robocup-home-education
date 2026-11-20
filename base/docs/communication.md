# Who talks to whom (Pi, Jetson, console)

Status: 2026-11-16, after "everything through the console" (design
`superpowers/specs/2026-11-16-console-single-control-design.md`). One page for anyone who adds a program or a
port: read the map, then the rules at the end.

## The map

Commands go down the left column; status comes back the same way.

```
 people, AI sessions, scripts
        |  page (http :8081)            robotctl over ssh (127.0.0.1:8081, X-HMC-Client)
        v
 Pi  console.py ----------------------------------------------------------+
        | jobs (one at a time, HMC_CONSOLE_JOB)        | HTTP :8090        |
        v                                              v                  |
     run_follow.py --remote                      Jetson agent.py          |
        | dashboard 127.0.0.1:8080 (/command, /status, /control)          |
        | <- Jetson base_bridge, TCP :5604 (voice flow's commands)        |
        | -> Jetson detector :5601 (frames out, people back)              |
        | (slam_bridge :5602: offline only, tools/slam_replay.py)         |
        | -> Jetson front_camera :5606 (bag look)                         |
        | -> plan_server 127.0.0.1:5603 (paths)                           |
        v                                              | starts 8 programs, POST /arm
     wheels (serial), LiDAR (serial), Xtion (USB)      v
                                                 bag_arm.py 127.0.0.1:5607 -> ROS /arm/* (domain 42) -> arm
                                                 hmc_flow.py 127.0.0.1:5608 <- POST /flow (park, speech heard, say)
                                                 webcam_view.py :8082 (arm camera)
                                                 hmc_flow, voice, base_bridge (ROS topics /robot/command, /robot/event)
```

| From | To | Transport | Port | What | Rate |
|---|---|---|---|---|---|
| page, robotctl | console | HTTP JSON | Pi 8081 | every command; GET status | on demand, page polls |
| console | run_follow | HTTP JSON (127.0.0.1) | Pi 8080 | task buttons, STOP, status | page polls |
| Jetson base_bridge | run_follow | TCP JSON lines | Pi 5604 | voice flow's follow/wait/return/bag; status back | ping 2 Hz, status 5 Hz |
| run_follow | detector | TCP length-prefixed JSON + JPEG | Jetson 5601 | frames out, people/bags/pose back | at most 10/s (one frame per 0.1 s cycle, newest only; a ceiling, not measured) |
| console (Skeleton on, no job) | detector | same | Jetson 5601 | pose for the live view | 5/s |
| tools/slam_replay.py (offline only, run_follow does not use it) | slam_bridge | TCP length-prefixed JSON | Jetson 5602 | recorded scans out, map correction back | 1 per saved scan, real time |
| run_follow | plan_server | TCP | Pi 5603 (local) | paths, in a second process | 2/s |
| run_follow (bag look) | front_camera in bag_arm | TCP | Jetson 5606 | bag top from the front camera | once per bag |
| console | agent | HTTP JSON | Jetson 8090 | programs, /arm, status | on demand, status every 3 s (REFRESH) |
| agent | bag_arm | TCP JSON line (127.0.0.1) | Jetson 5607 | arm commands | on demand |
| agent | hmc_flow | TCP JSON line (127.0.0.1) | Jetson 5608 | the console executive's park lease, speech heard, say | on demand, while a console run polls |
| agent | base_bridge | UDP (127.0.0.1) | Jetson 5605 | hold before a restart | rare |
| bag_arm, front_camera, console | webcam_view | HTTP JPEG | Jetson 8082 | arm camera frames | on demand |
| voice stt_node | whisper-server | HTTP (127.0.0.1) | Jetson 8080 | speech to text | per sentence |
| voice llm_node | llama-server | HTTP (127.0.0.1) | Jetson 8081 | the LLM brain | per sentence |
| browser | voice control_panel | HTTP | Jetson 8091 | the voice team's own page | on demand |
| PC virtual operator | run_follow | UDP | Pi 5610 | the simulated person | 10/s |

## What is shared, and how

- **Commands** have one entry for people and programs: the console (page or robotctl), logged in
  `logs/console/commands.jsonl`. The voice flow is the one other commander: it reaches run_follow on 5604
  through base_bridge, and the arm through `/robot/command` to bag_arm. Both end in the run's own log.
- **Devices** have one owner at a time: the console holds the LiDAR and the Xtion and lends them to its one job
  (`console_gate` refuses everyone else). The arm has one trajectory owner in the driver; bag_arm serialises its
  own commands and bag jobs (`Jobs.claim`).
- **The detector** serves one client at a time and a new client replaces the old one. The console closes its
  Skeleton link when a job starts, so run_follow always gets it. Nothing else may connect to 5601.
- **Status** is pulled, never pushed to the console: the page polls the console, the console polls the run and
  the agent. A slow Jetson never blocks the Pi's control cycle (run_follow's links are non-blocking threads).

## Findings and what to do about them

1. **Two commanders of one run** (console page and the voice flow). run_follow takes both and logs both.
   Done 2026-11-16: `GET /api/commands` also reads the last 1 MB (`RUN_TAIL`) of the newest run log and lists
   its `remote_cmd` lines and the client's `remote_refused` lines as client `voice (Jetson)`, in time order with
   the console's own lines. run_follow writes a `wall` time on those lines for this; lines from before that
   (monotonic `time` only) are left out. Keep 5604 as it is: it is the low-latency path with the 2 s silence
   hold, and moving voice through HTTP would add a hop to every ping.
2. **Port numbers live in many files**. Done 2026-11-16: `base/jetson/ports.py` is the one table (host,
   program, protocol per port, and `HOSTS`). The Jetson imports it as it is; deploy.sh copies it to the Pi's
   `~/hmc-base/jetson/` with server.py and the others, and `base/pi/ports.py` loads it from there (or from
   `../jetson` on the laptop). The modules keep their names (`command_server.PORT = ports.COMMAND`, ...).
   `base/pi/tests/test_ports.py` fails on a number twice on one host and on a module that assigns a table number
   to a `*PORT*` name, a `port=` default or a `--port` default. Not covered: robotctl.py (deploy.sh runs it
   alone through stdin, so it keeps 8081 written out), params.json `jetson_port`, the page's text, systemd units.
3. **The length-prefixed JSON framing is written four times** (link.py, slam_link.py, server.py,
   slam_bridge.py). It works and each copy is tested; merging them is low value and touches the control loop.
   Leave it unless one of them changes.
4. **The arm path has three hops** (console HTTP -> agent HTTP -> bag_arm TCP -> ROS). Each hop is local or on
   the cable and the commands are seconds long, so the cost is noise. It buys one place (bag_arm) that
   serialises every arm user.
5. **Pull everywhere** keeps every program restartable on its own; the price is up to one refresh period of
   staleness on the page, which is fine for people. Nothing safety-relevant depends on the console's view:
   STOP goes straight to the run and S# straight to the wheels.

## Rules for new code

- A new way to make the robot do something is a console endpoint first, then a robotctl subcommand, then
  (if people need it) a page button. Never a script that opens a device or calls ROS directly.
- A program that opens a device calls `console_gate.require` (Pi: `base/pi`, Jetson: `base/jetson`, arm:
  `so100_driver`).
- A new port goes in `base/jetson/ports.py` (and `HOSTS`), in the table above and in the design's ports table, on
  a number nobody else uses on that host.
- Anything long-running on the Jetson is an agent program (start, stop, logs and env come from the agent).
