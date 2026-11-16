# One way to command the robot: everything through the console

Status: draft for the owner, 2026-11-16.

## Why

On 2026-11-16 three programs fought over the robot in one hour: a deploy restarted the console in the
middle of a camera check and cancelled it; a floor check could only be started by reading the console PIN
over ssh; the arm was moved with hand-typed `ros2` commands because the console has no arm control. The
Pi's ROS containers and the console also want the same LiDAR port. The owner wants every person, AI
session and script to command the robot the same way, through the console, so nothing collides on a port
or a device and every command is seen.

## Goals

1. The console (`base/pi/console.py`) is the only way to command the base, the sensors, the arm and the
   Jetson programs. People use its page; programs and AI sessions use the same API.
2. A program API ("the back end"): `robotctl` on the Pi, a command line over the same HTTP API the page
   uses, so `ssh xohm@pi robotctl arm center` works from any PC or AI session. The PIN is read on the Pi
   and never leaves it.
3. Arm control in the console: home, rest, center (every joint to the middle of its range), torque on/off,
   stop. Bag pick and place stay in the task flow, as today.
4. Every tool in this repo that opens the LiDAR, the Xtion, the wheel port or commands the arm refuses to
   run unless the console started it.
5. A command log: who sent what, when, and the answer, for every command from the page or `robotctl`.
6. Deploys never cut a running or starting job.

## Non-goals

- OS-level enforcement (udev groups, file permissions, ROS access control). Chosen by the owner: option 1,
  enforcement in our code; ssh and ROS stay open for repairs.
- Starting the Pi's ROS containers from the console. The owner keeps them off (2026-11-16).
- Splitting the console into a separate back-end process (approach B, not chosen).
- New arm motions beyond the list above.

## Anti-goals

- A lock that keeps the owner out when the console is broken. There must be a logged maintenance
  override.
- A second copy of the PIN anywhere off the Pi.
- Breaking what works now: the page, the task runs, STOP, the virtual operator, the Jetson tab.

## Constraints

- Pi: Python with the `~/hmc-base/.venv`, no new packages. Jetson: the agent is plain Python without
  rclpy; ROS nodes run under the agent (`bag_arm.py` is the "arm" program, ROS domain 42).
- The arm driver already accepts one trajectory owner at a time; `so100_pick_place` has an auto rest
  (`rest_after_s`, 120 s) that must not pull the arm out of a pose the owner asked for.
- STOP must keep working without the PIN and without the program API.
- Another session is editing `console.py` today (skeleton overlay, 38ea592). The work goes on its own
  branch and is merged once, after the owner's go-ahead.

## Approaches

| | What | Trade-off |
|---|---|---|
| **A (chosen)** | Grow the existing console: program API, arm commands relayed to the Jetson, a console gate in our tools, `robotctl`, deploy asks first | Smallest change to working code; `console.py` gets longer |
| B | A new back-end process owns the devices; the console becomes a page over it | Cleaner split, but moves devices, jobs and PIN out of the console: large and risky |
| Dropped | A shared lock file every tool takes, without the console | Stops collisions but does not make the console the one way in |

The owner picked A.

## Design

### Program API and `robotctl`

- The console takes POSTs from programs as well as from its page. Today a POST needs `X-HMC-Console: 1`
  (a guard so other web pages cannot post). A program sends `X-HMC-Client: <name>` instead; that is
  accepted only from 127.0.0.1, so a program reaches it through ssh to the Pi. The PIN rule is unchanged.
- `base/pi/tools/robotctl.py` (installed as `~/hmc-base/robotctl`): reads `~/.config/hmc-console-pin`,
  sends `X-HMC-Client` (default `$USER@<SSH_CLIENT address>`, or `--who`), prints the JSON answer, exits
  non-zero on an error. Subcommands map 1:1 to the API: `state`, `stop`, `check wheel|floor`,
  `task dry|drive SPEED MINUTES [--virtual HOST]`, `run follow|wait|resume|return|stop`,
  `arm home|rest|center|torque on|off|stop|state`, `jetson start|stop|restart NAME`, `jetson start-all|stop-all`,
  `hold`, `release`, `log [N]`.
- The page keeps working unchanged.

### Command log

- `logs/console/commands.jsonl`: one line per POST: time, client (page session or `X-HMC-Client`), address,
  path, body without the PIN, HTTP code, answer's error. GET is not logged.
- `robotctl log` and a Logs-tab list show the last lines.

### Arm through the console

- Console: `POST /api/arm {"cmd": "home"|"rest"|"center"|"torque", "on": bool}` and `/api/arm/stop`
  (STOP-like: no PIN). Refused while a task run holds the arm (the bag phase).
- Jetson agent: `POST /arm` from its allow list, relayed to `bag_arm.py` over a new local TCP port
  (127.0.0.1:5607, JSON line in, JSON line out). `bag_arm` already holds the ROS clients
  (`/arm/home`, `/arm/rest`, `/arm/stop`, `/arm/torque`, MoveIt `move_action`).
- `center`: a MoveIt joint goal to the middle of each joint's driver limits (read from the driver's
  limits, not hard-coded), velocity scaling 0.3, then the gripper to the middle of its range. Before it
  moves: `/arm/home`. While the arm is held in a pose the owner asked for (center, or torque on after a
  move), `bag_arm` sets `rest_after_s` 0 and puts back the previous value on `rest`, `home` or a task.
- `GET /api/arm`: `/arm/state` plus the joint angles, for the page and `robotctl arm state`.
- Page: an Arm card on the Jetson tab with the five buttons and the state.

### Console gate in our tools

- The console starts every job with `HMC_CONSOLE_JOB=<job id>` in its environment (beside
  `HMC_SAFE_RUN`).
- `base/pi/console_gate.py`: `require(what)` raises with "start this through the console: robotctl ..."
  unless `HMC_CONSOLE_JOB` is set, the caller is the console itself, or `HMC_MAINTENANCE=<reason>` is set
  (then it appends a line to the command log and goes on).
- Called where a device is opened: `lidar.Lidar`, `camera.Camera`, `wheels.Wheels`, and in the Jetson arm
  entry points that move the arm from a shell (`bag_arm.py --pick/--place`, `floor_pick.py`, the arm
  team's `arm_cli` and `gcam.py`, `arm_client.ArmClient` when not inside `bag_arm`).
- Console checks today (`wheel`, `floor`) and the task run already go through jobs; the other tools
  (`base_check`, `approach`, `back_up`, `lap_check`, `tether_run`, `walk_check`, `air_check`) become
  console checks by name so they can still be run, through `robotctl check NAME [args]`, with an
  allow-list of names and argument patterns.

### Deploys and restarts

- `deploy.sh` asks first: `robotctl hold` (refused while a job runs or is starting); on success the console
  refuses new jobs until `robotctl release` or its own restart. Then the deploy, then `--restart`.
- The busy check covers a job that is starting (today's race: the job was not yet in `job`).

### The virtual operator app

- `remote.py` starts and stops runs only through `robotctl` (its direct `run_follow` fallback goes; with no
  console the app says so).

### Jetson programs

- Unchanged: the console's Jetson tab drives the agent. The agent keeps adopting a hand-started copy
  (`SHARED`), and the console marks such a program "started outside the console".

## Owner answers after the builder check (2026-11-16)

- **The arm team's moving tools are gated too.** `arm_cli`, `gcam.py`, the calibrate tools and
  `arm_client.ArmClient` outside bag_arm need the console or `HMC_MAINTENANCE` (logged on the Jetson). The
  gate is a small copy of `console_gate` inside the arm package (it cannot import `base/pi`), changed on
  `arm-system`.
- **Presence flags only from the page.** Flags that say someone is at the robot (`--yes-owner-at-robot`,
  `--yes-floor-run`, `--yes-wheels-raised`, `--yes-back-up`, `--yes-close-approach`) are accepted only from
  the console page, where a person presses the button; `robotctl` cannot send them.

## Calls made after the builder check (owner to check)

- **STOP stops the arm too.** The console's STOP calls `/arm/stop` through the agent as well as stopping the
  base, whoever started the arm move: everything goes through the console now, so an emergency stop must
  not leave the arm moving.
- **Busy means a job or the arm.** The console is busy while a job runs or is starting, or while bag_arm
  reports an arm command or a bag job running (`GET /arm` state). Arm buttons are refused during the whole
  task job (the bag phase drives the arm itself); `hold` is refused while the arm moves; while held, arm
  commands and Jetson start/stop are refused, STOP is not.
- **No console jobs on the Jetson for now.** The arm shell tools (`bag_arm.py --pick/--place`,
  `floor_pick.py`) are repair tools: they need `HMC_MAINTENANCE`, logged to `~/hmc-follow/logs/commands.jsonl`
  on the Jetson; the agent's `GET /status` carries the last lines and the console shows them.
- **Never above max_speed through the console.** `robotctl check` refuses any speed argument above
  `params.json` `max_speed` and raw packets (`air_check`); those stay maintenance tools.
- **Torque off only at rest.** `arm torque off` is refused unless `/arm/state` is `resting` (the arm drops
  when released elsewhere).
- Arm team code changes go on `arm-system` and reach the Jetson with `deploy.sh --arm`.

## Left to the builder

- Exact JSON shapes and HTTP codes of the new endpoints; the Arm card's layout.
- How `robotctl check NAME` validates each tool's arguments.
- Tests: each new endpoint, the gate (refuses, passes with the job id, passes with the logged override),
  the log line, `deploy.sh hold`, the arm relay with a fake `bag_arm`, `center` from fake limits.

## Ports (registered, no two programs share one)

| Host | Port | Program |
|---|---|---|
| Pi | 8081 | console |
| Pi | 8080 (127.0.0.1) | run_follow dashboard |
| Pi | 5603 | plan_server (run_follow's planner) |
| Pi | 5604 | command_server (run_follow --remote; base_bridge connects to it) |
| Pi | 5610 UDP | virtual operator datagrams |
| Jetson | 8090 | agent |
| Jetson | 8080 (127.0.0.1) | whisper server |
| Jetson | 5601 | detector |
| Jetson | 5602 | slam_bridge |
| Jetson | 5605 UDP (127.0.0.1) | base_bridge hold listener (agent asks for a hold) |
| Jetson | 5606 | front_camera bag measurements (bag_arm) |
| Jetson | 5607 (127.0.0.1) | bag_arm commands from the agent (new) |
| Jetson | 8082 | arm team webcam |

A test reads every `PORT` constant in `base/` and fails on a duplicate per host.
