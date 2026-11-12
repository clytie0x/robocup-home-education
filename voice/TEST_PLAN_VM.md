# Test plan: voice_assistant on the ROS 2 VM (for Claude Code)

User speaks Thai: report results in Thai. Code/commands in English.

## Environment
- This machine: VirtualBox VM `ros2-for-robot`, Ubuntu 24.04, ROS 2 Jazzy desktop (turtlesim, rqt), NAT network, user `ros2`.
- No GPU/CUDA here. Do NOT install whisper.cpp / llama.cpp / Piper / sounddevice in this VM.
- Jetson (real robot brain): `robotpsu@192.168.31.137`. It runs whisper-server :8080 and llama-server :8081 bound to 127.0.0.1.
- Package under test: `~/robot_voice/voice_assistant` (ament_python). See `~/robot_voice/CLAUDE.md` for the architecture.
- `sudo` needs a password: ask the user to run sudo commands themselves (prefix `!`).

## Rules for running tests
- Run every long-lived process in the background and log to `~/va_test/logs/<name>.log`; kill them all at the end (`pkill -f turtlesim_node; pkill -f command_bridge_node; pkill -f llm_node`).
- turtlesim needs a display. Run it headless: `QT_QPA_PLATFORM=offscreen ros2 run turtlesim turtlesim_node`.
- Verify motion from `/turtle1/pose` (x, y, theta), not by looking. Read a pose with:
  `ros2 topic echo --once /turtle1/pose`
- Before each movement test: reset with `ros2 service call /reset std_srvs/srv/Empty` (turtle back to x=5.544, y=5.544, theta=0) and restart the bridge if needed.
- Wait long enough for timed moves to finish (duration = distance/speed, turn = angle_rad/0.6) + 1 s margin, then read pose and `/robot/state`.
- Do not change the package code to make a test pass. If a test fails, record it, then propose a fix separately and ask before editing.
- Write results to `~/va_test/RESULTS.md` (table: id, input, expected, actual, PASS/FAIL, notes) and give the user a short Thai summary.

Note on units: turtlesim units are not real meters; checking direction and distance ratios is enough. Tolerances: distance ±10 %, angle ±8°.

## Setup (once)
```bash
sudo apt install -y unzip python3-requests      # user runs this (sudo)
mkdir -p ~/ros2_ws/src ~/va_test/logs
cp -r ~/robot_voice/voice_assistant ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 pkg executables voice_assistant   # expect 4 executables
```

## Level 1: command_bridge_node + turtlesim (no Jetson)
Start:
```bash
QT_QPA_PLATFORM=offscreen ros2 run turtlesim turtlesim_node > ~/va_test/logs/turtlesim.log 2>&1 &
ros2 run voice_assistant command_bridge_node --ros-args -p cmd_vel_topic:=/turtle1/cmd_vel > ~/va_test/logs/bridge.log 2>&1 &
```
Send a command:
```bash
ros2 topic pub --once /robot/command std_msgs/msg/String "{data: '{\"action\":\"move_forward\",\"params\":{\"distance\":1.0}}'}"
```
Defaults in the bridge: default_speed 0.15, max_speed 0.3, turn_speed 0.6 rad/s, max_distance 2.0.

| id | /robot/command JSON | expected |
|---|---|---|
| 1.1 | move_forward distance 1.0 | x increases ≈1.0, y,theta unchanged; /robot/state `move_forward 1.00 m` then `idle` |
| 1.2 | turn angle 90 | theta ≈ +1.571 rad (left), position unchanged |
| 1.3 | turn angle -45 | theta decreases ≈0.785 rad (right) |
| 1.4 | move_backward distance 0.5 | x decreases ≈0.5 |
| 1.5 | move_forward distance 10, speed 5 | clamped: moves ≈2.0 at 0.3 → finishes in ≈6.7 s |
| 1.6 | 1.5 then `stop` after 2 s | moved clearly < 2.0, state `stopped`, turtle stays still afterwards |
| 1.7 | go_to target kitchen | no motion, state `go_to not implemented yet` |
| 1.8 | `not json` (raw string) | warning `bad command` in bridge.log, node still alive (`ros2 node list`) |
| 1.9 | move_forward with no params | uses defaults: distance 0.5 at 0.15 |
| 1.10 | drop_bag (send during a move_forward 2.0) | turtle stops immediately, one `std_msgs/Empty` on `/robot/drop_bag` (`ros2 topic echo /robot/drop_bag`), state `bag dropped` |

## Level 2: llm_node (in VM) → Jetson LLM → bridge → turtlesim
Precondition (user does this, needs the Jetson password): on the Jetson `~/robot_voice/scripts/start_servers.sh`; in a separate VM terminal keep open:
```bash
ssh -N -L 8081:127.0.0.1:8081 robotpsu@192.168.31.137
```
Check: `curl -s http://127.0.0.1:8081/health` → `{"status":"ok"}`. If it fails, stop and ask the user to open the tunnel.

Start (turtlesim + bridge as in level 1), plus:
```bash
ros2 run voice_assistant llm_node > ~/va_test/logs/llm.log 2>&1 &
ros2 topic echo /llm/response > ~/va_test/logs/llm_response.log 2>&1 &
```
Send:
```bash
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'Move forward one meter'}"
```
Wait for the `llm_node` log line `(x.xx s) action=...` before the next sentence. Record latency.

| id | sentence | expected action / params |
|---|---|---|
| 2.1 | Hello, who are you? | none, friendly short reply, no motion |
| 2.2 | Move forward one meter | move_forward, distance 1 |
| 2.3 | Turn left ninety degrees | turn, angle +90 |
| 2.4 | Turn right a little | turn, angle negative (check the sign) |
| 2.5 | Go back half a meter slowly | move_backward, distance 0.5, low speed |
| 2.6 | Stop! | stop |
| 2.7 | Go to the kitchen | go_to, target kitchen |
| 2.8 | What is the weather today? | none, does not invent weather |
| 2.9 | Fly to the moon | none (must not invent an action) |
| 2.11 | Drop the bag / Put the bag down / Release it | drop_bag (all three phrasings) |
| 2.12 | Pick up the bag / Help me carry this bag | carry_bag (fast path), not drop_bag |
| 2.13 | Don't drop the bag | not drop_bag (negation skips the keyword fast path) |
| 2.10 | Move forward two meters then turn around | note what it does (only one action per reply is supported) |

Also check: every `/llm/response` parses as JSON with keys reply/action/params; reply ≤ 2 sentences, no markdown/emoji.
Run 2.2–2.4 three times each to check consistency.

## Level 3: manual (not for Claude Code)
Needs VirtualBox network changed to Bridged and someone speaking into the Jetson mic. Claude Code only helps with:
- checking cross-machine discovery after the change (`ros2 run demo_nodes_py listener` while the Jetson runs `talker`, `ROS_DOMAIN_ID` equal on both)
- running turtlesim + bridge here and logging `/turtle1/pose` + `/robot/state` while the user talks.

## Report
At the end: `~/va_test/RESULTS.md` + Thai summary: number passed/failed per level, LLM latency (min/avg/max), list of failures with the likely cause and a suggested fix (do not apply fixes without asking).
