# Help Me Carry on the real robot: integration design

Status: draft, 2026-11-14. Defaults chosen by Claude; every "Decision" line is the owner's to change.

## Why this order

Score sheet (hmc/docs/tasks/5.1-help-me-carry.md):

| Group | Rows | Points |
|---|---|---|
| Following to the car | follow_to_car | 300 |
| Obstacles | avoid_* (four rows) | 200 |
| The way back | re_reach_start, queue | 500 |
| The bag | perceive_bag, pick_bag | 115 |

Without the arm, following, avoiding obstacles, returning and queueing score 1000 of 1115.
So the first slice joins what already works on the Pi (following, return trip, queue) to the
voice team's task flow. The bag comes second.

## What exists

| Where | What | State |
|---|---|---|
| Pi `base/pi/run_follow.py` | follows the operator, phases FOLLOW, RETURN and DONE, breadcrumbs, keyframes, queue | deployed; FOLLOW field-driven; RETURN not yet on the floor |
| Pi firmware 0c020e4 | true speeds, turns while driving, updates over U# | flashed |
| Jetson `base/jetson/server.py` | person detector and looks on port 5601 for the Pi | running |
| Jetson `voice/` (ROS 2, domain 42) | flow_node (listening, picking_bag, announce_ready, following, dropping_bag, error); rule and LLM brains; arm_action_bridge; TTS and STT | not run on the real board yet (voice/CLAUDE.md) |

Missing pieces:
- Nothing joins `voice` to the Pi. The command bridge only logs `follow_me` as a TODO.
- No `see_person` or `lost_person` events are published, except by simulators.
- The flow has no way-back state.
- There are no intents for "go back" or "wait".
- There is no pointing or bag pose.

## Decisions (defaults)

- D1, the task flow. A separate `HmcFlow` in `base/jetson/hmc_flow.py` holds the Help Me Carry
  states. It reuses the voice topics (`/speech/heard`, `/robot/intent`, `/tts/say`,
  `/robot/command`, `/robot/event`) and is launched instead of the generic `flow_node` for this
  task. The voice team's generic flow is not changed. Alternative: add the states to `FlowLogic`
  in `voice/`, which is the voice team's code.
- D2, the Pi link. `run_follow.py --remote` replaces the Enter key with a small TCP command server
  on the Pi (port 5604, one client, JSON lines). Commands: `follow`, `wait`, `resume`, `return`,
  `stop`. Status goes back at 5 Hz: phase, locked, operator distance, breadcrumbs, queue tail,
  `at_start`, and the stop reason.
  - The wheels' safety (must_stop, coast, slow cycle, velocity loop) is unchanged.
  - A lost link keeps the robot's current phase, but the robot holds still after 2 s without a
    status ack.
- D3, the bridge. `base/jetson/base_bridge.py` is a ROS 2 node on domain 42.
  - It turns `/robot/command` `follow_me`, `stop`, `go_back` and `wait` into Pi commands.
  - It publishes `/robot/event`: `see_person` and `lost_person` from the lock state; `at_start`
    when the robot is back at the start; `base_link_lost` when the Pi link drops.
  - It also publishes `/base/status` (JSON).
  - It holds no logic of its own, so it is tested with a fake socket and a fake ROS publisher.
- D4, the start signal. The robot starts when the operator says "follow me" (rule brain). A
  start-button alternative costs 100 (alt_start_signal), so it is a fallback only.
- D5, the bag (second slice).
  - The robot asks the operator to point, and says which bag it sees, by colour or position.
  - The arm then picks with the pose from a new bag detector on the Jetson, publishing
    `/object/pose`, which arm_action_bridge already reads.
  - Until that works, the flow asks the operator to put the bag in the gripper. This keeps the way
    back and the queue (500 points) scoring, at the cost of pick_bag.

## HmcFlow states

`ready` → (intent follow_me) say "I will follow you." → `following` (Pi `follow`)
- `following`:
  - on `lost_person` for 5 s: say "Please come back." (dem_come_back -150 only if it then scores).
  - on intent `stop` or "this is the car" or "here": Pi `wait`, then go to `at_car`.
- `at_car`:
  - say "Please point at the bag." Wait for the bag pose (slice 2), or ask for a handover
    (slice 1).
  - On `grab_success` or after the handover: say "I will go back to the start now.", Pi `return`,
    then go to `returning`.
- `returning`:
  - the Pi does the breadcrumbs, the queue and the start heading by itself (run_follow RETURN);
  - on `at_start`: arm `drop_bag` (place on the floor), say "Here is your bag.", then `done`.
- `error` from any state on `base_link_lost` or an arm failure: Pi `stop`, say why, wait for "follow me" again.

## Tests

- HmcFlow: pure logic, unit tests for each transition, and a full scripted run with fake events.
- run_follow `--remote`: command server tests with fake devices, as the Return tests do.
- base_bridge: a fake socket and a fake ROS publisher.
- On the Pi: a dry run with the wheels off, commands sent by hand.
- Then floor runs with the owner: follow, wait, return, queue (return-trip Tasks 6 and 7).

## Not in this slice

Pointing detection, the bag detector, the "wait" voice intent in the voice brains (HmcFlow can
also match the words itself), and outside-arena runs (person-follow Task 14).
