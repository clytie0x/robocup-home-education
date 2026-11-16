# Voice stack assessment and handoff, 2026-11-13

Written in a cloud session that could not reach the Jetson or the Pi. Nothing in `voice/` was changed;
the only new file besides this one is `scripts/collect_info.sh`.

**Current goal (owner, 2026-11-13): get the robot moving correctly first.** Voice work comes after that.

## How this was checked

- Read on `main` (commit `ce22168`): `flow_node`, `rule_node`, `llm_node`, `stt_node`, `tts_node`,
  `command_bridge_node`, `arm_action_bridge`, the launch file, `params.yaml`, the docs.
  Not read: `dialog_node`, `fake_arm_server`, both sims, most of `control_panel.py`.
- `RuleParser`, `FlowLogic` and the `llm_node` fast path were run with ROS stubbed out (plain Python).
  Audio, the real LLM, the arm and the base were not tested.
- PR #3 (`sim/robustness-ladder`, adds `hmc/` and `base/`): read the description, the file list and parts of
  four docs only. Its code was not reviewed and its tests were not run here (the run was interrupted).

## Findings in `voice/` (by risk)

1. **Negation beats "stop" in rules mode.** `"Stop, don't go there"`, `"Never mind, stop"`,
   `"Please do not hit the wall, stop"` all parse to `none` ("Okay, I won't.") and the robot does not stop
   (`rule_node.py`, the `NEG` check runs before the rules).
2. **LLM mode: many stop sentences miss the fast path.** `"No, stop!"`, `"Okay stop"`, `"Can you stop"`
   go to the LLM (1 to 2 s); if llama-server is down the intent is `none`.
3. **Stop always goes through the brain node.** stt -> flow -> brain -> flow -> bridge, barge-in included.
   If `llm_node` / `rule_node` is dead, no stop reaches the wheels. `flow_node` should match stop itself on
   `/speech/heard`.
4. **Turn angle is not clamped.** `command_bridge_node` clamps distance (2 m) and speed, not the angle:
   `"turn 3600 degrees"` passes the parser and the flow and turns for about 105 s. A very low speed also
   gives a very long move.
5. **`see_person` never expires.** If the person detector dies without `lost_person`, the flow stays in
   `following` forever; a `see_person` from 10 minutes ago still lets following start.
6. **`carrying` goes wrong in `arm:=topics` mode.** A `grab_success` after the 30 s timeout is dropped (robot
   holds the bag, says "I don't have a bag"); "stop" during a drop followed by `drop_done` leaves
   `carrying` true. `arm:=actions` corrects itself through `arm_holding` / `arm_empty`.
7. **Control panel has no auth** and listens on `0.0.0.0:8090`.
8. **Docs behind the code.** `TEST_PLAN_VM.md` level 2 does not start `flow_node`, but `llm_node` now only
   publishes `/robot/intent`, so the turtle would not move; it also expects 4 executables (there are 10).
   The README diagram, the `llm_node` docstring and the "Status" section of `CLAUDE.md` describe the old design.
9. Minor: `"Okay go ahead"` -> move forward 0.5 m; `"Move forward a meter"` -> 0.5 m; intents during
   `error` are dropped silently; `follow_me` / `go_to` / `look_at` are TODO in the bridge; `dialog_node`
   is no longer launched; there are no automated tests although `FlowLogic`, `RuleParser` and
   `DialogLogic` are pure.

## What PR #3 changes for voice

Source: `base/docs/hardware-assessment-2026-11-13.md` and `base/docs/sim-to-robot-checklist.md` on that branch.

- **No link between voice and the follower yet.** `base/pi/run_follow.py` is plain Python (TCP to the Jetson
  on port 5601), not ROS. Nothing in `base/` takes `/robot/command follow_me` or sends `see_person` /
  `lost_person`. Finding 5 becomes "design that bridge", not "add a timeout".
- **Base top speed is 0.12 to 0.13 m/s whatever is commanded.** `command_bridge_node` defaults to 0.15 m/s
  and times moves open-loop, so "move forward 1 m" would cover about 0.8 m.
- **The follower and the ROS driver cannot use the base at the same time** (measurements stopped ROS on the
  Pi first), so voice move / turn over `/cmd_vel` does not work while the follower runs.
- **Jetson RAM.** 7.3 GB total; 4.6 GB was in use (rviz2 + MoveIt) when measured; the person detector adds
  about 0.74 GB; voice in LLM mode 2.1 to 2.5 GB. All together does not fit. Rules mode saves about 1.4 GB.
- **The Jetson is not on the robot** (power, Wi-Fi only), so the mic and speaker are not either.

## Open for "move correctly" (from PR #3's plan; all need the owner at the robot)

- Speed cap of 0.12 m/s: firmware or motor supply (wheels-raised test).
- Turn test never ran (`base_check.py --turn` was refused for clearance); turn odometry under `pi-drive`
  is an open gate.
- `camera_pitch_deg` is null; `lidar_front_deg` 178.7 vs 180.3 unresolved.
- Plan tasks 5.3, 6.3, 11.4, 11.5 and 12 to 15 (first drives and scenarios).

## Stages proposed for voice (after the base)

| Stage | Work | Needs the boards |
|---|---|---|
| 0 | Run `scripts/collect_info.sh` on the Jetson with the voice stack up (ROS graph, audio devices, servers, RAM) | yes |
| 1 | Unit tests for `FlowLogic` and `RuleParser`, including failing tests for findings 1, 2, 4, 5, 6 | no |
| 2 | Fix the stop path (1 to 3), clamp turn angle and duration (4), set bridge speeds to the real 0.12 m/s | no |
| 3 | Design the voice <-> follower bridge (`follow_me`, `stop`, `see_person`, `lost_person`, texts in `hmc/src/hmc/hri.py`) | owner decision |
| 4 | Bring the voice docs in line with the code | no |
| 5 | Measure total Jetson RAM with detector + voice + arm; choose LLM or rules | yes |
| 6 | End-to-end test on the robot | yes |

## Stage 0 on the Jetson

```bash
cd ~/robocup-home-education
git fetch origin ccr-4d0c3df0-8j5ook
git show FETCH_HEAD:voice/scripts/collect_info.sh | bash     # writes ~/stage0_<hostname>.txt
```

Read-only, no sudo. Only dry-run in a container without ROS or audio so far.

## Model choice for sub-agents (owner's rule)

- Hard, high-risk, irreversible (detector review before freezing it, flow / stop / safety logic): the
  strongest model.
- Code or text to a clear spec (tests, docs, control panel): Sonnet.
- Mechanical work (summaries, format checks, tables): Haiku.
- A few commands' worth of work: no sub-agent.
