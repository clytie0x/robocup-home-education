# Project context: Robot Voice (Jetson Orin Nano + ROS 2 Jazzy)

User speaks Thai — reply in Thai. Code comments in English.

## Goal
Offline voice interface for a mobile robot: STT -> LLM -> TTS, English speech,
total RAM (CPU+GPU, unified memory) must stay under 4 GB. LLM output is sent to
ROS 2 Jazzy as commands.

## Hardware / OS
- Jetson Orin Nano, user `robotpsu`, accessed over SSH from Windows (no GUI needed)
- Target: JetPack 7.2 (L4T R39, Ubuntu 24.04, CUDA 13.2, sm_87) so Jazzy installs via apt.
  If `cat /etc/nv_tegra_release` shows R36 (JetPack 6 / 22.04) -> need reflash or Jazzy in Docker.

## Architecture
- whisper.cpp `whisper-server` :8080 (base.en q5_1, CUDA) ~0.3 GB
- llama.cpp `llama-server` :8081 (Qwen2.5-1.5B-Instruct Q4_K_M, -c 2048 -np 1 --no-mmap --jinja) ~1.4 GB
- Piper TTS in-process (en_US-lessac-medium, CPU) ~0.1 GB
- Models + builds live in /opt/robot_voice (see scripts/install_jetson.sh)

ROS 2 package `voice_assistant` (ament_python), nodes:
- stt_node: LoudnessGate (adaptive background noise, min_snr) before whisper; robust mic open (mono->stereo, rate fallback, lists devices + retries)
  sounddevice mic -> webrtcvad -> POST whisper /inference (vocabulary prompt) -> /speech/heard
- flow_node (LAUNCHED): team state chart, no wake word. listening/picking_bag/announce_ready/following/dropping_bag/error.
  Brains publish /robot/intent; flow decides speech + /robot/command. Robot feedback on /robot/event
  (grab_success, grab_failed, drop_done, drop_failed, see_person, lost_person, bag_lost). FlowLogic is pure/unit-testable.
- dialog_node (NOT launched, wake-word version): wake word ("hey robot") -> greeting + capabilities, awake window (15 s), farewell -> sleep;
  wake_chime/sleep_chime (sounds/*.wav via /tts/play), intro_once; 'stop' always passes; forwards sentences to /speech/text; publishes /dialog/state. Logic in DialogLogic (no ROS, unit-testable)
- llm_node: /speech/text -> llama /v1/chat/completions with json_schema
  {"reply","action","params"} -> /tts/say, /robot/command (JSON String), /llm/response
- rule_node: no-LLM alternative to llm_node (keyword/regex RuleParser, same topics, ~ms). Launch arg brain:=llm|rules
- barge-in: stt_node BargeInDetector (echo p90 x barge_ratio) while muted -> whisper -> stop/halt/freeze -> /tts/interrupt + 'stop'
- arm_action_bridge (launch arm:=actions): /robot/command carry_bag/drop_bag -> SO-100 /arm/pick,/arm/place actions
  (needs so100_interfaces in workspace); fixed test pose (tape-measure forward/left) or /object/pose; result -> /robot/event
  'grab_success' | 'grab_failed|CODE|message'. fake_arm_server = same API without hardware. command_bridge stop -> /arm/stop.
- tts_node: /tts/interrupt -> sd.stop + drop queue; /tts/say -> Piper -> speaker; publishes /tts/speaking (Bool) to mute STT
- command_bridge_node: /robot/command -> timed /cmd_vel Twist (move/turn/stop, speed-clamped);
  carry_bag / drop_bag -> stop + std_msgs/Empty on /robot/carry_bag, /robot/drop_bag (gripper node);
  llm_node has a keyword fast path for stop / drop_bag / carry_bag; stt_node sends a whisper vocabulary prompt;
  go_to / follow_me / look_at are TODO (Nav2 etc.); publishes /robot/state

Config: voice_assistant/config/params.yaml (+ machine-specific overrides in ~/.robot_voice_local.yaml, loaded last).
beatbox action: rule/llm node -> /tts/say + /tts/play (path, beatbox_file = ~/robot_voice/sounds/cat_beatbox.wav); tts_node plays it with the mic muted. Launch: voice_assistant.launch.py
(args use_mic, use_bridge).

## Setup steps
1. scripts/install_jetson.sh  (build whisper.cpp + llama.cpp with CUDA, download models, pip deps)
2. scripts/servers.sh start (llm) | start-stt (rules, whisper only) | stop | status
3. copy voice_assistant to ~/ros2_ws/src, colcon build
4. set mic/speaker index in params.yaml (`python3 -m sounddevice`)
5. test: launch with use_mic:=false + scripts/test_text_only.sh, then with mic

## Status
See docs/assessment-2026-11-13.md for the latest findings, open risks and the staged plan (read it first).
Current goal (owner, 2026-11-13): get the robot moving correctly first; voice work comes after.
Verify RAM with `tegrastats` while all parts are running.
