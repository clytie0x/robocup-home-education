# robocup-home-education

Team repository for the RoboCup@Home Education robot (ROS 2 Jazzy, Jetson Orin Nano).

| Folder | What | Owner |
|---|---|---|
| `voice/` | Offline voice interface (whisper.cpp STT → rule/LLM brain → Piper TTS), task state chart, arm bridge, web control panel | voice team |
| `arm/` (not yet) | SO-100 arm stack (`so100_moveit_config`, `so100_interfaces`) | arm team |

See [`voice/README.md`](voice/README.md) for setup.
