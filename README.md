# robocup-home-education

Team repository for the RoboCup@Home Education robot (ROS 2 Jazzy, Jetson Orin Nano).

| Folder | What | Owner |
|---|---|---|
| `voice/` | Offline voice interface (whisper.cpp STT → rule/LLM brain → Piper TTS), task state chart, arm bridge, web control panel | folkza123 |
| `arm/` | SO-100 arm stack: ROS 2 packages `so100_driver`, `so100_interfaces`, `so100_moveit_config`, and `tools/bagpick` | hassan |
| `hmc/` | Help Me Carry (rulebook 5.1) slice 1: a plain-Python 2D simulation with a state machine and the scoresheet | unassigned |
| `base/` | Wheeled base (Raspberry Pi, STM32, LiDAR, camera) and person following on the real robot | Teerapat |

See [`voice/README.md`](voice/README.md), [`hmc/README.md`](hmc/README.md) and [`arm/README.md`](arm/README.md) for setup.
