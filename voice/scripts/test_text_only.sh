#!/usr/bin/env bash
# Test LLM + TTS + bridge without a microphone
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /speech/text std_msgs/msg/String "{data: 'Hello robot, please move forward one meter'}"
echo "--- /robot/command (Ctrl+C to quit) ---"
ros2 topic echo /robot/command
