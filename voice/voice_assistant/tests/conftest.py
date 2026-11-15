"""Make the package importable when pytest runs from voice/voice_assistant.

There is no colcon build on this machine (and no rclpy), so the package directory itself is put on
sys.path: the tests can then import the pure modules (rules, speech_rules, flow_logic) without ROS.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
