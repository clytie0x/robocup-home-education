"""Help Me Carry voice stack: stt_node, tts_node and rule_node (rules brain, no LLM).

The task flow for Help Me Carry is base/jetson/hmc_flow.py, not voice's flow_node: hmc_flow listens on
/speech/heard and /robot/intent and owns /tts/say and /robot/command. So flow_node and
command_bridge_node must not run next to it (they would publish /tts/say and /robot/command twice) and
llm_node is not needed (the rules brain answers in milliseconds, ~1.4 GB RAM less).

Order: the whisper server first (scripts/servers.sh start-stt), then
    ros2 launch voice_assistant hmc_voice.launch.py
See "Help Me Carry" in voice/README.md.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("voice_assistant"), "config", "params.yaml")
    # Machine-specific settings (mic/speaker index, volume, ...) live OUTSIDE the package so that
    # updating the code never overwrites them. Loaded after params.yaml, so its values win.
    local = os.path.expanduser("~/.robot_voice_local.yaml")
    params = [params, local] if os.path.isfile(local) else [params]
    use_mic = LaunchConfiguration("use_mic")

    return LaunchDescription([
        DeclareLaunchArgument("use_mic", default_value="true",
                              description='false = no microphone: type the sentences, '
                                          'ros2 topic pub /speech/heard std_msgs/String "{data: \'follow me\'}"'),
        Node(package="voice_assistant", executable="stt_node", name="stt_node",
             parameters=params, output="screen", condition=IfCondition(use_mic)),
        # rules brain: it reports what the user wants on /robot/intent, it never drives the robot itself.
        # task_words = the Help Me Carry words (this is the car, go back to the start, ready, ...)
        # speak_chat off whatever ~/.robot_voice_local.yaml says: hmc_flow alone speaks on /tts/say, a chat reply
        # from the brain would talk over its lines ("Please point at the bag.")
        Node(package="voice_assistant", executable="rule_node", name="rule_node",
             parameters=params + [{"task_words": True, "speak_chat": False}], output="screen"),
        Node(package="voice_assistant", executable="tts_node", name="tts_node",
             parameters=params, output="screen"),
    ])
