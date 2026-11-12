"""Simulation side (run on the VM, which has a screen):
turtlesim + command_bridge_node (-> /turtle1/cmd_vel) + bag simulator.
The voice side runs on the Jetson with:  ros2 launch voice_assistant voice_assistant.launch.py use_bridge:=false
For a single machine with a screen, start both launch files there.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('voice_assistant'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim', output='screen'),
        Node(package='voice_assistant', executable='command_bridge_node', name='command_bridge_node',
             parameters=[params, {'cmd_vel_topic': '/turtle1/cmd_vel'}], output='screen'),
        Node(package='voice_assistant', executable='turtlesim_bag_sim', name='turtlesim_bag_sim',
             output='screen'),
    ])
