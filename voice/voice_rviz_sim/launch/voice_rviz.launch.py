"""RViz view of the voice-controlled robot.

  ros2 launch voice_rviz_sim voice_rviz.launch.py
  ros2 launch voice_rviz_sim voice_rviz.launch.py use_bridge:=false   # bridge runs elsewhere
  ros2 launch voice_rviz_sim voice_rviz.launch.py software_gl:=true   # VirtualBox black screen fix
  ros2 launch voice_rviz_sim voice_rviz.launch.py simulate_person:=false   # test "I can't see you"
  ros2 launch voice_rviz_sim voice_rviz.launch.py simulate_events:=false   # real gripper sends events
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('voice_rviz_sim')
    rviz_cfg = os.path.join(share, 'rviz', 'voice_robot.rviz')

    cmd_vel = LaunchConfiguration('cmd_vel_topic')
    use_bridge = LaunchConfiguration('use_bridge')
    use_rviz = LaunchConfiguration('rviz')
    software_gl = LaunchConfiguration('software_gl')
    sim_events = LaunchConfiguration('simulate_events')
    sim_person = LaunchConfiguration('simulate_person')

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_bridge', default_value='true',
                              description='start voice_assistant command_bridge_node here'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('software_gl', default_value='false',
                              description='LIBGL_ALWAYS_SOFTWARE=1 for VirtualBox'),
        DeclareLaunchArgument('simulate_events', default_value='true',
                              description='fake grab_success / drop_done on /robot/event'),
        DeclareLaunchArgument('simulate_person', default_value='true',
                              description='fake see_person every second on /robot/event'),

        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                               condition=IfCondition(software_gl)),

        Node(package='voice_rviz_sim', executable='rviz_robot_sim', output='screen',
             parameters=[{'cmd_vel_topic': cmd_vel,
                          'simulate_events': ParameterValue(sim_events, value_type=bool),
                          'simulate_person': ParameterValue(sim_person, value_type=bool)}]),

        Node(package='voice_assistant', executable='command_bridge_node', output='screen',
             parameters=[{'cmd_vel_topic': cmd_vel}],
             condition=IfCondition(use_bridge)),

        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             output='screen', condition=IfCondition(use_rviz)),
    ])
