import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('voice_assistant'), 'config', 'params.yaml')
    # Machine-specific settings (mic/speaker index, volume, ...) live OUTSIDE the package so that
    # updating the code never overwrites them. Loaded after params.yaml, so its values win.
    local = os.path.expanduser('~/.robot_voice_local.yaml')
    if os.path.isfile(local):
        params = [params, local]
    else:
        params = [params]
    use_bridge = LaunchConfiguration('use_bridge')
    use_mic = LaunchConfiguration('use_mic')
    brain = LaunchConfiguration('brain')
    arm = LaunchConfiguration('arm')
    arm_topics = IfCondition(PythonExpression(["'", use_bridge, "' == 'true' and '", arm, "' == 'topics'"]))
    arm_actions = IfCondition(PythonExpression(["'", use_bridge, "' == 'true' and '", arm, "' == 'actions'"]))

    return LaunchDescription([
        DeclareLaunchArgument('use_bridge', default_value='true',
                              description='start the example /robot/command -> /cmd_vel bridge'),
        DeclareLaunchArgument('brain', default_value='llm',
                              description='llm = llama-server (smarter, ~1 s) | rules = keyword rules (instant, no LLM)'),
        DeclareLaunchArgument('arm', default_value='topics',
                              description='topics = old /robot/carry_bag, /robot/drop_bag | '
                                          'actions = SO-100 API /arm/pick, /arm/place (needs so100_interfaces)'),
        DeclareLaunchArgument('use_mic', default_value='true',
                              description='false = test by typing: ros2 topic pub /speech/heard (with wake word) or /speech/text (direct)'),
        Node(package='voice_assistant', executable='stt_node', name='stt_node',
             parameters=params, output='screen', condition=IfCondition(use_mic)),
        # flow_node = the robot's task state machine (team state chart). Replaces dialog_node (wake word).
        Node(package='voice_assistant', executable='flow_node', name='flow_node',
             parameters=params, output='screen'),
        Node(package='voice_assistant', executable='llm_node', name='llm_node',
             parameters=params, output='screen',
             condition=IfCondition(EqualsSubstitution(brain, 'llm'))),
        Node(package='voice_assistant', executable='rule_node', name='rule_node',
             parameters=params, output='screen',
             condition=IfCondition(EqualsSubstitution(brain, 'rules'))),
        Node(package='voice_assistant', executable='tts_node', name='tts_node',
             parameters=params, output='screen'),
        # wheels + old bag topics
        Node(package='voice_assistant', executable='command_bridge_node', name='command_bridge_node',
             parameters=params, output='screen', condition=arm_topics),
        # wheels only (bag topics off) + SO-100 actions
        Node(package='voice_assistant', executable='command_bridge_node', name='command_bridge_node',
             parameters=params + [{'bag_topics': False}], output='screen', condition=arm_actions),
        Node(package='voice_assistant', executable='arm_action_bridge', name='arm_action_bridge',
             parameters=params, output='screen', condition=arm_actions),
    ])
