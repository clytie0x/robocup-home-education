"""SO-100 bring-up.

  ros2 launch so100_moveit_config arm.launch.py                    # real arm + MoveIt + RViz
  ros2 launch so100_moveit_config arm.launch.py hardware:=sim      # no arm, simulated follower
  ros2 launch so100_moveit_config arm.launch.py mode:=sliders      # real arm follows joint sliders
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import xml.etree.ElementTree as ET


def real_limits(urdf_xml, calib_path):
    """Set URDF joint limits to the real arm's measured travel (calibration.json) +-1 deg.

    MoveIt must accept every pose the real arm can be in (e.g. folded against the hard stop),
    otherwise it refuses to plan ("start state out of bounds"). The driver itself still keeps
    ~2 deg away from each mechanical end, so a goal at the edge just stops 1-3 deg short.
    """
    import math
    from so100_driver.calib import JOINTS, Calibration
    cal = Calibration.load(calib_path)
    tol = math.radians(1.0)
    root = ET.fromstring(urdf_xml)
    for j in root.findall('joint'):
        lim = j.find('limit')
        name = j.get('name')
        if name in JOINTS and lim is not None:
            d = cal.data[name]
            lo, hi = sorted(cal.to_rad(name, r) for r in (d['min'], d['max']))
            lim.set('lower', f'{lo - tol:.4f}')
            lim.set('upper', f'{hi + tol:.4f}')
    return ET.tostring(root, encoding='unicode')


def setup(context):
    hw = LaunchConfiguration('hardware').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    share = get_package_share_directory('so100_moveit_config')
    mc = (MoveItConfigsBuilder('so_arm100', package_name='so100_moveit_config')
          .robot_description(file_path='urdf/so100.urdf')
          .robot_description_semantic(file_path='config/so100.srdf')
          .robot_description_kinematics(file_path='config/kinematics.yaml')
          .joint_limits(file_path='config/joint_limits.yaml')
          .trajectory_execution(file_path='config/moveit_controllers.yaml')
          .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
          # OMPL is the default: works with RViz's empty planner id, and in free space its simplified
          # paths are straight in joint space (checked 9/9). The judder came from very low
          # acceleration (slow ramps), fixed in joint_limits.yaml. Pilz (PTP/LIN) is selectable
          # in RViz's Context tab and for scripts (planner_id='PTP').
          .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'],
                              default_planning_pipeline='ompl')
          .to_moveit_configs())
    if hw == 'real':
        # MoveIt, RViz sliders and the driver all see the real calibrated ranges.
        calib = os.path.expanduser(LaunchConfiguration('calibration').perform(context))
        mc.robot_description['robot_description'] = real_limits(mc.robot_description['robot_description'], calib)
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[mc.robot_description]),
        Node(package='so100_driver', executable='driver', output='screen',
             parameters=[mc.robot_description,
                         {'sim': hw == 'sim', 'max_speed': 0.8 if mode == 'sliders' else 1.5,
                          'calibration': os.path.expanduser(LaunchConfiguration('calibration').perform(context))}]),
    ]
    if mode == 'sliders':
        nodes.append(Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                          remappings=[('joint_states', 'so100/joint_command')],
                          parameters=[mc.robot_description]))
        rviz_cfg = f'{share}/config/view.rviz'
    else:
        nodes.append(Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                          parameters=[mc.to_dict(), {'publish_robot_description_semantic': True}]))
        # Table top just under the base so MoveIt never plans through it (MoveIt mode only).
        nodes.append(Node(package='so100_driver', executable='scene_setup', output='screen',
                          parameters=[{'table_z': float(LaunchConfiguration('table_z').perform(context))}]))
        # IK pick-and-place test: draggable object/target boxes in RViz + buttons in the control panel.
        nodes.append(Node(package='so100_driver', executable='pick_place', output='screen',
                          parameters=[mc.robot_description,
                                      {'table_z': float(LaunchConfiguration('table_z').perform(context)),
                                       'sim': hw == 'sim',
                                       'calibration': os.path.expanduser(
                                           LaunchConfiguration('calibration').perform(context)) if hw == 'real' else ''}]))
        rviz_cfg = f'{share}/config/moveit.rviz'
        if LaunchConfiguration('camera').perform(context) == 'true':
            nodes += camera_nodes(float(LaunchConfiguration('table_z').perform(context)))
    if rviz:
        nodes.append(Node(package='so100_driver', executable='control_panel'))
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
                          parameters=[mc.robot_description, mc.robot_description_semantic,
                                      mc.robot_description_kinematics, mc.planning_pipelines, mc.joint_limits]))
    return nodes


def camera_nodes(table_z):
    """ASUS Xtion (openni2) -> /camera/depth_registered/points, plus the camera test node.
    The camera's own TF tree is not used: camera_test publishes base -> camera frame after calibration."""
    container = ComposableNodeContainer(
        name='container', namespace='camera', package='rclcpp_components', executable='component_container',
        output='screen', composable_node_descriptions=[
            ComposableNode(package='openni2_camera', plugin='openni2_wrapper::OpenNI2Driver', name='driver',
                           namespace='camera',
                           parameters=[{'depth_registration': True, 'use_device_time': False,
                                        'rgb_frame_id': 'camera_rgb_optical_frame',
                                        'depth_frame_id': 'camera_depth_optical_frame'}],
                           remappings=[('depth/image', 'depth_registered/image_raw')]),
            ComposableNode(package='depth_image_proc', plugin='depth_image_proc::PointCloudXyzrgbNode',
                           name='points_xyzrgb', namespace='camera', parameters=[{'queue_size': 10}],
                           remappings=[('rgb/image_rect_color', 'rgb/image_raw'),
                                       ('depth_registered/image_rect', 'depth_registered/image_raw'),
                                       ('points', 'depth_registered/points')]),
        ])
    test = Node(package='so100_driver', executable='camera_test', output='screen',
                parameters=[{'table_z': table_z}])
    return [container, test]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='real', choices=['real', 'sim']),
        DeclareLaunchArgument('mode', default_value='moveit', choices=['moveit', 'sliders']),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='ASUS Xtion camera test: finds objects, /camera_test/pick (MoveIt mode)'),
        DeclareLaunchArgument('calibration', default_value='~/so100_ws/calibration.json'),
        DeclareLaunchArgument('table_z', default_value='-0.075',
                              description='table top below the arm base (m): base sits on a ~7 cm platform, measured 2026-11-07'),
        OpaqueFunction(function=setup),
    ])
