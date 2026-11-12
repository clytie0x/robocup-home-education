#!/bin/bash
# Stop every SO-100 ROS process (the real arm keeps holding its last pose with torque ON).
pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch so100_moveit_config"
sleep 2
pkill -x rviz2; pkill -x move_group; pkill -x robot_state_pub
pkill -f "^/usr/bin/python3 .*/lib/so100_driver/(driver|control_panel|scene_setup|pick_place)"
pkill -f "^/usr/bin/python3 .*/lib/joint_state_publisher_gui/"
sleep 1
pkill -9 -f "^/usr/bin/python3 .*/lib/so100_driver/(control_panel|pick_place)"
echo stopped
