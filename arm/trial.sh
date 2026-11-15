#!/bin/bash
# usage: trial.sh "<servo_p>" "<servo_d>" "<servo_dead_zone>"   (arrays of 6, IDs 1..6)
cd ~/so100_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 param set /so100_driver servo_p "$1" 2>/dev/null | grep -v RTPS
ros2 param set /so100_driver servo_d "$2" 2>/dev/null | grep -v RTPS
ros2 param set /so100_driver servo_dead_zone "$3" 2>/dev/null | grep -v RTPS
sleep 1
timeout 200 ros2 run so100_driver motion_test 0.3 heavy 2>&1 | grep -v -e "^\s*$" -e RTPS | sed -E "s/ \| torque=ON.*//; s/ +code=0//"
