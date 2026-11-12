#!/bin/bash
# Start the SO-100 stack on the virtual screen. Args are passed to the launch file, e.g.:
#   ~/so100_ws/start_arm.sh hardware:=sim       (no arm)
#   ~/so100_ws/start_arm.sh                     (real arm + MoveIt)
#   ~/so100_ws/start_arm.sh mode:=sliders       (real arm follows joint sliders)
"$HOME/so100_ws/start_display.sh"
source /opt/ros/jazzy/setup.bash
source "$HOME/so100_ws/install/setup.bash"
export DISPLAY=:1 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
"$HOME/so100_ws/stop_arm.sh" >/dev/null
setsid ros2 launch so100_moveit_config arm.launch.py "$@" >/tmp/so100_arm.log 2>&1 &
echo "started; log: /tmp/so100_arm.log   stop: ~/so100_ws/stop_arm.sh"
