# SLAM on the Jetson

The Pi streams every LiDAR revolution and its pose to `slam_bridge.py` (port 5602). The bridge
publishes `/scan`, `/odom` and `odom -> base_link` to ROS2, slam_toolbox adds `map -> odom`, and
the bridge sends that correction back to the Pi twice a second. Map PNGs go to
`~/hmc-follow/logs/map-<time>.png` every 10 s.

One-off, by the owner (needs sudo, tell the voice and arm teams):

    sudo apt install ros-jazzy-slam-toolbox

Run (two terminals on the Jetson, in `~/hmc-follow`):

    source /opt/ros/jazzy/setup.bash && python3 slam_bridge.py
    source /opt/ros/jazzy/setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/hmc-follow/slam/params.yaml use_sim_time:=false

Check with a replayed log (the robot does not move), from the Pi:

    ~/hmc-base/.venv/bin/python ~/hmc-base/tools/slam_replay.py ~/hmc-base/logs/<run folder>

Rates, on the Jetson:

    ros2 topic hz /scan
    ros2 topic hz /odom
    ros2 run tf2_ros tf2_echo map odom
