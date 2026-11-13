# SLAM on the Jetson

The Pi streams every LiDAR revolution and its pose to `slam_bridge.py` (port 5602). The bridge
publishes `/scan`, `/odom` and `odom -> base_link` to ROS2, slam_toolbox adds `map -> odom`, and
the bridge sends that correction back to the Pi twice a second. Map PNGs go to
`~/hmc-follow/logs/map-<time>.png` every 10 s.

One-off, by the owner (needs sudo, tell the voice and arm teams):

    sudo apt install ros-jazzy-slam-toolbox

Run (two terminals on the Jetson, in `~/hmc-follow`). The arm team uses ROS_DOMAIN_ID 42 and the voice
team 43, so ours is 44 and loopback only: our `odom`, `base_link` and `/scan` never enter their graph.

    export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1; source /opt/ros/jazzy/setup.bash && python3 slam_bridge.py
    export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1; source /opt/ros/jazzy/setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/hmc-follow/slam/params.yaml use_sim_time:=false

Check with a replayed log (the robot does not move), from the Pi:

    ~/hmc-base/.venv/bin/python ~/hmc-base/tools/slam_replay.py ~/hmc-base/logs/<run folder>

Rates, on the Jetson (a third terminal, with the same two exports):

    ros2 topic hz /scan
    ros2 topic hz /odom
    ros2 run tf2_ros tf2_echo map odom

Stopping: Ctrl-C the launch. slam_toolbox 2.8.5 then aborts with "context cannot be slept with because it's
invalid" and apport writes a crash report. That is a shutdown bug in the package, not a fault in the map.
The first correction of a session can be far from zero: the map origin is the first pose slam_toolbox sees,
and the firmware odometry is not zeroed between runs. `SlamLink` takes the first one after each connection.
