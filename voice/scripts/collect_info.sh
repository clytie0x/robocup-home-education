#!/usr/bin/env bash
# Stage 0: collect facts about this machine (Jetson or Pi) into one text report.
# Read-only: no sudo, installs nothing, changes nothing. Run it while the robot stack is up
# to also capture the live ROS graph. Output: ~/stage0_<hostname>.txt
OUT="$HOME/stage0_$(hostname).txt"
T() { timeout "${2:-8}" bash -c "$1" 2>&1 || echo "(failed / timed out / not available)"; }
S() { echo; echo "===== $1 ====="; }

{
S "machine";      date; hostname; uname -a
T "tr -d '\0' < /proc/device-tree/model; echo"
T "grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release"
T "cat /etc/nv_tegra_release"
T "nproc; python3 --version"

S "memory / disk"; free -m
T "tegrastats | head -n 1" 3
T "df -h / /opt 2>/dev/null"
T "ps -eo rss,comm,args --sort=-rss | head -n 15 | cut -c1-160"

S "network";      T "hostname -I"
T "ss -ltnH 2>/dev/null | awk '{print \$4}' | grep -E ':(8080|8081|8090)\$'"

S "ros install";  ls /opt/ros 2>/dev/null || echo "(no /opt/ros)"
for d in /opt/ros/*/setup.bash; do [ -f "$d" ] && source "$d"; done
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
echo "ROS_DISTRO=${ROS_DISTRO:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(unset)} RMW=${RMW_IMPLEMENTATION:-(default)}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-(unset)} ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-(unset)}"
T "ls $HOME/ros2_ws/src"
T "ros2 pkg executables voice_assistant" 15
T "ros2 pkg list | grep -E 'so100|voice|nav2_bringup|turtlesim'" 15

S "ros graph (live)"
T "ros2 node list" 15
T "ros2 topic list -t" 15
T "ros2 action list -t" 15
T "ros2 service list | grep -E '^/arm'" 15
T "ros2 topic echo --once /flow/state" 5
T "ros2 topic echo --once /robot/state" 5
T "ros2 topic info -v /robot/event | grep -E 'Node name|count'" 10
T "ros2 topic info -v /cmd_vel | grep -E 'Node name|count'" 10

S "/robot/event during 10 s (rate of see_person etc.)"
timeout 10 ros2 topic echo /robot/event --field data 2>/dev/null | grep -v '^---' | cut -d'|' -f1 | sort | uniq -c
echo "(empty above = nothing published in 10 s)"

S "repo / config"
T "git -C $HOME/robocup-home-education log -1 --format='%h %ad %s' --date=short; git -C $HOME/robocup-home-education status -sb | head -n 15"
T "ls -ld $HOME/robot_voice $HOME/ros2_ws/src/voice_assistant"
echo "--- ~/.robot_voice_local.yaml"; T "cat $HOME/.robot_voice_local.yaml"

S "voice servers / models"
T "pgrep -af 'whisper-server|llama-server|control_panel|voice_assistant.launch'"
T "curl -s -m 3 -o /dev/null -w 'whisper :8080 http %{http_code}\n' http://127.0.0.1:8080/"
T "curl -s -m 3 http://127.0.0.1:8081/health; echo"
T "ls -lh /opt/robot_voice/models /opt/robot_voice/models/* 2>/dev/null | head -n 30"
T "pip list 2>/dev/null | grep -iE '^(piper|webrtcvad|sounddevice|numpy|requests|onnxruntime) '" 20

S "audio devices"
T "python3 -m sounddevice" 10
T "arecord -l"
T "aplay -l"
} > "$OUT" 2>&1

echo "wrote $OUT ($(wc -l < "$OUT") lines)"
