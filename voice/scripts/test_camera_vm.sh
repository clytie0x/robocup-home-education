#!/usr/bin/env bash
# One-command test: does the voice system send the CAMERA position (/object/pose) to the arm?
# Runs everything on this machine with a FAKE arm and a FAKE camera, no mic / speaker / hardware needed.
#
#   ~/robot_voice/scripts/test_camera_vm.sh              # fake camera at x=0.05 y=-0.2452 (20 cm forward, 5 cm left)
#   ~/robot_voice/scripts/test_camera_vm.sh 0.10 -0.30   # other position (frame "base", metres)
#
# Uses ROS_DOMAIN_ID=43 by default so it never talks to a real arm on domain 42.
# Needs voice_assistant + so100_interfaces built in ~/ros2_ws.
X=${1:-0.05}
Y=${2:--0.2452}
export ROS_DOMAIN_ID=${TEST_DOMAIN_ID:-43}
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
D=/tmp/robot_voice_camtest
rm -rf "$D" && mkdir -p "$D"

source /opt/ros/jazzy/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash

if ! ros2 interface show so100_interfaces/action/Pick >/dev/null 2>&1; then
  echo "FAIL: so100_interfaces is not built. Copy it into ~/ros2_ws/src and run:"
  echo "  cd ~/ros2_ws && colcon build --packages-select so100_interfaces voice_assistant"
  exit 1
fi

PIDS=()
start() { "$@" >"$D/$4.log" 2>&1 & PIDS+=($!); }      # log named after the node
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 1
  pkill -f "voice_assistant/(fake_arm_server|arm_action_bridge|flow_node|rule_node)" 2>/dev/null
  for p in "${PIDS[@]}"; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

echo "== ROS_DOMAIN_ID=$ROS_DOMAIN_ID, fake camera at x=$X y=$Y (frame base) =="
start ros2 run voice_assistant fake_arm_server
start ros2 run voice_assistant arm_action_bridge
start ros2 run voice_assistant rule_node
start ros2 run voice_assistant flow_node
ros2 topic echo /tts/say std_msgs/msg/String --field data >"$D/robot_says.log" 2>&1 & PIDS+=($!)   # stands in for the speaker

echo -n "waiting for the fake arm"
for i in $(seq 1 30); do
  ros2 action list 2>/dev/null | grep -q '^/arm/pick$' && break
  echo -n "."; sleep 1
done
echo
ros2 topic pub --rate 2 /object/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base}, pose: {position: {x: $X, y: $Y}}}" >"$D/camera.log" 2>&1 & PIDS+=($!)
sleep 6                      # flow asks "How can I help you?" first

echo '== saying: "Help me carry this bag" =='
ros2 topic pub --once /speech/heard std_msgs/msg/String "{data: 'Help me carry this bag'}" >/dev/null 2>&1

for i in $(seq 1 20); do
  grep -q -E "grab_success|grab_failed" "$D/arm_action_bridge.log" 2>/dev/null && break
  sleep 1
done
sleep 1

BR="$D/arm_action_bridge.log"
ARM="$D/fake_arm_server.log"
echo
echo "---- arm_action_bridge ----"; grep -E "/arm/pick at|event ->" "$BR" | sed 's/^.*\] //'
echo "---- fake arm ----";          grep -E "PICK goal|SUCCESS|FAILED|forced" "$ARM" | sed 's/^.*\] //'
echo "---- robot says ----";        grep -v '^---' "$D/robot_says.log"
echo

WANT=$(printf "x=%.3f y=%.3f" "$X" "$Y")
if grep -q "(camera" "$BR" && grep -q "PICK goal: $WANT" "$ARM" && grep -q "grab_success" "$BR"; then
  echo "PASS: the arm got the camera position ($WANT) and the pick succeeded."
elif grep -q "fixed test point" "$BR"; then
  echo "FAIL: used the fixed test point, not the camera. Is /object/pose arriving? (logs: $D)"
elif ! grep -q "/arm/pick at" "$BR"; then
  echo "FAIL: no pick was sent. Check $D/flow_node.log and $D/rule_node.log"
else
  echo "FAIL: see the logs in $D"
fi
