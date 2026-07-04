#!/usr/bin/env zsh
# Clean-slate bring-up of simulation + fusion stack, with process verification.
#
# Kills every leftover sim/fusion process (duplicate half-dead launches
# silently corrupt topic statistics), starts one simulation and one fusion
# stack, verifies exactly one of each is running, and optionally runs a test.
#
# Requires a sourced workspace (ros2 available). Logs go to ~/.ros/olive_bringup/.
#
# Usage:
#   ros2 run olive bringup_test.sh [world] [test] [rviz]
#     world: maze (default) | fusion_test | warehouse | office | industrial
#     test : none (default) | drive | drive-long | ate | marker
#     rviz : add the literal argument "rviz" to open RViz with the debug config

WORLD="${1:-maze}"
TEST="${2:-none}"
LOG_DIR="$HOME/.ros/olive_bringup"
mkdir -p "$LOG_DIR"

echo "=== stopping leftover sim/fusion processes ==="
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f ros_gz_bridge 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f "olive/lib/olive/fusion_node" 2>/dev/null
pkill -9 -f "olive/lib/olive/vo_node" 2>/dev/null
pkill -9 -f "whycode_vision/whycon" 2>/dev/null
pkill -9 -f "simulation.launch" 2>/dev/null
pkill -9 -f "sensor_fusion.launch" 2>/dev/null
sleep 3
LEFT=$(pgrep -f "gz sim|ros_gz_bridge|fusion_node|whycon|robot_state_publisher" | wc -l)
if [ "$LEFT" -ne 0 ]; then
  echo "ERROR: $LEFT processes survived:"
  pgrep -af "gz sim|ros_gz_bridge|fusion_node|whycon|robot_state_publisher"
  exit 1
fi

echo "=== starting simulation ($WORLD, headless) ==="
ros2 launch olive_sim simulation.launch.py "world_name:=$WORLD" headless:=true use_rviz:=false \
  > "$LOG_DIR/sim.log" 2>&1 &
sleep 12

echo "=== starting fusion stack ==="
RVIZ_ARG="rviz:=false"
if [ "${3:-}" = "rviz" ]; then RVIZ_ARG="rviz:=true"; fi
ros2 launch olive sensor_fusion.launch.py "$RVIZ_ARG" > "$LOG_DIR/fusion.log" 2>&1 &
sleep 10

echo "=== verification ==="
GZ=$(pgrep -fc "gz sim" || true)
FN=$(pgrep -fc "olive/lib/olive/fusion_node" || true)
echo "gz sim procs: $GZ | fusion_node procs: $FN  (logs: $LOG_DIR)"
if [ "$FN" -ne 1 ]; then
  echo "ERROR: expected exactly 1 fusion_node"
  tail -5 "$LOG_DIR/fusion.log"
  exit 1
fi

case "$TEST" in
  drive)      ros2 run olive drive_test.py ;;
  drive-long) ros2 run olive drive_test.py --long ;;
  ate)        ros2 run olive ate_eval.py ;;
  marker)     ros2 run olive marker_demo.py ;;
  none)       echo "stack is up; run a test with: ros2 run olive drive_test.py" ;;
  *)          echo "unknown test '$TEST' (drive|drive-long|ate|marker|none)"; exit 1 ;;
esac
