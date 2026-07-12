#!/usr/bin/env zsh
# Fixed full-stack CPU benchmark: replay the maze_1loop sensor bag through the
# live fusion stack (fusion_node + whycon + vo_node, debug off) and report
# per-process CPU-seconds, peak RSS, and the ATE/continuity guardrails.
#
# The input bag (~/olive_ws/bags/maze_1loop) is NEVER re-recorded — it is the
# fixed input for every run. A small temp bag of the stack's live OUTPUT
# topics is captured each run (overwritten) so analyze_bag.py can compute
# fused-vs-ground-truth error offline.
#
# CPU accounting reads /proc/<pid>/stat (utime+stime), which is independent
# of the hybrid P/E-core perf-event split. The stack is pinned to P-cores and
# the replay/record infra to E-cores for run-to-run stability.
#
# Usage:  benchmark/full_stack_replay.sh [label] [--perf]
#   label   tag for the output dir + summary (default: timestampless "run")
#   --perf  additionally attach `perf record` to the stack during the replay
set -o pipefail
source /opt/ros/jazzy/setup.zsh
source /home/adyansh/olive_ws/install/setup.zsh
# Source-built optimized deps (BUILDING.md): prepend so they win over apt
# copies when present; harmless no-ops otherwise.
for d in /home/adyansh/olive_ws/deps/gtsam-install/lib \
         /home/adyansh/olive_ws/deps/pcl-install/lib; do
  [[ -d "$d" ]] && export LD_LIBRARY_PATH=$d:$LD_LIBRARY_PATH
done

LABEL=${1:-run}
DO_PERF=0
[[ "${2:-}" == "--perf" || "${1:-}" == "--perf" ]] && DO_PERF=1
[[ "${1:-}" == "--perf" ]] && LABEL=run

BAG=/home/adyansh/olive_ws/bags/maze_1loop
RUNDIR=/home/adyansh/olive_ws/perf_data/bench_runs/$LABEL
OUTBAG=$RUNDIR/outputs
mkdir -p "$RUNDIR"
rm -rf "$OUTBAG"
rm -f "$RUNDIR"/*.log(N) "$RUNDIR"/*.txt(N)

PCORES=$(cat /sys/devices/cpu_core/cpus 2>/dev/null)   # e.g. 0-15
ECORES=$(cat /sys/devices/cpu_atom/cpus 2>/dev/null)   # e.g. 16-31
[[ -z "$PCORES" ]] && PCORES=0-$(($(nproc)-1)) && ECORES=$PCORES
CLK=$(getconf CLK_TCK)

pid_of() { pgrep -f "$1" | head -1 }
cpu_ticks() { awk '{print $14+$15}' /proc/$1/stat 2>/dev/null || echo 0 }
peak_rss_mb() { awk '/VmHWM/{printf "%.0f", $2/1024}' /proc/$1/status 2>/dev/null || echo 0 }

echo "=== [1/7] clean slate ==="
for pat in "olive/lib/olive/fusion_node" "olive/lib/olive/vo_node" \
           "whycode_vision/whycon" "sensor_fusion.launch" \
           "bag play" "bag record" "gz sim" ros_gz_bridge; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [[ "$p" == "$$" ]] && continue
    kill -9 "$p" 2>/dev/null
  done
done
sleep 3

echo "=== [2/7] launch stack (no sim; bag is the only input) ==="
ros2 launch olive sensor_fusion.launch.py rviz:=false > "$RUNDIR/fusion.log" 2>&1 &
FPID=""; VPID=""; WPID=""
for i in {1..30}; do
  sleep 1
  FPID=$(pid_of "olive/lib/olive/fusion_node")
  VPID=$(pid_of "olive/lib/olive/vo_node")
  WPID=$(pid_of "whycode_vision/whycon")
  [[ -n "$FPID" && -n "$VPID" && -n "$WPID" ]] && break
done
if [[ -z "$FPID" || -z "$VPID" || -z "$WPID" ]]; then
  echo "ERROR: stack did not come up (fusion=$FPID vo=$VPID whycon=$WPID)"
  tail -20 "$RUNDIR/fusion.log"; exit 1
fi
sleep 3
echo "  pids: fusion=$FPID vo=$VPID whycon=$WPID"

echo "=== [3/7] debug off + pin stack to P-cores ($PCORES) ==="
for p in publish_debug debug_path debug_keyframes debug_local_map \
         debug_scan_features debug_fiducials debug_imu_state; do
  ros2 param set /fusion_node $p false >/dev/null 2>&1
done
ros2 param set /vo_node debug false >/dev/null 2>&1
for p in $FPID $VPID $WPID; do taskset -acp "$PCORES" "$p" >/dev/null 2>&1; done
echo "  governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"

echo "=== [4/7] start output recorder (E-cores) ==="
taskset -c "$ECORES" ros2 bag record -o "$OUTBAG" \
  /olive/odometry /olive/odometry_local /olive/visual_odom /ground_truth /tf \
  > "$RUNDIR/record.log" 2>&1 &
REC=$!
sleep 4

PERF_PID=""
if [[ $DO_PERF -eq 1 ]]; then
  perf record -g --call-graph dwarf -F 497 -p "$FPID,$VPID,$WPID" \
    -o "$RUNDIR/perf.data" > "$RUNDIR/perf.log" 2>&1 &
  PERF_PID=$!
  echo "  perf attached ($PERF_PID)"
fi

echo "=== [5/7] replay $BAG (E-cores) ==="
F0=$(cpu_ticks $FPID); V0=$(cpu_ticks $VPID); W0=$(cpu_ticks $WPID)
T0=$(date +%s.%N)
taskset -c "$ECORES" ros2 bag play "$BAG" --clock > "$RUNDIR/play.log" 2>&1
RC=$?
T1=$(date +%s.%N)
F1=$(cpu_ticks $FPID); V1=$(cpu_ticks $VPID); W1=$(cpu_ticks $WPID)
FR=$(peak_rss_mb $FPID); VR=$(peak_rss_mb $VPID); WR=$(peak_rss_mb $WPID)
echo "  play rc=$RC"
sleep 2

echo "=== [6/7] stop recorder (clean SIGINT + wait for metadata) ==="
[[ -n "$PERF_PID" ]] && { kill -INT $PERF_PID 2>/dev/null; for i in {1..30}; do kill -0 $PERF_PID 2>/dev/null || break; sleep 1; done }
kill -INT $REC 2>/dev/null
for i in {1..20}; do kill -0 $REC 2>/dev/null || break; sleep 1; done
kill -TERM $REC 2>/dev/null
for i in {1..10}; do kill -0 $REC 2>/dev/null || break; sleep 1; done
kill -9 $REC 2>/dev/null
if [[ ! -f "$OUTBAG/metadata.yaml" ]]; then
  echo "  recorder did not finalize — reindexing"
  ros2 bag reindex "$OUTBAG" -s mcap >> "$RUNDIR/record.log" 2>&1
fi
for pat in "olive/lib/olive/fusion_node" "olive/lib/olive/vo_node" \
           "whycode_vision/whycon" "sensor_fusion.launch"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
done

echo "=== [7/7] guardrails (analyze_bag on the output capture) ==="
python3 "$(dirname "$0")/../scripts/evaluation/analyze_bag.py" "$OUTBAG" \
  > "$RUNDIR/analyze.txt" 2>&1
cat "$RUNDIR/analyze.txt"

WALL=$(printf "%.1f" $((T1 - T0)))
FC=$(printf "%.1f" $(( (F1 - F0) * 1.0 / CLK )))
VC=$(printf "%.1f" $(( (V1 - V0) * 1.0 / CLK )))
WC=$(printf "%.1f" $(( (W1 - W0) * 1.0 / CLK )))
TOT=$(printf "%.1f" $(( (F1 - F0 + V1 - V0 + W1 - W0) * 1.0 / CLK )))
ATE=$(grep -oE "RMSE=[0-9.]+" "$RUNDIR/analyze.txt" | head -1 | cut -d= -f2)
STEP=$(grep -oE "max inter-sample step: [0-9.]+" "$RUNDIR/analyze.txt" | grep -oE "[0-9.]+")

echo ""
echo "================ BENCH SUMMARY [$LABEL] ================"
echo "wall: ${WALL}s   (bag 202.7s)"
printf "%-12s %10s %10s %10s\n" "" fusion vo_node whycon
printf "%-12s %9ss %9ss %9ss   total %ss\n" "CPU-seconds" "$FC" "$VC" "$WC" "$TOT"
printf "%-12s %9s%% %9s%% %9s%%\n" "avg %CPU" \
  "$(printf %.0f $((FC*100/WALL)))" "$(printf %.0f $((VC*100/WALL)))" "$(printf %.0f $((WC*100/WALL)))"
printf "%-12s %9sM %9sM %9sM\n" "peak RSS" "$FR" "$VR" "$WR"
echo "ATE RMSE: ${ATE:-?} m    local max step: ${STEP:-?} m"
echo "========================================================"
