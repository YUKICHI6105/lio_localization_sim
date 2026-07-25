#!/bin/bash
# Run the full localisation stack plus fusion_stage_diag, which compares the matcher output,
# the fused state and the propagated odometry against ground truth at each stage's own stamp.
# Usage: stage4_fusion_diag.sh <tag>
set -u

TAG="${1:?usage: stage4_fusion_diag.sh <tag>}"
WS=/home/yukichi6105/ros2_ws
OUT="$WS/src/lio_localization_sim/docs/stage4_runs/$TAG"
CFG="$WS/src/lio_localization_sim/config/robocon2026_unity.yaml"

mkdir -p "$OUT"
rm -f "$OUT"/*.log "$OUT/READY" /tmp/sim_eval_report.txt

pkill -f "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint|fusion_stage_diag"
sleep 1

set +u
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
set -u

ros2 launch lio_localization_sim unity_sensor_localization.launch.py > "$OUT/ros.log" 2>&1 &
ROS_PID=$!

for _ in $(seq 1 60); do
  ss -ltn 2>/dev/null | grep -q ':10000' && break
  sleep 0.5
done
if ! ss -ltn 2>/dev/null | grep -q ':10000'; then
  echo "ERROR: endpoint never opened :10000 (see $OUT/ros.log)" >&2
  kill $ROS_PID 2>/dev/null; exit 3
fi

python3 "$WS/src/lio_localization_sim/tools/fusion_stage_diag.py" --ros-args \
  --params-file "$CFG" > "$OUT/fusion_stage.log" 2>&1 &
DIAG_PID=$!

sleep 2
touch "$OUT/READY"
echo "READY — enter play mode now"

for _ in $(seq 1 400); do
  [ -f /tmp/sim_eval_report.txt ] && break
  sleep 1
done
sleep 3

kill $DIAG_PID $ROS_PID 2>/dev/null
pkill -f "fusion_stage_diag|imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint"

cp /tmp/sim_eval_report.txt "$OUT/" 2>/dev/null

echo "=== [$TAG] final fusion_stage_diag report ==="
awk '/^.*={10,}/{buf=""} {buf=buf"\n"$0} END{print buf}' "$OUT/fusion_stage.log" | tail -30
