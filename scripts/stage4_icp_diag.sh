#!/bin/bash
# Decide whether the residual error is scan-side or fusion-side.
#
# icp_from_truth_diag starts point-to-line ICP AT the ground-truth pose using nothing but
# /scan and the wall map -- no IMU, no factor graph. If it walks away by roughly the observed
# localisation error, the scan itself supports the wrong pose and the estimator is faithfully
# reporting what it is given. If it stays near zero, the scan is fine and the error is
# introduced downstream in fusion.
#
# Only the TCP endpoint runs, so the robot's controller falls back to ground truth and drives
# the true path -- exactly what this measurement wants.
#
# Usage: stage4_icp_diag.sh <tag>
set -u

TAG="${1:?usage: stage4_icp_diag.sh <tag>}"
WS=/home/yukichi6105/ros2_ws
OUT="$WS/src/lio_localization_sim/docs/stage4_runs/$TAG"
CFG="$WS/src/lio_localization_sim/config/robocon2026_unity.yaml"

mkdir -p "$OUT"
rm -f "$OUT"/*.log "$OUT/READY"

pkill -f "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint|icp_from_truth"
sleep 1

set +u
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
set -u

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 -p use_sim_time:=true > "$OUT/endpoint.log" 2>&1 &
EP_PID=$!

for _ in $(seq 1 60); do
  ss -ltn 2>/dev/null | grep -q ':10000' && break
  sleep 0.5
done
if ! ss -ltn 2>/dev/null | grep -q ':10000'; then
  echo "ERROR: endpoint never opened :10000 (see $OUT/endpoint.log)" >&2
  kill $EP_PID 2>/dev/null; exit 3
fi

python3 "$WS/src/lio_localization_sim/tools/icp_from_truth_diag.py" --ros-args \
  --params-file "$CFG" -p report_every:=20 > "$OUT/icp_from_truth.log" 2>&1 &
DIAG_PID=$!

python3 "$WS/src/lio_localization_sim/tools/wall_bias_diag.py" --ros-args \
  --params-file "$CFG" -p report_every:=40 > "$OUT/wall_bias.log" 2>&1 &
BIAS_PID=$!

sleep 2
touch "$OUT/READY"
echo "READY — enter play mode now"

sleep 60

kill $DIAG_PID $BIAS_PID $EP_PID 2>/dev/null
pkill -f "icp_from_truth|wall_bias_diag|default_server_endpoint"

echo "=== [$TAG] ICP-from-truth (should be ~0 if the scan supports the true pose) ==="
grep "ICP moved" "$OUT/icp_from_truth.log" | tail -12
