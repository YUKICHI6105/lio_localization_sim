#!/bin/bash
# Stage-4 run driven by the interactive Editor via MCP (never batch mode: a second Unity
# instance OOMs this 15.7 GB machine and has already crashed the user's Editor once).
#
# This script owns everything except pressing Play. It brings the ROS stack up, arms the
# per-second topic-rate probes needed to localise an IMU dropout, and then writes a READY
# marker. The caller polls for that marker and enters play mode immediately, which keeps the
# ROS-up-to-Play gap short. The evaluator no longer cares about that gap (its window starts at
# the first /ground_truth_pose), but a short gap still keeps the logs readable.
#
# Usage: stage4_mcp_run.sh <tag> [--probes] [config_yaml]
set -u

TAG="${1:?usage: stage4_mcp_run.sh <tag> [--probes] [config_yaml]}"
PROBE_MODE="${2:-}"
WS=/home/yukichi6105/ros2_ws
OUT="$WS/src/lio_localization_sim/docs/stage4_runs/$TAG"

mkdir -p "$OUT"
export ROS_LOG_DIR="$OUT/ros2_logs"
mkdir -p "$ROS_LOG_DIR"
rm -f "$OUT"/*.log "$OUT"/*.txt "$OUT"/*.csv "$OUT/READY"
rm -f /tmp/sim_eval_report.txt /tmp/sim_eval_plot.png /tmp/diag_raw_errors.csv /tmp/diag_odom.csv

# Preserve the exact inputs used by every run. A tag alone was insufficient to reconstruct
# several 2026-07-28 experiments because the YAML and dirty source state were not archived.
CFG="${3:-$WS/src/lio_localization_sim/config/robocon2026_unity.yaml}"
cp "$CFG" "$OUT/robocon2026_unity.yaml"
snapshot_repo() {
  local name="$1"
  local repo="$2"
  git -C "$repo" rev-parse HEAD > "$OUT/${name}_commit.txt"
  git -C "$repo" status --short > "$OUT/${name}_status.txt"
  git -C "$repo" diff --binary HEAD > "$OUT/${name}_working_tree.patch"
  # git diff HEAD does not include newly-created (untracked) source files.
  # Archive them as standalone no-index patches so a run using an experimental
  # C++ node is reproducible even before that node has been committed.
  git -C "$repo" ls-files --others --exclude-standard > "$OUT/${name}_untracked_files_all.txt"
  : > "$OUT/${name}_untracked.patch"
  : > "$OUT/${name}_untracked_files.txt"
  while IFS= read -r file; do
    [ -n "$file" ] || continue
    # Run artifacts and agent scratch state are not source inputs; including
    # them makes the patch huge and can follow stale ros2_logs symlinks.
    case "$file" in
      docs/stage4_runs/*|.claude/*) continue ;;
    esac
    printf '%s\n' "$file" >> "$OUT/${name}_untracked_files.txt"
    git -C "$repo" diff --binary --no-index /dev/null "$file" >> "$OUT/${name}_untracked.patch" 2>/dev/null || true
  done < "$OUT/${name}_untracked_files_all.txt"
}
snapshot_repo lio_localization "$WS/src/lio_localization"
snapshot_repo lio_localization_sim "$WS/src/lio_localization_sim"
{
  printf 'run_tag=%s\n' "$TAG"
  printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'probe_mode=%s\n' "$PROBE_MODE"
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-not_sourced_yet}"
} > "$OUT/run_manifest.txt"

pkill -f "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint|fusion_stage_diag.py"
sleep 1

# The ROS setup scripts reference unset variables, so relax -u just for them.
set +u
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
set -u

ros2 launch lio_localization_sim unity_sensor_localization.launch.py \
  field_config:="$CFG" > "$OUT/ros.log" 2>&1 &
ROS_PID=$!

endpoint_open() {
  # Do not probe by opening and immediately closing a TCP connection.  The
  # ROS-TCP endpoint treats that as a malformed Unity client and emits a
  # misleading "No more data available" exception.  A listening-socket check
  # establishes readiness without injecting traffic into the experiment.
  ss -ltnH 'sport = :10000' | grep -q .
}

for _ in $(seq 1 60); do
  endpoint_open && break
  sleep 0.5
done
if ! endpoint_open; then
  echo "ERROR: ROS TCP endpoint never opened :10000 (see $OUT/ros.log)" >&2
  kill $ROS_PID 2>/dev/null; exit 3
fi

# Whether the IMU stops being *sent* or stops being *received* is the whole question, and a
# rate averaged over the run cannot answer it. Stamp each second so the dropout can be placed
# on the timeline against ros.log. /scan is the control: it shares the same TCP socket, so if
# /imu/data alone dies the socket is fine and the 250 Hz publisher is the suspect.
probe() {
  local topic="$1" file="$2"
  ros2 topic hz "$topic" --window 50 2>&1 \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%H:%M:%S)" "$line"; done > "$file"
}
if [ "$PROBE_MODE" = "--probes" ]; then
  probe /imu/data "$OUT/hz_imu.log" &
  probe /scan "$OUT/hz_scan.log" &
  ros2 topic echo /imu/data --field header.stamp 2>&1 \
    | while IFS= read -r line; do printf '%s %s\n' "$(date +%H:%M:%S.%3N)" "$line"; done > "$OUT/imu_stamps.log" &
  python3 "$WS/src/lio_localization_sim/tools/fusion_stage_diag.py" \
    --ros-args --params-file "$WS/src/lio_localization_sim/config/robocon2026_unity.yaml" \
    -p settle_sec:=0.0 > "$OUT/fusion_stage.log" 2>&1 &
fi

sleep 2
touch "$OUT/READY"
echo "READY — enter play mode now"

# The launch shuts itself down when the evaluator finishes; cap the wait so a stalled run
# still returns instead of hanging the session.
for _ in $(seq 1 400); do
  [ -f /tmp/sim_eval_report.txt ] && break
  sleep 1
done

# The evaluator writes its report before launch has finished shutting the C++ nodes down.
# Give their rclcpp shutdown hooks time to flush the in-memory 1 kHz diagnostic recorder;
# killing the launch two seconds after the report used to discard that evidence.
for _ in $(seq 1 20); do
  ! kill -0 "$ROS_PID" 2>/dev/null && break
  sleep 0.5
done

kill $ROS_PID 2>/dev/null
pkill -f "ros2 topic hz|ros2 topic echo"
pkill -f "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint|fusion_stage_diag.py"

# Rendering is deliberately post-run: importing matplotlib inside the former
# Python evaluator contended with the 1 kHz localization data path.
if [ -f /tmp/diag_raw_errors.csv ]; then
  python3 "$WS/src/lio_localization_sim/tools/render_eval_plot.py" \
    /tmp/diag_raw_errors.csv /tmp/sim_eval_plot.png \
    >> "$OUT/ros.log" 2>&1 || echo "WARNING: offline plot rendering failed" >> "$OUT/ros.log"
fi

cp /tmp/sim_eval_report.txt "$OUT/" 2>/dev/null
cp /tmp/sim_eval_plot.png "$OUT/" 2>/dev/null
cp /tmp/diag_raw_errors.csv "$OUT/" 2>/dev/null
cp /tmp/diag_odom.csv "$OUT/" 2>/dev/null

echo "=== [$TAG] done -> $OUT ==="
if [ -f "$OUT/sim_eval_report.txt" ]; then
  grep -E "位置誤差|ヨー誤差|判定|RMSE|max" "$OUT/sim_eval_report.txt" | head -10
else
  echo "NO EVALUATOR REPORT — check $OUT/ros.log"
fi
