#!/bin/bash
# Replay the reproducible Unity bag with only selected non-core components.
# Usage: bag_component_isolation.sh <tag> [--bag path] [--ball] [--endpoint]
#        [--no-record-stages] [--play-rate N] [--post-play-wait N]
#        [--scan-yaw-bias-rad N]
#        [--laser-param name:=value] [--backend-param name:=value] [--imu-param name:=value]
set -euo pipefail

TAG="${1:?usage: bag_component_isolation.sh <tag> [--ball] [--endpoint]}"
shift
WS=/home/yukichi6105/ros2_ws
BAG=/tmp/robocon_unity_88s_20260729.mcap
WITH_BALL=false
WITH_ENDPOINT=false
# 2026-07-31(ユーザー指示): ②(壁マッチング)と③(state_estimate)を真値と
# 突き合わせた誤差は毎回の検証に必要な標準データなので、既定で常時記録する。
# 必要な場合のみ--no-record-stagesで無効化できる。
WITH_STAGE_RECORD=true
PLAY_RATE=1.0
POST_PLAY_WAIT=10
PLAYBACK_DURATION=62
SCAN_YAW_BIAS_RAD=0.0
LASER_PARAMS=()
BACKEND_PARAMS=()
IMU_PARAMS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --bag)
      [ "$#" -ge 2 ] || { echo "--bag requires a rosbag2 path" >&2; exit 2; }
      BAG="$2"
      shift 2
      ;;
    --ball) WITH_BALL=true; shift ;;
    --endpoint) WITH_ENDPOINT=true; shift ;;
    --record-stages) WITH_STAGE_RECORD=true; shift ;;  # 既定で有効なので明示指定は不要(後方互換で残す)
    --no-record-stages) WITH_STAGE_RECORD=false; shift ;;
    --play-rate)
      [ "$#" -ge 2 ] || { echo "--play-rate requires a numeric rate" >&2; exit 2; }
      PLAY_RATE="$2"
      shift 2
      ;;
    --playback-duration)
      [ "$#" -ge 2 ] || { echo "--playback-duration requires seconds" >&2; exit 2; }
      PLAYBACK_DURATION="$2"
      shift 2
      ;;
    --post-play-wait)
      [ "$#" -ge 2 ] || { echo "--post-play-wait requires seconds" >&2; exit 2; }
      POST_PLAY_WAIT="$2"
      shift 2
      ;;
    --scan-yaw-bias-rad)
      [ "$#" -ge 2 ] || { echo "--scan-yaw-bias-rad requires radians" >&2; exit 2; }
      SCAN_YAW_BIAS_RAD="$2"
      shift 2
      ;;
    --laser-param)
      [ "$#" -ge 2 ] || { echo "--laser-param requires name:=value" >&2; exit 2; }
      # ros2 run requires -p before each independent name:=value override.
      LASER_PARAMS+=("-p" "$2")
      shift 2
      ;;
    --backend-param)
      [ "$#" -ge 2 ] || { echo "--backend-param requires name:=value" >&2; exit 2; }
      BACKEND_PARAMS+=("-p" "$2")
      shift 2
      ;;
    --imu-param)
      [ "$#" -ge 2 ] || { echo "--imu-param requires name:=value" >&2; exit 2; }
      IMU_PARAMS+=("-p" "$2")
      shift 2
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

OUT="$WS/src/lio_localization_sim/docs/stage4_runs/$TAG"
CFG="$WS/src/lio_localization_sim/config/robocon2026_unity.yaml"
EVAL_OUT="$OUT/evaluator_output"

test -d "$BAG" || { echo "bag not found: $BAG" >&2; exit 2; }
mkdir -p "$OUT" "$EVAL_OUT"
cp "$CFG" "$OUT/robocon2026_unity.yaml"

snapshot_repo() {
  local name="$1"
  local repo="$2"
  git -C "$repo" rev-parse HEAD > "$OUT/${name}_commit.txt"
  git -C "$repo" status --short > "$OUT/${name}_status.txt"
  git -C "$repo" diff --binary HEAD > "$OUT/${name}_working_tree.patch"
  git -C "$repo" ls-files --others --exclude-standard > "$OUT/${name}_untracked_files_all.txt"
  : > "$OUT/${name}_untracked.patch"
  : > "$OUT/${name}_untracked_files.txt"
  while IFS= read -r file; do
    [ -n "$file" ] || continue
    case "$file" in
      docs/stage4_runs/*|.claude/*) continue ;;
    esac
    printf '%s\n' "$file" >> "$OUT/${name}_untracked_files.txt"
    git -C "$repo" diff --binary --no-index /dev/null "$file" >> "$OUT/${name}_untracked.patch" 2>/dev/null || true
  done < "$OUT/${name}_untracked_files_all.txt"
}

snapshot_repo lio_localization "$WS/src/lio_localization"
snapshot_repo lio_localization_sim "$WS/src/lio_localization_sim"
cat > "$OUT/run_manifest.txt" <<EOF
run_tag=$TAG
mode=bag_component_isolation
with_ball=$WITH_BALL
with_endpoint=$WITH_ENDPOINT
with_stage_record=$WITH_STAGE_RECORD
play_rate=$PLAY_RATE
post_play_wait=$POST_PLAY_WAIT
scan_yaw_bias_rad=$SCAN_YAW_BIAS_RAD
laser_params=${LASER_PARAMS[*]}
backend_params=${BACKEND_PARAMS[*]}
bag=$BAG
started_at=$(date --iso-8601=seconds)
EOF

set +u
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_LOG_DIR="$OUT/ros2_logs"
mkdir -p "$ROS_LOG_DIR"

PIDS=()
start_node() {
  local log="$1"
  shift
  "$@" > "$OUT/$log" 2>&1 &
  PIDS+=("$!")
}
# backend_optimizer_nodeはパーティクル再収束(relocalization_event)受信時、
# GTSAM/TBB内部のクラッシュを避けるため終了コード42でプロセスごと自己終了する
# (docs/experiment_history/22番以降参照)。実運用ではros2 launchのrespawn=Trueが
# これを拾うが、このスクリプトはros2 runで直接起動するため、同じ役割を果たす
# 簡易respawnラッパーをここで用意する。42以外の終了(=本物のクラッシュ)では
# 通常通り終了させ、テストの異常終了を隠さない。
start_node_respawn() {
  local log="$1"
  shift
  (
    set +e
    child_pid=""
    forward_signal() {
      [ -n "$child_pid" ] && kill "-$1" "$child_pid" 2>/dev/null || true
    }
    trap 'forward_signal INT' INT
    trap 'forward_signal TERM' TERM
    while true; do
      "$@" &
      child_pid=$!
      wait "$child_pid"
      rc=$?
      if [ "$rc" -ne 42 ]; then
        exit "$rc"
      fi
      echo "[respawn] backend exited 42 (particle relocalization handoff): restarting immediately"
    done
  ) > "$OUT/$log" 2>&1 &
  PIDS+=("$!")
}
cleanup() {
  [ "${#PIDS[@]}" -gt 0 ] || return
  kill -INT "${PIDS[@]}" 2>/dev/null || true
  # ros2 bag record must receive SIGINT to write MCAP metadata, but a transport
  # shutdown can occasionally leave it alive after all producers have stopped.
  # Do not let that recorder stall a completed benchmark forever: give every
  # process a bounded graceful period, then terminate only the remaining PIDs.
  for _ in $(seq 1 20); do
    local alive=false
    for pid in "${PIDS[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=true
    done
    "$alive" || break
    sleep 0.25
  done
  for pid in "${PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
  done
  for _ in $(seq 1 8); do
    local alive=false
    for pid in "${PIDS[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=true
    done
    "$alive" || break
    sleep 0.25
  done
  for pid in "${PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  done
  wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT

start_node_respawn backend.log ros2 run lio_localization backend_optimizer_node --ros-args \
  --params-file "$OUT/robocon2026_unity.yaml" -p use_sim_time:=true \
  -p initial_x:=-2.419 -p initial_y:=1.354 -p initial_theta:=0.0 \
  -p initial_vx:=0.0 -p initial_vy:=0.0 \
  -p relocalization_handoff_path:="$OUT/relocalization_handoff.txt" "${BACKEND_PARAMS[@]}"
start_node imu_preintegration.log ros2 run lio_localization imu_preintegration_node --ros-args \
  --params-file "$OUT/robocon2026_unity.yaml" -p use_sim_time:=true "${IMU_PARAMS[@]}"
start_node scan_matching.log ros2 run lio_localization laser_scan_matching_node --ros-args \
  --params-file "$OUT/robocon2026_unity.yaml" -p use_sim_time:=true \
  -p initial_x:=-2.419 -p initial_y:=1.354 -p initial_theta:=0.0 \
  "${LASER_PARAMS[@]}"
start_node evaluator.log nice -n 10 ros2 run lio_localization realtime_evaluator_node --ros-args \
  --params-file "$OUT/robocon2026_unity.yaml" -p use_sim_time:=true \
  -p duration_sec:=60.0 -p settle_sec:=3.0 -p sim_start_time_sec:=0.0 \
  -p output_dir:="$EVAL_OUT"

BAG_REMAP_ARGS=()
if [ "$SCAN_YAW_BIAS_RAD" != "0.0" ]; then
  # Keep the stored Unity bag immutable.  The relay applies exactly one declared
  # perturbation and leaves IMU, ground truth and playback timing untouched.
  start_node scan_noise_injector.log python3 "$WS/src/lio_localization_sim/tools/sensor_noise_injector.py" --ros-args \
    -p use_sim_time:=true -p scan_yaw_bias_rad:="$SCAN_YAW_BIAS_RAD"
  BAG_REMAP_ARGS=(--remap /scan:=/scan_raw)
fi

if "$WITH_STAGE_RECORD"; then
  start_node stage_record.log ros2 bag record --storage mcap -o "$OUT/stage_derived" --topics \
    /ground_truth_pose /scan_match_result /state_estimate /odom_fast
fi

if "$WITH_BALL"; then
  start_node ball_tracking.log nice -n 10 ros2 run lio_localization ball_tracking_node --ros-args \
    --params-file "$OUT/robocon2026_unity.yaml" -p use_sim_time:=true
fi
if "$WITH_ENDPOINT"; then
  start_node endpoint.log ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
    -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
fi

# Give ROS 2 discovery a deterministic margin before the bag's first (non-latched)
# scan.  A shorter, scheduler-dependent delay can leave the scan matcher with only
# its initial scan while IMU and ground-truth already stream, which is not a valid
# localization trial.
sleep 6
# Only replay raw sensor/truth topics. Some bags (e.g. a live --topics capture used to
# check ground truth against the estimator's own live output) also contain the
# estimator's OUTPUT topics (/odom_fast, /scan_match_result, /state_estimate). Replaying
# those too would collide with the freshly-launched nodes started above, which publish
# the same topic names from scratch -- the evaluator then sees an interleaved mix of the
# bag's old recorded values and this run's newly computed ones and its ground-truth
# matching breaks silently (observed 2026-07-31: "No /odom_fast samples were matched").
ros2 bag play "$BAG" --clock 250 --rate "$PLAY_RATE" --playback-duration "$PLAYBACK_DURATION" --disable-keyboard-controls \
  "${BAG_REMAP_ARGS[@]}" --topics /clock /imu/data /scan /ground_truth_pose /unity_sensor_publish_counts \
  --progress-bar-update-rate 0 > "$OUT/bag_play.log" 2>&1

# Allow the C++ evaluator to flush its bounded CSV before signalling the
# remaining nodes, whose orderly shutdown emits the backend diagnostics.
for _ in $(seq 1 "$POST_PLAY_WAIT"); do
  [ -f "$EVAL_OUT/sim_eval_report.txt" ] && break
  sleep 1
done
cleanup
trap - EXIT

cp "$EVAL_OUT/sim_eval_report.txt" "$OUT/" 2>/dev/null || true
cp "$EVAL_OUT/diag_raw_errors.csv" "$OUT/" 2>/dev/null || true
if [ -f "$OUT/diag_raw_errors.csv" ]; then
  python3 "$WS/src/lio_localization_sim/tools/render_eval_plot.py" \
    "$OUT/diag_raw_errors.csv" "$OUT/sim_eval_plot.png"
fi
# ②/③(と①)を真値と突き合わせた標準比較データを毎回自動生成する
# (2026-07-31、ユーザー指示: 常時記録・評価できるようにする)。
if "$WITH_STAGE_RECORD" && [ -d "$OUT/stage_derived" ]; then
  mkdir -p "$OUT/consolidated"
  python3 "$WS/src/lio_localization_sim/tools/consolidate_stage_log.py" \
    "$OUT/stage_derived" "$OUT/consolidated" > "$OUT/consolidate.log" 2>&1 || \
    echo "WARNING: consolidate_stage_log.py failed, see $OUT/consolidate.log" >&2
fi
echo "DONE: $OUT"
