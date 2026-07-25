#!/bin/bash
# 第三段階: 破綻シナリオ(検知+自己復帰の検証)。60秒、イベント窓 sim時刻25〜35s。
# 使い方: bash run_stage3.sh <tag> <pattern> [fault_yaml_snippet]
SP=/tmp/claude-1000/-home-yukichi6105-ros2-ws/495ade90-3950-44ea-bb25-9a8d145d8fd7/scratchpad
TAG="$1"; PATTERN="$2"; FAULT="$3"
OUT="$SP/runs/$TAG"; mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash >/dev/null 2>&1
source /home/yukichi6105/ros2_ws/install/setup.bash >/dev/null 2>&1
pkill -f "imu_sim_node|lidar_sim_node|evaluator_node|laser_scan_matching|backend_optimizer|ball_tracking|imu_preintegration" 2>/dev/null
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /tmp/sim_eval_report.txt /tmp/diag_raw_errors.csv /tmp/sim_start_time.txt 2>/dev/null
sleep 1

YAML="$SP/field_$TAG.yaml"
python3 "$SP/gen_field.py" "$PATTERN" "$YAML" > "$OUT/gen.log" 2>&1
read W H < "$YAML.dims"
# センサ異常スニペットをyamlへ追記(/**: の下。未宣言paramは各ノードが無害に無視)
if [ -n "$FAULT" ]; then printf '%s\n' "$FAULT" >> "$YAML"; fi
cat "$OUT/gen.log"

timeout 130 ros2 launch lio_localization_sim sim_test.launch.py \
  field_config:="$YAML" field_width:="$W" field_height:="$H" \
  trajectory_pattern:="$PATTERN" duration_sec:=63.0 > "$OUT/launch.log" 2>&1
RC=$?
cp /tmp/sim_eval_report.txt "$OUT/" 2>/dev/null
cp /tmp/diag_raw_errors.csv "$OUT/" 2>/dev/null
cp /tmp/sim_start_time.txt "$OUT/" 2>/dev/null
echo "=== $TAG finished (rc=$RC) ==="
