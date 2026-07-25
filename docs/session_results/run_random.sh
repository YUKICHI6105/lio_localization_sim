#!/bin/bash
# 第二段階耐久試験: ランダム経路(seed)を60秒×指定回数実行する。
# 使い方: bash run_random.sh <seed> <duration>
SP=/tmp/claude-1000/-home-yukichi6105-ros2-ws/495ade90-3950-44ea-bb25-9a8d145d8fd7/scratchpad
SEED="$1"
DUR="${2:-60}"
OUT="$SP/runs/rand_$SEED"
mkdir -p "$OUT"

source /opt/ros/lyrical/setup.bash >/dev/null 2>&1
source /home/yukichi6105/ros2_ws/install/setup.bash >/dev/null 2>&1

pkill -f "imu_sim_node|lidar_sim_node|evaluator_node|laser_scan_matching|backend_optimizer|ball_tracking|imu_preintegration" 2>/dev/null
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
rm -f /tmp/sim_eval_report.txt /tmp/sim_eval_plot.png /tmp/diag_raw_errors.csv /tmp/sim_start_time.txt 2>/dev/null
sleep 1

# per-seedフィールド+円柱yamlを生成し、寸法を取得
YAML="$SP/field_rand_$SEED.yaml"
python3 "$SP/gen_field.py" "$SEED" "$YAML" > "$OUT/gen.log" 2>&1
read W H < "$YAML.dims"
cat "$OUT/gen.log"

DUR_HOLD=$(python3 -c "print($DUR + 3.0)")  # 起動ホールド3s分を足す
timeout $(python3 -c "print(int($DUR_HOLD)+60)") ros2 launch lio_localization_sim sim_test.launch.py \
  field_config:="$YAML" field_width:="$W" field_height:="$H" \
  trajectory_pattern:="$SEED" duration_sec:="$DUR_HOLD" > "$OUT/launch.log" 2>&1
RC=$?

cp /tmp/sim_eval_report.txt "$OUT/" 2>/dev/null
cp /tmp/sim_eval_plot.png "$OUT/" 2>/dev/null
cp /tmp/diag_raw_errors.csv "$OUT/" 2>/dev/null
cp /tmp/sim_start_time.txt "$OUT/" 2>/dev/null
echo "=== rand_$SEED finished (rc=$RC) ==="
