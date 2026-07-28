#!/bin/bash
# One repeatable measurement run for the point-of-no-return sweep.
#
# Everything except pressing Play is automated here; the caller drives Play/Stop over the
# Unity MCP connection between the READY marker and completion. Each invocation writes its
# own directory AND appends one fully-parsed row to a shared CSV, so the numbers survive
# independently of whoever ran it -- a summary passed back through an agent can lose detail,
# a CSV on disk cannot.
#
# Usage: stage4_sweep_run.sh <on|off> <hold_sec> <wall_sigma> <rep>
set -u

MODE="${1:?usage: stage4_sweep_run.sh on|off <hold_sec> <wall_sigma> <rep>}"
HOLD="${2:?}"
SIG="${3:?}"
REP="${4:?}"

WS=/home/yukichi6105/ros2_ws
BASE=$WS/src/lio_localization_sim/config/robocon2026_unity.yaml
SWEEP=$WS/src/lio_localization_sim/docs/stage4_runs/sweep
CSV=$SWEEP/results.csv

if [ "$MODE" = "off" ]; then
  # Neutering the override rather than branching in C++: min(sigma, 1.0) leaves the normal
  # sigma untouched, so "off" exercises the identical code path minus the effect.
  EFF_SIG=1.0; EFF_YAW=1.0
else
  EFF_SIG=$SIG; EFF_YAW=$(python3 -c "print($SIG*10)")
fi

TAG="${MODE}_h${HOLD}_s${SIG}_rep${REP}"
CFG=$SWEEP/cfg_${TAG}.yaml
OUT=$SWEEP/$TAG
mkdir -p "$OUT"
rm -f "$OUT"/*.log "$OUT"/*.txt "$OUT/READY" /tmp/sim_eval_report.txt

python3 - "$BASE" "$CFG" "$HOLD" "$EFF_SIG" "$EFF_YAW" <<'PY'
import sys
base, out, hold, sig, yaw = sys.argv[1:6]
lines = open(base).read().rstrip('\n').split('\n')
lines += [f"    collision_hold_sec: {hold}",
          f"    collision_wall_sigma: {sig}",
          f"    collision_wall_yaw_sigma: {yaw}"]
open(out, 'w').write('\n'.join(lines) + '\n')
PY

NODE_PATTERN="imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint|fusion_stage_diag"

# Tear down on ANY exit, including being killed. When the caller's background task is reaped --
# which happens when a driving agent finishes or dies -- this script is terminated mid-run, and
# without this the ROS nodes it started survive as orphans holding port 10000, poisoning every
# subsequent run.
cleanup() {
  kill "${ROS_PID:-}" 2>/dev/null || true
  pkill -f "$NODE_PATTERN" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

pkill -f "$NODE_PATTERN"
# Port 10000 must be genuinely free before we start our own endpoint. A leftover endpoint from
# an earlier run keeps listening, and the readiness check below cannot tell it apart from ours:
# Unity would happily connect to the stale one, which is wired to nothing, and the run would
# look "ready" while no data ever reached this run's nodes. That is precisely how two sweep
# runs silently produced no data. Escalate to SIGKILL, then refuse to start rather than measure
# something meaningless.
for _ in $(seq 1 20); do
  ss -ltn 2>/dev/null | grep -q ':10000' || break
  sleep 0.5
done
if ss -ltn 2>/dev/null | grep -q ':10000'; then
  pkill -9 -f "default_server_endpoint" 2>/dev/null || true
  sleep 2
fi
if ss -ltn 2>/dev/null | grep -q ':10000'; then
  echo "ERROR: port 10000 still held by a stale process; refusing to start $TAG" >&2
  ss -ltnp 2>/dev/null | grep ':10000' >&2
  echo stale_port > "$OUT/FINISHED"
  exit 4
fi
sleep 1
# The ROS setup scripts read variables they never initialise, so relax -u just for them.
set +u
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
set -u

ros2 launch lio_localization_sim unity_sensor_localization.launch.py \
  field_config:="$CFG" > "$OUT/ros.log" 2>&1 &
ROS_PID=$!
for _ in $(seq 1 60); do ss -ltn 2>/dev/null | grep -q ':10000' && break; sleep 0.5; done
if ! ss -ltn 2>/dev/null | grep -q ':10000'; then
  echo "ERROR: endpoint never opened (see $OUT/ros.log)" >&2
  kill $ROS_PID 2>/dev/null; exit 3
fi

python3 "$WS/src/lio_localization_sim/tools/fusion_stage_diag.py" --ros-args \
  --params-file "$CFG" -p settle_sec:=0.5 > "$OUT/speed.log" 2>&1 &
sleep 2
touch "$OUT/READY"
echo "READY $TAG"

# The caller starts Play at this point. If it never does -- which is exactly what happened when
# the driving agent died mid-sweep and its remaining launches ran with Unity idle -- bail out
# in seconds instead of burning the full report timeout and recording an empty row that looks
# like a measurement. The evaluator announces the first ground truth it receives, so that line
# is a direct signal that Unity is actually feeding this run.
STATUS=ok
if ! timeout 180 bash -c \
  'until grep -q "first ground truth received" "$1" 2>/dev/null; do sleep 1; done' _ "$OUT/ros.log"
then
  STATUS=nodata
else
  # A run can also fail after data starts (a transport stall trips the IMU watchdog, the
  # evaluator never reaches its 60 s, and no report is ever written). Signal completion either
  # way through a marker the caller waits on, rather than letting it wait on the report itself.
  for _ in $(seq 1 400); do [ -f /tmp/sim_eval_report.txt ] && break; sleep 1; done
  [ -f /tmp/sim_eval_report.txt ] || STATUS=timeout
fi
echo "$STATUS" > "$OUT/FINISHED"
# Give the caller a moment to stop Play before ROS goes away, so Unity is not left publishing
# into a closed endpoint and spamming "Queue full".
sleep 8
kill $ROS_PID 2>/dev/null
pkill -f "$NODE_PATTERN"
cp /tmp/sim_eval_report.txt "$OUT/" 2>/dev/null

python3 - "$OUT" "$CSV" "$MODE" "$HOLD" "$SIG" "$REP" "$STATUS" <<'PY'
import os, re, sys
out, csv_path, mode, hold, sig, rep, status = sys.argv[1:8]

def read(p):
    try:
        return open(os.path.join(out, p), encoding='utf-8', errors='replace').read()
    except OSError:
        return ''

rep_txt, speed, ros = read('sim_eval_report.txt'), read('speed.log'), read('ros.log')

f = {'mode': mode, 'hold': hold, 'sigma': sig, 'rep': rep, 'status': status}
m = re.search(r'位置誤差:\s*mean=([\d.]+)mm\s+RMSE=([\d.]+)mm\s+max=([\d.]+)mm', rep_txt)
f['eval_mean'], f['eval_rmse'], f['eval_max'] = m.groups() if m else ('', '', '')
f['verdict'] = 'PASS' if ('最大位置誤差(<=10mm) PASS' in rep_txt) else 'FAIL'

# Last report block wins: it covers the whole settled session.
w = re.findall(r'\((\d+) windows', speed)
f['windows'] = w[-1] if w else ''
for side in ('inside', 'outside'):
    g = re.findall(
        rf'{side}:\s*n=(\d+)\s+mean=\s*([\d.]+)mm\s+p95=\s*([\d.]+)mm\s+max=\s*([\d.]+)mm', speed)
    n, mean, p95, mx = g[-1] if g else ('', '', '', '')
    f[f'{side}_n'], f[f'{side}_mean'] = n, mean
    f[f'{side}_p95'], f[f'{side}_max'] = p95, mx
f['detections'] = str(ros.count('point of no return'))

cols = ['mode', 'hold', 'sigma', 'rep', 'status', 'eval_mean', 'eval_rmse', 'eval_max', 'verdict',
        'windows', 'detections', 'inside_n', 'inside_mean', 'inside_p95', 'inside_max',
        'outside_n', 'outside_mean', 'outside_p95', 'outside_max']
new = not os.path.exists(csv_path)
with open(csv_path, 'a', encoding='utf-8') as fh:
    if new:
        fh.write(','.join(cols) + '\n')
    fh.write(','.join(f.get(c, '') for c in cols) + '\n')
print('ROW ' + ','.join(f'{c}={f.get(c, "")}' for c in
                        ('mode', 'hold', 'sigma', 'rep', 'status', 'eval_rmse', 'eval_max',
                         'windows', 'inside_p95', 'outside_p95')))
PY

echo "=== $TAG done ==="
