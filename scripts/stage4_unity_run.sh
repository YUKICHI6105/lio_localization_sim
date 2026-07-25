#!/bin/bash
# Stage-4 autonomous verification: drive the Unity physics simulation headlessly from WSL and
# collect the ROS evaluator report, with no human pressing Play.
#
# Unity is installed on the Windows side but WSL interop can launch it, and the simulation is
# built procedurally by SimulationBootstrap's RuntimeInitializeOnLoadMethod hook, so entering
# play mode is enough.  We do that through a PlayMode test (Robocon2026.Tests) because batch
# mode supports -runTests officially, keeps the editor loop alive for the run, and exits on its
# own.  ROS must be listening before Unity starts or Unity's first /clock can expire the
# evaluator timer immediately.
#
# Usage: stage4_unity_run.sh <tag> [sim_seconds]
set -u

TAG="${1:?usage: stage4_unity_run.sh <tag> [sim_seconds]}"
SECONDS_ARG="${2:-45}"

WS=/home/yukichi6105/ros2_ws
# Batch runs use a dedicated clone so they never contend with the interactive Editor: Unity
# refuses to open one project twice, and requiring the user to close their Editor would defeat
# the point of autonomous verification.  Only Assets/Packages/ProjectSettings are synced (264
# KB); the clone keeps its own regenerable Library cache.
MASTER=/mnt/c/Users/kouza.FUKU-PC/UnityProjects/Robocon2026Sim
PROJECT=/mnt/c/Users/kouza.FUKU-PC/UnityProjects/Robocon2026Sim_Batch
WIN_PROJECT='C:\Users\kouza.FUKU-PC\UnityProjects\Robocon2026Sim_Batch'
UNITY="/mnt/c/Program Files/Unity/Hub/Editor/6000.5.4f1/Editor/Unity.exe"
OUT="$WS/src/lio_localization_sim/docs/stage4_runs/$TAG"

mkdir -p "$OUT" "$PROJECT"
rm -f /tmp/sim_eval_report.txt /tmp/sim_eval_plot.png /tmp/diag_raw_errors.csv

echo "syncing sources into the batch clone"
for d in Assets Packages ProjectSettings; do
  rsync -a --delete "$MASTER/$d/" "$PROJECT/$d/" || { echo "ERROR: sync of $d failed" >&2; exit 2; }
done

# The clone lets batch mode open a project while the Editor holds the master, but this machine
# has 15.7 GB of RAM and an idle Editor already takes ~4 GB, so a second Unity instance dies in
# graphics init with "Could not allocate memory".  Require the Editor to be closed and say so
# plainly rather than failing later with an opaque crash dump.
if tasklist.exe 2>/dev/null | grep -qiE '^Unity\.exe'; then
  echo "ERROR: a Unity Editor is running. Close it first — this machine cannot hold two Unity" >&2
  echo "       instances (15.7 GB RAM, Editor ~4 GB), and batch mode OOMs during graphics init." >&2
  echo "       Once it is closed, headless runs need no further interaction." >&2
  exit 2
fi
rm -f "$PROJECT/Temp/UnityLockfile"

pkill -f "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint" 2>/dev/null
sleep 1

# The ROS setup scripts reference unset variables, so relax -u just for them.
set +u
source /opt/ros/lyrical/setup.bash
source "$WS/install/setup.bash"
set -u

echo "=== [$TAG] starting ROS stack ==="
ros2 launch lio_localization_sim unity_sensor_localization.launch.py > "$OUT/ros.log" 2>&1 &
ROS_PID=$!

# Wait for the TCP endpoint to accept connections before Unity tries to attach.
for _ in $(seq 1 40); do
  ss -ltn 2>/dev/null | grep -q ':10000' && break
  sleep 0.5
done
if ! ss -ltn 2>/dev/null | grep -q ':10000'; then
  echo "ERROR: ROS TCP endpoint never opened :10000 (see $OUT/ros.log)" >&2
  kill $ROS_PID 2>/dev/null; exit 3
fi
echo "endpoint ready; launching Unity headless for ${SECONDS_ARG}s of sim time"

# -nographics keeps this off the GPU; the LiDAR uses Physics.Raycast, which does not need it.
timeout 900 "$UNITY" \
  -batchmode -nographics -accept-apiupdate \
  -projectPath "$WIN_PROJECT" \
  -runTests -testPlatform PlayMode \
  -testFilter Robocon2026.Tests.HeadlessSimulationRun \
  -testResults "$WIN_PROJECT\\Temp\\stage4_results.xml" \
  -logFile "$WIN_PROJECT\\Temp\\stage4_unity.log" \
  -roboconSeconds "$SECONDS_ARG" > "$OUT/unity_stdout.log" 2>&1
UNITY_RC=$?
echo "Unity exited rc=$UNITY_RC"

# The ROS launch shuts itself down when the evaluator finishes; give it a moment either way.
for _ in $(seq 1 40); do
  [ -f /tmp/sim_eval_report.txt ] && break
  sleep 0.5
done
sleep 2
kill $ROS_PID 2>/dev/null
pkill -f "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|evaluator_node|ball_tracking_node|default_server_endpoint" 2>/dev/null

cp /tmp/sim_eval_report.txt "$OUT/" 2>/dev/null
cp /tmp/sim_eval_plot.png "$OUT/" 2>/dev/null
cp /tmp/diag_raw_errors.csv "$OUT/" 2>/dev/null
cp "$PROJECT/Temp/stage4_unity.log" "$OUT/unity_editor.log" 2>/dev/null
cp "$PROJECT/Temp/stage4_results.xml" "$OUT/" 2>/dev/null

echo "=== [$TAG] done (unity rc=$UNITY_RC) -> $OUT ==="
[ -f "$OUT/sim_eval_report.txt" ] && grep -E "位置誤差|ヨー誤差|判定" "$OUT/sim_eval_report.txt" | head -5 \
  || echo "NO EVALUATOR REPORT — check $OUT/ros.log and $OUT/unity_editor.log"
