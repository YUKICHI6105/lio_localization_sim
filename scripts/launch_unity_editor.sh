#!/bin/bash
# Launch the Unity Editor on the Robocon2026Sim project from WSL.
#
# The Editor lives on the Windows side; WSL interop starts it directly. Nothing else is
# needed -- the scene is built procedurally by SimulationBootstrap's RuntimeInitializeOnLoad
# hook, and the MCP connection re-establishes itself once the project finishes loading.
#
# Only ever run ONE Editor. This machine has 15.7 GB of RAM and an idle Editor takes ~4 GB,
# so a second instance dies in graphics init with "Could not allocate memory" -- and has
# taken the user's running Editor down with it. The check below refuses rather than risk it.
#
# Usage: launch_unity_editor.sh
set -u

UNITY="${UNITY_EXECUTABLE:-/mnt/c/Program Files/Unity/Hub/Editor/6000.5.5f1/Editor/Unity.exe}"
WIN_PROJECT='C:\Users\kouza\UnityProjects\Robocon2026Sim'
WSL_PROJECT=/mnt/c/Users/kouza/UnityProjects/Robocon2026Sim
LOG=/tmp/unity_editor_launch.log

[ -x "$UNITY" ] || { echo "ERROR: Unity not found at $UNITY" >&2; exit 1; }
[ -d "$WSL_PROJECT" ] || { echo "ERROR: project not found at $WSL_PROJECT" >&2; exit 1; }

if tasklist.exe 2>/dev/null | grep -qiE '^Unity\.exe'; then
  echo "A Unity Editor is already running -- not starting a second one."
  tasklist.exe 2>/dev/null | grep -iE '^Unity\.exe'
  exit 0
fi

nohup "$UNITY" -projectPath "$WIN_PROJECT" > "$LOG" 2>&1 &
echo "launched (log: $LOG)"

# Loading the project takes 1-3 min. Memory growth is the readiness signal: the process
# starts around 90 MB and climbs past ~800 MB once the project is actually open.
echo "waiting for the project to finish loading..."
for _ in $(seq 1 60); do
  mem=$(tasklist.exe 2>/dev/null | grep -iE '^Unity\.exe' | head -1 | awk '{gsub(",","",$NF); print $NF}')
  [ -n "${mem:-}" ] && [ "$mem" -gt 800000 ] 2>/dev/null && { echo "Editor is up (${mem} K)"; exit 0; }
  sleep 5
done
echo "still loading after 5 min -- check the Editor window" >&2
