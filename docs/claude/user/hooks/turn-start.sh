#!/usr/bin/env bash
# UserPromptSubmit フック: ターン開始時刻を記録する。
# notify-stop.sh が Stop 時にこれを読んで所要時間を計算する。
sid=$(jq -r '.session_id // "unknown"' 2>/dev/null) || sid="unknown"
[ -z "$sid" ] && sid="unknown"
date +%s > "/tmp/claude-turn-start-${sid}" 2>/dev/null
exit 0
