#!/usr/bin/env bash
# Stop フック: AIの処理が完了したときにWindowsトースト通知を出す。
# 最後のアシスタント発言の要約と所要時間を通知本文に載せる。
# 表示側は既存の notify-windows.ps1 に stdin JSON を渡して任せる。

PS1_PATH='C:/Users/kouza.FUKU-PC/.claude/notify-windows.ps1'

payload=$(cat)
sid=$(printf '%s' "$payload" | jq -r '.session_id // ""' 2>/dev/null)
cwd=$(printf '%s' "$payload" | jq -r '.cwd // ""' 2>/dev/null)
transcript=$(printf '%s' "$payload" | jq -r '.transcript_path // ""' 2>/dev/null)

# --- 所要時間 -------------------------------------------------------------
elapsed_label=''
start_file="/tmp/claude-turn-start-${sid}"
if [ -n "$sid" ] && [ -f "$start_file" ]; then
    start=$(cat "$start_file" 2>/dev/null)
    rm -f "$start_file"
    if [ -n "$start" ]; then
        sec=$(( $(date +%s) - start ))
        [ "$sec" -lt 0 ] && sec=0
        if [ "$sec" -ge 60 ]; then
            elapsed_label="$((sec / 60))分$((sec % 60))秒"
        else
            elapsed_label="${sec}秒"
        fi
    fi
fi

# --- 最後のアシスタント発言の要約 ------------------------------------------
# tac で逆順にするため最初にヒットした行が最終発言。thinking ブロックは type 違いで除外される。
summary=''
if [ -n "$transcript" ] && [ -f "$transcript" ]; then
    summary=$(tac "$transcript" 2>/dev/null \
        | jq -r 'select(.type=="assistant") | .message.content[]? | select(.type=="text") | .text' 2>/dev/null \
        | head -1)
fi
# 改行を潰し、先頭100文字に切り詰める
summary=$(printf '%s' "$summary" | tr '\n\r\t' '   ' | sed 's/  */ /g; s/^ //; s/ $//')
if [ -z "$summary" ]; then
    summary='処理が完了しました'
elif [ "${#summary}" -gt 100 ]; then
    summary="${summary:0:100}…"
fi

# --- 通知 ------------------------------------------------------------------
if [ -n "$elapsed_label" ]; then
    title="Claude Code — 完了 (${elapsed_label})"
else
    title="Claude Code — 完了"
fi

jq -nc --arg t "$title" --arg m "$summary" --arg c "$cwd" --arg s "$sid" \
    '{title:$t, message:$m, cwd:$c, session_id:$s}' 2>/dev/null \
    | powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$PS1_PATH" >/dev/null 2>&1

# 通知の失敗でセッションを止めない
exit 0
