#!/usr/bin/env bash
# Claude Code status line for the VS Code native UI.
#
# The VS Code extension never invokes the `statusLine` command (confirmed: no
# rendering code in its webview bundle, and no per-session directory is ever
# created under statusline-data/sessions/). So this variant takes NO stdin and
# reconstructs the same information from sources reachable outside the TUI:
#
#   model / context / cumulative -> the session transcript JSONL
#   5h & 7d quota                -> quota.sh (cached /api/oauth/usage)
#
# Output is deliberately plain text - a VS Code StatusBarItem renders no ANSI -
# so the colour tiers of statusline.sh become severity glyphs instead:
#   ● comfortable   ▲ watch   ■ tight
#
# Line 1 = status bar text. Lines 2+ = tooltip detail. The extension splits on
# the first newline.
#
# Usage: statusline-vscode.sh [cwd]

set -u

CWD="${1:-$PWD}"

STATE_DIR="$HOME/.claude/statusline-data"
CUM_DIR="$STATE_DIR/vscode-cum"
QUOTA_SH="$HOME/.claude/quota.sh"

FIVE_HOUR_PERIOD_SEC=18000
WEEK_PERIOD_SEC=604800

mkdir -p "$CUM_DIR" 2>/dev/null

if resolved_jq=$(command -v jq 2>/dev/null); then
  JQ_BIN="$resolved_jq"
else
  JQ_BIN="${LOCALAPPDATA:-}/Microsoft/WinGet/Packages/jqlang.jq_Microsoft.Winget.Source_8wekyb3d8bbwe/jq.exe"
fi
jq() { "$JQ_BIN" "$@"; }

# --- locate the active transcript -------------------------------------------
# Claude Code stores transcripts under ~/.claude/projects/<cwd with every
# non-alphanumeric character replaced by '-'>/<session_id>.jsonl. The active
# session is simply the most recently written one (subagent transcripts live in
# a subdirectory, so a plain non-recursive glob already excludes them).
#
# CAVEAT: nothing maps a VS Code window to its session id, so with two sessions
# running in the same project the model/context figures follow whichever one
# wrote last. The quota figures are account-wide and unaffected.
proj_slug=$(printf '%s' "$CWD" | sed 's/[^a-zA-Z0-9]/-/g')
PROJ_DIR="$HOME/.claude/projects/$proj_slug"

transcript=""
if [ -d "$PROJ_DIR" ]; then
  transcript=$(ls -t "$PROJ_DIR"/*.jsonl 2>/dev/null | head -1)
fi

# --- model + current context -------------------------------------------------
model="Claude"
ctx_used=0

if [ -n "$transcript" ] && [ -s "$transcript" ]; then
  # The newest assistant turn carries the full prompt that was just sent, so
  # its input+cache tokens ARE the current context occupancy. Reading the tail
  # keeps this O(1) on multi-megabyte transcripts.
  read -r model_id ctx_used <<EOF
$(tail -n 400 "$transcript" 2>/dev/null | jq -rs '
    [ .[] | select(.type=="assistant" and .message.usage != null) ] | last |
    if . == null then "- 0"
    else
      ((.message.model // "-") | tostring) + " " +
      (((.message.usage.input_tokens // 0)
        + (.message.usage.cache_read_input_tokens // 0)
        + (.message.usage.cache_creation_input_tokens // 0)) | tostring)
    end' 2>/dev/null)
EOF
  case "${ctx_used:-}" in ''|*[!0-9]*) ctx_used=0 ;; esac

  case "${model_id:-}" in
    *opus-4-8*)   model="Opus 4.8" ;;
    *opus*)       model="Opus" ;;
    *fable-5*)    model="Fable 5" ;;
    *sonnet-5*)   model="Sonnet 5" ;;
    *sonnet*)     model="Sonnet" ;;
    *haiku-4-5*)  model="Haiku 4.5" ;;
    *haiku*)      model="Haiku" ;;
    -|'')         model="Claude" ;;
    *)            model="${model_id#claude-}" ;;
  esac
fi

# Window size is not recorded in the transcript. 200k is the default; a session
# on the 1M context beta simply exceeds it, which is itself the signal.
ctx_window=200000
[ "$ctx_used" -gt 200000 ] && ctx_window=1000000
ctx_pct=$(awk -v u="$ctx_used" -v w="$ctx_window" 'BEGIN{printf "%.0f", (u*100.0)/w}')

# --- cumulative tokens for this transcript -----------------------------------
# Incremental: remember how many bytes of the file have already been summed and
# only parse what has been appended since. A full re-scan every 30s refresh
# would be wasteful on long sessions.
cum_total=0
if [ -n "$transcript" ] && [ -s "$transcript" ]; then
  key=$(printf '%s' "$transcript" | md5sum 2>/dev/null | cut -c1-16)
  cum_file="$CUM_DIR/${key:-unknown}.json"

  offset=0; cum_total=0
  if [ -s "$cum_file" ]; then
    offset=$(jq -r '.offset // 0' "$cum_file" 2>/dev/null)
    cum_total=$(jq -r '.cum_total // 0' "$cum_file" 2>/dev/null)
    case "$offset" in ''|*[!0-9]*) offset=0 ;; esac
    case "$cum_total" in ''|*[!0-9]*) cum_total=0 ;; esac
  fi

  size=$(stat -c %s "$transcript" 2>/dev/null || echo 0)
  # File shrank (compacted or a different session reusing the name) -> restart.
  if [ "$size" -lt "$offset" ]; then
    offset=0; cum_total=0
  fi

  if [ "$size" -gt "$offset" ]; then
    # Claude Code appends to the transcript while this runs, so the unread
    # region may end mid-line. A partial line makes jq reject the whole batch,
    # and advancing the offset past it would drop those tokens permanently, so
    # only whole lines are ever consumed.
    # If the file's last byte is a newline the whole unread region is complete
    # lines; otherwise `head -n -1` drops the partial tail and wc -c measures
    # exactly how many bytes were safe to consume. Both paths stream, so a
    # first run over a multi-megabyte transcript never buffers it in a variable.
    if [ "$(tail -c 1 "$transcript" 2>/dev/null | wc -l)" -eq 1 ]; then
      trim() { cat; }
      consumed=$((size - offset))
    else
      trim() { head -n -1; }
      consumed=$(tail -c "+$((offset + 1))" "$transcript" 2>/dev/null | trim | wc -c)
    fi
    case "${consumed:-}" in ''|*[!0-9]*) consumed=0 ;; esac

    if [ "$consumed" -gt 0 ]; then
      delta=$(tail -c "+$((offset + 1))" "$transcript" 2>/dev/null | trim | jq -rs '
        [ .[] | select(.type=="assistant" and .message.usage != null) |
          (.message.usage.input_tokens // 0)
          + (.message.usage.output_tokens // 0)
          + (.message.usage.cache_creation_input_tokens // 0)
          + (.message.usage.cache_read_input_tokens // 0)
        ] | add // 0' 2>/dev/null)
      # jq failing (truncated JSON despite the trim) must not advance the offset.
      case "${delta:-}" in
        ''|*[!0-9]*) : ;;
        *)
          cum_total=$((cum_total + delta))
          printf '{"offset":%s,"cum_total":%s}\n' "$((offset + consumed))" "$cum_total" > "$cum_file" 2>/dev/null
          ;;
      esac
    fi
  fi
fi

# --- quota -------------------------------------------------------------------
quota_json='{"five_hour":null,"seven_day":null,"stale":true}'
if [ -x "$QUOTA_SH" ] || [ -r "$QUOTA_SH" ]; then
  q=$(bash "$QUOTA_SH" 2>/dev/null)
  [ -n "$q" ] && quota_json="$q"
fi

five_used=$(printf '%s' "$quota_json" | jq -r '.five_hour.utilization // empty' 2>/dev/null)
five_reset=$(printf '%s' "$quota_json" | jq -r '.five_hour.resets_at // empty' 2>/dev/null)
week_used=$(printf '%s' "$quota_json" | jq -r '.seven_day.utilization // empty' 2>/dev/null)
week_reset=$(printf '%s' "$quota_json" | jq -r '.seven_day.resets_at // empty' 2>/dev/null)
stale=$(printf '%s' "$quota_json" | jq -r '.stale // false' 2>/dev/null)

# Same semantics as statusline.sh: display remaining headroom, not consumption.
five_pct=$(awk -v p="$five_used" 'BEGIN{ if (p=="") print ""; else printf "%.0f", 100-p }')
week_pct=$(awk -v p="$week_used" 'BEGIN{ if (p=="") print ""; else printf "%.0f", 100-p }')

# --- formatting helpers ------------------------------------------------------
fmt_k() { awk -v n="$1" 'BEGIN{printf "%.1fk", n/1000}'; }

# Context severity: rises with consumption.
ctx_glyph() {
  awk -v p="$1" 'BEGIN{ if (p+0<60) print "●"; else if (p+0<90) print "▲"; else print "■" }'
}

# Quota severity: falls with remaining headroom.
quota_glyph() {
  awk -v p="$1" 'BEGIN{ if (p=="") print "·"; else if (p+0>=50) print "●"; else if (p+0>=20) print "▲"; else print "■" }'
}

# ISO8601 -> "3h20m" / "5d2h10m" until reset.
fmt_until() {
  local iso="$1" epoch now diff d h m
  [ -z "$iso" ] && { printf '?'; return; }
  epoch=$(date -d "$iso" +%s 2>/dev/null) || { printf '?'; return; }
  [ -z "$epoch" ] && { printf '?'; return; }
  now=$(date +%s)
  diff=$((epoch - now)); [ "$diff" -lt 0 ] && diff=0
  d=$((diff/86400)); h=$(((diff%86400)/3600)); m=$(((diff%3600)/60))
  if [ "$d" -gt 0 ]; then printf '%dd%dh%dm' "$d" "$h" "$m"; else printf '%dh%dm' "$h" "$m"; fi
}

# Pace ratio, ported from statusline.sh: remaining quota fraction divided by the
# fraction of the window's time still left. >=1.0 means the quota is being spent
# slower than an even pace and will last; <1.0 means it will run out early.
pace_ratio() {
  local iso="$1" pct="$2" period="$3" epoch now diff
  { [ -z "$iso" ] || [ -z "$pct" ]; } && { printf ''; return; }
  epoch=$(date -d "$iso" +%s 2>/dev/null) || { printf ''; return; }
  now=$(date +%s); diff=$((epoch - now)); [ "$diff" -lt 0 ] && diff=0
  awk -v pct="$pct" -v sec="$diff" -v period="$period" 'BEGIN{
    ef = sec/period; if (ef > 1.0) ef = 1.0
    if (ef <= 0) { print ""; exit }
    printf "%.2f", (pct/100.0)/ef
  }'
}

fmt_pct() { [ -z "$1" ] && printf 'n/a' || printf '%s%%' "$1"; }

# --- compose -----------------------------------------------------------------
staleMark=""
[ "$stale" = "true" ] && staleMark="~"

line1=$(printf '%s | ctx:%s %s%% (%s) | cum:%s | 5h:%s%s%s | 1w:%s%s%s' \
  "$model" \
  "$(ctx_glyph "$ctx_pct")" "$ctx_pct" "$(fmt_k "$ctx_used")" \
  "$(fmt_k "$cum_total")" \
  "$(quota_glyph "$five_pct")" "$staleMark" "$(fmt_pct "$five_pct")" \
  "$(quota_glyph "$week_pct")" "$staleMark" "$(fmt_pct "$week_pct")")

five_ratio=$(pace_ratio "$five_reset" "$five_pct" "$FIVE_HOUR_PERIOD_SEC")
week_ratio=$(pace_ratio "$week_reset" "$week_pct" "$WEEK_PERIOD_SEC")

printf '%s\n' "$line1"
printf 'Model: %s\n' "$model"
printf 'Context: %s / %s tokens (%s%%)\n' "$(fmt_k "$ctx_used")" "$(fmt_k "$ctx_window")" "$ctx_pct"
printf 'Cumulative (this session): %s tokens\n' "$(fmt_k "$cum_total")"
printf '5h quota: %s left, resets in %s (pace %s)\n' "$(fmt_pct "$five_pct")" "$(fmt_until "$five_reset")" "${five_ratio:-n/a}"
printf '7d quota: %s left, resets in %s (pace %s)\n' "$(fmt_pct "$week_pct")" "$(fmt_until "$week_reset")" "${week_ratio:-n/a}"
printf 'Pace >=1.00 means the quota outlasts the window at the current rate.\n'
[ "$stale" = "true" ] && printf 'Quota data is stale (~): refresh failed or rate-limited.\n'
[ -n "$transcript" ] && printf 'Transcript: %s\n' "$transcript"
