#!/usr/bin/env bash
# Claude Code status line
# Shows: model | current context (%/tokens) | cumulative context (per-session, logged) | 5h quota | 7d quota
#
# Cumulative context volume is NOT provided by Claude Code itself; this script
# accumulates it locally by appending each new API call's token usage to a log
# file (skip-locks not applicable here, this is a plain append, no git).
# Scoped per-session (resets when a different session starts) rather than
# all-time: the point is checking whether rate-limit usage% growth
# accelerates as cum grows, and that relationship only makes sense within one
# continuous session - see flush_pending_to_log below.
#
# The per-session counters live in a PER-SESSION state file (see STATE_FILE
# below), not a single shared one. Claude Code sessions run as independent
# processes with no coordination between them, so two sessions open at once
# would otherwise interleave reads/writes of one shared file: each session's
# statusLine call would see the OTHER session's session_id as "the last one
# seen", misdetect that as its own session having just started, wrongly
# flush the other session's still-in-flight prompt as finished, and reset
# cum out from under it - repeatedly, every time focus alternates between
# them. Keying the file itself by session_id removes the shared mutable
# state entirely, so there's nothing left to race or misdetect.
#
# State/log files (both per-session, in their own folder - kept
# unbounded-shared-file growth and cross-session interference off the table
# at the same time, and easy for a user to browse - open one session's
# folder, see two normally-named files):
#   ~/.claude/statusline-data/sessions/<session_id>/state.json      -> cumulative counters + last seen prompt_id
#   ~/.claude/statusline-data/sessions/<session_id>/usage_log.jsonl -> append-only per-call log for later analysis

input=$(cat)

# jq was installed via winget after Claude Code's own process started, so its
# PATH may not include the install dir until Claude Code itself restarts.
# Resolve PATH first, fall back to the known winget package path so the
# statusline self-heals without requiring an app restart.
if resolved_jq=$(command -v jq 2>/dev/null); then
  JQ_BIN="$resolved_jq"
else
  JQ_BIN="$LOCALAPPDATA/Microsoft/WinGet/Packages/jqlang.jq_Microsoft.Winget.Source_8wekyb3d8bbwe/jq.exe"
fi
jq() { "$JQ_BIN" "$@"; }

STATE_DIR="$HOME/.claude/statusline-data"
mkdir -p "$STATE_DIR" 2>/dev/null
SESSIONS_DIR="$STATE_DIR/sessions"
mkdir -p "$SESSIONS_DIR" 2>/dev/null

# --- colors (terminal renders these dim already) ---
RESET=$'\033[0m'
GRAY=$'\033[90m'
GREEN=$'\033[92m'
YELLOW=$'\033[93m'
RED=$'\033[91m'
CYAN=$'\033[96m'
MAGENTA=$'\033[95m'

# Pace-ratio gradient (matches antigravity-cli's statusline.ps1): compares
# actual remaining quota against the remaining fraction you'd "expect" to
# have left if it were being spent at a perfectly even pace across the
# window. ratio >= 1.0 means you're ahead of an even pace (cool colors);
# ratio < 1.0 means you're burning through it faster than an even pace
# would (warm colors, reddest when running out well before reset).
RATIO_12=$'\033[38;5;33m'    # ratio >= 1.2 - blue
RATIO_11=$'\033[38;5;51m'    # ratio >= 1.1 - cyan
RATIO_10=$'\033[38;5;49m'    # ratio >= 1.0 - turquoise
RATIO_09=$'\033[38;5;46m'    # ratio >= 0.9 - green
RATIO_08=$'\033[38;5;154m'   # ratio >= 0.8 - yellow-green
RATIO_07=$'\033[38;5;226m'   # ratio >= 0.7 - yellow
RATIO_06=$'\033[38;5;214m'   # ratio >= 0.6 - orange-yellow
RATIO_05=$'\033[38;5;208m'   # ratio >= 0.5 - orange
RATIO_LT05=$'\033[38;5;196m' # ratio <  0.5 - red

FIVE_HOUR_PERIOD_SEC=18000   # 5h
WEEK_PERIOD_SEC=604800       # 7d

model=$(printf '%s' "$input" | jq -r '.model.display_name // "Claude"')
prompt_id=$(printf '%s' "$input" | jq -r '.prompt_id // empty')
session_id=$(printf '%s' "$input" | jq -r '.session_id // empty')

# One directory per session, holding a plainly-named state.json and
# usage_log.jsonl (see the header comment for why the state file can't be
# shared; the log is append-only so it never had that race, but a single
# shared log file still grows forever across every session ever run). A
# real subdirectory per session - rather than dot-suffixed flat filenames -
# is so a user browsing ~/.claude/statusline-data/sessions/ can open one
# session's folder and immediately see two normally-named files instead of
# hunting through one flat directory of many sessions' files. Sessions
# without a session_id (shouldn't normally happen - it's documented as
# always present) fall back to a fixed name rather than silently writing
# nowhere.
SESSION_DIR="$SESSIONS_DIR/${session_id:-unknown}"
mkdir -p "$SESSION_DIR" 2>/dev/null
STATE_FILE="$SESSION_DIR/state.json"
LOG_FILE="$SESSION_DIR/usage_log.jsonl"

ctx_used_pct=$(printf '%s' "$input" | jq -r '.context_window.used_percentage // empty')
ctx_total_in=$(printf '%s' "$input" | jq -r '.context_window.total_input_tokens // 0')

cur_input=$(printf '%s' "$input" | jq -r '.context_window.current_usage.input_tokens // 0')
cur_output=$(printf '%s' "$input" | jq -r '.context_window.current_usage.output_tokens // 0')
cur_cache_creation=$(printf '%s' "$input" | jq -r '.context_window.current_usage.cache_creation_input_tokens // 0')
cur_cache_read=$(printf '%s' "$input" | jq -r '.context_window.current_usage.cache_read_input_tokens // 0')

# Cache-cost-weighted variant of the same snapshot, recorded (not displayed)
# alongside plain cum for later comparison. current_usage only exposes the
# COMBINED cache_creation_input_tokens - unlike CEER, which parses transcript
# files directly and can see the real ephemeral_5m/1h split, this has no
# access to that breakdown. Since real transcript data showed Claude Code's
# cache writes are predominantly 1-hour TTL (2.0x), that's used as a flat
# approximation here rather than the 5-min default (1.25x). Input/output are
# left unweighted (1.0x) - only the cache asymmetry is being reflected.
cur_weighted=$(awk -v i="$cur_input" -v o="$cur_output" -v cc="$cur_cache_creation" -v cr="$cur_cache_read" \
  'BEGIN{printf "%.0f", i + cc*2.0 + cr*0.1 + o}')

five_used_pct=$(printf '%s' "$input" | jq -r '.rate_limits.five_hour.used_percentage // empty')
five_reset=$(printf '%s' "$input" | jq -r '.rate_limits.five_hour.resets_at // empty')
week_used_pct=$(printf '%s' "$input" | jq -r '.rate_limits.seven_day.used_percentage // empty')
week_reset=$(printf '%s' "$input" | jq -r '.rate_limits.seven_day.resets_at // empty')

# display remaining headroom rather than consumption
five_pct=$(awk -v p="$five_used_pct" 'BEGIN{ if (p=="") print ""; else printf "%.2f", 100-p }')
week_pct=$(awk -v p="$week_used_pct" 'BEGIN{ if (p=="") print ""; else printf "%.2f", 100-p }')

# --- load / init persisted state ---
# Schema: cumulative_* are COMMITTED totals (fully finished prompts only).
# pending_* is the latest usage snapshot for the prompt_id still in flight
# (last_prompt_id) - not yet committed. It exists because statusLine fires
# on every UI refresh, and current_usage for a single prompt_id grows across
# refreshes (a prompt can involve many internal tool-call round trips before
# it's done). Committing on the FIRST sighting of a prompt_id would freeze
# the count at a partial snapshot; instead we keep replacing pending_* with
# the latest snapshot, and only fold it into cumulative_* once prompt_id
# changes (i.e. that prompt has definitively finished).
if [ -f "$STATE_FILE" ]; then
  state=$(cat "$STATE_FILE")
else
  state='{"last_prompt_id":"","pending_input":0,"pending_output":0,"pending_cache_creation":0,"pending_cache_read":0,"pending_weighted":0,"pending_session_id":"","pending_model":"","pending_five_remain_pct":null,"pending_week_remain_pct":null,"cumulative_input":0,"cumulative_output":0,"cumulative_cache_creation":0,"cumulative_cache_read":0,"cumulative_total":0,"cumulative_weighted":0}'
fi

last_prompt_id=$(printf '%s' "$state" | jq -r '.last_prompt_id // ""')
pend_input=$(printf '%s' "$state" | jq -r '.pending_input // 0')
pend_output=$(printf '%s' "$state" | jq -r '.pending_output // 0')
pend_cache_creation=$(printf '%s' "$state" | jq -r '.pending_cache_creation // 0')
pend_cache_read=$(printf '%s' "$state" | jq -r '.pending_cache_read // 0')
pend_weighted=$(printf '%s' "$state" | jq -r '.pending_weighted // 0')
pend_session_id=$(printf '%s' "$state" | jq -r '.pending_session_id // ""')
pend_model=$(printf '%s' "$state" | jq -r '.pending_model // ""')
# five_pct/week_pct (remaining%) must be captured PER-PROMPT, not read fresh
# at commit time - the invocation that finalizes a prompt is the NEXT
# prompt's call, whose own rate_limits (if any) belong to a different
# moment. Track them as pending state just like the token counts above.
pend_five_pct=$(printf '%s' "$state" | jq -r '.pending_five_remain_pct // empty')
pend_week_pct=$(printf '%s' "$state" | jq -r '.pending_week_remain_pct // empty')
cum_input=$(printf '%s' "$state" | jq -r '.cumulative_input // 0')
cum_output=$(printf '%s' "$state" | jq -r '.cumulative_output // 0')
cum_cache_creation=$(printf '%s' "$state" | jq -r '.cumulative_cache_creation // 0')
cum_cache_read=$(printf '%s' "$state" | jq -r '.cumulative_cache_read // 0')
cum_total=$(printf '%s' "$state" | jq -r '.cumulative_total // 0')
cum_weighted=$(printf '%s' "$state" | jq -r '.cumulative_weighted // 0')

# Commits whatever prompt is currently pending (its LAST seen snapshot) into
# cum_* and appends one audit log line - shared by the prompt-boundary and
# session-boundary transitions below. No-op if there's nothing pending yet.
flush_pending_to_log() {
  local pend_total=$((pend_input + pend_output + pend_cache_creation + pend_cache_read))
  [ -n "$last_prompt_id" ] || return 0
  [ "$pend_total" -gt 0 ] || return 0

  cum_input=$((cum_input + pend_input))
  cum_output=$((cum_output + pend_output))
  cum_cache_creation=$((cum_cache_creation + pend_cache_creation))
  cum_cache_read=$((cum_cache_read + pend_cache_read))
  cum_total=$((cum_total + pend_total))
  cum_weighted=$((cum_weighted + pend_weighted))

  local ts five_pct_json week_pct_json log_line
  ts=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
  # Use the FINISHED prompt's own remain% (captured while it was pending),
  # not this invocation's - this invocation belongs to whatever comes next.
  # "" (no rate_limits captured for that prompt) becomes JSON null.
  five_pct_json="${pend_five_pct:-null}"
  week_pct_json="${pend_week_pct:-null}"
  log_line=$(jq -nc \
    --arg ts "$ts" \
    --arg session_id "$pend_session_id" \
    --arg prompt_id "$last_prompt_id" \
    --arg model "$pend_model" \
    --argjson input_tokens "$pend_input" \
    --argjson output_tokens "$pend_output" \
    --argjson cache_creation_input_tokens "$pend_cache_creation" \
    --argjson cache_read_input_tokens "$pend_cache_read" \
    --argjson call_total "$pend_total" \
    --argjson cumulative_total "$cum_total" \
    --argjson call_weighted_total "$pend_weighted" \
    --argjson cumulative_weighted_total "$cum_weighted" \
    --argjson five_remain_pct "$five_pct_json" \
    --argjson week_remain_pct "$week_pct_json" \
    '{ts:$ts, session_id:$session_id, prompt_id:$prompt_id, model:$model, input_tokens:$input_tokens, output_tokens:$output_tokens, cache_creation_input_tokens:$cache_creation_input_tokens, cache_read_input_tokens:$cache_read_input_tokens, call_total:$call_total, cumulative_total:$cumulative_total, call_weighted_total:$call_weighted_total, cumulative_weighted_total:$cumulative_weighted_total, five_remain_pct:$five_remain_pct, week_remain_pct:$week_remain_pct}')
  printf '%s\n' "$log_line" >> "$LOG_FILE"
}

# cum is scoped PER-SESSION: the point of tracking it is to check whether
# rate-limit usage% growth accelerates as cum grows, and that relationship
# is only meaningful within one continuous session. This used to be enforced
# by detecting a session_id change and resetting cum here - but since
# STATE_FILE is now itself keyed by session_id (see header comment), a
# different session simply reads/writes a different file and can never
# reach this code with a stale last_prompt_id from another session. Nothing
# left to detect or reset.
if [ -n "$prompt_id" ] && [ "$prompt_id" != "$last_prompt_id" ]; then
  # The prompt that was pending has finished (or this is the first prompt of
  # a session) - commit its last snapshot, then log it once.
  flush_pending_to_log

  # start a fresh pending snapshot for the new prompt
  last_prompt_id="$prompt_id"
  pend_input="$cur_input"; pend_output="$cur_output"
  pend_cache_creation="$cur_cache_creation"; pend_cache_read="$cur_cache_read"
  pend_weighted="$cur_weighted"
  pend_session_id="$session_id"; pend_model="$model"
  pend_five_pct="$five_pct"; pend_week_pct="$week_pct"
elif [ -n "$prompt_id" ]; then
  # same prompt still in flight - replace pending with the latest snapshot
  pend_input="$cur_input"; pend_output="$cur_output"
  pend_cache_creation="$cur_cache_creation"; pend_cache_read="$cur_cache_read"
  pend_weighted="$cur_weighted"
  pend_session_id="$session_id"; pend_model="$model"
  pend_five_pct="$five_pct"; pend_week_pct="$week_pct"
fi

five_pct_state_json="${pend_five_pct:-null}"
week_pct_state_json="${pend_week_pct:-null}"
new_state=$(jq -n \
  --arg last_prompt_id "$last_prompt_id" \
  --argjson pending_input "$pend_input" \
  --argjson pending_output "$pend_output" \
  --argjson pending_cache_creation "$pend_cache_creation" \
  --argjson pending_cache_read "$pend_cache_read" \
  --argjson pending_weighted "$pend_weighted" \
  --arg pending_session_id "$pend_session_id" \
  --arg pending_model "$pend_model" \
  --argjson pending_five_remain_pct "$five_pct_state_json" \
  --argjson pending_week_remain_pct "$week_pct_state_json" \
  --argjson cumulative_input "$cum_input" \
  --argjson cumulative_output "$cum_output" \
  --argjson cumulative_cache_creation "$cum_cache_creation" \
  --argjson cumulative_cache_read "$cum_cache_read" \
  --argjson cumulative_total "$cum_total" \
  --argjson cumulative_weighted "$cum_weighted" \
  '{last_prompt_id:$last_prompt_id, pending_input:$pending_input, pending_output:$pending_output, pending_cache_creation:$pending_cache_creation, pending_cache_read:$pending_cache_read, pending_weighted:$pending_weighted, pending_session_id:$pending_session_id, pending_model:$pending_model, pending_five_remain_pct:$pending_five_remain_pct, pending_week_remain_pct:$pending_week_remain_pct, cumulative_input:$cumulative_input, cumulative_output:$cumulative_output, cumulative_cache_creation:$cumulative_cache_creation, cumulative_cache_read:$cumulative_cache_read, cumulative_total:$cumulative_total, cumulative_weighted:$cumulative_weighted}')
printf '%s' "$new_state" > "$STATE_FILE"

# display total = committed + whatever the in-flight prompt has accrued so far
# (cum_weighted is recorded in state.json/usage_log.jsonl only - not displayed)
pend_total_display=$((pend_input + pend_output + pend_cache_creation + pend_cache_read))
cum_total_display=$((cum_total + pend_total_display))

# --- helpers ---
fmt_k() {
  awk -v n="$1" 'BEGIN{printf "%.1fk", n/1000}'
}

ctx_color() {
  awk -v p="$1" 'BEGIN{ if (p=="" ) {print "'"$GRAY"'"} else if (p+0<60) {print "'"$GREEN"'"} else if (p+0<90) {print "'"$YELLOW"'"} else {print "'"$RED"'"} }'
}

# remaining-percentage semantics: higher = more headroom left (opposite of
# the old used_percentage thresholds)
quota_color() {
  awk -v p="$1" 'BEGIN{ if (p=="" ) {print "'"$GRAY"'"} else if (p+0>=50) {print "'"$GREEN"'"} else if (p+0>=20) {print "'"$YELLOW"'"} else {print "'"$RED"'"} }'
}

# resets_at: epoch (seconds) when the window resets.
# remain_pct: remaining quota, 0-100 (empty -> gray "?" style, no ratio calc).
# period_sec: full length of the window (18000 for 5h, 604800 for 1w).
fmt_remaining() {
  local resets_at="$1" remain_pct="$2" period_sec="$3"
  if [ -z "$resets_at" ] || [ "$resets_at" = "null" ]; then
    printf '%s?%s' "$GRAY" "$RESET"
    return
  fi
  local now diff d h m tstr tcolor
  now=$(date +%s)
  diff=$((${resets_at%.*} - now))
  [ "$diff" -lt 0 ] && diff=0
  d=$((diff/86400)); h=$(((diff%86400)/3600)); m=$(((diff%3600)/60))
  if [ "$d" -gt 0 ]; then
    tstr=$(printf '%dd%dh%dm' "$d" "$h" "$m")
  else
    tstr=$(printf '%dh%dm' "$h" "$m")
  fi

  if [ -z "$remain_pct" ] || [ "$remain_pct" = "null" ]; then
    printf '%s%s%s' "$GRAY" "$tstr" "$RESET"
    return
  fi

  # ratio = (actual remaining fraction) / (fraction of the window's TIME
  # still remaining). >=1.0 => quota is being spent slower than an even
  # pace across the window (fine); <1.0 => faster than even pace (will
  # likely hit the limit before the window resets).
  tcolor=$(awk -v pct="$remain_pct" -v sec="$diff" -v period="$period_sec" '
    BEGIN{
      remain_frac = pct/100.0
      expected_frac = sec/period; if (expected_frac > 1.0) expected_frac = 1.0
      ratio = (expected_frac > 0) ? (remain_frac/expected_frac) : 1.0
      if (ratio>=1.2)      { print "'"$RATIO_12"'" }
      else if (ratio>=1.1) { print "'"$RATIO_11"'" }
      else if (ratio>=1.0) { print "'"$RATIO_10"'" }
      else if (ratio>=0.9) { print "'"$RATIO_09"'" }
      else if (ratio>=0.8) { print "'"$RATIO_08"'" }
      else if (ratio>=0.7) { print "'"$RATIO_07"'" }
      else if (ratio>=0.6) { print "'"$RATIO_06"'" }
      else if (ratio>=0.5) { print "'"$RATIO_05"'" }
      else                  { print "'"$RATIO_LT05"'" }
    }')
  printf '%s%s%s' "$tcolor" "$tstr" "$RESET"
}

fmt_pct() {
  local pct="$1" color="$2"
  if [ -z "$pct" ] || [ "$pct" = "null" ]; then
    printf '%sn/a%s' "$GRAY" "$RESET"
  else
    awk -v p="$pct" -v c="$color" -v r="$RESET" 'BEGIN{printf "%s%.0f%%%s", c, p, r}'
  fi
}

# --- build pieces ---
ctxC=$(ctx_color "$ctx_used_pct")
ctxPctStr=$(fmt_pct "$ctx_used_pct" "$ctxC")
ctxTokStr=$(fmt_k "$ctx_total_in")
cumTokStr=$(fmt_k "$cum_total_display")

five_c=$(quota_color "$five_pct")
week_c=$(quota_color "$week_pct")
fivePctStr=$(fmt_pct "$five_pct" "$five_c")
weekPctStr=$(fmt_pct "$week_pct" "$week_c")
fiveTimeStr=$(fmt_remaining "$five_reset" "$five_pct" "$FIVE_HOUR_PERIOD_SEC")
weekTimeStr=$(fmt_remaining "$week_reset" "$week_pct" "$WEEK_PERIOD_SEC")

sep="${GRAY}|${RESET}"

# =========================================================================
# CEER v9 (Context-to-Edit Efficiency Ratio) - realtime, git-driven
#
# No commit hook is used. Every statusline refresh asks git directly for
# the last commit (hash + time) and the live working-tree diff, so the
# score updates in real time and self-heals across session restarts.
# Token consumption since that commit is summed from every transcript
# file under the project directory (main sessions + subagents/*.jsonl),
# incrementally via a per-file line offset cache, so subagent usage is
# included and nothing is re-counted on the next refresh.
#
# v9 drops the old per-quota-window (5h/1w) weight tables - CEER measures
# context-to-edit efficiency, not a specific rate-limit window, so there's
# only one weight set now (see the `deltas` awk block below for the
# official-pricing-derived constants). Instead it tracks two variants:
# ceer_cached (actual cost, real cache discounts applied) and ceer_uncached
# (counterfactual cost if caching had not been used at all), so cache
# effectiveness can be measured after the fact from ceer_history.jsonl.
# =========================================================================
cwd=$(printf '%s' "$input" | jq -r '.cwd // empty')
transcript_path=$(printf '%s' "$input" | jq -r '.transcript_path // empty')

ceerCachedStr="${GRAY}n/a${RESET}"
ceerUncachedStr="${GRAY}n/a${RESET}"

git_root=""
if [ -n "$cwd" ]; then
  git_root=$(git -C "$cwd" rev-parse --show-toplevel 2>/dev/null)
fi

if [ -n "$git_root" ] && [ -n "$transcript_path" ]; then
  head_hash=$(git -C "$git_root" rev-parse HEAD 2>/dev/null)
  commit_epoch=$(git -C "$git_root" log -1 --format=%ct 2>/dev/null)

  if [ -n "$head_hash" ] && [ -n "$commit_epoch" ]; then
    proj_dir=$(dirname "$transcript_path")

    # live uncommitted net line change (working tree, staged+unstaged, vs HEAD)
    diffstat=$(git -C "$git_root" diff HEAD --shortstat 2>/dev/null)
    added=$(printf '%s' "$diffstat" | grep -o '[0-9]\+ insertion' | grep -o '[0-9]\+')
    removed=$(printf '%s' "$diffstat" | grep -o '[0-9]\+ deletion' | grep -o '[0-9]\+')
    added=${added:-0}; removed=${removed:-0}
    net_lines=$((added + removed))

    repo_hash=$(printf '%s' "$git_root" | md5sum | cut -d' ' -f1)
    CEER_DIR="$STATE_DIR/ceer"
    mkdir -p "$CEER_DIR" 2>/dev/null
    ceer_state_file="$CEER_DIR/$repo_hash.json"
    ceer_log_file="$git_root/.claude/ceer_history.jsonl"

    if [ -f "$ceer_state_file" ]; then
      cstate=$(cat "$ceer_state_file")
    else
      cstate='{"last_head":"","window_start_epoch":0,"processed_offsets":{},"capacity_cached":0,"capacity_uncached":0,"sum_input_new":0,"sum_cache_write_5m":0,"sum_cache_write_1h":0,"sum_cache_read":0,"sum_output":0}'
    fi

    last_head=$(printf '%s' "$cstate" | jq -r '.last_head // ""')
    window_start_epoch=$(printf '%s' "$cstate" | jq -r '.window_start_epoch // 0')

    first_run=false
    if [ -z "$last_head" ]; then
      first_run=true
      window_start_epoch="$commit_epoch"
      cstate=$(printf '%s' "$cstate" | jq --argjson ws "$window_start_epoch" '.window_start_epoch = $ws')
    fi

    # commit boundary: HEAD moved since the last refresh -> close out the
    # previous window as one audited log entry, then reset it.
    #
    # HEAD moving is not always "one new commit": batched commits made
    # without an intervening statusLine refresh (manual git commands,
    # subagents committing before returning to the parent), or branch
    # switches/pulls/rebases, all move HEAD too. The former just means the
    # window legitimately spans several commits (recorded via
    # commits_in_window, not hidden); the latter is not a linear commit
    # progression at all, so diffing it would report a line count that has
    # nothing to do with actual work done - detect that with
    # merge-base --is-ancestor and skip logging rather than pollute the
    # audit log with a meaningless number.
    if [ "$first_run" = false ] && [ "$head_hash" != "$last_head" ]; then
      is_linear=false
      if git -C "$git_root" merge-base --is-ancestor "$last_head" "$head_hash" 2>/dev/null; then
        is_linear=true
      fi

      if [ "$is_linear" = true ]; then
        old_cap_cached=$(printf '%s' "$cstate" | jq -r '.capacity_cached // 0')
        old_cap_uncached=$(printf '%s' "$cstate" | jq -r '.capacity_uncached // 0')
        old_sum_in=$(printf '%s' "$cstate" | jq -r '.sum_input_new // 0')
        old_sum_cw5=$(printf '%s' "$cstate" | jq -r '.sum_cache_write_5m // 0')
        old_sum_cw1=$(printf '%s' "$cstate" | jq -r '.sum_cache_write_1h // 0')
        old_sum_cr=$(printf '%s' "$cstate" | jq -r '.sum_cache_read // 0')
        old_sum_out=$(printf '%s' "$cstate" | jq -r '.sum_output // 0')

        commits_in_window=$(git -C "$git_root" rev-list --count "$last_head..$head_hash" 2>/dev/null)
        commits_in_window=${commits_in_window:-1}

        commit_diffstat=$(git -C "$git_root" diff --shortstat "$last_head" "$head_hash" 2>/dev/null)
        c_added=$(printf '%s' "$commit_diffstat" | grep -o '[0-9]\+ insertion' | grep -o '[0-9]\+')
        c_removed=$(printf '%s' "$commit_diffstat" | grep -o '[0-9]\+ deletion' | grep -o '[0-9]\+')
        c_added=${c_added:-0}; c_removed=${c_removed:-0}
        commit_net_lines=$((c_added + c_removed))

        commit_ceer_cached=$(awk -v n="$commit_net_lines" -v c="$old_cap_cached" 'BEGIN{ if(c>0) printf "%.2f", (n/c)*500000; else print 0 }')
        commit_ceer_uncached=$(awk -v n="$commit_net_lines" -v c="$old_cap_uncached" 'BEGIN{ if(c>0) printf "%.2f", (n/c)*500000; else print 0 }')

        log_entry=$(jq -nc \
          --arg ts "$(date -u +"%Y-%m-%dT%H:%M:%SZ")" \
          --arg commit_hash "$head_hash" \
          --arg model "$model" \
          --argjson commits_in_window "$commits_in_window" \
          --argjson lines_added "$c_added" \
          --argjson lines_deleted "$c_removed" \
          --argjson ceer_cached "$commit_ceer_cached" \
          --argjson ceer_uncached "$commit_ceer_uncached" \
          --argjson input_new "$old_sum_in" \
          --argjson cache_write_5m "$old_sum_cw5" \
          --argjson cache_write_1h "$old_sum_cw1" \
          --argjson cache_read "$old_sum_cr" \
          --argjson output "$old_sum_out" \
          '{timestamp:$ts, commit_hash:$commit_hash, model:$model, commits_in_window:$commits_in_window, git_metrics:{lines_added:$lines_added,lines_deleted:$lines_deleted}, calculated_scores:{ceer_cached:$ceer_cached,ceer_uncached:$ceer_uncached}, raw_aggregated_tokens:{total_input_tokens_new:$input_new,total_cache_creation_5m_tokens:$cache_write_5m,total_cache_creation_1h_tokens:$cache_write_1h,total_cache_creation_tokens:($cache_write_5m+$cache_write_1h),total_cache_read_tokens:$cache_read,total_output_tokens:$output}}')
        mkdir -p "$(dirname "$ceer_log_file")" 2>/dev/null
        printf '%s\n' "$log_entry" >> "$ceer_log_file"
      fi

      # reset the window regardless of linearity - always resync to the new
      # HEAD so the next window starts clean from here
      window_start_epoch="$commit_epoch"
      cstate=$(printf '%s' "$cstate" | jq --argjson ws "$window_start_epoch" \
        '.capacity_cached=0 | .capacity_uncached=0 | .sum_input_new=0 | .sum_cache_write_5m=0 | .sum_cache_write_1h=0 | .sum_cache_read=0 | .sum_output=0 | .window_start_epoch=$ws')
    fi

    # incremental scan of every transcript under the project dir (all
    # sessions' main threads + all subagents/*.jsonl), new lines only.
    window_start_iso=$(date -u -d "@$window_start_epoch" +"%Y-%m-%dT%H:%M:%S.000Z" 2>/dev/null)
    tsv_accum="$STATE_DIR/ceer_tsv_tmp_$$"
    : > "$tsv_accum"

    if [ -n "$proj_dir" ] && [ -d "$proj_dir" ]; then
      while IFS= read -r f; do
        [ -f "$f" ] || continue
        cur_lines=$(wc -l < "$f" 2>/dev/null); cur_lines=${cur_lines:-0}
        prev_lines=$(printf '%s' "$cstate" | jq -r --arg f "$f" '.processed_offsets[$f] // 0')
        if [ "$cur_lines" -gt "$prev_lines" ]; then
          tail -n +"$((prev_lines + 1))" "$f" 2>/dev/null | jq -r --arg ws "$window_start_iso" '
            select(.type=="assistant" and .message.usage != null and ((.timestamp // "") > $ws)) |
            # cache_creation_input_tokens is the combined total; the actual
            # per-TTL split (5-min vs 1-hour ephemeral cache, priced at 1.25x
            # vs 2.0x) lives in the nested .cache_creation object. Fall back
            # to treating the whole write as 5-min (the API default TTL) only
            # if that nested object is missing (older transcript format).
            (if .message.usage.cache_creation != null then
               [(.message.usage.cache_creation.ephemeral_5m_input_tokens // 0),
                (.message.usage.cache_creation.ephemeral_1h_input_tokens // 0)]
             else
               [(.message.usage.cache_creation_input_tokens // 0), 0]
             end) as $cw |
            [(.message.usage.input_tokens // 0),
             $cw[0],
             $cw[1],
             (.message.usage.cache_read_input_tokens // 0),
             (.message.usage.output_tokens // 0),
             (.message.model // "unknown")] | @tsv
          ' >> "$tsv_accum" 2>/dev/null
          cstate=$(printf '%s' "$cstate" | jq --arg f "$f" --argjson n "$cur_lines" '.processed_offsets[$f] = $n')
        fi
      done < <(find "$proj_dir" -type f -name '*.jsonl' 2>/dev/null)
    fi

    # Unified weight set (no more per-quota-window variants) grounded directly
    # in the official prompt-caching economics: cache writes cost 1.25x base
    # input price on a 5-min ephemeral TTL, but 2.0x on a 1-hour TTL - these
    # are billed differently and Claude Code's own transcripts show it uses
    # the 1-hour TTL for cache writes, so both must be weighted separately
    # rather than assuming a flat 1.25x. Cache reads cost ~0.1x regardless of
    # which TTL wrote them. The 5.0x output weight matches the ACTUAL
    # output/input price ratio, which is exactly 5x for every current model
    # (Haiku $1/$5, Sonnet $3/$15, Opus $5/$25, Fable $10/$50) - not an
    # arbitrary guess. W_model scales the whole thing by the model's
    # absolute input-price tier (Haiku=1x).
    #
    # Two variants are tracked side by side so cache effectiveness can be
    # audited later:
    #   cached   - real weights (1.25 / 2.0 / 0.1) = what actually happened
    #   uncached - counterfactual: cache_write (either TTL) and cache_read
    #              priced as if they were plain new input (1.0x each), i.e.
    #              "what this would have cost with caching switched off"
    deltas=$(awk -F'\t' '
      BEGIN {
        w["claude-sonnet-5"]=3.0; w["claude-opus-4-8"]=5.0
        w["claude-haiku-4-5-20251001"]=1.0; w["claude-fable-5"]=10.0
      }
      {
        wm = ($6 in w) ? w[$6] : 3.0
        cap_cached   += ($1 + $2*1.25 + $3*2.0 + $4*0.1 + $5*5.0) * wm
        cap_uncached += ($1 + $2*1.0  + $3*1.0 + $4*1.0 + $5*5.0) * wm
        si+=$1; scw5+=$2; scw1+=$3; scr+=$4; so+=$5
      }
      END { printf "%.4f\t%.4f\t%d\t%d\t%d\t%d\t%d", cap_cached+0, cap_uncached+0, si+0, scw5+0, scw1+0, scr+0, so+0 }
    ' "$tsv_accum")
    rm -f "$tsv_accum" 2>/dev/null

    d_cap_cached=$(printf '%s' "$deltas" | cut -f1)
    d_cap_uncached=$(printf '%s' "$deltas" | cut -f2)
    d_si=$(printf '%s' "$deltas" | cut -f3)
    d_scw5=$(printf '%s' "$deltas" | cut -f4)
    d_scw1=$(printf '%s' "$deltas" | cut -f5)
    d_scr=$(printf '%s' "$deltas" | cut -f6)
    d_so=$(printf '%s' "$deltas" | cut -f7)

    cstate=$(printf '%s' "$cstate" | jq \
      --argjson dc "${d_cap_cached:-0}" --argjson du "${d_cap_uncached:-0}" \
      --argjson di "${d_si:-0}" --argjson dcw5 "${d_scw5:-0}" --argjson dcw1 "${d_scw1:-0}" \
      --argjson dcr "${d_scr:-0}" --argjson do_ "${d_so:-0}" \
      '.capacity_cached += $dc | .capacity_uncached += $du | .sum_input_new += $di | .sum_cache_write_5m += $dcw5 | .sum_cache_write_1h += $dcw1 | .sum_cache_read += $dcr | .sum_output += $do_')

    cstate=$(printf '%s' "$cstate" | jq --arg h "$head_hash" '.last_head = $h')
    printf '%s' "$cstate" > "$ceer_state_file"

    cap_cached_now=$(printf '%s' "$cstate" | jq -r '.capacity_cached // 0')
    cap_uncached_now=$(printf '%s' "$cstate" | jq -r '.capacity_uncached // 0')

    ceer_cached=$(awk -v n="$net_lines" -v c="$cap_cached_now" 'BEGIN{ if(c>0) printf "%.2f", (n/c)*500000; else print "" }')
    ceer_uncached=$(awk -v n="$net_lines" -v c="$cap_uncached_now" 'BEGIN{ if(c>0) printf "%.2f", (n/c)*500000; else print "" }')

    ceer_color() {
      awk -v s="$1" 'BEGIN{ if(s=="") {print "'"$GRAY"'"} else if (s+0>=20.0) {print "'"$MAGENTA"'"} else if (s+0>=10.0) {print "'"$GREEN"'"} else if (s+0>=5.0) {print "'"$YELLOW"'"} else {print "'"$RED"'"} }'
    }

    # cached is the actionable, real-world score (colored by tier); uncached
    # is the reference/comparison figure (always gray, de-emphasized).
    cc=$(ceer_color "$ceer_cached")
    if [ -n "$ceer_cached" ]; then ceerCachedStr="${cc}${ceer_cached}${RESET}"; fi
    if [ -n "$ceer_uncached" ]; then ceerUncachedStr="${GRAY}${ceer_uncached}${RESET}"; fi
  fi
fi

# CEER is no longer duplicated per quota window (5h/1w are just quota
# display now); it's a single trailing "CEER:<cached>/<uncached>" pair.
printf '%s %s ctx:%s (%s%s%s) %s cum:%s%s%s %s 5h:%s %s %s 1w:%s %s %s CEER:%s/%s\n' \
  "$model" "$sep" \
  "$ctxPctStr" "$CYAN" "$ctxTokStr" "$RESET" \
  "$sep" "$MAGENTA" "$cumTokStr" "$RESET" \
  "$sep" "$fiveTimeStr" "$fivePctStr" \
  "$sep" "$weekTimeStr" "$weekPctStr" \
  "$sep" "$ceerCachedStr" "$ceerUncachedStr"
