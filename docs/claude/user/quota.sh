#!/usr/bin/env bash
# Claude Code quota fetcher
#
# Fetches 5-hour / 7-day rate-limit utilization from Anthropic's undocumented
# OAuth usage endpoint and caches it on disk.
#
# WHY a separate script and a cache:
#   The statusLine stdin JSON (.rate_limits.*) is only delivered to the
#   statusLine command, which the VS Code native UI never invokes. The only
#   other source is GET /api/oauth/usage - but that endpoint 429s very
#   aggressively (anthropics/claude-code#31637), so it must never be polled
#   directly by a UI refresh loop. Everything reads the cache; only this
#   script touches the network, and only when the cache is older than
#   MIN_INTERVAL.
#
# Concurrency: several Claude Code windows (TUI + VS Code) may run this at the
# same time. flock on a dedicated lock file serialises them, so the first one
# fetches and the rest fall through to the fresh cache.
#
# Output (stdout, always valid JSON - callers can rely on that):
#   {five_hour:{utilization,resets_at}, seven_day:{...}, fetched_at, stale}
#     stale=true  -> the numbers are older than MIN_INTERVAL because the
#                    refresh could not happen (offline, 429, expired token)
#
# The OAuth token is read but NEVER printed, logged, or passed on argv.

set -u

CACHE_DIR="$HOME/.claude/statusline-data"
CACHE_FILE="$CACHE_DIR/quota-cache.json"
LOCK_FILE="$CACHE_DIR/quota-cache.lock"
CREDS_FILE="$HOME/.claude/.credentials.json"

# Minimum seconds between real API calls. Do not lower this: the endpoint
# rate-limits hard and a 429 leaves the display stuck on stale data.
MIN_INTERVAL=300

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

mkdir -p "$CACHE_DIR" 2>/dev/null

# jq may not be on PATH yet in the same situation statusline.sh documents
# (installed after the Claude Code process started), so resolve it the same way.
if resolved_jq=$(command -v jq 2>/dev/null); then
  JQ_BIN="$resolved_jq"
else
  JQ_BIN="${LOCALAPPDATA:-}/Microsoft/WinGet/Packages/jqlang.jq_Microsoft.Winget.Source_8wekyb3d8bbwe/jq.exe"
fi
jq() { "$JQ_BIN" "$@"; }

EMPTY_JSON='{"five_hour":null,"seven_day":null,"fetched_at":0,"stale":true}'

# Print the cache (or an empty skeleton), forcing stale to the given value.
emit_cache() {
  local stale="$1"
  if [ -s "$CACHE_FILE" ]; then
    jq -c --argjson stale "$stale" '.stale = $stale' "$CACHE_FILE" 2>/dev/null \
      || printf '%s\n' "$EMPTY_JSON"
  else
    printf '%s\n' "$EMPTY_JSON"
  fi
}

cache_age() {
  local fetched now
  [ -s "$CACHE_FILE" ] || { echo 999999; return; }
  fetched=$(jq -r '.fetched_at // 0' "$CACHE_FILE" 2>/dev/null)
  case "$fetched" in ''|*[!0-9]*) fetched=0 ;; esac
  now=$(date +%s)
  echo $((now - fetched))
}

# --- fast path: cache still fresh, no lock, no network ---
age=$(cache_age)
if [ "$FORCE" -eq 0 ] && [ "$age" -lt "$MIN_INTERVAL" ]; then
  emit_cache false
  exit 0
fi

# --- refresh path ---
# Serialise refreshes across processes. -w 5: if another process is already
# fetching, wait briefly and then just serve whatever cache exists rather than
# piling on a second request.
exec 9>"$LOCK_FILE" 2>/dev/null
if ! flock -w 5 9 2>/dev/null; then
  emit_cache true
  exit 0
fi

# Another process may have refreshed while we waited for the lock.
age=$(cache_age)
if [ "$FORCE" -eq 0 ] && [ "$age" -lt "$MIN_INTERVAL" ]; then
  emit_cache false
  exit 0
fi

# Token: if it is missing or already expired, do not spend a request - Claude
# Code itself refreshes the credentials file, so a later run will succeed.
if [ ! -r "$CREDS_FILE" ]; then
  emit_cache true
  exit 0
fi

expires_at=$(jq -r '.claudeAiOauth.expiresAt // 0' "$CREDS_FILE" 2>/dev/null)
case "$expires_at" in ''|*[!0-9]*) expires_at=0 ;; esac
# expiresAt is in milliseconds.
now_ms=$(( $(date +%s) * 1000 ))
if [ "$expires_at" -gt 0 ] && [ "$now_ms" -ge "$expires_at" ]; then
  emit_cache true
  exit 0
fi

token=$(jq -r '.claudeAiOauth.accessToken // empty' "$CREDS_FILE" 2>/dev/null)
if [ -z "$token" ]; then
  emit_cache true
  exit 0
fi

# --silent --show-error is deliberately NOT used: any curl diagnostic could in
# principle echo the request. Errors are detected via the HTTP status instead.
response=$(curl -s --max-time 10 \
  -H "Authorization: Bearer $token" \
  -H "anthropic-beta: oauth-2025-04-20" \
  -w '\n%{http_code}' \
  https://api.anthropic.com/api/oauth/usage 2>/dev/null)
unset token

http_code=${response##*$'\n'}
body=${response%$'\n'*}

if [ "$http_code" != "200" ]; then
  # 429 / network failure / auth failure all land here. Keep serving the old
  # numbers flagged as stale rather than blanking the status bar.
  emit_cache true
  exit 0
fi

parsed=$(printf '%s' "$body" | jq -c --argjson now "$(date +%s)" '
  {
    five_hour:  (if .five_hour  then {utilization: .five_hour.utilization,  resets_at: .five_hour.resets_at}  else null end),
    seven_day:  (if .seven_day  then {utilization: .seven_day.utilization,  resets_at: .seven_day.resets_at}  else null end),
    fetched_at: $now,
    stale: false
  }' 2>/dev/null)

# Shape changed or truncated response -> keep the previous cache.
if [ -z "$parsed" ] || [ "$(printf '%s' "$parsed" | jq -r '.five_hour')" = "null" ]; then
  emit_cache true
  exit 0
fi

printf '%s\n' "$parsed" > "$CACHE_FILE.tmp" 2>/dev/null \
  && mv -f "$CACHE_FILE.tmp" "$CACHE_FILE" 2>/dev/null
chmod 600 "$CACHE_FILE" 2>/dev/null

printf '%s\n' "$parsed"
