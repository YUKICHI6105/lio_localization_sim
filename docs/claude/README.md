# Claude Code configuration (backup)

Kept here so a machine rebuild does not lose it. These are **copies** — the live files sit
outside this repository and must be restored to their real locations to take effect again.

## Project level

| File here | Restore to |
|---|---|
| `CLAUDE.md` | `<workspace>/.claude/CLAUDE.md` |
| `ceer_history.jsonl` | `<workspace>/.claude/` — informational only, safe to drop |
| `memory/*.md` | `~/.claude/projects/-home-<user>-ros2-ws/memory/` |

`memory/MEMORY.md` is the index loaded at the start of each session; the other files under
`memory/` are the entries it points at. The directory name under `~/.claude/projects/` is
derived from the workspace path, so it changes if the workspace ends up somewhere else.

## User level

Everything in `user/` belongs directly in `~/.claude/`, preserving the `hooks/` and `skills/`
subdirectories. `quota.sh`, `statusline.sh` and `statusline-vscode.sh` need `chmod +x`.

| File | What it does |
|---|---|
| `user/CLAUDE.md` | Global instructions applied to every project |
| `user/settings.json` | Model, effort, theme, status line, and the hook wiring below |
| `user/hooks/turn-start.sh` | Runs on each prompt submission |
| `user/hooks/notify-stop.sh` | Runs when a turn ends — the Linux half of the notifications |
| `user/quota.sh` | Reports 5 h / 7 d quota usage into the status line |
| `user/statusline.sh`, `user/statusline-vscode.sh` | Status line: model, context usage, quota |
| `user/skills/` | Custom skills (`orchestration-greenfield`, `orchestration-with-base`) |

## Windows side

`windows/notify-windows.ps1` → `C:\Users\<you>\.claude\notify-windows.ps1`

`settings.json` invokes it by absolute path for desktop notifications:

```
powershell.exe -NoProfile -ExecutionPolicy Bypass -File C:/Users/kouza.FUKU-PC/.claude/notify-windows.ps1
```

**That path contains the old Windows username and must be updated** after a rebuild, or
notifications silently stop working.

## Deliberately not backed up

- `.credentials.json` — OAuth tokens. Secret, and re-created by logging in again.
- `history.jsonl`, `.claude.json` — prompt history and per-project state. Private, large, and
  not needed to restore behaviour.
