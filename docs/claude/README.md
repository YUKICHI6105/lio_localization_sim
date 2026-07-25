# Claude Code configuration and memory (backup)

Kept here so a machine rebuild does not lose it. These are **copies** — the live files sit
outside this repository and must be restored to their real locations to take effect again.

| File here | Restore to | What it does |
|---|---|---|
| `CLAUDE.md` | `<workspace>/.claude/CLAUDE.md` | Project instructions applied to every session in this workspace |
| `memory/*.md` | `~/.claude/projects/-home-<user>-ros2-ws/memory/` | Persistent notes carried across sessions: working preferences and project status |
| `ceer_history.jsonl` | `<workspace>/.claude/` | Usage history; informational only, safe to drop |

`memory/MEMORY.md` is the index loaded at the start of each session; the other files under
`memory/` are the entries it points at.

The directory name under `~/.claude/projects/` is derived from the workspace path, so it
changes if the workspace lives somewhere else after the rebuild.

Not copied: `scheduled_tasks.lock` holds a PID for a running session and means nothing on
another machine.
