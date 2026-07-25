---
name: feedback-debug-retry-limit
description: "Stop and ask the user after 3 failed attempts to identify a bug's root cause"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 4058ad05-5bce-4dba-bd61-eea7b31467a7
  modified: 2026-07-19T15:05:03.585Z
---

When investigating an error or bug, if 3 attempts to identify the root cause fail, stop self-directed investigation and report to the user instead of continuing to dig alone.

**Why:** During a long simulation-divergence debugging session (LIO-SAM-style SLAM stack), the user asked that repeated failed root-cause attempts trigger a check-in rather than open-ended autonomous digging. This was added directly to `~/.claude/CLAUDE.md` as a standing global rule, not just this project.

**How to apply:** Track investigation attempts on a single bug/error. After ~3 distinct hypotheses have been tried and ruled out (or fixes attempted and failed) without finding the true root cause, pause and summarize: what was tried, what was ruled out, what remains unknown — then ask the user how to proceed rather than starting a 4th independent line of investigation unprompted. This does not apply to routine multi-step debugging that is making steady forward progress (e.g., each step narrows the search and the previous step's finding directly motivates the next) — it applies specifically when attempts are failing to converge on a cause. See [[feedback_sudo_stop]] for a related "stop and ask" pattern (sudo-gated actions).
