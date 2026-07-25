---
name: feedback-sudo-stop
description: Never attempt or work around sudo-requiring operations; stop immediately and ask the user to run them
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 4058ad05-5bce-4dba-bd61-eea7b31467a7
  modified: 2026-07-20T14:00:00.547Z
---

When a command or fix requires `sudo` (or any elevated/system-level privilege this session doesn't have), stop immediately and ask the user to run it themselves — do not try alternate installs, background workarounds, or other approaches to route around the missing privilege.

**Why:** During spec-review work on the ROS2 LIO-SAM-based localization project, several tool gaps needed system packages (`poppler-utils` for PDF reading, `pandoc`/`weasyprint` for PDF generation, `fonts-noto-cjk` for Japanese font rendering). Asking the user to run `sudo apt-get install ...` each time worked cleanly. By contrast, when a later gap (mermaid-cli needing a headless Chromium for diagram rendering) didn't strictly require sudo, effort was spent digging through background installs, log inspection, and puppeteer browser-install attempts that ultimately failed and wasted time — the user then explicitly asked to stop that pattern and hand off anything sudo-related (and, by extension, similar rabbit-holes) immediately.

**How to apply:** The moment a needed fix appears to require `sudo` (installing an apt/system package, modifying system-level config, etc.), stop right there — don't attempt pip/npm/manual-download workarounds to avoid asking. State plainly what's missing and give the exact command for the user to run. Only resume once they confirm it's done. This applies broadly across the project, not just to the PDF-tooling case it was first observed in.

**2026-07-20 reinforcement**: The user explicitly restated this rule during the lio_localization ICP-outlier investigation — always ask the user for help the moment a sudo-requiring step appears, before proceeding. A subagent hit an apparent `sudo apt-get install ninja-build` requirement (sotoba's CMakeLists.txt demands the Ninja generator) and correctly stopped and reported it rather than attempting a workaround — the right behavior per this rule. Worth noting: before escalating, it's fine (and was productive here) to double-check whether the blocker is *real* — in that case it turned out to be a false alarm caused by the subagent mistakenly adding a header-only package (`sotoba`) to the colcon `--packages-select` list, which was never actually necessary (lio_localization includes it via a relative path, no colcon build of sotoba required). Verifying the root cause avoided an unnecessary sudo request. But once a privilege requirement is confirmed genuine, the original rule stands: stop and ask, don't route around it.
