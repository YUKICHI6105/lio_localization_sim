---
name: feedback-test-run-protocol
description: "検証実行は1回→確認→OKなら2,3回目。1回目でエラーが出たら即座に原因対処(まとめてN回流さない)"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 2d93f426-69e5-4786-8c3c-79b4be4f7d43
  modified: 2026-07-20T05:28:44.729Z
---

シミュレーション等の検証は、まず1回だけ実行して結果を確認し、うまくいった場合のみ2回目・3回目を実行する。1回目でエラーが出たら、残りの実行を続けずすぐにその原因への対処に移る。

**Why:** エラーがあるのに3回一括で流すと、壊れた条件での実行に時間(180秒×N)を浪費し、フィードバックループが遅くなる。

**How to apply:** バッチスクリプトを一括N回で起動せず、1回ずつ起動して結果(RMSE・エラー件数)を確認するゲートを挟む。実行中のバッチでも途中結果にエラーを見つけたら中断して対処する。[[feedback-debug-retry-limit]] [[feedback-subagent-cost-policy]]
