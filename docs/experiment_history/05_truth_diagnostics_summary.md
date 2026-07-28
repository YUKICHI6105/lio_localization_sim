# 実験履歴: 真値使用版診断(オフラインICP・シード比較)

- 実行環境: 本番のROSフルスタック(Unity含む)を**同時に**動かした状態で、
  `/ground_truth_pose`(真値)を種または比較対象に使うオフライン診断ノード群を並行実行する。
  診断結果を評価器や推定器へフィードバックすることはない、観測専用の構成。
- 目的: 本番マッチャ(laser_scan_matching)・融合(backend_optimizer)・出力(imu_preintegration)
  のどの段階で誤差が発生しているかを切り分けること。
- 主なツール: `tools/icp_from_truth_diag.py`, `tools/fusion_stage_diag.py`,
  `scripts/stage4_icp_diag.sh`, `icp_seed_mode_diag`(wall_bias診断含む)。
- 生ログ: `docs/stage4_runs/icp_blind90/`, `icp_seed_mode/`, `icp_seed_prod/`。

## 1. 絶対に守るべき前提: オープンループで測ってはいけない

このカテゴリの診断で最も重要な教訓(`04_stage4_resume_notes.md`§11-4)。

診断ツール単体だけを起動するとROSフルスタックが無く、`HasOdomEstimate=false`で
制御が真値へフォールバックする。ロボットは理想経路を走って公称の終点に停まり、
**本番とは別の姿勢を測ってしまう**。停止位置が違えば見える壁の構成も変わり、
ICPのバイアスも変わるため、フルスタックと診断ツールを同時に走らせて初めて
意味のある比較になる。

この教訓を無視した初期の測定では「本番マッチャ14.8mm vs オフラインICP 2.18mm」という
大きな食い違いが生じ、シード依存性・マップ差・デスキュー・Huber・間引きという
5つの実装仮説を無駄に検証して全て棄却する結果になった。フルスタック同時実行で
測り直すと、以下の通り実装差は存在しなかった。

| 測定対象 | |bias| |
|---|---:|
| production(動作中のマッチャ) | 14.81mm |
| truth_prod(同じスキャンをオフラインで、本番と同一設定) | 14.68mm |
| offset_prod(真値+15mmオフセットのシードから) | 14.78mm |
| truth(素の最小二乗、Huberなし) | 30.57mm |

Huberは害ではなく大きく効いている(外すと倍化する)ことも同時に判明した。

**以後、精度に関する計測は必ずフルスタックを動かした状態で行う**方針とした。

## 2. `icp_from_truth_diag`: 真値姿勢からのICP単独診断

真値の姿勢をシードにICPだけを実行し、マップとの整合を直接見る。静止時は1〜5mmに
収まることを確認済み(2026-07-21頃)。走行中は経路依存で最大30mm超まで悪化する
区間があり(`icp_blind90/icp_from_truth.log`より、scan#60〜#140付近でdx/dy 10〜35mm)、
経路が壁近傍を通過する際の対応点密度・幾何配置に依存することを示している。

## 3. `wall_bias_diag`: 壁面残差バイアス診断

各壁セグメントごとに、ICP対応点の残差(bias)・標準偏差・傾き(tilt)を計測する。
`icp_blind90/wall_bias.log`(静止・走行混在)では、壁法線方向の残差は概ね
-4〜+5mm、標準偏差は23〜28mm程度で安定しており、特定の壁だけが大きく歪んでいる
様子はない(1件のみperim_+xでbias -13.3mmの外れ値を観測)。この程度のノイズ床が
壁マッチングの物理的な下限であり、10mm要件に対しては無視できない水準であることが
分かる。

## 4. `icp_seed_mode_diag`: シードモード比較(truth/offset/chained)

同一スキャン集合に対し、異なるシード(初期値)からICPを解いた場合の収束先を比較する。

```
icp_seed_mode (2パス化前, 55.0s記録, 22 segments, corr=0.2m):
  truth   n=1840  |bias|=2.18mm  RMSE=3.06mm  dyaw=+0.0377deg
  offset  n=1840  |bias|=2.18mm  RMSE=3.07mm  dyaw=+0.0384deg   (真値+15mmオフセットからでも収束先は同じ)
  chained n=1840  |bias|=6628.35mm  RMSE=6628.35mm  dyaw=+21.6deg  (前回解を逐次シードにする方式は暴走)
```

```
icp_seed_prod (本番と同一設定=huber0.025/10 iters/clamped、比較として追加):
  truth       |bias|=2.18mm   RMSE=3.06mm
  offset      |bias|=2.18mm   RMSE=3.07mm
  chained     |bias|=6628.35mm RMSE=6628.35mm
  truth_prod  |bias|=2.13mm   RMSE=3.07mm
  offset_prod |bias|=2.12mm   RMSE=3.07mm
```

### 結論

- **truth/offset/truth_prod/offset_prodはいずれも|bias|~2.1〜2.2mmで収束**し、
  シードの与え方(真値そのものか、真値+15mmオフセットか)による有意差はない。
  ICPが真値近傍で安定した停留点を持つことを示す。
- **chained(前回推定値を逐次シードにする方式)は完全に発散する**(|bias|6.6m級)。
  このシードモードは採用不可と結論した。
- production(本番、動作中)の|bias|が上記より大きい(14.7〜14.8mm、§1参照)のは、
  シード依存の問題ではなく、ノーツ除外前のマップ外物体混入や、走行中の対応点構成の
  違いなど別要因であることが、この比較によって切り分けられた。

## 5. この診断カテゴリ全体の位置づけ

真値使用版診断は「本番の測位精度そのものを測る」ためのものではなく、**本番の
測位バイアスがどの段階(マッチャ単体か、シードの与え方か、融合か)に起因するかを
切り分けるための補助輪**である。判定に使う数値は必ず
[`07_stage4_unity_production_summary.md`](07_stage4_unity_production_summary.md)
側のフルスタックrunを参照すること。
