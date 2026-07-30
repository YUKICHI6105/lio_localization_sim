# プロジェクト総括レポート (2026-07-30時点)

千葉大学ロボットコンテスト2026向け自律走行システムの、2026-07-30時点での
到達状況をまとめる。個別の技術詳細は各リンク先を参照。目標・フィールド仕様・
全体構想は[robocon2026_autonomy_plan.md](robocon2026_autonomy_plan.md)を正本とする。

## 1. 全体目標(再掲)

競技規則PDFを正本とし、ロボットが認識・判断・走行・ノーツ回収・ビンゴ投入を
完全自律で実行する。最適化対象はコンボ完成後にVゴールを宣言できるまでの時刻。
ただしこのシステムは今回の競技専用ではなく、汎用的な「ロボコン向け自己位置推定」
を目指す設計としている。

## 2. 開発工程(autonomy_plan §5)の現在地

| # | ゲート | 合格条件 | 状況 |
|---|---|---|---|
| 1 | フィールド検証 | 図面とSDF照合、全寸法誤差5mm以内 | 完了(既存) |
| 2 | 三輪オムニ基盤 | 指令追従誤差5%、壁無接触で完走 | 完了(既存、`StartToBingoValidation.cs`) |
| 3 | 競技用既知マップ(自己位置推定) | 走行中RMSE 10mm以下・最大位置誤差10mm以下 | **達成**(2026-07-30。詳細は§3) |
| 4 | 機構物理モデル | 回収・保持・投入の成功率95%以上 | 未着手 |
| 5 | ルール状態機械 | 2球制約・投入順の遵守 | 未着手 |
| 6 | 固定戦略完走 | 既知位置3球で縦コンボ10回連続達成 | 未着手 |
| 7 | 時間最適化 | 軌道・加減速・機構並行動作の最適化 | 未着手(下記§4参照) |
| 8 | 自己判断 | 再計画90%以上完走 | 未着手 |
| 9 | ドメインランダム化 | 摩擦・質量・遅延・ノイズ振り100試合95%以上 | 未着手 |
| 10 | 実機移行 | Hardware-in-the-loop、段階的速度解放 | 未着手 |

ゲート3(自己位置推定)が今回のセッションで長期未達から達成に移行した、
最も進展があった領域である。ゲート4以降(回収・投入機構、競技戦略、経路計画・
MPC)は`robocon2026_autonomy_plan.md`に構想はあるが実装は未着手。

## 3. 自己位置推定(ゲート3)の達成状況

詳細は[localization_final_report_2026-07-30.md](requirements/localization_final_report_2026-07-30.md)
を参照。要点:

- 定常RMSE 0.68〜0.80mm・最大位置誤差1.7〜5.1mmで要件A(共に≤10mm)をPASS。
- 主要な修正: ノーツ除外クラスタリングのバグ修正、センサノイズのデータシート
  照合、②(`laser_scan_matching_node`)へのICP専用worker分離(③は2026-07-29に
  対応済み)。
- 残る1.7〜5mm級の残差はノーツ接近時の除外不完全性(支配的)と旋回ダイナミクスに
  起因すると特定済みだが、要件には十分な余裕があるため追加対応は不要と判断。
- 推定器のアーキテクチャ図・スレッド構成は
  [backend_estimator_architecture_2026-07-28.md](requirements/backend_estimator_architecture_2026-07-28.md)
  (2026-07-30更新版)を参照。

## 4. 経路計画・走行制御の現状

現行の走行制御は`StartToBingoValidation.cs`が実装する、区間(waypoint)ごとの
quintic minimum-jerkプロファイル + Cartesian PD servoである。`MaxSpeed`/
`MaxAcceleration`(`RobotDefinition.TargetLimits`)による並進速度・加速度の制約は
持つが、**旋回角速度・角加速度の上限を陽に制約する仕組みは無い**。waypoint間の
方向転換時に生じる急な角速度変化が、§3で特定した残差要因の一つ(旋回ダイナミクス)
の発生源になっている。

`robocon2026_autonomy_plan.md`§4.4が構想する「オムニ対応の時間最適軌道生成
(速度5m/s、加速度4.905m/s²、角速度2π rad/s以下)」「jerk・モータ飽和・横滑りを
含むMPC」は未実装。実装する場合、角速度上限を明示的に扱うことで、走行速度と
自己位置推定の残差の両方を制御できる可能性がある(未検証の仮説)。

過去の関連調査:

- [06_localization_10mm_validation_report_2026-07-27.md](experiment_history/06_localization_10mm_validation_report_2026-07-27.md)
  §5.5「衝突予測と実際の接触」: 直線制動距離ベースのPONR(Point of No Return)
  予測が、横移動できるオムニ機体の壁際旋回で誤発火することを確認し、常時無効化は
  不採用とした。予測器を修正する場合は計画軌道・横加速度限界・機体外形を使って
  「回避可能」と「接触不可避」を区別する必要があると記録されている。
- [start_to_bingo_baseline_and_algorithm_survey.md](start_to_bingo_baseline_and_algorithm_survey.md)
  にアルゴリズム候補の調査記録がある(内容は同ファイル参照)。

## 5. 次の候補

ユーザー判断待ちの分岐点として記録する(このセッションでは着手しない):

1. 経路計画・MPCの実装に着手するか、それとも回収・投入機構(ゲート4)を先に
   進めるか。
2. 旋回角速度を制約する走行制御(§4)を、自己位置推定の残差対策としてではなく
   速度・完走性最適化の一環として設計するか。
3. N5095/rosbag実機再生検証(以前より「Unityで経路まで詰め切ったら実行」として
   保留中)を開始するタイミング。

## 6. 主要ドキュメント索引

- 要件定義: [requirements/requirements_spec_2026-07-27.md](requirements/requirements_spec_2026-07-27.md)
- 自己位置推定 最終レポート: [requirements/localization_final_report_2026-07-30.md](requirements/localization_final_report_2026-07-30.md)
- 推定器アーキテクチャ図: [requirements/backend_estimator_architecture_2026-07-28.md](requirements/backend_estimator_architecture_2026-07-28.md)
- 全体構想: [robocon2026_autonomy_plan.md](robocon2026_autonomy_plan.md)
- 環境復元・実行手順: [setup/RESTORE.md](setup/RESTORE.md), [setup/UNITY_MCP_BENCHMARK_GUIDE.md](setup/UNITY_MCP_BENCHMARK_GUIDE.md)
- 実験履歴一覧: [README.md](README.md)
