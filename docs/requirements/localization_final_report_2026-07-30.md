# 自己位置推定 最終レポート (2026-07-30)

## 1. 結論

`lio_localization`(①`imu_preintegration_node`、②`laser_scan_matching_node`、
③`backend_optimizer_node`)による自己位置推定は、要件Aを達成した。

- 定常位置誤差 RMSE ≤ 10mm: **PASS**(実測 0.68〜0.80mm、複数run)
- 最大位置誤差 ≤ 10mm: **PASS**(実測 1.7〜5.1mm、ノーツ有無・旋回動作の有無で変動)

要件の定義・変更履歴は
[requirements_spec_2026-07-27.md §0/§2](requirements_spec_2026-07-27.md)を正本とする。
本書はそこに追記した2026-07-30時点の判定の根拠を詳細にまとめたものであり、
判定そのものは要件定義書側が正本である。

## 2. システム構成

推定器の因子グラフ構造・データフロー・スレッド構成の詳細図は
[backend_estimator_architecture_2026-07-28.md](backend_estimator_architecture_2026-07-28.md)
(2026-07-30更新版)を参照。要点だけ述べる。

- **①imu_preintegration_node**: `/state_estimate`(③の40Hz出力)を基準にIMUを
  1kHzへ外挿し`/odom_fast`を配信する。評価器・ロボット制御が実際に使う出力。
- **②laser_scan_matching_node**: LiDARスキャンをplanar 2D point-to-segment ICPで
  既知フィールド壁マップに整合し、`ScanMatchResult`(壁補正+円柱観測)を配信する。
  2026-07-30に専用`icp_worker_`スレッドを追加し、`odom_callback`/`imu_callback`
  (姿勢履歴の追記のみ)をICP本体から分離した。
- **③backend_optimizer_node**: IMU因子(`CombinedImuFactor`)・壁PriorFactor・
  円柱`BearingRangeFactor`をISAM2(fixed-lag smoother, lag=0.5s)で最適化し、
  `/state_estimate`を40Hzで配信する。専用`optimizer_worker_`スレッドとlock-free
  SPSC IMUリングを持つ(2026-07-29導入)。

## 3. 要件A達成までの主要な経緯

年代順。詳細runと数値は各experiment_historyを参照。

| 時期 | 内容 | 結果 | 出典 |
|---|---|---|---|
| 〜2026-07-27 | Unity統合(Stage 4)。定常RMSEは早期に達成したが最大位置誤差が長期未達(最良10.79mm、最終21.92mm) | 定常RMSE PASS / 最大 FAIL | `06`, `07` |
| 2026-07-28 | 減速時位置誤差スパイク(12〜16mm)の調査。バイアス・センサノイズ・smoother_lag・relinearizeThreshold・生IMU値を個別に切り分け、いずれも主要因ではないと判明。ICP側の幾何学的性質と、平面ICPの軸別対応点数均等割り当て/固定共分散値が優先候補として残った | 要因の絞り込み(未解決) | `08` |
| 2026-07-29 | ③にIMU専用executor・lock-free SPSC IMUリング・専用optimizer workerを導入 | パフォーマンス改善(スレッド競合の解消) | `09` |
| 2026-07-29 | callback遅延の要素分離調査 | 診断手法の確立 | `10` |
| 2026-07-29 | 最大10mm未達の根本原因調査。**notes除外がscan-index隣接性だけで判定されており、実空間で離れた壁ノイズ点まで同一クラスタに誤結合していたバグ**を特定・修正(`ball_exclusion_max_adjacent_gap`導入) | 根本原因特定・修正 | `11` |
| 2026-07-29 | センサノイズのデータシート照合。UTM-30LXの±30mm精度公差を誤ってGaussian σとして使っていた設定を、データシート記載の repeatability σ<10mm へ修正 | RMSE 0.65mm/max 2.18mm PASS達成 | `12` |
| 2026-07-30 | Unity直結(ライブ)実行時のみbag再生より最大誤差が悪化する現象を調査。②が単一executorスレッドでICPとodom/imu受信を両方処理しており、ICP実行中は姿勢履歴の更新自体が止まっていたことが原因と特定 | 根本原因特定(未修正) | `13` |
| 2026-07-30 | ②に専用`icp_worker_`スレッドを追加(③と同型のexecutor/worker分離)。シャットダウン時にsim時刻凍結でjoinがハングするバグを修正 | 最大位置誤差 17.55mm→4〜5mm台 | `14` |
| 2026-07-30 | 残る数mm級スパイクの原因調査。ログ相関では配送遅延が疑わしく見えたが、bag再生で転送ジッタを排除しても同じスパイクが再現し**この仮説を否定**。ノーツ接近時刻との相関を発見し、ノーツ無効化A/Bテストで確定 | 最大位置誤差 4〜5mm台→1.75mm | `15` |

## 4. 現在の残差の性質

要件A達成後になお残る1.7〜5mm級の残差は、単一の原因ではなく複数の要因が重なっている。

### 4.1 ノーツ(ボール)除外の不完全性 — 支配的要因

ロボットがノーツに接近すると、平面ICPのノーツ除外ロジックが点群を完全には除外
しきれず、除外し損なった点がわずかに壁とみなされて推定値を数mm引っ張る。
ノーツを無効化したA/Bテストで最大位置誤差が4〜5mm台から1.75mmまで低下し、これが
支配的要因であることを確認した(`15`)。競技本番ではノーツが存在するため、この
残差は実運用でも同程度発生し得る。

### 4.2 旋回ダイナミクスに起因する残差(ノーツ非依存)

ノーツ無効化後もなお残る1.7mm級のピークは、Unity記録の実角速度データで
急旋回(0→約1.5rad/s)と時刻が一致することを確認した(`15`)。壁補正の共分散を
角速度トリガーで締める対策を実装・検証したが、有意な改善は得られなかった
(1.7346mm→1.7333mm)。スキャン周期(40Hz=25ms)に内在する「補正が届くまでの
待ち時間」が律速している可能性が高いが、これは推論であり直接検証はしていない。

### 4.3 明確に否定された仮説

- **Unity↔ROS間の転送・スケジューリング遅延**: bag再生で転送ジッタ(callback_gap_max
  502ms→6.9ms、gaps>10ms 1276件→0件)を実質排除しても同じ時刻・同じ大きさの
  スパイクが再現されたため、否定された(`15`)。
- **壁補正の共分散を締める(線形加速度トリガー版)**: 通常の巡航減速でも頻発し
  選択性がないため2026-07-28時点で既に無効化されていた(`08`)。角速度トリガー版
  も上記4.2の通り効果なし。

## 5. 検証方法論

- **複数run再現**: 単一runでの判断を避け、同一条件で最低2〜3回の再現を確認する
  (`unity_direct_scan_worker_fix_20260730_r1/r2/r3`等)。
- **bag再生による要因分離**: `UnityRosSensorPublisher`がPlay開始ごとにROS-TCP送信
  直前のセンサ値をCSVへ記録する仕組み(`docs/setup/UNITY_SENSOR_CSV_ROSBAG_GUIDE.md`)
  を使い、同一物理走行をUnity↔ROSの実転送を経由せずROS単体で再生できる
  (`bag_component_isolation.sh`)。転送・スケジューリング要因とROS側推定器要因を
  切り分ける主要な手法として今回確立した。
- **ノーツ有無のA/Bテスト**: `RoboconFieldBuilder.DisableNotesExperimentFlag`
  (Unityプロジェクト直下にフラグファイルを置くと全ノーツGameObjectが非アクティブ化)
  を使い、同一経路でノーツの影響だけを分離した。

## 6. 既知の制約・今後の課題

- 現行のUnity試験構成(`robocon2026_unity.yaml`)には円柱(ビンゴポスト)が未登録で
  あり、壁補正が連続拒否された際の円柱ベース再ローカライズ経路は機能しない
  (`backend_estimator_architecture_2026-07-28.md`参照)。
- 平面ICPの軸別対応点数(X/Y)は実際の視認性を反映しない機械的な折半、共分散値は
  固定値`1e-5`であり、実測ではない。全体対応点数Nに基づくyaw方向のσ計算は健全。
- 経路計画・走行制御(`robocon2026_autonomy_plan.md`§4.4で構想されている時間最適
  軌道生成・MPC)は未実装。現行の`StartToBingoValidation.cs`は区間ごとの
  quintic minimum-jerkプロファイルのみで、旋回時の角速度上限を陽に制約していない。
  4.2節の残差を避ける・避けないに関わらず要件Aは満たしているため、これは
  「精度未達の穴埋め」ではなく速度・完走性の最適化課題として別途扱う。
- スキャン周波数(40Hz)を上げれば4.2節の残差が縮むという仮説は、実機LiDARの
  仕様上限に制約される可能性があり、直接検証していない。

## 7. 証跡

- `docs/stage4_runs/unity_direct_scan_worker_fix_20260730_r1/r2/r3/`
- `docs/stage4_runs/bag_replay_r3_compare_20260730/`
- `docs/stage4_runs/unity_direct_notes_disabled_20260730_r1/`
- `docs/stage4_runs/gyro_dynamics_baseline_20260730/`,
  `docs/stage4_runs/gyro_dynamics_enabled_20260730/`
- `docs/experiment_history/09` 〜 `15`
- `/mnt/c/Users/kouza/UnityProjects/Robocon2026Sim/SensorRecordings/`
  (`20260730_073915_684_2abade46`: notesあり, `20260730_083653_484_f5362162`: notes無効化)
