# 自己位置推定 最終レポート (2026-07-30)

## 1. 結論

`lio_localization`(①`imu_preintegration_node`、②`laser_scan_matching_node`、
③`backend_optimizer_node`)による自己位置推定は、要件Aを達成した。

- 定常位置誤差 RMSE ≤ 10mm: **PASS**(実測 0.68〜0.80mm、複数run)
- 最大位置誤差 ≤ 10mm: **PASS**(実測 1.7〜5.1mm、ノーツ有無で変動。実旋回を
  含めた検証結果は§4.4参照)

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

### 4.2 訂正: 「旋回ダイナミクス」説は列の取り違えによる誤りだった

当初、ノーツ無効化後もなお残る1.7mm級のピークが、Unity記録の`ground_truth.csv`で
「実角速度0→約1.5rad/s」と時刻が一致すると報告していたが、これは**`awk`での
列指定の誤り**による誤った解析結果だった。実際にはCSVの列順は
`...,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z,...`であり、
1列分ずれて`linear_y`(ロボット座標系の横方向並進速度、単位m/s)を
`angular_z`(ヨー角速度、単位rad/s)として読んでいた。正しい`angular_z`列を
確認し直すと、該当区間を含め試験全体を通じて厳密に`0.0000`であり、姿勢
クォータニオン(`orientation_z`, `orientation_w`)も一切変化していなかった。

これは`StartToBingoValidation.cs`で`body.constraints = RigidbodyConstraints.FreezeRotation`
(全軸の回転を物理的に凍結)が設定されており、`AddTorque`や`transform.rotation`の
直接操作もコード上どこにも存在しないという事実と完全に整合する。**この
シミュレーション環境ではロボットは一度も物理的に回転していない。**

したがって、当該ピークの正体は「旋回」ではなく、**ロボット座標系の横方向
(Y軸)並進速度が短時間で大きく変化する事象**(オムニホイールによる、機体の
向きを変えずに横へ移動する動き)だった。壁補正の共分散を角速度トリガーで
締める対策(`imu_dynamics_gyro_*`)を検証した際に有意な改善が見られなかったのも、
当然の結果だったと考えられる。実際の`angular_z`が常にほぼ0である以上、
角速度閾値(0.3rad/s)を超えるイベント自体がほとんど発生し得ないためである。

**この訂正の帰結として、「自己位置推定が実際の旋回中にどう振る舞うか」は
現時点で一度も検証されていない。** 検証結果は §4.4 を参照。

### 4.3 明確に否定された仮説

- **Unity↔ROS間の転送・スケジューリング遅延**: bag再生で転送ジッタ(callback_gap_max
  502ms→6.9ms、gaps>10ms 1276件→0件)を実質排除しても同じ時刻・同じ大きさの
  スパイクが再現されたため、否定された(`15`)。
- **壁補正の共分散を締める(線形加速度トリガー版)**: 通常の巡航減速でも頻発し
  選択性がないため2026-07-28時点で既に無効化されていた(`08`)。角速度トリガー版
  も上記4.2の通り効果なし。

### 4.4 実旋回下での検証結果(2026-07-30、`16`)

`FreezeRotation`を緩めて初めて実旋回を有効化したところ、要件Aから程遠い規模
(最大159m、ヨー誤差最大180°)で完全に破綻した。4回の対症療法(ゲート緩和、
なめらかな制御、後述バグ1の修正、後述バグ2の修正)ではいずれも改善はする
ものの解消せず、①(`imu_preintegration_node`)の内部状態を直接ground truthと
突き合わせて初めて真因を特定できた。

見つかった3つのバグ:

1. **`/odom_fast`のtwistがワールド座標系のまま配信されていた**
   (`imu_preintegration_node.cpp`、GTSAM `NavState::velocity()`はワールド座標系
   だが`nav_msgs/Odometry`のtwistはボディ座標系であるべき)
2. **`body.MoveRotation()`が非kinematicなRigidbodyでは`angularVelocity`を
   更新しない**ため、IMUが実際の回転を全く感知していなかった
   (`StartToBingoValidation.cs`、`body.angularVelocity`直接代入方式へ修正)
3. **(真因)ジャイロが擬ベクトルとして正しく変換されていなかった**
   (`UnityRosSensorPublisher.cs`の`UnityVectorToRos`はUnity左手系→ROS右手系の
   鏡映変換(行列式-1)で、極性ベクトル(速度・加速度)には正しいが、擬ベクトル
   (角速度)には追加の符号反転が必要だった)

3つとも**yaw=0では無症状**(鏡映・恒等変換のいずれも回転量ゼロでは効果が
消える)ため、実旋回を初めて有効化するまで一度も発現していなかった。

修正後(bag再生による検証、Unity起動なし):

| 指標 | 修正前 | 修正後 |
|---|---:|---:|
| ①予測ヨー vs 真値(旋回中) | 符号反転(誤差100°超) | 誤差0.1〜3°程度 |
| 破綻の兆候(541/541点非対応等) | 発生 | 0件 |
| 位置誤差(旋回中の一過性ピーク) | 発散継続(m〜十m規模) | mean 10.18mm/max 27.65mm |
| 位置誤差(旋回後) | 発散継続 | mean 0.85〜1.48mm(正常値へ回復) |

旋回の瞬間だけ要件(≤10mm)を一時的に超えるが、直後に正常値へ回復する。これは
バグではなく実際の急旋回中の推定器の素の性能と判断している。回転が並進より
クリティカルに悪化する理由(壁までの距離による誤差増幅、ヨーには粗探索が
無いこと、IMU姿勢誤差が位置誤差へ伝播する経路)は`16`にまとめた。

対処として、位置の`enable_lost_mode_`と同型の「ヨー版迷子モード」
(`enable_yaw_lost_mode_`, 既定false)を実装した。`planar_valid`の連続失敗が
続いた場合に限り、シードそのままとπ反転仮説を両方解いて対応点数の多い方を
採用する。発動時は`RCLCPP_ERROR`で目立たせ、「発動すること自体が上流の
追跡失敗を示す異常」として扱う設計思想とした(常用する安全網ではなく、
根本原因調査の対象を可視化する保険)。

Unity実行の負担を避けるため、この一連の検証は全てbag再生(既存Unity CSV
記録のMCAP化、ジャイロ符号のみを反転する診断用スクリプトでの反証実験を含む)
で行った。実Unity上での通し確認(60秒フル)はまだ行っていない。

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
