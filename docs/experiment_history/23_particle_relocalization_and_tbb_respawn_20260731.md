# 実験履歴: パーティクル再収束の実装とGTSAM/TBBクラッシュの発見・回避 (2026-07-31)

22番までで①(`imu_preintegration_node`)の衝突ロバスト性を確保した。本節は
その後にユーザーから出た「収束半径が狭い(並進約12cm・回転数度)ため、
それより大きくロストした場合の保険が欲しい」という要望に対応した記録。

## 前提調査: 円柱ベース再定位が機能しない理由

`backend_optimizer_node`には既に円柱BearingRangeFactorによる非対称
ランドマーク判別・再定位の設計(`registered_cylinders_`、
`cylinder_hypothesis_error`)があったが、Unity側`RoboconFieldBuilder.cs`を
確認したところ`CreateBox`(角柱)しか呼んでおらず、**円柱状の物体は
フィールド上に一切存在しない**ことが判明した。"Post_i"/"Shelf_i"という
衝突オブジェクトもビンゴ棚1構造物の角柱フレーム部材であり、散在する
ランドマークではない。

さらに、10x8mの長方形フィールドが壁だけで厳密に180°回転対称かをグリッド
探索で検証したところ、**厳密な回転対称点は存在しなかった**が、実際の
スキャンマッチングでは真の姿勢と180°反転姿勢とで対応点の重なりが80%以上
一致するサンプルが大半で、1サンプルでは反転側の方がスコアが高くなる
逆転すら確認された。つまり理論上は非対称でも、壁のみのマッチングでは
実質的に判別不能。円柱ベース設計はこのフィールドでは機能しない。

これによりユーザーから「パーティクルフィルタもどきを使って再収束できる
ようにしてみてください。円柱とかいう存在は忘れてください」との指示。

## 実装: MCLもどきの全域再収束

`laser_scan_matching_node`に`step_particle_relocalization()`を実装。

- `enable_particle_relocalization_`(既定false)、`particle_count_`(既定
  1000)、`particle_converge_xy_std_`/`particle_converge_yaw_std_`(既定
  0.05)、`particle_motion_noise_xy_`/`_yaw_`(既定0.02)、
  `particle_likelihood_sigma_`(既定0.05)をパラメータ化
- `consecutive_planar_invalid_ >= kYawLostModeThreshold`で起動(旧
  ±πヨーロスト検知`enable_yaw_lost_mode_`を置き換え)
- 起動時にフィールド全体へ一様ランダム散布、以後は①のオドメトリ差分
  (デルタ、body frame変換)で運動モデル伝播+ノイズ、壁地図への最近傍距離
  から尤度計算、systematic resampling、標準偏差が閾値未満で収束と判定
- 収束スキャンだけ`ScanMatchResult.wall.relocalization_event=true`

同時に、これまでの調査で無力と判明していた2機構を削除:
`enable_lost_mode_`(汎用3D ICP専用で実際には未使用の`result.wall`しか
書き換えず再収束に無力)、`solve_planar_coarse_to_fine`(粗探索、最大
ゲート拡大4倍≈0.48mしかなくkidnap規模の再捕捉には無力)。

初回の単独動作確認(意図的に誤らせた初期姿勢でのテスト)では、パーティクル
群が4回活動し、収束姿勢自体は真の開始位置(-2.419, 1.354)に極めて近い
値(例: (-2.384, 1.348))を複数回出しており、**局所的な収束アルゴリズム
自体は正しく機能している**ことを確認した。

## 発覚した問題1: backendのゲートに拒否され続ける

backendの`wall_gate_xy_`(1.0m)/`wall_gate_yaw_`(1.2rad)はπ反転ロック対策
用で、数十cm〜1m規模の補正しか想定していない。パーティクル再収束による
数m規模の意図的なジャンプはこのゲートで毎回拒否され、②が収束→backendに
無視される→②が再び迷子、のループに陥っていた。

**対策**: `WallCorrection.msg`に`bool relocalization_event`を追加、
収束直後の1回だけtrueにして、backend側で`d_xy`/`d_yaw`判定より先に
無条件バイパスして採用するよう変更。

## 発覚した問題2: GTSAM/TBB内部クラッシュ(本番)

バイパス実装後の再テストで、backendが収束姿勢を受理した**直後の次スキャン
から**`exception in smoother update: Index out of requested size range`が
発生し、既存の発散検知セーフティネットが誤発火→巻き戻し→再発散を
0.5秒間隔(`kResetCooldownSec`)で無限に繰り返す壊滅的リグレッションが
判明した(60秒のテストで700回以上)。

### 原因調査

1. **キー衝突ではない**: `next_index_`で使い捨てインデックス管理済み、
   `ValuesKeyAlreadyExists`の類ではない
2. **起動時の`initialize_graph()`自体は正常**: 同日のベースライン回帰
   テスト(パーティクル無効・正しい初期姿勢)ではこの例外は0件
3. **例外の型を`typeid`で特定**: `St12out_of_range`=`std::out_of_range`。
   `graph_size=1, values_size=3`(=CombinedImuFactor 1本+X/V/B 3変数)と
   起動直後と全く同じ形。単純なスレッド競合でもなさそう(スキャン処理は
   `optimizer_worker_`という単一専用スレッドで直列化されている)
4. **文字列一致でTBB由来と特定**: "Index out of requested size range"を
   `libtbb.so.12.17`の`strings`出力から発見(GTSAM/gtsamはビルド時に
   `libtbb.so.12`にリンクされている、`ldd`で確認)。GTSAMのISAM2/
   IncrementalFixedLagSmootherは並列消去にTBBの並行コンテナを使う
5. **仮説1(オブジェクト差し替えが原因)を検証・反証**: `smoother_ =
   make_unique<...>()`での丸ごと差し替えをやめ、既存smootherへ新アンカーを
   追加するだけの実装に変更して再テストしたが、**全く同じ場所(同じ
   graph_size/values_size)で全く同じ例外が再現**。原因はオブジェクト
   差し替えではないと判明
6. **仮説2(大量の孤立変数の一括周辺化)**: 起動直後は履歴ゼロで再現せず、
   走行中の呼び出しでは蓄積済みの数十〜100個規模の過去ノード(新アンカー
   とは非連結)を1回のupdate()でまとめて周辺化しようとしており、これが
   GTSAM/TBB内部の限界・未知のバグを踏んでいる可能性が高い(未確定、
   ソース非公開のTBBコンパイル済みライブラリの内部でこれ以上の追跡には
   gdb等が必要だが本環境には無い)

## 対策: プロセス再起動方式への統一

GTSAM/TBB内部を直接修正することはできないため、「バグる操作(走行中の
in-process再構築)自体をさせない」方向で回避した。

- `request_process_respawn()`: 姿勢・速度・バイアス・事前分布σを
  `relocalization_handoff_path_`(既定`/tmp/lio_localization_backend_
  relocalization_handoff.txt`、テストスクリプトは実行ごとに一意な
  `$OUT/relocalization_handoff.txt`を明示指定)へ書き出し、
  `relocalization_respawn_requested_`を立てて`rclcpp::shutdown()`
- `main()`はノード破棄前にこのフラグを読み、専用終了コード42を返す
  (通常の異常終了と区別するため)
- 起動時(コンストラクタ)は`try_consume_relocalization_handoff()`で
  ハンドオフファイルの有無を確認、あれば読み取って削除し
  `initial_x_`等を上書きしてから`initialize_graph()`を呼ぶ
  (=履歴ゼロの「安全な」経路だけを常に通る)
- `*.launch.py`(4ファイル)の`backend_optimizer_node`に
  `respawn=True, respawn_delay=0.2`を追加(実運用)
- `bag_component_isolation.sh`に`start_node_respawn()`を追加。終了コード
  42のときだけ即座に再起動し、それ以外(本物のクラッシュ)は通常通り
  終了させてテストの異常終了を隠さない(テスト運用)

この方式を、走行中に`initialize_graph()`を直接呼んでいた3経路すべて
(発散検知リセット・円柱ベース再定位・パーティクル再定位)に適用した。

## 検証結果

`mcp_collision_check_20260731/live_capture`bag(65秒、必須トピック
`/clock /imu/data /scan /ground_truth_pose /unity_sensor_publish_counts`を
含む実キャプチャ)、意図的に誤らせた初期姿勢(1.0, -1.0, 1.57rad)で②③を
起動。

| 指標 | 対策前 | 対策後 |
|---|---:|---:|
| `exception in smoother update` | 700回以上/60秒 | **0回** |
| 収束→backend受理→安定動作 | 4回収束するも位置誤差1300〜4000mmで高止まり | 1回で復帰、以後例外ゼロで安定 |
| 収束姿勢 vs 真の開始位置 | - | (-2.42, 1.35, 0.00) vs (-2.419, 1.354, 0.000)、ほぼ完全一致 |
| 受理ログ→復帰までの時間 | - | 実測約0.48秒 |

さらに、パーティクル機能を無効化した通常経路(正しい初期姿勢)での回帰
テストでも例外・respawnともに0件を確認し、既定動作に副作用が無いことを
検証した。

### テスト運用上の教訓

- **元のテストbag(`/tmp/robocon_angvel_fix_gyrosign_20260730.mcap`)が
  セッションクラッシュ後に`/tmp`ごと消失していた**。Unity側
  `SensorRecordings/<runId>/`にCSVとして永続化されてはいたが、
  `manifest.json`にタグが無く該当runを特定できず、CSV→bag変換ツールも
  存在しないため、別の実キャプチャbagで代替した。重要なbagは`/tmp`では
  なく`docs/stage4_runs/`配下に保存すべき、という教訓を得た
- **`bag_component_isolation.sh`はROS_DOMAIN_ID等の分離をしていない**ため、
  2つのテストを並列実行すると両方のノード・トピックが衝突・混線する。
  必ず1回ずつ順番に実行し、完了を待ってから次を開始する必要がある
  (並列実行を試みて実際に汚染を確認、破棄して再実行した)

## 対応状態

- `lio_localization/msg/WallCorrection.msg`: `bool relocalization_event`追加
- `laser_scan_matching_node.hpp`/`.cpp`: `step_particle_relocalization()`
  実装、`enable_lost_mode_`/`solve_planar_coarse_to_fine`削除
- `backend_optimizer_node.hpp`/`.cpp`: `request_process_respawn()`実装、
  発散リセット・円柱再定位・パーティクル再定位の3経路すべてをこれに統一、
  `initialize_graph()`のsmoother_再生成を`!smoother_ || smoother_poisoned_`
  条件へ変更(結果的にTBBバグには効かなかったが、無駄な再生成を避ける
  最適化としては残置)
- `lio_localization/launch/lio_localization.launch.py`、
  `lio_localization_sim/launch/{unity_sensor_localization,gazebo_stage4,
  sim_test}.launch.py`: `backend_optimizer_node`に`respawn=True`追加
- `lio_localization_sim/scripts/bag_component_isolation.sh`:
  `start_node_respawn()`追加、backend起動をこれに切り替え、
  `relocalization_handoff_path`を実行ごとに一意化
- ビルド・検証済み、いずれも未コミット
- 発散検知リセット経路(①②③④)は理論上は同じ修正が必要だったため今回
  一緒に直したが、実発火は本日時点で一度も無く未検証(受理直後の
  respawn自体は同一コードパスなので機能するはずだが、実際のトリガー
  条件下での確認はできていない)
