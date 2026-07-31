# 実験履歴: 衝突ロバスト性の実装 (2026-07-31)

21番までで①(`imu_preintegration_node`のodom_fast外挿)が評価対象の誤差の
正体であること、その原因が衝突である可能性が高いことを確認した。本番でも
衝突(あるいは類似の外乱)は容易に起こりうるため、安全上この経路の頑健性は
必須要件としてユーザーから明示された。

## 前提: 衝突の実在確認

Unity側`CollisionRecorder`(既存、`Debug.Log`のみで未配信)に`/robot_collision`
トピック配信を追加(`UnityRosSensorPublisher.PublishCollision`)。MCP経由で
ライブ実行し確認したところ、実際に**11件の衝突**(Post_2、Shelf_0、
baffle_left_2、相対速度0.4〜1.4m/s)を検出した。

## 試行1: backend側でCombinedImuFactorのσを吊り上げる方式(失敗、削除済み)

生の加速度ノルムが閾値を超えたIMUサンプルについて、`preint_params_`の
accelerometer/gyroscopeCovarianceをその場で吊り上げてから積分する方式を
`backend_optimizer_node`に実装。検知自体は正しく動作した(22.3・125.9・
21.3 m/s^2で発火)が、**位置誤差への効果はほぼゼロ**だった。理由: ③
(因子グラフ融合)は壁補正が毎スキャン効くため元々問題が無く、そもそも
「効かせる先」が間違っていた。①(単純なIMU外挿)には因子グラフも共分散も
無いため、この実装は的外れだったと判断し、コードごと削除した。

## 試行2: ①で積分を止めて外挿値を保持する方式(失敗、逆効果)

問題は①だけにあるため、`imu_preintegration_node`側で衝突検知時に
`integrateMeasurement`とpublishを両方スキップし、直前の外挿値を凍結する
方式を実装。しかし**`/odom_fast`はUnity側のPDサーボが実際の閉ループ制御
フィードバックとして使っている**ため、値を凍結すると制御側が古い推定値との
誤差を蓄積させたまま急に復帰し、衝突が連続する区間(t=30.9〜34.1s、4件)で
かえって悪化した(最大5.27mm→18.37mm)。単純な「情報が無いから止める」は、
推定値が同時に物理挙動を駆動する閉ループの中では危険という教訓。

## 試行3: 生の加速度ノルムを閾値へ切り詰めて積分する方式(採用)

配信は止めず、積分に使う加速度だけ方向を保ったままノルムを閾値
(既定20.0 m/s^2)へ切り詰める。`imu_callback`内で生の加速度により検知し、
`imu_buffer_`へ積む値自体を切り詰め済みにすることで、③確定到着時の
再積分(`state_estimate_callback`)でも同じ切り詰め後の値が一貫して使われる。

検証(`mcp_collision_check_20260731/live_capture`、正しいbackend-param
IMU×30 + wall_prior_base_yaw_sigma=0.15構成、`--imu-param
imu_accel_anomaly_enabled:=true imu_accel_anomaly_threshold:=20.0`):

| 窓 | 緩和策なし | clip方式 |
|---|---:|---:|
| t=16.5s | 2.64mm | 2.57mm |
| t=26.1s | 1.44mm | 1.34mm |
| t=30.9s | 2.65mm | 2.83mm |
| **t=35.6s(最悪点)** | **5.27mm** | **4.39mm** |
| t=45.2s | 1.87mm | 1.87mm |
| t=49.9s | 2.80mm | 2.83mm |

hold方式のような閉ループ悪化は無く、最悪点で約17%改善(他はほぼ横ばい)。
劇的ではないが副作用なく安全側に効く。

## 検証時の環境メモ

このテスト中、システムメモリ逼迫(空き1GB程度、スワップ2GB近く使用)により
evaluatorの初回マッチングが環境要因で複数回フレーキーに失敗した
(`imu_accel_anomaly_enabled`の有無に関わらず発生することを確認済み、
コード起因ではない)。再現しない場合は再試行で問題ない。

## 追加: 無意味な補正器の削除・縮退検出の実装し直し (2026-07-31)

ユーザー指示により、これまでの調査で「発動しない/効果がない」と確認済みの
機構を削除し、1つだけ「本来重要だが正しく実装されていなかった」ものを
修正した。

### 削除したもの

1. **未使用3D ICPのCensi共分散コード**(`laser_scan_matching_node.cpp`):
   汎用3D ICP経路が組んでいたresult.wall.covariance等一式。Unityフィールド
   では必ず後続のplanarソルバーの値で上書きされ、一度も使われていなかった。
   壁マッチ品質判定(wall_valid・迷子モード用consecutive_invalid_results_)
   は、ノーツ/ボール抽出の点分類に間接的に影響するため残した。
2. **wall_dcs_scale(DCS風スケーリング)**(`backend_optimizer_node.cpp`):
   物理的にあり得ない位置への補正が来た際にσを連続的に緩める機構。全検証を
   通じて一度も発火しなかった(d_xyが常にmm規模)ため削除。
3. **imu_dynamics_\*(高ダイナミクス検知)**(同上): 加速度版・ジャイロ版とも
   既定無効。Opusレビューで方向が逆(壁を締めてIMUを相対的に硬いままにする)
   と指摘済みで、有効化すると悪化するリスクがあった。
4. **zupt_enabled_(ZUPT疑似観測)**(同上): 2026-07-28に試した閾値0.1m/s
   (効果薄)・0.5m/s(まだ動いているのに速度=0を強制し27.92mmスパイクを
   誘発)のいずれも不採用のまま既定無効で放置されていた。imu_dynamics_*と
   同じ基準(試したが効かない・既定無効のまま)でコードごと削除した。

### 修正したもの(削除ではなく実装し直し)

**x/y軸別対応点数・縮退フラグ**: 「消すべきではなく直すべき」というユーザー
指摘を受け、削除ではなく本来の設計(main.md ②-2)通りに実装し直した。
`solve_planar`/`solve_planar_ray_to_wall`の各対応点で使っている法線ベクトル
から、実際にX向き壁由来かY向き壁由来かを分類する`planar_count_x_facing`/
`planar_count_y_facing`を追加し、以前の`planar_inliers/2`という機械的な
折半・常時false固定だった縮退フラグを、実データに基づく判定へ置き換えた。

検証(`diag_wall_sigma_path`で確認): x_n=87〜249、y_n=227〜434と、以前は
常に同値だった二軸が明確に異なる実測値を示すようになった。このbagでは
縮退は一度も発火しなかった(両方向とも常時十分見えている)。回帰確認
(`cleanup_regression_check_20260731`)でt=15 max=11.73mm、既存の11.85mmと
同水準で悪化なし。

## 対応状態

- `backend_optimizer_node.hpp`/`.cpp`: 試行1のコードは削除済み(コミット無し)
- `imu_preintegration_node.hpp`/`.cpp`: 試行3(clip方式)を実装・ビルド済み。
  `imu_accel_anomaly_enabled`(既定false)、`imu_accel_anomaly_threshold`
  (既定20.0 m/s^2)
- Unity: `/robot_collision`配信を追加(`UnityRosSensorPublisher.cs`、
  `StartToBingoValidation.cs`のCollisionRecorder)、ライブUnityプロジェクトへ
  反映・コンパイル確認済み
- `bag_component_isolation.sh`: `--playback-duration`オプション追加、
  再生対象トピックを生センサ+真値のみに制限(推定器出力トピックとの衝突回避)
- `tools/consolidate_stage_log.py`: 位置/姿勢のみ記録し速度は解析時に数値
  微分する標準診断ツールを新規作成
- いずれも未コミット
- yamlへの`imu_accel_anomaly_enabled`反映(既定有効化)は未実施、ユーザー
  判断待ち
- `laser_scan_matching_node.cpp`: 未使用3D ICPのCensi共分散・縮退検出コード
  削除、x/y軸別対応点数の実分類実装(`planar_count_x_facing`/
  `planar_count_y_facing`)。ビルド・回帰確認済み(未コミット)
- `backend_optimizer_node.hpp`/`.cpp`: `wall_dcs_scale`・`imu_dynamics_*`
  (加速度版・ジャイロ版)を削除。`WallSigmaRecord`から関連フィールド削除。
  ビルド・回帰確認済み(未コミット)
- `config/robocon2026_unity.yaml`: `imu_dynamics_enabled`削除(未コミット)
