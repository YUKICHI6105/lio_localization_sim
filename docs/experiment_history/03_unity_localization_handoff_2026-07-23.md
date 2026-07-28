# Unity 自己位置推定 パラメータ調整 引継ぎレポート

作成日: 2026-07-23  
対象: 千葉大学ロボットコンテスト2026 Unity物理シミュレーション  
達成条件: 位置RMSE 10 mm以下（未達）

## 1. 現在の状態

- Unity EditorとのMCP接続は正常。
- UnityからWSL上のROS TCP Endpointへ `127.0.0.1:10000` で接続できる。
- `/clock`、`/imu/data`、`/scan`、`/ground_truth_pose` はUnity物理側で生成する。
- `/ground_truth_pose` は評価専用で、自己位置推定入力には使用していない。
- 車輪エンコーダ相当オドメトリ案は、オムニ車輪のスリップに弱いため削除済み。
- 評価終了時にlaunch全体を終了する処理を追加済み。以前の無限稼働は解消。
- Unity Play Modeは本レポート作成前に停止済み。

## 2. 固定したセンサモデル

### LiDAR

北陽電機 UTM-30LX相当:

- 40 Hz、25 ms/scan
- 270度
- 1081点、0.25度分解能
- 0.1–30 m
- 10 m以内の測距誤差をプロジェクト既定モデルのガウスノイズ
  `sigma=0.030 m` としてUnityへ実装

公式仕様: https://www.hokuyo-aut.jp/search/single.php?serial=169

### IMU

実機型番はリポジトリ内に記載がない。現時点では `main.md` の暫定モデル:

- 1 kHz（Unityの `Time.fixedDeltaTime=0.001 s`）
- 加速度ノイズ `sigma=0.01 m/s^2`
- 角速度ノイズ `sigma=0.001 rad/s`
- 注入固定バイアス:
  - accel x = +0.05 m/s^2
  - accel y = -0.03 m/s^2
  - gyro z = +0.005 rad/s

実機IMU型番決定後、データシート値および静止実測Allan varianceへ置換すること。

## 3. 維持する自己位置推定アルゴリズム

ユーザー指示によりアルゴリズムは変更しない。

1. IMU Preintegrationによる高速予測 `/odom_fast`
2. 既知フィールド有限壁面への2D point-to-segment ICP
3. LiDAR到着駆動のGTSAM固定ラグバックエンド
4. 壁PriorFactor、Huber/DCS、物理ゲート
5. Unity真値との時刻補間評価

平面ICPでソース固定だった採否値は計算式を変えずROSパラメータ化した:

- `planar_max_iterations`
- `planar_min_inliers`
- `planar_correspondence_distance`
- `planar_huber_delta`
- `planar_max_mean_residual`
- `planar_max_shift`
- `planar_max_yaw_shift`

バックエンドには、静止校正結果を初期値として渡せるパラメータを追加した:

- `initial_bias_accel_{x,y,z}`
- `initial_bias_gyro_{x,y,z}`

現在の基準設定では全て0へ戻してある。既知バイアスを直接与えた最終試験が悪化したため、
符号・座標系を確認せず実機値を設定してはならない。

## 4. 実機相当センサーでの試験結果

| 試験 | 主な設定 | RMSE | 最大誤差 | 結果 |
|---|---|---:|---:|---|
| 第1組 | 対応0.20 m、残差0.09 m、内点率0.40、IMUバイアス初期値0 | 3990.65 mm | 9329.27 mm | FAIL（3組中最良） |
| 第2組 | 対応0.30 m、残差0.15 m、内点率0.25、bias RW拡大 | 29004.04 mm | 52264.73 mm | FAIL、誤対応受理 |
| 第3組（最終） | 第1組付近＋注入バイアスを既知初期値として設定 | 12844.10 mm | 21315.66 mm | FAIL |

参考: 理想センサーに近い旧条件では5秒時点で平均1.31 mm、最大4.09 mmだったが、
LiDAR補正が継続せず後半にIMUドリフトした。実機相当化後は問題が早く顕在化した。

最終試験ログ:

- `/home/yukichi6105/.ros/log/2026-07-23-12-05-41-835084-fuku-pc-327529`
- `/tmp/sim_eval_report.txt`
- `/tmp/sim_eval_plot.png`

## 5. 分かったこと

1. 採否条件を広げれば改善する問題ではない。第2組では対称壁への誤対応を受理し、
   位置・ヨーとも大きく発散した。
2. 厳格すぎると壁補正が途切れ、IMUデッドレコニングに移行して数m単位でドリフトする。
3. 最終試験で正確な注入バイアスをGTSAM初期値へ渡しても悪化した。主因は単純な
   「バイアス学習が遅い」ではない。
4. ログでは `LiDAR wall correction unavailable`、
   `wall match rejected by quality check`、
   `IMU coverage wait exceeded` が発散前に発生する。
5. 1 kHz Unity物理は実時間より遅くなる。壁時計watchdog 0.3秒では健全な低速実行を
   センサ断と誤判定したため、Unity設定のみ2.0秒へ変更した。実機設定は0.3秒を維持する。

## 6. 次回の最優先作業

アルゴリズムを変更せず、以下を順に診断する。

1. **単一静止フレームの座標整合確認**
   - Unity真値姿勢でLiDAR点をmapへ変換
   - 各点から有限壁面までの残差分布を記録
   - 外周壁、中央壁、スラローム板、ビンゴ支柱を別集計
   - 真値姿勢でも平均残差が30 mmを大幅に超えるなら、パラメータ調整前に壁マップ寸法、
     LiDAR高さ、Unity↔ROS軸変換を修正する必要がある
2. **IMU符号確認**
   - 静止時に推定器入力が `(0, 0, +9.81)` の想定と一致するか
   - GTSAM `MakeSharedU` とUnity specific forceの重力符号を確認
   - 注入バイアスと `ConstantBias` の軸・符号を1軸ずつ確認
3. **ICP採用率の計測**
   - 1秒ごとの `planar_inliers`、平均残差、補正量、採否をCSV化
   - 発散する前の最初の棄却フレームを対象に1パラメータずつ変更
4. 上記が正常なら、基準設定から小さく探索:
   - `planar_max_mean_residual`: 0.06 / 0.07 / 0.08
   - `planar_max_shift`: 0.10 / 0.12 / 0.14
   - `wall_prior_base_xy_sigma`: 0.02 / 0.03 / 0.04
   - 同時変更せず、各組を同一seedで最低3回評価

## 7. 再開手順

1. Unityを停止する。
2. WSLでビルド:

   ```bash
   source /opt/ros/lyrical/setup.bash
   cd /home/yukichi6105/ros2_ws
   colcon build --packages-select lio_localization lio_localization_sim --symlink-install
   ```

3. ROSを先に起動:

   ```bash
   source /home/yukichi6105/ros2_ws/install/setup.bash
   ros2 launch lio_localization_sim unity_sensor_localization.launch.py
   ```

4. ROS Endpointが `0.0.0.0:10000` で待受を開始してからUnity Playを開始する。
   Unityを先に再生すると古い `/clock` により評価タイマーが即時満了する場合がある。
5. 評価終了時にlaunchは自動停止する。

## 8. 主な変更ファイル

- `unity/Robocon2026Sim/Assets/Robocon2026/Scripts/UnityRosSensorPublisher.cs`
- `unity/Robocon2026Sim/Assets/Robocon2026/Scripts/RoboconFieldBuilder.cs`
- `src/lio_localization_sim/config/robocon2026_unity.yaml`
- `src/lio_localization_sim/launch/unity_sensor_localization.launch.py`
- `src/lio_localization/include/lio_localization/laser_scan_matching_node.hpp`
- `src/lio_localization/src/laser_scan_matching_node.cpp`
- `src/lio_localization/src/backend_optimizer_node.cpp`

Windows側Unityプロジェクト:

`C:\Users\kouza\UnityProjects\Robocon2026Sim`

UnityセンサーモデルのC#変更はWindows側へ同期済み。
