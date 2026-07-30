# UnityセンサCSVからrosbag2(MCAP)を作る手順

Unityで生成されたセンサ値を、ROS-TCP接続へ渡す**直前**にCSVへ記録し、そのCSVから
rosbag2(MCAP)を作るための手順です。ROS側の受信遅延・TCPキューあふれ・コールバック
遅延を含まないため、同じ物理走行をROSの条件だけ変えて再生する基準データになります。

## 記録されるもの

`UnityRosSensorPublisher`はPlay開始ごとに、実際に開いているUnityプロジェクト直下の
`SensorRecordings/<UTC時刻>_<乱数>/`を作成します。

- `imu.csv` — `/imu/data`、1 kHz
- `scan.csv` — `/scan`、40 Hz
- `ground_truth.csv` — `/ground_truth_pose`、100 Hz
- `manifest.json` — Unity版、fixed timestep、件数、完了状態

各行はUnityのセンサモデルが生成したメッセージそのものです。書き込みは専用スレッドで
行うので、1 kHzの`FixedUpdate`でディスクI/Oをしません。`manifest.json`の
`complete: true`は、Play停止時に全キューをCSVへフラッシュできた印です。`false`の
記録は原則として変換しません。

現行の実プロジェクトでは、Windows側の保存先は次です。

```
C:\Users\kouza\UnityProjects\Robocon2026Sim\SensorRecordings
```

WSLからは`/mnt/c/Users/kouza/UnityProjects/Robocon2026Sim/SensorRecordings`として読めます。

`/clock`はCSVへ保存しません。再生時に`ros2 bag play --clock 250`がbagの時刻から新しく
生成するため、実行時の250 Hz clock設定を再現できます。IMUサンプルのheader時刻は1 kHzの
まま保持されます。

## CSVをMCAPへ変換する

一度ワークスペースをビルドし、生成された記録ディレクトリを指定します。

```bash
cd /home/yukichi6105/ros2_ws
colcon build --packages-select lio_localization_sim
source install/setup.bash

ros2 run lio_localization_sim unity_csv_to_rosbag2 \
  /mnt/c/Users/kouza/UnityProjects/Robocon2026Sim/SensorRecordings/<run_id> \
  /tmp/robocon_<run_id>.mcap
ros2 bag info /tmp/robocon_<run_id>.mcap
```

出力先は新しいディレクトリでなければなりません。変換器は各CSVの連番とトピック内時刻の
単調性、CSV行数とmanifest件数を検証します。成功時はbag内に
`unity_csv_conversion.json`も書き、入力manifestのSHA-256とトピック別件数を残します。

同じbagを評価系へ流す例は次です。

```bash
ros2 bag play /tmp/robocon_<run_id>.mcap --clock 250
```

再生先の各ノードは`use_sim_time:=true`で起動します。bagにはUnityのROS送信後の挙動や
`/odom_fast`を使う閉ループ制御は含まれません。したがってこれは**センサ入力に対する
ROS側推定器の再現試験**であり、Unityまで含めた閉ループ走行の代替ではありません。

## 運用上の注意

- Playを強制終了せず、必ずStopして`complete: true`を確認する。
- CSVの保存先はUnity実プロジェクトであり、リポジトリの
  `unity/Robocon2026Sim`コピーではない。実験ごとにrunディレクトリを丸ごと保全する。
- `manifest.json`が未完了でも救済変換が必要な場合だけ`--allow-incomplete`を使う。
  そのbagは欠損の可能性があるため正式な比較データにしない。
- Unity側のランダムシード、設定YAML、ソースコミットも既存の
  `stage4_mcp_run.sh`スナップショットと同じ実験フォルダへ保存する。
