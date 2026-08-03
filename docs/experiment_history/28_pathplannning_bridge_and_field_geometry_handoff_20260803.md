# pathplannningブリッジ・フィールド形状修正 引継ぎレポート

作成日: 2026-08-03
対象: 千葉大学ロボットコンテスト2026、経路追従(B-spline+FF+PD)とUnity実機物理接続検証
想定読者: このワークスペースを初めて引き継ぐ人

## 1. これは何のドキュメントか

2026-08-01〜08-03に行った作業(1. `pathplannning`パッケージのUnity実機物理接続検証、
2. ルールブック(千葉大学ロボットコンテスト2026)に基づくフィールド形状の修正)の
到達点・残作業・ハマりどころをまとめた引継ぎ資料。環境構築自体は
[Environment restore](../setup/RESTORE.md)を参照。

## 2. リポジトリ構成(2026-08-03時点)

```
ros2_ws/src/
├── lio_localization/        (git, branch: feature/icp-covariance)  自己位置推定
├── lio_localization_sim/    (git, branch: feature/eval-fidelity)   本レポートの置き場所
│                                                                    フィールド定義・Unityプロジェクト本体
├── pathplannning/           (git, branch: main, 2026-08-03 git init) B-spline+FF+PD経路追従
├── sotoba/                  (git, branch: main)                    ICPライブラリ(フォーク版必須)
└── ROS-TCP-Endpoint/         (git, branch: main-ros2)               Unity⇄ROS TCPブリッジ
```

`pathplannning`は本レポート作成時点でリモート未設定のローカルリポジトリ(初回コミットのみ)。
リモートに push する場合は別途相談すること。

## 3. pathplannningパッケージとは

`経路生成v4.md`(パッケージルート直下)準拠のC++23 ROS2パッケージ。B-splineで経路を
生成し、FF+PDで追従制御し、状態機械(IDLE/ARMED/RUNNING/SETTLING/COMPLETE/ABORT)で
実行を管理する。詳細設計は`docs/architecture.md`(Mermaid図2枚)を参照。

車輪運動学(`Omni3Kinematics`)・車輪指令(`/wheel_cmd`)は実機投入時用に実装済みだが、
Unity側にもROS側にもオムニホイールの動力学(滑り・モータ遅れ)を再現する手段が無いため、
**現在はFF+PDが計算した車体速度指令(body twist)を`/cmd_vel_body`(Twist)として直接
Unityへ配信し、Unity側の薄いブリッジがRigidbodyへそのまま適用する**方式で検証している
(車輪運動学は経由しない)。

### 起動手順

```bash
# 1. ROS2スタック(lio_localization_sim)
source /home/yukichi6105/ros2_ws/install/setup.bash
ros2 launch lio_localization_sim unity_sensor_localization.launch.py enable_evaluator:=false

# 2. Unity側: プロジェクト直下に PathplanningBridgeMode.flag という名前の空ファイルを
#    置いてからPlay(このフラグが無いと通常の15leg方式ミッションが動いてしまう)

# 3. pathplannning_node(leg=YAMLファイル1本につき1回起動、完了後は再起動して次のlegへ)
ros2 run pathplannning pathplannning_node --ros-args \
  -p use_sim_time:=true -p abort_state_timeout_sec:=2.0 \
  -p control_period_overrun_ratio:=30.0 -p kp_yaw:=0.08 -p kd_yaw:=0.35 \
  -p max_translational_velocity:=1.5 -p max_translational_acceleration:=1.5 \
  -p trajectory_spec_path:=<workspace>/install/pathplannning/share/pathplannning/config/legs/<leg>.yaml \
  -p log_csv_path:=<出力先>.csv

# 4. 開始
ros2 service call /pathplannning_node/start_trajectory std_srvs/srv/Trigger "{}"
```

### 現在の推奨パラメータ(2026-08-02〜03に確定)

- `kp_yaw=0.08, kd_yaw=0.35`(v4.md §34の実機既定値2.0/0.0とは別、Unity⇄ROS
  ブリッジ検証専用のチューニング値)
- `abort_state_timeout_sec=2.0`
- `control_period_overrun_ratio=30.0`(Unity Editorのタイミング揺らぎを許容するため)

### 【最重要】過去にハマった符号バグ(解決済み、再発防止のため必読)

`PathplanningCmdVelBridge.cs`の`invertYawRateSign`は**`true`でなければならない**。
`FieldCoordinates.YawFromForward = atan2(-forward.x, forward.z)`の帰結として
`field_yaw = -eulerY`(Unityのオイラー角と符号が逆)であり、これを反転せずに
`angularVelocity.y`へ直接代入するとyaw制御ループが正帰還になり、SETTLING突入後に
yawがゆっくり発散して±πでチャタリングし続ける(位置はほぼ収束するのでyawだけの
問題に見える)。この現象は「ゲイン不足」に見えるため、過去に何度もゲイン調整
(2.0/0.0→0.3/0.1→0.15/0.2)で対処しようとして失敗した。**yawが振動ではなく
滑らかな単調増加を示し、かつ指令yaw_rateの符号と実測yaw変化の符号が逆なら、
まずこの符号を疑うこと。**

詳細な実測根拠は`project_pathplannning_v4_status`メモリ(Claude Codeのメモリ機構、
このリポジトリの外)に記録されている。

## 4. 経路(leg)の検証状態(2026-08-03時点)

`config/legs/`に15leg分のYAMLがある(start→pickup×5、pickup→bingo×5、bingo→pickup×5)。

| leg | 状態 |
|---|---|
| `start_to_pickup1` | **検証済み・完全成功**(衝突ゼロ、rms_pos_err≈14mm、rms_yaw_err≈0.003rad) |
| `pickup1_to_bingo` | **検証済み・完全成功**(衝突ゼロ、rms_pos_err≈10mm、rms_yaw_err≈0.014rad、下記フィールド修正済み壁6本を回避する制御点を追加済み) |
| `start_to_pickup2〜5` | スタート地点座標の更新のみ済み(下記フィールド修正反映)。スラロームを通らない短距離なので恐らく無修正で通るはずだが**未実走行検証** |
| `pickup2_to_bingo〜pickup5_to_bingo` | **未着手**。`pickup1_to_bingo`と同様のスラローム壁回避用制御点(3点、下記参照)が出発x座標に応じて必要になる可能性が高い |
| `bingo_to_pickup1〜5` | **未着手**(復路、上記の鏡像+順序反転が必要) |

### `pickup1_to_bingo`が回避している壁と、その制御点の根拠(他legの参考用)

```yaml
position_control_points:
  - [-2.539, 0.394337567]
  - [-2.539, 0.394337567]
  - [-1.579, 0.670]   # notes/start境界の開口部(安全y範囲[0.556, 0.780])
  - [-0.850, 0.380]   # baffle_left_1(x=-0.821, 下端y=0.800)の下をくぐる(y<0.511必要、余裕0.131m)
  - [-0.450, 0.500]
  - [-0.300, 1.060]
  - [0.021, 1.060]    # baffle_left_2の上端(y=0.819)をクリア
  - [0.350, 1.060]
  - [0.600, 0.400]    # goal_slalom_boundary_left(x=+0.821, y:[0.8,1.6715])の開口部を通す
  - [2.0325, 0.494337567]
  - [2.0325, 0.494337567]
duration_sec: 22.0   # 制御点が増えるほど曲率が上がるため、TrajectoryValidatorの
                     # max_translational_accelerationチェックを通すためにduration_secを延長する必要がある
```

## 5. フィールド形状修正(ルールブック照合、2026-08-02)

`config/robocon2026_field.json`をルールブックPDF(CADの実測値)と照合し、以下を修正済み
(WSL側・ライブWindows側Unityプロジェクト双方に同期済み):

- スタートゾーンの奥行き: 800mm→1240mm(`zones[0/1].x_max`変更、スタート座標も
  `(-2.419,1.354)`→`(-1.919,1.3715)`へ更新)
- スラロームバッフル`baffle_left_1`/`right_1`の位置・長さ修正
- スラローム境界に新規壁4本追加(`notes_start_boundary_top/bottom_outer/inner`)
- ゴール側スラローム境界に新規壁2本追加(`goal_slalom_boundary_left/right`)
- bingo棚の位置修正(`bingo.centre.x`: 2.316→2.0325、奥壁からのクリアランスを
  16.5mm→300mmへ拡大)
- bingoポケット仕切りを「両側壁」から「中央線1本」(`bingo_pocket_centre`)へ修正
  (ユーザー指摘により訂正)
- 各種`missions.*.finish`座標をbingo新centre.xに合わせて更新

**未解決・要注意**: `baffle_left_2`/`baffle_right_2`(x=0.021)の位置はルールブック上
どの壁に対応するか未確認のまま(ユーザーに確認したが「どこのことか分からない」との
回答で保留中)。ここまでの検証では衝突を起こしていないが、正式な確認はまだ済んでいない。

Unity側のロボットメッシュ(シャシー・タイヤ・LiDARマウント)の透過(半透明に見える)
バグも同じ調査の過程で発見・修正済み(法線の向き=ワインディング順序の誤りが原因、
`StartToBingoValidation.cs`の`CreateRoundedTrianglePrism`/`CreateCylinderMesh`)。

## 6. 運用上の注意点(このセッションで踏んだ地雷)

### WSL2のゾンビTCPソケット

Unity⇄ROS接続が「Unity側は繋がっているつもりなのにROS側に何も届かない」状態になったら、
まず`ss -tanp | grep 10000`で所有者の無いESTABソケットが残っていないか確認する。
これは`wsl --shutdown`(PowerShellから)でしか消えない。詳細は
Claude Codeのメモリ`feedback_wsl_orphan_socket_unity_ros`を参照(このリポジトリの外)。

### `ros_tcp_endpoint`の変更(2026-08-02、`ROS-TCP-Endpoint`リポジトリの
`main-ros2`ブランチにコミット済み、コミット`65aaf54`)

**1. スレッドプール枯渇バグ(実挙動に影響する本質的な修正)**

`src/ROS-TCP-Endpoint/ros_tcp_endpoint/server.py`の`setup_executor()`が、
`MultiThreadedExecutor`のスレッド数を`publishers_table`/`subscribers_table`/
`ros_services_table`/`unity_services_table`の**その時点の**要素数の合計+1から
計算していた。しかしこの関数はプロセス起動直後、Unityが接続してトピックを
登録する**前**に1回だけ呼ばれるため、これらのテーブルは常に空でnum_threadsは
常に1になっていた。

後から登録されるトピック(Unityが接続後に`on_publisher_registration`等経由で
`executor.add_node()`する)は同じexecutorインスタンスに追加されるが、
`MultiThreadedExecutor`のスレッドプールサイズは構築時に固定で、ノード数が
増えても後から広がらない。結果、Unityの単一センサーパブリッシャーが持つ
~8本のトピック(scan/imu/clock/odom_fast/ground_truth_pose等)が全て1本の
ワーカースレッドに詰め込まれて互いを枯渇させ、CPU負荷もネットワークも健全なのに
各パブリッシャーの送信キューが埋まり続け、Unity側コンソールに
`Queue full! Messages are getting dropped!`が多発していた
(IMU/LiDARサンプルが無言でロストし、自己位置推定の精度を独立に悪化させる)。

`num_threads=16`の固定値に変更済み。この環境ではdev-mode install
(`install/ros_tcp_endpoint/lib/ros_tcp_endpoint/server.py`が`src`側へ
symlinkされている、`readlink -f`で確認済み)のため、**src編集が即反映され
再ビルド不要**。

**2. `setup.cfg`/`setup.py`の現行setuptools対応(ビルドが通らなかったための修正)**

このワークスペースの環境にインストールされているsetuptoolsのバージョンでは、
`setup.cfg`の`script-dir`/`install-scripts`キー(ハイフン形式)と
`setup.py`の`tests_require`引数(非推奨・削除済みオプション)が拒否され
パッケージがビルドできなかった。`script_dir`/`install_scripts`
(アンダースコア形式)へのリネームと、`tests_require`の削除で解消済み。

### 起動・停止の順序

**起動**: ROS2スタック → `pathplannning_node` → Unity Play(最後)。
**停止**: Unity Playを先に停止 → その後ROS2プロセスをkill(逆順にすると
上記ゾンビソケットや`backend_optimizer_node`のIMU FAIL-SAFE HALTを誘発しやすい)。

### IMU FAIL-SAFE HALT

Unity Playが停止した状態でROS2ノードが動き続けると(またはIMUストリームが
0.3秒以上途絶すると)、`backend_optimizer_node`が自己位置推定を永久に凍結する
(`imu_watchdog`、`steady_clock`ベース、fail-safe halt方針で自動復帰しない)。
発生したらROS2プロセスの再起動が必要。

### `PathplanningBridgeMode.flag`

Unityプロジェクト直下(`unity/Robocon2026Sim/`と同じ階層、`Assets`の外)に
この名前の空ファイルを置いた状態でPlayすると、通常の15leg方式ミッション
(`StartToBingoValidation`本体)ではなく`pathplannning`ブリッジモードで起動する。
削除すれば通常ミッションに戻る(回帰確認済み)。

## 7. 次にやるべきこと

1. `start_to_pickup2〜5`の実走行検証(恐らく無修正で通るはずだが未確認)
2. `pickup2_to_bingo〜pickup5_to_bingo`への壁回避制御点追加(4.の表・5.の壁情報を参照)
3. `bingo_to_pickup1〜5`(復路)の経路作成
4. `baffle_left_2`/`baffle_right_2`のルールブック上の対応箇所確認
5. (任意)符号バグ修正後はyawゲインを上げてより高速な走行が可能なはず
   (`kp_yaw=0.08/kd_yaw=0.35`より速いゲインでの安定性検証は未実施)
