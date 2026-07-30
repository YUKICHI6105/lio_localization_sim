# センサデータシート準拠ノイズの段階試験 (2026-07-29)

## 方針

実機は未入手のため、メーカーの公開データシートを一次値とする。ただし、複数要因を
同時に変えて原因を失わないよう、各試験は固定の60秒Unity記録bagに**一要因だけ**を
加える。

- LiDAR: 北陽電機 UTM-30LX
- IMU: TDK InvenSense ICM-42688-P (DS-000347)
- 比較基準: `max8_ray_to_wall_no_notes_fixed_bag_20260729_r5`
  (mean 0.61 mm / RMSE 0.71 mm / max 2.40 mm)

## 仕様値と実装上の単位

| センサ | データシート値 | シミュレータ/GTSAMへ入れる値 |
|---|---:|---:|
| UTM-30LX 測距繰返し精度 (0.1--10m, 室内) | sigma <10 mm | LiDAR距離白色雑音 sigma=0.010 m |
| ICM-42688-P 加速度ノイズ密度 | XY 65, Z 70 ug/sqrt(Hz) | 全軸を保守的に 70 ug/sqrt(Hz) = 6.86466e-4 m/s^2/sqrt(Hz) |
| ICM-42688-P ジャイロノイズ密度 | 2.8 mdps/sqrt(Hz) | 4.88692e-5 rad/s/sqrt(Hz) |

Unityでは連続時間ノイズ密度 `n` を各標本の標準偏差 `n/sqrt(dt)` に変換して注入する。
GTSAMの`accelerometer_sigma`/`gyroscope_sigma`には同じ連続時間密度を渡す。

UTM-30LXの`±30 mm`は測距精度の境界であり、正規分布のsigmaではない。従来の
`sigma=30 mm`はこの二つを取り違えていたため、Unityソースと通常YAMLを10 mmへ訂正した。

## 試験1: ICM-42688-P白色雑音のみ

### 注入条件

固定bagには旧UnityモデルのLiDAR白色雑音sigma=30 mmが既に焼き込まれている。そのため
この試験中はlaser nodeの`wall_cov_range_sigma`だけを30 mmへ一時上書きし、新しい要因を
ICM-42688-Pの白色雑音だけに限定した。

入力bagをオフライン変換し、ROS実行中にPython callbackを増やさないようにした。

- source: `/tmp/robocon_openloop_no_notes_20260729.mcap`
- transformed: `/tmp/robocon_openloop_no_notes_icm42688_white_20260729.mcap`
- accel sample sigma at 1kHz: 0.021707961 m/s^2
- gyro sample sigma at 1kHz: 0.001545380 rad/s
- seed: 42688

### 結果（参考・再録待ち）

run: `max8_ray_to_wall_icm42688_white_noise_20260729_r1`

| mean | RMSE | max | 判定 |
|---:|---:|---:|---|
| 0.72 mm | 0.83 mm | 2.44 mm | 参考値（無効） |

IMU stamp gapは最大1.000 msであり、IMU fail-safe、GTSAM例外、グラフリセットはなかった。
このrunはIMU白色雑音だけを正しく追加したが、入力LiDARには旧モデルのsigma=30 mmが焼き込まれて
いる。更新済みUTM-30LXモデル(sigma=10 mm)との入力不整合があるため、10 mm要件を満たす証拠には
使わない。更新済みUnityから新規CSV/MCAPを録画後、同じICM-42688-P条件で取り直す。

## 未実施要因と制約

1. **UTM-30LX sigma=10 mm + ICM-42688-P白色雑音でのend-to-end再試験**: Unityの発行値を
   更新済みだが、既存bagは30 mm雑音を含むため、これを後処理で10 mmへ「減らす」ことはできない。
   更新済みUnityから新しいCSV/MCAPを記録してから実行する。
2. **バイアスランダムウォーク・温度ドリフト・振動**: ICM-42688-Pの同データシートには
   本システムの`bias_*_random_walk_sigma`へ直接入れられる値がない。Allan偏差または実機
   温度試験なしに任意値を採用しない。

## 試験1（再録・有効）: Unity再録のデータシート構成

更新済みUnity 6000.5.5f1から、UTM-30LXの距離白色雑音sigma=10 mmと、
ICM-42688-Pの白色雑音密度を発行時に注入した65.219秒のCSVを新規記録し、ROS 2 bagへ
変換した。旧bagへの後処理ノイズ注入は用いていない。

- Unity記録: `SensorRecordings/20260729_121252_415_61293a5a`
  (`manifest.json`: `complete: true`)
- ROS 2 bag: `/tmp/robocon_icm42688_utm30lx_65s_20260729.mcap`
- 入力件数: IMU 65,220件、scan 2,609件、ground truth 6,522件
- 評価run: `datasheet_unity65s_baseline_20260729_r3`
- 条件: ray-to-wall有効、追加のROS側ノイズ注入なし、1x再生、notes/ballなし

| 定常mean | 定常RMSE | 定常max | 判定 |
|---:|---:|---:|---|
| 0.56 mm | 0.65 mm | 2.18 mm | PASS (RMSE<=10 mm / max<=10 mm) |

scan matchは2,480件（62秒 x 40 Hz）すべて到着し、IMU stamp gapは最大1.000 ms、
GTSAM例外・IMU LOST・グラフリセットはいずれもなかった。ROS側のIMU callback wall-time
gapは最大32.199 ms（>10 ms: 111回）だったが、入力stampの連続性と本試験の誤差指標に
影響は確認されなかった。実行遅延は引き続き実機負荷評価で監視する。

## 試験2（有効）: ICM-42688-P初期オフセット最大値

校正後残差の分布は未入手のため、ユーザー指示によりデータシートの起動時許容差を
そのまま推定器の名目初期値からの残差として用いた。白色雑音、LiDAR雑音、既存の
X/Y加速度・Zジャイロのランダムウォークは試験1と同一であり、ROS側の初期バイアスYAMLは
名目値のまま据え置いた。

- 追加加速度オフセット: 各軸 +20 mg = +0.196133 m/s^2
- 追加ジャイロオフセット: 各軸 +0.5 deg/s = +0.008726646 rad/s
- Unityへ入れた合計初期バイアス:
  accel=(0.246133, 0.166133, 0.196133) m/s^2,
  gyro=(0.008727, 0.008727, 0.013727) rad/s
- Unity記録: `SensorRecordings/20260729_124528_289_9f65fe04`
  (`complete: true`; IMU 65,212件、scan 2,609件、ground truth 6,522件)
- ROS 2 bag: `/tmp/robocon_icm42688_utm30lx_initial_offset_20mg_050dps_65s_20260729_r2.mcap`
- 有効評価run: `datasheet_initial_offset_20mg_050dps_20260729_r3`

| 定常mean | 定常RMSE | 定常max | 判定 |
|---:|---:|---:|---|
| 0.76 mm | 1.04 mm | 6.65 mm | PASS (RMSE<=10 mm / max<=10 mm) |

scan matchは入力再生区間の2,480件すべてが連続して到着し、IMU stamp gapは最大1.000 ms、
GTSAM例外・IMU LOST・グラフリセットはなかった。callback wall-time gapは最大26.502 ms
（>10 ms: 18回）だが、入力時刻の欠損はない。途中のr1は追加乱数抽選が白色雑音列を変えた
ため、r2はscan 1件欠損のため、いずれも無効として保全した。r3のみを本試験の根拠とする。

## 試験3（有効）: バイアス・ランダムウォーク感度

データシートに直接のランダムウォーク値がないため、採用中の`1e-4`と、履歴に残っていた
最小の非ゼロ候補`1e-6`を比較した。Unityの乱数抽選回数は両条件で固定し、ドリフトの振幅
だけを変えた。`0`はGTSAMの零分散因子が`IndeterminantLinearSystemException`を起こし、
グラフを反復リセットして成立しなかったため、物理的な比較値としては採用しない。

- 有効Unity記録 (`1e-6`): `SensorRecordings/20260729_133241_292_9f87a94b` (`complete: true`)
- 有効run: `datasheet_bias_rw_1e6_20260729_r2`
- Unity注入とbackendの双方: accel/gyro random walk sigma=`1e-6`
- scan match: 2,480件連続、IMU stamp gap最大1.000 ms、例外・リセットなし

| random walk sigma | 定常RMSE | 定常max | 判定 |
|---:|---:|---:|---|
| `1e-4`（採用値、試験1再録） | 0.65 mm | 2.18 mm | PASS |
| `1e-6` | 0.87 mm | 2.31 mm | PASS |
| `0` | グラフ不定・推定破綻 | 5,447 mm | FAIL |

この65秒の限定走行では`1e-4`と`1e-6`に優劣を示す根拠はない。値`0`は数値的に許容されず、
データシート根拠もないため、現行の`1e-4`は維持する。温度ドリフト・長時間走行・振動を
評価するには、実機のAllan偏差または環境試験データが必要である。

## 変更ファイル

- `unity/Robocon2026Sim/Assets/Robocon2026/Scripts/UnityRosSensorPublisher.cs`
- `config/robocon2026_unity.yaml`
- `tools/rewrite_bag_with_imu_noise.py`

## 試験4: LiDAR取付ヨー残差（暫定）

実機の取付治具公差は未入手のため、以前の感度確認と同じ `+0.20 deg` を、Unityの
65秒記録を不変にしたROS relayで `/scan` の角度列だけに加えた。距離・IMU・時刻・
ground truthは変更していない。

| 条件 | 定常RMSE | 定常max | scan | 判定 |
|---|---:|---:|---:|---|
| scan yaw `+0.20 deg` | 1.19 mm | 3.11 mm | 2480/2480 | PASS |

- run: `datasheet_scan_yaw_bias_020deg_20260729_r1`

これは仕様値ではなく取付キャリブレーションの感度試験である。取付後には外部
キャリブレーションでこの残差を測定し、ゼロ近傍へ補正することが必要である。

## 試験5: 温度・振動・感度・クロス軸

いずれも更新済みUnity 65秒bagを入力とし、既に含まれる白色雑音を二重に加えず、
bag変換時に一要因だけを追加した。すべての有効runでscan matchは2480/2480、
`pending_overwrite=0`、`pending_queue_overflow=0`、GTSAM例外・IMU LOST・グラフ
リセットなしを確認した。

| 要因 | 注入条件 | 定常RMSE | 定常max | 判定 |
|---|---|---:|---:|---|
| 温度 | 25→45°C、加速度zero-G/scale、ジャイロoffset/scaleの全温度係数 | 4.55 mm | 8.20 mm | PASS |
| 振動（暫定） | IMU X軸、100 Hz正弦、peak 0.10 g | 0.65 mm | 2.18 mm | PASS |
| 加速度感度 | 全軸 `+0.5 %` | 2.06 mm | 5.40 mm | PASS |
| 加速度クロス軸 | X←Y `+1.0 %` | 2.15 mm | 6.05 mm | PASS |
| ジャイロ感度 | 全軸 `+0.5 %` | 0.65 mm | 2.16 mm | PASS |
| ジャイロクロス軸 | X←Y `+1.25 %` | 0.65 mm | 2.16 mm | PASS |

- 温度: `datasheet_temp_ramp_20c_full_20260729_r1`
- 振動: `datasheet_vibration_100hz_010g_20260729_r1`
- 加速度感度/クロス軸: `datasheet_accel_scale_p05pct_20260729_r1`,
  `datasheet_accel_crossaxis_1pct_20260729_r1`
- ジャイロ感度/クロス軸: `datasheet_gyro_scale_p05pct_20260729_r1`,
  `datasheet_gyro_crossaxis_125pct_20260729_r1`

温度係数、初期感度、クロス軸の数値はICM-42688-P DS-000347に基づく。20°Cランプ
自体は実機の温度履歴ではなく、暫定の環境感度ケースである。振動の振幅・周波数も
同データシートに定量規定がないため採用値にはせず、実機加速度ログまたは振動試験で
置換するまでのストレス確認とする。

## 除外: UTM-30LX未校正絶対距離バイアス

UTM-30LXの`±30 mm`は繰返し精度sigmaではなく絶対精度境界である。既知の一様
`+30 mm`を追加した有効run
`utm30lx_range_bias_30mm_fixed_pipeline_20260729_r1`はRMSE 21.46 mm、最大
33.16 mmとなりFAILした（scan 2480/2480）。これは未校正の絶対測距バイアスを
10 mm自己位置要件に含めると満たせないことを示すため、本要求のノイズ適合判定からは
除外する。実機導入時は壁面基準でLiDAR距離オフセットを校正し、その残差を別途再試験する。
