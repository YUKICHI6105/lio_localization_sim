# 実験履歴: Stage 4(Unity + ROS 本番フルスタック、closed-loop)

- 実行環境: Unity Editor(6000.5.4f1→6000.5.5f1)をセンサー・物理として実行し、
  ROS 2側のフルスタック(imu_preintegration・laser_scan_matching・backend_optimizer・
  evaluator等)を同時に動かした closed-loop 構成。`/odom_fast`(本番推定器の出力)を
  経路追従の制御フィードバックにも使う。
- 対象期間: 2026-07-23 〜 2026-07-27。
- 生データ: `docs/stage4_runs/<run名>/`(各runに`sim_eval_report.txt`, `ros.log`等)。
- 関連する詳細レポート: `04_stage4_resume_notes.md`(2026-07-23〜25の経緯)、
  `06_localization_10mm_validation_report_2026-07-27.md`(2026-07-27の最大10mmキャンペーン詳細)。

Stage 1〜3([`02_stage1-3_synthetic_sim_summary.md`](02_stage1-3_synthetic_sim_summary.md))
が物理エンジンなしの解析的シミュレータでRMSE~1mm/max<6mmを達成した実装を、Unityの
センサー・物理へ移植した段階。
移植直後は要件を大きく外れ、原因調査→修正→再検証を繰り返した。

## 1. Unity移植初期の根本原因(2026-07-23)

引継時点でRMSE 4m級未達だった原因は、**LiDARのスキャン平面(z=0.140m)が壁の上端
(z=0.100m)より40mm上を通っており、壁が1点も見えていなかった**こと。パラメータ
調整の問題ではなく幾何の設計ミスだった。ユーザー判断(機体設計未確定のため今は
壁を高くしてよい)により、`robocon2026_field.json`の`walls.height`を0.1→0.3へ変更して対処。

その後もLiDAR設置角度の90度死角の割り当て(ノーツ側へ向ける)、ノーツ(未マップ物体)の
ICP除外(2パスICP、空間的広がりによるクラスタ除外)など、複数の幾何・実装上の問題を
順に解消した。詳細は`04_stage4_resume_notes.md`§1・§10・§11、および
[真値使用版診断の履歴](05_truth_diagnostics_summary.md)を参照。

## 2. 要件A(旧基準: 定常RMSE10mm以下・最大ドリフト20mm以下)達成の経緯

ball exclusion(未マップのノーツをICPから除外する2パスICP)実装前後の比較(2026-07-24、
`ball_diag`/`ball_exclude`系列runとして記録):

|  | 定常RMSE | 最大誤差 | 判定 |
|---|---:|---:|---|
| ノーツ除外なし | 14.99mm | 18.00mm | 定常FAIL |
| ノーツ除外あり | 3.33mm | 14.27mm | 定常PASS / 最大PASS(20mm基準) |

要件A(当時基準: 定常±10mm、最大±20mm)を初めて両方満たした。原因は、死角の外に
残っていたオレンジ列のノーツ4個が、至近距離・マップから116〜142mm外れる・常に+y側
という3条件を同時に満たして系統バイアスを作っていたこと(詳細: `04_stage4_resume_notes.md`§11)。

### この時期の比較実験群(2026-07-24時点の20mm基準での判定)

| run | 内容 | 定常RMSE | 最大誤差 | 判定(20mm基準) |
|---|---|---:|---:|---|
| ball_diag / ball_extent | ball exclusion 有効(空間的広がり判定) | 3.33mm | 14.27mm | PASS/PASS |
| ball_exclude | 半径ゲート判定のみ(旧実装、効果ゼロ) | 14.98mm | 18.14mm | FAIL/PASS |
| decim1 | ノーツ除外なし比較用 | 14.92mm | 16.94mm | FAIL/PASS |
| fov_blind90 | 死角90度のみ(ノーツ除外なし) | 14.36mm | 21.29mm | FAIL/FAIL |
| fusion_stage | 融合段階診断(除外なし) | 15.06mm | 19.54mm | FAIL/PASS |
| huber60 | Huber閾値変更 | 21.04mm | 23.75mm | FAIL/FAIL |
| imu_a0.005_g0.0005 | IMUバイアス条件変更 | 7.28mm | 55.42mm | PASS/FAIL |
| motion_phase | 運動位相別診断 | 3.66mm | 14.71mm | PASS/PASS |
| no_deskew | デスキュー無効化比較 | 14.98mm | 17.85mm | FAIL/PASS |
| ponr_off_h0.15_s1.0 | PONR(衝突予測)無効化 | 12.49mm | 268.70mm | FAIL/FAIL |
| ponr_on_h0.15_s0.003 / h0.25_s0.001 | PONR有効(閾値違い) | 8.36〜8.71mm | 43.52〜53.47mm | PASS/FAIL |
| same_footing | 条件統一比較用 | 14.99mm | 17.88mm | FAIL/PASS |
| speed_split | 速度帯別診断 | 7.27mm | 14.52mm | PASS/PASS |
| wall_bias | 壁バイアス診断(除外なし) | 14.99mm | 18.00mm | FAIL/PASS |

このうちball_exclude(半径ゲート判定)はKasa円フィットがLiDARの片側弧しか見えない
ボール形状に対して実半径75mmを34mmと大きく外し、ゲート[0.065, 0.085]を一度も
通過できず除外が空振りしていた実装ミス。空間的広がり判定への変更(ball_diag/ball_extent)
で初めて効いた。

## 3. Unity 5.5f1再ビルド後の検証開始(2026-07-27 早朝)

| run | 実行時刻 | 定常RMSE | 最大誤差 | 判定(20mm基準) |
|---|---|---:|---:|---|
| unity_ros2_verified_20260727 | 02:45 | 7.09mm | 14.56mm | PASS/PASS |
| unity_demo_20260727 | 08:29 | 7.21mm | 14.42mm | PASS/PASS |

Unity 6000.5.5f1再ビルド後も、旧20mm基準では引き続きPASSであることを確認した上で、
同日中に新基準(最大位置誤差10mm以下)の検証キャンペーンへ移行した。

## 4. 2026-07-27 最大位置誤差10mm検証キャンペーン

要件が「定常RMSE10mm以下 **かつ** 最大位置誤差10mm以下」に更新されたことを受けた
集中検証。詳細な原因分析は`06_localization_10mm_validation_report_2026-07-27.md`を参照。
ここでは実行した全runを時系列で記録する。

### 4.1 初期診断(接続・計測条件の確認、18:40〜18:53)

| run | 時刻 | 位置誤差 mean/RMSE/max | 備考 |
|---|---|---|---|
| max10_baseline_diag | 18:40 | 7.86 / 21.53 / **864.42mm** | 診断条件の確認run。フルスタックでない/接続不安定な条件下の値で、本番の測位性能を表さない |
| max10_transport250_diag | 18:50 | 13689 / 19075 / **44609mm** | 同上。オープンループ的計測条件による無効値(Stage1教訓「フルスタック同時実行必須」参照) |
| max10_transport250_reset_diag | 18:53 | 34727 / 45646 / **101023mm** | 同上 |

これらは輸送・接続まわりの診断であり、真の測位精度を示す測定ではない。以降の
本番runと混同しないこと。

### 4.2 周波数構成の探索(20:31〜21:00)

| run | 時刻 | 定常RMSE | 最大誤差 | 判定(10mm基準) | 備考 |
|---|---|---:|---:|---|---|
| max10_1khz_direct_wsl_ip_fresh | 20:31 | 6.96mm | 143.60mm | PASS/FAIL | IMU/clockとも1kHz |
| max10_1khz_5_5f1_clean_fresh | 20:41 | 9.63mm | 183.70mm | PASS/FAIL | Unity5.5f1、IMU/clockとも1kHz。通信・処理負荷で悪化 |
| max10_1khz_5_5f1_dcs_guard | 20:47 | 11.69mm | 67.61mm | FAIL/FAIL | DCSガード試験 |
| max10_1khz_clock250 | 20:54 | 3.51mm | 13.72mm | PASS/FAIL | **`/clock`のみ250Hz化。ここで大幅改善** |
| max10_1khz_clock100 | 20:58 | 3.69mm | 12.42mm | PASS/FAIL | clock100Hz。250Hzに対する明確な優位なし |
| max10_1khz_clock250_biascal | 21:02 | 3.38mm | 13.24mm | PASS/FAIL | IMU固定バイアス較正、小幅改善 |
| max10_1khz_clock250_biascal_warmup | 21:05 | 3.85mm | 18.37mm | PASS/FAIL | 10秒静止初期化。最大値は改善せず |
| max10_1khz_clock250_biascal_warmup_no_ponr | 21:09 | 4.04mm | 29.97mm | PASS/FAIL | PONR無効化。再現性なし、不採用 |

「`/clock`のみ250Hz化」(全ROSノードへの時刻通知頻度のみを落とし、IMU本体は1kHzのまま)が
最大の改善要因だった。RMSE 9.63→3.51mm、最大183.70→13.72mm。

### 4.3 再積分・平滑化・ノイズモデルの調整(21:13〜21:48)

| run | 時刻 | 定常RMSE | 最大誤差 | 判定(10mm基準) | 備考 |
|---|---|---:|---:|---|---|
| max10_1khz_clock250_reintegration_diag | 21:13 | 3.91mm | 19.28mm | PASS/FAIL | 再積分境界修正前の診断。下流ほどピーク増幅 |
| max10_1khz_clock250_reintegration_clip | 21:17 | 3.67mm | 13.79mm | PASS/FAIL | 確定時刻境界クリップ後。**この修正は採用** |
| max10_1khz_clock250_reintegration_clip_diag | 21:21 | 3.63mm | 13.42mm | PASS/FAIL | 同上、診断付き |
| max10_1khz_clock250_lag05 | 21:24 | 3.74mm | 12.33mm | PASS/FAIL | fixed-lag smoother 0.5秒。**採用** |
| max10_1khz_clock250_lag025 | 21:27 | 3.75mm | 14.63mm | PASS/FAIL | 0.25秒。履歴不足で悪化、不採用 |
| max10_1khz_clock250_lag05_heuristic_cov | 21:29 | 3.75mm | 13.93mm | PASS/FAIL | ヒューリスティック共分散 |
| max10_1khz_clock250_lag05_huber3 | 21:32 | 3.76mm | 12.65mm | PASS/FAIL | Huber 3σ、改善なし |
| max10_1khz_clock250_lag05_lpf1ms | 21:34 | 3.72mm | 13.09mm | PASS/FAIL | LPF 1ms、改善なし |
| max10_1khz_clock250_lag075 | 21:37 | 3.67mm | 12.49mm | PASS/FAIL | lag 0.75秒 |
| max10_1khz_clock250_lag05_cov15 | 21:40 | 3.72mm | 13.03mm | PASS/FAIL | 壁共分散1.5倍 |
| max10_1khz_clock250_lag05_imu_density | 21:43 | 3.55mm | **10.79mm** | PASS/FAIL(僅差) | IMU連続時間ノイズ密度補正。**最良観測** |
| max10_1khz_clock250_lag05_imu_density_final | 21:46 | 3.89mm | 12.37mm | PASS/FAIL | 同条件、最良値を再現せず |
| max10_1khz_clock250_lag05_imu_density_repeat | 21:48 | 3.78mm | 12.65mm | PASS/FAIL | 同条件の再試験。10.79mmは再現しなかった |

`lag05_imu_density`系列の3試験(21:43/21:46/21:48)には、`bias_acc_random_walk_sigma` /
`bias_gyro_random_walk_sigma`を`0.0001`→`0.000001`へ変更する設定が導入時点から
含まれている。これは4.4の「バイアス事前分布強化」(初期値への拘束強度、撤回対象)とは
**別のパラメータ**で、走行中にバイアスが変動する速さそのものを表す。Unityは実際には
ランダムウォークではなく固定バイアスを注入しているため、この値をほぼゼロへ寄せるのは
Unityの固定バイアスセンサーモデルに合わせた意図的な変更であり、単独A/B試験こそ
行っていないものの、撤回対象ではない。

### 4.4 最終試験(21:52)

| run | 時刻 | mean/RMSE/max | 判定(10mm基準) | 備考 |
|---|---|---|---|---|
| max10_1khz_clock250_bias_prior_final | 21:52 | 3.01 / 3.50 / **21.92mm** | PASS/**FAIL** | IMUバイアス事前分布強化。衝突記録あり(baffle_left_2, left_side, Shelf_0, Post_0-2, bingo_end)。最大誤差は撤回対象 |

### 4.5 結論(2026-07-27時点)

- 定常RMSE10mm以下: **達成**
- 最大位置誤差10mm以下: **未達**(最良観測10.79mmも再現せず、最終試験は衝突混在で21.92mm)
- 採用した変更・不採用にした変更・残課題4項目は
  [`requirements_spec_2026-07-27.md`](../requirements/requirements_spec_2026-07-27.md)§2・§6、および
  `06_localization_10mm_validation_report_2026-07-27.md`§6・§7・§8を参照。
