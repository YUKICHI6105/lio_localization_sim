# 実験履歴: Backend専用optimizer worker導入影響評価(2026-07-29)

## 1. 目的

`backend_optimizer_node`は`SingleThreadedExecutor`で動作しており、従来は
`scan_match_callback`からiSAM2更新までを同期実行していた。このため最適化中は
1kHz IMU callbackも停止していた。

今回、callbackをキュー追加とcondition variable通知だけに限定し、IMU積分・因子生成・
iSAM2更新・`/state_estimate`配信を専用optimizer workerへ移した。GTSAM状態は引き続き
単一スレッドだけが所有する。本試験は、この変更がclosed-loop測位精度へ与える影響を
確認する初回評価である。

## 2. 再現性の確保

`scripts/stage4_mcp_run.sh`を変更し、各runへ次を自動保存するようにした。

- 実際に使用した`robocon2026_unity.yaml`
- `lio_localization` / `lio_localization_sim`のコミットID
- 両リポジトリの`git status`
- tracked fileの未コミット差分(binary patch)
- run名、開始時刻、probe有無を記したmanifest

これにより、2026-07-28以前の一部runにあった「run名から設定を推測するしかない」
状態を解消した。

## 3. 実験条件

- Unity Editor 6000.5.5f1、MCPからPlay/Stop
- Stage 4フルスタックclosed-loop、60秒評価
- IMU 1kHz設定、`/clock` 250Hz
- `smoother_lag_sec=0.5`
- `bias_acc_random_walk_sigma=0.0001`
- `bias_gyro_random_walk_sigma=0.0001`
- `zupt_enabled=false`
- `imu_dynamics_enabled=false`

比較対象はworker導入前の同系列3run
(`bias_random_walk_20260728`、`bias_random_walk_rep2_20260728`、
`bias_random_walk_rep3_20260728`)とした。

## 4. 結果

### 4.1 通常構成による3回反復

| 構成 | run | samples | mean | RMSE | max | slow smoother回数 |
|---|---|---:|---:|---:|---:|---:|
| worker前 | bias_random_walk | 58,769 | 6.28mm | 6.91mm | 12.90mm | 2 |
| worker前 | bias_random_walk_rep2 | 59,987 | 6.06mm | 6.59mm | 13.07mm | 3 |
| worker前 | bias_random_walk_rep3 | 59,361 | 6.14mm | 6.83mm | 12.99mm | 4 |
| worker後 | backend_worker_20260729_r2 | 58,933 | 6.19mm | 6.84mm | **22.72mm** | 2 |
| worker後 | backend_worker_20260729_r3 | 55,090 | 6.12mm | 6.77mm | 13.11mm | 0 |
| worker後 | backend_worker_20260729_r4 | 55,634 | 6.01mm | 6.63mm | 13.00mm | 7 |

3runの代表値:

| 指標 | worker前 | worker後 |
|---|---:|---:|
| meanの平均 | 6.16mm | 6.11mm |
| RMSEの平均 | 6.78mm | 6.75mm |
| maxの中央値 | 12.99mm | 13.11mm |
| slow smoother回数の中央値 | 3 | 2 |

定常的なmean/RMSEおよび典型的なmaxは実質同水準であり、worker化による通常精度の
明確な回帰は観測されなかった。最大10mm要件は従来どおり未達。

`backend_worker_20260729_r2`だけはt=58.454秒に22.72mmのピークを記録した。同runでは
GTSAM例外、IMU fail-safe、壁補正拒否、グラフリセットは発生しておらず、
slow smoother警告も2回でworker導入前の通常範囲だった。後続2runでは再現せず
13.11mm / 13.00mmへ戻ったため、現時点ではworker化に起因する再現性のある回帰とは
判定しない。ただし単発外れ値として保留し、今後同位相で再発するか監視する。

### 4.2 診断プローブ付きrun

`backend_worker_20260729_r1`は`--probes`付きで実行し、RMSE 6.54mm、max 55.30mmだった。
slow smoother警告15回、laser scan matcherのIMU coverage timeout 32回と、通常runより
明らかに高負荷だった。ピークは衝突相当の高加速度窓内で、窓外maxは15.38mm。

このプロジェクトでは過去にも診断subscriber自身が受信率を低下させ、「IMUを58%落とした」
ように見せた事例がある。そのため、このrunはworker精度の主比較から除外し、診断負荷を
含むストレスrunとしてのみ保存する。

## 5. Backend内部カウンタによる直接定量化

外部subscriberを増やさずに並行動作を測るため、backend内部へ次の軽量カウンタを追加した。
hot pathでは整数加算とatomicフラグ参照だけを行い、ログ出力はノード終了時の1回だけとした。

- IMU callback総数
- optimizer worker処理中に実行されたIMU callback数
- `smoother_->update()`と`calculateEstimate()`の実行中に実行されたIMU callback数
- IMU callback壁時計gapの最大値と2ms / 10ms超過回数
- worker jobとsmoother updateの回数、平均時間、最大時間

通常構成の`backend_worker_quant_20260729_r1`で得た値:

| 指標 | 結果 |
|---|---:|
| IMU callback総数 | 63,466 |
| worker処理中のIMU callback | 5,572 (総数の8.8%) |
| iSAM2/estimate処理中のIMU callback | 3,905 (総数の6.2%) |
| worker job | 2,336回、平均2.653ms、最大48.000ms |
| smoother update | 2,335回、平均1.713ms、最大35.068ms |
| callback gap最大 | 188.229ms |
| callback gap >2ms | 8,206回 |
| callback gap >10ms | 2,127回 |

丸め済み平均時間から概算すると、workerが実際に動いていた累計時間は約6.20秒、
smoother処理の累計時間は約4.00秒である。その間のIMU callback実行密度はそれぞれ
約899 callback/s、約976 callback/sだった。

従来構造ではiSAM2がIMU callbackと同じSingleThreadedExecutor上で同期実行されていたため、
iSAM2処理中に実行できるIMU callbackは構造上0件である。今回3,905件を直接計測できたことから、
**専用worker化によって最適化中もIMU callbackが継続することを定量確認した**。

一方、最大188msのcallback gapと10ms超過2,127回は残っている。専用workerは
「backend自身のiSAM2がexecutorを止める」原因を除いたが、Unity→ROS TCP輸送、
DDS/executorスケジューリング、ホスト負荷などによるバースト到着や欠落までは解消しない。
この2つは分けて扱う必要がある。

同runの精度はmean 6.30mm、RMSE 6.95mm、max 13.79mmで、定常RMSEはPASS、
最大10mm要件はFAIL。内部カウンタ追加後も過去系列と同水準だった。

### 5.1 再試験(最大5回、正常データ取得時点で終了)

再現確認では、次を「問題のないデータ」の条件とした。

- レポートが正常生成され、GTSAM例外・グラフリセット・IMU fail-safeがない
- iSAM2処理中のIMU callback数が0件より多い
- 定常RMSE 10mm以下
- 最大誤差が過去通常帯付近の15mm以下

`backend_worker_quant_20260729_r2`はPlay要求後30秒以上たってもUnityセンサーの
ROS実接続が成立せず、ground truthが1件も届かなかった。推定器評価ではなく既知の
Unity–ROS接続失敗としてログを保全し、無効runとした。

次の`backend_worker_quant_20260729_r3`で正常データが得られたため、最大5回まで
繰り返す予定をここで終了した。

| 指標 | 初回定量run | 正常再試験r3 |
|---|---:|---:|
| mean / RMSE / max | 6.30 / 6.95 / 13.79mm | 6.20 / 6.83 / 12.53mm |
| IMU callback総数 | 63,466 | 64,222 |
| worker中callback | 5,572 (8.8%) | 5,365 (8.4%) |
| iSAM2中callback | 3,905 (6.2%) | 3,660 (5.7%) |
| worker平均 / 最大 | 2.653 / 48.000ms | 2.626 / 42.575ms |
| smoother平均 / 最大 | 1.713 / 35.068ms | 1.668 / 36.758ms |
| callback gap最大 | 188.229ms | 177.674ms |
| gap >2ms / >10ms | 8,206 / 2,127回 | 9,466 / 2,232回 |

正常再試験r3ではworker累計時間が概算約6.37秒、smoother累計時間が約4.04秒であり、
その間のIMU callback実行密度はそれぞれ約842 callback/s、約905 callback/sだった。
初回定量runと独立に、iSAM2中もIMU callbackが継続することを再確認した。

## 6. 結論

- worker化後も定常RMSEは3runすべて10mm以下で、通常精度の明確な回帰はない
- 典型的な最大誤差は約13mmでworker導入前と同水準。最大10mm要件は未達のまま
- 22.72mmの単発ピークは再現せず、原因は未確定
- slow smoother回数は中央値では3回から2回だが、0〜7回とばらつきが大きく、
  worker化による計算時間改善を示す結果ではない
- evaluator sample数はworker後に平均約4.7%少なかったが、これはUnity/ホスト実行速度も
  含む値であり、backendのIMU callback受信数そのものではない
- iSAM2処理中に3,905件(概算976 callback/s)のIMU callbackが実行され、
  worker分離の目的は定量的に達成した
- 最大188msのcallback gapは残っており、輸送・ホストスケジューリング層は別途調査が必要

したがって、専用worker実装は**通常精度を維持し、iSAM2によるcallback停止を解消したため
採用可能**と判断する。22.72mmピークの再発監視と、iSAM2以外を原因とする受信gapの調査は
残課題とする。
