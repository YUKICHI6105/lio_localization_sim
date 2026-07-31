# 実験履歴: ②の実共分散(Censi型)実装と検証 (2026-07-30)

19番で候補1(バイアスσ)を除外した後の、候補2(②の実共分散配信)の実装結果。

## 実装内容

`lio_localization/src/laser_scan_matching_node.cpp`の平面ソルバー
(`solve_planar`/`solve_planar_ray_to_wall`)で、最終反復のGauss-Newton情報行列
`h`(x,y,yaw順、Huber重み込み、正則化前)を新しい外側変数`planar_h`として捕捉し、

```
Cov(x,y,yaw) = wall_cov_range_sigma_^2 * planar_h^-1
```

をbackendが期待する(yaw,x,y)対角順に並べ替えて`result.wall.covariance`へ配信する
ように変更した。固有値(縮退判定)が1e-6以下ならconvariance_validをfalseにし、
従来のヒューリスティックのみへフォールバックする。固定値`{1e-5,0,0,...}`だった
箇所を置換。backend側(`wall_use_icp_covariance_`)は既存のクランプ(ヒューリス
ティックσを下限、縮退σを上限として緩める方向にのみ反映)をそのまま使うため、
backend側の変更は不要だった。ビルド成功。

## 検証(bag再生、`/tmp/robocon_angvel_fix_gyrosign_20260730.mcap`)

realtime_evaluator_nodeの5秒窓ログ(`t=15.0s recent 5s`が旋回直後窓に相当、
以前のレポートと同じ定義)で比較。

| 条件 | t=15 mean[mm] | t=15 max[mm] |
|---|---:|---:|
| 旧baseline(実共分散なし、17/18番) | 10.2 | 27.7〜28.2 |
| **実共分散のみ(σ調整なし)** | 5.82 | **22.18** |
| IMU×30のみ(σ調整、実共分散なし、18番) | 1.32 | **12.06** |
| 実共分散 + IMU×30 + yaw_sigma=0.15 | 6.17 | **22.99** |

## 所見

1. **実共分散単体はbaselineより明確に改善**(max 27.7→22.18mm、約20%減、
   mean半減)。②の実共分散配信自体は無害どころか有効に働いている。
2. **しかしIMU×30単体(12.06mm)には遠く及ばない**。
3. **重ね合わせても加算的に効かない**: 実共分散+IMU×30の組み合わせ(22.99mm)は
   実共分散単体(22.18mm)とほぼ同値で、IMU×30単体(12.06mm)より明確に悪い。
   つまり実共分散を有効にした状態では、IMU側をいくら緩めてもその恩恵が
   ほぼ消えてしまう。
4. backendの`wall prior loosened by ICP covariance`診断ログ(x/y並進のみ監視)は
   両runとも0件 — ただしこの診断はyaw方向の緩みを見ていないため、yaw_sigmaが
   Censi値で押し上げられているかどうかはこのログからは判定できない
   (yaw方向の直接ログは未実装)。
5. 仮説: 旋回中は見える壁面の法線方向分布が偏り、平面ソルバーのyaw方向
   情報量(Hessianのyaw対角/相関)が一時的に低下する可能性がある。実共分散は
   これを正しく検出して「まさにIMUを信じるべきでない旋回中に」壁priorのyaw
   σを緩めてしまい、IMU緩和策の効果を打ち消している可能性がある
   (未確認、直接のyaw_sigma値ログが必要)。

## 追加: 標準診断(wall_sigma_diag)の追加と直接確認 (2026-07-31)

「ヨー方向で問題が繰り返し起きる」という指摘を受け、憶測で次の一手を決めず、
まずbackend側のσ計算過程を毎スキャン記録する標準診断を追加した
(`lio_localization/include/lio_localization/backend_optimizer_node.hpp`の
`WallSigmaRecord`、`diag_wall_sigma_path`パラメータ、既定は空文字列=無効、
`DiagRecorder`と同じ低オーバーヘッド設計)。記録項目: 対応点数(全体/X/Y)、
縮退フラグ、Censiσ(x,y,yaw)、ICPクランプ前のヒューリスティックσ、DCS/高
ダイナミクス適用後の最終σ。

`wallsigma_diag_realcov_imu30_20260730`(実共分散+IMU×30+yaw_sigma=0.15の
組み合わせ)で記録・解析した結果:

1. **ヨー方向の「実共分散が緩めてしまう」という前セクションの仮説は誤りだった**。
   旋回窓(t=9.5〜16.0s)の全サンプルで`censi_yaw`(0.0009〜0.0027rad)は
   ヒューリスティック`heuristic_yaw_sigma`(0.00645rad、yaw_sigma=0.15/√541)を
   一度も超えず、`final_yaw_sigma`はこの区間ずっとヒューリスティックのまま
   (clampの`max(heuristic, censi)`で常にheuristic側が勝つ)。ヨーは無関係。
2. **実際に緩んでいたのはX方向**。t=11.801〜12.151sの約0.35秒間、
   `censi_x`(0.00195〜0.00205)がヒューリスティック`heuristic_x_sigma`
   (≈0.00186)をわずかに上回り、`final_x_sigma`が0.00183→0.00205へ
   約12%緩む。この直後(約1.2〜1.6秒遅れ、fixed-lag smootherの伝播遅延と
   整合)、17番で確認したstate_estimateの誤差ピーク(t=13.40s、24.9mm)が
   現れる。同じ区間でY方向は逆に`censi_y`が0.00101から最低0.00056まで
   **下がり**(ヒューリスティックを一度も超えない)、Yは常にヒューリスティック
   のまま緩んでいない。
3. 解釈: 旋回中は見える壁面の構成が変わり、X方向の対応点由来の情報量が
   一瞬だけ実際に低下する(これ自体は物理的に妥当な検出)。しかし実共分散は
   これを検出して「まさにIMU予測がX方向にドリフトし始めるタイミングで」
   壁のX補正を12%緩めてしまい、IMU×30で既に緩めてある予測がその隙に
   さらにドリフトし、smootherが1.5秒後に気づいて戻すまでの間に大きな
   ピークとして現れている、という筋が最も辻褄が合う(ただし因果の完全な
   証明ではなく、タイミングの一致による強い状況証拠)。

## 訂正: 上記の「実共分散とIMU緩和が排他的」という結論は検証手順のミスだった (2026-07-31)

X方向クランプ(`wall_cov_max_loosen_ratio`)を追加してA/Bテストしたところ、
クランプの有無で結果が変わらなかった(ratio=1.0でも22.98mm、無制限でも
22.99mm)。おかしいと考えて`--imu-param`と`--backend-param`の運用を洗い直した
結果、**`accelerometer_sigma`/`gyroscope_sigma`/`integration_sigma`は
`imu_preintegration_node`と`backend_optimizer_node`の両方が同名パラメータを
宣言しており、実際の③因子グラフ融合に使われるのはbackend側の値**である
ところ、本セクション(「実共分散+IMU×30」等)の検証はすべて`--imu-param`
(①の診断用/odom_fastにしか効かない)で渡していたため、③のIMUノイズは
実際には一度も緩んでいなかったことが判明した(18番の元マニフェストを
確認し、`sweep_imuinflate_30x`が`--backend-param`で渡していたことと対比して
特定)。

正しく`--backend-param`で渡して再検証(`realcov_imu30_correctparam_20260731`):

| 条件 | t=15 max[mm] |
|---|---:|
| 18番: IMU×30のみ(旧固定共分散1e-5、backend-param) | 12.06 |
| **本実装の実共分散 + IMU×30(backend-param、正しい配線)** | **11.85** |

**実共分散とIMU緩和は排他的ではなく、正しく組み合わせれば旧結果と同等以上**。
上記のX方向クランプ・「censi_xが旋回中に緩む」という観測自体(wallsigma_diag
の生データ)は事実として残るが、「それがIMU緩和の効果を打ち消す」という
因果の結論は誤りだった(IMU緩和がそもそも③に届いていなかったため、
「打ち消す対象」自体が無かった)。`wall_cov_max_loosen_ratio`パラメータは
実害のない安全弁として残すが、確認された不具合の修正ではなく予防的な
安全マージンという位置づけに変更する。

## 結論と次の一手(2026-07-31 訂正版)

候補2(実共分散)と候補C(IMU緩和)は排他的ではなく、正しく組み合わせれば
**11.85mm**(18番の12.06mmと同等以上)。ただし**5mm目標にはまだ届かない**。
候補1(バイアスσ)・候補2単体の効果・候補Cの機序はいずれも実証済みで、
組み合わせても12mm前後が下限になっている。次の選択肢:

- (a) 候補3(ISAM2 relinearize_threshold)を検証する — 未着手の唯一の主要候補
- (b) X方向の一時的な情報量低下(wallsigma_diagで観測、censi_xが旋回中の
  0.35秒だけヒューリスティックの1.12倍まで上昇)がなぜ起きるかを②側で
  直接確認する(実害は今のところ確認されていないが、原理的な弱点ではある)
- (c) `wall_prior_base_yaw_sigma=0.15`+IMU×30をyamlへ正式反映し、5mm未達を
  前提に別の切り分け軸(smoother_lag_sec等)を探す

いずれも未着手。ユーザー判断待ち。

## 候補3: ISAM2 relinearize_thresholdの検証(2026-07-31、除外)

正しい配線(11.85mmのbackend-param構成)の上で`relinearize_threshold`を
0.001/0.01(既定)/0.03/0.1と100倍のレンジで振ったが、t=15 maxは全て
**11.85mmで完全に同一**だった。2026-07-28の別文脈での既存の知見
(0.001への締め込みで改善なし)と整合し、旋回時の過渡誤差についても
候補3は下限要因ではないと確定した。

これでバイアスσ(候補1)・②の実共分散(候補2)・ISAM2 relinearize閾値
(候補3)の3候補すべてが「効果なし、または既にIMU緩和+実共分散に
組み込み済み」で出尽くした。5mm目標(11.85mm、未達)に対して、これ以上の
既知候補が無い状態。次の方向性はユーザー判断が必要
(未探索の軸: smoother_lag_sec、DCS/imu_dynamicsゲートの旋回時挙動、
または17番で一度「コスト対効果が合わない」と判断した密結合化の再検討等)。

## 対応状態

- `lio_localization/src/laser_scan_matching_node.cpp`変更・ビルド済み(未コミット)
- `lio_localization/include/lio_localization/backend_optimizer_node.hpp`・
  `src/backend_optimizer_node.cpp`に`diag_wall_sigma_path`診断機能と
  `wall_cov_max_loosen_ratio`(既定1e6=実質無制限、安全弁として保持)を追加・
  ビルド済み(いずれも既定で無効/無変化、未コミット)
- 実験生成物: `docs/stage4_runs/realcov_baseline_20260730/`、
  `docs/stage4_runs/realcov_plus_imu30_20260730/`、
  `docs/stage4_runs/wallsigma_diag_realcov_imu30_20260730/`、
  `docs/stage4_runs/realcov_imu30_correctparam_20260731/`(いずれも未コミット)
- yamlへのσ変更は未反映(検証時の`--backend-param`一時上書きのみ)
- **教訓**: `accelerometer_sigma`/`gyroscope_sigma`/`integration_sigma`は
  `imu_preintegration_node`と`backend_optimizer_node`の両方が同名パラメータを
  持つ。③の融合結果に効かせたい場合は必ず`--backend-param`で渡すこと
  (`--imu-param`は①の診断用/odom_fastにしか効かない)。
