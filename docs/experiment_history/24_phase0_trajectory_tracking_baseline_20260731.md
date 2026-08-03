# 実験履歴: 経路追従アップグレードのPhase 0基準計測 (2026-07-31)

`docs/requirements/経路追従.md`(最小時間プリセット軌道生成・追従システム)の
Phase 0(現行実装の基準計測・回帰テスト)を実施した。実装は一切変更せず、
`StartToBingoValidation.cs`に計測用の計装(追跡誤差・実加速度・終端ヨー誤差)を
追加しただけで、既存のwaypoint+quintic minimum-jerk+Cartesian PDの挙動を測った。

## 前提: ノーツ形状ルール変更

同日、競技ルール変更(ノーツが直径150mmの球→一辺150mmの立方体、質量200g不変)に
対応。Unity側(`RoboconFieldBuilder.cs`のSphere→Cube、`FieldDefinition.cs`の
`NoteDiameter`→`NoteSize`、`config/robocon2026_field.json`の`note_diameter`→
`note_size`)を対応済み・Play modeで実機確認済み。`laser_scan_matching_node.cpp`の
円フィットベースのノーツ検出(`/ball_candidates`)は球形前提のため機能しなくなるが、
自己位置推定の安全経路には無いため今回は据え置き(コード内にコメント済み)。

## 計装の追加

`StartToBingoValidation.cs`に以下を追加(既存の`FixedUpdate`/`Complete`のロジックは
変更せず、並行して計測するだけ):

- `trackingRmse`/`trackingMax`: 参照軌道(target)と真値(truth)の差(制御追従精度)。
  既存の`odometryError`(単純デッドレコニング自己検算)や自己位置推定誤差
  (state_estimate/odom_fast vs ground truth)とは別の第三の指標。
- `maxAccelerationMagnitude`: 速度差分から実測した実加速度(指令力ではなく)。
- `terminalYawErrorDegrees`: 到達時の実ヨー(`body.transform.forward`から
  `UnityRosSensorPublisher.FieldYaw()`と同じ規約`atan2(-forward.x, forward.z)`で
  算出、既に検証済みの変換を再利用)と目標ヨー(`segmentYawEnd[^1]`)の差。

`Debug.Log`ではなく`Debug.LogWarning`を使った。理由は次項。

## 発見1: Unity MCPの`Unity_GetConsoleLogs`は`Debug.Log`(info)を拾わない

`logTypes`に`"log"`を明示指定しても`Debug.Log`メッセージが一切返らないことを
実測で確認した(`Debug.LogWarning`/`Debug.LogError`は正常に捕捉される)。
Unity実体の`Editor.log`(`%LOCALAPPDATA%\Unity\Editor\Editor.log`)から直接読む
代替も試したが、このMCP駆動のUnityインスタンスでは同ファイルが更新されておらず
使えなかった。初回の計測run(`phase0_baseline_20260731`)はこれに気づく前に実施し、
`[StartToBingo RESULT`行が一つも捕捉できず「ミッションが一度も完走していない」
ように見えた。`Debug.LogWarning`へ切り替えて再計測(`phase0_baseline_v2_20260731`)
したところ11ラップ全て完走を確認でき、v1の「未完走」は誤りだったと判明した
(実際には完走していたが、ログが捕捉できていなかっただけ)。
`unity-stage4-benchmark-operations`スキルに記録済み。

## 発見2: 毎ラップ、Shelf_0/Post_2/baffle_left_2へ衝突している

11ラップ全てで衝突が発生した(衝突ありラップ数 11/11)。相対速度0.13〜1.49m/s。
現行の固定waypoint経路(スラローム通過後、ビンゴ前で停止するルート)が、
ビンゴ棚構造(Shelf_0/Post_2)とスラロームバッフル(baffle_left_2)の実際の
コリジョン形状に対して十分なクリアランスを取れていない、既存のベースライン route
自体の設計上の問題であることが今回の計測で初めて定量的に確認された。
`docs/start_to_bingo_baseline_and_algorithm_survey.md`の最終ベースラインtest
(2026-07-23時点、衝突0件と報告)とは条件が異なる(あちらは
`IgnoreNoteContactForLocalizationBaseline`でノーツ接触のみ無効化、壁・棚・
バッフルとの接触は元々有効。おそらく当時はまだ`RepeatLaps`が無効・単発走行
だったため、この繰り返しラップ特有の衝突箇所を踏んでいなかったと推測される)。

## 発見3: 終端ヨー誤差が32〜47°と大きい(バグではなく実在するギャップ)

終端ヨー誤差の計測値が想定より大きかったため検証した。原因は測定バグではなく、
**現行の到達判定(`Complete`呼び出し条件)が位置誤差・速度のみをチェックしており、
ヨー誤差・ヨーレートを一切チェックしていない**ことだった。

検証手順:
- `MissionDefinition.Yaw`(field JSON `mission.yaw=0.0`)という「固定目標ヨー」
  フィールドが存在するが、`StartToBingoValidation.cs`のどこからも参照されて
  いない(未配線)。したがって現行実装が実際に追従しようとしている目標ヨーは
  `segmentYawEnd[^1]`(最終区間の進行方向)のみであり、計測の比較対象として
  これを使ったのは正しい。
- `FixedUpdate`の到達条件は`if (routeDone && finalError <= 0.005f &&
  finalPlanarSpeed <= 0.010f) Complete(truth);`であり、ヨー関連の条件が無い。

つまり、位置が収束した瞬間に(まだ目標ヨーへ回転しきっていない・回転中でも)
到達と判定してしまう。これは`docs/requirements/経路追従.md` §11「精密停止
モード」が到達判定にヨー誤差・ヨーレートの条件を追加すべきと明記している
まさにその問題であり、今回の計測でその必要性が定量的に裏付けられた形。

## 発見4: max_accel(最大370.8 m/s²)は衝突由来のスパイクと推定

指令力は`m*4.905`(0.5G)でクランプされているため、通常運転でこの値を超えることは
無いはずだが、実測は最大370.8 m/s²。衝突ラップ(11/11)と完全に相関しており、
物理衝突による瞬間的な速度変化(反発)を実加速度として拾ったものと推定される。
コントローラの指令加速度違反ではなく、発見2の衝突自体が原因と考えられる。

## 計測結果サマリー(11ラップ、`phase0_baseline_v2_20260731`)

| 指標 | 平均 | 最小 | 最大 |
|---|---:|---:|---:|
| completion_time | 7.130s | 6.753s | 7.614s |
| target_error | 2.118mm | 0.100mm | 2.900mm |
| terminal_yaw_error | 38.859deg | 31.910deg | 46.920deg |
| tracking_rmse(参照軌道vs真値) | 65.414mm | 59.110mm | 72.540mm |
| tracking_max | 208.100mm | 208.100mm | 208.100mm |
| max_accel | 351.121mps2 | 154.330mps2 | 370.800mps2 |

自己位置推定精度(対ground_truth、参考): state_estimate 位置誤差 mean=2.30mm
RMSE=3.32mm max=11.22mm、odom_fast mean=2.34mm RMSE=3.32mm max=16.07mm。
要件A(定常RMSE≤10mm)は満たすが、max位置誤差はstate_estimate/odom_fastとも
ぎりぎり超過(11.22mm/16.07mm、要件10mm)。

旧基準(経路追従.md §19記載の「6.53s」)との比較は参考情報に留めた:
`docs/start_to_bingo_baseline_and_algorithm_survey.md`の最終ベースラインtestの
表にはcompletion timeの実測値が無く("no measured completion timestamp"と
明記)、6.53sの出典は経路追従.md本文にのみ存在し実測検証できていないため。

## 成果物

- `tools/phase0_baseline_eval.py`(新規): 上記全指標を自動集計・レポート化する
  スクリプト。`docs/stage4_runs/<run>/`の`consolidated/*.csv`
  (`consolidate_stage_log.py`の出力)とUnityコンソールログ(手動保存、下記
  「未解決」参照)を入力に取る。
- `docs/stage4_runs/phase0_baseline_v2_20260731/phase0_baseline_report.txt`:
  本runの完全なレポート。
- `docs/stage4_runs/phase0_baseline_20260731/`: 初回計測(コンソールログ
  未捕捉のため参考値のみ、`consolidated/`は正常)。

## 未解決・今後の課題

- **Unityコンソール出力を自動でファイル保存する仕組みが無い**。現状は
  `Unity_GetConsoleLogs`で手動取得してテキストファイルへ保存している。
  `stage4_mcp_run.sh`等のワークフローに組み込む改善が必要(スコープ外として
  今回は据え置き)。
- 発見2(衝突)・発見3(ヨー到達判定)は、経路追従.md自体のPhase 1〜5計画で
  扱われる予定(Phase 1は経路形状のみ、Phase 5「精密停止モード」でヨー到達
  条件を追加)。今回はあくまで現状の定量記録。
