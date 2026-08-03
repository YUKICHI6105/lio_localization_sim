# 実験履歴: スプライン投資の中止とILC優先への方針転換 (2026-07-31)

24番のPhase 0基準計測に続き、`docs/requirements/経路追従.md`のPhase 1(C¹/C²スプライン+
forward-backward速度計画によるwaypoint間の不要な停止解消)を実装したが、3件の独立した
バグに連続して遭遇したため、ユーザー判断によりスプライン投資自体を中止し既存実装へ復元した。

## Phase 1実装中に遭遇した問題(時系列)

1. **自然3次スプラインの激しいオーバーシュート**: スラロームのジグザグ状waypoint配置で
   実測2m超逸脱し、中央壁(centre_to_bingo)へ衝突。自然3次スプラインは全区間の曲率を
   大域的に最小化するため、急な向き反転が連続する形状で局所的な逸脱を起こしやすいという
   既知の弱点を実際に踏んだ。
2. **ヨー到着判定の無限ハング**: 「到着判定が位置・速度のみでヨーを見ていない」という
   Phase 0の発見(24番)を受けてヨー誤差・ヨーレートの条件を追加したところ、
   open-loopヨー制御(`body.angularVelocity`への直接代入)の残差がフィードバック無しでは
   決して閉じず、ミッションが無限に停止し続けた。タイムアウト機構で応急対応。
3. **Catmull-Romスプラインへ置き換え後も経路途中で完全停止**: オーバーシュートは解消した
   (衝突が3件→1件に減少)が、pickup/finish以外の地点でロボットが完全静止(速度・
   角速度ともゼロ)したまま進まなくなる新たな不具合が発生。原因未特定のまま中断。

## ユーザーによる方針転換

3つ目の問題を報告した際、ユーザーから次の指摘があった:

> スプラインより最終的にILCで実機調整するので実装計画を再策定してもらってもいいですか？
> スプラインやっていても意味がないと思うので

これは合理的な指摘である。ILC(反復学習制御、経路追従.md §12)は「与えられた参照軌道に
対して、毎回同じ場所で出る再現性のある実機誤差(モータ個体差・摩擦の方向依存・遅延など)を
学習して補正する」仕組みであり、参照軌道そのものが曲率最適である必要はない。実機データが
存在しない現時点でシミュレーション内の経路形状を数学的に追い込む労力は、投資対効果が低い。

## 決定・実施内容

1. **`StartToBingoValidation.cs`をPhase 1着手前の状態へ完全復元**。
   `PathSample`/`CatmullRomSpline1D`/`leg1Samples`/`leg2Samples`/`BuildPath()`/
   `BuildLegPath()`/`SampleLeg()`を削除し、`segmentDurations`/`segmentYawEnd`/
   `segmentSpinDurations`/`BuildTimeline()`/`MinimumJerk()`/元の`EvaluateRoute()`
   (waypoint単位の独立5次多項式)を復元した。
2. **経路形状に依存しない今日の発見・修正だけは維持**:
   - 到着判定へのヨー誤差・ヨーレート条件の追加(位置5mm・速度10mm/sに加えて
     ヨー誤差2°・ヨーレート5°/s、タイムアウト2秒で無限ハングを回避)
   - `actualYawRad`の符号規約修正(`Atan2(forward.x, forward.z)`。`FieldYaw()`
     (`UnityRosSensorPublisher.cs`、ROS配信用の符号反転規約)と混同していたバグを解消)
   - Phase 0計装(`trackingRmse`/`trackingMax`/`maxAccelerationMagnitude`/
     `TerminalYawErrorDegrees`、`Debug.LogWarning`によるMCP経由回収)
   - ノーツ形状変更(ボール→150mm立方体)は別件のためそのまま維持
3. **Phase 1・2(結合加速度制約)・6(direct collocation)は中止**。実機テストで経路形状
   自体がボトルネックだと判明した場合の将来オプションとして`docs/requirements/経路追従.md`
   には手を加えず残す。

## 検証結果(復元後、Unity単体・ROSなし)

```
[StartToBingo RESULT lap=0] final=(2.316,0.494)m completion_time=7.608s
target_error=0.3mm terminal_yaw_error=0.07deg tracking_rmse=52.79mm
tracking_max=152.56mm max_accel=36.65mps2 wheel_odom_rmse=0.00mm
wheel_odom_max=0.00mm pickup_error=0.29mm pickup_speed=9.93mm/s
notes_captured=True collisions=Shelf_0,Post_2
```

正常に1ラップ完走。特筆すべきは**terminal_yaw_error=0.07°**(Phase 1のCatmull-Rom版で
観測した32〜47°、タイムアウト発動時の12.67°と比べ劇的に良好)。旧方式のwaypoint単位の
ヨー補間(`segmentYawEnd`ベース)は、経路形状から幾何学的に導出するPhase 1方式より
実際の物理積分との整合性がはるかに高く、ヨー到着条件を厳しいまま(2°)満たせている。
tracking_rmse/max_accelもPhase 0基準(24番、65mm/370mps2程度)と同水準の常識的な範囲。

## 今後の優先順位

経路形状に依存せずILCへの橋渡しになる部分を優先する。**訂正(同日)**: 実機は用意できて
おらず、ILCはシミュレーション上で行う。Phase 7(実機モデル同定)は実機投入後の将来課題
として温存し、当面はUnityシミュレーション自体が持つ再現性のある誤差(PDサーボの追従遅れ・
衝突外乱・物理エンジンの数値誤差など)を学習対象にPhase 9(ILC)のパイプライン(試行ログ・
安全ゲート・feedforward更新・ロールバック)をシミュレーションで先に検証する方針とする。
- 自動評価ログの充実(Phase 0計装を土台に、経路追従.md §15のTrialMetrics相当へ拡張)
- Phase 9(ILC)のシミュレーション内パイプライン構築
- Phase 3(加速度feedforward)・Phase 4(位相同期)・Phase 5(精密停止)は経路形状に
  依存しない安価な改善だが、今回は復元・安定化を優先し次のステップとしてユーザーと相談する

## 対応状態

- `StartToBingoValidation.cs`: Phase 1着手前へ復元+ヨー到着条件/符号修正/Phase 0計装を
  再適用。ライブUnityプロジェクトへ同期・コンパイル確認・Play mode動作確認済み(未コミット)
- `docs/requirements/経路追従.md`: 変更なし(Phase 1・2・6は将来オプションとして温存)
- プランファイル(`.claude/plans/synchronous-jumping-sunset.md`)を本方針転換の内容で更新済み
