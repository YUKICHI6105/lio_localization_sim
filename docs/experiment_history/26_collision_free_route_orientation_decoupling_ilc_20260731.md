# 実験履歴: 衝突ゼロ経路・向き分離・ILC安定化 (2026-07-31)

25番(スプライン投資中止・ILC優先化)で構築したILCパイプラインの検証中、往路が毎ラップ
100%Shelf_0/Post_2に衝突しておりILCが往路では一度も学習機会を得られていないことが判明した。
これを受けてユーザーから3点の指示があった:

1. 衝突は「起きるほど悪化する」ため、衝突したら直ちに学習を止め、その場でやり直すべき。
2. まず絶対に衝突しない基準経路を用意するのが先決。
3. 無駄に回転しないこと。ロボットは3輪オムニホイール(全方向移動)のため、機体の向きと
   進行方向は独立にできる。特定の1辺を事前に決め、その辺がノーツ側→ビンゴ側に向くよう
   制御し、スラローム中の姿勢に制約は設けない。スラロームwaypointでの速度0条件も廃止する
   べき(mission JSONの`required_zero_speed_points`はstart/pickup/finishのみ)。

## 実施内容(`StartToBingoValidation.cs`)

### 1. 向き制御の再設計
`segmentYawEnd`を各区間の進行方向から算出するのをやめ、`notesFacingYawRad`/
`bingoFacingYawRad`の2値のみを目標ヨーとした。往路は最終区間(slalom2→finish)でノーツ側→
ビンゴ側へ回転し、それ以外は向きを一切変えない(ヨーレート0、`segmentYawEnd[i]`が直前と
同じ値ならEvaluateRoute内でyawDelta=0になる)。復路はその逆(最終区間でビンゴ側→ノーツ側)。

### 2. 速度プロファイルの再設計(スラローム経由点で止まらない)
required_zero_speed_pointsに該当しないwaypointでは完全停止せず、通過速度
(`CornerSpeedFraction=0.2`×maxSpeed)で通過する。経路の形(直線waypoint列)は変えず、
`MinimumJerk`を一般化した`QuinticBlend(u,v0,v1,...)`(境界条件p(0)=0,p(1)=1,p'(0)=v0,
p'(1)=v1,p''(0)=p''(1)=0の5次多項式、v0=v1=0で旧MinimumJerkと完全一致)でタイミングのみ
変更した。

### 3. 衝突検知→即時中断・同一方向リトライ
`CollisionRecorder.ClearCollisions()`を追加(既存の`collisionNames`がPlay開始からの
累積でラップ単位にリセットされておらず、一度衝突するとILCの学習禁止ゲートが以後
永久に発火し続けるバグを併せて修正)。`FixedUpdate`で衝突を検知した瞬間、ILCの今トライアル分の
誤差蓄積を破棄し、ロボットを現在の走行方向の開始状態へ瞬間的にリセットして同方向で
やり直す`AbortAndRetryDueToCollision()`を実装。

### 4. 実測クリアランス診断によるwaypoint/向きの調整
`Unity_RunCommand`+`Physics.ComputePenetration`で、finish地点でのShelf_0/Post_2との
重なりをヨー2度刻みで走査した。旧実装の「最終区間の進行方向」由来のヨー(≈16度)は
重なり境界(≈19度)からわずか3度しか離れておらず余裕が無かった。101〜138度が両方とも
重ならないはるかに広い安全域だったため、中央値の120度を`bingoFacingYawRad`として採用
(実機の排出機構の向きが決まるまでの暫定値)。あわせて、baffle_left_2の復路横断地点は
新しい向き(ヨーを保持したまま通過)であれば0〜150度のどの向きでも重ならないことを確認。
crossLow/bypass地点のbaffle_left_1に対するクリアランス、notesFacingYawRadでの
slalom0/slalom1のbaffle_left_1に対するクリアランスもあわせて実測確認した。

## 発見: ILCのpickup dwell hold中feedforwardによる発散(検証中に発見・修正)

上記1〜4適用後の初回検証run(10ラップ)で、pickup_errorがラップを追うごとに
0.29→1.23→2.56→3.73→4.74mmと単調悪化し、11ラップ目でpickup到達ゲート(5mm)を超えて
ミッションが完全に停止(位置・速度ゼロのままFinished=Falseで無限待機)する新たな不具合を
発見した。

原因: pickup到達後のdwell hold中もILCのfeedforwardをかけ続けていた。この区間は
純P制御(積分項なし、`force=m*(feedforward+60*positionError+12*velocityError)`)による
静止保持であり、平衡状態では`positionError=-feedforward/60`に固定される。つまり
feedforwardを加えるほど、その分だけ位置誤差が生まれる方向に平衡がずれる。ILCの更新則が
この誤差を「まだ足りない」と解釈しfeedforwardをさらに強めることで、収束ではなく発散が
起きていた。

対応: `inPickupDwellHold`(pickup dwell待ち)・`routeDone`(finish到達後のヨー収束待ち)の
どちらかが真の間は、ILCの誤差蓄積・feedforward適用の両方を止める`ilcActiveThisFrame`
ガードを追加した。ILC(経路追従.md §12)は本来「移動中の追従遅れ」を学習対象とするもので
あり、静止保持中の精密停止はそもそも対象外とすべきという設計上の整理でもある。

## 検証結果(修正後、Unity単体・ROSなし、22ラップ連続)

```
lap=0  ilc_update=1  pickup_error=0.29mm tracking_rmse=41.88mm collisions=
lap=10 ilc_update=11 pickup_error=0.33mm tracking_rmse=37.47mm collisions=
lap=21 ilc_update=22 pickup_error=0.39mm tracking_rmse=36.08mm collisions=
```

- 22ラップ全てで`collisions=`欄が空、`collision_abort_total=0`(衝突ゼロを維持)。
- `ilc_update`が毎ラップ(往路・復路とも)増加し、`ilc_skip=0 ilc_rollback=0`
  (25番時点では往路が毎回衝突していたため往路のILCが一度も機能していなかったが、
  今回で初めて往復とも安定して学習が機能することを確認できた)。
- `tracking_rmse`が41.88mm→36.08mmへ緩やかに改善(ILCの効果)。
- `terminal_yaw_error`は0.11〜0.15度で安定(蓄積ドリフトなし、24番で発見した終端ヨー
  タイムアウト機構との相互作用も問題なし)。
- `pickup_error`は0.29mm→0.39mmへごくわずかに増加するが発散せず、5mmゲートに対し
  十分な余裕を保ったまま安定(修正前の発散パターンとは明確に異なる)。
- `max_accel`は5.5〜10mps2程度(24番の衝突由来スパイク350mps2超からも、コーナー速度
  導入によるものと思われる緩やかな増加はあるが常識的な範囲)。

## 追記: notesFacingYawRadの定義誤り(検証後にユーザー指摘・修正)

上記検証後、ユーザーから「スタート地点への帰還時の姿勢がノーツ側に向いていない」との
指摘があった。数値検証(`body.transform.forward`を直接確認)したところ、帰還時のヨーは
`notesFacingYawRad`(当時57.19度)に正確に到達しており、コードは設計通り動作していた。
しかし設計そのものに誤りがあった: `notesFacingYawRad`を「旧実装のsegment 0(start→pickup)の
進行方向」から流用していたが、これは「ロボットがpickupへ移動する向き」であって「pickup地点で
回収対象ノーツが実際にどちらにあるか」ではない。ユーザーへ確認したところ
「ノーツを回収する辺の定義をよく確認してほしい」との回答を得た。

field JSON実測: pickup=(-1.799, 0.394337567)、回収対象ノーツ(`pickup_note_indices=[4,5]`、
Orange)は(-1.899,0.125)と(-1.699,0.125)で、中間点は(-1.799,0.125)。これはpickupの
ちょうど真南(field -y方向)にあり、start→pickup方向(≈57度)とは全く異なる(正しい向きは
90度)。オムニホイールで並進とヨーが独立にできる以上、「回収辺が向くべき方向」は
「ロボットの移動経路」ではなく「pickup地点から回収対象ノーツそのものへの方向」で
定義すべきだった。

`notesFacingYawRad`を`Atan2(-(noteMidpoint.y-pickup.y), noteMidpoint.x-pickup.x)`
(=90度)に修正。`BuildRoute()`に`orangeNotes`(`FieldDefinition.Notes.Orange`)を渡し、
`mission.PickupNoteIndices`で参照する2点の中間点を使う。修正後、slalom0/slalom1/pickup
地点でのbaffle_left_1に対するクリアランスを実測クリアランス診断で再確認(重なりなし)、
8ラップの動的検証でも衝突ゼロ・ILC学習継続・pickup収束(0.29〜0.32mm)を確認した。

## 追記2: ILC学習の永続化・初期スポーン姿勢の修正

上記までの検証はすべて単一Play session内で完結しており、学習テーブル
(`ilcCorrectionOutbound`/`ilcCorrectionReturn`等)はメモリ上にしか無くPlayを止めると
消えていた。ユーザーから「それが一番大事」との指摘を受け、永続化を追加した。

- `Application.persistentDataPath`(Unity標準の書き込み可能領域、Assets外・git管理対象外)
  へ`ilc_learning_state.json`としてラップ完了ごとに保存(`SaveIlcState()`)、`Start()`で
  読み込み(`LoadIlcState()`)。保存内容: 往路・復路の補正テーブル、ロールバック用の
  最良テーブルとRMSE、`IlcUpdateCount`/`IlcSkippedCount`/`IlcRollbackCount`。
- プロジェクト直下に`ResetIlcLearning.flag`という名前の空ファイルを置くと、保存済み状態を
  無視して(削除して)まっさらから再学習する(既存の`DisableNotesExperimentFlag`と同じ
  フラグファイル方式)。
- 検証: 1回目のPlay session(3ラップ、`ilc_update`が1→2→3→5まで進行、バックグラウンドで
  さらに進行)でファイル生成を確認、Stop→再Playで`[StartToBingo] ILC: loaded saved state
  ...(update_count=5)`のログが出て`ilc_update`が6→7→8→9と**リセットされず連続**、
  `tracking_rmse`も前回session終盤の改善傾向をそのまま引き継ぐことを確認した。

また、ユーザーから「起動直後、大きく旋回してノーツに頂点を突っ込むように見える」との
指摘があった。原因は機体のスポーン姿勢が単位回転(0度)のまま放置されており、区間0
(start→pickup)の開始で0度→notesFacingYawRad(90度)まで大きく旋回しながら並進していた
ため。`BuildRobot()`でスポーン時の`robot.rotation`を`notesFacingYawRad`に設定し、
`BuildRoute()`で`startYawRad`も同じ値に初期化することで、区間0は並進のみ(無回転)になる
よう修正した。実機投入後、start地点で機体を手動でこの向きに合わせて配置する運用を想定。

## 追記3: notesFacingYawRad/bingoFacingYawRadの計算式自体の誤り(forward基準→平らな辺基準)

追記1の修正後もユーザーから「ノーツ手前姿勢と初期姿勢がおかしい」との指摘があった。
`CreateRoundedTrianglePrism`のコーナー配列(`(inradius,-side/2)`,`(inradius,side/2)`,
`(-circumradius,0)`)を確認すると、シャシーの平らな辺はlocal+X側にあり、
`transform.forward`(local+Z)とは90度ずれている。これまでの`notesFacingYawRad`/
`bingoFacingYawRad`の式(`Atan2(-delta.y,delta.x)`)は「forwardが目標方向を向く」角度を
計算しており、「決めた1辺(平らな辺)」が向く角度ではなかった。

MeshColliderの実頂点を`transform.TransformVector`で変換して実測したところ(手計算の
行列符号を一度間違えたため、必ず実測で検証した):
- 平らな辺(local+X)は、回転θのとき world`(cosθ,0,-sinθ)`を向く(forwardの
  `(sinθ,0,cosθ)`とは90度ずれ)。
- 「平らな辺が方向Dを向く」ために必要なθは`Atan2(-Dz,Dx)`(forwardがDを向く式
  `Atan2(Dx,Dz)`からちょうど90度引いた値)。
- フィールド座標のdelta(dx,dy)に対する正しい式は`Atan2(-delta.x,-delta.y)`
  (旧式`Atan2(-delta.y,delta.x)`から90度引いたもの)。

この式で再計算すると、pickup→ノーツ中間点も、finish→ビンゴ棚中心(`bingo.Centre`)も
どちらも**真南**(field -y方向)で、必要な角度は**どちらも0度**(=スポーン姿勢のまま
無回転)になった。実測(Physics.ComputePenetration)でfinish/Shelf_0・Post_2、
baffle_left_1(slalom0/slalom1/pickup/crossLow/bypass)、baffle_left_2(復路横断点)の
全チェックポイントを0度で確認し、重なり無し。動的検証(11ラップ)でも
`terminal_yaw_error=0.00deg`(一切回転なし)、衝突ゼロ、ILC学習(保存済みupdate_count=19
から継続)を確認した。

以前の暫定値(notesFacingYawRad≈90度、bingoFacingYawRad=120度)は、forward基準の
誤った定義のまま「衝突さえ避ければいい」という基準で選んだ値だったため、たまたま
衝突は避けられていたが、実際に平らな辺がノーツ/ビンゴを向いてはいなかった。

`BuildRoute()`は`bingo.Centre`を参照するため、`Start()`から
`definition.Bingo`も渡すよう変更した(`BuildRoute(orangeNotes, bingo)`)。

## 対応状態

- `StartToBingoValidation.cs`: 向き制御・速度プロファイル・衝突中断リトライ・
  ILCホールド中ガードの全てを適用、ライブUnityプロジェクトへ同期・コンパイル確認・
  22ラップPlay mode検証済み(未コミット)。
- `bingoFacingYawRad=120度`は実機の排出機構位置が決まるまでの暫定値である旨をコード
  コメントに明記済み。
- ROS連携時、衝突検知テレポートが自己位置推定へ与える影響は今回未検証(Unity単体での
  検証のみ)。ROS連携運用時に別途確認が必要。
- `notesFacingYawRad=90度`(pickup→回収対象ノーツ中間点への方向、追記参照)、
  `bingoFacingYawRad=120度`はどちらも実機設計が固まるまでの暫定値。
- ILC学習状態は`Application.persistentDataPath`(Windows実機では
  `%USERPROFILE%\AppData\LocalLow\DefaultCompany\Robocon2026Sim\ilc_learning_state.json`)
  へラップごとに自動保存・起動時に自動復元される。ユーザーがUnity Editorの▶ボタンで
  Playを開始・停止するだけで、何セッションにも分けて学習を積み重ねられる。まっさらから
  やり直したい場合はプロジェクト直下(`Assets`と同じ階層)に`ResetIlcLearning.flag`
  という名前の空ファイルを置く。
