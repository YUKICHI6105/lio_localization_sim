# 実験履歴: start/pickup/bingo共通到着点を結ぶ15区間の学習用ミッション化 (2026-07-31)

26番(衝突ゼロ経路・向き分離・ILC安定化)の後、ユーザーから「実際の運用は1往復ではなく、
ノーツ6個を2個ずつ5通りの組み合わせのどこからでも回収でき、ビンゴへの往復もstartを
経由せず直接行う」との指摘があり、経路構造を「start↔pickup(固定1組)↔bingo」の単一
往復から、3種類×5組=15パターンのpoint-to-pointミッションへ再設計した。

## 決定事項(ユーザーとの合意)

1. ノーツ6個を2個ずつ回収する5組([1,2][2,3][3,4][4,5][5,6]、1-indexed、隣接組は
   ノーツを共有するため実戦で連続して全部やるわけではない)は、それぞれ独立した
   学習用ミッションとして扱う(1回の連鎖した実戦フローではない)。
2. 学習対象は次の3種類×5組=15パターン: (a)start→pickup_i(リスタート用)、
   (b)pickup_i→bingo共通到着点、(c)bingo共通到着点→pickup_i。「bingo→start」は
   対象外。
3. 各legの終点(start/pickup_i/bingo共通到着点)では、既存のpickup待機と同じ基準
   (位置誤差5mm・速度10mm/s以下)で必ず静止してから次のlegへ進む。pickup_iでの
   静止時は既存の`CaptureNotes()`、bingo共通到着点での静止時は新設した
   `DeliverNotes()`を実行する(どちらもplaceholder、静止後`pickup_dwell_s`秒だけ
   待機してから次legへ進む=実機の吸引・投入機構の動作時間を模す)。
4. ビンゴ棚内の特定スロット・左右アームへの位置合わせ(ビンゴ共通到着点からさらに
   細かく移動する区間)は、スロット座標・アーム可動範囲のデータが無いため今回は
   スコープ外(データが揃い次第、将来のフェーズとして追加)。

## 実装

- `config/robocon2026_field.json`: `start_to_bingo_pair_1_2`/`2_3`/`3_4`/`4_5`を追加
  (既存`start_to_bingo_left`=組5-6は変更せず)。`start`/`slalom_waypoints`/`finish`/
  `pickup_dwell_s`は5ミッション共通、`pickup`は対象2ノーツの中間点。
- `FieldDefinition.cs`: `MissionsDefinition`に4フィールド追加、5つをまとめた
  `AllPairMissions`プロパティを追加。`Validate()`も5ミッション対応に拡張。
- `StartToBingoValidation.cs`を全面的にleg(区間)ベースへ再設計:
  - `LegType`(StartToPickup/PickupToBingo/BingoToPickup)と`currentPairIndex`で
    現在のleg・組を表現。`BuildLegRoute()`が3種類のwaypoint列を生成する
    (bingo→pickup_iはbypassのxを対象pickup_iに合わせて可変にした、旧「復路」の
    一般化)。
  - ヨーは`YawForLegEndpoint(isOrigin)`で起点/終点それぞれnotesFacingYawRad/
    bingoFacingYawRadを返すよう一般化(値自体は26番から変わらず両方0度)。
  - 到着判定(位置5mm・速度10mm/s・ヨー2度or タイムアウト2秒)は全leg共通のまま、
    新たに`pickup_dwell_s`(0.4秒)だけ追加で静止を維持してから完了させる
    `dwellSatisfied`条件を追加。これにより旧実装のタイムライン内蔵の
    pickup待機スケジュール(`EvaluateRoute`のi==1特殊分岐)が不要になり削除できた。
  - ILC学習テーブル・ベストテーブル・RMSE・カウンタを固定長配列から
    `Dictionary<string, ...>`(legIdキー)へ一般化。永続化フォーマットも
    `Dictionary<string, LegIlcState>`ベースに作り直した(スキーマ変更のため
    旧保存ファイルは読み込み失敗時に自動でまっさらから学習にフォールバックする、
    移行コードは書いていない)。
  - **legの巡回スケジュール**: 「bingo↔pickup_i」をi=0→1→2→3→4→0→...と往復で
    無期限に繰り返すのを基本ループとし、5往復(=pickup_iへの到着5回)ごとに
    `TeleportToStart()`(衝突リトライと同じ「瞬間的に位置・速度・向きをリセット」
    手法の応用)でstartへ戻し、start→pickup_iを1回挟む。`pairCounter`は両方の
    遷移で共有・単調に進めるため、セッションを長く走らせれば5組全てが
    start→pickup_iでも学習される。

## 発見: pair[1,2]のpickup位置がstart_end壁と衝突する

4つの新規pickup位置についてPhysics.ComputePenetrationで実測したところ、
最もstartに近い組(pair[1,2]、ノーツ2個の中間点x=-2.599)のpickup地点が、
start_end壁(x=-2.8025)と2.64cm重なることが判明した(シャシー外接半径0.289mに対し
壁までのクリアランスが0.2035mしかなかったため)。0.03m刻みで走査し、+0.06m
(x=-2.539)まで動かせば余裕を持ってゼロ重なりになることを確認し、field JSONの
pickupを修正した(ノーツ自体の回収対象は変わらず、進入位置だけの調整)。他の4組・
3種類のleg全てで実測クリアランス確認済み(finish/Shelf_0・Post_2、baffle_left_1の
各経由点、復路のbypass/crossLow、start_end壁)。

## 検証結果(16ラップ連続、Unity単体・ROSなし)

```
lap=0  leg=start_to_pickup1   final=(-2.539,0.394)m  collisions= ilc_update=1
lap=9  leg=pickup5_to_bingo   final=(2.316,0.494)m   collisions= ilc_update=1
lap=10 leg=start_to_pickup1   final=(-2.539,0.394)m  collisions= (テレポート後の再訪)
```

- 16ラップ全て衝突ゼロ。`bingo↔pickup_i`のi=1→2→3→4→5巡回、5往復後の
  `start_to_pickup1`再訪(テレポートリセット)が設計通り発生することを確認した。
- 各legのILC更新カウントが独立に増加する(例: `bingo_to_pickup2`が
  lap12時点でilc_update=2、新しく訪れたlegはilc_update=1)ことを確認。
- ロールバック機構(§12.5)もleg単位で正しく発火することを確認
  (`start_to_pickup1`/`pickup1_to_bingo`がlap10/11で`tracking_rmse`悪化により
  ロールバック、`ilc_rollback=1`)。
- 保存された`ilc_learning_state.json`を直接読み出し、10leg分(まだ訪れていない
  5legは未生成、遅延生成方式のため正常)の学習状態が正しい構造で保存されている
  ことを確認した。

## 対応状態

- `config/robocon2026_field.json`・`FieldDefinition.cs`・`StartToBingoValidation.cs`
  全て変更・ライブUnityプロジェクトへ同期・コンパイル確認・16ラップPlay mode検証済み
  (未コミット)。
- スコープ外とした「ビンゴ共通到着点→スロット/アーム別位置合わせ」は、スロット座標・
  アーム可動範囲のデータが揃い次第、別途フェーズとして追加する。

## 追記: スロット位置合わせを追加する際の設計上の注意(最速性)

ユーザーから「今の設計のままビンゴ共通到着点→スロットを別legとして追加すると、
共通到着点で毎回フル停止(位置5mm・速度10mm/s・ヨー収束+`pickup_dwell_s`待機)して
から動き出すことになり、最速にならないのでは」との指摘があった。その通りで、
別legとして追加するのは避けるべき。

現行実装は`IsRequiredZeroSpeedWaypoint`が`route`配列の**最初と最後のインデックスのみ**を
停止必須点とみなす単純なルールになっている(スラローム経由点が止まらず通過できるのは
これが理由)。したがって、スロットデータが揃った時点で「`pickup_i→bingo`とは別の新しい
leg」を追加するのではなく、**同じ`pickup_i→bingo`legのwaypoint列の末尾へ、共通到着点
(この時点で自動的に停止不要な通過点になる)→実際のスロット位置(新しい終点)を追加する**
形で拡張する。これによりleg構造・停止判定ロジックを変更せずに、共通到着点を経由しつつ
最速でスロットまで到達し、実際に止まって配置するのはスロット位置だけ、という動きに
できる。`YawForLegEndpoint`(現在origin/destinationの2値のみ)も、スロット追加時は
「共通到着点でのヨー」を挟む形に一般化が必要になる可能性がある(向きがスロットごとに
異なる場合)。
