# 第四段階 再開メモ (2026-07-23, Claude セッション)

WSL再起動でセッションが切れるため、途中状態を残す。ファイルは再起動で消えないので、
以下は「何が分かって、何を適用済みで、次に何をするか」の記録。

## 1. 根本原因を特定した(パラメータ調整の問題ではなかった)

引継レポートのRMSE 4m級未達は、**LiDARのスキャン平面が全ての壁より上を通っていた**ため。
幾何を追うと決定的だった。

| 要素 | z範囲 | 根拠 |
|---|---|---|
| 床上面 | 0.000 | `BuildFloor`: 中心 -0.01、厚さ 0.02 |
| 壁(外周・中央・スラローム板) | **0.000〜0.100** | `walls.height=0.1` を中心 z=0.05 で生成 |
| 機体原点 | 0.060 | シャシー角柱は原点中心(±height/2=±0.06)、摩擦0で床に直接静止 |
| **LiDARスキャン平面** | **0.140** | 機体原点 + ローカル 0.08 |

→ **壁の上端(0.100)より40mm上**を走査していたので、外周壁・中央壁・スラローム板が
1点も映らない。見えるのはビンゴ支柱(z=0.04〜0.90)だけで、ROS側yamlに「bingo posts」
4本が足されていたのはその帳尻合わせ。

これで観測症状が全て説明できる:
- 4本の細い支柱は同方位に固まり拘束がほぼ縮退 → `wall correction unavailable` /
  `rejected by quality check`
- 補正が続かず IMU デッドレコニングで数mドリフト(「5秒までは1.31mm、その後ドリフト」と一致)
- 閾値を緩めると支柱へ誤対応して発散(第2組の29m)
- 正しい注入バイアスを与えても悪化 → バイアスの問題ではなかったので当然

## 2. 適用済みの修正

ユーザー判断「機体設計が未確定なので今は壁を高くしてよい。完成したらルールに合わせる」に従い、
**壁を高くする**方向で対処した(LiDAR z=0.14m は高さ0.12mシャシーの直上20mmで、機体設計
としては妥当な位置のため)。

- `robocon2026_field.json`: `walls.height` 0.1 → **0.3**。`height_rule_value: 0.100` と
  復帰手順を注記として残置。**3コピー全て同期済み**
  (`src/lio_localization_sim/config/`, `unity/.../StreamingAssets/`, Windows実プロジェクト)
- `StartToBingoValidation.cs` / `UnityRosSensorPublisher.cs`: 「スキャン平面は壁上端より
  下でなければ壁が全く見えない」制約をコメントで明記、フォールバック経路が同じ罠を
  再現しないよう修正。**Windows側へ同期済み**
- ROSワークスペースはビルド通過済み

**未検証**: この修正の効果はまだ測れていない(Unityクラッシュ→PC再起動が挟まったため)。

## 3. 自律検証の経路(MCP)

- 公式Unity MCP(`com.unity.ai.assistant` 同梱)を Claude Code に登録済み:
  `claude mcp add unity-mcp --scope local -- /mnt/c/Users/kouza.FUKU-PC/.unity/relay/relay_win.exe --mcp`
  (2026-07-27時点の追記: このマシンのWindowsプロファイル名は`kouza.FUKU-PC`ではなく
  `kouza`だった。正しいパスは
  `/mnt/c/Users/kouza/.unity/relay/relay_win.exe`。同日、この正しいパスで再登録済み)
- 使えるツール7個。特に **`Unity_RunCommand`(EditorでC#をコンパイル・実行)** があるので
  Play Mode制御も診断も可能。他に `Unity_GetConsoleLogs`, `Unity_Camera_Capture` 等
- `src/lio_localization_sim/scripts/unity_mcp.py` … セッション再起動を待たずシェルからMCPを叩くヘルパー
  (stdinを開いたまま応答を待つ必要がある。閉じるとリレーがUnity接続前に終了する)
- **残る関門: Unity側の接続承認**。ツール一覧は取れるが呼び出しは
  `Connection revoked. Go to Unity Editor > Project Settings > AI > Unity MCP to change approval.`
  になる。Editorの **Edit > Project Settings > AI > Unity MCP** で `claude-code` を承認する

### 撤退した方式
Unity をバッチモード(`-runTests -testPlatform PlayMode`)でヘッドレス実行する経路も作ったが
(`src/lio_localization_sim/scripts/stage4_unity_run.sh`, `Assets/Robocon2026/Tests/`)、**2つ目のUnityインスタンスが
メモリ不足でEditorごとクラッシュさせた**ため撤退。バッチ用クローンは削除済み。
このマシン(15.7GB)ではUnityは1つしか動かせない。

## 4. WSLメモリ対処

`.wslconfig` が無く、WSLがホストRAMの50%(7.9GB)まで膨張し解放しない状態だった
(実測 vmmemWSL 2.5〜3.9GB、うち実使用2.0GB・ページキャッシュ1.7GB)。
`C:\Users\kouza\.wslconfig` を新規作成:

```ini
[wsl2]
memory=4GB
autoMemoryReclaim=gradual
swap=4GB
pageReporting=true
[experimental]
sparseVhd=true
```

**次の `wsl --shutdown` / 再起動で有効化される。**

## 5. 次にやること

1. Unity で MCP 接続を承認(上記3)
2. **壁高さ修正の効果を検証**: ROS起動 → Unity Play → RMSE を測る。
   壁点群が大量に入りRMSEが桁で改善するはず(第1〜3段階の実績は RMSE 1〜2mm)
3. 効いたら **`dropout_error_sec: 120.0` を本来値(2.5)へ戻す**。
   これは症状を隠すための暫定措置で、信頼性喪失検知を事実上無効化している
4. 余力があれば真値姿勢での残差診断(引継レポートの最優先項目)を
   `Unity_RunCommand` で実施し、壁マップとUnity幾何の一致を数値で確認

## 6. 注意点(未解決・申し送り)

- **ボール検出はこのLiDAR高さでは機能しない**。ボールは直径150mm(z=0〜0.15)で、
  スキャン平面0.14mでは半径37mm相当の弧にしか映らず、検出帯0.065〜0.085mを外れる。
  自己位置的には外れ値として捨てられるので検証には影響しないが、ボール検出を
  有効化する際はLiDAR高さの見直しが必要
- 内壁(中央壁・スラローム板)のROS側マップは JSON の**中心線**座標をそのまま使っている。
  壁厚33mmなのでLiDARが見る面は中心線から±16.5mmずれる。10mm目標に対しては
  無視できない系統バイアスなので、精度を詰める段階で要検討
- `unity/` と `src/lio_localization_sim/scripts/` は ros2_ws 直下にあり **git管理外**(ros2_ws自体がリポジトリでない)

## 7. Play/Stop連続実行の安定性問題を修正した(2026-07-23 追記)

クローズドループ制御の検証中、複数回Play/Stopを繰り返すと通信(`/clock`配信)が数秒で
停止する不具合が発生し、原因調査の末に修正した。

### 根本原因: ROSConnectionの切断がOnApplicationQuitのみに依存

`com.unity.robotics.ros-tcp-connector`パッケージの`ROSConnection.cs`を確認したところ：

```csharp
void OnApplicationQuit() { Disconnect(); }
```

**`OnDestroy()`が実装されておらず、`Disconnect()`は`OnApplicationQuit()`からしか
呼ばれない。** しかしUnity EditorでPlay Modeを停止(Stop)しても`OnApplicationQuit()`は
呼ばれない(実際のアプリケーション終了時のみ発火する)。

つまり、**Stopするたびに古いTCP接続(バックグラウンド接続スレッド・ソケット)が
切断されずに残り続け、次のPlayで確立される新しい接続と競合する**。これが
Play/Stopを繰り返すほど通信が不安定になっていった直接の原因。

### 修正1: UnityRosSensorPublisher.OnDestroy()で明示的にDisconnect

```csharp
private void OnDestroy()
{
    if (ros != null) ros.Disconnect();
}
```

GameObjectの破棄(Play Mode Stopで確実に発生する)をトリガーに、Editor停止時にも
確実に接続を切断する。`Disconnect()`は`m_ConnectionThreadCancellation?.Cancel()`と
nullセーフな実装で、複数回呼んでも安全(冪等)なことを確認済み。

### 修正2: セッション相対タイムスタンプ(スタート時刻の記録)

`Start()`で`sessionStartTime = Time.timeAsDouble`を記録し、`/clock`等へpublishする
全タイムスタンプを`Time.timeAsDouble - sessionStartTime`に変更。

背景: 調査中、`EditorApplication.isPlaying = false`を要求した直後に確認せず次の
操作(Play要求等)を行うと、Unity側でStop→Playの遷移が完了しないまま次のPlayが
実行され、`Time.time`が前セッションの経過時間(数千秒)から再開されない現象を
複数回観測した。この状態で`/clock`をpublishすると、ROS側のevaluatorが
`duration_sec`(60秒)をとっくに超過していると誤認し、データがほぼ届く前に
評価を打ち切ってしまう。セッション開始時刻をエポックとして記録し、そこからの
相対時間だけをpublishすることで、`Time.time`の絶対値が何であっても
(Stop確認を怠るような運用ミスがあっても)、配信されるタイムスタンプは
常にこのセッションの経過時間として正しく0から始まるようにした。

なお、**Stop操作の後は必ず別呼び出しで`isPlaying=False`を確認してから次の
操作に進む**ことも運用上の教訓として重要(MCP経由の操作は非同期に処理されるため)。

### 検証: Editor無再起動での3回連続Play/Stop

上記2つの修正後、Unity Editorを一度も再起動せずに、Play→60秒評価→Stop→
Play→...というサイクルを3回連続実行し、いずれも正常完走することを確認した。

| Run | 総サンプル数 | 定常RMSE |
|---|---|---|
| 1 | 61913 | 22.47mm |
| 2 | 61946 | 22.47mm |
| 3 | 61844 | 23.33mm |

3回ともサンプル数(約6万件=60秒×約1000Hz相当)・RMSEともほぼ一致しており、
Editor再起動を要さず連続シミュレーションできることを実証した。

## 8. クローズドループ制御の結果(参考、上記安定性検証時点)

自己位置推定の出力(`/odom_fast`)を経路追従の制御フィードバックに使うクローズド
ループ構成に変更し検証したところ、定常RMSEは22〜23mm(要件10mm、FAIL)。

オープンループ(真値制御、自己位置推定は並行評価のみ)ではRMSE 4.91mmだったのに対し、
クローズドループにすると誤差が悪化する。しかも t=10秒以降、誤差が
mean 22〜23mm・max 25〜26mmという極めて狭い範囲に収束し続けており、ランダムな
ノイズではなく**系統的なバイアス**が生じていることを示唆する。

仮説: 自己位置推定に系統的バイアス(20mm前後)があり、ロボットは「推定上は目標に
到達した」と判断してPD制御力がほぼゼロに収束するが、真の位置は目標から
ずれた場所で静止するため、ミッション完了判定(真値ベースで5mm以内・速度10mm/s
以下)を満たせず、ミッションが未完了のまま停止し続ける。

次のチューニング(自己位置推定の精度向上)で、このバイアスの正体(壁マップの
座標オフセット、LiDAR設置位置の誤差等)を確認する価値がある。

## 9. ルールブックの確認(2026-07-23追記)

`docs/rule/千葉大学ロボットコンテスト 2026.pdf`を確認した。重要な点:

- **試合時間は5分(300秒)**。現状の`duration_sec`はテスト用に30〜60秒で運用中。
  実際の競技条件に近づけるにはこれを伸ばす必要がある。
- **壁高さ100mmは公式ルールとして正式に規定されている**
  (「3.2. 壁は外枠が幅38mm高さ100mm」)。私が特定した「LiDAR平面(140mm) > 壁上端
  (100mm)で壁が見えない」問題は実機でも起こりうる設計上の制約であり、最終的には
  LiDAR取り付け高さを100mm以下に見直すべき(ロボットサイズ制限は高さ800/900mmな
  ので低い位置への設置は寸法上問題ない)。
- ルールブック自体に木枠(33mm/90mm)とフィールド壁(38mm/100mm)の寸法矛盾が
  未解決のQ&A(Q9)として残っている。現行field.jsonは壁厚33mm・高さ暫定300mm。

## 10. LiDAR死角のノーツ側割り当てと、精度調査の測定結果(2026-07-23追記)

### 10-1. 270度センサの90度死角をノーツ側(-90度)へ向けた

ボールは未マップ物体で、停滞位置での実測では0.25〜0.39mという最至近距離に
36度分も入り込んでいた。すぐ後ろ(0.39〜0.51m)に中央壁があり、距離差が約0.1m
しかないため`planar_correspondence_distance: 0.20`では確実に誤対応する。

対処として、LiDARの取り付け角に相当する`angle_min`をずらし、90度の死角を
ロボット-y側(ノーツ側)へ向けた(`UnityRosSensorPublisher.LidarBlindCentreRadians`)。
可視域-45〜+225度、死角-135〜-45度。スキャンは`angle_min`で自己記述されるため
ROS側の変更は不要。

効果(真値からのICP、静止時):
- 死角化前: 16〜38mm
- 死角化後: 1〜5mm

評価値(クローズドループ): RMSE 22.4mm → 14.36mm。ボールを物理的に撤去した
A/B(15.8mm)とほぼ同等の効果が、取り付け角だけで得られた。

なお「ビンゴ支柱の厚み(33mm/40mm)が誤差要因」という仮説は棄却された。
レイキャストで、停滞位置からビンゴへ到達する光線は**0本**(スラローム板が
1.82mの距離で完全遮蔽)であることを確認済み。

### 10-2. 誤りだった2つの測定(記録として残す)

いずれも「診断ツール側の欠陥」であり、パイプラインの問題ではなかった。
同じ誤診を繰り返さないために残す。

**(a) 「IMUの58%が転送中に欠落している」→ 誤り。**
診断ノードが`qos_profile_sensor_data`(深さ5・ベストエフォート)で購読していた
ため、1kHzの流れを**診断ノード自身が取りこぼしていた**。深さ5000のRELIABLE
キューで数え直した結果:

```
Unity送信: 37798件/37.8s = 1000.0Hz     ROS受信: 51456件/51.455s = 1000.0Hz
```

欠落ゼロ。実際の消費側C++ノード3つはいずれも`SensorDataQoS().keep_last(2000)`
であり、1kHzで2秒分を保持できる。**IMUの周期を下げる必要はない**(実機も1kHz)。

**(b) 「存在しないIMUバイアスを推定している」→ 誤り。**
Unityは実際にバイアスを注入している。`bias`という語を含まないインライン定数
だったため最初のgrepで見落とした(`UnityRosSensorPublisher.PublishImu`):

```csharp
accelerationRos.x    += 0.05  + Gaussian(0.01);
accelerationRos.y    += -0.03 + Gaussian(0.01);
angularVelocityRos.z += 0.005 + Gaussian(0.001);
```

推定値は`|accel_xy|=0.0563`(真値0.0583)、`gyro_z=+0.004865`(真値+0.005)で、
バイアス推定はほぼ正解を当てている。正常動作の証拠。

### 10-3. 残っている本物の問題: ICPのシード依存性

`tools/fusion_stage_diag.py`で段階別に真値と比較した結果(静止時、符号付き):

```
scan_match wall.corrected_pose : dx=+6.71  dy=-13.15mm  |bias|=14.77mm  dyaw=+0.297deg
state_estimate (融合後)        : dx=+6.77  dy=-13.26mm  |bias|=14.89mm
odom_fast (評価対象)           : dx=+6.79  dy=-13.33mm  |bias|=14.96mm
```

3段階とも同値。**誤差はマッチャ出力時点で既に存在し、融合は情報を追加も損失も
していない**。標準偏差0.8〜1.5mmで方向が固定された系統バイアス。

一方、同じスキャン・同じマップを**真値からICP**すると1〜5mmに収まる
(`tools/icp_from_truth_diag.py`)。

つまり、シードを真値にすると3mm、現在の推定値にすると14.8mm。ICPがシード依存の
別の停留点に居座っていることになる。次の検証は「真値+14mmオフセットから開始して
そこに留まるか、真値へ戻るか」。留まるなら対応距離0.20mが広すぎる/幾何の一部縮退。

### 10-4. 手順面の改善

- **評価器の計測窓を最初の`/ground_truth_pose`到着から数えるよう変更**
  (`evaluator_node.py`)。従来はノード起動時から回っており、MCP経由でPlayを
  押すまでの30秒超がまるごと窓を食い潰して「一度も評価されないまま時間切れ」に
  なっていた。
- **`imu_lost_timeout_sec`の起動トランジェント対策**。接続直後に一度だけ約2.075秒
  の転送停止があり(初回スキャンの1081レイキャスト+パブリッシャ登録)、閾値2.0秒が
  ちょうど境界上でラッチ式fail-safe停止を誘発して実行全体を殺していた。閾値は2.0秒
  のまま、`imu_watchdog_arm_delay_sec`を2.0→10.0秒にして武装を遅らせた。
- **`src/lio_localization_sim/scripts/stage4_mcp_run.sh`を新設**。ROS起動→レートプローブ武装→`READY`出力
  までを自動化し、Play以外の全工程を持つ。バッチモード(Editorを閉じる必要があり、
  以前OOMクラッシュを招いた経路)は使わない。
- 派生: `src/lio_localization_sim/scripts/stage4_icp_diag.sh`(スキャンのみのICP検証)、
  `src/lio_localization_sim/scripts/stage4_fusion_diag.sh`(段階別の誤差localisation)。

## 11. 要件A達成: 未マップのノーツをICPから除外した(2026-07-24)

### 11-1. 結果

```
                位置誤差RMSE    max        ヨー誤差RMSE   判定
除外なし          14.99mm      18.00mm     0.319deg      定常FAIL
ノーツ除外         3.33mm      14.27mm     0.113deg      定常PASS / 最大PASS
```

要件A(定常±10mm、最大±20mm)を初めて両方満たした。

### 11-2. 原因: 「ちょっと見えていた」オレンジのノーツ4個

10-1でLiDARの90度死角をノーツ側(-90度)へ向けたが、ノーツは左右2列あり、
死角に入ったのは片方(y≈0.02〜0.27)だけだった。オレンジの列(y=+0.162)は
死角の外に残り続けていた。死角は90度しかないため、取り付け角では両列を
同時に隠せない。

同じ土俵(フルスタック同時実行)で`scan_residual_diag`を回して特定した:

```
OUTLIERS (-2.5,0.0):n=67,d=118mm  (-2.0,0.0):n=28,d=140mm
→ Unityレイキャストで照合: OrangeNote_0..3 が93本、centre_to_bingoが26本
```

わずか数点〜9点(間引き後541点中)だが、次の3条件を同時に満たすため効いた:

1. 至近距離(0.39〜0.92m)で、マップ上の壁より手前にある
2. マップから116〜142mm外れる(正常な壁点の残差σ=30mmの4倍以上)
3. 4個とも+y側に固定配置で、**常に同じ側**に引く(打ち消し合わない)

Huberは重みを`δ/|e| = 0.025/0.13 ≈ 0.19`まで下げるが**ゼロにはしない**。
Huberを外すと誤差が14.7→30.6mmに倍化したのは、この抑制が効いていた分。
つまりHuberが半分抑え、残り半分が13mmとして残っていた。

### 11-3. 実装: planar ICPの2パス化

`laser_scan_matching_node.cpp`。1パス目で全点から暫定姿勢を出し、マップで
説明できない点をクラスタ化して落とし、2パス目で再度解く。

新パラメータ:
- `planar_exclude_balls` (true)
- `ball_exclusion_distance` (0.05) — マップからこれ以上離れた点が候補
- `ball_exclusion_max_extent` (0.35) — クラスタの広がりがこれ以下なら落とす

**判定を円フィットの半径ではなく空間的な広がりにしたのが決め手。**
LiDARはボールの手前側の短い弧しか見ないため、Kasa円フィットは実半径75mmを
**34mm**と大きく外し、ゲート`[0.065, 0.085]`を一度も通れずに除外が空振り
していた(最初の実装では効果ゼロ: 14.98mm vs 14.99mm)。広がりは同じ点群から
安定して求まり、「ノーツは自分の直径分しか広がらない/未マップの壁ならもっと
長い」という区別も保てる。

既存の`/ball_candidates`検出器は同じ半径ゲートの問題を抱えたまま(だから
`detected_balls=0`が続いていた)。別途修正する価値がある。

### 11-4. 方法論上の教訓: オープンループで測ってはいけない

10-3で「本番マッチャ14.8mm vs オフラインICP 2.18mm」という食い違いを追い、
シード依存性・マップ差・デスキュー・Huber・間引きと**5つの仮説をすべて外した**。

原因は実装差ではなく**測定条件の違い**だった。診断ツールだけを起動すると
ROSスタックが無く、`HasOdomEstimate=false`で制御が真値にフォールバックする。
ロボットは理想経路を走って公称の終点に停まり、**本番とは別の姿勢**を測って
いた。停止位置が違えば見える壁の構成も違い、ICPのバイアスも変わる。

フルスタックと診断ツールを同時に走らせて測り直すと:

```
production   |bias|=14.81mm     ← 動作中のマッチャ
truth_prod   |bias|=14.68mm     ← 同じスキャンをオフラインで、本番と同一設定
offset_prod  |bias|=14.78mm     ← 真値+15mmのシードから
truth(素の最小二乗) |bias|=30.57mm
```

実装差は**存在しなかった**。同時に、Huberは害ではなく大きく効いている
(外すと倍化する)ことも分かった。

以後、精度に関する計測は必ずフルスタックを動かした状態で行うこと。
`icp_seed_mode_diag.py`は`/scan_match_result`をスタンプで突き合わせ、
本番とオフラインを同一サンプル集合で比較できる。

### 11-5. 残る誤差は走行中のみ

除外後の誤差の時系列:

```
t=0-8s  (走行中): pos mean 4.3〜10.1mm  max 14.27mm  yaw max 0.502deg
t=8-60s (静止中): pos mean 2.0〜 2.6mm  max  4.54mm  yaw max 0.05deg
```

静止時は既にmax 4.5mmで、maxを10mm以下にするには走行中を詰める必要がある。

疑ったデスキューは**シロ**だった。Unityは`time_increment`に`1e-9f`を明示的に
設定しており(0ではない)、ROS側の`msg->time_increment > 0.0`が真になるため
`fallback_dt`は使われない。1081本を同一FixedUpdateで一瞬にレイキャストする
Unityの実態と整合しており、デスキューは実質的に恒等変換。既に対処済みだった。

したがって走行中の誤差の原因は未特定。次は推測せず測ることとし、
`fusion_stage_diag.py`を`settle_sec=0`で走らせて走行区間を段階別に分け、
誤差がマッチャ出力で既に出ているのか、融合/伝播で入るのかを先に確定させる。

なお実機のUTM-30LXは実際に掃引で歪むため、いずれUnity側で各レイを掃引時刻ぶん
過去にサンプルし`time_increment`/`scan_time`を実値にする改修は必要になる。
今は歪みが無い前提で整合が取れている状態。

## 12. Unity Editorの起動コマンド(2026-07-25)

WSLからWindows側のEditorを直接起動できる。スクリプト化済み:

```bash
bash src/lio_localization_sim/scripts/launch_unity_editor.sh
```

中身(手打ちする場合):

```bash
nohup "/mnt/c/Program Files/Unity/Hub/Editor/6000.5.4f1/Editor/Unity.exe" \
  -projectPath 'C:\Users\kouza\UnityProjects\Robocon2026Sim' \
  > /tmp/unity_editor_launch.log 2>&1 &
```

- シーンは`SimulationBootstrap`の`RuntimeInitializeOnLoadMethod`で手続き的に構築されるため、
  プロジェクトを開くだけでよい。MCP接続もロード完了後に自動で復帰する。
- **Editorは絶対に1つだけ**。このマシンはRAM 15.7GBでEditor 1つが約4GB使うため、
  2つ目はgraphics初期化で"Could not allocate memory"で落ち、**動作中のEditorごと
  巻き込んでクラッシュした実績がある**(§10参照)。スクリプトは既存Editorを検出したら
  起動せず終了する。
- ロードには1〜3分。プロセスのメモリが約90MB→800MB超に増えるのが完了の合図。
  スクリプトはこれをポーリングして待つ。
- 起動確認: `tasklist.exe | grep -iE '^Unity\.exe'`

### 関連スクリプト

| スクリプト | 用途 |
|---|---|
| `src/lio_localization_sim/scripts/launch_unity_editor.sh` | Editor起動(上記) |
| `src/lio_localization_sim/scripts/stage4_mcp_run.sh` | ROSスタック起動→READY出力(Play以外を自動化) |
| `src/lio_localization_sim/scripts/stage4_icp_diag.sh` | スキャンのみのICP検証 |
| `src/lio_localization_sim/scripts/stage4_fusion_diag.sh` | 段階別の誤差localisation |

実行手順は常に「ROS起動 → READY確認 → MCPでPlay → 完了待ち → MCPでStop(必ず
isPlaying=Falseを別呼び出しで確認)」。Unityを先に走らせるとpublisher登録が
`Start()`で行われる都合で`Not registered to publish topic`になる。
