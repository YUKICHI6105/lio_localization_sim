# Unity MCPを使ったStage 4ベンチマークの実行方法

Stage 4(Unity + ROS 本番フルスタック)のベンチマークを、キーボード/マウスの
自動操作やウィンドウフォーカス切り替えを一切使わずに、Unity Editorの
`com.unity.ai.assistant`(Unity MCP)経由のコード実行だけで走らせる手順。
2026-07-28〜29のセッションで実際に使い、17回以上のベンチマークで安定して
動作した方法。

**この手順を使う前に**: 一発目の計測を信用する前のチェックは
[`../../../../.claude/skills/unity-ros-restart-sanity-check/SKILL.md`](../../../../.claude/skills/unity-ros-restart-sanity-check/SKILL.md)
(Claude Code固有のパスにあるが、内容はUnity再起動直後の既知の落とし穴なので
どのツールで作業していても有効)を参照。

## 前提

- Unity MCP(`Unity_RunCommand`等)がツールとして使える状態になっていること。
  新規登録した場合はセッション再起動が必要(ツール一覧はセッション開始時にしか
  読み込まれない)
- Unity Editorが起動済みで、対象プロジェクト(`Robocon2026Sim`)が開かれていること
- ROS 2ワークスペースがビルド済みであること

## Play/Stopの制御方法: `EditorApplication.isPlaying`を直接読み書きするだけ

**キーボード操作・マウス操作・ウィンドウフォーカスは一切使わない。** `Unity_RunCommand`
はC#スニペットをその場でコンパイル・実行できるので、Play開始・停止は次の1行で行う。

Play開始:
```csharp
using UnityEngine;
using UnityEditor;

internal class CommandScript : IRunCommand
{
    public void Execute(ExecutionResult result)
    {
        EditorApplication.isPlaying = true;
        result.Log("Play requested");
    }
}
```

Play停止:
```csharp
using UnityEngine;
using UnityEditor;

internal class CommandScript : IRunCommand
{
    public void Execute(ExecutionResult result)
    {
        EditorApplication.isPlaying = false;
        result.Log("Stop requested");
    }
}
```

状態確認(Play開始前・停止後の検証に使う。停止直後は非同期に反映されることがあるため、
数秒待ってから確認する):
```csharp
using UnityEngine;
using UnityEditor;

internal class CommandScript : IRunCommand
{
    public void Execute(ExecutionResult result)
    {
        result.Log("isPlaying={0} Time.time={1}", EditorApplication.isPlaying, Time.time);
    }
}
```

Unity C#スクリプトを編集した直後は、次のPlay前に明示的にリフレッシュしてコンパイル
完了を確認する(自動検知を待たず確実に反映させるため):
```csharp
using UnityEngine;
using UnityEditor;

internal class CommandScript : IRunCommand
{
    public void Execute(ExecutionResult result)
    {
        AssetDatabase.Refresh();
        result.Log("Refresh triggered, isCompiling={0}", EditorApplication.isCompiling);
    }
}
```
(`isCompiling`が`false`に戻るまで数秒おいて再確認する)

## ベンチマーク1回の手順

1. **ROSスタックの残留プロセスがないか確認**:
   ```bash
   ps aux | grep -E "imu_preintegration_node|backend_optimizer_node|laser_scan_matching_node|realtime_evaluator_node|ball_tracking_node|default_server_endpoint" | grep -v grep
   ```
2. **Unityが待機状態であることを確認**(前回のPlayが残っていないか):
   上記の状態確認コードで `isPlaying=False` を確認する
3. **ROSスタックを起動**(バックグラウンド):
   ```bash
   scripts/stage4_mcp_run.sh <run名> > /tmp/run_driver.log 2>&1 &
   ```
   このスクリプトはROSノード群を起動し、TCPエンドポイント(:10000)が開くまで待ち、
   `$OUT/READY`ファイルを作った時点で `READY — enter play mode now` と出力する。
   また、実行時点のYAML、両リポジトリのコミット、`git status`、未コミット差分、
   実行条件manifestを`$OUT`へ自動保存する。
4. **READYを待ってからPlayを開始**(`grep -q READY /tmp/run_driver.log`をポーリングするか、
   READY出力を待ってから、上記の「Play開始」コードを実行する
5. **進行を監視**: `$OUT/ros.log` を tail し、evaluatorが5秒おきに出す
   `t=X.Xs  直近5秒: pos_err mean=...`行を追う。60秒経過すると
   `判定: 定常誤差... / 最大位置誤差...` と `レポートを保存しました: /tmp/sim_eval_report.txt`
   が出る
6. **レポートが出たらPlayを停止**(上記の「Play停止」コード)。4秒ほど待ってから
   状態確認コードで `isPlaying=False` を確認する
7. `stage4_mcp_run.sh` 自体がレポートファイルの出現を検知して自動的にROSノードを
   停止し、`sim_eval_report.txt` / `sim_eval_plot.png` / `diag_raw_errors.csv` 等を
   `$OUT` (=`docs/stage4_runs/<run名>/`) へコピーする。プロセスが残っていないことを
   念のため確認する

## Unity生成データをROSだけで再生する

Play停止時、Unityは送信前の`/imu/data`・`/scan`・`/ground_truth_pose`を実プロジェクト
直下の`SensorRecordings/<run_id>/`へ保存する。これをMCAPへ変換すれば、TCPやUnity Editorの
負荷を切り離してROS側だけを再試験できる。詳細は
[`UNITY_SENSOR_CSV_ROSBAG_GUIDE.md`](UNITY_SENSOR_CSV_ROSBAG_GUIDE.md)を参照。

## 標準の送受信欠損監査

`UnityRosSensorPublisher`は通常動作時に、同じUnity--ROS TCP接続で
`/unity_sensor_publish_counts`を0.25秒ごとに送る。これはUnity側で送信直後に
増やした累積件数であり、Pythonの`ros2 topic hz`のような観測用subscriberの
取りこぼしではない。

run終了時は次の二行を確認する。

- `backend concurrency stats: ... unity_audit ... source_imu_last=N receiver_at_last_report=N`
- `scan pipeline stats: ... unity_audit ... source_scan_last=N receiver_at_last_report=N`

それぞれの同名`N`が一致すれば、最後に受け取ったUnity監査チェックポイントまで、
Unityが送ったIMU/scanはROSの実受信callbackへ全件到達したことを示す。
`final_receiver`が`receiver_at_last_report`より少し大きいのは、最後の監査通知後に
正常到着したデータであり欠損ではない。逆に`source_*_last > receiver_at_last_report`
なら未到着を疑い、そのrunを有効な性能根拠に使わず、Unity CSV・endpointログ・
受信側ログを保全して原因を切り分ける。

## 入力センサ条件を変えたときの再録規則

**LiDAR/IMUの生成モデルを変えた試験は、必ずUnityから新しく録画してから評価する。**
たとえば距離ノイズ、IMU白色雑音・バイアス、欠測、時刻ずれ、取付け誤差、センサ更新率を
変更した場合、旧bagへ後処理で加える実験は原因切り分け用の参考値に限る。性能要件のPASS/FAIL、
採用判断、レポートの代表値には使わない。

一方、ROS側の推定器実装・重み・スレッド構成だけを変える比較は、同じ入力bagを再生してよい。

入力不整合が判明したrunは削除しない。runディレクトリとログを保全し、レポートに`無効（再録待ち）`
と理由を明記する。次の有効runでは、更新済みUnityからCSVを完了状態で出力し、MCAPへ変換して、
YAML・コミット・dirty diffとともに保存する。

## なぜこの方法なのか

- `EditorApplication.isPlaying`はUnity Editorの公開APIで、Playボタンを押すのと
  完全に同じ効果を持つ。キーボードでボタンをクリックしたように見せかける必要は
  そもそもない
- MCP経由でPlayを開始すると、AI Assistantパッケージ自身のrelay再接続や
  Asset Pipeline Refreshがちょうど同じタイミングで走り、`UnityRosSensorPublisher`の
  ROS接続確立と競合する既知の問題があったが、これは`ConnectAfterRosConnectionStarts()`
  側のリトライ実装(`HasConnectionThread`確認付き、最大10回)で解消済み
  (`unity-ros-restart-sanity-check`スキル参照)。Play制御の方法自体を変える必要はない
