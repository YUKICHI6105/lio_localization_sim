# 実験履歴: Unity直結時のscanカバレッジ遅延調査（2026-07-30）

## 目的

rosbag2再生時よりUnity直結時の最大位置誤差が大きくなる原因を、`/odom_fast`の
IMUカバレッジ不足とscan matchingの処理タイミングから切り分ける。

## 1. 内部レイテンシ診断

`imu_preintegration_node`の既存高速診断レコーダを一時有効化し、各`/odom_fast`出力の
`confirmed_t`、`deltaTij`、処理経路を保存した。

run: `docs/stage4_runs/unity_direct_imu_latency_diag_20260730_r1/`

| 指標 | 結果 |
|---|---:|
| 位置mean / RMSE / max | 0.63 / 0.93 / 17.55 mm |
| backend callback gap最大 | 511.133 ms |
| IMU stamp gap最大 | 1.000 ms |
| IMU stamp gap >1 ms | 0件 |
| Unity監査 | sourceと受信callbackがチェックポイントで一致 |

最大値付近では、古い補正状態による誤差が17.55 mmまで増えた直後、同じ推定時刻の
再積分済み出力で0.14 mmへ戻った。センサ時刻の欠損ではなく、補正反映とcallback実行の
遅延過渡である。

同時刻の`laser_scan_matching_node`ログには、scan区間末端まで`/odom_fast`が届かず、
50 ms待機後に「extrapolated tail」でscanを処理した警告がある。最大値の開始時刻と
約2 ms差で一致した。

## 2. 未カバースキャン一律棄却案

リアルタイム性を優先し、50 ms待ってもscan区間を`/odom_fast`がカバーしない場合、
外挿してICPへ投入せずscanだけを棄却する実装を試した。IMUメッセージとIMU積分は
継続させた。

run: `docs/stage4_runs/unity_direct_coverage_drop_20260730_r1/`

| 指標 | 結果 |
|---|---:|
| 位置mean / RMSE / max | 1.26 / 3.79 / 105.95 mm |
| scan受信 | 2,425 |
| scan処理 | 1,873 |
| coverage timeout棄却 | 551（22.7%） |
| 最大連続棄却 | 21枚（約0.53秒） |

scan更新を失う時間が長くなり、IMUのみの外挿ドリフトが増大した。したがって、
「間に合わなければscanを一律棄却」は不採用とする。詳細な理由はrun内の
[`VALIDITY.md`](../stage4_runs/unity_direct_coverage_drop_20260730_r1/VALIDITY.md)に保存した。

## 結論

不足していた原因は、Unityのセンサ生成欠損ではなく、`laser_scan_matching_node`が
単一の`rclcpp::spin()`上でIMU・`/odom_fast`受信とICPを同時に処理していることによる
姿勢履歴の遅延である。

次の実装対象は、一律棄却ではなく、scan受信・時刻付きpending queue・ICP workerを
分離し、姿勢履歴を更新し続けながら、カバレッジが揃ったscanだけを時刻整合して処理する
構造である。今回の実験ではこの構造変更自体は未実施である。

## 対応状態

- 一律棄却実装: 撤回済み
- 通常設定: 従来の外挿処理へ復帰済み
- ビルド: 復帰後に`lio_localization`を再ビルド済み
- Unity: 試験終了後`isPlaying=false`を確認済み
