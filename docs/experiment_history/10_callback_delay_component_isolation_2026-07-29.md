# 実験履歴: IMU callback遅延の構成要素切り分け (2026-07-29)

## 目的

backendのiSAM2を専用workerへ移し、1 kHz `/odom_fast`評価器をC++化した後にも、
フル構成bag再生で最大145.981msのIMU callback gapが一度観測された。
Unity CSVから変換した同一88.42秒MCAPの先頭62秒を使い、フル構成だけに含まれる
`ball_tracking_node` とROS-TCP endpointが原因かを切り分けた。

全runでIMUのheader stamp gapは最大1.000ms、`>1ms`は0件だった。従って、ここでの
gapはUnityのセンサ生成時刻ではなく、ROS側でbackend callbackが実行されるまでの壁時計遅延である。

## 結果

| 構成 | run | callback gap最大 | >10ms | IMU callback数 |
|---|---|---:|---:|---:|
| 最小LIO + C++評価器 | `imu_receiver_with_cpp_evaluator_20260729_r2` | 20.476ms | 28 | 62,001 |
| 最小LIO + ball | `imu_receiver_cpp_plus_ball_20260729_r1` | 16.923ms | 24 | 62,001 |
| 最小LIO + endpoint | `imu_receiver_cpp_plus_endpoint_20260729_r1` | 28.482ms | 11 | 61,202 |
| 最小LIO + ball + endpoint | `imu_receiver_cpp_plus_ball_endpoint_20260729_r1` | 21.872ms | 25 | 62,001 |
| フルlaunch（初回） | `bag_replay_88s_cpp_evaluator_20260729_r1` | **145.981ms** | 236 | 60,286 |
| フルlaunch（再試験） | `bag_replay_88s_cpp_full_retest_20260729_r1` | 32.417ms | 39 | 60,502 |
| フルlaunch（評価器・ballをnice +10） | `bag_replay_88s_cpp_nice10_20260729_r2` | **22.408ms** | **17** | 60,268 |

全runの評価精度はmean約6.25mm、RMSE約6.94mm、最大約13.1〜13.2mmで同水準だった。
定常RMSE 10mm要件はPASS、最大位置誤差10mm要件はFAILのままである。

## 判断

- `ball_tracking_node`の有無で悪化はなく、原因ではない。
- idle状態のROS-TCP endpointも単独・ballとの同時追加のいずれでも145ms級を再現しなかった。
- フル構成の再試験は32.417msまで低下した。初回145.981msは再現しないため、現時点では
  恒常的なノード構成の問題ではなく、試験時のホスト／WSLスケジューリング外乱として扱う。
- いずれもstamp gapはない。Unityの1kHz生成・CSV記録が原因という仮説は否定される。

## 実施した改善

`stage4_mcp_run.sh`のendpoint起動確認は、従来`/dev/tcp`で実接続して直ちに切断していた。
そのためendpoint自身が「No more data available」という偽の例外を記録していた。現在は`ss`で
listen socketのみを確認し、実験へ偽クライアントを注入しないよう修正した。

同時に`bag_component_isolation.sh`を追加した。YAML、commit、dirty diff、未追跡ソースを
runごとに保存して、今後の構成追加試験を再現可能にする。巨大な過去run成果物はソース
スナップショットから除外する。

フルlaunchでは、計測専用のC++評価器と自己位置推定の安全経路ではない
`ball_tracking_node`を`nice +10`で起動するよう変更した。backend、IMU前積分、scan matching、
ROS-TCP endpointは通常優先度のままとした。適用後の有効runは最大22.408ms、10ms超17回で、
位置誤差mean/RMSE/maxは6.26/6.94/13.17mmと変更前と同水準だった。

最初の`bag_replay_88s_cpp_nice10_20260729_r1`はROS launchのprefixをリスト形式で渡したため
`nice-n10`という存在しない実行ファイルになり、評価器とball追跡が起動しなかった無効runとして
ログを保全した。prefixを`"nice -n 10"`へ修正したr2のみを有効結果としている。

### IMU専用executor・固定リングバッファ

backend内の2-thread `MultiThreadedExecutor`を廃止した。IMU callback groupをnodeへ自動登録せず、
専用`SingleThreadedExecutor`へ手動割当した。scan結果callbackとwatchdogは別のcontrol
`SingleThreadedExecutor`、GTSAMは従来どおりoptimizer workerが所有する。さらに高頻度
`imu_queue_`を8,192件（1kHzで約8秒）の固定長リングに置換した。通常2秒の保持域を超える
過負荷では古いIMUを黙って破棄せず、overflowをfail-safeとしてラッチする。

`bag_replay_88s_dedicated_imu_executor_20260729_r1`では、位置誤差mean/RMSE/maxは
6.26/6.94/13.17mmで従来同一、IMU stamp gapは最大1.000ms・>1ms 0件だった。callback統計は
最大gap **12.900ms**、>2ms 201回、>10ms **1回**、queue mutex待ち最大1.539ms、callback処理時間
最大1.541msである。リングoverflow、IMU fail-safe、GTSAM例外は発生しなかった。

この結果は、スレッド数を一律に減らすのではなく、IMU受信・control callback・最適化を
固定した責務の3実行ドメインに分けたことで、callback内外の待ちを抑えたことを示す。

### lock-free SPSC IMUリング（mutex待ちの除去）

固定リングの初回測定では、専用IMU executorによりcallback gapは大幅に縮小した一方、
IMU callbackがworkerと`work_mutex_`を共有していたため、queue mutex待ち最大1.539ms、
callback処理時間最大1.541msが残った。そこでリングを、IMU executorだけがproducer、
optimizer workerだけがconsumerとなるlock-free SPSC実装へ変更した。2秒ホライズンの掃除は
IMU callbackから外し、watchdog通知を受けたworkerだけが、保留スキャンも実行中ジョブも無い
ときに行うようにした。overflow時は従来どおりfail-safeをラッチし、古いIMUを黙って捨てない。

最終ソースを再リンクして実行した `bag_replay_88s_lockfree_imu_ring_20260729_r3`（同じ88.42秒
MCAPの先頭62秒、bag再生倍率は1倍、`--clock 250`は`/clock`発行周波数250Hz）では、位置誤差
mean/RMSE/maxは **6.25/6.94/13.17mm**、IMU stamp gapは最大1.000ms・>1ms 0件で、推定精度と
入力連続性は維持した。IMUリング投入時間は最大**0.065ms**、>2ms **0回**である。callback全体では
最大4.063ms、>2ms 2回、callback gap最大28.828ms・>10ms 11回だった。直前の同構成r2でも投入時間
最大0.192ms・>2ms 0回だった一方で、gap最大は15.807msだった。callback全体の壁時計計測にはOSによる
deschedule時間も含まれるため、run間で揺れる残余の超過をmutex待ちと解釈してはいけない。少なくとも
共有mutexが直接作っていた1.539ms待ちは除去済みであり、残余はホストスケジューリング側の問題として
分離された。ring overflow、IMU fail-safe、GTSAM例外は両runで発生しなかった。

なお `bag_replay_88s_lockfree_imu_ring_20260729_r1` は、更新済みソースに対して古いリンク済み
実行物が起動していたことをログ文字列（`queue_lock_wait`）で検出したため、無効runとして保全した。
再リンク後に実行物へ`ring_enqueue`文字列が含まれることを確認してr2を実行し、起動時掃除も
含む最終ソースをr3で再確認した。

## 次の調査条件

145ms級が再発した場合のみ、そのrunのホスト負荷・同時プロセス・WSL CPU使用率を同じ時刻軸で
採取する。再発していない状態でscheduler優先度やCPU affinityを固定すると、偶然の外乱を
恒久設定として持ち込むだけになるため、現段階では採用しない。
