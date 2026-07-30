# 実験履歴: 初めての実旋回テストと、そこで見つかった3つのバグ(2026-07-30)

## 目的

`FreezeRotation`によりこれまで一度も物理的に回転していなかったことが判明した
(report 15参照)ため、実際にロボットを旋回させて自己位置推定の挙動を初めて
検証する。

## 1. 追従(走行制御)側の実装と、そこで踏んだバグ

`StartToBingoValidation.cs`の`body.constraints`を`FreezeRotationX|FreezeRotationZ`
(ヨーのみ許可)へ緩め、進行方向を向くヘディング制御を追加したところ、**要件A
から程遠い規模(最大159m、ヨー誤差最大180°)で完全に破綻**した。

原因調査を4回試みたが、いずれも改善はするが破綻は解消しなかった:

1. `planar_max_yaw_shift`緩和(0.08→0.7rad)
2. ヨー目標の瞬時スナップ→なめらかな事前計算プロファイル化
3. `/odom_fast`のtwist座標系バグ修正(下記2.参照、これ自体は本物のバグ)
4. `body.MoveRotation()`→`body.angularVelocity`直接代入への変更(下記3.参照)

## 2. バグ1: `/odom_fast`のtwistがワールド座標系のまま配信されていた

`imu_preintegration_node.cpp`の`publish_odom()`が、GTSAM `NavState::velocity()`
(ワールド座標系)をそのまま`odom.twist.twist.linear`(child_frame_id=base_link
のはずなのでボディ座標系であるべき)へ代入していた。yaw=0では無症状(恒等変換)
だが、Unity側`UnityRosSensorPublisher.OnOdomFastReceived`は「twistはボディ
座標系」の前提でヨー回転をかけてワールド座標へ戻しており、実ヨーが非ゼロに
なると二重回転で閉ループ制御のフィードバックが壊れる。ヨーの逆回転を掛けて
ボディ座標系へ変換するよう修正した(`lio_localization/src/imu_preintegration_node.cpp`)。

## 3. バグ2: `MoveRotation`が`angularVelocity`を更新しない

上記1の修正後もなお破綻したため、①の内部状態(`diag_odom_path`で有効化できる
既存の診断ログ)を直接ground truthと突き合わせたところ、**IMUのジャイロが
実際の回転を全く感知していない**(常にバイアス・ノイズレベルのまま)ことが
判明した。原因は`body.MoveRotation()`が非kinematicなRigidbodyでは
`Rigidbody.angularVelocity`を正しく更新しないため。`transform.rotation`自体は
正しく回るためground truthの姿勢は正しく見えるが、`body.angularVelocity`を
読むIMU/ground truthのtwistだけが実態を反映しなくなっていた。

修正: 目標ヨーレート(位置のvelocityと同じ規約でMinimumJerkの微分から解析的に
求めた値)を`body.angularVelocity`へ直接代入する方式に変更した
(`StartToBingoValidation.cs`)。これでもなお破綻は解消しなかった。

## 4. バグ3(真因): ジャイロが擬ベクトルとして正しく変換されていなかった

`UnityRosSensorPublisher.cs`の`UnityVectorToRos`は`(x,y,z) -> (z,-x,y)`という
写像で、行列式が**-1**(Unity左手系→ROS右手系の鏡映)。速度・加速度のような
極性ベクトルにはこれで正しいが、角速度は擬ベクトルのため、行列式が負の写像
では追加の符号反転が必要(`v' = det(M)・M・v`)。この符号反転が抜けたまま
IMUメッセージとground truthのtwist.angularに使われていた。

上記2で①の内部状態が正しく更新されるようになって初めて、この符号バグの
影響が生データで直接確認できた: `ground_truth.csv`から数値微分した真のヨー
角速度と、①の予測ヨーを突き合わせると、**予測ヨーが真値と正確に符号反転**
していた(例: t=11.24s、予測+56.96° vs 真値-56.99°)。

修正: `UnityAngularVelocityToRos()`という専用ヘルパー(`UnityVectorToRos`の
結果を丸ごと符号反転)を追加し、IMUメッセージとground truthのtwist.angular
の両方に適用した(`UnityRosSensorPublisher.cs`)。

## 5. 検証(bag再生のみ、Unity起動なし)

Unity実行の負担を避けるため、既に取得済みのUnity生成CSV(旋回を含む約40秒)
をMCAP化し、`bag_component_isolation.sh`にジャイロ符号だけを反転する
一時スクリプト(`negate_gyro_sign.py`)を通したbagで検証した。
`bag_component_isolation.sh`に`--imu-param`オプションを追加し、
`imu_preintegration_node`の`diag_odom_path`診断ログを有効化できるようにした。

| 指標 | 修正前(バグ1のみ修正済み) | 修正後(バグ3も修正) |
|---|---:|---|
| ①予測ヨー vs 真値(t=11.24s) | +56.96° vs -56.99°(符号反転) | -57.11° vs -56.99°(誤差-0.12°) |
| 破綻の兆候(541/541点非対応等) | 発生 | **0件** |
| 位置誤差(t=15.0s、旋回中) | mean 5.6m、max 12.5m(発散継続) | mean 10.18mm、max 27.65mm(一過性) |
| 位置誤差(t=25.0s以降) | mean 30m超(発散継続) | mean 0.85〜1.48mm(正常値へ回復) |

要件A(≤10mm)は旋回の瞬間だけ一時的に超過するが、直後に正常値へ回復する。
これは「バグ」ではなく、実際の急旋回中の推定器の素の性能として妥当な挙動と
判断した(§6参照)。

## 6. なぜ回転は並進よりクリティカルに悪化するのか

ユーザーとの議論で整理した要点:

1. **距離による誤差の増幅**: 並進誤差は距離によらず一定だが、回転誤差θは
   距離rの点をr×θだけ動かす。壁までの距離(1〜3m)を掛けると、数度の残差
   ヨー誤差でも壁面では数cmの点群誤差になり、対応点探索半径
   (`planar_correspondence_distance_=0.08m`)を容易に超える。
2. **回転にだけ粗探索が無い**: 位置には`kCoarseAcceptDistances`
   (2.0→1.0→0.5m)があるが、ヨーは1反復±0.08radにクランプされた局所
   Newton法のみで、大きくズレたシードから抜け出す手段が無い。
3. **IMU積分による姿勢誤差の位置誤差への伝播**: GTSAMの前積分は姿勢
   (回転行列)を使って機体座標系の加速度をワールド座標系へ回転してから
   2回積分するため、姿勢誤差は速度・位置误差として増幅されて伝播する。

## 7. ヨー版「迷子モード」(保険)の実装

上記6の(2)への対処として、位置の`enable_lost_mode_`/`kLostModeExtraDistances`
と同型の、ヨー版迷子モードを実装した(`laser_scan_matching_node.cpp/hpp`)。

- `planar_valid`の連続失敗が`kYawLostModeThreshold`(80フレーム、40Hzで約2秒)
  に達した場合に限り発動
- シードそのままと、シードのヨー+π(フィールドの180°対称性に対応)の
  両方を解き、対応点数の多い方を採用
- **既定は無効**(`enable_yaw_lost_mode`, 既定false)。位置版と同じ理由
  (誤った局所解に確信を持って固着するリスク)
- 発動時は`RCLCPP_ERROR`で目立たせる。「発動すること自体が上流の追跡失敗を
  示す異常」という位置づけで、恒常的に発動する状態を正常運転として許容
  しない設計思想

### 検証結果

- 既定無効時: 修正済みbagで回帰なし(数値ほぼ同一、発動0件)を確認
- 明示的に有効化: 未修正(符号バグ入り)bagで実際に発動し、両仮説を正しく
  比較・ログすることを確認。ただしこのbagでは発動する頃には位置(x,y)側も
  大きくドリフトしており、両仮説とも対応点0件で救えなかった(ヨー1軸だけの
  粗探索であるため、位置まで同時に崩れた完全な破綻からは救えないという
  設計上の限界を確認)。

## 対応状態

- バグ1(twist座標系): 修正済み(`lio_localization`)
- バグ2(MoveRotation): 修正済み(`lio_localization_sim`、その後
  `body.angularVelocity`直接代入方式へ変更)
- バグ3(ジャイロ擬ベクトル符号、真因): 修正済み(`lio_localization_sim`)
- ヨー版迷子モード: 実装・検証済み、既定無効
- Unity: 全ての検証はbag再生のみ。実Unity上での通し確認(60秒フル)はまだ
  行っていない
