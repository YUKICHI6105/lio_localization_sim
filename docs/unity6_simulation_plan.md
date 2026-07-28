# Unity 6.5 ロボコン2026シミュレーション移行計画

## 目的と計算分担

Unity 6.5を常時3D表示・GPUセンサー・操作確認の中心とし、ROS 2を自己位置推定、経路計画、
自己判断、採点の中心とする。Unity標準PhysXの剛体、接触、関節はCPU計算であり、描画、
深度画像、セグメンテーション、LiDAR用レンダリング／Compute ShaderをGPUへ割り当てる。

フィールド寸法の正本は
`src/lio_localization_sim/config/robocon2026_field.json`とし、Unity用コピーは同期スクリプトでのみ
生成する。座標変換は`(Unity X, Unity Y, Unity Z) = (-field Y, field Z, field X)`で固定する。

## 現在実装済み

- Unity 6000.5.4f1、URP 17.5プロジェクト
- Windows Direct3D 12 / Intel Iris Xeの検出
- JSONから床、色領域、外壁、中央壁、スラローム、ビンゴ、12球を自動生成
- 全静的形状のBoxColliderと、球のSphereCollider/Rigidbody
- 250 Hz固定物理、連続衝突判定、反復回数12/4
- 共有マテリアル、GPU Instancing、SRP Batcher、影なし、VSync 60 Hz
- 公式ROS-TCP-ConnectorとROS 2モードの利用準備
- Unity内検証: Collider 36個、球12個、ビンゴ位置・最上棚位置一致

## ファイル配置

- 編集正本: `/home/yukichi6105/ros2_ws/src/lio_localization_sim/unity/Robocon2026Sim`
- Windows実行用: `C:\Users\kouza\UnityProjects\Robocon2026Sim`

Windows版UnityはWSLの大文字小文字を区別するファイルシステムをプロジェクトとして開けない。
したがって次の2コマンドでJSONとWindows側プロジェクトを同期する。

```bash
cd /home/yukichi6105/ros2_ws
python3 unity/Robocon2026Sim/tools/sync_unity_field.py
python3 unity/Robocon2026Sim/tools/deploy_to_windows.py
```

Unity HubからWindows実行用ディレクトリを開き、生成済みの
`Assets/Robocon2026/Scenes/Robocon2026Simulation.unity`を開いてPlayする。

## 以降の開発工程

1. **機体剛体**: 実測した質量、重心、慣性テンソル、外形Colliderを登録する。
2. **三輪オムニ**: 120度配置、ローラ方向の異方性摩擦、モータトルク・回転数制限を実装する。
3. **機構**: 回収、最大2球保持、投入機構をArticulationBodyまたはJointでモデル化する。
4. **ROS 2**: `/cmd_vel`または車輪指令を購読し、`/tf`、`/odom`、IMU、LiDAR、真値を配信する。
5. **GPUセンサー**: 深度／ID RenderTextureとCompute Shaderから点群・色分類を生成する。
6. **競技判定**: ビンゴ占有、積み順、得点、Vゴール、壁接触、場外を自動判定する。
7. **校正**: 床・車輪・球の摩擦、反発、モータ応答を実測値へ合わせる。
8. **高速試験**: 描画なしバッチと描画ありGPU試験を分け、seed違い100試合を評価する。

## 精度合格条件

- JSON寸法とColliderの差: 1 mm以下
- 無負荷直進・横移動・旋回の速度誤差: 5%以下
- 走行軌跡の実機との差: 50 mm以下から開始し、最終10 mm以下
- 球投入100回の成功率: 95%以上
- 同じ初期条件での試験結果を記録し、Unity／実機双方の分布で比較する

摩擦係数とオムニローラ特性はルールPDFからは得られないため、現在値は仮値である。「正確な
シミュレーション」の完成には実機の質量・重心・慣性、床と車輪の摩擦、モータ特性の測定が必須。
