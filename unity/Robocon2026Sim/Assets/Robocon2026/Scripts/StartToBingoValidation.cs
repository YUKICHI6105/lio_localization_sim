using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using UnityEngine;

namespace Robocon2026.Simulation
{
    /// <summary>
    /// Physics baseline for the left start -> two-note pickup -> slalom -> bingo mission.
    /// Geometry and mission poses come from the engine-neutral field JSON.
    /// </summary>
    public sealed class StartToBingoValidation : MonoBehaviour
    {
        // 2026-07-31再設計: 「start→pickup(固定1組)→スラローム→bingo→start」という単一の
        // 往復ミッションから、ノーツ6個を2個ずつ回収する5組([1,2][2,3][3,4][4,5][5,6]、
        // 1-indexed)それぞれについて、3種類の独立した区間(leg)を学習する構成へ変更した。
        // ユーザーとの合意事項: (1)5組は独立した学習用ミッション(1回の連鎖した実戦フロー
        // ではない)、(2)学習対象はstart→pickup_i(リスタート用)・pickup_i→bingo共通到着点・
        // bingo共通到着点→pickup_iの計15パターン、(3)「bingo→start」は対象外、
        // (4)各leg終点(start/pickup_i/bingo共通到着点)では既存のpickup待機と同じ基準
        // (位置5mm・速度10mm/s)で必ず静止してから次のlegへ進む。
        private enum LegType { StartToPickup, PickupToBingo, BingoToPickup }

        private const int PairCount = 5;
        private readonly List<Vector2> route = new();
        // Return-leg(bingo→pickup_i)クリアランス経由点。cross_lowはバッフル線
        // (x=-0.779)を低く跨ぎ、bypassはノーツ列より高い位置を通ってから対象pickup_iへ
        // 降りる(bypassのxはleg構築時にpickup_iのxへ差し替える)。
        private const float ReturnCrossLowX = -1.1f;
        private const float ReturnCrossLowY = 0.5f;
        private const float ReturnPickupBypassY = 1.1f;
        // 5組共通のノード位置(mission JSONの5ミッションはstart/slalom_waypoints/finishが
        // 全て同一のためStart()で1回だけ読み取る)。pickupPoints[i]はi番目の組(0-indexed、
        // 1-indexedの[1,2]〜[5,6]に対応)のpickup座標。
        private Vector2 startPoint;
        private Vector2 bingoPoint;
        private readonly List<Vector2> slalomWaypoints = new();
        private readonly Vector2[] pickupPoints = new Vector2[PairCount];
        private double pickupDwellSecondsCommon;
        // 現在アクティブなlegの種類・対象組・識別文字列(ILCテーブルのキー、ログのleg=欄)。
        private LegType currentLegType = LegType.StartToPickup;
        private int currentPairIndex;
        private string currentLegId = "";
        // legの巡回スケジュール: bingo↔pickup_iをi=0→1→2→3→4→0→...と往復させるのを基本とし、
        // 5往復(=pickup_iへの到着5回)ごとにstartへテレポートしてstart→pickup_iを1回
        // 挟む(実戦では稀だがリスタート用に全組を学習させるため)。pairCounterは両方の
        // 目的で共有し、テレポートの有無に関わらず単調に進めることで、セッションを長く
        // 走らせれば5組全てがstart→pickup_iでも学習される。
        private int pairCounter;
        private int legsSincePickupArrivalReset;
        private readonly List<float> segmentDurations = new();
        // segmentSpinDurations[i]: 区間iの並進が終わった直後、次の区間の並進が
        // 始まる前に挟むその場スピンの長さ(秒、0なら無し)。EnableExtraSpins
        // 参照。
        private readonly List<float> segmentSpinDurations = new();
        // 2026-07-31追加: mission JSONのrequired_zero_speed_points(start/pickup/finish)は
        // その3点でのみ速度0を要求しており、スラローム経由点(waypoint)では要求していない。
        // 旧実装は全waypointで速度0にしていた(無駄な完全停止)。segmentEntryBlend[i]/
        // segmentExitBlend[i]は区間iの開始・終了時点でのu空間(正規化0〜1)上の速度で、
        // 経路形状(直線waypoint列)は変えずタイミングだけを変える。QuinticBlend参照。
        private readonly List<float> segmentEntryBlend = new();
        private readonly List<float> segmentExitBlend = new();
        // 通過点での目標速度(maxSpeedに対する割合)。方向が急に変わる経路形状のため、
        // 保守的に低めから始める(過去のスプライン実装がスラロームのジグザグで大きく
        // 逸脱した経緯を踏まえ、まずは「完全に止まらない」ことを優先し速度は抑える)。
        private const float CornerSpeedFraction = 0.2f;
        // u空間での速度の安全上限。v0=v1=0の通常区間でも導関数のピークは1.875程度
        // (BuildTimelineのコメント参照)になるため、それと同程度以下に抑える。
        private const float MaxCornerBlendVelocity = 1.5f;
        // 各区間の終端で向くべきヨー(Unity座標系、rad)。区間iはsegmentYawEnd[i]
        // まで、位置と同じタイムライン(MinimumJerk, segmentDurations[i])でなめらかに
        // 回転する。開始時の向き(区間0の始点)は機体のスポーン姿勢(yaw=0)。
        private readonly List<float> segmentYawEnd = new();
        // 現在のラップの区間0が始まる時点で向いているべきヨー(Unity座標系、rad)。
        // 初回は機体のスポーン姿勢(0)。ラップ反転時はStartNextLapで前ラップの
        // 最終区間の向きを引き継ぎ、ラップ間で不連続なヨーの飛びが起きないようにする。
        private float startYawRad;
        // 2026-07-31再設計: 3輪オムニホイール(全方向移動)のため、機体の向きと進行方向は
        // 独立にできる。以前はsegmentYawEndを各区間の進行方向から算出しており、
        // スラロームのジグザグに合わせて無駄に向きを変え続けていた。ユーザー指示により、
        // 機体の基準辺が向くべき方向を「ノーツ側」「ビンゴ側」の2値のみに固定し、
        // スラローム通過中(pickup後〜finish進入前)は向きを一切変えない。値はBuildRoute()で
        // 一度だけ、旧実装のsegment 0(start→pickup)・最終segment(slalom末尾→finish)の
        // 進行方向と同じ式で算出する(実機の吸入・排出機構の向きが決まるまでの暫定値)。
        private float notesFacingYawRad;
        private float bingoFacingYawRad;
        private Rigidbody body = null!;
        private UnityRosSensorPublisher sensorPublisher = null!;
        private Transform robot = null!;
        private Transform[] wheels = Array.Empty<Transform>();
        private CollisionRecorder collisionRecorder = null!;
        private RobotDefinition robotDefinition = null!;
        private float startTime;
        private float routeDuration;
        private float wheelRadius;
        // routeDoneになった時刻(-1は未到達)。ヨー収束待ちが万一収束しない場合に
        // 無限ハングしないためのタイムアウト起点(下記FixedUpdate参照)。
        private float routeDoneSinceTime = -1f;
        private Vector2 estimatedPosition;
        private float squaredErrorSum;
        private float maxOdometryError;
        private int samples;
        // Phase 0基準計測用(docs/requirements/経路追従.md §15/§19): trackingErrorは
        // 「参照軌道(target)と真値(truth)の差」であり、odometryError(単純デッドレコニング
        // 外挿の自己検算)や自己位置推定誤差(state_estimate/odom_fast vs ground truth、
        // 既存のconsolidate_stage_log.pyが別途計測)とは異なる第三の指標。
        private double trackingSquaredErrorSum;
        private float maxTrackingError;
        private int trackingSamples;
        private Vector3 previousBodyVelocity;
        private bool havePreviousBodyVelocity;
        private float maxAccelerationMagnitude;
        public float TrackingRmseMillimetres { get; private set; }
        public float TrackingMaxErrorMillimetres { get; private set; }
        public float MaxAccelerationMPerSSquared { get; private set; }
        public float TerminalYawErrorDegrees { get; private set; }
        // ILC(反復学習制御、経路追従.md §12)。実機がまだ用意できていないため、
        // 実機ではなくUnityシミュレーション自体が持つ再現性のある誤差(PDサーボの
        // 追従遅れ・衝突外乱・物理エンジンの数値誤差)を学習対象にする
        // (2026-07-31、docs/experiment_history/25番参照)。
        //
        // 経路進行度(§12.2は時刻より進行度を推奨)は、waypointが毎ラップ同一で
        // タイムラインも決定的(BuildTimelineが同じroute/速度制限から常に同じ
        // duration列を出す)なため、経過時間の正規化(0〜1)をそのまま進行度として
        // 使える。15種類のleg(3種類×5組)はそれぞれ経路形状が異なる独立したミッション
        // なので、feedforwardテーブル・カウンタはleg単位(legIdをキーとする辞書)で
        // 別々に学習する(2026-07-31、経路のバリエーション追加に伴い固定長配列から
        // 辞書ベースへ一般化。将来アーム関連legが増えても配列サイズを変えずに済む)。
        private const bool EnableIlc = true;
        private const int IlcBinCount = 100;
        // 位置誤差1mあたりの加速度feedforward補正[m/s^2]への変換ゲイン。大きすぎると
        // 発振するおそれがあるため保守的な値から始める(§12.5「学習率に上限」)。
        private const float IlcLearningRate = 6.0f;
        private const float IlcMaxCorrectionPerUpdateMPerSSquared = 0.3f;
        private const float IlcMaxTotalCorrectionMPerSSquared = 1.5f;
        // Unity座標系(x,z平面)でのfeedforward補正テーブル(legIdごと)。ラップをまたいで
        // 蓄積するため、segmentDurations等と違いAdvanceToNextLeg()ではクリアしない。
        private readonly Dictionary<string, Vector2[]> ilcCorrectionByLeg = new();
        // §12.5「性能悪化時にロールバック」用の直近最良テーブルとその時のRMSE(legIdごと)。
        private readonly Dictionary<string, Vector2[]> ilcBestCorrectionByLeg = new();
        private readonly Dictionary<string, float> ilcBestRmseByLeg = new();
        private readonly Dictionary<string, int> ilcUpdateCountByLeg = new();
        private readonly Dictionary<string, int> ilcSkippedCountByLeg = new();
        private readonly Dictionary<string, int> ilcRollbackCountByLeg = new();
        // 今ラップの進行度ビンごとの誤差蓄積(Complete()で更新に使った後クリアする)。
        private readonly Vector2[] ilcTrialErrorSum = new Vector2[IlcBinCount];
        private readonly int[] ilcTrialErrorCount = new int[IlcBinCount];

        // legId用のテーブル一式を(無ければ)確保する。5組×3種類=15leg分、初回参照時に
        // 遅延生成する。
        private Vector2[] GetOrCreateIlcTable(string legId)
        {
            if (!ilcCorrectionByLeg.TryGetValue(legId, out var table))
            {
                table = new Vector2[IlcBinCount];
                ilcCorrectionByLeg[legId] = table;
                ilcBestCorrectionByLeg[legId] = new Vector2[IlcBinCount];
                ilcBestRmseByLeg[legId] = float.PositiveInfinity;
                ilcUpdateCountByLeg[legId] = 0;
                ilcSkippedCountByLeg[legId] = 0;
                ilcRollbackCountByLeg[legId] = 0;
            }
            return table;
        }
        // 2026-07-31追加: ILC学習テーブルはこれまでPlay session中のメモリ上にしか
        // 存在せず、Playを止めると全て消えていた。ユーザー指示により、ラップ完了ごとに
        // Application.persistentDataPath(Unity標準の書き込み可能領域、Assets外・git管理
        // 対象外)へJSON保存し、次回Start()で読み込んで学習を引き継げるようにする。
        // プロジェクト直下にResetIlcLearningFlagFileNameという名前のファイルを置くと、
        // 保存済み状態を無視して(削除して)まっさらから再学習する(既存の
        // DisableNotesExperimentFlagと同じ「フラグファイル」方式に合わせた)。
        private const string IlcStateFileName = "ilc_learning_state.json";
        private const string ResetIlcLearningFlagFileName = "ResetIlcLearning.flag";
        // 衝突検知による即時中断・同方向リトライの回数(RESULT行としては記録されない、
        // 完走扱いにしない破棄された試行)。
        public int CollisionAbortCount { get; private set; }
        // Repeating laps for the full 60-second window instead of one outbound leg plus an idle
        // stationary hold maximises the useful (moving) data collected per run. This was
        // previously disabled because reversing the route sent the chassis back through the
        // pickup geometry without an intake model, colliding with notes that aren't part of the
        // specified mission; IgnoreNoteContactForLocalizationBaseline (see BuildRobot) now
        // removes exactly that contact force, so repeated laps no longer hit it. Re-enabling
        // this is contingent on notes staying non-contact -- see BuildRobot's contact-force note
        // for the plan to eventually re-enable it once an intake model exists.
        private const bool RepeatLaps = true;
        // 2026-07-30: 自己位置推定の残差調査で「旋回ダイナミクス」由来と誤報告した
        // 事象が、実際にはCSV列の取り違えによる誤りで、本ロボットは
        // body.constraints=FreezeRotation(全軸凍結)によりこれまで一度も物理的に
        // 回転していなかったことが判明した。実旋回下での自己位置推定挙動を初めて
        // 検証するためのフラグ。trueでヨー軸のみ回転を許可する。
        //
        // 最初の実装(進行方向を向くAddTorque PD)は、区間の切り替わりごとに
        // 目標ヨーが瞬時にスナップし、並進の立ち上がりと同時に大きな旋回要求が
        // 発生して自己位置推定が完全にロストする事象を引き起こした
        // (t=10〜15s付近で位置誤差が数十〜150m規模まで発散、全点非対応)。
        // ユーザー判断により、フィードバック制御(追従)を使わず、位置と同じ
        // quintic minimum-jerkタイムラインで事前に決めたヨーレートを
        // body.angularVelocityへ直接与えるopen-loop方式に変更した(詳細は
        // FixedUpdate内のコメント参照。当初MoveRotationを使ったところ
        // angularVelocityが更新されずIMU/ground truthが旋回を感知しない
        // 別のバグを踏んだため、angularVelocity直接代入方式へ再修正した)。
        // 追従のバグと自己位置推定側の問題を切り分けるための実験的措置。
        private const bool EnableYawRotation = true;
        // 2026-07-31: 位置サーボは既定で①の外挿(/odom_fast)を追従目標に使う
        // closed-loopだが、これだと自己位置推定の誤差がそのまま制御を揺らし、
        // 「推定精度だけ」を測りたい試験の解釈を難しくする。falseにすると
        // FixedUpdate内のcontrolPosition/controlVelocityが常にground truth
        // (body.position/linearVelocity)になり、ミッションサーボは推定値を
        // 一切参照しないopen-loopになる(推定自体は並行して動き続け、真値との
        // 差だけを純粋に測れる)。ユーザー指示により旋回多め試験ではfalse。
        private const bool EnablePositionClosedLoop = true;
        // 各区間の到達直後に、その場で追加の1回転(2π)を挟む。実運用のヨー
        // レートよりかなり激しい旋回を人為的に発生させ、自己位置推定の
        // 旋回耐性を単独で検証するための実験用フラグ(ユーザー指示による
        // 「旋回多め」試験)。最終区間の後には挿入しない(フィニッシュ判定を
        // 素直に保つため)。
        // 経路追従.md §17.7の方針により、検証用スピンは通常の最速プリセット
        // 実行から分離しValidationModeでのみ有効にする想定(将来対応、現状は
        // 既定falseのまま既存の区間ベース経路に組み込んでいる)。
        private const bool EnableExtraSpins = false;
        private const float SpinDurationSeconds = 2.0f;
        private const float SpinRadians = 2f * Mathf.PI;
        // 到着判定に追加したヨー条件(下記FixedUpdate参照)の許容値。位置5mm・
        // 速度10mm/sという既存の閾値と釣り合う厳しさとして、経路追従.md §11.3の
        // 目安(yaw_error/yaw_rateとも「specified tolerance」)を仮に具体化した値。
        private const float ArrivalYawErrorToleranceDegrees = 2.0f;
        private const float ArrivalYawRateToleranceDegreesPerSecond = 5.0f;
        // Match the real startup sequence: keep the chassis stationary while the IMU prior and
        // first LiDAR factors settle, then run the complete mission. Previously the robot moved
        // after only 0.5 s, so the reported maximum mixed filter startup convergence into the
        // motion requirement. The evaluator remains active throughout this warm-up; no samples
        // are hidden, and every moving sample is still included in the max-error result.
        private const float LocalizationWarmupSeconds = 10.0f;
        private int lapCount;

        public bool Finished { get; private set; }
        public Vector2 FinalFieldPosition { get; private set; }
        public float FinalTargetErrorMillimetres { get; private set; }
        public float OdometryRmseMillimetres { get; private set; }
        public float OdometryMaxErrorMillimetres { get; private set; }
        // 2026-07-31改称: pickup到着だけでなくbingo到着(配置)にも同じ静止判定基準・
        // 計測を使うため、"Pickup"限定の名前から一般化した。
        public float ArrivalPositionErrorMillimetres { get; private set; }
        public float ArrivalSpeedMillimetresPerSecond { get; private set; }
        public string CollisionNames => collisionRecorder == null ? "" : collisionRecorder.CollisionNames;

        // pathplannning ⇄ Unity 実機物理接続検証(2026-08-01計画): プロジェクト直下に
        // このファイルが存在する場合、15leg方式のミッションロジック(状態機械・quintic
        // 軌道・ILC)は一切走らせず、BuildRobot()で構築した同じロボット(同一チャシス・
        // 物理パラメータ・センサー)にPathplanningCmdVelBridgeを付けて/cmd_vel_body
        // (pathplannning_nodeが配信する機体座標系速度指令)で直接駆動する。既存の
        // ResetIlcLearningFlagFileName/DisableNotesExperimentFlagと同じフラグファイル
        // 方式。
        private const string PathplanningBridgeModeFlagFileName = "PathplanningBridgeMode.flag";
        private bool bridgeModeActive;

        private void Start()
        {
            var bridgeModeFlagPath = Path.GetFullPath(
                Path.Combine(Application.dataPath, "..", PathplanningBridgeModeFlagFileName));
            if (File.Exists(bridgeModeFlagPath))
            {
                bridgeModeActive = true;
                StartPathplanningBridgeMode();
                return;
            }

            LoadIlcState();
            var definition = GetComponent<RoboconFieldBuilder>().Definition;
            robotDefinition = definition.Robot;

            // 2026-07-31再設計: 5組(1-indexedで[1,2][2,3][3,4][4,5][5,6])は、
            // start/slalom_waypoints/finishが全て共通で、pickupだけが組ごとに異なる
            // (field JSON参照)。代表として先頭のミッションから共通のノード位置を
            // 読み取り、pickupだけ5組分すべて配列に格納する。
            var missions = definition.Missions.AllPairMissions;
            var template = missions[0];
            startPoint = ToVector(template.Start);
            bingoPoint = ToVector(template.Finish);
            slalomWaypoints.Clear();
            foreach (var point in template.SlalomWaypoints) slalomWaypoints.Add(ToVector(point));
            pickupDwellSecondsCommon = template.PickupDwellSeconds;
            for (var i = 0; i < PairCount; ++i) pickupPoints[i] = ToVector(missions[i].Pickup);

            // 2026-07-31再訂正: 当初notesFacingYawRadを「pickup→回収ノーツ方向」
            // (Atan2(-delta.y,delta.x)、当時90度)としていたが、これは「機体の前(local
            // +Z、transform.forward)がその方向を向く」角度であり、実際にノーツを回収する
            // 「決めた1辺」ではなかった。CreateRoundedTrianglePrismのコーナー配列
            // ((inradius,-side/2),(inradius,side/2),(-circumradius,0))を確認すると、
            // 平らな辺はlocal+X側にあり、forward(local+Z)とは90度ずれている。実測
            // (MeshCollider頂点をtransform.TransformVectorで変換)で確認したところ、
            // 平らな辺が方向Dを向くために必要な角度は「forwardがDを向く角度から90度引いた
            // 値」だった(Atan2(-delta.y,delta.x) - 90度、フィールド成分で書き直すと
            // Atan2(-delta.x,-delta.y))。5組ともpickup.y・ノーツ列のyは共通なので、
            // どの組で計算しても同じ値になる(実測: 真南のため0度=無回転)。
            var noteA = definition.Notes.Orange[template.PickupNoteIndices[0]];
            var noteB = definition.Notes.Orange[template.PickupNoteIndices[1]];
            var noteMidpoint = new Vector2(
                (float)((noteA[0] + noteB[0]) / 2.0), (float)((noteA[1] + noteB[1]) / 2.0));
            var pickupToNotes = noteMidpoint - pickupPoints[0];
            notesFacingYawRad = Mathf.Atan2(-pickupToNotes.x, -pickupToNotes.y);
            // ビンゴ側の固定目標ヨー: 同じ「平らな辺が向く方向」の定義で、finish地点から
            // ビンゴ棚中心(bingo.Centre)への方向を使う。実測: finish=(2.316,0.494)に対し
            // 棚中心=(2.316,0.0)は真南(ノーツと同じ向き)で、こちらも0度になる。
            // Physics.ComputePenetrationで実測確認済み(Shelf_0/Post_2とも重なり無し、
            // 往路finish・復路の全経由点で衝突ゼロ)。
            var finishToBingo = new Vector2((float)definition.Bingo.Centre.X, (float)definition.Bingo.Centre.Y) -
                bingoPoint;
            bingoFacingYawRad = Mathf.Atan2(-finishToBingo.x, -finishToBingo.y);

            // 初回のみ: スポーン姿勢(BuildRobot参照)をnotesFacingYawRadに揃えておくため、
            // 最初のleg(start→pickup1)の基準ヨーもここで同じ値にしておく。以後は
            // AdvanceToNextLeg()が前legの終端ヨーを引き継ぐため、ここでの代入は
            // 上書きされない初回限定の値。
            startYawRad = notesFacingYawRad;
            currentLegType = LegType.StartToPickup;
            currentPairIndex = 0;
            BuildLegRoute();
            BuildRobot();
            sensorPublisher = robot.GetComponent<UnityRosSensorPublisher>();
            estimatedPosition = route[0];
            startTime = Time.time + LocalizationWarmupSeconds;
            Debug.Log(
                $"[StartToBingo] leg={currentLegId} start={route[0]}, end={route[^1]}, " +
                $"wheel={wheelRadius * 2000f:F0}mm, duration={routeDuration:F2}s");
        }

        // pathplannning ⇄ Unity 実機物理接続検証(2026-08-01計画): BuildRobot()が
        // 必要とするroute[0](スポーン位置)・notesFacingYawRad(スポーン姿勢)だけを
        // 用意し、ミッション状態機械は一切開始しない。FixedUpdateは既存の
        // `route.Count < 2`ガードにより自動的に無効化される(念のためbridgeModeActive
        // でも明示的にガードする)。スポーン位置・姿勢は`pathplannning`側の
        // config/trajectory_spec_sample.yamlの始点(-2.419, 1.354)・fixed_yaw_rad(0)と
        // 一致させ、ROS2側の~/start_trajectoryサービスの§30開始条件を満たせるようにする。
        private void StartPathplanningBridgeMode()
        {
            var definition = GetComponent<RoboconFieldBuilder>().Definition;
            robotDefinition = definition.Robot;

            route.Clear();
            route.Add(new Vector2(-1.919f, 1.3715f));
            notesFacingYawRad = 0f;

            BuildRobot();
            robot.gameObject.AddComponent<PathplanningCmdVelBridge>();
            Debug.Log(
                $"[StartToBingo] {PathplanningBridgeModeFlagFileName} present: mission logic " +
                "disabled, driving via /cmd_vel_body only.");
        }

        // 現在のcurrentLegType/currentPairIndexに応じてrouteのwaypointを構築し、
        // タイムラインを組み立てる。5組×3種類=15パターンいずれも、経路の形(直線
        // waypoint列)はここで確定する。旧BuildRoute()と旧StartNextLap()の経路構築部分を
        // 統合したもの。
        private void BuildLegRoute()
        {
            route.Clear();
            var pickup = pickupPoints[currentPairIndex];
            switch (currentLegType)
            {
                case LegType.StartToPickup:
                    route.Add(startPoint);
                    route.Add(pickup);
                    currentLegId = $"start_to_pickup{currentPairIndex + 1}";
                    break;
                case LegType.PickupToBingo:
                    route.Add(pickup);
                    route.AddRange(slalomWaypoints);
                    route.Add(bingoPoint);
                    currentLegId = $"pickup{currentPairIndex + 1}_to_bingo";
                    break;
                case LegType.BingoToPickup:
                    // 旧実装の「復路(bingo→start)」と同じバッフル回避パターン
                    // (スラローム逆順→crossLow→bypass)を流用し、終点をpickup_iへ
                    // 差し替える。bypassのxはpickup_iのxに合わせる(旧実装ではstartの
                    // xに固定だったのを一般化した)。
                    route.Add(bingoPoint);
                    for (var i = slalomWaypoints.Count - 1; i >= 0; --i) route.Add(slalomWaypoints[i]);
                    route.Add(new Vector2(ReturnCrossLowX, ReturnCrossLowY));
                    route.Add(new Vector2(pickup.x, ReturnPickupBypassY));
                    route.Add(pickup);
                    currentLegId = $"bingo_to_pickup{currentPairIndex + 1}";
                    break;
            }

            segmentDurations.Clear();
            segmentYawEnd.Clear();
            segmentSpinDurations.Clear();
            segmentEntryBlend.Clear();
            segmentExitBlend.Clear();
            routeDuration = 0f;
            BuildTimeline();
        }

        // legの起点・終点それぞれで機体が向くべきヨー。start/pickupはノーツ側
        // (notesFacingYawRad)、bingoはビンゴ側(bingoFacingYawRad)。
        private float YawForLegEndpoint(bool isOrigin) => currentLegType switch
        {
            LegType.StartToPickup => notesFacingYawRad, // 起点(start)・終点(pickup)とも同じ
            LegType.PickupToBingo => isOrigin ? notesFacingYawRad : bingoFacingYawRad,
            LegType.BingoToPickup => isOrigin ? bingoFacingYawRad : notesFacingYawRad,
            _ => notesFacingYawRad,
        };

        // Per-segment minimum-jerk durations for the current route ordering. Split out from
        // BuildRoute so a reversed lap can recompute timing without re-adding waypoints. The
        // pickup dwell is only inserted on the outbound leg (segment 0 -> pickup); a reversed
        // lap has no pickup and gets no dwell.
        //
        // 2026-07-31訂正: 経路追従.md Phase 1で、この区間ごと独立5次多項式の代わりに
        // C¹連続なCatmull-Romスプライン+forward-backward速度計画を試したが、
        // (a) 経路途中で完全停止する不具合、(b) その前段の自然3次スプライン版では
        // スラロームのジグザグ状waypointで2m超の激しいオーバーシュートが起き
        // 中央壁へ衝突、という2つの問題に連続して遭遇した。ユーザー判断
        // (「スプラインより最終的にILCで実機調整するので...スプラインやっていても
        // 意味がない」)により、シミュレーション内での経路形状最適化への投資を
        // 中止し、この安全な実装へ全面的に戻した。経路形状に依存しない知見
        // (到着判定のヨー条件・符号修正・Phase 0計装)だけは維持している
        // (docs/experiment_history参照)。
        private void BuildTimeline()
        {
            var maxSpeed = (float)robotDefinition.TargetLimits.MaxSpeed;
            var maxAcceleration = (float)robotDefinition.TargetLimits.MaxAcceleration;
            for (var i = 0; i < route.Count - 1; ++i)
            {
                var distance = Vector2.Distance(route[i], route[i + 1]);
                // Quintic minimum-jerk peak factors: v=1.875*d/T, a=5.774*d/T^2.
                var speedLimited = 1.875f * distance / maxSpeed;
                var accelerationLimited = Mathf.Sqrt(5.774f * distance / maxAcceleration);
                var duration = 1.08f * Mathf.Max(speedLimited, accelerationLimited);
                segmentDurations.Add(duration);
                routeDuration += duration;
                // required_zero_speed_points(start/pickup/finish)に該当しないwaypointでは
                // 完全停止せず、cornerSpeedで通過する。u空間の速度に変換して
                // QuinticBlend(下記EvaluateRoute参照)の境界条件として使う。同じ
                // cornerSpeed定数を隣り合う区間の入り・抜けで共有するため、実速度としては
                // 連続になる(進行方向はwaypointで折れ線のまま瞬時に変わる、経路の形は
                // 直線waypoint列を維持しスプライン化はしない)。
                var cornerSpeed = CornerSpeedFraction * maxSpeed;
                var entryIsRequiredZero = IsRequiredZeroSpeedWaypoint(i);
                var exitIsRequiredZero = IsRequiredZeroSpeedWaypoint(i + 1);
                var entryBlend = entryIsRequiredZero
                    ? 0f
                    : Mathf.Min(cornerSpeed * duration / distance, MaxCornerBlendVelocity);
                var exitBlend = exitIsRequiredZero
                    ? 0f
                    : Mathf.Min(cornerSpeed * duration / distance, MaxCornerBlendVelocity);
                segmentEntryBlend.Add(entryBlend);
                segmentExitBlend.Add(exitBlend);
                // 2026-07-31再設計: 区間の進行方向ではなく、leg起点/終点の固定ヨーの
                // どちらか一方のみを使う(YawForLegEndpoint参照)。最終区間で起点側から
                // 終点側の向きへ回転し、それ以外は向きを一切変えない(ヨーレート0)。
                // segmentYawEnd[i]が直前の値と同じならEvaluateRoute内でyawDelta=0となり、
                // 自然に無回転区間になる。
                var isLastSegment = i == route.Count - 2;
                var holdYawRad = YawForLegEndpoint(isOrigin: true);
                var targetYawRad = YawForLegEndpoint(isOrigin: false);
                segmentYawEnd.Add(isLastSegment ? targetYawRad : holdYawRad);

                var spinDuration = EnableExtraSpins && !isLastSegment ? SpinDurationSeconds : 0f;
                segmentSpinDurations.Add(spinDuration);
                routeDuration += spinDuration;
            }
        }

        // leg構造(2026-07-31)では、各legの起点・終点(route[0]・route[^1])だけが
        // mission JSONのrequired_zero_speed_points相当(必ず静止する場所)で、
        // それ以外(スラローム経由点・bypass等)は通過点。
        private bool IsRequiredZeroSpeedWaypoint(int waypointIndex) =>
            waypointIndex == 0 || waypointIndex == route.Count - 1;

        private void FixedUpdate()
        {
            // pathplannning ⇄ Unity 実機物理接続検証(2026-08-01計画): bridgeMode中は
            // PathplanningCmdVelBridgeがRigidbodyを直接駆動するため、ミッション状態機械は
            // 一切走らせない(route.Count<2による暗黙のガードに加え、明示的にも早期return)。
            if (bridgeModeActive) return;

            // Script recompilation during Play mode clears non-serialized route data.
            // A fresh Play run rebuilds it in Start(); avoid a hot-reload error storm meanwhile.
            if (Finished || body == null || route.Count < 2) return;
            var elapsed = Time.time - startTime;
            if (elapsed < 0f) return;

            // 2026-07-31追加: 「接触するほど悪化する」というユーザー指摘により、衝突を
            // 検知した瞬間、そのlegの残りを走り切らせず直ちに中断し、同じleg(同じ
            // currentLegType/currentPairIndex)の開始状態からやり直す。ILCはこの破棄した
            // 試行の誤差を一切学習に使わない(蓄積前に丸ごとクリアする)。
            if (!string.IsNullOrEmpty(CollisionNames))
            {
                AbortAndRetryDueToCollision();
                return;
            }

            var effectiveElapsed = elapsed;
            EvaluateRoute(
                effectiveElapsed, out var target, out var targetVelocity, out var targetYawRad,
                out var targetYawRateRad, out var routeDone);

            var targetUnity = FieldCoordinates.ToUnity(target.x, target.y, 0.065);
            var velocityUnity = new Vector3(-targetVelocity.y, 0f, targetVelocity.x);

            // Closed-loop control: drive the mission servo off the localisation stack's own
            // estimate (/odom_fast via sensorPublisher), not ground truth. body.position/
            // linearVelocity remain the fallback only until the first estimate arrives
            // (~1s startup, before node ① has a /state_estimate to extrapolate from).
            Vector3 controlPosition;
            Vector3 controlVelocity;
            if (EnablePositionClosedLoop && sensorPublisher != null && sensorPublisher.HasOdomEstimate)
            {
                var estPos = sensorPublisher.EstimatedFieldPosition;
                controlPosition = FieldCoordinates.ToUnity(estPos.x, estPos.y, 0.065);
                var estVel = sensorPublisher.EstimatedFieldVelocityWorld;
                controlVelocity = new Vector3(-estVel.y, 0f, estVel.x);
            }
            else
            {
                controlPosition = body.position;
                controlVelocity = body.linearVelocity;
            }

            var positionError = targetUnity - controlPosition;
            positionError.y = 0f;
            var velocityError = velocityUnity - controlVelocity;
            velocityError.y = 0f;

            // ILC(経路追従.md §12): 今回のleg(currentLegId)の経路進行度ビンへ位置誤差
            // (Unity x,z平面)を蓄積する。進行度はwaypoint・タイムラインが毎回決定的
            // なため経過時間の正規化で代用できる(フィールド宣言のコメント参照)。
            // routeDone後(ヨー収束待ち・静止待機)は「静止状態を維持する」純粋な位置調整の
            // 問題であり、ILCが対象とする「移動中の追従遅れ」とは別物(2026-07-31発見、
            // pickup到達後の静止待機中もfeedforwardを適用し続けると純P制御の停止時平衡が
            // ずれ続け発散した経緯があるため、この間は蓄積もfeedforward適用も行わない)。
            var ilcActiveThisFrame = EnableIlc && !routeDone;
            var ilcProgress = routeDuration > 1e-6f ? Mathf.Clamp01(effectiveElapsed / routeDuration) : 0f;
            var ilcBin = Mathf.Clamp(Mathf.FloorToInt(ilcProgress * IlcBinCount), 0, IlcBinCount - 1);
            var ilcFeedforward = Vector3.zero;
            if (ilcActiveThisFrame)
            {
                var ilcErrorXz = new Vector2(positionError.x, positionError.z);
                ilcTrialErrorSum[ilcBin] += ilcErrorXz;
                ilcTrialErrorCount[ilcBin]++;
                var ilcCorrection = GetOrCreateIlcTable(currentLegId)[ilcBin];
                ilcFeedforward = new Vector3(ilcCorrection.x, 0f, ilcCorrection.y);
            }

            // A firm Cartesian servo is used instead of assuming a motor/gearing model.
            // Output remains capped by the mission's 0.5 G target acceleration.
            body.WakeUp();
            var force = body.mass * (ilcFeedforward + 60f * positionError + 12f * velocityError);
            var maxForce = body.mass * (float)robotDefinition.TargetLimits.MaxAcceleration;
            body.AddForce(Vector3.ClampMagnitude(force, maxForce), ForceMode.Force);

            if (EnableYawRotation)
            {
                // 2026-07-30訂正: body.MoveRotation()は非kinematicなRigidbodyでは
                // body.angularVelocityを正しく更新しないため、IMU/ground truthの
                // ジャイロが実際の回転を全く感知しない(常にバイアス+ノイズレベルの
                // まま)というバグを生んでいた。姿勢(transform.rotation)自体は
                // MoveRotationで正しく回るためground truthの"姿勢"は正しく見えるが、
                // angularVelocity由来のセンサ値だけが壊れており、これが3回の
                // 修正でも破綻が直らなかった真因だった。
                // 修正: 目標ヨーレート(位置のvelocityと同じ規約で解析的に求めた値、
                // EvaluateRoute参照)をangularVelocityへ直接代入する。物理エンジンが
                // これを毎ステップ積分して姿勢を更新するため、位置の力積分と
                // 同じ意味でopen-loopのまま、angularVelocityも正しく実態を反映する。
                body.angularVelocity = new Vector3(0f, targetYawRateRad, 0f);
            }

            estimatedPosition.x += body.linearVelocity.z * Time.fixedDeltaTime;
            estimatedPosition.y -= body.linearVelocity.x * Time.fixedDeltaTime;
            var truth = UnityToField(body.position);
            var odometryError = Vector2.Distance(estimatedPosition, truth);
            squaredErrorSum += odometryError * odometryError;
            maxOdometryError = Mathf.Max(maxOdometryError, odometryError);
            samples++;

            // Phase 0基準計測: 参照軌道(target)と真値(truth)の追従誤差、および実加速度
            // (指令力ではなく速度差分から実測)。
            var trackingError = Vector2.Distance(truth, target);
            trackingSquaredErrorSum += trackingError * trackingError;
            maxTrackingError = Mathf.Max(maxTrackingError, trackingError);
            trackingSamples++;
            if (havePreviousBodyVelocity)
            {
                var accel = (body.linearVelocity - previousBodyVelocity).magnitude / Time.fixedDeltaTime;
                maxAccelerationMagnitude = Mathf.Max(maxAccelerationMagnitude, accel);
            }
            previousBodyVelocity = body.linearVelocity;
            havePreviousBodyVelocity = true;

            RotateWheelVisuals(Time.fixedDeltaTime);

            // 2026-07-31: 到着判定は従来、位置誤差・並進速度のみを見ておりヨーを
            // 一切見ていなかった(Phase 0計測で発見、経路追従.md §11「精密停止
            // モード」が指摘する問題そのもの)。ヨー誤差・ヨーレートも到着条件に
            // 加える。
            // 符号規約: body.transform.forward = Unity_forward(θ) = (sinθ,0,cosθ)
            // (θ=Unity Y軸回転角)なので、Atan2(forward.x, forward.z) = θ そのもの。
            // これはtargetYawRad(=EvaluateRouteが返す値、body.angularVelocityへ
            // 直接代入している値と同じUnity角規約)と直接比較できる。符号を反転する
            // FieldYaw()(UnityRosSensorPublisher.cs、ROS配信用にfield frameへ
            // 変換する別規約)と混同しないこと(この関数の初期実装で混同していた
            // バグを本修正で解消した)。
            var actualYawRad = Mathf.Atan2(body.transform.forward.x, body.transform.forward.z);
            var yawErrorRad = Mathf.Repeat(actualYawRad - targetYawRad + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
            var yawErrorDeg = Mathf.Abs(yawErrorRad) * Mathf.Rad2Deg;
            var yawRateAbsDegPerSec = Mathf.Abs(body.angularVelocity.y) * Mathf.Rad2Deg;

            var finalError = Vector2.Distance(truth, route[^1]);
            var finalPlanarSpeed = new Vector2(body.linearVelocity.x, body.linearVelocity.z).magnitude;
            var positionSettled = routeDone && finalError <= 0.005f && finalPlanarSpeed <= 0.010f;
            if (positionSettled && routeDoneSinceTime < 0f)
                routeDoneSinceTime = Time.time;
            if (!positionSettled)
                routeDoneSinceTime = -1f;
            // 2026-07-31追加: 到着してすぐ完了させず、pickup_dwell_s秒だけ静止を維持して
            // から次のlegへ進む(実機の吸引・投入機構が動作する時間を模した待機。
            // ユーザー指摘により、pickup到着だけでなくbingo到着(配置)にも同じ待機を課す)。
            var dwellSatisfied = routeDoneSinceTime >= 0f &&
                Time.time - routeDoneSinceTime >= (float)pickupDwellSecondsCommon;

            var yawSettled = yawErrorDeg <= ArrivalYawErrorToleranceDegrees &&
                yawRateAbsDegPerSec <= ArrivalYawRateToleranceDegreesPerSecond;
            // ヨーはbody.angularVelocityへの直接代入によるopen-loop積分のため、
            // routeDone到達時点でのわずかな積算ずれがフィードバックなしでは
            // 二度と縮まらない場合がある(閉ループヨー制御は過去に破滅的発散を
            // 起こした実績があるため導入しない、上記EnableYawRotationのコメント
            // 参照)。ヨーが収束しないまま無期限にハングしないよう、位置・速度が
            // 収束済みのまま一定時間経過したら、ヨー未収束のまま到着を許可する
            // タイムアウトを設ける。
            const float yawSettleTimeoutSeconds = 2.0f;
            var yawTimedOut = positionSettled && routeDoneSinceTime >= 0f &&
                Time.time - routeDoneSinceTime >= yawSettleTimeoutSeconds;
            if (positionSettled && (yawSettled || yawTimedOut) && dwellSatisfied)
            {
                if (!yawSettled)
                {
                    Debug.LogWarning(
                        $"[StartToBingo] arrival yaw did not converge within " +
                        $"{yawSettleTimeoutSeconds:F1}s (yaw_error={yawErrorDeg:F2}deg, " +
                        $"yaw_rate={yawRateAbsDegPerSec:F2}deg/s); completing anyway to avoid a hang.");
                }
                TerminalYawErrorDegrees = yawErrorDeg;
                ArrivalPositionErrorMillimetres = finalError * 1000f;
                ArrivalSpeedMillimetresPerSecond = finalPlanarSpeed * 1000f;
                // legの終点の種類に応じて回収(pickup)か配置(bingo)のプレースホルダーを
                // 実行する。CaptureNotes/DeliverNotesとも実機構が決まるまでのplaceholder。
                if (currentLegType == LegType.PickupToBingo) DeliverNotes();
                else CaptureNotes();
                routeDoneSinceTime = -1f;
                Complete(truth);
            }
        }

        private void Complete(Vector2 truth)
        {
            OdometryRmseMillimetres = samples > 0 ? Mathf.Sqrt(squaredErrorSum / samples) * 1000f : 0f;
            OdometryMaxErrorMillimetres = maxOdometryError * 1000f;
            TrackingRmseMillimetres =
                trackingSamples > 0 ? Mathf.Sqrt((float)(trackingSquaredErrorSum / trackingSamples)) * 1000f : 0f;
            TrackingMaxErrorMillimetres = maxTrackingError * 1000f;
            MaxAccelerationMPerSSquared = maxAccelerationMagnitude;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            // 2026-07-31発見: ヨーはbody.angularVelocityへの直接代入によるopen-loop
            // 積分のため、到着タイムアウト(上記yawSettleTimeoutSeconds)でヨー未収束の
            // まま完了した場合、実姿勢の残差が全く補正されない。次ラップの計画は
            // startYawRad=segmentYawEnd[^1](理想値)からゼロで組み立てるため、この
            // 残差が是正されずラップをまたいでそのまま持ち越され、ラップを重ねる
            // ごとに単調に加算されていく(実測: terminal_yaw_errorが14ラップで
            // 0.07度から4.17度まで増加)。位置・速度と同様、ラップ境界で実姿勢を
            // 理想目標ヨーへスナップして残差を毎回ゼロへ戻す。
            if (segmentYawEnd.Count > 0)
                body.rotation = Quaternion.Euler(0f, segmentYawEnd[^1] * Mathf.Rad2Deg, 0f);

            UpdateIlc();
            SaveIlcState();

            // Debug.LogWarning、not Debug.Log: Unity MCPのUnity_GetConsoleLogsは
            // 実測でLog(info)レベルを一切拾わずWarning/Error止まりだった(2026-07-31発見、
            // unity-stage4-benchmark-operationsスキル参照)。MCP経由でこの行を確実に
            // 回収するためLogWarningにする。
            Debug.LogWarning(
                $"[StartToBingo RESULT lap={lapCount} leg={currentLegId}] " +
                $"final=({truth.x:F3},{truth.y:F3})m " +
                $"completion_time={Time.time - startTime:F3}s " +
                $"target_error={Vector2.Distance(truth, route[^1]) * 1000f:F1}mm " +
                $"terminal_yaw_error={TerminalYawErrorDegrees:F2}deg " +
                $"tracking_rmse={TrackingRmseMillimetres:F2}mm tracking_max={TrackingMaxErrorMillimetres:F2}mm " +
                $"max_accel={MaxAccelerationMPerSSquared:F2}mps2 " +
                $"wheel_odom_rmse={OdometryRmseMillimetres:F2}mm " +
                $"wheel_odom_max={OdometryMaxErrorMillimetres:F2}mm " +
                $"arrival_error={ArrivalPositionErrorMillimetres:F2}mm " +
                $"arrival_speed={ArrivalSpeedMillimetresPerSecond:F2}mm/s " +
                $"collisions={CollisionNames} " +
                $"ilc_update={ilcUpdateCountByLeg.GetValueOrDefault(currentLegId)} " +
                $"ilc_skip={ilcSkippedCountByLeg.GetValueOrDefault(currentLegId)} " +
                $"ilc_rollback={ilcRollbackCountByLeg.GetValueOrDefault(currentLegId)} " +
                $"collision_abort_total={CollisionAbortCount}");

            // collisions=はこのラップの分だけを表示するため、ログ・ILC判定で使い終えた
            // ここでクリアする(HashSetは元々Play開始からの累積で、ラップ単位ではなかった)。
            collisionRecorder?.ClearCollisions();

            if (RepeatLaps)
            {
                AdvanceToNextLeg();
                return;
            }
            Finished = true;
            FinalFieldPosition = truth;
            FinalTargetErrorMillimetres = Vector2.Distance(truth, route[^1]) * 1000f;
        }

        // ILC更新(経路追従.md §12)。今しがた終わったleg(currentLegId、
        // AdvanceToNextLeg()での切り替えより前に呼ばれる)のビン別誤差平均を使って
        // feedforwardテーブルを更新する。15種類のlegはそれぞれ別ミッションとして
        // 別々に学習する。
        private void UpdateIlc()
        {
            UpdateIlcForLeg(currentLegId);
            Array.Clear(ilcTrialErrorSum, 0, IlcBinCount);
            Array.Clear(ilcTrialErrorCount, 0, IlcBinCount);
        }

        private void UpdateIlcForLeg(string legId)
        {
            if (!EnableIlc) return;
            var table = GetOrCreateIlcTable(legId);
            var bestTable = ilcBestCorrectionByLeg[legId];
            var bestRmse = ilcBestRmseByLeg[legId];
            // §12.4「学習禁止条件」: 衝突があった試行はILC更新に使わない。
            // (自己位置推定の大きなジャンプ・ICP異常等の他条件は、今回のスコープでは
            // 本体側にまだ相当する検出手段が無いため、実装済みの衝突検出のみで判定する。)
            if (!string.IsNullOrEmpty(CollisionNames))
            {
                ilcSkippedCountByLeg[legId] = ilcSkippedCountByLeg[legId] + 1;
                Debug.LogWarning($"[StartToBingo] ILC({legId}): skipping update this lap (collision occurred).");
                return;
            }
            // §12.5「性能悪化時にロールバック」: 直近の最良テーブルより明確に
            // 悪化していれば、今回のビン更新はせず直近最良へ戻す。
            if (bestRmse < float.PositiveInfinity && TrackingRmseMillimetres > bestRmse * 1.1f)
            {
                Array.Copy(bestTable, table, IlcBinCount);
                ilcRollbackCountByLeg[legId] = ilcRollbackCountByLeg[legId] + 1;
                Debug.LogWarning(
                    $"[StartToBingo] ILC({legId}): rolled back to best-known feedforward table " +
                    $"(this lap tracking_rmse={TrackingRmseMillimetres:F2}mm > best*1.1={bestRmse * 1.1f:F2}mm).");
                return;
            }
            if (TrackingRmseMillimetres < bestRmse)
            {
                ilcBestRmseByLeg[legId] = TrackingRmseMillimetres;
                Array.Copy(table, bestTable, IlcBinCount);
            }
            // 更新式(§12.2): u_{k+1}(s) = u_k(s) + L*e_k(s)。学習率・1回あたりの
            // 補正量・累積補正量にそれぞれ上限を設ける(§12.5)。
            for (var i = 0; i < IlcBinCount; ++i)
            {
                if (ilcTrialErrorCount[i] == 0) continue;
                var meanError = ilcTrialErrorSum[i] / ilcTrialErrorCount[i];
                var delta = Vector2.ClampMagnitude(IlcLearningRate * meanError, IlcMaxCorrectionPerUpdateMPerSSquared);
                table[i] = Vector2.ClampMagnitude(table[i] + delta, IlcMaxTotalCorrectionMPerSSquared);
            }
            ilcUpdateCountByLeg[legId] = ilcUpdateCountByLeg[legId] + 1;
        }

        // JSON化用のフィールドを持つだけのプレーンなデータクラス。Vector2[]をそのまま
        // Newtonsoftへ渡さずfloat[]へ分解しているのは、UnityEngine型のシリアライズ設定
        // (JsonUtility/Newtonsoftどちらの既定でも挙動差が出やすい)に依存しないため。
        // legIdごとの学習状態。Vector2[]をそのままNewtonsoftへ渡さずfloat[]へ分解している
        // のは、UnityEngine型のシリアライズ設定(JsonUtility/Newtonsoftどちらの既定でも
        // 挙動差が出やすい)に依存しないため。
        [Serializable]
        private sealed class LegIlcState
        {
            public float[] CorrectionX = Array.Empty<float>();
            public float[] CorrectionY = Array.Empty<float>();
            public float[] BestCorrectionX = Array.Empty<float>();
            public float[] BestCorrectionY = Array.Empty<float>();
            public float BestRmse = float.PositiveInfinity;
            public int UpdateCount;
            public int SkippedCount;
            public int RollbackCount;
        }

        [Serializable]
        private sealed class IlcPersistedState
        {
            public Dictionary<string, LegIlcState> Legs = new();
        }

        private static string IlcStateFilePath =>
            Path.Combine(Application.persistentDataPath, IlcStateFileName);

        private static float[] ExtractComponent(Vector2[] source, bool x)
        {
            var result = new float[source.Length];
            for (var i = 0; i < source.Length; ++i) result[i] = x ? source[i].x : source[i].y;
            return result;
        }

        private static void ApplyComponents(Vector2[] destination, float[] xs, float[] ys)
        {
            var count = Mathf.Min(destination.Length, Mathf.Min(xs.Length, ys.Length));
            for (var i = 0; i < count; ++i) destination[i] = new Vector2(xs[i], ys[i]);
        }

        // 2026-07-31追加: ラップ完了ごとにILC学習テーブルをApplication.persistentDataPath
        // (Unity標準の書き込み可能領域、git管理対象外)へJSON保存する。以前はPlay session
        // 内のメモリにしか無く、停止すると学習が全て消えていた(ユーザー指摘)。
        // 2026-07-31再設計: 15種類のleg分をlegIdキーの辞書としてまとめて保存する。
        private void SaveIlcState()
        {
            var state = new IlcPersistedState();
            foreach (var legId in ilcCorrectionByLeg.Keys)
            {
                var table = ilcCorrectionByLeg[legId];
                var bestTable = ilcBestCorrectionByLeg[legId];
                state.Legs[legId] = new LegIlcState
                {
                    CorrectionX = ExtractComponent(table, true),
                    CorrectionY = ExtractComponent(table, false),
                    BestCorrectionX = ExtractComponent(bestTable, true),
                    BestCorrectionY = ExtractComponent(bestTable, false),
                    BestRmse = ilcBestRmseByLeg[legId],
                    UpdateCount = ilcUpdateCountByLeg[legId],
                    SkippedCount = ilcSkippedCountByLeg[legId],
                    RollbackCount = ilcRollbackCountByLeg[legId],
                };
            }
            try
            {
                File.WriteAllText(IlcStateFilePath, JsonConvert.SerializeObject(state));
            }
            catch (IOException ex)
            {
                Debug.LogWarning($"[StartToBingo] ILC state save failed (will retry next lap): {ex.Message}");
            }
        }

        // 起動時に前回までの学習を復元する。プロジェクト直下に
        // ResetIlcLearningFlagFileNameという名前のファイルが存在する場合は保存済み状態を
        // 無視して(削除して)まっさらから再学習する(RoboconFieldBuilderの
        // DisableNotesExperimentFlagと同じフラグファイル方式)。スキーマが変わった古い
        // 保存ファイル(15leg辞書化より前の固定4テーブル形式)は、デシリアライズに
        // 失敗するかLegsが空のまま読み込まれるため、下記のtry-catch/空チェックで
        // 自動的に「まっさらから学習」にフォールバックする(今回は移行コードを書かない)。
        private void LoadIlcState()
        {
            var resetFlagPath = Path.GetFullPath(
                Path.Combine(Application.dataPath, "..", ResetIlcLearningFlagFileName));
            if (File.Exists(resetFlagPath))
            {
                if (File.Exists(IlcStateFilePath)) File.Delete(IlcStateFilePath);
                Debug.LogWarning(
                    $"[StartToBingo] ILC: {ResetIlcLearningFlagFileName} present; " +
                    "starting learning from scratch (saved state discarded).");
                return;
            }

            if (!File.Exists(IlcStateFilePath))
            {
                Debug.Log($"[StartToBingo] ILC: no saved state at {IlcStateFilePath}; starting from scratch.");
                return;
            }

            try
            {
                var state = JsonConvert.DeserializeObject<IlcPersistedState>(File.ReadAllText(IlcStateFilePath));
                if (state?.Legs == null || state.Legs.Count == 0) return;
                foreach (var (legId, legState) in state.Legs)
                {
                    var table = GetOrCreateIlcTable(legId);
                    ApplyComponents(table, legState.CorrectionX, legState.CorrectionY);
                    ApplyComponents(ilcBestCorrectionByLeg[legId], legState.BestCorrectionX, legState.BestCorrectionY);
                    ilcBestRmseByLeg[legId] = legState.BestRmse;
                    ilcUpdateCountByLeg[legId] = legState.UpdateCount;
                    ilcSkippedCountByLeg[legId] = legState.SkippedCount;
                    ilcRollbackCountByLeg[legId] = legState.RollbackCount;
                }
                Debug.LogWarning(
                    $"[StartToBingo] ILC: loaded saved state from {IlcStateFilePath} " +
                    $"({state.Legs.Count} legs).");
            }
            catch (Exception ex) when (ex is IOException or JsonException)
            {
                Debug.LogWarning(
                    $"[StartToBingo] ILC state load failed, starting from scratch: {ex.Message}");
            }
        }

        // 2026-07-31追加: 衝突検知時の即時中断・同一leg内リトライ。routeもタイムライン
        // (segmentDurations等)も変えず、今のleg(currentLegType/currentPairIndex)の
        // 開始状態(route[0]・startYawRad・速度ゼロ)へロボットを瞬間的に戻し、
        // 経過時間・各種計測をこの試行の
        // 分だけ丸ごと破棄する。Complete()を呼ばないためRESULT行は出ず、完走扱いにもラップ
        // カウントにもしない。現状の検証はUnity単体(ROS非連携)のためテレポートによる
        // 自己位置推定への影響は考慮していない。ROS連携時にこの機構を使う場合は別途要検討。
        private void AbortAndRetryDueToCollision()
        {
            CollisionAbortCount++;
            Debug.LogWarning(
                $"[StartToBingo] Collision detected mid-lap (collisions={CollisionNames}); " +
                $"aborting this attempt immediately and retrying the same leg from its start " +
                $"(collision_abort_count={CollisionAbortCount}).");

            var startUnity = FieldCoordinates.ToUnity(route[0].x, route[0].y, 0.065);
            body.position = startUnity;
            body.rotation = Quaternion.Euler(0f, startYawRad * Mathf.Rad2Deg, 0f);
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;

            // ILCは今トライアル分の誤差蓄積を一切使わない(丸ごと破棄)。
            Array.Clear(ilcTrialErrorSum, 0, IlcBinCount);
            Array.Clear(ilcTrialErrorCount, 0, IlcBinCount);

            // Phase 0計装(参照軌道追従・デッドレコニング自己検算・実加速度)もこの試行の
            // 分だけリセットする。破棄した試行の値が次の(成功する)試行の統計へ混入しない
            // ようにするため。
            trackingSquaredErrorSum = 0.0;
            maxTrackingError = 0f;
            trackingSamples = 0;
            squaredErrorSum = 0f;
            maxOdometryError = 0f;
            samples = 0;
            havePreviousBodyVelocity = false;
            maxAccelerationMagnitude = 0f;
            estimatedPosition = route[0];

            startTime = Time.time + 0.25f;
            routeDoneSinceTime = -1f;
            collisionRecorder.ClearCollisions();
        }

        // 瞬間的にロボットをstart地点(notesFacingYawRad向き)へ戻す。衝突リトライの
        // テレポートと同じ発想だが、こちらは衝突ではなく通常のleg切り替え(5往復ごとに
        // start→pickup_iを挟むタイミング)で使う。
        private void TeleportToStart()
        {
            var startUnity = FieldCoordinates.ToUnity(startPoint.x, startPoint.y, 0.065);
            body.position = startUnity;
            body.rotation = Quaternion.Euler(0f, notesFacingYawRad * Mathf.Rad2Deg, 0f);
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            estimatedPosition = startPoint;
        }

        // 2026-07-31再設計: 今しがた終わったlegの種類に応じて次のlegを決め、
        // waypoint・タイムラインを組み立て直す。基本ループは「bingo↔pickup_i」を
        // i=0→1→2→3→4→0→...と往復させ、5往復(=pickup_iへの到着5回)ごとにstartへ
        // テレポートしてstart→pickup_iを1回挟む(実戦では稀だがリスタート用に全組を
        // 学習させるため)。pairCounterはどちらの遷移でも共有・単調に進めるため、
        // セッションを長く走らせれば5組全てがstart→pickup_iでも学習される。
        private void AdvanceToNextLeg()
        {
            lapCount++;
            // 前legの終端ヨーを、次legの区間0が始まる時点の向きとして引き継ぐ
            // (ヨーがlegの継ぎ目で不連続に飛ばないようにするため)。
            startYawRad = segmentYawEnd[^1];

            switch (currentLegType)
            {
                case LegType.StartToPickup:
                    // start→pickup_iが終わったら、同じ組でpickup_i→bingoへ。
                    currentLegType = LegType.PickupToBingo;
                    break;
                case LegType.PickupToBingo:
                    legsSincePickupArrivalReset++;
                    pairCounter = (pairCounter + 1) % PairCount;
                    if (legsSincePickupArrivalReset >= PairCount)
                    {
                        legsSincePickupArrivalReset = 0;
                        TeleportToStart();
                        // TeleportToStart()は実姿勢をnotesFacingYawRadへ強制的に戻すため、
                        // 次legの基準ヨーも(関数冒頭で設定したsegmentYawEnd[^1]=
                        // bingoFacingYawRadではなく)それに合わせて上書きする。
                        startYawRad = notesFacingYawRad;
                        currentLegType = LegType.StartToPickup;
                    }
                    else
                    {
                        currentLegType = LegType.BingoToPickup;
                    }
                    break;
                case LegType.BingoToPickup:
                    // bingo→pickup_iが終わったら、同じ組でpickup_i→bingoへ。
                    currentLegType = LegType.PickupToBingo;
                    break;
            }
            currentPairIndex = pairCounter;

            BuildLegRoute();
            startTime = Time.time + 0.25f;
            routeDoneSinceTime = -1f;
        }

        private void BuildRobot()
        {
            var previous = GameObject.Find("Omni3ValidationRobot");
            if (previous != null) Destroy(previous);

            var robotObject = new GameObject("Omni3ValidationRobot");
            robot = robotObject.transform;
            robot.SetParent(transform, false);
            robot.position = FieldCoordinates.ToUnity(route[0].x, route[0].y, 0.065);
            // 2026-07-31追加: スポーン姿勢を単位回転(0度)のまま放置すると、区間0
            // (start→pickup)の開始でnotesFacingYawRad(90度)まで大きく旋回しながら
            // 並進することになり、回頭中に(回収辺ではなく)頂点がノーツ側へ振れて
            // 突っ込む向きになる恐れがあるとユーザーから指摘があった。スポーン時点で
            // 既にnotesFacingYawRadを向かせておけば区間0は回頭せず並進のみになる。
            robot.rotation = Quaternion.Euler(0f, notesFacingYawRad * Mathf.Rad2Deg, 0f);

            var chassisMesh = CreateRoundedTrianglePrism(
                (float)robotDefinition.Chassis.Side,
                (float)robotDefinition.Chassis.Height,
                (float)robotDefinition.Chassis.CornerRadius, 12);
            var collider = robotObject.AddComponent<MeshCollider>();
            collider.sharedMesh = chassisMesh;
            collider.convex = true;
            collider.sharedMaterial = new PhysicsMaterial("OmniRollerLowFriction")
            {
                staticFriction = 0f,
                dynamicFriction = 0f,
                frictionCombine = PhysicsMaterialCombine.Minimum,
                bounciness = 0f,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };

            body = robotObject.AddComponent<Rigidbody>();
            body.mass = 20f;
            body.linearDamping = 1f;
            body.angularDamping = 2f;
            body.sleepThreshold = 0f;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            // EnableYawRotationの説明はフィールド宣言のコメント参照。ヨー軸のみ
            // 回転を許可する場合もロール/ピッチは倒立防止のため常に凍結する。
            body.constraints = EnableYawRotation
                ? (RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ)
                : RigidbodyConstraints.FreezeRotation;
            collisionRecorder = robotObject.AddComponent<CollisionRecorder>();
            IgnoreNoteContactForLocalizationBaseline(collider);
            robotObject.AddComponent<UnityRosSensorPublisher>();

            var chassisMaterial = MakeMaterial("ValidationRobotWhite", new Color(0.95f, 0.95f, 0.98f));
            var wheelMaterial = MakeMaterial("OmniWheelOrange", new Color(1f, 0.35f, 0.05f));
            var lidarMaterial = MakeMaterial("LidarCyan", new Color(0f, 0.85f, 1f));

            var chassis = new GameObject("RoundedTriangleChassis");
            chassis.transform.SetParent(robot, false);
            chassis.AddComponent<MeshFilter>().sharedMesh = chassisMesh;
            chassis.AddComponent<MeshRenderer>().sharedMaterial = chassisMaterial;

            wheelRadius = (float)robotDefinition.OmniWheels.SimulationDiameter / 2f;
            wheels = new Transform[robotDefinition.OmniWheels.Count];
            for (var i = 0; i < wheels.Length; ++i)
            {
                var angle = 180f + i * 120f;
                var radial = new Vector3(Mathf.Cos(angle * Mathf.Deg2Rad), 0f,
                    Mathf.Sin(angle * Mathf.Deg2Rad));
                var wheel = new GameObject($"OmniWheel_{i}");
                wheel.transform.SetParent(robot, false);
                wheel.transform.localPosition = radial * 0.235f + Vector3.down * 0.045f;
                wheel.transform.localRotation = Quaternion.FromToRotation(Vector3.up, radial);
                wheel.AddComponent<MeshFilter>().sharedMesh = CreateCylinderMesh(
                    wheelRadius, (float)robotDefinition.OmniWheels.Width, 64);
                wheel.AddComponent<MeshRenderer>().sharedMaterial = wheelMaterial;
                wheels[i] = wheel.transform;
            }

            var lidar = new GameObject("Lidar");
            lidar.transform.SetParent(robot, false);
            // The chassis prism is centred on the robot origin, so that origin rests one
            // half-height (60 mm) above the floor and this +80 mm local offset puts the scan
            // plane at 140 mm — i.e. 20 mm above the 120 mm chassis top, a natural mount.
            //
            // IMPORTANT: the scan plane must stay BELOW the field wall top or the LiDAR sees
            // no wall at all.  The competition wall is only 100 mm tall, which is below this
            // mount and made the perimeter, centre wall and slalom baffles physically
            // invisible — leaving only the 0.9 m bingo posts as returns.  That, not ICP
            // tuning, caused the metre-scale drift (no wall correction -> IMU dead reckoning).
            // While the robot design is still in flux the field JSON raises walls.height above
            // this plane; when the real mount height is fixed, restore the rule height (0.1 m)
            // and lower this mount so the beam sits mid-wall (see docs stage-4 notes).
            lidar.transform.localPosition = Vector3.up * 0.08f;
            lidar.AddComponent<MeshFilter>().sharedMesh = CreateCylinderMesh(0.045f, 0.07f, 64);
            lidar.AddComponent<MeshRenderer>().sharedMaterial = lidarMaterial;
        }

        private static void IgnoreNoteContactForLocalizationBaseline(Collider chassisCollider)
        {
            // Cargo intake/contact mechanics are deliberately outside this localisation
            // baseline.  The route's pickup centre is intentionally within the chassis
            // footprint of the two target notes; leaving their rigid-body contacts enabled
            // therefore guarantees an artificial impact before CaptureNotes can run.  Keep the
            // note colliders enabled so the LiDAR still observes the real unmapped objects,
            // but remove only robot-note contact forces until an intake model is supplied.
            var ignored = 0;
            foreach (var noteCollider in FindObjectsByType<Collider>(FindObjectsSortMode.None))
            {
                if (!noteCollider.name.StartsWith("BlueNote_") &&
                    !noteCollider.name.StartsWith("OrangeNote_")) continue;
                Physics.IgnoreCollision(chassisCollider, noteCollider);
                ignored++;
            }
            Debug.Log($"[StartToBingo] localisation baseline: ignored chassis contact with {ignored} note colliders; LiDAR remains active.");
        }

        private void CaptureNotes()
        {
            // Sensor/localization baseline: stop at the pickup pose, but leave the notes on
            // the field. Cargo handling will be re-enabled after the unobstructed baseline.
            Debug.Log($"[StartToBingo] Pickup stop reached ({currentLegId}); cargo capture is disabled for this baseline.");
        }

        // 2026-07-31追加: bingo共通到着点での配置(投入)動作のplaceholder。CaptureNotes()と
        // 同じ位置5mm・速度10mm/s以下の静止基準で呼ばれる。実機の投入機構が決まるまでの
        // 暫定実装(ビンゴ内の特定スロット・左右アームへの位置合わせは今回のスコープ外、
        // docs/experiment_history/27番参照)。
        private void DeliverNotes()
        {
            Debug.Log($"[StartToBingo] Bingo stop reached ({currentLegId}); cargo delivery is disabled for this baseline.");
        }

        private void RotateWheelVisuals(float dt)
        {
            for (var i = 0; i < wheels.Length; ++i)
            {
                var angle = 180f + i * 120f;
                var tangent = new Vector3(-Mathf.Sin(angle * Mathf.Deg2Rad), 0f,
                    Mathf.Cos(angle * Mathf.Deg2Rad));
                var omega = Vector3.Dot(body.linearVelocity, tangent) / wheelRadius;
                wheels[i].Rotate(Vector3.up, omega * Mathf.Rad2Deg * dt, Space.Self);
            }
        }

        private void EvaluateRoute(
            float time, out Vector2 position, out Vector2 velocity, out float yawRad,
            out float yawRateRad, out bool done)
        {
            // 2026-07-31再設計: 旧実装はここでpickup到着後の待機(mission.PickupDwellSeconds)
            // をタイムライン内にスケジュールしていたが、leg構造化後はpickup/bingo到着後の
            // 静止待機(pickup_dwell_s)がFixedUpdateの到着判定(dwellSatisfied)側に統一
            // されたため、この経路内スケジュールは不要になった(該当機構は
            // pickupDwellSecondsCommon参照)。
            var elapsed = 0f;
            for (var i = 0; i < segmentDurations.Count; ++i)
            {
                var duration = segmentDurations[i];
                if (time <= elapsed + duration)
                {
                    QuinticBlend(
                        (time - elapsed) / duration, segmentEntryBlend[i], segmentExitBlend[i],
                        out var blend, out var derivative);
                    var delta = route[i + 1] - route[i];
                    position = route[i] + delta * blend;
                    velocity = delta * (derivative / duration);
                    // ヨーも位置と同じタイムラインでなめらかに変化させる
                    // (最短角度でラップアラウンドを扱う)。区間0の開始向きは
                    // startYawRad(スポーン姿勢、またはラップ継ぎ目の引き継ぎ値)。
                    var yawStart = i == 0 ? startYawRad : segmentYawEnd[i - 1];
                    var yawDelta = Mathf.Repeat(
                        segmentYawEnd[i] - yawStart + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
                    yawRad = yawStart + yawDelta * blend;
                    // 位置のvelocityと同じ規約(delta * derivative/duration)でヨーレートも
                    // 解析的に求める。MoveRotationではなくangularVelocityへ直接
                    // 与えるための値(FixedUpdate参照)。
                    yawRateRad = yawDelta * (derivative / duration);
                    done = false;
                    return;
                }
                elapsed += duration;

                var spinDuration = segmentSpinDurations[i];
                if (spinDuration > 0f)
                {
                    if (time <= elapsed + spinDuration)
                    {
                        MinimumJerk((time - elapsed) / spinDuration, out var spinBlend, out var spinDerivative);
                        position = route[i + 1];
                        velocity = Vector2.zero;
                        // segmentYawEnd[i]から余分に1回転(SpinRadians)して同じ向きへ戻る。
                        // 次区間のyawStart(=segmentYawEnd[i])と連続になるよう、
                        // 角度そのものはラップさせず単調に足すだけでよい。
                        yawRad = segmentYawEnd[i] + SpinRadians * spinBlend;
                        yawRateRad = SpinRadians * (spinDerivative / spinDuration);
                        done = false;
                        return;
                    }
                    elapsed += spinDuration;
                }
            }
            yawRad = segmentYawEnd.Count > 0 ? segmentYawEnd[^1] : startYawRad;
            yawRateRad = 0f;
            position = route[^1];
            velocity = Vector2.zero;
            done = true;
        }

        private static Mesh CreateRoundedTrianglePrism(float side, float height, float radius, int arcSegments)
        {
            var inradius = side * Mathf.Sqrt(3f) / 6f;
            var circumradius = side / Mathf.Sqrt(3f);
            var corners = new[]
            {
                new Vector2(inradius, -side / 2f),
                new Vector2(inradius, side / 2f),
                new Vector2(-circumradius, 0f)
            };
            var outline = new List<Vector2>();
            var tangentDistance = radius / Mathf.Tan(Mathf.PI / 6f);
            var centreDistance = radius / Mathf.Sin(Mathf.PI / 6f);
            for (var i = 0; i < corners.Length; ++i)
            {
                var corner = corners[i];
                var toPrevious = (corners[(i + 2) % 3] - corner).normalized;
                var toNext = (corners[(i + 1) % 3] - corner).normalized;
                var centre = corner + (toPrevious + toNext).normalized * centreDistance;
                var start = corner + toPrevious * tangentDistance;
                var end = corner + toNext * tangentDistance;
                var startAngle = Mathf.Atan2(start.y - centre.y, start.x - centre.x);
                var endAngle = Mathf.Atan2(end.y - centre.y, end.x - centre.x);
                var sweep = Mathf.Repeat(endAngle - startAngle, 2f * Mathf.PI);
                for (var j = 0; j <= arcSegments; ++j)
                {
                    var angle = startAngle + sweep * j / arcSegments;
                    outline.Add(centre + radius * new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)));
                }
            }

            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            var bottomCentre = vertices.Count;
            vertices.Add(new Vector3(0f, -height / 2f, 0f));
            var topCentre = vertices.Count;
            vertices.Add(new Vector3(0f, height / 2f, 0f));
            for (var i = 0; i < outline.Count; ++i)
                vertices.Add(new Vector3(outline[i].x, -height / 2f, outline[i].y));
            var topOffset = vertices.Count;
            for (var i = 0; i < outline.Count; ++i)
                vertices.Add(new Vector3(outline[i].x, height / 2f, outline[i].y));

            for (var i = 0; i < outline.Count; ++i)
            {
                var next = (i + 1) % outline.Count;
                var b0 = 2 + i;
                var b1 = 2 + next;
                var t0 = topOffset + i;
                var t1 = topOffset + next;
                // 2026-08-02: bottom/top cap fans were wound backwards (RecalculateNormals
                // produced bottomCentre normal=(0,1,0) and topCentre normal=(0,-1,0) -- both
                // pointing INTO the solid instead of outward), so both caps faced away from an
                // outside viewer and got backface-culled, making the chassis look hollow/
                // see-through from above or below. Swapping each fan's last two indices flips
                // the winding so the caps face outward like the side walls already did.
                // 2026-08-02 continued: the side wall quad was wound backwards too (measured
                // dot(expectedOutwardXZ, actualNormalXZ) around -0.7 to -0.99 across the ring,
                // i.e. facing inward), causing the same hollow/see-through look on the chassis
                // sides the user reported after the cap fix. Reversed the same way.
                triangles.AddRange(new[] { bottomCentre, b0, b1, topCentre, t1, t0,
                    b0, t1, b1, b0, t0, t1 });
            }
            var mesh = new Mesh { name = "RoundedEquilateralTriangle500mm" };
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Mesh CreateCylinderMesh(float radius, float width, int sides)
        {
            var vertices = new List<Vector3>();
            var triangles = new List<int>();
            for (var i = 0; i < sides; ++i)
            {
                var angle = 2f * Mathf.PI * i / sides;
                var x = Mathf.Cos(angle) * radius;
                var z = Mathf.Sin(angle) * radius;
                vertices.Add(new Vector3(x, -width / 2f, z));
                vertices.Add(new Vector3(x, width / 2f, z));
            }
            for (var i = 0; i < sides; ++i)
            {
                var next = (i + 1) % sides;
                var a = 2 * i;
                var b = 2 * next;
                // 2026-08-02: same backwards winding as the chassis (measured normal at v[0]
                // pointing (-1,0,-0.02) against an expected outward (1,0,0)) -- used by both
                // the wheels and the lidar mount, so both looked hollow/see-through too.
                triangles.AddRange(new[] { a, b + 1, b, a, a + 1, b + 1 });
            }
            // 2026-08-02: this cylinder had no end caps at all (side wall only), so the wheels'
            // and lidar's flat faces were literally open holes, not just backwards-facing
            // triangles -- looking along the axis showed straight through. Add real caps,
            // using the same winding convention already verified correct on the chassis caps
            // (bottom fan = {centre, current, next} -> outward -Y; top fan = {centre, next,
            // current} -> outward +Y).
            var bottomCentre = vertices.Count;
            vertices.Add(new Vector3(0f, -width / 2f, 0f));
            var topCentre = vertices.Count;
            vertices.Add(new Vector3(0f, width / 2f, 0f));
            for (var i = 0; i < sides; ++i)
            {
                var next = (i + 1) % sides;
                triangles.AddRange(new[] { bottomCentre, 2 * i, 2 * next,
                    topCentre, 2 * next + 1, 2 * i + 1 });
            }
            var mesh = new Mesh { name = $"Cylinder{sides}" };
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Vector2 ToVector(IReadOnlyList<double> point) =>
            new((float)point[0], (float)point[1]);

        private static void MinimumJerk(float u, out float position, out float derivative)
        {
            u = Mathf.Clamp01(u);
            position = 10f * u * u * u - 15f * u * u * u * u + 6f * u * u * u * u * u;
            derivative = 30f * u * u - 60f * u * u * u + 30f * u * u * u * u;
        }

        // 2026-07-31追加: MinimumJerkの一般化。境界条件p(0)=0,p(1)=1,p'(0)=v0,p'(1)=v1,
        // p''(0)=p''(1)=0(躍度ゼロ端点は維持)を満たす5次多項式。v0=v1=0のときMinimumJerkと
        // 完全に一致する(下記係数式にv0=v1=0を代入すると10u³-15u⁴+6u⁵になることを確認済み)。
        // required_zero_speed_pointsに該当しないwaypointを完全停止せず通過するために使う
        // (BuildTimeline/EvaluateRoute参照)。
        private static void QuinticBlend(float u, float v0, float v1, out float position, out float derivative)
        {
            u = Mathf.Clamp01(u);
            var c3 = 10f - 6f * v0 - 4f * v1;
            var c4 = 8f * v0 + 7f * v1 - 15f;
            var c5 = 6f - 3f * v0 - 3f * v1;
            var u2 = u * u;
            var u3 = u2 * u;
            var u4 = u3 * u;
            var u5 = u4 * u;
            position = v0 * u + c3 * u3 + c4 * u4 + c5 * u5;
            derivative = v0 + 3f * c3 * u2 + 4f * c4 * u3 + 5f * c5 * u4;
        }

        private static Vector2 UnityToField(Vector3 value) => new(value.z, -value.x);

        private static Material MakeMaterial(string name, Color colour)
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            var material = new Material(shader) { name = name, color = colour, enableInstancing = true };
            if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", colour);
            // The URP Lit shader's default Smoothness(0.5)+EnvironmentReflections(on) made the
            // near-white chassis pick up a strong sky reflection/Fresnel sheen under the default
            // skybox, making an opaque (Surface=Opaque, alpha=1) mesh look glassy/translucent.
            // Flatten it to a matte finish so solid parts read as solid.
            if (material.HasProperty("_Smoothness")) material.SetFloat("_Smoothness", 0.12f);
            if (material.HasProperty("_EnvironmentReflections")) material.SetFloat("_EnvironmentReflections", 0f);
            return material;
        }
    }

    public sealed class CollisionRecorder : MonoBehaviour
    {
        private readonly HashSet<string> collisionNames = new();
        public string CollisionNames => string.Join(",", collisionNames);
        // UnityRosSensorPublisherはCollisionRecorderと同じGameObjectへBuildRobot内で
        // 後から追加されるが、Start()は同一フレームの全AddComponent完了後に呼ばれる
        // ため、ここで取得すれば必ず見つかる(2026-07-31、衝突をbagへ残すため追加)。
        private UnityRosSensorPublisher publisher;

        private void Start()
        {
            publisher = GetComponent<UnityRosSensorPublisher>();
        }

        private void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.name == "Floor") return;
            collisionNames.Add(collision.gameObject.name);
            Debug.LogWarning($"[StartToBingo] Collision with {collision.gameObject.name}");
            publisher?.PublishCollision(collision.gameObject.name, collision.relativeVelocity.magnitude);
        }

        // ラップ単位の判定(ILCの学習禁止ゲート等)に使うため、ラップ完了ごとに
        // 呼び出し側からクリアする。RESULT行の`collisions=`フィールドは呼び出し前の
        // 値をそのまま使うので、この呼び出しは記録・ILC判定の後に行うこと。
        public void ClearCollisions() => collisionNames.Clear();
    }
}
