using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Robocon2026.Simulation
{
    /// <summary>
    /// pathplannning ⇄ Unity 実機物理接続検証(2026-08-01計画)。
    /// pathplannning_nodeが配信する/cmd_vel_body(車輪運動学適用前の機体座標系速度
    /// 指令: linear.x/y=機体座標系並進速度、angular.z=yaw_rate)を購読し、Rigidbodyへ
    /// 直接適用する薄いブリッジ。オムニ車輪運動学は一切経由しない — 動力学
    /// シミュレータ(滑り・モータ遅れ・摩擦)がROS2側にもUnity側にも存在しないため、
    /// B-spline生成・FF+PD追従・実行状態機械を本物の物理(質量・摩擦・衝突)+本物の
    /// 自己位置推定(lio_localization)で検証することが目的。車輪運動学自体は
    /// pathplannningのgtestで既に検証済み。
    ///
    /// 接続の確立(RosIPAddress/Port/Connect)は同じGameObjectにアタッチされる
    /// UnityRosSensorPublisher側が担う(StartToBingoValidation.BuildRobot()参照)。
    /// このスクリプトはSubscribeのみ行う。
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class PathplanningCmdVelBridge : MonoBehaviour
    {
        private const string CmdVelBodyTopic = "/cmd_vel_body";

        // 2026-08-02修正: 既定をtrue(反転する)へ変更。
        // 旧コメントは「angular.z=+0.3を送るとRigidbody.angularVelocity.y=+0.30になる」ことを
        // 確認して「反転不要」と結論していたが、これはブリッジが数値をそのまま
        // Rigidbodyへ渡すことを確認しただけで、**その結果フィールド座標系のyawが
        // 正しい向きへ動くか**は検証できていなかった([[feedback_verify_mesh_local_axis_empirically]]
        // と同種の誤り)。
        // FieldCoordinates.YawFromForward = atan2(-forward.x, forward.z)。
        // eulerY=θ のとき forward=(sinθ,0,cosθ) なので field_yaw = atan2(-sinθ,cosθ) = -θ、
        // すなわち **field_yaw = -eulerY**。よって angularVelocity.y に +ω を入れると
        // field yaw_rate は -ω になり、FFPDのyawループが正帰還になっていた。
        // 実測(2026-08-02): pickup1_to_bingo走行中、yaw誤差が負(=負のyaw_rate指令)なのに
        // field yawが 1.14°→25.07° と単調増加し、Unity実測 eulerY=-23.3° に対し
        // CSVのfield yaw=+23.2°(符号が逆)であることを確認。これがこのセッションで
        // 「yaw発散」「180度反転」として何度も観測された事象の真因。
        [SerializeField] private bool invertYawRateSign = true;

        private Rigidbody body;
        private double bodyVx;
        private double bodyVy;
        private double yawRate;
        private bool hasCommand;

        private void Start()
        {
            body = GetComponent<Rigidbody>();
            var ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>(CmdVelBodyTopic, OnCmdVelBodyReceived);
        }

        private void OnCmdVelBodyReceived(TwistMsg msg)
        {
            bodyVx = msg.linear.x;
            bodyVy = msg.linear.y;
            yawRate = msg.angular.z;
            hasCommand = true;
        }

        private void FixedUpdate()
        {
            if (body == null || !hasCommand) return;

            // 機体座標系(フィールド座標系)→世界座標系(フィールド座標系)への回転。
            // UnityRosSensorPublisher.OnOdomFastReceivedで実績のある式をそのまま流用する
            // (地面真値yawを使う。推定値ではない — ここは実際の駆動ループなので当然)。
            var yaw = FieldCoordinates.YawFromForward(transform.forward);
            var cosYaw = System.Math.Cos(yaw);
            var sinYaw = System.Math.Sin(yaw);
            var vxWorldField = cosYaw * bodyVx - sinYaw * bodyVy;
            var vyWorldField = sinYaw * bodyVx + cosYaw * bodyVy;

            // フィールド座標系の自由ベクトル→Unity座標系。ToUnityは並進オフセットを
            // 持たない線形写像(x,y,z)->(-y,z,x)なので、位置だけでなく速度ベクトルにも
            // そのまま使える(SizeToUnityは符号を反転しない「大きさ」専用なのでここでは
            // 使わない)。
            body.linearVelocity = FieldCoordinates.ToUnity(vxWorldField, vyWorldField, 0.0);

            var unityYawRate = invertYawRateSign ? -yawRate : yawRate;
            body.angularVelocity = new Vector3(0f, (float)unityYawRate, 0f);
        }
    }
}
