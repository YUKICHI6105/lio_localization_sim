using System;
using UnityEngine;

namespace Robocon2026.Simulation
{
    public static class FieldCoordinates
    {
        // REP-103 field coordinates -> Unity left-handed coordinates.
        // (Unity X, Unity Y, Unity Z) = (-field Y, field Z, field X)
        public static Vector3 ToUnity(double fieldX, double fieldY, double fieldZ)
            => new((float)-fieldY, (float)fieldZ, (float)fieldX);

        public static Vector3 SizeToUnity(double fieldX, double fieldY, double fieldZ)
            => new((float)fieldY, (float)fieldZ, (float)fieldX);

        // 2026-08-01: UnityRosSensorPublisher.FieldYaw()から抽出した共有実装。
        // transform.forwardからフィールド座標系のyaw(rad)を求める式。
        // PathplanningCmdVelBridgeでも同じ式を再利用するため、同じ式を2箇所に
        // 手で書き写すこと(=このセッションでメッシュのlocal軸を手計算のみで誤った
        // 前例と同じリスク)を避けてここへ一本化する。
        public static double YawFromForward(Vector3 forward)
            => Math.Atan2(-forward.x, forward.z);
    }
}
