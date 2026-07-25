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
    }
}
