using UnityEngine;

namespace Robocon2026.Simulation
{
    public sealed class SimulationBootstrap : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void EnsureSimulationExists()
        {
            if (FindAnyObjectByType<SimulationBootstrap>() != null) return;
            var root = new GameObject("Robocon2026Simulation");
            root.AddComponent<SimulationBootstrap>();
        }

        private void Awake()
        {
            Debug.Log(
                $"[Robocon2026 GPU] device='{SystemInfo.graphicsDeviceName}', " +
                $"api={SystemInfo.graphicsDeviceType}, compute={SystemInfo.supportsComputeShaders}, " +
                $"instancing={SystemInfo.supportsInstancing}");
            gameObject.AddComponent<RoboconFieldBuilder>().Build();
            gameObject.AddComponent<StartToBingoValidation>();
        }
    }
}
