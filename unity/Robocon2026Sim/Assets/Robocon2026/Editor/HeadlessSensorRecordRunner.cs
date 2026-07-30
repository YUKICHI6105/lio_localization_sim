using System;
using System.Globalization;
using UnityEditor;
using UnityEngine;

namespace Robocon2026.Simulation.Editor
{
    /// <summary>
    /// Batch-mode entry point that runs the simulation long enough to create an
    /// authoritative Unity sensor CSV recording.  This deliberately does not use
    /// the Unity Test Framework: on Unity 6000.5 its -runTests CLI option can exit
    /// after import without dispatching PlayMode tests.
    /// </summary>
    public static class HeadlessSensorRecordRunner
    {
        private const string SecondsArgument = "-roboconSeconds";
        private const float DefaultSeconds = 60f;
        private static float durationSeconds;
        private static bool started;

        public static void Start()
        {
            durationSeconds = ParseSeconds();
            Debug.Log($"[HeadlessRecord] starting {durationSeconds:F1}s PlayMode sensor recording.");
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
            EditorApplication.update += Poll;
            EditorApplication.isPlaying = true;
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state != PlayModeStateChange.EnteredPlayMode) return;
            started = true;
            Debug.Log($"[HeadlessRecord] PlayMode entered at Time.time={Time.time:F3}.");
        }

        private static void Poll()
        {
            if (!started || !EditorApplication.isPlaying || Time.time < durationSeconds) return;
            Debug.Log($"[HeadlessRecord] duration reached at Time.time={Time.time:F3}; stopping PlayMode.");
            EditorApplication.isPlaying = false;
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
            EditorApplication.update -= Poll;
            EditorApplication.delayCall += () => EditorApplication.Exit(0);
        }

        private static float ParseSeconds()
        {
            var args = Environment.GetCommandLineArgs();
            for (var i = 0; i < args.Length - 1; ++i)
            {
                if (!string.Equals(args[i], SecondsArgument, StringComparison.Ordinal)) continue;
                if (float.TryParse(args[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture,
                        out var parsed) && parsed > 0f)
                {
                    return parsed;
                }
                break;
            }
            return DefaultSeconds;
        }
    }
}
