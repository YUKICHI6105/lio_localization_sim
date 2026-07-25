using System;
using System.Collections;
using System.Globalization;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace Robocon2026.Tests
{
    /// <summary>
    /// Headless driver for the localisation simulation.
    ///
    /// SimulationBootstrap builds the whole field, robot and sensor publishers from a
    /// [RuntimeInitializeOnLoadMethod(AfterSceneLoad)] hook, so entering play mode is all that
    /// is required — no scene asset and no human pressing Play.  Running this as a PlayMode
    /// test lets the whole stage-4 loop be driven from the command line:
    ///
    ///   Unity.exe -batchmode -runTests -testPlatform PlayMode -roboconSeconds 45 ...
    ///
    /// while the ROS side records the evaluation.  The test itself only holds play mode open
    /// for the requested span; the pass/fail that matters is the ROS evaluator report, so this
    /// deliberately does not assert on simulation behaviour.
    /// </summary>
    public sealed class HeadlessSimulationRun
    {
        private const string SecondsArgument = "-roboconSeconds";
        private const float DefaultSeconds = 45f;

        [UnityTest]
        public IEnumerator HoldsPlayModeForRequestedDuration()
        {
            // The simulation legitimately logs errors (e.g. a ROS endpoint that is not up yet),
            // and the Unity test runner fails a test on any logged error by default.  The real
            // verdict comes from the ROS evaluator, so do not let logs decide it here.
            LogAssert.ignoreFailingMessages = true;

            var seconds = ParseSeconds();
            Debug.Log($"[HeadlessRun] holding play mode for {seconds:F1}s of game time.");

            // Wait on game time so the span matches the /clock Unity publishes to ROS; batch
            // mode does not run at wall-clock rate and the ROS evaluator counts in sim time.
            var deadline = Time.time + seconds;
            while (Time.time < deadline)
            {
                yield return null;
            }

            Debug.Log($"[HeadlessRun] finished after {Time.time:F1}s of game time.");
            Assert.Pass("Headless simulation span completed; see the ROS evaluator report.");
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
                Debug.LogWarning($"[HeadlessRun] could not parse '{args[i + 1]}' after " +
                                 $"{SecondsArgument}; using {DefaultSeconds}s.");
                break;
            }
            return DefaultSeconds;
        }
    }
}
