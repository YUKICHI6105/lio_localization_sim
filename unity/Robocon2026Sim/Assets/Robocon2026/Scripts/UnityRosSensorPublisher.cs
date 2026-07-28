using System;
using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Rosgraph;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Robocon2026.Simulation
{
    /// <summary>
    /// Publishes measurements derived exclusively from Unity physics and colliders.
    /// No analytical trajectory or ROS-side sensor simulator is involved.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class UnityRosSensorPublisher : MonoBehaviour
    {
        private const string ClockTopic = "/clock";
        private const string ImuTopic = "/imu/data";
        private const string ScanTopic = "/scan";
        private const string GroundTruthTopic = "/ground_truth_pose";
        private const string OdomFastTopic = "/odom_fast";

        // Closed-loop control feedback: the localisation stack's own estimate (node ①'s
        // /odom_fast), consumed by StartToBingoValidation instead of the ground-truth
        // transform so the mission actually flies on what the estimator produces. Ground
        // truth remains available (via /ground_truth_pose and body.position) purely for
        // scoring, never for control, once this is populated.
        public bool HasOdomEstimate { get; private set; }
        public Vector2 EstimatedFieldPosition { get; private set; }
        public double EstimatedYaw { get; private set; }
        public Vector2 EstimatedFieldVelocityWorld { get; private set; }
        // Send-side counters. The ROS-side rate looked like ~40% of what this publishes, but a
        // Python subscriber can drop on its own, so the loss has to be located by comparing an
        // authoritative send count against the receive count rather than inferred from a rate.
        public static long ImuPublishCount;
        public static long ScanPublishCount;
        private const int LidarSamples = 1081;
        private const float LidarFovRadians = 270f * Mathf.Deg2Rad;
        // A 270 deg sensor has a 90 deg blind sector; where it points is a mounting choice.
        // Aim it at the notes side (robot's -y) so the unmapped balls, which sit 0.25-0.39 m
        // away and would otherwise be mis-associated to the centre wall 0.10 m behind them,
        // fall outside the scan entirely. Scans stay self-describing via angle_min, so the
        // ROS side needs no matching change.
        private const float LidarBlindCentreRadians = -90f * Mathf.Deg2Rad;
        private const float LidarRangeMin = 0.05f;
        private const float LidarRangeMax = 30f;
        // /clock drives ROS simulation time, not the physical IMU. Publishing it at every
        // 1 kHz physics step made every ROS node process 1,000 redundant clock updates/s and
        // competed with the actual 1 kHz IMU over the Unity TCP bridge. A 250 Hz clock still
        // resolves the 4 ms timing needed by watchdogs/evaluation while preserving every
        // 1 ms IMU sample and stamp.
        private const float ClockPeriod = 0.004f;
        private const float LidarPeriod = 0.025f;
        private const float GroundTruthPeriod = 0.010f;

        private readonly RaycastHit[] rayHits = new RaycastHit[32];
        private readonly HashSet<Collider> robotSelfColliders = new();
        private readonly System.Random random = new(20260723);
        private ROSConnection ros = null!;
        private Rigidbody body = null!;
        private Vector3 previousWorldVelocity;
        private double nextClockTime;
        private double nextScanTime;
        private double nextGroundTruthTime;
        private double previousImuSampleTime;
        private bool initializedVelocity;

        // Real IMU bias is not a fixed offset: it drifts slowly over the run (temperature,
        // mechanical settling, 1/f noise). A constant offset only exercises the estimator's
        // *initial* bias guess, never its ability to track drift via
        // bias_acc/gyro_random_walk_sigma -- so a constant-bias simulation cannot validate
        // that mechanism against anything resembling real hardware. These fields hold the
        // evolving bias state, seeded at Start() from the same constants PublishImu used to
        // add directly, and nudged by a small random walk each sample (see PublishImu).
        private double accelBiasX;
        private double accelBiasY;
        private double gyroBiasZ;

        // Epoch for every timestamp this publisher sends (/clock and all message headers),
        // recorded once at Start(). Time.timeAsDouble is normally 0 at the start of a fresh
        // Play session, but a Play/Stop transition issued before the previous session had
        // actually finished exiting has been observed to leave it at the *previous* session's
        // elapsed time (thousands of seconds) instead of resetting. Downstream, ROS's
        // evaluator times out mission duration against this /clock, so a large unexplained
        // jump made it conclude the run was already over before any data arrived. Subtracting
        // this epoch makes every published timestamp start at 0 for this session regardless
        // of what Time.timeAsDouble's absolute value happens to be.
        private double sessionStartTime;

        private void Start()
        {
            sessionStartTime = Time.timeAsDouble;
            ImuPublishCount = 0;
            ScanPublishCount = 0;
            body = GetComponent<Rigidbody>();
            foreach (var selfCollider in GetComponentsInChildren<Collider>())
                robotSelfColliders.Add(selfCollider);
            ros = ROSConnection.GetOrCreateInstance();
            // ROSConnection is a persistent singleton in the Editor.  After a Play/Stop
            // cycle its previous connection thread has been cancelled by OnDestroy(), but the
            // singleton itself survives; merely setting ConnectOnStart again does not restart
            // that thread.  Own the connection explicitly for each publisher session so the
            // next Play always registers a fresh socket with the waiting ROS endpoint.
            ros.Disconnect();
            var configuredRosIp = Environment.GetEnvironmentVariable("ROBOCON_ROS_IP");
            ros.RosIPAddress = string.IsNullOrWhiteSpace(configuredRosIp)
                ? "127.0.0.1"
                : configuredRosIp;
            ros.RosPort = 10000;
            ros.ConnectOnStart = false;
            ros.ShowHud = true;

            // Queue sizes sized to ~1s of buffering at each topic's publish rate. The
            // previous small sizes (1/4/16/32) caused the outgoing TCP send queue to overflow
            // continuously under WSL's virtualised loopback ("Queue full! Messages are getting
            // dropped!" in the Unity console), silently dropping IMU/LiDAR samples and
            // corrupting localisation accuracy independently of the wall-visibility fix.
            // Sized to ~2 s of buffering at each topic's ACTUAL publish rate. /clock and
            // /imu/data go out every FixedUpdate, i.e. 1 kHz, not the 250 Hz these queues were
            // originally cut for -- that left them holding only 0.25 s, and a transient stall
            // on WSL's virtualised loopback overflowed them within a couple of hundred
            // milliseconds ("Queue full! Messages are getting dropped!"). Losing IMU that way
            // trips the 2 s dead-reckoning watchdog and kills the whole run.
            ros.RegisterPublisher<ClockMsg>(ClockTopic, 500);
            ros.RegisterPublisher<LaserScanMsg>(ScanTopic, 80);
            ros.RegisterPublisher<OdometryMsg>(GroundTruthTopic, 200);
            // Register IMU last. Registering the larger message types can briefly block the
            // endpoint; publishing IMU before that makes the ROS watchdog see a false outage.
            ros.RegisterPublisher<ImuMsg>(ImuTopic, 2000);
            ros.Subscribe<OdometryMsg>(OdomFastTopic, OnOdomFastReceived);
            // GetOrCreateInstance can instantiate ROSConnection in this same Unity frame. Its
            // own Start() has not necessarily initialised yet, so wait one frame rather than
            // racing the connector's Play-mode lifecycle with an early connection thread.
            StartCoroutine(ConnectAfterRosConnectionStarts());

            previousWorldVelocity = body.linearVelocity;
            nextClockTime = 0.0;
            nextScanTime = 0.0;
            nextGroundTruthTime = 0.0;
            previousImuSampleTime = 0.0;
            // Seed the evolving bias at the same values PublishImu used to add as fixed
            // constants, so the starting behaviour is unchanged; only the drift over time is new.
            // (2026-07-28: bias/random-walk/sensor-noise isolation tests confirmed the
            // deceleration-linked spikes are independent of all three; reverted to adopted values.)
            accelBiasX = 0.05;
            accelBiasY = -0.03;
            gyroBiasZ = 0.005;
            Debug.Log("[Unity ROS Sensors] Unity physics publishers ready: " +
                $"/imu/data 1kHz, /clock 250Hz, /scan 40Hz, /ground_truth_pose 100Hz; " +
                $"ROS={ros.RosIPAddress}:{ros.RosPort}.");
        }

        private void OnDestroy()
        {
            // ROSConnection only closes its TCP connection (and the background thread that
            // owns it) from OnApplicationQuit, which Unity does NOT call when Play Mode is
            // stopped in the Editor -- only on a real application exit. Without this, each
            // Stop leaves the previous session's connection/thread alive; the next Play then
            // opens a second connection to the same ROS endpoint and the two compete, which
            // was observed to corrupt /clock delivery after a few Play/Stop cycles (message
            // sends silently failing partway through a run). Disconnecting here on the
            // GameObject's destruction -- which Play Mode Stop does trigger -- closes the
            // connection every time regardless of why the session is ending.
            if (ros != null) ros.Disconnect();
        }

        private IEnumerator ConnectAfterRosConnectionStarts()
        {
            yield return null;

            // A single one-shot Connect() here was observed to lose a race against the AI
            // Assistant/MCP package's own relay reconnect and asset-pipeline refresh churn
            // that happens in the same frame window when Play is entered via MCP automation
            // (EditorApplication.isPlaying = true) rather than a manual Editor click: the
            // outgoing per-topic queues (RegisterPublisher above) filled from FixedUpdate
            // with nothing ever consuming them, producing "Queue full!" from frame one and
            // zero ROS-side TCP accepts for the whole session. Retry with an explicit
            // HasConnectionThread check instead of trusting a single attempt. Disconnect()
            // first: Connect() unconditionally spawns a new connection thread with no guard
            // against a still-pending previous attempt, so retrying without disconnecting
            // would orphan it (the same class of bug OnDestroy() below guards against).
            const int maxAttempts = 10;
            const float retryIntervalSec = 1.0f;
            for (var attempt = 1; attempt <= maxAttempts; attempt++)
            {
                ros.Disconnect();
                ros.Connect();
                yield return new WaitForSecondsRealtime(retryIntervalSec);
                if (ros.HasConnectionThread)
                {
                    Debug.Log($"[Unity ROS Sensors] ROS connection thread confirmed alive " +
                        $"(attempt {attempt}/{maxAttempts}).");
                    yield break;
                }
                Debug.LogWarning($"[Unity ROS Sensors] ROS connection attempt {attempt}/{maxAttempts} " +
                    "has no connection thread yet; retrying.");
            }
            Debug.LogError("[Unity ROS Sensors] Failed to establish a ROS connection thread after " +
                $"{maxAttempts} attempts. Sensor data will not reach ROS this session.");
        }

        private void FixedUpdate()
        {
            // Relative to sessionStartTime (see its declaration) rather than
            // Time.timeAsDouble directly, so /clock always starts at 0 for this session.
            var simulationTime = Time.timeAsDouble - sessionStartTime;
            var stamp = ToRosTime(simulationTime);

            // Match the planned IMU update frequency.  Any communication loss must be fixed
            // in the transport/receiver path rather than hidden by lowering this sensor rate.
            if (simulationTime + 1e-9 >= nextClockTime)
            {
                ros.Publish(ClockTopic, new ClockMsg(stamp));
                nextClockTime += ClockPeriod;
            }
            PublishImu(stamp, simulationTime);

            if (simulationTime + 1e-9 >= nextGroundTruthTime)
            {
                PublishGroundTruth(stamp);
                nextGroundTruthTime += GroundTruthPeriod;
            }


            if (simulationTime + 1e-9 >= nextScanTime)
            {
                PublishScan(stamp);
                nextScanTime += LidarPeriod;
            }
        }

        private void PublishImu(TimeMsg stamp, double simulationTime)
        {
            // The acceleration sample spans the interval since the last published IMU value,
            // not an assumed fixed interval.  The nominal interval is 1 ms, but this keeps the
            // sample physically correct if an Editor frame delays an individual publication.
            var dt = initializedVelocity
                ? (float)Math.Max(simulationTime - previousImuSampleTime, 1e-6)
                : Time.fixedDeltaTime;
            var accelerationWorld = initializedVelocity
                ? (body.linearVelocity - previousWorldVelocity) / dt
                : Vector3.zero;
            initializedVelocity = true;
            previousWorldVelocity = body.linearVelocity;
            previousImuSampleTime = simulationTime;

            // An accelerometer measures specific force: a_world - gravity.
            var specificForceLocalUnity = transform.InverseTransformDirection(
                accelerationWorld - Physics.gravity);
            var angularVelocityLocalUnity = transform.InverseTransformDirection(body.angularVelocity);
            var accelerationRos = UnityVectorToRos(specificForceLocalUnity);
            var angularVelocityRos = UnityVectorToRos(angularVelocityLocalUnity);

            // Provisional real-sensor model from main.md.  Replace these constants with
            // the selected IMU data-sheet values once its exact model number is fixed.
            // TEMP DIAG (2026-07-28, revert after use): Gaussian noise fully disabled so the
            // raw published accel/gyro values around a deceleration-linked error spike can be
            // inspected without random per-sample jitter obscuring the underlying signal.
            const double accelNoiseSigma = 0.0;
            const double gyroNoiseSigma = 0.0;
            // A real IMU's bias is not a fixed offset: it wanders slowly over time (continuous-
            // time random walk), which is exactly what the estimator's bias_acc/gyro_random_walk_
            // sigma parameters (backend_optimizer_node) model and are meant to track. A constant
            // bias here only ever exercises the estimator's *initial* bias guess and can never
            // validate that tracking mechanism against anything resembling real hardware. Nudge
            // the running bias state each sample by a Gaussian step scaled by sqrt(dt) -- the
            // standard continuous-to-discrete conversion for a random walk -- so its statistics
            // match what a continuous-time density of biasRandomWalkSigma implies, regardless of
            // the actual sample interval. Keep this equal to the ROS-side default so the
            // estimator's assumed drift rate matches what is actually injected.
            // 2026-07-28: bias/random-walk isolation tests confirmed the deceleration-linked
            // error spikes are bias-independent (same magnitude with random walk on, fixed
            // bias, and zero bias). Reverted to the adopted value.
            const double biasRandomWalkSigma = 0.0001;
            var biasWalkStep = biasRandomWalkSigma * Math.Sqrt(dt);
            accelBiasX += Gaussian(biasWalkStep);
            accelBiasY += Gaussian(biasWalkStep);
            gyroBiasZ += Gaussian(biasWalkStep);
            accelerationRos.x += accelBiasX + Gaussian(accelNoiseSigma);
            accelerationRos.y += accelBiasY + Gaussian(accelNoiseSigma);
            accelerationRos.z += Gaussian(accelNoiseSigma);
            angularVelocityRos.x += Gaussian(gyroNoiseSigma);
            angularVelocityRos.y += Gaussian(gyroNoiseSigma);
            angularVelocityRos.z += gyroBiasZ + Gaussian(gyroNoiseSigma);

            var msg = new ImuMsg
            {
                header = MakeHeader(stamp, "base_link"),
                orientation = new QuaternionMsg(0.0, 0.0, 0.0, 1.0),
                orientation_covariance = new[] { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                angular_velocity = angularVelocityRos,
                angular_velocity_covariance = DiagonalCovariance3(gyroNoiseSigma * gyroNoiseSigma),
                linear_acceleration = accelerationRos,
                linear_acceleration_covariance = DiagonalCovariance3(accelNoiseSigma * accelNoiseSigma)
            };
            ros.Publish(ImuTopic, msg);
            ImuPublishCount++;
        }

        private void PublishGroundTruth(TimeMsg stamp)
        {
            var fieldPosition = new Vector3(transform.position.z, -transform.position.x,
                transform.position.y);
            var yaw = FieldYaw();
            var halfYaw = yaw / 2.0;

            var worldVelocityRos = UnityVectorToRos(body.linearVelocity);
            var cosYaw = Math.Cos(yaw);
            var sinYaw = Math.Sin(yaw);
            var bodyVx = cosYaw * worldVelocityRos.x + sinYaw * worldVelocityRos.y;
            var bodyVy = -sinYaw * worldVelocityRos.x + cosYaw * worldVelocityRos.y;
            var localAngularRos = UnityVectorToRos(
                transform.InverseTransformDirection(body.angularVelocity));

            var msg = new OdometryMsg
            {
                header = MakeHeader(stamp, "map"),
                child_frame_id = "base_link",
                pose = new PoseWithCovarianceMsg(
                    new PoseMsg(
                        new PointMsg(fieldPosition.x, fieldPosition.y, fieldPosition.z),
                        new QuaternionMsg(0.0, 0.0, Math.Sin(halfYaw), Math.Cos(halfYaw))),
                    new double[36]),
                twist = new TwistWithCovarianceMsg(
                    new TwistMsg(
                        new Vector3Msg(bodyVx, bodyVy, worldVelocityRos.z),
                        localAngularRos),
                    new double[36])
            };
            ros.Publish(GroundTruthTopic, msg);
        }

        private void PublishScan(TimeMsg stamp)
        {
            var ranges = new float[LidarSamples];
            // Visible arc starts where the blind sector ends and spans the full FOV.
            var angleMin = LidarBlindCentreRadians + (2f * Mathf.PI - LidarFovRadians) / 2f;
            var angleIncrement = LidarFovRadians / (LidarSamples - 1);
            // Use the visible LiDAR mount placed by BuildRobot (scan plane 140 mm above the
            // floor).  The scan plane must stay below the field wall top or no wall is seen
            // at all (see BuildRobot); the fallback matches the real mount rather than an
            // arbitrary chassis offset so it cannot silently produce a different plane.
            // Captured notes remain visible to these rays, so an unrealistic intake layout
            // cannot see through them.
            var lidarMount = transform.Find("Lidar");
            var origin = lidarMount != null
                ? lidarMount.position
                : new Vector3(transform.position.x, 0.14f, transform.position.z);

            for (var i = 0; i < LidarSamples; ++i)
            {
                var angle = angleMin + i * angleIncrement;
                // ROS body +x/+y maps to Unity local +z/-x.
                var localDirection = new Vector3(-Mathf.Sin(angle), 0f, Mathf.Cos(angle));
                var worldDirection = transform.TransformDirection(localDirection);
                var hitCount = Physics.RaycastNonAlloc(origin, worldDirection, rayHits,
                    LidarRangeMax, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
                var nearest = float.PositiveInfinity;
                for (var h = 0; h < hitCount; ++h)
                {
                    if (robotSelfColliders.Contains(rayHits[h].collider)) continue;
                    nearest = Mathf.Min(nearest, rayHits[h].distance);
                }
                if (float.IsFinite(nearest))
                {
                    // UTM-30LX catalogue accuracy is +/-30 mm within 10 m.  The
                    // established project sensor model treats this as sigma=30 mm.
                    nearest += (float)Gaussian(0.03);
                    ranges[i] = Mathf.Clamp(nearest, LidarRangeMin, LidarRangeMax);
                }
                else
                {
                    ranges[i] = float.PositiveInfinity;
                }
            }

            var msg = new LaserScanMsg(
                MakeHeader(stamp, "base_link"),
                angleMin,
                angleMin + angleIncrement * (LidarSamples - 1),
                angleIncrement,
                1e-9f, // instantaneous Unity ray batch; avoids fictitious ROS-side deskew
                LidarPeriod,
                LidarRangeMin,
                LidarRangeMax,
                ranges,
                Array.Empty<float>());
            ros.Publish(ScanTopic, msg);
            ScanPublishCount++;
        }

        private void OnOdomFastReceived(OdometryMsg msg)
        {
            var x = (float)msg.pose.pose.position.x;
            var y = (float)msg.pose.pose.position.y;
            var qz = msg.pose.pose.orientation.z;
            var qw = msg.pose.pose.orientation.w;
            var yaw = 2.0 * Math.Atan2(qz, qw);

            // /odom_fast's twist is body-frame (child_frame_id=base_link); rotate into the
            // world/field frame the mission controller's servo expects.
            var vxBody = msg.twist.twist.linear.x;
            var vyBody = msg.twist.twist.linear.y;
            var cosYaw = Math.Cos(yaw);
            var sinYaw = Math.Sin(yaw);
            var vxWorld = cosYaw * vxBody - sinYaw * vyBody;
            var vyWorld = sinYaw * vxBody + cosYaw * vyBody;

            EstimatedFieldPosition = new Vector2(x, y);
            EstimatedYaw = yaw;
            EstimatedFieldVelocityWorld = new Vector2((float)vxWorld, (float)vyWorld);
            HasOdomEstimate = true;
        }

        private double FieldYaw()
        {
            var forward = transform.forward;
            return Math.Atan2(-forward.x, forward.z);
        }

        private static TimeMsg ToRosTime(double seconds)
        {
            var sec = (int)Math.Floor(seconds);
            var nanosec = (uint)Math.Round((seconds - sec) * 1e9);
            if (nanosec >= 1_000_000_000)
            {
                sec += 1;
                nanosec -= 1_000_000_000;
            }
#if ROS2
            return new TimeMsg(sec, nanosec);
#else
            return new TimeMsg((uint)sec, nanosec);
#endif
        }

        private static HeaderMsg MakeHeader(TimeMsg stamp, string frameId)
        {
#if ROS2
            return new HeaderMsg(stamp, frameId);
#else
            return new HeaderMsg(0, stamp, frameId);
#endif
        }

        private static Vector3Msg UnityVectorToRos(Vector3 value) =>
            new(value.z, -value.x, value.y);

        private static double[] DiagonalCovariance3(double variance) => new[]
        {
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance
        };


        private double Gaussian(double sigma)
        {
            var u1 = Math.Max(random.NextDouble(), 1e-12);
            var u2 = random.NextDouble();
            return sigma * Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }
    }
}
