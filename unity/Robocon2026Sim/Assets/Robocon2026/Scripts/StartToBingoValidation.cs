using System;
using System.Collections.Generic;
using UnityEngine;

namespace Robocon2026.Simulation
{
    /// <summary>
    /// Physics baseline for the left start -> two-note pickup -> slalom -> bingo mission.
    /// Geometry and mission poses come from the engine-neutral field JSON.
    /// </summary>
    public sealed class StartToBingoValidation : MonoBehaviour
    {
        private readonly List<Vector2> route = new();
        // The outbound waypoints, cached so a return leg can be built from them without
        // re-reading the mission. The return is not a plain reversal: the outbound pickup sits
        // 20 mm from the orange notes and is only safe at the crawl speed used to pick up, so
        // driving it backward at full speed clips them. The return replaces that pickup point
        // with a bypass raised well above the note row.
        private readonly List<Vector2> outboundRoute = new();
        // Return-leg clearance waypoints (see StartNextLap). cross_low keeps the baffle-line
        // crossing low; bypass then rises above the note row. Left of the baffle at x=-0.779.
        private const float ReturnCrossLowX = -1.1f;
        private const float ReturnCrossLowY = 0.5f;
        private const float ReturnPickupBypassY = 1.1f;
        private readonly List<float> segmentDurations = new();
        // 各区間の終端で向くべきヨー(Unity座標系、rad)。区間iはsegmentYawEnd[i]
        // まで、位置と同じタイムライン(MinimumJerk, segmentDurations[i])でなめらかに
        // 回転する。開始時の向き(区間0の始点)は機体のスポーン姿勢(yaw=0)。
        private readonly List<float> segmentYawEnd = new();
        // 現在のラップの区間0が始まる時点で向いているべきヨー(Unity座標系、rad)。
        // 初回は機体のスポーン姿勢(0)。ラップ反転時はStartNextLapで前ラップの
        // 最終区間の向きを引き継ぎ、ラップ間で不連続なヨーの飛びが起きないようにする。
        private float startYawRad;
        private Rigidbody body = null!;
        private UnityRosSensorPublisher sensorPublisher = null!;
        private Transform robot = null!;
        private Transform[] wheels = Array.Empty<Transform>();
        private CollisionRecorder collisionRecorder = null!;
        private MissionDefinition mission = null!;
        private RobotDefinition robotDefinition = null!;
        private float startTime;
        private float routeDuration;
        private float pickupTime;
        private float pickupDelay;
        private float wheelRadius;
        private bool notesCaptured;
        private Vector2 estimatedPosition;
        private float squaredErrorSum;
        private float maxOdometryError;
        private int samples;
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
        // Match the real startup sequence: keep the chassis stationary while the IMU prior and
        // first LiDAR factors settle, then run the complete mission. Previously the robot moved
        // after only 0.5 s, so the reported maximum mixed filter startup convergence into the
        // motion requirement. The evaluator remains active throughout this warm-up; no samples
        // are hidden, and every moving sample is still included in the max-error result.
        private const float LocalizationWarmupSeconds = 10.0f;
        private int lapCount;
        private bool routeReversed;

        public bool Finished { get; private set; }
        public Vector2 FinalFieldPosition { get; private set; }
        public float FinalTargetErrorMillimetres { get; private set; }
        public float OdometryRmseMillimetres { get; private set; }
        public float OdometryMaxErrorMillimetres { get; private set; }
        public float PickupPositionErrorMillimetres { get; private set; }
        public float PickupSpeedMillimetresPerSecond { get; private set; }
        public string CollisionNames => collisionRecorder == null ? "" : collisionRecorder.CollisionNames;

        private void Start()
        {
            var definition = GetComponent<RoboconFieldBuilder>().Definition;
            mission = definition.Missions.StartToBingoLeft;
            robotDefinition = definition.Robot;
            BuildRoute();
            BuildRobot();
            sensorPublisher = robot.GetComponent<UnityRosSensorPublisher>();
            estimatedPosition = route[0];
            startTime = Time.time + LocalizationWarmupSeconds;
            Debug.Log(
                $"[StartToBingo] start={route[0]}, pickup={route[1]}, finish={route[^1]}, " +
                $"wheel={wheelRadius * 2000f:F0}mm, duration={routeDuration:F2}s");
        }

        private void BuildRoute()
        {
            route.Add(ToVector(mission.Start));
            route.Add(ToVector(mission.Pickup));
            foreach (var point in mission.SlalomWaypoints) route.Add(ToVector(point));
            route.Add(ToVector(mission.Finish));
            outboundRoute.Clear();
            outboundRoute.AddRange(route);
            BuildTimeline();
        }

        // Per-segment minimum-jerk durations for the current route ordering. Split out from
        // BuildRoute so a reversed lap can recompute timing without re-adding waypoints. The
        // pickup dwell is only inserted on the outbound leg (segment 0 -> pickup); a reversed
        // lap has no pickup and gets no dwell.
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
                // 区間iの終端で向くべきヨー: 区間の進行方向(フィールド座標の
                // delta)を、FixedUpdateの速度変換と同じ規約(Unity x=-field dy,
                // Unity z=field dx)でUnity座標系のradへ変換する。
                var delta = route[i + 1] - route[i];
                segmentYawEnd.Add(Mathf.Atan2(-delta.y, delta.x));
                if (i == 0 && !routeReversed)
                {
                    pickupTime = duration;
                    routeDuration += (float)mission.PickupDwellSeconds;
                }
            }
        }

        private void FixedUpdate()
        {
            // Script recompilation during Play mode clears non-serialized route data.
            // A fresh Play run rebuilds it in Start(); avoid a hot-reload error storm meanwhile.
            if (Finished || body == null || route.Count < 2) return;
            var elapsed = Time.time - startTime;
            if (elapsed < 0f) return;

            var effectiveElapsed = elapsed - pickupDelay;
            if (!notesCaptured && effectiveElapsed >= pickupTime)
            {
                // Hold the timeline at pickup until the physical body is genuinely stationary.
                effectiveElapsed = pickupTime;
                var pickupError = Vector2.Distance(UnityToField(body.position), route[1]);
                var planarSpeed = new Vector2(body.linearVelocity.x, body.linearVelocity.z).magnitude;
                if (pickupError <= 0.005f && planarSpeed <= 0.010f)
                {
                    PickupPositionErrorMillimetres = pickupError * 1000f;
                    PickupSpeedMillimetresPerSecond = planarSpeed * 1000f;
                    CaptureNotes();
                    notesCaptured = true;
                    pickupDelay = elapsed - pickupTime;
                }
            }
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
            if (sensorPublisher != null && sensorPublisher.HasOdomEstimate)
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
            // A firm Cartesian servo is used instead of assuming a motor/gearing model.
            // Output remains capped by the mission's 0.5 G target acceleration.
            body.WakeUp();
            var force = body.mass * (60f * positionError + 12f * velocityError);
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
            RotateWheelVisuals(Time.fixedDeltaTime);

            var finalError = Vector2.Distance(truth, route[^1]);
            var finalPlanarSpeed = new Vector2(body.linearVelocity.x, body.linearVelocity.z).magnitude;
            if (routeDone && finalError <= 0.005f && finalPlanarSpeed <= 0.010f)
                Complete(truth);
        }

        private void Complete(Vector2 truth)
        {
            OdometryRmseMillimetres = samples > 0 ? Mathf.Sqrt(squaredErrorSum / samples) * 1000f : 0f;
            OdometryMaxErrorMillimetres = maxOdometryError * 1000f;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            Debug.Log(
                $"[StartToBingo RESULT lap={lapCount}] final=({truth.x:F3},{truth.y:F3})m " +
                $"target_error={Vector2.Distance(truth, route[^1]) * 1000f:F1}mm " +
                $"wheel_odom_rmse={OdometryRmseMillimetres:F2}mm " +
                $"wheel_odom_max={OdometryMaxErrorMillimetres:F2}mm " +
                $"pickup_error={PickupPositionErrorMillimetres:F2}mm " +
                $"pickup_speed={PickupSpeedMillimetresPerSecond:F2}mm/s " +
                $"notes_captured={notesCaptured} collisions={CollisionNames}");

            if (RepeatLaps)
            {
                StartNextLap();
                return;
            }
            Finished = true;
            FinalFieldPosition = truth;
            FinalTargetErrorMillimetres = Vector2.Distance(truth, route[^1]) * 1000f;
        }

        // Reverse the waypoint list and rebuild the timeline so the robot drives back the way
        // it came. The robot body, its Rigidbody state and the ROS publisher are all left
        // untouched; only the route and the per-run timing/pickup state reset, so localisation
        // continues uninterrupted across laps.
        private void StartNextLap()
        {
            lapCount++;
            routeReversed = !routeReversed;
            route.Clear();
            if (routeReversed)
            {
                // Return leg. Retrace the outbound slalom (finish -> s2 -> s1 -> s0), which is
                // clean, then replace the pickup approach with two waypoints that keep the
                // notes and baffle_left_1 clear at full speed:
                //   - cross_low crosses the x=-0.779 baffle line at y=0.5, so the 0.29 m
                //     chassis reaches only y=0.79, below the baffle bottom at 0.869. (The
                //     naive reversal crossed at y=0.62, putting the chassis top at 0.91 into
                //     the baffle -- the collision seen on the return laps.)
                //   - bypass then rises left of the baffle, well above the y=0.125 note row.
                // finish, s2, s1, s0 are outbound indices 5..2; pickup (index 1) is dropped.
                for (var i = outboundRoute.Count - 1; i >= 2; --i) route.Add(outboundRoute[i]);
                route.Add(new Vector2(ReturnCrossLowX, ReturnCrossLowY));
                route.Add(new Vector2(outboundRoute[1].x, ReturnPickupBypassY));
                route.Add(outboundRoute[0]);
            }
            else
            {
                route.AddRange(outboundRoute);
            }
            // 前ラップの最終区間の向きを、新ラップの区間0が始まる時点の向きとして
            // 引き継ぐ(ヨーがラップの継ぎ目で不連続に飛ばないようにするため)。
            startYawRad = segmentYawEnd[^1];
            segmentDurations.Clear();
            segmentYawEnd.Clear();
            routeDuration = 0f;
            pickupTime = 0f;
            BuildTimeline();
            startTime = Time.time + 0.25f;
            pickupDelay = 0f;
            // Pickup is a physical event that only happens once, on the original outbound leg.
            notesCaptured = routeReversed;
        }

        private void BuildRobot()
        {
            var previous = GameObject.Find("Omni3ValidationRobot");
            if (previous != null) Destroy(previous);

            var robotObject = new GameObject("Omni3ValidationRobot");
            robot = robotObject.transform;
            robot.SetParent(transform, false);
            robot.position = FieldCoordinates.ToUnity(route[0].x, route[0].y, 0.065);

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
            Debug.Log("[StartToBingo] Pickup stop reached; cargo capture is disabled for this baseline.");
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
            var elapsed = 0f;
            for (var i = 0; i < segmentDurations.Count; ++i)
            {
                if (i == 1 && !routeReversed)
                {
                    var dwell = (float)mission.PickupDwellSeconds;
                    if (time <= elapsed + dwell)
                    {
                        position = route[1];
                        velocity = Vector2.zero;
                        // 停止中(ピックアップ待機)はその時点までに向いた区間0の
                        // 向きを保持する。
                        yawRad = segmentYawEnd[0];
                        yawRateRad = 0f;
                        done = false;
                        return;
                    }
                    elapsed += dwell;
                }

                var duration = segmentDurations[i];
                if (time <= elapsed + duration)
                {
                    MinimumJerk((time - elapsed) / duration, out var blend, out var derivative);
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
                triangles.AddRange(new[] { bottomCentre, b1, b0, topCentre, t0, t1,
                    b0, b1, t1, b0, t1, t0 });
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
                triangles.AddRange(new[] { a, b, b + 1, a, b + 1, a + 1 });
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

        private static Vector2 UnityToField(Vector3 value) => new(value.z, -value.x);

        private static Material MakeMaterial(string name, Color colour)
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            var material = new Material(shader) { name = name, color = colour, enableInstancing = true };
            if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", colour);
            return material;
        }
    }

    public sealed class CollisionRecorder : MonoBehaviour
    {
        private readonly HashSet<string> collisionNames = new();
        public string CollisionNames => string.Join(",", collisionNames);

        private void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.name == "Floor") return;
            collisionNames.Add(collision.gameObject.name);
            Debug.LogWarning($"[StartToBingo] Collision with {collision.gameObject.name}");
        }
    }
}
