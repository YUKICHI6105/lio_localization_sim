using UnityEngine;

namespace Robocon2026.Simulation
{
    /// <summary>Orbit, zoom and robot-follow controls for the simulation Game view.</summary>
    [RequireComponent(typeof(Camera))]
    public sealed class SimulationCameraController : MonoBehaviour
    {
        [SerializeField] private Vector3 fieldFocusPoint = new(0f, 0.2f, 0f);
        [SerializeField] private float minimumDistance = 0.25f;
        [SerializeField] private float maximumDistance = 18f;
        [SerializeField] private float zoomExponentPerStep = 0.18f;
        [SerializeField] private float orbitDegreesPerMouseUnit = 4f;
        [SerializeField] private bool showControls = true;

        private Vector3 initialPosition;
        private Quaternion initialRotation;
        private Vector3 initialFieldFocus;
        private Transform followTarget;
        private float distance;
        private float yaw;
        private float pitch;
        private bool followingRobot;

        public bool FollowingRobot => followingRobot;
        public float Distance => distance;

        [SerializeField] private bool startOverheadOnRobot = true;
        [SerializeField] private float overheadPitch = 88f;
        [SerializeField] private float overheadDistance = 4f;

        private void Awake()
        {
            initialPosition = transform.position;
            initialRotation = transform.rotation;
            initialFieldFocus = fieldFocusPoint;
            CaptureOrbitFromCurrentTransform(fieldFocusPoint);

            if (startOverheadOnRobot)
            {
                // Robot GameObject may not exist yet (built in another script's Start());
                // LateUpdate() already retries FindRobotTarget() every frame while
                // followingRobot is true, so it picks the robot up as soon as it spawns.
                followingRobot = true;
                pitch = overheadPitch;
                yaw = 0f;
                distance = Mathf.Clamp(overheadDistance, minimumDistance, maximumDistance);
            }
        }

        private void Update()
        {
            var steps = Input.GetAxis("Mouse ScrollWheel") * 10f;
            if (Input.GetKey(KeyCode.Equals) || Input.GetKey(KeyCode.KeypadPlus))
                steps += 3f * Time.unscaledDeltaTime;
            if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus))
                steps -= 3f * Time.unscaledDeltaTime;
            if (Mathf.Abs(steps) > 0.0001f) ZoomBySteps(steps);

            if (Input.GetMouseButton(1))
            {
                yaw += Input.GetAxis("Mouse X") * orbitDegreesPerMouseUnit;
                pitch -= Input.GetAxis("Mouse Y") * orbitDegreesPerMouseUnit;
                pitch = Mathf.Clamp(pitch, 5f, 88f);
            }

            if (Input.GetKeyDown(KeyCode.F)) ToggleRobotFollow();
            if (Input.GetKeyDown(KeyCode.Home)) ResetView();
        }

        private void LateUpdate()
        {
            if (followingRobot && followTarget == null)
                FindRobotTarget();
            ApplyPose(CurrentFocusPoint());
        }

        public void ZoomBySteps(float steps)
        {
            distance = Mathf.Clamp(
                distance * Mathf.Exp(-steps * zoomExponentPerStep),
                minimumDistance,
                maximumDistance);
            ApplyPose(CurrentFocusPoint());
        }

        public bool ToggleRobotFollow()
        {
            followingRobot = !followingRobot;
            if (followingRobot && !FindRobotTarget())
                followingRobot = false;
            ApplyPose(CurrentFocusPoint());
            return followingRobot;
        }

        public void ResetView()
        {
            followingRobot = false;
            followTarget = null;
            fieldFocusPoint = initialFieldFocus;
            transform.SetPositionAndRotation(initialPosition, initialRotation);
            CaptureOrbitFromCurrentTransform(fieldFocusPoint);
        }

        private bool FindRobotTarget()
        {
            var robot = GameObject.Find("Omni3ValidationRobot");
            followTarget = robot == null ? null : robot.transform;
            return followTarget != null;
        }

        private Vector3 CurrentFocusPoint()
        {
            if (followingRobot && followTarget != null)
                return followTarget.position + Vector3.up * 0.08f;
            return fieldFocusPoint;
        }

        private void CaptureOrbitFromCurrentTransform(Vector3 focus)
        {
            distance = Mathf.Clamp(Vector3.Distance(transform.position, focus),
                minimumDistance, maximumDistance);
            var euler = transform.rotation.eulerAngles;
            pitch = NormalizePitch(euler.x);
            yaw = euler.y;
        }

        private void ApplyPose(Vector3 focus)
        {
            var rotation = Quaternion.Euler(pitch, yaw, 0f);
            transform.SetPositionAndRotation(
                focus - rotation * Vector3.forward * distance,
                rotation);
        }

        private static float NormalizePitch(float value)
        {
            if (value > 180f) value -= 360f;
            return Mathf.Clamp(value, 5f, 88f);
        }

        private void OnGUI()
        {
            if (!showControls) return;
            var mode = followingRobot ? "Robot follow" : "Field orbit";
            GUI.Label(new Rect(12f, 12f, 520f, 24f),
                "Zoom: wheel / +/-    Orbit: right-drag    Follow: F    Reset: Home");
            GUI.Label(new Rect(12f, 34f, 300f, 24f),
                $"Camera: {mode}    Distance: {distance:F2} m");
        }
    }
}
