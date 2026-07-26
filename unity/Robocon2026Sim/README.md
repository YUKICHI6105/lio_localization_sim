# Robocon 2026 Unity simulation

Unity 6.5 / URP 17.5 project for the Chiba University Robot Contest 2026 field.
The authoritative geometry remains
`../../src/lio_localization_sim/config/robocon2026_field.json`.

## First launch

1. Run `python3 tools/sync_unity_field.py` from this directory.
2. Run `python3 tools/deploy_to_windows.py`. Windows Unity does not support a
   project stored directly on WSL's case-sensitive filesystem, so this creates
   `C:\\Users\\kouza\\UnityProjects\\Robocon2026Sim`.
3. In Unity Hub, add that Windows directory and open it with Unity 6.5.
4. Wait until Package Manager finishes resolving URP, Newtonsoft JSON and the
   official ROS-TCP-Connector.
5. Run `Robocon 2026 > Configure URP and Simulation Scene` once.
6. Press Play. The field is generated from `Assets/StreamingAssets/robocon2026_field.json`.

The Console prints the actual graphics device, API, compute-shader support and
GPU-instancing support. On Windows, prefer Direct3D 12 or Direct3D 11. Do not
run the Unity Editor itself through WSLg; run the Windows Editor and connect it
to ROS 2 in WSL through ROS-TCP-Endpoint.

## Accuracy and performance defaults

- Field coordinates are converted exactly as
  `(Unity X, Unity Y, Unity Z) = (-field Y, field Z, field X)`.
- Physics uses a fixed 4 ms step (250 Hz), 12 position iterations and 4 velocity
  iterations. Dynamic notes use continuous collision detection.
- Field meshes share instanced materials, are static, cast no shadows and use
  the SRP Batcher. Rendering is VSync-limited to 60 Hz.
- In Project Settings > Graphics > URP, enable GPU Resident Drawer. GPU
  occlusion is optional and may cost more than it saves on this small field.

PhysX rigid-body contacts remain CPU calculations. Rendering and future camera,
depth and LiDAR rasterization / compute shaders use the GPU. Friction values in
`RoboconFieldBuilder` are initial calibration values, not rule-defined values;
replace them with measurements from the actual floor, omni wheels and balls.

## ROS 2

The project declares Unity's official ROS-TCP-Connector package. After package
resolution, open `Robotics > ROS Settings`, select ROS 2, and set the endpoint
address. Keep strategy, localization and scoring in ROS 2; Unity should publish
sensor observations and ground truth and subscribe to actuator commands.
