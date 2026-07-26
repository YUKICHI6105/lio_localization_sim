# lio_localization_sim documentation

## Start here

- [Environment restore](setup/RESTORE.md): WSL2 limits, source layout, ROS build,
  Unity deployment, Claude Code configuration, and measurement order.
- [Stage 4 resume notes](stage4_resume_notes.md): the current Unity/ROS localisation
  status, validated operating procedure, known limitations, and next experiments.
- [Unity localisation handoff](unity_localization_handoff_2026-07-23.md): sensor model,
  estimator parameters, and the earlier validation results that led to Stage 4.

## Running the simulator

In a new shell, initialise both ROS and the built workspace:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
```

For the lightweight ROS-only simulator, use `sim_test.launch.py`.  For Unity sensor
integration, start `unity_sensor_localization.launch.py` first, wait for its READY
marker, then enter Unity Play mode.  The scripted Stage 4 flows and their safeguards are
described in [Stage 4 resume notes](stage4_resume_notes.md).

## Design and research

- [Robocon 2026 autonomy plan](robocon2026_autonomy_plan.md)
- [Unity 6 migration plan](unity6_simulation_plan.md)
- [Start-to-Bingo baseline and algorithm survey](start_to_bingo_baseline_and_algorithm_survey.md)
- [ICP outlier investigation](icp_outlier_investigation.md)

## Records and assets

- `session_results/`: reproducible results, analysis scripts, and historical Stage 2/3 runs.
- `stage4_runs/`: Stage 4 run logs and reports; preserve these as measurement evidence.
- `robocon2026_field_preview.svg`: field visualisation.
- Claude Code settings have been restored to their user and workspace locations; they are
  intentionally not retained as a copy in this repository.

## Conventions

Documents in this directory are retained as dated technical records.  Prefer adding a
new dated handoff or result document to rewriting past measurements; update this index
when adding a new primary entry point.
