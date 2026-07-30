# lio_localization_sim documentation

## Start here

- [Project status overview (2026-07-30)](project_status_overview_2026-07-30.md): where the
  whole autonomy stack stands right now (localisation, mechanisms, path planning), mapped
  against the `robocon2026_autonomy_plan.md` stage gates.
- [Environment restore](setup/RESTORE.md): WSL2 limits, source layout, ROS build,
  Unity deployment, Claude Code configuration, and measurement order.
- [Requirements spec (consolidated, 2026-07-27)](requirements/requirements_spec_2026-07-27.md): single
  source of truth for competition, localisation, and simulation-fidelity requirements,
  with a dated change history of how each threshold evolved.
- [Localisation final report (2026-07-30)](requirements/localization_final_report_2026-07-30.md):
  requirement A (steady RMSE and max position error both <=10mm) is now PASS; full fix
  history, remaining residual characterisation, and evidence.
- [Stage 4 resume notes](experiment_history/04_stage4_resume_notes.md): the current Unity/ROS
  localisation status, validated operating procedure, known limitations, and next experiments.

## Requirements (`requirements/`)

- [Requirements spec (consolidated, 2026-07-27)](requirements/requirements_spec_2026-07-27.md):
  competition rules, localisation acceptance criteria, fault-tolerance criteria, and
  simulation-fidelity thresholds, with a change-history table.
- [Localisation requirement update (2026-07-27)](requirements/localization_requirement_update_2026-07-27.md):
  the standalone announcement of the current 10 mm maximum-error acceptance criterion.
- [Localisation final report (2026-07-30)](requirements/localization_final_report_2026-07-30.md):
  the requirement A PASS determination, its evidence, and the residual-error root-cause chain
  (experiment_history 09-15).
- [Backend estimator architecture (2026-07-28, updated 2026-07-30)](requirements/backend_estimator_architecture_2026-07-28.md):
  factor-graph structure, data flow, and thread/executor design for both ② and ③'s dedicated
  worker threads, with a Mermaid diagram.

## Experiment history (`experiment_history/`, by execution environment)

- [Stage 1-3: analytic sim (no physics engine), known-map ICP](experiment_history/02_stage1-3_synthetic_sim_summary.md):
  no Gazebo, no Unity; `imu_sim_node`/`lidar_sim_node` synthesise noisy IMU/LiDAR data
  directly from an analytic trajectory, visualised in rviz2. Reached RMSE~1mm / max<6mm.
  Underlying raw investigation log: [ICP outlier investigation](experiment_history/01_icp_outlier_investigation.md).
- [Stage 4: Unity + ROS production, closed-loop](experiment_history/07_stage4_unity_production_summary.md):
  full-stack runs, including the full 2026-07-27 max-10mm campaign table.
  Underlying raw notes: [Stage 4 resume notes](experiment_history/04_stage4_resume_notes.md),
  [Unity localisation handoff](experiment_history/03_unity_localization_handoff_2026-07-23.md),
  [10mm validation report](experiment_history/06_localization_10mm_validation_report_2026-07-27.md),
  [Deceleration-linked spike investigation, 2026-07-28](experiment_history/08_deceleration_spike_investigation_2026-07-28.md):
  bias/noise/smoother isolation tests ruling out the estimator side; raw-IMU inspection found
  the 0.5G design cap exceeded ~25% but with no functional effect on the current gates.
- [09: backend dedicated optimizer worker](experiment_history/09_backend_worker_evaluation_2026-07-29.md),
  [10: callback-delay component isolation](experiment_history/10_callback_delay_component_isolation_2026-07-29.md),
  [11: maximum-10mm root cause (notes-exclusion clustering bug)](experiment_history/11_maximum_10mm_root_cause_investigation_2026-07-29.md),
  [12: sensor-noise datasheet validation (RMSE 0.65mm/max 2.18mm PASS)](experiment_history/12_sensor_noise_datasheet_validation_2026-07-29.md),
  [13: scan-coverage timeout investigation](experiment_history/13_scan_coverage_timeout_investigation_2026-07-30.md),
  [14: scan-matching ICP worker-thread separation (max 17.55mm -> 4-5mm)](experiment_history/14_scan_worker_thread_separation_2026-07-30.md),
  [15: residual spike root cause (notes-exclusion + turn-onset dynamics, transport-delay hypothesis disproven)](experiment_history/15_residual_spike_root_cause_20260730.md):
  the chain of investigations that took requirement A from a long-standing FAIL to a
  reproducible PASS. Summarised end-to-end in the
  [localisation final report](requirements/localization_final_report_2026-07-30.md).
- [Ground-truth-seeded diagnostics](experiment_history/05_truth_diagnostics_summary.md):
  offline ICP-from-truth and seed-mode comparisons used to isolate bias sources; not
  accuracy measurements in their own right.

## Running the simulator

In a new shell, initialise both ROS and the built workspace:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_ws/install/setup.bash
```

For the lightweight ROS-only simulator, use `sim_test.launch.py`.  For Unity sensor
integration, start `unity_sensor_localization.launch.py` first, wait for its READY
marker, then enter Unity Play mode.  The scripted Stage 4 flows and their safeguards are
described in [Stage 4 resume notes](experiment_history/04_stage4_resume_notes.md).

## Design and research

- [Robocon 2026 autonomy plan](robocon2026_autonomy_plan.md)
- [Unity 6 migration plan](unity6_simulation_plan.md)
- [Start-to-Bingo baseline and algorithm survey](start_to_bingo_baseline_and_algorithm_survey.md)

## Records and assets

- `session_results/`: reproducible results, analysis scripts, and historical Stage 2/3 runs.
- `stage4_runs/`: Stage 4 run logs and reports; preserve these as measurement evidence.
- `robocon2026_field_preview.svg`: field visualisation.
- Claude Code settings have been restored to their user and workspace locations; they are
  intentionally not retained as a copy in this repository.

## Conventions

Documents in this directory are retained as dated technical records.  Prefer adding a
new dated handoff or result document to rewriting past measurements; update this index
when adding a new primary entry point.  Requirements documents live under `requirements/`;
experiment records and raw investigation logs live under `experiment_history/`, grouped by
execution environment (analytic sim with no physics engine, Unity production, ground-truth
diagnostics).
