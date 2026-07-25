# Robocon 2026 Start-to-Bingo baseline test and algorithm survey

Date: 2026-07-23  
Scope: left start zone, two-note pickup, slalom, stop 50 mm before bingo

## Executive conclusion

The present waypoint + quintic minimum-jerk + Cartesian PD implementation is accepted only as a
deterministic feasibility baseline. It completed the final Unity physics test with no collision,
0.358 mm pickup position error, 9.939 mm/s pickup speed, and 0.312 mm final position error.

It should **not** be selected as the final fastest algorithm. Every intermediate waypoint is an
endpoint of an independent quintic segment, so the robot unnecessarily slows to zero inside the
slalom. The recommended competition architecture is:

1. offline free-final-time trajectory optimization by direct collocation or multiple shooting;
2. online nonlinear MPC (NMPC) tracking with feed-forward controls;
3. Nav2 MPPI as a robustness benchmark or fallback, not the primary deterministic race-line solver;
4. Ruckig and TOPP-RA as lower-complexity timing benchmarks.

## Fixed mission definition

All coordinates use the engine-neutral field frame: origin at field-floor centre, +x from start to
bingo, +y toward the left-team side.

| Item | Value |
|---|---:|
| Chassis | 500 mm equilateral triangle, 50 mm corner radius |
| Start centroid | (-2.419, 1.354) m |
| Pickup notes | Orange indices 4 and 5, the slalom-side pair |
| Pickup centroid | (-1.799, 0.394337567) m |
| Bingo stop centroid | (2.316, 0.494337567) m |
| Surface clearance | 50 mm at pickup and bingo |
| Heading | fixed; one triangle side faces the notes/bingo |
| Hard stop conditions | start, pickup, finish |
| Target speed limit | 5.0 m/s |
| Target acceleration limit | 4.905 m/s^2 (0.5 G) |
| Target yaw-rate limit | 2π rad/s |
| Simulated wheel diameter | 100 mm; 60 mm retained as an option |

The 50 mm pickup gap is currently a capture-plane abstraction. A real intake must extend across
that gap; intake dynamics are not yet simulated.

## Baseline algorithm

### Path generation

The rounded-triangle footprint was considered when manually choosing collision-free waypoints:

```text
start
 -> pickup/full stop
 -> (-0.450, 0.500)
 -> (-0.300, 1.060)
 -> ( 0.350, 1.060)
 -> bingo/full stop
```

Each segment uses the normalized quintic minimum-jerk blend

```text
p(u) = 10u^3 - 15u^4 + 6u^5,  0 <= u <= 1
```

This gives zero velocity and acceleration at each segment boundary. Segment duration is chosen
from the analytical peak factors of the quintic:

```text
T = 1.08 * max(1.875 d / vmax, sqrt(5.774 d / amax))
```

The ideal command schedule is 6.528434 s, including a 0.4 s pickup dwell. This is not an observed
lap time: the final test did not log the exact completion instant, only that it had completed when
queried. Therefore this test establishes feasibility and stop accuracy, not speed performance.

### Tracking

Unity applies a world-frame Cartesian PD servo to the Rigidbody:

```text
F = m * (60 position_error + 12 velocity_error)
|F| <= m * 4.905
```

Yaw is fixed. This deliberately avoids assuming a motor, gear ratio, or wheel torque curve. The
three wheel meshes are visual kinematics; the current baseline is not a contact-level omni-wheel
or roller model. Pickup and final completion require position error <= 5 mm and planar speed <=
10 mm/s. The timeline is held at pickup until the physical body satisfies both conditions.

## Final baseline test

Single final run, Unity 6.5.4f1 physics, 250 Hz fixed update:

| Metric | Result |
|---|---:|
| Completed | yes |
| Pickup position error | 0.358 mm |
| Pickup planar speed | 9.939 mm/s |
| Final field position | (2.316299, 0.494251) m |
| Final target error | 0.312 mm |
| Final velocity when inspected | 0 m/s |
| Recorded obstacle collisions | 0 |

Rendering was verified as 1920x1080 Game view, URP 4x MSAA, camera SMAA High. Wheel and LiDAR
cylinders use 64 radial divisions (128 vertices and 128 side triangles for a wheel mesh).

Limitations of this test:

- no measured completion timestamp;
- no motor, battery, gearbox, wheel slip, roller vibration, compliance, or intake dynamics;
- exact ground-truth odometry integration is not a realistic localization test;
- one nominal run is not a robustness/statistical result;
- collision checking records contacts but this run did not sweep friction, latency, or pose noise.

## Algorithm market survey

### Candidate comparison

| Method | Main strength | Main weakness here | Recommended role |
|---|---|---|---|
| Current minimum-jerk + PD | deterministic, transparent, easy to debug | stops at every waypoint; no joint path/time optimum | frozen baseline |
| Ruckig | sub-ms jerk-limited state-to-state online generation; time-optimal state transitions | Community edition is not a collision-aware multi-waypoint race-line optimizer | timing/control benchmark and emergency stop |
| TOPP-RA | time-optimal parameterization of a fixed geometric path under second-order constraints | cannot improve the path geometry; performance depends on supplied path | fast timing benchmark after spline generation |
| TEB/TESC | jointly adjusts poses and time online; mature mobile-base idea | nonconvex/local minima; less direct control over exact rounded-triangle and full dynamic model | secondary prototype |
| Nav2 MPPI | omnidirectional model, nonconvex/non-differentiable critics, online obstacle response | stochastic; many tuning weights; no guarantee of globally fastest nominal run | robustness benchmark/fallback |
| Smac lattice / Informed RRT* | explores collision-free topology and supplies feasible seeds | geometric planner, not a final dynamic time-optimal trajectory | multi-start initializer |
| Direct collocation/multiple shooting + NMPC | optimizes geometry, timing, dynamics, footprint and stops in one formulation | highest implementation effort; local NLP solution needs good seeds | **primary recommendation** |

### Evidence from maintained implementations and papers

- CasADi explicitly supports direct single shooting, multiple shooting, and direct collocation for
  optimal-control problems. Its documentation notes that multiple shooting can converge better
  than single shooting when initialized with a known state trajectory, while collocation yields a
  larger but very sparse NLP: <https://web.casadi.org/docs/>.
- acados targets nonlinear optimal-control structured problems and provides SQP/SQP-RTI,
  multiple-shooting formulations, high-performance QP solvers, and generated C suitable for
  embedded real-time NMPC: <https://docs.acados.org/>.
- Nav2 MPPI officially supports an omnidirectional motion model, uses batches of sampled controls
  scored by critic plugins, and is reported by Nav2 to run above 100 Hz on a fourth-generation
  Intel i5. The documented nominal batch is 1000 trajectories at 50 Hz or 2000 at 30 Hz:
  <https://docs.nav2.org/configuration/packages/configuring-mppic.html>.
- Nav2 MPPI supports lateral velocity/acceleration constraints and polygon-footprint collision
  evaluation, which makes it a credible three-omni benchmark, but its stochastic critic optimum is
  not a proof of minimum traversal time.
- Ruckig documents jerk-limited, time-optimal state-to-state generation and sub-millisecond online
  calculation. Its public feature table places intermediate-waypoint optimization in Pro, so the
  open Community edition is most useful here for point-to-point timing and stopping:
  <https://ruckig.com/>.
- TOPP-RA uses reachability analysis to compute a time-optimal parameterization of a supplied path:
  <https://arxiv.org/abs/1707.07239>. It is attractive when geometry is already fixed, but it does
  not discover a better slalom line.
- TEB optimizes execution time, obstacle avoidance and kinodynamic constraints online; the project
  overview is maintained by TU Dortmund:
  <https://rst.etit.tu-dortmund.de/en/research/robotics/online-trajectory-optimization-based-on-timed-elastic-ban/>.
- Nav2's maintained repository includes Smac state-lattice and MPPI components, and cites its
  algorithm survey and Smac work: <https://github.com/ros-navigation/navigation2>.

## Recommended next implementation

### Offline race-line optimizer

Use CasADi + IPOPT first for development. Optimize the state

```text
[x, y, yaw, vx, vy, yaw_rate]
```

and control

```text
[ax, ay, yaw_acceleration]
```

with free phase durations. Encode:

- exact start, pickup, and finish poses;
- zero velocity at start, pickup, and finish;
- 0.4 s or mechanism-derived pickup dwell;
- speed, acceleration, yaw-rate, yaw-acceleration, and later jerk/torque limits;
- signed-distance constraints between the rounded triangular footprint and every wall/bingo part;
- localization/simulation safety margin as a tunable robust constraint;
- objective `total_time + small jerk cost + small clearance risk cost`.

Run several initial seeds: the current baseline, a TEB-like spline, and lattice/RRT* alternatives.
The fixed field has very few relevant homotopy classes, so multi-start offline optimization is
practical and avoids relying on one local solution.

### Online tracker

Generate an acados C solver for NMPC at an initial target of 100 Hz. Track the optimized state and
feed-forward acceleration, enforce the same hard dynamic limits, and include measured command
delay. Begin with a centre-of-mass omni model; add wheel-speed/torque constraints only after the
60 mm versus 100 mm hardware choice and system identification.

### Required benchmark before selection

Compare four pipelines in identical Unity trials:

1. frozen minimum-jerk + PD baseline;
2. spline + TOPP-RA/Ruckig timing + feedback;
3. direct-collocation trajectory + acados NMPC;
4. Nav2 Omni MPPI.

For each pipeline, run nominal tests and Monte Carlo sweeps over localization error, command delay,
friction, mass, centre-of-mass offset, and note contact. Rank by 95th-percentile completion time
subject to zero collision and successful stop/capture, rather than by the single fastest run.

