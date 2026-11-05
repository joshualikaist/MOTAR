# SITL evidence (2026-11-05): PX4 v1.14.3 + Gazebo Classic 11 before real flight

Plan: [`docs/plans/sitl_validation_plan_v1.md`](../../plans/sitl_validation_plan_v1.md). The user's goal is real-vehicle
flight; SITL checks first whether Policy F can close the loop through PX4.
- PX4 checkout `/home/fair/workspaces/px4_ws/PX4-Autopilot` at v1.14.3 (`1dacb4cd`). It has three local changes,
  recorded in `vehicle/s2_install.out`: the navrl_quad model, airframe 10090, and the registration of both.
- NuttX tag `nuttx-10.3.0` was fetched so that PX4's version header builds.
- System packages were installed by the user on 2026-11-04 (19:24-19:27).
- Runs: headless, on cores 6,7,12,13 at `nice -n 10`, with no CUDA, alongside GR-1b PPO training on the GPU.

## `s1s2/`: offboard tests A0-A6 (exploratory, not preregistered)

- **Client:** `s12_offboard_mavlink.py` uses pymavlink on UDP 14540. MAVSDK 4.x is installed, but its native API
  differs, and installing the gRPC package was denied.
- **Command path:** body (x, y) velocity, rotated by the measured yaw to NED; altitude held by a z position setpoint;
  yaw rate. This is the path the bridge will use.
- **Earlier runs:** `s1_iris_02`, `s2_navrl_quad_01` and `_02` ran with `s1_offboard_mavlink.py`, with wall-clock
  time and a 1.5 m altitude.
- **Later runs:** `_03` to `_05` use PX4 sim time (`time_boot_ms`), 1.0 m (MOTAR's altitude target) and runtime
  parameter overrides (`param_overrides` in each JSON), and record the real-time factor (RTF).

| Run | Vehicle / change | RTF | x step 0→1: rise 10-90 (s) | overshoot | 0→1 final (m/s) | 1→2.5 final (m/s) | yaw 3 rad/s achieved | hover tilt RMS | diagonal 3.54 m/s achieved | host faults |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `s1_iris_02` | iris, PX4 defaults | n/r | 1.54 | 0.0 % | 0.96 | 2.42 | 2.91 | 1.3° | 3.36 m/s | 0/0 |
| `s2_navrl_quad_01` | navrl_quad draft (ODE decay 0.01/step) | n/r | — | 0.0 % | 0.56 | 1.54 | 2.59 | 3.8° | 2.01 m/s | 0/0 |
| `s2_navrl_quad_02` | + decay 4e-5/step | n/r | 0.88 | 10.7 % | 1.11 | 2.71 | 3.03 | 4.7° | 3.94 m/s | 0/0 |
| `s2_navrl_quad_03_rateA` | + rate gains scaled to inertia | 1.00 | 0.86 | 11.9 % | 1.11 | 2.71 | 3.03 | 4.0° | 3.89 m/s | 0/0 |
| `s2_navrl_quad_04_velB` | + MPC_XY_VEL P 2.5, I 0.1, D 0 | 1.00 | 0.38 | 6.9 % | 1.06 | 2.57 | 3.03 | 4.0° | 3.69 m/s | 0/0 |
| `s2_navrl_quad_05_thrC` | + THR_MDL_FAC 1 | 1.00 | 0.38 | 10.2 % | 1.08 | 2.57 | 3.00 | 2.8° | 3.52 m/s | 0/0 |

**Findings:**
1. **ODE damping semantics.** The draft SDF copied Isaac's damping 0.01. PhysX reads that as a rate (1/s); ODE
   applies it per 4 ms step, a drag with a time constant of about 0.4 s.
   - Measured: 1 m/s commanded gave 0.56 m/s after 3 s.
   - Corrected value: 0.01 × 0.004 = 4e-5 per step.
2. **Thrust model.** Gazebo's motor thrust goes as ω²; PX4 assumed a linear model (`THR_MDL_FAC` 0).
   - The mismatch held the vehicle at a constant 3.3° tilt in hover and sagged the climb (0.87 m for 1.0 m).
   - `THR_MDL_FAC` 1 removed both: hover mean roll and pitch 0.23° / −0.28°, climb 1.05 m.
3. **PX4's norm speed limit.** PX4 limits speed by the vector norm, while MOTAR limits each axis. The airframe sets
   `MPC_XY_VEL_MAX` 3.6 so that F's 3.54 m/s diagonal is kept.
4. **Default PX4 velocity loop.** Its 10-90 % rise is 1.4-1.7 s (iris). MOTAR's Lee loop is a pure first-order
   K_vel = 2.5 (τ 0.4 s, rise about 0.88 s), to be measured by G9.
   - With P 2.5, PX4 rises in 0.36-0.42 s with 5-10 % overshoot. Its velocity-setpoint feed-forward is a likely
     reason (not verified).
   - It is to be matched to G9 before S3b.
5. **Latency:** the MAVLink TIMESYNC round trip is about 0.1 ms locally, and the 10 Hz setpoint period holds
   (maximum 0.21 s once). The command-to-response onset is 0.10-0.24 s.

**Not yet:**
- G9, the Isaac reference;
- tuning the velocity loop to it;
- the hover altitude offset (+7 cm at 1.0 m; it is not known whether this is the frame definition or control);
- the bridge (S3a/S3b).

## `vehicle/`

`navrl_quad.sdf.jinja`, `model.config` and `10090_gazebo-classic_navrl_quad`, as installed after the fixes above;
`check_vehicle_spec.py` and `spec_check_01.json` (CPU spec check of 2026-11-04, PASS); `s2_install.out` (hashes and
changes in order).

## `s3a/`: does the bridge rebuild F's observation and action? (CPU, plan stage S3a)

- **Inputs:** `tools/sitl_s3a_equivalence.py` on the G8 r2 dump (`results/sitl_ref_v1_r2/g8/`), first 8 envs per cell,
  each env's first episode.
- **Run:** on `camonly/dev` `9882889`, which has the bridge fix. Records `s3a_70bars_9882889.json` (`35d2bebf…`) and
  `s3a_205bars_9882889.json` (`c371a864…`).
- **Gate** (plan S3a: every obs slice ≤ 1e-3, actions ≤ 1e-4, governor ≤ 1e-4): **NOT_EQUIVALENT** in both cells.
  Alignment mismatches 0.

| Cell | obs scan / obstacles / robot / target | action, bridge obs | action, Isaac's obs | governor |
| --- | --- | --- | --- | --- |
| 70 bars | 6.7e-6 / 2.3e-6 / 1.2e-7 / 6.9e-3 | 1.7e-3 | 1.6e-3 | 1.8e-7 |
| 205 bars | 2.3e-5 / 6.1e-6 / 1.2e-7 / 1.2e-2 | 9.8e-3 | 1.0e-3 | 5.5e-6 |

- **Bridge bug before the fix:** the bridge dropped F's lateral mu scale. Before `9882889` the lateral action error
  was 0.388 (WORKLOG 2026-11-05). This is fixed.
- **What remains:**
  - **Target slice:** the largest errors are the newest history slot's target relative velocity, x and z (obs indices
    885 and 887). The tracker's velocity estimate differs by up to 0.024 m/s, in early calls (call 4 at 205 bars,
    call 7 at 70 bars). At 205 bars it carries into the action (9.8e-3).
  - **Policy numerics:** the policy on Isaac's own observation differs by about 1e-3, consistent with TF32 in the
    Isaac process.
  - Neither cause is tested.

## `s3b_dev/`: Policy F flying PX4 SITL in closed loop (development runs, not preregistered)

- **Runner:** `tools/sitl_s3b_closed_loop.py`, with the MOTAR side in `tools/sitl_bridge.py`.
  - The loop runs at 10 Hz of PX4 sim time.
  - MOTAR's sensors are rendered against the episode's bars from the PX4/Gazebo truth pose.
  - The perception uses PX4's EKF pose.
  - F's governed velocity and yaw-rate commands go to PX4 offboard.
  - Termination uses Isaac's rules on the truth pose.
- **dev01-dev06 are confounded** and measure neither transfer nor F. Their reports are kept as recorded.
  - The bridge dropped F's lateral mu scale (S3a), so it ran a different controller.
  - PX4's EKF diverged under ODE quickstep iters 10 (`vehicle/2026-11-05_dev07/VIBRATION_BISECTION_2026-11-05.md`).
  - All of them used PX4's default GPS height, which drifts with the GPS model's vertical random walk.
- **dev07** (2026-11-05 10:03-10:18 KST, `camonly/dev` `9882889`; reports `dev07_70_s3b_report.json`,
  `dev07_205_s3b_report.json`; summary `dev07_summary.py` → `dev07_summary.json`). Setup:
  - the fixed bridge;
  - S3b worlds at ODE iters 100;
  - navrl_quad with a downward rangefinder as PX4's height reference (`vehicle/2026-11-05_dev07/`);
  - targets from the G8 r2 zero-action cells (60 s);
  - the first 10 envs of seed 4501 at 70 and 205 bars.

| Cell | Start checks valid | Isaac | SITL | Same outcome | Capture steps, SITL − Isaac (median, range) | RTF |
| --- | --- | --- | --- | --- | --- | --- |
| 70 bars | 10/10 | 10 capture | 10 capture | 10/10 | +2 (0 to +17) | 0.996-1.004 |
| 205 bars | 10/10 | 9 capture, 1 crash | 9 capture, 1 crash | 10/10 | +2 (−8 to +13) | 0.999-1.003 |

- The crash is env 8 at 205 bars, on a bar, at Isaac step 17 and SITL step 18.
- EKF − truth horizontal error: per-episode median at most 0.16 m, maximum 0.25 m.
- **This is a development result, not S3b acceptance.**
  - It ran alongside the GR-1b GPU evaluation (cores 6,7,12,13, nice 10). RTF was 1.0, and the metrics are in sim
    time.
  - It was not preregistered. n = 20 episodes, first episodes only, one seed.
  - The target replays its trajectory and does not react to the pursuer. The E2 target does not read the pursuer;
    see G8 r2 for the RNG coupling.
  - The sensors are MOTAR's own, rendered from truth, not Gazebo's.
  - PX4's velocity loop is not yet matched to MOTAR's (G9 env 1: rise 0.61 s; PX4 S2 tune: 0.36-0.42 s).
  - S3a is NOT_EQUIVALENT (a target-velocity slice difference).
  - The rangefinder is an addition to the training-spec vehicle.

## `tune_velloop/`: PX4 velocity loop against MOTAR's Lee loop (exploratory, 2026-11-05 10:18-10:39)

- **Method:**
  - `s12_offboard_mavlink_w.py`: the S1/S2 test script, plus a world selector.
  - navrl_quad with the rangefinder, worlds at ODE iters 100, the S3b runner's gains with overrides.
  - Cores 6,7,12,13 at nice 10, during the GR-1b GPU evaluation.
  - RTF 1.000 and host faults 0 in every run.
- **Reference:** G9 attempt 2, env 1, the only clean env. Summary in `tune_summary.json` (`summarize_tune.py`):
  velocity rise 0.62 s, onset 0.08 s, overshoot 0.9 %, settling 0.87 s, yaw rate rise 0.11 s.

| Overrides | Rise (median, range) | Onset | Overshoot | Settling | Yaw rise |
| --- | --- | --- | --- | --- | --- |
| P_ACC 2.5 (S3b dev07) | 0.38 s (0.34-0.42) | 0.18 s | 5.5 % | 1.57 s | 0.27 s |
| P_ACC 2.1 | 0.48 s (0.46-0.96) | 0.16 s | 3.3 % | 1.28 s | 0.27 s |
| P_ACC 2.1, YAWRATE P 0.2 / I 0.1 | 0.49 s (0.44-0.94) | 0.16 s | 3.4 % | 1.26 s | 0.20 s |
| P_ACC 2.0, YAWRATE P 0.2 / I 0.1 | 0.52 s (0.48-0.94) | 0.19 s | 3.1 % | 1.28 s | 0.20 s |
| P_ACC 1.8 | 0.77 s (0.62-0.97) | 0.18 s | 3.0 % | 1.32 s | 0.28 s |
| P_ACC 1.4 | 1.03 s | 0.22 s | 4.4 % | 1.90 s | 0.28 s |
| P_ACC 1.1 | 1.31 s | 0.23 s | 5.5 % | 2.30 s | 0.27 s |

- **Reading:** no single gain matches MOTAR's loop.
  - PX4 responds about 0.1 s later (onset 0.16-0.23 s against 0.08 s).
  - It settles more slowly (1.26-2.30 s against 0.87 s).
  - Its rise time is not proportional to the gain.
- **Candidate:** P_ACC about 2.0 with YAWRATE P 0.2 / I 0.1, the closest rise. Not yet applied to the S3b runner;
  `tools/sitl_s3b_closed_loop.py` still uses P_ACC 2.5. A dev08 with the candidate would show whether the outcome
  agreement is sensitive to the loop.
- The table numbers are rounded from `tune_summary.json`.
