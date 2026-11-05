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
