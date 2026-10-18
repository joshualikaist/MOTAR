# M2 — Action → motion → termination contract for an arbitration planner over frozen Policy F

Read-only audit of `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1` (worktree, branch `codex/mt-p1`, HEAD `1d4696c`), 2026-10-19.
Nothing was run except greps and file reads. No simulator, GPU or isaacgym import.

Abbreviations:
- `NT` = `aerial_gym/task/navrl_task/navrl_task.py`
- `TCFG` = `aerial_gym/config/task_config/navrl_task_config.py`
- `SG` = `aerial_gym/task/navrl_task/speed_governor.py`
- `TM` = `aerial_gym/task/navrl_task/target_motion.py`
- `EM` = `aerial_gym/env_manager/env_manager.py`
- `ENV` = `aerial_gym/config/env_config/navrl_bars_env.py`
- `VC` = `aerial_gym/control/controllers/velocity_control.py`
- `BLC` = `aerial_gym/control/controllers/base_lee_controller.py`
- `MM` = `aerial_gym/control/motor_model.py`
- `BQ` = `aerial_gym/config/robot_config/base_quad_config.py`
- `BMR` = `aerial_gym/robots/base_multirotor.py`
- `EV/` = `docs/evidence/`

Evidence folders:
- `EQ1D` = `EV/eq1d_qualification_2026-10-18/REPORT.md`
- `EQ1` = `EV/eq1_validation_2026-10-18/REPORT.md`
- `EQ0` = `EV/eq0_environment_audit_2026-10-17/`
- `ISO` = `EV/h_r3_env_isolation_2026-10-18/`
- `P02` = `EV/h_r3_p02_2026-10-18/`
- `P03` = `EV/h_r3_p03_preparation_2026-10-19/REPORT.md`

## 0. Canonical Policy F runtime (the values that actually apply)

The code defaults in `TCFG` are **not** the Policy F contract. The values below come from the EQ-1D receipt environment. That receipt is named at `docs/plans/eq1d_qualification_v1.json` → `frozen.receipt_environment`, which points to `../aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2/static/A1/E0_static__115bars__seed4301/receipt.json`, key `env_snapshot_navrl`.

- **Checkpoint:** Policy F = `runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth`, SHA `f7022139…`.
- **Perception and actor:** `NAVRL_VISION=1`, `NAVRL_PERCEPTION=1`, 898-D actor.
- **Velocity and yaw:**
  - `NAVRL_MAX_VELOCITY=2.5`, applied **per axis**.
  - `NAVRL_YAW_RATE_MAX=3.0`.
  - `NAVRL_ALT_HOLD_VMAX=2.5`.
  - `NAVRL_MAX_TILT_DEG=45`.
  - `NAVRL_TILT_COMP=1`.
- **LiDAR:** `NAVRL_LIDAR_HBEAMS=72`, `VBEAMS=4`, `RANGE=12`.
- **Speed governor:** `NAVRL_SPEED_GOVERNOR=riskcap`, with:
  - `FIXED=2.0`
  - `FREE=3.53553390593`
  - `SLOW=3.0`
  - `RELEASE=5.0`
  - `HALF_WIDTH=0.45`
  - `MARGIN=0.45`
  - `TTC=1.2`
  - `BRAKE=2.96`
  - `REACTION=0.1`
  - The last four only feed telemetry under riskcap.
- **Arena and obstacles:**
  - `NAVRL_ARENA_XY=40`, `ARENA_Z=3`.
  - `NAVRL_BAR_POOL=bars_h3`.
  - `NAVRL_PLACEMENT_MODE=navrl_band`, `TOUCH=0.4`, `GAP=1.6`.
  - `NAVRL_MAX_BARS=300`, `NAVRL_NUM_BARS ∈ {70,115,160,205}`.
  - `NAVRL_OOB_MARGIN=1.0`.
- **Episode and spawn:**
  - `NAVRL_EPISODE_LEN_STEPS=600`.
  - `NAVRL_GENERAL_TRAIN=1`, so spawn and yaw are random.
  - `NAVRL_KROC2027_INITIAL_DISTANCE_M=10`.
- **Robot:** `NAVRL_ROBOT` is unset, so the robot is `navrl_quad` (0.25 kg).
- **Sim:** `sim_name=base_sim`.
- **Target arms:** defined in `aerial_gym/rl_training/rl_games/eval_navrl_target_behavior_arms.sh:113-140`.
  - The pinned ramp of 300 epochs gives v_max = 1.5.
  - KROC2027 moving cells use E2 + `direct_l2_v1` (`docs/plans/kroc2027_A_amendment2_v1.json:22-25`, `tools/kroc2027_eval.py:1124,1260`).
- **Densities in the matched lineage:** 70/115/160/205 (`docs/plans/kroc2027_protocol_v1.json:105`).
- **Critical keys:** `tools/audits/check_eval_condition_contract.py:30-59` lists the keys whose change invalidates a frozen-policy comparison. They include max_velocity, yaw_rate_max, LiDAR geometry, tilt, arena, dt, physics steps, placement and the governor.

## 1. Timing (verified from config and code)

| Item | Value | Source |
|---|---|---|
| Physics dt | 0.01 s | `aerial_gym/config/sim_config/base_sim_config.py:28` |
| PhysX substeps | 1 | `base_sim_config.py:29` |
| Gravity | [0, 0, −9.81], z-up (`up_axis=1`) | `base_sim_config.py:30-31` |
| TGS solver | 4 position iterations / 1 velocity iteration | `base_sim_config.py:36-38` |
| Contact offsets | 0.002 / 0.001 m | `base_sim_config.py:39-40` |
| `contact_collection` | 1 (last substep) | `base_sim_config.py:45` |
| Physics steps per RL step | 10 (`num_physics_steps_per_env_step_mean=10`, std 0) | `ENV:54-55`, loop at `EM:458-473` |

**RL step and policy decision interval.**
- `step_dt = obs_dict["dt"] × n_phys = 0.1 s` (`NT:1359-1368`). The interval is therefore 0.1 s (10 Hz).
- The step count comes from `math.floor(random.gauss(10, 0))` (`EM:458-466`). This consumes Python's `random` stream each step.

**Controller rate: 100 Hz.**
- Each physics step calls `robot_manager.pre_physics_step` → `robot.step` → `call_controller` (`EM:358-361`, `aerial_gym/robots/robot_manager.py:486-489`, `BMR:246-258, 296-305`).
- The command tensor is a zero-order hold for the whole 0.1 s action. EQ-1D D1-T1 PASS confirms this (`EQ1D:39`).

**Altitude PI and governor rate: 10 Hz.**
- Both are computed once per action in `transform_action_to_command`, from the state at the start of the step (`NT:4953-5009`).

**Sensors and render timing.**
- LiDAR and depth are raycast in `post_reward_calculation_step`, after physics and after the env-manager reset (`EM:438-442`).
- After the task's `reset_idx`, LiDAR is rendered a second time (`NT:5168-5175`).
- The camera RGB-D is rendered inside `get_return_tuple` → `_process_obs_perception` (`NT:5182`, `NT:6450-6481`).
- The observation is therefore the end-of-step (post-reset) state, with 0-step latency (EQ-1D D3-T1, `EQ1D:80`).
- The governor at step k uses the LiDAR scan already in the observation at decision time: `obs_dict["depth_range_pixels"]` (`NT:5260-5265`).

## 2. Action interface (`transform_action_to_command`, `NT:4953-5009`)

**Planar velocity.**
- In vision/perception mode (Policy F), `vel_vehicle = clamp(actions[:,0:3], −1, 1) × max_velocity` (`NT:4960-4961`).
- The frame is the **vehicle frame**: yaw-only and level, `vehicle_frame_quat_from_quat` (`aerial_gym/utils/math.py:176-180`).
- The limit is per axis (±2.5 m/s), so the horizontal norm can reach 3.5355 m/s (`TCFG:466-472`).
- The non-vision goal-frame branch (`NT:4963-4965`) is not used by Policy F.

**The command is re-rotated to world at every 10 ms substep** with the *current* yaw: `quat_rotate(robot_vehicle_orientation, setpoint_velocity)` (`BLC:116-118`).
- During a 3 rad/s yaw the world-frame command turns by up to 0.3 rad (17°) within one action.

**Governor.** Only XY are passed through `_apply_sensor_speed_governor` (`NT:4967`, see §3).

**Executed-action feedback.**
- `_last_governed_action_xy` = governed xy / max_velocity (`NT:4968-4970`).
- It is written into `prev_action[:,0:2]`, which Policy F sees in its next observation (`NT:5051-5057`).
- `prev_action[:,2:4]` holds the raw clamped z and yaw requests.

**Z is overridden by an altitude PI** (`NT:4979-5002`):
- `vz = clamp(4·(1.0 − z) + 1.0·∫(1.0 − z)dt, ±2.5)`.
- The integral is clamped to ±2.5 and reset per episode (`NT:4891`).
- The z action has no actuator authority. It survives only as an observation channel in prev_action (`TCFG:459-463`).

**Yaw.**
- `command[:,3] = clamp(a3, ±1) × yaw_rate_max(3.0) × _governor_yaw_scale` (`NT:5006-5008`). The scale is 1.0 because the L5 yaw cap is off by default (`SG:71, 83-84`).
- The controller interprets this as an Euler yaw rate. It converts it to body rates (`VC:87-91`, `BLC:191-206`) and clamps it to `max_yaw_rate` = 3.0 (`BLC:128-130`, `aerial_gym/config/controller_config/lee_controller_config_navrl.py:7,23`).

**Final clip.** A last ±10 clip applies to the command tensor (`BMR:207-211`). It has no effect at these magnitudes.

**Existing arbitration insertion points.**
1. Task-level: `actions = self._kroc_search.apply(actions)` (`NT:5017-5019`). This runs before target advance and before the governor (`aerial_gym/task/navrl_task/kroc_eval_adapter.py:686-717`).
2. Player-loop replacement, as in `tools/h_r3_launcher.py:91-104`: `torch.where(active, scripted, policy_action)` before `env_step`.

In both cases the substituted action passes through the governor and the altitude PI, and it becomes Policy F's `prev_action`.

## 3. Speed governor / riskcap (`NT:5248-5395`, law in `SG:438-531`)

**Input.**
- The current LiDAR `depth_range_pixels` (normalised, `nan`→1) × `lidar_max_range` (12 m), shape [N, 4, 72] (`NT:5260-5265`).
- The sensor-associated target mask `perception.last_target_like` removes target returns (`NT:5285-5292`).
- Ground-truth geometry is not used (`SG:1-6`).

**Bearings.** `lidar_bin_bearings` (`NT:5267-5282`).

**Clearance under riskcap.** `directional_lidar_clearance` (`SG:189-255`; called at `NT:5332`):
- Valid rays satisfy `0 ≤ r < 0.995·Rmax` and are not target returns.
- Each ray is projected to horizontal: `r·cos(elev)`, with elevations linspace(+20°, −10°, 4).
- The per-bearing minimum over the vertical rows is taken.
- A ray counts as in path if its forward component is > 0 and its lateral component ≤ 0.45 m (a straight corridor along the commanded direction).
- Clearance = minimum forward distance among in-path rays, else Rmax. For |cmd| ≤ 1e-6, clearance = Rmax.
- `riskcap_arc` uses `arc_clearance` with the measured body yaw rate instead (`SG:377-435`, `NT:5317-5330`). It is not the Policy F default.

**Riskcap law** (`SG:465-475`):

```
cap = FIXED + clamp((c − SLOW)/(RELEASE − SLOW), 0, 1)·(FREE − FIXED)
    = 2.0 + clamp((c − 3)/2, 0, 1)·1.5355   [m/s]
```

- It is applied to the norm, not the direction: `v_exec = v_req · min(|v_req|, cap)/|v_req|` (`SG:497-503`).
- **The floor is 2.0 m/s and the law never brakes below it.** At 0.1 m clearance it still executes 2.0 m/s (`EQ1:86`; `EQ0/ISSUE_REGISTER.md:159-166`).
- A request below the cap is untouched.

**Parameter roles.**
- `hard_margin`, `ttc`, `brake`, `reaction` enter only the telemetry fields `stopping_margin_*` and `ttc_requested_s` (`SG:505-526`).
- `stopcap` / `dwa_arc` / `omni` use `v = sqrt((b·t_r)² + 2·b·usable) − b·t_r` (`SG:476-488`). This is the law a safety-respecting planner should emulate, not riskcap.

**Other properties.**
- It is a deterministic function of (scan, command), so a planner can replicate it exactly, CPU-testable.
- The LiDAR `min_range` is 0.2 m. Returns closer than that map to `near_out_of_range_value=−max_range`, which is clamped to 0 → clearance 0 (`aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py:44-48`; `NT:5262-5264`). The cap is still 2.0 m/s.

## 4. Low-level controller (`lee_velocity_control_navrl` → `LeeVelocityController`)

**Registration.** `aerial_gym/control/__init__.py:54`, with `controller_name` at `TCFG:124`.

**Gains.** `randomize_params=False`, so each gain is the mean of min and max (`BLC:59-62`; `aerial_gym/config/controller_config/lee_controller_config.py:21-45`):

| Gain | Value | Note |
|---|---|---|
| K_v | [2.5, 2.5, 2.5] | |
| K_R | [1.0, 1.0, 0.5] | |
| K_Ω | [0.15, 0.15, 0.15] | |
| K_x | [2.5, 2.5, 1.5] | Multiplies a zero error, because `setpoint_position = robot_position` (`VC:28-31`). There is no position feedback |

**Translational law** (`BLC:111-125`, `VC:28-32`):

```
a_cmd = K_v ∘ (R_ψ v_cmd^veh − v_w)
f = m (a_cmd − g),  g = [0, 0, −9.81]
```

- So f = m(a_cmd + 9.81·e3) with **e3 up** (z-up world, thrust along +b3).
- Lee et al. 2010 instead use e3 pointing down, `f = (−k_x e_x − k_v e_v + m g e3 + m ẍ_d)·R e3` with thrust along −b3. The sign convention is flipped, the structure is otherwise the same.
- There is no ẍ_d feedforward and no integral term.

**Tilt limit** (`VC:34-53`):
- `f_z ← max(f_z, 1e-6)`. The horizontal component is scaled so that |f_xy| ≤ f_z·tan 45°.
- This is altitude-priority, giving lateral accel ≤ ≈ g when f_z ≈ mg.
- The 45° bound is on the *commanded* attitude only. Measured actual tilt exceeds it; see §5.

**Thrust** (`VC:56-80`; config `lee_controller_config_navrl.py:32-37`):
- With tilt compensation on: `T = f_z / b3_z` when `b3_z > 0.5` (within 60° of upright).
- Otherwise the standard projection `T = f·b3`.

**Attitude.**
- `b3_d = f/|f|`. `b1_d` comes from the *current* yaw (`BLC:164-185`; `VC:83-85`). There is no yaw-angle setpoint, so yaw is rate-only and not held. The open-loop drift in §5 follows from this.
- Torque (`BLC:127-145`): `τ = −K_R e_R − K_Ω (ω − R^T R_d ω_d) + ω × Jω`, with `e_R = ½ vee(R_dᵀR − RᵀR_d)`.
- Torques are in N·m. They are **not** inertia-scaled, J = diag(4.2e-4, 4.2e-4, 8.45e-4) (`resources/robots/quad/quad_navrl_collide.urdf:21-24`). Large attitude errors therefore saturate the motors.

**Allocation** (`aerial_gym/control/control_allocation.py:52-93`, `BQ:159-173`):
- Thrusts = pinv(A)·[0, 0, T, τx, τy, τz]. Motors are at ±0.13 m (lever arm in A), yaw coefficient 0.01.
- Allocation is unaware of actuator limits ("does not account for actuator limits", `control_allocation.py:48-50`). Saturation happens per motor in the motor model, with no thrust/torque priority.

**Motor model** (`MM:91-160`, `BQ:175-194`):
- Per-motor thrust is clamped to [0, 2] N. Total 8 N at 0.25 kg gives T/W ≈ 3.26.
- The model is first order in RPM with τ_up = τ_down = 0.04 s and discrete mixing 1/(dt+τ). The default scheme is RK4 (`MM:17`).
- The effective per-step gain is 0.2214, within 0.096 % of 1 − exp(−dt/τ) (`EQ1D:41`).
- The thrust constant is randomised per env at reset over U[9.26e-6, 1.83e-5] (`MM:83-85, 155-157`). This changes the RPM↔thrust map, not the steady state.
- **At every reset the initial motor thrust is drawn U[0, 2] N per motor** (`MM:151-153`). This gives a random thrust transient in the first steps of each episode.

**Disturbance and drag.** Both are off: `enable_disturbance=False` and zero damping coefficients (`BQ:71-80`). PhysX linear/angular damping is 0.01 (`BQ:94-95`).

**Vehicle.**
- Mass 0.225 + 4×0.00625 = 0.25 kg (`quad_navrl_collide.urdf:22,57,...`).
- Collision box 0.28 × 0.28 × 0.08 m (`urdf:18`; `aerial_gym/config/robot_config/navrl_quad_config.py:21-27`).

## 5. Measured dynamics facts (EQ-1D, EQ-1, EQ-0)

**Velocity tracking.**
- Single-axis steps reach steady state within 0.4 % (1.0 → 0.99594, 2.5 → 2.49002 m/s).
- A diagonal 3.5355 m/s command reaches the right speed but **18° off** in direction (31 % relative vector error), because the vehicle yaws −17.8° with zero yaw command. D1-V1 FAIL (`EQ1D:44`).

**Open-loop yaw drift** under lateral motion: −17.8° (diagonal held), −32.8° (diagonal brake), −131° (saturated diagonal), −11° (30° corner fixture). The vehicle-frame command rotates with the body (`EQ1D:47, 109-111`).

**Braking.** D1-B1 PASS (`EQ1D:45`):

| From | Stopping distance |
|---|---|
| 2.0 m/s | 0.82–0.83 m |
| 2.5 m/s | 1.05–1.06 m |
| 3.52 m/s | 1.56–1.58 m |

- Mean deceleration is 2.1 / 2.5 / 3.4 m/s², and stopping takes 0.94–1.03 s.
- This is consistent (my derivation, not a separate measurement) with first-order velocity decay of time constant 1/K_v = 0.4 s: v₀·τ = 0.80 / 1.00 / 1.41 m. The excess above that is attributable to attitude and motor lag.

**Rise time, overshoot and velocity-loop bandwidth:** NOT_RECORDED. Only steady-state means and braking were measured.

**dt dependence.** Error is first order, about 2.08× per halving of dt. The 5-vs-2.5 ms position difference is 0.0098–0.0265 m and attitude up to 0.64°, so D1-C1 FAILs 2/20 (`EQ1D:42, 112-114`). A rollout model must use the same 10 ms plant to be consistent.

**Hover and altitude.**
- Hover thrust is 2.452504 N vs m·g = 2.4525 N (`EQ1D:43`).
- Altitude drift with vz = 0 is −0.039 m per 6 s at 10 ms (`EQ1D:113`).
- Recorded flight altitudes in OCC runs: 0.1014–2.7928 m (moving) (`EQ0/EQ0_ENVIRONMENT_AUDIT.md:68-71`). The extremes are near the termination band.

**Tilt above 45°.**
- With the diagonal saturated the vehicle holds exactly 45.0°.
- After oblique contact it reaches 90.8° at 10 ms. Post-contact motion is not converged in dt (`EQ1D:46, 51`).
- OCC records: 951 normal-flight frames exceed 45° (0.041 %, max 91°), and 1.48 % of last-1 s-before-crash frames do (max 111°) (`EQ1:70-79`).
- Horizontal speed reaches 4.47–4.79 m/s, above the 3.5355 command cap (`EQ1:70-79`; `EQ0_ENVIRONMENT_AUDIT.md:68-69`).

**Latency.**
- Configured sensor latency is 0, and camera, LiDAR and observation share the step counter (`EQ1D:80`; `EQ0/ISSUE_REGISTER.md:82`).
- True capture timestamps are NOT_RECORDED.
- Command→motion lag consists of the 10 Hz ZOH, τ_v ≈ 0.4 s (derived), τ_motor 0.04 s and an unmeasured attitude loop.

**Determinism.** The simulator is bitwise run-to-run deterministic, and snapshot/restore branching works. TF32 is on in the player process (`EQ1D:29-31, 82, 147-148`).

**Domain verdicts:** dynamics LIMITED_VALIDITY, sensors LIMITED_VALIDITY, firewall PASS, determinism PASS (`EQ1D:152-157`).

**`LINEAGE_DIFFS.md`.**
- It lists only opt-in, off-by-default proposals. None are active for Policy F.
  - P-L1: ZOH motor (`EV/eq1d_qualification_2026-10-18/LINEAGE_DIFFS.md:42-53`).
  - G-L1: stopcap plus a ≤ 0.09 s contact delay (`LINEAGE_DIFFS.md:55-63`).
  - S-L1: changes 768 of 898 actor inputs (`LINEAGE_DIFFS.md:6`).
- It contains no geometry or density differences (`LINEAGE_DIFFS.md:3-8`).

## 6. Collision and contact

**Physical contact, not a distance threshold.**
- `collision_tensor += |robot_contact_force| > 0.05 N`, accumulated over the 10 substeps (`EM:403-407`; threshold at `ENV:58-59`).
- With `reset_on_collision=True`, the task reads it as `obs_dict["crashes"]` (`NT:6071`).
- There is no CCD, no OBB/semantic crash rule and no geometric certificate (`EQ0/supporting/g1/findings.md:68`).

**Geometry.**
- Body: 0.28 × 0.28 × 0.08 m box, XY circumradius 0.19799 m, inscribed (at 45°) 0.099 m. Rotors are not modelled.
- Bars: boxes with footprints 0.403–0.790 m, 3 m tall, visual = collision, identity yaw (`EQ1` scene records; `findings.md:66,76`).
- There is no ground plane and no walls (`ENV:60`). The floor exists only as the z < 0.1 m termination.

**Timing.**
- The first contact force appears one substep after SAT separation ≤ 0.004 m.
- **The crash is registered 0.05–0.09 s after first contact**, because it is only evaluated at the end of the action (`EQ1D:47-48`).
- A head-on contact stops the vehicle within one substep. A 60° oblique contact keeps 1.3–1.7 m/s and can flip the vehicle (`EQ1D:48-50`).

**Crash rule** (`compute_state_reward_and_terminations`, `NT:6019-6304`):

```
crashed_out = contact | z < 0.1 | z > 4.0 | OOB
```

- Height bounds: `TCFG:731-732`.
- OOB: XY outside [0, 40]² ± 1.0 m (`NT:6098-6107`).
- Physical-target-only causes (`NT:6074-6097`) are inactive for bounded or legacy targets.
- Cause priority is contact > below > above > OOB (`NT:6129-6138`).

## 7. Capture, timeout, reset and terminal observation

**Capture** (`NT:6108-6125`):
- `rel = p_robot − p_target`, `seg = rel − prev_rel`.
- The test is the minimum distance of the linear segment prev_rel→rel to the origin, < `success_radius` = 0.5 m (`TCFG:727`). This is a swept test of both agents linearised over 0.1 s.
- **Capture wins over a same-step crash**: `crashed_out &= ~captured` (`NT:6124`). Collision-free capture needs E-L0 labels (`EQ1D:141-143`).
- The primary metric is `capture_0p5m` (`docs/DECISIONS.md:106-109`).
- Capture is centre-to-centre in 3-D. Target z is fixed at 1.0 m and pursuer z is held near 1.0 m.

**Timeout.**
- `truncations = sim_steps ≥ episode_len_steps` (`NT:315-322, 5065-5073`).
- Episodes are 600 actions = 60 s. `sim_steps` increments per RL step (`EM:474`) and is reset per env (`EM:331`).

**Order inside `step()`** (`NT:5011-5182`):
1. Action diagnostics.
2. KROC supervisor `apply`.
3. Target advance (`NT:5025`).
4. `transform_action_to_command` (governor + altitude PI) (`NT:5028`).
5. `prev_action` update (`NT:5051-5057`).
6. `sim_env.step` (10 × physics + control) (`NT:5058`).
7. `compute_state_reward_and_terminations` (`NT:5063`). This writes `terminations = crashed_out | captured` into the *same tensor* as `obs_dict["crashes"]` (`NT:1577, 6304`), so captures also trigger the env-manager reset.
8. Truncation.
9. Ledgers and curriculum.
10. `_sync_target_to_sensor` (`NT:5165`).
11. `post_reward_calculation_step`: the env manager resets every env with terminations or truncations (`EM:409-420`), then renders LiDAR (`NT:5168`).
12. Task `reset_idx(reset_envs)` (`NT:5170`), then target sync and LiDAR render again (`NT:5174-5175`).
13. `add_static_safety_reward`, which masks finished envs (`NT:5179`, `NT:6319-6325`).
14. Camera render, perception and observation (`NT:5182`).

**Terminal-observation hazard.**
- At a terminal or truncated step, the returned observation and every `obs_dict` state (robot pose, `target_position`, `tracker`, `_camera_visible_now`) **already belong to the next episode's spawn**.
- H-R3 pilot miscounted a terminal-index detection as reobservation (`EV/h_r3_pilot_2026-10-18/REVIEW.md:94`).
- P02/P03 use terminal-first scoring and exclude the terminal index (`P02/REPORT.md:46`; `P03/FABLE_REVIEW.md:17`).
- A planner must read `terminations`/`truncations` (or wrap `reset_idx` as `tools/h_r3_launcher.py:73-77` does) before trusting state.

**Reset content** (`NT:4701-4947`):
- General spawn draws a random XY ≥ 1 m inside the bounds, with footprint-aware clearance from every solid obstacle (`NT:2353-2424`).
- z = 1.0, yaw ~ U(−π, π), and velocities = 0.
- The target is placed by `sample_initial_view_for_task` at 10 m (`NT:4742-4752`).
- The altitude integral, `prev_pos`, `prev_rel` and `prev_action` are zeroed or re-seeded (`NT:4890-4901`).
- The motor-thrust random draw from §4 also applies here.

## 8. Target motion (Policy F evaluation arms)

`eval_navrl_target_behavior_arms.sh:113-140` defines the arms:

| Arm | Behavior level | Dynamics | Pattern | Speed |
|---|---|---|---|---|
| H | historical | legacy | mixed | U[0.3, 1.5] |
| E0 | e0_static | bounded | cv | 0 |
| E1 | e1_cv | bounded | cv | U[0.3, 1.5] |
| E2 | e2_obstacle_aware | bounded | waypoint | U[0.3, 1.5] |

- Speed is drawn per episode with `torch.rand` (`NT:7462-7472`).
- The behaviour contract is in `TM:187-230`. e3_reactive and e4 raise NotImplementedError (`TM:203-206`).
- Policy F was *trained* on the legacy model `symmetric_local_steer_v2_heading_continuity90`, mixed pattern, 0.3–1.5 m/s.

**Bounded model** (E0/E1/E2; `NT:7891-8231`, `TM:233-508`), dt = 0.1 s:
- Desired velocity:
  - cv: the current heading × speed.
  - waypoint: unit vector to the waypoint × speed. The waypoint is resampled within 0.5 m. Waypoints are drawn uniformly in [b_min + 0.5, b_max − 0.5] and **not screened against bars** (`NT:7547-7555`).
- `limit_planar_velocity` (`TM:233-293`):
  - heading slew ≤ 150°/s × dt when speed > 0.1 m/s
  - |Δv| ≤ 4 m/s² × dt
  - speed clip
  - position update p += v·dt
- `bounded_drone_target_step` (`TM:296-508`) builds 24 headings × {1, 0.5, 0.25} speed scales + stop = 73 candidates. Each is rolled out 10 steps (1 s).
- A candidate is safe if it stays in bounds and keeps bar *centre* distance ≥ 0.77 m (`TM:414-426`).
- Score: `1000 + 10·scale − |angle|` if fully safe, else the longest safe prefix. The first step of the argmax is executed.
- An infeasible first step is still written (`NT:8212-8220`). There is no reflection or push-out.
- Target z = 1.0 m (`NT:8215`).
- `target_vel_w` = the planner velocity. It feeds reward and the critic only, never the actor (`NT:6029, 6520-6540`).

**Legacy model** (H; `NT:8233-8407`, `TM:704-801`):
- cv reflects specularly at the walls. Waypoints and circles move toward their goal.
- Local steering uses 10 headings and 1-step lookahead, with centre distance ≥ 1.0 m.
- Push-out runs up to 6 iterations. A cv bounce adds ±10° random jitter (`NT:8389`).

**The target never reacts to the pursuer.**
- `robot_position` is used only for the optional initial CV heading, which is `random` in the canonical launcher (`NT:7492-7504`; `TM:666-701`).
- The target is also spawned at a set distance from the pursuer (`NT:2725-2733`).

**The target is a virtual point**, not collidable, for legacy and bounded dynamics (`NT:845-847`). It is injected analytically into LiDAR (radius 0.20 m, box half-extents 0.14/0.14/0.06) and the camera through `_sync_target_to_sensor` (`NT:7557-7563`; `navrl_lidar_config.py:54-61`).

**TF32 hazard.**
- Under `legacy_cdist`, the batched `torch.cdist` at `TM:423` can be off by about 2 m at bar counts divisible by 4 while rl_games has TF32 enabled. Historical E1/E2 cells at 160 bars are exposed.
- `direct_l2_v1` (`TM:424-425`) is the KROC2027 fix.
- `torch.cdist` is also used for goal and spawn clearance (`NT:2745, 4813, 4827`) and in legacy steering (`TM:763`).

**Randomness.**
- All target and reset draws use the global torch CUDA generator, seeded by `NAVRL_SEED` through `BaseTask.seed` (`aerial_gym/task/base_task.py:28-37`). There is no dedicated generator.
- Draw counts depend on which envs reset or reach waypoints, so RNG consumption is **coupled across environments**. The protocol declares arms an UNPAIRED_STREAM (`docs/plans/kroc2027_protocol_v1.json:30`).

## 9. Reset isolation and cross-env coupling

**P02 integrity failure** (`P02/README.md:3-4`, `P02/main_failure_audit.md:6-23`).
- The all-arm common-prefix check failed: 246 triggers, of which 14 had failed primary prefixes.
- The first mismatches coincided with a reset in *another* env that occurred in only one arm.
- Mechanism: `aerial_gym/env_manager/warp_env_manager.py:40-54` `WarpEnv.reset_idx`.
  - Lines 44-48 rewrite **every** env's vertex buffer from the current actor roots.
  - Lines 52-53 refit only the reset envs. The call comes from `EM:327` on every reset.
  - Static actor roots drift by up to 1.5e-5 m after physics (`ISO/REVIEW_RESPONSE.md:12-14`).
  - So a reset in one env perturbs RGB, depth and LiDAR in the others.
- An earlier SMOKE01 failure came from the full PhysX root setter. It was fixed by an indexed/batched reset repair (`P02/smoke01_failed/ASSESSMENT.md:7-11`).
- The cause of the 14 historical failures is **not retrospectively proven** (`ISO/REPORT.md:9`).

**ISO-Q02.**
- `tools/h_r3_env_isolation.py::WarpResetIsolation.reset_idx` writes only the selected slices and refits only those meshes.
- In the N128 forced-reset test, legacy mode changed 154,832 vertex rows and the sensors of 126 envs. Selected mode was exact for all 41 frames.
- Verdict: **GO — Mechanics Qualified**, only for seed4602 / N128 / moving160 / 40 steps (`ISO/REPORT.md:3, 9-30`).

**P03.**
- `tools/h_r3_p03_isolation.py` (RuntimeIsolation with on-device whole-vertex guards) passes exactly for the 600-step normal/forced pair, evaluator ON/OFF, and the B0–B5 common prefix.
- Verdict: **GO_QUALIFICATION_ONLY** for seed4801 / N128 / moving160 (E2 direct_l2_v1) / 600 steps (`P03:5, 22, 41-58, 79`).

**What is guaranteed.** Under these wrappers and only these profiles, a reset in env j does not change env i's sensors or state, and arm prefixes match bit for bit.

**What is not guaranteed.**
- Other seeds, densities, dynamic meshes, noisy sensors and long-run determinism (`ISO/REPORT.md:34`; `P03:229-230`).
- Shared global RNG consumption across envs: probes only showed equal episode counters (`P02/REVIEW_RESPONSE.md:17-19`).
- Root-translation drift: the PhysX stage responsible is not localised.
- Root writes at `NT:4845/4849/4936` are inside `reset_idx` and remain unaudited outside the transaction (`P02/REVIEW_RESPONSE.md:28-31`).
- **The production `reset_idx` still has the global rewrite.** Any matched-arm planner study needs the selected-only wrapper.
- Nominal command-budget parity ≠ realized-motion parity: an imbalance of about 20–31 % (`P03:182-189`).
- H-R3 itself stays PROVISIONAL / CONFIRMATION HOLD (`P03:6-8`).

## 10. Obstacle geometry

**Bars.**
- Pool `bars_h3` gives 3 m-tall boxes with the centre at z = 1.5 m (`aerial_gym/config/asset_config/env_object_config.py:810-846`).
- Placement covers the whole arena: x/y ratios 0..1 via `NAVRL_BAR_X_MIN=0`, `MAX=1`. Bars are upright and do not collide with each other (`env_object_config.py:823`).
- Up to `NAVRL_MAX_BARS=300` are built. Bars beyond `NAVRL_NUM_BARS` are parked at −1000 (`aerial_gym/env_manager/asset_manager.py:120-121`; `NT:2157-2170`).
- **Layouts are re-drawn at every reset** with the global RNG.

**Placement for Policy F: `navrl_band`** (`aerial_gym/env_manager/asset_manager.py:346-420`; `ENV:79-92`).
- A candidate is accepted only if every placed bar is either at centre distance ≤ 0.4 m (merge into a compound wall) or ≥ 1.6 m away. On saturation the candidate snaps onto an existing bar.
- **Overlap is intentional**: 102 of 112 sampled cells contain touching or overlapping pairs.
- Diagonal corner gaps can be 0.469 m. The minimum separate-pair gap is 0.592 m (`EQ0/ISSUE_REGISTER.md:178-183`). With a 0.396 m body diagonal, either gap is barely passable.

**Non-overlapping lineage.**
- It exists in code: `footprint_clearance` / `nonoverlap` / `surface_clearance` modes with a 0.45 m circumcircle surface clearance (`asset_manager.py:133-178`; `ENV:93-98`).
- It belongs to the physical lineage and is **not** the Policy F contract. `cfg_placement_*` are critical keys.

**Passability.** All 57,344 OCC-P2 layouts are kinematically connected at body radius. Four are UNRESOLVED with the 0.45 m reserve. Dynamic feasibility is NOT_TESTED (`EQ1:85`).

**Densities in the matched lineage:** 70 / 115 / 160 / 205. Arena 40 × 40 × 3 m, OOB at ±1 m.

## 11. Ego state available online

- Perception receives `drone_pos_w = obs_dict["robot_position"]`, `drone_vel_w = robot_linvel`, `vehicle_quat`, the body yaw rate and `prev_action` directly from simulator state (`NT:6457-6480`).
- The actor sees ego history from these values (5 × 10 = 50 dims per `EV/eq1_validation_2026-10-18/SENSOR_CANDIDATES.md:13`).
- **The project treats ego state as ideal onboard odometry.**
  - H-R3 online controllers may use own pose, vehicle quaternion, tracker belief and LiDAR. GT target and semantics are evaluator-only (`P03:234-235`; `P03/FABLE_REVIEW.md:16`; `AGENTS.md` observation contract).
- Pose noise and clock offset exist but default to 0 (`TCFG:337-342`; `aerial_gym/task/navrl_task/navrl_perception.py:838-847`).
- `NAVRL_PERCEPTION_PERTURB` is off, so the detection-dropout / RGB / depth noise settings are inactive (`EQ0_ENVIRONMENT_AUDIT.md:75-77`).
- No pose or velocity noise model has been validated.

**Allowed planner inputs:**
- `robot_position`, `robot_vehicle_orientation`, `robot_linvel`/`robot_vehicle_linvel`, `robot_body_angvel`
- the LiDAR `depth_range_pixels`
- the tracker belief (`perception.tracker.state`) and the camera/fused visibility flags

**Forbidden planner inputs** (evaluator-only):
- `target_position`, `target_vel_w`
- `segmentation_pixels`
- obstacle positions from `obs_dict`

## Summary table

| Stage | Input | Output | file:line | Validated by | Known error |
|---|---|---|---|---|---|
| Arbitration hook | Policy F action a∈[−1,1]⁴ | substituted action | `NT:5017-5019`; `tools/h_r3_launcher.py:91-104` | P03 common prefix B0–B5 (`P03:41-58`) | becomes Policy F's `prev_action` (OOD feedback) |
| Target advance | target state, global RNG | new target p, v (dt 0.1) | `NT:5025, 7705-8407`; `TM:296-508` | E2 audit tools; TF32 replay | legacy_cdist TF32 ~2 m at 160 bars; waypoints unscreened |
| Action scaling | a[0:2] | v_veh = clamp·2.5 m/s per axis (norm ≤ 3.54) | `NT:4960-4961` | D1-V1 axis PASS (`EQ1D:44`) | diagonal direction −18° from yaw drift |
| Governor (riskcap) | LiDAR scan (obs), target-like mask, v_veh xy | scaled v xy (direction kept) | `NT:5248-5395`; `SG:189-255, 465-503` | CPU tests; EQ-1 margin analysis | floor 2.0 m/s → negative stopping margin (−0.88 m at 0.1 m) (`EQ1:86`) |
| Altitude PI | z at step start | vz = clamp(4e + ∫e, ±2.5) | `NT:4979-5002` | D1 hover/climb PASS, drift −0.039 m/6 s | sag under tilt transients; z range 0.10–2.79 m recorded |
| Yaw | a[3] | ψ̇ = 3.0·a3 (Euler), clamp 3.0 | `NT:5006-5008`; `BLC:128-130` | — | no yaw hold; −17.8° to −131° open-loop drift (`EQ1D:109-111`) |
| Velocity → force | v_cmd^veh rotated by current yaw each 10 ms | f = m(K_v(v_d − v) + 9.81 e3), tilt ≤ 45° | `VC:20-53`; `BLC:111-125` | braking PASS 0.82/1.05/1.57 m | rise/overshoot NOT_RECORDED; τ_v ≈ 0.4 s derived only |
| Thrust / attitude | f, current R, ψ̇ | T = f_z/b3_z; τ (N·m) | `VC:56-95`; `BLC:127-185` | D1-A1 FAIL (contact flips) | actual tilt up to 91–111°; speed up to 4.79 m/s |
| Allocation + motors | wrench | 4 thrusts ∈ [0, 2] N, τ = 0.04 s | `control_allocation.py:52-114`; `MM:91-160` | D1-M1/M2 PASS (0.096 %) | no saturation priority; random initial thrust at reset |
| Physics | forces | state @ 100 Hz × 10 | `EM:444-473`; `base_sim_config.py:27-45` | determinism PASS | first-order dt error ~2.08×/halving (D1-C1 FAIL 2/20) |
| Contact detection | contact force | crashes flag | `EM:403-407`; `ENV:58-59` | D1-K1 PASS | detected 0.05–0.09 s after first contact (`EQ1D:48`) |
| Termination | pose, crashes, rel. segment | captured / crashed_out / truncation | `NT:6019-6304, 5065-5073` | EQ-1 evaluation contract | capture masks same-step crash; OOB ±1 m |
| Reset + render | terminations/truncations | post-reset obs for the same index | `EM:409-442`; `NT:5168-5182, 4701-4947` | ISO-Q02 / P03 (profile-limited) | global warp vertex rewrite (`warp_env_manager.py:44-53`); shared RNG |
| Observation | sensors at end of step | 898-D actor obs | `NT:6450-6481` | D3-T1 0-step latency | terminal obs = next episode |

## Model assumptions for a short-horizon candidate rollout

| # | Assumption | Status |
|---|---|---|
| A1 | ZOH 0.1 s decisions, plant at 10 ms × 10 substeps | **Measured** (D1-T1) |
| A2 | Command is in the yaw-only vehicle frame, re-rotated by the *current* yaw each substep, so it rotates with ψ inside the step | **Code fact**; consequence measured (D1-V1 −18°) |
| A3 | Per-axis clamp ±2.5 m/s, governed by exact riskcap on the decision-time scan (deterministic, replicable) | **Code fact**; governor CPU-tested |
| A4 | Horizontal velocity responds first order, τ_v = 1/K_v ≈ 0.4 s, lateral accel ≤ g·tan 45° ≈ 9.8 m/s², plus motor (0.04 s) and attitude lag | τ_v **derived** (consistent with braking 0.82 m @ 2 m/s); rise time/overshoot **NOT_RECORDED**; attitude-loop lag **unknown** |
| A5 | Braking distances 0.82 / 1.05 / 1.57 m from 2.0 / 2.5 / 3.52 m/s, time ≈ 1 s | **Measured** (D1-B1) |
| A6 | Altitude held at 1.0 m by PI; planner has no vertical authority | Hold **measured** in fixtures (drift −0.039 m / 6 s); behaviour during aggressive weaving **partly measured** (0.10–2.79 m extremes) |
| A7 | Yaw rate tracks the command (clamp 3 rad/s), with no yaw-angle hold | Clamp **code**; tracking lag **unknown**; drift **measured** |
| A8 | Collision = 0.28 × 0.28 box (circumradius 0.198 m) touching a bar box; must be avoided with margin ≥ 0.2 m plus one-step travel (≤ 0.35 m), because detection lags 0.05–0.09 s | Geometry **code**; delay **measured** |
| A9 | Capture = swept relative segment within 0.5 m (centre to centre) | **Code fact** |
| A10 | Target moves per E0/E1/E2 bounded law and does not react; future waypoints unknown to the planner | Law **code**; online only the tracker belief is legal |
| A11 | Episode-start transient from random motor thrusts U[0, 2] N | **Code fact**; magnitude **NOT_RECORDED** |
| A12 | Pre-contact validity only | **Measured**: post-contact dynamics not converged in dt |

## Top 8 dynamics/termination risks for a new arbitration planner

1. **Riskcap is not a brake.**
   - Its 2.0 m/s floor executes 2 m/s at 0.1 m clearance (margin −0.88 m). Crashes happen at about 1.96–1.98 m/s with a median of only 0.02–0.03 m of free travel (`EQ1:74, 86`).
   - Speeds the planner hands over are executed until a stop takes 0.8–1.6 m. Any safety claim must come from the planner's own stopcap-style law, not from the governor.
2. **Yaw–velocity coupling.**
   - Commands are vehicle-frame and rotate with yaw inside each step, and yaw is not held (drift −18° to −131°).
   - A world-frame point-mass rollout without a yaw model gives 31 %-scale vector errors. Candidates must include and close the loop on yaw rate.
3. **Lag versus horizon.**
   - τ_v ≈ 0.4 s and a stopping time of about 1 s mean one 0.1 s decision commits the vehicle for 5–10 steps.
   - Handing back to Policy F at speed inherits that momentum. Short horizons under 1 s cannot certify safety.
4. **Late contact plus capture-masks-crash plus post-reset terminal observation.**
   - Contacts are flagged 0.05–0.09 s late.
   - A capture in the same step erases the crash label.
   - The terminal-step observation, pose and tracker already belong to the next episode.
   - Planner scoring and evaluation must be terminal-first and use E-L0 contact labels.
5. **Handover distribution shift.**
   - Planner actions enter Policy F's `prev_action`: governed xy, plus raw z and yaw. The z channel is an indirect policy-state input.
   - Arbitration switches create out-of-distribution observation history for the frozen policy. The policy cannot be retrained (D-2026-10-14-0).
6. **Sensor-derived clearance errors.**
   - The planner's LiDAR or fused clearance inherits these defects:
     - the target-like carve-out can delete a real bar 0.53 m in front of the target (2.8 m false clear);
     - depth-upsampling creates phantom obstacles;
     - fusion has a −0.1 m bias;
     - the LiDAR minimum range is 0.2 m.
   - Also: 72-beam 5° quantisation, and the riskcap corridor ignores lateral returns, which account for 57–58 % of contacts (`SG:363-366`) (`EQ1D:59-62, 102-108`).
   - Overlapping `navrl_band` walls leave 0.47–0.59 m gaps against a 0.40 m body diagonal.
7. **Cross-env coupling invalidates matched arms.**
   - The global torch RNG is shared across envs and target/reset draws are event-dependent.
   - Production `WarpEnv.reset_idx` rewrites all vertex buffers, and root translations drift.
   - Paired planner-vs-Policy-F comparisons need the P03 RuntimeIsolation wrapper, which is qualified for one seed/density/profile only. Otherwise arms are UNPAIRED_STREAM.
8. **Envelope and model-validity limits.**
   - Actual tilt exceeds 45° (to 91–111°) and speed exceeds the command cap (to 4.79 m/s).
   - Post-contact motion is dt-unconverged, and position error is first order in dt (about 2 cm differences).
   - Altitude excursions approach the 0.1 m floor.
   - Historical moving cells with legacy_cdist are TF32-corrupted at 160 bars.
   - A rollout model is valid only pre-contact, at the 10 ms plant, in the E2 `direct_l2_v1` lineage. Its prediction error versus the simulator is **NOT_RECORDED** and must be measured before use.
