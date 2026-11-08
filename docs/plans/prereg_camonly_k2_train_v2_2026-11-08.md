# CAM-K2 v2 — camera-only continuation with the real vehicle's thrust-to-weight (a raised per-motor limit on navrl_quad), vehicle randomization and the OAK-D Pro depth model (preregistration DRAFT, 2026-11-08)

**State:** **DRAFT, NOT_RUN, not committed.** Repaired after design review r1 (AMEND_FIRST; section "Design review r1").
Its repository path is `docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md`, which `tools/camk2v2_train.py` names and
refuses to run without.
- It is never edited after the commit. Amendments go in a new dated file.
- The code and this text were written while S4F-1's real-time SITL run needed an idle host. After it ended (about
  18:10 KST), the focused CPU tests ran (section "Tests"); the PUBLIC_CPU suite has not run.
- This is training only. It claims no performance result.

**CAM-K2 v1 is WITHDRAWN_BEFORE_RUN.** The user's real-vehicle specs arrived after v1's commit (`62e2699`) and before
any run. The user, relaying their senior, 2026-11-08 at about 16:10 KST: "드론은 무게 1.1kg, 프로펠러 5x4.3x3, 모터
velox 2550kV" (the drone weighs 1.1 kg, with 5x4.3x3 propellers and Velox 2550 kV motors).
- v1 modelled a 0.5 kg placeholder at T/W 2.25. That is the wrong direction from navrl_quad's own 3.26: the real
  vehicle is stronger, about T/W 4.0 (INFERRED, below).
- v1's committed files and its preregistration stay byte-unchanged. How v1 is kept from running is in "How v1
  refuses".

The unit is committed and run from the `MOTAR-camonly` worktree on branch `camonly/dev`. It is written in
`MOTAR-realveh` (at `ab2eeb5`, plus the uncommitted v2 files). Code:
- launcher `aerial_gym/rl_training/rl_games/train_navrl_camk2v2.sh` (new; a copy of v1's launcher with v2's values);
- wrapper `tools/camk2v2_train.py` (new; a copy of `tools/camk2_train.py` with v2's constants and checks);
- entry `tools/realveh_entry.py` and `tools/realveh_python` (unchanged);
- modules `aerial_gym/task/navrl_task/navrl_vehicle_rand.py` (**changed**: the "raise" mode, below) and
  `aerial_gym/task/navrl_task/navrl_depth_model.py` (unchanged);
- tests `tests/test_camk2v2_train.py` (new) and `tests/test_realveh.py` (extended); the minimal edit to
  `tests/test_camk2_train.py` (below).

Design records (outside the repository):
- `~/motar_tmp/realveh_design_20261108/SURVEY.md` and `IMPLEMENTATION.md` (Part "CAM-K2 v2");
- v1's design reviews r1–r3, `~/motar_tmp/camk2_20261108/review_design_r{1,2,3}/REVIEW.md`. Their findings on the
  shared code (telemetry binding, step-time gate, manifest, motor-update count) carry over unchanged;
- the depth probe at f·B 66.1875: `~/motar_tmp/camk2_20261108/probe_depth_effects_fb66.{py,json}` (unchanged depth
  model).

## Why

- **The real vehicle** (the user, 2026-11-08): 1.1 kg; 5x4.3x3 propellers; Velox 2550 kV motors. Its only sensors are
  a Luxonis OAK-D Pro (the user, 2026-11-06).
- **T/W 4.0 is INFERRED, not measured** (review r1 m-2).
  - It assumes a 4S battery (usual for 2550 kV on 5-inch props) and a wheelbase of about 225 mm.
  - **The source point.** LIGPOWER V2306 V2 2400KV with T5143S props on 4S gives 1,084 g at 100 % throttle (bench).
    So 4 × 1.084 / 1.1 = 3.94.
  - **Two corrections, in opposite directions.** 2550 kV adds RPM over 2400 kV, so more thrust. Static bench thrust at
    full pack voltage overstates in-flight thrust, because the voltage sags. It is also unknown whether 1.1 kg includes
    the battery.
  - **So 4.0 nominal.** 4.0 × 1.1 kg / 4 = 1.1 kgf = 10.79 N per motor. TW_SCALE 0.15 spans 3.4–4.6 per env, which
    covers both 3.94 and the earlier 4.2 estimate.
  - Battery cells, wheelbase and thrust-stand data are open questions for the user (end of this file).
- CAM-K1 v1 trained the camera-only lineage on navrl_quad (0.25 kg, T/W 3.26) with noise-free, hole-free depth.
- **The source lineage is weak and unevaluated.**
  - CAM-K1 v1 seeds 1 and 2 read TRAINED.
  - In their last 100 training epochs, capture was 26.3 % (s1) and 25.6 % (s2), and crash 73.7 % and 74.5 %
    (`docs/evidence/camonly_2026-11-04/camk1/README.md`). These are training rollouts.
  - CAM-K1-E's evaluation is committed at `ab2eeb5` (`docs/evidence/camonly_2026-11-04/camk1e/`).
- **Three treatments are bundled:** the vehicle's T/W, the motor randomization and the depth model.
  - D-2026-11-06-1 lists them as later, separately preregistered units.
  - CAM-K2 trains them together without an ON/OFF arm, so no effect can be attributed to any one of them.
  - Unbundling is a user decision (end of this file).
- This unit starts a new policy lineage. **Policy F and CAM-K1 stay frozen and are never replaced.**

## The real vehicle's T/W as a raised per-motor limit on navrl_quad

- **The robot identity is unchanged.** CAM-K2 v2 flies navrl_quad, keeping:
  - its robot name, URDF, config and Lee gains;
  - its checkpoint lineage. The restore passes the task's robot-lineage guard natively; the guard is not bypassed (the
    coordinating agent's decision for v1, kept).
- **The limit.** `limit = TW_NOMINAL × m × |g| / 4 = 4.0 × 0.25 × 9.81 / 4 = 2.4525 N` per motor.
  - m and g are read at run time from the controller's own `mass` and `gravity` tensors. The limit is computed, not
    hard-coded, and recorded to a file.
  - navrl_quad's own motor maximum is 2.0 N (own T/W 3.262). Since TW_NOMINAL is above it, the module runs in mode
    **"raise"**. The raise is opt-in (review r1 m-1): it needs `NAVRL_VEH_RAND_LIMIT_MODE=raise`, which only
    `train_navrl_camk2v2.sh` and CAM-K2-E's treatment environment set, and which is part of the receipt settings.
    Without it, a TW_NOMINAL above the robot's own T/W refuses at init, exactly as committed for v1. After the original `init_tensors`, it replaces the motor model's run-time `max_thrust` tensor with the
    limit. `MotorModel.update_motor_thrusts` clamps its reference to `max_thrust` (`motor_model.py:93`), so delivered
    thrust up to the limit gets through.
  - No file or config is edited. The robot's own 2.0 N stays recorded (`max_thrust_per_motor_n`) next to the limit
    (`thrust_limit_per_motor_n`).
  - The construction-time motor state is rescaled from [0, 2] N to [0, limit] at init. After that, the original
    `reset_idx` draws the state in [0, limit] itself, because its `max_thrust` is the limit.
  - The module also still wraps the reference as `min(ref, limit)` and multiplies by the per-env and per-motor factors,
    as in v1.
- **Bounds.** `TW_NOMINAL` must be in [1, 6.0] (`MAX_TW_NOMINAL`); above 6.0 refuses. The modes:
  - **"lower"** below the robot's own T/W. This is v1's cap, behaviourally unchanged;
  - **"raise"** above it, only with `NAVRL_VEH_RAND_LIMIT_MODE=raise`. "raise" with a TW_NOMINAL below the robot's
    own T/W refuses;
  - **"none"** equal to it, within a relative 1e-6.

  A TW_NOMINAL that is above some envs' own T/W and below others' refuses. The worst-case floor stays: the entry
  refuses TW_NOMINAL × (1 − TW_SCALE) × (1 − MOTOR_IMBAL) below 1.8.
- **Identity** (review r1 m-4). The robot's own T/W is 8 / (m |g|) from its own tensors. On the CPU stand-in (m = 0.25
  kg, float32 gravity 9.81) that is 3.261977434, printed **3.2619774**. With that value, zero ranges and τ equal to the
  config's, the run is bitwise identical to OFF. Two CPU tests assert this: `test_identity_settings_are_bitwise_off`
  and `test_printed_identity_constant_is_bitwise_off`.
  - "3.2620" is 6.9e-6 away, outside the 1e-6 band. It now refuses without the opt-in raise (tested).
  - On the GPU the value follows PhysX's mass.

**Equivalence statement** (review r1 L-2 wording).
- **The thrust-to-weight matches a 1.1 kg vehicle at T/W 4.0.** That is, the maximum thrust acceleration is 4.0 g and
  the hover thrust fraction is 0.25.
- **Drag and frontal area per unit mass are navrl_quad's** (PhysX damping), not those of a 1.1 kg airframe.
- **Rotational dynamics do not match.** navrl_quad's arms (0.13 m) and inertia (Ixx 8.45e-4 kg m²) are kept. So at the
  raised limit its maximum roll acceleration is about 1.1–2.6× an estimate for a 1.1 kg 5-inch frame.
- **It is not a statement about the real vehicle's attitude dynamics, motor curve, τ or saturation behaviour.**

| Quantity | navrl_quad raised (CAM-K2 v2) | 1.1 kg vehicle at T/W 4.0 (INFERRED) | navrl_quad own (F, CAM-K1) |
| --- | --- | --- | --- |
| Per-motor maximum thrust | 2.4525 N (limit) | 10.79 N (1.1 kgf) | 2.0 N |
| Thrust-to-weight | 4.0 | 4.0 | 3.262 |
| Hover thrust fraction | 0.25 | 0.25 | 0.3066 |
| Maximum thrust acceleration 4·T/m | 39.24 m/s² (4.0 g) | 39.24 m/s² | 32.0 m/s² |
| Maximum roll acceleration 2·L·T/Ixx | 754.6 rad/s² (L 0.13 m, Ixx 8.45e-4) | 686.7 rad/s² (Ixx 2.5e-3) to 286.1 rad/s² (Ixx 6e-3); L = 0.225 / 2 / √2 = 0.0795 m | 615.4 rad/s² |
| Ratio of the sim's roll acceleration to the estimate | — | 1.10× (Ixx 2.5e-3) to 2.64× (Ixx 6e-3) | — |
| Maximum yaw acceleration 2·c·T/Izz | 29.02 rad/s² (c 0.01 m, Izz 1.69e-3) | about 20–62 rad/s² (c 0.01–0.02 m, Izz 0.007–0.011, INFERRED) | 23.67 rad/s² |

- **Formula.** The roll and yaw rows use v1's formula: two motors at the limit and two off. Because the hover fractions
  are matched, the sim-to-real roll ratio does not depend on the operating point.
- **Inertia basis** (review r1 m-3, INFERRED). A component build-up gives about 2.2–2.5e-3 kg m²:
  - 4 motors of about 37 g at y = 0.0795 m: about 0.9e-3;
  - a battery of about 0.2 kg on top: about 0.35e-3;
  - a central stack of about 0.55 kg: about 0.6–0.8e-3;
  - arms and the OAK-D: about 0.3e-3.

  A 0.75 kg 5-inch research frame scaled to 1.1 kg suggests about 3e-3. The range is 2.5e-3 to 6e-3, with 6e-3 as a
  heavy-frame bound. The arm, inertia and yaw ratio of the real column are estimates for a 225 mm 5-inch X frame, not
  measurements.
- **Tests.** The 1.1 kg column's thrust-to-weight rows are arithmetic on the limit's definition (review r1 L-1): CPU test
  `test_thrust_to_weight_equals_a_1p1_kg_vehicle_at_tw_4p0`. It shows that the record uses the controller's mass,
  nothing more. The rotational rows are calculated, not tested.

**Attitude control: PX4 against Lee.**
- The real vehicle flies PX4: an attitude P loop over body-rate PID loops, with mixer desaturation that gives the
  attitude priority over thrust.
- The simulator flies navrl_quad's Lee geometric controller with its gains, tuned for navrl_quad's inertia, through a
  per-motor clamp with no thrust/attitude priority.
- Unsaturated, raising the limit changes nothing in the attitude loop: Lee commands the same torques, and navrl_quad's
  inertia gives the same response. Only the saturation ceiling moves.
- The quantity to match against the vehicle remains the closed-loop velocity response (G9 / SITL S1–S2). No step
  response has been run with the raised limit.

**Where the raised limit can act.**
- The NavRL Lee velocity loop bounds the tilt at 45° (`lee_controller_config_navrl.py:74`, `NAVRL_MAX_TILT_DEG`
  unset): the lateral force is at most the vertical force.
- So the extra thrust mainly adds vertical-acceleration and attitude-torque headroom, not lateral acceleration at
  hover-level vertical thrust.
- How often F's or CAM-K1's motor references reach navrl_quad's 2 N was never recorded. This unit records it for its
  own runs (review r1 M-1, next paragraph). If they rarely do, the raised limit changes little, and the unit mainly
  tests the randomization and the depth model.
- **Fixed before the run (review r2 m1):** if fewer than 1 % of the recorded motor commands exceed 2 N in a seed, no
  result of that seed is attributed to the thrust-to-weight change. The record then states that the raise was nearly
  inert.

**Telemetry of the raise** (review r1 M-1). In raise mode, the module's motor-update wrapper accumulates counts on the
device, with no per-step synchronisation:
- the command elements (the reference passed to `update_motor_thrusts`, before any clamp);
- those above the robot's own `max_thrust` (2 N);
- those at the raised limit (≥ limit × (1 − 1e-6));
- the running maximum command.

The reset lines and the update lines (first and every 100,000th) carry these counts, together with a **live read-back
of the motor model's `max_thrust` tensor** (`live_max_thrust_n`). The init line and the record carry the read-back taken
just after the swap (`live_max_thrust_per_motor_n`). `tools/camk2v2_train.py` requires all of the following, else
TRAINING_FAIL:
- the record's read-back equal to the limit (relative 1e-6);
- every reset and update line with the counters and a live read-back equal to the init line's limit;
- consistent counts (0 ≤ at limit ≤ above own ≤ elements);
- at least one line after motor updates.

The shares of commands above 2 N and at the limit, up to the last log line, are reported in the train report
(`telemetry.veh_rand.raise`) as descriptive records. They are not gates. The modes "lower" and "none" carry no
counters.

## Fixed treatment

Everything is CAM-K1 v1's contract, apart from the rows marked in bold.
- Tests assert that every `export` and `unset` of `train_navrl_camk1.sh` is in `train_navrl_camk2v2.sh` (apart from the
  lines below), and that v2 adds only its own lines.
- CAM-K1's contract is GR-1 ctrl's:
  - fixed 205 bars, curriculum off;
  - LR 5e-6;
  - the riskcap governor parameters from the same validated screen summary;
  - main profile, 128 environments;
  - `no_lidar`, FOV 66° × 54°, `yaw_align_weight` 0.6, `visibility_bonus` 0.03;
  - robot navrl_quad.

| Item | CAM-K1 v1 | CAM-K2 v2 |
| --- | --- | --- |
| Start | F ep25000 | **CAM-K1 v1 seed s ep28000.** s1 `d7bf51a8…`, s2 `5128bf2e…`. Copied into the output folder, with the SHA-256 checked before and after the run (the runner writes a `*_rlnorm.pth` sibling next to the copy). Both record robot navrl_quad with config `a76f8a3d…` and URDF `ad9a3110…`, so the restore passes the task's robot-lineage guard natively. The guard is not bypassed |
| Epochs | 25001..28000 (3,000) | **28001..29000 (1,000)** |
| Vehicle dynamics | navrl_quad, T/W 3.26 | **navrl_quad with the per-motor limit raised to T/W 4.0** (2.4525 N; `NAVRL_VEH_RAND_LIMIT_MODE=raise`): the thrust-to-weight of a 1.1 kg vehicle at T/W 4.0; rotational dynamics and drag navrl_quad's. The robot identity is unchanged |
| Vehicle randomization | none | **`NAVRL_VEH_RAND=1`**, drawn per env at init and at every reset from its own generator. Delivered-thrust factor 1 ± 0.15 per env × 1 ± 0.05 per motor on the limited thrust; τ ∈ [0.02, 0.04] s per env. Effective T/W per env 3.4–4.6; per motor 3.23–4.83. Seeds 4911 (s1) and 4912 (s2) |
| Depth | renderer range, exact | **OAK-D Pro model `oakd`**, unchanged from v1: z-noise σ = z² · 0.15 px / 66.1875 px·m. Invalid where z < 0.4 m, z > 20 m, or at an iid random hole (p = 0.02); invalid is encoded as no return. Seeds 4921 (s1) and 4922 (s2) |
| Receipt and records | camtrain receipt | **realveh receipt** with all settings and the SHA-256 of 15 files (checked against HEAD after the run). The vehicle-randomization record of the computed limit and its mode. The telemetry lines of both modules. A sidecar manifest (unit "CAM-K2 v2") next to each checkpoint copy |
| Run tag | `…camk1-ep25000-…-camonly-s{s}` | `v2-camk2v2-ep28000-205bars-main-riskcap-camonly-tw40-s{s}`. "tw40" names the modelled T/W, not a robot identity |
| Wall cap | 5 h | **2.5 h**, `timeout -k 120`; the seeds refuse unless the smoke's step time projects inside it (below) |
| Disk floor | 2 GB | **1.0 GB** at preflight. About 0.45 GB expected for the smoke and both seeds |

**Depth mode.** Unchanged from v1 (review m-4). OAK-D Pro at 800P, extended disparity with subpixel.
- f·B 66.1875 px·m.
- MinZ 0.4 m (the user's value).
- σ_d 0.15 px (INFERRED).
- MaxZ 20 m.
- iid holes at p = 0.02.

The sources and the σ_z values are in v1's preregistration, "Depth mode".

**Reward, terminations, observation.** Unchanged from v1 (review m-1).
- The reward function, the terminations and the 898-D actor contract are CAM-K1's.
- The depth model changes the `depth` argument of perception's `observe`, and through it everything that reads it:
  - the actor's obstacle and target features;
  - the tracker's range;
  - the visibility input of `visibility_bonus`;
  - the `no_lidar` riskcap clearance.
- The reward's `r_ss` still reads the simulator's LiDAR (privileged, reward only).
- Vehicle randomization changes only the delivered motor thrust, the run-time limit and τ. The controller keeps
  navrl_quad's mass and gains.

**Why these values.**
- **TW_NOMINAL 4.0** is the real vehicle's T/W, INFERRED from the user's specs and the bench point above.
- **TW_SCALE 0.15 and MOTOR_IMBAL 0.05** are kept from v1, so the randomization's relative spread is unchanged. The worst
  case 4.0 × 0.85 × 0.95 = 3.23 is far above the 1.8 floor.
- **τ 0.02–0.04 s** is the plausible range for 5-inch motors in this class (INFERRED, survey). It replaces v1's
  0.03–0.06 s; navrl_quad's own τ is 0.04 s.
- **The 6.0 bound** has no external source. It was set for this unit (2026-11-08) as a ceiling on the raise, above
  any plausible 5-inch build at 1.1 kg.

## Runs, checks and labels

- **Development smoke first:** `tools/camk2v2_train.py 1 --smoke` runs epochs 28001..28002 under the same contract,
  with its own run tag `…-smoke`. It is not a result.
  - It reads **SMOKE_OK** when every post-run check below passes, with the smoke's own epoch and at least two epoch
    step-time lines.
  - Seeds refuse unless two things hold:
    - the smoke read SMOKE_OK at the same HEAD;
    - 1,000 epochs × the smoke's last epoch step time × 1.25 is at most 2.5 h.

    As in v1, the "step time" is rl_games' env-stepping time, about 9 % under the epoch wall time, and the 1.25
    margin covers it.
  - A failed smoke or gate stops the unit for repair.
- **Order:** seed 1, then seed 2, each once, through `tools/camk2v2_train.py {1,2}`. Seed 2 starts only if seed 1
  reads TRAINED.
- **The tree stays frozen from the commit of this unit until seed 2 has ended.** The smoke gate binds HEAD and the
  smoke folder is single-use. The records follow in a separate commit after the chain.
- **Preflight refusals:**
  - the approval `~/motar_tmp/camk2v2_20261108/execution_approval.json` does not name this SCOPE and HEAD;
  - not branch `camonly/dev`;
  - the tracked or untracked tree is not clean;
  - this preregistration is not committed;
  - not exactly one RTX 3070;
  - any GPU compute process other than the user's remote desktop;
  - a NavRL runner, PX4 SITL or Gazebo process;
  - the source checkpoint SHA-256 does not match;
  - less than 1.0 GB free;
  - an existing output directory;
  - `--max-hours` other than 2.5;
  - the smoke rate gate (seeds only).
- **Entry and module refusals:**
  - any setting that is missing or differs;
  - `NAVRL_ROBOT` other than unset or navrl_quad;
  - any `NAVRL_REALVEH_*` other than the receipt path;
  - worst-case T/W below 1.8;
  - TW_NOMINAL outside [1, 6.0], or above some envs' own T/W and below others';
  - `NAVRL_PERCEPTION_PERTURB` on together with the depth model;
  - a module parameter set while its switch is off;
  - a script other than `runner.py`;
  - an existing receipt or record;
  - a motor update before init.
- **Labels:**
  - **TRAINED** needs all of the following:
    - exit 0;
    - the host's fault count, boot id and online CPUs unchanged;
    - exactly one new run folder ending `…-tw40-s{seed}`;
    - `last_gen_ppo_ep_29000_*.pth` exists (28002 for the smoke);
    - **the realveh receipt** shows every setting in the table: the per-seed seeds, `no_lidar` and the four CAM-K1
      overrides, task robot navrl_quad, worst case ≥ 1.8, both modules installed with those settings,
      `NAVRL_PERCEPTION_PERTURB=0`, and no retired variable;
    - every receipt SHA-256 equals the file committed at HEAD;
    - **the vehicle-randomization record** shows all of these:
      - mode "raise" (with `NAVRL_VEH_RAND_LIMIT_MODE=raise` in the settings), TW_NOMINAL 4.0 and the limit applied;
      - limit = 4.0 × m × g / 4 for the controller's m in [0.249, 0.251] kg and g = 9.81;
      - the live read-back of `max_thrust` equal to the limit;
      - `thrust_limit_per_motor_n` equal to the computed limit and above 2 N;
      - navrl_quad's own 4 × 2 N, with its own T/W equal to 8 / (m g) within a relative 1e-6;
    - **the telemetry lines** are in the launcher log with plausible values:
      - vehicle randomization:
        - one install line with the settings;
        - an init line with mode "raise", the limit applied, and factors spanning more than 0.05 inside the bounds
          0.8075–1.2075;
        - the raise counters and the live read-back on every reset and update line (above);
        - at least one reset line with envs > 0, and factors and τ inside their bounds (τ in [0.02, 0.04] s);
        - a motor-update count > 0, from the reset lines or the first-update line;
      - depth model:
        - one install line with the settings;
        - a first-frame line;
        - a summary line after 50 frames, with the hole share inside 0.02 × (1 ± 0.5), the MinZ and MaxZ shares in
          [0, 0.5), mean |noise| in (0, 1) m, and σ_z at the median valid z equal to k·z²;
    - **the trained checkpoint** records robot navrl_quad with the committed `navrl_quad_config.py` and
      `quad_navrl_collide.urdf` SHA-256 (lineage unchanged). Its epoch equals the final epoch. Both are read from the
      checkpoint's pickle without torch and without running code from it;
    - the source copy is unchanged;
    - the task's startup line `hfov=66deg` is in the logs;
    - at least one epoch step-time line (two for the smoke);
    - the tracked tree is unchanged.

    Both modules write their lines to the stream that was `sys.stderr` when they were installed, so the init line
    reaches the log inside rl_games' quiet construction (v1 review r2 N-1, unchanged).
  - **VOID_HOST_FAULT**: the host changed. The run is not evidence.
  - **TRAINING_FAIL** or **PREFLIGHT_FAIL**: otherwise. A timeout reads TRAINING_FAIL after 2.5 GPU-hours.
- **Approval** (review r1 E-m4). A new approval naming this SCOPE and HEAD is required. It will cite:
  - the user's 12:07 KST approval, "예, 오늘 전부 (권장)", which named "실기체 모델 학습";
  - the user's 16:10 KST instruction after relaying the specs, "이에 맞춰서 데이터 다시 수집하면 될거같아".

  The approval file is `~/motar_tmp/camk2v2_20261108/execution_approval.json`. Neither answer is read as covering
  this scope by itself.
- **Descriptive records, not gates:**
  - the per-epoch step times;
  - the crash causes per cause (bar contact, below/floor, above, out of bounds) from the task's `NavRL crashdiag`
    lines: number of windows, first, last and mean of the last 10. NOT_RECORDED when the log has none, as in a
    2-epoch smoke;
  - the depth-model heartbeat count.
- **Outputs:** `~/motar_tmp/camk2v2_20261108/train_{smoke,s1,s2}/` (single use; not v1's folders). Each holds:
  - the report, the launcher log and the session log;
  - the realveh receipt and the vehicle-randomization record;
  - the source copy and its `*_rlnorm.pth` sibling;
  - a copy of the trained checkpoint with its SHA-256, and a sidecar `<checkpoint>.manifest.json`.

  The manifest is written after the verdict and carries it. It binds the checkpoint, the source, the receipt and record
  SHA-256, the treatments, the commit and this file. CAM-K2-E binds a manifest whose verdict is TRAINED and whose
  unit is "CAM-K2 v2".
- **Wall time** (INFERRED): about 1.3–1.5 h per 1,000-epoch seed, from CAM-K1's 4.43 / 4.40 s mean step time. The raised
  limit adds no compute. The depth model's GPU cost is gated by the smoke.

## How v1 refuses

- **No approval.** v1's preflight needs `~/motar_tmp/camk2_20261108/execution_approval.json` naming v1's SCOPE and HEAD.
  None exists, and none may be written.
- **Its single-use folders are occupied.** v1's preflight refuses an existing output folder ("output exists (single
  use)", `tools/camk2_train.py:450-451`). `~/motar_tmp/camk2_20261108/{train_smoke,train_s1,train_s2}/WITHDRAWN.md`
  occupy all three on purpose and state why. So v1 reads PREFLIGHT_FAIL for the smoke and both seeds, and v1's code
  stays byte-unchanged.
- **The seeds also need v1's smoke.** They refuse without a SMOKE_OK report in `train_smoke/`, which cannot be written.
- **v1's own refusal at run time is restored** (review r1 m-1). v1's settings carry no `NAVRL_VEH_RAND_LIMIT_MODE`, so
  a TW_NOMINAL above the robot's own T/W refuses at init, as committed. `tests/test_camk2_train.py` is byte-identical
  to `ab2eeb5`.
- **The CPU tests `tests/test_camk2v2_train.py::TestIsolation`** check these points:
  - v1's wrapper, launcher and preregistration have no diff against HEAD;
  - review r1 m-5: v1's own `main()` and preflight run on a temporary copy of v1's unit folder with the three markers.
    - The smoke reads PREFLIGHT_FAIL with "output exists (single use)" and creates nothing.
    - Both seeds read PREFLIGHT_FAIL ("the development smoke did not read SMOKE_OK").
    - Positive control: on an empty unit folder that problem is absent ("no approval file" is present).
    - The GPU, process, source and host probes are stubbed; git is read only.
    - On a host with `~/motar_tmp` a missing folder fails rather than skips.
- **Only the wrapper path is guarded** (review r1 L-5). v1's launcher, run directly with `CAMK2_SOURCE_CKPT` set,
  skips the wrapper, the markers and the approval check, as every unit's launcher does. The approval rule is the
  governance. The WITHDRAWN_BEFORE_RUN status is also committed with v2: v1's `docs/EXPERIMENTS.md` family entry and
  this section.
- **CAM-K2-E reads v2 only.** It imports `camk2v2_train`, reads `~/motar_tmp/camk2v2_20261108/`, and refuses a
  manifest whose unit or preregistration is v1's.

## The module change, and what it does to v1

- `navrl_vehicle_rand.py` is the one committed file that v2 changes. It is shared, so the change and its tests are
  stated here:
  - an opt-in "raise" mode (`NAVRL_VEH_RAND_LIMIT_MODE=raise`). Without it, the refusal of a TW_NOMINAL above the
    robot's own T/W stands as committed;
  - the upper bound on TW_NOMINAL goes from 10 to 6.0;
  - the record gains `mode`, `thrust_limit_per_motor_n` and `live_max_thrust_per_motor_n`. The init line gains `mode`
    and `live_max_thrust_n`;
  - raise mode only: the command counters (the M-1 telemetry above);
  - a second `initialize()` on one motor model refuses (review r1 L-3).
- **v1's own path ("lower") is behaviourally unchanged:**
  - the same cap min(ref, cap);
  - the same rescale of the reset state to [0, cap];
  - the same factors and τ draws.
  The code is not byte-identical: the mode selection replaced the two comparisons.
- **v1's committed test file is unchanged** (byte-identical to `ab2eeb5`). Its assertion that 0.5 kg refuses holds
  again.
- **`tests/test_realveh.py` is extended (48 tests):**
  - the bad-settings list gains TW_NOMINAL 6.5 (22 cases), with 6.0 as the edge positive control;
  - the run-time refusals now read:
    - 3.5 refuses without the opt-in, as committed, and with "lower";
    - 3.5 with "raise" raises (control);
    - 3.26 with "raise" refuses;
    - 6.5 with "raise" refuses;
    - "3.2620" refuses (not the identity);
  - `TestRaisedLimit` has five tests:
    - the raised clamp takes effect: the delivered thrust at saturation equals the limit, is above 2 N and never
      exceeds the limit; the OFF control stays at 2 N;
    - the v2 ranges scale around the limit, with τ in [0.02, 0.04];
    - the raise counters and the live read-back. Saturating: counts > 0 and the maximum command above the limit.
      Unsaturated control: 0. Modes lower, none and OFF: no counters;
    - the printed identity constant is bitwise OFF;
    - thrust-to-weight arithmetic for 1.1 kg / T/W 4.0.

## What does not follow

- **No claim about capture, crashes or transfer to the real vehicle.**
- **No claim that the raised limit, the randomization or the depth model helps.** There is no ON/OFF arm, and the three
  are bundled.
- **No claim that the simulated vehicle is the real one.**
  - Only its thrust-to-weight is matched, at an INFERRED T/W.
  - Its roll acceleration is about 1.1–2.6× the estimate, and its drag per unit mass is navrl_quad's.
  - The depth model reproduces the OAK-D Pro only in its stated constants.
- If both seeds read TRAINED, what may be said is limited to this: "CAM-K2 v2 seeds 1 and 2 were trained for 1,000
  epochs from CAM-K1 v1 on navrl_quad with the per-motor thrust limit raised to T/W 4.0. That is the thrust-to-weight
  of a 1.1 kg vehicle at T/W 4.0, an INFERRED value for the user's vehicle, with navrl_quad's rotational
  dynamics. Training also used ±15 % / ±5 % delivered-thrust randomization, τ 0.02–0.04 s, and the OAK-D Pro depth
  model as configured." Training-rollout statistics are descriptive only.
- Below 1 % of commands above 2 N (see "Where the raised limit can act"), nothing is attributed to the T/W change.
- **CAM-K2-E** (draft `~/motar_tmp/camk2e_20261108/PREREG_DRAFT.md`) evaluates v2. It needs its own committed
  preregistration, with thresholds fixed before it runs.

## Disclosed limits

- **T/W 4.0 is INFERRED** (from one bench point, corrected by judgement). The battery cells, wheelbase, thrust table, τ and frame inertia are unknown. A 6S battery
  or a heavier take-off weight would change it materially.
- **Rotational capacity is navrl_quad's, not the real frame's.**
  - The sim's maximum roll acceleration at the limit is 755 rad/s², against about 286–687 rad/s² estimated for the
    real frame (ratio 1.1–2.6×).
  - The sim's maximum yaw acceleration is 29.0 rad/s², against a wide estimate of 20–62 rad/s².
- **Drag per unit mass** is navrl_quad's (PhysX damping), not a 1.1 kg airframe's (review r1 L-2).
  - Attitude saturation is therefore rarer in the sim than on the vehicle, for the same commands.
- **PX4 against Lee.** See "Attitude control" above. The sim's per-motor clamp has no thrust/attitude priority, while
  PX4's mixer desaturates with attitude priority.
- **The 45° tilt limit** bounds lateral force at the vertical force. The raised limit mostly adds vertical and attitude
  headroom. Whether F's or CAM-K1's commands ever reach 2 N is NOT_RECORDED.
- **Reset motor state.** At reset, the motor state is drawn in [0, 2.4525] N instead of F's [0, 2] N. This is the
  raised vehicle's own reset, as v1's cap rescaled to [0, cap]. It changes the first-step transient.
- **Altitude sag at episode start** (v1 review M-1).
  - The steady vz command spent on the per-env thrust bias depends on the factor f, not on the limit, while hover is
    unsaturated. Hover is unsaturated: the weakest env's hover demand is 0.76 N per motor against the 2.4525 N limit.
    So v1's steady-state rows carry over: +0.69 m/s at f = 0.85, +0.94 m/s at f = 0.8075, −0.67 m/s at f = 1.2075.
  - The transient z_min was probed for v1 at τ 0.03–0.06 s with the motor state in 0–1.38 N
    (`review_design_r1/probe_altitude_sag.json`). It was **not rerun** for v2's τ 0.02–0.04 s and state 0–2.4525 N,
    so it is NOT_RECORDED for v2.
  - PX4's hover-thrust estimator and rate-loop integrators make the real vehicle less sensitive here than the sim.
- **Thrust scaling stands in for mass.**
  - The per-env factor scales thrust and yaw torque together. That equals a motor-constant change, or a body whose m
    and J both scale by 1/f; it is not a CoM payload change.
  - The effective T/W varies from 3.4 to 4.6 per env.
  - The per-motor ±5 % leaves a small steady attitude error (v1's review: at most about 0.9° roll/pitch).
- **Saturation.** At T/W 4.0, a 45° tilt takes 1.41 g and leaves 2.59 g. In the weakest env (3.23) it leaves 1.82 g.
  The governor's braking assumption (riskcap 2.96 m/s²) stays feasible.
- **Depth-model limits** (unchanged from v1):
  - the invalid-depth encoding is optimistic: a bar closer than MinZ disappears from the obstacle map and the filter;
  - the target is lost near 0.55–0.65 m and beyond about 18 m;
  - the column-minimum bias is low and conservative;
  - the noise is iid, not spatially correlated;
  - the EQ-1D upsampling phantoms remain;
  - depth and RGB share one FOV.

  The details are in v1's "Disclosed limits", which apply unchanged.
- **Detector:** still F's colour rule, not a learned detector and not the real camera.
- **Seeds and length:** two training seeds and 1,000 epochs. Two seeds show the seed spread only; they do not establish
  an effect.
- **What the receipt hashes:** 15 files, as in v1. It omits the controller code, `navrl_detector.py`,
  `robot_manager.py`, `navrl_task_config.py`, and this launcher and wrapper. The clean tree and the approval at HEAD
  bind them.
- **Not run on a GPU yet:**
  - the raised `max_thrust` tensor on CUDA and in the real reset order;
  - the limit from PhysX's mass;
  - the depth model's memory and time;
  - OFF reproducing native.

  The smoke is the first test of all of these.

## Tests (CPU, 2026-11-08, after S4F-1 ended)

The focused tests ran in one process, in both orders, under the required form: cores 6,7, `nice -n 10`, an 8 GB
`ulimit -v`, `CUDA_VISIBLE_DEVICES=""`. The logs are in `~/motar_tmp/camk2v2_20261108/tests_r1/`.

| File | Tests | Result |
| --- | --- | --- |
| `tests/test_realveh.py` | 48 | OK |
| `tests/test_camk2_train.py` (v1, byte-identical to `ab2eeb5`) | 33 | OK |
| `tests/test_camk2v2_train.py` | 38 | OK |
| `tests/test_navrl_sensor_ablation.py` | 15 | OK |
| All four in one process, forward and reverse | 134 | OK |

The PUBLIC_CPU suite has not run; it must pass before the commit.

## Review

- **v1's design reviews r1–r3** cover the shared machinery: telemetry binding, step-time gate, manifest and
  motor-update count. They carry over.
- **Design review r1 of v2** (below) read AMEND_FIRST.
- **Re-review.** A focused check of the M-1 and m-1 diffs and their tests is suggested before the commit.

## Design review r1 (Claude Opus 5.5, fresh context, 2026-11-08 16:47–17:10 KST): AMEND_FIRST, repaired as follows

The review is `~/motar_tmp/camk2_20261108/review_v2_r1/REVIEW.md`. The repairs follow, with the coordinating agent's
decisions.

| Finding | Repair |
| --- | --- |
| **M-1 MAJOR** whether the raise acts is unrecorded; the live clamp is not read back | Device-side counters in the update wrapper (raise mode only): command elements, above the robot's own 2 N, at the limit, running maximum; no per-step sync. `raise_counters()` adds them and a live read-back of `max_thrust` to the reset and update lines. The init line and the record carry the read-back taken after the swap. `camk2v2_train` requires the record's read-back, the counters and the live read-back on every line, and their consistency, and reports the shares (descriptive). `camk2e_run` requires the same per cell; `camk2e_analyse` reports the shares per treated cell. Tests: saturating counts > 0, unsaturated control 0, no counters in lower / none / OFF; five telemetry negatives, two record negatives |
| m-1 implicit mode removes v1's refusal | `NAVRL_VEH_RAND_LIMIT_MODE` ∈ {lower, raise}. Without it (v1's settings), a raise refuses at init with v1's message. "raise" is set only by `train_navrl_camk2v2.sh` and CAM-K2-E's treatment env, and is part of the receipt settings. `tests/test_camk2_train.py` is byte-identical to `ab2eeb5` again. **One deviation from the coordinator's wording ("required"):** the key is optional, with absent meaning v1's behaviour, because v1's committed launcher and test must stay byte-unchanged and still pass. An unknown value refuses |
| m-2 T/W 4.2 uncited | TW_NOMINAL 4.0, from LIGPOWER V2306 V2 2400KV / T5143S / 4S, 1,084 g → 3.94, with the two corrections stated. Every derived number is recomputed: limit 2.4525 N, hover 0.25, 39.24 m/s², roll 754.6 rad/s², effective T/W 3.4–4.6, worst case 3.23, tag `tw40` |
| m-3 inertia basis | Component build-up stated; Ixx 2.5e-3–6e-3; roll ratio 1.10–2.64× |
| m-4 identity value | 3.261977434 from the robot's tensors, printed 3.2619774, tested bitwise; "3.2620" shown not to be the identity |
| m-5 grep-only v1 test | v1's own `main()`/preflight on a temporary copy of the marked folder, with a positive control; fails rather than skips on the maintainer's host |
| L-1 | The 1.1 kg test is described as arithmetic on the limit's definition |
| L-2 | "Thrust-to-weight (maximum thrust acceleration and hover fraction) matches"; drag per mass disclosed |
| L-3 | A second `initialize()` on one motor model refuses (tested) |
| L-4 | `redraw` docstring corrected. In raise mode, "cap" in the field names is documented as the raised limit; the names are kept for v1's checks |
| L-5 | "Only the wrapper path is guarded" stated in "How v1 refuses" |

## Decisions and questions for the user before the commit

1. **The real vehicle's data (open questions):**
   - **battery cells** (4S assumed; 6S would change the T/W);
   - **wheelbase** (about 225 mm assumed);
   - **whether 1.1 kg includes the battery**;
   - **thrust-stand data** for the Velox 2550 kV motor with 5x4.3x3 props at that battery (the maximum thrust per
     motor, the thrust curve and, if available, the motor time constant). The motor's model number would also help.

   With these, TW_NOMINAL, τ and the rotational estimate can be set from data instead of INFERRED.
2. Bundling the three treatments (T/W, randomization, depth model) in one unit, or unbundling them into separately
   preregistered units with ON/OFF arms.
3. The invalid-depth convention: optimistic no-return as built, or clamp to MinZ.
4. Approve the run itself: smoke, then seed 1, then seed 2. This needs the commit of this text with the code, and an
   approval file `~/motar_tmp/camk2v2_20261108/execution_approval.json` naming SCOPE and HEAD.
5. Optionally, the OAK-D unit's calibration (f·B), and the frame's inertia.

## Design review r2 (Claude Opus 5.5, fresh context): PASS_WITH_LIMITS

The review is `docs/evidence/camonly_2026-11-04/camk2/REVIEW_v2_design_r2.md`. It found M-1, m-1, m-2 and m-4
repaired, with no MAJOR finding.
- **r2-m1:** no threshold had been set for the raise being inert. The 1 % rule above was added before the commit.
- **r2-L1:** `raise` with a T/W equal to the robot's own selects `none`, which is harmless; it is kept and disclosed.
- **r2-L2:** the counts cover the run up to the last log line.
- **r2-L3:** `tests/test_realveh.py` runs in PUBLIC_CPU before the commit.
