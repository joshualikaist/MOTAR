# CAM-K2 v1 — camera-only continuation with the 0.5 kg nominal vehicle (as a thrust cap), vehicle randomization and the OAK-D Pro depth model (preregistration, 2026-11-08)

**State:** preregistered, **NOT_RUN**. This text was repaired after design reviews r1 and r2 (both AMEND_FIRST; see
the sections "Design review r1" and "Design review r2"). Commit it before any run; it is never edited after the commit, and amendments go in a new dated file.
This is training only; it claims no performance result.

The unit is committed and run from the `MOTAR-camonly` worktree on branch `camonly/dev`. Code:
- launcher `aerial_gym/rl_training/rl_games/train_navrl_camk2.sh`;
- wrapper `tools/camk2_train.py`;
- entry `tools/realveh_entry.py` and `tools/realveh_python`;
- modules `aerial_gym/task/navrl_task/navrl_vehicle_rand.py` and `aerial_gym/task/navrl_task/navrl_depth_model.py`;
- tests `tests/test_camk2_train.py` and `tests/test_realveh.py`;
- the amended import guard in `tests/test_navrl_sensor_ablation.py`.

The Option A files, `aerial_gym/config/robot_config/navrl_rv05_quad_config.py` and
`resources/robots/quad/quad_navrl_rv05.urdf`, are documentation and the equivalence test's oracle only. No robot uses
them.

Design records (outside the repository):
- `~/motar_tmp/realveh_design_20261108/SURVEY.md` and `IMPLEMENTATION.md`;
- the reviews, `~/motar_tmp/camk2_20261108/review_design_r{1,2}/REVIEW.md`, with their probes;
- the depth probe rerun at f·B 66.1875: `~/motar_tmp/camk2_20261108/probe_depth_effects_fb66.{py,json}`.

## Why

- The real vehicle is about 0.5 kg with small motors (the user, 2026-11-06). Its only sensors are a Luxonis OAK-D Pro:
  a stereo depth pair and an autofocus RGB camera.
- CAM-K1 v1 trained the camera-only lineage on navrl_quad (0.25 kg, T/W 3.26) with noise-free, hole-free depth. Its
  preregistration named the vehicle and the stereo depth model as the next step, warm-started from this lineage.
- **The source lineage is weak and unevaluated.**
  - CAM-K1 v1 seeds 1 and 2 read TRAINED.
  - In their last 100 training epochs, capture was 26.3 % (s1) and 25.6 % (s2), and crash 73.7 % and 74.5 %
    (`docs/evidence/camonly_2026-11-04/camk1/README.md`). These are training rollouts.
  - CAM-K1-E has not run, so nothing is known about its evaluation performance.
- **Three treatments are bundled:** the 0.5 kg vehicle, the motor randomization and the depth model.
  - D-2026-11-06-1 lists them as later, separately preregistered units.
  - CAM-K2 trains them together without an ON/OFF arm, so no effect can be attributed to any one of them.
  - Unbundling is a user decision (end of this file).
- This unit starts a new policy lineage. **Policy F and CAM-K1 stay frozen and are never replaced.**

## The 0.5 kg vehicle as a thrust cap on navrl_quad

- **The vehicle defined.** The 0.5 kg nominal vehicle is the "Option A" placeholder: navrl_quad's geometry (motors at
  ±0.13 m, 0.28 × 0.28 × 0.08 m box) with every link mass and inertia element ×2:
  - 0.50 kg, J = 1.69e-3 / 1.69e-3 / 3.38e-3 kg m²;
  - per-motor maximum 2.7590625 N, so T/W 2.25;
  - Lee attitude gains ×2 (a uniform-density ×2 body flown with retuned gains);
  - τ 0.04 s and thrust-to-torque ratio 0.01 m, as navrl_quad.
- **Why it equals capped navrl_quad.**
  - Commanded wrench: the Lee velocity loop's force is (a − g)·m, and the attitude torque is the gains times the
    error. At the same state, Option A commands exactly 2× navrl_quad's wrench.
  - Thrust: the allocation is the same matrix. The motor model is homogeneous of degree 1 in thrust: its constant k
    cancels, and the 1e5 rate clamp never binds. So Option A delivers exactly 2× the thrust of navrl_quad with its
    per-motor reference capped at 2.7590625 / 2 N.
  - Mass and inertia are also 2×, so every linear and angular acceleration is identical.
- **It is equivalent to the Option A placeholder under these conditions only:**
  - Option A's ×2 Lee attitude gains. F's absolute gains on a real 0.5 kg body of this geometry would halve the
    attitude bandwidth.
  - The custom drag coefficients are 0 and the disturbance is disabled.
  - No IMU, which divides force by mass.
  - A kinematic target. The `legacy` target has no MotorModel.
  - The PhysX linear and angular damping of 0.01 acts as a mass-independent velocity decay (INFERRED from the code).
  - Contacts are terminal.
  - Not scaled: the absolute contact-detection threshold of 0.05 N (`env_manager.py:403-407`,
    `navrl_bars_env.py:59`). For the same marginal graze, Option A would produce about 2× the contact force, so it
    could count a graze that the capped quad does not. Real impacts (order 10 N) are unaffected.

  It is **not** a statement about "a 0.5 kg same-geometry vehicle" in general.
- **The cap.** CAM-K2 therefore flies navrl_quad, keeping its robot identity, URDF, config, Lee gains and checkpoint
  lineage, with the cap
  `cap = TW_NOMINAL × m × |g| / 4 = 2.25 × 0.25 × 9.81 / 4 = 1.37953125 N`.
  - m and g are read at run time from the controller's own `mass` and `gravity` tensors.
  - The cap is computed, not hard-coded, and recorded to a file.
  - At reset, the motor state that navrl_quad's reset draws in [0, 2] N is rescaled to [0, cap], as Option A's reset
    would draw it.
- **Numbers.** Analytic values from the two configs and URDFs, and the CPU test `TestOptionAEquivalence`:

| Quantity | navrl_quad, capped (CAM-K2) | Option A (0.50 kg) | navrl_quad uncapped (F, CAM-K1) |
| --- | --- | --- | --- |
| Per-motor maximum thrust | 1.37953 N (cap) | 2.7590625 N | 2.0 N |
| Thrust-to-weight | 2.25 | 2.25 | 3.262 |
| Hover thrust fraction | 0.4444 | 0.4444 | 0.3066 |
| Maximum thrust acceleration 4·T/m | 22.07 m/s² | 22.07 m/s² | 32.0 m/s² |
| Maximum roll acceleration 2·0.13·T/Ixx | 424.47 rad/s² | 424.47 rad/s² | 615.4 rad/s² |
| Maximum yaw acceleration 2·0.01·T/Izz | 16.33 rad/s² | 16.33 rad/s² | 23.67 rad/s² |

The test also simulates both on the real CPU `ControlAllocator` and `MotorModel`, with the same global seed: 60
random wrench steps driven to 95 % of the cap, then a saturating hold.
- Option A with 2× the wrench delivers 2× the motor forces within 9.5e-7 N and 2× the yaw torques within 1.1e-8 N m.
- The reset states are 2× apart.
- Per-step thrust/m and roll torque/Ixx agree.
- Control: the uncapped quad is not equivalent.
- The closed-loop Lee scaling itself is analytic and not simulated (review L-2).

## Fixed treatment

Everything is CAM-K1 v1's contract, apart from the rows marked in bold. Tests assert that every `export` and `unset`
of `train_navrl_camk1.sh` is in `train_navrl_camk2.sh` (apart from the lines below), and that CAM-K2 adds only its
own lines. CAM-K1's contract is GR-1 ctrl's:
- fixed 205 bars, curriculum off;
- LR 5e-6;
- the riskcap governor parameters from the same validated screen summary;
- main profile, 128 environments;
- `no_lidar`, FOV 66° × 54°, `yaw_align_weight` 0.6, `visibility_bonus` 0.03;
- robot navrl_quad.

| Item | CAM-K1 v1 | CAM-K2 v1 |
| --- | --- | --- |
| Start | F ep25000 | **CAM-K1 v1 seed s ep28000.** s1 `d7bf51a8…`, s2 `5128bf2e…`. Copied into the output folder with the SHA-256 checked before and after the run (the runner writes a `*_rlnorm.pth` sibling next to the copy). Both record robot navrl_quad with config `a76f8a3d…` and URDF `ad9a3110…`, so the restore passes the task's robot-lineage guard natively. The guard is not bypassed |
| Epochs | 25001..28000 (3,000) | **28001..29000 (1,000)** |
| Vehicle dynamics | navrl_quad, T/W 3.26 | **navrl_quad capped at T/W 2.25**: equivalent to the Option A placeholder under the conditions above. The robot identity is unchanged |
| Vehicle randomization | none | **`NAVRL_VEH_RAND=1`**, drawn per env at init and at every reset from its own generator: delivered-thrust factor 1 ± 0.15 per env × 1 ± 0.05 per motor on the capped thrust, and τ ∈ [0.03, 0.06] s per env. Effective T/W 1.82–2.72. Seeds 4811 (s1) and 4812 (s2) |
| Depth | renderer range, exact | **OAK-D Pro model `oakd`** (mode below): z-noise σ = z² · 0.15 px / 66.1875 px·m. Invalid where z < 0.4 m, z > 20 m, or an iid random hole (p = 0.02). Invalid is encoded as no return. Seeds 4821 (s1) and 4822 (s2) |
| Receipt and records | camtrain receipt | **realveh receipt** with all settings and the SHA-256 of 15 files (checked against HEAD after the run). The vehicle-randomization record of the computed cap. The telemetry lines of both modules. A sidecar manifest next to each checkpoint copy |
| Run tag | `…camk1-ep25000-…-camonly-s{s}` | `v2-camk2-ep28000-205bars-main-riskcap-camonly-rv05-s{s}`. "rv05" names the modelled 0.5 kg placeholder, not a robot identity |
| Wall cap | 5 h | **2.5 h**, `timeout -k 120`; seeds refuse unless the smoke's step time projects inside it (below) |
| Disk floor | 2 GB | **1.0 GB** at preflight. About 0.45 GB expected for the smoke and both seeds |

**Depth mode (review m-4).** OAK-D Pro, 800P, extended disparity with subpixel.
- **f·B:** 882.5 px (Luxonis' rectified 800P focal length) × 0.075 m = 66.1875 px·m. The unit's own calibration
  decides the real value; D-2026-11-06-1's 80° HFOV would give 57.2.
- **MinZ 0.4 m** is the user's value. It is conservative against the extended-disparity limit
  f·B / 190 px ≈ 0.35 m. Standard mode would be f·B / 95 px ≈ 0.70 m.
- **σ_d 0.15 px** is a subpixel figure (INFERRED). The resulting σ_z is 2.3 mm at 1 m, 3.6 cm at 4 m, 0.23 m at 10 m,
  0.73 m at 18 m and 0.91 m at 20 m.
- **MaxZ 20 m** equals perception's detection far plane.
- **Holes:** iid per pixel at p = 0.02. They barely affect the column-minimum obstacle map, because a full-height bar
  covers 26–90 rows of one column. They act on target pixel counts. The survey's column / blob holes are not built.

**Reward, terminations, observation (review m-1).** The reward function, the terminations and the 898-D actor contract
are unchanged from CAM-K1.
- The reward's static-safety term `r_ss` still reads the simulator's LiDAR (privileged, reward only).
- The depth model changes the `depth` argument of perception's `observe`, and through it everything that reads that
  depth:
  - the actor's obstacle and target features;
  - the tracker's range;
  - the camera visibility that feeds the reward's `visibility_bonus` (`navrl_task.py:6482`, `:6062-6069`). That is a
    change of the reward's input, not of the reward function;
  - under `no_lidar`, the riskcap governor's clearance, which is computed from the fused camera scan
    (`navrl_sensor_ablation.py:131-148`).
- Vehicle randomization changes only the delivered motor thrust and τ. The controller keeps navrl_quad's mass and
  gains.

**Why these values.**
- **TW_NOMINAL 2.25** is the T/W of the Option A placeholder.
- **TW_SCALE 0.15 and MOTOR_IMBAL 0.05** are one point on the frontier where the worst case
  2.25 × (1 − TW_SCALE) × (1 − MOTOR_IMBAL) stays ≥ 1.8. They are not "the largest ranges": 0.16 / 0.04 also passes.
- **The 1.8 floor** has no external source. It was set for this unit (2026-11-08) to keep the weakest env at least
  0.39 g above the 1.41 g a 45° tilt needs.
- **τ 0.03–0.06 s** brackets the inherited 0.04 s. The plausible small-motor range is 0.02–0.06 s (INFERRED, survey).

## Runs, checks and labels

- **Development smoke first:** `tools/camk2_train.py 1 --smoke` runs epochs 28001..28002 under the same contract,
  with its own run tag `…-smoke`. It is not a result.
  - It reads **SMOKE_OK** when every post-run check below passes, with the smoke's own epoch and at least two
    epoch step-time lines.
  - Seeds refuse unless two things hold:
    - the smoke read SMOKE_OK at the same HEAD;
    - 1,000 epochs × the smoke's last epoch step time × 1.25 is at most 2.5 h. For example, CAM-K1's smoke read
      4.77 s, which projects to 5,963 s.
    - The "step time" is rl_games' env-stepping time. It excludes policy inference and the PPO update, and runs
      about 9 % under the epoch wall time: CAM-K1 took 4.83 s of wall time per epoch against a mean step time of
      4.43 s. The 1.25 margin covers it: at the 7.2 s boundary the wall estimate is about 7,850 s
      (7.2 × 1,000 × 4.83 / 4.43), under the 9,000 s cap.
  - A failed smoke or gate stops the unit for repair.
- **Order:** seed 1, then seed 2, each once, through `tools/camk2_train.py {1,2}`. Seed 2 starts only if seed 1
  reads TRAINED.
- **The tree stays frozen from the commit of this unit until seed 2 has ended (review L-7).** The smoke gate binds
  HEAD and the smoke folder is single-use, so any commit in between strands the unit. The records follow in a
  separate commit after the chain.
- **Preflight refusals:**
  - the approval `~/motar_tmp/camk2_20261108/execution_approval.json` does not name this SCOPE and HEAD;
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
  - the smoke rate gate (seeds).
- **Entry and module refusals:**
  - any setting that is missing or differs;
  - `NAVRL_ROBOT` other than unset or navrl_quad;
  - any `NAVRL_REALVEH_*` other than the receipt path;
  - worst-case T/W below 1.8;
  - TW_NOMINAL above the robot's own T/W, computed at run time from the controller's mass;
  - `NAVRL_PERCEPTION_PERTURB` on together with the depth model;
  - a module parameter set while its switch is off;
  - a script other than `runner.py`;
  - an existing receipt or record;
  - a motor update before init.
- **Labels:**
  - **TRAINED** needs all of the following:
    - exit 0;
    - the host's fault count, boot id and online CPUs unchanged;
    - exactly one new run folder ending `…-rv05-s{seed}`;
    - `last_gen_ppo_ep_29000_*.pth` exists (28002 for the smoke);
    - **the realveh receipt** shows every setting in the table: per-seed seeds, `no_lidar` and the four CAM-K1
      overrides, task robot navrl_quad, worst case ≥ 1.8, both modules installed with those settings,
      `NAVRL_PERCEPTION_PERTURB=0`, and no retired variable;
    - every receipt SHA-256 equals the file committed at HEAD;
    - **the vehicle-randomization record** shows TW_NOMINAL 2.25 and the cap applied, with cap = 2.25 × m × g / 4 for
      the controller's m in [0.249, 0.251] kg and g = 9.81, navrl_quad's 4 × 2 N and its own T/W above 2.25;
    - **the telemetry lines** (review m-2) are in the launcher log with plausible values:
      - vehicle randomization:
        - one install line with the settings;
        - an init line with the cap applied and factors spanning more than 0.05 inside the bounds 0.8075–1.2075;
        - at least one reset line with envs > 0 and factors and τ inside their bounds;
        - a motor-update count > 0, from the reset lines or the first-update line (review r2 N-7);
      - depth model: one install line with the settings; a first-frame line; a summary line after 50 frames, with
        the hole share inside 0.02 × (1 ± 0.5), the MinZ and MaxZ shares in [0, 0.5), mean |noise| in (0, 1) m and
        σ_z at the median valid z equal to k·z²;
    - **the trained checkpoint** records robot navrl_quad with the committed `navrl_quad_config.py` and
      `quad_navrl_collide.urdf` SHA-256 (lineage unchanged). The epoch equals the final epoch. These are read from
      the checkpoint's pickle without torch and without running code from it;
    - the source copy is unchanged;
    - the task's startup line `hfov=66deg` is in the logs;
    - at least one epoch step-time line (two for the smoke), which the rate gate needs;
    - the tracked tree is unchanged.

    Both modules write their lines to the stream that was `sys.stderr` when they were installed. rl_games' quiet
    startup swaps `sys.stderr` for /dev/null while it builds the env, which is when the init line is written; the
    stream bound at install, outside that context, still reaches the log (review r2 N-1, tested inside the real
    `quiet_rl_io.devnull_stdio()`).
  - **VOID_HOST_FAULT**: the host changed. The run is not evidence.
  - **TRAINING_FAIL** or **PREFLIGHT_FAIL**: otherwise. A timeout reads TRAINING_FAIL after 2.5 GPU-hours.
- **Descriptive records, not gates** (the step-time line itself is required above, but its values gate only the
  seeds, through the smoke):
  - the per-epoch step times;
  - the crash causes per cause (bar contact, below/floor, above, out of bounds) from the task's `NavRL crashdiag`
    lines: number of windows, first, last and mean of the last 10. NOT_RECORDED when the log has none, as in a
    2-epoch smoke;
  - the depth-model heartbeat count.
- **Outputs:** `~/motar_tmp/camk2_20261108/train_{smoke,s1,s2}/` (single use). Each holds:
  - the report, the launcher log and the session log;
  - the realveh receipt and the vehicle-randomization record;
  - the source copy and its `*_rlnorm.pth` sibling;
  - a copy of the trained checkpoint with its SHA-256, and a sidecar `<checkpoint>.manifest.json` (review L-6).

  The checkpoint's `env_state` cannot distinguish CAM-K2's treatments from CAM-K1's contract. The manifest binds the
  checkpoint SHA-256, the source, the receipt and record SHA-256, the treatments, the commit and this file. It is
  written after the verdict and carries it (review r2 N-6). CAM-K2-E must bind a manifest whose verdict is TRAINED. The raw run folder stays in the worktree's `runs/`.
- **Wall time** (INFERRED): CAM-K1 took 14,499 s (s1) and 14,364 s (s2) for 3,000 epochs, a mean step time of
  4.43 / 4.40 s. That suggests about 1.3–1.5 h per 1,000-epoch seed. The depth model's GPU cost is gated by the
  smoke.

## What does not follow

- **No claim about capture, crashes or transfer to the real vehicle.**
- **No claim that the cap, the randomization or the depth model helps.** There is no ON/OFF arm, and the three are
  bundled.
- **No claim that the vehicle is the real one**, or that the depth model reproduces the OAK-D Pro beyond its stated
  constants.
- If both seeds read TRAINED, what may be said is limited to this: "CAM-K2 v1 seeds 1 and 2 were trained for 1,000
  epochs from CAM-K1 v1 under the T/W 2.25 cap, equivalent in this simulator to the Option A placeholder (2× masses
  and inertia, 2× Lee attitude gains); ±15 % / ±5 % delivered-thrust randomization and τ 0.03–0.06 s; and the OAK-D
  Pro depth model as configured." Training-rollout statistics are descriptive only.
- **CAM-K2-E** needs its own preregistration, with thresholds fixed before it runs.
  - It must compare CAM-K1 and CAM-K2 under the same evaluation settings: cap, randomization and depth model each on
    and off.
  - It must show that OFF reproduces native on the GPU.
  - It must bind the checkpoint manifests.
  - It must read `below` crashes in the light of the altitude sag (next section).

## Disclosed limits

- **Altitude sag at the start of each episode (review M-1).** The altitude PI absorbs the per-env thrust bias in
  steady state only.
  - The PI is P 4, I 1 s⁻¹, |I| and |vz_cmd| ≤ 2.5 m/s, updated every 0.1 s (`navrl_task.py:4993-5002`). Its
    integral restarts at 0 every episode (`:4891`). The actor's z output is overwritten (`:9486`), so the policy
    cannot compensate.
  - Hover never saturates: at f = 0.8075 the demand is 0.76 N per motor against the 1.38 N cap. But weak envs start
    each episode sagging and spend vertical authority on the bias.
  - The review's probe (`review_design_r1/probe_altitude_sag.json`; Lee vertical loop K_vz 2.5, use_rps motor,
    start at 1.0 m, τ 0.03–0.06 s, motor state 0–cap):

| Env thrust factor f | Steady vz command spent on the bias | z_min after episode start | z after 8 s |
| --- | --- | --- | --- |
| 1.0 (nominal, capped) | 0 | 0.80–0.996 m | ≈ 1.00 m |
| 1.0 (F / CAM-K1, uncapped) | 0 | 0.80–0.98 m | ≈ 1.00 m |
| 0.85 (env low) | +0.69 m/s (28 % of 2.5) | 0.64–0.77 m | 0.97–0.98 m |
| 0.8075 (env and all motors low) | +0.94 m/s (37 %) | 0.57–0.68 m | 0.96–0.97 m |
| 1.2075 (all high) | −0.67 m/s | 0.89–1.00 m, then about 2 % above | ≈ 1.02 m |

  - Recovery is slow (slow mode about 3.7 s). The authority spent on the bias leaves less margin against the
    tilt-induced sag that historically caused floor strikes.
  - CAM-K1's last 10 crash windows were almost all bar contacts: s1 99.8 % bar contact, 0.13 % below, 0.09 % OOB;
    s2 99.45 %, 0.55 % and 0.01 % (its `NavRL crashdiag` lines, parsed by `camk2_train.crash_causes`). CAM-K2 records
    the same shares, so that a rise in `below` can be attributed.
  - **The real vehicle is less sensitive here.** PX4 has a hover-thrust estimator and rate-loop integrators, so on
    this point the simulation is more pessimistic than the vehicle.
  - TW_SCALE 0.15 was kept by the coordinating agent (Claude Code, 2026-11-08) after design review r1. That is
    within the user's 12:07 approval of the unit.
- **Placeholder vehicle.**
  - Option A keeps navrl_quad's 0.13 m arms, 0.28 m box, inertia shape, τ, yaw ratio and gains-to-inertia ratio. Only
    the mass budget (0.5 kg) and the T/W (2.25) are modelled.
  - The real frame, arm length, inertia, thrust table, τ, yaw ratio and saturation behaviour are unknown and still
    needed from the user.
- **Thrust scaling stands in for mass.**
  - The per-env factor scales thrust and yaw torque together. That equals a motor-constant change, or a body whose m
    and J both scale by 1/f. It is not a CoM payload change, which would leave J nearly unchanged.
  - The factor scales the effective maximum thrust too, so the effective T/W varies from 1.82 to 2.72.
  - The per-motor ±5 % leaves a steady attitude error of at most about 0.9° roll/pitch and 0.15° yaw (calculated in
    the review), about 0.06 m/s of velocity error.
- **Lee gains, not PX4.** The simulated attitude loop is Lee. The real vehicle flies PX4 rate loops, and the quantity
  to match is the closed-loop velocity response (G9 / SITL S1–S2). No step response has been run with the cap.
- **Saturation.**
  - At the nominal T/W 2.25 a 45° tilt takes 1.41 g, leaving 0.84 g for attitude torque. In the weakest env (T/W
    1.82) it leaves 0.40 g.
  - The per-motor clamp has no thrust/attitude priority. F and CAM-K1 trained at 3.26.
  - The governor's braking assumption (riskcap 2.96 m/s²) stays feasible at the worst case.
- **Optimistic invalid-depth encoding.**
  - Invalid pixels become perception's `max_camera_range` (20 m), which every consumer reads as no return: the fusion
    maps ≥ the 10 m obstacle far plane to the LiDAR no-return, and the detector mask needs depth < 20 m.
  - A bar closer than MinZ (z < 0.4 m) therefore disappears from the obstacle map and from the riskcap filter, which
    does not brake in the last ≈ 0.15 s at 2.5 m/s.
  - The real OAK outputs 0, and the deployment pipeline must map 0 the same way. The conservative clamp to MinZ is
    not built; the choice is the user's.
- **Target pixels near and far** (review L-3; probe rerun with f·B 66.1875,
  `~/motar_tmp/camk2_20261108/probe_depth_effects_fb66.json`).
  - MinZ removes target pixels from the mask from a centre distance of 0.65 m on-axis, and all of them by 0.55 m.
    Capture is at 0.5 m, so the target is lost for at most about one 0.1 s step before capture.
  - Noisy target pixels clamp to 20 m and leave the mask. The drop probability per pixel is 0.3 % at 18 m, 11 % at
    19 m and 28 % at 19.5 m. "The target's camera range is unchanged" holds to about 18 m.
  - The visibility income drops near 0.55–0.65 m and beyond about 18 m.
  - The tracker's σ_r model is not adapted to the z² noise. The review estimated it about 1.5× overconfident near
    19 m at f·B 57.2; at 66.2 the gap is smaller, not recomputed.
- **Column-minimum bias** (calculated, review L-4, rerun at f·B 66.1875 in the same file). The fusion takes the minimum over a bar's
  rows in one column, which biases iid noise low: −0.09 m at 4 m, −0.18 m at 6 m, −0.30 m at 8 m and −0.45 m at
  10 m. It is conservative: the filter brakes earlier at range.
- **Other depth-model limits.**
  - Real stereo noise is spatially correlated; the model's is iid.
  - The 40 × 24 → 160 × 90 upsampling phantoms (EQ-1D) remain, and they are noised as if native.
  - Depth and RGB share one 66° × 54° FOV and one yaw-only mount.
- **Detector:** still F's colour rule, not a learned detector and not the real camera.
- **Seeds and length:** two training seeds and 1,000 epochs. Two seeds show the seed spread only; they do not
  establish an effect. 1,000 epochs may not be enough to adapt to the lower T/W.
- **Disk and memory** (review L-8): 2.4 GB free at review time, against about 0.45 GB expected. No per-process memory
  cap is set, as in CAM-K1.
- **What the receipt hashes** (review L-10). It covers 15 files but omits:
  - the controller code (`velocity_control.py`, `base_lee_controller.py`, `base_controller.py`);
  - `navrl_detector.py`, `robot_manager.py` and `navrl_task_config.py`;
  - this launcher and wrapper.

  The clean tree and the approval at HEAD bind them.
- **Not run on a GPU yet:**
  - the init_tensors / reset / motor wrappers on CUDA tensors and in the real reset order;
  - the cap from PhysX's mass;
  - the depth model's memory and time;
  - the restore through an absolute source path outside `runs/`;
  - OFF reproducing native.

  The smoke is the first test of all of these.

## Design review r1 (Claude Opus 5.5, 2026-11-08): AMEND_FIRST, repaired as follows

**MAJOR**

| Finding | Decision | Repair |
| --- | --- | --- |
| M-1 altitude sag | keep TW_SCALE 0.15 | Quantified in "Disclosed limits". Per-cause crash shares are recorded post-run (NOT_RECORDED when absent). The PX4 hover-thrust-estimator note is added |

**MINOR**

| Finding | Repair |
| --- | --- |
| m-1 depth reach | Disclosed: the visibility-bonus input, the `no_lidar` riskcap clearance and the tracker see the modelled depth; bars under 0.4 m vanish from the filter too |
| m-2 runtime evidence | Telemetry lines from both modules (install, init and resets; install, first frame, 50-frame summary and heartbeat). The wrapper requires them with plausible values, tested with positive and negative fixtures |
| m-3 step cost | The smoke records its epoch step times. Seeds refuse if 1,000 × the last × 1.25 exceeds 2.5 h |
| m-4 depth mode | Named: 800P, extended disparity + subpixel, f·B 66.1875 px·m, MinZ 0.4 m as the conservative user value, σ_d 0.15 px. iid holes are kept and stated as nearly inert for obstacles |
| m-5 import guard | The entry imports the ablation normally. `tests/test_navrl_sensor_ablation.py` lists `tools/realveh_entry.py`, with positive controls for the listed entries and for unlisted importers in three import forms |
| m-6 before commit | **Open, for the committer:** the full PUBLIC_CPU suite; the launcher preflight (`NAVRL_PREFLIGHT_ONLY=1`); a `docs/EXPERIMENTS.md` family entry for this file (`tests/test_experiment_index.py`); a clean tree |
| m-7 weak, bundled source | Disclosed in "Why" and "What does not follow". Unbundling is a user decision |

**LOW**

| Finding | Repair |
| --- | --- |
| L-1 equivalence wording | The conditions are listed, with "equivalent … under these conditions" |
| L-2 closed loop | Disclosed: the Lee scaling is analytic, not simulated |
| L-3 target near / far | Quantified |
| L-4 column-minimum bias | Calculated |
| L-5 wording | Frontier wording; the floor's source; the depth module docstring says "open decision"; 0.40 g at T/W 1.82 |
| L-6 checkpoint provenance | Sidecar manifest |
| L-7 tree freeze | Stated |
| L-8 disk | Disclosed |
| L-9 test gaps | The partial reset leaves the other envs' motor state untouched; the RNG probe has a positive control; the launcher contract is two-way |
| L-10 hashes | Disclosed |

## Design review r2 (Claude Opus 5.5, 2026-11-08): AMEND_FIRST on one new MAJOR, repaired as follows

| Finding | Repair |
| --- | --- |
| **N-1 MAJOR** | The vehicle-randomization init line went to /dev/null inside rl_games' quiet env construction, so every valid smoke would have read TRAINING_FAIL. Both modules now bind their output stream at install time. CPU tests run `initialize` and the first depth frames inside the real `quiet_rl_io.devnull_stdio()` and still find the lines (positive). A negative control shows the old late-binding pattern loses them. The module docstrings are corrected |
| N-2 | `camk2_train.py` docstring: branch `camonly/dev` |
| N-3 | "Step time" is env-stepping time, about 9 % under the epoch wall time, which the 1.25 margin covers. The required step-time line is stated under TRAINED |
| N-4 | The f·B 66.1875 probe rerun is copied to `~/motar_tmp/camk2_20261108/probe_depth_effects_fb66.{py,json}` and cited |
| N-5 | TW_SCALE 0.15: kept by the coordinating agent (Claude Code, 2026-11-08) after r1, within the user's 12:07 approval of the unit |
| N-6 | The manifest is written after the verdict and includes it |
| N-7 | The reset line carries the motor-update call count, and a first-update line is written. The post-run check requires a count > 0 |

## Decisions for the user before commit

1. Bundling the three treatments (vehicle cap, randomization, depth model) in one unit, or unbundling them into
   separately preregistered units with ON/OFF arms.
2. The invalid-depth convention: optimistic no-return as built, or clamp to MinZ.
3. Approve the run itself: smoke, then seed 1, then seed 2. This needs the commit of this text with the code, and an
   approval file naming SCOPE and HEAD.
4. Optionally, the real vehicle data (arm length, mass, thrust table, τ) and the OAK-D unit's calibration.
