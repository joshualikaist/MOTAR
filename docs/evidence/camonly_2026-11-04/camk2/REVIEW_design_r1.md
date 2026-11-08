# CAM-K2 v1 — independent design review r1 (code + draft preregistration)

- **Reviewer:** Claude Opus 5.5 (fresh context), 2026-11-08, read-only.
- **Worktree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-realveh`, branch `realveh/dev`, HEAD `814c28a29e6f1951efd648baf2f3c4768ff26b2d`.
  The tracked tree is unmodified (`git diff --stat HEAD` is empty). Every file under review is untracked and new.
- **Under review:** `navrl_vehicle_rand.py`, `navrl_depth_model.py`, `tools/realveh_entry.py`, `tools/realveh_python`,
  `tools/camk2_train.py`, `train_navrl_camk2.sh`, `tests/test_realveh.py`, `tests/test_camk2_train.py`, the Option A
  URDF and config (documentation only), and `~/motar_tmp/camk2_20261108/PREREG_DRAFT.md` (r2).
- **Reviewed bytes** (SHA-256 prefixes; they equal IMPLEMENTATION.md's table):
  - `navrl_vehicle_rand.py` `ccad2353…`, `navrl_depth_model.py` `68a67c69…`;
  - `realveh_entry.py` `3fe72f38…`, `realveh_python` `8936f9f8…`;
  - `camk2_train.py` `5fda69a0…`, `train_navrl_camk2.sh` `77cc713e…`;
  - `test_realveh.py` `92943d22…`, `test_camk2_train.py` `15f810b9…`;
  - `quad_navrl_rv05.urdf` `eefdd3f9…`, `navrl_rv05_quad_config.py` `58c026b0…`;
  - `PREREG_DRAFT.md` `aa2cda80…`.
- **What I did:**
  - read the files under review and the code they wrap or depend on: BaseMultirotor, MotorModel, ControlAllocator,
    the Lee controllers, robot_manager mass/inertia, env_manager contact crashes, navrl_task's altitude PI, reward and
    capture, navrl_perception `observe` / `_detect_rgbd` / fusion, navrl_detector rendering, the sensor ablation,
    camtrain_entry, camk1_train, both launchers and train_navrl_v2_search.sh;
  - ran CPU tests and probes only (no CUDA, no Isaac Gym), under `nice 19`, `taskset 6,7` and `ulimit -v 8 GB`.
- **Written here:**
  - this report;
  - `tests_run.log`;
  - `probe_altitude_sag.py` / `.json` (vertical channel under the cap and thrust factor);
  - `probe_depth_effects.py` / `.json` (column-minimum bias, far-plane drop, MinZ before capture).

## Verdict: **AMEND_FIRST**

- **No BLOCKER.** The code does what it says:
  - the wrappers are correct, fail closed and isolated;
  - OFF and identity settings are bitwise native on the CPU paths;
  - no pinned file is edited, and no ground truth reaches the actor;
  - the warm-start pins match CAM-K1's TRAINED reports and the committed evidence;
  - the wrapper's preflight and post-run checks are strong and non-vacuous.
- **Why amend first:** a preregistration cannot change after its commit, and one claim in it needs correcting.
  "The controller keeps navrl_quad's mass and gains, so the altitude PI absorbs the per-env thrust bias" holds in
  steady state only. The transient and the authority it costs are material and are not disclosed (MAJOR M-1).
- **Recommended in the same amendment:** several MINOR text and telemetry fixes. Before commit, also run the full
  PUBLIC_CPU suite and the launcher preflight (m-6).
- After the amendment, a check of the diff alone is enough. No new full review is needed unless the code changes go
  beyond added logging.

## Tests and checks run

| Check | Result |
| --- | --- |
| `test_realveh.py` (prescribed command) | 41 OK |
| `test_camk2_train.py` (prescribed command) | 26 OK, none skipped; the CAM-K1 copies are present and their SHA-256 match |
| `test_navrl_sensor_ablation.py -k test_no_production_import` | OK, but only because the new entry avoids the guard's regex (m-5) |
| Two-way diff of the `export`/`unset` lines of `train_navrl_camk1.sh` and `train_navrl_camk2.sh` | Clean. The only differences are the source, the epochs, the tag and log, PYTHON/receipt, the `NAVRL_VEH_RAND_*` and `NAVRL_DEPTH_*` lines, and `unset NAVRL_ROBOT…` |
| CAM-K1 pins | `d7bf51a8…` (s1) and `5128bf2e…` (s2) equal `checkpoint.sha256` and `copy_sha256` in `~/motar_tmp/camk1_20261106/train_s{1,2}/train_report.json`, both TRAINED at `5e58984`, and appear in `docs/evidence/camonly_2026-11-04/camk1/` |
| Robot identity SHAs | `navrl_quad_config.py` `a76f8a3d…` and `quad_navrl_collide.urdf` `ad9a3110…`, as the prereg states |
| Free disk on `/` at review time | 2.4 GB |

## 1. Equivalence: the T/W 2.25 cap on navrl_quad versus the 0.5 kg Option A

**It holds exactly, analytically, for the nominal vehicle without randomization.** Each step checked:
- **Commanded force:** `forces = (accel − gravity) · mass`, with mass the PhysX `robot_mass` (`velocity_control.py:32`,
  `base_controller.py:22`). robot_manager fills that mass at env creation, before `prepare_for_sim` →
  `robot.init_tensors` (`robot_manager.py:470`, `:112-121`). So the cap computed in the `init_tensors` wrapper sees the
  PhysX mass (INFERRED from the code order; the smoke is the first run).
- **Attitude torque:** `−K_R e_R − K_ω e_ω + ω×(Jω)` in absolute N·m (`base_lee_controller.py:152-170`). Option A, with
  gains ×2 and J ×2, commands exactly 2× the torque.
- **Tilt limit and tilt compensation:** both depend on force ratios only (`velocity_control.py:34-80`).
- **Allocation:** the pinv and the matrix are identical (motors at ±0.13 m, verified against the URDF joint origins).
  Forces are applied at the motor links (`force_application_level="motor_link"`). Option A's URDF has the same joints
  and collision box.
- **Motor model:**
  - `use_rps=True` with rk4 and discrete mixing (`base_quad_config.py:176`, `motor_model.py:91-141`).
  - In √thrust units `k` cancels exactly, so the per-env random `k` of navrl_quad is irrelevant. The dynamics are
    homogeneous of degree 1 in thrust.
  - The rate clamp `max_thrust_rate = 1e5` never binds. The largest rpm rates are about 9.7e3 (capped) and 1.1e4
    (Option A).
  - The reference clamp `[0, cap]` equals `[0, 2.759]/2`.
- **Reset state:** the original draws in [0, 2] N, which `redraw` rescales to [0, cap]. With the same uniform draw, that
  is Option A's reset distribution.
- **PhysX and drag:**
  - PhysX `linear_damping` and `angular_damping` are 0.01 (`base_quad_config.py:94-95`). They act as a velocity decay,
    independent of mass, so INFERRED is correct.
  - The custom drag coefficients are all 0 (`:77-80`) and the disturbance is disabled (`:72`).
- **Velocity clamps:** `max_linear_velocity` and `max_angular_velocity` are the same for both.
- **Task:**
  - the altitude PI works in velocity units (`navrl_task.py:4997-5002`);
  - capture is a distance test (`:6110-6123`); height bounds and OOB are kinematic;
  - the target is `legacy` (kinematic; train_navrl_v2_search.sh refuses `physical`, `:154-190`), so no MotorModel other
    than the robot's exists;
  - no reward term reads mass, thrust or motor state (grep);
  - the governor's braking assumption (riskcap `brake_mps2` 2.96 m/s², summary.json) stays feasible: at 45° tilt every
    env needs T/W ≥ 1.414, and the minimum is 1.82.

**Where it does not hold, or holds only by construction:**
- **Contact detection is an absolute force threshold** (LOW L-1). A crash is `|robot contact force| > 0.05 N`
  (`env_manager.py:403-407`, `navrl_bars_env.py:59`). For the same contact geometry and speed, Option A produces about
  2× the contact force. So a marginal graze that the capped quad does not count can count for Option A. For real
  impacts (order 10 N) this is irrelevant, but "contacts are terminal" does not cover the detection threshold.
- **Option A is a uniform-density ×2 body with retuned (×2) Lee attitude gains.** The equivalence is to that
  construction. It is not to "a 0.5 kg same-geometry vehicle" in general: flying F's absolute gains on a real 0.5 kg
  body of that geometry would halve the attitude bandwidth. The draft says "×2 gains" in the vehicle definition.
  Wording elsewhere (title, run tag "rv05", the coordinator's phrase) should not drop that condition (L-1).
- **Enabling** the custom drag, disturbances, the IMU (`imu_sensor.py:89` divides force by mass), or a physical target
  using MotorModel would break the equivalence or the wrapper's scope. None is on in this contract.
- **Testing:** the closed-loop Lee scaling is not tested. The 2× wrench is fed to the allocator test as an analytic
  assumption, which the draft states honestly (L-2).

## 2. Thrust factor with a controller that does not know it

**As domain randomization it is sensible:**
- The env factor scales thrust and yaw torque together. It therefore equals a motor-constant change, or a body whose
  m and J both scale by 1/f. It is not a CoM payload change, which would leave J nearly unchanged; the draft says this.
- The per-motor ±5 % leaves a steady attitude error. Calculated: at most ≈0.9° roll/pitch, ≈0.15° yaw. The resulting
  velocity error is ≈0.06 m/s, negligible.

**But the altitude hold degrades, and the draft overstates it (M-1).** Hover never fails. There is no saturation:
the steady per-motor demand at f = 0.8075 is 0.76 N against a 1.38 N cap. The floor (0.1 m) is not reached at rest.
The probe (`probe_altitude_sag.json`) models the task PI (P 4, I 1 s⁻¹, |I| and |vz_cmd| ≤ 2.5 m/s, updated every
0.1 s), the Lee vertical loop (K_vz 2.5) and the use_rps motor:

| Env factor f | Steady vz command spent on the hover bias | z_min after episode start (from 1.0 m; τ 0.03–0.06, motor state 0–cap) | z after 8 s |
| --- | --- | --- | --- |
| 1.0 (nominal; = F/CAM-K1) | 0 | 0.80–0.996 | ≈1.00 |
| 0.85 | +0.69 m/s (28 % of 2.5) | 0.64–0.77 | 0.97–0.98 |
| 0.8075 (env and all motors low) | +0.94 m/s (37 %) | 0.57–0.68 | 0.96–0.97 |
| 1.2075 (all high) | −0.67 m/s | 0.89–1.00, then +2 % above | 1.02 |

- The integral restarts at 0 every episode (`navrl_task.py:4891`), so weak envs start each episode sagging 0.2–0.43 m
  below the flight altitude, and recover slowly (slow mode ≈3.7 s).
- The policy cannot correct this: the actor's z output is overwritten by the altitude PI (`navrl_task.py:9486`).
- The real vehicle's PX4 has a hover-thrust estimator and rate-loop integrators. In this respect the simulation is
  more pessimistic than the vehicle.

## 3. The depth model

**What it wraps:**
- **The input:** the single `depth` argument of `NavRLPerceptionModule.observe`. That is
  `navrl_detector.render_raw_rgbd`'s depth (`navrl_detector.py:927-974`): the 40×24 obstacle range image, bilinear
  upsampled to 160×90 and clamped to 10 m, with the camera-resolution target-sphere range (and distractors, if any)
  painted over it.
- **So it models both** the obstacle depth and the detection (target) depth.
- **Unchanged:**
  - the RGB, the LiDAR and the detector's own buffers (`target_mask`, `target_depth`, `obstacle_depth`, used by
    evaluation);
  - the segmenter's depth channel: its weight in the default colour rule is 0 (`navrl_perception.py:622-629`), and
    v2-search unsets `NAVRL_DETECTOR_CHECKPOINT` (`:391`).
- **Range semantics are right:**
  - the renderer stores the Warp `t` along normalized rays, which is Euclidean range (`navrl_detector.py:28-43`);
  - the rays are pinhole with `fx = W/(2 tan(hfov/2))` and `cx = (W−1)/2`, the same formula perception uses
    (`navrl_perception.py:758-761`; there is no FOV-scale error in this contract);
  - so `z = range · cos` and `range += n/cos` are correct;
  - using the RGB (66°×54°) intrinsics is right for depth aligned to the RGB camera.
- **The formula** σ_z = z²σ_d/(fB) is the standard stereo propagation.

**The constants** (m-4):
- **fB:** 57.2 px·m follows D-2026-11-06-1's 80° HFOV at 800P. Luxonis' depth documentation uses a rectified focal
  length of 882.5 px at 800P (HFOV ≈ 71.9°), which gives fB 66.2 and σ 14 % lower. Read the unit's calibration.
- **MinZ:** for fB = 57.2, MinZ = fB/95 px = 0.60 m in standard mode and 0.30 m with extended disparity (the survey's
  own calculation). The user's 0.4 m corresponds to neither, so the mode is unstated.
- **Holes:** iid pixel holes at p = 0.02 do almost nothing to the obstacle map. The fusion takes the minimum over the
  26–90 rows a full-height bar covers in one column (`navrl_perception.py:1910-1921`). The holes act only on target
  pixel counts.

**Interactions with "invalid → no return (20 m)"**, checked in the code and the probe:
- **The detector mask** needs `depth < max_camera_range` (`navrl_perception.py:1601`). Invalid target pixels leave it.
- **MinZ** removes target pixels from centre distance 0.65 m on-axis, and all of them by 0.55 m. Capture is at 0.5 m,
  so the target is lost for at most about one 0.1 s step before capture (L-3).
- **Noise at the far plane:** noisy target pixels clamp to 20 m and leave the mask. The drop probability per pixel is
  0.9 % at 18 m, 15 % at 19 m and 31 % at 19.5 m. "MaxZ 20 m keeps the target's camera range unchanged" holds to about
  18 m (L-3).
- **Obstacles:**
  - Invalid obstacle pixels at 20 m, at or above the 10 m far plane, read as LiDAR no-return.
  - A bar with z < 0.4 m disappears from the scan entirely (the test confirms this). Under `no_lidar` it also
    disappears from the riskcap governor's clearance (`navrl_sensor_ablation.py:131-148`, m-1).
- **Upsampled phantoms:** the 40×24 → 160×90 phantoms (EQ-1D) are noised per high-resolution pixel, as if native.
- **Column-minimum bias:** the bias is a calculation, not INFERRED (L-4): −0.10 m at 4 m, −0.21 m at 6 m, −0.35 m at
  8 m and −0.52 m at 10 m. It is conservative.
- **Tracker variance:** the KF's σ_r (linear model) is not adapted to the z² noise. It is about 1.5× overconfident
  near 19 m (L-3).

**What changes beyond the observation (m-1):**
- The reward's `visibility_bonus` uses perception's pixel visibility: `self._visible_now = diagnostics["visible"]`
  (`navrl_task.py:6482`), consumed at `:6062-6069`. The depth model therefore changes the reward's input, though not
  the reward function.
- The governor's clearance reads the same modelled depth.

## 4. Install order, composition, signatures, RNG, OFF

- **Order:** vehicle randomization, then the depth model, then the ablation, then the camtrain overrides
  (`realveh_entry.py:184-199`).
- **Composition is safe:**
  - the depth wrapper uses `functools.wraps`, so the ablation's `inspect.signature` binds the original parameters;
  - the depth module refuses installation after the ablation (`navrl_depth_model.py:153-154`, tested);
  - the ablation's outer `observe` calls the depth wrapper, and the two touch disjoint arguments.
- **RNG:** each module has its own generator (CPU for vehicle randomization; the device's for depth). The global RNG
  draws are unchanged in number (tested on CPU).
- **Identity settings are bitwise OFF on CPU:**
  - cap None, factor exactly 1.0 and τ rewritten as the same float32 0.04;
  - for depth: σ 0, p 0, MinZ 0, MaxZ = far plane.
- **The entry with both switches off** is the camtrain path (ablation and overrides) with a different receipt.
- **Not shown:** OFF reproducing native on the GPU. That is not needed for training, but CAM-K2-E will need it.

## 5. Pinned files and the observation contract

- **No tracked file is modified.**
- **No path-contract file is touched.** The new modules sit beside the C-study ablation, and no test enumerates that
  directory.
- **Both modules read only rendered sensor data, intrinsics and robot parameters.** No ground truth target quantity
  reaches the actor. The diagnostic attributes (`_depth_model_invalid`, `_veh_rand_record`) are not in the
  observation.
- **The vehicle factors are not observed.** One option for later: they could be given to the critic.

## 6. Training wrapper

**Verified sound:**
- **Warm start:** SHA-pinned sources (match CAM-K1 TRAINED); the copy is checked before and after.
- **Lineage guard:** untouched; the robot stays navrl_quad.
- **Smoke gate:** SMOKE_OK at the same HEAD.
- **Single-use** output folders.
- **Preflight:** approval naming SCOPE and HEAD, the branch, a clean tree (untracked files included, `gr_run.py:96`),
  the preregistration committed, one RTX 3070 and no other GPU app, no runner or SITL, the disk floor.
- **Run bounds:** `timeout -k 120`; host fault, boot id and CPU state compared after the run.
- **Post-run checks:** the receipt against the preregistered settings and the SHA-256 at HEAD; the
  vehicle-randomization record (cap from the PhysX mass); the checkpoint's epoch and robot identity, read with a
  code-free unpickler; the FOV line; the tracked tree.
- **Tests:** each check has deviation tests and a passing fixture.

**Gaps:**
- **No evidence that the depth model ran during the run (m-2).** Nothing in SMOKE_OK or TRAINED records that the
  depth model was applied to frames, or what it did (invalid fraction, MinZ share), or that resets redrew. The chain
  is structurally sound, so the risk is low, but the record cannot describe the treatment's actual effect.
- **The smoke measures but does not gate the step cost (m-3).** The 2.5 h cap leaves about 1 h margin at CAM-K1's
  4.8 s/epoch. CAM-K1's smoke-based projection was 21 % short.
- **The tree must stay frozen through the whole chain (L-7).** `smoke_ok` requires commit == HEAD and the smoke folder
  is single-use, so any commit between the smoke and seed 2 strands the unit.
- **Disk is thin but sufficient (L-8):** 2.4 GB free now, against an expected ≈0.45 GB for the smoke and both seeds.
  No per-process memory cap is set (as in CAM-K1).

## 7. Tests: non-vacuity and PUBLIC_CPU safety

**Good:**
- every production module needing `aerial_gym` runs in a child process with stub packages;
- in-process loads are private and stdlib-only;
- env and argv changes go through `mock.patch`;
- negative checks come with positive controls (cap from mass; T/W refusal with a 3.26 control; MinZ 0.3/0.5;
  no-model control; seed determinism);
- the equivalence test includes the uncapped quad as a negative control.

**Gaps:**
- the older ablation guard is avoided, not amended (m-5);
- the full PUBLIC_CPU suite has not been run (m-6);
- minor coverage holes (L-9): the partial reset does not assert that the other envs' motor state is untouched;
  `observe_rng_equal` has no control showing the hash can detect consumption; the launcher contract test is
  one-direction only (my two-way diff is clean).

## 8. The draft preregistration

- **Honest and well limited overall:** training only; no ON/OFF claim; the placeholder vehicle; the optimistic
  invalid encoding; PX4 versus Lee; the GPU-untested items; EQ-1D; the shared FOV.
- **To correct or add:** M-1, m-1, m-4, m-7, L-1, L-3, L-4 and L-5. The fixes are under Findings.

## Findings

### MAJOR

**M-1. "The altitude PI absorbs the per-env thrust bias" holds in steady state only.**
- **Evidence:** §2 table, from `probe_altitude_sag.json`. The code it models:
  - `velocity_control.py:32`: the controller uses the nominal mass;
  - `navrl_task.py:4993-5002`: PI P 4, I 1 s⁻¹, bounds 2.5 m/s;
  - `:4891`: the integral is reset per episode;
  - `:9486`: the actor has no vertical authority.
- **Effect:** in weak envs (f 0.81–0.85), each episode starts with a 0.2–0.43 m sag below the 1.0 m flight altitude
  that takes seconds to recover, and 28–37 % of the vertical command authority goes to the bias. That leaves less
  margin against the tilt-induced sag that historically caused floor strikes. The policy cannot compensate.
- **Fix:**
  - replace the sentence with the quantified statement;
  - add it to "Disclosed limits", including that PX4's hover-thrust estimator makes the vehicle less sensitive;
  - add to the training record, descriptively (not a gate), the per-cause terminal shares (below / contact / OOB)
    from the existing `NavRL crashdiag` log lines for CAM-K1's last window and CAM-K2's windows, so that a rise in
    `below` can be attributed;
  - state that CAM-K2-E must read `below` crashes in this light.
- **For the user (optional):** whether to keep TW_SCALE 0.15. A smaller range, or a later hover-thrust-aware opt-in
  controller, would reduce the artefact.

### MINOR

**m-1. The depth model reaches further than "the depth argument of observe".**
- **Evidence:**
  - the reward's `visibility_bonus` reads `diagnostics["visible"]` (`navrl_task.py:6482`, `:6062-6069`);
  - under `no_lidar` the riskcap clearance is computed from the modelled fused scan (`navrl_sensor_ablation.py:131-148`);
  - the tracker sees the noised range.
- **Consequences:**
  - bars with z < 0.4 m vanish from the safety filter too, so it does not brake in the last ≈0.15 s at 2.5 m/s;
  - the noise's low bias makes the filter brake earlier at range;
  - the visibility income drops near 0.55–0.65 m and beyond ≈18 m.
- **Fix:** say this in "Reward, terminations, observation" and in the limits.

**m-2. No runtime evidence of the depth model or the reset redraws.**
- **Fix:**
  - a one-time first-frame line from `apply`, and a line every N frames, with cumulative frame count, invalid fraction
    split by MinZ / MaxZ / hole, and σ at the median valid z;
  - a one-time line from `redraw` at the first reset, with count, factor min/max and τ min/max;
  - post-run, require these lines with plausible values (hole share within ±50 % of HOLE_P; factors inside the
    bounds). This is a positive control that the treatment ran.

**m-3. The smoke does not gate the step cost against the 2.5 h cap.**
- **Fix:** parse the smoke's per-epoch time (excluding startup) and refuse seeds if 1,000 epochs project above, say,
  0.8 × 2.5 h. Otherwise state that a timeout reads TRAINING_FAIL after 2.5 GPU-hours.

**m-4. The depth constants mix modes, and the holes are nearly inert for obstacles.**
- **Fix:**
  - name the assumed DepthAI mode (resolution, extended disparity, subpixel);
  - record that MinZ 0.4 m is the user's figure, while fB 57.2 implies 0.60 m (standard) or 0.30 m (extended);
  - note the 882.5 px (fB 66.2) alternative and that the unit's calibration decides;
  - disclose that iid p = 0.02 holes affect target pixel counts only. The survey's column/blob holes were not built.

**m-5. The older ablation import guard is avoided rather than amended.**
- **Evidence:**
  - `realveh_entry.py:193-197` uses `importlib.import_module` because the guard in
    `tests/test_navrl_sensor_ablation.py:554-571` matches import statements only; the comment says so;
  - CAM-K1's precedent amended the allow-list (evidence README, Validation).
- **Fix:** add `realveh_entry.py` to that allow-list, extend its pattern to `import_module` (or reuse
  test_realveh's), and add a positive control.

**m-6. Required before commit, not yet done** (IMPLEMENTATION.md:141, 528, 564):
- the full PUBLIC_CPU suite;
- the launcher preflight (`NAVRL_PREFLIGHT_ONLY=1`), as CAM-K1 did;
- the `docs/EXPERIMENTS.md` family entry (`tests/test_experiment_index.py`);
- resolving the other agent's untracked `sitl_s4f1` files, which block the clean-tree preflight.

**m-7. The source lineage is weak and unevaluated, and three treatments are bundled.**
- **Evidence:**
  - CAM-K1's last training windows read 26.3 % / 25.6 % capture and 73.7 % / 74.5 % crash
    (`docs/evidence/camonly_2026-11-04/camk1/README.md`);
  - CAM-K1-E has not run;
  - D-2026-11-06-1 lists the vehicle, the motor randomization and the depth model as "later, separately
    preregistered units"; CAM-K2 bundles all three without an ON/OFF arm.
- **Fix:** state the source's training statistics and the missing CAM-K1-E in "Why" and "What does not follow", and
  add "confirm bundling the three treatments" to the user's decisions.

### LOW

- **L-1. Wording and limits of the equivalence.**
  - Add the absolute 0.05 N contact-detection threshold (`env_manager.py:403-407`).
  - Add the condition "Option A has ×2 Lee attitude gains (uniform-density body)".
  - Add that the zero custom drag and disabled disturbance are preconditions.
  - Say "equivalent to the Option A placeholder under these conditions", never "a 0.5 kg same-geometry vehicle".
- **L-2.** The closed-loop Lee scaling is analytic only. An optional CPU test could instantiate the velocity controller
  with both masses, inertias and gains (it needs pytorch3d).
- **L-3. Quantify in the limits** (from `probe_depth_effects.json`):
  - target lost from the mask at centre distance 0.55–0.65 m, with capture at 0.5 m;
  - far-plane drop 0.9 % / 15 % / 31 % per pixel at 18 / 19 / 19.5 m;
  - KF σ_r not adapted, about 1.5× overconfident near 19 m.
- **L-4.** Replace "the 90-row column minimum biases iid noise low (INFERRED)" with the calculated bias: −0.10 / −0.21
  / −0.35 / −0.52 m at 4 / 6 / 8 / 10 m.
- **L-5. Wording.**
  - "TW_SCALE 0.15 and MOTOR_IMBAL 0.05 are the largest ranges…" is one point on a frontier (0.16 / 0.04 also passes).
  - The 1.8 floor has no stated source.
  - `navrl_depth_model.py:28` calls the invalid convention a "user decision", while the draft lists it as open.
  - Worst-case saturation: at T/W 1.82 a 45° tilt leaves 0.40 g, not 0.84 g.
- **L-6. Checkpoint provenance.** CAM-K2 checkpoints' `env_state` does not record the cap, the randomization or the
  depth model, so by metadata they are indistinguishable from the CAM-K1 contract. Write a sidecar manifest
  (checkpoint SHA ↔ receipt SHA ↔ settings) and have CAM-K2-E bind it.
- **L-7.** State in the draft that nothing is committed between the smoke and seed 2. The records follow after the
  chain.
- **L-8.** Thin disk (2.4 GB free; 1.0 GB floor) and no memory cap. Acceptable; log `df` during the run as CAM-K1 did.
- **L-9. Test gaps:**
  - assert that the partial reset leaves the other envs' motor state untouched;
  - give `observe_rng_equal` a positive control;
  - make the launcher contract test two-way.
- **L-10.** HASHED omits the controller code (`velocity_control.py`, `base_lee_controller.py`, `base_controller.py`),
  `navrl_detector.py`, `robot_manager.py`, `navrl_task_config.py`, the launcher and the wrapper. The clean tree and the
  approval at HEAD bind them anyway; adding them is only for documentation.

## What may be said after the runs, if both seeds read TRAINED

- **May be said:** "CAM-K2 v1 seeds 1 and 2 were trained for 1,000 epochs from CAM-K1 v1 under:
  - the T/W 2.25 cap, equivalent in this simulator to the Option A placeholder (2× masses and inertia, 2× Lee
    attitude gains);
  - ±15 % / ±5 % delivered-thrust randomization and τ 0.03–0.06 s;
  - the OAK-D Pro depth model as configured."
- **Training-rollout statistics:** descriptive only.
- **Not to be said:**
  - any capture, crash, robustness or transfer claim;
  - any claim that a treatment helped;
  - that the vehicle is the real one;
  - that the depth model reproduces the OAK-D Pro beyond its stated constants.
- **All of those wait for CAM-K2-E,** with CAM-K1 as the baseline under the same settings and an OFF-reproduces-native
  check on the GPU.
