# CAM-S1 design review r1 (camera-only from-scratch training)

**Verdict: AMEND_FIRST.** The draft code does what it says. Two design premises do not hold for this run as
written: F's curriculum gates, and pausing the run for evaluations. The flight-runtime path cannot use a CAM-S1
checkpoint, and no CAM-S1 checkpoint can safely be flown tomorrow (section F).

- Reviewer: Claude Code subagent, Opus 5.5 (claude-opus-5-5), fresh context. Worked read-only. Written 2026-11-23.
- Inputs: worktree `MOTAR-camonly`, branch `camonly/dev`, HEAD `38ab630`. SHA-256 of both draft files matched
  `INPUT_SHA256`: `navrl_camsafe.py` 75581b52…, `tools/realveh_entry.py` 4b8d8ee5…. I also read three untracked
  drafts that appeared at 15:21–15:23 and are not in the pinned inputs: `tests/test_camsafe.py`,
  `tools/cams1_train.py` and `aerial_gym/rl_training/rl_games/train_navrl_cams1.sh`.
- No GPU was used and the repository was not written. Scratch probes are in `scratch/` next to this file.
- `tests/test_camsafe.py` ran as a script on CPU: 12/12 OK. Wrapper-order probe (`scratch/probe_order.py`), using the
  real `navrl_sensor_ablation.install_governor` on a fake task:
  - ablation then camsafe: output norms are 3.0 for an in-view command and 1.0 at 90°. The 45° diagonal (3.54 m/s)
    is cut to 1.0.
  - Reversed order raises `KeyError: 'directional_lidar_clearance'`, so the install guard is necessary and correct.

## Findings

### No P0 for the training run itself. Flight blocker P0-F (blocks flight use of any CAM-S1 checkpoint, not the run)
- **The runtime cannot load a CAM-S1 checkpoint.**
  - `tools/camera_policy_contract.py:150-156` pins only the CAM-K2 v3 checkpoints, reports and manifests.
  - `:196-206` requires `checkpoint_epoch == 29000`, and `:235` hard-codes lineage `CAM-K2-v3`.
- **The cap would not exist in flight.**
  - `CameraPolicyRuntime.step` (`tools/camera_policy_runtime.py:1294-1304`) applies riskcap and builds
    `previous_action` from `governed`, with no blind cap. Flight would see uncapped blind commands and a different
    prev-action than training.
  - The camsafe docstring (`navrl_camsafe.py:8-9`, "identically, in the flight runtime") is false today.
- **P9 would turn on in flight.**
  - The runtime copies the whole training `env_navrl` into `os.environ` (`camera_policy_runtime.py:1068-1072`;
    contract `:241`).
  - It then builds perception from `tc.perception` (`:1094-1095`), whose P9 fields come from `NAVRL_P9_*`
    (`navrl_task_config.py:387-391`).
  - P9 has no `training` gate (`navrl_perception.py:954-981, 1646-1671`). It would inject synthetic NO_LOCK on real
    detections, or fail if the JSON path is missing on the Orin.
- **The flight detector cannot see the target.**
  - The detector is `bootstrap_red` (`camera_policy_contract.py:255`). Per D-2026-11-18-2, an unmarked drone is not
    detected.
  - Qualification is NOT_TESTED on CPU, Gazebo and hardware (`:258`).
- **Fix:** do not fly CAM-S1 tomorrow (section F). Before any runtime use:
  1. a new contract lineage with its own pins;
  2. `navrl_camsafe.cap_blind` after `apply_speed_governor`, with `prev` built from the capped command;
  3. an explicit allow-list that drops `NAVRL_P9_*` from `runtime_env`;
  4. tests for each.

### P1-1 F's curriculum gates are a LiDAR ceiling: the run will very likely spend its whole budget at 70 bars
- **The gate values are tuned to F's LiDAR ceiling.**
  - The 70-bar gate of 0.82 was chosen just under F's measured 70-bar ceiling of 0.843 (camera + LiDAR;
    `train_navrl_v2_search.sh:314-328`).
  - The statistic is the training-rollout capture over 16,384 episodes, with exploration noise and 30 % detection
    dropout (`:392`).
- **F's own timeline, with LiDAR:**
  - 70→85 promoted at a 3,313-epoch dwell with 0.832 (WORKLOG.md:4718-4725).
  - 85, 100 and 115 took 1,360, 1,386 and 1,406 epochs; 130 took 2,080 with holds (WORKLOG.md:4493-4497).
  - 205 was reached only around ep 19,100 (WORKLOG.md:5426-5428, 5789).
- **Camera-only rollout records are far below the gate.**
  - CAM-K1 rollout capture at 205 bars reached 26.3 / 25.6 % after 3,000 camera-only epochs (crash 73.7 / 74.5 %;
    `docs/evidence/camonly_2026-11-04/camk1/training_curve.json`).
  - CAM-K2 v3 reached 26.7 / 28.5 % after 1,000 more (`camk2/training_curve_v3.json`).
  - GR-1 ctrl with LiDAR sat at about 80 % in the same rollout statistic.
- **Camera-only evaluations at 70 bars are also far worse than F.**
  - K1 crash is 7.18 / 7.67 % against F's 1.24 % (`camk1e/analysis.json` `per_density_rate_pct`), about 6×.
  - Passing 0.82 needs a rollout crash of about 16 % or less, which is F's LiDAR level. No camera-only record
    supports that.
- **Consequence:** there is no max-dwell fallback (the knobs are listed in `navrl_task_config.py:555-580`). The
  curriculum holds at 70 bars until MAX_EPOCHS, so every checkpoint is a 70-bar policy.
- **The budget lever has little effect.** Halving `DENSITY_MIN_EPOCHS` only shortens stages that pass at the first
  check after the dwell (F's 85, 100 and 115). It does nothing for a stage that never passes.
- **The run has no stop rule.** Nothing ends it early if learning fails.
- **Fix:** preregister one of the following, plus a stop rule.
  - (a) A calibration window at 70 bars. Run until two consecutive 16,384-episode windows agree within 1 pp, as F's
    0.816 / 0.837 calibration did. Then set threshold = plateau − 2 pp, and scale the later knots by the same ratio.
  - (b) A fixed lower schedule, stated as a guess.
  - (c) Explicitly accept a 70-bar-only policy.
  - Stop rule, for example: stop if rollout capture is below 0.3 at ep 3,000 (number to be fixed in the prereg).

### P1-2 "Pause training for evaluations" is not implementable as drafted
- **Concurrent evaluation does not fit.** Training uses about 6.3 GB of 8 GB (`train_navrl_v2_search.sh:35`).
  SIGSTOP keeps that memory, so an evaluation cannot run beside it.
- **A restart cannot resume.**
  - `train_navrl_cams1.sh:32,81` unsets `CKPT` and forces `NAVRL_V2_ALLOW_RESUME=0`.
  - Fresh mode refuses any checkpoint (`train_navrl_v2_search.sh:201-224`), so a restart would be a new run.
  - F's 2026-08-02 interruption needed SIGINT plus a dedicated recovery launcher (WORKLOG.md:5423ff).
  - A resume also restarts the episode-statistics window and the P9, depth and vehicle RNG streams.
- **Fix:** preregister either evaluation after the run, or a resume procedure. A resume procedure needs a
  continuation launcher with `--checkpoint … --branch_run`, and must restore the density state and state the RNG
  break. Alternatively, evaluate checkpoints on the GTX 1650 Ti host and never pool them with RTX 3070 results.

### P1-3 P9 here is not "a real drone detector's statistics": the size bins are mis-scaled and latency vanishes
- **The pixel units do not match.**
  - The bins are `sqrt(GT area)` in NPS original pixels (`results/perception_p8_2026-09-09/README.md`), with edges
    8/16/32/64 and fallback `[1,1,2,3,3]`.
  - In the 160 × 90, 66° detector image (fx 123.2 px; 0.15 m sphere; `navrl_task_config.py:183-184,206`), the target
    is under 8 px beyond 4.62 m. The bins switch at 2.31 m (16 px), 1.15 m (32 px) and 0.58 m (64 px).
  - So from 2.31 m to the 20 m detector range, every frame gets bin-1 statistics. Bin 1's stationary split is HIT
    0.635, FALSE_LOCK 0.229, NO_LOCK 0.137 (`scratch/probe_numbers.py`).
  - The model was calibrated on full-resolution NPS frames. A real detector at OAK-D resolution would put mid-range
    targets in bins 2-3, which are better. At more than about 17 m (raw bin 0, no NPS support) it would likely do
    much worse than bin 1.
- **There is effectively no latency.** Over 99 % of latency samples round to 0 control steps at 10 Hz (shares
  rounding to one step: 0.005 / 0.001 / 0.002; `navrl_empirical_error.py:200-202`).
  - P9 is mutually exclusive with `NAVRL_DETECTION_LATENCY_S` (`navrl_perception.py:961-962`).
  - The design therefore trains at about zero latency, although CAM-T1 d1 (100 ms) read **WORSE** in both seeds.
- **False positives are not modelled.**
  - NO_LOCK stacks on the i.i.d. 30 % dropout: dropout is applied first (`:1640-1641`) and P9 eligibility is taken
    after it.
  - FALSE_LOCK keeps the clean range and a sub-pixel offset. Its only effect is that the target stays in the
    obstacle map (`:1672-1677`).
  - With the target out of view, no false positive is ever produced.
- **What P9 can and cannot do:**
  - It is a mild, temporally correlated detection-loss perturbation; it cost F 4.57 pp (EVIDENCE.md row "Perception
    → policy").
  - It does not make the target "recognized as a drone". The policy never sees pixels; that is a runtime detector
    question (P0-F).
- **Fix:** keep P9 if wanted, but word it in the prereg as "NPS-calibrated detection-loss model, bin-1 statistics
  beyond 2.3 m, latency ≈ 0". Do not claim drone recognition. The launcher comment "from CAMS1_P9_FROM"
  (`train_navrl_cams1.sh:13-14`) describes nothing that exists, since only on/off is implemented.

### P1-4 The full treatment stack has never run together: smoke it with non-vacuous counters before launch
- P9 has run only in F's LiDAR configuration (P10). It has never run together with the OAK-D depth model, the
  `blob_near` validity rule, the `no_lidar` ablation, vehicle randomization and camsafe.
- **Fix:** run `cams1_train.py --smoke` on the committed tree, and require the following in its log or receipt:
  - a positive P9 eligible count;
  - a positive camsafe `capped_frac` and `penalised_frac`;
  - `cfg_reward_parameters.collision_penalty == -40` in the checkpoint;
  - every module SHA present in the receipt;
  - a measured s/epoch, then re-check that 12,000 epochs fit the 17 h wall cap (`cams1_train.py:31-32`). At
    CAM-K1's 4.83 s/epoch that is already 16.1 h, about 5 % margin. The P9 per-bin loops and the camsafe
    `.item()` syncs add time that has not been measured.

### P2
- **P2-1 The 4a rationale skips the trained-policy evidence.**
  - The docstring (`navrl_camsafe.py:4-6`) cites F zero-shot evidence: C2 out-of-FOV, G3, 51.8 %.
  - For a trained camera-only policy, CAM-T1 Stage A reads DEFINITION_SENSITIVE: the footprint profile puts 67.7 %
    of crashes IN_VIEW and 10.8 % BLIND.
  - In every VALID crash the camera was closer than MinZ 0.4 m (median 0.22 m).
  - Lower free caps did not help, and blind commands were already held at 2.0 m/s by the ablation governor
    (`navrl_sensor_ablation.py:181-183`).
  - Fix: call 4a a flight-safety rule ("neither justified nor forbidden" by CAM-T1), not a crash lever. Note that
    near-field MinZ blindness is untouched.
- **P2-2 4a is a hard cliff in the action-to-command map.**
  - Inside the FOV the fastest speed is 2.98 m/s, at ±33° (per-axis 2.5 m/s clamp, riskcap free 3.54).
  - Just outside the FOV a command drops to 1.0 m/s; the former fastest 45° diagonal goes from 3.54 to 1.0 m/s.
  - It also limits a sideways dash at the capture moment to 1.0 m/s.
  - This is learnable. A bearing taper (for example linear over hfov/2 to hfov/2 + 15°) would be gentler. Whatever
    shape is chosen must be identical in the runtime.
- **P2-3 4b mostly duplicates the existing crab penalty.**
  - Per step at |v| ≥ 1 m/s, `yaw_align` 0.6 against camsafe 0.5 gives 0.088 / 0.041 at 45°, 0.300 / 0.194 at 90° and
    0.600 / 0.500 at 180°.
  - Both are speed-gated and ≤ 0, so neither opens a standing income. They mostly reward slow-then-turn.
  - The reward-hacking risk is low. Watch mean speed and the timeout share.
  - 4c (−40) restores the config's own B3 invariant, which the 600-step contract had broken: a timeout costs −30,
    worse than F's −20 crash (`navrl_task_config.py:776-779`, `train_navrl_v2_search.sh:277`). It follows from
    the code's rationale, not from camera-only evidence.
- **P2-4 The goal-distance and FOV curricula run on the epoch clock.**
  - The k-ramp runs to ep 5,000 and the FOV widening to ep 3,000 (`navrl_task_config.py:241-250, 500-531`). Both
    were tuned with LiDAR.
  - The config itself records the failure mode of a sensor-limited run that cannot keep up (`:522-529`).
  - Cover this with the P1-1 stop rule, checked at about ep 3,000.
- **P2-5 Latency is the one transfer risk the evidence points to, and the design does not train it.**
  - Evidence: CAM-T1 d1 read WORSE (+4.95 / +3.67 pp), and G7 read TOLERANT for F.
  - Consider 0–1-step action-delay randomization later. The eval-only hook `tools/actionhold_hook.py` exists. It is
    a new treatment and needs its own review.
- **P2-6 One seed.** No seed-general wording; P10's three seeds disagreed. "Better than CAM-K2 v3" needs a
  preregistered evaluation under the same treatments.

### P3
- **The alive mask reads stale truncations.** It uses `self.truncations` (`navrl_camsafe.py:138`), but `step()`
  sets truncations after the reward call (`navrl_task.py:5063` then `5065-5073`).
  - Effect: the penalty is skipped on the first step after a truncation-reset and applied on the 600th step.
  - This is harmless, but differs from `add_static_safety_reward`'s semantics.
- **The statistics are cumulative and add syncs.**
  - The camsafe printout (`:115-119`) shows cumulative averages since start, not windowed ones.
  - Each step adds 5 `.item()` host syncs. `rw_calls` is never printed and `_report(self)` ignores `self`.
  - The governor's own diagnostics (`_speed_governor_last`, intervention rate) cannot see the cap.
- **Other correctness points, all fine:**
  - `perception.hfov` is in radians (`navrl_perception.py:754`).
  - The gate uses `yaw_align_speed_ref` = 1.0 m/s.
  - dtype and device are consistent; the reward wrapper returns the original's `None`.
  - Terminal envs keep the −40 overwrite (`navrl_task.py:6298-6304`).
  - `self.rw` picks up −40, because it is copied at task build (`:1273-1274`).
- **The docstrings cite things that do not exist yet.** They cite D-2026-11-23-1 and
  `docs/plans/prereg_camonly_s1_scratch_2026-11-23.md`, neither of which exists. `cams1_train.py` correctly refuses
  to run until the prereg is committed.
- **Test gap.** The tests use only a fake task class. Add one test of the real `install_governor` plus camsafe
  composition, like `scratch/probe_order.py`, so the order is pinned.

## A. From scratch vs continuation; the 898-D layout
- **From scratch is a legitimate research lineage.**
  - It drops F's LiDAR-shaped value function, its input normalizer statistics and its 87° habits.
  - The records do not show that the continuation's remaining crashes are inherited habits: CAM-T1 is
    DEFINITION_SENSITIVE, the footprint profile is IN_VIEW-dominant, and the near-field crashes sit under MinZ.
  - Against it: F needed about 19,000 epochs to reach 205 bars with LiDAR. A camera-only scratch run of
    12,000–15,000 epochs will very likely end weaker in simulation than CAM-K2 v3 (29,000-epoch lineage; pooled
    crash 13.3–13.7 %).
- **For a near-term flight, continuation (or v3 itself) dominates.** Combined with "fly tomorrow", from scratch is
  self-defeating. It is not unsafe as a training run.
- **Keeping 898-D is sound.**
  - From scratch, the constant out-of-FOV slots (about 59 of 72 bins × 4 rows, plus the LiDAR association fields)
    normalize to about 0 and cost almost nothing.
  - The runtime already builds exactly this layout (`camera_policy_contract.py` observation `dim 898`,
    `lidar_storage_shape [1, 288]`), so reuse needs contract changes only, plus the cap and the P9 exclusion (P0-F).

## B. Feasibility within the budget
- **Implausible as designed.**
  - The 0.82 gate at 70 bars is F's LiDAR ceiling minus 2 pp.
  - Camera-only rollout capture is 26–28 % at 205 bars, and camera-only eval crash at 70 bars is about 6× F's
    (P1-1).
- **What happens if the gate is never passed:** a silent hold at 70 bars until MAX_EPOCHS, with no fallback. The
  same-density capture guard can stop the run only after a 0.25 drop from a peak of 0.50 or more.
- **Change before launch:**
  - recalibrate the gates (P1-1 a, b or c) and add a stop rule;
  - keep the 500-epoch dwell; it is harmless.
  - P9 and depth noise from epoch 0 are not the main obstacle. Missing out-of-FOV obstacle information is, with
    exploration noise driving blind motion; CAM-K1's 74 % rollout crash fits that, though it is not shown causally.
  - Starting P9 later changes little.
  - The 4a cap may lift rollout capture by slowing noise-driven blind motion. That cannot be predicted, which is a
    further reason for the calibration window.

## C. Do 4a, 4b, 4c and P9 follow from the evidence?
- **4a** is a flight-safety choice, not evidence-derived (P2-1). It does not conflict with riskcap: it runs after
  the governor, keeps the direction and uses the ablation's own FOV predicate. It removes the 45° diagonal advantage
  and adds a cliff (P2-2).
- **4b** is collinear with `yaw_align`, with low hacking risk. **4c** follows from the B3 invariant (P2-3).
  **P9:** see P1-3 and D.
- **Code:** the wrapper order is correct, enforced and probed. The alive mask is right for terminal envs and stale
  only on truncation steps. Units, gate, dtype, device and return value are correct. The statistics are cumulative
  (P3).

## D. P9 validity for this camera
Mostly fallback. Every frame beyond 2.31 m, the bulk of a 6–28 m search-and-approach, uses the 8–16 px NPS bin.
- The policy learns NPS bin-1 loss statistics at every range from 2.31 m out. That is pessimistic at mid range for a
  full-resolution detector and optimistic at long range, where real detectors miss targets under 8 px.
- Latency is about 0, and there are no out-of-view false positives.
- **Meaningful fix:** map size by angular size to the flight detector's input resolution. That is a change to
  byte-pinned perception or a wrapper, and needs review. Otherwise, state the limits plainly.

## E. Sim-to-real gaps that remain, and inconsistencies this design adds
- **Remaining gaps:** the red-rule detector against an unmarked drone (D-2026-11-18-2); outdoor sky and phantom
  walls, with no outdoor GO (D-2026-11-13-1, D-2026-11-15-1); near-field MinZ blindness; thin branches against
  simulated bars; the VIO sharing the OAK-D, with the policy's streams unknown; unmeasured end-to-end latency (d1
  WORSE); PX4 offboard tracking against the Lee controller; T/W 4.0 INFERRED; a piloted target instead of the legacy
  motion pattern. The real OAK-D is untested: the 2026-11-21 WORKLOG has the desk test pending.
- **Added by this design:**
  - the cap is missing from the runtime;
  - P9 leaks into the runtime through `runtime_env`;
  - the contract pins only CAM-K2 v3 (all in P0-F);
  - any PX4-side speed limit (for example MPC_XY_VEL_MAX) must not differ from the trained caps.

## F. Flying tomorrow with a CAM-S1 checkpoint
- **What will be verified by tomorrow:** at most, rollout statistics of an unevaluated checkpoint after about
  12–16 h, most likely still at 70 bars.
- **What will not be verified:**
  - any preregistered sim evaluation, or a comparison with CAM-K2 v3;
  - runtime loading: the contract refuses the checkpoint;
  - the blind cap in the runtime;
  - P9 exclusion from the runtime;
  - CPU replay, SITL, or a real OAK-D desk test;
  - outdoor obstacle detection: no GO is recorded;
  - drone detection: the red rule cannot see an unmarked drone;
  - VIO stream integration and latency;
  - the airspace check required by D-2026-11-18-1.
- **Plain statement:** an autonomous closed-loop flight with a CAM-S1 checkpoint tomorrow is unsafe and unsupported
  by any record. It is also impossible without code changes that have not been made or reviewed.
- **What tomorrow can safely be:** manual or recording flights (data for the desk test, detector and VIO streams)
  under the licensed pilot, under the existing rules.

## G. Recommended changes, in order of importance
1. Do not fly CAM-S1 tomorrow. Use the slot for recording or manual data collection.
2. Prereg: replace F's gates with a camera-only calibration rule or an explicit 70-bar-only acceptance. Add a
   futility stop rule at about ep 3,000 (P1-1, P2-4).
3. Prereg: decide how checkpoints are evaluated, either after the run (or on the 1650 Ti host, not pooled) or with a
   specified resume procedure (P1-2).
4. Reword P9 (detection-loss placeholder, mis-scaled bins, latency ≈ 0) and drop "recognized as a drone" (P1-3).
5. Smoke the full stack with non-vacuous counters and a measured s/epoch before launch (P1-4).
6. Before any runtime or SITL use: new contract lineage and pins; the cap in `CameraPolicyRuntime.step` with the
   matching prev-action; `NAVRL_P9_*` excluded from `runtime_env`; tests (P0-F).
7. Reword the 4a rationale using CAM-T1. Optionally taper the cap (P2-1, P2-2).
8. Minor: windowed statistics, fewer syncs, truncation-mask note, an order-pinning test (P3).
