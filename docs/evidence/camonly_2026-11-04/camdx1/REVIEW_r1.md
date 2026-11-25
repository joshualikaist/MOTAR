# CAM-DX1 package — independent review r1

- **Reviewer:** Claude Opus 5.5 (independent, fresh context), read-only. No repository file edited, no GPU, no commit/stash.
- **Date:** 2026-11-25. **Worktree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b`, branch `camonly/s2b`,
  HEAD `7dd73ec`, uncommitted package (prereg, `navrl_inview_start.py`, `realveh_entry.py` diff, runner, analysis, tests,
  EXPERIMENTS/WORKLOG diffs).
- **Criteria audited against:** `docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md` (uncommitted, so fixable in
  place before commit), AGENTS.md research rules.

## Verdict: GO_WITH_FIXES

No P0. Three P1s, each cheap and pre-commit (the prereg is not yet committed, so they can go into its body now; update
the WORKLOG sentence "copied unchanged" accordingly). P2s are wording/limits or optional hardening.

## What I ran (CPU, cores 6,7,12,13, nice 10, `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`)

| command | result |
| --- | --- |
| `python -B -m unittest test_camdx1` (from tests/) | **34 tests OK, 0 skipped**, 0.98 s |
| `tools/camdx1_eval_run.py check --out …/review_r1/check_out` | exit 2; problems = only "uncommitted/untracked" + 5 "not committed"; FT checkpoint found at pin f2c80fcc…; branch camonly/s2b; no approval file (expected). Nothing written (check_out not created). Output: `check_stdout.json` |
| `tools/camdx1_eval_analyse.py controls …/review_r1/controls.json` | exit 0; all 5 controls read as required: identical → NDC/NDC (CI ±1.75 pp); +8 → MATERIAL_GAIN (8.01 [6.25, 9.78]); +2 @20k → SMALL (2.00 [1.60, 2.40]); −8 → MATERIAL_LOSS; S2a-E YC12k−S12k@205: capture −11.13 [−13.54, −8.56], crash +16.11 [+13.67, +18.55] → MATERIAL_LOSS/MATERIAL_LOSS |
| host fault count (`journalctl -k -b 0`) | 1 (unchanged during my runs); online CPUs 0-3,6-23 |

The full PUBLIC_CPU suite was **not** run by me (see P2-8).

## Audit 1 — visible start inside the real task: PASS (mechanism), with a weak control (P1-1)

Traced in the byte-pinned `navrl_task.py`:
- `reset_idx` (4701): `_randomize_general_drone_spawn` (2353, uniform yaw from global RNG) → target sampled by
  `_sample_general_target` (2673; uniform area, radial accept `[min,max]`) → `self.target_position[env_ids] = goal` →
  `self._align_general_spawn_yaw_to_target(...)` (4840, attribute lookup, so the class-level wrapper is used) →
  `robot.update_states()` + `IGE_env.write_to_sim()` (4841-4846, general-spawn mode). `update_states` recomputes
  `robot_vehicle_orientation` from `robot_orientation` (base_multirotor.py:289).
- The camera render uses `obs_dict["robot_vehicle_orientation"]` (`_process_obs_perception`, 6459-6461), called from
  `get_return_tuple` after `reset()` (4695-4699) and after mid-run resets in `step()` (5170-5175). So the wrapped yaw is
  the pose of the first rendered observation, for initial and per-env mid-run resets alike.
- Evaluation saturation: bulk eval forces `_eval_full_distribution` (230-237) → `_fov_curriculum_saturated` True (270) →
  original aligner is a no-op and consumes no global RNG; the wrapper uses its own `torch.Generator` (inview 186-191).
  OFF: realveh settings unchanged and module not loaded (tests `EntryHook`, additive-only diff vs 335a1fa).
- Target placement is not moved by the wrapper; only `NAVRL_GENERAL_GOAL_DIST_MAX` changes it (also used only in
  reporting strata 8459/8474 and the TASK-DISTRIBUTION warning). Target motion/route planning do not read drone yaw.
- Observation contract: the yaw is scenario construction from GT (as the trained curriculum aligner already does);
  nothing new enters the actor. The recorder reads `detector.target_mask` (GT) after the observation is written and
  returns nothing to the task. No leak found.
- No conflicting wrappers: only `navrl_cams2` wraps `reset_idx` (calls through); nothing else touches the aligner.

## Audit 2 — 624 × 468 arms

- **Same FOV: true.** HFOV/VFOV come from `NAVRL_CAMTRAIN_HFOV/VFOV_DEG` (66.7/52.6, unchanged); 624/468 = 312/234 = 4:3;
  fx, fy double; obstacle render keeps its own 40×24 intrinsics. Camera = detect, so not decoupled (D8 equality holds).
- **Not only pixel density** (P2-1): (a) the camera→scan fusion samples the bilinearly upsampled obstacle depth at
  `camera_u = round(… × (W−1))` (perception 1917-1919; upsample detector 927-932), so column samples move by
  sub-source-pixel amounts and the target blank mask is finer → obstacle input is approximately, not exactly, unchanged;
  (b) KF measurement variance changes: `sigma_r = … + 0.15/√px` (2229), `sigma_lat = 0.03 + r/detect_fx` (2231);
  (c) confidence saturates at `min_pixels×4 = 8 px` (1733) at longer range; (d) P9 draws by `size_px` (1669) shift
  bins (its own generator). These are what a finer camera would change *as modelled*, but the prereg sentence "obstacle
  information is unchanged" is too strong and the OOD list should name (b)-(d).
- **OOM fallback:** episode count preserved (64×16 = 1,024), arms homogeneous (624 cells done at 128 rerun at 64,
  `run_one`/eval loop 899-…; `eval_cells` takes only final-mode records). Comparability caveat P2-2. Memory estimate:
  the config comment's ~84 B/px downstream cost gives ~3.1 GB at 128 × 624 × 468, so OOM is plausible; the smoke will
  tell.

## Audit 3 — runner checks

- Env refusals non-vacuous: all 12 eval cells (and every smoke/identity/fallback env) accepted with a counted positive
  control; 12 foreign/missing/cross-arm cases refused (`test_foreign_and_missing_keys_refused`). Snapshot and receipt
  checks reject keys an arm must not set. MISMATCH allowlist per arm is exact-regex and arm-scoped (camdx1 467-470).
- Identity, host-fault (per cell and per stage: fault count, boot id, online list), stage linking, approval/scope SHA,
  allowlisted diff since 335a1fa (navrl_task/perception not allowlisted → byte pins enforced) — all present.
- **Smoke controls:** visible-start control is self-referential (P1-1); the 624 control can pass on
  `NO_MATCHED_FRAMES` (P1-2).
- Wall cap: observed CAM-S2a-E cells at 205 bars took 360-483 s; 12 cells at 128 envs fit 3.5 h comfortably unless 624
  is ≥ 3× slower. Fallback worst case (switch late in eval: up to 5 reruns + 6 cells at 64×16) can approach the cap
  (P2-4). CELL_TIMEOUT 2400/3600 s is ≥ 5× realistic.

## Audit 4 — analysis: implements the prereg readings

Per-block weighted differences, mean over 3 blocks; one `default_rng(4511)`, 10,000 multinomial draws per cell in the
fixed order; all effects on the same resamples; percentile 95 %; ±5 pp; crash sign inverted; INTERACTION unlabelled;
NOT_RUN_OOM → START = V312−R312 flagged. In-quota rows only; recorder joins required (missing rows → binding fail).
Controls meaningful for START and the label rule; DET/INTERACTION weights only checked at zero shift (P2-6).

## Findings

### P1 (fix before commit)

**P1-1 Visible-start positive control is tautological.** `start_bearing_deg` (inview 257-259) is computed from
`obs_dict["robot_orientation"]`, the very tensor the wrapper wrote (193); the [inview] `max_abs_rel_deg` is the sampled
`rel` itself. Neither shows the renderer saw the new pose. The mechanism is sound (Audit 1), but the smoke is the only
runtime evidence. Fix (either suffices, both preferred): (a) in the recorder's `transform_action_to_command`, when
`ep_steps == 0`, record the bearing from `obs_dict["robot_vehicle_orientation"]` (the quaternion the detector rendered
with, task 6459) and gate on that in `inview_rows_problems` (camdx1 362) and `visible_start_control` (510);
(b) add a downstream gate to SMOKE_OK: V-cell step-0 detected share ≥ 2 × R312 step-0 detected share and > 0
(expected roughly 5× from FOV geometry; occlusion and P9 first-frame no-lock reduce both arms alike).

**P1-2 The 624 pixel control can read NO_MATCHED_FRAMES and pass silently** (camdx1 600; prereg 109-110). At quota 1,
R312 has only a handful of detected step-0 frames at 6-12 m (≈128 × P(6-12 m) × ~0.19 in-FOV × visibility), so a
zero-match is a few-percent event; then the only resolution evidence is the `[inviewdiag]` camera_wh/detect_wh line
(config, not physics). Fix: if the step-0 ratio has no matched bin, gate on the already computed
`reported_frames_by_own_distance_ratio` ≥ 3 (hundreds of frames in shared 2 m bins); if that also has none →
SMOKE_FAIL. Additionally, in the analysis, gate DET labels on the pooled R624/R312 own-distance pixel ratio in the
6-12 m bins ≥ 3 (else DET reads CONTROL_FAIL), since `pixel_ratio_624_over_312` (analyse 395) is reported only.

**P1-3 START label is near-certain and does not answer the prereg's "how much of the loss".** V changes yaw *and*
distance (28 → 12 m), so START MATERIAL_GAIN — the trigger for the next design — is almost guaranteed by distance alone.
The recorder already logs `start_dist_xy_m` and `start_bearing_deg` in R cells, so a reported-only decomposition is
free: preregister, in `cell_measures`/`summarise` (analyse 219-…), R-arm capture/crash/never-detected by start class
{6-12 m & |b| ≤ 28.35°; 6-12 m & outside; > 12 m}, and the distance-matched yaw contrast V vs R(6-12 m) with the same
bootstrap (no label). R has ≈ 14 % of starts in 6-12 m (~430 episodes per arm over 3 seeds), enough for a coarse read.

### P2

1. Prereg "obstacle information is unchanged" → "approximately unchanged (sub-pixel resampling of the upsampled depth,
   finer target blanking)"; add KF variance, confidence saturation and P9 size bins to the stated OOD list; state that
   DET is "resolution as modelled by this pipeline", not pure pixel density (Audit 2).
2. Fallback comparability: under E64 the DET contrast compares 64-env with 128-env cells (different per-env random
   streams, layout count, TF32 batch shapes). State it in Limits, or under E64 run all four arms at 64 × 16.
3. An OOM may end as a process segfault, which the kernel-log counter treats as a host fault → VOID_HOST_FAULT instead
   of the fallback. Pre-specify how a segfault of the cell's own PID with an OOM line in `cell.log` is classified
   (MCE still voids).
4. Budget truncation reads FAIL: `timeout(arm)` (896) launches a cell with whatever remains, so a cap-killed cell
   reads FAIL → INCOMPLETE. Skip a cell whose arm's smoke-projected time exceeds the remainder and record
   BUDGET_EXHAUSTED; say in the prereg what INCOMPLETE means (no reading; continuation needs new approval).
5. Identity covers R312 only; V (CUDA generator) and 624 determinism untested on GPU. Accept as a stated limit.
6. Add a synthetic control with +8 pp in both 624 arms → DET MATERIAL_GAIN, START NO_DETECTED_CHANGE (exercises the DET
   weights and sign; currently DET is only tested at zero shift).
7. Recorder own-distance bins use `target_position` after `_advance_target` (task 5025, before transform 5028) while
   the frame was rendered before it: ≤ one step of target motion, reported-only bins; note it.
8. AGENTS.md requires confirming a new test file in the full PUBLIC_CPU suite (it registers `camdx1_eval_run`,
   `camdx1_eval_analyse`, a fake module and `tools/` on `sys.path`); no full-suite run is recorded. Run
   `tools/run_tests.py` before commit and record it in the WORKLOG.

## Anything making the result uninterpretable

Only P1-3 (without the decomposition the START label mostly measures the distance cap) and P1-2's analysis half
(a DET label without a demonstrated ≥ 3× pixel ratio on the eval data). With P1-1..3 applied I see no remaining
blocker; the 624 OOD caveats (P2-1) bound, but do not void, the DET reading.
