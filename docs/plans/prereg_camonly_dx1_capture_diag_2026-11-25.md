# Preregistration: CAM-DX1 — where does capture fail? Start condition and detection resolution, S2b-FT, 205 bars

- **Status:** preregistration, 2026-11-25; revised before any run and before its first commit after the independent
  design review r1 (GO_WITH_FIXES, `~/motar_tmp/camdx1_20261125/review_r1/REVIEW.md`; the changes are listed at the end). GPU approved by the user for this diagnostic evaluation
  ("오케이 그 확인용 평가는 진행하자", 2026-11-25). Runs only after this file is committed.
- **Policy:** CAM-S2b-FT final checkpoint `last_gen_ppo_ep_15000_rew_51.018074.pth` (sha256
  f2c80fcc2b2eaeb24e0b05d2284e74594a7ed8e7953f6b17b7dce49b09f2bbab), trained at the vehicle's 60 deg/s yaw limit,
  evaluated in its own training environment (`~/motar_tmp/cams2b_20261124/train_ft/receipt.json`, sha256
  3411d59cc1656a2b…) except the factors below. Evaluation only; no training; no checkpoint changes.
- **Why:** the CAM-S2a-E records show capture failures dominated by episodes that never come within 10 m of the
  target (205 bars, S12k: 26.9 % of all episodes; 24.1 % never detect it). The target starts 6-28 m away, uniform over
  the arena area, at a random bearing; the detector sees 66.7° and the 0.28 m quadrotor spans about 3.3 px at 20 m at
  312 × 234. The real use starts with the target in view, and the real camera's profile is 624 × 468. This unit
  measures how much of the capture loss each of these two conditions explains, before the next training is designed.
  It is a diagnostic: it does not select a flight candidate.

## Factors (2 × 2), all at 205 bars, seeds 4501-4503, 1,024 episodes per cell (128 envs × quota 8)

| arm | start | detection |
| --- | --- | --- |
| R312 | random (as trained; evaluation full distribution 6-28 m, uniform yaw) | camera = detection = 312 × 234 (as trained) |
| R624 | random | camera = detection = 624 × 468 |
| V312 | **visible start**: target distance 6-12 m (`NAVRL_GENERAL_GOAL_DIST_MAX=12`) and initial yaw so the target bearing lies within ±28.35° (half FOV − 5°) | 312 × 234 |
| V624 | visible start | 624 × 468 |

- **Visible start implementation:** a new evaluation-only opt-in module (`navrl_inview_start.py`, `NAVRL_INVIEW_START=1`,
  `NAVRL_INVIEW_START_HALF_DEG=28.35`, own seeded generator `NAVRL_INVIEW_START_SEED` = the cell's seed) wrapping
  `_align_general_spawn_yaw_to_target` so that it rotates the drone's initial yaw toward the sampled target regardless of
  the curriculum-saturation flag (evaluation forces saturation). Target placement is unchanged apart from the 12 m
  maximum. Occlusion is not controlled: report the share detected at step 0.
- **624 × 468:** `NAVRL_CAMERA_WIDTH/HEIGHT` = `NAVRL_DETECT_WIDTH/HEIGHT` = 624/468 (D8 requires equality). Same FOV
  66.7 × 52.6°. The obstacle depth stays the 40 × 24 render (upsampled), so obstacle information is approximately
  unchanged (the camera-to-scan fusion samples the upsampled depth at finer columns, and the target is blanked out of
  it with a finer mask); target pixels are 2× wider. The policy was trained at 312: this arm is out of the training
  distribution for the detection statistics — P9 error-model size bins, pixel counts, the tracker's measurement
  variance (its 0.15/√px term and the lateral term in metres per pixel) and the detection confidence, which saturates
  at 8 px — and this is stated with the result. DET therefore measures "resolution as this pipeline models it", not
  pixel density alone.
- **GPU memory fallback (fixed now):** if a 624 cell fails with CUDA out-of-memory at 128 envs, the 624 arms run at
  64 envs × quota 16 (1,024 episodes) and every 312 cell is kept at 128 × 8; the switch is recorded. If 64 envs also
  fail, the 624 arms read NOT_RUN_OOM and the 312 arms are still analysed. Rerunning the 312 arms at 64 envs too was
  considered and not chosen: at about twice the wall time per cell (CAM-S2a-E's 205-bar cells took 360-483 s at
  128 envs) twelve cells at 64 envs would exceed the 3.5 h cap. Under the fallback DET therefore compares 64-env 624
  cells with 128-env 312 cells (different per-env random streams and layouts): a stated limit of DET.
- **CUDA out-of-memory versus host fault (fixed now):** a rise of the kernel fault count (machine check, segfault,
  general protection) during a cell voids the stage, whatever the cell log says — an out-of-memory that ends in a
  counted segfault is VOID_HOST_FAULT, not a fallback. Without a rise, a 624 cell whose launcher receipt is FAILED and
  whose log carries CUDA out-of-memory text counts as out-of-memory (the fallback); any other failure is FAIL.

## Stages
1. **check** (CPU, no approval file needed): preflight problems only.
2. **smoke** (quota 1, never counted): R312 and V624 at 205 bars / seed 4501. SMOKE_OK needs exit 0, every cell check,
   ≥ 1 finished episode per cell, the visible-start positive control (V624: 100 % of step-0 target bearings within
   ±28.35°; at least one step-0 bearing outside ±33.35° in R312) read on the pose the first observation was RENDERED
   with (`robot_vehicle_orientation` when the observation is built), not on the tensor the module writes; the step-0
   detection control (the V cell's share of episodes detected on their first observation > 0 and ≥ 2 × R312's); and
   the 624 control (mean target pixel area per detected frame ≥ 3× the R312 value at matched step-0 distance bins; if
   no step-0 bin matches, every detected frame binned by its own distance must give ≥ 3×; if neither has a matched
   bin, SMOKE_FAIL).
3. **identity**: R312 at 205 / 4501 twice; result.json and the ledger must be identical apart from nonce and paths
   (determinism of the path). IDENTITY_FAIL stops the unit. It covers the R312 path only: the visible-start path (its
   CUDA generator) and the 624 path are not shown deterministic (a stated limit).
4. **eval**: the 12 cells, block-major by seed (each seed runs R312, R624, V312, V624), wall cap 3.5 h. A cell is not
   started when its projected time exceeds the remaining cap (projected time = the longest completed cell of the same
   arm in the stage; else 1.5 × the identity R312 cell time × the arm's smoke time over the R312 smoke time); the
   stage then records BUDGET_EXHAUSTED and reads INCOMPLETE. An INCOMPLETE eval is analysed on complete seed blocks
   only (a block counts for an effect when every arm of that effect has a cell in it); an effect with no complete
   block reads NOT_RUN. Continuing an INCOMPLETE eval needs a new approval.
- Host faults (kernel machine-check / segfault / GP count), boot id and online CPU list unchanged across each stage;
  a rise voids the stage. Idle host, one RTX 3070, no other GPU compute.

## Measures (per cell; episode-weighted, pooled over seeds)
capture, crash, timeout; crash classes (bar contact, target contact swept, out of bounds); share never detected;
share detected at step 0; minimum target distance bins (< 1, 1-2, 2-5, 5-10, ≥ 10 m) by outcome; capture time median
and p90; mean executed speed; for 624 vs 312 the mean target pixel area per detected frame. Reported only: the R arms'
capture, crash, timeout and never-detected shares by start class {6-12 m and step-0 bearing within ±28.35°; 6-12 m and
outside; > 12 m} (rendered step-0 pose).

## Readings (fixed now; descriptive effects, not a design gate)
For each effect, the paired difference by seed block (3 blocks) and an episode bootstrap (10,000 resamples, seed 4511)
of capture and crash:
- **START effect** = mean(V312 − R312, V624 − R624);
- **DET effect** = mean(R624 − R312, V624 − V312);
- **interaction** = (V624 − R624) − (V312 − R312), reported only;
- **START_6_12** = mean(V312 − R312₆₋₁₂, V624 − R624₆₋₁₂), where R₆₋₁₂ is the R cell's episodes that started 6-12 m
  from the target: the distance-matched contrast, in which the start bearing is the main difference. Same paired block
  difference, same bootstrap (the restricted R cells are resampled after the twelve cells, on the same generator).
  The full START effect changes the yaw and the distance range together and is reported as "visible start + shorter
  range".
- **DET is interpretable only** if the pooled 624 / 312 mean target pixel area per detected frame, over the frames at
  6-12 m (own distance), is ≥ 3 for both R624/R312 and V624/V312; otherwise DET reads NOT_INTERPRETABLE and gives no
  label.

Label per effect and metric: **MATERIAL_GAIN** if the difference ≥ +5 pp and the 95 % bootstrap CI excludes 0;
**MATERIAL_LOSS** if ≤ −5 pp and the CI excludes 0; **SMALL** if the CI excludes 0 but |Δ| < 5 pp;
**NO_DETECTED_CHANGE** otherwise (for crash, "gain" means a reduction).

**Use of the result (fixed now):**
- START_6_12 MATERIAL_GAIN in capture (the primary evidence for "visible start") → the next training's evaluation and
  design target the visible-start case (the flight case), and search is reported separately. The full START effect
  is reported as "visible start + shorter range" and does not by itself trigger this.
- DET MATERIAL_GAIN in capture → the next training uses 624 × 468 detection (memory permitting).
- START_6_12 and DET both NO_DETECTED_CHANGE → the capture loss is not explained by acquisition; the next design looks at the policy
  (curriculum, initialisation) instead.
- The best cell's absolute numbers are reported against the user's bar (crash ≤ 20 %, capture ≥ 70 %) but are not a
  flight decision: one policy, one density, no memory grid, no backstop.

## Analysis controls (run on the analysis code before it reads the eval)
Synthetic ledgers: identical arms → NO_DETECTED_CHANGE; +8 pp capture in every block → MATERIAL_GAIN; +2 pp in every
block with tight CI → SMALL; −8 pp → MATERIAL_LOSS; +8 pp capture (−8 pp crash) in both 624 arms → DET MATERIAL_GAIN
and START NO_DETECTED_CHANGE. The recorded CAM-S2a-E S12k-205 and YC12k-205 cells as a pair
(YC12k − S12k) → capture MATERIAL_LOSS, crash MATERIAL_LOSS (crash up), matching CAM-S2a-E's gate Y.

## Limits
One policy, one density, three evaluation seeds; the 624 arms are out of the training distribution; visible start
changes both yaw and the distance range (START_6_12 separates them only on the R episodes that started 6-12 m away,
about 14 % of them); identity covers the R312 path only; under the memory fallback DET compares 64-env with 128-env
cells; DET is resolution as modelled by this pipeline; no attribution beyond the two factors; no flight claim.
Records under `~/motar_tmp/camdx1_20261125/`.

## Implementation notes (added 2026-11-25 with the code, before any run; they fix definitions, not thresholds)
- **Package:** runner `tools/camdx1_eval_run.py`, analysis `tools/camdx1_eval_analyse.py`, module
  `aerial_gym/task/navrl_task/navrl_inview_start.py`, tests `tests/test_camdx1.py`; additive, OFF-identical hook in
  `tools/realveh_entry.py` (installed after `navrl_cams2diag`). No new launcher: the pinned CAM-S2a-E launcher passes the
  arm keys through unchanged.
- **Visible start:** the wrapper runs the original aligner first (a no-op at evaluation saturation), then writes the yaw
  into `obs_dict["robot_orientation"]` where the original writes it; `reset_idx` commits it in its own
  `update_states` + `write_to_sim` (`navrl_task.py:4840-4846`), before the first observation is rendered. The bearing is
  measured from the drone's centre (the camera sits 8 cm forward; the 5° margin covers it).
- **Cell environment:** S2b-FT's training `env_navrl` with CAM-S2b-FT-E's per-cell replacements for its unclipped FT arm
  (fixed density, module seeds 10 × seed + k, ledger, quota, contact code, `navrl_cams2diag`), plus in every cell the
  evaluation-only recorder of the new module (`NAVRL_INVIEW_DIAG=1`, rows `inview_episodes.jsonl` in the cell folder),
  plus the arm keys named above. Any other difference refuses the cell.
- **Measures, as recorded:** crash classes from `navrl_cams2diag` (`contact` = bar contact, which also includes a PhysX
  contact with the target mesh; `target_contact_swept`; `out_of_bounds`; `below` / `above` reported too); never
  detected = recorder `visible_steps` = 0; detected at step 0 = the task's detection flag on the episode's first
  observation; minimum target distance = the ledger's `min_relative_distance_m` (3-D centre distance after each step);
  capture time = `episode_steps` × `rl_step_dt_s`, numpy linear percentiles; executed speed = the horizontal norm of the
  executed command (after delay, governor and blind cap), the vehicle's own horizontal speed reported beside it; target
  pixel area = the renderer's private target-mask pixel count (`detector.target_mask > 0`, occlusion included) on the
  frames whose detection flag is set, evaluation-only, never returned to the task. The step-0 bearing and distance
  used by the checks are read from the pose and target position with which the first observation is built
  (`_process_obs_perception`); each frame's distance bin is the distance at that frame's render (the target advances
  by up to one step before the policy acts, so a bin taken at action time would be off by one step).
- **624 smoke control:** gated on the detected step-0 frames (the episode's first observation with the detection flag
  set; its rendered pixel count and its step-0 horizontal distance), so the trajectory after the start cannot confound
  the comparison; 1 m bins of the step-0 distance; matched bins = bins with ≥ 1 such frame in both cells; ratio
  = Σ_b w_b A624_b / Σ_b w_b A312_b with w_b the 624 cell's frames in bin b; PASS iff ≥ 3; no matched bin →
  then the ratio over every detected frame binned by its own distance (2 m bins, same standardisation) decides; no
  matched bin in either → NO_MATCHED_FRAMES = SMOKE_FAIL. Reported beside it: the ratio over every detected frame of
  the episodes binned by step-0 distance, and a d²-normalised step-0 ratio.
- **Bootstrap:** one `numpy.random.default_rng(4511)`; per cell and resample one multinomial draw of the outcome counts
  (the exact law of an episode resample with replacement inside the cell); cells in the order R312, R624, V312, V624 ×
  seeds; all effects on the same resamples; percentile interval.
- **Caps and memory fallback details:** smoke and identity are capped at 0.5 h each (the eval at 3.5 h, as above). In
  the fallback the smoke cell runs at 64 envs × quota 2; an out-of-memory cell's folder is kept and its rerun goes to
  `<arm>_e64/`; 624 cells already run at 128 envs are rerun at 64 envs at the end of the plan so each 624 arm is
  homogeneous. If V624 reads NOT_RUN_OOM in the smoke, a V312 smoke cell (205 / 4501, quota 1) carries the
  visible-start control.
- **Expected contract warnings:** the 624 arms log `CONFIG MISMATCH | NAVRL_DETECT_WIDTH/HEIGHT` (312/234 → 624/468) and
  the V arms `TASK-DISTRIBUTION MISMATCH | NAVRL_GENERAL_GOAL_DIST_MAX` (28 → 12); these exact lines are allowed for
  those arms only, on top of CAM-S2a-E's allowlist.

## Changes after design review r1 (2026-11-25, before any run and before the first commit)
Review r1 (Claude Opus 5.5, independent, `~/motar_tmp/camdx1_20261125/review_r1/REVIEW.md`) read GO_WITH_FIXES.
Thresholds, factors and the label rule are unchanged; these were changed in the body above:
- P1-1: the visible-start control reads the rendered step-0 pose; a step-0 detection control (V > 0 and ≥ 2 × R312)
  joins SMOKE_OK.
- P1-2: the 624 smoke control no longer passes on NO_MATCHED_FRAMES (own-distance fallback, then SMOKE_FAIL); DET is
  labelled only if the 6-12 m pixel-area ratio is ≥ 3 in both pairs, otherwise NOT_INTERPRETABLE.
- P1-3: START_6_12 (distance-matched) added and made the primary evidence for "visible start" in the use of the result;
  the full START is reported as "visible start + shorter range"; the R arms' start-class breakdown is reported.
- P2-1: the 624 wording (obstacle input approximately unchanged; KF variance, confidence and P9 bins named).
- P2-2: under the memory fallback only the 624 arms move to 64 envs (rerunning all arms would exceed the cap); stated
  as a limit.
- P2-3: out-of-memory versus host fault fixed.
- P2-4: the budget projection, BUDGET_EXHAUSTED and the reading of an INCOMPLETE eval fixed.
- P2-5: identity covers R312 only, stated.
- P2-6: the synthetic DET control added.
- P2-7: per-frame distance bins taken at the frame's render.
