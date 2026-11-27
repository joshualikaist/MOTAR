# CAM-S3 S3-W package — independent review r1

- Reviewer: Claude Code subagent, model Opus 5.5 (claude-opus-5-5), fresh context, read-only on the repository.
- Date: 2026-11-27 (~02:55-03:40 KST).
- Scope: uncommitted working tree of `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b` (branch `camonly/s2b`,
  base `da5f07b`): `docs/plans/prereg_camonly_s3w_2026-11-27.md`, `aerial_gym/task/navrl_task/navrl_cams3.py`,
  `tools/realveh_entry.py` (diff), `tools/cams3_train.py`, `aerial_gym/rl_training/rl_games/train_navrl_cams3.sh`,
  `tools/cams3_python`, `tools/cams3_build_mask_library.py`, `resources/models/environment_assets/bars_h3_thin/`,
  `aerial_gym/config/asset_config/env_object_config.py` (diff), `tests/test_cams3.py`, `docs/EXPERIMENTS.md` (diff).
- Gate criterion applied: P1 = must fix before any GPU run.

## Verdict: NO_GO

One open P1 (P1-1). The code does what the text says, the wiring, guards, receipt check and start checks are sound,
and no existing flight-runtime or Policy F contract is touched. But under the preregistration's own false-near model
the "outlier-robust" sector reduction (k = 24) is not robust: it reports every bar inside 7 m at 0.33-0.61 of its true
range, practically the same as the unreduced single-column minimum. Both the actor's observation and the riskcap
governor see that. The preregistration and the user's approval scope ("sector reduction robust to outliers (R27b)")
describe something else, and the k = 24 rationale does not address the regime the run actually creates. Fix P1-1
(or have the user accept the stated bias explicitly), then re-review.

## Findings

### P1-1 — Under the injected false near, the k = 24 sector reduction is not outlier-robust; every near bar reads at 0.33-0.61 of its range (unstated)

Evidence:
- `navrl_cams3.py:225-236`: false near is iid per pixel on EVERY valid non-target return, with P20 = 0.51 / 0.18 / 0.14
  and P40 = 0.29 / 0.024 / 0.011 at 1 / 2 / 3 m, held constant beyond 3 m (`:54-56`, `:143-157`).
- `navrl_cams3.py:335-346`: rule 2 takes the 24th-smallest over all 20-28 columns x 234 rows of a bin (4,680 pixels
  for a bar filling the bin at 1-3 m, 2,160 at 7 m in my synthetic render).
- Expected P40 hits per bin: 4,680 x 0.024 ≈ 112 at 2 m and 4,680 x 0.011 ≈ 51 at 3 m; at 1 m most pixels are
  shortened. Hundreds of shortened pixels per bin, so the 24th-smallest is a low quantile of the shortened
  distribution, not the bar.
- Reviewer CPU check `scripts/false_near_vs_k.py` (output `scripts/false_near_vs_k.out`). It loads `navrl_cams3.py` by
  path and renders one full-height bar straight ahead with a 40x24 pinhole render, bilinearly upsampled to 312x234 and
  clamped at 10 m, as `navrl_detector.render_raw_rgbd` does. It then applies `apply_measured` (masks all-valid, so
  false near alone) and `sector_values(k=24)`. Medians over 64 envs:

  | true range | k=24 without FN | k=24 with FN | original single-column min with FN |
  |---|---|---|---|
  | 1 m | 1.00 | 0.33 | 0.33 |
  | 2 m | 2.00 | 0.75 | 0.70 |
  | 3 m | 3.00 | 1.35 | 1.22 |
  | 5 m | 5.00 | 2.70 | 2.31 |
  | 7 m | 7.00 | 4.27 | 3.30 |

  So k = 24 removes almost none of the false-near bias that R27b is meant to remove. With false near off, rule 2
  behaves as intended (thin bars between sampled columns are recovered; second table in the same output).
- The compressed ranges also feed the riskcap governor. `navrl_sensor_ablation.py:104,141-149` computes the governor
  scan for removed in-FOV bins from `_ablation_fused_m`, the fused static scan that contains the sector value. The
  speed cap in training therefore reacts to bars 2-3x nearer than they are.
- The preregistration's k rationale (prereg lines 62-66: "an isolated false-near speckle of fewer than 24 pixels
  cannot create an obstacle") argues against a failure the model cannot produce. False near applies only to existing
  valid returns (`eligible = ~invalid & ~no_hit & ~target_like`, `navrl_cams3.py:229`), so it never creates an obstacle
  in empty space. The effect it does produce, systematic range compression, is not mentioned. RULES.md R27b says k and
  the percentile are "to be fixed with the R5/R5b model". The chosen k was derived from thin-bar geometry only.
- Direction: the Limits section calls the constant-beyond-3-m profile "conservative". For a trained policy it is not.
  The policy learns that an observed 1.3 m means a true 3 m. Where real obstacles are textured, the flight runtime
  sees true ranges, and the policy would treat a real 1.3 m as comfortable.

Why P1: the run would not test what the preregistration and the approval name ("an outlier-robust sector reduction
shared by training and runtime (R27b)"). The observation and safety-filter input change by a factor of 2-3 for every
near bar, which the document does not state. This is a silent departure from the stated contract.

Fix options (one is needed before launch; the choice is a design decision):
1. Choose the reduction with the false-near model, as R27b asks: for example a per-bin quantile with a minimum count,
   or a per-column robust statistic and then a minimum across columns. Fix it with a CPU table like the one above
   (bias at 1-7 m) plus thin-bar retention, and record both in the preregistration.
2. Change the false-near model so that it matches what is known: stripe/blob structure on large uniform surfaces
   only, rates on the trained grid after the runtime's 2x2 min-valid reduction (see P2-2), and no extrapolation
   beyond 3 m without data. Then recheck k.
3. Keep both and state the measured bias table, the governor coupling and the direction of the train/flight mismatch
   in the preregistration. This departs from the approved "robust to outliers" scope, so it needs the user's
   explicit acceptance.

### P2-1 — The k = 24 rationale is internally inconsistent

Prereg lines 62-65 say a 0.20 m bar at 10 m "is hit by one of the 40 render columns over its full height". Two
problems:
- The Limits (line 130-131) say a 0.20 m bar is hit by a render ray only within about 7 m (1.65 deg spacing).
- The obstacle far plane is 10 m. The renderer clamps at `obstacle_max_range` (`navrl_detector.py:927-938`), and
  `camera_no_return_to_lidar_range` maps a value ≥ 9.999 m to no return (`navrl_perception.py:226`), so a bar at
  10 m is never an obstacle.

The reviewer's synthetic check (`false_near_vs_k.out`, second block in the run log) shows a 0.20 m bar is kept at 4-6 m
and, off-ray, at 7-9 m, and lost when no render ray hits it (7 m and 9 m on-axis). Restate the rationale with a
reachable case and tie it to P1-1.

### P2-2 — The false-near rate is applied per trained pixel, but the runtime reduces 2x2 source pixels by min-valid

The mask library correctly models the runtime's min-valid projection: a trained pixel is invalid only if all 4 source
pixels are invalid (`cams3_build_mask_library.py:30-33`). False near is the opposite case. The runtime takes the
nearest valid sample per trained pixel, so one false-near source pixel makes the trained pixel near. If source false
near were iid, the runtime rate per trained pixel would be 1-(1-p)^4: P40 at 2 m 0.024 → ~0.092. Training applies p
per trained pixel. Whether the 2026-11-26 shares were measured per source pixel or on the policy grid is not stated in
the preregistration (RULES.md R5b reads per source pixel). State which grid the rates refer to, and either convert or
list it as a limit. This interacts with P1-1.

### P2-3 — Unstated consequences of the two rules on other consumers

The preregistration says only that "the observation layout (898-D) is unchanged; its values change". It should also
state:
- The riskcap governor's in-FOV clearance reads the sector-reduced, measured-depth scan
  (`navrl_sensor_ablation.py:141-149`).
- Target detection loses pixels under the recorded masks and the top band. `_detect_rgbd` requires
  `depth < max_camera_range` (`navrl_perception.py:1601`), and masked pixels are set to it (`navrl_cams3.py:246`).
  This is on top of the P9 detection-error injector, which stays on. The detection-rate change is part of the
  treatment.

### P2-4 — No preregistered decision rule turns the smoke's speed into a launch or no-launch decision

The preregistration says the rules add GPU work "which the smoke measures" (line 101-102), with a 4.5 h cap against
S2b-FT's measured 3.4 h (`run_report.json`: 13:44:30 → 17:08:45), a 32 % margin, inside a 6 h GPU budget that must
also hold the evaluation.

The reviewer's estimate: rule 1 makes about 60 full-frame elementwise passes over 128x234x312, plus one extra
segmenter forward. That is ~4-5 GB of memory traffic, ≈10-12 ms per step. Rule 2 adds ~2-3 ms per step and three small
device syncs per call (`navrl_cams3.py:345` `int(valid.sum(...).min())`, `:374-376` boolean-mask indexing). Together
that is ~+10 % per epoch, about 3.7-3.8 h, probably under the cap. The run report also does not compute seconds per
epoch, although the preregistration lists it as recorded (line 109).

Fix before measurement: a steady-state s/epoch from the smoke (excluding start-up), a fixed projection rule (for
example, do not launch if s/epoch x 3000 > 4.3 h), and its record in the report. Peak GPU memory (nvidia-smi) should
also be recorded in the smoke. The library is 135 MB on the GPU, and the per-step transients are ~0.6-0.9 GB.

### P3-1 — The navrl_band "touch ⇒ overlap" guarantee no longer holds for thin bars

`asset_manager.py:348-350` and `navrl_bars_env.py:85-87` rely on "touch <= 0.4 guarantees overlap for every footprint
pair in the 0.4..0.8 m pool". With 0.20-0.30 m bars, an accepted touch pair can leave a slit. The reviewer re-implemented
the placement loop (`scripts/thin_slits.py`, output `thin_slits.out`: 8 layouts x 205 bars, 40 x 40 m). bars_h3 had 0
slits; bars_h3_thin had ~0.5 slit pairs per layout, widths ≤ 0.07 m (median 0.057 m). They are impassable and small,
so the effect is probably minor. The preregistration's "same placement rule" should still note that the rule's
compound-wall guarantee is weakened. Spawn clearance (`navrl_task.py:116-136`, per-bar circumradius), the route
planner (`navrl_task.py:7451`, maximum radius) and the governor do not assume the 0.4-0.8 m range. No other
training-path code depends on the pool name except `env_object_config.py` (handled) and the provenance fields.

### P3-2 — Wording and number accuracy in the preregistration

- Line 3 says "Written 2026-11-27 ~04:00 KST", but the file's mtime is 02:53:44 KST and the review started at about
  02:55. Correct the time.
- Line 90 says "the seven `NAVRL_DEPTH_MEAS*` keys", but the driver expects eight (`NAVRL_DEPTH_MEAS` plus seven
  settings, `cams3_train.py:208-211`). The code is right and the wording is off by one.
- Rule 1 line 57 "linear between, constant outside": correct for the code (`navrl_cams3.py:147`).

### P3-3 — Fail-closed flight contract only incidentally

An S3-W receipt is refused by `camera_policy_contract_cams2.py` only because `VARIANTS` lacks "1.5708". The contract
does not refuse `NAVRL_CAMS3_SECTOR_K`: it is not in `FORBIDDEN_IN_TRAINING`, and it would stay in `runtime_env`
because only the `NAVRL_DEPTH_` prefix is training-only. A future lineage with yaw 1.0472 and the sector rule would
pass the contract and fly without the rule. Editing that file now would break the S2b-FT contract binding
(`SOURCE_PATHS`), so record this for the runtime unit named in the last Limits bullet, which must also add the 90 deg/s
variant.

### P3-4 — The checkpoint does not carry the new observation-changing settings

`set_env_state` warns on yaw and bar pool but knows nothing of `NAVRL_DEPTH_MEAS*` or `NAVRL_CAMS3_SECTOR_K`. Only the
receipt records them. The later S3-W evaluation preregistration must bind the training receipt and install both rules,
otherwise an evaluation without them loads silently.

### P3-5 — Minor efficiency

Cache the flat column and bin index tensors in `sector_columns`. That removes the per-call nonzero syncs at
`navrl_cams3.py:374-376` and the `int(...)` sync at `:345`.

## Questions from the brief, answered

1. **Rule 2 vs the preregistration.**
   - Bins: 13 in-FOV bins.
   - Columns: pinhole `atan((cx-u)/fx)` in [θ-2.5°, θ+2.5°), 20-28 per bin (measured: 28, 25, 23, 22, 22, 21, 20, 21,
     22, 22, 23, 25, 28). Sampled columns 16 ... 295 each lie inside their bin, as the code checks.
   - All rows are included, k-th smallest with k = min(k, rows x columns).
   - Target carve is kept, because the rule reads the caller's `depth_no_target`.
   - Only in-FOV bin columns are written, and only sampled columns are read.
   - Introspection is robust in the trained path: the ablation's `fuse` wrapper calls the original function, whose
     `co_name` matches. The rule refuses S-L1B, which has no `depth_no_target`, and any other caller. `f_locals` on
     CPython 3.8 is a read-only snapshot.
   - CPU vs CUDA: `amin` and `kthvalue` are exact. `torch.equal` in the first 50 calls is a valid self-check.
   - Runs without `NAVRL_CAMS3_SECTOR_K`: nothing changes. The module is loaded only when its keys are set
     (`realveh_entry.py:177`), and `env_object_config.py` changes only for the new pool name.
2. **Rule 1.**
   - Order: it is installed before the depth model, which makes it the inner wrapper running on the modelled frame,
     and the ablation is outermost (`realveh_entry.py:306-325`). The install guards refuse the wrong order.
   - Invalid encoding = `max_camera_range` (20 m ≥ 10 m far plane, so it reads as no return).
   - No-hit pixels pass through.
   - Target exclusion is the perception's own segmenter.
   - Pointer: advance 0.5, jump 0.01 or at scene end; verified 5 fps from the recording stamps (dt median 0.200 s in
     all six scenes).
   - Own `torch.Generator` on the frame's device; the global RNG is not consumed.
   - Memory and time: see P2-4.
   - The defect is P1-1, a model-level problem, not a code bug.
3. **Launcher, wrapper and driver.**
   - The receipt check compares exactly the stated keys (plus the off-by-one wording, P3-2).
   - The start-check strings match the real formats. From S2b-FT's `launcher.log`: the restore line
     `task_steps=%d bars=%d->%d k_min=%.1f k_max=%.1f`, CONFIG MISMATCH `%.3f`, and the density RESET line.
     ARENA MISMATCH is `navrl_task.py:3816`.
   - The source checkpoint's `env_state` has `cfg_bar_pool=bars_h3`, `cfg_yaw_rate_max=1.0472`,
     `num_task_steps=480000`, `n_bars_active=205` and LR 7.5e-06, so both warnings and the restore line will appear.
   - The bar pool reaches `env_object_config` before import: the wrapper exports it immediately before
     `realveh_python` and runner.py.
   - The import-origin guard is intact (launcher lines 21-26; `realveh_entry.py` package-path check).
   - Checkpoint and library SHA-256 values verified.
4. **Existing contracts.** No change to `TRAINING_SOURCE_PINS` files (perception, ablation, depth model, cams2,
   camsafe) or to any `SOURCE_PATHS` file. Receipts of non-CAM-S3 runs are unchanged. Policy F paths are untouched.
5. **Preregistration accuracy.** Numbers checked against the code and records: library 1,984 → 1,849 frames,
   14.7 % / 10.9 %, per-scene medians 1.7-27.7 %, thin share 14/54 = 25.9 %, SHA-256 values, gate step 512,000,
   20-28 columns. Inaccuracies: P1-1 (robustness claim), P2-1 (k rationale), P2-3 (unstated consumers), P3-2
   (timestamp, key count).

## What I ran (CPU only; `CUDA_VISIBLE_DEVICES=""`, `nice -n 10 taskset -c 6,7,12,13`, `PYTHONNOUSERSITE=1`)

- `python -B -m unittest -v test_cams3` (from `tests/`): 18 tests OK in 2.0 s. Log: `test_cams3_run.log`.
- `scripts/false_near_vs_k.py` → `scripts/false_near_vs_k.out`: the table under P1-1.
- An inline thin-bar retention check, k = 24 without false near: bins 20-28 columns; a 0.20 m bar at 4 / 6 / 7 / 9 m
  on-axis reads 4.00 / 6.00 / no return / no return; 2 deg off-axis 4.00 / 7.14 / 7.86 / 9.30. The original sampled
  column reads 6.62 / 10 / 10 / 10 off-axis.
- `scripts/thin_slits.py` → `scripts/thin_slits.out`: placement slits (P3-1).
- Library frame-rate check of the six source `masks.npz` stamps, and the library row and column statistics.
- Loaded the source checkpoint on the CPU and read only the six `env_state` fields listed above.
- Read-only: git diff and status, S2b-FT `launcher.log` / `receipt.json` / `run_report.json`, and RULES.md.
- Not run: the full CPU suite (the lead is running it), any GPU code, the smoke.
