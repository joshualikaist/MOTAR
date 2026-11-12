# CAM-T1 v1: independent pre-gate review r1

- **Reviewer:** Claude Code subagent, model Opus 5.5 (claude-opus-5-5). Fresh context, read-only on the repository.
  Wrote only under `~/motar_tmp/camt1_20261112/review_pre_r1/`. No GPU, no simulator, no commit. Usage state USAGE_UNKNOWN.
- **Object:** worktree `src/MOTAR-camonly-t1`, branch `camonly/t1`, HEAD `eefb832`, uncommitted: the preregistration
  `docs/plans/prereg_camonly_t1_2026-11-12.md`, `tools/camt1_run.py`, `tools/camt1_analyse.py`,
  `aerial_gym/rl_training/rl_games/eval_navrl_camt1_arms.sh`, `tests/test_camt1.py`, the `docs/EXPERIMENTS.md` and
  `WORKLOG.md` diffs, and the chain `~/motar_tmp/camt1_20261112/run_camt1.sh`.
- **Date:** 2026-11-12.

## Verdict: PASS_WITH_FIXES

There are no P0 or P1 findings. I found no bug that would make the GPU run produce wrong, unbound or uninterpretable
cells. Every way I tried to mis-configure a cell is caught by a fail-closed check: a cap that does not take effect,
a hook missing or loaded in the wrong arm, a broken or absent trace, or a wrong reference cell.

Six P2 findings remain. Four of them (F1–F4) change analysis or control rules that the preregistration freezes at
commit. Fix each one before the commit, or record it in the preregistration as consciously declined; after the commit
they need an amendment. F5 is a bounding gap in the chain script, which is outside the repository. F6 is about how the
CPU gate is worded.

## What I verified by running (CPU only, outputs in this folder)

| Check | Command / file | Result |
| --- | --- | --- |
| Unit tests | `unittest discover -s tests -p test_camt1.py` (aerialgym python, `PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= -B`, TMPDIR here) → `test_camt1.log` | 35/35 OK, 0 skipped (the real E3 / K2E / G3 data are present) |
| G3 positive control | `camt1_analyse.py g3-control …/camonly_c2trace_v1` → `g3_positive_control_review.json` | POSITIVE_CONTROL_PASS. A / B_out / C at 43.5° equal the record (475 / 5,138 / 5,109 episodes). At 33° (negative control): 56.0 / 79.99 / 80.23 %. The file is byte-identical to the builder's `build_smoke/g3_positive_control_build.json` (SHA-256 `cf27c4b6…`) |
| Stage B gate controls | `camt1_analyse.py controls` → `stage_b_controls_review.json` | K1s1−K2s1 WORSE +3.54 pp (boot [2.51, 4.67], t [1.65, 5.43]); K1s2−K2s2 WORSE +4.93 pp; both reverses SMALL_REDUCTION; null K2s1−K2s2 NEGLIGIBLE +0.42 pp (boot [−0.07, 0.90], t [−0.44, 1.27]); camk2e manifest OK. These are the preregistration's numbers |
| Timing/consistency fields on G3 traces (fields CAM-T1 records but does not gate) | `check_g3_consistency.py` → `.out.json` | Over 8 cells / 4,025 crash episodes: `actions_match` 100 %, `frames_contiguous` 100 %, `crash_rows_without_trace` 0, extrapolated-bar agreement 98–100 %, VALID 96–100 %, eye never inside a bar at L. Contact distance at L: median 0.23 m, p90 0.31 m |
| Yaw / quaternion convention, checked against independent ledger fields | `check_yaw_convention.py` → `.out.json` | Across 2,898 G3 episodes, `yaw_of(p_vehicle_quat)` at frame 0 equals the kroc2027 ledger's `initial.pursuer_yaw_rad` exactly (max error 0), and the target bearing equals `initial_world_bearing_rad` (p99 2e-8 rad). The xyzw ordering and the bearing sign are therefore right independently of G3's code |
| Preregistration claim "96–98 % of native episodes command > 2.5 m/s, max 2.95" | E3 kroc rows | V3s1/V3s2 at 115/205: 96.0–98.0 %, max 2.947–2.961 m/s. **Holds** |
| Preregistration claim "E3: 164 s per V3 cell, 167 s at 115/205" | E3 `eval_run_report.json` | 164.2 s / 166.7 s; V3s1 205/4501 = 172 s. **Holds** |
| Trace overhead (budget risk) | G3 receipts vs C1 cells | G3 no_lidar 205/4501: 173 s; untraced C1: 170–171 s. Negligible, so the 2.0 h Stage A cap is safe |
| Source drift between E3's run commit and the base | `git diff --stat aaca40e eefb832 -- aerial_gym motar_eval tools/realveh_* tools/camtrain_entry.py tools/actionhold_*` | **No change**; only `tools/sitl_*` and docs were added. Byte-identity with E3 is therefore a reasonable expectation |
| CPU suite claim (item 5) | Mechanical diff of the FAIL/ERROR lines in `build_smoke/public_cpu_full_r2.log` against `build_smoke/baseline_eefb832_failing_modules.log` (`r2_fail.txt`, `base_fail.txt`) | 17 vs 15. The 15 baseline names are a subset of the 17; the only 2 extra are `test_experiment_index` (×2). See F6 |
| Repository untouched | `git status --short` before and after | Same 7 entries; no new untracked or byte-cache files |

## What I verified by reading

- **Launcher copy vs `eval_navrl_camk2e_arms.sh`** (`diff`). The only differences are the header comment, the `[camt1]`
  `case` guard (lines 51–55) and line 127 `NAVRL_SPEED_GOVERNOR_FREE_MPS="${CAMT1_GOVERNOR_FREE_MPS}"`. The guard
  accepts exactly the strings `3.53553390593` / `2.5` / `2.0`. `NAVRL_MAX_VELOCITY=2.5` (line 121) and `FIXED_MPS=2.0`
  are unchanged. SHA-256 `c166f560…` matches `LAUNCHER_SHA`.
- **The cap takes effect.** `SpeedGovernorConfig.from_environ` reads `NAVRL_SPEED_GOVERNOR_FREE_MPS`, and riskcap
  accepts FREE ≥ FIXED, so FREE = FIXED = 2.0 is valid and gives a constant cap. `result.json`
  `condition.speed_governor_free_mps` echoes the configured value (`navrl_task.py:9498`).
  `pursuer_commanded_speed_max_mps` is the norm of the governed `command[:, 0:2]` (`kroc_eval_adapter.after_physics`,
  called after `transform_action_to_command`), so the per-cell commanded ≤ cap + 1e-3 check is a real manipulation
  check. The native maximum is 2.95, so a cap that did not apply would fail ~97 % of episodes. The float32 value
  2.0000002 is within tolerance.
- **The delay hook acts only in hook arms.** Only hook arms get `PYTHONPATH = tree:tools/actionhold_site` and
  `MOTAR_ACTION_*`. Inherited `MOTAR_ACTION_*` and `CAMT1_*` are refused, and `K.base_env` forces `PYTHONPATH` = tree
  for plain arms. The sidecar must be present for hook arms (config, patch_applied, calls > 0, delayed_zero > 0 for
  D = 1) and absent for plain arms. `realveh_entry` and runner.py install no competing `meta_path` finder for
  `navrl_players`. The import origins of both environments, and `sitecustomize` (plain: none), are checked.
- **The trace.** `NAVRL_KROC2027_OCCLUSION_TRACE` is scrubbed from the inherited environment and re-added only for
  traced arms. The snapshot must have it exactly for traced arms; E3 snapshots do not contain it. The trace's close
  reason `complete` matches what G3 recorded. The camera metadata comes from `vis.detector_hfov_deg`/`vfov`, which
  `NAVRL_CAMTRAIN_HFOV/VFOV` override to 66/54, so the 66 × 54 check is consistent.
- **Reference cells and pairing.** `reference_dir` uses `eval_01/V3s{policy seed}/E2_obstacle_aware__{d}bars__seed{s}`.
  Every file read is bound to the committed v3e manifest, and the E3 report must be COMPLETE at `aaca40e`. Stage B is
  paired by (density, seed) block. Seeds 4501–4503 and the module seeds (10·seed+1, +2) match E3. The order is
  density, seed, arm.
- **Snapshot and condition checks.** Comparing against the reference is strict: the only differences allowed are the
  run keys plus the arm's own key. The run-id and path variables do not appear in either ledger; I checked the E3
  rows. So byte identity across trees and run ids is attainable, as E3's own identity_01 showed.
- **Stage dependency, rate gates, VOID handling, report-once (`open("x")`) and exit codes** match the preregistration.
  Control needs exactly one finished trace report on HEAD, and the envelope needs exactly one CONTROL_PASS on HEAD
  plus the projection gate.
- **Analysis geometry.**
  - The detector renders with `robot_vehicle_orientation`, which is yaw-only (`navrl_task._process_obs_vision`).
  - The obstacle depth is a ray length with far plane `camera_obstacle_max_range` = 10 m and the same render FOV
    (`navrl_detector.py:363–392`), so the 33° / 27° / 10 m parameters are the policy's actual depth-camera parameters.
  - The eye transform, the vehicle-frame velocity rotation, roll and pitch, the axis-aligned box surface distance and
    the footprint angular-interval test (including the ±2π shifts and the eye-inside case) are correct.
  - Frame k is the state after k actions, and the trace's (env, episode) key joins the ledgers' (env_index,
    env_episode_index). G3's data confirm both empirically: actions == L + 1 in 100 % of crash episodes.
- **Statistics.**
  - The cell-cluster bootstrap within density is the right primary for correlated episodes.
  - Stage B reuses E3's `gate_d` (A_BAND 5, FLOOR 2, t df 4 with t = 2.776) and `strat_ci` (seed 20261108,
    10,000 resamples) unchanged.
  - The controls follow the preregistration's table.

## Findings

### F1 (P2): Stage A records the measurement-consistency fields but does not gate them

- **Where:** `tools/camt1_analyse.py:494–496` (`bad` uses only the crash count and the VALID share);
  `summarize` (lines 341–344) records `actions_match_share`, `frames_contiguous_share` and
  `extrapolated_bar_agrees_share`; `cell_episodes` (lines 270–272) counts `crash_rows_without_trace`. Preregistration
  *Reading: MEASUREMENT_INVALID*.
- **Failure scenario:** The whole Stage A timing rests on "L is the state one step before contact". This run uses
  the realveh entry, the OAK-D depth model and vehicle randomization; G3 used neither. Suppose that in this path the
  last staged frame of a crash episode is dropped, or the post-contact state is consumed under the old episode index.
  Then L shifts by one step. The categories would then be computed 0.2 s before contact, or after it, with no reading
  change. Likewise, a trace that silently misses many crash episodes would still read DESCRIBED.
- **Fix (cheap; G3 shows 100 % on all three):** add to MEASUREMENT_INVALID, per policy seed:
  - the share of quota crash rows with a trace < 0.99;
  - `actions_match_share` < 0.99 (or None);
  - `frames_contiguous_share` < 0.99.

  State the thresholds in the preregistration.

### F2 (P2): invalid (non-bar) crash episodes are assigned a category relative to a bar they did not hit

- **Where:** `camt1_analyse.py:189–244` (the category is computed for every crash episode) and `summarize`
  (category means over all episodes). Preregistration *Disclosed limits*: "ambiguous and invalid episodes stay in the
  counts".
- **Failure scenario:**
  - E3's V3 cells include out-of-bounds crashes: 3 of 161 (1.9 %) at 70 bars, 205/4501. On G3, 0.2–3.9 % of crash
    episodes were not VALID (`check_g3_consistency.out.json`).
  - Such an episode is classified relative to the nearest bar at L (more than 0.6 m away), mostly as BLIND or
    LEFT_FOV. That inflates those shares by an amount that depends on density.
  - This is a small bias, but it is a definitional error in the primary quantity. It could matter when the CLEAR rule
    compares close shares.
- **Fix:** compute the category shares and their intervals over VALID episodes (the contacted bar identified). Report
  the invalid count and its own category split separately. Alternatively, keep all-episode shares as the named
  sensitivity. Decide before the commit.

### F3 (P2): the identity controls are weaker than G7's and E3's: `result_equal` is recorded, not required

- **Where:** `tools/camt1_run.py:268–284` (`ledgers_equal`: `pass` = both ledgers byte-equal and outcome counts
  equal; `result_equal` recorded only) and `control_reading` (lines 291–296).
- **Comparison:**
  - G7 (`tools/actionhold_run.py:100–104`) required `episodes_equal and result_equal and episodes_byte_equal` for its
    identity cell.
  - E3's identity_01 recorded `result_equal: true` for its identity cells.
- **Failure scenario:** a launcher, commit or hook difference changes `result.json` content outside the outcome
  counts while the two ledgers stay byte-identical. Examples are `crash_causes`, `action`/`speed_governor` statistics,
  `strata`, or a condition field the snapshot does not carry. `native_identity` and `k1d0_identity` would still read
  CONTROL_PASS.
- **Fix:** for `native_identity` and `k1d0_identity`, also require `result_equal` (G.identity_compare: `result.json`
  minus the nonce and checkpoint path). Keep it recorded-only for the trace cells, or require it there too: the trace
  adds no condition key. Update the preregistration's control wording.

### F4 (P2): stage order and reading name. A reproduction failure is only detected after the 2 h trace stage, and is then labelled TRACE_INTERFERES

- **Where:** `camt1_run.py:393–414` (control requires a finished trace report) and `trace_reading`
  (lines 299–305). Preregistration *Order of stages* and *Stage A noninterference gate*.
- **Failure scenario:** suppose this commit, this launcher copy or this host does not reproduce E3 byte for byte, for
  a reason that has nothing to do with the trace (for example a GPU, driver or host change since 2026-11-10).
  - All 24 traced cells (about 1.1 h) then run first and read TRACE_INTERFERES, which attributes the difference to
    the trace.
  - Only afterwards does `native_identity` fail. That gives CONTROL_FAIL, and Stage B does not run.
  - The Stage A record then carries a misleading label, and GPU time has been spent before the cheapest
    discriminating check (one 3-minute cell).
- **Fix (any one):**
  - (a) Run `native_identity` first, as a one-cell `identity` stage before `trace`. Let the trace stage require its
    pass. A failed identity then stops the chain at about 3 min.
  - (b) Keep the order, but make Stage A's analysis read `NOT_REPRODUCED` instead of TRACE_INTERFERES when the
    control's `native_identity` fails, and state this rule in the preregistration.

  (a) is cheaper and keeps the trace reading meaningful. Either way, also state how Stage B is read if Stage A reads
  TRACE_INTERFERES while `native_identity` passes (as now: Stage B proceeds, because the native comparison rests on
  `native_identity`). Note that V3s2's native reproduction is then untested directly.

### F5 (P2): the chain's memory watchdog and `timeout` cannot stop the GPU process

- **Where:** `~/motar_tmp/camt1_20261112/run_camt1.sh:37–51` (`run_capped`) and `tools/gr_run.py:164–181`
  (`run_launcher` starts the launcher with `start_new_session=True`).
- **Failure scenario:**
  - The launcher (bash → play_navrl.sh → the Isaac Gym process) runs in its own session and process group.
  - When `tree_rss_kb` exceeds 20 GB, the watchdog TERMs/KILLs `timeout` and its direct children (the runner python)
    only. `pkill -P` does not reach grandchildren, and GNU `timeout`'s group signal does not cross the new session.
    The same holds when the chain's `timeout -k` fires.
  - The runner dies without a SIGTERM handler, and the GPU cell keeps running, orphaned and still growing. The
    watchdog loop has already `break`-ed, so nothing monitors it any more.
  - The chain then proceeds to the CPU analyses while a GPU evaluation is still running. That breaks the idle-host
    rule and the bounding rule ("a job without a memory cap").
  - The runner's own per-cell timeout (`killpg`) is correct; this gap is only in the outer bounds.
- **Fix:**
  - In `run_capped`, collect every descendant PID with the same ps walk `tree_rss_kb` uses, before killing. Kill them
    all (TERM, wait, KILL), or kill the launcher's session with `pkill -s <sid>`.
  - Optionally, give `camt1_run.py` a SIGTERM handler that `killpg`s the active launcher.
  - After any kill, wait until `nvidia-smi --query-compute-apps` shows no CAM-T1 process before the CPU steps start,
    and log it.

  E3's chain had no memory watchdog at all, so this is still an improvement; it is just not yet a bound.

### F6 (P2): CPU gate. The builder's claim is correct; it is acceptable only as a documented baseline-equivalence gate, not as PUBLIC_CPU 0/0

- **Evidence:** `public_cpu_full_r2.log` reads `Ran 3970 tests … FAILED (failures=9, errors=8, skipped=18)`. The
  sorted FAIL/ERROR lines (`r2_fail.txt`) minus the baseline's (`base_fail.txt`, 6 modules run on a clean detached
  worktree at `eefb832`: 127 tests, failures=7, errors=8) leave exactly the two
  `test_experiment_index.ExperimentIndexTest` failures (`test_every_member_pattern_matches_a_record`,
  `test_read_first_and_lineage_paths_exist`). These are explained by the untracked preregistration.
- **The other 15** fail identically at the base:
  - `test_boia_shadow_runner` ×7, `test_sitl_s3a_policy_precision` ×3, `test_result_manifest` ×2, `test_camk1e`,
    `test_navrl_stopcap_screen_contract` and `test_sitl_s4r1` ×1 each.
  - The causes visible in the logs are Policy F's ignored checkpoint missing under `runs/`, records that name the
    `MOTAR-camonly` path, untracked raw `results/`, and CUDA_VISIBLE_DEVICES refusals.
  - None imports CAM-T1 code.
- **Side note:** `failing_modules_t1_realindex.log` shows 3 more errors (`test_sitl_agq_geometry_diag` ×2,
  `test_sitl_s3b_dev08_prefix` ×1), all "OMP_NUM_THREADS must be 1". That run was missing the thread variables, which
  explains them. They do not appear in r2.
- **Not verified by me:** the claim that `test_experiment_index` passes 9/9 with the files indexed. I did not build a
  temporary index, because that writes blobs into the shared `.git`.
- **Acceptable on these conditions:**
  1. After the local commit, rerun `test_experiment_index` (must pass) and the full PUBLIC_CPU. The FAIL/ERROR set
     must equal exactly the 15 baseline names; diff it mechanically as above and record the diff.
  2. The WORKLOG and the report state "PUBLIC_CPU not 0/0: 15 pre-existing environment failures, identical to base
     `eefb832` in a fresh worktree", never PASS.

  Any 16th failure blocks the GPU run.

### F7 (P3): the positive control covers the shared plumbing, not the primary FOV test

- **Where:** preregistration *Positive control of the analysis*; `g3_control` (`camt1_analyse.py:382–416`) uses the
  `g3` profile (centre origin, nearest point, no vertical or range test).
- **Note:** the control proves the plumbing exactly: trace read, join, layout lookup, contact bar, windows, categories
  and percentages. The primary profile's new code (eye origin, footprint interval, vertical, range) is validated only
  by synthetic unit tests. Those tests are adequate: hand-computed IN_VIEW, BLIND, LEFT_FOV, EARLY and ambiguous
  cases, footprint vs nearest point, eye vs centre, vertical, range and the inside case.
- **Fix:** add one sentence to the preregistration that says so.
- **Related:** the chain reruns `g3-control` at the run commit, but its result is not bound to Stage A's reading.
  Either state that POSITIVE_CONTROL_FAIL at the run commit makes Stage A TRACE_BINDING_FAIL (and check it in
  `stage_a`), or say that it is reported only.

### F8 (P3): Stage B writes `labels_before_controls` when the controls fail

- **Where:** `camt1_analyse.py:617–618`.
- **Note:** the preregistration says "CONTROL_FAIL and no label is reported", but the JSON still carries every
  per-arm label under `labels_before_controls`.
- **Fix:** omit it on CONTROL_FAIL, or rename it to `labels_not_readable_controls_failed` and say so in the
  preregistration.

### F9 (P3): geometric approximations (all non-binding here; worth one line in *Disclosed limits*)

- **Vertical test:** elevation is measured over the horizontal distance r, whereas a pinhole camera's vertical bound
  is z / x_cam with x_cam = r·cos(bearing), so the test is slightly lenient off-axis. At 1 m altitude with 0–3 m
  bars it never binds.
- **Horizontal coverage:** the 40-column obstacle depth image has pixel centres only out to about ±32.3° (cx 19.5,
  fx 30.8), with about 1.65° spacing. The geometric ±33° is therefore an upper bound on the half-FOV the policy
  actually samples.
- **IN_VIEW at L:** with a median contact distance of 0.23 m at L, the eye is usually below the depth model's MinZ
  (0.4 m). A geometrically IN_VIEW bar at L is then often invalid in depth. The analysis reports `below_minz`; the
  reading text should keep IN_VIEW ≠ "visible in depth".

### F10 (P3): smaller notes

- The two "reverse" Stage B controls (K2s − K1s) are the mirror images of K1s − K2s on the same blocks, not
  independent controls. They test only the reduction branch of `gate_d`.
- `cell_episodes` raises on rotated obstacles (line 258). That fails closed, but `stage_a` then exits without writing
  a reading. Consider catching the error and writing TRACE_BINDING_FAIL with the reason.
- The preflight's environment allowlist is compared stage to stage, not with E3's `env_allowlisted`. An inherited
  `PYTHON` would redirect `REALVEH_REAL_PYTHON` (inherited from the reference runners). `native_identity` covers this
  empirically; a one-line comparison with E3's report would make it explicit.
- GPU idleness is checked at preflight only, not before each cell. This is the same as the reference runners.
- The cell-cluster interval has 3 clusters per density per seed, which is narrow and discrete. This is disclosed. The
  CLEAR rule is guidance, not a gate, so this is acceptable.

## Preregistration completeness

- **What is fixed before data:** the cells, order, references, checks, VOID_HOST_FAULT, budgets and rate gates, every
  analysis rule, the bootstrap seeds, the Stage B gate and controls (values computed on existing data and reproduced
  here), the guidance mapping and the wording limits. Text and code agree on every point I checked: SCOPE, plans,
  keys, reasons, thresholds, seeds and the E3 timing numbers.
- **Missing or worth stating:**
  - the F1 thresholds;
  - the F2 scope of the category shares;
  - the F3 identity rule;
  - the F4 order or reading rule;
  - whether the run-commit g3-control binds Stage A (F7).
- **The "Review before the commit" section** must record this review (r1, Opus 5.5, PASS_WITH_FIXES) and each
  repair. A re-review of the repaired diff is advisable, because F1–F4 change frozen rules.
- **`docs/EXPERIMENTS.md`:** the new entry, its lineage line and its Members line are consistent. Its Status text
  ("DRAFT … pending independent review") must be updated in the commit.

## Summary table

| ID | Severity | Topic | Blocks |
| --- | --- | --- | --- |
| F1 | P2 | Gate trace coverage, actions_match and frames_contiguous in MEASUREMENT_INVALID | fix or decline before commit (frozen rule) |
| F2 | P2 | Category shares should exclude non-VALID (non-bar) crashes | fix or decline before commit |
| F3 | P2 | Identity controls should require `result_equal` (as G7) | fix or decline before commit |
| F4 | P2 | Run native identity before the trace, or read NOT_REPRODUCED | fix or decline before commit |
| F5 | P2 | Chain watchdog/timeout cannot kill the launcher session | fix before launch (outside the repo) |
| F6 | P2 | CPU gate: accept as baseline-equivalence only; post-commit rerun must show exactly the 15 | condition on GPU launch |
| F7 | P3 | Positive control scope; bind the run-commit g3-control | note |
| F8 | P3 | `labels_before_controls` on CONTROL_FAIL | note |
| F9 | P3 | Vertical, pixel-coverage and MinZ approximations | note |
| F10 | P3 | Mirror controls, analysis crash path, environment comparison with E3, GPU idleness per cell, narrow intervals | note |

Files written by this review: `REVIEW.md`, `test_camt1.log`, `g3_positive_control_review.{json,log}`,
`stage_b_controls_review.{json,log}`, `check_g3_consistency.{py,out.json}`, `check_yaw_convention.{py,out.json}`,
`base_fail.txt`, `r2_fail.txt`, `tmp/` (empty test temp dir).
