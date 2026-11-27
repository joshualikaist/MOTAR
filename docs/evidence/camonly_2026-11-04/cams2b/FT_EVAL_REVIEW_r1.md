# CAM-S2b-FT-E v1 — independent design review r1

- Reviewer: **Claude Opus 5.5** (fresh context, read-only; this file is the only file written).
- Date: 2026-11-24. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b`, branch `camonly/s2b`, base `3a453e6`.
- Files reviewed (uncommitted): `docs/plans/prereg_camonly_s2b_ft_eval_2026-11-24.md`, `tools/cams2b_eval_run.py`
  (SHA-256 `37d7b5bf…`), `tools/cams2b_eval_analyse.py` (`f6168e97…`), `tests/test_cams2b_eval.py`.
- Also read: `tools/cams2a_eval_run.py`, `tools/cams2a_eval_analyse.py`, `eval_navrl_cams2a_arms.sh`, `play_navrl.sh`,
  `navrl_yawclip.py`, `navrl_cams2diag.py`, `tools/realveh_entry.py` (HASHED), `navrl_task.py` (restore / contract /
  MISMATCH sites), `runner.py` (LR handling), `tools/cams2b_train.py`, `tools/cams2b_python`, `train_navrl_cams2b.sh`,
  the CAM-S2a-E records under `~/motar_tmp/cams2a_eval_20261124/raw_a1/` and the draft controls JSON.
- Tests: `-m unittest test_cams2b_eval` from `tests/`, CPU, cores 6,7,12,13, no CUDA: **26 tests, OK, none skipped**.
  The real-cell reuse test ran on the YC12k cells present at review time (175 / 4501 and 4502 only; the 205-bar cells
  do not exist yet).

## Verdict: **GO_WITH_FIXES**

No P0. The runner's stage paths, keys and file names match what the launcher, `realveh_entry`, `navrl_yawclip`,
`navrl_cams2diag` and `cams2b_train.py` write. The A60 reuse is sound as designed. Two P1s must be fixed before the commit:
the controls placeholder and output path, and the missing positive control for the REUSE path of the analysis. The P2s
are mostly spurious-refusal risks: each would stop the night's unit without producing a wrong label.

---

## Q1. Will the runner run end to end on the GPU host?

I traced every stage and found no crash, wrong key, wrong path or default-argument trap.

- **Preflight** (`cams2b_eval_run.py:719-760`, `797-850`):
  - It matches CAM-S2a-E's preflight, with `player_pin_problems` replacing `K.static_problems`. That drops only the
    Policy F checkpoint check.
  - It reads `ft_report["preflight"]["branch"]`, `["head"]`, `final_checkpoint.{epoch,file,sha256}`, `run_folder`,
    `max_epochs`, `source_checkpoint.sha256`, `faults_before/after`, `unit/run/mode/verdict` and `preregistration`.
    These are exactly the keys `cams2b_train.py:404-413, 499-512` writes.
  - `settings.receipt` equals `str(UNIT/"train_ft"/"receipt.json")` (`cams2b_train.py:263`).
- **FT environment:**
  - `ft_env_problems` accepts the driver's own `expected_env` (tested, positive control at `tests:55`).
  - The eval passes `NAVRL_YAW_RATE_MAX=1.0472` through, and nothing downstream overrides it.
    - `play_navrl.sh` and the eval launcher do not pin it. Only `train_navrl_v2_search.sh` does, which is why training
      needs `cams2b_python`.
    - The task and the Lee controller read it at import (`navrl_task_config.py:483`,
      `lee_controller_config_navrl.py:7`).
- **Training-only keys passed into FT cells.** I checked all five for inertness in play:
  - `NAVRL_V2_ALLOW_RESUME`: read only by shell launchers.
  - `NAVRL_LEARNING_RATE=7.5e-06`: `resolve_action_learning_rate` returns the override, and `str(float(...))`
    round-trips, so the entry and launcher snapshots stay equal to the receipt.
  - `NAVRL_RESET_DENSITY_WINDOW`, `NAVRL_DENSITY_RESUME_WARMUP`, `NAVRL_DENSITY_CHECK_EPS=16`: they act only in
    `set_env_state`'s density-evidence block and in the promotion path, which returns early with the curriculum off
    (`navrl_task.py:8862-8863`).
- **MISMATCH allowlist on the FT checkpoint.**
  - `get_env_state` saves `cfg_yaw_rate_max` from the *running* config (`navrl_task.py:3097`). The FT checkpoint
    therefore carries 1.0472, and no `CONFIG MISMATCH | NAVRL_YAW_RATE_MAX` line is expected at evaluation.
  - The other contract lines are the same as CAM-S2a-E's (training seed, curriculum keys, P9 seed).
- **Cell checks with the new parameters:**
  - `[yawclip]` prints `limit_radps=%g scale_radps=%g` (`navrl_yawclip.py:84-87`), giving `0.7854` / `1.0472`.
    These parse back to exactly `float("0.7854")` / `float("1.0472")`, which `receipt_problems:335-341, 361-364`
    compares.
  - The recorder uses `task_config.yaw_rate_max` (`navrl_cams2diag.py:145`). Its range (0.1, 3.0) admits 0.7854.
  - `install_task` requires `limit < scale`, and 0.7854 < 1.0472 holds.
  - The hashed-module set uses fixed paths (`realveh_entry.py:58`), so running training through `cams2b_python`
    does not change `sha256.realveh_python`.
- **Identity post-processing** (`:892-905`):
  - It runs after the GPU cells, so an exception there would lose the stage.
  - `reuse_problems` catches unreadable reports, and the per-cell checks catch `OSError`/`ValueError`.
  - `identity_compare` strips the nonce and the checkpoint. `result.json` carries no path, commit or tree field (I
    checked the 79 condition keys of a real YC12k cell). Recorder rows carry no path either: S12k_identity and
    S12k_repeat rows are equal, 2,103 rows.
  - I found no crash path. This code is still untested as a unit (see P2-6).
- **Cell IDs and folders are consistent** between runner and analysis:
  - `cell_id` (`:165`) gives `S2B_FT__…` and `S2A_trained__…`.
  - The launcher's `CELL_ID="${ARM}__…"` uses `ARMS=cfg(arm)["label"]` (`:220`).
  - The analysis paths are `ident_dir/"FT_identity"/…` and `a60_dir/"A60"/…`.

## Q2. Is the A60 reuse sound?

Yes, as designed.

- **Same environment.** `cell_navrl_env(…,"A60") == S2A.cell_navrl_env(…,"YC12k")` and the expected snapshot values
  are equal (tested).
- **Same executed source:**
  - `git diff 3a453e6..HEAD` is restricted to non-executed paths.
  - The cell modules are re-hashed at `3a453e6` from the object store (`committed_sha256_at`).
  - The entry and script must lie in `S2A_E_TREE`.
- **Per-cell checks pass on real cells.** The generalised `receipt_problems` (tree-relative ablation paths,
  parameterised scale and root) accepts the real YC12k cells present. It refuses a real S12k cell and a wrong seed.
- **One exact reproduction:** result, ledger and recorder rows.

Holes and refusal risks:

- **Cells are not bound to CAM-S2a-E's own records.**
  - `s2a_e_report_problems` (`:527-544`) checks one OK stage-A record per block. It does not compare the record's
    `stats.result_sha256` with the on-disk `receipt.json`/`result.json` of the reused cell.
  - Only 175/4501 is reproduced. The other five are bound by the cell checks and the manifest, not by CAM-S2a-E's
    record (P3-1).
- **The evidence for condition 1 is not in the manifest.** `reuse_manifest` binds the 7 cell files but not
  CAM-S2a-E's `eval_run_report.json` / `identity_report.json` (P3-1).
- **The fallback stage `a60` is pointless in one case.**
  - Reuse can be refused because the reproduction differs (`reuse_decision:597-601`), not because of a cell-check
    problem. That means cross-run non-determinism, since code drift is excluded by the allowlist.
  - The common binding `FT_identity == eval FT` (`analyse:248-255`) relies on the same cross-run determinism.
  - The a60 binding `A60_repro == a60 cell` (`analyse:259-266`) is the same comparison again.
  - Running stage a60 (≈45 min) in that case would most likely end in A60_BINDING_FAIL or EVAL_BINDING_FAIL (P2-4).
- **Spurious refusal:** CAM-S2a-E's eval report is written only at the end of its stage. With an idle-GPU preflight it
  cannot be running during this unit's identity stage, so the reuse will not be refused for being early.

## Q3. Decision rules, controls, resolution

- **G1 code** (`analyse:68-79`) matches prereg `:176`. G2 is `SA.gate_y` with the prefix renamed: the same rule as gate
  Y, including `MIN_BINDING_SHARE`.
- **Labels and the questions they answer:**
  - G1 answers "better on crash and capture" with the family's gate-A crash rule and C2 capture scheme.
  - G2 reads SENSITIVE as "loses materially" (crash WORSE: mean ≥ +2 pp with both intervals > 0; or capture LOWER:
    mean ≤ −3 pp with both intervals < 0). This fills the threshold the training preregistration deferred to this
    document (`prereg_camonly_s2b_2026-11-24.md:184-185`).
- **A label reachable for the wrong reason** (P2-3). FT_BETTER via capture HIGHER admits crash NO_DETECTED_CHANGE with
  both crash intervals above 0 and a mean under +2 pp. Example: crash [+0.4, +3.6] t, mean +1.9 pp. That is a
  *detected* crash rise below the floor.
  - It is consistent with gate L of CAM-S2a-E.
  - For a "flight candidate" reading (`:179-180`) it should be disclosed, as `:253` does for TOLERATED's capture loss.
  - Alternatively, FT_BETTER could require the crash intervals not to be entirely positive.
- **Controls.**
  - The six-block, df-4 geometry is right, and the values in `:204-212` match the draft output
    `draft/controls_6blocks_draft1.json` exactly.
  - Adequate as a test of the rule and interval machinery. The known-bad (F66 − K1s1) and a +5 pp shifted null at
    1,024 episodes reach WORSE / LOWER.
  - Caveat: the "null" K1s1 − K1s2 at quota 8 is not a null. Both intervals lie below 0 (t [−3.11, −0.21], mean
    −1.66 pp), 0.34 pp from the SMALL_REDUCTION floor.
  - Its half-widths are a fair resolution estimate. Its requirement ("not MATERIAL_REDUCTION or WORSE") would also
    accept a false SMALL_REDUCTION. This is family-consistent (P3-6).
- **Resolution statement.** The arithmetic is right: SE ≈ 1.45/2.776 = 0.52 pp; P(NEGLIGIBLE | true 0) ≈ P(|Z| <
  1.06) ≈ 0.71; P(WORSE | +3 pp) ≈ P(Z > −1.92) ≈ 0.97.
  - It treats the observed SE as fixed. With df 4 the realised half-width varies, so these are rough. INFERRED is the
    right tag.

## Q4. Non-vacuous checks; missing tests

The refusal checks mostly have positive controls: FT env, receipt deviations, training run, final checkpoint, real-cell
reuse, report check, manifest, stage links, report bindings and rerun-mode end to end.

Missing:

- **P1-2 — No positive control for the REUSE path of the analysis**, which is the expected path for G1.
  - `test_a60_bindings` (`tests:501-507`) tests REUSE only negatively ("an empty manifest never binds").
  - `test_main_eval_rerun_mode` covers RERUN only.
  - Nothing shows that a correct REUSE tree yields all-true `a60_bindings` and a G1 label: the reused cells rechecked
    at `S2A_E_COMMIT` / `S2A_E_TREE` in `load_eval:217-219`, a real 42-entry manifest and `no_a60_rerun_report`.
  - AGENTS.md: "Pair every negative test with a positive control." A defect here would show up only after the run, as
    A60_BINDING_FAIL, and would need an amendment.
- **P2-6 — `reuse_problems()` and the identity-stage block of `main()` are never exercised as a whole.** This covers
  report paths `raw/eval_01/eval_run_report.json` and `raw/identity_01/identity_report.json`, `identity_compare`
  across trees, `diag_equal` and `reuse_manifest`.
  - A CPU test with a fake `G.run_launcher` that writes synthetic cells would exercise every post-GPU line of each
    stage. This is the CAM-S2a-E lesson applied to the runner itself.
  - Also: once all six YC12k cells and CAM-S2a-E's `eval_run_report.json` exist, run `reuse_problems(s2a_receipt)` on
    CPU before the identity stage. That way the non-reproduction part of the reuse decision is known before GPU time
    is spent.

## Q5. Time budget, caps, rate gate

Expected from CAM-S2a-E's `duration_s`:

| Stage | Estimate |
| --- | --- |
| Smoke | ≈ 2 × 100 s |
| Identity | ≈ 352 + 450–600 s |
| Eval | 3 × 175-bar FT (~380 s) + 3 × 175-bar FT45 (~380 s) + 3 × 205-bar FT (~450–600 s) + 3 × 205-bar FT45 + 12 × ~20 s |
| **Total** | **≈ 1.6–1.9 h** |

For the 205-bar FT45 cells: YC12k_negative took 579 s against 434 s for S12k at 205, so a clip slows the cell.

This meets "about 2 h" as an expectation. The caps allow 2.9 h, or 4.15 h with a60. The prereg discloses this
(`:128`).

- **P2-2 — The rate gate is miscalibrated in both directions.** `rate_projection` (`:636-646`) uses 12 × the unclipped
  `FT_identity` at 205 bars.
  - It over-counts the six 175-bar cells, which run 0.6–0.9× the 205-bar time in CAM-S2a-E.
  - It ignores the clip slowdown of the three 205-bar FT45 cells (+33 % for YC12k vs S12k).
  - The pass threshold is `FT_identity ≤ 614 s`, only 6 % above the 579 s of the slowest comparable 205-bar cell.
  - An FT that times out more than CAM-S2a would refuse an eval that fits the cap. This is the "spurious refusal that
    wastes the night".
  - Suggested fix: project `(6·r + 6)·t205·1.10` with r = CAM-S2a-E's measured 175/205 ratio fixed now. Or add an FT45
    205-bar identity cell (≈10 min), which also gives an FT45 determinism binding.
- **P3-5 — The a60 gate under-projects.** It projects 6 × `A60_repro` (175 bars), although half of its cells are at
  205: about 2,320 s projected against ≈2,900 s expected. This contradicts the eval gate's "slower density" rationale.
  The 4,500 s cap still fits.

## Q6. Preregistration text against the code

- **P1-1:**
  - `:197` still reads `__CONTROLS_SCRIPT_SHA__`.
  - The named output `~/motar_tmp/cams2b_eval_20261124/controls_camk1e_6blocks.json` does not exist. Only
    `draft/controls_6blocks_draft1.json` does, at script SHA `749c47f1…`, while the current script is `f6168e97…`.
  - Regenerate it at the final script and fill the SHA before the commit, as `:198-199` requires.
- **P2-1 — Contradiction on G2's independence from A60.**
  - `:98-99` says "G2 does not depend on A60". But `IDENTITY_PLAN` (`:129`) runs `A60_repro` first, and `run_plan`
    (`:873-874`) stops at the first non-OK cell.
  - `identity_verdict` (`:621-625`) requires both cells OK. So an A60-path failure under the new runner (never smoked
    on GPU) gives IDENTITY_FAIL and blocks G2.
  - Fix:
    - Run `FT_identity` first.
    - Make IDENTITY_PASS depend on `FT_identity` only.
    - Treat an A60_repro FAIL as `a60_source = RERUN_REQUIRED`, plus a stop for a60 if the A60 path itself fails.
  - Or state the dependence in the prereg.
- **P2-3** (above): FT_BETTER admits a detected sub-floor crash rise. Add it to "may not be said" and disclose it in the
  reading (`:179-185`).
- **P2-4** (above): say that stage a60 is not run when reuse was refused only because the reproduction differs (the
  unit's determinism premise failed). Or say how that case reads.
- **P2-5 — The analysis tree is not bound.**
  - `recheck_cell` defaults `root=W.ROOT` (`analyse:176-181`, called at `:204` and for rerun A60 cells at `:222`).
  - If the analysis is run from a different worktree than the eval, for example this s2b worktree after an eval in
    `MOTAR-camonly`, every cell fails `is_inside(entry, root)` and the result reads EVAL_BINDING_FAIL.
  - `script_equals_eval_commit` would still pass, because the bytes are equal.
  - Use `ev["tree"]` (and `a60_report["tree"]`) as root, or require `W.ROOT == ev["tree"]` with a clear message. The
    prereg should also name where the analysis runs.
- **P3-2 — Branch.** `BRANCHES` (`:82`) and `:66`/`:152` allow `camonly/s2b`.
  - `cams2b_train.py:48, 356` can only train on `camonly/dev`, and the eval requires the training branch (`:814-816`).
    So `camonly/s2b` is unreachable.
  - The test fixture writes `"branch": "camonly/s2b"` (`tests:81`), a value the driver never writes. Use
    `camonly/dev` in both places.
  - Corollary worth stating: the eval runs in `MOTAR-camonly`, the only worktree that can hold `camonly/dev`, which is
    `S2A_E_TREE`.
- **P3-3 — "exactly `tools/cams2b_train.expected_env`" (`:73`).** `FT_ENV_MAY_DIFFER_KEYS` (`:112-115`) also allows
  `NAVRL_SEED`, `NAVRL_SAVE_FREQUENCY` and the five module seeds. There is no hole in practice, because the driver
  enforces exact equality during training, but the wording overclaims.
- **P3-4 — Context numbers.** `:31-32` "53.3 % against 36.3 %" are `result.json` `outcome` rates over the first 1,024
  finished episodes. With the preregistered per-env quota rates (`gr_analyse.rates`, identity report `stats.counts`),
  the identity cells read **51.7 % (529/1024) and 35.4 % (363/1024)**. Cite the definition or use the quota rates.
- **P3-7:**
  - `:146-148` speculates that the FT checkpoint might carry 3.0 in its contract. `navrl_task.py:3097` shows the saved
    value is the running config's, so this can be stated as not expected.
  - The run report's final trained density is not checked or reported. FT is described as "175 → 205", but a TRAINED
    verdict does not require reaching 205 (`cams2b_train.verdict`). Report `promotions` / the last stage in the
    analysis.
- **P3-8 — Unbound file.** `REUSED_FILES` omits `realveh_receipt.json.veh_rand_init.json`. It exists in every
  CAM-S2a-E cell and is written by the vehicle-randomisation install. Bind it or say why it is not needed.
- **P3-9 — Allowlist refusals.** `S2A_CODE_ALLOWLIST` / `FT_CODE_ALLOWLIST` refuse any commit on `camonly/dev` before
  the eval that touches `ros2/`, `tools/camera_ros2_*` (changed in `b05467a`), `AGENTS.md`, `README.md` or `results/`.
  This is correct fail-closed behaviour, but it would stop the chain. A CPU-only `preflight --dry-run` before the
  night would catch it early.

---

## Findings (most severe first)

| ID | Sev | Finding | Where |
| --- | --- | --- | --- |
| P1-1 | P1 | Controls SHA placeholder unfilled; the named controls JSON does not exist (only a draft at an older script SHA) | prereg:197-199 |
| P1-2 | P1 | No positive control for the REUSE path of `a60_bindings`/`load_eval` (only a negative); the expected G1 path is untested end to end | analyse:154-160, 216-219; tests:501-507 |
| P2-1 | P2 | A60_repro runs first and its FAIL gives IDENTITY_FAIL, blocking G2; contradicts "G2 does not depend on A60" | run:129, 621-625, 873-874; prereg:98-99 |
| P2-2 | P2 | Rate gate 12 × FT205 × 1.1 ignores the FT45 clip slowdown and over-counts 175-bar cells; threshold 614 s is 6 % above the 579 s comparable → spurious refusal risk | run:636-656; prereg:120-121 |
| P2-3 | P2 | FT_BETTER reachable with a detected crash rise < 2 pp (crash NO_DETECTED_CHANGE, both intervals > 0) plus capture HIGHER; not disclosed | analyse:68-79; prereg:176-185, 249-255 |
| P2-4 | P2 | If reuse is refused because the reproduction differs, stage a60 almost surely fails its own repro binding (same determinism premise); rule should say not to run it / how it reads | run:597-601, 787-791; analyse:259-266 |
| P2-5 | P2 | Analysis rechecks cells against `W.ROOT`, not the eval's recorded tree; running it from another worktree → spurious EVAL_BINDING_FAIL | analyse:176-181, 204, 222 |
| P2-6 | P2 | `reuse_problems()` and the post-GPU identity block of `main()` are untested as a unit; no fake-launcher CPU run of the stage flows; check reuse on CPU before the identity stage | run:576-587, 892-905 |
| P3-1 | P3 | Reused cells not cross-checked to CAM-S2a-E records' `result_sha256`; CAM-S2a-E's two reports not in the manifest | run:527-544, 556-563 |
| P3-2 | P3 | `camonly/s2b` is unreachable as the FT training branch; test fixture uses it | run:82; prereg:66,152; tests:81; cams2b_train:48,356 |
| P3-3 | P3 | "exactly expected_env" overclaims; allowlist is a superset | prereg:73; run:112-115 |
| P3-4 | P3 | 53.3 %/36.3 % are result.json rates; quota rates are 51.7 %/35.4 % | prereg:31-32 |
| P3-5 | P3 | a60 rate gate projects from the 175-bar A60_repro for a half-205 grid | run:640 |
| P3-6 | P3 | Quota-8 "null" control is not a null (both intervals < 0, mean −1.66); requirement also admits a false SMALL_REDUCTION | analyse:101; prereg:208 |
| P3-7 | P3 | FT checkpoint contract speculation resolvable (navrl_task.py:3097); FT's final trained density not checked/reported | prereg:146-148; run:449-491 |
| P3-8 | P3 | `realveh_receipt.json.veh_rand_init.json` not in `REUSED_FILES` | run:95-96 |
| P3-9 | P3 | Diff allowlists will refuse flight-runtime/AGENTS/README commits on camonly/dev; add a CPU preflight dry run before the night | run:147-149 |

## Evidence

- Test run: 26 tests, OK, 0 skipped (CPU, cores 6,7,12,13, `CUDA_VISIBLE_DEVICES=""`).
- CAM-S2a-E identity_01 records:
  - S12k_identity 434 s, 363 crashes / 1,024 quota episodes.
  - YC12k_negative 579 s, 529 / 1,024, `clipped_frac` 0.5809, binding share 0.581.
  - `result.json` outcome crash_rate 0.3633 / 0.5332.
- CAM-S2a-E eval YC12k `duration_s`: 115 bars 320–364 s; 175 bars 332 / 362 s.
- Draft controls `draft/controls_6blocks_draft1.json`: all required OK, script SHA `749c47f1…` ≠ current `f6168e97…`.
