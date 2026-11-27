# Pre-measurement design review: camera-only study C1/C2

- **Reviewer:** Claude Code, a fresh-context review agent. **Model:** Opus 5.5 (`claude-opus-5-5`).
- **Date:** 2026-11-04, about 19:27-20:10 KST.
- **What I reviewed:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, commit `1908fd8`.
  - The unit under review is `git diff 959d9c0 1908fd8`.
  - I also read the files it reuses at `1908fd8`: `tools/gr_run.py`, `tools/gr_analyse.py`, `tools/kroc2027_eval.py`,
    `tools/camonly_entry.py`, `tools/camonly_python`, `navrl_sensor_ablation.py`, and the relevant parts of
    `navrl_task.py` and `navrl_perception.py`.
  - Also read: the plan with its r1 and r2 sections, the C0 review r2, and the GR-0 S00 cells (read-only).
- **Mode:** read-only.
  - I read the files through `git show`, `git diff` and `git grep`, plus `cat` and `grep` on GR-0's raw cells.
  - I wrote only inside `review_design/audit/`. I started no GPU process and touched no process, no `results/`
    folder and nothing of `MOTAR-gainrand`.
  - Disclosure: I ran `git status --short` once at the start. It writes nothing in the worktree, though git may
    refresh its index under the main repository's `.git/worktrees/`. I also ran two trivial read-only JSON reads of
    GR-0 files with `/usr/bin/python3`, without the CPU prefix. Every later Python run used the full CPU prefix.
- **What my findings rest on:** the code, the diff and my own CPU probes (in `audit/probes/`). I did not use the
  WORKLOG or the author's summaries.

## Verdict: **AMEND_FIRST**

The machinery is sound and I found nothing blocking:
- the launcher copy differs from the pinned launcher only by the three marked changes;
- the ablation module, the entry point and the wrapper are byte-identical to the C0-reviewed `842fcf5`;
- the receipt chain works;
- the analysis arithmetic is right: my independent recomputation with a different synthetic effect per condition
  matches it to 0.0 pp;
- GR-0's S00 cells match the committed manifest (85 of 85 files).

Five design items should still change before C2 measures anything. Each is a change to the documents and the
analysis (and, for M3, a record kept by the runner). None touches the ablation module.
- **M1:** the preregistration's own fourth question ("from perception only, with the governor keeping it") has no
  contrast.
- **M2:** nothing shows in the simulator that the governor ablation engaged. A dead governor wrapper passes every
  binding (probe).
- **M3:** nothing checks that C2 ran under the same non-NAVRL environment and recorded condition as C1 and GR-0.
  Without that check, C1's identity does not carry over to the C2 cells.
- **M4:** the label rule has no minimum size and no rule for equivalence. Effects below 1 pp get a label, and
  NO_DETECTED_CHANGE is easy to misread as "no effect".
- **M5:** C2 cannot separate camera quality from the in-FOV LiDAR target association. The "may not be said" list
  must say so.

Procedural consequence: any amendment commit changes HEAD. The runner (`tools/camonly_run.py:129-134`) and the
analysis (`tools/camonly_c2_analyse.py:79`) then require a new C1_IDENTITY_PASS on that commit. That is 2 cells,
about 5-6 min. The C1 now running on `1908fd8` will not be accepted as C2's gate after an amendment.

## Findings

### Blocking

None.

### Medium (should be in a dated amendment before C2 runs)

**M1. Question 4 ("from perception only, with the speed governor keeping it") has no contrast. Add `obs_all = C_gL − A`.**
- Where: `docs/plans/prereg_camonly_c1c2_v1.md:14-17` (the four questions) against `:88-96` (the contrasts);
  `tools/camonly_c2_analyse.py:39-41`.
- The four questions are "everywhere", "inside the FOV", "outside it" and "from perception only". They map to
  C − A, B_in − A, B_out − A, and naturally C_gL − A.
- The table has `obs_in` (B_in_gL − A, in-FOV only) and `gov_all` (C − C_gL), but no C_gL − A. C_gL appears only as
  a pooled rate without an interval or a label.
- `obs_all` also completes the decomposition total = obs_all + gov_all, the counterpart of in_fov = obs_in + gov_in.
  In my probe the analysis computes no such interval; the derived value was −12.9 pp.
- Fix: add `obs_all: {"C_gL": 1, "A": -1}`, which has OOD terms.

**M2. No in-simulator positive control for the governor ablation. A dead governor wrapper passes every binding.**
- Where: `tools/camonly_c2_analyse.py:98-103` (the only "is the ablation active" check is "differs from full");
  `tools/camonly_run.py:152-154` (C1's positive control is `no_lidar` only);
  `aerial_gym/task/navrl_task/navrl_sensor_ablation.py:170-203` (the governor wrapper).
- Why C1 cannot catch it: `no_lidar` masks perception too. So "episodes differ from GR-0" holds even if the
  governor wrapper never runs.
- The ablation receipt (`tools/camonly_entry.py:34-43`) proves only that `install()` ran. Nothing proves that the
  wrapped `_apply_sensor_speed_governor` was used.
- Probe (`probes/analysis_probe.json`, "governor_dead"): I made B_in_gL bit-identical to B_in and C_gL to C in
  every block. That is what a dead governor wrapper produces, because then both members of each pair run the raw
  governor with the perception mask.
  - The analysis reads **REPORTED**, with `gov_in` and `gov_all` both NO_DETECTED_CHANGE.
  - So a machinery fault would read as the finding "the governor's LiDAR does not matter".
- The code path looks correct by reading:
  - the wrapper replaces the class attribute and the module global;
  - `navrl_task.py:4967` calls `self._apply_sensor_speed_governor`;
  - `navrl_task.py:5332` looks up the global `directional_lidar_clearance` at call time.
  So this is a gap in fail-closed coverage, not a known bug.
- Fix (analysis only):
  - binding: for every block, episodes(B_in) ≠ episodes(B_in_gL) and episodes(C) ≠ episodes(C_gL);
  - also report each condition's `result.json` `speed_governor` telemetry (`mean_clearance_m`, `intervention_rate`,
    `mean_executed_speed_mps`; the block starts at `navrl_task.py:9696`), as reported and not gated. B_out and C
    should show the K2 floor; C_gL and B_in_gL should show the raw-LiDAR governor.

**M3. C1's identity carries over to C2 only if C2 runs under the same inherited environment. Nothing records or
compares it, and no C2 cell's recorded condition is compared with its GR-0 cell.**
- Where:
  - `tools/gr_run.py:101-111`: `cell_env` keeps every inherited non-`NAVRL_` variable;
  - `tools/kroc2027_eval.py:48-54,98-106`: only a fixed list is scrubbed;
  - `aerial_gym/rl_training/rl_games/eval_navrl_camonly_arms.sh:33,74`: `PYTHON` and `AERIAL_GYM_SIM_NAME` are taken
    from the caller when set.
- Example: an `AERIAL_GYM_SIM_NAME=base_sim_4gb` left in the C2 shell would change the simulation config
  (`result.json` records `runtime_sim_config_class`, `BaseSimConfig` in GR-0). C1 compares this for one cell only.
  No C2 check compares it.
- The receipts' `env_snapshot_navrl` holds only `NAVRL_*`, and the ablation receipt holds neither the interpreter
  nor `PATH`. Review r2 L-r2-2 asked that C1 record `PATH`; this was not done and not declined.
- Fix:
  - (a) Analysis binding: for each C2 cell, `K.comparable(result)["condition"]` equals the matching GR-0 S00 cell's.
    The condition dict carries no ablation field, so it must be equal.
  - (b) Analysis binding: `env_snapshot_navrl` equals GR-0's, except the expected keys `NAVRL_SENSOR_ABLATION`,
    `NAVRL_SENSOR_ABLATION_RECEIPT`, `NAVRL_BULK_EVAL_JSON`, `NAVRL_EVAL_RUN_NONCE` and `NAVRL_KROC2027_RUN_ID`.
  - (c) The runner records `PYTHON`, `PATH`, `AERIAL_GYM_SIM_NAME`, `CUDA_*` and a hash of the inherited non-NAVRL
    environment in `c1_report.json` and `c2_run_report.json`. C2's preflight requires them equal to C1's.

**M4. The label rule labels negligible effects and has no equivalence reading. The bootstrap is narrower than the
t-interval, and no rule says what happens when they disagree.**
- Where: `docs/plans/prereg_camonly_c1c2_v1.md:83-84,98-101,111-116`; `tools/camonly_c2_analyse.py:50-51,115-117`.
- From GR-0's S00 blocks (`probes/gr0_blocks.json`):
  - The block SD of paired level differences is 0.69 pp (S20) and 1.24 pp (S40). Within a density it is 0.58-0.77 pp.
  - The binomial SD of one paired cell difference is 0.32-0.91 pp.
  - GR-0's S20 effect of −0.62 pp got a CI excluding 0 (bootstrap [−1.01, −0.26]). Under this preregistration it
    would read LOWER.
- My synthetic `obs_in` of −0.96 pp also reads LOWER.
- Power (12 blocks, paired):
  - the label threshold is |mean| > 0.54·SD (bootstrap) or 0.64·SD (t);
  - the 80 % minimum detectable effect is 0.89·SD;
  - for a block SD of 1 / 3 / 5 pp this is 0.9 / 2.7 / 4.4 pp.
  - **So 12 blocks are enough for practical sizes (≥ 3 pp) unless density heterogeneity drives the block SD above
    about 5 pp.** The risk is the opposite one: labels for sub-1-pp effects.
- The percentile bootstrap is about 14 % narrower than the t-interval (measured: 0.75 vs 0.87 pp, 1.35 vs 1.58 pp).
  The preregistration calls t a "sensitivity check" but does not say what a disagreement means.
- Fix:
  - (a) Add a practical band, e.g. GR-0's ±3 pp for capture and ±2 pp for crash. Label "NEGLIGIBLE" when the CI lies
    inside the band. Report a label outside the band as "detected, below practical size" when |mean| is inside it.
  - (b) A label stands only if the bootstrap and the t-interval agree. Otherwise read NO_DETECTED_CHANGE (fragile).
  - (c) Add to "may not be said": "that removing the LiDAR [where] has no effect", unless the CI lies inside the
    band.

**M5. In-FOV "camera quality" is not separated from the in-FOV LiDAR target association; the rules must say so.**
- Where: `docs/plans/prereg_camonly_c1c2_v1.md:70` (the association-off condition is not run), `:91`, `:94`;
  plan `:99-100` (K5: "the camera-quality part … lives entirely inside the FOV") and `:258-266` (M2 correction).
- `in_fov` and `obs_in` both include:
  - the camera's misses and range;
  - the in-FOV association;
  - the governor's target carve-out after a camera loss.
  `in_fov` also includes the governor's phantom immunity and the M-r2-2 bearing offset.
- The readings list these parts, so they are correct. But the "may be said" template ("removing the LiDAR [where]
  changes …") and the user's question ("is camera quality separable from coverage?") invite a reading of
  `in_fov` / `obs_in` as "camera quality".
- C2 separates **by bearing (in-FOV vs out-of-FOV)**. Inside the FOV it does not separate camera quality from
  association.
- Fix: either run the optional `NAVRL_LIDAR_TARGET_ASSOC=0` condition (12 cells, about 30 min), or add to "may not be
  said": "that `in_fov` or `obs_in` measures the camera's resolution or range alone".
  - The sensor-level CPU check (plan K5) is what gives "camera quality" its meaning. It is not part of C1/C2 and
    should be named as a precondition for that reading.

### Low (recommended; fold into the same amendment where cheap)

- **L1. Condition A's inputs are not bound** (`tools/camonly_c2_analyse.py:67-77`).
  - The analysis takes GR-0's folder from argv. It does not check that the GR-0 report is COMPLETE or that its S00
    records are OK, as `gr_analyse.py:49-50` does. It also does not check that the folder is the `gr0_dir` that C1
    and C2 recorded (`camonly_run.py:126`).
  - Probe ("wrong_gr0_dir"): a fake GR-0 folder whose S00 holds the S40 cells and whose report reads INCOMPLETE gives
    **REPORTED**, with total = −19.8 pp computed against the wrong A.
  - Fix: require `gr0 == c1.gr0_dir == c2.gr0_dir`, a COMPLETE GR-0 report, all S00 records OK, the 12
    preregistered blocks, and the S00 bytes against
    `docs/evidence/gain_randomization_2026-11-04/SHA256_MANIFEST.json`. I checked these bytes myself: 85 of 85
    match.
- **L2. The M-r2-2 disclosure is imprecise** (`prereg_camonly_c1c2_v1.md:121`; plan `:310-311`).
  - The angle-to-column offset is not a residual "shared by every condition" that cancels. In A, min-fusion with
    the LiDAR hides much of it. Removing the in-FOV LiDAR exposes it.
  - In B_in and C it also reaches the governor (review r2: "it belongs to B_in_gL − B_in").
  - So it is part of `in_fov`, `obs_in`, `gov_in`, `total` and `gov_all`. Say that.
- **L3. Reading text for OOD and K2** (`prereg_camonly_c1c2_v1.md:59-69,91-96,111-112`).
  - `out_fov`, `total` and `interaction` also contain the K2 rule (out-of-FOV commands capped at 2.0 m/s). Their
    governor share is not separated, because there is no `no_lidar_out_fov_obs_only`.
  - B_in and B_in_gL are marked "OOD: no". They are still a shift: no in-FOV return at 10-12 m, camera misses, and
    no in-FOV LiDAR confidence. Better: "zero-shot, smaller shift, not shown to be in distribution".
  - The "may be said" sentence does not fit the difference-of-difference contrasts (`interaction`, `gov_in`,
    `gov_all`). Give each its own sentence.
- **L4. Deviation from K3 not disclosed** (plan `:73` against `:341-346`).
  - The user-approved K3 is a 2 × 2 of observation {fused, camera} × governor {LiDAR, camera}. C2 has three of its
    four cells: A, C_gL and C. Fused perception with a camera governor is missing.
  - So the governor's share is measured only under camera-only perception (`gov_all`) and in-FOV (`gov_in`), and
    the obs × governor interaction cannot be estimated.
  - Adding the missing cell needs a new module condition and a C0 review, so disclose it rather than add it.
- **L5. Budget and timeouts** (`prereg_camonly_c1c2_v1.md:41`; `tools/camonly_run.py:114,159-166`;
  `tools/gr_run.py:53`).
  - The GR-0 S00 cells took 108-176 s (mean 143 s). So 60 cells need about 2.4 h of a 3.0 h cap, a margin of about
    25 %, and conditions with more timeouts would be slower.
  - The cap is checked only between cells (`CELL_TIMEOUT_S` = 1,800 s), so a run can reach about 3.5 h.
  - C1 never checks a budget; its 1 h comes from 2 × 1,800 s.
  - No rule says what follows an INCOMPLETE C2. Fail-closed (EVAL_BINDING_FAIL) is right, but state the rerun rule:
    a new folder, the same commit, the same C1.
- **L6. Reported items** (`tools/camonly_c2_analyse.py:54-63`; plan `:169`).
  - The crash-cause table counts all `actual_episodes` (2,050 per cell), not the 2,048 quota episodes. It also drops
    `out_of_bounds` (4 of 186 crashes in the 205/4501 S00 cell).
  - The plan's other reported items are dropped without comment: crashes with the nearest bar inside vs outside the
    FOV, and time to capture. The time to capture is in `result.json` `speed_governor.outcome_steps`. Report it, or
    say it is deferred.
  - Per-density contrasts (as in GR-0's `by_density`) would show how the LiDAR's value grows with clutter.
    Descriptive only.
- **L7. Multiplicity** (`prereg_camonly_c1c2_v1.md:86-101`).
  - The 14 labels are not independent:
    - capture ≈ 1 − crash when timeouts are about 0 (GR-0: ≤ 0.05 %);
    - `interaction` = total − in_fov − out_fov;
    - `gov_in` = in_fov − obs_in.
  - There are about 5 independent contrasts per metric, so under a global null P(≥ 1 false label) is about 23 %.
  - The reading is descriptive. State "per-contrast 95 %, unadjusted", or name `total`, `in_fov` and `out_fov`
    primary and the decomposition secondary.
- **L8. Smoke-test coverage** (`docs/evidence/camonly_2026-11-04/c1c2/smoke_c1c2.py:47-55,72-73`).
  - The smoke test is valid for what it claims: 9 of 9 checks, run on the committed runner and analysis
    (`smoke_01.json` SHAs = `1908fd8`).
  - But only `no_lidar` carries an effect. The other contrasts are exactly 0 with zero-width CIs, so a swapped
    `NAMES` / `CONTRASTS` entry (say B_in ↔ B_in_gL) would pass. My probe with a different effect per condition
    closes that gap: the arithmetic is correct.
  - Not covered as known-bad cases:
    - a C1 `full` cell that differs from GR-0;
    - a module SHA mismatch;
    - receipt paths outside the tree;
    - a snapshot whose condition differs from the receipt's;
    - the M2, M3 and L1 cases above.
  - Because `gr_run.run_cell` is stubbed, the launcher's three changes are exercised only by C1 on the GPU. That is
    acceptable: C1 is the integration test.
- **L9. Free reproducibility check.** C1's `no_lidar` 205/4501 cell and C2's `no_lidar` 205/4501 cell run the same
  commit, condition and seed. Requiring their episodes to be equal shows that ablated cells are deterministic too.
  The reuse of A assumes this only for `full`, but it costs nothing.

### Information (no action)

- **The machinery as built is what the preregistration says:**
  - `diff` of the pinned launcher against the copy shows only the header comment and the three `[camonly]` changes.
  - `PYTHON=${CAMONLY_PY}` applies to the `play_navrl.sh` call only. Every helper uses the real interpreter.
  - Both interpreters must be absolute (`eval_navrl_camonly_arms.sh:36-40`).
  - The per-cell receipt path matches the runner's (`:178`, `camonly_run.py:46`).
  - The launcher SHA pin `1eadf8fb…` equals the committed bytes.
- **The ablation module, `camonly_entry.py` and `camonly_python` are byte-identical at `842fcf5`, `959d9c0` and
  `1908fd8`.** `git diff 842fcf5 1908fd8 -- aerial_gym` adds only the launcher copy.
- **Code between GR-0 (`c56be07`, tree `MOTAR-gainrand`) and C2 (`1908fd8`):**
  - The only runtime change is the removal of `**self._gain_rand_condition()` from `navrl_task.py`. It returns `{}`
    when OFF, so it is a no-op for S00.
  - `gr_run.cell_env` changed only by the `extra` argument.
  - `NAVRL_KROC2027_RUN_ID` differs (`GR0-GR0`, `CAMONLY-C1`, `CAMONLY-C2`). It is read only as recorder metadata
    (`kroc_eval_adapter.py:785`).
  - `result.json` and `episodes.jsonl` contain no tree path.
  - So C1's one-cell identity is a sound bridge for all 12 A cells, provided M3 holds. The differences are
    independent of density and seed, and the simulation was deterministic across trees in GR-0's identity_02, which
    read IDENTITY_PASS, the gate GR-0's main run required.
- **Is C1's `no_lidar` cell a real positive control?** Yes, for the perception path through the real launcher, entry
  point and classes:
  - a missing install fails the receipt check;
  - an install on classes the simulation does not use gives episodes identical to GR-0.
  It does not cover the governor (M2).
- **`sys.path[0]` is `tools/` during the entry point's imports.** My probe found no `tools/` module name that is also
  importable elsewhere (`probes/shadow_probe.json`, 277 names). A `git grep` found none imported by
  `aerial_gym` or `motar_eval` (`probes/tools_import_grep.txt`). So no shadowing.
- **`ablation_problems` treats a missing `module` / `perception` / `task` key as `""`** (`camonly_run.py:54-55`).
  `K.is_inside("")` then resolves to the caller's working directory. The entry point always writes these keys, so
  this is theoretical. A missing key should fail.
- **Order** is density, then seed, then condition. Condition A ran hours earlier, in GR-0. Because the simulation is
  deterministic, that does not matter.
- **The pairing is per block (density, seed).** Episodes diverge after the first per environment, which is fine for
  a bootstrap over blocks. The bootstrap treats density heterogeneity as noise, so it is conservative for the
  grid-mean estimand.

## Answers to D1-D5

**D1 (claim and contrasts).**
- The seven contrasts answer "what difference appears without the LiDAR" zero-shot for F. They separate the LiDAR's
  value **by bearing** (inside vs outside the FOV) and **by role** (perception vs governor), in-FOV and for camera-only
  perception.
- They miss question 4's contrast (M1).
- They do not isolate camera quality from the in-FOV association (M5).
- The readings are correct in substance. They need M4's rules on NO_DETECTED_CHANGE and size, L2's M-r2-2 wording,
  and L3's notes on K2 and B_in.
- The OOD set {B_out, C, C_gL} matches the code. B_in should be called "smaller shift", not "in distribution".
- The obs_in / gov_in split is additive (in_fov = obs_in + gov_in) and labelled honestly. It includes the carve-out
  in obs_in (L-r2-1) and L1/L2 in the obs_only conditions.

**D2 (validity of condition A).**
- Reusing the GR-0 S00 cells is sound. The code difference is a no-op for S00, the files are intact by the manifest,
  and C1 bridges the launcher copy, the entry point, the commit and the environment for one cell.
- The bridge to the other 59 cells holds only if C2 runs under C1's inherited environment and records the same
  condition, which M3 binds.
- C1's `no_lidar` is a real positive control for perception, not for the governor (M2).

**D3 (machinery).**
- Each cell's chain holds: the runner sets the condition; the launcher requires it and snapshots it; the entry point
  installs it and writes a receipt that cannot be overwritten, with the module SHA and paths inside the tree; the
  runner and the analysis check the receipt; the analysis requires each cell to differ from `full`.
- Silently ignorable today:
  - the governor wrapper (M2);
  - inherited non-NAVRL variables (M3);
  - the GR-0 input folder (L1).
- The smoke test is valid but its known-good case is weak (L8). My probe adds a different effect per condition and
  the M2 and L1 known-bad cases.

**D4 (statistics).**
- 12 blocks are enough for practical sizes. The 80 % minimum detectable effect is 0.89 × block SD, about 1 pp at
  GR-0's level of variability.
- The CI-excludes-0 rule needs a practical band and an equivalence reading, plus a rule for bootstrap vs t (M4).
- Multiplicity is moderate: about 5 independent contrasts per metric, and the two metrics are nearly redundant.
  State it (L7).

**D5 (amend before C2), as a new dated amendment, with C1 rerun on its commit:**
1. Add `obs_all = C_gL − A`, with OOD terms (M1).
2. Analysis bindings: B_in ≠ B_in_gL and C ≠ C_gL per block; governor telemetry reported (M2).
3. Analysis bindings: the recorded condition and the NAVRL snapshot of each C2 cell equal GR-0's, apart from the
   expected keys. The runner records `PYTHON`, `PATH`, `AERIAL_GYM_SIM_NAME`, `CUDA_*` and a hash of the inherited
   environment, and C2's preflight requires them equal to C1's (M3).
4. A practical band and NEGLIGIBLE label, a bootstrap/t agreement rule, and "no effect may not be said" (M4).
5. "`in_fov` / `obs_in` are not camera quality alone", or run the association-off condition (M5).
6. Recommended: L1 (bind the GR-0 inputs), L2, L3, L4 (wording), L5 (rerun rule), L6, L7, L8 (extend the smoke test
   with the cases above) and L9.

## Probe files (`audit/probes/`)

| File | What it shows |
| --- | --- |
| `gr0_blocks.py`, `gr0_blocks.json` | GR-0 per-block quota rates, cell durations, paired block SDs, bootstrap vs t widths |
| `check_manifest.py`, `check_manifest.out` | GR-0 S00 cells and run report against the committed manifest: 85 of 85 match |
| `analysis_probe.py`, `analysis_probe.json` | committed analysis (copied, SHA `af80c8d3…`) with a different effect per condition. Independent recomputation: max \|diff\| 0.0 pp. Known-bad: dead governor → REPORTED (M2); wrong GR-0 folder → REPORTED (L1); C1 on another commit → caught; a missing cell → caught |
| `shadow_probe.py`, `shadow_probe.json`, `tools_names.txt` | `tools/` module names that are importable elsewhere: none |
| `tools_import_grep.txt` | `git grep` of top-level imports of `tools/` names in `aerial_gym` and `motar_eval` at `1908fd8` |
| `tree/` | copies of `camonly_c2_analyse.py`, `gr_analyse.py`, `kroc2027_eval.py`, `motar_eval/{__init__,protocol}.py` at `1908fd8`, used by the probe |
| `work_analysis/` | the probe's synthetic C1/C2 folders (about 70 MB). The synthetic draws use Python's `hash()`, so rerunning gives slightly different numbers; the checks do not depend on them |
| `../*.sh`, `../*.py` | `git show` copies of the two launchers, the ablation module, `navrl_task.py`, `navrl_perception.py` and `kroc2027_eval.py` at `1908fd8` |

After the review the worktree was still clean (`git status --porcelain --untracked-files=all`: 0 lines) and HEAD was
still `1908fd8`.
