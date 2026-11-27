# CAM-K1-E v1: independent results review r1

- **Reviewer:** Claude Opus 5.5 (fresh context). Read-only on every repository. I wrote only to this folder.
- **Date:** 2026-11-08, about 14:50–15:05 KST.
- **Tree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `971ae67`.
- **Preregistration:** `docs/plans/prereg_camonly_k1_eval_v1_2026-11-08.md` (at `971ae67`).
- **Code read:** `tools/camk1e_run.py`, `tools/camk1e_analyse.py` and the helpers it imports (`gr_analyse.rates`,
  `camonly_c2_analyse.side/label/causes/governor`, `kroc2027_eval.comparable`, `gr_run.identity_compare`), the launcher
  `eval_navrl_camk1e_arms.sh` (dirty count, line 70), `tools/camtrain_entry.py`, the FOV path in
  `navrl_perception.py` and `kroc_eval_adapter.py`, and the design review `REVIEW_design_r1.md`.
- **Run outputs read:** everything in `~/motar_tmp/camk1e_20261108/`: `chain.log`, `run_camk1e.sh`,
  `execution_approval.json`, `identity_01.log`, `eval_01.log`, `raw/identity_01/` (report and 3 cells),
  `raw/eval_01/` (report and 36 cells), `analysis_01.json` and `.log`, `public_cpu_precommit.log`, `HANDOFF_running.md`.
- **Reference inputs:** GR-0 `gr0_main/S00` (arm A) and `S20`, C2 `c2/no_lidar` (arm C87), and the CAM-K1
  `train_s{1,2}/train_report.json` files.
- **Probes in this folder.** All are my own code and import nothing from `tools/`. They ran with
  `CUDA_VISIBLE_DEVICES=""`, `nice -n 19`, `taskset -c 6,7` and `ulimit -v 8000000`.
  - `probe_integrity.py` / `.out`: checks every cell, both checkpoints, the identity stage, the manifests, the order,
    the host and the tree.
  - `probe_stats.py` / `.out`: recomputes the rates from `episodes.jsonl`, the contrasts, the intervals (a numpy Monte
    Carlo bootstrap with a different RNG, the exact stratified bootstrap over all 27⁴ resamples, and the 8-df t
    interval), the controls and the gate labels. The gate rules are coded from the preregistration's text, and the
    results are compared with `analysis_01.json`.
  - `probe_ref_conditions.py` / `.out`: compares the result conditions and NAVRL snapshots of A with C87 and F66.

## Verdict: **PASS_WITH_LIMITS**

- **Execution:** the run is valid evidence.
  - The approval names the runner's SCOPE and HEAD.
  - IDENTITY_PASS holds on my own comparison.
  - All 36 eval cells pass my cell checks, in the preregistered order.
  - The host fault count stayed at 0 from start to end, and the boot id did not change.
  - The tree was clean while the run was active.
- **Recomputation:** every rate, contrast mean and t interval matches `analysis_01.json`:
  - means and pooled rates to machine precision;
  - t intervals to within 7 × 10⁻⁸ pp;
  - bootstrap ends to within 0.05 pp of my Monte Carlo bootstrap and 0.03 pp of the exact bootstrap.
- **Labels:** all four gate labels and all four controls are the same under all three interval computations. The
  margins are wide. The nearest gated interval end is 25.6 pp from gate R's −10 pp threshold and 8.0 pp from gate P's
  +3 pp threshold.
- **Reading:** the analysis follows the preregistration exactly, and no label is upgraded.
  - Gate R reads **MATERIAL_RECOVERY** for both seeds.
  - Gate P reads **MATERIAL_LOSS** for both seeds.
  - CRASH_TO_TIMEOUT does not apply: the timeout contrast mean is −0.004 pp.
- **Why "with limits":** the findings below concern wording, disclosure and process. None changes a number or a label.

No MAJOR finding. Five MINOR, seven LOW.

## Recomputed numbers

Quota rates, 12 blocks (4 densities × 3 seeds, 2048 quota episodes each). Contrasts are in pp. My bootstrap column is
the exact stratified bootstrap. The analysis's columns are its own 10,000-resample bootstrap and its t interval.

| Contrast (crash) | Mean | Bootstrap, exact (mine) | Bootstrap (analysis) | t, 8 df (mine = analysis) | Label (mine = analysis) |
| --- | --- | --- | --- | --- | --- |
| R, K1s1 − F66 | −37.67 | [−38.46, −36.79] | [−38.45, −36.78] | [−38.89, −36.45] | MATERIAL_RECOVERY |
| R, K1s2 − F66 | −36.74 | [−37.46, −35.96] | [−37.46, −35.98] | [−37.84, −35.65] | MATERIAL_RECOVERY |
| P, K1s1 − A | +11.77 | [11.25, 12.30] | [11.26, 12.30] | [11.01, 12.53] | MATERIAL_LOSS |
| P, K1s2 − A | +12.69 | [12.22, 13.20] | [12.22, 13.21] | [11.98, 13.40] | MATERIAL_LOSS |
| Control: P on C87 − A | +39.95 | [39.64, 40.22] | [39.64, 40.22] | [39.53, 40.36] | MATERIAL_LOSS (required) |
| Control: R on A − C87 | −39.95 | [−40.22, −39.64] | [−40.22, −39.64] | [−40.36, −39.53] | MATERIAL_RECOVERY (required) |
| Control: P on S20 − A | +0.61 | [0.27, 0.96] | [0.28, 0.96] | [0.11, 1.11] | NO_MATERIAL_LOSS (required) |
| Control: R on S20 − A | +0.61 | (as above) | (as above) | (as above) | NEGLIGIBLE (not a recovery, as required) |
| Reported: F66 − C87 (87° → 66°) | +9.49 | [8.66, 10.30] | [8.67, 10.30] | [8.30, 10.68] | not gated |
| Reported: F66 − A | +49.43 | [48.70, 50.17] | [48.70, 50.17] | [48.37, 50.50] | not gated |
| Reported: K1s1 − K1s2 | −0.92 | [−1.52, −0.34] | [−1.53, −0.33] | [−1.78, −0.07] | not gated (C2 scheme: SMALL_LOWER) |
| Timeout, R K1s − F66 (both seeds) | −0.004 | [−0.012, 0.000] | [−0.012, 0.000] | [−0.013, 0.005] | below the 2 pp CRASH_TO_TIMEOUT trigger |

- **Lineage:** gate R reads MATERIAL_RECOVERY and gate P reads MATERIAL_LOSS; both seeds share each label.
- **Blocks:** all 12 R blocks are negative for both seeds (−46.8 to −23.1 pp), and all 12 P blocks are positive
  (+5.0 to +20.0 pp).

**Pooled quota rates (%), mean of 12 blocks, equal to the pooled counts:**

| Arm | Capture | Crash | Timeout |
| --- | --- | --- | --- |
| A (F, camera + LiDAR, 87°) | 95.54 | 4.46 | 0.004 |
| S20 (GR-0 ±20 % gains) | 94.92 | 5.07 | 0.016 |
| C87 (F, `no_lidar`, 87°) | 55.59 | 44.40 | 0.012 |
| F66 (F, `no_lidar`, 66°) | 46.11 | 53.89 | 0.004 |
| K1s1 (CAM-K1 s1, `no_lidar`, 66°) | 83.78 | 16.22 | 0.000 |
| K1s2 (CAM-K1 s2, `no_lidar`, 66°) | 82.85 | 17.15 | 0.000 |

**Crash rate (%) by density, mean of 3 seeds.** Reported only; no density has a gate.

| Bars | A | S20 | C87 | F66 | K1s1 | K1s2 | K1s1 − F66 | K1s2 − F66 | K1s1 − A | K1s2 − A |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 70 | 1.24 | 1.68 | 25.76 | 32.15 | 7.18 | 7.67 | −24.97 | −24.48 | +5.94 | +6.43 |
| 115 | 2.90 | 3.29 | 40.14 | 48.75 | 13.07 | 13.18 | −35.68 | −35.56 | +10.17 | +10.29 |
| 160 | 4.88 | 5.81 | 51.45 | 62.11 | 18.18 | 20.00 | −43.93 | −42.11 | +13.30 | +15.12 |
| 205 | 8.81 | 9.49 | 60.25 | 72.56 | 26.46 | 27.73 | −46.09 | −44.82 | +17.66 | +18.93 |

The gap to A grows with density, and it is largest at 205 bars, CAM-K1's training density.

## Execution integrity (what I verified)

1. **Approval.** `execution_approval.json` names `commit` = `971ae6739bb0…` = HEAD. Its `scope` is the runner's SCOPE
   string byte for byte. The four keys are non-empty. The approval is dated 12:07 KST and quotes the user's AskUserQuestion
   answer, whose option text names "CAM-K1-E 평가". The commit is at 13:08:33 and the chain started at 13:08:54. The
   preregistration was committed before execution.
2. **Identity (stage 1), my own comparison:**
   - `F87_identity` equals C2's `no_lidar` 205/4501 cell. `result.json` is equal after removing the nonce and the
     checkpoint path, and `episodes.jsonl` is byte-identical.
   - `F66_negative` differs from C2.
   - `K1s1_smoke` differs from `F66_negative`.
   - The F66 and K1s1 identity cells equal the eval's 205/4501 cells, in both `episodes.jsonl` and `result.json` (after
     removing the nonce and the checkpoint path).
   - IDENTITY_PASS is confirmed. The identity cells are also valid under my checks. The F87 recorder reads
     vfov = 58.00000000000001, which is inside the runner's 1e-6 tolerance.
3. **Eval cells.** All 36 checks passed (0 issues):
   - **Receipt:** `receipt.json` reads status OK, `git_commit` = HEAD and `git_dirty_files` = 0. The launcher counts
     `git status --porcelain` lines, untracked files included.
   - **Checkpoint per arm:** F66 = `f7022139…`, K1s1 = `d7bf51a8…`, K1s2 = `5128bf2e…`. The SHA agrees in the receipt,
     `result.json`, `NAVRL_EVAL_CHECKPOINT` and the recorder context. I re-hashed the checkpoint files: they equal the
     reported SHA-256s and the TRAINED, non-smoke train reports.
   - **Camtrain receipt:** `no_lidar`, a 72 × 4 LiDAR, and the module and entry SHA-256 equal to their blobs at
     `971ae67`. The task and perception paths are absolute and inside the tree. `overrides_after` = {66, 54, 0.6, 0.03}.
   - **Snapshot:** `no_lidar` and the four values. No `NAVRL_GAIN_RAND*` key, backend `direct_l2_v1`, and the bars,
     seed and quota of the block.
   - **Recorder summary:** camera 66.0 × 54.0, and import origins inside the tree. The cell log has `hfov=66deg` and
     no `hfov=87deg`.
   - **Ledger:** COMPLETE with 0 partition violations. 2048 quota rows: 16 in each of 128 environments, and exactly
     the first 16 by episode index. The rows' SHA-256 matches, and the quota counts equal mine.
   - **Bindings against C87:** the result condition minus the nonce equals C87's. Across all 36 cells, the snapshot
     differs from C87's only in the ten expected keys.
4. **Arms differ.** In every block, F66 differs from C87, each K1 seed from F66, and K1s1 from K1s2.
5. **Order and timing.**
   - The cells ran by density, then seed, then arm, without overlap.
   - The eval took 13:17–14:52, inside the 3 h cap.
   - Free disk went from 2.64 to 2.18 GB.
6. **Host.** The fault count was 0 at identity start and end, at eval start and end, and after every cell. It is 0 in
   the kernel log now. The boot id is the same throughout, and the online cores are `0-3,6-23` (cpu4/5 offline).
7. **References.**
   - GR-0's S00 and S20 cells (48 files) and C2's `no_lidar` cells (24 files) match their committed manifests.
   - The result conditions of A, C87 and F66 are identical at every block.
   - A's snapshot differs from C87's only in `NAVRL_SENSOR_ABLATION` and bookkeeping keys, and from F66's also in the
     four camtrain keys.
8. **Analysis bytes.** The SHA-256 of `tools/camk1e_analyse.py` (`e9cfe4b6…`) equals its blob at `971ae67` and its
   recorded `script_sha256`.
9. **Freeze.** The tree is dirty now, but every currently dirty path has a ctime of 14:53:14 or later, after the
   chain's `CHAIN DONE` at 14:52:13. The worktree's index was last written at 14:55:37. Together with
   `git_dirty_files` = 0 in all 39 receipts, this shows the freeze held during the run. The dirty paths are another
   unit's files, copied with preserved mtimes from `MOTAR-realveh` (see m5).
10. **Repairs from design review r1 in effect on this data.**
    - M1: the task and perception paths are checked.
    - M2: the intervals are stratified.
    - M3: gate P's band is 3 pp.
    - M4: the condition and snapshot bindings hold.

## Findings

### MINOR

**m1. The approval's review condition was not fully met before launch.**
- The user's 12:07 answer approved each unit "only if it passes the Opus review and the preflight" (Korean in the
  original).
- Design review r1 read **AMEND_FIRST**. The prereg discloses that "the repaired files were not reviewed again before
  launch. Stage 1 is the execution check."
- This review confirms that the four MAJOR repairs are in effect on the real data and that the analysis implements the
  prereg's rules. It did not re-review the full repaired design or rerun `tests/test_camk1e.py`.
- The results record should state this plainly. Whether it satisfies the approval's condition is the user's decision.

**m2. The intervals exclude training-seed variation, and the two seeds do differ.**
- K1s1 − K1s2 crash is −0.92 pp: bootstrap [−1.53, −0.33], t [−1.78, −0.07].
- Each seed's interval describes these 12 blocks for that checkpoint only.
- A write-up must not:
  - pool the seeds into one interval;
  - call a per-seed interval an interval for "the CAM-K1 recipe";
  - say the seeds are "identical".
- It may say that both seeds read the same labels.
- The difference is small next to the gate margins (≥ 25.6 pp for R, ≥ 8.0 pp for P).

**m3. Training-rollout rates are not comparable with these evaluation rates.**
- In the last 100 training epochs, CAM-K1 rolled out at 26.3 % / 25.6 % capture (73.7 % / 74.5 % crash) at 205 bars.
  This was under PPO exploration noise, with the training target (`pattern=mixed dynamics=legacy` in
  `train_s1/launcher.log`) and without the A3 visible-start placement.
- At 205 bars here, the evaluation crash rates are 26.46 % (K1s1) and 27.73 % (K1s2), so capture is about 73 %.
- The near-mirror numbers invite a misreading.
- No sentence may set training-rollout rates beside these rates, or use the gap to argue about training length,
  exploration, or whether more epochs would help. The last of these is not tested.

**m4. Derived "share recovered" ratios and K1 − C87 are not preregistered and mix two changes.**
- F66 − A (+49.43 pp) combines removing the LiDAR with narrowing the camera from 87° to 66°, the governor's camera
  corridor included. A is at 87°, not 66°.
- A ratio such as "CAM-K1 removes about three quarters of the camera-only loss" therefore mixes two changes. Here it
  is 0.762 and 0.743, computed descriptively in `probe_stats.out` only.
- K1s − C87 is not a preregistered contrast either.
- Neither may appear as a finding. If the record wants either one, it must come from a machine-readable record, be
  labelled exploratory, and name both changes.
- Gate P's MATERIAL_LOSS must be worded as a loss: "about 12 pp more crashes than F with camera + LiDAR at 87° on this
  grid". Words such as "close to", "approaches" or "nearly matches" F are excluded.

**m5. The records commit could sweep in another unit's files.**
- The run's worktree now has S4F-1's files staged: `WORKLOG.md`, `docs/EXPERIMENTS.md`, `docs/plans/prereg_sitl_s4f1_v1_2026-11-08.md`,
  `tools/sitl_s4f1.py`, `tests/test_sitl_s4f1.py` and three `docs/evidence/sitl_2026-11-05/s4f1/` files.
- `tests/test_navrl_sensor_ablation.py` is modified, and the CAM-K2/realveh files are untracked, five of them under
  `aerial_gym/` and `resources/robots/`.
- A plain `git add <CAM-K1-E files>; git commit` would also commit the staged S4F-1 files. The CAM-K1-E record will
  also edit `WORKLOG.md` and `docs/EXPERIMENTS.md`, which already hold the other unit's staged hunks.
- Commit the two units separately, by path (AGENTS.md: "Separate independent research units into separate commits").

### LOW

**l1. "REPORTED" is not a preregistration label.**
- It is the analysis script's status for "controls passed, gate labels reported".
- The prereg says the readings are CONDITIONAL until the results review ends.
- After this review, record the gates by their prereg labels: gate R MATERIAL_RECOVERY and gate P MATERIAL_LOSS, each
  for both seeds and as the lineage reading. Name this review with its verdict.

**l2. The concurrent CPU window is not recorded yet.**
- During the eval, other CPU work ran: the S4F-1 and CAM-K2 design reviews, probes and test runs, about 13:13–14:20 by
  file times in `~/motar_tmp/s4f1_20261108/` and `~/motar_tmp/camk2_20261108/`. The files I read show
  `CUDA_VISIBLE_DEVICES=""`, nice 10 or 19, and cores 6,7.
- AGENTS.md's T-LOAD rule allows this. The prereg (l3) requires the window to be recorded, and I found it in no record.
- Within this run, the outcome is invariant: the identity cells run at 13:08–13:17 equal the eval's 205/4501 cells run
  later.
- No wall-clock quantity (`duration_s`, `wall_s`, step time) may be used from this unit.

**l3. PUBLIC_CPU read FAIL at the commit, by its own label.**
- The run had 3,575 tests, 0 failures and 2 errors, both from `test_boia_planner`'s 3 GB free-disk floor.
- The commit message discloses it. The results record should repeat it.
- I did not rerun the suite, because the tree is now dirty with another unit's files.

**l4. The prereg's precomputed A − C87 upper end is −39.65; the final value is −39.640 (−39.64).**
- The analysis draws all bootstraps from one RNG stream in contrast order, so the ends depend on that order.
- My exact bootstrap gives −39.644. The difference is immaterial.
- The prereg stays as committed.

**l5. `result.json` does not carry the arm.**
- Its `condition` records neither the sensor condition nor the camera FOV: A, C87 and F66 are identical there.
- An arm's identity rests on the receipt snapshot, the camtrain receipt and the recorder summary. The record's
  bindings should name those files, not `result.json` alone.

**l6. The pixel aspect is not in the disclosed camera-model limits.**
- The image grid stays 160 × 90 (`NAVRL_CAMERA_WIDTH/HEIGHT` at their defaults) while the FOV becomes 66° × 54°.
  Perception's intrinsics are then fx ≈ 123.2 px and fy ≈ 88.3 px: non-square pixels.
- At 87° × 58° they are 84.3 and 81.2.
- Add this to the camera-model disclosure, next to "one camera for RGB and depth, Euclidean ray depth".

**l7. Label the source of the crash causes and keep the governor telemetry descriptive.**
- The crash causes come from `result.json`'s legacy window: all actual episodes, about 24,590 per arm. They are not
  the quota rows behind the rates. Label them so.
- The governor telemetry differs strongly between arms:
  - perceived clearance: K1 about 6.6 m, F66 0.44 m;
  - intervention rate: K1 0.36, F66 0.96.
- The governor is part of the closed loop measured by R.
- Any mechanism wording is at most "consistent with", and no causal claim is made.

## What may and may not be said

**May be said** (prereg template, numbers from `analysis_01.json`, bootstrap then t):
- "On this grid, in simulation, CAM-K1 seed 1 changes the crash rate by −37.67 pp [−38.45, −36.78; t −38.89, −36.45]
  against camera-only F at the same 66° camera (gate R: MATERIAL_RECOVERY), and by +11.77 pp [11.26, 12.30; t 11.01,
  12.53] against F with camera + LiDAR at 87° (gate P: MATERIAL_LOSS)."
- Seed 2: −36.74 pp [−37.46, −35.98; t −37.84, −35.65] (MATERIAL_RECOVERY); +12.69 pp [12.22, 13.21; t 11.98, 13.40]
  (MATERIAL_LOSS).
- "Recovery" is allowed, because both seeds read MATERIAL_RECOVERY. It must be stated as recovery against camera-only F
  at 66°. It is not a recovery to the camera + LiDAR level, which gate P rejects.
- Both seeds share each label.

**May not be said:**
- anything about the real vehicle, real images, the OAK-D Pro's depth or real flight;
- that the LiDAR is unnecessary, or that CAM-K1 matches, approaches or is equivalent to F with camera + LiDAR;
- that camera-only training alone caused R. K1 − F66 measures CAM-K1's whole recipe: `no_lidar` training, the 66° FOV,
  the two reward weights and 3,000 more epochs;
- that more training would or would not help;
- an effect from one seed alone, or a pooled interval for the recipe (m2);
- comparisons with training-rollout rates (m3);
- derived "share of the loss recovered" ratios, or K1 − C87 as a finding (m4);
- generalisation beyond these four densities or this grid. The intervals describe these 12 blocks.
- that the 3 pp band is met at any density. Per-density numbers are reported, not gated, and the K1 − A gap is +5.9 pp
  even at 70 bars.

## Disclosures the results record must carry

1. Who reviewed: this review (Claude Opus 5.5, fresh context) and its verdict. Until it was recorded, the readings
   were CONDITIONAL.
2. Design review r1 read AMEND_FIRST, and its repairs were not re-reviewed before launch (m1).
3. Reference A is F with camera + LiDAR at 87°, not at 66°. It and C87 are cells from earlier commits, admitted through
   C1's identity and stage 1's identity pass.
4. There are two training seeds. The intervals are over evaluation blocks within four fixed densities and exclude
   training-seed variation. K1s1 − K1s2 is −0.92 pp, an interval excluding 0 (m2).
5. K1 − F66 measures the whole CAM-K1 recipe. F66 differs from C87 in the FOV, including the governor's camera corridor;
   the reward weights affect the reward only.
6. Vehicle: navrl_quad (0.25 kg, T/W 3.26), not the user's 0.5 kg vehicle.
7. Camera model: one camera for RGB and depth, Euclidean ray depth, no stereo minimum depth, noise or holes, and
   non-square pixels from 160 × 90 at 66° × 54° (l6). The detector is F's colour rule on a red sphere.
8. CAM-K1 trained at a fixed 205 bars, with a different target-motion condition. Training-rollout rates are not
   comparable (m3).
9. Concurrent CPU work ran during the eval. Record its window and conditions (l2). No wall-clock quantity is used.
10. Host faults stayed at 0, with no reboot. The tree was clean during the run (receipts, ctimes).
11. PUBLIC_CPU at the commit: 0 failures and 2 errors from the disk floor (l3).
12. Bindings: the SHA-256s of `analysis_01.json`, `eval_run_report.json` and `identity_report.json`, and the raw paths
    outside the repository.
13. Crash causes come from the legacy window (l7). The governor telemetry is descriptive only.
