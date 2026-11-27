# CAM-K1-E v1: independent design review r1

- **Reviewer:** Claude Opus 5.5 (fresh context), read-only. I wrote only to this folder.
- **Date:** 2026-11-08, about 12:10–12:45 KST.
- **Tree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `814c28a`.
- **Files reviewed (uncommitted, new):**
  - `docs/plans/prereg_camonly_k1_eval_v1_2026-11-08.md`
  - `aerial_gym/rl_training/rl_games/eval_navrl_camk1e_arms.sh` (SHA-256 `bdd0b07b…`, equal to the runner's pin)
  - `tools/camk1e_run.py`
  - `tools/camk1e_analyse.py`
  - `tests/test_camk1e.py`
- **Committed context read:**
  - `tools/camtrain_entry.py` and `tools/camtrain_python`
  - `aerial_gym/task/navrl_task/navrl_sensor_ablation.py`
  - `tools/gr_run.py`, `tools/kroc2027_eval.py`, `tools/camonly_run.py`, `tools/camonly_c2_analyse.py`, `tools/gr_analyse.py`
  - `navrl_task.py` (reward, FOV and `set_env_state`), `navrl_detector.py`, `navrl_perception.py`, `kroc_eval_adapter.py`, `motar_eval/initial_view.py`, `runner.py`, `navrl_players.py`, `play_navrl.sh`, `train_navrl_camk1.sh`
  - the C2 and CAM-K1 evidence folders
- **Raw inputs read:** GR-0 `gr0_main`, C2 `c2`, GR-1 `eval_ctrl`, and the CAM-K1 `train_s{1,2}` reports and receipts.
- **Tests:** `tests/test_camk1e.py` ran 14 tests, all OK, with `CUDA_VISIBLE_DEVICES=""`, `nice 19` and cores 6,7. Log: `test_camk1e.log`.
- **Probes in this folder:**
  - `probe_receipt_cwd.py` / `.out`: the receipt check run against the real CAM-K1 receipt from three working directories.
  - `probe_stats.py` / `.out`: block spread, interval widths and gate labels on existing contrasts.
  - `probe_strat.py` / `.out`: the iid intervals compared with density-stratified intervals.

## Verdict: **AMEND_FIRST**

I found no BLOCKER. The identity design is sound:
- The 87° cell runs the new launcher and entry with F's own values, and its result must equal C2's cell exactly.
- The 66° cell must differ from C2.

The launcher copy, the checkpoint flow and the stop-on-first-bad-cell runner are also sound. Four MAJOR findings should be fixed before the preregistration is committed and stage 1 starts, because the bands and intervals cannot change after measurement:
- One runner check is vacuous, and the tests hide this because their fixture does not match the real receipt.
- The interval design treats the four fixed densities as random. This drives the gate labels.
- Gate P's +10 pp band is lenient, and its wording overstates what NO_MATERIAL_LOSS means.
- The per-cell bindings that let C2 reuse arms from earlier commits were dropped.

## What I verified (positive findings)

1. **The launcher copy is exact.**
   - Its diff against the pinned `eval_navrl_target_behavior_arms.sh` is the header plus the four marked `[camk1e]` changes.
   - Its diffs against `eval_navrl_camonly_arms.sh` and `eval_navrl_gr1_arms.sh` contain only the renamed variables and the checkpoint-from-caller lines. Test `LauncherCopy` rebuilds the base byte for byte.
2. **The reward weights affect the reward only (question 3). I confirmed this by reading the code at HEAD.**
   - `yaw_align_weight` is read once (`navrl_task.py:6059`) and `visibility_bonus` once (`navrl_task.py:6068`). Both only write `self.rewards`, which is returned at `navrl_task.py:6333`.
   - `self.rw` is a float copy taken at construction (`navrl_task.py:1273-1274`), after the entry's override.
   - The env wrapper passes rewards through (`runner.py:94-107`). `NavRLPpoPlayerContinuous.get_action` (`navrl_players.py:184-245`) never reads rewards, and no observation, termination or reset reads them.
   - So, by the code, F66 and the K1 arms differ only in the checkpoint, and the reward weights cannot change an episode.
3. **The FOV override reaches every consumer.**
   - All of these read `task_config.vision` when they are built: the detector camera (`navrl_detector.py:321-342`), the perception intrinsics and fusion mask (`navrl_perception.py:754-781, 1916`), the ablation's FOV mask (`navrl_sensor_ablation.py:59-61, 151-153`) and the recorder (`kroc_eval_adapter.py:796-797, 1404`).
   - The A3 initial view faces the target directly (`motar_eval/initial_view.py:52-56, 140`). The 66° camera therefore does not change the initial-state distribution.
4. **The simulation code has not changed since C2.**
   - `git diff --stat 5522be0 814c28a -- aerial_gym motar_eval tools/kroc2027_eval.py tools/gr_run.py resources` lists only `train_navrl_camk1.sh` (+126).
   - The ablation module (`ea9ccacb…`) and the entry point (`bfe5e425…`) equal the bytes in the CAM-K1 receipts.
   - Together with C1's identity and stage 1, this supports reusing A and C87 (question 4), with the limits in M4.
5. **Checkpoints come from the reports.** The CAM-K1 copies exist, and their SHA-256 values equal the reports' and the committed `train_report_s{1,2}.json` (`d7bf51a8…`, `5128bf2e…`). GR-1 already evaluated checkpoint copies stored outside the tree (its `eval_ctrl` report shows one).
6. **Gate logic matches the table** (`camk1e_analyse.py:63-84`). UNRESOLVED is tested first and cannot overlap the other labels. The t critical value 2.200985 is t(0.975, 11).
7. **The runner cannot pass with zero valid cells.**
   - Identity needs both comparisons present (`camk1e_run.py:266-267`).
   - Eval needs 36 OK records, stops at the first non-OK cell (`:282-291`), and runs density → seed → arm.
   - The analysis needs `cells_expected == 36` and a complete grid per arm, and fails loudly (an exception) on a missing cell.
8. **Budget.**
   - Time: C2's `no_lidar` cells took 147–200 s, and all C2 cells 104–222 s (mean 153 s). The 36 eval cells should take about 90–105 min, up to about 135 min, plus about 6 min for identity. That fits inside the 3 h cap.
   - Disk: about 8.2 MB per cell gives about 0.31 GB for 38 cells, against 3.1 GB free.

## Findings

### MAJOR

**M1. A receipt path check is vacuous and depends on the working directory, and the tests hide it.**
- **Evidence:**
  - `camk1e_run.py:131-136` checks `K.is_inside(abl.get(k, ""), ROOT)` for `k in ("module", "task", "perception")`, and `rec.get(k, "")` for `entry` and `script`.
  - The real entry stores `ablation.install()`'s dict (`camtrain_entry.py:89`), which has no `module` key (`navrl_sensor_ablation.py:216-217`). The CAM-K1 receipts confirm this (keys: condition, hbeams, perception, task, vbeams).
  - `Path("").resolve()` is the working directory, so a missing key passes when the runner runs from `ROOT`. `run_camk1e.sh` runs it from `ROOT`.
  - From any other directory, every valid cell fails with "ablation module outside the tree: None", and stage 1 would read IDENTITY_FAIL.
  - Measured (`probe_receipt_cwd.out`): `cwd=ROOT -> []`; `cwd=review folder -> ['ablation module outside the tree: None']`; `cwd=/ -> same`.
  - The test fixtures add a `module` key the production entry never writes (`tests/test_camk1e.py:74-76, 258-260`). The positive control therefore does not use the production schema.
- **Fix:**
  - Require every checked key to be present and non-empty (`k in d and d[k]`).
  - Check `task` and `perception` (present) and drop `module`: the module is already bound by `ablation_module_sha256`.
  - Build the test fixtures from the real schema, for example by copying `~/motar_tmp/camk1_20261106/train_s1/camtrain_receipt.json` and replacing the values.
  - Add a negative test for a missing key, and run `RunnerSmoke` once from a working directory outside the tree.

**M2. The intervals treat the four fixed densities as random draws, so the gate labels on density-dependent contrasts are mostly set by the design.**
- **Evidence:**
  - `camk1e_analyse.py:87-90` uses `A.ci`, an iid resample of the 12 blocks, and `C2A.t_interval` with 11 df. The preregistration states this at lines 91-93.
  - On real data, C87 − A per seed within one density spreads by 0.8–1.8 pp, but its density means run 24.5 / 37.2 / 46.6 / 51.5 pp (`probe_stats.out`). The block SD of 10.7 pp is almost all the fixed density effect.
  - Measured (`probe_strat.out`) for C87 − A: iid bootstrap [+34.07, +45.48], t11 [+33.13, +46.76]. The stratified alternative (seeds resampled within each density; t from the pooled within-density variance, 8 df) gives [+39.65, +40.22] and [+39.53, +40.36].
  - For homogeneous contrasts the two agree, for example S40 − A at [+3.0, +4.4] against [+3.4, +4.0].
- **Why it matters:**
  - The estimand the "may be said" text names ("on this grid") is the equal-weight mean over 4 fixed densities × 3 seeds. Density heterogeneity adds no sampling uncertainty to that mean.
  - The likely K1 − F66 and K1 − A contrasts vary with density, as C87 − A does. Under the iid scheme a block SD of 8 pp moves the effective cut-offs: MATERIAL_RECOVERY then needs a mean below −15.1 pp, and NO_MATERIAL_LOSS a mean below +4.9 pp.
  - So a −13 pp recovery that grows with density would read SMALL_RECOVERY or UNRESOLVED instead of MATERIAL_RECOVERY because of the design, not because of uncertainty.
- **Fix (before commit):**
  - Make the density-stratified bootstrap and the 8-df t-interval the gated pair, and keep the iid pair as reported sensitivity.
  - Alternatively, keep the iid pair for continuity with C2/GR-0, but state that it treats the densities as random and is conservative, and put the per-density contrasts in the reported set.
  - Either way, name the estimand explicitly.

**M3. Gate P's +10 pp band is lenient for a 4.46 % reference, and the text overstates it.**
- **Evidence:**
  - The preregistration gives gate P as "parity with camera + LiDAR" (line 100) and argues that "a camera-only policy within +10 pp of A would keep its crash rate near F's" (line 105).
  - A's pooled crash rate is 4.46 %. NO_MATERIAL_LOSS therefore admits up to about 14 %, 3.2 times F's rate.
  - On real data, gate P reads NO_MATERIAL_LOSS for S40 − A (+3.69 pp, which GR-0 itself labelled GAIN_SENSITIVE with its 2 pp crash band) and for C87 − C87_gL (+5.05 pp) (`probe_stats.out`).
  - The project's crash materiality band elsewhere (GR-0, C2) is 2 pp.
- **Fix:** do one of the following.
  - (a) Use a margin consistent with the project's crash band: 2–3 pp absolute, or a relative margin such as ≤ 1.5 × A's per-block rate. With M2's stratified intervals this is resolvable.
  - (b) Keep 10 pp but rename the label to `WITHIN_10PP_OF_A`, delete "parity" and "near F's", and add to "may not be said": parity, equivalence or non-inferiority with Policy F.

**M4. Reusing A and C87 across all 12 blocks rests on one identity cell; C2's per-cell reuse bindings were dropped.**
- **Evidence:**
  - C2's analysis (amendment 1, M3) bound each cell's `result.json` condition to the reused arm's (`camonly_c2_analyse.py:161-162, 171`). It also bound the NAVRL snapshot to differ only in an expected key set (`:51-52, 159-160, 172`).
  - `camk1e_analyse.load_eval` (`:105-142`) has neither. It checks only the condition key, the module and entry SHA, `overrides_after` and the checkpoint SHA.
  - Stage 1 covers one block (205 bars, seed 4501). The other 11 blocks of A (recorded in `MOTAR-gainrand` at `c56be07`) and of C87 (recorded at `5522be0`) are admitted by code identity only.
  - Measured: `result.json["condition"]` minus the nonce is identical between GR-1 `ctrl` and GR-0 S00 (another checkpoint), and between C2 `no_lidar` and GR-0 S00 (another ablation). An empty expected difference is therefore implementable now.
- **Fix:**
  - Per eval cell, require `K.comparable(result)["condition"]` to equal that of the matching C2 `no_lidar` and GR-0 S00 cells.
  - Require the NAVRL snapshot to differ from C2 `no_lidar`'s only in {`NAVRL_BULK_EVAL_JSON`, `NAVRL_EVAL_RUN_NONCE`, `NAVRL_KROC2027_RUN_ID`, `NAVRL_SENSOR_ABLATION_RECEIPT`, `NAVRL_CAMTRAIN_RECEIPT`, `NAVRL_CAMTRAIN_HFOV_DEG`, `NAVRL_CAMTRAIN_VFOV_DEG`, `NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT`, `NAVRL_CAMTRAIN_VISIBILITY_BONUS`}, plus `NAVRL_EVAL_CHECKPOINT` for the K1 arms.
  - Record the `git diff --stat 5522be0..<eval commit>` of the simulation paths in the evidence.

### MINOR

**m1. "This stage is also the GPU smoke of every changed runner path" (preregistration line 71) is not true.**
- **Evidence:** stage 1 runs only Policy F, with `CAMK1E_CKPT` at the launcher's own in-tree path (`camk1e_run.py:88-91`). The K1 path is first exercised by eval cell 2: a checkpoint outside the tree, a non-F `CAMK1E_CKPT_SHA`, and a CAM-K1 `env_state` restored through `set_env_state`. The failure is fail-closed, but the claim is inaccurate.
- **Fix:** add an identity cell `K1s1_smoke` (205, 4501, CAM values). It must be OK and differ from `F66_negative`, and it should later equal the eval's K1s1 cell. Otherwise correct the sentence.

**m2. The FOV "in the simulator" evidence is a config echo, and vfov is never checked at run time.**
- **Evidence:** the required `hfov=66deg` line is `vis_cfg.detector_hfov_deg` printed at task build (`navrl_task.py:1502-1511`), not a property of the built camera. The preregistration (line 53) calls it "the built camera's FOV line". Combined with item 3 of the positive findings and the episode differences required in every block, the override does take effect. But vfov is checked only in the receipt.
- **Fix:** also require `kroc2027_summary.json["camera"]` from each cell to read `hfov_deg`/`vfov_deg` 66/54 (87/58 for the identity cell) within 1e-9. The recorder writes these inside the simulator process (`kroc_eval_adapter.py:1404`). Also reword line 53.

**m3. No experiment isolates the reward-only claim.**
- **Evidence:** `F66_negative` changes the FOV and the reward weights together, and the identity table (line 66) lists only "66° × 54°". The claim holds by the code (positive finding 2), but the reported "F66 − C87: narrowing the view" depends on it.
- **Fix:** add the reward weights to the table row, and cite the code lines as the basis. Optionally add one identity cell, F at 87° × 58° with 0.6/0.03, that must equal C2 exactly (about 3 min).

**m4. Several checks have no negative test.**
- **Evidence:**
  - No test makes `F66_negative` equal C2. That should read IDENTITY_FAIL, and it is the identity's own negative criterion (`fake_run_cell` always alters it, `tests/test_camk1e.py:249-250`).
  - The analysis negatives cover only "K1 = F66" and a wrong override (`:230-233`). These bindings are never shown to fail: identity pass, same commit, environment, C2 folder, manifests, checkpoint SHA, identity F66 = eval F66, and the grids.
  - The receipt negatives omit a wrong entry SHA, a wrong LiDAR layout and a wrong snapshot condition.
- **Fix:** add a parametrised test that breaks each binding once and asserts EVAL_BINDING_FAIL with exactly that binding False, plus the identity case `F66_negative == C2 → IDENTITY_FAIL`.

**m5. The CAM-K1 checkpoint SHA-256 values are not pinned.**
- **Evidence:** `camk1e_run.checkpoints` (`:85-107`) trusts whatever TRAINED report sits in `~/motar_tmp/camk1_20261106/train_s{1,2}/`, a mutable folder outside the repository. The analysis compares only with the run report (`:124-133`). The preregistration names `d7bf51a8…` and `5128bf2e…`, and committed byte copies exist in `docs/evidence/camonly_2026-11-04/camk1/`.
- **Fix:** pin both values next to `F_SHA`, or require the external reports to equal the committed copies byte for byte. Add an analysis binding that `ev["checkpoints"]` maps F, K1s1 and K1s2 to the pinned values.

**m6. The analysis does not bind its own bytes or its identity input.**
- **Evidence:** it records `script_sha256` (`camk1e_analyse.py:213`) but never compares it with the committed file at the eval commit. Its helpers (`camonly_c2_analyse`, `gr_analyse`, `kroc2027_eval`) are unbound. `IDENTITY_DIR` is a free CLI argument that is not checked against `ev["identity_report"]` (`camk1e_run.py:247`).
- **Fix:** add bindings that the SHA-256 of `git show <ev commit>:<path>` equals each running file's, and that `ev["identity_report"] == str(ident_dir / "identity_report.json")`.

**m7. SMALL_RECOVERY and WORSE also fire on negligible differences.**
- **Evidence:** on real data, gate R reads SMALL_RECOVERY for B_in − A (−0.69 pp) and WORSE for S20 − A (+0.61 pp) (`probe_stats.out`). The same label also covers, for example, a −25 pp mean whose upper end is −8 pp.
- **Fix:** add a floor. Read NEGLIGIBLE when |mean| < 2 pp, C2's crash band, or the interval lies inside ±2 pp. Rename SMALL_RECOVERY to something like `RECOVERY_UNDER_MARGIN`, and always report the mean with the label.

**m8. The real-data control set is one-sided.**
- **Evidence:** the only real-data control is gate P on C87 − A, which must read MATERIAL_LOSS. AGENTS.md asks that each negative be paired with a positive.
- **Fix:** add free controls from existing data:
  - gate R on A − C87 must read MATERIAL_RECOVERY (known good);
  - gate R on S20 − A must not read MATERIAL_RECOVERY (known null);
  - gate P on S20 − A must read NO_MATERIAL_LOSS, or the renamed label (known good).
  - Also, CONTROL_FAIL if any of them misreads.

**m9. Gate R reads crash only.**
- **Evidence:** a policy that converts crashes into timeouts (loitering) would read MATERIAL_RECOVERY. CAM-K1's training rollouts had timeouts ≤ 0.03 % (camk1 README), so the risk is low, but "recovery" implies task performance.
- **Fix:** allow the word "recovery" only if the capture contrast is not LOWER (C2's scheme) and timeouts did not rise by ≥ 2 pp. Otherwise say "crash reduction". Add to "may not be said": that capture improved, unless the capture contrast shows it.

**m10. The wording limits are incomplete.** The "may be said" sentence (line 119) should change as follows:
- (a) Name A as "F with the 87° × 58° camera + LiDAR". K1s − A mixes three things: the LiDAR removal, the narrower camera and the CAM-K1 recipe.
- (b) K1s − F66 measures the CAM-K1 recipe: camera-only, 66°, 0.6/0.03, 3,000 epochs at 205 bars, LR 5e-6. It does not isolate camera-only training. Add to "may not be said": that the camera-only condition, rather than continued training or the reward change, caused the difference. No matched continuation control exists.
- (c) Two training seeds: the lineage reading is not an estimate of training-seed variability.
- (d) Disclose that F66 − C87 also narrows the governor's camera corridor: commands outside the FOV are capped at the 2.0 m/s floor (`navrl_sensor_ablation.py:181-183`).

**m11. There are no rules for reruns.**
- **Evidence:** the eval accepts any IDENTITY_PASS on the commit in the parent folder (`camk1e_run.py:233-236`). An `identity_01` FAIL followed by an `identity_02` PASS would therefore be admitted silently. There is no rule for an INCOMPLETE or VOID eval.
- **Fix:**
  - State that an IDENTITY_FAIL ends the unit (a rerun only by amendment), and that the eval requires `identity_01`.
  - An INCOMPLETE or VOID eval stays recorded. Its rerun writes all 36 cells to a new `eval_02`, and cells from two attempts are never mixed.

### LOW

**l1. Time against the session clock.**
- The root session clock reads: wrap 15:08 KST, finish_by 15:58 KST.
- The cap is checked only between cells, and a cell may run 1,800 s (`gr_run.py:53`). The 3 h cap can therefore overrun by up to 30 min.
- `max_hours` is neither recorded in the report nor limited to 3.
- With an expected 1.6–2.3 h for the stages, a launch after about 13:00 can cross wrap.
- **Fix:** record `max_hours`, refuse a value above 3, and plan a process handoff.

**l2. No free-disk preflight.** CAM-K1 had its disk guard (K-11). Add a guard of at least 1 GB.

**l3. The preregistration does not state the concurrency rule during the eval.** The receipts carry `duration_s`. State "idle host, or disjoint cores at `nice` with the window recorded" (T-LOAD rules).

**l4. `--approved-commit ""` passes `gr_run.tree_problems`' prefix test** (`gr_run.py:94`). The approval JSON's commit check is the real guard. Requiring at least 7 hexadecimal characters would close this.

**l5. The chain's fault counter differs from the runner's.** `run_camk1e.sh`'s `faults()` regex lacks `mce: [Hardware Error]`, which `G.host()` counts. This is outside the repository but worth aligning.

## Answers to the specific questions

1. **Identity.**
   - The 87° cell proves that the new launcher, `camtrain_python` and `camtrain_entry` (with no-op overrides), at HEAD, reproduce C2's `no_lidar` cell exactly. Its environment also covers the new `NAVRL_CAMTRAIN_*` and `CAMTRAIN_*` variables.
   - The 66° cell plus the bindings (F66 ≠ C87 in every block, identity F66 = eval F66) show the override changes the simulation. By the code, the FOV is the only override that can.
   - The log line is a config echo (m2), and the K1 path is not smoked (m1).
2. **Wrong checkpoint, FOV or condition.**
   - Checkpoint: the launcher refuses a wrong SHA, K1 ≠ F66 and K1s1 ≠ K1s2 are bound, and the train-report seed is checked. The K1 SHA values are not pinned (m5).
   - FOV: a wrong FOV fails the receipt, the log line and F66 ≠ C87.
   - Condition: the entry refuses anything but `no_lidar`.
   - The remaining risk is other NAVRL knobs. The pinned launcher and the scrubbed environment make this unlikely, but nothing binds it (M4).
3. **Reward-only:** verified at HEAD (positive finding 2). No experiment isolates it (m3).
4. **Reusing A and C87:** valid by code identity since `5522be0` plus C1 and stage 1 at one block. Add the per-cell condition and snapshot bindings (M4).
5. **Statistics:**
   - Requiring both seeds and both intervals is conservative, so no multiplicity correction is needed for the intersection-style lineage reading.
   - The iid block intervals are inflated by the fixed density effect (M2).
   - The gate P band is lenient and overstated (M3), and the gate R labels need a negligible floor (m7).
6. **Non-vacuity:**
   - One check is vacuous, and its fixture differs from production (M1).
   - The identity's negative criterion and most analysis bindings have no negative test (m4).
   - The real-data controls are one-sided (m8).
7. **Zero cells and ordering:** no path passes with zero valid cells, and the order is fixed. Rerun admission is open (m11).
8. **Budget:** sane (positive finding 8). Note the session clock and the cap overrun (l1) and the missing disk guard (l2).
9. **Wording:** see M3, m9 and m10.
