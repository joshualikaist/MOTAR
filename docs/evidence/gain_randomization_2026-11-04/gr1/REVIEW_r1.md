# Independent review: GR-1, two 1,000-epoch continuations of Policy F (rand40 vs ctrl) on GR-0's grid

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`). Fresh context, independent of the author.
  2026-11-04, 19:14–19:30 KST.
- **Scope:**
  - the code at `8f64631` (training) and `ab6531b` (evaluation), and `git diff 8f64631 ab6531b`;
  - the GR-1 preregistration and its amendment 1 (`6d8ee65`), the design review it answers;
  - `tools/gr_run.py` (stages `gr1repro`, `gr1eval`), `tools/gr1_analyse.py` and its helpers, `tools/gr1_train.py`,
    `train_navrl_gr1.sh`, the launcher copy `eval_navrl_gr1_arms.sh` vs the pinned launcher;
  - the raw runs: `train_rand40/`, `train_ctrl/`, `repro/`, `repro_02/`, `eval_rand40/`, `eval_ctrl/`, GR-0's
    `gr0_main/`, the console logs, and `run_copy/gr1_analysis.json`.
  - The WORKLOG and the author's summaries were not used.
- **Host:**
  - Kernel fault count (`journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"`):
    **0 before** (19:14, `fault_count_before.txt`) and **0 after** (19:27, `fault_count_after.txt`).
  - Online CPUs: `0-3,6-23`.
  - No GPU process was started. Every probe ran with `taskset -c 0-3,8-11,14-23 nice -n 10 env CUDA_VISIBLE_DEVICES=
    PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 …/aerialgym/bin/python -B` (with `-I` except the analysis re-run). One
    process ran at a time. None imported `aerial_gym`. Only the checkpoint probes imported `torch`, on the CPU.
- **Writes:** only inside `review_gr1/audit/`. The worktree stayed clean at `ab6531b` (`git status --porcelain` empty
  after the probes). The smoke re-run copied about 600 MB of GR-0 cells into `audit/`. I deleted that copy afterwards
  and kept the two result files.

## Verdict: **PASS**

The reading stands as computed:
- **GR1_SUPPORTED**, wording **"reduced gain sensitivity"**, drift qualifier **not triggered**.
- The amendment requires the label **"single training seed per arm"** until a confirmation-seed decision. The
  analysis output does not carry it (L1), so every report of this result must add it.

What I verified:
- All 72 GR-1 cells and both `repro_02` cells are valid under my own checks, which are stricter than the runner's.
- Both evaluation folders are bound to the checkpoints their labels name, through seven independent records.
- The two arms' evaluation runs differ only in the checkpoint.
- Both checkpoints are genuine 1,000-epoch continuations of Policy F. The training runs show no failure or divergence.
- From the ledgers, my own code reproduces every preregistered quantity, CI and t-interval, and the reading and
  qualifiers.
- The committed `gr1_analyse.py` (`b230fb72…`), re-run on the same inputs, gives a byte-identical
  `gr1_analysis.json` (`fa178d72…`).

The reading is not fragile:
- All 12 blocks have the gated sign for P1, P2, P3 and the DiD.
- Under 50 other bootstrap seeds and under the t-interval, every gate and the wording come out the same.

The two Medium findings below limit what the label can be taken to mean. They do not change it.

## Recomputed numbers (mine) next to the analysis's

Units are pp, 12 (density, seed) blocks, quota rows only.
- "Replay" uses one `random.Random(20261111)` stream in the analysis's call order.
- "Fresh" uses an independent stream per quantity.

| Quantity | Analysis: mean, bootstrap CI, t-interval | Mine: mean, replay CI, fresh CI, t-interval | Blocks with sign |
| --- | --- | --- | --- |
| **P1** crash(rand40,S40) − crash(ctrl,S40) | −2.2095, [−2.6774, −1.7619], t [−2.7431, −1.6759] | −2.2095, [−2.6774, −1.7619] (equal), fresh [−2.6774, −1.7619], t [−2.7431, −1.6759] | 12/12 < 0 |
| **P2** capture(rand40,S00) − capture(ctrl,S00) | +1.1963, [+0.7528, +1.7008], t [+0.6378, +1.7548] | +1.1963, [+0.7528, +1.7008] (equal), fresh [+0.7487, +1.7049], t [+0.6378, +1.7548] | 12/12 > 0 |
| **P3** capture(rand40,S40) − capture(ctrl,S40) | +2.2095, [+1.7619, +2.6855], t [+1.6708, +2.7481] | +2.2095, [+1.7619, +2.6855] (equal), fresh [+1.7578, +2.6815], t [+1.6708, +2.7481] | 12/12 > 0 |
| **DiD** (rand40 S40−S00 crash gap) − (ctrl gap) | −1.0173, [−1.3713, −0.7121], t [−1.4020, −0.6325] | −1.0173, [−1.3713, −0.7121] (equal), fresh [−1.3590, −0.7202], t [−1.4020, −0.6325] | 12/12 < 0 |
| Drift: crash(ctrl,S40) − crash(F,S40) | +0.5371, [+0.0529, +1.0742] | +0.5371, fresh [+0.0529, +1.0946], t [−0.071, +1.145] | 9 > 0, 3 < 0 |
| Crash gap S40 − S00 | F 3.695, rand40 2.421, ctrl 3.438 | identical | – |
| Pooled rates (3 policies × 3 levels × 3 outcomes) | – | maximum absolute difference 0.0 | – |
| Crash-cause table (result.json aggregate window) | – | every count identical | – |

- **Range over 50 other bootstrap seeds:**
  - P1 upper bound: −1.78 to −1.75.
  - P2 lower bound: +0.74 to +0.77.
  - P3 lower bound: +1.75 to +1.78.
  - DiD upper bound: −0.72 to −0.71.
- **t critical value:** 2.2009852 (computed here by numerical integration) vs 2.200985 (`gr1_analyse.py:46`).
- **Reading, first match wins:**
  - EVAL_BINDING_FAIL: no. All 13 bindings hold, and I confirmed them independently (R1).
  - MATERIAL_LOSS: no (P2 = +1.20).
  - **SUPPORTED: yes.** P1 −2.21 ≤ −1.5; CI(P1) upper −1.76 < 0; CI(P2) lower +0.75 > −2; CI(P3) lower +1.76 > 0.
  - The same reading follows from the replay, fresh and t intervals.
- **Qualifiers:**
  - CI(DiD) upper −0.71 < 0 gives **"reduced gain sensitivity"**. The t-interval upper bound, −0.63, agrees.
  - |drift| 0.54 < |P1| 2.21, so the result is not "not separable from continuation drift".

## Findings by severity

### Blocking

None.

### Medium (interpretation limits; the reading does not change)

**M1. "Reduced gain sensitivity" follows the rule, but it rests on a 1.0 pp absolute DiD. About half of P1 is a
difference that already exists at nominal gains.**
- Rule: amendment `:66-67`, applied at `gr1_analyse.py:141`.
- Decomposition: P1 −2.21 = −1.19 (S00 crash difference) + DiD −1.02 (`audit/explore_relative_scale_out.json`).
  - With timeouts near 0, the S00 part is about −P2.
  - Only the DiD is a sensitivity effect, and it is below the 1.5 pp size the claim was framed around.
- On a relative scale (exploratory, not preregistered):
  - The pooled S40/S00 crash ratio is 1.60 for rand40, 1.66 for ctrl and 1.83 for F.
  - The per-block log-ratio DiD, rand40 − ctrl, is −0.054 [−0.130, +0.016], a ratio of 0.95. The interval includes
    0, and only 7 of 12 blocks are negative.
  - So the sensitivity reduction is resolved on the preregistered absolute (pp) scale only.
- Recommendation: report the DiD magnitude next to the wording, for example: "reduced gain sensitivity: the ±40 %
  crash excess is 2.42 pp vs 3.44 pp (DiD −1.02 pp [−1.37, −0.71]); about half of the S40 crash difference is
  already present at nominal gains".

**M2. One training seed per arm, and ctrl drifted measurably from F. The evaluation CIs do not cover training
variance.**
- References: preregistration `:89-90`; amendment `:68-69, :85-87`.
- ctrl − F crash:
  - S00: +0.79 pp [+0.48, +1.09], 11/12 blocks positive;
  - S40: +0.54 pp [+0.05, +1.07].
  - The drift is concentrated at 205 bars, the training density: +2.03 pp at S40 and +1.32 pp at S00
    (`audit/explore_per_density_out.json`).
- So a 1,000-epoch continuation alone moved the crash rate by about 0.5–0.8 pp pooled and 2 pp at one density. This
  drift also mixes in code drift since F's training (design review L5).
- The drift rule only compares 0.54 with |P1| and passes. The margin of P1 beyond the threshold is 0.71 pp, which is
  the same order as the observed drift.
- Breakdown of P1:
  - rand40 − F at S40 is −1.67 pp [−2.10, −1.29], 12/12 blocks;
  - ctrl's degradation contributes +0.54 pp.
- At S00, rand40 − F is −0.40 [−0.74, −0.05]. Most of the S00 half of P1 is therefore ctrl getting worse, not rand40
  getting better.
- Per density, P1 is −1.48 / −1.56 / −2.52 / −3.27 pp at 70 / 115 / 160 / 205 bars. rand40 − F at S40 is −1.51 /
  −1.42 / −2.52 / −1.24. The steep density gradient of P1 at 205 bars is mostly ctrl drift, not a robustness gradient.
- Consequence: SUPPORTED is evidence about this pair of runs only, as the amendment itself states. The amendment's
  confirmation-seed proposal (`:85-87`) is the right next step, and it matters here.

### Low

- **L1. The mandatory label is missing from the analysis output.** The amendment says a SUPPORTED reading "is labelled
  'single training seed per arm'" (`amendment:85-87`). `gr1_analyse.py:139-145` emits no such field. Downstream
  documents must add it.
- **L2. The known-bad smoke record misattributes its source.**
  - `smoke_gr_run_main.py:142` always records `G.sha(TREE/tools/gr_run.py)`, even when `GR_RUN_SOURCE`
    (`:18-23`) runs another file.
  - So `smoke_result_known_bad_6d8ee65.json` records `397e5512…`, the fixed `ab6531b` file. The `6d8ee65` source it
    actually ran is `de364b26…`.
  - My re-run reproduces the same checks; the provenance field is wrong, not the result.
- **L3. Scope of the smoke test.**
  - It stubs `preflight`, `run_cell` and `host` (`smoke_gr_run_main.py:59-61`).
  - It therefore validates `main()`'s control flow and wiring, and the discrimination of the repro comparison
    (perturbed → FAIL).
  - It does not exercise `check_cell`, `cell_env`, the launcher or the receipt-SHA check. For the real runs, I checked
    those from the GPU records instead (R2).
- **L4. The ctrl environment check covers NAVRL_* and run keys only.**
  - `ctrl_env_check.json:3-4` compares against a rand40 reference of 128 keys: 118 `NAVRL_*` keys and 10 run keys.
  - ctrl's other 96 inherited variables (desktop, IDE and agent-session variables, PATH, HOME) were not compared,
    because the rand40 reference does not hold them.
  - I confirmed that ctrl has no `NAVRL_*` key beyond the reference. None of the 96 is a CUDA, TORCH or OMP knob
    (`audit/probe_env_check.txt`).
- **L5. The trainer resumes from a derived file whose SHA is not recorded.**
  - The trainer actually resumes from `…_ep_25000_rew_39.742134_rlnorm.pth` (`runner.py:700-721, :962-966`), written
    into Policy F's run folder.
  - The ctrl start overwrote it at 14:55:05, so the bytes rand40 resumed from are gone.
  - `train_report.json` records only F's SHA (`gr1_train.py:90-91`).
  - I checked that the current sibling equals F tensor for tensor, apart from the stripped `_orig_mod.` prefix of the
    critic. The step is deterministic, and F's `.pth` is unchanged (`f7022139…`).
- **L6. The train report is bound by path only.** `gr_run.py:295` records the train-report path but not its SHA. The
  design review's H1 asked for both. The review manifest now binds `train_report.json` (`144b3df6…`, `7f5d6ae2…`).
- **L7. The t-interval agreement omits the wording.** `t_interval_agrees` (`gr1_analyse.py:144`) checks P1, P2 and P3
  but not the DiD wording. The DiD t-interval upper bound, −0.63, agrees.

### Info

- **The −2 pp cap did not hold.** Amendment `:76-78` expected P1 to be "capped near −2 pp" by floor strikes. Two
  things broke this: rand40 also crashes less at S00, and it reduced S40 floor strikes. In the aggregate window, per
  100 episodes:

  | Level | Cause | F | rand40 | ctrl |
  | --- | --- | --- | --- | --- |
  | S40 | bar contact | 6.38 | 5.76 | 7.51 |
  | S40 | below | 1.82 | 0.84 | 1.28 |
  | S00 | bar contact | 4.38 | 4.07 | 5.18 |
  | S00 | below | 0.10 | 0.02 | 0.05 |

  - About 80 % of P1 comes from bar contacts and 20 % from floor strikes.
  - Floor strikes still rise about 40× from S00 to S40 for rand40.
- **The gains are still not recorded per episode** (GR-0 review M1, design review L6). Nothing in GR-1 can say which
  end of the ±40 % range rand40 helps. The checkpoints' `env_state` is identical between the arms (232 keys), so
  randomization during training is evidenced only by the launcher echo and by the training floor-strike share (below).
- **`identity_compare` docstring.** It says it drops "the absolute checkpoint path" (`gr_run.py:207-211`), but
  result.json's `checkpoint` field is the SHA, so the runner's repro comparison ignores it. My comparison included the
  field: equal (F's SHA) for both repro cells.
- **Untested branches of the synthetic controls.** They did not exercise the "lower crash under ±40 % gains" branch or
  the drift branch (`test_gr1_analyse_synthetic.py:98-105`). Both are one-line conditions, and I applied them
  independently.
- **Unreachable edge case.** `gr_run.py:291-294` would raise a TypeError for a TRAINED report without a checkpoint.
  This cannot happen with these reports.
- **Information available when amending.** The amendment was written with the training logs visible. Those logs come
  from a different environment (legacy target, randomized gains for rand40). The new gates come from the design
  review, which was written before ctrl trained, and no threshold changed.

## Answers

### R1. Bindings: correct

| Check | rand40 | ctrl |
| --- | --- | --- |
| `train_report.json`: verdict, commit, arm | TRAINED, `8f64631`, rand40 | TRAINED, `8f64631`, ctrl |
| checkpoint sha256 = copy_sha256 = file now (copy and run dir) | `3f122b47…a6c5` | `0f6ca010…d137` |
| `gr1eval_run_report.json`: policy, sha, train_report path | rand40, `3f122b47…`, train_rand40 | ctrl, `0f6ca010…`, train_ctrl |
| receipt `checkpoint_sha256`, 36/36 | `3f122b47…` | `0f6ca010…` |
| receipt `NAVRL_EVAL_CHECKPOINT`, result.json `checkpoint`, recorder `context.checkpoint_sha256`, 36/36 each | `3f122b47…` | `0f6ca010…` |
| path the player loaded (`cell.log` line 1, `ckpt=`), 36/36 | `…/train_rand40/checkpoint/…rew_40.864136.pth` | `…/train_ctrl/checkpoint/…rew_55.227463.pth` |
| console log `[arms] ckpt=` (the launcher's own hash of the file), 36/36 | `3f122b47f08c638b` | `0f6ca010c2900bb0` |

- The launcher passes the path and SHA as `GR1_CKPT` / `GR1_CKPT_SHA` (`gr_run.py:358`). The copy hashes the file and
  refuses on a mismatch (`eval_navrl_gr1_arms.sh:48-53`).
- The launcher copy differs from the pinned launcher only in the header and in the CKPT / EXPECT_SHA lines (`:2-3`,
  `:29-32`). Its SHA is `418433eb…`, as pinned.
- **Distinct cells:**
  - rand40 = F: 0/36 identical ledgers; ctrl = F: 0/36; rand40 = ctrl: 0/36.
  - No two of the 108 ledgers are identical.
- **repro_02:** both cells (pinned launcher, and GR-1 copy with `GR1_CKPT` = F) equal GR-0's S40 / 205 bars / 4501
  cell.
  - Every episode row matches.
  - result.json matches with only the nonce and GR-0's three `cfg_gain_rand*` fields dropped. I kept the
    `checkpoint` field (F's SHA in all three).
  - Each cell's own checks pass, and the GR-0 cell was run on `c56be07`.

### R2. Cell validity: 72/72 valid; none should be excluded

My probe (`audit/probe_cells.py`) ran these checks on every cell (72 GR-1 cells plus the 2 repro cells):
- receipt: status OK, commit `ab6531be…`, 0 dirty files, `result_sha256` = the file;
- the recorded `NAVRL_GAIN_RAND*` equals the level: absent at S00; 1 / 0.2 or 0.4 / the cell seed at S20 and S40;
- no `cfg_gain_rand*` in the condition;
- backend `direct_l2_v1`; A3 placement with the visible initial view;
- ledger:
  - status COMPLETE, 0 partition violations;
  - 2,048 quota rows, 16 for each of 128 environments;
  - unique ids, `rows_sha256` and `rows_written` match, quota counts = the summary;
  - outcome flags consistent;
- recorder:
  - import origins inside the worktree, reason complete, motion gate PASS;
  - `episodes_sha256` matches, no violations;
  - it covers every ledger episode, and every outcome agrees with the ledger. The runner itself does not check
    coverage and agreement (GR-0 L1).

The host was stable for every cell: fault count 0, the same boot id, online CPUs `0-3,6-23` (eval reports
`host_start`, `host_end` and every `host_after`). Each run took under its 3-hour cap: rand40 16:12–17:42, ctrl
17:42–19:12.

**What differed between the two arms' runs:**
- The `NAVRL_*` snapshot in each receipt differs only in `NAVRL_EVAL_CHECKPOINT` and `NAVRL_KROC2027_RUN_ID`, apart
  from the per-cell nonce and output path. Against GR-0's F cells, the same two keys differ.
- The result.json `condition` blocks are identical across rand40, ctrl and F for 36/36 cells (`probe_condition.py`).
- The `cell.log` startup lines are identical between the arms once numbers are masked.
- Same commit, same launcher copy, same player.
- The arms ran one after the other on the same boot. Determinism is shown by repro_02.

### R3. Analysis: computed as amendment 1 fixes; the reading is right

- Numbers: see the table above.
- Bindings (`gr1_analyse.py:96-115`): they implement H1 and H2a, and all are true. I confirmed each one
  independently (R1).
- The reading order and conditions (`:129-138`) and the qualifiers (`:140-144`) match amendment `:55-69`.
- The bootstrap is the GR-0 estimator (`gr_analyse.py:31-33`, indices 250 and 9,749 of 10,000), with seed 20,261,111.
  It runs as one stream, but the conclusions do not depend on the stream.

### R4. Training: same contract, genuine continuations, no failure

- **Same contract.**
  - `train_navrl_gr1.sh` at `8f64631` sets the arm only in `:92-96`: the gain triple and the run tag.
  - It matches F's contract launcher (`train_navrl_v2_ep24000_speed_governor.sh:100-117`): 205 bars, curriculum off,
    seed 1, LR 5e-6, main profile, riskcap parameters, `MAX_EPOCHS` = previous + 1,000.
  - It differs from F's launcher in two ways (design review L4): it adds a worktree `PYTHONPATH` guard, and it does
    not rerun the summary validator.
  - Both arms trained on `8f64631`. Nothing under `aerial_gym/` or `motar_eval/` changes between `8f64631` and
    `ab6531b`.
  - ENV_MATCH holds for the `NAVRL_*` keys (L4).
  - The two training logs are identical outside the per-epoch metrics, apart from the gain-echo line and the log file
    name. The checkpoints' `env_state` is identical between the arms (`cfg_training_seed` 1, 205 bars, LR 5e-6, riskcap
    values).
- **Genuine continuations of F.**
  - Launcher log: "source=Policy F (f70221393660)"; SHA checked at `train_navrl_gr1.sh:39`.
  - Runner: resume from F's `_rlnorm` sibling, which is tensor-equal to F (L5), and "checkpoint state restored
    task_steps=800000", which is F's value.
  - Logged epochs: 25001–26000, contiguous, 1,000 per arm.
  - Checkpoints: `epoch` 26000; `frame` 106,496,000, which is F's 102,400,000 + 4,096,000.
  - Observation-normalizer count: +16,384,000, F's own rate per epoch × 1,000.
  - Actor weights moved 0.89 % (rand40) and 0.87 % (ctrl) from F (relative L2, counters excluded); the two arms are
    1.14 % apart.
  - The final `last_gen_ppo_ep_26000` file was used, as preregistered.
- **No failure or divergence.**
  - Exit reason max_epochs; wrapper rc 0; TRAINED; no host change.
  - Non-finite logged metrics: 0. Rollback totals in the checkpoints: 0.
  - Training crash rate (100-epoch windows): rand40 29.4 → 26.3 % under ±40 % gains; ctrl 16.6 → 17.4 %.
  - The rand40 training floor-strike share is 0.24–0.32 of crashes, against 0.00–0.04 for ctrl. This independently
    shows that randomization was active in rand40 only.
  - Wall time 70 / 65 min, under the 3-hour cap.

### R5. Interpretation limits

**Supported:** on the GR-0 grid (E2, A3, 70–205 bars × seeds 4501–4503, quota 16, navrl_quad, Lee-controller gains
randomized ±40 % in distribution), this rand40 continuation of Policy F, compared with this ctrl continuation:
- crashes 2.2 pp less under ±40 % gains;
- turns those crashes into captures, not timeouts;
- costs no nominal capture (+1.2 pp);
- has a smaller absolute S40 − S00 crash excess (2.42 vs 3.44 pp).

It also crashes 1.7 pp less than F itself at S40.

**Not supported:**
- a population claim about gain-randomized training: one seed per arm, and continuation drift of the same order as
  the margin (M2);
- reduced sensitivity on a relative scale (M1);
- any statement about gain ranges beyond ±40 %, which were not tested;
- which gain element or which end of the range is helped (no per-episode gains);
- a remedy for floor strikes, which persist at about 40× nominal.

**DiD rule:** the wording "reduced gain sensitivity" follows amendment `:66-67` exactly (CI upper −0.71 < 0). It
should be stated with its magnitude, −1.0 pp, and with the M1 caveat.

**Other airframes, PX4, real flight:** nothing may be claimed.
- Only the three Lee-controller gain sets of the simulated 0.25 kg `navrl_quad` vary. Mass, inertia, motor constants,
  latency and drag stay nominal.
- The altitude PI is the task's own.
- The 1.2 kg ref5in airframe, PX4's control architecture and tuning, and real flight are not tested (preregistration
  `:93-94`).
- The checkpoint replaces nothing (D-2026-11-04-1).

### R6. Process

- **The fix is correct and complete.**
  - Attempt 1 failed at `gr_run.py:312` of `6d8ee65`, because `args.policy` no longer existed. It failed after
    `out_root.mkdir`, which is why `repro/` exists and is empty: no cell ran and no report was written.
  - The fix, `ab6531b` `gr_run.py:257`, restores `policy`, `ckpt` and `ckpt_sha` as None. They are filled from the
    training report only in gr1eval (`:290`).
  - Every `args.*` attribute used later is defined. The run ids are unchanged: GR1-F for repro, GR1-RAND40 and
    GR1-CTRL in the receipts. The gr0 and identity stages are unaffected.
- **The smoke test is valid for its purpose, with a real known-bad control.**
  - I re-ran it: the fixed runner passes 7/7. The `6d8ee65` source reproduces the AttributeError. The checks are
    identical to the committed result files.
  - Its scope (L3) and provenance field (L2) are limited.
- **Changes between the amendment and the evaluation.** `6d8ee65` → `ab6531b` changes only:
  - one line of `gr_run.py`;
  - the smoke files;
  - the WORKLOG.

  Unchanged:
  - `gr1_analyse.py` keeps the bytes named in the amendment, `b230fb72…`, also used by the synthetic controls (staging
    copies verified);
  - the launcher copy (`418433eb…`), the protocol, and the pinned launcher, player and F.

  The amendment does not mention the fix, but the fix alters no behaviour the amendment fixes.
- **Timing.**
  - ctrl training ended 15:59:37; amendment `6d8ee65` at 16:03:34; fix `ab6531b` at 16:07:10; first evaluation cell
    16:07:16.
  - No JSON record elsewhere on the disk carries either GR-1 checkpoint SHA, apart from the training and evaluation
    folders, the smoke results and this review's own files. So no evaluation of a GR-1 checkpoint preceded the
    amendment.

## What I checked and how (files in `audit/`)

| Check | Probe | Result |
| --- | --- | --- |
| Input binding | `probe_manifest_verify.txt` (all 789 files of `SHA256_MANIFEST.json`) | 0 mismatches; `run_copy/gr1_analysis.json` = `analysis/gr1_analysis.json` |
| Cells, receipts, ledgers, recorder, env and identity | `probe_cells.py` → `probe_cells_out.json` | 0 problems in 72 + 36 F + 2 repro cells; R1 and R2 tables |
| Condition blocks | `probe_condition.py` | identical across arms and F |
| Analysis recomputation | `recompute_analysis.py` → `recompute_out.json` | exact match; reading and qualifiers equal |
| Committed analysis re-run | `tools_ab6531b/` + `rerun_gr1_analysis.json` | byte-identical to `run_copy` |
| Exploratory scale and density | `explore_relative_scale.py`, `explore_per_density.py` | M1, M2 |
| Checkpoints | `probe_checkpoints.py`, `probe_checkpoints_weights.py` | R4; rlnorm = F |
| Training logs | `probe_train_curves.py` | R4 |
| ctrl environment | `probe_env_check.txt` (key names only) | L4 |
| Runner smoke | `smoke_gr_run_main_ab6531b.py`, `gr_run_6d8ee65.py` → `smoke_result_*_rerun.json` | 7/7 fixed; known-bad fails as committed |
| Host | `fault_count_before.txt`, `fault_count_after.txt` | 0 → 0 |
