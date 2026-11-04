# Pre-measurement design review: GR-1 (Policy F continued with randomized controller gains)

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`). Fresh context, independent of the author. 2026-11-04, 14:35–14:55 KST.
- **Scope:** commit `8f64631` on `gainrand/dev`, read with `git show` (GR-1 files from `847e777`):
  - the GR-1 preregistration;
  - `train_navrl_gr1.sh` and `tools/gr1_train.py`;
  - `eval_navrl_gr1_arms.sh`, compared with the pinned `eval_navrl_target_behavior_arms.sh`;
  - the `gr1repro` and `gr1eval` stages of `tools/gr_run.py`, and `tools/gr1_analyse.py`;
  - the helpers these reuse: `kroc2027_eval.py`, `gr_analyse.py`, `train_navrl_v2_search.sh`, `train_navrl.sh`,
    `play_navrl.sh`, `runner.py`, `navrl_task.py`, `base_lee_controller.py` and the gain config;
  - the GR-0 preregistration, its amendment 1 and `REVIEW_r1.md`;
  - Policy F's training contract (`train_navrl_v2_ep24000_speed_governor.sh`) and its session log
    `v2_ep24000_speedgov_riskcap_260805_041322_665554.log`;
  - the 36 raw GR-0 cells.
  WORKLOG and the author's summaries were not used.
- **State when reviewed:**
  - The `rand40` arm was still training: `gr1_train.py rand40 --approved-commit 8f64631`, at epoch about 25,950 of
    26,000 at 14:50.
  - `ctrl` had not started. No GR-1 evaluation cell exists.
- **Host:**
  - Kernel fault count (`journalctl -k -b 0`, Machine check / segfault / general protection / mce) was 0 at 14:44
    and 0 at 14:51.
  - Online CPUs stayed `0-3,6-23`.
  - No GPU process was started. All probes ran on the CPU (`taskset -c 0-3,8-11,14-23`, `CUDA_VISIBLE_DEVICES=`,
    `PYTHONNOUSERSITE=1`, aerialgym python `-B -I`). At most one ran at a time, and none imported `aerial_gym`.
- **Writes:** only `review_gr1_design/audit/`. The running training was only observed:
  - its logs were read;
  - its environment was read from `/proc/3236004/environ`.
  It was not signalled or otherwise touched.

## Verdict: **AMEND_FIRST**

The design is sound in its core. Two arms differ only in `NAVRL_GAIN_RAND*` (I checked this for `rand40`'s live
environment). Both are evaluated through one launcher copy on the GR-0 grid. A reproduction check runs on the GR-1
commit, and the readings are fixed in advance. Under evaluation noise alone, the reading rule behaves as intended:
- NOT_SUPPORTED at a true P1 of 0, with probability about 1;
- SUPPORTED at −2 pp, with probability ≥ 0.89.

Four items must change, through a dated amendment, before any GR-1 cell is measured:
- three can silently produce a wrong reading (H1, H2, H3);
- one leaves the claim's own failure directions ungated (M1).

H3 is time-critical: it must be settled **before the `ctrl` arm trains**.

## Findings by severity

### High (must change before measurement)

**H1. No binding between arm label, trained checkpoint and analysis input; a mix-up flips the sign of P1 and P2
without an error.**
- *Prereg vs code:* the preregistration requires that each receipt's `checkpoint_sha256` equal "the trained
  checkpoint's" (`prereg_gain_randomization_gr1_v1.md:60`). The code has no such link:
  - `gr1eval` compares the receipt against `--ckpt-sha`, which the operator types (`gr_run.py:125-126, 276-280,
    335-338`);
  - nothing ties `--policy rand40` to `train_rand40/train_report.json`.
- *Analysis:* `gr1_analyse.py` takes the GR-0, rand40 and ctrl folders by position (`:36-40`). It checks only
  `verdict == COMPLETE` (`:28-32`). It never reads the report's `policy` or `checkpoint_sha256`, and never checks that
  the two folders differ.
- *Evidence:* synthetic run folders, symlinked from GR-0 cells (`audit/make_synth.py`, outputs in `audit/synth_out/`),
  run through the exported `8f64631` `gr1_analyse.py`:
  - `shift` (rand40 S40 := F's S20 cells): P1 −3.08 pp [−3.61, −2.52], **GR1_SUPPORTED** (the code works);
  - the same two folders with their labels exchanged: P1 +3.08 pp, **GR1_NOT_SUPPORTED**, with no warning;
  - the same folder given twice: P1 = P2 = 0, **GR1_NOT_SUPPORTED**, with no warning.
- *Amend:*
  - `gr1eval` reads `train_<policy>/train_report.json`. It requires verdict TRAINED and
    `--ckpt-sha == checkpoint.sha256 == checkpoint.copy_sha256`, and records that report's path and SHA.
  - `gr1_analyse` requires each run report's `policy` to match its position, the rand40, ctrl and F SHAs to be
    pairwise different, and both gr1eval reports to share one commit. It also checks the GR-0 inputs against
    `SHA256_MANIFEST.json`.

**H2. No control shows that the pipeline evaluated the checkpoint it was given. The failure it would miss reads
GR1_NOT_SUPPORTED.**
- *What is measured:*
  - `eval_navrl_gr1_arms.sh:48-53` hashes the file at `GR1_CKPT` and passes that path to `play_navrl.sh` (`:179`).
  - Every later record of the checkpoint is the caller's SHA string, not a measurement of the loaded weights: the
    receipt (`:195, :218`), `NAVRL_EVAL_CHECKPOINT` (`:110`), result.json's `checkpoint`
    (`navrl_task.py:9421`) and the recorder (`kroc_eval_adapter.py:793`).
- *Reproduction check:* `gr1repro` is a same-checkpoint equality test (`gr_run.py:321-333`). It shows reproducibility,
  not discrimination. It also runs the pinned launcher, not the GR-1 copy.
- *Consequence:* if both arms silently evaluated F, or the same weights, P1 = P2 = 0 exactly, and the reading is
  NOT_SUPPORTED (synthetic `null` test above).
- *Amend:* two cheap controls.
  - (a) No GR-1 cell's `episodes.jsonl` may equal the GR-0 F cell with the same (level, density, seed). No rand40
    cell may equal the matching ctrl cell. Any identical pair voids the reading: EVAL_BINDING_FAIL.
  - (b) Add one `gr1repro` cell through `eval_navrl_gr1_arms.sh` with `GR1_CKPT` = Policy F, required equal to the
    GR-0 cell. This validates the launcher copy end to end, for about 3 GPU-minutes (a 205-bar cell took 157 s).

**H3. Nothing requires the `ctrl` arm to train on the same runtime code or the same environment as `rand40`.**
- *Commit:* `gr1_train.py` checks only `approved commit = HEAD` and a clean tree (`:41-45`). It records the commit
  (`:57`) but nothing compares it across arms. Any amendment commit moves HEAD before `ctrl` trains.
- *Environment:*
  - The wrapper starts the launcher with the caller's whole environment (`:68-69`; no scrub, unlike `gr_run.py:97`).
  - `train_report.json` records no environment.
  - `train_navrl_v2_search.sh` inherits several knobs through `${VAR:-default}`: goal distances `:290-291`,
    density thresholds `:321-333`, geofence `:372-374`, detector pixels `:389`, target speed `:401-402`.
  - Many task variables are read with defaults inside `navrl_task.py`.
- *Evidence that `rand40` is clean:* its runner process environment (`audit/rand40_runner_env.txt`, 128
  `NAVRL_*` and run variables) contains only values set by the launcher chain:
  - `NAVRL_GAIN_RAND=1`, `_SCALE=0.4`, `_SEED=1`;
  - `SEED=1`, `MAX_EPOCHS=26000`, `NUM_ENVS=128`;
  - `PYTHONPATH` = the worktree.
- *Amend (before `ctrl` starts):*
  - `ctrl` trains on `8f64631`, or on a commit whose diff from it touches nothing under `aerial_gym/` or
    `motar_eval/` (training launchers included).
  - Its runner environment must equal `audit/rand40_runner_env.txt` apart from the three `NAVRL_GAIN_RAND*`
    variables and the run tag. Either `gr1_train.py` scrubs inherited `NAVRL_*` and records the environment, or the
    equality is checked from `/proc` during the run.
  - `gr1_analyse` verifies the commit rule from the two train reports.

### Medium

**M1 (must change). The claim is never stated, and two directions in which it can be false are not gated.**
- *Missing claim:* the preregistration names P1 "robustness" and P2 "nominal cost" (`:68-69`) but writes no
  hypothesis. In practice it says that this rand40 continuation crashes ≥ 1.5 pp less than this ctrl continuation at
  S40, at ≤ 2 pp nominal-capture cost.
- *Gated directions:* P1 (point ≤ −1.5 and CI upper < 0) and P2 (CI lower > −2), as written (`:73-78`). This part is
  correct.
- *Ungated (a), a uniform shift is not robustness:*
  - P1 is an absolute S40 difference. A rand40 policy that is better by the same amount at every level passes
    SUPPORTED with zero change in gain sensitivity.
  - With timeouts at about 0, DiD ≈ P1 + P2, where DiD = (rand40 S40 − S00 crash gap) − (ctrl gap).
  - The per-policy gaps are reported (`gr1_analyse.py:73`), but their difference has no CI and no interpretation rule.
- *Ungated (b), crashes turned into timeouts:* P1 counts crashes only.
  - A more cautious policy can lower S40 crashes by turning them into timeouts, without more captures. GR-0's F
    timeouts are 0.004 % at S00 and S40 (`audit/gr0_variability_power.json`).
  - P2 checks capture at S00 only.
- *Amend:*
  - write the claim, including the unit of inference (this pair of runs, E2/A3, the GR-0 grid);
  - report the DiD with its bootstrap CI. "Reduced gain sensitivity" may be written only if its upper bound < 0;
    otherwise SUPPORTED is described as "lower crash under ±40 % gains";
  - SUPPORTED additionally requires CI lower > 0 for capture(rand40, S40) − capture(ctrl, S40). If timeouts stay at
    about 0, this holds automatically, so the power figures below stand.

**M2 (should change). GR-0 review M2 is not accounted for.**
- *Timing:* the GR-1 preregistration (`847e777`, 13:42:26) predates the review (`8f64631`, 13:43:32), and nothing in
  it addresses floor strikes.
- *Independent recount* (`audit/gr0_crash_causes_check.json`, result.json `crash_causes` summed over cells; aggregate
  window, exploratory): the S40 − S00 crash excess is 3.71 per 100 episodes. It splits into:
  - 2.00 from bar contacts, which grow with density;
  - 1.72 from floor strikes ("below"), 1.63–2.15 per 100 at every density.
- *Policy authority:* the policy has no vertical action. `command[:, 2]` is the task's altitude PI
  (`navrl_task.py:4997-5002`), whose output then passes through the randomized K_vel. Its authority over floor strikes
  is only indirect, through its lateral and yaw commands.
- *Implication:* if floor strikes stay as they are, P1 is limited to about −2.0 pp, and −1.5 needs about 75 % of the
  bar-contact excess removed.
- *Exploratory context (must not be used to retune any threshold):* in the running `rand40` log, "below" is
  0.24–0.28 of training crashes throughout. In F's own continuation it was 0.03–0.06.
- *Amend:*
  - pre-declare a reported, not gated, crash-cause table per policy and level (bar_contact and below per 100 episodes,
    counts summed over cells, as corrected in GR-0 L5);
  - add a next-step rule per reading. GR-0 had one; GR-1 has none. In particular, NOT_SUPPORTED or INCONCLUSIVE with
    persistent floor strikes points to a controller-side remedy, not to more PPO variants (discipline review
    2026-08-22).

**M3 (should change). With one run per arm, "randomization did nothing" does not mean P1 = 0.**
- The block bootstrap covers evaluation noise only. Under a true null, P1 equals the run-to-run difference between
  two 1,000-epoch continuations, which has not been measured. The preregistration discloses one seed per arm
  (`:89-90`), but no reading accounts for it.
- *How large this could be:* if a run's S40 crash rate varies with SD σ, a spurious SUPPORTED needs a pair difference
  ≤ −1.5 pp. That has probability 2 % at σ = 0.5 pp and 14 % at σ = 1 pp.
- *Minimum amendment:* pre-declare ctrl − F at S40 and S00 (already computed, `gr1_analyse.py:68-72`) as a drift
  calibration. If |ctrl − F at S40| ≥ |P1|, SUPPORTED is reported as "not separable from continuation drift".
- *Better:* a negative control.
  - A second ctrl continuation with `SEED=2`, evaluated at S00 and S40 (24 cells, about 1 GPU-h of training and
    1 GPU-h of evaluation).
  - Cheaper and weaker: ctrl's `ep_25950` snapshot at S40 (12 cells).

### Low

- **L1. Power is not stated.** I used GR-0's 12 blocks (`audit/gr0_variability_power.json`).
  - *Block-level noise:*
    - The SD of the S20 − S00 crash difference is 0.68 pp (0.75 within density). That matches the binomial SD of a
      difference between two independent cells (0.64–0.68 pp), so same-seed blocking removes almost no variance.
    - At S40 rates the binomial SD is 0.85 pp, so the SE of P1 is about 0.25 pp and the CI half-width about 0.45 pp.
  - *Simulated rule* (4,000 simulations, 4,000 bootstrap resamples, block SD 0.68 or 1.02, uniform or
    density-shaped effects):

    | True P1 | SUPPORTED | Other readings |
    | --- | --- | --- |
    | 0 | 0.00 | NOT_SUPPORTED 1.00 |
    | −1.0 | 0.003–0.05 | NOT_SUPPORTED 0.42–0.76 |
    | −1.5 | 0.46–0.50 (by construction) | INCONCLUSIVE 0.45–0.50 |
    | −2.0 | 0.88–1.00 | – |

  - 80 % power is reached at about −1.7 pp.
  - Recommendation: state this in the amendment.
- **L2. Few blocks for the bootstrap.** A percentile bootstrap over 12 blocks is about 15 % narrower than a
  t-interval (coverage about 91 %). It is preregistered; report t-intervals as a sensitivity check.
- **L3. Launcher copy.**
  - Its diff from the pinned launcher is only the header and the `CKPT` / `EXPECT_SHA` lines
    (`eval_navrl_gr1_arms.sh:2-3, 29-32`; `audit/pinned_launcher.sh`, `audit/gr1_launcher.sh`).
  - `gr_run.py` checks only that the file exists (`:281-282`). It does not pin the copy's SHA (`418433eb…`) or its
    diff, so the commit is the only binding.
  - `report["prereg"]` names the GR-0 preregistration for GR-1 stages (`:257`).
  - `gr1eval` has no default `--max-hours`.
- **L4. Training launcher vs F's.**
  - `train_navrl_gr1.sh:42-63` reads the screen summary directly. It does not rerun the
    `summarize_navrl_v2_riskcap.py` revalidation, as F's launcher did (`train_navrl_v2_ep24000_speed_governor.sh:35`).
  - It prints a literal `riskcap` instead of checking the row's `mode` (F's `:62-64`).
  - No effect: the summary is tracked, with SHA `540a0f72…` identical to the main checkout's. `rand40`'s log shows
    the same governor values as F's log (fixed 2.00, free 3.54, margin 0.45, ttc 1.20, brake 2.96, reaction 0.10).
- **L5. "The contract that produced F" (`:21`) holds for the environment, not the code.**
  - 91 commits since 2026-08-05 touch `navrl_task.py`, the search launcher, `runner.py` or the agent.
  - The diff of startup lines (`audit/F_train_startup.txt` vs `audit/rand40_train_startup.txt`) shows, for example,
    a "TARGET HEADING CONTRACT … 0.1 ASSUMED" warning that is new in GR-1.
  - P1 and P2 are unaffected, because both arms share the code. The rand40 − F and ctrl − F differences confound the
    extra epochs with code drift; disclose this.
- **L6. Provenance of the randomization.**
  - With `navrl_task.py` at base bytes, GR-1 checkpoints' `env_state` cannot show that gains were randomized.
  - The only record is `launcher.log`'s echo of the variable.
  - Indirect positive evidence: `rand40` training shows the floor-strike pattern of S40 (the M2 numbers above).
  - Per-episode gains are still not recorded (GR-0 M1), so GR-1 cannot tell which gain end is helped.

### Info

- `_normalize_vf_checkpoint` writes `…_rlnorm.pth` into F's run folder (`runner.py:700-721`). F's `.pth` is
  untouched: its SHA is checked by `train_navrl_gr1.sh:39`.
- The S40 test range equals the training range, so the test is in distribution. S20 lies inside it. No out-of-range
  level is tested; the claim scope should say so.
- The training seeds (sim 1, gain 1) are disjoint from the evaluation seeds (4501–4503). Gain seed = cell seed gives
  both arms common gain draws for every first episode, which is good for blocking.

## Answers

**D1 — Claim and criteria.**
- The implied claim: this rand40 continuation crashes less than its matched ctrl at ±40 % gains, by ≥ 1.5 pp, with
  nominal capture within 2 pp.
- P1 and P2 test exactly that absolute claim, and the decisive failure direction is gated. That direction is no
  material S40 crash reduction (P1 > −1.5 pp, or ≥ 0), together with a nominal cost (P2 ≤ −2).
- The label "robustness" also suggests reduced sensitivity. That is not gated: a uniform improvement passes, and so
  does crash → timeout conversion (M1). Training-run variance is not gated either (M3).

**D2 — Power and known-bad control.**
- *Evaluation-noise power:* see L1. About 0.5 at exactly −1.5 pp (inevitable with a point-estimate threshold at the
  effect size), 0.8 at about −1.7, ≥ 0.88 at −2.0. At a true P1 of 0 the reading is NOT_SUPPORTED with probability
  about 1.
- *But:* a true null for randomization means P1 = the run-to-run difference, which is unmeasured (M3).
- *Discrimination control:* none. `gr1repro` tests equality only, and a pipeline that evaluated the wrong weights
  would read NOT_SUPPORTED (H2).

**D3 — Coverage and confounds.**
- *Matched control:* sound. The launcher changes only the three `NAVRL_GAIN_RAND*` variables (`:92-95`), and
  `rand40`'s live environment confirms this. The same rule is not enforced for `ctrl` (H3).
- *Training contract vs F's continuation:* equal at the environment level (L4). The code differs (L5).
- *Launcher copy:* minimal diff (L3), but never run against a known output (H2b).
- *Reproduction check:* sound for its purpose. `run_id` appears only in the receipt and the recorder summary, not in
  the compared files, so the different run id is harmless.
- *Checkpoint binding:* the file is bound to its SHA at launch. The SHA is not bound to the arm (H1).
- *GR-0 M2:* not accounted for (M2).

**D4 — Amend before evaluation** (one dated amendment; the preregistration file unchanged):
1. H3, before `ctrl` trains: ctrl's commit and environment equality with rand40's.
2. H1: bind arm ↔ train_report ↔ SHA in `gr1eval` and `gr1_analyse`.
3. H2: checkpoint-discrimination check, plus one `gr1repro` cell through the GR-1 launcher copy with F.
4. M1: an explicit claim, the DiD with its CI and wording rule, and the S40 capture guard on SUPPORTED.
5. Recommended:
   - M2: crash-cause table and a next-step rule;
   - M3: the drift qualifier, or a ctrl seed-2 negative control;
   - L1: a power statement.

Code changes for items 2–4 touch only `tools/`, so they leave `rand40`'s runtime unchanged. Under item 1, `ctrl` must
then train on a commit that differs from `8f64631` only outside `aerial_gym/` and `motar_eval/`. `gr1repro` must
rerun on the commit that evaluates.

## What I checked and how (files in `audit/`)

| Check | Method | Result |
| --- | --- | --- |
| Launcher copy vs pinned | `git show` both at `8f64631`, `diff -u`; SHA of the worktree file | only the header and CKPT lines differ; worktree = committed (`418433eb…`) |
| Training contract | `train_navrl_gr1.sh` vs `train_navrl_v2_ep24000_speed_governor.sh`, the search chain, `runner.py` resume path | L4, H3 |
| rand40 live environment | `/proc/3236004/environ` (read only) → `rand40_runner_env.txt` | no stray variables; gain = 1 / 0.4 / 1 |
| F's vs rand40's startup | the two session logs → `F_train_startup.txt`, `rand40_train_startup.txt`, diff | L5; identical governor and density settings |
| Randomization active in training | `crashdiag` / `NavRL progress` lines, rand40 vs F's session log | below share 0.24–0.28 vs 0.03–0.06 (exploratory) |
| GR-0 inputs bound | `verify_inputs.py` against the committed `SHA256_MANIFEST.json` | 72/72 files (episodes.jsonl, result.json) match |
| GR-0 rates, block variability, power | `gr0_variability_power.py` → `.json` | pooled S00 / S20 / S40 crash 4.46 / 5.07 / 8.15 % (matches); L1 table |
| Crash causes | `gr0_crash_causes_check.py` → `.json` | S40 excess = 2.00 bar + 1.72 floor per 100 |
| Analysis code behaviour | `git archive` of `tools/` and `motar_eval/` at `8f64631` → `trees/`; `make_synth.py` → `synth/`; `gr1_analyse.py` → `synth_out/` | shift → SUPPORTED; null, swapped and same-dir → NOT_SUPPORTED, no error (H1, H2) |
| Code path of the gain switch, altitude PI, env_state | `git show` / `git diff 62e59c0 8f64631`; `navrl_task_8f64631.py` | only the controller, config and two launchers differ from the base; `navrl_task.py` = base |
| Host | `journalctl -k -b 0` count, `/sys/devices/system/cpu/online` | 0 → 0; `0-3,6-23` |
