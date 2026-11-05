# GR-1b independent results review (Opus 5.5, read-only)

**Verdict: PASS.** I found no consequential errors. From the raw ledgers I get the same pair reading as the author, **GR1_NOT_SUPPORTED**, and the same confirmation reading, **NOT_CONFIRMED**. The labels stay "computed" until this review is recorded.

## Model identity

- **Model requested:** `claude-opus-5-5`. The launch command in `opus_review/review_process.json` passes `--model claude-opus-5-5`.
- **Model running:** my system context says Opus 5.5 (`claude-opus-5-5`).
- **Limitation:** I can't verify the serving model from inside the session. The identity rests on the launch command plus my own report of it.

## Scope and how the checks ran

- **Source:** the repository is at `7498c814…` and `git status --porcelain --untracked-files=all` is empty, so the tree is clean.
- **Training vs evaluation commit:** training ran on `31d5761`. Its only child commit is `7498c81`, which changes `AGENTS.md`, `WORKLOG.md`, `docs/DECISIONS.md` and the evidence README. Nothing under `aerial_gym/` or `tools/` changed.
- **How I computed:** my own script, `opus_review/audit_gr1b.py`, writes `audit_results.json`. It ran with CUDA hidden, a 4 GB memory cap, `timeout`, cores 6,7,12,13 and `PYTHONDONTWRITEBYTECODE=1`. It does not import project code, Isaac Gym or aerial_gym.
- **Analyser rerun:** I ran the unchanged `tools/gr1_analyse.py` (sha `b230fb72…`, the bytes pinned in amendment 1) into `opus_review/rerun_pair_analysis_s2.json`. The output is **byte-identical** to the author's `pair_analysis.json`: both have sha256 `423e69e221efb5cf…`.
- **First pair:** I recomputed it from the raw inputs named in `gr1_analysis.json` (GR-0 `gr0_main`, `gr1_v1/eval_rand40`, `gr1_v1/eval_ctrl`).

## Gate and binding checks (all from raw files)

| Check | rand40_s2 | ctrl_s2 |
|---|---|---|
| Training report | TRAINED, commit `31d5761`, GR-1b prereg, exit 0, no problems, faults 0→0, same boot | same |
| Checkpoint copy SHA (recomputed) | `2324419f…3051` = reported | `93bb9ef4…ce3c` = reported |
| Original checkpoint in `runs/…-s2/nn/` | same SHA | same SHA |
| Session log SHA | matches | matches |
| Evaluation report | COMPLETE, commit `7498c81`, no preflight problems, faults 0→0, same boot and CPU set | same |
| Grid | exactly 36 cells, no duplicates, matches S00/S20/S40 × 70/115/160/205 × 4501–4503 | same |
| In-quota episodes | 2048 per cell (128 envs × 16), unique IDs, outcome flags consistent, counts = receipt stats | same |
| Receipts | status OK, commit `7498c81`, 0 dirty files, `checkpoint_sha256` and `NAVRL_EVAL_CHECKPOINT` = arm SHA, `result_sha256` = recomputed result.json hash, gain variables match the level | same |
| Import origins (report and every cell) | inside `MOTAR-gainrand` | same |
| Recorder violations | none | none |

Further checks, all passing:
- **Ledgers differ where they must:** 0 identical ledgers rand40 vs F, ctrl vs F, or rand40 vs ctrl.
- **Same contract:** the receipt environments, minus per-cell keys, are identical across rand40, ctrl and F.
- **Training launcher and wrapper diffs** (`train_navrl_gr1b.sh` vs `train_navrl_gr1.sh`, `gr1b_train.py` vs `gr1_train.py`) change only these:
  - `SEED=2` and `NAVRL_GAIN_RAND_SEED=2`;
  - the `-s2` run tag;
  - the `gr1b_` session-log prefix and log glob;
  - the prereg path and launcher name;
  - message text.
- **Reproduction:** GR1_REPRO_PASS on `7498c81`. Both relaunches' `episodes.jsonl` (pinned launcher and GR-1 launcher copy) have the same sha256 as GR-0's cell (`f97cffc6…`).
- **Baseline:** GR-0 `gr0_main` (commit `c56be07`) has 36 cells × 2048, faults 0→0 and no problems.

## Fixed gates (preregistered)

| Pair | P1 crash S40, rand40−ctrl | P2 capture S00 | P3 capture S40 | DiD crash gap | Reading |
|---|---|---|---|---|---|
| s1 (GR-1) | −2.209 [−2.677, −1.762] | +1.196 [+0.753, +1.701] | +2.209 [+1.762, +2.686] | −1.017 [−1.371, −0.712] | GR1_SUPPORTED (reproduced) |
| **s2 (GR-1b)** | **+0.444 [0.000, +0.875]** | **−0.448 [−0.850, −0.094]** | **−0.452 [−0.887, −0.008]** | **+0.004 [−0.623, +0.618]** | **GR1_NOT_SUPPORTED** |

Values are in pp, with 95 % block bootstrap intervals (seed 20261111, 10,000 resamples).

- **Means and per-block values:** my independent block values match the author's exactly, and my bootstrap intervals match to six decimals.
- **How the rules apply to s2:**
  - Not MATERIAL_LOSS, because P2 = −0.45 is above −2 pp.
  - Not SUPPORTED, because P1 > 0.
  - NOT_SUPPORTED, because the lower bound of CI(P1), 0.0, is above −1.5.
- **Robustness:** the bound of 0.0 is not knife-edge. Five other bootstrap seeds give −0.016 to 0.000, all far above −1.5.
- **Confirmation mapping:** GR-1b prereg lines 51–55 map NOT_SUPPORTED to **NOT_CONFIRMED**. The author applied this correctly in `completion.json`.
- **Qualifiers:** the wording and drift qualifiers are null, correctly, because they apply only to SUPPORTED.

## Reported, not gated (exploratory)

**Across pairs.** Two training runs per arm are a replication check, not a training-variance estimate, so there are no intervals here.

| | P1 | P2 | P3 | DiD |
|---|---|---|---|---|
| Mean of s1 and s2 | −0.883 | +0.374 | +0.879 | −0.507 |
| s2 − s1 | +2.653 | −1.644 | −2.661 | +1.021 |

**Log-ratio crash DiD** (the GR-1 review's relative scale; point estimates only, no intervals):
- Per-block form: (1/12) Σ_b [ln(c_r40,S40,b / c_r40,S00,b) − ln(c_ctrl,S40,b / c_ctrl,S00,b)].
  - s1: −0.0540, with 7 of 12 blocks negative.
  - s2: −0.0160, with 6 of 12 blocks negative.
- Pooled form: ln(C̄_r40,S40 / C̄_r40,S00) − ln(C̄_ctrl,S40 / C̄_ctrl,S00).
  - s1: −0.0358; s2: −0.0351.
- No arm has a zero crash rate in any S00 or S40 cell, so both forms are defined.
- Pooled S40/S00 crash ratios: F 1.829; s1 rand40 1.597, ctrl 1.655; s2 rand40 1.629, ctrl 1.687.

**Contrasts against F (s2, author's intervals reproduced).** F's cells come from GR-0, including the T-LOAD-qualified `load_overlap` cells.

| Level | rand40_s2 − F capture | rand40_s2 − F crash | ctrl_s2 − F capture | ctrl_s2 − F crash |
|---|---|---|---|---|
| S00 | −0.680 [−1.046, −0.285] | +0.663 [+0.273, +1.021] | −0.232 [−0.496, +0.053] | +0.224 [−0.061, +0.492] |
| S20 | −0.415 [−0.720, −0.134] | +0.423 [+0.142, +0.736] | −0.130 [−0.488, +0.224] | +0.138 [−0.220, +0.496] |
| S40 | −0.195 [−0.582, +0.212] | +0.187 [−0.228, +0.574] | +0.256 [−0.033, +0.521] | −0.256 [−0.517, +0.033] |

**Crash causes per 100 episodes** (`result.json`, aggregate window of about 24,590 episodes per level, not the in-quota ledger; given as bar contacts / floor strikes):

| Policy | S00 | S20 | S40 |
|---|---|---|---|
| F | 4.38 / 0.10 | 4.86 / 0.29 | 6.38 / 1.82 |
| rand40_s2 | 5.04 / 0.05 | 5.28 / 0.25 | 7.09 / 1.28 |
| ctrl_s2 | 4.51 / 0.09 | 4.89 / 0.33 | 6.37 / 1.66 |
| rand40_s1 / ctrl_s1 at S40 | | | 5.76 / 0.84 vs 7.51 / 1.28 |

My counts match the author's table exactly.

## Findings

**High: none. Medium: none.**

**Low:**
- **L1. Labels in the reports.**
  - The `gr1eval` and `gr1repro` reports set `prereg` to the GR-1 preregistration, not GR-1b. This follows from `gr_run.py` being unchanged, as preregistered.
  - The receipts' `NAVRL_KROC2027_RUN_ID` is `GR1-RAND40` / `GR1-CTRL`, the same as in GR-1. GR-1b is identified by the training reports and the folder names.
  - Neither affects any outcome.
- **L2. A disclosed freeze violation.** `AGENTS.md` was edited during `rand40_s2` training at about 03:12 and reverted within a minute (evidence README lines 187–198). The trainer does not read that file, and both training reports read TRAINED with no problems. It stays recorded as a deviation.
- **L3. A path in the training reports.** Their `run_dir` + `file` omits the `nn/` subfolder. I found the files under `nn/` and their SHAs match.
- **L4. An unmapped case.** The GR-1b confirmation table does not say how EVAL_BINDING_FAIL maps; read literally it falls under "otherwise", which is MIXED. It wasn't triggered: the author's script stops on a binding failure, and binding passed.
- **L5. Bootstrap edge.** The P1 lower bound of exactly 0.0 is a bootstrap discreteness artefact. The t-interval disagrees for P1 and P3. Neither changes the reading.

**Interpretation limits** (no error; they constrain any write-up):
- **I1. The pairs disagree by as much as the original effect.** P1 differs by +2.65 pp between pairs. Under seed 1, ctrl was worse than F (+0.54 pp crash at S40) and rand40 better (−1.67 pp). Under seed 2 both are within ±0.26 pp of F at S40. So differences between training seeds are as large as the seed-1 effect itself. The GR-1 result should be labelled "not reproduced with a second seed", as GR-1b prereg line 70 requires.
- **I2. Two exploratory patterns appear in both pairs. They should not be promoted, and there are no intervals behind them.**
  - The pooled log-ratio DiD is about −0.035 in both pairs, a crash ratio about 3.5 % smaller.
  - rand40 has fewer floor strikes at S40 than ctrl in both pairs: −0.44 and −0.38 per 100 episodes.

## Limitations of this review

- **Blocked tool calls:**
  - heredocs;
  - `ls -la` on the results root;
  - `git diff --stat A B`.

  I worked around them with Glob, `git show --stat` and Python writing only to my directory. The training and evaluation commits are adjacent, so `git show --stat 7498c81` covers the full diff between them.
- **Host faults:** I relied on the fault counts recorded in the reports; I did not read the kernel journal myself.
- **Not checked:**
  - the seed-1 original checkpoints under `runs/` (the seed-1 copies' SHAs were verified);
  - checkpoint contents (I never loaded a checkpoint);
  - whether CPU work ran during the GR-1b evaluation.
- **Stray file:** I left a harmless `probe_write_test.txt` (contents "ok") in my audit directory.

Files are in `/home/fair/motar_tmp/gr1b_resume_20261105/opus_review/`:
- `audit_gr1b.py`
- `audit_results.json`
- `rerun_pair_analysis_s2.json`
