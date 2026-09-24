# Preregistration — independent bias-corrected replication of the H / E0 / E1 / E2 target-motion result

- **Written:** 2026-09-24.
- **Status:** `DRAFT_FOR_USER_REVIEW`. The text becomes binding when it is committed, and it must be
  committed before any cell runs. **Not run.**
- **Authority:** the canary and the replication each need the user's explicit approval to use the GPU.
  This document authorises no PPO training.

The original result is closed: `docs/prereg_2026-09-17_target_motion_complexity_e0_e1_e2.md`,
`results/target_motion_e0_e2_2026-09-18/` and `docs/results/target_motion_generalization_2026-09-19.md`.
This document changes none of them. The replication is a new study, and it is
**not part of the original preregistration**.

## 1. Question and estimand

With seven fresh evaluation seeds, do the three effects against the training target replicate for
frozen policy F, in direction and magnitude?

The original effects are E0 − H = −3.58 pp, E1 − H = −2.82 pp and E2 − H = +1.33 pp in capture, over
three seeds.

**This is an INDEPENDENT BIAS-CORRECTED REPLICATION, not a byte-for-byte repetition of the original
estimator.**
- The replication uses a **bias-corrected per-environment quota estimand**: the first 16 completed
  episodes of each of the 128 environments.
- The 2026-09-18 campaign used the **pooled legacy stopping window**: every episode finished by the
  step on which the pooled count reached 2,048.
- The legacy-window statistic is kept as a diagnostic, for comparison with the original only.

## 2. Held identical to the original (only measurement instrumentation is added)

- **Policy:** frozen policy F, `last_gen_ppo_ep_25000_rew_39.742134.pth`, SHA-256
  `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`.
- **Launcher:** `aerial_gym/rl_training/rl_games/eval_navrl_target_behavior_arms.sh`, byte-unchanged. It
  replays the checkpoint's recorded training environment and defines the four arms. The observation
  (898-D actor), reward, controller, safety filter (riskcap), the H / E0 / E1 / E2 target contracts,
  termination (capture radius 0.5 m, 600 steps of 0.1 s) and 128 environments therefore come from the
  same code path.
- **Densities:** 70 / 115 / 160 / 205 bars.
- **Aggregation:** the original Amendment 1 §A1.3 rule. Per seed, a cell's rate is averaged without
  weights over the four densities, and the seed is the unit of inference. Episodes are never pooled as
  independent samples.
- **Code:** the replication runs at the commit that contains this document. That commit differs from the
  original run's in two ways:
  - the episode ledger below, which is off unless switched on;
  - unrelated later commits.

  Canary check `result_contract_unchanged` (section 6) tests whether that difference matters.

## 3. Seeds, cells and episodes

- **Seed rule, fixed before any data:** the seven smallest integers above the original 4101–4103 that
  have no prior use in any seed context. The result is **4104, 4105, 4106, 4107, 4108, 4109, 4110**.
- **Prior-use check (2026-09-24):**
  - Scope: seed contexts in 40,709 files across `results/`, `docs/`, training-session logs, source
    receipts, tools, tests and preregistrations, including CSV seed columns and run-directory names.
  - Result: `PRIOR_USE = 0` for all seven.
  - Control: the same scan finds 4101 in 68 files.
- **Cells:** 4 arms × 4 densities × 7 seeds = **112 cells**, run seed by seed (the order printed by
  `tools/target_motion_replication.py plan`).
- **Episodes:** **exactly 2,048** per cell, that is **16 episodes** from each of the 128 environments.
  That gives 112 × 2,048 = **229,376** analysed episodes.

## 4. Instrumentation contract (the episode ledger)

`aerial_gym/task/navrl_task/navrl_episode_ledger.py` is switched on by `NAVRL_EPISODE_LEDGER=1`,
`NAVRL_EPISODE_QUOTA_PER_ENV=16` and `NAVRL_PLAYER_GAMES_CAP=1000000000`, with `PLAY_GAMES_NUM=2048`
unchanged. The task refuses any other combination.

- **One row per finished episode**, in `episodes.jsonl` next to the cell's `result.json`. Each row holds:
  - `cell_id`, `arm`, `bars`, `seed`;
  - `env_index`, the per-environment `env_episode_index` and `episode_id`;
  - `completion_step` and `completion_rank`;
  - `outcome`, and `capture`, `crash`, `timeout` (exactly one is true);
  - `min_relative_distance_m`;
  - `in_quota` and `in_legacy_window`.
- **`min_relative_distance_m`** is the per-episode minimum of ‖target − robot‖ at the end of each step
  (`ep_min_goal_dist`), taken before any reset. It is defined for every outcome, crashes included. The
  original run recorded it only for non-crash episodes, so it was `NOT_RECORDED` there.
- **Quota rule (the bias-corrected estimand used for inference).**
  - Only the first 16 episodes each environment completes are analysed, 2,048 in all.
  - Every environment keeps stepping until all have 16, so reaching a quota changes nothing in the
    simulation.
  - The ledger then writes `episode_ledger_summary.json` (status `COMPLETE`, with the rows file's
    SHA-256) and ends the process.
- **Legacy window (diagnostic only).**
  - This is every episode finished at or before the first step on which the pooled count reached 2,048.
    It is what the unchanged aggregate `result.json` summarises, and it is how the original counted.
  - It is reported beside the quota estimate as the measured length bias, and never used for the
    verdict.
  - Why it is not used: the count-based stop drops each environment's episode still in flight, mostly
    long timeouts. A CPU renewal model (a model, not a measurement) gives capture inflation of about
    +1.8 / +0.7 / +0.4 pp at 9 / 2.5 / 1.2 % timeouts. The quota rule's bias in the same model is within
    ±0.1 pp.
- **Code change:**
  - `navrl_task.py` gains 16 lines: it constructs the ledger, then calls `record(...)` after outcomes
    resolve and before `reset_idx`, and finalises when every environment has its quota.
  - `runner.py` gains one line: the player's cap reads `NAVRL_PLAYER_GAMES_CAP` before `PLAY_GAMES_NUM`.
    Unset, behaviour is unchanged.
- **CPU tests** (`tests/test_navrl_episode_ledger.py`):
  - input tensors are unchanged, and no torch, numpy or Python RNG state is consumed;
  - the ledger source contains no random draw and no reference to observations, rewards or resets;
  - the hook sits after outcomes resolve and before `reset_idx`;
  - the quota set is exact, and the legacy window equals the old stop rule;
  - the quota estimator is unbiased where the old rule is not.

## 5. Statistical contract (fixed before any data)

The primary analysis is **the new seven-seed replication alone**. The original three seeds are not
pooled into any confirmatory test. All analysis code is fixed in
`tools/target_motion_replication.py analyze`, committed with this document.

**Confirmatory contrasts:** capture rate, quota estimand, for **E0 − H, E1 − H and E2 − H**.

**Reported for every contrast, confirmatory or not:**
- the 7 individual seed-paired effects;
- the mean paired effect;
- the BCa 95 % confidence interval, with 20,000 resamples and seed 4100, using the original
  implementation (reproduced to 12 decimal places in `tests/test_target_motion_replication.py`);
- the exact two-sided paired sign-flip p over all 2⁷ = 128 sign patterns;
- the Holm-adjusted p across the three confirmatory contrasts;
- the sign-consistency count (how many of the 7 seeds share the sign of the mean).

**Classification.** A contrast is classified as a successful directional statistical replication when
(1) the replication mean preserves the direction observed in the original campaign (−, −, +) and
(2) its Holm-adjusted exact paired sign-flip p-value across the three confirmatory contrasts is below
0.05.
- Individual seed sign consistency is reported descriptively and is **NOT** an additional pass/fail
  criterion.
- Study verdict: `REPLICATED` if all three contrasts are successful directional statistical
  replications; `PARTIALLY_REPLICATED` if one or two are; otherwise `NOT_REPLICATED`.

**Interpretation.** Effect magnitude, direction, seed consistency and uncertainty come first. The
significance test is one component of the reading, and a contrast is never reduced to its p-value.

**Resolution of the exact test, stated before data:**

| Seeds | Smallest two-sided p | Smallest Holm-adjusted p over 3 contrasts |
| --- | --- | --- |
| 3 (original) | 0.25 | 0.75 |
| 6 | 0.03125 | 0.09375 |
| **7 (this study)** | **0.015625** | **0.046875** |

With seven seeds, the exact two-sided sign-flip p-value grid is fine enough that a Holm-adjusted
p < 0.05 is possible for the three-contrast family. Seven is the smallest chosen replication size that
gives useful exact-test resolution for this Holm-corrected family. **It does not guarantee rejection
of any null hypothesis.**

**Secondary, not confirmatory:**
- crash and timeout rates;
- the within-E contrasts;
- `min_relative_distance_m` (reported with intervals, without a replication claim, because the
  original could not measure it);
- whether E0 is the lowest and E2 the highest seed-averaged arm (descriptive);
- the twelve per-density contrasts against H (exploratory, uncorrected);
- the legacy-window estimate, and the quota − legacy difference (the measured length bias).

**No optional stopping. No threshold, estimand, estimator or contrast changes after any outcome is
opened.**

**Combined original + replication:** shown only after the replication result is committed and
frozen.
- It is a secondary, clearly labelled descriptive summary over ten seeds, using the legacy-window
  estimator so the two studies are counted the same way.
- It is never a confirmatory test.

## 6. Execution and stopping rules

1. **Preconditions,** enforced by `tools/target_motion_replication.py run` before every cell:
   - no modified tracked file;
   - this document is committed;
   - HEAD is unchanged since the run started;
   - exactly one RTX 3070 is present. The GTX 1650 Ti host is never used or pooled.

   The launcher also checks the checkpoint SHA.
2. **Canary** (`run --stage canary`) uses the OLD known cell H / 70 bars / seed 4101, so it consumes no
   fresh seed. Its purpose is **instrumentation non-interference only**; its capture rate is never
   research evidence.
   - It runs the cell twice, ledger off and ledger on. Both runs use the existing trajectory digest, a
     SHA-256 over every step's position, orientation and executed command up to the aggregate export.
   - Both runs also use the existing observation dump, sampling the exact actor-observation tensor every
     20th step with no decimation.

   All eleven gates must pass:
   1. clean process return codes;
   2. with the ledger off, the historical cell is reproduced under the legacy result contract: the
      aggregate `outcome` and the episode count equal the 2026-09-18 `result.json`;
   3. with the ledger on, `result.json` equals the ledger-off run in every field except the run nonce;
   4. the trajectory digests are identical over the matched scientific window: same SHA-256, same step
      count;
   5. no additional RNG consumption. Evidence: gates 3, 4 and 6 hold, and the CPU test that the ledger
      leaves torch, numpy and Python RNG state untouched passes on this commit;
   6. no actor-observation change: every actor-observation row sampled by both runs is bitwise
      identical, and at least one common step exists;
   7. no physics, control, reward or target change. Evidence: gates 3 and 4 hold, and against the
      baseline commit `839cc8e` the only simulator-code changes are the ledger module, the guarded hook
      in `navrl_task.py` (no removed line) and the player cap in `runner.py`;
   8. the exact quota ledger: status `COMPLETE`, 2,048 valid episodes, 0 outcome-partition violations;
   9. exactly 16 quota episodes for each of the 128 environments;
   10. all episode IDs unique, per-environment episode indices contiguous from 0, and completion ranks
       contiguous from 1;
   11. `min_relative_distance_m` recorded as a finite, non-negative value for every episode, crash
       episodes included.

   Any failure gives `CANARY_FAIL`. The study stops before a fresh seed is touched, the failure is
   preserved on disk, and it is reported to the user. Nothing is repaired and silently continued.
3. **Main** (`run --stage main`) needs `CANARY_PASS` on the same commit.
   - All 112 cells run; there is no interim analysis.
   - A failed or incomplete cell stops the run. Only that cell may be re-run, with the reason recorded.
   - No seed is replaced or added.
4. **Outputs:** `results/target_motion_replication_7seed/`, with one folder per cell holding
   `result.json`, `receipt.json`, `cell.log`, `episodes.jsonl` and `episode_ledger_summary.json`.
   `analysis.json` follows after all 112 cells.
5. **Runtime:**
   - The original rule took 3.15 h for 48 cells, about 236 s per cell.
   - The quota rule needs 1.56–2.01 × the steps, projected from the original cells' episode-length
     summaries.
   - That gives about **13 h** for 112 cells (range 11.5–14.8 h), plus about 12 min for the canary.
   - The canary measures the real figure.

## 7. Not claimed

- No change to the original result or its preregistration.
- No guaranteed significance.
- No new target law. TM-E3 and TM-E4 stay PLANNED.
- No result for policy R.
- No real-flight claim.
- No PPO training.
