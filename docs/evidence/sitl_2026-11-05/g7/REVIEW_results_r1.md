# G7 independent results review: Policy F under held or delayed actions

**Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only. I was given paths and the fixed criteria but no author conclusion.

**Limits:** CPU only on cores 6, 7, 12 and 13. Every probe ran with a 4 GiB memory cap and 300 s CPU cap. No GPU, no `aerial_gym` or Isaac imports, no experiments, and no git or Beads changes. I wrote only to `opus_results_review/`. I can't observe runtime behaviour on the GPU, only the recorded artifacts.

## Verdict

The measurement integrity checks pass. The three preregistered readings reproduce exactly from raw ledgers:

| Arm | Pooled crash Δ (pp) | 95 % CI | Rule outcome |
|---|---|---|---|
| k2d0 (hold 0.2 s) | +2.001953125 | [0.977, 3.027] | **INCONCLUSIVE** |
| k4d0 (hold 0.4 s) | +10.474 | [9.229, 11.743] | **HARMFUL** |
| k1d1 (delay 100 ms) | +1.514 | [0.537, 2.515] | **TOLERANT** |

## Integrity checks

| Check | Result | Evidence |
|---|---|---|
| Exact seven-cell plan, in order, seed 4501 | PASS | The run report's (arm, density, k, d) list equals the prereg plan; reading `COMPLETE`; 7/7 cells `OK`, no problems recorded |
| Quota 2048 and 128 envs × 16, every cell | PASS | Each ledger has 2048 quota rows from 128 envs, exactly 16 per env, each env's first 16 episodes. Episode ids are unique, outcomes exclusive. Recomputed counts equal the summaries; the ledger SHA-256 matches each summary |
| Source, checkpoint and import origins | PASS | HEAD is `06383eb`, branch `camonly/dev`, working tree clean now. Every receipt shows the measured commit with 0 dirty files. The checkpoint file re-hashes to `f7022139…ad40` (= F). Import origins of the report and all 7 recorders are inside MOTAR-camonly |
| Diff `3f6f7e5..06383eb` | PASS | The diff is limited to the runner's byte guard (`actionhold_run.py`), its test, amendment 4 and docs/evidence files. `aerial_gym/`, the hook, the analyser, the shared runners and the G7 prereg are unchanged. The prereg, hook and analyser were committed in `f3930c6` (03:24 KST), before launch (14:27:50) |
| Byte guard | PASS | It only enforces the prereg's existing "byte-identical" criterion. Its test has a positive control (same bytes accepted) and negative controls (reformatted JSON and a missing file rejected) |
| Receipts versus native F | PASS | The environment snapshot differs from GR-0 only in `RUN_ID` and `NAVRL_SENSOR_ABLATION=full`, ignoring output paths and the nonce. The ablation module SHA matches, the paths are in-tree, and LiDAR is 72×4 |
| Recorder violations | PASS (0) | Motion gate PASS with 0 violations in all 7 cells. Swept-clearance and wall violations are 0, `partition_violations` is 0, `failed_closed` is 0, and there is no `kroc2027_violations.jsonl` |
| Host | PASS | Boot id `407ae422…` at launch, at every cell, at completion and now. Fault count 0 → 0; online CPUs `0-3,6-23` throughout |
| Resource guard | PASS (watchdog, not a kernel cap) | Exit 0, `stop_reason` null, 0 live descendants, 4710 samples at 0.25 s. The wrapper bytes equal the committed copy (`75e18306…`). The r3 validation (14:26, before launch) has one positive control and RSS, timeout and descendant negative controls. Peak aggregate RSS was 12,414,902,272 B, **96.4 % of the 12 GiB limit** |

**Identity, k1d0 against native F 205/4501: PASS**
- `episodes.jsonl` is byte-identical: both are 1,118,450 bytes with SHA-256 `8899856375b19c6d1865e0f807827e479077325045bf82f1c44ab3113f77e6ad`, and the parsed rows are equal.
- `result.json` differs only in `/condition/evaluation_nonce`.
- `episode_ledger_summary.json` is identical, and `kroc2027_episodes.jsonl` is byte-identical.
- The k1d0 sidecar shows the patch applied, config k=1 d=0, and 981 calls, equal to `task_steps`.

**Treatment sidecars, all six cells: PASS (non-vacuous)**

Each sidecar shows the patch applied, the correct config, and calls equal to `task_steps`, written at `player_run_exit`.

| Cell | Treatment count | Consistency check |
|---|---|---|
| k2/205 | held 68,211 | latched + held = calls × 128 exactly; held:latched 0.98 |
| k2/70 | held 50,504 | latched + held = calls × 128 exactly; held:latched 0.98 |
| k4/205 | held 106,632 | latched + held = calls × 128 exactly; held:latched 2.88 |
| k4/70 | held 121,563 | latched + held = calls × 128 exactly; held:latched 2.87 |
| d1/205 | 3658 zero-action steps | 125 more than completed episodes |
| d1/70 | 2484 zero-action steps | 124 more than completed episodes |

The ratios of about 1 and about 3 show the latch cadence follows `sim_steps`, which goes up once per env step (`env_manager.py:474`) and is zeroed in `reset_idx` (`:331`). The delay counts fit about 128 initial starts plus one hover step per new episode, minus episodes that ended on the final step.

## Statistics, recomputed independently

- **Unchanged analyser:** re-run in isolated mode (`-I`) into my directory. Output `repro_analysis.json` is **byte-identical** to `analysis.json` (SHA-256 `ea315727…0853`; numpy 1.23.0).
- **My own code:**
  - Exact fractional Δs: k2 = 1025/512, k4 = 10725/1024, d1 = 775/512 pp.
  - Bootstrap reimplemented two ways (`choice` and `integers`), 10k resamples, seed 20261105, same stream order: the CIs equal the analyser's exactly.
  - Each rule checked by hand: TOLERANT needs Δ ≤ 2 and CI upper ≤ 4; HARMFUL needs Δ ≥ 5 and CI lower > 0.
- **Sensitivity (reported only, not a reading):**

  | Variation | Effect on readings |
  |---|---|
  | Separate RNG per arm | Unchanged |
  | Five other bootstrap seeds | Unchanged; CI ends move by at most 0.05 pp |
  | Env-cluster bootstrap (128 env clusters) | Unchanged; k2 [1.00, 3.00], k4 [9.30, 11.67], d1 [0.49, 2.56] |
  | Env-level variance ratio against binomial | 0.88 to 1.32, so little within-env clustering |

## Consequential findings

1. **k2d0 is INCONCLUSIVE by one episode.**
   - Arm crashes are 291 against 209 for F out of 4096 episodes each, so 82 extra crashes. TOLERANT allows at most 81.
   - One crash fewer gives Δ = 1.978, and with CI upper 3.03 that would read TOLERANT.
   - The reading is still correctly INCONCLUSIVE and must not be described as "effectively tolerant".
   - The CI excludes 0, so a small harm is resolved, concentrated at 205 bars (+3.22 pp against +0.78 at 70).
   - Neither of the prereg's k2 consequences is triggered.
2. **k1d1 is TOLERANT, but the CI excludes 0.**
   - 205 bars: +2.59 pp. 70 bars: +0.44 pp.
   - TOLERANT means the harm fell inside the prereg's +2 / +4 margin. It does not mean no effect.
3. **k4d0 is HARMFUL, and most so at high density.**
   - 205 bars: crash 8.84 % → 26.27 % (+17.4 pp), capture down 17.4 pp.
   - 70 bars: +3.5 pp.
4. **Identity was tested only at 205/4501.** The 70-bar comparisons assume the camonly `full` path with the hook installed reproduces GR-0's native 70-bar cell. G7 did not test this directly; it is a design limit, not a defect.
5. **The governor remains in the loop.** Riskcap, the controller and the altitude PI act every step on the held or delayed command. These results measure the tolerance of the F + governor stack. "Open loop" here applies only at the policy level.
6. **Operational: memory headroom was 3.6 %.** A larger G7-like run may trip the watchdog. Shared pages are counted once per process, so the true footprint is lower than the recorded figure.

## Scoped implications

- **Holding 0.2 s:** this experiment neither rules open-loop 2-step chunking out for F nor leaves it open (INCONCLUSIVE), with a small measurable harm. A trained chunked policy remains a separate, unapproved unit; this test is an upper bound on harm by design.
- **Holding 0.4 s:** open-loop 4-step holds of F's actions are harmful in this simulator, at these two densities and this seed.
- **Added 100 ms delay:** the delay was one whole step, with a hover action on each episode's first step. F's crash increase stayed within the prereg margin.
- **What this does not establish:** a latency threshold, tolerance to jitter or sub-step or longer delays, exact PX4 timing, or real-flight behaviour.

## Validity limits

- One seed (4501) and two densities. The CIs reflect only within-cell episode sampling, not seed or scene variance.
- Arm and F episodes are not paired. Episodes are treated as independent, but resets are coupled across envs through the RNG (previously recorded), so they are not strictly independent. The env-cluster sensitivity check does not address that coupling.
- With a one-episode margin, the k2 reading could flip under a different seed. These data can't decide it.
- The historical prereg and its readings are unchanged; nothing was edited.

## Checks not performed

- `tests/test_actionhold_hook.py` and `tests/test_actionhold_identity.py` were not executed. I read their code only.
- The kernel log was not re-read. Host evidence comes from the run records plus the current boot id.
- GPU exclusivity during the run can't be observed after the fact. I rely on the runner preflight's empty problem list.
- GR-0 S00's own provenance was not re-audited beyond its receipts (commit `c56be07`, clean, OK) and its ledger hashes.
- The prereg's pre-run analysis smoke test on stand-in cells was not re-run.

**Process note:** some long single-command probes were refused by the tool and I never determined why. I split them into shorter probes, which ran. My first index recorded the checkpoint re-hash as not performed; it was done afterwards and matched, which the addendum records.

Files are in `/home/fair/motar_tmp/g7_20261105/opus_results_review/`:
- `AUDIT_INDEX.json` — SHA-256 of every artifact
- `AUDIT_INDEX_addendum.json` — the checkpoint re-hash correction
- `probe1a_ledgers.json`, `probe1b_receipts.json`, `probe1c_source.json` — integrity probes, each next to the exact `.py` that produced it
- `probe2_stats.json` — the statistics and sensitivity, next to its `.py`
- `repro_analysis.json` and `repro_analysis.log` — the unchanged analyser re-run
- `probe0_write_test.txt` — a harmless write test
