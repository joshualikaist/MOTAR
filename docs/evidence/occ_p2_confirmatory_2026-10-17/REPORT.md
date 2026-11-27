# OCC-P2 independent confirmatory results — 2026-10-17

Completed at 2026-10-17T08:49:13.387063+00:00 (17:49:13 KST), wall time 2.493 h. All 112 cells / 57,344 primary quota episodes accepted, no cell integrity issues and no overlap with the frozen OCC-P1 scene-hash set. GPU evaluation has ended. Source `5ff285ad053f181c5a1c86dddb2462bb44e6327b`; policy/checkpoint, actor, tracker, motion and LOS runtime contracts unchanged.

OCC-P1 is exploratory; the results below are independent confirmation relative to OCC-P1. Seeds 4301–4307 previously appeared in hidden-start B, as disclosed before measurement. These are not globally unused seeds.

Raw: `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2`. SHA bindings and byte-identical compact evidence are in `index.json`. External backup is NOT_COMPLETED. Raw is untracked; merge or symlinks do not commit it. This compact package and the completion WORKLOG entry are recorded together in their own local result commit under the user-approved Git traceability protocol. The simulation source commit remains the one above; the result-record commit does not imply a remote push or main merge.

## Encounter frequency and episode outcomes

Each row has 14 seeds / 7,168 episodes. Rates are equal-weight seed means per post-first-fused-acquisition observation minute. CIs are seed-block percentile bootstrap 95%; exposures are pooled hours. The primary count is centre-ray geometric occlusion **in view**, including blocked re-entry and left-prevalent intervals; it is not full-silhouette occlusion or every physical LOS crossing. Strict clear-to-blocked incidence is supplemental. Capture/crash/timeout shorten observation; no counterfactual exposure after termination is inferred.

| Group | Bars | Exposure h | Intervals | Rate/min [95% CI] | Strict onset/min | Capture / crash / timeout, n | Never acquired, n |
|---|---:|---:|---:|---|---:|---|---:|
| static | 70 | 6.982 | 436 | 1.041 [0.962, 1.121] | 0.876 | 7143 / 25 / 0 | 0 |
| static | 115 | 7.093 | 823 | 1.933 [1.762, 2.122] | 1.550 | 7092 / 76 / 0 | 0 |
| static | 160 | 7.226 | 1351 | 3.116 [2.947, 3.274] | 2.408 | 7001 / 167 / 0 | 0 |
| static | 205 | 7.389 | 1848 | 4.167 [3.975, 4.361] | 3.061 | 6862 / 304 / 2 | 1 |
| moving | 70 | 7.831 | 1663 | 3.539 [3.348, 3.749] | 3.007 | 7064 / 104 / 0 | 0 |
| moving | 115 | 8.392 | 2965 | 5.886 [5.673, 6.110] | 4.732 | 6987 / 180 / 1 | 0 |
| moving | 160 | 8.697 | 4084 | 7.826 [7.666, 7.978] | 5.877 | 6774 / 393 / 1 | 0 |
| moving | 205 | 9.213 | 5315 | 9.616 [9.445, 9.828] | 6.683 | 6518 / 649 / 1 | 0 |

## Frozen confirmatory contrasts

Six one-sided paired t tests across 14 seed blocks (df 13), one Holm family. CIs below are nominal two-sided 95%; simultaneous Bonferroni 99.1667% CIs and bootstrap sensitivities are retained in analysis.json. Every contrast has 14/14 positive seed differences; exact sign-flip sensitivity p=0.00006103515625 (seed symmetry assumption, not randomized assignment).

| Contrast | Difference / min | Nominal 95% CI | Holm p | Decision |
|---|---:|---|---:|---|
| static_205_minus_70 | 3.126 | [2.893, 3.359] | 5.03e-13 | SUPPORTED |
| moving_205_minus_70 | 6.078 | [5.790, 6.366] | 2.98e-15 | SUPPORTED |
| moving_minus_static_70 | 2.498 | [2.273, 2.723] | 1.93e-12 | SUPPORTED |
| moving_minus_static_115 | 3.953 | [3.630, 4.276] | 1.09e-12 | SUPPORTED |
| moving_minus_static_160 | 4.711 | [4.467, 4.955] | 7.89e-15 | SUPPORTED |
| moving_minus_static_205 | 5.449 | [5.078, 5.821] | 2.13e-13 | SUPPORTED |

RQ1: **SUPPORTED** for the 205-versus-70 endpoint increase in each lineage. RQ3: **SUPPORTED** for Moving E2_direct_l2_v1 versus Static at all four densities. These are closed-loop whole-system effects, not isolated causal effects of target speed. Endpoint inference does not prove monotonicity at all intermediate densities.

RQ2: **INCONCLUSIVE_EQUIVALENCE_NOT_TESTED**. No defensible equivalence margin was available before measurement; no equivalence test or claim. Observed seed-median endpoint differences (nominal 95% CI): Static -0.125 s, CI [-0.23865791982900533, -0.011342080170994612]; Moving -0.021 s, CI [-0.07884873276835426, 0.03599158991121139]. These are censor-selected elapsed observations, not latent physical durations.

## Duration and censoring

Quantiles describe observed elapsed intervals, including competing exits, at 10 Hz. Threshold ranges are **identification bounds, not confidence intervals**: known-at-least proportion through known-at-least plus short censored intervals. FOV/range exit is a competitor/censor, not geometric restoration. Geometry-restored-only and incident-only quantiles, KM risk support and Aalen–Johansen competitors are in analysis.json. KM assumes independent censoring; that assumption may fail here.

| Group | Bars | Q25 / median / Q75, s | P90 / P95, s | ≥1 s lower–upper % | ≥2 s lower–upper % | Censored / all | FOV / crash ends, n |
|---|---:|---|---|---|---|---|---|
| static | 70 | 0.4 / 0.8 / 1.2 | 1.5 / 1.7 | 39.68–58.72 | 1.83–21.56 | 86 / 436 | 79 / 7 |
| static | 115 | 0.3 / 0.7 / 1.0 | 1.4 / 1.6 | 32.69–54.92 | 1.70–24.42 | 187 / 823 | 168 / 19 |
| static | 160 | 0.4 / 0.7 / 1.0 | 1.4 / 1.6 | 32.27–55.51 | 2.44–26.57 | 326 / 1351 | 270 / 56 |
| static | 205 | 0.3 / 0.7 / 1.0 | 1.3 / 1.5 | 29.27–56.06 | 1.24–30.09 | 533 / 1848 | 442 / 91 |
| moving | 70 | 0.6 / 0.9 / 1.2 | 1.6 / 2.0 | 45.64–57.25 | 5.41–19.18 | 240 / 1663 | 212 / 28 |
| moving | 115 | 0.6 / 0.9 / 1.2 | 1.7 / 2.0 | 43.98–59.02 | 5.56–22.29 | 519 / 2965 | 463 / 56 |
| moving | 160 | 0.5 / 0.9 / 1.3 | 1.7 / 2.1 | 45.59–64.05 | 6.61–27.64 | 887 / 4084 | 762 / 125 |
| moving | 205 | 0.5 / 0.9 / 1.3 | 1.8 / 2.1 | 43.74–67.07 | 7.34–33.83 | 1450 / 5315 | 1258 / 192 |

Capture/timeout ended zero active primary occlusion intervals in all groups, even though these episode outcomes occurred. No interval was observed to last 5 s; all eight groups have zero risk-set support at 5 s, so that tail is NOT_ESTIMABLE. This does not establish impossibility. Five seconds remains a separate stress-test candidate.

## Restoration, competitors and measurement resumption

Reacquisition means eventual fused-measurement resumption **conditional on actual geometry restoration**. FOV exits and crashes are not in its denominator. The previous-frame-fused-invalid subset avoids counting carried-through measurements as newly recovered. Neither denominator proves persistent target identity, recovery after tracker expiry, or long-duration recovery ability.

| Group | Bars | Geometry restoration / all intervals, n | Fused resumption after restoration | Prior-frame-loss subset | Crash CIF at 2 s % | FOV CIF at 2 s % |
|---|---:|---|---|---|---:|---:|
| static | 70 | 350 / 436 | 350 / 350 | 172 / 172 | 1.61 | 18.12 |
| static | 115 | 636 / 823 | 636 / 636 | 299 / 299 | 2.31 | 20.41 |
| static | 160 | 1025 / 1351 | 1025 / 1025 | 429 / 429 | 4.15 | 19.99 |
| static | 205 | 1315 / 1848 | 1315 / 1315 | 533 / 533 | 4.92 | 23.92 |
| moving | 70 | 1423 / 1663 | 1423 / 1423 | 545 / 545 | 1.56 | 12.21 |
| moving | 115 | 2446 / 2965 | 2446 / 2446 | 968 / 968 | 1.82 | 15.11 |
| moving | 160 | 3197 / 4084 | 3197 / 3197 | 1191 / 1191 | 2.94 | 18.12 |
| moving | 205 | 3865 / 5315 | 3865 / 3865 | 1397 / 1397 | 3.48 | 23.14 |

## Proposed next package (not executed)

1. Finish external raw backup after the user supplies a destination; retain all local raw, failed attempts, WIP and stash meanwhile. This compact result package is locally committed separately from the measurement source; main merge/push remain unapproved.
2. Prepare a CPU-only scenario/prefix feasibility package: static and moving targets, acquired-before-loss eligibility, actual geometrical blocking, separately classified FOV exits and sensor loss, matched scene and pre-loss actor-prefix hashes. Start with 1–2 s natural spells; treat 5 s as a deliberately constructed physical-occlusion stress condition. Initial distance 4–12 m remains a candidate range, not a frozen choice.
3. Proposed new files for review: `motar_eval/recovery_scenarios.py`, `tools/recovery_scenario_audit.py`, `tests/test_recovery_scenarios.py`, and a new dated scenario plan. CPU gates: prefix determinism, event timestamps, no-occlusion noninterference, no GT into online inputs, preservation of 5 s tracker expiry and the 898-D actor. Establish physical duration and usable loss samples before proposing Belief/Planner or any new GPU run.
4. Carry capture/crash/timeout and safe-abort eligibility into the next design. Safe abort was NOT_RECORDED as an independent endpoint in this OCC-P2 run. Success after geometric restoration alone must not replace end-to-end safety or long-duration recovery evaluation.

## Recorded geometry and renderer diagnostics

Offline-versus-online centre-box classification disagreed on 4 / 2,320,985 quota frames; offline-versus-mesh probe disagreed on 3 / 2,320,985. Rendered pixels were positive in 109 / 122,185 geometric-full-proxy frames; zero in 86 / 1,568,239 clear-in-view frames. These are recorded discrepancies, not zero-error renderer equivalence. The frozen primary offline centre-ray endpoint remains the one analyzed. Preserve all original classifications; audit boundary/tolerance and endpoint sensitivity on CPU before a subsequent experiment, rather than retrospectively changing this confirmatory endpoint.

A bounded independent CPU audit localized the recorded disagreements to five unique frames: four near-grazing LOS box boundaries and one horizontal FOV boundary. Alternate online/probe labels shift three in-view interval boundaries by one 0.1 s frame, with zero changes to interval counts or observation exposure in affected episodes. The six rate tests therefore remain unchanged under these alternate recorded classifications. Original offline labels and all raw remain unchanged.

## Reproduction and Git record

- Run ID: `OCC-P2`; preregistration v1: `docs/plans/prereg_occlusion_confirmatory_v1.md` and `.json`, committed before measurement in `c2d352c0d45a025e69dfe59546c2bc4d8171fc1b`.
- Actual simulation source and analysis implementation: `5ff285ad053f181c5a1c86dddb2462bb44e6327b`; GPU: NVIDIA GeForce RTX 3070; 128 environments and quota 4. Historical launch command, from the research worktree:

```bash
CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/occlusion_confirmatory.py run --approved-commit 5ff285ad053f181c5a1c86dddb2462bb44e6327b
```

This is the execution record, not approval for another GPU run. The registered existing output root must never be overwritten. The runner refuses an existing output root.

CPU-only reanalysis can read the preserved raw and write a new scratch path (choose an unused output file):

```bash
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/occlusion_confirmatory.py analyse --out /tmp/OCC-P2_reanalysis_new.json
```

The result-record commit contains only this directory's nine compact text artifacts and the appended worktree WORKLOG. It contains no raw binaries, checkpoint, symlink, user WIP or stash. See `git log --oneline -- docs/evidence/occ_p2_confirmatory_2026-10-17/` for its commit SHA. External raw backup remains incomplete.
