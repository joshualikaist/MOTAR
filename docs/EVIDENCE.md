# Evidence

What MOTAR has established, what it has not, and where each number comes from. Every value on this
page is copied from a machine-readable record and checked against it by
[`tests/test_canonical_docs.py`](../tests/test_canonical_docs.py). The numbers are not re-derived here.

**Scope of every row:** simulation only, or offline analysis of recorded real footage, and never real
flight. `pp` means percentage points. A difference is always *treatment − reference*.

## Main results

Two frozen checkpoints carry these results. **Policy F** is ep25000 + riskcap (SHA-256 `f7022139…`):
perception → policy, safety geometry and target motion. **Policy R** is ref5in D1 ep1900 (`197ea269…`):
the observation contract only. Readaptation trains new descendants of policy F, so it is not a
frozen-policy result.

| Axis | Result | Unit of inference | Interpretation | Limit |
| --- | ---: | --- | --- | --- |
| **Perception range error** | **6.2 %** median absolute relative range error (median of block medians); 3,107 frames, 9 blocks | 9 blocks of 15 s from **one flight**. The 3,107 frames are not independent samples; no interval is recorded | An image-size range proxy is usable on real UAV footage (`SIZE_RANGE_USABLE`) | One ETH ds5 flight; apparent-size proxy, not ground-truth boxes; the attitude decomposition (E3-P) is BLOCKED |
| **Perception → policy** | **−4.57 pp** capture, 95 % CI [−6.32, −2.82] | Episodes: 3,161 / 4,099 vs 3,350 / 4,101, pooled over **two evaluation seeds**. No seed-level interval exists | Injecting the measured detector error costs policy F capture | One density (205 bars), one injector. The value was identical in three campaigns because they re-ran the same deterministic frozen arms: a determinism check, not three replications |
| **Readaptation** | **+0.73 pp**, 95 % CI [−1.04, +2.50] — **INCONCLUSIVE** | Training seed, n = 3 (t interval) | 1,000 epochs of retraining under the error do not establish a net benefit | Three training seeds (+1.41, +0.78, −0.01 pp). A residual cost remains in every seed (−1.90, −2.71, −3.84 pp), and retraining costs 0.74–1.25 pp of clean-condition capture |
| **Safety geometry** | **−1.4903 pp** crash, 95 % CI [−1.8981, −1.0826]; seed-level 95 % CI [−2.42, −0.57]; lower in **15/15** seed × density cells | The first interval pools 15 seed × density cells with episode-binomial errors (fixed effect). The seed-level interval uses **3 evaluation seeds** (t, df = 2). The 15 cells share three seeds and one policy, so they are not 15 replicates | Measuring clearance along the turning arc lowers crash rate relative to riskcap's straight corridor | A configured contrast on policy F. Name it the "implemented arc-clearance speed filter": it is not a DWA planner and not a collision guarantee |
| **Target motion** (current evidence) | vs the training target H: **E0 −5.37 pp** [−5.78, −4.85], **E1 −3.45 pp** [−3.79, −3.11], **E2 +1.35 pp** [+1.07, +1.87] | Evaluation seed, **n = 7** fresh seeds (seed-paired BCa); exact sign-flip p = 0.015625 and Holm-adjusted p = 0.046875 for each, at the resolution boundary of the seven-seed design | All three preregistered capture contrasts met the directional replication criterion in the independent seven-seed bias-corrected replication. Target motion is not a monotonic difficulty ladder: the static target scores lowest and the obstacle-aware target highest | Bias-corrected per-environment quota, 229,376 primary episodes, policy F, one arena, four densities. Effect sizes, intervals and seed consistency (7/7 in each contrast, descriptive) are reported alongside the tests. The aggregate E2 − H contrast replicated, but its positive direction was not uniformly expressed at high obstacle densities (5/7 seeds at 160 bars, 4/7 at 205; exploratory). The historical pooled stopping window attenuated all three contrasts toward zero on the same seeds |
| Target motion, original campaign (historical) | vs the training target H: **E0 −3.58 pp**, E1 −2.82 pp, **E2 +1.33 pp**; E2 − E0 +4.91 pp | Evaluation seed, **n = 3**, legacy pooled stopping window (seed-paired BCa). The smallest exact two-sided permutation p attainable is 0.25 | Target motion is **not** a monotonic difficulty ladder. The static target scores lowest and the obstacle-aware target highest | All six arm-level capture contrasts are 3/3 same sign; the E2 − H crash contrast is sign-mixed. The twelve per-density contrasts are exploratory: 10 of 12 are 3/3 same sign, and E2 − H is mixed at 115 and 160 bars. Policy F, one arena, four densities; no significance claim. The mechanism (below) is association only |
| **Observation contract** | **−48.967 pp** capture, 95 % CI [−50.113, −47.821] — **MATERIAL_LOSS** | Paired evaluation seed, n = 3 (t, df = 2) | Rendering the target as a shaded mesh instead of the analytic proxy collapses policy R's capture | **Causality NOT_TESTED**: no renderer property was isolated as the cause. Measured on policy R (ref5in D1, ep1900, 70 bars), not policy F |

**Target-motion mechanism, association only (original campaign).** In 95.42 % of E0 (static-target) timeout episodes the
target was never acquired even once (1,667 of 1,747). The moving-target arms show 83–84 %, and E0 has
the most timeouts (7.11 % of episodes, against 1.17 % for E2). This is *consistent with* a
first-acquisition / visibility mechanism: a static target that starts out of view never moves into
view. No mediation experiment was run, so it is not a causal explanation.

### Sources

| Axis | Canonical record | Machine-readable value |
| --- | --- | --- |
| Perception range error | [`results/eth_ds5_e3s_2026-09-10/README.md`](../results/eth_ds5_e3s_2026-09-10/README.md) | `run/e3s_result.json` → `primary.gates.G2_central.median_of_block_medians` |
| Perception → policy | [`results/perception_p10_seed_replication_2026-09-10/README.md`](../results/perception_p10_seed_replication_2026-09-10/README.md) | `summary.json` → `per_training_seed.*.p9_cost_on_source` |
| Readaptation | same folder; preregistration [`preregistration_p10_seed_replication_2026-09-10.md`](preregistration_p10_seed_replication_2026-09-10.md) | `summary.json` → `R2_seed_level_t` |
| Safety geometry | [`results/independent_verification_2026-09-07/README.md`](../results/independent_verification_2026-09-07/README.md) | `recomputed.json` → `contrasts["dwa_arc-riskcap"].pooled` and `.seed_t` |
| Target motion (current) | [`results/target_motion_replication_7seed/README.md`](../results/target_motion_replication_7seed/README.md) | `analysis.json` → `estimators.quota.primary`; per-cell counts in `cells.csv` |
| Target motion, original campaign | [`docs/results/target_motion_generalization_2026-09-19.md`](results/target_motion_generalization_2026-09-19.md) and its [visibility audit](results/target_motion_visibility_reacquisition_audit_2026-09-19.md) | `results/target_motion_e0_e2_2026-09-18/canonical_summary.json`; per-density signs from the 48 `result.json` files in `results/target_motion_e0_e2_evaluation_2026-09-18/` |
| Observation contract | [`results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md`](../results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md) | `summary.json` → `primary` |

All six rows are also recorded in
[`quantitative_positioning_registry.json`](quantitative_positioning_registry.json)
(`internal_motar`), the single registry the site, README and figures draw from.

Minimum-distance results are **WITHHELD_SEMANTIC_MISMATCH**: capture uses swept closest approach,
but recorded distance samples step ends. Values remain in the frozen analysis.
[Current machine-readable status](../results/target_motion_replication_7seed/summary.json);
[raw availability and archive SHA](REPRODUCIBILITY.md#closed-target-motion-replication).

## Supporting results

| Result | Value | Verdict | Record |
| --- | ---: | --- | --- |
| Latency compensation (ego-motion corrected) | capture 37.82 % → 78.04 % (+40.21 pp) | GO in the registered contract; real-world latency is not solved | [`results/navrl_v2_latency_ego_motion/summary.md`](../results/navrl_v2_latency_ego_motion/summary.md) |
| Learned detector in the loop | −0.015 pp, 95 % CI [−1.752, +1.723] | Non-inferiority PASS at a −2 pp margin; not superiority | [`…replication_seed97_101_schema2/summary.md`](../results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/summary.md) |
| Riskcap adaptation (produced policy F) | capture +3.75 pp, 95 % CI [+1.30, +6.19] | Capture supported; crash not confirmed (its interval contains zero) | [`results/navrl_v2_riskcap_postadapt/summary.md`](../results/navrl_v2_riskcap_postadapt/summary.md) |
| Temporal perception selector | +0.00697 validation utility | Selected on validation; not held-out superiority (S4 test 0.4701 carries a generalization warning) | [`results/perception_temporal_p6_p7_2026-09-08/README.md`](../results/perception_temporal_p6_p7_2026-09-08/README.md) |
| Matched external comparisons | **Class A = 0** | No external number is subtracted from, or ranked against, a MOTAR result | [`relation_to_published_systems_2026-09-16.md`](relation_to_published_systems_2026-09-16.md) |
| Elastic Tracker external baseline, gate B0 | UPSTREAM_REPRODUCTION_FAIL | Environment-blocked (ROS needs `sudo`). Not a performance comparison | [`external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md`](external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md) |

## Not established

These are stated so that no page implies them:

- Real-flight performance, sim-to-real transfer, deployment readiness, or any hardware result.
- A formal collision-safety guarantee for any filter.
- A net benefit from readaptation to measured perception error (P10 INCONCLUSIVE).
- A cause for the D8b loss (`causality_vs_d8b = NOT_TESTED`).
- A live real-image perception → policy connection (`LIVE_RGB_POLICY = NOT_TESTED`).
- Metric target range, camera bearing in degrees, or persistent target identity (BLOCKED by missing ground truth).
- Any advantage over a published system (Class A matched comparisons = 0).
- Reactive or learned evaders (TM-E3, TM-E4 are PLANNED, not implemented).

## Status vocabulary

| Label | Meaning |
| --- | --- |
| COMPLETED | The named stage ran; this says nothing about whether the verdict is positive |
| INCONCLUSIVE | The preregistered rule neither confirmed nor withdrew the claim |
| MATERIAL_LOSS | A registered contrast crossed its loss margin |
| NOT_TESTED | An interface exists, but the claim was never evaluated |
| BLOCKED | Required ground truth, data or authority is unavailable |
| WITHDRAWN | An earlier interpretation was retracted and is kept visible |
| VOID | A run invalidated by its own integrity checks; kept with its receipt |

Component lifecycles are machine-readable in
[`research_status_registry.json`](research_status_registry.json) and
[`status_manifest.json`](status_manifest.json).

## Going deeper

- [Full evidence page](status/evidence.html): every matched-baseline table, the component lifecycle,
  literature positioning and the evidence boundaries (it is also on the public site).
- [`VERIFICATION.md`](../VERIFICATION.md): the verification ledger, with every gate, its authority and
  its history (Korean).
- [`results/MANIFEST.json`](../results/MANIFEST.json): the index of every result directory, including
  VOID, FAIL and INCONCLUSIVE ones. It records provenance only and interprets no research quantity.
- [`HISTORY.md`](HISTORY.md): when each result was obtained, and the negative record.
