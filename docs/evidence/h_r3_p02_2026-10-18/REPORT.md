# H-R3 P02 — NO-GO: full-N matching integrity failed

**Decision: NO-GO for confirmatory execution; H-R3 PROVISIONAL — NOT CONFIRMED.**
All twelve GPU processes completed, but the preregistered all-arm common-prefix condition failed. The numerical outcomes below are INVALID for causal/matched-effect inference. No mismatched event is removed to salvage an effect. No further GPU process, extra seed, confirmation run, production correction or training is started.

## B2/B3/B4/B5 outcomes — invalid nominal comparison, transparency only

| Seed | Geometric candidate n | B2 | B3 | B4 sham side | B5 nominal budget | B3−B2 pp | B3−B4 pp | B3−B5 pp |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 4601 | 31 | 17/31 | 18/31 | 22/31 | 15/31 | +3.23 | -12.90 | +9.68 |
| 4602 | 25 | 17/25 | 19/25 | 21/25 | 17/25 | +8.00 | -8.00 | +8.00 |

Equal-seed nominal differences: B3-B2 +5.61pp, B3-B4 -10.45pp, B3-B5 +8.84pp. These are not valid matched causal estimates. B4 numerically exceeds B3 in both seeds, but failed matching prevents an efficacy conclusion. P01 +36.4pp and its McNemar p are not reused as confirmation. Two seed clusters cannot support population inference.

## Integrity and failure evidence

- Main source HEAD: `d64cf84185018ab720040f266a4a481929cfa62b`. Implementation/smoke source: `1ca0c8a`; source hash map equals passing SMOKE02.
- Main processes: 12/12 complete, 2359.00 process wall-seconds (0.655h); peak 5402MiB.
- Reference first-episode triggers: 246; all-arm matched: 203; integrity reasons: 53 including non-trigger rows and t0 mismatches.
- Geometric candidates: 56; 14 have failed prefixes. All 56 have spatially confirmed immediate pre-loss detection in B0; this diagnostic does not cure prefix failure.
- SMOKE01 FAIL preserved. Instance-only cached-kinematics and indexed/batched PhysX reset repair allowed N16 SMOKE02 to PASS all8 processes, forced-reset/off-on exact equality, safety and budget gates. N16 does not prove N128 independence.
- Independent [failure localization](main_failure_audit.md) finds first true prebranch mismatches at seed4602 step7/env2 and seed4601 step11/env1, coincident with first differing reset sets. WarpEnv.reset_idx globally rewrites mesh vertices although only selected meshes are refit. This mechanism is consistent with evidence, NOT_PROVEN: vertex/per-modality snapshots are absent. SMOKE02 force40 shared an existing natural reset, a geometry-refresh blindspot. Do not assert full-N independence is solved. No production source is modified.

## Geometry population (zeroes explicit)

| Seed | FULL | CENTRE_ONLY | PARTIAL | FOV_EXIT (separate) | VISIBLE diagnostic |
|---|---:|---:|---:|---:|---:|
| 4601 | 25 | 6 | 0 | 92 | 4 |
| 4602 | 19 | 6 | 0 | 89 | 5 |

NONE/RANGE_EXIT/SCALE_LOSS each zero. No PARTIAL-class effect is claimed. Event counts do not become valid causal denominators after failure.

## Spatial association vs P01 metric

Actual `_detect_rgbd` mask and reported centroid are checked offline against visible target support: overlap≥1 pixel, detector-mask dominance≥0.5, centroid within Chebyshev1+1e−5 pixel of visible target pixel center, delivered camera flag, terminal-first. Sensitivities0.25/0.75 are diagnostic. This is spatial identity agreement, not proof of persistent tracker-ID continuity or real-camera realism. GT remains evaluator-only; controller consumes only own state, permitted tracker belief and RGB-D+LiDAR.

| Arm | Spatial primary | Literal P01 | Terminal-corrected P01 | Crash in W | Capture in W |
|---|---:|---:|---:|---:|---:|
| B0 | 46/56 | 48/56 | 46/56 | 3 | 1 |
| B1 | 21/56 | 21/56 | 21/56 | 1 | 0 |
| B2 | 34/56 | 34/56 | 34/56 | 1 | 0 |
| B3 | 37/56 | 37/56 | 37/56 | 1 | 0 |
| B4 | 43/56 | 43/56 | 43/56 | 1 | 0 |
| B5 | 32/56 | 32/56 | 32/56 | 1 | 0 |

Counts remain invalid for causal comparison. Only B0 changes under terminal correction (post-reset camera cannot establish a successful preterminal reobservation). All arms use a fixed20-control-step exposure unless terminal, unlike P01 early handback on detection.

## Movement/speed and safety diagnostics

B4 preserves informed lateral magnitude/activation and randomizes SIDE only. B5 matches same-state nominal speed norm, not realized effort; safety attenuation is never reversed. No pure viewpoint-effect claim is permitted.

| Contrast | Translation paired relative difference | Command xy²dt | Actual xy²dt |
|---|---:|---:|---:|
| B3-B2 | 15.22% | 69.62% | 24.78% |
| B3-B4 | 9.30% | 18.91% | 14.33% |
| B3-B5 | 12.04% | 26.79% | 18.89% |

Definition abs(a−b)/((abs(a)+abs(b))/2), control-only window, mean across candidates. Zero denominators are not imputed. The prereg10% diagnostic is not satisfied jointly. These are descriptive diagnostics on an invalid cohort, not validated causal balancing. Squared-speed integrals are kinematic proxies; mechanical energy is NOT_RECORDED.

| Arm | Translation3D m | Mean command xy m/s | Mean actual xy m/s | Point attenuation | Governor scale | Min raw LiDAR m (event minima mean) |
|---|---:|---:|---:|---:|---:|---:|
| B0 | 5.478 | 2.989 | 2.796 | 0.00% | 0.918 | 0.717 |
| B1 | 2.269 | 0.836 | 1.119 | 16.42% | 1.000 | 1.087 |
| B2 | 2.317 | 0.839 | 1.141 | 16.08% | 1.000 | 1.083 |
| B3 | 2.682 | 1.211 | 1.331 | 7.80% | 1.000 | 1.086 |
| B4 | 2.729 | 1.187 | 1.358 | 12.67% | 1.000 | 0.996 |
| B5 | 2.795 | 1.137 | 1.389 | 18.24% | 1.000 | 0.972 |

Full filter-stage vectors, nominal budgets, raw/governor clearances, contacts, terminal outcomes and W+3s outcomes are archived in trace and analysis.json. None is a safety guarantee.

## Confirmation planning — HOLD

`confirmation_planning.json` is an assumption grid, not an execution recommendation after this FAIL. Its 32-new-seed/640-event example assumes a meaningful15pp effect, discordance≤0.4, ICC≤0.1 and≥20 events/seed. Those assumptions are not validated by two seeds or this invalid cohort. Do not use the nominal effects to power a confirmation. Larger grid settings need more seeds. First fix and independently qualify full-N common-prefix/noninterference, then review control/effort design and derive defensible margins. A fresh preregistration and user review must precede any additional exploratory or confirmatory run.

Future analysis design: paired scenes, independent seed clusters, equal-seed effects, Holm correction for B3−B2/B4/B5; adequate-cluster uncertainty and registered joint-contrast power simulation. No event-only McNemar confirmation, no reliable ICC from2 clusters. P01/P02 are excluded from confirmation.

## Reproduction and preservation

Preregistration commit4083c6c; main source `d64cf84185018ab720040f266a4a481929cfa62b`; source plan/analysis SHA256 are in analysis.json provenance. Raw run path: `../aerial_gym_simulator/results/h_r3_p02_v1/HR3-P02`. The adjacent run_report.json and RAW_MANIFEST.json are exact compact copies. Payload stays outside Git; symlinks/merge do not add it. Existing external backup remains NOT_ESTABLISHED.
Preservation PASS: 5954 original tracked files except append-only WORKLOG unchanged; 26 historical manifests/5835 members and all three new run manifests verified; main HEAD/WIP3/stash and both FrozenF checkpoint bindings unchanged.
CPU evidence: 69 focused H-R3 tests, E-L0 14 and EQ-1D13 passed (96 distinct tests); these did not detect the N128 coupling. No blanket regression or unauthorized GPU was added.

Analysis replay (fresh output only; expected exit2 because integrityFAIL):

```bash
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/h_r3_p02_analysis.py --run ../aerial_gym_simulator/results/h_r3_p02_v1/HR3-P02 --out /tmp/HR3-P02-analysis-fresh.json
```

Original GPU invocation is bound in run_report.json/source commit; it is not rerun. Independent Fable5.1 review follows this result commit. Main merge/push, force-push/history rewrite, data deletion and production/Planner/PPO work remain forbidden.
