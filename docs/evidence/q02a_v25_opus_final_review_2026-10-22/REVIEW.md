# Q02-A v2.5 final review (Opus 5.5) — commit 5fc64a3

Date: 2026-10-22 KST. Reviewer: Opus 5.5 subagent (the user's instruction of 2026-10-22 replaced Fable for
reviews). Read-only, CPU only, 1 BLAS thread. Scope: the delta 8eea494 → 5fc64a3 (contract Q02A-v2.5), which
replaces the pixel-count identity rules with a fit of the declared target sphere, repairing F1/F2 of
[the v2.4 confirmation review](../q02a_v24_fable_confirmation_review_2026-10-22/REVIEW.md).

The reviewer's probe scripts are in [`probes/`](probes/) (`common.py` plus the scripts named below). They read
the worktree at a hard-coded path and write only their own outputs to the scratch directory.

The author reproduced G1 and G2 independently with [`g1_g2_reproduction.py`](g1_g2_reproduction.py); the
output is in [`g1_g2_reproduction.txt`](g1_g2_reproduction.txt).

This round was declared the final one before it ran (WORKLOG 2026-10-22, v2.5 entry): on P1, report HOLD
without a further iteration.

## Verdict: HOLD_P0_P1_REMAINS

No P0. All re-runnable numbers of the v2.5 record reproduce:
- 43 tests OK;
- the compare JSON is byte-identical;
- the Q01 sphere residual is ≤ 7.2e-5 m, with 4/12/14 target pixels at range;
- the L5/M1/M4 and `m1_z3` re-runs match the record.

But box obstacle pixels still decide identity, now through the sphere *score*, in the bar-corner geometry of F1.
A record claim about box-shaped targets is also false.

## Findings

| ID | Sev | File:Line (5fc64a3) | Problem | Evidence | Proposed fix |
|---|---|---|---|---|---|
| G1 | P1 | `geometry.py:364-374`; `evidence.py:257-266`; v2.5 REPAIR (identity gate, acquisition-gate row) | Obstacle blend pixels in the paste box are candidates. Their values blend toward the 10 m fill, so they read *farther* than the true sphere and veto it (score = inliers − violations). Any 2 candidates within 0.3 m fit a 0.15 m sphere, so a 2-inlier sphere on blend pixels wins. The correct track then gets RANGE_CONTRADICTED, and a wrong track metres away at the blend depth gets SUPPORTED. | **Author reproduction** (the record's own L5 example): the fitted sphere lies 4.45 m from the target, on 2 blend pixels at 7.30 m. The correct track gives RANGE_CONTRADICTED; tracks at 6.3 m (cov 0.04) and 6.6 m (cov 0.01) on the target ray give **SUPPORTED with 0 target pixels**. Reviewer sweeps (`p11_corner_cls.py`, ~700 tracks per scene): material false in 26/636 scenes for the 2 m bar corner and 24/607 for the 3 m `bars_h3` corner, with track errors −2.1…+5.5 m. Random production-shaped scenes (`p6_random.py`, `p12_random_cls.py`): a spurious winner in 1/9,000. | Abstain when the violation term decided the ranking (winner ≠ max-inlier sphere); require ≥ 3 inliers unless every candidate is explained; fix G3 seeding. The reviewer's scratch variants (`fixA.py`, `fixC.py`) reduce but do not close it (corner scenes 26 → 10/6) and double correct-track losses. It needs design work. |
| G2 | P1 (record claim) | v2.5 REPAIR "Remaining limits"; `contracts.py` (no target-shape field) | "A box-shaped target finds no consistent sphere, so identity is never SUPPORTED" is false. The box target profile is `NAVRL_TARGET_DYNAMICS=physical`, half extents 0.14/0.14/0.06 m (`navrl_detector.py:329-332`). | Author reproduction: a box target at 4 m with the correct track gives SUPPORTED, sphere found. Reviewer (`p8_boxtarget.py`, `p8b_boxtarget.py`): spheres found 84/84 and 222/222; false SUPPORTED 1,052 of 155,844 swept tracks. Nothing in the module refuses such input. | Correct the sentence; declare the target shape (sphere, r = 0.15 m) in the profile and return UNVERIFIABLE otherwise, as already done for distractors > 0. |
| G3 | P2 | `geometry.py:356-366` | Seeding (p + R·ray) misses the true sphere for thin limb slivers, giving NO_CONSISTENT_SPHERE. This is a second loss mechanism; the record's single-mechanism statement for the 24 losses is wrong. | `p5_detail.py`. Random scenes: about 0.8 % of flagged scenes lose correct-track support; all 99/99 such tracks end NOT_TESTABLE (`p13_lost_outcome.py`), which is fail-safe. | Closed-form 3-point or lateral seeds; correct the loss description. |
| G4 | P2 | `geometry.py:381-385`; `evidence.py:264` | Blend pixels join the true sphere's mask, and 2–3 px fits pass through blends, giving near-miss false SUPPORTED. | Tracker-like random decisions: 13/174,889 (7.4e-5), all near-misses (≤ 0.4 m lateral or ≤ 9 cm beyond the interval). | Require ≥ 1 candidate inlier among the at-range pixels. |
| G5 | NOTE | `geometry.py:345-347`; v2.5 REPAIR | The docstring says "Obstacle-render pixels are not candidates", but box obstacle pixels are candidates. The thin-occluder outcome holds (`p9_thin.py`: 0/420 losses); the stated reason is wrong. | — | Reword. |
| G6 | NOTE | `geometry.py:369` | At ≥ ~12 m (1–2 px) the fit degenerates: identity reduces to "one candidate pixel at range". | `p4_far.py`: 0 spurious in 249 far scenes. | Document. |
| G7 | NOTE | — | Runtime is fine: fit ≤ 23 ms, `estimate` ≤ 60 ms. | `p3_timing.py` | — |

## Status of the v2.4 findings (reviewer)

- **F1:** the filed route is closed (M1 0/1,836; `m1_z3` 2/492; `ReviewF1Tests` passes), but the class reopens at P1 through G1.
- **F2:** closed as filed (M4 0/630).
- **N1:** fixed.
- **N2:** historical record, left as is.
- **N3:** the lock file remains (out of scope).

## Answers (reviewer)

| Question | Answer | Conditions and evidence |
|---|---|---|
| (a) Read `depth_native` when `obstacle_depth == OK`? | Yes | Unchanged conditions: single 0.15 m sphere, no distractors, non-interpenetrating bodies, skip NaN cells. `validation.py` and the recovery code are unchanged since 8eea494. |
| (b) Use `SUPPORTED_BY_FOOTPRINT` as the acquisition gate? | No, not on its own | Correct tracks are SUPPORTED in ~99 %, and losses are fail-safe. The tracker-like false rate is 7.4e-5 (near-misses only), but metre-scale false SUPPORTED reaches ~4 % of bar-corner-in-silhouette scenes. Use it only paired with an independent range cross-check (LiDAR, or ≥ 3 sphere inliers at range), only with the sphere target and no distractors, and never with physical/box or mesh targets. |
| (c) Lift the downstream CPU-only hold? | Reasonable for design and implementation, with conditions | Build against the v2.5 record fields; treat SUPPORTED as necessary, not sufficient; carry the G1–G4 rates as known limits; make no result claims until a reviewed v2.6 (G1 fix and corrected record), then re-validate Q02-B against it. |

Hygiene:
- Git status was empty before and after; HEAD stayed 5fc64a3, and no git writes were made.
- The worktree, the Q01 raw and the Q02-B WIP are unchanged.
- No GPU, simulator, training or PUBLIC_CPU run.

## Disposition

**Final status of this repair series: HOLD_P0_P1_REMAINS.**

What is established (all reviews agree):
- The P0 (10 m fill read as a surface) and the original P1s (LiDAR line-of-sight window, 0 m values) are fixed.
- The hypothesis outputs (OCCLUDED / DETECTION_MISS / UNKNOWN / IDENTITY_AMBIGUOUS / TARGET_VISIBLE) are correct in every sweep of every round.
- `depth_native` is usable under the stated conditions.

What remains open: the identity status SUPPORTED_BY_FOOTPRINT on the camera-flag path (G1, G2, plus G3/G4 at P2).

Per the declared stop rule, no further iteration is made in this run.

Proposed v2.6 for a future, separately approved step:
- G1: abstain when the violation term decides the ranking; ≥ 3 inliers or all candidates explained; better seeding;
- G2: declare the target shape in `SensorProfile` and return UNVERIFIABLE for other shapes;
- G4: require a candidate inlier among the at-range pixels;
- corrected record text (G2, G3, G5).
