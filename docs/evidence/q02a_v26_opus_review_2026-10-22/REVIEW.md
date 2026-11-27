# Q02-A v2.6 independent review (Opus 5.5) — commit 3fb9050

Date: 2026-10-22 KST. Reviewer: Opus 5.5 subagent with a fresh context. Read-only, CPU only, 1 BLAS thread.

Scope: the delta 5fc64a3 → 3fb9050 (contract Q02A-v2.6, record schema Q02A-RECORD-3), which repairs G1/G2 of
[the v2.5 final review](../q02a_v25_opus_final_review_2026-10-22/REVIEW.md). The review also covered the unchanged renderer
contracts (`navrl_detector.py`, `navrl_task.py:1465-1477`, `navrl_task_config.py:206`), `validation.py`, and the
Q01 config and run attestation. The reviewer's probes are in [`probes/`](probes/); they read the worktree and write only
to their scratch directory. A separate model name alone does not establish independence, so the reviewer re-derived every claim from code and runs.

## Verdict: PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS

No P0 and no P1. Every committed number the reviewer re-ran reproduces exactly:
- the corner Z2 / Z3 sweeps;
- random seeds 4 / 5, with 3,476 cross-checks against `estimate`;
- the box seed 9 sweep;
- `compare_v25_v26.json`, byte-identical.

Across 22,500 production-shaped scenes from the reviewer's own families and seeds, a wrong track never got SUPPORTED, with
the track placed at the accepted sphere's centre (worst case):

| Family | Material false / scenes |
|---|---|
| Random seeds 1, 2, 3, 6, 7, 8 | 0 / 9,000 |
| Far 9–19 m | 0 / 4,500 |
| Gap | 0 / 4,500 |
| Clutter | 0 / 4,500 |

The adversarial corner families reproduce the record: 74 and 15 material false tracks, in 4 + 4 scenes.

## Findings

| ID | Sev | File:Line (3fb9050) | Problem | Evidence | Minimal fix |
|---|---|---|---|---|---|
| H1 | P2 (fail-open, adversarial corner families only; record inaccuracy) | `geometry.py:419` (ties broken by fewer farther pixels); `:430` (a rival must be > 2R away); REPAIR §2, §5 | Equal-inlier ties are broken by the farther-pixel count, so "ranking = geometric consistency only" does not hold for ties. Disjoint spheres 0.15–0.30 m apart are never COMPETING. A blend sphere can therefore win against a fully visible true sphere with the same inlier count. | `probe_residual.py`, `probe_classify.py` on the 8 committed residual scenes: 2 × tie with separation in (R, 2R] (8 vs 8 inliers; 6 vs 6); 1 × seeding miss (H2); 2 × mixed fits within 6 cm; only 3 × truly information-limited (2 px at 10 m). Counterfactual (`probe_fixtie.py`, rival = any distinct sphere > R away with disjoint inliers): corner Z2 material false 74 → 41 tracks (4 → 2 scenes) at +34 rejections (+0.3 pp); corner Z3 and random seed 5 unchanged. | Rival test as above, or COMPETING on any disjoint equal-inlier tie. Correct REPAIR §2/§5: 3 of the 8 residual scenes are rule or search artefacts, not information limits. |
| H2 | P2 (fail-safe; record inaccuracy) | `geometry.py:395-403` (seed p + R·ray, 3 × 3 Gauss-Newton steps); REPAIR §5, §7 | Seeding and convergence misses (former G3) are now the main cause of correct-track rejection under `min_inliers = 3`. They also produced the corner seeding-miss residual: the true sphere (5 inliers) was not found, and a 3-inlier blend sphere was accepted. | `probe_weak.py`, random seed 4: 47 TOO_FEW_INLIERS + 10 NO_CONSISTENT_SPHERE scenes, of which 56 / 57 have ≥ 3 candidates exactly on the true sphere. `probe_far.py`: an empty scene at 12 m with 4 exact pixels fits only 2 inliers. Far grid 12–18 m: 30 / 144 TOO_FEW scenes had ≥ 3 exact pixels, so "1–2 px pastes" explains ~79 % of them. | Closed-form 3/4-point seeds or lateral seeds; correct the loss attribution. |
| H3 | NOTE | `evidence.py:110-112` | `target_sphere_found` and `target_pixels_*` describe the accepted sphere, not true target pixels. `identity_status` keeps its legacy name. | — | Document them as accepted-sphere quantities; rename at the next schema change. |
| H4 | NOTE | `test_B_declared_geometry…` | The test checks that the mesh hook exists, not that Q01 leaves `NAVRL_DYNAMIC_MESH_TREATMENT` off (it is off: `tools/boia_shadow.py:137-140` strips NAVRL_* variables, and neither the config nor `campaign_start.json` contains the key). `target_geometry` is not tied to the run attestation. | grep of the config and `campaign_start.json` | Assert the key is absent; the Q02-B loader takes the geometry from `campaign_start.json` `runtime_env`. |

## Answers

1. **G1:** removed for the filed mechanism.
   - Only the most consistent sphere can support. A contradicted or tied best sphere gives AMBIGUOUS, and a lower-ranked candidate never inherits support.
   - Contradictions are the best sphere's own farther pixels, counted only when they lie on a distinct target-like sphere.
   - The G1 fixture is fixed. The corner residual is 8 / 1,243 scenes (H1, H2, mixed fits, information limit).
2. **G2:** safe.
   - A box under a SPHERE declaration gives 0 false support (seed 9 reproduced; reviewer seed 0, 175 scenes, 0 false).
   - A declared ORIENTED_BOX / MESH gives NOT_TESTABLE; UNKNOWN or any other value also gives NOT_TESTABLE.
   - No ground-truth leak: the shape is a static profile field. The renderer and the Q01 config confirm SPHERE for Q01.
3. **Footprint vs identity:** separated in code, record fields and REPAIR §3. `identity_verdict` is never confirmed. Residual misreadings are TARGET_VISIBLE with NOT_TESTABLE geometry, the legacy names (H3), and WIP `history.py:112`.
4. **F1/F2/F3:** intact. `validation.py` and the no-flag branch are unchanged. The 10 m fill, side-bar and zero-value fixtures behave as before.
5. **New wrong-track bypass:** none in production-shaped scenes (table above).
6. **Rejection increase:**

   | Family | v2.5 → v2.6 |
   |---|---|
   | Corner 2 m bar | 1.8 → 13.3 % |
   | Corner 3 m bar | 1.3 → 9.0 % |
   | Random seeds 4 / 5 | 0.8 / 0.7 → 3.47 / 3.54 % |
   | Committed families (losses) | 24 → 337 |

   - In random scenes the losses come mostly from the fitter (H2).
   - Support works as a *necessary* condition up to about 12 m: ~96.5 % of correct tracks in random scenes, 87–91 % at corners.
   - It degrades at long range: empty scenes are 91 % supported at 12 m, 44 % at 14 m and 0 % from 16 m.
7. **Q02-B conditions:** see below.

## Permitted scope for Q02-B (non-control CPU development only; never online intervention)

**Preconditions:**
- `target_geometry == SPHERE`, verified against the run's `runtime_env` (`NAVRL_TARGET_DYNAMICS` ≠ physical, no `NAVRL_DYNAMIC_MESH_TREATMENT`);
- `distractors == 0`;
- `detector_provenance == ORACLE_TRUE_POSE_RASTER`, stated in every output;
- `schema_version == Q02A-RECORD-3`, contract Q02A-v2.6.

**Permitted uses:**
- `hypothesis` for OCCLUDED / DETECTION_MISS / OUT_OF_FOV / UNKNOWN on the no-flag path.
- `geometry_support == FOOTPRINT_CONSISTENT` with `identity_verdict == CONSISTENT_NOT_CONFIRMED`, as a *necessary* condition for a footprint-consistent detection only, carrying the limits: residual corner false support, 3.5–13 % fail-safe loss, and mostly no support beyond ~13 m.
- `identity_verdict` ∈ {AMBIGUOUS, CONTRADICTED, NOT_TESTABLE} and `sphere_fit_status` as abstain or suspect reason codes.
- `tracker_association` as an unverified upstream label.
- `depth_native` under the previously stated conditions.

**Forbidden:**
- treating `identity_status` / SUPPORTED_BY_FOOTPRINT or `hypothesis == TARGET_VISIBLE` as same-target confirmation, or as a sufficient acquisition gate (fix WIP `history.py:112`, `shadow.py:177`);
- reading `target_pixels_*` as true-target counts;
- claiming image-detector identity performance;
- using the records under physical, box, mesh or distractor profiles;
- making result claims before H1/H2 are corrected or carried as limits;
- any GPU, simulator, control or policy use.

## Corrections to the v2.6 record (REPAIR.md, not edited)

- **§2:** ranking is by inliers, with ties broken by the farther-pixel count; a rival must be more than 2R away (H1).
- **§5 / §7:** of the 8 residual corner scenes, 3 are information-limited (2 px at 10 m); 2 are equal-inlier ties, 1 is a seeding miss, and 2 are mixed fits within 6 cm.
- **§5 / §7:** correct-track losses in random scenes come mostly from seeding and convergence misses (H2), not from 1–2 px pastes; in the far grid "1–2 px" holds for ~79 %.

## Disposition

The Q02-A identity repair passes with explicit identity limitations. H1/H2 (P2) are not fixed in this run, per the
instruction to make only the minimal G1/G2 repair and report new findings. Proposed minimal follow-up, for a separately
approved step:
- H1: rival = any distinct sphere > R away with disjoint inliers;
- H2: closed-form multi-point seeds;
- H4: assert the mesh key is absent and tie `target_geometry` to `runtime_env`.

The Q02-B hold remains under the user's decision. The resumption of non-control CPU development is now permissible
within the scope above, in a separate step.
