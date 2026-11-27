# Q02-A v2.6 — identity semantics repair (Opus final review G1, G2)

Date: 2026-10-22 KST. Contract `Q02A-v2.6`, record schema `Q02A-RECORD-3` (v2.5 = 5fc64a3). Findings:
[the v2.5 final review](../q02a_v25_opus_final_review_2026-10-22/REVIEW.md) (HOLD_P0_P1_REMAINS: G1, G2 at P1).

Scope:
- Changed: `motar_eval/q02/{contracts,geometry,evidence}.py` and `tests/test_q02_evidence.py`.
- Unchanged: `validation.py` and depth recovery; production code, Frozen F, the Q01 raw and the Q02-B WIP (11/11 hashes).
- CPU only. GPU/Isaac NOT_RUN; G5 HOLD.

## 1. Root causes

| ID | Cause |
|---|---|
| G1 | v2.5 ranked candidate spheres by *inliers minus farther pixels in the disc core* and supported from the winner. Obstacle-render blends next to a paste (bilinear mixes toward the 10 m fill) also read "farther". So they subtracted from the **true** sphere's score, and a 2-pixel sphere fitted to blends won (any 2 points within 0.3 m fit a 0.15 m sphere). Two defects followed: ranking and contradiction were one number, and the contradiction itself was not a real one. |
| G2 | The v2.5 record claimed that a box target never passes the sphere model. That was false: nothing declared the target's shape, and a box face has tangent patches that fit a sphere. The rendered shape is a static configuration fact (`NAVRL_TARGET_DYNAMICS=physical` → oriented box 0.14/0.14/0.06 m, `navrl_detector.py:329-332`; D8 mesh via `attach_dynamic_mesh_treatment`, `:1010`), but the module had no field for it. |

## 2. Fix (File:Line at the v2.6 commit)

| Part | What | File:Line |
|---|---|---|
| Separate concepts | `SphereFit` reports geometric consistency (inliers), contradiction (`contradictions`), unexplained farther pixels (`farther_unexplained`), the best rival and same-object band pixels as separate quantities. | `geometry.py:335-347` |
| Ranking ≠ contradiction | Spheres are ranked by inliers only. The most consistent sphere is judged with **its own** contradictions. A contradiction is a farther pixel in its disc core that lies on a *distinct target-like sphere* (≥ 3 inliers): a second target-like surface visible behind it, impossible for the single declared target. Farther pixels on no such sphere (blends) are counted but cannot veto. A contradicted best sphere is **not** replaced by a lower-ranked one: the status is `MOST_CONSISTENT_SPHERE_CONTRADICTED` → AMBIGUOUS. | `geometry.py:359-446` (ranking `:419`, contradiction `:420-429`, status `:436-446`) |
| Rivals / weak / shape | A distinct uncontradicted rival with equal inliers gives `COMPETING_SPHERES` → AMBIGUOUS. Fewer than 3 inliers, unless the sphere explains every candidate near it, gives `TOO_FEW_INLIERS` → not supported. Same-object pixels near it that are mostly off its surface give `SHAPE_INCONSISTENT` → AMBIGUOUS. | same |
| Support rule | SUPPORTED needs ≥ 1 *candidate inlier* (not only interior fill-in pixels) on an envelope ray inside the range interval (G4). | `evidence.py:261-282` |
| Declared target geometry | `SensorProfile.target_geometry` (SPHERE / ORIENTED_BOX / MESH / UNKNOWN), plus `geometry_contract` and `detector_provenance`. For anything other than SPHERE the geometry test is NOT_TESTABLE (`TARGET_GEOMETRY_MODEL_NOT_APPLICABLE`). The pinned Q01-V2 config has `NAVRL_TARGET_DYNAMICS=bounded` (→ sphere); a test checks the renderer text and the Q01 config. The shape is never derived from the hidden target pose. | `contracts.py:24`, `:103-110`; `evidence.py:312-316` |
| Identity contract | New record fields `detection_support`, `detector_provenance`, `geometry_support`, `tracker_association`, `identity_verdict`, `identity_confirmation`, `sphere_fit_status`, `target_geometry`, `schema_version`. `identity_status` is kept as a compatibility alias of the footprint-geometry statement only. | `evidence.py:117-127`, `:135-152` |

## 3. Geometry support vs identity verdict

| Statement | Field | Meaning |
|---|---|---|
| A. A detection exists | `detection_support` | CAMERA_FLAG / NONE. In this profile the flag comes from a true-pose raster (`detector_provenance = ORACLE_TRUE_POSE_RASTER`), not from an image detector. |
| B. It is at the predicted target position | `geometry_support` | FOOTPRINT_CONSISTENT / RANGE_INCONSISTENT / POSITION_INCONSISTENT / AMBIGUOUS / NOT_TESTABLE / NOT_APPLICABLE. This is a geometric footprint test only. |
| C. It is the same target as before | `identity_verdict` | **Never confirmed.** The best value is CONSISTENT_NOT_CONFIRMED; otherwise AMBIGUOUS / CONTRADICTED / NOT_TESTABLE. `identity_confirmation = NOT_AVAILABLE_SINGLE_FRAME_ORACLE_FLAG`. |
| (association) | `tracker_association` | What the upstream tracker reports (`UPSTREAM_MEASUREMENT_UPDATE_NOT_VERIFIED` / `PREDICTION_ONLY` / `NO_TRACK`); not verified here. |
| D. The target is occluded | `hypothesis` (no-flag path) | Unchanged. An uncertain C does not imply D. |

`SUPPORTED_BY_FOOTPRINT` (`identity_status`) = B is FOOTPRINT_CONSISTENT. It never means C.

The Q01 replay's TARGET_VISIBLE / SUPPORTED is a technical observation under the oracle renderer. It is not a
validation of image-based same-target identification.

## 4. Counterexamples before → after (identical inputs)

Source: [`compare_v25_v26.json`](compare_v25_v26.json), produced by [`compare_v25_v26.py`](compare_v25_v26.py), which extracts v2.5 from 5fc64a3. Of 74 cases, 6 changed, all G1/G2:

| Case | v2.5 | v2.6 |
|---|---|---|
| G1 fixture (target 3 m, 2 m bar 0.4 m in front, a=b=−0.7), correct track | RANGE_CONTRADICTED (blend sphere won) | SUPPORTED (15 target pixels); fit OK, 22 blends recorded |
| G1 fixture, tracks at 6.3 / 6.6 / 7.3 m on the target ray | SUPPORTED with 0 target pixels | RANGE_CONTRADICTED |
| G2 box target at (4, 0.2, 0) / (2, 0, 0.1), SPHERE declared | SUPPORTED | AMBIGUOUS |
| All earlier F1–F7, R, D, C, E and F counterexamples | — | unchanged |
| Q01-V2 frames 0–2, recorded flag | SUPPORTED | SUPPORTED: geometry FOOTPRINT_CONSISTENT, verdict CONSISTENT_NOT_CONFIRMED, 4/12/14 inlier pixels at range |

Additional test fixtures:
- A ring sphere at 3 m around a second sphere at 6 m that shows through its core: the higher-ranked sphere has a genuine contradiction → AMBIGUOUS for both tracks.
- Declared ORIENTED_BOX / MESH / UNKNOWN → NOT_TESTABLE.

## 5. False support and rejection, per family (never pooled)

Source: [`sweeps_summary.json`](sweeps_summary.json), produced by [`identity_sweeps.py`](identity_sweeps.py) and
[`summarize_sweeps.py`](summarize_sweeps.py). The sweeps run on identical seeded scenes for v2.5 (extracted from 5fc64a3) and v2.6.

- **Sweep:** about 700 swept tracks per scene (0.4–12 m along the target ray, lateral 0 / 0.2 m, three covariances).
- **TL:** 40 tracker-like decisions per scene (range error N(0, 1 m), lateral N(0, 0.15 m)).
- **False:** SUPPORTED with no true target pixel at range. "Material" = not ≤ 5 cm and not with the true centre inside the track interval.
- **Rejection:** correct tracks (|range error| ≤ 0.3 m, lateral ≤ 0.15 m) with truth that are not SUPPORTED.
- **Cross-check:** every ~300th v2.6 track was checked against the full `estimate` (10,371 checks, 0 mismatches).

| Family | Ver | Scenes | False (material) | AMBIGUOUS tracks | Correct supported / correct | Incorrect rejection | TL false (material) | TL rejection |
|---|---|---|---|---|---|---|---|---|
| Bar corner in silhouette, 2 m bar (adversarial) | v2.5 | 636 | 1,298 (1,199) | 0 | 11,227 / 11,430 | 203 (1.8 %) | 89 (74) | 43 / 2,397 |
| | v2.6 | 636 | **88 (74)** | 44,226 | 9,907 / 11,430 | 1,523 (13.3 %) | 15 (13) | 324 / 2,397 |
| Bar corner, 3 m `bars_h3` (adversarial) | v2.5 | 607 | 993 (854) | 0 | 10,608 / 10,752 | 144 (1.3 %) | 66 (45) | 32 / 2,309 |
| | v2.6 | 607 | **75 (15)** | 9,126 | 9,780 / 10,752 | 972 (9.0 %) | 11 (4) | 207 / 2,309 |
| Random production-shaped, seed 4 | v2.5 | 1,500 | 7 (0) | 0 | 26,754 / 26,970 | 216 (0.8 %) | 3 (1) | 45 / 5,486 |
| | v2.6 | 1,500 | **0 (0)** | 0 | 26,034 / 26,970 | 936 (3.5 %) | 0 | 187 / 5,486 |
| Random production-shaped, seed 5 | v2.5 | 1,500 | 116 (99) | 0 | 26,802 / 26,982 | 180 (0.7 %) | 5 (1) | 39 / 5,575 |
| | v2.6 | 1,500 | **0 (0)** | 1,404 | 26,028 / 26,982 | 954 (3.5 %) | 0 | 207 / 5,575 |
| Box target, SPHERE declared (G2) | v2.5 | 225 | 943 (882) | 0 | 3,839 / 4,050 | 211 | 71 (61) | 56 / 846 |
| | v2.6 | 225 | **0 (0)** | 100,386 | 1,134 / 4,050 | 2,916 (by design: wrong model) | 0 | 604 / 846 |

The box-target scenes split by fit status:
- `SHAPE_INCONSISTENT` in 68;
- `MOST_CONSISTENT_SPHERE_CONTRADICTED` in 74;
- `TOO_FEW_INLIERS` in 19;
- `COMPETING_SPHERES` in 1;
- OK (accepted as a sphere) in 63 of 225, still with 0 false support.

A declared ORIENTED_BOX profile is always NOT_TESTABLE.

The 22 committed families (real `estimate`, 18,258 cases; [`families_summary.json`](families_summary.json)):
- 0 false SUPPORTED, as at v2.5.
- 337 correct-track losses, versus 24 at v2.5: L2 far targets 10–19 m 120 (1–2 px pastes, `TOO_FEW_INLIERS`); L5 corner-occluded 183; L1 slivers 26.
- 124 wrong-track fail-safe losses.

3 m corner probe (`m1_z3.py`): 2 / 492 false, the same two interval-boundary cases as v2.5 ([`bars_h3_corner_v26.json`](bars_h3_corner_v26.json)).

**Residual G1-class (diagnosed):** the corner-family residual false support comes from 8 scenes (4 per bar pool;
[`residual_corner_z2.json`](residual_corner_z2.json), [`residual_corner_z3.json`](residual_corner_z3.json),
[`diag_residual_corner.py`](diag_residual_corner.py)). In every one the visible target is 2–8 pasted pixels behind
a bar corner, and a sphere through blend pixels (sometimes mixed with 1–2 target pixels) ties or beats it with ≥ 3
inliers. The paste does not contain enough target pixels to be explained uniquely: an information limit, not a
sensor/geometry error.

**Rejected alternative:** abstaining when a small paste (< 6 or < 10 inliers) sits next to unexplained blends.
- 2 m bar corner: it would flip 48 / 124 scenes where the true sphere wins, to catch 14 / 19 spurious-or-mixed fits.
- 3 m bar corner: 41 / 108 to catch 9 / 14.
- Random seed 4: 2 / 4 to catch 20 / 23, but random scenes already show 0 false support.

Source: [`rejected_small_paste_rule_cost.txt`](rejected_small_paste_rule_cost.txt).

## 6. Regression

- **Tests:** `tests/test_q02_evidence.py` **49 OK**. The 43 existing tests are kept; only the version string changed, and the G1 fixture assertions were rewritten to the v2.6 semantics. 6 tests are new (Groups A, B, D).
- **Earlier counterexamples:** F1–F7 and the R/D/C/E/F counterexamples are unchanged (§4).
- **Close-range map, recovery, fuzz:** `validation.py` and the depth-recovery code are unchanged, so these v2.5 results apply.
- **PUBLIC_CPU:** 3114 ran, 0 failures, 4 errors. The 4 errors are the same pre-existing `DetectReduction` errors (motar-6nn.5); there is no new error. The aggregate status is FAIL, as before; this is not a full-suite PASS.

## 7. Remaining limits

| Limit | Where it occurs | Effect |
|---|---|---|
| Residual false footprint support | 8 / 1,243 adversarial corner scenes (2–8 visible pixels) | Track errors at metre scale in those scenes. 0 in 3,000 random scenes. |
| Correct-track rejection | Corner families 9–13 %; random 3.5 %; far 1–2 px targets in the committed L2 family | Rejected tracks end AMBIGUOUS / NOT_TESTABLE; this is conservative, not wrong evidence. |
| Interval-boundary near-misses (G4 class) | 2 / 492 in the 3 m corner probe | Pixels lie 1–4 cm beyond the interval. |
| Misdeclared shape (box target under a SPHERE declaration) | 63 / 225 scenes accepted as a sphere | Still 0 false support. A declared box is always NOT_TESTABLE; correct declaration is a configuration responsibility, audited by test text. |
| Identity | All paths | Never confirmed (single frame, oracle flag). Temporal association is upstream and not verified here. |
| Inherited from earlier records | — | All v2.3 recovery limits still apply; see the v2.3 record. |

## 8. Q02-B WIP impact (WIP not modified; read-only grep)

| WIP place | Current use | Risk under v2.6 | Required change |
|---|---|---|---|
| `code/history.py:112` (ever-acquired gate) | `usable = informative and record.hypothesis == TARGET_VISIBLE and not suspect` | **Risk path.** TARGET_VISIBLE is also returned with `identity_verdict` NOT_TESTABLE (stale/diffuse belief, interior-only, too few inliers, declared non-sphere), so "acquired" can be set without footprint consistency. | Require `geometry_support == "FOOTPRINT_CONSISTENT"` and `identity_verdict == "CONSISTENT_NOT_CONFIRMED"`. Record "acquired" as a footprint-consistent detection, not as an identity confirmation. |
| `code/shadow.py:177` (state machine) | `IDENTITY_SUSPECT` only for `hypothesis == IDENTITY_AMBIGUOUS` | `identity_verdict` AMBIGUOUS / NOT_TESTABLE with hypothesis TARGET_VISIBLE is not treated as suspect. | Use `identity_verdict in ("AMBIGUOUS", "CONTRADICTED", "NOT_TESTABLE")`. Add ABSTAIN reason codes for `sphere_fit_status` (CONTRADICTED / COMPETING / SHAPE / TOO_FEW) and `TARGET_GEOMETRY_MODEL_NOT_APPLICABLE`. |
| Track-loss detection, reacquisition condition, timing features | Built on the hypothesis stream | The v2.6 AMBIGUOUS share rises in corner geometry (§5), which raises loss/ambiguity events. | Use the separate fields; re-validate thresholds against v2.6 records. |
| Any code using `identity_status == SUPPORTED_BY_FOOTPRINT` as same-target proof | — | Contract violation under v2.6. | Treat it as necessary, never sufficient. |
