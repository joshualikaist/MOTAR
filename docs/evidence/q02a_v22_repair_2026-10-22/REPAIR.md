# Q02-A v2.2 — repair of the v2.1 delta-review findings (D1–D8)

Date: 2026-10-22 KST. Contract `Q02A-v2.2` (v2.1 = aab9b31). The findings are in
[the v2.1 delta review](../q02a_v21_fable_delta_review_2026-10-22/REVIEW.md) (HOLD_P0_P1_REMAINS:
D1, D2 at P1, no P0). Only `motar_eval/q02/{contracts,geometry,validation,evidence}.py` and
`tests/test_q02_evidence.py` changed. Production code, the Q01 raw and the Q02-B WIP are unchanged.
CPU only.

## Causes and fixes (line numbers at the v2.2 commit)

| ID | Cause | Fix | File:Line |
|---|---|---|---|
| D1 (P1) | At 0.265–0.30 m the pasted sphere fills the image as a smooth bowl. Its curvature is below the 1e-3 m window tolerance, so the frame is a valid upsample of some native grid. Nothing distinguishes it from an obstacle render, and `obstacle_depth` was OK with the target surface (0.02–0.05 m) as "obstacle depth". | A declared contact range `obstacle_contact_m = 0.10`: any observed native cell nearer than it makes `obstacle_depth` UNAVAILABLE (`CONTACT_RANGE <min> m`). `depth_native` is then withheld and `near_obstacle_warning` is raised. The camera flag keeps working. | `contracts.py:96`; `validation.py:13`, `:58-61`, `:152-155`; `evidence.py:116-117` |
| D2 (P1) | `foreign` (the "pasted" mask) contained no-return fill pixels whose native cells are unobserved next to a paste (Q01 frame 0: 40 of 44). When the track interval contained 10 m, they counted as pasted depth at track range. | A pixel at the fill value (|y − 10 m| ≤ 1e-3) is never foreign: `recover_native_depth(…, fill_value=)`, which validation passes as the profile far plane. The docstring now states what `foreign` contains. | `geometry.py:105-115`, `:207`, `:280-281`; `validation.py:134` |
| D7 (NOTE) | The flag branch had no belief-age or envelope gate. | The same gates as the ray test (age > 2 s, σ_r > 1.5 m, kσ⊥ > 0.8 m) make the footprint NOT_TESTABLE (TARGET_VISIBLE kept). | `evidence.py:237-240` |
| D3 (P2) | The v2.1 record overstated completeness. | Corrected below ("Recovery: what the window test can and cannot see"). | this file |
| D6, D8 (NOTE) | The v2.1 record's close-range and real-bar wording. | Corrected under "Remaining limits". | this file |

**Not changed, with reasons:**

- **D4 (P2, fail-safe).** A flagged frame at 0.31–0.33 m is UNKNOWN because every pixel is excluded,
  giving residual inf and FRAME_MISMATCH. The proposed fix (residual 0 when nothing is kept) would
  also accept a frame of pure noise as a valid composition, since every pixel is flagged and nothing
  is kept. That would undo `test_F3_foreign_depth_content_fails_closed`. Kept as a documented limit.
- **D5 (P2).** The extent test is not range-aware. At ≥ ~9 m, adjacent 5° beams are ≥ 0.8 m apart,
  which is the maximum bar width, so beam adjacency cannot tell one wide object from two adjacent
  objects. A metric neighbourhood has the same ambiguity at that spacing. Kept as a documented limit,
  quantified by the reviewer: a bar 0.9 m or 1.8 m beside the LOS at 10.3 m turns F5 into OCCLUDED.

## Before / after on identical inputs

Source: [`compare_v21_v22.json`](compare_v21_v22.json), produced by [`compare_v21_v22.py`](compare_v21_v22.py),
which extracts v2.1 from aab9b31.

| Case | v2.1 | v2.2 |
|---|---|---|
| All 13 earlier counterexamples, the R1–R4 cases and the Q01 frames | — | unchanged hypothesis and identity status |
| D1: target 0.28 m, no flag | UNKNOWN; `obstacle_depth` OK with 956 phantom cells | UNKNOWN `CAMERA_OBSTACLE_DEPTH_UNAVAILABLE` (CONTACT_RANGE 0.032 m) |
| D1: target 0.28 / 0.30 m, flag | TARGET_VISIBLE SUPPORTED | TARGET_VISIBLE SUPPORTED, obstacle depth UNAVAILABLE |
| D2: target 10.0 / 9.8 / 9.5 m, track 0.5 m beside, flag | TARGET_VISIBLE SUPPORTED (fill pixels only) | TARGET_VISIBLE NOT_TESTABLE |
| D2: target 6.0 m, track 0.5 m beside, flag | IDENTITY_AMBIGUOUS RANGE_CONTRADICTED (fill pixels only) | IDENTITY_AMBIGUOUS CONTRADICTED (camera clear at the track) |
| D7: flag, belief age 5 s / σ 2 m | TARGET_VISIBLE SUPPORTED | TARGET_VISIBLE NOT_TESTABLE |
| Q01-V2 frames 0–2, recorded flag | SUPPORTED, pasted at range 36 / 65 / 14 | SUPPORTED, pasted at range 4 / 12 / 14 (real target pixels only) |

Close-range map (the reviewer's `close_map.py`, 310 flagged frames, 0.255–0.40 m × lateral 0–0.15 m
× z 0/0.05 m):

| | v2.1 | v2.2 |
|---|---|---|
| `obstacle_depth` OK with phantom cells | 16 | 0 |
| TARGET_VISIBLE SUPPORTED, obstacle depth UNAVAILABLE | — | 289 |
| Depth INVALID (≤ 0.275 m, camera on the sphere) | — | 13 |
| FRAME_MISMATCH (0.31–0.33 m, D4) | — | 8 |

## Recovery: what the window test can and cannot see

The exclusion logic is unchanged since v2.1.
- Source: [`fuzz_results.json`](fuzz_results.json), produced by [`fuzz_recovery.py`](fuzz_recovery.py).
- Fuzz seeds 7 ×1000, 101 ×3000 and 2024 ×1500: 0 silent leaks.
- Sphere sweep across the border: 0/400 usable frames with a kept paste pixel.
- Recovery takes a median of 30 ms and at most 93 ms (1 BLAS thread).

Correction to the v2.1 record (D3): the test does not flag only pastes that equal the
upsample. It accepts any content that, window by window, lies within ~1e-3 m of a piecewise-bilinear
surface at native resolution. A constant offset passes up to ~2.8e-3 m. A smooth bump of low
curvature passes at full amplitude (up to ~1 m at 80–100 px scale), and a constant paste with a
small hole reads as obstacle depth. This is a hard limit of the decomposition, not a tolerance
setting.

In the Frozen F profile the only pastes are the 0.15 m sphere target (distractors 0). Its limb is
steep at every range ≥ 0.305 m, and the delta review found 0 phantom cells in 438 such frames
(0.305–19 m). Below that is the contact range, which v2.2 withholds (D1). Exactness of
`depth_native` is therefore empirical for the production paste class, not guaranteed for arbitrary
content.

## Tests

`tests/test_q02_evidence.py`: **40 OK**.
- The 37 v2.1 tests are kept; the contract version string is updated.
- 3 new tests in `DeltaReviewD1D2Tests`:
  - D1: an 80-frame contact-range grid, with no phantom cell when obstacle depth is OK, the warning raised, and the flag kept.
  - D2: no fill pixel is pasted, and the 0.5 m-beside track is not SUPPORTED.
  - D7: stale or diffuse belief.

PUBLIC_CPU (`tools/run_tests.py`): 3105 ran, 0 failures. The same 4 pre-existing `DetectReduction`
errors remain (motar-6nn.5), so the aggregate status is FAIL, as before.

## Remaining limits (current, v2.2)

- **Decomposition ceiling (D3).** Content that is piecewise-bilinear within ~1e-3 m per window is
  read as obstacle depth. In production this matters only at contact range, which is withheld.
- **Close range.**

  | Range (straight ahead) | Behaviour |
  |---|---|
  | ≤ 0.275 m | Depth INVALID (the camera is on the sphere) |
  | 0.28–0.30 m | CONTACT_RANGE |
  | 0.31–0.33 m | FRAME_MISMATCH; a flagged frame is UNKNOWN (D4, fail-safe) |
  | 0.34–0.49 m | PASTED_DOMINATES |

  Close range is out of scope for occlusion evidence. A real obstacle within 0.10 m of the camera also withholds the grid, with a warning.
- **Extent test (D5).** At ≥ ~9 m an adjacent separate object at the same range (±0.3 m) can make
  the foreground "extended", so a false OCCLUDED is possible when the camera has no paste on the target.
- **Real bars (D6).** "Real bar on the LOS → OCCLUDED" holds for the tall cylinders of the test scene.
  For a 2 m/3 m bar at ≥ ~9.5 m only one LiDAR ring hits, and the result is UNKNOWN (fail-safe).
- **Native sparsity.** 2.72° × 2.65° spacing. Bars thinner than one native ray gap at range give UNKNOWN.
- **`foreign` neighbours.** `foreign` can include obstacle-blend pixels next to a paste whose cells are
  unobserved. These are not fill-valued but can, in principle, fall inside a wrong track interval.
- **Calibration and LiDAR.** Not calibrated; no labels; the camera flag is oracle-grade. One INVALID LiDAR sample drops the scan (R7).

## Q02-B WIP interface changes (v2.1 → v2.2; WIP not modified)

These are additional to [the v2.1 list](../q02a_v21_repair_2026-10-22/REPAIR.md#q02-b-wip-interface-changes-v2--v21-wip-not-modified).

| Area | Change |
|---|---|
| Validation | `obstacle_depth` can be UNAVAILABLE with detail `CONTACT_RANGE <m>`; new property `ValidationReport.camera_contact_range` (a safety warning, already folded into `near_obstacle_warning`). |
| Depth arrays | `depth_pasted` never contains no-return fill pixels. `depth_native` may be read only when `obstacle_depth` is OK. Its exactness is empirical for the production sphere paste, not guaranteed for arbitrary content. |
| Geometry | `recover_native_depth(…, fill_value=None)`; validation passes the far plane. |
| Record | New reason `BELIEF_STALE_OR_DIFFUSE_FOOTPRINT_NOT_TESTABLE`. Acquisition must require `SUPPORTED_BY_FOOTPRINT` (a stale or diffuse belief now gives NOT_TESTABLE). |
| Profile | `SensorProfile.obstacle_contact_m = 0.10` (DECLARED). |
