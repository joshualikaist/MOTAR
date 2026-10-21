# Q02-A v2.1 — repair of the v2 re-review findings (R1–R9)

Date: 2026-10-22 KST. Contract `Q02A-v2.1` (v2 = 87cced6). The findings are in
[the v2 re-review](../q02a_v2_fable_rereview_2026-10-21/REVIEW.md) (PASS_WITH_DOCUMENTED_LIMITATIONS, no P0/P1).
Only `motar_eval/q02/{contracts,geometry,validation,evidence}.py` and `tests/test_q02_evidence.py`
changed. Production detector, perception and task code, the Q01 raw and the Q02-B WIP are unchanged.
CPU only.

## Causes and fixes (line numbers at the v2.1 commit)

| ID | Cause | Fix | File:Line |
|---|---|---|---|
| R1 | The local test took second differences *inside* a linear segment, so a jump lying exactly on a segment boundary was invisible. A paste outline was therefore broken wherever an edge ran along a segment-boundary row or column. 4-connected labelling split the ring, and per-piece bounding boxes missed a self-consistent interior. A paste spanning two opposite borders has no closed outline at all. | (1) A pair-window projection test: along every image column and row, each window of two adjacent segments must lie in the span of the ≤ 3 native columns it depends on. Consecutive windows share a native value, so a line passes all windows iff it is a 1-D upsample. Rows ∈ range(A_h) and columns ∈ range(A_v) give y = A_v X A_hᵀ, so the test is complete for the separable operator. The offending line's whole window is flagged, closing the outline. (2) Flags are grouped on a 1-px-dilated mask with 8-connectivity. (3) A component whose box spans opposite borders sets `PASTE_SPANS_OPPOSITE_BORDERS`. (4) The residual pass iterates to convergence (≤ 8), otherwise `NOT_CONVERGED`. | `geometry.py:162-203` (segments, pair windows, test), `:133-159` (dilate, 8-conn), `:231-236`, `:266-272` |
| R1 / R5 | `observed` was computed before pixels touching unobserved cells were dropped. A singular normal matrix silently fell back to `lstsq`. The Woodbury `_solve_without` was dead code. | Observed cells reach a fixed point after the drop. A Cholesky diagonal-ratio check (< 1e-5) marks `ILL_CONDITIONED`, and the grid is all NaN and not reported. The dead code is deleted; the pure-upsample path is the separable normal-equation solve. | `geometry.py:238-245`, `:254-264`, `:222` |
| R1 | `report.depth_native` was handed out whatever the recovery quality. | Recovery returns `NativeDepth(native, excluded, foreign, residual, status)`. A new `obstacle_depth` check is UNAVAILABLE for `PASTED_DOMINATES` (> 0.5), `PASTE_SPANS_OPPOSITE_BORDERS` or `ILL_CONDITIONED`. `NOT_CONVERGED` makes `depth_structure` FRAME_MISMATCH. `depth_native` is `None` unless `obstacle_depth` is OK. | `validation.py:122-149`, `:197-200`; `geometry.py:105-115` |
| R2 | The extent test pooled every hit of the 360° scan within ±0.3 m of the foreground range. | Only hits on rays within ±1 row and ±2 columns of a foreground ray count; LiDAR azimuth wraps, and camera native rays use the same rule on the 24×40 grid. | `evidence.py:139-175`, params `:53-54` |
| R3 | Identity support was bearing-only and accepted obstacle surfaces. | Pasted pixels count as support only if their depth lies inside that pixel ray's envelope interval [entry − kσ_r − tol, exit + kσ_r + tol]. The flag with pasted pixels in the footprint but none in range → IDENTITY_AMBIGUOUS `RANGE_CONTRADICTED`. A camera-native or LiDAR surface at range is reported as `SURFACE_AT_TRACK_RANGE_NOT_IDENTITY_EVIDENCE`. A flag with no pasted pixel anywhere is noted `NO_PASTED_PIXELS_IN_IMAGE`. "Pasted" now means `foreign`: pixels flagged by the test that the recovered grid does not reproduce, not the grown exclusion box. | `evidence.py:198-209`, `:234-249`; `geometry.py:274-276` |
| R4 | The pasted-fraction limit was part of `depth_structure`, so it made the camera flag unusable. | It moves to `obstacle_depth`. The camera flag needs only `camera_usable`; the obstacle ray test needs `obstacle_depth_usable`. Without a flag, UNKNOWN `CAMERA_OBSTACLE_DEPTH_UNAVAILABLE`. | `validation.py:52-55`, `:139-149`; `evidence.py:178-180`, `:234`, `:256-257` |
| R6 | The v2 REPAIR.md stated native spacing 2.2°×2.4°. | Computed from the contract camera (hfov 87°, vfov 58°): fx_n = 21.08, fy_n = 21.65, centre spacing **2.72° × 2.65°**. The re-review's 2.53° vertical figure also differs from this computation. | this file |
| R7 | One INVALID LiDAR sample invalidates the scan; NaN is mapped to 12 m upstream; 0.2 m in float32 is VALID_HIT. | Documented, behaviour kept (fail-closed). | — |
| R8 | The 0.05 m camera near clip has no production counterpart; the bar-height citation was wrong. | Marked DECLARED; the citation is corrected: the bar pool is 2 m (`env_object_config.py:796-797`), and `bars_h3` (pinned by Q01 v2) is 3 m (`:813-818`). | `contracts.py:95`, `:99-100` |
| R9 | No adversarial recovery tests; F4 assertion weak. | `test_F4` asserts the actual outcome; 7 new tests (below). | `tests/test_q02_evidence.py:378-383`, `:434-532` |

Correction to the v2 record, which is not edited: the masked solve in 87cced6 was an explicit
normal-equation build, not a Woodbury downdate (R5).

## Before / after on identical inputs

Source: [`compare_v2_v21.json`](compare_v2_v21.json), produced by [`compare_v2_v21.py`](compare_v2_v21.py),
which extracts v2 from 87cced6.

| Case | v2 | v2.1 |
|---|---|---|
| The 13 v1→v2 counterexamples (F1–F6, visible target) | — | unchanged hypothesis and identity status |
| R2: F5 far case + bar 90° left at 10.3 m | OCCLUDED | UNKNOWN `FOREGROUND_COMPATIBLE_WITH_TARGET_SURFACE` |
| R2: F5 far case + bar behind the vehicle at 10.3 m | OCCLUDED | UNKNOWN (same) |
| R2: F5 far case, real bar on the line of sight | OCCLUDED | OCCLUDED `LIDAR_EXTENDED_FOREGROUND` |
| R3: pasted 3 m, belief 8 m, flag | TARGET_VISIBLE SUPPORTED | IDENTITY_AMBIGUOUS `RANGE_CONTRADICTED` |
| R3: no target, bar r0.15 at 8 m, flag | TARGET_VISIBLE SUPPORTED | IDENTITY_AMBIGUOUS CONTRADICTED |
| R3: no target, bar r0.4 at 8 m, flag | TARGET_VISIBLE SUPPORTED | TARGET_VISIBLE NOT_TESTABLE |
| R4: flagged target at 0.35 / 0.40 / 0.45 m | UNKNOWN | TARGET_VISIBLE SUPPORTED (obstacle depth UNAVAILABLE) |
| R4: 0.30 / 0.50 / 0.60 m | SUPPORTED | SUPPORTED |
| R1: constant disc r40 / r60 paste, no flag | UNKNOWN (r40: phantom cells inside an OK frame) | UNKNOWN `CAMERA_OBSTACLE_DEPTH_UNAVAILABLE` |
| Q01-V2 frames 0–2, recorded flag | TARGET_VISIBLE SUPPORTED | TARGET_VISIBLE SUPPORTED (14–65 pasted pixels at track range) |
| Q01-V2 frames 0–2, flag cleared | IDENTITY_AMBIGUOUS | IDENTITY_AMBIGUOUS (pasted pixels without detection) |

On the real Q01 frames (read-only), recovery reports status OK with residual 8.6e-7 / 1.1e-6 / 1.2e-6.
Native cells at the 10 m fill number 935 / 937 / 919 of 960, and 16 / 16 / 25 cells are unobserved. Because the
window test flags wider bands than the v2 stencil, slightly more cells beside the paste are unobserved.

## Recovery robustness

Source: [`fuzz_results.json`](fuzz_results.json), produced by [`fuzz_recovery.py`](fuzz_recovery.py).
The pastes are discs, rectangles, rings and disc pairs, at constant or ramped depth, over random,
empty and smooth native grids.

| Run | Cases | Silent leaks | OK | PASTED_DOMINATES | SPANS_BORDERS | ILL_CONDITIONED | Median / max ms (1 BLAS thread) |
|---|---|---|---|---|---|---|---|
| seed 7 | 1000 | 0 | 843 | 135 | 20 | 2 | 30.3 / 98.6 |
| seed 101 | 3000 | 0 | 2534 | 404 | 55 | 7 | 30.4 / 97.3 |
| Sphere sweep across the top/bottom border, 0.5–2.0 m | 400 | 0 usable frames with a kept paste pixel | — | — | — | — | 0 OCCLUDED |

A "silent leak" is an OK frame within the pasted limit with any observed native cell off by
more than 1e-3 m. The v2 property test (`test_R1_random_constant_pastes_are_excluded_or_reported`,
40 cases) found 4 such leaks with the 87cced6-style test plus 8-connected labelling. All four came from paste edges
on segment boundaries, which motivated the window test. The re-review's 46-shape adversarial
set (×4 native grids) has no OK frame with a phantom cell. The only pastes left unflagged equal the
underlying upsample within 1e-3 m and so carry no wrong value.

## Tests

`tests/test_q02_evidence.py`: **37 OK**.
- 30 existing tests: 2 adapted to the API (`NativeDepth`, contract version) and 1 strengthened (F4).
- 7 new tests in `ReReviewR1toR4Tests`:
  - R1 disc exclusion
  - R1 border spanning
  - R1 random property test (40 cases)
  - R1 segment-aligned edges with injected NOT_CONVERGED/ILL_CONDITIONED mapping
  - R2 unrelated objects
  - R3 range-checked support
  - R4 close range

The Q01 real-frame test ran (raw present) and left mtimes unchanged.

PUBLIC_CPU (`tools/run_tests.py`): 3102 ran, 0 failures. The same 4 pre-existing `DetectReduction`
errors remain (motar-6nn.5), so the aggregate status is FAIL, as before this change.

## Remaining limits

- **Hard ceiling of the decomposition.** A paste equal to the obstacle upsample within 1e-3 m is
  indistinguishable from the render. It carries no wrong value, but the pixels count as obstacle depth.
- **Interior of a large smooth paste.** The interior is excluded but not flagged `foreign`. A footprint lying wholly
  inside such a paste reads no pasted pixels, so the result is NOT_TESTABLE or UNKNOWN, never SUPPORTED, OCCLUDED or DETECTION_MISS.
- **Close range.**
  - ≤ 0.45 m (straight ahead): the obstacle depth is UNAVAILABLE.
  - ≤ 0.25 m: the camera is on or inside the 0.15 m sphere, and the depth is INVALID.
  - Close range is out of scope for occlusion evidence.
- **Native sparsity (R6).** 2.72° × 2.65° native spacing. Bars thinner than one native ray gap
  at range give UNKNOWN (sensor disagreement or censored), not OCCLUDED.
- **Extent test (R2).** It still accepts an adjacent separate object at the same range (±0.3 m) as
  extension of the foreground surface.
- **Not calibrated.** No labels; the comparison is counterexamples and smoke only. The camera flag is oracle-grade.
- **LiDAR (R7).** A single INVALID sample drops the whole scan to camera-only, with a reason.

## Q02-B WIP interface changes (v2 → v2.1; WIP not modified)

These changes come on top of the v2 list in [the v2 record](../q02a_v2_repair_2026-10-21/REPAIR.md#q02-b-wip-interface-changes-required-wip-not-modified).

| Area | Change |
|---|---|
| Validation | New check `obstacle_depth` (OK / UNAVAILABLE / INVALID) and `ValidationReport.obstacle_depth_usable`. "Camera usable" (flag credible) and "obstacle rays usable" are now separate. |
| Depth arrays | `depth_native` is `None` unless `obstacle_depth` is OK; only then may it feed a local map. `depth_pasted` is now the `foreign` mask, not the grown box; the box is `depth_excluded`. |
| Geometry | `recover_native_depth` returns `NativeDepth` (with `status`), not a 3-tuple. |
| Record | New field `pasted_pixels_at_track_range`. New identity status `RANGE_CONTRADICTED`. New reasons: `PASTED_DEPTH_AT_TRACK_RANGE`, `CAMERA_FLAG_PASTED_DEPTH_OUTSIDE_TRACK_RANGE`, `SURFACE_AT_TRACK_RANGE_NOT_IDENTITY_EVIDENCE`, `NO_PASTED_PIXELS_IN_IMAGE`, `CAMERA_OBSTACLE_DEPTH_UNAVAILABLE`. Acquisition should require `identity_status == SUPPORTED_BY_FOOTPRINT`, not just `!= CONTRADICTED`. |
| Params | `extent_neighbour_rows/cols` (declared priors). |
