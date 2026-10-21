# Q02-A v2.3 — repair of the v2.2 confirmation-review findings (C1–C8)

Date: 2026-10-22 KST. Contract `Q02A-v2.3` (v2.2 = 94acd1a). The findings are in
[the v2.2 confirmation review](../q02a_v22_fable_confirmation_review_2026-10-22/REVIEW.md)
(HOLD_P0_P1_REMAINS: C1 at P1, no P0). Only `motar_eval/q02/{contracts,geometry,validation,evidence}.py`
and `tests/test_q02_evidence.py` changed. Production code, the Q01 raw and the Q02-B WIP are unchanged.
CPU only.

## Causes and fixes (line numbers at the v2.3 commit)

| ID | Cause | Fix | File:Line |
|---|---|---|---|
| C1 (P1) | The window test flags a whole two-segment window when any pixel in it deviates. Obstacle-blend pixels on a line through the paste were therefore flagged too. When their native cells were unobserved they were "foreign", and inside a wrong track's envelope they granted identity support with no target pixel. | `foreign` (the pasted-pixel candidates) needs the flag in both directions: `by_col & by_row`, plus late-residual pixels, which are individually inconsistent with fully observed cells. A pasted 2-D blob deviates along its rows and its columns; a blend pixel beside it usually deviates along one only. The union still drives the exclusion, so recovery is unchanged. | `geometry.py:192-211`, `:239-241`, `:285`, `:289`; docstring `:105-117` |
| C3 (NOTE) | The contact check ran only after the dominate/span checks. | The contact check runs first, so CONTACT_RANGE and the warning appear for every recovery status. | `validation.py:148-153` |
| C7 (NOTE) | The camera term of `NEAR_CLIPPED_IN_LINE_OF_SIGHT` is unreachable since v2.2. | Comment; the LiDAR term is live. | `evidence.py:298` |
| C2 (P2) | The 0.10 m contact threshold has no margin against interpenetrating bodies. | Kept. The assumption "bodies do not interpenetrate" is stated in code and below. Raising the threshold to 0.15 m would withhold real bars at 0.10–0.13 m from the lens. | `validation.py:149-152` |
| C4, C5, C6 (NOTE) | Close-range bands, the D5 reasoning and the timing claim were imprecise. | Corrected under "Remaining limits". | this file |

## Before / after on identical inputs

| Check | v2.2 | v2.3 | Source |
|---|---|---|---|
| Reviewer family A: sphere beside a bar, wrong track 0.4–1.2 m (504) | 162 false SUPPORTED (reviewer) | 0 | [`blend_support_v23.txt`](blend_support_v23.txt) |
| Family B: same scenes, correct track (64) | 64 SUPPORTED | 64 SUPPORTED | same |
| Correct tracks: slivers behind front bars, beside/behind bars, z ±0.3 m, yaw 0/0.4, 2–12 m (252) | SUPPORTED 180 | SUPPORTED 180 = cases with true paste at range 180; 0 lost, 0 false | [`identity_support_sweep.json`](identity_support_sweep.json) |
| 55 comparison cases (all earlier counterexamples, R/D cases, Q01 frames) | — | 4 changed: the C1 wrong-track cases at 8 m and 5 m, SUPPORTED → NOT_TESTABLE ×3 or CONTRADICTED ×1; all others identical | [`compare_v22_v23.json`](compare_v22_v23.json) |
| Q01-V2 frames, recorded flag | SUPPORTED, pasted at range 4 / 12 / 14 | identical | same |
| Close-range map, 310 flagged frames | 0 leaks; 289 / 13 / 8 | identical | reviewer `close_map.py` re-run |
| Recovery fuzz (7 ×1000, 101 ×3000, 2024 ×1500) | 0 silent leaks | 0 silent leaks (exclusion unchanged) | [`fuzz_results.json`](fuzz_results.json) |

Test changes:
- `test_native_recovery_is_exact_and_matches_torch_operator` asserted in v2.1 that *every* pixel of
  a test blob is foreign. That blob is constant along columns, so its interior is flagged along rows
  only. The assertion now checks that the paste is fully excluded (unchanged; the safety property),
  that `foreign` is non-empty on it, and that `foreign` stays within 15 px of it.
- New `ConfirmationC1Tests` pins the C1 family.

**41 OK.**

PUBLIC_CPU (`tools/run_tests.py`): 3106 ran, 0 failures. The same 4 pre-existing `DetectReduction` errors remain
(motar-6nn.5), so the aggregate status is FAIL, as before.

## Remaining limits (current, v2.3)

- **`depth_pasted` (`foreign`).** Evidence that non-upsample content is present, with its depth. It is not a
  segmentation: it omits the smooth interior of large pastes and pixels within 1e-3 m of the 10 m fill. It
  can still include obstacle pixels inside the paste's window-aligned box. Identity support requires such
  pixels at the track range inside the envelope.
- **Decomposition ceiling (D3).** Content within ~1e-3 m per window of a piecewise-bilinear surface is read
  as obstacle depth. In production this occurs only at contact range, which is withheld.
- **Contact range (C2).** Threshold 0.10 m, assuming bodies do not interpenetrate. A sphere intersecting a
  bar can pass with cells at 0.10–0.12 m (6/2768 reviewer frames).
- **Close range.** Bands at 1 mm steps, straight ahead (reviewer `fine_band.py`):

  | Range | Behaviour |
  |---|---|
  | ≤ 0.275 m | depth INVALID |
  | 0.295–0.299 m | CONTACT_RANGE |
  | 0.300–0.308 m | PASTED_DOMINATES |
  | 0.309–0.333 m | FRAME_MISMATCH; a flagged frame is UNKNOWN (D4, fail-safe) |
  | ≥ 0.334 m | PASTED_DOMINATES, up to ~0.49 m |

  Out of scope for occlusion evidence.
- **Extent test (D5/C5).** Below ~9.1 m (0.8 m / tan 5°) one wide bar and two adjacent objects
  cannot be told apart by beam adjacency. Beyond that, an adjacent same-range hit is always a second object
  but is still counted. A false OCCLUDED needs an unpasted target (camera miss) with a bar within 2 beams.
- **Real bars (D6).** For a 2 m/3 m bar at ≥ ~9.5 m only one LiDAR ring hits, and the result is UNKNOWN.
- **Native sparsity.** 2.72° × 2.65° spacing; bars thinner than one ray gap give UNKNOWN.
- **Timing (C6).** Recovery ~30 ms median and ≤ ~0.1 s on this host (1 BLAS thread, load-dependent). Not a latency guarantee.
- **Calibration and LiDAR.** Not calibrated; no labels; the camera flag is oracle-grade. One INVALID LiDAR sample drops the scan (R7).

## Q02-B WIP interface changes (v2.2 → v2.3; WIP not modified)

These are additional to [the v2.2 list](../q02a_v22_repair_2026-10-22/REPAIR.md#q02-b-wip-interface-changes-v21--v22-wip-not-modified).

| Area | Change |
|---|---|
| Depth arrays | `depth_pasted` = pixels flagged along both row and column (or by residual), unexplained, not fill. Use it as "non-upsample content present" and through `pasted_pixels_in_footprint` / `pasted_pixels_at_track_range`, never as a target mask. |
| Validation | `obstacle_depth` reports CONTACT_RANGE before PASTED_DOMINATES or a recovery status, so `camera_contact_range` / `near_obstacle_warning` are set at every recovery status. |
| Consumer rule | `depth_native` may be read only when `obstacle_depth` is OK. It is exact for the production paste class (single 0.15 m sphere, no distractors, bodies not interpenetrating); skip NaN cells. |
