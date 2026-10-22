# Q02-A v2.5 — identity evidence from the declared target sphere (v2.4 review F1, F2)

Date: 2026-10-22 KST. Contract `Q02A-v2.5` (v2.4 = 8eea494). The findings are in
[the v2.4 confirmation review](../q02a_v24_fable_confirmation_review_2026-10-22/REVIEW.md)
(HOLD_P0_P1_REMAINS: F1 at P1, F2 at P2, no P0). Only `motar_eval/q02/{contracts,geometry,evidence}.py`
and `tests/test_q02_evidence.py` changed:
- `geometry.py` gains one function, and the depth recovery is unchanged;
- `validation.py` is unchanged.

Production code, the Q01 raw and the Q02-B WIP are unchanged. CPU only.

## Why a different kind of fix

C1 (v2.3), E1 (v2.4) and now F1/F2 are one failure class: obstacle pixels inside the paste's box, whose native
cells are unobserved, look like pasted pixels and decide identity. Each pixel-count rule (both-direction flags,
`at > near`, and the reviewer's S1 `at > near and at > far`) closed the routes tested so far and left others
(S1: 46 false and 10 lost on 17,803 cases). The reviewer named the root fix: do not let box pixels count as
target evidence.

## Fix (line numbers at the v2.5 commit)

| Part | What | File:Line |
|---|---|---|
| Paste model | The production target is an analytic sphere of the declared radius (`CameraParams.target_radius` = 0.15 m), pasted with its exact ray depth wherever it is the nearest surface (`navrl_detector.py:111-132`). On the three Q01-V2 frames, the target pixels lie on such a sphere along our pixel rays with residual ≤ 1e-4 m. This is a declared sensor-profile property, not ground truth. | — |
| `fit_target_sphere` | Seeds a sphere centre from each candidate pixel (non-upsample `depth_pasted` pixels; up to 64 seeds), refines it by Gauss-Newton on \|p − c\| = R, and counts front-facing inliers within 5 mm. Each sphere is scored as inliers minus the candidate pixels inside its disc core that show content *farther* than the sphere: only one target exists and nothing pasted can be visible behind it. Obstacle-render pixels are not candidates, so an occluder thinner than the native grid does not count against the true sphere. The target mask = the pixels whose ray hits the best sphere and whose depth equals its surface within 5 mm. | `geometry.py:323-385` |
| Identity rule | With the camera flag and testable belief: SUPPORTED_BY_FOOTPRINT iff ≥ 1 target-sphere pixel lies on an envelope ray inside the range interval. If target-sphere pixels are in the footprint but none at range → RANGE_CONTRADICTED (`TARGET_SPHERE_NEARER/FARTHER_THAN_TRACK`). If there is paste but no consistent sphere, the result is noted `PASTE_NOT_EXPLAINED_BY_TARGET_SPHERE` and falls to the existing CONTRADICTED / NOT_TESTABLE checks. The no-flag path is unchanged. | `evidence.py:214-226`, `:253-272` |
| Record | New fields `target_sphere_found`, `target_pixels_in_footprint`, `target_pixels_at_track_range`. `pasted_pixels_*` stay as the non-upsample counts for transparency; they no longer decide identity. | `evidence.py:106-108` |
| Tests | Version string; two reason names; **new `ReviewF1Tests`**. That test uses a box-bar scene helper (`box_scene`), since the cylinder test scene has no top corners, and reproduces the reviewer's diagnostic case: v2.4 gives SUPPORTED at all three track positions, v2.5 gives RANGE_CONTRADICTED; the correct track is SUPPORTED. | `tests/test_q02_evidence.py:614-662` |

## Before / after

Identity attack: 22 production-shaped families, run against the real module. Families W1–W6 and L1–L4 come from
[`../q02a_v24_repair_2026-10-22/identity_attack.py`](../q02a_v24_repair_2026-10-22/identity_attack.py). Families
M1–M7, L5, L6 and RND come from the v2.4 reviewer's
[`identity_attack_v24_families.py`](identity_attack_v24_families.py). Summary:
[`identity_families_summary.json`](identity_families_summary.json), produced by
[`summarize_families.py`](summarize_families.py). Full outputs stay outside Git.

| Rule (same 17,803 flagged cases, reviewer's `rule_eval.py` for the first three) | False SUPPORTED | Correct tracks lost |
|---|---|---|
| v2.4 `at > near` | 261 | 0 |
| S1 `at > near and at > far` | 46 | 10 |
| S2 `at > near + far` | 46 | 16 |
| **v2.5 sphere** (real module, 18,258 cases incl. unflagged rows of W/L) | **0** | **24** (L5 18, M1 4, M3 2) |

Per family (v2.5): 0 false in every family, including:
- M1 near corner (v2.4: 121);
- M1b far corner (89);
- M4 diagonal (48);
- random scenes RND_0/1 (3).

The 3 m `bars_h3` corner geometry is not among the 22 families. The reviewer's separate probe
(`../q02a_v24_fable_confirmation_review_2026-10-22/probes/m1_z3.py`) gives **2 / 492** false at v2.5
(v2.4: 146). Source: [`bars_h3_corner_v25.json`](bars_h3_corner_v25.json).

Both are boundary coincidences, not box pixels deciding identity:
- a bar-edge pixel whose upsampled blend value lies within 5 mm of the sphere surface (5.850 m and 9.807 m) joins the target mask;
- the wrong track is 0.90–0.94 m too near, and its interval edge falls 1–4 cm short of the true target pixels.

Rejected alternative: a 1 mm tolerance keeps the same 2 cases and raises the correct-track losses from 24 to 57
([`rejected_eps1mm_families_summary.json`](rejected_eps1mm_families_summary.json),
[`rejected_eps1mm_bars_h3_corner.json`](rejected_eps1mm_bars_h3_corner.json)).

Wrong-track fail-safe losses total 61, mostly M1.

The 24 correct-track losses all share one mechanism: a front bar's corner inside the silhouette leaves only
1–2 target pixels among the candidates, and a 2-inlier sphere on the bar corner outscores them. Each case
ends RANGE_CONTRADICTED or NOT_TESTABLE, never SUPPORTED: the gate fails safe.

| Check | Result | Source |
|---|---|---|
| v2.4 → v2.5 comparison (68 cases: all earlier counterexamples, R/D/C/E cases, F1 cases, Q01 frames) | 3 changed: the F1 tracks at the bar face, SUPPORTED → RANGE_CONTRADICTED. All others are identical; the Q01 frames are SUPPORTED with 4/12/14 target-sphere pixels at range. | [`compare_v24_v25.json`](compare_v24_v25.json) |
| Close-range map (310 flagged frames) | 0 leaks; 289/13/8 | reviewer `close_map.py` re-run |
| Recovery fuzz | Recovery code is unchanged, so the v2.3 results apply (0/5,500) | [v2.3 `fuzz_results.json`](../q02a_v23_repair_2026-10-22/fuzz_results.json) |
| Full `estimate` on a Q01 frame | ~50 ms median (1 BLAS thread; includes recovery ~30 ms and the sphere fit). Not a latency guarantee. | measured this session |

**Tests:** 43 OK.

**PUBLIC_CPU:** 3108 ran, 0 failures. The same 4 pre-existing `DetectReduction` errors remain (motar-6nn.5), so the aggregate status is FAIL, as before.

## Remaining limits (current, v2.5)

- **Identity gate.** False SUPPORTED: 0 in 18,258 production-shaped cases and 2/492 in the adversarial 3 m
  bar-corner probe, all at the interval boundary. This is empirical, not proven. The
  gate relies on the declared profile: one 0.15 m sphere target, no distractors, exact pasted depth. A
  box-shaped target (another target-dynamics profile) finds no consistent sphere, so identity is never SUPPORTED there.
- **Corner-occluded correct tracks.** 24 of the 22 families' correct tracks lose support (fail-safe).
- **`depth_pasted`** is still the non-upsample mask, including box obstacle pixels; identity no longer reads it directly.
- **Unchanged since v2.3.** All recovery limits still hold (decomposition ceiling, contact range,
  close-range bands, extent test, finite bars, native sparsity, R7 LiDAR fail-closed scan); see
  [the v2.3 record](../q02a_v23_repair_2026-10-22/REPAIR.md#remaining-limits-current-v23).
- **E4.** The PUBLIC_CPU test creating a git-ignored flock file is out of scope.

## Q02-B WIP interface changes (v2.4 → v2.5; WIP not modified)

| Area | Change |
|---|---|
| Record | New fields `target_sphere_found`, `target_pixels_in_footprint`, `target_pixels_at_track_range`. New reasons `TARGET_SPHERE_AT_TRACK_RANGE`, `CAMERA_FLAG_TARGET_OUTSIDE_TRACK_RANGE`, `TARGET_SPHERE_NEARER_THAN_TRACK`, `TARGET_SPHERE_FARTHER_THAN_TRACK`, `PASTE_NOT_EXPLAINED_BY_TARGET_SPHERE`. Removed (v2.4 only): `PASTED_DEPTH_AT_TRACK_RANGE`, `PASTED_DEPTH_MOSTLY_NEARER_THAN_TRACK`, `CAMERA_FLAG_PASTED_DEPTH_OUTSIDE_TRACK_RANGE`. |
| Acquisition gate | `identity_status == SUPPORTED_BY_FOOTPRINT` with the camera flag, under the profile conditions above. |
| Geometry | New `geometry.fit_target_sphere(depth, candidates, camera, radius)`. |
