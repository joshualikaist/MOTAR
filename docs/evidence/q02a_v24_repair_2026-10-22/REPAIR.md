# Q02-A v2.4 — repair of the v2.3 final-review findings (E1–E4)

Date: 2026-10-22 KST. Contract `Q02A-v2.4` (v2.3 = 17cf5e9). The findings are in
[the v2.3 final review](../q02a_v23_fable_final_review_2026-10-22/REVIEW.md) (HOLD_P0_P1_REMAINS: E1 at P1,
no P0). Only `motar_eval/q02/{contracts,evidence}.py` and `tests/test_q02_evidence.py` changed;
`geometry.py` and `validation.py` are unchanged since v2.3. Production code, the Q01 raw and the
Q02-B WIP are unchanged. CPU only.

## Causes and fixes (line numbers at the v2.4 commit)

| ID | Cause | Fix | File:Line |
|---|---|---|---|
| E1 (P1) | The both-direction rule (v2.3) removes blend pixels *beside* a paste but not obstacle pixels *inside* its tight bounding box. When an obstacle is behind the target, those corner pixels carry the obstacle's depth. A track that is too far (at the obstacle's range) then found "pasted depth at track range" with no target pixel in range. | `_pasted_in_footprint` also counts the pasted pixels inside the envelope that are **nearer** than the track interval. SUPPORTED needs `at > near`; otherwise any pasted pixels in the footprint give RANGE_CONTRADICTED (+ `PASTED_DEPTH_MOSTLY_NEARER_THAN_TRACK`). The rule follows the renderer: the target is pasted only where it is the nearest surface, so pasted content on the track's rays that is mostly nearer than the track means the detection is nearer than the track. New record field: `pasted_pixels_nearer_than_track`. | `evidence.py:208-220`, `:252-261`, `:102` |
| E2 (NOTE) | The v2.3 assertion accepted foreign pixels anywhere within 15 px of the test blob. | `foreign` must lie inside the blob's tight bounding box and cover ≥ 50 % of the blob (measured 117/145, 0 outside the box). New `FinalReviewE1Tests`. | `tests/test_q02_evidence.py:290-294`, `:596-610` |
| E3 (NOTE) | The v2.3 record left the box-corner content unquantified. | Quantified below. | this file |
| E4 (NOTE) | A PUBLIC_CPU test creates the git-ignored 0-byte `aerial_gym/rl_training/rl_games/train_session_logs/.navrl_training.lock` (via `train_navrl_v2_search.sh:450`). | Not changed: it is outside `motar_eval/q02`. The file is only an advisory `flock` target, so leaving it blocks nothing. Reported for a separate fix. | — |

## Before / after

Identity attack: the reviewer's 10 production-shaped scene families. Bars are boxes 2 m/3 m tall seen
from 1 m altitude, or cylinders, with slivers, yaw, elevation and image borders. Sources:
[`identity_attack.py`](identity_attack.py) (the reviewer's script, unchanged except its docstring),
[`summarize_attack.py`](summarize_attack.py) and
[`identity_attack_summary.json`](identity_attack_summary.json). The full per-case outputs stay outside Git.

| Family | Cases | v2.3 false SUPPORTED (reviewer) | v2.4 false SUPPORTED | v2.4 lost SUPPORTED |
|---|---|---|---|---|
| W1 bar behind, wrong range/lateral track | 3,528 | 1,483 | 0 | 297 (fail-safe) |
| W2 front sliver + bar behind | 540 | 89 | 0 | 6 (fail-safe) |
| W3 two bars beside | 1,440 | 20 | 0 | 34 (fail-safe) |
| W4 bar-top corner | 1,620 | 217 | 0 | 0 |
| W5 yaw / bearing / elevation / border | 486 | 210 | 0 | 0 |
| W6 belief σ 0.05 / 0.2 m | 432 | 135 | 0 | 18 (fail-safe) |
| L1–L4 correct tracks (slivers, 10–19 m, borders, 0.45–1.2 m) | 1,436 | 0 false / 0 lost | 0 | **0** |
| **Total** | 9,482 | 2,154 | **0** | 355 wrong tracks, 0 correct tracks |

"Lost" in a W family is a wrong track that still has some target pixels inside its range interval,
but more pasted pixels nearer than it. All 355 are RANGE_CONTRADICTED, at range errors 0.5 m (51),
0.7 m (297), 0.8 m (1) and 1.0 m (6). The reviewer's `fixeval.py` predicted the same counts.

| Check | v2.3 | v2.4 | Source |
|---|---|---|---|
| 64 comparison cases (all earlier counterexamples, R/D/C cases, Q01 frames) + E1 cases | — | 3 changed: E1 "track 1.0 m too far" at 2, 3, 8 m, SUPPORTED → RANGE_CONTRADICTED. Correct tracks and 0.5 m errors are unchanged. | [`compare_v23_v24.json`](compare_v23_v24.json) |
| Q01-V2 frames, recorded flag | SUPPORTED | SUPPORTED (identical) | same |
| Reviewer C1 families A/B, close-range map | 0 false; 289/13/8, 0 leaks | identical | re-run (C1 attack: [`../q02a_v23_repair_2026-10-22/blend_support_v23.txt`](../q02a_v23_repair_2026-10-22/blend_support_v23.txt)) |
| Recovery (fuzz, exclusion) | 0/5,500 silent leaks | geometry unchanged → identical | [v2.3 `fuzz_results.json`](../q02a_v23_repair_2026-10-22/fuzz_results.json) |

**Tests:** 42 OK. 41 are kept: the version string is updated and the E2 assertion strengthened. 1 is new (E1).

**PUBLIC_CPU:** 3107 ran, 0 failures. The same 4 pre-existing `DetectReduction` errors remain (motar-6nn.5), so the aggregate status is FAIL, as before.

## Identity gate after v2.4 (E3 quantified)

`identity_status == SUPPORTED_BY_FOOTPRINT` now requires that the camera flag is set and that the belief is
fresh and narrow (D7). Of the pasted pixels on the envelope rays, more must lie inside the track's range
interval than nearer than it.

Measured on the scene families above: 0 false SUPPORTED in 8,046 wrong-track cases, and 0 lost in
1,436 correct-track cases. A wrong track within ~0.5–1 m of the target in range can lose support
(RANGE_CONTRADICTED), which is fail-safe.

`depth_pasted` still contains obstacle pixels in the paste's tight box. On the test blob that is 24 pixels
against 117 target pixels. These pixels no longer decide identity on their own.

## Remaining limits (current, v2.4)

All limits listed in [the v2.3 record](../q02a_v23_repair_2026-10-22/REPAIR.md#remaining-limits-current-v23) still apply:
- the decomposition ceiling;
- contact range with non-interpenetrating bodies;
- the close-range bands;
- the extent test at ≥ ~9 m;
- finite-height bars;
- native sparsity;
- timing ~30 ms, ≤ ~0.1 s;
- no calibration and an oracle-grade flag;
- the R7 LiDAR fail-closed scan.

v2.4 adds one: the identity gate is a majority rule over pixel counts. It is validated empirically on
the scene families above, not proven; a scene whose pasted box holds more at-range obstacle pixels than
nearer target pixels inside a wrong track's envelope would still pass.

## Q02-B WIP interface changes (v2.3 → v2.4; WIP not modified)

| Area | Change |
|---|---|
| Record | New field `pasted_pixels_nearer_than_track`; new reason `PASTED_DEPTH_MOSTLY_NEARER_THAN_TRACK`. SUPPORTED_BY_FOOTPRINT now implies `pasted_pixels_at_track_range > pasted_pixels_nearer_than_track`. |
| Acquisition gate | `identity_status == SUPPORTED_BY_FOOTPRINT` (with the camera flag) is the gate, under the conditions above. |
