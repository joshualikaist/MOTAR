# Q02-A v2.4 confirmation review (Fable 5.1) — commit 8eea494

Date: 2026-10-22 KST. Reviewer: Fable 5.1 subagent, read-only, CPU only (1 BLAS thread). The run stopped
once on the API session limit and was resumed from its saved state after the reset; the verdict comes
from the resumed run.

Scope: the delta 17cf5e9 → 8eea494 (contract Q02A-v2.4), which repairs E1 of
[the v2.3 final review](../q02a_v23_fable_final_review_2026-10-22/REVIEW.md).

The reviewer's attack families are committed as
[`../q02a_v25_repair_2026-10-22/identity_attack_v24_families.py`](../q02a_v25_repair_2026-10-22/identity_attack_v24_families.py),
unchanged except the docstring. The other probes are in [`probes/`](probes/): rule evaluation, aggregation, the
`bars_h3` corner case, diagnostics, and the E2 blob probe. They read the worktree at a hard-coded path and
write only the JSON path given on their command line.

## Verdict: HOLD_P0_P1_REMAINS

No P0. E1 is fixed as filed, and every re-runnable number of the v2.4 record reproduces exactly:
- the 10-family summary is byte-identical: 0 false / 9,482, 0 lost / 1,436, 355 fail-safe;
- the compare JSON is identical;
- 42 tests OK;
- the close map is 0 leaks with 289/13/8.

The same pixel class still decides identity by a route the committed sweep never exercised. The pixel class
is obstacle pixels inside the paste's tight box, kept in `foreign` because their native cells are unobserved.
v2.4 counted pasted pixels nearer than the track, but not farther. A wrong track nearer than the detection,
on a front bar whose top corner lies inside the target's silhouette, gets SUPPORTED from the bar's own pixels.

## Findings

| ID | Sev | File:Line (8eea494) | Problem | Reviewer evidence | Proposed fix |
|---|---|---|---|---|---|
| F1 | P1 | `evidence.py:220, 255`; v2.4 REPAIR "Identity gate after v2.4", Q02-B interface line | A front bar's top corner inside the paste silhouette puts bar pixels (cells unobserved) in `foreign`. A track at the bar face then has `at` = bar pixels, `near` = 0, and all target pixels farther → SUPPORTED with `true_at = 0`. Edge-only occlusion is safe; the corner is not. This contradicts the record's "0 false in 8,046 wrong-track cases", "these pixels no longer decide identity", and the gate claim. | `M1_near_corner`: **121/1,836** false (errors −1.0…−2.8 m). `bars_h3` corner (`m1_z3.py`): **146/492**. `M1b` 10–15 m targets: **89/384**, errors to −8.7 m. `diag_m1.py` R 4 m, d_f 1.5 m: the `at` pixels are at the bar face 2.56–2.58 m; the 15 target pixels are all farther. Random scenes: 0/105. | S1 `at > near and at > far`: false 261 → 46, correct lost 0 → 10/3,696. Root fix upstream: do not let box pixels with unobserved cells count as target evidence. Add a near-track corner test. |
| F2 | P2 | `evidence.py:255`; `geometry.py:289` | The envelope can contain box-corner obstacle pixels at the track range and no target pixel (cross-LOS error ≈ envelope radius + ~0.2 m, obstacle 0.3–1.5 m behind). S1 does not fix this. | `M4_far_diag`: **48/630**. Random scenes: **3/1,881** (0.16 %). | Document, bound the cross-LOS error, or apply the upstream fix. |
| N1 | NOTE | `tests/test_q02_evidence.py:596-610` | The E1 test covers far tracks only. | reading | Add a near-track corner subtest. |
| N2 | NOTE | v2.4 REPAIR "24 pixels against 117" | 24 is the test path (no `fill_value`); the production path gives 20. | `probe_e2.py` | Say "on the test path". |
| N3 | NOTE | E4 | The 0-byte lock file is still present. | `stat` | As filed. |

## Status of E1–E4 (reviewer)

- **E1:** fixed as filed; F1/F2 re-open the pixel class.
- **E2:** fixed (117/145, 0 outside the box).
- **E3:** quantified for the tested families only.
- **E4:** unchanged, as declared.

## Answers (reviewer)

| Question | Answer | Conditions and evidence |
|---|---|---|
| (a) Read `depth_native` when `obstacle_depth == OK`? | Yes | Unchanged since v2.3: single 0.15 m sphere, no distractors, non-interpenetrating bodies, skip NaN. |
| (b) Use `SUPPORTED_BY_FOOTPRINT` as the acquisition gate? | No, not alone | Correct tracks 3,696/3,696 SUPPORTED; near-wrong corner tracks 7–30 % false (F1); 0.16 % in random scenes (F2). |
| (c) Lift the downstream CPU-only hold? | Reasonable for design and implementation, with conditions | Target the post-F1 contract; do not use v2.4 SUPPORTED to confirm tracks nearer than the detection; carry F2 with its rate; validate Q02-B against a v2.5. |

Hygiene:
- Git status was empty before and after, and no git writes were made.
- Nothing was written outside the reviewer's scratch directory.
- The Q01 raw and the Q02-B WIP were untouched.
- No GPU, simulator, training or PUBLIC_CPU run.

## Disposition

The pixel-count rules have now been patched four times in this class (C1, E1, F1, F2). v2.5 therefore replaces
them with the root fix the reviewer named: identity evidence comes only from pixels of the single
declared-radius sphere that explains the paste. See [the v2.5 repair record](../q02a_v25_repair_2026-10-22/REPAIR.md).
From this point, independent reviews use Opus 5.5 (user instruction, 2026-10-22).
