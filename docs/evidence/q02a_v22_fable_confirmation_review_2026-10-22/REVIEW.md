# Q02-A v2.2 confirmation review (Fable 5.1) — commit 94acd1a

Date: 2026-10-22 KST. Reviewer: Fable 5.1 subagent, read-only, CPU only (1 BLAS thread).
Scope: the delta aab9b31 → 94acd1a (contract Q02A-v2.2), which repairs D1/D2/D7 of
[the v2.1 delta review](../q02a_v21_fable_delta_review_2026-10-22/REVIEW.md). The reviewer's new
probe scripts are in [`probes/`](probes/). They target the 94acd1a API, read the worktree at a
hard-coded path, and write only the JSON path given on their command line.

## Verdict: HOLD_P0_P1_REMAINS

No P0. D1 (contact range), D2 as filed (fill pixels) and D7 are fixed as claimed. Every numeric claim
of the v2.2 record that the reviewer re-ran reproduces:
- the comparison JSON is byte-identical;
- 40 tests OK;
- the fuzz shows 0 leaks;
- the close-range map is 310 = 289/13/8 with 0 leaks;
- the Q01 frames are unchanged.

One P1 remains. The D2 failure class, SUPPORTED_BY_FOOTPRINT with zero target pixels, is still
reachable in production-shaped scenes through obstacle-blend neighbours. It affects only the
identity status on the flag path; hypothesis outputs are correct in every sweep.

## Findings

| ID | Sev | File:Line (94acd1a) | Problem | Reviewer evidence | Proposed minimal fix |
|---|---|---|---|---|---|
| C1 | P1 | `geometry.py:278-281`, `evidence.py:203-214, 246-248`; REPAIR "foreign neighbours" | Window-flagged obstacle-blend pixels beside a paste (cells unobserved, not fill-valued) count as pasted depth at track range. Example: a 0.15 m sphere seen next to a bar edge, with the track 0.5–0.8 m beside the target (on the bar), gives SUPPORTED_BY_FOOTPRINT with 0 true target pixels in the envelope. This is a fail-open identity gate; the record called it "in principle". | `blend_support.py`: 162/504 cases (R 3–9.5 m, image gap 0.05–0.8 m, bar ΔR 0/±0.4, track error 0.4–1.2 m). `blend_breakdown.py`: 44/120. Correct tracks: 64/64 SUPPORTED. Gaps ≥ 0.32 m respect the planner's 0.77 m clearance, so the family is legal. | Count only foreign pixels flagged by both the row and the column window tests. `fix_direction.py`: family A 0/504 false (was 162), 0 lost true SUPPORTED, B 64/64, no-bar sweep 38/38, Q01 4/12/14 unchanged. |
| C2 | P2 | `validation.py:152-155`, `contracts.py:96` | `obstacle_contact_m = 0.10` has no margin. A bowl whose limb is continuous with a bar render passes with observed min 0.102–0.121 m and phantom cells. Reached only with interpenetrating bodies. | `ext_close_map.py`: 6/2768 leaks, all with the sphere centre 0.11–0.16 m from the bar axis (intersecting). Separated bodies: 0/340. | Document "non-interpenetrating bodies", or raise to 0.15 m (would withhold bars at 0.10–0.13 m). |
| C3 | NOTE | `validation.py:148-155`, `evidence.py:117` | The contact warning was evaluated only for status OK and paste ≤ 0.5. | `ext_cm.json`: r ≤ 0.40 m, 105 UNAVAILABLE non-contact frames without a warning. | Evaluate the contact test before the status branch. |
| C4 | NOTE | REPAIR close-range table | Measured in 1 mm steps, the bands differ: CONTACT 0.295–0.299, PASTED_DOMINATES 0.300–0.308, FRAME_MISMATCH 0.309–0.333, PASTED_DOMINATES ≥ 0.334. | `fine_band.py` | Wording. |
| C5 | NOTE | REPAIR D5 paragraph | At ≥ 9.14 m (0.8/tan 5°) a single max-width bar cannot reach the adjacent beam, so a same-range adjacent hit there is always a second object. In production-shaped frames the paste makes the no-flag path IDENTITY_AMBIGUOUS first; a false OCCLUDED needs an unpasted target. | `d5_check.py` | Wording; limit acceptable. |
| C6 | NOTE | REPAIR timing | "At most 93 ms" is seed- and load-specific (reruns 104.7 / 101.8 ms). | fuzz reruns | "~30 ms median, ≤ ~0.1 s". |
| C7 | NOTE | `evidence.py:296-297` | The camera `NEAR_CLIPPED_IN_LINE_OF_SIGHT` term is unreachable since v2.2. | code | Comment or drop. |
| C8 | NOTE | REPAIR tests | PUBLIC_CPU was not re-run by the reviewer. | — | — |

## Status of D1–D8 (reviewer)

- **D1:** fixed for separated bodies. Extended map 2768 frames: contact fired 46×, with the warning 46/46, and 0 phantom cells in 1617 od-OK no-bar frames. Legitimate frames are withheld only when a surface is < 0.083 m from the lens.
- **D2:** fixed for fill pixels (0 fill-valued pixels in 225 frames), but not closed for blend neighbours (C1). Legitimate pixels lost near 10 m: 7 over 10.0–10.5 m, identity unaffected.
- **D3:** wording corrected; the window test shows 0/2000 false flags and 0/2000 misses.
- **D4:** claim verified (residual 0 would make a noise frame SUPPORTED and break `test_F3`); keeping the limit is acceptable.
- **D5:** conclusion accurate, reasoning imprecise (C5).
- **D6:** wording.
- **D7:** fixed; Q01 frames have age 0, σ_r ≤ 0.19 and SUPPORTED at range 4/12/14 on real target pixels.
- **D8:** wording.

## Consumer question (reviewer)

| Field | Answer | Conditions |
|---|---|---|
| `depth_native` when `obstacle_depth == OK` | Yes, with conditions | Production paste class (a single 0.15 m sphere, no distractors, non-interpenetrating bodies); skip NaN cells. Exactness is empirical. |
| `depth_pasted` as a pasted-pixel mask | No | Non-empty means non-upsample content is present. It is not a segmentation, and not identity evidence until C1 is fixed. |

## Disposition

See [the v2.3 repair record](../q02a_v23_repair_2026-10-22/REPAIR.md):
- C1 is fixed with the reviewer's proposed both-direction rule.
- C3 and C7 are fixed.
- C2, C4, C5 and C6 are corrected in wording or documented.
