# Q02-A v2.3 final confirmation review (Fable 5.1) — commit 17cf5e9

Date: 2026-10-22 KST. Reviewer: Fable 5.1 subagent, read-only, CPU only (1 BLAS thread).
Scope: the delta 94acd1a → 17cf5e9 (contract Q02A-v2.3), which repairs C1 of
[the v2.2 confirmation review](../q02a_v22_fable_confirmation_review_2026-10-22/REVIEW.md). The
reviewer's probe scripts are in [`probes/`](probes/):
- `attack_c1.py`: 10 production-shaped scene families with box bars 2 m/3 m tall at 1 m altitude, cylinders and slivers;
- `fixeval.py`: the candidate-fix comparison;
- `diag_false.py`.

They read the worktree at a hard-coded path and write only the JSON path given on their command line.

## Verdict: HOLD_P0_P1_REMAINS

No P0. C1 as filed, C3 and C7 are fixed, and every numeric claim of the v2.3 record that the reviewer
re-ran reproduces. The identity gate is still fail-open through a second route of the same class:
obstacle pixels in the corners of the paste's own bounding box, combined with a track range error
toward an obstacle behind the target. This route already existed at v2.2 and was not introduced by the
delta. It is reachable in production-shaped scenes, and the committed sweeps had tested only lateral track errors.

## Findings

| ID | Sev | File:Line (17cf5e9) | Problem | Reviewer evidence | Proposed minimal fix |
|---|---|---|---|---|---|
| E1 | P1 | `geometry.py:241, 289`; `evidence.py:215-216, 248-250` | Background pixels in the paste's tight bbox corners are `foreign` whenever an obstacle nearer than the 10 m fill is behind the target. With the track beyond the target by more than the interval half-width (\|range error\| ≥ ~0.8–0.9 m) and an obstacle 0.5–2 m behind, those pixels alone give SUPPORTED_BY_FOOTPRINT with 0 true target pixels at range. 60/60 sampled cases are also SUPPORTED at 94acd1a (pre-existing). | `attack_c1.py`: 9,482 cases, **2,154 false SUPPORTED**. By \|range error\|: ≤ 0.7 m 0/3,498; 0.8–1.0 m 705/2,010; ≥ 1.1 m 1,449/2,538. Families: W1 bar behind 1,483/3,528, W2 front sliver + bar behind 89/540, W3 two bars beside 20/1,440, W4 bar-top corner 217/1,620, W5 yaw/bearing/elevation 210/486, W6 σ 135/432. Correct tracks: 0 lost in 1,436 (L1 slivers 260, L2 10–19 m 426, L3 borders 130, L4 0.45–1.2 m 324). | Also count pasted pixels in the envelope that are nearer than the interval (`near`), and grant SUPPORTED only if `at > near`, else RANGE_CONTRADICTED. `fixeval.py`: false 2,154 → 0; lost on correct tracks 0; 355 wrong tracks with range error 0.5–0.7 m become RANGE_CONTRADICTED (fail-safe). Rejected: "any nearer pixel → not SUPPORTED" (887 lost), "majority at range" (35 false). |
| E2 | NOTE | `tests/test_q02_evidence.py:289-292` | The re-scoped assertion is weak. 117/145 blob pixels are foreign, and 24 non-blob bbox-corner pixels are foreign; the 15 px bound accepts exactly the E1 pixels. | probe on the test blob | Assert `foreign` is inside the paste's tight bbox and covers ≥ 50 % of the blob; add an E1 test. |
| E3 | NOTE | v2.3 REPAIR.md, `depth_pasted` limit | "Can still include obstacle pixels inside the paste's window-aligned box" is accurate but unquantified; the before/after "0 false" covers lateral errors only. | sweeps above | State the rate and the condition. |
| E4 | NOTE (process) | `tools/run_tests.py` → `train_navrl_v2_search.sh:450` | The PUBLIC_CPU suite creates the git-ignored 0-byte `aerial_gym/rl_training/rl_games/train_session_logs/.navrl_training.lock`. | `stat`, `git check-ignore -v` | Use a temp dir in that test. |

## Status of C1–C8 (reviewer)

- **C1:** fixed as filed (A 0/504, B 64/64, breakdown 0/120, identity sweep identical), but see E1.
- **C2:** documented; reproduces exactly (6/2,768, sphere centre inside a bar).
- **C3:** fixed; contact reported 201/201, 0 frames with native min < 0.10 m lacking CONTACT_RANGE.
- **C4:** corrected.
- **C5:** corrected.
- **C6:** corrected (seed 77: 28.9 / 92.9 ms).
- **C7:** fixed.
- **C8:** closed; PUBLIC_CPU re-run gives 3,106 ran, 0 failures, 4 errors, FAIL as claimed.

## Consumer answers (reviewer)

| Question | Answer | Conditions and evidence |
|---|---|---|
| (a) Read `depth_native` when `obstacle_depth == OK`? | Yes | Single 0.15 m sphere paste class, no distractors, non-interpenetrating bodies; skip NaN. Fuzz 0/1,500 (seed 77), close map 0 leaks. |
| (b) Use `SUPPORTED_BY_FOOTPRINT` as the acquisition gate? | Not yet | Correct tracks are 1,436/1,436 SUPPORTED and lateral errors give 0 false, but range errors ≥ 0.8–0.9 m with an obstacle behind give 35–74 % false SUPPORTED. Usable after the E1 fix (measured 0 false, 0 correct-track loss) or with an independent bound on range error < ~0.8 m. |

Hygiene:
- 41 tests OK.
- The compare JSON is identical.
- Fuzz seed 77 found 0 silent leaks.
- Git status was empty before and after; nothing was written outside the reviewer's scratch directory, except the E4 lock file created by the PUBLIC_CPU suite.

## Disposition

See [the v2.4 repair record](../q02a_v24_repair_2026-10-22/REPAIR.md):
- E1 is fixed with the reviewer's rule C (`at > near`).
- E2 is fixed by strengthening the test.
- E3 is quantified.
- E4 is reported only: production and test code outside `motar_eval/q02` is out of scope.
