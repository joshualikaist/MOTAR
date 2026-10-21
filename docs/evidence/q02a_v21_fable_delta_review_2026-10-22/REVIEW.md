# Q02-A v2.1 delta review (Fable 5.1) — commit aab9b31

Date: 2026-10-22 KST. Reviewer: Fable 5.1 subagent, read-only, CPU only
(`CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 OMP/OPENBLAS_NUM_THREADS=1 …/aerialgym/bin/python -B`).
Scope: the delta 87cced6 → aab9b31 (contract Q02A-v2.1), which repairs R1–R9 of
[the v2 re-review](../q02a_v2_fable_rereview_2026-10-21/REVIEW.md). The reviewer's probe scripts are in
[`probes/`](probes/). They target the aab9b31 API, read the worktree at a hard-coded path, and write
only the JSON path given on their command line.

## Verdict: HOLD_P0_P1_REMAINS

No P0. Hypothesis outputs for production-shaped frames (sphere paste over bar render, ≥ 0.305 m)
are correct in every sweep. The F1–F7 outcomes are unchanged, and `compare_v2_v21.py` reproduces the
committed JSON byte for byte. Two P1 items remain.

## Findings

| ID | Sev | File:Line (aab9b31) | Problem | Reviewer evidence | Proposed minimal fix |
|---|---|---|---|---|---|
| D1 | P1 | `validation.py:142-147`, `geometry.py:225-227` | A production-faithful sphere at 0.265–0.30 m (lateral ≤ 0.03, z ≤ 0.05) is a smooth bowl whose curvature is below tolerance. It passes as a (near-)pure upsample, so `obstacle_depth` is OK while `depth_native` holds 700–959 of 960 phantom cells at 0.02–0.2 m. The REPAIR.md claim "≤ 0.45 m straight ahead: UNAVAILABLE" is false. | `close_map.py`: 16 leaks, e.g. `r=0.28 lat=0 -> ds OK od OK phantom=956 native_min=0.032`; `r=0.30 z=0.05 -> od OK phantom 710`. | `obstacle_depth` UNAVAILABLE (CONTACT_RANGE) when any observed native cell is < 0.10 m. Leak minima are ≤ 0.051 m; production frames with bars ≥ 0.6 m away have minima ≥ 0.41 m. |
| D2 | P1 | `geometry.py:276-277`, `evidence.py:198-209` | `foreign` includes 10 m fill pixels whose cells are unobserved (at 8 m, 72 = 8 paste + 64 fill; Q01 frame 0, 44 = 4 paste + 40 fill). The definition does not say so. When the track interval contains 10 m, fill pixels count as "pasted depth at track range", giving SUPPORTED with zero target pixels. This regresses the F1 semantics inside R3. | `reach.py`: target 10.0 m, track 0.5 m lateral → `SUPPORTED at=10` (10 fill, 0 paste); 0.6 m → `at=8`; at 6 m the same error gives RANGE_CONTRADICTED from 15 fill pixels. | Exclude no-return-valued pixels from `foreign`/`at`. |
| D3 | P2 | `geometry.py:190-203, 230-237`; REPAIR.md | The claim "only pastes equal to the upsample within 1e-3 m are unflagged" is false. Smooth pastes with curvature below tolerance pass with error equal to their amplitude (up to ~1 m at L = 80–100 px, `od OK`). The constant-offset ceiling is 2.8e-3, not 1e-3. A complement topology (constant paste except a 2 px hole) gives `od OK` with native = paste. Reach is adversarial only; a false OCCLUDED needs LiDAR invalid + track 9.9 m + a 1 m bump. | `r1_leak.py`, `bump_ceiling.py`, `reach.py`. | Reword ("within ~1e-3 of some piecewise-bilinear surface at native resolution"); optionally add smooth/hole fuzz families or a larger-scale curvature check. |
| D4 | P2 | `geometry.py:274`, `validation.py:133` | All pixels excluded → residual inf → FRAME_MISMATCH → a flagged frame becomes UNKNOWN at 0.31–0.33 m (fail-safe). | `close_map.py`. | `residual = 0.0` when nothing is kept. |
| D5 | P2 | `evidence.py:160-167` | ±2 beams is not range-aware. At 10.3 m an unrelated bar 0.9 m (5°) or 1.8 m (10°) beside the LOS at the same range turns F5 into OCCLUDED. | `r2_probe.py`. | Metric neighbourhood or contiguity. |
| D6 | NOTE | tests F5; REPAIR.md | "Real bar on LOS → OCCLUDED" holds only for the test scene's infinite cylinder. A 2 m/3 m bar at 9.5 m gives one LiDAR row → UNKNOWN (fail-safe); ≤ 6 m gives OCCLUDED. | `r2_probe.py`. | Note; add a finite-height case. |
| D7 | NOTE | `evidence.py:219-249` | The flag path has no belief-age or envelope cap: age 5 s + flag → SUPPORTED, and σ 2 m with a (1, 1) m track error → SUPPORTED. | `r3_probe.py`. | Same gates, or NOT_TESTABLE. |
| D8 | NOTE | REPAIR.md; `test_R1_random` | Depth is INVALID up to 0.275 m (not 0.25). The R1 property test uses constant pastes only. | `close_map.py`. | Wording; ramp/curved values. |

## Status of R1–R9 (reviewer)

- **R1: partially.** Closed for production-shaped pastes ≥ 0.305 m:
  - 0 phantom cells in 438 frames from 0.305 to 19 m;
  - fuzz seed 2024 × 1500: 0 leaks;
  - 46 shapes × 4 grids × 14 value fields: 0 leaks except the hole topology.

  The 1-D window test is numerically complete and sound: 0 false flags and 0 misses in 2000 + 2000 lines; the smallest detectable single-pixel jump is 1.2–2.0e-3. R1 is not closed at contact range (D1) or for smooth sub-tolerance pastes (D3).
- **R2: fixed.** Azimuth wrap verified. A bar at 90° or behind the vehicle gives UNKNOWN. Residual limit: D5.
- **R3: partially.** The interval math is correct: 400/400 correct tracks (0.5–19 m) are SUPPORTED, but fill pixels count as support (D2).
- **R4: partially.**
  - 0.34–0.60 m: correct.
  - 0.31–0.33 m: the flag is lost (D4).
  - 0.265–0.30 m: phantom cells (D1).

  No branch reads obstacle rays when `obstacle_depth_usable` is False.
- **R5:** fixed.
- **R6:** fixed (2.72° × 2.65° reproduced).
- **R7:** documented.
- **R8:** fixed.
- **R9:** partially.

Can a consumer read `depth_native` whenever `obstacle_depth == OK`? Reviewer: "No, not unconditionally". The cause is D1, and D2 also affects `depth_pasted`. With D1 fixed, production-shaped frames ≥ 0.305 m were exact in every observed cell. That exactness is empirical for sphere pastes, not a guarantee for arbitrary smooth content (D3).

Hygiene:
- 37 tests OK.
- The fuzz rerun with seed 2024 × 1500 found 0 silent leaks.
- No torch/simulator/random import.
- The diff touches no production file.
- `git status` was empty before and after; nothing was written outside the reviewer's scratch directory.
- Q01 raw and the Q02-B WIP mtimes are identical.

## Disposition

The fixes are recorded in [the v2.2 repair record](../q02a_v22_repair_2026-10-22/REPAIR.md):
- D1, D2 (the P1s) and D7 are fixed in contract Q02A-v2.2.
- D3, D6 and D8 are corrected in wording.
- D4 and D5 stay as documented fail-safe or narrow limits, with reasons given there.
