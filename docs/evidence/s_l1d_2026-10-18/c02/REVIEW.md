# SL1D-C02 independent review and the final S-L1D decision — 2026-10-18

## Independent review

A read-only Claude subagent on `claude-fable-5-1` (Fable 5.1), prompted neutrally, recommended **GO
with no blocker**. It checked and reproduced the following from raw data:

- **Timing:** the amendment and tools were committed before the run. Commits at 14:10 and 14:13;
  raw files at 14:16:25–14:16:42.
- **Unchanged inputs:** the v1 plan, gates, reference and earlier tools are unchanged.
- **Independence:** the scene design and the analysis use only NumPy.
- **Reproduction:** the analysis re-run is byte-identical.
- **Requirements A, B and C:** met on 13 unique undetected cases, with the oracle agreeing in all
  13.
  - Scale: 2 cases at alone 0, 2 at alone 1, and 1 at alone 1 occluded.
  - Partial: 4 cases.
  - Centre-only: 4 cases. The exact centre ray and the nearest-pixel proxy agree in every one.
- **Non-vacuity:**
  - In the alone passes, the production segmenter *detects* the same 2-pixel targets. Detection is
    lost only because occlusion leaves 1 pixel.
  - The occluder gaps are 6.1–6.7 m, far larger than τ = 0.052 m.
- **Production:** identical to `76f0f4c`, with worktree import origins.
- **Process independence:** `s_l1c` and `s_l1d` renders and probes are identical.
- **Records:** the report rows match the JSON (0 mismatches); the raw manifest checks 23/23.

## Notes recorded (not blockers)

1. **C02-UNIQUE coverage.** The C02-UNIQUE code compares depth and seven probe keys, not LiDAR. The
   reviewer checked all render keys from raw. Across the attitude passes, everything is identical
   (including `lidar_m`, the detection mask and the observation) except the recorded roll/pitch
   quaternion `robot_orientation`, which the yaw-only camera/LiDAR frame does not use. Read
   "bitwise identical" in the report with that exception.
2. **C02-MAPPING tightening.** Forcing PRED_SCALE_LOSS for an undetected case with alone below the
   minimum is a *tightening* of the analysis gate. The v1 `required()` left an undetected VISIBLE
   case unconstrained. The tightening was committed before the run and matches the unchanged
   production precedence.
3. **Scope of the GO.**
   - Every online result was obtained with the belief primed at the exact true centre and
     covariance 1e−6·I. The 2-pixel targets were at 12–13 m, with sphere-boundary margins of
     3.4–14.7 mm.
   - The evidence shows the classifier acts correctly on real GPU depth. It does **not** show that
     the PARTIAL/CENTRE distinction survives realistic belief error; C01's closed-loop confusion
     shows that drift matters.
   - Fixture counts are constructed, not population rates.
4. **C02-INTEGRITY coverage.** The analysis checks only status, flags and production identity.
   Checkpoint, origins, frozen tree and NaN come from the runner preflight and `validate_done`, and
   the reviewer checked them manually.
5. **Minor points.**
   - The design grid hx/hy straddles the realised extents (0.231–0.352 and 0.274–0.386).
   - `preservation_after.json` carries the inherited label `AFTER_S_L1A`.

## Decision

**S-L1D: GO (under amendment 1).**

- **SL1D-C01:** its eight v1 gates are retained.
- **CPU fixtures:** the missing ones were completed in `bd65d55`; that remains a separate review
  record.
- **SL1D-C02:** it realised the three missing undetected states on GPU, and every C02 gate passed.
- **Regression:** SL1D-C01 covers the closed-loop and lower-lineage regressions on the identical
  production source.

The GO does not claim real-robot sensor accuracy, robustness to belief error or Recovery
performance.
