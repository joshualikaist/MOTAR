# S-L1A v1 — obstacle depth and conservative target-like carve-out

Preregistered before S-L1A implementation or GPU measurement, 2026-10-18. Execution authority is
the user's "Sensor Contract 정정 확인 및 S-L1A 착수" instruction. This new opt-in environment
lineage does not modify any earlier preregistration. The [JSON contract](s_l1a_v1.json) governs
fixtures, numerical gates and run limits. Sensor requirements remain [RGB-D + LiDAR](../SENSOR_CONTRACT.md).

## Calculation and intervention

Bilinear interpolation of first-hit depths is not ray casting: at an occlusion boundary its
weighted depth usually lies on neither surface, and a thin obstacle absent from the coarse grid
cannot be recovered by interpolation. Prediction: exact camera-pixel ray queries remove obstacle
depth interpolation errors, with more query cost and memory. The legacy target sphere renderer's
float32 error is inherited and must be reported separately, not silently repaired.

The historical carve-out accepts range differences below 0.55 m and bearings within 15 degrees.
It can therefore erase the S14 bar about 0.53 m in front of the estimated target surface. A purely
LiDAR-associated track can also confirm the same bar it subsequently removes. Prediction: requiring
fresh camera pixel support, limiting angular support to the modeled target extent and refusing
nearer returns preserves these foreground returns. It may retain more genuine target returns as
obstacles because camera/LiDAR radii, extrinsics, range estimation and identity remain uncorrected.

Approved runtime scope only:

- `navrl_detector.py`: when `NAVRL_SENSOR_LINEAGE=s_l1a_v1`, query obstacle first-hit slant range
  at the actual camera pixel rays into an additional buffer. Keep the legacy native 40×24 query
  and RGB appearance calculations intact. Substitute corrected obstacle depth before the existing
  target/distractor depth overlays. OFF allocates/queries nothing extra and follows the old path.
- `navrl_perception.py`: gate only obstacle-map/governor target-like exclusion. Require current
  camera-pixel support and use the minimum finite positive depth on that perception mask as the
  carve anchor; narrow bearing support to `min(15 deg, asin(r/(anchor+r)))`. Retain every return
  nearer than that anchor (zero inward tolerance); keep the existing upper 0.55 m agreement bound.
  Invalid/nonfinite/nonpositive measurements and no-return values cannot authorize removal.
  Camera blanking uses only valid current candidate pixels within the same anchor window.
  Preserve projection/fusion, target measurement estimation, association, tracker and covariance.
- A small sensor-only helper may validate the switch and implement that mask. No evaluator GT,
  semantic ID, privileged target position or provenance is accepted by the online helper.

ON is limited to fresh measurements: reject nonzero detection latency or reconstructed/backfilled
target masks instead of treating them as fresh camera confirmation. This is a declared applicability
limit, not a change to the 5 s tracker contract. B/C/D, P-L1, G-L1, PPO and Recovery remain excluded.

## Sequence and evidence

1. Commit this preregistration and the documentation-change audit.
2. Implement and independently test the float64 CPU ray/box/sphere oracle, without production
   sensor imports. Commit it before the runtime correction.
3. Implement the two runtime corrections; test real production branches on CPU against the oracle
   and historical source behavior. Commit code, launcher, analysis and tests before GPU use.
4. Run the bounded canary at a clean fixed source SHA, with E-L0 and independent pixel/beam labels.
   No tracked changes, including WORKLOG, during execution. Preserve any FAIL; no automatic retry.
5. Verify manifests/preservation, record raw paths and numerical results in a separate local commit.

The GPU canary compares legacy, depth-only and full S-L1A on the unchanged EQ-1D D2 scenes, including
S14, with extra boundary/two-depth/thin-obstacle fixtures specified in JSON. It measures carve-out
OFF versus legacy versus corrected exclusion on identical sensor inputs, camera/LiDAR target
obstacle retention and E-L0 identity labels. Report populations/denominators; no population-rate
or algorithm-performance claim follows from constructed scenes.

Closed-loop static and E2_direct_l2_v1 moving cases compare unset versus explicit OFF, and ON.
Compare OFF with the preserved E-L0 historical digest logs where the configuration matches.
OFF equality covers observations, deterministic actions, trajectories, outcomes/resets, RMS,
sensors and tracker. ON changes depth/map inputs and can change behavior: this is Frozen F
environment robustness, not a new algorithm improvement. E-L0 must remain non-interfering on ON.

## Publication and scope limits

The existing conditional push authorization applies only to `codex/mt-p1`, after reviewing the entire
unpublished history for secrets, user WIP, raw/checkpoints, symlinks, local-path exposure and the
actual remote target. Refresh the audit at the final HEAD; stop publication on an unresolved failed
check. No main push/merge, force-push, deletion or history rewrite.

No S-L1A PASS resolves Camera–LiDAR pinhole/SE(3), false identity association, radius conventions,
range-only covariance, silhouette validity, dynamics or hardware realism. Report these Known FAILs.
Session reset time is not observable; use a conservative four-hour wrap-up from this unit's start,
reserve validation/commits/handoff time, and start no run that cannot finish within that reserve.
