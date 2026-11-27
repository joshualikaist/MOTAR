# Q02-A v2 repair — sensor semantics (Fable F1–F7)

2026-10-21 KST · `codex/mt-p1` · base `a10be96` · contract `Q02A-v2` · CPU only · GPU NOT_RUN · G5 HOLD.
Review being repaired: [Q02-A Fable review](../q02a_fable_review_2026-10-21/REVIEW.md).
Before/after on identical inputs: [compare_v1_v2.json](compare_v1_v2.json) (script beside it).
Not an accuracy result: no labels exist, and the camera flag is oracle-grade in this profile.

## Production sensor contract (re-derived from source)

| Item | Production | Source |
|---|---|---|
| Obstacle depth native grid | 40×24 pinhole rays from the camera origin, same FOV (native fx ≈ 21.1, cx = 19.5), environment mesh only (target is analytic, never in this render) | `navrl_task_config.py:229-231`; `navrl_detector.py:380-398, 407, 875-886` |
| No return | ray-cast limit = far plane: `dist = far_plane` = 10.0 m (finite, so it blends) | `navrl_detector.py:47-49` |
| Upsampling | `F.interpolate(mode="bilinear", align_corners=False)` to 160×90, `nan_to_num(10)`, `clamp(0, 10)` | `navrl_detector.py:927-937` |
| Composition | distractor pixels pasted, then visible target pixels pasted, each with its own exact depth | `navrl_detector.py:959-974` |
| LiDAR | hit in [0.2, 12) m; no hit → 12 m (`far_out_of_range_value = max_range`); < 0.2 m → `-max_range` → getter clamps to **0.0**; NaN/Inf → 12 m | `navrl_lidar_config.py:71-72`; `warp_sensor.py:233-238`; `navrl_task.py:6013-6017` |
| Confidence | max(camera segmenter support, LiDAR association gate `exp(-|Δr|/gate)`) | `navrl_perception.py:2132-2134, 2396` |

The real Q01 frames are consistent with this contract: native recovery residual ≈ 1e-6, pasted
blobs exactly where the target is, ≈ 947 of 960 native cells at the 10 m no-return.

## Root causes and fixes

| ID | Root cause in v1 | v2 fix (file:line) |
|---|---|---|
| F1 P0 | Depth read as a 20 m full-resolution range image; the 10 m no-return fill and bilinear blends of hits with the fill were counted as surfaces | `geometry.py:83 bilinear_matrix` (exact torch operator), `:114 _solve_without` (Woodbury masked LS), `:163 _nonlinear_pixels` (local test for pasted pixels), `:189 recover_native_depth` (exact native grid, pasted blobs and hidden cells marked); `validation.py:93 classify_depth_native`, `:114 _depth_structure`; `evidence.py:159 _camera_evidence` uses native rays; `contracts.py:81 SensorProfile` declares 40×24 / 10 m |
| F2 P1 | LiDAR beams chosen by an angle window padded by half the beam spacing | `geometry.py:247 ray_envelope`; `evidence.py:131 _ray_counts` — a ray counts only if its line passes within R + kσ⊥ of the predicted centre, from that sensor's own origin; `evidence.py:43 max_envelope_extra_m` caps kσ⊥ at 0.8 m (wider → UNKNOWN) |
| F3 P1 | 0 m depth/LiDAR passed validation and counted as foreground | `validation.py:103 classify_lidar` (0 → NEAR_CLIPPED, (0, 0.2) and NaN/Inf → INVALID), `:140, :154` whole-frame zeros INVALID; `evidence.py:255` near-clipped ray in the line of sight → UNKNOWN; `near_obstacle_warning` reported separately |
| F4 P2 | 81-pixel square window vs an ≈ 8-pixel target | evidence counted over envelope rays only; any foreground in the envelope vetoes "clear" (`evidence.py:273 PARTIAL_FOREGROUND_IN_ENVELOPE`) |
| F5 P2 | A nearer-than-predicted target surface became OCCLUDED | camera-native samples never contain the target; LiDAR foreground must be *extended* beyond the target proxy + 0.15 m (`_ray_counts` extent test) to be an obstacle, else UNKNOWN (`evidence.py:270`); pasted target pixels in the footprint → IDENTITY_AMBIGUOUS; `BELIEF_ERROR_NOT_EXCLUDED` on every tracker-based OCCLUDED/DETECTION_MISS |
| F6 P2 | TARGET_VISIBLE = raw camera flag | reason renamed `CAMERA_FLAG_VISIBLE`; `identity_status` SUPPORTED_BY_FOOTPRINT / CONTRADICTED / NOT_TESTABLE; flag with nothing at the track → IDENTITY_AMBIGUOUS (`evidence.py:215`); declared distractors → IDENTITY_AMBIGUOUS |
| F7 P2 | Confidence labelled segmenter score | `evidence.py:33 CONFIDENCE_SEMANTICS` = max of camera support and LiDAR gate, not a probability |
| F8 NOTE | Far planes absent from the calibration tuple | declared in `SensorProfile` and source-text tested; still not in the Q01 packet (recorder unchanged) |
| F9 NOTE | `fused`/age semantics depend on `NAVRL_LIDAR_SILENT_CORRECT=0` | unchanged; documented here; Q01 pins the default |
| F10 NOTE | `-m unittest tests/...` cannot import | run as `python -B tests/test_q02_evidence.py` or `tools/run_tests.py` |

## Sample semantics

Per sample: VALID_HIT · NO_RETURN (MAX_RANGE_CENSORED) · NEAR_CLIPPED · INVALID/UNOBSERVED.
A camera native sample at 10 m means "no environment surface within 10 m along this native ray";
space beyond 10 m is never observed free (rays whose envelope exit is beyond the far plane are
CENSORED). NEAR_CLIPPED is not a range for occlusion inference and not free space: it makes a
line-of-sight test UNKNOWN and raises `near_obstacle_warning`. An all-zero frame is INVALID; one
invalid sensor degrades the rule (reason recorded) without invalidating the other.

## Line-of-sight test

For a unit ray u from sensor origin o and predicted centre c (vehicle frame), s = u·(c − o),
d⊥ = |(c − o) − s u|. The ray informs about the target iff s > 0 and d⊥ ≤ R_env = R_sensor + kσ⊥,
with R_sensor = 0.15 m (camera) or 0.20 m (LiDAR proxy), k = 2, σ⊥ the largest lateral standard
deviation of the position covariance. Its chord is [s − h, s + h], h = √(R_env² − d⊥²), widened by
kσ_r along range. A VALID_HIT before the chord minus 0.5 m is foreground; inside the chord ± 0.3 m
is consistent; after it, or a NO_RETURN whose far range lies beyond the chord, is clear; a
NO_RETURN whose far range ends inside the chord is censored. Thresholds are declared priors; no
probability is attached to them.

## Tests

`tests/test_q02_evidence.py`: 30 tests (16 v1 tests — 3 updated only for renamed record fields
and the corrected F7 label, scene helper made production-faithful — plus 14 new: production
sentinels from source text, exact native recovery vs torch, F1 fill/far plane/foreground/blend,
F2 envelope/side/behind/adjacent/spread cap/origins, F3 zero/near-clip/foreign depth, F4–F6, Q01
frames read-only).

## Q01 frames (read-only)

| Frame | v1 | v2 |
|---|---|---|
| 0, 1, 2 with recorded flag | TARGET_VISIBLE | TARGET_VISIBLE, identity SUPPORTED_BY_FOOTPRINT |
| 0, 1, 2 with flag cleared | IDENTITY_AMBIGUOUS — from the 10 m fill (**withdrawn**) | IDENTITY_AMBIGUOUS — pasted target pixels in the footprint (the target is visible in these frames) |

## Limits that remain

- No accuracy: no labels; oracle-grade detector; three frames, all with the target visible.
- Sparse sampling: native camera rays are 2.2°×2.4° apart and LiDAR 5°×10°; small envelopes at
  long range fall between rays (UNKNOWN `CAMERA_NATIVE_NO_COVERAGE`), and a thin occluder seen by
  the camera but missed by sparse LiDAR gives UNKNOWN (sensor disagreement), not OCCLUDED.
- Pasted blobs larger than a native cell hide the obstacle depth behind them (UNOBSERVED cells).
- Native recovery costs ≈ 2 ms without pasted pixels and ≈ 25–70 ms with them (CPU, numpy).
- Thresholds (k, margins, fractions, extent) are declared priors, not fitted.

## Q02-B WIP interface changes required (WIP not modified)

| Area | Change needed in `/home/fair/workspaces/aerial_gym_ws/q02b_staged_wip_2026-10-21/` |
|---|---|
| Record fields | `depth_foreground/consistent/free`, `lidar_*` → `record.camera.*`, `record.lidar.*` (`SensorRays`); add `identity_status`, `pasted_pixels_in_footprint`, `near_obstacle_warning`, `contract_version` |
| Hypotheses | TARGET_VISIBLE now means "camera flag, footprint not contradicted"; acquisition should require `identity_status != CONTRADICTED`; IDENTITY_AMBIGUOUS includes the flag-contradiction case |
| Sensor validity | new checks `depth_structure` and LiDAR INVALID categories; "informative decision" must use `report.camera_usable` (now includes `depth_structure`) |
| Loss detector / features | `occlusion_evidence` feature must use `SensorRays`; UNKNOWN reasons `CAMERA_NATIVE_NO_COVERAGE`, `NEAR_CLIPPED_IN_LINE_OF_SIGHT`, `SENSOR_DISAGREEMENT_*` must count as non-informative, not as misses; `near_obstacle_warning` is a new safety feature |
| Confidence | label change only; never used as a probability |
| Map | `localmap.build` must classify LiDAR with `classify_lidar` (0 → NEAR_CLIPPED is occupied-near, not free; 12 → no return) |
| Tests | the WIP tests import the old 20 m scene helper; they must be rerun on the v2 helper |
