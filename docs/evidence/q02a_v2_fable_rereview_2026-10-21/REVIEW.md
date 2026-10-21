# Q02-A v2 re-review (Fable 5.1) — commit 87cced6

Date: 2026-10-21 KST. Reviewer: Fable 5.1 subagent, read-only, CPU only
(`CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 …/aerialgym/bin/python -B`). Scope: the Q02-A v2 sensor-semantics
repair (87cced6) of the P0/P1 findings in [the first Q02-A review](../q02a_fable_review_2026-10-21/REVIEW.md).
The reviewer's probe scripts are in [`probes/`](probes/). They target the 87cced6 API, where
`recover_native_depth` returned a 3-tuple. Each script reads the worktree at a hard-coded path and
writes nothing.

## Verdict: PASS_WITH_DOCUMENTED_LIMITATIONS

No P0/P1 remains. F1, F2, F3, F4 and F7 are fixed against the production source; F5 and F6 are
partially fixed. The pass carries three conditions:

- (a) Q02-B must not consume `report.depth_native` as an obstacle or free-space map until R1 is fixed.
- (b) The ≤ 0.5 m regime is UNKNOWN or unusable by construction (R4).
- (c) Thin occluders and bars narrower than one native pixel resolve to UNKNOWN, not OCCLUDED (R6).

## Findings

| ID | Sev | File:Line (87cced6) | Problem | Reviewer evidence | Proposed minimal fix |
|---|---|---|---|---|---|
| R1 | P2 | `geometry.py:137-151, 207-211` | Pasted-blob exclusion is not closed. The local second-difference test cannot see a jump lying exactly on a segment boundary, so a curved paste edge is only 8-connected there while `_components` was 4-connected. The ring splits into pieces whose bounding boxes miss the interior, and the self-consistent interior is read as obstacle depth with zero residual. Bounding-box growth also cannot fix a paste touching two opposite borders, and the late residual pass cannot catch either case. | Constant disc r40 inside the image: 2616 pasted pixels kept, 209 native cells read as 2.0 m, residual 1.2e-14, `depth_structure` OK. r25 disc: 456 px kept. r60 disc across top/bottom: 8180 px kept. Half-image paste 3.0: native min 3.0, OK. Production-faithful sphere pastes: 0 leaks and 0 OCCLUDED in 150 + 400 border-crossing frames at 0.5–2.5 m. A leak was reachable only at ≈0.30 m (phantom near-obstacle samples, hypothesis still UNKNOWN). Box-like rectangles: 0/2100 leaks. | Label on a 1-px-dilated mask (8-connectivity). Iterate the late pass to convergence, and return FRAME_MISMATCH if late pixels remain or a flagged component touches two opposite borders. Add adversarial tests. |
| R2 | P2 | `evidence.py:146-153, 268-269` | The LiDAR extended-foreground test takes every hit in the 360° scan within ±0.3 m of the foreground range. An unrelated object at a similar range anywhere converts "target surface nearer than predicted" (UNKNOWN) into OCCLUDED. | F5 case (target 10.5 m, belief 11.5 m, no paste) → UNKNOWN; adding a bar 90° left at 10.3 m → OCCLUDED `LIDAR_EXTENDED_FOREGROUND`. Same with a bar directly behind. | Restrict hits to beams within ≤ 2 beam spacings of the foreground beams, or to points near the line of sight. |
| R3 | P2 | `evidence.py:179-186, 211-213` | `SUPPORTED_BY_FOOTPRINT` is bearing-only and accepts any surface. Pasted depth is never compared with the track range, and camera-native or LiDAR `consistent` hits (obstacle surfaces) count as identity support with zero pasted pixels. | Target pasted at 3 m, belief 8 m, flag → SUPPORTED. No target, bar r0.15 at 8 m, flag → SUPPORTED (LiDAR consistent 1, pasted 0). Bar r0.4 at 8 m, flag → SUPPORTED (camera consistent 4). | Require pasted pixels with depth inside the envelope interval. Report surface agreement separately. |
| R4 | P2 | `validation.py:22, 121` | Close-range availability is non-monotonic. `max_pasted_fraction = 0.5` trips at 0.35–0.45 m (camera unusable, so a flagged frame becomes UNKNOWN). 0.30 m passes only through the R1 leak; 0.25 m is INVALID. | Sweep: 0.25 INVALID, 0.30 OK, 0.35–0.45 FRAME_MISMATCH, 0.50–0.60 OK. A 0.4 m target with flag → UNKNOWN `CAMERA_FLAG_WITH_UNUSABLE_CAMERA_INPUT`. | Make "pasted fraction too large" a distinct check, so the flag branch keeps TARGET_VISIBLE/NOT_TESTABLE. Document ≤ 0.5 m as out of scope. |
| R5 | NOTE | `geometry.py:114-134, 213-228` | The `_solve_without` Woodbury branch is dead code; REPAIR.md attributes the masked solve to Woodbury, but it is an explicit normal-equation build with a `support >= 1.0` heuristic. `observed` is computed before the pixel drop. No failure found. | Single call with an empty `removed`. Checkerboard/sine/staircase → FRAME_MISMATCH, residual inf. | Delete the dead code; recompute `observed` after the drop; mark ill-conditioned grids instead of falling back to `lstsq`. |
| R6 | NOTE | REPAIR.md "Limits"; `evidence.py:143` | Native ray spacing is misstated as "2.2°×2.4°". A 0.4 m bar beyond ≈8.4 m centred between native columns is invisible to the obstacle render. The outcome is UNKNOWN; this is acceptable and needs a wording fix only. | Bar (9, 0, 0.20), target 9.8 → UNKNOWN `CAMERA_NO_RETURN_BEYOND_OBSTACLE_FAR_PLANE`. Thin bar r0.05/0.10 at 6 m → native min 10.000, UNKNOWN `SENSOR_DISAGREEMENT`. | Correct the spacing; keep the limitation. |
| R7 | NOTE | `validation.py:150-152`; `contracts.py:88-89` | One INVALID LiDAR sample invalidates the whole scan (fail-closed; the rule degrades to camera-only with a reason). The production getter maps NaN → 12 m, which the module cannot see. A 0.2 m float32 round-trip classifies VALID_HIT (0.20000002). | One NaN/0.1/12.5 → `lidar:INVALID 1 samples`. | Document; optionally tolerate 1e-6 at the 0.2 m edge. |
| R8 | NOTE | `contracts.py:95, 99` | Camera `depth_near_clip_m = 0.05` has no production counterpart (fail-safe). The comment "3 m tall" cites `env_object_config.py:796-818`, whose :797 says 2 m (3 m only for the `bars_h3` pool at :813). | Source reading. | Mark 0.05 m as declared; fix the comment. |
| R9 | NOTE | `tests/test_q02_evidence.py` | No adversarial recovery case. `test_F4` asserts only `!= DETECTION_MISS` (the actual result is UNKNOWN SENSOR_DISAGREEMENT). The torch comparison is CPU float64; GPU float32 is covered only by the skip-if-absent Q01 test. | 30 OK; real-frame residual 8.6e-7 / 1.1e-6 / 1.2e-6. | Add R1 cases; assert F4's actual outcome. |

## Status of the original findings

| ID | Status | Reviewer's basis |
|---|---|---|
| F1 (P0) | Fixed | The contract matches source (`navrl_detector.py:380-398, :47-49, :927-937, :959-974`). `bilinear_matrix` reproduces torch to 1e-12 and real GPU frames to ≈1e-6. The residual paste-exclusion defect is R1. |
| F2 (P1) | Fixed | `ray_envelope` brute force: 0/20 000 mismatches. Per-sensor origins (camera entry 7.75 vs LiDAR 7.80). The σ caps hold. |
| F3 (P1) | Fixed | LiDAR 0 → NEAR_CLIPPED; other out-of-contract values → INVALID. No exception in 200 random + 30 pathological inputs. |
| F4 (P2) | Fixed | Native sparsity remains a documented limit (R6). |
| F5 (P2) | Partially | The extent test is triggerable by unrelated objects (R2). |
| F6 (P2) | Partially | `SUPPORTED_BY_FOOTPRINT` is bearing-only (R3). |
| F7 (P2) | Fixed | The label matches `navrl_perception.py:2132-2134, :2396`. |

Other checks:
- No torch, simulator or random import in `motar_eval/q02`.
- The whitelist refuses `target_position`.
- Inputs are read-only.
- `compare_v1_v2.py` re-run reproduces the committed JSON byte for byte.
- The LiDAR no-hit constant +1000 maps to 12 m.
- The Q01 run has no target-dynamics override.
- Nothing under `/home/fair/workspaces/aerial_gym_ws` was created or modified, and Q01 raw mtimes are untouched.

## Disposition

R1–R4 (P2) and the cheap notes R5, R6, R8 and R9 were repaired in contract Q02A-v2.1; see
[the v2.1 repair record](../q02a_v21_repair_2026-10-22/REPAIR.md). R7 is documented as a limitation.
Condition (a) is lifted only by an independent review of v2.1.
