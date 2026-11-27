# Q02-A independent review — HOLD at Q02-B Phase 0

2026-10-21 KST. Reviewer: Fable 5.1 subagent, fresh context, read-only, CPU only, on
`93fbabf290aa32641a9940ee69479168aa1a6654`. Findings re-checked by the main session against the
source and the Q01-V2 frames before recording. **Verdict: HOLD_P0_P1_FIX_REQUIRED.** Per the
Q02-B instruction, Q02-B development stops at Phase 0; no Q02-B code is committed.

## Findings

| ID | Sev | File:Line | Problem | Independent check | Minimal fix |
|---|---|---|---|---|---|
| F1 | **P0** | `motar_eval/q02/evidence.py:236-239, 277`; `contracts.py:25,43`; Q02A_SPEC §2/§5 | Camera depth is treated as a 20 m Euclidean range image. In production the non-target depth is a 40×24 obstacle render, bilinearly upsampled to 160×90 and filled/clamped at a **10 m** far plane (`navrl_task_config.py:229-231`, `navrl_detector.py:927-937`, target pixels overwritten at `:974`); production maps that fill to "no return" (`navrl_perception.py:216-229`). The module counts the fill as a measured surface. | Real Q01 frames: 97.2–98.4 % of depth pixels at the 10 m fill. Empty scene with 10 m fill and belief at 12 m → OCCLUDED. The Q02-A receipt's "IDENTITY_AMBIGUOUS with camera flag cleared" on real frames is this artefact, not target evidence. | Declare `camera_obstacle_far_m = 10.0` and the 40×24 obstacle resolution in the contract; classify window pixels ≥ far − 1e-3 as NO_RETURN (excluded from foreground/consistent); "free" only if the interval ends before the far plane; if the expected surface reaches the far plane → UNKNOWN `CAMERA_NO_RETURN_BEYOND_OBSTACLE_FAR_PLANE`; tests with a 10 m clamp. |
| F2 | **P1** | `evidence.py:246-248` | LiDAR beams are selected by an angular window padded by half the beam spacing (±2.5° az, ±5° el) on top of the belief spread, so beams 0.7 m beside a clear line of sight count as foreground. | Bar r = 0.30 m at 8°/4 m, clear LOS to an empty belief at 8 m → OCCLUDED `LIDAR_ONLY_FOREGROUND` (depth 81/81 free). | Select beams by the ray's perpendicular distance to the belief centre ≤ k·σ_lat + proxy radius, no spacing padding; none → `LIDAR_NO_BEAM_COVERAGE`. |
| F3 | **P1** | `validation.py:94-98`; `evidence.py:238, 253` | Zero values fail open. Production LiDAR writes near-out-of-range (< 0.2 m) as −max_range, clamped to 0.0 by the getter (`navrl_lidar_config.py:72`, `warp_sensor.py:237`, `navrl_task.py:6013-6017`); an all-zero buffer is indistinguishable from "everything occluded". | All-zero depth → OCCLUDED with no validation problem; all-zero LiDAR → OCCLUDED. | Classify depth < 0.05 m and LiDAR == 0 as NEAR_CLIPPED (excluded from foreground/consistent); an all-zero frame is INVALID. |
| F4 | P2 | `evidence.py:231-239, 291-306` | Fractions over a square window (≈ 81 px) while the target disc is ≈ 8 px; the 40×24 obstacle depth (≈ 2.2° per pixel) makes thin occluders vanish or interpolate into "consistent" values. | Reviewer counterexamples (thin bar → DETECTION_MISS). | Evaluate over the projected target footprint; window ≥ one obstacle pixel; declare the resolution. |
| F5 | P2 | `evidence.py:295-305` | A target nearer than predicted returns its own surface before the interval → OCCLUDED without `BELIEF_ERROR_NOT_EXCLUDED`. | Reviewer counterexample. | Add the reason to OCCLUDED; expose foreground extent. |
| F6 | P2 | `evidence.py:260-262` | TARGET_VISIBLE is the raw camera flag; with same-colour distractors (88.5 % false-lock rate in the distractor envelope) it reports TARGET_VISIBLE. | Reviewer counterexample. | Rename reason `CAMERA_FLAG_VISIBLE`; cross-check against depth/LiDAR footprint; demote to IDENTITY_AMBIGUOUS when unsupported. |
| F7 | P2 | `evidence.py:183`, `contracts.py:110` | Packet `confidence` is max(camera score, LiDAR gate confidence) (`navrl_perception.py:2132-2135, 2396`), not the segmenter score. | Code reading. | Relabel semantics. |
| F8 | NOTE | `validation.py:74-85` | Calibration check lacks the two camera far planes (20 m detector, 10 m obstacle). | — | Add them to the tuple. |
| F9 | NOTE | `contracts.py:109-113` | `fused_update`/`track_age_s` semantics hold only with `NAVRL_LIDAR_SILENT_CORRECT=0`. | — | Document or pin. |
| F10 | NOTE | `tests/test_q02_evidence.py` | `python -m unittest tests/...` cannot import (no `tests/__init__.py`); script form and `tools/run_tests.py` work. | — | Document the run form. |

Passed: frames and yaw sign, camera origin/orientation, pixel ray table, LiDAR layout, proxy radii,
tracker posterior handoff (no double prediction; PSD margins safe in float32), clock fail-closed,
GT firewall structure (no truth field, whitelist).

## Consequences

- Q02-A's hypothesis rule is not usable as Q02-B input until F1–F3 are fixed; its geometry,
  validation framework and contracts otherwise stand.
- The Q02-A receipt `docs/evidence/q02a_cpu_2026-10-21/validation.json` and replay JSON stay as
  recorded history; their real-frame "camera flag cleared → IDENTITY_AMBIGUOUS" result is
  superseded by F1 (an artefact of the 10 m fill), not evidence about the target.
- Q02-B: HOLD. Staged code and design/plan drafts are kept outside Git at
  `/home/fair/workspaces/aerial_gym_ws/q02b_staged_wip_2026-10-21/` (README there); they consumed
  the unfixed rule and must be re-run after the fix.
