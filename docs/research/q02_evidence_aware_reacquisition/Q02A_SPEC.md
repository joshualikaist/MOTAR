# Q02-A — Evidence-Aware Reacquisition: code audit and technical specification

2026-10-21 KST · `codex/mt-p1` · base `8d286cd` · CPU only, no GPU, no online connection. G5 HOLD.

**Research goal.** Moving-target interception in unknown cluttered environments with
*evidence-aware intervention timing*: decide, from sensor evidence and its reliability, when to
keep Frozen Policy F and when to switch to an active reacquisition/avoidance trajectory
(intervention B). Frozen F, its checkpoint and controller stay unchanged. Q02-A delivers the
audit, the contracts and a CPU-only evidence layer; it claims no detection accuracy, no
intervention benefit and no safety property.

Paths: **T** `aerial_gym/task/navrl_task/navrl_task.py`, **P** `.../navrl_perception.py`,
**D** `.../navrl_detector.py`, **TC** `aerial_gym/config/task_config/navrl_task_config.py`,
**EM** `aerial_gym/env_manager/env_manager.py`. Production code is unchanged since the G4-R5
audit at `45a7b68` (`git diff --stat 45a7b68 8d286cd -- aerial_gym/` is empty), so the
[G4-R5 field inventory](../../evidence/boia_g4r5_2026-10-21/SHADOW_READINESS.md) remains valid and is
not repeated here.

## 1. How MOTAR already treats the environment as unknown

| Fact | Evidence |
|---|---|
| Obstacle layout is re-sampled at every reset | EM.reset_idx:321–326 calls `asset_manager.reset_idx`; placement mode from `NAVRL_PLACEMENT_MODE` (T:3476; Q01 profile `navrl_band`) |
| F's perception call receives only rendered RGB-D, sanitized LiDAR, own state, previous action and arena bounds | T:6461–6480 (`render_raw_rgbd`, `_lidar_distance_m` T:6013, `perception.observe`) |
| Obstacle features are built from the LiDAR scan fused with camera depth, not simulator positions | P:1835 `_fuse_static_and_extract_obstacles(lidar_m, raw_depth, …)`; off-lineage fusion P:1910–1921; "position and covariance come from range/angle geometry" P:1925 |
| Actor input = robot history 5×10 + target history 5×16 + obstacle tokens 5×8×12 + fused scan 288 = 898 | P:455–463, histories P:2174 |
| No persistent map: memory is a 5-step token history and the current scan | P:31–39 (history lengths); no occupancy/voxel/ESDF code for the pursuer anywhere in `aerial_gym/`, `motar_eval/`, `tools/` |
| Speed governor uses LiDAR clearance only | T:4967 → `_apply_sensor_speed_governor`, `directional_lidar_clearance` T:5332 |
| The only occupancy planner belongs to the *target* (environment side, GT allowed, never shown to the pursuer) | `target_route_planner.py:1–6` |

So Frozen F is a **reactive, map-free** policy in randomized layouts. That is a statement about
F's inputs; it does not make any new BOIA module GT-free automatically — that has to be enforced
by the Q02 contract (§4).

## 2. Actual sensor and tracker contract (Frozen F profile)

| Item | Contract | Source |
|---|---|---|
| Control / physics | 0.1 s decision, 10 × 0.01 s substeps; one render per decision | T step; Q01 runtime attestation |
| Camera | 160×90 RGB [0,1] + depth; HFOV 87°, VFOV 58°; fx = W/(2 tan(hfov/2)) = 84.30, cx = 79.5, cy = 44.5 | TC:171–208, P:754–762 |
| Camera frame | YAW_VEHICLE axes (level, yaw only; +x fwd, +y left, +z up), origin (0.10, 0, 0.03) m | TC:208, P:784–786 |
| Pixel ray / depth | ray ∝ [1, −(u−cx)/fx, −(v−cy)/fy]; depth = Euclidean range along the ray (not optical z) | P:1738–1746; D kernels |
| Camera detection | renderer rasterizes the target from its true pose; bars correctly remove pixels; `visible` = ≥ 2 pixels; confidence = score × support (not a probability) | D:1–12, P:1608/1639, P:1732–1734, P:800–801 |
| LiDAR | 4×72 zero-width rays, azimuth 180° − 5°·j, elevation 20° − 10°·i, 0.2–12 m, yaw-only at vehicle centre, flattened i·72+j, target proxy sphere 0.20 m | `warp_lidar.py:79–89`, LC:46–61, G4-R5 §1 |
| Tracker | CV Kalman, dt 0.1 s, Q = diag(0.025 dt² ×3, 0.25 dt ×3), H = position, new-track velocity cov I, memory 5 s; predicts once per `observe`, then camera and optional LiDAR corrections | P:640–723 (Q P:660, step P:670, correct P:696, expiry P:692) |
| Camera → tracker measurement | centroid ray × (surface range + 0.15 m) + camera offset, rotated to world | P:1738–1748 |
| LiDAR association | heuristic beam/range window; semantic IDs not used | P:2033; G4-R5 |
| Frozen F action (1,4) | clamp(a)·2.5 m/s vehicle-frame vx, vy, vz and clamp(a3)·3.0 rad/s yaw rate; riskcap rewrites vx, vy; altitude PI overwrites vz; governor scales yaw | T:4953–5008 (T:4967, 5002, 5008) |
| Controller | `lee_velocity_control_navrl` (velocity tracking, 45° tilt cap); **no trajectory follower** in the F loop | TC:124 |
| Clocks | no published capture/arrival time; Q01 recorder declares `DECLARED_SIM_SCHEDULE` (capture = delivery = decision) | G4-R5 §1; `motar_eval/boia_shadow.py` |

## 3. Ground-truth exposure

| Path | Status for Q02 |
|---|---|
| Actor observation | GT-free by the observation contract (AGENTS.md); verified call site T:6461–6480 |
| Tracker inputs | Pixel-derived camera measurement and LiDAR association; no GT state. **But** the camera detection is oracle-grade: it is a rendering of the true target in a unique colour. Detection quality, false positives and identity are therefore not realistic in this profile. |
| LiDAR | Ray casts against the scene including a 0.20 m target proxy sphere: sensor-like, not GT state |
| Renderer arguments, `target_position`, `segmentation_pixels`/semantic 50, `navrl_target_*`, `obstacle_position`, E-L0/H-R3 labels | GT/privileged: evaluator-only, forbidden as Q02 inputs |
| Q01 records | Target/obstacle truth stored only as SHA-256 hashes (`host.jsonl`, `parity.jsonl`) — not usable as labels |

**Q02 firewall.** Q02 modules accept only `motar_eval.q02.contracts.EvidenceInput`, built from a
Q01 `SensorPacket` or the saved packet whitelist; `from_arrays` refuses any other key. No Q02 module
imports the simulator, torch or `random`, holds a task handle or re-calls a sensor getter
(enforced by `tests/test_q02_evidence.py`). Evaluation labels (§7) must be produced evaluator-side
and joined by record ID, never passed into the estimator.

## 4. Module plan and reuse

| Module | Q02-A status | Reuse | Missing |
|---|---|---|---|
| A Sensor evidence | **Implemented (CPU)**: validation, evidence record, geometric hypothesis | Q01 `SensorPacket` whitelist; `boia/geometry.py` conventions | Detector realism; calibration of thresholds; labels |
| B Target belief | Contract only: the tracker's `state/cov` at the decision is the belief; propagate only *beyond* it | `boia/belief.py::tracker_Q, predict, sample` (Q identical to P:660) | Motion-model error (target intent), NIS/NEES coverage |
| C Local map | Data-structure draft (§5) | `boia/geometry.py::lidar_points_xy, known_free, exclude_target` (unknown space blocks) | 3D free volume (B03), freshness, camera-depth integration |
| D Reacquisition candidates | Not started | `boia/candidates.py` (20 commands, F at index 0), `boia/visibility.py` (FOV/LOS), `boia/dynamics.py` (riskcap mirror, rollout), `boia/safety.py` (unknown-blocking screen) | Executor for intervention B, return-to-F feasibility, plant qualification |
| E Intervention timing | Not started (§6) | `boia_research/decision.py` refusal structure | CONTINUE vs REACQUIRE outcome data; F continuation model |

Do not double-predict: the tracker already advances once per `observe` (P:670–681). A decision at
time t uses `track_mean/cov` as delivered; forecasts for t+τ apply `boia.belief.predict` for τ
only.

## 5. Contracts fixed in Q02-A

**EvidenceInput** (`motar_eval/q02/contracts.py`): rgb (3,90,160), depth (90,160), lidar (4,72),
camera_detected, fused_update, confidence, track mean (6) / cov (6×6) / age / active, own position,
velocity, yaw-only vehicle quaternion (xyzw), camera and LiDAR `Clock` (capture, delivery,
provenance, frame, depth convention), decision time, calibration tuple. Arrays are copied and
read-only.

**Validation** (`validation.py`): per check `OK | MISSING | STALE | INVALID | FRAME_MISMATCH`.
Missing or unqualified clocks are MISSING; capture older than 0.1 s is STALE; non-yaw-only or
non-unit quaternions and calibration/convention mismatches are FRAME_MISMATCH; non-finite, out-of-
range or non-PSD inputs are INVALID. Problems are reported, never repaired.

**Evidence record** (`evidence.py`): hypothesis ∈ {TARGET_VISIBLE, OCCLUDED, OUT_OF_FOV,
DETECTION_MISS, IDENTITY_AMBIGUOUS, UNKNOWN}, ordered reasons, source (camera detection vs tracker
prediction), belief kind (measurement-updated vs predicted-only), predicted vehicle-frame position,
range/bearing/elevation, line-of-sight and lateral σ, belief points inside the image, expected
camera (0.15 m) and LiDAR (0.20 m) surface intervals, and depth-window and LiDAR-beam counts
(foreground / consistent / free or far). Always `utility = ABSTAIN`, `ground_truth_used = False`,
`calibration = NONE_DECLARED_PRIORS`; confidence is labelled "not a probability".

**Rule** (deterministic, in order): unusable camera with a detection → UNKNOWN; detection →
TARGET_VISIBLE; no usable track, unusable camera, stale (> 2 s) or diffuse (σ_r > 1.5 m) belief →
UNKNOWN; all belief points outside the image → OUT_OF_FOV; some outside → UNKNOWN; beyond 20 m or
below pixel support → UNKNOWN; depth foreground before the expected surface → OCCLUDED; a surface
at the expected range without detection (depth or LiDAR) → IDENTITY_AMBIGUOUS (G4-R5 B04); LiDAR-
only foreground → OCCLUDED; camera sees beyond the expected surface → DETECTION_MISS (belief error
not excluded); otherwise UNKNOWN. Thresholds are declared priors, not fitted.

**Local map draft (C, not implemented).** Per decision, a sensor-only record: origin pose and
clock; LiDAR rays as (direction, first-hit range, status ∈ {RETURN, FAR, NEAR_CLIPPED}); free
segments only along valid rays up to the hit (G4-R5 B03: inter-ray gaps UNKNOWN); camera depth
columns as additional rays inside the camera FOV. Aggregation over a short window in a 2.5-D
polar/sector grid with per-cell {occupied evidence, free evidence, last update, origin}, decaying to
UNKNOWN; inflation by the 0.14 m hard radius plus pose/time error; unknown never treated as free.
Start from `boia/geometry.known_free`, which already blocks unknown space.

**Budgets (draft).** The Module A estimator measured 0.14–0.16 ms per decision with a detection and
0.33–0.70 ms on the tracker-evidence branch, on the CPU over real Q01 frames (single calls, Python). The planning target for the full A–E stack is ≤ 20 ms per
0.1 s decision on the CPU, with memory ≤ 50 MB per environment for a 5 s local-map window.
Neither is measured for B–E.

## 6. Evidence-aware intervention timing (Module E) — research design, not implemented

At each decision, E compares CONTINUE(F) with REACQUIRE(candidate k) using: the Module A record and
its validation; belief age, σ and their growth; F-continuation risk features (clearance, belief
leaving the FOV, time since last detection); per-candidate features from D (expected visibility
gain, clearance against observed occupancy, unknown-space exposure, dynamic feasibility, time to
reacquire, return-to-F feasibility). Outputs: CONTINUE, REACQUIRE(k), or ABSTAIN when evidence is
insufficient. The decisive quantity is the conditional utility difference
Δ(t) = E[outcome | REACQUIRE now] − E[outcome | CONTINUE], and whether waiting changes it. Δ is
**not** estimable from existing data: it needs matched fresh-process branches (G4-R5 §3) with
evaluator-side outcomes. Until then E may only log features and abstain; no probability or
utility value is invented.

## 7. Data that require a separately approved GPU run

1. Evaluator-side occlusion labels per decision (target pixels before bar removal, LOS truth,
   FOV truth), joined by record ID to the Module A records, to measure the hypothesis rule.
2. Longer and more varied episodes (seeds, densities, initial-view hidden) so that OCCLUDED,
   OUT_OF_FOV and long prediction-only intervals actually occur; Q01 has 3 unique frames, all
   with the target visible.
3. Full RGB-D/LiDAR packets at every decision (Q01 kept 8 snapshots), with a size budget.
4. Matched CONTINUE vs REACQUIRE branches for Δ(t) (needs the executor and plant qualification
   first; G5 HOLD).

## 8. Tests (Q02-A)

`tests/test_q02_evidence.py` (16 tests, synthetic ray-cast scenes with a sphere target and
vertical-cylinder bars): production constants by source text; LiDAR layout, pinhole projection and
ray convention against the production formula; yaw frames; the B04 5.8 m sphere value; every
validation status; visible / occluded / out-of-FOV / detection-miss / identity-ambiguous / UNKNOWN
cases; LiDAR failure degrading to depth with a reason; yaw invariance; record semantics and GT
firewall; purity (no RNG, read-only inputs); import isolation; the Q01 `SensorPacket` adapter.
`docs/evidence/q02a_cpu_2026-10-21/q01_frame_replay.py` runs the estimator over the real Q01 frames
(smoke only, no labels).
