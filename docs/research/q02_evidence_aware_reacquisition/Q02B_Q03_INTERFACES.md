# Q02-B / Q03 interfaces and record schemas (CPU shadow package, 2026-10-22)

`codex/mt-p1`, CPU only. GPU NOT_RUN, G5 HOLD. No online intervention: the shadow record can't carry
an action, and no executor exists. This file supersedes the staged design draft for the parts that
changed. That draft is kept unchanged outside Git in `q02b_staged_wip_2026-10-21/docs/Q02B_DESIGN.md`;
its sections 1, 5, 7 and 8 still describe the scope, state machine, firewall and comparison
policies. Evidence contract: Q02A-v2.6 (record schema `Q02A-RECORD-3`); see
[the v2.6 repair record](../../evidence/q02a_v26_repair_2026-10-22/REPAIR.md).

## 1. Pipeline (one call per 0.1 s decision)

```
EvidenceInput ─► evidence.estimate (Q02A-v2.6) ─► history.append ─► LossState
      │                                                   │
      ├─► belief.from_tracker / forecast                  │
      ├─► obstacle_map.LocalObstacleMap.integrate ────────┤
      │        (camera depth 40×24 + LiDAR 4×72, fused,    │
      │         timestamped, ego-motion via world anchor)  ▼
      │                         reacq_planner.plan  (only when ever acquired AND currently lost)
      ▼                                                   ▼
 timing.extract (19 features, delta_u NOT_ESTIMATED) ─► shadow.ShadowMachine.step ─► ShadowDecision
```

The only entry point is `pipeline.ShadowRecorder.observe(episode, decision, inp, provenance, terminal=None)`,
which returns `(ShadowDecision, candidates)`. It takes no ground-truth argument: an unknown keyword
raises `TypeError`. It holds no simulator handle and draws no random numbers.

## 2. Previously-acquired eligibility and acquisition (`history.py`)

**Usable detection.** A decision counts as a usable detection only when all of these hold:
- it is informative;
- the record is flagged;
- `geometry_support == FOOTPRINT_CONSISTENT`;
- `identity_verdict == CONSISTENT_NOT_CONFIRMED`;
- the NIS continuity gate passes: χ²₃(0.999) = 16.27 on consecutive tracker means;
- the identity is not suspect.

**Identity outcomes.**
- **AMBIGUOUS or CONTRADICTED** makes the identity suspect. The decision is not a hit and the
  shadow decision is `IDENTITY_SUSPECT`.
- **Flagged + NOT_TESTABLE** is `IDENTITY_UNVERIFIED`. The decision is neither a hit nor a miss.

**Acquisition and reacquisition.**
- Acquisition needs 2 consecutive usable detections. Loss needs 2 consecutive informative misses.
  Reacquisition needs 2 consecutive usable detections.
- Every acquisition and reacquisition carries
  `acquisition_kind = PROVISIONAL_FOOTPRINT_CONSISTENT_NOT_IDENTITY_CONFIRMED`.
- No record says "confirmed".

**Provenance.** Every record's `detector_provenance` must match the declared lineage, using
`RECORD_PROVENANCE`, for example `FROZEN_F_ORACLE_PROXY_RENDER → ORACLE_TRUE_POSE_RASTER`. A
mismatch raises `HistoryError("DETECTOR_PROVENANCE_MISMATCH")`.

## 3. Local obstacle map v0 (`obstacle_map.py`)

**Structure.** A world-anchored rolling 2-D grid over an altitude slice:
- 0.2 m cells, 120 × 120, which covers ±12 m;
- the slice is ±0.5 m around the vehicle altitude;
- it re-anchors by whole cells when the vehicle is 4 m from the centre;
- an altitude change above 0.3 m resets the map.

Each cell keeps three last-evidence timestamps: free, occupied and conflict. The state is computed
at query time:
- OCCUPIED, if the occupancy evidence is less than 5 s old and newer than the free evidence;
- otherwise CONFLICT, if the conflict evidence is less than 2 s old;
- otherwise FREE, if the free evidence is less than 1 s old;
- otherwise UNKNOWN.

Within one frame, occupancy wins. Free space from one sensor that coincides with a hit from the
other sensor is CONFLICT, never FREE. Evidence stamped after the query time is ignored.

**Why this structure.**
- Polar single-scan maps (the WIP `localmap.py`) lose evidence as soon as the vehicle moves.
- A 3-D voxel grid costs about 20× more for a controller that holds altitude.
- An ESDF needs integration that MOTAR does not have.

The 2-D world-anchored grid is the simplest structure that keeps the following:
- ego-motion consistency, because the anchor is in world coordinates from the validated pose;
- free, occupied and unknown kept separate;
- per-evidence time.

**Cost, measured on CPU.**
- Memory: four float64 120 × 120 arrays, about 0.46 MB.
- Integrate plus one plan: median 78 ms, p95 113 ms per decision in the Phase 4 benchmark
  (`shadow_benchmark.json`). The p95 is above the 0.1 s period; this is P2.

**Sensor semantics.** The map uses no ground truth.
- **Camera depth.** Native 40 × 24 depth only. The 0.2–10 m no-return is filled up to the far
  plane and never beyond it; this preserves the 10 m no-return semantics.
- **LiDAR.** No-return carves free space up to 12 m. A near-clipped beam carves nothing and raises
  `NEAR_CLIPPED_RETURNS`.
- **Free space between adjacent rays.** It is filled only under the declared minimum obstacle width
  of 0.40 m (`SensorProfile.min_obstacle_width_m`). The fill reaches 4.6 m for LiDAR and 8.4 m for
  the camera.
- **Rejected frames.** These give a `FrameReceipt` with `accepted=False`:
  - `FRAME_NOT_NEWER_THAN_MAP`;
  - `POSE_INVALID`;
  - `CALIBRATION_MISMATCH`;
  - `NO_USABLE_RANGE_SENSOR`.

**Query interface.**

| Call | Returns |
| --- | --- |
| `state_at(xy, now_s)` | FREE / OCCUPIED / UNKNOWN / CONFLICT |
| `segment_status(a, b, radius, now_s)` | `(SAFE \| BLOCKED \| UNCERTIFIED, unknown_fraction, reasons)` |
| `line_of_sight(a, b, now_s)` | CLEAR / UNKNOWN / BLOCKED |
| `clearance(xy, now_s, max_m=3.0)` | metres to the nearest non-FREE cell (OCCUPIED, CONFLICT **or UNKNOWN**), capped at 3 m; 0 for an empty map |
| `validity(now_s)` | dict: frames, last stamp, age, stale flag, near-obstacle warning, z_ref, representation, `ground_truth_used=False` |

`segment_status` returns SAFE only when every sampled body disc is FREE. A stale map never
certifies (`MAP_STALE`).

## 4. Candidate planner v0 (`reacq_planner.py`)

**Inputs.** `plan(map, position, yaw, velocity, belief, now_s, camera, limits, params)`:
- own state;
- a `BeliefSnapshot` (mean, covariance, validity);
- the local map;
- the camera FOV;
- `MotionLimits`;
- `PlannerParams`.

**Output.** `(candidates, status, reasons)`, where `status` is `None` or `NO_FEASIBLE_PLAN`.

**Candidates.**
- Kinds:
  - LEFT / RIGHT lateral at 0.75 and 1.5 m;
  - ADVANCE 1 m;
  - RETREAT 1 m;
  - ROTATE in place.
- Each candidate has a straight-line trajectory sampled at 0.1 s. The heading faces the belief mean.

**Fields of each `Candidate`.**
- `candidate_id`, `kind`, `position_xy`, `heading_rad`, `trajectory`;
- view shares at the candidate's pose: `visibility_score`, `visibility_unknown_share`,
  `visibility_blocked_share`;
- the same shares at the current pose: `current_visibility_score`, `current_blocked_share`;
- path terms: `known_space_clearance_m`, `unknown_exposure`, `travel_time_s`;
- the four levels and `reasons`.

All shares are computed over 7 deterministic sigma points of the belief: the mean and ±2σ along
each principal axis, weighted 1/3 for the mean and 1/9 for each of the others. `score_semantics`
says the score is uncalibrated and is not a probability.

**Four levels, kept separate.**

| Level | Field | Meaning | Values it can take |
| --- | --- | --- | --- |
| L1 | `l1_observed_direction_open` | path not intersecting observed occupancy or conflict | PASS / FAIL |
| L2 | `l2_kinematic_check` | L1, plus: the path is entirely in observed FREE for the 0.45 m body radius and the map is fresh; travel time ≤ 3 s, using a trapezoid speed profile (1.5 m/s, 2 m/s²), the velocity-cancel time (≤ 1 s) and the yaw-rate cap (3 rad/s); the stopping overshoot (the braking distance at peak speed beyond the endpoint, adapted from `boia.safety`) is in observed FREE for the body radius | PASS / FAIL |
| L3 | `l3_dynamic_feasibility` | executable by the real controller/plant | **always NOT_VALIDATED** |
| L4 | `l4_closed_loop_safety` | safe under closed-loop execution | **always NOT_VALIDATED** |

The score never overrides L1 or L2.

**Plan status.** `NO_FEASIBLE_PLAN` is returned when no candidate passes L2 with a view gain. The
view-gain rule is: the clear share rises, or the viewable share (clear + unknown) rises. Other
reasons:
- `BELIEF_INVALID`;
- `LOCAL_MAP_EMPTY`;
- `LOCAL_MAP_STALE`.

There is no trajectory follower and nothing is connected to an executor.

## 5. Timing features (`timing.py`)

There are 19 features in `FEATURES` order. Each is `Feature(name, AVAILABLE|UNAVAILABLE, value, provenance)`.

Compared with the WIP's 18 features:
- `identity_evidence` is added. It carries the identity verdict, geometry support, sphere-fit
  status and detector provenance.
- `local_clearance` now comes from `LocalObstacleMap`.
- The `candidate_*` features come from the planner. They count L2 PASS candidates with a view gain.

`delta_u` is always `NOT_ESTIMATED`.

## 6. Shadow decision record (`shadow.ShadowDecision`)

**Fields.**
- `episode`, `decision_index`, `decision_s`, `policy`, `state_before`, `state_after`;
- `decision`: CONTINUE_F / ABSTAIN / PROPOSE_REACQUIRE_SHADOW;
- `reasons`: they include the trigger reason, for example `H3_OCCLUSION_TRIGGERED`;
- `features`, `delta_u`, `evidence_hypothesis`, `plan_status`;
- `owner` (always `F`), `action` (always `None`), `action_applied` (always `False`). `__post_init__`
  refuses anything else, and also refuses a BOIA proposal;
- `fallback_if_no_plan` (`UNDEFINED_PENDING_CONTROLLER_QUALIFICATION`), `ground_truth_used` (False),
  `terminal_source`;
- `identity_verdict`, `geometry_support`, `detector_provenance`;
- `local_map_status`: map validity plus the frame receipt;
- `candidate_summary`: evaluated, L1-open and L2-pass counts, L2 passes with a view gain, the L3/L4
  values (NOT_VALIDATED), plan status and reasons;
- `runtime_s`: host-measured.

**ABSTAIN codes.**
- `NOT_PREVIOUSLY_ACQUIRED`
- `INPUT_INVALID`
- `EVIDENCE_UNKNOWN`
- `IDENTITY_SUSPECT`
- `IDENTITY_UNVERIFIED`
- `BELIEF_INVALID`
- `NO_FEASIBLE_PLAN`
- `LOCAL_MAP_UNAVAILABLE`
- `MIN_DWELL_NOT_MET`
- `COOLDOWN_ACTIVE`
- `SWITCH_BUDGET_EXHAUSTED`
- `UTILITY_NOT_ESTIMATED`
- `TERMINAL`

**Proposals.** Only H1, H2 and H3 can propose. Every proposal carries `EXECUTOR_NOT_AVAILABLE`.
An example record is in
[shadow_record_examples.json](../../evidence/q02b_q03_cpu_package_2026-10-22/shadow_record_examples.json).

## 7. Superseded WIP modules

`localmap.py` (single-scan polar map) and `candidates.py` (fixed lattice) remain importable. They
are kept for their original tests, including the equality test against `boia/geometry.known_free`.
The pipeline no longer uses them.
