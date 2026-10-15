# KROC2027 — moving-target collision counterexamples: read-only findings and analysis plan

Status: plan for the user's review (2026-10-15). Nothing here changes H, E1 or E2, and nothing runs on
the GPU without a separate approval.

## 1. What the canary measured (twice, identical)

Canary KROC27-A-C01 and its re-run (`results/kroc2027_A_canary_v1*/cells/A3|A4/`), seed 4190, 4 quota
episodes per environment. G1/G2 passed in every cell. Swept clearance of the 0.15 m target proxy failed
only at 160 bars, for both E2_obstacle_aware and E1_cv, in A3 and in A4:

| cell (160 bars) | episodes | below 0.15 m | footprint reached (0 m) | distinct scenes |
|---|---|---|---|---|
| A3 E1_cv | 667 | 11 | 5 | 11 |
| A3 E2_obstacle_aware | 871 | 18 | 6 | 18 |
| A4 E1_cv | 1,527 | 57 | 32 | 57 |
| A4 E2_obstacle_aware | 1,503 | 45 | 17 | 45 |

No violation at 70, 115 or 205 bars. The re-run reproduced the verdicts exactly, so the counterexamples
are deterministic for this seed and can be replayed.

## 2. Read-only code findings

1. The bounded lineage (E0/E1/E2, `NAVRL_TARGET_DYNAMICS=bounded`) screens candidates against bar
   **centres** with a 0.77 m clearance (`_target_planner_clearance`, `bars_half_extents=None`;
   `navrl_task.py` around 7936–7985). With bar circumradii up to 0.5465 m, any screened position is at
   least 0.22 m from every bar surface, so a 0.15 m proxy cannot touch a bar at a screened position.
2. `bounded_drone_target_step` (`target_motion.py` 268–466) scores infeasible candidates by their
   longest safe prefix and returns the best one **even when no candidate's first step is safe**; it
   reports that in its fourth return value (`immediate_feasible` of the chosen candidate).
3. For a virtual target the task writes the chosen step back **unconditionally**
   (`target_position = where(moving, bounded_xy, old_xy)`, around 8126). Feasibility gates only the
   physical-target path. The per-step flag is kept in `task._tm_last_step_feasible`.
4. Candidates are constant-heading rollouts limited to 150 °/s turning and 4 m/s² acceleration over a
   1.0 s horizon; there is no terminal braking certificate. A fast target entering a pocket narrower
   than two clearances can reach a state from which no first step is safe.

**Working hypothesis H-MT1:** every counterexample step is an executed step with
`_tm_last_step_feasible == False` (the fallback), reached through a pocket geometry that the 160-bar
`navrl_band` layouts produce more often than the other densities. Alternatives to rule out:
H-MT2 a reference velocity pointing into a bar cluster (E2 waypoint inside a cluster, or E1's held
heading), H-MT3 the wall margin squeezing a corridor, H-MT4 a scene-construction effect (merged bars) at
160 bars.

## 3. The violation packet (recorder only; no change to H/E1/E2 or to navrl_task.py)

The target state between two steps does not change, so the recorder can snapshot the executor's inputs
at the end of the previous step (`after_perception`) and judge the step in `after_target_advance`. On
the first N = 200 violating steps of a cell it writes `kroc2027_violations.jsonl` with:

- episode key, env, action index, scene hash;
- previous and new position, previous and new velocity, `_tm_speed`, pattern, waypoint or held
  heading, avoid sign, planner bounds and wall margin;
- `task._tm_last_step_feasible` for that step (the executor's own flag);
- the bars within 3 m: centres and half extents;
- swept clearance of the step and the nearest bar;
- a deterministic replay of `bounded_drone_target_step(..., return_certificate=True)` on the
  snapshotted inputs: chosen index, every candidate's angle, speed scale, first-step safety, safe prefix,
  full-horizon safety and minimum clearance; plus a check that the replayed step equals the executed one
  (if not, the packet says so and the replay is not used as evidence);
- the previous 20 positions of that target (ring buffer) to show how it entered the pocket.

Tests (CPU): packet fields on a synthetic trap; replay equals the executor on recorded inputs; no RNG
use, no task writes (the existing recorder contract tests extend to the packet path).

## 4. Analysis run (needs a separate GPU approval)

Re-run only the 4 counterexample cells (A3/A4 × E1/E2, 160 bars, seed 4190, 4 per environment) with the
packet recorder: about 0.15 GPU hour. Outputs: share of violating steps with the executor flag False
(H-MT1 predicts 100 %), pocket widths from the packet bars, reference-velocity direction relative to the
blocking bars (H-MT2), wall distance (H-MT3), and bar-overlap statistics of the scenes (H-MT4). Report
written to `results/kroc2027_moving_target_packets_v1/`.

Estimate: implementation 3–5 h, CPU tests 1–2 h, GPU 0.15 h, analysis 1–2 h.

## 5. If a new motion lineage is needed (separate package)

H, E1 and E2 stay as they are. A new lineage "E3" would keep E2's references and limits but never
execute a first step that is not certified: candidates must keep a full braking trajectory clear of
every bar footprint (exact rectangles, as the gate measures), and if none does, the target brakes along
the certified stopping trajectory of the previous step (a terminal-safe invariant from a safe start).
`braking_aware_route_step` already contains most of the braking certificate.

| item | estimate |
|---|---|
| implementation and CPU audit on the recorded real layouts | 10–16 h |
| canary for E3 (gates at all four densities, A3/A4) | 0.3 GPU h |
| A3/A4 main run, 56 cells (from the canary's moving-cell times × 4) | 3.5–5 GPU h |
| preregistration amendment 2 (H-A2 restored with E3) | before any E3 data |

Total about 4–5.5 GPU hours, which fits the remaining budget after B (about 22 h). Freeze is 10/28;
the critical path is implementation and its CPU audit.
