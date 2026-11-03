# Terminal interception: separate problem statement (SPECIFICATION ONLY, not implemented)

Date 2026-10-29. Context: P0-R treats every LiDAR return, the target's included, as an obstacle at plan radius
R = 0.45 m. With the target sphere r_T = 0.20 m a certified segment keeps a centre distance >= R + r_T = 0.65 m, while
capture needs a swept centre distance < 0.5 m (`W/docs/plans/prereg_b1r_classical_baseline_v1.md:261-270`). The
final approach is therefore **outside the certificate by construction**. This document fixes the questions a terminal
mode must answer before any implementation or evaluation; nothing here is approved.

## Facts it must respect (MOTAR, read-only sources)

| Fact | Value / source |
| --- | --- |
| Target | virtual analytic sphere r = 0.20 m, no contact (`target_motion.py`, LiDAR kernel `warp_lidar_kernels.py:167-251`) |
| Capture / crash | capture: swept target-relative distance < 0.5 m; crash: contact > 0.05 N, z < 0.1 or > 4.0, or xy more than 1 m outside the 0-40 m arena; capture wins over a same-step crash (`navrl_task.py:6071-6124`) |
| Body | quad 0.28 x 0.28 x 0.08 m; any-attitude ball 0.202 m |
| Sensing near the target | LiDAR rows +20/+10/0/-10 deg, 5 deg azimuth: at range d the +-10 deg rows pass d tan 10 = 0.176 d above/below the sensor height, so a 0.2 m sphere at sensor height is seen by the 0 deg row only beyond about 1.1 m, while a 3 m bar is seen by every row that reaches it |
| Camera | detector range MAE 0.178 m (v7 CNN, offline); frozen F ran the bootstrap colour segmenter; closed-loop range error std 0.705 m (`navrl_detector_v7_error_profile_seed419/profile.json`); latency 0 in simulation |
| Braking | from 2.49 m/s: stop time 0.9-1.0 s, stop distance 1.04-1.05 m, p10 deceleration 2.96 m/s^2 (`results/navrl_v2_speed_governor_braking.json`) |

## Q1. When does Navigation hand over to Terminal mode?

All of, evaluated on the current packet and the current tracker belief:
1. The tracker is active and fresh (age <= 0.2 s) and the camera detected the target in each of the last N_T = 3
   steps (no interpolation across misses).
2. Predicted centre distance to the target at the end of the next step <= D_T, with D_T >= R + r_T + v_max dt so that
   the hand-over happens while the last certified segment still ends >= 0.65 m away (D_T about 0.9 m at 2.5 m/s;
   fixed before evaluation).
3. The last certified Navigation segment ends inside the terminal entry ball and its certificate is for this step.
4. The target-corridor check of Q2 passes.
Otherwise Navigation continues (certified segments only) or brakes.

## Q2. How are non-target obstacles near the target told apart?

Not by the camera alone (F1 showed that a noisy estimate cannot separate them). Proposed discriminators, LiDAR first:
1. **Vertical extent.** Bars are full-height (0-3 m), so a bar at range d returns in the 0 deg row AND in every
   other row whose beam meets it below 3 m; the target sphere (0.4 m) returns in at most one row beyond about 1.1 m.
   Returns at the target's azimuth in two or more rows are a tall object, not the target.
2. **Motion consistency.** Over the last M packets, target returns move with the tracker's velocity (up to 1.5 m/s);
   static returns stay within the cell. A return cluster that does not move with the track is static.
3. **Static map.** Any STATIC_OCCUPANCY / CONFLICT / unresolved OBSERVED_HIT cell within the terminal corridor
   (the swept body ball from the vehicle to the predicted capture point, plus a margin) forbids terminal entry.
4. **Shape.** Returns of the target lie on a 0.20 m sphere; residuals above a fixed bound are a different object.
The camera may only *select* which return cluster is the goal; it never removes a return from the corridor check.

## Q3. When is the approach aborted?

- Target position uncertainty (tracker 1-sigma) above sigma_abort (for example 0.10 m), or camera miss for 2 steps.
- Any return in the corridor that fails Q2, or a new map veto in the corridor.
- Predicted relative speed above what the braking data allow within the remaining corridor (stop distance about 1 m
  from 2.5 m/s, so terminal speed must already be low at entry).
- Abort action: brake to hover along the last certified segment, then return to Navigation.

## Q4. Swept capture versus physical collision

- Capture is a geometric event on the target-relative swept distance (< 0.5 m centre to centre); the target has no
  collision body, so capture itself cannot be a crash.
- A physical crash in the terminal phase can only come from a bar, the floor/ceiling bounds or the arena. Because
  capture wins over a same-step crash in the frozen termination code, the evaluation must additionally record
  `collision_free_capture` (contact with any bar during the terminal phase, including the capture step), which B1-R
  lists as NOT_RECORDED. Without it, "capture" may hide a bar contact on the same step.

## Q5. If the terminal mode cannot be claimed safe, what can be evaluated?

- Report two numbers: capture with **certified Navigation only** (terminal segments counted as uncertified) and total
  capture; never merge them into a certified capture rate.
- Count terminal entries, aborts, terminal-phase crashes and collision-free captures separately.
- Compare with Policy F only with matched information access (F sees a target-carved, camera-fused scan; the
  classical baselines see the raw 4x72 scan plus the tracker; C would see the raw scan, the tracker and the camera
  labels). The table goes into the preregistration before any GPU run.
- No statement that the terminal mode is safe; at most "empirically measured terminal-phase crash rate".

Until these are fixed and approved, no claim of a complete C-Planner capture-rate comparison is made.
