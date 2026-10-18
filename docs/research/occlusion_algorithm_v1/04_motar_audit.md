# 04 — MOTAR perception, dynamics and occlusion-failure audit (G2)

This is a read-only audit of `codex/mt-p1` at `1d4696c`, made on 2026-10-19. Nothing was run on a
GPU, and no source changed.
- **File:line contracts:** [M1](notes/M1_perception_contract.md) (perception) and
  [M2](notes/M2_dynamics_contract.md) (action → motion → termination).
- **Data:** [M3](notes/M3_occlusion_failure_data.md), with its scripts in `notes/m3_scripts/`. M3
  reproduces the committed KROC arm capture totals exactly.
- **Spot checks.** Three claims were re-checked by hand against the source:
  - the detector paint and segmenter (`navrl_detector.py:966-974`, `navrl_perception.py:621-629`);
  - the riskcap law (`speed_governor.py:466-475`);
  - BPOD Eq.15 on p.4.

## 1. Perception chain: findings that change the design

1. **The camera detector is an oracle-grade silhouette.**
   - The renderer paints exactly the occlusion-aware GT target pixels flat red
     (`navrl_detector.py:969-974`).
   - The bootstrap segmenter is a fixed 1×1 conv (3R − 2G − 2B − 0.9), so it reproduces that mask:
     target score 0.816 against threshold 0.55, background ≤ 0.27 (`navrl_perception.py:621-629`).
   - No sensor noise is active for Policy F.
   - Structurally the actor is GT-free, and task-level mutation tests are bitwise clean (EQ-1D D3 and
     the S-L1 firewalls). Informationally, however, "camera visible" means "GT pixel-centre visible at
     ≥ 2 px".
   - **Consequence:** a calibrated detection model p_det would be calibrated to an oracle. Detector
     realism must therefore be an experimental factor, with perturbation arms (09 §4).
2. **The camera and LiDAR are an ideal yaw-only gimbal, and ego odometry is perfect** (D2-Y1). The
   camera optical axis is the yaw-only vehicle x, not body x.
3. **Tracker** (`BatchedConstantVelocityTracker`, `navrl_perception.py:640-723`).
   - CV model with diagonal heuristic Q.
   - **No innovation gate on camera measurements.**
   - Camera R is diagonal in **world** axes, whatever the bearing (a new audit finding, unrecorded).
   - LiDAR updates reset `age`, so `active` is not identity.
   - Expiry after 5 s.
   - The covariance is not calibrated. NIS/NEES were never measured for this filter.
4. **Online visibility.**
   - Legal online: the flags `_camera_visible_now` and `_visible_now`.
   - The S-L1D online class is diagnostic only. It was qualified with a primed exact belief and is
     disabled on C-v2.
   - E-L0 classes are evaluator-only.
5. **Local obstacle evidence.**
   - Raw LiDAR (N, 4×72, 12 m) **includes target returns**. Exclude them using the belief, never
     semantics.
   - Fused-scan defects:
     - legacy false clear of 2.8 m;
     - depth-upsampling phantoms;
     - S-L1B keeps about 82 % of target returns;
     - A7 camera range bias of +5 cm.
   - Depth saturates at 10 m for obstacles, while the camera reaches 20 m.
6. **Online whitelist** (M1 §10). Allowed:
   - own pose, velocity and angular velocity;
   - raw LiDAR;
   - delivered RGB-D;
   - `tracker.{state, cov, active, age}`;
   - camera/fused flags, `last_target_like`, `last_scan_nearest`;
   - calibration constants;
   - Policy F's obs/action and `prev_action`.

   Forbidden:
   - `target_position`/`vel` and `navrl_target_*` (writable GT aliases);
   - `segmentation_pixels`, `detector.target_mask`/`_targets`, `DETECT_CHANNEL`;
   - `task_obs["states"]` and obstacle positions;
   - every E-L0, KROC or forensics output.

## 2. Dynamics and termination: findings that change the design

1. **Timing.** Physics dt 0.01 s × 10 per 0.1 s decision; the controller runs at 100 Hz. Sensors
   render after physics, with 0-step latency.
2. **Action.**
   - a[0:2] × 2.5 m/s is in the yaw-only vehicle frame and is **re-rotated by the current yaw every
     10 ms**.
   - a[2] is overwritten by the altitude PI.
   - a[3] × 3.0 rad/s is the yaw rate. There is no yaw-angle hold, and measured open-loop drift is
     −18° to −131°.
3. **The riskcap governor is not a brake.**
   - cap = 2.0 + clamp((c − 3)/2, 0, 1) × 1.5355 m/s, with c the clearance in the LiDAR corridor.
   - It never goes below 2 m/s.
   - Crashes happen at about 1.96–1.98 m/s, with a median of only 0.02–0.03 m of free travel (EQ-1).
4. **Response.**
   - Lee velocity loop with K_v = 2.5, a 45° tilt cap and first-order motors (τ = 0.04 s).
   - Braking 0.82 / 1.05 / 1.57 m from 2.0 / 2.5 / 3.52 m/s, measured.
   - τ_v ≈ 0.4 s is only **derived**. Rise time and overshoot are NOT_RECORDED, and rollout-model
     error against the simulator is NOT_RECORDED.
5. **Termination.**
   - **Contact.** Force > 0.05 N, flagged 0.05–0.09 s late.
   - **Capture.** A swept relative segment within 0.5 m; it masks a crash in the same step.
   - **The observation at a terminal step already belongs to the next episode.**
6. **Target.** It never reacts to the pursuer. The `E2_direct_l2_v1` lineage is the valid one. Draws
   use the shared global torch RNG.
7. **Isolation.** Reset isolation is qualified for two registered profiles only (ISO-Q02, P03-Q01).
   The production reset rewrites every env's mesh vertices.
8. **Hand-over.** Planner actions enter Policy F's `prev_action`, which creates out-of-distribution
   history for the frozen actor.

## 3. Occlusion-failure data: where the measurable gap is

All of these are associations; no causal claim is made. Sources: OCC-P2 `analysis.json`, the KROC
episodes and H-R3 P01/P03 (M3 §1–5).

| Finding | Value | Implication |
| --- | --- | --- |
| In-view centre-occlusion rate rises with density | Moving E2 3.54 → 9.62 per min (70 → 205 bars); static 1.04 → 4.17; all contrasts SUPPORTED | Occlusion exposure is real and density-driven |
| Occlusions are short | Median 0.7–0.9 s, P95 ≤ 2.1 s; none reached 5 s | Long-occlusion recovery has no headroom |
| Fused resumption after geometric restoration | 100 % | — |
| Policy F re-observes FOV exits | 96–99 % | FOV-exit recovery has no headroom |
| Policy F re-observes geometric losses within 2 s | 77–82 % | Small headroom; the scripted arms that raised it lowered capture by 5 s |
| Visible-start failures | Almost all crashes. Moving 205: capture 90.93 %, crash 9.05 % | **The mission failure is collision** |
| Crashes with an in-view centre occlusion open | 25–34 %, against a 1.4–15 % time share | Crashes cluster during occlusion (common cause: bar proximity) |
| Crashes while the target is not geometrically visible | 43–67 % | — |
| Crash within 2 s after an incident geometric-occlusion onset | 3.23 % (160 bars), 3.82 % (205) | **Recommended primary failure target** |
| Hidden start | 32–62 % of failures never acquired the target; yaw-scan supervisor +0.21 pp, INCONCLUSIVE | A separate first-acquisition problem; v1 extension, not the MVP target |

**Design consequence.** A pure re-observation optimiser would chase the small part of the problem.
Policy F already re-observes almost everything, and the H-R3 evidence shows the extra viewpoint
manoeuvres cost capture time.

The measurable failure is **collision while the target is uncertain or occluded**. The governor
offers no brake there, and the target belief is degrading. The recommended algorithm must therefore
couple:
- occlusion forecasting;
- stopping-feasible safety;
- capture progress.

It must not optimise visibility alone. Details: [`05_candidates.md`](05_candidates.md),
[`07_recommended_spec.md`](07_recommended_spec.md).
