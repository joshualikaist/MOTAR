# Minimal diff proposals per new lineage (after EQ-1D; not applied)

Status: **proposal only. Nothing below is implemented, and none of it is approved by the EQ-1D
approval.** Every lineage is an opt-in switch that is off by default, so Frozen Policy F and every
historical result stay byte-identical. Each lineage needs its own baseline, a matched evaluation
and its own qualification rerun. S-L1 changes the actor input distribution (768 of 898 inputs).
It is not a bug fix, and Policy F's historical numbers cannot be compared across it. Order
follows the user's priority.

## 1. E-L0 — evaluation telemetry, identity provenance, loss types, safe end

Switch `NAVRL_EVAL_CONTRACT=1`. Evaluation only: actor input and plant unchanged.

| Change | Where | Evidence motivating it |
| --- | --- | --- |
| Per-substep contact flags and outcome partition (`capture_clean`, `capture_with_contact`, `crash:<cause>`, `timeout_at_rest`, `timeout_moving`) | new `motar_eval/eval_contract.py`, attached through `env_manager.set_physics_step_callback` (the EQ-1D recorder pattern) | crash detection 0.05–0.09 s after first contact (D1-K2); capture wins same-step contact |
| LiDAR association provenance | read-only hook after `_associate_lidar_target`: read `segmentation_pixels` (target id 50) on the associated beam and label the return target or bar | 7/7 target-absent accepts (D2-N2), A6, T3 |
| Silhouette labels: target-alone support, visible support, scale loss, silhouette FOV, range exit | evaluation-only second launch of the detector's analytic target kernel without the mesh query | D2-O1 disagreements S08/S12/S14; C4 edge views |
| Safe-end label | from substep states and layouts (EVALUATION_CONTRACT.md) | G3.12 NOT_RECORDED |

Gate: OFF/ON bitwise non-interference (G0 standard) plus the EQ-1A/EQ-1D label fixtures.

## 2. S-L1 — sensor calibration

Switch `NAVRL_SENSOR_LINEAGE=s_l1`, read once at import like the existing perception knobs.

| Change | Where | Evidence |
| --- | --- | --- |
| Column map `u = cx − fx·tan θ` (\|θ\| < hfov/2) | `navrl_perception._fuse_static_and_extract_obstacles` | A1 5.8164 px / 3.4109° |
| Reproject camera hits through the 0.10/0.03 m mount and re-bin by body bearing | same | A2 −0.0998 m; D2-F1 ≈ −0.1 m on 33 beams |
| Per-elevation binning, not one column minimum copied to 4 rows | same | A3 |
| Render obstacle depth at the delivered 160 × 90, or declare the upsampling as a sensor model and stop fusing blended edge depths | `navrl_detector.render_raw_rgbd` / fusion | D2-R3: 2.3 % of pixels off by > 5 cm, max 7.3 m; phantom obstacles −4.6 / −7.2 m |
| Target-like carve-out keeps returns nearer than the target surface minus the measurement tolerance; carve only within the target's own angular radius | `_fuse_static_and_extract_obstacles` target_like mask | D2-F1 S14: real bar 0.53 m in front of the target deleted, 2.8 m false clear |
| `center_range = measured_surface + lidar_target_radius` (0.20 m), or one radius for both sensors | `_associate_lidar_target` | A4 −0.050 m |
| Camera centre range from per-pixel sphere inversion or minimum depth | `_detect_rgbd` | A7/D2-A1 +0.045–0.051 m |
| `lidar_range_only_update=True` (existing path) | config | T4: tangential variance 1.0 → 0.022 |
| A LiDAR accept resets the age only after camera re-confirmation within a declared window; otherwise label it range association | `_associate_lidar_target` / tracker | T3, D2-N2 |

Policy F under S-L1 is an out-of-distribution robustness evaluation, not a comparison with its
history. Retraining is a separately approved package.

## 3. P-L1 — motor ZOH and physical response

Switch: robot config `integration_scheme = "zoh"`, next to the existing `euler` / `rk4`.

| Change | Where | Evidence |
| --- | --- | --- |
| `rpm ← rpm_ref + (rpm − rpm_ref)·exp(−dt/τ)`, rate limit unchanged | `aerial_gym/control/motor_model.py` | EQ-1B M1–M3 (−0.091 % gain change at dt 0.01 s); D1-M2 composite already within 0.1 % |
| No controller change in P-L1; report yaw/altitude drift under zero command and first-order dt dependence as plant characterisation | — | D1-C1 (2.08× per halving, altitude −0.039 m / 6 s at 10 ms), D1-V1 (yaw −17.8° … −131°) |

Gate: rerun D1 at all three dt with the gates unchanged. Because the ZOH is exact, the remaining
dt dependence would be PhysX and controller, now separable. A controller/yaw-hold lineage would be
separate.

## 4. G-L1 — stopping-safe governor from the measured plant

Switch: `NAVRL_SPEED_GOVERNOR=stopcap` (existing mode; margin ≥ 0 by construction, EQ-1B B5),
with the braking model declared from D1-B1. Measured stopping 0.82 / 1.05 / 1.57 m from
2.0 / 2.5 / 3.52 m/s, mean deceleration 2.1–3.4 m/s². Add the contact-detection delay
(≤ 0.09 s) to the reaction term. The S-L1 carve-out fix is a prerequisite: the governor reads the
same LiDAR. Policy F was not trained with stopcap, so this is a new closed-loop baseline. A preregistered ep25000 stopcap screen already exists (2026-09-02, `results/navrl_v2_ep25000_stopcap_seed49_screen`) and should be reviewed first. Gate:
B5 grid, D1 braking/contact fixtures, then a matched evaluation under the EVALUATION_CONTRACT
outcomes.

## 5. LiDAR-free camera-based perception/navigation

Not a diff of Policy F. A new observation schema without the 288-D scan, RGB-D first
(SENSOR_CANDIDATES.md). It needs new training (not approved) and a body-fixed or declared-gimbal
camera model; D2-Y1 shows the current camera is an ideal yaw-only mount. It also needs its own
EQ-1A/EQ-1D-style qualification and the shared evaluation contract.
