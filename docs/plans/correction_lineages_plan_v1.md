# S-L1 / P-L1 / G-L1 — minimal change plans for the correction lineages (plan only, not approved)

Status: **design document requested on 2026-10-18 together with E-L0. No production change is made or
authorised by it.** Each lineage is a new opt-in switch, default off, so Frozen Policy F and all
historical results stay byte-identical. Each one needs its own preregistration, implementation
approval, E-L0-instrumented qualification and matched evaluation before any claim. The registered
environment problems it addresses are EQ-1A/B/C and EQ-1D (`be07847`).

Common rules:
- The switch-off path must be byte-identical. This is shown with the E-L0 ON/OFF digest test
  (V3 standard) on the lineage commit.
- E-L0 records every lineage run, so improvements are measured by evaluation-only, independent
  labels rather than by the changed component itself.
- S-L1 and G-L1 change what Policy F sees or does. A result of the old checkpoint in the new
  environment is a robustness measurement, never an algorithm improvement. Retraining is a
  separately approved package.

## S-L1 — sensor calibration (`NAVRL_SENSOR_LINEAGE=s_l1`)

| Item | Minimal change (location) | Qualification gate (independent oracle) |
| --- | --- | --- |
| Calibrated pinhole/SE(3) fusion | `navrl_perception._fuse_static_and_extract_obstacles`: map beam bearing θ to column `u = cx − fx·tanθ`; transform each camera hit `p_cam + t_cam` into the body frame and bin it by body bearing and elevation, instead of copying one column minimum to 4 rows | EQ-1A A1/A2/A3 exact fixtures (≤ 0.5 px, ≤ 1e-4 m); EQ-1D D2-F1 on rendered scenes (error ≤ the declared depth-model tolerance) |
| Exact obstacle depth | `navrl_detector.render_raw_rgbd`: render obstacle depth at 160 × 90, or fuse only the native 40 × 24 grid with its own intrinsics; never fuse blended upsampled depths | EQ-1D D2-R3 within 1e-4 m on robust pixels; no phantom obstacle in D2-F1 |
| Target-like carve-out | carve only returns within the target's own angular radius and not nearer than the predicted target surface minus a declared measurement tolerance; never delete a return that LiDAR places in front of the target | D2-F1 S14 has no false clear; a new fixture places a bar 0.3–0.6 m in front of the target |
| Target identity confirmation | a LiDAR accept updates the track but resets the expiry clock only after camera re-confirmation within a declared window; otherwise it is labelled range association. Identity requires appearance evidence | E-L0 provenance: bar-only fixtures produce no identity-preserving accept; the false-association rate is measured on the full E-L0 population |
| Radius/range convention | `_associate_lidar_target`: `center_range = surface + lidar_target_radius` (0.20 m), or one radius for both sensors. Camera centre range from per-pixel sphere inversion or the minimum depth | A4/A7: ≤ 1e-4 m on analytic frames |
| Range-only KF covariance | `lidar_range_only_update=True` (existing path: anisotropic R about the ray) | EQ-1C T4: tangential and vertical variance not reduced by a range-only return |
| Silhouette-based visibility | evaluator side already in E-L0. If the actor's own visibility flags are to change, that is a perception change requiring retraining | E-L0 V1 labels |

Evaluation: S-L1 OFF must be bitwise equal to Frozen F (E-L0 V3 digests); with S-L1 ON, run the
EQ-1A/EQ-1D sensor fixtures and then a matched E-L0 evaluation. Retraining is a separate
decision.

## P-L1 — motor ZOH and physical response (robot config `integration_scheme = "zoh"`)

| Item | Minimal change | Gate |
| --- | --- | --- |
| Exact discrete motor model | `motor_model.py`: `rpm ← rpm_ref + (rpm − rpm_ref)·exp(−dt/τ)`, rate limit as now; `use_discrete_approximation` irrelevant in this scheme | EQ-1B M1–M3 (−0.091 % gain change at 10 ms), EQ-1D D1-M1/M2 rerun |
| Timestep convergence | none in the motor beyond ZOH; rerun D1 at 10 / 5 / 2.5 ms and separate the remaining PhysX/controller dt dependence | D1-C1 with the same 0.01 m / 0.5° gate; report what remains |
| Yaw-drift cause analysis | diagnostic only first: log the Lee controller's commanded yaw rate, body yaw rate and yaw torque under the D1 diagonal and saturation fixtures. Hypotheses to separate: yaw-rate tracking error under tilt (vehicle- vs body-frame yaw rate), allocator rank-4 warning (yaw authority), tilt-limit scaling | D1-V1 diagonal direction error; drift under zero yaw command per fixture |
| Control response and saturation | characterise the step, ramp and saturation response with E-L0 contact labels; any controller change is a separate controller lineage | D1-A1 tilt, D1-T1 periods |

## G-L1 — stopping-safe governor (`NAVRL_SPEED_GOVERNOR=stopcap` plus a declared braking model)

| Item | Minimal change | Gate |
| --- | --- | --- |
| Measured braking and contact timing | stopcap's admissible speed `v` with `a` = measured plant deceleration (D1-B1: 2.1–3.4 m/s²) and reaction = control period + crash-detection delay (D1-K2 ≤ 0.09 s) | EQ-1B B5 grid: margin ≥ 0 everywhere; D1 braking fixtures within the model |
| Safe stop and avoidability | a stop/hold mode that satisfies E-L0 `timeout_at_rest` / safe-abort rules. Safe abort needs an explicit abort channel; until then E-L0 reports `NOT_IMPLEMENTED` | E-L0 episode columns; no synthetic success |
| Matched comparison with riskcap | same seeds, scenes, E-L0 instrumentation, quota population; outcomes `collision_free_capture`, crash causes, timeouts. The 2026-09-02 ep25000 stopcap screen is prior evidence to review first | preregistered contrasts; no metric chosen afterwards |
| Prerequisite | S-L1 carve-out fix: the governor reads the same LiDAR that the carve-out falsely clears | — |

## Order and independence

E-L0 is the instrument for all three. The proposed order is S-L1 (sensor errors, which affect
every later safety number), then P-L1, then G-L1. Each runs as its own preregistered package,
reusing the EQ-1D fixtures and the E-L0 non-interference standard. The LiDAR-free camera lineage
is prepared separately after the single-RGB versus RGB-D choice.
