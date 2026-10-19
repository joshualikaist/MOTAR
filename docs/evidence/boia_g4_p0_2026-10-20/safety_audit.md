# BOIA G4 P0-C/P0-D — physical footprint and braking audit

2026-10-20. Base `13d5f195399bf1a7f1cdf9ddeccd3338d8e34c0f`. New opt-in BOIA CPU changes only; frozen production controller, collision asset, Policy F, RGB-D + LiDAR contract and historical EQ-1D measurements are unchanged. This audit does **not** qualify physical collision freedom or authorize a simulator/GPU run.

**Result:** the body-radius underbound and own-model stopping-tail inconsistency are fixed with independent CPU regressions. Full-body known-free containment and actual-plant safety remain unqualified; G5/hard-safety qualification stays on hold. Machine-readable measurements, commands, pre/post logs, source SHA-256 and all function contracts are in [safety_audit.json](safety_audit.json).

## C01 — collision footprint: FAIL before, CPU PASS after

Asset [quad_navrl_collide.urdf:13–20](../../../resources/robots/quad/quad_navrl_collide.urdf#L13) has one centred collision box `0.28 × 0.28 × 0.08 m`. [NavRLQuadCfg:21–27](../../../aerial_gym/config/robot_config/navrl_quad_config.py#L21) selects this asset. The visual centre sphere is not the collision shape. The canonical Policy F receipt selects `navrl_quad` (M2 §0).

- Level XY half diagonal: `0.197989898732233 m`.
- Orientation-independent 3-D half diagonal: `0.201990098767242 m`.
- A `11.421753659°` tilt places an interior box point at world `(0.200,0,0) m`; its distance exceeds both old `.14 m` hard radius and `.198 m` level-XY soft prior. An independent inverse SE(3) point-in-box test verifies this, and all transformed box vertices at the declared test attitudes are bounded by the new radius.

Actual body tilt cannot be assumed level or bounded by the desired 45° limit. [LeeVelocityController.update:34–53](../../../aerial_gym/control/controllers/velocity_control.py#L34) caps the desired force direction; [EQ-1D D1-A1](../eq1d_qualification_2026-10-18/REPORT.md#L46) reports contact tilt beyond that bound. The full 3-D bound avoids requiring a roll/pitch input for this physical footprint.

[types.py](../../../motar_eval/boia/types.py#L9) declares `VERIFIED_COLLISION_RADIUS_M` and uses it as the new `r_hard` default. Both the swept and stopping screens use `max(r_hard, r_body, VERIFIED_COLLISION_RADIUS_M)`, so a caller cannot shrink the physical floor. `.198 m` remains explicitly the legacy level-XY **soft** prior. Finite/nonnegative radii and delay are checked to prevent a negative delay from erasing the geometry bound. No uncertainty margin is invented.

| Speed (m/s) | Old hard threshold (m) | Amended threshold (m) | Tilted point hard_ok before → after | Stop-tail overlap stop_ok before → after |
| --- | --- | --- | --- | --- |
| 0 | 0.140000 | 0.201990099 | true → false | true → false |
| 0.6 | 0.194000 | 0.255990099 | true → false | true → false |
| 1.2 | 0.248000 | 0.309990099 | false → false | true → false |
| 2 | 0.320000 | 0.381990099 | false → false | true → false |
| 2.5 | 0.365000 | 0.426990099 | false → false | true → false |
| 3.52 | 0.456800 | 0.518790099 | false → false | true → false |

Thresholds are physical floor plus the existing `v × .09 s` scalar allowance. Rejection at higher speed under the old rule does not repair the low-speed/rest footprint defect. `.55 m` synthetic gap at `.6 m/s` still passes: amended hard threshold is below `.275 m` half-gap. Caller-increased radii are retained.

## C02/C03 — limits that remain

**Whole-body known-free containment is BLOCKED.** The current centreline check can accept a stationary centre `(1,0)` with no sensed obstacles, while body point `(1,.1)` lies in a neighbouring sector centred at `+5°` with free radius `.5 m`. The point is inside the actual level collision box, but its scan-origin range is `1.004987562112 m`, outside that sector. An independent angular-sector and box test reproduces this after the radius fix. The test records the limitation; its PASS is **not** whole-body qualification. No global-minimum radius or sampled-occupancy workaround was added.

Physical extent is separate from position, control-tracking and sensor uncertainty. Their bounds are **NOT_RECORDED**. The `.09 s` prior came from historical contact-to-action-end detection delay ([EQ-1D D1-K2](../eq1d_qualification_2026-10-18/REPORT.md#L48)); it is not measured sensing latency, guaranteed pre-contact response, or an independent uncertainty budget. PhysX contact/rest offsets `.002/.001 m` are [source configuration](../../../aerial_gym/config/sim_config/base_sim_config.py#L39), not a newly calibrated sensor/control margin. Tangency, finite numerical precision, missing surface returns and between-ray obstacles are not certified away by a circumscribed radius.

## D01 — stopping-tail contradiction: FAIL before, CPU PASS after

Below `2 m/s`, the unchanged empirical helper extends its first knot with `D=.205 v²`. Including one `.1 s` reaction term gives `.1338/.4152 m` at `.6/1.2 m/s`. The existing zero-command first-order surrogate instead travels `.234/.468 m` asymptotically at `tau=.4 s`, substep `.01 s`.

Independent geometric-series and rotated-box tests place sensed points `.40/.64 m` ahead of the endpoint. After the footprint fix alone, both are `hard_ok=True, stop_ok=True`; the points fall **inside** the actual box during the model's subsequent zero-command braking. This is an internal inconsistency, distinct from the user-accepted high-speed prior mismatch.

The authorized minimal correction is in [safety._stopping_envelope](../../../motar_eval/boia/safety.py#L24), with no edit to `dynamics.py`, the empirical table, production riskcap or Lee controller:

```text
D_own(v) = tau*v + max(v-a*tau,0)^2/(2*a)
D_screen(v) = max(D_table(v) + v*t_react, D_own(v) + v*t_react)
```

For `dv/dt=-min(v/tau,a)`, the first term is the exponential tail, with a constant-deceleration prefix above `a*tau`. This continuous distance bounds the resolved semi-implicit zero-command integration. The implementation validates finite positive `tau`, `a`, `dt` and `dt/S ≤ tau`, using the actual Rollout substep count. The caller must supply the same decision interval in `params.dt` that generated the rollout; the record has no independently logged physical timestamp. Reaction appears **once per alternative before max**, never twice.

| Speed (m/s) | Empirical total (m) | Own continuous total (m) | Amended stop envelope (m) |
| --- | --- | --- | --- |
| 0 | 0.000000 | 0.000000 | 0.000000 |
| 0.6 | 0.133800 | 0.300000 | 0.300000 |
| 1.2 | 0.415200 | 0.600000 | 0.600000 |
| 2 | 1.020000 | 1.000000 | 1.020000 |
| 2.5 | 1.300000 | 1.250000 | 1.300000 |
| 3.52 | 1.922000 | 1.760000 | 1.922000 |

Low-speed counterexamples now reject. A separate acceleration-saturated scalar-step oracle (`a=2 m/s²`, `v=3.52 m/s`) checks the prefix term. This envelope fixes **own-model consistency only**; it is not a calibrated upper bound on actual Lee/motor dynamics.

## Four braking models must remain distinct

| Nominal v (m/s) | U4 discrete stop, no reaction (m) | Fixed empirical knot / +reaction (m) | Riskcap telemetry +reaction (m) | Historical actual Lee 10 ms stop (m), actual v | U4 − knot (%) |
| --- | --- | --- | --- | --- | --- |
| 2 | 0.780000 | 0.820000 / 1.020000 | 0.875675676 | 0.824523925781 @ 1.991752387358 | -4.878048780 |
| 2.5 | 0.975000 | 1.050000 / 1.300000 | 1.305743243 | 1.052917480469 @ 2.489704132081 | -7.142857143 |
| 3.52 | 1.372800 | 1.570000 / 1.922000 | 2.444972973 | 1.560141398840 @ 3.521020907060 | -12.560509554 |

U4 values follow the independent infinite geometric sum `v0*(tau-delta)=.39*v0`; the focused test uses 8 s/800 substeps and its exact finite sum, with the residual explicitly tolerated. Signed distance errors are `−.04/−.075/−.1972 m`. The `12.5605%` high-speed miss remains **LIMITED_VALIDITY**, above the 10% target and below the user-authorized 25% hard rejection threshold. No retuning occurred.

The fixed knots are declared planning values derived from historical measurement ranges, **not newly measured means at exactly those speeds**. Exact actual speeds, distances, stopping times, all three physics-dt rows and source SHA are retained in the JSON from [eq1d_analysis.json → D1.per_fixture.braking](../eq1d_qualification_2026-10-18/eq1d_analysis.json). No new actual Lee run occurred.

Riskcap telemetry is `v*.1 + v²/(2*2.96)`; its separate hard margin `.45 m` enters usable clearance. Those telemetry quantities do not limit the riskcap action. [Original apply_speed_governor:465–503](../../../aerial_gym/task/navrl_task/speed_governor.py#L465) implements `min(requested, cap)`: **2 m/s is the cap floor, not a minimum executed velocity.** At zero clearance, requests `0/.6/1.2` stay `0/.6/1.2`. The coordinated parent-owned dated [M2 banner](../../research/occlusion_algorithm_v1/notes/M2_dynamics_contract.md) and [amendment14](../../research/occlusion_algorithm_v1/14_p0_source_audit_amendment.md) explicitly supersede contrary historical wording while retaining the original text below.

Actual [LeeVelocityController.update](../../../aerial_gym/control/controllers/velocity_control.py#L20) and [BaseLeeController.compute_acceleration](../../../aerial_gym/control/controllers/base_lee_controller.py#L111) re-rotate vehicle velocity at each physics step, form force/attitude/torque commands, allocate motor thrust and apply finite motor response. They are not the first-order velocity assignment. Physics/controller interval is `.01 s`, control decision interval `.1 s` (10 held-command physics steps); motor time constants are `.04 s`. Actual attitude can overshoot commanded tilt, and historical diagonal/yaw behavior is not represented by the surrogate. No current low-speed plant calibration or state-dependent safety bound is claimed.

`braking_distance` remains an uncalibrated extension below/above measured knots. Its above-last warning exists in that helper, but the public `screen` return tuple does not expose it. A zero XY command is **not an instantaneous stop**. No sampled feasible candidate means no feasible sampled candidate under this model; it is not evidence that physical collision is unavoidable, and a fallback command is not a safety certificate.

## Whole-file contract review

Ten functions were read, including both validators and the newly introduced stopping helper. Functions remain stateless across environments; only explicit per-row tensors/configuration are consumed. Complete inputs/outputs, units, frames, time assumptions, batches and limitations are recorded in `function_coverage` in the JSON.

| Function | Contract and qualification |
| --- | --- |
| `types.py::BoiaParams.__post_init__` | Frozen scalar priors to validated record; no state or sensor reads. Counts H/S/M/K; dt/tau/t_det/t_react seconds, radius/distances metres, a_max m/s^2; weights/thresholds dimensionless. Config is shared explicitly across N; no global mutable state. **LIMITED_VALIDITY**. Only explicit positive finite tau_on, nonnegative declared r_bar_est, p_miss range and positive H/S/M/K/dt are checked here; this is not complete semantic validation or calibration. Added safety consumer validates its geometry, response and discretization requirements. |
| `dynamics.py::RiskcapConfig.__post_init__` | Frozen riskcap-only config to finite nonnegative checks, positive width/brake/range, cap/release ordering. Caps m/s; width/margins/clearances metres; reaction/TTC seconds; brake m/s^2. No integration or batch state. **PASS_CONTRACT**. Only receipt-bound riskcap branch, with runtime brake=2.96 unlike upstream default=2.0; not a stopping policy. |
| `dynamics.py::directional_lidar_clearance` | (N,V,B) measured slant ranges, (B,) vehicle bearings, (N,2) commanded vehicle XY velocity and optional permitted association mask -> (N,) forward corridor clearance. Metres/radians/m/s; horizontal projection cos(elevation), per-bearing minimum rows; every env independent, no dt. **PASS_MATCHED_ARITHMETIC**. Original pure file bitwise equality CPU float32/64; finite rows below .995 Rmax; fixed original vertical FOV convention; sparse returns do not certify complete obstacle volume. |
| `dynamics.py::apply_riskcap` | (N,2) vehicle XY request + (N,) corridor clearance -> governed vehicle request and N-first cap/scale/TTC/stopping-margin telemetry. Velocity m/s, clearance/margin metres, TTC seconds. No integration; rows independent. **PASS_MATCHED_ARITHMETIC**. 2 m/s is cap FLOOR, never a minimum requested/executed velocity; brake/reaction/margin affect telemetry only. Does not enforce stopping feasibility. |
| `dynamics.py::point_corridor_clearance` | Original world XY points/validity (N,P,2)/(N,P), predicted world pose (N,K,3), yaw (N,K), vehicle request (N,K,2) -> (N,K) corridor clearance. Metres/radians/m/s; subtract world translation and invert current yaw; N and K independent; no new sensor scan or integration. **LIMITED_VALIDITY**. Reprojects measured snapshot at moved poses, cannot discover unseen obstacles; explicitly ADAPTED and not raw-scan bitwise equivalence. |
| `dynamics.py::rollout` | Normalized candidates (N,K,4), world p/v (N,3), yaw (N,), points, config and explicit H/S/dt -> Rollout records. Clip normalized action, XY*2.5 m/s per axis, yaw*3 rad/s; body/vehicle command re-rotated using each current yaw. dt=.1 s, S=10 -> .01 s substep; semi-implicit world XY integration, fixed z/zero vz; N/K independent. **LIMITED_VALIDITY**. tau_v=.4 s unqualified prior, clipped acceleration 9.81 m/s^2; no actual Lee attitude/motor/altitude dynamics. Positive tau_psi requires explicit initial Euler yaw rate. H/S/dt are call arguments; safety assumes params.dt is the same decision interval used to generate Rollout. |
| `dynamics.py::braking_distance` | Finite nonnegative speed tensor, declared knot tuple and reaction -> total distance and above-last-knot flag of same shape. Speed m/s, distance metres; reaction seconds included ONCE. Below-first constant-deceleration extension, linear interpolation, above-last anchored quadratic extension. Tensor shape preserved. **LIMITED_VALIDITY**. Exact nominal knots .82/1.05/1.57 are fixed design values, not newly measured means. Extrapolation is a prior; no low-speed actual Lee calibration. Original helper remains unchanged; safety now envelopes its low-speed inconsistency. |
| `safety.py::_segment_clearance` | World XY segment batches (N,...,2), points (N,P,2), validity (N,P) -> minimum sensed-point segment distance (N,...). Metres, all shared world XY; element-wise difference-based distances, no matrix-distance TF32 path; N-independent, no dt. **PASS_ANALYTIC_POINT_GEOMETRY**. Empty/all-invalid point sets give +inf; only sensed points, not obstacle surfaces, ray gaps or whole-body known-free containment. |
| `safety.py::_stopping_envelope` | Final planar speed (N,K), explicit response config and actual Rollout substep count -> conservative maximum of table total and own-model continuous total. Metres from m/s, seconds, m/s^2. Checks finite positive tau/a/dt and params.dt/substeps <= tau. Each N/K entry independent. **PASS_OWN_MODEL_BOUND_ONLY**. Continuous derivation for dv/dt=-min(v/tau,a) bounds resolved semi-implicit zero-command response; one reaction term per branch before max. No actual-plant certificate or uncertainty budget; decision interval consistency is an explicit caller precondition. |
| `safety.py::screen` | Rollout (N,K,H,S,...), permitted world XY point set/validity and FreeContext -> hard_ok/stop_ok (N,K), clr_min (N,K,H), soft deficit (N,K). All positions/clearances metres, velocity world m/s; speed*delay metres. Uses every initial-to-substep and substep-to-substep segment; chunks 256 segments, no cross-env reductions. Endpoint stop extends current planar velocity direction by new envelope. **LIMITED_VALIDITY**. Physical radius floor fixed for current NavRL URDF; full-body containment in known free unresolved; position/control/sensor errors NOT_RECORDED. Linear tail direction/first-order response is not actual Lee certification; finite candidate rejection does not prove unavoidable collision. |

All record fields were reviewed as well (`record_field_contracts` in JSON):

| Record | Fields and units/frame limitations |
| --- | --- |
| `BoiaParams` | `H,dt,S,M,K,d_int,p_pre,H_pre,sigma2_lost,n_lost,r_hard,r_body,m_s,t_det,t_react,m_free,r_T,r_bar_est,s_Q,tau_v,tau_psi,a_max,weights_visible,weights_lost,tau_on,tau_off_ratio,n_min,n_off,min_pixels,p_miss,beta,n_0,unknown_blocks,R_camera_diag`. Counts/weights/probabilities and explicit priors; time seconds, distance metres, variance m^2, acceleration m/s^2; R_camera_diag is declared mixed camera measurement covariance, no invented default. tau_on and r_bar_est remain explicit/None as documented. Only r_hard default amended; r_body .198 is legacy XY soft prior. |
| `CameraParams` | `hfov,vfov,fx,fy,cx,cy,camera_offset,target_radius,max_camera_range,min_pixels`. FOV radians, intrinsics/pixel threshold pixels, offset/target radius/range metres; offset expressed in yaw-only vehicle frame. Declaration alone is not calibration. |
| `FreeContext` | `pos,yaw,free_radius,angles,far_m`. Original scan origin world (N,3), yaw radians (N,), sector radii metres (N,B), bearing radians (B,), far metres; centre-point radial set, not body occupancy. |
| `OnlineInputs` | `pos,vel,vehicle_quat,yaw,body_angvel,lidar_m,angles,elevations,mu,Sigma,active,age,camera_visible,policy_action,cam,lidar_far_m`. N-first copied allowed online inputs. Own p/v world m,m/s, XYZW yaw vehicle quaternion, body angular velocity rad/s, scan metres/bearings/elevations radians; CV target belief [position m,velocity m/s], mixed covariance, age seconds, camera flag, normalized action. Tensor contents are mutable even inside a frozen dataclass; access module owns copying. |
| `Rollout` | `pos,vel,psi,step_pos,step_vel,step_psi,governed_cmd,diagnostics`. World p/v (N,K,H,S,3) m,m/s and yaw (N,K,H,S) rad; step_* select substep end. governed_cmd (N,K,H,4) physical vehicle [m/s,m/s,0,rad/s]. Initial p/v diagnostics are mandatory for safety; tuple is frozen, tensor storage is not immutable. |
| `Decision` | `action,candidate_commands,hard_ok,stop_ok,cost,terms,advantage,mode,engaged,reason,new_state,trace,diagnostics`. Action/candidates normalized (N,4)/(N,K,4); masks/costs (N,K), status N-first, optional explicit next state and trace. No online sensor reads inside record declaration; scores have cost-defined normalization. |

## Findings and closure boundaries

| ID | Proposed severity | Status | Finding |
| --- | --- | --- | --- |
| C01 | HIGH | FIXED_CPU | Default .14 m and caller shrinking override underbound collision footprint, including at rest. Full XYZ box half diagonal .20199009876724155 m now immutable declared floor/default in both screens. |
| C02 | BLOCKER_FOR_HARD_SAFETY_CLAIM | UNRESOLVED | known_free certifies centreline, not entire body. Centre (1,0) can be accepted while physical body point (1,.1) is inside an adjacent unknown sector. |
| C03 | HIGH_FOR_PLANT_QUALIFICATION | NOT_RECORDED | Independent position/control/sensor uncertainty bounds are absent. The .09 s term reflects historical contact-to-step-end delay, not calibrated sensing delay or guaranteed pre-contact response. |
| D01 | HIGH | FIXED_INTERNAL_MODEL_ONLY | Below-2 quadratic table extension understated own zero-command surrogate tail; hard+stop accepted boxes intersected during that response. Safety-only max envelope now prevents this inconsistency. |
| D02 | MEDIUM | LIMITED_VALIDITY_USER_ACCEPTED | tau=.4 prior underpredicts declared high-speed knot by 12.56050955%; exceeds 10% target, within 25% user-approved continuation threshold. |
| D03 | MEDIUM_DOCUMENTATION | CORRECTED_BY_EXPLICIT_M2_BANNER_AND_AMENDMENT14 | M2 section3 sentence saying the law never brakes below 2 overstates a cap FLOOR as an executed-speed minimum. Zero/.6/1.2 requests remain zero/.6/1.2. |
| D04 | HIGH_FOR_PLANT_QUALIFICATION | NOT_TESTED_CURRENT_PLANT | Scalar first-order rollout and straight-line braking tail omit actual Lee attitude/motor dynamics, yaw drift, altitude transients and state dependence. Zero command is not instantaneous stop; fallback cannot certify safety. |
| D05 | MEDIUM | LIMITED_VALIDITY | Low/above-table distances are uncalibrated extrapolation. Above-3.52 warning is returned by braking_distance, not exposed by safety.screen tuple. Candidate rejection means no feasible sampled action under this model, not physically unavoidable collision. |

These eight audit records are not eight independent root causes. Two implementation defects are fixed, one documentation interpretation is explicitly corrected, and five qualification/interpretation limits remain. Parent review owns final severity aggregation.

## Regression evidence and preservation

- Pristine-base C01 capture: **15 tests, 19 failed assertions/subtests**, no simulator. The initially incorrect test harness was corrected before this accepted base capture and is not used as defect evidence.
- After C01, before D01 capture: **19 tests, 10 failed assertions/subtests**. This explicitly isolates the later stopping-tail and parameter-check correction.
- Final frozen focused runs: **19 P0 + 16 existing U5 + 11 unchanged U4 = 46 PASS**, zero failures/errors. Commands, timings, exact output, original-file mirror checks and source hashes are embedded in JSON. Broad joint/public-suite evidence belongs to the parent report.
- Independent methods: XML collision extraction, direct inverse rigid transforms, transformed vertices, scalar table arithmetic, exact geometric-series integration, saturated scalar integration, and the original governor loaded in an isolated CPU child. Calling the same geometry helper is not the only oracle.
- `types.py` and `safety.py` are the only existing modules changed by this subaudit. `tests/test_boia_p0_safety.py` and these two audit artifacts are new. No production file, URDF, historical EQ-1D evidence, `dynamics.py`, raw output or checkpoint was changed. The M2 annotation is explicitly parent-owned and preserved separately from immutable reference bindings.
- No commit, push, merge, GPU/simulator run or data deletion was performed by this subagent. Parent owns final commit and complete raw/checkpoint/WIP/stash preservation inventory; this subaudit does not claim a fresh full raw rehash. Disk floor remains the user's **3 GB**.

## Git Traceability

- Branch: `codex/mt-p1`
- Base SHA: `13d5f195399bf1a7f1cdf9ddeccd3338d8e34c0f`
- New Commit SHA: parent-owned consolidation pending at subagent freeze
- Changed Files: `motar_eval/boia/safety.py`, `motar_eval/boia/types.py`, `tests/test_boia_p0_safety.py`, this `safety_audit.md` and `safety_audit.json`
- Tests: focused 46 PASS; accepted pre-fix failures retained in JSON
- Working Tree: coordinated P0 WIP; protected result symlinks preserved
- Remote Push: NO
- Main Merge: NO
- Raw/Checkpoint Integrity: no writes by this subaudit; full rehash not run here
