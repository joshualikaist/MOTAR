# EQ-0 issue register

Measurement source: `5ff285ad053f181c5a1c86dddb2462bb44e6327b`. Audit base:
`7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2`. No remedy below was implemented. Existing OCC-P1/P2,
Policy F and preregistrations remain unchanged. PASS/FAIL evaluate the explicitly named gate,
not the entire historical experiment. Source references are file:function:line at measurement source.

BLOCKER means a missing prerequisite for the specified **future claim/experiment**, not permission
to stop preserving data or to rewrite a result. There is no blocker to this read-only audit.
HIGH means a material construct/safety/calibration limitation; MEDIUM a bounded model/provenance
limitation; LOW a small localized diagnostic limitation. Independent reproductions below use
analytic or recorded-data oracles; proposed simulator/hardware checks remain NOT_TESTED.

## EQ0-00 — Provisional hypotheses are not a frozen qualification/experiment contract

- **BLOCKER for formal H-R1/H-R2/H-R3 testing; NOT_TESTED.** The user supplied provisional
  definitions during EQ-0, recorded in [PROVISIONAL_HYPOTHESES.md](PROVISIONAL_HYPOTHESES.md).
  They are not preregistered; final metrics, sensor contracts and experimental conditions remain open.
- Evidence: existing [OCC preregistration](../../plans/prereg_occlusion_confirmatory_v1.md) defines
  OCC RQ1/2/3, not H-R1/2/3. Do not interchange them.
- Impact: conditional environment impacts can now be mapped, but no per-H-R effectiveness verdict
  is possible. OCC-P2 can inform design, not confirm the newly selected algorithm hypotheses.
- Minimal check/gate: bind each H-R text to an immutable new protocol, map every required
  qualification subgate, define sensor-only inputs and stopping/outcome semantics before testing.
  PASS means the complete mapping exists, not that the research hypothesis was confirmed.

## EQ0-01 — Camera-to-LiDAR registration uses inconsistent geometry

- **HIGH; FAIL for calibrated common-frame registration.**
- Source: `navrl_perception.py:NavRLPerceptionModule._fuse_static_and_extract_obstacles:1818–1830`,
  particularly linear `camera_u` at 1826; pinhole camera rays
  `navrl_detector.py:NavRLTargetDetector.__init__:367–371`, target pixel inversion
  `navrl_perception.py:1694–1699`. Camera translation [0.10,0,0.03] m is not reprojected into the
  body-centre LiDAR origin in this fusion path. [Detailed source review](supporting/g2/audit.md).
- Actual values: width 160, HFOV 87°, cx=79.5, fx=84.3024100. Pinhole column is
  `u=cx-fx*tan(theta)`; implemented map is proportional to angle, then rounded. At theta=20°,
  correct u=48.8164, selected u=43, ray=23.4109°: 5.8164 pixels / 3.4109° disagreement.
- Physical reason: equal pixel spacing is uniform in tangent, not bearing. A range originating
  0.10 m ahead of the LiDAR cannot be reused as a body-origin return without transforming the hit.
  An aligned front-plane fixture isolates a −0.10 m range error even without angular mismatch.
- Impact: frozen OCC trajectories include this approximation; their count/exposure statistics
  stay valid for that loop. A calibrated recovery map, visibility candidate score or clearance
  supervisor cannot assume these fused rays are metrically registered.
- Independent reproduction: [ray/pinhole script](supporting/g2/independent_checks.py) and
  [math results](supporting/g2/independent_math.json); no project geometry functions are imported.
- Future gate: nongrazing analytic planes/boxes at on/off-axis rays and both origins; projected
  direction error ≤0.5 pixel and transformed range error ≤1e−4 m for exact synthetic fixtures.
  Predefine boundary bands. This does not constitute hardware calibration.
- Proposed remedy: new opt-in sensor lineage with point reprojection and documented elevation
  aggregation; compare to frozen baseline. Do not change old source, Policy F or old outputs.

## EQ0-02 — Active motor routine is not classical RK4

- **MEDIUM at fixed OCC dt; FAIL for the named RK4 mathematics. HIGH before timestep/model transfer.**
- Source: `motor_model.py:rk4_integration:169–175`,
  `discrete_mixing_factor:178–180`, `MotorModel.update_motor_thrusts:100–120`; inherited
  `base_quad_config.py:motor_model_config:175–194` selects RPM mode, tau=0.04 s and default RK4.
- Independent derivation: for `y'=k(r-y)`, e=r−y decreases as y grows. Intermediate slopes require
  `k(e−dt*k1/2)` etc.; the active routine uses plus signs. With h=k*dt its gain is
  `h+h²/2+h³/6+h⁴/24`, versus classical RK4 `h−h²/2+h³/6−h⁴/24`.
- Actual fixed parameters use k=1/(dt+tau)=20, dt=0.01 and h=0.2. Gain 0.2214 differs from the
  same-ODE RK4 gain 0.1812667, but is close to exact **nominal tau** gain 0.2211992; equivalent
  tau≈0.0399588 s. Do not present the 22% same-coefficient difference as the actual plant's
  nominal-lag error. A dt=0.1/tau=0.04 illustrative counterexample overshoots the RPM reference.
- Impact: current synthetic plant remains the measured OCC plant. Physical calibration and
  discretization/model transfer are not qualified; outcome sensitivity has not been measured.
- Minimal independent reproduction: [closed-form cases](supporting/g1/motor_algebra.json),
  reproduced by [independent math checks](independent_math_repro.py); no production integrator call.
- Future gate: analytic first-order step/ramp response and independently integrated nonlinear
  rate limits; correct named method, bounded monotone positive step in its declared stable range,
  and convergence to the chosen continuous-time model. Log actual RPM/thrust before claiming
  physical accuracy. Any fix is a new plant lineage, not a repair to historical evidence.

## EQ0-03 — Clean synthetic perception does not qualify realistic redetection

- **HIGH; LIMITED_VALIDITY. BLOCKER to a real RGB+LiDAR deployment claim.**
- Source: `navrl_detector.py:render_raw_rgbd:894–967`, fixed target RGB [0.88,0.08,0.045];
  `navrl_perception.py:AppearanceTargetSegmenter:615–637`, `_detect_rgbd:1536–1602`;
  `navrl_task_config.py:vision/perception:183–206,274–284`,
  `navrl_task.py:_process_obs_perception:6450–6481`.
- Actual path: analytic sphere RGB-D, no distractors/learned detector/P9/dynamic mesh;
  zero configured latency; ideal yaw stabilization. Noise/dropout magnitudes are recorded but
  the perturbation-enable switch is false. Depth is exact simulated metric range, not RGB-only
  range estimation or measured stereo depth.
- Reason/evidence: independent scan finds camera validity exactly equals ≥2 rendered target pixels
  in all 2,320,985 quota frames. Appearance is effectively an availability code under these scenes;
  it is not evidence of a direct GT argument into the actor. Range/bearing errors may still exist.
- Impact: P1/P2 can describe this ideal clean loop; conditional instant redetection cannot validate
  realistic Recovery. Future online supervision cannot use unavailable synthetic depth/GT.
- Minimal independent reproduction: [G3 standalone reader](supporting/g3/reproduce_independent_audit.py),
  and G2 target-alone ray/sphere fixtures. At 10 m the centred unobstructed target has four pixels;
  at 17.6/20 m it has zero in the same independent grid fixture without an occluder.
- Future gate: qualify online inputs available under the chosen real sensor contract, with no
  unavailable synthetic depth/GT. Current Policy F is LiDAR+synthetic RGB-D; a camera-only candidate
  or real RGB-D hardware change needs a separately approved contract. Ideal baseline analysis is
  a limited simulation study, not a substitute for that Recovery constraint. Require
  independently labelled appearance/scale/distractor controls, deployment-compatible range
  provenance, measured sensor error/latency. Real detection thresholds remain NOT_DEFINED until
  the target task/data requirements are fixed. No invented realistic-accuracy PASS.

## EQ0-04 — Centre LOS is not full silhouette loss or a long-loss benchmark

- **HIGH; LIMITED_VALIDITY for recovery construct; full-silhouette/5–15 s qualification NOT_TESTED.**
- Source: `occlusion_confirmatory_analysis.py:extract_cell:125–159` and frozen preregistration;
  `occlusion_geometry.py:line_of_sight` finite centre/rim samples. Actual primary is in-view centre
  ray blockage, including FOV re-entry; full target-alone support is not recorded.
- Reason: a centre ray can be blocked while the target silhouette remains visible; conversely
  zero pixels may reflect pixel phase/scale/FOV, not obstruction. FOV exit ends the observable
  interval even while physical blockage may continue. Finite rim samples do not cover all pixels.
- Impact: retain RQ1/RQ3 count results, with that operational definition. No supported 5 s risk
  set exists for the primary geometric spells. There are nevertheless 41 Moving fused-loss
  expiries from other loss modes; do not claim no long loss of any kind occurred.
- Independent reproduction/gate: analytic projected target silhouette with known isolated
  occluders, plus target-alone support and acquisition prefix. PASS requires correct classification
  of full/partial/centre-only/FOV/scale loss against independent ray/pixel labels. A requested
  duration must actually be realized or NOT_REALIZED; no post-hoc reseeding to pass. Proposed
  validation of such fixtures is EQ-1 only; no new Recovery Scene was implemented here.

## EQ0-05 — Plant/control authority differs from physical 3D UAV recovery

- **HIGH for 3D/physical-target claims; LIMITED_VALIDITY for planar authority/virtual target.**
  The narrower assertion that the current actor independently commands Z is FAIL.
- Source: `navrl_task.py:transform_action_to_command:4953–5008` overwrites actor Z with
  `clip(4*(1−z)+I,±2.5)`; `step:5051–5057` retains raw policy Z in previous-action history.
  `velocity_control.py:LeeVelocityController.update:34–95` limits desired tilt, not actual attitude.
  `navrl_bars_env.py:env_config:110–126` creates a physical target only in the inactive physical
  branch; `navrl_task.py:_advance_target:8213–8224` directly writes bounded target pose.
- Actual values: altitude setpoint 1 m, full dynamic pursuer; E0/E2 target z=1 m exactly. Actual
  measured UAV tilt/speed can exceed desired limits. A yaw-only camera remains ideally level
  while the body tilts; no gimbal dynamics are represented.
- Impact: no vertical recovery authority is available to the current actor. Virtual target motion
  does not establish target thrust/drag/attitude/contact feasibility. A purely planar virtual-target
  hypothesis can retain these assumptions; 3D/hardware hypotheses need a separate lineage.
- Minimal reproduction/gate: independently derive action-to-command Jacobian (`dvz/da_z=0` at
  fixed state) and inspect recorded pose/quaternion; future separately approved hover/step/yaw
  tests log inner states. PASS is the declared authority and measured response, not “all commands
  equal actual motion.” Do not modify the frozen actor or remove its auxiliary Z history.

## EQ0-06 — LiDAR association and covariance are not identity-qualified

- **HIGH; NOT_TESTED for correct physical identity/covariance calibration.**
- Source: `navrl_perception.py:_associate_lidar_target:1925–2000`; association requires a live track,
  no camera detection and range near prediction, then normally corrects 3D using predicted
  bearing/height. `:1808–1816` target-like returns are carved from obstacle input.
- Actual: 187,114 LiDAR-only valid frames; camera target radius 0.15 m versus LiDAR radius 0.20 m;
  default range-only/silent correction options are off. Age resets on accepted LiDAR corrections.
- Reason: a bar at compatible range can satisfy the gate; predicted bearing/z are not new
  measured tangential information. An aligned target-centre range can have a −0.05 m radius
  convention bias. Neither implication quantifies the actual false-association rate.
- Impact: fused loss/expiry and obstacle carving affect approach/recovery; they cannot establish
  persistent identity. Track age is not elapsed camera loss; old tokens remain after deactivation.
- Minimal future test/gate: target-absent matching-bar controls, independent per-return provenance
  kept only in evaluator, hand-derived range-only KF covariance oracle. Zero target-specific false
  accepts on exact negative fixtures for a claim of identified target sensing; otherwise retain
  range-association semantics. Unobserved axes must not gain fictitious fresh measurement evidence.

## EQ0-07 — Riskcap and capture labels do not guarantee safe arrival

- **HIGH; FAIL for a stopping-distance guarantee; safe abort NOT_TESTED/NOT_RECORDED.**
- Source: `speed_governor.py:apply_speed_governor:465–475,505–531`; cap never falls below 2 m/s
  for a requested speed ≥2 m/s. Margin/brake/reaction values do not constrain this cap law.
  `navrl_task.py:compute_state_reward_and_terminations:6112–6125` gives capture same-step priority.
- Independent counterexample: v=2, reaction 0.1, brake 2.9608856678, margin 0.45 requires
  `0.45+v*0.1+v²/(2*2.9608856678)=1.32547 m`. The cap remains 2 at clearance 0.1 m. This fails
  the stated inequality without a rollout. The actual riskcap implementation matches its frozen
  contract; a collision-free guarantee was not promised by canonical docs.
- Impact: historical capture is close approach under its precedence rule, not collision-free
  completion. Future H-R safety must retain raw contact, capture, crash, timeout and abort separately.
- Minimal future test/gate: independent braking inequality on approach fixtures; 100 Hz contact
  chronology and a separate evaluator label for simultaneous capture/contact, without rewriting old
  outcome rules. A safety-qualified lineage requires feasible braking/clearance and a defined abort
  contract. Do not silently replace Policy F's riskcap or tune it after inspecting OCC outcomes.

## EQ0-08 — Local scene placement does not certify global passage

- **MEDIUM; NOT_TESTED.** Source: `asset_manager.py:_navrl_band_xy_spacing:346–432`,
  `navrl_task.py:_robot_spawn_inflation_radius_m:2296–2338`, spawn acceptance and initial-view sampler.
- Actual: centre-spacing touch/gap thresholds 0.4/1.6 m; box collision support radius 0.19799 m
  versus conservative spawn reserve 0.72365 m. In a predetermined env0/episode0 sample of all
  112 cells, 102 have intentional touching/overlapping bar pairs; smallest separate pair surface
  gap is 0.591925 m. These are compound walls, not by themselves a defect.
- Reason: maximum 0.8 m square bars at diagonal centre spacing 1.6 m may leave only
  `1.6−0.8*sqrt(2)=0.46863 m` corner gap. Centre spacing or a visible initial ray is not a global
  swept-body/dynamic route certificate.
- Impact: no-path frequency could affect safety/outcomes and future planner comparisons; its
  magnitude is unknown, not proven high. No scene is discarded or relabelled here.
- Minimal future test/gate: independent configuration-space obstacles with actual support and
  tracking/braking reserve, connectivity and exact near-boundary route checks. Certify each
  admitted future scene or mark it unresolved; disclose rejections rather than outcome-based filtering.

## EQ0-09 — Time, contact and physical model convergence remain unqualified

- **HIGH for physical-fidelity claims; NOT_TESTED.** Source: `env_manager.py:step:444–476`,
  `navrl_task.py:_runtime_physics_contract:2484`, trace frame contract and
  `compute_state_reward_and_terminations:6114–6123`.
- Actual: 100 Hz physics/Lee, 10 Hz actions/altitude loop/consumed observations, substeps=1;
  no real capture timestamp or 100 Hz state/contact/motor history. No ground plane or physical
  arena wall; height z<0.1 or z>4 and XY beyond [−1,41]² are termination rules. Bars are 3 m high.
- Reason: a curved relative path is replaced by a chord for capture; constant-acceleration sag
  `a*dt²/8` is illustrative (12.26 mm at a=9.81, dt=0.1), not a bound on all actual motion.
  Contact sampling/thresholds and motor integration can depend on timestep.
- Impact: observed timing is sufficient for registered 0.1 s intervals, not mechanical collision,
  landing, flying over bars or metric latency guarantees.
- Minimal future test/gate: approved 10/5/2.5 ms independent analytic and convergence fixtures,
  fixed 0.1 s action schedule, logged inner states/contact/forces, explicit sensor stage stamps.
  Proposed tolerances and resources are in [EQ-1 plan](EQ1_VALIDATION_PLAN.md).

## EQ0-10 — Structural actor separation is not a dynamic GT/RMS proof

- **MEDIUM; dynamic intervention/normalization invariance NOT_TESTED.** Source:
  `navrl_task.py:_process_obs_perception:6450–6539`, `runner.py:ExtractObsWrapper:70–90`,
  `navrl_perception.py:2179–2242`; [G3 review](supporting/g3/REPORT.md).
- Verified: no discovered direct target GT/semantic/GT-visibility argument to the executed actor
  path; privileged 8-D tail is in critic state only. Actor 898-D/logs/Policy F hash agree.
  Eight-pair G0 noninterference PASS is bounded support, not all privileged-data interventions.
- Impact: simulator GT is legitimate for rendering/evaluation, but dynamic aliasing or RMS state
  drift was not tested by source inspection. No observed GT leak is asserted.
- Minimal future test/gate: hold RGB-D/ranges/ego/track fixed; mutate privileged target/evaluator
  fields, reorder environments and resets; actor tensors/actions must be bit-identical and RMS
  hashes unchanged in eval. An independent fixed-input expectation is the oracle, not another
  call to the same observation helper. Any checkpoint loading needs explicit scoped authorization.

## EQ0-11 — Runtime and full-state replay evidence is incomplete

- **MEDIUM; LIMITED_VALIDITY.** Source: actual target-behaviour launcher's receipt writer,
  `tools/runtime_fingerprint.py` (not called by that receipt path), trace schema.
- Actual: source/import origin, BaseSimConfig/dt, checkpoint, PyTorch/Warp log excerpts are bound;
  Moving TF32 metadata exists, Static equivalent record is absent. Complete versions/flags,
  historical external-library byte hashes, RMS/RNG/full simulator snapshot are not recorded.
  The result action mode says `configured`; deterministic mean execution is source/YAML-bound.
- Impact: same records can be reanalysed; exhaustive bitwise simulator replay across stacks is not
  established. Hashing today's installed library cannot retroactively fill the historical record.
- Minimal future gate: receipt binds resolved configuration, code/assets/dependencies, seeds/RNG,
  action/normalizer state and platform; independent repeat/counterfactual prefix comparisons.
  Preserve missing fields as NOT_RECORDED; add a new receipt format only in a later approved unit.

## EQ0-12 — Legacy diagnostics and quota outcomes use different populations

- **MEDIUM; LIMITED_VALIDITY for secondary diagnostics; primary quota analysis PASS.**
- Source: `navrl_task.py:_export_bulk_eval_result:9123`, `_log_progress:9861`, ledger completion
  `step:5143–5159`; `occlusion_confirmatory_analysis.py:extract_cell:113–125` selects quota keys.
- Independent scan: raw legacy result snapshots total 57,475 completions, 55,478 capture /1,997
  crash /0 timeout. Quota is 57,344, 55,441 /1,898 /5. Distributions differ in 96/112 cells.
  Five quota timeouts can be absent from the earlier legacy window. This is not missing raw.
- Impact/gate: primary P2 conclusions use quota and remain unchanged. Do not combine legacy
  crash causes/governor samples with quota denominators. Before a future causal/safety comparison,
  independently join all metrics by (seed,cell,env,episode), gate exact episode-set equality and
  explicit exclusion of tails. Do not amend old result.json.

## EQ0-13 — Tiny geometry disagreements and interval resolution

- **LOW; LIMITED_VALIDITY.** Four online-box and three probe disagreements span five unique frames
  among 2,320,985 quota frames. [Independent boundary localization](supporting/g2/boundary_audit.json).
- Four grazing LOS boundaries and one FOV boundary have micrometre margins; numeric precision is
  consistent with the discrepancy, but the exact GPU arithmetic cause is not proven.
- Alternate recorded labels change three duration boundaries by one 0.1 s sample, but zero event
  counts/exposures; six frequency tests are unchanged. Hardware equivalence or zero error is not claimed.
- Future gate: independent high-precision face/pixel oracles with a declared boundary ambiguity
  band; no false-clear classification outside the band. Retain boundary cases and both labels;
  never patch historical classifications to manufacture agreement.

## EQ0-14 — External backup remains incomplete

- **MEDIUM for preservation; LIMITED_VALIDITY.** Existing raw and backups reside on one filesystem;
  external destination is pending. Git tracks compact reports/manifests, not full raw or checkpoint.
- Impact: hardware loss could prevent raw replication. This is not a scientific failure and does
  not block the authorized read-only audit. Nothing is deleted to free space.
- Minimal future gate: copy to the separately approved destination, independently recompute all
  manifest member hashes and archive SHA, and record an actual verified receipt. Until then,
  external backup remains NOT_COMPLETED. No assumption that merge/symlinks add raw to Git.

## Conditional impact on the user's provisional hypotheses

The definitions arrived during the audit; [their status](PROVISIONAL_HYPOTHESES.md) is
PROVISIONAL — NOT PREREGISTERED, not an effectiveness verdict. Each issue below affects the
ability to **test** the candidate. OCC data is design evidence only for these new hypotheses.

| Issue | H-R1 Tracking Continuity | H-R2 Safe Target Approach | H-R3 Active Visibility Contribution |
| --- | --- | --- | --- |
| EQ0-00 | Final valid-ID/time-bounded continuity endpoint open | Joint collision-free success/safety/abort rule open | Matched predictor/modalities/prefix and ablation endpoints open |
| EQ0-01 | Biased geometry can affect loss/association/state estimate | Fused obstacle position/clearance can be wrong | Viewpoint scoring cannot assume calibrated metric rays |
| EQ0-02 | Motion response changes observation trajectory | Braking/attitude/time-transfer validity unqualified | Movement cost/response must be the same qualified plant across arms |
| EQ0-03 | Clean pixel-code redetection has a ceiling, not realistic continuity | Uncertainty/range source differs from deployed sensors | No hidden synthetic-depth advantage; choose one sensor contract |
| EQ0-04 | Centre blockage is not necessarily a valid measurement/ID loss | Observation-instability eligibility and terminal competitors differ | Apparent LOS improvement is not necessarily valid reobservation |
| EQ0-05 | Virtual planar motion limits target-motion diversity | Physical contact/vertical manoeuvre feasibility not qualified | Current actor cannot choose climb viewpoints; planar scope remains possible |
| EQ0-06 | False association may reset age and inflate continuity | Target-like obstacle carving may affect hazards | All arms need identical sensor/association information, not GT identity |
| EQ0-07 | Tracking gains must retain crash/abort costs | Capture priority and cap floor fail a safety guarantee | More visibility must account for collision risk and movement cost |
| EQ0-08 | Unreachable viewpoints can make loss irreversible | Distinguish no feasible route from controller failure | Match route feasibility and pre-intervention scene across arms |
| EQ0-09 | Capture/delivery/expiry clock realism unqualified | Inner-step collision and stopping chronology absent | Unequal latency/action timing can confound active-versus-yaw ablation |
| EQ0-10 | Prediction/continuity metrics require actor information isolation | Uncertainty and safety decisions must use sensor-derived state | Dynamic GT firewall and fixed-information comparison are essential |
| EQ0-11 | Track/history/normalizer replay incomplete | Full plant/controller/RNG replay incomplete | Existing P2 traces do not certify a common branching state/prefix |
| EQ0-12 | Loss/reobservation denominator must use one eligible population | Legacy crash causes cannot use quota denominator | All baseline/active outcomes must join the same admitted episode set |
| EQ0-13 | Single-frame latency changes need an uncertainty band | No safety inference from ray-boundary agreement alone | Grazing viewpoint gains must survive declared boundary handling |
| EQ0-14 | Raw auditability vulnerable to disk loss | Same preservation risk, no outcome relabeling | Same preservation risk, no comparison result established |
