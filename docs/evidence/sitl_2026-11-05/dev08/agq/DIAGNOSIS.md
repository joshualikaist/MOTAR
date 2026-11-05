# Dev08 failure mechanism inspection (motar-agq)

Read-only post hoc CPU diagnosis of the four existing205/env2 and205/env5 members. No new simulator, GPU, flight, gain selection or outcome measurement. Existing outcomes remain unchanged. Analyst: Codex `/root/agq_diagnosis`, inherited GPT-6 family (exact runtime model variant NOT_RECORDED). Root inherited finish-by2026-11-05T18:35:22Z; no new time allowance.

## Finding and fix location

A concrete platform mismatch exists at the vehicle collision model. Installed `navrl_quad.sdf` has a0.28×0.28×0.08m base collision box PLUS four rotor collision cylinders (centers±0.13m, radius0.128m, length0.005m). Maximum horizontal rotor reach when level is `sqrt(0.13²+0.13²)+0.128 =0.311848m`. Historical terminal `box_hit` instead tests center-to-bar distance against0.14m. The latter is an inscribed disc of the base square and is **not** a conservative enclosure of either the base box or rotors. The parent's independent training-URDF inspection finds only the base collision box. A new opt-in SDF with only the extra rotor collisions removed is therefore the appropriate first platform repair. Keep all historical files and verdicts frozen.

The actual contact stream/impulses were NOT_RECORDED. The analysis below identifies **modeled physical footprint overlap**, not a recorded contact. Abrupt uncommanded angular motion aligns with those overlap intervals and precedes the largest position/height discrepancies. This supports a contact hypothesis much more strongly than interpreting a terminal snapshot discrepancy as a primary estimator fault; it does not establish contact causality.

## Time basis and geometry

PX4 ULogs contain50Hz `vehicle_local_position_groundtruth`,~125Hz `vehicle_local_position`,~100Hz `vehicle_attitude_groundtruth`/angular-velocity truth, controller setpoints, actuators and5Hz allocator status. All four ULogs have zero recorded dropouts. Their `timestamp` values share PX4 `hrt` publication time. PX4 simulator code stamps incoming HIL truth at receipt; this is not exact physics simultaneity. Estimator sample times and prediction horizon differ. Original outbound HIL MAVLink has zero time_usec and its actual stream phase remains unknown.

Truth's local reference was initialized before the airframe settled; the runner chose a later ground origin without persisting it. The10–13s ground interval in each ULog has exactlyzero x/y/z spread and is used to reconstruct that origin (env2 z0.7409973m; env5 z0.7399902m in ULog NED). Estimates retain their native original local origin; no initial estimate subtraction is silently applied. We report both absolute discrepancy and change from the firstfive20ms samples' mean offset. Initial offsets are only centimeters; the candidate env2 multi-meter divergence is growth.

Estimated state is linearly interpolated onto truth publication times. Footprints use the **full3D truth quaternion**, FRD→NED→recorded MOTAR world transform,8base-box vertices, and72angular rotor-rim vertices on each end cap. A convex horizontal hull is intersected with the recorded bar rectangle by separating axes, requiring overlapping vertical spans. Rotor polygonization is inscribed, not conservative. Projected intersection and vertical overlap are necessary but not sufficient for a3D contact. Geo position quantization, publication phase and model/world numerical precision remain limits. No synthetic contact is substituted for an unrecorded stream.

## Event order

All times below are elapsed from each member's gate start, in seconds. Thresholds are post hoc descriptive definitions, not acceptance criteria. `Yaw5` means first|actual body yaw rate|>5rad/s. `z0.8` means first reconstructed truth height<0.8m. Error0.5 means first same-publication-clock position discrepancy>0.5m. “Logical end” is the actual archived checkpoint outcome.

| Member | First modeled rotor overlap | Yaw5 | z0.8 | Error0.5 | Logical end |
|---|---:|---:|---:|---:|---:|
|205/env2 native|0.940|1.280|1.260|6.180|7.400, bar crash|
|205/env2 candidate|2.736|2.736|3.336|7.076|8.320, bar crash|
|205/env5 native|5.728|5.768|5.988|6.668|7.000, altitude crash|
|205/env5 candidate|none on sampled trace|none|4.080|none|13.000, capture|

### Env2 native

First rotor-overlap proxy is bar13 at36.664s (elapsed0.940), center clearance0.279m, truth height0.960m, discrepancy0.138m and tilt28.06°. Prior to overlap minus40ms, max discrepancy0.132m, max tilt30.40°, and max body rates[3.697,2.166,2.211]rad/s. Actual roll first exceeds5rad/s at elapsed1.000, then tilt exceeds45° at1.120 and truth height falls below0.8m at1.260. First discrepancy growth>0.3m occurs later at1.480s. Native survives that disturbance; later overlap proxies occur around5.86s (bar54) and7.30s (bar69), before the archived bar termination7.40s. The allocator first reports both torque and thrust unachievable at1.336s; only2of32 sampled post-first-overlap allocator records report thrust unachievable. Do not generalize that sampled count into a continuous guarantee.

### Env2 candidate

The first rotor-overlap proxy is bar169 at38.436s (elapsed2.736), clearance0.225m, height0.924m, discrepancy0.070m. At that same publication sample actual body yaw rate is+7.618rad/s versus PX4 rate setpoint−0.564rad/s. Prior to overlap minus40ms, max discrepancy0.203m and max body yaw rate2.221rad/s. A second overlap interval at elapsed6.696–6.896s (bar181) includes base-box projection overlap at6.756/6.776s. Major position divergence follows: discrepancy>0.5m at7.076,>1m at7.316, and2.665m at8.316, before the archived logical termination8.320. Thus the large discrepancy **does grow before the final logical collision**, but **after earlier modeled overlap/angular disturbances**. It is not evidence that an initially diverged estimator caused the first physical encounter. Under a diagnostic±20ms truth-publication shift, the final discrepancy remains2.655–2.665m; plausible stream phase cannot account for a2.66m separation at these speeds. Exact estimation error and the instant of physical contact are still not established. The rotor-only repair may not remove the later base collision; it changes the earlier trajectory disturbance and must be measured.

### Env5 native versus candidate

Native's first rotor-overlap proxy is bar168 at41.244s (elapsed5.728), clearance0.239m, truth height0.959m, estimate height0.994m, discrepancy0.217m, tilt28.54°. At41.244 true body yaw rate is−1.337rad/s;40ms later it is+6.562rad/s versus desired−0.146rad/s. By41.404 actual yaw reaches8.698rad/s. Height<0.8m and discrepancy growth>0.3m first occur at41.504 (elapsed5.988), after this angular excursion. The sampled allocator first reports torque unachievable at41.396 (elapsed5.880), with unallocated yaw torque−0.939; thrust remains achieved in all6 sampled post-overlap allocator records. Consequently this record supports a yaw torque/response disturbance and delayed height response; it does **not** support claiming collective thrust saturation as the native env5 cause. The high-rate trace contains no base-box-overlap proxy for this member, making native env5 the cleanest rotor-geometry intervention scene.

Candidate takes a different path after the early turn. It has no modeled footprint overlap on the sampled trace, no body angular rate exceeding5rad/s, maximum position discrepancy0.243m and truth height minimum0.763m. It still exhibits actual tilt overshoot to62.23° despite a45°tilt setpoint ceiling: dynamic response remains imperfect independent of modeled overlap. Its capture does not prove any one of the three changed gains caused the rescue. Native and candidate commands, observations and paths all evolve in closed loop; exact boot/noise repeatability was not recorded.

Generic tracking mismatch is not contact-specific. An axis|actual−requested rate|>5rad/s appears before the first overlap in candidate env2(elapsed0.556s) and native env5(4.128s), consistent with lag around changing commands. Those thresholds alone cannot diagnose a physical impulse. The strongest onset evidence is abrupt measured angular motion with low contemporaneous setpoints at the larger-than-logical collider boundary, followed by height/discrepancy growth.

## Commands versus measured tracking before modeled overlap

`command_projection.py/json` provides an additional bounded offline diagnostic. Starting from the recorded truth position and full3Dorientation at0.20s before first modeled overlap, hold current height/orientation and either the latest generated checkpoint velocity or actual truth velocity constant for0.30s (5ms samples). This omits dynamics, future policy changes and exact send/receive time; a missing projected overlap is **not** a safe-command verdict.

| Scene at200ms before first proxy | Generated command world vx/vy (m/s) | Actual truth vx/vy (m/s) |0.30s projected overlap using command|0.30s projected overlap using actual velocity|
|---|---|---|---|---|
|env2 native|[2.00,0.02]|[1.50,−0.93]|none|rotor/bar13 at+0.210s|
|env2 candidate|[1.37,1.45]|[2.18,1.50]|rotor/bar169 at+0.295s|rotor/bar169 at+0.190s|
|env5 native|[1.30,−1.52]|[0.64,−2.66]|none|rotor/bar168 at+0.170s|

At100ms before nativeenv5 overlap the same diagnostic still separates request[1.58,−1.22] (none within0.30s) from actual[0.77,−2.35] (rotor/bar168 at+0.090s). Thus native's measured momentum/tracking difference already contributes to its approach before the angular excursion; “all commands were unsafe first” is not supported. Candidateenv2's requested direction itself enters the enlarged rotor envelope in this simplified projection, so perception/governor geometry is also relevant. Candidateenv5 supplies a cautionary counterexample: near its minimum-clearance pass, the commanded constant-velocity projection intersects a rotor envelope while the measured-velocity projection does not, yet no sampled actual footprint overlap occurs. Lag can sometimes avoid an encounter, and these projections cannot rank controller gains or establish collision-free commands.

## Minimum discriminating next test

Change **only collision geometry**: native controller/F/input/seed/stream/timing/world fixed, OFF reproduces historical SDF byte-for-byte; ON removes four rotor collision nodes and preserves base box, masses, inertia, motors, aerodynamic constants, sensors and PX4 gains. First use205/env5 native, where the existing modeled overlap is rotor-only, and include205/env2 native if the bounded unit permits. Use fresh run folders and paired OFF/ON order fixed before execution; repeat because physics/noise boot repeatability is unverified. Do not retune gains concurrently.

Record actual Gazebo contact events with collision/link names and physics timestamps, raw inbound/outbound MAVLink with receipt/send monotonic times, source uORB truth timestamps, all relevant full-rate estimates/attitudes/rates/controller setpoints/allocator/motor signals, and the exact command and observation slices. A contact logger must cover ground and bar contacts, positive-control known-contact detection, and report drops. Test predictions: removing rotor geometry should remove early rotor/bar contacts and associated angular impulses if that mismatch is causal; persistence of the impulse without contact refutes this explanation and directs a separate no-obstacle command-replay/plant-response test. Success does not imply formalS3b acceptance; unresolved S3a observation mismatch and strict600/601 endpoint inputs remain prerequisites.

## Artifacts and reproducibility

- `analyze_v3.py` produces `analysis_v3/diagnosis.json`, per-case checkpoint/timeline and20ms series JSON directly from the existing raw ULogs, checkpoints, world and pinned scene specs.
- `analysis_v3/diagnosis.json` SHA-256:774e215fc9b563eb19fdbd1e9eb56e496ea860e87bff57badf3c0ea5a2aae9dd. It binds the four ULogs/checkpoint/world files and local PX4 transform/stream source, plus the analysis code itself.
- `supplement_v3.py` produces `presentation_v3/`: four compactCSV/NPZ time series, `events.csv`, `supplement.json`, standalone12panel `failure_timeline.png`/`.svg` (visually inspected).
- CSV logged-trajectory vx/vy are empty until the first velocity-control setpoint is published; the native position-hold uORB fields are NaN, not fabricated zeros. NPZ retains that optional-signal NaN; all other CSV fields are finite. Primary compact diagnosis JSON forbids nonfinite JSON.
- CPU: CUDA hidden, PYTHONNOUSERSITE1, OMP/MKL1, affinity6/7/12/13, nice10,8GiB address-space cap, timeout≤600s with kill10s. Existing `px4` Python environment supplies pyulog1.3.0/numpy/matplotlib. Kernel matched fault count0 before/after; no owned process remains. No installation or package/source/raw mutation.
- Failed intermediate probes remain preserved: first analysis produced NaN tracking fields through0×NaN matrix propagation; v2 excludes undefined vertical-request slots. First presentation probe rejected optional position-hold velocity NaNs; v2 represented them explicitly. v2 allocator count inverted uint8 flags and produced negative counts; v3 uses explicit `==0`, giving nativeenv2=2/32, candidateenv2=0/28, nativeenv5=0/6 sampled thrust-unachieved records. Only v3 primary/presentation outputs are used here.

No causal or formal gate verdict is changed by this post hoc diagnosis. The concrete fix location is the platform collider; estimator settings and gain selection should wait for the geometry-only discriminating measurement.
