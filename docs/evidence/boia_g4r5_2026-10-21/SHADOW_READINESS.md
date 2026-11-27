# G4-R5: actual inputs and the shortest path to disconnected shadow

2026-10-21 KST. Audited source **45a7b6839e9b3855cf0a847b362d18dac29af315**.
Static audit and small CPU probes only. No simulator, GPU, checkpoint load, raw
replay, calibration, production/controller/governor change or online activation.
This is a **DESIGN specification, not GPU authorization or preregistration**.

| Item | Current status | Source | Missing / next action |
| --- | --- | --- | --- |
| Real sensor inputs | Measurements exist; current BOIA copies only a subset | P.observe; T._process_obs_perception; access.read_online | Bind clocks/frames/lineage; copy existing outputs once. |
| B03 sparse coverage | Ray evidence only, gaps UNKNOWN | WarpLidar.initialize_ray_vectors | No measured 3D volume certificate. |
| B04 identity ambiguity | Range/appearance can support hypotheses; no calibrated identity | P._detect_rgbd/_associate_lidar_target | Preserve hits and competing explanations. |
| Evidence adapter | Existing copied whitelist + new CPU contract probe | access.py; shadow_readiness_probe.py | Actual observer/metadata producer NOT_IMPLEMENTED. |
| Frozen F continuation | Fresh-process matched-prefix structure exists | P03 launcher/RNG/isolation/analysis | Full arbitrary snapshot restore absent; new profile not qualified. |
| Capture/crash telemetry | Outcomes and substep contact primitives exist | T.compute...; E-L0 | Pre-mask causes, joint event/phase/owner need explicit recorder. |
| Shadow readiness | Specification fixed; execution HOLD | This document; validation.json | Minimal evaluator wrapper + CPU parity checks, then separate GPU approval. |
| Git integrity | Base and protected assets preserved | validation.json | Local commit only; G5 HOLD. |

## 1. Source and field inventory

Abbreviations are actual paths, all relative to repository root:

- **T**: `aerial_gym/task/navrl_task/navrl_task.py`.
- **P**: `aerial_gym/task/navrl_task/navrl_perception.py`.
- **D**: `aerial_gym/task/navrl_task/navrl_detector.py`.
- **LC**: `aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py`.
- **TC**: `aerial_gym/config/task_config/navrl_task_config.py`.
- **Access**: `motar_eval/boia/access.py::read_online`, lines51–124.

Classification vocabulary below: **A**=AVAILABLE_AND_ONLINE_PERMITTED content;
**PVT**=AVAILABLE_BUT_PRIVILEGED; **OFF**=AVAILABLE_OFFLINE_ONLY; **M**=MISSING;
**U**=UNCERTAIN. Availability of simulator own state is the permitted own-state
proxy, not a claim of real onboard pose accuracy. Evaluator-facing sensor content
can be permitted even when not currently copied into BOIA; truth cannot.

### Camera

| Field | Actual producer / function | Shape / unit | Timestamp and frame | Class / BOIA access | Missing contract |
| --- | --- | --- | --- | --- | --- |
| RGB | D.render_raw_rgbd:916–991 → T._process_obs_perception:6461/6502 | N×3×90×160 float[0,1] in pinned raster | No capture/arrival tags; camera raster | A / NO | Explicit detached sensor-only copy, lineage binding. |
| Depth | Same; T:6503 | N×90×160 metres | Unit-ray Euclidean surface range, **not optical Z**; unstamped | A / NO | Legacy obstacle field bilinear40×24→160×90 is not exact per-pixel first hit. |
| Intrinsics | P.__init__:754–783; D:342–356 | fx/fy/cx/cy pixels; FOV87°×58° | Static configured camera/detect domains | A / YES camera values | detect_fx differs if decoupled; no current calibration version. |
| Camera-to-vehicle transform | P:784–794; D:555–559/827–831 | Nominal t=(.10,0,.03)m,R=I | **Level yaw-only vehicle** frame, not tilted body | A nominal,U physical / translation YES | Capture-pose/error binding; private renderer perturbations are not online calibration. |
| Depth convention / measurement | D kernels:75/110–118/132; P:1738–1748 | Normalized ray, surface range; adds radius.15m for centre estimate | Local +x forward,+y left,+z up; right/down pixels mean−y/−z | A nominal / convention not explicit | Centroid average+radius is not true centre geometry. |
| Detection confidence | P._detect_rgbd:1726–1734; diagnostics:2404 | N scalar[0,1] | Delivered detection, no capture tag | A / NO | Segmenter score×pixel support, **not identity probability or calibrated interval**. |
| Detection bbox | D.detect:1069/1083–1089; optional P:1646–1668 local computation | N×4 pixels if that path runs | No current-frame tag | M in Frozen perception output / NO | Frozen actor898 never calls D.detect; last_bbox is stale/other-path, not fresh evidence. |
| Capture and arrival timestamps | No public producer in render_raw_rgbd/observe | None | M | M / NO | Relative latency indices do not publish clocks. Host callback time is not capture time. |
| Capture-time pose | Nominal own pose; latency pose buffers P:1266–1268 | N×3,N×4 | Not exported with identified clock/calibration | U / NO bound pose | Renderer actual origins/orientations are PVT; do not read them as measured pose. |

Pixels are simulated using scene geometry, as any renderer must. That fact alone
does not mean reading the returned RGB-D leaks GT. Reading target_mask/depth,
target IDs, renderer target pose, detect_bbox or E-L0 labels directly would.
The reduced DETECT_CHANNEL has a separate synthetic-equivalence contract; it is
not a new measured identity source and is excluded from the proposed first shadow.

### LiDAR

| Field | Actual producer | Shape / unit | Timestamp / frame | Class / BOIA access | Missing contract |
| --- | --- | --- | --- | --- | --- |
| Directions/resolution | warp_lidar.py::initialize_ray_vectors:68–92; LC:46–61 | 4×72 unit rays, azimuth180→−175°/5°, elevation20/10/0/−10° | No stamp; yaw-only sensor,+x forward | A / YES angles,fixed elevation assumption | Zero-width rays; no physical footprint/divergence. |
| Range limits | LC:64–72 | .2–12m pinned F; class default4m | Static config | A / YES far range | Runtime actual layout/range must match, not inferred from defaults. |
| First-hit values | warp_lidar_kernels.py:203–249 → WarpSensor.apply_range_limits:220–243 → T._lidar_distance_m:6013–6017 | N×1×4×72 normalized→N×288 metres | No capture tag; per ray | A postprocessed range / YES | Getter is sanitized, **not lossless raw first-hit provenance**. |
| No-hit / invalid / saturation | Kernel NO_HIT1000→far fill; getter NaN/±Inf→1,clamp[0,1] | far12m; near negative fill→0 | No typed event | U / numeric only | Far may mean no hit, true limit or invalid; pre-getter pixels still post-Warp clipping. |
| Timestamp | No public producer in WarpLidar.capture:234–245 / WarpSensor.update:184–218 | None | M | M / NO | Clock/generation binding absent. |
| Mount | LC:92–100; WarpSensor.update:184–205 | Nominal zero offset/rotation; yaw_only_attach=True | Level sensor at vehicle centre | A nominal / implicit | Hardware extrinsics/error bounds absent. |
| Scan pose | WarpSensor.sensor_position/orientation:198–205; current own pose | N×1×3,N×1×4 | Not bound to capture timestamp | U as packet / current pos+yaw YES | Cannot silently treat current pose as delayed scan pose. |
| Uncertainty | LC.sensor_noise:102–108 disabled,std0 | No calibrated per-ray intervals | None | M calibrated / NO | Noiseless simulator configuration is not zero physical uncertainty. |
| Semantic identity companion | segmentation_pixels; kernel:249–251; T:7557–7563 | N×1×4×72 IDs, target ID50 | Simulator provenance | PVT / FORBIDDEN | Must not enter online association/BOIA. |

### Tracker

| Field | Actual producer | Shape / unit | Time / frame | Class / BOIA access | Missing contract |
| --- | --- | --- | --- | --- | --- |
| Target estimate | BatchedConstantVelocityTracker P:648/670–694 | N×6 position m,velocity m/s | World; once per observe, no absolute stamp | A / YES | Actual observation-generation ID. |
| Covariance | P:649/696–723,Joseph update | N×6×6 mixed position/velocity units | Same update generation | A / YES | Q/R are model priors; no NIS/NEES or coverage qualification. |
| Camera confirmation | P:2201–2235/2395 → T:6483 | N bool delivered pixel detection | May be delayed | A / YES | Not identity-correct camera confirmation. |
| Fused update | P:2258–2287/2394–2396 | N bool camera OR LiDAR | Delivery generation | A / NO fused flag | Active flag is different. |
| Measurement age | P:651/677/722–723 | N seconds since accepted fused correction | Resets on camera OR non-silent LiDAR | A / YES | Not camera-only age or identity age. |
| Prediction age | No separate public field | None | Predictor advances dt per call | M / NO | Neither step_count nor fused age is a capture clock. |
| Reacquisition | No native identity-correct event | CameraFalse→True can be derived | Requires per-env episode history | M native,A derived / NO | Flag transition is not identity-correct reacquisition or GT occlusion. |
| Association | P._associate_lidar_target:2033–2135 | Beam/range residual,heuristic confidence | No capture tag | A heuristic / NO | Semantic ground truth and E-L0/H-R3 labels are PVT. |
| Miss/expiry | P:670–694,optional LiDAR correction | predict+Q,active/age; expires age>5s | Nominal dt.1s | A / age,active YES | LiDAR may keep a track young without camera reacquisition. |
| Camera identity clock | P:1092–1095/2274–2283 | N identity_age seconds/update_source | S-L1C/Cv2 only | A conditional / NO | Absent Frozen off lineage; not calibrated correctness. |

### UAV, control and execution metadata

| Field | Actual producer | Shape / unit | Time / frame | Class / BOIA access | Missing contract |
| --- | --- | --- | --- | --- | --- |
| Position / velocity | BaseRobot.init_tensors:37–43; T:6467–6471 | N×3 world m,m/s | Latest physics | A own-state / YES | Sensor-capture pose binding. |
| Orientation / angular velocity | BaseMultirotor.update_states:287–294 | Full robot_orientation vs yaw-only robot_vehicle_orientation; xyzw,rad/s | Latest physics | A / yaw-only+body angular velocity YES | Arbitrary tilted-body SE(3) not supplied. |
| Actual command | T.transform_action_to_command:4953–5008 | N×4 vehicle m/s,yaw rad/s | Pre-physics | A / NO | Access takes normalized proposed a_F, not actual governed/PI command. |
| Previous action / history | T:5051–5057; P:1129–1144/2174–2182 | N×4; robotN×5×10,targetN×5×16,obstacleN×5×8×12 | Stride/step schedule,no sample clocks | A / NO histories | XY feedback can be governor-rewritten; z history differs from PI physical command. |
| Controller / motor | base_lee_controller.py:29–76; T:4997–5002; motor_model.py:91–157 | Gains,buffers,wrench,motor thrust,time constants,z integral | Control/physics | A internal / NO | Not a full calibrated plant snapshot. |
| Governor | T:1764–1784; speed_governor.py:438–525 | N×2 feedback; speed,clearance,TTC,scale | Command phase | A / NO | Riskcap function stateless; surrounding feedback/history is stateful. |
| Env/episode/decision IDs | sim_env.sim_steps; num_task_steps; optional obs_dump_ep_idx T:1964–1973/4710–4715; Kroc episode_index | Integers | Reset-dependent bookkeeping | OFF existing counters / dedicated BOIA packet M | External reset-aware IDs permitted; global step is not sensor clock. |

Configured Frozen F profile is actor898,72×4/12m LiDAR,control .1s,600 actions if
the evaluator explicitly pins600; **class default is300** (TC:465). Current horizon
uses>=N (T:315–322), not the old601-action off-by-one. Physics .01s×10substeps=.1s
must be recorded from actual runtime. Current host runtime is **NOT_RUN** here.
Configured dropout.3/depth-noise.02/RGB-noise.015 apply only with perturbations
enabled (P:1593–1641; T:6477; TC:277 defaultsFalse); knobs are not proof of active
noise. Record the actual flag. Off/A/B/Cv1/Cv2/D are separate sensor lineages;
Cv2 inherits A/B/C, not D qualification. First shadow pins **off, equal camera and
detect resolution, zero declared detection latency**, without silently changing F.

## 2. What current evidence can support: B03 and B04 together

**B03:** finite valid ray segments before a return can be labelled
OBSERVED_FREE_UNDER_ASSUMPTIONS only for the static zero-width nominal ray model.
Returns are POSSIBLY_OCCUPIED; inter-ray gaps, rear/out-of-range space and unaged
dynamic occupancy are UNKNOWN. Physical 3D free-volume certification is
UNSUPPORTED. At6m, adjacent5° ray chord is0.52343m and10° is1.04587m. A radius.1m
disk at6m midway between5° rays is missed by both. A minimum cross-section,
beam footprint, multiview support and bounded pose/time/motion assumptions would
be necessary to expand evidence; none is inferred from sector interpolation.
Forward87°×58° camera rays may add nominal support, not rear coverage. Bilinear
depth pixels are not newly measured rays; S-L1A is a distinct correction lineage.

**B04:** actual LiDAR target sphere radius is **.20m** (LC:83–90,kernel:239–249),
whereas RGB-D radius is.15m. For unit ray u and sphere centre c,
`t=u·c−sqrt(R²−|c_perp|²)`. On axis at6m,R=.20 gives **5.8m exactly**. A foreground
surface at5.8m gives the same single range. This proves ambiguity of that range /
predicted-centre summary, **not identity of every RGB-D pixel and beam in arbitrary
scenes**. Camera origin(.10,0,.03) and its different proxy radius prevent direct
range subtraction: ideal range to centre is hypot(5.9,.03), surface≈5.750076m.
Earlier radius.15 toy examples are not the actual LiDAR proxy contract.

Registered image depth and a measured candidate mask can sometimes reject a
foreground interpretation, but only with common-ray/pose/time/error support. Current
confidence is not identity probability. Retain target/foreground/false-association
hypotheses or UNKNOWN; never remove occupied first hits using semantic50 or GT.
Current `boia/geometry.py::exclude_target:25–38` preserves valid hits; target self-
blocking is an explicit unresolved trade-off. Real association intervals, future
occupancy support, calibrated loss and F-continuation error are missing, so utility
must **ABSTAIN**, even with apparently useful geometry.

R1's full selector may return NO_CERTIFIED_BACKUP because no independent nominal
safety record was supplied. Shadow logs must separate **utility ABSTAIN**, safety
INPUT_CERTIFICATE_UNAVAILABLE, optional selector outcome and actual emergency
NOT_ASSESSED. Do not fabricate a safety certificate to obtain a preferred label,
or call baseline F physically unsafe because the research packet is incomplete.

## 3. Frozen F continuation: reuse, not a new branch simulator

Existing P03 runs separate fresh processes, with FixedNRandomness / selected-mesh
isolation and strict prefix comparison (`tools/h_r3_p03_launcher.py:92–148`;
`tools/h_r3_p03_analysis.py::matched_prefix:73–97`). It does not restore a full
simulator snapshot. Its prior pass was seed4801,N128,moving160,600process-steps,
sensor-lineage off, not a guarantee for N1/new seeds or all profiles.

| State / API | Available reuse | Missing qualification / exact boundary |
| --- | --- | --- |
| Actor + RMS | eq1d_fixture_launcher.build_player:399–439/find_rms:451–460; P03 digest36–38/154–155 | Reloading fixed weights is not dynamic-state restore. |
| RNN / observation | navrl_transformer_network.is_rnn:152–153=False; player get_action:184–244 | Explicit histories/previous actions still matter. After intervention, retain each branch's actual history. |
| Perception / delay | eq1d_telemetry.snapshot:144–148/restore:162–181; P:983–1024/1221–1273 | Limited Python tensor/scalar copy for observation re-evaluation; no complete RNG/nn.Module/physics restore. |
| UAV/PhysX | IGE_env_manager.write_to_sim:620–632 / refresh588–594 | Root/DOF writes do not restore contact/solver/motor/controller state atomically. |
| Controller / governor | z integral,motor thrust/time constants,Lee buffers; riskcap function | Include all feedback and motor histories; P03 digest is incomplete for arbitrary continuation. |
| Target / geometry | Pose/velocity,waypoint/heading/speed/pattern T:1143–1153/8226–8233; Warp diagnostic snapshots | Target intent and physics/mesh/BVH/cache state are not a complete restore bundle. |
| RNG | h_r3_p02_rng._scope:147–162,step:280–289,action:292–296,waypoint:255–277 | Fixed-N/profile streams; no automatic private-generator/noise/new-N inheritance. |
| Terminal/reset | T:5065–5107/5167–5182 | Need episode/reset IDs,counters,queue phase,done masks and consumed observation. |

The minimal future counterfactual is **fresh-process matched-prefix replay**, then
one interval F or g, followed by actual unchanged F on each branch's own measured
history. Same seed alone is insufficient. Verify actor/RMS,own/target/obstacle,
controller/motor/governor,tracker/delay/history,RNG and terminal state fingerprints;
digest equality is still not a complete PhysX restore proof. Full arbitrary
snapshot restore is NOT_IMPLEMENTED/NOT_QUALIFIED. This branch work is **not a
prerequisite for disconnected availability shadow**, which estimates no actual gain.

## 4. Capture/Crash/Timeout: preserve semantics, separate telemetry

T.compute_state_reward_and_terminations:6071–6107 builds contact/height/OOB masks;
6119–6123 uses relative swept segment distance<.5m. At6124 capture clears crash.
The exported crash cause6133–6141 is already masked. Horizon>=configuredN and
timeout excludes resolved capture/crash (T:315–337/5065–5088). In the pinned600×.1s
profile the episode cap is60s; process-budget censoring is a separate event.

**Aliasing:** `terminations` aliases obs_dict['crashes'] (T:1577), also bound as
EnvManager collision_tensor (env_manager.py:172). T:6304 overwrites it with
capture OR crash. Reading it after compute is **not pre-mask collision telemetry**.

| Required telemetry | Exact safe observation seam / semantics |
| --- | --- |
| Raw capture condition | T:6119–6123; preserve existing successes before reset; source-equivalent evaluator swept calculation if needed, never rerun reward. Eval-only GT permitted here. |
| Pre-mask collision/contact | Existing adapter.after_physics at T:5058–5063 before compute; clone collision data there. E-L0 ContactChain:198–212 collects every physics substep. |
| Pre-mask height/OOB | T:6071–6107 before masking; future evaluator uses copied pre-compute own state/runtime bounds and source-equivalent predicates, explicitly labelled recomputed if not native export. |
| Simultaneous capture/failure | Join raw capture with independent contact/height/OOB predicates by immutable episode/action ID. Do not infer from final crash code. |
| Terminal reason / timeout | T:5065–5088 existing resolved outcome; retain original priority and add joint-event fields only. |
| Reset boundary | T:5145–5147 episode-end hook before reset5167–5175; assign new episode ID before post-reset packet. |
| Pre-terminal observation | E-L0 staged/consumed counters:282–315/357–359 identify the observation used by the terminating action. It is not a newly rendered end-of-step pre-reset frame. Post-reset returned obs cannot substitute. |
| Physical owner / actual command | Shadow driver owner=F, record actual transformed/governed/PI command T:4953–5008/5051–5057, never infer ownership from a proposal. |

E-L0 after_physics:318–379 and on_episode_end:388–438 already accumulate threshold
contacts and report capture_with_contact/contact-based collision_free_capture.
That label means no recorded UAV contact above threshold; it does **not** prove no
height/OOB failure at capture. Strict prospective collision-free capture requires
episode-wide substep contact accumulation plus separately retained height/OOB
failures, alongside the original capture metric. Threshold .05N and body/target
lineage must be explicit. Old unrecorded causes remain **NOT_RECORDED**; no rates
are reconstructed here. No production telemetry change was made in this task.

## 5. Minimal passive packet and output boundary

Reuse `access.read_online` for detached copies of permitted own state,belief,
LiDAR and a_F. Extend only an **evaluator-owned collector**, not actor898 or
production, to copy existing `perception.observe` allowed inputs/diagnostics.
Call the original observe exactly **once** and return its outputs untouched; do
not call detection,association,tracker.step or observe again for logging. Additional
range-only depth_range_pixels copy can retain pre-getter invalidity; it is still
post-Warp clipping. Record FAR_OR_NO_RETURN/INVALID/NEAR_OR_CLIPPED/finite-return
statuses and preserve values, never convert them to proved free volume.

Proposed immutable packet: env/episode/action/decision and observation-generation
IDs; phase; source/config/checkpoint/lineage; raw RGB-D and postprocessed LiDAR plus
available pre-getter validity; own pose/velocity; tracker state/cov/age; delivered
camera/fused/association diagnostics. Camera and LiDAR each need capture_sim_s,
delivery_sim_s,capture pose/frame/version and optional host arrival monotonic time.
Do not compare different clock epochs or invent missing capture time from NOW.
For pinned zero-latency synchronous simulation, explicit wrapper scheduling can
produce **DECLARED_SIM_SCHEDULE** tags; that is not a measured hardware timestamp.
Unknown fields carry None/reason. Runtime must attest equal raster/detect domains,
actual intrinsics/offset/ranges/yaw-only frame and active perturbation flags.

There is no task/model/renderer/evaluator object handle in the research packet.
Truth lives in a separate evaluator-only record, joined **after** BOIA by immutable
IDs. Never forward generic P03 snapshots: they contain privileged roots/target/mesh
data; its arm launcher also replaces non-B0 actions and uses GT silhouettes.

Output is **diagnostics/IDs only**, no command-shaped array or action-writer API.
Original F remains actual owner. Missing utility means ABSTAIN; C02 may be logged
MODEL_ONLY/UNQUALIFIED without granting execution. The prototype in
shadow_readiness_probe.py checks declarations on synthetic copied inputs; it is
**not an installed runtime adapter**, nor a sensor/pose/timestamp proof.

## 6. Proposed first GPU shadow: bounded specification, approval required

**Purpose:** field availability/phase/abstention/latency and Frozen F noninterference.
Real WAIT/INTERVENE effectiveness, physical safety and capture improvement are
NOT_TESTED. Unsupported utility must not be replaced with synthetic loss tables.
Initially skip the full planner/paired-value computation; log SKIPPED_UNSUPPORTED
and ABSTAIN. If almost all rows abstain, report that denominator, not a success rate.
An existing N128/K20 C02 CPU single-call observation took3.863113s
(`../boia_g4r_2026-10-20/model_receipt.json`); the Master Plan also retains the
full-planner13.2449s observation. Neither measures N1 GPU cost or WCET. These
motivate N1, decimated diagnostics and watchdogs, not assumed real-time planning.

| Contract | Proposed fixed choice / guard |
| --- | --- |
| Run | BOIA-SHADOW-Q01, exploratory qualification only; no confirmation/sample expansion. |
| Baseline | Same frozen F898/checkpoint,governor/controller,RGB-D+LiDAR off lineage,zero detection latency/equal camera-detect160×90. Source/config guard must reject unexpected inherited knobs. |
| Population | **N=1**, design seed4901, moving E2_direct_l2_v1,70bars,initial visible10m; no N128 run. Seed4701–4732 stays reserved for historical H-R3 confirmation planning;4901 has no match in the searched source/plans/evidence. This qualification seed is never a confirmatory sample. |
| S0 | GPU boot/observer safety smoke≤20actions; wall cap2min. Source/config/import/shape/phase checks; failure stops campaign. |
| S1 | Two fresh processes: OFF versus passive ON,≤200actions each and≤1ended episode; task episode cap600; each wall cap5min. Process cap before episode end is censoring, not timeout. |
| Total cap | ≤12min GPU wall time across stages (≤0.2GPUh conservative reservation); no automatic extension. N2 expansion requires review/new approval. |
| Cadence | F retains .1s simulated control; shadow sample every5actions (2Hz simulated),explicitly not a10Hz/5ms claim. IDs distinguish sampled versus unsampled decisions. |
| Scope | Packet copy/schema/ABSTAIN logs; full planner SKIPPED if inputs unsupported. Optional nominal profiling must be separately named, never actual mission utility. |
| Latency | Host perf_counter for copy/schema/log pipeline; GPU CUDA-event sections where used plus end-to-end synchronization/transfer costs. Separate cold compile,warmP50/P95/P99/max and deadline misses. No timing off a nonexistent computation. |
| Slow/OOM | Per-sample watchdog2s; stop after3consecutive samples>500ms. Per-process wall watchdog; catch OOM/device faults,savepartialreceipt/manifest,stop. No retry with silent smaller model/N. |
| VRAM | Actual peak NOT_MEASURED. Planning ceiling min(6.5GiB,totalVRAM−.75GiB), total-device plus torch peaks; preflight actual capacity required. No linear N scaling assumption. |
| Disk | Preflight≥3.3GB; stop<3,000,000,000bytes. Bounded raw/log≤256MB,≤8RGB-D snapshots; checkpoint read/hash only,no duplicate. |
| Output | NEW persistent `/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01/`; exclusive creation,no overwrite. Current task does not create this run directory. Compact source/config/checkpoint hashes and raw manifest after approval. |

OFF/ON parity must compare exact actor input,a_F,actual governed/PI command,own/
target state,sensor/history and resolved outcomes over the common assigned prefix,
with evaluator truth isolated from BOIA. No changed action, extra perception step,
RNG consumption, normalization update or termination semantics is allowed. Stop at
first mismatch; report INTEGRITY_FAIL without relaxed thresholds/excluded rows.
The CPU action-copy test does **not** establish this runtime parity. Existing N128
P03 evidence does not qualify the new N1/profile automatically. Perform S0 and
the first OFF/ON pair as qualification gates; analyze availability/latency only
after exact parity passes. CPU output disconnection must pass before GPU approval.

Required log fields: immutable IDs/phase; modality clocks/frames/source versions;
availability/validity and camera detection (not identity truth); estimated loss /
occlusion UNKNOWN when unsupported; hypotheses/reasons; a_F and actual command;
proposal status/ID; utility ABSTAIN and optional research-selector outcome;
supported opportunity UNKNOWN; model/physical status; stage-specific latency;
contact/joint/terminal/censor/reset and F owner; source/checkpoint/config hashes.
GT LOS/identity may only appear in separate evaluator records; adding such a
recorder needs explicit implementation scope and noninterference qualification.

### Gate separation and shortest critical path

| Class | Must satisfy / may retain unresolved |
| --- | --- |
| MUST_BEFORE_SHADOW / P0 | Implement tiny evaluator observer/header/phase IDs; strict whitelist and isolated truth; F output/action-path invariance CPU tests; source/config/checkpoint/lineage guards; terminal/reset joins; bounded output/watchdogs/VRAM/disk/latency plan; separate user GPU approval. Actual runtime metadata/parity remain qualification questions in that approved run. |
| MAY_DEFER_UNTIL_AFTER_SHADOW / P1 | Physical B03/B04 resolution,calibrated association/detection/mission value,F-continuation branch model/MC bounds. They remain UNKNOWN/ABSTAIN; no supported gain claim. Full arbitrary snapshot restore is unnecessary for passive packet observation. |
| MUST_BEFORE_ONLINE_CONTROL / P2 | Evidence/utility/coverage qualification wherever used; physical C02,executable backup,controller/plant/tracking/error bounds,action ownership/hand-back,deadline/endgame and separate user approval. Shadow permission is not G5 release. |

**Next single Codex unit:** add one evaluator-only observer/logger using existing
`perception.observe` boundary, `access.read_online`, Kroc/E-L0 callback patterns and
ContactChain as needed; CPU-test exactly-once observation,immutable packet,F-command
identity,episode/reset joins and clock/domain tags. Do not copy P03 intervention
arms or build a branch simulator. Freeze source+runtime contract/preregistration,
then ask separately for the bounded N1 OFF/ON GPU run. That is the shortest path;
physical C02 or a new perception network is not a prerequisite for passive logging.

## 7. Verification and fourteen answers

New standalone CPU probe14/14PASS; independent reviewer14PASS plus7narrow controls
on the pre-final schema. Final root review corrected the source identifier to
40-hex Git commit versus64-hex config SHA-256, added swapped-width negatives and
reran14/14PASS. The follow-up independent recheck hit the review agent's usage
limit; **final identifier-format delta is NOT_INDEPENDENTLY_REREVIEWED**. The
earlier review is not presented as covering different final bytes.
The combined BOIA CPU suite was run once at the local-commit checkpoint:
252PASS,0failures/errors,15.095s. Tier2 access/research regressions are included
in that single Tier3 run; no second duplicate run. PUBLIC_CPU NOT_RERUN; its four
known errors remain unresolved. This is not an all-green public-test claim.
It reuses access fake fixtures, actual copied whitelist and R1 missing-proof
behavior; no production/Isaac import. Header validity and hashes are declarations,
not actual packet binding; phase rejection is not a tested runtime reset cache;
raw-validity flag does not reconstruct a sensor mask; cloned action is not real
OFF/ON parity. All runtime flags remain NOT_TESTED. Initial14tests passed but the
first receipt serialization exited1 due to a relative path; resolved path fixed
and finalrun exited0. No negative research result was overwritten.

1. Available: RGB-D,postprocessed LiDAR,own state,tracker mean/covariance/age and delivered sensor diagnostics, with the limitations in§1.
2. Missing: published capture/delivery clocks,current Frozen bbox,calibrated identity/error intervals,complete pose-at-capture and mission utility/provider.
3. GT risk: renderer semantics,DETECT_CHANNEL,task target/critic/evaluator snapshots; strict packet whitelist and isolated evaluator namespace are mandatory.
4. B03/B04: valid nominal rays and registered surface support can constrain hypotheses; one5.8m return cannot identify its source.
5. UNKNOWN: ray gaps,unaged dynamics,unregistered/deferred observations,identity and unsupported future utility.
6. Continuation: fresh-process common-prefix architecture exists; arbitrary full simulator snapshot restore is not qualified.
7. Missing: complete controller/motor/intent/queue/history/RNG/terminal and PhysX/cache restore/validation, not just seed/model weights.
8. Collision-free capture: pre-compute/substep contacts+height/OOB,existing raw capture and immutable episode joins; never use aliased post-compute crashes as premask evidence.
9. Minimum shadow code: evaluator observer/header/recorder only; no production/planner activation or gain model.
10. Current shadow blockers: runtime packet/time/domain/phase producer and output-disconnection/parity wiring are not implemented/qualified; GPU approval absent.
11. Online blockers: physical safety/backup/plant/deadline/evidence+utility/endgame qualification; G5HOLD.
12. Novelty: evidence-supported selective interception timing remains CANDIDATE/NOT_VERIFIED; passive availability shadow does not confirm it.
13. Git: base preserved; only protected raw links initially untracked; explicit local commit, no push/merge and no data deletion.
14. Next approval: the single evaluator-only implementation+CPU unit above, followed by a separate N1 GPU execution approval after source/prereg freeze.

Exact tests, source inventory SHA, preservation and Git scope: validation.json.
The Master Plan remains the living research entry; this document is its R5 evidence.
