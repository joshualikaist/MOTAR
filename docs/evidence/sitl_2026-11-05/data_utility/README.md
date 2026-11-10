# Existing-data integrity, coverage and utility audit r1

2026-11-10; source worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `504eb4e139af2edff254af88a8481a75be4ee0a1`. Read-only audit by Codex delegated worker `/root/data_utility`; no simulator or GPU. Fixed `PROTOCOL.md` preceded computation. This is a post hoc descriptive audit: no tuned threshold, changed gate, synthetic fill or causal/real-flight claim. Independent review is **PENDING**; this report is not its own independent approval.

The collected outcomes are worth retaining and reusing. All four checked historical manifests match, the CAM quota rows are complete, and the SITL outcome/step ledgers have their intended coverage. The important limitations concern what those data can explain. They do not record enough synchronized perception/contact/transport detail to identify a failure mechanism. Existing ULogs recover substantially more attitude, truth and controller history than the flat traces alone suggest. Recollect only missing synchronized streams in a bounded repeatability diagnostic, after using the existing ULogs; do not repeat all prior outcome evaluations.

## Integrity and coverage measured directly

| Historical manifest | Files | Bytes | Missing / SHA or size mismatch |
|---|---:|---:|---:|
| CAM-K1-E | 322 | 306,571,245 | 0 / 0 |
| CAM-K2-E | 572 | 491,429,712 | 0 / 0 |
| CAM-K2-E3 | 490 | 404,581,600 | 0 / 0 |
| S4F1 | 2,021 | 581,012,414 | 0 / 0 |

The E3 independent review (`review_summary_v3e_r1.json`) had already checked the old 894 and new 490 files, 3,726 comparison leaves and checkpoint bindings. This audit rehashed all four pertinent original manifests and directly checked the actual quota and SITL schemas/counts; it does not merely rely on that review. S3b/S3c/S4 top-level attempt/final/ledger/reading hashes also match their compact execution-record pins. Their step files and ULogs are freshly bound in `ORIGINAL_MANIFEST.json`; that fresh binding alone is not a claim that every such file had a prior committed historical hash. The new original manifest binds 5,329 distinct source paths.

| CAM primary new eval data | Cells | All ledger rows | Quota rows | Outside quota, retained |
|---|---:|---:|---:|---:|
| K1-E | 36 | 96,263 | 73,728 | 22,535 |
| K2-E | 60 | 156,813 | 122,880 | 33,933 |
| K2-E3 | 48 | 124,687 | 98,304 | 26,383 |

Every primary cell has 128 environments × first 16 completed episodes = 2,048 quota rows. No duplicate episode identity appears within a cell; every quota row joins its KROC episode summary; recomputed counts agree with the original quota summary. Zero primary eval byte-duplicate ledger groups were found. Each family has two duplicate-byte groups across *all* raw ledgers because deliberate identity/repeat controls agree; those control copies do not add primary sample size. The equally pooled K2s1/K2s2 crash totals (3,647 each) are not duplicate episode files.

`in_quota=false` rows are excluded by the preregistered primary rule, not missing, invalid or deleted. Legacy `result.json` crash causes/governor summaries use a different aggregate completion window and must not be spliced into quota-row mechanisms. One F66 quota timeout exists (1/24,576), whereas the K1 policy arms and all K2/E3 primary arms have zero timeouts. A rounded 0.00% is not an exact zero for F66.

| SITL data | Scheduled / ledger rows | VALID | Primary outcome coverage |
|---|---:|---:|---|
| S3b A / B / C | 122 / 30 / 10 | 122 / 30 / 10 | A 122 capture; B 27 capture, 3 crash; C 10 capture |
| S3c iris / delay1 | 40 / 40 | 40 / 40 | iris 40 capture; delay1 37 capture, 3 crash |
| S4 ground / raw | 40 / 20 | 39 / 19 | ground 20 capture, 17 crash, 2 timeout; raw 5 capture, 11 crash, 3 timeout |
| S4F1 four cells | 40 each | 40 each | lit/rendered 22/14/4; emissive/rendered 24/16/0; lit/unrendered 27/11/2; emissive/unrendered 30/10/0 (capture/crash/timeout) |

All 462 scheduled episodes exist with unique indices. Actual `steps.npy` rows: S3b 6,887; S3c 3,642; S4 12,445; S4F1 19,722. All have width 27 and exactly their ledger's step count. S4/F1 additionally have the same row counts at width 16 in `sensor_steps.npy`. No missing step file or row-count disagreement was found. All scheduled outcomes, including INVALID ones, are preserved. The two S4 exclusions are ground env37/index34, capture, LATE 0.160 s; raw env20/index57, timeout, LATE 0.152 s. They are excluded from the VALID primary denominators only.

## ULogs: useful reconstruction, with two historical truncated tails

All **462 original SITL ULogs** were streamed, hashed and structurally scanned using stdlib code (162 S3b, 80 S3c, 60 S4, 160 F1). All contain the ten checked channels: estimated and ground-truth attitude/position, rate setpoint, vehicle command/ACK/status, allocator status and actuator motors. No ULog `O` dropout record was observed in the parsed prefixes. That is not a proof that no message was lost upstream.

460 ULogs parse to complete records. Two F1 gzip files decompress with valid gzip checksums and match the recorded *original decompressed SHA*, but end in a partial final ULog Data record:

| F1 episode | Outcome terminal PX4 sample | Partial tail | Meaning |
|---|---:|---|---|
| ep004, emissive_rendered/env1 | capture, 43.336 s | `vehicle_local_position`, 44.336 s; expected169 payload bytes, actual131 | historical incomplete tail ~1 s after outcome |
| ep139, lit_rendered/env37 | capture, 43.324 s | `estimator_innovation_variances`, 44.332 s; expected154, actual118 | historical incomplete tail ~1 s after outcome |

Both original decompressed lengths are 9,216,000 bytes. The parsed truth stream extends past the outcome in both. These are **LIMITED_VALIDITY for complete cleanup-tail/losslessness claims**, not missing/corrupted manifest files or grounds for removing the outcomes. Preserve the negative structural finding. A tolerant offline analysis must explicitly report the incomplete tail and use only complete records; do not edit the originals.

One actual representative ULog per family is summarized with full topic schemas and approximate message rates in `metrics.json`. Ground-truth quaternion/body rate are about100Hz, ground-truth local position about50Hz, estimated local position125Hz, rate setpoint50Hz, allocator5Hz and motor10Hz. These are measured representative publication-message rates, not configured guarantees for every episode. `vehicle_attitude_groundtruth` includes `timestamp`, `timestamp_sample`, `q`; `vehicle_local_position_groundtruth` includes xyz, velocity and timestamps; command ACK and status retain native publication timestamps. Flat traces do not save full attitude, but **attitude is recoverable from ULogs**. The historical S4 prose saying attitude was not logged should be read as a flat-trace limitation, not absence from all available evidence.

## Usefulness and limits of inference

| Data | Fixed-grid outcome comparison | Failure mechanism | Real-sensor transfer |
|---|---|---|---|
| CAM-K1-E/K2-E/E3 | Useful within registered checkpoint/grid/treatment contrasts; labels unchanged | Insufficient: no contact-time per-quota hitbar/token/FOV/clearance trace; whole training recipe/combined treatments remain mixed | NOT_TESTED; simulated OAK-D model is not bench qualification |
| S3b/S3c | Useful for eligible fixed G8 scene outcomes and the scoped S3b rule; S3a stays NOT_EQUIVALENT, physical collider vs logical termination limits remain | Partly reconstructible from ULogs, steps and world; no recorded actual contact/transport timing; no causal attribution | NOT_TESTED |
| S4/S4F1 | Useful VALID condition rates and fixed historical factorial tests; weak repeatability limits scene-by-scene explanation | Trajectory/attitude/controller/sensor-age reconstruction useful; missing frames/observations/transport/contact prevent exact first-divergence reconstruction | NOT_TESTED |
| dev08/AGQ | Descriptive observed-prefix record: 2 excluded smoke +40 main; 4 later geometry members cannot establish generalized efficacy | Four prior ULogs support footprint/tracking proxies; later geometry has actual contact+send/arrival but did not reproduce historical failures | NOT_TESTED |

The CAM inference unit is the **12 density × evaluation-seed blocks per checkpoint pair** (K2 nominal:6), conditional on those checkpoints and that grid. There are only two training seeds. The 294,912 new CAM quota outcomes are not 294,912 independent training replicates. The KROC `initial.stream` is `UNPAIRED_STREAM`; identical episode IDs in different arms do not establish matched physical episode trajectories after policy-dependent reset/global-RNG evolution. Historical bootstrap/t intervals are block-conditional and do not establish population-wide training-recipe uncertainty.

Directly recomputed S4 `ground` versus F1 `lit_rendered` on the39 scenes VALID in both: **17 same, 22 different**, expected chance agreement17.0769, κ=−0.00350877. S4 has20 captures,17 crashes,2 timeouts; F1 has21 captures,14 crashes,4 timeouts. Ten crashes become captures, eight captures become crashes. Similar rates therefore do not establish deterministic per-scene replication. S3b A→C is10/10, but it uses idealized sensing and does not qualify Gazebo sensor runs. Preserve the historical F1 paired sign/Holm tests and their **neither primary reached** label. Weak scene covariance/power and unverified boot/render phase constrain paired mechanistic interpretation; this audit does not replace the preregistered analysis with a newly chosen test or claim that all causal controls are qualified.

## Exact schema gaps and what can be reconstructed

- CAM example: `camk2v3e_20261110/raw/eval_01/V3s1/E2_obstacle_aware__205bars__seed4501/episodes.jsonl` has completion step, identity, quota flag, outcome and minimum relative *target* distance. The joined `kroc2027_episodes.jsonl` has episode visibility/detection counts, speed summaries and initial scene/state hashes. It has no per-quota contact-bar ID, contact-time obstacle token, hit-bar FOV, pursuer obstacle clearance or aligned pose/action trace. `swept_min_distance_m` is target distance; `target_swept_min_obstacle_clearance_m` is *target* clearance. Neither is pursuer collision clearance.
- CAM settings are **present elsewhere**, not missing: `kroc2027_summary.json` records66°×54°, translation[0.1,0,0.03]m, max range20m; the KROC episode rows record `rl_dt_s=0.1`. The E3 `realveh_receipt.json` and cell log bind depth MinZ0.4m, invalid encoding, noise/hole settings/seeds and depth-validity rule. `cell.log` depth-validity summaries aggregate calls/frames/invalid/near pixels. These cell aggregates and training hit-token diagnostics cannot supply contact-aligned per-quota measurements.
- SITL `steps.npy` has27 columns: k/PX4 sample time, estimated xyz/velocity/yaw, policy action, governed command, target xyz, distance/clearance/compute/visible, truth xyz/yaw. It saves sampled command generation but not serialized packet send/arrival/acceptance association. `sensor_steps.npy` has16 columns: k, lidar/rgb/depth/target timestamps, colour/shadow pixels, visible flags, aggregate differences, removed return counts, Gazebo sim_now and PX4 sample time. These can recompute sampled ages, decision trajectories and controller discrepancies.
- Actual S4/F1 model SDF/runtimes bind configured sensor metadata (20Hz LiDAR/RGB/depth,160×90 cameras, horizontal FOV1.51843645rad, physics250Hz). These are configured metadata; distinct received/used/dropped frame counts and acquisition→arrival→decision lineage are NOT_RECORDED. Original images/depth/LiDAR arrays and exact actor observations were not saved. Rerendering a shadow from geometry cannot reproduce historical noisy Gazebo frames.
- ULogs enable estimated/truth orientation/velocity/controller timeline and modeled footprint overlap proxies with documented coordinate/time conversion. Publication timestamp and interpolated estimate vs truth do not give exact physics-contact simultaneity or network delay. `vehicle_command_ack`/`vehicle_status` native records can narrow actual mode history but cannot recover old MAVLink RX arrival time.
- There is actual contact/transport evidence **elsewhere**: `/home/fair/motar_tmp/agq_geometry_diag_20261106/attempt01/{00..03}_*/recording/contacts.jsonl` has physics `t_ns`, collision names, points, forces/depths; `mavlink_sends.jsonl` has call/args and monotonic entry/return; arrivals have type/src/fields/seq/recv. All four originals were inspected and bound. A known floor contact is recorded, so this was non-vacuous. These four later diagnostic members do not fill the missing streams in the old20dev08 pairs or anyS4/F1 episode. Both geometry pairs remain **INCONCLUSIVE_NOT_REPRODUCED**.

## Minimum next recording contract and work priority

1. **Reuse current outcome records.** Keep the registered primary/excluded windows and all negative results. Use existing ULogs/worlds/steps first for attitude, tracking and modeled-contact-proxy timelines. Treat old per-scene explanations as hypotheses. Do not require a full outcome recollection or OAK-D hardware to finish this offline work.
2. **Gazebo repeatability diagnostic:** preserve the exact old lit/rendered stack for OFF and record every boot's source/checkpoint/SDF/world/plugin hash, seed/noise/initial state/reset/order, simulation/start phase and readiness. At each decision, save exact raw RGB/depth/LiDAR, observation, requested and governed command, frame IDs and hashes; save acquisition/native simulator/PX4 timestamp, local monotonic acquisition/arrival/use/send association, repeated/stale/skipped/dropped/ACK counters. Store full ULogs with per-topic counts, rates, native timestamps, dropout/parse validation; retain actual contact events with collision IDs/forces/physics time and preflight floor positive control. Keep contact logging OFF/ON explicit to measure recorder interference. A bounded12boot diagnostic can collect these missing channels; it does not replace old outcomes or itself establish broad repeatability.
3. **S5 trigger→actual-mode contract:** explicit setup, pre-trigger, triggered assessment, recovery and cleanup phase boundaries. Log trigger/input and exact serialized TX request with sequence/correlation ID; log inbound ACK/status/heartbeat/manual-control fields, source/sequence, native timestamp when present, monotonic arrival, age/cadence, duplicates/drop counts. Distinguish generated/requested/ACKed/actually reported mode and actuator/vehicle response; log command/stream stopping and resumed stream. Preserve mandatory hashed ULog inventory with command/ACK/status/failsafe/controller/motor/estimated/truth topics. Cleanup AUTO_LAND must begin after the assessment window and cannot count as the observed failsafe response. If actual contact is outside S5 scope, write NOT_RECORDED; do not substitute a clearance proxy.
4. **Real-sensor transfer remains NOT_TESTED.** User reports OAK-D hardware unavailable. Later bench collection needs raw RGB/depth/confidence or invalid mask, synchronized timestamps/rate/drop metadata, device+firmware/calibration/intrinsics/extrinsics/settings and fixed near-MinZ/distance/texture/lighting cases. No synthetic fill or simulated sensor rule can supply this evidence.

## Reproduction and limits

`audit_data.py` streams original manifest/raw ledgers, parses NPY headers/data without numpy, and scans ULog subscriptions/records without pyulog; `enrich_metrics.py` adds original dev08/AGQ checks, tail localization, duplicate classification and the utility matrix. Run both in order with CUDA hidden, PYTHONNOUSERSITE1, single-thread math, taskset6,7/nice10,8GiB address-space cap and timeout≤600s. `metrics.json` contains per-cell and per-ULog results; `ORIGINAL_MANIFEST.json` binds originals; `MANIFEST.json` binds audit artifacts. The ULog scanner checks structure, topic presence and logged dropout records, not every message field, time calibration or all-topic continuity. This audit does not independently recompute every historical confidence interval or re-review execution authorization. Existing independent review disclosures continue to apply.

Kernel matching fault count was0 before/after. No owned compute remains after the scripts finish. Only this external audit directory was written by this worker; raw/source/checkpoint bytes were not modified. Parent owns independent review, Beads/WORKLOG integration and any approved local record commit. No remote push or main merge.
