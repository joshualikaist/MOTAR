# S4R-1 amendment 2 results review — PASS_WITH_LIMITS

Reviewer: fresh independent Codex worker `/root/s4r1_results_review`, 2026-11-10. The inherited Codex deployment's exact model identifier is not exposed. This is **not an Opus review**. The reviewer received source paths and criteria, not the root's substantive result interpretation. No subagents, PX4/Gazebo launches, CUDA, full PUBLIC_CPU, repository edits, raw edits or commits were performed.

**Audit verdict: PASS_WITH_LIMITS. No required repair or amendment before recording the preregistered reading.** The experiment reading is **REPORTED_DESCRIPTIVE**, not a repeatability/equivalence PASS. Historical attempt01 stays INCOMPLETE_INVALID.

## Scope and evidence

Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `d6e83126f8671e64f3ff490a82b29205c83e4df6`, clean at review start. Read `AGENTS.md`, latest external handoff, current WORKLOG and Beads, the actual `tools/sitl_s4r1.py` and inherited integrity/validity/host/contact code, and:

- `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md` §§3–7;
- `docs/plans/prereg_sitl_s4r1_v1_amendment1_2026-11-10.md` (lossless bytes/contact arrival bounds);
- `docs/plans/prereg_sitl_s4r1_v1_amendment2_2026-11-10.md` (fresh folders, unchanged latency/first-invalid rules);
- `/home/fair/motar_tmp/s4r1_20261110/execution_approval.json` and `/home/fair/motar_tmp/sitl_amend2_20261110/execution/authorization.json` (same final HEAD/scope, sequential S4R-1 then S5, no retry);
- actual `smoke03_contact_on_env0/` and `attempt02/` under `/home/fair/motar_tmp/s4r1_20261110/`: attempt/final/episodes, session receipts, recording arrays/indexes/events/timeline, contact logs, ULog gzip/integrity/reconstruction and attempt02 reading.

The analyzed reading SHA-256 is `5f8d4a6e67a414d6feda51b98f85f3fa20a4874b87922fa9b69a8478db0a6cb0`. Audit scripts and machine-readable results are beside this report: `probe.py`/`probes.json`, `extra_probe.py`/`extra_probes.json`, `control_probe.py`/`control_probes.json`, `final_probe.py`/`final_probes.json`. All compute used aerialgym Python `-B`, CUDA hidden, user site disabled, OMP/MKL/OpenBLAS 1, cores 6,7,12,13, nice 10, a 4 GiB address-space cap and 30–120 s timeouts. The live run was already over and S5 had not started. Root clock retained: 13:27 UTC start, 17:27 wrap, 18:17 finish-by; never reset/closed by reviewer.

## Validity and non-vacuous recording

PASS for the preregistered measured scope. Smoke is COMPLETE, one valid env0/ON boot; it has no result reading. Attempt02 is COMPLETE with all 12 valid fresh boots in the exact alternating schedule, 3 repeats per scene/condition, no extra/replacement ledger member. Every operational validity check passes independently from row fields; no recording problems/warnings. Attempt LATE range is 0.064–0.128 s, below unchanged 0.15 s; wall time 46.04–103.49 s per boot and 736.25 s total, inside 180/1800 s. Kernel fault counts are 0→0, including every row. Source and final git snapshots match clean d6e8312. Per-boot idle checks and accepted GPU snapshots pass; snapshot evidence is not a continuous host profiler. The source's explicit allowed RustDesk process appears in GPU raw text, while disallowed-job lists are empty.

Independently hashed all 51 bound worktree source files for each unit, the bound external replay source, Frozen F checkpoint (`f7022139…`), and 71 distinct runtime resource paths. Scene/world bindings remain identical within each scene, and runtime cores match across every attempt boot. Seed 4501 is read back from every launched gzserver argv. Logger stop returns 0. Owned watchdogs have no abort/error; attempt peak RSS 1,731,203,072 B < 8 GiB. Actual attempt regular-file bytes 385,522,019 < 3 GiB. The reading's `output_bytes` excludes its own later-written 303,389-byte file, explaining the size difference.

Verified 4,293 recording-manifest entries (366,308,003 B), and separately every entry in the root's complete 208-file smoke and 4,222-file attempt SHA manifests in `execution/records_s4r1/`. All ULogs independently decompress to original sizes/hashes; strict framing and each of the six required topics' nonzero counts/control coverage pass. Every boot has steps>0, packets=steps+1, sequential packet choices, required received telemetry and ACKs; raw first/last array signatures match indexes. Event sequence IDs cover every event once; recorded MAVLink counts match raw events, with control phases present.

All 7 ON boots, including smoke, have successful floor-contact positive controls and nonzero raw floor evidence. All 6 OFF boots retain NOT_RECORDED semantics. Recomputed actual contact audits using the recorder's private API with `VEHICLE='navrl_quad_s4::'`; every saved audit matches. Independently parsed the time grid: attempt ON windows have 1,002–13,726 messages, zero missing/duplicate/off-grid steps and zero late arrivals. Non-vacuous controls made **only in this audit directory** accept an unchanged copy of ep001's log and reject a one-step deletion and one-step duplication as RECORDER_LOSS and RECORDER_DUPLICATE. No raw log was changed.

## Supported descriptive reading

All 30 same-scene pairs exist: per scene 3 within OFF, 3 within ON and 9 between. Independently recomputed every primary trajectory maximum/mean and fixed-step truth distance from `steps.npy`, all pair classifications/agreement and question-2 range counts. Independently compared stored arrays at step0 for all seven stages in all 30 pairs, including finite-only maxima and separate nonfinite mismatch counts. Re-ran complete per-stage comparisons for pairs (0,1), (2,3) and (5,8); every returned field matches the saved reading. This is not an independent reimplementation of all later-step fields for all pairs.

In **both scenes, all 15 pairs first differ at all seven stages at k=0: ALL_TIED**; their first above-tolerance step is also 0. Tied stages are unordered. Hover payloads already differ in 15/15 pairs per scene. These records therefore do not locate a first causal initiator within the control pipeline. No incomparable steps appear in the saved pair results; raw nonfinite sensor differences are counted separately rather than erasing finite magnitudes.

| Scene | Primary pooled within range (m) | Between inside/below/above | Outcome agreement OFF / ON / between |
|---|---|---|---|
| env0 | 1.463906–5.898559 | 9/9 inside; 0 below, 0 above | 1/3; 3/3; 6/9 |
| env1 | 1.991966–21.814642 | 8/9 inside; 1 below, 0 above | 3/3; 3/3; 9/9 |

Primary is maximum same-index truth-position distance over **each pair's common steps**, as preregistered. env0 contains five crashes and one capture (OFF repeat1, ep005); env1 six crashes. Matching terminal labels do not imply matching trajectories. Secondary observation-first-above reading is [0,0] within range with 9/9 inside in each scene; it supplies no additional discrimination here.

Fixed-step counts must travel with the claim: env0 k10 is 6/9 inside (3 above); k50 is 4/6 inside (1 below/1 above), with only 4 within pairs. Env1 k0 is 7/9 inside (2 above), k10 7/9 (2 below), k50 1/3 (2 below), k100 2/3 (1 below), each later range using only the available pairs. env0 k100 has no between pair; env1 k200–500 has one between pair but no within pair. Those readings are NOT_AVAILABLE; neither scene has a k599 comparison.

## Limits to retain in the result record

No determinism, statistical equivalence, causal subscriber effect, historical-recorder noninterference, camera-only/OAK-D/new-vehicle/S5 or real-flight qualification follows. ON includes the floor-control/liveness work as well as subscription. n=3, dependent pair statistics, differing common trajectory horizons and variable initial states limit generalization. The fixed seed's control of sensor-plugin noise remains NOT_TESTED. Pre-heartbeat messages, sensor production counters and per-packet setpoint acceptance remain NOT_RECORDED.

Two concrete reporting cautions require no source change under this freeze:

1. Large stamp maxima can include the plugin's uninitialized target timestamp `-1`, not a physical sensor delay. Example: ep000 step0 has target_t=-1/count0, ep001 target_t=36.112/count1; the pair's 37,084,000 µs maximum reflects that initialization boundary. Preserve raw fields and separate relative stamps, absolute offsets and reference-free readings; do not call this 37 s latency.
2. No new boot reached 600 steps (attempt maximum 578). The generic reading limit text says the 600-step path was first exercised live in attempt01; it must not be used to claim complete live 600-step coverage from this attempt. State actual observed coverage; retain raw reading bytes unchanged. The CPU synthetic 600-step evidence is a separate scope.

Git Traceability: branch `camonly/dev`; session base `fe7daed4a521bde2ded617f66cdcfd7934d50f72`; reviewed HEAD d6e83126f8671e64f3ff490a82b29205c83e4df6; New Commit SHA NONE (read-only/source freeze); Changed Files only this external audit directory; Tests bounded independent integrity/numeric/contact controls PASS; Working Tree clean at audit start and end; Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity checked as described, no modifications.
