# S5 safety v2 amendment 2 — independent results review r1

**Verdict: PASS_WITH_LIMITS. Actual fixed reading: PASS_SITL. No required source/protocol repair before recording this bounded result.**

Reviewer: fresh-context delegated Codex reviewer `/root/s5_results_review`; inherited Codex model, exact deployment identifier not exposed. This is **not an Opus review**. Date: 2026-11-10T15:22:41.924516+00:00. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, reviewed HEAD `d6e83126f8671e64f3ff490a82b29205c83e4df6`. Root shared clock was read and retained; start 13:27 UTC, wrap 17:27 UTC, finish by 18:17 UTC on 2026-11-10. No repository/raw edit, live rerun, GPU job, full PUBLIC_CPU run, commit, or subagent was performed by this reviewer.

## Authority and exact scope

I read the actual AGENTS.md, shared handoff/clock, latest WORKLOG/Beads state, `tools/sitl_s5_safety.py`, its relevant `sitl_s3b_formal.py`/`sitl_owned_session.py` paths, original `docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md`, and both dated amendments. I reviewed the actual source and raw records independently of the root's conclusion.

Current `execution_approval.json` exactly matches source SCOPE and final HEAD; both attempts' approval SHA matches that file. The immutable execution authorization at `/home/fair/motar_tmp/sitl_amend2_20261110/execution/authorization.json` records the user's “어어 하나씩 진행하자” and sequential S4R-1 then S5 scope. S4R-1 attempt02 finished 2026-11-10 14:49:09.294252 UTC; S5 smoke03 started 14:59:13.352419 UTC, attempt01 started 15:03:02.408978 UTC and completed 15:11:45.096546 UTC. This supports the authorized sequence. No replay or replacement member appears in either S5 schedule.

This result is only the configured safety-behavior checklist item in **PX4 v1.14.3 / Gazebo Classic 11, navrl_quad airframe 10090 stand-in, fixed parameter set v1, scripted setpoints, one fresh-rootfs boot for each scheduled scenario**. The stand-in is 0.25 kg, whereas the proposed real vehicle is 1.1 kg. It contains no learned policy, camera GPU rendering, real RC receiver, or hardware trial, and is not an upgrade of all of S5 or a real-flight qualification. Live discrimination remains **NOT_TESTED**: all broken-response/missing-data controls in this audit are offline mutations, not a live known-bad PX4 trial.

## Integrity, resources and completeness

- smoke03: COMPLETE, fixed `N0 N0R N0J`, all three VALID and nominal. Recomputed smoke gate has no problem; smoke itself has no result reading (VOID as required if read as a result).
- attempt01: COMPLETE, fixed `N0 OBL GFH GFV OVJ OVR RCL DLL KIL FDS`, all ten VALID and passed. Recomputed `reading()` exactly equals the original `reading.json`, including every criterion, no integrity problem and no recording warning.
- Both recorded start/end trees are clean at the approved HEAD. All nine source bindings were independently rehashed against actual current bytes. All 68 runtime binding-core digests match across all thirteen boots, including mavutil and both installed v10/v20 `all.py` sources. All 81 distinct bound file paths (including per-episode worlds) still match size and SHA. The first connection in each invocation begins v10 and pins after switching v20; subsequent connections begin v20, without changing the source set. These are the bound runtime sources, not an exhaustive qualification of every OS/shared-library dependency.
- Independent verification of the root's full regular-file packages: smoke03 39 files / 92,274,649 bytes; attempt01 124 files / 258,449,061 bytes. Path sets, sizes and every SHA match actual files; trace/session/ULog ledger hashes also recompute. Packages: `/home/fair/motar_tmp/sitl_amend2_20261110/execution/records_s5_safety_v2/`.
- Per-boot receipts show fresh distinct roots, 240 s deadline and 8 GiB owned-RSS cap, no watchdog abort or body exception, successful logger stop before PX4 teardown, joined RX thread, closed link and no leaked/unreaped processes or cleanup error. All boots end disarmed. Highest recorded owned-session RSS is 550,961,152 bytes. See the distinct nonzero Gazebo shutdown exit below; “normal cleanup” does not mean every exit code is zero.
- Recorded idle samples pass the fixed runner-core rule, and recorded GPU lists contain only permitted RustDesk. Kernel regex counter is 0 throughout both units. This is a sampled/recorded idle-host check plus the operator's procedural exclusion of other work, not continuous proof of every host process. The 3 GiB launch free-disk floor is enforced in the executed launch path; an exact launch-free-byte value is NOT_RECORDED in S5 `attempt.json` and I do not invent one.
- Offline first-invalid controls stop exactly after member 1 (two rows, no member 2 call) for an explicit invalid record and for changed binding. Positive genuine records are accepted. This verifies the bounded control behavior offline, not a live failure-discrimination result.

## Actual receive and log coverage

All thirteen real receive pins are `(system, component)=(1,1)`, autopilot 12, vehicle type 2, while pymavlink's wire target is `(1,0)`. Raw-source heartbeats exactly reproduce each frozen assessment list. I decoded the first/last available packets of each relevant telemetry/ACK/text/manual/RC type in every frozen window through installed real pymavlink and checked source, type and full payload equality; this is selected-packet wire verification, not a claim to decode every recorded packet again. All required periodic raw payloads are finite. Native LPN/attitude/HIL timestamps are monotonic and satisfy the 0.10 s bound independently of the stored prior-LPN receive stamp.

Maximum frozen-window simulated receive gaps (boundaries included):

| Stream | Applicable boots | Maximum gap s | Fixed bound s |
| --- | ---: | ---: | ---: |
| LOCAL_POSITION_NED | 13 | 0.024000 | 0.10 |
| ATTITUDE | 13 | 0.024000 | 0.10 |
| HIL_STATE_QUATERNION | 13 | 0.024000 | 0.10 |
| HEARTBEAT | 13 | 1.008000 | 1.60 |
| RC_CHANNELS | 4 | 0.088000 | 0.20 |
| MANUAL_CONTROL | 6 | 0.088000 | 0.25 |

Maximum wall-clock receive position-stamp age is 0.049513 s; sampled truth age is 0.025258 s (both below 0.10 s). Recorded phase real-time factors range 0.998507–1.000732. Explicit pre/injection/observation-end/cleanup markers and frozen count boundaries are present and ordered; later cleanup AUTO_LAND/disarm does not enter the outcome assessment.

All thirteen actual ULogs were reparsed with installed px4-env pyulog and strict framing from the original files; complete inventories exactly equal the recorded inventories. No malformed tail, corruption flag or dropout appears. Required common and scenario-specific topics are nonzero; flags have all four required fields. vehicle_local_position maximum native gap is 0.008000 s; maximum window flag gap is 0.600000 s. Coverage reaches the observation end in eleven boots; KIL ends at its first disarmed heartbeat 40.320 s with final position stamp 40.872 s, and FDS at 38.512 s with final position stamp 38.920 s, following the preregistered SDLOG_MODE=1 rule. Logger flush succeeds before teardown and hashing.

## Responses independently checked against raw packets and flags

All five pre-window heartbeats per attempt are armed OFFBOARD and live pre-window flags remain false. Flag scenarios' first actual live flag is the scenario's own flag alone within the fixed cause window; RCL and DLL are distinct manual-loss and GCS-loss flags. Numeric checks below derive actual heartbeat modes/times, text, selected input echoes, ACKs and spatial/drop quantities from raw receive records, with ULog flags independently reparsed. Each reached response has three actual persistence heartbeats at offsets 0/1/2 s inside its fixed 2.5 s sampled persistence window; this does not imply continuous sub-heartbeat observation.

| Scenario | Observed response | Cause | Selected additional evidence |
| --- | --- | --- | --- |
| N0 | 15 nominal null heartbeats | live flags false |  |
| OBL | AUTO_LOITER; reach 0.584 s | offboard_control_signal_lost rise +0.536 s | event 0.600 s after last setpoint |
| GFH | AUTO_LOITER; reach 7.600 s | primary_geofence_breached rise +7.572 s | EKF trigger 17.081046 m; full-HIL max 19.438658 m |
| GFV | AUTO_LOITER; reach 3.600 s | primary_geofence_breached rise +3.564 s | EKF trigger 2.518473 m; full-HIL max 2.873000 m |
| OVJ | POSCTL; reach 0.600 s | text +0.040 s | 83 centered echoes; 5 pulse echoes |
| OVR | POSCTL; reach 0.600 s | text +0.040 s | 83 centered echoes; 5 pulse echoes |
| RCL | AUTO_LAND (via AUTO_LOITER); reach 6.544 s | manual_control_signal_lost rise +0.628 s | Hold departure 1.544 s; event 0.664 s after last RC |
| DLL | AUTO_LAND (via AUTO_LOITER); reach 11.600 s | gcs_connection_lost rise +6.372 s | Hold departure 6.600 s; event 7.504 s after last GCS heartbeat |
| KIL | DISARMED; reach 5.544 s | text +0.080 s | 149 kill-ON echoes; full-HIL drop 1.004000 m in 1.5 s |
| FDS | DISARMED; reach 0.600 s after forced command | ACK 1 then 0 | command 400: (0,0), then (0,21196); three nominal refusal-window heartbeats |

Timing is the preregistered simulated-time observation, not exact internal-event timing: heartbeats have about 1 s resolution and RX records carry the preceding local-position stamp. Cause texts provide earlier event observations where specified. No action latency of a real flight controller is inferred. KIL drop is a kinematic proxy; physical contact and actual motor-stop are NOT_RECORDED.

## Finding L1 — nonzero Gazebo shutdown exit must remain visible

`attempt01/ep000` (N0) has **gzserver returncode -6 (SIGABRT)**, while PX4, spawn and logger-stop return 0. `logs/gzserver.log` says `boost::wrapexcept<boost::thread_resource_error>` and `boost thread: trying joining itself: Resource deadlock avoided`. Do not summarize all process exits as zero or the teardown as exception-free.

The frozen observation ended at sim 49.736 s, approximately session +51.148266 s; cleanup landing ended at sim 57.336 s, approximately session +58.741898 s. Logger stop started at session +58.745 s and returned 0 after 0.025805 s. OwnedSession then records SIGINT (signal 2) sent to exact PX4 PID 2042065/start 72805056 and gzserver PID 2041987/start 72804933 at session +58.807 s; session closes at epoch 1794323045.615779. Event-to-session alignment here uses recorded receive epoch-minus-monotonic offsets, whose span is 0.000051022 s; the stored SIGINT `t_s` values themselves are directly recorded. The untimestamped Gazebo log gives **no exact abort instant (NOT_RECORDED)**, so SIGINT causation is not proved.

The actual measurement and landing telemetry are complete, logger flush is successful, and no watchdog/body/ownership cleanup failure was recorded. `run_loop` checks required live PX4/Gazebo roles, while `OwnedSession.__exit__` raises on leaks/refused identity/unreaped roles/cleanup errors/watchdog error, not on every nonzero termination returncode. The frozen reading criteria require kernel-count stability and frozen-window data, not zero for every post-measurement shutdown role. Thus this distinct shutdown failure is a reportable limitation, with no basis to alter the fixed PASS_SITL reading or require a rerun. It remains preserved in its original raw log/receipt and must accompany the result record. A future shutdown-robustness investigation would be separate work.

## Controls, remaining limits and record requirements

The audit passes 310 recorded checks: 263 primary and 47 independent numeric/stop checks. Primary assessment probes include 13 accepted genuine positive controls and 18 offline mutations: absent departures, wrong cause flags with otherwise correct modes, missing causal text, missing FDS ACK and wrong non-forced ACK. Missing required ATTITUDE RX rejects coverage; a one-byte-truncated owned ULog copy rejects strict framing while all original ULogs parse; faults/order mutations produce VOID while the genuine result remains PASS_SITL. Offline stop controls preserve the first-invalid/no-retry behavior. These controls are non-vacuous and support the CPU/offline assessment only.

Initial reviewer-probe failures were corrected in the audit scripts only: JSON lists versus source tuples in config comparison, and an incorrect reviewer kill-switch enum assumption (actual `ManualControlSwitches.msg` gives ON=1/OFF=3; the production source already used it correctly). Initial logs are retained as `probe.log` and `numeric_probe.log`; corrected runs are `probe_r2.log` and `numeric_probe_r2.log`, both passing. No measured result, threshold or operational source was edited to obtain a pass.

Known preregistered limit retained: allowed ULog flag gap 1.0 s exceeds the 0.7 s cause window; it did not affect this run (maximum actual window gap stated above, every required cause observed). One boot per scenario establishes occurrence in this fixed run only; no repeatability, failure probability, motor-stop guarantee, real receiver/wiring behavior, real-vehicle arena stopping distance, learned-policy performance, or hardware/flight readiness is established. No live known-bad control exists. This is one S5 checklist item, not S5 as a whole.

**Before records: no source/protocol repair is required.** Preserve the exact PASS_SITL label, stand-in scope, NOT_TESTED live discrimination, contact/motor-stop limits and finding L1. Keep all historical ABORTED smokes and original raw bytes/labels unchanged; never relabel this as a generic safety validation. Results can now be recorded under the root's existing authorized unit, with review/model identity and the raw package bindings.

Audit artifacts only: `probe.py`, `probe_results.json`, both primary logs, `numeric_probe.py`, `numeric_results.json`, both numeric logs, two initial and two corrected `unit_stop_*.jsonl` fixtures, `truncated.ulg`, this report and its generator. Probes ran with `CUDA_VISIBLE_DEVICES=''`, `PYTHONNOUSERSITE=1`, OMP/MKL/OpenBLAS=1, affinity 6,7, nice10, 2 GiB virtual-memory cap and 360/120 s hard outer timeouts via px4-env Python. No live processes were started. Full PUBLIC_CPU was not repeated during this read-only review.

Git Traceability: branch camonly/dev; base/reviewed HEAD d6e83126f8671e64f3ff490a82b29205c83e4df6; new commit NONE (reviewer is read-only); changed tracked files NONE; tests/probes as above; working tree clean at review checks; remote push NO; main merge NO; reviewed source/raw hashes and sizes match the bindings/manifests, and original raw/checkpoint bytes were not written by this reviewer (no exhaustive checkpoint rehash claimed).
