# Dev08 independent results review r1

Verdict: **LIMITED_VALIDITY — acceptable as the preregistered observed-prefix descriptive development record.** The integrity and recomputation checks below PASS. No BLOCKER or MAJOR was found in the audited results. This is not formal S3b acceptance, gain selection, statistical superiority, a complete 60-second performance claim, or evidence of real-flight readiness. S3a remains NOT_EQUIVALENT; strict600 input coverage remains BLOCKED.

Reviewer: `/root/dev08_results_review`, fresh-context Codex/GPT-6 family; exact runtime model variant **NOT_RECORDED**. Review date: 2026-11-05 UTC / 2026-11-06 KST. Execution branch/commit: `camonly/dev`, `d8f9bc66ad8819c230603b23dc7d4563525c2386`. The root session clock was read and inherited, never reset or closed: start13:45:22.052906UTC, wrap17:45:22, finish-by18:35:22UTC. All reviewer writes are confined to this new external audit directory. No simulator, GPU, outcome rerun, policy checkpoint deserialization, repository/raw modification, Beads mutation, Git commit, push or merge was performed.

## Method and reproducible evidence

I read the actual worktree AGENTS.md, shared handoff, current WORKLOG/Beads state, amendments1–5 and runtime sources. The primary audit independently reconstructs the fixed schedule, binary target traces, numeric layout geometry, physical decisions, parameter maps, outcome counts, logical bounds, paired transitions and nominal time differences. It does not import the production runner/analyzer or the root analysis script. Only after computing the results does it compare them with `analysis_v1/analysis.json`.

Commands executed for the recorded probes, from external scripts:

```bash
ulimit -v 4194304
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 timeout -s TERM -k 10 180 nice -n 10 taskset -c 6,7,12,13 /home/fair/miniconda3/envs/aerialgym/bin/python -B /home/fair/motar_tmp/dev08_execute_20261105/results_review_r1/audit_v2.py
```

The same environment/affinity/nice/4GiB cap, with timeout120s, ran `supplement_v1.py`. The standard-library-only `receipt_check_v1.py` and `analysis_fields_v1.py` ran with a1GiB address-space cap and timeout30s. Outputs are in the corresponding `.out` files. Each completed in under1s. The supplementary probe measured kernel faults0→0. The actual guarded outcome run ended before any reviewer CPU probe.

The initial `audit_v1.py` stopped with a reviewer-script KeyError: `empty_world_template` carries a SHA but no optional byte-size field. Its source/output remain preserved. `audit_v2.py` corrects only that metadata assumption, retains mandatory SHA verification, and PASSes. This was not a runtime/result defect or a changed outcome. No failed evidence was overwritten.

Machine-readable outputs:

- `independent_results_v2.json`: independent recomputation, all member/pair facts, check counts and measured timing/resource distributions.
- `verified_hashes_v2.json`:631 stream-verified input/source/runtime/raw paths and hashes, including Policy F; no checkpoint was loaded.
- `supplement_v1.json`: independent generated-world geometry, package/launcher bindings, RTF consistency and input/report source hashes.
- `receipt_check_v1.json`:126 journal-to-session launch identities,84 owned cleanup actions,42 rootfs `etc` links, and no active simulator/dev08 runner at the recorded check.
- `analysis_fields_v1.json`: all42 primary member summaries and four explicitly post hoc failure/counterpart summaries re-derived from immutable raw checkpoint rows.

## Integrity and protocol checks

The attempt is COMPLETE with exactly42 ordered valid records,2 native smokes followed by40 main records, no unrun members and no record problems. The independently constructed order matches the header and every ledger identity/index:70bars before205, env0..9, even native→candidate and odd candidate→native. Smoke is excluded from the fixed20-per-arm/10-per-density main denominators. All records retain seed4501/episode0/observed-prefix-v1 and skip_max_s0.15.

The attempt-header SHA roots the append-only line chain; all42 links and the final-record digest/count close correctly. The exact approval file and user authorization receipt hashes match the header. All30 bound source/preregistration files match both the approved execution commit blobs and current bytes. The external replay source, all34 inventoried inputs/assets, Policy F's frozen SHA `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`, all103 unique runtime-bound files, and both external package code manifests verify. The external launcher matches its reviewed snapshot. The raw manifest contains exactly the actual507 files,697,582,825bytes; every byte count/SHA verifies by streaming.

The process receipt records exit0 at2026-11-05T14:51:24.577907UTC, unchanged clean execution HEAD, and kernel faults0→0. The actual command is the preregistered `timeout -s TERM -k 90 5800 nice -n 10 taskset -c 6,7,12,13 ... --execute` command. The captured timeout `/proc` limits show soft/hard address space17,179,869,184bytes (16GiB). Header environment confirms CUDA hidden, PYTHONNOUSERSITE1, OMP/MKL1, nice10 and the four permitted cores. The committed source enforces torch compute threads1. The16GiB limit is inherited from the external launcher; separate per-descendant limits snapshots are NOT_RECORDED. All42 sampled aggregate-RSS watchdogs are nonempty and have no abort/error, and all member/density/overall recorded durations remain within their configured budgets. The attempt's recorded elapsed time is1888.205803704s.

All42 session-receipt hashes match; owned-session import/exit source hashes and boot IDs match. Each launch journal agrees with its three session launches, including PID/start ticks/argv. Each PX4 cwd is its own exclusively created external rootfs, with its own `etc` link. All84 cleanup signals target journaled owned launch identities; all three launch roles exit0. Every receipt reports normal cleanup, no leaked/unreaped/refused identities, no signal/error and a stopped receiver. Fresh-rootfs creation is established by the committed exclusive-directory implementation, unique launch cwd, boot/default readbacks and retained logs; no pre-boot filesystem snapshot exists.

For each member, all27 recorded booted parameters equal independently reconstructed float32 firmware/airframe defaults. Echo keys, selected overrides and final effective maps agree exactly; non-overridden recorded parameters remain unchanged. Native effective yaw P/I are0.109999999404/0.054999999702. Candidate changes exactly MPC_XY_VEL_P_ACC2.5→2.0, MC_YAWRATE_P0.11→0.2 and MC_YAWRATE_I0.055→0.1 (stored float32 representations). The other gain/flight-envelope values match their specified maps. Raw PARAM_VALUE packets/read-request timestamps are NOT_RECORDED, so the post-request/source matching proof uses the hashed guard implementation and its retained accepted outputs.

## Checkpoints, physical outcomes and timing

All2,001 raw checkpoint rows are contiguous k0..final, with exact target[k] independently decoded from the pinned binary zero-action trace. All20 admitted windows are exact:208frames for70/env5,600otherwise. No target padding, replacement or index clamp appears. All42 records have final k, k+1 checks and k commands, matching the raw row counts. Every nonfinal row is CONTINUE with a command; every terminal row has no command, and there is no subsequent scored row.

Using numeric layouts and recorded `p_true`/target rows, I independently recomputed all2,001 distance/swept-distance/clearance values and capture-before-crash decisions, including altitude/bounds crash predicates. All agree. No checkpoint simultaneously satisfied capture and crash, so the precedence rule is confirmed by source/logic inspection and consistency, not an observed simultaneous-terminal case. All42 generated XML worlds independently match the raw scene transforms, box dimensions and100ODE iterations (5,775bar instances checked); layout quaternions are identities.

The two preregistered pre-main smokes are valid captures at k34 and31. Their RTF values are1.00402180335 and1.00031553832, and compute maxima10.3867700091ms and5.64771605423ms: both satisfy the fixed[0.95,1.05] and[0,100)ms checks. They completed before the first main session.

Across all42 episodes/2,001 checkpoints:

| Recorded quantity | Independently verified range |
| --- | --- |
| Episode real-time factor |0.998868285652–1.00576712075|
| Checkpoint compute |0.130891975–11.195752013ms; median4.9924449995ms|
| Position lateness |0–0.02399999999999s|
| Attitude minus position time |−0.02000000000000–+0.00800000000000s|
| Truth minus position receipt time |−0.0138721920084–+0.000469579012133s|
| Sampled peak aggregate RSS |1,145,364,480–1,514,156,032bytes|

All recorded sequence numbers and selected position simulator times increase, and the observed skew/lateness fields remain inside their frozen bounds. All per-member compute maxima/medians/counts and other primary member-summary fields match the raw rows. The RTF reconstructed using final checkpoint `wall_s` differs by at most0.000002017632 from the ledger RTF, consistent with the source's later end-of-terminal-compute sample. Its exact final denominator timestamp is NOT_RECORDED.

## Independently reproduced primary reading

| Main cohort | Native capture/crash/censor | Candidate capture/crash/censor |
| --- | --- | --- |
|70bars,10scenes/arm|10/0/0|10/0/0|
|205bars,10scenes/arm|8/2/0|9/1/0|
|Both densities,20scenes/arm|18/2/0|19/1/0|

Paired native→candidate transitions:18capture→capture,1crash→crash and1crash→capture;0pairs contain censoring. The18 jointly captured pairs have candidate-minus-native nominal capture-checkpoint differences median0.0s, minimum−0.9s and maximum+1.3s. These are checkpoint-time summaries, not latency/causal/superiority estimates. Native and candidate both crash in205/env2 (k74 and83);205/env5 is native crash k70 versus candidate capture k130. No hypothesis test or gain choice follows from one discordant fixed pair.

Because U=0 in both arms, preregistered logical capture-count bounds degenerate to[18,18]/20 and[19,19]/20; crash-count bounds to[2,2]/20 and[1,1]/20. These are bookkeeping bounds for this fixed observed-prefix cohort, not confidence intervals or a measured complete60-second success/crash rate. The supported windows remain20.7/59.9nominal seconds despite earlier physical terminals.

The four supplementary `posthoc_failure_cases.json` entries match recorded trace hashes, outcomes/reasons/k, sampled z ranges, terminal estimated/truth positions and Euclidean snapshot discrepancies. They remain post hoc descriptive facts. Sampled extrema are not continuous minima, snapshot discrepancy is not an independently timestamp-aligned estimator-error measurement, and these facts do not identify a crash mechanism or establish a gain-induced rescue.

## Limits and unchanged boundaries

Raw telemetry packets/source clocks, actual setpoint transmit/receive timestamps, receiver scheduling delays, exact PX4 truth-stream phase and an empirical50Hz stream-rate series are NOT_RECORDED. Rechecking recorded skews and monotonically selected sequence numbers does not independently reconstruct omitted packets, end-to-end timing, or rule out the disclosed receipt-window phase ambiguity. Numeric geometry is reproduced from recorded transformed positions; the original raw truth/position messages are absent. Main RTF is near1 and smoke checks pass, but this does not establish receiver promptness at each packet.

The idle-core observations and aggregate RSS/descendant scans are sampled, not continuous hard guarantees. Source cleanliness was checked at recorded guard points, with source hashes now matching the commit; there is no independent continuous filesystem event log. Physics/noise boot seed and run-to-run reproducibility beyond the fixed input/layout seed are NOT_RECORDED. This is one seed,20fixed scenes and one fresh execution per scheduled member in a fixed nonrandomized order. Truth-rendered sensors/replayed target prefixes, prior source-review MINORs, historical unrecoverable external evidence and all formal-stage/sensor/template limits remain unchanged. The absence of censored members does not qualify the missing strict600 inputs. No additional measurement is requested or authorized by this review.

## Git Traceability

Branch: `camonly/dev`. Base/execution SHA: `d8f9bc66ad8819c230603b23dc7d4563525c2386`. New Commit SHA: none (read-only external review). Changed Files: only new files under `/home/fair/motar_tmp/dev08_execute_20261105/results_review_r1/`; repository files changed by reviewer: none. Tests: independent audit v2, supplement v1, receipt check v1, analysis-field/posthoc check v1 PASS; failed reviewer v1 probe retained. Working Tree: clean at receipt-check snapshot15:01:33UTC; root may append authorized result documentation later. Remote Push: NO. Main Merge: NO. Raw/Checkpoint Integrity:507raw files/all manifest hashes verified, original sources/inputs/F unchanged, policy checkpoint hashed only.
