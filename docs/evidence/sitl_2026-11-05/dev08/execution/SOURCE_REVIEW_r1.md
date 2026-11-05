# Dev08 amendment5: independent source/execution review r1

Reviewer: fresh-context Codex delegated reviewer `/root/dev08_execution_source_review`.
Model: GPT-6 family per the supplied Codex role; exact runtime variant **NOT_RECORDED**. Not attributed to Opus.
Date: 2026-11-05, approximately 13:54–14:08 UTC.
Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`; branch `camonly/dev`.
Base and observed HEAD: `cbf5527d1d42fe96b1a9b49b0e52afc72543c7ba`.

Reviewed the actual uncommitted source/test delta against cbf5527 and the new amendment5, not a proposed
conclusion. `snapshot/` and `probe_results.json` bind the five submitted files by SHA-256. The tests include the
parent's `receipt_env={}` fixture correction. Parent reports an earlier focused136 run with one fixture error;
I did not run that earlier full test invocation. The parent's original failure evidence must remain preserved.

## Verdict: PASS_WITH_LIMITS — source/protocol and CPU boundary only

No BLOCKER or MAJOR found in the reviewed delta. The CLI activation, numeric value conversion and pre-main smoke
timing checks are consistent with amendment5 and preserve the observed-prefix estimator and stop semantics.
This review does not establish live validity, authorize a rerun, certify the future launcher, or review any results.

Before live launch the parent must finish its required focused/PUBLIC_CPU/docs/index checks, commit the reviewed
source and amendment, confirm those commit blobs equal the bound reviewed bytes, and bind the approval to that
clean commit. The scope receipt inspected names the unchanged two-native-smoke plus40-main unit, fixes0.15 and
one compute thread, and denies GPU/push/merge. The current dirty/uncommitted submission itself is not launchable.

## Source and probe assessment

| Criterion | Assessment |
| --- | --- |
| Default/explicit CLI | Default returns2 without execute. `--execute` requires approval/output arguments and main thread, validates exact float0.15, binds original approval bytes by SHA, and returns0 only for COMPLETE. Actual execute checks required approval keys, exact scope, matching HEAD and no tracked dirt before warm-up. Selected CLI fixture passes; disabling its main-thread guard is detected. |
| Original cohort/order | Independent equality against cbf5527 `schedule()` confirms all42 ordered dictionaries unchanged: native70/env0 smoke, native205/env0 smoke, then original40 main members. No phase split, substitution or rerun added. |
| One compute thread | Existing environment checks still require OMP/MKL string1 before bridge imports. The production-import branch sets/checks torch1 before warm-up. New permanent fixture's positive branch reaches42 fake members; bad torch/OMP/MKL refuse before warm-up/output. Deleting the setter is detected. This resolves r7 m7-a within the fake-import branch scope. |
| Numeric records | Per-member `_finite_json` converts numpy integral/real/array/bool values to JSON primitives. Ledger no longer uses `default=str`. Nested nonfinite/unsupported values become null with explicit problems. Independent synthetic ledger retains2 records and closes SMOKE_FAILED with40 main unrun after a valid numeric first smoke and invalid second smoke. Header/final serializers remain unchanged, as amendment5 explicitly states. |
| Smoke timing | Positive controls include RTF0.95/1/1.05 and compute0/5/99.999999:9 accepted. Fifteen out-of-range/missing/bool/string/nonfinite controls rejected. An independently added second-smoke failure records both smokes, ends SMOKE_FAILED and leaves all40 main unrun. Smoke checking runs only for otherwise VALID smoke members; an existing source/binding/operational failure cannot be promoted. |
| Compute recording | Source timing begins after `gate.checkpoint`, includes `ep.step`, metadata/list append and actual command send, and records the terminal/censor branch before breaking. Independent synthetic `_fly` injects20ms checkpoint waits,3ms step work and7ms command sends: command records measure about10.2ms; terminal about3.1ms; maximum is the true maximum over rows;3 checks yield2 commands. This exercises actual timing locations without a simulator, receiver or launched session. |
| Estimator/physical stops | Step, prefix contract, historical runner, target windows, terminal precedence, command predicates and complete-pair analyzer are unchanged. Added timing fields are observational metadata. A bad smoke is relabelled HARNESS_INVALID, retained and stopped through existing SMOKE_FAILED; main estimand/denominators are unchanged. Physical terminal/censor checks still issue no terminal command. |
| Old guards | Actual diff changes neither parameter readback/default freshness, telemetry source/order/freshness/skip, owned launch/cleanup nor watchdog implementation. Source list adds amendment5; clean tracked source and per-source equality still apply at recorder.open. Existing input/PX4/resource/hash binding and host/fault checks remain. This is inspection of retention, not a repeated exhaustive audit of all earlier code. |
| Bounds | OwnedSession still defaults240s/member and12GiB sampled aggregate RSS. Recorder keeps2400 density/5400 overall, checked between members. Amendment5 explicitly retains timeout5800/kill90 and adds inherited16GiB per-process AS through the external launcher's `ulimit -v16777216`. AS/outer timeout are launcher duties; this review did not run or certify that launcher. |
| Interrupt regression repairs | New selected post-return/post-append test passes and kills both the early-current-clear and removed-rec.next-condition mutants, resolving r6 m6-a's named test gap. Independently tested handler hold for SIGTERM/SIGHUP/SIGINT with both process-directed and idle-thread-directed delivery:6/6 writes complete before redelivery and handlers restore. New permanent test covers the same signal/delivery shapes, addressing r6 m6-b; no claim to eliminate handler-swap windows. |

## Remaining limits and small note

- The misleading old execute docstring still says “Not called by main in this repository unit,” and the nearby
  section comment says the unit is not run. These comments should eventually match the newly enabled guarded
  entry. They do not affect the runtime boundary and are not a launch blocker.
- The permanent tests check smoke thresholds with fake record values; this review independently exercised the
  timing recording locations. That external instrumentation is not a permanent suite regression test.
- RTF remains the existing simulator-endpoint delta divided by scored-loop wall duration. The engineering
  tolerance is predeclared and conservative; very short/zero-length smoke paths can fail it. No receiver
  promptness, live stream timing, fresh-rootfs boot, parameter truth or shutdown is established by CPU fixtures.
- Direct library `execute()` retains its historical broader skip-range/injection interface; the reviewed CLI is
  the path that enforces main-thread entry and the user's exact0.15 choice. Do not bypass that entry for the live
  invocation. Approval provenance is recorded metadata, not a cryptographic authorization system.
- Step M20, owned-session R2_N27, handler-swap/receipt windows and first-held-signal semantics, sampled
  ownership/RSS limits, budgets checked only between members, other inherited review notes and the two lost
  historical external files remain open. This review does not silently resolve those findings.
- S3a NOT_EQUIVALENT, strict600 BLOCKED, actual-sensor/transfer qualification and formalS3b/S3c/S4/S5 unchanged.
  No full60s outcome, gain selection, formal acceptance, GPU or real-flight claim follows.

## What ran and evidence

Both probe commands used CUDA hidden, PYTHONNOUSERSITE1, OMP/MKL1, bytecode disabled, nice10, cores16–19,
8GiB address-space limit, timeout60s/kill5s; outputs and tempfile root confined to this new audit directory.

```bash
ulimit -v 8388608
export CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export TMPDIR=/home/fair/motar_tmp/dev08_execute_20261105/source_review_r1/scratch
timeout -k 5 60 nice -n 10 taskset -c 16-19 /home/fair/miniconda3/envs/aerialgym/bin/python -B \
  /home/fair/motar_tmp/dev08_execute_20261105/source_review_r1/probe.py
timeout -k 5 60 nice -n 10 taskset -c 16-19 /home/fair/miniconda3/envs/aerialgym/bin/python -B \
  /home/fair/motar_tmp/dev08_execute_20261105/source_review_r1/supplemental.py
```

- `probe.py`:3 selected permanent positive tests pass, schedule equality,9 accepted/15 rejected timing controls,
  second-smoke stop, numeric schema, synthetic timing instrumentation and3 killed mutants. Final command0,
  about0.24s shell-reported wait. Results in `probe_results.json` and `probe_v2.out`.
- Initial reviewer probe failed in its own synthetic truth-origin fixture before reaching timing assertions;
  production start validation correctly rejected it. Preserved `probe_v1.py` and `probe.out`; corrected probe
  gives0. This was a reviewer fixture issue, not a submitted-source failure.
- `supplemental.py`: numeric ledger retention/stop,6 signal controls,2 killed interrupted-member mutants. Final
  command0, about0.07s. Results in `supplemental_results.json` and `supplemental.out`.
- `git diff --check cbf5527` on the four changed source/test paths passes. Submitted file hashes were equal at
  probe start/end. Full PUBLIC_CPU/focused validation is the parent's separate work, not claimed as my run.
- No real bridge/F policy/checkpoint deserialize, PX4, Gazebo, SITL, GPU, live `execute()` or session launch.
  Fake execute fixtures inject bridge/torch/specs/recorder/member callbacks. No spawned simulator/sentinel
  processes; the signal probe used only its own idle thread. All reviewer compute has ended.
- No repository/Beads/Git mutation. Only read-only Git subprocesses were used, including test fixture git-state
  reads. Shared root clock was checked and never reset/closed; finish-by18:35:22 UTC retained.

Git Traceability: Branch camonly/dev; Base SHA cbf5527d1d42fe96b1a9b49b0e52afc72543c7ba; New Commit SHA NONE
(read-only reviewer); Changed Files NONE in repositories; Tests/probes above; Working Tree parent's submitted
changes (tools/sitl_s3b_dev08_prefix.py, tools/sitl_dev08_attempt.py, their two tests, docs/EXPERIMENTS.md and
new amendment5); Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity preserved, no writes/deserialization.
