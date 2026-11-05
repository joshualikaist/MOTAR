# Dev08 runtime stack (motar-5md) — CPU implementation, 2026-11-05

Protocol: [amendment1](../../../../plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md) (rows of
*Runtime stack and acceptance interfaces*), and the execution proposal
[amendment2](../../../../plans/prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md).
Scope: source and CPU fixtures only. **No PX4, Gazebo, SITL or GPU process was started**; measurement NOT_RUN;
every module reports `live_verified=false`, `execution_supported=false`, `launch_ready=false`.

## What was built

| Row | Source | Tests |
| --- | --- | --- |
| Owned session | `tools/sitl_owned_session.py` | `tests/test_sitl_owned_session.py` |
| Parameter/telemetry boundary | `tools/sitl_dev08_link.py` | `tests/test_sitl_dev08_link.py` |
| Policy/command step | `tools/sitl_dev08_step.py` | `tests/test_sitl_dev08_step.py` |
| Attempt recorder | `tools/sitl_dev08_attempt.py` | `tests/test_sitl_dev08_attempt.py` |
| Integration, guarded execution path | `tools/sitl_s3b_dev08_prefix.py` | `tests/test_sitl_s3b_dev08_prefix.py` |

[validation.json](validation.json) binds the sources, test results and external evidence by SHA-256.

## Findings made while building it

- **Shared PX4 parameter state.** rcS imports `parameters.bson` from the working directory, and with an unchanged
  SYS_AUTOSTART the airframe defaults are not re-applied. The historical runner used the shared
  `build/px4_sitl_default/rootfs`, so parameters set in one SITL run persisted into the next. All 23 dev07 PX4 logs
  (`/home/fair/motar_tmp/sitl_s3b_20261105/dev07_*/ep*_sitl.log`) show `importing from 'parameters.bson'`: 22 with
  `(INT32:13, FLOAT:16)`, `dev07_smoke_70` with `(INT32:13, FLOAT:6)` (`dev07_records.json`). dev07's effective values of parameters it did not set, including the yaw-rate gains,
  are therefore NOT_RECORDED. The new session
  boots every episode in an empty rootfs, and the parameter guard checks the booted values against the airframe's
  `param set-default` lines.
- **Sample-time comparison.** The historical loop waits while `pos_time < start + k*DT` in floating point. A
  tolerant comparison (`t >= t_k - 1e-9`) selects a different 50 Hz sample whenever the rounding of
  `start + k*DT` falls on a sample time. `tolerance_divergence.out` counts this over start samples 1500–1999:
  - 463 of 500 diverge within 600 checkpoints, 198 of them at k = 1 or 2;
  - first divergences range from k = 1 to k = 384;
  - the CPU harness's start sample diverges at k = 164.
  With the real bridge and Policy F on the CPU, the tolerant comparison changed the 600-checkpoint scene's commands
  from k = 164 on (`real_bridge_identity_prefix_gate_eps.json`, below). The gate now uses the historical
  comparison.
- **Bar orientation.** The historical world writer and footprint test treat bars as axis-aligned boxes. All twenty
  fixed scenes have identity quaternions (`layout_quats.out`, `scene_specs_check.json`); the new writer refuses
  any other orientation (LAYOUT_NOT_QUALIFIED).
- **The truth message has no clock** (found by review r1 of the link). PX4 v1.14.3 builds HIL_STATE_QUATERNION
  zero-initialised and never sets `time_usec` (`src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp`). The
  historical runner stamped truth with wall-clock receipt. The first gate design assumed a PX4 clock on it and would
  have refused every live episode at alignment. The gate now takes truth by arrival: the latest received from
  0.03 s before to 0.003 s after the selected position sample, and before the next position message. PX4 schedules
  each stream separately. The truth used is therefore sent in the position's PX4 pass or 4–16 ms before it, fixed
  per boot; in one phase a receive delay can admit the truth sent 4 ms after it (link review r3 probe). The receipt skew is recorded per checkpoint.
  A source-pinned test keeps the time_usec fact visible.
- **A late sample in dev07.** `dev07_lateness.out` recomputes, from the dev07 step logs, how late each consumed
  position sample was. One of 23 SITL episodes, `dev07_70/ep02` at k = 28, used a sample 0.136 s late and reused
  it at k = 29; the rest were within 0.04 s. The step log's compute column shows 218.4 ms at k = 27
  (`dev07_records.json`), so the delay was the runner's. Under a 0.05 s bound that member aborts. The bound is a decision for the approval
  (amendment2).
- **A historical report field is mislabelled.** The frozen runner's `compute_ms_median/max` index `r[-3]`, which is
  p_true y in metres, not the compute time (column 21 of the step log). The dev07 reports' compute fields are
  therefore not compute times; the step logs hold the real values. The historical file is not changed.

## External evidence (`/home/fair/motar_tmp/dev08_runtime_stack_20261105/`)

| File | What |
| --- | --- |
| `layout_quats.py`, `layout_quats.out` | read-only bar quaternions of the twenty scenes: all identity |
| `scene_specs_check.py`, `scene_specs_check.json` | `scene_specs()` on the verified inputs: 34 files, 20 scenes, windows 208/600 |
| `real_bridge_identity.py`, `real_bridge_identity_prefix_gate_eps.json` | real bridge + Policy F, CPU, before the comparison fix: 600-checkpoint scene differs from k = 164 |
| `real_bridge_identity.json`, `real_bridge_identity.log` | the same after the fix: both scenes identical over every nonterminal checkpoint (599 and 63 commands) |
| `dev07_lateness.py`, `dev07_lateness.out` | lateness of every consumed position sample in the dev07 step logs |
| `dev07_records.py`, `dev07_records.json` | compute time (step-log column 21) per dev07 episode; parameters.bson import counts per PX4 log |
| `link_mutants/`, `link_mutants/results.txt` | order-bound and settle mutants against the link tests: killed; unmutated control passes |
| `public_cpu_pre_fix.log` | full PUBLIC_CPU before the comparison fix: 3372 tests, 1 failure (that identity test) |
| `tolerance_divergence.py`, `tolerance_divergence.out` | where a tolerant sample comparison first departs from the historical one |
| `step_mutants/run_mutants.py`, `step_mutants/results.json` | the seven step mutants review r1 found surviving, now all killed; unmutated control passes |

## Reviews

Independent reviewers: Claude Opus 5.5 (Agent model opus), fresh context, read-only. Each round reviewed the bytes
named in its report. Copies of the reports are in this folder; [validation.json](validation.json) binds them by
SHA-256.

| Component | Round | Verdict | Main points |
| --- | --- | --- | --- |
| Owned session | r1 | FAIL | Runner termination left processes; leak-detection blind spots; cleanup not exception-safe; weak tests |
| | r2 | FAIL | The SIGTERM handler consumed the termination request (R2-M1) |
| | r3 | PASS_WITH_LIMITS | R2-M1 repaired; 4 MINOR |
| Parameter/telemetry | r1 | FAIL | Truth `time_usec` is 0 in PX4 (BLOCKER); dev07 late sample; freshness mutants survived |
| | r2 | PASS_WITH_LIMITS | Condition: stream-scheduling wording (N1) |
| | r3 | PASS_WITH_LIMITS | Conditions: offset numbers (N8), real-clock test robustness (N9) |
| Step, recorder, runner, amendment2 | r1 | FAIL | Identity broken by the sample comparison; Episode built after alignment; untracked code accepted; mutants |
| | r2 | FAIL | amendment2 statements; idle-core precondition |
| | r3 | FAIL | amendment2 statements only (text) |
| | r4 | FAIL (narrow) | Kill-grace reconciliation (text); three signal windows in the recorder/runner |
| | r5 (on `a672fc8`) | FAIL (narrow) | The signal hold covered one thread only; corrected in amendment3 and the following commit |

**Applied after the last round, not independently re-reviewed:**
- Owned session (after r3):
  - SIGINT deferred during cleanup;
  - a robust import test;
  - a teardown safety net by the test's own path;
  - an unreadable-marker refusal;
  - tests for marker ordering and `reap_journal` survivor reporting.
- Parameter/telemetry (after r3):
  - the N8 wording;
  - fake-clock settle tests at checkpoint and alignment (N9, N11);
  - the order-bound test (N10);
  - the receiver-promptness assumption (N12). `link_mutants/results.txt` shows the order-bound and settle mutants
    killed.
- Step, recorder, runner, amendment2 (after r4):
  - the reviewer's exact text corrections;
  - signals held over the ledger line with its chain update, and over all of final.json (with a test);
  - the interrupted-member naming.

**After r5 (amendment3's commit, not independently re-reviewed):**
- the handler-swap signal hold, tested with another thread alive;
- `signal_hold_check/results.txt`: the test fails on the `a672fc8` recorder and passes on the repaired one;
- the interrupted-member test;
- a 30 s timeout on the kernel-log read;
- amendment3: the outer timeout becomes 5800 s, and the complete stop list.

**Open:**
- surviving mutants listed in the reviews: step M19–M21, M27, M28; attempt A10, A19, A20, A23, A24, A26, A27, A29;
  owned session X18, X20, R2_N27;
- numpy scalars or arrays serialised as strings by `default=str` are not flagged;
- budgets are checked only between members;
- the notes in each review.

## Limits

- CPU fixtures replace PX4, Gazebo and MAVLink with fakes; the live assumptions listed in amendment2 are
  NOT_VERIFIED until a separately approved smoke.
- The identity harness uses scripted 50 Hz streams on a common grid with near-simultaneous arrival. On real PX4
  the attitude sample is the nearest in PX4 time and truth the latest received in the receipt window; the
  historical runner used the latest received of both. The two then differ in some stream phases, always by whole
  truth samples (20 ms).
- No candidate result exists. Nothing here is S3b acceptance, a gain choice or a flight claim.
