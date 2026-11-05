# Dev08 amendment3 — corrections after review r5, 2026-11-05

Parent: [`amendment2`](prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md), committed at `a672fc8`.
Review: `docs/evidence/sitl_2026-11-05/dev08/runtime_stack/REVIEW_step_runner_r5.md` (FAIL, narrow: one MAJOR).
This file corrects amendment2, which is immutable, and records the code change that makes one of its statements
true. It changes nothing of amendment1's estimand, cohort, order or analysis. **No execution is authorized.** No
PX4, Gazebo, SITL or GPU process was started.

## 1. Held signals (review r5 MJ1-r5)

amendment2 says, under *Proposed execution*, that a ledger line with its chain update and final.json with its
fault read are written with SIGTERM/SIGHUP/SIGINT held, and that a held signal is delivered afterwards. At
`a672fc8` this was true only in a single-threaded process.
- `signals_held()` used `pthread_sigmask`, which blocks the calling thread only.
- The runner has other threads from the warm-up on: the torch pool, the session watchdog and the MAVLink receiver.
- A signal delivered to one of them ran the Python handler in the main thread inside the held region. The review
  probe lost final.json in 8 of 8 runs.

From the commit that adds this file, `signals_held()` swaps the Python handlers for a recorder and re-delivers the
first recorded signal with `signal.raise_signal` once the write is complete. This works whichever thread receives
the signal, the same way the owned session defers signals.
- A test holds with another thread alive: `test_signals_are_held_with_other_threads_alive`, 8 repetitions.
- The test fails against the `a672fc8` recorder and passes against the repaired one
  (`/home/fair/motar_tmp/dev08_runtime_stack_20261105/signal_hold_check/results.txt`).
- amendment2's statement applies from this commit on.

## 2. Outer timeout (review r5 n5-a)

The outer `timeout` counts from process start; the 5400 s overall budget counts from the attempt's open. In
between come input verification, the bridge setup, the policy load and the warm-up. With 5700 s, a worst-case last
member (240 s) plus its cleanup left about 19 s for that setup. The outer timeout becomes
`timeout -s TERM -k 90 5800`, leaving about 120 s. It is still a stop rule: if it fires, the SIGTERM runs the
session cleanup and final.json is written, as amendment2 describes.

## 3. Added stops, complete list (review r5 n5-b)

amendment2's line 6 lists some added stops. The complete set the code can end an attempt with, each one an
implementation of amendment1's abort conditions (harness, telemetry, resource, source or host failure), is:

| Reason | When |
| --- | --- |
| SMOKE_FAILED | a smoke member not VALID or with record problems |
| MEMBER_NOT_VALID | a main member not VALID. This covers member statuses ERROR, HARNESS_INVALID, BINDING_CHANGED and SOURCE_CHANGED, and RX_THREAD_STUCK as an ERROR |
| RECORD_INVALID | a main record breaking amendment1's per-record rules (VALID records) or holding a non-finite value (any record) |
| SOURCE_CHANGED | the tree changed before a member |
| HOST_NOT_IDLE | the runner's cores not idle before a member |
| HOST_FAULT (status VOID_HOST_FAULT) | the kernel fault count rose |
| FAULT_CHECK_ERROR | the kernel log could not be read or timed out (30 s) |
| BUDGET_OVERALL, BUDGET_DENSITY_70, BUDGET_DENSITY_205 | the 5400 s or a 2400 s budget exceeded between members |
| RUNNER_EXCEPTION | a termination signal or another exception in the runner; `interrupted_member` names a member that ran but was not recorded |

A refusal at open (busy cores, an unreadable kernel log, an untracked or changed source, the environment, the
approval record) creates no attempt directory.

## 4. Smaller repairs in the same commit

- The kernel log read has a 30 s timeout (review r5 m5-c).
- Naming of the interrupted member is tested: `test_an_interrupted_member_is_named` (review r5 m5-a).
