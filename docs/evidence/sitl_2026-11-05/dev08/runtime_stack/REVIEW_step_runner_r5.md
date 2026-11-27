# dev08 runtime stack (amendment2, README, recorder/runner r4 fixes): independent review r5

Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only on every repository and
the PX4 tree. Date 2026-11-05, about 19:30–20:05 KST. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`,
branch `camonly/dev`, HEAD `a672fc88f16b9dc7ecce5afcbb4d5c391c5c19de`, tracked tree clean. The reviewed files equal
their committed blobs (`REVIEWED_SHA256SUMS`, 7/7 OK against `git show a672fc8:<path>`).

## Verdict: **FAIL** (narrow: one MAJOR, which needs a code change plus amendment3 text; no BLOCKER)

- **Resolved:** MJ1-r4 and every r4 text correction are in the committed bytes. MJ2-r4 is fixed in a process with
  one thread: probes Q1, Q2/Q2b and Q3 now behave as amendment2 l.225–230 says.
- **MJ1-r5.** `signals_held()` uses `pthread_sigmask`, and that masks only the calling thread.
  - The real runner always has other OS threads when it appends or writes final.json: the torch intra-op pool exists
    from `warm_up` on, and the session watchdog and MAVLink receiver threads exist during a member.
  - A process-directed SIGTERM, which is what `timeout` and `kill` send, is then delivered to an unblocked thread.
    CPython runs the Python handler in the main thread at its next bytecode, so the handler fires inside the "held"
    region.
  - With one idle extra thread, or with the torch pool, the r4 Q2b window is back in every run. That is 8/8 runs:
    no final.json after a COMPLETE 42-line ledger (single-threaded: 4/4 held). The committed unit test also fails in 4 of 8 multi-threaded runs.
  - So amendment2 l.225 and l.227–228 do not hold for the process the unit would run.
- **No false positive.** No path yields a false COMPLETE. Each failure ends with no final.json or an unverifiable
  ledger, and `read_ledger`/`main_records` refuse both.
- With MJ1-r5 fixed and tested (and an amendment3 that states the mechanism), I would expect PASS_WITH_LIMITS.

## 1. Resolution table (r4 items against the committed bytes)

| r4 item | Status | Evidence (committed line numbers) |
| --- | --- | --- |
| MJ1-r4: kill-grace reconciliation | **RESOLVED** | amendment2 l.6–9 states the two changed bounds: kill grace 20 s → 90 s, and the outer timeout of 5700 s with the budgets checked between members. l.202–205 is the r4 replacement text, word for word. Parent l.104–105 (`timeout -k20`) and amendment1 l.150–151 ("timeout kill grace20s") are quoted correctly. `KILL_GRACE_S = 20.0` (session :92) |
| MJ2-r4 Q1: ledger write | **RESOLVED (single-threaded)**; **reopened by MJ1-r5** | attempt :368–372: the line, `prev_sha` and `next` are updated under one hold. Probe Q1: 4 lines, recorded 4, `read_ledger` OK, `interrupted_member` null. Q11 (member 5) is the same |
| MJ2-r4 Q2/Q2b: `_final` fault read | **RESOLVED (single-threaded)**; **reopened by MJ1-r5** | attempt :396–412: `_final` holds `_final_held` (status, fault read, write). Probe Q2: final.json exists, MEMBER_NOT_VALID, recorded 3. Q2b: COMPLETE, 42/42, `read_ledger` OK. Multi-threaded: Q2b has no final.json in 8/8 runs |
| MJ2-r4 Q3: post-member sample | **RESOLVED** | runner :464 `current = member`, :478 `current = None` after append, :484–486 names the member only if `rec.next == cur["index"]`. Probes: Q3, Q3b and Q10 (signal in the post-member git check) name `01_smoke_205bars_env0_native`. Q8 (signal in append's post-write fault read) and Q9 (signal in the pre-member check of m2) correctly give null |
| amendment2 l.6 | **RESOLVED** | now l.6–9, the r4 text |
| amendment2 l.118 | **RESOLVED** | l.121 "before and after it" |
| amendment2 l.129–130 | **RESOLVED** | l.132–134, the r4 text |
| amendment2 l.158–160 | **RESOLVED** | l.162–165 adds the previous-truth (−20 ms) effect without a count. The "8 of 100" count was dropped (n4-d) |
| amendment2 l.197–199 | **RESOLVED** | l.202–205, the r4 text, word for word |
| amendment2 l.201–202 | **RESOLVED** | l.207–210. It matches `idle_check(tries=3)` (attempt :180–189) and the open refusal (:291–293, Q5b). The post-member sample is "recorded and is not a stop" (runner :468–469) |
| amendment2 l.217–221 | **TEXT AS ASKED; FALSE IN THE RUNTIME CONFIGURATION** → MJ1-r5 | l.225–230 is the r4 replacement text. It is true in a single-threaded process (probes Q1–Q3, Q8–Q12) and false with any other thread alive |
| README l.48–49 | **RESOLVED** | l.48–49 is the r4 optional text |
| README l.70 | **RESOLVED** | `link_mutants/results.txt` lists the control as passing and 3/3 mutants killed, and is bound in validation.json |

## 2. amendment2 against the parent and amendment1

**Unchanged:**
- **Estimand, analysis:** `tools/sitl_dev08_prefix_contract.py` is not in commit a672fc8. `main_records` (attempt
  :432–440) requires COMPLETE, an unchanged fault count and no record problems before `summarize_complete_pairs`.
- **Cohort, order:** `schedule()` (attempt :59–67) has smoke env0 at 70 then 205, then 70 before 205, env0..9, even
  envs native then candidate and odd envs the reverse. This is amendment1 l.44–51. The attempt path (l.193) is
  amendment1's.
- **Thread and process bounds:** 240 s per episode, 12 GiB sampled RSS at 0.25 s, cores 6/7/12/13, nice 10, CUDA
  hidden, PYTHONNOUSERSITE=1 and bounded threads are kept.

**Changed bounds:** the two runtime bounds amendment2 changes are now stated (l.6–9, l.200–205, l.218–219):
- the outer kill grace;
- the outer timeout, with the 2400 s and 5400 s budgets becoming between-member checks.

**Added items:** they map to amendment1's existing conditions:
- idle-core check: "idle host";
- source and binding change: "hash/schema mismatch";
- unreadable kernel log: "host fault";
- non-finite record: "non-finite states";
- `skip_max_s`: a number amendment1 left open, fixed in the approval.

**Remaining gaps:** l.6–7 does not list all of the added stops (n5-b). The outer timeout counts from a different
origin than the budgets (n5-a).

**Answer:** yes, with those two notes. amendment2 now states every bound it changes, and changes nothing in
amendment1's estimand, cohort, order or analysis.

## 3. Factual check of amendment2, README and validation.json

- **validation.json binds what it claims** (all recomputed):
  - 11 committed sources equal `git show a672fc8:<path>`.
  - 41/41 external evidence files match their hashes.
  - 10/10 review files match, and each `REVIEW_*.md` copy in the folder is byte-equal to its external original.
  - The focused (125 OK) and PUBLIC_CPU (3423 OK, 10 skipped) logs match their hashes, and both were written after
    the last source edit (19:43 and 19:48 against 19:40–19:41).
  - The `make_validation.py` hash matches.
  - It does not bind the README itself. That is acceptable, because the README links validation.json.
- **README.** l.47–49, l.70 and the reviews table (l.81–95) match the reports. The "Applied after the last round, not
  independently re-reviewed" list is honest. For the r4 deltas it says "(with a test)" only for the signal hold, and
  not for interrupted-member naming, which is in fact untested (m5-a).
- **amendment2.** I found no factual error apart from MJ1-r5 and the notes below. The r4-verified numbers were
  unchanged in this round's text: 463/500, 198 at k ≤ 2, k = 164, 0.136 s, 218.4 ms at k = 27, 18–75 checks, RTF
  0.996–1.004, 62 bound files, 40 plugins.

## 4. Code findings (post-r4 code)

### MAJOR

**MJ1-r5. The signal hold is per-thread, and the runner is multi-threaded** (attempt :192–204, used at :368 and :397;
probe `probes/probe_r5_threads.py`, outputs `probes/probe_r5_threads_*.out`, `probes/repeats_summary.txt`).

How it fails:
- `signal.pthread_sigmask(SIG_BLOCK, …)` changes only the calling thread's mask.
- Linux delivers a process-directed signal to any thread that does not block it.
- CPython's C handler then sets the eval-breaker flag, and the main thread runs the Python handler (`on_signal` →
  `RunnerTerminated`) at its next bytecode. Inside the hold, that is the next bytecode of the write or fault-read
  code.

Why it matters for this runner: there is always another thread at append/final time. The torch pool is created by
`warm_up`'s Policy F forward, before `rec.open()`. The session watchdog thread (session :541) and the receiver
thread (runner :138) run during each member. GNU `timeout` signals its child and, without `--foreground`, its process
group (coreutils behaviour, not measured here). Both are process-directed.

Measured (CPU only; the probe signals itself):

| Process threads | Q1 (signal inside the ledger write, m3) | Q2b (signal in `_final` fault read at COMPLETE) | committed `test_signals_are_held_…` |
| --- | --- | --- | --- |
| 1 (none extra), 4 runs | held: 4 lines, recorded 4, `read_ledger` OK | held: COMPLETE, 42/42, 4/4 | pass 4/4 |
| 2 (one idle `threading.Thread`), 4 runs | **not held**: raised before the write; 3 lines, m3 named interrupted (consistent only because the probe signals before writing) | **no final.json, 42 lines**, 4/4 | **fail 3/4** |
| 7 (torch `set_num_threads(4)` + one matmul), 4 runs | **not held** (as above) | **no final.json, 42 lines**, 4/4 | fail 1/4 |

A signal that lands after the write and before the chain update gives the r4 Q1 result: final.json that does not
close the ledger. The unit test fails exactly there, because its signal is sent from `hashlib.sha256` after the write.

**Scenario.** The approved run is stopped by SIGTERM (an operator, or `timeout` at 5700 s) while `_final` reads the
kernel log at COMPLETE or after an abort. The attempt then has no final.json, which contradicts amendment2 l.225 and
l.227–228. If the signal comes between a ledger write and its chain update, final.json does not close the ledger.
Neither outcome can be analysed, and neither is a false COMPLETE.

**Fix (small):**
- **Code.** Defer at the handler level, as the owned session already does (`_on_signal` with
  `_critical`/`_closing`, session :554–560). For example, `signals_held()` on the main thread could:
  - save the SIGTERM, SIGHUP and SIGINT handlers;
  - install a handler that only records the signal number;
  - `yield`;
  - restore the saved handlers;
  - call `signal.raise_signal(first)` if a signal was recorded.

  Python-level handlers always run in the main thread, so this holds whichever thread receives the signal. Keep the
  off-main-thread no-op.
- **Test.** Run `test_signals_are_held_over_the_ledger_line_and_final` with at least one idle `threading.Thread`
  alive. That is the non-vacuous control: the current mask implementation fails it in 3 of 4 runs, shown above. Repeat the test
  inside the runner's `Execute` fixture.
- **Text, for an amendment3:**
  - "Ledger lines with their chain update, and final.json with its fault read, are written while
    SIGTERM/SIGHUP/SIGINT are deferred by the runner's handler; a deferred signal is re-raised afterwards. This holds
    whichever thread receives the signal."
  - Until then: "amendment2's statement at l.227–228 holds only in a single-threaded process. In the runner, which
    has the torch thread pool and the session threads, a termination signal during final.json's fault read can leave
    no final.json, and one between a ledger line and its chain update can leave final.json not closing the ledger."

### MINOR

- **m5-a. Interrupted-member naming is untested** (runner :464, :478, :484–486). I ran three mutants against
  `tests/test_sitl_s3b_dev08_prefix.py` (`mutants/results_R.json`, `mutants/run_R.out`):
  - R1: always null;
  - R2: the r4 form, `current = None` right after `run`;
  - R3: the `rec.next == cur["index"]` condition dropped.

  All three survive (10/10 pass); the control passes. The attempt-side mutants are killed by the new test:
  - A1: chain update outside the hold;
  - A2: the r4 `_final`;
  - A3: `signals_held` as a no-op.

  Add `Execute` tests for the Q3 shape (named), the Q10 shape (named) and the Q8/Q9 shapes (null).
- **m5-b. The new test only covers the configuration that works.** It is single-threaded, so it passes for an
  implementation that fails in the real process (MJ1-r5). Folded into the MJ1-r5 fix.
- **m5-c. `kernel_faults()` has no subprocess timeout** (attempt :79). Under a working hold (after the MJ1-r5 fix),
  a hung `journalctl` inside `_final` would block every termination signal until `timeout`'s 90 s SIGKILL. Add
  `timeout=` (for example 30 s) and treat expiry as `fault_check_error`.

### NOTE

- **n5-a (r4 n4-a, quantified).** The two clocks start at different points:
  - `timeout … 5700` counts from process start.
  - The 5400 s budget counts from `rec.open()`, after scene inputs, bridge setup, Policy F load and warm-up.

  The worst case leaves little margin. A last member checked just under 5400 s can run 240 s (watchdog), plus about
  35 s of cleanup, plus up to 6 s of idle samples. That leaves about 19 s for the setup before open. The outer
  timeout can therefore end a unit that the stated "can exceed them by one member" rule would let finish. The result
  is an ABORTED attempt with RUNNER_EXCEPTION and the member named, never a false COMPLETE.

  For amendment3: "The outer 5700 s timeout counts from process start, including setup before the attempt opens
  (recorded as `warm_up` and the open time). The between-member budgets count from the open."
- **n5-b.** l.6–7's list of added stops leaves some out:
  - BINDING_CHANGED;
  - receiver thread not stopped (l.119–120);
  - CLOCK_SKIP/RUNNER_LATE/CLOCK_STALL/BAD_QUATERNION/STALE_TRUTH;
  - RUNNER_EXCEPTION.

  All of them implement amendment1's hash/telemetry/clock/ERROR aborts. For amendment3, "including" instead of an
  apparently closed list would be accurate.
- **n5-c.** If the signal arrives during append's post-write fault read (Q8), the member's own classification is
  skipped (HOST_FAULT/MEMBER_NOT_VALID/SMOKE_FAILED). final.json then reads ABORTED/RUNNER_EXCEPTION even when the
  fault count rose, not VOID_HOST_FAULT. `kernel_faults_end` is still recorded, and `main_records` refuses either way.
- **n5-d.** The expected duration (l.231–235) still leaves out the before-member idle check (up to 3 s). l.209 now
  counts the after-member sample's 3 s.
- **n5-e.** These are listed as open in the README and are unchanged:
  - numpy finite scalars and arrays are serialised by `default=str` without a record problem (r4 m4-f; probe Q4
    unchanged);
  - the surviving mutants.

## What I ran

Every command ran CPU-only:
`PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 6,7,12,13 timeout -k 10 <s>`, with `-B`.
- No PX4, Gazebo or GPU was used.
- The only signals were SIGTERMs that my probe processes sent to themselves.
- Nothing was written in a repository or the PX4 tree.
- Git use was read-only: `rev-parse`, `status`, `log`, `show`.

Runs and checks:
- **Unit tests.**
  - `test_sitl_dev08_attempt.py`: 15 OK in 0.46 s (`unit_test_sitl_dev08_attempt.py.out`).
  - `test_sitl_s3b_dev08_prefix.py`: 10 OK in 22.8 s (`unit_test_sitl_s3b_dev08_prefix.py.out`).
- **`probes/probe_r5.py`** → `probes/probe_r5.out`. It adapts r4's probe to the committed code, using the real
  recorder and `execute()` with the test fixture's fakes:
  - Q1–Q7 as in r4;
  - Q8: signal in append's post-write fault read;
  - Q9: signal in the pre-member idle check;
  - Q10: signal in the post-member git check;
  - Q11: Q1 at member 5;
  - Q12: a second signal during the except-path final.
- **`probes/probe_r5_threads.py`** (modes none/pythread/torch, 4 runs each; the first run's output per mode is
  `probe_r5_threads_<mode>.out`, the repeats `rep_<mode>_<n>.out`) → `probes/probe_r5_threads_*.out`,
  `probes/rep_*.out`, `probes/repeats_summary.txt`.
- **Mutants.** `mutants/make_and_run.py` and `driver.py` → `mutants/run.out`, `mutants/run_R.out`,
  `mutants/results_*.json`. They covered A0–A3 against both test files and R0–R3 against the runner tests. The
  first R pass failed in its control because of my scratch layout (`results_all_first_pass.json`, superseded by
  `results_R.json` after symlinking the step module).
- **validation.json.** Every hash was recomputed against the committed blobs, the external folder and the review
  files; log times were compared with the source mtimes.
- **Read-only reading.**
  - amendment2 (whole), parent prereg l.88–125, amendment1 l.40–60, l.76–90 and l.138–160.
  - README (whole).
  - Attempt (whole) and runner :1–140 and :290–500.
  - Session `_on_signal` and constants.
  - The r4 diffs against the r4 snapshot.
  - `link_mutants/results.txt`.
