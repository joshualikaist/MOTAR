# dev08 runtime stack (amendment3, signal hold, r5 repairs): independent review r6

Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only on every repository and
the PX4 tree. Date 2026-11-05, about 20:20–20:45 KST. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`,
branch `camonly/dev`, commit `c4974f416249e543384f7dedf8128d805c408b68` (parent `a672fc8`).

Scope: the committed bytes only.
- The 8 reviewed files were extracted with `git show c4974f4:<path>` into `snapshot/`. All 8 equal their blobs
  (`REVIEWED_SHA256SUMS`, `REVIEWED_HEAD`).
- The tests and probes ran from `tree/`, a `git archive c4974f4` of tools/, tests/, docs/, motar_eval/, configs/ and
  aerial_gym/.
- The worktree's `tests/test_sitl_dev08_attempt.py`, `test_sitl_dev08_step.py` and `test_sitl_owned_session.py`
  were modified while I worked. None of those edits was used.

## Verdict: **PASS_WITH_LIMITS** (no BLOCKER, no MAJOR)

- **MJ1-r5 is resolved.** The handler-swap `signals_held()` holds SIGTERM, SIGHUP and SIGINT in a multi-threaded
  process. This covers the ledger line with its chain update, and all of `_final`. The held signal is re-delivered
  once, to the restored handler, after the write. It does not matter which thread the kernel picks: this was checked
  with signals aimed at a specific idle thread or torch pool thread through `tgkill`, as well as with `kill`. Result:
  15/15 runner probe runs, 5 queries each, with 1, 2 or 7 OS threads.
- **The other r5 items:**
  - m5-c is resolved in the code.
  - m5-b is resolved for the regression it named: the new threaded test fails 20/20 against the `a672fc8` mask and
    passes 20/20 on c4974f4.
  - m5-a is only partly resolved. The new test kills the "always null" mutant. The r4-era bug (`current` cleared
    right after `run`) still survives, and so does the dropped `rec.next` condition.
- **amendment3:** every statement is true for the code and records, with small imprecisions listed below.
  - The outer-timeout arithmetic is right: 5800 − (5400 + 3 + 240 + 35 + 3) = 119 s.
  - The stop list is complete for every status the code can reach.
  - Nothing in amendment1's estimand, cohort, order or analysis changes.
- **validation_amendment3.json** binds the committed sources correctly. Two bound external files were overwritten
  after the commit (m6-c).

## 1. Resolution table (r5 items, committed c4974f4 bytes)

| r5 item | Status | Evidence |
| --- | --- | --- |
| **MJ1-r5**: the hold was per-thread | **RESOLVED** | Fix at attempt :197–215: the recorder is swapped in at :208 and restored at :212–213, then `raise_signal(pending[0])` at :214–215.<br>Probe `probes/probe_r6_threads.py` (15 runs, modes none/pythread/torch × `kill`/thread-directed `tgkill`; `repeats_summary_r6.txt`). In every run final.json exists, `read_ledger` is OK, and the outcome is correct:<br>• Q1 (in the ledger write, m3) and Q1b (between write and chain update): 4 lines, recorded 4, `interrupted_member` null.<br>• Q2 (in `_final`'s fault read after MEMBER_NOT_VALID): reason kept, 4/4.<br>• Q12 (a second signal in the except-path final): RUNNER_EXCEPTION, m2 named.<br>• Q2b (at COMPLETE): COMPLETE 42/42, then RunnerTerminated.<br>Semantics probe `probes/probe_r6_hold.py`: three SIGTERMs in a hold give 1 call, on MainThread, to the restored handler (S1/S1b). Nested holds deliver after the outer hold (S3). A body exception still restores the handlers, and the signal's exception chains to the body exception (S4). Off the main thread the hold is a no-op, without error (S5). A saved SIG_DFL ends the process with rc −15 after the write (S6). A saved SIG_IGN stays ignored (S7). |
| n5-a: outer timeout origin | **RESOLVED (text)** | amendment3 §2: 5800 s with `-k 90`; the arithmetic is checked in §3 below. Residuals: n6-c |
| n5-b: incomplete stop list | **RESOLVED** | amendment3 §3 matches every `abort()` call site (attempt :348, :353, :360, :387, :390, :392; runner :461, :485) and every member status (runner :151, :247, :474, :476). Precision notes: n6-d |
| m5-a: interrupted member untested | **PARTLY RESOLVED** → m6-a | `test_an_interrupted_member_is_named` (prefix test :468) raises inside `run`. Mutants: R1 (always null) killed; R2 (r4's `current = None` right after `run`) survives 11/11; R3 (no `rec.next == cur["index"]`) survives 11/11 (`mutants/run.out`) |
| m5-b: single-thread test | **RESOLVED** (residual coverage m6-b) | `test_signals_are_held_with_other_threads_alive` (attempt test :357–390): control 20/20 pass, a672fc8 mask (A4) 20/20 fail (`probes/threaded_test_robustness.out`); `signal_hold_check/results.txt` reproduced. Mutants A1, A2, A3, A5, A6, A7 killed; A8 (SIGTERM only) survives |
| m5-c: journalctl without timeout | **RESOLVED (code)** | attempt :81–85. Fake `journalctl` that sleeps 45 s: `AttemptError: kernel log read timed out…` after 30.1 s, child reaped (`probes/probe_journal_timeout.out`). No suite test (n6-i) |
| n5-c, n5-d, n5-e | unchanged, not in amendment3's scope | r5 probe Q4 (numpy `default=str`) unchanged; n5-d (expected duration omits the pre-member idle check) not addressed |

The r5 single-thread probe re-run on c4974f4 (`probes/probe_r5_on_c4974f4.compare.txt`) gives the same result as at
a672fc8 for Q1–Q6 and Q8–Q12. Only Q7 differs, as expected: there is no mask any more.

## 2. Answers

**Q1: MJ1-r5.**
- *Mechanism.* Python-level handlers always run in the main thread, whichever OS thread took the signal. Swapping
  the Python callable therefore defers the signal process-wide. Measured: thread-directed delivery to the idle
  thread or a torch worker, with the main thread busy or sleeping, is held (S1b; all 9 directed runs).
- *Re-delivery.* It happens once, to the handler restored at :212–213, in the main thread. Two distinct signals in
  one hold re-deliver only the first one (S2). That is what amendment3 says, and harmless, because all three
  handlers end the runner (n6-b).
- *Nested holds:* correct (S3).
- *Exceptions in the body:* handlers are restored and the held signal still fires (S4).
- *Off the main thread:* a no-op (S5). The runner's holds are all on the main thread if `execute()` is (n6-g).
- *OwnedSession.*
  - The session swaps handlers only inside `run_episode`: SIGTERM/SIGHUP at session :538–540, and SIGINT added at
    :771 in `__exit__`.
  - It restores them before its own `raise_signal` (:789, :841).
  - The recorder's holds are only in `append`, after `run` has returned, and in `_final`. `_final` runs from
    `check_budget`, `check_host`, `append` or `finish`, or from the except path after the session's `__exit__`.
  - So the two swaps never overlap.
- *New defect:* none that affects a record. A µs-wide non-atomic window in the swap or restore can leave a recorder
  lambda installed (n6-a).

**Q2: m5-a, m5-b, m5-c.**
- m5-c is resolved (probe).
- m5-b is covered. The threaded test is robust: 0/20 control failures and 20/20 failures on the mask implementation.
  It is not vacuous against MJ1-r5's implementation.
- Two limits remain on the threaded test:
  - It sends with `os.kill(getpid())`. When the main thread does not block the signal, Linux hands a
    process-directed signal to the main thread first, so under the new implementation the test never forces the
    other-thread path. My `tgkill` probes do.
  - SIGHUP and SIGINT are never exercised (A8 survives) → m6-b.
- m5-a is only partly resolved → m6-a.

**Q3: amendment3.**
- *§1.* Every statement is true, including "fails against the `a672fc8` recorder":
  - `attempt_a672fc8.py` equals the `a672fc8` blob;
  - results.txt was reproduced, 20/20.

  "amendment2's statement applies from this commit on" is true for a main-thread runner. Re-delivery is
  first-signal-only, as §1 itself says, and the window in n6-a remains.
- *§2.* r5's worst case: 5400 (budget passes just under it), + 3 (pre-member idle), + 240 (deadline), + 35
  (cleanup), + 3 (post-member idle) = 5681 s. 5700 − 5681 = 19 s. 5800 − 5681 = 119 s, "about 120 s".
  - The kill-grace still holds after the new 30 s timeout. The worst SIGTERM-to-final path is session cleanup
    (about 35 s) plus `_final`'s held fault read (≤ 30 s), about 65 s < 90 s.
  - The 120 s margin for setup is not measured. The bridge identity run, which does setup plus 664 steps, took
    54.7 s. The process start time is not recorded in attempt.json, so the margin cannot be audited (n6-c).
- *§3.*
  - The table covers every reachable end reason.
  - `INCOMPLETE` (attempt :402–403) is not listed. It cannot be reached from `execute`: every iteration either
    appends or breaks after an abort.
  - Precedence imprecisions: n6-d.
  - "A refusal at open … creates no attempt directory" is true. Every refusal check and `faults0` come before
    `mkdir` (:286–323).
- *Unchanged:* `schedule()`, `record_problems`, `main_records`, `read_ledger` and `tools/sitl_dev08_prefix_contract.py`
  are untouched (diff a672fc8..c4974f4: attempt :35, :49–50, :80–89, :197–215 only, plus tests and docs). amendment3
  changes no estimand, cohort, order or analysis. It changes one amendment2 runtime bound, the outer timeout, and
  says so. `AMENDMENT3` is now a bound source (:35, :50).
- *validation_amendment3.json*, all recomputed (`probes/check_validation.out`):
  - The 12 sources equal `git show c4974f4:<path>`.
  - 11/11 reviews match, and the repo's `REVIEW_step_runner_r5.md` is byte-equal to the external r5.
  - Both logs match: focused 127 OK at 20:07:07; PUBLIC_CPU 3425 OK, 10 skipped, at 20:12:02. Both come after the
    last committed source mtime (20:06:10) and before the commit (20:13:05).
  - `make_validation.py` matches.
  - 46/48 external files match. `step_mutants/results.json` and `run_mutants.py` were rewritten at 20:29–20:30,
    after the commit (m6-c). Their bound hashes equal validation.json's at a672fc8, which r5 verified, so the
    binding was correct when made.
  - `base_head` = a672fc8 is correct: the file cannot bind its own commit.
  - `test_experiment_index.py` passes on the committed EXPERIMENTS.md (9 OK).

## 3. Findings

### BLOCKER
None.

### MAJOR
None.

### MINOR

**m6-a. Interrupted-member naming is tested in one shape only** (runner :456, :464, :478, :484–486; test :468–477).
- *Scenario:* the r4 regression comes back: `current = None` is set right after `run` (mutant R2). A signal in the
  post-member idle sample or git check (r5 Q3, Q10) would then give `interrupted_member: null` for a member that ran
  and was not recorded. No test fails, because the test raises inside `run`. R3 survives for the same reason.
- *Fix:* add two `ExecuteStops` cases.
  - Raise `R.RunnerTerminated` from the recorder's `host_check` on its post-member call for member 2. Expect m2
    named, `recorded` 2. This kills R2.
  - Raise it from `faults()` on append's post-write read of member 2. Expect null, `recorded` 3. This kills R3.

**m6-b. The hold test covers SIGTERM only, and under the new code never forces delivery to another thread**
(attempt test :357–390).
- *Scenario:* `held` loses SIGHUP or SIGINT (mutant A8). Every test passes, yet amendment2 l.227–228 names all three.
  Separately, with `os.kill(getpid())` Linux gives the signal to the unblocked main thread, so the test proves the
  regression against the mask but not the "whichever thread" claim of amendment3 §1.
- *Fix:* parameterise the threaded test over SIGTERM, SIGHUP and SIGINT. Send with
  `signal.pthread_kill(idle.ident, s)` in addition to `os.kill`.

**m6-c. Bound external evidence was overwritten in place after the commit.**
- *Files:* `/home/fair/motar_tmp/dev08_runtime_stack_20261105/step_mutants/results.json` and `run_mutants.py`, both
  modified 20:29–20:30. They are bound in validation_amendment3.json and validation.json.
- *Scenario:* a later audit recomputes the hashes and finds 2/48 mismatches. The content that was bound is not on
  disk any more.
- *Fix:* write new mutant runs (M19, M21, M27, M28) into a new directory. Recover the bound versions from wherever
  they were produced, if they are available. If they are not, record the mismatch in the next validation file
  rather than re-binding silently.

### NOTE

- **n6-a. The swap and the restore are not atomic** (attempt :208, :212–213). A signal whose Python handler has not
  been swapped yet can run in the main thread between the three `signal.signal` calls:
  - in setup, before `try`;
  - or, during the restore, after SIGTERM's real handler is back.

  Either way it raises out of the comprehension or the restore loop, and leaves recorder lambdas installed.
  - Injected with a wrapping `signal` proxy: S8 leaves SIGTERM swallowed; S9 leaves SIGHUP and SIGINT swallowed.
  - Effect: no record is lost. A termination exception is already propagating, the except path still writes
    final.json, and `execute`'s finally (runner :488–490) restores SIGTERM/SIGHUP. SIGINT can stay swallowed after
    `execute` returns.
  - The window is µs-wide.
  - Fix, optional: install one persistent recorder with a hold-depth counter, as `OwnedSession._on_signal` does with
    `_critical`. Or make each lambda forward to its saved handler once the hold has ended.
- **n6-b. First signal only.** Two distinct signals in one hold re-deliver only the first (S2). Text for a future
  amendment: "a held signal is delivered afterwards" → "the first held signal is re-delivered afterwards; the
  others are dropped (each ends the runner)".
- **n6-c. The outer-timeout margin cannot be audited from the records.**
  - Text for a future amendment: "attempt.json records the process start time, from `/proc/self/stat` field 22 and
    the boot time, so that the setup duration and the margin to the 5800 s outer timeout are known for every
    attempt."
  - A code change is needed for that: the header has `wall_start` at open and `warm_up_s`, nothing earlier.
  - Also state: "On a path that already ends with FAULT_CHECK_ERROR, the two 30 s kernel-log timeouts after the
    last member can use up to 60 s of the 120 s margin."
- **n6-d. Precision of amendment3 §3**, as text for a future amendment:
  - "MEMBER_NOT_VALID: a main member not VALID whose record has no non-finite value (one with a non-finite value
    ends RECORD_INVALID)." Code: attempt :374–375, :391–393.
  - "HOST_FAULT takes precedence over every per-member reason."
  - "FAULT_CHECK_ERROR: any error of the post-member kernel-log read (unreadable, 30 s timeout, or another
    exception)." Code: :384–388.
  - "INCOMPLETE (finish before the last member) exists in the recorder and cannot be reached from `execute`."
- **n6-e. GNU `timeout` signals the whole process group** (unless `--foreground`). The runner's `journalctl` and
  `git` children are in that group.
  - If the outer timeout fires during `_final`'s held fault read at COMPLETE, `journalctl` dies. That leaves
    `kernel_faults_end: null` and `fault_check_error`.
  - `main_records` then refuses a 42/42 attempt. This is conservative, never false, and unlikely at 5800 s.
  - PX4 and Gazebo are in their own sessions and are not affected.
  - Optional: start `journalctl` with `start_new_session=True`.
- **n6-f. A signal during `open()` after `mkdir` (:323)** leaves one of two things, both unanalysable, never a false
  COMPLETE:
  - an attempt directory without attempt.json and without final.json, if it lands before `t0` (:324);
  - final.json next to a partial header.

  amendment3's sentence about refusals is true; this is a different path.
- **n6-g. Off-main-thread holds are no-ops** (:204–206), and so are the handlers in `execute` (:420). The approving
  commit's CLI must call `execute()` in the main thread. Check it in that review.
- **n6-h. README wording.** README :93 reads "corrected in amendment3 and the following commit". The code fix is in
  the same commit as amendment3 (c4974f4). Suggested: "corrected in c4974f4 (code and amendment3)".
- **n6-i. The 30 s kernel-log timeout path has no suite test** (removing `timeout=` fails nothing). It was verified
  by probe only.
- **n6-j. Unchanged open items from r5:** n5-c, n5-d, n5-e (numpy finite values serialised by `default=str`,
  surviving mutants).

## 4. What I ran

All runs were CPU only:
`PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 16-19 timeout -k 10 <s> …/envs/aerialgym/bin/python -B`
(Python 3.8.20, torch 2.4.1).
- No PX4, Gazebo or GPU was used, and no cores other than 16–19.
- The only signals went to my own probe processes and their own threads (`kill`, `tgkill`, `raise_signal`).
- No repository or PX4-tree writes. Git was read-only: `show`, `diff`, `log`, `grep`, `ls-tree`, `archive`,
  `rev-parse`, `hash-object`, `status`.
- The experiment-index test ran read-only in the worktree, because it needs `git ls-files`.
  `docs/EXPERIMENTS.md` and the test there equal c4974f4.

Runs:
- **Unit tests** from `tree/`:
  - `test_sitl_dev08_attempt.py`: 16 OK in 3.8 s;
  - `test_sitl_s3b_dev08_prefix.py`: 11 OK in 22.9 s;
  - `test_experiment_index.py`: 9 OK.
  - Logs: `unit_*.out`.
- **`probes/probe_r6_hold.py`**, none/torch, giving `probe_r6_hold_{none,torch}.out`: S1–S9 as above, S8/S9 with an
  injected signal proxy.
- **`probes/probe_r6_threads.py`**, adapted from r5's probe: Q1, Q1b, Q2, Q12 and Q2b plus both committed hold tests,
  modes none, pythread and torch × `kill`/`tgkill`, 3 repeats each. Outputs: `rep_*.out`, `repeats_summary_r6.txt`.
- **`probes/probe_r5_on_c4974f4.py`**, r5's single-thread Q1–Q12: `probe_r5_on_c4974f4.{out,compare.txt}`.
- **`probes/threaded_test_robustness.py`**: 20× control and 20× the a672fc8 mask.
- **`probes/probe_journal_timeout.py`**, with `fakebin/journalctl`.
- **`mutants/make_and_run.py`**, giving `mutants/run.out` and `results_all.json`: A0–A9 against both test files,
  R0–R3 against the runner tests.
- **`probes/check_validation.py`**, giving `check_validation.out`: every hash in validation_amendment3.json.
- `journalctl -k -b 0` timing: 0.01 s, 3525 lines.

Read: the diff a672fc8..c4974f4, all of the attempt module, the runner's `run_episode`/`execute`, OwnedSession's
signal code (:20–45, :495–600, :760–848), the r5 review, amendment2 l.1–20 and l.195–238, all of amendment3, the
README delta, and the EXPERIMENTS.md delta.
