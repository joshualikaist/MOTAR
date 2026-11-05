# REVIEW r3: dev08 owned SITL session (tools/sitl_owned_session.py, tests/test_sitl_owned_session.py)

- Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only on every repository.
- Date: 2026-11-05.
- Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, HEAD `9051cea552f9c408824ac34baf9883e8d25e6024`. Both reviewed files are untracked.
- Reviewed bytes: copied to `snapshot/` (relative paths kept) and checked against `SUBMITTED_SHA256SUMS`. They matched at the start, and at the end both in the worktree and in the snapshot:
  - `tools/sitl_owned_session.py` `c26f0eac…9f03` (1031 lines)
  - `tests/test_sitl_owned_session.py` `b7c0caa3…d87` (1063 lines)
- Inputs:
  - `../review_r2/REVIEW_r2.md`, with its probes and mutants (reused, adapted);
  - round 1;
  - the contract: amendment1 row "Owned session" (line 125);
  - AGENTS.md ("Bound every long run": `timeout -k`) and OPERATIONS.md:504 (long runs via `nohup` or `tmux`);
  - the consumer `tools/sitl_s3b_dev08_prefix.py` (read only, for composition).
- Safety: I launched no PX4, gzserver, gz or gazebo. I touched no instance-0 port and not `/tmp/px4_lock-0`, used no GPU, and edited nothing in any repository. Every process I signalled was one of mine, identified by pid + start time and by an `r3probe-*` argv token, a `review_r3/tmp/` path, or `OWNED_MODULE=…/review_r3/…` in its environment.
- References: `:N` is `tools/sitl_owned_session.py` line N; `test:N` is `tests/test_sitl_owned_session.py` line N.

## Verdict: PASS_WITH_LIMITS (no BLOCKER, no MAJOR, 4 MINOR, 12 NOTE)

R2-M1 is repaired. The termination request is no longer consumed:
- `SessionTerminated` is a `BaseException` (`:127`).
- A signal that arrives in `start()` or during cleanup is deferred (`:550-556`, `:572-578`).
- The first signal is re-delivered to the previous handler after the receipt and the lock release (`:832-834`).
- Evidence: P1b and P1e now stop after episode 0, and P1d drops from 3/20 to 0/20 unrecorded launches. Three new runner tests cover the behaviour, and all six re-delivery mutants (X01–X06) are killed.

The other repairs also hold:
- R2-m1 (start window): P1c, P1d.
- R2-m2 (cleanup re-sampling): P5b.
- R2-m3 (`reap_journal` members-first + re-sampling): P9.
- R2-m4 ("open" leak marker): P16, P22. The marker turns every runner death I produced before or during cleanup into a refusal of the next session, and `reap_journal` recovers from it.
- R2-m5 (`__enter__` exception safety): P10.

The core safety property still holds. No path signals a process outside the session:
- every `signal_verified` target comes from `members()` over launch or journal identities (`:719`, `:861`);
- `Popen.kill` acts only on the process's own unreaped child (`:589`);
- `raise_signal` targets the runner itself (`:834`).

What remains is MINOR:
- **R3-m1:** a SIGINT (Ctrl-C) during cleanup still abandons the cleanup and becomes an `Exception`.
- **R3-m2:** one test is flaky.
- **R3-m3:** test processes outlive the run under regression.
- **R3-m4:** four repairs have no test.

The other residues are fail-closed or declared limits (NOTEs).

## 1. Repair table

| Finding | Status | Evidence |
| --- | --- | --- |
| **R2-M1** SIGTERM/SIGHUP consumed; multi-episode runner continues | **REPAIRED** | **Code:** `SessionTerminated(BaseException)` `:127`; `_first_signal` kept for the whole session `:553-554`; deferred while closing `:555`; re-delivery after receipt and lock close `:832-834` (handlers restored at `:781-782`). Order: the receipt is written at `:831` and the lock closed at `:810` before the re-delivery, and it happens once per instance (`__exit__` runs once; a second `__enter__` is refused by `check_episode_dir`). **P1b** (`swallow` mode, `except Exception`): runner rc 124 at 2.0 s, episode 1 never started, ep0 `terminated`, child dead. **P1e**: SIGTERM during the 2 s grace is recorded (`during_cleanup`, `deferred`); runner ends at 3.6 s, no episode 1. **Tests:** `test_a_multi_episode_runner_stops` test:707, `test_a_signal_during_cleanup_is_delivered_after_it` test:720, rc = −15 asserted in test:692. **Mutants killed:** X01 (base back to `SessionError`, static test:573), X02 (no re-delivery), X03 (before receipt), X04 (before lock release), X05 (before handler restore), X06 (`_first_signal` only when raising). **Not done:** item 5 of the r2 fix (state the required `timeout -k` margin), see NOTE n-6. **Residues:** R3-m1, NOTEs n-1…n-5 |
| **R2-m1** signal between fork and recording | **REPAIRED** | **Code:** `_critical` around `_start_recorded` `:572-578`; Popen's exec-pipe wait is retried after a non-raising handler (PEP 475). **P1c** (adapted: a recording runner handler, because the re-delivery killed the unadapted probe, rc 143): launch recorded and journalled (228 B), receipt `terminated`, child dead, re-delivered once (`[15]`). **P1d**, no injection: **0/20** trials with an unrecorded live launch (r2: 3/20). **Test:** `test_a_signal_inside_start_is_deferred_until_recorded` test:735. **Mutants killed:** X07 (no critical section), X08 (deferred signal not raised; fails in 30 s, no hang), X09 (handler ignores `_critical`) |
| **R2-m2** declared limit wrong during cleanup | **REPAIRED** (untested) | **Code:** `_wait_gone` re-samples every `sample_s` `:702-713`; new members reach `_known`, then `more` (`:744`) and SIGKILL. Docstring `:34-35` and `LEAK_DETECTION` `:96-97` updated. **P5b**: D (setsid, ignores SIGINT, parent lives 1.0 s) SIGKILLed at `normal:after_grace`, `leaked []`. **Gap:** mutant X18 (re-sampling disabled) **survives**: no test (R3-m4) |
| **R2-m3** `reap_journal` single scan | **REPAIRED** for the r2 scenario; **RECORDED_AS_LIMIT** for the rest | **Code:** members first (`:871-872`), re-scan during the waits (`:863-868`), launches last, `alive_after` from a fresh scan (`:883-884`). **P9**: C SIGINT, D found and SIGKILLed, L SIGINT, D dead. **Test:** `kill_fork` test:746-777 (X19 launches-first and N29 no-SIGKILL killed). **Declared limit** (docstring `:847-848`), confirmed by **P9b**: a launch that forks on SIGINT and exits leaves D alive, and the report says `alive_after: []` (NOTE n-9). **Gaps:** X20 (no re-scan in `wait`) and X21 (`alive_after` reporting) survive (R3-m4) |
| **R2-m4** nothing durable for a session without `__exit__` | **REPAIRED** (as suggested); residues | **Code:** "open" marker naming the journal is written under the lock at entry (`:529-530`); at the next entry the journal's launches are checked (`:515-524`); on exit the marker is replaced atomically by "leaked" (`:803-807`) or renamed `.closed-*` (`:808-809`). **P22** (`timeout -k 1` < grace): runner SIGKILLed mid-cleanup, child alive, marker `open`, next session **LockBusy**, `reap_journal` cleans it. **P16:** the same outcome. **P15a:** the same outcome, and the next session is refused. **Test** test:746 asserts LockBusy for all three crash variants plus a positive control. **Mutants killed:** X10, X11, X12, X14. **Residues** (NOTE n-7): only launches are journalled (P11); the journal file must exist (P20b); an empty marker gives `JSONDecodeError` (P20a); marker files accumulate |
| **R2-m5** `__enter__` not exception-safe | **REPAIRED** | **Code:** handler installation and watchdog start inside the try; the except path restores handlers and closes the journal and the lock (`:534-547`). **P10**: handlers restored on 7 paths; after a failed `__enter__` the handler is not the session's, and the next session in the same process enters. **Test:** `test_enter_failure_restores_handlers_and_lock` test:602. **Mutants killed:** X16, X17. **Residue:** P19, the watchdog is left running (NOTE n-8) |
| **R2-m6** new-code test gaps | **PARTIAL** | Of the r2 informative survivors, N19, N20, N25, N29 and N15 are now killed. **N27 still survives** (leaked non-launch members omitted from the marker). **New survivors:** X15, X18, X20, X21 (R3-m4). See §4 for the full mutant table |
| n-4 abort cannot interrupt a blocked main thread | **REPAIRED** (stated) | `:47-48`: "callers use bounded waits" |
| n-6 lock file / marker changed on refusal | **PARTIAL** | The marker check now runs after the contention checks (`:505-524`), so a refused entry no longer renames a stale marker. This is untested: X15 (the r2 order) survives. The lock dir and file are still created on a refused entry (`:319-320`). New: every clean session leaves a `.closed-<ns>` file (NOTE n-7) |
| n-8 quoted kill forms accepted | **REPAIRED** for the reported forms; general limit stays | `_TOKENS` adds `'` and `"` (`:400`). Test test:622; X22 killed. Still accepted (`probes/n8_validate_argv_out.txt`): `sh -c "p''kill …"`, `bash -c '{pkill,-x,gazebo}'`, `sh -c 'p\kill …'`, `sh -c '$(printf pk)ill …'`. It is a lint, not a boundary (NOTE n-10) |
| r1 m-1 / r2: test processes outlive the run | **PARTIAL** | The `late` pid is now registered (test:340-342). New leaks under regression: R3-m3 |
| r2 N-1, N-2, N-4, N-7, N-9 | unchanged | Not re-opened. r2 N-3 is extended by n-4 and n-5 below. r2 N-5 (docstring `:30-31` overstates the marker) is unchanged (NOTE n-11) |

## 2. New findings

### R3-m1 MINOR: a SIGINT (KeyboardInterrupt) during cleanup abandons the cleanup and turns into `CleanupError`, an `Exception`
- **Where:**
  - `:773-779`: `except BaseException` turns any interruption of `_terminate` into `internal`, and `:836` then raises `CleanupError`.
  - Only SIGTERM and SIGHUP are deferred (`:534-536`).
- **Scenario P18** (`probes/p17_p18_second_signal_and_sigint.py`):
  1. A three-episode loop with an `except Exception` wrapper (the pattern of `run_episode`, `tools/sitl_s3b_dev08_prefix.py:148`).
  2. The SIGINT-ignoring child is in its 2 s grace when the runner gets SIGINT.
  - **Result:** receipt `reason cleanup_error`, `internal KeyboardInterrupt`, the child is **alive** and listed in a `leaked` marker. The `CleanupError` is swallowed, episodes 1 and 2 are refused (`LockBusy`), and the runner prints `finished` with rc 0. The SIGKILL escalation never ran, so the survivor has no deadline.
- **Why it matters:**
  - coreutils `timeout` forwards a SIGINT it receives to the command (`probes/timeout_forwards_sigint_out.txt`: child exit 7 from its SIGINT handler). So a Ctrl-C at the prompt of a `timeout -k …` run reaches the runner.
  - The first Ctrl-C (in the body) starts a cleanup that can take the 20 s grace. A second, impatient Ctrl-C then lands in the cleanup.
  - The outcome is fail-closed: the survivor is in the marker and later sessions are refused. But the user's stop request becomes "leave the simulator running", and the loop keeps going.
- **Fix:** while `__exit__` runs, defer SIGINT like SIGTERM: install the deferring handler for SIGINT at `:764`, then restore it and re-deliver after the receipt. Alternatively, in the except path:
  1. If the exception is a `KeyboardInterrupt` or a `SessionTerminated`, run `_terminate` once more.
  2. Write the marker and the receipt.
  3. Re-raise that `BaseException`, not `CleanupError`.

### R3-m2 MINOR: `test_import_starts_nothing_and_claims_nothing` fails falsely when any host process exits during its /proc scan
- **Where:** test:1045-1046. The child script opens `/proc/<pid>/stat` for every host pid, with no `try`.
- **Measured** (`flake_import_test.sh`, only this test, pinned to the brief's cores):
  - **12/300** false failures on this host as is (other agents and editors running);
  - **266/300** with a pinned `/bin/true` churn loop;
  - in the mutant run, 4 of 33 runs failed only here (N27, N37, X14, X16).
  - Every failure is `FileNotFoundError: /proc/<pid>/stat`.
- **Effect:** false failures only. It never passes falsely, but it makes the 46-test command flaky. Two of my mutants looked killed when they were not; a rerun showed both survive.
- **Fix:** read the child's own children instead (`/proc/self/task/*/children`), or skip pids whose stat vanished (`try/except FileNotFoundError, ProcessLookupError`).

### R3-m3 MINOR: under the regressions they test, two runner tests leave their own processes alive after the run
- **The leaks:** after the 33-mutant run, 4 fakes were alive with `notes=` paths under `review_r3/tmp/owned_session_*` (`mutant_leftovers_killed.txt`; I killed them by pid + start):
  - 3 × the `startsig` launch, under N20, X07 and X09;
  - 1 × the `kill_fork` late process, under X19.
- **Causes:**
  - `test_a_signal_inside_start_is_deferred_until_recorded` (test:735): under its own regression the launch is not journalled, so the tearDown journal fallback (test:158-161) cannot find it.
  - `test_sigkill_leaves_a_journal_that_reaps_by_identity` (test:770-772): it reads `late` but registers it only implicitly, by asserting it dead. When `assertIn(late, actions)` fails, nothing kills it.
- **Fix:** make the tearDown fallback independent of the module under test: SIGKILL, by pid + start, every live process of this uid whose argv contains `notes=<self.tmp>/`. The FAKE already puts that path in its argv. Also register `late` as a stray before asserting.

### R3-m4 MINOR: four repairs have no test (surviving informative mutants)

| Mutant | Change | Unguarded repair |
| --- | --- | --- |
| X18 | `_wait_gone` never re-samples | R2-m2: P5b passes on the code, but no test fails if re-sampling is removed |
| X15 | marker check moved back before the contention checks | n-6: a refused entry renaming a stale marker |
| X20 | `reap_journal`'s `wait` never re-scans | R2-m3: only the phase-2 scan catches the `kill_fork` process; a setsid escape during phase 1 is not tested |
| X21 | `alive_after` without the fresh scan | `reap_journal` reporting: no test has a survivor that must be reported |
| R2_N27 | leaked non-launch members omitted from the leak marker | carried from r2: a leaked grandchild would not block the next session |

- **Fix:**
  - For X18: P5b as a test.
  - For X15: a stale marker plus a Contention refusal; assert the marker is unchanged.
  - For X20: a phase-1 setsid escape.
  - For X21: a survivor via the patched `signal_verified` pattern of test:453.
  - For N27: `immune` on a grandchild instead of the launch.

### Notes
- **n-1 The session overrides an inherited SIG_IGN (nohup) for SIGHUP; the outcome depends on timing** (P14, `:534-536`).
  - Under `nohup` (OPERATIONS.md:504 recommends it for long runs), the control loop without a session ignores SIGHUP and finishes.
  - With the session, a SIGHUP in the body ends the runner: `SessionTerminated` traceback, rc 1.
  - A SIGHUP during cleanup is re-delivered to SIG_IGN, and the runner continues.
  - The consumer's `execute` also replaces SIGHUP (`sitl_s3b_dev08_prefix.py:424-426`), so for dev08 the composed result is "a hangup stops the run". It fails safe either way.
  - Fix: skip installing the handler for a signal whose previous disposition is SIG_IGN, or document "start runners with `setsid`/`tmux`, not `nohup`" (master_plan_2026-09-07.md:204 already says `setsid`).
- **n-2 "So a multi-episode runner stops too" (`:39-40`) holds only if the previous handler ends the process or raises a `BaseException`** (P15).
  - A runner handler that raises a plain `RuntimeError` subclass raises it from inside `raise_signal`, in `__exit__`, and an `except Exception` wrapper swallows it. Episode 1 starts and is then SIGKILLed. The "open" marker keeps this fail-closed (next session LockBusy; `reap_journal` cleans it).
  - With a survivor, the runner's exception replaces `CleanupError` (P15b: the runner sees `Stop`, with context `SessionTerminated`); the receipt and the `leaked` marker are correct.
  - The consumer complies: `RunnerTerminated(BaseException)` at `sitl_s3b_dev08_prefix.py:362`.
  - Fix: state the requirement in the docstring.
- **n-3 Two windows in which a SIGTERM skips `__exit__` entirely** (P16).
  - (a) At the first bytecode of `__exit__`, before `_closing = True` (`:764`). This is module-fixable: in `_on_signal`, defer when `frame.f_code is OwnedSession.__exit__.__code__`.
  - (b) CPython 3.8's with-exit sequence. After `POP_BLOCK` the eval breaker is checked before `BEGIN_FINALLY` and before `WITH_CLEANUP_START`, so an exception raised by the handler there leaves the `with` without calling `__exit__`. This is a language limitation of raising from a handler.
  - Both were injected by opcode tracing. Result: no receipt, child alive, marker `open`, next session LockBusy, `reap_journal` cleans it.
  - The windows are one or two bytecodes wide, so this is rare. The r3 marker makes it fail-closed, but "the same cleanup runs" (`:36-37`) is not unconditional.
- **n-4 A second SIGTERM after the handlers are restored loses the receipt** (P17, r2 N-3).
  - Handlers are restored at `:781-783`, before the marker, the lock close and the receipt (`:799-831`).
  - With a 1 s pause injected in the receipt's `_sha`: rc −15, child dead, marker already `.closed-*`, **no `session.json`**.
  - Fix: keep `_closing` True and restore the handlers just before `:832`.
- **n-5 The re-delivery is not in a `finally`** (P21).
  - An `OSError` while writing the receipt (`:831`) or the leak marker (`:804`) propagates before `:834`. The termination request is then lost to an `except Exception` wrapper.
  - With ENOSPC injected on `ep0/session.json`: `swallowed 0 OSError`, episode 1 started, then SIGKILL; ep1's child alive; ep1's open marker blocks the next session.
  - This needs an I/O failure. Fix: `try: … finally: if self._first_signal: signal.raise_signal(…)`.
- **n-6 The required `timeout -k` margin is not stated** (item 5 of the r2 fix).
  - Cleanup can take: watchdog join up to `kill_grace_s` + 15 s, then the grace, 5 s, 5 s and the reaps. With the defaults that is more than 30 s.
  - P22 (`-k 1` < grace 3 s): runner SIGKILLed mid-cleanup. The r3 marker makes it fail-closed (LockBusy; `reap_journal` recovers).
  - Fix: state in the docstring and the receipt that `-k` must exceed `kill_grace_s` + 25 s.
- **n-7 Open-marker residues** (`:515-530`, `:799-809`):
  - (a) Only launches are journalled. If the runner is SIGKILLed and a launch then dies while a session member lives, the next session **enters**, and `reap_journal` finds nothing (`alive_at_start []`, `alive_after []`) (P11).
  - (b) The check needs the journal file. With the episode directory moved away, the next session **entered** beside the live launch (P20b).
  - (c) An empty or torn marker, as left by a crash inside `write_new` (`:529`, not atomic, unlike the `tmp` + `os.replace` at `:803-807`), raises `JSONDecodeError`. That is not a `SessionError`; every later entry fails until a human removes the file (P20a). This is fail-closed.
  - (d) Every clean session leaves `*.leaked.closed-<ns>` and every cleared marker leaves `*.cleared-<ns>`; neither is ever deleted.
  - In production the name, port and PX4-lock checks still catch a surviving px4 or gzserver in (a) and (b).
  - Fixes:
    - the watchdog appends newly known member identities to the journal (fsync'd);
    - write the open marker through `tmp` + `os.replace`;
    - map a malformed marker to `LockBusy`.
- **n-8 A SIGTERM after `Thread.start` inside `__enter__` leaves the watchdog running** (P19, `:539-547` does not set `_stop`).
  - The watchdog kept sampling (0 → 8 samples in 0.5 s) until its own deadline abort ran `_terminate` on an empty session (no actions), so this is harmless.
  - The "open" marker also stays, and it is self-clearing.
  - Fix: `self._stop.set()` in the except path.
- **n-9 `reap_journal`'s declared limit gives a false all-clear** (P9b).
  - The launch itself forks D on SIGINT and exits. D is alive, not in the actions, and `alive_after` is `[]`. "Reported in alive_after when it is seen" (`:848`) is literally true, but D is never seen.
  - Possible rule: a process whose sid equals a launch's pid, and whose start tick is at most the last tick at which that launch was seen alive, is a member. The pid stays allocated while any process uses it as a session id, and a session id is only inherited. Alternatively, freeze the members with SIGSTOP before killing.
  - Carried from r2: `reap_journal` takes no lock and does not check that the runner is dead.
- **n-10 `validate_argv` is a lint.** Shell obfuscations still pass (see n-8 in the table). The risk is low, because the plan is built in code.
- **n-11 Docstring `:30-31`** still says an identity refusal or a signalling error leaves a marker "that blocks the next session". A marker with no live identity is cleared at once (r2 N-5).
- **n-12 Test order assumption:** test:764 asserts `alive_at_start[0] == child[0]`, which assumes the launch pid is the smallest. A pid wrap between the launch and its fork would fail the test falsely. This is rare.

## 3. Acceptance criteria (round 1) re-assessed

| # | Criterion | Status |
| --- | --- | --- |
| 1 | Context manager owning PX4/gzserver/spawn handles; per-episode fresh rootfs/world; exclusive instance lock | **Met, with limits.** Ownership now survives runner termination: SIGTERM and SIGHUP are deferred, re-delivered, and stop the runner (P1b, P1e). A crash before or during cleanup leaves an "open" marker that blocks the next session (P16, P22). Limits: SIGINT during cleanup (R3-m1); the two skip-`__exit__` windows (n-3, fail-closed) |
| 2 | Fresh/no-overwrite paths; contention fails without creating anything | **Met.** The marker is no longer renamed on refusal (untested, X15). Residue: lock file and dir created on refusal; marker files accumulate (n-7d) |
| 3 | Unrelated sentinel survives normal/exception/timeout cleanup; nothing outside the session signalled | **Met.** Re-confirmed over every signalling call site (`:589`, `:719`, `:834`, `:861`). Sentinels survive in every test. The new code (`_wait_gone` re-sampling, members-first `reap_journal`, re-delivery to the runner itself) signals only owned identities |
| 4 | PID start-time tracking, reaping, leaks detected | **Met, with declared limits:** a setsid-and-orphan within one period; a launch that forks while it shuts down after a runner crash, where `alive_after` falsely reads all-clear (n-9); a member that survives its launch after a runner crash (n-7a) |
| 5 | No `sitl_run.sh`, no shared build rootfs, no global pkill/killall | **Met.** `validate_argv` is a lint (n-10); the AST test is non-vacuous |
| 6 | Every rejection has a positive control; checks are non-vacuous | **Mostly met.** 22 of 33 mutants killed; 6 survivors equivalent or untestable; 5 informative survivors (R3-m4). Robustness: one flaky test (R3-m2) and leaks under regression (R3-m3); no hang (slowest mutant run 51 s) |
| 7 | Import starts nothing; no live-verification claim | **Met.** `STATUS` flags all False; NOT_VERIFIED live. The behavioural test is flaky towards false failure (R3-m2) |

The MINORs and NOTEs can be fixed later or recorded as open. None of them blocks the use of this module under the contract.

## 4. What I ran (CPU only: `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13`, `timeout -k`)

| Run | Result |
| --- | --- |
| Host check, start and end | No gzserver/gzclient/gazebo/px4 process. No listener on the 7 instance ports. `/tmp/px4_lock-0` absent. cpu4-5 offline. No `/tmp/owned_session_*` left |
| Brief's command (repository files) | 46 tests OK in 21.2 s at the start and 20.8 s at the end. ResourceWarnings are the test's own immune zombies (r2 N-9) |
| Harness (`harness_test.py` = the test file with `MODULE` from `OWNED_MODULE` and `ROOT` fixed; `TMPDIR=review_r3/tmp`) on `orig/` (byte-identical copy) | 46 tests OK |
| 33 mutants (`make_mutants.py`, `run_mutants.sh`; logs in `mutlogs/`; `mutants_result.txt`, `mutants_result_rerun.txt`) | **Killed (22):** R2_N15, R2_N19, R2_N20, R2_N25, R2_N29, X01–X12, X14, X16, X17, X19, X22. **Survive, equivalent or untestable (6):** R2_N04, R2_N30 (boot id), R2_N06 (fsync), R2_N12 (no later scan), R2_N37 (can only over-block), X13 (unlink instead of rename). **Survive, informative (5):** R2_N27, X15, X18, X20, X21. The first X05 build duplicated rather than moved the re-delivery (my error); the corrected X05 is killed. R2_N27 and R2_N37 were first "killed" only by the R3-m2 flake; the rerun shows they survive |
| `flake_import_test.sh` | Import test alone: 12/300 false failures as is; 266/300 with pinned churn (R3-m2) |
| P1/P1b `probes/p1_sigterm.py` | TERM and HUP plain: cleanup + receipt. Swallow mode: stops after episode 0 (R2-M1 repaired) |
| P1e | SIGTERM during the grace: deferred, then re-delivered; no episode 1 |
| P1c (adapted: recording handler) | Launch journalled and recorded; child dead; re-delivered once |
| P1d | 0/20 unrecorded live launches |
| P5b | Cleanup-phase escape found and SIGKILLed |
| P9 / P9b | r2 scenario repaired / declared limit confirmed, `alive_after []` with D alive (n-9) |
| P10 | Handlers restored on 7 paths; failed `__enter__` restores the handler and releases the lock |
| P14 `p14_sig_ign_nohup.py` | nohup: body SIGHUP ends the runner (rc 1); cleanup SIGHUP is ignored and the runner continues (n-1) |
| P15 `p15_raising_runner_handler.py` | Plain-Exception runner handler swallowed; CleanupError replaced (n-2) |
| P16 `p16_exit_windows.py` | Both skip-`__exit__` windows: no receipt, child alive, marker `open`, LockBusy, reaped (n-3) |
| P17 / P18 `p17_p18_second_signal_and_sigint.py` | Second SIGTERM: no receipt (n-4) / SIGINT during cleanup: survivor, loop finishes rc 0 (R3-m1) |
| P19–P22, P11 `p19_p22_misc.py` | Watchdog left running (n-8); empty marker gives JSONDecodeError; moved journal admits; member survives its launch and is admitted and not reaped (n-7); OSError skips the re-delivery (n-5); `-k` < grace is fail-closed (n-6) |
| `timeout_forwards_sigint_out.txt` | coreutils 8.30 `timeout` forwards SIGINT to the command |
| `n8_validate_argv_out.txt` | 4 shell obfuscations accepted, 2 quoted forms refused |

## 5. Cleanup of my own processes and files
- **Mutant-run leftovers.** Each was identified by a `notes=` path under `review_r3/tmp/owned_session_*` and by `OWNED_MODULE` naming my mutant, then SIGKILLed through `signal_verified` (pid + start); see `mutant_leftovers_killed.txt`:
  - 2220678 (R2_N20)
  - 2225774 (X07)
  - 2227038 (X09)
  - 2231298 (X19)
- **Probe children.** They carry `r3probe-*`; each probe killed its own by identity at its end. Final `kill_harness_leftovers.py` scan: 0 left.
- **Stopped run.** One `flake_import_test.sh` run passed the test name wrongly and ran the whole suite. I stopped it (TaskStop) before its churn phase started (stop file set first), then reran it correctly pinned.
- **Files.** Nothing written outside `review_r3/`. The probe sandboxes (receipts, journals, markers) are kept in `review_r3/tmp/` (5 MB) as evidence.
- **Worktree.** `git status` is the same at the start and the end: ` M docs/EXPERIMENTS.md` plus untracked files, none written by me. The reviewed files still match `SUBMITTED_SHA256SUMS`.
