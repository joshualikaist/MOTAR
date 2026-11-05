# REVIEW r2: dev08 owned SITL session (tools/sitl_owned_session.py, tests/test_sitl_owned_session.py)

- Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only on every repository.
- Date: 2026-11-05.
- Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `9051cea552f9c408824ac34baf9883e8d25e6024`.
- Reviewed bytes, both untracked. They matched the brief at the start and at the end of the review:
  - `tools/sitl_owned_session.py` sha256 `b999a6dded523ed868102faa954d44ace12e80e116d1459a3538ad90b4d2c296` (962 lines)
  - `tests/test_sitl_owned_session.py` sha256 `110c2640a6b5cc25148cefee068c6e49160eb17f1f8bbdb47a4db27ba0aa4679` (900 lines)
  - `tools/sitl_s3b_closed_loop.py` is unchanged against HEAD (`git diff --stat HEAD` is empty; sha256 `d170b027…84b4`).
- Inputs: round 1 (`../review_r1/REVIEW_r1.md`, its probes and mutants); the contract (amendment1, row "Owned session" and the paragraph after the table); `sitl_run.sh`; PX4 `main.cpp`; AGENTS.md.
- I did not launch PX4, gzserver, gz or gazebo. I touched no instance-0 port, did not open `/tmp/px4_lock-0`, used no GPU and edited no repository file. Every process I signalled was one of mine, identified by pid + start time and by an argv token or environment marker. All probes, mutants and logs are in this directory.
- File paths below are relative to the worktree. `test:N` means `tests/test_sitl_owned_session.py` line N; a bare `:N` means `tools/sitl_owned_session.py` line N.

## Verdict: FAIL (no BLOCKER, 1 MAJOR, 6 MINOR)

Most of the repair works:
- All four round-1 MAJORs are repaired in their own scenarios.
  - P1: `timeout` SIGTERM and SIGHUP now run the cleanup and write the receipt.
  - P2/P3: the pidfd EINVAL case maps to "gone".
  - P4: a process forked during shutdown is found and killed.
  - P6: a watchdog error aborts and is reported.
- All nine informative round-1 surviving mutants are now killed.
- The core safety property still holds: I found no path that signals a process outside the session. This includes the new code: `reap_journal`, zombie leaders kept for the session scan, the marker check and the unreaped-identity check.

The verdict is FAIL because the M-1 repair consumes the runner's termination request (R2-M1):
- A SIGTERM that arrives during a session's cleanup is recorded and then dropped.
- A SIGTERM in the body is raised as an ordinary `RuntimeError`, which the repository's own episode wrapper catches.
- In both cases a multi-episode runner keeps going, and `timeout -k` SIGKILLs it during the next episode. That episode's processes are left alive with no receipt and no leak marker. This is the round-1 M-1 outcome, reproduced (P1b, P1e).

The fix is small and local to the module (see R2-M1).

## 1. Repair table (round-1 findings)

| Finding | Status | Evidence |
| --- | --- | --- |
| **M-1** runner ended by timeout/SIGTERM/SIGHUP | **PARTIAL** | Repaired for one session: handlers `:522-524`, `_on_signal` `:529-533`, restore `:742-745`; journal fsync'd in `start()` `:564-567`; `reap_journal` `:799-817`. **P1** (TERM and HUP via `timeout`): owned child dead, `session.json` reason `terminated`, `signals` [15] / [1], no leak. Tests `test_sigterm_and_sighup_run_the_cleanup` and `test_sigkill_leaves_a_journal_that_reaps_by_identity`; mutants N18, N21, N41, N05 killed. PDEATHSIG and cgroup (optional in r1) not done. **New defects in the repair: R2-M1 (MAJOR), R2-m1** |
| **M-2** A: fork during shutdown after the leader was reaped | **REPAIRED** | Launches stay unreaped until the final scan (`:691-723`); exit is observed with `waitid(WNOWAIT)` (`:256-261`). **P4** (unmodified r1 probe): D SIGKILLed at `normal:after_grace`, `leaked []` is now true. Test `test_process_forked_during_shutdown_is_found`; mutants N10 (reap during grace), N11 (zombie leaders dropped) and N43 (no session scan) are killed by it. `test_unexpected_exit_is_reported_without_reaping` asserts state `Z` (N08, N09 killed) |
| **M-2** B: setsid+fork+exit within one sample | **RECORDED_AS_LIMIT** (declaration incomplete) | **P5**: still missed, `leaked []`. The limit is in the docstring (`:34-35`) and in receipt key `leak_detection` (`:89-90`, `:778`; asserted at test:265). The declaration understates the window during cleanup: see **R2-m2** (P5b) |
| **M-3** EINVAL / errno aborting cleanup; watchdog exception | **REPAIRED** | `signal_verified` never raises an OS error (`:226-253`); `_send` also catches (`:680-689`). **P2**: reaped pid pinned as sid gives `gone`. **P3**: no exception; G dead; receipt written; lock released (second session entered). `__exit__` does terminate + receipt under try/except/finally (`:729-745`). The watchdog body has try/except and aborts with `watchdog_error` (`:642-667`). `_require_watchdog` is used in `start`/`check`/`wait`/`sleep`. `/proc` readability is checked at entry (`:488-489`, fail-closed). **P6**: abort `watchdog_error`, child dead, CleanupError, marker written. Tests `test_reaped_pid_pinned_as_session_id_is_gone_not_an_error`, `test_watchdog_error_aborts_and_is_reported`, `test_dead_watchdog_refuses_work`; N22, N23, N24 killed. Residue: the `__exit__` internal-error path is untested (N25 survives; R2-m6) and a few steps after the try are unguarded (NOTE N-3) |
| **M-4** surviving mutants | **REPAIRED** | M01 (3 tests), M02, M03 (`test_survivor_…`), M04 (behavioural import test test:878-889), M05 and M15 (`test_busy_ports_refused`: `SO_REUSEADDR` UDP holder and TIME_WAIT positive control), M06 (`exit=1` spawn case test:284), M14 and M19 (`test_rss_cap_covers_the_tree_and_the_runner`) are all killed. M07, M16 and M17 are also killed (M17 now fails in 25 s instead of hanging). All eight r1 fix items are present (`assert_refused` checks `lexists` at test:620; the gzserver identity is checked dead at test:297). New-code test gaps are in **R2-m6** |
| m-1 test robustness | **PARTIAL** | The lock holder is bounded (`LOCK_NB` loop + `select`, test:653-662). tearDown has a journal fallback (test:158-161). That fallback kills journalled launches only, not their session members or descendants. Under M07, N10, N11, N26 and N43 the `late` process of `test_process_forked_during_shutdown_is_found` (test:339, never registered as a stray) outlived the test run, 5 leftovers in all. Under N05 two runner-test children outlived it, because the fallback depends on the journal under test. I removed all of them by identity (see §5) |
| m-2 git probe in the real repo | **REPAIRED** | Temporary `.git` (test:637-642). M13 is killed, and `git status` of the worktree was identical before and after the M13 run |
| m-3 unstated launch differences | **PARTIAL** | Differences stated in the docstring (`:10-14`, `:879`). The test pins `GAZEBO_MODEL_PATH`, `LD_LIBRARY_PATH`, `PX4_SIM_WORLD`, `HEADLESS`, all three cwds and the append order (test:771-787). Still unstated: the empty `LD_LIBRARY_PATH` element makes the loader search the cwd, now the episode dir / rootfs; the dropped `VERBOSE_SIM`/`ROS_VERSION`/`PX4_SITL_WORLD` branches; the dependence on the caller's `base_env`. The differences are in the docstring, not in the receipt |
| m-4 runtime binding coverage | **PARTIAL** (contract minimum met) | Now bound: recursive plugin glob (OpticalFlow, test:825-826), `px4-rc.*`, recursive `model://` resources, `parameters.json`, and an SDF that must be the PX4 tree's (`:848-850`). Not bound: `build/etc/init.d/rc.vehicle_setup`, `rc.mc_apps`, `rc.logging`, `rc.sensors` (sourced by rcS lines 262, 334 and onward), `px4-alias.sh`, Gazebo shared libraries |
| m-5 lock released after failed cleanup; narrow admission | **REPAIRED** | Leak marker written under the lock (`:761-769`), checked at entry (`:495-500`); admission also checks PX4's fcntl lock (`:507-510`). `test_survivor_is_a_cleanup_error_and_blocks_the_next_session` with a positive control; N01, N02, N03 killed. Limits: the marker is written only by `__exit__` (**R2-m4**); a refusal-only marker clears at once (NOTE N-5) |
| m-6 one-time port probe | **REPAIRED** | Watchdog aborts on more than one socket on UDP 14540 (`:657`, `:377-389`); `test_udp_contention_during_the_session_aborts` with a one-socket positive control; N13, N14 killed. TCP condition stated (`:359-362`). Residue: other ports and simulator names are not re-checked during the session (NOTE N-8) |
| n-2 os.kill wording | REPAIRED | `:32-33` |
| n-3 watchdog cost | re-measured | P13: 584 processes, 32 leaders (30 zombie spawns): median 5.5 ms, p90 6.1, max 15.7 ms per sample, so 2.2 % duty at 0.25 s. Measured while my mutant runs were loading the same cores |
| n-4 abort does not interrupt a blocked main thread | NOT_REPAIRED (not stated) | SIGTERM now interrupts. A watchdog abort still does not, and the docstring does not say that callers need bounded receives |
| n-5 PX4 instance lock | REPAIRED | `fcntl_lock_holder` (`:328-340`) uses F_GETLK. PX4 takes an fcntl `F_SETLK F_WRLCK` (`main.cpp:688-695`) and checks with F_GETLK itself (`:649-663`), so the probe sees it. `test_px4_instance_lock_refused` (lockf holder, pid matched, positive control); N16, N17 killed. The struct layout `hhxxxxqqixxxx` is the 32-byte x86-64 `struct flock` |
| n-6 lock file created on refusal | NOT_REPAIRED (NOTE N-6) | `acquire_lock` still creates the lock dir and file. In addition, a stale marker is renamed to `.cleared-*` before the contention checks, so a refused entry can also rename it |
| n-7 module hash at import | REPAIRED | `:97`, `:774`, asserted equal at test:264 |
| n-8 validate_argv | PARTIAL | Tokenised (`:392-401`); N44 killed. Quoted forms still pass: `["python3","-c","import os; os.system('pkill -x gazebo')"]` and `["sh","-c","\"killall\" gzserver"]` are accepted (`probes/n8_validate_argv_out.txt`). Low risk, since the plan is built in code |

## 2. New findings

### R2-M1 MAJOR: the SIGTERM/SIGHUP handler consumes the termination request; a multi-episode runner continues and `timeout -k` SIGKILLs it mid-episode (M-1 again)
- Where:
  - `:120-121`: `SessionTerminated` is a `SessionError`, so a `RuntimeError`/`Exception`.
  - `:522-524`: the session replaces the runner's handlers without chaining them.
  - `:529-533`: during cleanup the signal is only appended to `_signals`.
  - `:742-745`: the previous handlers are restored, but the received signal is never re-delivered or re-raised.
- Scenario P1e (`probes/p1e_sigterm_during_cleanup.py`; no exception handling in the runner; unmodified module):
  1. A runner loops over episodes.
  2. Episode 0's launch takes the 2 s grace to stop (it ignores SIGINT).
  3. `timeout -s TERM -k 3 1.5` sends SIGTERM during that cleanup.
  - Result: episode 0's receipt shows `reason: normal`, `signals: [{signal 15, during_cleanup: true}]`, `exception: null`. Nothing is raised, and the runner starts episode 1 at +3.09 s. At +4.5 s `timeout` sends SIGKILL. Episode 1's launch **stays alive**, with no `session.json` and no leak marker; only `launches.jsonl` names it.
- Scenario P1b (`probes/p1_sigterm.py`, mode `swallow`): the caller wraps each episode in `except Exception` and records it. This is the repository's own composing module, `tools/sitl_s3b_dev08_prefix.py:123-151`, which loops over the attempt schedule at `:429-447`.
  - SIGTERM in the body becomes `SessionTerminated`, which the wrapper records as an ERROR. The loop starts episode 1, and `timeout -k 2` SIGKILLs it.
  - Episode 1's child stays alive, with no receipt and no marker.
- The same shadowing also disables the consumer's own handler: `sitl_s3b_dev08_prefix.py:419-426` installs `RunnerTerminated`, which OwnedSession replaces for the whole duration of every episode.
- Why MAJOR: AGENTS.md requires `timeout -k` for long runs, and dev08 runs a multi-episode schedule. With the actual consumer, almost every SIGTERM during an episode ends in this state. With a plain caller, a SIGTERM during the cleanup does. The outcome is the round-1 M-1 outcome: an owned simulator stays alive with no deadline, and the only record is the journal, which nothing forces anyone to read. The next session refuses beside a live `px4`/`gzserver` (names, ports, PX4 lock), so episodes do not overlap. A human still has to find and reap the survivor.
- Suggested fix:
  1. Remember the first signal for the whole session, whether it arrives in the body or during cleanup.
  2. At the very end of `__exit__`, after the receipt and the marker are written and the lock is released, restore the saved handlers and re-deliver it: `signal.raise_signal(signum)` (Python 3.8; no `os.kill`, so the AST test stays green). The runner's own handler, or the default action (exit 128+15, which `timeout` expects), then runs.
  3. Derive `SessionTerminated` from `BaseException`, like `KeyboardInterrupt`, so that recorder-style `except Exception` blocks do not swallow it.
  4. Add a P1e-style test (signal during cleanup, three-episode loop: episode 1 never starts) and a P1b-style test (an `except Exception` wrapper still stops).
  5. State in the docstring and the receipt that the outer `timeout -k` must exceed `kill_grace_s` + 10 s + the watchdog join.
- Note for the consumer's own review: `RunnerTerminated` is also a `RuntimeError` (`sitl_s3b_dev08_prefix.py:361`), so `run_episode`'s `except Exception` would swallow it too.

### R2-m1 MINOR: a signal in `start()` between fork and recording leaves an unrecorded, unjournalled launch, and the receipt says clean
- Where: `:547-563`. Between `subprocess.Popen(...)` (which waits for exec on its error pipe; signal handlers run there) and `self._procs[role] = …` / the journal write, a `SessionTerminated` raised by `_on_signal` loses `p`.
- P1c (`probes/p1c_signal_in_start.py`): the signal is raised right after the real `Popen` returns, which is equivalent to arriving during its exec wait. `__exit__` writes `launches: []`, `leaked: []`, no errors, no marker, and the launched child **stays alive**.
- P1d (`probes/p1d_natural_race.py`, **no injection**): back-to-back starts (2.5 ms each) ended by `timeout -s TERM` at a random time. **3 of 20** trials left a live launch that was in neither the journal nor the receipt; every receipt said `leaked: []`.
- In production this needs the signal inside the fork+exec window of one of about five starts per 100–240 s episode. That is my estimate, not a measurement: well under 1e-3 per SIGTERM, larger for a runner with a big address space (fork time). A leaked `px4`/`gzserver` is caught at the next entry by name, port and PX4 lock.
- Fix (same family as R2-M1): while `start()` is between `Popen` and the journal fsync, have `_on_signal` set a pending flag instead of raising, and raise at the end of `start()`. Or block SIGTERM/SIGHUP in every thread, the watchdog included, with `pthread_sigmask` around that region.

### R2-m2 MINOR: the declared detection limit is inaccurate for the cleanup phase
- Where: `:34-35` and `LEAK_DETECTION` (`:89-90`) say a setsid-and-orphan is missed only "within one watchdog period". `__exit__` stops the watchdog first (`:727`), and `_terminate` scans only at `:702`, `:706`, `:711` and `:715`. `_wait_gone` (`:669-675`) does not rescan, for up to `kill_grace_s` = 20 s.
- P5b (`probes/p5b_daemonize_during_shutdown.py`):
  1. On SIGINT the leader exits.
  2. Its in-session child forks D at once (D calls `setsid()` and ignores SIGINT), then lives **1.0 s = 4 production periods** before exiting.
  - Result: D survived, `leaked: []`, not signalled, nothing raised. With the watchdog sampling, D would have been seen as a descendant within 0.25 s.
- Fix: let `_wait_gone` refresh `_members()` every `sample_s` and add new members to the targets, so the declaration becomes true. Or state the cleanup-phase window ("between cleanup scans, up to kill_grace_s") in the docstring and in `leak_detection`.

### R2-m3 MINOR: `reap_journal` scans once; a process forked on SIGINT survives and the report says `alive_after: []`
- Where: `:806-817`. It makes one `members()` scan, then SIGINT, a wait and SIGKILL of that set only. The round-1 P4 blind spot comes back here, because after the runner's death the journalled leaders are reaped by the new parent as soon as they exit.
- P9 (`probes/p9_reap_journal_fork.py`): the runner is SIGKILLed, then `reap_journal()` runs on a launch whose child forks a SIGINT-ignoring D on SIGINT. Report: `alive_at_start [L, C]`, `alive_after []`. **D is alive** with sid = L.
- Fix: in the crash reaper, freeze before killing. SIGSTOP every member, rescan with `known` = everything found until the set stops growing, then SIGKILL all and SIGCONT. Or SIGKILL directly, with no graceful SIGINT. Report `alive_after` from a fresh scan.
- Also (NOTE): `reap_journal` takes no lock and does not check that the runner which wrote the journal is dead, so running it on a live episode would stop that episode's processes. They are owned, but by another runner.

### R2-m4 MINOR: nothing durable marks a session that ended without `__exit__`
- Where: the leak marker is written only in `__exit__` (`:761-769`). A runner killed by SIGKILL, the OOM killer, or `timeout -k` during a long cleanup leaves no marker.
- Consequence: the next `__enter__` admits on names, ports and the PX4 lock only. Survivors with other names, such as helpers or a `gz` client, are not blocked, and nothing points at the journal (P1b, P1e).
- Fix: write the marker as "open" (episode dir, journal path) under the lock at `__enter__`, and replace it at a clean exit. At entry, an "open" marker should run `reap_journal` on its episode, or refuse until it has been reaped. Alternatively, read the previous holder record from the lock file before truncating it (`acquire_lock` `:318`) and check that its episode has a `session.json`.

### R2-m5 MINOR: `__enter__` is not exception-safe after the handlers are installed
- Where: `:519-527` are outside the try. If `Thread.start()` raises, or a SIGTERM lands between the handler installation and `return self`, `__exit__` never runs.
- P10 (`probes/p10_handlers.py`, `Thread.start` raising as under a thread limit):
  - The session's SIGTERM handler stays installed with `_closing False`, so a later SIGTERM raises `SessionTerminated` from unrelated code.
  - The lock fd and the journal stay open, so a second session in the same process gets `LockBusy`.
  - The episode dir is left without `session.json`.
- On every other path P10 found the handlers restored: normal, exception, watchdog abort, SIGTERM, CleanupError, and a signal during cleanup.
- Fix: start the watchdog before installing the handlers, and wrap the tail in try/except that restores the handlers, closes the journal and the lock fd, and writes a receipt or marker.

### R2-m6 MINOR: new-code behaviours without a test (surviving mutants)
46 mutants were run (`make_mutants.py`, `mutants_result.txt`, `mutants_result_rerun.txt`): 35 killed, 11 survive. The informative survivors:

| Mutant | Change | What is left unguarded |
| --- | --- | --- |
| N19 | handlers never restored | handler restore (P10 shows the code is correct; no test) |
| N20 | handler raises during cleanup | the `_closing` guard (P10 shows the code is correct; no test) |
| N25 | `__exit__`'s `except BaseException` removed | "receipt is written with an internal cleanup error" (no test makes `_terminate` raise) |
| N27 | marker ids omit leaked non-launched members | a leaked grandchild would not block the next session |
| N29 | `reap_journal` never SIGKILLs | SIGKILL escalation in the crash reaper (the fake dies on SIGINT) |
| N15 | `/proc/net/udp6` ignored | IPv6/dual-stack contention on 14540 |

Equivalent or low-value survivors:
- N04 and N30: boot id ignored (untestable on one boot).
- N06: no fsync.
- N12: reap even if the final scan is non-empty (no later scan exists).
- N37: marker liveness check without start time (fails safe: it can only over-block).

The first N43 build was a no-op (my error, `{} or {…}`). The corrected N43 (`{} and {…}`) is killed. Fix: add one test per informative row (for N25, patch `_terminate` to raise after SIGINT and assert receipt + CleanupError + marker), and register the `late` pid as a stray (m-1).

### Notes
- N-1 `wait()` (`:592-604`) reads the exit code before checking `_abort`. After a watchdog abort it can return the cleanup-induced code (for example −2) instead of raising `SessionAborted`. `launch_px4_gazebo` still ends in `SessionAborted` through `sleep`.
- N-2 `_rc` (`:570-574`) can raise `ChildProcessError` if the watchdog thread's final reap (abort path) happens between its `returncode` check and `waitid`. This is cosmetic: `__exit__` still reports the abort. `waitid` would also raise if other code in the runner reaps (SIGCHLD = SIG_IGN, `os.wait()`, an asyncio FastChildWatcher).
- N-3 After the try in `__exit__`, three steps are unguarded: a handler restore that raises, a marker write failing with an `OSError` other than `FileExistsError`, and a `KeyboardInterrupt` during the fallback `members()` call. Any of them would skip the receipt and the lock close. The handlers are restored before the marker and receipt are written, so a second SIGTERM in that window takes the previous action.
- N-4 If the final scan is not empty, no launch is reaped and the receipt's `exit_codes` are all `null`, even for launches whose code `_rc` already knew.
- N-5 The docstring (`:30-31`, `:422-425`) says an identity refusal or a signalling error "leaves a leak marker that blocks the next session". The marker blocks only while a listed identity is alive, so a refusal-only marker is cleared by the next entry at once (P12). That behaviour is right; the sentence overstates it. There is no helper to clear a marker by identity.
- N-6 The lock dir and file are created on refused entries, and a stale marker is renamed to `.cleared-*` before the contention checks run (`:495-511`). This is infrastructure, not session output.
- N-7 The journal is written after `Popen` returns. A SIGKILL between fork and fsync loses the identity; that cannot be avoided without an exec shim.
- N-8 Only UDP 14540 is watched during the session. A foreign gzserver started after `__enter__` could share TCP 4560 (the plugin sets `SO_REUSEPORT`) without being seen.
- N-9 Zombies: up to 30 spawn zombies are held per episode until the final reap (P13: all reaped, `unreaped []`). The `test_survivor…` immune process stays a zombie in the test process; that is harmless, and it is the `ResourceWarning` in the test output.

## 3. Acceptance criteria (round 1) re-assessed

| # | Criterion | Status |
| --- | --- | --- |
| 1 | Context manager owning PX4/gzserver/spawn handles; per-episode fresh external rootfs/world; exclusive instance lock | **Partially met.** Handles, fresh rootfs/world, the flock, PX4's fcntl lock, UDP 14540 contention and the leak marker all work and are tested. Ownership still ends for the *next* episode when a multi-episode runner is ended by `timeout` (R2-M1), and an unrecorded launch is possible in the start window (R2-m1) |
| 2 | Fresh/no-overwrite paths; contention fails without creating anything | **Met** (M01 killed by three tests). Residue: the lock file and marker rename on refusal (N-6) |
| 3 | Unrelated sentinel survives normal/exception/timeout cleanup; nothing outside the session signalled | **Met.** Sentinels survive in every test and probe; `reap_journal`, zombie leaders and the marker logic signal only identities owned by the session |
| 4 | PID start-time tracking, reaping, leaks detected | **Mostly met.** WNOWAIT observation, reaping after the final scan, the survivor → CleanupError path and the marker are all tested. Remaining gaps: R2-m1 (clean receipt for an unrecorded launch), R2-m2 (declared limit understated during cleanup), R2-m3 (`reap_journal` single scan) |
| 5 | No `sitl_run.sh`, no shared build rootfs, no global pkill/killall | **Met.** `validate_argv` is tokenised (quoted forms pass, n-8); the AST test is non-vacuous |
| 6 | Every rejection test has a positive control; checks non-vacuous | **Mostly met.** Every new rejection has a positive control, and all r1 informative mutants are killed. Gaps: R2-m6 (6 informative new-code survivors); m-1 leftovers on regression |
| 7 | Import starts nothing; no live-verification claim | **Met and now tested behaviourally** (M04 killed). `STATUS` flags are all False; the docstring says NOT_VERIFIED live |

Re-review is needed after R2-M1 is repaired, with the MINORs fixed or recorded as open.

## 4. What I ran (CPU only: `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13`, `timeout -k`)

| Run | Result |
| --- | --- |
| Host check | no gzserver/gzclient/gazebo/px4 process; no listener on the 7 instance ports; `/tmp/px4_lock-0` absent; cpu4-5 offline; kernel crash/MCE count 0 at the end of the boot's log |
| Repository test file, the command from the brief | 38 tests OK in 17.3 s; no `/tmp/owned_session_*` left |
| Harness (`harness_test.py` = the test file with the module path taken from `OWNED_MODULE` and `ROOT` fixed; temp dirs under `tmp/`) on the original copy `orig/` | 38 tests OK |
| 46 mutants (`make_mutants.py`, `run_mutants.sh`, logs in `mutlogs/`) | 35 killed, 11 survive (6 informative, R2-m6); no hang (slowest 47 s, M07) |
| P1 `probes/p1_sigterm.py` (plain) | TERM and HUP: child dead, receipt `terminated`, no leak |
| P1b, same script (`swallow`) | runner continued to episode 1, SIGKILLed; episode 1 child alive, no receipt or marker (R2-M1) |
| P1e `probes/p1e_sigterm_during_cleanup.py` | SIGTERM during cleanup dropped; episode 1 child alive after SIGKILL (R2-M1) |
| P1c `probes/p1c_signal_in_start.py` | unrecorded live launch, receipt clean (R2-m1) |
| P1d `probes/p1d_natural_race.py` | 3/20 natural trials left an unrecorded live launch (R2-m1) |
| P2 / P3 (r1 probes, copied) | `gone`; no exception, G dead, receipt written, lock released |
| P4 / P5 `probes/p4_p5_escapes.py` | P4 repaired; P5 still missed (declared limit) |
| P5b `probes/p5b_daemonize_during_shutdown.py` | a 1.0 s escape during cleanup missed, `leaked []` (R2-m2) |
| P6 `probes/p6_watchdog_error.py` | abort `watchdog_error`, child dead, CleanupError, marker |
| P9 `probes/p9_reap_journal_fork.py` | `reap_journal` reports `alive_after []`, D alive (R2-m3) |
| P10 `probes/p10_handlers.py` | handlers restored on 6 paths; failed `__enter__` leaves the handler and lock fd (R2-m5) |
| P12 / P13 `probes/p12_p13_marker_cost.py` | refusal-only marker cleared at once (N-5); sample cost 5.5 ms median (n-3) |
| n-8 `probes/n8_validate_argv_out.txt` | quoted pkill/killall accepted |

## 5. Cleanup of my own artefacts
- Mutant-harness leftovers, each identified by argv (a `notes=` path under `review_r2/tmp/owned_session_*`) and by `OWNED_MODULE` pointing at my mutant, then killed with `signal_verified` (pid + start):
  - 2127973 (M07)
  - 2135526 and 2135529 (N05)
  - 2136017 (N10)
  - 2136485 (N11)
  - 2141536 (N26)
  - 2146925 (N43)
- Probe children identified by the argv token `r2probe-*` or the environment marker `R2PROBE=*`; each probe killed its own at the end. A final scan found none left (`kill_harness_leftovers.py`; `common.mine`/`mine_env`).
- I did not signal any other process. A `step_mutants_r2.py` process of another session was running on the same cores; I left it alone.
- I removed the probe and harness sandboxes under `review_r2/tmp/` after recording their outputs (`probes/*_out.txt`, `mutlogs/`).
- Another session changed the worktree during this review: `git status` at the end shows ` M docs/EXPERIMENTS.md`, which was not modified at the start. I did not write it. The two reviewed files still had the SHA-256 values listed at the top.
