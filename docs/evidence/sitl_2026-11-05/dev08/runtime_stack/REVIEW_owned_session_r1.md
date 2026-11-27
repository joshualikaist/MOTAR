# REVIEW r1: dev08 owned SITL session (tools/sitl_owned_session.py, tests/test_sitl_owned_session.py)

- Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus), fresh context, read-only on all repositories.
- Date: 2026-11-05.
- Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `9051cea552f9c408824ac34baf9883e8d25e6024`.
- Reviewed bytes (both untracked):
  - `tools/sitl_owned_session.py` sha256 `bcfda3996e0275cbda4b0784e681c2d4c8c5323ab69fb06ddd8c3891588c0106` (692 lines)
  - `tests/test_sitl_owned_session.py` sha256 `a74cf7a08e046b6396bb6ad4b0b05b1d0f8256d0d3f48b1ba5066b9ea992151c` (616 lines)
  - `tools/sitl_s3b_closed_loop.py` unchanged against HEAD (`git diff --stat HEAD` empty), sha256 `d170b027...84b4`.
- Contract: `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md`, row "Owned session" and the paragraph after the table.
- No PX4, gzserver, gz or gazebo process was launched. No instance-0 port was touched, the GPU was not used, and no repository file was edited. All probes and mutants live in this directory.

## Verdict: FAIL (no BLOCKER, 4 MAJOR)

The core safety property holds: I found no realistic path by which cleanup signals a process outside the session. Membership needs a matching pid and start time. Signals go through a pidfd that is opened before the start time is re-read. Every sentinel survived normal, exception and deadline cleanup. Each rejection test has a positive control. The direct launch plan's argv and the environment it adds match `sitl_run.sh` + `setup_gazebo.bash` and the echo in a historical S3b log (the intended differences aside).

The verdict is still FAIL, on the acceptance criteria themselves:

- Owned processes outlive a runner that is ended by `timeout`, SIGTERM or SIGHUP (M-1).
- An owned survivor can get a clean receipt (`leaked: []`, no CleanupError) in two scenarios I reproduced without injecting anything (M-2). This contradicts the module's own claim "a survivor is a CleanupError".
- A single unexpected errno aborts the cleanup half-way, with no receipt. An exception in the watchdog silently removes the deadline and RSS bound (M-3).
- Eleven of 21 single-change mutants pass the whole test file, nine of them informative. They include disabling the leak report, disabling the unreaped report, creating the episode directory before the contention checks, and starting a process at import (M-4).

All of these can be fixed inside the module and its tests.

## Findings

### M-1 MAJOR: owned processes survive external termination of the runner, and no receipt is written
- Where: `tools/sitl_owned_session.py:406-407` (`start_new_session=True`), `:548-586` (cleanup only in `__exit__`). No signal handling anywhere in the module.
- Scenario (probe P1, reproduced): a runner holds a session with one owned child. It is ended the way AGENTS.md prescribes for long runs (`timeout -s TERM -k 3 1.5 python runner.py`, and again with `-s HUP`). The runner dies at once (rc 124) and `__exit__` never runs. The owned child is in its own session, so neither `timeout`'s process-group signal nor the runner's death reaches it. It stayed alive (`alive=True` for both SIGTERM and SIGHUP) and `session.json` was not written. The pid and start time existed only in the dead runner's memory, so nothing can clean up by verified identity afterwards. A runner killed by SIGKILL or the OOM killer behaves the same way. With real px4 + gzserver, this leaves a lockstep simulator running with no time limit on a host that must stay idle.
- What limits the damage: the next `__enter__` refuses, because the survivors are named `px4`/`gzserver` and hold probed ports.
- Suggested fix:
  1. While a session is open, install SIGTERM and SIGHUP handlers in the main thread that raise an exception, so `__exit__` runs. Restore the previous handlers on exit.
  2. Append each launch (role, pid, start ticks, boot_id) to a journal in the episode directory at `start()`, before returning, so a separate reaper can clean up by identity after a SIGKILL.
  3. Optionally, give direct children `PR_SET_PDEATHSIG=SIGKILL` through a reviewed exec shim (for example `setpriv --pdeathsig KILL`), not `preexec_fn`, because the watchdog thread runs. Or run the launches in a transient cgroup and use `cgroup.kill` (kernel 5.15 has it).
  4. Add a test that sends SIGTERM to a runner subprocess and asserts that its owned child is gone and the receipt exists.

### M-2 MAJOR: leak detection has blind spots; the receipt reports `leaked: []` while an owned process is alive
- Where: `:495-510` (`_reap` is called inside `_wait_gone`), `:117-148` (the session-id scan runs only while the leader still has its recorded start), `:471-493` (0.25 s sampling), `:546`. The docstring at `:11-15` and the class docstring claim that a survivor is a CleanupError.
- Scenario A (probe P4, unmodified module, nothing injected):
  1. Leader L exits on SIGINT. Its child C is in L's session and, on SIGINT, forks D (D ignores SIGINT) and exits.
  2. `_wait_gone` reaps L. From then on `members()` skips the session-id scan for L, because `table.get(L)` is None.
  3. D is not yet in `_known`, and its parent is now the subreaper (ppid 3221). The post-grace re-scan therefore misses D.
  - Result: D stayed alive with **sid equal to L's pid**. No action names D, the receipt shows `leaked: []`, and `__exit__` raises nothing.
  - The members() docstring states the right invariant: the pid stays allocated while it is used as a session id, so an *unreaped* leader makes the scan valid. Reaping leaders before the last scan throws that invariant away.
- Scenario B (probe P5, unmodified module, production `sample_s=0.25`): a child does `setsid(); fork()` and exits within one sampling period (an ordinary daemonize). D was never in `_known`, survived cleanup, and the receipt again showed `leaked: []`.
- Neither pattern is expected from the current gzserver/px4 binaries: neither forks during shutdown, and px4's `rcS` shell stays in px4's session. Shell wrappers and helpers do produce these patterns, and the receipt is the evidence a later analysis relies on.
- Suggested fix:
  1. Do not reap leaders during cleanup. Observe their exit with `os.waitid(P_PID, pid, WEXITED | WNOHANG | WNOWAIT)` (available on Python 3.8). Reap only after a final scan finds no live process with `sid` in the leader set, then scan once more.
  2. For B, either track membership in a cgroup, or set `PR_SET_CHILD_SUBREAPER` on the runner and treat adopted children born after `_t0` as owned.
  3. Whatever remains, state it in the receipt (for example `leak_detection: "sampled_descendants_0.25s"`) and in the docstring, instead of "a survivor is a CleanupError".
  4. Add a P4-style regression test.

### M-3 MAJOR: cleanup and the watchdog are not exception-safe; one unexpected errno masks the result
- Where: `:175-193` (only `ProcessLookupError` counts as "gone"), `:512-519`, `:521-546`, `:548-574` (`os.close(self._lock_fd)` and `write_new(session.json)` run only if `_terminate` returns), `:471-493` (no try/except in `_watch`).
- P2 (kernel fact, Python 3.8.20 ctypes path, kernel 5.15): a reaped process whose pid number is still used as the sid/pgid of a live process makes `kill(pid, 0)` return ESRCH, but `pidfd_open` returns **EINVAL**. `signal_verified` then raises `OSError(EINVAL)` instead of returning "gone". EPERM (a setuid descendant) and EMFILE escape the same way.
- P3 (consequence, end to end): an owned child C is its own session leader with a grandchild G. The leader reaps C between the member snapshot and C's signal; in the probe a 0.3 s pause after signalling the leader stands in for scheduling delay. `__exit__` raised a bare `OSError EINVAL`. The signal loop stopped before G, so **G stayed alive**, **no session.json was written**, and the lock fd was never closed: a second session in the same process got `LockBusy`. In production the window is narrow. The watchdog-abort path makes it more reachable, because the main thread's `poll()` can reap a leader while the watchdog thread is signalling.
- P6: a single exception in the watchdog thread kills it. The probe made `snapshot()` raise `PermissionError` on that thread, as `/proc` mounted with `hidepid=1` would. The thread died with only a traceback on stderr. The 0.5 s deadline never fired, the body ran 2.0 s, `check()` passed, nothing was raised, and the receipt showed `abort: null, samples: 0`. The 240 s deadline and the 12 GiB cap that the contract requires were silently gone. (`foreign_processes` catches `PermissionError`; `proc_stat` does not.)
- Suggested fix:
  1. In `signal_verified`, map EINVAL/ESRCH from `pidfd_open` to "gone" after a `/proc` check.
  2. Catch any other `OSError` per target, record `error:<errno>`, keep going, and report it as a CleanupError.
  3. Put `_terminate` and the receipt write in try/finally, so the receipt is always written with any internal error. Then decide explicitly whether a failed cleanup keeps the lock (see m-5).
  4. Wrap the `_watch` loop body: on an exception set `_abort = "watchdog_error"` and terminate.
  5. Check `self._watchdog.is_alive()` in `check()`, `sleep()` and `wait()`, and record it in the receipt.
  6. Make `proc_stat` treat `PermissionError` like a vanished process, or fail closed explicitly.

### M-4 MAJOR: the tests do not guard several acceptance criteria (11 of 21 single-change mutants survive, 9 of them informative)
Mutants live in `mut/` and the runner is `run_harness.sh`. The harness is the repository test file with two changes: the module path comes from an environment variable, and the git-tree probe uses a temporary `.git` (see m-2). Results are in `mutants_result.txt`.

| Mutant | Change | Result | Criterion left unguarded |
| --- | --- | --- | --- |
| M01 | episode dir `mkdir` moved before the foreign/port checks | **survives** | 2: the weak assertion `test:399` (`lexists(dir) and rootfs.exists()`) passes when the dir exists without rootfs. Only the LockBusy case checks `lexists` directly (`test:433`) |
| M02 | `_terminate` returns `{}` (leak report disabled) | **survives** | 4: no test has a survivor. P8 shows the real code does report one when it sees it |
| M03 | `unreaped = []` | **survives** | 4 |
| M04 | `subprocess.Popen` at module level | **survives** | 7: `test:599-605` checks only top-level AST node types, and an `Assign` may start a process. A behavioural import check found 0 threads/children for the real module and 1 child for M04 |
| M05 | UDP probe with `SO_REUSEADDR` | **survives** | 1/2: the test's UDP holder (`test:449-450`) lacks `SO_REUSEADDR`, but pymavlink's `udpin` sets it (mavutil.py:1000). P7 shows the real probe catches such a holder; M05 would not |
| M06 | spawn accepted when rc ≠ 0 | survives | launch plan: no fake spawn exits non-zero |
| M14 | RSS of leaders only | survives | RSS cap over the owned *tree* |
| M15 | TCP probe without `SO_REUSEADDR` | survives | no TIME_WAIT positive control; P7b shows it would refuse after a reuse-listener TIME_WAIT |
| M19 | `include_self_rss` ignored | survives | tests always pass `False` |
| M07, M08, M09, M10, M11, M12, M13, M16, M20 | no SIGKILL; no start check; leader at any start; known ignored; no flock; no foreign check; no git check; no deadline; known start ignored | killed | — |
| M17 | lock fd never released | killed, **by hanging** | see m-1 |
| M18 | `validate_argv` checks argv[0] only | survives | see n-8 |
| M21 | final `popen.wait` removed | survives (equivalent: `_wait_gone` polls) | — |

- Suggested fixes:
  1. Assert `not os.path.lexists(episode_dir)` in `assert_refused`.
  2. Add a leak test: patch `signal_verified` so that one identity is never really killed, then assert CleanupError, a non-empty `leaked`, `unreaped_roles == [role]`, and the process still alive (kill it in tearDown). P8 is a template.
  3. Add an import test that imports the module in a subprocess and asserts no children and one thread.
  4. Give the UDP holder `SO_REUSEADDR`.
  5. Add a TIME_WAIT positive control with a reuse listener.
  6. Add a spawn that exits 1 without the not-ready phrase.
  7. Add an RSS test where a grandchild does the allocation.
  8. In the launch-sequence failure case, assert the real gzserver identity is dead, not only the self-reported `leaked == []` (`test:251`).

### m-1 MINOR: test robustness. A lock regression hangs the suite, and teardown has no fallback for session-launched processes
- `test:434-439`: the holder uses blocking `flock(LOCK_EX)` and `holder.stdout.readline()` has no timeout. Under M17, `test_lock_contention` hung until my outer `timeout` (25 s alone, 240 s in the batch). PUBLIC_CPU would stall, not fail. Fix: `LOCK_EX | LOCK_NB` in the holder, plus `communicate(timeout=...)`.
- `test:121-129`: tearDown kills only `self.children`/`self.strays`. Under M07 a SIGINT-ignoring fake launched by the session outlived the test run (pid 2025542, attributed to my harness through its `OWNED_MODULE` environment). I removed it with `signal_verified`. Fix: in tearDown, SIGKILL every identity in the session's `pids()`/`_known` that is still alive.

### m-2 MINOR: `test_git_tree_and_forbidden_root_refused` probes inside the real repository
- `test:417-422` uses `ROOT / "__owned_session_probe__"` with `allowed_root=ROOT.parent`. If the git check regresses, this test creates a directory with `rootfs/`, `logs/`, ... and `session.json` inside the worktree. That violates the tree freeze when PUBLIC_CPU runs during a guarded run. Fix: a temporary directory holding a `.git` file. My harness does this and still kills M13.

### m-3 MINOR: launch-plan differences from `sitl_run.sh` that are not stated, and a plan test that pins little
Verified equal to `sitl_run.sh` + `setup_gazebo.bash` and to the echo in `/home/fair/motar_tmp/sitl_s3b_20261105/dev06_205/ep01_sitl.log`:
- gzserver argv `[gzserver, world]`;
- spawn argv (`--verbose --spawn-file=<models>/navrl_quad/navrl_quad.sdf --model-name=navrl_quad -x 1.01 -y 0.98 -z 0.83`);
- px4 argv `[<build>/bin/px4, -d, <build>/etc]` (log: `SITL COMMAND: ".../bin/px4" -d ".../build/px4_sitl_default"/etc`);
- `GAZEBO_PLUGIN_PATH` and `GAZEBO_MODEL_PATH` with the leading `:`, the `LD_LIBRARY_PATH` append, `PX4_SIM_MODEL`, `PX4_SIM_WORLD`, `HEADLESS`, `CUDA_VISIBLE_DEVICES`.

Differences beyond the intended ones (fresh rootfs, GAZEBO_LOG_PATH, no pkill, bounded retry, no gzclient):
- **cwd of gzserver and of the spawn** is the episode dir (`:618-620`). Historically it was `build/src/modules/simulation/simulator_mavlink`, the `cwd` given to `sitl_run.sh` (`sitl_s3b_closed_loop.py:106`), and `sitl_run.sh` does not change directory before gzserver. Because `LD_LIBRARY_PATH` holds an empty element (`...::<build>/build_gazebo-classic`, in the historical run as well), the dynamic loader also searches that cwd. That is now the fresh episode dir, and for px4 the fresh rootfs.
- **Spawn acceptance** needs `rc == 0` *and* the phrase absent (`:654`). `sitl_run.sh` stops retrying whenever the phrase is absent, whatever the exit code. Stricter is acceptable, but say so.
- `NO_PXH` is no longer exported. Harmless: only `sitl_run.sh` reads it; `rcS` does not.
- The `VERBOSE_SIM` → `--verbose`, `ROS_VERSION=2` → `-s libgazebo_ros_*.so` and `PX4_SITL_WORLD` branches are dropped. None of these variables is set on this host, so the historical runs had the same behaviour.
- The historical `LD_LIBRARY_PATH` started with `/home/fair/miniconda3/envs/aerialgym/lib/python3.8/site-packages/cv2/../../lib64:`, which the runner's `cv2` import puts into `os.environ`. The directory does not exist. Equality now depends on what `base_env` the caller passes.

The test `test:504-522` pins argv, the px4 cwd, `GAZEBO_PLUGIN_PATH`, `GAZEBO_LOG_PATH` and `CUDA_VISIBLE_DEVICES`. It does not pin `GAZEBO_MODEL_PATH`, `LD_LIBRARY_PATH`, `PX4_SIM_WORLD`, `HEADLESS`, the gzserver/spawn cwd, or the append-versus-prepend order. Fix: list these differences in the docstring and in the receipt, pin them in the test, and preferably compare against `sitl_run.sh`'s own lines, as `test_spawn_pose_matches_historical_sources` already does for the pose.

### m-4 MINOR: `runtime_binding` covers the contract minimum but misses boot and plugin inputs
- `:679` is a non-recursive glob. It binds 39 of the 40 `.so` under `build_gazebo-classic` and misses `OpticalFlow/libOpticalFlow.so`.
- Files that define ports and the simulator at boot are not bound: `px4-rc.mavlink`, `px4-rc.simulator`, `px4-rc.params`.
- The `model://iris/meshes/*` resources that `navrl_quad.sdf` uses are not bound.
- `model_sdf` takes the first `navrl_quad.sdf` on `GAZEBO_MODEL_PATH`, as `sitl_run.sh` does, with no check that it is the PX4-tree original the contract names.
- Fix: hash a manifest of `build/etc` and a recursive plugin glob, and assert (or pin by SHA-256) that the SDF is the PX4-tree one.

### m-5 MINOR: after a failed cleanup the lock is released and admission is narrow
- `:553-555`: the lock is closed before `leaked` is examined. The next `__enter__` refuses only on the 4 process names and 7 ports, so a leaked `gz`, `sh` or other helper would not block the next episode. Fix: on CleanupError, leave a durable `dirty` marker beside the lock (or keep it held) until a reaper or a human clears it.

### m-6 MINOR: port exclusivity is a one-time probe
- `:375-377`: the ports are probed once at `__enter__` and are not held. Two UDP sockets that both set `SO_REUSEADDR`, such as two pymavlink `udpin:0.0.0.0:14540` links, share 14540 silently. One that binds after `__enter__` is never seen.
- The TCP rationale in the `busy_ports` docstring ("TIME_WAIT ... do not count") is conditional. It holds only when the earlier listener set `SO_REUSEADDR`. P7 shows a TIME_WAIT from a non-reuse listener still reads busy; P7b shows a reuse listener's TIME_WAIT passes. It is true for the actual listeners: the Gazebo mavlink plugin's 4560 listener sets `SO_REUSEADDR` and `SO_REUSEPORT` (`mavlink_interface.cpp:129,136`), PX4 1.14 is the TCP *client* (`SimulatorMavlink.cpp:1114-1134`, `connect()` at 1134), and boost-asio acceptors set the option by default.
- Fix: state the condition. Have the watchdog count sockets on `:14540` in `/proc/net/udp` (more than one means contention), or have the runner's link bind without `SO_REUSEADDR`.

### Notes
- n-1 Identity limits, theoretical only:
  - Start time is in USER_HZ ticks (10 ms), so a pid reused within one tick would pass. With `pid_max` 4194304 that is negligible.
  - `snapshot()` is not atomic. If a member's pid were reused during one /proc scan, an unrelated child could look like a descendant (`:138-146`). That needs a full pid wrap within milliseconds.
  - A main-thread `poll()` racing the watchdog's signalling can produce a false `refused_identity` CleanupError. That errs on the safe side.
- n-2 `start()` uses `p.kill()` (`:412`), which is `os.kill` on an unreaped child of this process. That is safe, but it contradicts the docstring's "No ... os.kill" (`:15`).
- n-3 Watchdog cost, measured: `snapshot()` takes a median 4.6 ms (p90 4.9, max 5.8) over 525 processes. At 0.25 s that is a 1.8 % duty cycle, holding the GIL in the runner process. Record it for real-time acceptance runs.
- n-4 A watchdog abort kills the children but does not interrupt a main thread blocked in a syscall (for example a pymavlink receive with no timeout). Until the main thread reaches `__exit__`, there is no receipt and the lock stays held. Callers need bounded receives.
- n-5 Rootfs isolation is sound:
  - PX4 creates `etc → build/etc`, `test_data`, `log/` and `eeprom/` itself (`platforms/posix/src/px4/common/main.cpp:318-328, 454-465`).
  - `rcS` reads `parameters.bson` only from cwd (`rcS:66-100`).
  - So every episode boots with first-boot `AUTOCNF=yes` defaults. The historical runs imported 29 persisted parameters (`ep01_sitl.log`: "INT32:13, FLOAT:16"). This is intended by the contract, but it is a behaviour change to record when comparing with dev01–dev06.
  - PX4 also keeps global instance-0 state outside the rootfs: `/tmp/px4_lock-0` (an fcntl lock) and `/tmp/px4-sock-0`. `F_GETLK` on the lock file would be a cheap extra admission check.
- n-6 The lock directory and lock file are created even when the session is refused (`:243-244`). That is infrastructure, not session output, but strictly it is "something created".
- n-7 `module_sha256` is computed at exit (`:559`): it hashes the file as it is then, not the code that ran. Hash at import and compare the two at exit.
- n-8 `validate_argv` (`:287-292`) is a basename denylist applied to single elements, so `["bash", "-c", "pkill -x gazebo"]` passes and M18 survives. Low risk, because the plan is built in code.
- n-9 The real import starts nothing: 0 children and 1 thread, checked behaviourally. `STATUS` has `live_verified`, `execution_supported` and `launch_ready` all False, and the docstring says NOT_VERIFIED live. Criterion 7 is met by the code; only its test is weak (M04).
- n-10 During this review another session added further untracked files to the same worktree. I did not review them: `tools/sitl_dev08_{link,step,attempt}.py`, `tools/sitl_s3b_dev08_prefix.py`, their tests, `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md` and `docs/evidence/sitl_2026-11-05/dev08/runtime_stack/`. At the end of the review the two reviewed files still had the SHA-256 values listed above.

## What I ran (all CPU-only: `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13`, `timeout`)

| Run | Result |
| --- | --- |
| Host check before the runs | kernel crash/MCE count 0; no listener on 14540/14580/18570/14280/13030/4560/11345; no gzserver/gzclient/gazebo/px4 process |
| Repository test file, the command from the brief | 29 tests, OK, 4.46 s. No leftover processes or `/tmp/owned_session_*` directories afterwards |
| Harness on the original copy (`orig/`) | 29 tests, OK |
| 21 mutants (`make_mutants.py`, `mutants_result.txt`) | 9 killed by failures, 1 killed by a hang (M17), 11 survive (9 informative, see M-4; M18 → n-8; M21 equivalent) |
| P1 `probes/p1_sigterm.py` | SIGTERM and SIGHUP via `timeout`: owned child alive, no session.json (M-1) |
| P2 `probes/p2_pidfd_einval.py` | Python 3.8.20 has no `os.pidfd_open`, so the ctypes path runs. A reaped pid pinned as sid: `kill` → ESRCH, `signal_verified` raises `OSError EINVAL` (M-3) |
| P3 `probes/p3_einval_cleanup.py` | `__exit__` raises `OSError EINVAL`, grandchild alive, no session.json, LockBusy in the same process (M-3) |
| P4/P5 `probes/p4_p5_escapes.py` | unmodified module: survivor alive, `leaked: []`, nothing raised, in both scenarios (M-2) |
| P6 `probes/p6_p8.py` | watchdog dead, 0.5 s deadline not enforced, `abort null, samples 0` (M-3) |
| P7/P7b `probes/p6_p8.py`, `probes/p7b_timewait_reuse.py` | UDP holder with `SO_REUSEADDR` detected; TIME_WAIT passes only behind a reuse listener (m-6) |
| P8 `probes/p6_p8.py` | a survivor that is visible *is* reported: CleanupError, `leaked [pid]`, `unreaped ['px4']` |
| Import check | real module 0 children; M04 1 child |
| Snapshot timing | median 4.6 ms over 525 processes (n-3) |

Cleanup of my own artefacts:
- One leftover fake from the M07 harness run (pid 2025542, start 27323730): identified through its `OWNED_MODULE` environment and killed with `signal_verified`.
- Three `/tmp/owned_session_*` directories left by the hung M17 harness runs: identified through their lock-holder episode `a`, then removed.
- Every probe child was killed by verified identity at the end of its probe.
- No other process was signalled.

## Acceptance criteria

| # | Criterion | Status |
| --- | --- | --- |
| 1 | Context manager owning PX4/gzserver/spawn handles; per-episode fresh external rootfs/world; exclusive instance lock | **Partially met.** Handles, fresh rootfs/world and the flock work across processes and within one process (tested). Ownership ends when the runner is externally terminated (M-1). The lock binds only OwnedSession users and the ports are probed once (m-6) |
| 2 | Fresh/no-overwrite paths; contention fails without creating anything | **Met in code** (checks run before `mkdir`; `x`-mode files; `exist_ok=False`). Only partially guarded by tests: M01 survives. The lock file is created on refusal (n-6) |
| 3 | Unrelated sentinel survives normal, exception and timeout cleanup; nothing outside the session is signalled | **Met.** Tested with sentinels in a new session, in the runner's own session, and started mid-session; no path found that signals an unowned process (n-1 theoretical limits) |
| 4 | PID start-time tracking, reaping, leaks detected | **Partially met.** pidfd + start-time check and reaping work (tested; P8). Leak detection misses P4 and P5 with a clean receipt (M-2), aborts on an unexpected errno (M-3), and is untested (M02/M03 survive) |
| 5 | No `sitl_run.sh`, no shared build rootfs, no global pkill/killall | **Met** (code, `validate_argv`/`validate_plan`, AST test with a non-vacuous legacy control). The unstated cwd/spawn-acceptance differences are in m-3 |
| 6 | Every rejection test has a positive control; checks non-vacuous | **Partially met.** Every rejection has a positive control, but 9 informative mutants survive (M-4) and one regression hangs instead of failing (m-1) |
| 7 | Import starts nothing; no live-verification claim | **Met in behaviour** (import check; STATUS all False; NOT_VERIFIED docstring). The test is syntactic only (M04 survives) |

Re-review: needed after M-1 to M-4 are repaired (m-1 to m-6 fixed or recorded as open).
