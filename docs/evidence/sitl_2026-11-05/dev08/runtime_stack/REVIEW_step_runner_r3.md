# dev08 runtime stack (step / attempt / runner / amendment2): independent review r3

Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context. I worked read-only on every
repository and on the PX4 tree. Date 2026-11-05, 19:07–19:30 KST.
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `9051cea`. The reviewed
files are uncommitted.

## Verdict: **FAIL** (narrow: text only)

There is no BLOCKER, and every MAJOR left is text.

**What holds.** The round-2 code repairs hold on the submitted bytes:
- **Finish.** The runner finishes the attempt explicitly. A source change during the last member now ends ABORTED.
- **Signals.** A signal during the session cleanup is held until the cleanup ends, then delivered again, and the
  unit stops.
- **NaN.** A NaN value is kept in the ledger as null and stops the attempt.
- **Start phase.** The position is checked for finite values.
- **Final record.** A failing kernel-fault read still writes final.json.
- **SOURCES.** The list now covers the eight modules that were missing.
- **Packages.** pymavlink and rl_games are recorded.
- **Bound.** The approval carries `skip_max_s`, checked to lie in [0.02, 0.5], and the value reaches the
  TelemetryGate.
- **Idle cores.** The runner's cores are checked for idleness when the attempt opens and before every member, and
  the result is recorded.

The probes P1–P7 confirm each of these.

**What still fails.** The decision rests on one MAJOR item:
- amendment2 is a `docs/plans/prereg_*.md` path, so it becomes immutable once committed. It still contains
  statements that the code or the records contradict, and the round-2 repairs created two of them.
- The README still gives two sets of numbers without a machine-readable record (MJ1-r3).

These are line edits. No code change is needed. Once they are made, I would expect PASS_WITH_LIMITS, with the
MINOR items below as limits.

**Moving target.** I copied the submitted files to `review_r3/snapshot/`, and all 9 hashes matched at 19:07:43. At
19:24:58 two of them had changed in the worktree. I did **not** review the new bytes.

| File | Submitted | Worktree at 19:25 | mtime |
| --- | --- | --- | --- |
| amendment2 | 10866de9 | 6aea6e58 | 19:19:52 |
| runtime_stack/README.md | 469caf63 | 8ac8cc18 | 19:19:52 |
| tools/sitl_dev08_link.py (dependency) | aa2551a7 at start | adcd73df | 19:19:52 |
| tests/test_sitl_dev08_link.py (dependency) | — | c4b8f1cf | 19:20:19 |

- **amendment2 and README.** In both new versions, the truth-lag wording I flag in MJ1-r3 (e) is rewritten
  ("…or 4, 8, 12 or 16 ms before it…"). A grep of the new bytes shows that items (a)–(d) and (f) are still there.
- **Link.** My `execute_paths_r3` and `binding_count_r3` probes ran at 19:23–19:24 through symlinks. They may
  therefore have used link adcd73df. They exercise the gate only through the recorded `skip_max_s` keyword, and the
  binding goes through the owned session (c26f0eac, unchanged), so I keep their results.
- **Unit tests.** The unit tests ran at 19:08 on the submitted bytes.

## Repair table (round-2 findings)

| r2 | Status | Evidence (snapshot bytes unless stated) |
| --- | --- | --- |
| MJ1 amendment2 statements | **PARTIAL** | (a) "62 files" is correct: `probes/binding_count_r3.out` shows 62 recorded, 40 plugins, 7 model resources, and the template included. (b) It now says "rcS also sources further scripts … not bound"; installed `init.d-posix` has px4-rc.{mavlink,params,simulator}, all bound. (c) SOURCES now includes the 8 modules (attempt :41–44), so the r2 executed-module trace list is covered. (d) "the last member included" is true (P1). (e) Deferral and re-delivery are true (P2). The 35 s worst case is stated. (f) 463/500 and 198 are correct. (g) The "same burst" wording is replaced by "receipt window" (l.121–122). (h) BAD_QUATERNION is now listed (l.94). (i) The runner-side cause and the 218.4 ms are stated (l.162–164). (j) is NOT repaired: see MJ1-r3 (b). Several new contradictions exist: see MJ1-r3. |
| MJ2 idle-host precondition | **REPAIRED** (limits in m3-a) | `idle_check` (attempt :176–181) runs in `open()` (:268–270) and in `check_host()` (:322–328), which is called before every member (runner :458). It is recorded as `host_idle_at_open` and per record as `host_idle_before`. amendment2 l.186–189 states that the cores are exclusive and that AGENTS.md's allowance does not apply. Probe: a busy core reads 0.0 and fails; a 20 % duty job reads 0.79 and fails (`probes/idle_probe.out`). Mutant A35 is caught (open refuses), and so is A36 (check_host aborts). |
| m-a last-member source change | REPAIRED | Runner :472–473 compares `git_state` after `run()` and before `append()`. P1: a change during member 41 gives ABORTED and `last_status SOURCE_CHANGED`, with 42 recorded. P1b: a change during member 40 gives the same, with 41 recorded. The test covers members 3 and 41, and mutant D05 is caught. |
| m-b signal swallowed during cleanup | REPAIRED (in the session dependency) | The session's `_on_signal` defers the signal during cleanup or `_critical`, and `__exit__` re-raises the first signal after the receipt, through the runner's handler. P2 (real OwnedSession, fake processes): SIGTERM arrives during cleanup (`during_cleanup: true, deferred: true`); 1 member ran, then RunnerTerminated, and ABORTED RUNNER_EXCEPTION. P2b: SIGTERM inside the session body also gives RUNNER_EXCEPTION, so the r2 misattribution is gone. |
| m-c sources and external code | REPAIRED (two NOTEs) | Eight modules were added. `package_manifest` (runner :379–389) records path, version and a .py hash. Probe: pymavlink 2.4.50 from the px4 environment, 131 files, 0.08 s. **rl_games `version: null`** (see MJ1-r3 c). pytorch3d is imported but not recorded. Untracked imported files remain unbound (n3-a). |
| m-d paths without final.json | PARTIAL | `real_deps()` is now inside the try (:453), and the handlers are installed before `open()` (:419–422). `_final` tolerates a failing fault read, and `append` records FAULT_CHECK_ERROR; the tests and mutants A40 and A41 are caught. **Remaining** (m3-c): signals are not blocked around `append` or `_final`, and `open()` makes the directory before the fault read (P7). |
| m-e NaN drops a ledger line | REPAIRED | `_finite_json` (attempt :184–197) runs before `dumps`. P3: a NaN RTF in member 1 is kept as null, with `record_problems` "non-finite value at record.real_time_factor", then SMOKE_FAILED and 2 lines. The test and mutants A37 and A38 are caught. See m3-d for the edge cases. |
| m-e start-phase finiteness | REPAIRED (untested) | Runner :208–210 and :216–218. Mutants D12 and D13 survive. |
| m-f budgets between members | RECORDED_AS_LIMIT | amendment2 l.195–196 says "checked between members, so a run can exceed them by one member". |
| m-g README log count | PARTIAL | I recounted the 23 logs myself: 22 show `(INT32:13, FLOAT:16)` and `dev07_smoke_70/ep00` shows `FLOAT:6`. The README text is now correct. **No machine-readable record was added**, and the same is true for 218.4 ms (MJ1-r3 f). |
| m-h test gaps | NOT_REPAIRED | Step: the step and step-test bytes are identical to r2 (sha 9b1c9135 and 430ddf89), so M19–M21, M27 and M28 still survive. Attempt: A10, A19, A20, A23, A24, A26, A27 and A29 **still survive** (`probes/mutants_r3_attempt.out`). New gaps are listed in m3-b. |
| m-i approval lacks the bound | REPAIRED (flow untested) | `APPROVAL_KEYS` includes `skip_max_s`, refused unless it is a float in [0.02, 0.5] (:404–406). P6: 0.02 and 0.5 are accepted; 0.019999, 0.5000001, 1, True, nan, inf, "0.1", None and 0.0 are refused. P5: approval 0.15 reaches `TelemetryGate(skip_max_s=0.15)`, and the record's `gate_opts` holds 0.15. **Mutants D01 and D02 survive**: nothing tests the flow (m3-b). FRESH_MAX_S, TRUTH_AGE_S and BURST_S stay code constants, bound through the commit. |
| m-j live assumptions | PARTIAL (unchanged list, NOTE) | Lockstep TCP 4560, (1,1) heartbeat and mode acceptance, and SET_MESSAGE_INTERVAL for id 115 are still not listed. |
| n-a compute_ms note | REPAIRED | amendment2 l.162–164 and README. I re-read `sitl_s3b_closed_loop.py:294–317`: the row ends `compute_ms, visible, *p_true, yaw_true`, so compute is column 21 and `r[-3]` is column 24, p_true y. |

## MJ1 check: each number against its source

| Statement | Source | Result |
| --- | --- | --- |
| 62 bound files, 40 plugins | `probes/binding_count_r3.out` (`real_deps().binding` through the snapshot runner) | 62 / 40 ✓ |
| 27 REAL32 parameters | `sitl_dev08_link.RECORD_PARAMS` | 27 ✓ (counted) |
| 463 / 500 diverge, 198 at k = 1 or 2, range 1–384, harness start → k = 164 | `~/motar_tmp/dev08_runtime_stack_20261105/tolerance_divergence.out` | 463, 117 + 81 = 198, 1–384, `start_1775_first_k` 164 ✓; the script's range is 1500–1999 ✓ |
| 218.4 ms at k = 27, about 6 ms around it, t[28] = t[29] | raw `dev07_70/ep02_steps.npy` column 21 (I re-read it) | k25–30: 6.2, 6.3, **218.4**, 5.7, 5.9, 6.2; t = 38.300 twice ✓. **No record in the evidence set** (MJ1-r3 f) |
| 1 late sample in 23 episodes, 0.136 s, others ≤ 0.04 s | `dev07_lateness.out` | 23 episodes, one over 0.05 (`dev07_70/ep02`, 0.136 s, k 28, one reuse), max of the rest 0.04 ✓ |
| 22 of 23 logs `FLOAT:16`, smoke_70 `FLOAT:6` | the dev07 `ep*_sitl.log` files (I grepped them) | ✓; no record (MJ1-r3 f) |
| 25× step-time rise | r2 `probes/spike_probe.out` median 150.45 ms against about 6 ms (`timing_probe.out` rep 0) | ✓ |
| 1 − 0.95^42 ≈ 0.88; 36 s of waits; 22 s start phase | arithmetic; `TIMING` | ✓ |
| ports, flock, 240 s, 12 GiB, 0.25 s, 20 s grace | `sitl_owned_session.py:84–93` (dependency) | ✓ |
| 0.03 s / 0.003 s truth window, attitude 0.05 s, stall 2 s | `sitl_dev08_link.py:52–57` | ✓ |
| skip bound refused outside 0.02–0.5 s | runner :405 and P6 | ✓ |
| amendment1 unchanged in estimand, cohort, order, stop rules, analysis | amendment1 l.33–52, 59–111, 149–158; `schedule()` unchanged since r2 (attempt diff) | ✓. HOST_NOT_IDLE, SOURCE_CHANGED, FAULT_CHECK_ERROR and non-finite stops are added operational stops. They put into practice amendment1's "idle host", "hash … mismatch … abort" and "host fault" (l.81–82, 149). Neither the estimand nor the analysis changes. |

## New findings

### MAJOR

**MJ1-r3. amendment2 (immutable once committed) and the README still contain statements that the code or the
records contradict.** Fix them before the first commit; this is text only.
- (a) **The stated bound contradicts the approval mechanism.** l.126 says "RUNNER_LATE bounds the difference to
  0.05 s", and l.155–156 says "This implementation sets 0.05 s".
  - Since the r2 repair, the bound is the approval's `skip_max_s`, anything in [0.02, 0.5] (runner :404–406 and
    :465; l.97 and l.169 of the same file). With an approved 0.15 s, l.126 is false.
  - *Fix:* write "bounds the difference to the approved skip bound", and "the default in the code is 0.05 s; the
    approval fixes the value".
- (b) **The kill grace is not reconciled with amendment1** (r2 MJ1-j, not repaired).
  - amendment1 l.151 fixes "timeout kill grace 20s"; amendment2 l.182 proposes `timeout … -k 90 5700`.
  - l.51 mentions the session's "SIGKILL after 20 s", but never says that this is amendment1's 20 s grace and that
    `-k 90` is an extra outer net. Without that sentence, a reader sees a changed cap while l.6 says no stop rule
    changes.
- (c) **"recorded by version" is false for rl_games** (l.110). `package_manifest` reads `__version__`, which rl_games
  1.6.5 does not define. The probe returns `"version": null` (`probes/package_manifest_probe.out`).
  - *Fix:* use `importlib.metadata.version(name)`, or say "path and hash (version where the package defines it)".
- (d) **"All keep the attempt and list unrun members in final.json"** (l.202) **has exceptions.**
  - P7: when the kernel-log read fails at `open()`, an empty attempt directory is left, with no header and no
    final.json. The fixed attempt path `dev08_prefix_attempt01/` (l.176) is then used up.
  - A signal during `_final`'s journalctl call, or between the ledger write and the `prev_sha` update, leaves no
    final.json, or one that does not close the ledger.
  - A member interrupted by a signal has no ledger line and is listed as "unrun", although its episode directory
    exists. l.101 ("kept on failure") does not cover terminations.
  - *Fix:* state these limits, or close them (m3-c).
- (e) **The truth-lag wording is out of date** (l.90–93, l.144–145; README l.48).
  - With BURST_S = 0.003, the +4 ms phase's truth is outside the window, so the gate takes the previous truth,
    16 ms earlier (20 ms stream period).
  - The truth used is therefore the position's own PX4 pass or 4–16 ms before it, not "an adjacent 4 ms cycle".
  - The "-12 to +4 ms" phase comes from the **link** module's review r2 (`dev08_link_20261105/review_r2/REVIEW_r2.md`
    l.128). This runner's r2 did not measure it.
  - The 19:19:52 worktree bytes appear to fix this; I did not review them.
- (f) **README numbers have no machine-readable source.** This is AGENTS.md "Numbers travel with their source" and
  r2 m-g.
  - "218.4 ms at k = 27" (README l.52–53; amendment2 l.162) and the bson counts "22 with (INT32:13, FLOAT:16) …
    FLOAT:6" (README l.26–27) appear in no file under `~/motar_tmp/dev08_runtime_stack_20261105/`.
  - `dev07_lateness.out` has no compute column, and `make_validation.py` binds only that directory.
  - *Fix:* add a small script and output there, for example column 21 at `dev07_70/ep02` k 25–30 plus the per-log
    bson line counts, and cite it.
- (g) (NOTE-level, same file.) l.183 "5400 s plus one member's 240 s plus margin". The 5400 s budget starts at
  `rec.open()`, but `timeout` counts from process start (scene inputs, bridge setup, Policy F load, warm-up, open).
  A member that starts just under 5400 s can therefore meet the timeout's SIGTERM, not the budget check. Both end
  ABORTED with final.json; only the reason differs. Say that the 60 s margin also covers the setup before open.

### MINOR

- **m3-a. The idle check is a single 1 s sample that aborts the attempt; it does not block forever and does not
  pass vacuously.**
  - **Not vacuous, and it never blocks.** A missing or offline core changes `len` and fails. A zero jiffy delta
    gives idle 0 and fails. There is no loop: the check costs 1 s.
  - **Sensitive.** A busy core reads 0.00; a 20 % duty-cycle job reads 0.79; both fail (`probes/idle_probe.out`).
  - **Contaminated samples.** At 19:13, 3 of 3 checks failed while a nice-10 python job of another session ran on
    core 6. Unpinned nice-0 `claude` CLIs were also seen on core 6 (18.5 % CPU).
  - **Thin margin.** At 19:24, with that job gone, 15 of 15 passed, but the minimum was 0.912
    (`probes/idle_series.out`).
  - **Risk over the unit.** Over 43 checks, routine desktop or agent bursts can end the unit with HOST_NOT_IDLE and
    use up the approval.
  - **Contention during a member is not prevented.** The runner (nice ≥ 10 is required) and its PX4/gzserver
    children (which inherit affinity and nice) yield to any unpinned nice-0 process the scheduler puts on
    6, 7, 12 or 13.
  - Running dev07 on the same cores gave a median step of 6.2 ms with 3 of 1318 steps over 100 ms, so the
    arrangement itself is the historical one.
  - *Fix (suggested):*
    - Before a member, re-sample for a bounded time (for example, up to 30 s) and abort only if the cores never
      become idle. A wait before the member does not depend on any outcome, and every sample can be recorded.
    - Also record a sample after each member, so that a RUNNER_LATE can be attributed.
    - State in amendment2 that the check is a point sample.
    - 90 % over 1 s is otherwise a reasonable threshold (1 % resolution at USER_HZ 100).
- **m3-b. New test gaps** (mutants that survive the submitted tests; the code is correct by inspection or by probe):
  - **Idle check.** A33 (`ok = True`), A34 (`len` check removed), A42 (threshold 0.0) and A43 (busy ticks counted as
    idle) all survive. The real `idle_check` is never asked to reject: the tests stub `host_check`, and the positive
    control checks only the reader's keys and range. This is the AGENTS.md "non-vacuous" pairing in reverse: the
    negative case is untested for the real guard. My probe covers it.
  - **Runner, stops and bound.**
    - D06: `check_host()` not called in `execute()`.
    - D10: `host_idle_before` not recorded.
    - D01 and D02: `skip_max_s` not forwarded to `run`, or not passed to the gate. The fixture's `run_fake` ignores
      `gate_opts`, and the test approval's 0.05 equals the link default. P5 shows the flow works.
    - D14: the check before each member is redundant with the post-member check in the fixture.
  - **Runner, recording.**
    - D11: no warm-up in `execute()`.
    - D12 and D13: start-phase `pos` finiteness.
    - D15 and D16: `skip_max_s` and packages missing from the header. The fixture's `make_recorder` drops
      `extra_header`, so the header extras are never asserted.
  - Carried from r2: step M19–M21, M27 and M28; attempt A10, A19, A20, A23, A24, A26, A27 and A29.
  - *Fix:* use a non-default `skip_max_s` (for example 0.15) in the Execute test and assert it in the run kwargs and
    the gate. Add a failing `host_check` inside `execute()` (HOST_NOT_IDLE plus final.json). Test the real
    `idle_check` with a patched `core_idle` returning 0.89/0.90. Pass `extra_header` through the fixture recorder.
- **m3-c. Windows without final.json** (r2 m-d remainder; see MJ1-r3 d).
  - `open()` runs `self.dir.mkdir()` (attempt :288) before `self.faults()` (:289), so a failing fault read leaves an
    empty directory (P7).
  - Signals are not blocked around `append` (:344–348) or `_final` (:372–384).
  - *Fix:* read the faults before `mkdir`. Use `signal.pthread_sigmask(SIG_BLOCK, {SIGTERM, SIGHUP})` around the
    ledger write, the `prev_sha` update and the final write.
- **m3-d. `default=str` stringifies silently** (attempt :344).
  - `_finite_json` catches only Python `float` and its subclasses, which include np.float64. An `np.float32` NaN or
    inf, a numpy array or a tensor in a record is written as a string ("nan") with no record problem.
  - Fields that `record_problems` type-checks are still caught (type ≠ int/float). The current producers emit
    Python floats and ints (`min_dist_m` starts as `float("inf")` and becomes a float after one step), so the
    practical risk is low.
  - *Fix:* treat any `numbers.Real` or numpy scalar, and flag every value `default` has to convert as a record
    problem.
- **m3-e. Experiment-index tests fail before commit.** `public_cpu_r2fix.log` (18:58, in the external evidence set)
  shows 3 failures: two `test_experiment_index` failures, because amendment2 and `runtime_stack/` are untracked,
  and one link timing test (`test_same_burst_truth_is_awaited`). The validation's PUBLIC_CPU log must come from the
  staged or committed tree. The link failure belongs to that module's review.

### NOTE

- **n3-a.** An untracked file imported by a tracked module is still not refused (`git_state` uses
  `--untracked-files=no`; r2 m-c).
- **n3-b.** pytorch3d, yaml and gym are imported from site-packages and are not recorded, apart from the
  environment's numpy, torch and warp versions. amendment2 names only pymavlink and rl_games, so the text is
  accurate.
- **n3-c.** The torch global-RNG fact (r2 n-b: warm-up consumes the global torch RNG, and the outputs do not depend
  on it) is not stated in amendment2 §6.
- **n3-d.** `check_host` and `check_budget` raise AttemptError out of `execute()` after writing final.json, while
  the other stops return the status. The future CLI must treat both as a finished attempt.
- **n3-e.** P5's `header.extra.skip_max_s = null` comes from the fixture recorder, which ignores `extra_header`. The
  production path passes `extra_header=extra` (:448–451). The approval copy in the header holds 0.15.

## Answers to the specific questions

- **Idle-core check.** It is non-vacuous and bounded (1 s, no loop), and it cannot block forever. 90 % over 1 s is
  sensitive enough to catch one busy job, but a single sample can abort on transient load (m3-a). The test does not
  exercise its rejection (m3-b).
- **Explicit finish.** `append` no longer completes the attempt. Mutant A39, which puts back the auto-COMPLETE, is
  caught by `test_complete_attempt_feeds_the_pair_summary`. `execute()` calls `finish()` once after the loop, and
  mutant D07 is caught. Every abort path sets the status first, so `finish()` is then a no-op, and COMPLETE needs 42
  appended VALID records.
- **Source check after each member.** It runs before `append`, for the last member too (P1). A change between
  members is caught by both checks.
- **NaN handling.** Correct for Python floats (P3). Edge cases are in m3-d.
- **Approval `skip_max_s`.** Its type and range are validated (P6). It flows to the gate, the record and the
  header's approval copy (P5). The flow is untested (m3-b).
- **Signal handlers before open.** They are installed at :419–422, before `cfg.prepare`, `scene_specs`, `B.setup`
  and `open()`, and they are restored in `finally` (test and P2). A signal before `open()` leaves nothing. The
  remaining windows are in m3-c.
- **package_manifest.** It works on the real packages in 0.08 s. A namespace package without `__file__` would raise
  before `open()`, so nothing is left on disk. The rl_games version is null (MJ1-r3 c).

## What I ran

All runs were CPU-only: `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13`,
`timeout -k 10`, and `-B`.
- No PX4, Gazebo or GPU was used, and no forbidden port was opened (the fixtures bind random ports).
- The only processes I signalled were my own probe process (P2 and P2b) and my own busy-loop children.
- Nothing was written in a repository or in the PX4 tree; `git status` was unchanged at the end. Only read-only git
  commands were used.

The runs:
- **Snapshot.** `review_r3/snapshot/`: 9/9 hashes OK at 19:07:43. At 19:24:58, 7/9 still matched (see Moving
  target).
- **Unit tests** (worktree = submitted bytes, 19:08):
  - step: 13 OK (1.4 s);
  - attempt: 14 OK (0.4 s);
  - runner: 10 OK (22.8 s).

  Outputs: `review_r3/unit_*.out`.
- **Mutants** (`probes/mutants_r3.py`, run in `probes/tree_{att,run}`; each tree is the worktree symlinked with the
  snapshot files copied over it, built by `probes/build_tree.py`; the files were restored after each mutant, and
  their hashes were re-checked):
  - attempt, 20 mutants: A35–A38 and A40–A41 caught; the r2 survivors and A33, A34, A42 and A43 survive.
  - runner, 16 mutants: D03, D05, D07 and D09 caught; D01, D02, D06 and D10–D16 survive.
  - A39 was re-run with correct syntax (`probes/mutant_a39.py`): caught.
- **`probes/execute_paths_r3.py`** (`execute_paths_r3.out`):
  - P1/P1b: last-member and member-40 source change.
  - P2/P2b: SIGTERM during the cleanup and during the body of a real OwnedSession.
  - P3: NaN.
  - P5: `skip_max_s` 0.15 to the gate.
  - P6: approval values.
  - P7: open with an unreadable kernel log.
- **Other probes:**
  - `probes/binding_count_r3.py`: 62 / 40 / 7.
  - `probes/package_manifest_probe.py`.
  - `probes/idle_probe.py`: busy, duty-cycle and post-torch samples; contaminated by another session's job, as
    reported.
  - `probes/idle_series.py`: 15 samples.
- **Read-only checks:**
  - `tolerance_divergence.out/.py`, `dev07_lateness.out/.py` and `make_validation.py` in
    `~/motar_tmp/dev08_runtime_stack_20261105/`.
  - The `dev07_70/ep02_steps.npy` columns 1, 21 and 24, and the bson lines in the dev07 PX4 logs.
  - `sitl_s3b_closed_loop.py:294–317`.
  - The PX4 build `init.d-posix` listing and the rcS sourcing.
  - The amendment1 caps and stop rules.
  - The link review r2 phase statement.
  - The owned-session signal and exit code (dependency, current worktree bytes).
  - The `docs/EXPERIMENTS.md` diff against HEAD: the sitl-transfer entry adds the runtime stack and amendment2 to
    the Status, Lineage and Members fields; the wording is consistent with the records.
  - The process list and affinities, and the CPU topology (6/7 and 12/13 are SMT siblings of two P-cores).
