# dev08 runtime stack (step / attempt / runner / amendment2): independent review r2

Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context. Read-only on every repository
and on the PX4 tree. Date 2026-11-05, 18:31–19:00 KST.
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `9051cea`.

## Verdict: **FAIL**

There is no BLOCKER.

**What the round-1 repairs achieved.** Most of them hold, and I checked them on the submitted bytes:
- The identity test passes: 13/13 in the step test, 11/11 in the attempt test and 10/10 in the driver test.
- With the real bridge and Policy F on the CPU, native prefix identity holds on the submitted step, link and test
  files: 599 and 63 commands are identical, and so are the observations, states and sample times. I re-ran this
  myself.
- The seven step mutants that survived round 1 are now killed.
- The bridge Episode is built before alignment, and `warm_up` changes nothing that Policy F computes.
- Untracked or modified SOURCES are refused.

**Why the verdict is FAIL.** Two MAJOR items must be fixed before commit:
1. amendment2 is a `docs/plans/prereg_*.md` path, so it is immutable once committed. It still states several
   things that the code, a probe or a record contradicts (MJ1).
2. The run is wall-clock-fragile: a runner delay of about 0.15 s aborts it. Nothing checks or records that the host
   and the four runner cores are idle. Yet AGENTS.md allows concurrent CPU work on exactly those cores (6, 7, 12,
   13) during development SITL runs. During this review one such job raised the median step time from about 6 ms
   to about 150 ms (MJ2).

The MINOR items also include two control defects in `execute()`. A source change during the last member still ends
COMPLETE (P1). A SIGTERM or SIGHUP that lands during the owned session's cleanup is swallowed, and the unit then runs
all remaining members to COMPLETE (P2).

**Moving target.** I copied the submitted files to `review_r2/snapshot/`; all 10 hashes matched at 18:31:19. During
the review, five worktree files changed. I did **not** review the new bytes:

| File | Submitted | Worktree now | mtime |
| --- | --- | --- | --- |
| tools/sitl_owned_session.py | b999a6dd | c26f0eac | 18:46:59 |
| tools/sitl_dev08_link.py | bf3ab740 | aa2551a7 | 18:50:25 |
| amendment2 | b8f7ef62 | 9ecd89f0 | 18:50:50 |
| runtime_stack/README.md | 60218874 | e08ce40a | 18:50:50 |
| tools/sitl_s3b_dev08_prefix.py | 911a2229 | f452d36d | 18:52:11 |

- `docs/EXPERIMENTS.md` (not submitted) was modified at 18:31:26.
- Every finding below refers to the snapshot bytes.
- My probes that loaded worktree files finished before 18:46:59 (`execute_paths.py`, 18:45:58), or they loaded
  snapshot copies. One exception: `real_bridge_identity_r2.json` used link aa2551a7. I discarded it and re-ran on
  the snapshot (`real_bridge_identity_r2_snapshot.json`).

## Repair table (round-1 findings)

| r1 | Status | Evidence (snapshot bytes) |
| --- | --- | --- |
| B1 identity test | REPAIRED | `test_sitl_dev08_step.py` 13/13 OK. The harness feeds a per-message receipt clock. Real bridge + Policy F re-run by me on the submitted step/link/test (`probes/real_bridge_identity_r2_snapshot.json`): 70/env0 599 commands, 205/env8 63 commands, all identical; observations, states and samples identical. Integration 10/10 OK. Real `fresh_expectations` (installed airframe + parameters.json) covers all 27 names for both arms, 64/66 round trips (`probes/fresh_check.out`). |
| B2a truth clock | REPAIRED | amendment2 §7 and live assumption 3 now use receipt time. PX4 `HIL_STATE_QUATERNION.hpp` never assigns `time_usec` (grep: no match). |
| B2b identity record | REPAIRED | `real_bridge_identity.json` exists, and my run on the submitted bytes reproduces it. That record carries no source hashes, so validation.json must bind it (or bind my run). |
| B2c truth pairing wording | REPAIRED | The harness now passes. The "same burst" wording remains imprecise (MJ1-g). |
| B2d parameter count | REPAIRED | 27 parameters, airframe-or-firmware expectation. Checked: none of the 27 is set by rcS, rc.mc_defaults or px4-rc.* (rc.vtol_defaults sets MC_ROLLRATE_MAX but is not used for this airframe). |
| B2e binding | PARTIAL | 40 recursive plugins is correct, and the other bound files are listed. "61 files" is wrong: the record will hold 62. "Everything the launch runs or reads at boot" overclaims (MJ1-a, b). |
| B2f float comparison | REPAIRED | `TelemetryGate.checkpoint` uses `not s["t"] < t_k`. |
| M1 start order / warm-up | REPAIRED | `sitl_s3b_dev08_prefix.py:219` builds PrefixEpisode before `gate.align()` (:222). `warm_up` (:252–268) runs once before `rec.open()`. The Timing test has a negative control (0.1 s constructor + 0.5 s first call, no warm-up → RUNNER_LATE) and a positive one (warm → VALID). Warm-up side effects: see "Episode construction, warm-up and timing". |
| M2 source guard | PARTIAL | `source_problems` refuses untracked or HEAD-different SOURCES (mutants A12–A14 caught). The driver and both amendments are listed; the cplanner `replay.py` is bound by hash, HEAD and state (currently 2837b18, clean, 922da95d…). **Gaps:** 8 executed production modules are not in SOURCES, and external code is not bound (m-c). |
| M3 test blind spots | REPAIRED | Moving target, vz, fast crossing, capture+crash, out of bounds and the ceiling were added. All 7 r1 survivors are caught by the submitted test (`probes/step_mutants_r2.out`: M01–M03, M10–M13). New, smaller survivors in m-h. |
| m1 self-declared VALID | REPAIRED | `record_problems` runs for every VALID record, smoke included (A15–A18, A28 caught). A19/A20/A29 are untested (m-h). |
| m2 environment first | REPAIRED | `environment_problems` runs before the bridge import (:389–391). The `aerial_gym` resolution check (:404–406) comes after `B.setup` but before any member, and the bridge loads production files by explicit path. Probe: 0 files from other checkouts executed (`warm_probe_warm.json`). |
| m3 no final.json / budgets / -k | PARTIAL | `except BaseException` → RUNNER_EXCEPTION, the SIGTERM handler and `-k 60` are in place. Remaining (m-d, m-f): `real_deps()` is outside the try (:416); there is a window before the handler is installed; `faults()` can raise inside `append`/`_final`; budgets are checked only between members, with no remaining-time refusal. |
| m4 change during the last member / binding | PARTIAL | The binding comparison works (ExecuteStops test). The post-member source check (:444) is skipped for the last member by the `break` at :442. Probe P1: a change during member 41 → **COMPLETE**. For other members the check only duplicates the next pre-member check. So the scenario r1 named is still open (m-a). |
| m5 evidence on failure | PARTIAL | `out=rec` is filled progressively; rows carry seqs, `pos_lateness_s`, `att_skew_s` and `truth_recv_skew_s`. If a BaseException escapes `run_episode`, `checkpoints.jsonl` is not written. A non-JSON value drops the member from the ledger (P3, m-e). Raw telemetry is not persisted (n-c). |
| m6 header | PARTIAL | Inventory summary, platform, CPUs online/offline, OMP/MKL env, journalctl rc (A25 caught), chain root (A21/A22 caught) and the faults_end check are all present. Thread counts are recorded but not enforced. Host idleness is neither recorded nor checked (MJ2). |
| m7 rx thread / boot telemetry | REPAIRED | `rx_thread_alive_after_join` is recorded, and RX_THREAD_STUCK makes the member ERROR. The gate's start cursor is 1 s before alignment. |
| m8 live assumptions | PARTIAL | (i) truth timing and (ii) runner timing were added. Still missing: lockstep TCP 4560; heartbeat/mode from (1,1), system-245 setpoints accepted, SET_MESSAGE_INTERVAL honoured for id 115; per-episode wall time ≤ 240 s; writes outside the episode directory (`~/.gazebo`, model database). |
| m9 README | PARTIAL | Tolerance numbers and Limits are corrected. validation.json and Reviews are pending, as expected at commit (RECORDED). New factual error (m-g). |
| m10 index | REPAIRED (not hash-bound) | `docs/EXPERIMENTS.md` (outside the submitted set, 18:31:26) lists amendment2 and runtime_stack/. |
| m11 schedule-name check | REPAIRED | Mutant A08 caught. |
| m12 ekf_minus_truth_xy_m | REPAIRED | :213; asserted in the Timing test. |
| n1 position selection | REPAIRED | amendment2, Differences bullet 3. |
| n2 one bridge setup | REPAIRED | Header `extra.bridge_setup` and amendment2. |
| n3 fixture not hermetic | NOT_REPAIRED (NOTE) | The fixture still uses the default `px4_lock=/tmp/px4_lock-0` and `exclusive_udp=(14540,)`. |
| n4 approval provenance | RECORDED_AS_LIMIT | Deferred to the approving CLI commit (amendment2, Proposed execution). See also m-i. |
| n5 shared lock | REPAIRED | — |
| n6 truth origin | unchanged NOTE | Covered only by the fake-PX4 integration test. |

## New findings

### MAJOR

**MJ1. amendment2 (immutable once committed) still contains statements that the code, a probe or a record
contradicts.** Correct all of them before the first commit.
- (a) §1 l.41 "61 files on this host". `runtime_binding()` binds 61 files here (`probes/binding_count.out`), and
  `real_deps().binding` adds `empty_world_template` (`sitl_s3b_dev08_prefix.py:345`). The record will therefore
  hold **62**, and the same list says it includes the template.
- (b) §1 "binds … everything the launch runs or reads at boot". Installed rcS also sources files that are not bound:
  `px4-alias.sh`, `etc/init.d/rc.vehicle_setup`, `rc.mc_apps`, `rc.logging`, `rc.sensors`. Say "the following files".
- (c) §8 l.96–97 "Every source must be tracked and equal to HEAD. That covers … the production modules the bridge
  loads". False. A trace of what actually executes (`probes/warm_probe_warm.json`, `executed_not_in_SOURCES`) lists
  eight modules missing from SOURCES:
  - `aerial_gym/rl_training/rl_games/navrl_players.py`, `navrl_transformer_network.py`, `navrl_action_models.py`,
    `ppo_update_safety.py`, `navrl_mode_probe.py`;
  - `aerial_gym/task/navrl_task/navrl_corridor.py`, `navrl_search_state.py`;
  - `aerial_gym/utils/math.py`.

  See m-c.
- (d) §8 l.101 "The member is not VALID if … the source changes before or after it". A member after which the
  source changes is appended VALID, and the attempt then aborts. After the **last** member nothing is checked
  (P1: COMPLETE).
- (e) §2 l.54 "SIGTERM/SIGHUP to the runner run the same cleanup", together with the Stops list ("All keep the
  attempt"). A signal that lands during the session's cleanup is swallowed, and the unit continues (P2, m-b).
  Proposed execution l.161 "cleanup, which takes up to 25 s" is not a bound from the code: SIGINT + 20 s, SIGKILL
  + ≤5 s, final scan + ≤5 s, and before that `rx.join(5)`. That is ≤35 s, still inside `-k 60`.
- (f) Differences l.120 "(and, for other start samples, at k = 1)". The record says 117 of 500 start samples first
  diverge at k = 1, and the range is 1–384 (`tolerance_divergence.out`). Write it as the README does.
- (g) Differences l.107 "truth: the latest received in the same burst". The code takes the latest received in
  [−0.03, +0.005] s of the position sample, which covers any stream phase. "Same burst" is the model behind the
  +5 ms edge only.
- (h) §7 lists the conditions but omits BAD_QUATERNION (truth quaternion norm within 1e-3).
- (i) Decision section l.141–149. It does not state the recorded cause of the dev07 late sample. The step log's
  compute column (index 21; `probes/dev07_columns.out`) shows a **218.4 ms runner compute** at
  `dev07_70/ep02` k=27, then t[28] = t[29] = 38.300 s, so the delay was on the runner side, not a PX4 gap. Over all
  1318 dev07 steps: median 6.2 ms, 3 steps over 100 ms, 1 over 150 ms (`probes/dev07_compute_tail.out`). This
  matters for the decision:
  - the late check (RUNNER_LATE) and the skip check (CLOCK_SKIP) guard different causes and could take separate
    bounds;
  - "0.15 s would have admitted that episode" is true only because the cause was on the runner side (with a PX4 gap
    the next checkpoint would reuse the truth sample and fail STALE_TRUTH);
  - the main lever may be runner isolation (MJ2), not only the bound.

  With these added, the decision is stated fairly. It presents both options, says what a wider bound costs (stale
  state), says both arms share the bound, and fixes it before the smoke.
- (j) amendment1 fixes "timeout kill grace 20s" (l.151). Amendment2 keeps the session's 20 s SIGINT→SIGKILL grace
  and adds an outer `-k 60`. State that the 20 s refers to the session grace, so it does not read as a changed
  bound.
- Checked and correct: estimand, cohort, order (the 42-member schedule is identical to amendment1, smoke order
  70→205 added), stop rules, analysis; ports/lock/process names; spawn args; 240 s / 12 GiB / 0.25 s; 40 plugins;
  27 parameters; 18–75 checks; 36 s of waits; 59.9 s; 1 − 0.95^42 ≈ 0.88; 0.136 s at k=28, others ≤ 0.04 s.

**MJ2. The idle-host and exclusive-core precondition is neither checked nor recorded, and project rules let other
jobs share exactly the runner's cores.**
- The runner's validity depends on wall-clock time. RUNNER_LATE fires when step time + receipt slack exceeds about
  0.15 s, and any abort ends the unit and uses up the approval.
- AGENTS.md (D-2026-11-05-1) allows CPU jobs on cores 6, 7, 12, 13 with `nice -n 10`, and lets "development runs"
  (dev08 is a descriptive development record) run alongside. amendment2 asks for an "idle host (… T-LOAD's
  wall-clock rule)", which under that rule does not exclude such jobs. `execute()` and `AttemptRecorder.open()`
  check CUDA, affinity and nice only. They record no load, and nothing re-checks between members.
- **Measured during this review.** Another user process, PID 2184361 `tools/run_tests.py` (affinity 6, 7, 12, 13,
  96 % CPU, started about 18:52:50; not started by me), ran on the runner's cores. My per-phase probe at that time
  (`probes/spike_probe.out`), 150 steps on a warmed 205-bar episode at 10 Hz:
  - median 150 ms, 77/150 steps over 150 ms;
  - policy calls up to 614 ms, observe calls about 100 ms.
- Before that job started (`probes/timing_probe.out` rep 0; `warm_probe_*.out`), steps were about 6 ms. Isolated
  first steps after warm-up ranged up to 106 ms, and one later step took 187 ms; the host load average was about
  1.5 then.
- In this state the smoke would very likely fail RUNNER_LATE, which is the same class of risk as r1 M1.
- *Fix:*
  - State in amendment2 that this unit needs the four runner cores exclusive and the host idle, whatever the T-LOAD
    classification.
  - In `open()` and before each member, refuse when any non-owned process used CPU on the runner cores within the
    last few seconds (sample `/proc/<pid>/stat` field 39 and utime), or when loadavg is above a fixed threshold.
  - Record the check in the header and in each record.
  - Decide the bound with the runner-side tail in view (MJ1-i).

### MINOR

- **m-a. A source change during the last member still ends COMPLETE.** `sitl_s3b_dev08_prefix.py:441–446`: `append()`
  finishes the attempt inside the call for member 41, `break` at :442, so the check at :444 never runs (probe P1:
  `final.status COMPLETE, recorded 42`; control P1b, change at member 40 → ABORTED SOURCE_CHANGED, recorded 41). For
  members 0–40 the post-member check only duplicates the next pre-member check. *Fix:* compare `git_state` after
  `run()` and **before** `append()`; on a change mark the record not VALID (e.g. `SOURCE_CHANGED`). This also makes
  MJ1-d true.
- **m-b. A SIGTERM/SIGHUP that lands during the owned session's cleanup is swallowed, and the unit continues.**
  - `OwnedSession._on_signal` (`sitl_owned_session.py:529–533`) only records a signal while `_closing`.
  - `__exit__` (:725–796) then restores `execute()`'s handler and returns normally.
  - `run_episode` returns VALID and `execute()` goes on to the next member.
  - Probe P2 (real OwnedSession, fake processes, fake PX4): SIGTERM at t=2.497 s, `during_cleanup: true` in
    session.json. Member 0 VALID; **41 further members ran; final COMPLETE**.
  - Under `timeout … 5600` this happens only past the 5400 s budget, so the next `check_budget` aborts. An operator's
    stop is ignored, though, and a follow-up SIGKILL leaves no final.json and orphaned simulator processes (they run
    in their own sessions).
  - Related: SIGTERM inside the session raises SessionTerminated, which `run_episode` records as ERROR, so the
    abort reason reads MEMBER_NOT_VALID/SMOKE_FAILED, not a termination.
  - *Fix:*
    - Have the session re-raise SessionTerminated at the end of `__exit__` when a signal arrived during cleanup.
    - Or have `run_episode` mark the record not VALID when the receipt's `signals` list is non-empty.
    - Use a flag-setting handler in `execute()`, checked between members, and block signals around
      `append`/`_final`.
- **m-c. The source list is incomplete, and external code is unbound.**
  - The eight modules in MJ1-c are tracked and equal to HEAD today. Because `git_state.dirty` covers every tracked
    file, edits to them would be refused, but their hashes are not in `attempt.json.sources`.
  - A file that is untracked but imported by an unchanged tracked file would not be refused.
  - pymavlink comes from `PYLINK` → symlink to `/home/fair/miniconda3/envs/px4/.../site-packages/pymavlink`.
    `rl_games` and `pytorch3d` are imported too. None is version- or hash-recorded; `platform_info` records numpy,
    torch and warp only.
  - *Fix:* derive the executed list from `sys.modules` after `B.setup()` + `B.Policy()` and check each file, or
    extend SOURCES. Record the pymavlink path, version and hash and the rl_games version.
- **m-d. Remaining paths that end without a final.json.**
  - `deps = deps or real_deps()` (:416) runs after `rec.open()` and outside the try.
  - A SIGTERM between `rec.open()` and handler installation (:422–425) kills the process.
  - `kernel_faults()` raising mid-run (journal unreadable) inside `append` (attempt :289) or `_final` (:314): `_final`
    has already set `status`, so no final.json is ever written.
  - A signal inside `append` between the ledger write and the `next`/`prev_sha` update gives a final.json that does
    not close the ledger (detected by `read_ledger`).
  - *Fix:* move :416 into the try. Install the handler before `open()`. Make `_final` tolerate a failing fault read
    (status VOID, `kernel_faults_end=null`). Block signals around `append`/`_final`.
- **m-e. A non-finite or non-JSON value loses the member's ledger line.** `append` serialises with
  `allow_nan=False`, so a NaN in any field raises after the member ran.
  - Probe P3: a NaN `real_time_factor` in member 1 → `ValueError`, RUNNER_EXCEPTION, ledger holds only member 0.
  - This can happen in practice: the start phase does not finite-check `pos` (only `pn` and truth), so a NaN position
    yields a NaN `start_check` and HARNESS_INVALID, and that record cannot be written. The same `pos` feeds the
    `hold_at` setpoints sent to PX4.
  - *Fix:* sanitise non-finite values to a string before `dumps`, and finite-check `pos` in the start phase.
- **m-f. Budgets are checked only between members.** A member started at 5399 s may run until about 5675 s
  (240 s + cleanup). There is still no "do not start without one deadline plus cleanup left" rule, and amendment2
  does not state the overshoot.
- **m-g. README factual error.** README l.26–27: "All 23 dev07 PX4 logs show … `(INT32:13, FLOAT:16)`". 22 do;
  `dev07_smoke_70/ep00_sitl.log` l.29 shows `(INT32:13, FLOAT:6)`, the first run, before the S2 map was stored. The
  conclusion, shared parameters.bson state, stands. No external record supports the count; add one (rule: numbers
  travel with their source).
- **m-h. Test gaps (mutants that survive the submitted tests).**
  - Step (`probes/step_mutants_r2.out`, 33 mutants; M32 was an equivalent mutant of my own making):
    - M19 altitude floor removed (no ground-crash scene);
    - M20 crash-reason precedence;
    - M21 out-of-bounds checked on x only;
    - M27 min_dist from point distance;
    - M28 prev_rel never updated (sweep from the first relative position).
  - Attempt (`probes/attempt_mutants_r2.out`, 32 mutants, 24 caught):
    - A10 tracked-dirty non-source file ignored;
    - A19/A20 time checks;
    - A23/A24 `main_records` defences;
    - A26 external-source problems ignored;
    - A27 Git-tree check: the test refuses for other reasons, so it is not specific;
    - A29 `recorded_frames` range.
  - Driver: the post-member source check cannot be distinguished from the pre-member one (m-a). The SIGTERM test
    covers only a signal outside a session.
  - The code is correct by inspection on all of these, and identical to the historical code where applicable.
- **m-i. The bound decision is not bound to the approval.** amendment2 says the skip/late bound is "recorded in the
  approval". `APPROVAL_KEYS` has no such field, and `execute()` does not compare the approved value with
  `LK.SKIP_MAX_S` (or with `FRESH_MAX_S`, `TRUTH_AGE_S` and `BURST_S`). *Fix:* add the field, refuse on a mismatch,
  and write it into the header.
- **m-j. Live assumptions still missing** (r1 m8): see the repair table.

### NOTE

- **n-a. The historical report field `compute_ms_*` is mislabelled.** `tools/sitl_s3b_closed_loop.py` computes
  `compute_ms_median/max` from `r[-3]` of a 27-column row, which is `p_true[1]` (metres). The compute time is column
  21. Example: `dev07_70` report `compute_ms_max` 28.915 equals the episode's p_true-y maximum. The true column shows
  48–60 ms at k=0 of each process's first episode and the 218.4 ms spike (MJ1-i).
  - This corrects my own r1 remark ("dev07 compute_ms_max ≤ 38 ms").
  - The historical file is frozen. No submitted file cites the field; any future record must use column 21.
- **n-b. Warm-up side effects.** Checked for warm_up (`probes/warm_probe_{cold,warm}.json`):
  - The model's parameters and buffers are unchanged (hash), and the model is in eval mode.
  - numpy and python RNG are untouched.
  - The global torch RNG *is* consumed by warm_up and by every observe/act. Outputs did not depend on it: cold and
    warm processes, with different torch RNG states, gave bit-identical observation/action/governor digests over
    40 steps on 70/env0 and 205/env8 (`f7c96fa15379987e`, `a45ec92fe351d885`).
  - In `navrl_reflection_offline_audit.deterministic_actions`, `rnn_states` is None, the forward runs under
    `no_grad`, `build_player` calls `model.eval()` and checks `training` is False, and RunningMeanStd does not update
    in eval.
  - So "Policy F's forward has no state" holds. Record the global-RNG fact.
- **n-c. Checkpoint rows bind sample `seq` numbers of an inbox that is never persisted.** The raw PX4 telemetry of a
  member is not kept.
- **n-d. A VALID record is not required to carry certain fields.** `record_problems` does not require `parameters`,
  `runtime_binding`, `session_receipt` or `rx_thread_alive_after_join=False` (defence in depth).
- **n-e. `read_ledger` raises FileNotFoundError or IndexError rather than AttemptError.** This happens for a missing
  final.json or more than 42 lines.
- **n-f. `aerial_gym` resolution check.** The editable-install finder (`__editable___aerial_gym_2_0_0_finder.py`) is
  imported, but the bridge's stub packages shadow it. The check at :404–406 would catch a real package imported
  earlier.

## Answers to the specific questions

- **Is the new order plus warm_up sufficient?** For the first-call cost, yes.
  - The Episode (16–100 ms here) is built before alignment.
  - After warm_up, the first step on a fresh Episode took 6–26 ms in clean conditions, one outlier 106 ms. That is
    within the about 0.15 s RUNNER_LATE margin.
  - It is not sufficient against host contention (MJ2) or runner spikes like dev07's 218 ms.
- **Would an untracked or modified runtime file be refused?**
  - Yes for every SOURCES entry: the untracked, differs-from-HEAD and tracked-dirty cases are tested, and mutants
    A12–A14 are caught.
  - For the eight unlisted executed modules: yes if modified (tree-wide dirty check); no if untracked (m-c).
  - The list is not complete for what executes.
- **Interaction of the two signal handlers:**
  - inside a session: SessionTerminated → ERROR → abort (fail-closed, reason misattributed);
  - during cleanup: swallowed (m-b);
  - between members: RunnerTerminated → RUNNER_EXCEPTION + final.json (tested);
  - before handler installation: no final.json (m-d).
- **Can a failed member carry a VALID outcome that the recorder accepts?** No.
  - `operational_status="VALID"` is set only at the end of `_fly`. Any later exception (mode command,
    `session.sleep`, RX_THREAD_STUCK, CleanupError/SessionAborted from `__exit__`) overwrites it with ERROR or
    HARNESS_INVALID.
  - A BaseException is never appended.
  - BINDING_CHANGED replaces VALID.
  - The recorder accepts only VALID records with no `record_problems`.
- **Can smoke enter main?** No. Phase is checked against the schedule; `main_records` filters `phase == "main"`
  and requires COMPLETE, unchanged faults and no record problems.
- **Can COMPLETE hide an invalid or missing member?** No, but COMPLETE can follow a source change during the last
  member (m-a) or a swallowed operator SIGTERM (m-b).
- **Binding comparison.** File sha256 only, world excluded, against the first VALID member, VALID records only
  (tested). The env map is not compared (GAZEBO_LOG_PATH differs per episode). Fixed keys such as PX4_SIM_MODEL and
  GAZEBO_PLUGIN_PATH could be compared.
- **Historical code unchanged.** `tools/sitl_s3b_closed_loop.py` d170b027 and `tools/sitl_bridge.py` f0031f11 equal
  PINS; git shows no modification.

## What I ran

All runs were CPU-only: `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13`,
`timeout`, and `-B` / `PYTHONDONTWRITEBYTECODE=1`. No PX4, Gazebo or GPU was used, and no forbidden port was opened
(fixtures bind random ports). The only process I signalled was my own probe process (P2). Nothing was written in a
repository or the PX4 tree. The `tools/__pycache__` entries at 18:48–18:49, including a cpython-313 one, are not
mine. Only read-only git commands were used.

- **Snapshot.** `review_r2/snapshot/`, 10/10 hashes OK at 18:31:19 and again at the end.
- **Unit tests (worktree = submitted bytes, 18:31–18:33).**
  - step 13 OK;
  - attempt 11 OK;
  - driver 10 OK in 22.7 s (`test_prefix_run1.out`).
- **Mutants.**
  - `probes/step_mutants_r2.py`: 33 step mutants run against the submitted step test, with the snapshot link and the
    pinned historical files.
  - `probes/attempt_mutants_r2.py`: 32 attempt mutants.
- **`probes/execute_paths.py` (fakes, 18:45).**
  - P1: source change during the last member, and during member 40 as a control.
  - P2: SIGTERM during OwnedSession cleanup.
  - P3: NaN in a VALID record.
- **`probes/warm_probe.py`.** Cold and warm processes; RNG, model-state and output digests; executed-module trace
  against SOURCES.
- **Timing probes.**
  - `probes/timing_probe.py`: Episode construction and per-step times on fresh episodes after warm-up.
  - `probes/spike_probe.py`: per-phase times; contaminated by PID 2184361, as reported.
- **`probes/real_bridge_identity_copy.py`.** A byte-identical copy of the author's script (6ca63e10), run on
  `probes/identity_tree/`: the snapshot step, link and test, with pinned files symlinked to the worktree.
- **Other probes.**
  - `probes/binding_count.py`: 61 files bound, 40 plugins, 7 model resources; dummy world in `probes/binding_ep/`.
  - `probes/fresh_check.py`: real airframe and parameters.json expectations for both arms.
  - `probes/dev07_columns.py`, `dev07_compute_tail.out`: dev07 step-log compute column.
- **Read-only checks.**
  - PX4 tree: `HIL_STATE_QUATERNION.hpp`, rcS sourcing, rc.* parameter sets, airframe 10090, plugin count.
  - dev07 PX4 logs: bson import counts.
  - External evidence in `~/motar_tmp/dev08_runtime_stack_20261105/`: identity, eps, lateness, tolerance,
    scene_specs, layout, step_mutants, public_cpu logs and make_validation.py.
  - amendment1; `docs/EXPERIMENTS.md` diff; cplanner `replay.py` state; process list and affinities.
