# dev08 runtime stack (step / attempt / runner / amendment2) — independent review r1

Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only on every repository
and the PX4 tree. Date 2026-11-05, 17:58–18:21 KST.
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `9051cea`.

## Verdict: **FAIL**

The policy/command step (`tools/sitl_dev08_step.py`) is a faithful copy of the historical 10 Hz loop body. With
snapshots chosen the way the historical loop chose them, my link-independent probe found its states, sample
times, commands, outcomes and counts identical in all four scenarios, including 600 checkpoints. The attempt
recorder's order, chain and stop logic are sound. The unit still fails, for four reasons:

1. At the reviewed bytes the repository's identity test failed. At review start 1/7 failed, from the
   sample-comparison tolerance. After the link rewrite 4/7 failed, from receipt-time truth selection against a
   harness that fed all messages at once.
2. amendment2 is a `docs/plans/prereg_*.md` path, so it becomes immutable once committed. It states facts that
   the PX4 source contradicts, and it claims a real-bridge identity result for which no record exists.
3. The driver fixes the start time before it builds the bridge Episode (40–100 ms measured). Checkpoint 0 should
   therefore fail RUNNER_LATE on most scenes, so the smoke would very likely abort.
4. The execution guard treats untracked runtime sources as a clean tree, and all five runtime modules are
   untracked right now.

**Moving target.** The author edited the dependencies and two in-scope files while this review ran. All findings
refer to the snapshot `snapshot_1803/` (SHA256SUMS there). At 18:16–18:20 I saw later bytes that address parts
of B1. They are listed under "Post-snapshot state" and are **not reviewed**; a round r2 must cover them.

| File | Reviewed SHA-256 (prefix) | Later bytes seen (not reviewed) |
| --- | --- | --- |
| tools/sitl_dev08_step.py | 9b1c9135 | unchanged |
| tests/test_sitl_dev08_step.py | 8f3abcc7 | 29499175 (18:17:54) |
| tools/sitl_dev08_attempt.py | a5db6b4e | unchanged |
| tests/test_sitl_dev08_attempt.py | d6e19a39 | unchanged |
| tools/sitl_s3b_dev08_prefix.py | 3904b8e5 | aff090f3 (≈18:16) |
| tests/test_sitl_s3b_dev08_prefix.py | 8dfe6d30 | 4652cc13 (≈18:16) |
| amendment2 | 28048219 | unchanged |
| runtime_stack/README.md | 5cb2b139 | unchanged |
| dep tools/sitl_dev08_link.py | first read: unhashed (21 RECORD_PARAMS, `t >= t_k - 1e-9`); fcead778 (17:59) | cda7e758, bf3ab740 |
| dep tools/sitl_owned_session.py | bcfda399 (692 lines), b999a6dd (962 lines) | — |

## Findings

### BLOCKER

**B1. The native-prefix identity test did not pass at the reviewed bytes.**
`tests/test_sitl_dev08_step.py` (8f3abcc7) `run_new` lines 235–265; link `TelemetryGate`.
- *Link version at review start* (`pos t >= t_k - 1e-9`): 6/7 pass. `test_full_600_frame_censor_prefix_identity`
  fails, first differing command 164. I reproduced it without the link module. A snapshot selector using the
  historical comparison `not (n*20e-3 < start + k*0.1)` is identical through k=599. The same selector with the
  1e-9 tolerance diverges at k=164 (command, sample and state). The failure is the sample-comparison issue.
- *Link fcead778 / bf3ab740* (truth chosen by receipt time, `_truth_for`: latest received within
  [pos.recv−0.03, pos.recv+0.005] s): 3/7 pass. Capture, crash, censor and censor600 all fail at k=5, the first
  checkpoint after the scripted vehicle starts moving. The cause is that `feed_to(n + 1)` puts the whole burst,
  including the look-ahead sample n+1, into the Inbox with near-identical `recv`. The gate then takes truth n+1;
  the historical loop used truth n.
- *Integration test at 3904b8e5/8dfe6d30 with link fcead778*: 2/8 fail. `fresh_boot()` asserts 21 vs 27
  RECORD_PARAMS, and `real_deps()` passed `airframe_defaults` only. Against the current link,
  `apply_and_verify` raises `ValueError: no fresh-boot expectation` for 13 parameters, so every live member
  would be ERROR. I verified this with the installed 10090 airframe.
- Amendment1 requires "identical CPU observation/action/setpoint prefixes for native" before live integration.
  At no reviewed state did the repository's own test show that in full.
- *Fix:* freeze the link API. Give the harness a per-message receipt clock that models PX4's send burst (the
  later bytes 29499175 do this). Keep a control asserting that the selected truth is the same-burst sample. Then
  rerun the real-bridge identity script and record it (see B2b).

**B2. amendment2 states things that the PX4 source contradicts or that have no record.** It must be corrected
before commit, because a committed preregistration is never edited.
- (a) §"What the runtime does" item 7 says "HIL_STATE_QUATERNION samples within 0.05 s of it (truth on PX4's
  clock)", and live assumption 3 says "HIL_STATE_QUATERNION.time_usec is on PX4's clock (alignment refuses with
  TRUTH_CLOCK_MISMATCH otherwise)". PX4 v1.14.3
  `src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp` `send()` never assigns `time_usec`, so it is 0.
  Under the gate the document describes, the second truth message would have raised CLOCK_NOT_INCREASING on
  every live run. The current link uses receipt time. Assumption 3 must be replaced by the receipt/burst
  assumption (see Q6).
- (b) §Differences, last bullet, claims real-bridge identity "for every nonterminal checkpoint (one
  600-checkpoint censored scene, one crash scene)". The only record,
  `~/motar_tmp/dev08_runtime_stack_20261105/real_bridge_identity_prefix_gate_eps.json`, shows 70/env0 (600
  checkpoints) `commands_identical_prefix: false`, `first_command_difference: 164`. Only 205/env8 (crash, 63
  commands) is identical. No post-fix record exists. Rule: numbers travel with their source.
- (c) §Differences, bullet 2: "ATTITUDE/truth: the sample nearest the position sample … the CPU harness shows
  identity there". Truth is now "latest received in a window", and at the reviewed bytes the harness failed.
- (d) Item 5: "the 21 recorded REAL32 parameters" (now 27). "Require those the airframe sets … to equal it" is
  now every recorded parameter against the airframe-or-firmware expectation from parameters.json.
- (e) Item 1: "all 39 Gazebo plugins". The current `runtime_binding` uses `rglob("*.so")`, which finds 40
  (`OpticalFlow/libOpticalFlow.so`). 39 was correct only for the earlier top-level glob. It now also binds
  parameters.json, px4-rc.*, rc.mc_defaults and model resources, which amendment2 does not mention.
- (f) Item 7: "the historical loop's own float comparison" was false for the link it was written against
  (`t >= t_k - 1e-9`). It is true for fcead778 and later.

### MAJOR

**M1. The start is fixed before the bridge Episode exists, so checkpoints 0 and 1 are predictably RUNNER_LATE.**
`tools/sitl_s3b_dev08_prefix.py` 3904b8e5 lines 208–209 (aff090f3 lines 216–217); `sitl_dev08_step.py:94`.
- The historical runner builds `ep = B.Episode(...)` before `sim_start = L.pos[0]` (`sitl_s3b_closed_loop.py`
  lines 258–262). The driver calls `gate.align()` (start = newest position) and only then
  `ST.PrefixEpisode(...)`, which constructs `B.Episode`.
- Measured with the real bridge on CPU (`probes/episode_ctor_timing.out`): construction took 60–99 ms at
  70 bars and about 40 ms at 205 bars.
- At checkpoint 0 the newest 50 Hz position is then 0.06–0.10 s past t_0, above SKIP_MAX_S = 0.05, so
  RUNNER_LATE fires on most scenes and on every 70-bar scene. It is fail-closed, but it predictably aborts the
  smoke and so uses up an approval.
- Second effect: the process's first observe+act took 0.53 s (later calls 5 ms). On smoke 0 that would give
  RUNNER_LATE at k=1 even after reordering. The historical runner silently used a later sample instead (dev07
  `compute_ms_max` ≤ 38 ms does not show the warm-up). Amendment2 also lists this order as "as the historical
  runner".
- *Fix:* build PrefixEpisode before `align()`. Warm up Warp and torch once per process on a scratch Episode
  before the first member (check that the policy player is stateless, or reset it). Add a CPU test with a
  100 ms Episode constructor and a 0.5 s first act.

**M2. The execution guard accepts uncommitted runtime code.**
`tools/sitl_dev08_attempt.py:68` (`status --porcelain --untracked-files=no`), `:130`, SOURCES `:28–30`;
`tools/sitl_s3b_dev08_prefix.py` 3904b8e5 lines 333–337 (aff090f3 line 345).
- In this worktree right now `dirty` is "" while `sitl_owned_session.py`, `sitl_dev08_link.py`,
  `sitl_dev08_step.py`, `sitl_dev08_attempt.py` and `sitl_s3b_dev08_prefix.py` are all untracked. An approval
  naming HEAD `9051cea` would pass `execute()` and `AttemptRecorder.open()` and run code that is in no commit.
- SOURCES also omits `tools/sitl_s3b_dev08_prefix.py` (the module that executes) and amendment2.
- The bridge loads `MOTAR-cplanner/cplanner/motar/replay.py` from another worktree (`cplanner/dev 2837b18`).
  Its hash and Git state are recorded nowhere.
- *Fix:* for every runtime path require `git ls-files --error-unmatch` and `git hash-object <path>` ==
  `git rev-parse HEAD:<path>`. Add the driver, amendment2 and the bridge's production files to SOURCES. Record
  the cplanner file's SHA-256 together with that repository's HEAD and clean state.

**M3. The identity test is blind to several parts of Q1's contract.**
`tests/test_sitl_dev08_step.py` line 207 (`np.tile(target, …)`, a constant target), line 62 (`"vz": 0.0`
always); no scenario for out-of-bounds, the altitude ceiling, a fast crossing, or capture and crash together.
- I ran 18 mutants of the step module against a harness that mirrors the test's own comparison
  (`probes/step_mutations.out`). 11 were caught: setpoint vx/vy order, yaw-rate sign, EKF yaw sign, command yaw
  EKF instead of truth, no prev_action feedback, render pose = EKF, velocity N/E swap, angvel sign, truth E/N
  swap, cos(lat0) dropped, and a command counted at the terminal.
- **7 survived:** target index k+1 (clamped), target index 0, vz sign, point instead of swept capture, crash
  evaluated before capture, out-of-bounds margin removed, altitude ceiling removed.
- The test docstring's claim that "any difference in the state, the sample chosen or the frame transform
  changes every later command" is therefore not true for the target index or vz. The code is correct by
  inspection on all seven. The partial real-bridge record (205/env8, moving target, 63 identical commands) is
  external and pre-fix.
- *Fix:* a moving target (tpos varies with k), nonzero vertical velocity, a fast crossing where the swept
  distance is < 0.5 m and the point distance > 0.5 m, a state where capture and crash both hold, and
  out-of-bounds and altitude crash scenes.

### MINOR

- **m1. The recorder trusts a self-declared VALID.** `sitl_dev08_attempt.py:199` checks only
  `operational_status`. A smoke record with inconsistent counts, a `timeout` outcome or a premature censor lets
  the main phase start. Amendment1 requires the smoke to be a valid terminal or censor. For main records the
  analyzer refuses later. *Fix:* run the per-record checks of `summarize_complete_pairs` (counts, outcome set,
  censor only at the last index, times) inside `append`, for smoke too.
- **m2. Bridge and GPU are touched before the environment check.** `execute()` runs `B.setup()` (`wp.init()`)
  and `B.Policy()` (aff090f3 lines 357–359) before `rec.open()` checks CUDA_VISIBLE_DEVICES, affinity and nice
  (line 365). If CUDA were visible, Warp would initialise it before the refusal. There is also no fail-closed
  check that `aerial_gym` resolves inside the worktree (AGENTS.md worktree rule). *Fix:* check the environment
  and package resolution first.
- **m3. Paths that leave no final.json, and budget boundaries.**
  - An exception that escapes `run` (for example from the post-session writes or a KeyError), or a SIGTERM
    while no session is open, leaves a ledger without final.json and without the unrun list.
  - The overall and density budgets are checked only between members, so a run can exceed 5400 s by one member
    (up to 240 s plus about 45 s of cleanup).
  - Amendment2's `timeout -k 20 5600` leaves 200 s of margin, and its 20 s kill grace equals the session's
    SIGINT grace, so SIGKILL can land in the middle of cleanup.
  - *Fix:* wrap the loop in try/finally with `abort("RUNNER_EXCEPTION")`, handle SIGTERM for the whole of
    `execute()`, refuse to start a member when less than one episode deadline plus cleanup remains, and use
    `-k` of 60 s or more.
- **m4. The source-change check is incomplete.** It runs only before each member, so a change during the last
  member still ends COMPLETE. The per-episode PX4/Gazebo `runtime_binding` is recorded but never compared
  across episodes. *Fix:* check after each member too, and abort on any binding change.
- **m5. Evidence is lost on failure.**
  - When `_fly` raises after the parameter phase, `out` (booted parameters, start check) is discarded.
    Amendment1 says to preserve the failed attempt.
  - The checkpoint rows lack the snapshot's freshness evidence (`pos_lateness_s`, `att_skew_s`,
    `truth_recv_skew_s`, sample seqs). Amendment1 asks the live boundary to bind those checks.
  - *Fix:* fill the record progressively and store the snapshot metadata in every row.
- **m6. The attempt header is thin.**
  - Not recorded: the `prepare()` inventory result (34 verified files, discarded in `execute()` as `_`);
    platform (Python, torch and warp versions); CPU topology and offlined cores; OMP/MKL thread counts, which
    amendment2 sets to 4 but nothing checks; the idle-host state.
  - `kernel_faults()` ignores journalctl's return code, so the fault check would be vacuous if the journal
    were unreadable. It is readable on this host: rc 0, count 0.
  - The chain root (`prev_sha256` of line 0 = None) is not bound to attempt.json.
  - `main_records` does not check `kernel_faults_end == kernel_faults_start`.
- **m7. Rx thread and boot telemetry (3904b8e5 only).** The rx thread's liveness after `join(timeout=5)` is not
  checked. In 3904b8e5 the gate was created before the start phase with start_cursor 0, so boot and takeoff
  telemetry was judged; aff090f3 changes this (not reviewed).
- **m8. Amendment2's live-assumption list is incomplete** (see Q6).
- **m9. README draft.** It links `validation.json`, which does not exist, and Reviews is a placeholder. "From
  k = 1 or 2 for other start samples" overgeneralises: over start samples 1500–1999, 198/500 diverge at k=1–2
  and the rest at k = 4…257. The Limits bullet "attitude/truth chosen by time (nearest)" is outdated for truth.
- **m10. Not indexed.** `docs/EXPERIMENTS.md` lists neither amendment2 nor `dev08/runtime_stack/`. AGENTS.md
  ("Connect every new record") and `tests/test_experiment_index.py` require it in the same commit.
- **m11. One recorder check is untested.** A mutant that drops the schedule-name check in `read_ledger` passes
  the attempt tests (`probes/attempt_mutations.sh`, A08). The other 10 attempt mutants are caught.
- **m12. Start-check record.** The driver's `start_check` omits the historical `ekf_minus_truth_xy_m` field.

### NOTE

- **n1. Position sample selection.** The gate uses the first position at or after t_k; the historical loop used
  the newest at loop exit. They are identical when the runner is on time. RUNNER_LATE bounds the gap to 0.05 s,
  but amendment2's Differences section should say this.
- **n2. One bridge setup for both densities.** `B.setup()` runs once, with the 70-bar receipt's NAVRL_*
  snapshot, for all 42 members; dev07 ran one process per density. The differing keys are NAVRL_NUM_BARS,
  NAVRL_DENSITY_START/FINAL and path/nonce keys. Only `task_config.density` reads them, and the bridge does not
  use that class, so there is no effect found, but the record should state it.
- **n3. The integration fixture is not hermetic.** It uses the owned session's defaults
  (`px4_lock=/tmp/px4_lock-0`, `exclusive_udp=(14540,)`), so it would fail while a real SITL runs on the host.
- **n4. Approval provenance.** `execute()` checks only the form of a caller-supplied dict. The CLI that
  amendment2 defers must read a user-written approval record and bind its hash.
- **n5. Shared send lock.** At 3904b8e5 the Commander and the transport held separate locks. All sends are on
  the main thread, so there was no race; aff090f3 shares the lock.
- **n6. Truth origin not exercised by the identity test.** The identity test passes the origin at t=13.0 s; the
  legacy runner reads it at 13.5 s. The scripted world is static then, so the driver's choice of origin is
  tested only by the fake-PX4 integration test.

## Answers

**Q1. Native command-prefix identity.**
- *Code: yes.* `PrefixEpisode.state/step` reproduces the historical loop body expression by expression:
  - ENU from NED (`gx, gy = pn[2], pn[1]`), `rot(yaw0, …)` and `p0` offset; the EKF yaw is
    `yaw0 + (π/2 − att.yaw)` and the truth yaw comes from the truth quaternion the same way;
  - the truth origin uses `R_E`, `cos(lat0)` and `alt − alt0`;
  - the render pose is truth and the policy pose is the EKF estimate;
  - `angvel = (0, 0, −yawspeed)`; the target is `ttraj[k]` with no clamp (bounded by the window and a length
    check); prev_action is fed back from `ep.command`;
  - the command yaw is truth (`S3B_COMMAND_YAW` is ignored); the setpoint is
    `(TM_POS_Z_VEL_XY_YAWRATE, 0, 0, −ALT, gey, gex, −wz)`, which the Commander sends with the same 16-argument
    order;
  - geometry and constants come from the hash-pinned runner (`RUNNER_SHA` == PINS, tested).
- *Measured:* exact identity in all four scenarios, including 600 checkpoints, under historical sample selection.
- *Harness: genuine.* It loads the real `tools/sitl_s3b_closed_loop.py`, fakes only time, Link, write_world and
  start/stop, and compiles the `Link.rx` conversions from that file's AST. It is not a copy.
- *Detection:* it would catch sign, frame and sample errors on fields that vary (11/18 mutants), plus a
  one-sample shift (positive control). It misses the target index, vz, swept capture, precedence, out-of-bounds
  and the altitude ceiling (M3).
- *Stub sensitivity:* a dense projection of every state field with prev_action feedback is sensitive enough.
  The gap is input coverage, not the stub.
- *Terminal relation:* asserted correctly. For capture and crash, legacy commands = n_cmd + 1, legacy observes =
  n_cmd + 1, and the new last command is None. For the censor, legacy continued with the clamped target.

**Q2. Stopping.** Correct.
- Swept capture is computed from the previous relative position; crash is bar, altitude (< 0.1 or > 4.0) or
  out of bounds with the 1.0 m margin; `checkpoint_decision` applies capture > crash > CENSORED at the last index
  > CONTINUE to the current snapshot.
- On any decision other than CONTINUE, `step` returns before observing, and `_fly` breaks before `setpoint`.
- `counts()` gives final_frame_index = k, observation_checks = k + 1, commands_issued = k. The integration test
  checks this against the fake PX4's own count of velocity setpoints.

**Q3. Attempt recorder.**
- Correct: two smoke members (70 then 205), then 70 before 205, env0..9, even envs native first; enforced
  member by member (no skip, rerun or reorder); append-only SHA chain; abort on non-VALID, smoke failure, a
  higher fault count or a budget overrun; source change aborts in `execute()`; the unrun list goes in final.json;
  `main_records` refuses anything not COMPLETE; the output is compatible with `summarize_complete_pairs`
  (tested).
- A COMPLETE attempt cannot hide a non-VALID or missing member. It needs 42 in-order VALID appends with an
  unchanged fault count, and `read_ledger` checks chain, order and final count.
- Smoke cannot enter the main denominator: phase is checked against the schedule and filtered, and the analyzer
  also requires `phase=main`.
- Weak points: VALID is self-declared (m1); the chain is not rooted in the header; the end-of-run fault count is
  not rechecked; some exception and SIGTERM paths leave no final.json (m3).

**Q4. Driver.**
- Faithful: boot 3 s; heartbeat 120 s; 20 ms message intervals for 32/30/115; streams wait 90 s polling at
  0.2 s; settle 10 s; then parameters; then origin; holds of 2 / 1 / 10 s and 3 × 3 s truth-offset holds; the
  start check with identical thresholds; DO_SET_MODE 1,4,6 and 1 s afterwards.
- Not faithful: the Episode is built after the start is fixed (M1).
- The parameter guard runs before arming.
- Failures become HARNESS_INVALID or ERROR without raising: `Exception` is caught, while KeyboardInterrupt and
  other BaseException propagate. Session cleanup errors and aborts after a VALID `_fly` overwrite the record to
  ERROR.
- No path turns a harness failure into a valid censor. A censor needs a successful gate checkpoint at the last
  window index, and the analyzer additionally rejects a premature censor.
- Cleanup: stop event, join, conn.close inside the session; the session's owned-only cleanup is tested with zero
  leaks. Gaps: rx liveness and partial evidence (m5, m7).

**Q5. execute().**
- Without an approval it raises PermissionError. The approval is checked only for form; nothing checks where it
  came from (n4).
- A commit other than HEAD is refused, and a tracked-dirty tree is refused.
- **Untracked runtime sources pass** (M2).
- `main()` only prints and returns 2. Importing the module has no side effects, and no other caller of
  `execute()` exists.

**Q6. amendment2.**
- Checked against sources and correct:
  - ports, process names and spawn arguments (`sitl_run.sh` lines 104/130/154);
  - `px4 -d <build>/etc`; the rcS `parameters.bson` import (rcS lines 67/73);
  - the candidate gains; the 42-member order;
  - 36 s of historical waits (3 + 10 + 22 + 1);
  - dev07 "18–75 observation checks" (205/env8 crash at 18, 205/env3 at 75) and RTF 0.996–1.004 (0.9964 and
    1.0037);
  - 59.9 s for a full censor; 34 inventory files; 20 identity-quaternion scenes.
- Wrong or stale: listed in B2 (a)–(f).
- Amendment1's estimand, cohort, envs, arms, analysis and stop rules are unchanged. Two additions: the smoke
  order 70 → 205 (amendment1 did not fix it) and the outer `timeout 5600` (see m3).
- Live assumptions the smoke would also need to confirm:
  - (i) The truth timing model: HIL_STATE_QUATERNION arrives in the same send burst as LOCAL_POSITION_NED,
    within +5 ms or −30 ms, never with a usable time_usec. This replaces assumption 3.
  - (ii) The runner keeps up at k=0 and k=1: Episode construction and first-call warm-up against RUNNER_LATE
    (M1).
  - (iii) The direct launch is equivalent: lockstep connection on TCP 4560, and PX4_SIM_MODEL resolves to
    airframe 10090 in an empty rootfs that has the `etc` link.
  - (iv) The booted values in an empty rootfs equal airframe-or-firmware parameters.json defaults for all 27
    parameters.
  - (v) UDP loss or jitter at 3 × 50 Hz stays inside the 0.05 s skip, late and freshness bounds over 600
    checkpoints (assumption 4 only states the rates).
  - (vi) Heartbeat custom_mode and armed bit come from (1,1); the onboard link accepts system 245 setpoints and
    SET_MESSAGE_INTERVAL for id 115.
  - (vii) The per-episode wall time (PX4 boot in a fresh rootfs, parameter exchange of 64 native or 67 candidate round trips,
    `runtime_binding` hashing of about 40 plugins plus the binaries) fits 240 s, and the cohort fits the 2400 s
    and 5400 s budgets.
  - (viii) Nothing writes outside the episode directory: gzserver's `~/.gazebo`, the model database, PX4 logs.
  - (ix) Boot and takeoff telemetry does not trip the gate's finite, quaternion or monotonic checks (relevant to
    3904b8e5).

## Post-snapshot state (observed, not reviewed)

- 18:16–18:19: driver aff090f3 and test 4652cc13 with link bf3ab740. `test_sitl_s3b_dev08_prefix.py` 8/8 OK
  (17 s). The driver now uses `fresh_expectations(airframe, parameters.json)`, a gate start cursor 1 s before
  alignment, a shared send lock and `latest()` for the start phase; the fixture uses `time_usec=0`. M1 is still
  present (aff090f3 lines 216–217).
- 18:20: step test 29499175 (receipt clock: sample i at i·0.02 s + 1e-4·j per message) with link bf3ab740:
  7/7 OK in 0.8 s. This appears to resolve B1's harness defect. M3's blind spots remain, since the target is still
  constant and vz still 0.
- These bytes, the real-bridge identity rerun and a corrected amendment2 need review round r2.

## What I ran

All runs were CPU-only (`CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`,
`taskset -c 6,7,12,13`, `timeout`). No PX4, Gazebo or GPU was used, and no forbidden port was opened (the
fixtures bind random ports). Nothing was written in any repository or the PX4 tree. Only read-only git
commands were used.

- `unittest` on `test_sitl_dev08_step.py`, `test_sitl_dev08_attempt.py` and `test_sitl_s3b_dev08_prefix.py`,
  repeated as the dependencies changed (results in B1 and Post-snapshot). I also started
  `test_sitl_dev08_link.py` once; it is out of scope and ended at my 300 s timeout.
- `probes/trace_driver*.py`: located the integration failure in the link fixture `fresh_boot` assert.
- `probes/step_mutations.py` on `probe_tree/` (copies): link-independent identity baseline (4 scenarios),
  tolerance reproduction, 18 step mutants → `probes/step_mutations.out`.
- `probes/attempt_mutations.sh`: 11 recorder mutants against its test file; 10 caught, A08 survived.
- `probes/episode_ctor_timing.py`: real `sitl_bridge.Episode` construction and the first observe/act timing on
  CPU, with the real Policy F (read-only inputs) → `probes/episode_ctor_timing.out`.
- Read-only checks:
  - PX4 v1.14.3 `HIL_STATE_QUATERNION.hpp`, `sitl_run.sh`, rcS, the 10090 airframe, the plugin count (39
    top-level, 40 recursive);
  - the dev07 reports (outcomes, steps, RTF, compute time) and the 70- vs 205-bar receipt env diff;
  - `git status --porcelain --untracked-files=no` (empty) against the untracked runtime modules;
  - MOTAR-cplanner replay.py state; journalctl readability; EXPERIMENTS.md index entries.
- Snapshot of every reviewed file: `snapshot_1803/` with `SHA256SUMS`, plus `*.181638` copies of the later
  driver and test.
