# AGQ geometry-v1 source review r1

**Verdict: FAIL.** There are 4 open MAJOR findings, 0 BLOCKER and 11 MINOR.

The guarded unit and the recording are sound in the ways that matter for safety:
- the approval, clean-tree, environment and single-launch gates hold;
- no recorded value reaches Policy F, the bridge, the gate or the controller;
- the subscriber is owned by the session and cleaned up with the other processes;
- the stop rules are enforced in the stated order, and an interrupted member is named;
- the Gazebo 11 and PX4 v1.14.3 source claims hold.

The failures are in the fixed reading (section 6 and its code) and in the portability of the tests. The reading must be fixed before the preregistration is committed, because committing it freezes the analysis.

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, independent read-only audit.
- **Date:** 2026-11-06, 02:08–02:27 +09:00.
- **Repository:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb7664b1828dabd11bd2d45f770a8cafec7883`.
- **Writes:** only under `/home/fair/motar_tmp/agq_geometry_diag_20261106/source_review_r1/`. Nothing was written into any Git worktree. No Git state was changed.
- **Simulator and GPU:** no simulator process (gzserver, gz, px4) and no GPU were used.

## Hash check

Hashes were checked against `BYTES_UNDER_REVIEW.sha256` with `sha256sum -c`.

| File | Start (02:08:00) | End (02:26:33) |
| --- | --- | --- |
| tools/sitl_agq_geometry_diag.py `352cad66…` | OK | OK |
| tools/sitl_agq_geometry_launch.py `ad918c92…` | OK | OK |
| tests/test_sitl_agq_geometry_diag.py `d84c4145…` | OK | OK |
| docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md `42ae2c29…` | OK | OK |
| tools/sitl_training_geometry.py `cadd5167…` | OK | OK |

`docs/EXPERIMENTS.md` was modified in the worktree during the review (mtime 02:08:07). The diff is the author's index entry for this preregistration. It is not under review and was not written by me.

## What I verified and how

### Source read in full

- The four files under review and `tools/sitl_training_geometry.py`.
- The dev08 runtime stack that the new code wraps or relies on: `sitl_s3b_dev08_prefix.py` (run_episode, _fly, execute), `sitl_owned_session.py`, `sitl_dev08_link.py`, `sitl_dev08_step.py` and `sitl_dev08_attempt.py`.
- The parent proposal, the AGQ README and DIAGNOSIS.md.
- `git diff --stat d8f9bc6 HEAD` over the nine runtime-stack files is empty. The stack is byte-identical to the dev08 unit's source, as the preregistration says.

### Upstream source

I downloaded the Gazebo gazebo11 branch with `curl` into `upstream_gazebo11/`. The installed version is 11.15.1, protobuf 3.6.1.

- `ContactManager.cc`
  - :80 `Advertise<msgs::Contacts>("~/physics/contacts", 50)`: the queue is 50.
  - :286–313 `PublishContacts()` publishes on every call, unless minimal comms is set. It skips contacts with count 0, so empty messages are still sent. The message is stamped `world->SimTime()`.
- `World.cc`
  - :839–841: `simTime += stepTime`, then `Update()`.
  - :1003: one `PublishContacts()` per `Update()`.
  - So there is one message per 4 ms step, stamped with the step's simulator time.
- `Publisher.cc`:129–139: a message beyond the queue limit is dropped, with a `gzwarn`.
- `gz_topic.cc`
  - :191–198: `-u` prints `ShortDebugString() << "\n"`, with no flush.
  - :201–231: echo blocks on `sigCondition`.
  - `gz.cc`:1315: SIGINT is handled, so the process exits cleanly.
  - gz and its libraries do not call `sync_with_stdio` or `setvbuf` (checked with `nm -D`), so `stdbuf -oL` reaches `std::cout`.
- `contact.proto`, `contacts.proto`, `joint_wrench.proto`, `wrench.proto` and `Contact::FillMsg`: the fields match the parser.
- `ODEPhysics.cc`:989–999: the collision callback skips disabled bodies unless `SubscribersConnected()`. See finding m10.

### World and models

These are in the PX4 v1.14.3 tree.

- **World:** `empty.world` is named `default`. The topic is therefore `/gazebo/default/physics/contacts`. The step is 0.004 s.
- **Floor and bars:**
  - `asphalt_plane` is a static 200×200×0.1 box. Its collision is `asphalt_plane::link::collision`, and the vehicle rests on it.
  - The ground plane is `ground_plane::…`.
  - The bars are named `bar_<i>::l::c` by `world_text`.
- **Vehicle:**
  - The spawn uses `--model-name=navrl_quad`.
  - The navrl_quad collisions are exactly `base_link_inertia_collision` and `rotor_0..3_collision`.
  - The included `gps` and `lidar` models have no collision elements, so no vehicle part goes unnamed.

### Time axis

- `gazebo_mavlink_interface.cpp`:713 sets `time_usec = llround(SimTime*1e6)`.
- `SimulatorMavlink.cpp`:481–483 sets `px4_clock_settime(imu.time_usec)`.
- `drv_hrt.cpp`:101–105 returns `hrt_absolute_time() = lockstep_scheduler.get_absolute_time()`, with no offset.
- `build.ninja` defines `-DENABLE_LOCKSTEP_SCHEDULER`.
- The groundtruth uORB topics are stamped with `hrt_absolute_time()` (`SimulatorMavlink.cpp`:521).
- **Conclusion:** the source-level claim of a single time axis holds, to within one step.
- `logged_topics.cpp`:250–253 logs angular-velocity groundtruth at a 10 ms interval and local-position groundtruth at 20 ms.

### External evidence

- **`stdbuf_check/`:** `probe.cpp` mirrors gz's `<< "\n"` pattern. `result.txt` shows 0 lines without stdbuf and 50 with it; the 50 lines are present in the output.
- **`dry_binding_v1/`:**
  - `dry_binding.json` has 60 identical core runtime files across the four members (20 non-plugin files and 40 plugins).
  - `model_sdf` is `f9188e90…` for the native members and `1b658c0b…` for the training-box members.
  - The native package is byte-identical to the PX4 SDF.
  - Limits of this check: it used a placeholder world and `PATH=/usr/bin:/bin`. The live BINDING_CHANGED check covers the real environment.
- **dev08 attempt01 records** (read-only):
  - members took 40.7–51.2 s of wall time, real-time factor about 1.000, compute max ≤ 11 ms;
  - cleanup was SIGINT only and the attempt took 1888 s for 42 members;
  - start truth altitude was 0.966–1.000 m.

### Commands I ran

All commands used the required envelope:

```
CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 timeout -k 10 900 nice -n 10 taskset -c 16,17,18,19 bash -c 'ulimit -v 8388608; /home/fair/miniconda3/envs/aerialgym/bin/python -B …'
```

1. **The focused test file.**
   - Command: `python -B tests/test_sitl_agq_geometry_diag.py -v`, logged in `focused_tests.log`.
   - Result: **18 run, 2 ERROR** (`Unit.test_complete_unit_in_fixed_order`, `Unit.test_stop_rules`). Both errors are `AttemptError: CPU affinity [16, 17, 18, 19] outside [6, 7, 12, 13]`. See M4.
2. **The same file with only the affinity report patched.**
   - Script: `scripts/run_tests_affinity_patched.py` reports `os.sched_getaffinity` as {6,7,12,13}. The real affinity stays 16–19.
   - Log: `focused_tests_affinity_patched.log`.
   - Result: **18/18 OK** in 43 s.
3. **Cost probe.**
   - Script: `scripts/cost_probe.py`, output in `cost_probe/cost_probe.json`. It uses synthetic ShortDebugString logs with 4-point floor contacts and full wrenches, 2276 B per message.
   - `floor_control` takes 0.098–0.104 s on a 17.5 MiB log.
   - `contact_audit` takes 4.36 s on a 25.7 MiB member log.
   - When the vehicle touches the floor again after the final checkpoint, `floor.last_t` moves to the end of the log.
4. **Old ULog check** (px4 env, pyulog; `scripts/post_end_floor.py`).
   - In dev08 `33_main_205bars_env5_native`, the vehicle is within 2 cm of its resting height 8 ms after the final checkpoint.
   - The ULog continues for another 0.99 s.
5. **Reading probes** against the real functions and the real `execute()` with the test file's fake fixtures (`scripts/reading_probes.py`, output in `reading_probes.json`):
   - **P1:** `takeoff_check` passes when the last floor contact is at takeoff. It fails (`ok False`) when the last floor contact is after a crash.
   - **P2:** with the native member reproduced (contact 5.73 s, Rate5 5.77 s), each of these box members reads `CONSISTENT_WITH_ROTOR_CONTACT`:
     - first Rate5 at 4.9 s;
     - first Rate5 at 6.3 s;
     - its own bar contact at 6.25–6.30 s with Rate5 at 6.3 s.
   - **P3:** in an attempt that ended `VOID_HOST_FAULT` after member 1, both members are `evaluable: true` and the env5 pair gets a reading, not NOT_EVALUABLE.
   - **P4:** in an attempt stopped by `TIMING_GUARD` at member 1 (real-time factor 0.9), member 1 is evaluable and the env5 pair gets a reading.
6. **Analysis in the px4 environment.** `D.modules()` loads in the `px4` env (Python 3.8, pyulog present). The aerialgym env has no pyulog, as the preregistration says.

## Findings

### MAJOR

**M1. The fixed pair reading can return `CONSISTENT_WITH_ROTOR_CONTACT` by construction when the training-box member had its own disturbance.**

Where:
- `tools/sitl_agq_geometry_diag.py:595-596` (Rate5 is the *first* sample after start);
- `tools/sitl_agq_geometry_diag.py:632-651` (path guard and window logic);
- preregistration lines 137-138 and 152-159;
- the test encodes this behaviour at `tests/test_sitl_agq_geometry_diag.py:425` (`rate=6.5 → CONSISTENT_WITH_ROTOR_CONTACT`).

Failure scenarios (each confirmed by probe P2):
- **(a) Earlier excursion.** The box member exceeds 5 rad/s before the window, for example 0.8 s before the native contact. Only its first excursion is computed, so a later excursion inside the window is never examined. The pair reads CONSISTENT.
- **(b) Later disturbance.** The box member has the same disturbance, or its own bar contact followed by a disturbance, just after `t_c + 0.5` s. Without the rotor cylinders a slightly different path can delay the encounter. The pair still reads CONSISTENT.
- **(c) Path divergence.** Paths are only compared up to `t_c − 0.2` s, but the decision window runs to `t_c + 0.5` s. A box member that drifts away from the bar in the last 0.2–0.7 s has "no contact" because it was elsewhere. That also reads CONSISTENT. At about 2 m/s, 0.2 s is 0.4 m, which exceeds the 0.3 m guard. The old env5 clearance margin between the rotor reach and the base box was about 0.04–0.1 m (DIAGNOSIS: clearance 0.239 m against a base half-diagonal of 0.198 m).

Related but conservative: using the *first* Rate5 for the native member means a harmless earlier excursion blocks reproduction.

These are the hypothesis-favourable outcomes the review was asked to look for. The label states more than the definition supports.

Suggested fix, before commit:
- In the window, test "any ULog sample in `[t_c−0.5, t_c+0.5]` with a rate above 5 rad/s", not the first sample after start. Do the same for the native member around `t_c`.
- Read CONSISTENT only if the box member has no Rate5 and no bar contact in `[t_c−0.5, t_c+T]`, with T ≥ 2 s or until the member ends. Otherwise return a distinct label such as `DISTURBANCE_LATER` or `INCONCLUSIVE`.
- Require the path match through `t_c + 0.5` s, or report the box member's minimum clearance to the native contact bar in the window and return `INCONCLUSIVE_PATHS_DIVERGED` above a fixed bound.
- Change the test at :425 and add negative controls for (a), (b) and (c).

**M2. The clock check anchors on the last floor contact of the whole log, which is after the crash in the case the unit is designed for.**

Where:
- `tools/sitl_agq_geometry_diag.py:738` passes `audit["floor"]["last_t"]`;
- `tools/sitl_agq_geometry_diag.py:675-689`;
- preregistration lines 161-168.

Failure scenario:
- The old env5 native member ended in an altitude crash (`p_true` z < 0.1 m). In its ULog the vehicle is back at its resting height 8 ms after the final checkpoint.
- The runner then sleeps `after_s` = 1 s before cleanup, and the contact subscriber records throughout.
- A reproducing native member will therefore log floor contacts after the crash. `floor.last_t` is then that touchdown, not the takeoff.
- The 1 cm rise is then not found after `last − 1 s` (probe P1: `ok False`), so the pair becomes `LIMITED_VALIDITY`.
- By the preregistration's own rule, the contact-versus-ULog-rate ordering, which is the reproduction test, is then not read.
- The check rejects the key valid case. Bar crashes that fall to the floor within the after-phase have the same problem.

Suggested fix:
- Use the end of the last floor-contact interval that begins before `start_sim_time_s`, or before the first offboard setpoint.
- Add a test in which the vehicle returns to the floor after the final checkpoint.

**M3. `analyze()` reads members that the stop rules invalidated.**

Where:
- `tools/sitl_agq_geometry_diag.py:717-723`: `final` is read but never gated, and `evaluable` ignores `timing_problems`;
- `tools/sitl_agq_geometry_diag.py:743-750`;
- preregistration line 147.

Failure scenarios:
- **Host fault:** the kernel fault count rises during member 1. `append()` records member 1 as VALID and then ends the attempt `VOID_HOST_FAULT`. `analyze()` still marks members 0 and 1 evaluable and produces an env5 pair reading (probe P3). AGENTS.md says host faults void a run: "its results are not evidence".
- **Timing guard:** a member outside the timing guard (real-time factor 0.9, or compute max ≥ 100 ms) stops the unit as TIMING_GUARD but stays evaluable. Its reading is produced (probe P4).
- **Kernel log unreadable:** FAULT_CHECK_ERROR is handled the same way.

Suggested fix:
- If `final.status == "VOID_HOST_FAULT"` or `final.reason == "FAULT_CHECK_ERROR"`, make every pair NOT_EVALUABLE, or refuse to analyze.
- Treat non-empty `timing_problems` as not evaluable.
- Write both rules into section 6.1.
- Add tests for both.

**M4. The two `Unit` tests pass only when the process affinity is within {6,7,12,13} and nice ≥ 10.**

Where:
- `tests/test_sitl_agq_geometry_diag.py:502-505`. The recorder fixture does not override `cores` or `min_nice`, and `AttemptRecorder.open()` re-reads the real process environment.

Failure scenario:
- `python -B tools/run_tests.py` (PUBLIC_CPU) discovers every `test_*.py` and runs them in one process with no pinning. These two tests would error, which breaks the repository's "0 failures, 0 errors" gate.
- They error in this review's envelope (cores 16–19). They pass once only the affinity report is patched.
- The author's runs (`logs/focused_v*.log`) were evidently pinned to the runner's cores, which are currently busy.
- The dev08 tests avoid this: `tests/test_sitl_s3b_dev08_prefix.py:403-406` passes `cores=frozenset(os.sched_getaffinity(0)), min_nice=os.nice(0)`, and :332 patches `environment_problems`.

Suggested fix:
- Do the same in `recorder()`.
- Confirm the file with the full PUBLIC_CPU suite, unpinned, before commit.

### MINOR

**m1. The positive control's cost and side effects are not recorded.**

Where: `tools/sitl_agq_geometry_diag.py:480-486`.

- `floor_control` runs on the main thread between the parameter guard and `pn`/`tr0`/`hold_pre`. It delays the truth origin and the first setpoint by about 0.1 s on idle cores (measured), and more on the shared runner cores.
- The effect on a disarmed vehicle is negligible. But the preregistration's "unchanged" claim should state it, and the record should carry `t_entry` and `t_return` of the control.
- On a control failure, the parameter result (`out["parameters"]`) is lost, because the exception replaces the return value. Store it in `tap` first.

**m2. The control does not check freshness.**

Where: `tools/sitl_agq_geometry_diag.py:230-262`.

- A subscriber that stalled, while its process stays alive, still passes on an old, contiguous, floor-contact tail.
- The audit after the member catches the loss later, so this is not silent.
- Suggested fix: also require the last control message's simulator time to be within about 0.5 s of the newest `LOCAL_POSITION_NED` `time_boot_ms`.

**m3. Continuity is audited over the whole log, including boot.**

Where: `tools/sitl_agq_geometry_diag.py:293-301` and `:361-363`.

- A publisher queue drop during PX4 boot, about 30 s before the gate start, stops the unit as RECORDER_LOSS, although it is outside every reading window.
- This fails closed, so it is safe, but it is an avoidable way to lose the unit.
- Suggested fix: either keep it and state the risk, or gate continuity on `[start−1 s, final+0.5 s]` and report other gaps without stopping.

**m4. The raw contact log is not bound, and the recomputed audit does not gate.**

Where: `tools/sitl_agq_geometry_diag.py:909-917` and `:728-740`.

- The ledger's recording manifest is computed before `contacts.jsonl` is written, so it does not cover that file. `logs/contacts.log` is hashed nowhere.
- `analyze()` recomputes from this unbound file, and `audit_problems_recomputed` does not change `evaluable`.
- Suggested fix: hash both files into the ledger record, and make a mismatch, or any recomputed problem, NOT_EVALUABLE.

**m5. The cross-clock label is miscounted, and the reading is emitted regardless of it.**

Where: `tools/sitl_agq_geometry_diag.py:748-750`.

- `cross_clock_order` is `SUPPORTED` when only one member of the pair is evaluable; the preregistration requires both.
- The `reading` label is emitted unchanged when the pair is `LIMITED_VALIDITY`, although the preregistration says the ordering is then not read.
- Suggested fix: count both members, and qualify or suppress order-dependent labels under LIMITED_VALIDITY.

**m6. An unreadable ULog crashes the analysis instead of giving NOT_EVALUABLE.**

Where: `tools/sitl_agq_geometry_diag.py:692-706`.

- A ULog that is present but corrupt, or lacks one of the two topics, raises from pyulog or a `KeyError`, and `analyze()` fails.
- The preregistration (line 138) says the pair is then NOT_EVALUABLE.
- Suggested fix: catch the error and return `None`, with the reason.

**m7. The velocity comparison pairs the new command with the old response.**

Where: `tools/sitl_agq_geometry_diag.py:656-672`.

- `velocities_at` compares the command generated at checkpoint k, which is sent after that checkpoint, with the truth velocity at k, which is the response to command k−1.
- That is not "actual versus requested" in the usual sense.
- The definition matches the preregistration, so this is not a code error. Suggested fix: also report the pair (command k−1, truth k), with both labelled.

**m8. The single-use and resource limits live only in the launcher.**

Where: `tools/sitl_agq_geometry_diag.py:816-837` and `:956-999`.

- `--execute` can be run directly, without the outer timeout or RLIMIT_AS, and with any fresh `--attempt-dir`. The same approval file could then be used twice.
- The environment, affinity and nice checks still apply. This matches dev08.
- Suggested fix: optionally check `resource.getrlimit(RLIMIT_AS)` and require the attempt directory to be the fixed `attempt01`.

**m9. Some narrowings of the parent proposal are not documented, and the subscriber binaries are not bound.**

- The parent proposal asks for "decoded MAVLink arrivals". The code keeps the Inbox types plus COMMAND_ACK and STATUSTEXT, and only counts the other types.
- Rate5 is any body axis, where DIAGNOSIS's "old pattern" used yaw only (env2 native: roll exceeded 5 rad/s 0.28 s before yaw did).
- "Impulses" are recorded as per-step forces.
- None of these is listed in section 9.
- `stdbuf` and `libstdbuf.so` are not hash-bound in the runtime binding; `gz` is bound as `spawn_executable`.
- Suggested fix: list the narrowings and bind the two files.

**m10. Section 8's ODE claim omits a concrete mechanism in the source.**

Where: preregistration lines 206-208.

- The claim "This should not change ODE's solution" is labelled NOT_VERIFIED.
- Upstream `ODEPhysics.cc`:989-999 bypasses the skip for disabled (auto-disabled) bodies whenever `SubscribersConnected()` is true. With the subscriber, contact joints are therefore generated for a resting, disabled body.
- This probably has no effect, because a contact with the static world does not wake an ODE island. But it is the one mechanism by which the subscriber changes collision processing, and it should be cited in the limits.

**m11. The worst-case cleanup can exceed the outer kill margin.**

Where: `tools/sitl_agq_geometry_launch.py:24-27`.

- The `-k 90` reasoning holds; amendment2 made the same change. dev08's real cleanups were SIGINT only, with about 0 s span.
- The worst case is:
  - the watchdog join, up to 35 s;
  - a second `_terminate`, up to 30 s;
  - the kernel-log read in `_final`, which has a 30 s `journalctl` timeout.
- Together these can exceed 90 s and orphan processes in their own sessions. The leak marker and `reap_journal` remain as the backstop.
- Accept as a stated limit, or shorten the journalctl timeout on the abort path.

## Criteria verdicts

**1. Faithful to the parent proposal: yes, with limits.**
- Members, order, gains, the single geometry change, the fixed runtime, the 240 s watchdog, the 12 GiB RSS cap, the 16 GiB address-space limit, one compute thread, the stop list and no retry all match the parent.
- The three stated deviations hold:
  - `-k 90` follows the amendment2 precedent;
  - the timing checks on every member are stricter (but see M3);
  - the native SDF is exported byte-identical, as verified.
- Undocumented narrowings: m9.

**2. Evaluation-only: yes.**
- The Inbox contents and the gate are unchanged.
- Sends are proxied inside the existing lock, with only clock reads and an append added.
- The per-checkpoint additions are a `waitid` and a list append. The subscriber starts after px4 and before `boot_wait`.
- The only start-phase timing change is the control's roughly 0.1–0.3 s (m1).
- Load from the subscriber is covered by the real-time-factor and compute guards.

**3. Subscriber design: sound for Gazebo 11 and PX4 v1.14.3.** All of these were verified in source:
- topic and world name;
- one message per step, empty or not, stamped with simulator time;
- ShortDebugString fields and the parser;
- the need for `stdbuf` and that it is effective;
- SIGINT handling;
- ownership and cleanup by OwnedSession;
- collision names;
- the time axis.

**4. Positive control, audit and recorder checks: non-vacuous.**
- They accept valid cases and reject the failures they claim to catch: 124 versus 125 floor messages, non-floor contacts, gaps, short tails, foreign lines, coverage and order.
- The thresholds fit 4 ms steps and the runner's 1 s after-phase.
- Limits: m2 and m3.

**5. Stop rules: complete and enforced.**
- Order, no retry, every attempted member recorded or named as `interrupted_member`, the recording written on termination.
- BINDING_CHANGED, RECORDER_*, TIMING_GUARD, SOURCE_CHANGED (before and during a member), HOST_NOT_IDLE, budget, HOST_FAULT and the signals all work.
- The private-module override does not leak: `_load` never registers in `sys.modules`, and the test confirms a fresh instance keeps observed-prefix-v1 and 42 members.
- AttemptRecorder default-argument pitfalls are handled: `sources` and `budget_s` are passed explicitly, and `schedule`, `PROTOCOL` and `AMENDMENT` resolve at call time.
- The gap is downstream: `analyze()` ignores the stop outcome (M3).

**6. Reading code: matches section 6 literally, but section 6 is inadequate.**
- The windows, units and NOT_EVALUABLE paths implement section 6 literally.
- The definitions allow a favourable reading by construction (M1).
- The clock check fails in the target case (M2).
- Invalidated members are read (M3).
- See also m5, m6 and m7.

**7. Launcher: no run without approval, on a dirty tree, or twice.**
- None is possible through the launcher.
- Processes are bounded by the owned session and the outer timeout.
- Limits: m8 and m11.

**8. Tests: meaningful, with paired positive and negative controls and non-zero counts.**
- Not portable (M4).
- Untested paths:
  - `analyze()` on a VOID, ABORTED or timing-flagged attempt;
  - `takeoff_check` after a return to the floor;
  - pair reading with the box member's Rate5 outside the window or a late contact;
  - HOST_NOT_IDLE, BUDGET_OVERALL, FAULT_CHECK_ERROR and RECORD_INVALID in this unit (they are covered for the shared recorder in the dev08 tests);
  - the launcher's success path, and refusal when the attempt directory or `run01` exists;
  - the `--execute` and `--analyze` command-line paths;
  - `ulog_series` on a real ULog;
  - a signal arriving inside an open session (the test raises it after `run_episode` returns);
  - real protobuf-generated contact text.

**9. Statements in the preregistration: no false statements found. Each claim is supported by source or evidence:**
- the single time axis;
- one message per step;
- the queue of 50;
- the stdbuf probe;
- the 60 identical runtime files (dry binding; limits as above);
- the byte-identical native SDF;
- the runtime stack identical to `d8f9bc6`;
- pyulog in the px4 environment.

Partly unsupported:
- "unreadable ULog → NOT_EVALUABLE" (m6);
- "recomputes the contact audit", which is reported but does not gate (m4);
- the "Tests show identical Inbox and gate output" claim holds only for one synthetic stream and checkpoint 0;
- the section 8 ODE claim (m10).

The duration estimate (4 × 75 s) is conservative; dev08 members took 41–51 s.

## Limits of this review

- No live Gazebo or PX4 run. The subscriber's real start-up output, the real queue and drop behaviour under load, the real ShortDebugString bytes and the floor-contact density at rest are NOT_VERIFIED.
- The parser was tested only on synthetic text that follows the protobuf 3.6 format.
- The upstream Gazebo source is the gazebo11 branch head, assumed to match the installed 11.15.1 package. The ODE auto-disable and contact-joint effects (m10) were reasoned from source, not measured.
- `ulog_series` was not run on a real ULog, but the field names were checked against the PX4 logger and uORB definitions.
- PUBLIC_CPU (the full suite) was not run, because it is a long job outside the scope. M4's prediction that it fails comes from `tools/run_tests.py`'s discovery and the fixture code, plus the reproduced error in a single process.
- The runner cores 6, 7, 12 and 13 were not used, so the author's pinned test conditions were emulated only by patching the affinity report.
- The scripts and outputs I created are in this directory: `scripts/`, `cost_probe/`, `reading_probes.json`, `focused_tests*.log`, `ulogs.txt` and `upstream_gazebo11/`. Treat the downloaded upstream files as untrusted data.
