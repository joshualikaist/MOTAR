# Dev08 amendment2 — runtime stack and bounded execution proposal, 2026-11-05

Parent: [`prereg_sitl_dev08_candidate_gains_v1.md`](prereg_sitl_dev08_candidate_gains_v1.md), then
[`amendment1`](prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md) (observed-prefix-v1).
Beads: motar-5md. This file adds what implementing amendment1's runtime stack fixed, and proposes the execution.
It changes no estimand, cohort, order or analysis of amendment1. Its added stops (HOST_NOT_IDLE, SOURCE_CHANGED,
FAULT_CHECK_ERROR, non-finite record) implement amendment1's existing abort conditions. It changes two amendment1
runtime bounds, stated under *Proposed execution*: the outer timeout's kill grace (20 s → 90 s) and the outer
timeout (5700 s; the 2400 s and 5400 s budgets are checked between members). **No execution is authorized by this
file.** No PX4, Gazebo, SITL or GPU process was started while preparing it; every live assumption below is
NOT_VERIFIED until the smoke.

## Runtime stack (implemented, CPU-verified only)

| Amendment1 row | Source | CPU fixtures |
| --- | --- | --- |
| Owned session | `tools/sitl_owned_session.py` | `tests/test_sitl_owned_session.py` |
| Parameter/telemetry boundary | `tools/sitl_dev08_link.py` | `tests/test_sitl_dev08_link.py` |
| Policy/command step | `tools/sitl_dev08_step.py` | `tests/test_sitl_dev08_step.py` |
| Attempt recorder | `tools/sitl_dev08_attempt.py` | `tests/test_sitl_dev08_attempt.py` |
| Integration, execution path | `tools/sitl_s3b_dev08_prefix.py` | `tests/test_sitl_s3b_dev08_prefix.py` |
| Analyzer | `tools/sitl_dev08_prefix_contract.py` `summarize_complete_pairs` (amendment1, unchanged) | existing |

Every module reports `live_verified=false`, `execution_supported=false`, `launch_ready=false`.
`tools/sitl_s3b_dev08_prefix.py main` refuses. Evidence and reviews:
[`docs/evidence/sitl_2026-11-05/dev08/runtime_stack/`](../evidence/sitl_2026-11-05/dev08/runtime_stack/).

## What the runtime does, per episode

1. **Session.** A new directory `<attempt>/<NN_phase_BBbars_envE_arm>/` under `~/motar_tmp`, outside every Git
   tree and the PX4 tree, with its own empty `rootfs/`, `world/`, `logs/` and `gazebo_log/`. One flock for PX4
   instance 0. The session is refused if any of these holds:
   - ports UDP 14540/14580/18570/14280/13030 or TCP 4560/11345 are bound;
   - PX4's own instance lock `/tmp/px4_lock-0` is held;
   - a process named gzserver, gzclient, gazebo or px4 runs;
   - an earlier session left a leak marker naming a live process.
   The three launches `sitl_run.sh` makes are made directly: `gzserver
   <world>`, `gz model --verbose --spawn-file=<navrl_quad.sdf> --model-name=navrl_quad -x 1.01 -y 0.98 -z 0.83`
   (bounded retry while Gazebo is not ready), `px4 -d <build>/etc` in the episode's rootfs. The environment is
   setup_gazebo.bash's plus `PX4_SIM_MODEL=gazebo-classic_navrl_quad`, `PX4_SIM_WORLD`, `HEADLESS=1`,
   `GAZEBO_LOG_PATH=<episode>/gazebo_log`, `CUDA_VISIBLE_DEVICES=`. No `sitl_run.sh`, no pkill, no shared rootfs, no
   gzclient. gzserver and the spawn run in the episode directory, and a spawn counts only on exit 0 without Gazebo's
   not-ready phrase (both stated differences). The record binds these by SHA-256, 62 files on this host (rcS also
   sources further scripts under build/etc that are not bound):
   - binaries;
   - installed rcS, px4-rc.*, rc.mc_defaults and airframe;
   - source rcS and airframe;
   - parameters.json;
   - the model SDF and every model:// resource it names, recursively;
   - the world and the empty.world template;
   - all 40 Gazebo plugins, recursively.
2. **Cleanup** signals only owned processes: those it launched, their session members, and descendants seen by the
   watchdog. The pid and start time are re-checked through a pidfd before each signal.
   - Sequence: SIGINT, SIGKILL after 20 s, a final scan, then reaping. Launched processes stay unreaped until that
     scan, so their session ids remain valid.
   - A survivor or a signalling error is an error and leaves a leak marker.
   - SIGTERM/SIGHUP to the runner run the same cleanup. A signal that arrives during start() or during the cleanup is
     deferred, then re-delivered after the receipt, so the unit stops; it is never dropped.
   - Every launch is journalled; an "open" leak marker names the journal, so a session killed before its cleanup
     blocks the next one. `reap_journal` reaps by identity after a SIGKILL.
   - Worst-case cleanup is about 35 s (20 s SIGINT grace, two 5 s SIGKILL waits, reaping).
   - Detection limit: a process that leaves its session and orphans itself within one sampling period (0.25 s; the
     cleanup's grace wait samples at the same period) is not seen.
   - Watchdog: 240 s per episode; 12 GiB sampled RSS over the runner and the owned tree (0.25 s; not a kernel cap);
     more than one socket on UDP 14540; an internal watchdog error.
3. **World.** The historical writer's text, byte-identical (CPU test), written into the episode directory. Bar
   orientation is dropped by that writer, so every bar quaternion must be the identity; all twenty fixed scenes
   are (read-only check, evidence folder).
4. **Link.** MAVLink `udpin:0.0.0.0:14540`, source system 245, as the historical runner. Heartbeat, message
   intervals 20 ms for LOCAL_POSITION_NED, ATTITUDE, HIL_STATE_QUATERNION, settle 10 s.
5. **Parameters.**
   - Read the 27 recorded REAL32 parameters as booted (`RECORD_PARAMS`). They include the yaw-rate group, which
     gives the fresh native values amendment1 requires, and the K/FF/MAX gains that scale the overridden roll and
     pitch gains.
   - Require each booted value to equal its fresh-boot expectation: the airframe's `param set-default`, else the
     bound build's parameters.json default. None of the 27 is set by rcS or rc.mc_defaults, all are Float there, and
     a reused parameters.bson fails here.
   - Set each override and require a PARAM_VALUE with exactly that float32 received after the set. Read everything
     back: overrides hold and nothing else moved.
   - Native is the historical S2 map. Candidate is the same with MPC_XY_VEL_P_ACC 2.0, MC_YAWRATE_P 0.2 and
     MC_YAWRATE_I 0.1.
6. **Start phase** as the historical runner: truth origin, hold 2 s, offboard, hold 1 s, arm, hold 10 s, three
   3 s truth-offset holds, the same start check (armed, offboard, |EKF alt − 1| < 0.3, truth alt in (0.3, 2.5),
   truth within 0.25 m of spawn); failure is HARNESS_INVALID. The start phase reads the newest messages, as the
   historical runner did, with a finiteness check. The bridge episode is built before the start is fixed, as in
   the historical runner. Once per process, before the first member, a scratch episode and two policy calls pay
   the first-call costs (Warp, torch); Policy F's forward has no state.
7. **Prefix.** The gate judges messages from one second before alignment on. Checkpoint k uses:
   - the first LOCAL_POSITION_NED sample with `not t < start + k*0.1`, the historical loop's own float comparison;
   - the ATTITUDE sample nearest it, within 0.05 s;
   - the latest HIL_STATE_QUATERNION received from 0.03 s before to 0.003 s after it, and before the next
     position message. PX4 v1.14.3 leaves that message's time_usec at 0 (source-pinned test), and the historical
     runner stamped it at receipt. Each MAVLink stream keeps its own 20 ms schedule, and the truth stream's phase
     relative to the position stream is fixed per boot. The truth used was sent in the position's own PX4 pass or
     4, 8, 12 or 16 ms before it, depending on that phase (review r3 probe, PX4-shaped streams). A receive delay can
     instead admit the pass 4 ms after it, switching the choice by one truth sample (20 ms). Which pass is not
     checked: the receipt age is bounded and recorded per checkpoint (`truth_recv_skew_s`); the PX4-time offset is
     not.
   Conditions: all values finite and the truth quaternion of unit length (BAD_QUATERNION); position and attitude
   time increasing; the first position sample within the skip bound of the schedule (CLOCK_SKIP) and the newest
   not later either (RUNNER_LATE); arrival within 2 s wall (CLOCK_STALL); a new truth sample at each checkpoint. The
   skip bound is the approval's `skip_max_s` (decision below). Then
   the historical state, terminal predicates (pinned historical functions) and amendment1's checkpoint_decision;
   only CONTINUE computes Policy F and sends the velocity setpoint (truth command yaw). Afterwards, the historical
   mode command, outside the scored prefix.
8. **Record.** One ledger line per member, filled as the member proceeds and kept on failure: amendment1's fields,
   parameters, start check, alignment, binding, session receipt hash and RTF. `checkpoints.jsonl` stores the
   samples used and their lateness and skews; `session.json` is in the episode directory too.
   - The recorder applies amendment1's per-record rules to every VALID record, smoke included. The chain's root
     is the attempt header's hash.
   - Every source in `tools/sitl_dev08_attempt.SOURCES` must be tracked and equal to HEAD. The list covers the
     runtime stack, the historical runner and bridge, the production modules the bridge and the Policy F player
     load, and both amendments.
   - The MOTAR-cplanner replay module is bound by hash with that tree's HEAD and state. pymavlink and rl_games,
     which live outside this tree, are recorded by path, a hash over their .py files, and version where the
     package declares one (rl_games does not).
   - Header: the platform (versions, online and offline CPUs, thread settings), the verified inputs, the warm-up
     time and the core idleness at open.
   - The member is not VALID in any of these cases: the PX4/Gazebo binding differs from the first member's; the
     source changes before or during it, the last member included; the receiver thread does not stop.
   - Each member records the core idleness measured before and after it. A record with a non-finite value is
     kept, with the value nulled, and ends the attempt.

## Differences from the historical runner (both arms share them)

- State is checked before the command; no command at a terminal or censor checkpoint (amendment1).
- ATTITUDE: the sample nearest the position sample in PX4 time. Truth: the latest received within the receipt
  window. The historical runner used the latest received of both, polling every 2 ms. On a common 50 Hz grid with
  near-simultaneous arrival the two coincide, and the CPU harness shows identity there. With a per-boot stream
  phase they can differ, always by whole truth samples (20 ms). In the same-pass phase the historical 2 ms poll
  can read the position before its truth has arrived and use the previous truth; the gate's 3 ms wait avoids
  that. In the phase whose truth is 16 ms older than the position, a receive delay can put the next truth, sent
  4 ms after the position, inside the gate's window. In the other three phases they coincide (link review r3
  probe).
- Position: the first sample at or after the checkpoint time; the historical loop used the newest at the moment
  its wait ended. They coincide while the runner is on time, and RUNNER_LATE bounds the difference to the
  approval's `skip_max_s`.
- One bridge setup per process, using the 70-bar receipt's NAVRL_* snapshot for both densities. dev07 ran one
  process per density. The keys that differ do not reach the bridge (review r1 n2).
- Telemetry or parameter faults abort the member; the historical runner had no such checks.
- Fresh rootfs and parameter readback: the historical runner reused parameters.bson across episodes, so dev07's
  effective non-overridden parameters (including the yaw-rate gains) are NOT_RECORDED.
- Prefix identity, CPU: with the real bridge and Policy F on the CPU and scripted 50 Hz streams, the new step's
  states, observations, actions and setpoints equal the historical loop's for every nonterminal checkpoint
  (one 600-checkpoint censored scene, one crash scene); the historical runner adds one command at the terminal
  step. A tolerance in the sample comparison broke this from k = 164, the harness's start sample. Over start
  samples 1500–1999, 463 of 500 diverge within 600 checkpoints, 198 of them at k = 1 or 2
  (`tolerance_divergence.out`). The gate uses the historical comparison.

## Live assumptions the smoke must confirm (NOT_VERIFIED)

1. PX4 boots in an empty rootfs (creates its `etc` link, dataman, parameters) and finds airframe 10090.
2. PARAM_SET yields one PARAM_VALUE echo with the stored float32; PARAM_REQUEST_READ by name answers.
3. A HIL_STATE_QUATERNION is received within 0.03 s before to 0.003 s after each checkpoint's LOCAL_POSITION_NED
   sample. A truth more than 0.03 s old shows as STALE_TRUTH. A truth inside the window is accepted from whatever
   PX4 pass sent it: expected the position's pass or up to 16 ms earlier, or 4 ms later under receive delay. Its
   receipt skew is recorded, and the smoke reports its distribution.
4. The three streams arrive at about 50 Hz with real-time factor about 1, so the skip/late/stall bounds hold.
5. Gazebo honours GAZEBO_LOG_PATH; gzserver and px4 stop on SIGINT within 20 s.
6. Per checkpoint, observe + policy + governor + send finish well inside 0.1 s after the warm-up (CPU measure in
   review r1: about 5 ms after the first call), so RUNNER_LATE is not triggered by the runner itself.
7. The MAVLink receiver thread runs promptly. Under a receiver stall of 30–45 ms the link review r3 probe saw
   truths 4–16 ms newer than the position accepted silently. The order bound (truth before the next position
   message) limits this but does not remove it. A stall of 3 ms or more between a position and its same-pass truth
   makes the gate use the previous truth, 20 ms older, also silently. The recorded receipt skews show both.
A smoke that violates one of these is a harness failure: the unit stops before the main phase, the attempt is
kept, and a repair needs its own review. Bounds are not widened after seeing a smoke.

## Decision for the approval: the skip/late bound

amendment1 makes a clock stall or a telemetry mismatch abort the unit. It does not fix a number. The code
default is 0.05 s, one 50 Hz period plus margin. The unit runs with the approval record's `skip_max_s`, accepted
in 0.02–0.5 s, for both CLOCK_SKIP and RUNNER_LATE.

The dev07 step logs (`/home/fair/motar_tmp/sitl_s3b_20261105/dev07_*/ep*_steps.npy`, recomputed in
`docs/evidence/sitl_2026-11-05/dev08/runtime_stack/`) show one late sample in 23 SITL episodes:
- `dev07_70/ep02`, k = 28: the position sample used was 0.136 s late, and the same sample was reused at k = 29;
- the other 22 episodes were within 0.04 s.
- The cause was on the runner side. The step log's compute column (column 21) shows 218.4 ms at k = 27, against
  about 6 ms around it (`dev07_records.json` in the evidence folder's external records). The historical report's `compute_ms_*` fields do not show this: they index column 24,
  p_true y, a bug in the frozen runner.

If 1 in 20 main episodes did this, a 42-member unit would abort with probability 1 − 0.95^42 ≈ 0.88. A wider
bound, for example 0.15 s, would have admitted that episode, but then a checkpoint may act on a state up to 0.15 s
old. Both arms share whichever bound is chosen. It is fixed before the smoke, carried in the approval record
(`skip_max_s`, refused outside 0.02–0.5 s), and not changed after seeing data. A runner-side spike is less likely
with the warm-up and idle cores below, but nothing here bounds it.

## Proposed execution (awaiting a new scoped approval)

- **Members:** amendment1's 42 in its fixed order (`tools/sitl_dev08_attempt.schedule`): smoke native env0 at 70
  then 205 bars, then 40 main members; smoke never enters the main denominator; no rerun or substitution.
- **Attempt:** `/home/fair/motar_tmp/sitl_s3b_20261105/dev08_prefix_attempt01/` (new; refused if present).
- **Approval record** consumed by `execute()`: approved_by, approved_at, scope = "dev08 observed-prefix-v1: 2 native
  smoke + 40 main episodes (amendment1 schedule)", commit = the clean HEAD that is run, skip_max_s = the chosen
  bound. The approving change adds
  the CLI entry that calls `execute()`; it is reviewed with that commit.
- **Process:** `CUDA_VISIBLE_DEVICES=`, `PYTHONNOUSERSITE=1`, `taskset -c 6,7,12,13`, `nice -n 10`, OMP/MKL
  threads 4, and `timeout -s TERM -k 90 5700`.
  - Timeout: 5400 s plus one member's 240 s plus margin. The SIGTERM runs the session cleanup (about 35 s worst
    case) before the 90 s kill.
  - The parent plan and amendment1 fix `timeout -k20`, a 20 s kill grace for the outer timeout. This amendment
    changes it to 90 s. With 20 s, `timeout` could SIGKILL the runner during the session cleanup (up to about
    35 s), leaving owned processes and no final.json. Inside the session each owned process keeps a 20 s
    SIGINT-to-SIGKILL grace (`KILL_GRACE_S`). This is a changed runtime bound, not a reading of amendment1's.
  - Idle host: no GPU run, T-LOAD's wall-clock rule.
  - The runner's cores must be at least 90 % idle in one of up to three consecutive 1 s samples (point samples,
    all recorded), when the attempt opens and before every member. Otherwise the attempt is refused at open, or
    the unit stops (HOST_NOT_IDLE). One further sample after each member is recorded and is not a stop; it adds
    up to 3 s per member. AGENTS.md's concurrent-CPU allowance does not apply to this unit,
    because RUNNER_LATE depends on wall time; review r2 measured a 25× step-time rise with another job on these
    cores.
  - Source tree frozen; kernel fault counts before and after (a rise voids the attempt).
  - After a SIGKILL, `reap_journal` on the open episode is a separate, recorded step.
- **Stops:**
  - a smoke or any member not VALID, or an invalid record;
  - 240 s per episode;
  - 2400 s per density main block and 5400 s overall, checked between members, so a run can exceed them by one
    member;
  - 12 GiB sampled RSS;
  - source change;
  - cores not idle;
  - host fault or an unreadable kernel log;
  - a termination signal or a runner exception.
  These keep the attempt and write final.json listing the unrun members. A member interrupted by a signal or
  exception is named as `interrupted_member` and is also in the unrun list, because it was not recorded.
  A ledger line with its chain update, and final.json with its fault read, are written with SIGTERM/SIGHUP/SIGINT
  held; a held signal is delivered afterwards. A refusal at open, for example an unreadable kernel log or busy
  cores, creates no attempt directory. A SIGKILL of the runner leaves no final.json; if a session was running,
  its open leak marker and journal remain.
- **Expected duration (estimate, not a measurement):** the historical waits add up to 36 s per episode (3 s boot,
  10 s settle, 22 s start phase, 1 s after), plus PX4/Gazebo start, parameter exchange and cleanup, plus the prefix
  at real-time factor about 1. The dev07 reports (`docs/evidence/sitl_2026-11-05/s3b_dev/dev07_{70,205}_s3b_report.json`)
  show members ending after 18–75 observation checks and real-time factors 0.996–1.004; a fully censored
  600-checkpoint member adds 59.9 s. The caps can stop a slow unit; they do not guarantee completion.
- **After:** `main_records` → `summarize_complete_pairs` (amendment1's analysis, unchanged), an independent
  results review, records. Descriptive development record only; no S3b acceptance, gain selection or real-flight
  claim.
