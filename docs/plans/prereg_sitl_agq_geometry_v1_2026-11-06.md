# AGQ geometry-v1 — four-member collision-geometry diagnostic, preregistration, 2026-11-06

**Status: PROPOSED, NOT_RUN. No execution is authorized by this file.** Committing it fixes the members, the
recording, the stop rules and the reading before any measurement. A run needs the user's new scoped approval that
names this scope and the exact clean commit (AGENTS.md, *What needs the user*). No PX4, Gazebo, SITL or GPU process
was started to prepare it.

- Parent proposal: [`sitl_agq_followup_v1_2026-11-06.md`](sitl_agq_followup_v1_2026-11-06.md), section
  *Minimum geometry intervention*. Diagnosis: [`dev08/agq/DIAGNOSIS.md`](../evidence/sitl_2026-11-05/dev08/agq/DIAGNOSIS.md).
- Code: `tools/sitl_agq_geometry_diag.py` (recording, unit, reading), `tools/sitl_agq_geometry_launch.py`
  (envelope), `tools/sitl_training_geometry.py` (model packages). The dev08 runtime stack runs unchanged.
- This is a separate development lineage. It is not a dev08 rerun, amendment or acceptance unit, and it ranks no
  gains. A result here does not change S3a (NOT_EQUIVALENT), the missing 412 target states or any formal gate.

## 1. Question and prediction

Did the four extra rotor collision cylinders of the Gazebo `navrl_quad`, which the training URDF does not have,
start the yaw disturbance and altitude loss of the dev08 205/env5 native member? The existing ULogs show a modelled
rotor-footprint overlap 40 ms before a yaw rate above 5 rad/s. Actual contacts were not recorded.

Prediction, fixed now: removing the four rotor cylinders removes their contact opportunity. If the native env5
disturbance depends on those contacts, the training-box member has no vehicle–bar contact and no angular-rate
departure at that point of the path. Nothing is predicted for env2's outcome. Its candidate trace also has a later
base-box proxy, and command-tracking overshoot appears before some intersections.

## 2. Members (fixed order, run once each)

| Index | Name | Scene | PX4 gains | Collision model |
| --- | --- | --- | --- | --- |
| 0 | `00_diag_205bars_env5_native_native` | 205/env5 | dev08 native | native (byte-identical to the installed SDF) |
| 1 | `01_diag_205bars_env5_native_training_box` | 205/env5 | dev08 native | training-box (four rotor collisions removed) |
| 2 | `02_diag_205bars_env2_candidate_training_box` | 205/env2 | dev08 candidate | training-box |
| 3 | `03_diag_205bars_env2_candidate_native` | 205/env2 | dev08 candidate | native |

Pairs: env5 = (0 native model, 1 training-box), env2 = (3 native model, 2 training-box). Both env2 members use the
candidate gains because the parent proposal fixes them. The env2 pair compares geometry under candidate gains only.

Fixed inside every member, exactly as the completed dev08 unit ran them (source `d8f9bc6`):
- Frozen Policy F and the verified seed-4501 episode-0 scene inputs (`scene_specs`).
- The 600-checkpoint 205-bar prefix and the historical transform, decisions and terminal rules (`sitl_dev08_step`).
- The start phase, the 10 Hz loop and the TelemetryGate with `skip_max_s = 0.15` s (D-2026-11-05-2).
- The parameter echo/readback guard, the arm parameters from `tools/sitl_s3b_dev08.py`, and a fresh rootfs.
- The world text (ODE 100 iterations, the rangefinder model and the spawn pose), the 240 s owned-session
  watchdog, the 12 GiB sampled RSS guard and the 20 s kill grace.
- One compute thread, the one bridge setup and the warm-up.
- The smoke timing checks (real-time factor 0.95–1.05, compute max < 100 ms), applied to every member here.

The only differences from a dev08 member:
1. The spawn SDF comes from a fresh package written by `sitl_training_geometry.prepare` in the attempt
   directory. The native package's SDF is byte-identical to the installed one (SHA-256 `f9188e90…`). The
   training-box package removes exactly `rotor_0..3_collision`; every other byte, the mass, the inertia, the
   motors, the plugins and the sensors are unchanged. The plan adapter changes only the spawn file argument. A dry
   binding check found 60 identical runtime files across the four members, with only `model_sdf` differing.
2. The evaluation-only recording in section 3, including one more owned process, the contact subscriber.

## 3. Recording contract

Per member, in `<attempt>/<member>/`:
- The dev08 files, unchanged: `session.json`, `launches.jsonl`, `checkpoints.jsonl`, `logs/*.log`, the
  rootfs with its PX4 ULog, and the world.
- `logs/contacts.log`: raw output of `stdbuf -oL gz topic -u -e /gazebo/default/physics/contacts`, one
  `gazebo.msgs.Contacts` message per line.
  - Gazebo 11 publishes one message per world step (4 ms), empty or not, stamped with simulator time.
  - `stdbuf -oL` makes every complete line reach the file before a kill. In a CPU check, a C++ program writing
    `std::cout << msg << "\n"` as gz does lost all 50 lines to SIGTERM without it and kept all 50 with it.
- `recording/`:
  - `mavlink_arrivals.jsonl`: the runner's Inbox in arrival order, with type, source, fields, sequence and
    monotonic receive time;
  - `mavlink_extra.jsonl`: COMMAND_ACK and STATUSTEXT arrivals;
  - `mavlink_sends.jsonl`: every `*_send` call with its arguments and entry/return monotonic times;
  - `snapshots.jsonl`: the complete position/attitude/truth samples each checkpoint used;
  - `recording.json`: per-type arrival counts, the truth origin and the positive-control result;
  - `contacts.jsonl`: non-empty message summaries derived from the raw log.
- Ledger record additions:
  - geometry and package receipt;
  - recording manifest (bytes and SHA-256);
  - `contact_files`: SHA-256 of `logs/contacts.log` and `contacts.jsonl`;
  - `member_files`: SHA-256 of `checkpoints.jsonl` and every ULog;
  - `scene_geometry`: bar centres and halves, `p0` and `yaw0`;
  - contact audit and bar contact events with intervals;
  - recorder problems, timing problems and host idleness.

The recorded values never reach Policy F, the bridge, the gate or the controller. The recording receiver differs
from the dev08 receiver only by the extra COMMAND_ACK/STATUSTEXT branch and the type counter. On one synthetic
stream, a test shows identical Inbox contents and identical gate alignment and checkpoint output. A live stream is not
tested.

Time axis: PX4 v1.14.3 lockstep sets its clock from `HIL_SENSOR.time_usec`, which the Gazebo plugin sets to
`llround(SimTime · 1e6)`. PX4 time (`time_boot_ms`, ULog timestamps, the runner's `sim_s`) and contact times are
therefore one axis. This is a source-level fact. Section 6 gives the live check.

## 4. Positive control and recorder completeness

- **Before the first setpoint** (immediately after the parameter guard, the vehicle disarmed on the asphalt plane),
  the last 250 messages of the contact log must:
  - be parseable Contacts messages;
  - be contiguous steps (every difference 4 ms ± 1 µs);
  - include at least 125 with a contact between a `navrl_quad::` collision and `asphalt_plane::` or
    `ground_plane::`;
  - be current: the last message within 0.5 s of PX4's newest `LOCAL_POSITION_NED` time.

  Otherwise the member is HARNESS_INVALID (CONTACT_CONTROL). Silence never counts as no contact.
  - The control reads the log tail, about 0.1 s on idle cores. This delays the first hold setpoint of a disarmed
    vehicle by that much.
  - Its entry/return times and the parameter result are recorded, also when it fails.
- **During the scored loop**, each checkpoint first checks that the subscriber process still runs; an exit makes the
  member ERROR.
- **After the member**, the log must:
  - have no simulator-time gap or non-increasing time inside the reading window [gate start − 1.0 s, final
    checkpoint + 0.5 s]. Gaps outside it, during boot or the after-phase, are counted and reported only;
  - start at least 1.0 s before the gate start;
  - end at least 0.5 s after the final checkpoint.

  In addition:
  - every checkpoint must have a snapshot;
  - every velocity command must have a recorded send;
  - the receiver must have latched no error;
  - the recording files must be written.

  The raw `logs/contacts.log` and the derived `contacts.jsonl` are bound by SHA-256 in the ledger record.

  A failure is RECORDER_LOSS (or RECORDER_ORDER or RECORDER_COVERAGE) and stops the unit after the member is
  recorded.

## 5. Stop rules (complete list; no outcome-dependent retry, no reorder, no repetition)

| Reason | When |
| --- | --- |
| MEMBER_NOT_VALID | the member is not VALID: ERROR, HARNESS_INVALID (including CONTACT_CONTROL), BINDING_CHANGED, SOURCE_CHANGED |
| RECORD_INVALID | amendment1's per-record rules fail, or a non-finite value appears in the record |
| RECORDER_LOSS | any section 4 completeness failure in a member that is otherwise valid |
| TIMING_GUARD | real-time factor outside 0.95–1.05 or compute max ≥ 100 ms |
| SOURCE_CHANGED | the tree changed before a member |
| HOST_NOT_IDLE | cores 6, 7, 12 and 13 are not ≥ 90 % idle before a member |
| HOST_FAULT (VOID_HOST_FAULT) / FAULT_CHECK_ERROR | the kernel fault count rose, or the kernel log could not be read |
| BUDGET_OVERALL | more than 850 s since the attempt opened, checked between members |
| RUNNER_EXCEPTION | a termination signal or another exception; `interrupted_member` names a member that ran but was not recorded |

BINDING_CHANGED: any runtime file other than the world and the model SDF differs from member 0's, or the model SDF's
SHA-256 is not the package's. A cleanup leak, a watchdog abort (deadline, RSS) or a parameter/stream failure is an
ERROR or HARNESS_INVALID member. Refusals at open (the approval, a dirty tree, an untracked or changed source, the
environment, busy cores, an unreadable kernel log, Policy F's hash) create no attempt directory. Every attempted
member stays in the ledger, including invalid and negative ones.

## 6. Reading (fixed; descriptive, one pair per scene)

All event times are seconds after the gate start `start_sim_time_s`, on the PX4/simulator axis.
- **Bar contact:** a recorded contact between a `navrl_quad::` collision and a `bar_<i>::` collision. Contact
  messages at most two steps apart form one interval. The vehicle part is `rotor` (`rotor_<n>_collision`) or
  `base` (`base_link_inertia_collision`). Reported: the first contact after the start (part, bar, elapsed) and
  every interval.
- **Rate windows:** the intervals at or after the start in which `vehicle_angular_velocity_groundtruth` (ULog,
  multi_id 0) has any body-axis rate above 5 rad/s. Samples at most 30 ms apart join. Rate5 is the first onset.
  - If the ULog is unreadable, the member's rates are NOT_RECORDED and the pair is NOT_EVALUABLE.
  - **Rate coverage:** a window counts as observed only if the ULog samples span it. Neither may it contain a gap
    between samples longer than 50 ms nor a ULog dropout. Silence in the ULog never counts as no disturbance.
- **Bar clearance:** the runner's `box_hit` distance from the truth position to one named bar (its horizontal
  rectangle distance, infinite outside the bar's height). The scene's bar centres and halves are recorded in each
  ledger record (`scene_geometry`).
- **Z0.8:** the first checkpoint with truth height `p_true[2] < 0.8` m.
- **Err0.5:** the first checkpoint with `|p − p_true| > 0.5` m (estimate versus truth, the runner's transform).
- Also reported:
  - at the first bar contact and at Rate5 (`velocities_at`, PX4 local NED): the truth velocity at the last
    checkpoint k at or before the event, the command in effect before it (sent at k−1), and the new command sent at
    k;
  - the outcome, the crash reason and the final elapsed time.

Member evaluability (`analyze`):
- Nothing is read from an attempt that ended VOID_HOST_FAULT or FAULT_CHECK_ERROR.
- Otherwise a member is evaluable only if all of these hold:
  - it is VALID;
  - it has no record, recorder or timing problem;
  - its raw contact log, `checkpoints.jsonl`, `recording/snapshots.jsonl` and ULog still have the SHA-256
    recorded in the ledger;
  - the contact audit recomputed from that log is clean.
- The attempt is also read as void if `final.json`'s kernel fault count at the end differs from the start or could
  not be read, even with status COMPLETE.
- A pair is read only if both of its members are evaluable.

Pair reading (`pair_reading` in the code; a native-model member N and a training-box member X; times are seconds
after each member's gate start):
1. NOT_EVALUABLE in any of these cases:
   - either member is not evaluable or lacks its ULog rates;
   - N's rates do not cover [0, N's final elapsed time].
2. **Reproduction:** N's first rotor–bar contact, with bar b, starts at `t_c`, and one of N's rate windows starts in
   `[t_c, t_c + 0.5]` s.
   - Any onset counts, not only the first, but only onsets up to N's final checkpoint.
   - No reproduction if a rate window of N that began before `t_c` overlaps `[t_c − 0.2, t_c)`: the excursion was
     already in progress at the contact.
   - Not reproduced, but N has a rate onset or Z0.8 with no vehicle–bar contact in the 0.5 s before it:
     **DISTURBANCE_WITHOUT_CONTACT**. The controller/plant hypothesis stays open.
   - Not reproduced otherwise: **INCONCLUSIVE_NOT_REPRODUCED**. No replacement member is run.
3. **Comparable paths**, otherwise **INCONCLUSIVE_PATHS_DIVERGED**:
   - at every checkpoint up to `t_c − 0.2` s, X's truth position is within 0.3 m of N's;
   - X lasted until at least `t_c + 0.5` s.
4. X's window is `[t_c − 0.5, min(t_c + 1.5, X's final elapsed time)]`. It extends past N's so that a later
   encounter is not missed, and it ends with X's flight. X's rates must cover it, otherwise NOT_EVALUABLE. The
   contact audit's continuity window already covers it.
5. **Exposure**, otherwise **INCONCLUSIVE_NOT_EXPOSED**: X's smallest clearance to bar b over its exposure window
   `[t_c − 0.5, t_c + 0.5]` s is no larger than N's reference clearance to bar b at `t_c`.
   - N's reference is the smaller of two values: the clearance interpolated linearly between the checkpoints either
     side of `t_c`, and the clearance extrapolated from N's last two checkpoints at or before `t_c`. A stop or bounce
     at the contact therefore cannot enlarge it.
   - X's positions are interpolated every 5 ms over the exposure window. This does not remove all dependence on
     10 Hz sampling, but it bounds it.
   - Both candidate values for N are reported (`native_clearance_interpolated_m`, `native_clearance_extrapolated_m`).
   - The rule errs towards INCONCLUSIVE_NOT_EXPOSED, not towards a conclusive reading. If N slows down or turns away
     just before the contact, the extrapolated value undershoots its clearance while X's chords overstate X's closest
     approach (review r5 m1). In a synthetic turn-away probe at 4 m/s², 11 of 25 valid pairs were lost this way.
   - There is no tolerance beyond 1e-9 m of floating-point slack.
   - The scene geometry must be recorded and equal for both members, otherwise NOT_EVALUABLE.
   - Absence of contact counts only if X came at least as close to the bar that N's rotor touched.
6. X crashed inside its window without a rate window: **INCONCLUSIVE**.
7. In X's window:
   - the first rate window of X that overlaps it has a vehicle–bar contact of X in the 0.5 s before its onset:
     **CONTACT_PERSISTS** (the base box also hits);
   - the first overlapping rate window of X has no such contact: **DISTURBANCE_PERSISTS_WITHOUT_CONTACT**. Not
     consistent with the rotor-contact explanation for this pair; directs a no-obstacle command-replay test;
   - no rate window of X but a vehicle–bar contact of X overlaps it: **INCONCLUSIVE**;
   - neither: **CONSISTENT_WITH_ROTOR_CONTACT**.

Clock check (`takeoff_check`):
- The anchor is the takeoff: the last vehicle–floor contact before the gate start. A touchdown after a crash
  does not count.
- Take the median of the ULog `vehicle_local_position_groundtruth` z over [anchor − 2 s, anchor − 1 s].
- Find the first ULog sample after anchor − 1 s that is 1 cm higher than that median. It must fall in
  [−0.05, +0.3] s of the anchor.
- The check catches a gross offset between the contact and PX4 clocks. It does not resolve their alignment more
  finely than that window, and the old ULogs rose 0.10–0.12 s after liftoff.

If both members of a pair pass, the pair's cross-clock order is SUPPORTED. Otherwise it is LIMITED_VALIDITY, and
the pair's reading is reported as NOT_READ_CLOCK_UNVERIFIED, with the computed reading kept only as
`provisional_reading`. Contact-versus-checkpoint comparisons still use the runner's own `sim_s`.

The reading is produced by `tools/sitl_agq_geometry_diag.py --analyze <attempt> --out <new file>`, run with the
`px4` environment's Python, which has pyulog. It recomputes the contact audit from the raw log and verifies the
ledger chain.

What does not follow from any reading:
- causality beyond the pair;
- a treatment effect, since boot and noise repeatability are unverified and one pair per scene is a mechanism
  diagnostic;
- a gain ranking;
- S3b acceptance or flight readiness.

The ledger, the reading and every invalid member are reported together, with an independent fresh-context review
(Opus 5.5) before anything is called more than CONDITIONAL.

## 7. Execution envelope

```bash
cd MOTAR-camonly && python3 tools/sitl_agq_geometry_launch.py            # read-only preparation check
cd MOTAR-camonly && python3 tools/sitl_agq_geometry_launch.py --launch   # only after the scoped approval
```

The launcher runs, once:
`timeout -s TERM -k 120 1200 nice -n 10 taskset -c 6,7,12,13 <aerialgym python> -B tools/sitl_agq_geometry_diag.py --execute ...`.
- Environment: RLIMIT_AS 16 GiB; `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=1`,
  `MKL_NUM_THREADS=1` and `PYTHONPATH` set to the worktree.
- External unit: `~/motar_tmp/agq_geometry_diag_20261106/`.
  - The approval goes in `execution_approval.json`, with the keys `approved_by`, `approved_at`, `scope`,
    `commit` and `skip_max_s = 0.15`.
  - The attempt goes in `attempt01/` and the process record in `run01/`. A second launch refuses.
- The kernel fault count is read before and after. All tracked files are frozen while it runs. The host stays idle:
  real-time SITL keeps an idle host under D-2026-11-05-1.
- `--execute` refuses outside this envelope (the refusal names the envelope):
  - an attempt directory other than `attempt01/`;
  - no RLIMIT_AS, or one above 16 GiB;
  - a parent process other than `timeout`.
- Expected duration: about 4 × 75 s plus about 2 min of setup, roughly 7 min. Bounds:
  - the 850 s budget is checked between members;
  - the 1200 s outer timeout leaves room for one member's 240 s watchdog and its cleanup: 850 + 240 + 95 = 1185 s,
    plus a few seconds of post-member checks (host idleness, the contact audit, file hashes);
  - the 120 s kill margin exceeds the worst-case cleanup of about 95 s: watchdog join, a second terminate and the
    kernel-log read.

## 8. Limits and assumptions NOT_VERIFIED until a run

- The contact subscriber changes collision processing in one concrete way. While a subscriber is connected,
  `ContactManager::NewContact` creates contacts, and `ODEPhysics` (gazebo11, about lines 989–999) no longer skips
  auto-disabled bodies. A resting, disabled vehicle therefore gets contact joints with the static floor. A contact
  with the static world does not wake an ODE island, so this should not change the solution. The subscriber also
  adds per-step serialization load.
  - Both members of each pair carry the subscriber.
  - The real-time factor is checked.
  - The new native members are not identical to the old dev08 runs.
- Gazebo's contact publisher has a queue of 50. A slow subscriber would drop messages, which the continuity audit
  detects (RECORDER_LOSS) rather than prevents.
- `gz topic -e` on a live server, its startup lines and its behaviour on SIGINT/SIGTERM were not run here. Lines
  that are not messages are counted, not fatal.
- A Gazebo contact is ODE's penetration-based contact, not a physical impulse measurement. Forces are the reported
  `body_1_wrench` sums, in Gazebo's frame convention.
- Each member is one boot; physics/noise repeatability is unknown.
- A checkpoint's truth position may be up to 30 ms older than its position sample: the gate's truth-age bound.
  At about 1 m/s that is about 0.03 m in the exposure comparison, in either member (review r5 l1; magnitude
  NOT_VERIFIED).
- A rate onset caused by a floor impact shortly before N's final checkpoint counts as a disturbance like any other.
  If every earlier onset and Z0.8 had a contact before it, such an onset can read DISTURBANCE_WITHOUT_CONTACT.
  The reading reports `onset_s`, so the case is visible against N's final time.

## 9. Differences from the parent proposal, with reasons

- Outer timeout `-k 120` instead of `-k 20`. The owned session's cleanup waits up to 20 s of kill grace and then
  reaps; its worst case is about 95 s. A 20 s outer kill could arrive during that cleanup and orphan gzserver/px4,
  as amendment2 of the dev08 unit noted for the same envelope.
- Narrower than the parent's recording list, and why:
  - MAVLink arrivals: the runner's own message types are recorded in full, COMMAND_ACK/STATUSTEXT are recorded, and
    every other type is counted only. Raw packets are not recorded.
  - Contact "impulses": recorded as Gazebo's per-step `body_1_wrench` forces. Force × 4 ms is a proxy, not a
    measured impulse.
  - Actual versus requested velocity: reported at the first bar contact and at Rate5 from 10 Hz checkpoints, not
    as a continuous series. The ULogs keep the full-rate series for later description.
  - The angular-rate departure uses any body axis above 5 rad/s. The diagnosis tabulated yaw separately (`Yaw5`).
    For the old env2 native member the roll rate crossed first.
- The smoke timing checks apply to every member, because the recording adds load.
- Native members spawn from an exported native package rather than the installed path, so both arms of a pair launch
  the same way. The SDF bytes are identical.
