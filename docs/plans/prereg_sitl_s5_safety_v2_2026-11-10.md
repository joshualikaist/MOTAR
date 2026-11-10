# S5 safety v2: PX4 SITL failsafe scenarios with a phase-bounded data contract (preregistration draft, 2026-11-10; NOT_RUN)

- **Status:** NOT_RUN. User 2026-11-10 requested auditing existing data first, then planning and progressing S5 safety and Gazebo repeatability while OAK-D is unavailable. This is the S5-only, no-policy, no-camera-GPU stand-in unit. Before any live run, the fixed source and this preregistration must be committed, independently reviewed, CPU-qualified and bound by the execution receipt naming exact SCOPE and HEAD.
- **Code:** `tools/sitl_s5_safety.py`, `tests/test_sitl_s5_safety.py`. Repaired focused tests: 38 PASS. Source and recording changes from the never-executed November 8 draft are fixed in section 9. Full PUBLIC_CPU and fresh design review are pending at this draft state; the launch gate remains closed until they pass.
- **v2 design review r1 (2026-11-10, Claude Opus 5.5): AMEND_FIRST** (1 blocker, 2 major, 6 minor). Repaired before any
  commit or run (section 10): B1 ULog coverage ends at the first disarm; M1 live discrimination stated NOT_TESTED; M2
  each scenario needs its own cause evidence; m1-m6 (RSS cap 8 GiB, disk estimate and a free-disk refusal, idle-host
  refusal, new negative tests, logger and age facts). Focused tests after the repair: 47 PASS. A re-review of the repair
  diff and the full PUBLIC_CPU run are still pending; the launch gate stays closed until they pass.
- **Live discrimination: NOT_TESTED.** Every must-FAIL control (broken PX4, wrong cause, missing data) runs against the
  fake PX4 in the CPU tests only; the live schedule has positive injections and nulls but no scenario that must FAIL.
- **Plan row:** `docs/plans/sitl_validation_plan_v1.md`, S5 ("geofence, kill switch and RC override"; pass: "checklist;
  no real flight before S3b and S5 pass and the user decides"). This unit is one checklist item
  (`~/motar_tmp/s5_20261108/S5_CHECKLIST.md`), not S5 as a whole.
- "INFERRED" = drawn from code reading or arithmetic, not measured. "By reading" = traced in PX4 v1.14.3 source.

## 1. Question

With one fixed safety parameter set applied to PX4 v1.14.3 in SITL, does each failsafe that MOTAR's real-vehicle flight
would rely on act as configured when its trigger is injected over MAVLink, and stay quiet when nothing is injected?
Failsafes: geofence (horizontal, vertical), offboard (companion) loss, stick override (joystick and RC paths), RC loss,
GCS link loss, RC kill switch, GCS forced disarm.

## 2. What may and may not be said

**May (if PASS_SITL):** "In PX4 v1.14.3 SITL (Gazebo Classic 11, navrl_quad stand-in airframe 10090), with parameter set
v1, each of the ten scenarios produced its preregistered mode change or disarm within its fixed time, with its
preregistered cause evidence (section 3.3, cause rule), and nothing changed in the null windows." Per-scenario numbers may
be quoted (event, departure and arrival times at about 1 s resolution, trigger distance/height, overshoot, lockdown drop).
A PASS_SITL shows that the configured behaviour occurred in this run; it does **not** show that the live harness would
have detected its absence (live discrimination NOT_TESTED, review r1 M1).

**May not, whatever the reading:** anything about the real flight controller, its PX4 version or wiring; anything about a
real RC transmitter/receiver (SITL RC is MAVLink `RC_CHANNELS_OVERRIDE` into `input_rc`; a real receiver's own failsafe,
"no pulses" vs "hold last values", decides whether PX4 sees RC loss at all); that the real vehicle stops inside the arena
(the fence acts on PX4's global position and home; indoors that needs a set EKF origin; the braking of the 1.1 kg vehicle
is not modelled); that the kill switch is safe to use in flight; anything about Policy F, CAM-K1/K2 or MOTAR's task (no
policy runs; scripted setpoints only); "real-flight ready", "safe", "validated", or any upgrade of S5; that the live
assessment discriminates a broken failsafe from a working one (every must-FAIL control is a fake-PX4 CPU test; no live
negative scenario is scheduled, so live discrimination is NOT_TESTED).

## 3. Design

### 3.1 Stack and harness (reused)

- PX4 v1.14.3 SITL + Gazebo Classic 11, headless, `navrl_quad` / airframe `10090_gazebo-classic_navrl_quad`; world =
  PX4's `empty.world` with ODE iterations 100 and no bars (`SF.world_text`, the S3b physics). navrl_quad (0.25 kg) is a
  stand-in: the real vehicle is now known to be a 1.1 kg 5-inch quad (section 6), for which no Gazebo model exists yet.
- One owned session per scenario (`tools/sitl_owned_session.py`): fresh rootfs, exclusive instance lock, owned-only
  cleanup, 240 s deadline, 8 GiB RSS cap (the value the code passes, `rss_cap_bytes=8 * 1024**3`; v2 review r1 m1),
  runtime binding equal across VALID scenarios (else BINDING_CHANGED).
- S3b's `ParamLink` with S5's logs; S3b's take-off (spawn hold 2 s, OFFBOARD, 1 s, arm, 10 s) and start check (armed and
  OFFBOARD from the **autopilot's** last heartbeat, EKF altitude within 0.3 m of 1.0 m, true altitude 0.3-2.5 m).
- Every parameter (safety set + S3b's vehicle set) is read back; a mismatch is INVALID `PARAM_MISMATCH` **before arming**.
  INT32 values are byte-cast with `MAV_PARAM_TYPE_INT32` (`mavlink_parameters.cpp:128-134`).
- Harness outputs from the main loop: setpoints 10 Hz; GCS heartbeats (`MAV_TYPE_GCS`) 1 Hz in every scenario from the
  start (`NAV_DLL_ACT` > 0 refuses arming without a GCS link); `RC_CHANNELS_OVERRIDE` 20 Hz in the RC scenarios with the
  unmapped **channel 8 dithered 1000/1001**, so that rc_update publishes every frame (it re-publishes unchanged RC only
  every 300 ms, `rc_update.cpp:529-530`); `MANUAL_CONTROL` 20 Hz in the joystick scenarios.
- Logged against PX4 simulation time (latest `LOCAL_POSITION_NED`): every autopilot HEARTBEAT (**fixed 1 Hz** in v1.14.3,
  `mavlink_main.cpp:2143-2144`; no rate request is made), STATUSTEXT, COMMAND_ACK, RC_CHANNELS (echo of `input_rc`),
  **PX4's MANUAL_CONTROL** (msg 69 at 20 Hz: the *selected* manual setpoint and rc_update's kill-switch bits 12-13,
  `streams/MANUAL_CONTROL.hpp:60-91`), EXTENDED_SYS_STATE, EKF and true positions. Per scenario also: the real-time
  factor, ULog files and sizes, and whether the scenario ended disarmed (each armed scenario ends with AUTO_LAND and waits
  for the disarm, so its ULog closes).
- Phases after the take-off are bounded in **simulation time**; a phase that needs more than duration / 0.5 of wall time
  makes the scenario INVALID `SIM_TOO_SLOW`.
- `attempt.json` records the host-wide load average and the GPU's compute processes; each ledger row records them again
  before its scenario, with the runner-core idle sample.

### 3.2 The fixed parameter set v1

Names, types and lines checked against v1.14.3 (`test_types_and_lines_match_px4_definitions`; review r1 re-checked them).
**Every value is a user decision for the real vehicle**; these are conservative SITL stand-ins.

| Parameter | Type | v1 | Meaning / why | PX4 default |
| --- | --- | --- | --- | --- |
| `GF_ACTION` | INT32 | 2 | Hold on a breach (pilot can switch modes); Land (5) is the alternative | 2 |
| `GF_MAX_HOR_DIST` | FLOAT | 17 m | 3 m inside the 20 m half width of the 40 × 40 m arena **if home is its centre**; a polygon fence is the real answer | 0 |
| `GF_MAX_VER_DIST` | FLOAT | 2.5 m | 0.5 m under the 3 m arena height; flight altitude 1.0 m | 0 |
| `GF_SOURCE` | INT32 | 0 | estimator global position (indoors needs a set EKF origin) | 0 |
| `GF_PREDICT` | INT32 | 0 | predictive breach off (experimental) | 0 |
| `COM_OF_LOSS_T` | FLOAT | 0.5 s | 5 missed 10 Hz steps | 1.0 |
| `COM_OBL_RC_ACT` | INT32 | 5 | Hold on offboard loss; Land (4) is the alternative | 0 |
| `COM_RC_OVERRIDE` | INT32 | 3 | stick override in auto **and offboard** | 1 |
| `COM_RC_STICK_OV` | FLOAT | 30 % | override threshold | 30 |
| `COM_RC_IN_MODE` | INT32 | 2 | RC and joystick (SITL needs both paths); **the real vehicle should use 0, RC only** (not tested) | 3 (SITL rcS: 1) |
| `NAV_RCL_ACT` | INT32 | 3 | Land on RC loss | 2 |
| `COM_RCL_EXCEPT` | INT32 | 0 | RC loss also acts in offboard | 0 |
| `NAV_DLL_ACT` | INT32 | 3 | Land on GCS link loss; Hold (1) if the pilot keeps RC | 0 |
| `COM_KILL_DISARM` | FLOAT | 5 s | disarm after the kill switch; motors stop at once (lockdown) | 5 |
| `RC_MAP_KILL_SW` | INT32 | 7 | kill channel (SITL emulation; 8 is the dither channel) | 0 |
| `COM_RC_LOSS_T` | FLOAT | 0.5 s | RC loss timeout | 0.5 |
| `COM_DL_LOSS_T` | INT32 | 5 s | PX4's minimum | 10 |
| `COM_FAIL_ACT_T` | FLOAT | 5 s | Hold before a Hold/RTL/Land failsafe action (not before the offboard-loss fallback) | 5 |
| `RC_KILLSWITCH_TH` | FLOAT | 0.75 | kill switch threshold | 0.75 |
| `RC_MAP_ROLL/PITCH/THROTTLE/YAW` | INT32 | 1/2/3/4 | SITL RC emulation (AETR) | 0 |

Left at the PX4 default, not tested, still real-vehicle decisions: `CBRK_FLIGHTTERM`, `FD_FAIL_R`/`FD_FAIL_P`,
`COM_LKDOWN_TKO`, `COM_DISARM_LAND`, `MAN_ARM_GESTURE`, `COM_RC_ARM_HYST`, `RC_MAP_FLTMODE` (the safety pilot's mode
switch). `table_problems()` refuses an inconsistent set at launch. A changed value is a new SCOPE and preregistration.

### 3.3 Scenarios (fixed order; one boot each)

Common rules:
- After the start check, a **5 s pre-injection window** with nothing injected: every heartbeat in it must read armed +
  OFFBOARD (else FAIL `BEFORE_INJECTION`); at least 3 heartbeats (else INVALID).
- *t_ref* = the last message of a stopped stream (OBL, RCL, DLL), else the injection. *Earliest* is measured from t_ref,
  *deadline* from the injection.
- *Departure* = the first heartbeat after the injection that is not armed + OFFBOARD. The **event time** = the earlier of
  the departure and the first preregistered STATUSTEXT after the injection (GFH "Maximum distance from home reached",
  GFV "Maximum altitude above home reached", OVJ/OVR "Pilot took over using sticks", RCL "Failsafe activated", DLL
  "Connection to ground station lost" or "Failsafe activated", KIL "Kill-switch engaged"). Both lie at or after the true
  event, so "not too early" cannot fail on the 1 s heartbeat resolution; an early action is detectable only when it is
  earlier than this resolution (OBL has no STATUSTEXT and is resolved to about 1 s).
- The expected state must be reached by the deadline through allowed intermediate states only, and hold for
  **PERSIST_S = 2.5 s** (at least 2 heartbeats; a disarm after Land is allowed).

| Id | Injection (positive control) | Expected | Earliest | Deadline | Allowed between | Extra gate |
| --- | --- | --- | --- | --- | --- | --- |
| N0 | none, 15 s | armed + OFFBOARD throughout (≥ 10 heartbeats) | — | — | — | — |
| OBL | setpoints stopped | AUTO_LOITER | 0.2 s | 3.5 s | AUTO_LOITER | — |
| GFH | north at 2.5 m/s, z held at 1 m | AUTO_LOITER | — | 10.8 s | none | EKF distance from home at the event ≥ 16.5 m; true distance from spawn ≤ 20 m |
| GFV | climb at 0.5 m/s | AUTO_LOITER | — | 7.0 s | none | EKF height above home at the event ≥ 2.2 m; true height ≤ 3.0 m |
| OVJ | `MANUAL_CONTROL` pitch 0.5 for 0.3 s | POSCTL | — | 3.0 s | none | joystick path gate |
| OVR | RC channel 2 at 1750 µs for 0.3 s | POSCTL | — | 3.0 s | none | RC path gate |
| RCL | RC stopped | AUTO_LAND | 0.2 s | 8.5 s | AUTO_LOITER (the 5 s hold) | RC path gate |
| DLL | GCS heartbeat stopped | AUTO_LAND | 4.5 s | 16.75 s | AUTO_LOITER | — |
| KIL | RC channel 7 at 2000 µs | disarmed | — | 7.0 s | any | RC path gate; kill switch ON seen in PX4's echo after the injection; true altitude drops ≥ 0.3 m within 1.5 s, or disarmed within 1.5 s |
| FDS | `COMPONENT_ARM_DISARM` 0 without force, then after 3 s with 21196 | non-forced: still armed + OFFBOARD (≥ 2 heartbeats) and ACK `TEMPORARILY_REJECTED`; forced: ACK `ACCEPTED` and disarmed | — | 2.0 s after the forced command | — | — |

**Path gates (harness validity, review r1 M1).** RC path (OVR, RCL, KIL, smoke N0R): `RC_CHANNELS` echo of the sent
values **and** at least 10 PX4 `MANUAL_CONTROL` echoes in the pre-window with centred sticks (|x|, |y|, |z|, |r| ≤ 20 of
1000) and the kill switch OFF, which only rc_update publishes (else INVALID `RC_PATH_NOT_SELECTED`). Joystick path (OVJ,
smoke N0J): at least 10 centred echoes (else `JOYSTICK_PATH_NOT_SELECTED`). KIL: an echo with the kill switch ON after the
injection (else `KILL_SWITCH_NOT_SEEN`). So a harness that never reached rc_update or the selector is INVALID, never a
FAIL. Each injection is the positive control of its failsafe; N0, the smoke nulls and every pre-window are the null
controls.

**Cause rule (v2 review r1 M2).** The mode sequence alone never passes a scenario. Evidence comes from data the unit
already records: the ULog `failsafe_flags` fields `offboard_control_signal_lost`, `manual_control_signal_lost`,
`gcs_connection_lost`, `primary_geofence_breached` (v1.14.3 `msg/FailsafeFlags.msg:33,37,38,46`; extracted row by row by
the installed parser; a test checks the names on a real v1.14.3 SITL ULog) and the STATUSTEXT/ACK records. A flag is
*live* while its input is fed: offboard with setpoints, GCS with the heartbeat, manual control with RC or joystick, the
geofence always (`manual_control_signal_lost` is true from boot without RC or joystick, so it is not live in N0, OBL, GFH,
GFV, DLL, FDS). PX4 publishes `failsafe_flags` on a check-result change or every 500 ms from 10 Hz checks
(`HealthAndArmingChecks.cpp:101-106`, `Commander.cpp:1720-1724`), so a published flag may trail the internal flag and the
mode change by up to about 0.6 s: **CAUSE_LAG_S = 0.7 s**.

| Id | Cause evidence | Outcome if absent / wrong |
| --- | --- | --- |
| all | every live flag false in every row of the 5 s pre-window | FAIL `FLAG_BEFORE_INJECTION` |
| N0 (N0R, N0J) | every live flag false through the 15 s null window | FAIL `NULL_FLAG_RAISED` |
| OBL | first row after the injection with any live flag true lies by departure + 0.7 s and shows `offboard_control_signal_lost` alone | none by then: FAIL `CAUSE_NOT_SEEN`; another or an additional flag: FAIL `WRONG_CAUSE` |
| GFH, GFV | as OBL with `primary_geofence_breached` | as OBL |
| RCL | as OBL with `manual_control_signal_lost` (so RC loss and GCS loss are told apart) | as OBL |
| DLL | as OBL with `gcs_connection_lost` | as OBL |
| OVJ, OVR | STATUSTEXT "Pilot took over using sticks" (`Commander.cpp:2766`) from the injection to departure + 0.7 s, and no live flag true from the injection to that text + 0.7 s | a live flag: FAIL `WRONG_CAUSE`; no text: INVALID `EVENT_TEXT_NOT_SEEN` (checked after every other criterion) |
| KIL | as OVJ with "Kill-switch engaged" (`Commander.cpp:1534-1541`), in addition to the kill-ON echo gate | as OVJ |
| FDS | the two ACKs (TEMPORARILY_REJECTED, ACCEPTED) as before, and no live flag true from the first command to the forced one + 0.7 s | a live flag: FAIL `WRONG_CAUSE` |

A missing flag row in the pre-window is INVALID `ULOG_FLAGS_NOT_RECORDED`; missing flag fields or a flag gap over 1.0 s in
the ULog coverage window are INVALID (`RECORDING_CONTRACT ULOG_FLAGS_FIELDS_MISSING` / `ULOG_FLAGS_COVERAGE`).

**smoke01** runs N0, N0R (N0 with RC streamed) and N0J (N0 with the joystick streamed), one boot each. It is never read;
attempt01 refuses unless smoke01 is COMPLETE at the same HEAD and source bytes with all three VALID and nominal.

### 3.4 Validity (INVALID is a harness result)

`ERROR` / `HARNESS_INVALID` (session, link, no telemetry, start check, `PX4_TIME_STALLED`); `PARAM_MISMATCH`;
`SIM_TOO_SLOW`; `STREAM_GAP` (setpoints 0.25 s, RC 0.2 s, joystick 0.25 s, GCS heartbeat 1.6 s, before or after the
injection); too few heartbeats (pre-window < 3, persistence window < 2, null window < 10, FDS reject window < 2);
`RC_INPUT_NOT_SEEN`; the path gates; `NO_COMMAND_ACK` (FDS); `HOST_BUSY` (not run); `BINDING_CHANGED`.
Added by the v2 review r1 repair: `ULOG_FLAGS_NOT_RECORDED`, `EVENT_TEXT_NOT_SEEN` (OVJ, OVR, KIL), and
`RECORDING_CONTRACT ULOG_FLAGS_FIELDS_MISSING` / `ULOG_FLAGS_COVERAGE`; `HOST_BUSY` now also covers a GPU compute
process other than `/usr/share/rustdesk/rustdesk` (or an unreadable GPU process list) before a scenario.

### 3.5 Unit stop rules (as S3b/S3c)

ABORTED on a kernel-fault rise (Machine check, segfault, general protection), a tracked-tree or source-binding change, the first INVALID scenario at any schedule position, or SIGTERM (recorded, re-raised). No retries.

### 3.6 Reading (fixed)

1. **VOID** on any integrity failure (ledger not the schedule's prefix in order; COMPLETE with fewer rows; kernel faults
   changed; HEAD changed or tree dirty at the end; a trace missing or its SHA-256 differing; a recomputed assessment
   differing from the ledger; a development smoke);
2. else **FAIL_SITL** if any VALID scenario failed its criterion;
3. else **INCOMPLETE** if not COMPLETE or any scenario INVALID / NOT_RUN;
4. else **PASS_SITL**.
Reported as CONDITIONAL at most until an independent results review has finished.

## 4. Bounded procedure and execution receipt

1. After the PUBLIC_CPU suite passes with the test file and an independent fresh-context pre-run review (reviewer/model recorded below) has no
   open blocking finding: commit the two code files and this file, with its `docs/EXPERIMENTS.md` family entry, as their
   own unit.
2. Record the current user authorization source and the concrete reviewed scope in `~/motar_tmp/sitl_next_20261110/s5_safety_v2/execution_approval.json`, with `approved_by`, `approved_at`, `scope` exactly equal to the code's `SCOPE`, and `commit` equal to the committed HEAD. A missing receipt or a scope/HEAD mismatch refuses. No earlier session approval may be reused.
3. On an idle host (no GPU job, no SITL, no concurrent compute), from a worktree whose tracked tree is clean (the launch refuses a
   dirty tree), with no `NAVRL_*`, `PX4_*`, `S3B_*` or `GAZEBO_*` variable. The launch refuses (v2 review r1 m2, m3)
   when the runner cores are busy, when any GPU compute process other than `/usr/share/rustdesk/rustdesk` runs or the
   GPU process list is unreadable, or when the unit directory's filesystem has less than 3 GiB free; each scenario
   re-checks the cores and the GPU processes (`HOST_BUSY`). Other concurrent CPU work is excluded by the operator
   (procedural; the load average is recorded, not gated). Command:
   `CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=$PWD taskset -c 6,7,12,13
   nice -n 10 timeout -k 30 1800 /home/fair/miniconda3/envs/px4/bin/python -B tools/sitl_s5_safety.py --smoke`, then the same with `--launch` under
   `timeout -k 30 3600`, then `--analyze`. Kernel fault count before and after.
4. Cost (INFERRED): 60-110 s per boot including the final landing; 3 smoke + 10 attempt boots, about 20-25 min; about
   40-80 MB per boot (corrected in v2 review r1 m2: a ULog from boot for 80-110 s at about 200 kB/s is 16-22 MB, and
   `trace.json` keeps every decoded onboard-link packet with payload and hex, likely tens of MB), so about 0.5-1 GB for
   the 13 boots; `/` had 5.2 GB free at the review, hence the 3 GiB launch refusal.
5. Records: `smoke01/`, `attempt01/` (`attempt.json`, `episodes.jsonl`, `epNNN/trace.json` + session receipt,
   `final.json`, `reading.json`), all written once.
6. Then an independent results review, the WORKLOG entry and a local commit.

## 5. Known limits

- Every mechanism is traced in source; that it behaves the same over MAVLink in this SITL build is what the run tests.
  Settled by reading in review r1: offboard loss goes straight to Hold (no `COM_FAIL_ACT_T` hold); runtime
  `COM_RC_IN_MODE`/`RC_MAP_*` changes apply without a reboot; the default RC calibration passes; a centred throttle reads
  as zero; a GCS heartbeat on the 14540 instance counts. Still INFERRED: that rc_update accepts the MAVLink RC as the
  selected source in this build (now a validity gate, not an assumption).
- Mode-change timing is resolved to about 1 s (1 Hz heartbeat), sharpened by STATUSTEXT where PX4 prints one.
- The channel-8 dither is a SITL artefact that avoids rc_update's 300 ms re-publish of constant RC; real receivers jitter.
- Stand-in vehicle navrl_quad (0.25 kg, T/W 3.26) instead of the 1.1 kg vehicle; fence overshoot and the fall after a kill
  depend on real dynamics.
- `COM_RC_IN_MODE` 2 is a SITL necessity; the recommended real value 0 is not tested.
- n = 1 per scenario; Terminate/Return actions, a polygon fence, a real receiver's failsafe, a real GCS radio and a
  companion crash (as opposed to a stopped stream) are not tested.
- **Live discrimination NOT_TESTED** (v2 review r1 M1): the broken-PX4, wrong-cause and missing-data negatives run only
  against the fake PX4 in the CPU tests. Live negatives (for example GF_ACTION 0 or COM_RCL_EXCEPT 4 overrides) would
  change the approved schedule and are not part of this unit; a later scope may add them.
- Cause attribution (section 3.3) reads `failsafe_flags` as published, up to about 0.6 s behind PX4's internal flag. In
  OVJ/OVR/KIL/FDS a wrong-cause flag raised less than that before the cause text or command could be missed; in KIL the
  check ends 0.7 s after "Kill-switch engaged", so flags raised by the fall after the lockdown are not attributed.
- The 0.10 s position-stamp and truth-sample age limits are **wall-clock** (monotonic receive time), not simulation
  time, and are checked on every LOCAL_POSITION_NED, ATTITUDE, HIL_STATE_QUATERNION and HEARTBEAT packet in the window,
  including the KIL/FDS ground impact. Basis (measured by review r1 on historical records): S3b formal 162 and S3c 80
  episodes, `truth_age_max_s` at most 0.019 s, real-time factor 0.995-1.006.

## 6. Vehicle facts (from the coordinator's 2026-11-08 brief; the decision record is the coordinator's)

1.1 kg, 5-inch frame, propellers 5 × 4.3 × 3, T-Motor Velox 2550 KV; battery and wheelbase unknown (4S and 225 mm
assumed); T/W about 4.0 (INFERRED from LIGPOWER 2400 KV data). This supersedes the "about 0.5 kg" of D-2026-11-06-1 for
planning. It does not change this unit (navrl_quad is a stand-in), but every real-vehicle value above must be chosen for
it.

## 7. Decisions for the user

1. Approve or change the SITL values (3.2) and choose the real vehicle's values (`COM_RC_IN_MODE` 0, kill channel,
   `NAV_DLL_ACT` Hold vs Land, `COM_OBL_RC_ACT` Hold vs Land, fence circle vs polygon and the indoor EKF-origin plan, the
   untested parameters listed in 3.2).
2. The current user request names S5 progression. The execution receipt must bind this fixed three-smoke plus ten-scenario unit and the reviewed committed source. Any changed scenario, retry or wider hardware test needs a new scope.
3. Later versions: Terminate/Return scenarios, a polygon fence, a repeat, a model of the 1.1 kg vehicle.

## 8. Design review r1 (Claude Opus 5.5, 2026-11-08): AMEND_FIRST, repaired as follows

Review: `~/motar_tmp/s5_20261108/review_design_r1/REVIEW.md`. The repairs are in the code and tests; they have not been
executed and not re-reviewed.

| Finding | Repair |
| --- | --- |
| **B1** HEARTBEAT fixed at 1 Hz; the 1.0 s / 2-heartbeat persistence gate would make about half the scenarios INVALID | `PERSIST_S` 2.5 s, `post_hold_s` 3.5 s; the HEARTBEAT interval request and its INFERRED sentence removed; all assessment tests and the fake PX4 at 1 Hz with ±20 ms jitter; the null-window thresholds kept at ≥ 10 of about 15 heartbeats |
| **M1** RC path validated only by the `input_rc` echo; a harness RC defect would read as FAIL_SITL | PX4 `MANUAL_CONTROL` (msg 69) requested at 20 Hz and logged from the autopilot only; path gates for OVR/RCL/KIL/N0R (centred echoes + kill OFF) and OVJ/N0J; `KILL_SWITCH_NOT_SEEN` gate in KIL; smoke01 now N0, N0R, N0J and attempt01 requires all three VALID and nominal |
| **M2** rc_update re-publishes constant RC only every 300 ms; a 0.25 s gap limit admitted a harness-caused RC loss | channel 8 dithered 1000/1001 on every frame; RC gap limit 0.2 s; the fake PX4 models the 300 ms re-publish and times RC loss from the last published sample; limit stated in section 5 |
| **M3** fake PX4 unfaithful; RCL and DLL lacked broken-PX4 negatives; no DLL assess test | fake PX4: 1 Hz heartbeats, 300 ms re-publish, edge-triggered kill after two identical frames, MANUAL_CONTROL echo, STATUSTEXT at events; faults `rcl_ignored`, `dll_ignored`, `rc_not_selected` added (every scheduled scenario now has a broken-PX4 negative, asserted); DLL assess test (pass, too early, never, Hold only, late) |
| **M4** OAK-D note did not reproduce the training depth | `OAKD_BRIDGE_NOTE.md` section 3 rewritten (40 × 24 obstacle depth, bilinear upsample, 10 m clamp, target at full resolution up to 20 m, CAM-K2 invalid rule) with the residual mismatches as user decisions |
| m1 FDS refusal not tied to PX4's answer | ACK `TEMPORARILY_REJECTED` required for the non-forced command, `ACCEPTED` for the forced one; a missing ACK is INVALID `NO_COMMAND_ACK` |
| m2 little power at 1 Hz | event time from the preregistered STATUSTEXT where PX4 prints one (section 3.3); the spatial gates use the position at the event time |
| m3 wall-time phases, RTF not recorded | post-take-off phases end on simulation time with a wall cap (INVALID `SIM_TOO_SLOW`); real-time factor recorded per scenario |
| m4 vehicle facts stale | section 6; checklist and notes updated |
| m5 user-decision list incomplete | `COM_RC_LOSS_T`, `RC_KILLSWITCH_TH`, `GF_PREDICT` added; untested real-vehicle parameters listed (3.2, `UNTESTED_REAL_VEHICLE_PARAMS`, lines tested) |
| m6 dirty tree blocks the launch | the unit is self-contained (it no longer imports `sitl_s3c.py`) and is to be committed as its own unit; the launch must run from a clean worktree |
| m7 OAK-D FOV assumed | note: crop and undistort from the calibrated intrinsics of the exact ISP size, then anisotropic resize; checked against MOTAR's principal point |
| m8 latency inputs incomplete | `LATENCY_BENCH_DESIGN.md`: previous action derived from the previous row, yaw rate from a dump that has it or by recomputation; the bitwise equivalence claim dropped |
| l1 | HEARTBEAT request and docstring removed |
| l2 | host load and GPU compute processes recorded in `attempt.json` and per scenario |
| l3 | latency note: "was associated with" |
| l4 | DepthAI details listed as INFERRED in the note |
| l5 | each armed scenario ends with AUTO_LAND and waits for the disarm; ULog files and sizes recorded |
| l6 | start and end checks use the autopilot's component-filtered heartbeat |

## 9. Current user scope and v2 changes (2026-11-10; fixed before live measurement)

The user requested deferring OAK-D work, auditing existing data first, then planning and progressing steps 1 (S5) and 2 (Gazebo repeatability). This is the S5-only unit. It uses scripted setpoints and the historical navrl_quad stand-in, no learned policy, camera GPU rendering or real hardware. The fixed unit is three null smokes followed once by the ten-scenario schedule; it does not qualify the 1.1 kg vehicle or authorize flight.

Existing-data audit has found that hashes and usable outcome records do not ensure complete causal telemetry: two historical S4F1 ULogs have incomplete final native records after the outcome. The new protocol therefore qualifies actual log structure and coverage before trusting a scenario. All historical raw, drafts, results, checkpoint bytes and labels remain unchanged.

The initial S5 WIP CPU baseline ran 33 tests with one failure: cleanup AUTO_LAND was included in the assessment window. V2 freezes observation-end before cleanup, retains all cleanup raw records with explicit phase markers, and excludes those records from trigger-response assessment. It also fixes the first-INVALID stop behavior and adds bounded transport/clock/topic coverage checks with non-vacuous positive and broken-controller controls.

The fixed new recording contract is:

| Channel | Required data and coverage |
| --- | --- |
| Phases | Explicit setup, pre-trigger, injection, frozen observation-end, cleanup and shutdown markers. Freeze both native simulation boundary and per-list record-count boundary; cleanup modes/status/ACK/motion cannot enter assessment. |
| RX | Every decoded message with full fields, raw bytes where exposed, sequence and source; monotonic/epoch receive time and actual native timestamp or NOT_RECORDED. Only the autopilot source updates actual state. |
| TX | Every serialized send call and command arguments/destination, monotonic/epoch entry, return and errors. Requested mode, returned ACK and reported actual mode remain distinct. |
| Periodic telemetry | Request LOCAL_POSITION_NED, ATTITUDE and HIL_STATE_QUATERNION at 50 Hz; RC and MANUAL_CONTROL at 20 Hz; heartbeat stays 1 Hz. Frozen-window maximum simulated-stamp gaps: 0.10 s local/attitude/truth, 1.6 s heartbeat, 0.20 s RC, 0.25 s selected manual. At least two periodic samples; selected manual path has at least ten centered pre-echoes and an actual pulse echo for OVJ/OVR. |
| Age and streams | Latest local and sampled truth age at most 0.10 s, measured on the **wall clock** (monotonic receive time; section 5 gives the historical basis). TX gaps include window boundaries: setpoints 0.25 s, GCS heartbeat 1.6 s, RC 0.20 s, joystick 0.25 s. Intentional stopped streams are excluded from their post-trigger requirement, never filled synthetically. Nonfinite or reversed required telemetry is INVALID. |
| ULog structure | Every armed boot has a hashed, size-bound ULog, strict complete 16-byte header and all complete framed records, installed px4-env pyulog corruption=false, recorded topic schemas/counts/native ranges/max gaps and dropouts. Any logged dropout or malformed tail makes the scenario INVALID. |
| ULog coverage | Nonzero vehicle_status, failsafe_flags, actuator_armed, vehicle_local_position and vehicle_command; scenario-specific manual_control_setpoint, input_rc/manual_control_switches and FDS command ACK. SITL sets SDLOG_MODE 1 (log from boot until the first disarm; `rcS:177-178`) and closes the file about 1 s after that disarm (`logger.cpp:1145-1150, 1588-1597`). The coverage window therefore runs from the pre-window start to the observation end, or to the first disarmed autopilot heartbeat if the scenario disarms inside the window (KIL, FDS, and RCL/DLL if the land-disarm falls inside it; v2 review r1 B1); a scenario that stays armed must still be covered to the observation end. vehicle_local_position covers both window ends within 0.20 s with maximum gap 0.25 s; with SITL SDLOG_PROFILE 131 it is logged at full rate (36 historical SITL ULogs: maximum gap 0.008 s, no dropout; the earlier "10 Hz" statement was wrong, v2 review r1 m5). failsafe_flags (four cause fields present) covers the same window with maximum gap 1.0 s. Event-driven topics have no periodic cadence claim. |
| Flush | Stop and join the exact owned instance-0 logger before process teardown and hashing, require success within 15 s, bind its binary/rootfs and module-stop/parser sources. Preserve partial logs on failure, no retry. |
| Contacts | NOT_RECORDED in this unit. KIL altitude-drop remains a proxy; no physical-contact or actual motor-stop claim. |

Mandatory-data loss is harness INVALID, not a controller FAIL. Existing phase-gated criteria distinguish VALID-but-failed safety responses from invalid recording. Analysis and the smoke gate recompute recorded integrity, coverage and response results; missing records cannot pass by rejecting every test case.

Runtime source bindings include the logger binary/source, logger topic definitions, ModuleBase stop implementation, installed parser interpreter/core and live pymavlink files. Seven local source paths cover the actual executed S5 path; unused Policy F/reflection/occlusion modules are not imported as S5 execution authority.

The initial 33-test WIP had one cleanup-contamination failure; repaired tests include valid fake scenarios and ten exact broken-PX4 failures, missing/stale telemetry negatives, smoke integrity, logger ownership/stop checks, an actual old ULog positive, synthetic valid ULog and malformed/truncated negatives. New tests run on the actual root: 38 PASS. Fresh independent source/design review and full PUBLIC_CPU results will be recorded before the preregistration commit. No live measurement precedes that commit.

## 10. v2 design review r1 (Claude Opus 5.5, 2026-11-10): AMEND_FIRST, repaired as follows

Review: `~/motar_tmp/sitl_next_20261110/s5_review_r1/REVIEW_design_r1.md`; repair note with file:line and tests:
`~/motar_tmp/sitl_next_20261110/s5_review_r1/REPAIR_r1.md`. The repairs are in the code, tests and this file; none was
executed live, and the repair diff has not been re-reviewed yet. No scenario, parameter, threshold or schedule entry
changed. One check is relaxed, stated here as the repository rules require: B1 shortens the ULog coverage window of a
scenario that disarms inside its observation window, to its first disarmed heartbeat, because PX4 SITL stops logging
there by design (the old window was unreachable on a working PX4); everything else (the cause rule, the flag coverage,
the host and disk refusals) only adds ways to read FAIL or INVALID.

| Finding | Repair |
| --- | --- |
| **B1** SDLOG_MODE 1 closes the ULog ~1 s after the first disarm, so KIL/FDS were deterministically INVALID `ULOG_POSITION_COVERAGE` | the coverage window ends at the first disarmed autopilot heartbeat inside the observation window (`ulog_coverage_end`); armed scenarios keep the full window; section 9 states the logger facts. Fake PX4 ULog now follows SDLOG_MODE 1 (closes 1 s after the fake disarm): all 12 scenarios VALID and nominal; a ULog stopping 1 s before the disarm is INVALID for KIL and FDS; the old rule is shown to fail on the faithful log |
| **M1** no claim-scale known-bad control | option (b): live discrimination NOT_TESTED stated in the status, section 2 (may and may not) and section 5; no live negative added (the schedule is unchanged) |
| **M2** cause not required; RCL and DLL indistinguishable | cause rule (section 3.3) from ULog `failsafe_flags` and the cause STATUSTEXT/ACKs; fake negatives with the right mode sequence and the wrong cause flag FAIL `WRONG_CAUSE` for OBL, GFH, GFV, RCL (RC loss reported as GCS loss), DLL, and with another live flag for OVJ, OVR, KIL, FDS; a missing cause text is INVALID `EVENT_TEXT_NOT_SEEN`; pure tests for RCL vs DLL, no-rise, late rise, simultaneous rise, pre-window and null flags |
| m1 RSS cap | section 3.1 says 8 GiB, the value the code passes and the prepared interface names |
| m2 disk estimate | about 40-80 MB per boot, 0.5-1 GB per unit (section 4); the launch refuses below 3 GiB free on the unit directory's filesystem |
| m3 idle host | the launch refuses a GPU compute process other than `/usr/share/rustdesk/rustdesk` (or an unreadable list) besides busy runner cores; each scenario re-checks both (`HOST_BUSY`) |
| m4 untested negatives | new tests with positive controls: ULog dropout, scenario-specific missing topic (OVR without `input_rc`; OBL unaffected), missing flag fields, flag gap, position coverage (start and end), stick pulse, cause text, stale wall-clock stamp age, logger-stop plan/ownership/binary mismatch, GPU/disk refusals |
| m5 logger facts | section 9: SDLOG_MODE 1 and full-rate local position (max gap 0.008 s in 36 historical SITL ULogs) |
| m6 age bounds | section 9 and section 5: wall-clock, with the historical basis |

Not repaired (LOW, left for the re-review to decide): l1 SIM_TOO_SLOW wall cap `duration / 0.5 + 5 s` (section 3.1
omits the +5 s); l2 section 3.4 still omits some INVALID labels (`RECORDING_CONTRACT ...`, `STICK_PULSE_NOT_SEEN`,
`LOGGER_STOP_*`, `RX_THREAD_STUCK`, `NO_OBSERVATION_BOUNDARY`); l3 RX rows carry the previous LPN stamp; l4 commands
appear twice in `tx_calls`; l5 `recording_contract` hard-codes autopilot `[1, 1]`.

## 11. v2 design review r2 (Claude Opus 5.5, focused, 2026-11-10): PASS_WITH_LIMITS

- Every r1 repair was verified, and nothing must be fixed before the commit.
- **B1:** coverage ending at the first disarmed heartbeat leaves about 0.18 s of margin against PX4's 1 s logger tail.
  Armed scenarios still need the full window.
- **M2:** each scenario's cause flag rises as claimed in the PX4 v1.14.3 source. Historical `failsafe_flags` rows are
  at most 0.540 s apart, so the 0.7 s cause window is reachable.
- **Idle-host check:** a host with only RustDesk passes, checked on the live machine.
- **Open, carried as known limits (not repaired at this commit):**
  - **MINOR n1:** the allowed `failsafe_flags` gap (1.0 s) is wider than the cause window (0.7 s). A 0.7–1.0 s
    logging gap right after the departure would make a working PX4 read FAIL `CAUSE_NOT_SEEN` instead of INVALID.
    This is unlikely, given historical gaps of at most 0.54 s. If it happens, the FAIL is reported with this caveat
    and not relabelled.
  - **LOW:** no test at the 0.7 s boundary; a misleading HOST_BUSY abort message; untested reversed or non-finite RX
    rows and VOID on a changed file hash; r1's l1–l5 above.
- Review files: `docs/evidence/sitl_2026-11-05/s5_safety_v2/`.
