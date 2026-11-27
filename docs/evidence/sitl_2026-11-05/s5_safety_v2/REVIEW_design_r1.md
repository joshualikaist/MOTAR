# S5 safety v2: independent design review r1 (pre-commit, pre-run)

- Reviewer: Claude Opus 5.5 (fresh context, read-only), 2026-11-10.
- Object: worktree `MOTAR-camonly` (branch camonly/dev, HEAD 504eb4e), untracked
  `docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md`, `tools/sitl_s5_safety.py`, `tests/test_sitl_s5_safety.py`;
  prep notes `~/motar_tmp/sitl_next_20261110/s5_prepare_r1/`; PX4 source `/home/fair/workspaces/px4_ws/PX4-Autopilot`
  (`git describe` = v1.14.3).
- Not run: PX4, Gazebo, GPU, `--smoke/--launch/--analyze`. Run: the focused test suite (below), and read-only pyulog
  scans of 36 historical SITL ULogs (s3b_formal, s3c, sitl_s3b dev08) on cores 10,11 under nice/timeout.

## Test run (item 4)

`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 10,11 timeout 900
.../aerialgym/bin/python -B -m unittest discover -s tests -p test_sitl_s5_safety.py` -> **Ran 38 tests, OK** (rc 0, 8.0 s).

What they exercise: faithful fake PX4 passes all 10 attempt + 2 smoke scenarios with positive counts
(test:838-851); a broken-PX4 negative per scheduled scenario, each VALID-and-FAILED with the exact reason
(test:853-863); harness faults INVALID, never FAIL (test:865-876); recording contract positive (>100 LPN rows) plus
missing-HIL/HB/LPN, stale-truth and missing-TX negatives (test:878-895); installed pyulog positive on a synthetic and a
real historical ULog, truncated and bad-magic negatives (test:726-746); launch refusals one by one; smoke gate positive
and negatives; reading labels. Good structure, but see B1/m4: the fake ULog hides the one deterministic live failure.

## Findings

### BLOCKER

**B1. KIL and FDS are deterministically INVALID (`ULOG_POSITION_COVERAGE`), so PASS_SITL is unreachable and the
single-use attempt aborts at index 8.**
- PX4 SITL sets `SDLOG_MODE 1` "log from boot until first disarm" (`ROMFS/px4fmu_common/init.d-posix/rcS:177-178`,
  `init.d/rc.logging:11-14` -> `logger -e`). Neither the airframe nor `V0.PARAMS` / S5 params change it.
- On disarm the logger sets `_should_stop_file_log` and stops the file at the next load print, 1 s later
  (`src/modules/logger/logger.cpp:1113-1150`, `1588-1597`, `721-727`). So the ULog's last `vehicle_local_position`
  is at about disarm + 1 s.
- KIL and FDS observation windows end at (first disarmed heartbeat) + `post_hold_s` 3.5 s
  (`tools/sitl_s5_safety.py:1048-1052` via `_reached`, `1020-1024` `done_b`; TIMING `post_hold_s` :159), and
  `t_observation_end` is taken after that (:966). `ulog_problems` requires `last_us >= t_observation_end - 0.2`
  (:632-639), i.e. disarm + >= 3.3 s. It fails by >= 2.3 s every time -> `RECORDING_CONTRACT ULOG_POSITION_COVERAGE`
  -> INVALID -> `run_unit` aborts on the first INVALID (:1320-1321) after 8 boots; reading INCOMPLETE; a new scope is
  needed. The smoke (N0/N0R/N0J, all end armed) cannot reveal it.
- The tests miss it because `synthetic_ulog()` writes every topic continuously 0.1-150 s regardless of arming
  (test:709-724, written by `FakeSession.__exit__` :773-777).
- (No historical ULog in ~/motar_tmp contains a disarm; all S3b/S3c sessions were torn down armed. This finding is by
  source reading, consistent with the prereg's own §9 description of the logger.)
- **Fix:** end the ULog position-coverage requirement at `min(t_observation_end, t_disarm_first_hb)` (or at the first
  disarmed ULog `vehicle_status` sample) and state in §9 that SDLOG_MODE 1 closes the log ~1 s after disarm; keep the
  pre-window start requirement. Add a fake ULog that ends 1 s after the fake disarm and assert KIL/FDS (and RCL/DLL,
  whose land-disarm can fall inside the window) stay VALID, plus a negative where the log ends before the event.
  (Setting SDLOG_MODE 2 by PARAM_SET would not help: the logger mode is fixed at logger start in rc.logging.)

### MAJOR

**M1. No claim-scale known-bad control.** Every broken-PX4 negative is in the fake only. The live run contains
positive injections and nulls but nothing that must FAIL on real PX4 telemetry, so the discriminating power of the
live assessment (mode decoding, event timing, persistence, spatial gates) is never shown at claim scale (the user's
gate checklist: "gate a known-bad control at claim scale"). **Fix (choose one, fix before commit):** (a) add one or two
live negatives with a per-scenario parameter override that must read FAIL with a preregistered reason, e.g. GFH-NEG
with `GF_ACTION 0` -> `NO_DEPARTURE`, RCL-NEG with `COM_RCL_EXCEPT 4` -> `NO_DEPARTURE` (reading rules must exclude
them from FAIL_SITL and require them to FAIL, else INCOMPLETE); or (b) state explicitly in §2/§5 that live
discrimination is NOT_TESTED and that the negatives are fake-only.

**M2. Cause attribution is not required.** `_expect_state` (:1195-1253) passes on the expected mode sequence alone;
the STATUSTEXT only sharpens timing (`_event_time` :1184-1192 falls back to the heartbeat). RCL and DLL have identical
expected signatures (AUTO_LOITER -> AUTO_LAND), and "Failsafe activated" (framework.cpp:211/274) is shared by both.
STREAM_GAP limits make a cross-trigger unlikely, but nothing proves the right failsafe acted. **Fix:** require the
scenario's own cause: either extract `failsafe_flags` values from the ULog (`offboard_control_signal_lost`,
`manual_control_signal_lost`, `gcs_connection_lost`, `primary_geofence_breached`, msg/FailsafeFlags.msg:33-46) and
require the flag true at or before the departure and the others false; or require the cause-specific STATUSTEXT
(GFH/GFV/OVJ/OVR/KIL/DLL "Connection to ground station lost") and record its absence as INVALID
`EVENT_TEXT_NOT_SEEN`. Add fake negatives where the right mode comes from the wrong cause.

### MINOR

- **m1 RSS cap mismatch:** prereg §3.1 says 12 GiB (prereg:39); code passes 8 GiB (:816; INTERFACE_FINAL says 8).
  Make the prereg say 8 GiB (or the code 12).
- **m2 Cost/disk estimate wrong:** "about 9 MB per boot" (prereg:164). Historical SITL ULogs are ~8.8 MB for ~42 s
  (~200 kB/s; SDLOG_PROFILE 131); S5 logs from boot for ~80-110 s (~16-22 MB). The trace keeps every decoded packet
  with payload and hex (:362-365) from the onboard link, whose default stream set is ~500-600 msg/s
  (mavlink_main.cpp onboard list), so trace.json is likely tens of MB. Expect ~40-80 MB/boot, ~0.5-1 GB for 13 boots;
  `/` has 5.2 GB free (96 %). Correct the estimate and add a free-space refusal (e.g. < 3 GB) to `launch_problems`.
- **m3 Idle host only partly enforced:** `launch_problems`/`run_unit` refuse only on runner-core idleness
  (:1348-1350, :1290-1305); GPU compute apps and load average are recorded (:323-336) but do not refuse, while the
  prereg (:158) and AGENTS.md require an idle host for real-time SITL acceptance runs, and this unit now has wall-clock
  validity bounds (m6). Refuse on non-empty `gpu_compute_apps` or host loadavg above a fixed value, or state that the
  operator check is procedural.
- **m4 Untested negatives:** no test drives `ULOG_POSITION_COVERAGE`, `ULOG_DROPOUT`, a scenario-specific
  `ULOG_TOPIC_MISSING` (e.g. OVR log without `input_rc`), `STICK_PULSE_NOT_SEEN`, `RX_POSITION_STAMP_STALE`,
  `RX_TIME_REVERSED`, `RX_NONFINITE`, RC/MANUAL RX coverage, logger-stop plan/ownership/binary mismatch, or VOID on a
  changed runtime file/ULog hash. `test_ledger_row_recomputes_and_tampering_voids` (test:908-926) asserts only VOID
  (already VOID from the shortened schedule); its positive recomputation is covered only via the smoke gate. Add one
  negative per rule, each paired with the existing positive.
- **m5 Prereg logger facts:** §9 "Local position default logger 10 Hz" (prereg:244). With SITL SDLOG_PROFILE 131
  (rcS:180) `vehicle_local_position` is logged at full rate: measured max gap 0.008 s, 0 dropouts, 0 corruption in
  36/36 historical SITL ULogs. Harmless for the 0.25 s bound but wrong; also state SDLOG_MODE 1 (see B1).
- **m6 Age bounds are wall-clock, not simulated:** `pos_age_s` and `truth_age_s` use the monotonic receive clock
  (:360, :743-747, :901-902, :764) and are checked on every packet of four types. Prereg §9 does not say so. History
  supports the bound (S3b formal 162 and S3c 80 episodes: `truth_age_max_s` max 0.019 s, RTF 0.995-1.006), but S5
  checks every packet, including the KIL/FDS ground impact. State "wall-clock" in §9 and quote these numbers as the
  basis.

### LOW

- l1 SIM_TOO_SLOW cap is `duration/0.5 + 5 s` (:914); prereg §3.1 (:54-55) omits the +5 s.
- l2 §3.4's INVALID list omits `RECORDING_CONTRACT ...`, `STICK_PULSE_NOT_SEEN`, `LOGGER_STOP_*`, `RX_THREAD_STUCK`,
  `NO_OBSERVATION_BOUNDARY`; list them.
- l3 RX rows are stamped before the state update, so an LPN row carries the previous LPN stamp (:362 vs :372).
  Harmless for gaps; document it.
- l4 Commands appear twice in `tx_calls` ("command" and its inner "send"); harmless, document it.
- l5 `recording_contract` hard-codes autopilot `[1, 1]` (:732) instead of `target_system/target_component`.

## Checked and found sound (evidence)

- Parameters/types/lines: unchanged from the r1-reviewed set; `table_problems` refuses inconsistent sets.
- PX4 behaviour: non-forced disarm in the air -> `TRANSITION_DENIED` -> `TEMPORARILY_REJECTED`
  (Commander.cpp:595-615, 885-889); forced 21196 accepted. All preregistered STATUSTEXT strings exist as
  `mavlink_log_*` (geofence.cpp:264/292, Commander.cpp:1534/2634/2766, framework.cpp:211/274). MANUAL_CONTROL echo
  scaling x1000 with joystick z=500 -> throttle 0 (mavlink_receiver.cpp:2054, streams/MANUAL_CONTROL.hpp:70-73) and
  RC throttle trim re-centred (rc_update.cpp:205-220), so the centred-echo path gate is reachable. Kill bits at
  `<< 12` match (MANUAL_CONTROL.hpp:84).
- Timing by reading: OBL 0.5 s + <=1 s HB < 3.5 s; GFH ~7.2 s + 1 s < 10.8 s (estimated overshoot ~1-2 m, true
  distance <= 20 m holds with ~1 m margin); GFV ~3 s + 1 s < 7 s; RCL 0.5 + 5 + 1 < 8.5 s; DLL worst t_ref + 3.75
  (telemetry flag, mavlink_receiver.cpp:2908-2921) + 5 + 5 + 1 < 16.75 s, earliest physical 5.0 s > 4.5 s; KIL
  disarm ~5.1 s + 1 s < 7 s; FDS < 2 s. Null windows 15 s / >= 10 HB and pre-window 5 s / >= 3 HB are reachable at
  1 Hz.
- Observation/cleanup separation: frozen sim boundary plus per-list counts (:966-972, `bounded_rows` :681-689); the
  cleanup-contamination test is now a positive/negative pair (test:748-759).
- Stop rules (first INVALID anywhere, kernel faults, tracked tree, binding, SIGTERM) and VOID/FAIL/INCOMPLETE/PASS
  ordering match prereg §3.5-3.6; a VALID FAIL does not stop the unit.
- Approval binds `scope == SCOPE` and `commit == HEAD` (sitl_s3b_formal.py:152-163); single use via the attempt
  directory; smoke must be COMPLETE at the same HEAD/source bytes. Sources: the seven OWN_SOURCES are the modules S5
  actually imports at run time; runtime binding adds parser, logger sources, pymavlink files.
- Logger stop: client `px4-logger` -> `/tmp/px4-sock-0` (sock_protocol.cpp:44-47) of the exclusively locked
  instance 0; argv/cwd/ownership checks precede it; failure is INVALID. Session wall budget (normal ~90-110 s,
  worst-case caps ~235 s) fits the 240 s session deadline, but only narrowly in the worst case.
- "May / may not be said" (§2) is appropriately narrow; add the NOT_TESTED live-discrimination limit (M1b) if M1(a)
  is not adopted.

## Required before commit

B1 fix + tests; M1 (a) or (b); M2 fix; m1, m2 (estimate + free-space refusal), m3, m5, m6 text. Re-run the focused
suite and full PUBLIC_CPU, then a short re-review of the diff.

VERDICT: AMEND_FIRST
