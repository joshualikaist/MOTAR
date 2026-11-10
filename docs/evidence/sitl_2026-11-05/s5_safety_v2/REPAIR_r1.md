# S5 safety v2: repair of design review r1 (AMEND_FIRST) — 2026-11-10

- Implementer: Claude Opus 5.5 (delegated worker), worktree `MOTAR-camonly` (branch camonly/dev, HEAD 504eb4e), all
  three unit files still untracked; nothing staged, committed or stashed. No PX4, Gazebo, GPU, network or
  `--smoke/--launch/--analyze` was run.
- Review addressed: `REVIEW_design_r1.md` (this folder). PX4 facts re-checked read-only in
  `/home/fair/workspaces/px4_ws/PX4-Autopilot` (v1.14.3).
- Pre-repair copies of the three files: scratchpad
  `/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/05cdf826-4c23-43b1-898d-a2b5834f699d/scratchpad/s5_repair_r1_orig/`
  (volatile; `diff` against them gives the repair diff).
- Focused suite (exact command of the brief, cores 10,11): **Ran 47 tests, OK** (was 38; 9 new tests, several existing
  ones extended). No test skipped: the real historical v1.14.3 SITL ULog test ran. Full PUBLIC_CPU not run (root gate).

Line numbers: `T` = `tools/sitl_s5_safety.py`, `X` = `tests/test_sitl_s5_safety.py`, `P` =
`docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md`, after the repair.

## B1 (BLOCKER) — ULog coverage ends at the first disarm

PX4 facts re-checked: SITL `rcS:177-178` `SDLOG_MODE 1`; `logger.cpp:1145-1150` sets the delayed stop on disarm,
`1588-1597` schedules it 1 s later, `721-727` closes the file.

| Change | Where |
| --- | --- |
| `SDLOG_STOP_AFTER_DISARM_S`, `ULOG_POSITION_GAP_S`, `ULOG_BOUNDARY_TOL_S` constants | T:171-174 |
| `ulog_coverage_end(res, trace)`: min(observation end, first disarmed autopilot heartbeat in [pre start, observation end]) from the bounded trace; armed scenarios keep the observation end | T:679-690 |
| `ulog_problems(..., trace)` uses it for vehicle_local_position (start bound unchanged) and for the new failsafe_flags coverage; the stale "10 Hz" comment replaced | T:704-741 |
| both call sites pass the trace (run_scenario and integrity recomputation) | T:~983, T:~1681 |
| `attempt.json` data contract records the rule | T:launch(), `data_contract` |
| prereg §9 "ULog coverage" row, §10 | P:300, P:310-330 |

Tests:
- `history_ulog()` X:882 — the fake PX4 ULog now follows SDLOG_MODE 1 (closes 1 s after the fake's first disarm; flags
  on change or every 0.5 s); `FakeSession` writes it from the fake link's history (X:~1000-1020, `run_one` X:~1062).
- `test_every_scenario_passes_against_a_faithful_px4` X:1077 — positive: all 10 attempt + 2 smoke scenarios VALID and
  nominal with the disarm-closed log.
- `test_ulog_closing_after_the_first_disarm_is_covered` X:1097 — KIL, FDS, RCL VALID+PASS; for KIL/FDS the end is the
  first disarmed heartbeat, the ULog's last position is >1 s before the observation end, and the old rule (no trace)
  reports `ULOG_POSITION_COVERAGE` on the same faithful log (the blocker reproduced); negative: a log stopping 1 s
  BEFORE the disarm makes KIL and FDS INVALID `RECORDING_CONTRACT ... ULOG_POSITION_COVERAGE`.
- `test_ulog_coverage_window` X:955 (pure) — disarming case passes at disarm+1 s and fails at disarm−1 s; old rule fails;
  armed scenario still needs the full window (fails at 23.3 s of 30 s, passes at 30 s); late start fails; sparse
  flags fail `ULOG_FLAGS_COVERAGE`.

Relaxation statement (AGENTS.md): B1 shortens the coverage window for scenarios that disarm inside the observation
window; stated in P:§10. Nothing else was relaxed.

## M1 (MAJOR) — live discrimination NOT_TESTED (option b)

No live negative scenario added; schedule, SCOPE and parameters unchanged.

| Change | Where |
| --- | --- |
| Status bullet "Live discrimination: NOT_TESTED" | P:10-11 |
| §2 May: PASS_SITL shows the configured behaviour occurred, not that the harness would detect its absence live | P:~26-30 |
| §2 May not: no claim that the live assessment discriminates broken from working | P:~36-38 |
| §5 limits bullet | P:229-231 |
| module docstring line | T:~22 |

Test: n/a (text); the fake-PX4 must-FAIL controls it refers to are the existing and new negatives below.

## M2 (MAJOR) — cause attribution

PX4 facts re-checked: field names `msg/FailsafeFlags.msg:33,37,38,46` (`offboard_control_signal_lost`,
`manual_control_signal_lost`, `gcs_connection_lost`, `primary_geofence_breached` — the review's suggested name
`primary_geofence_breached` is correct); setters `offboardCheck.cpp:40,65`, `rcAndDataLinkCheck.cpp:45,58,69,76`,
`geofenceCheck.cpp:44` (from `navigator_main.cpp:989/1051`); publication on change or every 500 ms from 10 Hz checks
(`HealthAndArmingChecks.cpp:101-106`, `Commander.cpp:1720-1724`); logged per publication (`logged_topics.cpp:64`).
`failsafe_flags` was already a required ULog topic (T:705). Cause texts: "Pilot took over using sticks"
`Commander.cpp:2766` (mavlink_log_info), "Kill-switch engaged" `Commander.cpp:1534-1541` (mavlink_log).

| Change | Where |
| --- | --- |
| `CAUSE_FLAGS`, `CAUSE_LAG_S = 0.7` (500 ms republish + 100 ms check period + margin), `FLAG_GAP_S = 1.0` | T:162-170 |
| per-scenario `cause_flag` (OBL offboard, GFH/GFV geofence, RCL manual, DLL gcs) / `cause_text` (OVJ, OVR, KIL) | T:281-337 |
| `live_flags(scen)` (manual not live without RC/joystick: true from boot) | T:347-354 |
| ULog parser script extracts failsafe_flags rows for the four fields (names passed as argv) | T:627-640, T:660 |
| `ulog_flag_rows`; `ULOG_FLAGS_FIELDS_MISSING`, `ULOG_FLAGS_COVERAGE` in the recording contract | T:693-701, T:722-741 |
| assess: INVALID `ULOG_FLAGS_NOT_RECORDED` without a pre-window flag row; FAIL `FLAG_BEFORE_INJECTION` (every scenario); FAIL `NULL_FLAG_RAISED` (N0/N0R/N0J) | T:1246-1272 |
| `_cause`: flag scenarios need the first live-flag row after the injection, by departure + 0.7 s, to be the own flag alone (`CAUSE_NOT_SEEN` / `WRONG_CAUSE`); text scenarios need no live flag up to text + 0.7 s | T:1278-1316, called T:1369-1372 |
| FDS: no live flag from the first command to the forced one + 0.7 s | T:1339-1345 |
| missing cause text → INVALID `EVENT_TEXT_NOT_SEEN` after every other criterion | T:1422-1423 |
| prereg cause-rule table (§3.3), §3.4 new INVALID labels, §5 lag limit | P:140-165, P:~172-175, P:232-234 |

Decision recorded: a positively wrong or missing cause FLAG is FAIL (the ULog is complete by the recording contract);
a missing cause STATUSTEXT is INVALID (as the review suggested), because a STATUSTEXT can be lost in transport while
the mechanism acted; it therefore can never pass.

Tests:
- Fake PX4 now publishes failsafe flags from its own state (X:615-646) with faults `wrong_cause` (a flag rising after
  arming is reported as another one, mode sequence unchanged), `flag_glitch` (GCS-lost flag for 1 s from the
  injection) and `silent_text` (no cause text).
- `test_wrong_cause_fails_and_missing_cause_text_is_invalid` X:1117 — OBL, GFH, GFV, RCL, DLL with `wrong_cause`: VALID,
  FAIL `WRONG_CAUSE ... (expected <own flag>)`, departure AUTO_LOITER (sequence right); RCL reported as GCS loss fails
  `WRONG_CAUSE gcs_connection_lost (expected manual_control_signal_lost)`; OVJ, OVR, KIL, FDS with `flag_glitch`: FAIL
  `WRONG_CAUSE gcs_connection_lost`; OVJ, OVR, KIL with `silent_text`: INVALID `EVENT_TEXT_NOT_SEEN`. Positive: the
  faithful run X:1077 asserts the own flag is the first raised (flag scenarios) and no other flag (text/FDS).
- `test_cause_attribution_rcl_vs_dll` X:475 — RCL passes with manual; FAILs with gcs (DLL's cause), both at once, none,
  a late rise, a pre-window flag; DLL passes with gcs, fails with offboard or with only the (not live) manual flag;
  no flags → INVALID `ULOG_FLAGS_NOT_RECORDED`.
- `test_null_and_fence_flags` X:506 — N0 null passes; a raised live flag fails `NULL_FLAG_RAISED`; manual-lost in N0 (no RC)
  is not live and passes; N0R manual-lost fails; GFH passes with geofence, fails with offboard.
- `test_stick_pulse_and_cause_text` X:523, `test_kill` X:434, `test_forced_disarm` X:455 — text/ACK cause positive and
  negatives.
- `test_actual_historical_ulog_parser_positive` X:923 — the four field names exist in a real v1.14.3 SITL ULog and
  >10 rows are extracted.

## Minor findings

| Finding | Change | Test |
| --- | --- | --- |
| m1 RSS cap | prereg says 8 GiB (code passes `8 * 1024**3` T:~921; INTERFACE_FINAL says 8; nothing in the code intends 12) | n/a |
| m2 disk | prereg §4.4 now ~40-80 MB/boot, ~0.5-1 GB/unit (P:207-210); `MIN_FREE_BYTES = 3 GiB` (T:175), `free_bytes` (T:372-377), launch refusal (T:1531-1537) | `test_idle_host_and_free_disk` X:1355 (refuses at 3 GiB − 1 B, accepts at 3 GiB; nearest-ancestor lookup) |
| m3 idle host | `GPU_ALLOWED_APPS` (rustdesk) T:176, `gpu_problems` T:362-369 (unreadable list refuses); launch refusal T:1530; per-scenario `HOST_BUSY` in `run_unit` T:1465-1468; prereg §4.3 and §3.4 | X:1355 (rustdesk-only accepted, another GPU process and an unknown list refused); `test_host_busy_and_binding` X:1305 (rustdesk COMPLETE; GPU process → row 0 NOT_RUN HOST_BUSY, ABORTED) |
| m4 negatives | dropout, missing topic, stick pulse, stale age, logger mismatch, each with a positive | `test_ulog_dropout_missing_topic_and_flag_fields` X:933 (real parser: 'O' record → `ULOG_DROPOUT`; OVR without `input_rc` → `ULOG_TOPIC_MISSING input_rc`, OBL unaffected; missing flag field); `test_stick_pulse_and_cause_text` X:523 (`STICK_PULSE_NOT_SEEN`); `test_recording_contract_...` X:1159 (one ATTITUDE stamp age 0.15 s → `RX_POSITION_STAMP_STALE ATTITUDE`, 0.10 s accepted, assess INVALID); `test_logger_stop_refuses_plan_ownership_and_binary_mismatch` X:1200 (host-independent binary via new `expected_binary` parameter T:878: positive, `INSTANCE_PLAN_MISMATCH`, `OWNERSHIP_MISMATCH`, `BINARY_MISMATCH`) |
| m5 logger facts | §9 row: SDLOG_MODE 1, full-rate local position (max gap 0.008 s, review's scan of 36 SITL ULogs) | n/a |
| m6 wall-clock ages | §9 row and §5 bullet: wall clock, checked per packet, basis S3b formal 162 + S3c 80 episodes truth_age_max ≤ 0.019 s, RTF 0.995-1.006 (numbers quoted from the review, not re-measured); data contract records `sample_age_clock` | n/a |

Prereg status/review record: Status bullets P:9-11 (review r1 AMEND_FIRST, repairs, 47 PASS, re-review and
PUBLIC_CPU pending); new §10 table P:310-330 with each repair and the LOW items left open.

## Not done / open

- LOW l1-l5 not repaired (listed in P:§10 for the re-review).
- Full PUBLIC_CPU suite and the re-review of this diff: root-owned gates, not run here.
- `REVIEW_r1.md`/`REPAIR_r1.md` copies into `docs/evidence/` are a commit-time task for the root.
- Note: while saving pre-repair copies I first wrote `test.orig`/`prereg.orig` at the scratchpad top level (overwriting
  any same-named files there) and then moved them into `s5_repair_r1_orig/`.
