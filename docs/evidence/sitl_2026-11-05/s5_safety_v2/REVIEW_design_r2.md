# S5 safety v2: independent design review r2 (repair of r1 AMEND_FIRST), pre-commit, pre-run

- Reviewer: Claude Opus 5.5 (fresh context, read-only), 2026-11-10.
- Object: worktree `MOTAR-camonly` (camonly/dev), untracked `tools/sitl_s5_safety.py`, `tests/test_sitl_s5_safety.py`,
  `docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md`; diffed against the pre-repair copies in the scratchpad
  `s5_repair_r1_orig/`. Inputs: `s5_review_r1/REVIEW_design_r1.md`, `s5_review_r1/REPAIR_r1.md`; PX4 v1.14.3 source
  read-only.
- Not run: PX4, Gazebo, GPU work, `--smoke/--launch/--analyze`, network. Run: the focused suite; one read-only
  `nvidia-smi --query-compute-apps` + `host_snapshot()/gpu_problems()/free_bytes()` call; a read-only pyulog scan of
  `failsafe_flags` in 3 historical S3b-formal SITL ULogs (ep032/036/056), cores 10,11, nice, timeout.

## Test run

Exact brief command -> **Ran 47 tests, OK** (rc 0, 8.3 s).

## r1 findings: verification

### B1 (ULog coverage ends at first disarm): REPAIRED, sound

- PX4: `logger.cpp:1140-1150` on the logger-observed disarm (vehicle_status.arming_state) calls
  `initialize_load_output(Postflight)` = `_next_load_print = now + 1 s` (`:1588-1597`); data keeps being written until
  `:721-727` stops the file. So the last `vehicle_local_position` sample is >= disarm + ~0.99 s (profile 131, topic at
  full rate / <= 0.1 s interval, `logged_topics.cpp:124,272`). Time base: ULog hrt µs and the trace stamp (last
  LOCAL_POSITION_NED `time_boot_ms`) are the same lockstep clock.
- The 1 Hz HEARTBEAT (armed bit from the same arming_state) gives a first disarmed heartbeat at <= disarm + 1.0 s plus
  ms-level stream jitter, and its stamp is the latest LPN sample time, i.e. <= the send time. Hence
  `hi - last_pos` <= ~0.01 s + jitter against the 0.20 s tolerance: **margin ~0.18 s; it cannot fail by ~1 s.** The
  only route to a later anchor is a lost heartbeat, which already makes the RX HEARTBEAT gap (1.6 s) INVALID, so no new
  INVALID path is created.
- Non-vacuous: armed scenarios keep `hi = t_observation_end` (test: 23.3 s of 30 s fails, 30 s passes; late start fails);
  disarming scenarios fail when the log stops before the disarm (KIL/FDS INVALID in the live-fake test); the old rule is
  shown to fail on the faithful SDLOG_MODE-1 log. Integrity recomputation passes the stored trace (T:1681).
- The flag-coverage window uses the same end; for FDS the cause window runs to `t_b + 0.7` < disarm + 1.0 s, so the log
  still contains it (it is just not gap-checked past the first disarmed heartbeat; harmless).

### M1 (live discrimination): REPAIRED (option b)

Status bullet, §2 may/may-not, §5 limit and module docstring all say NOT_TESTED, and the may-say sentence now adds the
explicit "does not show the harness would have detected its absence". Wording is correct and not over-claiming.

### M2 (cause attribution): REPAIRED, PX4 claims verified

- Flags rise as claimed (v1.14.3): OBL `offboardCheck.cpp:40-65`; RCL `rcAndDataLinkCheck.cpp:49-58`
  (COM_RC_LOSS_T); DLL `Commander.cpp:2627-2634` -> `rcAndDataLinkCheck.cpp:76`; GFH/GFV `geofenceCheck.cpp:44` and
  acted on by `failsafe.cpp:404` (GF_ACTION). The failsafe state machine reads the internal flags set by the checks, so
  the mode change never precedes the internal rise. `manual_control_signal_lost` without RC from arming does not
  trigger RCL (`failsafe.cpp:340,470`), consistent with `live_flags()` excluding it.
- `gcs_connection_lost` is true from boot (`Commander.cpp:657`) and cleared only by a MAV_TYPE_GCS heartbeat
  (`:2550-2556`); NAV_DLL_ACT > 0 blocks arming without it (`rcAndDataLinkCheck.cpp:76-95`), so in S5 it is false when
  armed. (The S3b history cannot confirm this: S3b sent no GCS heartbeat and shows it true throughout.)
- Reachability at ULog rates: `failsafe_flags` is logged per publication (`logged_topics.cpp:64`), published on a
  result change or when `now > ts + 500 ms` at the 10 Hz checks (`HealthAndArmingChecks.cpp:101-106`); most of these
  rises also add a reporter failure (modeCheck offboard, dl arming failure, geofence action), i.e. publish at once.
  **Measured** in 3 S3b-formal ULogs during armed OFFBOARD: 41-45 rows per ~22-24 s, max gap **0.540 s**, offboard
  and geofence flags 0 in every row. Worst structural published lag ~0.61 s after the internal rise; the departure
  stamp can trail the mode change by the LPN stamp lag (50 Hz, <= 0.10 s by the RX contract). So "first live rise by
  departure + 0.7 s" is reachable on a working PX4; the worst-case margin is ~0-0.06 s only when every worst case
  coincides (see l-new2).
- STATUSTEXT causes: "Pilot took over using sticks\t" (`Commander.cpp:2766`, emitted only when no failsafe is active,
  which holds in OVJ/OVR) and "Kill-switch engaged\t" (`:1534-1541`; source RC_SWITCH -> rc_switch,
  `ManualControl.cpp:231`, `Commander.cpp:1502`). STATUSTEXT is sent only while the instance sees a GCS heartbeat
  (`STATUSTEXT.hpp` `is_gcs_connected()`); S5 streams it in OVJ/OVR/KIL, so the text is expected. Substring matching
  tolerates the trailing tab. Correctly, DLL uses the flag, not "Connection to ground station lost", which is never
  transmitted once the GCS heartbeat stops.
- INVALID on missing text: sound. A text can be lost in transport while the mechanism acted; INVALID can never PASS and
  stops the unit (fail-closed). It is checked last, so a missing text plus a real FAIL reads FAIL, which is conservative.
- Fake negatives: wrong flag with correct mode sequence (OBL/GFH/GFV/RCL/DLL), RCL-as-GCS-loss, simultaneous flags,
  late rise, pre-window flag, null flags, other live flag in OVJ/OVR/KIL/FDS, missing text. Each paired with a positive.

### Minor items

- m1 RSS 8 GiB: repaired. m2 disk: estimate corrected; `MIN_FREE_BYTES` 3 GiB refusal at launch (today 5.15 GiB free
  on the attempt filesystem -> accepted). m5, m6 text: repaired.
- m3 idle host: repaired. Live check today: `gpu_compute_apps = ['736814, /usr/share/rustdesk/rustdesk, 190 MiB']` ->
  `gpu_problems() == []`, so a rustdesk-only host passes; another app, a non-zero nvidia-smi rc or a missing binary
  refuses (fail-closed). Per scenario the GPU list is re-checked after the core busy-wait; a hit gives NOT_RUN
  `HOST_BUSY` -> ABORTED, attempt consumed, reading INCOMPLETE (same semantics as busy cores; see l-new3).
- m4: mostly repaired; still untested: `RX_TIME_REVERSED`, `RX_NONFINITE`, VOID on a changed runtime file / ULog hash,
  and a recomputation-differs negative (l-new4).

## New findings

### BLOCKER
None.

### MAJOR
None.

### MINOR

- **n1 Flag gap tolerance looser than the cause lag.** `FLAG_GAP_S = 1.0` > `CAUSE_LAG_S = 0.7`. A `failsafe_flags`
  gap of 0.7-1.0 s right after the departure is accepted by the recording contract yet makes a working PX4 read VALID
  FAIL `CAUSE_NOT_SEEN` (an irreversible false FAIL_SITL) instead of INVALID. Practically unlikely (publication
  period <= ~0.6 s, measured max 0.540 s; logger dropouts are caught separately), so not a blocker. Fix (cheap,
  recommended before commit): either set `FLAG_GAP_S` to 0.65 s (measured 0.540 s + check period), or make "no flag row
  in (departure, departure + CAUSE_LAG_S]" INVALID `ULOG_FLAGS_COVERAGE`; one test each way.

### LOW

- l-new2 No boundary test for the lag: positives use rises before the departure (5.3 vs 5.6 s); add a rise at
  departure + 0.55 s (PASS) and + 0.75 s (`CAUSE_NOT_SEEN`). Document the worst-case composition (0.5 s republish +
  0.1 s check + LPN stamp lag) in §3.3.
- l-new3 After HOST_BUSY the abort reason reads "first INVALID scenario ...: the harness does not work" (pre-existing
  text, now also reachable by a GPU app); the GPU check gets no busy-wait retry. Wording only; behaviour is fail-closed.
- l-new4 m4 residue (above). Also, integrity recomputation trusts the ledger's stored `failsafe_flags` rows (the ULog
  hash is checked, but the rows are not re-parsed from it); acceptable while the ledger row hash binds them.
- r1 l1-l5 remain open and are correctly listed in prereg §10; none affects a verdict. l2 (incomplete §3.4 INVALID
  list) is the only one worth closing at commit, as text.

## Prereg/code consistency

Cause table (§3.3), new INVALID labels (§3.4), host/disk refusal (§4), limits (§5), coverage row (§9) and §10 match the
code (`live_flags`, `_cause`, `_raised` closed/open bounds, `ulog_coverage_end`, `gpu_problems`, `MIN_FREE_BYTES`,
data_contract keys). The only declared relaxation (B1) is stated in §10 as the rules require.

## Required before commit

Nothing mandatory. Recommended: n1 (+ test), l-new2 test, l2 text. Then run the full PUBLIC_CPU suite (still pending).

VERDICT: PASS_WITH_LIMITS
