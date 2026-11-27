# REPAIR_smoke_r2: fixes for REVIEW_amend1_r1 (S5 v2 amendment 1, S4R-1 v1 amendment 1), 2026-11-10

Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, base HEAD `fd753f2`.
CPU only. No PX4, Gazebo, GPU, `--smoke`, `--launch` or network was used. The raw smoke folders were read and never
written. Nothing was staged, committed or stashed. The committed preregistrations are untouched. Implementer: Claude
Opus 5.5 (subagent). A separate reviewer re-checks this.

Changed (all uncommitted): `tools/sitl_s5_safety.py`, `tests/test_sitl_s5_safety.py`, `tools/sitl_s4r1.py`,
`tests/test_sitl_s4r1.py`, `docs/plans/prereg_sitl_s5_safety_v2_amendment1_2026-11-10.md`,
`docs/plans/prereg_sitl_s4r1_v1_amendment1_2026-11-10.md`.

## BLOCKER-1 (S5): component-0 filter, the first failure in smoke01

- **Evidence (raw, read-only):** `smoke01/ep000/session.json` has
  `exception: RuntimeError: HARNESS_INVALID no telemetry within 90 s` and cleanup `exception` at t 95.214 s.
  `ep000/logs/px4.log` shows a normal boot ("partner IP: 127.0.0.1", "Ready for takeoff!").
- **Cause:** pymavlink 2.4.50 sets only `sysid` on a heartbeat (`mavutil.py:423-426`). `target_component` is
  `param_sysid[1]`, which is 0 (`:215-221`, `:182`). v2's filter therefore dropped every PX4 message, which come from
  component 1. The finally-block TypeError then replaced this reason.
- **Fix: the pinning rule.** `autopilot_of` and `pin_autopilot` (`sitl_s5_safety.py:200-213`, `:439-479`). The
  first HEARTBEAT that meets all of these pins `link.autopilot = (system, component)`:
  - source component 1 (MAV_COMP_ID_AUTOPILOT1);
  - `autopilot` 12 (MAV_AUTOPILOT_PX4);
  - a vehicle `type`, i.e. not GCS 6, ONBOARD_CONTROLLER 18, GIMBAL 26 or ADSB 27.

  The wait is 120 s of wall time, as before. Rejected heartbeats and pymavlink's own target are recorded in
  `res["autopilot"]`.
- **Fix: receiving.** `handle_message` reads only from `link.autopilot`; before the pin it reads nothing.
- **Fix: replies.** `instrument_link` records `"autopilot"` next to the wire `"destination"`. `command_inventory`
  matches ACKs to the autopilot; live, the old destination match would have produced NOT_RECORDED.
- **Fix: recording contract.** `recording_contract` uses the pinned source.
- **Unchanged: sends.** Sends keep v2/S3b's wire address, component 0 = MAV_COMP_ID_ALL. The PX4 handlers accept it
  (source lines are in the amendment).
- **Rechecked receive path:** the type and field names in `dialects.v20.all`, and `get_src*`/`get_seq`/`get_msgbuf`.
  No other component or system assumption was found.
- **First failure kept:** `run_scenario`'s `finally` calls `_record_scenario`. If recording fails after a body
  failure, it raises `RecordingAfterError("<first>; then, while recording: <second>")` from the recording error, so
  the first failure leads `run_unit`'s abort reason. With no earlier failure, the recording error is raised
  unchanged. During a propagating BaseException, the recording error goes to stderr and the termination continues;
  this path has no test.
- **Tests:**
  - **Real mavutil.** `real_mavlink()` also imports `mavutil`, with `MAVLINK20=1` set only during the import; the
    environment, `sys.path` and `sys.modules` are restored. `px4_wire()` packs real MAVLink 2 bytes, read through
    `mavutil.mavlink_connection(file, notimestamps=True)`.
  - **(a) Positive.** Pin (1, 1) while pymavlink's target is (1, 0). Position, attitude, truth, heartbeat log and ACK
    are read, and the ACK is matched.
  - **(b) Negative.** v2's `wait_heartbeat` plus the (1, 0) filter drops all 5 PX4 messages. With a GCS heartbeat
    first, `wait_heartbeat` returns the GCS's heartbeat.
  - **Other pin tests.** No pin without an autopilot heartbeat. `autopilot_of` rejects 11 non-autopilot cases. An
    unpinned link reads nothing.
  - **(c) Fakes.** Every fake is now `target_component=0`. `FakePX4Link.m` is `RealWire`, a real mavutil connection,
    so every fake-PX4 live test pins through real mavutil.
  - **Mutation check (scratch copy).** Unmutated: 62/62 pass. Pin set to pymavlink's target: 13 fail. v2's filter
    restored: 15 fail.
  - **First-failure tests:**
    - positive: no telemetry gives HARNESS_INVALID with its trace;
    - chained: `RecordingAfterError` with the first failure first, `__cause__` set, and the `run_unit` reason in the
      same order;
    - negative: a recording error on its own is raised unchanged.

## S5 amendment text

The title is changed. The amendment now has:

- a "Revised" note;
- §1, the true first failure (receipt quote, PX4 boot);
- new §2.1, diagnosis with mavutil line numbers, the recheck and the S3b/S3c note;
- §2.2, which states that the bytes were not the sole cause;
- §3, rows for the pin, the filter, the sends, `command_inventory`, `recording_contract`, the first-failure rule and
  the tagged count, with line numbers refreshed;
- §5, the new tests;
- §6, which states that the heartbeat wait and wire addressing are unchanged.

## S4R-1 MINOR-1, L1, L2

- **MINOR-1.** §3.2 now says the 2-step bound is "set at the observed maximum (no margin)". It equals
  `INTERVAL_JOIN_NS`, but AGQ builds intervals in arrival order. A message 2 or fewer steps late can therefore still
  open a new interval in the descriptive `bar_events`/`other_events` (example: 988, 984, 980, 992). A late first
  contact can also give start > end or an onset 1 step late. These intervals are not read by the analysis or any
  validity rule. The "~1,875" is corrected to 1,876. A "Revised" note is added, and the line numbers are refreshed.
- **L2.** The code comment now says 1,876 and adds the same wording.
- **L1.** New test `test_v1_approval_is_refused_under_amendment1`: the v1 SCOPE with `ede0a14` is refused (scope and
  commit), and the new SCOPE at HEAD is accepted. The real `execution_approval.json` carries the v1 SCOPE and is
  refused; it is only read.

## MINOR-2 (both tools)

- **S5.** `unserializable_values(trace)` counts the `{"unserializable_type", "repr"}` values in the written
  `trace.json`. The scenario result gets `res["unserializable_values"]` (`count`, `types`) and, when the count is
  nonzero, `res["recording_warnings"]`. `integrity_problems` recomputes the count from the trace and flags a
  mismatch, and `reading` lists the warnings.
- **S4R-1.** `tagged_types` runs over everything `write_json`/`dump_tap` writes and over the loaded ULog extraction.
  `finish_episode` records `unserializable_values` and `recording_warnings`, and `--analyze` lists the warnings.
  These are occurrences, so a value copied into two files counts twice.
- **Both.** A nonzero count is a recorded warning, not a validity problem.
- **Tests:** a set and an opaque object give count 2 and one warning, with VALID or no recording problem, and the
  count matches the written file. A clean run gives 0 and no warning. In S5, a tampered ledger count is flagged.

## Results (aerialgym, cores 10-11, focused command)

| Suite | r1 | Now |
| --- | --- | --- |
| `test_sitl_s5_safety.py` | 54 | 62 run, OK |
| `test_sitl_s4r1.py` | 42 | 44 run, OK |

- `test_experiment_index.py` passes.
- Importing the S5 test module leaves `os.environ` unchanged and adds no `pymavlink` module.
- Full PUBLIC_CPU was not run.

## Not done or open

- PUBLIC_CPU was not run.
- The BaseException-during-recording path has no test.
- L3: the NaN in S5's trace is unchanged and disclosed.
- L4: unchanged.
- L5: the WORKLOG wording is for the next entry (WORKLOG is outside the edit scope).
- Still needed: re-review, commit with WORKLOG and the EXPERIMENTS.md entries, and a new scoped user approval before
  smoke02.
