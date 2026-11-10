# S5 safety v2, amendment 1: the autopilot pinned from its heartbeat, the first failure kept, lossless bytes, a gateable exit status and a new smoke folder (2026-11-10; NOT_RUN)

- **Amends:** `docs/plans/prereg_sitl_s5_safety_v2_2026-11-10.md` (committed at `ede0a14`, immutable). That
  preregistration stays in force in every point this file does not change.
- **Date:** 2026-11-10, written after the development smoke `smoke01` ABORTED and before any rerun.
- **Status:** NOT_RUN. Source and tests repaired on CPU only; no PX4, Gazebo, GPU, `--smoke` or `--launch` was run for
  this amendment. It needs an independent review, a commit, PUBLIC_CPU and a new scoped approval naming the new
  `SCOPE` and HEAD before any live run.
- **Revised** (still uncommitted, before any run) after the independent review `REVIEW_amend1_r1` (Claude Opus 5.5,
  AMEND_FIRST): the first draft named the bytearray `TypeError` as smoke01's cause. It was not the first failure. The
  review's BLOCKER-1 (§1, §2.1, the autopilot pin and the first-failure rule in §3), MINOR-2 (tagged-repr count) and the
  corresponding tests (§5) are added here.

## 1. Reason: smoke01 ABORTED

Recorded in `~/motar_tmp/sitl_next_20261110/s5_safety_v2/smoke01/final.json` (raw evidence, unchanged) and
`~/motar_tmp/sitl_runs_20261110/s5_smoke.log`, at HEAD `ede0a14`:

```
"status": "ABORTED", "reason": "TypeError: Object of type bytearray is not JSON serializable",
"scenarios_recorded": 0, "scenarios_scheduled": 3, "kernel_faults_before": 0, "kernel_faults_after": 0
```

**That reason is the second failure, not the first.** The first scenario's owned-session receipt,
`smoke01/ep000/session.json`, records the exception that left the scenario body:

```
"exception": "RuntimeError: HARNESS_INVALID no telemetry within 90 s"
cleanup: reason "exception", SIGINT to px4 and gzserver at t_s 95.214; exit codes gzserver 0, spawn_00 0, px4 0
```

PX4 had booted normally (`ep000/logs/px4.log`: "partner IP: 127.0.0.1", "Ready for takeoff!"), and the harness had
received a heartbeat (`wait_heartbeat` returned, so the body went on to wait for telemetry). The body raised the
`RuntimeError`, `run_scenario`'s `except` recorded it in the scenario result (outcome HARNESS_INVALID), and then the
`finally` block's `json.dumps(trace)` raised the `TypeError`, which replaced the result and became the unit's abort
reason. The first draft of this amendment and `REPAIR_smoke_r1.md` named only the `TypeError`; the review found the
receipt. Both failures are repaired here (§2.1, §2.2), and a later failure can no longer hide the first (§3).

- The chain log `~/motar_tmp/sitl_runs_20261110/chain.log` has `S5_smoke rc=0`: the tool exited 0 on ABORTED, so an
  outer chain could not gate on its exit status. `--launch` then refused ("no development smoke record in
  .../smoke01", `rc=1`) and `--analyze` crashed on the missing `attempt01/` (`rc=1`). No result exists.
- `ep000/` holds no `trace.json`: the `TypeError` was raised by `json.dumps(trace)` in `run_scenario`'s `finally`
  (v2 line 977), before the file was opened, and `run_unit` turned it into the unit's ABORTED.

## 2. Diagnosis

### 2.1 First failure: every PX4 message was dropped (component-0 filter)

- v2's `handle_message` (v2 line 432) read a message only if its source was
  `[link.m.target_system, link.m.target_component]`; every other message was kept in `rx_packets` and otherwise
  ignored. v2 assumed `wait_heartbeat()` sets both (its test fakes hard-coded `target_component=1`).
- Installed pymavlink 2.4.50 (px4 env, `mavutil.py`): `wait_heartbeat()` is `recv_match(type='HEARTBEAT')` (:554-556);
  `post_message` locks only `self.sysid` on the first vehicle-like heartbeat (:423-426); `target_system` is `sysid`
  and `target_component` is `param_sysid[1]` (:215-221), initialised to 0 (:182) and never set from a heartbeat. After
  PX4's heartbeat the target is therefore `(1, 0)`, while PX4 sends from component 1 (MAV_COMP_ID_AUTOPILOT1).
- Consequence: `pos`, `att`, `truth` and `hb_log` never filled, PX4's own heartbeats were counted as
  `other_component_hb`, and `_fly` raised "HARNESS_INVALID no telemetry within 90 s" after `boot_wait_s`. Reproduced
  offline with a real `mavutil` connection over real MAVLink 2 bytes (§5): target `(1, 0)`, every PX4 message dropped.
- Also measured: `wait_heartbeat()` returns the first HEARTBEAT of any source (a GCS or a camera component, if one
  came first), so it is not a statement about the autopilot either.
- S3b/S3c, which use the same link, read messages without a component filter, so they were unaffected.
- The rest of the S5 receive path was rechecked against pymavlink 2.4.50: the message type names
  (`LOCAL_POSITION_NED`, `HIL_STATE_QUATERNION`, `HEARTBEAT`, `ATTITUDE`, `PARAM_VALUE`, `STATUSTEXT`, `COMMAND_ACK`,
  `RC_CHANNELS`, `MANUAL_CONTROL`, `EXTENDED_SYS_STATE`) and fields exist in `dialects.v20.all`; the remaining
  source-dependent places were `command_inventory` (ACKs and reported state matched to the wire destination `[1, 0]`,
  so a live ACK would have been NOT_RECORDED) and `recording_contract` (constant `[1, 1]`); both now use the pinned
  autopilot. `get_srcSystem`/`get_srcComponent`/`get_seq`/`get_msgbuf` exist on regular and BAD_DATA/UNKNOWN messages.

### 2.2 Second failure: bytearray in the trace (`TypeError`)

- v2's `json_safe` (v2 line 744) converted `bytes` only. A `bytearray` in a received payload passed through
  unchanged, and `trace.json` was written with a plain `json.dumps` (v2 line 977).
- Which pymavlink objects carry a bytearray, measured against the installed pymavlink 2.4.50 (`pymavlink.dialects.v20.all`,
  the module the live link decodes with; px4 env, also reached by the aerialgym env through
  `~/motar_tmp/sitl_plan_20261104/pylink`): every message class of the dialect was packed and decoded once, and no
  regular field of `to_dict()` is a bytearray (char arrays are decoded to `str`, uint8 arrays are lists). The two
  exceptions are the robust-parsing pseudo-messages that `recv_match` also returns: `BAD_DATA` (a CRC or framing
  failure) and `UNKNOWN_<msgid>` (an id not in the dialect). Both have `data` as a `bytearray`. `get_msgbuf()` is a
  bytearray too, but v2 already wrote it as `bytes(...).hex()`.
- Which of the two arrived in smoke01 is NOT_RECORDED: the trace was never written. The fix covers both. The bytes
  were not the cause of the abort on their own: without §2.1 the scenario was already HARNESS_INVALID.

## 3. Changes (line numbers are in the amended files)

| File:line | Change | Why |
| --- | --- | --- |
| `tools/sitl_s5_safety.py:89` | `AMENDMENT` = this file's path; added to `OWN_SOURCES` (`:99-101`), so its bytes are bound by the launch receipt | the amendment is part of the executed protocol |
| `tools/sitl_s5_safety.py:90-93` | `SCOPE` changed (verbatim in §4) | it names the new smoke folder and the amendment |
| `tools/sitl_s5_safety.py:96` | `SMOKE_DIR` = `.../s5_safety_v2/smoke02` (was `smoke01`); `ATTEMPT_DIR` stays `attempt01`, which was never created | smoke01 is spent evidence; a rerun writes to a new folder |
| `tools/sitl_s5_safety.py:200-213` | `MAV_COMP_ID_AUTOPILOT1 = 1`, `MAV_AUTOPILOT_PX4 = 12`, `NON_VEHICLE_MAV_TYPES` (GCS 6, ONBOARD_CONTROLLER 18, GIMBAL 26, ADSB 27), `HEARTBEAT_WAIT_S = 120` (v2's `wait_heartbeat(timeout=120)`), with the rule in a comment | §2.1 |
| `tools/sitl_s5_safety.py:439-479` | `autopilot_of(msg)`: a HEARTBEAT pins `(system, component)` only if its source component is 1, its `autopilot` is MAV_AUTOPILOT_PX4 (PX4 v1.14.3 sends it, `streams/HEARTBEAT.hpp:134`) and its `type` is a vehicle; otherwise None. `pin_autopilot(link, timeout_s)` replaces `wait_heartbeat`: it reads HEARTBEATs (wall time, at most `HEARTBEAT_WAIT_S`) until `autopilot_of` accepts one, sets `link.autopilot` and records `link.autopilot_pin` (source, type, autopilot, rule, the rejected heartbeats, and pymavlink's own target for comparison) | the autopilot is decided from what it sends, never from pymavlink's `target_component` |
| `tools/sitl_s5_safety.py:426-430, 482-530` | `handle_message` reads a message only if its source is `link.autopilot`; before the pin nothing is read (fail closed); every message is still kept in `rx_packets`; `other_component_hb` counts heartbeats of the autopilot's system from other components. `init_logs` keeps a pinned `link.autopilot` | §2.1 |
| `tools/sitl_s5_safety.py:1061-1066` | `run_scenario` calls `pin_autopilot(L, HEARTBEAT_WAIT_S)` and records `res["autopilot"]`; no pin is an `ERROR` ("no autopilot heartbeat ...") | — |
| sends (unchanged) | the wire addressing stays v2's: `target_system`/`target_component` as pymavlink holds them, i.e. component 0 = MAV_COMP_ID_ALL, which every PX4 v1.14.3 handler the harness uses accepts (`mavlink_receiver.cpp:382-383` COMMAND_LONG, `:951` SET_POSITION_TARGET_LOCAL_NED, `:1959` RC_CHANNELS_OVERRIDE and `:2046` MANUAL_CONTROL check the system only, `mavlink_parameters.cpp:103` PARAM_SET, `:176` PARAM_REQUEST_READ), as S3b and S3c sent | no change on the wire |
| `tools/sitl_s5_safety.py:861-869` | `instrument_link` records `"autopilot"` next to the wire `"destination"` of every send | replies can be matched |
| `tools/sitl_s5_safety.py:906-933` | `command_inventory` matches ACKs and reported heartbeats to the call's `"autopilot"` (v2: its `"destination"`, `[1, 0]` live) | §2.1 |
| `tools/sitl_s5_safety.py:936-948` | `autopilot_source(res)`; `recording_contract` counts the pinned autopilot's packets (`[1, 1]`, v2's constant, for a record without a pin) | §2.1 |
| `tools/sitl_s5_safety.py:1085-1112` | `run_scenario`'s `finally` calls `_record_scenario`; if recording raises after the body already failed, `RecordingAfterError` is raised with the first failure at the head of its message ("<first>; then, while recording: <second>", `first`/`second` attributes, `__cause__` the recording exception), so `run_unit` records the first failure in the abort reason; a recording failure with no earlier failure is raised as itself; during a propagating termination (SIGTERM, KeyboardInterrupt) the recording failure is printed to stderr and the termination continues | smoke01's reason hid its first failure |
| `tools/sitl_s5_safety.py:1115-1124, 1138-1142, 1860-1863, 1894` | review r1 MINOR-2: `unserializable_values(trace)` counts the `{"unserializable_type", "repr"}` values in the written `trace.json`; `res["unserializable_values"]` (`count`, `types`) and, when nonzero, `res["recording_warnings"]` (`UNSERIALIZABLE_VALUES n (types): kept as tagged repr in trace.json`); `integrity_problems` recomputes the count from the trace ("unserializable-value count of scenario X differs"); `reading` lists the warnings per scenario | the tagged fallback no longer fails open silently; a warning, not a validity problem |
| `tools/sitl_s5_safety.py:818-843` | `BYTES_LIKE`, `_bytes_hex`, `_is_numpy`; `json_safe` maps `bytes`, `bytearray` and `memoryview` to `{"bytes_hex": "<hex>"}` (the tag v2 already used for `bytes`) and numpy scalars/arrays to Python values | lossless: the hex string is every byte; the Python container type (bytes vs bytearray) is not kept |
| `tools/sitl_s5_safety.py:845-858` | `json_default` (bytes-like as hex, numpy as Python values, anything else as `{"unserializable_type", "repr"}`) and `trace_text(trace)` = `json.dumps(trace, default=json_default)` | a value json cannot encode no longer aborts the unit; NaN/inf floats are written exactly as v2 wrote them |
| `tools/sitl_s5_safety.py:1136-1137` | `trace.json` is written with `trace_text(trace)` | the failing write |
| `tools/sitl_s5_safety.py:1766` | `attempt.json` records `"amendment": AMENDMENT` | binding |
| `tools/sitl_s5_safety.py:1836-1838` | `integrity_problems` also requires `attempt["amendment"] == AMENDMENT` (problem text "attempt protocol/scope/prereg/amendment binding differs") | a v2-bound smoke or attempt cannot be read under this amendment; stricter, not weaker |
| `tools/sitl_s5_safety.py:1742-1813` | `launch` returns an exit status: `exit_code(final, gate)` is 0 only for COMPLETE (and, for `--smoke`, `smoke_problems` of the new smoke = []), 3 when not COMPLETE, 4 for a COMPLETE smoke that the attempt's gate refuses; the gate problems are printed | smoke01 exited 0 on ABORTED |
| `tools/sitl_s5_safety.py:1923-1943` | `main` returns that status; `__main__` is `sys.exit(main())`. A refusal stays `SystemExit("refused: ...")`, exit status 1 | an outer chain can gate on `rc != 0` |
| docstring `:24-28`, `:54-63`, ``:1706`` | text naming smoke02 and amendment 1 (both failures) | — |
| `tests/test_sitl_s5_safety.py` | see §5 | — |

## 4. SCOPE

The execution approval must quote the new `SCOPE` exactly. v2's `SCOPE` (at `ede0a14`):

```
S5 safety v2: PX4 v1.14.3 SITL on navrl_quad (stand-in vehicle), fixed safety parameter set v1, scenarios N0 OBL GFH GFV OVJ OVR RCL DLL KIL FDS once each in this order, one fresh-rootfs boot per scenario; development smoke01 (nulls N0 N0R N0J, read for harness validity only) then attempt01; no retries; SITL only, no claim about real hardware
```

`SCOPE` under this amendment (a test checks that both strings appear here verbatim and that they differ only by
"amendment1" and the smoke folder):

```
S5 safety v2 amendment1: PX4 v1.14.3 SITL on navrl_quad (stand-in vehicle), fixed safety parameter set v1, scenarios N0 OBL GFH GFV OVJ OVR RCL DLL KIL FDS once each in this order, one fresh-rootfs boot per scenario; development smoke02 (nulls N0 N0R N0J, read for harness validity only) then attempt01; no retries; SITL only, no claim about real hardware
```

The existing `execution_approval.json` names v2's `SCOPE` and HEAD `ede0a14`; it is refused (scope and commit), and
the tests show the refusal.

## 5. Tests added or changed (`tests/test_sitl_s5_safety.py`)

Autopilot pin and first failure (review r1 BLOCKER-1):

- `real_mavlink()` now also imports pymavlink's `mavutil` (with `MAVLINK20=1` set during the import only, so it starts
  on the MAVLink 2 'all' dialect and never writes the environment again; `os.environ`, `sys.path` and `sys.modules` are
  restored). `px4_wire()` packs real MAVLink 2 bytes: optionally a GCS heartbeat (255, 190) and a camera-component
  heartbeat (1, 100), PX4's heartbeat (1, 1, quadrotor, MAV_AUTOPILOT_PX4), and from (1, 1) LOCAL_POSITION_NED,
  ATTITUDE, HIL_STATE_QUATERNION, COMMAND_ACK and a second heartbeat. `wire_connection()` reads them through
  `mavutil.mavlink_connection(file, notimestamps=True)`, the same `recv_msg`/`post_message` path as the UDP link.
- `TestAutopilotPinRealMavutil.test_negative_control_v2_component_zero_drops_every_px4_message`: PX4 alone on the
  link (smoke01's case); v2's `wait_heartbeat` returns PX4's heartbeat and pymavlink's target is `(1, 0)`; with that
  filter all five later PX4 messages are recorded raw and none is read (`pos`, `att`, `truth`, `hb`, `hb_log`, `acks`
  empty; PX4's heartbeat counted as another component's). The same bytes with `pin_autopilot` are read. With a GCS
  heartbeat first, `wait_heartbeat` returns the GCS's.
- `test_pin_from_real_mavutil_accepts_px4_messages`: before the pin nothing is read; `pin_autopilot` pins `(1, 1)`
  while pymavlink's target is `(1, 0)`, rejecting the GCS and camera heartbeats; position, attitude, truth, heartbeat
  log and ACK are read; the camera heartbeat after the pin is counted as another component's; a command records
  destination `[1, 0]` and autopilot `[1, 1]`, and `command_inventory` finds its ACK and reported mode; negative
  control: matched to the destination (v2) it is NOT_RECORDED.
- `test_no_autopilot_heartbeat_pins_nothing`: only GCS/camera heartbeats -> no pin within the timeout, both rejected.
- `TestAutopilotRule` (no pymavlink needed): `autopilot_of` accepts (1, 1) and (2, 1) and refuses component 0, 100 and
  154, autopilot 8 and 3, type 6, 18, 26 and 27, a non-heartbeat and None; an unpinned link reads nothing.
- Every fake link now has pymavlink's real addressing: `stub()` and the `Streams` fake have `target_component=0`, and
  `FakePX4Link`'s connection is `RealWire`, a real `mavutil` connection over `px4_wire()` bytes (a fake with
  `target_component=0` only where pymavlink is absent). So every fake-PX4 live-path test, including the faithful-PX4
  positives, pins the autopilot through real `mavutil`. Mutation check on a scratch copy (not kept): the unmutated
  copy passed 62 of 62; the pin set to pymavlink's target (as v2) failed 13, restoring v2's filter in
  `handle_message` failed 15, the whole fake-PX4 live path among them each time.
- `TestLivePath.test_live_path_pins_the_autopilot_through_real_mavutil`: N0 runs to COMPLETED/VALID/passed;
  `res["autopilot"]` is `[1, 1]` with pymavlink's `[1, 0]` and the two rejected heartbeats; every send records
  destination `[1, 0]` and autopilot `[1, 1]`; no tagged values, no warning.
- `test_body_failure_stays_the_reason_when_recording_fails`: positive control, a fake PX4 without telemetry gives
  smoke01's "HARNESS_INVALID no telemetry within 5 s" (fast timing) as a recorded HARNESS_INVALID result with its
  trace; with a recording failure added (a value whose `repr` raises), `RecordingAfterError` starts with that first
  failure, then the recording's `ValueError` (also its `__cause__`), and `run_unit`'s abort reason has the first
  failure ahead of the second; negative control: a recording failure with no earlier failure is raised as itself.
- `test_unserializable_values_are_counted_and_warned`: a set and an opaque object in a received message: count 2, one
  warning, the scenario still VALID and passed, the count matches the written trace; a clean run has count 0 and no
  warning; `integrity_problems` flags a ledger count that differs from the trace; `reading` lists the warning.
- `test_recording_contract_...`: picks an ATTITUDE packet with a simulation time (packets before the first position
  now have none, because nothing is read before the pin).

Bytes and exit status (first draft):

- `real_mavlink()` / `real_messages()`: real pymavlink objects decoded by a robust-parsing `MAVLink` receiver from wire
  bytes: HEARTBEAT, STATUSTEXT with a non-UTF-8 byte, PARAM_VALUE, FILE_TRANSFER_PROTOCOL (uint8[251]), a CRC-broken
  heartbeat (BAD_DATA) and msgid 54321 (UNKNOWN_54321). pymavlink is imported from its dialect module, not `mavutil`
  (which writes `os.environ`); `sys.path` and `sys.modules` are restored. Skipped only where pymavlink is absent.
- `TestRealMavlinkRecording` (3 tests): the six real messages go through `handle_message` and `trace_text` and come back
  byte for byte (`packet_hex` = `get_msgbuf()`, payload = `to_dict()` after un-hexing, two `bytes_hex` fields, the
  decodable four re-parse from the recorded wire bytes to the same message); negative control: the raw BAD_DATA
  `to_dict()` raises the smoke01 `TypeError` under plain `json.dumps`; sends with bytearray, memoryview and bytes
  arguments through `instrument_link`; `json_safe`/`json_default` types, NaN unchanged, tagged fallback.
- `TestLivePath.test_live_path_records_real_bad_data_and_unknown_messages`: the fake-PX4 live path with the six real
  messages injected through `handle_message` runs N0 to COMPLETED/VALID/passed, writes `trace.json`, the packets
  round-trip, and the ledger recomputation from the written trace agrees (the only integrity problem is the
  deliberately shortened schedule).
- `TestLaunchRefusals.test_amendment1_binds_scope_smoke02_and_refuses_the_v2_approval`: SCOPE and v2 SCOPE quoted
  here, `AMENDMENT` in `OWN_SOURCES`, `smoke02`/`attempt01`, an approval with v2's SCOPE refused, the new one accepted.
- `test_smoke_gate`: the fixture binds the amendment; a smoke whose `attempt.json` lacks it is refused, then accepted
  again (positive control). `test_analyze_refuses_to_overwrite`: fixture binds the amendment.
- `TestExitStatus` (2 tests): the exit rule (COMPLETE 0; ABORTED, STOPPED_AFTER_A, empty or missing final 3; COMPLETE
  smoke refused by the gate 4); `main` through the real `launch()` with its refusals, `run_unit` and smoke gate
  replaced and the real `write_final`: `--smoke` ABORTED 3 (smoke01's case), gate-refused 4, COMPLETE 0; `--launch`
  ABORTED 3, COMPLETE 0; a refusal or a refused smoke gate raises `SystemExit("refused: ...")`.

## 6. What did not change

The parameter set, scenarios, schedule (`N0 N0R N0J` smoke, ten attempt scenarios), every criterion, path gate,
validity rule, stop rule, timing (the heartbeat wait stays 120 s), the wire addressing of every send, the data contract
other than the additions in §3, ULog rules, reading labels, the launch environment and commands,
`attempt01` as the attempt folder, and the single-use rules. The recorded smoke01 stays ABORTED in its folder; it is
not reread or rewritten. Everything not listed in §3 keeps the v2 preregistration's wording and force.
