# REVIEW_amend1_r1: S5 v2 amendment 1 and S4R-1 v1 amendment 1 (2026-11-10)

- Reviewer: Claude Opus 5.5 (independent subagent, fresh context). Read-only on the worktree
  `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly` (camonly/dev, HEAD fd753f2 plus the uncommitted diff of 4 files
  and 2 new amendment files). No PX4, Gazebo, GPU, `--smoke`, `--launch` or network. Raw smoke folders read, never written.
- Inputs: `git diff`, both amendments, both committed preregs, `REPAIR_smoke_r1.md`, raw
  `~/motar_tmp/sitl_next_20261110/s5_safety_v2/smoke01/`, `~/motar_tmp/s4r1_20261110/smoke01_contact_on_env0/`,
  committed manifests in `docs/evidence/sitl_2026-11-05/{s5_safety_v2,s4r1}/`, installed pymavlink 2.4.50 (px4 env).

## Tests run (cores 10,11, nice 10, aerialgym env, focused command as given)

| Suite | Result |
| --- | --- |
| `test_sitl_s5_safety.py` | 54 run, OK, 0 skipped |
| `test_sitl_s4r1.py` | 42 run, OK, 0 skipped |

Raw evidence integrity: S5 smoke01 manifest 13/13 files SHA-256 match; S4R-1 smoke01 manifest 142/142 match. Committed
preregs and evidence files are not in the diff.

## Findings

### BLOCKER-1 (S5; must fix before commit): the S5 smoke01 body had already failed with "HARNESS_INVALID no telemetry within 90 s"; the bytearray TypeError masked it, and the repair does not address the cause

Evidence:
- `smoke01/ep000/session.json` (OwnedSession receipt) records `"exception": "RuntimeError: HARNESS_INVALID no telemetry
  within 90 s"`, cleanup reason `exception` at t=95.2 s. PX4's log shows a normal boot ("Ready for takeoff", "partner IP:
  127.0.0.1"). The TypeError came afterwards from `trace.json` in `run_scenario`'s `finally` and replaced the result.
  Neither amendment §1/§2 nor `REPAIR_smoke_r1.md` mentions this; the amendment's account of smoke01 is incomplete.
- Cause, reproduced offline: pymavlink 2.4.50 `wait_heartbeat()` sets `sysid` only. `target_component` stays **0**
  (`mavutil.py:424-426`; a tlog with a sys1/comp1 HEARTBEAT gives `target 1 0`). `handle_message` (`sitl_s5_safety.py:432`)
  returns early whenever `src != [target_system, target_component]` = `[1, 0]`. Every PX4 (comp 1) message is therefore
  dropped: `pos`, `att`, `truth` and `hb_log` never fill, and PX4's heartbeats count as `other_component_hb`. Feeding real
  HEARTBEAT, LOCAL_POSITION_NED and ATTITUDE objects through `handle_message`: with target_component 0, pos False,
  att False, hb_log 0, other_component_hb 1; with 1, all filled.
- Every test fake sets `target_component=1` (`tests/test_sitl_s5_safety.py:254, 373, 705`), which hides this. The new
  "real pymavlink" tests use real messages but not real `mavutil` target semantics.
- Consequence: smoke02 under this amendment would almost certainly ABORT again, on the same body failure, after the
  boot. It needs a dated repair (pin the target to the autopilot component, e.g. 1, or take it from the first vehicle
  heartbeat's source), recorded in amendment 1 (or an amendment 2), with a test that derives the target from a real
  `mavutil` connection (an offline tlog works), and a negative control for target_component 0.

### MINOR-1 (S4R-1; fix wording before commit): the tolerance justification is inaccurate
Amendment §3.2 says that within `INTERVAL_JOIN_NS` "a late message neither opens nor splits a contact interval". AGQ's
`contact_audit` builds intervals in arrival order (`_last_ns` = last arrival). Order 988, 984, 980, then 992 gives a
12 ms (3-step) difference, which opens a new interval. A late first contact can also leave an interval with start > end
and an onset 1 step late. The effect is small and only touches descriptive `bar_events` (not read in analysis), but the
sentence must be corrected or qualified. The 2-step bound also equals the smoke's maximum with zero margin. That is
disclosed and fail-closed, but it should be stated as "set at the observed maximum", not "not fitted".

### MINOR-2 (both tools): the tagged-repr fallback fails open without a flag
`{'unserializable_type', 'repr'}` keeps a recording valid for any unexpected type (set, object), and nothing counts it.
Count these entries into `recording_problems` (or at least into a descriptive count) so a silent loss is visible.

### LOW
- L1: S4R-1 has no test that the old `execution_approval.json` (v1 SCOPE, ede0a14) is refused. It is refused by
  construction (`SF.approval_problems`: scope and commit), and S5 has the test. REPAIR note Finding 3 overstates the
  test coverage for S4R-1.
- L2: The code comment `sitl_s4r1.py` CONTACT_LATE block says "1875 window messages"; measured 1,876.
- L3: S5 trace NaN/inf are still written as bare `NaN` (non-strict JSON); unchanged from v2 and disclosed.
- L4: S4R-1 `--smoke` exit 0 uses `instrumentation_smoke_problems` only. `--launch` also applies
  `S3C.smoke_problems`, so a 0 is necessary but not sufficient.
- L5: The WORKLOG fd753f2 entry says "the outcome was capture". That is the Isaac reference. The SITL member's physical
  outcome was `crash` (bar) at 61 steps. A smoke is not a result, but the next entry should say so.

## Checks requested

1. **Bytes.** Every bytes-like path found is covered: S5 `rx_packets.payload` and `tx_calls.args` (`json_safe`),
   `trace.json` (`json_default`), packet_hex; S4R-1 `mav_recv` and `mav_send` events, `write_json`, `dump_tap`, and the
   ULog extraction `jsonable`. final.json and attempt.json use `default=str`, which never aborts. Numpy scalars and arrays
   are handled, and nonfinite values are tagged in S4R-1 (`allow_nan=False`). The encoding is lossless (hex); the
   container type is not kept (disclosed). Tests use real pymavlink BAD_DATA and UNKNOWN_54321 objects, with a
   negative control. PASS, except MINOR-2.
2. **Exit status.** COMPLETE gives 0; ABORTED, any non-COMPLETE status or a missing final gives 3; a gate-refused smoke
   gives 4; a refusal gives `SystemExit` with status 1; SIGTERM re-raises (non-zero); `--extract-ulog` returns 0. No
   abort path exits 0. PASS.
3. **Contact amendment (S4R-1).** Re-derived independently with my own depth-0 parser: 10,723 lines, 10,723 parsed,
   all distinct, sorted intervals all 4 ms (0.372–43.260 s), 1,876 in the window [35.184, 42.684], 3 late arrivals at
   35.980–36.000 s (lateness 1, 2, 1 steps; maximum index displacement 2). AGQ-style adjacent counts are 4 gaps, 6
   missing and 3 non-increasing. This matches the diagnosis. The rule is disclosed as post-smoke. It is non-vacuous:
   tests fail a real missing step (also when reordered), a duplicate, an off-grid time, a 3-step lateness and more than
   1 % late, and accept the boundaries. Coverage and floor_control are unchanged, and AGQ's replaced messages are kept.
   PASS_WITH_LIMITS (MINOR-1).
4. **Folders and SCOPE.** smoke02 and smoke02_contact_on_env0; the attempt gates read SMOKE_DIR; the SCOPE strings are
   quoted verbatim (tests and my own check); the old approvals carry the old scope and commit ede0a14, so they are
   refused; the original evidence is untouched (manifests match). PASS (L1).
5. **Missed issues.** BLOCKER-1 (S5). For the S4R-1 smoke member: ULog all_verified with 0 integrity problems, seed
   4501 bound, logger stop rc 0, contact control ok (250/250 floor, 0 gaps), RTF 1.0003, LATE max 0.084 s, all 62 packet
   and 61 step npz files written before the events write failed. No other latent S4R-1 defect found.

## Must fix before commit
BLOCKER-1 (with a test and amendment text), MINOR-1 (wording). MINOR-2 is recommended; LOWs optional.

VERDICT S5 v2 amendment 1: AMEND_FIRST
VERDICT S4R-1 v1 amendment 1: PASS_WITH_LIMITS
VERDICT overall: AMEND_FIRST
