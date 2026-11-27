# REVIEW_amend1_r2: repairs to S5 v2 amendment 1 and S4R-1 v1 amendment 1 (2026-11-10)

- Reviewer: Claude Opus 5.5 (independent subagent, fresh context). Read-only on the worktree
  `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly` (camonly/dev, HEAD fd753f2, 4 modified and 2 untracked files).
  No PX4, Gazebo, GPU, `--smoke`, `--launch` or network. Raw smoke01 folder read only. Mutations ran on a scratch copy
  outside the worktree (session scratchpad, `mut/`).
- Inputs: REVIEW_amend1_r1.md, REPAIR_smoke_r2.md, `git diff`, both amendments, raw `s5_safety_v2/smoke01/`
  (final.json, ep000/session.json, logs/px4.log, attempt.json), pymavlink 2.4.50 in the px4 env and the
  `~/motar_tmp/sitl_plan_20261104/pylink` copy, and PX4 v1.14.3 source (`git describe` = v1.14.3).

## Tests (cores 10,11, nice 10, aerialgym env, focused command as given)

| Suite | Result |
| --- | --- |
| `test_sitl_s5_safety.py` | 62 run, OK, 0 skipped |
| `test_sitl_s4r1.py` | 44 run, OK, 0 skipped |

A full PUBLIC_CPU (`tools/run_tests.py`, cores 12-15, PID 1370874) was already running from another session when I
checked, so I did not start a second one. Its result is not part of this review.

## BLOCKER-1 re-check: FIXED

1. **Live interpreter and library.** The prereg's command runs `/home/fair/miniconda3/envs/px4/bin/python`, which
   imports site-packages pymavlink 2.4.50 (default dialect `v10.all`). `sitl_s3b_closed_loop` appends the `pylink` path
   last, so it does not shadow that copy. `diff -rq` finds the `pylink` copy byte-identical, so the tests (aerialgym
   env, through `pylink`) use the same library.
2. **Wire version.** PX4 SITL's `rcS:173` sets `MAV_PROTO_VER 2`, so PX4 sends MAVLink 2 from the start.
   `mavutil.auto_mavlink_version` switches to `v20.all` on the first 0xFD byte (`mavutil.py:284-298`). The tests'
   MAVLink 2 'all' setup therefore matches the live link.
3. **Source address.** `rcS:131` sets `MAV_SYS_ID = instance+1 = 1`. PX4's component id is 1. `HEARTBEAT.hpp:134`
   sends `MAV_AUTOPILOT_PX4` with the `MAV_TYPE` vehicle type, so PX4's heartbeat comes from (1, 1) and passes
   `autopilot_of`.
4. **Other heartbeats.** The S5 link (udpin 14540) receives from PX4's onboard instance (14580, `-f`). The SITL
   instances are GCS 18570 `-f`, payload 14280 `-f` and gimbal 13030 without `-f`. Forwarded traffic only arrives if
   an external peer sends some, and OwnedSession refuses foreign gazebo/px4 processes. A GCS, gimbal or camera
   heartbeat would be rejected anyway, because the rule requires component 1, autopilot 12 and a vehicle type. No
   PX4 v1.14.3 path sends a non-autopilot heartbeat from component 1.
5. **Receive paths.** `handle_message` now reads only `link.autopilot`, and reads nothing before the pin.
   - Every receive-side reader uses the pinned source: `hb`/`hb_log`, ACKs, PARAM_VALUE (and therefore
     `read_params`), `command_inventory` (`r["autopilot"]`) and `recording_contract` (`autopilot_source`).
   - `target_component` now appears only on the send side (Streams `_send`, `param_set`, base `cmd`/`read_params`)
     and in the `"destination"` record.
   - PX4 accepts component 0 on all of these sends. I checked `mavlink_receiver.cpp:382-383` and `:951`, RC
     override and MANUAL_CONTROL (system only), and `mavlink_parameters.cpp:103` and `:176`.
   - No `[1,0]` or `[1,1]` assumption is left, apart from the documented `[1,1]` fallback for records without a pin.
6. **The real-mavutil test is genuine.** It packs real bytes with the dialect and reads them with
   `mavutil.mavlink_connection(file, notimestamps=True)`, which goes through `recv_msg`/`post_message`. The camera
   heartbeat (1, 100) first locks pymavlink's sysid, and the asserted target is (1, 0) while the pin is (1, 1).
   `FakePX4Link.m` is a real connection, so the full live-path tests pin through mavutil.
7. **Mutations (my own, scratch copy).** The unmutated copy passes. Each mutation below is killed:

   | Mutation | Result |
   | --- | --- |
   | Pin set to pymavlink's target | 13 fail (matches the repair note) |
   | v2's filter restored | 15 fail (matches the repair note) |
   | Component check dropped from `autopilot_of` | 14 fail |
   | Type check dropped | 1 error |
   | `RecordingAfterError` chaining removed | 1 error |
   | `command_inventory` matched to the destination | 1 fail |
   | `recording_contract` source set to `[1,0]` | 9 fail |

8. **First-failure chaining is correct.**
   - The `except Exception` path stores `res["error"]`. If recording then fails, the code raises
     `RecordingAfterError(first, second)` from e2, so `run_unit`'s reason starts with the body failure.
   - With no earlier failure, the recording error is raised unchanged.
   - `sys.exc_info()` in the `finally` is non-None only while a BaseException is propagating. The only call site
     (`run_unit`, line 1787) is not inside an `except`.
   - The BaseException path has no test, which is disclosed.

## Other items

- MINOR-1 (S4R-1 §3.2): fixed. The text now says "set at the observed maximum (no margin)", the arrival-order caveat
  carries the 988/984/980/992 example, and the count is 1,876. The code comment says the same. L1 test added.
- MINOR-2: fixed in both tools. Tagged values are counted and produce a warning that `integrity_problems` re-checks.
  Positive and negative tests exist.
- S5 amendment text: §1 gives the true first failure (the receipt quote and PX4 boot), §2.1 gives the diagnosis with
  correct mavutil lines, and §2.2 says the bytes were not the sole cause.
- SCOPE: the S5 `SCOPE` in the code is quoted verbatim in the amendment (checked independently). The S4R-1 SCOPE test
  passes. Both `SMOKE_DIR` values are the smoke02 folders. The old approvals are refused.
- Live path before and after smoke01's stopping point: the `SET_MESSAGE_INTERVAL` ids are all below 256 and were used
  live by S3b. With RC lost at arming, PX4 v1.14.3 does not apply the RC-loss action (`failsafe.cpp:470`), so N0
  should arm. The GCS heartbeat satisfies the data-link arming check. I found no other predictable live defect.
  Everything after telemetry remains INFERRED and untested live, as the prereg already states.

## Findings

- BLOCKER: none.
- MAJOR: none.
- MINOR: none.
- LOW-1 (S5): sends use pymavlink's `target_system` (the first vehicle-like heartbeat). This is not checked against
  the pinned system. They agree in single-vehicle SITL. A one-line fail-closed check is optional.
- LOW-2: the BaseException-during-recording path has no test (disclosed). The r1 items L3-L5 stay as disclosed.

Must fix before commit: nothing. LOW items are optional.

VERDICT S5 v2 amendment 1: PASS_WITH_LIMITS (live behaviour after telemetry is not tested; LOW-1)
VERDICT S4R-1 v1 amendment 1: PASS
VERDICT overall: PASS_WITH_LIMITS
