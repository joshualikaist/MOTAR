# REVIEW r1 — dev08 parameter/telemetry boundary (`tools/sitl_dev08_link.py`, `tests/test_sitl_dev08_link.py`)

- Reviewer: independent reviewer, Claude Opus 5.5 (Agent model opus), fresh context, read-only
- Date: 2026-11-05
- Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly` (branch camonly/dev), both files untracked
- Reviewed bytes (sha256):
  - `tools/sitl_dev08_link.py` 4d07ca64f39fab5320b6eb677470bf03e7197b3e0cb75681b15f0b7f952d22cc
  - `tests/test_sitl_dev08_link.py` f0ec7197cfcf0c3b5a99d2098d20343413c77a583f6c99cd2931f28a3ebc66a1
- Contract: `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md`, row "Parameter/telemetry
  boundary" and "Checkpoint and stopping contract"
- PX4: `/home/fair/workspaces/px4_ws/PX4-Autopilot` at tag v1.14.3 (1dacb4c). `git diff` is clean for
  `src/modules/mavlink`, `src/lib/parameters` and `src/systemcmds/param`; the only local change is
  `sitl_targets_gazebo-classic.cmake`. The mavlink object (2026-11-05 01:19) is newer than the stream headers.

## Verdict: FAIL

The parameter guard is sound. Its PX4 assumptions are confirmed by source, and its rejection paths are tested
except for two gaps (findings 4 and 5). The telemetry gate is not. Its truth-stream model contradicts PX4 v1.14.3:
`HIL_STATE_QUATERNION.time_usec` is never set and is always 0. Against PX4's real stream, `align()` therefore
raises `CLOCK_NOT_INCREASING` in both `truth_clock` modes. The gate cannot produce a snapshot live, and its
truth-freshness, truth-reuse and truth-clock checks have no valid clock to work from. It fails closed, so this is
not a validity hazard, but the boundary is not ready for live integration. Separately, the contract requirement
that "all packet freshness … checks … reject mutants" is only partly met by the tests (finding 3).

## Findings

### 1. BLOCKER — The truth-clock assumption is contradicted by PX4 source, so the gate cannot align against real PX4
- Code:
  - `tools/sitl_dev08_link.py:194-195`: `sim_time("truth") = time_usec*1e-6`.
  - `:241-243`: strict per-stream increase.
  - `:274-285`: `truth_clock="px4"` default, or offset `"estimate"`.
  - `:324-336`: truth freshness and reuse use that clock.
  - The docstring at `:14-15` and `:20-21` calls the clocks an assumption for the smoke to confirm, but this one can
    be checked from source.
- PX4 fact: `src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp:80-105` builds `mavlink_hil_state_quaternion_t
  msg{}` and assigns the quaternion, rates, lat/lon/alt, velocities and accelerations. It never assigns `time_usec`.
  The field exists (`build/px4_sitl_default/mavlink/common/mavlink_msg_hil_state_quaternion.h:8`), so it is sent as
  0. The clock information exists but is dropped twice:
  - Gazebo stamps its own message with sim time (`gazebo_mavlink_interface.cpp:753`), and PX4 ignores it.
  - PX4 re-stamps the uORB groundtruth with its lockstep clock (`SimulatorMavlink.cpp:521`), and the stream does not
    copy that timestamp.
- The historical runner is consistent with this. It stamps truth with `time.time()` on receipt
  (`tools/sitl_s3b_closed_loop.py:131,145`).
- Failure scenario, reproduced in `probe_px4_truth.py`: real pymavlink encode/decode, no socket, PX4-shaped streams
  on a 4 ms grid, `time_usec=0`.
  - `px4` mode: `CLOCK_NOT_INCREASING: truth 0.000000 after 0.000000`.
  - `estimate` mode: the same.
  - Control: the same streams with `time_usec` on PX4's clock align in both modes.
  - Every live episode, smoke included, would abort at alignment.
- The test fixture models PX4 incorrectly: `tests/test_sitl_dev08_link.py:193` stamps truth with
  `(t + offset) * 1e6`. The "valid" positive control is not PX4's stream, and PX4's real stream is in effect a
  rejected mutant.
- Suggested fix: choose and disclose a truth time model, then rebuild the truth checks on it.
  - (a) Bracket each truth sample in PX4 time by arrival order. PX4 sends all three streams from one mavlink
    instance. Pair each truth sample with the LOCAL_POSITION_NED/ATTITUDE samples received just before and after
    it. Require the checkpoint's truth to be received in the same send burst as the selected position sample (recv
    gap bound), and require a new seq. State the uncertainty of about one stream period (20–24 ms). Burst adjacency
    is live-only and must be confirmed in the smoke.
  - (b) A truth source that carries simulator time, e.g. a Gazebo-side pose with sim time, which is PX4's lockstep
    clock. This is a new component and needs its own review.
  - (c) Wall-clock receipt as in dev07, with the limit disclosed.
  - Whichever is chosen:
    - drop the `time_usec` monotonicity check for truth, or replace it with recv/seq order;
    - make `time_usec=0` the default in the valid fixture;
    - add a source-pinned test that skips when the PX4 tree is absent, as `test_airframe_parser_and_installed_airframe`
      does, and asserts that `HIL_STATE_QUATERNION.hpp` does not assign `time_usec`.

### 2. MAJOR — Historical live data contains a position skip that this gate aborts on; at the dev07 rate, a 42-episode unit is likely to abort
- Evidence: `probe_hist_lateness.py` and `probe_hist_ep02.py` over the 20 dev07 step logs (column 1 is the position
  sim time each check consumed).
  - 19 episodes: lateness ≤ 0.040 s. Consumed samples are all on a 4 ms grid (864 samples).
  - `dev07_70/ep02`, k=28: the sample was 0.136 s late (compute 5.7 ms), and k=29 reused the same sample (t=38.300).
  - The historical loop had no check and recorded this episode as valid.
- With `SKIP_MAX_S=0.05` (`:37`, `:312-316`) this episode aborts. Probe P6 reproduces it: `CLOCK_SKIP`. Amendment1
  makes a clock stall or telemetry mismatch abort the unit. If 1 in 20 holds, P(at least one abort in 42
  episodes) = 1 − 0.95^42 ≈ 0.88.
- This is not a code defect. It is a viability risk for the proposed scope, and it should be decided before approval.
- Suggested fix:
  - keep the bound, but record diagnostics in the error: recv times and sim times of the samples around the gap, plus
    the rx-thread queue depth, so that a runner-side (GIL/thread) stall can be told apart from a PX4/Gazebo stall;
  - have the smoke measure the gap rate;
  - state in the protocol that the expected abort probability was considered.

### 3. MAJOR — The tests do not show that each freshness check rejects a mutant (contract: "all packet freshness … checks … reject mutants")
Seven single-site mutants survive the whole test file (`mutate.py`, 39 mutants). For each, `survivors*.py` shows an
input that the original rejects and the mutant accepts, with a valid positive control.

| Mutant | Code | Input the original rejects | Original | Mutant |
| --- | --- | --- | --- | --- |
| M21: align-time attitude freshness removed | `:268-269` | attitude 0.08 s behind position at alignment | STALE_ATT | accepted |
| M22: align-time truth receive pairing removed | `:270-273` | truth received about 0.4 s apart from the positions | STALE_TRUTH | accepted |
| M31: per-checkpoint truth freshness unbounded | `:324-326` | every truth sample 0.08 s old, new seq (control: 0.02 s lag accepted) | STALE_TRUTH | accepted |
| M35: attitude freshness bound doubled | `:322` | attitude 0.08 s old at every checkpoint (control: 0.02 s accepted) | STALE_ATT | accepted |
| M18: `<=` weakened to `<` | `:242` | duplicated position timestamp | CLOCK_NOT_INCREASING | accepted |

`test_stale_companions` (`tests:298-309`) tests only total silence. The truth-silence case is killed by the reuse
check, not by the freshness bound. Suggested fix: add the five inputs above as tests, each paired with a valid
control at the bound (e.g. 0.04 s accepted, 0.06 s rejected).

### 4. MINOR — Stale native yaw in the native arm is not tested (mutant M02 survives)
- Restricting the fresh check to overrides (`:173-174` → `for n in overrides`) survives. The only stale test
  (`tests:123-128`) uses MC_ROLLRATE_P, which both arms override.
- The module itself is correct. Probe P10: booting the native arm with MC_YAWRATE_P=0.2 gives PARAM_NOT_FRESH.
- This case is central to the amendment's "effective native yaw gains require fresh readback". Add it as a test.

### 5. MINOR — "Exactly that float32" echo is not tested (mutant M11 survives)
- A 1e-3 echo tolerance survives, because `stale_echo` differs by 0.09. Probe: an echo of want + 1 float32 ulp gives
  PARAM_ECHO_MISMATCH in the original and is accepted by M11.
- Add a ulp-off echo test.

### 6. MINOR — Parameters that scale the overridden roll/pitch gains are not recorded
- `RECORD_PARAMS` (`:43-46`) includes the whole yaw group but not MC_ROLLRATE_K / MC_PITCHRATE_K (or the roll/pitch
  FF and MAX).
- PX4 multiplies P/I/D by K (`mc_rate_control/MulticopterRateControl.cpp:81-86`; `mc_rate_control_params.c:106-121,
  186-201`). mc_autotune writes these gains (`mc_autotune_attitude_control.cpp:546-571`), and rcS enables it by
  default (`rcS:175 MC_AT_EN 1`).
- Add MC_ROLLRATE_K, MC_PITCHRATE_K, MC_ROLLRATE_FF, MC_PITCHRATE_FF, MC_ROLLRATE_MAX and MC_PITCHRATE_MAX.

### 7. MINOR — The fresh-boot check covers only parameters the airframe sets
- `:173-174` compares only against `fresh_defaults`, the airframe set-default lines. Probe P11: MC_YAWRATE_FF=0.5 and
  MC_YAWRATE_K=2.0 at boot are accepted silently. The values are still recorded.
- The risk from a reused dev08 file is covered, because both arms override airframe-defaulted MC_ROLLRATE_P
  (0.013 vs 0.025).
- Suggest also comparing against firmware defaults from the bound build, e.g. its `parameters.json` or the
  `PARAM_DEFINE_*` values. The fixture's FIRMWARE dict matches source: MPC_XY_VEL_P/I/D_ACC 1.8/0.4/0.2,
  THR_MDL_FAC 0, MC_YAWRATE_D/FF 0, K 1.

### 8. MINOR — The gate has no start cursor, so it validates every message since the Inbox was created
- `TelemetryGate.__init__` sets `self.cursor = 0` (`:210`). The Inbox is shared with the parameter phase, so the
  first `align()` ingests the whole boot, parameter and takeoff history.
- Any non-finite or non-increasing sample there latches an error and aborts. Probe P4: a NaN position about 1 s
  before alignment gives NONFINITE at align.
- Whether PX4 emits such samples during EKF initialisation is live-only.
- Suggest `TelemetryGate(inbox, start_cursor=inbox.cursor())`, or starting validation at an explicit episode cursor.

### 9. MINOR — Checkpoint order is not anchored
- The first `checkpoint()` may use any k (`:296` only checks k = last_k + 1). Probe P2: k=5 is accepted as the first
  call.
- A second `align()` silently resets the order and start (`:286-289`). Probe P3 confirms this.
- The contract's records start at frame 0. Require k == 0 first, and allow `align()` only once per gate.

### 10. MINOR — An rx-thread failure is not recorded
- `PymavlinkTransport.rx_loop` (`:363-369`) has no exception handling. Probe P9: an `OSError` from `recv_match`
  kills the thread and surfaces only through `threading.excepthook`. The Inbox stays empty.
- The guard then fails later as a generic PARAM_MISSING or CLOCK_STALL. That is fail-closed, but the cause is lost.
- Catch the exception, put an `RX_ERROR` item (or latch it on the transport), and have the gate and parameter waits
  report it.

### 11. MINOR — The airframe parser drops unparsable values silently
- `:76-78` `continue`s on ValueError. Probe P13: `0.11#x` and `${V}` are dropped, and those parameters leave the
  fresh check without notice.
- `airframe_checked` is returned, but no expected set is asserted. Fail on any `param set-default` line that does not
  parse, or assert that the parsed names equal the names found by a looser scan.

### 12. NOTE — Abort semantics depend on the runner
- `ParamError` and `TelemetryError` are `RuntimeError` subclasses, and errors latch (`:218-221`, `:246-248`). That
  meets "failures are errors".
- However, the historical `run_episode` catches `Exception` into `outcome="ERROR"` in the episode record
  (`tools/sitl_s3b_closed_loop.py:320-322`). Reusing that pattern would turn an abort into an outcome.
- Suggest a common base class (e.g. `BoundaryAbort`) and a runner-level test that it propagates and marks the unit
  invalid.

### 13. NOTE — Sample selection differs from the historical runner (disclosed in the docstring, reasonable)
- Historical: the latest received `L.pos`/`L.att`, with truth stamped on wall-clock receipt, no freshness or
  companion wait, and samples reusable across checks (ep02 k=28/29).
- New:
  - position is the first sample at or after `start + k*DT` (`:303`);
  - attitude and truth are the nearest within ±0.05 s and may be later than the position (probe P8);
  - it waits up to 0.2 s for companions (`:319`);
  - it requires a new truth seq.
- The two differ only when the runner is up to 0.05 s late (probe P7: the snapshot is 0.04 s older than the newest
  sample) or when a companion is late. 10 Hz simulator-time pacing on LOCAL_POSITION_NED `time_boot_ms` is kept.
- Record the rule change in the runtime receipt as a lifecycle difference from dev07. The amendment already treats
  dev07 as descriptive only.

### 14. NOTE — Smaller points
- `CLOCK_SKIP` is implied by `RUNNER_LATE`, because the newest sample is never older than the selected one
  (`:312-316`). Deleting either still fails closed (M25/M26), so it is diagnostic only.
- The Inbox keeps every message for the life of the connection (`:89-96`; probe P14: 20 000 retained).
- The parameter guard cannot tell PX4's echo apart from PX4's later broadcast of unsaved parameters
  (`mavlink_parameters.cpp:358-407`). Both carry the stored value, so the guarantee "stored value == requested
  float32" still holds; the docstring's "PX4's own PARAM_VALUE echo" is slightly stronger than what is checked.
- `PymavlinkTransport.lock` (`:351`) serialises only its own parameter sends. Setpoint/command sends elsewhere must
  take the same lock: pymavlink's `send` updates the sequence number without locking.
- The gate is single-consumer and not thread-safe. Only the Inbox is shared with the rx thread, and it is correct: a
  Condition, a predicate on length, no lost wakeup in `wait_for` (`:110-125`).
- No new ground-truth policy input is created. Truth is returned as a separate snapshot field. The runner must keep
  using it only for rendering/termination, plus the amendment's fixed truth command yaw.
- The test file does not touch `sys.modules`, `os.environ` or `sys.path`, so PUBLIC_CPU pollution risk is low.
  PUBLIC_CPU discovers it automatically (`tools/run_tests.py:111`).
- The 0.2 s wall-clock STREAM_STALE in the fixtures could flake on a very loaded host.

## PX4 assumption table

| # | Assumption in the module | Status | Source (PX4 v1.14.3 unless noted) |
| --- | --- | --- | --- |
| A1 | Autopilot is system 1, component 1 (instance 0) | Confirmed | `rcS:131` MAV_SYS_ID=px4_instance+1; `mavlink_params.c:41,50` defaults 1/1; `mavlink_main.cpp:108-118` |
| A2 | PARAM_SET (matching target, known name, REAL32 to a float parameter) is answered by PARAM_VALUE carrying the stored float32 | Confirmed, with caveats | `mavlink_parameters.cpp:102-136` (param_set then send_param on the same channel, `:538`). No reply for an unknown name or a type mismatch (`:125-131`), or when the TX buffer is short (`:471-473`); each fails as PARAM_ECHO_MISSING. Value stored unless within FLT_EPSILON (absolute) of the stored value (`parameters.cpp:736-739`) |
| A3 | PARAM_VALUE `param_type` 9 = float, 6 = int32; int32 bit-copied into the float field | Confirmed | `mavlink_parameters.cpp:481-488,526-534`; `mavlink_types.h:130,133`; `common.h:743` |
| A4 | PARAM_REQUEST_READ by name (index −1) gets a PARAM_VALUE; unknown name gets none | Confirmed | `mavlink_parameters.cpp:175-202`, `:466-468` |
| A5 | Unsolicited PARAM_VALUE may also arrive | Confirmed (harmless) | `mavlink_parameters.cpp:358-407` (unsaved params ≥ 5 ms after an update) |
| A6 | A fresh boot reports the airframe's `param set-default` values | Confirmed by source for this airframe, given no `parameters.bson`/backup in the working directory | Import precedes the airframe (`rcS:67-102` vs `rcS:225-227`). param_get precedence user > custom default > static (`parameters.cpp:383-413`). set-default parses `strtod`→float (`param.cpp:873`), the same as `f32()`, and is applied only if Δ > FLT_EPSILON (`param.cpp:875`); for this airframe no value is within ε of firmware. No later boot script sets these names (grep of `build/.../etc`: only `rc.vtol_defaults`, not sourced for MC). Runtime writers before the read (mc_autotune `:546-571`): live-only |
| A7 | A reused parameters.bson shows up as booted ≠ airframe | Confirmed | Precedence above; autosave on change (`parameters.cpp:753`). Both arms override airframe-defaulted MC_ROLLRATE_P |
| A8 | LOCAL_POSITION_NED and ATTITUDE `time_boot_ms` are on PX4's clock | Confirmed | `LOCAL_POSITION_NED.hpp:67`, `ATTITUDE.hpp:73` (uORB timestamp/1000); lockstep `SimulatorMavlink.cpp:473-512`. Live: dev07 consumed samples on a 4 ms grid |
| A9 | HIL_STATE_QUATERNION `time_usec` is on PX4's clock (default `truth_clock="px4"`) | **Contradicted** | `HIL_STATE_QUATERNION.hpp:80-105` never sets it, so it is 0. `"estimate"` mode fails too (`:241-243`) |
| A10 | Truth source is (1,1); values are groundtruth | Confirmed | `HIL_STATE_QUATERNION.hpp:60-63,105`; `SimulatorMavlink.cpp:516-558` |
| A11 | The three streams run at about 50 Hz after SET_MESSAGE_INTERVAL 20000 µs | Live-only | Streams send only on uORB update (`LOCAL_POSITION_NED.hpp:64`, `ATTITUDE.hpp:66`, `HIL_STATE_QUATERNION.hpp:67`). dev07 lateness 0.020–0.024 s typical, max 0.040 s, one 0.136 s event |
| A12 | Millisecond timestamps strictly increase per stream | Live-only | Same send-on-update logic; not observable from the 10 Hz dev07 subsample |
| A13 | pymavlink `to_dict` gives a NUL-stripped str `param_id` (16-char safe), a float `param_value` and a list quaternion | Confirmed | `pylink/pymavlink/dialects/v20/common.py:175-180,17192`; probe P1 (MPC_Z_VEL_MAX_UP round trip) and `probe_px4_truth.py` |

## What I ran (all under `/home/fair/motar_tmp/dev08_link_20261105/review_r1/`)

All runs used CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE=1, nice 10, taskset 6,7,12,13 and a timeout. No socket, PX4,
Gazebo or GPU was used, and no repository or PX4 file was written.

- `unittest_run.out`: the repository test file, as given. **22 tests, OK** (3.0 s).
- `probe_hist_lateness.py` / `probe_hist_ep02.py` (`.out`): dev07 checkpoint lateness, read-only, 20 episodes.
- `probe_px4_truth.py` (`.out`): PX4-real truth stream through real pymavlink encode/decode. Finding 1.
- `probe_semantics.py` (`.out`): probes P1–P14, quoted in the findings.
- `mutate.py` (`mutate.out`, `mutants.json`, `mut/*`): 39 single-site mutants run against the unmodified test file.
  - 31 killed.
  - 1 killed only by the 90 s timeout: M28, a defective mutant that loops forever. The corrected M28b is killed by
    `test_stall`.
  - 7 survived: M02, M11, M18, M21, M22, M31, M35.
- `survivors.py`, `survivors2.py` (`.out`, `survivors.json`): for every survivor, an input the original rejects and
  the mutant accepts, plus positive controls. All 7 gaps are confirmed.
- `src/sitl_dev08_link.py`: the copy that was probed (same sha256 as reviewed). `reviewed_sha256.txt`: hashes of the
  reviewed files.

## Contract criteria

| Criterion | Status | Basis |
| --- | --- | --- |
| Bounded, source-matched echo/readback | Met | 3 s per operation (`:40`); source (1,1) plus a request cursor (`:130-132,147-149,157-159`); M06/M07/M08 killed; PX4 behaviour A2–A5 confirmed |
| Wrong/missing/stale echo fails | Met (ulp-exactness untested, finding 5) | `drop_echo`, `stale_echo`, `ignore_set` tests; foreign-component echo probe P12 |
| Actual fresh native yaw values saved | Partial | Booted and effective MC_YAWRATE_P/I/D/FF/K/MAX are read and returned in the record. Persisting it is the runner's job. Stale native-yaw detection works but is untested (finding 4). Roll/pitch K not recorded (finding 6) |
| Coherent finite state snapshots | Not met live | Fixtures pass, but the truth clock model contradicts PX4 (finding 1) |
| All freshness/skip/stall checks accept valid fixtures and reject mutants | Partial | Skip, stall, runner-late, non-finite, quaternion, order, increasing clock, source, missing/stale stream and truth reuse: met. Freshness bounds and align-time checks: not pinned (finding 3). The valid truth fixture is not PX4's stream (finding 1) |
| Failures abort, never an outcome | Met in module | Typed errors that latch; runner obligation in finding 12 |
| No new ground-truth policy input | Met | Truth is a separate snapshot field; no policy path in this module |
| 10 Hz simulator-time pacing kept | Met | LOCAL_POSITION_NED `time_boot_ms`, DT = 0.1 (`:36,298-303`); A8 confirmed |
