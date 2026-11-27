# REVIEW r2: dev08 parameter/telemetry boundary (`tools/sitl_dev08_link.py`, `tests/test_sitl_dev08_link.py`)

- **Reviewer:** independent reviewer, Claude Opus 5.5 (Agent model opus). Fresh context, read-only.
- **Date:** 2026-11-05. This is round 2; round 1 is `../review_r1/REVIEW_r1.md`.
- **Worktree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch camonly/dev, base 9051cea. Both
  files are untracked.
- **Reviewed bytes (sha256).** These match the brief and were re-checked at the end of the review.
  - `tools/sitl_dev08_link.py`: bf3ab740bbfbcec3f67a141157bfe3e15e1c96d260c6f3a7df03e5450dea43e3
  - `tests/test_sitl_dev08_link.py`: 862f3ee305c0f0bbfa20ce8f376ff30fe678c90ee5c43a979cd8df19ca7071fb
  - Copies are in `src/`; `reviewed_sha256.txt` lists the hashes.
- **Context read (not under review).** Another session was editing these during the review; hashes are as read.
  - `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md`: b8f7ef62…
  - `docs/evidence/sitl_2026-11-05/dev08/runtime_stack/README.md`: 60218874…
  - `tools/sitl_s3b_dev08_prefix.py`: 911a2229…, for how the gate is used.
- **PX4 tree:** `/home/fair/workspaces/px4_ws/PX4-Autopilot` at v1.14.3 (1dacb4c). There is no diff under
  `src/modules/{mavlink,mc_pos_control,ekf2}`, `src/lib/parameters` or `src/systemcmds/param`.
  - The installed airframe `build/px4_sitl_default/etc/init.d-posix/airframes/10090_gazebo-classic_navrl_quad` and
    its untracked ROMFS copy are identical (sha256 f7cc8cd5…).
  - `parameters.json` has sha256 050ff98d….

## Verdict: PASS_WITH_LIMITS (the code and tests), on one condition

The round-1 BLOCKER is repaired. I built PX4-shaped streams that follow PX4's stream scheduler and
HIL_STATE_QUATERNION.time_usec = 0, and passed them through real pymavlink encode/decode. The gate aligns and
serves every checkpoint for every relative stream phase I tried (10 phases × 200 checkpoints). Round-1 MAJOR 3 is
repaired: all five surviving freshness mutants (M18/M21/M22/M31/M35) and the M02/M11 parameter mutants are now
killed by tests that have positive controls at the bounds. MINOR findings 4–11 are repaired. MAJOR 2, the skip
bound, is recorded in amendment2 as a decision for the approval.

The parameter guard's fresh-boot expectation is correct for this build:
- all 27 recorded parameters are Float in parameters.json;
- their parameters.json defaults equal the `PARAM_DEFINE_FLOAT` defaults in the source;
- no boot script other than the airframe sets any of them;
- the airframe runs after the parameter import.

**Condition:** one new MAJOR finding (N1) is about description, not behaviour. The module, the tests and
amendment2 describe the truth sample as coming from the position sample's "same send burst" or "same PX4 cycle",
and amendment2 says a violation "shows as STALE_TRUTH". PX4's scheduler contradicts this. The gate neither
guarantees nor checks it. The 5 ms burst bound also admits the next 4 ms PX4 cycle. The gate is sound as a bounded
receipt-age proxy, and it records the skew at every checkpoint. The description must say that, not claim
same-cycle pairing, before amendment2 is committed or a smoke is approved. If the text is not corrected, treat the
truth-pairing claim as FAIL.

The remaining new findings are MINOR test-pinning gaps and one live-only robustness point (N2–N5), plus notes.

## Repair table (round-1 findings)

| # | R1 finding | Status | Evidence |
| --- | --- | --- | --- |
| 1 | BLOCKER: truth clock contradicted by PX4 (time_usec = 0) | **REPAIRED** | See the notes below this table |
| 2 | MAJOR: dev07 ep02 0.136 s late sample, so P(abort in 42) ≈ 0.88 | **RECORDED_AS_LIMIT** (diagnostics PARTIAL) | Amendment2 l.136-150 makes the bound a decision for the approval and states the 0.88 estimate. Diagnostics were added to CLOCK_STALL, CLOCK_SKIP and RUNNER_LATE (`:344-348, :391, :394, :398`), and `pos_lateness_s` is recorded per checkpoint. Two diagnostic weaknesses remain (N6) |
| 3 | MAJOR: freshness checks do not reject mutants | **REPAIRED** | Round-2 equivalents of all five are KILLED (see below) |
| 4 | MINOR: stale native yaw untested (M02) | **REPAIRED** | `tests:138-147` (native MC_YAWRATE_P 0.2 and MC_YAWRATE_FF 0.5 give PARAM_NOT_FRESH); M02 KILLED |
| 5 | MINOR: exact float32 echo untested (M11) | **REPAIRED** | `ulp_echo` fault `tests:93-96`, `:163-165`; M11 KILLED |
| 6 | MINOR: roll/pitch K/FF/MAX not recorded | **REPAIRED** | `RECORD_PARAMS` `:54-59` (27 names); `tests:133-134` |
| 7 | MINOR: fresh check covered airframe names only | **REPAIRED** | `firmware_defaults`/`fresh_expectations` `:103-123`; real-file test `tests:205-215`; mutants F1-F4 KILLED. Probe: all 27 are Float, and the parameters.json defaults equal the source `PARAM_DEFINE_FLOAT` |
| 8 | MINOR: no start cursor | **REPAIRED** | `start_cursor` `:270,278`; `Inbox.cursor_since` `:160-166`; test `tests:457-465` (negative and positive); C1/C2 KILLED. The integration uses `cursor_since(now − 1 s)` (prefix `:221`) |
| 9 | MINOR: checkpoint order not anchored; align repeatable | **REPAIRED** | ALIGN_TWICE `:352-353`; first k = 0 and k+1 `:376-377`; A1/A2/A3 KILLED |
| 10 | MINOR: rx-thread failure lost | **REPAIRED** | `Inbox.fail` `:150-154`; `wait_for` raises RxError (`:184-185`) and becomes ParamError RX_ERROR (`:217-218`); `pump` RX_ERROR `:319-320`; `rx_loop` latches `:436-444`; R1/R2/R3 KILLED; probe Q4 shows a dead rx thread reported as gate RX_ERROR. One path is untested (R4, N5) |
| 11 | MINOR: airframe parser drops bad values silently | **REPAIRED** | `:86-100` raises on any `param set-default` line it cannot parse; P1/P3 KILLED. Probe Q6: `-s`, hex and quoted values fail closed. "Last one wins" is untested (P2, N5) |
| 12 | NOTE: abort semantics depend on the runner | **REPAIRED** (module) | `BoundaryAbort` `:66-79`, `tests:600`. The integration records `operational_status=ERROR` and `error_code` (prefix `:148-151`); I did not review a runner-level test |
| 13 | NOTE: sample selection differs from dev07 | **RECORDED**, wording wrong | Amendment2 l.105-111 records it. Its truth wording ("same burst") is contradicted (N1) |
| 14 | NOTE: smaller points | Mostly **REPAIRED** | Echo wording weakened (`:9-10`). The shared send lock is documented (`:418-419`), and the integration passes `transport.lock` to the commander (prefix `:164`). The STREAM_STALE test uses the fake clock (`tests:443-444`). Unchanged and still fine: CLOCK_SKIP is implied by RUNNER_LATE; the Inbox keeps every message |

Notes on finding 1:
- **Code.** Truth carries no clock (`:299-300`). Selection is by receipt, the latest within
  [pos.recv − 0.03, pos.recv + 0.005] (`:325-329`). The burst wait is `_settle` (`:335-342`).
- **Tests.**
  - The valid fixture now has time_usec = 0 (`tests:243`), and the test asserts it (`tests:294`).
  - A source-pinned test, `tests:515-526`, asserts that `HIL_STATE_QUATERNION.hpp` does not assign `msg.time_usec`.
- **Probe.** `probe_px4_truth_r2.py` uses real pymavlink, time_usec = 0 and PX4 scheduling. Every phase aligns and
  serves 200/200 checkpoints (`probe_px4_truth_r2.out`). In round 1, align raised CLOCK_NOT_INCREASING.

Round-1 freshness mutants in round 2 (`mutate_r2.out`):

| R1 mutant | Round-2 site | Killed by |
| --- | --- | --- |
| M18 (`t < prev`) | `:312` | `test_clock_not_increasing` (duplicated time stamp, `tests:332-336`) |
| M21 (no STALE_ATT at align) | `:363-364` | `test_attitude_freshness_at_alignment` (0.04 s accepted, 0.08 s rejected) |
| M22 (no STALE_TRUTH at align) | `:365-367` | `test_truth_pairing_at_alignment` (truth received 0.4 s earlier) |
| M31 (truth age unbounded) | `:327` | `test_truth_age_bound` (0.02 s accepted, 0.04 s rejected) |
| M35 (attitude bound × 2) | `:332` | `test_attitude_freshness_bound` (0.04 s accepted, 0.08 s rejected) |
| M30 (attitude unbounded) | `:332` | the same tests |

## New findings

### N1. MAJOR: "same burst / same PX4 cycle" is contradicted by PX4's scheduler, is not checked, and the 5 ms bound admits the next PX4 cycle

**Text concerned.** These places claim same-burst or same-cycle pairing:
- `tools/sitl_dev08_link.py:22-23`: "assumes … that PX4 sends the streams together".
- `:49`, `BURST_S = 0.005  # … (same send burst)`.
- `:336-337`, the `_settle` docstring: "a same-burst truth sample is in".
- `tests/test_sitl_dev08_link.py:231` and `:297-302`: "pos, att, truth of one burst".
- Amendment2 l.107: "truth: the latest received in the same burst as the position sample".
- Amendment2 l.127-128, live assumption 3: "…the LOCAL_POSITION_NED sample of the same PX4 cycle. The truth time
  model rests on this. A violation shows as STALE_TRUTH."
- The evidence README, l.74-75: "truth the latest received in the position sample's burst".

**What PX4 does (v1.14.3 source):**
- **Each stream is scheduled on its own.** `MavlinkStream` keeps its own `_last_sent`.
  - The constructor initialises it to `hrt_absolute_time()` (`mavlink_stream.cpp:47-51`).
  - A stream sends when `dt > interval − 450 µs` and its `send()` sees a uORB update (`:108`, `:115-116`).
  - On an on-time send, `_last_sent += interval`, so each stream keeps its own phase.
- **SET_MESSAGE_INTERVAL changes only the interval of an existing stream.**
  - The path is `mavlink_receiver.cpp:2169-2203` → `configure_stream_threadsafe`
    (`mavlink_main.cpp:1193-1218`) → `check_requested_subscriptions` (`:2551-2572`) →
    `configure_stream` (`:1142-1183`).
  - LOCAL_POSITION_NED (30 Hz) and ATTITUDE (100 Hz) are onboard defaults created at boot (`:1450-1451`), so they
    keep their boot phase.
  - HIL_STATE_QUATERNION is not a default. It is created at the main-loop pass that handles the runner's third
    command and appended to the end of the list (`List.hpp:63-84`).
  - So the truth phase relative to the position is set by when that command is processed. It is fixed for one boot
    and differs between boots.
- **One mavlink pass per 4 ms lockstep step.**
  - `-r 4000000` (`px4-rc.mavlink:26`) gives a main-loop delay of 2 µs, which is clamped to 1500 µs
    (`mavlink_main.cpp:2159-2169`).
  - Gazebo steps every 4 ms (`empty.world:31,33`).
  - Within a pass, streams are updated in list order (`:2402-2403`), so a same-pass truth always follows the
    position.
  - Each message is its own `sendto` (`send_finish`, `:751-788`).
- **What a truth message carries.**
  - Its send is gated on the ground-truth topics, which update every physics step
    (`HIL_STATE_QUATERNION.hpp:67`; `gazebo_mavlink_interface.cpp:559,603`; `gazebo_groundtruth_plugin.cpp:168-169`).
  - LOCAL_POSITION_NED sends only on EKF output updates (`LOCAL_POSITION_NED.hpp:64-67`; `EKF2.cpp:681-684`).
  - ATTITUDE sends on every IMU sample (`ATTITUDE.hpp:66-73`; `EKF2.cpp:631-635`).

**Consequence**, from `probe_px4_truth_r2.out`, a generator that implements the rules above with real pymavlink:
- The truth used is a constant per-boot offset from the position sample: −12, −8, −4, 0 or +4 ms of PX4 time.
  Only one phase in five is the same cycle.
- At +4 ms, the truth is the **next** PX4 cycle. It is admitted because 4 ms < BURST_S = 5 ms.
- With ±1.5 ms receive jitter, 10 of 200 checkpoints in that phase flipped to the −16 ms truth. So which truth is
  selected depends on host timing within an episode.
- In that phase the gate also departs from the historical runner. That loop polled every 2 ms
  (`sitl_s3b_closed_loop.py:265-266`) and read `L.truth` straight away, so it would normally have used the −16 ms
  truth.
- No cycle mismatch produces STALE_TRUTH. Amendment2's "a violation shows as STALE_TRUTH" is therefore false.
  STALE_TRUTH fires only when no truth was received within the window.

**Why it matters.** The behaviour is fail-closed and bounded. At RTF ≈ 1 the sim-time offset is about −20 to
+5 ms, and `truth_recv_skew_s` is stored per checkpoint. The text, however, claims a pairing that is neither
provided nor checked, in a preregistration amendment that becomes immutable once committed.

**Fix:**
1. **Text** (module, tests, amendment2 l.107 and l.127-128, README l.74-75). Describe the truth sample as "the
   latest received within [−0.03, +0.005] s of the position sample's receipt (STALE_TRUTH if none)". Say which PX4
   cycle it comes from is not checked. By PX4's scheduler it is the position's cycle or up to one stream period
   earlier, and it can be one 4 ms cycle later under the 5 ms bound. The phase is fixed per boot by when the
   stream-interval command is handled. Have the smoke report the per-episode distribution of `truth_recv_skew_s`.
2. **Optional code change.** If "same cycle or earlier" is the intent, set BURST_S below one 4 ms step, for example
   0.002–0.003 s. This also matches the historical 2 ms poll more closely. Pin it with a test at the bound (N2).

### N2. MINOR: the burst wait and the BURST_S value are not pinned by any test

Seven mutants survive the whole test file (`mutate_r2.out`):

| Mutant | Change |
| --- | --- |
| S1 | `_settle` removed from `checkpoint` |
| S2 | `_settle` removed from `align` |
| S3 | `end = pos.recv` |
| S4 | the settle loop does not pump |
| T3 | BURST_S = 0.015 |
| T3b | BURST_S = 0.0035 |
| T3c | BURST_S = 0.019 |

The fixtures always ingest the truth before the gate pumps, and they place it only at +0.2 ms or +20.2 ms.
`survivors_r2.out` shows what each mutant does differently:
- A same-cycle truth arriving 10 ms of wall time after the gate starts waiting:
  - the original selects it (+0.2 ms);
  - S1/S3/S4 select the previous truth (−19.8 ms);
  - S2 fails closed at align (STALE_TRUTH).
- A truth received 4 ms after the position: the original selects +4.0 ms; T3b selects −16.0 ms.
- A truth received 10 ms after the position: the original selects −10.0 ms; T3/T3c select +10.0 ms.

Fix:
- Add a test in which the truth is delivered after `checkpoint()` and `align()` have begun waiting (a thread, as in
  `survivors_r2.py:s_settle_checkpoint`).
- Add an inside/outside pair at the chosen burst bound.

### N3. MINOR: a non-finite truth quaternion is not tested (contract: "coherent finite state snapshots")

Q1 (`values += q` removed, `:302`) and Q3 (NONFINITE skipped for truth, `:309`) survive. Under either mutant, a
NaN `attitude_quaternion` is accepted silently: BAD_QUATERNION requires all-finite components (`:303`), and the
truth's other fields are integers. The original raises NONFINITE (`survivors_r2.out`). The quaternion is the
truth's only float field, and the truth feeds termination and the command yaw.

Fix: add a NaN-quaternion case to `test_nonfinite_and_bad_quaternion`, which already has a valid control.

### N4. MINOR (live-only): the receipt window does not bound the PX4-time offset if the rx thread batches messages

Receipt time is stamped by `Inbox.put` in the rx thread (`:145`), not at socket arrival. If that thread is starved
(GIL, host load), several PX4 cycles are stamped within a millisecond. A truth from a later cycle then falls
inside +5 ms of an earlier position and is selected (max seq, `:329`). RUNNER_LATE does not fire while the newest
position is within 0.05 s.

Probe (`probe_px4_truth_r2.out`, rows "rx stall"):

| rx stall per 0.1 s | Truths newer than the position, accepted without error |
| --- | --- |
| 30 ms | +8 ms in 8 of 100 checkpoints |
| 45 ms | +16 to +28 ms in 8 of 100 checkpoints |

The main thread is blocked in `wait_new` when the selected position arrives, so this needs an unusual stall. It is
still undisclosed.

Fix (either):
- add an order bound, `truth.seq < seq of the first position sample newer than the selected one` when one exists;
- or fail when the selected position and its successor are stamped much closer together than their PX4 times.

Also list rx-thread promptness among the live assumptions.

### N5. MINOR: three parameter-path rules are unpinned (each confirmed in `survivors_r2.out`)

- **F5.** The fresh check is exact (`:248`), but a 1e-6 tolerance survives. A booted value one float32 ulp from
  the expectation gives PARAM_NOT_FRESH in the original and is accepted by the mutant. This is the same exactness
  that M11 pins for the echo.
- **P2.** "Last one wins" (`:90`, `:99`) is untested: first-one-wins survives. With duplicated set-default lines,
  the original reads 0.12 and the mutant reads 0.11.
- **R4.** A receiver failure that occurs *during* a parameter wait is untested. The original wakes and reports
  RX_ERROR. The mutant (no wake on error, `:174`) waits the full timeout and reports PARAM_MISSING.

Fix: add three small tests.

### N6. NOTE: diagnostics

- `inbox_backlog` (`:347`) is computed just after `pump()`, so it is always about 0 (probe Q3). It says nothing
  about the socket queue that round-1 finding 2 wanted, to tell a runner stall from a PX4 stall. Socket depth
  needs, for example, the SIOCINQ ioctl. The `(t, recv)` pairs are the useful part.
- `positions_near[:20]` (`:348`) keeps the **oldest** 20 samples within ±0.3 s. In probe Q3 the RUNNER_LATE
  message lists positions only up to t_k + 0.1 s while the newest was t_k + 0.2 s, so the samples that show the
  lateness are cut off. Keep the samples nearest t_k, or the last 20.

### N7. NOTES: smaller points

- **Malformed message (theoretical).** A stream message with a missing field raises a plain KeyError from `_ingest`
  (`:298`). It is not a BoundaryAbort and is not latched (probe Q1). Decoded pymavlink messages always carry every
  field, so this cannot happen with the real link. Wrapping it as `TelemetryError("MALFORMED")` would keep the
  error contract uniform.
- **Second align after an error.** `align()` on a gate that has already latched an error raises ALIGN_TWICE rather
  than the latched cause (probe Q2). Harmless.
- **Comment.** `SKIP_MAX_S` says the first sample is ≤ 0.02 s late (`:46`). dev07 shows 0.024 s on the 4 ms grid
  (EKF output cadence) and 0.040 s at most.
- **TRUTH_AGE_S pinning and the RTF floor.** TRUTH_AGE_S is pinned only to ±0.01 s: T6 (0.039) and T6b (0.021)
  survive, which is acceptable. At RTF ≤ about 0.5 and an unlucky phase, align fails with STALE_TRUTH (probe:
  RTF 0.5, phase +16). This is consistent with the "RTF ≈ 1" assumption; record the floor.
- **Equivalent mutants.** T8 (nearest-receipt instead of latest truth) cannot differ at 50 Hz, because the window
  is 35 ms wide and samples are 20 ms apart. C3 (`cursor_since` off by one) only admits one more old message.
- **The reuse check is redundant.** With the default bounds, a repeated truth is outside the window before the
  reuse check runs (probe Q5). It is kept and tested with a widened window. Fine.
- **Parameters PX4 writes at boot.** mc_pos_control's boot-time constraints write MPC_XY_CRUISE, MPC_VEL_MANUAL
  and MPC_Z_V_AUTO_UP (`MulticopterPositionControl.cpp:170-245`), because the airframe lowers MPC_XY_VEL_MAX and
  MPC_Z_VEL_MAX_UP.
  - None of these is in RECORD_PARAMS, so there is no PARAM_DRIFT.
  - Offboard velocity control does not use them.
  - Both arms share them.
  - The gates that would write recorded names default to inactive: SYS_VEHICLE_RESP −0.4, MPC_XY_VEL_ALL −10 and
    MPC_Z_VEL_ALL −3 (`:89-136`). The MPC_TILTMAX_AIR clamp and the MPC_THR_HOVER clamp (to [0.12, 1]) are not
    triggered by 45 and 0.31. mc_autotune writes gains only after MC_AT_START (`mc_autotune_attitude_control.cpp:273,
    551-583`).
- **Tests are non-vacuous and robust.**
  - Every rejection test has a positive control at or near the bound.
  - Telemetry tests use the fake receive clock.
  - All waits are bounded: `_settle` ≤ 55 ms of wall time, stall bound 0.2 s, and `test_live_feed_thread` joins a
    producer that checks its stop event every 2 ms.
  - Five runs gave 32/32 OK in about 4 s each. One of them ran with four busy loops pinned to the same cores.
  - The file does not touch `sys.modules`, `os.environ` or `sys.path`.

## PX4 assumption table (updated)

| # | Assumption | Status | Source (v1.14.3) / evidence |
| --- | --- | --- | --- |
| A1 | Autopilot is (1,1) | Confirmed (R1) | `rcS:131`; `mavlink_params.c:41,50` |
| A2-A5 | PARAM_SET echo, PARAM_REQUEST_READ by name, type codes, unsolicited PARAM_VALUE | Confirmed (R1, unchanged) | `mavlink_parameters.cpp:102-136,175-202,358-407,481-534` |
| A6 | A fresh boot reports the airframe set-default, else the firmware default | **Confirmed** for this build | Detailed after the table |
| A7 | A reused parameters.bson shows as booted ≠ expectation | Confirmed | Import precedes the airframe; both arms override airframe-defaulted MC_ROLLRATE_P (0.025 → 0.013); test `tests:139` |
| A8 | LOCAL_POSITION_NED/ATTITUDE `time_boot_ms` on PX4's clock, strictly increasing per stream | Confirmed (source); live: dev07 | `LOCAL_POSITION_NED.hpp:64-67`, `ATTITUDE.hpp:66-73` (send only on a uORB update; timestamp/1000); `EKF2.cpp:631-635,681-684` |
| A9 | HIL_STATE_QUATERNION.time_usec is 0 | **Confirmed**; the gate no longer relies on a truth clock | `HIL_STATE_QUATERNION.hpp:80-105` (`msg{}`, no assignment); source-pinned test `tests:515-526` |
| A10 | Truth from (1,1) carries ground truth | Confirmed (R1) | `HIL_STATE_QUATERNION.hpp:60-63`; `SimulatorMavlink.cpp:516-602` |
| A11 | 50 Hz after SET_MESSAGE_INTERVAL 20000 µs | Mechanism confirmed; rates live-only | `mavlink_receiver.cpp:2169-2203` → `configure_stream` `mavlink_main.cpp:1142-1183`; the scheduler gives exact 20 ms for ATTITUDE and truth (topics update every 4 ms step); position depends on the EKF output cadence (dev07: 20/24 ms) |
| A12 | Streams are sent "together" / in the same burst | **Contradicted** | Independent `_last_sent` per stream (`mavlink_stream.cpp:47-51,108-116`); truth created later by the runner's command, so its phase is set per boot (`List.hpp:63-84`, `mavlink_main.cpp:1450-1451`). Probe: offsets −12/−8/−4/0/+4 ms (N1) |
| A13 | Within one pass, order is LOCAL_POSITION_NED → ATTITUDE → … → HIL_STATE_QUATERNION | Confirmed (source) | Onboard default order `mavlink_main.cpp:1450-1451`; truth appended (`List::add`); loop `:2402-2403`. So a same-pass truth always follows its position |
| A14 | Receive order = send order (one UDP socket, one datagram per message) | Confirmed (source); loopback ordering live-only | `send_finish` `mavlink_main.cpp:751-788` (`sendto` per message) |
| A15 | One mavlink pass per 4 ms simulator step | Confirmed (source) | `-r 4000000` (`px4-rc.mavlink:26`) gives a 1500 µs clamp (`mavlink_main.cpp:2159-2169`); `px4_usleep` on lockstep time (`:2207`); `empty.world:31,33` |
| A16 | Receipt time tracks PX4 time (RTF ≈ 1, prompt rx thread) | **Live-only** | dev07 RTF 0.996–1.004. Probe: RTF ≤ 0.5 with phase +16 gives STALE_TRUTH; a 30–45 ms rx stall admits truths 8–28 ms newer silently (N4) |
| A17 | The truth content is from the cycle it was sent in | Live-only | The ground-truth pose reaches the mavlink interface through asynchronous Gazebo transport (`gazebo_groundtruth_plugin.cpp:120,168-169` → `gazebo_mavlink_interface.cpp:428,817-821`); a lag of up to one step is possible |
| A18 | All 27 recorded names are REAL32 with parameters.json default = source default | **Confirmed** | `probe_params_json.out` (all "Float"; equal to `PARAM_DEFINE_FLOAT` in `mc_rate_control_params.c`, `mc_att_control_params.c`, `mc_pos_control_params.c`, `motor_params.c`) |
| A19 | No runtime writer changes the 27 before readback | Confirmed for defaults; live-only otherwise | N7, "Parameters PX4 writes at boot" |
| A20 | pymavlink field shapes (`param_id` str, quaternion list) | Confirmed (R1, P1); round 2 uses real encode/decode | `probe_px4_truth_r2.py` |

A6 in detail:
- **rcS order.** The rcS parameter import runs at `rcS:67-102`. The SYS_AUTOCONFIG reset is at `:110-127`.
  rcS's own set-defaults (`:154-188`) include none of the 27. The airframe is sourced at `:225-227`.
- **Other boot scripts.** `px4-rc.params` is all comments. `px4-rc.simulator` sets only IMU_INTEG_RATE and the
  SIH/GZ names. `rc.vehicle_setup` sources `rc.mc_apps`, which only touches SYS_MC_EST_GROUP and MC_AT_EN. There
  is no `.post` file.
- **Inside the airframe.** It sources `rc.mc_defaults` (none of the 27) before its own lines.
- **Grep.** Across the installed `etc/`, only `rc.vtol_defaults` sets any of the 27, and it is not sourced for a
  multicopter.
- **set-default semantics.** `param set-default` parses with `strtod`→float and applies only if
  `|Δ| > FLT_EPSILON` (`param.cpp:870-875`). The only airframe value within ε of the firmware default is
  MPC_TILTMAX_AIR 45 = 45 (`probe_params_json.out`), which is equal anyway.

## Is the truth time model sound and honestly described? (question 2)

- **Sound as a bounded proxy, yes, at RTF ≈ 1 with a prompt receiver.**
  - **Bounds.** The latest truth with receipt in [−30, +5] ms of the selected position's receipt corresponds to
    a PX4-time offset of about [−20, +5] ms.
  - **Ordering.** It does not depend on any truth clock, and within a pass the truth always follows the position
    (A13, A14).
  - **Failure mode.** Silence or excessive age fails closed (STALE_TRUTH), and every checkpoint records
    `truth_recv_skew_s`.
- **0.03 s is reasonable at 50 Hz.** The truth is sent exactly every 20 ms, so the newest truth not after
  +5 ms is at most about 15–20 ms old in PX4 time. The window leaves a margin of 1.5 periods. The floor is about
  RTF 0.5 (A16).
- **0.005 s is too wide for the stated meaning.** Same-pass messages are microseconds apart, while the next PX4
  cycle is 4 ms later. So 5 ms admits a next-cycle truth, selected or not depending on host jitter (N1).
- **If PX4 sends the truth just after the window,** the previous truth (about 16–20 ms older) is used, accepted
  and recorded with a negative skew. If the receipt is delayed past the window, the same happens. Nothing aborts
  unless RTF falls to about 0.5.
- **Not honestly described yet.** "Same burst / same PX4 cycle / a violation shows as STALE_TRUTH" is contradicted
  by PX4 (A12) and is not what the gate checks (N1).
- **Live assumptions that remain:**
  - RTF ≈ 1 (A16);
  - receipt promptness (A16, N4);
  - truth content lag (A17);
  - 50 Hz delivery (A11);
  - loopback ordering (A14).

## Contract criteria (amendment1 row "Parameter/telemetry boundary")

| Criterion | Status |
| --- | --- |
| Bounded, source-matched echo/readback | Met (R1); exact echo now pinned (M11 killed) |
| Wrong/missing/stale echo fails | Met; ulp echo tested |
| Actual fresh native yaw values saved | Met in the module: the booted and effective values of all 27 are returned. Stale native yaw is tested (M02 killed). Persisting them is the runner's job; the ledger has a `parameters` field (amendment2 l.89) |
| Coherent finite state snapshots | Met with limits: they are served live-shaped (BLOCKER repaired). Coherence is bounded by receipt age, not same-cycle (N1, N4). Non-finite truth is untested (N3) |
| All freshness/skip/stall checks accept valid fixtures and reject mutants | Met for every round-1 check. New gaps are the burst-wait selection mechanism and the BURST_S value (N2), and truth NONFINITE (N3) |
| Failures abort, never an outcome | Met in the module (BoundaryAbort, latched); the integration records ERROR |
| No new ground-truth policy input | Met: truth stays a separate snapshot field |
| 10 Hz simulator-time pacing, historical comparison | Met: `not t < start + k*DT` (`:378,383`); K1/K2 killed; `test_historical_float_comparison` |

## What I ran (all outputs in `/home/fair/motar_tmp/dev08_link_20261105/review_r2/`)

All runs used `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `nice -n 10`, `taskset -c 6,7,12,13` and a timeout.
No socket was opened, no PX4 or Gazebo was started, no GPU was used, and no repository or PX4 file was written.

- `unittest_run.out`: the repository command given in the brief, 32 tests, OK in 4.0 s.
  - Three more runs were OK.
  - One more run, with four busy loops on the same cores, was OK in 4.3 s.
- `probe_params_json.py` / `.out`: `fresh_expectations` on the real airframe and parameters.json; type and default
  of the 27 against the source `PARAM_DEFINE_FLOAT`; ε-skip check.
- `probe_px4_truth_r2.py` / `.out` / `.json`: the round-1 truth probe rebuilt on PX4's scheduler, with real
  pymavlink encode/decode and time_usec = 0. Variants:
  - 10 stream phases × 200 checkpoints;
  - receive jitter 0.5 and 1.5 ms;
  - RTF 0.8, 0.6, 0.5 and 0.4;
  - rx stalls of 15, 30 and 45 ms.
- `mutate_r2.py` / `mutate_r2.out` / `mutants_r2.json` / `mut/*`: 54 single-site mutants against the unmodified
  test file.
  - **38 killed**, including every round-1 survivor (M02, M11, M18, M21, M22, M31, M35) and the new
    truth-window, start-cursor, order, RX_ERROR, parser and fresh-expectation mutants.
  - **16 survived:**
    - real gaps: T3/T3b/T3c, S1–S4, Q1, Q3, R4, P2 and F5 (N2, N3, N5);
    - weak but acceptable pin: T6/T6b;
    - equivalent or harmless: T8, C3.
- `survivors_r2.py` / `.out` / `.json`: for every survivor, a scenario on which the original and the mutant
  differ, with positive controls.
- `probe_semantics_r2.py` / `.out`: Q1 malformed message, Q2 align after a latched error, Q3 RUNNER_LATE
  diagnostics, Q4 rx-thread death surfacing as RX_ERROR, Q5 default-bound truth silence, Q6 parser forms.
- Read-only source reading of the PX4 tree, pymavlink, the historical runner, the integration module, amendment2
  and the evidence README, at the lines cited.
- `src/`: the reviewed copies. `reviewed_sha256.txt`: their hashes.
