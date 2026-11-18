# Unit B r1 (2026-11-18): PX4 vertical fields gateway -> camera policy node, and a bounded refusal hold

Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD
`fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde` (unchanged). Edited on top of the 26 uncommitted paths of the outdoor
runtime work (snapshot `preB_tree/`, all 26 verified byte-equal to `preB_tree/SHA256SUMS` before the first edit).
No git state change, no ROS / SITL / Gazebo / GPU / camera execution, no package install. Author: Claude Code
(Opus 5.5) as a delegated worker. Usage: USAGE_UNKNOWN (`tools/agent_session.py status`: no provider signal).

**Status: IMPLEMENTED, CPU tests PASS (see Tests). No independent review has been run on this unit; treat it as
CONDITIONAL until an Opus review audits the diff. Nothing here is a flight qualification or an outdoor GO.**

## 1. Goal 1 design: forwarding z_valid / epv / dead_reckoning

### Chosen: a versioned private encoding inside the gateway's own Odometry

The gateway (`tools/camera_ros2_px4_node.py`) already publishes a private `nav_msgs/Odometry` on
`/motar/state/odometry` whose 72 covariance values were all NaN ("unavailable, private policy input, must not feed an
estimator"). Unit B writes PX4's three fields into that same message:

| Odometry slot | Content |
| --- | --- |
| `pose.covariance[0]` | tag -20261118.0 (`camera_ros2_adapter.VERTICAL_ENCODING_TAG`; a negative variance, never a valid covariance) |
| `pose.covariance[1]` | `z_valid`, 1.0 / 0.0 |
| `pose.covariance[2]` | `epv` [m], finite, >= 0 |
| `pose.covariance[3]` | `dead_reckoning`, 1.0 / 0.0 |
| all other `pose.covariance`, all `twist.covariance` | NaN |

Encoding name `motar-px4-vertical-v1` (`camera_ros2_adapter.VERTICAL_ENCODING`). Fields absent or malformed at the
gateway: every covariance NaN, i.e. byte-for-byte the pre-unit-B message.

Why this design:
- **Atomic by construction.** The fields travel in the very message whose pose the policy uses; the camera node pairs
  nothing by time. The fields come from the `VehicleLocalPosition` sample that `decode_estimator` already pairs with
  the `VehicleOdometry` sample in the same snapshot to gate pose validity (`|t_odom - t_lpos| <= 0.025 s`, both fresh);
  that intra-gateway pairing existed before unit B (it gates `xy_valid`/`z_valid` today) and is not new.
  `BridgeSession._state` decodes pose and vertical fields from one `snapshot()` dict, and `poll()` returns them in one
  state dict that `_publish_odometry` writes into one message.
- A dropped DDS message drops pose and fields together (no partial pairing).
- No new topic, QoS, freshness watchdog, message package or rosidl build (installs were out of scope anyway).
- Strict decoder: FORWARDED needs the exact tag, NaN in all 68 other slots and exact 0/1 flags, so an arbitrary
  producer's real covariance can never decode as fields; and the decoder is only consulted under an explicit binding
  declaration (below).
- Versioned: any future change of the gateway's covariance use needs `-v2` and a new declaration.

Rejected alternatives:
1. **Separate private topic paired by exact stamp equality** (e.g. a `geometry_msgs/Vector3Stamped` or similar
   carrying the three values). Still a private float encoding of booleans (no standard typed message fits), plus a
   fifth input queue in the session, an exact-stamp lookup, a fifth freshness watchdog and new failure modes: with the
   BEST_EFFORT depth-5 sensor QoS a single dropped vertical message refuses a frame whose pose arrived, and DDS has no
   cross-topic ordering.
2. **Custom typed `.msg` (an interface package)**: needs a new ament_cmake interface package build and install (out
   of scope, "no package installs"), and would still need pairing if published separately.
3. **Nearest-in-time pairing**: explicitly excluded (race).
4. **Camera node subscribing to `/fmu/out/vehicle_local_position` itself**: needs `px4_msgs` in the policy node,
   pairs a different PX4 message by time with the gateway's odometry, and breaks the "gateway owns PX4 decoding"
   boundary.
5. **Encoding in `frame_id` / `child_frame_id` strings**: those are validated frame identifiers.

### Explicit declaration (camera node)

The binding (`camera_ros2_adapter.validate_binding`) accepts one optional key `odometry_source`, which must equal
exactly `{"producer": "motar_px4_gateway", "vertical_encoding": "motar-px4-vertical-v1"}`
(`gateway_odometry_source()`); any other value is refused. Bindings without it (every existing one, the template and
`config/unconfigured.json`) are unchanged. `camera_ros2_node.load_node_config` refuses an outdoor contract without the
declaration as `UNCONFIGURED: outdoor_vertical_fields_not_forwarded: ...` (the existing named reason; the existing
test that asserts it is untouched and passes). `ROS_PATH_FORWARDS_VERTICAL_FIELDS` stays `False` and now documents the
UNDECLARED path (an existing test asserts it). `main(check_only=True)` reports `profile`, `odometry_source` and the
PROVISIONAL `refusal_hold_max_s` for an outdoor configuration.

### Fail closed

- Gateway: `camera_ros2_px4.vertical_fields(local_position)` validates (booleans, finite real `epv >= 0`); missing or
  malformed -> nothing forwarded (all NaN) with the problem in the gateway diagnostics (`vertical_fields`). The gateway
  does not judge `epv`/`dead_reckoning` values (the outdoor contract does) and still revokes on `z_valid` false
  (`decode_estimator`, unchanged).
- Camera side: `build_frame(..., vertical_encoding=...)` (passed by the session only for an outdoor-profile runtime
  with the declaration) decodes the Odometry message(s) used for the pose: the nearest sample, or both samples of an
  interpolation bracket, combined conservatively (`z_valid` only if both, larger `epv`, `dead_reckoning` if either).
  FORWARDED -> the three fields join the state; ABSENT / MALFORMED -> no field is attached, so the runtime refuses with
  its existing `estimator_vertical_fields_absent` (zero hold). Forwarded bad values give `estimator_z_invalid`,
  `estimator_epv_exceeds_limit`, `estimator_dead_reckoning`. No default is ever substituted.
  `frame['vertical_provenance']` and the session diagnostics (`vertical_fields`) record status and detail.
- Indoor sessions never pass `vertical_encoding`, so they never read the covariance (state stays exactly 5 fields).
- `build_frame` refuses `vertical_encoding` without the matching binding declaration.

### PX4 message checks

`PX4_MESSAGE_SHA256` is unchanged (a test pins the literal pre-unit-B dict). New
`camera_ros2_px4.READ_FIELD_TYPES` lists the generated type of every field the gateway reads (incl. `epv: float`,
`dead_reckoning: boolean`); `validate_generated_type(cls, definition, required)` checks them on the installed classes
at `Px4GatewayNode` construction, after the existing full .msg-vs-class comparison. A test confirms the table against
the local PX4 v1.14.3 `.msg` sources (SHA-checked), and that a build whose `.msg` and class agree with each other but
declare `epv` as float64 or `dead_reckoning` as uint8 is refused.

## 2. Goal 2 design: bounded refusal hold (PROVISIONAL)

- **Episode** (`camera_policy_runtime.RefusalHoldEpisode`, shared by the session and both replays): consecutive ticks
  whose proposal is the explicit zero hold (frame refusal or re-entry hold) without an intervening policy command.
  A refusal inside an episode never restarts it. A tick that proposes nothing (`WAITING_ALTITUDE`) neither extends
  nor ends it (its time still passes on the stamp clock). Only a policy command (any non-hold proposal, including an
  exactly-zero policy command) or `reset()` ends it.
- **Limit**: `ground_estimate.refusal_hold_max_s = {"value": 3.0, "status": "PROVISIONAL"}` in the outdoor contract,
  bounds [1.0, 10.0] s inclusive (new `INCLUSIVE_LOW_BOUNDS` in `camera_policy_contract.py`; every other bound keeps
  its (low, high] form). Defaults and bounds are the suggested ones, unchanged. 3.0 s = 30 trained ticks, six re-entry
  holds; not derived from a measurement. Placed in `ground_estimate` (the block that owns the refusals) so existing
  outdoor-contract helpers and the "every dict-valued threshold is PROVISIONAL" test pick it up; the README threshold
  table gained the row (its test passes).
- **Clock: capture stamps** (`min(item.stamps)`, the proposal's `capture_stamp_s`). Justification: every proposal
  carries one; they strictly increase tick to tick (offer drops non-increasing samples, prepare takes only newer
  ones); each is checked within 0.15 s of the ROS clock at prepare and again at finish; the ROS clock is checked
  never backwards and never paused against monotonic time, and the 10 Hz period is enforced on it, so ticks cannot be
  skipped silently; the replays have the same stamps and no receipt clock, so they compute the identical episode;
  under simulated time this is the vehicle's own time base (a monotonic measure would fire at a different vehicle
  time). Elapsed = this hold's stamp - the episode's first hold stamp; fires at `elapsed >= limit - 1e-6 s` (the
  tolerance absorbs ns-rebuilt float sums, e.g. 6.6 - 3.6 = 2.9999999999999996).
- **Latch**: at the limit `finish()` proposes nothing and raises `HoldTimeout` (a `SessionFault` subclass); the
  session latches fault `refusal_hold_timeout: ...` (status `FAULT`, runtime invalidated). No automatic re-entry:
  `prepare()`/`watchdog()` keep raising the fault until an explicit `reset()` (the node's `~/reset_policy_session`).
  Held proposals span less than the limit (30 holds at 0.0 .. 2.9 s, the 3.0 s tick latches).
- **Hand-off to PX4**: no new vehicle command. The gateway receives no fresh command; its existing stale-command path
  revokes (tested: revoked 0.2 s after the last held proposal at 10 Hz), PX4 keeps the last setpoint (the zero hold)
  until `COM_OF_LOSS_T`, then `COM_OBL_RC_ACT`, and the pilot takes over.
- **Indoor**: `refusal_hold_limit_s(indoor) is None`; indoor sessions never hold (110 ticks = 11 s > the 10 s upper
  bound: all commands, no timeout). A hold in a session without a limit (impossible from a production indoor runtime)
  faults instead of holding unbounded.
- **Diagnostics**: `RosPolicySession.diagnostics()` (now used by the node; all previously published keys kept, plus
  `fault`) adds `hold_episode_s`, `hold_limit_s`, `hold_limit_status`, `hold_timeout` (reason, episode_s, limit_s,
  limit_status, first/last capture, held_proposals, refused_frames, last_refusal, epoch, sequence),
  `vertical_encoding`, `vertical_fields`. Proposals that hold carry `hold_episode_s`.
- **Replays** (report-only; they keep processing after the frame): `tools/oakd_outdoor_replay.py` rows carry
  `hold_episode_s` and `refusal_hold_timeout: true` on the frame where the session would latch; the summary has
  `refusal_hold_max_s`, `longest_hold_episode_s`, `refusal_hold_timeout` ({index, t, episode_s} or null). The NPZ
  `camera_policy_runtime.replay` (outdoor only) marks the line and adds `refusal_hold_max_s`,
  `longest_hold_episode_s`, `refusal_hold_timeout_capture_s` to its report; the indoor report is unchanged (existing
  equality test passes).

## 3. Changed files

| File | Why |
| --- | --- |
| `tools/camera_ros2_adapter.py` | encoding spec (tag, slots), `gateway_odometry_source`, optional `odometry_source` binding key, encoder/decoder, `build_frame(vertical_encoding=)` attaching fields from the pose's own Odometry message(s) |
| `tools/camera_ros2_session.py` | passes the encoding for declared outdoor sessions; hold episode, `HoldTimeout` latch, `diagnostics()`; docstring supersedes "NO time limit" |
| `tools/camera_ros2_node.py` | outdoor contract needs the binding declaration (existing refusal name kept); publishes `session.diagnostics()`; check output |
| `tools/camera_ros2_px4.py` | `VERTICAL_FIELDS`, `READ_FIELD_TYPES`, `vertical_fields()` validation |
| `tools/camera_ros2_px4_node.py` | `BridgeSession._state` attaches the same snapshot's fields; `odometry_covariances`; `_publish_odometry` writes them; generated-type check of read fields; diagnostics; adapter added to the source-root check |
| `tools/camera_policy_runtime.py` | `REFUSAL_HOLD_TIMEOUT`, `refusal_hold_limit_s`, `RefusalHoldEpisode`; NPZ `replay` reports the would-be timeout (outdoor only) |
| `tools/camera_policy_contract.py` | `ground_estimate.refusal_hold_max_s` PROVISIONAL 3.0 s, bounds [1.0, 10.0] (`INCLUSIVE_LOW_BOUNDS`) |
| `tools/oakd_outdoor_replay.py` | per-row `hold_episode_s`, timeout frame and summary fields |
| `ros2/motar_camera_policy/README.md` | outdoor section only: threshold row, forwarding/encoding/declaration text, bounded hold text (pre-outdoor prefix untouched) |
| `docs/evidence/sitl_2026-11-05/camera_runtime/README.md` | appended section "Unit B, 2026-11-18" (earlier text untouched) |
| `tests/test_camera_runtime_unit_b.py` (new) | 28 tests below |

Not changed: every existing test file (all 16 byte-equal to `preB_tree`), `ros2/.../config/*.json`, `package.xml`,
`oakd_camera.py`, PX4 `.msg` bindings, geofence logic, indoor code paths. README topic table (pre-outdoor prefix) was
deliberately NOT edited: `OutdoorRecords.test_dated_sections_append_without_changing_earlier_text` pins that prefix by
SHA-256, so the topic payload is documented in the outdoor section instead.

## 4. Tests

New file `tests/test_camera_runtime_unit_b.py`, 28 tests (each negative paired with a positive control asserting a
nonzero count):
- Gateway: message SHA bindings unchanged (literal); `vertical_fields` 4 accepted / 11 malformed refused; BridgeSession
  forwards the same snapshot's fields (covariance layout) vs. pre-unit-B fixture all-NaN; gateway still revokes on
  `z_valid` false (control publishes); generated-type check refuses float64 `epv` / uint8 `dead_reckoning` (1 accepted);
  `READ_FIELD_TYPES` matches the SHA-checked local PX4 sources (27 fields).
- Encoding: 3 round trips FORWARDED, ABSENT, 11 MALFORMED variants (other version tag, filler, twist, flags, epv
  negative/inf/NaN, short array, a real covariance, missing attribute), encoder refuses 5 bad inputs.
- Binding: declaration optional/exact/versioned (6 bad values refused); fields reach the state only with it; undeclared
  or wrong-version request refused; bracket combination (worst case) and one-absent bracket.
- Session + production outdoor preprocess (no checkpoint): full CPU chain PX4 mappings -> BridgeSession -> Odometry
  fields -> declared session -> production `estimate_ground`: 8/8 frames accepted with the gateway's values; paired
  negative (fixture without epv/dead_reckoning): 8/8 refused `estimator_vertical_fields_absent`. Matrix: forwarded
  accepted (3 frames) vs undeclared / absent / malformed (`estimator_vertical_fields_absent`), `z_valid` false,
  `epv` 0.9, dead reckoning (their named reasons, zero holds). Indoor session with a declared binding never reads them.
- Node: real seed-1 outdoor contract loads only with the declaration (CONFIG_VALID, check output); without it
  `UNCONFIGURED: outdoor_vertical_fields_not_forwarded` / exit 2; wrong version refused; indoor loads either way.
- Hold: contract default/bounds (6 accepted incl. 1.0 and 10.0; 12 bad + missing refused by contract validation and by
  `refusal_hold_limit_s`); session refuses an outdoor contract without the field; fires at 3.0 s and not before
  (30 holds), also at the bounds 1.0 s (10) and 10.0 s (100); latch needs `reset()` (3 faulted ticks, then commands);
  intermittent refusals every 3rd frame time out (30 holds, 11 refusals) vs control with commands in between (no
  fault); a policy command ends the episode (2.4 + 0.5 s then restart from 0); gated ticks do not end it (latch at
  3.0 s capture difference); indoor 110 ticks never hold; a limit-less hold faults; gateway revokes 0.2 s after the
  last held proposal on its stale path; `RefusalHoldEpisode` unit rules.
- Replays: NPZ replay marks line 30 (3.0 s) vs control (none, longest < 1 s); OAK-D replay (synthetic folder, limit
  1.0 s, `epv` 0.9) marks index 10 (t 11.0) vs accepted control (none, longest 0.4 s).
- Records: README encoding/declaration/hold terms and numbers tested against the code constants; evidence section
  appended after the pinned prefix.

Mutation checks (process-local patches, no file changed; `logs/unitB_mutation_checks.log`): 7/7 mutants killed
(never times out; refusal restarts episode; session ignores declaration; node trusts any odometry; bracket uses first
sample only; malformed decoded as default; no required-type check).

Results:

| Run | Result | Log |
| --- | --- | --- |
| New file alone (dev) | Ran 28, OK | `unitB_newtests_dev1.log` |
| 13 existing camera modules + new file, one process (before the last two test-only edits) | Ran 269, OK (expected failures 10) | `unitB_camera_modules_dev2.log` |
| Full PUBLIC_CPU before unit B (prompt baseline; also `repair_r2_public_cpu.log`) | Ran 4231, OK (skipped 10, expected failures 10) | `repair_r2_public_cpu.log` |
| **Full PUBLIC_CPU, final tree** (`tools/run_tests.py`, 921.8 s, exit 0) | **Ran 4259, OK (skipped 10, expected failures 10)** = 4231 + 28 | `unitB_public_cpu.log` |
| `tools/check_public_docs.py`, final tree | PASS, exit 0 | `unitB_check_public_docs_final.log` |

The known `test_occlusion_confirmatory` race did not occur. The tree was byte-identical before and after the final
run (`unitB_tree_before_public_cpu.sha256` re-checked OK; git status identical; HEAD unchanged). A first full run was
stopped by me after about one minute to restart on the final tree after a test-only edit; its partial log is kept as
`unitB_public_cpu_run1_STOPPED_BY_AGENT_partial.log` and is not evidence. The indoor golden digests
(`OutdoorOffIsBitIdentical`) and every existing test pass unchanged; no existing test file was edited.

Kernel fault count (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`; cpu4-5 offline,
runs pinned to 0-3,6-23): 0 at unit start (14:34), 0 before the final run (14:55:12), **1 after (15:10:39)**. The one
line is `14:59:46 QGroundControl[4066934]: segfault at 8 ... error 4 in libQt5Gui.so.5`, a user-space Qt null read
from the coordinator's QGroundControl AppImage smoke test with `QT_QPA_PLATFORM=offscreen` (attribution from the
coordinator, recorded in `unitB_kfault_attribution.txt`); not a machine check, not the test process. No other new
fault line. Per the coordinator, the suite was not rerun for this line.

## 5. Residual risks and open items (honest)

1. **NOT_TESTED (needs ROS 2)**: the rclpy code paths. `Px4GatewayNode._publish_odometry` assigning the encoded lists
   (with NaN) to a real `nav_msgs/Odometry`, the camera node's deep copy and numpy covariance read, the
   `READ_FIELD_TYPES` check on an installed `px4_msgs`, DDS delivery, and the node's diagnostic publication. The pure
   functions they call are tested; the ROS glue is a few assignment lines.
2. **NOT_TESTED (PX4/SITL/vehicle)**: real `epv` / `dead_reckoning` values in flight and whether they pass the outdoor
   contract (`epv_max_m` 0.5 m PROVISIONAL); PX4's `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` response after the gateway stops.
3. **The limit is PROVISIONAL** (3.0 s, bounds [1.0, 10.0]); the total time the vehicle holds is below the limit, plus
   the gateway's stale revoke (0.2 s at 10 Hz in the CPU test), plus `COM_OF_LOSS_T` (1.0 s default), then
   `COM_OBL_RC_ACT`. Choosing the value is the user's decision.
4. **Clock**: the episode runs on the ROS clock. On the vehicle that is the system clock; a uniformly slow ROS clock
   (sim time slower than real time) stretches the real-time hold proportionally (vehicle time is correct). The
   session's pause check (0.15 s) and the gateway's per-tick ROS/monotonic check catch jumps and pauses, not a slow
   rate.
5. **Operational**: the latch also fires before OFFBOARD (e.g. ground not verified for 3 s right after take-off while
   the pilot hovers in POSITION mode, gateway PRIMING); the operator must reset the session before handing over.
   A landing (gated ticks) does not end an open episode: re-climbing in the same session latches at the first hold
   (fail closed; reset needed).
6. **Intra-gateway pairing**: the forwarded fields belong to the `VehicleLocalPosition` sample paired (<= 25 ms) with
   the odometry sample by the pre-existing validity gate; "atomic" refers to the gateway -> policy hop.
7. **Malformed vs absent**: a malformed encoding refuses with the existing reason `estimator_vertical_fields_absent`
   (status MALFORMED and detail are in the diagnostics). A distinct refusal reason would extend `REFUSAL_REASONS`;
   left open.
8. **Exactly-zero policy command** ends an episode (it is the policy's command, not a hold); by design, stated here.
9. **README topic table** in the pre-outdoor prefix is pinned by an existing test; the encoding is documented in the
   outdoor section only.
10. The replays are report-only after the timeout frame (they keep processing; the session would propose nothing).
11. **Review**: no independent (Opus) review of unit B yet; no WORKLOG entry, Beads update or commit (hard limits of
    this unit). The 27 uncommitted paths (26 + the new test file) remain for the caller.

## 6. SHA-256 of changed files (final tree)

```text
5115bb2ed08757250cff76ecdda7226d4acfba27d9c3c9eb731612c69014e27c  tools/camera_ros2_adapter.py
e9164f8027d7f85eef43ae24975f9436ce420ba3e99caf535f2fe541d76a0f51  tools/camera_ros2_session.py
560b1f95b027a90a285a3ff10918ab5b92e9830220a3f928025470b7f7d9f7ee  tools/camera_ros2_node.py
79f471264213d1c405aab4fd257d9a5c76d0da0e430cfc8d06d376b76817e50c  tools/camera_ros2_px4.py
dc11c95ca29eecc503b6fc08dee1a5caed88bf2a22c03e84ed75935acb1a441b  tools/camera_ros2_px4_node.py
6c3cb03d750050d6b2092226e228d9e05667e957055dd39ac7c1c56e61b49bb8  tools/camera_policy_runtime.py
58d057d9034858af3faac6204377ea9dfbf5e1db96562163a7ccaa99901c47d5  tools/camera_policy_contract.py
0119b6f7b7b5164b7b9d8f3f669a936a1144fa4cccd0dbe77f2c83e720a17939  tools/oakd_outdoor_replay.py
fff4abfa85c58e7912b5b7449b2ed03c49bbc46688b569572ad7d8fd9d0f6a4c  ros2/motar_camera_policy/README.md
7b8d378d45ad2619e7fc6067e93953d5dd3dd5400525a858242af0e81126de22  docs/evidence/sitl_2026-11-05/camera_runtime/README.md
ad7a667321a2f3e1266c487418fba61483fa3dcaf2a111b3539a526315644ca7  tests/test_camera_runtime_unit_b.py
```

Also in `logs/unitB_changed_files.sha256`. Unified diff against `preB_tree`: `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/diff/unitB_vs_preB.diff` (regenerated on the final tree).

## 7. Logs (`/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/logs/`)

- `unitB_kfaults_before_start.txt`, `unitB_git_status_before.txt` (unit start)
- `unitB_camera_modules_dev1.log` (expected README-table failure before the README row), `unitB_camera_modules_dev2.log`
- `unitB_newtests_dev1.log`, `unitB_mutation_checks.log` + `unitB_mutants.py` (process-local mutation driver)
- `unitB_public_cpu.log`, `unitB_public_cpu_exit.txt`, `unitB_kfaults_before_public_cpu.txt`,
  `unitB_kfaults_after_public_cpu.txt`, `unitB_kfault_attribution.txt`, `unitB_git_status_before_public_cpu.txt`,
  `unitB_tree_before_public_cpu.sha256`
- `unitB_public_cpu_run1_STOPPED_BY_AGENT_partial.log`, `unitB_run1_*` (stopped first run, not evidence)
- `unitB_check_public_docs.log`, `unitB_check_public_docs_final.log`
- `unitB_changed_files.sha256`, `unitB_checkpoint_sha256.txt`

## 8. Git traceability

- Branch: `camonly/dev`; Base SHA: `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`; New commit: NONE (no git state change
  allowed in this unit).
- Changed files: the 10 modified files and 1 new file in section 3 (on top of the 26 pre-existing uncommitted paths,
  16 of which are byte-identical to `preB_tree`).
- Tests: see section 4. Working tree: 27 uncommitted paths. Remote push: NO. Main merge: NO.
- Raw/checkpoint integrity: no `results/**`, raw data, preregistration, checkpoint, `aerial_gym/task/navrl_task/*`,
  WORKLOG or settings file edited; seed-1 checkpoint SHA-256 `888e8056...c8e1` equals its pin (`logs/unitB_checkpoint_sha256.txt`); git status shows no change under `results/`, `aerial_gym/` or WORKLOG.
