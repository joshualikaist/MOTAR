# Camera-runtime deployment fixes r2 (2026-11-12/13)

Worker: Claude Code subagent (Opus 5.5). Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch
`camonly/dev`, HEAD `fc8aa4d`, with round 1 applied and uncommitted (kept; its 10 paths verified by SHA-256 before
any r2 edit: `r2/baseline_r1/sha256_r1_state.txt`, runtime `717e5c6e…` as in FIX_r1). CPU only: no GPU, simulator,
ROS launch or hardware. No commit, push or stash. Usage: USAGE_UNKNOWN (no provider-visible signal). Scratch:
`/home/fair/motar_tmp/ros2_runtime_fix_20261112/r2/`.

Not touched: `aerial_gym/task/navrl_task/*`, checkpoints, preregistrations, `results/`, evidence files, Frozen
Policy F, permission and settings files, MAX_AGE_S / MAX_SKEW_S (still 0.15 / 0.025), `tools/oakd_camera.py`,
`tools/camera_policy_contract.py`, launch files and `ros2/motar_camera_policy/config/unconfigured.json`.

## Decisions outside the written spec (read first)

1. **G1. Each velocity component is limited against the wall it points at**, with d = the distance to that wall.
   For the component toward the nearer wall this is exactly the specified rule. A component pointing away from
   the nearer wall but toward the farther wall is limited by the same formula; it only binds when the farther
   wall is inside that component's stopping distance (boxes narrower than about 2-3 m). Reason: a
   nearer-wall-only rule lets a narrow box be crossed. Measured in a test (1 m box, 0.1 m from x_min, 2.5 m/s
   toward x_max, latency 0.2 s): the nearer-only rule passes 2.5 m/s and the stop crosses x_max by 0.655 m; the
   directional rule publishes 1.791 m/s and the stop ends at the wall (overshoot 5.9e-15 m). Inward and
   along-wall components in an ordinary box are unchanged bit for bit.
2. **G2. A breach also zeroes the yaw rate.** The spec says zero horizontal velocity with altitude hold. I reused
   the gateway's existing neutral setpoint (zero XY velocity, zero yaw rate, the same z hold), because following
   the policy's yaw rate while ignoring its velocity has no defined meaning once the policy is overridden.
3. **G3. The breach latch is a new latched `geofence_fault`, not `revoke`.** The gateway's existing fault idiom,
   `revoke`, stops both streams, which contradicts "sends zero horizontal velocity, still holding altitude". So
   the breach keeps the streams up and latches `geofence_fault`. It is cleared only by an explicit reset, in the
   same style as the existing operator services:
   - `Px4OffboardGateway.reset_geofence()`;
   - `BridgeSession.reset_geofence()`;
   - the Trigger service `~/reset_geofence` (next to `~/prime_from_manual_hover` and `~/stop`).

   Every other fault still revokes as before. In particular, a stale or missing policy command during a latched
   breach still revokes (unchanged path), so a dead policy is not hidden behind the hold.
4. **G4. Reset semantics.**
   - The reset decodes the snapshot exactly as `update()` does. A decode failure (stale, estimator reset,
     invalid) revokes, as in `update()`.
   - A position still outside the box refuses the reset; the fault stays latched and the gateway is not revoked.
   - "Inside" means not strictly outside: the boundary counts as inside. At the boundary d = 0, so the outward
     component becomes 0.
   - A reset needs a primed or active gateway with a latched fault; otherwise it is refused.
5. **G5. A breach found at `prime` latches** (the neutral priming output is unchanged). It does not refuse the
   prime.
6. **G6. Validation details.**
   - The `decel_mps2` default (2.9608856678) lives in the template and in `Geofence`'s constructor. An *enabled*
     block with `decel_mps2: null` refuses, like every other missing value.
   - `latency_s` must be > 0.
   - Values must be JSON numbers; strings and booleans are refused.
   - A disabled block is checked for its exact key set and its frame; its box and latency values are ignored.
   - `Px4OffboardGateway(..., geofence=...)` is a required keyword argument (enabled or disabled), so no caller
     gets an implicit no-fence.
7. **G7. The frame is the literal `px4_local_enu`**: x east, y north, metres, origin at the PX4 EKF
   local-position origin. It is the frame of `decode_estimator()['pos']`, the position the gateway already uses,
   and of its published Odometry.
8. **H1. CameraInfo D handling.**
   - A message with no `d` attribute is refused (a real `sensor_msgs/CameraInfo` always has one).
   - An empty or all-zero D gives all-True coverage (case `zero_distortion_all_covered`).
   - A nonzero D needs `distortion_model` in {`plumb_bob`, `rational_polynomial`} with 4/5/8/12/14
     coefficients; otherwise the frame is refused (fail closed: `equidistant`/fisheye is not derived).
   - OpenCV is imported lazily and only for a nonzero D.
   - The session's calibration identity now also includes `distortion_model`.
9. **I1. A finite 32FC1 z <= 0 stays sensor-invalid** (REP 117 does not define it, but some drivers publish 0).
10. **J1. An altitude-gated tick counts as a successful tick** for the output-stream watchdog
    (`_last_success_wall`). Otherwise a session would fault 0.15 s after descending below the band, which
    contradicts "no fault".
    - The input freshness, ROS clock pause and 10 Hz period checks apply from the first finished tick (gated or
      not; new flag `live`). Before r2 they applied only from the first proposal. That is stricter for a session
      that waits on the ground, and identical otherwise.
    - Gated ticks consume their inputs (`_last_used`) and advance the session `sequence`; they never call
      `runtime.step`.
11. **J2. A shared predicate `height_band(state, contract)` was added to `tools/camera_policy_runtime.py`.**
    `remove_floor_and_ceiling` now refuses through it, with the same messages, so the session and the runtime
    cannot disagree about the band. Consequence: the runtime's SHA-256 changed again, so every previously built
    policy contract is invalid (as after r1). Rebuild it with `tools/camera_policy_contract.py` and update
    `policy_contract_sha256`.

Nothing here changes the trained observation computation. One execution-layer note: when the geofence clamps,
the vehicle executes a smaller velocity than the governed command the runtime records as `previous_action`. The
measured velocity in the observation shows the real motion. This is inherent to any post-policy limiter in the
approved design.

## Changes, mapped to file:line (working tree)

### G. Geofence velocity clamp, `tools/camera_ros2_px4.py`

- Module docstring (:8-17): purpose, the frame, the formula and the semantics of `latency_s`.
- Constants (:45-50):
  - `GEOFENCE_FRAME = "px4_local_enu"`;
  - `GEOFENCE_KEYS`;
  - `GOVERNOR_BRAKE_MPS2 = 2.9608856678` (the training environment's `NAVRL_SPEED_GOVERNOR_BRAKE_MPS2`; a test
    pins it to both seeds' training reports);
  - `MIN_GEOFENCE_WIDTH_M = 0.5`.
- Template and formula:
  - `geofence_template()` (:57): disabled, no stand-in values;
  - `stop_speed(d, a, tau)` (:63): `-a*tau + sqrt(a^2*tau^2 + 2*a*d)`, 0 at d <= 0.
- `class Geofence` (:72-147):
  - validation (:82-104) and `from_config` (:107): exact keys, literal frame, fail closed;
  - `outside` (:121): strict breach test;
  - `limit` (:126): per-axis clamp; never increases a component; unclamped components are returned as the same
    float.
- `Px4OffboardGateway`:
  - `geofence=` is a required keyword argument (:340); state fields `geofence_fault` and `geofence_record`.
  - `prime` latches a breach (:420).
  - `_neutral` records its output (:426); `_record` (:432) writes the per-command `geofence_record`: position,
    output kind, `geofence_clamped`, `pre_clamp_velocity_xy_mps`, `post_clamp_velocity_xy_mps`, per-axis
    distance, v_max and clamped flag, and the fault.
  - `_check_geofence` (:438) latches a strict breach from the freshly decoded position.
  - `_geofenced` (:443) encodes the stored policy command limited at this update's position. A latched fault
    gives the neutral setpoint. Only the XY components change; yaw rate and the z hold are untouched.
  - `reset_geofence` (:459): explicit reset; fresh decode; refused outside.
  - `update` decodes first (freshness, validity, reset counters) and then calls `_check_geofence` (:519). The
    PRIMING neutral outputs are recorded; ACTIVE output goes through `_geofenced` (:547).

`tools/camera_ros2_px4_node.py`:
- `unconfigured_template()` (:18-33) has the required, disabled `geofence` block (documented in its docstring).
- `load_node_config` validates it before any message-directory check or publisher (:99). An enabled fence with
  a missing, non-finite or narrow box makes `load_node_config` raise and `--check` print `NOT_CONFIGURED`.
- `BridgeSession`:
  - passes the block to the gateway (:124);
  - `reset_geofence` (:205);
  - `diagnostics()` (:215) carries the geofence metadata, the fault and the last record.
- ROS node: Trigger service `~/reset_geofence` (:304, handler :358); the diagnostics JSON uses
  `session.diagnostics()` (:406); `--check` prints `geofence: ENABLED/DISABLED` (:431).

`ros2/motar_camera_policy/config/px4_unconfigured.json` (:20-29): the disabled block with `latency_s` and the box
null, and `decel_mps2` 2.9608856678.

### H. Coverage on the ROS image path, `tools/camera_ros2_adapter.py`

- `REMAP_DISTORTION_MODELS` and a 4-entry cache (:200-203).
- `camera_info_coverage(msg, width, height)` (:206-258):
  - validates K, R, P and D;
  - nonzero D: `cv2.initUndistortRectifyMap(K, D, R, P[:, :3])` remapping a ones image, nearest, zero border
    (:246), exactly as `oakd_camera.Rectifier.apply`;
  - D empty or zero: all True (:237);
  - the cache is keyed by (size, model, K, D, R, P); arrays are read-only; the provenance records `case`,
    `distortion_model` and `uncovered_pixels`.
- `build_frame` attaches `frame['coverage']` and `frame['coverage_provenance']` (:389-394); its docstring is
  updated. The runtime's existing `_source_classes` then classifies uncovered pixels as no-return (r1).
- `tools/camera_ros2_session.py`: the calibration identity includes `distortion_model` (:122); `infer` passes
  `coverage_case` into the result (:223-224).

### I. REP 117 in `decode_depth`, `tools/camera_ros2_adapter.py:176-197`

- 32FC1:
  - +inf keeps z = +inf with `invalid` False (:194-195), so the runtime classifies it as z > MaxZ, i.e.
    no-return;
  - -inf, NaN and a finite z <= 0 are invalid with z = 0, i.e. sensor-invalid.
- 16UC1: 0 is invalid, unchanged. The docstring cites REP 117.

### J. Altitude gating

`tools/camera_policy_runtime.py`:
- `height_band(state, contract)` (:220-238) is a pure predicate. It returns `in_band`, `camera_height_m` and
  `reason`.
- `remove_floor_and_ceiling` refuses through it (:285-287), with the same messages; the two inline checks were
  removed.
- `CameraPolicyRuntime.height_band(state)` (:438).

`tools/camera_ros2_session.py`:
- docstring (:8-13);
- `WorkItem.fresh_history` (:45);
- state fields `live`, `waiting_altitude`, `altitude` and `_fresh_history` (:64-70);
- `status` property: `FAULT`, `WAITING_ALTITUDE`, `RUNNING_PROPOSALS` or `WAITING_FOR_INPUTS` (:83-88);
- the clock-pause and stream watchdogs key on `live` (:140, :151);
- `prepare` carries `fresh_history` (:196);
- `infer` checks the band and, out of band, returns a gated result without calling the runtime (:225-228). In
  band after gating, it calls `runtime.reset()` right before `step` (:229-230).
- `finish`:
  - a gated result goes through the same pending, epoch, watchdog and expiry checks, consumes its inputs,
    keeps the session live, sets WAITING_ALTITUDE and returns None (:242-248);
  - an in-band proposal clears the waiting and fresh flags (:260-263).

`tools/camera_ros2_node.py`: publishes only non-None proposals (:153); the diagnostics state is `session.status`
plus `altitude` (:172).

## Tests and their controls

New file `tests/test_camera_runtime_fixes_r2.py` (867 lines, 22 tests). Measurements are in
`r2/measurements_r2.json`, from `r2/dump_r2.py`.

### G (gateway geofence)

- `test_template_is_disabled_without_stand_in_values_and_decel_is_the_trained_governor` (:141). The node
  template, the JSON template and `geofence_template()` are equal: disabled, box and latency null, decel equal
  to `NAVRL_SPEED_GOVERNOR_BRAKE_MPS2` in both training reports.
- `test_enabled_fence_with_missing_nonfinite_or_narrow_values_refuses` (:156). 40 malformed blocks are refused:
  - None, NaN, inf, bool or string for each of the six values;
  - widths 0.49 and 0.4 m, an inverted box, latency 0, negative decel;
  - wrong frame, `enabled: 1`, a missing or extra key, a non-mapping.

  Positive controls: a valid box, and a box exactly 0.5 m wide. The gateway without `geofence=` raises
  TypeError. `BridgeSession` with an enabled fence missing `y_max_m` refuses; with a valid fence it constructs.
- `test_node_configuration_refuses_an_enabled_fence_without_a_box_before_starting` (:189):
  - `load_node_config` refuses an enabled fence without x bounds; `--check` returns 2, `NOT_CONFIGURED`, with
    the reason naming the geofence;
  - a config without the block is refused as a schema change;
  - positive control, with the local PX4 v1.14.3 message source: enabled and disabled blocks load, and `--check`
    returns 0 with `ENABLED` / `DISABLED`.
- `test_centre_of_the_box_passes_commands_bit_exactly` (:227). Six commands at the centre of a 20 m box are
  IEEE-bit-identical to a disabled-fence twin gateway (control) and to `encode_setpoint`. `geofence_clamped` is
  False and pre == post.
- `test_approach_at_2p5_follows_the_formula_and_the_stop_never_crosses_the_wall` (:244). 24 cases: latency 0.2
  and 0.4 s, d = 3, 1.5 and 0.5 m, all four walls of a 10 m box, command 2.5 m/s toward the wall.
  - The published normal speed equals min(2.5, v_max) within 1e-12, with the correct sign; the other component
    stays 0.
  - The double-integrator rollout (hold the published speed for the latency, then brake at a, exact
    constant-acceleration steps) never crosses the wall: overshoot <= 1e-9 m, measured at most 2.5e-14 m.
  - Control: the unclamped 2.5 m/s stop crosses by > 0.01 m whenever v_max < 2.5, and does not otherwise.

  | tau s | d m | v_max m/s | published | clamped | stop overshoot published | unclamped 2.5 m/s overshoot |
  |---|---|---|---|---|---|---|
  | 0.2 | 3.0 | 3.664 | 2.5 | no | -1.445 m | -1.445 m |
  | 0.2 | 1.5 | 2.446 | 2.446 | yes | 2.5e-14 m | +0.055 m |
  | 0.2 | 0.5 | 1.228 | 1.228 | yes | 8.4e-15 m | +1.055 m |
  | 0.4 | 3.0 | 3.194 | 2.5 | no | -0.945 m | -0.945 m |
  | 0.4 | 1.5 | 2.023 | 2.023 | yes | 4.4e-15 m | +0.555 m |
  | 0.4 | 0.5 | 0.905 | 0.905 | yes | 6.4e-15 m | +1.555 m |

- `test_parallel_component_yaw_rate_and_altitude_hold_are_untouched_while_the_normal_is_clamped` (:275). At
  0.5 m from x_max, command (2.0, 1.5):
  - east is clamped to v_max(0.5, 0.3) and north stays bit-exactly 1.5;
  - yawspeed, the z-hold position and `offboard_control_mode` are bit-identical to the disabled twin;
  - the record shows pre [2, 1.5], post [v, 1.5] and per-axis clamped [True, False].

  At a corner on the boundary, both outward components become 0 and no breach is latched.
- `test_inward_and_along_wall_commands_are_unchanged_near_a_wall` (:297). Four inward or along-wall commands
  near walls are bit-identical to the disabled twin. Control: the same position with the command turned toward
  the wall is clamped.
- `test_narrow_box_limits_the_far_wall_component_too` (:309), decision G1. Published 1.791 m/s, stop overshoot
  5.9e-15 m. Control: a nearer-only rule's 2.5 m/s crosses by 0.655 m.
- `test_breach_sends_zero_velocity_with_altitude_hold_latches_and_resets_only_inside` (:322):
  - strictly outside (x = 5.01): zero XY velocity, yawspeed 0, the z hold bit-identical to before, state ACTIVE,
    `geofence_fault` latched;
  - back inside without a reset: still zero;
  - reset while outside: refused, still latched, not revoked;
  - reset inside: cleared, and the next command passes (1, 2); a second reset is refused (nothing latched);
  - control: the same excursion with the fence disabled passes (-2, 1) through.
- `test_breach_during_priming_latches_and_bridge_reset_uses_the_session_snapshot` (:358):
  - `BridgeSession` primed at x = 6 latches without a session fault;
  - the bridge reset is refused outside and succeeds after the vehicle is inside;
  - the diagnostics serialize with `allow_nan=False`;
  - control: an observation-only bridge refuses the reset ("primed or active") with no fault.
- `test_stale_or_missing_position_is_rejected_before_the_geofence_sees_it` (:393):
  - a stale odometry receipt revokes, both outside and inside, and never latches a breach;
  - control: the same outside position, fresh, latches with the streams up;
  - a missing position revokes;
  - a reset with a stale estimator revokes;
  - a prime with a stale estimator refuses.

  Every position path (`prime`, `update`, `reset_geofence`, and `BridgeSession.poll`, which also decodes)
  decodes with freshness before the geofence reads the position.

### H (coverage)

- `test_distorted_camera_info_border_is_no_return_and_the_same_border_without_coverage_is_near` (:471). The test
  sends a plumb_bob CameraInfo with k1 = 0.45 and K = P = the trained 160x90 camera, and a wall at 9.9 m with
  the remap border delivered as depth 0. It goes through `build_frame`, then the production
  `CameraPolicyRuntime.preprocess` with the production blob_near, loaded privately.

  | case | border px | near | sensor-invalid | no-return |
  |---|---|---|---|---|
  | with coverage | 3516 | **0** | 0 | 3516 |
  | without coverage (control, pre-r2 ROS path) | 3516 | 3516 | 3516 | 0 |

  Every near pixel in the control lies in the border.
- `test_zero_distortion_gives_all_true_coverage_and_matches_the_device_path` (:502):
  - D = [] and D = 0 give all True, case `zero_distortion_all_covered`, also through `build_frame`;
  - device parity: for k1 = 0.45, -0.3 and 0.1, the ROS mask is identical to `oakd_camera.Rectifier.apply`'s
    coverage. It is non-vacuous: the k1 = 0.45 border is > 500 px and the k1 = 0.1 border is > 0.
- `test_coverage_is_cached_per_camera_info_content_read_only_and_fails_closed_for_unknown_models` (:530):
  - the same content with a new stamp returns the same read-only array object; k1 = 0.2 gives a different,
    smaller border;
  - `equidistant`, an empty or missing model with nonzero D, a NaN in D and a missing D are refused;
  - positive control: `rational_polynomial` works.
- Policy level (real seed-1 checkpoint, floor filter on). Distorted border:
  - with coverage: near 0, in-FOV bins < 1 m **0/13**;
  - without coverage (control): near 1852, **13/13** in-FOV bins < 1 m.

### I (REP 117)

`test_plus_inf_is_no_return_while_minus_inf_and_nan_patches_are_near` (:562) and
`test_16uc1_zero_is_unchanged_and_stays_near_when_dense` (:584). A 30x40 patch above the horizon, on a 32FC1 or
16UC1 message through `build_frame` and `preprocess`:

| patch | near | sensor-invalid | no-return | policy level: in-FOV bins < 1 m |
|---|---|---|---|---|
| +inf | **0** | 0 | 1200 | 0/13 |
| -inf | 1188 | 1200 | 0 | 3/13 |
| NaN | 1188 | 1200 | 0 | 3/13 |
| 3 m valid (control) | 0 | 0 | 0 | 0/13 |
| 16UC1 0 | > 1000 | 1200 | — | — |

For 16UC1, `decode_depth` is bit-identical to the pre-r2 rule. At policy level the +inf patch adds exactly 1200
no-return pixels to the valid-patch control (the floor filter also removes the synthetic wall below 0.1 m).

**Changed existing test, with the reason** (`tests/test_camera_ros2_adapter.py:77-92`). The old assertion pinned
32FC1 +inf as invalid. It now asserts z = +inf with invalid False, and the reason is written in the test: REP 117
defines +Inf as "no return within range". Pinned as invalid, a dense +Inf region became a 0.4 m surface under
blob_near. -1 and NaN stay invalid, and a new row adds -inf and 0. The fixture `info()` gained `d=[]` and
`distortion_model=""`, because every real CameraInfo carries D.

### J (altitude gating)

The session tests use `BandRuntime`, which has the production `height_band`, and whose `step` raises the
runtime's InputError out of band.

- `test_on_the_ground_no_proposal_no_fault_then_climb_starts_a_fresh_history` (:642):
  - 5 ground ticks (z = 0, camera at 0.03 m): no proposal, no fault, WAITING_ALTITUDE, runtime never called;
    the reason and camera height are recorded;
  - the climb to z = 1: proposal; the runtime saw exactly `reset, step`; the next tick saw `step` only;
  - a descent to 0.05 m mid-session waits without a fault, and the next climb is fresh again;
  - control: the same ground state passed directly to `step` is refused ("ground margin"). That is the old
    fault path.
- `test_ceiling_band_waits_and_a_first_in_band_frame_without_gating_is_not_reset` (:671):
  - with a 3 m ceiling, z = 2.9 waits ("ceiling band") without a fault;
  - the first in-band tick after a session reset is not reset twice; after gating it is.
- `test_stale_streams_and_timing_faults_still_fault_while_waiting` (:681). Positive control: fresh 10 Hz inputs
  keep waiting without a fault. While waiting, each of these still faults:
  - a stale odometry input stream (`input stream: odometry`);
  - a 0.13 s tick period (`10 Hz`);
  - a ROS clock pause (`paused`);
  - a malformed (non-unit) odometry quaternion: inference fails and the session faults.
- Policy level, `ActualPolicySeed1R2.test_j_...` (:828): the real seed-1 runtime through `RosPolicySession`,
  heights 1, 1, 0, 0, 0, 1, 1.
  - Statuses: RUNNING x2, WAITING_ALTITUDE x3, RUNNING x2. No fault.
  - During the waits, perception `step_count` (2 -> 2) and `_last_now` are unchanged.
  - The first proposal after the climb has an observation **identical** to a freshly reset runtime stepped
    with the same frame, and so does the first tick of the session.
  - Control, showing that history matters: the second in-band tick differs from a fresh-runtime observation.
  - Control: the ground frame passed directly to `rt.step` is refused (`camera height above the floor does not
    exceed the ground margin`).

Other existing tests changed: in `tests/test_camera_ros2_px4.py` (8 constructor calls) and
`tests/test_camera_ros2_px4_node.py:80`, the gateway constructors pass `geofence=P.geofence_template()`
(disabled, explicit). The fake runtimes in `tests/test_camera_ros2_session.py:57` and
`tests/test_camera_runtime_fixes.py:442` gained `height_band` (always in band). No assertion was weakened.

### Closed-loop observation (scratch model, not a test)

`r2/scratch/closed_loop2.py` and its output `closed_loop2.out` model the vehicle tracking the published command
after a pure response delay L, with acceleration bounded by a and a 10 Hz zero-order hold, while the policy
commands 2.5 m/s at the wall. The vehicle stayed inside only when `latency_s` was at least about
2 x (L + 0.1 s): with L = 0, 0.05 and 0.1 s, at least 0.2, 0.3 and 0.4 s. The minimum overshoot below that was
1-26 cm. With L = 0.2 s it still crossed by 0.6-0.8 mm at 0.6 s. This is a crude model, recorded only to say
that `latency_s` is not the transport latency alone. The operator margin and the measured latency must cover it.

## Verification (exact summary lines)

- Focused: all 16 `tests/test_camera_*.py`, plus `test_oakd_camera`, `test_depth_validity` and `test_realveh`
  (`r2/focused_r2.log`, runner `r2/run_focused.sh`): `Ran 211 tests in 43.569s` / `OK` / EXIT 0. r1 had 189;
  +22 new.
- Full PUBLIC_CPU (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= nice -n 10
  timeout 3600 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/run_tests.py`, log
  `r2/public_cpu_r2.log`): **`Ran 4114 tests in 783.768s` / `OK (skipped=10)` / `EXIT=0`**. 0 failures,
  0 errors, 4114 > 4092 (r1). The 5 GPU_REQUIRED exclusions are the same as in r1.
- Source SHA-256 at launch and after the suite are identical: `sha256sum -c` of
  `r2/sha256_at_public_cpu_launch.txt` gives 0 mismatches.

## Kernel fault counts

`journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"`:

| when | count | time (KST) |
|---|---|---|
| start of work | 0 | 2026-11-12 23:58 |
| before focused | 0 | 2026-11-13 00:11 |
| after focused | 0 | 2026-11-13 00:12 |
| before PUBLIC_CPU | 0 | 00:12 |
| after PUBLIC_CPU | 0 | 00:26 |

cpu4-5 were offline throughout.

## Changed files

Round 2 changed these paths. Round 1's changes in them are kept; the r2-only diff is `r2/FIX_r2_only.patch`
(sha256 `840159e5…`).

| path | status | lines added / removed by r2 |
|---|---|---|
| `tools/camera_ros2_px4.py` | tracked, new in r2 | +180 / -7 |
| `tools/camera_ros2_px4_node.py` | tracked, new in r2 | +42 / -5 |
| `tools/camera_ros2_node.py` | tracked, new in r2 | +3 / -3 |
| `ros2/motar_camera_policy/config/px4_unconfigured.json` | tracked, new in r2 | +11 / -1 |
| `tools/camera_ros2_session.py` | also changed by r1 | +48 / -6 |
| `tools/camera_ros2_adapter.py` | also changed by r1 | +76 / -2 |
| `tools/camera_policy_runtime.py` | also changed by r1 | +30 / -6 |
| `tests/test_camera_ros2_px4.py` | tracked, new in r2 | +8 / -8 |
| `tests/test_camera_ros2_px4_node.py` | tracked, new in r2 | +1 / -1 |
| `tests/test_camera_ros2_session.py` | tracked, new in r2 | +2 / 0 |
| `tests/test_camera_ros2_adapter.py` | also changed by r1 | +12 / -3 |
| `tests/test_camera_runtime_fixes.py` | untracked r1 file | +2 / 0 |
| `tests/test_camera_runtime_fixes_r2.py` | untracked, new | 867 lines |

The working tree now has 18 uncommitted paths: r1's 10 and r2's 8 new ones. `git status --short` lists 16
modified tracked paths and 2 untracked files.

The r1 paths `tools/camera_policy_contract.py`, `tools/oakd_camera.py`, `tests/test_camera_policy_contract.py`,
`tests/test_camera_policy_runtime.py` and `tests/test_oakd_camera.py` were not changed by r2.

Outside the repository (`/home/fair/motar_tmp/ros2_runtime_fix_20261112/r2/`):
- `baseline_r1/`: copies of the r1 state of every file r2 touched, the r1 patch and r1 SHA-256s;
- `FIX_r2_only.patch` and `FIX_r1_r2_tracked.patch` (`c7384bc7…`, tracked r1+r2);
- `test_camera_runtime_fixes_r2.py.copy` (`661167c5…`);
- `measurements_r2.json` and `dump_r2.py`;
- `focused_r2.log` and `public_cpu_r2.log`;
- the `kernel_faults_*.txt` files and `sha256_at_public_cpu_launch.txt`;
- `run_focused.sh`;
- `scratch/closed_loop*.py` and `scratch/closed_loop2.out`.

## Open items

1. **No independent review of r2** (nor of r1). AGENTS.md requires a fresh-context Opus review before this is
   called verified or GO.
2. **Not committed** (instruction). A commit needs the user's approval and a WORKLOG entry.
   `ros2/motar_camera_policy/README.md` (outside my edit scope) does not yet describe the `geofence` block, the
   `~/reset_geofence` service or `WAITING_ALTITUDE`.
3. **Measured values needed before enabling the fence.**
   - The flyable inner box, in PX4 local ENU from the EKF origin.
   - `latency_s` from the estimator sample to the vehicle's velocity response, including the 0.1 s gateway
     period and the estimator sample age (up to 0.15 s).
   - Whether the real vehicle can brake at 2.96 m/s^2. Its T/W of 4.0 is inferred, not measured.

   See the closed-loop observation: in that model, latency_s needed about 2 x (response delay + 0.1 s).
4. **User decision:** offboard publication is allowed with the geofence disabled. Should an enabled fence be
   required when `enable_offboard_publication` is true?
5. **Limits of the clamp itself.**
   - It limits the commanded velocity using position only; the measured vehicle velocity is not used. A vehicle
     already moving faster than v_max (overshoot, wind or estimator lag) is stopped only by the breach latch at
     the wall.
   - A breach holds with zero velocity. Leaving the latched hold needs the pilot (POSITION mode, which revokes
     the gateway) or a reset once the vehicle is inside.
6. **`previous_action` vs the executed command.** When the fence clamps, the policy's `previous_action` (the
   governed command) differs from what the vehicle executes. The approved design accepts this; it is not
   measured.
7. **OpenCV is now a runtime dependency** of the ROS policy node whenever CameraInfo D is nonzero.
   `package.xml` (outside my scope) does not declare `python3-opencv`.
8. **Not derived:**
   - fisheye/`equidistant` CameraInfo models, which are refused when D is nonzero;
   - a K-to-P zoom border with D = 0.
9. **Still open from r1:**
   - SITL `gazebo_frame` still flags +inf as invalid (the sky);
   - real OAK-D K/D, the invalid-pixel statistics of real frames and a bench replay: NOT_TESTED;
   - MAX_AGE_S/MAX_SKEW_S, sky, thin poles, MAVLink interpolation and the goal-closer-than-MinZ colour change,
     all out of scope and unchanged.
10. **Previously built policy contracts are invalid**, because the `camera_policy_runtime.py` SHA-256 changed
    (now `bc178e99…`). Rebuild with `python tools/camera_policy_contract.py --output NEW.json --seed 1
    [--ceiling-height-m MEASURED]` and update `policy_contract_sha256`.

## Git Traceability

- **Branch:** `camonly/dev`.
- **Base SHA:** `fc8aa4d`.
- **New Commit SHA:** none (commit not authorized).
- **Changed Files:** as listed above.
- **Tests:** focused `Ran 211 tests` OK; PUBLIC_CPU `Ran 4114 tests` `OK (skipped=10)`.
- **Working Tree:** 18 uncommitted paths (r1 + r2), all from these two units.
- **Remote Push:** NO.
- **Main Merge:** NO.
- **Raw/Checkpoint Integrity:** the checkpoint was only read (contract SHA pin `888e8056…` checked at every
  runtime construction); `results/` and the evidence files are untouched.
