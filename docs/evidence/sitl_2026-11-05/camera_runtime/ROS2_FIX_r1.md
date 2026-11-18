# Camera-runtime deployment fixes r1 (2026-11-12)

Worker: Claude Code subagent (Opus 5.5). Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch
`camonly/dev`, base HEAD `fc8aa4d` (clean before the work). CPU only; no GPU, simulator, ROS launch or hardware. No
commit, push or stash. Usage: USAGE_UNKNOWN (no provider-visible signal; `tools/agent_session.py status` reports
`usage_observation_status: UNKNOWN`). Input: `/home/fair/motar_tmp/ros2_runtime_verify_20261112/REPORT.md`
(H1-H5) plus the coordinator's scope addition B2 (ceiling), with its probe
`ceiling_probe/ceiling_probe.py`.

Not touched: `aerial_gym/task/navrl_task/*` (trained perception and `navrl_depth_validity.py` unchanged; `apply_mask`
is only called with a different mask), checkpoints, preregistrations, `results/`, existing evidence files, Frozen
Policy F, ROS 2 package config and launch files, permission and settings files.

## Decisions outside the written spec (read first)

1. **The plane rule for sensor-invalid pixels is bounded to [MinZ, MaxZ].** Spec B says a sensor-invalid pixel
   whose ray meets the floor at a distance >= MinZ becomes no-return; B2 says the same for the ceiling. With both
   filters on, that literal rule converts *every* sensor-invalid row, because the trained `cy = 44.5` lies between
   two rows, so no row is exactly on the horizon. The coordinator's required positive control (a 0.3 m slab in
   front of a textureless floor *and* ceiling still reads near) would then fail. I made the conversion apply only
   when the plane hit lies inside the sensor's measurable range, `MinZ <= z_plane <= MaxZ` (optical z, 0.4-20 m).
   Near the horizon the planes are farther than 20 m, so a real close surface keeps a band of sensor-invalid rows,
   for example rows 36-49 at 1.03 m camera height with a 3 m ceiling. This keeps *more* pixels sensor-invalid than
   the literal rule, which is the conservative direction. Consequence: a textureless floor or ceiling seen *beyond
   20 m* near the horizon (a large hall with no wall within 20 m) stays sensor-invalid and can read near. This is
   the same class as the out-of-scope sky/far no-return issue.
2. **"Distance >= MinZ" is evaluated as level-camera optical z**, because the sensor and the contract define MinZ
   (and MaxZ) on optical z. Floor: `z_floor = h_cam / v`. Ceiling: `z_ceil = (H - h_cam) / -v`, with
   `v = (row - cy) / fy`. Using ray range instead would differ only for bottom/top-edge rows at camera heights
   below about 0.5 m.
3. **Contract schema: two new required fields, validated structurally rather than by equality.**
   - `ground = {enabled, margin_m, height_reference: "floor"}` defaults to `{true, 0.10, "floor"}`.
   - `ceiling = {enabled, height_m, margin_m, height_reference: "floor"}` defaults to `{false, null, 0.10, "floor"}`.
     No placeholder height is written: an enabled ceiling needs a measured `height_m` in (1.0, 100] m, and a
     disabled ceiling must carry `height_m: null`.
   - Margins must be in (0, 0.5] m. JSON booleans are required, so `1` is not accepted for `true`.
   - CLI: `--no-ground-removal`, `--ground-margin-m`, `--ceiling-height-m` (enables the ceiling filter) and
     `--ceiling-margin-m` (only together with a measured height).

   The ROS 2 package config and launch files bind the contract only by path and SHA-256, so they did not change.
   **Every previously generated contract file is now invalid.** It already was, because the contract binds the
   SHA-256 of the changed runtime sources. Rebuild it with
   `python tools/camera_policy_contract.py --output NEW.json --seed 1 [--ceiling-height-m MEASURED]`, then update
   `policy_contract_sha256` in the node config.
4. **New fail-closed refusals** (`InputError`, so the tick is rejected and the history does not advance):
   - when the floor filter is on and the camera height `pos_z + 0.03 <= ground.margin_m`;
   - when the ceiling filter is on and the camera height `>= ceiling.height_m - ceiling.margin_m`.

   In both cases the filter would delete real obstacles at camera height. **Operational consequence:** if the ROS
   node already runs on the ground before take-off (estimator z about 0 at the floor), its session latches a fault
   on the first frame, and `~/reset_policy_session` is needed after climbing above 0.07 m (body z). The
   alternatives, a non-latching skip or starting the node after take-off, are a user decision.
5. **Pose interpolation does not relax acceptance.** The existing nearest-within-25 ms selection still gates every
   frame unchanged. The bracket only replaces the pose. Interpolation adds a constant `MAX_POSE_BRACKET_S = 0.15`
   (equal to `MAX_AGE_S`) in `camera_ros2_adapter.py`. `MAX_AGE_S` and `MAX_SKEW_S` are unchanged and are not
   contract-visible: they are defined separately in five modules, so making them contract parameters is not trivial
   (left for the latency budget decision).
6. **Spec C consequence, inside the spec but changing what the colour detector sees on the inverse-nearest path:**
   - A dense invalid region *on the target* is now black on both paths. It was already black on the z-buffer path.
   - Its pixels stay a near obstacle blob in the scan. Measured: 6 and 10 near pixels on the half-invalid sphere.
   - A target closer than MinZ is no longer colour-detected on the inverse-nearest path; it appears as a near
     obstacle instead. In training, such pixels kept their red colour with a 0.4/cos range. That case is a
     collision regime in any event.
7. `depth_near_pixels` is kept in the step output as an alias of `near_pixels` for existing readers. The projection
   report's ambiguous `invalid_pixels` (the old union) is replaced by `sensor_invalid_pixels` and
   `no_return_pixels`. No reader of `invalid_pixels` exists in the repository; only the external probes read it.

## Changes, mapped to file:line (working tree)

### A. Sensor-invalid vs no-return (H1, H5a), `tools/camera_policy_runtime.py`

- `_source_classes` (:86-107) classifies each source pixel.
  - Coverage comes first: `coverage` False means no-return. Then the sensor: the explicit mask, NaN, and z < MinZ
    including -inf mean sensor-invalid. Then range: z > MaxZ, including +inf, means no-return.
  - The optional `frame['coverage']` must be a same-shape bool array, otherwise the frame is rejected with
    `InputError`.
- `_splat_index` (:110-118) is the shared nearest-pixel splat, used for points and for ray directions.
- `project_rgbd` (:121-217):
  - The inverse-nearest path classifies each pixel by its sampled source pixel (:178-179). The FOV-coverage
    refusal is kept (:172-173).
  - The z-buffer path splats valid points as before. Sensor-invalid source rays are projected by rotation only;
    the translation is ignored and documented (:197-200).
  - Target class (:202-204): valid if any valid point lands; otherwise sensor-invalid if any invalid ray lands;
    otherwise no-return. The level camera's MinZ and MaxZ apply to reprojected valid points.
  - Returns `(rgb, ranges, sensor_invalid, no_return, cosine, report)`; both masked kinds hold 20 m (:207-217).
- `CameraPolicyRuntime.preprocess` (:438-466) calls `apply_mask` with **sensor_invalid only** (:453).
- `step` (:468-) reports `sensor_invalid_pixels`, `no_return_pixels`, `near_pixels` (alias `depth_near_pixels`),
  `ground_removed_pixels`, `ceiling_removed_pixels`, `height_filter` and `rgb_blanked_pixels` (:516-522).
- `replay` (:526-) accepts an optional per-frame `coverage` array (:531).

`tools/oakd_camera.py`:
- `Rectifier.apply` (:211-232) remaps a ones image with the depth's own map, nearest interpolation and border 0
  (:229-230). It returns `(rgb, z, invalid, coverage)`; `invalid` keeps the raw `depth == 0` meaning.
- The provenance records the coverage rule (:255).
- `frames()` unpacks and returns `"coverage"` (:311, :316).
- The capture CLI saves `coverage` (:399).

### B and B2. Floor and ceiling removal (H2 and the scope addition), `tools/camera_policy_runtime.py`

- `remove_floor_and_ceiling` (:220-300) is a port of `tools/sitl_s4_gazebo.py:remove_ground` into the already-level
  camera: `hit_z = pos_z + 0.03 + range * (-(v - cy)/fy * cosine)`. It uses exact float64 arithmetic, no cdist, and
  does not mutate its inputs.
- Floor (:270-283):
  - a valid pixel with `hit_z < margin` becomes MaxZ, no-return (:275);
  - a sensor-invalid pixel with `v > 0` and `MinZ <= h/v <= MaxZ` becomes no-return (:276-279);
  - refusal when the camera height is at or below the margin (:273).
- Ceiling (:284-297):
  - a valid pixel with `hit_z > H - margin` becomes no-return (:291);
  - a sensor-invalid pixel with `v < 0` and `MinZ <= (H - h)/-v <= MaxZ` becomes no-return (:292-295);
  - refusal at or above the band (:289).
- It is called in `preprocess` between projection and `apply_mask`.
- `tools/camera_policy_contract.py`:
  - defaults and bounds (:18-28);
  - fields added to the metadata (:135);
  - `height_filter_problems` (:144-174);
  - `build_contract(..., ground=None, ceiling=None)` (:177-);
  - validation (:211-214);
  - CLI (:269-296);
  - the `--check` output prints both fields.

### C. Goal range (H3), `tools/camera_policy_runtime.py:457-458`

RGB is set to 0 for `sensor_invalid | no_return | near` before `observe`, on both paths. The docstring at :441-447
explains the change. The obstacle map is unchanged.

### D. Pose interpolation (H4)

`tools/camera_ros2_adapter.py`:
- `MAX_POSE_BRACKET_S` (:28).
- `_slerp` (:247-259): shortest arc.
- `interpolate_state` (:262-279): position, world velocity and yaw rate are linear; orientation is slerped. It
  refuses extrapolation and brackets longer than 0.15 s.
- `build_frame(..., odometry_bracket=None)` (:282-327): the existing 25 ms gate is unchanged. With a bracket, the
  state is evaluated at `t = rgb_t`; a sample exactly at `rgb_t` is used as is. The result is recorded in
  `frame['pose_provenance']` as `pose_interpolated`, `odometry_t` and `pose_skew_s`.

`tools/camera_ros2_session.py`:
- `WorkItem.odometry_bracket` (:36).
- `prepare` attaches `_odometry_bracket(...)` (:171).
- `_odometry_bracket` (:176-190): the newest fresh sample at or before `rgb_t` and the oldest at or after it, within
  the 0.15 s age bound. Nothing is extrapolated: no queued sample is newer than now.
- `infer` (:192-199) passes the bracket and adds `pose_interpolated` and `pose_skew_s` to the result.
- `finish` adds `pose_interpolated` to the proposal (:218).

## Tests (positive and negative controls)

New file `tests/test_camera_runtime_fixes.py` (913 lines). The scene renderer extends the verification probe's
`synth_scene.py` with ceilings, side walls, textureless planes, holes, borders and far regions.
`legacy_project_rgbd` (:172) reproduces the pre-fix fc8aa4d union-mask projection as the negative control.

`ActualPolicySeed1` runs the **real CAM-K2-v3 seed-1 checkpoint** (contract pin `888e8056…`) through
`CameraPolicyRuntime.step` in one fresh CPU process. It runs 111 scenes in about 7 s; the measurements are in
`actual_seed1_measurements.json`. The test switches only the validated `ground`/`ceiling` contract fields between
scenes: the contract is rebuilt by `build_contract` and assigned to the runtime. This is disclosed as a test-only
switch. Source is OAK-like (1014x760, 69x54.6 deg, ASSUMED) unless stated; body at 1.0 m. Scan columns are the
in-FOV bins (13 bins, |bearing| <= 33 deg) of obs[:288]. "cols" are the 160 depth-column minima after
preprocessing; tokens are the newest obstacle slot.

| # | Test (file:line) | Fixed result | Negative control (pre-fix or filter off) | Positive control |
|---|---|---|---|---|
| 1 | `test_1_tilt_and_roll_leave_no_phantom_wall` (:750) | Pitch 0, +-1, +-3, +-5, +-10 and roll +-5, filters off and on: near 0, in-FOV bins < 1 m: 0, cols < 1 m: 0, no token < 1 m. No-return band 160 / 800 / 1280-1440 / 2724 / 240 px. | Legacy: pitch +-3, +-5, +-10 give **13/13 bins (100 %)** < 1 m, 160/160 cols, near 800-2722, token 0.42-0.44 m. Roll +-5 gives 6/13 bins and 76 cols (0.47 m @ +-20 deg). Pitch 0 and +-1 give 0 % (below the OAK-like onset). | Level observation is bit-identical to legacy (the fix is the identity where nothing was wrong). |
| 2 | `test_2_pole_at_3m_survives_tilt` (:775) | Pitch 0/3/5 and roll 5, filters off and on: token 2.80 m @ 0 deg; no token < 1 m. | Legacy pitch 3/5: only 0.442/0.435 m @ -5 deg, no 2.8 m token. | Legacy pitch 0: 2.801 m @ 0 deg. |
| 3 | `test_3_dense_slab_stays_near_and_isolated_holes_do_not` (:788) | Slab at optical z 0.3 m, pitch 0 and 3, off/on/legacy: interior slab cols 0.40-0.48 m. Fixed: 58 (0 deg) / 56 (3 deg) cols < 1 m; near 5220 / 4754 (off) and 2894 / 2508 (on, where the floor explains the below-horizon rows); outside cols >= 9 m. Legacy pitch 3: 160 cols (phantom band). | iid 2 % holes: near 0, cols < 1 m: 0 (OAK 0/3 deg, trained-equal 0/3 deg). | Holes present as sensor-invalid: 155 / 169 / 142 px (OAK 0 deg, trained-equal 0 and 3 deg). |
| 4 | `test_4_uncovered_border_is_no_return_while_invalid_border_is_near` (:808) | 60 px coverage-False border, pitch 0 and 3: near 0, cols < 1 m: 0, no-return 3000 / 2812. | — | Same border as sensor-invalid: near 3000 / 1860, 160/160 cols < 1 m. |
| 5 | `test_5_beyond_maxz_is_no_return_not_near` (:817) | Optical z 25 m or +inf above the horizon, pitch 0 and 3: near 0, no-return 7200 / 7840. | — | Same region flagged invalid: near 7200 / 7040, 160 cols < 1 m. |
| 6 | `test_6_floor_is_removed_and_obstacles_on_it_are_kept` (:827) | Textured floor, h 1.0 / 1.5, filter on: no token < 9 m (9.9 m wall only); 5920 / 5120 px removed. Pitch 3 on: no token < 9 m. Textureless floor on: near 0, cols < 1 m: 0. | Filter off: tokens 2.30 m (h 1.0) and 3.416 m (h 1.5). Textureless floor off: near 5760 / 4960, cols 0.402 / 0.405 m. | Pole over floor at h 1.5, on: 2.801 m @ 0 deg (off adds 3.416 m floor). Slab over textureless floor, on: interior cols 0.40-0.48, near 2894. |
| B2 | `test_b2_ceiling_is_removed_and_vertical_walls_and_poles_are_kept` (:849) | Textured and textureless ceiling at 2.5 / 3.0 m with the matching ceiling filter: near 0, every col >= 14.90 m (far wall), ceiling removed 5920 / 5440 px. | Ceiling filter off: textured 3.269 m (2.5) and 4.380 m (3.0) col min, tokens 3.282 and 4.398 m. Textureless 0.402 / 0.404 m, near 5760 / 5280. | (a) Room h 1.5, floor plus 3 m ceiling plus pole: only token 2.801 m @ 0 deg. (b) Slab at 0.3 m before a textureless floor and ceiling, both filters on: interior cols 0.40-0.48 m, near 800. (c) Textureless empty room: near 0. (d) Frontal wall at 2.0 m: centre col 2.000 m, identical with filters on and off. (e) Side wall at y = 1.1 m: left-edge cols 2.027 m upward, equal to exact geometry within 1 %; tokens 2.494 / 3.002 / 3.873 m at 25 / 20 / 15 deg. |
| 7 | `test_7_goal_range_never_uses_substituted_pixels` (:884) | Half-invalid sphere at 5 m (true surface 4.75 m): trained-equal level 4.799, OAK level 4.804, OAK pitch 0.5 (z-buffer) 4.792. All within 0.1 m. | Legacy inverse-nearest: **3.599** (trained-equal) and **3.231** (OAK), more than 1 m short. Legacy z-buffer was already 4.792. | Valid sphere: same surface (4.799 / 4.804 / 4.792) and same mask (32 / 36 / 44 px) before and after. Invalid half present as sensor-invalid (16 / 18 / 10 px). |
| 8 | `test_8_ros_pose_interpolation_lifts_the_goal_at_capture_time` (:902) | ROS messages, session, adapter and real runtime at 2.5 m/s; image 20 ms before the latest pose (samples at -20 / +20 ms): first tracker state x = 5.548635, equal to the truth-pose run (error < 1 cm; 0 at float32 print). | Nearest only (+20 ms sample): 5.598636, **0.0500 m** error, `pose_interpolated` False. | — |

Fast tests (no checkpoint):

- `SourceClassification` (:233):
  - precedence (coverage over flag over range) on single pixels, with 5 sensor-invalid and 5 no-return;
  - coverage-mask validation;
  - z-buffer invalid rays are rotation-only: a 0.5 m mount shift leaves the masks identical; control: an
    all-valid tilted frame has 0 sensor-invalid pixels;
  - a valid point wins over an invalid ray, and the level MinZ applies.
- `FloorAndCeilingGeometry` (:305):
  - exact hit-height arithmetic (floor hit 0 within 1e-4 m; every removed pixel < 0.1 m, every kept one >= 0.1 m;
    the wall's bottom 10 cm is removed by definition);
  - filters off are the identity;
  - textureless planes are fully explained, while the slab keeps sensor-invalid rows in every slab column (>= 8 in
    the centre column);
  - refusal at camera heights 0.08 m and 2.93 m, with a positive control at 1.03 m.
- `PoseInterpolation` (:446), session level with a capturing runtime:
  - at yaw rate 0 and 1 rad/s, the interpolated lifted goal error is < 1 cm and t = rgb_t;
  - nearest control: 0.050 m (yaw rate 0) and > 0.05 m (1 rad/s);
  - a 0.2 s gap around the image is rejected (no runtime call);
  - a 0.16 s bracket is not interpolated, while a 0.14 s bracket is (positive control);
  - two samples before the image are not extrapolated; an exact sample is used as is;
  - `interpolate_state` refuses extrapolation and a 0.16 s bracket, and slerps the short arc.
- `tests/test_camera_policy_contract.py`:
  - :110: defaults; 4 accepted variants; 18 malformed `ground`/`ceiling` mutations rejected, including an enabled
    ceiling without a height and a disabled ceiling with one;
  - :147: CLI passes the measured height and the margin, refuses `--ceiling-margin-m` without a height and refuses
    filter options with `--check`.
- `tests/test_oakd_camera.py`:
  - :242: no distortion gives coverage all True (positive control); k1 = +0.45 gives a coverage-False border with
    zero depth and invalid True there, and no invalid pixel inside coverage;
  - :263: the `frames()` dict carries `coverage`.
- `tests/test_camera_ros2_adapter.py:128`: `build_frame` with a bracket interpolates to `rgb_t` (x 1.02 m); without
  one it keeps the nearest (skew 0.01 s); a non-bracketing pair is refused.

Existing tests changed, with reasons (no check weakened):
- `tests/test_camera_policy_runtime.py` (:39, :51, :66, :86): updated to the 6-tuple return and to assert
  `no_return` too.
- `test_invalid_depth_keeps_independent_mask_and_never_mutates`: the old union count 104 is now asserted as
  103 sensor-invalid plus 1 no-return. The +inf pixel is z > MaxZ under spec A. Per-pixel classes, disjointness and
  the report counts are added.
- `test_estimator_tilt_uses_measured_z_buffer_leaves_disocclusions_invalid` is renamed to `..._no_return`. The old
  assertion "the uncovered band is invalid" *was* the H1 bug. It now asserts no-return > 0 **and**
  sensor-invalid == 0.
- The existing real-checkpoint tests (`ActualPolicyCPU`, both seeds; `test_camera_ros2_integration`, both seeds)
  pass unchanged with the floor filter on by default.

## Verification

- Focused (all `tests/test_camera_*.py`, `test_oakd_camera`, `test_depth_validity` and `test_realveh`):
  `Ran 189 tests in 42.335s` / `OK` (log `focused_r1.log`).
- Full PUBLIC_CPU (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= nice -n 10 timeout
  3600 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/run_tests.py`, log `public_cpu_r1.log`):
  `Ran 4092 tests in 772.748s` / `OK (skipped=10)` / `EXIT=0`. The 5 GPU_REQUIRED exclusions are the same as the
  baseline. Baseline on fc8aa4d: 4067 tests, OK, skipped 10, so this adds 25 new tests. Source SHA-256 at launch
  and after are identical (`camera_policy_runtime.py` 717e5c6e…, `test_camera_runtime_fixes.py` 4e36fbc0…).
- Kernel fault count (`journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"`): 0
  before the work; 0 before and 0 after the focused run; before PUBLIC_CPU 0 (23:35); after PUBLIC_CPU 0 (23:48).
  cpu4-5 offline throughout.

## Changed files

Tracked, modified (+515 / -60): `tools/camera_policy_runtime.py`, `tools/camera_policy_contract.py`,
`tools/oakd_camera.py`, `tools/camera_ros2_adapter.py`, `tools/camera_ros2_session.py`,
`tests/test_camera_policy_runtime.py`, `tests/test_camera_policy_contract.py`, `tests/test_oakd_camera.py` and
`tests/test_camera_ros2_adapter.py`. Untracked, new: `tests/test_camera_runtime_fixes.py`. `tools/sitl_camera.py`
was not changed.

Outside the repository (this folder):
- `FIX_r1_tracked.patch`, the tracked diff;
- `test_camera_runtime_fixes.py.copy`;
- `actual_seed1_measurements.json`;
- `dump_actual.py`;
- `run_focused.sh`;
- `focused_r1.log`;
- `public_cpu_r1.log`;
- `debug/side_wall_col20.py`, which established that the floor is nearer than the side wall in column 20 when the
  filter is off.

## Open items

1. **No independent review yet.** AGENTS.md requires a fresh-context Opus review before any GO, and before the
   commit is described as verified.
2. **Not committed** (instruction). A commit needs the user's approval, a WORKLOG entry, and a record family entry
   if an evidence folder is created. `docs/evidence/sitl_2026-11-05/camera_runtime/*` still describes the pre-fix
   runtime and was not edited.
3. **Ground refusal before take-off** (decision 4): choose latching (current), non-latching skip, or starting the
   node after take-off.
4. **Measured ceiling height.** The ceiling filter stays off until a contract is built with a measured
   `--ceiling-height-m`. The estimator z must be height above the floor (`height_reference: floor`). If the EKF
   origin is not at floor level, both filters are wrong; this is unverifiable from code.
5. **Coverage on the ROS path.** A ROS rectified depth image cannot carry a coverage mask, so its zero border still
   reads sensor-invalid (near, if inside the trained FOV). It could be derived from CameraInfo K, D and P. Only
   `oakd_camera.py` produces coverage now.
6. **ROS 32FC1 +inf.** `decode_depth` flags +inf as invalid, which makes it sensor-invalid. REP 117 defines +Inf as
   "beyond range", which would make it no-return. This was left unchanged because an existing adapter test pins it.
   The runtime itself classifies an unflagged +inf as no-return.
7. **Out of scope, still open:** sky and far no-return. `gazebo_frame` flags +inf and <= 0 as invalid, so the SITL
   sky reads near; a textureless floor or ceiling beyond 20 m does the same. Also open: thin-pole column min-filter
   (H5b), altitude hold in the PX4 gateway, and the latency budget.
8. **Interpolation coverage.**
   - Only the ROS session/adapter path interpolates. The MAVLink companion path (`sitl_camera.run_control`) still
     pairs the nearest sample.
   - The session's odometry queue holds 8 samples. At >= 100 Hz odometry, an image older than about 70 ms loses its
     earlier bracket sample and falls back to the nearest (`pose_interpolated` False is reported).
9. **Margin semantics.** The bottom 10 cm (floor) and top 10 cm (ceiling) of every vertical obstacle are removed by
   definition. Objects lower than the margin are invisible. Hit height uses the 20 m-clamped range, as S4 did.
10. **Real-sensor qualification:** the OAK-D's real K and D, the share of invalid pixels in real frames, and a CPU
    replay of a bench recording. All NOT_TESTED.

Git Traceability: Branch `camonly/dev`; Base SHA `fc8aa4d`; New Commit SHA none (commit not authorized); Changed
Files as listed above (9 tracked modified, 1 untracked new); Tests as in Verification; Working Tree: 10 uncommitted
paths, all from this unit; Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity: checkpoint read-only (SHA pin
verified by `build_contract` at every runtime construction); `results/` untouched.
