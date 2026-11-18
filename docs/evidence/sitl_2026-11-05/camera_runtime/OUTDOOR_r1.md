# Camera-runtime outdoor profile r1 (2026-11-13): sky rule, camera-verified ground plane, virtual ceiling

Worker: Claude Code subagent (Opus 5.5). Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch
`camonly/dev`, HEAD `fc8aa4d` (unchanged). The 24 uncommitted paths of the repair series (r1/r2/repair-r1) were the
base and were only added to (snapshot `pre_tree/`, verified byte-equal to the worktree before any edit). No commit,
`git add`, stash, reset or checkout. Nothing under `aerial_gym/`, checkpoints, raw data, recorded evidence or the
gateway geofence was edited. CPU only (`CUDA_VISIBLE_DEVICES=""`, cores 4/5 avoided). Usage: `USAGE_UNKNOWN`
(`tools/agent_session.py status`: `usage_observation_status: UNKNOWN`).

**Evidence status: SYNTHETIC.** There is no OAK-D device and no OAK-D recording on this machine. Every scene below is
a ray-cast emulation (`tests/test_camera_runtime_outdoor_r1.py:outdoor_scene`) with ASSUMED sensor values. Nothing
here is real-sensor, PX4, ROS-graph, Gazebo or flight evidence. **No outdoor flight GO is given.**

**Result.** Sky and far ground are no-return outdoors (tree line and open field at heights 1.0/1.5 m, pitch -10..+10 deg, roll -5..+5 deg: 102-160 near bearings before, 160 at zero roll, -> 0 after); near obstacles stay near; the ground is verified per frame against PX4 and refused with a named reason otherwise (zero hold in the ROS session); indoor/default behaviour is byte-identical. Final PUBLIC_CPU: `Ran 4182 tests ... OK (skipped=10)`, kernel faults 0 -> 0. Open: real-data calibration, PX4 epv/dead_reckoning not forwarded on the ROS path (outdoor frames refused there), residual risks in section 10. No flight GO.

## 1. Files changed

`git status --short` before (26 lines incl. HEAD/branch: `logs/git_status_before.txt`): 21 modified tracked paths and
3 untracked test files (the repair series). After: the same 24 paths plus 2 new untracked files. Changes relative to
the pre-change snapshot (`diff/` holds the unified diffs of the source and README changes):

| Path | Change |
| --- | --- |
| `tools/camera_policy_contract.py` | +190 / -14: outdoor profile (`profile`, `sky`, `ground_estimate`, optional `ceiling.physical`), PROVISIONAL thresholds, `outdoor_blocks()`, `outdoor_problems()`, `is_outdoor()`, `threshold_value()`, `build_contract(outdoor=)`, CLI `--profile outdoor`, `height_reference_problem` returns None outdoors |
| `tools/camera_policy_runtime.py` | +422 / -16: `FrameRefused`, `split_state`, `project_rgbd(unimaged=)`, `estimate_ground`, `remove_floor_and_ceiling_plane`, `apply_sky_rule`, `_preprocess_outdoor`, outdoor branch in `preprocess`/`step`/`height_band`, NPZ `replay` handles refusals; indoor code paths untouched |
| `tools/camera_ros2_session.py` | +38 / -1: `FrameRefused` -> explicit zero-velocity hold proposal, `HOLD_FRAME_REFUSED`, `refusal`, `consecutive_refusals`, fresh history after a refusal |
| `tools/camera_ros2_node.py` | +6 / -2: diagnostics carry `refusal` and `consecutive_refusals` |
| `ros2/motar_camera_policy/README.md` | +96 / -0: dated section *Outdoor profile (2026-11-13, opt-in)* incl. capture protocol |
| `docs/evidence/sitl_2026-11-05/camera_runtime/README.md` | +25 / -0: dated section *Outdoor profile r1, 2026-11-13* |
| `tests/test_camera_runtime_outdoor_r1.py` | new, 37 tests (12 classes) |
| `tools/oakd_outdoor_replay.py` | new, real-OAK-D replay tool (no DepthAI dependency) |

Not touched: the gateway (`tools/camera_ros2_px4.py`, `tools/camera_ros2_px4_node.py`, geofence), the adapter,
`tools/oakd_camera.py`, every other repair-series file, `aerial_gym/**`, checkpoints, `results/`, WORKLOG, Beads.

## 2. Design as implemented

**A. Contract.** Opt-in outdoor profile: `profile: "outdoor"`, `sky`, `ground_estimate` (all three or none), and
`ceiling.physical` (optional key; absent = historical physical ceiling). `--profile outdoor` builds: floor filter on
(margin 0.10 m, plane rule off), virtual ceiling 3.0 m with `physical: false` (overridable by `--ceiling-height-m`),
sky on, ground estimate on, `estimator_height: null`. `physical: false` with `invalid_plane_rule: true` is refused on
every contract; the outdoor profile additionally refuses a disabled floor filter, either plane rule, a physical
ceiling, a disabled ground estimate and a non-null operator assertion. Every threshold is `{"value", "status":
"PROVISIONAL"}` with bounds; no other status is accepted yet. Existing contracts validate exactly as before (the
optional keys are absent; `CEILING_DEFAULT`/`GROUND_DEFAULT` unchanged).

*Estimator-height assertion outdoors:* not used, and refused if present. Reason: outdoors PX4 z is relative to the
take-off point (not "height above the floor" in general), so the indoor assertion would be false on uneven terrain;
the per-frame camera fit, cross-checked against PX4 within a tolerance, is a stronger and per-frame check. Indoors
nothing changed: an enabled filter still needs the assertion.

**B. Ground plane (`estimate_ground`, pure, deterministic).** Valid source pixels on a regular stride (about 20 000
samples), reprojected into the gravity-aligned yaw-level trained-camera frame exactly as `project_rgbd`; keep rays
from the real camera that point below the horizon with range <= `fit_max_range_m` (10 m); at most 2000 points,
evenly spaced in raster order. Sequential RANSAC (seed 0, 512 hypotheses per pass, MSAC score, near-horizontal
hypotheses within 30 deg below the camera, up to 3 planes) with two total-least-squares refinements. Per plane:
camera height, tilt, inliers, share, RMS, share of points below the plane. Checks in priority order: PX4 fields
present; `z_valid`; `epv <= 0.5 m`; not dead reckoning; support (>= 300 inliers and >= 25 %); tilt <= 3.0 deg; at
most 2 % of the points more than `0.06 + 0.015 x range` m below the plane; |height - PX4 camera height| <= 0.3 m.
Two consistent planes with support ratio >= 0.5 and heights > 0.15 m apart -> `ground_ambiguous`; otherwise the
consistent plane with the most support. Finally the verified camera height must lie in the floor/ceiling band.
Refusal -> `FrameRefused(reason, report)` before any history advances.

**Floor and virtual ceiling outdoors (`remove_floor_and_ceiling_plane`).** Height of a valid return above the
verified plane (normal and offset, not raw PX4 z). Floor: height < 0.10 m + z (0.5/fy + 0.015) (range term, see
deviation 1). Ceiling: height > 3.0 - 0.10 m. Sensor-invalid pixels are never touched by these filters.

**C. Sky rule (`apply_sky_rule`).** Sensor-invalid pixels only. With the verified plane (n, h) and the level ray
d = (1, -u, -v): meets iff n.d < 0, at optical z_g = h / -(n.d). Freed (no-return) if
(rule 2, far ground, no RGB) `meets & z_g > MaxZ` or `~meets & v > 0`; or (rule 1, sky) `~meets & v < 0 & luma >=
0.5 & gradient <= 0.03 & 4-connected to row 0` through rule-1 candidates or through unimaged / virtual-ceiling-removed
pixels above the horizon. Far-ground band width (level plane) = atan(h / MaxZ): **2.86 deg at h 1.0 m (4 trained
rows), 4.29 deg at h 1.5 m (7 rows)**; 2.95 deg / 5 rows at the 1.03 m camera of a 1.0 m flight
(`analysis/case_summary.json: far_ground_band`).

*Why ground-standing near obstacles stay detected:* an object standing on the ground at horizontal distance D < MaxZ
occupies every row from its base (depression atan(h/D) > atan(h/MaxZ)) upwards. The rows below the far-ground band
meet the ground inside MaxZ, so rule 2 does not apply there, and they are below the horizon, so rule 1 does not
apply either: they stay sensor-invalid (if invalid) and blob_near makes them near. Tested: trunk 0.3 m (84/84
silhouette columns near at all poses), white post 0.3 m (78/78), and the pre-existing F1 box (repair-r1 tests,
unchanged). Lost only: invalid parts lying wholly inside the far-ground band or above the horizon and sky-like.

**D. Training equivalence.** With the outdoor profile absent every path is the old code: `preprocess` branches to
`_preprocess_outdoor` only for `profile == "outdoor"`; `split_state` returns the state unchanged; `project_rgbd`'s six
default outputs are computed identically (extra mask only with `unimaged=True`). Checked three ways (section 9):
the repair-r1 golden SHA-256 of the real fc8aa4d observation (9 scenes x filters off/on, real seed-1 checkpoint),
a byte comparison against the pre-change runtime module (7 scenes x 4 indoor contracts: rgb, depth, counts,
projection, plus `project_rgbd` arrays), and the `unimaged` option leaving the six outputs byte-identical.

**Session (refusal -> hold).** See section 6.

**Replay tool.** `tools/oakd_outdoor_replay.py` (folder format in its docstring): rectified aligned RGB, depth uint16
mm (0 invalid), `up_optical` (IMU gravity in the optical frame) or `roll_pitch_deg`, timestamps, tape height and its
uncertainty; it runs the production outdoor `preprocess` without the checkpoint (production blob_near loaded
privately; the colour-target carve-out is not applied and is reported as `NOT_APPLIED`), writes `frames.jsonl` and
`summary.json` (refusals by reason, height error vs tape, tilt, sky/far-ground/near statistics) and labels synthetic
folders `SYNTHETIC_NOT_EVIDENCE`. No DepthAI import. Capture protocol: `ros2/motar_camera_policy/README.md`.

## 3. Deviations from the requested design, with reasons

1. **Range-dependent floor band (new PROVISIONAL `floor_noise_margin_per_m` = 0.015).** With the fixed 0.10 m band,
   noisy ground on the tilted (z-buffer) path leaked as valid returns 0.10-0.23 m above the plane at 4.5-10 m (544
   trained pixels at h 1.5 m, pitch +10 deg; `scratch/smoke2.py`), i.e. phantom obstacles inside the 10 m obstacle
   range on many bearings. Causes: stereo noise grows with range, the z-buffer keeps the nearest of about 40 noisy
   samples, and a trained pixel spans 0.65 deg (up to 0.057 m height error at 10 m). The band is now
   0.10 + z (0.5/fy + 0.015) m: about 0.20 m at 5 m and 0.31 m at 10 m. Cost: valid returns that low are removed
   (no collision risk at a 1.0 m flight altitude, but a real effect).
2. **Lowest-surface check (new PROVISIONAL `max_below_share` = 0.02, new reason `ground_not_lowest_surface`) and
   MSAC scoring.** With inlier counting, a tilted "bridging" plane through near ground and a far box top was
   accepted with height errors up to +0.15 m. MSAC plus the check removed most cases (box exploration: every accepted
   plane within 0.053 m, `scratch/explore_box.py`), but not all (section 5).
3. **`tilt_max_deg` = 3.0 instead of the suggested 5.** Budget: PX4 attitude error 2 deg (the test's stated value) +
   1 deg site slope. A wrong plane's tilt lifts the floor band by r tan(t) (0.87 m at 10 m for 5 deg); synthetic
   bridging planes had 3.3-3.8 deg. Consequence: a 5 deg slope is refused (accepted at 5 deg tolerance in 1 of 2
   heights; tilt 4.96 deg), a 10 deg slope is refused at either tolerance.
4. **Rule 1 connectivity also passes through unimaged pixels and virtual-ceiling-removed pixels above the horizon.**
   Without the first, a nose-down camera (its top band is not imaged) kept the whole sky as a wall; without the
   second, the sky below a removed 3.2 m canopy stayed a near wall on all 160 bearings.
5. **Rule 1 gradient is measured against neighbouring sensor-invalid pixels only.** Edges against valid objects
   (tree line, removed canopy) or unimaged black otherwise blocked the connection.
6. **Rule 2 for a tilted plane.** "Below the horizon" uses the plane: an invalid pixel is far ground if its ray meets
   the verified plane beyond MaxZ, or points below the gravity horizon and never meets it (a plane tilted away).
   Identical to the requested rule for a level plane; needed so that the band follows the actual ground horizon
   under a 2 deg attitude error.
7. **`dead_reckoning` is refused** (`refuse_dead_reckoning: true`), beside `z_valid` and `epv`.
8. **Ground fit uses source pixels, not the 160x90 trained grid** (more ground, exact rays, no z-buffer bias).
9. **Plane selection when the dominant plane fails a check:** a smaller plane that passes every check is accepted
   (this is how the "correct lowest plane" is found beside a box top that fails the PX4 or lowest-surface check).
10. **Synthetic sensor model choices (tests and analysis only):** disparity-domain noise (fB 60 px m, 0.15 px, ASSUMED)
    instead of a z^2 Gaussian (which produced unrealistic 1-8 m returns from 50 m ground); texture as brightness
    noise shared by the three channels (independent per-channel noise of 25-30 levels created scattered red pixels
    that the bootstrap red-target detector picked up and confounded the first policy comparison).

## 4. Results per case

Full table (178 rows: every case x heights 1.0/1.5 m x pitch x roll; before = shipped indoor default and outdoor with
the sky rule off; after = outdoor profile): `analysis/case_table.md` from `analysis/case_summary.json`
(`analysis/case_summary.py`). Condensed (near = trained columns whose minimum is < 1 m, of 160):

| Case | Variants (h, pitch, roll) | Before: indoor default | Before: outdoor, sky off | After: outdoor profile | Object columns after | Ground after |
|---|---|---|---|---|---|---|
| 1 sky above tree line 15 m | 22 (h 1/1.5; pitch -10/-5/+0/+5/+10; roll -5/+0/+5) | near 102-160, min 0.41-0.41 | near 102-160, min 0.41-0.41 | near 0-0, min 12.50-13.47 |  | abs(err) <= 0.006, tilt 0.00-0.06 |
| 2 open field, far ground valid | 10 (h 1/1.5; pitch -10/-5/+0/+5/+10; roll +0) | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 0-0, min 20.00-20.00 |  | abs(err) <= 0.010, tilt 0.01-0.09 |
| 2 open field, far ground invalid | 10 (h 1/1.5; pitch -10/-5/+0/+5/+10; roll +0) | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 0-0, min 20.00-20.00 |  | abs(err) <= 0.010, tilt 0.01-0.09 |
| 3 trunk 0.3 m (invalid, dark) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 88-90, min 0.40-0.40 | near 84/84/84/84/84/84 of 84; min 0.40-0.40 | abs(err) <= 0.008, tilt 0.01-0.08 |
| 3 trunk 1.5 m (valid) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 136-138, min 0.41-0.42 | near 136-138, min 0.41-0.42 | near 0-0, min 1.48-1.49 | near 0/0/0/0/0/0 of 18; min 1.48-1.49 | abs(err) <= 0.003, tilt 0.01-0.04 |
| 4 branch above camera 0.3 m (invalid, dark) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 160/160/160/160/160/160 of 160; min 0.40-0.40 | abs(err) <= 0.002, tilt 0.00-0.02 |
| 4 branch above camera 1.0 m (valid) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 1.00-1.01 | near 0/0/0/0/0/0 of 160; min 1.00-1.01 | abs(err) <= 0.004, tilt 0.01-0.04 |
| 4 white smooth object above horizon 0.3 m (invalid) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 160-160, min 0.40-0.41 | near 160-160, min 0.40-0.41 | near 0-0, min 13.02-13.26 | near 0/0/0/0/0/0 of 156; min 13.02-13.26 | abs(err) <= 0.002, tilt 0.01-0.03 |
| 7 white smooth post on ground 0.3 m (invalid) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 80-84, min 0.40-0.40 | near 78/78/78/78/78/78 of 78; min 0.40-0.40 | abs(err) <= 0.004, tilt 0.01-0.07 |
| 7 white smooth wall on ground 0.3 m, full width (invalid) | 6 (h 1/1.5; pitch -10/+0/+10; roll +0) | near 160-160, min 0.40-0.40 | refused 6/6 (ground_insufficient_points) | refused 6/6 (ground_insufficient_points) |  |  |
| 5 wire 2 m 1 px invalid | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.97-13.47 | near 0/0 of 156; min 12.97-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 2 m 1 px valid depth | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 2.04-13.47 | near 0/0 of 156; min 2.04-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 2 m 2 px invalid | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.97-13.47 | near 0/0 of 156; min 12.97-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 2 m 2 px valid depth | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 2.03-13.47 | near 0/0 of 156; min 2.03-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 2 m 3 px invalid | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.97-13.47 | near 0/0 of 156; min 12.97-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 2 m 3 px valid depth | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 2.03-13.47 | near 0/0 of 156; min 2.03-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 5 m 1 px invalid | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.97-13.47 | near 0/0 of 144; min 12.97-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 5 m 1 px valid depth | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 5.00-13.47 | near 0/0 of 144; min 5.00-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 5 m 2 px invalid | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.97-13.47 | near 0/0 of 144; min 12.97-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 5 m 2 px valid depth | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 4.99-13.47 | near 0/0 of 144; min 4.99-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 5 m 3 px invalid | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.97-13.47 | near 0/0 of 144; min 12.97-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 5 wire 5 m 3 px valid depth | 2 (h 1; pitch +0/+3; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 4.93-13.47 | near 0/0 of 144; min 4.93-13.47 | abs(err) <= 0.001, tilt 0.01-0.02 |
| 6 canopy underside 2.0 m valid | 1 (h 1; pitch +0; roll +0) | near 0-0, min 2.14-2.14 | near 0-0, min 2.14-2.14 | near 0-0, min 2.14-2.14 |  | abs(err) <= 0.001, tilt 0.01-0.01 |
| 6 canopy underside 2.0 m invalid dark | 1 (h 1; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 |  | abs(err) <= 0.001, tilt 0.01-0.01 |
| 6 canopy underside 2.5 m valid | 1 (h 1; pitch +0; roll +0) | near 0-0, min 3.23-3.23 | near 0-0, min 3.23-3.23 | near 0-0, min 3.23-3.23 |  | abs(err) <= 0.001, tilt 0.01-0.01 |
| 6 canopy underside 2.5 m invalid dark | 1 (h 1; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 |  | abs(err) <= 0.001, tilt 0.01-0.01 |
| 6 canopy underside 3.2 m valid | 1 (h 1; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.69-13.69 |  | abs(err) <= 0.001, tilt 0.02-0.02 |
| 6 canopy underside 3.2 m invalid dark | 1 (h 1; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 |  | abs(err) <= 0.001, tilt 0.02-0.02 |
| 8 flat textured ground | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.42-13.47 |  | abs(err) <= 0.001, tilt 0.02-0.02 |
| 8 textureless ground | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.40-0.40 | refused 2/2 (ground_insufficient_points) | refused 2/2 (ground_insufficient_points) |  |  |
| 8 slope 2 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.42-13.47 |  | abs(err) <= 0.005, tilt 1.98-2.02 |
| 8 slope 2 deg (tilt tolerance 5 deg) | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.42-13.47 |  | abs(err) <= 0.005, tilt 1.98-2.02 |
| 8 slope 5 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_tilt_exceeds_tolerance) | refused 2/2 (ground_tilt_exceeds_tolerance) |  |  |
| 8 slope 5 deg (tilt tolerance 5 deg) | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41; refused 1/2 (ground_tilt_exceeds_tolerance) | near 0-0, min 13.47-13.47; refused 1/2 (ground_tilt_exceeds_tolerance) |  | abs(err) <= 0.015, tilt 4.96-4.96 |
| 8 slope 10 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_tilt_exceeds_tolerance) | refused 2/2 (ground_tilt_exceeds_tolerance) |  |  |
| 8 slope 10 deg (tilt tolerance 5 deg) | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_tilt_exceeds_tolerance) | refused 2/2 (ground_tilt_exceeds_tolerance) |  |  |
| 8 PX4 z drift +0.2 m | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.42-13.47 |  | abs(err) <= 0.201, tilt 0.02-0.02 |
| 8 PX4 z drift +0.5 m | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_height_disagrees_with_px4) | refused 2/2 (ground_height_disagrees_with_px4) |  |  |
| 8 PX4 z drift -0.2 m | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.42-13.47 |  | abs(err) <= 0.200, tilt 0.02-0.02 |
| 8 PX4 z drift -0.5 m | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_height_disagrees_with_px4) | refused 2/2 (ground_height_disagrees_with_px4) |  |  |
| 8 z_valid false | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (estimator_z_invalid) | refused 2/2 (estimator_z_invalid) |  |  |
| 8 epv 2.0 m | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (estimator_epv_exceeds_limit) | refused 2/2 (estimator_epv_exceeds_limit) |  |  |
| 8 dead reckoning | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (estimator_dead_reckoning) | refused 2/2 (estimator_dead_reckoning) |  |  |
| 8 vertical fields absent | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (estimator_vertical_fields_absent) | refused 2/2 (estimator_vertical_fields_absent) |  |  |
| 8 low wide box hiding the ground (x1.5-8, top 0.2) | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 13.42-13.47 |  | abs(err) <= 0.202, tilt 0.01-0.03 |
| 8 low wide box, ground visible (x3-8, top 0.2) | 4 (h 1/1.5; pitch +0/+10; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41; refused 1/4 (ground_not_lowest_surface) | near 0-0, min 2.95-3.12; refused 1/4 (ground_not_lowest_surface) |  | abs(err) <= 0.110, tilt 0.14-2.80 |
| 8 low wide box hiding the ground (x1.5-8, top 0.3) | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_height_disagrees_with_px4) | refused 2/2 (ground_height_disagrees_with_px4) |  |  |
| 8 low wide box, ground visible (x3-8, top 0.3) | 4 (h 1/1.5; pitch +0/+10; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41; refused 2/4 (ground_not_lowest_surface, ground_tilt_exceeds_tolerance) | near 0-0, min 2.93-3.08; refused 2/4 (ground_not_lowest_surface, ground_tilt_exceeds_tolerance) |  | abs(err) <= 0.031, tilt 0.42-0.81 |
| 8 low wide box hiding the ground (x1.5-8, top 0.6) | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | refused 2/2 (ground_height_disagrees_with_px4, ground_not_lowest_surface) | refused 2/2 (ground_height_disagrees_with_px4, ground_not_lowest_surface) |  |  |
| 8 low wide box, ground visible (x3-8, top 0.6) | 4 (h 1/1.5; pitch +0/+10; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41; refused 1/4 (ground_not_lowest_surface) | near 0-0, min 2.87-2.98; refused 1/4 (ground_not_lowest_surface) |  | abs(err) <= 0.054, tilt 0.44-1.39 |
| 8 attitude error roll +0 pitch +2 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.81-12.92 |  | abs(err) <= 0.003, tilt 2.00-2.01 |
| 8 attitude error roll +0 pitch -2 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.42 | near 0-0, min 12.81-12.92 |  | abs(err) <= 0.004, tilt 1.99-2.00 |
| 8 attitude error roll +2 pitch +0 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.81-12.92 |  | abs(err) <= 0.001, tilt 1.99-2.02 |
| 8 attitude error roll -2 pitch +0 deg | 2 (h 1/1.5; pitch +0; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 0-0, min 12.81-12.92 |  | abs(err) <= 0.001, tilt 1.98-2.01 |
| 9 partly cloudy (textured bright clouds) | 3 (h 1; pitch -10/+0/+10; roll +0) | near 160-160, min 0.41-0.41 | near 160-160, min 0.41-0.41 | near 51-68, min 0.42-0.42 |  | abs(err) <= 0.002, tilt 0.01-0.04 |
| 10 sun/flare blob 8 deg below horizon (invalid, saturated) | 3 (h 1; pitch -10/+0/+10; roll +0) | near 160-160, min 0.40-0.40 | near 160-160, min 0.40-0.40 | near 12-12, min 0.40-0.40 |  | abs(err) <= 0.002, tilt 0.01-0.03 |

Pose sweep (pitch -10..+10 step 5, roll -5/0/+5, h 1.0/1.5): {"accepted": 30, "refused": 0, "reasons": {}}
Far-ground band: {"1.0": {"angle_deg": 2.862, "rows": 4}, "1.03": {"angle_deg": 2.948, "rows": 5}, "1.5": {"angle_deg": 4.289, "rows": 7}, "1.53": {"angle_deg": 4.375, "rows": 7}}

Unit-test pins of the same cases: `tests/test_camera_runtime_outdoor_r1.py` (`SkyRule`, `NearObstaclesStayDetected`,
`MeasuredLimitations`, `GroundEstimate`).

## 5. Ground-reliability assessment (camera + PX4 only)

Trusted (synthetic, this emulation): textured flat ground visible within 10 m; camera height 1.0-1.5 m; pitch -10..+10
deg and roll -5..+5 deg (30/30 accepted, |error| <= 0.01 m, tilt <= 0.1 deg); PX4 attitude error +-2 deg (accepted,
tilt 2.0 deg, |error| <= 0.004 m); PX4 z drift +-0.2 m (accepted: the camera, not PX4, sets the height; the 0.2 m
discrepancy is reported); slopes up to about 2 deg.

Refused (fail closed, named reason, zero hold in the ROS session): textureless ground (`ground_insufficient_points`);
a wall or object hiding the lower view; slopes of 5 deg (at the 3 deg tolerance) and 10 deg (`ground_tilt_...`);
PX4 drift +-0.5 m (`ground_height_disagrees_with_px4`); `z_valid` false, `epv` 2 m, dead reckoning, fields absent
(the ROS path today, see section 6); most step/box geometries (`ground_not_lowest_surface`, `ground_tilt_...`).

Not trustworthy and NOT refused (residual):
- **A wide horizontal surface hiding all ground inside the fit range** is accepted as the ground when its top lies
  within the PX4 tolerance: box top 0.2 m -> accepted with error -0.20 m (both heights); 0.3 m at h 1.0 accepted at
  the tolerance edge in exploration, refused in the case table; 0.6 m refused. The camera cannot see the ground; only
  PX4's 0.3 m tolerance bounds the error.
- **Bridging planes across a step** still pass in 2 of 12 box-with-visible-ground variants: +0.110 m / tilt 2.80 deg
  and +0.069 m / 2.15 deg (h 1.0 m, box top 0.2 m, pitch 0 and +10; `case_summary.json`). Effect: floor band and
  far-ground band shifted by the spurious tilt (up to ~0.5 m at 10 m; the band can move up to ~3 deg above the
  horizon, so an invalid horizontal structure at camera height inside that band would be freed).
- Real OAK-D noise, outliers, calibration error, vegetation (grass height) and IMU/EKF attitude accuracy are
  unknown: every threshold is PROVISIONAL until the replay tool is run on the user's recordings.

## 6. Refusals: session, node, gateway and PX4

Before this unit (code reading; PX4 v1.14.3 source `src/modules/commander`, `mc_pos_control`): any `InputError` from
`runtime.step` faulted the ROS session (`fail_inference` -> latched FAULT, explicit reset needed); the node stopped
publishing proposals; the gateway's next 10 Hz update found the command receipt older than 0.15 s, revoked (latched)
and stopped both OffboardControlMode and TrajectorySetpoint. **No zero or hold command was sent.** PX4 keeps the last
received TrajectorySetpoint (mc_pos_control has no setpoint staleness check) until `offboard_control_signal_lost`
(last OffboardControlMode + `COM_OF_LOSS_T`, default 1.0 s), then `COM_OBL_RC_ACT` (default Position mode) after
`COM_FAIL_ACT_T` hold logic. So a refusal left the **last nonzero velocity in effect for about 1.0-1.25 s** (up to
3.5 m/s horizontally). This is a finding; it applies to every session fault and to a stopped node, not only to
refusals. NOT_TESTED in SITL or on the vehicle.

Fixed in the session layer for the outdoor refusals (`FrameRefused` only): the tick produces an explicit
zero-velocity, zero-yaw-rate proposal with the frame's capture stamps (valid 0.15 s), status `HOLD_FRAME_REFUSED`,
`refusal` and `consecutive_refusals` in the diagnostics, and the next accepted frame starts a fresh runtime history.
The gateway publishes it as zero XY velocity with its altitude hold (tested), within one 0.1 s tick. Every other
`InputError` still faults. Open (not in scope, reported): the generic fault path keeps the 1 s stale-setpoint window;
shorten `COM_OF_LOSS_T` for policy flights and consider a gateway brake-on-command-loss. The hold has no time limit
(the vehicle hovers in OFFBOARD until frames verify or the pilot takes over): a user decision.

PX4 vertical validity: the gateway's `decode_estimator` already revokes on `z_valid` false but does not decode `epv`
or `dead_reckoning`, and its republished Odometry has NaN covariance. They reach the runtime only as extra outdoor
state fields (`split_state`). **Through the ROS node today every outdoor frame is refused with
`estimator_vertical_fields_absent` (hold): the profile fails closed there until the gateway forwards these fields.**

## 7. Observation and actual-policy comparison

No real OAK-D data exists here; scenes are synthetic. (i) TRAINING-EQUIVALENT: `trained_equal` source, level camera,
exact depth, no ground and no sky (+inf, no-return), z < MinZ invalid, objects wholly above 2.9 m omitted, filters
off. (ii) OAK-D-LIKE BEFORE: emulated OAK-D frame at the real pose, shipped default contract (floor filter on, CPU
fixture assertion, exact PX4 z). (iii) OAK-D-LIKE AFTER: same frame, outdoor profile. A red target sphere at
(6, 1, camera height). Real CAM-K2 v3 checkpoints, seeds 1 and 2, read-only through `CameraPolicyRuntime`, CPU,
30 frames at 10 Hz per variant; final step compared (`analysis/obs_policy_compare.py`,
`obs_policy_compare_s1.json`, `_s2.json`; full table `analysis/policy_table.md`). Column metrics are per-bearing
minima of the processed depth clipped at the 10 m obstacle range; scan metrics are the 288 observation scan slots in
metres; obs % = share of the 898 entries that differ (> 1e-6).

| Scene | Seed | (i) v_FLU m/s; yaw rad/s; near | (ii) BEFORE: near; column L1 / Linf m; obs %; dv m/s / ddir deg / dyaw rad/s | (iii) AFTER: same |
|---|---|---|---|---|
| tree line 15 m | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 15%; 2.87 / 73 / 2.09 | 0; 0.00 / 0.12; 4%; 0.00 / 0 / 0.00 |
| tree line 15 m | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 15%; 2.31 / 60 / 1.57 | 0; 0.00 / 0.12; 4%; 0.00 / 0 / 0.00 |
| tree line 15 m pitch +5 | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 17%; 2.86 / 73 / 2.08 | 0; 0.06 / 4.23; 7%; 0.02 / 0 / 0.01 |
| tree line 15 m pitch +5 | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 17%; 2.31 / 60 / 1.57 | 0; 0.06 / 4.23; 7%; 0.02 / 0 / 0.01 |
| tree line 15 m pitch -5 | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 16%; 2.86 / 73 / 2.08 | 0; 0.06 / 4.17; 6%; 0.01 / 0 / 0.00 |
| tree line 15 m pitch -5 | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 16%; 2.31 / 60 / 1.57 | 0; 0.06 / 4.17; 6%; 0.01 / 0 / 0.01 |
| open field, far ground valid | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.42 / 9.60; 16%; 2.87 / 73 / 2.09 | 0; 0.00 / 0.12; 5%; 0.00 / 0 / 0.00 |
| open field, far ground valid | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.42 / 9.60; 16%; 2.32 / 60 / 1.58 | 0; 0.00 / 0.12; 5%; 0.00 / 0 / 0.00 |
| open field, far ground invalid | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.42 / 9.60; 16%; 2.87 / 73 / 2.09 | 0; 0.00 / 0.12; 5%; 0.00 / 0 / 0.00 |
| open field, far ground invalid | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.42 / 9.60; 16%; 2.32 / 60 / 1.58 | 0; 0.00 / 0.12; 5%; 0.00 / 0 / 0.00 |
| open field h1.5 pitch +5 | 1 | (2.42, -0.98); -1.51; 0 | 160; 9.42 / 9.60; 16%; 2.63 / 68 / 1.69 | 0; 0.01 / 0.21; 6%; 0.00 / 0 / 0.01 |
| open field h1.5 pitch +5 | 2 | (2.45, -0.57); -1.39; 0 | 160; 9.42 / 9.60; 16%; 2.18 / 56 / 1.36 | 0; 0.01 / 0.21; 6%; 0.01 / 0 / 0.01 |
| bars in open field (golden-like) | 1 | (-1.53, 1.29); 1.69; 0 | 151; 8.81 / 9.58; 18%; 0.12 / 3 / 0.13 | 0; 0.01 / 0.28; 8%; 0.00 / 0 / 0.00 |
| bars in open field (golden-like) | 2 | (-1.53, 1.28); 1.29; 0 | 151; 8.81 / 9.58; 18%; 0.09 / 3 / 0.19 | 0; 0.01 / 0.28; 8%; 0.00 / 0 / 0.00 |
| bars in open field pitch +3 | 1 | (-1.53, 1.29); 1.69; 0 | 152; 8.82 / 9.58; 18%; 0.12 / 3 / 0.13 | 0; 0.09 / 7.10; 9%; 0.00 / 0 / 0.00 |
| bars in open field pitch +3 | 2 | (-1.53, 1.28); 1.29; 0 | 152; 8.82 / 9.58; 18%; 0.09 / 3 / 0.19 | 0; 0.09 / 7.10; 9%; 0.00 / 0 / 0.00 |
| bars in open field h1.5 | 1 | (-1.57, 1.24); 1.64; 0 | 151; 8.81 / 9.59; 18%; 0.18 / 5 / 0.11 | 0; 0.01 / 0.32; 9%; 0.00 / 0 / 0.00 |
| bars in open field h1.5 | 2 | (-1.56, 1.25); 1.35; 0 | 151; 8.81 / 9.59; 18%; 0.15 / 4 / 0.25 | 0; 0.01 / 0.32; 9%; 0.00 / 0 / 0.00 |
| trunk 0.3 m | 1 | (-1.47, 1.35); 1.67; 88 | 160; 4.29 / 9.56; 5%; 0.01 / 0 / 0.21 | 90; 0.12 / 9.56; 0%; 0.00 / 0 / 0.00 |
| trunk 0.3 m | 2 | (-1.49, 1.34); 1.29; 88 | 160; 4.29 / 9.56; 5%; 0.01 / 0 / 0.43 | 90; 0.12 / 9.56; 0%; 0.00 / 0 / 0.00 |
| trunk 1.5 m | 1 | (1.98, -0.27); -1.13; 0 | 138; 8.09 / 9.58; 22%; 1.88 / 56 / 2.00 | 0; 0.00 / 0.13; 7%; 0.00 / 0 / 0.00 |
| trunk 1.5 m | 2 | (2.00, 0.13); -1.02; 0 | 138; 8.09 / 9.58; 22%; 1.44 / 42 / 1.72 | 0; 0.00 / 0.13; 8%; 0.00 / 0 / 0.00 |
| branch above camera 0.3 m | 1 | (1.37, 1.46); 0.56; 160 | 160; 0.00 / 0.00; 6%; 0.00 / 0 / 0.00 | 160; 0.00 / 0.00; 6%; 0.00 / 0 / 0.00 |
| branch above camera 0.3 m | 2 | (1.44, 1.39); 0.22; 160 | 160; 0.00 / 0.00; 6%; 0.00 / 0 / 0.00 | 160; 0.00 / 0.00; 6%; 0.00 / 0 / 0.00 |
| branch above camera 1.0 m | 1 | (1.41, 1.41); 0.44; 0 | 160; 0.64 / 0.71; 13%; 0.06 / 2 / 0.11 | 0; 0.00 / 0.01; 12%; 0.00 / 0 / 0.00 |
| branch above camera 1.0 m | 2 | (1.47, 1.35); 0.11; 0 | 160; 0.64 / 0.71; 13%; 0.05 / 1 / 0.10 | 0; 0.00 / 0.01; 12%; 0.00 / 0 / 0.00 |
| white object above horizon 0.3 m | 1 | (1.37, 1.46); 0.56; 160 | 160; 0.00 / 0.00; 6%; 0.00 / 0 / 0.00 | 0; 9.41 / 9.60; 16%; 2.87 / 73 / 2.09 |
| white object above horizon 0.3 m | 2 | (1.44, 1.39); 0.22; 160 | 160; 0.00 / 0.00; 6%; 0.00 / 0 / 0.00 | 0; 9.41 / 9.60; 16%; 2.32 / 60 / 1.58 |
| wire 2 m 3 px (invalid) | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 15%; 2.87 / 73 / 2.09 | 0; 0.00 / 0.12; 4%; 0.00 / 0 / 0.00 |
| wire 2 m 3 px (invalid) | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 15%; 2.31 / 60 / 1.57 | 0; 0.00 / 0.12; 4%; 0.00 / 0 / 0.00 |
| canopy 2.0 m valid | 1 | (1.50, 1.32); 0.19; 0 | 0; 0.01 / 0.03; 13%; 0.00 / 0 / 0.00 | 0; 0.01 / 0.03; 13%; 0.00 / 0 / 0.00 |
| canopy 2.0 m valid | 2 | (1.55, 1.27); -0.11; 0 | 0; 0.01 / 0.03; 13%; 0.00 / 0 / 0.00 | 0; 0.01 / 0.03; 13%; 0.00 / 0 / 0.00 |
| canopy 2.0 m invalid dark | 1 | (1.50, 1.32); 0.19; 0 | 160; 1.84 / 2.01; 14%; 0.18 / 5 / 0.37 | 160; 1.84 / 2.01; 14%; 0.18 / 5 / 0.37 |
| canopy 2.0 m invalid dark | 2 | (1.55, 1.27); -0.11; 0 | 160; 1.84 / 2.01; 14%; 0.16 / 5 / 0.33 | 160; 1.84 / 2.01; 14%; 0.16 / 5 / 0.33 |
| canopy 3.2 m valid | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 16%; 2.86 / 73 / 2.09 | 0; 0.00 / 0.18; 5%; 0.00 / 0 / 0.00 |
| canopy 3.2 m valid | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 16%; 2.31 / 60 / 1.57 | 0; 0.00 / 0.18; 6%; 0.00 / 0 / 0.00 |
| white post on ground 0.3 m | 1 | (-1.47, 1.35); 1.67; 82 | 160; 4.65 / 9.56; 5%; 0.01 / 0 / 0.21 | 82; 0.00 / 0.00; 7%; 0.00 / 0 / 0.00 |
| white post on ground 0.3 m | 2 | (-1.49, 1.34); 1.29; 82 | 160; 4.65 / 9.56; 5%; 0.01 / 0 / 0.43 | 82; 0.00 / 0.00; 7%; 0.00 / 0 / 0.00 |
| white wall full width 0.3 m | 1 | (-1.48, 1.34); 1.88; 160 | 160; 0.00 / 0.00; 0%; 0.00 / 0 / 0.00 | refused `ground_insufficient_points` -> zero hold; dv 2.00 |
| white wall full width 0.3 m | 2 | (-1.49, 1.33); 1.72; 160 | 160; 0.00 / 0.00; 0%; 0.00 / 0 / 0.00 | refused `ground_insufficient_points` -> zero hold; dv 2.00 |
| partly cloudy | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 16%; 2.86 / 73 / 2.09 | 67; 3.85 / 9.58; 17%; 2.45 / 60 / 1.45 |
| partly cloudy | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 16%; 2.31 / 60 / 1.57 | 67; 3.85 / 9.58; 17%; 1.99 / 50 / 1.17 |
| sun blob below horizon | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.41 / 9.60; 15%; 2.87 / 73 / 2.09 | 12; 0.72 / 9.60; 9%; 1.10 / 21 / 0.45 |
| sun blob below horizon | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.41 / 9.60; 15%; 2.31 / 60 / 1.57 | 12; 0.72 / 9.60; 9%; 1.00 / 21 / 0.37 |
| low wide box hiding ground (top 0.2) | 1 | (1.47, 1.35); 0.26; 0 | 160; 1.50 / 1.64; 13%; 0.15 / 4 / 0.29 | 0; 7.90 / 8.16; 15%; 2.73 / 69 / 1.80 |
| low wide box hiding ground (top 0.2) | 2 | (1.52, 1.30); -0.04; 0 | 160; 1.50 / 1.64; 13%; 0.12 / 4 / 0.26 | 0; 7.90 / 8.16; 15%; 2.20 / 56 / 1.32 |
| slope 5 deg | 1 | (2.43, -1.21); -1.54; 0 | 160; 9.40 / 9.58; 15%; 2.87 / 73 / 2.09 | refused `ground_tilt_exceeds_tolerance` -> zero hold; dv 2.71 |
| slope 5 deg | 2 | (2.45, -0.69); -1.36; 0 | 160; 9.40 / 9.58; 15%; 2.31 / 60 / 1.57 | refused `ground_tilt_exceeds_tolerance` -> zero hold; dv 2.55 |

Reading (numbers from `obs_policy_compare_s1.json` / `_s2.json`):
- **Before the fix (ii)** the observation has a near wall on 138-160 of 160 bearings in 23 of 24 scenes (all except
  the valid canopy at 2.0 m, which hides the sky). In the 12 scenes whose training-equivalent view has no near
  obstacle and no canopy cover (tree line, open field, trunk at 1.5 m, wire, canopy 3.2 m, clouds, sun, slope) the
  commanded velocity differs from (i) by 1.88-2.87 m/s (seed 1) and 1.44-2.32 m/s (seed 2), 42-73 deg in direction.
  Where (i) is itself blocked (near trunk, branch, post, wall) or the policy happens to give the same escape command
  (bars, whose target is occluded), (ii) differs by at most 0.18 m/s.
- **After the fix (iii)** 17 of 24 scenes match (i) within 0.017 / 0.018 m/s and 0.010 / 0.011 rad/s (seed 1 / 2):
  every sky, open-field, bars, trunk, branch, wire, valid-canopy and white-post scene. The other 7: the white object
  above the horizon is lost (2.87 / 2.32 m/s); textured clouds (2.45 / 1.99) and the sun blob (1.10 / 1.00) leave
  phantom near bearings; the low box hiding the ground (2.73 / 2.20) is an obstacle in (i), because training had no
  ground and (i) keeps every surface, while (iii) takes its top as the ground; the invalid dark canopy at 2.0 m stays
  near in both (ii) and (iii) (0.18 / 0.16, as intended); the full-width white wall and the 5 deg slope are refused
  (zero hold, so the difference is the speed of (i)).
- The share of differing observation entries is 0-13 % in the 17 matching scenes (target range and stereo noise in the
  target features); it is not by itself a measure of behavioural difference.

## 8. Measured CPU cost (this workstation)

| Source frame | estimate_ground median / p95 ms | apply_sky_rule median / p95 ms | existing project_rgbd median / p95 ms |
|---|---|---|---|
| oakd_like | 3.85 / 4.83 (n 90) | 0.76 / 0.81 (n 90) | 105.66 / 111.98 (n 30) |
| trained_equal | 3.60 / 3.96 (n 90) | 0.83 / 0.93 (n 90) | 2.05 / 2.25 (n 30) |

13th Gen Intel(R) Core(TM) i7-13700K, one thread (OMP/MKL/OpenBLAS 1), idle host, synthetic frames (`analysis/timing.py`, `timing.json`). Orin NX: NOT_MEASURED. The plane floor filter adds a few vectorised operations on 160x90 (not separately timed).

## 9. Verification

- New module `tests/test_camera_runtime_outdoor_r1.py`: 37 tests (12 classes; 3 run the real seed-1 checkpoint in one fresh CPU process). OK in the final PUBLIC_CPU run 3.
- Focused run of the 13 camera/ROS test modules in one process (before the 3 records tests were added): `Ran 191 tests in 69.6 s, OK` (`logs/focused_outdoor_r1.log`). Includes the repair-r1 golden test: the real seed-1 observation equals the fc8aa4d SHA-256 for 9 scenes with filters off and on.
- PUBLIC_CPU run 1 (shell without OMP/MKL variables): `Ran 4181 tests, FAILED (errors=3, skipped=10)` (`logs/public_cpu_outdoor_r1.log`). The 3 errors are the SITL attempt guards `test_sitl_agq_geometry_diag.Unit` (2) and `test_sitl_s3b_dev08_prefix.ProductionThreads` (1): "OMP_NUM_THREADS must be 1; MKL_NUM_THREADS must be 1". Reproduced with the two modules alone in the same shell, and on the pre-change copy; they pass alone with `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1` (the command the repair-r1 baseline used). Environment precondition, not this change.
- PUBLIC_CPU run 2 (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= nice -n 10 timeout 3600 python -B tools/run_tests.py`): `Ran 4182 tests, FAILED (errors=1, skipped=10)` (`logs/public_cpu_outdoor_r1_run2.log`): `test_occlusion_confirmatory.SupervisionTest.test_term_ignoring_grandchild_dies_when_group_leader_exits`, ProcessLookupError while reading /proc/<pid>/stat of an exiting grandchild (the test catches only FileNotFoundError): a race in an unrelated committed test; it passed in runs 1 and 3. Not edited.
- **PUBLIC_CPU run 3 (same command as run 2): `Ran 4182 tests in 825.093s`, `OK (skipped=10)`** (`logs/public_cpu_outdoor_r1_run3.log`): the 4145-test baseline plus the 37 new tests; 0 failures, 0 errors.
- Kernel faults (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`): 0 before and 0 after the focused run and every PUBLIC_CPU run (`logs/kernel_faults_*.txt`); cpu4/5 offline and not used.
- New tests on the pre-change tree (`pretree_run/`: the pre_tree files over a copy of tools/tests/ros2, other paths symlinked; log `logs/new_tests_on_pre_change_tree.log`): `Ran 34 tests, FAILED (failures=2, errors=34)`: every test fails, mostly on the missing API (28 AttributeError, 2 ModuleNotFoundError for the replay tool, 1 TypeError for `unimaged`, 1 ImportError) plus the README checks and the actual-policy setUpClass. The behavioural "before" (sky = 160/160 near bearings) is shown inside the module by the indoor-contract and sky-off controls, whose code path is byte-identical to the pre-change runtime (`OutdoorOffIsBitIdentical`: 7 scenes x 4 indoor contracts compared with the pre_tree runtime module).
- Training equivalence (D): golden SHA-256 of the real fc8aa4d observation unchanged (filters off and on); `OutdoorOffIsBitIdentical` byte-equal to the pre-change runtime; the `unimaged` option leaves the six projection outputs byte-identical.
- Checkpoints read only through `build_contract`/`CameraPolicyRuntime`; SHA-256 prefixes after the runs: seed 1 `888e8056b2a67683`, seed 2 `203deea459ef1d93` (equal to the pins).

## 10. Limitations and residual risks that block an outdoor GO

1. No real OAK-D data: sky appearance, stereo noise and outliers, vegetation, auto-exposure, IMU gravity and EKF
   attitude accuracy are unmeasured; every threshold is PROVISIONAL. Run the replay tool on the user's recordings.
2. ROS path: PX4 `epv`/`dead_reckoning` are not forwarded, so outdoor frames are refused (hold) on the node.
3. Thin wires (1-3 px) reported invalid are never detected; valid-depth wires are detected only on the tilted path.
4. A bright, smooth near object above the horizon that touches the sky is lost (policy command differs by
   2.3-2.9 m/s from the training-equivalent one).
5. Textured bright clouds, a saturated sun blob below the far-ground band and an invalid canopy above the virtual
   ceiling stay near (phantom obstacles); textureless ground inside MaxZ is refused.
6. Ground model: one plane, flat site required; slopes > 3 deg refused; a horizontal surface hiding all ground is
   accepted within the PX4 tolerance (0.2 m error); bridging planes across steps still pass in 2 of 12 variants
   (0.07-0.11 m, 2.2-2.8 deg), shifting the floor and far-ground bands.
7. Rule 2 frees invalid pixels whose rays meet the ground beyond MaxZ without an RGB test: an invalid object lying
   wholly inside that band (from the horizon down to about 2.9-4.4 deg below it at camera heights 1.03-1.53 m, or the
   band shifted by a wrong plane tilt) is lost.
8. Refusal holds are unbounded in time; the generic fault path still leaves PX4's last setpoint for `COM_OF_LOSS_T`.
9. The existing 1014x760 z-buffer projection costs about 105 ms on this workstation (pre-existing; Orin NX not
   measured), above a 0.1 s control period by itself.
10. The outdoor profile has not been reviewed independently (AGENTS.md: an independent Opus review is required before
    any GO); the gateway geofence, wind and braking conditions of the protocol are unchanged and still open.

## Git traceability

- Branch: `camonly/dev`; Base SHA: `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`; New commit SHA: none (no commit by
  instruction).
- Changed files: section 1 (6 modified by this unit among the 24 pre-existing uncommitted paths, 2 new untracked).
  Remaining uncommitted paths: the 24 of the repair series plus `tests/test_camera_runtime_outdoor_r1.py` and
  `tools/oakd_outdoor_replay.py` (`logs/git_status_after.txt`).
- Tests: section 9. Working tree: uncommitted by instruction. Remote push: NO. Main merge: NO.
- Raw/checkpoint integrity: checkpoints read through `build_contract`/`CameraPolicyRuntime` only (SHA-256 verified
  against the pins by `validate_contract`); nothing written under the checkpoint folders, `results/` or `aerial_gym/`.
