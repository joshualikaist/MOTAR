# Camera-runtime outdoor profile, repair r1 (2026-11-13): answers to the independent review r1

Worker: Claude Code subagent (Opus 5.5, `claude-opus-5-5`). Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`,
branch `camonly/dev`, HEAD `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde` (unchanged). Usage: `USAGE_UNKNOWN` (no
provider-visible signal; no estimate made). The 26 uncommitted paths were preserved and only edited forward: the 8
outdoor-r1 files were first snapshotted to `../r1_tree/files/` with `../r1_tree/SHA256SUMS` (before any edit); 7 of them
changed (`tools/oakd_outdoor_replay.py` did not need to). No commit, `git add`, stash, reset, checkout, push or merge. Nothing under
`aerial_gym/`, `results/`, checkpoints, raw data, WORKLOG, Beads, the gateway files `tools/camera_ros2_px4.py` /
`tools/camera_ros2_px4_node.py` or any settings file was touched. CPU only (`CUDA_VISIBLE_DEVICES=""`,
`PYTHONNOUSERSITE=1`, `python -B`, `taskset -c 0-3,6-23`, `timeout -k`), checkpoints read only.

**Evidence status: SYNTHETIC.** Every scene is the ray-cast OAK-D emulation of `tests/test_camera_runtime_outdoor_r1.py`
(`outdoor_scene`) with ASSUMED sensor values. The "realistic twins" add the assumption that the OAK-D's horizontal
stereo baseline matches an object's vertical silhouette edges (valid strips 0.03-0.05 m wide) while its bright face is
invalid; that assumption is NOT_MEASURED on the user's OAK-D. Nothing here is real-sensor, PX4, ROS-graph, Gazebo or
flight evidence. **No flight-readiness verdict is given.** Every threshold added is PROVISIONAL.

Paths: a relative path in this report is relative to `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/repair_r1/`
(so `../logs/`, `../r1_tree/`, `../review_r1/` and `../analysis/` (r1's) are siblings in the unit folder).

## Design decisions (what changed and why)

1. **Virtual ceiling never touches invalid pixels.** `apply_sky_rule` lost its `above_ceiling` argument; the flood
   passes only through sky candidates and through `unimaged` pixels, which on the tilted path are now restricted to
   trained pixels whose ray lies OUTSIDE the source camera's field of view (or its uncovered border); a disocclusion or
   sampling gap inside the FOV is no longer a connector. `remove_floor_and_ceiling_plane` still returns `roof`, for
   counting only. Contract comment and `SKY_CONNECTIVITY` (`top_border_4_through_sky_or_unimaged`, value kept: it now
   describes the runtime exactly) say so.
2. **Far-ground rule opt-in.** New contract field `sky.far_ground_rule` = `{"value": false, "status": "PROVISIONAL"}`
   (JSON boolean, validated). Off: no sensor-invalid pixel below the horizon is freed. README note: enabling it needs
   real OAK-D evidence that far ground drops out to depth 0 AND the user's explicit acceptance of the loss of thin
   invalid obstacles in the altitude band.
3. **Edge-enclosure veto** (`camera_policy_runtime.edge_enclosure`). Row and column scans through sensor-invalid
   pixels; both first non-invalid pixels must be near edges: valid returns of the projection (before the floor/ceiling
   filters; floor returns excluded; ceiling-removed returns count) with range <= `edge_veto_range_m` 5.0 m, subtending
   <= `edge_veto_span_deg` 70 deg, ranges within `edge_veto_range_tolerance_m` 1.0 m. The enclosed set then grows by
   `EDGE_VETO_DILATION_PX` = 2 invalid 4-neighbour steps (rows whose edge sample is unresolved, e.g. at the FOV boundary
   of a nose-down camera). Enclosed pixels are neither sky candidates (they also block the flood) nor freed far ground.
   All PROVISIONAL. R_edge = the trained governor's release distance (training env `NAVRL_SPEED_GOVERNOR_RELEASE_M`
   5.0) > worst-case stop 3.2 m (3.54 m/s free cap, 2.96 m/s^2, 0.3 s); span covers a 2.4 m face at 2 m (64.6 deg).
4. **Ground wrong-surface checks** in `estimate_ground` (contract thresholds, PROVISIONAL): `rival_min_share` 0.10
   (any other horizontal plane below the camera, tilt <= 30 deg after refinement, at > 0.15 m height difference ->
   `ground_ambiguous`); `min_half_share` 0.10 and `min_range_span_m` 1.0 (-> `ground_inlier_spread_insufficient`);
   `planarity_max_angle_deg` 1.0 for the TLS planes of the nearest/farthest third and each image half of the inliers
   (-> `ground_not_planar`). The plane-normal-vs-PX4-gravity check is the existing `tilt_max_deg` (3.0, now bounded to
   (0, 5]); it cannot be lowered below the must-accept 2 deg attitude-error cases (bridging planes tilt 2.15-2.80 deg).
5. **ROS node / session.** `camera_ros2_node.load_node_config` refuses an outdoor contract:
   `UNCONFIGURED: outdoor_vertical_fields_not_forwarded: ...` (`ROS_PATH_FORWARDS_VERTICAL_FIELDS = False`); `--check`
   prints `NOT_CONFIGURED`. Session: `REFUSAL_REENTRY_FRAMES` = 5 (PROVISIONAL; 0.5 s = one trained history interval):
   after a refusal the next 5 accepted frames run the policy (history refills) but propose the zero hold
   (`HOLD_REENTRY`, `reentry_hold_remaining` in proposal and diagnostics); `CameraPolicyRuntime.applied_zero_hold()`
   sets the previous action to the applied zero command; a refusal restarts the count; a runtime without the method
   faults the session (fail closed). No hold time limit (user decision, OPEN).
6. **P3.** epv: real scalar only (`int`/`float`/numpy integer/floating, not bool) -> strings, arrays (also 0-d), lists,
   None, complex raise `InputError`. Contract: `ceiling.physical` only with the outdoor profile; `refuse_dead_reckoning`
   must be JSON true and the runtime refuses dead reckoning regardless of the flag; `tilt_max_deg` (0, 5],
   `height_tolerance_m` (0, 0.5], outdoor virtual ceiling [2.5, 6.0] m; `height_reference_problem` exempts only a
   well-formed outdoor profile (height-filter and outdoor problems both checked). Fixed runtime constants carry
   `# PROVISIONAL` and are listed in `PROVISIONAL_RUNTIME_CONSTANTS` (10, tested). The byte-identity test embeds
   golden digests produced from pre_tree in a fresh process (no `~/motar_tmp` path); the README append test embeds the
   pre-change text's length and SHA-256.

## Review findings -> status

| Finding (REVIEW_r1.md) | Status | Evidence (synthetic; numbers from the records named in "Records") |
|---|---|---|
| **P0-1** far-ground band frees dark invalid rails at the flight altitude | **FIXED** (rule off by default) | p1 rerun: dark rail 0.01-0.11 m below the camera, h 1.0/1.5, pitch 0/+5/-5: 0 -> all columns at 1.5 m (936/936 over 6 variants), 2 m (936/936), 3 m (684/684); level path 0 -> 156/156 and 114/114. Test `ReviewerScenesFixed.test_p0_1_invalid_rail_at_the_flight_altitude_is_near` (+ opt-in control: the rule loses it again, 0 columns). Policy (seed 1/2): rail 2 m dv vs training-equivalent 0.15/0.14 m/s (review: 2.79). At 5 m the 0.10 m bar (< 2 trained rows) is still 0/396: blob_near density, the pre-existing thin-structure limit (P2-2 class); r1 lost it at 5 m too. Cost: invalid far ground in an open field is a near band on all 160 bearings (table, repair 2). |
| **P0-2a** light-grey/white invalid objects at/above the flight altitude freed by rule 1 | **PARTIAL** | Fixed when the object reaches below the horizon (rows below the horizon stay invalid now): altitude panel 0 -> 66/66 (white/light grey, all three backgrounds), raised box (bottom camera -0.10 m) 0 -> 138/138 at pitch 0/+5/-5, bars centred on camera height at 2 m 312 -> 936/936. Fixed by the edge veto when the object has valid vertical edges: twins (table, repair 3) 2676/2696 object columns kept, 29/30 twins fully; tests `ObjectsWithValidEdges.*`. **OPEN**: sensor-invalid sky-like objects with NO valid edge lying wholly above the horizon that connect to the image top (light-grey panel camera +0.05..+0.55 m: still 0/66 at every pitch; p2 b panels reaching above the image top: 0/66). Expected-failure tests `FullyInvalidLimitations.*`. |
| **P0-2b** ceiling-removed pixels bridge the sky flood | **FIXED** | Connector removed. White panel beside a 5 m tree line 0 -> 66/66 (p2c, p2d); test `SkyRule.test_virtual_ceiling_never_connects_the_sky`. Policy: dv 0.09 m/s (review: 2.51). Consequence measured: sky enclosed below a valid 3.2 m canopy is now a near band on 160/160 bearings (r1: 0); policy dv 2.86/2.31 m/s (seed 1/2) against the training-equivalent view, the phantom direction. |
| **P0-3** bright invalid overhead canopy below 3 m lost | **PARTIAL** | Realistic twin (3 x 3 m gazebo canopy, invalid bright underside, valid skirts/posts) at 2.0/2.5 m: 152/152 columns kept, white/light grey/dark; test `ObjectsWithValidEdges.test_gazebo_canopy_twin_below_the_virtual_ceiling`; policy dv 0.12/0.10. **OPEN**: the reviewer's 10 m-wide fully invalid canopy (no edge in view): still 0/160 < 4 m (p7); expected-failure test `test_bright_invalid_canopy_below_the_virtual_ceiling_is_detected`; policy dv 2.70/2.16. |
| **P1-1** wrong-surface ground fits accepted | **PARTIAL** (bridging FIXED; raised platform hiding all ground OPEN as a named residual) | Ground sweep: must-accept (30 poses incl. flat, +-0.2 m drift, +-2 deg attitude, 2 deg slope, open field; 50 cases) 50/50 accepted, max abs height error 0.0063 m; obstacle scenes (trunks, post, forest edge, bars, walls alongside at 1 and 2 m, bench, car-like box, twins; 26) 26/26 accepted; low wide box with ground visible 0/14 accepted (r1: 10/14, worst +0.110 m / 2.80 deg). p3a: the reviewer's bridging case now refused (`ground_not_planar`, `ground_ambiguous`); its bar cases refused (zero hold) instead of 52/60 and 42/48 columns. Tests `GroundWrongSurface.*`, `GroundEstimate.test_low_wide_box`. Cost: 3 frames where r1 already had the true ground (error <= 0.005 m) beside a large raised top are now refused (`ground_ambiguous`). Residual: a 0.2 m surface hiding all ground within 10 m is still accepted at -0.20 m (both heights); it cannot be refused without refusing the must-accept +-0.2 m PX4 drift (tolerance 0.3 m): expected-failure test `test_raised_platform_hiding_all_ground_is_refused`. |
| **P2-1** outdoor profile cannot run on the ROS node; holds unlimited, no hysteresis | **PARTIAL** | Node: refuses an outdoor contract at load, `UNCONFIGURED: outdoor_vertical_fields_not_forwarded: ...`, `--check` -> `NOT_CONFIGURED` exit 2; indoor control loads (`CONFIG_VALID`). Session: hysteresis `REFUSAL_REENTRY_FRAMES` = 5 (PROVISIONAL), tests `SessionHoldOnRefusal.test_refused_frames_propose_an_explicit_zero_hold_then_restart_history`, `test_a_refusal_during_reentry_restarts_the_count`, `test_reentry_hold_needs_the_runtime_to_record_the_applied_hold`, `test_production_runtime_records_the_applied_zero_hold`. **OPEN**: forwarding z_valid/epv/dead_reckoning (gateway + adapter, out of scope); hold time limit / escalation (user decision). |
| **P2-2** invalid thin wires never detected (pre-existing) | **OPEN** (kept visible) | Expected-failure tests `MeasuredLimitations.test_invalid_thin_wires_are_detected` and `test_valid_thin_wires_on_the_level_path_are_detected`. |
| **P3-1** vacuous assertion; miss pinned as passing | **FIXED** | `assertGreaterEqual(detected_valid, 0)` replaced: valid wires on the tilted path ranged at their distance on > 50 bearings each (measured 129/132/74/84) and never closer. Every test that asserted a miss now asserts the safe outcome under `@unittest.expectedFailure` (7 in the module). |
| **P3-2** epv as string/array accepted | **FIXED** | p6 rerun: `"0.1"` and `[0.1]` -> `InputError`; tests also refuse 0-d arrays, lists, None, complex, bool; numpy float/int scalars accepted (controls). |
| **P3-3** contract schema permissive | **FIXED** (`sky.enabled: false` still allowed, see note) | `ceiling.physical` refused on indoor contracts (pre-change behaviour restored); `height_reference_problem` exempts only a well-formed outdoor profile; `refuse_dead_reckoning` must be true (and the runtime refuses dead reckoning regardless); `tilt_max_deg` (0, 5], `height_tolerance_m` (0, 0.5], outdoor virtual ceiling [2.5, 6.0] m. p6: the four "VALID" contract cases are now refused with named problems. `sky.enabled: false` was left valid: it frees nothing (every invalid pixel stays near), the safe direction. |
| **P3-4** fixed runtime constants not PROVISIONAL | **FIXED** | 10 constants marked and listed in `PROVISIONAL_RUNTIME_CONSTANTS` (incl. `GROUND_SEARCH_TILT_DEG`, the new `GROUND_REMOVE_FACTOR` for the former literal `2*threshold`, `EDGE_VETO_DILATION_PX`); test `test_fixed_runtime_constants_are_marked_provisional`. |
| **P3-5** SKY_CONNECTIVITY / comment contradict the runtime | **FIXED** | The runtime now matches the constant (`top_border_4_through_sky_or_unimaged`); contract comment rewritten; unimaged restricted to pixels outside the source FOV. |
| **P3-6** byte-identity test depends on `~/motar_tmp` and mixes modules | **FIXED** | `indoor_golden_digests()` (14 scenes x 6 indoor contracts: preprocess, projection, height_band, extra-field state; 8 contract-layer variants) run in a fresh process with the pre_tree modules (`scratch/gen_goldens.py pre`) and with the current ones (`new`): identical; the pre digests are embedded (`INDOOR_GOLDENS`). The README-append test embeds the pre-change text's length and SHA-256. No test of the module reads `~/motar_tmp` except the checkpoint directory used (as before) by the actual-policy class. |
| Criterion 1 (ROS node path) | **PARTIAL** | Explicit fail-closed refusal at load instead of an endless hold; forwarding remains a separate unit. |

**Unimaged connector (repair 1 condition).** Kept. Test `SkyRule.test_unimaged_connector_frees_no_near_object_with_a_valid_edge`:
twins with valid edges at nose-down 5/10/15 deg that touch the unimaged band keep every sensor-invalid face pixel with
and without the connector (white, light grey, dark), and all object columns stay near. It cannot be shown for objects
with NO valid edge: the white fully invalid panel kept at pitch 0 and +5 (its luma edge against the sky encloses it)
is freed at +10/+15 when it touches the band (p2: 0/66), exactly like a fully invalid object touching the image top at
pitch 0 is freed (tall wall to 4 m: 0/66 at pitch 0; `scratch/explore2.py`, console output only). That is the FullyInvalidLimitations class (expected-failure
test `test_white_panel_touching_the_unimaged_band_is_detected`). Without the connector the open sky becomes a near
wall on 160/160 bearings at +5 and +10 deg nose-down (tree line and open field; at +15 the tree-line sky lies wholly
in the band). Removing it is a one-line change if the lead prefers a hold over nose-down flight; I did not, because
that trade is not mine to make silently. The restriction to pixels OUTSIDE the source FOV is new: disocclusion and
sampling gaps inside the FOV no longer connect.

## Repair 1 and 2 tables (`repair12.json`, `scratch/measure_r12.py`)

Near bearings (of 160), far-ground rule OFF (default) / ON (opt-in) / indoor default before the outdoor profile.
Same result at h 1.0 and 1.5 m and pitch -10, -5, 0, +5, +10 deg (10 frames per row):

| Scene | far ground beyond 20 m | rule OFF (default) | rule ON (opt-in) | indoor before |
|---|---|---|---|---|
| open field | ranged by the OAK-D (z > 20 m kept, no-return) | 0 | 0 | 160 |
| open field | reported as depth 0 | **160** (phantom band) | 0 | 160 |
| tree line 15 m | ranged | 0 | 0 | 160 |
| tree line 15 m | depth 0 | 0 (the band is behind the tree line) | 0 | 160 |

Whether the OAK-D reports far ground as depth 0 at the user's confidence settings is NOT_MEASURED; with the rule off
the open-field case is a phantom wall, and the policy command then differs from the training-equivalent one by
2.87/2.32 m/s (seed 1/2; open field h1.5 pitch +5: 2.63/2.18).

Virtual ceiling (h 1.0, canopy x 1-6 m, y +-5 m): valid 3.2 m canopy: 160 near bearings (r1: 0), its return removed
(4142 px), 1280 sky candidates not freed; dark invalid 3.2 m canopy: 160 (unchanged); valid 2.0 m canopy: 0 near,
ranged at 2.14 m (unchanged). Unimaged connector, near bearings with / without it: tree line pitch 0: 0/0, +5: 0/160,
+10: 0/160, +15: 0/0; open field (far ground ranged): 0/0, 0/160, 0/160, 0/160.

## Repair 3 trade-off (`veto_sweep.json`, `veto_dilation.json`, `scratch/sweep_veto.py`, `scratch/sweep_dilation.py`)

30 twins (bright faces with valid vertical edges: panels at 1.5/3/4.5 m and pitch 0/10, altitude panels, raised
boxes, gazebo canopies, rails with posts and ends; white and light grey) and 43 phantom scenes (trunk pairs at 2-6 m
with 1.5-5 m spacing in front of a tree line or an open horizon, unequal pairs, a forest edge, bars, tree line, single
trunks). Twin columns = object columns kept inside 10 m; phantom bearings added = near bearings with the veto minus
without. Chosen default: **5.0 m, 70 deg, 1.0 m** (largest protection; misses preferred to be phantoms). Span 45 deg
protects the same 30 twins with fewer phantoms (722) but loses a 2.4 m face wholly above the horizon at 2 m (64.6 deg:
138/138 kept at 70 deg, 0/138 at 45 deg; `scratch/explore3.py`). Tolerance matters only for phantoms. Dilation 0/1/2:
phantom bearings 1242 in all three; dilation 2 is needed for the face rows next to the unimaged band at 15 deg
nose-down (1 and 2 rows unresolved at 10 and 15 deg).

### Veto sweep (R_edge x span x tolerance; dilation 2)
| R_edge m | span deg | tol m | twin cols kept (of total) | twins fully kept | twin cols kept without veto | phantom bearings added | phantom scenes with added |
|---|---|---|---|---|---|---|---|
| 3 | 30 | 0.5 | 2378 / 2696 | 25 / 30 | 2266 | 50 | 1 / 43 |
| 3 | 30 | 1 | 2378 / 2696 | 25 / 30 | 2266 | 50 | 1 / 43 |
| 3 | 30 | 2 | 2378 / 2696 | 25 / 30 | 2266 | 50 | 1 / 43 |
| 3 | 45 | 0.5 | 2576 / 2696 | 28 / 30 | 2266 | 202 | 3 / 43 |
| 3 | 45 | 1 | 2576 / 2696 | 28 / 30 | 2266 | 202 | 3 / 43 |
| 3 | 45 | 2 | 2576 / 2696 | 28 / 30 | 2266 | 202 | 3 / 43 |
| 3 | 70 | 0.5 | 2576 / 2696 | 28 / 30 | 2266 | 202 | 3 / 43 |
| 3 | 70 | 1 | 2576 / 2696 | 28 / 30 | 2266 | 202 | 3 / 43 |
| 3 | 70 | 2 | 2576 / 2696 | 28 / 30 | 2266 | 202 | 3 / 43 |
| 5 | 30 | 0.5 | 2478 / 2696 | 26 / 30 | 2266 | 402 | 7 / 43 |
| 5 | 30 | 1 | 2478 / 2696 | 26 / 30 | 2266 | 402 | 7 / 43 |
| 5 | 30 | 2 | 2478 / 2696 | 26 / 30 | 2266 | 402 | 7 / 43 |
| 5 | 45 | 0.5 | 2676 / 2696 | 29 / 30 | 2266 | 722 | 11 / 43 |
| 5 | 45 | 1 | 2676 / 2696 | 29 / 30 | 2266 | 722 | 11 / 43 |
| 5 | 45 | 2 | 2676 / 2696 | 29 / 30 | 2266 | 813 | 12 / 43 |
| 5 | 70 | 0.5 | 2676 / 2696 | 29 / 30 | 2266 | 1242 | 15 / 43 |
| 5 | 70 | 1 | 2676 / 2696 | 29 / 30 | 2266 | 1242 | 15 / 43 |
| 5 | 70 | 2 | 2676 / 2696 | 29 / 30 | 2266 | 1333 | 16 / 43 |
| 8 | 30 | 0.5 | 2478 / 2696 | 26 / 30 | 2266 | 732 | 14 / 43 |
| 8 | 30 | 1 | 2478 / 2696 | 26 / 30 | 2266 | 732 | 14 / 43 |
| 8 | 30 | 2 | 2478 / 2696 | 26 / 30 | 2266 | 766 | 14 / 43 |
| 8 | 45 | 0.5 | 2676 / 2696 | 29 / 30 | 2266 | 1248 | 20 / 43 |
| 8 | 45 | 1 | 2676 / 2696 | 29 / 30 | 2266 | 1248 | 20 / 43 |
| 8 | 45 | 2 | 2676 / 2696 | 29 / 30 | 2266 | 1464 | 22 / 43 |
| 8 | 70 | 0.5 | 2676 / 2696 | 29 / 30 | 2266 | 2004 | 26 / 43 |
| 8 | 70 | 1 | 2676 / 2696 | 29 / 30 | 2266 | 2004 | 26 / 43 |
| 8 | 70 | 2 | 2676 / 2696 | 29 / 30 | 2266 | 2220 | 28 / 43 |

### Default (5.0 m, 70 deg, 1.0 m): per twin
| twin | kept / cols | kept without veto |
|---|---|---|
| panel_above white D1.5 p0 | 66 / 66 | 66 |
| panel_above white D1.5 p10 | 66 / 66 | 0 |
| panel_above white D3.0 p0 | 30 / 30 | 30 |
| panel_above white D3.0 p10 | 30 / 30 | 30 |
| panel_above white D4.5 p0 | 18 / 18 | 18 |
| panel_above white D4.5 p10 | 18 / 18 | 18 |
| panel_alt white tree5m | 66 / 66 | 66 |
| panel_alt white none | 66 / 66 | 66 |
| raised_box white p0 | 138 / 138 | 138 |
| raised_box white p5 | 138 / 138 | 138 |
| raised_box white p-5 | 138 / 138 | 138 |
| canopy gazebo white z2.0 | 152 / 152 | 152 |
| canopy gazebo white z2.5 | 152 / 152 | 152 |
| rail posts+ends white D2 open | 156 / 156 | 156 |
| rail posts+ends white D3 open | 114 / 114 | 114 |
| panel_above lgrey D1.5 p0 | 66 / 66 | 0 |
| panel_above lgrey D1.5 p10 | 66 / 66 | 0 |
| panel_above lgrey D3.0 p0 | 30 / 30 | 30 |
| panel_above lgrey D3.0 p10 | 30 / 30 | 30 |
| panel_above lgrey D4.5 p0 | 18 / 18 | 18 |
| panel_above lgrey D4.5 p10 | 18 / 18 | 18 |
| panel_alt lgrey tree5m | 66 / 66 | 66 |
| panel_alt lgrey none | 66 / 66 | 66 |
| raised_box lgrey p0 | 138 / 138 | 138 |
| raised_box lgrey p5 | 138 / 138 | 138 |
| raised_box lgrey p-5 | 138 / 138 | 138 |
| canopy gazebo lgrey z2.0 | 152 / 152 | 152 |
| canopy gazebo lgrey z2.5 | 152 / 152 | 152 |
| rail posts+ends lgrey D2 open | 136 / 156 | 24 |
| rail posts+ends lgrey D3 open | 114 / 114 | 14 |

### Default: phantom scenes (near bearings with / without veto)
| scene | near with veto | near without | added |
|---|---|---|---|
| trunk pair D2 s1.5 tree | 76 | 0 | 76 |
| trunk pair D2 s3.0 tree | 0 | 0 | 0 |
| trunk pair D2 s5.0 tree | 0 | 0 | 0 |
| trunk pair D3 s1.5 tree | 50 | 0 | 50 |
| trunk pair D3 s3.0 tree | 114 | 0 | 114 |
| trunk pair D3 s5.0 tree | 0 | 0 | 0 |
| trunk pair D4 s1.5 tree | 38 | 0 | 38 |
| trunk pair D4 s3.0 tree | 84 | 0 | 84 |
| trunk pair D4 s5.0 tree | 146 | 0 | 146 |
| trunk pair D5 s1.5 tree | 0 | 0 | 0 |
| trunk pair D5 s3.0 tree | 0 | 0 | 0 |
| trunk pair D5 s5.0 tree | 0 | 0 | 0 |
| trunk pair D6 s1.5 tree | 0 | 0 | 0 |
| trunk pair D6 s3.0 tree | 0 | 0 | 0 |
| trunk pair D6 s5.0 tree | 0 | 0 | 0 |
| trunk pair 2/3 s3 tree | 0 | 0 | 0 |
| trunk pair 2/4 s3 tree | 0 | 0 | 0 |
| trunk pair 3/5 s3 tree | 0 | 0 | 0 |
| forest edge tree | 98 | 0 | 98 |
| trunk pair D2 s1.5 open | 76 | 0 | 76 |
| trunk pair D2 s3.0 open | 0 | 0 | 0 |
| trunk pair D2 s5.0 open | 0 | 0 | 0 |
| trunk pair D3 s1.5 open | 50 | 0 | 50 |
| trunk pair D3 s3.0 open | 114 | 0 | 114 |
| trunk pair D3 s5.0 open | 0 | 0 | 0 |
| trunk pair D4 s1.5 open | 38 | 0 | 38 |
| trunk pair D4 s3.0 open | 84 | 0 | 84 |
| trunk pair D4 s5.0 open | 146 | 0 | 146 |
| trunk pair D5 s1.5 open | 30 | 0 | 30 |
| trunk pair D5 s3.0 open | 0 | 0 | 0 |
| trunk pair D5 s5.0 open | 0 | 0 | 0 |
| trunk pair D6 s1.5 open | 0 | 0 | 0 |
| trunk pair D6 s3.0 open | 0 | 0 | 0 |
| trunk pair D6 s5.0 open | 0 | 0 | 0 |
| trunk pair 2/3 s3 open | 0 | 0 | 0 |
| trunk pair 2/4 s3 open | 0 | 0 | 0 |
| trunk pair 3/5 s3 open | 0 | 0 | 0 |
| forest edge open | 98 | 0 | 98 |
| bars golden-like | 0 | 0 | 0 |
| tree line | 0 | 0 | 0 |
| tree line p10 | 0 | 0 | 0 |
| trunk 0.3 m | 90 | 90 | 0 |
| trunk 1.5 m | 0 | 0 | 0 |


The one twin not fully kept at the default is the light-grey rail at 2 m (136/156): its segments beyond the outer
posts run out of the field of view and the image border is not an edge. Rail columns right beside a valid post can
lose blob_near density (pre-existing; one column of 114 in the 3 m test, allowed by the test).

## Repair 4 tables (`ground_sweep.json`, `ground_sweep_summary.json`, `scratch/sweep_ground.py`)

Variants change one threshold; "r1 checks only" disables the three new checks. Margins on the accepted must-accept
and obstacle cases: max planarity angle 0.47 deg (threshold 1.0), min half share 0.32 (0.10), min range span 3.55 m
(1.0). The thresholds are insensitive in this synthetic set (planarity 0.5-2.0 deg and rival 0.05-0.20 give the same
accept/refuse); real OAK-D noise and vegetation are NOT_MEASURED.

### Ground sweep: accepted / cases (max |height error| of accepted, m)
| variant | must_accept | obstacle | box_ground_visible | box_hides_ground |
|---|---|---|---|---|
| repair default | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |
| r1 checks only | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 10 / 14 (0.1095) | 2 / 6 (0.2021) |
| planarity 0.5 | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |
| planarity 1.5 | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |
| planarity 2.0 | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |
| rival 0.05 | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |
| rival 0.20 | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |
| no rival | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 3 / 14 (0.005) | 2 / 6 (0.2021) |
| no planarity | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 1 / 14 (0.0689) | 2 / 6 (0.2021) |
| half 0.2 | 50 / 50 (0.0063) | 26 / 26 (0.0014) | 0 / 14 (None) | 2 / 6 (0.2021) |

### Box cases: r1 checks only vs repair default
| case | r1 | repair |
|---|---|---|
| box x3-8 top 0.2 h1 p0 | ACCEPT err +0.110 m, tilt 2.80 deg | REFUSE ground_not_planar, ground_ambiguous |
| box x3-8 top 0.2 h1 p10 | ACCEPT err +0.069 m, tilt 2.15 deg | REFUSE ground_not_planar |
| box x3-8 top 0.2 h1.5 p0 | REFUSE ground_not_lowest_surface | REFUSE ground_not_lowest_surface |
| box x3-8 top 0.2 h1.5 p10 | ACCEPT err +0.004 m, tilt 0.14 deg | REFUSE ground_ambiguous |
| box x3-8 top 0.3 h1 p0 | REFUSE ground_tilt_exceeds_tolerance | REFUSE ground_tilt_exceeds_tolerance |
| box x3-8 top 0.3 h1 p10 | ACCEPT err +0.012 m, tilt 0.42 deg | REFUSE ground_not_planar, ground_ambiguous |
| box x3-8 top 0.3 h1.5 p0 | REFUSE ground_not_lowest_surface | REFUSE ground_not_lowest_surface |
| box x3-8 top 0.3 h1.5 p10 | ACCEPT err +0.031 m, tilt 0.81 deg | REFUSE ground_inlier_spread_insufficient, ground_not_planar, ground_ambiguous |
| box x3-8 top 0.6 h1 p0 | ACCEPT err +0.054 m, tilt 1.39 deg | REFUSE ground_inlier_spread_insufficient, ground_not_planar, ground_ambiguous |
| box x3-8 top 0.6 h1 p10 | ACCEPT err +0.013 m, tilt 0.44 deg | REFUSE ground_not_planar, ground_ambiguous |
| box x3-8 top 0.6 h1.5 p0 | REFUSE ground_not_lowest_surface | REFUSE ground_not_lowest_surface |
| box x3-8 top 0.6 h1.5 p10 | ACCEPT err +0.037 m, tilt 0.97 deg | REFUSE ground_inlier_spread_insufficient, ground_not_planar, ground_ambiguous |
| box x4-9 top 0.3 h1 p0 | ACCEPT err +0.005 m, tilt 0.14 deg | REFUSE ground_ambiguous |
| box x2-6 top 0.3 h1.5 p0 | ACCEPT err -0.004 m, tilt 0.05 deg | REFUSE ground_ambiguous |
| box x1.5-8 top 0.2 h1 | ACCEPT err -0.201 m, tilt 0.01 deg | ACCEPT err -0.201 m |
| box x1.5-8 top 0.3 h1 | REFUSE ground_height_disagrees_with_px4 | REFUSE ground_height_disagrees_with_px4 |
| box x1.5-8 top 0.6 h1 | REFUSE ground_not_lowest_surface | REFUSE ground_not_lowest_surface |
| box x1.5-8 top 0.2 h1.5 | ACCEPT err -0.202 m, tilt 0.03 deg | ACCEPT err -0.202 m |
| box x1.5-8 top 0.3 h1.5 | REFUSE ground_height_disagrees_with_px4 | REFUSE ground_height_disagrees_with_px4 |
| box x1.5-8 top 0.6 h1.5 | REFUSE ground_height_disagrees_with_px4 | REFUSE ground_height_disagrees_with_px4 |

## Reviewer probes rerun (copies; reviewer folder untouched)

The reviewer's probe scripts were copied (`reviewer_probes_copy/`, SHA-256 equal to the originals) and run unchanged
except for an output redirect (`scratch/probe_common.py`); p2c and p3a were adapted to the new `apply_sky_rule`
signature (`scratch/*_adapted.py`, geometry unchanged). Outputs: `reviewer_rerun/`; kernel faults 0 -> 0.

| P1 scene (all h x pitch variants summed) | D m | variants | before: rail cols < 10 m | after | of |
|---|---|---|---|---|---|
| CONTROL_dark_rail_well_below_band | 1.5 | 6 | 936 | 936 | 936 |
| CONTROL_dark_rail_well_below_band | 2.0 | 6 | 936 | 936 | 936 |
| CONTROL_dark_rail_well_below_band | 3.0 | 6 | 684 | 684 | 684 |
| CONTROL_dark_rail_well_below_band | 5.0 | 6 | 198 | 396 | 396 |
| dark_bar_centred_10cm | 1.5 | 6 | 936 | 936 | 936 |
| dark_bar_centred_10cm | 2.0 | 6 | 312 | 936 | 936 |
| dark_bar_centred_10cm | 3.0 | 6 | 0 | 684 | 684 |
| dark_bar_centred_10cm | 5.0 | 6 | 0 | 0 | 396 |
| dark_rail_below_cam_10cm | 1.5 | 6 | 0 | 936 | 936 |
| dark_rail_below_cam_10cm | 2.0 | 6 | 0 | 936 | 936 |
| dark_rail_below_cam_10cm | 3.0 | 6 | 0 | 684 | 684 |
| dark_rail_below_cam_10cm | 5.0 | 6 | 0 | 0 | 396 |
| dark_rail_below_cam_10cm_level_path | 2.0 | 1 | 0 | 156 | 156 |
| dark_rail_below_cam_10cm_level_path | 3.0 | 1 | 0 | 114 | 114 |
| grey_bar_centred_10cm | 2.0 | 6 | 312 | 936 | 936 |
| white_bar_centred_10cm | 2.0 | 6 | 312 | 936 | 936 |

| P2/P2c/P2d/P7 scene | pitch | before (obj cols < 10 m) | after | of |
|---|---|---|---|---|
| p2 a_panel_above_horizon_white_top+0.55 | +0 | 0 | 66 | 66 |
| p2 a_panel_above_horizon_white_top+0.55 | +5 | 0 | 66 | 66 |
| p2 a_panel_above_horizon_white_top+0.55 | +10 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_white_top+0.55 | +15 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_white_top+0.45 | +0 | 0 | 66 | 66 |
| p2 a_panel_above_horizon_white_top+0.45 | +5 | 0 | 66 | 66 |
| p2 a_panel_above_horizon_white_top+0.45 | +10 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_white_top+0.45 | +15 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.55 | +0 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.55 | +5 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.55 | +10 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.55 | +15 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.45 | +0 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.45 | +5 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.45 | +10 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_lightgrey_top+0.45 | +15 | 0 | 0 | 66 |
| p2 a_panel_above_horizon_dark_top+0.55 | +0 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.55 | +5 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.55 | +10 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.55 | +15 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.45 | +0 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.45 | +5 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.45 | +10 | 66 | 66 | 66 |
| p2 a_panel_above_horizon_dark_top+0.45 | +15 | 66 | 66 | 66 |
| p2 a2_panel_cam-0.10_to_+0.45_white | +0 | 0 | 66 | 66 |
| p2 a2_panel_cam-0.10_to_+0.45_white | +10 | 0 | 66 | 66 |
| p2 a2_panel_cam-0.10_to_+0.45_white | +15 | 0 | 66 | 66 |
| p2 a2_panel_cam-0.10_to_+0.45_lightgrey | +0 | 0 | 66 | 66 |
| p2 a2_panel_cam-0.10_to_+0.45_lightgrey | +10 | 0 | 66 | 66 |
| p2 a2_panel_cam-0.10_to_+0.45_lightgrey | +15 | 0 | 66 | 66 |
| p2 b_panel_with_beam_white_bottom+0.05 | +0 | 0 | 0 | 66 |
| p2 b_CONTROL_panel_no_beam_white_bottom+0.05 | +0 | 0 | 0 | 66 |
| p2 b_panel_with_beam_white_bottom-0.10 | +0 | 0 | 66 | 66 |
| p2 b_CONTROL_panel_no_beam_white_bottom-0.10 | +0 | 0 | 66 | 66 |
| p2 b_panel_with_beam_lightgrey_bottom+0.05 | +0 | 0 | 0 | 66 |
| p2 b_CONTROL_panel_no_beam_lightgrey_bottom+0.05 | +0 | 0 | 0 | 66 |
| p2 b_panel_with_beam_lightgrey_bottom-0.10 | +0 | 0 | 66 | 66 |
| p2 b_CONTROL_panel_no_beam_lightgrey_bottom-0.10 | +0 | 0 | 66 | 66 |
| p2 b_panel_with_beam_dark_bottom+0.05 | +0 | 66 | 66 | 66 |
| p2 b_CONTROL_panel_no_beam_dark_bottom+0.05 | +0 | 66 | 66 | 66 |
| p2 b_panel_with_beam_dark_bottom-0.10 | +0 | 66 | 66 | 66 |
| p2 b_CONTROL_panel_no_beam_dark_bottom-0.10 | +0 | 66 | 66 | 66 |
| p2c panel_white_treeline5m | +0 | 0 | 66 | 66 |
| p2c panel_white_treeline2.5m_CONTROL | +0 | 66 | 66 | 66 |
| p2c panel_white_no_treeline_CONTROL | +0 | 66 | 66 | 66 |
| p2c panel_lightgrey_treeline5m | +0 | 0 | 0 | 66 |
| p2c panel_lightgrey_treeline2.5m_CONTROL | +0 | 0 | 0 | 66 |
| p2c panel_lightgrey_no_treeline_CONTROL | +0 | 0 | 0 | 66 |
| p2c panel_dark_treeline5m | +0 | 66 | 66 | 66 |
| p2c panel_dark_treeline2.5m_CONTROL | +0 | 66 | 66 | 66 |
| p2c panel_dark_no_treeline_CONTROL | +0 | 66 | 66 | 66 |
| p2c raised_box_white_bottom_cam-0.10 | +0 | 0 | 138 | 138 |
| p2c raised_box_white_bottom_cam-0.10 | +5 | 0 | 138 | 138 |
| p2c raised_box_white_bottom_cam-0.10 | -5 | 0 | 138 | 138 |
| p2c raised_box_lightgrey_bottom_cam-0.10 | +0 | 0 | 138 | 138 |
| p2c raised_box_lightgrey_bottom_cam-0.10 | +5 | 0 | 138 | 138 |
| p2c raised_box_lightgrey_bottom_cam-0.10 | -5 | 0 | 138 | 138 |
| p2c raised_box_dark_bottom_cam-0.10 | +0 | 138 | 138 | 138 |
| p2c raised_box_dark_bottom_cam-0.10 | +5 | 138 | 138 | 138 |
| p2c raised_box_dark_bottom_cam-0.10 | -5 | 138 | 138 | 138 |
| p2d white panel, tree5m | 0 | 0 | 66 | 66 |
| p2d white panel, tree2.5m | 0 | 66 | 66 | 66 |
| p2d white panel, none | 0 | 66 | 66 | 66 |
| p2d lightgrey panel, tree5m | 0 | 0 | 66 | 66 |
| p2d lightgrey panel, tree2.5m | 0 | 0 | 66 | 66 |
| p2d lightgrey panel, none | 0 | 0 | 66 | 66 |
| p2d dark panel, tree5m | 0 | 66 | 66 | 66 |
| p2d dark panel, tree2.5m | 0 | 66 | 66 | 66 |
| p2d dark panel, none | 0 | 66 | 66 | 66 |
| p7 canopy z 2.0 white_invalid (cols < 4 m) | 0 | 0 | 0 | 160 |
| p7 canopy z 2.0 lightgrey_invalid (cols < 4 m) | 0 | 0 | 0 | 160 |
| p7 canopy z 2.0 dark_invalid (cols < 4 m) | 0 | 160 | 160 | 160 |
| p7 canopy z 2.0 textured_valid (cols < 4 m) | 0 | 160 | 160 | 160 |
| p7 canopy z 2.5 white_invalid (cols < 4 m) | 0 | 0 | 0 | 160 |
| p7 canopy z 2.5 lightgrey_invalid (cols < 4 m) | 0 | 0 | 0 | 160 |
| p7 canopy z 2.5 dark_invalid (cols < 4 m) | 0 | 160 | 160 | 160 |
| p7 canopy z 2.5 textured_valid (cols < 4 m) | 0 | 160 | 160 | 160 |

| P3a (bridging plane) | before | after |
|---|---|---|
| report case h 1.0, box x3-8 top 0.2 | accepted +0.1095 m / 2.7957 deg | accepted=False |
| with_box(wrong plane) D 2.0 bar [0.02, 0.12] | 52/60 cols | refused `ground_not_planar` |
| CONTROL_no_box(correct plane) D 2.0 bar [0.02, 0.12] | 60/60 cols | 60/60 cols |
| with_box(wrong plane) D 2.0 bar [0.0, 0.08] | 52/60 cols | refused `ground_not_planar` |
| CONTROL_no_box(correct plane) D 2.0 bar [0.0, 0.08] | 60/60 cols | 60/60 cols |
| with_box(wrong plane) D 2.5 bar [0.02, 0.12] | 42/48 cols | refused `ground_not_planar` |
| CONTROL_no_box(correct plane) D 2.5 bar [0.02, 0.12] | 48/48 cols | 48/48 cols |
| with_box(wrong plane) D 2.5 bar [0.0, 0.08] | 42/48 cols | refused `ground_not_planar` |
| CONTROL_no_box(correct plane) D 2.5 bar [0.0, 0.08] | 48/48 cols | 48/48 cols |

| P6 fail-closed input | before | after |
|---|---|---|
| control | ACCEPTED | ACCEPTED |
| epv as string "0.1" | ACCEPTED | EXC InputError: estimator epv must be a finite nonnegative real scalar in metres |
| epv as 1-element array | ACCEPTED | EXC InputError: estimator epv must be a finite nonnegative real scalar in metres |
| z_valid numpy bool | ACCEPTED | ACCEPTED |
| dead_reckoning int 0 | EXC InputError: estimator z_valid and dead_reckoning must be booleans | EXC InputError: estimator z_valid and dead_reckoning must be booleans |
| epv inf | EXC InputError: estimator epv must be a finite nonnegative number of m | EXC InputError: estimator epv must be a finite nonnegative number of metres |
| epv missing | REFUSED estimator_vertical_fields_absent | REFUSED estimator_vertical_fields_absent |
| pos z NaN | EXC InputError: pos must be finite with shape (3,) | EXC InputError: pos must be finite with shape (3,) |
| dead reckoning with refuse_dead_reckoning=false | ACCEPTED | ACCEPTED |
| validate: refuse_dead_reckoning=false | VALID | ['ground_estimate.refuse_dead_reckoning must be JSON true'] |
| validate: sky.enabled=false | VALID | VALID |
| validate: height_tolerance 2.0 m | VALID | ['ground_estimate.height_tolerance_m.value must be finite in (0, 0.5]'] |
| validate: tilt_max 30 deg | VALID | ['ground_estimate.tilt_max_deg.value must be finite in (0, 5]'] |
| validate: virtual ceiling 1.2 m | VALID | ['the outdoor virtual ceiling height_m must lie in [2.5, 6] m (PROVISIONAL)'] |
| all-invalid frame | REFUSED ground_insufficient_points | REFUSED ground_insufficient_points |
| all-NaN depth, invalid mask False | REFUSED ground_insufficient_points | REFUSED ground_insufficient_points |
| pitch attitude error 2.9 deg | ACCEPTED | ACCEPTED |
| pitch attitude error 3.2 deg | REFUSED ground_tilt_exceeds_tolerance | REFUSED ground_tilt_exceeds_tolerance |
| preprocess fields absent | FrameRefused estimator_vertical_fields_absent | FrameRefused estimator_vertical_fields_absent |

## Observation and actual-policy comparison (seeds 1 and 2)

`analysis/obs_policy_compare_repair_r1.py` = the r1 script (same method: 30 frames at 10 Hz, CAM-K2 v3 checkpoints
read only through `CameraPolicyRuntime`, CPU, one fresh process per seed) with the repaired outdoor default for (iii)
and 15 added scenes (review failing scenes, twins, a phantom). dv = |v_iii - v_i| (m/s) against the
training-equivalent observation (i); r1 (iii) from `../analysis/obs_policy_compare_s*.json`. Kernel faults 0 -> 0.
Records: `analysis/obs_policy_compare_s1.json`, `_s2.json`, `policy_table.md`, `policy_summary.json`.

| Scene | Seed | (i) training-equiv. v_FLU m/s; yaw rad/s; near | (ii) indoor default: near; dv | r1 (iii): near; dv | repair (iii): near; dv m/s / ddir deg / dyaw rad/s |
|---|---|---|---|---|---|
| tree line 15 m | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| tree line 15 m pitch +5 | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 0; 0.02 | 0; 0.02 / 0 / 0.01 |
| tree line 15 m pitch -5 | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 0; 0.01 | 0; 0.01 / 0 / 0.00 |
| open field, far ground valid | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| open field, far ground invalid | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 160; 2.87 / 73 / 2.09 |
| open field h1.5 pitch +5 | 1 | (2.42, -0.98); -1.51; 0 | 160; 2.63 | 0; 0.00 | 160; 2.63 / 68 / 1.69 |
| bars in open field (golden-like) | 1 | (-1.53, 1.29); 1.69; 0 | 151; 0.12 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| bars in open field pitch +3 | 1 | (-1.53, 1.29); 1.69; 0 | 152; 0.12 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| bars in open field h1.5 | 1 | (-1.57, 1.24); 1.64; 0 | 151; 0.18 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| trunk 0.3 m | 1 | (-1.47, 1.35); 1.67; 88 | 160; 0.01 | 90; 0.00 | 90; 0.00 / 0 / 0.00 |
| trunk 1.5 m | 1 | (1.98, -0.27); -1.13; 0 | 138; 1.88 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| branch above camera 0.3 m | 1 | (1.37, 1.46); 0.56; 160 | 160; 0.00 | 160; 0.00 | 160; 0.00 / 0 / 0.00 |
| branch above camera 1.0 m | 1 | (1.41, 1.41); 0.44; 0 | 160; 0.06 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| white object above horizon 0.3 m | 1 | (1.37, 1.46); 0.56; 160 | 160; 0.00 | 0; 2.87 | 0; 2.87 / 73 / 2.09 |
| wire 2 m 3 px (invalid) | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m valid | 1 | (1.50, 1.32); 0.19; 0 | 0; 0.00 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m invalid dark | 1 | (1.50, 1.32); 0.19; 0 | 160; 0.18 | 160; 0.18 | 160; 0.18 / 5 / 0.37 |
| canopy 3.2 m valid | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 0; 0.00 | 160; 2.86 / 73 / 2.09 |
| white post on ground 0.3 m | 1 | (-1.47, 1.35); 1.67; 82 | 160; 0.01 | 82; 0.00 | 82; 0.00 / 0 / 0.00 |
| white wall full width 0.3 m | 1 | (-1.48, 1.34); 1.88; 160 | 160; 0.00 | refused | refused `ground_insufficient_points` -> zero hold; dv 2.00 |
| partly cloudy | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 67; 2.45 | 67; 2.45 / 60 / 1.45 |
| sun blob below horizon | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 12; 1.10 | 12; 1.10 / 21 / 0.45 |
| low wide box hiding ground (top 0.2) | 1 | (1.47, 1.35); 0.26; 0 | 160; 0.15 | 0; 2.73 | 0; 2.73 / 69 / 1.80 |
| slope 5 deg | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | refused | refused `ground_tilt_exceeds_tolerance` -> zero hold; dv 2.71 |
| REV P0-1 dark invalid rail 2 m (cam-0.11..-0.01) | 1 | (1.48, 1.34); 0.24; 0 | 160; 0.15 | n/a | 160; 0.15 / 4 / 0.32 |
| REV P0-1 dark invalid rail 3 m (cam-0.11..-0.01) | 1 | (1.59, 1.22); -0.07; 0 | 160; 0.32 | n/a | 124; 0.24 / 7 / 0.46 |
| REV P0-2 white invalid raised box 2 m (bottom cam-0.10) | 1 | (-1.47, 1.36); 1.97; 0 | 160; 0.02 | n/a | 158; 0.02 / 1 / 0.09 |
| REV P0-2 white invalid panel 1.5 m beside 5 m tree line | 1 | (1.63, 1.17); -0.19; 0 | 160; 0.39 | n/a | 90; 0.09 / 3 / 0.12 |
| REV P0-2 light-grey invalid altitude panel 1.5 m (cam-0.10..+0.45) | 1 | (-1.48, 1.35); 1.70; 0 | 160; 0.00 | n/a | 88; 0.01 / 0 / 0.03 |
| REV P0-2 white invalid panel 1.5 m pitch +10 (touches unimaged band) | 1 | (1.63, 1.17); -0.19; 0 | 160; 0.38 | n/a | 0; 2.50 / 62 / 1.35 |
| REV P0-3 white invalid canopy 2.0 m (10 m wide) | 1 | (1.50, 1.32); 0.19; 0 | 160; 0.18 | n/a | 0; 2.70 / 68 / 1.72 |
| REV P1-1 bridging box x3-8 top 0.2 + dark bar 2 m (cam+0.02..+0.12) | 1 | (1.46, 1.36); 0.47; 0 | 160; 0.13 | n/a | refused `ground_not_planar` -> zero hold; dv 2.00 |
| TWIN light-grey panel 1.5 m (cam+0.05..+0.55), valid edges | 1 | (1.63, 1.17); -0.19; 0 | 160; 0.39 | n/a | 82; 0.09 / 3 / 0.12 |
| TWIN light-grey panel 1.5 m pitch +10, valid edges | 1 | (1.63, 1.17); -0.19; 0 | 152; 0.38 | n/a | 82; 0.09 / 3 / 0.12 |
| TWIN white panel 3 m (cam+0.05..+0.55), valid edges | 1 | (1.99, 0.15); -0.94; 0 | 160; 1.44 | n/a | 40; 0.38 / 11 / 0.19 |
| TWIN light-grey raised box 2 m wholly above horizon, valid edges | 1 | (1.48, 1.35); 0.25; 0 | 152; 0.16 | n/a | 148; 0.16 / 4 / 0.31 |
| TWIN light-grey gazebo canopy 2.0 m, valid skirts/posts | 1 | (1.46, 1.37); 0.32; 0 | 160; 0.12 | n/a | 160; 0.12 / 3 / 0.24 |
| TWIN light-grey rail 3 m open horizon, valid posts/ends | 1 | (-1.40, 1.42); 1.96; 0 | 160; 0.11 | n/a | 114; 0.06 / 2 / 0.18 |
| PHANTOM trunks 3 m apart at 3 m (sky between) | 1 | (2.40, -0.44); -1.11; 0 | 132; 2.16 | n/a | 114; 2.09 / 55 / 1.67 |
| tree line 15 m | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| tree line 15 m pitch +5 | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.02 | 0; 0.02 / 0 / 0.01 |
| tree line 15 m pitch -5 | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.01 | 0; 0.01 / 0 / 0.01 |
| open field, far ground valid | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.32 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| open field, far ground invalid | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.32 | 0; 0.00 | 160; 2.32 / 60 / 1.58 |
| open field h1.5 pitch +5 | 2 | (2.45, -0.57); -1.39; 0 | 160; 2.18 | 0; 0.01 | 160; 2.18 / 56 / 1.36 |
| bars in open field (golden-like) | 2 | (-1.53, 1.28); 1.29; 0 | 151; 0.09 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| bars in open field pitch +3 | 2 | (-1.53, 1.28); 1.29; 0 | 152; 0.09 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| bars in open field h1.5 | 2 | (-1.56, 1.25); 1.35; 0 | 151; 0.15 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| trunk 0.3 m | 2 | (-1.49, 1.34); 1.29; 88 | 160; 0.01 | 90; 0.00 | 90; 0.00 / 0 / 0.00 |
| trunk 1.5 m | 2 | (2.00, 0.13); -1.02; 0 | 138; 1.44 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| branch above camera 0.3 m | 2 | (1.44, 1.39); 0.22; 160 | 160; 0.00 | 160; 0.00 | 160; 0.00 / 0 / 0.00 |
| branch above camera 1.0 m | 2 | (1.47, 1.35); 0.11; 0 | 160; 0.05 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| white object above horizon 0.3 m | 2 | (1.44, 1.39); 0.22; 160 | 160; 0.00 | 0; 2.32 | 0; 2.32 / 60 / 1.58 |
| wire 2 m 3 px (invalid) | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m valid | 2 | (1.55, 1.27); -0.11; 0 | 0; 0.00 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m invalid dark | 2 | (1.55, 1.27); -0.11; 0 | 160; 0.16 | 160; 0.16 | 160; 0.16 / 5 / 0.33 |
| canopy 3.2 m valid | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.00 | 160; 2.31 / 60 / 1.57 |
| white post on ground 0.3 m | 2 | (-1.49, 1.34); 1.29; 82 | 160; 0.01 | 82; 0.00 | 82; 0.00 / 0 / 0.00 |
| white wall full width 0.3 m | 2 | (-1.49, 1.33); 1.72; 160 | 160; 0.00 | refused | refused `ground_insufficient_points` -> zero hold; dv 2.00 |
| partly cloudy | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 67; 1.99 | 67; 1.99 / 50 / 1.17 |
| sun blob below horizon | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 12; 1.00 | 12; 1.00 / 21 / 0.37 |
| low wide box hiding ground (top 0.2) | 2 | (1.52, 1.30); -0.04; 0 | 160; 0.12 | 0; 2.20 | 0; 2.20 / 56 / 1.32 |
| slope 5 deg | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | refused | refused `ground_tilt_exceeds_tolerance` -> zero hold; dv 2.55 |
| REV P0-1 dark invalid rail 2 m (cam-0.11..-0.01) | 2 | (1.54, 1.28); -0.12; 0 | 160; 0.14 | n/a | 160; 0.14 / 4 / 0.27 |
| REV P0-1 dark invalid rail 3 m (cam-0.11..-0.01) | 2 | (1.63, 1.15); -0.36; 0 | 160; 0.29 | n/a | 124; 0.23 / 7 / 0.36 |
| REV P0-2 white invalid raised box 2 m (bottom cam-0.10) | 2 | (-1.50, 1.32); 1.79; 0 | 160; 0.01 | n/a | 158; 0.01 / 0 / 0.07 |
| REV P0-2 white invalid panel 1.5 m beside 5 m tree line | 2 | (1.67, 1.10); -0.40; 0 | 160; 0.36 | n/a | 90; 0.09 / 3 / 0.09 |
| REV P0-2 light-grey invalid altitude panel 1.5 m (cam-0.10..+0.45) | 2 | (-1.49, 1.33); 1.34; 0 | 160; 0.00 | n/a | 88; 0.01 / 0 / 0.06 |
| REV P0-2 white invalid panel 1.5 m pitch +10 (touches unimaged band) | 2 | (1.67, 1.10); -0.40; 0 | 160; 0.36 | n/a | 0; 1.95 / 49 / 0.95 |
| REV P0-3 white invalid canopy 2.0 m (10 m wide) | 2 | (1.55, 1.27); -0.11; 0 | 160; 0.16 | n/a | 0; 2.16 / 55 / 1.25 |
| REV P1-1 bridging box x3-8 top 0.2 + dark bar 2 m (cam+0.02..+0.12) | 2 | (1.50, 1.32); 0.24; 0 | 160; 0.10 | n/a | refused `ground_not_planar` -> zero hold; dv 2.00 |
| TWIN light-grey panel 1.5 m (cam+0.05..+0.55), valid edges | 2 | (1.67, 1.10); -0.40; 0 | 160; 0.36 | n/a | 82; 0.09 / 3 / 0.10 |
| TWIN light-grey panel 1.5 m pitch +10, valid edges | 2 | (1.67, 1.10); -0.40; 0 | 152; 0.36 | n/a | 82; 0.09 / 3 / 0.09 |
| TWIN white panel 3 m (cam+0.05..+0.55), valid edges | 2 | (1.96, 0.39); -0.89; 0 | 160; 1.13 | n/a | 40; 0.25 / 7 / 0.13 |
| TWIN light-grey raised box 2 m wholly above horizon, valid edges | 2 | (1.52, 1.30); -0.03; 0 | 152; 0.13 | n/a | 148; 0.13 / 4 / 0.28 |
| TWIN light-grey gazebo canopy 2.0 m, valid skirts/posts | 2 | (1.51, 1.31); 0.00; 0 | 160; 0.10 | n/a | 160; 0.10 / 3 / 0.21 |
| TWIN light-grey rail 3 m open horizon, valid posts/ends | 2 | (-1.48, 1.34); 1.69; 0 | 160; 0.02 | n/a | 114; 0.01 / 0 / 0.32 |
| PHANTOM trunks 3 m apart at 3 m (sky between) | 2 | (2.43, 0.25); -0.92; 0 | 132; 1.55 | n/a | 114; 1.46 / 37 / 1.20 |

## CPU cost (`timing_repair_r1.json`, `scratch/timing.py`; one thread, idle host, i7-13700K)

| Source | estimate_ground median / p95 ms | apply_sky_rule incl. veto | edge_enclosure alone |
|---|---|---|---|
| oakd_like | 4.06 / 4.17 | 1.92 / 1.97 | 1.34 / 1.37 |
| trained_equal | 4.09 / 4.24 | 2.26 / 2.30 | 1.34 / 1.37 |

r1 measured 3.85 / 0.76 ms (median) for the first two on its scenes; Orin NX: NOT_MEASURED.

## Files changed (diff against `r1_tree/files/`, unified diffs in `diff/`)

| Path | Lines | SHA-256 (16) after |
|---|---|---|
| `tools/camera_policy_contract.py` | +67 / -11 | 476441abe2558fde |
| `tools/camera_policy_runtime.py` | +205 / -49 | 014c5adeaa6b8986 |
| `tools/camera_ros2_session.py` | +25 / -1 | d4673be90469d8cf |
| `tools/camera_ros2_node.py` | +13 / -1 | 7fd8cb8f7b43481d |
| `ros2/motar_camera_policy/README.md` | +74 / -21 (outdoor section only; pre-change text unchanged, tested) | 5c147a46ee9d9059 |
| `docs/evidence/sitl_2026-11-05/camera_runtime/README.md` | +19 / -0 (appended paragraph) | 2928f6ac48f5de66 |
| `tools/oakd_outdoor_replay.py` | unchanged | a482a93d70a95e36 |
| `tests/test_camera_runtime_outdoor_r1.py` | +761 / -146 (63 tests, 7 expected failures) | f657ea5eaa6acb91 |

No other path changed: the same 26 uncommitted paths exist, and the other 19 keep the SHA-256 they had before this repair
(`repair_r1/start_uncommitted_26.sha256` vs `logs/repair_r1_final_tree.sha256`). New tests: `ObjectsWithValidEdges`
(4), `VetoPhantomCost` (2), `ReviewerScenesFixed` (2), `FullyInvalidLimitations` (6, 4 expected failures),
`GroundWrongSurface` (5, 1 expected failure), plus new or rewritten tests in `OutdoorContractProfile`, `SkyRule`,
`MeasuredLimitations` (2 expected failures), `GroundEstimate`, `OutdoorOffIsBitIdentical`, `SessionHoldOnRefusal`,
`OutdoorRecords` and the actual-policy class (panel twin). Expected failures assert the SAFE outcome.

## Verification

All CPU (`CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`, `python -B`,
`taskset -c 0-3,6-23`, `timeout -k`); cpu4/5 offline and unused. Logs in `../logs/` with the `repair_r1` prefix.

| Check | Result | Log |
|---|---|---|
| Focused module `tests/test_camera_runtime_outdoor_r1.py` (frozen tree) | `Ran 63 tests in 81.132s`, `OK (expected failures=7)` | `repair_r1_focused_outdoor.log` |
| All 15 `tests/test_camera_*` modules in one process | `Ran 224 tests in 110.668s`, `OK (expected failures=7)` | `repair_r1_camera_modules_one_process.log` |
| PUBLIC_CPU, baseline command `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python -B tools/run_tests.py`, frozen tree, idle host | **`Ran 4208 tests in 867.308s`, `OK (skipped=10, expected failures=7)`, exit 0**: 0 failures, 0 errors (r1: 4182) | `repair_r1_public_cpu_final.log` |
| Earlier PUBLIC_CPU run (informative only: a source comment was edited during it) | `Ran 4208 tests in 861.191s`, `OK (skipped=10, expected failures=7)` | `repair_r1_public_cpu_run1_informative.log` |
| Kernel faults (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`) | 0 at start; 0 before and after the focused/camera runs, the final suite, the probe reruns and the policy runs | `repair_r1_kfaults_*.txt`, `reviewer_rerun/kfaults_*`, `analysis/kfaults_*` |
| Tree frozen during the final runs | the 26 paths' SHA-256 equal before and after (`TREE_UNCHANGED_SINCE_FREEZE`) | `repair_r1_final_tree.sha256` |
| Gateway files | `tools/camera_ros2_px4.py` c487a5ea..., `tools/camera_ros2_px4_node.py` 9d714391...: equal to the start of this repair and to pre_tree | `repair_r1_gateway_after.sha256` |
| pre_tree | all 28 files byte-equal to the start of this repair | `start_pre_tree.sha256` |
| r1 snapshot | `r1_tree/SHA256SUMS` verifies | |
| Checkpoints (read only) | seed 1 888e8056b2a67683, seed 2 203deea459ef1d93, manifests 30ff4a68..., f28cfa2f...: all equal to the pins | `repair_r1_checkpoint_sha256.txt` |
| Reviewer folder, OUTDOOR_r1.md | probe scripts equal to the copies; no file in `review_r1/` newer than the r1 snapshot; `OUTDOOR_r1.md` SHA-256 = its `SHA256SUMS.txt` entry | |
| One-line README wording fix after the final suite ("`REPAIR_r1.md` in the unit folder above", evidence README only) | the 15 camera modules + `test_public_docs_consistency` in one process: `Ran 231 tests in 109.791s`, `OK (expected failures=7)`; kernel faults 0 -> 0; tree hashes `repair_r1_final_tree_after_readme_fix.sha256` (only that README differs from the frozen-tree list) | `repair_r1_camera_modules_after_readme_fix.log` |
| Byte identity (indoor/default) | goldens from pre_tree (fresh process) == goldens from the current tree: 14 scenes x 6 indoor contracts, 8 contract variants (`goldens_pre.json`, `goldens_new.json`); embedded in the test | |

Not done: independent review of this repair; ROS graph, SITL, OAK-D, PX4, GPU, flight (none was in scope).

## Residual risks (not resolved here; none is a flight-readiness statement)

1. **Fully sensor-invalid sky-like objects** (no valid edge, bright and smooth, wholly above the horizon) that connect
   to the image top, through low-contrast sky or through the unimaged band of a nose-down camera, are lost: light-grey
   panels, a 10 m-wide white canopy at 2.0-2.5 m, a white panel touching the band at 10-15 deg nose-down. One frame of
   depth plus luma cannot separate them from sky; resolving them needs RGB/temporal evidence or the OAK-D confidence
   map. Kept visible as 4 expected-failure tests.
2. **The twin assumption** (valid vertical silhouette edges on a bright invalid face) is synthetic. If the user's OAK-D
   does not resolve those edges at 160x90 after the z-buffer share rule, the twins fall into risk 1. Measure with
   `tools/oakd_outdoor_replay.py` on recordings of such objects.
3. **Phantom costs (safe direction, but they change behaviour):** sky between near trunks (up to 146 of 160 bearings in
   the sweep; policy dv 2.09/1.46 m/s for trunks 3 m apart at 3 m), sky below a removed canopy above 2.9 m (160),
   invalid far ground in an open field with the far-ground rule off (160; dv 2.87/2.32 m/s), plus r1's clouds and sun.
   An outdoor run in such scenes would mostly avoid phantom walls or stop.
4. **Thin invalid structures** under about 3 trained rows (0.10 m at 5 m) and invalid wires are not near (blob_near,
   pre-existing).
5. **Ground:** a horizontal surface hiding all ground within 10 m inside the PX4 tolerance is accepted (0.2 m top:
   -0.20 m); frames with a large raised surface beside the ground are refused (zero hold) even when r1 had the true
   ground; the site must be one plane within 3 deg; real stereo noise and vegetation are NOT_MEASURED.
6. **ROS path:** the outdoor profile cannot run on the node until z_valid/epv/dead_reckoning are forwarded (separate
   unit, gateway files out of scope). The refusal hold has no time limit (user decision). The generic session-fault
   path still leaves PX4's last setpoint for `COM_OF_LOSS_T` (r1 finding, unchanged).
7. All thresholds and the 10 fixed runtime constants are PROVISIONAL (synthetic only). Not independently reviewed yet:
   AGENTS.md requires a fresh-context Opus review of this repair before any gate is reported above CONDITIONAL.

## Records (all under `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/`)

This report `REPAIR_r1.md`; `r1_tree/` (pre-repair snapshot + SHA256SUMS); `repair_r1/`: `start_*.sha256`, `veto_sweep.json`,
`veto_dilation.json`, `ground_sweep.json`, `ground_sweep_summary.json`, `repair12.json`, `goldens_pre.json`,
`goldens_new.json`, `timing_repair_r1.json`, `explore3_span_wide_face.log`, `tables.md`, `reviewer_tables.md`,
`reviewer_probes_copy/`, `reviewer_rerun/`, `analysis/` (policy comparison), `diff/`, `scratch/` (all scripts);
`logs/repair_r1_*` (test logs, kernel fault counts, git status). `OUTDOOR_r1.md` and `review_r1/` were not edited.

## Git traceability

- Branch `camonly/dev`; Base SHA `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`; New commit SHA: none (no commit by
  instruction). Remote push: NO. Main merge: NO.
- Changed files (this repair, all uncommitted): `tools/camera_policy_contract.py`, `tools/camera_policy_runtime.py`,
  `tools/camera_ros2_session.py`, `tools/camera_ros2_node.py`, `ros2/motar_camera_policy/README.md`,
  `docs/evidence/sitl_2026-11-05/camera_runtime/README.md`, `tests/test_camera_runtime_outdoor_r1.py`.
- Working tree: 26 uncommitted paths, the same set as before (21 modified tracked + 5 untracked:
  `tests/test_camera_runtime_fixes.py`, `tests/test_camera_runtime_fixes_r2.py`, `tests/test_camera_runtime_outdoor_r1.py`,
  `tests/test_camera_runtime_repair_r1.py`, `tools/oakd_outdoor_replay.py`); `logs/repair_r1_git_status_after.txt`.
- Tests: Verification above. Raw/checkpoint integrity: checkpoints, `results/`, `aerial_gym/`, raw data untouched
  (pins verified); WORKLOG and Beads not edited (no commit in this unit).
