# Camera-runtime outdoor profile, repair r2 (2026-11-15): answers to the independent review r2

Worker: Claude Code subagent (Opus 5.5, `claude-opus-5-5`). Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`,
branch `camonly/dev`, HEAD `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde` (unchanged). Usage: `USAGE_UNKNOWN` (no
provider-visible signal; `tools/agent_session.py status` reports `usage_observation_status: UNKNOWN`; no estimate made).
Scope: the user's option A (fix what is clearly fixable, then stop synthetic patching). Paths: a relative path is
relative to `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/repair_r2/` unless it starts with `tools/`, `tests/`,
`ros2/` or `docs/` (worktree paths); `../` is the unit folder.

Before any edit the 8 outdoor files were snapshotted to `../r2_tree/files/` with `../r2_tree/SHA256SUMS` (plus
`../r2_tree/start_uncommitted_26.sha256`, equal to the review r2 after-hashes); `../pre_tree/` (24 files + 4 metadata
files) and `../r1_tree/` verified. The 26 uncommitted paths were preserved and only edited forward. No commit, `git add`, stash, reset,
checkout, push or merge. Nothing under `aerial_gym/`, `results/`, checkpoints, raw data, WORKLOG, Beads, the gateway
files `tools/camera_ros2_px4.py` / `tools/camera_ros2_px4_node.py`, any settings file, `review_r1/`, `review_r2/`,
`OUTDOOR_r1.md` or `REPAIR_r1.md` was touched (the review r2 probes were COPIED to `reviewer_probes_copy/`; only the
output folder constant `OUT` differs in `probe_common.py` and `q8_policy.py`). CPU only (`CUDA_VISIBLE_DEVICES=""`,
`PYTHONNOUSERSITE=1`, `python -B`, `taskset -c 0-3,6-23`, `timeout -k`), checkpoints read only.

**Evidence status: SYNTHETIC.** Every scene is the ray-cast OAK-D emulation of `tests/test_camera_runtime_outdoor_r1.py`
(`outdoor_scene`) with ASSUMED sensor values, including the assumption that a bright invalid face shows valid vertical
silhouette edges. Nothing here is real-sensor, PX4, ROS-graph, Gazebo or flight evidence. **No flight-readiness verdict
is given.** Every new threshold or constant is PROVISIONAL. The OAK-D Pro facts are documentation, not measurements of
the user's unit.



## Summary

- **P0-N1 (one edge / oblique): fixed on the review's scenes at a large, measured phantom cost.** In a row, one near
  valid edge now encloses an invalid run when the other side is the image border or any stop that is not a near valid
  return. The review's one-edge faces are kept (open horizon 36/36 scenes fully kept, was 25/36; poses 59/60, was 40/60;
  tree line 36/36, was 26/36; tall 6/6, was 2/6; oblique 15/15, was 12/15). **Cost:** one frame cannot tell such a face
  from sky beside a single near object, so the sky in the rows of ANY near valid return within 5.0 m that rises above
  the horizon stays near: a single trunk at 1.5-4.5 m ahead makes 138-152 of 160 bearings near (was 0), a trunk at the
  image border up to 151, a 5 cm post 157-160, 0.1 % synthetic sky speckle 160 (was 0). With the real policy, the
  r1 scene "trunk 1.5 m" goes from 0 near bearings and dv 0.00 m/s to 138 near bearings and dv 1.88 / 1.44 m/s
  (seeds 1 / 2) against the training-equivalent observation; a single trunk at 3 m: 148 near, dv 2.20 / 1.51 m/s. This
  is the safe direction the user set, but it removes most of the outdoor profile's benefit within 5 m of trees, posts
  and poles; the user should weigh it (residual risk 1).
- **P1-N3 (coverage-border connectors): fixed.** Only unimaged pixels above the source camera's imaged region connect
  the sky flood. A bug found in this round (fully uncovered side-border columns counted as "above") was fixed before the
  final measurements. Open-sky availability is unchanged (0 near bearings in 120 open-sky/tree-line scenes, pitch 0-20,
  roll 0/+-5/+-10, borders 0/12/30 px); an elevation margin was measured and not adopted.
- **P3-N7/N8/N10: fixed.** Re-entry count restarts on every history reset (outdoor only); the NPZ replay and the OAK-D
  replay tool apply / label the same hold; README wording corrected (one-edge class, what stays near).
- **Not fixed (no measured fix without new misses or with a bounded phantom):** P0-N2 (shallow-reach fully invalid
  objects), the edge-range part of P2-N4, P2-N6 (noise-driven ground refusals). Each is an `unittest.expectedFailure`
  test asserting the safe/correct outcome (`RepairR2Residuals`). The sub-pixel part of P2-N4 is fixed by the one-edge rule.
- **OAK-D Pro:** documented facts with sources (accessed 2026-11-15) are in the ROS README; the existing `oakd_like` test
  preset matches the fixed-focus variant; the auto-focus variant (66/54 deg in the docs, about 51.9 deg VFOV on its 4:3
  sensor) does not cover the trained FOV and is refused fail-closed (new preset `oakd_pro_af`, test `OakdProPresets`).
  The runtime's projector state is unchanged (off by default). The capture protocol now records each scene with the
  projector off and on, the device model/variant/serial, and capture items for every open question.

## Design decisions (what changed and why)

1. **One-edge enclosure, rows only** (`tools/camera_policy_runtime.py:edge_enclosure`, review r2 P0-N1). Per row scan,
   each side's stop (the first pixel that is not sensor-invalid) is a near edge when it is a valid return of the
   projection (floor returns excluded) within `edge_veto_range_m` (5.0 m). New: a run with a near edge on exactly one
   side is enclosed when the other side is the image border or any non-near stop (unimaged/uncovered, no-return beyond
   MaxZ, valid beyond 5.0 m, floor). Two near edges that are not one face (see 2) still do not enclose (the user's
   model: two different objects). Columns keep the two-edge rule: the one-edge rule on columns was measured
   (`r2_rows_cols`) and added a 160-bearing phantom under a dark branch above the camera without keeping any extra
   column; the OAK-D's horizontal baseline ranges vertical silhouette edges, the premise of the veto.
2. **Oblique faces** (same function). Two near edges whose ranges differ by more than `edge_veto_range_tolerance_m`
   (1.0 m) enclose when the chord between their 3-D points is seen at an incidence (angle between the chord's normal
   and the line of sight to its midpoint) of at most `EDGE_VETO_MAX_INCIDENCE_DEG` = 70 deg (PROVISIONAL runtime
   constant). The review's oblique faces need 51 deg (1.60 m range difference) and 58-59 deg (2.19 / 2.21 m). In the
   655-scene sweep, 45/60/70/80/90 deg and "no oblique rule" gave identical kept columns and phantom bearings: the
   one-edge rule plus the 2-pixel dilation already kept every oblique face (a 4 cm end post is not resolved in every
   row). The oblique rule is exercised on synthetic masks (`EdgeEnclosureRules`) and kept for faces whose two edges are
   resolved in every row.
3. **Top-band connectors** (`project_rgbd(..., unimaged=True)`, `_first_covered_rows`, review r2 P1-N3). A trained pixel
   connects the sky flood only if no sample landed on it and its ray lies above the source camera's imaged region:
   above the top FOV edge, or in the uncovered border above the first covered row of its source column (a fully
   uncovered side-border column takes the first covered row of the nearest covered column). Uncovered side and bottom
   borders, rays beyond a side and rays below the bottom edge never connect. A first implementation counted fully
   uncovered side columns as "above" (their first covered row defaulted to the image height); the sweep exposed it
   (`sweep_r2_all_prefix_bug.json`: fully invalid objects beside a 12/30 px border still lost 6/18) and it was fixed
   before every final measurement. An elevation margin for connectors (3/6/10 deg) was measured and NOT adopted: 3 and
   6 deg kept no extra column (396/616 in 56 band-touching scenes, as 0 deg), 10 deg kept 22 more columns but turned
   the open sky into a 160-bearing wall in 6 of 120 availability scenes (pitch 20). Removing the connectors altogether
   gives 75 of 120 availability scenes with near walls.
4. **Re-entry hold on every history reset** (`tools/camera_ros2_session.py`, review r2 P3-N7). With an outdoor-profile
   runtime the count `REFUSAL_REENTRY_FRAMES` (5, now defined in the runtime and imported by the session) is set on
   `reset()` (session start, explicit reset), when an altitude-gated tick finishes (the next in-band frame starts a
   fresh history) and on a refusal. Indoor contracts never refuse frames and keep 0 (their proposals are unchanged).
5. **Replays** (review r2 P3-N8). `camera_policy_runtime.replay` (NPZ) applies the same hold for outdoor contracts: the
   first 5 accepted frames and 5 after every refusal propose the zero hold, keep the policy's command as
   `policy_velocity_world_enu_mps` / `policy_yaw_rate_rps`, record `previous_action` [0, 0, 0, 0] and call
   `runtime.applied_zero_hold()`; the report adds `reentry_held_frames`. Indoor replays are byte-identical in content
   (tested). `tools/oakd_outdoor_replay.py` (perception only, no policy) marks accepted frames `reentry_hold` and adds
   `reentry_hold_frames_after_reset`, `reentry_held_frames`, `frames_free_to_command` to the summary.
6. **README** (`ros2/motar_camera_policy/README.md`, outdoor section only; P3-N10): the one-edge and oblique rules with
   their cost, the top-band connectors, the corrected "what stays near" (objects reaching fewer than about 3 trained rows
   below the horizon are lost; blob_near's 5 x 5 window at density 0.5), the re-entry restart, new limitations, the
   OAK-D Pro section (documented facts, DepthAI 2.29 emitter API, consequences) and the updated capture protocol.
   `docs/evidence/sitl_2026-11-05/camera_runtime/README.md`: one appended repair-r2 paragraph. Every new number in both
   READMEs is tested against its source (`OutdoorRecords.test_repair_r2_numbers_match_their_sources`; OAK-D Pro facts
   against the machine-readable `OAKD_PRO_DOCS` record in the test module).
7. **Not changed:** `tools/camera_policy_contract.py` (no new contract key; contract hashes of the runtime and the
   session change, so an existing contract file must be rebuilt), `tools/camera_ros2_node.py`, default projector state,
   the gateway files, every indoor/default code path (embedded golden digests pass).

## Review findings -> status

Labels: FIXED (on the measured synthetic scope), PARTIAL, OPEN, NOT_ADDRESSED. "Before" = the review r2 tree (repair
r1 state, `r2_tree/`), measured either by the review's own probes (`review_r2/probes/*.json`) or by the pre-round runtime
loaded from `r2_tree/files/` in the sweep harness (`scratch/harness.py`, variant `r1`, checked equal to that runtime).

| Finding | Status | Numbers (before -> after) | Tests |
|---|---|---|---|
| r2 P0-N1 one-edge faces | **FIXED** on the review's scenes; 1 residual geometry (PARTIAL overall) | Review probes rerun on copies: q1b one-edge 4871/6856 cols (51/76 scenes full) -> 6789/6856 (75/76); q1 A 2374/3324 (26/36) -> 3324/3324 (36/36); q1 B only-right-edge 126/192 -> 192/192; q1 D tall 179/537 (2/6) -> 537/537 (6/6). Residual: light grey at 4 m, pitch +5: 17/84 (its 3 cm edge is narrower than one trained pixel on the tilted path and vanishes under the z-buffer share rule). Policy: one-edge panel 2 m open horizon 0 near, dv 2.53 / 2.07 -> 160 near, dv 0.32 / 0.24 m/s | `EdgeEnclosureRules`, `OneEdgeAndObliqueFaces` |
| r2 P0-N1 oblique faces | **FIXED** | q2 a 606/669 (8/9 full) -> 669/669 (9/9); sweep 880/1080 (12/15) -> 1080/1080 (15/15). Policy oblique 1.6 m: 156 near, dv 1.04 / 1.01 | `OneEdgeAndObliqueFaces.test_oblique_faces`, `EdgeEnclosureRules` |
| r2 P0-N1 cost (new phantoms) | measured, not tuned (safe direction) | single trunk 1.5/3/4.5 m: 0 -> 138-152 near bearings; trunk at the image border (1.5/3 m): 0 -> up to 151; 5 cm post 2 m: 0 -> 157-160; trunk pairs: 538 -> 1717 summed over 20 open scenes; synthetic sky speckle 0.1 %: 0 -> 160. Unchanged 0: open sky, tree line, ranged far ground, objects beyond 5 m, posts whose top stays below the camera | `OneEdgePhantomCost`, updated `VetoPhantomCost`, `NearObstaclesStayDetected` |
| r2 P0-N2 shallow-reach fully invalid objects | **OPEN** | unchanged (sweep: 28/36 open-horizon scenes full at lower edges 0.02/0.05/0.10 m below the camera). Attempt "keep a column run that continues k rows below the horizon": k=1 keeps 35/36 but makes open sky a 160-bearing wall (1319 near bearings over 10 scenes) and a sun at the horizon 160; k=2 keeps 31/36 and adds phantoms (open sky up to 14, thin far-ground bands, sun); k=3 changes nothing. Not adopted | `RepairR2Residuals.test_fully_invalid_light_grey_panel_reaching_just_below_the_horizon_is_detected` (xfail), `test_controls_of_the_shallow_reach_class` |
| r2 P1-N3 coverage-border connectors | **FIXED** | q2 f 2024/2208 (22/24) -> 2208/2208 (24/24); fully invalid objects beside a 12/30 px border 96/144 (12/18) -> 144/144 (18/18); availability: 0 near bearings in 120 open-sky/tree-line scenes before and after | `SkyConnectorTopBandOnly`, `OneEdgeAndObliqueFaces.test_one_edge_face_beside_a_side_coverage_border` |
| r2 P2-N4 sub-pixel edge alignment | **FIXED** (by the one-edge rule) | q2 c 770/868 (30/36) -> 868/868 (36/36); sweep incl. 5.0 m: 952/1158 (40/54) -> 1158/1158 (54/54). Tilted-path residual: see P0-N1 residual | sweep only (`scratch/sweep_r2.py`) |
| r2 P2-N4 edge range (edges beyond 5 m) | **OPEN** | 4.8 m face, edges at 5.19 m: 0/106 before and after; policy dv 2.59 / 2.11 m/s, 0 near. Raising `edge_veto_range_m` enlarges every one-edge phantom (not attempted beyond analysis) | `RepairR2Residuals.test_face_whose_edges_lie_beyond_the_edge_range_is_detected` (xfail), control `test_face_with_edges_inside_the_edge_range_is_kept` |
| r2 P2-N5 band exposure with roll | **PARTIAL** | lowest connector elevation at the image sides rises with the top-band restriction (e.g. pitch 15 roll 5 left 0.97 -> 14.9 deg; pitch 20 roll 0 sides 1.62 -> 7.42 deg); the rolled top band itself still reaches 2.27 deg (pitch 20, roll 10, right). Fully invalid objects touching the band: 392/616 -> 396/616 kept (26/56 full); without any connector 608/616 | `n5_probe.json` |
| r2 P2-N6 ground margins vs noise | **OPEN** | unchanged: flat-ground refusals 1/1/7/24 of 60 at 0.15/0.25/0.35/0.5 px; max accepted height error 0.134 m (0.25 px, far-only patch) | `RepairR2Residuals.test_flat_ground_with_half_pixel_disparity_noise_is_accepted` (xfail), `test_flat_ground_noise_refusal_is_named_and_the_low_noise_control_is_accepted` |
| r2 P3-N7 re-entry count | **FIXED** | review q6 rerun: S3 (altitude gate mid re-entry) now holds 5 fresh-history frames after the gate (was 3); S4 (explicit reset) and S6 (first start) now start with the hold (was 0; the review's scripted plans end inside it, the tests assert exactly 5); S1/S2 5 as before | `SessionReentryRestartsOnEveryHistoryReset` (3), updated `SessionHoldOnRefusal` |
| r2 P3-N8 replays | **FIXED** | NPZ replay holds 2 x 5 frames around one refusal in the test; indoor replay unchanged | `NpzReplayRefusal` (2), `OakdOutdoorReplayTool` |
| r2 P3-N9 contract refusal messages / diagnostics | **NOT_ADDRESSED** (outside the user's option-A list) | contract layer untouched this round; messages as in review r2 | - |
| r2 P3-N10 README wording | **FIXED** | one-edge class, what stays near (fewer than about 3 trained rows below the horizon) | `OutdoorRecords` |
| r1 P0-2a bright invalid objects | **PARTIAL** | one-edge / oblique / both-edge twins kept; fully invalid ones (no valid edge) still lost | `FullyInvalidLimitations` (4 xfail) |
| r1 P0-3 bright invalid canopy | **OPEN** | 10 m white canopy at 2.0 m: 0/160 | xfail |
| r1 P1-1 wrong-surface ground | **PARTIAL** (unchanged) | 0.12 / 0.25 m platforms hiding the ground accepted at -0.12 / -0.25 m (review r2 q5, rerun identical) | `GroundWrongSurface` xfail |
| r1 P2-1 ROS node / hold | **PARTIAL** (unchanged) | node refuses outdoor contracts until PX4 vertical fields are forwarded; hold time limit = user decision | - |
| r1 P2-2 invalid thin wires | **OPEN** | `[0]*8`, `[0]*4` | 2 xfail |

## Measurement tables

Sweep: `scratch/sweep_r2.py` over `scratch/scenes.py` (655 scenes: the review r2 failing scenes q1/q1b/q2 a-f re-created, N2/N5 scenes, the repair-r1 twins, phantom scenes, OAK-D Pro presets, 120 connector-availability scenes), records `sweep_r2_all.json`, `tables_r2_all.md/.json`, `report_tables.md`; N2 attempt `scratch/sweep_n2.py` -> `sweep_r2_n2.json`; N5 `scratch/n5_probe.py` -> `n5_probe.json`. Each scene is projected once; the sky stage then runs per variant. Variants: `r1` = the pre-round runtime's enclosure and connectors (equivalence asserted against `r2_tree/files/tools/camera_policy_runtime.py`); `r2` = this repair's default (asserted equal to the production geometry runtime); `r2_no_one_edge` = oblique rule only; `r2_rows_cols` = one-edge also on columns; `r2_reach{0.5,1,2}` = one-edge closure limited to that lateral distance (m) from the near edge (informational, not adopted); `r2_conn_r1` / `r2_conn_none` = pre-round connectors / none; `r2_elev{3,6,10}` = connector elevation margin (deg); `crossK` = P0-N2 attempt. Kernel faults 0 -> 0 for every run (`kf_sweep*_*.txt`).

### Kept object columns: sum kept / sum columns (scenes fully kept / scenes)

| group | r1 | r2 | r2_no_one_edge | r2_conn_r1 | r2_reach1 | r2_reach2 |
|---|---|---|---|---|---|---|
| q1b_one_edge | 2264/3249 (25/36) | 3249/3249 (36/36) | 2264/3249 (25/36) | 3249/3249 (36/36) | 2887/3249 (27/36) | 3208/3249 (33/36) |
| q1b_control_both_edges | 1494/1494 (36/36) | 1494/1494 (36/36) | 1494/1494 (36/36) | 1494/1494 (36/36) | 1494/1494 (36/36) | 1494/1494 (36/36) |
| N2_fully_invalid_open | 1062/1494 (25/36) | 1062/1494 (25/36) | 1062/1494 (25/36) | 1062/1494 (25/36) | 1062/1494 (25/36) | 1062/1494 (25/36) |
| q1b_one_edge_pose | 4033/5415 (40/60) | 5348/5415 (59/60) | 4033/5415 (40/60) | 5348/5415 (59/60) | 4915/5415 (45/60) | 5308/5415 (55/60) |
| q1b_one_edge_h1.5 | 179/358 (2/4) | 358/358 (4/4) | 179/358 (2/4) | 358/358 (4/4) | 289/358 (2/4) | 358/358 (4/4) |
| q1A_one_edge_tree | 2374/3324 (26/36) | 3324/3324 (36/36) | 2466/3324 (27/36) | 3324/3324 (36/36) | 3234/3324 (32/36) | 3324/3324 (36/36) |
| q1B_only_right_edge | 126/192 (3/4) | 192/192 (4/4) | 126/192 (3/4) | 192/192 (4/4) | 192/192 (4/4) | 192/192 (4/4) |
| q1B_only_none_edge | 126/192 (3/4) | 126/192 (3/4) | 126/192 (3/4) | 126/192 (3/4) | 126/192 (3/4) | 126/192 (3/4) |
| N2_fully_invalid_tree | 510/576 (11/12) | 510/576 (11/12) | 510/576 (11/12) | 510/576 (11/12) | 510/576 (11/12) | 510/576 (11/12) |
| q1D_tall_one_edge | 179/537 (2/6) | 537/537 (6/6) | 179/537 (2/6) | 537/537 (6/6) | 399/537 (2/6) | 537/537 (6/6) |
| q2a_oblique | 880/1080 (12/15) | 1080/1080 (15/15) | 1080/1080 (15/15) | 1080/1080 (15/15) | 1080/1080 (15/15) | 1080/1080 (15/15) |
| q2b_edge_range | 538/644 (5/6) | 538/644 (5/6) | 538/644 (5/6) | 538/644 (5/6) | 538/644 (5/6) | 538/644 (5/6) |
| q2c_subpixel | 952/1158 (40/54) | 1158/1158 (54/54) | 952/1158 (40/54) | 1158/1158 (54/54) | 1158/1158 (54/54) | 1158/1158 (54/54) |
| q2d_twin_band | 1368/1368 (24/24) | 1368/1368 (24/24) | 1368/1368 (24/24) | 1368/1368 (24/24) | 1368/1368 (24/24) | 1368/1368 (24/24) |
| q2f_coverage_border | 1104/1656 (12/18) | 1656/1656 (18/18) | 1656/1656 (18/18) | 1656/1656 (18/18) | 1656/1656 (18/18) | 1656/1656 (18/18) |
| q2f_fully_invalid_border | 96/144 (12/18) | 144/144 (18/18) | 144/144 (18/18) | 96/144 (12/18) | 144/144 (18/18) | 144/144 (18/18) |
| N5_side_band | 112/112 (8/8) | 112/112 (8/8) | 112/112 (8/8) | 112/112 (8/8) | 112/112 (8/8) | 112/112 (8/8) |
| r1_twin | 2156/2156 (26/26) | 2156/2156 (26/26) | 2156/2156 (26/26) | 2156/2156 (26/26) | 2156/2156 (26/26) | 2156/2156 (26/26) |
| oakdpro_one_edge | 209/644 (0/7) | 644/644 (7/7) | 209/644 (0/7) | 644/644 (7/7) | 531/644 (0/7) | 644/644 (7/7) |

### Phantom near (< 1 m) bearings: sum over the group's scenes (max per scene)

| group | scenes | r1 | r2 | r2_no_one_edge | r2_rows_cols | r2_reach0.5 | r2_reach1 | r2_reach2 | r2_conn_none |
|---|---|---|---|---|---|---|---|---|---|
| open_sky | 10 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 640 (160) |
| tree_line | 10 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 640 (160) |
| open_field_far_invalid | 10 | 1600 (160) | 1600 (160) | 1600 (160) | 1600 (160) | 1600 (160) | 1600 (160) | 1600 (160) | 1600 (160) |
| trunk_pair_tree | 20 | 508 (146) | 1505 (152) | 508 (146) | 1505 (152) | 867 (146) | 1071 (146) | 1285 (146) | 1505 (152) |
| forest_edge_tree | 1 | 98 (98) | 115 (115) | 98 (98) | 115 (115) | 115 (115) | 115 (115) | 115 (115) | 115 (115) |
| single_trunk_centre_tree | 4 | 0 (0) | 434 (152) | 0 (0) | 434 (152) | 186 (106) | 294 (134) | 416 (148) | 434 (152) |
| trunk_at_image_border_tree | 8 | 0 (0) | 702 (145) | 0 (0) | 702 (145) | 164 (46) | 316 (94) | 515 (134) | 702 (145) |
| thin_post_tree | 2 | 0 (0) | 317 (160) | 0 (0) | 317 (160) | 108 (71) | 201 (131) | 300 (157) | 317 (160) |
| short_post_tree | 2 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| trunk_pair_open | 20 | 538 (146) | 1717 (152) | 629 (146) | 1717 (152) | 1022 (146) | 1261 (146) | 1497 (146) | 1717 (152) |
| forest_edge_open | 1 | 98 (98) | 115 (115) | 98 (98) | 115 (115) | 115 (115) | 115 (115) | 115 (115) | 115 (115) |
| single_trunk_centre_open | 4 | 0 (0) | 434 (152) | 0 (0) | 434 (152) | 188 (108) | 295 (134) | 418 (148) | 434 (152) |
| trunk_at_image_border_open | 8 | 0 (0) | 860 (151) | 0 (0) | 860 (151) | 196 (46) | 360 (93) | 586 (134) | 860 (151) |
| thin_post_open | 2 | 0 (0) | 317 (160) | 0 (0) | 317 (160) | 109 (72) | 202 (132) | 301 (157) | 317 (160) |
| short_post_open | 2 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| branch_above | 1 | 0 (0) | 0 (0) | 0 (0) | 160 (160) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| canopy_valid | 2 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| sky_speckle | 4 | 304 (160) | 640 (160) | 304 (160) | 640 (160) | 586 (160) | 640 (160) | 640 (160) | 640 (160) |
| oakdpro_open_sky | 7 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 958 (160) |
| oakdpro_tree_line | 7 | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 796 (160) |

### Connector availability (open sky / tree line, pitch 0-20 x roll 0/5/10/-10 x coverage border 0/12/30 px; near bearings)

| variant | scenes | sum | scenes with a near bearing | max |
|---|---|---|---|---|
| r1 | 120 | 0 | 0 | 0 |
| r2 | 120 | 0 | 0 | 0 |
| r2_conn_none | 120 | 10410 | 75 | 160 |
| r2_elev3 | 120 | 0 | 0 | 0 |
| r2_elev6 | 120 | 0 | 0 | 0 |
| r2_elev10 | 120 | 927 | 6 | 160 |

### P0-N2 attempt: keep a column run that continues k rows below the horizon (crossK)

| group | kind | r1 | r2 | r2_cross1 | r2_cross2 | r2_cross3 |
|---|---|---|---|---|---|---|
| N2_fully_invalid_open | keep | 1206/1494 (28/36) | 1206/1494 (28/36) | 1472/1494 (35/36) | 1306/1494 (31/36) | 1206/1494 (28/36) |
| N2_fully_invalid_tree | keep | 1428/1494 (35/36) | 1428/1494 (35/36) | 1494/1494 (36/36) | 1428/1494 (35/36) | 1428/1494 (35/36) |
| N2_fully_invalid_pose | keep | 1290/1494 (29/36) | 1290/1494 (29/36) | 1494/1494 (36/36) | 1472/1494 (35/36) | 1290/1494 (29/36) |
| n2_open_sky | phantom | 0 (max 0) | 0 (max 0) | 1319 (max 160) | 28 (max 14) | 0 (max 0) |
| n2_thin_far_band | phantom | 4988 (max 160) | 4988 (max 160) | 6378 (max 160) | 5379 (max 160) | 4994 (max 160) |
| n2_far_invalid | phantom | 1600 (max 160) | 1600 (max 160) | 1600 (max 160) | 1600 (max 160) | 1600 (max 160) |
| n2_tree_line | phantom | 0 (max 0) | 0 (max 0) | 0 (max 0) | 0 (max 0) | 0 (max 0) |
| n2_cloudy | phantom | 672 (max 76) | 672 (max 76) | 672 (max 76) | 672 (max 76) | 672 (max 76) |
| n2_sun_at_horizon | phantom | 113 (max 12) | 113 (max 12) | 1336 (max 160) | 134 (max 19) | 113 (max 12) |

### P2-N5: lowest connector elevation (deg) at the image sides / centre, pre-round (r1) vs top band (r2)

| pitch | roll | r1 left | r1 centre | r1 right | r2 left | r2 centre | r2 right |
|---|---|---|---|---|---|---|---|
| 0 | 0 | None | None | None | None | None | None |
| 0 | 5 | None | None | 25.17 | None | None | 25.17 |
| 0 | 10 | 17.88 | None | 23.01 | None | None | 23.01 |
| 5 | 0 | 23.01 | 23.01 | 23.01 | 23.01 | 23.01 | 23.01 |
| 5 | 5 | 14.9 | 22.45 | 20.2 | 24.63 | 22.45 | 20.2 |
| 5 | 10 | 9.95 | 22.45 | 17.88 | None | 22.45 | 17.88 |
| 10 | 0 | 16.7 | 17.88 | 16.7 | 17.3 | 17.88 | 17.3 |
| 10 | 5 | 6.78 | 17.88 | 14.9 | 20.2 | 17.88 | 14.9 |
| 10 | 10 | 3.56 | 17.88 | 12.45 | 22.45 | 17.88 | 12.45 |
| 15 | 0 | 7.42 | 13.07 | 7.42 | 12.45 | 13.07 | 12.45 |
| 15 | 5 | 0.97 | 12.45 | 9.95 | 14.9 | 12.45 | 9.95 |
| 15 | 10 | 0.32 | 13.07 | 7.42 | 17.88 | 13.07 | 7.42 |
| 20 | 0 | 1.62 | 8.06 | 1.62 | 7.42 | 8.06 | 7.42 |
| 20 | 5 | 0.32 | 7.42 | 4.85 | 10.58 | 7.42 | 4.85 |
| 20 | 10 | 0.32 | 8.06 | 2.27 | 13.07 | 8.06 | 2.27 |

Fully invalid light-grey/white objects at the image side touching the band (56 scenes): kept columns r1 392/616 (26 full), r2_elev0 396/616 (26 full), r2_elev3 396/616 (26 full), r2_elev6 396/616 (26 full), r2_elev10 418/616 (29 full), r2_none 608/616 (48 full).

### Review r2 probes rerun on copies (before = the review's own outputs)

Copies in `reviewer_probes_copy/` (only `OUT` changed in `probe_common.py` and `q8_policy.py`), outputs in `reviewer_rerun/`, comparison `reviewer_compare.md/.json` (`scratch/reviewer_compare.py`). q5 (ground) rerun identical in its summary; q9 (expected failures): 10 of 10 fail by `AssertionError` (the 7 of r1 and 3 new).

### q1_one_edge (kept columns before -> after; scenes fully kept)

| group | scenes | before (review r2) | after (repair r2) |
|---|---|---|---|
| A one-edge-in-view | 36 | 2374/3324 (26 full) | 3324/3324 (36 full) |
| A0 CONTROL | 9 | 432/432 (9 full) | 432/432 (9 full) |
| B only-right-edge | 4 | 126/192 (3 full) | 192/192 (4 full) |
| B only-none-edge | 4 | 126/192 (3 full) | 126/192 (3 full) |
| C fully-invalid | 27 | 1230/1296 (26 full) | 1230/1296 (26 full) |
| D tall | 6 | 179/537 (2 full) | 537/537 (6 full) |
| D0 CONTROL | 6 | 234/234 (6 full) | 234/234 (6 full) |

### q1b_open_horizon (kept columns before -> after; scenes fully kept)

| group | scenes | before (review r2) | after (repair r2) |
|---|---|---|---|
| E one-edge-in-view | 76 | 4871/6856 (51 full) | 6789/6856 (75 full) |
| E0 CONTROL | 72 | 2988/2988 (72 full) | 2988/2988 (72 full) |
| E1 no-edge-in-view(fully | 36 | 1062/1494 (25 full) | 1062/1494 (25 full) |

### q2_veto_geometry (kept columns before -> after; scenes fully kept)

| group | scenes | before (review r2) | after (repair r2) |
|---|---|---|---|
| a_oblique | 9 | 606/669 (8 full) | 669/669 (9 full) |
| b_edge_range | 6 | 538/644 (5 full) | 538/644 (5 full) |
| c_edge_resolution | 36 | 770/868 (30 full) | 868/868 (36 full) |
| d_twin_band | 24 | 1368/1368 (24 full) | 1368/1368 (24 full) |
| f_coverage_border | 24 | 2024/2208 (22 full) | 2208/2208 (24 full) |

## Observation and actual-policy comparison (seeds 1 and 2)

`analysis/obs_policy_compare_repair_r2.py` = the repair-r1 script (same method: 30 frames at 10 Hz of the static
scene, CAM-K2 v3 checkpoints read only through `CameraPolicyRuntime`, CPU, one fresh process per seed) with the repair-r2
outdoor default for (iii), its 40 scenes, the review r2 failing scenes (one-edge 2 m open horizon and tree line, one-edge
4 m, one-edge pitch +5, white one-edge pitch +10, oblique 1.6 m, both-edge control, N2 3 m -0.05 m, N3 30 px border
roll 5, N4 sub-pixel 4.5 m and edge range 5.19 m) and the new phantom scenes (single trunk 3 / 4.5 / 6 m, trunk at the
image border, thin post). dv = |v_iii - v_i| (m/s) against the training-equivalent observation (i) (level camera, no
ground, sky as no-return, exact depth); ddir = direction change (deg), dyaw = yaw-rate change (rad/s). "repair r1 (iii)"
comes from `../repair_r1/analysis/obs_policy_compare_s*.json`, or for four scenes from the review's `q8_policy_s*.json`
(marked). Checkpoints `888e8056b2a67683...` and `203deea459ef1d93...` unchanged before/after
(`analysis/checkpoint_sha256_{before,after}.txt`); kernel faults 0 -> 0. Records: `analysis/obs_policy_compare_s1.json`,
`_s2.json`, `policy_table.md`, `policy_summary.json`.

Changes against repair r1 (iii), both seeds: every scene of the r1 list is identical except the ones with a near valid
return above the horizon within 5 m: bars in an open field (golden-like, three scenes) 0 -> 151-152 near, dv 0.00 ->
0.09-0.18; trunk 1.5 m 0 -> 138 near, dv 0.00 -> 1.88 / 1.44; light-grey panel twin 1.5 m (both pitches) 82 -> 152-154
near, dv 0.09 -> 0.36-0.39; light-grey raised-box twin 148 -> 152; rail twin 3 m 114 -> 150; trunks 3 m apart at 3 m
114 -> 132 near, dv 2.09 / 1.46 -> 2.16 / 1.55.

| Scene | Seed | (i) training-equiv. v_FLU m/s; yaw rad/s; near | (ii) indoor default: near; dv | repair r1 (iii): near; dv | repair r2 (iii): near; dv m/s / ddir deg / dyaw rad/s |
|---|---|---|---|---|---|
| tree line 15 m | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| tree line 15 m pitch +5 | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 0; 0.02 | 0; 0.02 / 0 / 0.01 |
| tree line 15 m pitch -5 | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 0; 0.01 | 0; 0.01 / 0 / 0.00 |
| open field, far ground valid | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| open field, far ground invalid | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 160; 2.87 | 160; 2.87 / 73 / 2.09 |
| open field h1.5 pitch +5 | 1 | (2.42, -0.98); -1.51; 0 | 160; 2.63 | 160; 2.63 | 160; 2.63 / 68 / 1.69 |
| bars in open field (golden-like) | 1 | (-1.53, 1.29); 1.69; 0 | 151; 0.12 | 0; 0.00 | 151; 0.12 / 3 / 0.13 |
| bars in open field pitch +3 | 1 | (-1.53, 1.29); 1.69; 0 | 152; 0.12 | 0; 0.00 | 152; 0.12 / 3 / 0.13 |
| bars in open field h1.5 | 1 | (-1.57, 1.24); 1.64; 0 | 151; 0.18 | 0; 0.00 | 151; 0.18 / 5 / 0.11 |
| trunk 0.3 m | 1 | (-1.47, 1.35); 1.67; 88 | 160; 0.01 | 90; 0.00 | 90; 0.00 / 0 / 0.00 |
| trunk 1.5 m | 1 | (1.98, -0.27); -1.13; 0 | 138; 1.88 | 0; 0.00 | 138; 1.88 / 56 / 2.00 |
| branch above camera 0.3 m | 1 | (1.37, 1.46); 0.56; 160 | 160; 0.00 | 160; 0.00 | 160; 0.00 / 0 / 0.00 |
| branch above camera 1.0 m | 1 | (1.41, 1.41); 0.44; 0 | 160; 0.06 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| white object above horizon 0.3 m | 1 | (1.37, 1.46); 0.56; 160 | 160; 0.00 | 0; 2.87 | 0; 2.87 / 73 / 2.09 |
| wire 2 m 3 px (invalid) | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m valid | 1 | (1.50, 1.32); 0.19; 0 | 0; 0.00 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m invalid dark | 1 | (1.50, 1.32); 0.19; 0 | 160; 0.18 | 160; 0.18 | 160; 0.18 / 5 / 0.37 |
| canopy 3.2 m valid | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 160; 2.86 | 160; 2.86 / 73 / 2.09 |
| white post on ground 0.3 m | 1 | (-1.47, 1.35); 1.67; 82 | 160; 0.01 | 82; 0.00 | 82; 0.00 / 0 / 0.00 |
| white wall full width 0.3 m | 1 | (-1.48, 1.34); 1.88; 160 | 160; 0.00 | refused | refused `ground_insufficient_points` -> zero hold; dv 2.00 |
| partly cloudy | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.86 | 67; 2.45 | 67; 2.45 / 60 / 1.45 |
| sun blob below horizon | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | 12; 1.10 | 12; 1.10 / 21 / 0.45 |
| low wide box hiding ground (top 0.2) | 1 | (1.47, 1.35); 0.26; 0 | 160; 0.15 | 0; 2.73 | 0; 2.73 / 69 / 1.80 |
| slope 5 deg | 1 | (2.43, -1.21); -1.54; 0 | 160; 2.87 | refused | refused `ground_tilt_exceeds_tolerance` -> zero hold; dv 2.71 |
| REV P0-1 dark invalid rail 2 m (cam-0.11..-0.01) | 1 | (1.48, 1.34); 0.24; 0 | 160; 0.15 | 160; 0.15 | 160; 0.15 / 4 / 0.32 |
| REV P0-1 dark invalid rail 3 m (cam-0.11..-0.01) | 1 | (1.59, 1.22); -0.07; 0 | 160; 0.32 | 124; 0.24 | 124; 0.24 / 7 / 0.46 |
| REV P0-2 white invalid raised box 2 m (bottom cam-0.10) | 1 | (-1.47, 1.36); 1.97; 0 | 160; 0.02 | 158; 0.02 | 158; 0.02 / 1 / 0.09 |
| REV P0-2 white invalid panel 1.5 m beside 5 m tree line | 1 | (1.63, 1.17); -0.19; 0 | 160; 0.39 | 90; 0.09 | 90; 0.09 / 3 / 0.12 |
| REV P0-2 light-grey invalid altitude panel 1.5 m (cam-0.10..+0.45) | 1 | (-1.48, 1.35); 1.70; 0 | 160; 0.00 | 88; 0.01 | 88; 0.01 / 0 / 0.03 |
| REV P0-2 white invalid panel 1.5 m pitch +10 (touches unimaged band) | 1 | (1.63, 1.17); -0.19; 0 | 160; 0.38 | 0; 2.50 | 0; 2.50 / 62 / 1.35 |
| REV P0-3 white invalid canopy 2.0 m (10 m wide) | 1 | (1.50, 1.32); 0.19; 0 | 160; 0.18 | 0; 2.70 | 0; 2.70 / 68 / 1.72 |
| REV P1-1 bridging box x3-8 top 0.2 + dark bar 2 m (cam+0.02..+0.12) | 1 | (1.46, 1.36); 0.47; 0 | 160; 0.13 | refused | refused `ground_not_planar` -> zero hold; dv 2.00 |
| TWIN light-grey panel 1.5 m (cam+0.05..+0.55), valid edges | 1 | (1.63, 1.17); -0.19; 0 | 160; 0.39 | 82; 0.09 | 154; 0.39 / 11 / 0.74 |
| TWIN light-grey panel 1.5 m pitch +10, valid edges | 1 | (1.63, 1.17); -0.19; 0 | 152; 0.38 | 82; 0.09 | 152; 0.38 / 11 / 0.73 |
| TWIN white panel 3 m (cam+0.05..+0.55), valid edges | 1 | (1.99, 0.15); -0.94; 0 | 160; 1.44 | 40; 0.38 | 40; 0.38 / 11 / 0.19 |
| TWIN light-grey raised box 2 m wholly above horizon, valid edges | 1 | (1.48, 1.35); 0.25; 0 | 152; 0.16 | 148; 0.16 | 152; 0.16 / 4 / 0.31 |
| TWIN light-grey gazebo canopy 2.0 m, valid skirts/posts | 1 | (1.46, 1.37); 0.32; 0 | 160; 0.12 | 160; 0.12 | 160; 0.12 / 3 / 0.24 |
| TWIN light-grey rail 3 m open horizon, valid posts/ends | 1 | (-1.40, 1.42); 1.96; 0 | 160; 0.11 | 114; 0.06 | 150; 0.07 / 2 / 0.15 |
| PHANTOM trunks 3 m apart at 3 m (sky between) | 1 | (2.40, -0.44); -1.11; 0 | 132; 2.16 | 114; 2.09 | 132; 2.16 / 57 / 1.91 |
| REV2 P0-N1 lgrey panel 2 m one edge in view (cam+0.03..+0.30), open horizon | 1 | (1.58, 1.22); -0.10; 0 | 160; 0.32 | 0; 2.53 (review r2 q8) | 160; 0.32 / 9 / 0.66 |
| REV2 P0-N1 lgrey panel 2 m one edge in view (cam+0.03..+0.30), tree line | 1 | (1.58, 1.22); -0.10; 0 | 160; 0.32 | 98; 0.35 (review r2 q8) | 98; 0.35 / 10 / 0.45 |
| REV2 P0-N1 lgrey panel 4 m one edge in view (cam+0.03..+0.30), open horizon | 1 | (2.39, 0.03); -0.92; 0 | 160; 1.75 | n/a | 160; 1.75 / 46 / 1.47 |
| REV2 P0-N1 lgrey panel 2 m one edge in view, pitch +5 | 1 | (1.58, 1.22); -0.10; 0 | 160; 0.32 | n/a | 160; 0.32 / 9 / 0.64 |
| REV2 P0-N1 white panel 2 m one edge in view, pitch +10 (unimaged band) | 1 | (1.58, 1.22); -0.10; 0 | 160; 0.32 | n/a | 160; 0.32 / 9 / 0.65 |
| REV2 P0-N1 lgrey oblique face 2.0->3.6 m (edge ranges differ 1.60 m) | 1 | (2.41, 1.39); -0.32; 0 | 160; 1.04 | n/a | 156; 1.04 / 17 / 0.87 |
| REV2 CONTROL lgrey panel 2 m both edges (cam+0.03..+0.30), open horizon | 1 | (2.42, 1.06); -0.55; 0 | 160; 1.12 | 60; 0.69 (review r2 q8) | 160; 1.12 / 23 / 1.11 |
| REV2 P0-N2 lgrey fully invalid panel 3 m reaching cam-0.05, open horizon | 1 | (1.99, 0.15); -0.92; 0 | 160; 1.45 | 0; 1.43 (review r2 q8) | 0; 1.43 / 31 / 0.60 |
| REV2 P1-N3 white one-edge panel 2 m, 30 px coverage border, roll 5 | 1 | (1.58, 1.22); -0.10; 0 | 160; 0.32 | n/a | 97; 0.34 / 10 / 0.44 |
| REV2 P2-N4 lgrey twin 4.5 m, lateral offset 0.015 m (level path) | 1 | (2.42, -0.14); -1.07; 0 | 160; 1.92 | n/a | 160; 1.92 / 50 / 1.63 |
| REV2 P2-N4 lgrey face 4.8 m, edges at 5.19 m ray range | 1 | (2.42, 1.42); -0.26; 0 | 160; 1.06 | n/a | 0; 2.59 / 56 / 1.26 |
| PHANTOM single trunk 3 m ahead | 1 | (2.41, -0.43); -1.18; 0 | 148; 2.20 | n/a | 148; 2.20 / 58 / 2.04 |
| PHANTOM single trunk 4.5 m ahead | 1 | (2.42, -0.55); -1.24; 0 | 152; 2.30 | n/a | 152; 2.30 / 61 / 2.08 |
| PHANTOM trunk 3 m at the image border (29 deg left) | 1 | (2.42, -1.46); -1.54; 0 | 145; 3.10 | n/a | 145; 3.10 / 78 / 2.25 |
| PHANTOM thin post 2 m (5 cm, 2.5 m tall) | 1 | (2.43, -1.21); -1.51; 0 | 157; 2.88 | n/a | 157; 2.88 / 73 / 2.12 |
| PHANTOM single trunk 6 m ahead (beyond the 5 m edge range) | 1 | (2.42, -0.67); -1.30; 0 | 154; 2.40 | n/a | 0; 0.01 / 0 / 0.01 |
| tree line 15 m | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| tree line 15 m pitch +5 | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.02 | 0; 0.02 / 0 / 0.01 |
| tree line 15 m pitch -5 | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.01 | 0; 0.01 / 0 / 0.01 |
| open field, far ground valid | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.32 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| open field, far ground invalid | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.32 | 160; 2.32 | 160; 2.32 / 60 / 1.58 |
| open field h1.5 pitch +5 | 2 | (2.45, -0.57); -1.39; 0 | 160; 2.18 | 160; 2.18 | 160; 2.18 / 56 / 1.36 |
| bars in open field (golden-like) | 2 | (-1.53, 1.28); 1.29; 0 | 151; 0.09 | 0; 0.00 | 151; 0.09 / 3 / 0.19 |
| bars in open field pitch +3 | 2 | (-1.53, 1.28); 1.29; 0 | 152; 0.09 | 0; 0.00 | 152; 0.09 / 3 / 0.19 |
| bars in open field h1.5 | 2 | (-1.56, 1.25); 1.35; 0 | 151; 0.15 | 0; 0.00 | 151; 0.15 / 4 / 0.25 |
| trunk 0.3 m | 2 | (-1.49, 1.34); 1.29; 88 | 160; 0.01 | 90; 0.00 | 90; 0.00 / 0 / 0.00 |
| trunk 1.5 m | 2 | (2.00, 0.13); -1.02; 0 | 138; 1.44 | 0; 0.00 | 138; 1.44 / 42 / 1.72 |
| branch above camera 0.3 m | 2 | (1.44, 1.39); 0.22; 160 | 160; 0.00 | 160; 0.00 | 160; 0.00 / 0 / 0.00 |
| branch above camera 1.0 m | 2 | (1.47, 1.35); 0.11; 0 | 160; 0.05 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| white object above horizon 0.3 m | 2 | (1.44, 1.39); 0.22; 160 | 160; 0.00 | 0; 2.32 | 0; 2.32 / 60 / 1.58 |
| wire 2 m 3 px (invalid) | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m valid | 2 | (1.55, 1.27); -0.11; 0 | 0; 0.00 | 0; 0.00 | 0; 0.00 / 0 / 0.00 |
| canopy 2.0 m invalid dark | 2 | (1.55, 1.27); -0.11; 0 | 160; 0.16 | 160; 0.16 | 160; 0.16 / 5 / 0.33 |
| canopy 3.2 m valid | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 160; 2.31 | 160; 2.31 / 60 / 1.57 |
| white post on ground 0.3 m | 2 | (-1.49, 1.34); 1.29; 82 | 160; 0.01 | 82; 0.00 | 82; 0.00 / 0 / 0.00 |
| white wall full width 0.3 m | 2 | (-1.49, 1.33); 1.72; 160 | 160; 0.00 | refused | refused `ground_insufficient_points` -> zero hold; dv 2.00 |
| partly cloudy | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 67; 1.99 | 67; 1.99 / 50 / 1.17 |
| sun blob below horizon | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | 12; 1.00 | 12; 1.00 / 21 / 0.37 |
| low wide box hiding ground (top 0.2) | 2 | (1.52, 1.30); -0.04; 0 | 160; 0.12 | 0; 2.20 | 0; 2.20 / 56 / 1.32 |
| slope 5 deg | 2 | (2.45, -0.69); -1.36; 0 | 160; 2.31 | refused | refused `ground_tilt_exceeds_tolerance` -> zero hold; dv 2.55 |
| REV P0-1 dark invalid rail 2 m (cam-0.11..-0.01) | 2 | (1.54, 1.28); -0.12; 0 | 160; 0.14 | 160; 0.14 | 160; 0.14 / 4 / 0.27 |
| REV P0-1 dark invalid rail 3 m (cam-0.11..-0.01) | 2 | (1.63, 1.15); -0.36; 0 | 160; 0.29 | 124; 0.23 | 124; 0.23 / 7 / 0.36 |
| REV P0-2 white invalid raised box 2 m (bottom cam-0.10) | 2 | (-1.50, 1.32); 1.79; 0 | 160; 0.01 | 158; 0.01 | 158; 0.01 / 0 / 0.07 |
| REV P0-2 white invalid panel 1.5 m beside 5 m tree line | 2 | (1.67, 1.10); -0.40; 0 | 160; 0.36 | 90; 0.09 | 90; 0.09 / 3 / 0.09 |
| REV P0-2 light-grey invalid altitude panel 1.5 m (cam-0.10..+0.45) | 2 | (-1.49, 1.33); 1.34; 0 | 160; 0.00 | 88; 0.01 | 88; 0.01 / 0 / 0.06 |
| REV P0-2 white invalid panel 1.5 m pitch +10 (touches unimaged band) | 2 | (1.67, 1.10); -0.40; 0 | 160; 0.36 | 0; 1.95 | 0; 1.95 / 49 / 0.95 |
| REV P0-3 white invalid canopy 2.0 m (10 m wide) | 2 | (1.55, 1.27); -0.11; 0 | 160; 0.16 | 0; 2.16 | 0; 2.16 / 55 / 1.25 |
| REV P1-1 bridging box x3-8 top 0.2 + dark bar 2 m (cam+0.02..+0.12) | 2 | (1.50, 1.32); 0.24; 0 | 160; 0.10 | refused | refused `ground_not_planar` -> zero hold; dv 2.00 |
| TWIN light-grey panel 1.5 m (cam+0.05..+0.55), valid edges | 2 | (1.67, 1.10); -0.40; 0 | 160; 0.36 | 82; 0.09 | 154; 0.36 / 10 / 0.62 |
| TWIN light-grey panel 1.5 m pitch +10, valid edges | 2 | (1.67, 1.10); -0.40; 0 | 152; 0.36 | 82; 0.09 | 152; 0.36 / 10 / 0.61 |
| TWIN white panel 3 m (cam+0.05..+0.55), valid edges | 2 | (1.96, 0.39); -0.89; 0 | 160; 1.13 | 40; 0.25 | 40; 0.25 / 7 / 0.13 |
| TWIN light-grey raised box 2 m wholly above horizon, valid edges | 2 | (1.52, 1.30); -0.03; 0 | 152; 0.13 | 148; 0.13 | 152; 0.13 / 4 / 0.28 |
| TWIN light-grey gazebo canopy 2.0 m, valid skirts/posts | 2 | (1.51, 1.31); 0.00; 0 | 160; 0.10 | 160; 0.10 | 160; 0.10 / 3 / 0.21 |
| TWIN light-grey rail 3 m open horizon, valid posts/ends | 2 | (-1.48, 1.34); 1.69; 0 | 160; 0.02 | 114; 0.01 | 150; 0.01 / 0 / 0.20 |
| PHANTOM trunks 3 m apart at 3 m (sky between) | 2 | (2.43, 0.25); -0.92; 0 | 132; 1.55 | 114; 1.46 | 132; 1.55 / 39 / 1.52 |
| REV2 P0-N1 lgrey panel 2 m one edge in view (cam+0.03..+0.30), open horizon | 2 | (1.59, 1.21); -0.16; 0 | 160; 0.24 | 0; 2.07 (review r2 q8) | 160; 0.24 / 7 / 0.41 |
| REV2 P0-N1 lgrey panel 2 m one edge in view (cam+0.03..+0.30), tree line | 2 | (1.59, 1.21); -0.16; 0 | 160; 0.24 | 98; 0.31 (review r2 q8) | 98; 0.31 / 9 / 0.47 |
| REV2 P0-N1 lgrey panel 4 m one edge in view (cam+0.03..+0.30), open horizon | 2 | (2.44, 0.54); -0.79; 0 | 160; 1.32 | n/a | 160; 1.32 / 32 / 1.04 |
| REV2 P0-N1 lgrey panel 2 m one edge in view, pitch +5 | 2 | (1.59, 1.21); -0.16; 0 | 160; 0.24 | n/a | 160; 0.24 / 7 / 0.41 |
| REV2 P0-N1 white panel 2 m one edge in view, pitch +10 (unimaged band) | 2 | (1.59, 1.21); -0.16; 0 | 160; 0.23 | n/a | 160; 0.23 / 7 / 0.39 |
| REV2 P0-N1 lgrey oblique face 2.0->3.6 m (edge ranges differ 1.60 m) | 2 | (2.45, 1.37); -0.44; 0 | 160; 1.01 | n/a | 156; 1.01 / 15 / 0.69 |
| REV2 CONTROL lgrey panel 2 m both edges (cam+0.03..+0.30), open horizon | 2 | (2.46, 1.08); -0.64; 0 | 160; 1.07 | 60; 0.72 (review r2 q8) | 160; 1.07 / 20 / 0.89 |
| REV2 P0-N2 lgrey fully invalid panel 3 m reaching cam-0.05, open horizon | 2 | (1.96, 0.39); -0.87; 0 | 160; 1.13 | 0; 1.19 (review r2 q8) | 0; 1.19 / 27 / 0.46 |
| REV2 P1-N3 white one-edge panel 2 m, 30 px coverage border, roll 5 | 2 | (1.59, 1.21); -0.16; 0 | 160; 0.24 | n/a | 97; 0.30 / 9 / 0.49 |
| REV2 P2-N4 lgrey twin 4.5 m, lateral offset 0.015 m (level path) | 2 | (2.45, 0.21); -1.00; 0 | 160; 1.56 | n/a | 160; 1.56 / 39 / 1.22 |
| REV2 P2-N4 lgrey face 4.8 m, edges at 5.19 m ray range | 2 | (2.46, 1.43); -0.38; 0 | 160; 1.03 | n/a | 0; 2.11 / 46 / 0.96 |
| PHANTOM single trunk 3 m ahead | 2 | (2.00, 0.05); -1.06; 0 | 148; 1.51 | n/a | 148; 1.51 / 44 / 1.74 |
| PHANTOM single trunk 4.5 m ahead | 2 | (2.45, -0.09); -1.12; 0 | 152; 1.84 | n/a | 152; 1.84 / 48 / 1.76 |
| PHANTOM trunk 3 m at the image border (29 deg left) | 2 | (2.44, -0.55); -1.25; 0 | 145; 2.21 | n/a | 145; 2.21 / 58 / 1.77 |
| PHANTOM thin post 2 m (5 cm, 2.5 m tall) | 2 | (2.45, -0.69); -1.33; 0 | 157; 2.32 | n/a | 157; 2.32 / 60 / 1.60 |
| PHANTOM single trunk 6 m ahead (beyond the 5 m edge range) | 2 | (2.45, -0.20); -1.17; 0 | 154; 1.93 | n/a | 0; 0.01 / 0 / 0.01 |

## OAK-D Pro facts (Luxonis documentation, accessed 2026-11-15)

Collected by a research subagent (Sonnet) from the pages below with verbatim quotes; I checked the values it reported
against each other and wrote them into `OAKD_PRO_DOCS` (test module) and the ROS README table, which a test compares.
Nothing below is measured on the user's unit; its variant (auto or fixed focus) and serial are NOT_MEASURED.

| Fact | Value | Source |
|---|---|---|
| Stereo pair | OV9282 monochrome 1/4", global shutter, fixed focus, DFOV / HFOV / VFOV 89.5 / 80 / 55 deg, 1280 x 800 | https://docs.luxonis.com/hardware/products/OAK-D%20Pro ("DFOV / HFOV / VFOV: 89.5° / 80° / 55°"); resolution from the datasheet ("Active pixels 1280x800@120FPS") |
| RGB, auto-focus variant | IMX378 colour 1/2.3", rolling shutter, 78 / 66 / 54 deg, 4056 x 3040, AF 8 cm - inf | product page ("DFOV / HFOV / VFOV: 78° / 66° / 54°"); datasheet ("Active pixels 4056x3040@60fps") |
| RGB, fixed-focus variant | IMX378, 82 / 69 / 55 deg, fixed focus 50 cm - inf | product page ("82° / 69° / 55°"); datasheet |
| Baseline | 7.5 cm | product page ("a baseline of 7.5 cm (75 mm)") |
| IR filters | 940 nm notch on the stereo pair; the colour camera does not perceive IR | product page |
| IR dot projector | VCSEL 940 nm, Belago 1.1, about 4700 dots, Class 1, up to 1 W, strobed from the left camera | datasheet section 6.1; product page ("Dot projector (active stereo): Up to 1W") |
| IR flood LED | 940 nm, SFH 4725AS, FOI 80 deg, up to 1 W; for the mono streams in low light | datasheet section 6.2; product page |
| IMU | BNO086 (9-axis) | product page |
| Depth | ideal range ~80 cm - 12 m; MinZ ~80 cm at 800P, ~40 cm at 800P + extended; depth 0 = invalid/unknown; disparity search 0-95 px (extended 0-190 px) | product page; https://docs.luxonis.com/hardware/platform/depth/configuring-stereo-depth.md |
| Datasheet | Dec 2021 | https://github.com/luxonis/depthai-hardware/blob/master/DM9098_OAK-D-Pro/Datasheet/OAK-D-Pro_Datasheet.pdf |
| DepthAI 2.29 emitter API | `Device.setIrLaserDotProjectorIntensity(intensity, mask=-1)`, `Device.setIrFloodLightIntensity(intensity, mask=-1)`: intensity 0..1, "0 or negative to turn off", "The emitter is turned off by default"; projector up to 765 mA at 30 % duty (up to 1200 mA at shorter duty), flood normalised to 0-1500 mA; deprecated mA setters `setIrLaserDotProjectorBrightness` / `setIrFloodLightBrightness`; `getIrDrivers()` (OAK-D-Pro: `[("LM3644", 2, 0x63)]`) | https://raw.githubusercontent.com/luxonis/depthai-core/v2.29.0/include/depthai/device/DeviceBase.hpp (identical to the depthai-core commit `d6a37a5b` pinned by depthai-python v2.29.0.0); https://raw.githubusercontent.com/luxonis/depthai-python/v2.29.0.0/src/DeviceBindings.cpp |

NOT_VERIFIED in the documentation: how the sky or very far surfaces appear (only "0 means invalid/unknown" and "farther
than MaxZ will not be seen"); the location and width of the invalid band left by the disparity search in the RGB-aligned
depth; whether `setDepthAlign` leaves uncovered borders; the default confidence threshold; the projector's current per
intensity step. These are capture items.

**Consequences used in this repair (PROVISIONAL until measured on the unit).**
- `oakd_like` (1014 x 760, 69 deg HFOV; 4:3 gives 54.5 deg VFOV) matches the fixed-focus variant (docs 69 / 55 deg); it
  is unchanged. It leaves about 7.8 source pixels above and below the trained rows (the trained VFOV is 54 deg), so a
  top rectification border wider than that uncovers the trained top rows; they are top-band connectors.
- The auto-focus variant does not match: 66 deg HFOV equals the trained HFOV, and its 4:3 sensor gives about 51.9 deg
  VFOV at that HFOV (the docs list 54 deg, which is inconsistent with 4:3 and 78 deg DFOV). New test preset
  `oakd_pro_af` (in the outdoor test module; `test_camera_runtime_fixes.py` untouched). Measured: the level path refuses
  the frame (`source camera does not cover the trained FOV`), `oakd_camera.Rectifier.require_training_fov` refuses the
  intrinsics (the FF intrinsics pass), and on the tilted path (pitch 1 and 5 deg) open sky behind a tree line stays
  free (0 near bearings). If the user's unit is the AF variant, the existing fail-closed check would stop every frame
  until a calibrated FOV shows coverage.
- Projector state unchanged: neither the runtime nor `tools/oakd_camera.build_pipeline` calls the emitter setters, so
  the boot default (off) holds. The capture protocol (ROS README, steps 1, 5 and 6) now records the device model,
  variant, serial and DepthAI version, each scene with the projector off and on, and measurement items for the
  rectification border, the stereo search band, far ground beyond 20 m, the vertical silhouette edges of bright and
  light-grey faces (frontal, one edge out of view, 30-60 deg), sky/cloud/sun and sky speckle, thin wires/branches at
  1-5 m and flat-ground depth noise.

## Files changed (diff against `../r2_tree/files/`, unified diffs in `diff/`)

| Path | Lines (+ / -) | SHA-256 (16) after |
|---|---|---|
| `tools/camera_policy_runtime.py` | +102 / -26 | 5f9de284d65fc6c3 |
| `tools/camera_ros2_session.py` | +17 / -6 | ba958186d86f2f98 |
| `tools/oakd_outdoor_replay.py` | +10 / -1 | 2b3bd830e9d9e8f8 |
| `ros2/motar_camera_policy/README.md` | +93 / -14 (outdoor section only; the pre-change prefix is hash-tested) | 54aee240347f2809 |
| `docs/evidence/sitl_2026-11-05/camera_runtime/README.md` | +16 / -0 (appended paragraph) | a35f381798ce383a |
| `tests/test_camera_runtime_outdoor_r1.py` | +509 / -21 (86 tests, 10 expected failures; was 63 / 7) | 62fc9319f8ed26d1 |
| `tools/camera_policy_contract.py` | unchanged | 476441abe2558fde |
| `tools/camera_ros2_node.py` | unchanged | 7fd8cb8f7b43481d |

No other path changed: the same 26 uncommitted paths exist and the other 20 keep the SHA-256 they had at the start of
this round (`../r2_tree/start_uncommitted_26.sha256` vs `../logs/repair_r2_tree_final.sha256`). New test
classes: `SessionReentryRestartsOnEveryHistoryReset` (3), `EdgeEnclosureRules` (2), `OneEdgeAndObliqueFaces` (4),
`OneEdgePhantomCost` (2), `SkyConnectorTopBandOnly` (2), `RepairR2Residuals` (6, 3 expected failures), `OakdProPresets`
(2), plus `NpzReplayRefusal.test_indoor_replay_is_unchanged` and `OutdoorRecords.test_repair_r2_numbers_match_their_sources`;
updated: `SessionHoldOnRefusal` (start hold), `NpzReplayRefusal` (hysteresis), `OakdOutdoorReplayTool` (labels),
`NearObstaclesStayDetected.test_tree_trunk_close_is_near_and_far_is_ranged` and `VetoPhantomCost.test_no_phantom_in_open_scenes`
(the one-edge phantom is now asserted; their open-scene controls moved beyond the 5 m edge range),
`SkyRule.test_sky_rule_never_alters_valid_depth_and_needs_the_verified_plane` (control trunk at 6 m),
`GroundWrongSurface.test_fixed_runtime_constants_are_marked_provisional` (12 constants). The `outdoor_scene` helper
accepts the new `oakd_pro_af` source; `oakd_like` is unchanged.

## Verification

All CPU (`CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`, `python -B`,
`taskset -c 0-3,6-23`, `timeout -k`); cpu4/5 offline and unused. Logs in `../logs/` with the `repair_r2` prefix.

| Check | Result | Log / record |
|---|---|---|
| Focused module `tests/test_camera_runtime_outdoor_r1.py` | `Ran 86 tests`, `OK (expected failures=10)` (inside the one-process runs below; development runs in `logs_tmp/`) | `logs_tmp/focused_try3.log` and the runs below |
| All 15 `tests/test_camera_*` modules in one process | `Ran 247 tests in 153.676s`, `OK (expected failures=10)` | `repair_r2_camera_modules_one_process.log` |
| PUBLIC_CPU, `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python -B tools/run_tests.py`, frozen tree, idle host | **`Ran 4231 tests in 912.953s`, `OK (skipped=10, expected failures=10)`, exit 0**: 0 failures, 0 errors (repair r1: 4208) | `repair_r2_public_cpu.log`, `repair_r2_public_cpu_exit.txt` |
| Tree frozen during PUBLIC_CPU | the 26 paths' SHA-256 equal before and after (`TREE_UNCHANGED_DURING_RUN`) | `repair_r2_tree_frozen_public_cpu.sha256`, `repair_r2_tree_after_public_cpu.sha256` |
| Edit after the full run (README wording only: measured numbers without a machine-readable source removed from the ROS README and the evidence README) | the 15 camera modules + `test_public_docs_consistency` in one process: `Ran 254 tests in 156.330s`, `OK (expected failures=10)`; tree unchanged during that run | `repair_r2_camera_modules_and_docs_after_readme_fix.log`, `repair_r2_tree_final.sha256` |
| Expected failures | 10 of 10 fail by `AssertionError` on the safe/correct outcome (7 of r1, 3 new) | `xfail_r2.json`, `reviewer_rerun/q9_xfail.log` |
| Indoor/default byte identity | `OutdoorOffIsBitIdentical` (embedded pre_tree golden digests, 14 scenes x 6 indoor contracts + contract variants; projection's 6 default outputs equal with and without `unimaged`) passes; indoor session never holds (`test_indoor_session_never_holds`); indoor NPZ replay unchanged (`test_indoor_replay_is_unchanged`) | runs above |
| Harness equivalence | `prepare()+sky_stage()` == production geometry runtime on 4 scenes; variant `r1` == pre-round runtime (`r2_tree` copy) | `scratch/harness.py:check_equivalence` (asserted at the start of each sweep) |
| Kernel faults (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`) | 0 at start; 0 before and after every sweep, probe rerun, policy run, the camera runs and PUBLIC_CPU | `kf_*`, `reviewer_rerun/kf_*`, `analysis/kfaults_*`, `../logs/repair_r2_kfaults_*` |
| Gateway files | `tools/camera_ros2_px4.py` c487a5ea..., `tools/camera_ros2_px4_node.py` 9d714391...: byte-equal to `pre_tree` | `../logs/repair_r2_gateway_check.txt`, `../logs/repair_r2_gateway_after.sha256` |
| pre_tree / r1_tree / r2_tree | 24/24 files OK + 4 metadata hashes equal to `repair_r1/start_pre_tree.sha256`; r1_tree 8/8 OK; r2_tree 8/8 OK | console (recorded here) |
| Checkpoints (read only) | seed 1 888e8056b2a67683..., seed 2 203deea459ef1d93...: equal to the pins before and after the policy runs | `analysis/checkpoint_sha256_{before,after}.txt` |
| `aerial_gym/`, `results/` | no status entries; no `__pycache__` created | console |
| Review folders | `review_r1/`, `review_r2/`, `OUTDOOR_r1.md`, `REPAIR_r1.md` not edited | `review_r2_dir_end.sha256` |

Not done: independent review of this repair; ROS graph, SITL, OAK-D, PX4, GPU, flight (none was in scope).

## Residual risks (complete list as far as known; none is a flight-readiness statement)

1. **One-edge phantom walls (new, large).** Sky in every row of any near valid return within 5.0 m that rises above
   the horizon stays near: one trunk, post, pole, sign post or branch end within 5 m makes 138-160 of 160 bearings
   near (policy dv up to 3.10 m/s against the training-equivalent command); valid sky speckle within 5 m does the same
   (0.1 % synthetic speckle: 160). In parks and near trees the outdoor profile then behaves like the indoor default (a
   near wall) and the vehicle holds or turns away. The informational reach-bounded variants show the trade-off is
   intrinsic in one frame: limiting the closure to 1 m (2 m) from the edge keeps 27/36 (33/36) open-horizon one-edge
   faces and still leaves 294-295 (416-418) near bearings over 4 single-trunk scenes (r2: 434). Deciding between this
   cost and losing one-edge faces, or using more evidence (RGB, temporal, OAK-D confidence), is the user's decision.
2. **Fully sensor-invalid sky-like objects** (no valid edge, bright and smooth) are lost when wholly above the horizon
   or reaching fewer than about 3 trained rows below it (P0-N2, r1 P0-2a/P0-3: light-grey panels, a 10 m white canopy
   at 2.0 m, white panels touching the unimaged band at 10-15 deg nose-down; 392-396/616 columns kept when touching the
   band with roll). Kept as 5 expected failures.
3. **Edge resolution and range.** A face whose valid edges lie beyond 5.0 m ray range is lost (4.8 m face, edges
   5.19 m: 0/106; policy dv 2.59 / 2.11 m/s). A face whose only valid edge is narrower than about one trained pixel on
   the tilted path disappears under the z-buffer share rule (3 cm at 4 m, pitch +5: 17/84 kept).
4. **The twin assumption is unmeasured.** Every protected bright face relies on its vertical silhouette edge being
   matched by the OAK-D Pro's stereo. A light-grey face against a sky of similar luma may give no matched edge at all,
   which puts it into risk 2. Capture item "bright faces" measures this.
5. **Oblique rule unexercised by scenes.** `EDGE_VETO_MAX_INCIDENCE_DEG` changed no scene of the sweep (the one-edge
   rule already kept every oblique face); it is tested on synthetic masks only.
6. **Unimaged band exposure** remains for fully invalid objects (risk 2): the rolled top band reaches 2.27 deg above the
   horizon at pitch 20 / roll 10 on one side (7.4-8.1 deg at roll 0).
7. **Invalid far ground** (full or a thin depth-0 band just below the horizon) is a phantom band on most bearings with
   the far-ground rule off (thin band beyond 60-200 m: up to 160 bearings in the sweep); textured clouds (52-76 near
   bearings) and a sun blob stay near as in r1. Real OAK-D far-ground behaviour is NOT_MEASURED.
8. **Thin invalid structures** (wires 1-3 px, rails thinner than about 3 trained rows) are not detected (r1 P2-2).
9. **Ground estimate.** A horizontal surface hiding all ground within the PX4 tolerance is accepted (0.12 / 0.25 m
   platforms at -0.12 / -0.25 m); a far-only patch was accepted at -0.134 m with 0.25 px noise; flat ground is refused
   more often with more stereo noise (24/60 at 0.5 px, a zero hold); the site must be one plane within 3 deg;
   vegetation and the OAK-D Pro's disparity noise are NOT_MEASURED.
10. **Re-entry hold (now on every history reset).** An outdoor session holds 0.5 s after start, reset, an
    altitude-gated stretch or a refusal; with intermittent refusals the hold can last indefinitely (no time limit, the
    user's decision). The NPZ replay and the OAK-D replay tool start each recording with the hold too.
11. **ROS path.** The node still refuses outdoor contracts until PX4's `z_valid` / `epv` / `dead_reckoning` are forwarded
    (separate unit; gateway untouched). A generic session fault or a stopped node still leaves PX4's last setpoint for
    `COM_OF_LOSS_T`. Contract-layer refusal messages and the additive diagnostics (r2 P3-N9) are unchanged.
12. **OAK-D Pro specifics.** If the user's unit is the auto-focus variant, its documented FOV does not cover the trained
    FOV (fail closed: no frame passes the level path or `require_training_fov`). The disparity search may leave a
    permanent depth-0 band at one image side of the RGB-aligned depth, which the runtime would treat as sensor-invalid
    (near) on every frame, indoors too (NOT_MEASURED; capture item). The projector's effect outdoors is NOT_MEASURED;
    the RGB camera does not see IR.
13. **CPU cost.** `edge_enclosure` doubled (median 1.33 -> 2.67 ms on the i7-13700K, one thread, idle host; whole sky
    rule 3.20 ms); Orin NX NOT_MEASURED.
14. **Everything is synthetic and PROVISIONAL**: thresholds, the 12 fixed runtime constants (2 new), the stereo noise
    model, the twin assumption. Not independently reviewed yet: AGENTS.md requires a fresh-context Opus review of this
    repair before any gate is reported above CONDITIONAL.

## CPU cost (`timing_repair_r2.json`, `scratch/timing.py`; one thread, idle host, i7-13700K)

| Source | estimate_ground median / p95 ms | apply_sky_rule incl. veto | edge_enclosure alone | pre-round edge_enclosure, same input |
|---|---|---|---|---|
| oakd_like | 4.53 / 5.83 | 3.20 / 3.94 | 2.67 / 3.16 | 1.33 / 1.55 |
| trained_equal | 4.27 / 4.93 | 3.13 / 3.27 | 2.66 / 2.72 | 1.33 / 1.38 |

Orin NX: NOT_MEASURED.

## Records (all under `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/`)

This report `REPAIR_r2.md`; `r2_tree/` (pre-round snapshot of the 8 outdoor files, `SHA256SUMS`, HEAD, git status,
26-path hashes); `repair_r2/`: `scratch/` (harness, scene catalogue, sweeps, tables, N5 probe, timing, xfail check,
reviewer comparison), `sweep_r2_all.json` (+ `.log`), `sweep_r2_all_prefix_bug.json` (the first sweep, before the
connector bug fix; kept as the bug's evidence), `sweep_r2_n2.json`, `n5_probe.json`, `tables_r2_all.md/.json`,
`report_tables.md`, `timing_repair_r2.json`, `reviewer_probes_copy/` (copies), `reviewer_rerun/` (their outputs and
kernel-fault counts), `reviewer_compare.md/.json`, `review_r2_dir_end.sha256`, `analysis/` (policy comparison script,
JSON, logs, table, checkpoint hashes, kernel faults), `diff/` (unified diffs vs `r2_tree`), `report_parts/` (this
report's sources), `logs_tmp/` (development test runs); `logs/repair_r2_*` (final test logs, kernel-fault counts, git
status, tree hashes). `OUTDOOR_r1.md` (SHA-256 5d1cab45... = its `SHA256SUMS.txt` entry), `REPAIR_r1.md`, `review_r1/`
and `review_r2/` were not edited (no file in `review_r1/` or `review_r2/` is newer than `REVIEW_r2.md` except the
review's own `mce_final.txt` written in the same minute).

## Git traceability

- Branch `camonly/dev`; Base SHA `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`; New commit SHA: none (no commit by
  instruction). Remote push: NO. Main merge: NO.
- Changed files (this repair, all uncommitted): `tools/camera_policy_runtime.py`, `tools/camera_ros2_session.py`,
  `tools/oakd_outdoor_replay.py`, `ros2/motar_camera_policy/README.md`,
  `docs/evidence/sitl_2026-11-05/camera_runtime/README.md`, `tests/test_camera_runtime_outdoor_r1.py`.
- Working tree: 26 uncommitted paths, the same set as before (21 modified tracked + 5 untracked:
  `tests/test_camera_runtime_fixes.py`, `tests/test_camera_runtime_fixes_r2.py`, `tests/test_camera_runtime_outdoor_r1.py`,
  `tests/test_camera_runtime_repair_r1.py`, `tools/oakd_outdoor_replay.py`); `../logs/repair_r2_git_status_after.txt`.
- Tests: Verification above. Raw/checkpoint integrity: checkpoints unchanged (pins verified before/after the policy
  runs), `results/`, `aerial_gym/` and raw data untouched (no status entries); WORKLOG and Beads not edited (no commit in
  this unit). Gateway files byte-equal to `pre_tree`.
