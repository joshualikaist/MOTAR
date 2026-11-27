# Independent review r3: camera-runtime outdoor profile after repair r2 (2026-11-15)

Reviewer: Claude Code subagent, fresh context, read-only audit. Model: Opus 5.5 (`claude-opus-5-5`).
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`.
Audit directory (the only place written): `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/review_r3/`.
Usage: USAGE_UNKNOWN (no provider-visible signal available to this subagent).

No flight-readiness verdict is given. Every scene is the implementer's SYNTHETIC OAK-D emulation
(`tests/test_camera_runtime_outdoor_r1.py:outdoor_scene`, imported read-only) with its ASSUMED sensor values
(textureless faces read depth 0; 3-5 cm valid vertical silhouette edges; 0.15 px disparity noise). Nothing here is
real-sensor, PX4, ROS-graph, Gazebo or flight evidence. Where I write "lost", the object's columns have no column
minimum below 10 m in the production preprocess output (the implementer's `kept_columns` metric).

## Integrity of this review

| Check | Result | Record |
|---|---|---|
| `git status --short` before / after | identical, 26 entries (21 M, 5 ??) | `git_status_before.txt`, `git_status_after.txt` |
| SHA-256 of the 26 reviewed paths before / after | identical | `reviewed_files_sha256_before.txt`, `_after.txt` |
| Current tree vs repair's final hashes | `logs/repair_r2_tree_final.sha256` equals the current 26 hashes | console |
| Gateway `tools/camera_ros2_px4.py`, `tools/camera_ros2_px4_node.py` | byte-equal to `pre_tree/files` (cmp) | console |
| Files in the worktree newer than the review start | 0 (no `__pycache__`, no outputs) | console (`find -newer`) |
| Checkpoints (read only) | seed 1 `888e8056...`, seed 2 `203deea4...`; equal before/after and equal to the repair's pins | `probes/checkpoint_sha256_{before,after}.txt` |
| Kernel faults (`journalctl -k -b 0 \| grep -cE "Machine check\|segfault\|general protection"`) | 0 at start, 0 before/after every run, 0 at the end | `journal_count_{before,after}.txt`, `kf_*`, `probes/kf_*` |
| Run constraints | `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `python -B`, `taskset -c 0-3,6-23` (cpu4/5 offline), `timeout -k 10` on every run, OMP/MKL 1 thread | commands in `probes/*.log` headers / this file |

## Tests run

- The 15 `tests/test_camera_*` modules (focused module included) in ONE process (from `tests/`): `Ran 247 tests in
  157.046s`, `OK (expected failures=10)` (`camera_modules_one_process.log`). Matches the repair's claim (247, 10).
- Full PUBLIC_CPU not rerun (instruction). `logs/repair_r2_public_cpu.log`: `Ran 4231 tests in 912.953s`, `OK
  (skipped=10, expected failures=10)`, exit 0, kernel faults 0 -> 0, tree hashes equal before/after the run. That run
  predates the README-only edit; the final tree was re-verified with the 15 camera modules + `test_public_docs_consistency`
  (`Ran 254`, OK, 10 expected failures). Confirmed from the logs only.
- All 10 `@unittest.expectedFailure` bodies run directly (`probes/p9_xfail.py`): 10/10 fail by `AssertionError` on the
  safe/correct outcome (`0 != 160`, `0 != 66` x2, `0 != 106`, `0 != 48`, `True is not false`, `[0]*8`, `[0]*4`, two
  `False is not true`); none by another exception.

## Per-criterion verdicts

| # | Criterion | Verdict | Measured scope |
|---|---|---|---|
| 1 | Sky depth 0 not a near wall in open views; phantom cost quantified with the real policies | **LIMITED_VALIDITY** | PASS for open views: 0 near bearings in 120 open-sky / tree-line scenes (pitch 0-20, roll 0/5/10/-10, coverage border 0/12/30 px; `p2_availability`). Phantoms (all reproduce the repair or extend it): any near valid return above the horizon within 5.0 m makes 134-160 of 160 bearings near (single trunk, surface at 1.25 / 2.75 / 4.25 m: 134 / 148 / 152; trunk at the image border 1.5/3 m: 134/145; 5 cm pole 2-4.5 m: 156-159; one valid 3 cm branch tip at 2-4 m, open horizon: 160; small valid sign on a post at 3 m: 158); near sky speckle: 0 bearings at 1e-5/1e-4 of sky pixels, **160 at 3e-4 and 1e-3** (far speckle >5 m: 0); far ground at depth 0: 160; clouds 67; sun at the horizon 8. With a tree line behind, objects whose above-horizon part lies in front of it add 0. Real policies (seeds 1/2): every measured phantom changes the command by 1.44-2.87 m/s against the training-equivalent command, and equals the indoor-default (wall) command exactly (pole 3 m and speckle: dv 2.87 / 2.31; trunk 1.5 m: 1.88 / 1.44; trunk 3 m: 2.20 / 1.51). The node still refuses every outdoor contract (UNCONFIGURED), so none of this is available on the ROS node. |
| 2 | Real near obstacles in the altitude slice not missed; try to defeat the one-edge rule, incidence rule, connector | **FAIL** | New R3-P0-1 (no valid edge IN VIEW: wide faces and rails wider than the view, tall bright faces filling the image top; pre-existing since r1, absent from the residual list) and R3-P1-1 (faces seen only through a gap between two near objects at different ranges). Kept on measured scope: the review-r2 one-edge face (r2_tree 0/94 -> 94/94), one-edge faces at 1.5-3 m in 11 poses (pitch -10..20, roll -10..10) except columns the camera does not image, grazing oblique faces at 72.8-77.9 deg incidence (10/10 light grey with 4 and 10 cm end posts; dark controls 10/10), tall grazing faces (3/3, two above 70 deg), faces between trunks whenever the sky between them is a phantom (6/24), all dark objects, white/mid-grey faces whose top boundary meets visible sky, bars/rails with a valid end post in view (9/9). Residual (documented mechanism, broader than reported): tilted path at 4 m (R3-P2-1). The restricted connector was not defeated beyond physically unimaged columns. |
| 3 | Constraints unchanged: 40x40 m box, 3 m virtual ceiling never licenses deleting/bridging invalid pixels, gateway untouched | **PASS** | Gateway files byte-equal to pre_tree (geofence untouched). `remove_floor_and_ceiling_plane` changes valid returns only and returns `si` unchanged (`tools/camera_policy_runtime.py:776-778`); connectors are `unimaged & (v < 0)` from the projection only (`:952`), never ceiling-removed pixels; ceiling-removed returns stay edge stops (`:1182-1183`, more protection). Contract ceiling bounded to [2.5, 6] m (1.2 and 6.5 refused, `p4_session`). `SkyRule.test_virtual_ceiling_never_connects_the_sky` passes. |
| 4 | Ground/height from camera + PX4 only, unreliable -> refuse; node UNCONFIGURED; re-entry hold restarts on every history reset; replays apply the same hold | **LIMITED_VALIDITY** | Session/node/replay PASS (`p4_session`): 5 held frames before the first command after session start, explicit reset, a refusal, a refusal on the first frame, an altitude-gated stretch in the middle of a re-entry, a fault + reset (S1-S9); `applied_zero_hold` on every held frame; intermittent refusals every 3rd frame hold indefinitely (documented, no time limit). Indoor sessions/replays: 0 holds, statuses unchanged. NPZ replay: `RHHHHHCCC`, `HHRHHHHHCRHHHHHCCC`, `HHHHHCCC`, previous action [0,0,0,0] on held lines. Node: outdoor (ceiling 3.0 and 2.5) -> `UNCONFIGURED: outdoor_vertical_fields_not_forwarded`, check exit 2 `NOT_CONFIGURED`; indoor control LOADED / `CONFIG_VALID`. Ground: unchanged this round; the wrong-surface acceptance (platform hiding all ground accepted within the PX4 tolerance) and noise refusals stay as expected failures (confirmed failing by AssertionError); not re-swept. |
| 5 | Tests non-vacuous; xfail assert the safe outcome; OAK-D Pro presets vs Luxonis docs | **LIMITED_VALIDITY** | Mechanics PASS: new negatives have positive controls with non-zero counts (`lost_without_veto > 300`, oblique no-veto control, `full.sum() > 0`, indoor session/replay controls, `test_face_with_edges_inside_the_edge_range_is_kept`); 10/10 xfail fail by AssertionError; no refuse-everything guard. Presets: Luxonis product page (fetched 2026-11-15) confirms OV9282 89.5/80/55 deg, IMX378 AF 78/66/54, FF 82/69/55, 7.5 cm baseline, projector/flood up to 1 W, BNO086, ideal 0.8-12 m; the stereo-depth page confirms the 0-95 px search (extended 0-190) and depth 0 = invalid. `oakd_like` (1014x760, 69 deg HFOV, 54.5 deg VFOV) matches the FF variant; the AF 4:3 inconsistency (51.9 vs 54 deg) is correctly stated. Gaps: no test (pass or xfail) for R3-P0-1 / R3-P1-1 / light-grey rails at camera height; a knife-edge control (R3-P3-2); `OakdProPresets` asserts that the AF variant is accepted on the tilted path (R3-P3-1); README omits the documented MinZ per stereo mode (R3-P2-2). |
| 6 | Reproducibility: >= 3 REPAIR_r2 numbers incl. one phantom-cost number and one policy dv | **PASS** | Reproduced exactly (own CPU runs, checkpoints read only): single trunk centre with tree line D 1.5/3/4.5/6 = 134/148/152/0 (sum 434, max 152, as the table); trunk at the image border 134/134/145/145 (per-scene equal to `sweep_r2_all.json`); thin post 157/160 (sum 317); sky speckle 160 x 4 (sum 640); policy (method copied verbatim): trunk 1.5 m 138 near, dv 1.877 / 1.441 (1.88 / 1.44), single trunk 3 m 148, dv 2.199 / 1.508 (2.20 / 1.51), face 4.8 m (edges 5.19 m) 0 near, dv 2.595 / 2.106 (2.59 / 2.11); connector availability 0 near in 120 scenes; camera modules 247 / 10 xfail; `edge_enclosure` median 2.64 ms (claim 2.67; informational, host not strictly idle). |
| 7 | Opt-in: default/indoor byte-identical to pre_tree (two fresh processes); raw/checkpoints/results/aerial_gym untouched; thresholds PROVISIONAL; README numbers sourced | **PASS** | Two fresh processes (`p7_make`, `p7_eval pre`, `p7_eval new`): 19 scenes x 7 indoor contracts; 133 preprocess, 133 projection, 133 floor/ceiling, 133 height_band, 266 extra-state, 7 contract-layer variants (validate + height ref), 7 indoor ROS-session runs (14 ticks incl. altitude gating + explicit reset), 7 indoor NPZ replays: **826/826 identical, digest files byte-identical** (`a0b574bd...`), incl. 324 identical exceptions. Checkpoints unchanged; no status entry or new file under `results/`, `aerial_gym/`. 12 runtime constants listed PROVISIONAL; contract thresholds `{"value", "status": "PROVISIONAL"}`. `OutdoorRecords.test_repair_r2_numbers_match_their_sources` passes; I found no new README number without a source (capture-protocol parameters are not measurements). |

Other checks:
- **PUBLIC_CPU single-process pollution: PASS** (15 modules in one process; repair's PUBLIC_CPU log). The focused module adds
  no `sys.modules`/`os.environ` writes; `mock.patch.object` uses are context-scoped; the real-policy class runs in a
  subprocess with a filtered environment; the new `oakd_pro_af` preset is a local function (the repair's scratch
  monkeypatch of `F.source_intrinsics` is not in the test module).
- **REPAIR_r2.md labels:** mostly honest (OPEN/NOT_ADDRESSED kept as such; costs stated). Two understatements: "1
  residual geometry" for one-edge faces (R3-P2-1), and the residual list, introduced as "complete as far as known",
  lacks the no-edge-in-view class (R3-P0-1), the gap class (R3-P1-1), the single-sample speckle trigger, the MinZ per
  stereo mode (R3-P2-2) and the AF tilted-path acceptance (R3-P3-1).

## Status of the earlier findings

| Finding | REPAIR_r2 label | Reviewer status | Evidence |
|---|---|---|---|
| r2 P0-N1 one edge in view | FIXED (1 residual) | **Fixed for one edge in view; partial overall** | Review's scene: r2_tree 0/94 -> current 94/94 (`p6_extra`). 44 pose cases 1.5-4 m: 1.5-3 m full except 1-5 image-border columns not imaged at pitch 15-20 / roll 10; 4 m tilted path lost at several pitches (R3-P2-1). The no-edge-in-view class is not covered (R3-P0-1). |
| r2 P0-N1 oblique | FIXED | **Fixed (via one-edge rows)** | Grazing faces at 72.8-77.9 deg incidence (above the 70 deg constant) kept 10/10 (light grey; dark 10/10); tall grazing faces 3/3. Rows where only the nearer end exists are one-edge enclosed and cut the flood, so the incidence rule never decided a scene (agrees with REPAIR residual 5). |
| r2 P0-N2 shallow reach | OPEN | **Open (confirmed)** | xfail `0 != 48`; light-grey rails centred on the camera height are members (R3-P0-1 table). |
| r2 P1-N3 coverage-border connectors | FIXED | **Fixed** | 0 near in 120 availability scenes; one-edge faces with a 30 px border at pitch 10 / roll 10 lose only columns 2-5 (inside the uncovered border). |
| r2 P2-N4 sub-pixel (level path) | FIXED | **Fixed on the level path; tilted-path residual broader (R3-P2-1)** | |
| r2 P2-N4 edge range | OPEN | **Open (confirmed)** | 0 near; dv 2.595 / 2.106 m/s. |
| r2 P2-N5 band exposure with roll | PARTIAL | **Not re-measured** (availability side only) | |
| r2 P2-N6 ground vs noise | OPEN | **Open (xfail confirmed)** | |
| r2 P3-N7 re-entry count | FIXED | **Fixed** | S1-S9 above. |
| r2 P3-N8 replays | FIXED | **Fixed** | NPZ replay sequences above; OAK-D replay tool labels (test). |
| r2 P3-N9 contract messages / diagnostics | NOT_ADDRESSED | **Open** (not re-measured) | |
| r2 P3-N10 README wording | FIXED | **Fixed for the row count; the lost class is still described as "no valid edge at all"** (R3-P0-1) | `README.md:301`, `:370` |
| r1 P0-1 far-ground band | FIXED | **Fixed** (tests pass; rule off) | |
| r1 P0-2a bright invalid objects | PARTIAL | **Partial** | |
| r1 P0-2b ceiling bridge | FIXED | **Fixed** | criterion 3 |
| r1 P0-3 bright canopy | OPEN | **Open** (`0 != 160`) | |
| r1 P1-1 wrong-surface ground | PARTIAL | **Partial** (`True is not false`) | |
| r1 P2-1 node / hold | PARTIAL | **Partial** (node refuses; no hold time limit) | |
| r1 P2-2 thin wires | OPEN | **Open** (`[0]*8`, `[0]*4`) | |

## New findings

### R3-P0-1: an invalid face or rail whose valid silhouette edges are not IN VIEW is freed as sky (wide faces, rails, tall bright faces)

`tools/camera_policy_runtime.py:872-873` and `:893`: the one-edge rule needs one near valid stop in the row
(`near_a ^ near_b`); with the image border on both sides nothing encloses the run. `:947-953`: such pixels are sky
candidates when luma >= 0.5 and the gradient against invalid neighbours is <= 0.03, and `_flood_from_top` seeds row 0
with candidates (`:784`). Consequences:
- a sky-luma (light-grey) face wider than the view at its range (both valid edges out of view) is freed whenever its
  top meets the sky; a white or mid-grey one is protected only by the luma step at its top boundary;
- a bright face that fills the image top (no sky above it in the image) is freed at ANY luma >= 0.5 (white, mid grey);
- a light-grey rail/bar across the whole view at camera height is freed above the horizon, and its rows below are
  fewer than blob_near needs.

Not a regression: the pre-round runtime (`r2_tree`) loses the same scenes (`p6_extra`). Not in REPAIR_r2's residual
list and not in the README, which describe the lost class as "sensor-invalid with no valid edge at all" (`README.md:301`,
`:370`) and the one-edge rule as protecting "a face whose other silhouette edge is out of view" (`:276`).

Failing inputs (`probes/p1_geometry_W.log`, `p8_bars.log`; open horizon, camera 1.03 m, pitch 0):

| Scene | 1.5 m | 2 m | 2.5 m | 3 m | 4 m |
|---|---|---|---|---|---|
| light grey, 6 m wide, cam+0.03..+0.30 | **0/156** | **0/156** | **0/156** | **0/156** | **0/156** |
| light grey, 6 m wide, cam-0.05..+0.55 | 156/156 | **0/156** | **0/156** | **0/156** | **0/156** |
| white, 6 m wide, cam+0.03..3.5 m (fills image top) | 156/156 | **0/156** | **0/156** | **0/156** | **0/156** |
| mid grey (luma 0.55), same tall face | 156/156 | **0/156** | **0/156** | **0/156** | **0/156** |
| CONTROL dark, same tall face | 156/156 | 156/156 | 156/156 | 156/156 | 156/156 |
| CONTROL light grey 6 m, tree line behind | 156/156 | 156/156 | 156/156 | 156/156 | 156/156 |
| light-grey rail 0.06 m centred on camera, full width | - | **0/156** | - | **0/156** | **0/156** |
| light-grey rail 0.10 m, full width | - | **0/156** | - | **1/156** | **0/156** |
| CONTROL same rails 1.2 m long, valid 5 cm end posts in view | - | 74/74 | - | 46/46 | 34/34 |

Approach of a 4 m-wide light-grey face (cam+0.03..+0.30): 6 m 0/78, 5 m 0/96 (edges beyond 5.0 m, P2-N4), **4 m 120/120,
3.5 m 138/138** (both edges in view), **3 m, 2.5 m, 2 m, 1.5 m: 0/156** (both edges out of view). The face is visible to
the policy only between 3.5 and 4 m and disappears inside the 3.2 m stop distance.

Real policies (`p3_policy_s{1,2}.json`, method copied verbatim from the repair): 4 m face at 2.5 m: outdoor 0 near,
command equals the open-field command, **dv 2.63 / 2.12 m/s, ddir 66 / 54 deg** against the training-equivalent
command; at 3.0 m: dv 2.56 / 2.04; tall white face at 3 m: dv 2.56 / 2.04. Control: the same face at 3.5 m 160 near, dv
0.38 / 0.36.

Plausibility: the same object class as r1 P0-2a and r2 P0-N1 (light-grey or white textureless faces), in the most
common geometry for large faces at close range: a box-truck body, trailer or container side, an elevated board, a
silver rail or barrier arm across the path, approached head-on. It depends on the emulation's assumptions (face depth
0, luma close to the sky), which the user's OAK-D Pro recordings must test.

Suggested fix (no one-frame fix without a phantom; user trade-off): (1) document the class in the README limitations and
the residual list; add expected-failure tests (4 m face at 3 m, tall white face at 3 m, light-grey rail at camera
height) with their controls; (2) add capture items: frontal light-grey/white faces wider than the view at 2-3 m, a
silver rail across the view at camera height, a face filling the image top; (3) candidate algorithm changes to
measure on recordings: keep pixels enclosed in recent frames near (short world-frame persistence, so a face seen with
edges at 3.5-4 m does not vanish at 3 m), positive sky evidence (OAK-D confidence map, blue ratio, temporal stability)
instead of luma >= 0.5, or no sky candidates in row runs that span border to border within a low-elevation band (this
re-creates a horizon phantom in open fields).

### R3-P1-1: a face seen only through a gap between two near objects at different ranges is freed

`tools/camera_policy_runtime.py:874-884`: two near stops that are not one face (range difference > 1.0 m and chord
incidence > 70 deg) do not enclose (design decision 1 of REPAIR_r2: "two different objects"), and XOR is false, so the
run between two trunks at different ranges is free; a light-grey face behind that gap, whose own edges are hidden by
the trunks, is freed with it. Failing input (`p1_geometry_B1s.log`): trunks (r 0.15 m) with centres at 0.8-2.0 m and 2.2-4.0 m range,
light-grey face cam+0.03..+0.30 m, 0.5 m behind the farther trunk: **18 of 24 configurations lose the face (0 kept)**,
exactly the configurations in which the sky between the trunks is free; 6/24 are kept, where that sky is a phantom.
Example (`p1_geometry_B1.log`): trunks at (x, y) = (1.0, +0.35) and (2.6, -0.45) m, face at x = 3.5 m: 0/38 (white, mid grey, dark: 38/38). Real policy for
that example: 133 near bearings (trunks), dv only 0.13 / 0.16 m/s against the training-equivalent command, because the
nearer trunk dominates. Same one-frame ambiguity as R3-P0-1; not in the residual list. Fix: document; add an xfail test
and a capture item (light-grey board behind two trees at different ranges).

### R3-P2-1: the tilted-path loss of a thin one-edge face at 4 m is broader than "1 residual geometry"

`tools/camera_policy_runtime.py:328` (share rule) removes a 3 cm valid edge (about 0.9 trained pixel at 4 m), so no near
stop remains. REPAIR_r2 reports one residual geometry (4 m, pitch +5: 17/84). Measured (`p1_debug_target.log`,
`p1_geometry_B4.log`, light-grey one-edge face at 4 m, cam+0.03..+0.30): pitch 3 / 5 / 11 / 13 / 15 deg: 14 / 17 / 27 / 11 /
4 of 85 kept; pitch 7 / 9 / 17: 85/85; pitch 20 roll 10: 16/85; at pitch 15 the result depends on the noise seed (4, 85,
4, 16 of 85 for seeds 0-3, and 85/85 when the policy method's target sphere is added). Within 3 m all poses are kept.
Mechanism is listed (residual 3); the scope is not. Fix: state the measured scope; consider building the edge map from
"any valid sample" (`n_valid > 0`, nearest depth) instead of the share-rule classification, for the veto only, and
measure its speckle phantom cost.

### R3-P2-2: README OAK-D Pro table omits the documented MinZ per stereo mode

Luxonis: product page "~20cm (400P + extended disp), ~40cm (400P OR 800P + extended), ~80cm (800P)"; stereo-depth page
about 70 cm standard, about 35 cm extended (7.5 cm baseline, 800P). The contract MinZ is 0.4 m and
`tools/oakd_camera.py:180-181` supports `subpixel` mode without extended disparity. In that mode objects at about
0.4-0.8 m read depth 0 including their silhouette edges, i.e. they join the fully-invalid class that the outdoor sky
rule frees when bright and above the horizon (indoors they are near). `README.md:401` lists only the ideal range and
the search width. Fix: add the MinZ row; require `extended` for the outdoor profile or record MinZ per mode in the
contract; add a capture item (a light board at 0.5 m in each mode).

### P3 items

- **R3-P3-1** AF variant on the tilted path is accepted by the runtime: with `oakd_pro_af` intrinsics the level path
  refuses and `oakd_camera.Rectifier.require_training_fov` refuses, but on the tilted path (any nonzero attitude) the
  uncovered trained rows become no-return without refusal: rows 83-89 at pitch -3, 86-89 at -1 (image bottom, where an
  obstacle 0.4-0.5 m ahead and 0.2 m below the camera projects); rows 0-6 at pitch +3 become connectors (`p6_extra`).
  `OakdProPresets.test_auto_focus_variant_does_not_cover_the_trained_fov` asserts this acceptance (0 near). REPAIR_r2
  ("the existing fail-closed check would stop every frame") holds only for the level path and the oakd_camera
  publisher; another ROS driver feeding an AF unit is not stopped. Fix: refuse in `project_rgbd` when the source FOV at
  zero attitude does not cover the trained FOV, independent of the path, or state the limit.
- **R3-P3-2** `OneEdgePhantomCost.test_sky_beside_a_single_near_object_is_near` (`tests/...:2109`) uses a 4.5 m
  image-border trunk as a no-phantom control on the left side only; the repair's own sweep has the right-side twin at
  144 near bearings (edges just inside 5.0 m). Knife-edge control; move it to 5.5 m or assert both sides.
- **R3-P3-3** Sky speckle trigger is one sampled pixel per row: on the level path near speckle at 3e-4 of sky pixels
  already gives 160 bearings (0 at 1e-4). `tools/oakd_camera.py:172` uses the `HIGH_DENSITY` preset (confidence
  threshold 245 in the Luxonis stereo-depth page's example), which favours low-confidence matches. Already a capture
  item; record the confidence map with the sky captures.
- **R3-P3-4** The PUBLIC_CPU log predates the README-only edit (documented by the repair; camera modules and docs test
  re-run after it).

## Phantom-vs-miss trade-off as measured (synthetic; seeds 1 / 2 where a policy number is given)

| Scene (open horizon unless noted) | Near bearings (of 160) | Object columns kept | Policy dv vs training-equivalent (m/s) |
|---|---|---|---|
| Open sky / tree line, pitch 0-20, roll 0/5/10/-10, borders 0/12/30 (120 scenes) | 0 | - | - |
| Single trunk 1.5 m (surface), tree line | 138 | - | 1.88 / 1.44 (same as indoor wall) |
| Single trunk 3 m, tree line | 148 | - | 2.20 / 1.51 (same as indoor) |
| 5 cm pole 3 m, 4 m tall, tree line | 158 | - | 2.87 / 2.31 (same as indoor) |
| Valid 3 cm branch tip at 2-4 m, cam+0.3/0.6 | 160 (tree line: 0-81) | - | not run |
| Near sky speckle 1e-4 / 3e-4 / 1e-3 of sky pixels | 0 / 160 / 160 | - | 3e-4: 2.87 / 2.31 |
| Far ground at depth 0 | 160 | - | 2.87 / 2.32 (repair) |
| Trunk centre 6 m (edges beyond 5 m) | 0 | - | 0.01 (repair) |
| Review-r2 one-edge light-grey face 2 m | 160 | 94/94 (r2_tree 0/94) | 0.32 / 0.24 (repair) |
| 4 m-wide light-grey face, 3.5 m (edges in view) | 160 | 138/138 | 0.38 / 0.36 |
| Same face, 3.0 m / 2.5 m (edges out of view) | 0 | **0/156** | **2.56 / 2.04; 2.63 / 2.12** |
| Tall white face 6 m wide at 3 m | 0 | **0/156** | **2.56 / 2.04** |
| Light-grey face in a gap, trunks 1.0 / 2.6 m, face 3.5 m | 103 (with target 133) | **0/38** | 0.13 / 0.16 |
| Light-grey rail 0.06 m at camera height, full width, 2-4 m | 0 | **0/156** | not run |
| Same rail, valid end posts in view | 152-156 | 74/74, 46/46, 34/34 | not run |
| One-edge face 4 m, pitch 15 (tilted path) | 4 | **4/85** (seed-dependent) | with target sphere: kept, 1.75 / 1.32 |

Reading: near any valid return within 5 m (trees, poles, branch tips, speckle) the outdoor profile issues the same
command as the indoor default (a wall); in open views it frees the sky; sky-luma or bright faces without a valid edge in
view, or seen through a gap between objects at different ranges, are removed and the policy proceeds as in an open
field.

## Claims confirmed or refuted

| Claim (REPAIR_r2.md) | Reviewer measurement | Status |
|---|---|---|
| 15 camera modules: `Ran 247`, OK, 10 expected failures | `Ran 247 tests in 157.046s`, OK (expected failures=10) | Confirmed |
| PUBLIC_CPU `Ran 4231 ... OK (skipped=10, expected failures=10)` | log lines present | Confirmed (log) |
| 10 xfails fail by AssertionError on the safe outcome | 10/10 | Confirmed |
| Single trunk 1.5-4.5 m: 138-152 near; 4 scenes sum 434 (max 152) | 134/148/152/0 (sum 434); policy scene "trunk 1.5 m" 138 | Confirmed |
| Thin post 157-160; speckle 0.1 % -> 160 | 157/160; 160 x 4 | Confirmed |
| Policy: trunk 1.5 m dv 1.88 / 1.44; trunk 3 m 2.20 / 1.51; face 4.8 m 2.59 / 2.11 | 1.877 / 1.441; 2.199 / 1.508; 2.595 / 2.106 | Confirmed |
| Open-sky availability 0 near in 120 scenes | 0 in 120 | Confirmed |
| Re-entry restarts on every history reset; replays hold | S1-S9; replay sequences | Confirmed |
| Node refuses outdoor contracts | UNCONFIGURED, exit 2; indoor control loads | Confirmed |
| Indoor/default byte identity | 826/826 in two fresh processes | Confirmed |
| P0-N1 "fixed on the review's scenes; 1 residual geometry" | fixed for one edge in view; 4 m tilted residual at 5/8 pitches and seed-dependent | Confirmed for its scenes; residual understated |
| Oblique rule unexercised; one-edge rule keeps oblique faces | grazing faces 72.8-77.9 deg kept 10/10 | Confirmed |
| "Unchanged 0: ... objects beyond 5 m" | trunk 6 m: 0; trunk centred at 5.0 m (edges ~4.9 m): 151-152 | Confirmed (threshold sharp) |
| `edge_enclosure` median 2.67 ms | 2.64 ms | Confirmed (informational) |
| Residual list "complete as far as known" | no-edge-in-view faces/rails (0/156), gap faces (18/24), MinZ per mode, AF tilted acceptance missing | Refuted (incomplete) |
| AF variant: "the existing fail-closed check would stop every frame" | tilted path accepts AF frames (rows 83-89 no-return at pitch -3) | Refuted outside the level path / oakd_camera publisher |
| oakd_like matches the FF variant; AF VFOV inconsistent with 4:3 | docs 69/55 vs preset 69/54.5; AF 66 deg HFOV at 4:3 -> 51.97 deg | Confirmed |

## Reproduction

Scripts and outputs: `review_r3/probes/` (each imports the worktree read-only, writes only there). Run from that folder
with `env CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c 0-3,6-23 timeout -k 10
<s> /home/fair/miniconda3/envs/aerialgym/bin/python -B <script> [arg]`:
`p1_geometry.py A|S|B1|B1s|B1t|B2|B4|B5|W|P|D4` (logs `p1_geometry_<part>.log`, JSON), `p1_debug_target.py`,
`p2_availability.py`, `p3_policy.py 1|2` (`p3_policy_s{1,2}.json`), `p4_session.py`, `p6_extra.py`,
`p7_make.py` then `p7_eval.py pre|new` (`p7_data/digests_{pre,new}.json`), `p8_bars.py`, `p9_xfail.py`.
Shared helpers: `pc.py`. Kernel-fault counts: `probes/kf_*`. Parts B1t and B2 were run twice: the first run had a reviewer bug (a label format error; object columns taken from one strip box only, giving 0/0) and was overwritten by the corrected run reported here; no other output was replaced. `SHA256SUMS` hashes every file of this folder.

## Summary

Repair r2 fixes what it targeted: one-edge-in-view and oblique faces are kept, uncovered side/bottom borders no longer
carry the sky flood, the re-entry hold restarts on every history reset in the session and both replays, the indoor
path is byte-identical in two fresh processes, and the reported numbers reproduce exactly. The cost is as reported:
any near valid return above the horizon within 5 m (trunk, pole, branch tip, one near speckle sample per row) turns
134-160 bearings into a near wall and the real policies then issue the indoor wall command (dv 1.4-2.9 m/s).

Criterion 2 still fails. The protection needs a valid silhouette edge in view in the face's rows: light-grey faces
and rails wider than the view (0/156 at 1.5-4 m; a 4 m face is seen only at 3.5-4 m during an approach), bright faces
filling the image top (white and mid grey too), and faces seen through a gap between objects at different ranges
(18/24) are freed, and the policies then command as in an open field (dv 2.0-2.6 m/s). This class is pre-existing
since r1 but missing from REPAIR_r2's residual list and the README; it is not clearly fixable in one frame and is a
user trade-off to settle with the OAK-D Pro recordings.
