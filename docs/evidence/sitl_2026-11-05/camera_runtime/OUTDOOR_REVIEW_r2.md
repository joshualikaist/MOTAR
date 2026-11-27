# Independent review r2: camera-runtime outdoor profile after repair r1 (2026-11-13)

Reviewer: Claude Code subagent, fresh context, read-only audit. Model: Opus 5.5 (`claude-opus-5-5`).
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`.
Audit directory (the only place written): `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/review_r2/`.
Usage: USAGE_UNKNOWN (no provider-visible signal available to this subagent). The run was interrupted once by a
network error (EAI_AGAIN) and resumed; no step was repeated and no output was lost.

No flight-readiness verdict is given. Every scene below is the implementer's synthetic OAK-D emulation
(`tests/test_camera_runtime_outdoor_r1.py:outdoor_scene`), imported read-only, with its own ASSUMED sensor values
(textureless surfaces read as depth 0; valid 3-5 cm vertical silhouette edges for "twins"). Nothing here is
real-sensor, PX4, ROS-graph, Gazebo or flight evidence.

## Integrity of this review

| Check | Result | Record |
|---|---|---|
| `git status --porcelain --untracked-files=all` before / after | identical, 26 lines, equal to the repair's `logs/repair_r1_git_status_after.txt` | `git_status_before.txt`, `git_status_after.txt` |
| 26 uncommitted paths vs the repair's final hashes | `sha256sum -c logs/repair_r1_final_tree_after_readme_fix.sha256`: all OK | console |
| Reviewed files before / after | identical SHA-256 (26 files) | `reviewed_files_sha256_{before,after}.txt` |
| Gateway files vs pre_tree | `tools/camera_ros2_px4.py` (c487a5ea...) and `tools/camera_ros2_px4_node.py` (9d714391...) byte-equal to `pre_tree/files` | console (cmp) |
| Files differing from pre_tree | exactly 6 (contract, runtime, session, node, two READMEs); all 18 other pre_tree files byte-equal | console (cmp) |
| `aerial_gym/`, `results/` | no status entries | console |
| Checkpoints (read only) | seed 1 `888e8056b2a67683...`, seed 2 `203deea459ef1d93...`, unchanged before/after the policy runs | `checkpoint_sha256_{before,after}.txt` |
| Kernel faults (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`) | 0 at start, 0 before and after every run, 0 at the end | `mce_before.txt`, `mce_after.txt`, `kfaults_*`, `probes/kf_*` |
| Run constraints | `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `python -B`, `taskset -c 0-3,6-23` (cpu4/5 offline), `timeout -k 10` on every run | commands in this file |

## Tests run

- The 15 `tests/test_camera_*` modules (focused module included) in ONE process: `Ran 224 tests in 110.882s`,
  `OK (expected failures=7)` (`camera_modules_one_process.log`). Matches the repair's claim (224, 7).
- Full PUBLIC_CPU not rerun (per instruction). Its log `logs/repair_r1_public_cpu_final.log`: `Ran 4208 tests in
  867.308s`, `OK (skipped=10, expected failures=7)`, kernel faults 0 -> 0. Confirmed (log only).
- The 7 `@unittest.expectedFailure` tests, each run directly (`probes/q9_xfail.py`): all 7 fail by `AssertionError`
  on the SAFE outcome (e.g. `0 != 66`, `0 != 160`, `[0, 0, 0, 0, 0, 0, 0, 0]`), none by an unrelated exception. They
  are not vacuous.

## Per-criterion verdicts

| # | Criterion | Verdict | Measured scope |
|---|---|---|---|
| 1 | Sky depth 0 no longer a near wall (outdoor profile) | **LIMITED_VALIDITY** | Runtime/replay: PASS on synthetic scenes (SkyRule tests pass: 12 tree-line poses, 0 near columns; my policy run reproduces the phantom-free tree line). Limits: (a) the ROS node refuses every outdoor contract (UNCONFIGURED, verified), so the fix is not available on the node; (b) far ground reported as depth 0 is a near band on all 160 bearings (far-ground rule off by design; policy dv 2.87 / 2.32 m/s, reproduced); (c) sky below a ceiling-removed canopy is a 160-bearing phantom; (d) at nose-down pitch the sky is freed only through the unimaged connector, whose exposure is measured under criterion 2. |
| 2 | Depth 0 not a blanket 0.4 m obstacle, and real near obstacles in the altitude slice not missed | **FAIL** | New P0-N1 (one-edge / oblique sky-luma faces defeat the edge veto) and P0-N2 (fully invalid sky-luma objects reaching only 2-5 cm below the camera are lost). Kept on measured scope: trunks, low dark branches, dark invalid rails/bars at camera height (P0-1 fixed), dark and white twins, all both-edge twins up to pitch 20 deg and roll +-10 deg (24/24 configurations), dark objects in every scene I built. |
| 3 | Constraints unchanged: 40x40 m box, 3 m virtual ceiling never licenses deleting/bridging invalid pixels, gateway untouched | **PASS** | Gateway/geofence files byte-equal to pre_tree. `apply_sky_rule` no longer receives the ceiling mask; the flood passes only `candidate | (unimaged & v<0)` (`camera_policy_runtime.py:890-892`); `remove_floor_and_ceiling_plane` changes valid pixels only and returns `si` unchanged (`:742`); ceiling-removed returns are used only as veto EDGES (more protection). Contract: `physical=false` with `invalid_plane_rule=true` refused; outdoor ceiling bounded to [2.5, 6.0] m. Note: the unimaged connector also includes uncovered rectification borders (P1-N3), which is not the ceiling but is a second bridge. |
| 4 | Ground/height from camera + PX4 only; unreliable -> refuse, never pass; new checks; NaN/missing fields; node refuses outdoor; session hysteresis | **LIMITED_VALIDITY** | Fail-closed paths PASS on measured scope (`probes/q5_ground.py`): NaN / +inf / negative depth -> `ground_insufficient_points`; missing z_valid/epv/dead_reckoning -> `estimator_vertical_fields_absent`; epv inf/negative, z_valid None, NaN pos/mount, unknown state key -> InputError (session fault); an unvalidated contract with a NaN planarity threshold refuses (`ground_not_planar`). Node: outdoor contract -> `ValueError: UNCONFIGURED: outdoor_vertical_fields_not_forwarded...`, indoor control LOADED (`probes/q6_session.py`). Session: exactly 5 held accepted frames before the first nonzero command after the last refusal, in 3 scenarios incl. intermittent refusals; reset only on the first accepted frame; `applied_zero_hold` on every held frame. Limits: wrong-surface accepts remain inside the PX4 tolerance (0.12 and 0.25 m platforms accepted at -0.12 / -0.25 m; a far-only patch accepted at -0.134 m / 0.99 deg); flat-ground false refusals grow with stereo noise (below); P3-N7 session edge cases. Real OAK-D noise/vegetation NOT_MEASURED. |
| 5 | Tests non-vacuous (positive controls, non-zero counts, xfail asserts SAFE outcome, no refuse-everything guard) | **LIMITED_VALIDITY** | Mechanics PASS: every negative I inspected has a positive control with a non-zero count; the 7 xfail tests fail by AssertionError on the safe outcome; the must-accept ground sweep accepts 44/44 (no refuse-everything guard). Gaps: no test (pass or xfail) for one-edge, oblique, shallow-below-horizon light-grey objects (P0-N1/N2), the coverage-border connector (P1-N3), the unimaged band with roll, or sub-pixel edge offsets (P2-N4); `ObjectsWithValidEdges.test_rail_twin_against_an_open_horizon` restricts the 2 m case to inner columns 15-144, so the image-border loss is neither asserted nor kept visible as an xfail. |
| 6 | Observation/policy comparison reproducible | **PASS** (synthetic scope) | Independent CPU run (`probes/q8_policy.py`, method functions copied verbatim from the repair script, checkpoints read only), 6 numbers of `REPAIR_r1.md` reproduced exactly: open field far ground invalid seed 1 `160; 2.87 / 73 / 2.09`, seed 2 `160; 2.32 / 60 / 1.58`; white panel pitch +10 seed 1 `0; 2.50 / 62 / 1.35`, seed 2 `0; 1.95 / 49 / 0.95`; light-grey twin 1.5 m seed 1 `82; 0.09 / 3 / 0.12`, seed 2 `82; 0.09 / 3 / 0.10`. |
| 7 | Opt-in only: default/indoor byte-identical (two processes); raw/checkpoints/results/aerial_gym untouched; thresholds PROVISIONAL | **PASS** (runtime and session scope) | Two fresh processes, pre_tree modules vs current modules (`probes/q7_make.py`, `q7_eval.py pre|new`): 16 scenes x 7 indoor contracts; 112 preprocess, 112 projection, 112 floor/ceiling, 112 height_band, 224 extra-state, 7 indoor ROS-session sequences (9 ticks incl. altitude gating): **all 679 identical** (incl. 273 identical exceptions). Contract layer: 13 variants; accept/refuse verdicts identical; 6 refusal MESSAGES differ, all on inputs refused before and after (P3-N9). Contract thresholds are `{"value", "status": "PROVISIONAL"}`; the 10 fixed runtime constants carry `# PROVISIONAL`. READMEs: 0 lines removed vs pre_tree (167 and 51 added). |

Other checks:
- **PUBLIC_CPU single-process pollution: PASS** on the 15-module one-process run. The new module adds no
  `sys.modules` stubs and no `os.environ` writes (only `sys.path` inserts of `tools/` and `tests/`, as the other camera
  modules); its `mock.patch.object` uses are context-scoped; `oakd_outdoor_replay.geometry_runtime` executes the
  depth-validity file without registering it; the actual-policy class runs in a subprocess with a filtered
  environment. The pre-existing helper modules it imports (`test_camera_runtime_fixes(_r2)`) register private names,
  unchanged.
- **Honesty of REPAIR_r1.md labels:** mostly honest; two PARTIAL descriptions overstate what is fixed (see the table
  below and P0-N1/N2), and the residual-risk list omits the one-edge/oblique/coverage-border classes.

## Status of the round-1 findings

| Finding | REPAIR label | Reviewer status | Evidence |
|---|---|---|---|
| P0-1 far-ground band frees dark rails | FIXED | **Confirmed fixed** (rule off by default) | `ReviewerScenesFixed.test_p0_1...` passes in my run (h 1/1.5, pitch 0/+-5, D 1.5/2/3, two bands, 0 far-ground pixels); its opt-in control loses the rail. Cost confirmed: open field with depth-0 far ground = 160 near bearings, dv 2.87 / 2.32 m/s. |
| P0-2a light-grey/white invalid objects freed by rule 1 | PARTIAL | **Partial, and the OPEN part is larger than reported** | Both-edge twins kept (my q1/q1b/q2 controls: 66/66, 48/48, 30/30, 22/22; 24/24 band configurations). But "fixed by the edge veto when the object has valid vertical edges" fails when only one edge is in view or the edges differ in range by > 1 m (P0-N1), and "fixed when the object reaches below the horizon" fails for 2-5 cm below the camera at 2-4 m (P0-N2). |
| P0-2b ceiling-removed pixels bridge the flood | FIXED | **Confirmed fixed** | Code (`:890-892`) and `SkyRule.test_virtual_ceiling_never_connects_the_sky`. A different bridge (uncovered coverage border) is P1-N3. |
| P0-3 bright invalid canopy below 3 m | PARTIAL | **Confirmed partial** | Gazebo twin kept (tests pass); the 10 m-wide fully invalid canopy stays lost (xfail fails by `AssertionError: 0 != 160`). |
| P1-1 wrong-surface ground fits | PARTIAL | **Confirmed partial; residual slightly larger than stated** | Bridging planes refused (tests pass). A 0.25 m platform hiding the ground is accepted at -0.25 m (3 of 4 poses; report names 0.2 m), a 0.12 m one at -0.12 m. A far-only ground patch (h 2.0, pitch -10, ranges 6.9-9.3 m, noise 0.25 px) is accepted at -0.134 m / 0.99 deg tilt. |
| P2-1 ROS node / hold hysteresis | PARTIAL | **Confirmed partial** | Node refusal and hysteresis verified (above). Open: forwarding of PX4 vertical fields; hold time limit (user decision). P3-N7 edge cases. |
| P2-2 invalid thin wires | OPEN | **Confirmed open** | xfail tests fail by AssertionError (`[0]*8`, `[0]*4`). |
| P3-1 vacuous assertion | FIXED | **Confirmed fixed** | The replacement asserts > 50 ranged bearings; all 7 xfails assert the safe outcome. |
| P3-2 epv as string/array | FIXED | **Confirmed fixed** | Tests (string, arrays, list, None, complex, bool) pass; my probe: inf / negative -> InputError, numpy bool z_valid accepted (control). |
| P3-3 permissive contract schema | FIXED | **Confirmed fixed** | Indoor `ceiling.physical` refused; contract-layer verdicts identical to pre_tree for 13 variants. |
| P3-4 runtime constants not PROVISIONAL | FIXED | **Confirmed fixed** | 10 constants marked and listed; test passes. |
| P3-5 SKY_CONNECTIVITY wording | FIXED | **Confirmed fixed (wording)** | The runtime matches `top_border_4_through_sky_or_unimaged`; "unimaged" includes uncovered borders, whose effect was not measured (P1-N3). |
| P3-6 byte-identity test | FIXED | **Confirmed fixed** | Embedded goldens pass; my independent two-process run: 679/679 runtime and session outputs identical. |

## New findings

### P0-N1: the edge-enclosure veto is defeated by one-edge and oblique geometry (missed sky-luma faces in the altitude slice)

`tools/camera_policy_runtime.py:825-833` (`edge_enclosure`): a pixel is enclosed only if the scan finds a near valid
edge on BOTH sides (`both = si & (lo >= 0) & (hi < n)`, the image border is not an edge), both within
`edge_veto_range_m`, and the two edge ranges within `edge_veto_range_tolerance_m` = 1.0 m. A light-grey (luma 0.80,
close to the sky's 0.72-0.86) sensor-invalid face is then a sky candidate (`:888`) and joins the sky flood through
the low-contrast sky.

Failing input (`probes/q1b_open_horizon.py`, `q1b_open_horizon.json`): open horizon (no tree line; far ground ranged,
the emulation's default), h 1.0 (camera 1.03 m), a light-grey sensor-invalid panel from camera +0.03 m to +0.30 m
(lower edge inside the vehicle's own altitude slice), right edge valid at y = -0.3 m and in view, left side extending
out of the field of view (y = +3 m), face covering straight ahead.

| Scene (pitch 0) | D 1.5 m | 2 m | 3 m | 4 m |
|---|---|---|---|---|
| light grey, one edge in view: object columns kept (< 10 m) | **0/98** | **0/92** | **0/87** | **0/84** |
| same, lower edge 0.02 m BELOW the camera | **0/98** | **0/92** | **0/87** | **0/84** |
| same, lower edge 0.05 m below the camera | 98/98 | **0/92** | **0/87** | **0/84** |
| CONTROL light grey 1 m wide, both edges in view | 66/66 | 48/48 | 30/30 | 22/22 |
| CONTROL dark twin, one edge in view | 98/98 | 92/92 | 87/87 | 84/84 |

At pitch +-5 deg or roll 5 deg the one-edge face keeps only 1-48 of 84-98 columns and 0-9 of the 14-16 columns
straight ahead. With a 5 m tree line behind, the panel is kept when its top is below the tree line, and lost when it
rises above it (`q1_one_edge.json`: top camera +0.55 m at 2 m, 0/92 at all 4 poses; control 48/48). A white face
with one edge is kept at pitch 0 by its own luma edge against the sky, but lost at pitch 10 deg when it touches the
unimaged band (0/98 at 1.5 m, 0/92 at 2 m) and with a coverage border (P1-N3).

Oblique faces (`probes/q2_veto_geometry.py`, group a): a light-grey face, both silhouette edges valid and in view,
lower edge camera +0.03 m, from (2.0, -0.6) to (3.6, +0.8) m (edge ranges differ by 1.60 m): **0/63** columns.
Controls: the same face at 0.69 m and 0.54 m range difference 84/84 and 76/76; the dark twin 63/63.

Policy (`probes/q8_policy.py`, seed 1 / seed 2, 30 frames, the repair's method): one-edge light-grey panel at 2 m,
open horizon: outdoor profile 0 near bearings, command differs from the training-equivalent command by **2.53 / 2.07
m/s**, direction **63 / 53 deg**, yaw **1.42 / 1.18 rad/s** (the vehicle proceeds as in an open field). With the
tree line behind (panel top below it): 98 near, dv 0.35 / 0.31.

Plausibility: the same object class as r1's P0-2a (light-grey or silver textureless faces: trailer or truck body, sign
boards, barrier arms, awnings), seen partly out of view while approaching or turning, or at an angle. The repair's
assumption that both vertical silhouette edges are valid and in view is needed for every protected case.

Suggested fix (options; the first two are code, the third is a user trade-off):
1. Treat "unknown" stops as edges for the veto: when one side's stop is a near valid edge and the other side's run
   reaches the image border, an unimaged/uncovered pixel or a far valid return, keep the run enclosed (bounded by
   `edge_veto_span_deg`). Measure the extra phantom cost (sky beside a single trunk).
2. Scale the range tolerance with geometry (an oblique face's edges legitimately differ by up to width x sin(yaw)), or
   enclose with the nearer edge only.
3. Do not free sky candidates in a low-elevation band above the horizon where near obstacles in the altitude slice
   project, unless positively identified as sky (RGB/temporal/OAK-D confidence evidence). This re-creates a horizon
   phantom in open fields; it is the user's trade-off.
4. Add negative tests (pass or xfail) for one-edge, oblique and laterally offset light-grey faces at 1.5-4 m,
   pitch +-10, roll +-10, open horizon and tree line.

### P0-N2: fully invalid sky-luma objects that reach only slightly below the horizon are lost (the OPEN class is larger than stated)

`camera_policy_runtime.py:888` frees candidates above the horizon; the rows below the horizon stay sensor-invalid,
but `blob_near` (5x5 window, density 0.5) needs about 3 invalid trained rows to mark them near. Diagnosed in
`probes/q1c_debug.log`: a light-grey panel at 2 m reaching 0.05 m below the camera leaves only rows 45-46 invalid in
its columns after the sky rule, so no near pixel results; at 0.10 m below (rows 45-49) it is kept.

| Fully invalid light-grey panel 1 m wide, top camera +0.30 m, open horizon | 1.5 m | 2 m | 3 m | 4 m |
|---|---|---|---|---|
| lower edge camera -0.02 m | **0/66** | **0/48** | **0/30** | **0/22** |
| lower edge camera -0.05 m | 66/66 | **0/48** | **0/30** | **0/22** |
| CONTROL dark, same geometry | 66/66 | 48/48 | 30/30 | 22/22 |

Policy (3 m, -0.05 m): 0 near bearings; dv 1.43 / 1.19 m/s against the training-equivalent command (seeds 1 / 2).

This refutes two statements: REPAIR_r1.md P0-2a "Fixed when the object reaches below the horizon" (true for its
-0.10 m / 1.5 m geometry only) and `ros2/motar_camera_policy/README.md:281` "invalid structures at or just below the
horizon ... become near". The OPEN class is "fully invalid, sky-like, with fewer than about 3 invalid trained rows
below the horizon", i.e. a lower edge less than about 2.5 / fy x D below the camera (0.05 m at 2.5 m, 0.08 m at 4 m),
not "wholly above the horizon".

Suggested fix: state the class correctly in README/REPAIR; add the xfail tests; consider keeping the invalid run of
a column that continues below the horizon (a component that crosses the horizon is an object, not sky) instead of
freeing its upper part.

### P1-N3: uncovered rectification borders act as sky-flood connectors along the image sides

`camera_policy_runtime.py:272-273` (level path) and `:320-325` (tilted path): every trained pixel whose source
coverage is False counts as unimaged, and `apply_sky_rule` connects through any unimaged pixel above the horizon
(`:891`). The repair restricted the connector to rays outside the source FOV "(or its uncovered border)" but measured
only the top band of a nose-down camera.

Failing input (`probes/q2_veto_geometry.py`, group f): a white one-edge panel at 2 m (camera +0.03..+0.30 m, 5 m tree
line), coverage False on a 30-pixel source border, roll 5 deg: **0/92** columns, against 92/92 without the border.
Light grey: 0/92 vs 92/92. Dark: 92/92 both. With a 12-pixel border no loss in these cases (connectors only in the
top row and top corners). The real OAK-D rectification border width is NOT_MEASURED.

Suggested fix: count as connectors only pixels whose ray lies above the source camera's top FOV edge (the band a
nose-down camera does not image), not uncovered side or bottom borders; add a test with a coverage border and roll.

### P2-N4: veto margins depend on sub-pixel alignment and on the 5 m edge range

- Edge resolution (`q2_veto_geometry.py`, group c): a 1 m light-grey twin with 3 cm valid edges at 4.5 m, swept
  laterally over one trained pixel in 5 mm steps: level path 2/9 offsets lose the face (0/19, 0/18); tilted path
  (pitch 3) 4/9 offsets keep only 0-6 of 18-19 columns. At 3 m all 18 offsets are kept. The repair's "29/30 twins
  fully kept" rests on one alignment per twin.
- Edge range (group b): a 4.4 m-wide light-grey face at 4.8 m whose edges lie at 5.19 m ray range: 0/106 columns;
  at 4.6 m (edges 4.92 m) 100/100. Beyond the 3.2 m stop distance, but the governor releases at 5 m.
- Fix: dilate the near-edge map by one trained pixel before the scan, or accept an edge found within 1-2 pixels; test
  sub-pixel offsets at 3-5 m.

### P2-N5: unimaged-band exposure with roll reaches close to the horizon at the image sides

`q2_veto_geometry.py`, group e (open field, no objects): lowest elevation of a connector pixel above the horizon,
against the trained image top at 26.74 deg.

| pitch \ roll | 0 | 5 | 10 |
|---|---|---|---|
| 0 | none | right 25.2 | left 17.9 |
| 5 | centre 23.0 | left 14.9 | left 9.95 |
| 10 | centre 17.9, sides 16.7 | left 6.8 | left 3.6 |
| 15 | centre 13.1, sides 7.4 | left 0.97 | left 0.32 |
| 20 | centre 8.1, sides 1.6 | left 0.32 | left 0.32 |

At pitch 10-15 deg with 5-10 deg roll (an accelerating turn) a fully invalid sky-like object touching the side band
within 1-4 deg of the horizon is exposed (the image-top case needs 26.7 deg). Objects with both edges valid are kept
in every band configuration I tested (24/24: white and light grey, 1.5/2 m, pitch 10-20, roll -10/0/+10). Within the
already-OPEN fully-invalid class; the repair measured roll 0 only. Fix: record the exposure in README; add roll to the
connector tests.

### P2-N6: ground-estimate margins against noise and far-only patches

`probes/q5_ground.py` (flat textured ground + tree line, h 1.0/1.5/2.0, pitch -10/0/10/15/20, roll 0/10, 2 seeds):

| Disparity noise sigma | flat-ground frames refused | reasons | max accepted planarity (threshold 1.0 deg) | max abs height error accepted |
|---|---|---|---|---|
| 0.15 px (implementer's value) | 1/60 | ground_not_planar | 0.84 deg | 0.013 m |
| 0.25 px | 1/60 | ground_not_planar | 0.91 deg | 0.134 m |
| 0.35 px | 7/60 | ground_not_planar | 0.99 deg | 0.069 m |
| 0.50 px | 24/60 | not_lowest_surface 18, not_planar 4, tilt 2 | 0.99 deg | 0.024 m |

Refusals concentrate at h 2.0 and pitch -10 (ground seen only at 6-10 m). The 0.134 m case is accepted with 0.99 deg
tilt from a 6.9-9.3 m patch (`min_range_span_m` 1.0 passes with 2.4 m). The repair's "max planarity 0.47 deg" holds
for its narrower sweep (h 1.0/1.5). Other scenes: a 1.2 m corridor between walls, a 2 m hedge corridor at h 1.5,
and a 0.15-0.17 m kerb are refused (hold, safe direction); a 0.12 m and a 0.25 m platform hiding the ground are
accepted at -0.12 / -0.25 m. Fix: require a minimum near-range support (e.g. inliers within 5 m) or scale the
planarity threshold with range noise; record the availability cost; name the 0.25 m residual.

### P3 items

- **P3-N7** `tools/camera_ros2_session.py:281` / `:76`: (a) an altitude-gated interval in the middle of re-entry keeps
  the remaining count (3) while the next in-band frame restarts the history, so a nonzero command follows after 3
  fresh-history frames instead of 5 (`q6_session.json`, S3); (b) an explicit `reset()` clears the count, so the
  first accepted frame after reset commands immediately (S4). Both may be intended; document or restart the count.
- **P3-N8** `tools/camera_policy_runtime.py:1206` (`replay`) and `tools/oakd_outdoor_replay.py` do not apply the
  re-entry hysteresis: offline proposals after a refusal differ from what the ROS session would propose.
- **P3-N9** Contract layer: 6 refusal messages differ from pre_tree for inputs refused before and after (e.g.
  `ceiling.physical` on an indoor contract, `profile` without `sky`/`ground_estimate`); verdicts identical. The node's
  diagnostics JSON now carries `refusal`, `consecutive_refusals`, `reentry_hold_remaining` for indoor contracts too
  (`tools/camera_ros2_node.py:194`): not byte-identical on the diagnostics topic (additive).
- **P3-N10** README wording: `ros2/motar_camera_policy/README.md:268` mentions "The image border is not an edge" only
  as a phantom-cost note; `:281` overstates what stays near (P0-N2).

## Claims confirmed or refuted

| Claim (REPAIR_r1.md) | Reviewer measurement | Status |
|---|---|---|
| 15 camera modules in one process: `Ran 224`, `OK (expected failures=7)` | `Ran 224 tests in 110.882s`, `OK (expected failures=7)` | Confirmed |
| PUBLIC_CPU `Ran 4208 ... OK (skipped=10, expected failures=7)` | Log lines present (not rerun) | Confirmed (log) |
| Expected failures assert the SAFE outcome | 7/7 fail by AssertionError | Confirmed |
| Gateway files and pre_tree unchanged | cmp equal | Confirmed |
| Indoor/default byte identity (goldens) | 679/679 runtime and session outputs identical in two fresh processes; 6 contract messages differ on refused inputs | Confirmed (runtime/session); messages differ (P3-N9) |
| Policy table rows (3 scenes x 2 seeds) | All 6 rows reproduced to the printed precision | Confirmed |
| P0-2a "fixed by the edge veto when the object has valid vertical edges" | One edge in view: 0/92 (2 m), 0/87 (3 m); oblique 1.6 m range difference: 0/63 | Refuted outside the both-edges-in-view geometry |
| P0-2a "fixed when the object reaches below the horizon" | -0.05 m: 0/48, 0/30, 0/22 at 2/3/4 m; -0.10 m at 1.5 m kept | Refuted for shallow reach |
| Twins "29/30 fully kept" | Sub-pixel sweep at 4.5 m: 2/9 (level) and 4/9 (tilted) offsets lose most of the face | Alignment-dependent |
| Unimaged connector frees no object with a valid edge | Both-edge twins kept in 24/24 band configurations incl. roll +-10 | Confirmed for both-edge twins; one-edge faces lost at pitch 10 (0/92) |
| Ground must-accept sweep accepted, max planarity 0.47 deg | Repaired sweep passes; wider sweep (h 2.0, pitch 15/20, roll 10): 1/60 refused, max 0.84 deg | Confirmed on its scope; margin smaller outside it |
| Raised platform residual "0.2 m ... accepted at -0.20 m" | 0.25 m accepted at -0.25 m (3/4 poses), 0.12 m at -0.12 m | Confirmed; residual extends to 0.25 m |
| Node refuses outdoor contract (UNCONFIGURED) | Reproduced; indoor control loads | Confirmed |
| Re-entry hysteresis 5 frames, refusal restarts count | 5 held frames in S1/S2/S3; restart on refusal | Confirmed (P3-N7 edge cases) |
| Checkpoint pins | Unchanged before/after | Confirmed |

## Reproduction

Scripts and outputs are in `review_r2/probes/`; each imports the worktree read-only and writes only into that folder.
Run from that folder with `env CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c
0-3,6-23 timeout -k 10 <s> /home/fair/miniconda3/envs/aerialgym/bin/python -B <script>`:
`q1_one_edge.py`, `q1b_open_horizon.py`, `q1c_debug.py`, `q2_veto_geometry.py`, `q5_ground.py`, `q6_session.py`,
`q7_make.py` then `q7_eval.py pre` and `q7_eval.py new` (outputs `q7_data/digests_{pre,new}.json`), `q8_policy.py 1`
and `q8_policy.py 2`, `q9_xfail.py`. Kernel fault counts per run: `probes/kf_*`.

## Summary

The repair fixes what it targeted for the tested geometry: the far-ground rule is off (dark rails at the flight
altitude are near again), the virtual ceiling no longer bridges the sky flood, wrong-surface ground fits are mostly
refused, the ROS node refuses outdoor contracts, the session re-entry hysteresis works, the indoor/default path is
byte-identical in two processes, and the policy numbers reproduce exactly.

Criterion 2 still fails. The edge-enclosure veto protects a bright face only when BOTH vertical edges are valid, in
view, within 5 m and within 1 m of each other in range. A light-grey sensor-invalid face in the vehicle's altitude
slice with one edge out of view, or seen obliquely, is freed as sky (0/84-98 columns at 1.5-4 m; policy dv 2.53 /
2.07 m/s, as in open field). Fully invalid light-grey objects reaching only 2-5 cm below the camera are lost too, so
the OPEN class is larger than REPAIR_r1.md and the README state. Uncovered rectification borders add a side connector.
These findings are open.
