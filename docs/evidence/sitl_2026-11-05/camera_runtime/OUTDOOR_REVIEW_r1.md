# Independent review r1: camera-runtime outdoor profile (2026-11-13)

Reviewer: Claude Code subagent, fresh context, read-only audit. Model: Opus 5.5 (`claude-opus-5-5`).
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `fc8aa4d9d06f87b4115b1628f5ff0cc77a30adde`.
Audit directory (the only place written): `/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/review_r1/`.
Usage: USAGE_UNKNOWN (no provider-visible signal available to this subagent).

Integrity of this review: `git status --porcelain --untracked-files=all` identical before and after (26 lines,
`git_status_before_review.txt` / `git_status_after_review.txt`); the 8 reviewed files keep their SHA-256
(`reviewed_files_sha256.txt`). CPU only (`CUDA_VISIBLE_DEVICES=""`, `taskset -c 0-3,6-23`, `python -B`,
`PYTHONNOUSERSITE=1`, `timeout -k 10`). Kernel fault count 0 before and 0 after every run (`kfaults_*.txt`).
Checkpoints read only (seed 1 loaded once through `CameraPolicyRuntime`); both SHA-256 equal their pins
(`888e8056b2a67683…`, `203deea459ef1d93…`). No flight-readiness verdict is given; only whether the code meets the
seven criteria.

All scenes are the implementer's synthetic OAK-D emulation (`tests/test_camera_runtime_outdoor_r1.py:outdoor_scene`),
imported read-only by the probes in `probes/`. Its "sensor-invalid" assumptions (textureless / thin / dark surfaces
read as depth 0) are the implementer's own and are reused unchanged. Nothing here is real-sensor evidence.

## What I verified first

- Exactly six files differ from `pre_tree/` (sha256 -c: 6 FAILED, 18 OK). The `diff/` folder matches my own diffs.
  `tools/camera_ros2_px4.py` and `tools/camera_ros2_px4_node.py` match pre_tree byte for byte (c487a5ea…, 9d714391…),
  and so does `ros2/motar_camera_policy/config/px4_unconfigured.json`. Nothing under `aerial_gym/` or `results/` changed.
- Focused module: `Ran 37 tests in 36.5s, OK` (`focused_outdoor_r1.log`).
- All 13 `tests/test_camera_*.py` modules in one process: `Ran 198 tests in 65.0s, OK` (`camera_modules_one_process.log`).
- PUBLIC_CPU run 3 log (`logs/public_cpu_outdoor_r1_run3.log`): `Ran 4182 tests in 825.093s / OK (skipped=10)`.
  I read the log but did not rerun the full suite.

## Per-criterion verdicts

| # | Criterion | Verdict | Measured scope |
|---|---|---|---|
| 1 | Sky / far-ground depth 0 becomes no-return under the outdoor profile | **LIMITED_VALIDITY** | Runtime and replay: PASS on synthetic scenes (SkyRule tests: 12 tree-line poses and 12 open-field variants, 0 near columns after; my seed-1 policy run of the tree-line scene matches the training-equivalent command, dv 0.001 m/s). ROS node path: the fix never takes effect, because the adapter state has no `z_valid`/`epv`/`dead_reckoning` fields. Every outdoor frame is refused (`estimator_vertical_fields_absent`), so the vehicle holds. |
| 2 | Depth 0 no longer means a blanket 0.4 m obstacle, and real near obstacles are not missed | **FAIL** | P0-1, P0-2, P0-3 below. Trunks, posts standing on the ground, dark canopies and branches closer than MinZ are kept, as the report says. Missed: invalid horizontal structures at the drone's own altitude; bright or light-grey objects whose lower edge sits at or just below camera height; bright overhead obstacles below 3 m; invalid thin wires (pre-existing). |
| 3 | Outdoor constraints unchanged | **FAIL** (one sub-item) | Gateway, geofence and wind/braking files: byte-identical (PASS). The default 3.0 m virtual ceiling has `physical:false` and refuses `invalid_plane_rule` (PASS). But the ceiling-removed pixels are used as sky-flood connectors (`camera_policy_runtime.py:745`, `:962`), and that frees a NEAR invalid object (P0-2b). That is exactly the "license to delete invalid pixels" the criterion forbids. |
| 4 | Ground/height discrimination is fail-closed; a refusal becomes a zero hold; indoor unaffected | **FAIL** (one sub-item); the rest PASS on measured scope | Refused with a named reason: missing fields, `z_valid` false, epv 2 m, dead reckoning, textureless or all-invalid or all-NaN frames, slope 5° and 10°, attitude error 3.2°, PX4 drift ±0.5 m. Malformed epv/NaN/int-typed booleans raise InputError, which faults the session (fail-closed). No silent fallback or exception-to-accept path was found. FAIL: wrong-surface fits are accepted. Box top hiding the ground: −0.20 m (report). Bridging plane: my spot check reproduces it exactly (+0.1095 m, tilt 2.7957°, `below_share` 0.0); with it, 8 of 60 columns of a dark bar just above the camera are lost (P1-1). Refusal → zero hold: PASS in the session with a stand-in runtime (`SessionHoldOnRefusal`); not exercised on a ROS graph. |
| 5 | Tests cover the exceptions with non-vacuous positive controls | **FAIL** | Positive controls exist for most negatives, and no guard passes by refusing everything (the 30/30 pose sweep is accepted). Gaps: no negative test for invalid obstacles at the drone's altitude (the branch tests use a valid branch at 1.15 m or a branch closer than MinZ; the wires are 0.25·D above the camera); no test of a near object next to ceiling-removed pixels; one vacuous assertion (`assertGreaterEqual(detected_valid, 0)`, test line 605). Several "MeasuredLimitations" tests pin missed obstacles as passing behaviour. |
| 6 | Training-vs-OAK-D and before/after policy comparisons exist and are reproducible | **PASS** (synthetic scope) | `analysis/obs_policy_compare.py` and `case_summary.py` exist, run on CPU and read the checkpoints read-only. Spot checks reproduced: tree line, seed 1, (i) v = (2.429, −1.206), yaw −1.535 against reported (2.43, −1.21), −1.54; (iii) dv 0.001 against reported 0.00. The bridging plane +0.1095 m / 2.7957° matches. The comparison does not include the failing scene classes found here. |
| 7 | Opt-in only: default/indoor path byte-identical; raw data, checkpoints, results and aerial_gym untouched; thresholds PROVISIONAL | **LIMITED_VALIDITY** | Runtime: byte-identical. Independent two-process comparison (pre_tree modules against current modules, 12 scenes × 6 indoor contracts): 60 preprocess outputs, 72 projections, 72 `height_band` results and 96 exception messages all identical (`probes/p4_*`). Contract layer: not identical for inputs the old code refused. An indoor contract with `ceiling.physical` true or false now validates (before: refused), and `height_reference_problem` returns None for an invalid contract carrying only `profile:"outdoor"` (P3-3). Raw data, checkpoints, results and aerial_gym untouched (PASS). Contract thresholds are all PROVISIONAL, but the runtime's fixed constants are not (P3-4). |

Other checks:
- Test pollution: PASS on the 13-module one-process run. The new module adds no `sys.modules` stubs and no `os.environ` writes. Its private module loads are not registered, the mock patches are context-scoped, and the actual-policy run happens in a subprocess with a filtered environment.
- `tools/oakd_outdoor_replay.py`: PASS. No DepthAI import (a test asserts `depthai` is not in `sys.modules`), no device. It writes only to the required `--output` directory (`mkdir(exist_ok=False)`, files opened with `'x'`), and there is no default path inside the repository.

## Findings

### P0-1: the far-ground band frees dark invalid obstacles at the drone's own altitude (rule 2, no RGB test)

`tools/camera_policy_runtime.py:741`: `far_ground = si & ((meets & (z_ground > maxz)) | (~meets & (v > 0)))`.

Every sensor-invalid pixel whose level ray meets the ground beyond 20 m becomes no-return. Those are the rows from the
horizon down to atan(h/MaxZ): 2.95° at h 1.03 m, 4.4° at 1.53 m. At distance D this band covers heights from camera
height down to camera height − D·tan(2.95°), which at 1–5 m is the vehicle's own vertical extent. Any invalid
structure that lies wholly in the band, or leaves fewer than 3 rows outside it (the blob_near 5×5 / 0.5 density rule),
disappears.

Failing input (`probes/p1_horizontal_bar.py`, `p1_horizontal_bar.json`): a dark, textureless, sensor-invalid rail
0.10 m tall, 3 m wide, its top 0.01 m below the camera.

| Variant | 1.5 m | 2 m | 3 m |
|---|---|---|---|
| Outdoor profile, rail columns < 10 m | **0/156** | **0/156** | **0/114** |
| Same rail with valid depth (control) | 156/156 | 156/156 | 114/114 |
| Same invalid rail 0.30–0.45 m below the camera (control) | 156 near | 156 near | 114 near |
| Indoor-like view without sky (pre-change logic) | 156 | 156 | 114 |
| Training-equivalent view | 156 (1.40 m) | 156 (1.90 m) | 114 (2.90 m) |

The loss is identical for h 1.0 and 1.5 m, pitch 0/+5/−5°, and on both projection paths (`trained_equal` level path:
0/156 at 2 m, 0/114 at 3 m). A dark 0.10 m bar centred on camera height is also lost at 2–5 m for pitch 0 and −5°
(kept at 1.5 m, and at +5° for 2 m). At h 1.5 m and 5 m, even the "well below" 0.30–0.45 m control is lost (0/66),
because the band there is 4.4°.

Policy level (`probes/p5_policy.py`, seed 1, 30 frames): with the rail at 2 m, the outdoor-profile command is
(2.432, −1.279), within **0.072 m/s** of the open-field command, i.e. no reaction. The training-equivalent command is
(1.482, 1.343): it turns away, 2.72 m/s from the open-field command. At 3 m the figures are 0.072 and 2.57 m/s.

Why this is plausible: horizontal-baseline stereo is weakest on horizontal, baseline-parallel, textureless or shaded
structures, and the emulation already treats such surfaces as depth 0. Examples are fence rails, ropes and chains
across a path, gate barrier arms and horizontal branches at about 1 m.

Suggested fix:
- Do not free band pixels on geometry alone. Decide per connected invalid component: keep the whole component if any
  part of it lies outside the band and is not sky.
- Require positive far evidence: colour/luma continuity with the valid ground below, or a sky region freed by rule 1
  directly above in the same column.
- Add negative tests for invalid rails and branches from −0.15 to +0.05 m around camera height, at 1.5–5 m,
  h 1.0/1.5 m and pitch ±10°.

A rail lying wholly inside the band cannot be told apart from invalid far ground by depth alone. Resolving it needs RGB
or temporal evidence (or the OAK-D confidence map). Otherwise the user must accept it explicitly as a residual risk.

### P0-2: bright or light-grey invalid objects at or above the drone's altitude are freed as sky (rule 1, combined with rule 2)

`camera_policy_runtime.py:743-746`. A sky candidate needs only luma ≥ 0.5 and gradient ≤ 0.03, computed against
neighbouring invalid pixels only. It must also be 4-connected to row 0 through candidates, unimaged pixels or
ceiling-removed pixels.

(a) Light-grey objects (RGB 200/205/210, luma about 0.80, close to the sky luma) are freed whatever the background.
This is the low-contrast case.
- A panel 1.5 m away spanning camera −0.10 m to +0.45 m: **0/66 columns** with a 5 m tree line, a 2.5 m tree line,
  or no tree line (`p2d_altitude_panel.json`). The dark twin is 66/66 in all three.
- A trailer-like raised box at 2 m (bottom 0.10 m below the camera, top 2.6 m), white or light grey: **0/138 columns**
  at pitch 0 and ±5°. The dark twin is 138/138 (`p2c_roof_bridge.json`).
- A white or grey 0.10 m bar centred on camera height at 2 m: 0/156 at pitch 0 and −5° (`p1`).

(b) White objects are freed when the virtual ceiling bridges the flood (see criterion 3).
- The same altitude panel in white: **0/66** with a distant 5 m tree line. Its top above 2.9 m is removed by the
  ceiling (928 pixels) and connects the sky to the panel. With a 2.5 m tree line or none: 66/66.
- Same frame, `apply_sky_rule(..., above_ceiling=None)`: 2178 panel pixels stay invalid; with the connector, 132.
- Policy (seed 1): for a white panel at 1.5 m (camera +0.05 to +0.55) beside a 5 m tree line, the outdoor command
  equals the open-field command (dv **0.000**). The training-equivalent command differs by 2.51 m/s.
- For the raised box (scene C), 2 edge columns (38 near pixels) survived, and the seed-1 command stayed within
  0.10 m/s of the training-equivalent one. Here the loss is shown at the observation level only.

Plausible scenes: the cargo body of a box truck or trailer (lower edge about 1 m), white sign boards, gate barrier arms,
awnings, light concrete or overexposed surfaces in sun.

Suggested fix:
1. Drop `above_ceiling` from the connector set, so the virtual ceiling acts on valid returns only, as the contract
   comment says (`camera_policy_contract.py:47`). Accept the 3.2 m-canopy sky as a fail-safe near region, or refuse
   that frame.
2. Do not free invalid pixels inside the vehicle's altitude slice (the elevation band that obstacles at the drone's
   height occupy at near range) without positive far evidence.
3. Replace the luma/gradient test with a sky model calibrated on real OAK-D data (hue/saturation, exposure reference
   from the top rows) or add temporal/parallax evidence.
4. Add negative tests for bright and light-grey objects whose lower edge is between camera −0.15 m and +0.10 m, and
   for near objects next to ceiling-removed and unimaged pixels.

### P0-3 (gate-listed exception): bright invalid overhead obstacles below the 3 m virtual ceiling are lost

`p7_overhead_bright.json`: a white or light-grey invalid canopy underside at 2.0 m and at 2.5 m gives 0/160 columns
< 4 m, minimum 13.26 m. The valid textured twin is 160/160 (2.14 m and 3.23 m); the dark invalid twin is 160 near.
The object is above a 1.0 m flight, so the collision risk applies to climbs and to the airframe's top. I list it at P0
only because the criterion names overhead obstacles below 3 m explicitly; by collision geometry alone it would be P1.
Fix: as P0-2 (sky candidates touching a large bright invalid region that meets the ground plane's "near" geometry
should stay invalid), plus a test.

### P1-1: wrong-surface ground fits are accepted (criterion 4)

`camera_policy_runtime.py:604-614`: the consistent plane with the most support is taken, with no requirement that the
true ground was seen. Reproduced: h 1.0, pitch 0, box x 3–8 m with top 0.2 m gives an accepted plane at
+0.1095 m / 2.7957°, `below_share` 0.0. The true ground is not among the candidates.

Effect: the band shifts above the gravity horizon. A dark invalid bar from camera +0.02 to +0.12 m at 2 m keeps 52/60
columns (control without the box: 60/60); at 2.5 m it keeps 42/48 (control 48/48) (`p3a_bridging_plane.json`). The
report's other case is a box hiding all ground, accepted at −0.20 m.

Fix:
- Require plane support spread across a minimum range interval and azimuth sector.
- Require the residual RMS to be consistent with stereo noise at range.
- Refuse when the fitted tilt disagrees with PX4 attitude by more than the attitude-error budget and no slope evidence
  exists.
- Or add temporal plane consistency.

### P2-1: the outdoor profile cannot run on the ROS node; holds have no limit and no hysteresis

- `camera_ros2_adapter.py:293`, `odometry_state`: the state never carries the PX4 vertical fields. Every outdoor ROS
  frame is therefore refused (`camera_policy_runtime.py:528-531`).
- `camera_ros2_node.py:65` accepts the outdoor contract at load, because `height_reference_problem` returns None for
  outdoor contracts (`camera_policy_contract.py:248`). The node starts and holds forever.
- `camera_ros2_session.py:282-283`: every refusal sets `_fresh_history`, so intermittent refusals alternate zero holds
  with policy commands, each starting from a reset history. There is no time limit and no N-good-frames re-entry rule.

Fix: the node refuses an outdoor contract as UNCONFIGURED until the gateway forwards `z_valid`/`epv`/`dead_reckoning`.
Add re-entry hysteresis and a bounded hold that escalates (a user decision).

### P2-2: invalid thin wires are never detected (pre-existing)

1–3 source-pixel invalid wires: 0 near columns before and after (`test_wires_one_to_three_pixels`). This comes from
blob_near density at 160×90, not from this change, but the gate lists wires. The test also asserts nothing about
valid wires (see P3-1).

### P3 items

- **P3-1.** `tests/test_camera_runtime_outdoor_r1.py:605`: `assertGreaterEqual(detected_valid, 0)` is vacuous. The
  "MeasuredLimitations" class pins missed obstacles as passing; rename these tests or mark them as expected-failure
  characterisations so a fix is not reported as a regression.
- **P3-2.** `camera_policy_runtime.py:444`: epv as the string `"0.1"` or as a 1-element array is accepted
  (`p6_failclosed.json`). Require a real float type.
- **P3-3.** Contract schema:
  - `camera_policy_contract.py:277` now accepts `ceiling.physical` on indoor contracts (before: refused).
  - `:248` returns None for any contract with `profile:"outdoor"`, even an invalid one; `validate_contract` still
    refuses it.
  - `:377` allows `refuse_dead_reckoning:false`, `sky.enabled:false`, `height_tolerance_m` up to 2 m and
    `tilt_max_deg` up to 30°. The outdoor CLI `--ceiling-height-m` accepts e.g. 1.2 m (`p6`).

  Restrict `physical` to the outdoor profile, and tighten the bounds or require an explicit override.
- **P3-4.** `camera_policy_runtime.py:38-46`: the fixed ground-fit constants (notably `GROUND_SEARCH_TILT_DEG = 30`,
  the sample sizes, and the `2*threshold` removal at `:602`) are thresholds in effect but are not marked PROVISIONAL.
- **P3-5.** `SKY_CONNECTIVITY = "top_border_4_through_sky_or_unimaged"` (`camera_policy_contract.py:56`), and the
  comment at `:46-48` says the virtual ceiling "never explains a sensor-invalid pixel". Both contradict the runtime,
  which also connects through ceiling-removed pixels (`runtime.py:745`).
- **P3-6.** `OutdoorOffIsBitIdentical` depends on an absolute `~/motar_tmp` path (it skips elsewhere). It also runs the
  old runtime against the NEW `camera_policy_contract` module, because the import is resolved through `sys.modules`.
  My independent two-process comparison covers this gap.

## Report claims: confirmed or refuted

| Claim (OUTDOOR_r1.md) | Reviewer measurement | Status |
|---|---|---|
| 37 new tests OK; focused camera modules OK | 37/37 OK; 13 modules in one process: 198 OK | Confirmed |
| PUBLIC_CPU run 3 `Ran 4182 … OK (skipped=10)` | Log line present (not rerun) | Confirmed (log only) |
| Sky: near 160 → 0 at zero roll | Asserted by SkyRule, which passes; seed-1 tree line (iii) − (i) = 0.001 m/s | Confirmed |
| Tree line seed 1: (i) (2.43, −1.21), −1.54; (iii) dv 0.00 | (2.429, −1.206), −1.535; dv 0.001 | Confirmed |
| Bridging plane +0.110 m / 2.80° accepted | +0.1095 / 2.7957, accepted | Confirmed |
| Indoor/default behaviour byte-identical | Runtime: identical (60 preprocess, 72 projections, 72 band results, 96 exceptions). Contract validation: 4 differences on previously refused or invalid inputs | Confirmed for runtime; narrowly refuted for contract validation |
| Gateway/geofence untouched | SHA-256 equal to pre_tree | Confirmed |
| Checkpoint SHA pins unchanged | Both equal to the pins | Confirmed |
| "Why ground-standing near obstacles stay detected … Lost only: invalid parts lying wholly inside the far-ground band or above the horizon and sky-like" | True as a mechanism, but these classes include whole obstacles at the vehicle's altitude: 0/156 rail columns, 0/138 raised-box columns, 0/66 altitude-panel columns, policy dv 0.072 / 0.000 m/s against open field | Mechanism confirmed; impact refuted (the report calls these residual risks; the gate makes them failures) |
| Ceiling connectivity (deviation 4) needed for the 3.2 m canopy sky | Also frees a near white panel at the vehicle's altitude (0/66 against a 66/66 control) | Side effect not reported; refutes the criterion-3 compliance |
| Ground pose sweep 30/30 accepted | Test `test_attitude_error_and_pose_sweep` passes (refused == 0) | Confirmed |
| ROS path refuses every outdoor frame | Adapter state has no vertical fields; code path confirmed | Confirmed |

## Reproduction

Every probe is in `review_r1/probes/` and imports the worktree read-only. Run them from that folder with
`CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 taskset -c 0-3,6-23 timeout -k 10 900 <aerialgym python> -B <probe>.py`.
p4 runs as `p4_make_scenes.py`, then `p4_eval.py pre`, then `p4_eval.py new`. JSON outputs sit next to each script.

## Summary

The change fixes the sky wall in the runtime and keeps the default/indoor runtime byte-identical. Ground verification
fails closed for estimator faults, missing fields, textureless ground and steep slopes.

It does not meet criterion 2 (near obstacles), criterion 3 (the virtual ceiling must not license deleting invalid
pixels) or criterion 5 (test coverage). Both sky-rule paths delete sensor-invalid obstacles at the vehicle's own
altitude in plausible scenes:
- rule 2, the far-ground band without any RGB test;
- rule 1, the luma/gradient test, which ceiling-removed and unimaged pixels can bridge.

On the ROS node path the profile is permanently in hold. Criterion 4 fails one sub-item (wrong-surface fits are
accepted). These findings are open.
