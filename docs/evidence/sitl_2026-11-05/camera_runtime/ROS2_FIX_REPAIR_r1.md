# Camera-runtime repair r1 (2026-11-13): fixes for review r1 findings F1-F10 and cheap P3 items

Worker: Claude Code subagent (Opus 5.5). Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch
`camonly/dev`, base `fc8aa4d`. Rounds r1 and r2 were uncommitted in 18 paths and were kept (pre-repair copies and
SHA-256 in `repair_r1/baseline/`; the runtime `bc178e99…` and gateway `913df863…` match the reviewer's manifest).
CPU only: no GPU, simulator, ROS launch or hardware. No commit, push or stash. Usage: USAGE_UNKNOWN
(`tools/agent_session.py status`: `usage_observation_status: UNKNOWN`). Scratch: `repair_r1/` next to this file.

Not touched: `aerial_gym/**`, checkpoints, preregistrations, `results/`, other evidence, Frozen Policy F, launch
files, `config/unconfigured.json`, permission and settings files. `tools/camera_ros2_adapter.py`,
`tools/camera_ros2_session.py`, `tools/oakd_camera.py` and `tools/sitl_camera.py` were not edited in this round
(byte-identical to their pre-repair copies).

## Decisions outside the written spec (read first)

1. **F6 lives in the contract and is enforced by the runtime, on every path.** New required contract field
   `estimator_height`: `null` (default, never invented) or `{"z_is_height_above_floor": true, "evidence": {"path":
   absolute, "sha256": hex}}`, built by `--height-reference-evidence FILE` (the tool hashes the file;
   `validate_contract` re-hashes it). `height_band()` raises `InputError` when a filter is enabled without it, so the
   frame fails closed in the ROS session (FAULT), the OAK-D/MAVLink companion, the SITL path and the CPU replay alike;
   `camera_ros2_node.load_node_config` additionally refuses such a contract (`--check` → NOT_CONFIGURED). Consequence:
   **the default contract (floor filter on) now refuses every frame until the operator asserts the height
   reference**, or the contract is built with `--no-ground-removal`. The ROS binding was not changed; its templates
   stay as they were (unconfigured). Tests use a clearly labelled CPU fixture file as evidence.
2. **F5 colour on the tilted path.** The z-buffer path used to leave sensor-invalid trained pixels black. It now
   gives such a pixel the colour of the invalid ray nearest to its centre (the level path's nearest-sample
   semantics), so the production rule can score it. The score is `perception.segmenter(rgb, ranges,
   max_camera_range) >= perception.pixel_threshold` on the projected frame, with MaxZ at masked pixels; measured in
   the real runtime: `AppearanceTargetSegmenter`, threshold 0.55, max range 20.0, depth-channel weight 0.0, so depth
   does not enter the score. Non-goal sensor-invalid pixels are still blanked, so the new colour reaches the
   observation only through this goal decision.
3. **F5 consequences to state plainly.** (a) The bootstrap rule scores any red object as the goal. Its valid
   pixels were already carved out of the obstacle map (training semantics); now its sensor-invalid pixels also
   never become near, so a red obstacle closer than MinZ is not an obstacle. (b) A goal entirely closer than MinZ
   is now neither detected nor an obstacle (training: detected at about 0.4 m with a biased range; r1/r2: a near
   obstacle).
4. **F3 is strict.** Any coverage-False pixel with a finite, unflagged depth in [MinZ, MaxZ] refuses the frame;
   there is no tolerance band. A producer whose border interpolation differs from the nearest-neighbour map by one
   pixel will be refused on every frame: a bench check, not a code question.
5. **F2 details.** Every nonzero component is checked against the wall it points at (r2 decision G1 kept: the
   far wall binds only in boxes narrower than the stopping distance). The per-axis record now carries
   `scale_limit` (replacing `v_max_mps`) and `clamped` marks the binding axis; `geofence_record` gains `scale`.
   `stop_speed()` is kept; a test pins `axis_scale(|v|,|v|,d)·|v| == stop_speed(d)` on a single axis.
6. **F8 format and placement.** `geofence_waiver` is `null` or exactly `{"reason": "<non-empty text>"}`
   (free text, not hash-bound: the coordinator's spec; the reviewer had suggested a hash-bound waiver). A waiver
   with an enabled fence, or in an observation-only configuration, is refused. Enforced in `load_node_config`
   (after the flight-check binding check), at `BridgeSession` construction (waiver shape) and in
   `BridgeSession.prime` (fence-or-waiver). The pure `Px4OffboardGateway` does not know about waivers.
7. **P3 disabled geofence block.** Box and latency must be null; `decel_mps2` may be null or a number in (0, 10].
8. **F1 contract details.** `invalid_plane_rule` is a required JSON boolean under `ground` and `ceiling`;
   `true` with that filter disabled is refused. CLI: `--ground-invalid-plane-rule`, `--ceiling-invalid-plane-rule`
   (the latter needs `--ceiling-height-m`).
9. **F11.** `tools/camera_ros2_adapter.py` and `tools/camera_ros2_session.py` were added to the contract's
   `SOURCE_PATHS`. With the schema change (two new fields), every previously built contract is invalid again.

## Findings → change → test → before/after

Line numbers refer to the working tree after this repair. "Fails on the pre-repair code" was checked by running the
new test file against a scratch tree with the pre-repair (r1+r2) sources (`repair_r1/pre_tree/`, log
`new_tests_on_pre_repair_tree.log`): `Ran 24 tests` / `FAILED (failures=138, errors=16)` (the 7 actual-policy tests
did not run: the old contract API has no `estimator_height_assertion`). The only two tests that pass there are
controls meant to hold on both versions (the plane rule erases the box when ON; iid 2 % holes stay valid).

| Finding | Change (file:line) | Regression test (fails before) and controls | Before → after evidence |
|---|---|---|---|
| **F1** plane rule erases a near box below camera height | `camera_policy_contract.py:29-31` defaults `invalid_plane_rule: false`; validation `:194-224`; CLI `:358-361`, `:384-397`. `camera_policy_runtime.py:344-349` (floor) and `:359-364` (ceiling): the invalid-pixel plane rule runs only when opted in; valid-hit removal unchanged | `test_camera_runtime_repair_r1.py:118` box 0.3 m ahead, tops 1.00/0.98/0.90 (3/5/13 cm below the 1.03 m camera), trained-equal level, OAK-like pitch 0/1/3°, with and without a textured floor, default contract: 24 cases, near > 0, column min over the box columns in [0.40, 0.48], no box column ≥ 1 m. Negative control `:143` rule ON erases it (min > 9 m); positive control top 1.20 kept. Contract/CLI `:161`. Policy level `:935`. Test 2 (rule explicitly ON, textureless floor/ceiling) kept in `test_camera_runtime_fixes.py:370` plus a rule-off control | probe A, production preprocess, default contract: before (r1/r2) tops 1.00/0.98/0.95/0.90 → near 0, col min 9.9002 (all sources); after → near 5760/4800/3360/1120, col min 0.4023/0.4061/0.4151/0.4364 (trained-equal level), 5920/4960/3520/1120 and 0.4019/0.4054/0.4139/0.4364 (OAK-like pitch 1°). Real policy: newest obstacle token 0.404/0.408/0.438 m (level), 0.404/0.407/0.438 (pitch 1°), 0.403/0.407/0.436 (pitch 3°); with the rule ON 9.9 m, no token < 1 m. Review's box scene `box_top0.98_level_trained_floor` with the floor filter on: before tokens 9.9 m, after 0.408 m and the observation bit-identical to fc8aa4d |
| **F2** diagonal corner overshoot | `camera_ros2_px4.py:15-29` docstring (direction preservation, parallel components slow together, latency guidance); `:89-98` `axis_scale`; `:166-191` `Geofence.limit` (one scale s); `:476-484` record `scale`; `:499` clamped flag | `repair_r1.py:263`: 128 2-D rollouts (d 0.4/0.5/1.5/3 m × τ 0.2/0.4 s × speed 2.5/3.54 m/s × 8 geometries: corner 45/30/60°, asymmetric corner, opposite corner, single axis ±, edge diagonal) with an independent total-deceleration double integrator (`braking_rollout`, `:195`): no crossing (≤ 1e-9 m), direction kept, never faster, binding axis reaches its wall (> −1e-6 m), 90 clamped and 38 unclamped (non-vacuous). Negative control `:290` the old per-axis clamp crosses by 0.419 m; single-axis control equal to the old clamp. Gateway `:304`: corner scaled, box centre bit-exact vs the disabled twin. `:325` single-axis equivalence | exact rollout over the 128 cases: max overshoot new 7.8e-14 m; old clamp crossed in 64 cases, max 0.4353 m. Review case d 1.5 m, τ 0.2 s, (2.5, 2.5): before published (2.4465, 2.4465), overshoot 0.4186 m; after (2.1222, 2.1222), 8.9e-16 m. probe_geofence (norm-limited): before +0.1054/+0.4186/+0.0572/+0.2862 m; after +0.0000 in all four; single-axis rows unchanged |
| **F3** coverage erases valid depth | `camera_policy_runtime.py:93-123` `_source_classes`: coverage False with a valid return raises `InputError`; without one it stays no-return | `repair_r1.py:340` the review's pole (cols 11-25, 0.956 m) with coverage False on 26 columns is refused (ground off/on, and `project_rgbd`); controls: correct (all-True) coverage and no key read 0.956 m; a coverage-False border delivered at depth 0 is no-return with near 0. `:362` ROS path: a distorted CameraInfo over an already undistorted image is refused; control with the border at depth 0 accepted | probe B: before col min 20.0 m (2340 no-return px); after `refused: coverage is False at 2340 pixels that carry a valid depth return…`; without the mask 0.9562 m before and after |
| **F4** speckle hides a near surface when tilted | `camera_policy_runtime.py:29-35` `ZBUFFER_INVALID_SHARE = 0.5` with the rationale; `:126-140` `_splat_index(offset=)`; `:208-240` share rule and nearest-invalid-ray colour | `repair_r1.py:383` slab 0.3 m ahead, 5/10/30 % speckle at 6 m, pitch 0.5° and 3°, floor filter off and on: near > 0, slab columns 0.40-0.48 m; negative controls: share 1.0 (the r1/r2 "any valid wins" rule) and the fc8aa4d re-implementation read > 5.9 m. `:413` iid 2 % holes at 3° and 0.5° (two sources): near 0, no column < 1 m. `:424` 2×-resolution source: 0 or 1 of 4 samples invalid → valid, 2 (tie), 3, 4 → sensor-invalid; the old rule only at 4. Policy level `:947` | probe C, slab col min: pitch 0.5°, 5/10/30 %: before 6.0002/6.0002/6.0001 m (near 0) → after 0.4 (near 14400/14400/14387); pitch 3°: before 6.0004 m → after 0.4 (near 13600/13600/13575). Real policy: token 0.402 m at 0.5° and 3° |
| **F5** dense invalid goal patch becomes a 0.4 m obstacle | `camera_policy_runtime.py:515-523` `goal_coloured` (production segmenter and threshold); `:540-541` goal-coloured sensor-invalid → no-return; `:549` they keep their RGB; count `goal_invalid_pixels` `:552`, `:614` | `repair_r1.py:896` real runtime, half-invalid red sphere at 5 m and 1.5 m, trained-equal level, OAK-like level, OAK-like pitch 0.5° (z-buffer): goal pixels > 0, near 0, no token < 1 m within ±10° of the goal, surface within 0.1 m of 4.75 (1.25 for 1.5 m); negative control (goal rule disabled, i.e. r2): near > 0 and a ≤ 0.45 m token; full-sphere control same surface. `:919` grey dense slab still near (token < 0.5 m) at 0° and 0.5°. `:890` the rule is the perception's own. `:927` bit identity: 9 review scenes without invalid goal pixels (level and 0.5° all-valid), floor filter off and on, equal to golden SHA-256 of the real fc8aa4d observation | real policy, newest tokens: before (pre-F5) 0.4 m @ 0° in all five half-invalid cases (near 6/10/6/194/192); after 9.908/9.908/9.908 m (5 m) and 9.97/9.97 m (1.5 m), near 0; goal surface 4.7988/4.8041/4.7918 m (5 m), 1.2961/1.2958 m (1.5 m). run_rt (review frames): `pitch0.5_sphere1p5_halfinvalid_oak` before token 0.4 m (as fc8aa4d), after none; all 9 golden scenes after == fc8aa4d bytes (off and on) |
| **F6** unchecked height reference | `camera_policy_contract.py:33-38`, `:158-191` (`_evidence_problem`, `estimator_height_assertion`, `height_reference_problem`), `:232-246` validation, `:310-317` evidence re-hash, `:249-262` `build_contract(estimator_height=)`, CLI `:362-365`, `:395-397`, `--check` field `height_filters` `:403`; `camera_policy_runtime.py:255-270` `height_band` raises; `camera_ros2_node.py:63-67` refusal | `repair_r1.py:454` unasserted contract with floor, ceiling or both refuses (`height_band`, preprocess); asserted runs; filters off need nothing; malformed assertions refused. `:480` ROS session FAULT without, proposal with. `:505` contract structure, no invented default, CLI binding. `:544` (real contract files) the ROS node refuses an unasserted contract and loads the asserted one; evidence change detected | before: an unasserted default contract processed every frame (pre-repair tree: 6 failing subtests of `:454`); after: `InputError: floor/ceiling filter enabled without the operator assertion…`; contract `--check` prints `height_filters: REFUSES_EVERY_FRAME_UNTIL_ASSERTED` |
| **F7** stale records | `ros2/motar_camera_policy/README.md:69-75` (latching exceptions), `:83-136` (contract rebuild, REP 117, coverage, share rule, goal colour, height filters, `estimator_height`, WAITING_ALTITUDE, watchdogs), `:138-166` (geofence, `~/reset_geofence`, latency guidance "2 x (vehicle response delay + 0.1 s)", breach hold, waiver), `:191-192`; `package.xml:22-23` `python3-opencv`; `docs/evidence/sitl_2026-11-05/camera_runtime/README.md:194-227` new dated section (history above unchanged) | `repair_r1.py:581` README terms; `:590` SITL section dated, "not comparable", earlier text byte-equal to fc8aa4d; `test_camera_ros2_package.py:65-67` dependency | before: none of the 14 terms present (pre-repair tree: 14 failing subtests) |
| **F8** publication without a fence | `camera_ros2_px4_node.py:18-67` template `geofence_waiver: null`, `geofence_waiver_problem`, `offboard_geofence_problem`; `:144-146` `load_node_config`; `:162-164` construction; `:233-235` `prime`; diagnostics `:261`, `--check` `:477`; `config/px4_unconfigured.json:30` | `repair_r1.py:617` prime refused with neither; starts with a reasoned waiver and with an enabled fence; malformed waivers, waiver + enabled fence, waiver in observation-only refused; observation-only needs neither. `:640` node `--check`: neither → 2/NOT_CONFIGURED, fence → 0/ENABLED, waiver → 0/DISABLED with the reason | before: publication primed with the fence disabled (pre-repair: test errors on the missing key) |
| **F9** test gaps | covered by the rows above: every failing input of F1-F5 is a test; the geofence tests use `braking_rollout`, a total-deceleration model independent of the clamp; bit identity is pinned to the real fc8aa4d run (golden hashes in `repair_r1.py:751-761`) | — | — |
| **F10** SITL +inf | `camera_policy_runtime.py:373-384` `gazebo_frame`: +inf unflagged (no-return); NaN, −inf, z ≤ 0 sensor-invalid | `repair_r1.py:681` +inf patch: no sensor-invalid, no-return > 0, near 0; NaN/−inf/0/−1 patches near > 0; negative control (old rule) near > 0. Policy level `:955` | real policy, Gazebo frame: +inf patch near 0, tokens ≥ 9.9 m; NaN patch near 1844, token 0.403 m |
| **P3** F11, F12 | `camera_policy_contract.py:54-57` adapter and session in `SOURCE_PATHS`; `camera_ros2_px4.py:63-66`, `:117-143` bounds and disabled-block values | `repair_r1.py:705`, `:713` (decel 10 and latency 1.0 accepted; 10.01, 0, 1.01, 5, −0.1 refused), `:721` | before: unbounded, disabled values ignored, adapter unbound |

### Existing tests changed by design (no check weakened beyond the specified semantics)

- `test_camera_runtime_fixes.py`: `contract()` (`:40-58`) gains `plane_rule` and the labelled CPU assertion fixture;
  `test_precedence…` (`:252`): the uncovered valid pixel now refuses the frame (F3), then +inf there is no-return;
  `test_valid_majority_pixels_stay_valid_and_level_minz_applies` (`:306`) renamed from `…valid_point_wins_over_
  invalid_ray…`, same assertions (F4); textureless test (`:370`) opts into the rule and adds the rule-off control;
  actual-policy script: assertion fixture, `on+plane` variants; `test_6`/`test_b2` (`:863`, `:891`): the
  textureless cases use the opt-in rule and the default is asserted to read near; `test_7` (`:932`): the invalid
  goal half is now `goal_invalid > 0`, `si == 0`, `near == 0` (F5; before: `si > 0`, `near > 0` at level); the
  legacy stand-in reports `goal_invalid_pixels = 0` (`:603`).
- `test_camera_runtime_fixes_r2.py`: `test_parallel_component…` (`:275`): north is now scaled with east (F2;
  before: bit-exact 1.5), with the exact wall equation and `scale` checked; `NoGoalPerception`/`geometry_runtime`
  (`:467-488`) for checkpoint-free preprocess; actual-policy contract asserted (`:764`).
- `test_camera_policy_runtime.py:144-150`, `test_camera_ros2_integration.py:18-33`: assertion fixture; the
  integration bridge carries a reasoned waiver (F8). `test_camera_ros2_px4_node.py:29`, `:154`: waiver for the
  publishing bridge fixtures. `test_camera_ros2_package.py:65-67`: `python3-opencv`.

## Verification (exact lines)

- Focused (all 17 `tests/test_camera_*.py`, `test_oakd_camera`, `test_depth_validity`, `test_realveh`; runner
  `repair_r1/run_named.sh`, log `repair_r1/focused_repair_r1.log`): `Ran 242 tests in 53.412s` / `OK` / `EXIT=0`
  (r2: 211; +31 new in `test_camera_runtime_repair_r1.py`).
- Full PUBLIC_CPU (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= nice -n 10 timeout
  3600 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/run_tests.py`, log `repair_r1/public_cpu_repair_r1.log`):
  **`Ran 4145 tests in 777.144s` / `OK (skipped=10)` / `EXIT=0`**: 0 failures, 0 errors, 4145 > 4114 (r2; +31 new).
  The 5 GPU_REQUIRED exclusions are the same as in r1/r2. Source SHA-256 at launch (`repair_r1/sha256_at_public_cpu_launch.txt`,
  every uncommitted path plus the three docs) re-checked after the run: 0 mismatches.
- Kernel fault count (`journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"`):
  0 at the start of work (00:50 KST); 0 before and 0 after the focused run; 0 before PUBLIC_CPU (01:19:42 KST) and 0 after
  (01:32:50 KST). cpu4-5 offline throughout (`repair_r1/kernel_faults_*`).
- Review probes before/after: `repair_r1/probes_before_after.txt` (script `repair_r1/compare_probes.py`). Before =
  the reviewer's outputs, re-run unchanged on the pre-repair tree with byte-identical results
  (`repair_r1/probes_before/`). After = `repair_r1/probes_after/`; `probe_scan` and `run_rt` needed minimal
  adaptations, recorded as diffs (`probe_scan_after.diff`: a no-goal perception stand-in, the `on+plane` variant,
  `InputError` recorded instead of raised; `run_rt_after.diff`: the estimator-height fixture). `probe_slerp` output
  is byte-identical (pose interpolation untouched). Real-policy measurements: `repair_r1/measurements_repair_r1.json`,
  `measurements_r1_tests_after.json`, `measurements_r2_tests_after.json`; geofence summary
  `repair_r1/geofence_rollout_summary.json`.

## Changed files

Repair-only patch: `repair_r1/REPAIR_r1_only.patch` (sha256 `a4a19ef2…`), against the pre-repair copies.

| Path | Status | Repair r1 lines (+/−) |
|---|---|---|
| `tools/camera_policy_runtime.py` | modified (also r1, r2) | +121 / −52 |
| `tools/camera_policy_contract.py` | modified (also r1) | +120 / −16 |
| `tools/camera_ros2_px4.py` | modified (also r2) | +79 / −32 |
| `tools/camera_ros2_px4_node.py` | modified (also r2) | +47 / −2 |
| `tools/camera_ros2_node.py` | modified (also r2) | +5 / −0 |
| `ros2/motar_camera_policy/config/px4_unconfigured.json` | modified (also r2) | +2 / −1 |
| `ros2/motar_camera_policy/README.md` | modified (new this round) | +91 / −4 |
| `ros2/motar_camera_policy/package.xml` | modified (new this round) | +2 / −0 |
| `docs/evidence/sitl_2026-11-05/camera_runtime/README.md` | modified (appended section only) | +34 / −0 |
| `tests/test_camera_runtime_repair_r1.py` | untracked, new | 964 lines |
| `tests/test_camera_runtime_fixes.py` | untracked (r1) | +77 / −29 |
| `tests/test_camera_runtime_fixes_r2.py` | untracked (r2) | +39 / −9 |
| `tests/test_camera_policy_runtime.py` | modified (also r1) | +8 / −2 |
| `tests/test_camera_ros2_integration.py` | modified (new this round) | 11 lines differ |
| `tests/test_camera_ros2_package.py` | modified (new this round) | 3 lines differ |
| `tests/test_camera_ros2_px4_node.py` | modified (also r2) | +5 / −0 |

The working tree now has 24 uncommitted paths: 21 modified tracked and 3 untracked (r1's 10, r2's 8, plus
`ros2/…/README.md`, `ros2/…/package.xml`, `docs/evidence/sitl_2026-11-05/camera_runtime/README.md`,
`tests/test_camera_ros2_integration.py`, `tests/test_camera_ros2_package.py` and the new test file).

## Open items

1. **Independent review of this repair: not done.** AGENTS.md requires a fresh-context Opus review before GO.
2. **Not committed.** A commit needs the user's approval, a WORKLOG entry, an `docs/EXPERIMENTS.md` entry if an
   evidence folder is added, and the review and this repair copied as `REVIEW_r1.md`/`REPAIR_r1.md` into the
   unit's evidence folder.
3. **Textureless floor or ceiling (F1 open item).** With the plane rule off by default they read as a 0.4 m
   surface again. Decide from bench recordings (OAK-D Pro IR projector on, the real floor) whether to enable
   `--ground-invalid-plane-rule`; the rule's cost is the erased near box above.
4. **F6 needs real evidence.** Until the operator binds an estimator-height record (landed z, `dist_bottom`
   agreement), every path refuses frames with the default contract, including SITL and the companion. SITL users
   must assert (for example, the EKF origin at the spawn on the ground plane) or build with `--no-ground-removal`.
   The ceiling `height_m` still has no provenance binding of its own (F16).
5. **F5 limits** (decision 3): red non-goal objects' invalid pixels are never obstacles; a goal entirely closer
   than MinZ is neither detected nor an obstacle. Both follow from the bootstrap red rule and the specified F5.
6. **F3 strictness** (decision 4): a one-pixel border mismatch refuses every frame. Check the real producer's
   border against `camera_info_coverage` on the bench.
7. **Geofence values are unmeasured.** Box, `latency_s` (about 2 × (response delay + 0.1 s) in the crude model) and
   achievable deceleration on the 1.1 kg vehicle. The clamp uses position only, not the measured velocity; a breach
   leaves PX4 in OFFBOARD holding until `~/reset_geofence` or pilot takeover (documented, F17). The waiver is free
   text, not hash-bound.
8. **P3 left open** (as instructed): F13 (watchdogs from the first gated tick: documented operator reset), F14
   (the inverse-nearest path only for an exactly level attitude), F15 (a target within the floor/ceiling margin is
   removed), F16 (ceiling height provenance).
9. **Still open from r1/r2:** real OAK-D K/D, real invalid-pixel statistics and a bench replay (NOT_TESTED); thin-pole
   min-filter (H5b); MAVLink path without pose interpolation; MAX_AGE_S/MAX_SKEW_S as contract parameters;
   `previous_action` vs executed velocity under the clamp; fisheye CameraInfo refused; Python 3.8 vs Humble ABI.
10. **Every previously built policy contract is invalid** (new required fields and two more bound sources).
    Rebuild: `python tools/camera_policy_contract.py --output NEW.json --seed 1 --height-reference-evidence FILE
    [--ceiling-height-m H]`, then update `policy_contract_sha256`.

## Git Traceability

- Branch: `camonly/dev`. Base SHA: `fc8aa4d`. New Commit SHA: none (commit not authorized).
- Changed Files: as listed above; Working Tree: 24 uncommitted paths (21 modified, 3 untracked), all from r1, r2 and
  this repair.
- Tests: focused `Ran 242 tests` OK; PUBLIC_CPU `Ran 4145 tests` `OK (skipped=10)` EXIT=0; kernel faults 0 → 0.
- Remote Push: NO. Main Merge: NO.
- Raw/Checkpoint Integrity: checkpoints only read (contract pins `888e8056…` seed 1 and `203deea4…` seed 2 verified
  by `build_contract` at each runtime construction); `results/`, preregistrations and `aerial_gym/` untouched.
