# Independent review r2 of C0 (camera-only study): the repairs after review r1

- **Reviewer:** Claude Code fresh-context review agent. **Model:** Opus 5.5 (`claude-opus-5-5`).
- **Date:** 2026-11-04, 18:16-18:40 KST.
- **Reviewed:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `842fcf5`. The
  tree was clean before and after (`git status --porcelain` empty).
  - The repair is `git diff 4d03f35 842fcf5`; the whole unit is `git diff ab6531b 842fcf5`.
  - Also read: review r1, the plan with its "After review r1" section, D-2026-11-04-3, AGENTS.md, and `gate_02/`
    (with `gate_01/` for comparison).
- **Mode:** read-only. I wrote only to this `audit/` directory. I started no GPU process, made no commit, and touched
  no other process, worktree or `results/` directory, and nothing of `MOTAR-gainrand`.
- **Basis:** the code, the diffs and my own probes. I did not use the WORKLOG or the author's summaries.

## Verdict: **PASS** (C0 may be labelled PASS)

I found no blocking finding.
- **B1 is closed.** The association now runs in two camera-loss scenes and is checked there. My own three scenes
  confirm it: under `full` the association is active (16, 4 and 16 env-steps), and in every condition that removes
  the target's bearings it is off, with bit-identical outputs. A known-bad module (`assoc_raw`) is caught.
- **M1 is closed.** The governor now returns the camera value after its own projection. I checked 630 unit cases and
  99 end-to-end cases, the 10 m far plane, values near 0.995 × 12 m, and V = 1, 2, 3, 4 and 8 rows.
- **M2 is implemented as described.** I give one precision note on the reading (L-r2-1).
- **M3 is closed.** `ninja` resolves to the env bin, as in a native run.
- **No regression.** Nothing r1 found correct has regressed.

Two medium findings stay open. They do not block C0, which covers the switch on CPU tensors, and the code they
concern is correct by my probes. Both must be fixed, or carried into the record, before C2:
- **M-r2-1:** the tests do not pin the `*_obs_only` governor's semantics;
- **M-r2-2:** the plan's new M1 residual disclosure is factually wrong.

## Findings

### Blocking

None.

### Medium (non-blocking for C0; resolve before the C2 preregistration)

**M-r2-1: the semantics of the raw-LiDAR governor in `no_lidar_obs_only` and `no_lidar_in_fov_obs_only` are not pinned
by any test. Three known-bad modules pass all 12 `SensorAblationC0` tests.**
- **Where:**
  - tests: `tests/test_navrl_sensor_ablation.py:371-389` (replica, checked under `full` only) and `:422-428`
    (obs_only, inequality only);
  - code: `aerial_gym/task/navrl_task/navrl_sensor_ablation.py:109-121` (the raw-LiDAR target-mask replica) and
    `:175-177` (the raw governor branch).
- **Mutants that pass every test** (`probe_extra_mutants.json`):
  - `replica_on_masked`: the replica is computed on the masked LiDAR. This is r1's "missing mutant 3", still not
    added.
  - `raw_mask_masked`: the raw branch drops `_ablation_raw_target_like`.
  - `raw_k2`: the K2 cap is applied in the raw branch.
- **Why the tests miss them:**
  - The replica test runs under `full`, where raw = masked, so it cannot tell raw input from masked input.
  - The obs_only test only asserts that the governor changes when the LiDAR is perturbed.
- **The code is correct**, by my probe (`probe_m2_obsonly.json`):
  - In `scene(11)`, `governed` under C_gL and under B_in_gL is bit-identical to A. That includes the 60° and 135°
    commands, so there is no K2 in the raw branch.
  - In the camera-loss scene, steps 0-2 under C_gL and B_in_gL equal A, while A's target mask has 48 hits per step.
    So the replica does carve the target on the raw LiDAR.
- **Effect if it regressed:** C_gL and B_in_gL would treat a camera-visible target as an obstacle, or cap lateral
  commands. This would confound C_gL − C and A − B_in_gL with nothing in the suite to detect it.
- **Fix (test-only):**
  - Add an identity check: governed(obs_only) == governed(`full`) bit for bit, in a scene with a camera-visible target,
    target-like LiDAR returns (assert hits > 0) and out-of-FOV commands.
  - Add the three mutants to `tests/camonly_c0_mutants.py`.

**M-r2-2: the plan's M1 residual disclosure is wrong. The obstacle camera renders range, not depth along the principal
axis, so there is no cos(bearing) under-estimation. The real off-axis residual is a bearing offset of the
camera column each bin samples.**
- **Where:** `docs/plans/camonly_study_plan_v1.md:254-256`. Evidence:
  - `aerial_gym/task/navrl_task/navrl_detector.py:39` (`rd = wp.normalize(...)`), `:48` (`dist = t`) and `:403`
    (normalised rays): the hit distance along a unit ray, i.e. Euclidean range;
  - `:927-931`: bilinear upsampling;
  - `navrl_perception.py:1917`: the per-bin column `camera_u` is linear in angle, but the render is pinhole.
- **Measured** (`probe_camera_geometry.json`). This is a plain-torch re-implementation of the 40 × 24 ray table, the
  upsampling to 160 × 90 and perception's column map, with an analytic cylinder in place of Warp:
  - The nearest camera column reads the bar's range: 3.003 m for a bar 3 m away at 40°, where the plan's claim gives
    2.30 m.
  - Each in-FOV bin samples a column whose pinhole bearing lies outward of the bin: 20° → 23.4°, 30° → 32.9°,
    40° → 41.1°.
  - So a bar enters the camera scan up to one bin nearer the axis, or is missed. A bar at 20° and 3 m reads 7.39 m in
    the 20° bin and is seen in the 15° bin.
- **Effect:**
  - In A this offset is in the observation only.
  - In B_in and C it also reaches the governor's corridor, so it belongs to "B_in_gL − B_in".
- **Fix:**
  - Replace the disclosure in the C2 preregistration, or in a dated amendment.
  - Confirm it with the plan's sensor-level CPU check, using the real Warp kernel.

### Low

- **L-r2-1: the corrected readings need a precision note** (plan `:262-264`, `:274-275`).
  - **A − B_in_gL is not purely the observation's use.** The in-FOV association also acts through the governor's
    target carve-out (L2).
    - Measured (`probe_m2_obsonly.json`): camera-loss scene, a forward command of 3.5 m/s.
    - A: 3.5 m/s on all steps. B_in_gL and C_gL: 2.0 m/s on all 3 camera-loss steps, because the lost target's return
      becomes an obstacle.
    - B_in_gL − B_in carries the opposite term.
  - **L2 says "camera-only visibility".** In B_in_gL the visibility also includes the out-of-FOV association. In the
    out-of-FOV scene `lidar_visible` = 12 env-steps and `governed` equals A.
  - The C2 preregistration must state both.
- **L-r2-2: the L5 launcher requirements are incomplete** (plan `:281-285`).
  - **Missing from r1:** `gr_run`-style launchers drop inherited `NAVRL_*` variables, so the condition must be set
    per cell.
  - **New (`probe_path.log`):**
    - `PYTHON` must be an absolute path. `play_navrl.sh:19` changes directory before the call at `:56`; a relative
      `tools/camonly_python` exits with 127.
    - `CAMONLY_REAL_PYTHON` must be an absolute path. `tools/camonly_python:9` prepends its directory name, and a
      bare `python` puts `.` first on `PATH`.
  - **PATH against native:** with the wrapper, `PATH` is `envbin:tools:caller`; natively it is `envbin:caller`.
    - The extra `tools/` entry is harmless today: `camonly_python` is the only executable without an extension there.
    - C1 should record `PATH`.
- **L-r2-3: the quiet-startup test (the r1 L3 repair) is close to vacuous.**
  - Where: `tests/test_navrl_sensor_ablation.py:553-555` and `:592-593`; `tools/camonly_entry.py:27-31`.
  - The stub `devnull_stdio` records nothing. A mutant that deletes the quiet branch passes both entry tests
    (`entry_mut/probe_entry_mutants.json`).
  - Add an `ORDER` marker in the stub.
  - The receipt-overwrite and wrapper-PATH mutants are caught.
- **L-r2-4: plan text the C2 preregistration must update.**
  - Stage table `:166` still says "four new conditions … about 2 h". With B_in_gL there are five, plus the optional
    `NAVRL_LIDAR_TARGET_ASSOC=0`.
  - L1 (`:272-273`) omits r1's note that the ego velocity carries the governed command as well.
- **L-r2-5 (information): mutant `fill_zero` is not a single fault** (`tests/camonly_c0_mutants.py:49-67`).
  - Its inner `"full"` wrapper resets `_ablation_removed`, so the governor also reverts to the raw scan.
  - It is still caught for the right reason, by the three K1 subtests.

### Information (no action)

- **The `no_return` branch is redundant.** In `governor_scan` (`navrl_sensor_ablation.py:144-145`), slant ≥ c ≥
  0.995 × max is already invalid in the governor. Removing the branch is not caught, and is harmless.
- **The governor scan honours a non-default `vertical_fov_deg`, but no test covers it.** Hard-coding (20, −10) passes;
  production never passes another value (`navrl_task.py:5332-5339`).
- **`install_governor` reads `inspect.signature(orig_dlc)`** (`:167`). A later wrapper without `functools.wraps` makes
  the install fail loudly with `KeyError`, which is fail-closed. My first M1 probe run hit exactly this.

## Answers to the questions

### R1. Closure of r1

**B1: closed.**
- **The test's own scenes** (`probe_m2_obsonly.json`):
  - `full`: 12 env-steps in each scene;
  - conditions that remove the target's bearings: 0;
  - conditions that keep them: 12.
- **The plan's numbers hold.**
- **My own scenes** (`probe_b1_assoc.json`): in-FOV at 0°, in-FOV at +25°, and the vehicle yawed −75° so the target
  sits at +75°, on the side the test does not use.
  - Under `full` the association is active on 16, 4 and 16 env-steps. Replacing the LiDAR with no return changes
    `lidar_visible` and the state (positive control).
  - In every removing condition, `lidar_visible` = 0, and every perception output is bit-identical when the removed
    bins hold the scene LiDAR, an exact return on the target, uniform random 0.3-11.9 m, or no return. For the
    `same`-governor conditions, `governed` is also bit-identical.
  - Each removing condition equals an unwrapped module fed "no return" in those bins: K1 holds with the association
    active.
  - My own `assoc_raw` mutant differs in every removing condition.
- **The bit-identity is real.**

**M1: closed.**
- **Unit level** (`probe_m1_governor.json`, part A): `governor_scan` followed by the real
  `directional_lidar_clearance`.
  - Production geometry (V = 4, +20…−10°): the clearance equals c within 4e-7 m for c from 0.3 to 11.94 m, and equals
    12 m (no return) at or above 0.995 × 12 m.
  - All 630 cases (V = 1, 2, 3, 4, 8; three vertical FOVs; 0° and 20° bins): the camera path equals a LiDAR seeing the
    same bar within 9.5e-7 m. Where no row is near 0°, the LiDAR also reports no return near 12 m.
- **End to end** (part B): the real perception and the real `_apply_sensor_speed_governor`, with bars at 0°, 20° and
  −30° from 1 to 9 m.
  - `full`, `no_lidar_in_fov` and `no_lidar` give the same clearance within 4.8e-7 m and the same speed. At 4 m that
    speed is 2.7678 m/s.
  - At 9.9995 m and 10 m (the far plane) the camera maps to no return. That is above the 5 m release distance, so the
    speed is identical.
- **The `no_elevation_fix` mutant** is caught by `test_same_bar_same_clearance` (an independent physical check) and by
  the K2 in-FOV clearance test (a formula replica).
- **The `elevation_reversed` mutant** is also caught.

**M2: implemented as described.**
- **Definition:** `no_lidar_in_fov_obs_only` = `{in_fov: False, out_fov: True, governor: raw}`.
- **Perception** is bit-identical to `no_lidar_in_fov`.
- **The governor** is bit-identical to A when the association does not run.
- **Phantom immunity:** a camera phantom at 3.5 m gives A 3.5 m/s, B_in 2.384 m/s and B_in_gL 3.5 m/s.
- **The reading is right in structure.** It needs L-r2-1's precision note.

**M3: closed.**
- `play_navrl.sh` is byte-copied into the audit directory with a stub `runner.py` (`probe_path.log`).
- For caller `PATH` = `/usr/bin:/bin` and for the full shell `PATH`, the native and wrapper runs both start `PATH` with
  the env bin and resolve `ninja` there.
- The only difference is the extra `tools/` entry (L-r2-2).

### R2. Regressions

- **Online LiDAR cut.**
  - The repair changes only how the governor scan fills removed in-FOV bins (still the camera-fused value only) and
    plumbs `vertical_fov_deg` through.
  - Perception and the timing path are unchanged.
  - The pinned `navrl_task.py`, `navrl_perception.py`, `speed_governor.py`, `play_navrl.sh` and `runner.py` show 0 diff
    lines against `ab6531b`.
  - r1's consumer trace and timing trace therefore stand.
- **Transparency.** `full` equals the unwrapped module bit for bit, all perception keys and `governed` included, in all
  three of my camera-loss scenes, where the association is active. The test only checks this in `scene(11)`, where it
  never runs.
- **Isolation.** A repository-wide `git grep` at `842fcf5` finds `navrl_sensor_ablation`, `NAVRL_SENSOR_ABLATION`,
  `camonly_entry` and `camonly_python` only in the unit's own files. The new condition's name likewise appears only
  in the unit's files.
- **Are the new tests non-vacuous?** Yes:
  - **Association test:** it asserts active > 0 under `full`, camera loss, a change under perturbation, the removal
    asserted per condition, 0 where removed, and > 0 where kept.
  - **Same-bar test:** it checks a value inside riskcap's slope region.
  - **43/44° test:** the positive and negative sides are both covered.
  - **Refusal additions:** each has a control, and my no-op `_check_perception` mutant is caught.
  - **Quiet startup:** close to vacuous (L-r2-3).
  - **Gaps:** M-r2-1.
- **The seven mutants are each caught for the right reason** (my rerun of the harness is byte-identical to
  `gate_02/mutants.json`, SHA-256 `8a68412f…`):

  | Mutant | Caught by |
  | --- | --- |
  | `no_mask` | 17 subtests: leak, K1, association, obs_only, governor |
  | `governor_raw` | governor leak, and the K2 in-FOV clearance |
  | `no_k2` | K2, and the 43/44° test |
  | `fill_zero` | K1 (plus an incidental governor reversion, L-r2-5) |
  | `fov_shifted` | leak, K1, and the in-FOV obs_only test |
  | `assoc_raw` | only the 7 removing subtests of the new association test |
  | `no_elevation_fix` | the same-bar test, and the in-FOV clearance |

  The unmodified module passes.

### R3. Disclosures

- **L4 and L9:** accurate.
- **L1:** accurate, but omits the ego velocity (L-r2-4).
- **L2:** inaccurate for B_in_gL ("camera-only"), and not tied into the A − B_in_gL reading (L-r2-1).
- **L5:** incomplete (L-r2-2).

The C1/C2 preregistration must also carry:
- the corrected M1 residual (M-r2-2);
- the test gap or its fix (M-r2-1);
- the condition count and budget (L-r2-4);
- the absolute-path requirements (L-r2-2).

### R4. The gate

- **gate_02's C0_CONDITIONAL_PASS is computed as the plan fixes.**
  - Preflight: HEAD = `842fcf5…`, tree clean.
  - Host faults: 0 → 0.
  - Item 1: 14 tests OK.
  - Item 2: the unmodified module passes and all 7 mutants fail.
  - Item 3: the suite comparison below.
  - Item 4: the pinned diff is empty.
  - Item 5 is this review. CONDITIONAL was the correct label until now.
- **The suite comparison with the base is valid.**
  - The candidate ran 3,248 tests = 3,234 + 14, with skipped 17 = 17 and excluded 5 = 5.
  - The set of 6 failing tests is identical to the base. All 6 are environmental (missing untracked `results/`
    files, the BOIA inventory and ref5in fixtures), and none touches the unit's files.
  - Reusing the 17:38 base log is sound, because the base tree `ab6531b` is unchanged.
  - **Limits:**
    - the base log records no command or environment header;
    - a new failure inside one of the 6 already-failing tests would be masked;
    - the suite log lists no test names, so the +14 is matched by count, not by name.

## What I ran

Every Python process ran with `taskset -c 6,7,12,13 nice -n 10 env CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1
PYTHONDONTWRITEBYTECODE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B`, with `TMPDIR` set to `audit/tmp` and
one process at a time. Modules were loaded by path from the worktree, as the tests do.

There is one exception to the `-B` flag. The M3 probe runs bash `play_navrl.sh` (a byte copy, SHA-256 `9f300a20…`)
under the same `taskset`/`nice`. The interpreters it starts inherit the CPU affinity and `PYTHONDONTWRITEBYTECODE=1`,
`PYTHONNOUSERSITE=1` and `CUDA_VISIBLE_DEVICES=`, but are not given `-B`. They ran only my stub `runner.py` and stub
packages in `audit/m3/`.

| What | Output | Result |
| --- | --- | --- |
| Kernel fault count, before and after | `faults_before.txt`, `faults_after.txt` | 0 → 0 (18:16:41 / 18:23:07 → 18:34:36); offline CPUs 4-5 |
| `tests/test_navrl_sensor_ablation.py -v` | `rerun_c0_tests.log` | 14 OK |
| `tests/camonly_c0_mutants.py` | `rerun_mutants.json`, `rerun_mutants.log` | pass; byte-identical to `gate_02` |
| `probe_b1_assoc.py` | `probe_b1_assoc.json`, `.log` | B1 closed (own scenes, 4 fill variants, K1, `full` transparency, own `assoc_raw`) |
| `probe_m1_governor.py` | `probe_m1_governor.json`, `.log` | M1 closed (630 unit and 99 end-to-end cases) |
| `probe_m2_obsonly.py` | `probe_m2_obsonly.json`, `.log` | M2 as described; L-r2-1 numbers; the test scenes' counts |
| `m3/probe_path.sh` and a relative-`PYTHON` run | `probe_path.log` | M3 closed; L-r2-2 |
| `probe_extra_mutants.py` (13 mutants) | `probe_extra_mutants.json`, `.log` | M-r2-1 (3 escape); others caught or harmless |
| `probe_entry_mutants.py` (copies in `entry_mut/`) | `entry_mut/probe_entry_mutants.json`, `probe_entry_mutants.log` | L-r2-3 |
| `probe_camera_geometry.py` | `probe_camera_geometry.json`, `.log` | M-r2-2 |

**Disclosures about the probes:**
- **`probe_m1_governor.py`, run 1** failed at install. My clearance recorder lacked `functools.wraps`; this is the
  `KeyError` noted under Information.
- **Run 2:**
  - its part-B bar was one column wide, which perception's column map misses at 0°. That was my scene's error; I
    widened the bar to 3 columns.
  - its part-A criterion was too strict for geometries with no row near 0°. I revised it to "equal to a LiDAR seeing
    the same bar; equal to c for the production geometry". The revision is commented in the script.
- **Run 4 is the one reported.**
- **One cleanup was blocked:** a request to remove the (already empty) `audit/tmp/` contents. Nothing else was removed.

**SHA-256** of the main outputs:

| File | SHA-256 |
| --- | --- |
| `probe_b1_assoc.json` | `64839c021a91a2bc2d79240a8632d5fbf8b8052856d714d9fade36f14e597d74` |
| `probe_m1_governor.json` | `4a6331ec5b208512428efcd4db9b222334d5f0f4088cb6ee354d2e7ff221e1ae` |
| `probe_m2_obsonly.json` | `736d33c922d6173ec9db10411c1653ee545a192626118505acd4a1087501dc76` |
| `probe_path.log` | `e7cb85e7654002a92b4eb6fb14bbc8a1fd88a7b40b1a9a68da396e359a1eb0bd` |
| `probe_extra_mutants.json` | `c2c8c1fb14458c42807620a592b896edd2c29855ea293b00516fb4f70ab634bd` |
| `entry_mut/probe_entry_mutants.json` | `7c59391b9083bb7062630b5ebe03e3bcc278152145ea42cdf41620ec1699a11d` |
| `probe_camera_geometry.json` | `c7fa96d8a8d1939ecdc6e3c7d103a7b0b31cf52c77d30fc60ddcf62cdd29cf16` |
| `rerun_mutants.json` | `8a68412f9389bd123ffd201d6f71b90899142ecd09a59b4924728b247ac06113` |
