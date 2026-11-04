# Independent review of C0: the opt-in LiDAR ablation switch of the camera-only study

- **Reviewer:** Claude Code fresh-context review agent. **Model:** Opus 5.5 (`claude-opus-5-5`).
- **Date:** 2026-11-04, 17:48-18:04 KST.
- **Reviewed:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `4d03f35`
  (clean tree before and after). The unit is `git diff ab6531b 4d03f35`.
- **Mode:** read-only. I wrote only to this directory. I started no GPU process, made no commit and touched no
  other process, worktree or `results/` directory.
- **Basis:** the code, the diff, the plan and the gate outputs. I did not use the WORKLOG or the author's summaries.

## Verdict: **FAIL**

There is one blocking finding, **B1**. The ablation code itself is correct in every channel I could exercise, and the
probes confirm it. But the C0 gate claims leak checks with positive controls for "the association state", and that
check is vacuous:
- the test scene never activates the LiDAR target association;
- a known-bad module whose association reads the raw LiDAR passes every C0 test.

AGENTS.md says "pair every negative test with a positive control and assert a non-zero count". The plan's C0 gate
item 1 makes the same demand. This check does neither.

The fix is test-only and small (see B1). After it, C0 can be labelled PASS on re-review. The medium findings M1-M3 do
not block C0, which is about the switch on CPU tensors. But M1 and M2 must be fixed or disclosed in the C2
preregistration, and M3 must be fixed before C1.

## Findings

### Blocking

**B1 (HIGH, blocking): the leak check for association, visibility, confidence and tracker is vacuous. A mutant whose
association reads the raw LiDAR is not caught.**
- **Where:**
  - `tests/test_navrl_sensor_ablation.py:97-125` (`scene`);
  - `tests/test_navrl_sensor_ablation.py:274-289` (`test_leak_and_positive_controls`);
  - `tests/camonly_c0_mutants.py:81-82` (mutant list).
- **Why the association never runs:**
  - In the test scene the camera sees the target in every frame of envs 0-1, and envs 2-3 never have a track.
  - `_associate_lidar_target` is valid only when `~camera_visible` (`navrl_perception.py:2087`), so it is never
    active.
  - The positive control only asserts that `obs` and `governed` change under `full`. Both change through the static
    scan and the governor, not through the association.
- **Measured** (`probe_assoc.py`, `probe_assoc.json`):
  - The test's own scene under `full`: `lidar_visible` = 0 of 24 env-steps, `camera_visible` = 12, track active = 12.
    The association-leak check has no non-zero count.
  - Mutant `assoc_raw` (perception masked for fusion only; `_associate_lidar_target` gets
    `self._ablation_raw_lidar_m`): the official `SensorAblationC0` suite runs 9 tests with **0 failures, 0 errors**.
  - The implementation is correct where it can be checked. I used a camera-loss scene: the blob is removed from step
    3 on, and the LiDAR keeps a return on the target surface at about 0°.
    - Under `full`, `lidar_visible` = 6 env-steps, and perturbing the LiDAR changes visibility, confidence and
      tracker state (positive control).
    - Under `no_lidar_in_fov`, `no_lidar` and `no_lidar_obs_only`, `lidar_visible` = 0, and visibility, confidence,
      tracker state and covariance stay bit-identical under the same perturbation.
    - In this scene the `assoc_raw` mutant is caught: the association outputs and `obs` both differ.
- **Fix:**
  1. Add a camera-loss scene to `test_leak_and_positive_controls`, or as a separate test. Assert that
     `diag["lidar_visible"].sum() > 0` under `full`, that the association outputs change under `full`, and that they
     are bit-identical in each ablated condition.
  2. Add `assoc_raw` as a sixth mutant in `tests/camonly_c0_mutants.py`.
  3. Preferably also cover an out-of-FOV association case for `no_lidar_out_fov`: a target that leaves the FOV, held
     by the LiDAR.

### Medium (not blocking C0; must be resolved or disclosed before C2 or C1 as stated)

**M1: the ablated governor shrinks camera-fused clearances by cos 20° (×0.94). This is a projection artifact, not
camera information.**
- **Where:** `navrl_sensor_ablation.py:133-134`, together with `speed_governor.py:230-232`.
- **Mechanism:**
  - Perception replicates one camera-column value over all 4 LiDAR rows (`navrl_perception.py:1920-1921`).
  - The governor treats each row as a slant range at elevation +20, +10, 0 or -10°, and takes the minimum of
    `range·cos(el)`.
  - A camera value c therefore becomes a horizontal clearance of 0.94·c. A LiDAR return from the same bar,
    d/cos(el) per row, projects back to d.
- **Measured** (`probe_governor.json`; a bar 4.0 m ahead seen by both sensors; forward command 3.5 m/s):

  | Condition | Clearance (m) | Executed speed (m/s) |
  | --- | --- | --- |
  | `full` | 3.985 | 2.756 |
  | `no_lidar_in_fov`, `no_lidar` | 3.745 | 2.572 |

  3.745 = 3.985 × cos 20°.
- **Effect:** a systematic speed reduction inside riskcap's 3-5 m release band in B_in and C. It enters A − B_in and
  C_gL − C as if it were a sensor effect.
- **Fix, either:**
  - place the camera-fused value only in the 0° row (index 2) and set the other rows to no return; or
  - divide by cos(el) before the projection.

  Otherwise disclose and quantify the artifact in the C2 preregistration.

**M2: "K5 revised" is incorrect for the governor and the target association. A − B_in is not only "the cost of the
camera's misses".**
- **Where:** `docs/plans/camonly_study_plan_v1.md`, section "K5 revised"; its table and the "A − B_in" reading.
- **What holds:** for the observation (static scan and tokens), min-fusion indeed keeps camera phantoms with or
  without the LiDAR.
- **What does not hold:**
  - **The governor.** In A, the production governor reads only the raw LiDAR (`navrl_task.py:5260-5265`), so it is
    immune to camera phantoms. In B_in it reads the camera-fused value in in-FOV bins, phantoms included.
    - Measured (`probe_governor.json`): a 3.5 m camera phantom with a clear LiDAR gives 3.5 m/s in `full` and
      2.21 m/s in `no_lidar_in_fov` and `no_lidar`.
    - So the comparison does credit the in-FOV LiDAR with removing phantoms, through the governor. M1 adds to this.
  - **The target association.** The in-FOV LiDAR also holds the target track when the camera loses it inside the FOV
    (occlusion, detector miss, latency backup). That is target-tracking value, not obstacle-depth misses.
- **What the bearing split can conclude:** the total value of the in-FOV LiDAR against the out-of-FOV LiDAR, for F
  with this governor, zero-shot. The in-FOV part combines:
  - observation misses (resolution, range 10-12 m);
  - in-FOV target association;
  - the governor's phantom immunity, its misses and M1.
- **What it cannot conclude:**
  - camera resolution or range by itself;
  - observation against governor inside B_in (there is no B_in arm with a raw-LiDAR governor);
  - what a better camera would give;
  - a clean interaction term, because B_out and C are out of distribution for F and the closed loop is nonlinear.
- **Fix:**
  - add `no_lidar_in_fov_obs_only` (and optionally `no_lidar_out_fov_obs_only`) as conditions; or
  - restate the reading of A − B_in in the C2 preregistration.

  Rely on the planned CPU sensor-level miss and phantom count for the camera-quality question.

**M3 (C1 hazard): with `PYTHON=tools/camonly_python`, `play_navrl.sh` no longer puts the interpreter's bin on `PATH`,
and the Isaac Gym gymtorch build step then fails.**
- **Where:** `play_navrl.sh:39-42`; `tools/camonly_python:6-7`.
- **Mechanism:**
  - `play_navrl.sh` prepends `dirname "$PYTHON"`. With the wrapper that is `<worktree>/tools`, not
    `.../envs/aerialgym/bin`.
  - `ninja` exists only in the aerialgym env bin.
  - `isaacgym/gymtorch.py:45` calls `torch.utils.cpp_extension.load`. In a fresh process this always reaches
    `_write_ninja_file_and_build_library`, and so `verify_ninja_availability()` (torch 2.4.1 `_jit_compile`), even
    when the `.so` is cached.
- **Measured** (`probe_path.log`): with the current shell PATH, or with `/usr/bin:/bin`, the real interpreter finds
  ninja and the check passes; `camonly_python` gets `ninja=NOT_FOUND`, and the check raises "Ninja is required to load
  C++ extensions".
- **Effect:** loud, not silent. The run dies at the first `aerial_gym` import, before the receipt is written, unless
  the caller's PATH already contains the env bin. Either way the environment differs from the native run.
- **Fix:** in `tools/camonly_python`, add `export PATH="$(dirname "${REAL}"):${PATH}"` before `exec`. C1 should
  check this.

### Low

- **L1: the observation in C_gL is not strictly LiDAR-free.**
  - Where: `navrl_task.py:5057`; `navrl_sensor_ablation.py:163-165`.
  - The robot history's previous action is the governed (executed) command, `prev_action[:, 0:2] =
    _last_governed_action_xy`. In `no_lidar_obs_only` that command depends on raw-LiDAR clearance whenever the cap
    binds above 2.0 m/s. The ego velocity carries the same.
  - This is inherent to K3. The C2 preregistration should say "camera-only observation except the executed-command
    feedback".
- **L2: the target mask of C_gL uses camera-only visibility.**
  - Where: `navrl_sensor_ablation.py:104-115`.
  - The replica is computed with the fusion call's `visible`, `target_surface_range` and `target_bearing`. In C_gL
    these come from the camera-only track.
  - So when the camera has lost the target, the raw-LiDAR governor treats the target's return as an obstacle. In A,
    the LiDAR association would have carved it out. C_gL − C therefore contains this mask difference. This is a
    defensible choice; disclose it.
- **L3: the entry-point test asserts an import order that holds only for its stub.**
  - Where: `tests/test_navrl_sensor_ablation.py:405-449`; `tools/camonly_entry.py:5-6`.
  - The stub's `aerial_gym/__init__.py` is empty. The real one (`aerial_gym/__init__.py:2`) imports isaacgym and
    every task at the first `aerial_gym.*` import, which is the origin-guard import. The real order is therefore
    isaacgym, then the tasks, then the origin check, then install.
  - That is the same order as native `runner.py`, whose first import (`quiet_rl_io`) has the same effect. Behaviour
    matches, but the docstring's "import-origin guard first" and the test's `["origin", "isaacgym", ...]` describe
    the stub. My stub probe, with an `aerial_gym/__init__` that imports isaacgym, shows
    `isaacgym, aerial_gym/__init__, origin, install`.
  - The test does not cover the quiet-startup branch, the `camonly_python` wrapper (`CAMONLY_REAL_PYTHON`,
    `readlink`) or `play_navrl.sh`. Those belong to C1, as the plan says.
- **L4: `__file__` and `argv[0]` of runner become absolute.** Native Python 3.8 gives `"runner.py"`.
  - `runner.py` uses only `"--play" in sys.argv` (`:74`) and `Path(__file__).resolve()` in the train branch
    (`:1044`), so there is no effect.
  - The rest matches the native run: `__name__ == "__main__"`, `sys.path[0]` (the rl_games directory),
    `__spec__ = None`, and `sys.modules["__main__"]` being the runner's globals during the run (stub probe).
  - Nothing in `aerial_gym` or `motar_eval` imports a top-level module from the rl_games or tools directory, and no
    name in `tools/` shadows an installed module.
- **L5: receipt and opt-in hazards for the launcher copy (C1/C2).**
  - Where: `tools/camonly_entry.py:39-43`; `tools/gr_run.py:102`; `eval_navrl_target_behavior_arms.sh:169-202`.
  - `NAVRL_SENSOR_ABLATION_RECEIPT` is one path per process, opened with `"x"`. A multi-cell launcher needs one per
    cell; reuse fails loudly.
  - `NAVRL_SENSOR_ABLATION` set without the entry point is silently ignored by production code. The launcher should
    require the receipt or the `[camonly]` line for every cell.
  - `gr_run`-style launchers drop inherited `NAVRL_*` variables, so the condition must be set per cell.
  - The pinned launcher calls `${PYTHON} -c ...` and `${PYTHON} - ...` helpers. `PYTHON=camonly_python` must
    therefore be scoped to the `play_navrl.sh` call only, as the wrapper's comment says; otherwise `camonly_entry`
    would treat `-c` as a script.
- **L6: refusal and boundary coverage is thin.**
  - Where: `tests/test_navrl_sensor_ablation.py:235`, `:370-383`.
  - Not tested: the lineage-flag refusal (`_check_perception`), `mode != riskcap`, and the guard "governor ran before
    perception observed".
  - The K2 bearing threshold is not probed between 40° and 60°: any threshold in (40°, 60°) passes. The `fov_shifted`
    mutant shifts only the bin mask.
- **L7: the test camera differs from production.**
  - Where: `tests/test_navrl_sensor_ablation.py:86-88`.
  - The test camera namespace has no `camera_obstacle_max_range`, so it defaults to 12 m (`navrl_perception.py:751-753`).
    Its 10 m background is therefore a valid obstacle reading: the in-FOV "phantom 10 m wall" that production (10 m)
    maps to no return.
  - So `camera_no_return_to_lidar_range` (in-FOV removed bins becoming 12 m in open space) is not exercised. This is
    not a vacuity problem for the checks that exist, but the production geometry is not covered.
- **L8: the isolation test cannot see every kind of reference.**
  - `test_no_production_import` (`:385-398`) matches import statements in `*.py` only. It would not see
    `importlib.import_module("...")` or a shell launcher.
  - My repository-wide `git grep` at `4d03f35` found no reference to `navrl_sensor_ablation`,
    `NAVRL_SENSOR_ABLATION`, `camonly_entry` or `camonly_python` outside the unit's own files.
- **L9: joint telemetry mixes clearances under ablation.**
  - Where: `navrl_task.py:5038-5049`.
  - The `requested_direction_clearance` field is the ablated governor clearance, while the actual and executed
    clearances come from `_diagnostic_directional_clearance` on the raw LiDAR (outside the wrapper's active window).
  - This is recorder-only, but a C2 analysis using these fields must know it.

## Answers to the questions

**Q1. Completeness: is every online LiDAR consumer cut in the removed bins? Yes.**
- I traced every read of `depth_range_pixels`, `_lidar_distance_m()`, `lidar_m`, `last_scan_nearest` and
  `last_target_like` in `navrl_task.py`, `navrl_perception.py`, `kroc_eval_adapter.py` and `motar_eval`.
- **Paths that reach the actor or the command:**
  - **Perception.** `lidar_m` enters perception only through `observe`. Its two readers, `_fuse_static_and_extract_obstacles`
    and `_associate_lidar_target`, receive the masked tensor (`navrl_sensor_ablation.py:86-97`). This covers the
    static scan, the obstacle tokens, corridor tokens (from `last_scan_nearest`, which is masked), the association,
    the tracker correction, the LiDAR confidence field and fused visibility. The detector, latency path and search
    state read no LiDAR.
  - **Governor.** It reads the raw `depth_range_pixels` (`navrl_task.py:5260-5265`) and is redirected through the
    wrapped `directional_lidar_clearance` (`:5332`). Every other governor feature raises.
  - **KROC B1 supervisor (opt-in).** It reads `_visible_now`, `tracker.active` and `last_scan_nearest`, all from the
    ablated perception.
  - **Previous action.** It carries the governor's output: camera-only in A-C, raw LiDAR in C_gL (L1).
- **Paths that keep the LiDAR (boundary):**
  - the r_ss reward (`:6306-6319`);
  - the legacy and vision observation modes (not used by F);
  - the viewer (`:2969`);
  - the action diagnostics (`:5518`);
  - joint telemetry (`:5402`, L9);
  - BOIA access (`motar_eval/boia/access.py:73`, opt-in shadow).

  Terminations are physics contacts and height/out-of-bounds tests (`:6071-6074`). The module docstring
  (`navrl_sensor_ablation.py:22-23`) and the plan's "Definition" section state this boundary, and the code respects
  it.

**Q2. Transparency and OFF: yes, with B1's caveat on coverage.**
- **`full` is transparent:**
  - no bin is removed, so `observe` passes `lidar_m` unchanged;
  - the fusion wrapper only adds attributes;
  - the governor's `directional_lidar_clearance` calls the original when the condition is `full`;
  - no randomness is drawn and no tensor is modified in place.
- The test `test_full_is_transparent` passes, and I re-ran it.
- **OFF:** without the entry point nothing imports the module (`git grep`), and the pinned files, `play_navrl.sh`,
  `runner.py` and `speed_governor.py` are unchanged against `ab6531b`.

**Q3. Semantics: implemented as the plan says, and the timing matches production.**
- **K1:** removed bins become `lidar_max_range` in all 4 rows. The in-FOV result is the camera value and the
  out-of-FOV result is 1.0 (`test_removal_equals_lidar_returning_nothing`).
- **Governor scan:** kept bins are raw; removed in-FOV bins are `_ablation_fused_m`, the post-fusion camera value;
  removed out-of-FOV bins are 12 m (`:125-137`).
- **K2:** clearance 0 for moving out-of-FOV commands only when `out_fov` is removed (`:168-170`), so riskcap gives
  exactly 2.0 m/s. The FOV test (|bearing| > hfov/2) matches the fusion mask.
- **C_gL:** it uses the raw scan with a replica of the historical target mask, recomputed on the raw LiDAR.
- **Timing:** the render at the end of step t−1 (`navrl_task.py:5168`, plus the re-render after resets at `:5175`)
  is followed by `get_return_tuple`, then `observe`. Step t's governor (`:4967`) reads the same `depth_range_pixels`
  and the `_ablation_*` tensors from that observe. No render happens between them; `reset()` renders before its
  observe (`:4698`).
- **Sensibility:** M1 (the cos 20° artifact), M2 (the governor's phantom immunity in A), L1 (executed-command
  feedback in C_gL) and L2 (C_gL's target mask) are the places where the comparison is confounded, or where LiDAR
  information remains.

**Q4. Tests.**
- **Non-vacuous:**
  - the static scan, tokens and governor leak with their controls;
  - the kept-bin control;
  - K1 equality;
  - the replica (hits > 0);
  - K2 with its open-LiDAR control;
  - `obs_only`.
- **Vacuous:** the association and visibility part (B1).
- **Mutants:** the five reproduce. I re-ran the harness: `pass true`, the unmodified module passes, and each mutant
  fails. Missing ones, by importance:
  1. association on the raw LiDAR (B1; escapes);
  2. a K2 threshold error inside (40°, 60°) (L6);
  3. a governor target mask taken from the raw rather than the masked LiDAR.
- **Suite comparison:** it is valid as a replacement for "0 failures" here.
  - The base and candidate logs have the same 6 failing test ids, with identical failure messages apart from
    temporary-directory names. All 6 are environmental: two missing untracked `results/` files, a missing
    `/tmp/codex_occ`, and a missing ref5in checkpoint.
  - The candidate log has exactly 10 more tests (3,244 against 3,234), the 10 C0 tests, and none of them fails.
  - Limit: a new failure inside one of the 6 already-failing tests would be masked. None of the 6 touches the unit's
    files.
- **Entry-point test:** it is meaningful for `__name__`, `argv`, `sys.path[0]` and receipt refusal. It is not
  meaningful for the real import order (L3).

**Q5. Entry point under Isaac Gym.**
- **Same as `python runner.py`:**
  - the import order (it matches native, because `aerial_gym/__init__` imports isaacgym first in both flows);
  - `__name__`, `sys.path[0]`, `__spec__`, `sys.modules["__main__"]` and `SystemExit` propagation;
  - quiet startup (`install()` imports modules that `aerial_gym/__init__` has already imported, so no extra output
    except the `[camonly]` line);
  - the origin guard (idempotent).
- **The receipt:** it records the condition, the module path, its SHA-256 and the `__file__` of the perception and
  task modules, and refuses to overwrite.
- **Not the same:** `PATH` (M3), which makes the gymtorch build step fail unless the caller's PATH has the env bin.
  `__file__` and `argv[0]` are absolute (L4; harmless).
- **Nothing in `runner.py` depends on being the real `__main__` file.** C1 remains the only test of the simulator
  path.

**Q6. Plan.**
- K5 revised is correct for the observation and wrong for the governor and the in-FOV association (M2).
- The bearing split measures in-FOV against out-of-FOV LiDAR value as a zero-shot diagnostic. On its own it does not
  measure camera resolution.

## What I ran

All runs were CPU-only, with `taskset -c 6,7,12,13 nice -n 10 env CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1
PYTHONDONTWRITEBYTECODE=1 .../aerialgym/bin/python -B`, `TMPDIR` inside this directory, at most one process at a
time, each run lasting seconds.

| What | Output | Result |
| --- | --- | --- |
| Kernel fault count, before and after | `faults_before.txt`, `faults_after.txt` | 0 → 0 (17:48:41 → 18:03:52); offline CPUs 4-5 |
| `tests/test_navrl_sensor_ablation.py -v` | `rerun_c0_tests.log` | 10 tests OK |
| `tests/camonly_c0_mutants.py` | `rerun_mutants.json`, `rerun_mutants.log` | pass true; reproduces `gate_01` |
| `probe_assoc.py` | `probe_assoc.json`, `probe_assoc.log` | B1 |
| `probe_governor.py` | `probe_governor.json`, `probe_governor.log` | M1, M2 |
| `probe_path.sh` | `probe_path.log` | M3 |
| `entry_stub/`: native `runner.py` against `tools/camonly_entry.py` on stub packages | console, quoted in L3 and L4 | normal, quiet and no-condition runs |

I also compared the base and candidate suite failure messages (`public_cpu_base_02.log` against
`gate_01/public_cpu.log`), and read `gate_01/c0_gate.json`.

**Disclosure.** Around 17:55 I ran two one-line lookups without the CPU prefix:
- `python -c "import isaacgym"`, which loads `gymapi` only, to find the package path;
- `python -c "import torch; print(torch.__version__)"`.

Neither created a CUDA context or a simulation, and each ended in about a second. The gymtorch extension directory's
17:55:00 timestamp comes from the other unit's evaluation process, which started at 17:55:02; `gymapi` does not touch
it.

**SHA-256** of the main outputs:

| File | SHA-256 |
| --- | --- |
| `probe_assoc.json` | `a6cb8cc8145c1dc7a12148668daf50ec95730b36b5fe883639dfa1c4bdcd6efa` |
| `probe_governor.json` | `5f6fbc4c8e67e489d543d33d2db345f3fb3b88faf1fe4134e38063300185d82f` |
| `probe_path.log` | `b2ecc7f5f378f558b62b917a634ce81e6b44bf20abd0a5862b5bc16d493dd970` |
| `rerun_mutants.json` | `fcfe65c34234c2bc84a91a114ccf41765789dd8400ac8855d662bd8cba73b15b` |
