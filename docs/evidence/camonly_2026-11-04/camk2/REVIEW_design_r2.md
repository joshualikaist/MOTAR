# CAM-K2 v1: design review r2 (a focused check of the r1 repairs)

- **Reviewer:** Claude Opus 5.5 (fresh context), 2026-11-08, about 14:03–14:20 KST. Read-only on every repository.
  I wrote only to this folder.
- **Worktree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-realveh`, branch `realveh/dev`, HEAD `814c28a`. The
  repaired files are uncommitted. The unit is meant to be committed and run from `MOTAR-camonly`, branch `camonly/dev`,
  HEAD `971ae67` plus later commits.
- **Reviewed bytes:** SHA-256 prefixes, all equal to IMPLEMENTATION.md Part 4:
  - `navrl_vehicle_rand.py` `6ac0667f…`, `navrl_depth_model.py` `08b503e1…`;
  - `realveh_entry.py` `b153057e…`, `realveh_python` `8936f9f8…` (unchanged since r1);
  - `camk2_train.py` `7c7eb34c…`, `train_navrl_camk2.sh` `2ff0494b…`;
  - `test_realveh.py` `7f65032c…`, `test_camk2_train.py` `0e6c6307…`;
  - `test_navrl_sensor_ablation.py` `891c87fa…` (the `git diff` is +12 −4);
  - the prereg `e2ea1dcb…`, byte-identical to `~/motar_tmp/camk2_20261108/PREREG_DRAFT.md`.
- **Copies of the r1 bytes do not exist**, so I could not diff against them. I read the changed regions and the
  code they depend on: quiet_rl_io, runner.py, rl_games a2c_common, robot_manager, env_manager, pretty_train_stats,
  and navrl_task's construction, reset and observe.
- **Files here:**
  - this report;
  - `tests_run.log`;
  - `probe_quiet_stderr.py` and its output `.json`.
- **CPU windows** (cores 6 and 7, `nice -n 19`, `ulimit -v 8000000`, `CUDA_VISIBLE_DEVICES=""`, no writes to any
  tree). They ran while CAM-K1-E's evaluation (`camk1e_run.py eval`, from MOTAR-camonly) was active; the T-LOAD rule
  in AGENTS.md and the CAM-K1-E preregistration allow this on cores 6 and 7. Record the windows in the WORKLOG:
  - 14:10:08–14:10:09: the probe;
  - about 14:11: a stdlib regex pass over the CAM-K1 logs;
  - 14:12:18–14:12:35: the three test files.

## Verdict: **AMEND_FIRST**

- **One new MAJOR (N-1).** In every real training run, one of the telemetry lines that the wrapper now requires is
  written to /dev/null. A valid smoke therefore reads TRAINING_FAIL, deterministically, and the unit stops for repair
  after the preregistration has been committed. The fix is small and CPU-testable. Make it before the commit.
- **Every r1 finding is resolved or disclosed,** apart from:
  - m-2: partly. The design is right, but N-1 breaks it;
  - m-6: open by design, for the committer.
- **What else holds:**
  - the f·B change is consistent everywhere;
  - the import guard is not weakened;
  - the BRANCH and path changes fit MOTAR-camonly;
  - the step-time gate and the manifest are sound;
  - the prereg matches the code, apart from LOW items.
- **Re-review scope:** after N-1 is fixed, a check of that diff and its new test is enough.

## Tests run (prescribed command form)

| File | Result |
| --- | --- |
| `test_realveh.py` | 43 OK |
| `test_camk2_train.py` | 31 OK, none skipped (the CAM-K1 copies are present) |
| `test_navrl_sensor_ablation.py` | 15 OK |

After the runs, `git status` was unchanged.

## Status of the r1 findings

| r1 | Status | Evidence |
| --- | --- | --- |
| **M-1** altitude sag | **Resolved** | The prereg's "Disclosed limits" quantify the sag. The table matches `probe_altitude_sag.json`: I checked the new "uncapped" row, 0.80–0.98 m. Also present: the integral reset, the overwritten actor z, the PX4 note, and that CAM-K2-E must read `below` in this light. `crash_causes` is descriptive. I recomputed CAM-K1's last-10 shares from its logs: s1 0.9980 / 0.0013 / 0 / 0.0009 and s2 0.9945 / 0.0055 / 0 / 0.0001 (bar / below / above / OOB). They equal the prereg's. See N-5 for the decision's attribution |
| m-1 depth reach | Resolved | The visibility-bonus input, the `no_lidar` riskcap clearance, the tracker, and bars under MinZ vanishing from the filter are all disclosed |
| **m-2** runtime evidence | **Partly** | The lines, their thresholds and the tests are as recommended. But the vehicle-randomization `init` line cannot reach the log in a real run (N-1) |
| m-3 step cost | Resolved | `smoke_rate_problems` gates seeds at 1000 × last × 1.25 ≤ 9000 s; the boundaries are tested; a timeout reads TRAINING_FAIL. See N-3 for what "step time" measures |
| m-4 depth mode | Resolved | The mode is named. f·B = 882.5 × 0.075 = 66.1875 is exact in binary64 and identical in the module docstring, `DEPTH_MODEL`, SCOPE, the launcher export and the prereg. The MinZ arithmetic is right: 66.1875 / 190 = 0.348 m and / 95 = 0.697 m. The σ_z table is right (2.27 mm, 3.63 cm, 0.227 m, 0.734 m, 0.906 m). The generic unit tests in `test_realveh.py` still use 57.2 (lines 311, 821, 953), which is harmless; `test_camk2_train.py` pins 66.1875 end to end, from the launcher through the entry to the expected settings |
| m-5 import guard | Resolved | See "Import guard" below |
| m-6 before commit | **Open** (as disclosed) | Still to do: the PUBLIC_CPU suite, the `docs/EXPERIMENTS.md` entry and the clean tree. The launcher preflight was run in MOTAR-realveh only. See "Sequencing" |
| m-7 weak, bundled source | Resolved | Stated in "Why" and "What does not follow"; unbundling is the user's first decision |
| L-1 | Resolved | Conditions listed in the prereg and the module docstring; "equivalent … under these conditions" |
| L-2 | Resolved (disclosed) | |
| L-3 | Resolved | I re-derived the f·B 66.1875 numbers analytically: bias × 57.2/66.1875 = × 0.864; drop 0.34 % / 11.1 % / 28.1 %. They match. The source file is not durable (N-4) |
| L-4 | Resolved | −0.086 / −0.179 / −0.302 / −0.448 m, matching the prereg's rounding |
| L-5 | Resolved | |
| L-6 | Resolved, with a LOW (N-6) | `write_manifest` binds the checkpoint, source, receipt and record SHA-256, the treatments, the commit and the prereg. Opened with mode `x`; its SHA-256 is in the report |
| L-7 | Resolved | Stated in the prereg |
| L-8 | Resolved | Disclosed. 2.2 GB free now, against about 0.45 GB needed |
| L-9 | Resolved | The partial reset leaves the other envs untouched (`test_realveh.py:674`); the RNG probe's positive control (`:345-346`, `:870`); the two-way launcher contract with a positive control (`test_camk2_train.py:148-159`) |
| L-10 | Resolved (disclosed) | |

## New findings

### MAJOR

**N-1. The `[veh-rand] init` line goes to /dev/null in every quiet training run, so a valid smoke reads TRAINING_FAIL.**
- **The mechanism:**
  - In training, `runner.py:1082-1083` applies `patch_rlgames_quiet_train_init()`. `AERIAL_RL_QUIET_STARTUP` defaults
    to on and no launcher sets it.
  - The patched `run_train` builds the agent inside `devnull_stdio()` (`quiet_rl_io.py:115-116`). That context swaps
    `sys.stdout` and `sys.stderr` for /dev/null (`:147-156`).
  - Building the agent creates the env: `a2c_common.py:132` `create_vec_env` → `runner.py:121` `env_creator` →
    NavRLTask → `env_manager.py:309` → `robot_manager.py:128` `robot.init_tensors`.
  - The init_tensors wrapper calls `initialize()`, which calls `log("init", …)` (`navrl_vehicle_rand.py:142-146`).
  - `log` writes to `sys.stderr` at call time (`:69-71`), which is /dev/null at that moment.
  - Logging handlers keep the stream they were created with. That is why the task's `hfov=66deg` line and
    `NavRL crashdiag` reach the log, and why this was not visible in CAM-K1.
- **The other lines are unaffected, from the code:**
  - both `install` lines are written by the entry, before runner.py;
  - the first reset and the first observe happen in `agent.train()` → `env_reset()`, outside the context;
  - `set_env_state` (`_restore`) neither resets nor observes;
  - the record file is written to a file, so it survives too.
- **The effect:** `telemetry_problems` requires an init line (`camk2_train.py:298-299`). The smoke would read
  TRAINING_FAIL with "veh-rand: no init line". Per the prereg, that stops the unit. A code fix after the commit then
  needs a dated amendment, a new commit and a new approval.
- **The verdict also depends on an unstripped variable:** `clean_env()` strips `NAVRL_`/`CAMK*`/… but not `AERIAL_`.
  With `AERIAL_RL_QUIET_STARTUP=0` in the caller's environment, the line would appear.
- **Evidence:** `probe_quiet_stderr.py` and `.json`. They use the test's own `real_telemetry()` and wrap only
  `VR.initialize` in the real `quiet_rl_io.devnull_stdio()`:
  - control: install, init and 3 resets, and no problems;
  - as in training: install and 3 resets, and the problem `["veh-rand: no init line"]`;
  - a logger line written inside the same context survives.
- **The tests miss it:** the fixture captures with `contextlib.redirect_stderr`. That is the same mechanism
  `devnull_stdio` uses, so the fixture never models the quiet construction.
- **Fix (any one), plus a test:**
  - (a) Bind the stream at install time, as a logging handler does: `install()` stores `sys.stderr`, and `log()`
    writes to the stored stream. The entry installs outside the quiet context, and the tests keep working because
    they install inside `redirect_stderr`.
  - (b) Write to `sys.__stderr__`. The test fixtures would then need a child process.
  - (c) Drop the init-line requirement. Put the first draw's factor and τ range into the record file and check it in
    `veh_rand_record_problems`.
  - **Add a CPU test** that calls `initialize` (or the init_tensors wrapper) inside the real `quiet_rl_io.devnull_stdio()`
    and asserts that the wrapper still reads no problem. Apply the same binding to `navrl_depth_model.log`: it is
    safe as built, but only by call order.
  - **Correct the docstrings** that claim "the quiet print filter does not touch stderr" (`navrl_vehicle_rand.py:44`,
    `navrl_depth_model.py:34`). The print filter does not touch stderr, but quiet startup does.

### LOW

- **N-2. A stale docstring.** `camk2_train.py:13` still says "branch realveh/dev". `BRANCH` (`:58`), the test and the
  prereg say `camonly/dev`.
- **N-3. "Step time" is rl_games' env-stepping time, not the epoch's wall time.**
  - It is `step_time` from `a2c_common.py:783-803` (env.step durations only), boxed by `pretty_train_stats.py:92`.
    It excludes policy inference and the PPO update.
  - For CAM-K1: wall / 3,000 = 4.83 s per epoch, against a mean step time of 4.43 s. That is about 9 %, which the
    1.25 margin covers: at the 7.2 s boundary the estimate is about 7,970 s, still under the 9,000 s cap.
  - Say so in the prereg's m-3 text.
  - Also, the prereg lists the per-epoch step times as "descriptive, not gates". But `post_run_problems` requires at
    least one step-time line for a seed (`camk2_train.py:517-518`). Either mention it under TRAINED or drop it.
- **N-4. The f·B 66.1875 probe rerun is not durable.** The prereg cites it (L-3, L-4), but per IMPLEMENTATION.md it
  is only in the implementer's session scratchpad. Copy `probe_depth_effects_fb66.json`, and the script, to
  `~/motar_tmp/camk2_20261108/`. My analytic re-derivation agrees with its numbers.
- **N-5. "TW_SCALE stays 0.15 (decision 2026-11-08)" does not say who decided.** IMPLEMENTATION.md Part 4 says the
  repairs follow the coordinator's decisions, and r1 left this as an optional user decision. Name the decider, or add
  it to "Decisions for the user before commit".
- **N-6. The manifest carries no verdict.** It is written for any run that produced a final checkpoint, including
  TRAINING_FAIL (`camk2_train.py:501`). Have CAM-K2-E bind the manifest together with `train_report.json`
  verdict TRAINED, and say so in the prereg. Alternatively, write the manifest after the verdict, with the verdict in
  it.
- **N-7 (optional). Nothing shows that the motor-update wrapper ran.** The init and reset lines show the draws, and
  the record shows the cap. An integer counter of `update_motor_thrusts` calls in the reset line (CPU, no
  synchronisation) would show that the cap and factors were applied.

## Specific checks requested

- **The new stderr lines and their requirement (valid run fails / invalid run passes):**
  - **Valid run fails:** yes, N-1. Nothing else did:
    - the smoke makes 1 + 2 × 32 = 65 observe calls (horizon 32), at least the 50 that the summary needs;
    - the reset lines are calls 1–3, which come from `env_reset()` in `train()`;
    - the hole share is the iid rate among eligible pixels, and the large pixel count makes it tight around 0.02;
    - the parser takes the text after the prefix, so a stdout fragment in front of a line, from block buffering,
      is harmless.
  - **Invalid run passes:** I found no path. The install lines must carry the seed's settings, and the lines come
    only from the run's own fresh `launcher.log`. A depth wrapper that never ran gives no first or summary line. The
    motor-update path is unevidenced (N-7).
- **The step-time refusal rule:** correct and tested at the boundary. It runs before preflight and creates nothing.
  The smoke's second epoch still carries 17 of the 50 synchronising depth-statistics calls, so the projection
  errs high, which is the safe side.
- **f·B 66.1875:** consistent in the module, the wrapper (constant and SCOPE), the launcher (a literal export, with
  the comment moved off the line) and the prereg (m-4 row).
- **Import guard** (`test_navrl_sensor_ablation.py`, `git diff` +12 −4):
  - the pattern is unchanged;
  - one basename is added to the skip list;
  - a positive control for `realveh_entry.py` and three unlisted-importer controls are added.
  - Unlisted importers are still caught, and nothing else is weakened. Two limits predate this change: the
    name-based skip and the pattern's blindness to `import_module`. `test_realveh.py:1106-1127` covers the
    `import_module` and `spec_from_file_location` forms for the new modules and bounds the ablation's importers to
    the three entries.
  - At MOTAR-camonly `971ae67`, the only other references to the ablation are path strings in `camonly_run.py` and
    `camk1e_run.py`, which neither pattern matches. The new modules have no importer there.
- **Sidecar manifest:** sound. It is written in a single-use folder with mode `x`, and its SHA-256 is in the report.
  See N-6.
- **BRANCH `camonly/dev` and the tree-root-relative paths:**
  - `ROOT` is the wrapper's own tree. `LAUNCHER`, `RL`, `PREREG` and `hashed_relpaths` are relative to it. The unit
    and CAM-K1 folders are absolute under `~/motar_tmp`.
  - The launcher resolves `ROOT` from its own location. The screen summary it reads exists in MOTAR-camonly with the
    same SHA-256 (`540a0f72…`).
  - `gr_run.py`, `camtrain_entry.py`, `train_navrl_camk1.sh` and the ablation are unchanged between `814c28a` and
    `971ae67`.
  - CAM-K1-E's commit overlaps CAM-K2 only in `docs/EXPERIMENTS.md`, where the m-6 entry goes.
- **The prereg against the code:** they match on:
  - the settings, seeds, tag and epochs;
  - the preflight list, the entry refusals and the TRAINED checks;
  - the telemetry thresholds: bounds 0.8075–1.2075, span > 0.05, holes 0.02 × (1 ± 0.5), MinZ and MaxZ shares
    < 0.5, mean |noise| in (0, 1), σ = k·z²;
  - the 15 hashed files and the smoke-rate example (4.7705 × 1,250 = 5,963 s).

  The exceptions are N-3 (the step-line gate) and N-2 (the docstring).

## Sequencing (m-6, for the coordinator)

- CAM-K1-E is running from MOTAR-camonly now: `camk1e_run.py eval`, started 13:17 with a 12,600 s timeout. Under
  AGENTS.md's freeze, the CAM-K2 files may not be copied into that tree or committed until CAM-K1-E has ended and its
  record is committed.
- The launcher preflight also has to wait. It creates `train_session_logs/.navrl_training.lock` under `aerial_gym/`.
- After that: re-run, from MOTAR-camonly, the PUBLIC_CPU suite, `NAVRL_PREFLIGHT_ONLY=1` and
  `test_experiment_index`, with the EXPERIMENTS.md entry. The approval must name the HEAD that results.
- The other agent's S4F-1 files are not in MOTAR-camonly, so the clean-tree blocker is gone there.
