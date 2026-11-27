# CAM-K2 v1: design review r3 (a focused check of the r2 N-1 repair and the N-2..N-7 fixes)

- **Reviewer:** Claude Opus 5.5 (fresh context), 2026-11-08, about 14:22–14:35 KST. Read-only on every repository.
  I wrote only to this folder.
- **Worktree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-realveh`, branch `realveh/dev`, HEAD `814c28a`. The
  files are uncommitted. `git status` showed 15 entries before and after my runs. The `git diff` is still only
  `test_navrl_sensor_ablation.py`, +12 −4.
- **Reviewed bytes.** The SHA-256 prefixes all equal IMPLEMENTATION.md Part 5:
  - `navrl_vehicle_rand.py` `02e7f4b4…`, `navrl_depth_model.py` `694578e6…`;
  - `camk2_train.py` `2e506074…`;
  - `test_realveh.py` `7c483834…`, `test_camk2_train.py` `02e07a97…`;
  - unchanged since Part 4: `realveh_entry.py` `b153057e…`, `realveh_python` `8936f9f8…`, `train_navrl_camk2.sh`
    `2ff0494b…`, `test_navrl_sensor_ablation.py` `891c87fa…`;
  - the prereg `6de6afd6…`, byte-identical to `PREREG_DRAFT.md`;
  - `probe_depth_effects_fb66.py` `c8d15644…` and `.json` `4467dfcb…` (N-4).
- **CPU windows.** All ran on cores 6 and 7 with `nice -n 19`, `ulimit -v 8000000` and `CUDA_VISIBLE_DEVICES=""`,
  with no Isaac Gym import and no write to any tree. CAM-K1-E's `camk1e_run.py eval` was active in MOTAR-camonly
  throughout; the T-LOAD rule allows CPU work on cores 6 and 7 then. Record these in the WORKLOG:
  - 14:26:15–14:26:30: the two test files;
  - 14:26:37–14:26:38: the r2 probe, rerun;
  - 14:27:04–14:27:06: probe A, mutants;
  - 14:28:01–14:28:05: probe B, the call chain;
  - 14:29:37–14:29:38: probe C, `main()`.

## Verdict: **PASS_WITH_LIMITS**

- **N-1 is fixed for the real training call order.** I found no new MAJOR or minor finding.
- **The new quiet-startup tests are non-vacuous.** Every mutation that brings back late binding fails the positive
  test.
- **N-6 and N-7 add no failure mode for a valid run.**
- **The prereg's "Design review r2" section matches the code.**
- **Limits:**
  - CPU only. I checked statically that the real `import isaacgym` and env construction leave `sys.stderr` alone,
    but did not run them. The development smoke is the end-to-end check. It fails closed: a missing line reads
    TRAINING_FAIL, never a false TRAINED.
  - m-6 (the PUBLIC_CPU suite, the EXPERIMENTS.md entry, the clean tree and the preflight in MOTAR-camonly) and the
    user decisions stay open, as disclosed.

## Tests (prescribed command form)

| File | Result |
| --- | --- |
| `test_realveh.py` | 43 OK (13.4 s) |
| `test_camk2_train.py` | 33 OK (0.7 s), none skipped |

Log: `tests_run.log`.

## (1) N-1 in the real call order

- **Where install runs.** The training process is `realveh_python` → `exec python realveh_entry.py runner.py …`. That
  is `train_navrl.sh:85-87`, whose `2>&1 | tee` puts fd 2 into the session log and, through the wrapper's
  `stderr=STDOUT`, into `launcher.log`. In `realveh_entry.main()`, the only stream swap before the installs is the
  `devnull_stdio()` around `import isaacgym` (`realveh_entry.py:165-167`). It restores `sys.stderr` in its `finally`
  (`quiet_rl_io.py:150-155`). The installs come after it:
  - `veh.install` at `:187` binds `_STREAM` at `navrl_vehicle_rand.py:245`;
  - `depth.install` at `:192` binds at `navrl_depth_model.py:215`.

  `runpy.run_path(runner.py)` comes only after both (`:206`).
- **Nothing else swaps the stream before install.** A search for assignments to `sys.stderr`/`sys.stdout`, for
  `redirect_std*` and for `dup2` found:
  - in `aerial_gym/`: only `quiet_rl_io.py:151/155`;
  - in `tools/camtrain_entry.py`, `tools/realveh_entry.py`, `navrl_task/*.py`, the isaacgym Python package
    (`/home/fair/isaacgym/python/isaacgym`) and rl_games: nothing;
  - in warp: only its own test files.

  The imports inside `install()` (`base_multirotor`, `motor_model`, `navrl_perception`) run before the bind line but
  cannot swap the stream.
- **When the quiet context is active.** runner.py applies it in two places:
  - `devnull_stdio` at module import (`runner.py:39`) and around `runner.load` (`:1055`). Both come after install,
    and neither builds the env;
  - `patch_rlgames_quiet_train_init` (`:1083`). Its `run_train` builds the agent, and so the env, inside
    `devnull_stdio()` (`quiet_rl_io.py:115-116`). The modules now write to the bound stream there.
- **The play/eval path.** `--play` applies neither the quiet `runner.load` (`:1054`) nor the patch (`:1082`), so
  construction happens with normal stdio. The entry admits only `runner.py` (`realveh_entry.py:157`), so any eval
  installs at the same point, outside every quiet context.
- **No eval caller yet.** No eval harness uses these modules: the only users are the entry, the wrapper, the
  launcher and the tests, in MOTAR-realveh and MOTAR-camonly.
- **Remaining dependency, fail-closed:**
  - the binding takes whatever `sys.stderr` is at install;
  - a future caller that installed inside a quiet context would lose every line, the install lines included;
  - `telemetry_problems` would then read TRAINING_FAIL ("expected one install line"), never a false pass.
- **The r2 side remark no longer applies:** the lines no longer depend on `AERIAL_RL_QUIET_STARTUP`, which
  `clean_env()` does not strip.

### Probes

- **The r2 probe, rerun** (`probe_quiet_stderr_r2_rerun.py`, byte-identical to r2's, `2a028833…`; output in
  `.json`). Wrapped in the real `devnull_stdio`, the run now reads install, init, reset, update, reset, reset, with
  problems `[]`. Under r2 it read `["veh-rand: no init line"]`.
- **Probe B, the call chain in child processes** (`probe_r3_chain.py` / `probe_r3_chain_child.py`; output in
  `.json`, logs in `probe_r3_chain_{bound,late}.log`). It follows the production order more closely than the unit
  test:
  - the entry pattern: a `devnull_stdio` import block, then `install`/`install_perception` at top level;
  - then the real `quiet_rl_io.patch_rlgames_quiet_train_init()` on the real `rl_games.torch_runner.Runner`;
  - then `Runner.run_train`. Its `algo_factory.create` runs inside the real `devnull_stdio`, through the installed
    `init_tensors` wrapper, plus one reset and the first depth frame (the worst case);
  - then `agent.train()` runs the resets, the motor updates and ≥ 50 frames;
  - stdout and stderr go to one file, as the launcher does, and the wrapper's own `telemetry_problems(seed 1)` reads
    it;
  - quiet startup is at its default (on).

  | | bound (repaired code) | late (pre-r2 pattern) |
  | --- | --- | --- |
  | `sys.stderr` at install | `is sys.__stderr__`, fileno 2 | the same |
  | `sys.stderr` inside `create` | `/dev/null` | `/dev/null` |
  | veh-rand events | install, init, reset, reset, update, reset | install, reset, update, reset |
  | depth events | install, first, summary | install, summary |
  | problems | `[]` | `no init line`, `no first-frame line` |
  | motor_updates_max | 16 | 16 |

## (2) Are the new tests non-vacuous?

- **What the negative control checks** (`TestQuietStartup`, `test_camk2_train.py:561-584`). Late binding inside the
  real `devnull_stdio` gives "veh-rand: no init line" and "depth-model: no first-frame line" (asserted). Late binding
  without the quiet context gives no problem (asserted), so the loss is caused by the quiet context and not by the
  fixture. The bare mechanism is asserted too: a write inside the context is lost and one after it is kept.
- **Mutation check** (probe A, `probe_r3_mutants.py`, output in `.json`; the mutants are copies in this folder).
  Each mutant swaps the test module's `VR`/`DM` globals. The `quiet_fixture_problems` column is the wrapper's reading
  of the quiet fixture:

  | Mutant | positive test | negative test | `quiet_fixture_problems` |
  | --- | --- | --- | --- |
  | none | PASS | PASS | `[]` |
  | VR: drop the install-time bind | **FAIL** | PASS | no init line |
  | DM: drop the install-time bind | **FAIL** | PASS | no first-frame line |
  | VR: `log()` always uses `sys.stderr` | **FAIL** | PASS | no init line |
  | DM: `log()` always uses `sys.stderr` | **FAIL** | PASS | no first-frame line |

- **A fidelity limit (INFO):** the unit test calls `VR.initialize` directly, not through the installed `init_tensors`
  wrapper, and uses a StringIO as the "real" stream. Probe B covers the wrapper, the real rl_games patch and fd 2.
- **The attribution in the brief:** the quiet-startup test is only in `test_camk2_train.py`. `test_realveh.py`
  gained the N-7 counter assertions (`:690-692`). IMPLEMENTATION.md Part 5 states this correctly.

## (3) N-6 and N-7: no new failure mode for a valid run

- **N-6, the manifest after the verdict** (`camk2_train.py:580-586`). Probe C (`probe_r3_main.py`, output in
  `.json`) drives the real `main()` end to end. It redirects UNIT_DIR, RL and SOURCES into a temporary folder in
  this probe folder and replaces preflight, host, git and the launcher subprocess with stand-ins. The fake launcher
  writes the fixture's real-module telemetry, a run folder with a checkpoint, the receipt, the record and the
  session log. In every case the report is written, the report's manifest SHA-256 equals the file, and the
  manifest's checkpoint SHA equals the copy:

  | Case | exit | verdict | manifest |
  | --- | --- | --- | --- |
  | smoke | 0 | SMOKE_OK | yes, verdict SMOKE_OK |
  | seed 1 | 0 | TRAINED | yes, TRAINED |
  | seed 2 | 0 | TRAINED | yes, TRAINED |
  | seed 1, init line removed | 1 | TRAINING_FAIL | yes, TRAINING_FAIL |
  | seed 1, exit 124 and no checkpoint | 1 | TRAINING_FAIL | none, as no checkpoint exists; the report is written |
  | seed 1, host fault | 1 | VOID_HOST_FAULT | yes, VOID_HOST_FAULT |

  - TRAINED implies a manifest. TRAINED needs no problems, which needs the checkpoint, and `details["checkpoint"]`
    then exists.
  - `write_manifest` reads only files that the same run has just written or checked. Its folder is single-use and
    opened with mode `x`, so it cannot raise in a valid run.
  - The unit test (`test_camk2_train.py:696-701`) re-enacts this step rather than calling `main()`. Probe C closes
    that gap.
- **N-7, the motor-update counter.**
  - Every thrust path goes through `MotorModel.update_motor_thrusts`: the allocator's forces, wrench and root-link
    paths (`control_allocation.py:73,82,91`), from `BaseMultirotor.call_controller` → `control_allocation`
    (`base_multirotor.py:241`). No subclass overrides it.
  - The "update" line is written unconditionally at the first update, `motor_updates=1`, now to the bound stream.
    So max > 0 holds for any run that took one physics step. A count of 0 means that no step ran, or that the wrapper
    is not on the class in use, which is the case the gate is meant to catch.
  - A Python int increment per physics step, and one line per 100,000 updates, is negligible.

## (4) The prereg's "Design review r2" section against the code

| r2 | Prereg text | Code / evidence | Match |
| --- | --- | --- | --- |
| N-1 | bind at install; tests inside the real `devnull_stdio` (positive); late-binding negative control; docstrings corrected | `:245` / `:215`; `TestQuietStartup`; docstrings VR `:44-52`, DM `:34-41`. The old "does not touch stderr" claim is gone from both modules, the prereg and the wrapper. The TRAINED text at prereg `:225-228` agrees | yes |
| N-2 | docstring branch `camonly/dev` | `camk2_train.py:13` | yes |
| N-3 | env-stepping time, about 9 % under wall; 1.25 covers it; the step line listed under TRAINED | prereg `:160-164`. 7,200 × 4.83 / 4.43 = 7,850 s < 9,000 s; r2's "about 7,970" was the rougher figure. TRAINED bullet "at least one … (two for the smoke)" = `camk2_train.py` `(2 if smoke else 1)`; "not gates" heading qualified | yes |
| N-4 | probe copied and cited | both files present, SHA as recorded; cited at prereg `:22`, `:324` | yes |
| N-5 | coordinator kept TW_SCALE 0.15 | prereg `:296`, `:402` | yes |
| N-6 | written after the verdict, with it; CAM-K2-E binds TRAINED | `main()` `:582-585`; probe C | yes |
| N-7 | count in the reset line, first-update line, count > 0 required | `navrl_vehicle_rand.py:229,270-272`; `camk2_train.py:308-312`; TRAINED bullet `:213` | yes |

## Findings (all LOW or INFO; none blocks the commit)

- **R3-1 (LOW, for CAM-K2-E's preregistration).** Probe C shows the manifest is written for TRAINING_FAIL and
  VOID_HOST_FAULT too, with that verdict. It is also written a moment before `train_report.json`, so a process
  killed in between leaves a "TRAINED" manifest with no report.
  - **Fix (no code change needed now):** CAM-K2-E binds `train_report.json` with verdict TRAINED, and requires
    `report.checkpoint.manifest.sha256` to equal the manifest file's SHA-256 and the manifest's verdict to be
    TRAINED. That is stronger than the manifest's own self-reported verdict.
- **R3-2 (INFO).**
  - `_STREAM` is module-global and is never reset (`navrl_vehicle_rand.py:72`, `navrl_depth_model.py:61`).
  - In production this is the intent, and it fails closed (see (1)).
  - In the tests, the private module copies stay bound to the last fixture's StringIO. A later direct
    `initialize`/`apply` call in the same process writes there, not to the current `redirect_stderr`. No current test
    depends on that: `real_telemetry` always installs first, and `test_realveh.py` reads telemetry from child
    processes.
  - **Optional:** restore `_STREAM` to None at the end of `real_telemetry`, or note it in the fixture's docstring.
- **R3-3 (INFO, optional).** Probe B could become a child-process test, since it imports rl_games and torch._dynamo.
  It would then pin the real `patch_rlgames_quiet_train_init` order, not only `devnull_stdio`.
- **R3-4 (NIT).** `navrl_depth_model.py:36` is a 187-character docstring line, left over from the edit. Re-wrap it.

## Files here

- this report;
- `tests_run.log`;
- the r2 probe, rerun: `probe_quiet_stderr_r2_rerun.{py,json,err}`;
- probe A: `probe_r3_mutants.{py,json,err}` and `mutant_*.py`;
- probe B: `probe_r3_chain.py`, `probe_r3_chain_child.py`, `probe_r3_chain.json`, `probe_r3_chain_{bound,late}.log`,
  `probe_r3_chain_settings.json`;
- probe C: `probe_r3_main.{py,json,err}`.
