# Review r1: S3b formal v1 (runner, tests, preregistration)

Reviewer: Claude Opus 5.5 (claude-opus-5-5), fresh context, read-only. Tree: `camonly/dev` e4364c1 plus 3 untracked files. Probes and outputs are in `review_r1/probes/`. Started nothing: no PX4, Gazebo or GPU, no attempt01, no approval file.

## Commands run
- Tests: 13 run, 0 skipped, OK. `test_real_inputs` and the world-identity test both ran.
- `--check` (taskset 6,7,12,13, nice 10, OMP/MKL 1): input_problems `[]`; A 122, B 30, C 10 (162); environment_problems `[]`.
- Probes: all 162 scenes have 600 zero-action target frames and matching Isaac outcomes. Every `aerial_gym.*` module resolves inside the worktree after `B.Policy()`. The first observe+act takes 38.6 ms, later ones about 4 ms.

## 1. Computational identity (delay 0)
Identical to the development runner: observation, policy, governor, command yaw, setpoint (`cex,cey,cwz` = `gex,gey,float(wz)`, `wz` already a float), termination and crash_reason, start check, message intervals, the 10 parameters, settle and hold timings, final mode command.

Differences:
- (a) Fresh external rootfs and direct launch instead of `sitl_run.sh` with the shared `parameters.bson`. dev07 imported 29 persisted parameters, and its effective non-overridden values (yaw-rate gains among them) are NOT_RECORDED (dev08 amendment2:141-142). The formal run uses airframe defaults (MC_YAWRATE_P 0.11, I 0.055).
- (b) The gzserver cwd, world path/name and GAZEBO_LOG_PATH differ; NO_PXH is not exported. rcS does not read PX4_SIM_WORLD.
- (c) OwnedSession refusals, which do not wait, replace `other_sitl()` and its 10 s wait.
- (d) `read_params` sits between the parameter set and the origin/takeoff. That adds 0.7–8.7 s while the vehicle is on the ground and disarmed.
- (e) Teardown: owned-only cleanup instead of killpg/pkill.
- (f) Logging: both runners write the same 27 columns. Index 21 is compute_ms, so the formal `r[21]` is correct. The development runner's `r[-3]` is index 24, p_true y. The dev07 reports' `compute_ms_median/max` equal the median/max of column 24 in all 20 episodes (dev07_70 ep0: 24.952 = 24.952). Those historical numbers are not compute times.
- (g) There is one bridge setup with the 70-bar NAVRL snapshot. dev07 ran one process per density. The differing keys are NUM_BARS, DENSITY_* and paths (dev08 review: they do not reach the bridge).

## Findings

**B1 BLOCKER: the second episode cannot start, and the runner dies without final.json.**
- `run_episode` never closes the pymavlink socket. `finally` only sets `L.stop`, at :349-351.
- pymavlink keeps the last connection in `mavutil.mavfile_global` (mavutil.py:175-177), so UDP 14540 stays bound.
- The probe with the real `ParamLink` (redirected port) shows the port still busy after `gc.collect()`, and `OS.busy_ports` returning errno 98. The control: after `close()` the port is free.
- Episode 1's `OwnedSession.__enter__` then raises Contention before it creates `ep_dir`.
- `np.save(ep_dir/"steps.npy")` (:352) then raises FileNotFoundError (probe `enter_refusal_probe.out`). launch() dies with no ledger line for ep001 and no final.json, and attempt01 is consumed.
- The same chain follows any LockBusy or leak marker left by a CleanupError.
- Fix:
  - stop the receiver, join it, then `L.m.close()`, as dev08 `run_episode` does at :145-150;
  - save steps only if `ep_dir` exists;
  - wrap each episode in launch() with `try/except BaseException` that writes final.json ABORTED;
  - add a two-episode fake-PX4 test.

**M1 MAJOR: unbounded pacing wait (:286).**
- Scenario: PX4 or gzserver dies mid-episode, `L.pos` freezes, and the loop spins forever.
- The 240 s watchdog abort cannot reach a main thread that never calls `session.check()`.
- The unit then hangs until `timeout -k 30 14400`, without final.json or a reason.
- Fix: call `session.check(required=("gzserver","px4"))` and add a per-step wall bound in the pacing and hold loops (dev08 :197-200, :233).

**M2 MAJOR: the clean-tree check can be bypassed, and Policy F is not bound.**
- `git_state` uses `--untracked-files=no`. An untracked runner or prereg (the current state) passes, while the approval names HEAD.
- `OWN_SOURCES` are hashed but never compared with HEAD.
- The F checkpoint is untracked (`git ls-files` says UNTRACKED; sha f7022139…). Nothing verifies it or records it, although the prereg names it.
- Fix: run `AT.source_problems(ROOT, OWN_SOURCES + the bridge's production modules)`, pin `F_SHA` and record it, and require the external replay file to be clean (dev08 `external_binding`).

**M3 MAJOR: the tests miss the live path.**
- The helper tests are non-vacuous, with paired controls.
- Nothing exercises `run_episode`, `ParamLink.rx`/`read_params` parsing, the `launch()` guards and stop rules (first-episode, invalid-count, kernel-fault, existing attempt dir, dirty tree), the lateness computation or the `r[21]` index.
- B1 would have been caught. dev08's fake PX4 plus the real OwnedSession (`test_sitl_agq_geometry_diag.py`) is the model.

**m1 MINOR: `kernel_faults()` errors leave no final.json.**
- AttemptError after an episode (:490) is unhandled.
- `faults0` is read after `ATTEMPT_DIR.mkdir`, so a journalctl failure consumes the attempt.
- Fix: read faults before mkdir and handle errors as ABORTED.

**m2 MINOR: the lateness bound has two problems.**
- Lateness is the newest position sample time minus the schedule. It is non-vacuous: reconstructed from the dev07 steps, ep02 at 70 bars reached 0.136 s with a 218 ms compute spike.
- It excludes sample-to-command compute and attitude/truth staleness. There is no STALE_TRUTH check.
- D-2026-11-05-2 is scoped to "the dev08 observed-prefix diagnostic". The prereg extends it, so the approval should restate 0.15 s.

**m3 MINOR: the INVALID exclusion is not strictly outcome-independent.**
- LATE risk grows with episode length.
- A mid-episode ERROR hides an outcome.
- Report, ungated, a worst case that counts INVALID block-A episodes as crashes.
- The scene eligibility is outcome-independent. The 6 excluded scenes at 70 bars are zero-action captures and Isaac captures, decided from the zero ledger.

**m4 MINOR: the analyzer does not check integrity.**
- Not checked: rows against `attempt.json` schedule (count, order, duplicates), `kernel_faults_after`, and `git_after` against git (the frozen tree).
- Add these as VOID conditions. They need a new prereg sentence now, before any data.

**m5 MINOR: envelope gaps against dev08 and AGENTS.md.** There is no idle-core check, no per-process memory cap (the AGQ launcher used 16 GiB) and no launcher that enforces taskset or timeout. Another session is building `tools/sitl_s4/` in this worktree right now and has modified `docs/EXPERIMENTS.md`.

**m6 MINOR: prereg wording.**
- "PX4 parameters as dev07" holds only for the 10 overrides. Disclose (a).
- The plan's Gazebo contact cross-check is not implemented and not disclosed.
- The plan lists B1-R/GR-0 layouts; this unit uses G8 r2.
- Disclose that dev07 already flew 9 block-A scenes (envs 0–4, 6–9) and 10 block-B scenes, and that its compute numbers are mislabeled (f).

**LOW findings:**
- l1: `B.ROOT == ROOT` is tautological (`sitl_bridge` loads from the same `TOOLS`). Harmless, per the import probe.
- l2: INT32 PARAM_VALUE (EKF2_HGT_REF) is bytewise-encoded and recorded as a denormal float. Decode by `param_type`.
- l3: a px4/gzserver user-space segfault counts as a host fault. This errs safe; the current boot has 0.
- l4: `episodes_run` counts the unrun scene on the target-check abort. The session.json SHA is not in the ledger.
- l5: FEASIBLE is a point-estimate rule (7/122 passes). Report a CI, ungated.

## What is correct
- **Launch:** apart from B1/M1, the session usage is correct: context manager, `session.sleep`, `write_world`, model, the episode path under ~/motar_tmp (no `.git` above) and udpin 14540 (PX4 binds 14580).
- **Parameters:** the REAL32 read-back, the `param_request_read_send(ts, tc, id, -1)` arguments and the 1e-6 tolerance are sound.
- **Guards:** the approval, input-pin and single-use checks are non-vacuous. The stop rules match the prereg text but are untested.
- **Analyzer:** it matches the prereg (VOID conditions, Δcrash ≤ +5.0, exact at n=120, exclusion from both rates, C pairing).
- **Criterion:** verbatim from the plan.

## Verdict
**FAIL.** B1 stops the unit at episode 1, before any reading. M1–M3 must be fixed and reviewed again before execution approval.
