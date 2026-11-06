# Review r2: S3b formal v1

Reviewer: Claude Opus 5.5 (claude-opus-5-5), fresh context, read-only. Tree: e4364c1 plus the untracked runner
(784083e9…), tests (d9ba3705…) and prereg (bc8142a9…). No PX4, Gazebo, SITL or GPU work was started, and no
attempt01 was created.

**Disclosure:** the wrong-commit probe briefly wrote a scratch `execution_approval.json` in `probes/launch_scratch/`.
It has since been deleted.

## Commands
- Tests: 26 ran, 0 skipped, OK, in 122 s.
- `--check` (taskset 6,7,12,13, nice 10): no input problems, A 122 / B 30 / C 10, no environment problems.

## r1 findings
- **B1: FIXED.**
  - The receiver is stopped and joined, and `L.m.close()` runs in a `finally` inside the session (:246-275).
    `steps.npy` is saved only if `ep_dir` exists.
  - Probe with the real ParamLink, port redirected to 24548, against a fake MAVLink PX4. It ran 6 episodes: normal,
    normal, process died, KeyboardInterrupt, port held, normal.
  - The port was free after each one.
  - With the port held, the run stopped at Contention before `ep_dir` existed: a clean ERROR.
  - The later episodes were VALID, with `param_problems []`.
- **M1: FIXED.** The step wait is bounded at 2 s and calls `session.check(("gzserver","px4"))` every 0.1 s; those
  role names are correct. In the probe, a dead process gave ProcessExited after 1 step.
- **M2: FIXED.** The refusals are non-vacuous, tested with the real AT/B:
  - Of 33 sources, only the 3 untracked files are flagged.
  - All 17 worktree modules that are loaded appear in the list.
  - The F SHA matches the pin, and the cplanner replay file is clean.
  - A wrong commit and the dirty tree are refused.
  - Before `mkdir`, launch only reads, so nothing is created.
- **M3: FIXED, with limits.** The tests drive the real `run_episode`, `_fly`, `wait_for_step`, `run_unit` and
  `launch_problems`, each with paired controls. `launch()` after `mkdir` and `analyze()` are untested.
- **m3 and m4:** fixed.
- **m6:** fixed, apart from two LOWs.
- **m1, m2 and m5:** partly fixed (MINOR-1 to MINOR-4).
- **l4 and l5:** fixed.
- **l2:** still open.

## Identity and reading
- **`_fly`:** line by line it is the development runner at delay 0. The setpoint is `(gey,gex,-float(wz))`, and
  termination, logging and the start check are unchanged. It adds three things:
  - the lateness record;
  - `session.check`, called only while waiting for a step;
  - `read_params`.
- **Compute column:** `r[21]` is correct.
- **`reading()`:** implements the prereg's VOID list and gate exactly.

## Remaining findings

**MAJOR-1: the PX4/Gazebo stack under test is not bound.**
- `run_episode` never calls `OS.runtime_binding(plan)`, so nothing records the px4 binary, airframe 10090, rcS,
  navrl_quad SDF, plugins or the PX4_/GAZEBO_ environment.
- dev08 recorded the binding per member and marked changes BINDING_CHANGED (`sitl_s3b_dev08_prefix.py:133, :491`).
- The shared PX4 build is changing today: airframes 10091 navrl_iris (15:17, "Re-running CMake… ROMFS") and 10092
  navrl_quad_s4 (15:22) were installed.
- **Scenario:** an S3c/S4 rebuild during the 2.3 h run silently changes the vehicle.
- **Fix:**
  - bind each episode after `write_world`;
  - make a change from the first episode's binding (without the world entry) INVALID, and a VOID in the integrity
    check;
  - optionally pin the px4, airframe and SDF SHAs.

**MINOR-1: two prereg claims are false.**
- It says the 0.15 s bound is "named in its approval scope". `SCOPE` does not mention it.
- It says the truth sample's age is recorded. It is not: there is no `L.truth[0]` per step.
- Fix: add the bound to `SCOPE`, and log the truth age, or correct the wording.

**MINOR-2: some paths leave no final.json.**
- `run_unit`'s first `kernel_faults()` (:511) is outside its `try`.
- `schedule`, `episodes_from_trace`, `B.setup` and `B.Policy` run after `mkdir` without a handler.
- A SIGTERM outside an OwnedSession takes the default action and kills the process.
- In each case attempt01 is consumed, and `analyze()` raises instead of reading VOID.
- Fix: wrap everything after `mkdir`, add a SIGTERM handler, and read VOID when final.json is missing.

**MINOR-3: the environment can switch the treatment.**
- `S3B_COMMAND_YAW=ekf` is honoured through `V0.COMMAND_YAW`. dev08 ignored it.
- Stray `NAVRL_*` variables survive `B.setup`.
- `PX4_SIM_*` variables pass into PX4.
- Nothing refuses these. Fix: refuse unless the command yaw is `truth` and none of these variables is set, and record
  the environment keys.

**MINOR-4: idle is checked only at launch.**
- dev08 checked before and after each member (it had seen 25× slowdowns).
- Concurrent sessions are active in this worktree.
- Fix: record `idle_check` per episode.

**LOW:**
- **INT32 parameters:** `EKF2_HGT_REF` is recorded as 2.8e-45 (l2).
- **Stall test:** its patch of `STEP_WALL_MAX_S` has no effect, because the value is a default argument. The test
  still fails correctly, after 2 s.
- **Parameter tolerance:** the code is absolute below 1; the prereg says relative.
- **Envelope:** it omits `python -B`, which writes `.pyc` files into `tools/` otherwise.
- **Not disclosed:** r1 (f), the dev07 compute column, and r1 (d), the 0.7–8.7 s ground delay.
- **Analyzer:** it does not aggregate timing, EKF error or read-backs.
- **Leak after block A:** a leak in block B or C turns the remaining episodes into ERROR rows, with no stop.

## Verdict
**FAIL.** All r1 blockers and majors are fixed, and each fix was verified by probe. MAJOR-1 is new. Fix MAJOR-1 and
MINOR-1 to MINOR-3, then run a narrow re-review: the binding, `launch()` and the prereg diff.
