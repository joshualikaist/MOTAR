# Review r3: S3b formal v1 (focused)

Reviewer: Claude Opus 5.5 (claude-opus-5-5), fresh context, read-only. Tree: e4364c1 plus runner `90662692…`, tests `eccff215…` and prereg `bf6a6b87…`. Probes are in `review_r3/probes/`. I started no PX4, Gazebo, SITL or GPU work, and created no attempt01. The scratch approval files were deleted.

**Process note:** the test file changed during this review (`e2515583…` became `eccff215…` at 15:58:17; `FakeOS.runtime_binding` now deep-copies). I re-ran the tests on the final bytes. Before approval, confirm that the committed SHAs equal these.

## Commands
- Tests (taskset 6,7,12,13, nice 10): 33 ran, 0 skipped, OK, 120 s. Run twice, the second time on the final bytes.
- `--check`: no input problems; A 122 / B 30 / C 10; no environment problems; no `.pyc` written.
- Full PUBLIC_CPU suite: NOT_TESTED.

## r2 findings
- **MAJOR-1: FIXED.**
  - `run_episode` binds every episode after `write_world`, with `AIRFRAMES["navrl_quad"]` = `10090_gazebo-classic_navrl_quad` (equal to `OS.AIRFRAME`).
  - `run_unit` compares `binding_core` (file SHAs, world removed) against the first VALID episode. A test covers it: a world-only change stays VALID, a px4 change gives BINDING_CHANGED.
  - **Real plan probe:** `direct_launch_plan` plus `runtime_binding` succeed. 61 files, 40 plugins, 0 missing, hashed in 0.17 s.
  - Every shared key is byte-identical to the AGQ attempt01 (11:57) and dev08 bindings. The 15:17/15:22 airframe installs changed nothing that is hashed.
  - rcS anchors `_${PX4_SIM_MODEL}$`, so 10092 `navrl_quad_s4` cannot be selected.
- **MINOR-1: FIXED.** `SCOPE` names the 0.15 s bound. `truth_age_max_s` is recorded. The prereg wording matches.
- **MINOR-2: FIXED.** Probe of the real `launch()` after `mkdir`, with stubs and scratch dirs:
  - a SIGTERM 2.5 s in gives final.json ABORTED "Terminated: signal 15", 2 ledger rows, re-raised, and reads VOID;
  - a `schedule()` exception gives final.json ABORTED, and reads VOID;
  - a missing final.json reads VOID (tested);
  - inside a session, SessionTerminated is a BaseException, re-delivered at `__exit__` to the runner's handler.
- **MINOR-3: FIXED, with no false refusal found.**
  - None of the four prefixes is set in this process, a login shell, an interactive shell, the conda activate.d scripts, or `run_chain.sh`, which exports only CUDA/PYTHONNOUSERSITE/OMP/MKL/DISPLAY.
  - The earlier owned-session bindings show no inherited GAZEBO_/PX4_ variable either. They had leading-colon paths, and launches succeeded.
  - The refusal runs before `B.setup` sets NAVRL_*.
  - In the chain environment with the real AT/B, the only refusals were: no approval, the tracked tree not clean (`docs/EXPERIMENTS.md`, another session's edit), and the 3 untracked sources.
  - The `PX4_` refusal matters: rcS honours `PX4_SYS_AUTOSTART` over the model.
- **MINOR-4: FIXED.** The idle check runs before each episode, with one 30 s wait. HOST_BUSY gives NOT_RUN without calling `run_one` (tested).
- **LOWs:**
  - FIXED: INT32 decoding (type 6, tested); the stall test (asserts "in 0.3 s"); tolerance wording; `python -B`; the disclosures of (d) and (f); the consecutive-invalid stop with STOPPED_AFTER_A, read only when all block-A rows exist (tested).
  - Timing summary: PARTLY fixed (see L3).

## Identity
A normalized diff of `_fly` against the `sitl_s3b_closed_loop.py` loop shows only these additions:
- `session.sleep`;
- `read_params`;
- `wait_for_step` (same condition, with a 2 ms poll);
- the lateness and truth-age appends;
- `CommandDelay(0)`, which returns its input object;
- `float(wz)`;
- column 21 for compute time.

Observation, policy, governor, setpoint, termination and logging are unchanged. Probe: `probes/loop_diff.txt`.

The session deadline is 240 s. A full 600-step episode is about 110 s (dev08 sessions: median 42 s at flight start ≈ 35.7 s sim), so the margin holds.

## Remaining findings
**L1 LOW: the empty.world template is not bound, and the world is excluded.**
- dev08 hashed `empty_world_template`.
- **Scenario:** S4 work edits PX4's `empty.world` mid-run, and the arenas change silently.
- **Fix:** add the template's SHA to the core.

**L2 LOW: a termination after block A is complete reads VOID.**
- An outer timeout or SIGTERM in block B or C is ABORTED, not STOPPED_AFTER_A. That is the preregistered behaviour.
- The budget covers 88.9 s per episode. The estimate is about 50–60 s, so the risk is small.
- SIGHUP outside a session takes the default action and leaves no final.json. That still reads VOID (fail-closed).

**L3 LOW: the analyzer omits two of the prereg's "reported" items.**
- It has no aggregate of lateness p99 and no aggregate of read-back values. They exist only per row.

**L4 LOW: one refusal is untested, and only the pre-episode idle is checked.**
- The command-yaw refusal has no negative test. It is redundant with the `S3B_` refusal.
- Idle is checked only before each episode (dev08 also checked after). A mid-episode load shows only in lateness and the real-time factor, which is ungated.

**Operational:** the launch will refuse until `docs/EXPERIMENTS.md` is committed or restored. A concurrent rebuild or commit during the run fails closed (BINDING_CHANGED, then the stop; or a tree change ABORTS).

## Verdict
**PASS_WITH_LIMITS.** All r2 findings are fixed, verified by probes. No BLOCKER, MAJOR or MINOR remains. The limits are L1–L4, the untested full suite, and the bytes still to be matched at commit.
