# REVIEW unit B r1: PX4 vertical fields on the ROS path, bounded refusal hold

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, independent, read-only on the worktree and the
  snapshot. Writes only to this directory.
- **Date:** 2026-11-18.
- **Object:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `fc8aa4d9`
  (unchanged), uncommitted tree, compared with the preB snapshot
  (`/home/fair/motar_tmp/ros2_runtime_outdoor_20261113/preB_tree/files/`).
- **Inputs used:** the code, the tests, the specification of Goals 1 and 2 and criteria C1-C7. I read the implementer's
  work record (`../UNIT_B_r1.md`) only after my own analysis, to check its claims.

## Verdict: **PASS_WITH_LIMITS**

I found no P0 or P1. The change does what the specification asks:
- The fields travel in the same message as the pose.
- The node reads them only with an explicit, versioned declaration.
- Every absent, malformed or undeclared path ends in a named refusal or UNCONFIGURED.
- The hold episode is bounded and latched, and needs an explicit reset.
- No vehicle command is added, and the gateway's existing stale-command path revokes.
- Indoor and gateway outputs are byte-identical to preB in my differential run.
- All 27 of my source mutations were caught.

The limits:
- **NOT_TESTED:** rclpy, DDS, the installed `px4_msgs` package, PX4, SITL and the vehicle. These are declared honestly.
- **Two P2 items to document before commit.**
  - A refusal duty cycle with a policy command at least every 7th tick is never bounded.
  - A README statement in the pinned prefix is now contradicted, and nothing marks it as superseded.
- **Eight P3 items.**

No flight qualification and no outdoor GO follow from this review.

| Severity | Count |
| --- | --- |
| P0 (unsafe or wrong) | 0 |
| P1 (must fix before commit) | 0 |
| P2 (should fix or document) | 2 |
| P3 (nit or hardening) | 8 |

## Per-criterion results

| Criterion | Result | Evidence |
| --- | --- | --- |
| C1 Forwarding correctness | **PASS** (gateway-to-node hop); LIMITED_VALIDITY for the rclpy glue | See C1 below. |
| C2 Fail-closed and declaration | **PASS** | See C2 below. |
| C3 Indoor and geofence unchanged | **PASS** | See C3 below. Additive diagnostics keys: P3-7. |
| C4 Hold-timeout semantics and safety | **PASS_WITH_LIMITS** | See C4 below. Limits: P2-1, P3-1, P3-2, P3-4. |
| C5 Tests | **PASS** | See C5 below. rclpy glue is only covered through a test-side copy: P3-5. |
| C6 Docs | **PASS_WITH_LIMITS** | See C6 below. Limits: P2-2, P3-1, P3-2. |
| C7 Scope | **PASS** | See C7 below. |

### C1: Forwarding correctness

- **Same estimator sample.** The gateway takes the fields from the same `snapshot()` that `decode_estimator` pairs
  (|t_odom - t_lpos| <= 0.025 s): `tools/camera_ros2_px4_node.py:228-243`.
- **Same message.** The fields are written into the same Odometry as the pose (`:319-324`, `:470-472`). On the
  policy side they are decoded from the very message or messages whose pose is used (`tools/camera_ros2_adapter.py:498-517`).
- **Bracket.** An interpolation bracket combines both samples conservatively (`:127-144`). Mutations M01-M03 and M07
  are caught.
- **Cannot pass as a real covariance.** Slot 0 holds a negative tag, all 68 other slots must be NaN, and the flags must
  be exactly 0 or 1 (`:99-124`). Mutations M04-M06 and M26 are caught.
- **Exact decode.** Values are float64, both ways.
- **No silent default.** `tools/camera_ros2_px4.py:353-370`. Mutation M19 is caught.

### C2: Fail-closed and declaration

- **Binding key.** The key is optional and must match exactly when present (`camera_ros2_adapter.py:206-211`, M08).
- **Node.** The node refuses an outdoor contract without the declaration as `UNCONFIGURED:
  outdoor_vertical_fields_not_forwarded` (`tools/camera_ros2_node.py:80-84`, M09 and M21).
- **Frames.** Absent, malformed or undeclared fields give `estimator_vertical_fields_absent`. Bad values give
  `estimator_z_invalid`, `estimator_epv_exceeds_limit` or `estimator_dead_reckoning`. All are zero holds. Unit B test
  `test_named_refusals_for_absent_malformed_undeclared_and_bad_values` checks this, with 3 accepted frames as the
  positive control.
- **z_valid false.** The gateway still refuses it (`camera_ros2_px4.py:335-337`, unchanged; tested with a control).
- **.msg bindings.** The PX4 `.msg` SHA-256 bindings are unchanged; a test pins the literal values.
- **Read-field types.** These checks are extended to the installed classes (`camera_ros2_px4_node.py:81-105`, M20).

### C3: Indoor and geofence unchanged

- **Differential run** (`diffcheck/`). I ran the same indoor scenario through the preB tree and the current tree, each
  in a fresh process. The scenario covers 15 ticks: gating, nearest and bracketed odometry, re-entry after gating, and
  the production indoor preprocess on the adapter's frames. It also runs the gateway's `BridgeSession` through
  prime, PRIMING, OFFBOARD and ACTIVE.
  - Proposals, statuses, the legacy diagnostics keys, the runtime's frames and states, the indoor preprocess outputs,
    and the gateway payloads, states and diagnostics (new keys dropped) are **EQUAL**. The only difference is the
    random tempdir path of the test fixture.
- **Golden digests.** The pre-existing `OutdoorOffIsBitIdentical` indoor golden test passes.
- **Gateway class.** `Px4OffboardGateway` and the geofence code have no diff.
- **Indoor never reads the fields.** `camera_ros2_session.py:141` (M10).

### C4: Hold-timeout semantics and safety

- **Episode rules** (`camera_ros2_session.py:347`, `:376`, `:385`, `:400-416`; `camera_policy_runtime.py:122-161`):
  - Refusals do not restart the episode (M11).
  - Re-entry holds count toward it (M12).
  - Gated ticks do not end it (M13).
  - The timeout raises and latches the fault (M14).
  - `reset()` is the only way to clear it (M15).
  - The timer fires exactly at the limit, not one tick late (M16).
- **Bounds.** The limit is [1, 10] s, inclusive at both ends, with status PROVISIONAL, checked in two places
  (M17, M18, M27).
- **Gateway revocation** happens on the unchanged path at `camera_ros2_px4.py:626`; tested 0.2 s after the last hold.
- **Clock.** Capture stamps strictly increase, which closes the stamp-jump hole. The wall-time stretch is bounded at
  about 1.33-1.5× the limit (P3-2).
- **Stalled camera.** A stalled stream faults through the watchdog within 0.15 s, so a stall cannot prolong a hold.

### C5: Tests

- **Unit B file:** 28/28 pass (`test_unit_b.log`).
- **Related camera and ROS modules, one process:** 269 tests OK, 10 expected failures. The expected failures are
  pre-existing perception limitations (`test_related.log`).
- **Mutation probe:** 27/27 killed, and the unmutated baseline passes (`mut/`).
- **Positive controls:** each negative is paired with a positive control that asserts a nonzero count.
- **Full PUBLIC_CPU:** I did not rerun it. The implementer's run (Ran 4259, OK) is bound to a 27-path SHA list that I
  re-verified against the current tree: 27/27 OK.

### C6: Docs

- The text matches the code.
- Honest NOT_TESTED list.
- No flight, GO or real-flight claim.

### C7: Scope

- Exactly the 10 listed files changed, plus the new test file.
- The 16 other preB paths are byte-equal.
- Nothing changed under `results/**` or `.claude/`, and neither the checkpoint (SHA matches its pin) nor any
  preregistration, `aerial_gym/task/navrl_task/*` file or WORKLOG was edited.
- HEAD is unchanged.

## Findings

### P0: none

### P1: none

### P2-1. A refusal duty cycle is never bounded (residual of the episode definition, undocumented)

- **Where:** `tools/camera_ros2_session.py:383-385`, where any policy command ends the episode, and
  `ros2/motar_camera_policy/README.md:394-410`.
- **Failure scenario.** Outdoors, the ground fit refuses every 7th frame. Each refusal is followed by 5 re-entry holds
  and then one policy command, so every episode ends after 0.5 s and the bound never trips.
- **Probe** (`probes/hold_semantics_probe.json`, case a): over 600 ticks (60 s), 515 ticks (86 %) were zero holds and
  85 (14 %) were policy commands. There was no fault and the longest episode was 0.5 s.
- **Why it matters.** The vehicle can sit in a mostly-zero-hold regime indefinitely, commanded once per 0.7 s from a
  history that is 5 frames old. This is consistent with the specification's episode definition and is not unsafe in
  itself. But the README's "bounded hold" text does not say that the bound covers consecutive holds only.
  The unit test's control (`k % 7`) shows the behaviour without naming it.
- **Fix.** State this residual in the README and the evidence section. Whether to add a windowed hold-ratio bound is
  the user's decision.

### P2-2. The README's pinned prefix now contradicts unit B, and the gateway encoding is unconditional

- **Where:** `ros2/motar_camera_policy/README.md:79-80`, `tools/camera_ros2_px4_node.py:470-472` and
  `README.md:359-363`.
- **The stale statement.** The pinned general section still says "The private derived Odometry contains NaN
  covariance entries ... The policy ignores covariance." Since unit B:
  - `pose.covariance[0:4]` carries a negative tag and the flags whenever PX4 provides the fields, which a real PX4
    always does.
  - A declared outdoor policy node reads them.
- **Unconditional.** The gateway writes the encoding for every deployment, indoor included. There is no gateway-side
  opt-in. Any other consumer of `/motar/state/odometry` (RViz, bag tools, a later estimator) now sees a "variance" of
  -20261118 in slot 0.
- **What the unit B section lacks.** It describes the encoding but never says that it supersedes lines 79-80, or that
  the encoding is written whatever the policy profile.
- **Fix.** Add one dated sentence in the unit B section. The prefix is test-pinned, so it should not be edited.

### P3 (nits, hardening, wording)

**P3-1. Gated time counts toward the episode, and the latch can fire before OFFBOARD.**
- **Where:** `camera_ros2_session.py:38` and `README.md:396`, which say a gated tick "neither extends nor ends it".
- **What the code does.** The episode is measured on capture stamps, so time spent gated still counts.
- **Probe** (case b): an open episode, then 60 s gated (landed), then a re-climb gives `HoldTimeout` at the first
  in-band hold, with `episode_s` 60.3.
- **Before OFFBOARD.** The latch can also fire while the gateway is PRIMING and the pilot holds in POSITION mode.
  Both cases fail closed and need an operator reset.
- **Fix.** The work record (item 5) states this; the README should too.

**P3-2. The README overstates the clock check.**
- **Where:** `README.md:399` says the ROS clock "is checked against monotonic time".
- **What the code checks.** In the policy node, only "not backwards", "not paused for more than 0.15 s" and the 10 Hz
  period.
- **Effect.** A steady but slow ROS clock is bounded only indirectly: by the gateway's per-update |Δros - Δmono| <=
  0.025 s and by the 0.15 s output-stale check. That stretches the wall-clock hold to at most about 1.33-1.5× the limit.
- **Fix.** The work record (item 4) discloses this; the README should say "pauses and jumps", not "rate".

**P3-3. The producer identity is asserted by configuration only.**
- **Where:** `camera_ros2_adapter.py:61-64`.
- **Risk.** Any node publishing the exact tag on the declared topic is trusted.
- **Hardening.** Check `get_publishers_info_by_topic` for exactly one publisher whose node name is
  `motar_px4_gateway`.

**P3-4. A reset within about 0.15 s of the latch pre-empts the gateway revocation.**
- **Where:** `camera_ros2_node.py:145-152`.
- **What happens.** A `~/reset_policy_session` call inside that window starts a fresh session. Its re-entry zero holds
  arrive with newer capture stamps, so the gateway stays ACTIVE and the PX4 hand-off never happens.
- **Scope.** This is pre-existing reset semantics, and no code calls the service automatically. Document that no
  supervisor may auto-reset.

**P3-5. The ROS publish code is tested only through a test-side copy.**
- **Where:** `tests/test_camera_runtime_unit_b.py:66-75` (`gateway_odometry`) re-implements
  `Px4GatewayNode._publish_odometry` (`camera_ros2_px4_node.py:446-473`).
- **Gap.** A swapped pose/twist assignment in the real method would go uncaught on CPU. I checked the method by
  reading it, and it is correct.
- **Status and fix.** NOT_TESTED is declared. Optionally test the real method with stub ROS modules.

**P3-6. The NPZ `replay` now aborts on duplicate capture stamps.**
- **Where:** `camera_policy_runtime.py:1386`.
- **Change.** With equal capture stamps on consecutive held lines (possible after a refusal, because `reset()` clears
  `_last_stamps`), the replay raises `ValueError` from `episode.hold`. It used to complete.
- **Impact.** Offline tool, fails closed, not documented.

**P3-7. Indoor diagnostics gained keys.**
- **Where:** `camera_ros2_node.py:198-201`.
- **What changed.** The diagnostics topic now also carries `fault`, `hold_*: null` and `vertical_*: null`. This is
  additive only. The comment lists the unit B keys but not `fault`.

**P3-8. One malformed-epv path revokes instead of forwarding nothing.**
- **Where:** `camera_ros2_px4.py:367`.
- **What happens.** `float(epv)` of a huge Python int raises `OverflowError`, which is not caught as `GatewayError`,
  so the gateway would revoke instead of forwarding nothing. This is unreachable with typed `px4_msgs`, because
  `READ_FIELD_TYPES` pins `epv` to float.

### Accepted by design (not findings)

- A malformed encoding is refused with the existing `estimator_vertical_fields_absent` (MALFORMED shows in the
  diagnostics).
- An exactly zero policy command ends an episode.
- The replays keep processing after the timeout frame.
- The gateway's geofence-fault neutral hold stays operator-reset only; it is pre-existing and outside unit B.

## Commands run (all read-only on the worktree; env `CUDA_VISIBLE_DEVICES="" OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1`, `taskset -c 0-3,6-23`, `timeout -k 30`, `python -B`)

| Command | Result line |
| --- | --- |
| Scope check: `cmp` of each `preB_tree/paths.txt` entry; `git status --porcelain` compared with the snapshot's copy; `find -newer`; `find -L results` | 10 DIFF (the listed files), 16 SAME, plus 1 new test file. `results/` newer files: 0. `.claude` newer files: 0. |
| `diff -u` regenerated → `my_unitB_vs_preB.diff` | 1860 lines, the same size and file set as the provided diff |
| `python -m unittest test_camera_runtime_unit_b -v` (first attempt used `tests.` module names and failed with ImportError because `tests/` is not a package; rerun with `PYTHONPATH=$PWD:$PWD/tests`) | `Ran 28 tests in 1.459s OK`, 0 skipped (`test_unit_b.log`) |
| The same runner with 14 modules: unit_b, ros2_px4, ros2_px4_node, ros2_session, ros2_adapter, ros2_integration, ros2_package, runtime_outdoor_r1, runtime_fixes, runtime_fixes_r2, runtime_repair_r1, policy_contract, policy_runtime, oakd_camera | `Ran 269 tests in 156.642s OK (expected failures=10)`, exit 0 (`test_related.log`) |
| `mut/run_mutations.py` (scratch copy `mut/tree`, one mutation at a time) | BASELINE OK; **27/27 KILLED** (`mut/mutation_run.log`, `mut/mutation_results.json`); scratch files restored and asserted byte-equal |
| `diffcheck/indoor_diff.py`: preB tree, then the current worktree, each in a fresh process | session 15, runtime_inputs 11, gateway 17: `EQUAL_AFTER_TMP_PATH_NORMALISATION` (`diffcheck/diff_summary.txt`) |
| `probes/hold_semantics_probe.py` | (a) 600 ticks: 85 commands, 515 holds, fault null, longest 0.5 s. (b) After 600 gated ticks: HoldTimeout, `episode_s` 60.3, 3 held proposals. |
| `python -B tools/check_public_docs.py` | exit 0 (`check_public_docs.log`) |
| `sha256sum -c logs/unitB_tree_before_public_cpu.sha256` and `logs/unitB_changed_files.sha256` | 27/27 OK and 11/11 OK; the implementer's `unitB_public_cpu.log` ends `Ran 4259 tests in 921.762s / OK (skipped=10, expected failures=10)` |
| `sha256sum` of the seed-1 checkpoint | `888e8056…c8e1`, equal to its pin (`checkpoint_sha256.txt`) |
| `journalctl -k -b 0 \| grep -cE "Machine check\|segfault\|general protection"` before and after | 1 and 1 (the pre-existing 14:59:46 QGroundControl user-space segfault); cpu4-5 offline |
| Final state | HEAD `fc8aa4d9`; 27 uncommitted paths (unchanged); preB snapshot self-check 0 mismatches; the ignored lock file's mtime is unchanged by my runs |

## Files in this directory

`REVIEW_unitB_r1.md` (this file), `my_unitB_vs_preB.diff`, `test_unit_b.log`, `test_related.log`,
`check_public_docs.log`, `checkpoint_sha256.txt`, `mut/` (`run_mutations.py`, `mutation_run.log`,
`mutation_results.json`, scratch `tree/`), `diffcheck/` (`indoor_diff.py`, `trace_pre.json`, `trace_post.json`,
`diff_summary.txt`, scratch `pretree/`), and `probes/` (`hold_semantics_probe.py` and its JSON output).
