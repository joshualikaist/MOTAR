# CAM-T1 repair r1 (answers `review_pre_r1/REVIEW.md`, PASS_WITH_FIXES), 2026-11-12

- **Author:** Claude Code build subagent (Opus 5.5). CPU only, no GPU, no commit.
- **Scope of edits:** worktree `src/MOTAR-camonly-t1` (branch `camonly/t1`, HEAD `eefb832`, nothing staged) and
  `~/motar_tmp/camt1_20261112/`.
- **Not touched:** `MOTAR-camonly` and `review_pre_r1/`.
- **Usage state:** USAGE_UNKNOWN.
- **Line numbers** refer to the files as they stand after this repair. "prereg" is
  `docs/plans/prereg_camonly_t1_2026-11-12.md`.

## Finding → change → test or evidence

### F1 (P2): gate trace coverage, `actions_match` and `frames_contiguous` → FIXED

- **Change:** `tools/camt1_analyse.py:466` (`CONSISTENCY_FLOOR = 0.99`) and `:469` (`consistency_problems`).
  - Per policy seed, MEASUREMENT_INVALID results when any of these holds:
    - traced / (traced + untraced) quota crash rows < 0.99;
    - `actions_match` share < 0.99, or any value missing;
    - `frames_contiguous` share < 0.99;
    - no crash episode;
    - VALID share < 0.90.
  - The output records the reasons. Wired in at `stage_a`, `:536`.
- **Prereg:** :224 (Reading, MEASUREMENT_INVALID).
- **Test:** `tests/test_camt1.py:711` `test_f1_consistency_gate`.
  - Positive cases: all 1.0; coverage 100/101 = 0.990; 99 % `actions_match`.
  - Negative cases: coverage 100/102; 98 % `actions_match`; one `None`; 98 % contiguous; 89 % VALID; a seed without crashes.

### F2 (P2): non-VALID crashes in the category shares → FIXED

- **Change:** `tools/camt1_analyse.py:496` (`split_summaries`).
  - Pooled and per-seed summaries, including every interval, are computed over VALID episodes.
  - `invalid_not_in_shares` reports, per seed: count, nominal category split and distance quantiles.
  - `per_density_invalid` is reported.
  - `sensitivity_all_crashes` keeps the all-crash shares.
  - Guidance uses the VALID-only shares.
- **Prereg:** :196 (*Shares over VALID crash episodes*). The limits section is updated.
- **Test:** `test_camt1.py:730` `test_f2_shares_over_valid_only`.
  - 40 invalid BLIND episodes are excluded from the shares: IN_VIEW reads 1.0.
  - Counted separately: 40 invalid, split into categories.
  - Still in the all-crash sensitivity: 40/150.
- **Smoke:** `build_smoke/smoke_stage_ab_r1.out.json` (stand-in G3 cells): 27 / 24 invalid per seed reported
  separately.

### F3 (P2): identity requires `result_equal` → FIXED

- **Change:**
  - `tools/camt1_run.py:294`: `identity_pass` = both ledgers byte-equal, outcome counts equal and `result_equal` (G7's rule).
  - `:302` `identity_reading` and `:309` `control_reading` use it.
- **Prereg:** :78 (*Identity rule*).
- **Test:** `test_camt1.py:361` `test_result_equal_is_required_for_identity`.
  - Byte-equal ledgers with a changed `crash_causes` give `pass` true but `identity_pass` false, so NOT_REPRODUCED.
  - A changed checkpoint path alone is still an identity (positive case).
  - `test_control_reading` (:382) covers `identity_pass` false → CONTROL_FAIL.

### F4 (P2): reproduction checked first; NOT_REPRODUCED → FIXED

- **Runner changes** (`tools/camt1_run.py`):
  - `:107` `IDENTITY_PLAN = (native_identity, 205, 4501)`.
  - `:108` `CONTROL_PLAN` is now three cells (`k1d0_identity`, `d1_smoke`, `cap20_smoke`).
  - `MAX_HOURS["identity"] = 0.25`.
  - `REPORT["identity"]`.
  - `:420`: trace needs one IDENTITY_PASS on HEAD.
  - `:572`: identity reading.
  - `:590`: the `identity` stage on the CLI.
  - SCOPE, docstring and `e3_problems` blocks are updated.
- **Analysis bindings:**
  - Stage A: `tools/camt1_analyse.py:524` `stage_a_binding_extra` (identity IDENTITY_PASS on the trace commit).
  - Stage B: `:658` (`identity_pass_same_commit`).
- **Chain:** `run_camt1.sh:134` runs `stage identity` first. A non-zero exit stops before any trace cell, with the log
  line "NOT_REPRODUCED or identity failure".
- **Prereg:**
  - :65 stage table;
  - :81 *Identity stage, first*;
  - Order of stages;
  - the TRACE_INTERFERES-with-identity-passed rule (Stage B proceeds; V3s2's native reproduction is untested, disclosed);
  - budget (identity cap 0.25 h, total cap 5.25 h).
- **Tests:**
  - `test_constants_and_plans` (plans 1 / 24 / 3 / 36, PLANS order);
  - `test_stage_dependency` (:402): trace refused without identity, and refused on NOT_REPRODUCED;
  - `test_f7_bindings` (:745);
  - `test_f8_no_labels_on_control_fail` (:794): Stage B reads EVAL_BINDING_FAIL on NOT_REPRODUCED.
- **Smoke:** `build_smoke/smoke_runner_loop_r1.out.json`.
  - identity IDENTITY_PASS, then trace NONINTERFERENCE_PASS, control CONTROL_PASS (3 cells), envelope COMPLETE.
  - A differing `native_identity` reads NOT_REPRODUCED.

### F5 (P2): the chain cannot stop a launcher in its own session → FIXED

- **Change:** `~/motar_tmp/camt1_20261112/run_camt1.sh`.
  - Helpers:
    - `:43` `descendants` (walks the parent chain, so it crosses sessions);
    - `:51` `record_sessions` (session ids of all descendants, every `POLL_S`);
    - `:67` `kill_tree` (TERM, then a grace period, then KILL for every descendant and every recorded session);
    - `:76` `reap_sessions` (after every stage, kills and logs anything left in a recorded session, for example a
      launcher orphaned by a `timeout` kill);
    - `:87` `run_capped`;
    - `:106` `gpu_wait_idle` (waits until `nvidia-smi` lists no compute process other than rustdesk, up to 900 s).
  - `:156` the chain stops before any CPU analysis if the GPU is still busy.
  - The functions are loadable with `CHAIN_LIB_ONLY=1`.
- **Prereg:** :148 (*Chain bounds*), the idle-host bullet, and the memory limit in *Disclosed limits*.
- **Smoke:** `build_smoke/chain_kill_smoke.sh` → `chain_kill_smoke.out`. The stub runner `stub_runner.sh` spawns a
  child in a new session.

  | Case | Result |
  | --- | --- |
  | A: memory cap (150 MB) | rc 143, child GONE (log `KILL TREE … sessions [600237]`) |
  | B: timeout 4 s | rc 124, child GONE (log `ORPHANS after the stage …; terminating`) |
  | C: negative control, a bare `timeout` | rc 124, child **ALIVE** (cleaned up afterwards) |
  | D: GPU busy twice, then idle | rc 0 ("GPU idle after 20s") |
  | E: GPU always busy | rc 1 ("GPU STILL BUSY after 10s") |

- **Not done:** the optional SIGTERM handler in `camt1_run.py`. The chain-level session kill covers the case, and
  adding the handler would change `gr_run`'s launcher call.

### F6 (P2): CPU gate as baseline equivalence → FIXED

- **Files:**
  - `~/motar_tmp/camt1_20261112/cpu_gate_baseline_failures.txt`: the 15 FAIL/ERROR names from
    `build_smoke/baseline_eefb832_failing_modules.log` (clean detached worktree at `eefb832`, since removed), sorted
    with `LC_ALL=C`.
  - `~/motar_tmp/camt1_20261112/cpu_gate_diff.sh LOG`: prints BASELINE_EQUIVALENT (exit 0) only on an identical set;
    otherwise BLOCK with the diff (exit 1).
- **Prereg:** :357 (*CPU gate before the GPU run: baseline equivalence*).
  - The gate is run after the commit.
  - `test_experiment_index` must pass.
  - The result is recorded as "not 0/0 (baseline-equivalent)", never PASS.
  - Any 16th failure, or a missing one, blocks the GPU run.
- **Positive and negative checks of the script:**
  - On the baseline log it reads BASELINE_EQUIVALENT.
  - On the pre-commit r2 and r3 logs it reads BLOCK, with exactly the two `test_experiment_index` lines as the difference.

### F7 (P3): positive-control scope, and binding the run-commit control → FIXED

- **Change:**
  - `g3_control` records `script_sha256` (`tools/camt1_analyse.py:421`).
  - `stage_a_binding_extra` (`:524`) requires the chain's g3-control JSON to read POSITIVE_CONTROL_PASS at half-FOV
    43.5 with this script's SHA-256.
  - `stage-a` needs `--g3-control` (`:729`).
  - The chain passes the flag (`run_camt1.sh:160`) after running `g3-control` first.
- **Prereg:** :222 (TRACE_BINDING_FAIL list) and :249 (scope sentence).
- **Test:** `test_f7_bindings` (:745). Bad reading, half-FOV 33, wrong SHA, missing file and `None` each fail; the
  good file passes.
- **Smoke:** a failing control gives TRACE_BINDING_FAIL (`smoke_stage_ab_r1`).

### F8 (P3): no labels on CONTROL_FAIL → FIXED

- **Change:** `tools/camt1_analyse.py:704`.
  - `labels_before_controls` is removed.
  - On CONTROL_FAIL `labels` is `null`; per-arm labels are not written anywhere, and the printed summary prints
    `res["labels"]`.
- **Prereg:** :283.
- **Test:** `test_f8_no_labels_on_control_fail` (:794).
  - Positive: REPORTED with labels.
  - Negative: a forced control failure gives CONTROL_FAIL, `labels: null` and no `labels_before_controls` key.

### F9 (P3): geometric approximations → DISCLOSED

- **Prereg:** :335 (*Disclosed limits*). Covers the vertical test's leniency off-axis, the ±32.3° pixel coverage
  against the geometric ±33°, and IN_VIEW ≠ visible in depth below MinZ.
- No code change. This is a limit, not a rule change.

### F10 (P3): smaller notes

- **Analysis exit on rotated obstacles → FIXED.**
  - `tools/camt1_analyse.py:568`: `ValueError`, `OSError` or `KeyError` during episode extraction writes
    MEASUREMENT_INVALID with the reason.
  - Test: `test_f10_unsupported_geometry_writes_a_reading` (:770). The negative twin is the existing
    `test_rotated_obstacles_refused`, and the identity-quaternion cells pass in `TestCrashEpisodes`.
- **GPU idleness per cell → ADDED.**
  - `tools/camt1_run.py:544`: `G.gpu_apps()` runs before every cell; if busy, `gpu_busy_stop` is recorded and the
    stage reads INCOMPLETE.
  - Test: `test_gpu_idle_check_before_every_cell` (:846). Idle: IDENTITY_PASS with one cell run. Busy: INCOMPLETE
    with no cell run.
  - Smoke: `smoke_runner_loop_r1` `busy_gpu`.
- **Mirror controls → DISCLOSED** (prereg limits).
- **Environment against E3 → DISCLOSED** (prereg limits). `native_identity` covers it empirically.
- **Narrow intervals:** already disclosed.

## Reruns after the repair (exact lines)

- **`tests/test_camt1.py`:** `Ran 42 tests in 22.444s` / `OK` (`build_smoke/test_camt1_repair_r1.log`; 0 skipped,
  real E3, K2E and G3 data present).
- **G3 positive control:** POSITIVE_CONTROL_PASS (`build_smoke/g3_positive_control_repair_r1.json`, SHA-256
  `2d269bc9…`; the file differs from the build run's only by the new `script_sha256` field).

  | Condition | Crash episodes | out_last | never_in_1s | At 33° (out_last) |
  | --- | --- | --- | --- | --- |
  | A | 475 | 48.8421 % | 33.6842 % | 56.0 % |
  | B_out | 5,138 | 72.0708 % | 56.4617 % | 79.99 % |
  | C | 5,109 | 72.8518 % | 56.7430 % | 80.23 % |

  Every value matches the record.
- **Stage B controls:** `build_smoke/stage_b_controls_repair_r1.json`, byte-identical to the build run (SHA-256
  `6fb094ce…`).
  - K1s1−K2s1 WORSE +3.54 pp; K1s2−K2s2 WORSE +4.93 pp.
  - Both reverses SMALL_REDUCTION.
  - Null NEGLIGIBLE +0.42 pp.
- **Runner `plan` mode:** `build_smoke/plan_smoke_r2.json`. Cells: identity 1, trace 24, control 3, envelope 36.
  The preflight lists only the expected pre-commit blockers (dirty tree, no approval file, new files uncommitted).
- **Loop smokes:** `build_smoke/smoke_runner_loop_r1.out.json`.
  - Positive path: IDENTITY_PASS, NONINTERFERENCE_PASS, CONTROL_PASS, COMPLETE.
  - Negatives: NOT_REPRODUCED; a busy GPU gives INCOMPLETE; TRACE_INTERFERES; the rate gate stops a slow stage.
- **Stage A and B end to end on stand-in data:** `build_smoke/smoke_stage_ab_r1.out.json`.
  - Stage A: DESCRIBED (reasons empty). It gives TRACE_BINDING_FAIL with a failing positive control and with
    NOT_REPRODUCED.
  - Stage B: REPORTED. It gives EVAL_BINDING_FAIL on NOT_REPRODUCED.
- **Full PUBLIC_CPU** (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= nice -n 10
  timeout 3600 python -B tools/run_tests.py`, real index): `Ran 3977 tests in 773.531s` /
  `FAILED (failures=9, errors=8, skipped=18)` (`build_smoke/public_cpu_full_r3.log`).
  - **Mechanical diff** (`cpu_gate_diff.sh`): BLOCK before the commit. The 15 base names, plus exactly
    `FAIL: test_every_member_pattern_matches_a_record` and `FAIL: test_read_first_and_lineage_paths_exist`
    (`test_experiment_index`), which fail because the preregistration is untracked.
  - With the preregistration in a temporary git index (`GIT_INDEX_FILE`, a copy; the real index untouched; this
    writes one loose blob into the shared object store), `test_experiment_index` reads `Ran 9 tests` / `OK`.
  - The gate must read BASELINE_EQUIVALENT after the commit.

## Declined or deferred

- **F5 optional runner SIGTERM handler:** not added. The reason is given under F5.
- **F6 "after the commit" rerun:** not possible in this round, because commits are out of scope. It is the next owner's
  step, and the prereg makes it a GPU precondition.
- **A re-review (r2)** of the repaired diff is still needed before the commit (F1–F4 change frozen rules). The prereg
  status says so.
