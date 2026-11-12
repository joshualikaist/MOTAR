# CAM-T1 v1: independent pre-commit re-review r2

- **Reviewer:** Claude Code subagent on Opus 5.5 (`claude-opus-5-5`), fresh context. Read-only on the repository;
  I wrote only under `~/motar_tmp/camt1_20261112/review_pre_r2/`. No GPU, no simulator, no commit, no git index
  write. Usage state: USAGE_UNKNOWN.
- **Object:** worktree `src/MOTAR-camonly-t1`, branch `camonly/t1`, HEAD `eefb832`. The diff is uncommitted (same 7
  `git status` entries before and after this review, and no new `.pyc` in the tree).
  - In the repository: the preregistration `docs/plans/prereg_camonly_t1_2026-11-12.md`, `tools/camt1_run.py`,
    `tools/camt1_analyse.py`, `tests/test_camt1.py` and `aerial_gym/rl_training/rl_games/eval_navrl_camt1_arms.sh`,
    plus the `docs/EXPERIMENTS.md` and `WORKLOG.md` diffs.
  - Outside it: `run_camt1.sh`, `cpu_gate_diff.sh` and `cpu_gate_baseline_failures.txt`.
- **Inputs:** review r1 (`review_pre_r1/REVIEW.md`, F1–F10). I treated the builder's repair map (`REPAIR_r1.md`) as a
  set of claims and checked each one.
- **Date:** 2026-11-12.

## Verdict: PASS_WITH_FIXES

The prereg and the code may be committed. I found no P0 and no P1.

- Every r1 finding is RESOLVED or properly DISCLOSED at the lines the repair map cites.
- The identity → trace → control → envelope → analyses chain is ordered and fail-closed in both the runner and the
  analyses.
- Text, SCOPE and code agree.
- The F5 kill logic does not touch unrelated processes. I checked this by running it with bystander processes.

Two things are still needed:

1. **Record updates in the commit:** the prereg *State* line, an r2 entry under *Review before the commit*, the
   EXPERIMENTS.md status, and the WORKLOG headline and *Next*.
2. **N1 (P2) before the GPU run.** The CPU-gate script accepts a partial-suite log as BASELINE_EQUIVALENT, and
   nothing mechanically ties a gate result to the launch.

The other new findings are P3.

## Per-finding table (r1 → r2)

| ID | r1 sev. | Verdict | Evidence (file:line) | Tests: positive / negative |
| --- | --- | --- | --- | --- |
| F1 | P2 | **RESOLVED** | `camt1_analyse.py:466–493` `consistency_problems` checks per policy seed: coverage traced/(traced+missing) < 0.99, any `actions_match` None or share < 0.99, `frames_contiguous` < 0.99, VALID < 0.90, no crash. Called on the primary profile at `:574`; reasons are written to the output. Prereg `:224–231` states the same thresholds. | `test_f1_consistency_gate` (`test_camt1.py:711`). Positive: all 1.0; 100/101 = 0.990; 99 % actions. Negative: 100/102; 98 %; a None; 98 % contiguous; 89 % VALID; a seed with no crash; a seed-2-only defect names V3s2. Non-vacuous. |
| F2 | P2 | **RESOLVED** | `split_summaries` (`:496–521`): pooled and per-seed summaries and every interval over VALID only. Non-VALID episodes are reported per seed (count, nominal split, distance quantiles) and per density. The all-crash shares are kept as `sensitivity_all_crashes`. Guidance uses VALID shares (`:578`). Prereg `:196–200`. I also checked the prereg's example "3 of 161 out-of-bounds in E3 V3s1 70/4501" against E3's `result.json` `crash_causes`: it holds. | `test_f2_shares_over_valid_only` (`:730`): 40 invalid BLIND excluded (IN_VIEW reads 1.0), counted separately, still in the sensitivity (40/150). |
| F3 | P2 | **RESOLVED** | `camt1_run.py:289–294`: `identity_pass` = both ledgers byte-equal, outcome counts equal and `result_equal` (`gr_run.identity_compare`, which drops the nonce and the checkpoint). Used by `identity_reading` (`:302`) and `control_reading` (`:309–315`). Trace noninterference still uses `pass`, as the prereg says (`:73–80`). | `test_result_equal_is_required_for_identity` (`:361`): a changed `crash_causes` with byte-equal ledgers gives `identity_pass` false and NOT_REPRODUCED; a changed checkpoint path alone still passes; an empty dict gives NOT_REPRODUCED. `test_control_reading` covers `identity_pass` false → CONTROL_FAIL. |
| F4 | P2 | **RESOLVED** | `IDENTITY_PLAN` (`:107`); stage `identity` with a 0.25 h cap; `stage_dependency` (`:412–435`): trace needs exactly one IDENTITY_PASS on HEAD in the same parent. Analyses bind it: Stage A `stage_a_binding_extra` (`camt1_analyse.py:524–533`), Stage B `:656–659`. Chain `run_camt1.sh:134–154`: identity first, and a non-zero exit stops before any trace cell. Prereg `:65`, `:81–84`, `:92–97`. | `test_stage_dependency` (`:402`): trace is refused with no identity, refused on NOT_REPRODUCED, accepted on IDENTITY_PASS. `test_f7_bindings`: identity on another commit fails. `test_f8…`: Stage B reads EVAL_BINDING_FAIL on NOT_REPRODUCED. `test_gpu_idle_check…` runs the real identity loop to IDENTITY_PASS. |
| F5 | P2 | **RESOLVED** (P3 residue: N2, N3) | `run_camt1.sh:43–113` (`descendants`, `record_sessions`, `kill_tree`, `reap_sessions`, `run_capped`, `gpu_wait_idle`) and `:156`. | **Re-run by me** (`chain_smoke/smoke.sh` → `smoke.out`). Memory cap: rc 143, child in its own session GONE. Timeout: rc 124, orphan GONE (log "ORPHANS … terminating"). **Bystanders untouched:** a same-user process in another `setsid` session and one in the caller's own session both stayed ALIVE. The builder's bare-`timeout` negative control (child ALIVE) is in `build_smoke/chain_kill_smoke.out`. |
| F6 | P2 | **PARTIAL** (see N1) | Prereg `:357–372` specifies the gate correctly. `cpu_gate_diff.sh` diffs the sorted FAIL/ERROR set mechanically. But it accepts any log with a `Ran N tests` line, including a partial run, and no launch step checks a gate record. | **Re-run by me** (`cpu_gate_diff_runs.txt`). Baseline log: BASELINE_EQUIVALENT. r2 and r3 full logs: BLOCK, exactly the two `test_experiment_index` lines. Real-index failing-modules log: BLOCK (3 OMP lines). The baseline file equals the baseline log's sorted FAIL/ERROR lines. |
| F7 | P3 | **RESOLVED** | `g3_control` records `script_sha256` (`:421`). `stage_a_binding_extra` requires POSITIVE_CONTROL_PASS, half-FOV 43.5 and this script's SHA. `--g3-control` is required (`:729`). The chain runs g3-control first (`:157–160`). Prereg `:219–223`, `:234–252`. | `test_f7_bindings` (`:745`): good file passes; a FAIL reading, 33°, a wrong SHA, None and a missing file each fail. |
| F8 | P3 | **RESOLVED** | `:703–704`: on CONTROL_FAIL `labels` is null and no per-arm label is written anywhere (the contrasts hold no label). Prereg `:282–284`. | `test_f8_no_labels_on_control_fail` (`:794`): REPORTED with labels; a forced control failure gives CONTROL_FAIL, `labels: null` and no `labels_before_controls`. |
| F9 | P3 | **RESOLVED (disclosed)** | Prereg `:335–343`. | n/a (limits) |
| F10 | P3 | **RESOLVED**, with N4 and N6 residue | Analysis error → written MEASUREMENT_INVALID (`:568–572`). GPU idle check before every cell (`camt1_run.py:544–547`). Mirror controls and environment-vs-E3 disclosed (prereg `:344–347`). | `test_f10_unsupported_geometry_writes_a_reading` (`:770`), with the existing rotated-obstacle refusal and identity-quaternion passes as twins. `test_gpu_idle_check_before_every_cell` (`:846`): idle → IDENTITY_PASS with 1 cell; busy → INCOMPLETE with 0 cells. |

The repair map's file:line citations are accurate. The one exception is minor: the F1 wiring is at `camt1_analyse.py:574`, not `:536`.

## New findings

### N1 (P2, fix before the GPU run; outside the repository): the CPU gate accepts a partial run and is not bound to the launch

- **Where:** `~/motar_tmp/camt1_20261112/cpu_gate_diff.sh:10–14`; `run_camt1.sh:132` checks only that
  `execution_approval.json` exists. The prereg (`:364–372`) says the gate "blocks the GPU run".
- **Failure scenario 1 (demonstrated):** `cpu_gate_diff.sh build_smoke/baseline_eefb832_failing_modules.log` prints
  `BASELINE_EQUIVALENT: Ran 127 tests …` and exits 0 (`cpu_gate_diff_runs.txt`).
  - Any log with a `Ran N tests` line and the same 15 failing names passes. Examples: only the failing modules plus
    `test_experiment_index`; a `-k`-filtered run; a run whose discovery silently dropped `test_camt1.py`.
  - Newly skipped tests also pass unnoticed: the skip count is not compared. `test_camt1`'s data-dependent tests
    skip when the E3, K2E or G3 folders are absent.
- **Failure scenario 2:** nothing mechanical connects a BASELINE_EQUIVALENT result for HEAD to the chain or the
  runner. The approval file can be written and the chain launched with no gate log at all. Enforcement today is
  procedural only.
- **Fix (≈10 lines, outside the repo, no prereg change needed):**
  - In `cpu_gate_diff.sh`, also require:
    - the run_tests PUBLIC_CPU report (the log's `"PUBLIC_CPU"` JSON block);
    - `Ran N` ≥ 3977 (the r3 count; after the commit it should be exactly 3977);
    - `skipped=18`.
  - Write `cpu_gate_<HEAD>.json` (HEAD, the log path and its SHA-256, Ran, the skipped count, the verdict).
  - In `run_camt1.sh`, refuse to start unless `cpu_gate_$H.json` exists and reads BASELINE_EQUIVALENT for `$H`.
  - Optionally, add one clause to prereg `:366` before the commit: "a complete run (≥ 3,977 tests, 18 skipped)".

### N2 (P3): an unbounded or unhandled `nvidia-smi` can hang the chain, or end a stage with no report

- **Where:** `run_camt1.sh:37,109` (`GPU_APPS_CMD` has no `timeout`); `camt1_run.py:544`, which calls
  `G.gpu_apps()` before every cell. `gr_run.gpu_apps` uses `timeout=30` but raises `TimeoutExpired` or `OSError`,
  and `run_stage` does not catch either.
- **Scenario:**
  - A wedged driver makes `nvidia-smi` block. `gpu_wait_idle` then never returns: its 900 s bound is only checked
    between calls. The chain hangs with no log line.
  - Inside the runner, the same event raises mid-stage. The runner exits with a traceback and writes no stage
    report. That is fail-closed, but the cells already run are left without a reading.
- **Fix:** `GPU_APPS_CMD="timeout 30 nvidia-smi …"`. In `run_stage`, wrap the call: on an exception, record
  `gpu_check_error` and stop the stage as INCOMPLETE.

### N3 (P3): a launcher orphaned within one watchdog poll is not reaped

- **Where:** `run_camt1.sh:92–103`. Sessions are recorded every `POLL_S` (10 s).
- **Scenario:**
  - The chain's `timeout` fires within 10 s of the runner starting a new launcher. That launcher's session is
    never recorded, and after the runner dies it is no longer a descendant.
  - `reap_sessions` misses it. The orphan cell runs to completion, about 3 min, writing into the stage folder.
  - The final `gpu_wait_idle` then waits it out and the analyses proceed. Stage-to-stage, the next runner's
    preflight GPU check would refuse.
  - Unlikely in practice: the runner's own budget (per-cell timeout ≤ remaining budget) ends 600–900 s before each
    chain timeout. The memory-cap path is not affected, because `kill_tree` re-records sessions at kill time.
- **Fix (optional):** after `wait`, also `pgrep -u $(id -u) -f "$RAW/<stage>_01"` (the launcher's command line or
  environment carries the out dir) and kill and log matches. Or state the gap in the prereg's memory and bounds
  bullet.

### N4 (P3): the per-cell GPU idle check has no grace, so one transient client voids a stage

- **Where:** `camt1_run.py:544–547`.
- **Scenario:**
  - Today the only compute client on the host is `/usr/share/rustdesk/rustdesk`. I checked this read-only with
    `nvidia-smi --query-compute-apps`. Both the runner (`ALLOWED_GPU_APPS`) and the chain (`grep -v -i rustdesk`)
    exclude it, so it does **not** block cells.
  - Any other short-lived CUDA client between cells stops the stage as INCOMPLETE. That covers another user
    session's tool, or a desktop app that opens a CUDA context for video decode.
  - Because outputs, `open("x")` reports and the chain's fixed `RAW` refuse reuse, the unit then needs a fresh
    parent folder and a new launch. This is consistent with the idle-host rule, so it is fail-closed rather than
    wrong.
- **Fix (optional):** re-poll for up to about 60 s before stopping, and record every poll. Or keep the behaviour
  and say in the prereg that a stopped stage is rerun only in a new parent folder (see N7).

### N5 (P3): the review brief's test command does not work as written

- `python -B -m unittest tests.test_camt1` fails with `ModuleNotFoundError: No module named 'tests.test_camt1'`
  (`test_camt1_module_form.log`), because `tests/` has no `__init__.py`. This is not a code defect.
- The repository's form, `python -B -m unittest discover -s tests -p test_camt1.py` (as in r1 and the repair log),
  gives **Ran 42 tests / OK**, 0 skipped (`test_camt1.log`).
- **Fix:** name the discover form in the WORKLOG and the prereg's commands so the next owner does not get a false
  ERROR.

### N6 (P3): Stage A's "write a reading, never exit silently" misses `TypeError`

- **Where:** `camt1_analyse.py:568`, which catches `(ValueError, OSError, KeyError)`; `cell_episodes:259` calls
  `float(meta.get("dt_s"))`.
- **Scenario:** trace metadata without `dt_s` (None) raises `TypeError`. The analysis then exits with a traceback
  and no output file, which is the case F10 meant to close.
- **Related (conservative, no change needed):** the try block covers all four profiles. An error that only a
  sensitivity profile meets (for example `all_episodes`, scope "all") makes the whole Stage A MEASUREMENT_INVALID.
- **Fix:** add `TypeError` to the tuple. Optionally, evaluate `primary` first and record sensitivity-profile errors
  separately.

### N7 (P3): records and rerun wording before the commit

- These still say "pending re-review" and must be updated in the commit: the prereg `:3` *State* line and
  `:396` *Status*; the EXPERIMENTS.md CAM-T1 bullet; the WORKLOG headline and *Next*.
- Add an r2 entry (this review, PASS_WITH_FIXES, N1–N7 dispositions) under *Review before the commit*.
- The WORKLOG entry's *Built* bullet still describes the pre-repair runner: no identity stage, "control (native and
  K=1/D=0 hook identity …)", caps "2.0 / 0.5 / 2.5 h". It is accurate as chronology, because the repair bullet
  follows, but mark it "as built, before repair r1" to avoid misreading.
- **Rerun rule:** the runner (`out_root.exists()`, a single report per parent), the analyses (`open("x")`) and the
  chain (fixed `RAW`, `g3_positive_control.json`, `stage_*_analysis.json`) all refuse reuse. A rerun after VOID or
  INCOMPLETE therefore needs a new parent folder and new analysis output names. One sentence in the prereg's VOID
  rules would make that explicit.
- Small test nit: the "nonce" case in `test_result_equal_is_required_for_identity` actually changes the checkpoint
  path. The nonce removal is covered by `K.comparable`, which `identity_compare` uses, but this test never sets a
  different `evaluation_nonce`.

## Regression check (task 2)

- **Identity stage and ordering.**
  - Identity has no prerequisite and refuses a second identity report in the same parent. Trace needs exactly one
    IDENTITY_PASS on HEAD; control needs exactly one finished trace reading; envelope needs exactly one
    CONTROL_PASS.
  - Each stage compares `env_allowlisted` and `--e3-raw` with the previous one.
  - The runner exits 0 only for IDENTITY_PASS, NONINTERFERENCE_PASS, TRACE_INTERFERES, CONTROL_PASS or COMPLETE, so
    the chain continues after TRACE_INTERFERES, as the prereg `:95–97` says, and stops on everything else.
  - E3 references for the identity block are included in `e3_problems` (`:354`).
  - Chain timeouts exceed the runner caps: 1500/900, 8100/7200, 2400/1800 and 9900/9000 s.
- **Analysis bindings.**
  - Stage A requires trace finished and in plan order, cells passing the runner checks again, E3 at the manifest,
    the script equal to the run commit, IDENTITY_PASS on the same commit beside it, and the run-commit G3 control
    at the script's SHA.
  - Stage B requires CONTROL_PASS, COMPLETE with 36 cells in order, one commit, IDENTITY_PASS on the same commit,
    cells passing again, determinism, E3 at the manifest and the script at the run commit.
  - The builder's stand-in smoke (`build_smoke/smoke_stage_ab_r1.out.json`) shows positive and negative cases for
    both.
- **`result_equal` false-negative risk (F3).**
  - In E3's V3s1 205/4501 `result.json`, the only run-specific fields are `evaluation_nonce` (dropped) and
    `checkpoint` (dropped). No path, launcher or time field exists. Condition keys with "sha" or "file" are
    equal-required by `camt1_problems` anyway.
  - E3's own `identity_01` (a different commit and tree from CAM-K2-E) recorded `result_equal: true`, and G7
    required it for its K1/D0 hook cell.
  - A spurious NOT_REPRODUCED is therefore unlikely.
- **F5 kill scope.**
  - Signals go only to: the `timeout` pid; its ppid-descendants; and `pkill -u <uid> -s <sid>` for session ids
    seen on descendants. The chain's own session is excluded.
  - Launcher sessions are created by `start_new_session` (sid = launcher pid).
  - `pid_max` is 4,194,304, with current pids around 650k, so sid reuse within a stage is negligible.
  - Other users' processes are excluded by `-u`. I ran bystanders: untouched.
  - Waits are bounded: the grace is `sleep`; GPU waits are 900 s (except N2's `nvidia-smi` hang); `wait` follows
    KILL.
- **GPU idle check:** see N4. rustdesk, the one current client, is allowed consistently in the runner and the
  chain. Graphics-only clients (Xorg or the desktop shell) are not listed by `--query-compute-apps`. A process
  remaining from a previous cell is not expected, because the launcher runs `play_navrl.sh` in the foreground and
  `realveh_python` `exec`s.
- **Prereg / SCOPE / code consistency.**
  - SCOPE (`camt1_run.py:73–77`): 1 + 24 + 3 + 36 = 64 cells, caps 0.25 / 2.0 / 0.5 / 2.5 h. That matches
    `MAX_HOURS`, the prereg table (`:63–68`), the budget (`:142`, 5.25 h) and `plan_smoke_r2.json` (1/24/3/36,
    arms and extras as specified, `NAVRL_KROC2027_OCCLUSION_TRACE` only on trace cells, the hook only on
    k1d0/d1).
  - The docstrings of both scripts match the prereg's rules.
- **Launcher:** the diff against `eval_navrl_camk2e_arms.sh` is only the header, the `[camt1]` case guard and the
  `NAVRL_SPEED_GOVERNOR_FREE_MPS` line. SHA-256 is `c166f560…` = `LAUNCHER_SHA`.

## CPU gate (task 4)

- **Correctly specified?** Yes in intent (prereg `:357–372`):
  - run after the commit;
  - `test_experiment_index` passes;
  - the full PUBLIC_CPU command;
  - the failure set equals the 15 base names exactly;
  - recorded as "not 0/0 (baseline-equivalent)", never PASS;
  - any 16th or missing failure blocks.
- **Is the baseline sound?** The 15 names equal the FAIL/ERROR lines of the clean-worktree base log. The pre-commit
  full runs (r2, r3) differ from the baseline only by the two `test_experiment_index` failures, which the uncommitted
  prereg explains.
- **Mechanically enforced?** Only partly.
  - The set diff is mechanical and correct on full logs.
  - It does not establish completeness of the run (N1, demonstrated with the 127-test log).
  - It is not bound to HEAD or checked by the chain or runner before launch.
  - Fix N1 before the approval file is written.

## What I verified by running vs by reading

**Running** (CPU only, `CUDA_VISIBLE_DEVICES=` empty, `PYTHONNOUSERSITE=1`, `-B`, `PYTHONDONTWRITEBYTECODE=1`,
`TMPDIR` in my folder, nice 10; outputs in this folder):

| Check | Output | Result |
| --- | --- | --- |
| `python -B -m unittest tests.test_camt1` (as briefed) | `test_camt1_module_form.log` | ImportError: `tests` is not a package (N5) |
| `python -B -m unittest discover -v -s tests -p test_camt1.py` | `test_camt1.log` | **Ran 42 tests, OK**, 0 skipped (E3, K2E and G3 data present) |
| G3 positive control `camt1_analyse.py g3-control …/camonly_c2trace_v1` (ulimit -v 16 GB) | `g3_positive_control_review.{json,log}` | **POSITIVE_CONTROL_PASS**. A / B_out / C = 475 / 5,138 / 5,109 episodes; out_last 48.8421 / 72.0708 / 72.8518 %; never_in_1s 33.6842 / 56.4617 / 56.7430 %, all matching the record; at 33°: 56.0 / 79.99 / 80.23 %. SHA-256 `2d269bc9…` is **byte-identical** to the builder's `g3_positive_control_repair_r1.json`. Script SHA `9d039b38…` |
| `cpu_gate_diff.sh` on the baseline, r2, r3 and real-index logs | `cpu_gate_diff_runs.txt` | Baseline: BASELINE_EQUIVALENT (127 tests: N1). r2 and r3: BLOCK, exactly the 2 `test_experiment_index` lines. Real-index: BLOCK (3 OMP-thread errors) |
| Baseline file vs the base log | inline | The sorted FAIL/ERROR lines of `baseline_eefb832_failing_modules.log` equal `cpu_gate_baseline_failures.txt` |
| F5 chain kill smoke, with bystanders | `chain_smoke/{smoke.sh,smoke.out,chain.log}` | Cap: rc 143, child GONE. Timeout: rc 124, orphan GONE. Both bystanders ALIVE. A first attempt showed false "ALIVE" readings because my own shell's command line contained the marker strings; it was rerun cleanly |
| Host facts (read-only) | inline | `pid_max` 4,194,304; current compute apps: rustdesk only; GPU RTX 3070 |
| E3 facts | inline | E3 V3s1 70/4501 `crash_causes`: 3 of 161 out_of_bounds. E3 `identity_01`: `result_equal` true for its identity cells. E3 205/4501 `result.json` has no path or time fields besides the nonce and checkpoint |
| Repository untouched | `git_status_before.txt` and after | Identical; no new `.pyc` |

**Reading:**

- the full `camt1_run.py`, `camt1_analyse.py`, the prereg, `run_camt1.sh`, `cpu_gate_diff.sh`, and the repair-relevant
  tests (`test_camt1.py:92–116`, `336–445`, `679–904`);
- `gr_run.py` (`gpu_apps`, `run_launcher`, `run_cell`, `check_cell`, `identity_compare`) and
  `kroc2027_eval.comparable`;
- `camk1e_run.approval_problems` and `actionhold_run`'s G7 identity rule;
- the launcher diff and `tools/realveh_python`;
- `run_tests.py`'s environment handling;
- the EXPERIMENTS.md and WORKLOG.md diffs;
- the builder's smokes (`plan_smoke_r2.json`, `smoke_runner_loop_r1.out.json`, `smoke_stage_ab_r1.out.json`,
  `chain_kill_smoke.*`).

**Not verified:** `test_experiment_index` passing with the files committed (that needs an index write); the
post-commit full PUBLIC_CPU; anything on GPU.

## Summary

| ID | Severity | Topic | When |
| --- | --- | --- | --- |
| F1–F5, F7–F10 | (r1) | Resolved or disclosed as cited | — |
| F6 | (r1 P2) | PARTIAL: specified correctly, enforcement incomplete → N1 | before GPU |
| N1 | P2 | CPU gate accepts a partial-suite log; no gate record checked before launch | fix before the approval file / launch (outside repo) |
| N2 | P3 | `nvidia-smi` without timeout in the chain; uncaught `gpu_apps` exception in `run_stage` | should fix |
| N3 | P3 | Launcher orphaned within one 10 s poll by the chain timeout is not reaped | optional / disclose |
| N4 | P3 | Per-cell GPU check has no grace; one transient client voids the stage | optional / disclose |
| N5 | P3 | `-m unittest tests.test_camt1` does not work; use the discover form | note |
| N6 | P3 | Stage A error path misses `TypeError` | should fix (one token) |
| N7 | P3 | Update status lines and add r2 to the prereg/EXPERIMENTS/WORKLOG; rerun-folder sentence; test nit | in the commit |

Files written by this review: `REVIEW.md`, `test_camt1.log`, `test_camt1_module_form.log`,
`g3_positive_control_review.{json,log}`, `cpu_gate_diff_runs.txt`, `git_status_before.txt`,
`chain_smoke/{smoke.sh,smoke.out,chain.log,kill_a.log,kill_b.log}`, `tmp/` (test temporary directory).
