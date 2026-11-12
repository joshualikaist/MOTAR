# CAM-T1 repair r2 (answers `review_pre_r2/REVIEW.md`, PASS_WITH_FIXES), 2026-11-12

- **Author:** Claude Code build subagent (Opus 5.5). CPU only, no GPU, no commit.
- **Edits:** `src/MOTAR-camonly-t1` (branch `camonly/t1`, HEAD `eefb832`, nothing staged) and `~/motar_tmp/camt1_20261112/`.
- **Not touched:** the review folders and `MOTAR-camonly`.
- **Line numbers** refer to the files after this repair. "prereg" is `docs/plans/prereg_camonly_t1_2026-11-12.md`.

## Finding → change → test or evidence

### N1 (P2): gate accepted partial logs and was not bound to the launch → FIXED

- **`cpu_gate_diff.sh LOG [HEAD]`** (rewritten). It reads BASELINE_EQUIVALENT only when all of these hold:
  - run_tests.py's `"profile": "PUBLIC_CPU"` report block is present and agrees with the last `Ran N tests` line;
  - N ≥ 3,977 and skipped = 18;
  - failures + errors = 15;
  - the sorted (`LC_ALL=C`) FAIL/ERROR set equals `cpu_gate_baseline_failures.txt`;
  - the log is newer than HEAD's commit time.

  Otherwise it reads BLOCK. In both cases it writes `$CPU_GATE_DIR/cpu_gate_<HEAD>.json` (default: the unit folder)
  with the HEAD, the log path, the log SHA-256, the report, the counts, the set differences, the problems and the
  verdict. It never overwrites a record: a second run for the same HEAD reads BLOCK.
- **`run_camt1.sh`:**
  - `:125` `cpu_gate_ok` checks the record;
  - `:149` the chain stops (exit 2, logged, `chain.pid` removed) unless `cpu_gate_<HEAD>.json` exists for the HEAD it
    launches and reads BASELINE_EQUIVALENT. This check runs before the approval-file check.
- **Prereg:** :377–389 (≥ 3,977 tests, 18 skipped; the record; the chain's refusal).
- **Smoke:** `build_smoke/gate_smoke.sh` → `gate_smoke.out`, with gate records in `build_smoke/gate_smoke*/`.

  | Case | Verdict | Reason or detail |
  | --- | --- | --- |
  | baseline 127-test log | BLOCK | no report block |
  | real pre-commit r3 log | BLOCK | 17 failures; the 2 `test_experiment_index` lines |
  | synthetic full log with exactly the 15 names (r3 minus those 2 lines, failures 7) | BASELINE_EQUIVALENT | — |
  | same, run again | BLOCK | the record exists |
  | skipped 17 | BLOCK | |
  | ran 3,900 | BLOCK | |
  | no report block | BLOCK | |
  | commit time after the log | BLOCK | |
  | chain, record missing | exit 2 | |
  | chain, BLOCK record | exit 2 | |
  | chain, record for another HEAD | exit 2 | |
  | chain, BASELINE_EQUIVALENT record (positive) | passes the gate | then stops at the missing `execution_approval.json`; no `chain.pid` left |

### N2 (P3): unbounded GPU query in the chain; uncaught error in the runner → FIXED

- **Chain:**
  - `run_camt1.sh:41`: `GPU_APPS_CMD` runs under `timeout -k 5 30`.
  - `:114`: `gpu_wait_idle` treats a non-zero query exit as "not idle". The 900 s bound then applies, and the chain
    stops without analysing.
- **Runner:** `tools/camt1_run.py:547` catches `subprocess.TimeoutExpired` and `OSError` around `G.gpu_apps()` before
  each cell, records `gpu_check_error`, and the stage reads INCOMPLETE.
- **Tests:** `tests/test_camt1.py:917` (in `test_gpu_idle_check_before_every_cell`). Both a `TimeoutExpired` and an
  `OSError` give INCOMPLETE with no cell run; the idle case (IDENTITY_PASS) stays as the positive twin.
- **Smoke:** `gate_smoke.out`. A failing query gives `gpu_wait_idle` rc 1 ("GPU query failed"); rustdesk only gives
  rc 0.

### N3 (P3): orphan within one poll → DISCLOSED

- Prereg :362 (*Orphan gap*). The chain header also names the gap.
- No code change. The `pgrep -f` idea does not work, because the stage folder is in the launcher's environment, not
  in its command line.

### N4 (P3): no grace period for the GPU check → DISCLOSED

- Prereg :129: no grace period, only rustdesk excluded, and a stopped stage is rerun in a new parent folder.

### N5 (P3): wrong test-command form → FIXED in the text

- The prereg (:10) and the WORKLOG name the discover form,
  `python -B -m unittest discover -s tests -p test_camt1.py`, and note that the module form does not import.

### N6 (P3): `TypeError` missed → FIXED

- `tools/camt1_analyse.py:568` now catches `(ValueError, OSError, KeyError, TypeError)`.
- **Test:** `test_n6_missing_dt_writes_a_reading` (`tests/test_camt1.py:803`).
  - The raw `cell_episodes` call on a trace without `dt_s` raises `TypeError`.
  - `stage_a` writes MEASUREMENT_INVALID and an output file.

### N7 (P3): records → FIXED

- **Status wording:** the prereg *State* (:3) and *Status* lines, the EXPERIMENTS.md CAM-T1 bullet, and the WORKLOG
  headline and *Next* now read "reviewed r1 + r2, ready to commit; GPU pending the user's approval".
- **r2 record:** added under *Review before the commit* (prereg :411), with N1–N7 dispositions.
- **WORKLOG:** the build bullet is marked "as built, before repair r1". The build test line names the discover form.
- **Reruns:** one sentence in the prereg's VOID rules (:137) says a rerun needs a new parent folder and new analysis
  output names.
- **Test fix:** the "nonce" case (`tests/test_camt1.py:376`) now changes `condition.evaluation_nonce`. The
  checkpoint-path case is kept as its own case, and a changed condition key (`bars`) is added as the negative.

## Reruns after repair r2 (exact lines)

- **`tests/test_camt1.py`, discover form:** `Ran 43 tests in 22.562s` / `OK` (`build_smoke/test_camt1_repair_r2.log`;
  0 skipped).
- **G3 positive control:** POSITIVE_CONTROL_PASS (`build_smoke/g3_positive_control_repair_r2.json`, SHA-256
  `a20a4fcd…`; script SHA-256 `4728d985…`).

  | Condition | Crash episodes | out_last | never_in_1s | At 33° (out_last) |
  | --- | --- | --- | --- | --- |
  | A | 475 | 48.8421 % | 33.6842 % | 56.0 % |
  | B_out | 5,138 | 72.0708 % | 56.4617 % | 79.99 % |
  | C | 5,109 | 72.8518 % | 56.7430 % | 80.23 % |

  Every value matches the record.
- **Stage B controls:** byte-identical to the earlier runs (SHA-256 `6fb094ce…`).
  - K1s1−K2s1 WORSE +3.54 pp; K1s2−K2s2 WORSE +4.93 pp.
  - Both reverses SMALL_REDUCTION.
  - Null NEGLIGIBLE +0.42 pp.
- **Runner `plan`:** `build_smoke/plan_smoke_r3.json`. Cells 1 / 24 / 3 / 36. The preflight lists only the pre-commit
  blockers: dirty tree, no approval file, 5 new files uncommitted.
- **Full PUBLIC_CPU** (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= nice -n 10 timeout
  3600 python -B tools/run_tests.py`): `Ran 3978 tests in 764.662s` / `FAILED (failures=9, errors=8, skipped=18)`
  (`build_smoke/public_cpu_full_r4.log`).
- **Hardened gate on that log:**

  ```
  BLOCK: ran 3978, skipped 18, failures+errors 17; failures + errors = 17, not 15; FAIL/ERROR set differs from the base: 0 only in the base, 2 only in this run
    > run only:  FAIL: test_every_member_pattern_matches_a_record (test_experiment_index.ExperimentIndexTest)
    > run only:  FAIL: test_read_first_and_lineage_paths_exist (test_experiment_index.ExperimentIndexTest)
  ```

  The record was written to `build_smoke/gate_smoke/cpu_gate_r4_precommit_eefb832.json`; no gate record exists in the
  unit folder. This is BLOCK only because of the two lines from the untracked preregistration, as expected before the
  commit.

## After the commit (not done here)

1. `tests/test_experiment_index.py` must pass.
2. Run the full PUBLIC_CPU with the command above, then `cpu_gate_diff.sh <log>`. It must read BASELINE_EQUIVALENT and
   write `~/motar_tmp/camt1_20261112/cpu_gate_<HEAD>.json`.
3. Get the user's GPU re-confirmation, then write the approval file naming SCOPE and HEAD.
