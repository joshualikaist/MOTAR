# S3a-P1 policy precision control: independent review r2

- **Verdict: PASS_WITH_LIMITS.** No BLOCKER or MAJOR finding is open. Round 1's MAJOR (M1) is resolved in its core:
  the GPU run is bound to one fixed directory, created before CUDA starts, and a second attempt is refused. The
  in-tool fault count works for runs that finish. What remains is MINOR: an attempt can be spent, or a finished GPU
  measurement can end, without a result record (N1); the fixed reading mislabels mixed per-density outcomes (N2);
  and the preregistration's preflight provenance is inaccurate (N3).
- **Counts (open after r2):** BLOCKER 0, MAJOR 0, MINOR 5 (m3 and m5 residuals, N1, N2, N3), LOW 14 (l1–l7 residuals,
  N4–N10).
- **Reviewer:** Claude Code subagent, model Claude Opus 5.5, fresh context. This was a read-only audit of the actual
  files. Writes went only to `~/motar_tmp/s3a_policy_precision_20261106/review_r2/`. No GPU and no simulator were
  used: every Python process ran with `CUDA_VISIBLE_DEVICES=""` on cores 20–23, under `nice 10`, a 12 GiB `ulimit -v`
  and `timeout -k`.
- **Date:** 2026-11-06, 02:59–03:25 KST.
- **Object:** worktree `MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb766`. The three files are untracked.

## Hash checks

`sha256sum -c BYTES_UNDER_REVIEW.sha256` returned OK for all three files at the start (02:59) and again at the end
(03:10, after every probe):

| File | SHA-256 |
| --- | --- |
| `tools/sitl_s3a_policy_precision.py` | `26799cc6…e908` |
| `tests/test_sitl_s3a_policy_precision.py` | `228be5a2…e92f` |
| `docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md` | `8e942104…f5bf` |

`~/motar_tmp/s3a_policy_precision_20261106/gpu_run01` did not exist at the start or at the end. The probes
redirected `run_dir` into this review directory.

## Round-1 findings

| ID | Round-1 issue | r2 status | Evidence and remaining gap |
| --- | --- | --- | --- |
| **M1** (MAJOR) | No single-attempt binding, no in-tool host-fault record, `--out` anywhere | **Resolved as MAJOR.** The residual is MINOR (N1). | `RUN_DIR` is fixed (`:46`) and created with `mkdir(exist_ok=False)` before torch is imported (`:330`). `--out` is refused for `--execute` (`:321`). There is no CLI override of `run_dir`. A second attempt is refused with rc 2 and nothing is written (probe B). `kernel_faults()` runs before and after; a change gives VOID_HOST_FAULT with `reading_if_valid` kept (probe C, both for a rise and for a fall). The accept path works end to end (probe A). **Gap:** no attempt record is written before the measurement. An aborted, killed or precondition-failed attempt therefore leaves an empty `gpu_run01` with no fault count and no approval hash (N1). |
| **m1** (MINOR) | NaN mis-read; truncated JSON | **Resolved.** | `compare()` returns `finite: False` and `max: None` (`:95-98`). `reading()` checks NONFINITE_OUTPUT before any `max` comparison, so `None > float` cannot be reached (`:152-154`). `_write_json` serialises before it opens the file (`:295-298`). End to end, a NaN in GPU_ON reads NONFINITE_OUTPUT and the JSON parses under a strict parser (probe E). Other paths still leave an npz without a JSON (N1). |
| **m2** (MINOR) | ON against OFF not reported; an ineffective switch is indistinguishable | **Resolved.** One new issue (N2). | `gpu_on_vs_gpu_off` is recorded (`:249`). TF32_SWITCH_NO_EFFECT is preregistered (§4 item 5) and implemented (`:159-160`). The bitwise-equal counts and the ON–OFF–ON repeat suggested in round 1 were not added; both were optional. |
| **m3** (MINOR) | Important paths untested | **Partially resolved.** The residual is MINOR. | Added: the `approval_problems` positive control (test `:151`); `measure(gpu=True)` with the CPU standing in, a known-bad control that reads GPU_REPLAY_NOT_FAITHFUL (`:231-251`); a watchdog child test (`:176-185`); NaN in `compare`/`controls`; all 8 labels of `reading()`. **Still missing:** a committed test of the `main(--execute)` accept path through the guard, `mkdir`, `faults`, `measure` and the write; the VOID_HOST_FAULT path; NaN in a *recorded* action; `set_tf32` raising on a mismatch. The reviewer smoked the first two (probes A and C), and they work. |
| **m4** (MINOR) | §1 numbers quoted without scope | **Partially resolved.** The remaining part is N3. | The prior-audit scope is now stated (`prereg:19-20`). The full-grid values (1.720e-3 and 1.730e-3) are not written in the prereg. Instead they are deferred to `dev08/agq/s3a_p1/`, which does not exist yet. "About 1e-6" for CPU batching remains (round 1 measured 1.34e-6 at batch 128). |
| **m5** (MINOR) | NAVRL_* and module origins neither controlled nor recorded | **Partially resolved.** The residual is MINOR. | Recorded: NAVRL_* for each player (`:236`, `:240`) and SHA-256 plus path of every `aerial_gym.*`/`rl_games.*` file (`:206-213`). In probe A, all 11 `aerial_gym` modules lie inside ROOT, and 36 `rl_games` modules are hashed. **Still open:** the second `B.setup` is a silent no-op, so density 205 runs under the 70-bar receipt and records `NAVRL_NUM_BARS=70` (probe A). The 7 differing keys are not policy inputs. Nothing fails closed when `aerial_gym` resolves outside ROOT, which AGENTS.md requires of worktree runs, and the §5 command sets no `PYTHONPATH`. Stray NAVRL_* are not stripped. The test children still inherit `os.environ` (test `:212`, `:238`). |
| l1 | Rationale for 1e-5 not written down | Open (LOW) | Neither the code comment (`:42`) nor §4 derives the value. |
| l2 | No residual-present precondition; `CPU_ONLY` not in §4 | Open (LOW) | `CPU_ONLY` is still unnamed in §4. This is harmless with the pinned inputs, because the residual of 1.72e-3 is reproduced bitwise. |
| l3 | §3 "CPU: none (float32)" | Open (LOW) | Unchanged (`prereg:38`). The process flags are TF32 ON/`high` during the CPU run, which has no effect on CPU float32 arithmetic. |
| l4 | SDPA and cuDNN/CUDA versions; wording of RESIDUAL_NOT_TF32 | Partially resolved (LOW) | The §4 item 5 note mentions attention kernels. Still missing from `backend_flags()`: `sdp_*_enabled`, `cudnn.version()` and `torch.version.cuda`. The label still reads "NOT_TF32", not "not explained by the TF32 switches". |
| l5 | `--out` edge cases | Resolved for `--execute`, open for `--cpu-only` (LOW) | `--cpu-only --out x.npz` runs the measurement, writes `x.npz`, then fails with `FileExistsError`; no JSON is written (probe I). A missing parent also fails only after the measurement. |
| l6 | No check for other CUDA processes or the device name | Open (LOW) | `device_name` is recorded but not enforced (`:250`). |
| l7 | Preflight does not bind the tool bytes | Partially resolved (LOW) | Module hashes are recorded, but `sitl_bridge`, `sitl_bridge_offline_audit` (`build_player`, `deterministic_actions`), the cplanner replay and the tool itself are not (probe A). For `--execute` the clean commit binds the tracked files. For `--cpu-only` nothing does (see N3). |

## New behaviour checked

| Behaviour | Result |
| --- | --- |
| Fixed single-use `RUN_DIR`, created before CUDA | PASS. Nothing imports torch before `mkdir` (`:320-330`). `mkdir(exist_ok=False)` is atomic, so a concurrent second start fails. The real `RUN_DIR.parent` is outside every Git tree and is not a symlink (probe H). The run is accepted when valid (probe A) and refused when repeated (probe B). |
| Refusal inside a Git tree | PASS for `--execute` (test `:172-174`). For `--cpu-only`, the check uses `.absolute()`, so a symlink into a work tree is not detected (N6). The refusal JSON gives no reason (N7). |
| Kernel faults before/after; VOID_HOST_FAULT | PASS for completed runs (probe C). The counting matches the AGENTS.md `grep -cE` command (case-sensitive, any of three patterns). The current count is 0. The "vacuous" guard accepts a log that has no kernel entries (N8). An aborted run records nothing (N1). |
| NONFINITE_OUTPUT | PASS end to end (probe E). |
| TF32_SWITCH_NO_EFFECT and reading order | The order is CONTROL_FAILED → CPU_ONLY → NONFINITE → NOT_FAITHFUL → EXPLAINS → NO_EFFECT → NOT_TF32. It matches §4 for single-density-consistent outcomes. Mixed per-density outcomes are mislabelled (N2). |
| GPU_ON against GPU_OFF recording | PASS. Recorded: the flags right after construction, `flags_on` and `flags_off` (each read back, with a raise on mismatch), all 6 pairwise comparisons and per-row arrays in the npz. GPU_ON is measured first, GPU_OFF second, then the flags are restored. |
| NAVRL_* and loaded module files | Recorded (probe A). See the m5 residual and l7. |
| Positive control for `approval_problems` | PASS: a valid approval on a clean HEAD gives `[]` (test `:151`). The full `main` guard chain also accepts a valid case (probe A), but no committed test shows this (m3). |

**Can the GPU still run twice?** Not through the CLI. It still can through:
- deleting `gpu_run01`;
- `HOME=<other>` together with a pre-created `<other>/motar_tmp/s3a_policy_precision_20261106/`, because `RUN_DIR` uses `Path.home()`;
- calling `measure(gpu=True)` directly, which has no guard of its own (N10).

All three are deliberate circumvention or library use, not an operator slip.

**Can a partial result file be written?** Yes. Each of these paths leaves `result.npz` without `result.json`, and the
attempt is spent:
- after the GPU work has finished:
  - the post-run `faults()` raises (probe D);
  - the approval file carries a NaN in any field, which `json.loads` accepts and `approval_problems` does not check
    (probe G);
- because the process is killed (timeout, watchdog exit 3, crash).

`--execute` with CUDA hidden spends `gpu_run01` and leaves it empty with no record (probe J). See N1.

**Is any reading favourable by construction?** No.
- A device mix-up reads GPU_REPLAY_NOT_FAITHFUL (probe A: ON−recorded = 1.720e-3 > 1e-5).
- NaN reads NONFINITE_OUTPUT.
- A fault reads VOID_HOST_FAULT.
- TF32_EXPLAINS needs ON within 1e-5 of the recording and OFF within 1e-4 of CPU. Against CPU−recorded = 1.72e-3,
  this forces ON ≠ OFF.

The one by-construction bias is that RESIDUAL_NOT_TF32 absorbs mixed outcomes (N2).

**Is any valid case rejected?** None found on this host. The checks behind this:
- the journal is readable (user in `adm`, 3792 kernel lines);
- `RUN_DIR.parent` is not inside Git;
- a valid approval with `HEAD` equal to `commit` and a clean tree is accepted.

Two strictness notes:
- `commit` must be the full 40-hex SHA;
- the tree must be clean, which includes the other unit's untracked files. That is by design.

## New findings

| ID | Severity | Location | Scenario | Fix |
| --- | --- | --- | --- | --- |
| **N1** | MINOR | `tools/sitl_s3a_policy_precision.py:323-344` | The attempt directory is created before the CPU-side preconditions are checked: inputs and F pin, CUDA visibility, readability of the fault log, JSON-safe approval. No attempt record is written. Consequences: (a) `--execute` with CUDA hidden spends `gpu_run01` and leaves it empty (probe J); (b) a post-run `faults()` failure or a NaN anywhere in the approval loses `result.json` after the GPU work, leaving only the npz (probes D and G); (c) a killed or crashed run (the host-fault case) records neither `faults0` nor the approval hash. Because `RUN_DIR` is a constant named in the immutable prereg, any spent attempt also needs a code change and an amendment to rerun. | Before `mkdir`: run `verify_inputs`, check the F pin, require a non-empty `CUDA_VISIBLE_DEVICES`, call `faults()` once and run `json.dumps(approval, allow_nan=False)`. Right after `mkdir`: write `attempt.json` (`STARTED`, approval sha, `faults0`, `t0`, pid, tool sha256) with mode `"x"`. After `measure`: catch failures of `faults()` and `git_state()`, and still write `result.json`, read as VOID (`kernel_faults_end: null`). Write the npz together with the JSON, for example via `*.pending` and `os.replace`. |
| **N2** | MINOR | `:155-161`; prereg §4 items 4–6 (`:61-68`); test `:118` encodes the behaviour | Quantifiers over densities are mixed: EXPLAINS needs both, NO_EFFECT needs both, and NOT_TF32 takes everything else. Probe F: "70: switch had no effect, residual 1.7e-3; 205: TF32 explains (OFF−CPU 5e-5)" reads RESIDUAL_NOT_TF32, a cause-negative label that neither density shows. "70 explained, 205 not" (a borderline 1e-4 split is plausible) also reads RESIDUAL_NOT_TF32. Per-density metrics are recorded, so nothing is lost, but the fixed headline overclaims. The prereg is about to become immutable. | Classify each density separately (EXPLAINS / NO_EFFECT / NOT_TF32). Give the overall reading as the common label, or as a preregistered `MIXED_BY_DENSITY` when they differ, and record the per-density labels. Update §4 and test `:118`. |
| **N3** | MINOR | prereg `:5`, `:21`, `:86-88` | (a) "It ran once as a preflight of this code": `cpu_only_v1/result.json` (sha `ac940770…`) was produced by the round-1 tool bytes (`d41f…`). It lacks the r2 fields (`kernel_faults_*`, `loaded_module_files`, `navrl_env_*`) and binds no tool hash. The reviewer's run with the r2 bytes reproduces its CPU actions bitwise (0.0 at both densities, probe A), so the numbers stand. (b) §1 points to `dev08/agq/s3a_p1/`, which does not exist, uses an ambiguous relative path, and states none of the full-grid values. (c) Line 21 merges the §1 question ("Is GPU TF32 arithmetic …?") into a bullet. | Run the preflight once more with the final bytes into a new `cpu_only_v2/`, or state that v1 ran on bytes `d41f…` and that r2 reproduces them bitwise. Write 1.720e-3 (70) and 1.730e-3 (205) into §1 with their source file. Add `docs/evidence/sitl_2026-11-05/dev08/agq/s3a_p1/` in the same commit and give its full path. Restore the paragraph break. |
| N4 | LOW | test `:181`, `:183` | The watchdog children run `sys.executable -c …` without `-B` and load the tool with `spec_from_file_location`, so they write `tools/__pycache__/sitl_s3a_policy_precision.cpython-38.pyc` into the worktree. This was reproduced on a copy (`pyc_probe_r2.log`). The worktree already holds that file, mtime 02:58:42, from the author's test run. It is gitignored (`*/__pycache__/`), but it is a runtime file under `tools/`, which the freeze rule forbids. | Add `"-B"` to both child commands, or set `PYTHONDONTWRITEBYTECODE=1` in their env. Optionally set `sys.dont_write_bytecode = True` at the top of the tool. |
| N5 | LOW | test `:166-173`, `:203-206` | The in-process `main(--execute)` refusal tests pass `faults=lambda: 0` and leave `measure` real. If a guard regresses, or in a checkout without `.git` (`ROOT / "gpu_run01"` is then not "inside Git"), the test process would `mkdir` in the tree, start a watchdog thread, set torch threads to 1, import the bridge (NAVRL_* leak) and call `measure(gpu=True)` on `cuda:0` (FULL_RESEARCH keeps CUDA visible). | In those tests, patch `P.measure` and `faults` with functions that raise `AssertionError("guard did not refuse")`, or run them in a child with CUDA hidden. |
| N6 | LOW | `:280-282` | `inside_git_tree` uses `.absolute()`. A `--cpu-only --out` through a symlink into a work tree is not detected (probe H: `False`, while the resolved path gives `True`). | Check `path.resolve()` as well. |
| N7 | LOW | `:326-327` | A `run_dir` inside a Git tree is refused with `"problems": []`, so the operator sees no reason. | Append the reason, and the same for `run_dir` existing. |
| N8 | LOW | `:261-264`, `:285-292` | `git_state` ignores return codes, so a failing `git status` with a working `rev-parse` reads as clean. `kernel_faults` treats any non-empty stdout as readable; `-- No entries --` (rc 0) passes and counts 0 (`journal_guard_probe_r2.log`). On this host both work. | Check the return codes. Require at least one kernel line from this boot, for example a line containing `Linux version`. |
| N9 | LOW | prereg `:47-49`, `:54-58`, `:70`, `:96`; code `:230` | Wording does not match the code: (a) "rose" against `!=` (a fall also voids); (b) item 1's "or a compared array is not finite" against NONFINITE_OUTPUT for GPU arrays; (c) item 2's "Nothing is compared" against the finite pairs that are still computed; (d) module hashes are recorded once per result, not "for each density"; (e) the 205-bar `B.setup(receipt)` is a no-op, so density 205 records the 70-bar receipt's NAVRL_*. That record is truthful, but it is neither stated nor commented. | Align the wording. Call `setup` once, with a comment that the 7 differing keys are not policy inputs. |
| N10 | LOW | `:46`, `:216` | `RUN_DIR` depends on `HOME`. `measure(gpu=True)` has no guard of its own. Both need deliberate misuse. | Hard-code the prereg's absolute path. Optionally make `measure` refuse a CUDA device unless it is handed the created `run_dir`. |

## Tests

- **Result:** 12/12 OK in 27.8 s, rc 0, maximum RSS 0.73 GB (`test_run_r2.log`).
- **Meaningfulness:**
  - Each negative is paired with a positive: grid placement against refusals, pins, `compare`/`controls` with
    n = 40, all 8 reading labels, approval acceptance against 5 refusals, watchdog kill against survive, and the real
    preflight (residual > 1e-4 and < 1e-2, controls ok, 43008 elements).
  - The GPU branch with the CPU standing in is a claim-scale known-bad control.
  - Missing items are listed under m3.
- **Leaks into the shared process:**
  - The torch flags are restored. The flags test is correct because `matmul.allow_tf32` and the precision are one
    state.
  - `CUDA_VISIBLE_DEVICES` is restored by `mock.patch.dict`, and `sys.path` by `load()`.
  - No in-process `main` reaches `_rss_watchdog`.
  - All writes go to `tempfile` directories, and the tests assert that nothing appears in ROOT.
  - Exceptions: the bytecode leak (N4); the regression hazard (N5); the children's inheritance of NAVRL_* (m5).
- **Portability:**
  - The real-input tests skip without the G8 inputs.
  - The preflight test needs a readable `journalctl`; without one it fails rather than skips.
  - The test of the "inside a Git tree" refusal needs a `.git` (N5).

## Commands run

All Python ran under `CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 timeout -k 10
<t> nice -n 10 taskset -c 20,21,22,23 bash -c 'ulimit -v 12582912; …/aerialgym/bin/python -B …'`.

1. **Hash checks:** `sha256sum -c BYTES_UNDER_REVIEW.sha256` at the start and the end, plus `git status --short`
   (read only).
2. **Test file:** `python -B tests/test_sitl_s3a_policy_precision.py -v` from the worktree, with
   `PYTHONDONTWRITEBYTECODE=1` added so that the N4 children could not write into the tree.
   - Output: `test_run_r2.log` (`e3f856f8…`).
   - `find -newer` showed no new worktree file.
3. **Main probe:** `review_probe_r2.py` (`134dfa15…`) wrote `review_probe_r2.json` (`7ea89f77…`) and
   `review_probe_r2.log`. Its run directories are under `probe_runs/`, and the approval files there are reviewer
   fixtures with a fake commit, not approvals. Probes:
   - A: the accept path;
   - B: a second attempt;
   - C: VOID on a rise and on a fall;
   - D: a post-run fault-read failure;
   - E: NaN end to end;
   - F: mixed readings;
   - G: NaN in an approval;
   - H: Git-tree detection;
   - I: `--out x.npz`;
   - J: `--execute` with CUDA hidden.

   `find -newer` showed no new worktree file. `torch.cuda.is_initialized()` was False at the end.
4. **Bytecode leak:** `pyc_probe_r2.py` with `pyc_probe_r2.log`, run on a copy of the tool in `pyc_probe/`.
5. **Journal guard:** `journal_guard_probe_r2.log`, which ran `journalctl -k -b 0 --since 2099-01-01` and called
   `kernel_faults()`.
6. **Reading:**
   - the round-1 report and probes, `cpu_only_v1/` and the author's `focused_v*.log`;
   - `sitl_bridge.py`;
   - `install_cpu_only_aerial_gym_packages`, `read_checkpoint_schema`, `apply_schema_environment`, `build_player`
     and `deterministic_actions`;
   - `S3A_INPUT_AUDIT.md` and the parent plan section;
   - the receipts' `env_snapshot_navrl` for both densities (read only);
   - `tools/run_tests.py`, for how the profiles handle CUDA.

The round-1 probes were not re-run as written. `review_probe_execute_smoke_r1.py` passes `--out` with `--execute`,
which r2 refuses by design. Its purpose is covered by probe A, which reproduced the CPU actions of `cpu_only_v1`
bitwise (0.0) and the r1 figures 1.7202e-3 / 1.7297e-3 and 1.932 / 1.933 for the negative control.

## Limits

- No GPU was used. The following rest on reading PyTorch, cuDNN and cuBLAS behaviour, not on measurement: the GPU
  RSS staying under the 8 GiB watchdog; whether GPU_ON reproduces the recording; whether the switches reach the
  relevant kernels; and whether cuDNN benchmark selection is stable across processes.
- cuDNN benchmark selection is timing-dependent. "No other CUDA job" (§5) is an operator rule only (l6).
- Prereg §5 pins the GPU run to cores 6, 7, 12 and 13, which are currently busy. The operator must free them, or the
  prereg must change before commit.
- The full PUBLIC_CPU suite was not run. Leaks were assessed by reading the code and by the isolated test run.
- `MOTAR-cplanner/cplanner/motar/replay.py` and the site-packages `rl_games` were not re-audited in r2.
- The probes stood in for `git_state` (clean at a fake HEAD) and for the device (`"cpu"`). The real `git status`
  check on a committed tree was not exercised.
