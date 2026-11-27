# S3a-P1 policy precision control: independent review r3

- **Verdict: PASS_WITH_LIMITS.** No BLOCKER or MAJOR finding is open. Round 2's MINOR findings N1, N2 and N3 are
  resolved in their core, and the m3/m5 leftovers have shrunk to LOW. One new MINOR (N12) remains: `density_readings`
  carries cause labels even when the headline reading assigns no cause (VOID_HOST_FAULT, CONTROL_FAILED,
  GPU_REPLAY_NOT_FAITHFUL), and the preregistration does not say what the field means in those cases. Fix it before
  the preregistration is committed, because the text is immutable afterwards. Everything else is LOW.
- **Counts (open after r3):** BLOCKER 0, MAJOR 0, MINOR 1 (N12), LOW 18 (l1–l7; the m3, m5, N3, N5 and N9 residuals;
  N6, N8, N10; new N11, N13, N14).
- **Reviewer:** Claude Code subagent, model Claude Opus 5.5 (`claude-opus-5-5`), fresh context. This was a read-only
  audit of the actual files. All writes went to `~/motar_tmp/s3a_policy_precision_20261106/review_r3/`. I used no GPU
  and no simulator. Every Python process ran with `CUDA_VISIBLE_DEVICES=""` on cores 20–23, under `nice 10`, a 12 GiB
  `ulimit -v`, `timeout -k` and `PYTHONDONTWRITEBYTECODE=1`. At the end, `torch.cuda.is_initialized()` was False.
- **Date:** 2026-11-06, 03:16–03:40 KST.
- **Object:** worktree `MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb766`. The three files are untracked.

## Hash checks

`sha256sum -c review_r3/BYTES_UNDER_REVIEW.sha256` returned OK for all three files at the start (03:16) and again at
the end (03:25, after every probe):

| File | SHA-256 |
| --- | --- |
| `tools/sitl_s3a_policy_precision.py` | `edecd05da07f…a88117` |
| `tests/test_sitl_s3a_policy_precision.py` | `0d535f144f0a…2df8b9` |
| `docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md` | `bcaa2decf390…aadb9` |

- `~/motar_tmp/s3a_policy_precision_20261106/gpu_run01` did not exist at the start or at the end.
- `find <worktree> -newer review_r3/.marker_before`, excluding `.git`, listed nothing at 03:25, after all my probes
  and tests.
- At 03:27, after my last process had ended, a recheck showed that another session had changed things:
  - `tools/sitl_agq_geometry_diag.py` was modified at 03:27:06;
  - its `tools/__pycache__/sitl_agq_geometry_diag.cpython-38.pyc` was written at 03:27:08.

  This belongs to the AGQ geometry unit. None of my processes imported that module, and none of the three files
  under review changed.
- `tools/__pycache__/` contains no `sitl_s3a*` file. Round 2's leftover pyc is gone.
- **Observation outside this unit:** that other unit's run is writing a runtime pyc under `tools/`. This is the
  pattern round 2 flagged as N4.

## Provenance of the CPU preflights (question 6)

| Claim (prereg §5, `:98-100`) | Finding |
| --- | --- |
| `cpu_only_v2/` came from the bytes under review | **Supported.** `cpu_only_v2/tool.sha256` = `edecd05…`, the file under review. The tool's mtime (03:14:26) is earlier than the v2 outputs (03:15:49). My rerun with the current bytes (`review_r3/cpu_only_r3/`) gave a byte-identical npz (`83c26c21…`). Every non-volatile JSON field was equal: all densities, `loaded_module_files`, `inputs`, `git`, `density_readings` and `aerial_gym_path`. **Caveat:** `tool.sha256` was written by the author 0.3 s after `result.json`. The result itself binds no tool hash (l7). "The bytes committed with this file" is prospective. It stays true only if these exact bytes are committed. |
| `cpu_only_v1/` came from the review-r1 bytes | **Not verifiable.** v1 binds no tool hash. v1 was written at 02:29, but round 1 hashed its bytes (`d41f…`) at 02:40. Round 1 reproduced v1's CPU actions bitwise, which shows numerical identity, not byte identity. Wording fix in N3. |
| Review r2 reproduced v1's CPU actions bitwise | True (r2 probe A: 0.0 at both densities). |
| Is v2 bitwise equal to v1? | **The npz is byte-identical** (`83c26c21…` for v1, v2 and r3). `result.json` is not byte-identical, because v2 adds fields: `finite` in each comparison, `kernel_faults_*`, `loaded_module_files`, `navrl_env_cpu`, `density_readings`, `aerial_gym_path`, `fault_check_error`, `nice`. Every common value is equal. CPU−recorded is 1.7202e-3 at 70 bars and 1.7297e-3 at 205 bars. The negative controls are 1.932 and 1.933. |

## Round-2 findings

| ID | r2 severity | r3 status | Evidence and remaining gap |
| --- | --- | --- | --- |
| **N1** attempt spent or result lost without a record | MINOR | **Resolved as MINOR.** LOW residual in N11. | The refusals now happen before `mkdir` (`:349-370`): CUDA hidden (`""` or unset), directory exists, inside Git, unreadable kernel log, NaN/±Infinity tokens, approval problems. Each one leaves no directory (test `:164-195`; probes G, J). `attempt.json` is written right after `mkdir`, with the approval and its SHA-256, Git state, `faults0`, scope and prereg (`:372-374`; probe A). An exception or KeyboardInterrupt in `measure` writes `aborted.json` and re-raises (`:378-383`; probe D). A failed post-run fault read still writes `result.json` as VOID_HOST_FAULT with `fault_check_error` (`:384-394`; test `:222-234`; probe C). **Residual:** an approval with an overflowing number literal (`1e999`) gets past the strictness check and then fails `_write_json(attempt.json)` after `mkdir`. That leaves an empty, spent run directory (probe G, N11). |
| **N2** mixed per-density outcomes mislabelled | MINOR | **Resolved.** New issue N12. | Each density is classified on its own (`:146-153`). GPU_REPLAY_NOT_FAITHFUL is global (`:168-169`). Agreement gives the shared label, and disagreement gives MIXED (`:170`). `density_readings` is recorded (`:269`). Probe F: NO_EFFECT+EXPLAINS, NO_EFFECT+NOT_TF32 and EXPLAINS+NOT_TF32 all read MIXED. EXPLAINS+unfaithful reads NOT_FAITHFUL. The boundary values 1e-5 and 1e-4 are accepted, as §4 says. The tests encode r2's case (`:119`). |
| **N3** preflight provenance in the prereg | MINOR | **Resolved in the core.** LOW residual. | (a) v2 now exists, from the current bytes (see above). (c) The paragraph break is restored (`prereg:24-25`). (b) The link is now a correct relative path (`../evidence/sitl_2026-11-05/dev08/agq/s3a_p1/README.md`), but the folder does not exist yet. The full-grid values (1.720e-3 and 1.730e-3) are still not written in the prereg. **Pre-commit condition:** add that folder, with the v2 numbers and the v2 SHA-256s, in the same commit. Reword `:98` as "an earlier, unhashed version of the tool; round 1 reproduced its CPU actions bitwise with bytes `d41f…`". |
| **m3** important paths untested | MINOR | **Mostly resolved.** LOW residual. | Committed now: the `main(--execute)` accept path through every guard, `mkdir`, `attempt.json` and `result.json` (test `:197-212`). Also: refusal of a second attempt, VOID on a rise and on an unreadable post-run read, and every pre-`mkdir` refusal, with `measure` patched to raise (`:164-195`). **Still untested** (this review smoked the first two, and they work): `aborted.json` (probe D); the aerial_gym guard's negative case (guard probe); `set_tf32` raising on a mismatch; NaN in a recorded action; the `1e999` approval. |
| **m5** NAVRL_* and module origins | MINOR | **Mostly resolved.** LOW residual in N14. | The fail-closed guard is added (`:271-274`). It is non-vacuous: a foreign top-level `aerial_gym` raises `RuntimeError` (guard probe). That setup takes effect once is documented in prereg §3 (`:54-56`). I verified that the 7 keys that differ between the receipts (`BULK_EVAL_JSON`, `DENSITY_FINAL`, `DENSITY_START`, `EVAL_RUN_NONCE`, `NUM_BARS`, `OBS_DUMP`, `SENSOR_ABLATION_RECEIPT`) are not read by any of the 11 loaded `aerial_gym` modules, `navrl_reflection_offline_audit.py` or `sitl_bridge.py`; the only matches are comments. The NAVRL_* environment is recorded per player (probe A: density 205 records `NAVRL_NUM_BARS=70`, as §3 states). **Still open:** stray NAVRL_* are not stripped (only recorded); the test children inherit `os.environ`; the guard's timing and scope (N14). |
| l1 | LOW | Open | The rationale for 1e-5 is not written in §4 or at `:42`. |
| l2 | LOW | Open | `CPU_ONLY` (`:162-163`) is not named in §4. There is no residual-present precondition (`cpu_vs_recorded > 1e-4 + 1e-5`). This is harmless with the pinned inputs: by the triangle inequality, EXPLAINS then forces \|ON−OFF\| ≥ 1.6e-3, so it is not favourable by construction. |
| l3 | LOW | Open | `prereg:41` "none (float32)". The process flags are TF32 ON/`high` during the CPU run, which has no effect on CPU arithmetic. |
| l4 | LOW | Open | `backend_flags()` (`:116-122`) still lacks `sdp_*_enabled`, `cudnn.version()` and `torch.version.cuda`. |
| l5 | LOW | Open (`--cpu-only` only) | `--out x.npz` still runs the measurement, then fails with `FileExistsError` and leaves no JSON (probe I; `:342-344`). |
| l6 | LOW | Open | The device name is recorded, not enforced (`:265`). |
| l7 | LOW | Partially resolved | `aerial_gym`/`rl_games` files are hashed. `sitl_bridge`, `sitl_bridge_offline_audit`, `sitl_bridge_navrl_task_config`, the tool itself and `sitl_bridge_cplanner_replay` are not (probe A). The last of these is loaded from `MOTAR-cplanner`, which this worktree's HEAD does not bind. It is not on the policy path. |
| N4 | LOW | **Resolved** | All four child commands pass `-B` (test `:241`, `:244`, `:275`, `:301`). No new worktree file. |
| N5 | LOW | **Mostly resolved** | The refusal tests in `:164-195` patch `measure` to raise. `test_refusals` (`:262-268`) still runs `main(--execute)` with the real `measure`, the real `kernel_faults` and an unpatched CUDA environment. It is refused only by the approval's content. Safe unless `approval_problems` regresses. |
| N6 | LOW | Open | `inside_git_tree` uses `.absolute()` (`:302`). A symlink into a work tree is not detected (probe H: False; resolved: True). The real `RUN_DIR.parent` is no symlink and lies outside Git (probe H). |
| N7 | LOW | **Resolved** | Both refusals now give a reason (`:361`, `:363`). |
| N8 | LOW | Open | `git_state` ignores return codes (`:283`). `kernel_faults` accepts any non-empty stdout, including `-- No entries --` (`:310`). On this host both work: the user is in `adm`, the count is 0, and the preflight read it. |
| N9 | LOW | Partially resolved | (e) is fixed by the prereg text (`:54-56`). Still open: (a) `:80` says "rose", but the code voids on any change (`!=`) and on an unreadable count (`:393`); a fall is not mentioned. (b) Item 1 (`:61-62`) says "or a compared array is not finite", but a non-finite GPU array reads NONFINITE_OUTPUT. (c) Item 2's "Nothing is compared": the finite pairs are still computed. (d) `:50-52` says "for each density … SHA-256 of every loaded module", but the hashes are recorded once per result (`:270`). |
| N10 | LOW | Open | `RUN_DIR` depends on `$HOME` (`:46`). `measure(gpu=True)` has no guard of its own. Both need deliberate misuse. |

## New execute path (question 2)

| Check | Result |
| --- | --- |
| Refusals before the run directory exists | PASS for: CUDA `""` or unset (probe J); directory exists (probe B); inside Git (test `:179`); unreadable kernel log (test `:184`); NaN or `-Infinity` token (probe G); non-dict approval; non-string `commit` (probe G, reported as "missing commit"). **Gaps:** `1e999` (N11); `CUDA_VISIBLE_DEVICES=-1`, which hides every GPU but passes the check (probe J), so the attempt is spent and would end in `aborted.json`. |
| `attempt.json` | PASS. Keys: `approval` (with `approval_file.sha256`), `git`, `kernel_faults_start`, `prereg`, `scope`, `started`. The file is created with `"x"` after full serialisation. The approval SHA in `attempt.json` equals the one in `result.json` (probe A). |
| `aborted.json` on an exception | PASS for exceptions and KeyboardInterrupt (probe D): `{error, wall_s}`, then re-raised. A SIGTERM from `timeout`, `SIGKILL`, the watchdog's `os._exit(3)` or a segfault leaves only `attempt.json`. That is truthful, but §5 does not say how such a directory reads (N11). |
| `result.json` when the post-run fault read fails | PASS: VOID_HOST_FAULT, `kernel_faults_end: null` and `fault_check_error` (test `:222-234`; probe C). The post-run `git_state()` (`:388`) is not caught (N11). |
| VOID_HOST_FAULT | PASS on a rise, a fall and an unreadable read, with `reading_if_valid` kept (probe C). `density_readings` is not voided (N12). |
| Can the GPU run twice? | Not through the CLI. `mkdir(exist_ok=False)` on a fixed path is atomic, a second start is refused, and nothing is written (probe B). The routes that remain need deliberate misuse: deleting `gpu_run01`, a different `$HOME`, or a library call to `measure` (N10). |
| Partial or misleading record? | No partial record on any exception path. Exceptions to that: a kill between `np.savez` (`:276`) and `_write_json` (`:395`), which leaves an npz without a JSON, with `attempt.json` present; and the N11 gaps. The misleading part is N12. |

## Reading (question 3)

- **The code matches §4** in labels, order and thresholds:
  - `> 1e-5` unfaithful, `<= 1e-4` explained, `== 0.0` no effect;
  - NOT_FAITHFUL is global;
  - the other densities are classified per density, and MIXED is given on disagreement.
  - Exceptions: the wording items N9a–d and N13, and `CPU_ONLY` (l2).
- **Is any reading favourable by construction?** No. With CPU−recorded = 1.72e-3, EXPLAINS needs \|ON−OFF\| ≥ 1.6e-3, and NO_EFFECT and EXPLAINS cannot both hold. A device mix-up reads NOT_FAITHFUL (probe A: ON−recorded = 1.720e-3). NaN reads NONFINITE_OUTPUT (probe E, strict JSON). A fault reads VOID.
- **Is any valid case rejected?** None found. The boundary values are accepted (probe F). Every valid
  `CUDA_VISIBLE_DEVICES` index passes.
- **TF32_SWITCH_NO_EFFECT** depends on exact equality between two sequential GPU passes. No ON–ON repeat checks that
  GPU_ON is reproducible. If the forward pass is not bitwise reproducible, a no-effect switch would read
  RESIDUAL_NOT_TF32. Round 1 offered the repeat as optional. Noted under Limits; not scored.

## Tests and leaks (question 5)

- **Test file:** 13/13 OK in 27.6 s, maximum RSS 0.73 GB, rc 0 (`test_run_r3.log`). No new file in the worktree.
- **In-process leak probe** (`leak_probe_r3.py`): I ran the whole module through `unittest` and compared the
  process state before and after:
  - `os.environ`: 0 keys added, removed or changed;
  - `sys.path`: unchanged;
  - TF32, precision, `cudnn.benchmark`/`deterministic` and the thread count: unchanged;
  - no `aerial_gym`, `rl_games`, `sitl_bridge` or `warp` module imported into the parent;
  - only `MainThread` alive (no watchdog thread);
  - CUDA not initialised.

## New findings

| ID | Severity | Location | Scenario | Fix |
| --- | --- | --- | --- | --- |
| **N12** | MINOR | `tools/…:173-176`, `:268-269`, `:393-394`; prereg `:66-69`, `:80-81` | `density_readings` is computed whatever the headline reading is, and it is never voided. Probe F: CONTROL_FAILED gives `{"70": "TF32_EXPLAINS_POLICY_RESIDUAL", "205": "TF32_EXPLAINS_POLICY_RESIDUAL"}`. Global NOT_FAITHFUL gives `{"70": "TF32_EXPLAINS_POLICY_RESIDUAL", …}`, although §4 item 3 says "no cause is assigned". VOID_HOST_FAULT keeps the labels unmarked, while `reading` is renamed to `reading_if_valid`. A reader who quotes the per-density field gets a cause label from a run whose reading is not evidence. §4 defines the field only for the MIXED case. | Set each entry to null unless `reading` is one of items 4–6 or MIXED, and move it to `density_readings_if_valid` on VOID. Alternatively, state in §4 that `density_readings` is a diagnostic and never a reading under items 1–3 or VOID. Add a test case. |
| N11 | LOW | `:322-325`, `:358`, `:371-374`, `:379`, `:388` | Gaps that remain before and around `mkdir`: (a) `{"x": 1e999}` is RFC-valid JSON. It parses to `inf`, which `parse_constant` never sees, so it passes the guard. `_write_json(attempt.json, allow_nan=False)` then raises after `mkdir`, leaving an empty `gpu_run01` with neither `attempt.json` nor `aborted.json` (probe G). (b) `CUDA_VISIBLE_DEVICES=-1` passes. (c) The input and F pins are checked only inside `measure`, after `mkdir`; that is recorded in `aborted.json`, but the attempt is spent. (d) The post-run `git_state()` is not caught. (e) A signal kill leaves only `attempt.json`, and §5 gives no reading for that. Because `RUN_DIR` is fixed in the immutable prereg, each spent attempt needs a code change and an amendment. No case misreports a GPU result. | Before `mkdir`: run `json.dumps(approval, allow_nan=False)` (or pass `parse_float`, rejecting non-finite values), run `verify_inputs` and the F pin check, and require `CUDA_VISIBLE_DEVICES` to match `^[0-9]+$`. Wrap the post-run `git_state()` like `faults()`. Add to §5: "a run directory with `attempt.json` and neither `result.json` nor `aborted.json` reads ABORTED (killed)." |
| N13 | LOW | prereg `:66`, `:71-74`; `:288-289` | Editorial issues: item 3 says "This applies when either density is unfaithful: For either density, …", which duplicates its own condition. Items 4 and 5 still say "for both densities", although they are now per-density classes. The overall rule at `:68-69` makes the outcome the same. A non-string `commit` is reported as "missing commit". | Reword before commit. |
| N14 | LOW | `:271-274`; prereg `:53`, `:104` | The aerial_gym guard runs after every forward pass. The guard probe raised after both CPU passes; in `--execute` that means after all the GPU work, which is then discarded except for `aborted.json`. The guard checks only `aerial_gym.__path__[0]`, which `sitl_bridge.setup` itself sets to this tree; it does not check the recorded module files. The §5 GPU command sets no `PYTHONPATH`, which AGENTS.md literally requires of worktree runs. The bridge's stand-in packages make that harmless here: all 11 files are inside ROOT (probe A). | Move the check to just after the first `players(B, "cpu")`, before any GPU player is built, and also require every `loaded_module_files` path under `aerial_gym.*` to lie inside ROOT. Add `PYTHONPATH=<worktree>` to the §5 command. |

## Commands run

The envelope was `CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
timeout -k 10 900 nice -n 10 taskset -c 20,21,22,23 bash -c 'ulimit -v 12582912; …/envs/aerialgym/bin/python -B …'`.

1. `sha256sum -c review_r3/BYTES_UNDER_REVIEW.sha256` at the start and at the end. Read-only `git rev-parse HEAD` and
   `git status --short`.
2. `review_probe_r3.py` (`ff4db175…`) → `review_probe_r3.json` (`409a967d…`) and `review_probe_r3.log`. It adapts
   r2's probes to the new API: `main` now refuses a hidden CUDA, so `os.environ` carried `"0"` only during `main`'s
   pre-checks, and the measure stand-in reset it to `""` before torch's first CUDA query. The device was `"cpu"`, `git`
   was mocked clean at a fake HEAD, and `run_dir` was redirected to `review_r3/probe_runs/`. The approval files there
   are reviewer fixtures, not approvals. Probes:
   - A: the accept path;
   - B: a second attempt;
   - C: VOID on a rise, a fall and an unreadable read;
   - D: `aborted.json`;
   - E: NaN;
   - F: readings;
   - G: approval strictness;
   - H: Git detection;
   - I: `--out x.npz`;
   - J: CUDA visibility;
   - Z: the end state.
3. `python -B tests/test_sitl_s3a_policy_precision.py -v` from the worktree under `/usr/bin/time -v` →
   `test_run_r3.log` (`e032bf5c…`).
4. `leak_probe_r3.py` → `leak_probe_r3.json` (`ab8a0966…`).
5. `guard_probe_r3.py top_outside` → `guard_probe_r3.log`: a foreign top-level `aerial_gym` placed in `sys.modules`
   before `measure(gpu=False)`.
6. A rerun of the CPU preflight with the current bytes (cores 20–23): `--cpu-only --out review_r3/cpu_only_r3/result.json`.
   - Outputs: `result.json` `5363a974…` and `result.npz` `83c26c21…`.
   - It was compared field by field with `cpu_only_v2` and `cpu_only_v1`.
7. Reading:
   - the r1/r2 reports and r2's probe script;
   - `sitl_bridge.setup`/`Policy` and `install_cpu_only_aerial_gym_packages`;
   - the receipts' `env_snapshot_navrl` diff;
   - a grep of the 7 differing keys over the 11 loaded `aerial_gym` files, the audit and the bridge;
   - `focused_v*.log`.

## Limits

- No GPU was used. Three things are therefore not measured:
  - whether GPU_ON reproduces the recording to 1e-5;
  - whether the switches reach the kernels that matter, including the attention kernels;
  - whether two sequential GPU passes are bitwise reproducible, which the NO_EFFECT/NOT_TF32 split relies on.
- The execute paths were exercised with `measure` stood in (by the CPU device or a stub), with `git` mocked and with
  `faults` injected. The real `git status` check on a committed, clean tree and the real `RUN_DIR` were not
  exercised. `RUN_DIR` was checked only for location, absence and symlinks.
- The full PUBLIC_CPU suite was not run. The in-process leak probe covers this module alone.
- `MOTAR-cplanner/cplanner/motar/replay.py` and site-packages `rl_games` were not re-audited.
- Prereg §5 pins the GPU run to cores 6, 7, 12 and 13, which the brief reports as busy. The operator must free them,
  or the prereg must change before commit.
- I did not verify v1's byte provenance; it is not recoverable.
