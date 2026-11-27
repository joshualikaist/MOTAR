# S3a-P1 policy precision control: independent review r1

- **Verdict: FAIL.** One MAJOR finding is open: there is no single-attempt binding or in-tool host-fault record for the
  GPU run. The numerical design itself is sound. Taken alone, the design would be PASS_WITH_LIMITS.
- **Counts:** BLOCKER 0, MAJOR 1, MINOR 5, LOW 7.
- **Reviewer:** Claude Code subagent, model Claude Opus 5.5, with fresh context. This was a read-only audit. Writes
  went only to this directory. No GPU or simulator was used: every check ran with `CUDA_VISIBLE_DEVICES=""` on cores
  20–23, with `nice 10`, a 12 GiB `ulimit -v` and `timeout -k`.
- **Date:** 2026-11-06, 02:41–03:00 KST.
- **Object:** worktree `MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb766`. The three files under review are
  untracked.

## Hash checks

`BYTES_UNDER_REVIEW.sha256` was checked with `sha256sum -c` at the start (02:41) and again at the end. All three
files were OK both times:

| File | SHA-256 |
| --- | --- |
| `tools/sitl_s3a_policy_precision.py` | `d41f7712…7f032a` |
| `tests/test_sitl_s3a_policy_precision.py` | `ba7df9bb…096887` |
| `docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md` | `a2f55dc1…0c581d` |

Other inputs:
- The prior audit's `summary_v2.json` is `7bd1633a…` and matches the hash cited in `S3A_INPUT_AUDIT.md`.
- The author's preflight `result.json` is `ac940770…`.

## What was verified and how

1. **Read in full:**
   - the tool, the test and the prereg;
   - `sitl_bridge.setup` and `Policy`;
   - `navrl_reflection_offline_audit.install_cpu_only_aerial_gym_packages`, `read_checkpoint_schema`,
     `apply_schema_environment`, `build_player` and `deterministic_actions`;
   - `NavRLPpoPlayerContinuous.__init__` and `get_action`;
   - `staterec_hook.py`, the recorder of the actions;
   - `navrl_task._collect_obs_dump_frame`, the recorder of the observations;
   - the installed rl_games 1.6.5 `torch_runner` (`Runner.__init__` and `run_play`);
   - the prior probe `probe_replay_v2.py`;
   - the parent plan section and `S3A_INPUT_AUDIT.md`.
2. **Test file:** I ran `python -B tests/test_sitl_s3a_policy_precision.py -v` in the given envelope. 8 tests passed
   in 9.1 s with rc 0 (`test_run_r1.log`).
3. **Inputs** (numpy only, `-I`):
   - Both dumps are 10752 × 898 float32 and in call-major, environment order.
   - `stride_eff` = 1, `decimations` = 0, `dropped_reset_orphan_frames` = 0 and `run_num_envs` = 128.
   - For all 10752 rows of both densities, the dump's `ep_step` equals the state record's `sim_steps[call, env]`.
     This independently confirms the (call, env) pairing, which is stronger than the shifted-call control.
   - The recorded actions lie within ±0.9978, so the live path's clamp is a no-op. The live path's
     `rescale_actions(−1, 1)` is the exact identity (d = 1, m = 0).
   - Hence `deterministic_actions` is arithmetically the recorded `get_action` output.
4. **Recording path:**
   - The recorded "action" is the return value of `get_action` (`staterec_hook.py:112-115`).
   - The dumped observation is the exact `task_obs["observations"]` tensor, copied to float32 on the CPU.
   - rl_games `run_play` does not call `torch.compile`; only `run_train` does.
   - `Runner.__init__` sets matmul and cuDNN TF32 True, `benchmark` True and precision `high`.
   - No MOTAR task or hook code writes these flags (grep).
   - The recording's actual flags are NOT_RECORDED in G8. GPU_ON's faithfulness is therefore the only empirical link
     between the replay and the recording, which is what the reading requires.
   - The recording commit `c4bc416` and HEAD have identical bytes for the player, network, action-model, audit
     helper and YAML files. Only `sitl_bridge.py` changed, by 4 added lines.
   - The cell log shows torch 2.4.1+cu121 and an RTX 3070 (sm_86), the same as the planned run.
   - The network uses `Conv2d` (cuDNN), `nn.TransformerEncoder` with `batch_first=False` (no fast path),
     `Linear` and `LayerNorm`. Both TF32 switches therefore reach real kernels.
5. **Receipts:** the 70- and 205-bar `env_snapshot_navrl` differ only in these keys:
   - `NAVRL_NUM_BARS`, `NAVRL_DENSITY_START` and `NAVRL_DENSITY_FINAL`;
   - `NAVRL_OBS_DUMP`, `NAVRL_BULK_EVAL_JSON` and `NAVRL_SENSOR_ABLATION_RECEIPT`;
   - `NAVRL_EVAL_RUN_NONCE`.

   No policy-path code reads any of them.
6. **Reviewer CPU probe** (`review_probe_r1.py`, outputs `review_probe_r1.json` sha `10b05aa5…` and its `.log`):

   | Check | 70 bars | 205 bars |
   | --- | ---: | ---: |
   | CPU actions vs the author's preflight npz | 0.0 (bitwise) | 0.0 |
   | GPU-construction branch of `players()` run with `torch.device("cpu")`, vs the CPU player (as constructed / ON / OFF) | 0.0 / 0.0 / 0.0 | 0.0 / 0.0 / 0.0 |
   | CPU player with TF32 flags OFF vs ON | 0.0 | 0.0 |
   | CPU singleton vs batch 128 (envs 0–7, all 84 calls) | 7.75e-7 | 1.34e-6 |
   | Prior-audit subset (envs 0–7, calls before each env's first reset; 287 / 372 rows), singleton vs recorded | **1.5853345394134521e-3** | **1.012176275253296e-3** |
   | The same subset, batch 128 vs recorded | 1.58545e-3 | 1.01231e-3 |
   | Full grid, batch 128 vs recorded | 1.72025e-3 | 1.72970e-3 |
   | Full grid, first episodes only / after the first reset | 1.5855e-3 / 1.7202e-3 | 1.7297e-3 / 1.6773e-3 |
   | Alternative negative control (environment shifted by one): max / mean | 1.95 / 0.117 | 1.97 / 0.140 |

   Two conclusions follow:
   - The singleton subset reproduces the prior audit's numbers to every printed digit.
   - The preflight maxima (1.720e-3 and 1.730e-3) are larger only because the scope is larger: all 128 envs and all
     84 calls, including episodes after a reset. Batch 128 against batch 1 moves them by less than 2e-6.
7. **`--execute` path smoke on the CPU** (`review_probe_execute_smoke_r1.py`, `smoke_execute_summary.json` sha
   `32955d7a…`):
   - Setup: `main(["--execute", ...])` with a scope-correct approval, `git_state` mocked clean, `cuda:0` replaced by
     the CPU device and `get_device_name` stubbed.
   - Result: rc 0, the JSON was written, the flags were read back as specified, and the reading was
     **GPU_REPLAY_NOT_FAITHFUL**.
   - So a process whose "GPU" computes like the CPU cannot reach TF32_EXPLAINS. The faithfulness gate catches a
     device mix-up. This is a known-bad control at claim scale.
8. **RSS watchdog** (`watchdog_probe_r1.log`): with cap = 1 byte it aborts with exit 3; with cap = 8 GiB it survives.
   The host has 31 GiB of RAM, 21 GiB available.
9. **Module resolution** (`review_probe_modules_r1.json`):
   - Every loaded `aerial_gym.*` module file is inside the worktree. The bridge's stubs pre-empt the main checkout's
     editable finder.
   - The NAVRL environment in effect is the checkpoint schema: `squashed_gaussian`, mu_scale `1.0,0.4,1.0,1.0` and
     std `0.35,0.35,0.05,0.08`.
10. **`--cpu-only`:**
    - The flag requires `CUDA_VISIBLE_DEVICES == ""`, which is checked before torch is imported.
    - After a full measurement, `torch.cuda.is_initialized()` is False.
    - Warp's `wp.init()` calls the driver, finds no device and continues.
    - No CUDA work is possible.
11. **NaN behaviour** (in the probe):
    - `compare()` returns `nan`, and `reading()` maps an all-NaN GPU comparison to `RESIDUAL_NOT_TF32` (see m1).
    - `json.dump(..., allow_nan=False)` into an `open(..., "x")` file leaves a truncated JSON file
      (`nan_dump_probe_r1.json`).

## Judgement on the questions asked

### 1. One factor

The GPU_ON and GPU_OFF runs share one player, the same arrays, the same `run()` loop and the same device. They differ
only in what `set_tf32` sets:
- `cuda.matmul.allow_tf32`;
- `cudnn.allow_tf32`;
- the float32 matmul precision.

Each item the brief listed:

- **`cudnn.benchmark`:** True in both conditions and in the recording. Under benchmark, cuDNN autotunes separately
  for TF32 on and off, because `allow_tf32` is part of the conv cache key (my reading of PyTorch's conv params). A
  change of algorithm is therefore part of the TF32 factor, not a confound. Benchmark selection is timing-dependent
  per process, though. This is the source of the risk behind M1 and l1.
- **Batch shape and order:** one call's 128 environments, contiguous, in environment order. This matches the
  recording (dump order verified).
- **Preprocessing and normalisation:** the same code on the device. `_preproc_obs` and the model's `norm_obs` run
  inside `model.forward`, in float32 elementwise ops that the TF32 switches do not touch.
- **Player construction order** per density: CPU player, then GPU player. Building the CPU player again for 205
  re-sets the flags to ON/high, which is harmless (verified bitwise on the CPU). The cuDNN benchmark cache persists
  across densities inside the process, and that is the same for ON and OFF.
- **Second `B.setup`:** a no-op, because `_LOADED` is already set. 205 therefore runs under the 70 receipt's NAVRL_*.
  The keys that differ are not policy inputs, so this is no confound, but it should be stated or recorded (m5).
- **`torch.load` mapping:** `map_location=cuda:0` for the schema load and, through the temporary patch, for the
  rl_games restore. The result is the same as the recording's unmapped load of a checkpoint saved on cuda:0. The patch
  is restored in `finally`.
- **`apply_schema_environment`:** applied again with the identical schema, so the CPU and GPU players see the same
  NAVRL_* environment.
- **The recording versus GPU_ON:** they are different processes. Algorithm choice (cuDNN benchmark, possibly limited
  by workspace in the Isaac Gym process) and SDPA backend choice can differ. This is properly handled by the
  faithfulness gate and §6, at the cost of a possibly uninformative outcome.

### 2. Pairing, grid, comparison and thresholds

- **Grid construction:** correct. It refuses duplicate, out-of-range, incomplete and non-finite input.
- **Positive control** (true pairing < 1e-2): measured 1.7e-3. **Negative control** (shift by one call ≥ 1e-2):
  measured 1.93. Both are meaningful. The max statistic is weak for a negative control but adequate as a sanity
  check. The `ep_step` == `sim_steps` identity confirms the pairing.
- **Faithfulness at 1e-5:** sensible.
  - Identical kernels on identical float32 inputs reproduce bitwise.
  - A different TF32 algorithm choice would plausibly produce differences well above 1e-5.
  - The threshold is conservative, not favourable: a replay that lands between 1e-5 and 1e-4 is read as
    uninformative, never as supportive (l1).
- **Favourable by construction?** No reading is favourable by construction:
  - TF32_EXPLAINS needs GPU_ON within 1e-5 of the recording and GPU_OFF within 1e-4 of the CPU. Given the measured
    CPU−recorded residual of 1.7e-3, the triangle inequality then forces ON ≠ OFF.
  - A device mix-up reads NOT_FAITHFUL (smoke 7).
  - The only exposure is NaN (m1) and an ineffective switch reading as RESIDUAL_NOT_TF32 (m2).

### 3. The fixed reading

`reading()` matches prereg §4 in order, thresholds and the both/either quantifiers. The boundary cases are tested
(1e-5 counts as faithful; 1e-4 counts as explained). `CPU_ONLY` is an extra label for the preflight that §4 does not
name (LOW, inside l2).

The interpretation is justified, with three wording limits:
- RESIDUAL_NOT_TF32 means "not explained by the TF32 switches" (l4).
- TF32_EXPLAINS covers this GPU, this torch build, these stored observations and S3a's tolerance.
- Neither reading says anything about sensing.

### 4. Execution safety

- **Without approval, or on a dirty tree:** refused. This holds for the scope string, the commit equal to HEAD and
  `git status --porcelain`, which is empty only if the other unit's untracked files are also committed.
- **Twice: yes.** See M1.
- **The RSS watchdog without RLIMIT_AS** is adequate for this job:
  - the data is about 39 MB per density and the model is small;
  - the sample period is 0.5 s, plus `timeout -k 30 900`;
  - torch ops release the GIL, so a GIL-holding spike between samples is the only gap, and it is bounded by the
    host's 21 GiB available memory.
- **`--cpu-only` avoids CUDA:** yes (item 10).
- **Test leakage:** none found.
  - The flags test restores the shared precision/TF32 state correctly, because the two variables are one state in
    torch.
  - `mock.patch.dict` restores `CUDA_VISIBLE_DEVICES`, and no CUDA query happens inside the patched window.
  - Every in-process `main()` call returns before `_rss_watchdog()`, so the PUBLIC_CPU process never gets a watchdog
    thread.
  - The real-input run is a child process.
  - That child does inherit any NAVRL_* that earlier PUBLIC_CPU tests leaked (m5).

### 5. Tests

- **Paired positive and negative controls with non-zero counts** exist for:
  - `batches`;
  - `verify_inputs`;
  - `compare` and `controls` (n = 40; preflight n = 43008);
  - all five reading outcomes;
  - the CLI refusals;
  - the real-input preflight (residual > 1e-4 and < 1e-2, controls ok).
- **Missing:** see m3.

### 6. Prereg against code, and the preflight against the prior audit

The code implements §2–§5 as written, with these exceptions:
- the stated scope of the §1 numbers (m4);
- §3's "CPU: none" (l3);
- §4's "each condition pair", where ON vs OFF is missing (m2);
- §5's fault counts, which are an operator step that the tool does not record (M1).

The preflight is consistent with the prior audit (item 6). The difference between 1.585e-3/1.012e-3 and
1.720e-3/1.730e-3 is scope: 8 environments up to their first reset at batch 1 or 16, against 128 environments and all
84 calls at batch 128. It is not a change of method.

## Findings

### M1 (MAJOR): the GPU run is not bound to one attempt, and its host-fault and output-location rules are not enforced

- **Location:** `tools/sitl_s3a_policy_precision.py:235-271` (`main`), `:243` (`--out`), and prereg §5.
- **Scenario:**
  1. The first approved run reads GPU_REPLAY_NOT_FAITHFUL, for example because the timing-based cuDNN benchmark
     chose a different conv algorithm from the recording process.
  2. A second invocation with the same approval file and a new `--out` runs again. Nothing refuses it, and nothing
     records that a first attempt existed.
  3. Because benchmark selection is per-process and timing-dependent, the reading can plausibly change between
     attempts. This is a selection path that the prereg ("GPU run, once") and AGENTS.md (single-use approvals) forbid,
     but only procedurally.
- **Related gaps:**
  - The prereg says the kernel fault count "is read before and after; an increase voids the run", but the tool
    neither reads nor records it. The void rule depends on the operator remembering.
  - `--out` may be relative to the cwd or inside a Git work tree, for example `tools/x.json`. A write there breaks the
    source freeze during the run.
  - The sibling launcher in this session, `tools/sitl_agq_geometry_launch.py`, enforces all three: a fixed attempt
    directory created with `exist_ok=False`, `faults()` before and after, and a refused existing attempt.
- **Fix:**
  - For `--execute`, write to a fixed attempt directory outside every Git work tree, for example
    `~/motar_tmp/s3a_policy_precision_20261106/gpu_attempt01/`. It can be named in the prereg or derived from the
    approval's SHA-256. Create it with `exist_ok=False` before any CUDA initialisation.
  - Write an `attempt.json` (PREPARED/STARTED) that records the approval SHA and the kernel fault count before the
    run. Record the count after the run as well, and mark the result VOID when it rose.
  - Refuse a relative `--out`, or one inside a Git work tree.
  - Add a refusal test for each rule, plus an accept-path positive control (see m3).

### m1 (MINOR): a non-finite output is mis-read, then leaves a truncated result file

- **Location:** `:87-92` (`compare`), `:95-103` (`run`), `:136-146` (`reading`), `:267-269`.
- **Scenario:**
  - A NaN in any GPU action makes `compare()["max"]` NaN. `nan > 1e-5` is False, so the replay counts as faithful,
    and `nan <= 1e-4` is False, so `reading()` returns RESIDUAL_NOT_TF32 (reviewer probe).
  - `json.dump(allow_nan=False)` then raises part-way through writing and leaves a truncated JSON at `--out`
    (verified). `--out must be new` blocks that path from then on.
  - The effect is fail-closed by accident, but it spends the single approval and leaves a malformed record.
- **Fix:**
  - Assert `np.isfinite` on every action array in `run()`, and abort with an explicit status.
  - Make `compare()` raise on non-finite differences.
  - Write the JSON to a `*.pending` file and `os.replace` it into place.
  - Add a NaN test.

### m2 (MINOR): the direct ON-against-OFF contrast is not reported, so an ineffective switch is indistinguishable

- **Location:** `:217-219`, and prereg §4 ("each condition pair").
- **Scenario:**
  - The reported pairs are cpu/rec, on/rec, off/rec, off/cpu and on/cpu. The pair on/off, the one factor's direct
    effect, is missing.
  - If ON == OFF bitwise, the switch had no effect, and the run reads RESIDUAL_NOT_TF32. That cannot be told apart
    from "the switch acted, and a residual remains".
  - The npz allows this to be computed after the fact, but the record should carry it.
- **Fix:**
  - Add `gpu_on_vs_gpu_off`, together with the count of bitwise-equal elements for on/rec and on/off.
  - Optionally run ON a second time after OFF (ON–OFF–ON) to record in-process repeatability.
  - Decide before commit whether these are descriptive only, or whether ON == OFF exactly forces CONTROL_FAILED.
    Either way, preregister it.

### m3 (MINOR): important paths are untested

- **Location:** `tests/test_sitl_s3a_policy_precision.py:139-155` and the absent tests listed below.
- **Gaps:**
  - The approval guard has refusal tests only. No positive control shows that a valid approval on a clean HEAD is
    accepted, as AGENTS.md "Checks must be non-vacuous" requires.
  - Neither `players(B, <non-cpu>)` nor `measure(gpu=True)` runs in any test. The user's 2026-11-04 lesson is to
    smoke every changed runner path before commit.
  - Nothing tests the watchdog, the refusal of non-finite recorded actions, or `set_tf32` raising on a mismatch.
- **Status:** the reviewer smoked the GPU branch, the full `--execute` path (CPU-substituted) and the watchdog, and
  all of them worked. The gaps are in the committed tests, not, so far, defects.
- **Fix:**
  - Add a child-process test that mocks `git_state` clean, substitutes the CPU device for `cuda:0`, stubs
    `get_device_name`, and asserts rc 0, the recorded approval and the reading GPU_REPLAY_NOT_FAITHFUL. The
    reviewer's `review_probe_execute_smoke_r1.py` is a template.
  - Add a watchdog child test with cap = 1, expecting exit 3, and a large cap, expecting it to survive.
  - Add a case with NaN in a recorded action.

### m4 (MINOR): prereg §1 quotes the prior-audit numbers without their scope

- **Location:** prereg lines 16-18.
- **Problem:**
  - The figures 1.585e-3 and 1.012e-3 come from envs 0–7, calls before each env's first reset, at batch 1/16.
  - On the full 84 × 128 grid at batch 128 (this control's scope), the maxima are 1.720e-3 and 1.730e-3.
  - "CPU batching explains less than 1e-6" is also exceeded by the reviewer's batch-1 against batch-128
    measurement: 1.34e-6 at 205 bars, still far below 1e-4.
- **Why it matters:** the prereg is about to become immutable, and "numbers travel with their source".
- **Fix:** state the scope of each number, add the full-grid preflight values with their source file, and write
  "below 2e-6 (batch 1 against 16 against 128)".

### m5 (MINOR): the effective NAVRL_* environment and module origins are neither controlled nor recorded

- **Location:** `:199-204` (`B.setup` per density), `tests/...:160-161` (child `env=dict(os.environ, ...)`), and the
  GPU command in prereg §5.
- **Problems:**
  - The second `setup` is a silent no-op (harmless, as shown in item 5).
  - Stray NAVRL_* keys from the operator shell, or leaked by earlier PUBLIC_CPU tests into the test child, are
    inherited. Some are read at player construction: `NAVRL_EVAL_REFLECTION_MODE`, `NAVRL_MODE_PROBE_JSON` and
    `NAVRL_TRUNCATED_DMIN`. The current deterministic path does not consume them.
  - AGENTS.md says a worktree run must fail closed unless `aerial_gym` resolves inside the worktree. The tool relies
    on the bridge's stubs, which work (item 9), but it neither asserts this nor records it.
- **Fix:**
  - Call `setup` once, with a comment that only non-policy keys differ.
  - Record the effective NAVRL_* environment and the `__file__` of every loaded `aerial_gym.*` module, and assert
    that each lies under ROOT.
  - Refuse or strip NAVRL_* keys that are not in the receipt or the schema.
  - Build the test child's environment without NAVRL_* keys.

### LOW

- **l1, the faithfulness threshold (`:39`):** the derivation of 1e-5 is not written down. Add the rationale to §4 or
  §6: identical kernels reproduce bitwise, and a different TF32 algorithm choice would be well above 1e-5. Also
  report the fraction of bitwise-equal elements. No change of value is needed.
- **l2, the reading (`:136-146`):** `reading()` has no "residual present" precondition (cpu_vs_recorded.max > 1e-4),
  and `CPU_ONLY` is not named in prereg §4. Neither matters with these data (1.7e-3 is present), but both are cheap
  to make explicit.
- **l3, prereg §3 "CPU: none (float32)":** during the CPU run the process flags are TF32 ON / `high`, because
  `build_player` sets them globally. The reviewer verified that CPU float32 actions are bitwise identical with the
  flags ON or OFF. This is a wording fix only.
- **l4, coverage of the switches:**
  - To my knowledge, though not verified on this host, PyTorch's fp32 memory-efficient SDPA kernel on sm80+ ignores
    `allow_tf32` and uses its own fp32-accurate tensor-core path. Such a kernel is near fp32 and cannot carry a 1e-3
    residual.
  - Record `torch.backends.cuda.{flash,mem_efficient,math}_sdp_enabled()`, `torch.backends.cudnn.version()` and
    `torch.version.cuda`.
  - Word RESIDUAL_NOT_TF32 as "not explained by the TF32 switches".
- **l5, `--out` edge cases (`:243`, `:262`):** `--out x.npz` makes the JSON and npz paths identical, so the JSON
  `open('x')` fails after the run. A missing parent directory fails only after the measurement. Require a `.json`
  suffix and an existing parent before starting.
- **l6, host checks:** the tool does not check that no other CUDA process is running (`nvidia-smi
  --query-compute-apps`), or that the device name is an RTX 3070. These are operator rules only. Consider refusing on
  either.
- **l7, preflight provenance:** the CPU preflight `result.json` does not bind the tool's bytes (the tool was untracked;
  `git.dirty` lists it).
  - The reviewer's rerun with the hashed bytes reproduces the preflight's CPU actions bitwise (0.0 for both
    densities), and the tool's mtime (02:28:37) precedes the result's (02:29:01).
  - Fix: record the SHA-256 of the tool, `sitl_bridge.py`, the audit helper and the model sources in each result.

## Limits

- No GPU was used. Every statement about GPU kernels (the conv cache key including `allow_tf32`, SDPA behaviour,
  cuBLAS determinism) is from my reading of PyTorch and cuDNN behaviour, not from a measurement on this host.
- The recording's runtime TF32 flags are NOT_RECORDED. That the recording ran with TF32 ON is an inference from the
  code (rl_games Runner, no MOTAR override). GPU_ON's faithfulness is the planned empirical test of it.
- The full PUBLIC_CPU suite was not run. The isolated test file was run, and in-process leakage was assessed by
  reading the code.
- `MOTAR-cplanner/cplanner/motar/replay.py` (imported by `B.setup`) and the site-packages rl_games lie outside the
  tree's clean check. They were read only for side effects on the flags (none found).
- Reviewer artefacts in this directory:
  - `test_run_r1.log`;
  - `review_probe_r1.py`, its `.json` and `.log`;
  - `review_probe_modules_r1.py` and its `.json`;
  - `review_probe_execute_smoke_r1.py`, its `.log`, `smoke_execute_result.json`, `smoke_execute_result.npz`,
    `smoke_execute_summary.json` and `smoke_fixture_NOT_AN_APPROVAL.json` (a reviewer smoke fixture with a fake commit, renamed after the run; not an approval);
  - `watchdog_probe_r1.log`;
  - `nan_dump_probe_r1.json`.
