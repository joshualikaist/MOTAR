# S3a-P1 — policy precision control on stored observations, preregistration, 2026-11-06

**Status: PROPOSED; the GPU conditions are NOT_RUN. No GPU execution is authorized by this file.** A run needs the
user's scoped approval that names this scope and the exact clean commit. The CPU condition has no simulator and
no GPU. Section 5 lists its preflight runs before commit.

- Parent: [`sitl_agq_followup_v1_2026-11-06.md`](sitl_agq_followup_v1_2026-11-06.md), section *S3a numerical
  control*, comparison 1, and [`S3A_INPUT_AUDIT.md`](../evidence/sitl_2026-11-05/dev08/agq/S3A_INPUT_AUDIT.md).
- Code: `tools/sitl_s3a_policy_precision.py`.
- This control does not change S3a's historical verdict (NOT_EQUIVALENT), its tolerances or Frozen F, and no
  formal gate follows from it. A TF32-off reference would be a new numerical lineage, never a correction of the
  stored G8 bytes.

## 1. Question

On the identical stored observations, Frozen F's CPU actions differ from the actions the G8 r2 GPU process recorded
by up to 1.585e-3 (70 bars) and 1.012e-3 (205 bars). S3a's action tolerance is 1e-4. CPU batching explains only
about 1e-6.
- These figures come from the prior audit (`S3A_INPUT_AUDIT.md`, `summary_v2.json`). Its scope was envs 0–7, up to
  each env's first reset.
- On the full 84 × 128 grid the residual is larger. The preflight's values are recorded in
  [`dev08/agq/s3a_p1/`](../evidence/sitl_2026-11-05/dev08/agq/s3a_p1/README.md).

`build_player`, like rl_games' Runner, enables TF32 matmul and cuDNN TF32 at construction and sets float32 matmul
precision `high`. Is GPU TF32 arithmetic the policy part of the residual?

## 2. Inputs (pinned by SHA-256 in the code)

- G8 r2 `obs_dump_{70,205}bars_seed4501.npz`: 10752 rows each, which is 84 calls × 128 environments × 898.
- `state_record_{70,205}bars_seed4501.npz`: the recorded actions, [84, 128, 4].
- The two full-cell receipts, for the bridge setup snapshot.
- Policy F `f7022139…`.

Each (call, environment) must appear exactly once. Every value must be finite. The replay batch is one call's 128
environments in environment order, the recording's shape.

## 3. Conditions (one process, one factor)

| Condition | Device | Precision switches |
| --- | --- | --- |
| CPU | cpu | none (float32) |
| GPU_ON | cuda:0 | as constructed: `cuda.matmul.allow_tf32 = cudnn.allow_tf32 = True`, precision `high` |
| GPU_OFF | cuda:0 | after construction: both False, precision `highest` |

- The switches are set after player construction and read back. A mismatch stops the run.
- GPU_ON and GPU_OFF use one player, the same arrays, batch shape and order. `cudnn.benchmark` stays as
  constructed, so its algorithm choice under each setting is part of the factor.
- Also recorded: the flags right after construction, `cudnn.benchmark`, `cudnn.deterministic` and the device name.
- The GPU player is built by the same `build_player` call as the CPU player, with the checkpoint mapped to `cuda:0`.
- For each density the result records:
  - the NAVRL_* environment in effect for each player;
  - the SHA-256 of every loaded `aerial_gym`/`rl_games` module file.
- The run stops if `aerial_gym` resolves outside this worktree.
- The bridge setup takes effect once per process. Both densities' players therefore run under the first (70-bar)
  receipt's NAVRL_* snapshot, as in the dev08 unit's single bridge setup. The recorded environment shows this. The
  keys that differ between the two receipts are not policy inputs.

## 4. Fixed reading

Over both densities (`reading()` in the code):
1. **CONTROL_FAILED.** Either density fails its pairing controls on the CPU actions, or a compared array is not
   finite:
   - the true pairing must differ from the recorded actions by less than 1e-2 (positive control);
   - the pairing shifted by one call must differ by at least 1e-2 (negative control).
2. **NONFINITE_OUTPUT.** A GPU condition produced a non-finite action. Nothing is compared.
3. **GPU_REPLAY_NOT_FAITHFUL.** For either density, max |GPU_ON − recorded| > 1e-5. The replay does not reproduce
   the recording, so no cause is assigned. GPU_OFF versus CPU is still reported.

Otherwise each density is classified on its own, as one of 4–6 below. The reading is that label when both
densities agree, and **MIXED** when they differ. `density_readings` keeps both labels, and is reported only for
these four readings. It is null for CONTROL_FAILED, NONFINITE_OUTPUT, GPU_REPLAY_NOT_FAITHFUL, CPU_ONLY and
VOID_HOST_FAULT; for VOID_HOST_FAULT the labels move to `density_readings_if_valid`.

4. **TF32_EXPLAINS_POLICY_RESIDUAL.** GPU_ON is faithful and, for both densities, max |GPU_OFF − CPU| ≤ 1e-4
   (S3a's action tolerance).
5. **TF32_SWITCH_NO_EFFECT.** GPU_ON is faithful, GPU_OFF differs from CPU by more than 1e-4, and GPU_ON and
   GPU_OFF are bitwise identical for both densities.
   - The switches were read back, but no kernel that matters honoured them; some attention kernels may ignore them.
   - The residual is then not shown to be TF32, and not shown not to be.
6. **RESIDUAL_NOT_TF32.** GPU_ON is faithful, the switch changed the outputs, but GPU_OFF still differs from CPU by
   more than 1e-4. The next step would be layer-level outputs, not a threshold or weight change.

A run whose kernel fault count rose reads **VOID_HOST_FAULT**, with the otherwise computed reading kept as
`reading_if_valid`.

Reported for each density, for every condition pair including GPU_ON against GPU_OFF: the maximum, its (call,
environment, dimension), the mean and the count above 1e-4. Per-row actions go to a new `.npz` beside the result.

What does not follow from any reading:
- a sensing (raster/KF) result: comparison 2 of the parent is separate;
- an S3a equivalence;
- a new tolerance.

## 5. Execution and preflight

- CPU preflight:
  - command: `CUDA_VISIBLE_DEVICES="" ... python -B tools/sitl_s3a_policy_precision.py --cpu-only --out <new>`;
  - the file must be new and outside every Git work tree;
  - environment: cores 8–11, `nice -n 10`, a 12 GiB address-space cap and the in-process 8 GiB RSS watchdog;
  - outputs:
    - `~/motar_tmp/s3a_policy_precision_20261106/cpu_only_v1/` came from the review-r1 bytes.
    - `cpu_only_v2/` came from the review-r3 bytes, `cpu_only_v3/` from the review-r4 bytes, and `cpu_only_v4/`
      from the bytes committed with this file.
      Each run's tool hash is recorded beside it. The CPU actions are compared bitwise across the runs.
  - The numbers are reported with the commit that adds this file, in `dev08/agq/s3a_p1/`. They do not decide
    the reading alone.
- GPU run, once, after the approval:
  - command: `CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=<worktree> timeout -k 30 900 nice -n 10 taskset -c 6,7,12,13 python -B tools/sitl_s3a_policy_precision.py --execute --approval-file <file>`;
  - approval keys: `approved_by`, `approved_at`, `scope` and `commit`, all strings. The approval must be strict
    JSON. The tree must be clean at that commit;
  - before anything is created, the run refuses in any of these cases:
    - `CUDA_VISIBLE_DEVICES` is not one GPU index;
    - an input or Policy F differs from its pin;
    - the run directory exists or lies inside a Git tree;
    - the kernel log cannot be read;
    - the approval holds a non-finite number;
  - `aerial_gym` must resolve inside the worktree. This is checked right after the first CPU player is built, before
    any GPU work;
  - a run killed by a signal leaves `attempt.json` without `result.json`. That reads ABORTED, and the GPU conditions
    count as NOT_RUN;
  - the run writes only to the fixed directory `~/motar_tmp/s3a_policy_precision_20261106/gpu_run01/`. It is
    created before CUDA starts and must not exist, so a second run with the same approval refuses;
  - `attempt.json` is written right after the directory is created: the approval with its SHA-256, the Git state
    and the starting fault count;
  - an exception during the measurement writes `aborted.json`;
  - an unreadable fault count after the run still writes `result.json`, with VOID_HOST_FAULT.
- Conditions on the host:
  - no other CUDA job may run, and the tracked tree is frozen while it runs;
  - the tool itself reads the kernel fault count before and after; an increase gives VOID_HOST_FAULT;
  - no RLIMIT_AS is set for the GPU run (CUDA reserves address space); the in-process RSS watchdog (8 GiB,
    sampled every 0.5 s) bounds it instead.
- Expected duration: under 2 minutes.

## 6. Limits

- One GPU (the RTX 3070 of the recording host), one torch build and one process. `cudnn.benchmark` stays True, as
  in the recording. Kernel selection is not otherwise controlled.
- The recorded actions are float32 outputs of the live player. Their exact kernel path in the Isaac Gym process is
  not observable. GPU_REPLAY_NOT_FAITHFUL is therefore a possible, uninformative outcome, and it is reported as such.
