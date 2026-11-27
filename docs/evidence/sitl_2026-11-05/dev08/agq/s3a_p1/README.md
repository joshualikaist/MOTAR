# S3a-P1 — policy precision control, CPU preparation and preflight, 2026-11-06

**Status (2026-11-06, after gpu_run01):** the single approved GPU run ran at `f286e29`. Its reading is
**TF32_EXPLAINS_POLICY_RESIDUAL** for both densities. Results review r1: PASS_WITH_LIMITS. See
[GPU run](#gpu-run-gpu_run01-2026-11-06) below. S3a's historical verdict stays NOT_EQUIVALENT; no tolerance, weight
or formal gate changes. The preparation record follows unchanged: CPU implementation, CPU preflight and independent
reviews, before the GPU conditions had run.

Read the [preregistration](../../../../../plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md) first.

## CPU preflight

The CPU condition and pairing controls ran on the stored G8 r2 grid: 84 calls × 128 environments × 4 action
dimensions per density, in batches of one call's 128 environments.
- Command: `tools/sitl_s3a_policy_precision.py --cpu-only`.
- Environment: CUDA hidden, cores 8–11, `nice -n 10`, one thread.
- Record: [`cpu_preflight_v4.json`](cpu_preflight_v4.json), a byte-identical copy of the external
  `cpu_only_v4/result.json`, which ran the committed tool bytes (hash in [`validation.json`](validation.json)).

| Density | CPU vs recorded GPU, max | at (call, env, dim) | components > 1e-4 | shifted-pairing control, max |
| --- | ---: | --- | ---: | ---: |
| 70 bars | 1.7202e-3 | (79, 119, 1) | 8453 of 43008 | 1.93 |
| 205 bars | 1.7297e-3 | (28, 34, 1) | 8468 of 43008 | 1.93 |

- The known residual is present on the full grid. The largest error is in action dimension 1, the lateral
  component with F's 0.4 mu scale.
- The positive control (true pairing < 1e-2) and the negative control (shifted pairing ≥ 1e-2) both pass.
- The kernel fault count was 0 before and after.
- The earlier preflights gave the same action npz bytes (`83c26c21…`):
  - `cpu_only_v1` (review-r1 bytes);
  - `cpu_only_v2` (review-r3 bytes);
  - `cpu_only_v3` (review-r4 bytes).
- The prior audit's smaller figures came from envs 0–7 before their first reset.
- These numbers do not decide the preregistered reading. That needs the GPU_ON and GPU_OFF conditions.

## Reviews (Claude Opus 5.5, fresh context each round)

- [r1](REVIEW_r1.md): FAIL, 1 MAJOR. The GPU run was not single use, the tool kept no fault-count record, and
  `--out` could point inside the worktree.
- [r2](REVIEW_r2.md): PASS_WITH_LIMITS. Its MINORs were repaired before commit:
  - the attempt record;
  - checks before the run directory is created;
  - the MIXED reading;
  - preflight provenance;
  - no `.pyc` written by test children.
- [r3](REVIEW_r3.md): PASS_WITH_LIMITS. Its MINOR N12 (per-density labels when no cause is assigned) and LOW N11/N14 were repaired before commit.
- [r4](REVIEW_r4.md): PASS_WITH_LIMITS. This round was shared with the geometry unit. LOW B4-l1 to B4-l3 were
  repaired before commit:
  - an ASCII GPU index;
  - Policy F's pin checked before the run directory;
  - refusal tests that assert their reason.
- [r5](REVIEW_r5.md): PASS_WITH_LIMITS, 0 MINOR. The LOWs N13 (wording), N14 and B5-l1 (an unreachable git failure
  after `mkdir`) remain open as stated.

## GPU run: gpu_run01 (2026-11-06)

- **Approval.** The approval file was written at 11:40 KST. It names the code's SCOPE and commit `f286e29`.
  - The user wrote it by running a snippet the session provided. One process wrote both this file and the
    geometry unit's, so they share one `recorded_at`.
  - The session's own two attempts to write them were refused by the auto-mode classifier.
- **Execution.** The prereg §5 GPU command ran once, 11:54:08–11:54:13 KST, on the RTX 3070 (torch
  2.4.1+cu121) with `CUDA_VISIBLE_DEVICES=0`.
  - The measurement took 4.68 s. Kernel faults were 0 → 0, the tree was clean at `f286e29`, and `aerial_gym`
    resolved inside this worktree.
  - The tool's record holds the affinity (cores 6/7/12/13), nice 10 and the environment.
  - **NOT_RECORDED** (results review r1, S-m1):
    - the outer `timeout -k 30 900` wrapper of the session's launch command;
    - the absence of other CUDA jobs. The session's `nvidia-smi` check at 11:40 KST listed only the desktop's
      remote-access program, but no file records it.
  - The geometry unit's SITL run started at 11:54:42, after this run had ended.
- **Switches read back as preregistered.** After construction and in GPU_ON: matmul and cuDNN TF32 True, precision
  `high`. In GPU_OFF: both False, precision `highest`. `cudnn.benchmark` stayed True.
- **Records.**
  - [`gpu_run01_record.json`](gpu_run01_record.json) holds the bindings (approval, `attempt.json`, the action
    `.npz`, stdout) and the per-density table, written by `~/motar_tmp/agq_geometry_diag_20261106/make_results.py`.
  - [`gpu_run01_result.json`](gpu_run01_result.json) is a byte-identical copy of the run's `result.json`.

| Density | GPU_ON vs recorded | GPU_OFF vs CPU | GPU_ON vs CPU | GPU_ON vs GPU_OFF | CPU vs recorded |
| --- | ---: | ---: | ---: | ---: | ---: |
| 70 bars | 0 (0 of 43008 > 1e-4) | 2.3544e-6 (0) | 1.7202e-3 (8453) | 1.7198e-3 (8451) | 1.7202e-3 (8453) |
| 205 bars | 0 (0 of 43008 > 1e-4) | 1.4901e-6 (0) | 1.7297e-3 (8468) | 1.7296e-3 (8467) | 1.7297e-3 (8468) |

Maximum absolute action difference over 84 calls × 128 environments × 4 dimensions; in brackets, the count of
components above 1e-4.

- **Reading (prereg §4):** TF32_EXPLAINS_POLICY_RESIDUAL for both densities, so the global reading is the same.
  - The pairing controls passed.
  - GPU_ON reproduced the recorded actions exactly, so the replay is faithful.
  - With TF32 off, the GPU differs from the CPU by at most 2.4e-6, below S3a's action tolerance of 1e-4.
- **What follows:** the policy part of the CPU-versus-recorded residual (1.72e-3 and 1.73e-3) is explained by GPU
  TF32 arithmetic in Frozen F's forward pass, on this GPU, torch build and batch shape.
  - The switch is one bundle: matmul TF32, cuDNN TF32, the float32 matmul precision setting and the cuDNN algorithm
    choice under each. They are not separated.
  - "Explained" means "within S3a's 1e-4 tolerance". A GPU-versus-CPU difference of up to 2.35e-6 remains with
    TF32 off.
- **What does not follow:**
  - a sensing (raster/KF) result, which is the separate comparison 2;
  - an S3a equivalence;
  - a new tolerance;
  - any change to the stored G8 bytes.

  A TF32-off reference would be a new numerical lineage.

## Results review r1 (Claude Opus 5.5, fresh context; shared with the geometry unit)

[RESULTS_REVIEW_r1.md](RESULTS_REVIEW_r1.md) is a copy of `~/motar_tmp/agq_geometry_diag_20261106/results_review_r1/REVIEW.md`.
The verdict for this unit is **PASS_WITH_LIMITS**, with no BLOCKER or MAJOR.
- **Checks.**
  - Scope, commit, clean tree, faults and single use: PASS.
  - Input pins, the `.npz` hash and the grid: PASS. The CPU actions are bitwise equal to the `cpu_only_v4`
    preflight.
  - Every maximum, argmax and count recomputes, and the reading follows prereg §4.
- **Findings, all repaired in this section before commit:**
  - MINOR S-m1: the unrecorded envelope claims, now marked NOT_RECORDED;
  - LOW S-l1: what the switch bundles and what "explained" means;
  - LOW S-l2: answered under *Approval*.
- The envelope check is LIMITED_VALIDITY for the two NOT_RECORDED items.
