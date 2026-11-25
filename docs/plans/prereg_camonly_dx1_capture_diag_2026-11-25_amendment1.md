# CAM-DX1 amendment 1 (2026-11-25 ~20:45 KST, after the first smoke; no eval cell has run)

**What happened.** Smoke `~/motar_tmp/camdx1_20261125/raw/` at `b759b47` read SMOKE_FAIL (host faults 1 → 1):
- R312_smoke (128 envs, quota 1): OK.
- V624_smoke at 128 envs: `torch.OutOfMemoryError: CUDA out of memory` (6.67 GiB in use by the process; 7.66 GiB total;
  other processes on the GPU: the desktop session and a remote-desktop server, about 0.7 GiB). Classified OOM, as
  preregistered; the 624 arms switched to 64 envs × quota 16.
- V624_smoke_e64 at 64 envs: `RuntimeError: cuDNN error: CUDNN_STATUS_INTERNAL_ERROR` in the target segmenter's
  convolution on the 624 × 468 frames (`navrl_perception.py:633`, called from `_detect_rgbd`), during the first
  observation. The runner classified it FAIL, because its log has no out-of-memory text, and stopped the smoke.

**Conflict in the preregistration.** Lines 40-41 say "If 64 envs also fail, the 624 arms read NOT_RUN_OOM and the 312
arms are still analysed"; the review r1 P2-3 rule (line 48) counts a failure as out-of-memory only with CUDA
out-of-memory text. The code followed line 48.

**Change.** At the 64-env fallback only (a 624 cell after a recorded out-of-memory at 128 envs), a cuDNN
`CUDNN_STATUS_INTERNAL_ERROR` or `CUDNN_STATUS_ALLOC_FAILED` also counts as the second out-of-memory, so the 624 arms
read NOT_RUN_OOM as lines 40-41 intend (`tools/camdx1_eval_run.py` `OOM_FALLBACK_RE`, `is_oom(..., fallback=True)`;
`tests/test_camdx1.py` with a positive and a negative control). That cuDNN's internal error here comes from memory
pressure is INFERRED (it occurs in the first large convolution after an out-of-memory at twice the batch), not proven.
The host-fault rule is unchanged: a rise of the kernel fault count still voids the stage.

**Consequences, fixed now.**
- The smoke is repeated from the start in a new output parent `~/motar_tmp/camdx1_20261125/raw_a1/` (R312_smoke,
  V624_smoke at 128 then 64 envs, and, if V624 reads NOT_RUN_OOM, V312_smoke carrying the visible-start and step-0
  detection controls, as preregistered). The failed `raw/` folder is kept as recorded.
- If the 624 arms read NOT_RUN_OOM: the eval runs the six 312 cells (R312, V312 × 3 seeds); DET and the interaction
  read NOT_RUN; START and START_6_12 use the 312 pair only (preregistered handling). The verdict
  COMPLETE_624_NOT_RUN_OOM is the expected complete reading.
- The question "does 624 × 468 detection raise capture" is not answered by this unit on this 8 GB GPU; it needs a
  separate design (fewer environments, or a detection-range probe without policy rollouts), not part of this unit.
- Thresholds, factors, readings and controls are otherwise unchanged.
