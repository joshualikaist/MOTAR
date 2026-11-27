# Q01-V2 GPU runbook — requires separate GPU execution approval

Nothing below has been executed. Run from `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1`
on branch `codex/mt-p1`, with the pinned aerialgym Python, only after the user approves the
exact commit. Do not edit, commit or create any tracked file while a stage runs.

## 1. Bind and preflight (CPU only)

Set `BOIA_SOURCE_SHA` to the commit the user approved, then confirm it is HEAD. Preflight runs
no torch, Isaac or GPU code and does not unpickle the checkpoint.

```bash
cd /home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1
BOIA_SOURCE_SHA=<approved 40-hex commit>
test "$(git rev-parse HEAD)" = "$BOIA_SOURCE_SHA" && echo HEAD_MATCHES_APPROVAL
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py preflight --approved-commit "$BOIA_SOURCE_SHA"
```

Continue only if it prints `PASS_CPU_PREFLIGHT`. Its checks include branch, exact HEAD, clean
tracked tree, no untracked files under `aerial_gym/ motar_eval/ tools/ tests/`, committed
inputs, checkpoint SHA-256, interpreter, ≥ 3.3 GB free, import origins inside this worktree,
unchanged v1 files, and that the v1 output root was never created.

## 2. Run (GPU; needs the explicit approval)

Either the whole campaign, which runs S0 → S1_NATIVE → S1_OFF → S1_ON, stops at the first
stage that does not complete, and then runs `verify` on the CPU:

```bash
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py campaign --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
```

or the same stages one at a time, stopping on any nonzero exit:

```bash
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s0        --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s1-native --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s1-off    --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s1-on     --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py verify    --approved-commit "$BOIA_SOURCE_SHA"
```

`verify` is CPU-only, writes `parity_result.json` once and refuses to overwrite it. After a
stopped stage it still classifies what exists. There is no retry: an existing campaign or
stage path is never reused or deleted. A rerun needs a new, separately approved RunID.

## 3. Limits

| Item | Cap |
|---|---|
| Profile | N1, seed 4901, moving `E2_direct_l2_v1`, 70 bars, visible at 10 m, OFF lineage |
| Actions | S0 20; S1_NATIVE, S1_OFF, S1_ON 200 each; each stops at its first ended episode |
| Wall per process | S0 180 s; S1 stages 300 s each |
| Campaign wall | 1080 s cumulative (18 min); no expansion |
| Shutdown | 4 s before a stage cap: SIGTERM, 3 s grace, then SIGKILL |
| Disk | preflight ≥ 3.3 GB free; stop below 3.0 GB |
| Output | ≤ 256 MB total; 64 MB logger cap per stage; 8 MB host trace per stage; ≤ 8 sensor snapshots |
| VRAM | planning ceiling min(6656 MiB, total − 768 MiB), sampled every 2 s (not a true peak) |
| Guards | resources every 2 s; source/config every 10 s; observer watchdog 2 s or 3 × > 500 ms |

## 4. Output

`/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/`

- `campaign_start.json`; per stage `<STAGE>.log`, `<STAGE>_supervisor.json` (embeds the worker's
  own first cause and return code) and `<STAGE>_RAW_MANIFEST.json` (SHA-256 of every file,
  partial outputs included); finally `parity_result.json`.
- Every stage directory has `host.jsonl` and `process_done.json` or `failure.json`.
  Observer stages (S0, S1_OFF, S1_ON) add `sensors/actions/evaluator/shadow/parity/events.jsonl`
  and bounded `sensor_NNN.npz`. `S1_NATIVE` has only `host.jsonl` and its receipt.
- Nothing is written to `results/`, checkpoints or tracked files.

## 5. Reading the result

`parity_result.json` gives one verdict:

- `PASS_EXACT_NATIVE_OFF_ON_PARITY_WITH_REPLAY_CONTROL` — all three exact comparisons and the
  runtime-profile check passed. This supports observer and diagnostic noninterference for this
  exact profile at host-trace level. It is not a mission, safety or latency result.
- `REPLAY_INSTABILITY_OR_PROFILE_MISMATCH` — at least one mismatch. The first row, differing
  fields and both stages' numerics attestation are recorded; the cause is `NOT_VERIFIED`.
  Do not read it as observer interference or as CUDA nondeterminism without new evidence.
- `INCOMPLETE_CAMPAIGN_NOT_EVALUATED` — a stage is missing and nothing mismatched.
- `STOPPED_INTEGRITY_NOT_EVALUATED` — a supervisor stopped a stage (e.g. `DEVICE_VRAM_CEILING`,
  `CHECKPOINT_CHANGED`), a supervisor receipt is missing or unreadable, or worker and supervisor
  receipts disagree. It overrides every other verdict; no claim is SUPPORTED.

Real GPU runtime parity, device build, VRAM and timing remain `GPU_PENDING` until a run.
G5 HOLD whatever the verdict.
