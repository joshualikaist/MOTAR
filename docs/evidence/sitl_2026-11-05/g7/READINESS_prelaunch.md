# G7 read-only readiness audit — 2026-11-05

Reviewer: Codex subagent `/root/g7_readiness` (exact model identifier not exposed). This is a source/readiness audit,
not the required independent results review, not a GPU run, and not an experimental PASS. No repository file was
changed and no tests, simulator or experiment was launched.

## Finding before launch

**MEDIUM — enforce the preregistered byte identity explicitly.** `prereg_actionhold_g7_v1.md` requires byte-identical
`episodes.jsonl` for the k1d0 control. `tools/gr_run.py:204–218`'s `identity_compare` compares parsed JSONL objects;
`tools/actionhold_run.py:105–110` currently accepts that boolean. Logically identical JSON with different formatting
could pass this check. Add a G7-only raw bytes/SHA identity check and include it in the stop condition before launch;
preserve historical `gr_run.py`. A same-byte positive fixture and a logically-equal/different-byte negative fixture
exercise the actual requirement. Independently recheck the identity hashes in the results review.

No other source blocker was identified in this limited read-only inspection.

### Repair authorized by parent after this audit

The parent then authorized edits only to `tools/actionhold_run.py` and new `tests/test_actionhold_identity.py`.
The G7 runner now adds `episodes_byte_equal` by comparing raw ledger bytes and requires it in the identity stop
condition together with parsed ledger/result equality. Missing ledger files return false. Historical gr_run.py
was not changed. Three focused tests passed: same bytes accept, logically identical/different bytes reject, and
missing ledgers reject. The combined `test_actionhold*.py` suite passed all11 tests (0.020s), using CUDA hidden,
cores6,7,12,13, nice10, 4GiB address-space cap, threads1 and timeout180. No GPU or experimental run occurred.
The files remain uncommitted for the parent's independent diff review and local commit before launch.

## What is ready

- Existing preregistration: `docs/plans/prereg_actionhold_g7_v1.md`; user now explicitly approves through G7.
- Intended output `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/actionhold_g7_v1` is absent.
- Seven cells: 205/4501 k1d0, k2d0, k4d0, k1d1; then 70/4501 k2d0, k4d0, k1d1. Native Policy F, quota16×128.
- `camonly_run.preflight` checks approved clean camonly/dev HEAD, committed prereg/entry/launcher/module files,
  pinned launcher and F checkpoint SHA, one RTX3070, no other GPU compute process, internal import origins and new
  output. G7 additionally requires its prereg and hook files committed.
- Each cell checks host fault count/boot/CPU topology, receipt provenance, quota ledger, recorder violations and
  clean source; hook sidecars require applied patch/config/calls and positive held/delayed counts in treatment arms.
- Identity hook returns the exact policy tensor before reading sim_steps or changing RNG. Treatment latching uses
  the task's per-env sim_steps and resets on zero. `_sim_steps` can traverse AERIALRLGPUEnv → ExtractObsWrapper → task.
- Hook sidecars are written only after a player actually calls the controller, so helper Python processes that only
  import sitecustomize do not reproduce G8's earlier existing-output failure.
- Governor and altitude PI remain per step. No new checkpoint or training is involved.

## Suggested commands

Run the eight existing hook tests before committing/launching; use the interpreter with Torch available:

```sh
env CUDA_VISIBLE_DEVICES='' PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 nice -n 10 taskset -c 6,7,12,13 timeout -k 10 180 prlimit --as=4294967296 /home/fair/miniconda3/envs/aerialgym/bin/python -B -m unittest discover -s tests -p test_actionhold_hook.py
```

After source/preregistration corrections are reviewed and committed, with no concurrent GPU or uncapped CPU job:

```sh
PYTHONPATH=/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 taskset -c 0-3,8-11,14-23 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/actionhold_run.py --approved-commit FULL_CLEAN_HEAD --out /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/actionhold_g7_v1 --max-hours 1.5
```

Use the project's established GPU memory/time wrapper; retain per-cell timeout1800s and clean-tree freeze. If an
outer supervisor aborts, remember `gr_run.run_launcher` starts a separate process group, so terminate its descendants
as well. Do not edit files after launch or pre-create the output folder.

After COMPLETE, run `tools/actionhold_analyse.py G7_RUN_DIR FRESH_OUT_JSON` with CUDA hidden, threads1, timeout300,
4GiB memory cap and the same CPU affinity. Only NumPy and standard library are imported by the analysis; no Torch or
Isaac is needed. Bootstrap temporaries are about 328MB for simultaneous index/value arrays at 10000×2048; cap4GiB
is sufficient for the expected data. A few seconds is a reasonable analysis estimate, not a measured runtime.

## Measured runtime basis

Read from existing reports, cells at seed4501. No timing experiment was performed.

| Reference | S00 /70 cell | S00 /205 cell | 3×70 + 4×205 |
|---|---:|---:|---:|
| GR0 native F | 109.34s | 170.33s | 16.82min |
| GR1b rand40_s2 | 113.04s | 224.50s | 20.62min |
| GR1b ctrl_s2 | 120.89s | 163.50s | 16.94min |

Reports: `results/gain_randomization_gr0_v1/gr0_main/gr0_run_report.json` and
`results/gain_randomization_gr1b_v1/eval_{rand40,ctrl}_s2/gr1eval_run_report.json`, under the shared main results root.
Native F is the best contract match; the other policies bracket recent host conditions only. Changed actions can
change episode length, so allow about20–30min GPU, then independent review/recording roughly10–20min in addition to
G9 preparation/execution. These are estimates; held/delayed trajectories have not been measured.

## Results review and reporting

Readings are per-arm TOLERANT/HARMFUL/INCONCLUSIVE exactly as preregistered. k2 HARMFUL rules out unmodified F's
open-loop200ms chunks; d1 HARMFUL motivates a sub100ms bridge or a separately approved latency-aware policy.
Do not infer training permission or a real-flight claim. Scope remains one seed/two densities.

Give a fresh independent reviewer the actual committed source, fixed criteria, complete report/receipts/sidecars and
raw ledgers, not the author's conclusion. Record reviewer/model; in Claude Code use actual Opus5.5. Until review
finishes, keep conclusions computed/conditional. Preserve failures and write any rerun into a new folder. Record
the completed unit, its evidence/index connection and CPU windows in WORKLOG only after all guarded runners end.
