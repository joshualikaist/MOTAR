# Q01 runbook — requires separate GPU execution approval

The commands below are prepared, **not executed** by G4-R6. Run from
`/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1` with the pinned aerialgym Python.
Set BOIA_SOURCE_SHA to the exact commit approved by the user. The command below
obtains HEAD for convenience; compare it to that explicit approval before launching.
Preflight executes no torch/Isaac/GPU code or checkpoint unpickling.

```bash
BOIA_SOURCE_SHA=$(git rev-parse HEAD)
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py preflight --approved-commit "$BOIA_SOURCE_SHA"
```

Only after explicit GPU approval, execute in this order. Stop on a nonzero exit;
never continue after a failed receipt. Each command has a supervisor and wall cap.

```bash
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s0 --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s1-off --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py s1-on --approved-commit "$BOIA_SOURCE_SHA" --execute-gpu
PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/boia_shadow.py parity --approved-commit "$BOIA_SOURCE_SHA"
```

Alternatively, the approved `campaign` subcommand runs S0→S1_OFF→S1_ON with those
same gates and stops at the first failure. No retry or automatic expansion.
An existing campaign/stage path cannot be reused. Do not delete it to retry;
review the failed evidence and create a separately approved amended RunID/source.

Inputs: committed [JSON configuration](../../plans/boia_shadow_q01_v1.json), frozen
Policy F checkpoint SHA f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40
at its existing path; current approved source and unchanged player/production code.
The copied runtime environment is committed; the historical receipt is provenance,
not a runtime dependency or a source of replayed sensor data. Native F RNG is not
reseeded/replaced by the observer.

Output: `/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01/`.
S0/S1_OFF/S1_ON contain sensors/actions/shadow/evaluator/parity/events JSONL,
bounded sensor NPZ snapshots, process_done or failure receipt. Each parent stage
adds a supervisor receipt and SHA-256 RAW_MANIFEST including partial outputs.
Source/config/checkpoint hashes and stage profile are recorded. Nothing is written
to historical results/checkpoints. Do not edit tracked files during execution.

Limits: N1/seed4901/moving70bars;20/200/200actions;120/300/300s per process,
720s cumulative; stop at first completed episode; native600action episode cap;
3.3GB preflight /3GB stop;256MB total output;8 full sensor snapshots;
VRAM ceiling min6656MiB,total−768MiB. The observer is sampled every5actions;
copy/join and exact parity run everyaction. Watchdog2s or three sampled pipeline
times>500ms. Host cold/warm copy/schema/diagnostic/SHA/log timing and full-step
timing are separated. Actual GPU/compile/VRAM/timing remain GPU_PENDING.

GT firewall check: `sensors.jsonl` has a strict fixed field list and scalar
metadata; `shadow.jsonl` carries refusal/hypothesis diagnostics only. Target/obstacle
truth fingerprints and pre-mask outcomes are only in evaluator/parity records;
these are never passed to diagnose. Full sensor snapshots contain permitted
RGB-D/LiDAR/track/own-state fields, no semantic50 or GT target state.

Parity gates compare every completed action and its returned/reset sensor boundary,
including the terminal action. First pre-action/command mismatch stops before
physics; later mismatches stop before another action. Final parity also requires
equal nonempty trace lengths and matching completed metadata/source/config/checkpoint.
Only PASS_EXACT_OFF_ON_PARITY supports noninterference for this exact profile.

Qualification is not efficacy: unsupported utility stays ABSTAIN and physical safety
NOT_ASSESSED. Censored outcomes are not timeouts. The new collision-free proxy
is thresholded contact +action-boundary height/OOB qualified. B03/B04, actual F
continuation/value calibration, physical C02/backup/deadline/endgame remain open.
G5 HOLD even if all Q01 gates pass.
