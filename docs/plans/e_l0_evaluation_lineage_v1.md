# E-L0 — evaluation-only lineage: contract and fixture matrix (preregistration v1)

Status: **approved by the user on 2026-10-18** ("MOTAR — EQ-1D 결과 검토 및 E-L0 평가 계보 개발 승인").
The machine-readable contract is [`e_l0_evaluation_lineage_v1.json`](e_l0_evaluation_lineage_v1.json);
where the two differ, the JSON governs. Both are committed before any E-L0 GPU process.

## What E-L0 is

An opt-in recorder (`NAVRL_KROC2027_E_L0=1`, which needs `NAVRL_KROC2027_RECORDER=1`), built by the
existing evaluation adapter. It only reads task tensors, chains the env manager's physics-step
callback, and wraps `perception._associate_lidar_target` with a call-through that returns the
original outputs. It never changes Policy F's observation, action, normalisation, perception,
tracker, controller, physics or termination. Evaluation GT is written to files only. No actor
and no online supervisor reads it.

It records:
- 100 Hz contact chronology with the first contact time;
- 10 Hz observation/action timing;
- camera and LiDAR identity provenance, from the renderer's target mask and the LiDAR semantic
  buffer;
- camera / LiDAR / fused validity;
- correct and false association;
- pixel-centre silhouette classes: full, partial, centre-only, FOV exit, range exit, scale loss;
- tracker age and expiry;
- per episode: the unchanged task outcome plus `capture_with_contact`, `collision_free_capture`,
  evidence-based crash causes and `timeout_at_rest`.

`safe_abort` is recorded as `NOT_IMPLEMENTED`, and no synthetic success event is produced.

## Validation (GPU run `EL0-R1`)

- **V3 non-interference.** Three configurations: static and moving targets at 160 bars with N = 8 for
  600 steps, and static with N = 128 for 320 steps. Each runs in three arms:
  - P0: Policy F only;
  - P1: recorder plus the OCC trace;
  - P2: P1 plus E-L0.

  The arms run on the same commit, seed and initial conditions. Per step they must have identical
  898-D observations, deterministic actions, pursuer and target trajectories, done/capture/crash
  flags and reset envs, RMS digest, and sensor and tracker checksums. Any mismatch stops the run
  before the next process.
- **V1 label fixtures.** The 32 EQ-1D D2 scenes plus four association scenes (correct, stale-track
  false, out-of-gate, camera-visible). E-L0 labels are compared with independent CPU oracles:
  pixel-centre analytic geometry and the analytic LiDAR first hit. The bar-only negative scenes
  are included.
- **V2 contact fixtures.** The 24 EQ-1D D1 fixtures at 10 ms. E-L0's contact summary is compared
  with the independent substep recorder and the SAT geometric window.

Gates E0-N1…N6, E0-L1…L3, E0-C1…C3 and E0-T1…T2 are in the JSON. The EQ-1D 7/7 false accepts
remain a known failure of the production association, not a population rate. The historical
capture label is kept, and `collision_free_capture` is an added column. OCC-P2 results and
populations are not changed.

## Execution contract

As EQ-1D:
- the plan and code are committed first; the approved commit equals HEAD on a clean, frozen tree;
- imports come from the worktree;
- Policy F and the RMS digest are verified;
- each process has its own watchdog and a GPU memory cap measured at launch;
- the run stops below 8 GB free disk and above 1.5 GB of output, with no retry;
- nothing tracked is edited during the run;
- after the run, the manifest is verified and the results get a separate local commit;
- no push or merge.
