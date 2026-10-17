# EQ-1D — GPU environment qualification of the frozen Policy F simulator (preregistration v1)

Status: **approved by the user on 2026-10-18** ("MOTAR — EQ-1D GPU Qualification 승인 및 후속 환경
수정 우선순위"). The machine-readable contract is [`eq1d_qualification_v1.json`](eq1d_qualification_v1.json);
where the two differ the JSON governs. This file and the JSON are committed before any EQ-1D GPU
process. Fixtures and gates do not change after execution.

## Purpose and limits

EQ-1D checks that the dynamics, sensors and information flow of the current simulator, as actually
executed on the GPU, agree with independent references. It tests no algorithm, Recovery Planner,
PPO or H-R hypothesis. A PASS does not certify hardware physics or real-camera redetection.
EQ-1A/B/C's known FAILs are not fixed. They are reproduced or observed on the GPU and recorded
separately from the CPU mathematics.

## Batches

- **D1 plant and timing** (3 processes, dt 10 / 5 / 2.5 ms, 24 fixtures, 6 s each, 0.1 s actions):
  scripted vehicle-frame velocity commands through `sim_env.step`. Every substep is logged:
  pose, velocity, angular velocity, command, motor reference and actual thrust, force/torque,
  contact force. Gates: control/physics period separation, motor fidelity and intended-lag
  agreement, dt convergence (0.01 m / 0.5°), hover thrust, velocity tracking, braking against
  the riskcap model, tilt bound, and contact onset against an oriented-box overlap test.
  Pre-contact, contact and post-contact states are separated.
- **D2 sensors and rendering** (1 process, 32 scenes × 3 attitudes × with/without bars): the
  production camera and LiDAR on placed scenes, against analytic pixel/ray tables (EQ-1A
  references). Gates: target mask/depth, native 40 × 24 obstacle depth, delivered 160 × 90 depth,
  LiDAR ranges, the yaw-only mount, fusion, camera measurement, occlusion classes, scale,
  camera and LiDAR negatives, capture/delivery counters.
- **D3 task-level GT firewall** (4 processes, N = 4 / 1 / 2 / 128): Policy F in closed loop. At branch
  steps the observation is recomputed with the step's rendered sensors replayed while only
  privileged/evaluator/critic fields change, all envs, permuted or a subset. A positive control
  re-renders after moving the target. Gates: bitwise 898-D actor rows and deterministic actions,
  RMS digest, delivery latency, batch sizes.

Sensor-changing interventions (the positive control) and actor-leak tests (replayed sensors) are
separate branches. A mismatch records the tensor, source edge and reproduction step.

## Execution contract

- Before launch: this plan, the launcher (`tools/eq1d_fixture_launcher.py`), telemetry
  (`motar_eval/eq1d_telemetry.py`), orchestrator (`tools/eq1d_run.py`) and analysis
  (`tools/eq1d_analysis.py`) committed. The approved commit equals HEAD, the tracked tree is clean,
  `aerial_gym` and `motar_eval` resolve inside the worktree, Policy F's SHA is verified, the output
  root is fresh, and free disk is ≥ 8 GB.
- Watchdog: each process has its own wall cap (20/20/20/20/30/30/30/40 min). There is no total GPU
  budget. The GPU memory cap is measured at launch from the actual free memory, other processes
  and a 512 MiB OOM reserve, and is recorded.
- Integrity failures stop the whole run with no retry. Qualification FAILs never stop it.
- During the run nothing tracked is edited, WORKLOG included. Afterwards: manifest
  verification, CPU analysis, preservation checks, then a separate local result commit. No push or
  merge.
- Development smoke runs of the launcher mechanics are allowed only in the session scratchpad. They
  are not evidence and do not change this plan.
