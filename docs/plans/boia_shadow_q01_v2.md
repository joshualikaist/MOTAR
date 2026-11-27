# BOIA-SHADOW-Q01-V2 — passive N1 qualification preregistration

2026-10-21 KST. G4-R7 authorizes implementation and CPU verification, **not GPU**.
This document and the [executable configuration](boia_shadow_q01_v2.json) must be in the
exact source commit the user approves for execution; the launcher's `--approved-commit`
binds that commit. No source, config, test, WORKLOG or tracked document may change while
a GPU process runs. It replaces Q01 v1, which was withdrawn before execution; see the
[v1 amendment](boia_shadow_q01_v1_amendment_2026-10-21.md).

**Purpose.** Qualify permitted-packet acquisition, phase/ID joins and passive logging, and
test three separate exact comparisons. No mission-effect, H-R3, physical-safety,
identity-calibration, continuation-model or real-time-planner claim is tested.
RGB-D + LiDAR, the 898-D actor, Frozen F, riskcap and the controller stay unchanged.

## Profile (unchanged from v1)

- N1, seed 4901, moving `E2_direct_l2_v1`, 70 bars, initially visible at 10 m. Seeding happens
  twice: loading the rl_games runner seeds the global generators with the player yaml's
  `seed: 42`, then building the task reseeds numpy, torch, CUDA and `random` with
  `task_config.seed`, read from `NAVRL_SEED=4901` (the player path passes no seed argument).
  4901 is therefore the effective seed; the runtime profile requires `task_config.seed == 4901`.
- OFF sensor-correction lineage. Native F sensing, physics and RNG are retained: no reset or
  RNG replacement, sensor correction, intervention, PPO or branch simulator.
- The runtime environment is identical to v1's (same keys and values). `NAVRL_DETECTION_DROPOUT`,
  `NAVRL_RGB_NOISE_STD` and `NAVRL_DEPTH_NOISE_STD` are present but inactive: with
  `NAVRL_PERCEPTION_PERTURB=0`, `observe` runs with `training=False`, and the perception
  code applies dropout and noise only when `training=True`. Runtime checks attest this.

## Stages, in this order

| Stage | Observer (EvaluatorTap + ContactChain) | BOIA diagnostic | Actions / wall | Snapshots |
|---|---|---|---|---|
| S0 | ON | ON (stride 5) | 20 / 180 s | 2 |
| S1_NATIVE | OFF | OFF | 200 / 300 s | 0 |
| S1_OFF | ON | OFF | 200 / 300 s | 3 |
| S1_ON | ON | ON (stride 5) | 200 / 300 s | 3 |

Then `verify` runs on the CPU. Each stage is a fresh process and stops at its first
completed episode; the native 600-action episode cap is unchanged. A stage starts only if
every earlier stage's supervisor receipt is COMPLETED. The cumulative wall budget is
**1080 s**, the sum of the stage caps; there is no retry or automatic expansion. S0 has
180 s rather than 120 s because N1 has never run and its cold start is unmeasured.

`S1_NATIVE` installs no EvaluatorTap, no ContactChain and runs no diagnostic; a runtime hook
audit must show none live. Every stage, `S1_NATIVE` included, runs the same player loop and
writes the same read-only host trace (`host.jsonl`): actor-input hash taken before the policy
call, F action bytes and values, RNG-state fingerprints before and after the policy and after
the step, returned (including reset) observation hash, reward, done, and evaluator-side state
hashes, including `sim_steps`, which drives the native time-limit truncation. Host-side reads therefore exist in every stage; Q01-V2 compares no fully
uninstrumented run.

## Comparisons (exact; no tolerance, row exclusion or salvage)

1. **S1_NATIVE vs S1_OFF**, `host.jsonl`, full length — whether installing the observer and
   logger changes the host-visible Frozen F path.
2. **S1_OFF vs S1_ON**, `parity.jsonl` and `host.jsonl`, full length — whether the diagnostic
   computation changes anything over an installed observer. S1_ON also stops online at its
   first mismatch with S1_OFF records.
3. **S0 vs S1_ON**, the same files, S0 prefix — fresh-process replay of the ON configuration
   over at most 20 actions. Lengths must match unless S0 stopped at its own action cap.
   By design it cannot separate replay instability from interference for a mismatch after the
   S0 prefix, or for one specific to the NATIVE process: there is no NATIVE replay control.

The terminal/reset step is compared only when both compared stages end an episode within their
own action cap (20 for S0, 200 otherwise), so the S0/S1_ON replay control will usually not cover
it; `verify` reports this per comparison as `terminal_reset_compared`.

Each stage also records torch/CUDA/cuDNN versions, device identity, `cudnn.benchmark`,
`cudnn.deterministic`, both TF32 switches and float32 matmul precision. They must be equal
across stages and unchanged within a process. Q01-V2 never changes them; rl_games enables
TF32 and `cudnn.benchmark` for Frozen F.

Verdicts: `PASS_EXACT_NATIVE_OFF_ON_PARITY_WITH_REPLAY_CONTROL` only when every comparison
passes. Any mismatch is `REPLAY_INSTABILITY_OR_PROFILE_MISMATCH`, with the first row, the
differing fields and both stages' runtime attestation recorded, and cause `NOT_VERIFIED`:
it is not attributed to the observer or to CUDA nondeterminism without separate evidence.
Missing stages give `INCOMPLETE_CAMPAIGN_NOT_EVALUATED`. Every stage also needs its supervisor
receipt: COMPLETED, matching stage/source/config/checkpoint, worker return code 0 and no stop
reason. A supervisor stop (e.g. `DEVICE_VRAM_CEILING`, `CHECKPOINT_CHANGED`), a missing or
unreadable receipt, or a worker receipt that contradicts it gives
`STOPPED_INTEGRITY_NOT_EVALUATED`, which overrides every other verdict and allows no SUPPORTED
claim. (Correction of 2026-10-21, made before any execution: the first v2 verifier read only the
worker's receipt and could report PASS for a stage its supervisor had stopped.) Observer noninterference is
supported only if comparisons 1 and 3 and the runtime profile all pass; diagnostic
noninterference only if 2 and 3 and the runtime profile pass.

## What Q01-V2 measures and does not

- Sensor acquisition and packet validity: measured.
- BOIA utility ABSTAIN; safety certificate UNAVAILABLE; WAIT/INTERVENE computation
  NOT_QUALIFIED; physical safety NOT_ASSESSED; actual owner F, shadow action never applied.
- `diagnose()` reads the join key and clocks only, never sensor values. Its time is
  diagnostic-only latency, `NOT_INFORMATIVE_FOR_FULL_PLANNER`; full-planner latency is
  NOT_MEASURED. No mission-utility or capture-gain claim.
- Camera/LiDAR timing is DECLARED_SIM_SCHEDULE relative to the first explicit reset, not
  measured hardware time. Missing clocks refuse evidence.

## Limits and stop conditions

- Preflight free space ≥ 3.3 GB; stop below 3 GB. New output ≤ 256 MB in total
  (64 MB logger cap per stage, 8 MB host-trace cap); at most 8 full sensor snapshots.
- Device planning ceiling min(6656 MiB, total − 768 MiB). VRAM is sampled every 2 s by the
  supervisor and is not a true peak; actual VRAM NOT_MEASURED until a run.
- Supervisor: resource checks every 2 s, source/config guard every 10 s, shutdown 4 s before
  the wall cap with SIGTERM and a 3 s grace. The worker turns SIGTERM into `failure.json`
  when it can, and the supervisor receipt embeds the worker's own first cause.
- Observer watchdog 2 s per sampled observation, or three consecutive > 500 ms.
- Any source/config/checkpoint, import, packet/phase/count, F-action, hook-state, reset, RMS,
  numerics, logger, resource or online OFF/ON mismatch stops that stage. Process caps are
  censoring, never task timeouts. Failed and partial files stay.

Ground truth stays evaluator-side: outcomes and target/obstacle hashes never enter
`SensorPacket` or diagnostics. Pre-mask contact and height/OOB are captured before the
original compute masks its crash buffer; substep contact uses ContactChain at 0.05 N. The
collision-free capture proxy (capture, zero thresholded contact, no action-boundary
height/OOB) does not certify absence of sub-threshold contact or between-boundary excursion.
B03/B04 remain unresolved. **G5 HOLD.** A full PASS qualifies these mechanics only.

Output root: `/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2`.
Commands: [GPU runbook](../evidence/boia_g4r7_shadow_v2_preflight_2026-10-21/GPU_SHADOW_RUNBOOK.md).
