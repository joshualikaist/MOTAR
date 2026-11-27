# EQ-1D — simulator qualification: GPU validation design and reproducibility contract (proposal)

Status: **proposal for approval, nothing implemented or run.** EQ-1D qualifies the frozen
environment only. It does not test the Recovery Planner, train PPO, test a new hypothesis or
implement a correction lineage. The user removed the total GPU budget; each process below has its
own safety cap so that a failure or a hang cannot run unbounded.

## Questions EQ-1D answers (left open by the CPU packages)

| Batch | Question | Why CPU could not answer it |
| --- | --- | --- |
| D1 plant and time | Do pose/attitude converge as the physics dt shrinks (10 → 5 → 2.5 ms, actions held at 0.1 s)? Does the logged motor response follow the CPU oracle? When does contact happen within the step, at what speed? | PhysX / Lee controller / allocator run only in Isaac Gym; 10 Hz records hide substeps (B2) |
| D2 sensor scenes | Do the Warp camera and LiDAR reproduce the EQ-1A analytic labels (target-alone support, partial / centre-only / full, scale loss, FOV edge), under body tilt versus the declared yaw-only mount, with target-absent bars? | A1–A7 used analytic frames; the renderer itself was not run |
| D3 task-level firewall | With the rendered sensor tensors held fixed, does mutating privileged state (target pose/velocity, critic extras, GT visibility, semantic IDs, evaluator fields) leave the 898-D actor input and Policy F's deterministic actions bitwise unchanged? Is the RMS state unchanged after a play window? | C1b was module-level; the task, renderer and Runner need Isaac Gym |

## Fixture matrix

- **D1:** 24 deterministic trajectories (hover; velocity steps ±1/±2.5 m/s on x, y; reversals; yaw
  steps; maximum-tilt command; saturation; braking from 2 and 3.5 m/s; 4 contact-corner approaches to
  a single bar at 1, 2, 3 m/s and 30°/60°) × dt 10/5/2.5 ms. One process per dt (sim dt is global),
  24 envs, no obstacles except the contact fixtures, open-loop scripted velocity commands (no policy),
  6 s physical time each. Inner-step log: pose, velocity, angular velocity, command, motor reference
  and actual RPM/thrust, net force/torque, contact flags.
  - Gates (from the EQ-1 plan, fixed now): non-contact position error 5→2.5 ms ≤ 0.01 m and attitude
    ≤ 0.5° over the 6 s horizon, with the 5→2.5 error smaller than 10→5. Motor: logged RPM against
    the CPU formula fed with the logged inputs ≤ 1e-5 relative, and against the ZOH oracle reported.
  - Contact: report first-contact substep, speed and tilt per dt. No pass gate (characterisation).
- **D2:** 32 static scenes (the 11 A5 fixtures, 4 A7 measurement fixtures, 8 FOV-edge and
  scale-band targets at 8/12/16/20/24 m, 9 target-absent bar controls) × 3 body attitudes (level,
  15° roll, 15° pitch). Each scene is rendered twice: target alone (obstacles moved out of the
  frustum) and with obstacles. Logged: RGB/depth, LiDAR ray table, poses, render/capture/delivery
  step stamps, detector outputs, association provenance (evaluator-only), covariance/age.
  - Gates: EQ-1A gates on rendered output (direction ≤ 0.5 px, range ≤ 1e-4 m on exact fixtures,
    labels equal to the analytic labels). Current production is expected to FAIL A1/A2/A4/A5/A6
    exactly as on CPU, which is recorded, not relaxed. Tilt fixtures check the yaw-only mount claim.
- **D3:** Policy F task (OCC-P2 receipt environment, static 115 bars, seed 4401) with N = 4, plus
  N = 1/2/128 shape checks. At steps 50/150/300 the task state is snapshotted; branch A continues;
  branch B mutates only privileged fields while the cached rendered RGB/depth/LiDAR tensors of that
  step are re-injected. Compare actor rows and deterministic actions bitwise within each batch shape
  (permutations undone); record cross-shape differences separately. RMS digest before and after a
  600-step play window must equal `642fc9f7…`.

## Implementation footprint for the later approval package (not created)

New files only: `tools/eq1d_fixture_launcher.py` (scripted D1/D2/D3 driver), `motar_eval/eq1d_telemetry.py`
(inner-step and sensor-scene recorder, evaluation-only), `tests/test_eq1d_*.py` (CPU parts),
`docs/plans/eq1d_qualification_v1.{md,json}` (preregistration). No change to existing sensor,
physics, controller, task or Policy F source; runtime hooks live in the launcher and are
declared in the preregistration.

## Estimates (NOT_MEASURED)

| Batch | Processes | Expected wall each | Safety cap each | Output |
| --- | ---: | --- | --- | --- |
| D1 | 3 | 3–8 min (startup dominated) | 20 min | ≈ 70 MB |
| D2 | 1 | 5–10 min | 20 min | ≈ 50 MB |
| D3 | 4 | 4–10 min | 30 min | ≤ 300 MB (sampled rows) |
| Total | 8 | ≈ 0.5–1.0 GPU-h | — | ≤ 0.5 GB expected, 1.5 GB hard limit |

Basis: OCC-G0 used 0.2472 GPU-h for 16 production cells and OCC-P2 2.4927 h for 112. These do not
benchmark fixture speed. The first process measures its own startup overhead; if any process is
projected past its cap, the batch stops before the next process.

## Stop conditions

- First failure stops the batch: nonzero exit, NaN/Inf in a logged state, a missing output file, or
  a gate-computation error. No automatic retry; a rerun needs a new run id and a note.
- Per-process wall cap reached: kill, keep the partial output, mark `CAPPED`.
- Free disk below 8 GB (now about 18 GB) before any process: stop. Output above 1.5 GB: stop.
- GPU memory above 7.5 GB, or another GPU job present at launch: stop or refuse.
- Any change to a tracked file or HEAD during the run, Policy F SHA mismatch, or `aerial_gym`
  resolving outside the worktree: refuse to start, or stop.

## Reproducibility contract

1. The preregistration, launcher and telemetry are committed locally before launch. Approved commit
   = HEAD, tracked tree clean, `PYTHONPATH` pinned to the worktree, imports fail closed outside it.
2. Pinned inputs: Policy F `f7022139…`, receipt environment snapshot, seeds, fixture table, and the
   TF32 state recorded per process. rl_games enables TF32 in the Runner; D1/D2 run without the Runner
   and keep it off; D3 runs as historical evaluation did and compares only within the same setting.
3. Output to a fresh root `../aerial_gym_simulator/results/eq1d_qualification_v1/<RUN_ID>/` with a
   RAW_MANIFEST (path, bytes, SHA-256) and per-process receipts (command, environment, GPU,
   wall seconds, bytes). Nothing is overwritten; failed attempts are kept.
4. During the run: no tracked edits, WORKLOG included; no heavy concurrent CPU or disk work.
5. After the process exits: validate manifests, re-hash Policy F and the historical raw manifests,
   run the CPU gate scripts on the outputs, then make a separate local result commit with compact
   evidence. No push or merge without approval.

## Explicitly out of scope

Recovery Planner, PPO / retraining, new hypotheses, constructed 5 s loss scenarios, any correction
lineage (E-L0, P-L1, S-L1, G-L1), and changes to historical results.
