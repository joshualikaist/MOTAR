# Project state

A short index of where MOTAR stands now. It is not a log; history lives in [`HISTORY.md`](HISTORY.md),
[`WORKLOG.md`](../WORKLOG.md) and the evidence folders. Established results and their limits live in
[`EVIDENCE.md`](EVIDENCE.md); what the project is lives in [`PROJECT.md`](../PROJECT.md). Every status
below names its record. Where this file and a record disagree, the record is right.

**Last verified:** 2026-11-28 KST, on `main` after the merge of `camonly/s2b` (unit PUB-2026-11-28).
Newer state is at the bottom of [`WORKLOG.md`](../WORKLOG.md).

## In one line

MOTAR is a simulation study of an aerial pursuer that tracks and closes in on a moving target among
dense vertical obstacles. The historical line uses RGB-D camera + LiDAR fusion and one frozen policy
(Policy F). The current line (since November 2026) trains camera-only policies for a real vehicle that
carries no LiDAR.

## Research lines

| Line | Status | Record |
| --- | --- | --- |
| Sensor contract | RGB-D camera + LiDAR governs Policy F and the historical records; the real-vehicle line is camera-only by user decision (D-2026-11-04-3, D-2026-11-06-1) | [`SENSOR_CONTRACT.md`](SENSOR_CONTRACT.md), [`DECISIONS.md`](DECISIONS.md) |
| Camera-only line, simulation | CAM-S3-E: S3-W crash 25.4 %, BAR_NOT_MET_POINT; W − S0 NO_IMPROVEMENT; pointer STRUCTURAL_NEXT; results review PASS_WITH_LIMITS | [`evidence/camonly_2026-11-04/cams3/`](evidence/camonly_2026-11-04/cams3/REVIEW_E_results_r1.md) |
| Camera-only line, vehicle runtime | RT-S3W: S3-W observation and action match the training-side functions on 7 analytic CPU scenes (built from the runtime's own configuration); review GO for a desk / hand-held shadow test only; no flight approved | [`evidence/camonly_2026-11-04/rt_s3w/`](evidence/camonly_2026-11-04/rt_s3w/REVIEW_r1.md) |
| Outdoor depth (hand-held, 2026-11-26) | Ground read as near obstacles depends strongly on the assumed camera height; sky false-near returns are a few percent of frame-bins | [`evidence/camonly_2026-11-04/outdoor_1126/`](evidence/camonly_2026-11-04/outdoor_1126/README.md) |
| Target-motion replication (historical) | REPLICATED, 7 seeds | [`../results/target_motion_replication_7seed/`](../results/target_motion_replication_7seed/README.md) |
| Classical baselines B1-R (historical) | Policy F above both reactive baselines; C1/C2 SUPERIOR | [`evidence/b1r_v1/`](evidence/b1r_v1/analysis_summary.json) |
| BOIA (historical RGB-D + LiDAR line) | Shadow-only; Q02-B and Q03 review FAIL / HOLD; G5 HOLD | [`research/boia-progress.md`](research/boia-progress.md) |
| Sensor-fusion corrections S-L1 | S-L1A/B/C/D v1 GO in their recorded scopes | [`evidence/s_l1c_v2_2026-10-18/`](evidence/s_l1c_v2_2026-10-18/QUALIFICATION.md) |
| KROC2027 study | H-A supported; H-A2/I_A and H-B inconclusive | [`evidence/mt_precision_review_2026-10-16/`](evidence/mt_precision_review_2026-10-16/README.md) |

All families, with what to read first and what compares, are in [`EXPERIMENTS.md`](EXPERIMENTS.md).

## Branch and phase

- **Branch:** `main` carries all research after the 2026-11-28 merge of `camonly/s2b` (which contained
  `camonly/dev`). GitHub keeps `main` and `archive/*` tags only; no new branches (user, 2026-11-26).
- **Phase:** camera-only line after CAM-S3-E. Fine-tuning the S2b-FT → S3-W lineage stopped
  (STRUCTURAL_NEXT). The next design (memory without a map, a separate target channel, measured depth
  artefacts, latency) is for the user to choose; every GPU run needs approval and a committed preregistration.
- **Vehicle:** OAK-D, VIO, PX4, Jetson Orin NX, about 1.1 kg (D-2026-11-08-1, D-2026-11-18-2). The S3-W
  bundle goes to a desk / hand-held shadow test; no autonomous flight is approved.

## Known problems and blockers

- **Not flight-ready.** S3-W crashes in 25.4 % of simulated episodes; ground handling of outdoor depth and
  sky false-near returns are open; the target detector for an unmarked drone is not integrated.
- **Shadow-test checks open (RT-S3W review P3):** the top-band invalid share of the vehicle's depth must match
  training; Orin NX step time is not measured.
- **Review findings open before any CAM-S3-E rerun:** package review r2 P2-1 (output root must lie under the
  unit folder) and P3-1..8.
- **PUBLIC_CPU on `main` (2026-11-28):** 4,664 tests, 4 failures, 0 errors, all pre-existing: the CAM-S2a-E and
  CAM-K1-E receipt checks, the SITL S4R1 prereg command check, and the CAM-S3-E launch-allowlist guard (failing since
  RT-S3W `4f6b709`; Beads `motar-7zc`).
- **Host faults:** cpu4/5 machine-check history; check the kernel crash count before and after long runs.
- **No external backup.** Raw results are local only (BACKUP_DECLINED_BY_USER).

## Recent decisions

Full list with reasons: [`DECISIONS.md`](DECISIONS.md).

- **2026-11-28:** merge `camonly/s2b` into `main`, publish with README and site rewritten (unit PUB-2026-11-28).
- **2026-11-27:** S3-W trained without false-near measurement; CAM-S3-E evaluated under its amendment 1.
- **2026-11-26:** GitHub keeps `main` plus `archive/*` tags; no new branches.
- **2026-11-23:** the simulated target is the same quadrotor; capture = centre distance ≤ 1.0 m at
  closing speed ≤ 1.0 m/s.
- **2026-11-06:** the real vehicle has no LiDAR; camera-only lineage on the OAK-D.
- **2026-11-04:** new PPO training allowed again (D-2026-11-04-1); camera-only study opt-in (D-2026-11-04-3).

## Next priorities

Run `bd ready` for the live list. Open decisions for the user:

- the next structural design for the camera-only line;
- a measured outdoor depth-artefact model (CPU) and the vehicle-side shadow test results;
- the hold-k / delay-d evaluation probe (GPU, needs approval).

## Where things are

- **Commands, test profiles and run rules:** [`OPERATIONS.md`](OPERATIONS.md). The default check is
  `python -B tools/run_tests.py`.
- **Experiment families:** [`EXPERIMENTS.md`](EXPERIMENTS.md).
- **Plans:** [`plans/`](plans/). **Evidence:** [`evidence/`](evidence/). **Long-term direction:**
  [`ROADMAP.md`](ROADMAP.md).
- **Tasks:** Beads (`bd ready`, `bd show <id>`). The database is untracked and local, in the main
  checkout's `.beads/`.
