# Control-gain randomization (GR) evidence, from 2026-11-04

Family `gain-randomization` ([`docs/EXPERIMENTS.md`](../../EXPERIMENTS.md)). Decision D-2026-11-04-1. The
preregistration is [`docs/plans/prereg_gain_randomization_gr0_v1.md`](../../plans/prereg_gain_randomization_gr0_v1.md),
amended by [`..._amendment1_2026-11-04.md`](../../plans/prereg_gain_randomization_gr0_v1_amendment1_2026-11-04.md).
The raw outputs stay in the main checkout, in `aerial_gym_simulator/results/gain_randomization_gr0_v1/` (277 MB,
303 files). They are bound by path and SHA-256 in [`SHA256_MANIFEST.json`](SHA256_MANIFEST.json).

## Identity stage

| Attempt | Commit | Verdict | Record |
| --- | --- | --- | --- |
| identity_01 | `d38b247` | **IDENTITY_FAIL**: the runner lacked the KROC2027 initial-view placement, so the recorder stopped all four cells (base tree included) when a target spawned overlapping a bar; no comparison made | [`identity_01_report.json`](identity_01_report.json) |
| identity_02 | `c56be07` (amendment 1, A3 placement) | **IDENTITY_PASS**: `off` = base tree `62e59c0` (result and every episode row identical); `s00` (switch ON, scale 0) = `off`; `s40` differs (capture 240 -> 224 of 256) | [`identity_02_report.json`](identity_02_report.json) |

## GR-0 (`c56be07`, 2026-11-04 11:58 – 13:25 KST)

- Policy F, `E2_obstacle_aware`, A3 placement.
- 70 / 115 / 160 / 205 bars x seeds 4501-4503 x levels S00 / S20 / S40, 2,048 quota episodes per cell.
- Run verdict COMPLETE: 36 / 36 cells passed every runner check, with 0 kernel fault records and the online CPU list
  unchanged ([`gr0_run_report.json`](gr0_run_report.json)).
- The analysis is [`gr0_analysis.json`](gr0_analysis.json), from `tools/gr_analyse.py` as preregistered.

| Level | Gains | Capture | Crash | Delta capture [95 % CI] | Delta crash [95 % CI] | Reading |
| --- | --- | --- | --- | --- | --- | --- |
| S00 | nominal (OFF) | 95.54 % | 4.46 % | – | – | – |
| S20 | nominal x [0.8, 1.2] | 94.92 % | 5.07 % | -0.62 pp [-1.01, -0.26] | +0.61 pp [+0.25, +0.98] | **GAIN_ROBUST** |
| S40 | nominal x [0.6, 1.4] | 91.85 % | 8.15 % | -3.69 pp [-4.36, -3.03] | +3.69 pp [+3.03, +4.37] | **GAIN_SENSITIVE** |

Crash rate by density, in %:

| Level | 70 bars | 115 bars | 160 bars | 205 bars |
| --- | --- | --- | --- | --- |
| S00 | 1.24 | 2.90 | 4.88 | 8.81 |
| S20 | 1.68 | 3.29 | 5.81 | 9.49 |
| S40 | 3.53 | 5.88 | 9.62 | 13.57 |

At S40, all 12 blocks show more crashes than S00 (+1.9 to +5.4 pp).

**Exploratory, not preregistered** ([`gr0_crash_causes_exploratory.json`](gr0_crash_causes_exploratory.json)).
These counts come from each cell's aggregate result window, not from the quota ledger.

| Crash cause | S00 | S20 | S40 |
| --- | --- | --- | --- |
| bar contact | 1,076 | 1,194 | 1,568 |
| below (floor) | 24 | 72 | 447 |
| out of bounds | 6 | 7 | 4 |

Floor strikes rise about 18-fold from S00 to S40. This pattern is consistent with degraded altitude tracking under
detuned gains; the mechanism is not tested.

**Next-step rule (preregistered).** S40 reads GAIN_SENSITIVE, so training with gain randomization (GR-1) is
motivated and gets its own preregistration. The readings stand as computed only after the independent review.
