# Independent bias-corrected replication of the target-motion result (7 fresh seeds)

**Verdict: `REPLICATED`.** All three preregistered capture contrasts met the directional replication
criterion in the independent seven-seed bias-corrected replication. The Holm-adjusted p-values were
0.046875, at the resolution boundary of the seven-seed exact sign-flip design. Effect sizes,
uncertainty intervals and seed-level consistency are reported alongside the hypothesis tests.

- **Policy and scope:** frozen policy F (ep25000 + riskcap), in simulation only.
- **Record:**
  - Preregistration: [`docs/prereg_2026-09-24_target_motion_replication_7seed.md`](../../docs/prereg_2026-09-24_target_motion_replication_7seed.md).
  - Analysis: `analysis.json`, from `tools/target_motion_replication.py analyze` at measurement commit
    `b1c0bc6`.
  - Raw freeze: `RAW_MANIFEST.json`, committed in `2f723ed` before any outcome was read.

## Design as run

| Item | Value |
| --- | --- |
| Arms | H (historical training law), E0 static, E1 constant velocity, E2 obstacle-aware waypoint |
| Densities | 70 / 115 / 160 / 205 bars |
| Fresh evaluation seeds | 4104, 4105, 4106, 4107, 4108, 4109, 4110 (prior use 0) |
| Cells | 4 × 4 × 7 = 112, all valid |
| Primary episodes | exactly 2,048 per cell: the first 16 completed episodes of each of 128 environments. 229,376 in total |
| Raw ledger | 397,619 rows; the 168,243 beyond the quota are `NONPRIMARY_TAIL`, kept as raw evidence and never used for the primary estimate |
| Estimand | per-environment quota (bias-corrected). The 2026-09-18 campaign used the pooled legacy stopping window |
| Unit of inference | evaluation seed (n = 7); per seed, the unweighted mean of the four density cells |
| Run | RTX 3070, 2026-09-25 14:48 → 2026-09-26 03:34 UTC (12 h 45 min); measurement commit `b1c0bc6`; checkpoint `f7022139…3ad40` |
| Canary | the historical cell H / 70 / 4101 passed all 11 non-interference gates ([`canary_provenance.json`](canary_provenance.json)); it is not evidence |

## Outcomes by arm (primary quota, mean over seeds)

| Arm | Capture | Crash | Timeout |
| --- | ---: | ---: | ---: |
| H | 87.76 % | 8.94 % | 3.29 % |
| E0 | 82.39 % | 8.68 % | 8.93 % |
| E1 | 84.32 % | 10.22 % | 5.47 % |
| E2 | 89.12 % | 9.37 % | 1.52 % |

## Confirmatory contrasts (capture, percentage points)

| Contrast | Seed effects 4104 … 4110 | Mean | BCa 95 % | Exact p | Holm p | Same direction | Classification |
| --- | --- | ---: | --- | ---: | ---: | --- | --- |
| E0 − H | −5.57 −5.86 −5.31 −4.27 −5.64 −6.21 −4.72 | **−5.37** | [−5.78, −4.85] | 0.015625 | 0.046875 | 7/7 | successful directional statistical replication |
| E1 − H | −3.12 −3.23 −3.63 −3.54 −3.66 −4.22 −2.71 | **−3.45** | [−3.79, −3.11] | 0.015625 | 0.046875 | 7/7 | successful directional statistical replication |
| E2 − H | +1.22 +1.27 +2.42 +1.17 +0.93 +0.79 +1.68 | **+1.35** | [+1.07, +1.87] | 0.015625 | 0.046875 | 7/7 | successful directional statistical replication |

- Seed sign consistency is reported descriptively. It is not a pass/fail criterion.
- The ordering E0 < E1 < H < E2 of mean capture holds (descriptive).
- **Density limitation:** the per-density contrasts are exploratory and uncorrected. E2 − H kept its
  direction in 7/7 seeds at 70 and 115 bars, but in only **5/7 at 160 bars** and **4/7 at 205 bars**.
  The aggregate E2 − H contrast replicated, but its positive direction was not uniformly expressed at
  high obstacle densities.

## Legacy window versus quota (same seven seeds)

| Contrast | Legacy window | Quota (primary) | Quota − legacy |
| --- | ---: | ---: | ---: |
| E0 − H | −4.36 | −5.37 | −1.01 pp |
| E1 − H | −3.12 | −3.45 | −0.33 pp |
| E2 − H | +1.09 | +1.35 | +0.27 pp |

| Arm | Capture, quota − legacy | Crash, quota − legacy | Timeout, quota − legacy |
| --- | ---: | ---: | ---: |
| H | −0.50 | −0.18 | +0.69 |
| E0 | −1.51 | −0.09 | +1.60 |
| E1 | −0.83 | −0.16 | +0.98 |
| E2 | −0.23 | −0.14 | +0.37 |

The historical pooled stopping window attenuated all three capture contrasts toward zero in the
seven-seed replication. It overstates capture and understates timeouts, most in the arm with the
most timeouts (E0). The legacy window is a diagnostic here and plays no part in the verdict.

## Original campaign, side by side (not pooled)

| Contrast | Original 2026-09-18: legacy stopping window, n = 3 | Replication 2026-09-25: bias-corrected quota, n = 7 |
| --- | --- | --- |
| E0 − H | −3.58 [−4.04, −3.29] | −5.37 [−5.78, −4.85] |
| E1 − H | −2.82 [−3.53, −2.44] | −3.45 [−3.79, −3.11] |
| E2 − H | +1.33 [+1.15, +1.50] | +1.35 [+1.07, +1.87] |

The estimator change explains a substantial component of the larger replicated E0 loss, while
seed-set variation cannot be separated from the cross-campaign difference: the two campaigns use
different seeds. The original campaign stays closed and unchanged in
[`../target_motion_e0_e2_2026-09-18/`](../target_motion_e0_e2_2026-09-18/). No combined confirmatory p-value is computed.

## Minimum relative distance: recorded, not published

`min_relative_distance_m` is recorded for every episode, crashes included. It failed the semantic
check in [`min_distance_semantic_check.json`](min_distance_semantic_check.json):
- 306 of 197,026 primary capture episodes have a recorded minimum above the 0.5 m capture radius;
- the maximum is 0.5294 m.

The cause is a definition mismatch, not a data error. Capture uses a swept-segment closest-approach
test within each 0.1 s step, while the metric samples the point distance only at the end of each step,
as preregistered. The metric is therefore an end-of-step upper bound on the closest approach. Its values
remain in `analysis.json` as the preregistered analysis produced them, but they are not published as a
distance result.

## What this does not show

- It is not a significance claim at every density, and not a claim that obstacle-aware targets are
  universally easier.
- It does not show that E0/E1 degradation calls for retraining: no deployment-performance threshold
  was preregistered. This is a frozen-policy generalization measurement. `PPO_TRAINING_STARTED = false`.
- The replication is closed at seeds 4104–4110. Any further replication is a new preregistered study.

## Files

| File | In Git | Content |
| --- | --- | --- |
| `RAW_MANIFEST.json`, `integrity_report.json`, `preflight_main.json` | yes (freeze commit `2f723ed`) | SHA-256 of 562 raw files; integrity counts; run environment |
| `primary_selection_manifest.json` | yes | the exact 2,048 primary episode IDs per cell, and the tail IDs |
| `analysis.json` | yes | the preregistered analysis output |
| `cells.csv` | yes | compact per-cell counts (primary and legacy), raw row counts, row-file SHA-256 |
| `canary_provenance.json`, `canary/canary_report.json` | yes | canary gates and digests |
| `min_distance_semantic_check.json`, `replication_record.json` | yes | metric check; archive SHA-256 and sizes |
| 112 cell directories, `main_run.log`, canary run folders | no | raw evidence, read-only locally; archived deterministically (`replication_record.json`) |

`tools/build_tm_replication_record.py --check` re-derives the compact files from the raw cells when
they are present. `tests/test_target_motion_replication_result.py` binds every number above to these
files.
