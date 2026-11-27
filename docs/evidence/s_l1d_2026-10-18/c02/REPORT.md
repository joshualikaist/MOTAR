# SL1D-C02: GPU evidence for undetected partial, centre-only and scale-loss states — 2026-10-18

This canary runs under [S-L1D amendment 1](../../../plans/s_l1d_v1_amendment1.md). It adds GPU evidence for
three classes that SL1D-C01 lacked: undetected PARTIAL, undetected CENTRE_ONLY and undetected SCALE_LOSS.
The original [SL1D-C01 report](../REPORT.md) and its [review addendum](../REVIEW_ADDENDUM.md) are not
edited.

**C02 analysis result: all 8 C02 gates PASS.** The S-L1D GO/NO-GO decision is recorded only after the
independent review, in [`REVIEW.md`](REVIEW.md).

Numbers come from [`s_l1d_c02_analysis.json`](s_l1d_c02_analysis.json). Re-running the analysis on the raw
output reproduces that file byte for byte, and the analysis imports no production code.

## Run

| Item | Value |
| --- | --- |
| Source commit (fixed during the run) | `7d0bb37358c0e4a26fe6581cf9ff3f6db30254ed` |
| Production code | `aerial_gym/` and `motar_eval/` identical to the qualified S-L1D source `76f0f4c` (preflight and `git diff`) |
| Processes | 2 of 2 COMPLETED (`sensors_s_l1c`, `sensors_s_l1d`), 10.3 s each, 20.7 s in total, no retry |
| Peak GPU memory | 4,042 MiB, under launch caps of 7,015–7,020 MiB |
| Disk | 19.0 GB free after the run; output 0.37 MB |
| Raw | `../aerial_gym_simulator/results/s_l1d_v1/SL1D-C02`, outside Git. Manifest SHA-256 `b131bc25…`, 23 members, 0 mismatches |
| Policy F | `f7022139…` before and after |

## Per-scene results

Each row is the **level scene pass**, which is the only independent case per scene. The roll15 and
pitch15 passes are identical yaw-only inputs; they were verified bitwise identical (C02-UNIQUE) and are
not counted. Every scene ran and is reported here; none was excluded.

- **Primed belief:** the true centre in the body frame, with covariance 1e−6·I. This is fixture
  input only, so τ = 0.052 m.
- **Bar:** the bar centre x and its edge, in m.
- **Depth min / surface:** the minimum delivered GPU depth over the predicted support, against the
  predicted sphere surface.
- **Gap:** the occluder-to-target gap.
- **Oracle online:** the independent float64 class recomputed on the delivered GPU depth.

| Scene | Primed belief (m) | Bar | E-L0 GPU / oracle class | alone_px E-L0 / oracle | Rendered visible px | Camera detected | Depth min / surface (m) | Gap (m) | Online class | Oracle online | Agrees |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| C02_SC0_1 | (19.800, 0.000, 0.030) | — | SCALE_LOSS / SCALE_LOSS | 0 / 0 | 0 | no | — | — | PRED_SCALE_LOSS | PRED_SCALE_LOSS | yes |
| C02_SC0_2 | (20.000, 0.000, 0.030) | — | SCALE_LOSS / SCALE_LOSS | 0 / 0 | 0 | no | — | — | PRED_SCALE_LOSS | PRED_SCALE_LOSS | yes |
| C02_SC1_1 | (15.000, -0.090, -0.070) | — | VISIBLE / VISIBLE | 1 / 1 | 1 | no | 14.751 / 14.751 | — | PRED_SCALE_LOSS | PRED_SCALE_LOSS | yes |
| C02_SC1_2 | (18.000, -0.100, -0.070) | — | VISIBLE / VISIBLE | 1 / 1 | 1 | no | 17.751 / 17.751 | — | PRED_SCALE_LOSS | PRED_SCALE_LOSS | yes |
| C02_SC1_FULL | (18.000, -0.100, -0.070) | x 9.00 y 0.000 | FULL / FULL | 1 / 1 | 0 | no | 8.589 / 17.751 | 9.16 | PRED_SCALE_LOSS | PRED_SCALE_LOSS | yes |
| C02_PA_1 | (12.000, -0.020, -0.045) | x 6.00 left edge 0.0125 | PARTIAL / PARTIAL | 2 / 2 | 1 | no | 5.669 / 11.759 | 6.11 | OCCLUSION_PARTIAL_EVIDENCE | OCCLUSION_PARTIAL_EVIDENCE | yes |
| C02_PA_2 | (12.000, 0.005, -0.050) | x 6.00 right edge -0.0163 | PARTIAL / PARTIAL | 2 / 2 | 1 | no | 5.626 / 11.765 | 6.14 | OCCLUSION_PARTIAL_EVIDENCE | OCCLUSION_PARTIAL_EVIDENCE | yes |
| C02_PA_3 | (13.000, -0.060, -0.055) | x 6.50 left edge 0.0041 | PARTIAL / PARTIAL | 2 / 2 | 1 | no | 6.108 / 12.751 | 6.73 | OCCLUSION_PARTIAL_EVIDENCE | OCCLUSION_PARTIAL_EVIDENCE | yes |
| C02_PA_4 | (13.000, 0.005, -0.060) | x 6.50 right edge -0.0177 | PARTIAL / PARTIAL | 2 / 2 | 1 | no | 6.073 / 12.769 | 6.70 | OCCLUSION_PARTIAL_EVIDENCE | OCCLUSION_PARTIAL_EVIDENCE | yes |
| C02_CO_1 | (12.000, -0.020, -0.045) | x 6.00 right edge 0.0125 | CENTRE_ONLY / CENTRE_ONLY | 2 / 2 | 1 | no | 5.635 / 11.759 | 6.12 | OCCLUSION_CENTRE_EVIDENCE | OCCLUSION_CENTRE_EVIDENCE | yes |
| C02_CO_2 | (12.000, 0.005, -0.050) | x 6.00 left edge -0.0163 | CENTRE_ONLY / CENTRE_ONLY | 2 / 2 | 1 | no | 5.655 / 11.765 | 6.11 | OCCLUSION_CENTRE_EVIDENCE | OCCLUSION_CENTRE_EVIDENCE | yes |
| C02_CO_3 | (13.000, -0.060, -0.055) | x 6.50 right edge 0.0041 | CENTRE_ONLY / CENTRE_ONLY | 2 / 2 | 1 | no | 6.059 / 12.751 | 6.69 | OCCLUSION_CENTRE_EVIDENCE | OCCLUSION_CENTRE_EVIDENCE | yes |
| C02_CO_4 | (13.000, 0.005, -0.060) | x 6.50 left edge -0.0177 | CENTRE_ONLY / CENTRE_ONLY | 2 / 2 | 1 | no | 6.048 / 12.769 | 6.72 | OCCLUSION_CENTRE_EVIDENCE | OCCLUSION_CENTRE_EVIDENCE | yes |

## Realised states (unique scenes)

| State | Unique undetected cases | Distinct geometries | Online result |
| --- | --- | --- | --- |
| SCALE_LOSS, alone 0 | C02_SC0_1, C02_SC0_2 | 2 (19.8 m, 20.0 m) | PRED_SCALE_LOSS, matches oracle |
| Scale, alone 1, clear (evaluator VISIBLE 1 px) | C02_SC1_1, C02_SC1_2 | 2 (15 m, 18 m) | PRED_SCALE_LOSS |
| Scale, alone 1, occluded (evaluator FULL) | C02_SC1_FULL | 1 | PRED_SCALE_LOSS (scale takes precedence) |
| PARTIAL (alone 2 / visible 1, centre ray open, centre-robust) | C02_PA_1–4 | 4 (12 m and 13 m, both sides) | OCCLUSION_PARTIAL_EVIDENCE, matches oracle |
| CENTRE_ONLY (alone 2 / visible 1, centre ray blocked, centre-robust) | C02_CO_1–4 | 4 | OCCLUSION_CENTRE_EVIDENCE, matches oracle |

In the target-alone passes, the 2-pixel targets were detected (DETECTED), and the 0/1-pixel targets were
not (PRED_SCALE_LOSS). No alone pass produced occlusion evidence.

## Gates

| Gate | Result | Evidence |
| --- | --- | --- |
| C02-UNIQUE | PASS | Attitude passes are bitwise identical for every scene; 13 unique cases |
| C02-ORACLE | PASS | Production support equals E-L0 `alone_px` and the oracle (0 ambiguous pixels). The online class equals the oracle on the GPU depth in 13/13 |
| C02-MAPPING | PASS | 13/13 undetected cases meet the v1 mapping plus the explicit scale precedence; 0 violations; no alone-pass occlusion |
| C02-SCALE | PASS | 5 cases |
| C02-PARTIAL | PASS | 4 cases |
| C02-CENTRE | PASS | 4 cases |
| C02-SENSOR-NONINTERFERENCE | PASS | `sensors_s_l1c` and `sensors_s_l1d` renders (105 keys, NaN-aware) and detection flags are identical |
| C02-INTEGRITY | PASS | COMPLETED; lineage flags (s_l1c: c without d; s_l1d: c and d); production identical to `76f0f4c` |

## Scope

- **What C02 completes.** C02 provides the GPU evidence that SL1D-C01 lacked.
- **What the earlier review added.** It found that the committed CPU tests at `aab7379` did not assert
  scale loss or three preregistered CPU cases. That stays a separate review record, completed on CPU in
  `bd65d55`.
- **What C02 covers.** It adds real-render confirmation for the undetected classes. Fixture counts are
  constructed and are not population rates. The canary is sensor-only: closed-loop and lower-lineage
  regressions are covered by SL1D-C01 on the identical production source.
- **What it does not claim.** It does not claim real-robot sensor accuracy or Recovery performance.
