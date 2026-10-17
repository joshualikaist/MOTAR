# S-L1C GPU qualification SL1C-C01 — 2026-10-18

**Verdict: `S-L1C_QUALIFIED`.** All nine verdict gates of the [preregistration](../../plans/s_l1c_v1.md)
pass: C-NEG, C-POS, C-COV, C-PROV, C-RADIUS, C-REGRESSION, C-EL0, C-FIREWALL and C-INTEGRITY.
C-RESOURCE is a characterisation only.

`NAVRL_SENSOR_LINEAGE=s_l1c_v1` is cumulative over S-L1A and S-L1B. A LiDAR return now updates the
target track only as an identity-free, range-only measurement. The return must:

- lie on a row that can meet the 0.20 m proxy around the predicted centre;
- not belong to a vertically extended surface;
- be at a conclusive range (≥ 1.152 m).

When accepted, the update:

- uses the proxy radius for the centre range;
- constrains only the measured ray (R = 0.08²·uuᵀ + 50²·(I − uuᵀ));
- leaves the evaluator-only identity clock unreset.

LiDAR target measurement is not disabled. Numbers come from
[`s_l1c_analysis.json`](s_l1c_analysis.json). Re-running the analysis on the raw output reproduces
that file byte for byte, and the analysis imports no production code.

## Run

| Item | Value |
| --- | --- |
| Source commit (fixed during the run) | `4d5911d3d2fcbf0a285f395e38428dc39ef67868` |
| Processes | 13 of 13 COMPLETED, every return code 0, no retry |
| Wall time | 541.0 s in total: sensors 10.2–10.3 s each, closed loop 46.9–52.0 s each |
| Peak GPU memory | 4,668 MiB, under launch caps of 7,278–7,283 MiB |
| Disk | 19.1 GB free after the run; output 23.98 MB |
| Raw | `../aerial_gym_simulator/results/s_l1c_v1/SL1C-C01`, outside Git. Manifest SHA-256 `29448df8…`, 176 members, 0 mismatches |
| Policy F | `f7022139…` before and after |

## Identity: false association and recall (C-NEG, C-POS)

The scene passes are identical yaw-only inputs under three attitudes, so the unique cases are a
third of the totals.

| Fixture set | legacy | S-L1B | **S-L1C** |
| --- | --- | --- | --- |
| EQ-1D D2-N2 bar-only negatives (7 scenes × 3 passes) | 21/21 accepted | 21/21 accepted | **0/21 accepted** |
| Eligible positives (13 target scenes × 3 offsets × 3 passes = 117) | 114/117 | 114/117 | **117/117** |
| Target scenes ineligible by LiDAR truth (99 trials) | 18 accepted | 18 accepted | **0 accepted** |

- **Negatives.** Production S-L1C decisions equal the independent float64 oracle on every negative
  (0 mismatches). The oracle identifies all seven as vertically extended returns.
- **Positive eligibility.** This comes from analytic LiDAR truth only: some row within the proxy's
  angular radius hits the proxy in the bearing-nearest column, and the range is conclusive. It is
  never decided by the S-L1C rule.
- **Legacy recall misses.** All three are S05 (12 m) primed +0.15 m, where the legacy predicted
  surface reaches the 12 m limit.
- **Legacy false accepts among ineligible trials.** These are S14 and A_FOREGROUND: a foreground
  bar accepted as the target.

## Range-only covariance and radius (C-COV, C-RADIUS)

| Probe (prior position covariance I) | legacy / S-L1B | **S-L1C** |
| --- | --- | --- |
| Accepted updates | 45 | 39 |
| Minimum posterior/prior variance orthogonal to the ray | 0.0128 | **0.9996** |
| Maximum posterior/prior variance along the ray | 0.0155 | 0.0066 |

The legacy diagonal update hands the filter tangential and vertical information it never measured,
which reproduces EQ-1C T4 on rendered scans. S-L1C keeps that information out and still uses the
range.

On the 33 beam-centred positive trials, the S-L1C centre range differs from the true centre by at
most **2.5e−5 m**. That fixes the A4 −0.050 m bias of the legacy association, which adds the
camera's 0.15 m to the LiDAR's 0.20 m proxy return.

## Provenance and identity clock (C-PROV)

These are evaluator-only closed-loop traces from the N8, 600-step runs:

| Config | Camera updates | LiDAR range updates | No update |
| --- | --- | --- | --- |
| static, s_l1c (P1 = P2) | 3,886 | 421 | 501 |
| moving, s_l1c (P1 = P2) | 3,551 | 362 | 895 |

The identity clock behaves as preregistered in all four s_l1c runs:

- every camera update resets `identity_age` to 0;
- no LiDAR range update resets it, and it advances by exactly one step (maximum error 0);
- every LiDAR accept still resets the fused-recency `track_age`, so the 5 s expiry contract is
  unchanged.

LiDAR accepts per run:

| Config | off | S-L1A | S-L1B | S-L1C |
| --- | --- | --- | --- | --- |
| static | 453 | 564 | 529 | 421 |
| moving | 360 | 490 | 516 | 362 |

## Regression and E-L0 (C-REGRESSION, C-EL0)

Each comparison below is bitwise identical over 14 digest keys and 600/600 steps, at static 4501
and moving 4502:

| Comparison | Result |
| --- | --- |
| off vs EL0-R1 P2 | identical |
| s_l1a vs SL1A-C01 on_p2 | identical |
| s_l1b vs SL1B-C01 s_l1b_p2 | identical |
| s_l1c E-L0 P1 vs P2 | identical |

On CPU, the off, s_l1a and s_l1b association and `observe` are bitwise equal to `a1f97e9`. The
actor input is 898-D in all ten closed-loop processes.

## ON behaviour change (distribution only, no claim)

| Config | Captures (off / S-L1B / S-L1C) | Crashes (off / S-L1B / S-L1C) |
| --- | --- | --- |
| static | 123 / 120 / 116 | 1 / 2 / 5 |
| moving | 93 / 89 / 96 | 9 / 3 / 4 |

These are single-seed N=8 runs. Frozen F under a changed input distribution measures robustness,
not improvement.

## Integrity and preservation

[`preservation_after.json`](preservation_after.json) records PASS:

- runtime/test files: 2,078, of which exactly 27 S-L1A/B/C paths are allowed to change;
- historical evidence files: 101;
- raw manifests: 10, with 3,479 members;
- Policy F: both copies unchanged;
- main checkout: `cb644bb`, its three WIP files and stash `23ddb33` unchanged.

The SL1A-C01 and SL1B-C01 manifests still verify (138/138 each).

## Limits and Known FAILs

- **Shape test.** It assumes vertical-faced obstacles. A 3 m bar whose front face is beyond about
  11.35 m returns one row and remains indistinguishable from the target. This is a declared
  residual band; it was demonstrated on CPU and not sampled on GPU.
- **Short range.** LiDAR association is refused below 1.152 m, which costs recall there.
- **Inherited from S-L1B.** The carve retains about 82% of target-centre returns as obstacles.
- **Still open:**
  - camera centre-range A7 bias;
  - silhouette and measurement validity (S-L1D, not started);
  - target renderer residual;
  - dynamics.
- **Population rates.** Fixture counts are constructed. The population false-association rate on
  the full E-L0 population is NOT_TESTED.
