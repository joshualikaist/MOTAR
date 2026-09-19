# Paper claim / evidence matrix — 2026-09-19

Supersedes [`paper_claim_evidence_matrix_2026-09-18.md`](paper_claim_evidence_matrix_2026-09-18.md),
which was written while the target-motion evaluation was still `RESULT_PENDING`.

`Benchmark parity` is **NO DIRECT MATCH** in every row. That is why the MOTAR column and the
external column are never combined: no external value is subtracted from a MOTAR value anywhere in
this repository, and Class A matched comparisons remain **0**.

MOTAR values are source-bound in
[`quantitative_positioning_registry.json`](quantitative_positioning_registry.json); external values
are recorded under each publication's own benchmark in
[the ledger](literature_quantitative_ledger_2026-09-18.json).

| Research axis | MOTAR numerical evidence | Closest published work | External reported evidence (own benchmark) | Benchmark parity | MOTAR's defensible distinction | Missing evidence |
|---|---|---|---|---|---|---|
| **Moving-target tracking** | Close-approach outcomes per cell with receipts; `P2 held-out` **STRICT FAIL** (timeout 5.56 %) | YOPOv2-Tracker | 6 m/s max real forest tracking; 8.2 ms perception-to-action. Success rate vs target speed `NOT_EXTRACTED` (plot only) | NO DIRECT MATCH | Sensor-only pursuit of a **pursuer-independent** target, with the failing held-out gate retained | No external tracker run in MOTAR's contract; YOPOv2-Tracker's code is unreleased |
| **Obstacle density** | Sweeps at 70/115/160/205 bars; arm gap widest at 70, attenuating by 205 (**EXPLORATORY**) | NavRL | Success 94.33 % → 68.65 % as dynamic obstacles rise 60 → 120, max 2.0 m/s | NO DIRECT MATCH | Density swept as an independent variable for a **pursuit** task, not goal navigation | "Bars" and "obstacles" are not the same unit; no shared density definition |
| **Target-motion generalization** | H 87.89 %; E0 **−3.58 pp** [−4.04, −3.29]; E1 **−2.82 pp**; E2 **+1.33 pp** [+1.15, +1.50]; E2−E0 **+4.91 pp**; 3/3 seed sign-consistency | Elastic Tracker | Cooperative target, position broadcast, max 2 m/s; escaping targets named as **future work** | NO DIRECT MATCH | The only measured statement that one frozen policy was tested across a **ladder of target-motion contracts**, including an arm it was not trained on | n = 3 seeds; one checkpoint, one arena family |
| **Perception uncertainty** | E3-S **6.2 %** median absolute relative range error, 3,107 frames, 9 blocks, 30.7–108.4 m | NavRL++ | Perception failure the largest perturbation factor, "> 5 % drop" under its own injected model | NO DIRECT MATCH | The error is **measured on real imagery**, not a chosen noise level | One flight; apparent-size proxy; E3-P `ATTITUDE_NOT_RELIABLE`, decomposition BLOCKED |
| **Perception → policy propagation** | **−4.57 pp** capture cost on the frozen policy, interval excludes zero, identical across 3 training-seed campaigns | NavRL++ | Same > 5 % figure, but from perturbation rather than a measured distribution | NO DIRECT MATCH | Closed loop: measure → inject → evaluate → retrain, with the retraining answer reported as inconclusive | One density, one arena, one injector |
| **Temporal association** | Transformer **+0.00697** validation utility over CNN-only; learned detector non-inferior at **−0.015 pp** [−1.752, +1.723] | Fast-Tracker / YOPOv2-Tracker | Prediction error 1.82 / 2.54 / 3.45 m under three noise levels (Fast-Tracker); EKF + spatiotemporal consistency (YOPOv2) | NO DIRECT MATCH | Selector evaluated **separately from the detector**, with the held-out warning retained | No persistent-identity evidence; no prediction-error metric comparable to Fast-Tracker's |
| **Safety-filter geometry** | riskcap vs arc **−1.4903 pp** crash [−1.8981, −1.0826], 15/15 cells; arc width **−5.60 pp** crash / **+4.44 pp** capture at 205 bars | Temporal Barrier | 0.170 / 0.034 / 0.012 collisions per 100 s (no CBF / HOCBF / aTTC-CBF), **obstacle-free** domain, full relative state | NO DIRECT MATCH | Matched configured geometries in **dense clutter**, with the opposite sign on a sibling geometry reported | No formal guarantee; C3 explanation WITHDRAWN; route gates `FAIL_ROUTE_MECHANISM` |
| **Renderer / observation sensitivity** | D8b **−48.967 pp** [−50.113, −47.821], `MATERIAL_LOSS` | **None found** | No screened work reports frozen-policy outcome under a changed observation-rendering treatment | NO DIRECT MATCH | A magnitude for observation-contract sensitivity an order larger than the perception-error cost | **`causality_vs_d8b = NOT_TESTED`** — no renderer result shows what caused it |
| **Retraining robustness** | P10 **+0.73 pp** [−1.04, +2.50] `INCONCLUSIVE`; residual cost −1.90 / −2.71 / −3.84 pp; target-motion verdict `NO_RETRAINING_JUSTIFIED_FOR_E2_TARGET_MOTION_SHIFT` | NavRL++ | Curriculum + perturbation training raise combined success 63.05 % → 94.08 % inside its own framework | NO DIRECT MATCH | Two independent retraining questions answered, one inconclusive and one scoped to a named shift — neither promoted | Three training seeds; the verdict covers target motion only |
| **Reproducibility / provenance** | **204** indexed results with receipts; public snapshot `CLEAN`, 0 denylist matches; raw immutability 144/144 | **None found** | No screened work publishes per-result receipts, negative-result retention or a comparable machine-readable manifest | NO DIRECT MATCH | The failure record is part of the artifact: a withdrawn threshold, a `NOT_RECORDED` primary and a FAILED gate all remain visible | Hosted CI unverified; several datasets cannot be redistributed |

## What changed since the 09-18 matrix

The target-motion row is no longer pending. It now carries the finalized effects, and it gained a
second line of evidence from the acquisition audit: in **95.42 %** of E0 timeout episodes the target
was never acquired even once, against 83–84 % in the moving arms. That moves the mechanism reading
from "lost and not regained" toward "never found in the first place", and it was obtained **without
generating new data**.

## The distinction, stated without overclaiming

MOTAR does not claim a cross-paper performance advantage, and this repository contains no evidence
that would support one. Its contribution is that the stages are connected by measurement rather than
assumption, and that the chain retains its negative and inconclusive links:

```text
measured perception error     6.2 % median absolute relative range error
  -> injected into simulation P8 distribution, P9 injector
  -> frozen-policy cost       -4.57 pp, interval excludes zero, 3/3 campaigns
  -> readaptation             +0.73 pp, CI [-1.04, +2.50], INCONCLUSIVE
  -> safety-filter geometry   -1.4903 pp crash, 15/15 cells
  -> target-motion shift      -3.58 pp (static) .. +1.33 pp (obstacle-aware) vs H
  -> observation rendering    -48.967 pp, MATERIAL_LOSS, causality NOT_TESTED
```
