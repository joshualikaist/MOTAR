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
| **Target-motion generalization** | Independent bias-corrected replication, 7 fresh seeds, 229,376 primary episodes: H 87.76 %; E0 **−5.37 pp** [−5.78, −4.85]; E1 **−3.45 pp** [−3.79, −3.11]; E2 **+1.35 pp** [+1.07, +1.87]; all three met the directional replication criterion (Holm p 0.046875 each). Original 3-seed campaign −3.58 / −2.82 / +1.33 pp kept as historical | Elastic Tracker | Cooperative target, position broadcast, max 2 m/s; escaping targets named as **future work** | NO DIRECT MATCH | The only measured statement that one frozen policy (F) was tested across a **ladder of target-motion contracts**, including an arm it was not trained on | n = 7 evaluation seeds (the unit of inference); Holm p at the resolution boundary; policy F, one arena family; E2 − H not uniform at high density (5/7 at 160, 4/7 at 205 bars) |
| **Perception uncertainty** | E3-S **6.2 %** median absolute relative range error: median of 9 block medians from one flight (3,107 frames, not independent), 30.7–108.4 m | NavRL++ | Perception failure the largest perturbation factor, "> 5 % drop" under its own injected model | NO DIRECT MATCH | The error is **measured on real imagery**, not a chosen noise level | One flight; apparent-size proxy; E3-P `ATTITUDE_NOT_RELIABLE`, decomposition BLOCKED |
| **Perception → policy propagation** | **−4.57 pp** capture cost on frozen policy F, episode-level interval (two evaluation seeds) excludes zero; three deterministic re-runs gave identical counts and are not replications | NavRL++ | Same > 5 % figure, but from perturbation rather than a measured distribution | NO DIRECT MATCH | Closed loop: measure → inject → evaluate → retrain, with the retraining answer reported as inconclusive | One density, one arena, one injector |
| **Temporal association** | Transformer **+0.00697** validation utility over CNN-only; learned detector non-inferior at **−0.015 pp** [−1.752, +1.723] | Fast-Tracker / YOPOv2-Tracker | Prediction error 1.82 / 2.54 / 3.45 m under three noise levels (Fast-Tracker); EKF + spatiotemporal consistency (YOPOv2) | NO DIRECT MATCH | Selector evaluated **separately from the detector**, with the held-out warning retained | No persistent-identity evidence; no prediction-error metric comparable to Fast-Tracker's |
| **Safety-filter geometry** | riskcap vs arc **−1.4903 pp** crash, cell-pooled [−1.8981, −1.0826], seed-level [−2.42, −0.57] (3 evaluation seeds), lower in 15/15 seed × density cells; arc width **−5.60 pp** crash / **+4.44 pp** capture at 205 bars | Temporal Barrier | 0.170 / 0.034 / 0.012 collisions per 100 s (no CBF / HOCBF / aTTC-CBF), **obstacle-free** domain, full relative state | NO DIRECT MATCH | Matched configured geometries in **dense clutter**, with the opposite sign on a sibling geometry reported | No formal guarantee; C3 explanation WITHDRAWN; route gates `FAIL_ROUTE_MECHANISM` |
| **Renderer / observation sensitivity** | D8b **−48.967 pp** [−50.113, −47.821] on frozen policy R (3 paired evaluation seeds), `MATERIAL_LOSS` | **None found** | No screened work reports frozen-policy outcome under a changed observation-rendering treatment | NO DIRECT MATCH | The largest loss measured in the programme. It was measured on policy R, not the policy F of the perception-error cost, so the two magnitudes are not directly comparable | **`causality_vs_d8b = NOT_TESTED`** — no renderer result shows what caused it |
| **Retraining robustness** | P10 **+0.73 pp** [−1.04, +2.50] `INCONCLUSIVE`; residual cost −1.90 / −2.71 / −3.84 pp; target-motion verdict `NO_RETRAINING_JUSTIFIED_FOR_E2_TARGET_MOTION_SHIFT` | NavRL++ | Curriculum + perturbation training raise combined success 63.05 % → 94.08 % inside its own framework | NO DIRECT MATCH | Two independent retraining questions answered, one inconclusive and one scoped to a named shift — neither promoted | Three training seeds; the verdict covers target motion only |
| **Reproducibility / provenance** | **208** result directories indexed with provenance links and hashes (`results/MANIFEST.json`, build of 2026-09-26); public snapshot `CLEAN`, 0 denylist matches; raw immutability 144/144 | **None found** | No screened work publishes per-result receipts, negative-result retention or a comparable machine-readable manifest | NO DIRECT MATCH | The failure record is part of the artifact: a withdrawn threshold, a `NOT_RECORDED` primary and a FAILED gate all remain visible | Hosted CI unverified; several datasets cannot be redistributed |

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
  -> policy-F cost            -4.57 pp, episode-level interval excludes zero (2 evaluation seeds)
  -> readaptation             +0.73 pp, CI [-1.04, +2.50], INCONCLUSIVE
  -> safety-filter geometry   -1.4903 pp crash; seed-level CI [-2.42, -0.57], 3 evaluation seeds
  -> target-motion shift      -5.37 pp (static) .. +1.35 pp (obstacle-aware) vs H, replicated (n = 7)
  -> observation rendering    -48.967 pp on policy R, MATERIAL_LOSS, causality NOT_TESTED
```
