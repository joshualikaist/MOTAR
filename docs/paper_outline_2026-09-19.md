# Paper outline — 2026-09-19

Supersedes [`paper_evidence_outline_2026-09-18.md`](paper_evidence_outline_2026-09-18.md), which
predates the completed target-motion result and the acquisition audit.

Evidence per section is in [the evidence spine](paper_evidence_spine_2026-09-19.md); the
axis-by-axis external pairing is in
[the claim/evidence matrix](paper_claim_evidence_matrix_2026-09-19.md).

## Thesis

Published UAV work isolates navigation, tracking, pursuit, perception or safety and reports a number
inside one of them. MOTAR measures the **chain that connects them** on a single frozen policy, and
keeps the links that failed. The strongest single new result is that target behaviour did not form a
monotonic difficulty ladder: the static arm was the hardest for the frozen policy, not the easiest.

## Structure

### 1. Introduction

**Q1 — What do prior studies usually isolate?**
Each screened system fixes one axis and reports inside it: NavRL and YOPO obstacle navigation
(NavRL: 94.33 % → 68.65 % success as dynamic obstacles rise 60 → 120); OPEN multi-UAV pursuit with
privileged relative state (1.000 capture in its unseen scenarios, no camera anywhere in the paper);
Fast-Tracker and Elastic Tracker single-UAV tracking of a **cooperative** target; NavRL++ perception
perturbation (> 5 % success drop) under its own failure model. Twelve works, ten site rows, and no
two of them share a contract.

**Q2 — What does MOTAR measure jointly that is usually separated?**
Measured perception error → its cost to a policy → whether retraining recovers it → the safety
geometry that executes the policy's output → the target-motion distribution the policy is evaluated
against → the observation contract that renders all of it. One frozen checkpoint, one arena family,
one set of receipts.

**Q3 — What numerical evidence supports the distinction?**
Measured range error **6.2 %** (3,107 frames) → frozen-policy cost **−4.57 pp** (interval excludes
zero, 3/3 campaigns) → readaptation **+0.73 pp**, CI [−1.04, +2.50], `INCONCLUSIVE` → safety
geometry **−1.4903 pp** crash, CI [−1.8981, −1.0826], 15/15 cells → target-motion shift **−3.58 pp**
(static) to **+1.33 pp** (obstacle-aware) against the historical reference → observation treatment
**−48.967 pp**, `MATERIAL_LOSS`, causality `NOT_TESTED`.

### 2. Related Work
2.1 Target tracking — Fast-Tracker, Elastic Tracker, YOPOv2-Tracker; all evaluate cooperative or
marker-based targets, and Elastic Tracker names escaping targets as future work.
2.2 Obstacle-aware navigation — YOPO, PILOT, FlowPilot, MAD.
2.3 RL navigation — NavRL, NavRL++, OPEN, AgilePE.
2.4 Perception uncertainty — NavRL++ perturbation analysis; no screened work injects a *measured*
error distribution.
2.5 Why the combined evidence gap remains — and why Class A comparisons are **0**, with the
benchmark incompatibilities named per system rather than asserted.

### 3. MOTAR System and Experimental Contract
Arena, sensor-only observation contract (898-dim actor / 906-dim critic), control stack, and the
declared boundary. Includes the honest note that 14 of 112 config values are env-var opt-in, so the
training contract is **not reproducible from HEAD defaults**.

### 4. Perception and Temporal Estimation — C1, C3
### 5. Perception-to-Policy Error Propagation — C2
### 6. Obstacle-Aware Navigation and Safety Filtering — C4
### 7. Target-Motion Generalization: H / E0 / E1 / E2 — C5

Core claim, bounded:

> Target behavior did not form a monotonic difficulty ladder under the frozen policy. The
> static-target arm produced the lowest close-approach success, while the bounded obstacle-aware arm
> produced the highest. Visibility records were consistent with a reacquisition mechanism, and the
> target-behavior gap attenuated as obstacle density increased.

Plus the acquisition evidence: in **95.42 %** of E0 timeout episodes the target was never acquired
even once, against 83–84 % in the moving arms — which locates the failure at **first contact**, not
only at reacquisition.

### 8. Observation / Rendering Sensitivity — C6
### 9. Quantitative Positioning Against Published Systems
Three layers kept separate: MOTAR internal evidence, published context under each paper's own
benchmark, and matched comparison (**empty**).

### 10. Limitations
n = 3 seeds; 98,319 observed vs 98,304 preregistered; `min_relative_distance_m` `NOT_RECORDED`;
5 pp threshold never preregistered and withdrawn; no real flight; `causality_vs_d8b = NOT_TESTED`;
P2/D1 FAIL and P3 BLOCKED; Class A = 0.

### 11. Conclusion

## Contribution bullets

Each carries a measured number or a precisely bounded capability. No "novel framework" claim.

1. **We quantify target-motion distribution shift under one frozen policy across four motion
   contracts**, observing **−3.58 pp** for static targets and **+1.33 pp** for bounded
   obstacle-aware targets relative to the historical reference (BCa95 [−4.04, −3.29] and
   [+1.15, +1.50], 3/3 seed sign-consistency), showing that target behaviour is not a monotonic
   difficulty ladder for this policy.
2. **We measure a perception error distribution on real imagery and propagate it into policy
   outcome**, costing the frozen policy **−4.57 pp** of close-approach success with an interval
   excluding zero in all three training-seed campaigns, and we report that readaptation does **not**
   establish a net benefit (**+0.73 pp**, CI [−1.04, +2.50]).
3. **We separate safety-filter geometry from speed-governor effects** under matched configured arms,
   measuring **−1.4903 pp** crash (CI [−1.8981, −1.0826], lower in 15/15 cells) and showing the
   opposite sign for the same widening on a sibling geometry.
4. **We show that the observation-rendering contract can dominate every other effect measured**: the
   same frozen policy loses **−48.967 pp** of capture under a mesh-shaded observation
   (CI [−50.113, −47.821]), with renderer causality explicitly `NOT_TESTED`.
5. **We release the negative record with the positive one** — 204 indexed results with receipts,
   including a withdrawn decision threshold, a preregistered primary recorded as `NOT_RECORDED`, and
   a preregistered gate that FAILED — so that the evidence chain can be audited rather than trusted.
