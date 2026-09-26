# Paper evidence spine — 2026-09-19

What is the scientific contribution of MOTAR now that the experiments are complete?

Organised by evidence, not by chronology. Each contribution states what it establishes, what it
measures, and — the part that decides whether this is a paper — what it does **not** prove.

Numbers here are the canonical, source-bound values in
[`quantitative_positioning_registry.json`](quantitative_positioning_registry.json) and
[the target-motion result](results/target_motion_generalization_2026-09-19.md). External context is
in [the quantitative ledger](literature_quantitative_ledger_2026-09-18.json); no external value is
ever subtracted from a MOTAR value.

---

## C1. Perception uncertainty is measured rather than assumed

**Question.** How wrong is the perception stack, on real imagery, in units the simulator can use?

**Method.** P8 error measurement on recorded flight imagery; E3-S image-size-to-range proxy on the
ETH ds5 flight with held-out blocks.

**Result.** E3-S `SIZE_RANGE_USABLE`: held-out median absolute relative range error **6.2 %**, the
median of 9 block medians (15 s blocks) from one flight, 30.7–108.4 m. The 3,107 frames are not
independent samples; the unit is the block, and there is one flight.

**Uncertainty.** One flight; an apparent-size proxy, not ground-truth boxes. Range and time are
confounded in that flight.

**Closest external work.** No screened system publishes a measured error distribution intended for
injection. NavRL++ perturbs with its own synthetic failure model instead.

**What MOTAR adds.** The error is a *measurement* with provenance, not a chosen noise level.

**What MOTAR does not prove.** That this error generalizes beyond the studied flight. E3-P is
`ATTITUDE_NOT_RELIABLE` and the pose/range decomposition stays **BLOCKED**.

**Source.** `results/eth_ds5_e3s_2026-09-10/`. **Figure.** Fig 2.

---

## C2. Measured perception error is propagated into the navigation policy

**Question.** What does that measured error cost a policy that never saw it?

**Method.** P9 injects the P8 distribution into simulation; frozen policy F is evaluated under it;
P10 retrains under it across three training seeds.

**Result.** The injected error costs frozen policy F **−4.57 pp** of capture, with an episode-level
interval (two evaluation seeds at 205 bars) excluding zero. The value is identical in all three
campaigns because they re-ran the same deterministic frozen arms: a determinism check, not three
replications. Readaptation does not establish a recovery: **+0.73 pp**, 95 % CI **[−1.04, +2.50]**
over three training seeds — `INCONCLUSIVE`. The residual cost after readaptation
is never zero (−1.90 / −2.71 / −3.84 pp per seed), and readaptation costs clean performance
(−1.02 pp mean).

**Uncertainty.** Three training seeds, one density, one arena, one injector.

**Closest external work.** NavRL++ reports perception failure as its largest perturbation factor,
">5 % drop", under its own model and benchmark.

**What MOTAR adds.** A closed measure→inject→evaluate→retrain loop where the injected quantity was
measured on real data, and where the retraining answer is reported as inconclusive rather than
promoted.

**What MOTAR does not prove.** That readaptation works. It does not. That is the finding.

**Source.** `results/perception_p10_seed_replication_2026-09-10/`. **Figure.** Fig 2.

---

## C3. Temporal perception and association are tested explicitly

**Question.** Does a temporal selector beat a per-frame one, and does that survive held-out data?

**Method.** P6/P7 detector-candidate preparation and a Transformer temporal selector, chosen on
validation utility, then checked on S4.

**Result.** Transformer **0.6725** vs CNN-only **0.6655** validation utility, **+0.00697**. Selected
on validation. S4 test utility **0.4701** retains its generalization warning. A learned detector is
non-inferior for navigation: **−0.015 pp** capture, 95 % CI [−1.752, +1.723], against a −2 pp margin.

**Uncertainty.** Dataset- and lineage-specific. Non-inferiority is not superiority.

**Closest external work.** Fast-Tracker (EKF/FIFO history, Bézier prediction, prediction error
1.82/2.54/3.45 m under three noise levels) and YOPOv2-Tracker (EKF plus spatiotemporal consistency).

**What MOTAR adds.** The selector is evaluated separately from the detector, and the held-out
warning is retained rather than dropped.

**What MOTAR does not prove.** Persistent instance identity. The dataset cannot supply it; the
identity claim is explicitly not made.

**Source.** `results/perception_temporal_p6_p7_2026-09-08/`. **Table.** Site Table 3.

---

## C4. Safety-filter geometry is separated from speed-governor effects

**Question.** What do the configured filter geometries actually do, and where do they fail?

**Method.** Matched configured arms over recorded runs with crash-cause attribution and replication.

**Result.** riskcap vs arc-clearance on policy F: crash **−1.4903 pp**, 95 % CI **[−1.8981, −1.0826]**
pooled over 15 seed × density cells with episode-binomial errors; the seed-level 95 % CI over the
three evaluation seeds is **[−2.42, −0.57]**. Crash is lower in **15/15 cells**, which share three
seeds and one policy and are not independent replicates. Arc width 0.45 → 1.2 m at 205 bars: crash **−5.60 pp**, capture **+4.44 pp** — and
the same widening degrades straight stopcap capture, so the effect is geometry-specific rather than
a general width rule.

**Uncertainty.** Configured comparisons in simulation, one policy lineage.

**Closest external work.** Temporal Barrier's aTTC-CBF: 0.170 / 0.034 / 0.012 collisions per 100 s
(no CBF / HOCBF / aTTC-CBF), in an **obstacle-free** domain with full relative state.

**What MOTAR adds.** The geometry is varied while everything else is held fixed, and the opposite
sign on a sibling geometry is reported.

**What MOTAR does not prove.** Any formal safety property. There is no certificate; the C3
explanation is `WITHDRAWN` and route-mechanism gates are `FAIL_ROUTE_MECHANISM`.

**Source.** `results/independent_verification_2026-09-07/`. **Figure.** Fig 3.

---

## C5. Frozen-policy target-motion generalization is measured and replicated

**Question.** Does a policy frozen on one target-motion lineage stay valid when the motion law changes?

**Method (current evidence).** An independent bias-corrected replication, preregistered before it
ran (`docs/prereg_2026-09-24_target_motion_replication_7seed.md`).
- Design: 4 arms × 4 densities × **7 fresh seeds** = 112 cells, against arm **H**, the historical
  lineage the checkpoint was trained on.
- Estimand: exactly 2,048 primary episodes per cell (the first 16 of each of 128 environments),
  229,376 in all. This per-environment quota removes the length bias of the pooled stopping window.
- Inference: seed-paired BCa95, exact sign-flip p over 2⁷ patterns, Holm across E0 − H, E1 − H and
  E2 − H.

**Result.**
- Arm means: H **87.76 %**; E0 static **82.39 %**; E1 CV **84.32 %**; E2 obstacle-aware **89.12 %**.
- E0 − H **−5.37 pp** [−5.78, −4.85], E1 − H **−3.45 pp** [−3.79, −3.11], E2 − H **+1.35 pp**
  [+1.07, +1.87].
- All three preregistered capture contrasts met the directional replication criterion in the
  independent seven-seed bias-corrected replication. The Holm-adjusted p-values were 0.046875, at the
  resolution boundary of the seven-seed exact sign-flip design.
- Effect sizes, uncertainty intervals and seed-level consistency (7/7 in each contrast, descriptive)
  are reported alongside the hypothesis tests.

**Target behavior did not form a monotonic difficulty ladder.** The static arm produced the lowest
close-approach success and the bounded obstacle-aware arm the highest.

**Estimator.** Within the seven-seed data, the historical pooled stopping window attenuated all
three capture contrasts toward zero (quota − legacy: E0 − H −1.01, E1 − H −0.33, E2 − H +0.27 pp).
The estimator change does not by itself explain the full difference between the original n=3
campaign and the n=7 replication because the seed sets differ.

**Uncertainty and limits.**
- The aggregate E2 − H contrast replicated, but its positive direction was not uniformly expressed at
  high obstacle densities: 5/7 seeds at 160 bars and 4/7 at 205 bars (exploratory).
- `min_relative_distance_m` is recorded for every episode, but it failed its semantic check
  (end-of-step sampling versus swept-segment capture), so it is not published.
- Frozen policy F, one arena family, four densities, simulation only.

**Original campaign (historical, not pooled).** Three evaluation seeds, legacy pooled window:
- E0 − H **−3.58 pp** [−4.04, −3.29], E1 − H **−2.82 pp**, E2 − H **+1.33 pp** [+1.15, +1.50].
- With n = 3, exact permutation inference had insufficient resolution for conventional significance
  testing.
- 98,319 observed episodes against a preregistered 98,304.
- The 5 pp materiality rule was **not** preregistered and is withdrawn.
- Its first-acquisition record is consistent with a visibility mechanism (association only).

**Closest external work.** None. No screened system evaluates a frozen policy across a ladder of
target-motion contracts. Elastic Tracker names escaping targets as future work; every classical
tracker evaluates a cooperative target.

**What MOTAR adds.** The only measured, independently replicated statement in this set about whether a
target-motion distribution shift degrades a frozen policy.

**What MOTAR does not prove.**
- That E2 is universally easier, that obstacle-aware targets improve every policy, or that the effect
  holds at every density.
- That the E0/E1 losses call for retraining: no deployment-performance threshold was preregistered.

The result is bound to this checkpoint, this arena family and this simulator contract.

**Source.** `results/target_motion_replication_7seed/` (current); `results/target_motion_e0_e2_2026-09-18/`
(historical). **Figure.** Fig 4.

---

## C6. Observation and rendering changes can materially alter policy behaviour

**Question.** Does changing how the world is *rendered into an observation* move a frozen policy?

**Method.** D8b evaluates one frozen policy, R (ref5in D1 ep1900; not policy F), under an analytic
versus a mesh-shaded observation, with three paired evaluation seeds at 70 bars.

**Result.** **−48.967 pp** capture, 95 % seed-t CI **[−50.113, −47.821]**, `MATERIAL_LOSS`.

**Uncertainty.** Same policy, same task; only the observation treatment changed.

**Closest external work.** None found. Published systems fix one observation pipeline and do not
report frozen-policy sensitivity to changing it.

**What MOTAR adds.** A magnitude for observation-contract sensitivity that dwarfs every other effect
measured here — an order of magnitude larger than the perception-error cost.

**What MOTAR does not prove.** **Causality. `causality_vs_d8b = NOT_TESTED`.** No renderer result
shows that area mismatch, shading or mesh geometry caused the loss. D8b says a frozen policy scored
worse under a new observation treatment, and nothing more.

**Source.** `results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/`. **Figure.** Fig 5 companion.

---

## C7. Every result carries provenance, and negative results are retained

**Question.** Can an outsider tell what produced each number, including the ones that failed?

**Method.** Per-result receipts and source manifests, a repository-wide result manifest, Record
Envelope v2, preregistrations committed before execution, and tests that bind documents to sources.

**Result.** **208** result directories indexed with links and hashes in `results/MANIFEST.json`
(build of 2026-09-26). The public snapshot builds `CLEAN`
with 0 denylist matches. Raw evaluation artifacts verified immutable at **144/144** files.

**Retained negatives, unedited.** `P2 held-out` **STRICT FAIL**; `D1 adaptation` **FAIL**; `P3`
**BLOCKED**; P10 **INCONCLUSIVE**; D8b **MATERIAL_LOSS** with untested causality; C3 explanation
**WITHDRAWN**; RC-R1/R2/R3 failures standing; and, in this cycle, a preregistered primary metric
recorded as `NOT_RECORDED` and a decision threshold withdrawn as never preregistered.

**Uncertainty.** Hosted CI is unverified; several datasets cannot be redistributed.

**What MOTAR adds.** The failure record is part of the artifact, not an appendix.

**What MOTAR does not prove.** That the results are externally reproducible on other hardware.

**Source.** `results/MANIFEST.json`, `docs/REPRODUCIBILITY.md`.

---

## The spine in one line

Perception error is **measured** (C1), **propagated** into a policy with a known cost (C2), the
temporal and safety components are **isolated** (C3, C4), the policy's validity is **tested against a
distribution shift it was not trained on** (C5), the observation contract produces the largest
measured loss, on a different frozen checkpoint (C6), and every step — including the ones that
failed — is **traceable** (C7).

The contribution is the chain and its honesty, not the size of any single number.
