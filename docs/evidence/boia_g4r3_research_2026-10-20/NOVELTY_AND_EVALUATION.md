# G4-R3: opportunity-window research, prior art and mission evaluation

**CPU_RESEARCH_ONLY / NOT_PREREGISTERED / NOVELTY_NOT_VERIFIED.**
PART C began only after C02 local commit
`8689efcd066d64e7b49cecf011e3bd8675380cdb`. This document and the research module
are uncommitted by the user's instruction. No online integration, calibrated
sensor model, new policy, GPU/Isaac run or mission-effect estimate is provided.

## What is and is not established

The proposed research claim is that, with the **same sensor evidence, candidate
pool and safety constraints**, selecting whether and when to intervene using a
supported opportunity window may improve collision-free capture beyond reactive
or predictive-visibility planning. It is a hypothesis. An implementation that
chooses WAIT rather than an immediate threshold on a synthetic table establishes
decision-rule differences, not a navigation or capture improvement.

In the current prototype, the supported wait mask and candidate/wait argmax alter
selection; the `closing` flag summarizes the first connected current window and
is diagnostic. It is not a separately tested trigger that overrides the argmax.
Thus a WAIT-versus-immediate result isolates temporal option selection, **not** a
proven novel opportunity-closing principle. An arbitrary closing override would
not establish novelty either. This distinction must remain in any paper wording.

No mechanism currently receives **VERIFIED novelty**. The labels below separate
ALREADY_KNOWN broad principles, CANDIDATE specific contributions and NOT_VERIFIED
originality. CPU test PASS is a different judgment from VERIFIED novelty.

## Reading boundary

Original-copy reading receipts, paper_manifest and notes in
`docs/research/occlusion_algorithm_v1/` support the prior named-tracker comparisons.
Their earlier full-read provenance is carried forward, not presented as a new
full reread. The earlier G4-R novelty review remains unchanged and untracked.

For runtime assurance, the author publication record and arXiv abstract were
checked on2026-10-20, **abstract/metadata scope only**, not its complete proofs or
implementation: Hobbs,Mote,Abate,Coogan,Feron, *Run time assurance for safety-critical
systems: an introduction to safety filtering approaches for complex control
systems*, IEEE Control Systems Magazine2023 ([author record](https://coogan.ece.gatech.edu/papers/hobbs2022csm.html),
[arXiv2110.03506](https://arxiv.org/abs/2110.03506)). The primary record supports
the generic monitor/backup/performance-controller overlap. It does not qualify
BOIA or establish the novelty of a mission-specific timing rule.

For robust baseline regret, the previously checked venue paper pp.1–3, Definition2
and Eq.(2), directly overlap with shared-model policy improvement. See
[Petrik,Ghavamzadeh,Chow2016 venue PDF](https://proceedings.neurips.cc/paper_files/paper/2016/file/9a3d458322d70046f63dfd8b0153ece4-Paper.pdf)
and [arXiv record](https://arxiv.org/abs/1607.03842). The venue/arXiv author orders
differ. No infinite-horizon MDP theorem is transferred to the current short-window
surrogate or partially observed UAV.

## Required mechanism comparison

| Proposed mechanism / closest work | Existing mechanism and overlap | Candidate specific difference | Novelty status / necessary experiment |
| --- | --- | --- | --- |
| Whole-prefix backup / [FASTER](https://arxiv.org/abs/1903.03558v3) | Known-free committed trajectory and backup stopping. | Correct C02 time alignment is validity work. | ALREADY_KNOWN; not a paper contribution. |
| Future observation belief / [BPOD](https://arxiv.org/abs/2306.06363v2) | Probabilistic FOV/LOS detection and expected covariance. | Separate evidence support from predicted probability and intervention timing. | Broad mechanism ALREADY_KNOWN; new construction NOT_VERIFIED. Compare at equal samples/safety. |
| Active viewpoint / [Eva-Tracker](https://arxiv.org/abs/2602.12549v1) | Visibility-oriented viewpoint and trajectory/yaw planning. | Preserve safe F when a supported later opportunity dominates current intervention. | CANDIDATE timing/application distinction; compare reactive/immediate variants. |
| Visibility corridor / [Elastic Tracker](https://arxiv.org/abs/2109.07111v2) | Target prediction, visible regions and spatiotemporal optimization. | Explicit refusal for unsupported association and action ownership over F. | NOT_VERIFIED; visibility/corridor planning ALREADY_KNOWN. |
| Loss/reacquisition / [UAST](https://github.com/qinliangql/UAST) | Prediction/search and finite learned candidate grid. | This unit has no search policy; tests pre-loss timing decisions. | ALREADY_KNOWN search ideas; candidate timing must add mission benefit. |
| Tracking primitives / [YOPOv2-Tracker](https://arxiv.org/abs/2505.06923v1) | Predicted-target tracking/navigation primitives. | Unchanged Frozen F and a measured-evidence gate, without learned new primitives. | NOT_VERIFIED; primitive scoring alone ALREADY_KNOWN. |
| RL safety mediation / [NavRL++](https://arxiv.org/abs/2605.15559v1) | Navigation learning with system sensing/filtering. | Utility timing must improve a target capture mission beyond a common safety layer. | NOT_VERIFIED; safety mediation ALREADY_KNOWN. |
| Proposal acceptance / [PRIMER](https://arxiv.org/abs/2406.10060v4) | Learned proposal checks, delayed checks and prior-plan fallback. | Sensor-identifiable paired option benefit over a frozen interceptor. | General check/fallback ALREADY_KNOWN; specificity CANDIDATE only. |
| Uncertainty objective / [risk-averse MPC](https://arxiv.org/abs/1704.00342) | Predictive decisions with ambiguity/risk and terminal conditions. | Finite wait-then-intervene options relative to a shared baseline. | Generic robust objective ALREADY_KNOWN; timing novelty NOT_VERIFIED. |
| Mandatory filter / [predictive safety filter](https://arxiv.org/abs/1812.05506) | Validate/modify a proposed input with predictive backup constraints. | Utility ABSTAIN and opportunity timing sit outside the mandatory gate. | Filter ALREADY_KNOWN; no recursive-feasibility theorem here. |
| Monitor/backup / [runtime assurance](https://arxiv.org/abs/2110.03506) | Unverified primary controller monitored and modified/replaced to enforce safety. | Mission utility determines optional intervention only after independent safety. | ALREADY_KNOWN architecture; no new RTA guarantee. |
| Belief/action / [active context-POMDP tracking](https://arxiv.org/abs/1912.06754) | Bayesian/particle context belief and actions for occluded/missing targets. | Bounded, abstaining action-option comparison with explicit F continuation mismatch. | Belief/active sensing ALREADY_KNOWN; formal originality NOT_VERIFIED. |
| Paired robust gain / [robust baseline regret](https://arxiv.org/abs/1607.03842) | Same uncertain model evaluates candidate-minus-baseline performance. | Only concrete sensed-evidence/timing/calibration construction can differ. | The formula ALREADY_KNOWN; do not rename it as a new principle. |

Potential contribution is at most the **specific evidence-supported temporal
option rule plus a useful matched mission result**. Generic thresholding,
inflation, candidate rollout, filtering, detection probability, robust regret
and safe fallback are not new. Active perception, viability/event-triggered
control, selective prediction and safe improvement under partial observability
still require deeper comparison. Absence from a small set of papers is not proof.

## Prospective contrasts and endpoints

Keep the existing F0–F5 definitions by mechanism: literal F0; F1 with the SAME
qualified mandatory safety layer; reactive F2; deterministic predictive F3;
belief-sampled F4; evidence/paired-benefit/wait-option F5. Setting two weights to
zero is not F1, dropping U is not removing belief, and tau_on=0 is not unconditional
switching. A guarded F0 is not literal unchanged F0. Unsafe backup-OFF is offline
only. No arm is executed in this unit.

The key new contrast is finite wait grid versus immediate-only `{0}` with the
same information, scoring, uncertainty set, samples and hard safety. A second
contrast uses an actual-loss trigger rather than a future supported option.
Evidence ON/OFF and robust ON/OFF should form a declared2×2 comparison; neither
OFF is permitted to remove collision hits, add GT or weaken mandatory safety.
Association/latency stresses require equal measured inputs and offline truth.

Primary proposed mission outcomes remain collision-free capture, crash, timeout
and time to capture. Reacquisition/visibility are intermediate. Report every
assigned episode, safe abort, no-backup and deadline miss; waiting forever is
not successful safety. Measure actual action/effort and controller motion, not
only whether a planner declares itself engaged. Retain FULL/CENTRE_ONLY/PARTIAL,
FOV/range exit, identity-correct versus flag-only reacquisition and censored
durations with their denominators.

Use independent calibration/design/test seed-scene sets. Compare seed-cluster
effects with fixed density/motion/start weights, cluster intervals and a declared
multiplicity family. Capture/crash/timeout/abort are competing outcomes. Arm-specific
post-treatment occlusion populations are descriptive, not automatically causal
matched recovery samples. Common-prefix branches require identical full allowed
state/history and qualified independent exogenous streams before intervention.
Do not pool placement/motion lineages or call bar count physical density.

No new sample size or event prevalence is estimated here. Calibration of likelihoods,
probability intervals, set/MC/model allowances and F-continuation discrepancy is
still absent. Supplying a CALIBRATED tag is a caller declaration, not evidence
that the underlying artifact exists or generalizes. A finite-scenario synthetic
minimum is not a bound over unenumerated real scenes.

## Research rejection conditions

If finite-window timing differences do not survive actual perception/continuation
calibration, reject the timing model. If the method refuses almost everything,
report inadequate evidence/coverage rather than relax bounds. If visibility gains
harm capture, redesign or remove that utility. If matched F1/F2/F3/F4 explain all
mission benefit, narrow or withdraw the novelty claim. CPU oracle success alone
cannot decide any of these mission questions.
