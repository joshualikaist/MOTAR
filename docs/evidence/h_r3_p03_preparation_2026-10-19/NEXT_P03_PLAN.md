# Next H-R3 P03 — design proposal only

Status: **PLANNED / NOT_EXECUTED / USER_REVIEW_REQUIRED**. H-R3 PROVISIONAL; confirmation HOLD.
This document is not a new preregistration and authorizes no experiment or production changes.

## Decision before another GPU pilot

Mechanics now pass the seed4801/N128/moving160/600-step profile. The remaining bottleneck is
causal control design: common nominal budgets passed, but B3/B4 and B3/B5 realized motion failed
the predeclared10% diagnostic. More scenes cannot remove that interpretation problem.

The directly supportable future estimand is a **direction-policy total effect under a shared
pre-loss nominal magnitude/speed budget**, including direction-dependent safety and motion.
It must not be named a pure viewpoint effect. If the narrower mechanism is required, first
approve a minimal separate control-design change; do not automatically launch the pilot below.

Minimal candidates to review before implementation:

1. Replace10Hz independent B4 signs with a preregistered event-held side or coarser decision
   epochs; compare turning/switching histories. A shared slew rule applied to B3/B4 could reduce
   acceleration asymmetry, but changes both interventions and must be frozen before new data.
2. Preserve the same pre-loss magnitude schedule and independent sham key, keeping impossible
   or zero-lateral events in the denominator. Do not copy B3's future norm/state to another arm.
3. Retain B5 as a common nominal-speed reference with realized-effort diagnostics, not a claim of
   actual-speed matching. If evaluating attenuation, choose any common cap only from pre-loss
   information and apply the same safe rule prospectively; never compensate through obstacles.
4. Do not match/exclude episodes on post-treatment path or safety outcomes. Analyze differences
   as mediators/diagnostics. Even formal motion equivalence would not by itself identify a
   controlled direct viewpoint effect; that requires an explicit causal estimand and intervention.

These are designs only; no new controller, sensor lineage, planner or PPO work is implemented.

## Optional small exploratory pilot after user review

- Seeds4802 and4803, proposed fresh identifiers to recheck against manifests before preregistration.
- B0/B2/B3/B4/B5, moving E2_direct_l2_v1,160bars,N128,600steps: ten processes. B1 is omitted
  because the current unit already measured it as a diagnostic. A genuinely new RunID is required.
- No claim of independent population confirmation from two seeds. Freeze any control revision,
  analysis and sample size before GPU; keep600-step prefix/reset/evaluator/mode guards intact.
- Proposed caps:600s per process,3600s aggregate, disk8GB minimum,512MiB GPU reserve, no retries,
  no adaptive seed expansion. Estimated process wall35–45min from current~220s/runtime process;
  allow preparation/analysis separately. About1.7GB saved output if current snapshot policy holds.
- Current26 geometry events per128 first-episode opportunities suggest around52 across two seeds
  only as a rough single-seed yield illustration. Event counts and classes can differ substantially;
  no minimum event count or partial-class coverage is promised. Report zero/sparse strata honestly.
- Keep primary spatial identity-correct reobservation by2s with terminal-first scoring. Report
  FULL/CENTRE_ONLY/PARTIAL and FOV_EXIT separately, all denominators, pre-loss identity diagnostic,
  zero-budget events, sham/switching patterns, and paired path/speed/effort/safety distributions.
- Preregister final original-episode capture/crash/timeout/censor follow-up in addition to early
  reobservation; no new claim that early reacquisition is task success. Safe abort remains
  NOT_RECORDED unless separately specified/implemented. LiDAR nearest return remains a proxy;
  obstacle-only GT clearance, if approved later, is evaluator-only and requires a separate metric.

## Future confirmation design — HOLD

Use entirely new independent seeds and paired scene opportunities. P01, failed P02 and this
qualification/pilot data are design evidence only and cannot enter the confirmatory sample.
Freeze the target population, arms, handling of no-event seeds, all terminal competing events,
estimation weights, margins, contrasts and stopping rules before data collection.

At least the three planned contrasts are B3−B2, B3−B4 and B3−B5. Use two-sided Holm family-wise
alpha.05 for these three; class-specific effects are estimation-only unless separately powered
and registered. Average paired outcome differences within seed, then use prespecified seed
weights (equal-seed or opportunity-weighted, chosen before data). Uncertainty must resample/test
independent seed clusters, retaining within-seed scene pairing and the common B3 dependence.
For zero-event seeds, do not silently drop or replace them: define the estimand and missing-event
handling prospectively, and report opportunity/event counts by seed. Event-onset selection is
common pre-intervention; outcomes/realized motion must not select the analysis set.

Use a sufficiently large independent-cluster design (proposed32-cluster planning floor, not a
proven statistical threshold). Before finalizing the design, run joint cluster-level power and
coverage simulations across event-yield variation, discordance, contrast ICC, imbalance and the
three correlated contrasts. No such simulation or confirmatory experiment was run in this unit.
If adaptive sample sizing is desired, preregister an appropriate blinded or alpha-controlled
procedure rather than inspecting favourable effects and extending the run.

## Assumption-only sample-size sensitivity

Paired binary difference variance is q−delta², where q is discordance probability. The stored
[sensitivity grid](sample_size_sensitivity.json) uses target differences10/15/20pp, q=.2/.4/.6,
10/20 events per seed and contrast ICC0/.05/.10/.20. It uses a two-sided Bonferroni3 planning
bound for eventual Holm tests and80% normal-approximation power, with design effect1+(m−1)ICC.
The observed favourable qualification effect is not used for power planning.

Illustration at20 events/seed and ICC.10:

| Target difference pp | Assumed discordance q | Paired scenes | Seeds arithmetic | With proposed32-cluster floor |
| --- | --- | --- | --- | --- |
| 10 | 0.2 | 577 | 29 | 32 |
| 10 | 0.4 | 1185 | 60 | 60 |
| 10 | 0.6 | 1792 | 90 | 90 |
| 15 | 0.2 | 240 | 12 | 32 |
| 15 | 0.4 | 510 | 26 | 32 |
| 15 | 0.6 | 780 | 39 | 39 |
| 20 | 0.2 | 122 | 7 | 32 |
| 20 | 0.4 | 274 | 14 | 32 |
| 20 | 0.6 | 426 | 22 | 32 |

The32-seed floor is an explicit conservative proposal; it is not validated by this formula.
Equal cluster sizes/exchangeable ICC, normal approximation and no joint simulation make these
numbers planning assumptions, not a guaranteed sample size or an execution recommendation.
The pure-viewpoint design limitation remains regardless of sample size.

## Next-session handoff

Completed: preregistration e5d4c791; source0e879010;52 CPU tests; one focused Fable review with
pre-GPU closure;11/11 bounded qualification processes; independent221-member new raw SHA audit;
compact primary/effort/resource/terminal evidence and preservation record.
No owned GPU/review process remains. No large pilot, push, merge, deletion, retraining or planner.
Next action is user review of REPORT and this estimand/control proposal. Do not rerun qualification
or start the optional pilot automatically. The local result commit contains the complete handoff.
