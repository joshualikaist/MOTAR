# OCC-P2 independent confirmation — preregistration v1, 2026-10-17

**Approved conditionally by the user on 2026-10-17.** Commit this preregistration and the analysis
before any OCC-P2 simulation. When its preconditions pass, the user explicitly authorizes this
particular GPU main run without another approval. The [JSON contract](prereg_occlusion_confirmatory_v1.json)
governs executable settings. No main merge, push, PPO, Planner, deletion or overwrite is authorized.
This file becomes immutable at the local pre-execution commit. Amendments require new files.

## Prior knowledge and independence

OCC-P1 is exploratory: density was associated with more in-view geometric occlusion intervals,
while observed intervals remained relatively short. Its result informed the directional questions
below, not a claimed independent test. OCC-P2 outcomes have not been collected or inspected.
No cutoff, endpoint, seed, test or margin will be changed after collecting them.

Use exactly seeds **4301–4314**, independent of OCC-P1's **4201–4207**. Seed numbers 4301–4307
were previously used for hidden-start Experiment B. They are new to this OCC/visible-start
confirmation, not globally never-used seed numbers. The user's explicit list is retained; B
occlusion outcomes were not available or used to select these hypotheses. Independence is claimed
relative to the exploratory OCC-P1 data, not to all prior MOTAR observations.

Preflight freezes and hashes the recorded OCC-P1 scene-hash set. Each newly generated cell is
checked against it before acceptance; any intersection stops the run, preserves the evidence,
and is reported without replacing the seed/scene. Actual new scenes cannot be verified before
generation without a simulator run. Scene hashes are the existing order-independent 1 mm geometry
contract; this is a conservative duplicate screen, not proof that different hashes are statistically
independent. Within-seed repeated/matched scenes stay in the same inferential cluster.

## Design and unchanged system

Static A1/E0_static and Moving A3/E2_direct_l2_v1; densities70/115/160/205;
14 seeds ×2 groups ×4 densities =**112 cells**, each128 environments ×quota4 =**57,344 primary episodes**.
Use the fixed10 m visible start,600-action timeout, Policy F checkpoint SHA
`f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`, 898-D actor, original tracker
expiry, capture/crash rules, E2 direct-distance motion and existing LOS/telemetry byte for byte.
No training, scene injection, policy adaptation or online GT supervisor is included.

Use `NAVRL_KROC2027_OCCLUSION_TRACE=1`, recovery events with compact trace, and the existing moving
`direct_l2_v1`/`full_v2`/stop-on-motion-violation settings. Run seed-major, then increasing density,
then Static/Moving. Include only the first four quota episodes per environment, never batch-tail
completion episodes. Require exactly512 validated quota episodes in every cell, no missing cells,
no deletion of failed episodes and no imputation. A failed or partial design is not confirmatory.

## Primary measurement and estimand

Keep the OCC-P1 primary indicator **`center_occlusion_in_view`**: actual recorded-scene 3D LOS
centre ray blocked, with target centre inside the camera frustum and range. Bar collision and visual
geometry match under the frozen upright bars_h3 contract; G0 mesh/renderer validation is inherited
only after runtime-source identity is proved. It is not fused measurement loss or full silhouette
occlusion. Other P1 indicators are reported separately as diagnostics, not alternative confirmatory
endpoints selected for significance.

The first valid **fused** measurement is acquisition frame A. At physical terminal frame T, exposure
is `max(0,T-A-1)*dt`, excluding the acquiring observation/action; never-acquired episodes contribute
zero. Count contiguous primary-indicator entries strictly after acquisition, including entries
at the observation boundary or re-entry into view, exactly as in P1. Exposure includes blocked and
out-of-view time until capture/crash/timeout. Thus this is **observable in-view interval-entry
frequency per closed-loop observation time**, not a clear-time hazard or every physical LOS crossing.

For each seed s, group g, density d, let `r[s,g,d]=60*N[s,g,d]/E[s,g,d]` in intervals/minute.
Primary point estimates are **equal-weight means across14 seed rates**, not pooled counts divided
by pooled time. Also report pooled counts/time and acquired/never-acquired episodes for context.
Zero cell exposure is NOT_ESTIMABLE and invalidates complete confirmation; do not drop a seed,
substitute zero, add epsilon or select a different denominator.

## Six fixed directional tests, one Holm family

1. RQ1/primary, Static: seed-paired `r205-r70 >0`.
2. RQ1/primary, Moving: seed-paired `r205-r70 >0`.
3. RQ3/secondary: `rMoving-rStatic >0` separately at70,115,160,205.

Use one-sided paired Student t tests of the mean of14 seed differences (df13). Apply Holm
step-down correction to **all six** p values, family alpha.05, without selecting a favorable subset
or gating secondary reporting on the primary result. Report absolute differences/minute, per-seed
differences, nominal two-sided95% t intervals and simultaneous Bonferroni intervals at
`1-.05/6` confidence. Nominal intervals are labeled as such and are not used as corrected tests.
The t test assumes independent seed blocks and an adequately regular seed-contrast distribution.
If seed-contrast variance is zero, t inference/intervals are NOT_ESTIMABLE and that contrast is
INCONCLUSIVE; include a conservative1 in the six-test Holm calculation without reporting it as
an estimated p value. Do not substitute the sensitivity result as the primary test.

Every seed keeps all scenes, recurrent intervals, environments, episodes, groups and densities
together; episodes/frames/individual intervals are never treated as independent replicates for
these tests. Prespecified sensitivity checks: all16,384 seed sign flips (requires symmetric seed
contrasts; this is not literal randomized treatment assignment) and paired seed-block percentile
bootstrap95% intervals,20,000 resamples, RNG20261017. Reuse the same resampled14-seed vectors for
all eight group/density summaries and six contrasts. The primary decision remains the t/Holm rule
even if a sensitivity result differs. Report the discrepancy.

Each contrast is SUPPORTED iff its estimate is positive and Holm p<.05; CONTRADICTED if its
simultaneous interval lies wholly below zero; otherwise INCONCLUSIVE. RQ1 is SUPPORTED only if
both primary contrasts pass; RQ3 only if all four secondary contrasts pass. Otherwise the overall
RQ is INCONCLUSIVE and all individual decisions are retained. Endpoint confirmation does not prove
strict monotonicity through both intermediate densities. Intermediate values are fully reported.
Moving-versus-Static is a whole-system motion-lineage effect, not target speed's isolated causal effect.

## RQ2 duration: estimation, no equivalence claim

There is no externally justified practical equivalence margin for these geometric durations.
Do not invent one from P1 or choose one after P2. **RQ2 judgment is
INCONCLUSIVE_EQUIVALENCE_NOT_TESTED**: no claim that density conditions are equivalent, even if
a duration contrast is nonsignificant. Estimate endpoint changes in seed-specific observed median
elapsed duration with nominal seed-level t intervals; do not promote a small estimate to equivalence.

Report median and25/75/90/95th percentiles of elapsed interval observations and of actually
geometry-restored intervals, clearly marked censor-selected descriptive quantities. Report the
original P1 observable-duration summary separately for direct comparison. At0.5/1/2/3/5seconds,
give known-at-least fractions and bounds: lower=`observed duration>=threshold / all intervals`;
upper also includes intervals censored before that threshold. A censored0.3 s observation is
unknown at1 s, not a measured short duration. Thresholds1/2 s are the user-requested main summaries;
5 s remains a stress-test candidate.

Split each apparent interval end into **geometric_restored**, **out_of_view** (including range
exit), **capture**, **crash**, **timeout**, or unknown. FOV/range exit is not LOS restoration.
Use geometry-restoration KM curves only with their independent-censoring caveat; compute
Aalen–Johansen cumulative incidence retaining FOV and terminal events as competitors. Give risk
sets at each threshold; zero support at5 s means NOT_ESTIMABLE, not proof that5 s cannot occur.
All quantities are sampled at10 Hz; sub-frame transitions and within-step cause ordering are unresolved.

Classify interval onset as actual clear-to-blocked, already-blocked on post-acquisition entry
(left-prevalent), or re-entry into view while blocked. Also give incident-only duration/CIF summaries
for clear-to-blocked events; do not present residual observed intervals as complete physical spells.
Reacquisition is fused-measurement resumption after actual geometric restoration, with terminal
censoring recorded. Report all restorations and the subset whose immediately preceding blocked
frame had no fused measurement; carried-through measurements are not mislabeled as a new lost-track
recovery. This does not prove persistent physical target identity.

## Outcome-dependent observation and sensitivity

Capture, crash and timeout shorten exposure differently. Report all episode terminal fractions,
observed time, acquired/never-acquired counts and occlusion end causes by seed/group/density.
These rates describe the frozen closed loop with its actual stopping rule; do not infer an
unobserved counterfactual trajectory after termination or a geometry-only causal density effect.
Report fixed first1/2/5 s post-acquisition restricted-window encounter rates using observed
exposure truncated at the earlier episode end. Report actual clear-to-blocked incidence as a
sensitivity endpoint, separately from the frozen P1-compatible primary counts. Duration/CIF point
distributions are interval-weighted; inferential duration contrasts use seed blocks, not event-IID errors.

## Execution, preservation and stop rules

New runner and analysis: `tools/occlusion_confirmatory.py`,
`tools/occlusion_confirmatory_analysis.py`; CPU tests `tests/test_occlusion_confirmatory.py`.
All runtime paths must match G0 commit6441c1ff74b6d085d14ed5604d1d60d66a4634fe exactly; no new G0
claim for changed simulation code. Pin G0 PASS report, P1 analysis and all prior SHA manifests.
Verify all manifest members, source HEAD/tree, checkpoint, pinned launcher/player, import origins
inside this worktree, exactly one RTX3070, and a fresh resolved output root before launch.

Hard wall limit **6 hours** for GPU execution and **600seconds per cell**, with2 s polling of
free space on the actual output filesystem and tracked source cleanliness. Refuse start/terminate
the entire running process group at free space<**8 GB**; keep every partial file. TERM cleanup has
at most10 seconds of grace followed by KILL of the original group even if its Bash leader exits.
Source HEAD or
tracked dirtiness—including WORKLOG—also stops immediately. Log first failure and stop; no automatic
retry/resume, seed substitution or outcome-dependent sampling. No total30 h GPU budget applies.
Finish all code/docs/WORKLOG edits and the local commit first; **no tracked edits during execution**.

Raw output is a new folder under main checkout's `results/occlusion_confirmatory_v1/OCC-P2`;
compact reports and raw SHA manifest are preserved separately from bulk raw. Existing worktree raw
symlinks remain untracked and are not auto-added by a merge. Before/after verify old manifests and
Policy F; after completion create a new P2 RAW_MANIFEST covering raw, receipts, report and analysis.
External backup remains incomplete and must not be confused with a same-filesystem copy. Preserve
all existing raw, stopped attempts, manifests, WIP and stash; no cleanup is authorized.

## Method references

[SciPy paired t-test](https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.ttest_rel.html)
and [paired/sign-flip permutation contract](https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.permutation_test.html).
[Competing-event censoring and incidence](https://www.bmj.com/content/378/bmj-2022-071349)
motivates reporting competing outcomes instead of interpreting1−KM as marginal restoration probability.
