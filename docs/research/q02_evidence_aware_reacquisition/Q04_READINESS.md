# Q04 timing-learning readiness: utility, counterfactual data, statistics, model selection

2026-10-22 KST, `codex/mt-p1`. **DESIGN_ONLY.** Nothing here has been trained, run or approved.

- ΔU is NOT_ESTIMATED.
- The BOIA rule abstains.
- GPU NOT_RUN, G5 HOLD.
- Every GPU step needs its own preregistration, committed first, and a separate user approval.

This file adapts §§2–4 of the staged plan, which is kept unchanged outside Git as
`q02b_staged_wip_2026-10-21/docs/BOIA_TIMING_EXPERIMENT_PLAN.md`. It adapts them to the Q02A-v2.6
identity contract and to the Q03 map and planner v0.

The draft preregistration is [Q04_PREREG_DRAFT.md](Q04_PREREG_DRAFT.md). It is a draft at a
non-preregistration path. It becomes a preregistration only when it is copied, completed, to
`docs/plans/prereg_*.md` and committed.

## 0. Readiness verdict (what blocks what)

| Q04 stage | Needs | State on 2026-10-22 |
| --- | --- | --- |
| Q04-D0: CONTINUE-only shadow data (F0 arm with all five shadow triggers logged) | the Q02-B/Q03 pipeline as an observer, evaluator labels, approval for a GPU run | **HOLD**. The CPU code exists, and the observer tap and evaluator labels are designs. It becomes READY_FOR_PREREG only after Opus review P1-1 and P1-2 (loss counting, L2 current velocity) and P2-2 (trigger-met flag) are fixed and re-reviewed |
| Q04-D1: paired CONTINUE vs REACQUIRE branches (labels for ΔU) | an executor that turns a candidate into Frozen F's velocity/yaw-rate command; candidates qualified at L3 | **BLOCKED**: no executor; L3/L4 are NOT_VALIDATED |
| Q04-L: learning E[ΔU \| h] | D1 data | **BLOCKED** on D1 |
| Q04-C: closed-loop comparison F0/H1/H2/H3/BOIA | an executor, L3/L4 evidence and a learned or abstaining BOIA rule | **BLOCKED** |

The only data-collection step that the current code can support is D0. D0 gives three things:
- E[U | h, CONTINUE];
- shadow trigger distributions;
- per-decision features with evaluator labels.

D0 gives no REACQUIRE outcome, so it cannot estimate ΔU.

The scripted yaw/lateral arms of the H-R3 pilot (`HR3-P01`) are the only existing branch mechanism
that does not belong to F. They are not a Q03 executor, and using them as one needs its own design
and approval.

## A. Timing utility

**Utility per episode.**

  U = 1[collision-free capture]  (the primary outcome; capture as defined in Q01).

Capture means swept distance < `success_radius` = 0.5 m (`navrl_task_config.py:727`), with:
- no thresholded contact substep in the episode;
- no action-boundary height or out-of-bounds violation.

Reacquiring the target without capturing it scores 0.

**Decision quantity.**

  ΔU(h) = E[U | h, REACQUIRE] − E[U | h, CONTINUE] − C_switch

**Hard constraints come before any weighted sum.** A REACQUIRE proposal is admissible only if all of
these hold:
1. **Eligibility.** The target was previously acquired (2 usable detections) and a loss is confirmed.
2. **Identity.** No IDENTITY_SUSPECT and no IDENTITY_UNVERIFIED at the decision. A footprint-consistent
   identity is provisional, so ΔU is computed only for decisions where provisional acquisition
   preceded the loss.
3. **Feasibility.** At least one candidate passes L2 with a view gain. A closed-loop arm also needs
   L3 = VALIDATED. Today L3 is never VALIDATED, which is why every closed-loop stage is BLOCKED.
4. **Risk.** The upper confidence bound of P(collision | h, REACQUIRE) must not exceed the lower
   bound of P(collision | h, CONTINUE) by more than a declared margin δ_risk. δ_risk is fixed in the
   preregistration and is not traded against capture.
5. **Validity.** The map is fresh, the belief is valid, and the input validates.

**Decision among admissible proposals.**
- PROPOSE if LCB(ΔU) > 0.
- CONTINUE if UCB(ΔU) < 0.
- ABSTAIN otherwise. ABSTAIN is a recorded outcome, not a safe command.

**Secondary terms.** Time to capture, mode switches and planner latency are reported as separate
endpoints. They do not enter U unless a later preregistration declares a lexicographic tie-break.

**C_switch.** A declared prior fixed in the preregistration, for example the probability-equivalent
of one mode switch. It is never fitted to outcomes.

## B. Counterfactual data contract

### B.1 What a true mid-episode branch would have to snapshot

| Item | Where | Restorable today? |
| --- | --- | --- |
| Vehicle and target rigid-body pose and velocity | Isaac Gym root states | settable, but not a full snapshot |
| PhysX contact and solver caches | PhysX internals | **NO** |
| Motor and Lee-controller buffers | `aerial_gym/control` | NOT_TESTED |
| Altitude-hold PI integral `_z_err_integral` | T:1608, T:4998–5002 | tensor, settable (NOT_TESTED) |
| Riskcap / arc filter state, previous action | navrl_task | NOT_TESTED |
| Actor observation and RMS normaliser state | rl_games player | frozen RMS (`642fc9f7…`); per-step state NOT_TESTED |
| Tracker mean, covariance, age and active flag | navrl_perception P:640–681 | tensor, settable (NOT_TESTED) |
| Target motion intent (waypoint, speed, RNG) | target-motion module | NOT_TESTED |
| Warp/BVH sensor buffers | warp sensors | rebuilt per step (NOT_TESTED) |
| RNG streams: torch, numpy, python, warp | process | partially keyed by `FixedNRandomness` (`tools/h_r3_p02_rng.py:39`) |
| Shadow state: history, map, machine | `motar_eval/q02` | yes, pure Python (CPU-tested deterministic) |

The full snapshot is **NOT available**, so the design does not depend on it.

### B.2 Design: fresh-process common-prefix replay

For a branch at decision b of an episode, run CONTINUE and REACQUIRE(k) as separate fresh
processes. Each resets the same seed and configuration and executes F up to b. At b, one arm keeps
F and the other hands the command to the executor. That executor is what makes D1 BLOCKED.

**Integrity checks.** A pair is void if any check fails, and is never salvaged:
1. **Prefix digests.** Per-decision host-trace digests (Q01-V2 `host.jsonl`) are equal for every
   row up to and including b, for that env.
2. **Replay coupling.** In HR3-P01 the coupling broke at the first arm-dependent reset in *any* env,
   because a shared stream shifted every env's noise. D1 therefore uses N1 processes, or N > 1 with
   per-env keyed randomness and per-env prefix digests.
3. **Exogenous randomness.** Target motion, resets and waypoints are drawn through
   `FixedNRandomness` by logical coordinates, so they do not depend on the branch's own RNG use.
4. **Numerics profile.** It is recorded and identical across arms: cudnn.benchmark, TF32, and
   whether rl_games enabled TF32 (known cdist hazard, about 2 m error at bar counts divisible by 4).
   Each batch includes an S0 replay control, a CONTINUE vs CONTINUE pair that must match after b.
5. **Bindings.** Source, configuration, checkpoint (`f7022139…`) and RMS SHA are bound in the
   manifest.
6. **Terminal-step rule.** Trace index i at a terminal step holds the post-reset observation and
   never counts as a detection.
7. **Outcomes.** They are computed evaluator-side and joined by record ID. Labels never enter the
   online modules.

**Limit.** Post-branch dynamics are deterministic but sensitive. One pair is one draw, not an
individual causal effect. Estimation therefore uses repeated pairs per (seed, branch time,
candidate) under keyed exogenous jitter, such as target speed or waypoint keyed by the branch
index.

### B.3 D0 data contract (CONTINUE only)

**Per decision.**
- the Q02-A evidence record (`Q02A-RECORD-3`) and the 19 Q02-B features;
- the five shadow decisions (policy, decision, reasons);
- the Q03 map validity and candidate summary;
- the Frozen F action and its diagnostics;
- host runtime;
- a packet subsample under a disk cap.

**Evaluator-only labels.**
- true target pose;
- line of sight to the camera and LiDAR;
- FOV status;
- occlusion by obstacle meshes;
- detector true or false positive and identity correctness;
- the episode outcome.

**Conditions never pooled:** oracle proxy, error-injected proxy, image-based detector.

**Disk.** About 5.8 GB was free on `/` on 2026-10-22. The output cap and a stop below 3 GB are
mandatory, and packets must be subsampled.

## C. Statistical design (for Q04-C, and for D1 where applicable)

**Units.**
- An *episode* is one fresh reset to terminal or cap.
- Episodes are paired across arms by (seed, episode index), which gives an identical scene and
  exogenous stream.
- The independent unit for inference is the **seed**. Episodes within a seed are clustered.

**Eligibility, decided on the common prefix only.** An episode is eligible if, on the prefix shared
by all arms, the target has been provisionally acquired and a loss is confirmed. This point precedes
every arm-dependent decision, because no trigger can act before a confirmed loss. The prefix digest
must match across arms at the eligibility decision.

The following are **never conditioned on**, because each is post-intervention:
- loss duration;
- whether an arm triggered;
- reacquisition;
- any later event.

Conditioning on any of them is post-intervention selection bias. For example, "losses longer than
1 s" excludes the episodes in which H3 succeeded early.

**Analysis approach.** All eligible episodes are analysed by arm, whether or not that arm proposed.

**Primary endpoint.** Collision-free capture per eligible episode.

**Secondary endpoints**, each with its own stated denominator:

| Endpoint | Denominator |
| --- | --- |
| Reacquisition | eligible episodes |
| Time to reacquisition and time to capture | **all eligible episodes**, right-censored at the cap (Kaplan–Meier / RMST). Never conditioned on success, because that is survivor selection (Opus review P2-7) |
| Collision rate and out-of-bounds rate | all episodes |
| Mode switches | per episode |
| Host latency | per decision |

Reacquisition is reported separately from capture and is never substituted for it.

**Contrasts and correction.**
- Four seed-paired contrasts: BOIA − F0, BOIA − H1, BOIA − H2 and BOIA − H3.
- Per contrast: a seed-cluster bootstrap 95 % CI of the paired difference in rate, and an exact
  sign-flip test over seeds.
- McNemar on discordant episodes is a sensitivity analysis only, because it ignores clustering.
- Holm correction across the four contrasts.
- Preregistered non-inferiority or superiority margins.
- Per-density and per-speed results are exploratory.

**Held fixed across arms.**
- Frozen F checkpoint;
- detector lineage;
- tracker;
- local map;
- planner;
- executor;
- state-machine gates and thresholds.

Only the trigger differs. Detector variants are separate experiments.

**Sample size.** Not fixed here. With seed as the unit, an exact sign-flip test over 8 seeds has a minimum two-sided p of 2/256 ≈ 0.0078, so Holm over four contrasts leaves almost no power. The seed count must be planned explicitly. The confirmatory B3−B2 design recommended after the HR3-P01 pilot needed 141 matched events,
which shows the order of magnitude. The preregistration will set the number of seeds from a pilot
discordance rate, with a cap.

## D. Model selection plan for E[ΔU | h] (no model is chosen)

| Candidate | Data need | Calibration | OOD behaviour | CPU latency (budget ≤ 1 ms per decision; the pipeline itself is about 80 ms) | Interpretability |
| --- | --- | --- | --- | --- | --- |
| Calibrated logistic regression on paired outcome differences or arm-specific outcomes | lowest (≈ 10–20 events per parameter, so several hundred pairs for 19 features) | good with Platt/isotonic on held-out seeds | extrapolates linearly; needs an explicit OOD gate | µs | high |
| Gradient-boosted trees | moderate | needs post-hoc calibration | flat outside the training range (silent) | µs–sub-ms | medium (SHAP) |
| Small MLP | moderate–high | needs temperature scaling | unpredictable outside the range | sub-ms | low |
| Temporal GRU over decision history | highest (sequences) | needs recalibration | unpredictable | sub-ms–ms | low |
| Conservative threshold rule with abstention on LCB(ΔU) | uses any of the above | inherits | abstains when uncertain | negligible | high |

**Selection criteria, in order.**
1. Enough D1 pairs for the model class.
2. Calibration on held-out seeds: reliability curve, Brier score, ECE, and coverage of the
   interval on ΔU.
3. OOD behaviour on a held-out density or target speed. The rule must abstain more there, not
   propose more.
4. Latency within the budget.
5. Interpretability.

**Process.**
- Model choice is preregistered before D1 data are examined.
- Hyperparameters are chosen by seed-grouped cross-validation only.
- The Q04-C held-out seeds are never used for selection.

BOIA's timing policy remains **not learned** until all of the above have been done.
