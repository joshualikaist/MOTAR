# G4-R experiment draft: mission benefit and mechanism separation

**DESIGN_ONLY / NOT_PREREGISTERED / NO EXECUTION AUTHORITY.** Date2026-10-20, base `cdfaffd28534cd37693a6cd678de9766bef4bd25`, branch `codex/mt-p1`. This prospective draft clarifies the mechanisms in historical09 without editing it. No GPU, simulator, learning, new arbitrator, raw replay/calibration, commit or push follows from this document. B03/B04 BLOCKED; physical C02 and G5 remain HOLD even if nominal CPU counterexamples pass. Method and claim limits: [ALGORITHM_PROPOSAL](ALGORITHM_PROPOSAL.md), [NOVELTY_REVIEW](NOVELTY_REVIEW.md).

## 1. Estimands fixed before sample size

The principal scientific question is the effect of an executable, evidence-aware intervention policy on **episode-level collision-free capture**, measured from the common episode start. Let `Y_cf(a)=1` only when the actual simulator reports capture without a preceding or simultaneous collision under arm a. Preserve original capture/crash flags and termination priorities; record simultaneous flags separately rather than rewriting historical outcomes. Predicted closeness never defines capture.

Primary proposed contrast: equal-weight seed-cluster difference `E_seed[Y_cf(F5)-Y_cf(F1)]` on the prespecified moving/initially-visible160/205-bar conditions. This compares the new mission layer against the SAME qualified mandatory safety layer. Secondary comparisons against F2/F3/F4 determine whether it adds more than reactive or predictive planning. This is a prospective proposal, not an effect claim or finalized confirmatory family.

The ultimate comparison to literal unchanged Frozen F (F0) requires its own approved execution protocol; see§2. Capture superiority and a prespecified crash-harm guard are separate hypotheses. Capture non-inferiority alone cannot establish improved mission success. A lower crash rate achieved by waiting forever is not success: timeout, path/time cost and no-backup/abort rates accompany capture.

Incident recovery has a different estimand. Create a common prefix with the same pre-intervention state, sensor history, actor/tracker/BOIA history and per-env exogenous streams; branch BEFORE the assigned intervention, at a stopping time defined by prefix information. Compare one-step action options or complete policies from that shared state. Treatment may change subsequent geometry/sensing; preserve that change. Do not select/match on future path, actual post-branch occlusion onset, successful reacquisition or realized target motion.

The original09 primary crash CIF after each arm's own post-acquisition onset compares arm-specific incident populations, because control can change onset incidence/timing. Retain such per-policy curves as descriptive/secondary, including onset rates and denominators. They do not alone estimate a causal recovery effect for identical occlusion events or overall mission benefit.

## 2. F0–F5 mechanisms, not weight labels

Every proposed executable variant keeps the same Frozen F checkpoint, actor898, RGB-D+LiDAR lineage, tracker expiry, governor/controller, action limits, candidate set where applicable, observed-space contract and mandatory prefix-plus-backup filter. Utility variants cannot weaken that filter. No unknown-to-free fill or target-ID-based obstacle deletion is an ablation.

| Arm | Exact mechanism to specify in a future separate interface | Excluded mechanism / comparison purpose |
| --- | --- | --- |
| F0 | Literal Frozen F with its original governor/controller and sensor contract; source/checkpoint unchanged. | Reference, with no newly inserted BOIA override. Preserve historical/raw F0. New GPU execution is NOT proposed while it would require removing the required hard gate; qualification and an explicit baseline abort protocol are prerequisites. |
| F1 | F plus the common mandatory safety gate. **Utility gain path disabled by an explicit boolean/enum**, not zero weights. Safe F is returned exactly. Unsafe/unknown F uses a certified safety-only alternative/backup; otherwise NO_CERTIFIED_BACKUP. | No P_det/PRE trigger, target-progress ranking, uncertainty utility, opportunity timing or gain threshold. Emergency ranking uses only qualified safety margins/command change. This is the executable safety-only reference. |
| F2 | F1 plus reactive recovery enabled only after an actual permitted camera-loss event after acquisition. Use the same candidate pool and a fixed current-geometry/mean-target viewpoint rule, with identical safety gates. | No future visibility/occlusion forecast, no covariance sampling and no robust benefit threshold. While currently observed, safe F remains unchanged. |
| F3 | Predictive visibility planning over a deterministic CV mean: set target sampling uncertainty/process increments to zero in the **utility model only**; no expected-covariance term. Select the minimum declared expected/surrogate utility candidate on every eligible decision, including F; safety unchanged. | Removes belief uncertainty genuinely, not merely `w_u=0`. Sensor/pose safety error bounds remain intact. No robust/selective gain criterion. |
| F4 | Same predictive planner/candidates/mission feature definitions as F3, but full calibrated target-belief samples and expected-observation uncertainty propagation. Select the utility-best safe candidate each eligible decision without a positive baseline-benefit certificate. | F3→F4 isolates the belief/uncertainty feature package; it is not an unconditional physical departure from F if F itself wins. No robust selective arbitration or wait-option timing. |
| F5 | Full proposed evidence hypotheses, separate safety/utility gates, paired lower-bound benefit, switching cost and supported wait-then-intervene timing. | Proposed method; all parameters/models/data splits fixed in a separate preregistration before evaluation. |

The F0/F1 distinction must not be hidden. Adding the SAME overriding C02 filter to F0 makes it F1, so it cannot be presented as a separate unchanged baseline. A globally enforced abort/interruption monitor can preserve F's command proposal but changes the episode protocol; label that `F0_WITH_ABORT_PROTOCOL`, record every trigger as a failure/abort outcome, and do not silently equate it with historical F0. Until an acceptable protocol is approved, literal F0 comparisons are CPU/shadow or historical descriptive only; initial closed-loop method comparisons use F1. No intentionally unguarded action arm is authorized by this draft.

The existing candidate19 translates0.6m/s; label it “heading translation plus target yaw.” A new stationary-yaw comparator would need a separate candidate/API amendment. All arms inherit the same explicit near-capture/never-acquired scope; any endgame extension is a separate factor, not hidden inside F5.

## 3. Additional mechanism contrasts

| Contrast | What changes | What stays fixed / admissibility |
| --- | --- | --- |
| Evidence reliability ON/OFF | ON retains supported scene hypotheses and refuses unsupported utility claims; OFF uses a declared plug-in target/foreground interpretation for utility. | Identical raw measured packet, collision first-hit preservation, occupancy/free-space certificate and hard filter. OFF may not invent identity or change safety. Any utility model outside admissible support is evaluated offline until separately approved. |
| Robust advantage ON/OFF | ON uses paired scenario/error lower bound; OFF uses the paired point expected score difference with the SAME switching cost and threshold calibration procedure. | Same evidence inputs, candidate pool, target samples and hard filter. No hidden extra sensor or model advantage. |
| Evidence×robust2×2 | Four combinations above. | Reveals whether evidence refusal, robust scoring or their interaction changes mission outcomes; choose a limited confirmatory contrast family before data. |
| Next-cycle backup ON/OFF | ON includes actual next-Delta prefix and its complete backup; OFF reproduces horizon-end-only screening. | **OFF is CPU/offline counterfactual only**, including the user counterexample. No GPU closed-loop arm drops the required next-cycle safety condition. |
| Immediate / predictive timing | Immediate uses the same evidence/benefit rule with wait grid `{0}`; predictive uses the declared wait grid/opportunity window. | Same samples, runtime budget, loss and hard safety. Reactive F2 is a separate actual-loss trigger. This isolates timing rather than just PRE labels. |
| Selective / unconditional selection | Selective needs a valid positive net paired-bound certificate; unconditional picks the expected/specified utility-best safe candidate every eligible step. | Both keep the SAME hard filter, utility-evidence refusal and candidate pool. “Unconditional” does not require selecting a worse non-F action. Implement an explicit selection enum, not `tau_on=0`. |
| Association ambiguity | Independently defined H1 true-target/H2 foreground/H3 false-association cases, including observationally indistinguishable pairs and the5.8/6m fixture. | Online packets are measurement-derived and may be identical across paired scenes. Evaluator-only truth measures errors. Keep the .53m foreground regression and near/far geometry controls. |
| Camera latency | Declared capture-to-arrival delay/jitter with capture-time pose alignment and a held-out stress level. | Same actual RGB-D+LiDAR contract; LiDAR timing/update source recorded separately. Latency is not represented by changing target GT online. |

F1 cannot be defined by `w_l=w_u=0`: current PRE remains visibility-dependent and progress/clearance gain can still switch (`planner.py:111–118`, `arbitration.py:115–120`). F3 cannot claim “without belief” merely by removing U. `tau_on=0` is rejected by current code and would not itself mean always switching. All required new mode switches/interfaces are **design only**; current planner/arbitration are unchanged.

## 4. Conditions, matching and information boundaries

Use70/115/160/205 bars; static and moving `E2_direct_l2_v1`; visible and hidden starts. Bar count is not measured physical density. Preserve each layout lineage and never pool `navrl_band` with a different placement lineage. Moving/visible160/205 is the proposed primary scope; the other cells test scope/robustness, with hidden acquisition reported separately because v1 does not search before acquisition. Use independently new seed sets; enumerate all historical seeds from run manifests rather than assuming an old exclusion list is exhaustive.

Episode-start comparisons match seed/layout and declared episode assignment before treatment. Branch comparisons require exact prefix state/history equality and independent per-env reset/MC streams. Other env terminations cannot alter live samples. Requalify the actual profile and preserve full exogenous streams; identical seed integers alone are insufficient. After intervention, action-induced sensor differences are expected and must remain.

Online inputs for all active variants are the same approved measured packet and own state. Branching may copy simulator state for evaluator-controlled counterfactual setup, but no privileged field reaches the algorithm. E-L0 identity/silhouette and GT target/obstacle geometry are joined offline by immutable event/time identifiers. Preserve provenance and separately record source/config/checkpoint/calibration/RNG/history hashes. A64-bit input digest is not a complete reproducibility record.

## 5. Metrics and denominators

| Family | Required definitions and reporting |
| --- | --- |
| Mission | Collision-free capture; original capture; collision; timeout; OOB; verified safe abort; NO_CERTIFIED_BACKUP and best-effort emergency counts. All randomized/assigned episodes remain in denominators; do not drop refused/deadline/failed episodes. |
| Time/efficiency | Capture cumulative incidence and restricted mean time without capture through a fixed horizon, retaining crash/timeout/abort as competing endpoints. Report all-episode path/time cost, and any success-conditional summaries explicitly as descriptive. Define efficiency as captures per total flight time/path cost, with seed-level numerator/denominator recorded. |
| Occlusion | Onset frequency/exposure, duration/censoring, FULL/CENTRE_ONLY/PARTIAL counts, FOV/range exits, visibility retention, identity-correct camera recovery, fused resumption separately, delay and false association. No effect claim for absent classes. |
| Reliability | Brier score and fixed-bin reliability/ECE with counts and cluster intervals; predicted-vs-realized identity-correct camera availability. Geometry visibility alone is not a detection label. Calibration/ABSTAIN coverage and confidence-set coverage accompany risk. |
| Safety | False-safe vs independent geometric oracle; false-block vs oracle **under the same declared model assumptions**; unknown/invalid fractions; prefix and backup feasibility; no-certified-backup cause; model rollout drift; actual collision outcomes. A model false-block rate is not real-world ground truth. |
| Switching/system | Intervention duration/frequency; actual action difference vs mere engaged status; hand-back actor-history drift; effective effort; processing/transfer latency; P95/P99 and worst observed decision time with deadline misses; peak VRAM only in separately approved future GPU runs. |

Reacquisition delay analysis starts at a common pre-treatment eligible branch or is explicitly policy-specific. Capture/crash/timeout are competing events, not censoring assumed independent of recovery. Genuine missing recordings and run-cap censoring are reported separately. Do not convert terminated post-reset frames into the preceding episode's sensor outcome.

## 6. Calibration and statistical analysis

Separate design/calibration/test seeds and scenes. Existing OCC-P1/P2/P01/P02/P03 data may motivate the problem and data availability; they do not become new confirmatory BOIA samples. Any raw calibration needs separate authorization and adequate timestamped fields. Tune evidence/model intervals, Q/R, switching costs and score-to-realized gap allowances on calibration data only; freeze them before held-out tests. Changes after an exploratory result require a new amendment and independent evaluation.

For belief calibration, compute timestamp-aligned NEES for the declared state dimension and NIS using `S=H P H^T+R`; do not substitute the R-normalized innovation from another method. Report association/gating selection effects and rejected measurements. Per-step examples are clustered within episodes/scenes/seeds. Report nominal coverage and seed-cluster confidence intervals; the historical “95% band in80% of cases” is a permissive gate, not proof of well-calibrated95% coverage. Do not revise it silently.

For benefit calibration, independently branch a common allowed prefix into candidate-prefix-plus-F versus F-prefix-plus-F continuations. Preserve action-history effects in the frozen actor. Compare held-command score differences to these actual finite-window option losses; calibration is not a GT critic or online capture oracle. A paired local score cannot guarantee globally improved episode return. Evaluate its reliability and then the complete policy's mission effect separately.

Seed is the inference cluster. Compute equal-weight seed effects, with preregistered density/motion/start weighting; use paired seed differences when the design truly supports them and cluster bootstrap or declared randomization/sign-flip tests. Model scene/episode dependence and report numbers of independent seeds, layouts, episodes and eligible events. Do not count thousands of decisions as independent trials. Small-seed p-value resolution is reported; intervals and effects remain primary.

Use Aalen–Johansen for explicitly defined competing-risk event curves where appropriate, with seed-block uncertainty and fixed event priorities/horizons. The tool's event contract must be checked against the new endpoint; frozen OCC-P2 analysis is not automatically valid for every new metric. Predefine multiplicity across the primary mission contrast, crash guard and any confirmatory mechanism/density tests, for example Holm within a declared family. Do not select the best density or hypothesis after outcomes.

No fixed confirmatory seed count is justified yet. Estimate event prevalence, seed-level effect dispersion and censoring under an independently preregistered exploratory profile, then perform a cluster-aware power/precision calculation. The old30-seed figure in09 is a planning assumption, not a new power result. Any exploratory cap and later sample count require separate approval; this draft authorizes zero runs. Rare-event calibration needs explicit class/bin counts and may remain insufficient.

## 7. CPU/GPU stage separation and go/no-go criteria

| Stage | Intended evidence | Present status / authority |
| --- | --- | --- |
| G4-R C02 CPU | Independent whole-body planar and next-cycle backup counterexamples; default-path noninterference. | Separately implemented/reported by the C02 owner; this draft makes no new test-result claim. Nominal model only. |
| Measured-evidence design | Resolve B03/B04 with explicit observability limits, identity/timestamp/SE(3) packet and ABSTAIN contract. | BLOCKED/PROPOSED; current whitelist insufficient. No source change here. |
| Proposed CPU benefit kernels | Independent finite-hypothesis/paired-bound/timing and outcome-precedence oracles. | Approval required; no new arbitrator implemented or tested here. |
| G5 eligibility | Sensor/free-volume/identity validity, per-env RNG/terminal isolation, NEES/NIS, benefit/detection calibration, plant drift and computation contract. | HOLD; unmeasured items are NOT_TESTED, not PASS. |
| Future GPU qualification | Frozen source/preregistration, import/checkpoint checks, bounded tiny profile, actual deadline/VRAM/trace and backup execution. | Separate approval required. No automatic launch. |
| Exploratory/confirmatory | Fixed independent seeds/arms/endpoints, compact evidence and raw SHA manifests, then a separate test family. | Not authorized; no automatic scale increase. |

The07 target is≤5ms on RTX3070;09 labels≤5ms as CPU. Resolve this device-specific inconsistency with an explicit prospective amendment. The recorded N128 one-shot CPU total13.2449s (core10.7210s, trace2.5239s) exceeds the literal CPU gate; it is not a GPU measurement. Future runtime includes evidence construction, all candidates/wait options, safety/backup, calibration lookup, transfer and logging. A slower method cannot be called10Hz from mean kernel time alone. A deadline miss requires a previously valid backup and a logged system outcome; without it, return NO_CERTIFIED_BACKUP rather than assume a safe zero command.

If conservative evidence/bounds eliminate interventions, report coverage/feasibility failure. If prediction adds nothing beyond current detection, drop the predictive component. If timing improves visibility but harms capture, reject that timing/utility design. If evidence/robust gates show no mission advantage over F1/F2/F4 at matched resources, the proposed mechanism is unsupported and the method novelty claim must narrow or be withdrawn. Repeated tuning on the test set is not a remedy.

No original source/preregistration/result is modified by this design. No unsafe backup-disabled arm, policy retraining, sensor-lineage change, planner activation or Git operation is implied.
