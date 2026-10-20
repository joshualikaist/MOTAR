# BOIA: evidence-aware mission arbitration — G4-R proposal

**DESIGN_ONLY / NOT_IMPLEMENTED / NOT_PREREGISTERED.** Base `cdfaffd28534cd37693a6cd678de9766bef4bd25`, branch `codex/mt-p1`, 2026-10-20. The user authorizes separate C02 CPU work and this mathematical design, not activation of a new arbitrator. Existing06–09/13 and their historical evidence are preserved. RGB-D Camera + LiDAR, Frozen Policy F, actor898, tracker5s, governor and controller remain the reference contract. B03/B04 remain BLOCKED; nominal C02 success is not physical qualification. **G5 HOLD.**

## 1. Problem and change from current BOIA

The question is whether a short intervention, selected before or during target observation loss, improves eventual collision-free capture relative to continuing Frozen F. Visibility is an intermediate variable, not the mission objective. The current implementation ranks held-command rollouts using one Gaussian track and point-valued detection forecast, then thresholds modeled cost advantage (`planner.py:99–118`, `arbitration.py:115–127`). Its sensor access does not provide association evidence or measurement timestamps. A low predicted detection probability can therefore mean occlusion, unsupported sensing or wrong identity.

The proposed change is a decision architecture with three distinct objects: a set of sensor-consistent scene/identity hypotheses; an independent next-cycle safety certificate; and a calibrated lower bound on paired intervention benefit. It allows an explicit utility ABSTAIN while retaining the safety obligation. This is more than adjusting weights or thresholds, but its individual principles and robust baseline comparison are prior art; see [NOVELTY_REVIEW](NOVELTY_REVIEW.md).

## 2. Inputs, records and unavailable information

The proposed pure CPU interface is `propose(evidence, own_state, a_F, history, candidates, models, calibration, safety_contract) -> ArbitrationResult`. Names are design interfaces, not functions now present. Inputs/outputs are N-first, with independent env/episode/decision identifiers; distances m, velocity m/s, time s, angles rad, poses in W and calibrated sensor frames. All timestamps share a declared clock.

| Proposed record | Required contents and validity |
| --- | --- |
| `SensorEvidence` | Actual RGB-D detections/depth support and uncertainty; LiDAR first hits, valid/no-return distinction, beam directions and support assumptions; capture/arrival timestamps; pose-at-capture and uncertainty; actual detection intrinsics/resolution/threshold; sensor-to-vehicle extrinsics and depth convention. |
| `AssociationEvidence` | Measurement-derived target-match likelihood intervals, camera-confirmation age, fused update source, competing foreground/false-match hypotheses; never semantic GT. Detector confidence is not automatically an identity probability. |
| `EvidenceBelief` | A finite hypothesis set, optional credal weights, conditional track distributions, unknown region support, provenance/expiry and calibration-domain flags. |
| `SafetyCertificate` | Candidate prefix plus backup envelope status, model/observation assumptions, error-bound availability, boundary version/time and `physical_qualified` flag. |
| `BenefitCertificate` | Paired means, uncertainty-set support, MC error allowance, held-out model-error allowance, switching cost, lower bound and validity scope. |
| `ArbitrationResult` | Outcome, candidate or optional certified backup, evidence/safety reason codes, new explicit state, certificate references and timing. No predicted capture/crash outcome declaration. |

Current `OnlineInputs` (`types.py:95–112`; `access.py:69–82,102–119`) lacks most new evidence. A separately approved whitelist amendment is required. Rendered semantic masks, target/obstacle GT, E-L0, forensics and critic state are forbidden online. Sensor-predicted masks and measured depth would need their own explicit provenance contract. Do not repurpose the broad forbidden `detect_*` namespace to smuggle semantic labels; distinguish numerical calibration from privileged channels in the approved schema.

Calibration constants cannot establish time alignment. Transform each measurement using its capture-time pose and declared SE(3) extrinsic. Require residual clock/pose errors within separately calibrated bounds. Current yaw-only sensor geometry is usable only inside that declared mounting model; it does not supply full body attitude or an arbitrary camera extrinsic. Missing bounds are `UNAVAILABLE`, not zero. Retain all valid first hits in the collision evidence.

## 3. Evidence-conditioned hypotheses and identifiability

Let `H_k` retain every hypothesis not excluded by the approved measurement likelihood/error bounds:

- `T`: target surface explains a first hit and sensor-predicted target support is consistent;
- `O`: a foreground obstacle explains the hit, including a target hidden behind it;
- `A`: the tracked identity is wrong or association is ambiguous;
- `U`: the relevant region or measurement is unobserved, stale or outside calibrated support.

These labels describe scene interpretations, not online truth labels. They may coexist: a genuine target and a foreground obstacle are not mutually exclusive events. Each member is therefore a complete scene hypothesis with its own target-assignment and occupancy variables, rather than four mutually exclusive probabilities forced into a Gaussian. Keep an unknown/other alternative whenever the hypothesis family is not exhaustive.

For each scene h, retain conditional continuous model sets `Theta_h`: pose/extrinsic/range errors, target-motion uncertainty, detector-reliability intervals and occluder extent consistent with declared evidence. If mixture probabilities are available, keep a credal set `W={w>=0,sum(w)=1,l_h<=w_h<=u_h}` calibrated from separate data. Without that calibration, use the scenario union and no invented posterior weights. The full set is `Theta(E_k)=union_h Theta_h`, or the declared credal extension. Do not multiply confidence scores as if independent.

The6m centre/5.8m first-hit counterexample remains mandatory. If target and foreground scene hypotheses generate the same permitted observations, identity is unidentifiable: retain both, preserve the hit, and allow a detection interval or ABSTAIN. Do not erase the hit, extend its ray to sensor far range or force true detection probability to0/1. Model probability bounds may include0 or1; that is not an assertion that detection is impossible/certain.

B03 imposes a separate observation limit. Finite zero-width rays cannot certify a continuous free volume between them without minimum-size/beam-support assumptions. Multi-view evidence is usable only with timestamp/pose/error/dynamic-age bounds. Its absence reduces coverage and feasible-candidate counts; it does not authorize optimistic filling.

## 4. Two gates: safety and utility evidence

Define an explicitly observation-supported free set `F_obs(t)` under the approved B03 contract. Ambiguous occupied support is retained; possible occupancy is unioned across relevant hypotheses and certified free support is intersected. Identity confidence can change target utility, but cannot delete a collision first hit or expand physical free space.

For candidate g, execute only its next decision prefix `[0,Delta]`, Delta=.1s, followed by a specified backup controller until rest. The nominal envelope is

`W_g = union_t [p_g(t) + R_g(t) B + E_g(t)]`,

over the prefix AND complete backup. Reaction is included once. Require `W_g subset F_obs(t)` and input/dynamic bounds throughout; an available complete horizon may be checked additionally. A horizon-end stop alone is insufficient. The user’s `(v0=(2,0),v_cmd=(0,2),point=(.97,.2))` counterexample must be rejected at the next-cycle check even when the full-horizon endpoint passes.

Let `S_k` be candidates with a current valid certificate. Certificates explicitly distinguish `MODEL_SAFE`, `MODEL_UNSAFE`, `UNKNOWN`, `INVALID` and `PHYSICAL_QUALIFICATION_NOT_AVAILABLE`. A planar assumed free boundary supports a nominal result only. Unmeasured tracking, sensor or latency bounds prevent a physical certificate. C02 does not fix B03/B04.

Separately define `V_k subset S_k`: candidates for which forecast/benefit evidence is valid. This requires timestamp/calibration support, retained association hypotheses, bounded forecast error and the correct candidate/continuation distribution. Failure of V never removes a candidate from S or disables a necessary safety action. Safety UNKNOWN is not MODEL_SAFE.

## 5. Mission surrogate, paired comparison and switching cost

Use a declared finite-horizon loss `L(g;theta,z)` with bounded, normalized components: closing progress; identity-correct observation retention/reacquisition delay; conditional track uncertainty; effective physical command effort; clearance/backup reserve. Horizon, normalizers, clipping bounds and weights must be fixed before calibration outcomes. Capture remains an actual simulator endpoint, not inferred from closeness. The old four-action smoothness term is not reused as physical effort: a_z is overwritten by altitude PI. A future explicit command-space revision requires approval.

A possible implementable loss interface is `loss_features(g,theta,path) -> (progress,loss,delay,uncertainty,effort,reserve)`, followed by a declared bounded weighted sum. Reacquisition delay is truncated at the forecast horizon with a distinct no-reacquisition indicator; absence of a detection does not become a fabricated successful delay. Uncertainty reductions require the declared observation model, not identity-confirming GT.

For each retained parameter scenario b and independent trajectory path m, use the SAME exogenous realization to compute `d_bm(g)=L(F;b,z_m)-L(g;b,z_m)`. Compare paired differences, not independent worst cases for F and g. Candidate selection over K requires simultaneous uncertainty control.

The current held-command rollout is not the closed-loop value of Frozen F. Define the future evaluation option as: apply candidate g for Delta, then resume the same Frozen F continuation with its actual updated action history. The comparison option applies F for Delta with the same continuation rule. Offline paired branch data must calibrate the discrepancy between the held-command score difference and these realized finite-window option losses. Until then the score is only `HELD_COMMAND_SURROGATE`, not expected capture improvement. No learned value model or new policy training is authorized here.

For a finite, declared B-scenario approximation with `d in [-D,D]`, a conservative MC allowance is

`eps_MC = D sqrt(2 log(2 K B / alpha_MC) / M)`.

This uses independent paths m; time points on a path are not independent samples. Common randomness across candidates is allowed. A finite scenario minimum is a true robust bound only if the scenarios exhaust Theta or a proven interpolation remainder `eps_set` covers the omitted set. Otherwise label it `SCENARIO_SENSITIVITY`, and forbid a confidence-certified utility intervention.

Define

`A_low(g) = min_b mean_m d_bm(g) - eps_MC - eps_set - b_cal(g,E_k) - c_switch(g,history)`.

`b_cal` is a held-out, simultaneously calibrated upper allowance for score-to-realized paired-loss error in a supported evidence stratum. It must include continuation/actor-history/model mismatch. A seed-block residual calibration may be proposed, but arbitrary conditional or repeated-time coverage is not claimed. Unsupported strata or unavailable allowances yield ABSTAIN. A lifetime probability statement would additionally need a declared time-uniform method or error-spending schedule; a per-decision confidence level is not a mission guarantee.

`c_switch` is a nonnegative normalized estimate/bound of effective command change, measured scheduling latency and hand-back disruption. It is zero only for an identical continuation with no switch; unavailable switch costs are not silently zeroed. Do not double-count these same terms in L: L uses within-option effort, while c_switch covers transition overhead absent from that option model.

Even a rigorous MC allowance can be uninformative. For example, D=2,K=20,B=4,M=32,alpha_MC=.05 gives eps_MC about1.42 in the chosen loss units. Report resulting abstention instead of weakening confidence after seeing the result. Increasing M changes compute and needs a preregistered choice. No numeric thresholds are fitted in this design.

## 6. Decisions, precedence and explicit state

| Outcome | Required condition | Action meaning |
| --- | --- | --- |
| `KEEP_F` | F in S; valid utility comparison finds no supported net gain | F remains in control. |
| `INTERVENE` | F and g in S, g in V, valid `A_low(g)>tau_on>0`, and declared endgame guard passes | Apply only g's next Delta prefix; retain its valid backup. |
| `ABSTAIN` | F in S but utility evidence, model set or benefit calibration is unsupported | Execute certified F; record a utility refusal, not a safety bypass. |
| `SAFETY_OVERRIDE` | F not in S, but a current certified backup or alternative exists | Execute a certified backup first; otherwise choose a safety-ranked candidate without target utility. |
| `NO_CERTIFIED_BACKUP` | No current candidate/retained backup has a valid required certificate | No certified action exists. Return this explicit emergency result; do not relabel a zero command as successful SAFE_HOLD. |

Before integration, the executor must define a separately qualified response to `NO_CERTIFIED_BACKUP`; the pure design returns no certified action plus any explicitly marked best-effort command. It cannot send an undefined action or claim a safe abort. Distinguish `COVERAGE_INSUFFICIENT`, `FINITE_SET_EMPTY`, `MODEL_STOP_INFEASIBLE` and `INVALID_INPUT`. None alone proves unavoidable collision.

For a safety override, prefer a still-valid previously committed backup. Otherwise rank S by worst-case clearance/backup margin and effective command change, with deterministic tie-breaking. Do not use a stale target mean to rank an identity-unconfirmed emergency. All such choices are conditional on the model/sensor certificate, not a real-world safety theorem.

Maintain explicit state: current owner/candidate, env episode/time, last executed command, intervention duration, evidence age, backup validity and calibration version. Per-env RNG is keyed independently of N_live and other environments' resets (ON-02); terminal rows are excluded before copying/validation (ON-01). A common prefix copies full allowed state/history, not just pose. Trace the actual returned command and source/config/RNG/state provenance.

Use prospective hysteresis `tau_off<tau_on` and a declared minimum hold as a switching-cost device. Recompute safety and evidence every decision. Safety failure, invalid evidence, invalid calibration or terminal status immediately overrides dwell. A supported harmful advantage must not be forced for minimum duration. Fallback/hand-back requires F itself to be in S. Near capture, retain the current INTERCEPT safety-only behavior until paired endgame calibration supports a distinct rule; a progress guard is not a guarantee of timely capture. Never-acquired search remains out of this first proposal.

### Proactive timing and remaining intervention opportunity

Add a finite wait-then-intervene option index j, with a preregistered grid `t_j=j*Delta` inside the short horizon. An option follows the declared F surrogate for t_j, applies g for one Delta, then uses the common continuation rule. Evaluate paired benefit `A_low(g,j)` relative to the same uninterrupted-F option. Propagate only existing observation support with its age/error bounds: future motion does not magically acquire unseen RGB-D/LiDAR evidence. Prediction validity at a later time may be UNKNOWN.

At each waiting knot define `O_j={g: F-wait prefix and candidate-prefix/backup valid through t_j, utility evidence valid at t_j, A_low(g,j)>tau_on}`. The entire predicted F waiting prefix must remain inside the propagated observation-supported set; a feasible candidate only at t_j cannot certify a preceding unsafe wait. If O_0 is empty or unsupported, the current opportunity window is EMPTY, not a future closing deadline. Later nonempty masks are advisory future options. Otherwise the current window ends at the first knot after j=0 with an empty or unsupported O_j; later reappearance does not bridge that uncertified gap. If no such knot exists, its end is right-censored by the forecast horizon. Store the complete feasible mask, not just a candidate count or a falsely precise continuous deadline. `OPPORTUNITY_CLOSING` applies only to a nonempty current window and means its end is no later than the calibrated processing/transfer delay plus one decision period; missing delay bounds invalidate this flag.

Choose the best supported `(g,j)` by its paired bound with deterministic tie-breaking. Execute g now only if j=0, its current safety certificate remains valid and its net bound exceeds the switching threshold; otherwise keep certified F and re-evaluate next cycle. A closing window cannot authorize an unsupported or negative-gain intervention. A future option is advisory, never a committed safe future action. Retain the independent backup on every executed prefix.

This option structure permits intervention while the target is currently visible, based on the predicted loss of an evidence-supported manoeuvre opportunity, rather than requiring a present LOS break or treating an unreliable forecast0 as PRE. It also supports deliberately waiting when early detours are costly. The opportunity window, timing policy and delay calibration are new design work for MOTAR, **not a claimed new viability/POMDP principle**. Held-F approximation and candidate incompleteness still limit its meaning. Compare against immediate and reactive variants in the experiment draft.

## 7. Pseudocode and complexity

```text
propose(E, x, aF, history, models, calibration):
    handle current terminal rows first; clear only their explicit state
    validate timestamps/frames/schema per live env; no cross-env validation failure
    hypotheses = retain_all_sensor_consistent_scenes(E)  # includes UNKNOWN/OTHER
    collision_evidence = preserve_first_hits_and_union_possible_occupancy(hypotheses)
    Gamma = candidates_with_F_and_backup(aF, x, permitted_belief)
    S, certificates = next_prefix_and_backup_screen(Gamma, collision_evidence)
    if F not in S:
        if current_committed_backup_is_valid: return SAFETY_OVERRIDE(backup)
        if S nonempty: return SAFETY_OVERRIDE(safety_rank(S))
        return NO_CERTIFIED_BACKUP(reason, no_certified_action)
    if never_acquired_or_uncalibrated_endgame: return KEEP_F(scope_reason)
    V, Theta, valid = utility_evidence_gate(E, hypotheses, calibration, S)
    if not valid: return ABSTAIN(F, reason)
    shared_paths = per_env_paths(env, episode, step, sample)  # same for F/g
    for supported (g,j) in wait_then_intervene_options(V):
        A_low[g,j] = paired_bound(F, (g,j), Theta, shared_paths) - switch_cost(g,j)
    window = first_contiguous_supported_opportunity_window(A_low, certificates)
    (gstar,jstar) = deterministic_argmax(A_low)  # bounds include option selection
    if jstar==0 and valid_hysteretic_gain(gstar) and endgame_guard(gstar):
        return INTERVENE(gstar.first_prefix, certificates[gstar].backup)
    return KEEP_F
```

With J waiting options, dominant current-style forecast cost is O(N J B K M H P); rollout O(N J K H S); capsule/boundary safety O(N J K L T_backup), with L boundary features and a declared bounded backup discretization/analytic tail. Small-matrix belief work is O(N J B K H). Replace K by the tested option count J*K in simultaneous MC bounds. Streaming J/B/K/M chunks bounds memory; it does not reduce total work. A continuous uncertainty optimizer needs separate complexity/certification. There is no claim that this fits10Hz or5ms. Existing N128 CPU one-shot core10.7210s/trace2.5239s is evidence of a large implementation gap; GPU latency/VRAM are NOT_TESTED. Resolve the07 GPU-target versus09 CPU-gate inconsistency prospectively.

## 8. Independent oracle plan and failure cases

No new arbitration code/tests run in this package. Future CPU acceptance must include: indistinguishable target/foreground observations retain both hypotheses; utility ABSTAIN never enlarges S; unsafe F cannot pass through ABSTAIN; next-cycle counterexample rejects; same uncertainty affects both paired options; candidate-selection uncertainty is simultaneous; unsupported calibration cannot yield INTERVENE; switching cost reverses a marginal gain; terminal/reset/batch permutation leaves other env paths unchanged; no certified backup remains an emergency; effective effort ignores overwritten a_z; identical model/action options have zero paired physical benefit.

For a numerical paired-regret oracle, use two explicit scenarios with `(J_F,J_g)=(10,9),(1,0)`: paired robust gain is1, whereas `min J_F-max J_g=-8`. Add a third unidentified hypothesis `(0,2)` and the robust gain becomes-2, correctly blocking intervention. This is an arithmetic fixture and prior-art principle, not an experiment result or new theorem.

Expected failures include unidentifiable association, poor beam/depth coverage, stale multi-view occupancy, covariance miscalibration, unseen target manoeuvres, detector shift, no feasible narrow-gap capsule, conservative bounds giving zero interventions, wrong hand-back model and missed computation deadline. Report abstention/coverage/feasibility and failed mission endpoints. If the calibrated score adds no useful discrimination over reactive recovery and the same safety filter, redesign or reject the method; do not tune against confirmatory seeds.

Potential contributions are at most the three conditional candidates in [NOVELTY_REVIEW](NOVELTY_REVIEW.md). Implementability of this specification is not approval to implement it. Next authority required: the measured-evidence contract and independent CPU evidence/benefit kernels, followed by review; B03/B04 resolution and G5 remain separate gates.
