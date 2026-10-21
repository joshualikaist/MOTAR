# BOIA Research Master Plan

2026-10-21 KST · `codex/mt-p1` · G4-R7 living research entry point.
Committed C02 base: `8689efcd066d64e7b49cecf011e3bd8675380cdb`.
R1 research source: local `82ef02c340fba0baec18162ad33fc03dca7301d5`.
This plan preserves the original specifications, preregistrations, experiments and
receipts as historical evidence. It does not retroactively qualify them or authorize
GPU, simulation, training, production/controller changes, online activation or push.
**Current scope: passive evaluator observer/logger with a native Frozen F baseline (Q01-V2),
bounded launcher and CPU qualification completed; GPU execution requires separate approval. G5 HOLD.**
R0 is PASS_CONTROLLED; R1 is CPU_VERIFIED within its nominal declaration contract
(50 focused /252 combined BOIA tests PASS). Independent read-only review found no
blocker within the nominal typed CPU scope. Exact source/results belong to the R4 evidence
record. Physical qualification and online/C02 integration remain unavailable.
The [G4-R5 evidence](../../evidence/boia_g4r5_2026-10-21/SHADOW_READINESS.md)
binds the actual field inventory, B03/B04 counterexamples and missing continuation/
terminal contracts to source `45a7b6839e9b3855cf0a847b362d18dac29af315`.
That R5 receipt predates implementation. G4-R6 now adds `motar_eval/boia_shadow.py`
and `tools/boia_shadow.py`: strict copied sensor packets, phase/reset joins,
refusal-only diagnostics and isolated outcome/parity logging.54 focused and306
combined BOIA CPU tests pass. Actual GPU packets/parity remain NOT_TESTED.
See the [G4-R6 implementation evidence](../../evidence/boia_g4r6_shadow_preflight_2026-10-21/IMPLEMENTATION_REPORT.md)
and [Q01 preregistration](../../plans/boia_shadow_q01_v1.md). No GPU run is authorized.
G4-R7 answers the Fable G4-R6 audit: v1's OFF/ON gate could not see observer-versus-native
interference, so Q01 v1 is withdrawn before execution ([amendment](../../plans/boia_shadow_q01_v1_amendment_2026-10-21.md))
and replaced by [Q01-V2](../../plans/boia_shadow_q01_v2.md): an `S1_NATIVE` stage without
observer hooks, one host trace in every stage, and exact NATIVE/OFF, OFF/ON and S0/ON replay
comparisons. See the [G4-R7 report](../../evidence/boia_g4r7_shadow_v2_preflight_2026-10-21/IMPLEMENTATION_REPORT.md).

## 1. Research question and mission

Develop a navigation algorithm for **moving-target interception in random obstacle
environments**: predict consequential target occlusion, select whether and when to
intervene, and improve eventual collision-free capture. Shorter occlusion, better
tracking or a passed safety filter alone do not answer the mission question.

The baseline is Frozen Policy F with its existing **RGB-D Camera + LiDAR**, fused
tracker, actor898, governor and UAV controller. Retain these sensors; there is no
camera-only/LiDAR-free objective. Policy F has no new BOIA mission-benefit model or
evidence-aware temporal arbitrator. Whether those additions improve its outcomes
is an open empirical question. They must preserve permitted information and account
for actions that change observations, policy history and later F continuation.

Obstacle density affects observation opportunities, reachable viewpoints and
collision risk. Target motion changes interception and reacquisition opportunities.
Neither effect is identified by a synthetic timing example or by equating moving
and static target lineages. Static-versus-E2 effects compare whole system/lineage
conditions, not target speed alone. OCC-P1 was exploratory; OCC-P2 independently studied
natural-occlusion occurrence/duration under its frozen conditions. Those data do
not establish new algorithm capture gains. H-R1/H-R2/H-R3 remain provisional unless
a separate preregistration and qualified experiment establish otherwise.

BOIA's candidate question is: **with the same measured evidence and mandatory safety
constraints, does supported wait-then-intervene timing improve actual mission
completion over reactive, immediate and unconditional predictive alternatives?**
The outcome must include a finite interception/endgame and explicit episode end,
not indefinite motion around a target or observation retention for its own sake.

## 2. Architecture: real connections and missing bridges

Legend: `-->` exists in the stated implementation; `-X->` is NOT CONNECTED.
Existing connections do not imply physical qualification.

```text
EXISTING SIMULATOR / FROZEN BASELINE
Rendered RGB-D + LiDAR --> NavRLPerceptionModule / fused tracker
                      --> actor898 Frozen F
                      --> optional historical KROC supervisor
                      --> riskcap governor + altitude PI / yaw command
                      --> UAV controller / PhysX --> NavRL capture/crash/timeout

OLD BOIA CPU REPLAY (static synthetic packets, no plant)
Whitelist copies --> CV belief samples / candidate commands / held-command rollout
                 --> legacy safety.screen + visibility/covariance/cost
                 --> threshold/dwell arbitration --> action tensor / trace
                     -X-> actual controller

STANDALONE CPU RESEARCH KERNELS
C02 screen_next_cycle --> nominal prefix-plus-backup MODEL result
Synthetic measured-support declarations --> retained scene hypotheses
Paired surrogate loss declarations --> error-accounted benefit / wait-grid choices
                                  --> candidate-ID decision / explicit state
Independent nominal safety declarations --> validate_safety --> strict selector
Actual sensor evidence -X-> these research records
C02 result -X-> actual research safety-evidence provider (NOT CONNECTED)
Research decision -X-> Frozen F / controller / endgame executor

G4-R6 PASSIVE EVALUATOR (implemented; runtime GPU_PENDING)
Exactly one existing observe result --> immutable permitted sensor bytes + IDs
                                   --> research hypotheses/refusal --> ABSTAIN log
Existing action/command/terminal/reset --> separate evaluator/parity trace
Shadow diagnostics -X-> action writer (no action sink exposed)
```

The desired full chain is sensor evidence → hypotheses → occlusion/reacquisition
forecast → candidate trajectories → independent safety gate → paired mission benefit
→ opportunity window → control ownership → F/controller → interception/termination.
Its **sensor-to-evidence, calibrated forecast/value, C02-to-arbitration, execution and
endgame bridges are incomplete**. Do not draw that desired chain as an implemented
online system. The order is conceptual; safety does not depend on utility validity.

Source receipts: `navrl_perception.py:726,2184` defines perception/observe;
`navrl_task.py:5011–5028` runs optional supervisor then command transformation;
`:4953–5008` applies vehicle-frame velocity, governor, altitude PI and yaw;
`:5058–5063` advances the plant and calculates outcomes. Repository source search
finds no `BoiaPlanner` or `boia_research` caller in `aerial_gym`. The only non-test
`BoiaPlanner` caller is `tools/boia_replay.py:76–99`, explicitly static observation
replay. `boia/planner.py:107` calls legacy `safety.screen`, not the standalone C02
`screen_next_cycle` (`safety.py:189–198`).

Reference prefixes throughout this plan: `navrl_task.py`, `navrl_perception.py` and
`kroc_eval_adapter.py` are in `aerial_gym/task/navrl_task/`; `config/` and
`rl_training/` are beneath `aerial_gym/`; `boia/` and `boia_research/` are beneath
`motar_eval/`. Unqualified `e_l0.py` means `motar_eval/e_l0.py`, and
`h_r3_p03_launcher.py` means `tools/h_r3_p03_launcher.py`. Historical references are
bound to their named commit/receipt; R1 references describe the frozen R4 source
snapshot, whose final hashes are recorded with its validation receipt.

### Actual baseline settings versus class defaults

| Setting | Class/default | Attested Frozen F / OCC conditions |
| --- | --- | --- |
| Task / observation | Vision default OFF | `navrl_task`, vision+perception ON, actor898. Separate shooting-moving task is not this baseline. |
| LiDAR | 36×4, 4m | 72×4, 12m. |
| Horizontal command / yaw | ±2m/s per axis; ±2.5rad/s | ±2.5m/s per axis; ±3rad/s. Norm may reach sqrt(2)×2.5≈3.536m/s before riskcap. |
| Episode limit / OOB margin | 300 actions /0.5m | 600 actions /1m. At0.1s/control interval this is60s. |
| Physics / control | dt0.01s;10 fixed physics steps/action | dt0.01s and control0.1s in this lineage. |
| Capture / height | Swept relative distance<0.5m; z<0.1m or z>4m fails | Same operational task definitions. |
| Governor | Environment-dependent | riskcap; brake2.9608856678m/s², reaction0.1s, TTC1.2s. |
| Target | Default dynamics legacy | E0 bounded static, speed0; E2 bounded waypoint virtual executor, final speed1.5m/s. E2 is not a PhysX target actor. Moving OCC uses direct_l2_v1. |
| Sensors | Runtime-dependent; perception perturbations default OFF | Launcher dropout0.3, depth-noise0.02, RGB-noise0.015 apply only when `enable_perturbations` is true. The knobs alone do not establish active noise or empirical reliability. |

References: `config/task_config/navrl_task_config.py:135–158,239,465–483,595–620,727–732`;
`rl_training/rl_games/eval_navrl_target_behavior_arms.sh:57–98,120–134`;
`config/sim_config/base_sim_config.py:28`; `config/env_config/navrl_bars_env.py:54–60`.
Frozen checkpoint SHA is `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`
(launcher:30); this plan does not load or replace it. Historical OCC execution
scrubbed inherited NAVRL knobs and did not opt into later sensor corrections; do
not silently substitute a newer sensor lineage for those original results.
G4-R5 checks the actual perturbation gate (`navrl_task.py:6477`,
`navrl_perception.py:1593–1641`, task config:277); no historical run's active flag
is inferred from configured noise amplitudes alone. Public modality capture/arrival
clocks are absent, current Frozen perception does not produce a bbox, and sanitized
LiDAR ranges collapse no-hit/saturation/some invalid values. See the R5 inventory
for exact available, privileged, missing and uncertain fields.

## 3. Module status and evidence

IMPLEMENTED describes source existence; CPU_VERIFIED describes only the linked CPU
scope. Neither means ONLINE_INTEGRATED, calibrated or physically qualified.

| Module | Function / implementation status | CPU validation | Online integration | Main blocker |
| --- | --- | --- | --- | --- |
| Existing perception/tracker | `navrl_perception.py:observe`; IMPLEMENTED | Historical sensor-lineage receipts remain separate | Existing Frozen F path | Real timestamp/extrinsic/identity evidence support not supplied to research layer. |
| BOIA access | `boia/access.py:51`; IMPLEMENTED | Existing focused tests | CPU synthetic caller only | Whitelist lacks most measurement provenance/timing/association evidence; active track≠confirmed identity. |
| BOIA belief | `belief.py:71,82,127,167`; CV/process sampling and expected covariance IMPLEMENTED | CPU_VERIFIED algebra | CPU replay | NEES/NIS and model/measurement noise calibration not established. |
| Visibility / candidate rollout | `visibility.py:46`; `candidates.py:generate`; `dynamics.py:130`; IMPLEMENTED | CPU_VERIFIED under declared model | CPU replay | Detector/geometry/plant validity, B03/B04, full attitude/dynamic obstacles. |
| Legacy cost/arbitration | `cost.py:18`; `arbitration.py:44,96`; IMPLEMENTED | CPU_VERIFIED specified behavior | CPU replay | Weighted surrogate gain is not actual F/capture advantage; no F0–F5 mechanism switch API. |
| C02 geometry / next-cycle check | `geometry.py:capsule_in_free_model`; `safety.py:189`; IMPLEMENTED | CPU_VERIFIED C02, including candidate-local failures | NOT CONNECTED | Planar model, unavailable physical/error/coverage evidence; result remains MODEL-only. |
| Research hypotheses/intervals | `boia_research/evidence.py:26,37`; IMPLEMENTED | R3 30 focused tests / independent review | NOT CONNECTED | Caller declarations, not measured sensor calibration. |
| Research paired bound | `boia_research/benefit.py:8`; IMPLEMENTED | R3 exact arithmetic / sampled-bound checks | NOT CONNECTED | No actual mission-loss/F-continuation predictor or calibrated dataset. |
| Research timing/state | `boia_research/decision.py:41,112`; IMPLEMENTED | CPU_VERIFIED R4 focused50 / combined252 tests | Candidate IDs only; NOT CONNECTED | Real evidence/mission models and qualified executor remain absent. |
| Independent SafetyContext/SafetyEvidence | `boia_research/safety.py:15`; `decision.py:18–38`; IMPLEMENTED nominal safety validator and strict selector | CPU_VERIFIED R1: valid independent safety survives missing/stale utility; stale/missing safety refuses | Actual sensor/C02 provider NOT CONNECTED | Checks supplied bindings/declarations, not calibration or physical proof. |
| Trace / replay | `boia/trace.py:94`; `tools/boia_replay.py:76`; IMPLEMENTED | CPU hashes/terminal/fixtures | No plant replay | No qualified counterfactual closed-loop mission outcomes. |
| Passive Q01 observer/logger | `boia_shadow.py:EvaluatorTap/Recorder/EventLogger/HostTrace/drive`; IMPLEMENTED evaluator-only, Q01-V2 native baseline | G4-R7: 49+23 focused /324 combined BOIA CPU PASS; Fable re-review in G4-R7 evidence | Runtime launcher PREPARED, GPU NOT_RUN | Actual NATIVE/OFF, OFF/ON, replay, compile and resource qualification pending; no utility or physical certificate. |
| Mission execution / endgame bridge | Proposed ID-to-action/backup/handoff executor; NOT_IMPLEMENTED | NOT_TESTED | NOT CONNECTED | Plant qualification and explicit approval. |

The committed C02 receipt is [C02_REPORT](../../evidence/boia_g4r3_c02_2026-10-20/C02_REPORT.md).
R3 historical research validation is [validation.json](../../evidence/boia_g4r3_research_2026-10-20/validation.json):
30 focused PASS,232 combined BOIA PASS,17 independent scalar checks; read-only agent
review found no blocker within that typed-declaration scope. That receipt predates
R4's stronger independent safety contract; it does not validate that subsequent fix.
The subsequent [R1_REPORT](../../evidence/boia_g4r4_2026-10-20/R1_REPORT.md) and
[R4 validation](../../evidence/boia_g4r4_2026-10-20/R1_validation.json) record50 focused
PASS (30 preserved test-method bodies/assertions plus20 new cases), and252 combined
BOIA PASS in15.123s,0 failures/errors. The pre-R1 selector failed14 of the same20
new tests and errored on2; archived preimages and hashes bind that negative control.
Old fixture construction was migrated explicitly to the new independent safety
schema; existing test-method bodies/assertions were preserved. This is contract
correction evidence, not a larger mission-performance sample. R4 independent
read-only review ran50/50 focused tests and additional utility-poison/no-access,
expiry/endgame and sibling-isolation probes, finding no blocker within the nominal
typed CPU scope. References, model truth, calibration and physical qualification
remain unverified. PUBLIC_CPU was NOT_RERUN for R1.
Committed C02 PUBLIC_CPU had2941 tests,0 failures and4 exact-known errors; its
process status remains FAIL, with previous-error equivalence recorded. Do not call
that an all-green public run or imply it was rerun for later research code.
G4-R5 reran the combined BOIA suite once before local commit:252 PASS,
0 failures/errors (15.095s), plus14 new standalone CPU contract probes. Independent
read-only review reran14 and checked7 narrow controls before the final identifier-
format correction; root reran14 after it, while independent recheck hit usage limit.
PUBLIC_CPU remains
NOT_RERUN with its four existing errors unresolved. Actual sensor packets,
runtime OFF/ON parity and simulator/GPU behavior remain NOT_TESTED.

G4-R6 supersedes the implementation-readiness status only:54 focused and306
combined BOIA CPU PASS. Final-source PUBLIC_CPU ran3045 tests,0failures/4errors,
10skips/5profile exclusions; exact error IDs, exception chains and terminal frames
match the prior DetectReduction errors. Aggregate PUBLIC_CPU remains FAIL and
motar-6nn.5 remains open. No test was deleted/skipped to obtain BOIA qualification.
G4-R7 (Q01-V2 source `a0840c2`): 49 + 23 focused and 324 combined BOIA CPU tests pass; PUBLIC_CPU
ran 3063 tests with 0 failures and the same 4 DetectReduction errors, so its aggregate stays FAIL.

## 4. Mathematics, decision rule and interception

**Belief and evidence.** Existing CV prediction is `mu+=F mu`,
`Sigma+=F Sigma Fᵀ+Q`; `belief.tracker_Q` uses position diagonal0.025dt² and velocity
diagonal0.25dt (`belief.py:71–79`). Those are model priors. Existing fused tracker
expires at `age>memory_s`, with memory5s (`navrl_perception.py:692`; config:173).
Camera misses and identity confirmation are different from this fused recency.

Retain scene interpretations `TARGET_SURFACE`, `FOREGROUND_OCCLUDER`,
`FALSE_ASSOCIATION`, `UNKNOWN_OR_UNOBSERVED` instead of forcing an online truth
label. Aligned measured intervals, timestamps and calibration domains are necessary
conditions. The research implementation checks supplied records; it does not prove
their provenance or infer missing physical evidence. Ambiguous first hits remain
occupied evidence and are never deleted to make a trajectory safe.
The actual LiDAR target proxy radius is0.20m, versus RGB-D0.15m with camera
offset(.10,0,.03)m. Thus an on-axis6m LiDAR sphere centre produces5.8m, as can a
foreground surface. This proves ambiguity of one range/centre summary, not of
every RGB-D pixel and beam. Nominal5° ray gaps at6m span0.52343m; valid rays alone
do not certify the intervening volume. R5 preserves these UNKNOWN/ABSTAIN limits
without using semantic identity or synthesizing mission loss.

**Visibility and old cost.** The legacy forecast combines sampled target paths,
camera FOV, size gate and sensed-point LOS. `forecast(...,mode="oracle")` is its
default; it is an analytic detector assumption, not detector validation
(`visibility.py:46–72`). Its `P_any=mean_m[1-prod_h(1-p_det)]` requires additional
temporal detection assumptions. Legacy cost combines progress, observation loss,
expected covariance logdet, command smoothness and clearance (`cost.py:39–57`).
Progress is not a capture probability; fixed-command F rollout is not closed-loop F.

**Safety.** The requested executed set is the next0.1s prefix plus its complete
reaction/braking backup, with a whole-body/error envelope inside supported free
space. C02 computes a planar MODEL check, retains horizon checks, and uses the UAV
collision-box half diagonal `sqrt(.14²+.14²+.04²)=0.201990099m`, not0.14m as a full
body bound. Sparse rays do not prove free volume, and a zero command is not a
proved safe stop. `MODEL_SAFE` must not become a physical certificate.

R1 now implements a separate `SafetyContext`, `SafetyEvidence`, `SafetyProvenance`
and `SafetyValidity` (`boia_research/types.py:109–155`), validated independently by
`safety.validate_safety` (:15–66). A current nominal declaration is bound to
env/episode/decision, candidate/wait, sensor/boundary versions and timestamps,
issuance/expiry, model identity, prefix horizon and complete-backup references.
Its prefix horizon must cover elapsed time plus F wait plus the next decision
interval. Missing versions/model/timestamps, stale/expired evidence, incomplete
prefix or unavailable backup refuse safety. Only explicitly supported
EXACT_SYNTHETIC/NOMINAL_CPU_ORACLE_DECLARATION provenance in the
NOMINAL_CPU_MODEL_ONLY scope can pass. A boolean may veto, but cannot authorize.
Utility calibration, validity and loss-table keys are not consulted by this safety
validator. These declarations are not created from C02 or sensors automatically;
all results remain CPU_RESEARCH_ONLY / physical_qualified=False.

**Paired mission comparison.** For the same scenario and path use

```
d[b,m,g,j] = L(F;b,z_m) - L(wait_F(t_j),g,then_F;b,z_m)
A_low(g,j) = min_b mean_m d - eps_MC - eps_set - b_cal - b_F_continuation - c_switch
```

`paired_bound` implements arithmetic on supplied loss tables, matching exact
env/episode/decision/scenario/path IDs. It does not produce these losses from
measurements. Finite exact synthetic worlds justify zero errors only within that
declared enumerated model. For iid paths with `|d|<=D`, its conditional simultaneous
Hoeffding check is `eps_MC >= D sqrt(2 log(2 O B/alpha)/M)`, where O includes every
tested candidate×wait option. Time knots are not extra independent samples. Missing
calibration/continuation/set coverage refuses utility; a citation string alone is
not a verified artifact. The shared-model regret formula is prior art.

**Timing and ownership.** At each finite wait knot, require the complete F waiting
prefix, candidate prefix/backup, evidence and positive supported net gain. The
current window is the connected supported prefix starting at0; later reappearance
does not bridge a gap. Select the largest supported `(candidate,time)` bound;
ties use earliest time then stable ID. A preferred future option means KEEP_F now
and re-evaluate, not queue an executable command. The R3 `closing` flag is only a
finite-grid diagnostic; it does not create an independently demonstrated mechanism
or calibrated last-safe deadline. Hysteresis is last and cannot retain an unsafe,
unsupported or harmful owner. Safety override precedes utility refusal. R1 now
enforces that separation: `_option_safe`/`_current_safe`/`_baseline_safe`
(`decision.py:18–38`) use independent safety evidence. Missing, uncalibrated or
wrong-key utility cannot invalidate an otherwise current safety-only backup.
Conversely, missing/stale safety cannot be repaired by a positive benefit.
The strict public `decide` validates current baseline F safety before KEEP_F,
ABSTAIN, endgame hand-back or WAIT (:141–155); an unavailable baseline proof must
select a separately supported immediate backup or return NO_CERTIFIED_BACKUP.
This corrects the research contract, without changing production/Frozen F.

**Actual interception and end.** OCC-P2's source was
`5ff285ad053f181c5a1c86dddb2462bb44e6327b` ([launch receipt](../../evidence/occ_p2_confirmatory_2026-10-17/launch_receipt.json):64–65).
Git-show of that source confirms the current definitions below:

1. Sweep target-relative segment `prev_rel→rel`; find its closest point to origin;
   capture iff its distance is **strictly<0.5m** (`navrl_task.py:6119–6123`). This
   linearly approximates both agents' motion across a control interval. It is a
   geometric `capture_0p5m` proxy, not physical contact/grasping or the separate
   shooting task's0.12m contact criterion.
2. Capture wins over a same-step crash: `crashed_out &= ~captured` (:6124–6126).
   Contact, z<0.1, z>4 and vision-mode OOB enter the pre-mask failure condition
   (:6071–6107). Contact force threshold is0.05N; there is no physical floor mesh.
3. Episode termination is capture OR crash (:6304). Timeout is a truncation at
   `sim_steps>=600`; reported timeout excludes success/crash (:322,332,5065–5087).
   Thus endless orbiting is bounded by60s in the attested profile.
4. Ended rows reset before the returned sensor observation (:5167–5182). Consume
   pre-terminal observations only and preserve terminal physical telemetry before
   reset; a new-episode detection cannot count as reacquisition in the old episode.

Historical capture and crash therefore **cannot alone establish collision-free
capture**: a simultaneous obstacle/height/OOB failure can be suppressed by capture
priority. This is a source-level possibility, not a measured contamination rate.
Future collision-free capture needs separately recorded pre-mask contacts/safety
failures and a preregistered combination rule. Existing definitions and results
remain unchanged. E-L0/P03 have independent contact telemetry primitives
(`e_l0.py:182,350`; `h_r3_p03_launcher.py:61,133`); this is not a claim all historical
datasets have sufficient telemetry. Existing outcome classes are capture/crash/
timeout; safe abort is NOT_RECORDED (`kroc_eval_adapter.py:540`). Research
NO_CERTIFIED_BACKUP is not an executed safe abort. The old INTERCEPT mode only
disables gain switching when camera-visible and belief distance<3m; safety may
still override (`arbitration.py:67,115–123`). No new endgame controller is qualified.

## 5. Novelty boundaries and contribution candidates

**No new theorem or verified originality is claimed.** Original-paper receipts and
their exact reading scope are retained in [NOVELTY_REVIEW](g4r_2026-10-20/NOVELTY_REVIEW.md)
and `paper_manifest.json`. Original supplied-paper reads and abstract-only/method-
partial additional sources are not interchangeable. The following is a synthesis
of those bounded checks, not a fresh exhaustive literature search.

| Prior work / principle | ALREADY_KNOWN overlap | Remaining BOIA question |
| --- | --- | --- |
| FASTER | Known-free backup, committed trajectory, unknown-space constraints | Conditional target-mission intervention; no FASTER guarantee transfers to sparse planar rays. |
| BPOD | Belief-conditioned detection/FOV/LOS and expected covariance | Sensor/identity ambiguity and actual mission-calibrated timing. |
| Eva-Tracker / Elastic Tracker | Active viewpoints, visible corridors and predictive tracking | Selective control ownership over unchanged F, rather than visibility alone. |
| UAST / YOPO / YOPOv2-Tracker | Target prediction/search, finite or learned primitives/scores | Evidence-supported ID decisions and qualified mission hand-back; no borrowed runtime. |
| NavRL++ | RL-compatible sensing and safety intervention | Added target-mission timing beyond the common safety layer. |
| PRIMER / shielding / predictive safety filtering | Check/reject/retain plan or correct policy action | Utility evidence refusal is separate from mandatory safety; C02 correction is not novelty. |
| Robust MPC / runtime assurance / CBF-based safety | Uncertainty-aware predictive feasibility, protected baseline/controller | No invariant-set, recursive-feasibility or real-world safety proof is supplied here. |
| Active-perception POMDP | Belief, ambiguous observations, information-driven actions and terminal rewards | Specific implementable/calibrated temporal arbitration, not new problem vocabulary. |
| Robust baseline regret | Same-model candidate-versus-baseline robust difference | Only a concrete evidence/temporal construction and mission result could distinguish the method. |

At most three **CANDIDATE / NOT_VERIFIED** contributions remain: (1) supported
intervention timing before losing an actionable observation/viewpoint opportunity;
(2) ambiguity-selective mission comparison with independently preserved safety
obligations; (3) reproducible, mechanism-controlled episode/failure evaluation.
Paired regret, inflation, Gaussian prediction, MC detection, threshold/dwell,
weighted costs and fallback alone are **ALREADY_KNOWN**. Software validation is
VERIFIED only for its specific receipt; paper novelty is not VERIFIED. Timing
argmax overlaps finite-horizon planning, so a decision difference is necessary but
insufficient originality evidence. If matched mission gains or a distinct mechanism
fail, revise or withdraw that contribution rather than broaden the wording.

## 6. Blockers and research limits

| Blocker | Present fact | Required evidence before claim/execution |
| --- | --- | --- |
| B03 sparse LiDAR coverage | Finite zero-width rays cannot certify the unmeasured intervening volume | Justified beam/size/multiview/dynamic-age support, or UNKNOWN/refusal. No optimistic fill. |
| B04 target/foreground identity | Different worlds may produce identical permitted first hits | Retained hypotheses and independent identifiability/error tests; no semantic GT online or first-hit erasure. |
| C02 physical qualification | Prefix/backup MODEL API exists, no sensor/plant certificate | Valid observation volume, dynamic/error bounds and execution qualification. |
| Sensor timing/extrinsics | Old access has no capture-time evidence; yaw-only calibration is limited | Capture/arrival clocks, pose-at-capture, calibrated SE(3)/depth convention, source/expiry. |
| R1 safety/utility coupling | CPU_VERIFIED nominal separation; current independent declarations required | Real sensor/C02 proof generation and physical qualification remain unavailable; no declaration-to-reality inference. |
| Frozen F continuation | Held current action is not actual closed-loop F | Qualified matched one-prefix-then-F branches including actor/history changes; calibrated mismatch. |
| Actual mission-loss predictor | Research takes external synthetic loss tables | Permitted-info predictor, independent held-out support/error budget; no privileged critic online. |
| Model/calibration | Priors and declaration tags are not calibrated detector or tracker uncertainty | Detector Brier/ECE, association error, NEES/NIS, supported strata and benefit error accounting. |
| Deadline / plant tracking | Full planner CPU N128 observation13.2449s; GPU timing NOT_TESTED | Device-specific deadline amendment, end-to-end latency/VRAM, controller tracking-error qualification. |
| Interception/endgame | Baseline geometric endpoint exists; new handoff is not validated | Preserve old metric; independently measure collision-free endgame and hand-back behavior. |
| Episode performance / novelty | Synthetic timing choices only | New independent seeds, mission endpoints, appropriate cluster inference and mechanism ablations. |

The original07 GPU≤5ms target and09 CPU≤5ms gate are inconsistent; amend
prospectively rather than silently reinterpret either. Tiny prebuilt research-kernel
timing does not measure sensing, forecasts, plant rollout, transfer or full planning.
Main WIP/stash, raw/checkpoints and protected links remain outside cleanup. External
raw backup remains unavailable; deletion is not a remedy for qualification or space.

## 7. Experiment plan and comparison rules

All items here are **prospective DESIGN_ONLY**, not launch approval/preregistration.
Preserve original OCC/P01/P02/P03 evidence with its own scope. Independent seeds and
scenes are required for a new algorithm claim; do not recycle those exploratory or
failed-integrity samples as confirmation. Verify same pre-branch measured state,
history, policy/RMS and exogenous streams; behavior-induced sensor differences stay.

Final endpoints are collision-free capture, historical capture, crash, timeout,
time-to-capture, occlusion duration, identity-correct reacquisition and delay,
intervention frequency, decision latency/misses and NO_CERTIFIED_BACKUP. A no-backup
result is a failure category requiring an explicit future executor response, not
successful safe stopping. Distinguish competing capture/crash/timeout events and
duration censoring; stopping indefinitely cannot count as a safety performance win.

Cross70/115/160/205 bars with static/moving target lineage and initially visible/
hidden strata explicitly. The existing primary proposal emphasizes moving visible
160/205 conditions; full matrix/seeds/sample size require feasibility and separate
preregistration. Initial hidden acquisition is different from post-acquisition
occlusion recovery. Compare end-to-end assigned episode outcomes with seed as an
independent cluster; do not condition the main effect on post-treatment occlusion
events. Qualified common-prefix branch analyses are separate counterfactual estimands.

| Arm | Intended mechanism | Current implementation/execution status |
| --- | --- | --- |
| F0 | Literal Frozen F, original governor/controller/sensors | Historical baseline exists; new research activation absent. A common overriding gate would make it F1, not literal F0. |
| F1 | F plus common mandatory safety-only gate; utility explicitly disabled | DESIGN_ONLY interface; no fake safety-only arm from zeroing weights. |
| F2 | F1 plus actual camera-loss-after-acquisition reactive viewpoint rule | DESIGN_ONLY F arm; historical H-R3 B arms are different experiments. |
| F3 | Same safety, deterministic-CV predictive utility without sampled belief/expected-covariance package | DESIGN_ONLY; zero uncertainty weight alone does not implement this removal. |
| F4 | Same safety, full belief-aware predictive utility, unconditional safe utility-best selection | DESIGN_ONLY; no positive paired-benefit/wait criterion. F may legitimately be best. |
| F5 | Independent evidence/safety, supported paired benefit, switching cost and wait timing | CPU principles partially implemented; full system NOT CONNECTED/BLOCKED. |

The corrected mechanism definitions in [EXPERIMENT_DRAFT](g4r_2026-10-20/EXPERIMENT_DRAFT.md)
supersede the old09 weight-only proposal for **future planning**, preserving09 as
history. PRE still uses predicted detection even with old visibility weights zero,
and tau_on0 is rejected. The old candidate19 translates0.6m/s; it is not yaw-only.
Required mechanism contrasts include evidence ON/OFF, robust/point comparison,
immediate grid{0}/predictive waits, and selective/unconditional selection, with
matched sensor information, safety, compute and switching costs. Removing next-cycle
backup is CPU/offline only. No unsafe hardware/GPU arm is implied. Any literal F0
abort-monitor protocol must be separately approved and labelled F0_WITH_ABORT_PROTOCOL.

## 8. Roadmap, gates and traceability

| Stage | Input → output | PASS / HOLD condition and current status |
| --- | --- | --- |
| R0 Git hygiene / ownership | All-worktree/staged/unstaged/untracked inventory → protected classification and controlled local exclusions | PASS_CONTROLLED per R0_GATE; initial target29 and main3725 explained, no raw deletion. Final gate rechecked after scoped commits. |
| R1 CPU safety/utility separation | Independent safety context/evidence → strict research selector, preserved assertions,20 new failure controls | CPU_VERIFIED nominal contract:50 focused /252 combined BOIA PASS; missing/stale utility preserves a valid safety backup and missing/stale safety refuses. Read-only review found no blocker in this scope; no physical/online provider or production coupling. |
| R2 Real measurement contract | Actual RGB-D/LiDAR/tracker clocks, calibration and permitted identity support → approved evidence adapter/schema | G4-R6 observer/phase/reset/schema IMPLEMENTED and CPU_QUALIFIED; G4-R7 adds the S1_NATIVE baseline and exact host-trace comparisons (CPU); actual GPU packets/parity NOT_TESTED. Clock tags are DECLARED_SIM_SCHEDULE, not measured hardware time. Physical bounds/calibration remain unavailable. |
| R3 B03/B04 identifiability | Independent range ambiguity and sparse-coverage tests → supported-volume/ambiguity limits | R5 counterexamples verified in nominal CPU geometry; physical coverage/identity remain unresolved. Preserve occupied/UNKNOWN evidence. |
| R4 F continuation | Qualified matched prefix branches + actor/history state → continuation model/error receipts | Existing fresh-process prefix pattern is reusable; full snapshot restore and new-profile continuation NOT_QUALIFIED. Required state/API gaps recorded in R5. |
| R5 Mission benefit calibration | R2–R4 models + held-out independent scenes → bounded loss/benefit/selection calibration | HOLD on unsupported strata, missing error terms or no predictive discrimination. |
| R6 Real-time / G5 eligibility | Qualified geometry/plant/sensing/models → device-specific deadline and eligibility report | G5 HOLD; CPU/GPU/transfer/trace checks and separate user decision required. |
| R7 Approved small execution | Frozen source/preregistration + passed gates + explicit approval → bounded small-run raw/manifest | NOT_AUTHORIZED; stop on integrity/resource/deadline failures; no automatic expansion. |
| R8 Full episode experiment | Reviewed R7 feasibility + fixed independent design → capture/crash/timeout/competing-risk estimates | NOT_AUTHORIZED; seed-cluster inference and complete negative/failed outcomes. |
| R9 Ablation / paper claim | Mechanism-controlled results + bounded primary literature review → retain/revise/reject contributions | NOT_TESTED; no first/new-theorem claim from known components or CPU tests alone. |

R0's [R0_GATE.json](../../evidence/boia_g4r4_2026-10-20/R0_GATE.json) accounts for
3474 historical raw files,169 verified CPython3.8 cache files,78 user presentations,
one user test and3 tracked WIP files in main. Only verified bytecode received local
exclusion; raw remains visible/protected. PASS_CONTROLLED does not mean clean main
or permission to stage its assets. Existing source/history and raw manifests remain
authoritative; this plan is their entry point, not a replacement evidence record.

### Shortest next execution unit

G4-R6 completes the observer/logger and launcher without production edits. The
single `observe` result is copied once; actual F action binds at task.step entry,
transform is called once, pre-mask causes precede alias overwrite, and the returned
normal/reset observation boundary is also compared. ContactChain preserves the
existing callback. Body/target/semantic truth never enters BOIA packets.
Next: **seek separate approval for Q01-V2 N1 S0/S1_NATIVE/S1_OFF/S1_ON**, using the fixed
[runbook](../../evidence/boia_g4r7_shadow_v2_preflight_2026-10-21/GPU_SHADOW_RUNBOOK.md) (Q01 v1 withdrawn, never run).
No additional implementation project is required before that bounded execution.

| Gate | Requirement |
| --- | --- |
| MUST_BEFORE_SHADOW | Observer/logger/phase/whitelist/copy and CPU noninterference tests complete; launcher/config/preregistration prepared. Exact containing commit, clean tree and checkpoint/config guards must pass final CPU preflight. Separate GPU approval is still required. |
| MAY_DEFER_UNTIL_AFTER_SHADOW | B03/B04 physical resolution, calibrated utility and F-continuation model. Keep them UNKNOWN/ABSTAIN; passive availability shadow estimates no capture gain. |
| MUST_BEFORE_ONLINE_CONTROL | Physical C02/backup, all used evidence/model bounds, action ownership/hand-back, controller/plant/deadline/endgame qualification and explicit user approval. G5 remains HOLD. |

The implemented but unexecuted first shadow (Q01-V2) is N1, seed 4901, moving 70 bars: a 20-action
smoke plus NATIVE, OFF and ON stages of at most 200 actions each, at most 1080 s (18 min) GPU wall.
Actual NATIVE/OFF and OFF/ON parity are early qualification objectives, not a present PASS.
Full planner/value computation is skipped where unsupported. Its exact resource,
latency, censoring and failure guards are in the Q01 executable plan/preregistration.
CPU preflight does not import Isaac/torch, unpickle checkpoint or query/run GPU.
Observer timing includes copy/schema/diagnostic/hash/log apart from full-step time;
no real-time guarantee. Physical C02/B03/B04/F-continuation and utility remain open.

Stop resource work below
3,000,000,000 free bytes. Keep commits separated by approved research unit, stage
explicit paths, report local source/result SHA and distinguish local from remote.
No main merge/push, force push, history rewrite, raw/checkpoint deletion or automatic
G5 release. If the session cannot finish, freeze the current unit and hand off exact
completed steps, changed files, verification and next blocked/authorized action.
