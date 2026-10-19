# 13 — Original-copy corrections and proposed design deltas (G1-R)

**Scope:** factual corrections and proposals only. The equations, parameters and experiment design
in07–09 are not changed by this reading pass. References below use physical PDF pages, starting
at1, of the supplied originals; the detailed reading receipts are in P1–P6 and the v2 manifest.
Proposals are **NOT APPLIED BEFORE USER APPROVAL**. G4 implements the current07 specification,
subject to the explicit amendments in the user's CPU implementation instruction.

## 1. Facts changed by the original copies

| Previous statement or uncertainty | Original-copy finding | Consequence |
| --- | --- | --- |
| YOPO unavailable; YOPO-Simple predicts a cost selected by argmin | YOPO original p.5 Eq.(21) trains its score against **−J**; higher score is preferable. The audited `YOPO-Simple` bd8a7f9 branch uses a cost interpretation. P3 §6/K2 distinguish paper and branch. | Do not attribute that branch's score convention to the original. BOIA's own explicitly defined J remains minimized. |
| Original YOPO anchor extents/radius inferred from code | Original has15 anchors and its own trajectory interface; numerical angle extents/radius are NOT_RECORDED in the original text. Code's5×3,±36°/±20°,5m is branch-specific. | Keep BOIA's own15 heading/speed commands; no unsupported transplant of YOPO endpoint geometry. |
| OCTrack unavailable; acronym and depth meaning unknown | P5 §4 identifies COM as Clustered Occlusion Modeling, ACP as the model's parameterization, and CADB as Cost-Aware Depth Bias. “Depth” is transformer layer depth, not metric range. | Structured occlusion training is relevant, but this does not supply a calibrated p_det(size,visibility). |
| ORTrack Eq.(1) may mask template centre | Original p.3–4 Eq.(1) sets sampled-mask entries to1 then multiplies features by that mask; centre-heavy samples are retained as printed, while prose describes masking the target. | INTERNAL_PAPER_INCONSISTENCY remains; do not claim the original resolves centre erasure. P5 also records the Eq.(3) slope/intercept prose discrepancy. |
| EGO-Planner references from arXivv2 | Final original: Eq.(1) p.2; Eqs.(3–6) p.3; Eqs.(11–12) p.4. Time allocation Eqs.(14–15) retains its numbering. | Version/page mapping in P4 §4 replaces ambiguous citations; no BOIA equation changes. |
| Lee thrust/moment/desired-axis are Eqs.(12–14) | CDC original p.3: desired b3 Eq.(12), thrust Eq.(15), moment Eq.(16); assumptions Eqs.(13–14). Stability Eq.(27) uses λ_max(J), not the λ_min written in an old checklist. | Use the six-page equation map in P4 §2. MOTAR's clipped modified velocity law inherits no Lee proof. |
| Star-Convex Eq.(10) unreadable | Original p.3 rendered. Printed gradient appears opposite in sign to a chain-rule derivation from its own Eqs.(1),(8–9); distinguish printed formula from independent derivation in P1 §5. | Not a BOIA formula: v1 does not implement that gradient/SCP. No speculative silent correction. |
| BPOD adjacent-bar product is necessarily optimistic | p.4 Eq.(15) explicitly assumes mutually independent events. Positive-correlated clear events can make the product **underestimate** joint clearance; other correlation can reverse this. | U1 must state a constructed correlation and assert its predicted sign, not a universal direction. |
| UAST's two FOV ReLUs may be added | Original p.6 Eq.(13) multiplies them. Innovation onp.4 is normalized by R, not S. Threshold/horizon values remain unspecified. | Correct comparison notes; BOIA's own geometry and explicit parameters remain unchanged. |
| PANTHER optimization numbered Eq.(3) | Original p.8 is an unnumbered boxed problem. Deep-PANTHER p.5 footnote4 cubes the in-FOV function. | Citation correction, not a new LOS model. |
| Broad learned-policy-plus-fallback structure might be new | PRIMER pp.2–3 Fig.1/§II.B checks learned trajectories using RMADER Check/Delay Check, rejects conflicts and reuses a committed trajectory. PARM* is the DAgger expert. | Broad arbitration/fallback novelty is NOT_NOVEL_ENOUGH. Narrow calibrated-advantage arbitration over FrozenF is still unverified outside the reviewed set. |
| PPTracker's GT might be training-only | Original p.3 uses first-frame GT boxes at online initialization; subsequent priors are tracked boxes. | Do not transplant GT initialization into BOIA. Track activity is not identity evidence. |
| Similarity-Guided and Layer-Guided might be the same work | Supplied Similarity-Guided original identifies arXiv2503.06625v1; Layer-Guided is2602.13636 with different identity. | Separate papers. Embedded-device results do not prove concurrent IsaacGym/N128 feasibility. |
| AerialGym feedforward sign is an unconditional error | p.3 Eq.(4)/p.4 Eq.(5) prints a negative desired-acceleration contribution; the paper does not fully fix error/gravity conventions. p.7 control-rate numbers are not a global physicsdt. | Record a convention-dependent concern, not a proven production bug. Frozen velocity mode has zero desired-acceleration feedforward. |

Other page, table, architecture and author-internal arithmetic corrections are preserved next to
their prior statements in P1–P6. They are not evidence that BOIA has been calibrated or validated.

## 2. Proposals, not applied

| Target | Evidence and proposal | Expected use | Risk and next gate |
| --- | --- | --- | --- |
| 07 §8 lattice | YOPO p.5/P3 §6: preserve the distinction between endpoint anchors and BOIA heading/speed commands; no automatic anchor matching | Honest finite-candidate provenance | Candidate coverage must be checked in V4/V5; no claimed benefit from matching15 alone |
| 07 §5 stressed detector | OCTrack P5 §4: later consider structured/temporally correlated masks as a diagnostic, in addition to independent dropout | More realistic detector stress | No calibration function is supplied; V2/V5 and a separate approval are required |
| 07 §10 arbitration | PRIMER pp.2–3: explicitly distinguish safety reject/reuse from calibrated expected advantage, hysteresis and dwell over a frozen policy | Narrow the potential contribution | Broader shield/runtime-assurance/residual-policy literature remains G1b, not resolved by four extras |
| 07 §§1–3 identity | PPTracker p.3: consider sensor-only confirmation/identity bookkeeping in a later lineage | Separate track activity from actual identity | No GT boxes online; tracker expiry/sensor contracts stay fixed; V2/V3 first |
| Detector stress implementation | P6 Similarity-Guided: possible separately qualified passive visual tracker, not an automatic replacement for MOTAR's detector | A future realism diagnostic | No demonstrated IsaacGym coexistence/8GB/N128 budget; V2 and separate GPU authority required |

## 3. Novelty and the Part-A → Part-B gate

Elements(a),(b),(d) are prior art. Generic learned-planner arbitration with safe fallback is also
prior art, now directly evidenced by PRIMER. The narrower element(c)—calibrated expected advantage,
hysteresis and dwell over FrozenF—remains **CANDIDATE_CONTRIBUTION_UNVERIFIED**. Absence in this set
is not proof of novelty; G1b remains open. Mission-level evaluation is a candidate methodology,
not an established contribution or performance result.

None of the original-copy corrections above establishes that a formula actually used by07 is
false. Star-Convex/PATimeOptimal source inconsistencies concern unused optimization formulas;
Lee's stability assumptions were already explicitly not inherited. The07 response model is a
declared uncalibrated approximation. The proposals above are not silently applied to07–09.

**Conclusion: MINOR_ADJUSTMENTS_PROPOSED — novelty narrowed; G3 equations unchanged.**

Proceed to the approved G4 CPU units only after the PartA verification/commit. This is not
permission for G5, simulator/GPU work or raw-based calibration.

## 4. Implementation concerns identified before coding

- **Bar geometry is a pool, not one width.** Read-only `env_object_config.py:813–820` selects
  `bars_h3` with `file=None`;40 URDFs have different rectangular footprints. For example
  `resources/models/environment_assets/bars_h3/bar_000.urdf:14` has collision box
  0.7096×0.5756×3m. Across the pool, horizontal half-width components range0.20145–0.3951m.
  No unique online bar radius follows. Keep `r_bar_est=None` until the caller declares a prior;
  do not read the chosen asset identity/layout online or invent a mean radius. The40-dimension
  read-only inventory is recorded with G4 evidence.
- **U4 braking has two criteria in the instruction.** Before coding, the scalar semi-implicit
  dt=.01,τ_v=.4 model predicts asymptotic zero-command distances0.78/0.975/1.3728m at2/2.5/3.52m/s.
  Against the declared0.82/1.05/1.57m table, errors are4.88/7.14/12.56%. Thus the10% target is
  NOT_MET at the high speed; the separately declared25% model-form stop is not triggered.
  Do not retune τ_v or claim10% agreement. This must remain explicit in U4 tests/report; any
  later calibration or revision is G5/user review, not an outcome-driven G4 default change.
- **U7 supersedes the old raw-calibration row in08.** The current user instruction permits
  synthetic fake-task replay and raw header/key inventory only. NEES/NIS, reliability and
  false-alarm conclusions remain G5. No missing field is synthesized from GT.
- **Trace dependencies and the firewall.** Package imports remain math/dataclasses/typing/torch
  and internal modules, with only the explicit U4 governor exception. Required forbidden-name
  declarations are audit data, not permission to access those attributes; AST checks must
  distinguish the declared blocklist from an actual attribute/string lookup.

Implementation-specific resolutions, tests and limitations will be appended below. They do not
change07 defaults, silently relax acceptance criteria or supply any performance claim.

## Proposal issue index

- `motar-3p3.8`: G1-R proposal: retain BOIA heading-speed lattice; separate YOPO paper/code interfaces
- `motar-3p3.9`: G1-R proposal: structured occlusion stress and detector calibration
- `motar-3p3.10`: G1-R proposal: narrow FrozenF arbitration novelty against PRIMER and shielding
- `motar-3p3.11`: G1-R proposal: sensor-only identity confirmation without GT initialization
- `motar-3p3.12`: G1-R proposal: qualify SGLATrack as later detector stress reference

## User clarification after the usage-limit pause

The user explicitly selected: a10% braking-target miss is **LIMITED_VALIDITY**, and CPU units may
continue when the error is within25%. Above25% remains a model-form failure. Keep the uncalibrated
τ_v=.4 prior; this does not waive or claim the10% target, authorize calibration or change07.

## Implementation resolutions — U3

- The instruction's sampling signature omits the transition timestep. `sample` therefore adds an
  explicit keyword `dt=.1`; the orchestrator must pass `params.dt`. Every returned future state
  includes that step's process increment, matching the t+dt through t+Hdt prediction convention.
- No-detection preservation means no measurement-induced reduction relative to the supplied
  predicted covariance. CV propagation with negative position–velocity cross-covariance can reduce
  a position variance; a blanket Loewner-growth assertion would be mathematically false. The test
  includes this counterexample instead of changing the CV model.
- Expected covariance uses the algebraically equivalent convex mixture of prior and Joseph
  posterior for numerical PSD stability. Probability zero returns an exact copy. Across-horizon
  propagation of these expected updates belongs to U7, not independent pointwise calls over H.
- Cholesky retries are per environment and returned as optional explicit diagnostics; any jitter
  changes the sampling covariance by the recorded diagonal amount. Neither Q nor R is calibrated.

## Implementation resolutions — U4

- Normal `aerial_gym` package import reaches Isaac Gym. The BOIA package therefore mirrors the
  pure riskcap/raw-scan arithmetic; tests load only the original standalone governor file in an
  isolated CPU child. Equality is conditional on identical scan and configuration. At predicted
  poses, the original measured world-point corridor is an **ADAPTED** approximation, not a new
  scan or an equivalence claim for the complete production filter.
- Frozen runtime values follow M2 §0, including free cap3.53553390593 and telemetry brake2.96;
  the standalone source defaults differ. `governed_cmd` stores physical m/s and rad/s, whereas
  input/output actions remain normalized. Positive yaw lag requires an explicit initial rate;
  unknown lag is not inferred from body-z angular velocity.
- The first braking knot fixes a=2²/(2·.82). Below it use v²/(2a); between knots interpolate;
  above the last use D_last+(v²-v_last²)/(2a) with a warning. This retains continuity without a
  new fit. `braking_distance` returns the reaction-inclusive total, so safety adds no second
  reaction term. The default tau_v=.4 prior has a12.5605% high-speed discrepancy: **LIMITED_VALIDITY**,
  below the user's25% stop criterion. The10% target is not passed and no parameter is retuned.

## Implementation resolutions — U5

- Candidate generation additionally returns a diagnostics dictionary alongside commands and family
  IDs, so inactive/zero-distance belief fallback and failed arc searches are explicit. Candidate0
  is copied exactly; an invalid out-of-box F command fails closed rather than being changed.
- Inactive beliefs use own velocity heading, or own yaw when stationary, with zero yaw request and
  zero/invalid arc offsets; no synthetic target range is invented. Arc-path geometry uses07 §4's
  belief-excluded obstacle set, whereas the older H-R3 script used its unexcluded set for that leg.
- Safety checks each consecutive substep segment, including the initial segment, with the larger
  endpoint speed. Both U4 initial-position and initial-velocity diagnostics are required, so a
  missing initial endpoint cannot silently be invented. This is conservative relative to point-only
  substep checks, but remains an unqualified sensed-point surrogate.
- The body soft deficit below.398m is a diagnostic in metres; it is not a hard rejection and not
  the §9 cost. U6 computes the specified normalized clearance cost with d_soft=1m from clr_min.
  Unknown always blocks safety, even when the visibility unknown-space ablation is disabled.
