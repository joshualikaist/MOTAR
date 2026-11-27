# EQ-1 — proposed independent validation plan (NOT_EXECUTED)

This is the required follow-up plan from EQ-0, **not a preregistration, implementation approval or
GPU authorization**. No new scene generator, Belief, Planner, PPO, environment fix or EQ-1 rollout
was implemented. User review of EQ-0 precedes selection/approval of any package below. Original
OCC source, Policy F, tracker expiry, actor width and results stay immutable.

## Order and independent reference principle

1. Review the user's [provisional H-R definitions](PROVISIONAL_HYPOTHESES.md), then fix the intended
   qualification claim, metrics and sensing/plant contract. Decide whether ideal RGB-D/yaw stabilization/virtual planar target is an explicit
   research assumption, or real RGB+LiDAR/3D flight is required. Do not imply one by testing the other.
   Online Recovery may use only information available in its declared real sensor contract, no
   unavailable synthetic depth/GT. Current F's LiDAR+synthetic RGB-D is distinct from a long-term
   camera-only candidate; choosing an ideal baseline analysis does not waive that distinction.
2. Run separately approved CPU geometry/math/dataflow qualification before any GPU fixture run.
3. Review counterexamples and a concrete opt-in correction diff. Keep baseline and corrected
   lineages side by side. No fix is authorized by this plan.
4. Commit a new fixture preregistration and code, then obtain explicit GPU approval. Run a small
   environment qualification canary; this still tests no Recovery algorithm or paper hypothesis.

Each expected value must come from analytic geometry/ODE, independent measurement, a separately
implemented reference, or held-fixed-input invariance. Calling production code twice is not an
independent oracle. Version/reference inputs and all failing fixtures must be retained. A test
runner successfully reproducing a known counterexample is not a qualification PASS for that code.

## EQ1-A — CPU geometry, projection and sensor conventions

**Proposed files, not created:** `tools/audits/eq1_geometry_reference.py`,
`tests/test_eq1_geometry_reference.py`, a new dated plan/receipt under `docs/plans/` and new evidence.
No existing sensor/execution source changes in the validation-only package.

Fixtures: known plane/box/sphere positions; bearings 0/±5/±20/±40°; camera/LiDAR translations;
target-centre versus surface returns; vertical and partial/full occluders; unoccluded subpixel
targets; contact/tangent ambiguity cases. Use independent face inequalities/polygon projection,
analytic ray/sphere intersections and explicit SE(3) transforms. Retain target-alone pixel support.

Proposed synthetic nongrazing gates, fixed before execution:

- Ray/pixel direction correspondence ≤0.5 pixel (nearest-column quantization); common-origin
  metric error ≤1e−4 m for exact fixtures. These tolerances are below task safety margins and
  account for quantization/floating point, not a claim about hardware noise. Exact GPU ray fixtures
  can use angular/range equivalents derived from their ray table.
- Do not collapse vertical bearings unless the output is explicitly a conservative 2D envelope.
  Conservatism means no false-clear obstacle bin outside the registered boundary band.
- Camera/LiDAR target-radius conventions are either identical or transformed explicitly before
  association. Identity-negative target-absent/bar fixtures cannot produce a target-specific accept
  if persistent identity is claimed. Otherwise label range association and do not assert re-ID.
- Full/partial/centre-only/FOV/scale cases must agree with the independent labels. Zero pixels
  alone never supplies the full-occlusion ground truth.

Current fusion must FAIL the appropriate exact fixtures; do not relax their gate to recover PASS.
Any correction follows in an independently approved sensor-lineage package.

Estimated implementation/review effort: 8–14 person-hours for this package and sensor-fixture
   integration; CPU execution <1 core-hour, output <100 MB. These are planning estimates, not measured
EQ-1 performance or a commitment to execute.

## EQ1-B — CPU plant mathematics and scene feasibility

**Proposed files, not created:** `tools/audits/eq1_motor_reference.py`,
`tools/audits/eq1_scene_feasibility.py`, independent tests and evidence.

- Motor: choose the intended continuous-time first-order/rate-limited model explicitly. Compare
  analytic step/ramp and independent high-precision integration over declared dt/tau ratios.
  Require the named method's algebra and convergence; separately characterize current fixed-OCC
  composite response. Do not simply replace plus signs with minus while retaining a modified
  coefficient and call it the same calibrated plant.
- Vehicle/asset geometry: independently parse URDF visual/collision transforms, rotated body
  support and target/bar geometry. Distinguish collision proxy from rotor envelope and control
  reserve; compare independent face/SAT/swept-body fixtures. Gate zero false-clear outside a declared
  contact tolerance; physical rotor clearance requires measured geometry, not the proxy alone.
- Layouts: start/target pair connectivity in conservative configuration space for the actual body
  and declared tracking/braking reserve. Use exact interval/polygon reasoning or multi-resolution
  grids with exact boundary checks. Prove a route, prove disconnection, or mark unresolved. Do not
  interpret a failed coarse-grid search as proof of impossibility.
- A future feasible-Recovery benchmark may admit only certified pairs, with rejection rate and
  all original scenes retained. Do not retroactively filter OCC data. A global kinematic path
  does not yet prove a dynamically feasible trajectory.

Expected compute: motor/geometry <1 core-hour; full recorded-layout connectivity 1–4 core-hours
depending on resolution/method, output <1 GB. These are estimates. Real motor/airframe parameter
identification is an additional hardware task with resources NOT_ESTIMABLE until a platform exists.

## EQ1-C — CPU actor firewall, tracking and statistical populations

**Proposed files, not created:** `tools/audits/eq1_information_flow.py`,
`tests/test_eq1_information_flow.py`, `tests/test_eq1_tracking_reference.py` and a new record.

Fixtures and pass/fail gates:

- Fixed cached sensor/ego/track inputs; independently mutate privileged target pose/velocity,
  GT visibility, semantic IDs, evaluator fields and critic state. Exercise N=1/2/128, environment
  permutations and reset boundaries with the same cached track/history state. Within each matched
  batch shape/order, actor 898-D tensors and deterministic actions must be bit-identical under
  privileged-only interventions; critic extras may change. Undo permutations before comparing
  corresponding environments. Cross-batch-size kernel rounding is a separate numerical question,
  not an automatic GT-leak verdict. Hash mean/variance/count of normalization state before
  and after play; invariance is required. Checkpoint loading, if needed, is separately scoped;
  no checkpoint or actor interface change.
- Derive CV/KF reference by hand. Camera acquisition then 0/49/50/51/60 misses, LiDAR-only updates,
  target-absent compatible bars, reset and reacquisition. Bind the clock to accepted fused updates;
  do not substitute geometric or camera loss. Record current validity, age, covariance and historic
  feature availability separately. Retain the 5 s contract; validate strict comparison/dtype/timing.
- A range-only observation cannot provide independent fresh tangential/vertical information;
  compare covariance to the independent measurement Jacobian. Calibration requirements need
  declared measurement errors before PASS. A synthetic coordinate made from a prediction is not
  three independent measured coordinates.
- Join all primary metrics by (seed,cell,env,episode). Require exact population equality with the
  declared quota and separate tails/legacy windows. Define contact/capture overlaps and abort
  separately; preserve original outcome rules in the historical baseline.

Compute estimate <30 CPU minutes, excluding engineering/fixture annotation. Cached real-image
appearance qualification needs independently labelled data and requirements first; realistic
accuracy thresholds and acquisition cost are currently NOT_DEFINED.

## EQ1-D — small simulator qualification only after separate approval

**Proposed additions only:** a standalone diagnostic fixture launcher plus new telemetry module
and preregistration; exact code paths/diff are a later approval package. No PPO or Recovery Planner.

Two bounded batches are suggested, with a **combined proposed 1 GPU-hour cap** and first-failure
stop, no automatic retries. Stage approval may choose a smaller design.

1. Up to 24 deterministic dynamics trajectories × three physics dt values (10/5/2.5 ms), keeping
   action and sensor schedules at the same physical 0.1 s intervals. Hover, velocity step/reversal,
   yaw/tilt, saturation/braking and contact-corner fixtures. Log inner-step pose/velocity/angular
   velocity, command, motor reference/actual RPM/thrust, force/torque and contacts. Compare with
   the CPU/analytic oracle, not two controller helpers. Proposed noncontact convergence gate:
   5→2.5 ms error decreases relative to 10→5 ms, position error ≤0.01 m and attitude ≤0.5° over
   a prespecified comparison horizon. The 0.01 m tolerance is 2% of the 0.5 m capture radius;
   hardware tracking accuracy needs separate calibration and must not be inferred from this gate.
2. Small known sensor scenes with independent target-alone and occluded support, on/off-axis
   landmarks, body tilt versus declared camera mount, target-absent bar controls and stage stamps.
   Save actual RGB/depth/LiDAR ray tables, poses, capture/delivery times, association identity
   provenance in evaluator-only output, covariance/age/history and actor inputs. Evaluate the
   synthetic geometric gates from EQ1-A; actual sensor realism still requires measured data.

Any requested geometric-loss duration fixture is an **environment qualification proposal**, not
an implemented Recovery Scenario. It must first demonstrate acquired-before-loss eligibility and
actual physical obstruction versus FOV/scale/dropout. A future 5 s/longer stress exposure requires
an explicit realized-duration acceptance gate; no new such scene was generated in EQ-0.

Estimate: roughly 0.5–1 GPU-hour total / <1 GB for bounded instrumented batches, **NOT_MEASURED**.
For context only, historical G0 used 0.2472 GPU-hour and OCC-P2 112 production cells used 2.4927 h
wall; these do not benchmark new fixture speed or overhead. Measure first-cell overhead and stop
if the approved cap/design cannot fit. Check free space before/after and stop below 8 GB.

Before any launch: independent CPU gates reviewed; exact files/fixture matrix/source/config and
tolerances approved; new output path; preregistration and code locally committed; checkpoint/source/
imports/raw SHA verified; HEAD/tracked tree clean and frozen. No source or WORKLOG edit during
execution. Failed fixtures retain raw/manifest; no overwrite/delete. Result commit occurs only
after process exit and validation. A new plant/sensor correction requires its own lineage and
comparison, preserving all historical OCC results.

## Decision requested after EQ-0, not executed now

Choose/approve EQ1-A/B/C scope and the intended research sensing/plant assumptions first. Review
the resulting evidence and any proposed correction diff before deciding EQ1-D. No additional
GPU, Recovery Scenario, Belief, Planner, PPO, main merge or remote push follows automatically.
External raw backup remains pending, but read-only CPU qualification need not wait for it.
