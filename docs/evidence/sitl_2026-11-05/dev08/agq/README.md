# AGQ diagnosis and formal-validation preparation — 2026-11-06

**Verdict:** existing-log diagnosis is descriptive; causal contact mechanism and the
corrected model's live response remain **NOT_TESTED**. Source preparation review r2
is **PASS_WITH_LIMITS** after the retained r1 FAIL was repaired. Formal readiness is
**BLOCKED**: the original S3a is NOT_EQUIVALENT and complete target states are absent.

Read [DIAGNOSIS.md](DIAGNOSIS.md) for onset/command details and
[S3A_INPUT_AUDIT.md](S3A_INPUT_AUDIT.md) for the numerical and input controls.
[diagnosis.json](diagnosis.json) binds all numbers below to the raw inputs,
reproduction scripts, CSV/NPZ series, current integrity audit and generated model packages.
The [next-unit proposal](../../../../plans/sitl_agq_followup_v1_2026-11-06.md)
fixes the minimum intervention and the separate S3a/input collection boundaries.

## What changed first

Times are seconds after each run's alignment. Truth and estimator publication share
PX4 hrt time, with estimator interpolation onto the approximately 50Hz truth grid.
Publication timestamps are not exact physical simultaneity. Geometric intersection
is a full3D-pose footprint projection plus vertical-span **proxy**, not a contact log.
No actual Gazebo contact record exists in the old runs.

| Scene / arm | First footprint overlap proxy | First actual yaw rate >5 rad/s | First true z <0.8 m | First position difference >0.5 m | End |
| --- | ---: | ---: | ---: | ---: | --- |
| 205/env2 native | 0.940 | 1.280 | 1.260 | 6.180 | 7.400 crash |
| 205/env2 candidate | 2.736 | 2.736 | 3.336 | 7.076 | 8.320 crash |
| 205/env5 candidate | none | none | 4.080 | none | 13.000 capture |
| 205/env5 native | 5.728 | 5.768 | 5.988 | 6.668 | 7.000 crash |

The env2 candidate's large position discrepancy grows before its final logical
collision, after earlier rotor/base intersection proxies and angular disturbances.
Moving truth publication by ±20ms leaves its terminal difference in
2.655–2.665m.
That shift is a sensitivity test, not a measured bound on clock/estimator error.
The latest ULog sample and the terminal MAVLink snapshot need not have identical z.
The original diagnostic report's approximate100Hz rate label is corrected here:
groundtruth attitude/rates have a median125Hz publication rate in these logs.
Geometry v3 zeroes the ground gate height and omits the settled body-centre offset
of about9cm in its world-z conversion. The independent actual-rounded-world probe
includes that offset and reproduces every intersection sample in these tall-bar
scenes; do not reuse the proxy unchanged for low or non-vertical obstacles.

For native env5, the first footprint proxy precedes the yaw excursion and altitude
collapse. The generated altitude setpoint stayed at1m; PX4's sampled allocator
showed a yaw-torque deficit after the disturbance, with no sampled collective
thrust deficit in this case. This does not prove the estimator initiated the failure.
Do not extend the env5 thrust statement to env2, whose sampled deficits differ.

Control tracking also matters before possible contact. In the fixed-orientation,
constant-velocity0.3s projection at200ms before native env5's first proxy, the last
generated command does not intersect a bar, while actual velocity reaches the
extra rotor envelope after0.170s. Native env2 is similar. Candidate env2's command
and actual velocity both enter the enlarged envelope. These local projections
are diagnostic assumptions, not feasible trajectories or command-safety proofs;
actual send times were not recorded. The sampled logged PX4 trajectory_setpoint request and
the generated checkpoint command are reported separately. The logged trajectory_setpoint
is upstream of position-controller shaping; this diagnosis does not reconstruct the
downstream shaped local-position setpoint series.

The env5 candidate has no sampled intersection proxy or >5rad/s actual body-rate
excursion, but reaches62.2° tilt. Thus capture does
not qualify control matching. The45° PX4 tilt parameter limits setpoints, not
transient actual attitude. No gain bundle was selected or causally credited.

## Repair prepared

The training URDF has only the base collision box. The Gazebo SDF additionally
contains four rotor cylinders of radius0.128m at
±0.13m offsets; the level outer radius reaches about0.312m.
The old logical bar test instead used a0.14m disc. This is an established model
contract mismatch independently of the unresolved contact cause.

`tools/sitl_training_geometry.py` exports a fresh external model package:
`native` is byte-identical; opt-in `training-box` removes exactly the four extra
collision elements. All other SDF bytes and all dynamics/gains/sensors stay intact.
The plan adapter changes only the spawn SDF and adds its receipt. Dry runtime
binding comparison differs only at `model_sdf`. Nothing was installed or launched.
No existing runtime launcher, termination rule or historical result was edited.

## Remaining formal prerequisites

The structural audit now requires every scene's finite contiguous states0..600,
valid wire dtype, matching context/receipt identities and internal/external hash
bindings. It never authorizes formal GO. Current missing states total
412:70/env5 lacks208..600; the other19scenes lack frame600.
Old hidden target/RNG/reset state is unavailable; a new full-horizon collection
needs a new evidence lineage rather than a fabricated continuation.

On identical stored observations, CPU batch16 versus singleton differs by at most
5.36e-07/8.64e-07
(70/205bars), while CPU versus stored GPU actions differ by
0.00158533/0.00101218.
CPU singleton-versus-batch16 does not explain the residual; GPU batch/kernel
effects remain unisolated. TF32's contribution still needs its own GPU control. Target-velocity outliers are consistent with a one-pixel raster
boundary changing measurement covariance. GPU pixels were inferred from posterior
values, not directly recorded. A float64 discriminant probe repairs only one of
two cases; no detector/KF retuning was deployed and old S3a thresholds stay fixed.

## Validation and evidence limits

[Source review r1](SOURCE_REVIEW_r1.md) retained one MAJOR and one MINOR in the
initial new utility code. Identity/schema/internal-binding checks and canonical
model.config verification were repaired. [Source review r2](SOURCE_REVIEW_r2.md)
independently accepts the final four source/test files with no open findings.
It covers offline structure only, not the proposed live protocol or layout physics.
The source review's17focused tests and independent mutants are separate from the
root PUBLIC_CPU validation recorded in `validation.json` at closeout.

All507original dev08 raw files/697582825bytes were rehashed successfully;
canonical F and all legacy execution source bytes are unchanged. Kernel faults
remained0→0during this integrity check.
The original S3a utility manifest deliberately retains its pre-repair source hashes;
the current integrity record binds the repaired utility separately. No old raw or
scientific probe was rewritten. Initial bounded-probe MemoryError, XML test-fixture
failures and the dry-binding missing-world refusal remain in the external unit.

External unit:`/home/fair/motar_tmp/agq_diagnosis_20261106`. The plotted [timeline](/home/fair/motar_tmp/agq_diagnosis_20261106/agent_analysis_r1/presentation_v3/failure_timeline.png)
and [SVG](/home/fair/motar_tmp/agq_diagnosis_20261106/agent_analysis_r1/presentation_v3/failure_timeline.svg) are standalone diagnostic artifacts.
This is not a formal acceptance, real-sensor result or flight-readiness claim.

[Independent evidence review](RESULTS_REVIEW_r1.md): **LIMITED_VALIDITY** for descriptive diagnosis, no BLOCKER/MAJOR. It independently reproduces the four onset/geometry/parameter records,48constant-velocity projections, S3a numerical controls and missing-state count. Its four wording/geometry clarifications are reflected above; actual contact causality and new model response stay NOT_TESTED. Initial reviewer fixture/binding failures are preserved in the external audit.

Root [validation](validation.json): PUBLIC_CPU3451 OK/10skip, faults0→0 and source hashes stable. Public docs PASS; staged experiment-index9/9 OK. No simulator/GPU run was performed in this unit.
