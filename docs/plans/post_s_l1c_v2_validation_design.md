# Post-C-v2 validation designs — PLAN ONLY

Authority: user's S-L1C v2 instruction. No B/D production change, GPU run, P-L1, G-L1,
PPO, Recovery Planner or H-R confirmation is authorized by this document. These are designs
for separate review/preregistration; no new qualification is claimed.

## S-L1D realistic belief-error stress

Problem: D-v1 was qualified on C-v1 and some ideal priming fixtures. C-v2 changes association
availability. D API compatibility and non-mutation do not establish validity accuracy on the
new belief distribution. Keep D disabled in `s_l1c_v2` pending a separately qualified combination.

1. Collect evaluation-only camera measurement error, fused track position/velocity error,
   prediction horizon, covariance, identity age and update-source traces on actual RGB-D + LiDAR
   observations. Use full source/checkpoint/config manifests. GT computes error labels after
   recording; it never initializes the deployed track or enters the online validity rule.
2. Split seeds and scenes before deriving distributions: calibration/design and held-out
   validation must share neither scene nor pre-loss prefix. Preserve time correlations and
   camera failure/association modes with blocked sequence resampling; independent Gaussian
   samples are an additional diagnostic. Include empirical error tails, missed camera detections,
   range-only intervals, approaching obstacles and the C-v2 near-range off-row band. Existing
   data may design the stress distribution, not serve as a newly independent test set.
3. Replay identical permitted sensor histories and pre-intervention states through C-v1 and
   C-v2; then evaluate D on their actual beliefs. Stratify by prediction age, covariance,
   radial/tangential error, range/elevation, camera support and foreground geometry. GT-perfect
   initialization is a labelled diagnostic control, never the primary condition.
4. Independent ray/silhouette oracle labels camera measurement validity and obstacle first hits.
   Report false-valid, false-invalid, ABSTAIN/coverage, reliability versus error/age, and eventual
   camera reobservation separately. Do not interpret conditional validity as end-to-end Recovery.
5. Before execution register exact counts, error distributions, baseline coverage, maximum
   false-valid rate and paired noninferiority tolerances. Do not set numerical margins after
   seeing held-out outcomes. Recheck non-mutation, matched prefix, E-L0 provenance, actor GT
   firewall, identity clock/5s expiry and all previous modes. Accuracy failure blocks D-on-C-v2.

Expected first package: CPU trace inventory/oracle and empirical-distribution design (2–4 h),
then a separately approved bounded sensor-only GPU matrix after counts and runtime are estimated.
No current canary budget is treated as approval for that run.

## S-L1B target-return carve retention

Keep the prior [planning record](s_l1c_v2_and_s_l1b_carve_plan.md) and v1 evidence unchanged.
Compare CB-0 frozen B-v1, CB-1 analytic pixel-sampling bound and CB-2 per-beam sphere prediction.
Derive tolerances from the permitted RGB-D/LiDAR geometry, declared measurement error and sampling;
do not use semantic identity to decide what to carve.

Independent CPU first-hit fixtures: prior S14/A_FOREGROUND/B_TIGHT and EQ-1D front-bar case,
plus 0–0.1m foreground gaps, tangent/partial overlap, multiple elevations/bearings, near and far
targets. Pair exact sensor packets and camera candidates. Evaluate target-return retention,
bar false-clear, unsupported carve, ABSTAIN and sensitivity to camera centre/range error.
A target-retention gain cannot compensate for an additional erased foreground obstacle.
Preregister exact matrix and no-additional-bar-loss gate before any new GPU test.

CB-2 depends on the next centre-range investigation; do not assume its centre is unbiased.
The C-v2 GO does not qualify B carve changes. Expected design/oracle package 2–4 h CPU;
production change and GPU canary remain separate approval decisions.

## Camera centre-range bias

Keep A7 and all prior measured residuals as recorded. Audit the complete pinhole/SE(3), candidate
mask/anchor, surface-to-centre conversion and camera/LiDAR proxy-radius chain. Distinguish axial
depth, camera slant range, body-origin centre range and LiDAR surface range explicitly.

Independent CPU analytic sphere/box rays should vary pixel-centred and interpixel targets,
1.16/1.5/2/2.28/2.30/3/6m, elevation and image-centre/periphery bearings, partial silhouettes,
foreground depth and sensor offsets. Compare signed bias, MAE/max/quantiles, missingness,
pixel sampling and propagated covariance using only permitted measurements for each estimator.
Include non-spherical/uncertain-size targets as out-of-model diagnostics.

Select an estimator only after a separate preregistration and held-out matrix, then test GT
firewall, camera/identity recency, C-v1/v2 interaction, B carve and old-mode noninterference.
No current production estimator is changed. Expected CPU characterization 2–3 h; GPU rendering
validation, if needed, will receive its own fixed-source preregistration and approval.
