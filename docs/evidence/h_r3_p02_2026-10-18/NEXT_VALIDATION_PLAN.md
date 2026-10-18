# P02 runner qualification follow-up — PLAN ONLY / NOT EXECUTED

Current decision is NO-GO. P01/P02 source, raw, preregistration and failed outcomes remain intact.
This plan does not authorize any new GPU process, exploratory seed or confirmation experiment.
No production Warp/sensor/controller/checkpoint, PPO, SANDO/Recovery Planner or P-L1/G-L1 change.
RGB-D Camera + LiDAR and FrozenF actor898 remain the information contract.

## First unit: independent CPU geometry/reset isolation

Proposed new paths (not created/implemented): `tools/h_r3_rng_isolation_v2.py`,
`tests/test_h_r3_isolation_v2.py`, and a new dated preregistration under `docs/plans/`.
Preserve all measured P02 paths. Inspect `WarpEnv.reset_idx` global vertex write, selective BVH refit,
PhysX actor-root normalization and derived kinematic caches. A possible instance-only wrapper must
update/refit only the vertices/meshes belonging to selected environments, while original methods,
GT firewall, sensor algorithms and actual own-env reset semantics remain unchanged.

Independent NumPy float64 SE(3)/ray-box or ray-triangle oracle with manually specified two-or-more
scene fixtures: update one scene's actor pose/quaternion, retain unrelated scene geometry exactly,
compare known first-hit ranges. Do not validate by merely recalling Warp's own transform function.
Test nonuniform vertex counts, actor-index mapping, multiple/reordered reset ids, initial all-env
reset, normalized/non-normalized representations, manager/task batching and API failures. Assert
unselected cached kinematics, vertex bytes, BVH-dependent ray first hits and all RNG streams are
unchanged; include source-matched global-rewrite negative fixture. Establish whether the proposed
Warp mechanism actually reproduces the observed sensor-first chronology. Current historical
vertex values are NOT_RECORDED; no retrospective numerical-cause proof is claimed.

Expected implementation/CPU time: 1–2h after scope review. Completion criterion: independent
oracle and focused RNG/legacy/GT-firewall regression PASS, exact diff inspected, local source commit.

## Second unit: preregistered actual-N128 mechanics diagnostic

Proposed new path: `tools/h_r3_isolation_canary.py`, plus focused tests and new output RunID.
Use already observed seed4602 solely for bug reproduction/mechanics, not new inferential evidence.
Normal B0 versus forced unrelated reset *before the first normal reset*, e.g. predeterminedstep2,
env127; assert baseline has no reset then and treatment does. A shared natural reset at the probe
step is an invalid diagnostic, explicitly fail closed. Also test evaluator OFF/ON and six-arm
common prefixes at N128, rather than extrapolating N16. Any alternate probe schedule must be
registered before execution, not selected after seeing a successful result.

Record separate full SHA256 for RGB/depth/LiDAR, state/actor/tracker/history, common exogenous draws,
and exact numeric snapshots of orientations/angular velocity/derived buffers, static actor roots,
selected/unselected mesh vertices and reset/write/refit indices around first mismatch. Keep GT
snapshots evaluator-only. Test both adding a reset and removing/delaying a naturally scheduled
reset in diagnostic runs if separately registered. Physical action-dependent sensor changes after
own intervention must remain; equality applies only to untouched envs/prebranch states.

Gate: zero mismatches in the audited configuration, correct reset/probe chronology, unchanged
checkpoint/RMS/imports/actor898, finite complete telemetry and clean committed tree. Watchdog,
OOM reserve, fresh output paths and disk8GB protections remain. No tolerance relaxation or dropped
cohort to rescue a failed gate. Failed diagnostic blocks further pilot/confirmation.

Expected GPU mechanics budget: roughly10–20min with a hard cap fixed in its preregistration,
not a new main run. Actual process matrix/duration needs approval before execution.

## Confirmation remains HOLD

After qualified mechanics and independent review, review B4 sham-side interpretation and B5's
realized effort imbalance. Choose scientifically justified practical margins and register geometry
strata, spatial identity validity, terminal handling, fixed exposure, matched-scene/seed-cluster
analysis, Holm3 contrasts and joint power simulation. The existing32seed/640event planning example
is conditional only and is not a launch recommendation. No PARTIAL-class effect was measured;
no post hoc exclusion of14 failed primary prefixes or43 failed trigger prefixes is permitted.
No additional exploratory data or141event confirmation is launched before user review.

Post-review planning qualification: model sham-side mixture with evolving informed side and trajectories; do not assume an exact50% episode effect dilution or fixed sample multiplier. Integer-pixel/visible-mask source contracts have been checked (review_source_checks.json); numerical renderer fidelity remains unqualified under the shared-mesh failure. The cited three task writes are already inside the reset transaction, so do not label them a proven bypass.
