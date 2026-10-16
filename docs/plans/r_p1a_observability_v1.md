# R-P1a: post-acquisition observability — implementation approval proposal

Status: **PROPOSED; implementation not authorized or applied; new tests NOT_RUN.**
Base: `c038ba50193fb6401a473d8eb9ff4be097883e0e` on `codex/mt-p1`.
This proposal prepares measurement of a previously acquired target lost behind an obstacle.
Initial search, prediction/belief, scenario generation and recovery planning are separate work.

## Scope and exact files

R-P1a has two sequential implementation milestones within one proposed package. Finish and
review the CPU event contract before connecting the geometry probe. GPU execution remains a
separate approval even if implementation and CPU verification are approved.

| Milestone | File | Proposed change |
| --- | --- | --- |
| 1: events | NEW `motar_eval/recovery_events.py` | Independent immutable scalar input records and per-episode event intervals; no simulator or planner dependency |
| 1: integration | MODIFY `aerial_gym/task/navrl_task/kroc_eval_adapter.py` | Default-OFF recovery-events flag; stage detached snapshots, consume once before an action, close/censor at episode end or shutdown; separate sidecar |
| 1: CPU tests | NEW `tests/test_recovery_events.py` | Acquisition eligibility, chronology, independent loss causes, expiry and censoring |
| 1: CPU tests | NEW `tests/test_recovery_observability_contract.py` | Stub/AST integration, OFF identity, RNG and input immutability, no control readback |
| 1: CPU tool | NEW `tools/recovery_observability.py` | Validate recorded event traces, timestamps, joins and run-length summaries; no launch or rollout command |
| 2: geometry | NEW `aerial_gym/task/navrl_task/recovery_visibility_probe.py` | Evaluation-only scene-mesh centre-ray query using the actual renderer camera pose and mesh IDs; own output buffers |
| 2: CPU tests | NEW `tests/test_recovery_visibility_geometry.py` | Analytic reference geometry, camera transforms, supported/unsupported model contracts, timestamp identity |
| record | APPEND `WORKLOG.md` | Implementation diff, measured CPU checks and unresolved runtime validation |

The existing recorder lifecycle is sufficient: `on_reset`, `after_perception`, `before_action`,
`on_episode_end`, `finalize`. No changes are proposed to `navrl_task.py`, `navrl_perception.py`,
`navrl_detector.py`, the existing acquisition-events schema, frozen Policy F, the 898-D actor
layout/order/normalization, tracker memory/F/Q, target dynamics, governor, reward or termination.

The companion `.draft.diff` is an **unapplied event-core design draft only**. It is NOT a complete
implementation patch: integration, the mesh probe, trace tool and tests are absent. It must not
be applied as a finished package or described as tested. The JSON companion enumerates the whole
proposed scope for implementation approval.

## Event contract

- Eligibility requires an earlier valid camera-started track. A never-acquired hidden start is
  initial search and does not start a recovery interval. LiDAR alone cannot establish the prefix.
- Record camera measurement loss, fused measurement loss, track inactivity, centre FOV exit,
  range exit and actual scene-mesh centre-ray blockage as independent signals. Multiple causes
  may coexist. Do not force one exclusive cause from final detector validity.
- The primary occlusion candidate requires prior acquisition, known in-FOV/in-range geometry,
  actual mesh centre-ray blockage and its own measurement-loss chronology. A centre ray blocked
  while visible target pixels remain is not full visual occlusion.
- Keep camera/LiDAR/fused validity aligned with final diagnostics after their existing latency,
  association and error gates. Missing sources stay `NOT_RECORDED`; validate their OR identity
  only under a source-checked contract. Configured dropout probability is not a sampled-dropout
  event. Stochastic dropout versus threshold/appearance rejection is currently `NOT_RECORDED`.
- Expiry requires an actual active-to-inactive tracker transition in the same episode plus the
  observed expiry-age contract. An inactive track with unknown/inconsistent age is an unclassified
  deactivation. Never change tracker state or independently expire it from an evaluator timer.
  The implementation uses strict `age > memory_s`; retain its float32 boundary behavior and
  record actual age. Reset and terminal cleanup are not expiry events.
- Join final capture/crash/timeout to each episode. A shutdown is a censored run, not a timeout.
  `safe_abort` is `NOT_RECORDED` until an explicit real abort reason is available; do not infer
  it from a stationary or collision-free command.

API draft: `FrameStamp`, `SensorFrame`, `EvaluationGeometry`,
`RecoveryEpisodeEvents.observe/finish`. Input records are copied immutable scalars. Geometry
is evaluation-only and has no output path to an actor, tracker, supervisor or command.
Sensor/track resumption is not proof of persistent physical target identity. Record that limit
explicitly rather than treating a successful association as GT-verified identity.

## Time and geometry requirements

Stage the snapshot after perception and consume it exactly once before its action. Keep an
episode key, consumed frame ID, episode-local physical state time, and sensor capture time when
actually recorded. Match geometry to the actual camera `_origins`/`_orientations`, mesh IDs and
target state of that render; refuse or report UNKNOWN for stale or mismatched frames. A delayed
measurement's state time and capture time are different fields. Do not invent an unavailable
capture timestamp. Externally joined legacy acquisition indices remain explicitly 1-based;
the new draft's consumed frame index is 0-based with reset observation at physical time zero.

Intervals end on observed resumption or are censored at the physical terminal boundary after
the final action, before reset. Duplicated observations cannot double-count time. Reset starts a
new episode instance and clears eligibility. Prefix results must not depend on a later suffix.
Bind sidecars to source/checkpoint, motion lineage, seed and the existing episode key. Reserve
scene-manifest and pre-loss actor-trajectory-prefix hash fields for matched R-P1b scenes; absent
values remain `NOT_RECORDED`. An event-prefix hash or the same seed label is not proof of the same
scene and pre-loss actor/action prefix. R-P1b must establish those stronger identities before
comparing interventions.

The current adapter's AABB centre-ray visibility is a proxy, not a validated actual-mesh LOS
oracle. Milestone 1 must emit UNKNOWN geometry until milestone 2 has a supported probe. The
probe reads actual renderer inputs and allocates separate evaluator outputs; it must not rerun
the existing detector or edit its buffers. Initially scope the probe to the current analytic
virtual target with frozen scene geometry. Unsupported physical/dynamic target geometry and
decoupled camera models fail closed or emit UNKNOWN rather than silently using a proxy.

An existing target mask reports visible target pixels. Zero can also arise from FOV, range or
subpixel support, so it cannot by itself establish full occlusion. Full/partial silhouette
classification requires a target-alone support raster with matched rays and an additional
explicit contract; until then it remains `NOT_RECORDED`. These GT fields are metrics only.
No synthetic depth, semantic mask or GT target state enters any online recovery supervisor.

## Planned CPU acceptance — not yet executed

1. Previously acquired versus never-acquired prefixes, both stationary and moving scalar traces;
   independent camera/fused losses, LiDAR coasting, mixed FOV/LOS causes and unknown geometry.
2. Strict expiry boundary cases at reported ages 5.0 and 5.1 seconds, actual float32 drift cases,
   active-to-inactive transitions, reset without expiry, and invalid age/source diagnostics.
3. Duplicate or skipped frame IDs, stale/missing capture stamps, non-monotonic state time,
   latency alignment, reset after partial loss, competing capture/crash/timeout and shutdown
   censoring, and absent safe-abort provenance.
4. Identical prefixes with different suffixes produce identical prefix records. No Python,
   NumPy or torch RNG consumption; inputs and all pre-existing task/perception/tracker buffers,
   checkpoint bytes and parameters remain unchanged. Only monitor-owned state may change.
5. OFF mode constructs/imports no probe and preserves existing outputs and sidecar bytes.
   ON mode on CPU stand-ins preserves all supplied action/observation/reward/termination values;
   AST/import checks reject any event-output readback into control or actor observation.
6. Independent segment/mesh-reference scenes and actual camera transform fixtures distinguish
   clear, blocked, FOV exit, range exit and partial-support ambiguity. Unsupported geometry
   stays UNKNOWN. Existing PUBLIC_CPU and KROC regression checks must pass after implementation.

CPU mocks at 2/5/10/15 seconds test timestamp arithmetic only. They do not prove natural
occlusion-duration feasibility or simulator trajectory noninterference. Actual Warp/scene parity
and matched GPU on/off trajectory traces remain `NOT_TESTED` until a separate GPU approval and
preregistration. No GPU run is requested in this implementation package.

## Completion, estimates and blockers

Milestone 1: event contract, guarded adapter hooks, CPU trace tool and tests pass; geometry is
explicitly UNKNOWN without a probe. Estimated development/CPU verification effort: 10–15 hours.
Milestone 2: supported exact-scene centre LOS adapter and CPU reference tests pass; unvalidated
runtime parity remains labeled `NOT_TESTED`. Estimated effort: another 8–12 hours. These are
engineering estimates, not measured runtimes or schedule guarantees.

Current blockers to a completed empirical occlusion measurement are the actual-mesh probe,
sensor capture-time provenance where absent, full-silhouette support if that stronger label is
needed, and separately approved runtime parity/noninterference evidence. R-P1b scene generation
(including 2/5/10/15-second natural occlusions and the unconfirmed 4–12 m feasibility candidates),
R-P1c belief/prediction and any planner are outside this package. Do not use existing A/B
first-acquisition results as recovery validation.
