# S3b dev08 candidate gains v1 — preparation, 2026-11-05

**State:** configuration prepared; experiment **NOT_RUN**. The user authorized handoff,
planning and implementation preparation, CPU verification and its local record. No SITL/GPU
measurement, later gate, checkpoint replacement, push or main merge is authorized here.
This is a development sensitivity protocol, not formal S3b acceptance. S3a remains
NOT_EQUIVALENT. Future amendments get a new file; preserve this protocol after commitment.

## Question and fixed treatment

Does dev07's descriptive outcome agreement depend on the chosen PX4 velocity/yaw gains?
The choice comes from the existing exploratory `tune_velloop/tune_summary.json`, not a new
gain search. Frozen F, its 898-D RGB-D + LiDAR contract, bridge, governor, 10Hz simulator-time
loop, altitude/termination rules and ODE iterations100 stay fixed.

| PX4 override | Native/dev07 override map | Candidate |
| --- | --- | --- |
| MPC_XY_VEL_P_ACC | 2.5 | 2.0 |
| MC_YAWRATE_P | not explicitly overridden | 0.2 |
| MC_YAWRATE_I | not explicitly overridden | 0.1 |

Native yaw values were not recorded as effective parameter readbacks; do not substitute an
assumed PX4 default. All other entries in the historical PARAMS map are identical. Command yaw
is `truth`, matching the historical reports; inherited environment overrides must not change it.
No hold/delay or chunked policy is introduced. The effect is this bundled three-gain contrast;
individual gain effects cannot be identified.

## Inputs and comparison

Use the recorded dev07 selection: seed4501, densities70/205, env0–9, episode index0 in each cell.
Twenty candidate episodes are proposed, without replacement or a changed denominator.
Historical comparators are
`docs/evidence/sitl_2026-11-05/s3b_dev/dev07_70_s3b_report.json` and
`docs/evidence/sitl_2026-11-05/s3b_dev/dev07_205_s3b_report.json`.

Raw root:
`/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/sitl_ref_v1_r2/g8/`.
For each density use `full/E2_obstacle_aware__<bars>bars__seed4501` for the initial state,
bar layout and Isaac outcome, and `zero/E2_obstacle_aware__<bars>bars__seed4501` for the
recorded target path. These are the exact historical launch paths, not a new G8 rerun.
Full and zero targets coincide on the selected common frames; this does not supersede G8's
TARGET_DEPENDS_ON_PURSUER result or establish an exact F counterfactual.

The new `tools/sitl_s3b_dev08.py` is deliberately a **configuration preparation tool**, with
native defaults and opt-in `--arm candidate`. It parses the original runner as data, checks
pinned source/report SHA-256 values and emits JSON. It never imports that runner, the bridge,
Isaac, pymavlink or F, and has no execution option. Native configuration equality is not a
functional episode/output identity result. Runtime implementation remains open in Beads.

```bash
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 python3 -B tools/sitl_s3b_dev08.py
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 python3 -B tools/sitl_s3b_dev08.py --arm candidate
```

Hashes, exact cell/asset paths and the read-only raw inventory are retained in
`docs/evidence/sitl_2026-11-05/dev08/`. Preserve the historical runner and bridge bytes;
runtime work must be additive.

## Input blocker discovered before measurement

The zero-action 70bars/env5/episode0 path has **208 frames**, not600. Its ledger records a
capture at completion_step208. The other nineteen selected zero-action paths have600 frames.
The historical dev07 report explicitly records208 and captured at step35; no tail exhaustion
occurred in that episode, so this finding does not invalidate or rewrite dev07.

The proposed runtime preflight requires600 contiguous finite target frames from frame0 for
every selected episode before any SITL launch. Therefore this exact20-episode protocol is
**BLOCKED_INCOMPLETE_TARGET_TRACE**. Do not hold the last target sample, extrapolate,
substitute another episode, drop env5, shorten the horizon or regenerate data under this
preparation authorization. Resolving the missing path needs a separately recorded input
decision/amendment and, if it requires GPU measurement, new scoped approval. Keeping the
blocker explicit is part of preparation completion, not an experimental verdict.

## Future runtime requirements, before asking for execution approval

These are requirements for the later implementation, not features claimed by this tool.

- Keep candidate overrides OFF by default; prove the additive native path reproduces the
  legacy command/episode behavior with non-vacuous CPU fixtures before selecting candidate.
  Any new SITL native smoke is itself a measurement needing the same scoped execution approval.
- Assert exact20-member selection, both densities/seed and matching layouts, starting pose,
  complete target coverage and receipt contracts. Reject missing envs/targets rather than the
  legacy scan-forward/fallback behavior. Verify hashes of actual input payloads at launch.
- Read back each applied PX4 override with bounded acknowledgement waits and persist effective
  yaw gains for both modes. Use a fresh isolated PX4 parameter state per episode so saved
  parameters cannot silently carry over. Never edit the installed airframe to apply treatment.
- Bind source HEAD/clean status, F identity, bridge/production dependencies, trace/layout hashes,
  PX4 and Gazebo versions, executable and vehicle assets. Record the live generated SDF and
  airframe; they currently match dev07. The jinja lacks the added rangefinder: prohibit
  regeneration until a separately reviewed asset decision, and preserve both original files.
- Own an exclusive SITL lock/UDP14540 and unique per-attempt world paths. Fail if another instance
  exists. Cleanup signals only recorded owned descendants, with PID start-time checks; never
  inherit the legacy user-wide `pkill`. Remove only newly created owned temporary worlds.
- Use 10Hz PX4 simulator-time scheduling. Reject telemetry gaps/non-finite states, a stalled
  simulator clock, failed start checks or unreadable inputs. Abort the unit on HARNESS_INVALID,
  ERROR, trace exhaustion, resource stop or host fault; retain all attempted output, do not
  silently replace episodes. Report actual RTF/compute time, with no new wall-clock claim.
- Use a fresh output root per attempt, proposed
  `/home/fair/motar_tmp/sitl_s3b_20261105/dev08_attempt01/` (must not exist at launch).
  Put candidate density outputs, receipts, host/resource checks, steps and reports beneath it.
  New attempts increment the suffix. Historical outputs and main `results/` remain untouched.
- Bound the whole process tree: cores6,7,12,13, nice10, CUDA hidden and
  PYTHONNOUSERSITE=1, limited thread counts. Use the already tested RSS watchdog at12GiB,
  poll0.25s (sampled bound, not kernel hardcap), a2400s timeout per density and5400s overall,
  plus `timeout -k20`. An episode watchdog must abort after240s and reap owned descendants.
  Keep an idle host, inventory other jobs and compare kernel fault counts before/after.
  Any increased machine-check/crash count voids the attempt; do not retry until the host is fixed.
- Commit source/protocol and review before launch, and freeze all tracked files during the run,
  including docs/WORKLOG/tests. An output receipt records the exact approval scope and launch
  interval. No command-line flag itself supplies user approval.

## Fixed descriptive analysis and stop rules

For each density report all ten planned candidate episodes alongside their historical dev07
and Isaac outcomes, including the historical205bars/env8 crash. Report paired counts and outcome
transitions, capture/crash/timeout counts, start-check validity, per-episode steps and RTF;
for episodes captured by both systems report candidate-minus-dev07 and candidate-minus-Isaac
step differences (n, median, minimum, maximum). Missing/unrecorded values stay explicit.
Do not pool an incomplete cohort into a rate or infer safety/noninferiority from this sample.

No gain ranking, acceptance threshold, significance test, S3b PASS, transfer or real-flight
claim follows. A complete valid run is a descriptive development record only. A failure remains
recorded; no posthoc thresholds or automatic repeat. Results need independent review before
any positive gate decision. A new formal S3b protocol is separate work.

## Verification and next boundary

CPU checks accept intact historical inputs/native settings, verify the exact three-gain delta,
reject drift in each pinned input and altered timing constants, and ensure no imports/environment
mutation or execution CLI. Run them in isolation and the shared PUBLIC_CPU suite. A fresh-context
read-only reviewer examines the actual code, tests, raw-input inventory and this protocol, with
its report in the existing SITL unit's `dev08/` subfolder.

Current completion means preparation/configuration is verified and locally committed while
measurement stays NOT_RUN and runtime/input blockers stay open. The next action is an additive
runtime/input design resolving the208-frame blocker; execution approval is requested only after
the resulting concrete implementation, review and amended inputs are ready.
