# Dev08 amendment1 — observed-prefix development design, 2026-11-05

Parent: [`prereg_sitl_dev08_candidate_gains_v1.md`](prereg_sitl_dev08_candidate_gains_v1.md).
User authorization: proceed with the related issues/design and prepare the next stack.
This covers offline contracts, input verification, design, CPU tests, independent review and
local records. Runtime integration, SITL/GPU measurements, a later formal gate, push/main merge
and real flight remain outside this unit. No measurement preceded this amendment.

## What this amendment changes

The parent strict600 plan remains immutable and BLOCKED_INCOMPLETE_TARGET_TRACE. This amendment
defines a separately named, opt-in **observed-prefix-v1** development diagnostic. It changes
the observation support, stopping/reporting semantics and proposes a fresh paired native arm;
it does not claim to supply the missing target trajectory or unblock a600-action experiment.
Original source, outputs, failure verdicts and Frozen F remain unchanged.

For all raw paths, frame k is the state **after k completed actions**, recorded when that
observation is consumed, before action k. A ledger completion_step is a post-action count,
whereas the legacy SITL report's steps is an observation-check count. Retain these raw fields.

| Target data | Admitted indices | Last scheduled state check | Commands before that check |
| --- | --- | --- | --- |
| 70bars/env5,208 positions | 0..207 | nominal20.7s | 207 |
| Other nineteen,600 positions | 0..599 | nominal59.9s | 599 |

Actual simulator timestamps are recorded separately from nominal k×0.1. In particular,
the zero-action ledger's capture at208 does not record frame208. A true endpoint after600
actions would need frame600, hence601 state/target positions0..600; it is absent in these data.
No final position is copied forward or inferred. These facts do not invalidate historical
dev07: all its physical terminal checks are within the available prefixes. Its env5 steps35
means35 checks, final index34, nominal3.4s, not35 measured completed transitions.

## Fixed cohort, treatment and proposed future unit

Keep seed4501, densities70/205, env0–9/episode0 and the exact full/zero inputs from the parent.
Preserve the twenty scene identities, including205/env8's historical crash. Use the original
native override map by default, with the same three candidate gains in an explicit opt-in arm.
Effective native yaw gains require fresh parameter-state readback; do not assume their values.
Frozen F,10Hz, ODE100, bridge/sensors, governor, physical capture/crash thresholds and truth
command yaw remain fixed. Per-bar geometry must be qualified before runtime; hashed NPZ
payloads alone do not prove the world writer handles quaternion geometry correctly.

Proposed execution scope, only after runtime completion/review and new explicit approval:

- Two native smoke episodes: env0 at each density. These are not main-analysis observations.
  Smoke must complete a valid physical terminal or valid prefix censor with all operational
  guards satisfied. A smoke harness failure stops before the main phase.
- Twenty main pairs: native and candidate on every fixed scene,40 main episodes. Density70
  precedes205; within each, env0..9. Even envs run native then candidate; odd envs candidate
  then native. The order is fixed before observing any response; no outcome-dependent reruns.
- Total proposed42 episodes. Each starts a fresh simulator/rootfs and parameter state. Never
  reuse the smoke output as a main record, substitute a failed pair member or mix protocols.

Fresh native comparisons are necessary because this runtime changes lifecycle/stopping from
the legacy runner. Native/candidate share the new lifecycle; only their three gain overrides
differ. Historical dev07 remains a descriptive compatibility check, not the primary paired
control or a randomized experiment. No execution is authorized by this proposal.

## Checkpoint and stopping contract

The pure offline function `checkpoint_decision` in
[`tools/sitl_dev08_prefix_contract.py`](../../tools/sitl_dev08_prefix_contract.py) specifies:

1. Admit exact target[k] only for the fixed contiguous prefix. Never clamp an index or fall
   back to another episode/target. Wait for the scheduled simulator checkpoint; validate
   source, coherent fresh telemetry and the timing rule before using any state.
2. Apply unchanged physical terminal geometry to the current snapshot, with **capture before
   crash** as in legacy when both predicates hold. Neither predicate is supplied by a new
   ground-truth actor input. A terminal on the last supported point wins over censoring.
3. If no physical terminal occurred and k is the final supported index, record
   **CENSORED_TRACE_END**, final physical outcome UNKNOWN, and end this scored window.
   This also applies at frame599 on the600-position paths. Never call it timeout600.
4. Only on CONTINUE compute the policy/governor command and issue the next control interval.
   No command is issued at a physical terminal or censor checkpoint. Thus final_frame_index=k,
   observation_checks=k+1 and commands_issued=k for a valid record starting at frame0.
5. Cleanup/hold occurs outside the scored prefix and is recorded separately. Its movement is
   not another target sample or scored terminal. Preserve the original runner's redundant
   post-check commands in its historical source; do not claim full lifecycle identity with it.

Expected administrative censoring is a valid prefix record and permits the remaining planned
episodes to proceed. A hash/schema/layout/telemetry/start/parameter mismatch, clock stall,
missing unexpected frame, resource stop or host fault is **not censoring**: abort the unit,
preserve the failed attempt and label incomplete/invalid. No summary may hide an unrun member.

The offline helper currently accepts terminal booleans and enforces counts. It does not
calculate physical collision geometry, verify live timing or prove commands were issued.
Those are mandatory runtime boundaries with their own CPU controls before any launch.

## Fixed analysis and unknowns

The main ledger must contain exactly forty unique phase=main records: both arms for all
twenty scene identities. Report ten planned scenes per density and twenty per arm; smoke
records never enter these denominators. Every expected member remains visible.
Each record names seed4501/episode index0, observed-prefix-v1, a VALID operational status, finite nondecreasing actual
simulator start/end timestamps, scheduled_elapsed_s=k×DT and the exact count fields above.
The analyzer rejects shortened windows, wrong protocols and failed/unverified status. Scalar
timestamps/status labels alone do not prove per-step freshness, pacing or parameter guards;
the live boundary must establish and bind those checks before producing a VALID record.

Report observed captures C, observed crashes R and valid censored nonterminal paths U, with
N=C+R+U fixed at20 per arm (10 per density). C/N and R/N are **observed terminal fractions**.
They are not a complete60s success/crash rate. Do not discard U or classify it as a timeout.
Under the bookkeeping assumption that physical terminal events are absorbing, logical
compatibility bounds are capture[C,C+U]/N and crash[R,R+U]/N under arbitrary unobserved
continuations. They are not confidence intervals, measured full-horizon results, identified
future trajectories, causal effects, noninferiority or safety bounds. They may be uninformative.

Report paired outcome transitions, how many pairs contain censoring, per-density counts, and
candidate-minus-native nominal capture-checkpoint time (n, median, min, max) only for pairs
physically captured within both admitted windows. Store actual simulator elapsed time too;
nominal checkpoint differences do not establish real-time latency. Compare historical dev07
check indices only with its preserved steps convention; do not mix those with Isaac action
completion counts as though they were identical timestamps. No hypothesis test or gain
selection follows. An incomplete/invalid ledger refuses the complete-pair summary.

## Runtime stack and acceptance interfaces

This is the implementation sequence, with task state held in Beads rather than this document.
The offline input/checkpoint/ledger contracts are implemented here; all live boundaries remain
NOT_IMPLEMENTED and the tool reports execution_supported=false/launch_ready=false.

| Component | Required interface/behavior | Verification before live integration |
| --- | --- | --- |
| Input contract | pinned receipts/source/raw -> exact20 specs and prefix limits | real read-only hashes/counts; finite/contiguous indices; wrong/hash/missing/duplicate positive and rejection controls; layout geometry qualification still required |
| Owned session | context manager -> own PX4/gzserver/spawn handles, per-episode external rootfs/world, exclusive UDP14540 lock | fresh/no-overwrite paths; contention fails; unrelated sentinel survives normal/error/timeout cleanup; PID start-time tracking and reaping |
| Parameter/telemetry boundary | bounded source-matched echo/readback and coherent finite state snapshots | wrong/missing/stale echo fails; actual fresh native yaw values saved; all packet freshness/clock skip/stall checks accept valid fixtures and reject mutants |
| Policy/command step | native legacy transform on admitted nonterminal checkpoints, candidate overrides only | identical CPU observation/action/setpoint prefixes for native; same DT/frames and exact three-gain delta; terminal/censor sends no extra command |
| Attempt recorder | two smoke + fixed40 main sequence, append-only receipts, source/platform/host/resource binding | aborted/unrun pairs stay invalid; sampled RSS/timeout/start/host guard controls; no runtime starts from preflight |
| Analyzer | validated complete main ledger -> observed counts/pairs and unknown bounds | hand-counted complete/censored controls; all-censored remains unknown; missing/duplicate/premature censor/error/timeout/post-terminal command rejected |

The installed PX4 `sitl_run.sh` itself calls `pkill -x gazebo` and uses shared
`build/px4_sitl_default/rootfs`; rcS imports parameters.bson from that working directory.
Replacing Python stop_sitl alone is insufficient. The new session must directly own its
launches or use a reviewed additive launcher, with an external fresh rootfs per episode.
Do not invoke the historical launcher unchanged, edit installed airframes or regenerate the
rangefinder-mismatched jinja. Bind both source and installed build/etc rcS/airframe, actual
PX4/Gazebo binaries, original SDF and environment/plugin paths in the runtime receipt.
Exact direct-launch arguments/rootfs isolation must be implemented and independently reviewed,
not inferred to work from this offline design. No PX4/Gazebo process was launched here.

## Runtime bounds, failure preservation and approval boundary

Use new outputs, proposed
`/home/fair/motar_tmp/sitl_s3b_20261105/dev08_prefix_attempt01/`, with a unique per-episode
subdirectory and externally generated worlds/rootfs. Refuse existing attempt/output paths.
An attempt records protocol ID/hash, approved scope, clean source HEAD, all inputs/assets,
F identity, boot/topology, start/end host fault counters and actual source-freeze interval.

Keep CUDA hidden, PYTHONNOUSERSITE=1, idle host, cores6/7/12/13/nice10, bounded thread counts,
12GiB aggregate process-tree sampled RSS watchdog at0.25s (not a kernel hardcap),240s per
episode,2400s per density main block and5400s overall, plus timeout kill grace20s. These
caps can abort a slow unit; they do not guarantee completion or authorize the42-episode scope.
Budget source/review/analysis time and the existing root session clock before proposing launch.

Source/protocol and independent runtime review must be committed before any scoped execution
approval/launch. Freeze all tracked files during a guarded/SITL run. Abort and retain any
host-fault/resource/harness attempt, do not automatically repeat. Subsequent result analysis
and independent review produce a descriptive development record only. Formal S3b, later
stages and real flight need their own protocol and authorization.

## Remaining qualification sequence

This design is not formal stage advancement. The remaining work is: implement/review the live
stack above; separately approve/measure the prefix diagnostic; resolve S3a target-relative-
velocity and CPU/GPU action differences without widening frozen tolerances; qualify full
endpoint inputs and preregister formal S3b acceptance; then staged S3c dynamics/delay, S4 real
Gazebo sensor paths and S5 actual vehicle/sensor/compute/emergency-stop readiness. PX4 asset
template consistency and actual downward-sensor availability remain requirements. GR1b's
NOT_CONFIRMED rule still prevents automatic further gain-randomization PPO variants. Historical
local-asset gaps keeping PUBLIC_CPU red remain separately tracked, without fabricated repairs.
