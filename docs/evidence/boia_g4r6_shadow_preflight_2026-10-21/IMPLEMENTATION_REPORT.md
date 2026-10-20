# G4-R6 — passive evaluator implementation and CPU qualification

2026-10-21 KST. Base a9300660fbf1917c8431f060ddda9253f523ead1,
branch codex/mt-p1. **GPU/Isaac execution, online control, push and merge NOT_RUN.**
Production, Frozen F, actor898, controller/governor, existing BOIA/research modules,
Raw/checkpoints and historical evidence are unchanged.

| File | Actual implementation | Reason / CPU evidence |
| --- | --- | --- |
| motar_eval/boia_shadow.py | TensorBytes, strict SensorValues/SensorPacket/JoinKey/Binding/SensorTime | Detached cloned bytes; no Tensor storage or mutable handles retained. Shapes, units, tags, clocks, whitelist and GT poison tests. |
| same | diagnose | Existing research build_hypotheses with unavailable calibration; preserves all four hypotheses. Utility ABSTAIN, safety unavailable, selector NOT_EVALUATED, physical NOT_ASSESSED, ownerF/appliedFalse. No synthetic loss tables. |
| same | Recorder / EventLogger | Every-action protocol and copied action/command, sampled sensor/diagnostic logs, separate evaluator namespace, thresholded episode contact, terminal/censor/reset, bounded snapshots and fatal I/O handling. |
| same | EvaluatorTap | Instance call-through wrappers around existing observe/render/capture/reset/step/transform/compute/post. Original calls/args/results unchanged; hooks restored on exit. No action sink exposed to BOIA/logger. |
| tools/boia_shadow.py | preflight / validate_plan / process_env / guard_sources | CPU-only fixed config, exact commit/clean tree/tracked files, import resolution, checkpoint hash and disk guards; inherited runtime knobs scrubbed. No checkpoint unpickling/device queries in preflight. |
| same | run_stage / worker / compare_qualified | Actual N1 evaluator launcher with native F RNG, three bounded stages, watchdog/resource/partial receipt/manifest handling and first-mismatch exact OFF/ON comparison. GPU imports are lazy inside supervised worker. |
| tests/test_boia_shadow.py | PacketTests / ProtocolTests / CallThroughTests | Copy/firewall, metadata, clock/frame, sequence, exactly-once, original F/PI outputs, RNG, contact/terminal/reset and final returned-packet parity CPU fixtures. |
| tests/test_boia_shadow_runner.py | RunnerTests | Source/config/environment, native InitialViewSpec/schema, real-source AST seams, no implicit launch, stand-in watchdog/resource/partial failure, receipt binding and exact parity. |
| docs/plans/boia_shadow_q01_v1.json / .md | Executable configuration / prospective preregistration | N1/4901/moving70bar/OFF/10m, fixed stage limits and inference/resource/failure contracts before GPU measurements. |

## Actual lifecycle and noninterference boundary

1. Install wrappers before the runner's explicit env_reset. Initial reset assigns
   episode0 and stages observation generation0. Native sensing still runs once.
2. Observe wrapper calls the original exactly once with unchanged arguments,
   clones permitted sensor inputs and returned diagnostics, and returns the exact
   original result. Returned camera flag is used: task camera flag is still stale
   until its caller updates it. No get_return_tuple/observe/detector re-evaluation.
3. Original player infers once. At task.step entry, read_online checks current
   state/track/camera against the staged packet, then joins the actual F action to
   immutable episode/action/observation/decision IDs. Original transform runs once;
   returned governor/altitude-PI command is copied before physics, never replaced.
4. ContactChain preserves the existing pre/post substep callback. Before original
   compute, copy raw contact/height/OOB predicates; after it, copy capture/resolved
   crash. Before post-reward/reset, read actual truncation and record old action.
5. Original manager reset/render, task reset and next observation continue normally.
   Final parity includes the returned observation/actor/history/obstacle/target/RNG
   boundary, including after the last terminal action. Reset-only corruption cannot
   escape comparison just because the process stops at that episode.
6. Logging/diagnostics stride5 never alters bookkeeping or native history_stride.
   Observe/renderer/transform/compute counts are checked; LiDAR captures must be
   one per normal step and two on the audited reset step. No observer RNG draws,
   extra inference, normalization update or fallback action. Logger failure aborts.

CPU OFF/ON fixtures are exact for the supported seam; they are not GPU parity.
All actual GPU sensing/compile/device/VRAM/latency checks remain GPU_PENDING.

## Errors found and fixed without weakening assertions

| Error / counterexample | Cause | Minimal correction / regression |
| --- | --- | --- |
| Evaluator JSON failed on nested JoinKey | Frozen dataclass was not JSON serializable | Explicit scalar key serialization; parsed capture/crash/contact truth-table records. Initial30-test run had19 errors, then passed after correction. |
| Mutable dict/GT-bearing clock payload could change a frozen packet | Metadata constructor accepted generic object fields | Strict scalar/None times, scalar string tags, immutable field pairs/calibration; direct source-handle and mutation regressions. |
| Changing clock/frame interpretation left digest unchanged | Fingerprint omitted deterministic time metadata | Include all deterministic clock/provenance/frame/depth semantics, excluding only host monotonic time. Positive/negative digest tests. |
| Huge numeric clock could raise OverflowError | Unguarded isfinite conversion | Explicit finite refusal; huge integer regression. |
| Final reset-generated sensor was outside parity | Pre-reset action row ended comparison too early | Complete row only after original step returns, including next/reset packet and state/RNG. Reset-only LiDAR negative control now refuses. |
| Original reference-load error could lose failure receipt | recorder not yet constructed after logger creation | Initialize recorder=None; CPU stand-in missing-reference test preserves FileNotFoundError/failure.json and makes no CUDA call. |
| Native InitialViewSpec is an object, not a string | Initial draft guard used string comparison | Check actual .view/.distance_m contract; valid native-spec and wrong-latency guard tests. |

Test-authoring syntax, incomplete stand-in preflight dictionaries and a shorthand
source-method name were also corrected to the actual contracts. No assertion was
removed, skipped or tolerance widened; no production bug was concealed by fixtures.

## Measurements, limits and remaining qualification

- Tier1 focused54 PASS; Tier2 included once in Tier3 BOIA306 PASS,
  0 failures/errors,15.090s. Independent read-only review54 PASS and exact source
  hashes matched; two reviewers found no remaining blocker in declared CPU scope.
- Final PUBLIC_CPU:3045 tests,0failures/4errors,10skips/5profile exclusions,
  exit1, aggregate FAIL. Exact error IDs/chains/terminal frames match the four
  historical DetectReduction errors; no new failures/errors. Details are in
  validation.json and CPU_TEST_RESULTS.txt. This is not an all-green public run.
- Packet time is explicitly DECLARED_SIM_SCHEDULE, relative to first explicit
  reset. Missing or mismatched times refuse evidence. No hardware timestamp or
  calibrated error interval is synthesized from host time or a zero-noise setting.
- LiDAR raw copy is post-Warp/pre-getter: clipping/censoring already occurred.
  FAR_OR_NO_RETURN cannot distinguish real far surfaces/no-hit/saturation; invalid
  status is retained where available. No physical free-volume inference. B03 open.
- LiDAR target proxy R=.20m vs RGB-D R=.15m. A6m sphere centre and foreground
  can both yield5.8m in one LiDAR beam. B04 remains unresolved; occupied hits remain.
- New collision-free metric is a **qualified proxy**: original capture, zero
  accumulated substep contact>0.05N, no action-boundary height/OOB. No claim about
  subthreshold contact or transient between-boundary height/OOB. Old unrecorded
  causes remain NOT_RECORDED and historical outcome priority is unchanged.
- No real loss/continuation calibration: WAIT/INTERVENE/benefit effectiveness,
  active visibility contribution, actual capture improvement and full snapshot
  restore are NOT_TESTED. No C02/physical certificate or safe-abort executor added.
- Observer copy/schema/diagnostic/hash/log host costs are separated from native
  full-step times and diagnostic-only compute. Cold first /warmP50/P95/P99/max are
  prepared; real GPU numbers are NOT_MEASURED. No5ms/10Hz claim.

The local commit binds source/tests/config/preregistration and this evidence.
Final exact-HEAD/clean-tree preflight runs after that commit; its actual outcome is
reported in the completion message. It is not fabricated inside a self-referential
receipt. See [validation.json](validation.json) for hashes and scoped CPU gates.

Next and only execution step after separate user approval: the bounded commands in
[GPU_SHADOW_RUNBOOK.md](GPU_SHADOW_RUNBOOK.md). No new implementation/design is
required before attempting S0/S1 qualification. **G5 HOLD**, even if Q01 later passes.
