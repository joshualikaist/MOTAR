# S3a and full-horizon readiness audit — 2026-11-06 KST

Author: Codex subagent `/root/s3a_readiness`, GPT-6 family (exact runtime variant NOT_RECORDED).
Worktree `MOTAR-camonly`, branch `camonly/dev`, base `f14bd059aab4c107cd547a221c1b10726ea57ca2`.
CPU only; no SITL, GPU, model/checkpoint modification, historical rewrite or gate execution.

## Findings

1. **Formal readiness remains BLOCKED.** The fixed20-scene corpus needs601 states (0..600) for600 completed actions with a scored60.0s endpoint. The current corpus is missing412 states:70/env5 lacks393 states208..600; the other19 scenes each lack state600. Old zero-action70/env5 ended at a real capture; neither a later episode nor repeated/extrapolated final target state repairs it. The trace was not merely truncated on disk.
2. **S3a has two separable numerical problems.** Its historical verdict remains NOT_EQUIVALENT under observation1e-3/action1e-4 tolerances. Replaying the same dumped observations isolates a policy numerical residual independently of sensing.

| CPU control on existing stored observations | 70 bars | 205 bars |
| --- | ---: | ---: |
| CPU batch16 vs CPU singleton action max | 5.364418029785156e-7 | 8.642673492431641e-7 |
| CPU singleton vs recorded GPU action max | 0.0015853345394134521 | 0.001012176275253296 |
| CPU batch16 vs recorded GPU action max | 0.0015852153301239014 | 0.0010121464729309082 |

CPU batching does not explain the residual. The restored action schema, deterministic mean/tanh selection and lateral mu scale are applied by the existing bridge. `navrl_reflection_offline_audit.build_player` explicitly enables CUDA matmul/cudnnTF32 and float32 precision `high` at construction. A GPU TF32-off control is needed to establish its contribution; CPU-only evidence does not establish that cause.

3. **The target-velocity mismatch starts with a raster measurement boundary.** The original GPU target mask/count/measurement was not recorded. Inversion of the dumped posterior covariance, using preceding CPU cross-covariance, yields these approximate GPU pixel counts:

| First mismatch | CPU sphere pixel count | Inferred GPU count | CPU mean surface depth m | Inferred GPU depth m |
| --- | ---: | ---: | ---: | ---: |
|70/env6 call7, target index885 error0.006935715675354004|14|12.999933292288931|8.837148666381836|8.829916698736008|
|205/env4 call4, target index887 error0.012056786566972733|8|8.99997548538|9.593835830688477|9.605255872771163|

These are explicitly **inferences**, not recovered GPU telemetry. A one-pixel target mask difference changes the measurement and measurement variance, then KF velocity and history. This local onset argues against tuning the KF to hide the discrepancy.

An external-only arithmetic intervention computes the analytic sphere discriminant in float64. At70/env6 call7 it changes14→13 pixels and the CPU measured world position to[33.6812477112,25.9493026733,1.0091849566], agreeing within6e-6m with the posterior-inferred GPU measurement[33.6812535166,25.9492980236,1.0091848065]. At205/env4 call4 it remains8 pixels versus inferredGPU9. Thus higher-precision discriminant arithmetic identifies part of the boundary sensitivity but is **not a complete qualified correction**. The production detector/bridge/KF is unchanged.

## Concrete preparation delivered

New `tools/sitl_formal_readiness.py` is a stdlib-only, read-only structural audit. It verifies inventory-bound raw bytes, fixed cohort/seed/F identity, finite contiguous episode0 target states0..600 and actual numeric/nonempty first8-env S3a report coverage under the original tolerances. It reports missing indices and rejects padding, episode substitution, duplicate/nonfinite frames and weakened report thresholds. It never launches or authorizes anything.

Even an all-positive synthetic corpus/report yields only `STRUCTURAL_CHECKS_PASS_REVIEW_REQUIRED`, `formal_gate_authorized=false`, `formal_gate_verdict=NOT_TESTED`. Source-bound positive and1°yaw negative controls plus independent review remain separate requirements. S3a's first8-env replay scope and the target corpus's fixed20-scene scope are distinct. The same-observation CPU/GPU action residual is checked as an explicit readiness diagnostic; no historical verdict is rewritten.

New `tests/test_sitl_formal_readiness.py`:10 tests PASS, including an end-to-end20-scene/601-state positive control and paired600/208/malformed/empty/control failures. Actual historical inventory/report audit returns BLOCKED (exit2 intentional). `git diff --check` PASS.

## Smallest new GPU controls to resolve S3a

A small recorded-state GPU replay is sufficient before another fullG8 rollout; no physics/PX4 run is needed for these numerical questions.

- **Policy control, one factor: TF32 precision.** Hold the checkpoint, dumped observation tensors, normalization and batch shape fixed. Run the existing saved observations on GPU with TF32 ON and OFF and compare both with the CPU result. Set and assert both matmul/cudnn TF32 OFF and precision `highest` **after player construction**, since construction resets those switches. Keep the action1e-4 threshold. Persist actual backend flags, source/checkpoint hashes and per-row actions. If OFF still exceeds tolerance, inspect layer-level outputs before changing any threshold or weight.
- **Sensor control, one factor: arithmetic precision.** Replay the existing states for calls0..9 in both cells (the original128-env batch shape can be retained) on CPU/GPU. Record RGB-D, segmented target pixel coordinates/counts, centroid/depth sums, camera measurement/variance and KF posterior/covariance; retain this evaluation-only provenance outside actor inputs. Compare original analytic sphere arithmetic with an explicitly opted-in higher-precision intersection path; include rotated rays/origins in the record because discriminant precision alone did not resolve the second case. Retain the positive replay and1°yaw negative control. Do not tune pixel/KF thresholds to fit the old output.

Each control changes one numerical factor and writes a new evidence directory. Frozen F weights and the original G8/S3a records stay unchanged. A new source-bound S3a equivalence result and independent review, not this diagnostic, are required to advance.

## Full target corpus boundary

No qualified tool currently recovers the missing original states on CPU. E2's generator/reset state and global random stream were not captured at the missing boundary. Global RNG coupling across environment resets is already recorded by G8 as TARGET_DEPENDS_ON_PURSUER; suppressing capture/reset or extending the simulation would create a new continuation lineage, not restore the original episode.

A newly preregistered collector must explicitly define the fixed20 initial scenes, target generator/random-stream/reset lifecycle and a601-state endpoint; store all target state needed to replay0..600, and qualify a fresh matched Isaac reference on those same externally recorded trajectories. Keep early-terminal historical episodes. A CPU-generated new deterministic E2 input model is possible as a separate model/qualification task, but is not an established substitute for the missing canonical trace. New GPU data collection and formal advancement require their scoped approval; this work provides neither.

## Reproduction and evidence

Persistent directory: `/home/fair/motar_tmp/agq_diagnosis_20261106/s3a_audit_r1/`.
`summary_v2.json` SHA256 `7bd1633a5d7383b8d2f3efeedd1b4d1ab32b44db4b0f9dacf0496f609af79965`.
`manifest_v2.json` SHA256 `83524e506d78f5d418a1736500e0e37e0b359916b2e2c5664e7b6523fc6db1ec` binds actual G8 dump/state/layout/receipt/frame inputs, production source, external probes and final source/test bytes.

All probe commands used CUDA hidden, PYTHONNOUSERSITE, OMP/MKL1, torch1, affinity6/7/12/13, nice10,8GiB address-space cap and timeout/kill bounds. Host faults0→0. Initial replay probe failed with MemoryError under the cap; its script/log remain. V2 cachesNPZ arrays and caps inference batches at16 and completed under the same cap. Originalfailed/control records are preserved. All owned probe processes ended.
