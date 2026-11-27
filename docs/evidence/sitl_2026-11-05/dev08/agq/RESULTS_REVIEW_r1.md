# Independent AGQ evidence review r1 — 2026-11-06 KST

Reviewer: Codex `/root/agq_evidence_review`, GPT-6 family; exact runtime model variant **NOT_RECORDED**. Fresh-context read-only review on `MOTAR-camonly`, branch `camonly/dev`, base/HEAD `f14bd059aab4c107cd547a221c1b10726ea57ca2`. The inherited clock remains start2026-11-05T13:45:22Z, wrap17:45:22Z, finish18:35:22Z; it was neither reset nor closed. Review writes are confined to this external review directory. No SITL/GPU/flight, source edit, checkpoint replacement, outcome rerun or gate advancement occurred.

## Verdict

**LIMITED_VALIDITY** for the existing-log descriptive diagnosis and S3a/input numerical audit. Integrity and bounded independent numerical recomputations **PASS**. **BLOCKER:0; MAJOR:0.** Four MINOR clarifications identified below have been incorporated in the root summary, with the original worker records preserved. The vertical-datum simplification remains an explicit reuse limit, with its lack of effect on these four tall-bar traces independently checked.

This verdict does not establish actual contacts, causal crash mechanisms, a gain-induced rescue, a qualified corrected model, S3a equivalence or formal/flight readiness. S3a remains **NOT_EQUIVALENT**, strict600-action inputs remain **BLOCKED**, and the proposed live/GPU follow-ups remain **NOT_RUN**.

## Audited bindings and provenance

The primary inputs were the four-case `agent_analysis_r1/REPORT.md`, `analyze_v3.py`, `analysis_v3/diagnosis.json`, `supplement_v3.py`, `presentation_v3/supplement.json`, associated series/checkpoints/plots, and the S3a report/summary/probes. The later `command_projection.py/json` and root compact README/JSON/proposed follow-up were also inspected.

The current diagnosis manifest is `agent_analysis_r1/validated_manifest_v2.json`, SHA256 `deea2dbb457df6cc14a6a3453d26f32e28f30724c67277236c44c507849fe068`. The initial `validated_manifest.json` predates the appended command-projection discussion; it is retained historical evidence, not the current report binding.

I independently rehashed all610 unique bindings/747541480bytes in `root/integrity_v1/validation.json` (SHA256 `bb35e9bc32503f96fdbf704615efa59f1c2c0d07d71d216e85ed55c5effb2193`). This includes the complete507-file dev08 raw set, all four actual ULogs/checkpoints/worlds, canonical F, scientific probe/source inputs and current preparation files. All current bindings match. The S3a `manifest_v2.json` deliberately retains exactly two earlier uncommitted utility-source hashes; the root record names both and separately binds current source/test bytes. No unexplained scientific/raw mismatch remains. This evidence audit does not substitute for the separate source reviews of the new utilities.

The first bounded reviewer probe stopped on the historical report-binding mismatch. Later reviewer-development attempts retained an incorrect module accessor, an initial-only ULog-parameter comparison, and an optional empty-mask depth NaN serialization failure. Those failed scripts/logs/receipts remain here. Final successful probes use the correct manifest, effective ULog parameter timeline and explicit null for undefined depth; no input was altered.

## Independent recomputations

`evidence_probe.json` reconstructs the common-publication-clock series directly from each raw ULog. Truth origin10..13s has zero x/y/z spread in all four cases. The NED/world transform, linear estimator interpolation, normalized quaternion interpolation, truth rates, position discrepancy/growth and descriptive onsets reproduce v3 within1e-8. Quaternion signs are continuous over these logs; all interpolated topics bracket the episode interval. All27 effective parameters per case agree with the ledger after applying ULog parameter changes through gate start. Comparing only ULog initial parameters would incorrectly compare generic startup values.

| Case | First footprint proxy | First yaw abs>5rad/s | First z<0.8m | First discrepancy>0.5m | Archived end |
| --- | ---: | ---: | ---: | ---: | --- |
| env2 native |0.940|1.280|1.260|6.180|7.400 crash |
| env2 candidate |2.736|2.736|3.336|7.076|8.320 crash |
| env5 candidate |none|none|4.080|none|13.000 capture |
| env5 native |5.728|5.768|5.988|6.668|7.000 crash |

All times are elapsed PX4 publication seconds, with sampled/interpolated onset precision. The source stamps incoming HIL truth at PX4 receipt. These observations do not recover unknown MAVLink publication phase, raw packet times, physics simultaneity or estimator sample-time alignment. The ±20ms sensitivity envelope is a diagnostic choice, not an empirically established clock-error bound.

I separately reconstructed all sampled intersections against the actual archived rounded SDF-world bar poses and included the absolute Gazebo settled-centre z offset. The full-trace overlap-index sets exactly match v3: env2native18, env2candidate25, env5candidate0, env5native8. Base-box overlap samples occur only in env2candidate (6); nativeenv5 remains rotor-only. This is a positive/negative geometric control with nonzero overlaps in three traces and none in candidateenv5. It remains an inscribed rotor-polygon projection plus vertical-span proxy, not exact3D collision detection or a contact event. Actual contact/impulse streams are **NOT_RECORDED**.

The source-pinned SDF contains the stated base box and four rotor cylinders. A0.14m logical disc cannot enclose the box corners or rotor reach. The established contract mismatch justifies a geometry-only discriminating proposal; it does not establish that the mismatch caused these outcomes.

Sampled allocator thrust-unachieved counts reproduce as env2native2/32, env2candidate0/28, env5native0/6 after first overlap. Nativeenv5's sampled yaw-torque deficit must not be relabelled collective-thrust saturation. Five-Hz status cannot certify the absence of unobserved intersample deficits. Candidateenv5's no-overlap/no-large-rate result,0.763m minimum height,0.243m maximum discrepancy and62.23° actual tilt are a useful negative case, not qualified controller matching or evidence selecting gains.

`projection_probe.json` independently reproduces all48 fixed-pose0.3s projections from raw commands, ULog pose/velocity and reviewer intersection code. The latest generated checkpoint command, sampled accepted `trajectory_setpoint`, downstream rate setpoint and actual truth velocity are different signals. Generated command timing is not send/receive timing. Interpolated5Hz trajectory requests cannot recover every accepted command or controller shaping. Native pre-overlap momentum/tracking differences and candidateenv2's command-envelope intersection remain descriptive. A projection with no overlap is not a feasible-trajectory or safe-command verdict. Earlier rate-error>5rad/s samples and candidateenv5's tilt overshoot keep control/plant explanations open.

`s3a_probe.json` independently reproduces the policy controls over287stored rows at70bars and372 at205bars. CPU batch16-minus-singleton maxima are5.364418e-7/8.642673e-7; singleton-minus-recordedGPU maxima are0.00158533454/0.00101217628. This isolates a residual on identical stored observations; it does not identify TF32 or rule out GPU batch/kernel effects. Player construction explicitly enables TF32/high precision mode, so a future OFF control must set/assert flags after construction.

Replaying the two onset sequences reproduces CPU counts14/8, CPU surface depths8.83714867/9.59383583m, and posterior-inferred GPU counts12.9999333/8.99997549. The inversion uses preceding CPU cross-covariance and recorded GPU posterior diagonals; original GPU masks/counts/measurements were not recorded. It supports a raster-boundary hypothesis, not recovered GPU telemetry. `precision_probe.json` independently reproduces the external float64-discriminant intervention:70/env6 becomes13pixels and its measurement is within5.91e-6m per coordinate of the inferred GPU measurement;205/env4 remains8pixels and differs by up to0.01370m. The intervention is incomplete and provides no basis to tune the detector/KF or weaken S3a tolerances.

The independent binary target scan finds412 missing states:393 at70/env5 (208..600), plus state600 in each of the other19scenes. The original `episodes.jsonl` identifies70/env5 episode0 as a capture at completion_step208, confirming an actual terminal rather than mere disk truncation. Repeated/padded/extrapolated final states or another episode cannot restore its missing lineage. No extension was fabricated. A new collector must define/reset/record its own generator/RNG semantics and qualify a new matched reference.

## MINOR findings and disposition

1. **m1 — vertical geometry datum.** `analyze_v3.py:55,74–76` uses height above the settled gate origin against bars in the world floor datum. The omitted settled-centre offset is0.0890027m(env2)/0.0900098m(env5). The actual-world independent probe reproduces every original overlap sample for these tall bars, so current onsets are unaffected. Root README now explicitly preserves this reuse limitation; low/short/nonvertical obstacles require an absolute-datum geometry calculation.
2. **m2 — approximate rate label.** Worker `REPORT.md:13` says approximately100Hz for truth attitude/angular velocity. All four ULogs have median8ms intervals, or125Hz. Root README adds the explicit125Hz erratum without rewriting the worker record. No event changes result.
3. **m3 — batching inference scope.** The initial compact README said batch size was rejected as the explanation. CPU1-versus16 does not eliminate GPU128/kernel effects. Current README narrows the claim to the measured CPU comparison and explicitly leaves GPU batch/kernel effects unisolated. **Corrected.**
4. **m4 — accepted versus shaped setpoint label.** The initial compact README called `trajectory_setpoint` a shaped request. PX4 `mavlink_receiver.cpp:1048–1051` publishes accepted offboard requests; `MulticopterPositionControl.cpp:549–555` separately publishes internal/PID-corrected `vehicle_local_position_setpoint`. Analysis loads the latter but does not use it. Current README labels the measured `trajectory_setpoint` separately; no shaping history should be inferred from it. **Corrected wording; full shaping/arrival evidence remains unmeasured.**

`summary_snapshot_probe.json` verifies the compact JSON's retained scientific fields against the original diagnosis and binds the inspected root README/proposed plan. The summary preserves the candidate tilt caveat, env5-only allocator statement, no gain selection, no command-safety proof and blocked prerequisites. The proposed geometry/S3a/input boundaries are within this evidence; recording implementation, fixed live analysis rules, source review and scoped execution approval are still prerequisites, not delivered measurements.

## Execution record and Git Traceability

All numerical/replay/hash probes used CUDA hidden, PYTHONNOUSERSITE, OMP/MKL/OpenBLAS1, torch1 where applicable, affinity6/7/12/13, nice10,8GiB address-space cap, timeout540s with10s kill grace. Every recorded invocation has kernel matched-fault count0→0. All reviewer jobs ended; no simulator launched. The successful probes and failed development attempts have individual logs and resource/time receipts. The Warp CPU kernel cache is confined to this review directory.

Branch:`camonly/dev`; Base/HEAD:`f14bd059aab4c107cd547a221c1b10726ea57ca2`; New Commit:NONE(read-only reviewer); Changed Files:only this external audit directory; Tests:independent raw/numeric/integrity/projection/summary probes above; Working Tree:root-owned preparation/docs edits present, not touched by reviewer; Remote Push:NO; Main Merge:NO; Raw/Checkpoint Integrity:current bound inputs match, no reviewer mutation.
