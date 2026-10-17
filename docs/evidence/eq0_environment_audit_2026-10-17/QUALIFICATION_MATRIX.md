# EQ-0 qualification matrix

Actual measurement source `5ff285ad053f181c5a1c86dddb2462bb44e6327b`; audit base
`7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2`. Every PASS below applies only to its written
criterion. No row is a general physical/hardware qualification. The acceptance criteria are audit
criteria for a named property, not retrospective amendments to OCC-P2.

PASS = supplied evidence satisfies the bounded criterion; FAIL = independent counterexample to
that criterion; LIMITED_VALIDITY = useful evidence under explicitly restricted assumptions;
NOT_TESTED = no adequate qualifying measurement/test. No unknown is promoted to PASS.

| ID | Property and explicit criterion | Status | Evidence / limitation |
| --- | --- | --- | --- |
| EQ-G1 | Geometry / physics / UAV dynamics overall | LIMITED_VALIDITY | Synthetic 3D pursuer, restricted authority and virtual target; see [G1](supporting/g1/findings.md) |
| G1.1 | Recorded source, robot identity, dt and configured loop count agree across all 112 cells | PASS | Condition/trace/receipt inventory and source blob equality; not measured inner-step convergence |
| G1.2 | Target bounded executor finite kinematics meet its registered limits | PASS | Existing motion gates; independent endpoint velocity/acceleration/displacement checks, float32 tolerance; no physical target claim |
| G1.3 | Pursuer PhysX/Lee/allocator path exists and is selected | PASS | Recorded engine/robot/config plus source call chain; physical calibration is separate |
| G1.4 | Current actor provides independent Z command authority | FAIL | At fixed state the altitude PI overwrites actor Z; raw altitude still varies dynamically, EQ0-05 |
| G1.5 | Active named RK4 routine implements classical RK4 for its first-order tracking ODE | FAIL | Independent sign/gain counterexample; fixed-OCC composite response close to nominal lag, EQ0-02 |
| G1.6 | Realistic 6-DoF target flight/contact/vertical motion | NOT_TESTED | Actual target is virtual bounded planar executor |
| G1.7 | All admitted scenes have globally connected, reserve-passable, dynamically feasible routes | NOT_TESTED | Local spawn/LOS and centre gap do not prove connectivity; predetermined 112-layout diagnostics only |
| G1.8 | Mass/inertia/motor/drag/contact calibrated to a real vehicle | NOT_TESTED | Nominal URDF and logs are not physical identification |
| G1.9 | Timestep/contact/capture convergence independently bounded | NOT_TESTED | 10 Hz consumed state records cannot reconstruct 100 Hz forces/contact chronology |
| EQ-G2 | Rendering / camera / LiDAR / occlusion overall | LIMITED_VALIDITY | Ideal synthetic sensor model, one calibrated-fusion FAIL; see [G2](supporting/g2/audit.md) |
| G2.1 | Sensor-path provenance matches recorded OCC source/settings | PASS | All 112 receipt/meta pairs and checked source blobs agree |
| G2.2 | Centre LOS agrees with stored online/mesh diagnostics within disclosed boundary cases | LIMITED_VALIDITY | Four online/three probe disagreements, five unique frames; zero primary count change, EQ0-13 |
| G2.3 | Camera depth→LiDAR fusion is pinhole-consistent and transformed to a common origin | FAIL | 3.4109° off-axis fixture and −0.10 m origin fixture, EQ0-01 |
| G2.4 | Pixel count proves complete physical target silhouette obstruction | NOT_TESTED | Rim proxy and low-resolution zero pixels are insufficient; target-alone support absent |
| G2.5 | Camera model qualifies a rigid body-mounted or actuated hardware camera | NOT_TESTED | Yaw-only ideal stabilization and synthetic metric depth; no hardware/gimbal error/delay data |
| G2.6 | LiDAR angular grid/range/radius conventions documented | PASS | Source-derived 72×4/5°/0.2–12 m/.20 m target convention; documentation is not metric calibration |
| G2.7 | LiDAR per-return metric accuracy and association identity calibrated | NOT_TESTED | No raw-return truth/provenance audit on all associations; .15/.20 m convention mismatch |
| G2.8 | Configured dropout/noise magnitudes were actually active | FAIL | For that assertion: perturb switch false/absent and code gated; clean run preserved, EQ0-03 |
| G2.9 | Actual sensor capture timestamps and stage latencies match delivery | NOT_TESTED | Capture time explicitly NOT_RECORDED; source supports zero configured delivery latency only |
| EQ-G3 | Perception / tracker / GT leakage / control overall | LIMITED_VALIDITY | See [G3](supporting/g3/REPORT.md) and [independent reader](supporting/g3/reproduce_independent_audit.py) |
| G3.1 | Recorded camera flag matches clean model's ≥2-pixel availability | PASS | Zero disagreements over 2,320,985 quota frames; not realistic detection accuracy |
| G3.2 | Fused validity equals camera OR accepted LiDAR and valid measurements activate track | PASS | Independent literal wire-format scan; identity correctness excluded |
| G3.3 | Real appearance variation, false detections, persistent identity and realistic redetection | NOT_TESTED | Clean fixed color, no distractors; no real-sensor qualification |
| G3.4 | Source expiry contract and observed all-cause miss/deactivation sequences are consistent | PASS | Strict age>5 s since fused correction; 41 Moving transitions after 51 misses; raw age not logged |
| G3.5 | Five-second geometric occlusion / expiry recovery benchmark realized | NOT_TESTED | Primary 5 s risk sets zero; all-cause FOV/sensor loss can still expire tracks |
| G3.6 | Source actor/critic interface and checkpoint identity preserved | PASS | 898/906 widths, 112 logs, source dataflow and F hash; no checkpoint load |
| G3.7 | No direct privileged GT/semantic input edge discovered by source review | PASS | Structural path only; evaluator/critic isolated in inspected path; not dynamic proof |
| G3.8 | Full dynamic GT intervention and normalization-state invariance | NOT_TESTED | No fixed-sensor/altered-GT actor/action experiment or RMS digest, EQ0-10 |
| G3.9 | Trace enabled/disabled noninterference in all 112 P2 cells | NOT_TESTED | Per-cell metadata remains NOT_TESTED; bounded inherited G0 test is separate |
| G3.10 | Eight G0 on/off paired conditions preserve sampled actor/trajectory/episodes | PASS | Frozen G0 report 8/8; 11,520 sampled actor rows plus trajectory/ledger/recorder checks |
| G3.11 | Current riskcap law enforces nonnegative stopping margin / collision-free recovery | FAIL | Independent 2 m/s floor counterexample; frozen implementation fidelity itself is consistent |
| G3.12 | Safe-abort state and endpoint measured | NOT_TESTED | All 112 recovery summaries say safe_abort NOT_RECORDED |
| EQ-G4 | Experimental validity / reproducibility / safety overall | LIMITED_VALIDITY | Registered clean frozen-loop evidence, not complete environment qualification |
| G4.1 | OCC-P2 source/preregistration committed before measurement; tracked freeze preserved | PASS | Launch/source/run reports, Git ancestry and immutable hashes |
| G4.2 | Exactly 128 env×4 quota episodes per cell; tails excluded from primary analysis | PASS | 112 cells/57,344 quota; 29,042 retained tails; independent ledger scan |
| G4.3 | Six seed-level confirmatory contrasts and multiplicity match frozen plan | PASS | Original independent post-run recomputation; 14 seed blocks, Holm family unchanged |
| G4.4 | OCC-P1/P2 independence provenance and scene duplicate screen disclosed | PASS | Distinct seed sets, P1 overlap zero; P2 no cross-seed hashes; B seed reuse disclosed; not proof of independence |
| G4.5 | Duration equivalence / unsupported 5 s tail established | NOT_TESTED | No equivalence margin/test; censoring/FOV competitors; no impossibility claim |
| G4.6 | Full silhouette, pure-speed causality, long Recovery or collision-free capture established | NOT_TESTED | Registered endpoint and task rules do not establish these constructs |
| G4.7 | Legacy diagnostics and quota analysis always share one population | FAIL | For that assertion: 96 cells differ; analysis correctly selects quota, EQ0-12 |
| G4.8 | Full historical execution stack, simulator/RNG state and bitwise replay qualified | NOT_TESTED | Partial log/TF32/config evidence only; no complete external-byte/state snapshot |
| G4.9 | Original evidence, main WIP/stash, Policy F preserved through EQ-0 | PASS | Final preservation check in audit evidence; no source/raw writes |
| G4.10 | External backup independently verified | NOT_TESTED | NOT_COMPLETED; not a blocker to authorized read-only work |
| G4.11 | H-R1 / H-R2 / H-R3 environment prerequisites satisfied | NOT_TESTED | User supplied provisional hypotheses; final endpoints/modalities/experiment not preregistered; conditional mapping only |

No EQ-1 simulator or hardware test has run in this unit. The
[EQ-1 validation plan](EQ1_VALIDATION_PLAN.md) lists future independent oracles and approval gates.
Issue severity and remedies are in [ISSUE_REGISTER.md](ISSUE_REGISTER.md).
