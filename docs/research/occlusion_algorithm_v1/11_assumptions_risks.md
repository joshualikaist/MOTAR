# 11 — Unverified assumptions, risks and stop conditions

| ID | Risk or unverified assumption | Evidence | Effect if true | Stop / mitigation |
| --- | --- | --- | --- | --- |
| R1 | **Novelty overlap.** Arbitration over a learned policy may already exist (shielding, simplex, CBF, residual policies) | Outside the 25; not reviewed (06 §3) | The method claim narrows to an evaluation contribution | G1b before any claim. If equivalent → `NOT_NOVEL_ENOUGH`, redesign |
| R2 | **Oracle detector.** Visibility is a colour-coded GT mask (M1 §2.2) | Code spot-checked | Gains may not transfer to a real detector, and p_det calibration is trivial | Detector-realism stress factor (09 §2). Never tune on the oracle only |
| R3 | **Response model error.** τ_v ≈ 0.4 s is derived; yaw lag is unknown; rollout error is NOT_RECORDED | M2 A4/A7 | Infeasible or unsafe candidates scored as safe | V4 gate. Model valid pre-contact only. Return to G3 on failure |
| R4 | **Governor is not a brake.** The 2 m/s floor gives negative stopping margins | M2 §3, EQ-1 | A safety screen built on the governor alone is meaningless | BOIA's own stopping feasibility test (07 §7.2) |
| R5 | **Hard-screen false alarms in narrow gaps.** Gaps of 0.47–0.59 m against a 0.40 m body | M2 §10 | Constant overrides; capture collapses | Two-tier screen; false-alarm gate ≤ 2 % in V5 |
| R6 | **Hand-back out of distribution.** BOIA actions enter Policy F's `prev_action` history | M2 risk 5 | Policy F misbehaves after hand-back | Log action change after hand-back; minimum dwell; capture guard |
| R7 | **Belief miscalibration.** No camera gate, world-axis R, LiDAR resets age, heuristic Q | M1 §5, §9 | Wrong P_loss and U | V3 NEES/NIS; camera-only miss streak; Q inflation |
| R8 | **Sensed map errors.** Target returns, 5° beam gaps, depth saturation at 10 m, S-L1B retention | M1 §1, risk 6 | False occlusion or false clearance | Belief-based target exclusion; unknown ≠ free; V2 offline check |
| R9 | **Cross-env coupling.** Shared RNG and global vertex rewrite | M2 §9 | Paired comparisons invalid | Qualified isolation runner per profile, else unpaired analysis |
| R10 | **Terminal semantics.** Late contact flag, capture masks crash, post-reset terminal obs | M2 §7 | Mis-scored episodes | Terminal-first module; evaluation from simulator flags only |
| R11 | **Correlation is not causation.** Crashes cluster during occlusion, plausibly by common cause | M3 §2 | Reducing occlusion-conditioned crash may not reduce total crash | Episode crash is a co-primary guard |
| R12 | **Compute.** About 2.4 × 10⁸ distance tests per decision for N = 128 | 07 §13 | Deadline missed; lower throughput | Culling, then smaller M/K, then the BPOD analytic path; measure in V5/V6 |
| R13 | **Different-version papers.** Equation numbers for EGO-Planner, ORTrack, Star-Convex and Lee may differ from the user's copies | Manifest | Citation errors | Re-read the user's copies when provided |
| R14 | **Missing papers.** YOPO and OCTrack were not read | Manifest | Any statement about them is unsupported | None is made; YOPO only via code |
| R15 | **Schedule.** Conflict with the KROC2027 freeze and paper (P0) | Beads `motar-d43` | KROC slips | This line stays secondary until after 2026-10-28 unless the user decides otherwise |
