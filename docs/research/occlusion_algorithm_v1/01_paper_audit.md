# 01 — 25-paper verification and reading record

**Scope.** This is the G1 reading record for the 25-paper set the user named on 2026-10-19.

**Sources.**
- **Identity:** [`paper_manifest.json`](paper_manifest.json), which compares SHA-256 against the
  user's inventory.
- **Detail:** the per-paper notes, written by five read-only reading passes and linked below. Each
  note records the PDF pages actually read and gives equations as "p.N Eq.(k)".

**Labels used:**
- **FULL_READ:** every page was read.
- **METHOD_REVIEWED:** the method and experiments were read in full, but not every appendix page.
- **ABSTRACT_ONLY:** only the abstract was read.
- **NOT_AVAILABLE:** no PDF was available, so the paper is not summarised.
- **CODE_VERIFIED:** never used here, because no official code was run.
- **REPRODUCED:** never used here, because no paper result was reproduced.

## Status table

| # | Paper (short) | File identity vs user copy | Pages | Read status | Notes |
| --- | --- | --- | ---: | --- | --- |
| 1 | BPOD — Probabilistic Visibility-Aware Trajectory Planning (arXiv 2306.06363v2) | IDENTICAL | 8 | FULL_READ (pp.1–8) | [P1 §1](notes/P1_visibility_planning.md) |
| 2 | Eva-Tracker (arXiv 2602.12549v1) | IDENTICAL | 8 | FULL_READ | P1 §2 |
| 3 | Elastic Tracker (arXiv 2109.07111v2) | IDENTICAL | 7 | FULL_READ | P1 §3 |
| 4 | Visibility-aware Trajectory Optimization (arXiv 2103.06742v1) | IDENTICAL | 8 | FULL_READ | P1 §4 |
| 5 | Star-Convex Constrained Optimization for Visibility Planning (arXiv 2204.04393v1) | **DIFFERENT_VERSION** | 7 | FULL_READ of v1 | P1 §5 |
| 6 | OA-VAT — Instance-level Visual Active Tracking with Occlusion-Aware Planning | IDENTICAL | 15 | FULL_READ | [P2 §1](notes/P2_active_tracking.md) |
| 7 | UAST — Unified Active Search and Tracking (CVPR 2026, CVF open access) | IDENTICAL | 10 | FULL_READ (no supplement available) | P2 §2 |
| 8 | TrackVLA (arXiv 2505.23189v1) | IDENTICAL | 25 | FULL_READ (text level) | P2 §3 |
| 9 | TrackVLA++ (arXiv 2510.07134v1) | IDENTICAL | 9 | FULL_READ | P2 §4 |
| 10 | Open-World Drone Active Tracking with Goal-Centered Rewards (DAT) | IDENTICAL | 36 | METHOD_REVIEWED (main pp.1–10, appendix prose pp.16–29) | P2 §5 |
| 11 | YOPOv2-Tracker (arXiv 2505.06923v1) | IDENTICAL | 19 | FULL_READ | [P3 §1](notes/P3_learned_planners.md) |
| 12 | NavRL++ | IDENTICAL | 18 | FULL_READ | P3 §2 |
| 13 | PANTHER (arXiv 2103.06372v3) | IDENTICAL | 16 | METHOD_REVIEWED | P3 §3 |
| 14 | Deep-PANTHER (arXiv 2209.01268v2) | IDENTICAL | 9 | METHOD_REVIEWED | P3 §4 |
| 15 | RAPTOR (arXiv 2007.03465v1) | IDENTICAL | 16 | METHOD_REVIEWED | P3 §5 |
| 16 | FASTER (arXiv 1903.03558v3) | IDENTICAL | 7 | FULL_READ | [P4 §1](notes/P4_safety_dynamics.md) |
| 17 | Lee, Leok, McClamroch — Geometric Tracking Control on SE(3) (arXiv 1003.2005v1) | **DIFFERENT_VERSION** (8-page arXiv v1; user holds the 6-page CDC version) | 8 | FULL_READ of v1 | P4 §2 |
| 18 | Aerial Gym Simulator (RA-L 2025) | IDENTICAL | 8 | FULL_READ | P4 §3 |
| 19 | EGO-Planner (arXiv 2008.08835v2) | **DIFFERENT_VERSION** | 8 | FULL_READ of v2 | P4 §4 |
| 20 | Adaptive Tracking and Perching for Quadrotor in Dynamic Scenarios (arXiv 2312.11866v2) | IDENTICAL | 20 | METHOD_REVIEWED (pp.1–18) | P4 §5 |
| 21 | ORTrack — Occlusion-Robust ViT for Real-Time UAV Tracking (arXiv 2504.09228v1) | **DIFFERENT_VERSION** | 11 | FULL_READ of v1 | [P5 §1](notes/P5_perception.md) |
| 22 | Perception-Aware Time-Optimal Planning for Quadrotor Waypoint Flight (arXiv 2603.04305v1) | IDENTICAL | 21 | FULL_READ | P5 §2 |
| 23 | Perception-aware Planning in Unknown and Feature-limited Environments (arXiv 2503.15273v2) | IDENTICAL | 8 | FULL_READ | P5 §3 |
| 24 | YOPO — You Only Plan Once | **NOT_AVAILABLE** (IEEE RA-L only, no arXiv) | — | Not summarised; YOPOv2-Tracker's own statements about YOPO are recorded in P3 §1, and the official code is audited in [K2 §2](notes/K2_code_audit.md) | — |
| 25 | Rethinking Occlusion Modeling for UAV Tracking (OCTrack) | **NOT_AVAILABLE** (not found on arXiv or CVF) | — | Not summarised | — |

## Totals

| | Count | Of which |
| --- | ---: | --- |
| Readable | 23 of 25 (302 PDF pages) | 19 identical to the user's copies; 4 different versions |
| FULL_READ | 18 | — |
| METHOD_REVIEWED | 5 | DAT, PANTHER, Deep-PANTHER, RAPTOR, Adaptive Perching |
| CODE_VERIFIED | 0 | — |
| REPRODUCED | 0 | — |

**What the user can provide.** The user can place the original YOPO and OCTrack PDFs, and the
original versions of the four DIFFERENT_VERSION papers, on this machine. The four would then be
re-read for their page and equation references. The equation numbers cited from those four papers
refer to the versions listed above.

## One-line takeaway per paper

1. **BPOD.** It forecasts a belief-conditioned detection probability:
   - γ = γ_FOV · Π_i γ_LOS,i (p.4 Eq.15). This assumes mutually independent events and a perfect
     detector (footnote, p.2).
   - It evaluates each factor in closed form with erf on a linearised signed distance (p.5 Eq.19).
   - It turns the probability into an expected posterior covariance, P̃ = P⁻ − γ K̃ C̃ P⁻
     (p.3 Eq.11d).
   - It works in 2D, with a known map of convex obstacles.
2. **Eva-Tracker.** It puts candidate viewpoints on an arc around the predicted target and keeps the
   nearest one with a clear line of sight (p.3). It looks up visibility in a camera-frame "FoV-ESDF"
   (p.3–4). After a loss it follows a deterministic Bézier extrapolation of about 1 s; the authors
   say this suits short losses only (p.8).
3. **Elastic Tracker.** It adds an asymmetric distance penalty and a visible sector, using a
   horizontal/vertical split. There is no uncertainty model and no re-acquisition. In simulation it
   is fed the GT target position.
4. **Visibility-aware Trajectory Optimization.** It places balls along the line of sight and requires
   min_i(Ξ(c_i) − r_i) > 0 (p.3 Eqs.3–5), which batches easily against circular bars. It reports no
   runtime of its own.
5. **Star-Convex.** It tests visibility directly on a point cloud by spherical flipping plus a convex
   hull. It assumes an omnidirectional sensor and offline planning.
6. **OA-VAT.**
   - Identity comes from instance prototypes (DINOv3 + EMA).
   - A Kalman filter in the image plane maps detector confidence to measurement noise through a
     sigmoid (p.4 Eq.9).
   - When confidence drops, a diffusion planner takes over. The trigger rule is not fully specified,
     and the paper describes it differently on p.4 and p.5.
7. **UAST — the closest prior art.** Forward RGB-D UAV, dense pillars and first-acquisition search.
   - **Tracking:** a constant-velocity KF with an innovation gate. As printed, the innovation is
     normalised by R only.
   - **Modes:** visible / temporarily invisible / lost.
   - **Control:** a V×H grid of quintic end-state candidates scored by a learned cost. In training,
     that cost includes SDF line-of-sight and FOV terms (pp.5–6).
   - **Missing:** thresholds and the horizon are not given.
8. **TrackVLA.** A 7B VLA with an anchor-diffusion head that needs about an RTX 4090. It is not
   feasible on an 8 GB GPU; it is useful only for metric design.
9. **TrackVLA++.** Adds an `<invalid>` (occluded) token and an identity memory gated by entropy. It
   classifies current visibility; it does not forecast it.
10. **DAT.** An RL tracking benchmark with privileged GT used only in the reward. Its metric
    templates are useful for MOTAR's secondary endpoints.
11. **YOPOv2-Tracker.**
    - A single network maps RGB-D to thrust and attitude through 5×3 primitive anchors.
    - Training back-propagates analytic trajectory costs computed from a privileged ESDF.
    - It names target occlusion as its main failure mode (p.13).
12. **NavRL++.**
    - A transformer PPO velocity policy fine-tuned with sensor noise, dropouts, latency and gain
      perturbations.
    - An optional safety filter projects commands and stops the robot when nothing is safe.
    - It is close prior art for MOTAR's base-policy lineage, not for the arbitration layer.
13. **PANTHER.** Joint translation and yaw optimisation with a field-of-view term for the dynamic
    obstacle most likely to collide. There is no line-of-sight model.
14. **Deep-PANTHER.** Imitation learning produces 6 candidate trajectories ranked by cost. If none is
    feasible it keeps the previous plan. Evaluation uses GT obstacle prediction.
15. **RAPTOR.** Parallel optimisation over topologically distinct paths. Line-of-sight visibility and
    the yaw information gain are about unknown map space, not a target.
16. **FASTER.**
    - Safety holds by construction because a committed trajectory always stays in known-free space
      and ends at rest. Implicitly it assumes a static world and perfect tracking.
    - Velocity is checked only at interval starts (p.3).
    - It is the direct precedent for the safe fallback.
17. **Lee SE(3).**
    - Convention: e3 points down, gravity is +mg e3 and thrust is −fRe3 (Eq.3).
    - Stability: exponential stability needs an initial attitude error below 90° (Eqs.20, 27).
    - MOTAR's velocity controller changes the thrust law (45° tilt cap, T = f_z/b3_z), so Lee's proof
      does not transfer.
18. **Aerial Gym.**
    - The paper never states the physics timestep.
    - Eq.4/5 print an acceleration feed-forward sign that needs a test.
    - The ray-casting mesh rebuild is described only for static scenes.
    - Its velocity + yaw-rate action interface (p.6) matches Policy F's.
19. **EGO-Planner.** The B-spline convex-hull property gives per-axis feasibility. Safety is a pipe
    post-check, and there is no backup trajectory.
20. **Adaptive Perching.** Occlusion is a deterministic chain of balls along the line of sight
    (Eqs.15–18), and terminal states can be relaxed (Eqs.46–50). There is no formal guarantee.
21. **ORTrack.**
    - Training masks part of the template with centre-weighted random patterns (Eq.1) and adds a
      feature-invariance loss (Eq.2).
    - Partial-occlusion robustness improves (Fig.3, p.7), but the confidence scores are not
      calibrated.
    - The masking formula may be inverted as written; check against the code before reuse.
22. **PA Time-Optimal.** Smooth tanh-based FOV terms (Eqs.30–33), range noise ∝ d² (Eq.44) and a
    Fisher-information uncertainty cost (Eqs.40, 48). Planning is offline only.
23. **PA Unknown/Feature-limited.** A viewpoint-graph reward (Eq.9) with line-of-sight raycasts. A run
    counts as lost when visible features stay low for t_f = 0.35 s. It replans in 23–62 ms on a CPU.

**Ground truth used online in the papers' own evaluations.**
- Elastic and Eva-Tracker feed the GT target position in simulation, with no occlusion gate in the
  code that was read.
- Visibility-aware Trajectory Optimization's real-world target broadcasts its own location.
- PANTHER's default simulation uses GT obstacle trajectories.

None of these performance numbers transfers to MOTAR.
