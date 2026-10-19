# 01 — Original-copy paper verification and reading record (G1-R)

**2026-10-19 G1-R complete.** The original25 and four extras are separately accounted for.
Physical PDF pages start at1. Sources are read-only local originals, identified by title/content,
metadata and SHA, not filenames alone. [Manifestv2](paper_manifest.json) and
[machine reading receipts](reading_receipts.json) bind the notes. [Manifestv1](paper_manifest.v1.json)
preserves the prior public-copy/availability state.

FULL_READ is cumulative:15 originals were fully read this pass;14 byte-identical previously
FULL_READ papers retain that status after current original-copy spot checks. This does not claim
29 entirely new full readings. FASTER has only two numbered equations; both plus algorithm/curve
identities were checked rather than inventing a third. No external code or result was reproduced.

## Status table — original25

| # | Paper | Identity | PDF pages | Status / current pass | Notes |
| --- | --- | --- | --- | --- | --- |
| 1 | Adaptive Tracking and Perching | IDENTICAL_TO_USER_COPY | 20 | FULL_READ / full original | [P4_safety_dynamics](notes/P4_safety_dynamics.md) |
| 2 | Aerial Gym Simulator | IDENTICAL_TO_USER_COPY | 8 | FULL_READ / identical-copy spot checks | [P4_safety_dynamics](notes/P4_safety_dynamics.md) |
| 3 | Deep-PANTHER | IDENTICAL_TO_USER_COPY | 9 | FULL_READ / full original | [P3_learned_planners](notes/P3_learned_planners.md) |
| 4 | EGO-Planner | USER_COPY_NOW_READ | 8 | FULL_READ / full original | [P4_safety_dynamics](notes/P4_safety_dynamics.md) |
| 5 | Elastic Tracker | IDENTICAL_TO_USER_COPY | 7 | FULL_READ / identical-copy spot checks | [P1_visibility_planning](notes/P1_visibility_planning.md) |
| 6 | Eva-Tracker | IDENTICAL_TO_USER_COPY | 8 | FULL_READ / identical-copy spot checks | [P1_visibility_planning](notes/P1_visibility_planning.md) |
| 7 | FASTER | IDENTICAL_TO_USER_COPY | 7 | FULL_READ / identical-copy spot checks | [P4_safety_dynamics](notes/P4_safety_dynamics.md) |
| 8 | Lee SE3 | USER_COPY_NOW_READ | 6 | FULL_READ / full original | [P4_safety_dynamics](notes/P4_safety_dynamics.md) |
| 9 | OA-VAT | IDENTICAL_TO_USER_COPY | 15 | FULL_READ / identical-copy spot checks | [P2_active_tracking](notes/P2_active_tracking.md) |
| 10 | ORTrack | USER_COPY_NOW_READ | 11 | FULL_READ / full original | [P5_perception](notes/P5_perception.md) |
| 11 | NavRL++ | IDENTICAL_TO_USER_COPY | 18 | FULL_READ / identical-copy spot checks | [P3_learned_planners](notes/P3_learned_planners.md) |
| 12 | Open-World DAT | IDENTICAL_TO_USER_COPY | 36 | FULL_READ / full original | [P2_active_tracking](notes/P2_active_tracking.md) |
| 13 | PANTHER | IDENTICAL_TO_USER_COPY | 16 | FULL_READ / full original | [P3_learned_planners](notes/P3_learned_planners.md) |
| 14 | Perception-Aware Time-Optimal | IDENTICAL_TO_USER_COPY | 21 | FULL_READ / identical-copy spot checks | [P5_perception](notes/P5_perception.md) |
| 15 | Perception-aware Unknown Feature-limited | IDENTICAL_TO_USER_COPY | 8 | FULL_READ / identical-copy spot checks | [P5_perception](notes/P5_perception.md) |
| 16 | BPOD | IDENTICAL_TO_USER_COPY | 8 | FULL_READ / identical-copy spot checks | [P1_visibility_planning](notes/P1_visibility_planning.md) |
| 17 | RAPTOR | IDENTICAL_TO_USER_COPY | 16 | FULL_READ / full original | [P3_learned_planners](notes/P3_learned_planners.md) |
| 18 | OCTrack | USER_COPY_NOW_READ | 11 | FULL_READ / full original | [P5_perception](notes/P5_perception.md) |
| 19 | Star-Convex | USER_COPY_NOW_READ | 7 | FULL_READ / full original | [P1_visibility_planning](notes/P1_visibility_planning.md) |
| 20 | TrackVLA | IDENTICAL_TO_USER_COPY | 25 | FULL_READ / identical-copy spot checks | [P2_active_tracking](notes/P2_active_tracking.md) |
| 21 | TrackVLA++ | IDENTICAL_TO_USER_COPY | 9 | FULL_READ / identical-copy spot checks | [P2_active_tracking](notes/P2_active_tracking.md) |
| 22 | UAST | IDENTICAL_TO_USER_COPY | 10 | FULL_READ / identical-copy spot checks | [P2_active_tracking](notes/P2_active_tracking.md) |
| 23 | Visibility-aware Traj Opt | IDENTICAL_TO_USER_COPY | 8 | FULL_READ / identical-copy spot checks | [P1_visibility_planning](notes/P1_visibility_planning.md) |
| 24 | YOPOv2-Tracker | IDENTICAL_TO_USER_COPY | 19 | FULL_READ / identical-copy spot checks | [P3_learned_planners](notes/P3_learned_planners.md) |
| 25 | YOPO | USER_COPY_NOW_READ | 8 | FULL_READ / full original | [P3_learned_planners](notes/P3_learned_planners.md) |

## Totals

| Population | Unique papers | PDF pages | FULL_READ | METHOD_REVIEWED | ABSTRACT_ONLY |
| --- | --- | --- | --- | --- | --- |
| Original25 | 25 | 319 | 25 | 0 | 0 |
| Extra4 | 4 | 33 | 4 | 0 | 0 |

CODE_VERIFIED=0; REPRODUCED=0. Original inventory:24 known SHA matches, YOPO inventory SHA null; no mismatch. Original-copy identity19same bytes/6nowread. Two EGO-Planner files are byte-identical; the filename without `(1)` is representative and the other is retained as duplicate. Thirty files represent29 unique papers.

## Extra papers (not in the original25)

| Paper | Pages | Read status | Decision | Notes |
| --- | --- | --- | --- | --- |
| EGO-Swarm | 7 | FULL_READ | REFERENCE_ONLY | [P6](notes/P6_extra_papers.md) |
| PPTracker | 8 | FULL_READ | ADD_TO_CORE | [P6](notes/P6_extra_papers.md) |
| PRIMER | 7 | FULL_READ | ADD_TO_CORE | [P6](notes/P6_extra_papers.md) |
| Similarity-Guided Layer-Adaptive ViT | 11 | FULL_READ | REFERENCE_ONLY | [P6](notes/P6_extra_papers.md) |

PRIMER narrows generic arbitration/fallback novelty; PPTracker exposes online GT initialization and identity constraints. EGO-Swarm is a navigation precedent, SGLATrack a possible later detector stress reference. No performance number is compared with MOTAR.

## One-line takeaways from G1, corrected by G1-R


1. **BPOD.** It forecasts a belief-conditioned detection probability:
   - γ = γ_FOV · Π_i γ_LOS,i (p.4 Eq.15). This assumes mutually independent events and a perfect
     detector (footnote, p.2).
   - It evaluates each factor in closed form with erf on a linearised signed distance (p.5 Eq.19).
   - It turns the probability into an expected posterior covariance, P̃ = P⁻ − γ K̃ C̃ P⁻
     (p.3 Eq.11d).
   - It works in 2D, with a known map of convex obstacles.
2. **Eva-Tracker.** It puts candidate viewpoints on an arc around the predicted target and keeps the
   nearest one with a clear line of sight (p.3). It looks up visibility in a camera-frame "FoV-ESDF"
   (p.3–4). After a loss it follows Bézier extrapolation; its configured horizon is about1s (pp.3/5), and p.8 explicitly limits it to short losses.
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
8. **TrackVLA.** A 7B VLA with an anchor-diffusion head that needs about an RTX 4090. Concurrent operation on the MOTAR 8 GB GPU is NOT_TESTED; it is useful for metric design.
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
    - Stability: exponential stability needs an initial attitude error below 90° (CDC Eq.27 and Prop.2).
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
    (original pp.6 Eqs.15–19), and terminal states can be relaxed (pp.8–9 Eqs.46–50). There is no formal guarantee.
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

24. **YOPO original.** Fifteen anchors with learned trajectories and a **−J score** (p.5 Eq.21).
    The audited YOPO-Simple branch is different; original numerical anchor extents/radius are not
    recorded. Keep BOIA's own heading/speed lattice, not an unsupported endpoint transplant.
25. **OCTrack.** COM/ACP synthesizes structured occlusion; CADB refers to transformer layer depth.
    It does not supply calibrated detection probability for BOIA; see P5 §4.

## Mandatory original-copy questions and corrections

- **BPOD:** independence p.4 Eq.15, expected covariance p.3 Eq.11d and erf p.5 Eq.19 confirmed;
  perfect sensor assumption p.2 footnote1. Independence bias has no universal sign.
- **UAST:** p.6 Eq.13 is a product, p.4 innovation uses R, not S; numeric thresholds/horizon absent.
- **Eva:** configured approximately1s extrapolation derives from pp.3/5; p.8 states short-loss limit.
- **OA-VAT:** p.4 Eq.9 decreasing sigmoid confirmed; p.4 versusp.5 trigger descriptions still differ.
- **YOPOv2:** p.13 names occlusion a main failure; P3 §1 verifies5×3×14 output.
- **NavRL++:** dropout0.3 and safety-stop rules rechecked in P3 §2; code-release statement is bounded
  to the original manuscript/audited repository, not a live guarantee of perpetual unavailability.
- **FASTER:** p.4 by-construction claim depends on static known-free space and tracking assumptions;
  p.3 velocity bounds are checked at interval starts.
- **AerialGym:** global physics timestep absent; control-rate numbers are not that timestep.
  Eq.4/5 concern is convention-dependent; static mesh randomization description is explicit.

ORTrack complement/sign ambiguity and several author-internal formula discrepancies remain
explicitly unresolved. No source typo is silently converted into a BOIA formula. Detailed version
maps and rendered-page checks are in P1–P6; the design consequence is [13](13_design_deltas.md).
