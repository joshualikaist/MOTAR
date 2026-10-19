# 06 — Prior-art overlap and novelty

**Basis.** The 29 original-copy papers ([01](01_paper_audit.md)), the 11 audited repositories
([03](03_code_audit.md)) and MOTAR's own H-R3 work.

**What counts as "covered".** An element is marked covered only where a page or code reference
supports it; the references are in the notes. **Absence from these29 papers is not evidence of
novelty in the wider literature** (see §3).

## 1. Matrix

Column key:
- **Vis model:** how future target visibility is modelled.
- **Det rel:** whether detector reliability enters the model.
- **Cand:** whether a finite candidate set is evaluated.
- **Capture/terminal:** whether the task has an interception or terminal phase.
- **Arbit:** arbitration over a separate base policy.
- **Fallback:** a runtime safe fallback.
- **GT online in own eval:** GT target or obstacle state was fed online in the paper's own
  simulation.

| Work | Problem | Sensing | Map | Target belief | Vis model | Det rel | Cand | Capture/terminal | Arbit | Fallback | GT online in own eval |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| BPOD | Keep a target visible (ground robot) | Range–bearing / tag camera | Known, convex 2D | EKF + intermittent-observation indicator | **Probabilistic** FOV×Π LOS (Eq.15) | No (perfect) | No (SCP) | No | No | No (chance constraint) | Vicon for evaluation |
| Eva-Tracker | Aerial tracking + re-acquisition | Depth + detector | ESDF-free FoV field | Bézier prediction | Deterministic FoV-ESDF | No | Arc viewpoints | No | No | No | Yes (sim) |
| Elastic Tracker | Aerial tracking | Depth | Grid | CV-EKF | Deterministic sector | No | No | No | No | No | Yes (sim) |
| VisAware TrajOpt | Aerial tracking | Depth | ESDF | Prediction | Deterministic balls on LOS | No | No | No | No | No | Broadcast target |
| Star-Convex | Inspection visibility | Omni point cloud | Known | — | Geometric visible set | No | No | No | No | No | — |
| OA-VAT | Visual active tracking | RGB | No | Image KF, confidence-scaled R | Current confidence | Partly (current) | No | No | Weak (rule switch) | No | No |
| UAST | Search + tracking, UAV | RGB-D | None online | CV-KF, innovation gate | **Learned** LOS/FOV cost (training) | Implicit | **V×H grid** | No | No | No (training loss) | No (z-buffered GT in sim) |
| TrackVLA / ++ | Embodied tracking (ground) | RGB | No | Implicit / memory | `<invalid>` token (current) | Partly | Anchors | No | No | Deploy-level band | No |
| DAT | Drone tracking benchmark | RGB | No | Implicit | — | No | No | No | No | No | Reward only |
| YOPOv2-Tracker | Agile tracking + navigation | RGB-D | Privileged ESDF in training | EKF | None (occlusion is its main failure) | No | **5×3 lattice** | No | No | No | Training labels |
| NavRL++ | RL navigation, sim-to-real | Rays + KF obstacles | Occupancy | — (no target) | — | Dropout training | No | No | **Safety-only override** | **Stop if infeasible** | — |
| PANTHER / Deep-PANTHER | Perception-aware avoidance of dynamic obstacles | Depth | Point cloud | Gaussian obstacle prediction | Sigmoid FOV (no LOS) | No | 6 learned (Deep-) | No | No | Keep previous plan | GT obstacle trajectories |
| RAPTOR | Fast flight, unknown map | Depth | Occupancy/ESDF | — | LOS of unknown space | No | Topological set | No | No | Stopping criterion | — |
| FASTER | Safe fast flight | Depth | Known/unknown split | — | — | — | No | No | No | **Committed safe backup** | — |
| EGO-Planner | Local planning | Depth | Occupancy | — | — | — | No | No | No | Pipe check | — |
| Adaptive Perching | Moving-target tracking + perching | DGPS / tag | ESDF | CTRV EKF | Deterministic balls on LOS | No | Greedy viewpoints | **Terminal state** (relaxed) | No | No | — |
| ORTrack | Passive UAV tracking | RGB | — | — | Occlusion-robust features | Uncalibrated score | — | — | — | — | — |
| PA Time-Optimal | Waypoint flight, perception-aware | Camera | Known | — | tanh FOV + range noise | Range-noise model | No | — | — | — | Mocap |
| PA Unknown/Feature-limited | Navigation, feature visibility | Depth | Online | — | LOS raycast | Count thresholds | Viewpoint graph | — | — | — | — |
| Lee SE(3), Aerial Gym | Control / simulator substrate | — | — | — | — | — | — | — | — | — | — |
| MOTAR H-R3 arms | Re-observation after loss | RGB-D + LiDAR | Raw LiDAR | MOTAR tracker | Deterministic LOS-clear offset | No | 17 offsets | Capture recorded | Event-triggered window | Shared filter | No (firewalled) |
| YOPO original (P3 §6) | Goal navigation | Depth + state | ESDF only in training | — | Trajectory collision cost | None |15anchors | No | No | No | Privileged map training |
| OCTrack (P5 §4) | Passive visual tracking | RGB | — | Template features | COM training masks | Uncalibrated score | — | No | No | No | Ground-truth labels in training/evaluation |
| EGO-Swarm (P6 §1) | Multi-UAV navigation | Depth + neighbor state | Occupancy | Neighbor motion | No target visibility | No | Local optimization | Goal only | No | Planning checks | Neighbor broadcasts |
| PPTracker (P6 §2) | Multi-target UAV-image tracking | RGB + initial boxes | — | Prior tracks | Structured crowd/occlusion features | Uncalibrated | — | No | No | No | First-frame GT boxes online |
| PRIMER (P6 §3) | Multi-agent learned navigation | Depth + neighbors | Local sensor model | Neighbor trajectories | FOV awareness, no explicit LOS forecast | No | Learned trajectories | Goal only | Safety accept/reject | Prior committed trajectory | Expert training state |
| Similarity-Guided ViT (P6 §4) | Passive UAV visual tracking | RGB | — | Template tracking | Layer adaptation, no future LOS forecast | Not calibrated for BOIA | Layer routing | No | No | No | Dataset annotations |
| **BOIA (proposed)** | **Crash-and-capture under occlusion** for a frozen RL interceptor | RGB-D + LiDAR | Sensed points, unknown ≠ free | MOTAR KF with an expected-covariance forecast | **Joint Monte Carlo** P_det over the horizon, from the same samples | **Yes**, p_det(size) and centre-ray LOS in v1; detector stress unvalidated | 20 rolled-out commands, **Policy F's own included** | **Yes**: capture progress in J, terminal-first | **Calibrated advantage over a frozen policy + hysteresis + dwell** | **Stopping-feasible screen + SAFE_HOLD** | No (whitelist + firewall test) |

## 2. Element-wise overlap of BOIA

| BOIA element | Closest prior art | Status of the element |
| --- | --- | --- |
| (a) Belief-conditioned probability of future detection, FOV × LOS | BPOD Eq.9/15 (p.3–4) | **Not new.** Only these changes are new for the MOTAR setting: the joint Monte Carlo over correlated bars instead of the independence product, the 3D-from-2D bar geometry from sensed points instead of a known map, and the detector-reliability factor |
| (a′) Expected covariance under forecast detection | BPOD Eq.11d (p.3) | **Not new.** Reused as published |
| (b) Finite candidate set with rollout scoring | UAST V×H grid (p.5); YOPO 5×3 lattice | **Not new.** New only for MOTAR: Policy F's own command as a candidate, the cost mixing capture progress with loss probability and covariance, and rollout through the measured governor and response model |
| (c) Arbitration over a frozen RL policy by calibrated expected advantage, with hysteresis and dwell | NavRL++ (safety-only override); OA-VAT (rule switch) | **CANDIDATE_CONTRIBUTION_UNVERIFIED.** Generic safety arbitration is covered by PRIMER; narrower calibrated advantage/dwell still pending §3 |
| (d) Stopping-feasible safety screen + safe hold + terminal-first | FASTER (backup); NavRL++ (stop); Deep-PANTHER (keep plan) | **Not new** as a fallback. Its *coupling* to an occlusion-risk arbitration and the terminal-first handling are integration details, not a contribution |
| (e) Mission-level evaluation: crash within 2 s after occlusion onset as a competing risk, plus capture guards | None of the29 (they report tracking success or visibility) | Candidate contribution (evaluation methodology) |
| (f) Detector-realism stress (oracle against perturbed detector) as a factor | NavRL++ trains with dropouts; ORTrack masks occlusions | Methodological necessity, not a contribution |

**Honest contribution statement, provisional.** A **minimally invasive arbitration layer** over a
frozen RL interceptor. It intervenes only when a calibrated expected advantage exceeds a threshold,
and it combines:
- a published belief-visibility forecast (BPOD), adapted to sensed vertical-bar clutter and to
  detector reliability;
- a FASTER-style stopping feasibility test.

It is evaluated on **mission-level competing risks** (crash and capture) under occlusion, which none
of the read works measure.

Mechanisms (a), (b) and (d) are reused, so the paper must not claim them. If (c) turns out to be
covered by the outside-25 literature in §3, the contribution narrows to (e) plus the empirical
finding. That would be `NOT_NOVEL_ENOUGH` for a method claim, and the design would need to change.

## 3. Prior art outside the 25 that must be checked before any novelty claim (not reviewed here)

These lines of work are known to exist. They were **not** in the user's paper set and were **not**
read in this pass:
- **Safety shields over RL policies:** shielding, safety layers, simplex / runtime-assurance
  architectures that switch between a learned controller and a verified fallback.
- **Control-barrier-function safety filters** over learned velocity policies.
- **Residual and arbitrated policies:** blending or switching between a learned policy and a
  model-based planner by estimated value or advantage.
- **Active perception and target search in POMDPs:** information-gain and visibility-aware pursuit,
  pursuit–evasion with occlusions.

Checking these is a separate task: Beads issue `motar-3p3.6` (G1b), "read-only literature check
before any novelty claim". Until it closes, element (c) stays **CANDIDATE_CONTRIBUTION_UNVERIFIED**.

## G1-R novelty correction

Original25+four extras are now read; see [01](01_paper_audit.md) for fresh-read versus retained
identical-copy FULL_READ scope. PRIMER pp.2–3 is direct prior art for learned trajectories followed
by safety rejection and reuse of a committed trajectory. Therefore **generic learned-policy plus
fallback is NOT_NOVEL_ENOUGH**. PARM* is the training expert, not a demonstrated online calibrated
expert/learner advantage switch. The narrower FrozenF/expected-advantage/hysteresis combination
remains **CANDIDATE_CONTRIBUTION_UNVERIFIED**, not proved novel by absence here. Outside-set G1b
remains open; four added papers do not close the shield/CBF/runtime-assurance/residual-policy audit.
Mission-level evaluation is also only a candidate contribution. No new performance is established.
