# 02 — Methods and equations compared

Equation references are to the PDF versions listed in [`01_paper_audit.md`](01_paper_audit.md),
given as p.N Eq.(k). Each formula is labelled:
- **[PAPER]:** as published;
- **[ADAPTED]:** changed to fit MOTAR;
- **[NEW]:** proposed here.

All [ADAPTED] and [NEW] items are hypotheses until the CPU tests of
[`08_implementation_plan.md`](08_implementation_plan.md) pass.

## 1. Target visibility and detection models

| Approach | Formulation | Uncertainty | Detector reliability | Occluder model | Cost per evaluation | Fit to MOTAR |
| --- | --- | --- | --- | --- | --- | --- |
| BPOD [PAPER] | γ_k = γ_FOV · Π_i γ_LOS,i (p.4 Eq.15). Each factor is a Gaussian linear-inequality probability on a linearised signed distance, ½(1 − erf((aᵀx̂ − b)/√(2aᵀΣa))) (p.5 Eq.19) | Robot and target Gaussian beliefs | None (perfect sensor, p.2 footnote) | Known convex 2D polygons; independent across obstacles | Below 1 ms per γ (MATLAB, p.7) | Strong: vertical bars make line of sight a 2D problem. Weak: needs a known map, and independence bias depends on correlation between clear events |
| Visibility-aware Traj. Opt. [PAPER] | Balls c_i along the line of sight; occlusion margin min_i(Ξ(c_i) − r_i) (p.3 Eqs.3–5) | None (deterministic) | None | ESDF Ξ | Not reported | Batchable against LiDAR points |
| Eva-Tracker [PAPER] | A camera-frame FoV-ESDF lookup penalising obstacles inside the frustum (p.3–4); line-of-sight-clear viewpoint on an arc (p.3) | Deterministic Bézier prediction | None | Point cloud within the view frustum | Real-time ROS | Arc-viewpoint generation reusable (AGPL code, so clean-room only) |
| Elastic Tracker [PAPER] | Visible sector around the target and an angular visibility hinge | None | None | Grid map | Real-time ROS | Sector idea reusable (GPL code, so clean-room only) |
| UAST [PAPER] | Learned cost containing L_occ = Σ_i Σ_j ReLU(d_voxel − d_ij) along the line-of-sight ray (p.6 Eq.12) and L_fov (Eq.13), used in **training** | Belief steers a guidance point only | Implicit in the score | SDF in training; no map online | 8.6 ms on an RTX 4090 | Line-of-sight sampling is MIT-licensed and portable (`track_loss.py:114`). The probability itself would be new for MOTAR |
| Star-Convex [PAPER] | Spherical flipping + convex hull gives a point-cloud visibility test | None | None | Point cloud, omnidirectional sensor | Offline | Idea only |
| PANTHER [PAPER] | Sigmoid in-FOV term for a tracked obstacle | Gaussian obstacle prediction | None | No line of sight | ~49–59 ms (NUC) | FOV-only factor |
| PA Time-Optimal [PAPER] | tanh-smoothed FOV product (p.7 Eqs.30–33); range noise ∝ d² (Eq.44); close-range partial-visibility correction (Eq.45) | Fisher information (Eqs.40, 48) | Implicit | None | Offline | Range-dependent measurement noise model |
| **BOIA forecast [NEW]** | P_det(γ, j) = E_{x∼b_T(j\|γ)}[1_FOV(x) · 1_LOS(x; M_t) · p_det(x)], estimated jointly by Monte Carlo over target belief samples and the sensed local point model; P_loss over the horizon computed from the **same** samples (no product over time) | KF belief (MOTAR tracker) | p_det(apparent size) with centre-ray LOS in v1; visible-fraction calibration is a v2 proposal; see07 §5 | LiDAR points → inflated discs; unknown ≠ free | 07 target≤5ms per decision on RTX3070 at N128,K20,M32; NOT_MEASURED | Designed for MOTAR. BPOD's analytic form is kept as a fast path and an ablation |

## 2. Target estimation

| Approach | Model | Gating | On a miss | Identity |
| --- | --- | --- | --- | --- |
| BPOD [PAPER] | EKF with an intermittent-observation indicator μ_k (p.3 Eqs.5–6) | — | Predict only | — |
| UAST [PAPER] | 3D CV-KF, A = [[I, ΔtI], [0, I]] (p.4 Eqs.4–5) | Innovation threshold τ_innov, normalised by R only (as printed) | KF prediction for T_lost frames, then search | Fixed template |
| OA-VAT [PAPER] | Image-plane KF with confidence-scaled measurement noise (p.4 Eq.9) | Confidence threshold | Hand over to the planner | DINOv3 prototypes + EMA |
| YOPO-Tracker [code] | Gated CV-KF (`target_ekf.py:5-132`) | Yes | Brake + yaw spin search | Network detection head |
| MOTAR tracker [as built] | `BatchedConstantVelocityTracker`: CV, diagonal Q, Joseph form (`navrl_perception.py:640-723`) | **None for camera measurements** | Predict; `active` turns off after 5 s; LiDAR resets `age` | None (single hypothesis) |
| **BOIA belief adapter [ADAPTED]** | A read-only shadow copy of the MOTAR belief, propagated in the planner for each candidate; expected-covariance update after BPOD Eq.11d with γ = P_det | Mahalanobis gate with the full S = HPHᵀ + R for planner-side consistency checks (diagnostic) | Shadow prediction; covariance never shrinks without a forecast detection | Camera-only recency counter (or the S-L1C identity clock when that lineage is on) |

## 3. Candidate and decision structures

| Approach | Candidate set | Score | Executes | Switching rule |
| --- | --- | --- | --- | --- |
| UAST [PAPER] | V×H grid of quintic end states (p.5) | Learned cost regressed on the trajectory loss (p.6 Eq.17) | Lowest-cost polynomial (p/v/a) | Rule-based modes: τ_det, τ_innov, T_lost |
| YOPO-Simple / YOPOv2 [code / PAPER] | 5×3 anchor lattice, quintic (`primitive.py:30-84`) | Network-predicted cost | Lowest cost, then an attitude controller | None |
| Deep-PANTHER [PAPER] | 6 learned trajectories | Cost ranking | Best feasible, else the previous plan | Fallback only |
| FASTER [PAPER] | Committed trajectory + new plan | Feasibility tests | New plan only if the safe part is in known-free space; otherwise keep the committed one | Hard tests (p.4) |
| NavRL++ [PAPER] | RL velocity, then projection | Velocity-obstacle safety | Projected command or stop | Safety only |
| OA-VAT [PAPER] | One diffusion sample | — | Planner when confidence is low | Rule threshold, no hysteresis |
| MOTAR H-R3 arms [as built] | Scripted lateral offsets toward a line-of-sight-clear viewpoint (`motar_eval/h_r3_arms.py`) | Smallest clear offset | Scripted action for up to 20 steps after a camera loss | Event trigger, fixed window |
| **BOIA arbitration [NEW]** | Finite set Γ: Policy F's own command, brake, and lattice / lateral-viewpoint commands, all rolled out through a calibrated response + governor model (07 §6) | J = capture progress + loss probability + expected covariance + smoothness + clearance risk, after hard safety and stopping feasibility (07 §7) | Policy F unless the expected advantage over F exceeds τ_on (hysteresis τ_off < τ_on, minimum dwell). A safety override bypasses hysteresis | Mode machine + calibrated advantage + hysteresis (07 §8) |

## 4. Safety

| Approach | Mechanism | What is guaranteed | Assumptions |
| --- | --- | --- | --- |
| FASTER [PAPER] | Committed trajectory in known-free space, ending at rest | Collision-free by construction (p.4) | Static world, perfect tracking; velocity checked at interval starts only |
| BPOD [PAPER] | Collision chance constraint Pr(x ∈ O) < δ (p.4 Eqs.16–17) | Probabilistic (linearised) | Known convex map |
| EGO-Planner [PAPER] | Soft collision term + pipe post-check | Structural feasibility only | Static map |
| NavRL++ [PAPER] | Velocity-obstacle projection; stop if infeasible | Empirical | Accurate obstacle tracks |
| MOTAR riskcap [as built] | Speed cap 2.0 + clamp((c − 3)/2, 0, 1)·1.5355 m/s (`speed_governor.py:466-475`) | **Not a brake.** The 2.0 m/s floor leaves a −0.88 m stopping margin at 0.1 m clearance (EQ-1 record, see [M2 §3](notes/M2_dynamics_contract.md)) | — |
| **BOIA screen [NEW + ADAPTED from FASTER]** | Swept-disc clearance along each rollout, plus a terminal stopping-feasibility test in known-free space, using the measured braking table | Empirical only. No formal guarantee: unknown space, sensor errors, model error | Calibrated response model (V4); LiDAR returns with the target excluded through the belief |

## 5. Dynamics and control (reference for the rollout model)

- **Lee SE(3) [PAPER].**
  - Equations of motion, p.2 Eqs.2–5: ẋ = v, mv̇ = mg e3 − fRe3, Ṙ = RΩ̂, JΩ̇ + Ω×JΩ = M. Here e3
    points **down**.
  - Stability, Props. 2–3: needs Ψ(0) < 1 (attitude error below 90°) and bounded desired acceleration.
- **MOTAR as built.** See [M2 §4](notes/M2_dynamics_contract.md).
  - **Controller:** a Lee *velocity* loop, z-up, thrust along +b3; f = m(K_v(v_d − v) + g e3) with
    K_v = 2.5.
  - **Tilt:** capped at 45°, with T = f_z/b3_z.
  - **Motors:** first order, τ = 0.04 s, 0–2 N each.
  - **Rates:** physics dt 0.01 s ×10 per 0.1 s decision.
  - Because the thrust law differs from Lee's Eq.12, Lee's stability proof **does not** apply.
- **Rollout model [ADAPTED, to be calibrated].**
  - Horizontal velocity responds first order, τ_v ≈ 1/K_v ≈ 0.4 s. This is **derived**, not measured.
  - The command is in the yaw-only vehicle frame, re-rotated by the current yaw.
  - Speed is capped by the exact riskcap law.
  - The altitude PI holds z.
  - Braking table (EQ-1D D1-B1): 0.82, 1.05 and 1.57 m from 2.0, 2.5 and 3.52 m/s.

## G1-R original-copy amendment (2026-10-19)

The original-copy references in P1–P6 supersede historical version-specific citations above.

| Work | Original mechanism | Implication |
| --- | --- | --- |
| YOPO original, P3 §6 | 15 anchors; score supervised as−J (p.5 Eq.21), high-score selection; learned quintic trajectory interface | Distinct from the cost/argmin YOPO-Simple branch; BOIA's explicit J minimization and heading/speed lattice are its own adaptation |
| OCTrack original, P5 §4 | COM, ACP and layer-depth CADB, structured occlusion training | No metric-depth obstacle model and no calibrated p_det(size,visible fraction) |
| PRIMER, P6 §3 | RMADER Check/Delay Check after learned trajectory prediction; reject/reuse committed trajectory (pp.2–3) | Generic learned-policy/safety fallback already exists; calibrated FrozenF advantage is a narrower unverified candidate |
| PPTracker, P6 §2 | First-frame GT-box online initialization (p.3), then prior tracked boxes | No permission to introduce GT initialization into BOIA or equate active track with identity |

BPOD p.4 Eq.15 explicitly assumes independence. Positive-correlated clear events can make its
product pessimistic about clearance, while other correlations reverse the bias. UAST p.6 Eq.13
multiplies two ReLUs; it does not add them. PANTHER's p.8 optimization is unnumbered, not Eq.3.
Lee original CDC uses b3d/f/M Eqs12/15/16; its six-page reference replaces the8-page equation map.
The forecast's within-sample temporal product still assumes conditionally independent detector
errors in stressed mode (07 §5); this is distinct from BPOD's between-occluder independence.
