# P1 — Visibility-aware planning for target tracking: deep-read notes

Reader notes for the MOTAR occlusion-aware navigation project. Compiled 2026-10-19.

Conventions:
- "p.N" = physical PDF page index (1-based) of the file named in each section.
- Equation numbers are those printed in the paper. `[UNCERTAIN]` marks a transcription I could not fully verify from the rendered page and the pdftotext output.
- Numbers quoted from experiments are context only. They do not transfer to MOTAR.
- "Observed" limitations are my own reading. "Stated" limitations are the authors' own words, with the page given.
- Method of reading: every page of every PDF was rendered with the Read tool (images). pdftotext was used to cross-check equation text (BPOD Eq. 18, Star-Convex Eq. 10, Elastic Eq. 14).
- File identity: identity status is as given in the task brief (I did not have the user's copies to compare). For paper 5 the brief says the local file is a DIFFERENT version from the user's copy. I cannot say what differs, because I only had this file.

---

## 1. BPOD — Probabilistic Visibility-Aware Trajectory Planning for Target Tracking in Cluttered Environments

- **Authors:** Han Gao, Pengying Wu, Yao Su, Kangjie Zhou, Ji Ma, Hangxin Liu, Chang Liu (Peking University / BIGAI). p.1
- **Venue/year as printed:** arXiv:2306.06363v2 [cs.RO], 27 Aug 2024 (side stamp p.1). No conference or journal name is printed in the PDF. Ref [20] on p.8 is a self-citation to the arXiv version, 2023.
- **File:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/BPOD__2306.06363v2.pdf
- **Identity status:** identical to the user's copy (per brief).
- **Pages read:** 1, 2, 3, 4, 5, 6, 7, 8 (all 8; there is no appendix).
- **Read status:** FULL_READ.

### Problem
A ground robot tracks a stochastically moving target in a 2D cluttered environment. It plans a non-myopic, real-time trajectory that keeps the target visible (limited FOV plus obstacle occlusion) and avoids collision under robot, target and measurement uncertainty. MPC/belief-space planning (BSP). p.1–2

### Assumptions
- **Dimension:** 2D environment. p.2 Sec. II
- **Map:** a *known* continuous map with convex obstacles given as point sets O_i ⊂ R², i=1..N_o. Non-convex obstacles are split into convex pieces. p.2 Sec. II-A
- **Obstacles:** static. Future work: "extend the proposed method to unknown and dynamic environments." p.8
- **Robot:** knows its current state. Robot and target motion are stochastic, with additive Gaussian process noise (Eqs. 1–2). p.2
- **Sensor:** limited FOV V_k ⊂ R² and intermittent measurements (Eq. 3).
  - Footnote 1 (p.2): a "perfect sensor model": a measurement is returned **iff** the target is inside the FOV and not occluded.
  - The authors claim the method "can be easily extended to imperfect sensor models that give false positive or false negative measurements". This is not demonstrated.
- **Simulation FOV:** an annular sector with r1=2 m, r2=10 m and sensing angle 2π/3. It is shrunk to its convex subset when computing Eq. 13, to fit GJK/EPA. p.6
- **Simulation sensors:** range–bearing (Eq. 23) in V-A. A "camera" model of distance, bearing and relative orientation (Eq. 24) in V-B. p.6–7
- **Target motion model:**
  - Single integrator with known control u^t in V-A. p.6
  - Unicycle with *unknown* controls in V-B, estimated by differentiating target displacement and rotation. p.7
  - Over the horizon, the target mean is propagated with the *same* control input held constant: u^t_{k+i} = u^t_{k−1}. p.3
- **Prediction horizon:** N=4 steps with Δt=0.5 s, so 2 s. p.6

### Method pipeline
1. **Target estimation:** an EKF variant with intermittent observations (after Sinopoli et al. [18]). Predict with Eqs. 4a–b. Update with Eqs. 5a–c, gated by a detection variable μ_k ∈ {0,1} (Eq. 6). p.2–3
2. **MPC problem (Eq. 7a–e):** minimise J over robot controls u^r_{k:k+N−1}.
   - Belief propagation g^t, g^r.
   - Feasible sets for beliefs and controls.
   - Collision constraint f^o(b^r, O_j) < 0. p.3
3. **Belief prediction in the horizon** (Sec. III-A, p.3):
   - The robot belief is propagated by an EKF prediction step (Eq. 8).
   - The target mean is propagated by its kinematics.
   - The target covariance gets an EKF prediction followed by a *visibility-aware expected update* (Eq. 11), weighted by BPOD γ_k.
4. **BPOD factorisation (Sec. IV-A, p.4):** BPOD is the FOV-membership probability times the product of per-obstacle LOS-free probabilities (Eqs. 13–15).
5. **Collision chance constraint:** γ^ro_{k,i} < δ^s (Eqs. 16–17). p.4
6. **PoSSDF (probability of stochastic SDF satisfaction):**
   - Each probability (FOV, LOS, collision) is rewritten as Pr(sd(·,·) ≤/≥ 0) (Eq. 18).
   - The SDF is linearised about the Gaussian mean (Alg. 1, with GJK/EPA for closest points and the contact normal).
   - The resulting Gaussian linear-inequality probability is evaluated with erf (Eq. 19). p.4–5
   - LOS variant: the closest point on the LOS segment is re-parameterised by a ratio λ̂ (Eq. 20). p.5
7. **Optimiser:** SCP with an l1 penalty (Eq. 21), a trust region, and an outer loop on the penalty coefficient (Alg. 2).
   - "Two-stage" speed-up: the SDF parameter set C_0 is held *fixed* while computing gradients, and is updated only after a new solution is obtained.
   - Solver: MOSEK. p.5–6
8. **Objective:** either J1 = cumulative entropy of the target belief, or J2 = −cumulative BPOD (Eq. 12). p.4

### Key equations
| Ref | Equation (transcribed) | Symbols / notes |
|---|---|---|
| p.2 Eq.(1)–(2) | z^r_{k+1} = f^r(z^r_k,u^r_k)+w^r, w^r~N(0,R^r); z^t_{k+1} = f^t(z^t_k,u^t_k)+w^t, w^t~N(0,R^t) | robot (r) and target (t) state z, control u, zero-mean Gaussian process noise |
| p.2 Eq.(3) | y_k = f^s(z^t_k,z^r_k)+w^s if x^t_k ∈ V^v_k; y_k = ∅ if x^t_k ∉ V^v_k | V^v_k = the part of the FOV V_k not occluded by any obstacle; w^s ~ N(0,R^s) |
| p.2 Eq.(4a,b) | ẑ^t_{k\|k−1} = f^t(ẑ^t_{k−1\|k−1}, u^t_{k−1}); P_{k\|k−1} = A P A^T + R^t | standard EKF prediction; A = Jacobian of f^t |
| p.3 Eq.(5a–c) | K_k = P C^T(C P C^T+R^s)^{-1}; ẑ_{k\|k} = ẑ_{k\|k−1}+μ_k K_k(y_k − f^s(·)); P_{k\|k} = P_{k\|k−1} − μ_k K_k C_k P_{k\|k−1} | C = measurement Jacobian; μ_k = detection variable |
| p.3 Eq.(6) | μ_k = 1 if y_k ≠ ∅, 0 if y_k = ∅ | — |
| p.3 Eq.(9) | γ_k = Pr(y_k ≠ ∅ \| b^r_k, b^t_{k\|k−1}) | **BPOD**: detection probability conditioned on the robot belief [ẑ^r, Q] and the predicted target belief [ẑ^t, P] |
| p.3 Eq.(10) | μ_k = Pr(y_k ≠ ∅ \| z^r_k, z^t_k), so γ_k = E_{z\|b}(μ_k) | total probability rule (TPR) |
| p.3 Eq.(11a–d) | P̃_{k\|k} = E_{z\|b}(P_{k\|k}) ≈ P_{k\|k−1} − γ_k K̃_k C̃_k P_{k\|k−1} | the **expected posterior covariance** given detection probability. C̃ is the Jacobian evaluated at the *mean* robot state ẑ^r_k; K̃ is from Eq. 5a with C̃. (11b→11c) uses "the independency of K̃_k and C̃_k from z^t_k and z^r_k". |
| p.4 Eq.(12) | J1 = Σ_{i=1..N} H(b^t_{k+i}); J2 = −Σ γ_{k+i}; H-sum = N d_t/2 (ln(2π)+1) + Σ ½ ln\|P̃_{k+i\|k+i}\| | d_t = dimension of z^t |
| p.4 Eq.(13) | γ^tf_k = Pr(x^t_k ∈ V_k \| b^r_k, b^t_{k\|k−1}) | probability the target is inside the FOV |
| p.4 Eq.(14) | γ^lo_{k,i} = Pr(L_k ∩ O_i = ∅ \| ·) | L_k = LOS segment from the robot position to x^t_k; probability that obstacle i does not occlude |
| p.4 Eq.(15) | γ_k = γ^tf_k Π_{i=1}^{N_o} γ^lo_{k,i} | **independence assumption**: "we can assume that all variables in {γ^tf, γ^lo_{k,1..N_o}} are mutually independent" (p.4) |
| p.4 Eq.(16)–(17) | γ^ro_{k,i} = Pr(x^r_k ∈ O_i \| ·) < δ^s | collision chance constraint |
| p.4 Eq.(18a–c) | γ^tf = Pr(sd(x^t_k,V_k) ≤ 0); γ^lo = Pr(sd(L_k,O_i) ≥ 0); γ^ro = Pr(sd(x^r_k,O_i) ≤ 0) | sd = signed distance (PoSSDF). Inequality directions were checked against pdftotext. |
| p.5 Eq.(19) | Pr(a^T x ≤ b) = ½(1 − erf((a^T x̂ − b)/√(2 a^T Σ a))) | exact for Gaussian x; used after the SDF linearisation (Alg. 1, lines 4–7) |
| p.5 Eq.(20) | p_1(x) = λ̂ x^r_k(x) + (1−λ̂) x^t_k(x), λ̂ = ‖p̂_1 − x̂^t_k‖/‖x̂^r_k − x̂^t_k‖ | closest point on the LOS, re-parameterised so that it moves with both robot and target |
| p.5 Eq.(21) | J_m = J + η(Σ\|g^n_i\|^+ + Σ\|h^n_i\|) | SCP l1 penalty |
| p.6 Eq.(22) | unicycle: z^r_{k+1} = z^r_k + [v cosθ, v sinθ, ω, a]^T Δt | Δt=0.5 s; a ∈ [−4,2] m/s², ω ∈ [−π/3, π/3] rad/s, v ≤ 4 m/s |
| p.6 Eq.(23), p.7 Eq.(24) | f^s = [‖x^t−x^r‖, ∠(x^t−x^r) − θ^r] (V-A); plus the target heading term θ^t − θ^r (V-B) | range–bearing (V-A); camera-like model (V-B) |

Assumptions the paper itself states:
- Gaussian process and measurement noise.
- EKF linearisation.
- Measurement Jacobian evaluated at the mean robot belief (p.3).
- Independence of FOV and per-obstacle LOS events (p.4).
- SDF linearised around the Gaussian mean, so PoSSDF is a Gaussian linear-inequality probability (p.5).
- The collision approximation over-estimates γ^ro, because the obstacle is expanded to a half-space. The authors argue this keeps Eq. 17 strictly satisfied (p.5).
- Perfect detector (footnote, p.2).

### Online inputs vs ground truth
- **Map:** the obstacle map is assumed known, with convex polygons (p.2).
- **Robot state:** the robot "knows its current state" (p.2).
- **Target:** estimated by the EKF from noisy intermittent measurements. In V-A the target control is known to the robot (p.6). In V-B it is unknown and estimated by differencing (p.7).
- **Real world:** AprilTag detection on a Jetson TX1 gives distance, bearing and orientation. Planning runs on a laptop.
  - "The ground truth of the robot and target poses are obtained by a Vicon motion-capture system." (p.7)
  - It is not stated explicitly whether the planner consumes Vicon robot pose online or uses it only for evaluation.

### Decision logic
- There are no explicit search/track/reacquire modes. A single MPC objective is used throughout.
- Loss of the target is handled implicitly. μ=0, so the covariance grows through Eq. 11d and the planner tends to reduce entropy or raise BPOD.
- **Failure definition (evaluation only):** collision, or losing sight for 15 consecutive steps (p.6).
- **Termination:** the SCP convergence tolerances (Alg. 2). There is no task-level termination logic.

### Timing / compute
- **Contributions:** the planner reaches "a computational speed of 10Hz" (p.2).
- **Simulation setup:** MATLAB on a 12th-gen Intel i7 CPU @ 2.10 GHz, with the MOSEK solver (p.6).
- **Scenario V-A:**
  - "we can calculate one BPOD within 1ms with an MAE of 2.5×10⁻³" against Monte-Carlo ground truth (p.7).
  - Mean computing time 0.176 s/step (p.7).
- **Scenario V-B:** "the overall duration remains below 0.1 seconds" (p.7, Fig. 7 box plots).
- **Real world:** the laptop (11th-gen i7 @ 2.30 GHz) reports t_cal ≈ 0.087–0.090 s/step (Table I, p.8).

### Experiments
- **V-A (challenging scenario):**
  - 60×50 m map of polygon obstacles; the target trajectory has sharp turns near obstacles.
  - 30 runs, metrics r_suc, e_est, r_los and t_cal (p.6–7).
  - Qualitatively, the robot takes "roundabout" paths away from obstacles before the target turns, to avoid loss (Fig. 4, p.6).
- **V-B (stochastic target trajectories):**
  - Baseline: the deterministic visibility cost of Wang et al. [11] (= paper 4 here) embedded in the *same* MPC/SCP framework.
  - Noise levels β ∈ {0.5, 2, 5}, 50 random trajectories per level, target speed 3 m/s (p.7).
  - Qualitatively, both are similar at low noise. At high noise the baseline degrades sharply in success rate and loss rate, while BPOD stays robust, at somewhat higher compute (Fig. 7, p.7).
- **VI (real world):**
  - Ground robot with an ORBBEC Femto W camera tracking a Turtlebot3 carrying AprilTags.
  - Three runs of more than 2 min each in an indoor box-obstacle map.
  - Metrics: e_est, t_cal and minimum obstacle distance (p.7–8).

### Limitations
- **Stated:** future work is non-Gaussian BSP, and unknown and dynamic environments (p.8). The perfect-sensor assumption is stated in the footnote (p.2).
- **Observed:**
  - 2D only, with a known convex-polygon map. GJK/EPA requires a convex FOV, so the FOV is shrunk (p.6).
  - Target prediction holds the last control constant over 2 s, with no multi-hypothesis.
  - Independence across obstacles (Eq. 15) is optimistic when occluders are correlated (adjacent bars).
  - There is no detector false-negative model, so a visible but undetected target is impossible by construction.
  - There is no explicit reacquisition behaviour.
  - Ground robot (unicycle); quadrotor or 3D aspects are not addressed.
  - Real-time claims are from MATLAB on a desktop or laptop CPU.

### Code
None printed in the PDF.

### MOTAR relevance
**Reusable:**
1. **γ = γ^tf · Π γ^lo (Eq. 15).** This gives a cheap, belief-conditioned detection-probability forecast.
   - With *vertical bars*, LOS occlusion is essentially a 2D horizontal-plane problem (circle obstacles).
   - So BPOD's 2D restriction is far less of a conflict for MOTAR than it would be for general 3D clutter.
   - The SDF between a segment and a circle is analytic, so GJK/EPA is unnecessary.
2. **Eq. 19 erf closed form on a linearised SDF.** This gives Pr(LOS clear) and Pr(in FOV) from the KF covariance in well under 1 ms per evaluation (p.7). It is suitable for scoring a finite candidate set.
3. **Expected covariance update (Eq. 11d).** P̃ = P⁻ − γ K̃ C̃ P⁻ is a direct, principled "covariance growth" term for a candidate cost. MOTAR's Kalman tracker already provides P⁻, K and C.
4. **Detection variable μ_k gating the KF (Eqs. 5–6).** This matches a KF that coasts on misses.

**Conflicts:**
- The known map: MOTAR has only the local LiDAR/RGB-D map, with bars observed online.
- Perfect detector: MOTAR needs a detector-reliability factor p_det(range, bearing, partial occlusion).
- Continuous SCP/MOSEK MPC: MOTAR uses a frozen RL policy, not an optimiser.
- Unicycle ground robot.
- Constant-control target extrapolation.

### BOIA novelty-overlap check
| BOIA element | BPOD does it? | Evidence |
|---|---|---|
| (a) belief-conditioned forecast of future detection probability under FOV + LOS + detector reliability | **Partly (strong overlap on FOV+LOS)** | Eq. 9 (p.3) defines exactly a belief-conditioned detection probability. Eqs. 13–15, 18–19 (p.4–5) cover FOV and LOS under robot and target Gaussian beliefs, over an N-step horizon. **Detector reliability is not modelled**: perfect sensor (footnote, p.2), with extension only claimed. |
| (b) finite set of dynamics-feasible short-horizon candidates; cost = capture progress + loss-of-track probability + covariance growth | **Partly** | The cost uses cumulative entropy of the expected covariance (J1, Eq. 12 with Eq. 11d) or cumulative BPOD (J2); both are p.4. It is a continuous SCP-optimised MPC, not a finite candidate set. There is no capture/progress term (the goal is tracking). |
| (c) arbitration overriding a frozen RL policy only when a calibrated expected advantage exceeds a threshold, with hysteresis | **No** | The MPC always acts. There is no learned policy and no arbitration (p.3–6). |
| (d) explicit safe fallback and terminal-first handling | **No** (collision chance constraint only) | Chance constraint Eq. 17 (p.4); there is no fallback mode and no terminal phase. |

---

## 2. Eva-Tracker — ESDF-update-free, Visibility-aware Planning with Target Reacquisition for Robust Aerial Tracking

- **Authors:** Yue Lin, Yang Liu*, Dong Wang, Huchuan Lu (Dalian University of Technology). p.1
- **Venue/year as printed:** arXiv:2602.12549v1 [cs.RO], 13 Feb 2026 (side stamp p.1). No venue is printed.
- **File:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/EvaTracker__2602.12549v1.pdf
- **Identity status:** identical (per brief).
- **Pages read:** 1, 2, 3, 4, 5, 6, 7, 8 (all 8; no appendix).
- **Read status:** FULL_READ.

### Problem
A quadrotor tracks a moving target (a person) in cluttered 3D environments. The planner must:
- keep the target visible: no occlusion, an appropriate distance, and the target centred;
- avoid collision;
- recover after short target loss.

It must also avoid per-frame ESDF updates, which are expensive. p.1

### Assumptions
- **3D environment, unknown and online.** Local mapping from LiDAR (Fig. 2, p.2). Obstacles may move: obstacle points w_{k,t} are "updated online to handle dynamic occlusions" (p.4).
- **Sensors:**
  - A camera with a fixed, known FOV, modelled as a quadrilateral pyramid with horizontal angle α, vertical angle β and depth D (Eq. 4, p.3).
  - Simulation FOV: 69.4°×42.5° (p.5).
  - Real world: Livox Mid-360 LiDAR plus a camera (p.6–7).
- **Target motion model:** a smooth Bézier fitted through the last m+1 observations, extrapolated forward over T = mτ (Eqs. 1–2, p.3). With τ=0.5 s and m=2 (p.5), the prediction horizon is ≈1 s, from 3 observations. The fit is deterministic, with no uncertainty.
- **Rotation:** the FOV rotation uses yaw only (R(ψ), Eq. 13, p.4). Pitch and roll of the camera are ignored in the visibility cost (observed).

### Method pipeline
1. **Perception (p.2, p.7):**
   - Detection with YOLOv11 skeleton keypoints, then MonoLoco 3D pose.
   - Self-localisation with Faster-LIO.
   - LiDAR points within the estimated target radius are removed before mapping.
2. **Target trajectory prediction:** an EQP fit of a Bézier curve (degree n=2m) through the history, minimising the second derivative, in closed form (Eqs. 1–2, p.3).
3. **Visibility-aware initial path** (p.3, Fig. 3). For each future k:
   - Place a candidate c_k at the preferred distance d from the predicted target ρ(kτ), along the horizontal ray toward the previous waypoint (Eq. 3).
   - Check the line c_k→ρ(kτ) for occlusion.
   - If it is occluded, do a bidirectional search along a circular arc of radius d around ρ(kτ) for the nearest occlusion-free point.
   - This yields waypoints p_1..p_m.
4. **FoV-ESDF (p.3–4):** an ESDF computed offline once in the camera frame.
   - Value = Euclidean distance to the nearest FOV boundary, for points inside the FOV; 0 outside.
   - Built with a distance transform [23]; queried by interpolation.
   - Obstacle points are transformed into the tracker frame at each step, so no map ESDF is needed.
5. **MINCO trajectory (p.4):** positions and yaw, with waypoints P, Ψ and durations t (Eqs. 5–6).
6. **Optimisation:** the cost J_v + J_e + λ_p J_p (Eq. 7), solved with L-BFGS plus Lewis–Overton line search.
   - **Visibility J_v** = λ_o·occlusion + λ_v·observation-distance + λ_a·angle (Eq. 8), using numerical integration (Eqs. 9–11).
   - **Energy J_e:** jerk plus time-sync regulariser (Eq. 19).
   - **Penalties J_p:** collision via RC-ESDF [14], plus velocity, acceleration, angular velocity and angular acceleration limits (Eq. 20).

### Key equations
| Ref | Equation | Notes |
|---|---|---|
| p.3 Eq.(1) | ρ(t) = Σ_{i=0}^{n} n!(T+t)^i (T−t)^{n−i} / (i!(n−i)!(2T)^n) · ϱ_i, t∈[−T,T] | Bézier with control points ϱ_i; n=2m, T=mτ; past t≤0, future t>0 |
| p.3 Eq.(2) | min_ϱ ∫_{−T}^{T} ‖ρ^{(2)}(t)‖² dt s.t. ρ(iτ−T) = h_i, i=0..m | interpolates the m+1 historical observations h_i exactly (no noise model) |
| p.3 Eq.(3) | c_k = ρ(kτ) − d · Π₂(ρ(kτ)−p_{k−1}) / ‖Π₂(ρ(kτ)−p_{k−1})‖ | Π₂ = horizontal projection; d = preferred observation distance |
| p.3 Eq.(4) | FOV = {(x,y,z) \| 0<x≤D, \|y\|/x ≤ tan(α/2), \|z\|/x ≤ tan(β/2)} | D chosen so that (d,0,0) has the maximum FoV-ESDF value |
| p.4 Eq.(7) | min_{P,Ψ,t} J_v + J_e + λ_p J_p | — |
| p.4 Eq.(8) | J_v = λ_o J_o + λ_v J_v + λ_a J_a | the symbol J_v is reused for both total visibility and the distance term, as printed; p.5 lists λ_d=10³, not λ_v (naming inconsistency in the paper) |
| p.4 Eq.(9) | J_μ = Σ_i (t_i/k_i) Σ_j η_j f_μ(p_i, ψ_i, ρ_i, j t_i/k_i), μ∈{o,v,a} | trapezoid weights η=(0.5,1,…,1,0.5) |
| p.4 Eq.(12a,b) | f_o = ½(Σ_{k=1}^{N_t} Ξ(v_{k,t}))²; v_{k,t} = R(ψ_i(t))(w_{k,t} − p_i(t)) | Ξ = FoV-ESDF; w = obstacle points in the world frame; N_t = number of obstacle points inside the FOV. This pushes obstacle points out of the FOV. |
| p.4 Eq.(13) | R(θ) = [[cosθ, sinθ, 0], [−sinθ, cosθ, 0], [0,0,1]] | yaw-only |
| p.5 Eq.(15) | f_v = ½[Ξ(d e₁) − Ξ(R(ψ_i)δ_i(t))]² | δ_i = ρ_i − p_i; drives the target to the FoV-ESDF maximum (centred at distance d) |
| p.5 Eq.(16)–(17) | φ_i = atan2(e₂ᵀδ_i, e₁ᵀδ_i); f_a = 1 − cos(ψ_i − φ_i) | yaw alignment |
| p.5 Eq.(19) | J_e = Σ ∫(‖p^{(s)}‖² + (ψ^{(s)})²)dt + γ(T − Σt_i)² | s=3 (jerk); syncs the tracker horizon with the target prediction horizon |
| p.5 Eq.(20a–d) | J_p = Σ (t_i/k_i) Σ η_j Σ_{μ∈P} G_μ; G_d = H(p,ψ) (RC-ESDF); G_{p_h} = max{‖p^{(h)}‖² − v_h², 0}; G_{a_h} = max{(ψ^{(h)})² − ω_h², 0} | h∈{1,2} |

Assumptions stated by the paper: deterministic Bézier prediction; fixed camera FOV (stated as a limitation on p.8). There are no probabilistic or Gaussian assumptions. Visibility is geometric only.

### Online inputs vs ground truth
- **Simulation:** "During the experiment, the target's position is continuously transmitted to the tracker." (p.5) The target position is ground truth in simulation.
- **Real world:** the target pose comes from YOLOv11 + MonoLoco. Self-pose comes from Faster-LIO. The map is built online from LiDAR (p.7).
- The occlusion check in path generation uses the current local map. Obstacles outside the FOV are ignored in the visibility cost (p.4, Fig. 4).

### Decision logic
- There are no explicit modes. On loss, "the algorithm continues to generate paths using predicted positions, enabling prompt reacquisition" (p.2–3).
- When the target reappears, "the tracker seamlessly transitions back to normal tracking without the need for additional recovery mechanisms" (p.3, p.7).
- **Real world:** "When visual detection fails, the predicted position is used to attempt target reacquisition." (p.7)
- There are no switching thresholds and no search mode. Reacquisition is just tracking the extrapolated prediction.

### Timing / compute
- **Simulation hardware:** Intel NUC Phantom Canyon, i7-1165G7 CPU plus RTX 2060 GPU (p.5).
- **Table II (p.6), average:** path 0.02±0.01 ms, optimisation 10.73±3.89 ms, total 10.75 ms. Baselines: Elastic 7.12 ms total, Vis-Planner 36.69 ms, SF-Tracker 37.52 ms.
- **Real world:** all computation is onboard a Jetson Orin NX (p.6–7). No onboard timing is reported.

### Experiments
- **Benchmark simulation (p.5–6):**
  - 100 random obstacles; a 200 s target trajectory with sharp turns and approach toward the tracker.
  - Identical FOV for all methods; d = 2.5 m; tracker limits 2 m/s and 1.5 rad/s.
  - Metrics: tracking distance (TD), yaw angle error (AE), occlusion rate (OR) and failure rate (FR). Failure = the target is occluded or leaves the FOV (p.6).
  - Baselines: Elastic Tracker [1], Vis-Planner [9] (= paper 4), SF-Tracker [10].
  - Qualitatively, Eva-Tracker has the lowest occlusion and failure rates and the most consistent distance. Elastic has the best yaw error but "struggles with managing 3D occlusions" (p.6, Table I).
  - FOV heatmaps: Fig. 6.
- **3D scene (Fig. 7, p.6):** an n-shaped tunnel plus a two-layer obstacle. Elastic loses the target in the tunnel; Eva-Tracker does not.
- **Real world (p.7):**
  - Outdoors among trees, including a thin landmark (Fig. 8).
  - Indoors: the target walks through a door that then closes. The tracker flies around using the prediction and reacquires on the other side (Fig. 9).

### Limitations
- **Stated (p.8):**
  1. "the target reacquisition mechanism is effective only for short-term losses": prolonged occlusion with deviation from the prediction fails.
  2. The fixed camera FoV assumption excludes zoom or gimbal cameras.
- **Observed:**
  - The prediction is deterministic and uses only 3 points. There is no covariance, so it cannot express "where the target might be".
  - Visibility is binary or geometric; there is no detection probability.
  - The occlusion cost sums FoV-ESDF over all obstacle points in the FOV, including points *behind* the target. It is unclear whether points farther than the target are excluded; the text does not say.
  - Yaw-only rotation.
  - The simulation uses the broadcast ground-truth target position.

### Code
https://github.com/Yue-0/Eva-Tracker (p.1 abstract).

### MOTAR relevance
**Reusable:**
1. **The FoV-ESDF idea:** a fixed camera-frame field that scores how deep a point is inside the FOV.
   - MOTAR's forward RGB-D camera has a fixed FOV.
   - A precomputed camera-frame lookup gives a smooth "inside-FOV margin" for the predicted target and for LiDAR bar points, with zero map maintenance.
   - It fits a GPU batched evaluator well, since it is just a table lookup after a rigid transform.
2. **The arc-search occlusion-free viewpoint** (Eq. 3 plus the arc search) is an excellent *candidate generator*. Candidates are placed on a circle of radius d around the predicted target and checked for LOS clearance against bars. This maps directly onto a finite candidate set.
3. **The sensor suite** is a close analogue: Livox Mid-360 LiDAR plus a camera, with points within the target radius removed from the map.

**Conflicts:**
- MINCO/L-BFGS optimisation, not a policy.
- Deterministic prediction.
- The simulation uses the ground-truth target position.
- No probabilistic treatment of loss.

### BOIA novelty-overlap check
| BOIA element | Eva-Tracker? | Evidence |
|---|---|---|
| (a) belief-conditioned detection-probability forecast (FOV+LOS+detector) | **No** (geometric only) | Deterministic Bézier prediction (Eqs. 1–2, p.3). FOV and occlusion are encoded geometrically via the FoV-ESDF (p.3–4). There are no probabilities. |
| (b) finite dynamics-feasible candidates + cost (progress, loss-prob, covariance) | **Partly** | A discrete candidate search exists for *waypoints* (arc search, p.3), but it only picks the nearest occlusion-free point, with no scored cost. The final trajectory is continuous optimisation (Eq. 7, p.4). There is no loss probability or covariance term. |
| (c) calibrated-advantage arbitration over a frozen RL policy with hysteresis | **No** | — |
| (d) explicit safe fallback + terminal-first handling | **No** | Collision is handled only as a penalty (Eq. 20b, p.5). Reacquisition is implicit (p.3, p.7); there is no fallback mode. |

---

## 3. Elastic Tracker — A Spatio-temporal Trajectory Planner for Flexible Aerial Tracking

- **Authors:** Jialin Ji, Neng Pan, Chao Xu, Fei Gao (Zhejiang University, FAST Lab). p.1
- **Venue/year as printed:** arXiv:2109.07111v2 [cs.RO], 2 Mar 2022 (side stamp p.1). No venue is printed in this PDF. Eva-Tracker (paper 2, p.8 ref [1]) cites it as ICRA 2022, pp. 47–53.
- **File:** /home/fair/.claude/projects/-home-fair-workspaces-aerial-gym-ws/626171c0-3144-45d9-ba31-4e1386f262dc/tool-results/webfetch-1789740878632-4s1lxh.pdf
- **Identity status:** identical (per brief).
- **Pages read:** 1, 2, 3, 4, 5, 6, 7 (all 7; p.7 is references).
- **Read status:** FULL_READ.

### Problem
Aerial tracking of a moving person with guaranteed safety and visibility. "Elastic" means the distance is allowed to stretch or compress as needed, rather than being rigidly held. A joint spatio-temporal trajectory optimisation runs inside a safe flight corridor. p.1

### Assumptions
- **3D, unknown environment, online mapping.**
  - The drone localises and maps with a RealSense D435, citing VINS-Mono [22]. p.5
  - Yaw heads toward unknown space if the future trajectory enters the unknown area. p.5
- **Sensor:** a mono camera for the target, FOV 85°×72° in the real world (p.5). In simulation the camera is 640×480 with an 80°×65° FOV (p.6).
- **Visibility model:** a *sector* (cone) around each predicted target position, defined by a seed visible point s_k, a bisector ξ_k and a half-angle θ_k (Eq. 4, p.3). Eva-Tracker calls this "2D sectors" (paper 2, p.2).
- **Target:**
  - Human detection with OpenPose, tracked by an EKF "under the assumption of the constant velocity" (p.5).
  - Prediction: "generate motion primitives for the target and choose a safe and minimum acceleration one" (p.5).
  - Prediction duration T_p (Eq. 1). The value is not given in the text.

### Method pipeline
1. **Detection and EKF (CV); target prediction by motion primitives** (p.5). This yields predicted waypoints z_k at times t_k (Eq. 1, p.2).
2. **Hierarchical multi-goal A\*** (p.2):
   - For each z_k, define an occlusion-free region Φ_k: an annulus at the desired distance d_d with a clear LOS (Fig. 2, p.3).
   - Greedy successive A* from Φ_k to Φ_{k+1}, with heuristic Eq. 2b.
   - Nodes at the proper distance but occluded are invalid.
3. **Safe flight corridor** (Liu et al. [17], modified, p.2–3):
   - The first polytope is seeded from the initial position along the initial velocity.
   - Subsequent polytopes are built from the guiding path to avoid excessive overlap (Eq. 3).
4. **Visible region per waypoint:** a sector V_k (Eq. 4, p.3).
5. **MINCO trajectory (Eqs. 5–7)** with the problem in Eq. 8:
   - minimum jerk plus ρT;
   - velocity and acceleration limits;
   - stay inside the corridor;
   - p(t_k) ∈ V_k;
   - d_l ≤ ‖p(t_k) − z_k‖ ≤ d_u;
   - T ≥ T_p (p.3).
6. **Constraint handling:**
   - Relative-time integral penalties for feasibility and the corridor (Eqs. 10–11).
   - Absolute-time penalties for visibility and distance (Eqs. 12–18).
   - A time transformation removes T ≥ T_p (Eqs. 19–20).
7. **Yaw:** planned separately, with a limited yaw rate, heading toward the target or toward unknown space (p.5).

### Key equations
| Ref | Equation | Notes |
|---|---|---|
| p.2 Eq.(1) | T = {t_k ∈ [0,T_p] \| t_k → z_k, 0<k≤M_T} | predicted target waypoints |
| p.2 Eq.(2a,b) | f^k(n) = g^k(n)+h^k(n); h^k(n) = √({d^k_xy(n) − d_d}² + {d^k_z(n)}²) | heuristic pulls nodes to horizontal distance d_d and zero vertical offset from z_k |
| p.3 Eq.(3) | P = ∪ P_i, P_i = {x ∈ R³ \| A_i x ≤ b_i} | corridor polytopes |
| p.3 Eq.(4) | V_k = {x ∈ R³ \| ⟨x − z_k, ξ_k⟩ ≤ θ_k} | as printed. ⟨·,·⟩ must denote the *angle* between vectors, consistent with Eq. 16 [UNCERTAIN notation] |
| p.3 Eq.(8a–g) | min ∫‖p^{(3)}‖² dt + ρT, s.t. boundary conditions, ‖p^{(1)}‖≤v_m, ‖p^{(2)}‖≤a_m, p(t)∈P, p(t_k)∈V_k, d_l ≤ ‖p(t_k) − z_k‖ ≤ d_u, T ≥ T_p | — |
| p.4 Eq.(10) | G_h = A_i p_i(t) − b_i ≤ 0; G_v = ‖p^{(1)}‖² − v_m² ≤ 0; G_a = ‖p^{(2)}‖² − a_m² ≤ 0 | — |
| p.4 Eq.(11a) | I^⋆_i = (T_i/κ_i) Σ_j ω̄_j max[G_⋆(c_i,T_i,j/κ_i),0]³ | trapezoid quadrature |
| p.4 Eq.(14) | H_d(x) = (x−d_l)³ for x ≤ d_l; 0 for d_l ≤ x < d_u−ε; (1/ε³)(x−d_u+ε)³(3ε − x + d_u) for d_u−ε ≤ x < d_u+ε; 16(x−d_u) for x > d_u+ε | C² distance penalty. As printed, the first branch (x−d_l)³ is *negative* for x<d_l, whereas Fig. 4 shows a positive cost rising toward 0. The sign is likely (d_l−x)³ [UNCERTAIN sign]. |
| p.4 Eq.(15) | J^d_a = Σ_k H_d(δ^h_k) + max{δ^v_k − δ^v_d, 0}³ | horizontal/vertical split, "since the drone lacks the independent DOF of pitch" |
| p.4 Eq.(16) | G_o(x,k) = cos(θ_k − θ_ε) − ((x − z_k)·ξ_k)/‖x − z_k‖ ≤ 0 | θ_ε = tunable angle clearance |
| p.4 Eq.(17) | J^v_a = Σ_k max[G_o(p(t_k),k),0]³ | — |
| p.5 Eq.(20a–c) | T_Σ = T_p + τ_M²; T_i = e^{τ_i}/(1+Σ e^{τ_j}) T_Σ; T_M = T_Σ − Σ T_i | eliminates T ≥ T_p |

Stated assumptions: the target follows constant velocity in the EKF (p.5), and the prediction is deterministic. Visibility is a deterministic sector constraint. There is no probabilistic treatment.

### Online inputs vs ground truth
- **Simulation:** "its position is broadcasted to each chaser" (p.6), i.e. the ground-truth target position. Three chasers chase the same target simultaneously.
- **Real world:** OpenPose detection plus EKF. Target points are removed from the map manually ("the surrounding point clouds of the target are removed manually", p.5). Mapping and localisation are online.

### Decision logic
- There are no modes and no reacquisition logic.
- The yaw rule is a heuristic: face the target, except head toward unknown space if the planned trajectory enters unknown area (p.5).
- The time slack T ≥ T_p allows the drone to "lag" when the target is faster (p.3).

### Timing / compute
- **Simulation (Table I, p.6), Intel i7-6700:**
  - At target v_max = 1 m/s: total 3.184 ms (path 0.856, corridor 0.538, optimise 1.79).
  - At 2 m/s: 5.2 ms total.
  - Baselines: Han (Fast-Tracker) 9.94 / 15.56 ms; Wang (visPlanner) 12.56 / 12.67 ms, including 6.43 ms of ESDF.
- **Real world:** Jetson Xavier NX for detection; Manifold 2-C for "the other computation tasks" (p.5). No onboard timings are given.

### Experiments
- **Real world (p.5):** the target approaches the drone suddenly, and the drone keeps distance "like an invisible spring" (Fig. 6). The target walks a figure-eight around two cabinets, and the drone keeps it in the FOV (Fig. 7).
- **Simulation (p.5–6):**
  - An aggressive target path in a pillar environment (Fig. 8a); target max speed 2 m/s; chasers 3 m/s and 6 m/s².
  - Metrics: projected target velocity on the image (blur proxy); failure time split into out-of-FOV, too-near (<1 m) and occluded; target-relative position heatmap.
  - Qualitatively, the lowest failure time of the three, and a more compact heatmap (Figs. 8–9).

### Limitations
- **Stated (p.6):** future work is to "estimate the target's intention and improve the motion prediction" and to handle "more extreme scenarios like tracking escaping target".
- **Observed:**
  - The visible region is a sector with a single seed. Eva-Tracker notes this overlooks 3D occlusion (paper 2, p.2, p.6, Fig. 7).
  - No uncertainty is modelled.
  - No reacquisition is attempted.
  - The simulation uses the broadcast ground-truth target.
  - Target point removal is manual in the real world.

### Code
https://github.com/ZJU-FAST-Lab/Elastic-Tracker (p.1 footnote 1).

### MOTAR relevance
**Reusable:**
1. **Φ_k as a candidate region:** an annulus at the desired distance with LOS clearance (p.2–3). This is a clean template for generating short-horizon candidates around the predicted target.
2. **The C² asymmetric distance penalty H_d (Eq. 14, Fig. 4):** steep when too close, linear when too far. This suits a capture-progress or safety-distance term. For MOTAR "capture", the far-side term would be replaced by progress.
3. **Horizontal/vertical distance split (Eq. 15).** This makes sense for vertical bars, where occlusion is horizontal-plane dominated.
4. With vertical bars, the 2D sector visibility model is largely adequate. Eva-Tracker's 3D criticism matters mostly for overhangs and tunnels, which MOTAR's bar world lacks.

**Conflicts:**
- Optimisation-based (MINCO), not policy-based.
- Deterministic CV prediction.
- Tracking (keep distance), not capture/approach.

### BOIA novelty-overlap check
| BOIA element | Elastic Tracker? | Evidence |
|---|---|---|
| (a) belief-conditioned detection-probability forecast | **No** | Deterministic sector visibility (Eq. 4, p.3) on deterministic predicted waypoints (p.5). |
| (b) finite candidates + mixed cost incl. loss-prob and covariance | **No / marginal** | A graph search (A*) through Φ_k (p.2) gives a topology guide, then continuous optimisation (Eq. 8, p.3). There is no loss probability or covariance term. |
| (c) RL-policy arbitration | **No** | — |
| (d) safe fallback + terminal-first | **No** (corridor hard constraint only) | Corridor constraint Eq. 8d (p.3); there is no fallback mode. |

---

## 4. Visibility-aware Trajectory Optimization with Application to Aerial Tracking

- **Authors:** Qianhao Wang, Yuman Gao, Jialin Ji, Chao Xu, Fei Gao (Zhejiang University). p.1
- **Venue/year as printed:** arXiv:2103.06742v1 [cs.RO], 11 Mar 2021 (side stamp p.1). No venue is printed in this PDF. Eva-Tracker (paper 2, p.8 ref [9]) cites it as IROS 2021, pp. 5249–5256.
- **File:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/VisAwareTrajOpt__2103.06742v1.pdf
- **Identity status:** identical (per brief).
- **Pages read:** 1, 2, 3, 4, 5, 6, 7, 8 (all 8; p.8 is a figure plus references).
- **Read status:** FULL_READ.

### Problem
A general, differentiable visibility cost that jointly optimises a quadrotor's position and yaw trajectory. It accounts for:
- Distance of Observation (DO);
- Angle of Observation (AO);
- Occlusion Effect (OE).

It is applied as the back-end of a tracking system (Fast-Tracker). p.1–2

### Assumptions (stated explicitly, p.3 Sec. III-A)
1. "Only one target can be chosen at the same time."
2. "The target is static or has smooth and bounded velocity and acceleration."
3. "The sensor equipped directly in front of robot has conical limited field of view."

Further assumptions:
- **3D environment, unknown map built in real time.** "The quadrotor performs real-time mapping since the environment is unknown." (p.6)
- **Obstacles:** an ESDF is needed for OE and collision, so ESDF updates are required.
- **Target prediction:** "generated by the same method based on Bézier regression in our previous work [14]" (p.4). The horizon is not specified in this paper.

### Method pipeline
1. **Front end:** a kinodynamic hybrid-A* search with motion primitives [15]. It adds an occlusion-feasibility check: an expanded node is rejected if its connection line to the corresponding predicted target node is occluded (Fig. 6, p.5).
2. **Back end:** a uniform cubic B-spline in R³×SO(2) (control points Q_i, Φ_i) (p.3).
3. **Weighted sum cost (Eq. 8), solved unconstrained, with:**
   - J_DO, J_AO, J_OE;
   - feasibility J_f, J_fΦ;
   - smoothness J_s, J_sΦ;
   - collision J_c;
   - plus the "safe tracking" cost J_v (Eq. 22), which keeps the velocity direction inside the FOV so that the environment ahead is known (p.5–6).
4. **OE (p.3, Fig. 4):** the "confident FOV" cone from robot p to target c is approximated by M balls B_i, with centres on the segment and radii growing linearly. Each ball must be obstacle-free, meaning its radius is less than the ESDF at its centre (Eqs. 3–5).
5. The optimiser is not named in the text.

### Key equations
| Ref | Equation | Notes |
|---|---|---|
| p.3 Eq.(1) | od_min < ‖p − c‖ < od_max | DO; p = robot position, c = target |
| p.3 Eq.(2) | ψ_best = atan2(e_yᵀ(p − c), e_xᵀ(p − c)) | AO, as printed (p − c) [UNCERTAIN: facing the target would usually use c − p; transcribed as printed] |
| p.3 Eq.(3)–(4) | c_i = p + λ_i(c − p); r_i = ρ·λ_i·‖p − c‖, λ_i = i/M | balls along the LOS; ρ sets the confident-FOV size |
| p.3 Eq.(5) | r_i < Ξ(c_i) | Ξ = ESDF (distance to the closest obstacle) |
| p.3 Eq.(6)–(7) | p_k = (Q_k + 4Q_{k+1} + Q_{k+2})/6; d_k = ‖p_k − c_k‖ | B-spline waypoint from control points |
| p.4 Eq.(8) | min J = [J_DO, J_AO, J_OE, J_f, J_fΦ, J_s, J_sΦ, J_c]·ω | g(x) = max{0,x}³ (C² penalty) |
| p.4 Eq.(9) | J_DO = Σ g(od_min² − d_k²) + g(d_k² − od_max²) | — |
| p.4 Eq.(10) | J_AO = Σ (ψ_{p,k} − ψ_best,k)² | the gradient flows to both yaw and position (Eqs. 11–13) |
| p.4 Eq.(14)–(15) | J_OE = Σ_k Σ_{i=1}^{M} g(F_OE(p_k)); F_OE = r²_{k,i} − Ξ²(c_{k,i}) | — |
| p.4 Eq.(16) | ∂F_OE/∂p_k = 2[ρ r_{k,i} λ_i ∂d_k/∂p_k − (1−λ_i) Ξ(c_{k,i}) ∂Ξ(c_{k,i})/∂c_{k,i}] | "λ_i acts as a lever" |
| p.4 Eq.(18)–(19) | J_f = Σ g(‖V_i‖² − v_m²) + Σ g(‖A_i‖² − a_m²); analogous for yaw | — |
| p.4 Eq.(21) | J_c = Σ g(d²_thr − Ξ²(Q_i))(Ξ(Q_i)) | as printed; the trailing factor (Ξ(Q_i)) looks odd [UNCERTAIN] |
| p.5 Eq.(22) | J_v = Σ g((ψ_{v,k} − ψ_{p,k})² − ψ²_thr) | keeps the velocity direction within the FOV half-angle |

Stated assumptions: the target is smooth with bounded velocity and acceleration; the FOV is conical. There is no probabilistic model. The OE ball approximation is conservative by design.

### Online inputs vs ground truth
- **Real world:** "The target is an autonomous quadrotor as well broadcasting its location to avoid the impact of the identification" (p.6). The target position is effectively ground truth.
  - The tracker is localised by VINS-Mono [17].
  - Sensors: RealSense D435, FOV 86°×57°; onboard DJI Manifold 2C (p.6).
- **Simulation:** the target is lost when occlusion occurs or it is out of the FOV (p.6). The text implies the same target-prediction method for both planners.
- **General test (p.7):** the failure definition is losing the target. The authors note that "we may be incapable to obtain the target location directly, which requires the quadrotor to identify the target's location".

### Decision logic
- None beyond front-end/back-end replanning.
- There is no reacquisition behaviour.
- **Failure:** target lost, i.e. occluded or out of FOV (p.6–7).

### Timing / compute
- No runtime is reported for the proposed method. The only timing statement is about prior work: a graph-search method "takes over 200 ms [14]" (p.3).
- Elastic Tracker (paper 3, p.6 Table I) later measured this method ("Wang") at ≈12.6 ms on an i7-6700, including 6.43 ms of ESDF. That is a third-party number.

### Experiments
- **Real world (p.6, Fig. 5):** the target swerves behind an obstacle; the tracker keeps the confident FOV obstacle-free.
- **Simulation vs Fast-Tracker [14]:** FOV 80°×65°, same target prediction.
  - **Case 1 (OE, Fig. 7):** sharp turns behind obstacles. Fast-Tracker gets occluded or loses the target; the proposed method avoids it.
  - **Case 2 (AO/DO, Figs. 8–10):** turns of more than 90°. Tracking angle error ψ_err stays below the FOV boundary for the proposed method; Fast-Tracker exceeds it.
  - **General test (Fig. 11–12, Table III, p.7):** random pillar maps, 100 s missions, 10 missions per V_max. Metric: time to failure.
- Parameters: od_min = 2.5 m, od_max = 3.5 m, ρ = 0.8, M = 10 (Table I, p.5).

### Limitations
- **Stated (p.7–8):** "we will continue to improve the prediction method to make it a more robust system".
- **Observed:**
  - Requires an ESDF with frequent updates. Eva-Tracker (p.1) and Elastic Tracker (p.2) criticise this.
  - Weighted-sum trade-offs.
  - Deterministic target prediction.
  - No reacquisition.
  - No compute numbers in the paper.
  - The target position is broadcast in the real world.

### Code
https://github.com/ZJU-FAST-Lab/visPlanner (p.2 footnote 1). Video: https://www.youtube.com/watch?v=PhhrOBx54YY (p.6 footnote 2).

### MOTAR relevance
**Reusable:**
1. **Balls-along-LOS (Eqs. 3–5).** This is a simple, conservative LOS-clearance score: min_i (Ξ(c_i) − r_i). It needs only distance-to-nearest-bar.
   - For vertical bars, Ξ(c) in 2D is min over bars of (‖c_xy − bar_xy‖ − radius), which is trivial to batch on GPU.
   - It also gives a *margin*, which can feed a soft or probabilistic LOS term.
2. **The DO / AO / OE decomposition** is a clean checklist of visibility factors for candidate scoring.
3. **J_v (Eq. 22).** Velocity direction inside the FOV, so the robot flies into observed space. This is relevant because MOTAR's forward camera is also the obstacle sensor.

**Conflicts:**
- ESDF.
- Optimisation-based.
- Deterministic prediction.
- Broadcast target position.

### BOIA novelty-overlap check
| BOIA element | VisAware? | Evidence |
|---|---|---|
| (a) belief-conditioned detection-probability forecast | **No** | Deterministic DO/AO/OE metric (p.3). |
| (b) finite candidates + loss-prob/covariance cost | **No** | Hybrid-A* front end with a binary occlusion check (p.5); B-spline optimisation back end (p.4). |
| (c) RL-policy arbitration | **No** | — |
| (d) safe fallback + terminal-first | **No** | Collision penalty only (Eq. 21, p.4); J_v keeps motion in known space (p.5), a weak form of safety-aware sensing. |

---

## 5. Star-Convex Constrained Optimization for Visibility Planning (with Application to Aerial Inspection)

- **Title as printed in this file:** "Star-Convex Constrained Optimization for Visibility Planning with Application to Aerial Inspection" (p.1).
- **Authors:** Tianyu Liu, Qianhao Wang, Xingguang Zhong, Zhepei Wang, Chao Xu, Fu Zhang, Fei Gao (Zhejiang University / HKU). p.1
- **Venue/year as printed:** arXiv:2204.04393v1 [cs.RO], 9 Apr 2022 (side stamp p.1). No venue is printed.
- **File:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/StarConvex__2204.04393v1.pdf
- **Identity status: DIFFERENT version from the user's copy** (per brief). This is arXiv **v1**. I had no access to the user's copy, so I cannot list the differences. Equation numbers, page numbers and any numbers below refer to v1 only. They should be re-checked against the user's version before citing.
- **Pages read:** 1, 2, 3, 4, 5, 6, 7 (all 7; p.7 is references).
- **Read status:** FULL_READ (of v1).

### Problem
Visibility-*guaranteed* trajectory planning, applied to aerial inspection.
- Visit a list of task representative points (TRPs) c_i ∈ R³.
- Each must be inspected (visible) for at least τ_i seconds.
- The trajectory must be safe, dynamically feasible and energy-efficient.

Visibility is a *hard* constraint, via the star-convex polytope (SCP) of the region visible from each TRP. p.1–2

### Assumptions (p.2 Sec. III)
1. "The sensor mounted on the robot has omnidirectional coverage." The real-world sensor is an Insta360 One X2 (p.6).
2. Visibility counts only when the whole robot body is inside a ball-shaped "sensible region" around the point.

Further assumptions:
- **Visibility definition:** c_i is visible from x iff the segment from x to c_i is collision-free (p.2).
- **Map:** a point-cloud map M_g in a k-d tree.
  - In the real world it is "pre-built using lidar by LIO-SAM [23] and the trajectory is planned offline" (p.6). So the map is known and static.
  - The map is inflated for the configuration space ("one-to-eight cubic inflation", p.4).
- **Targets:** static points. There is no target motion and no prediction.
- **3D.**

### Method pipeline
1. **SCP construction per TRP (p.2–3, Fig. 2):**
   - Range-query points within radius R of c_i, and add augmented points evenly on the sphere of radius R.
   - Spherical flipping (Eq. 1, flipping radius r) maps points outward.
   - Take the convex hull (quickhull [20]).
   - Points on the hull are images of visible points; inverting the hull gives the star-shaped visible region S_i.
   - **Point-in-SCP test:** flip the point and check that it lies outside the convex hull.
2. **Visibility constraint:** x̂ = F(x) must lie outside the flipped convex polytope P_i (H-representation, Eq. 3), i.e. Ξ(x̂) > d_min (Eqs. 4–5).
   - The max over faces is smoothed with log-sum-exp (Eq. 6) and relaxed into a C² penalty (Eqs. 7–9).
   - Analytic gradient: Eq. 10.
3. **Route (p.4):**
   - ATSP via Lin–Kernighan (LKH) for the visiting order.
   - Waypoint refinement on SCPs: an unconstrained NLP with a visibility penalty (Eqs. 11–13).
4. **Kinodynamic A\*** on the voxelised map between route waypoints (p.4).
5. **Safe and visible corridor (SVC):** SCPs connected by overlapping convex polytopes generated from the path [17] (p.4).
6. **Trajectory:** MINCO [5], spatio-temporal (Eq. 14).
   - The time-integral visibility penalty (Eq. 15) keeps σ^i(t) inside S_i.
   - T_i = e^{ξ_i} + τ_i (Eq. 16) enforces the minimum inspection time (p.4–5).

### Key equations
| Ref | Equation | Notes |
|---|---|---|
| p.2 Eq.(1) | x̂ = F(x) = x − c_i + 2(r − ‖x − c_i‖)(x − c_i)/‖x − c_i‖ | spherical ("ball") flipping, radius r (as printed; the result is expressed relative to c_i) |
| p.3 Eq.(2) | min_x J(x) s.t. x ∈ S_i | star-convex constrained optimisation |
| p.3 Eq.(3) | P_i = {x ∈ R³ \| A x ⪯ b}, A = [n_1..n_K]ᵀ, b = [n_iᵀ a_i] | flipped convex polytope |
| p.3 Eq.(4)–(5) | Ξ(x̂) > d_min; Ξ(x̂) = max_i {d_i = n_iᵀ(x̂ − a_i)} | signed distance to the hull, positive outside |
| p.3 Eq.(6) | Ξ ≈ LSE(d_1..d_K) = (1/α) log(Σ e^{α d_i}) | → max as α → ∞; α = 100 was used (p.5) |
| p.3 Eq.(7)–(9) | min J(x) + V(LSÊ); V = λ max{LSÊ, 0}³; LSÊ = d_min − (1/α) log Σ e^{α d_i} | λ = very large penalty weight |
| p.3 Eq.(10) | g_scp = ∂V/∂x = 6λ LSÊ² · (Σ e^{αd_i} n_i / Σ e^{αd_i}) · (r/‖x‖³)(‖x‖² − x xᵀ − ‖x‖³/(2r)) | [UNCERTAIN] Multi-line typeset formula; the pdftotext extraction is garbled. The grouping and signs are as best read from the rendered p.3. |
| p.4 Eq.(11) | min Σ‖w_i − w_{i−1}‖ (+ start/end) s.t. w_i ∈ S_i | route waypoint refinement |
| p.4 Eq.(12) | J_w = Σ √(‖w_i − w_{i−1}‖² + ε) + Σ λ_i V(LSÊ(w_i\|S_i)) | — |
| p.4 Eq.(14a–d) | min ∫‖σ^{(3)}‖² + ρT_Σ, s.t. boundary conditions, σ(t) ∈ F (SVC), v/a limits, σ^i(t) ∈ S_i with T_i > τ_i | — |
| p.5 Eq.(15) | I(S_i, η_i) = (T_i/η_i) Σ_{j=0}^{η_i} V(LSÊ(σ(j T_i/η_i) \| S_i)) | time-integral visibility penalty |
| p.5 Eq.(16) | T_i = e^{ξ_i} + τ_i | enforces T_i > τ_i |

Stated assumptions: omnidirectional sensing; a static, known point-cloud map; LSE as a smooth approximation of max (p.3).

### Online inputs vs ground truth
- Everything is known a priori: the pre-built LIO-SAM map, and TRPs given in advance (p.6).
- The real-world state estimate uses a RealSense D435 (p.6).
- The plan is computed offline (p.6).

### Decision logic
- None at runtime. The task order comes from ATSP; minimum dwell time comes from τ_i.
- This is a one-shot plan, not a feedback planner.

### Timing / compute
- **Simulation hardware:** 2.6 GHz Intel i7-9750H (p.5).
- **Large scene (40×150 m, 150 pillars, 60 rings, 40 spots):** "The trajectory is generated in 4.4s" (Fig. 4 caption, p.5).
- **Scaling (Fig. 6, p.6):** "faster than Zhou's by orders of magnitude"; "can still finish in seconds" as scale increases. About 42% of the total time goes to SCP, route and SVC generation (p.6).
- The authors say the computation time "shows it is also adequate for replanning" (p.5).

### Experiments
- **Simulation (p.5–6):** small, medium and large random pillar/ring scenes; v_max 4 m/s, a_max 6 m/s².
  - Baseline: a modified FUEL (Zhou et al. [8]).
  - Metrics: trajectory duration, Int(J²), "visible capability" (fraction of spots observed) (Table I, p.5).
  - Qualitatively, the proposed method guarantees all spots are seen, with smoother trajectories and slightly longer duration. The baseline misses spots (Fig. 5).
- **Real world (p.6, Fig. 7):** indoor cylinders as inspection targets; 1.5 m/s, 1.0 m/s², minimum inspection 1.0 s. The drone slows and inspects through gaps.

### Limitations
- **Stated (p.6):** "The main limitation of our method is the omnidirectional perception assumption of the sensor model". Future work adds limited FOV and yaw planning.
- **Observed:**
  - Static TRPs.
  - Offline planning on a known map.
  - Binary deterministic visibility.
  - The SCP is built from the point cloud around the *target point*, which presumes the map near the target is known.

### Code
None printed in the PDF (v1). The only URL is the Insta360 footnote (p.6).

### MOTAR relevance
**Reusable:**
1. **SCP via spherical flipping (hidden-point-removal style; Eq. 1 plus the convex hull).** This builds the star-shaped set of positions from which the predicted target is visible, directly from the local LiDAR point cloud.
   - A Point-In-SCP test (flip, then a half-space check) is O(K) per query.
   - This is attractive for vectorised LOS checks of many candidate drone positions against one predicted target position.
   - For MOTAR's bars, a direct analytic 2D segment–circle test is likely simpler and exact. SCP becomes valuable if the occluders are arbitrary point clouds.
2. **The LSE-smoothed signed distance to the visible region (Eqs. 6–9).** It gives a differentiable *margin* that could be squashed into a soft LOS probability.

**Conflicts:**
- Omnidirectional sensor, while MOTAR has a forward camera.
- Static targets.
- Offline, with a known map.

### BOIA novelty-overlap check
| BOIA element | Star-Convex? | Evidence |
|---|---|---|
| (a) belief-conditioned detection-probability forecast | **No** | Deterministic geometric visibility (p.2–3); omnidirectional; static targets. |
| (b) finite candidates + loss-prob/covariance cost | **No** | Continuous MINCO optimisation within the SVC (p.4–5). |
| (c) RL-policy arbitration | **No** | — |
| (d) safe fallback + terminal-first | **No** | Safety comes only from the corridor (p.4). |

---

## Cross-cutting observations for MOTAR / BOIA

1. **(a) is the only BOIA element with substantive prior art in this set.** BPOD (Eq. 9, Eq. 15, Eq. 19) already forecasts a belief-conditioned detection probability under FOV and LOS, propagated over a horizon, and folds it into an expected-covariance update (Eq. 11d). A BOIA claim on (a) must therefore be positioned as an *extension*:
   - adding detector reliability, which BPOD explicitly assumes away (footnote, p.2);
   - a 3D/forward-camera setting with online LiDAR occluders rather than a known 2D map;
   - correlation-aware LOS for dense bars, rather than BPOD's independence product (Eq. 15).
2. **(b), (c) and (d) do not appear in any of the five papers.**
   - All five are continuous optimisers: SCP/MPC (BPOD) or MINCO/B-spline (the FAST-Lab family and Eva-Tracker).
   - None arbitrates over a learned policy.
   - None has a calibrated-advantage gate, hysteresis, an explicit fallback mode or a terminal/capture phase.
   - All four tracking papers are *keep-distance tracking* tasks, not interception or capture. Star-Convex is inspection.
3. **Reacquisition is weak across the set.**
   - Only Eva-Tracker addresses it, by following a deterministic 1 s extrapolation; it is stated to work only for short losses (p.8).
   - BPOD handles loss implicitly through covariance growth.
   - Elastic, VisAware and Star-Convex do not address it.
4. **Ground-truth leakage in evaluation.**
   - The simulations of Elastic (p.6) and Eva-Tracker (p.5) broadcast the true target position.
   - VisAware's real-world target broadcasts its location (p.6).
   - BPOD's real-world test uses Vicon for ground truth (p.7).
   - MOTAR's evaluation should state explicitly that it uses detector-only target observations, to differentiate.
5. **Dense vertical bars make 2D LOS models (BPOD, Elastic sector) closer to exact.** The bars are effectively circles in the horizontal plane. LOS SDF and probability can be computed analytically, without GJK/EPA, ESDF or SCP.
   - Note: independence across adjacent bars (BPOD Eq. 15) is a modelling choice MOTAR should either validate or replace. For example, compute Pr(LOS clear) by sampling the target belief jointly over all bars.

## Cross-paper table

| Paper | Problem | Sensing | Map assumption | Visibility model | Optimisation / decision | Target prediction | Reacquisition handled? | Runtime (as reported) | Code URL |
|---|---|---|---|---|---|---|---|---|---|
| BPOD (arXiv 2306.06363v2, 2024) | Non-myopic visibility-aware tracking under uncertainty (ground robot) | Limited-FOV range–bearing / camera model (annular sector, 2π/3 in simulation); perfect detector | Known, static, 2D convex polygons | Probabilistic: BPOD = Pr(in FOV)·Π Pr(LOS clear), linearised SDF + erf | MPC via SCP (MOSEK), horizon N=4 × 0.5 s; objective entropy or −ΣBPOD; collision chance constraints | EKF; mean propagated with the last (known or differenced) control | Implicitly (covariance grows when γ low); no mode | ~1 ms per BPOD; 0.176 s/step (V-A), <0.1 s (V-B) in MATLAB on an i7 desktop; ~0.09 s/step real-world laptop (p.7–8) | none printed |
| Eva-Tracker (arXiv 2602.12549v1, 2026) | Visibility-aware aerial tracking without ESDF updates, with short-term reacquisition | Camera (fixed FOV pyramid; 69.4°×42.5° in simulation) + Livox Mid-360 LiDAR | Unknown, online local map; dynamic obstacle points allowed | Geometric: precomputed camera-frame FoV-ESDF; occlusion = obstacle points inside the FOV | Arc-search occlusion-free waypoints + MINCO, L-BFGS | Deterministic Bézier fit to last m+1 obs (m=2, τ=0.5 s) | Yes, implicitly: keeps planning to the predicted target; short-term only (p.8) | 10.75 ms total (simulation, i7-1165G7) (p.6); onboard Orin NX untimed | github.com/Yue-0/Eva-Tracker |
| Elastic Tracker (arXiv 2109.07111v2, 2022) | Flexible ("elastic") aerial tracking with safety and visibility | D435 (localisation/mapping) + mono camera, 85°×72° (real) / 80°×65° (simulation) | Unknown, online map | Geometric sector V_k around each predicted target point + distance band | Multi-goal A* → SFC → MINCO spatio-temporal optimisation with penalties | EKF (constant velocity) + motion primitives (min-acceleration) | No | 3.2–5.2 ms total (simulation, i7-6700) (p.6) | github.com/ZJU-FAST-Lab/Elastic-Tracker |
| VisAware Traj Opt (arXiv 2103.06742v1, 2021) | Differentiable visibility metric (DO/AO/OE) with joint position-yaw optimisation | D435, 86°×57°; conical FOV assumed | Unknown, real-time mapping; ESDF required | Geometric: distance band, yaw-to-target, balls-along-LOS vs ESDF | Occlusion-checked hybrid A* + B-spline weighted-sum optimisation | Bézier regression (from Fast-Tracker [14]) | No | Not reported (third party ≈12.6 ms incl. ESDF, Elastic p.6) | github.com/ZJU-FAST-Lab/visPlanner |
| Star-Convex (arXiv 2204.04393v1, 2022; DIFFERENT version from user's copy) | Visibility-guaranteed multi-point inspection trajectory | Omnidirectional (Insta360) assumed | Known, pre-built static point cloud; offline planning | Hard constraint: star-convex visible polytope via spherical flipping + hull, LSE-relaxed | ATSP (LKH) + waypoint NLP + kinodynamic A* + SVC + MINCO | N/A (static points) | N/A | Seconds for large scenes (4.4 s, 40 spots; i7-9750H) (p.5) | none printed |
