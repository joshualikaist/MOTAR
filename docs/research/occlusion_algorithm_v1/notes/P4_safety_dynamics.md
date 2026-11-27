# P4 — Safety, dynamics, simulator substrate (FASTER, Lee SE(3), Aerial Gym, EGO-Planner, Adaptive Perching)

Reader notes for MOTAR (forward RGB-D + LiDAR quadrotor, Isaac Gym / Aerial Gym, Lee-style controller, frozen RL velocity/yaw-rate policy at 0.1 s decision steps, speed governor, dense random vertical bars) and for the BOIA concept:
(a) belief-conditioned detection-probability forecast; (b) a finite set of dynamically feasible candidates scored by capture progress + loss probability + covariance growth; (c) arbitration over a frozen RL policy with a calibrated threshold and hysteresis; (d) safe fallback + terminal-first handling.

Conventions: "p.N" is the physical 1-based PDF page. "Eq. (k)" uses the paper's own numbering. "[UNCERTAIN]" marks a transcription I am not sure of. "[observed]" marks my own analysis, which the paper does not state. "[code]" marks something I read in the MOTAR worktree source, not in the paper.
Method: I extracted the text of every page with `pdftotext`. Equation-heavy pages were also checked visually with the Read tool. FASTER pp.2–4, Lee pp.2–4, Aerial Gym pp.3,4,6, EGO pp.4–6 and Perching pp.13–17 were checked visually.

---

## 1. FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments

- **Authors / venue / year:** J. Tordesillas, B. T. Lopez, J. P. How. Accepted for IROS 2019 (header, p.1). This file is arXiv:1903.03558v3, dated 13 May 2020.
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/FASTER__1903.03558v3.pdf`
- **Identity:** 7 pp, identical to the user's copy (per caller).
- **Pages read:** 1–7, all text. pp.2–4 checked visually (Alg. 1, MIQP Eq. (1), Fig. 3/4).
- **Read status:** FULL_READ

### Problem
High-speed local planning in unknown environments. Classical safe planners keep the whole local trajectory in known-free space and end it with a stop. This is safe but slow when known-free space F is small (p.1). FASTER plans fast in F ∪ U while always keeping a safe backup trajectory in F (abstract, p.1).

### Space partition and definitions (p.1, p.2)
- R³ = O ∪ F ∪ U, three disjoint sets: O = occupied-known, F = free-known, U = unknown (p.1).
- **Whole Trajectory:** A → E (goal), contained in F ∪ U, with a final stop condition (p.2).
- **Safe Trajectory:** R → F. R is a point on the Whole Trajectory. F is any point inside the polyhedra of a convex decomposition of free-known space, chosen by the optimizer. The trajectory is completely inside free-known space and ends with a stop (p.2).
- **Committed Trajectory:** the piece A→R of the Whole Trajectory followed by the Safe Trajectory. It is guaranteed to be inside free-known space. It is what the UAV executes "in case no feasible solutions are found in the next replanning steps" (p.2).

### Dynamics model and parametrisation (p.2–3)
- Triple integrator. State xᵀ = [xᵀ vᵀ aᵀ], input u = jerk j (p.2).
- Jerk is constant per interval, so the trajectory is a cubic spline. Each interval is rewritten as a cubic Bézier curve with control points r_n0..r_n3 (formulas on p.3).
- **Convex-hull containment:** binary variables b_np. b_np = 1 implies all 4 control points of interval n lie in polyhedron p (A_p r_nj ≤ c_p), and Σ_p b_np ≥ 1 (p.3). The Bézier convex-hull property then gives continuous-time containment in the corridor.

### Key equations
- **Eq. (1), p.3, MIQP:** min Σ_n ‖j_n‖² subject to:
  - boundary conditions x₀(0) = x_init and x_{N−1}(dt) = x_final;
  - cubic/derivative consistency and j_n = 6a_n(0);
  - polyhedron implication constraints;
  - continuity x_{n+1}(0) = x_n(dt);
  - **‖v_n(0)‖∞ ≤ v_max, ‖a_n(0)‖∞ ≤ a_max, ‖j_n‖∞ ≤ j_max**, i.e. per-axis limits imposed at interval starts.
  - Solved with Gurobi.
- **Eq. (2), p.3:** dt = f · max{T_vx, T_vy, T_vz, T_ax, …, T_jz}/N.
  - T_* are the solutions of constant-input per-axis motions at v_max, a_max, j_max.
  - f ≥ 1 is searched in [f_worked,k−1 − γ, f_worked,k−1 + γ′] (Fig. 2).
  - "if f = 1, then dt is a lower bound on the minimum time per interval required for the problem to be feasible" (p.3).
- [observed] Velocity limits are imposed only at interval starts. Acceleration is linear within an interval, so bounding it at the knots also bounds it in between. Velocity is quadratic within an interval, so mid-interval |v| could exceed v_max. The paper does not discuss this.

### Algorithm 1 (p.3; prose p.3–4)
Inputs: L (current position), Committed_{k−1}, JPS_{k−1}, G_term, O, F, U, r.
1. k ← k+1, δt ← αΔt_{k−1}, δt′ ← βΔt_{k−1} (line 2). α ≥ 1, "typically α ≈ 1.25" (p.4). β ≥ 1.
2. Point A on Committed_{k−1} at offset δt from L (line 3).
3. G ← projection of G_term into the sliding map M (line 4).
4. JPS_a: run JPS from A to G (line 5). JPS_b: JPS_{k−1} modified to avoid O, re-running JPS A→I1, I1→I2, I2→I_G (line 7; p.4).
5. Intersections with sphere S of radius r: C and D. Cost-to-go J_a = N·dt_a + ‖JPS_a(C→G)‖/v_max, and likewise J_b. Choose the argmin (lines 8–14).
6. Whole: convex decomposition in U ∪ F around JPS_in, then the MIQP from A to G (lines 15–18).
7. Safe: JPS_in,known (the part in F), convex decomposition in F, then the MIQP from R to any F (lines 19–22). R lies on Whole at offset δt′ from A.
8. Committed_k ← Whole_{A→R} ∪ Safe (line 23). Store the f factors and the replanning time (lines 24–26).

### Exact safety conditions (p.4, "Thus safety is guaranteed by construction")
- **Do not commit** a new trajectory in a replanning step if any of these holds:
  - either MIQP fails;
  - the piece A–R intersects U. They explicitly check this because A and R can both be in F while A–R is not;
  - the replanning step takes longer than δt.
- In that case the UAV "continues executing the Committed Trajectory of the previous replanning step" (p.4).
- "the UAV will only fly Committed Trajectories, which are always guaranteed to be in F with a terminal stopping condition" (p.4).
- **Mapping prerequisite (p.4):** sliding map centred on the UAV. Depth is fused by 3D Bresenham ray-tracing. "O and U are inflated by the radius of the UAV to ensure safety."
- **Stated assumptions:**
  - triple-integrator dynamics (p.2);
  - O and U inflated by the UAV radius (p.4);
  - the δt/δt′ intervals are designed so that "with high probability, at least one replanning step can be solved within that interval" (p.4). This is a probabilistic design choice, not a guarantee.
- **Implicit assumptions [observed]** (needed for the "by construction" argument but not stated):
  - static world: free-known space does not later become occupied (no moving obstacles);
  - perfect tracking of the jerk-limited reference by the low-level controller;
  - accurate state estimate (hardware uses mocap fusion, p.6);
  - an initial committed trajectory exists, e.g. hovering in free-known space.

### Guaranteed vs empirical
- **Guaranteed, by construction:** the executed trajectory always lies in inflated free-known space and ends at rest, given the implicit assumptions above. The convex-hull Bézier property gives continuous-time containment (p.3).
- **Not guaranteed:** replanning finishes within δt (only "high probability", p.4); progress/completeness; mid-interval velocity bound [observed]; robustness to tracking error.
- **Empirical:** all speed, distance and time results.

### Online inputs vs GT
Online inputs are the depth map fused into an occupancy grid (sliding map) and the UAV state. In hardware the state came from a Kalman filter fusing IMU with external motion capture (p.6), so pose is near-GT. The simulation map source is not further specified.

### Control interface
The planner outputs a jerk-controlled triple-integrator trajectory (p.2). Control runs on a Qualcomm SnapDragon Flight (p.6). The controller type is not described.

### Timing / compute
- Parameters: P_max = 2–3; N = 10 (Whole) and N = 7 (Safe) (p.5).
- The Safe MIQP runtime is ~constant in P_max (p.5, Fig. 6).
- 75th percentile of the MIQP runtimes is "always below 32 ms" (p.6, Fig. 12, P_max = 2).
- CPU: Intel i7-7700HQ for simulation; onboard Intel NUC in hardware, also running the mapper and RealSense (p.6).

### Experiments (numbers with context)
- **Forest simulation:** 10 random 50×50 m forests, density 0.1 obstacles/m² (p.5).
  - Limits: per-axis v_max = 5 m/s, a_max = 5 m/s², j_max = 8 m/s³ (p.5).
  - Table I (p.5): FASTER 10/10 successes; distance avg 77.6 m, std 5.9, max 88.0, min 70.7. Improvement over the other methods: 8–51% min/max (p.5) [UNCERTAIN pairing of the improvement columns from the extracted table].
  - Table II (p.5): flight time FASTER avg 29.2 s vs Mult-Fid 61.2 s; "improvement of 52%" (text p.5).
- **Bugtrap simulation:** limits v_max = 10 m/s, a_max = 10 m/s², j_max = 40 m/s³ (p.5).
  - Table III: distance 55.2 vs 56.8 m; time 13.8 vs 37.6 s, i.e. 63.3% improvement (p.5).
- **Hardware:** max speed 2.1 m/s in experiments 1–2 and 3.6 m/s in experiments 3–4 (p.6). RealSense D435 (Fig. 7, p.5).

### Limitations
- **Stated:** effectively none beyond the probabilistic interval design (p.4).
- **Observed:**
  - static-world assumption;
  - requires an occupancy map with explicit unknown space;
  - MIQP with Gurobi;
  - safety is in the planner's model only, with no tracking-error margin beyond the radius inflation;
  - the mid-interval v bound noted above;
  - the guarantee is collision-avoidance with stop, not progress.

### Code
https://github.com/mit-acl/faster; worlds at https://github.com/jtorde; video https://youtu.be/gwV0YRs5IWs (p.1).

### MOTAR relevance / conflicts
- The core idea transfers directly to the **BOIA fallback (d)**. At each 0.1 s decision, MOTAR must hold a braking ("committed") trajectory whose swept volume lies in observed-free space. A new policy action is executed only if a new committed braking plan from the post-action state validates. Otherwise MOTAR continues the previous committed brake.
- **Conflict 1: no persistent map.** MOTAR's RL policy is reactive with no explicit F/U map. Free-known space must be defined from the current depth/LiDAR frustum. A thin vertical bar hidden behind a nearer bar is in U.
- **Conflict 2: forward-only camera.** Backup trajectories must stay inside the observed frustum. Braking straight ahead is the natural choice. LiDAR coverage can enlarge free-known space.
- **Conflict 3: tracking error.** FASTER assumes perfect tracking. MOTAR's Lee controller plus tilt limit has lag, so the inflation must include the tracking-error envelope, measured empirically.
- **Conflict 4: dynamic targets / moving objects** break "F stays free".

### BOIA novelty overlap
- **(a) No.** No detection/visibility model at all.
- **(b) Partly.**
  - Evidence: FASTER compares exactly two guidance directions (JPS_a vs JPS_b) by a dynamics-aware cost-to-go, Alg. 1 lines 6–14 (p.3–4).
  - Dynamic feasibility is via the v/a/j limits in Eq. (1) (p.3).
  - It does not use a finite scored candidate set over capture/loss/covariance.
- **(c) No.** It does arbitrate between "commit new" and "keep previous committed" by hard feasibility/timing tests (p.4). There is no learned policy, no calibrated threshold, no hysteresis.
- **(d) Partly to strong** for safe fallback: the committed backup trajectory in F with a terminal stop is the canonical precedent (p.2, p.4). It has no terminal-first / capture-terminal handling.

#### 2026-10-19 original-copy re-read (Codex)

- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/FASTER Fast and Safe Trajectory Planner for Flights.pdf`; SHA-256 `f31547f387a29588`; 7 physical PDF pages. Previous-public-copy SHA is **identical**.
- **Read coverage:** physical pp.1,2,3,4; **FULL_READ** (retained earlier FULL_READ of identical bytes; both numbered equations and the cubic-Bezier identity checked (only two numbered equations exist)). No source code was executed and no result was reproduced.

- **Equation sampling exception:** the paper has only two numbered equations: MIQP Eq.(1) and time-allocation Eq.(2), both p.3. Both were checked, plus the unnumbered cubic-to-Bezier conversion and Alg.1 lines18–23. A third numbered equation cannot be sampled; this is reported rather than fabricated.
- **Metadata/model:** Jesus Tordesillas, Brett T. Lopez, Jonathan P. How; accepted IROS2019 (p.1), arXiv:1903.03558v3 stamp13May2020. Triple integrator/constant jerk per interval, depth-derived occupancy with known-free F, occupied O, unknown U (pp.2–4). The model is a planning abstraction, not rotor-level dynamics.
- **Mandatory safety answer — confirmed, conditional:** p.4 permits commit only if both whole and safe optimizations succeed, the complete A-to-R piece does not intersect U, and replanning finishes within delta_t. Otherwise it continues the previous committed trajectory. The committed prefix plus terminal-stopping safe tail lies wholly inside F; p.4 inflates O and U by robot radius. Accurate free-space mapping, collision inflation, existence of the initial committed backup, static/persistent free space and sufficiently accurate tracking/state/timing are assumptions needed to carry this planned-trajectory argument to the physical robot. Perfect tracking/static obstacles are not a robust tracking-error theorem proven in the paper. The 'high probability' statement applies to timely replanning opportunities, not to a quantified collision probability for real flight.
- **Mandatory derivative-bound answer — confirmed:** p.3 Eq.(1) constrains `||v_n(0)||_inf<=vmax` and `||a_n(0)||_inf<=amax` at each interval start; jerk is constant and bounded per interval. A quadratic velocity can overshoot between starts. The Bezier convex-hull containment certifies geometric containment, not these continuous derivative bounds by itself. BOIA must inspect all rollout substeps/swept segments rather than infer them from FASTER's knots.
- **Online inputs/logic/experiments:** online state/depth occupancy and goal; no target, detector, trained policy or training GT. Whole trajectory may explore F union U; only committed trajectory is flown in F. Existing hardware/timing and flight benchmarks in the earlier identical FULL_READ remain valid with their recorded context, not as bounds for MOTAR. Code URL in historical Code subsection is retained; no learned weights. No experiment in this pass was run.
- **BOIA overlap:** (a) **no**; (b) **partial** (candidate direction cost-to-go, continuous MIQP rather than finite loss/covariance scorer); (c) **no** (commit/keep-previous is not learned-policy advantage arbitration); (d) **partial/strong for retained safe backup**, not simulator terminal-first or unconditional safe-hold; (e) **no** (flight collision/success rather than post-occlusion rendezvous protocol). A newly generated zero command is not automatically FASTER's previously certified brake.


---

## 2. Geometric Tracking Control of a Quadrotor UAV on SE(3), Lee, Leok, McClamroch

- **Venue / year:** arXiv:1003.2005v1 [math.OC], 10 Mar 2010 (p.1). The published version is CDC 2010.
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/LeeSE3__1003.2005v1.pdf`
- **Identity:** **this is the arXiv v1, 8 pp. It is DIFFERENT from the user's 6-page CDC copy.**
  - Equation numbers, gain conditions and the proof appendix (pp.5–8 here) may be numbered or condensed differently in the CDC version. I did not open the user copy.
  - Aerial Gym cites a different Lee paper: [34] "Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)", 2011 (Aerial Gym p.8).
- **Pages read:** 1–8 (all, including the appendix proofs). pp.2–4 checked visually.
- **Read status:** FULL_READ

### Problem
Tracking of position x_d(t) and heading direction b⃗_1d(t) for an underactuated quadrotor (4 thrust inputs → 6 DOF). The controller is defined globally on SE(3), avoiding Euler and quaternion singularities and ambiguities (p.1–2).

### Model, signs and frames (p.1–2)
- **Frames:** inertial {e⃗1, e⃗2, e⃗3}; body {b⃗1, b⃗2, b⃗3}, origin at the CoM. R ∈ SO(3) maps body → inertial. Ω is the body angular velocity. x and v are inertial.
- **Thrust direction:**
  - f_i is "the thrust generated by the i-th propeller along the −b⃗3 axis" (p.2).
  - The text also says total thrust "acts along the direction of Re3" (p.2). The dynamics, Eq. (3), have the force −fRe3. So the physical thrust vector is **−fRe3**: it points along −b3, upward when b3 points down.
  - [observed] The prose is loose, but Eq. (3) is authoritative.
- **Gravity:** Eq. (3) has +mg e3 with e3 = [0;0;1]. So **e3 points DOWN (NED-like)**, and gravity acts along +e3. Fig. 1 draws e3 downward.
- **Rotor torque:** τ_i = (−1)^i c_τf f_i. Propellers 1 and 3 rotate CCW, 2 and 4 CW (p.2).
- **Eq. (1), mixer:** [f; M1; M2; M3] = [[1,1,1,1],[0,−d,0,d],[d,0,−d,0],[−c_τf, c_τf, −c_τf, c_τf]]·[f1..f4].
  - Determinant 8c_τf d², invertible if d ≠ 0 and c_τf ≠ 0 (p.2).
  - Rotor dynamics are ignored: "the thrust of each propeller is directly controlled" (p.2).
- **Eqs. (2)–(5), EOM (p.2):**
  - ẋ = v
  - m v̇ = m g e3 − f R e3
  - Ṙ = R Ω̂
  - J Ω̇ + Ω × JΩ = M

### Tracking errors (p.2–3)
- e_x = x − x_d, Eq. (6). e_v = v − v_d, Eq. (7).
- Ψ(R, R_d) = ½ tr[I − R_dᵀR], Eq. (8). It is locally positive-definite where the rotation angle is < 180°. Sublevel set L2 = {Ψ < 2} (p.3).
- e_R = ½ (R_dᵀR − RᵀR_d)^∨, Eq. (10).
- e_Ω = Ω − RᵀR_dΩ_d, Eq. (11).

### Control law (p.3)
- f = −(−k_x e_x − k_v e_v − m g e3 + m ẍ_d) · R e3, Eq. (12).
- M = −k_R e_R − k_Ω e_Ω + Ω × JΩ − J(Ω̂ RᵀR_d Ω_d − RᵀR_d Ω̇_d), Eq. (13).
- b⃗_3d = R_d e3 = −(−k_x e_x − k_v e_v − m g e3 + m ẍ_d)/‖·‖, Eq. (14).
- R_d = [b⃗_1d; b⃗_3d × b⃗_1d; b⃗_3d] (p.3).
  - [observed] In v1 this lies in SO(3) only if b⃗_1d ⟂ b⃗_3d, which is not stated or enforced in v1.
  - Aerial Gym and the MOTAR code [code] orthonormalise instead: b2 = b3 × χ/‖·‖, b1 = b2 × b3.
- **Assumption Eq. (15):** ‖−k_x e_x − k_v e_v − m g e3 + m ẍ_d‖ ≠ 0, i.e. the commanded force is never zero.
- **Assumption Eq. (16):** ‖−m g e3 + m ẍ_d‖ < B, i.e. desired accelerations are bounded.
- **Design rationale (p.3):** "f is reduced when the angle between [b3 and b3d] becomes larger". The projection onto Re3 is essential to the stability proof.

### Stability claims and assumptions
- **Prop. 1 (p.3): attitude only.** Holds for any k_R, k_Ω > 0, under:
  - Ψ(R(0), R_d(0)) < 2, Eq. (17);
  - ‖e_Ω(0)‖² < (2/λ_min(J)) k_R (2 − Ψ(0)), Eq. (18). [CORRECTED: original CDC p.4 Eqs.(18),(27),(29) use lambda_max(J); old arXiv transcription must not supply a runtime stability bound.]
  - Then (e_R, e_Ω) = 0 is exponentially stable and Ψ(t) ≤ min{2, α₂e^{−β₂t}}, Eq. (19).
  - Interpretation: initial attitude error < 180°. A larger k_R enlarges the angular-velocity region (p.3).
- **Prop. 2 (p.4): complete dynamics, exponential stability.** Conditions:
  - Ψ(R(0), R_d(0)) ≤ ψ₁ < 1, Eq. (20), i.e. initial attitude error **< 90°**.
  - W1, W12, W2 as in Eqs. (21)–(23), with α = √(ψ₁(2−ψ₁)) and e_vmax = max{‖e_v(0)‖, B/(k_v(1−α))}.
  - Gain conditions Eq. (24) on c1; Eq. (25) on c2; Eq. (26) λ_min(W2) > 4‖W12‖²/λ_min(W1).
  - Region of attraction: (20) plus ‖e_Ω(0)‖² < (2/λ_min(J)) k_R (1 − Ψ(0)), Eq. (27). [CORRECTED: original CDC p.4 Eqs.(18),(27),(29) use lambda_max(J); old arXiv transcription must not supply a runtime stability bound.]
- **Prop. 3 (p.4): almost-global exponential attractiveness.** Conditions 1 ≤ Ψ(0) < 2, Eq. (28), and Eq. (29), same form as (18).
  - The attitude first enters the Prop. 2 region in finite time; e_x and e_v stay bounded meanwhile (proof p.8).
  - Definition 1 (p.4) distinguishes "exponentially attractive" from exponential stability.
- **Topological caveat (p.4–5):**
  - Critical points of Ψ are I and exp(πv̂), a 2-D set outside the region of attraction.
  - "It is impossible to construct a smooth controller on SO(3) that has global asymptotic stability."
- **Proof details (pp.5–8):**
  - ė_R = C(R_dᵀR) e_Ω with ‖C‖ ≤ 1, Eq. (30).
  - J ė_Ω = −k_R e_R − k_Ω e_Ω, Eq. (33).
  - The translational error with attitude mismatch X is bounded by ‖X‖ ≤ (k_x‖e_x‖ + k_v‖e_v‖ + B)α, Eq. (52).
  - ‖e_v(t)‖ < max{‖e_v(0)‖, B/(k_v(1−α))}, Eq. (54).

### Guaranteed vs empirical
- **Formal:** Props. 1–3 for the idealised model: no rotor dynamics, no saturation, no drag, exact state, exact J and m.
- **Empirical:** two numerical simulations only (p.5). No hardware.

### Numerical example (p.5)
- Plant: J = [0.0820, 0.0845, 0.1377] kg m², d = 0.315 m, m = 4.34 kg, c_τf = 8.004×10⁻⁴ m.
- Gains: k_x = 16m, k_v = 5.6m, k_R = 8.81, k_Ω = 2.54.
- **Case I** (elliptic helix): Ψ(0) < 0.15, satisfies Prop. 2.
- **Case II** (upside-down recovery): initial attitude error 178°, Ψ(0) = 1.995. Ψ < 1 at t = 0.88 s.

### Online inputs vs GT; control interface; timing
- Inputs: full state (x, v, R, Ω) assumed known. Outputs: (f, M), mapped to f_i through Eq. (1).
- No timing or compute data.

### Limitations
- **Stated:**
  - exponential stability only for attitude error < 90°;
  - almost-global attractiveness only for < 180°;
  - no global smooth controller exists (p.4–5).
- **Observed:**
  - assumes direct thrust control, with no motor lag or saturation;
  - the gain conditions (24)–(26) are only sufficient;
  - the b1d ⟂ b3d issue noted above;
  - no robustness to disturbances in this version.

### Code
None given. Animations at http://my.fit.edu/~taeyoung (p.5–6).

### MOTAR relevance / conflicts
- **Sign conventions differ from MOTAR / Aerial Gym.** Lee uses e3 down, gravity +g e3, thrust −fRe3. Aerial Gym / MOTAR use ENU: sim gravity [0, 0, −9.81] [code: `aerial_gym/config/sim_config/base_sim_config.py`] and body z up.
- **MOTAR's velocity controller departs from Lee (12)** [code: `MOTAR-mt-p1/aerial_gym/control/controllers/velocity_control.py`]:
  - errors are defined setpoint − current, with positive gains;
  - forces = (accel − gravity)·m;
  - optional tilt limit (`max_tilt_angle_deg`, default 45° in `lee_controller_config_navrl.py`);
  - optional `tilt_thrust_compensation` T = f_z/b3_z, used when b3_z > 0.5.
  - **Consequence:** Lee's Prop. 2/3 proof does not carry over to the compensated thrust law, because it relies on f = A·Re3 shrinking with the attitude error. Any stability claim MOTAR makes must be empirical.
- **MOTAR's torque law omits the J(…) feed-forward term of Eq. (13)** [code: `compute_body_torque` in `base_lee_controller.py`]. It keeps Ω × JΩ and uses the yaw rate via Euler-rate → body-rate conversion. Aerial Gym Eq. (3) also omits the term (Aerial Gym p.3).
- **Useful runtime monitors:**
  - Ψ(R, R_d) = ½ tr(I − R_dᵀR), logged per step. Ψ < 1 is the Prop. 2 regime.
  - ‖e_Ω‖² vs (2/λ_min(J)) k_R (1 − Ψ), Eq. (27). [CORRECTED: original CDC p.4 Eqs.(18),(27),(29) use lambda_max(J); old arXiv transcription must not supply a runtime stability bound.]
  - commanded-force norm > 0, Eq. (15).
  - bounded desired acceleration, Eq. (16), which the speed governor should enforce.
  - These can gate BOIA fallback execution: a fallback brake is only "trackable" when they hold.

### BOIA novelty overlap
- **(a) No. (b) No. (c) No.**
- **(d) No direct overlap.** It only supplies the conditions under which a fallback reference is trackable. Region-of-attraction bounds: Eqs. (17)–(18), (20), (27) (p.3–4).

#### 2026-10-19 original-copy re-read (Codex)

- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/Geometric Tracking Control of a Quadrotor UAV on SE(3).pdf`; SHA-256 `a664bdd644e82db8`; 6 physical PDF pages. Previous-public-copy SHA is **different**.
- **Read coverage:** physical pp.1,2,3,4,5,6; **FULL_READ** (full reread of six-page original CDC2010 copy including examples and references; earlier eight-page arXiv copy remains historical). No source code was executed and no result was reproduced.

- **Original metadata:** Taeyoung Lee, Melvin Leok, N. Harris McClamroch; 49th IEEE Conference on Decision and Control, Atlanta,15–17December2010; printed pp.5420–5425 correspond to physical pp.1–6. The six-page paper sends all proofs to reference[20], the eight-page arXiv:1003.2005v1 (p.3); the proofs are not in this original.
- **Signs/model — confirmed:** p.1 body b3 points down, thrust opposite it; p.2 Eqs.(2)–(5) give `xdot=v`, `m vdot=m g e3-f R e3`, `Rdot=R hat(Omega)`, `J Omegadot+Omega cross J Omega=M`. Rotor thrust/moment are directly commanded, with no motor dynamics/saturation. p.2 says rotors1/3 clockwise and2/4 counterclockwise; the earlier arXiv prose is not the source for this original's direction convention.
- **Errors and desired orientation:** p.3 Eqs.(6)/(7) `ex=x-xd`, `ev=v-vd`; Eq.(8) `Psi=tr(I-Rd^T R)/2`; Eqs.(10)/(11) `eR=vee(Rd^T R-R^T Rd)/2`, `eOmega=Omega-R^T Rd Omega_d`. Unlike the earlier v1 shorthand, p.2 and p.3 explicitly project b1d into the plane normal to b3d: normalized `b2d=(b3d cross b1d)/||...||`, `Rd=[b2d cross b3d,b2d,b3d]`. Nonparallel axes are required.

| Quantity | Earlier arXiv equation | Original CDC equation / physical page |
|---|---|---|
| Dynamics | (2)–(5) | (2)–(5), p.2 |
| ex, ev, Psi, eR, eOmega | (6)–(11) | (6)–(11), p.3 |
| Thrust f | (12) | **(15), p.3** |
| Moment M with angular feedforward | (13) | **(16), p.3** |
| Desired b3d | (14) | **(12), p.3** |
| Nonzero desired force | (15) | **(13), p.3** |
| Bounded desired acceleration/gravity term | (16) | **(14), p.3** |
| Propositions/gain conditions | (17)–(29) | (17)–(29), **p.4** |

- **Control:** write `A=-kx ex-kv ev-mg e3+m xdd_d`; original Eq.(12) `b3d=-A/||A||`, Eq.(15) `f=-A dot R e3`, Eq.(16) `M=-kR eR-kOmega eOmega+Omega cross J Omega-J(hat(Omega) R^T Rd Omega_d-R^T Rd Omegadot_d)`. Eq.(13) requires `||A||!=0`; Eq.(14) requires bounded `||-mg e3+m xdd_d||<B`. The thrust projection is integral to the stability argument, not an optional interchangeable detail.
- **Stability — confirmed with full conditions:** p.4 Prop.1 requires `Psi(0)<2` **and** Eq.(18) `||eOmega(0)||^2 < 2 kR(2-Psi(0))/lambda_max(J)`. Prop.2 complete-system exponential stability requires `Psi(0)<=psi1<1`, gain conditions Eqs.(24)–(26), and Eq.(27) `||eOmega(0)||^2 < 2 kR(1-Psi(0))/lambda_max(J)`. Thus '<90deg' alone is insufficient. Prop.3 with `1<=Psi(0)<2` plus Eq.(29) angular-rate bound and Prop.2 design gives exponential **attractiveness**, not global exponential stability. The denominator is **lambda_max(J)** in original Eqs.(18),(27),(29), not lambda_min.
- **Inputs/cost/validation:** exact full state/parameters and differentiable reference; no perception, map, target-belief training or GT-boundary experiment. Outputs direct f/M/individual rotor thrust. p.5 numerical helix and178deg recovery, figures p.6; no real-flight claim, compute/hardware latency or trained model size. Animation URL `http://my.fit.edu/~taeyoung` (p.6 figure captions); no implementation or weights URL.
- **MOTAR comparison:** M2 §4 source audit reports ENU/+b3 thrust,45deg desired tilt clipping, `T=f_z/b3_z` compensation and omitted angular feedforward. These are departures from CDC Eq.(15)/(16), so the ideal Lee stability proof cannot be cited as a guarantee for FrozenPolicyF/BOIA. No production code is changed. The M2 verbal Lee-force expression is also historically sign-inconsistent with this original; the authoritative A/f expression above resolves it. That source-attribution correction does not change M2's correctly read MOTAR velocity law or BOIA's stated surrogate rollout.
- **BOIA overlap:** (a) **no**, (b) **no**, (c) **no**, (d) **no direct backup/terminal-first algorithm**, (e) **no**. It supplies conditional trackability context only. Any dynamic safety claim still needs G5 qualification of the actual controller and braking envelope.


---

## 3. Aerial Gym Simulator: A Framework for Highly Parallelized Simulation of Aerial Robots

- **Authors / venue / year:** M. Kulkarni, W. Rehberg, K. Alexis. IEEE RA-L, preprint accepted February 2025. arXiv:2503.01471v1, 3 Mar 2025 (p.1).
- **Path:** `/home/fair/Downloads/Aerial Gym Simulator: A Framework for Highly Parallelized.pdf`
- **Identity:** 8 pp, identical (per caller).
- **Pages read:** 1–8, all text including references. pp.3, 4, 6 checked visually (controller equations, Tables I–II).
- **Read status:** FULL_READ

### Problem
A parallel GPU simulator for arbitrary multirotors with a control suite at many abstraction levels, custom GPU ray-cast sensors, and DRL integration (p.1).

### Architecture (p.2)
- Managers for rendering, robots and environments, plus physics-engine and DRL interfaces.
- All share the **Global Tensor Dictionary (GTD)**, with in-place tensor operations.
- Aerial Gym "obtains states … from the Isaac Gym physics engine and updates the GTD at each physics step". Controllers read GTD states and output forces/torques.
- Tasks follow the Gymnasium API (p.2).

### Physics engine and timesteps
- The physics engine is NVIDIA Isaac Gym [7] (p.1–2). The paper does not name the underlying solver.
- **The paper does NOT state the physics dt or substeps.** The only timestep stated is for the motor-control policy experiment: "control time-step duration of 0.01 s in simulation. Inference is performed at 250 Hz" (p.7).
- [code] The MOTAR worktree `base_sim_config.py` has dt = 0.01, substeps = 1, gravity [0, 0, −9.81]. Variants `sim_config_2ms.py` (dt = 0.002) and `sim_config_4ms.py` (dt = 0.004) also exist.
- **Aerodynamics:** "linear and quadratic terms can be configured to simulate aerodynamic drag" (p.2).
- **Collisions:** "handle complex meshes and convex decompositions of non-convex meshes to simulate collisions … at a higher computation cost" (p.2).

### Controllers (p.3–4)
- "Adapted geometric controllers based on [34]" (Lee et al. 2011). They have "sub-optimal performance on non-planar configurations". They are PyTorch-JIT on GPU (p.3).
- **Attitude-thrust and rate-thrust control** "as per [12] (RotorS) which is inspired by [34]":
  - e_R = ½(R_dᵀR − RᵀR_d)^∨, Eq. (1). e_Ω = Ω − RᵀR_dΩ_d, Eq. (2).
  - M = −k_R e_R − k_Ω e_Ω + Ω × JΩ, Eq. (3). This **omits Lee's J(…) feed-forward term** [observed].
  - Attitude-thrust uses Ω_d = 0. Rate-thrust uses R_d = R. Thrust f goes directly to allocation (p.3).
- **Position, velocity and acceleration control:**
  - **f = (k_x e_x + k_v e_v + m g e3 − m ẍ_d)·Re3, Eq. (4) (p.3).**
  - b_3c = −(−k_x e_x − k_v e_v − m g e3 + m ẍ_d)/‖·‖₂, Eq. (5) (p.4).
  - χ = [cos Φ_d, sin Φ_d, 0]ᵀ, b_2c = (b_3c × χ)/‖·‖, R_d = [b_2c × b_3c; b_2c; b_3c] (p.4).
  - Modes: velocity control sets e_x = 0 and ẍ_d = 0; acceleration control sets e_x = e_v = 0; position control sets e_v = 0 and ẍ_d = 0 (p.4).
  - e3 is "a unit vector in the z-direction". The sign of e_x is not defined in the paper.
  - [observed] **Sign inconsistency in print.** With ENU and z up, Eq. (4) is consistent only if e_x = x_d − x. The −m ẍ_d term then has the wrong sign for an acceleration feed-forward. Eq. (5) is Lee's NED Eq. (14) copied verbatim. [CORRECTED: original pp.3–4 indeed print negative acceleration feedforward, but physical inconsistency is conditional on ENU/error conventions; the PDF alone does not establish a production bug.]
  - [code] The MOTAR velocity controller uses e = setpoint − current and accel_cmd = K_pos e_x + K_vel e_v. In velocity mode there is no ẍ_d term, so the printed −mẍ_d sign does not bite there. **Verify the acceleration-control mode separately.**
- **Body-wrench control:** W = B U and U_ref = B⁺ W_ref (pseudoinverse), Eqs. (6)–(7).
  - "suboptimal for systems with input constraints". That is, **no saturation-aware allocation** (p.4).
- **RPM / thrust setpoint interfaces:** u = c_f r² (p.4).

### Motor model (p.4)
- ṙ_k = (r_k^ref − r_k)/τ_k, Eq. (8).
- τ_k = τ_k^inc if r^ref ≥ r, else τ_k^dec, Eq. (9).
- Integrated with RK4 or Euler. Force u = c_f r². Torque −d_k c_τ u.

### Sensors (p.4–5)
- **Native Isaac Gym limits (p.4):** pinhole cameras only; no ToF/LiDAR; slow loop-based pose randomisation.
- **Custom NVIDIA Warp ray-casting (p.4):**
  - A per-environment mesh M_i^base with per-sub-mesh transforms; a BVH is recomputed.
  - "While computationally expensive, performing this exclusively for randomization of static environments allows …" (p.4).
- **Per-pixel rays (p.4):**
  - **range** (sensor → hit distance) for ToF and LiDAR;
  - **depth** (distance of the hit from the image plane) for cameras;
  - stereo shadows: rays are projected back to the second sensor, and pixels whose rays hit the environment are marked invalid;
  - segmentation, face index, barycentrics and surface normals; point clouds.
- **Provided idealised configs:** Ouster OS-0/1/2/Dome, RealSense D455, Oak-D (Pro W), ST VL53L5CX (p.4).
- **IMU, Eqs. (10)–(13) (p.4–5):**
  - a_meas = a_true + b_a + n_a and Ω_meas = Ω_true + b_Ω + n_Ω, with random-walk biases.
  - **a_true = R_tᵀ(F_true/m + g e3)**, where F_true is the net force in the world frame W.
  - Noise parameters are provided for the VN-100 and BMI085 (p.5).

### Reset / parallel assumptions
- Individual environments can be "randomized separately and modified at runtime" (p.1).
- Meshes can be maintained and updated per environment (p.6).
- **The BVH rebuild is framed as a static-environment randomisation step (p.4).** The paper does not describe per-step BVH updates for moving objects. Treat dynamic-object rendering as **unverified**.

### Learning interfaces and experiments
- **Frameworks:** RL Games, Sample Factory, CleanRL (p.5).
- **Benchmark hardware:** Threadripper Pro 3975WX + RTX 3090. Constant-RPM quadrotors in obstacle-free scenes (physics); 20 cubes and a 270×480 depth+seg camera (rendering) (p.5).
- **Table I (p.6), Aerial Gym:**
  - physics SPS (×10⁶) = 0.007 / 0.130 / 1.43 / 4.43 at 2⁴ / 2⁸ / 2¹² / 2¹⁶ envs;
  - rendering FPS = 1404 / 2370 / 2714 / 3921 at 2⁴ / 2⁶ / 2⁸ / 2¹⁰ envs.
- **Table II (p.6):** Warp vs Isaac Gym, room with 15 obstacles, controller in the loop.
  - e.g. 270×480: Aerial Gym 1592 / 2057 / 2408 / 2627 / 2750 FPS vs Isaac Gym 1430 / 1574 / 1875 / 1994 / ✗ at 128 / 256 / 512 / 1024 / 2048 envs.
  - "scales worse with resolution" (p.6).
- **Position-setpoint task (p.6):**
  - action a_t = [v_ref, Ω_z^ref] (velocity + yaw rate) or [ẍ_ref, Ω_z^ref], **expressed in the vehicle frame V**;
  - ArduPilot + ROVIO + D455.
  - This is the same action family as MOTAR's frozen policy.
- **Vision navigation (p.6):**
  - state includes a 64-D Deep Collision Encoder latent from 848×480 depth; trained 142 min wall-clock;
  - in the real world the vertical velocity was constrained;
  - max 2.83 m/s, mean 1.54 m/s in a cluttered corridor.
- **Motor-control policy (p.7):**
  - 2-layer MLP (32, 24), 5 seeds; sim control step 0.01 s; 250 Hz inference;
  - average position tracking error ‖e‖₂ = 0.09 m;
  - real-world motor commands differ across motors, attributed to asymmetric mass or motor constants (p.7).

### Guaranteed vs empirical
No formal guarantees. Everything is a benchmark or demonstration. Sim2real for the controllers was done by matching "the response of the simulated controllers and motor models … with that of the real-world system" (p.6).

### Stated limitations
- geometric controllers are sub-optimal on non-planar airframes (p.3);
- pseudoinverse allocation ignores input constraints (p.4);
- Warp rendering scales worse with resolution (p.6);
- BVH rebuild is expensive (p.4);
- Isaac Gym native cameras use a lot of memory and crash at high env counts (p.6).

### Observed limitations
- physics dt not reported;
- printed controller sign inconsistencies, Eqs. (4)–(5);
- no sensor latency, motion blur or depth-noise model described for the cameras, other than stereo shadows;
- dynamic-object ray-casting not described.

### Code
https://github.com/ntnu-arl/aerial_gym_simulator (p.1, p.7).

### MOTAR relevance / conflicts
This is the MOTAR substrate. What must be validated:
- **(i)** controller sign/frame;
- **(ii)** depth vs range semantics. Camera depth is the image-plane distance, LiDAR returns range. Any geometric visibility or occlusion forecast must convert correctly;
- **(iii)** whether the moving target's mesh is updated in the Warp BVH every step. If not, the camera sees a stale target. This is critical for detection-probability calibration;
- **(iv)** motor-lag effect on braking distance;
- **(v)** the IMU convention if IMU is used;
- **(vi)** no saturation-aware allocation, so aggressive fallback brakes may be clipped silently.

### BOIA novelty overlap
- **(a) No. (b) No. (d) No.**
- **(c) No.** It provides only the action interface v_ref + Ω_z^ref in the vehicle frame (p.6). Arbitration must sit between the policy and this controller.

#### 2026-10-19 original-copy re-read (Codex)

- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/Aerial Gym Simulator A Framework for Highly Parallelized.pdf`; SHA-256 `5c73cafa8c22f042`; 8 physical PDF pages. Previous-public-copy SHA is **identical**.
- **Read coverage:** physical pp.1,2,3,4,5,6,7; **FULL_READ** (retained earlier FULL_READ of identical copy; equations plus mandatory timing/mesh/sign questions checked this pass). No source code was executed and no result was reproduced.
- **Equation spot checks:** p.3 Eq.(4); p.5 Eq.(10); p.4 Eq.(8). The sampling rule uses a local `random.Random` seeded from this PDF SHA; mandatory checks below are additional.

- **Original metadata:** Mihir Kulkarni, Welf Rehberg, Kostas Alexis; RA-L preprint accepted February2025, arXiv:2503.01471v1 stamp3March2025 (p.1). Simulator methods and benchmarks are not automatically MOTAR qualification.
- **Physics timestep question — confirmed:** pp.1–8 do not specify a global physics dt/substeps. p.7 explicitly states **0.01s control timestep** for the motor-policy example and250Hz hardware inference. This is not evidence that every simulation experiment uses a0.01s physics integrator. M2's actual source-config/receipt evidence remains necessary.
- **Eq.(4)/(5) sign question — printed negative, physical interpretation conditional:** p.3 Eq.(4) prints `(kx ex+kv ev+mg e3-m xdd_d) dot R e3`; p.4 Eq.(5) outer-minus form yields the same direction proportional to `kx ex+kv ev+mg e3-m xdd_d`. The negative acceleration term is unambiguous in print. With MOTAR's ENU,+body-z thrust and setpoint-minus-current errors, a nonzero desired acceleration would require the opposite feedforward sign. However, the paper does not fully define ex/ev signs or inertial gravity-axis orientation, and the formula algebra is consistent with the original Lee downward-e3 convention. Therefore **a production sign bug is NOT established by the PDF alone**. In p.4 velocity mode `xdd_d=0`, so this issue has no direct effect on FrozenPolicyF's velocity interface or BOIA v1. Acceleration-mode sign remains a separate source/physics validation task, not executed here.
- **Mesh question — confirmed with scope:** p.4 introduces per-env base meshes/transforms/BVH but explicitly says expensive transforms are performed exclusively for randomization of **static environments**. pp.5–6 also advertise runtime modification of individual env meshes. These capability statements do not demonstrate same-step dynamic-target freshness or cross-env reset isolation; MOTAR's independent ISO evidence supplies that qualification.
- **Other equation checks:** p.3 Eqs.(1)–(3) omit Lee angular-feedforward; p.4 Eqs.(6)–(7) use unconstrained pseudoinverse allocation; Eqs.(8)–(9) model first-order RPM response with separate rise/fall constants. p.4 distinguishes optical-plane camera depth from radial LiDAR range. Those semantics matter to BOIA's geometry; do not interchange them.
- **Online/training/logic/cost:** p.2 GTD shares simulator state internally; this is not permission to expose GT to a new online supervisor. p.6 state-based setpoint tasks use full robot state, navigation uses depth features; p.7 motor-control deployment uses Qualisys pose. Those are different task contracts, not proof of MOTAR target identity realism. No recovery/expiry/termination supervisor is specified. p.5 benchmark uses ThreadripperPro3975WX/RTX3090; p.6 TableII rendering is scene/resolution/env-count-specific, not a BOIA5ms guarantee. Source: https://github.com/ntnu-arl/aerial_gym_simulator (pp.1,7); no BOIA weights exist.
- **BOIA overlap:** (a) **no**, (b) **no**, (c) **no**, (d) **no**, (e) **no**. Relevant as simulator/control/sensor context only; current RGB-D+LiDAR contract remains unchanged.


---

## 4. EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors

- **Authors / venue / year:** X. Zhou, Z. Wang, H. Ye, C. Xu, F. Gao. IEEE RA-L, accepted October 2020. arXiv:2008.08835v2, 7 Dec 2020 (p.1).
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/EGOPlanner__2008.08835v2.pdf`
- **Identity:** **8 pp, a DIFFERENT version from the user's copy.** I did not open the user copy, so page and equation numbers may differ.
- **Pages read:** 1–8, all text. pp.4–6 checked visually (Eqs. (5)–(19), Alg. 2, Tables I–II).
- **Read status:** FULL_READ (requested METHOD_REVIEWED+)

### Problem
ESDF construction dominates the computation of gradient-based local planners. EGO replaces the ESDF collision gradient with a guiding-path-based repulsion (p.1).

### Assumptions
- Static environment. It "can tackle slowly moving obstacles (below 0.5 m/s) without any modification" (p.8).
- Occupancy map from depth (D435).
- Uniform cubic B-spline in flat-output space, following Mellinger [15] (p.3).

### Pipeline (Alg. 1 p.2; Alg. 2 p.6)
1. Initialise Q from Q_last and the goal (Alg. 2 line 2).
2. While not collision-free:
   - CheckAndAddObstacleInfo: for each colliding segment, run A* for a collision-free path Γ. For each control point in the segment, find {p, v} anchor/direction pairs. p lies on the obstacle surface (chosen on Γ). v is the unit vector from the control point toward p.
   - Evaluate the penalty and take one optimiser step (Alg. 2 lines 3–7).
3. If infeasible: ReAllocateTime, then CurveFittingOptimize (lines 8–11).

### Key equations
- **Collision distance:** d_ij = (Q_i − p_ij)·v_ij, Eq. (1), p.3.
  - An obstacle counts as new only if d_ij > 0 for all valid j (p.3).
- **Derivative control points:** V_i = (Q_{i+1} − Q_i)/Δt; A_i and J_i likewise, Eq. (2), p.3.
- **Convex-hull property** (p.3, Fig. 4): the span on (t_i, t_{i+1}) lies in the hull of Q_{i−p_b}..Q_i.
- **Objective:** J = λ_s J_s + λ_c J_c + λ_d J_d, Eq. (3), p.4.
- **Smoothness:** J_s = Σ‖A_i‖² + Σ‖J_i‖², Eq. (4).
- **Collision:** c_ij = s_f − d_ij (s_f is the safety clearance). Eq. (5) is C²:
  - j_c = 0 for c ≤ 0;
  - j_c = c³ for 0 < c ≤ s_f;
  - j_c = 3s_f c² − 3s_f² c + s_f³ for c > s_f.
  - J_c = Σ_i Σ_j j_c, Eq. (6). Analytic gradient in Eq. (7) (p.4).
- **Feasibility:** per axis, |Φ_r^(k)(t)| < Φ_r,max^(k) (p.4).
  - J_d = Σ w_v F(V_i) + Σ w_a F(A_i) + Σ w_j F(J_i), Eq. (8). F(C) = Σ_r f(c_r), Eq. (9).
  - Piecewise f in Eq. (10) with derivative limit c_m and elastic coefficient λ < 1 − ε.
  - **Soft penalty**: "the cost function is a tradeoff of all weighted terms" (p.4).
- **Solver:** L-BFGS, Eqs. (11)–(13), with BB initial Hessian and a strong-Wolfe line search (p.5).
- **Time re-allocation (p.5):**
  - r_e = max{|V_i,r/v_m|, √|A_j,r/a_m|, ∛|J_k,r/j_m|, 1}, Eq. (14).
  - Δt′ = r_e Δt, Eq. (15).
  - Then refit: J′ = λ_s J_s + λ_d J_d + λ_f J_f, Eq. (16). J_f uses an anisotropic spheroid metric: axial d_a, radial d_r, Eqs. (17)–(18).
- **R_i = (Q_{i+1} − Q_{i−1})/(2Δt)**, Eq. (19), p.6.
- **Post-check:** "a collision check of a circular pipe with a fixed radius around the final trajectory is performed to provide enough obstacle clearance. The optimizer stops when no collision is detected." (p.6)

### Guaranteed vs empirical
- **Structurally exact:** the convex-hull sufficiency for feasibility (p.3–4). If every derivative control point is within its bounds, the whole curve is (per axis).
- **Not guaranteed:**
  - feasibility is a soft penalty, with λ < 1 − ε as margin;
  - collision-freeness rests on the termination test plus the pipe check against the *known* map;
  - **no backup trajectory and no treatment of unknown space**;
  - local minima from A* (p.8).
- **Empirical:**
  - Table I (p.6), 100 random maps: L-BFGS success 0.89; time min / avg / max 0.17 / 0.37 / 0.80 ms; function evaluations avg 79.04.
  - Table II (p.6): EGO success 0.89, energy 49.92, avg / max velocity 2.12 / 2.24 m/s, total 0.37 ms. EI (ESDF + collision-free initialisation) total 5.55 ms, of which ESDF 5.07 ms.
  - Table III (p.7): EGO t = 24.38 s, length 42.24, energy 196.64, t_plan 0.81 ms. Fast-Planner 30.76 s / 45.18 / 135.21, t_ESDF 4.01, t_plan 3.29.
  - Real world: 3.56 m/s indoors; narrowest passage < 1 m; > 3 m/s in forest (p.7).

### Implementation details (p.5)
p_b = 3. N_c ≈ 25, from a ~7 m horizon at ~0.3 m spacing (empirical). O(N_c) complexity.

### Online inputs vs GT
Online depth-built map and onboard state. Hardware platform from [19]. RealSense D435 with a modified emitter-strobe driver (p.6).

### Control interface
B-spline reference trajectory. The controller is not described here.

### Limitations
- **Stated (p.8):** A* local minima; conservative unified time re-allocation; static environments only (< 0.5 m/s moving obstacles tolerated).
- **Observed:** per-axis (box) limits, not a norm or thrust/tilt limits; no visibility or occlusion reasoning; known-map only.

### Code
https://github.com/ZJU-FAST-Lab/ego-planner (footnote, p.2). Video https://youtu.be/UKoaGW7t7Dk (p.7).

### MOTAR relevance / conflicts
- **Useful building blocks for BOIA (b):**
  - represent candidates as uniform cubic B-splines, so that the convex-hull feasibility check (Eq. (2) + p.3–4) is cheap and exact per axis;
  - the time-rescaling ratio r_e (Eq. (14)) is a principled speed-governor-compatible scaling;
  - the pipe collision check (p.6) is the swept-volume check MOTAR needs against vertical bars.
- **Conflicts:**
  - EGO is an optimiser, not a finite candidate scorer;
  - it relies on a map, which MOTAR's reactive policy lacks;
  - its per-axis limits do not match the Lee tilt limit (45° gives a lateral accel bound of g·tan45° ≈ 9.81 m/s² [code comment]).

### BOIA novelty overlap
- **(a) No.**
- **(b) Partly.** Dynamics-feasible trajectory generation with feasibility penalties and time re-allocation (Eqs. (8)–(10), (14)–(15), p.4–5). It is not a finite scored set and has no capture/loss/covariance terms.
- **(c) No.**
- **(d) No.** No fallback. The only safety measure is the post-optimisation pipe check (p.6).

#### 2026-10-19 original-copy re-read (Codex)

- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/EGO-Planner An ESDF-Free Gradient-Based.pdf`; SHA-256 `0d316681d1ae9ecd`; 8 physical PDF pages. Previous-public-copy SHA is **different**.
- **Read coverage:** physical pp.1,2,3,4,5,6,7,8; **FULL_READ** (full original final RA-L copy reread; duplicate original has identical SHA and remains preserved; rendered p.6 tables). No source code was executed and no result was reproduced.

- **Original identity:** Xin Zhou, Zhepei Wang, Hongkai Ye, Chao Xu, Fei Gao; IEEE RA-L6(2),April2021,pp.478–485, DOI10.1109/LRA.2020.3047728 (p.1). Both user EGO-Planner filenames have SHA `0d316681d1ae9ecd22a589f6b3e7e8c1b93b3d66a1f309d2a6b0e2686f88b7c7`; this normal filename is representative and the `(1)` duplicate is retained. The earlier arXiv v2 SHA differs.
- **Collision/mapping:** p.2 Eq.(1) `d_ij=(Q_i-p_ij) dot v_ij`, with per-control-point anchor/direction pairs found from a collision-free A* guide. p.3 only newly relevant obstacles are added; this is an on-demand obstacle surrogate, not an ESDF or semantic-GT target map.
- **Feasibility:** p.3 Eq.(2) derivative control points `V=(Q_next-Q)/dt`, then A,J recursively; convex-hull containment gives sufficient per-axis bounds over entire B-spline spans when derivative control points meet those bounds. p.4 Eqs.(8)–(10) enforce limits as weighted soft penalties, so convex-hull theory does not mean every numerical solution automatically meets them. Norm/thrust/tilt/rate feasibility is not implied by per-axis spline bounds.

| Formula | Earlier v2 citation | Original final copy |
|---|---|---|
| Anchor distance Eq.(1) | physical p.3 | **p.2 Eq.(1)** |
| Derivative control points Eq.(2) | p.3 | p.3 Eq.(2) |
| Objective/costs Eqs.(3)–(6) | p.4 | **p.3 Eqs.(3)–(6)** |
| Collision gradient/feasibility Eqs.(7)–(10) | p.4 | p.4 Eqs.(7)–(10) |
| L-BFGS step/update Eqs.(11)–(12) | p.5 | **p.4 Eqs.(11)–(12)** |
| Initial Hessian Eq.(13) | p.5 | p.5 Eq.(13) |
| Time scaling Eqs.(14)–(15) | p.5 | p.5 Eqs.(14)–(15), unchanged numbering |
| Anisotropic fitting Eqs.(16)–(18) | p.5 | p.5 Eqs.(16)–(18) |
| Tangent Eq.(19)/pipe check | p.6 | p.6 Eq.(19)/unnumbered pipe-check prose |

- **Time allocation and pipe check — confirmed:** p.5 Eq.(14) uses the maximum of velocity ratio, square-root acceleration ratio, cube-root jerk ratio and1 across axes/control points; Eq.(15) scales uniform dt by this ratio. It then refits under feasibility/smoothness/anisotropic shape cost. p.6 separately tests a fixed-radius circular pipe around the **final trajectory** and stops only if no collision is detected. Plain rescaling or endpoint checks are insufficient substitutes for this final geometric check.
- **Inputs/control/GT:** depth-built occupancy map and own state, fixed/abruptly supplied navigation goals, uniform cubic B-spline reference; no target-tracking or learned frozen-policy interface, no privileged learned training. p.6 uses RealSenseD435 depth with emitter strobing; controller/platform details are delegated to ref[19]. Hardware/CPU model is not explicitly printed in the original implementation section: **NOT_RECORDED in this PDF**, rather than inferred from a related platform.
- **Cost/experiments:** p.6 TableI,100 randomized maps, L-BFGS success0.89 and average optimizer time0.37ms; TableII total EGO0.37ms vs EI5.55ms, but failures are excluded from successful-case timing as stated p.6. No runtime bound or MOTAR comparison follows. p.7 reports indoor3.56m/s and sub1m passage. p.8 explicitly limits design to static environments; '<0.5m/s moving obstacles' is empirical tolerance, not a dynamic-obstacle safety theorem. Stated limitations are A*-induced local minima and conservative unified reallocation.
- **Code/weights:** https://github.com/ZJU-FAST-Lab/ego-planner (p.2), video https://youtu.be/UKoaGW7t7Dk (p.7); no learned weights.
- **BOIA overlap:** (a) **no**; (b) **partial** (feasible trajectory optimization and post-check, pp.3–6); (c) **no**; (d) **no** (no preserved terminal-stopping backup or task termination handling); (e) **no**. Useful feasibility inspiration, not proof that BOIA's first-order rollout matches its real controller.


---

## 5. Adaptive Tracking and Perching for Quadrotor in Dynamic Scenarios

- **Authors:** Y. Gao*, J. Ji*, Q. Wang, R. Jin, Y. Lin, Z. Shang, Y. Cao, S. Shen, C. Xu, F. Gao (ZJU / DJI / HKUST) (p.1).
- **Venue / year:** arXiv:2312.11866v2, 18 Jan 2024 (p.1). The venue is not printed in the PDF; it extends conference papers [10], [11] (p.2).
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/AdaptivePerching__2312.11866v2.pdf`
- **Identity:** 20 pp, identical (per caller).
- **Pages read:** 1–18, text. pp.13–17 checked visually (Tables III–VIII, Figs. 11–19). pp.19–20 are references only and were not read.
- **Read status:** METHOD_REVIEWED

### Problem
- Track a moving ground platform with reliable visual perception (avoiding occlusion and FoV loss).
- Then perch with full-state synchronisation (position, velocity, attitude) on a moving, possibly inclined surface.
- Safety and actuator constraints apply, with high-rate onboard replanning (p.1–2).

### Assumptions / model
- **Simplified quadrotor model (Eq. (1), p.4):**
  - τ = f̃ R e3/m, p̈ = τ − ḡ e3, Ṙ = R ω̂;
  - **e3 = [0, 0, 1], gravity −ḡ e3, i.e. ENU z-up, the same as Aerial Gym** [observed];
  - the body rate is the input.
- Flat outputs z = (p_x, p_y, p_z, ψ), Eq. (2).
- **Target model (p.4):**
  - EKF with the CTRV model, state χ = (ϱ_x, ϱ_y, ϱ_z, θ, v_h, v_v, ω_ϱ), Eq. (4);
  - prediction by CTRA motion primitives, choosing the collision-free primitive with the least control effort;
  - output: a discrete series Ψ = {ϱ_k}, Eq. (5).
- **Target measurement (p.3, p.16):** fusion of differential-GPS relative localisation (meter-level) and visual detection (cm-level, AprilTag).
- **Geometry:**
  - the drone underside is a disc C, Eq. (6), with radius r̄ and offset ℓ̄;
  - the perching surface is a half-space P = {hᵀx ≤ b}, Eq. (7), with h = −z_s (p.5).

### Visibility metrics (tracking, Sec. IV, p.5–6)
- **Observation distance (OD):** d_ℓ ≤ δ ≤ d_u separately for horizontal and vertical, Eq. (8).
  - Penalties in Eq. (9), using g(x) = max(x, 0)³ and the C² smooth L_µ of Eq. (10). The "3" in g's definition is [UNCERTAIN] in the extraction. [CORRECTED: original p.5 Eq.(9) confirms cubic max(x,0)^3.]
- **Observation angle (OA):** ψ_e = atan2(e2ᵀ(ϱ−p), e1ᵀ(ϱ−p)), Eq. (11). J_OA = (ψ − ψ_e)², Eq. (12). Gradients in Eqs. (13)–(14).
- **Elastic occlusion effect (OE):**
  - the "confident FoV" is approximated by M balls along the line of sight: c_i = p + λ_i(ϱ − p), r_i = ρ λ_i ‖p − ϱ‖, λ_i = i/M, Eqs. (15)–(16);
  - constraint r_i < Ξ(c_i), where Ξ is the ESDF distance, Eq. (17);
  - J_OE = Σ g(r_i − Ξ(c_i)), Eq. (18); gradient Eq. (19);
  - the gradient both moves the drone toward free areas and **elastically shortens the tracking distance** (Fig. 4, p.6).

### Perching constraints (Sec. V, p.6–9)
- **SE(3) collision:** C ⊂ P, i.e. r̄‖BᵀRᵀh‖ + hᵀo − b ≤ 0, Eqs. (20)–(21c).
  - R is recovered from the flat output: τ = p⁽²⁾ + ḡe3, Eq. (22); z_b = τ/‖τ‖, Eq. (23); Hopf-fibration quaternion, Eq. (24).
  - Activated only when ‖p − ϱ‖ ≤ d̄, through the smoothed logistic L_ε, Eqs. (26)–(28).
- **Relative height:** J_Δz, Eq. (29).
- **Pixel-level perception:** tag kept central, F_p = u_c² + v_c², with pinhole projection Eqs. (30)–(31).
  - Activated only within [d̄_min, d̄_max], Eqs. (32)–(33). Gradients in Eqs. (34)–(35).
- **Actuator limits:**
  - τ_min ≤ ‖τ‖ ≤ τ_max, Eqs. (36)–(37);
  - body rates, Eqs. (38)–(39), with ‖ω_xy‖ = ‖ż_b‖ and ż_b = f_DN(τ) p⁽³⁾, Eqs. (40)–(42);
  - ω_z ≈ ψ̇.
- **Flexible terminal adjustment (p.8–9):**
  - p(T) = ϱ(T) + l̄ z_s(T), Eq. (43); ψ(T) = θ(T), Eq. (44).
  - The hard relative-velocity condition, Eq. (45), is **relaxed** to p⁽¹⁾(T) = ϱ⁽¹⁾(T) + δ_v(T), with tangential slack ν = [ν_x, ν_y] and J_ν = ‖ν‖², Eqs. (46)–(47).
  - Motivation (Fig. 7, p.8): hard terminal velocity conflicts with the safety and actuator constraints.
  - Attitude alignment: z_b(T) = z_s(T), Eqs. (48)–(49). It is converted to terminal acceleration p⁽²⁾(T) = τ_e z_s(T) − ḡe3, with τ_e = τ_m + τ_r sin(τ_f) keeping the thrust in bounds implicitly, Eq. (50).
  - Terminal jerk = 0. Collected as z^{s−1}(T) = F(ϱ(T)), Eq. (51).

### Front-end and optimiser (Sec. VI, p.9–11)
- **Alg. 1 (p.9), occlusion-aware greedy viewpoint search:**
  - sample along the ray from the previous viewpoint to the previous target point, or along the target-trajectory segment;
  - CheckRay picks the first occlusion-free point;
  - extend to the desired tracking distance d̄;
  - A* between viewpoints.
- **Optimisation problem:** min ∫‖z^(s)‖² + ρT, Eq. (53), subject to boundary conditions plus continuous constraints G and discrete-time constraints H.
- **Solver details:**
  - MINCO (T_s) parametrisation, Eq. (54);
  - penalty method with trapezoidal sampling, Eqs. (55)–(61);
  - "a fast post-optimization check for safety and dynamic feasibility" (p.10);
  - time slack by diffeomorphic substitution, Eq. (63).
- **Tracking setup:** T_3 MINCO for position, T_2 for yaw. J_G = [J_o, J_v, J_a] (ESDF obstacle cost and v/a limits), J_H = [J_OD, J_OA, J_OE], Eqs. (64)–(67). T ≥ T_p, and the end is fixed to the last viewpoint (p.11).
- **Perching setup:** T_4 MINCO, the lowest order able to constrain the terminal angular velocity. J_G = [J_c, J_p, J_ω, J_τ, J_Δz, J_ν], Eq. (68) (p.11).

### Guaranteed vs empirical
- **Nothing is formally guaranteed.** The abstract and contributions claim "safety and feasibility guaranteed by terminal constraint relaxation" (p.2), but every inequality constraint is a soft penalty plus a post-check (p.10).
- **Exact by construction (implicit):** terminal-thrust bounds via the sin transformation (Eq. (50)); equality constraints via MINCO (p.10).
- **Empirical results:**
  - **Tracking sim (Table I, p.12):**
    - drone v_max 3.0 m/s, a_max 6.0 m/s²; target 1.5 m/s, 2.0 m/s²;
    - ‖ω_max‖ 3.0 rad/s; FoV 80°×65°; 640×480; tracking distance 5.5 m.
  - **Computation (Table II, p.12):** Ours t_path 0.96 + t_opt 1.85 = 2.81 ms; Elastic 3.58 ms; Fast 9.94 ms; Jeon's 14.32 ms [UNCERTAIN: Jeon's sum 11.32 + 3.20 ≠ 14.32 as extracted]. [CORRECTED: rendered original p.12 TableII confirms all three printed values; this is a source arithmetic inconsistency.]
  - **Occlusion threshold:** D_occ = 0.3 m (Fig. 11, p.13). Obstacle spacings 6 / 5 / 4 m for sparse / medium / dense (p.12).
  - **Perching sim (Table III, p.13):** τ_min 5.0, τ_max 17.0 m/s², ‖ω_max‖ 3.0 rad/s.
  - **Perching computation (Table IV, p.13):** ours 3.76 / 4.31 / 16.68 ms at 30° / 60° / 90°; Paneque 239.59 / 308.05 / 208.81 ms.
  - **Visibility ablation (Table VI, p.15):** observable distance 0.64 → 1.96 m (0.0 rad), 0.44 → 1.99 m (0.5 rad), 3.61 → 4.15 m (1.5 rad).
  - **Real world (p.16–17):**
    - DJI Mavic 3 with onboard replanning at 20 Hz on a platform "8× slower than simulation settings" (p.16);
    - Table VII field: target 8.0 m/s, drone 9.51 / 8.44 m/s (max / avg), error 14.19 cm;
    - Case 2: SUV at 30 km/h (8.3 m/s), drone max ≈ 10 m/s, x–y perching error 10.7 cm (p.17);
    - Table VIII: 1.0 rad pitch, target 2.0 m/s, error 9.33 cm.
- **Simulation hardware:** i5-9400F + GTX 2060 (p.11).

### Online inputs vs GT
- Target position from DGPS + visual detection, then EKF.
- Mapping and odometry from the DJI onboard stack ("localization and mapping functionality of the DJI MAVIC3 is retained", p.16).
- The simulation benchmark uses the Elastic-Tracker simulator (p.12). The source of target state in simulation is not stated in what I read.

### Control interface
The trajectory is converted to "control instructions … transmitted to the flight controller" (p.16). The controller uses integrator control against wind and ground effect (p.17).

### Limitations
- **Stated:**
  - perching error is mostly due to target-estimation delay (detection and link) and tracking error (p.17);
  - the perching stage starts on an external order, not autonomously (p.16).
- **Observed:**
  - visibility is deterministic geometry (ESDF balls), with no probabilistic detection model;
  - no explicit recovery when the target is lost;
  - the ESDF dependence contrasts with EGO;
  - no formal safety guarantee;
  - the CTRV/CTRA prediction suits road vehicles.

### Code
No repository URL found in pp.1–18. Videos: youtu.be/5XKm7qkp2Xs and youtu.be/fBwW93Zq9ss (p.1).

### MOTAR relevance / conflicts
- **Most relevant to BOIA (a) and (b)** as a deterministic baseline.
  - The OE ball-chain test (Eqs. (15)–(17)) can be computed on GT bars to label "geometric occlusion" and to calibrate or validate MOTAR's belief-conditioned detection-probability forecast.
  - "Elastic" distance shrinking in clutter (Fig. 4D, p.6) is a behaviour BOIA candidates should be able to express.
  - The relaxed terminal-velocity idea (Eqs. (46)–(47)) is a precedent for terminal-first handling: soften terminal conditions rather than failing.
- **Conflicts:**
  - a single optimised trajectory, not finite candidates;
  - it is the planner itself, with no frozen RL policy to arbitrate over;
  - ESDF/map-based;
  - the target is a ground vehicle with CTRV dynamics.

### BOIA novelty overlap
- **(a) Partly.** It has a visibility-aware forecast along the *predicted* target trajectory (OD/OA/OE at discrete future times, J_H, Eq. (64b), p.11). It is deterministic, not belief-conditioned, and gives no detection probability. The EKF covariance is not used in planning.
- **(b) Partly.** Alg. 1 does a discrete viewpoint search (p.9), and dynamic feasibility comes from Eqs. (36)–(42) and (66)–(67). It does not score a finite candidate set by capture progress + loss probability + covariance growth.
- **(c) No.**
- **(d) Partly.** Terminal handling via flexible terminal adjustment and relaxation (Eqs. (43)–(51), p.8–9). There is no safe fallback trajectory.

#### 2026-10-19 original-copy re-read (Codex)

- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/Adaptive Tracking and Perching for Quadrotor.pdf`; SHA-256 `d22709ab18b007b6`; 20 physical PDF pages. Previous-public-copy SHA is **identical**.
- **Read coverage:** physical pp.1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20; **FULL_READ** (full original copy reread including previously unfinished pp.19–20; identical SHA to earlier public copy; rendered pp.8/12). No source code was executed and no result was reproduced.

- **Metadata:** Yuman Gao, Jialin Ji, Qianhao Wang, Rui Jin, Yi Lin, Zhimeng Shang, Yanjun Cao, Shaojie Shen, Chao Xu, Fei Gao; this copy is arXiv:2312.11866v2,18January2024 (p.1), not a journal typeset copy. All20physical pages were read. pp.19–20 finish Fig.21, references[16]–[52], and author biographies; there is no hidden algorithm appendix or new controller proof there.
- **Assumptions/inputs:** p.3 target measurements fuse visual tags with cooperative differential-GPS relative localization; pp.4–5 CTRV EKF and collision-aware CTRA primitives give deterministic future target states. The ESDF map and onboard own-state estimates feed flat-output planning; there is no calibrated target-belief detection-probability model or learned-policy training. Real deployment p.16 retains DJI localization/mapping and uses front/down cameras/tags, with cooperative coarse localization. These are not MOTAR's noncooperative RGB-D+LiDAR measurement contract.
- **LOS ball chain — confirmed original numbering:** p.6 Eq.(15) `c_i=p+lambda_i(rho_target-p)`; Eq.(16) `r_i=rho lambda_i ||p-rho_target||`; Eq.(17) `r_i<Xi(c_i)`; Eq.(18a/b) `J_OE=sum g(r_i-Xi(c_i))`; Eq.(19) supplies the p-gradient. The cone/ball volume criterion is a deterministic visibility margin, not the probability of a partial silhouette or detector output. p.5 Eq.(9) uses `g(x)=max(x,0)^3`; the cubic exponent is confirmed. Eq.(10) separately smooths the far-distance linear penalty.
- **Terminal relaxation — confirmed:** p.8 Eq.(43) aligns position with the moving plane; Eq.(44) yaw; Eq.(45) hard relative velocity includes normal approach `-vbar_n z_s`. Eq.(46a/b) adds tangential velocity components `nu_x x_s+nu_y y_s`, while keeping the normal approach; Eq.(47) penalizes `||nu||^2`; Eq.(48) aligns body-z/plane normal. p.9 Eqs.(49)/(50) convert this to final acceleration and bound terminal thrust with `tau_e=tau_m+tau_r sin(tau_f)`. This is terminal *state optimization*, not simulator capture/termination ordering and not a braking backup.
- **Printed issue beyond historical extraction uncertainty:** rendered p.8 Eq.(40) prints `||omega_xy||^2=omega_x^2+omega_y^2=||zdot_b||` with no square on the last norm. For a unit body-z vector, the physically consistent relation is `||omega_xy||^2=||zdot_b||^2`; the printed final term is dimensionally inconsistent. Eqs.(41)–(42) define `zdot_b=f_DN(tau) p^(3)` correctly as a rate. This is an apparent paper typo, not a reproduced implementation error, and BOIA v1 does not implement this perching constraint.
- **Guarantee/cost/experiments:** p.10 replaces inequalities by sampled penalties plus a fast post-optimization safety/feasibility check, so do not elevate all continuous constraints into a formal robust guarantee. pp.11–13 simulation uses i5-9400F and GTX2060; p.12 TableII ours0.96+1.85=2.81ms. The same rendered table actually prints Jeon's11.32+3.20=14.32ms: the total is arithmetically inconsistent and must remain a flagged source issue, not 'corrected' to invented data. p.16 onboard MAVIC3 replans20Hz on a platform described as8x slower; this is not a measured RTX3070 BOIA budget. pp.17–19 perching errors/trajectory plots remain system-level cooperative experiments, not MOTAR recovery evidence.
- **Transitions/limitations/URLs:** p.16 perching begins on an **external order**. p.17 cites perception/link delay and tracking error as perching-error sources. No explicit blind long-loss search, identity clock, safe fallback or terminal-first capture rule is defined. Videos at https://youtu.be/5XKm7qkp2Xs and https://youtu.be/fBwW93Zq9ss (p.1); no repository/weights URL is printed in pp.1–20. Observed limitations: cooperative target localization, ESDF dependence and deterministic extrapolation; distinguish these from author-stated latency issues.
- **BOIA overlap:** (a) **partial** (deterministic future LOS/FOV, no probabilistic belief forecast); (b) **partial** (greedy finite front-end plus continuous MINCO, pp.9–11); (c) **no**; (d) **partial only for terminal-boundary relaxation**, no fallback/terminal-first ordering; (e) **partial at task level** (safe moving-platform rendezvous/perching endpoints), no post-occlusion two-second competing-risk evaluation. No change to 07–09 or production is applied.


---

## Checks MOTAR must run (derived from these papers)

Each check names its source and a pass criterion. [code] references are to `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/aerial_gym/...`.

### A. Frames, signs, controller

1. **Gravity / thrust sign (Lee Eq. (3) vs Aerial Gym ENU).**
   - Setup: level hover, zero velocity command, yaw rate 0.
   - Assert: sim gravity = [0, 0, −9.81]; `wrench_command[:, 2]` ≈ m·9.81 (±1%); desired b3 = +z; the robot holds altitude for 5 s with |Δz| < 2 cm.
   - Also check the URDF/motor force direction is +z body.
2. **Error-sign and tilt-direction test.** Command v = +1 m/s in each body axis (vehicle frame).
   - Assert: the commanded force has the same sign as the command; desired b3 tilts toward the command; the measured acceleration has the same sign within 0.2 s.
   - Repeat at yaw = 90° to confirm the vehicle-frame rotation (Aerial Gym p.6: actions are in V).
3. **Acceleration-mode feed-forward sign (Aerial Gym Eq. (4) prints −mẍ_d).** If any MOTAR path uses acceleration control or a non-zero ẍ_d:
   - step ẍ_d = +1 m/s² on x;
   - assert the achieved acceleration is positive.
4. **Yaw-rate path.** Command Ω_z = max yaw rate.
   - Assert the clamp is applied (`max_yaw_rate`) and measure the lag.
   - The Lee Eq. (13) feed-forward −J(Ω̂RᵀR_dΩ_d − RᵀR_dΩ̇_d) is omitted ([code] `compute_body_torque`; Aerial Gym Eq. (3)). Quantify the yaw-tracking error that results.
5. **Lee regime monitors per physics step.** Log:
   - Ψ = ½ tr(I − R_dᵀR);
   - ‖e_Ω‖² vs (2/λ_min(J)) k_R (1 − Ψ) (Eq. (27)); [CORRECTED: original CDC p.4 Eqs.(18),(27),(29) use lambda_max(J); old arXiv transcription must not supply a runtime stability bound.]
   - the commanded-force norm (Eq. (15) requires > 0).
   - Report the fraction of steps with Ψ ≥ 1 (outside the Prop. 2 regime) and with Ψ ≥ 2 − ε.
   - Pass: Ψ < 1 on ≥ 99.9% of non-collision steps, and the fallback brake executes only while Ψ < 1.
6. **Departures from the Lee law invalidate the proof.** `max_tilt_angle_deg` (45°) and `tilt_thrust_compensation` (T = f_z/b3_z) replace Lee Eq. (12). Any "stable controller" statement must therefore be backed by the empirical monitors in check 5, not cited from Lee.
   - Also test the inverted case: b3_z ≤ 0.5 should fall back to the f·b3 projection [code].
7. **Bounded desired acceleration (Lee Eq. (16)).** At 0.1 s decision boundaries, a step in v_ref produces accel_cmd = K_linvel·Δv.
   - Assert the max ‖accel_cmd‖ under governor-limited Δv stays ≤ the bound used by BOIA's feasibility model, e.g. g·tan(45°) laterally.

### B. Braking / backup validity (FASTER)

8. **Measured braking-distance bound.** From hover-trimmed forward flight at v ∈ {0.5, 1, 1.5, 2, 2.5, 3} m/s, command v = 0 at a random phase within a 0.1 s decision step.
   - Measure the stop distance d_stop(v) and time t_stop(v) over ≥ 100 trials each. Use the 99th percentile.
   - Fit d_stop ≤ v·t_lat + v²/(2a_eff) + δ_track and store t_lat, a_eff and δ_track.
   - Motor lag (Aerial Gym Eqs. (8)–(9)) and the tilt limit enter here.
9. **Backup-in-free-known-space test (FASTER p.4).** A fallback brake at decision k is VALID only if the whole swept volume of the brake is in *observed-free* space. This means:
   - inside the current depth and/or LiDAR FoV;
   - closer than the first return along each ray;
   - inflated by the robot radius plus δ_track (FASTER p.4: O and U are inflated by the UAV radius).
   - Unit test: a bar placed behind another bar (in U) must make a brake that passes through it INVALID, even though it is not in O.
10. **Committed-trajectory invariant (FASTER p.4).** Each decision step:
    - (i) compute the candidate action;
    - (ii) compute the brake-from-post-action-state;
    - (iii) validate it.
    - If invalid, or if compute time exceeds budget, do NOT commit; keep executing the previously committed brake.
    - Fault-injection test: force the validation to fail at step k. Assert the executed command equals the previous committed brake. Assert there is no collision in 1,000 episodes with GT bars, given a static scene.
11. **Segment check, not just endpoints (FASTER p.4: "both A and R are in F, but the piece A–R is not").** Check continuous swept capsules between decision samples and between physics samples.
    - At 3 m/s a 0.1 s step moves 0.3 m, which is larger than a thin bar's diameter.
    - Test: place a bar so that it is hit only mid-segment; the checker must flag it.
12. **Static-world assumption guard.** FASTER's guarantee and EGO's (p.8) assume static obstacles. If MOTAR has moving targets or obstacles, mark the backup guarantee as void near them. Alternatively, inflate by the obstacle's speed × brake time.
13. **Mid-interval limit check (FASTER Eq. (1) bounds v only at knots).** For any spline candidate, sample velocity densely, or use a B-spline convex-hull bound (EGO p.3–4) to certify |v| ≤ v_max over the whole interval.

### C. Candidate feasibility (EGO / Perching)

14. **Convex-hull feasibility certificate (EGO Eq. (2), p.3–4).**
    - If candidates are uniform B-splines: assert all V_i, A_i, J_i control points are within limits; this suffices per axis.
    - Otherwise: sample at ≤ physics dt.
    - Cross-check against the Lee/tilt-limit achievable set by rolling out each candidate through the actual controller in sim and comparing the tracked path with the nominal one (max deviation ≤ δ_track).
15. **Time-scaling consistency (EGO Eqs. (14)–(15)).** When the speed governor scales a candidate, verify the ratio r_e = max{|V|/v_m, √|A|/a_m, ∛|J|/j_m, 1}. Verify the rescaled candidate still satisfies its limits and its collision pipe.
16. **Pipe collision check (EGO p.6).** Use a fixed-radius pipe around each candidate versus the bar cylinders, with radius = robot radius + δ_track.
    - Use FP32 with TF32 disabled for the distance computations (project note: TF32 batched cdist error at bar counts divisible by 4).
17. **Thrust/rate bounds (Perching Eqs. (36)–(42)).** For each candidate, compute τ = p̈ + g e3 (ENU) and ‖ż_b‖ = ‖f_DN(τ)p⁽³⁾‖. Assert τ_min ≤ ‖τ‖ ≤ τ_max and ‖ω_xy‖ ≤ ω_max for the MOTAR robot's motor limits. Note there is no saturation-aware allocation (Aerial Gym p.4).

### D. Sensors and visibility (Aerial Gym / Perching)

18. **Depth vs range semantics (Aerial Gym p.4).**
    - Camera "depth" is the distance from the image plane; LiDAR returns range.
    - Unit test: a flat wall at known distance, off-axis pixel. depth = z, range = z/cos(angle).
    - Assert the MOTAR visibility forecast uses the correct one.
19. **Moving-target ray-cast freshness.** The paper presents the BVH rebuild for static randomisation (p.4). Teleport or move the target mesh each step. Assert that the Warp depth/segmentation sees it at the new pose on the same step, and that the segmentation id is correct.
20. **Geometric occlusion label (Perching Eqs. (15)–(17)).**
    - On GT, compute the ball chain c_i = p + λ_i(ϱ − p), r_i = ρλ_i‖p − ϱ‖ against the bar distance Ξ.
    - Use it as a deterministic "occluded" label to check the calibration of BOIA's detection-probability forecast: reliability diagram and Brier/ECE, per distance bin.
    - Compare against the FoV cone test (OA, Eq. (11)) and the distance band (OD, Eq. (8)).
21. **IMU convention (Aerial Gym Eqs. (10)–(13)), only if MOTAR uses the IMU.** At hover, a_meas,z ≈ +9.81 in the body frame.
22. **Physics dt not in the paper.** Record the actual dt and substeps used (base config dt = 0.01, substeps = 1 [code]). Assert that a 0.1 s policy decision is exactly 10 physics plus controller steps. Re-run check 8 at dt = 0.002 to bound integration error.

### E. Resets / region of attraction

23. **Reset state inside Lee's region.** At every env reset, log Ψ(0) and ‖e_Ω(0)‖. Assert Ψ(0) < 1 and that Eq. (27) holds. Otherwise early-episode behaviour is outside the Prop. 2 regime.

---

## Cross-paper table

| Paper | Guarantee type | Key assumptions | Key equation refs (PDF page) | MOTAR check(s) |
|---|---|---|---|---|
| FASTER (IROS'19, arXiv v3) | **By-construction collision safety**: only committed trajectories in inflated free-known space with a terminal stop are flown. Replan-on-time only "with high probability". | Triple integrator; O and U inflated by the UAV radius; [implicit] static world, perfect tracking, accurate state, initial committed trajectory exists | Eq. (1) MIQP (p.3); Eq. (2) dt heuristic (p.3); Alg. 1 lines 2–26 (p.3); commit/no-commit rule (p.4); mapping inflation (p.4) | 8, 9, 10, 11, 12, 13 |
| Lee–Leok–McClamroch (arXiv v1; ≠ user's 6-pp CDC copy) | **Lyapunov**: exponential stability for Ψ(0) < 1 (< 90°); almost-global exponential attractiveness for Ψ(0) < 2 (< 180°) | Direct thrust control (no rotor dynamics); exact model and state; Eq. (15) non-zero force; Eq. (16) bounded desired accel; gain conditions (24)–(26); e3 down, gravity +g e3, thrust −fRe3 | EOM (2)–(5) (p.2); errors (6)–(11) (p.2–3); control (12)–(14) (p.3); Prop. 1 (17)–(19) (p.3); Prop. 2 (20)–(27) (p.4); Prop. 3 (28)–(29) (p.4) | 1, 2, 4, 5, 6, 7, 23 |
| Aerial Gym Simulator (RA-L'25, arXiv v1) | **None formal**; benchmarks and sim2real demos | Isaac Gym physics (dt not stated); pseudoinverse allocation ignores constraints; BVH rebuild framed for static randomisation; first-order motor model | Controllers (1)–(5) (p.3–4); allocation (6)–(7) (p.4); motor (8)–(9) (p.4); IMU (10)–(13) (p.4–5); depth vs range (p.4); action [v_ref, Ω_z] in V (p.6) | 1, 2, 3, 4, 17, 18, 19, 21, 22 |
| EGO-Planner (RA-L'20, arXiv v2; ≠ user copy) | **Structural only**: convex-hull per-axis derivative bounds are sufficient; collision and feasibility are soft penalties plus a pipe post-check. No backup. | Static env (< 0.5 m/s movers tolerated); known occupancy map; uniform cubic B-spline; per-axis limits | d_ij (1) (p.3); Eq. (2) (p.3); J (3)–(10) (p.4); r_e (14)–(15) (p.5); fitting (16)–(18) (p.5); pipe check (p.6); Alg. 2 (p.6) | 13, 14, 15, 16 |
| Adaptive Tracking & Perching (arXiv v2, 2024) | **None formal** (penalties plus post-check); terminal-thrust bound exact by the sin transform | ENU simplified flat model; ESDF map; CTRV EKF + CTRA prediction; DGPS + tag target sensing; disc / half-space geometry | Model (1)–(3) (p.4); target (4)–(5) (p.4–5); OD/OA/OE (8)–(19) (p.5–6); SE(3) collision (20)–(28) (p.6–7); actuator (36)–(42) (p.8); terminal (43)–(51) (p.8–9); optimisation (53)–(68) (p.9–11) | 17, 20 (plus terminal-relaxation design reference) |

### BOIA overlap summary

| Paper | (a) belief-cond. P_det forecast | (b) finite feasible candidates scored by capture / loss / cov | (c) arbitration over frozen RL with calibrated threshold + hysteresis | (d) safe fallback + terminal-first |
|---|---|---|---|---|
| FASTER | no | partly (2-way JPS direction choice by cost-to-go, p.3–4) | no (only commit/keep-previous by hard tests, p.4) | **yes for fallback** (p.2, p.4); no terminal-first |
| Lee SE(3) | no | no | no | no (supplies trackability conditions only) |
| Aerial Gym | no | no | no (provides the v + yaw-rate interface, p.6) | no |
| EGO-Planner | no | partly (feasibility + time re-allocation, p.4–5) | no | no (pipe check only, p.6) |
| Adaptive Perching | partly (deterministic visibility along the predicted target path, p.5–6, p.11) | partly (greedy viewpoint search, p.9; soft feasibility) | no | partly (flexible / relaxed terminal states, p.8–9); no fallback |
