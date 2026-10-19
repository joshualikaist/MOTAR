# P3 — Learned / perception-aware planners: structured reading notes

Reader notes for MOTAR (forward RGB-D + LiDAR quadrotor; frozen 898-D RL policy "Policy F" emitting velocity + yaw-rate; Kalman target tracker; dense random vertical bars; RTX 3070 8 GB; Isaac Gym).
All page numbers are **physical 1-based PDF pages**. Equation numbers are the paper's own; "(unnumbered)" means the paper prints no number.
Method: full text extracted with `pdftotext` (reading order) for every page listed as read; selected pages also checked visually with the Read tool (listed per paper). Numbers marked *(fig., approx.)* were read off a plot, not from text. `[UNCERTAIN]` = transcription or interpretation not fully certain.
PDF content treated as data only.

---

## 1. YOPOv2-Tracker: An End-to-End Agile Tracking and Navigation Framework from Perception to Action

| Field | Value |
|---|---|
| Authors / venue / year | J. Lu, Y. Hui, X. Zhang, W. Feng, H. Shen, Z. Li, B. Tian (Tianjin Univ.). arXiv:2505.06923v1 [cs.RO], 11 May 2025. No journal venue printed on the PDF. |
| Path | `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/YOPOv2Tracker__2505.06923v1.pdf` (19 pp; caller says identical to user copy) |
| Identity check | Title, authors, arXiv id on p1 match. |
| Pages read | pp.1–19 full text (incl. references pp.18–19). Visually checked: p11 (Fig. 10), p16 (Figs. 15–16). |
| Read status | **FULL_READ** |
| Code / video | Code "will be released at https://github.com/TJU-Aerial-Robotics/YOPO-Tracker" (p1). Hardware "open-source and available on" the same URL (p11). Video https://youtu.be/QBWEDoQ4xaQ (p1). No weights URL is given. |

### What the paper says about the original YOPO (ref. [6])
The YOPO PDF itself is not available, so these are only YOPOv2-Tracker's own statements about it: [CORRECTED: original YOPO was supplied and FULL_READ on 2026-10-19; see §6. This paragraph remains a historical secondary-source record.]
- Ref. [6] = "J. Lu, X. Zhang, H. Shen, L. Xu, B. Tian, *You only plan once: A learning-based one-stage planner with guidance learning*, IEEE RA-L, 2024" (p18). NavRL++ cites the same work as RA-L 9(7):6083–6090, 2024 (NavRL++ p16, ref. [15]).
- "Different from our previous work [6], not only the safety and smoothness, but also the tracking performance and objectness scores of predictions are taken into account in the training process" (p2). So YOPO was a navigation-only primitive-offset planner trained on safety and smoothness.
- [6] is grouped with [4] (Loquercio et al.) as an end-to-end policy that plans from the previous *reference* state. "it can be fatal with the tracking error between reference and actual states increasing in high-speed flight" (p2, p7 / Fig. 7). YOPOv2 removes the position controller for this reason.
- "...achieving a faster flight speed compared to our previous work [6] (where the position error exceeds 0.5 m at 5.5 m/s flights)" (pp.16–17).
- YOPOv2's training method extends "traditional gradient-based planners" by back-propagating trajectory-cost gradients to the network (p3). The paper does not say how much of this was already in [6]. Nothing more about YOPO is stated.

### Problem
Agile tracking of a non-cooperative, evasive moving target (a person) by a quadrotor in dense clutter, using only onboard RGB-D and an onboard computer. The aim is to cut the latency of the usual detection → mapping → planning → control pipeline (pp.1–2). The same network can also be reduced to a general high-speed navigation policy (p6, p15).

### Assumptions
- **Sensors:** RealSense D455 RGB-D, 90° HFOV, 16:9 (p6, p11). State from VINS-Fusion VIO (p11). In simulation, odometry comes from the simulator: "localization is not considered" (p12).
- **Map:** none at inference. A privileged ground-truth point cloud and ESDF are used only during training (pp.5, 10; Fig. 3 caption p5).
- **Obstacles:** static clutter (forests, buildings). Sim forest trees follow a homogeneous Poisson process at 1/16 tree/m² (p12). No dynamic-obstacle model.
- **Target:** a single target class ("larger targets such as humans", p6). No motion prediction model, by design (pp.2, 4). The EKF assumes constant velocity with "large process noise" and is used only for filtering, "rather than predicting the future trajectory" (p7, p14). "The only assumption is that the target is visible at the mission start" (p14).
- **Dynamics:** ideal point-mass in differentially flat space. Model mismatch is handled online by a disturbance observer, so there is no domain randomisation (pp.2, 8, 17).

### Pipeline (Fig. 3, p5)
RGB-D (160×96) + normalised (v, a) state → modified ResNet-18 → 5×3 grid of 512-D features → shared 1×1 convs → 5×3×14 predictions.
Post-processing:
1. Drop grids with low objectness.
2. EKF predict/update with a consistency gate (Eq. 10). This is the NMS step.
3. Among the remaining grids, pick the one with the lowest predicted cost y_c.
4. Solve the closed-form quintic for that grid only (Eqs. 2, 4).
5. Discretise the trajectory. Yaw always points at the EKF target estimate; with no detection, yaw points toward where the target was lost (p7).
6. Disturbance observer (Eq. 15) → desired attitude and thrust (Eqs. 11–13) → PX4 attitude controller over MAVROS (p11).

### Key equations
| Eq. | Page | Content | Symbols / assumptions |
|---|---|---|---|
| (1) | p5 | f_µ(t)=a0+a1 t+…+an tⁿ, µ∈{x,y,z} | Three independent one-segment polynomials; yaw handled separately. Quintic used. |
| (2) | p5 | f(t)=t M d (Hermite form) | t=[1,t,…,t⁵]; d=[d_F,d_P]ᵀ; d_F=[f(0),ḟ(0),f̈(0)] (initial state, 2nd-order continuous); d_P=[f(T),ḟ(T),f̈(T)] are the free end derivatives; M depends on the fixed execution time T. |
| (3) | p5 | p_ij=(r cosθ_j cosφ_i, r cosθ_j sinφ_i, r sinθ_j) | Primitive (anchor) endpoints in the camera frame, sampled uniformly over M_θ polar × M_φ azimuth angles within the depth-sensor visible range. r = planning-horizon radius. Higher end derivatives of the anchor are 0. |
| (4) | p5–6 | Refined endpoint with r′=r+Δr, φ′=φ+Δφ, θ′=θ+Δθ | Offsets replace back-end optimisation. |
| (5) | p6 | ḟ_n(t)=ḟ(αt)/(α v_max), f̈_n(t)=f̈(αt)/(α² a_max) | Time-scaling normalisation, so one model serves many speeds; α = ratio of the desired max speed to the training max speed. |
| (6) | p6 | Δθ=tanh(y_θ)d_θ, Δφ=tanh(y_φ)d_φ, Δr=tanh(y_r) r | d_θ, d_φ "slightly larger than the local FOV of grid". |
| (7) | p6 | ḟ(T)=tanh(y_v) α v_max, f̈(T)=tanh(y_a) α² a_max | Constant T=2r/(α v_max). |
| (8) | p7 | u=(σ(y_Δu)+u_grid)d_s, v=(σ(y_Δv)+v_grid)d_s | Per-grid target pixel centre; d_s=32. |
| (9) | p7 | p_w = ʷT_c C⁻¹ y_d p_uv | Image→3D target position: y_d is the predicted target distance (depth), C is the intrinsics, ʷT_c the camera pose from VIO. |
| (10) | p7 | x_temp = x + K(p_w − Hx) | EKF tentative update per remaining detection. H=[I,0]∈R^{3×6}; K is stated as "∈R^{3×6}" (as printed, presumably 6×3). x = position + velocity. Detections with ‖x_temp − x‖ above a threshold are rejected; the one with the smallest inconsistency below the threshold is used for the real update. |
| (11) | p8 | f̈ = (1/m) ʷR_b F_c e_z − g + (1/m) d | Rigid-body translational dynamics; d = lumped disturbance (drag + external). |
| (12) | p8 | R_z=F/‖F‖, R_y=R_z×R_yaw/‖·‖, R_x=R_y×R_z | Flatness map, desired acceleration → attitude. |
| (13) | p8 | F=[f̈_x−d_x, f̈_y−d_y, f̈_z−d_z+g]ᵀ, R_yaw=[cos ψ, sin ψ, 0]ᵀ, F_c = m‖F‖ | Here d enters in acceleration units; Eq. (11) has d/m. Small notational inconsistency in the paper `[UNCERTAIN]`. |
| (14) | p8 | ż₁ = ʷR_b F_c e_z − m g + d, z₁ = m v | |
| (15) | p8 | ẑ̇₁ = ʷR_b F_c e_z − m g + ẑ₂ + (α₁/ζ)(z₁−ẑ₁); ẑ̇₂ = (α₂/ζ²)(z₁−ẑ₁) | High-gain disturbance observer; α₁, α₂>0, ζ≪1. Inputs are only the VIO velocity and the last command. |
| (16) | p8 | L = 1_pos(λ_s J_s+λ_c J_c+λ_g J_g+L_cost+L_tgt+L_obj) + λ₁(1−1_pos)(λ_s J_s+λ_c J_c+L_cost) + λ₂ 1_neg L_obj | λ₁=0.2, λ₂=0.5. Positive / negative / ignored grids. Navigation mode drops L_tgt and L_obj and treats every grid as positive. |
| (17)–(19) | p9 | J_s = dᵀMᵀQMd, block form, ∂J_s/∂d_P = 2d_Fᵀ B_FP + 2d_Pᵀ B_PP | Smoothness = integral of squared higher derivatives (after Richter et al.). |
| (20)–(21) | p9 | J_c = ∫₀ᵀ c(f(t))dt ≈ Σ_κ c(f(κδt))δt; gradient via ESDF ∇c and L_P | c(·) rises sharply near obstacles, e.g. exponential in negative distance. ESDF is privileged (training only). |
| (22)–(23) | p9 | J_g = (d_Pp − g_p)²; ∂J_g/∂d_Pp = 2(d_Pp − g_p) | Tracking: g_p = target position, *unscaled*, so trajectories are longer when the target is far and shorter when it is near. Navigation: goal normalised to a fixed-length direction. |
| (24) | p9 | Chain rule ∂J/∂y = ∂J/∂d_P · ∂d_P/∂Δ · ∂Δ/∂y | Gradients go directly to the network weights: no expert, no critic. |
| (25) | p9 | L_cost = SmoothL1(ŷ_c, y_c) | ŷ_c = λ_sJ_s+λ_cJ_c+λ_gJ_g (navigation) or λ_sJ_s+λ_cJ_c (tracking, since the target may be absent). |
| (26) | p10 | L_tgt = SmoothL1(p̂_c, p_c), p_c = C⁻¹ y_d p_uv | GT relative target position used in training. |
| (27) | p10 | L_obj = BCE(ŷ_o, σ(y_o)) | Positive = the grid containing the projected target; far grids negative; near grids ignored. |
| (28) | p10 | Log-normal pdf for sampled forward velocity, f(v) with (v_m − v) | Lateral and vertical v, a are drawn from a normal fitted to real measurements. Random states are assigned per image at each step. |

### Network and tensor shapes (p6)
- Input: RGB-D resized to **160×96**. How RGB and depth are combined (e.g. channel stacking) is not stated `[UNCERTAIN]`. State: **6-D** (initial velocity + acceleration), normalised by Eq. 5 and rotated into each grid's local frame.
- Backbone: modified ResNet-18, 32× downsampling → **M_φ×M_θ = 5×3 feature map with 512-D features**. Each grid matches one primitive in its frustum.
- Head: state concatenated with features → stacked 1×1 convs, shared weights → **y ∈ R^{5×3×14}**. The 14 channels per primitive are: offsets [y_θ,y_φ,y_r] (3), end derivatives [y_v, y_a] (3+3), cost y_c (1), objectness y_o (1), target [y_Δu, y_Δv, y_d] (3).
- Navigation variant: goal direction added as input, target outputs removed (p6, Fig. 3).
- Not stated: training epochs, dataset size, batch size, training GPU.

### Online inputs vs GT
- **Inference:** RGB-D image, VIO velocity, acceleration and pose (pose for Eq. 9 and the EKF), last command (for the disturbance observer). Evader and environment states are unavailable (pp.12, 14–15).
- **Training:** a pre-collected dataset of poses, images and a point-cloud map (privileged ESDF and point cloud) plus target states. Targets are placed randomly in the FOV and checked for visibility by raycasting (p10). Real-world data: LiDAR-odometry map for GT, targets labelled by YOLO + depth (p10). HSV augmentation on RGB (p10). No simulator interaction during training (pp.3, 10).

### Control interface
Network output → quintic trajectory (differentially flat) → desired acceleration + yaw → minus disturbance-observer estimate → **collective thrust + attitude** to PX4 via MAVROS. The position controller is removed (pp.7–8, 11).

### Timing, compute, hardware
- Platform: 155 mm wheelbase, 850 g, theoretical thrust-to-weight 4.7, NxtPX4 + PX4, Orin NX (8-core A78 at 2.0 GHz, "100 TOPS" GPU) (pp.10–11). Camera tilted 20° (p17).
- Desktop (i7-9700, RTX 3060): **3 ms** perception → command, of which 2 ms is network inference on TensorRT and 1 ms post-processing (p12).
- Baselines: Vis. Tracker 66.7 ms serial, about 50 ms of it mapping/ESDF (p12). Elastic Tracker faster (Fig. 10A).
- Onboard Orin NX: **8.2 ms** average perception → action (p15).

### Experiments and metrics
- **Sim tracking vs Elastic Tracker [1] and Visibility-aware Tracker [2]** (p12). Baselines receive targets from YOLOv5s + depth. Target (evader) speed 3–7 m/s, 10 trials per speed, goal 40 m ahead with a random switch midway; the evader uses Fast-Planner with the GT map.
  - Fig. 10C pie, all tests *(fig.)* (p11):
    - Ours: 88% success / 2% planning failed / 10% target missed.
    - Elastic: 34 / 56 / 10.
    - Vis: 44 / 28 / 28.
  - Text: "without target trajectory prediction, the failures of our method are mainly caused by target occlusion" (p13).
  - Target distribution stays in a 2–6 m band (p13). Only one run had minimum clearance < 0.2 m, at 7 m/s (p14).
- **Large-scale sim, ∞-shaped path, target up to 7 m/s:** target estimation error within ±0.5 m lateral and ±0.75 m longitudinal, including occluded frames (p14).
- **Real world:**
  - Forest of ~3600 m², 3–4 m tree spacing, human target, max 6 m/s (pp.14–15).
  - Abandoned school, 120×120 m², max 5.6 m/s (p15).
- **Navigation mode:** sim forests with 4 m and 5 m average spacing, 4–10 m/s, 10 trials each, vs "Agile" [4] (p15). Results are only in bar plots (Fig. 16, p16, no numbers in the text). Real forests at 8 m/s with goals 90 m and 35 m (p16).

### Limitations
- **Stated:**
  - Occlusion is the main failure cause (p13).
  - No explicit distance constraint to the target's future states, so the target is spread out in the forward direction (p13).
  - Depth camera range is < 10 m and noisy (p11).
  - Assumes the target is visible at the start (p14).
- **Observed:**
  - Very coarse candidate set: 15 primitives.
  - The cost head scores only safety and smoothness in tracking mode, so there is no visibility or occlusion term.
  - The behaviour of the *translational* trajectory when no grid is positive (target lost) is not specified `[UNCERTAIN]`: only the yaw rule is given (p7).
  - Single target class; binary objectness.
  - The EKF is a filter only, with no occlusion-aware covariance use.
  - No safety fallback or emergency stop is described for the method itself. "Planning failed" includes emergency stop for the baselines (p11).

### MOTAR relevance and conflicts
- **Conflict:** YOPOv2 *is* the policy. It replaces perception, planning and control and outputs thrust and attitude. That breaks MOTAR's rule that the frozen velocity/yaw-rate Policy F must not change. Its training needs a privileged ESDF, which is cheap for analytic vertical bars in Isaac Gym but would be a new training effort.
- **Reusable ideas**, without replacing Policy F:
  1. A camera-frustum primitive library (Eq. 3) with time-scaling normalisation (Eq. 5) as a finite, dynamics-feasible candidate set.
  2. A learned per-candidate cost head trained against analytic trajectory costs (Eq. 25). This is a way to calibrate a scorer cheaply.
  3. EKF with large process noise and a consistency gate (Eq. 10). This closely matches MOTAR's Kalman tracker.
  4. "Yaw toward the last-seen direction" on target loss (p7) as a simple baseline.
- **Evidence for BOIA's motivation:** the strongest learned tracker here still fails mainly through occlusion (p13, Fig. 10C), and it has no future-visibility term.
- **Compute:** the ResNet-18 at 160×96 fits easily on an RTX 3070 (3 ms on an RTX 3060, p12).

### BOIA novelty-overlap check
| Item | Verdict | Evidence |
|---|---|---|
| (a) Belief-conditioned forecast of future target detection probability (FOV + LOS + detector reliability) | **No** | Objectness is a current-frame classification (Eqs. 8, 27; p7, p10). The EKF predict step is used for filtering only, not future-visibility prediction (p7, p14). No LOS or occlusion term in the costs (Eqs. 16–22). |
| (b) Finite dynamics-feasible short-horizon candidates scored by capture progress + loss probability + covariance growth | **Partly** | 15 anchor primitives, closed-form quintics from the current state (Eqs. 2–7). Selection uses the predicted safety+smoothness cost after objectness filtering (p7, Fig. 5). The goal/target cost J_g (Eq. 22) is a capture-progress analogue, but only in training; it is excluded from y_c in tracking (p9). No loss-probability or covariance terms. |
| (c) Arbitration overriding a frozen RL policy only when calibrated expected advantage > threshold, with hysteresis | **No** | Single end-to-end policy. No arbiter. |
| (d) Safe fallback + terminal-first handling | **No / minimal** | Only the yaw rule on missed detection (p7). No safe fallback or terminal logic is described. |

---

#### 2026-10-19 original-copy re-read (Codex)

**Identity and scope.** User original `YOPOv2-Tracker An End-to-End Agile Tracking and Navigation.pdf`, SHA-256 `a7e0d432aed87849…`, 19 physical pages, is byte-identical to the previously read copy (both payload hashes verified). Prior **FULL_READ pp.1–19 is carried forward**, not claimed as a second full read. New random equation spot-checks use `random.Random(20261019)`: p.8 Eq.(16), p.6 Eq.(5), p.9 Eq.(25). All three agree with the earlier notes. Additional mandatory checks: pp.6 and 13. No external code executed.

- **Confirmed:** p.6 gives the 5×3×14 output and 6-D velocity/acceleration input, with RGB-D resized to 160×96. The channels are 3 endpoint offsets + 3 endpoint velocities + 3 accelerations + cost + objectness + 3 target-image/depth values.
- **Confirmed:** p.13 explicitly attributes the main failures to target occlusion without target-trajectory prediction. This is the authors' stated limitation, not evidence that BOIA repairs it.
- Eq.(16) includes positive/negative/ignored-grid masks with λ₁=0.2 and λ₂=0.5 (p.8); Eq.(5) uses α and α² velocity/acceleration scaling (p.6); Eq.(25) supervises **positive cost**, unlike original YOPO's negative-cost score (p.9).
- **Unresolved source notation:** p.8 Eq.(11) defines disturbance as force (`d/m`), whereas Eq.(13) subtracts `d` as acceleration. A consistent implementation must choose units explicitly; the PDF does not resolve the mismatch. No BOIA force-level controller is being implemented.
- **BOIA overlap:** (a) **no** future belief-conditioned detection forecast (p.13); (b) **partial**, finite learned trajectories with cost selection (pp.6–7); (c) **no**, its own policy replaces planning/control rather than arbitrating Policy F (p.8); (d) **partial** short EKF/yaw loss handling, without terminal-first or a proven stop (p.7); (e) **no** collision-competing 2-second recovery evaluation (pp.12–14). No design-formula invalidation found; G5 detector/dynamics checks remain necessary.

## 2. NavRL++: A System-Level Framework for Improving Sim-to-Real Transfer in Reinforcement Learning-Based Robot Navigation

| Field | Value |
|---|---|
| Authors / venue / year | Zhefan Xu, Hanyu Jin, Kenji Shimada (CMU). arXiv:2605.15559v1 [cs.RO], 15 May 2026 (p1). No venue printed. |
| Path | `/home/fair/Downloads/NavRL++: A System-Level Framework for Improving Sim-to-Real Transfer in Reinforcement Learning-Based Robot Navigation.pdf` (18 pp) |
| Identity check | Title, authors and arXiv id match (p1). Baseline "NavRL [16]" = Xu et al., *NavRL: Learning safe flight in dynamic environments*, RA-L 10(4):3668–3675, 2025 (p16). |
| Pages read | pp.1–18 full text (incl. references). Visually checked: p10 (Tables II–IV), p12 (Table V/VI, Figs. 10–11). |
| Read status | **FULL_READ** |
| Code | "The framework will be released on GitHub" (p2). **No URL given.** |

### Problem
RL local navigation (collision avoidance + goal reaching) with zero-shot sim-to-real transfer on UAV and quadruped. The paper systematically breaks down how sensing noise, perception failure, latency and control-response mismatch affect performance (pp.1–3).

### Assumptions
- **Sensors:** Livox Mid-360 LiDAR + RealSense D435i RGB-D on every platform (Fig. 2, p2; p10). Three modes: camera-only, LiDAR-only, fusion (p4). State estimation from VINS-Mono [63] or FAST-LIO2 [64] (pp.4, 10).
- **Map:** local voxel occupancy map with log-odds update (Eq. 15, p8). The policy consumes ray-cast distances from it, not raw sensor data (pp.4–5).
- **Obstacles:**
  - Static: arbitrary geometry, held in the voxel map.
  - Dynamic: rigid bodies with position, velocity and radius. "We assume that dynamic obstacles move only within the planar space" (p8). At most N=5 dynamic obstacles are fed to the policy (p5).
- **Target:** none. Navigation is to a user goal or global-planner waypoint (Alg. 1, p9).

### Observation design (pp.5, 6)
| Part | Eq. | Content |
|---|---|---|
| Static | (2) p5 | Egocentric ray casting into the voxel map: horizontal 10° steps (36 beams) × vertical −10° to 20° in 10° steps (4 beams) = **144 distances**. Max range 4 m; distances are encoded relative to the limit, and out-of-range rays become 0 after normalisation (exact normalisation formula not given `[UNCERTAIN]`). |
| Dynamic | (3) p5 | S_dyn = [S_d,1…S_d,5], S_d,i = [o_i^{t−K}…o_i^t], o = [p, v, r]. **2 s history at 0.5 s** spacing. Per-obstacle dimensionality of p and v inside the policy is not stated explicitly; the perception feature in Eq. 16 is 2-D p, v `[UNCERTAIN]`. |
| Internal | (4) p5 | S^t = [unit goal direction, goal distance, robot velocity] in a **goal-aligned frame** (x from start to goal, z vertical). History of K steps (2 s / 0.5 s). |

### Temporal reasoning network (p6)
- Fixed **12 tokens**: 1 [CLS] + 5 dynamic-history tokens + 5 internal-state-history tokens + 1 static token. Positional encodings are added.
- Static encoder: CNN with 3 layers (channels 4, 16, 16) + flatten. Token-projection MLP [128, 64] into a shared embedding.
- Transformer: d=64, 4 heads, 4 encoder layers, FF 128, dropout 0.1.
- Actor and critic MLPs: 2×256 each, fed from the [CLS] feature. **0.31 M parameters total.**

### Action and control interface (p5)
- Eq. (5): v_i = v_max(2 v̂_i − 1). Eq. (6): Beta(α, β) per axis. Actions are sampled in training; the **mean is used at deployment**.
- **Velocity only**: 3-D for UAV, 2-D for quadruped, in the goal-aligned frame. No yaw or yaw-rate output. The paper says the parameterisation "can be adapted" for linear speed + yaw rate (p5).
- Velocity is executed by the low-level controller (p6).
- Note: the real-world deployment uses "the same trained policy (2D output)" on both UAV and quadruped (p10).

### Reward (pp.5–6), Eq. (7) = weighted sum
| Term | Eq. | Weight | Definition |
|---|---|---|---|
| Velocity | (8) | λ1=1.0 | Goal unit vector · v_r |
| Survival | (9) | λ2=1.0 | Constant 1 per step |
| Static clearance | (10) | λ3=2.0 | Mean log of the ray distances |
| Dynamic clearance | (11) | λ4=2.0 | Mean log of the distances to dynamic obstacles |
| Smoothness | (12) | λ5=0.1 | −‖v_t − v_{t−1}‖ |
| Height | (13) | λ6=8.0 | Penalty for leaving the [start, goal] height band; exact printed form `[UNCERTAIN]` |

Eq. (7) misprints λ6 for the smoothness term.

### Training curriculum and perturbations (pp.6–7, 9–10)
- **Pre-training:** Isaac Sim, 1024 parallel UAVs, PPO + GAE, Adam lr 2×10⁻⁴, entropy bonus (Eq. 14). Environments use box and cylinder static obstacles. Dynamic obstacles move at U[0, 1.5] m/s and change direction every U[0, 2] s.
- **Curriculum (Table I, p7):**

  | Stage | Static | Dynamic |
  |---|---|---|
  | S1 | 300 | 60 |
  | S2 | 350 | 80 |
  | S3 | 400 | 100 |
  | S4 | 400 | 120 |
  | S5 | 400 | 140 |

  Pre-training uses S1–S4; post-training uses S1–S3.
- **Perturbation-aware fine-tuning** (lr 5×10⁻⁵):
  - (a) Zero-mean Gaussian input noise on all states, "empirically determined" std (values not given).
  - (b) **Perception failure**: Bernoulli drop of detected dynamic obstacles, p_drop = 0.3.
  - (c) Input latency U[0, 0.1] s.
  - (d) Action latency U[0, 0.04] s.
  - (e) Velocity-controller gain randomised to 60–140%: rise time 1.12–0.32 s, settling time 2.14–0.96 s.
- **Goal randomisation:** goals sampled every episode, even inside obstacles (p13).
- **Compute:** S1 ≈ 80 GPU-h on an RTX 4090; S2–S4 ≈ 20–30 GPU-h each; fine-tune stages 10–15 GPU-h each. Total ≈ **200 GPU-h**, more than 80,000 h of aggregate experience (pp.9–10).

### Deployment: perception, safety layer, tasks (pp.8–9)
- **Dynamic obstacles:**
  - LiDAR: downsample → DBSCAN → AABB.
  - RGB-D: U-depth map detector + clustering, combined by ensemble agreement.
  - Association: constant-velocity prediction + cosine similarity on [p, v, r] (Eq. 16), then a **Kalman filter**.
  - Classified dynamic by a velocity threshold, or by a 2-D detector class when the object is in the camera FOV.
- **Safety shield (optional):** velocity-obstacle (VO) projection, Eq. (17a–d): min ‖v_safe − v_π‖² subject to half-space constraints (v_safe − (v_π + δv_i))·δv_i ≥ 0 and box limits. Static rays are converted to position–radius obstacles with zero velocity. **If infeasible, the robot defaults to zero velocity** (p9).
- **Task integration (Alg. 1):** a global planner (exploration [71] or inspection) gives goals; the RL policy does local navigation.
- **Runtime:** policy at **20 Hz, 4.1 ms average inference onboard** (p14). The onboard computer model is not named.

### Experiments and metrics (numbers with context)
- **Table II** (p10). Isaac Sim with perturbations at evaluation, 10,000 trials per cell, v_max 2 m/s. Combined SR / CE (m/s²):

  | Variant | SR (%) | CE |
  |---|---|---|
  | NavRL | 63.05 | 0.050 |
  | NavRL-IT | 92.85 | 0.093 |
  | NavRL-IT-T (transformer) | 90.54 | 0.043 |
  | NavRL-IT-PF (conv + fine-tune) | 77.30 | 0.081 |
  | **NavRL++** | **94.08** | 0.048 |

  High-complexity dynamic SR: NavRL++ 83.96 vs NavRL 38.35.
- **Fig. 9** (pp.11–12). Clean-trained NavRL-IT-T under single perturbations:
  - Perception failure costs > 5% SR. Other single factors cost about 1–3%.
  - Combined perturbations cost > 10% (> 15% at high density) and raise CE about 2.5×.
- **Table IV** (p10). NavRL++ overall SR:
  - By speed: 1.5 m/s 90.43; 2.0 m/s 94.08; 2.5 m/s 94.68; 3.0 m/s 93.25.
  - By control rate: 2 Hz 84.69; 5 Hz 91.06; 10 Hz 93.20; 20 Hz 94.03; 50 Hz 94.08 (50 Hz is the shared baseline configuration).
- **Table V** (p12). Gazebo cross-simulator, 100 trials, SR low / mid / high:

  | Setting | Method | Low | Mid | High |
  |---|---|---|---|---|
  | Static | NavRL++ | 100 | 99 | 100 |
  | Static | EGO-Planner | 99 | 100 | 99 |
  | Static | Fast-Planner | 99 | 100 | 96 |
  | Dynamic | NavRL++ | 94 | 85 | 81 |
  | Dynamic | ViGO | 71 | 70 | 57 |
  | Dynamic | NavRL | 68 | 65 | 50 |

  Trajectory effort for NavRL++ is 0.130 (static), higher than EGO's 0.080–0.082.
- **Curriculum ablation:** +1% SR in pre-training, about +10% in fine-tuning, and > 15% in dynamic environments (p13, Fig. 11).
- **Real world:** UAV and quadruped among pedestrians; exploration and inspection tasks. Qualitative only (pp.14–15).

### Limitations
All stated (pp.14–15):
1. Rule-based dynamic detection misses objects and is "sensitive to occlusion".
2. Action smoothness still trails optimisation planners.
3. Non-negligible collisions in dense dynamic scenes.
4. The VO shield is over-conservative and can leave the robot stationary.
5. Oscillation in narrow spaces (< 0.1–0.2 m clearance), attributed to the sparse ray representation.

### MOTAR relevance and conflicts
This paper is about MOTAR's frozen Policy F lineage, which is NavRL-derived, transformer-based and fuses RGB-D + LiDAR.
- **Overlaps**, i.e. prior art as of 15 May 2026, that MOTAR must cite and must not claim as novel:
  - Transformer temporal policy over static, dynamic-obstacle and internal-state histories, with a [CLS] token.
  - Unified camera / LiDAR / fusion input via occupancy map + ray-cast distances.
  - Perturbation-aware fine-tuning: noise, perception dropout, input and action latency, controller-gain randomisation.
  - Curriculum on obstacle density.
  - Kalman-tracked dynamic obstacles.
  - A post-hoc safety layer overriding the RL velocity.
- **Differences** I can confirm from this paper alone:
  - NavRL++ outputs velocity only, in a goal-aligned frame, via Beta-mean.
  - MOTAR's Policy F outputs velocity + yaw-rate, per the brief.
  - NavRL++ trains in Isaac Sim with 1024 envs on an RTX 4090 for ~200 GPU-h. MOTAR uses Isaac Gym on an 8 GB RTX 3070.
  - NavRL++ has no target or tracking objective and no notion of target visibility.
- **898-D decomposition:** I cannot map Policy F's 898-D observation onto NavRL++'s tokens from this PDF (not in scope). Any claimed equivalence needs a code check.
- **Useful for BOIA:**
  - The p_drop = 0.3 perception-failure model and latency ranges are empirical anchors for a detector-reliability prior.
  - The VO shield (Eq. 17) is a clean template for the "safe fallback" layer.
  - The 2-Hz to 50-Hz sensitivity (Table IV) bounds how slow an arbiter decision period can be before the base policy degrades.
- **Hazard:** any MOTAR contribution phrased as "making a NavRL-style transformer policy robust to latency, perception dropout or sensor fusion" is now directly anticipated by this paper.

### BOIA novelty-overlap check
| Item | Verdict | Evidence |
|---|---|---|
| (a) | **No** | No target. Perception failure appears only as training-time dropout (p7), not as a forecast. |
| (b) | **No** | Single stochastic policy action (Eqs. 5–6). The shield is a QP projection, not candidate scoring (Eq. 17). |
| (c) | **Partly (weak)** | A layer overrides the RL command at deployment (Eq. 17, pp.8–9), but only for safety (VO constraint violation) with a minimum-deviation projection. No calibrated advantage, no threshold or hysteresis. |
| (d) | **Partly** | Fallback to zero velocity when the shield QP is infeasible (p9). No terminal-first logic. |

---

#### 2026-10-19 original-copy re-read (Codex)

**Identity and scope.** User original `NavRL++ A System-Level Framework for Improving Sim-to-Real.pdf`, SHA-256 `4d79836c6854ffa…`, 18 physical pages, is byte-identical to the previously read copy (both payload hashes verified). Carry forward prior **FULL_READ pp.1–18**; newly inspected pp.1–2,5–9,16. Random spot-checks: p.8 Eq.(15), p.8 Eq.(16), and Eq.(8), which is physically on **p.6, not p.5**. Their algebra agrees with the earlier notes; the page correction is explicit. Author names and arXiv v1 date remain as printed on p.1.

- **Confirmed:** p.7 randomly drops detected dynamic obstacles with Bernoulli `p_drop=0.3` in perturbation-aware post-training. This is not a predicted target-detection probability and does not validate temporal independence of misses.
- **Confirmed:** pp.8–9 Eq.(17) projects the RL velocity onto VO half-spaces, with zero velocity when infeasible. This is an existing safety override over an RL policy; the broad claim that every policy-external safety/fallback layer is new is untenable. It has no BOIA expected-advantage threshold/hysteresis or target recovery objective.
- Random checks: Eq.(15) is the recursive voxel log-odds update (p.8); Eq.(16) is `[p_i,v_i,r_i]` with **2-D** position and velocity (p.8); Eq.(8) is the goal-direction dot velocity reward with λ₁=1 (p.6).
- **Code availability:** p.2 promises future GitHub release; no repository URL appears in this PDF. Current remote availability is **NOT_CHECKED in this original-copy audit**, not inferred from the promise.
- **BOIA overlap:** (a) **no** (p.7 training dropout); (b) **no** finite trajectory scoring (pp.5,8); (c) **partial**, RL safety shield only (pp.8–9); (d) **partial**, zero-velocity infeasibility fallback (p.9), no terminal-first contract; (e) **no**, navigation success/collision rather than occlusion-triggered recovery (pp.9–10). No change to Policy F or retraining follows from this reading.

## 3. PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments

| Field | Value |
|---|---|
| Authors / venue / year | J. Tordesillas, J. P. How (MIT ACL). arXiv:2103.06372v3, 22 Mar 2022 (p1). IEEE Access template. The published version is IEEE Access vol. 10, pp. 22662–22677, 2022, per Deep-PANTHER ref. [7] (Deep-PANTHER p8). |
| Path | `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/PANTHER__2103.06372v3.pdf` (16 pp) |
| Pages read | pp.1–14 full body text (incl. conclusion); references pp.14–16 skimmed. Figures not visually inspected; numbers come from the text. |
| Read status | **METHOD_REVIEWED+** (all body text read; figures not inspected) |
| Code / video | https://github.com/mit-acl/panther ; https://youtu.be/jKmyW6v73tY (p1) |

### Problem
Real-time perception-aware trajectory planning for a multirotor with a limited-FOV camera, avoiding **unknown dynamic obstacles** while keeping them in the FOV with low image blur, to help tracking and prediction. Translation and yaw are optimised **jointly** (pp.1–3). The paper explicitly sets its goal (B.3 "avoidance of dynamic obstacles") apart from chasing or recording a target (B.2) (pp.2–3).

### Assumptions
- **Sensors:** forward depth camera (D435i), IMU, downward monocular camera for VIO (pp.1, 11, 14).
- **Map:** none. Obstacles are point-cloud clusters with AABBs.
- **Obstacles:** dynamic, tracked by clustering + Hungarian assignment. Future motion comes from a polynomial fit with Gaussian prediction intervals: mean from the polynomial, diagonal covariance (p4).
- **No target.** Simulations assume perfect tracking of the planned trajectory (pp.8, 10). Single-obstacle simulations give the agent the exact obstacle trajectory (p10).

### Pipeline (p4, Fig. 2A)
1. Tracker / predictor.
2. Select one obstacle i* for the PA term using a Boole-inequality collision-probability heuristic (p4, unnumbered):
   i* = argmax_i Σ_u P(‖p_i(t_u) − κ(u)‖_∞ ≤ R).
   i* is used only if the angle between the goal direction and the obstacle is < α₀ ≈ 90°; otherwise the FOV is aligned with the direction of travel.
3. Octopus Search Algorithm: position guess + separating planes between MINVO hulls (pp.4, 6).
4. ψ guess by Dijkstra over a layered (p, ψ) graph (p6).
5. Nonconvex joint (p, ψ) optimisation with IPOPT/CasADi (p8).

### Key equations
| Eq. | Page | Content | Notes |
|---|---|---|---|
| inFOV | p3 (Table 2) | 1(p ∈ FOV) ≈ σ(γ(−cos(θ/2) + (p^c)_z/‖p^c‖)) | Cone FOV with opening θ; sigmoid smoothing. **FOV only, no line-of-sight check.** |
| ψ-graph edge cost | p6 (unnumbered) | c_ψ wrap(Δψ)² + c_Ψmax·1(\|wrap Δψ\|/Δt > Ψ_max) + c_FOV(1 − inFOV(T_n2, p_i*(t_n2))) | Dijkstra with early termination. |
| (1) | p6 | ψ spline least-squares fit with ψ(t_in), ψ̇(t_in) fixed and ψ̇(t_f)=0 | Solved via linear KKT. |
| (2) | p6 | rot(q_b^w) e_z = ξ̄ | Underactuation constraint; ξ = a + g e_z. |
| Hopf map | p7 (Table 3, Def. 3) | q_b^w = q_ξ ∘ q_ψ | Singularity only when inverted (ξ̄ = −e_z). Embedded directly in the joint optimisation, so dynamics are satisfied by construction. |
| PA reward | pp.7–8 | inFOV(·) / (ε₁ + ε₂‖ṡ‖²), s(t) = f [p̃^c]_{x:y} / [p̃^c]_z | ṡ = projected velocity on the image plane, used as a blur proxy. |
| Optimisation | p8 (printed with "(3)" `[UNCERTAIN]`) [CORRECTED: the p.8 boxed optimization is unnumbered; original-copy visual check] | min α_j∫‖j‖² + α_ψ∫ψ̈² − α_FOV∫ inFOV/(ε₁+ε₂‖ṡ‖²) + α_g‖p(t_f)−g‖² | **Constraints:** x(t_in) = x_in; v(t_f) = a(t_f) = 0 and ψ̇(t_f) = 0 (terminal hover); plane separation nᵀq + d < 0 for all MINVO control points; v, a, j and Ψ bounds. **Decision variables:** p is a cubic B-spline, ψ a quadratic B-spline. |
| Collision | p8 | Obstacle MINVO hull inflated by norminv(δ)·σ_i(t_end,j) + half the obstacle and agent AABBs | Chance-constraint-like inflation via the prediction covariance. |

### Online inputs vs GT
- Hardware: everything is estimated onboard (clusters, tracks, VIO).
- Simulation: obstacle info is restricted to obstacles inside the FOV (corridor test, p10), but the single-obstacle and compute tests use known obstacle trajectories (pp.10–11).

### Control interface
p(t), ψ(t) → Hopf-based controller [36],[63] at 100 Hz → desired orientation and angular rates → geometric controller at 500 Hz → motor thrusts (p14).

### Timing, compute, hardware
- Onboard NUC i7DNK replanning: **48.70 / 51.66 / 58.59 ms** for 1 / 2 / 3 obstacles. Tracker about 8.6 ms per point cloud; depth point cloud at 90 Hz (p14).
- Components (sim, i9-9900K): LP 0.09 ms each, Dijkstra 0.137 ms, ψ-fit 0.048 ms. Totals grow roughly linearly with obstacle count, tested 4–20 obstacles (p11, Fig. 4).

### Experiments and metrics
- **Single obstacle** (120×120 px, 60°×60°, 60 Hz):
  - Obstacle in FOV **7.9×** (vs no-PA) and **1.5×** (vs PA-decoupled) more often.
  - Mean projected velocity −18% and −34%.
  - Mean continuous-detection length 155 frames vs 39 and 46 (p10).
- **Corridor**, 5 obstacles, FOV 70°×70°, 5 m range, v_max 2.6 m/s, 30 runs, constant-velocity predictor:
  - Slow obstacles (≤ 2.12 m/s): PANTHER **87%** success, others < 47%.
  - Fast obstacles (≤ 4.07 m/s): **70%**, others < 27% (pp.10–11).
- **Hardware:** 9 experiments with 1–3 obstacle UAVs. Max 5.77 m/s, relative speed up to 6.28 m/s (p14).

### Limitations (stated, p14)
1. Choosing which obstacle to keep in view (the exploration–exploitation trade-off).
2. Visibility vs time-optimality.
3. Disturbances and tracking-error bounds are not handled.

Observed:
- Visibility = FOV cone only. Occlusion of the obstacle by other obstacles is not modelled.
- Only one obstacle in the PA term.
- Optimisation-heavy (~50 ms CPU).

### MOTAR relevance and conflicts
- **Conflict:** PANTHER outputs a full SE(3) trajectory to a geometric controller, so it cannot sit "inside" a frozen velocity/yaw-rate policy. In MOTAR it could only serve as an offline expert, a candidate generator, or a fallback planner.
- **Ideas that transfer:**
  - The sigmoid in-FOV term + image-plane blur term (pp.3, 7–8) as a differentiable proxy for detector reliability.
  - Collision constraints inflated by prediction-covariance percentiles (p8).
  - The "include the obstacle in the PA term only when roughly ahead (α₀)" heuristic (p4), which is a hand-tuned arbitration between perception and progress.
- **Scaling to dense vertical bars:** cost scales linearly with the number of clustered obstacles (p11), so dense bars would be costly. Static bars would come out as clusters with near-zero predicted velocity.

### BOIA novelty-overlap check
| Item | Verdict | Evidence |
|---|---|---|
| (a) | **Partly (FOV-only)** | Integrated future in-FOV probability-like sigmoid of the predicted (Gaussian-mean) obstacle position plus a blur penalty (pp.3, 7–8). No LOS occlusion and no detector-reliability model. The object being perceived is an obstacle, not a target. |
| (b) | **No** | Continuous nonconvex optimisation from one initial guess. Only the ψ initial guess uses a discrete graph (p6). No capture or covariance scoring. |
| (c) | **No** | |
| (d) | **Partly** | Every plan ends in hover (v = a = 0, ψ̇ = 0 at t_f; p8), a terminal-safe plan. No explicit fallback logic in the pages read. |

---

#### 2026-10-19 original-copy re-read (Codex)

**Receipt.** Jesus Tordesillas and Jonathan P. How, *PANTHER: Perception-Aware Trajectory Planner in Dynamic Environments*. User original `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/PANTHE Perception-Aware Trajectory.pdf`, SHA-256 `c0f83fff0ac6f54f…`, 16 physical pages; byte-identical to the previous copy. Newly read **all pp.1–16**, including references and biographies; **FULL_READ**. Page 8 was rendered to verify the previously uncertain equation numbering. The p.1 arXiv stamp is v3, 22 March 2022; IEEE Access template publication fields are placeholders, not reliable publication dates. No appendix is included.

**Problem, assumptions and pipeline.** This is avoidance of dynamic obstacles, not a moving-target pursuit system (pp.2–3). A forward depth camera, IMU and downward VIO camera provide onboard sensing; no prior obstacle shape/trajectory is available in hardware (pp.11,14). Depth-cloud Euclidean clusters/AABBs → Hungarian association → polynomial prediction with diagonal Gaussian prediction intervals → select one obstacle for the perception cost → OSA position guess/separating planes → Dijkstra yaw guess → coupled position/yaw NLP (pp.4–8). All tracked obstacles remain in collision constraints, though only one is in the PA reward (p.4). Prediction horizon is selected through the planned spline interval; a single universal numerical horizon is **NOT_RECORDED** in the method definition.

**Equations and decisions.**

| Original reference | Verified content and assumptions |
|---|---|
| p.3 Table 2 | `inFOV ≈ sigmoid(γ[−cos(θ/2)+(p_c)_z/‖p_c‖])`: a cone-angle gate, **not obstacle LOS or calibrated detection probability**. |
| p.4 §II-B, unnumbered | Select maximal summed collision-probability heuristic; include its PA term only if goal/obstacle angle is below α₀ (typically ≈90°), otherwise point toward travel. |
| p.6 Eq.(1) | Least-squares yaw-spline fit, initial ψ/ψ̇ and terminal ψ̇=0 fixed. |
| p.6 Eq.(2); p.7 Table 3 | `rot(q_b^w)e_z=ξ/‖ξ‖`, ξ=a+ge_z; Hopf map embeds underactuation, singular only for inverted thrust direction. |
| p.8 boxed optimization, **unnumbered** | Position jerk + yaw acceleration − `inFOV/(ε₁+ε₂‖ṡ‖²)` + goal error. Initial state fixed; terminal v=a=ψ̇=0; MINVO hull separation and v/a/j/yaw-rate limits. The old proposed Eq.(3) citation is incorrect. |

The blur term uses projected image-plane speed, not a measured detector calibration. Obstacle predictions are inflated by a chosen normal percentile and AABB half-sizes (p.8). Ipopt success includes both Solve_Succeeded and Solved_To_Acceptable_Level (p.8 footnote 7). There is no finite Policy-F-inclusive candidate set, advantage threshold or hysteresis.

**Online/GT boundary and execution.** Hardware uses onboard estimated point clouds/tracks/VIO. Single-obstacle simulation and compute scaling explicitly supply exact future obstacle trajectories; all simulation planners perfectly track their planned trajectories (pp.8,10–11). These are limitations of the evidence. Position/yaw splines feed a Hopf-based controller at 100 Hz and geometric attitude controller at 500 Hz (p.14), incompatible with direct substitution into frozen actor898.

**Cost and evidence.** Onboard NUC i7DNK mean replanning is 48.70/51.66/58.59 ms for 1/2/3 obstacles; tracker/predictor ≈8.6 ms per point cloud (p.14). No neural model/parameter count applies. The i9-9900K simulation study reports 155 vs 39/46 frames mean continuous FOV detection for coupled vs no-PA/decoupled PA, and 87%/70% corridor success with slow/fast obstacles, 30 runs per algorithm (pp.10–11). Nine hardware experiments reach 5.77 m/s and 6.28 m/s relative speed (p.14). These are paper-only evidence, not MOTAR performance comparisons.

**Limits and reuse.** Authors identify obstacle-selection exploration/exploitation, visibility/time-optimality and disturbance tracking bounds as unresolved (p.14). Observed: geometric in-FOV is not actual occlusion-aware redetection, and simulation perfect prediction/tracking restricts applicability. Reuse only the FOV/blur intuition, prediction-aware constraints and explicit terminal hover; BOIA needs sensed LOS and its own measured dynamics. Code/video URLs printed on p.1: https://github.com/mit-acl/panther ; https://youtu.be/jKmyW6v73tY . Weights are **NOT_APPLICABLE** to this optimization planner.

| BOIA element | Overlap | Physical-page evidence |
|---|---|---|
| (a) | partial | Predicted obstacle FOV/blur, no LOS or detector reliability, pp.3,7–8. |
| (b) | partial | Dynamically bounded continuous trajectory optimization with progress/smoothness, not the complete finite scoring design, pp.6–8. |
| (c) | no | No frozen RL baseline/advantage arbitration, pp.4–8. |
| (d) | partial | Terminal hover constraint, not stop-distance proof or simulator terminal-first, p.8. |
| (e) | no | Continuous-detection/navigation study, not competing-risk target recovery, pp.10–14. |

No invalidation of BOIA's stated equations was found. Correct citations and keep the source's assumptions; no change to 07–09 was made.

## 4. Deep-PANTHER: Learning-Based Perception-Aware Trajectory Planner in Dynamic Environments

| Field | Value |
|---|---|
| Authors / venue / year | J. Tordesillas, J. P. How (MIT). IEEE RA-L, accepted Jan 2023 (p1 header). Published as RA-L 8(3):1399–1406, 2023, per YOPOv2 ref. [12] (YOPOv2 p18). arXiv:2209.01268v2, 14 Feb 2023. |
| Path | `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/DeepPANTHER__2209.01268v2.pdf` (9 pp) |
| Pages read | pp.1–8 full text (body + references start). Visually checked: p6 (Figs. 5–7). p9 (references end) not read. |
| Read status | **METHOD_REVIEWED+** (entire body read) |
| Code / video | https://github.com/mit-acl/deep_panther ; http://youtu.be/53GBjP1jFW8 (p1) |

### Problem
Compress the PANTHER-style joint perception-aware planner into a fast multimodal neural policy through imitation learning, keeping the *multiple* locally optimal trajectories (p1).

### Assumptions
- One dynamic obstacle in the observation. With several obstacles, the one with the highest collision probability is chosen (p7).
- The predicted obstacle trajectory is given as a spline. **In all simulations it is fitted to ground-truth future positions** (p5).
- Simulation only; no real-world experiments (p8 future work).

### Expert / student
- **Expert PANTHER\*** (p3) adds two decision variables to PANTHER: the separating planes and the total time T (0 ≤ T ≤ T_pred). It runs n_runs = 10 times from OSA initial guesses and keeps the best n_e = min(n_sols, n_s) distinct solutions.
- **Student:** fully connected MLP, **2 hidden layers × 64, ReLU** (p3).

### Observation and action (p3–4)
- **Observation**, **43-D**, expressed in frame f (yaw-aligned, gravity-aligned):
  - v_f (3), a_f (3), g_f (3, projected goal), ψ̇ (1)
  - Q_p,obst: 10 control points × 3 = 30 (obstacle prediction spline in S³_{3,13})
  - s_obst (3, AABB sides)
- **Action**, **13·β**: for each trajectory, the interior position B-spline control points (all except the first 3 and last 2, which are fixed by the initial/final conditions and T) plus T. Trajectories are in S³_{3,12}.
  - Student β = n_s = 6, so the output is **78-D**.
  - Every student trajectory meets the initial and final conditions by construction and is C²-smooth (p4).

### Key equations
| Eq. | Page | Content |
|---|---|---|
| LSA loss | p4 (unnumbered) | L = 1ᵀ(β_p A⊙D_p + β_T A⊙D_T)1. A comes from linear-sum assignment (Jonker–Volgenant) on D_p, so each expert trajectory gets a distinct student trajectory. β_p = β_T = 1 (p5). |
| (1) | p5 | b₁ = ((p_obst − p) − ((p_obst − p)ᵀξ / ‖ξ‖²) ξ)_n. Closed-form body x-axis maximising the obstacle's presence in the FOV, given the position trajectory (cone FOV: cos(θ/2) ≤ b₁ᵀ(p_obst − p)_n). Singular when (p_obst − p) ∥ ξ (footnote 3). |
| Cost | p5 (footnote 4) | c_obj = α_j∫‖j‖² + α_ψ∫ψ̈² − α_FOV∫inFOV³ + α_g‖p(t_f) − g‖² + α_T T `[UNCERTAIN: exponent "3" on inFOV as printed]` [CORRECTED: genuine cubic exponent verified visually, original p.5 footnote4] |
| Selection | p5 | Choose the collision-free candidate with the smallest c_obj + λ c_dyn_lim, where c_dyn_lim is a soft penalty on v/a/j violations. **If none is collision-free, keep executing the previous (collision-free) trajectory and replan.** |

### Online inputs vs GT
- **Inference:** UAV state, goal, predicted obstacle spline, obstacle size. The predictor is supposed to come from past observations as in PANTHER, but the evaluation uses a GT future fit (p5).
- **Training:** expert labels (PANTHER\*).
  - Static case: 2K pairs, 75% used for training.
  - Dynamic case: DAgger with about 23K pairs on randomised trefoil-knot obstacle trajectories. Adam, lr 10⁻³, T_pred = 6 s (pp.5–6).

### Control interface
Position and ψ splines, i.e. a trajectory. A low-level controller is not described in this paper.

### Timing and compute
Fig. 7 (p6, log axis) *(fig., approx.)*:
- Deep-PANTHER replanning is roughly 15–30 ms.
- PANTHER is roughly 100 ms.
- PANTHER\* is in the thousands of ms.

Text: "two orders of magnitude faster" than PANTHER\* with similar cost, and one order faster than PANTHER with lower cost (pp.1, 6). The hardware for the timing is not stated in the pages read.

### Experiments and metrics
- **Multimodality:** MSE vs expert is 1.09–18.02× smaller than RWTAr and 2.35–2.68× smaller than RWTAc (p6). LSA always yields ≥ 1 collision-free trajectory over the 64-goal grid; some RWTA variants do not (Fig. 9, p7).
- **Generalisation** (Table II, p7), trained on trefoil:

  | Obstacle path | Steps with 4–6 of 6 candidates collision-free |
  |---|---|
  | Trefoil | 94% |
  | Static | 100% |
  | Square | 93% |
  | Eight | 98% |
  | Epitrochoid | 98% |

  At least one collision-free candidate in all ~740 replanning steps.
- **Several obstacles** (Fig. 12, p8): always safe with 1–2 obstacles; "succeed on average" with 3–5.

### Limitations
- Stated (p8): trained with one obstacle only; future work is multi-obstacle, raw images and real-world.
- Observed:
  - Visibility is FOV-cone only (no LOS).
  - Obstacle prediction is GT in evaluation.
  - Dynamic limits are soft in selection, so feasibility is not guaranteed.

### MOTAR relevance and conflicts
- **Most relevant pattern for BOIA (b) and (d):**
  - A learned generator of a **small fixed set (6) of multimodal candidate trajectories** that meet boundary conditions by construction.
  - **Explicit ranking by cost + constraint satisfaction**.
  - **Fallback to the previous collision-free plan** (p5).
- The LSA multimodal loss (p4) is a ready tool if MOTAR ever distils an expensive candidate generator.
- **Conflicts:**
  - Outputs position trajectories, not velocity/yaw-rate, so a tracker layer would be needed to feed Policy-F-compatible commands.
  - Single-obstacle observation; dense bars would need a different encoding.

### BOIA novelty-overlap check
| Item | Verdict | Evidence |
|---|---|---|
| (a) | **Partly (FOV-only, deterministic)** | Closed-form yaw maximising in-FOV (Eq. 1, p5) and an in-FOV cost term (p5). No LOS, no detector reliability, no probability calibration. Object = obstacle. |
| (b) | **Partly** | n_s = 6 boundary-condition-satisfying candidates ranked by cost + soft dynamic-limit penalty + collision check (p5). No capture-progress, loss-probability or covariance-growth terms. |
| (c) | **No** | |
| (d) | **Partly** | Keep the previous collision-free trajectory if no candidate is safe (p5). No terminal-first logic. |

---

#### 2026-10-19 original-copy re-read (Codex)

**Receipt.** Jesus Tordesillas and Jonathan P. How, *Deep-PANTHER: Learning-Based Perception-Aware Trajectory Planner in Dynamic Environments*. User original `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/Deep-PANTHER Learning-Based.pdf`, SHA-256 `ebd98093f2a37de7…`, 9 physical pages; same payload as prior copy. Newly read **pp.1–9**, body/results/limitations/references; **FULL_READ**. Page 5 was rendered to resolve the uncertain exponent. Printed RA-L preprint accepted January 2023; arXiv v2 stamp 14 February 2023 (p.1). No appendix included.

**Problem and assumptions.** Generate several obstacle-avoiding, perception-aware trajectories by imitating a multi-solution optimizer. Input is a 43-D vector: own v/a/goal/yaw-rate, predicted dynamic-obstacle B-spline and AABB size, not raw imagery (p.3). One obstacle is encoded; for multiple obstacles choose the most collision-likely one (p.7). Cone FOV, differential-flatness state and predicted trajectory horizon `T_pred=6s` are assumed; the trajectory duration itself is predicted, with 0≤T≤T_pred (pp.3,5).

**Pipeline and formulas.** PANTHER* runs ten OSA-initialized optimizations with plane/time decision variables; retain best distinct expert solutions. Student MLP has two 64-neuron ReLU layers and predicts six trajectories, each 13 values (interior position B-spline controls + duration), 78 outputs total (pp.3–5). Initial/final boundary conditions and C² continuity are imposed by the representation. On p.4 the **unnumbered** LSA loss is `1ᵀ(β_p A⊙D_p + β_T A⊙D_T)1`, with assignment solved from position MSE and each expert assigned to a distinct student. β_p=β_T=1 (p.5). On p.5 Eq.(1), body axis b₁ is the normalized projection of the obstacle-bearing vector onto the plane perpendicular to ξ=a+ge_z; it is singular if those directions are parallel (footnote 3).

The p.5 footnote-4 cost **does contain `(inFOV)^3`**, visually confirmed; this is a genuine exponent, not an equation number. It replaces PANTHER's blur-weighted PA expression in this experiment and includes an α_T T term. At inference rank collision-free candidates by `c_obj+λ c_dyn_lim`; v/a/j limit violations receive a **soft** penalty. If none is collision-free, continue the previous collision-free plan and replan (p.5). No safety hysteresis or simulator terminal-first is specified. A previous safe plan is not automatically safe under newly revealed obstacles/prediction errors [audit inference].

**GT, execution and evidence.** All reported simulations fit the obstacle spline to **ground-truth future positions** (p.5); the paper suggests past-observation prediction only for future deployment. The result cannot establish real-sensor recovery. Expert labels train the student offline (2K static pairs, 75% training) or via DAgger (~23K dynamic pairs), Adam 10⁻³ (pp.5–6). Output is a position/yaw trajectory; a production velocity-action adapter would need independent dynamics checks. No hardware timing platform/model weight URL is printed; Figure 7 p.6 supports only an approximate timing comparison, with the stated two-order improvement over PANTHER* and one order over PANTHER. It does not establish a measured RTX3070 latency.

Results: MSE is 1.09–18.02× lower than RWTAr and 2.35–2.68× lower than RWTAc on the declared comparisons (p.6). Table II p.7 has at least one collision-free generated trajectory in all approximately 740 replanning steps across tested obstacle path families; 4–6/6 candidates are collision-free in 94% trefoil, 100% static, 93% square, 98% eight and 98% epitrochoid steps. Multi-obstacle tests have ten simulations per obstacle count; the authors say always successful for 1–2 obstacles and successful **on average** for 3–5, not guaranteed safe (pp.7–8).

**Limitations.** Authors reserve raw-image input, multi-obstacle training and real flight for future work (p.8). Observed: FOV-only reward; privileged future prediction; soft dynamic feasibility; old-plan fallback is conditional. Printed code/video (p.1): https://github.com/mit-acl/deep_panther ; http://youtu.be/53GBjP1jFW8 . The paper does not print a weights URL. MOTAR may reuse small multimodal candidates, explicit feasibility ranking and previous-plan fallback as ideas; it cannot import the privileged predictor into online BOIA.

| BOIA element | Overlap | Physical-page evidence |
|---|---|---|
| (a) | partial | Predicted obstacle FOV, not target belief+LOS+reliability, p.5. |
| (b) | partial | Six trajectory candidates with cost/feasibility ranking, no loss/covariance target objective, pp.3–5. |
| (c) | no | Learnt planner replaces its expert; no Policy-F advantage arbitration, pp.3–5. |
| (d) | partial | Keep previous safe plan when none available; no terminal-first, p.5. |
| (e) | no | Candidate collision/safety ratio rather than 2s target recovery competing risk, pp.7–8. |

No source finding requires changing BOIA's frozen design formulas; retain a conditional rather than absolute safety claim.

## 5. RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight

| Field | Value |
|---|---|
| Authors / venue / year | B. Zhou, J. Pan, F. Gao, S. Shen (HKUST/ZJU). arXiv:2007.03465v1, 6 Jul 2020 (p1). Published IEEE T-RO 37(6):1992–2009, 2021, per Deep-PANTHER ref. [1] (Deep-PANTHER p8). |
| Path | `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/RAPTOR__2007.03465v1.pdf` (16 pp) |
| Pages read | pp.1–15 body text (incl. conclusion on p14). Visually checked: p12 (Fig. 21). References pp.15–16 not read. |
| Read status | **METHOD_REVIEWED** (body fully read; references not read) |
| Code / video | "To be released at github.com/HKUST-Aerial-Robotics/Fast-Planner" (p1 footnote); https://youtu.be/6wrh4G1-cQ4 (p1) |

### Problem
Fast local replanning in unknown static clutter. Two issues are addressed:
1. Local minima and feasibility within limited time, handled by topology-guided parallel optimisation.
2. Perception awareness: see dangerous unknown space early and point yaw at it (pp.1–2).

### Assumptions
- **Sensors:** D435 depth (p10). Map: occupancy grid + ESDF fused online (p10). VIO in real tests; GT odometry in simulation (p10).
- **Obstacles:** static and unknown. Unknown space is treated optimistically in the first step, then refined with risk awareness (pp.3, 7).
- **Target:** none. A global reference trajectory is given (straight line in tests) (p10).

### Pipeline (Fig. 3, p3)
1. Topological roadmap with Uniform Visibility Deformation (UVD) classes (Def. 1, p5; Alg. 1, p6).
2. Shortening (Alg. 2) and pruning to K_max paths (pp.6–7).
3. Parallel path-guided optimisation (PGO, 2 phases).
4. Best trajectory → risk-aware refinement (Alg. 3, p8).
5. Yaw: graph search over discrete yaw by information gain (IG), then B-spline yaw optimisation (pp.9–10).
6. Geometric controller tracks position + yaw (p10).

### Key equations
| Eq. | Page | Content |
|---|---|---|
| (1) | p4 | f_p1 = λ_s Σ‖q_{i+1} − 2q_i + q_{i−1}‖² + λ_g Σ‖q_i − g_i‖². Phase-1 warm-up pulls B-spline control points toward the guiding path. Unconstrained QP, closed form. |
| (2) | p5 | f_p2 = λ_s f_s + λ_c Σ F(d(q_i), d_min) + λ_d Σ_µ [F(v²_max, q̇²) + F(a²_max, q̈²)]. Phase-2 ESDF-based optimisation (convex-hull property of B-splines). |
| (3) | p5 | Penalty F(x,y) = (x−y)² if x ≤ y, else 0. |
| (4) | p5 | Velocity and acceleration control points from finite differences. |
| UVD Def. 1 | p5 | τ₁, τ₂ are equivalent if the segment τ₁(s)τ₂(s) is collision-free for all s. |
| (5) | p7 | Visibility level ψ(p, p_f) = min_{q ∈ l(p, p_f)} d(q). This is an **LOS clearance metric** via the ESDF, from p to the frontier-intersection point p_f. |
| (6) | p8 | Worst-case safety: v_c²/(2a_max) ≤ d_cf − R_q. Stopping distance from the critical-view point. |
| (7)–(9) | p8 | View constraint d_v = (p(t_s) − p_f) − ((p(t_s) − p_f)·v_c)v_c with ‖d_v‖ ≤ δ_v; safe reaction distance d_sf ≥ d_s = v_s²/(2a_max) + R_q. |
| (10)–(14) | p8 | Soft penalties, Jacobians, refinement cost f_r = f_p2 + λ_r(f_dv + w_r f_dsf). Iterative speed-estimate loop (Alg. 3). |
| (15) | p9 | Yaw path cost c(Ξ) = Σ(−g_{i,j} + µ(ξ_i − ξ_{i−1})²), solved by Dijkstra. |
| (16) | p9 | IG voxel weight Ig(m) = exp(−w_l D_l − w_s D_s), favouring unknown voxels near the trajectory and near the current position. |
| (17)–(18) | p10 | Yaw B-spline optimisation: jerk-smoothness + soft waypoints + soft φ̇ and φ̈ limits. |

### Online inputs vs GT
All online. Map, ESDF and IG are built from onboard depth. Simulation uses GT odometry and rendered noisy depth (p10).

### Control interface
Position B-spline + yaw B-spline → geometric controller (p10).

### Timing, compute, hardware
- Real world: everything on an i7-8550U (p10).
- Simulation: i7-8700K + GTX 1080 Ti; NLopt solver (p10).
- Average replan time: proposed ≈ 7–10 ms vs FASTER ≈ 28–35 ms across 0.2–0.4 obs/m² *(fig., approx., Fig. 21d, p12)*.
- IG evaluation sped up by "over two orders of magnitude" through subsampling, parallelism and cached raycasts (p9).

### Experiments and metrics
- **Real-world PA test** (Table I, p10). 3 trials per scene, v_max 3 m/s, a_max 2.5 m/s². Successes, scene 1 / scene 2:

  | Planner | Scene 1 | Scene 2 |
  |---|---|---|
  | A (optimistic + velocity-yaw) | 0 | 0 |
  | B (optimistic + active-yaw) | 0 | 0 |
  | C (risk-aware + velocity-yaw) | 2 | 0 |
  | D (risk-aware + active-yaw) | **3** | **3** |

  Scene 2 is an obstacle hidden behind a corner (pp.10–11).
- **Table II** (p13) vs safe local exploration (SLE), 10 maps per density, 5 densities. Successes SLE / proposed:

  | Density (obs/m²) | SLE | Proposed |
  |---|---|---|
  | 0.2 | 10 | 10 |
  | 0.25 | 10 | 10 |
  | 0.3 | 9 | 10 |
  | 0.35 | 9 | 10 |
  | 0.4 | 8 | 10 |

  The proposed method also has shorter flight time: e.g. 23.639 s vs 27.903 s at 0.4.
- **Fig. 21** (p12): best distance, time and energy vs FASTER, EWOK and RE-Traj; 10/10 successes at all densities *(fig.)*.
- **Real flights:**
  - Indoor: max 2.90 m/s, average 1.77 m/s.
  - Forest 1: max 3.19 m/s, average 2.29 m/s.
  - Forest 2: max 3.41 m/s, average 2.14 m/s (pp.13–14).

### Limitations
- No dedicated section in the pages read.
- Observed:
  - Static world only, no target.
  - Yaw is decoupled from translation (two-step).
  - Depends on an ESDF map, and the optimistic step is not conservative.
  - Perception awareness targets map unknowns, not a specific object.

### MOTAR relevance and conflicts
- **Ideas that transfer:**
  1. LOS-clearance visibility metric via a distance field (Eq. 5). For dense vertical bars, an analytic distance-to-cylinders query in Isaac Gym would give the same LOS test cheaply for target-visibility forecasting. [CORRECTED: simulator analytic obstacle GT is evaluator-only; online LOS must use permitted sensed geometry, original RAPTOR p.10 and MOTAR sensor contract.]
  2. Discrete yaw-candidate graph search with an IG/visibility reward (Eq. 15), which is analogous to scoring discrete yaw-rate candidates.
  3. Worst-case stopping-distance criterion (Eq. 6) as a fallback trigger.
- **Conflicts:**
  - Trajectory + geometric-controller interface.
  - Requires an online occupancy map + ESDF on the target side.
  - Planning time of 7–10 ms is acceptable, but this is a whole planner, not an add-on to a frozen policy.

### BOIA novelty-overlap check
| Item | Verdict | Evidence |
|---|---|---|
| (a) | **Partly (map-unknowns, deterministic)** | Forecasts the visibility of the future frontier-intersection point with LOS clearance (Eq. 5, p7), and the IG of unknown voxels per yaw candidate with raycast occlusion (pp.9–10). No target, no detector reliability, no probability. |
| (b) | **Partly** | Several topologically distinct trajectories optimised in parallel, with the best selected (pp.3, 6); discrete yaw candidates (p9). Scored by smoothness/collision/dynamics and IG, not capture, loss probability or covariance. |
| (c) | **No** | |
| (d) | **Partly** | Worst-case stopping criterion and safe-reaction-distance constraint (Eqs. 6–9, p8). Emergency stop used in experiments (p10). No terminal-first logic. |

---

#### 2026-10-19 original-copy re-read (Codex)

**Receipt.** Boyu Zhou, Jie Pan, Fei Gao and Shaojie Shen, *RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight*. User original `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/RAPTOR Robust and Perception-aware Trajectory.pdf`, SHA-256 `208a3f0ec3ee23bf…`, 16 physical pages, byte-identical to prior copy. Newly read **pp.1–16**, all body/results/conclusion/references; **FULL_READ**. Printed arXiv v1, 6 July 2020 (p.1); this PDF is not relabelled as the later T-RO version. No appendix included.

**Problem/assumptions.** Point-to-point navigation in unknown **static** clutter; no target tracking. Forward D435 depth, online occupancy/ESDF, VIO and a global reference trajectory; no prior map in flight. Simulation uses GT odometry and noisy rendered depth (p.10). The local replanning horizon in indoor experiments is 7m (p.13); this is not a universal probabilistic detection horizon.

**Pipeline and decisions.** Sample a compact guard/connector roadmap, extract topologically distinct paths by uniform visibility deformation (UVD), shorten/prune them, parallel path-guided optimization (PGO), choose the best, refine dangerous unknown frontiers for stopping visibility, then plan yaw by IG graph search and smooth B-spline optimization (pp.3–10). Algorithm 1 terminates by wall time/sample limit; shortest-path count K_max and ratio r_max bound cost, but numerical values are **NOT_RECORDED** in the paper text (pp.6–7). A straight-line initial global path is sufficient for tests (p.10).

| Physical reference | Content |
|---|---|
| p.4 Eq.(1); p.5 Eq.(2) | Warm-up QP pulls control points to a geometric path, then optimize smoothness + ESDF collision + v/a limits. |
| p.5 Definition 1; p.6 Algorithms 1–2 | UVD requires connecting segments between corresponding path parameters to be clear; roadmap and path shortening are not target-belief samples. |
| p.7 Eq.(5) | LOS visibility `ψ(p,p_f)=min_{q∈segment(p,p_f)} d(q)` is ESDF clearance toward the first unknown frontier intersection. This **includes occlusion**, not only camera FOV. |
| p.8 Eq.(6) | Stop criterion `v_c²/(2a_max) ≤ d_cf−R_q`; constant available deceleration and map/trajectory validity are assumptions, not a certification for MOTAR dynamics. |
| p.8 Eqs.(7)–(14), Algorithm 3 | Constrain earlier view to frontier ray and sufficient stopping distance; iterate an estimated speed until checked after refinement. Eq.(9) prints `p_i` though surrounding prose and Algorithm 3 line13 use refined `p_r`; retain this notation inconsistency rather than copying it silently. |
| p.9 Eqs.(15)–(16) | Dijkstra yaw graph trades negative visible-unknown IG against yaw change; near-path unknown voxels get greater weight. Raycasting rejects occupied-voxel occlusion. |
| p.10 Eqs.(17)–(18) | Smooth yaw B-spline with angular limits and waypoints. |

**Online/GT and control.** Only the sensed occupancy/ESDF can support an online LOS query. Simulator obstacle identities or analytic GT bar locations are evaluation-only; the previous suggestion of cheap analytic-cylinder queries must not be interpreted as an approved online input. Position/yaw trajectories feed a geometric controller (p.10), whereas MOTAR preserves the actor898 velocity interface. No frozen-policy advantage arbitration appears.

**Costs/results.** Real modules run on i7-8550U; simulation i7-8700K/GTX1080Ti, NLopt solver (p.10). The old ~7–10ms estimate is figure-derived (p.12 Fig.21d), not a new measured timer; exact per-frame/RTX3070 values are **NOT_RECORDED**. Table I p.10: each of four strategies gets three trials per scene; combined risk-aware refinement + active yaw succeeds 3/3 in both scenes, other combinations 0/0, 0/0 and 2/0. Tests use 3m/s and 2.5m/s² limits; emergency stop triggers when a detected collision point is within .5m (p.10). Table II p.13: reported successful-map counts are 10/10 across .2–.4 obstacles/m² versus SLE 10,10,9,9,8; p.12 states five trials per map, so these table counts must not be casually reinterpreted as only ten individual episodes. Flight examples reach 2.90m/s indoors (p.13), 3.19/3.41m/s outdoors (p.14). The paper calls integrated squared jerk 'energy'; it is **not measured mechanical energy** (p.13).

**Limits.** No dedicated limitations section; observed static world/map fidelity/finite sampling, soft constraints and simplified stopping acceleration restrict guarantees. Its optimistic first stage treats unknown as free, followed by explicit refinement; BOIA's unknown-blocked v1 is a different declared choice. Reuse LOS and stopping-check ideas, not an online privileged distance field. Code link printed p.1 footnote: https://github.com/HKUST-Aerial-Robotics/Fast-Planner ; video https://youtu.be/6wrh4G1-cQ4 . No neural weights apply.

| BOIA element | Overlap | Physical-page evidence |
|---|---|---|
| (a) | partial | Deterministic LOS/IG for unknown-map frontiers; no target detection distribution, pp.7,9. |
| (b) | partial | Multiple optimized topological trajectories, progress/safety/smoothness scoring, pp.3–7. |
| (c) | no | No frozen RL advantage arbitration, pp.3–10. |
| (d) | partial | Stopping-distance condition + emergency stop, not simulator terminal-first, pp.8,10. |
| (e) | no | Navigation success rather than post-occlusion competing-risk recovery, pp.10–14. |

No BOIA formula is invalidated by this source; its braking assumption motivates the already planned independent dynamics gate.

## 6. YOPO (original copy)

#### 2026-10-19 original-copy re-read (Codex)

**Receipt.** Junjie Lu, Xuewei Zhang, Hongming Shen, Liwen Xu and Bailing Tian, *You Only Plan Once: A Learning-based One-stage Planner with Guidance Learning*. Original `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/You Only Plan Once A Learning-based One-stage.pdf`, SHA-256 `91c48521ecc2837e…`, **8 physical pages**. Newly read **pp.1–8 in full**, including experiments/conclusion/references; **FULL_READ**. Page 5 visually checked for the score sign and threshold wording. No appendix included. Publication venue/year are **NOT_PRINTED in this manuscript**; the prior code audit's RA-L2024 bibliographic label is not evidence that this PDF prints those fields. First availability is now USER_COPY_NOW_READ; there was no previous PDF SHA or inventory SHA for YOPO.

**Problem and assumptions.** Mapless local quadrotor navigation, no moving target and no detector. Noisy forward depth + own velocity/acceleration + goal unit direction are body-frame inputs; global position/attitude are unnecessary to the network (p.3). Hardware D455 is 87°×58°, ~6m range (p.8). Privileged complete environment/state is available for offline learning only (p.5). Differentially flat point dynamics and quintic boundary-value trajectories underlie the planner; collision safety is learned cost, not guaranteed.

**Pipeline and core formulas.** Uniformly sample a polar/azimuth endpoint lattice in camera FOV. Per-anchor network predicts local endpoint changes, final derivatives and score; select the optimal score, solve only that quintic's coefficients, execute a short prefix and replan (pp.2–4). Modified ResNet18 with stride32 maps 160×96 depth to **5×3 anchors**; nine state inputs are 3 velocity + 3 acceleration + 3 goal direction (pp.3,6). Output is **5×3×10**: Δθ,Δφ,Δr, v_xyz,a_xyz,score (p.4). No parameter count is printed.

| Reference | Verified formula/meaning |
|---|---|
| p.3 Eq.(3) | Anchor endpoint `(r cosθ cosφ,r cosθ sinφ,r sinθ)`. The **numerical angle samples and r are NOT_RECORDED**; a generic FOV-uniform rule is given. |
| p.3 Eqs.(4)–(5) | Refined endpoint with additive r/angle offsets; quintic coefficients `A=M⁻¹d` from start/end states and fixed duration. |
| p.3 Eqs.(6)–(7); p.4 Eqs.(8)–(9) | Primitive-frame state rotation; tanh-bounded offsets and end velocity/acceleration. Bounds are local-prior parameters, not numeric ±15° or r=5m in this PDF. |
| p.4 Eq.(10) | `J=λ_s J_s+λ_o J_o+λ_g J_g`: smoothness, obstacle proximity, goal. |
| pp.4–5 Eqs.(12)–(20) | Quadratic integrated derivative cost; time-integrated ESDF proximity with `c(d)=exp(−(d−d₀)/k)`; endpoint-goal squared error; manual gradients backpropagated through boundary states and offsets. |
| p.5 Eq.(21) | **`SmoothL1(y_s,−J)`**, so score represents negative cost and larger is preferable. This differs from YOPO-Simple's nonnegative predicted cost/argmin. |

**Guidance learning / privileged input.** Numerical smoothness/ESDF/goal gradients directly supervise trajectory predictions through the chain rule; no expert trajectory labels or learned critic is needed (pp.4–5). Scores are fitted with Eq.(21). Flightmare provides 100K pose/depth samples and privileged map; train50 epochs, Adam lr1.5e−4, batch16, then DAgger fine-tuning (pp.5–6). State/goal can be resampled per depth image. The paper says train end states whose score is greater than threshold, then prints threshold **+0.04** (p.5); this is internally difficult to reconcile with −J for a nonnegative cost. The sign/selection-threshold relation is **UNRESOLVED in the paper**, not silently rewritten as the code's rule. No online ESDF/GT is fed to the policy (p.3 Fig.3/p.5).

**Decisions, control and costs.** Fixed **2s horizon at15Hz**; execute1/30 prefix, sample reference p/v/a/yaw at50Hz; yaw bisects velocity and goal (p.4). Geometric controller follows references (pp.4,8). There is no stopping fallback, terminal-first simulator logic or calibrated advantage to a frozen RL policy. Runtime: **1.6ms** i7-9700/RTX3060 simulation inference (pp.6–7), **16ms** XavierNX TensorRT (p.8). A 250mm,1.13kg quadrotor carries D455 and VINS-Fusion onboard (pp.7–8).

**Paper evidence.** Table I p.6 compares the same-cost gradient expert with learned planner: expert average/optimal cost .078/.036 at30ms; final network .050/.036 at1.6ms. Table II p.7 reports at4m/s average speed: YOPO mean/min obstacle clearance1.63/.24m in1/30 tree/m² and1.28/.16m in1/20, versus declared baselines. These are dataset-specific metrics, not MOTAR validation. Figure7 p.6 has ten trials per speed/density; exact success rates are not newly digitized here. Author-stated tradeoff: smoothness training sacrifices safety relative to Agile Autonomy at higher speeds in dense scenes (p.7). Real forest reaches5.52m/s at1/10 tree/m² with ~.25m trees (p.8).

**Original paper vs audited YOPO-Simple bd8a7f9.** Code statements below retain K2's prior FUNCTIONS_READ provenance; no external code was executed or newly audited.

| Item | Original paper | YOPO-Simple code K2 §2 | Consequence |
|---|---|---|---|
| Lattice | 5×3, FOV-uniform; numeric angles/radius absent, pp.3,6 | 5×3, horizontal±36° with18° spacing, vertical±20° with20° spacing, r5m | Do not attribute code numerics to paper. |
| Inputs/output | Depth160×96, v/a/goal9; 15×10, pp.3–4,6 | Same main dimensions, ResNet18, offsets/end derivatives/score | Architecture similarity is not version equivalence. |
| Score | Negative cost, Eq.(21), p.5 | Positive cost + softplus + argmin | Ranking convention differs; do not reverse BOIA's explicit lower-J selection. |
| Horizon/rate | 2s,15Hz replan,50Hz reference, p.4 | Segment time depends on speed ratio; ~1.667s at ratio1; depth callback and50Hz reference | Code is not a reproduction of the printed runtime contract. |
| Training | Numerical ESDF gradients, no expert labels, pp.4–5 | Autodifferentiated cost, added acceleration smoothness and different goal-loss details (K2) | Maintain separate equation/source lineage. |

Printed code/video URLs (p.1): https://github.com/TJU-Aerial-Robotics/YOPO ; https://youtu.be/GoqZM3TxDbM . No weights URL is printed. The audited default **YOPO-Simple** is a later refactor; the repository README says paper-consistent version is `main` (K2), which was not code-audited.

**MOTAR/design judgment.** There is no mathematical obligation to replace BOIA's 15 bearing×speed commands with YOPO's 15 azimuth×elevation endpoints: their dimensions, control authority, horizons, training and scoring differ. Preserve 07§8 and test its coverage in later G5/G6. Potential learned candidate refinements are future training work and out of G4 scope. Do not import ESDF privileged training data into online belief or actor inputs.

| BOIA element | Overlap | Evidence |
|---|---|---|
| (a) | no | No target/detection belief, pp.2–5. |
| (b) | partial | Finite learned primitive trajectories, progress/smoothness/safety scoring, pp.3–5. |
| (c) | no | A full learned planner, not advantage arbitration over frozen RL, pp.2–5. |
| (d) | no | No explicit stop-distance/terminal-first design, pp.4–5,8. |
| (e) | no | Navigation success and obstacle clearance, not target recovery competing risks, pp.6–8. |

**Design impact:** citation/version correction and prior-art overlap only. The unresolved negative-score threshold does not enter BOIA's specified analytic cost. No G3 formula invalidation found.

## Cross-paper summary table

| Paper | Learned or optimisation | Inputs (online) | Outputs | Control interface | Perception-aware term | Handles target occlusion? | Compute | Code |
|---|---|---|---|---|---|---|---|---|
| YOPOv2-Tracker (2025) | Learned (ResNet-18 FCN, trained by back-propagating analytic trajectory costs; privileged ESDF) | RGB-D 160×96, 6-D v/a state (+ VIO pose for EKF / disturbance observer) | 5×3×14: primitive offsets, end derivatives, cost, objectness, target (u, v, d) | Thrust + attitude (via disturbance observer) to PX4 | Yaw locked to the EKF target estimate; target-goal cost J_g (training) | **No** explicit handling. EKF predict-only + yaw toward the loss direction. Occlusion is the main failure (p13) | 3 ms (RTX 3060, TensorRT); 8.2 ms (Orin NX) | github.com/TJU-Aerial-Robotics/YOPO-Tracker ("will be released") |
| NavRL++ (2026) | Learned (PPO, transformer 0.31 M params) | 144 ray distances from occupancy map, ≤5 KF-tracked dynamic obstacles × 2 s history, goal/velocity history | Beta-mean velocity (2-D/3-D), goal-aligned frame | Velocity → low-level controller; optional VO safety projection | None (no target); perception-failure dropout only in training | **No** (no target); detection "sensitive to occlusion" (p15) | 4.1 ms, 20 Hz onboard; ~200 GPU-h training on RTX 4090 | "Will be released on GitHub", no URL |
| PANTHER (2022) | Optimisation (IPOPT, joint p–ψ, Hopf fibration, MINVO) | Depth point cloud → tracked and predicted obstacles (Gaussian) | Position (cubic) + ψ (quadratic) B-splines | Trajectory → Hopf controller 100 Hz → geometric 500 Hz | Sigmoid in-FOV of predicted obstacle × blur penalty | **No** (FOV only, no LOS; object = obstacle) | ≈49–59 ms onboard NUC (1–3 obstacles) | github.com/mit-acl/panther |
| Deep-PANTHER (2023) | Learned (IL from PANTHER\*, MLP 2×64, LSA multimodal loss, DAgger) | 43-D: v, a, goal, ψ̇, obstacle prediction spline (10 control points), obstacle size | 6 trajectories × 13 (control points + T), then closed-form ψ | Trajectory (p, ψ) | Closed-form in-FOV yaw (Eq. 1) + in-FOV cost in ranking | **No** (FOV only; GT obstacle prediction in evaluation) | ~2 orders faster than PANTHER\* (≈ tens of ms, fig.) | github.com/mit-acl/deep_panther |
| RAPTOR (2020/21) | Optimisation (topological roadmap + parallel PGO + risk-aware refinement; yaw graph + B-spline) | Depth → occupancy grid + ESDF; global reference | Position B-spline + yaw B-spline | Trajectory → geometric controller | LOS visibility of the frontier point (ESDF), IG-driven yaw | **No** (no target; LOS/IG for map unknowns) | ≈7–10 ms replan (fig.), i7-8550U onboard | github.com/HKUST-Aerial-Robotics/Fast-Planner ("to be released") |

### Cross-paper BOIA verdict
- **(a):** no paper forecasts a *calibrated probability* of future *target* detection combining FOV, LOS and detector reliability.
  - PANTHER and Deep-PANTHER have FOV-only visibility for obstacles.
  - RAPTOR has LOS-based visibility and IG for map unknowns.
  - YOPOv2 has no visibility forecast and names occlusion as its main failure mode.
- **(b):** finite candidate sets with ranking exist: YOPOv2 (15 primitives), Deep-PANTHER (6 learned trajectories) and RAPTOR (topological set). None scores capture progress + loss probability + covariance growth.
- **(c):** none arbitrates against a frozen RL policy by calibrated expected advantage with hysteresis. NavRL++'s VO shield overrides RL commands for safety only.
- **(d):** pieces exist separately:
  - NavRL++: zero-velocity fallback.
  - Deep-PANTHER: keep the previous safe plan.
  - RAPTOR: stopping-distance criterion.
  - PANTHER: terminal hover constraint.

  None combines these with terminal-first handling.
- **Biggest prior-art hazard for MOTAR:** NavRL++ (May 2026), not for BOIA but for anything claimed about the base policy (transformer temporal fusion, perturbation-aware fine-tuning, multi-sensor ray representation).
