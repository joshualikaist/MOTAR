# P2 — Active / embodied visual tracking papers: structured notes

Reader notes written 2026-10-19 for the MOTAR occlusion-aware navigation project.
How I read them: the full text of every PDF came from `pdftotext` (non-layout mode, form-feed page split, so page numbers are **physical 1-based PDF pages**). I then checked equations and tables against page images with the Read tool (pages listed per paper). Two-column tables garble in the text extraction, so I only quote table numbers I checked on a page image, unless marked otherwise.
Conventions: "pN" means physical PDF page N. **[UNCERTAIN]** marks a transcription or interpretation I could not confirm. "Observed" items are my own inferences and are not claims made by the authors. Numbers are quoted with their context only. They come from different benchmarks and setups, so do not carry them over to MOTAR.

MOTAR reference frame used below: a forward RGB-D camera plus LiDAR on a quadrotor. A frozen RL policy (898-D actor input, which must not change) outputs velocity and yaw-rate. The setup also has a Kalman target tracker, dense random vertical bars as obstacles, and an 8 GB GPU.
BOIA concept components: (a) a belief-conditioned forecast of future detection probability (FOV + LOS + detector reliability); (b) a finite set of dynamics-feasible short-horizon candidates, scored by capture progress + loss probability + covariance growth; (c) arbitration that overrides the frozen RL policy only when the calibrated expected advantage exceeds a threshold, with hysteresis; (d) a safe fallback plus terminal-first handling.

---

## 1. OA-VAT — "Instance-level Visual Active Tracking with Occlusion-Aware Planning"

- **Authors / venue as printed:** Sun, Zhou, Gao, Zhang, Hu, Wen, Ye, Tan (SCUT et al.). arXiv:2604.21453v1 [cs.CV], 23 Apr 2026 (p1). The paper's own Table 1 lists "Ours — CVPR 2026" (p6).
- **Path:** `/home/fair/Downloads/Instance-level Visual Active Tracking with Occlusion-Aware Planning.pdf`. 15 pp. sha256 prefix `359b64b75bc57720`. Identity: PDF title metadata matches. The caller says it is identical to the user copy, and I did not compare it separately.
- **Pages read:** text of p1–p15, all of it, including the references and appendix A–D. Page images checked: p4–p8 (Eqs. 4–11, Tables 1–5, Figs. 3–7) and p13 (implementation details, Eqs. 35–36).
- **Read status: FULL_READ.**

### Problem and assumptions
- Task: Visual Active Tracking (VAT), which means controlling a camera to follow one specific target *instance* in 3D (p2 §2). The paper targets two failure modes: distractor confusion from category-level perception, and loss of the target under occlusion because nothing plans actively (p1–2).
- Observation: an RGB image I_t (example 160×120) plus a single reference image I_ref of the target, given at the start of the episode (p2). Depth, LiDAR and a map are **not** used at inference. FOV is not stated.
- Action: continuous a_t = [v_f, v_l, v_v, ω_y]^T, i.e. forward, lateral and vertical linear velocity plus yaw rate (p2, Eq. 1: a_t = π_θ(I_t, I_ref)).
- Platform: simulators UnrealCV (ground/camera agent maps) and the DAT drone benchmark; a real DJI Tello drone in velocity-control mode, streaming 320×240 video over Wi-Fi to an RTX 3090 ground station (p9, p15).
- Targets: persons and vehicles. The prototype step for humans adds diffusion-generated novel views (p3). Target motion is whatever the benchmark provides.
- Map: none at inference. A 2D occupancy map plus A* is used **only to label training trajectories** for the planner (p5).

### Pipeline step by step
1. **Offline instance prototype (training-free), p3, Fig. 2(a).** Start from I_ref. Augment it with horizontal/vertical flips; for humans, also add novel views from an off-the-shelf diffusion model [49]. Segment with YOLO-E (`yoloe-11l-seg`, p13 [UNCERTAIN: the text extraction reads "yoloe-l1l-seg"]) and crop. Extract features with DINOv3 plus global average pooling (Eq. 2). The prototype is f̃ = (f_ref + (1/N)Σ f_i) / ‖f_ref + (1/N)Σ f_i‖₂ (Eq. 3, p3).
2. **Detection by prototype matching (Alg. 1 lines 1–11, p3).** This runs while the tracker is uninitialised (b_t = ∅). YOLO-E produces M candidate masks of the target category. Each crop is passed through Desc(·), and the cosine similarity S(f̃′, f_cand^i) is computed (Eq. 4, p4). The best candidate is accepted if S > η_s, with η_s = 0.5 (p13). Its box then initialises ORTrack [46], a passive single-object tracker (p4).
3. **Online tracking plus prototype enhancement (Alg. 1 lines 13–25).** The tracker T returns a confidence c_t and a box z_t. If c_t ≥ η_c, the KF is updated and the prototype gets an EMA update f̃′ ← β f̃′ + (1−β) f̂_tar with β = 0.8 (Eq. 5, p4; p13). The EMA runs in a separate thread (p4).
4. **Confidence-aware Kalman filter (p4).** It runs in the **image plane**: x_t = [x, y, w, h, ẋ, ẏ, ẇ, ḣ]^T. Predict with Eq. 6, update with Eqs. 7–8, and use R_t = σ²(c_t) I with σ²(c_t) = 1/(1+e^{λ(c_t−γ)}) (Eq. 9), where λ = 15, γ = 0.4 (Eq. 36, p13). When c_t < η_c, z_t is dropped (Alg. 1 line 19) and the box is taken from the prediction, b_{t+1} ← H x̂_{t|t−1}.
5. **Search / track / recovery switching.** There are two rule-based modes:
   - detection mode when b_t = ∅;
   - track mode otherwise.
   Recovery planning is triggered "if no z_t is available for many consecutive frames" (p4). On p5 the trigger is described instead as "when the tracker becomes unreliable, i.e., c_t < η_c". **Neither the η_c value nor the frame count is given anywhere I read** [observed: the two trigger descriptions are inconsistent]. Algorithm 1 has no explicit path back to detection mode after tracking has started [UNCERTAIN how re-detection is re-entered].
6. **Occlusion-aware trajectory planner (p5, Fig. 3).** This is a conditional diffusion policy (in the style of Diffusion Policy [7]) that models p(A_t | I_t, b_t), where A_t is the trajectory points over horizon T_p. Denoising follows Eq. 10 and training uses the MSE noise loss of Eq. 11. The condition b_t is the **KF-predicted box of the occluded target** (p5). Training data is Planning-20k:
   - 20k samples collected in UnrealCV SimpleRoom;
   - random obstacles; the target is spawned on an obstacle's bbox edge and the tracker on an adjacent edge;
   - samples in which the target is fully visible are discarded;
   - A* expert trajectories on the 2D occupancy map;
   - 8k default plus 12k texture/illumination-randomised samples;
   - single-side, double-side and corridor occlusion types (p5).
   The values of T_p and K, the waypoint frame, and how waypoints become velocity commands are **not specified** (observed).
7. **Low-level control.** Velocity commands (linear and angular) are sent to the Tello (p15). The normal (non-occluded) tracking controller is not described explicitly. The ablation "w/o Planning (i.e., PID)" (Table 4, p7) implies a PID image-centring controller is the base [UNCERTAIN].

### Key equations (page, eq. no.)
- Eq. 3 (p3): the L2-normalised mean of the reference feature and the average augmented feature.
- Eq. 4 (p4): cosine similarity between the prototype and the candidate feature.
- Eq. 5 (p4): EMA prototype update, β = 0.8.
- Eqs. 6–8 (p4): standard KF predict, update and gain. F is the transition matrix, H the observation matrix, P the covariance.
- Eq. 9 (p4): R_t = σ²(c_t) I, with σ²(c) = 1/(1+exp(λ(c−γ))). The authors' stated assumption is "We hypothesize that c_t reflects the uncertainty of the current observation" (p4).
- Eq. 10 (p5): A_t^{k−1} = α(A_t^k − φ ε_θ(I_t, b_t, A_t^k, k)) + N(0, σ²I). Eq. 11 (p5): L = E‖ε − ε_θ(I_t, b_t, A_t^0+ε, k)‖².
- Proposition 1 / Eq. 12 (p5) and Appendix B, Eqs. 13–34 (p12–13): prototypes from multi-view augmentation separate instances better than raw features. This holds **under Assumption 1** (multi-view augmented features cover the feature manifold better than the reference, Eq. 13) and **Assumption 2** (intra-manifold cosine ≥ δ, inter-manifold ≤ η) (p12).

### Occlusion, re-acquisition, identity, first acquisition
- **Occlusion detection:** indirect only. Low tracker confidence (c_t < η_c) or missing measurements trigger the KF coast and then the planner (p4–5). No geometric or LOS reasoning happens online. The planner is supposed to infer navigable paths "from spatial relationships between targets and obstacles" in the image (p5).
- **Re-acquisition:** not formally defined. Qualitatively, "accurately locates and re-identifies the designated target after temporary occlusions" and "Target recovered → Success" (p9, Fig. 6). Metrics measure keeping the target in view (EL, SR, TSR), not the re-acquisition event itself.
- **Identity:** the instance prototype plus EMA. Evaluation uses UnrealCV maps with 2 or 4 distractors (Table 1), the prototype similarity margin (0.28 for persons vs 0.08 for DINOv3, Fig. 4(c), p7), and real-world distractor demos (Fig. 9, p15). There is no ID-switch metric.
- **First acquisition:** the reference image is given and the target is detected by matching when it is in view. No active search for a never-seen target is described. In the benchmarks used, the target starts in view.

### Online inputs vs training-time ground truth
- **Online (inference):** RGB frames and I_ref only (p2). Masks come from the YOLO-E model, **not** from simulator segmentation. No GT target position is used online.
- **Training (planner only):** GT target and tracker positions plus the 2D occupancy map are used to produce the A* labels; the GT target bbox in the captured image serves as the conditioning input (p5, Fig. 3(a)). The tracker and prototype modules are training-free.

### Timing / compute / model size
- 35 FPS on an RTX 3090 (p1, p7, p9; ground-station pipeline about 35 FPS, p15). Model size 584M parameters (Table 1, p6). Training: 60 epochs, batch 64, one RTX 3090, about 15 h (p13; p6).
- DINOv3 `vitl16` (300M params), 384×384 input, "768-dimensional features" as printed (p13). [UNCERTAIN: ViT-L normally gives 1024-D; this is recorded as printed.]
- Ablation: DINOv3 ViT-B/16 (86M) costs only about 1.1% relative performance (p14, Table 6).
- TrackVLA comparison quoted by the authors: TrackVLA runs at 10 FPS on an RTX 4090, and its training used 24×H100 for the same duration (p6–7).
- GPU memory is not stated.

### Experiments
- **UnrealCV with distractors** (Parking Lot 2D, UrbanCity 4D, ComplexRoom 4D; 100 episodes; 500 steps; early termination when the target is out of view for >50 steps):
  - AR = average accumulated reward;
  - EL = average episode length;
  - SR = fraction of episodes in which the target is followed for 500 steps (p6).
  - Context: average SR is 0.93 vs TrackVLA 0.91 and EVT 0.81 (Table 1, p6). The TrackVLA and other baseline rows match TrackVLA's own Table 7 (p18 of the TrackVLA PDF), so they appear to be copied, not re-run. The caption says TrackVLA AR could not be reproduced because its code and checkpoints are unavailable (p6).
- **UnrealCV single-target, 5 maps:** SR = 1.00 and EL = 500 in all of them (Table 3 p6; Table 7 p14).
- **DAT benchmark** (6 scenes, daytime, compared against within-scene-trained RL models; zero-shot for OA-VAT):
  - CR = cumulative reward over 40 episodes;
  - TSR = success within 1,500 steps (p6).
  - Context: average CR 242→321 and TSR 0.72→0.86 vs GC-VAT (p7). Baseline numbers equal the DAT paper's within-scene Table 2. "SARL [27]" in OA-VAT is the same Luo et al. TPAMI 2019 method that DAT calls "AOT [39]" (observed).
- **Real-image CAR** (Correct Action Rate: would the predicted action move the target toward the image centre?), with 8 videos each from VOT, DTB70 and UAVDT: average 0.908 vs GC-VAT 0.797 and ORTrack-with-GT-box-init 0.774 (Table 5, p8; p8 text).
- **Real DJI Tello:** TSR 81.6% vs FAn 18.9% and EVT 13.8%; FPS 35.0 / 22.3 / 30.2 (Fig. 7, p8).
- **Ablations (Table 4, p7), average SR:**
  - DINOv3 prototype 0.87 vs ours 0.93;
  - no online enhancement 0.82; average enhancement 0.89;
  - no KF 0.87; linear KF 0.90;
  - PID-only (no planning) 0.85; EVT planner 0.87; planner without the bbox condition 0.89.
- **Baselines:** SARL, AD-VAT, AD-VAT+, TS, D-VAT, GC-VAT, EVT (RL); DiMP, RSPT, FAn, FAn+SAM2, TrackVLA (pipeline) (p6).

### Limitations
- **Stated:** there is no dedicated limitations section. The paper only motivates against RL sample inefficiency and the sim-to-real gap of prior work (p1, p12).
- **Observed:**
  - η_c and the "many consecutive frames" trigger are unspecified, and the trigger is described inconsistently (p4 vs p5).
  - Planner horizon, waypoint frame and the trajectory→velocity conversion are unspecified.
  - The planner was trained only on 2D room/box layouts with three occlusion archetypes, so 3D obstacles are not covered.
  - The KF is image-plane only (no 3D or metric state) and has no ego-motion compensation.
  - There is no collision metric and no safety layer.
  - Real deployment is offboard over Wi-Fi.
  - Several baseline numbers are copied from other papers.

### Code / weights
- `https://github.com/SHWplus/OA-VAT` (p1). Weights are not mentioned.

### MOTAR relevance
- **Reusable:**
  - The confidence→measurement-noise sigmoid (Eq. 9) can go straight into the MOTAR Kalman tracker (detector score → R).
  - The Algorithm 1 gating pattern: drop the measurement when confidence < η_c and keep predicting.
  - The general idea of conditioning a recovery behaviour on the **predicted target state** rather than on appearance.
  - The prototype+EMA re-ID would matter only if MOTAR has look-alike distractors. With vertical bars as the clutter, identity confusion is probably secondary (observed).
- **Conflicts:**
  - The diffusion planner outputs trajectories and would replace the frozen policy's action. It would have to be wrapped as an override, which needs the arbitration logic OA-VAT lacks.
  - It is 2D-occupancy/A*-trained in rooms, while MOTAR has dense random vertical bars in 3D flight.
  - The image-plane KF does not match a metric 3D KF.
  - The 584M-parameter stack (DINOv3-L + YOLO-E-L + ORTrack + diffusion) probably fits on 8 GB with ViT-B (p14). Memory is not reported.

### BOIA overlap check
- (a) Forecast of future detection probability: **No.** The only "reliability" modelling is the mapping of the current confidence to R (Eq. 9, p4). Nothing forecasts FOV, LOS or detection probability. Conditioning the planner on the KF-predicted bbox (p5) is belief-conditioned *planning*, not a detection forecast. **Partly-adjacent at most.**
- (b) Finite feasible candidates scored by capture/loss/covariance: **No.** The diffusion model samples one denoised trajectory imitating A* (Eqs. 10–11, p5). There is no candidate set and no scoring.
- (c) Calibrated-advantage arbitration over a frozen policy with hysteresis: **Partly (weak).** A hard, rule-based switch from tracking control to the planner on c_t < η_c or prolonged missing z_t (p4–5). It has no value or advantage estimate, no calibration and no hysteresis.
- (d) Safe fallback + terminal-first: **No.** There is no explicit safety fallback or terminal handling. Collision-freeness comes only from the A*-labelled training data (p5).

---

## 2. UAST — "Unified Active Search and Tracking for Arbitrary Targets with UAVs"

- **Authors / venue as printed:** Qin, Wang, Lu, Qiu, Zhou, Li (USTC; IAI Hefei). The CVPR Open Access watermark states it is "identical to the accepted version" (p1). Proceedings page numbers 13464–13473; the venue is CVPR (2026 per the filename and the caller) [year not printed on the pages I read].
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/UAST__CVPR2026_Qin.pdf`. 10 pp. sha256 prefix `e19a723113bdc763`. Identity: title metadata matches. The caller says it is identical to the user copy. No supplementary material is in this PDF.
- **Pages read:** text and page images of p1–p10, all of them.
- **Read status: FULL_READ** (main paper only; the supplement is not available, so threshold values are missing).

### Problem and assumptions
- Task: active **search** plus persistent **tracking** of an arbitrary, user-specified target by a UAV in cluttered 3D, "mapping-free", "using only RGB-D observations" (p1, p3 §3.1).
- Inputs at time t:
  - RGB-D I_t = {I_t^rgb, I_t^d};
  - kinematic state s_t = (v_t, a_t);
  - target template M^target;
  - start state S_t^start = {p_t, v_t, a_t} (p3).
  The world-frame guidance point and exploration also need the pose p_t, i.e. odometry. The real system uses Fast-LIO [36] (LiDAR-inertial) for localisation (p8).
- FOV: not given numerically. Real camera: Intel RealSense D435 (p8).
- Platform: quadrotor. Simulation runs on an i9 + RTX 4090 workstation (p6). The real system uses a Jetson NX onboard (p8).
- Targets: synthetic quadrotor, sphere, human, ball, animal objects projected into images for training (p6, Fig. 5). Test target: a UAV following YOPO point-to-point motion (short range) or abrupt maneuvers — sharp turns, stops, reversals (long range) (p7). Other objects act as distractors (p7).
- Environments: "open" (hall, forest) with sparse obstacles, and "cluttered" with **dense pillars and irregular low walls** (p6–7).
- Map: no map at inference. A point-cloud/SDF of the scene is used during **training** for the loss terms (p5–6).

### Pipeline step by step (Fig. 2, p3)
1. **Target branch (p4).**
   - RGB and the template go through a shared AlexNet backbone, then cross-correlation, then a light head that predicts a bbox and a coarse mask.
   - The mask is encoded to F_t^target ("class-agnostic").
   - The template feature is extracted **once** and reused, i.e. never updated (p4).
   - The branch is trained separately with the SiamFC++ protocol and then frozen (p5).
2. **Depth branch (p4).** Depth goes through ResNet-18 to give F_t^depth (geometry, free space).
3. **Rule-based point search policy (p4–5), three modes:**
   - **Reliably visible.** Condition: response score m_t > τ_det. Take the median valid depth z̄ inside the central half-box b_t^c (Eq. 2), back-project to g_t^cam (Eq. 3), then transform to world g_t^world.
   - **Temporarily invisible.** A constant-velocity KF runs on x_t = [g^world, ġ^world] (Eqs. 4–5) with innovation η_t = ‖y_t − H x̂_{t|t−1}‖_{R^{-1}} (p4). If η_t > τ_innov, the detection is rejected and the predicted point Hx̂_{t|t−1} is used (p4–5). [Observed: as printed, the innovation is normalised by R rather than by the innovation covariance S = HPH^T + R.]
   - **Lost.** When the target is invisible for T_lost frames, **or at the start of a search when no target has been detected yet**:
     - initialise an unexplored disc of radius r_max around p_t;
     - mark the current camera FOV region "up to the maximum sensing range or until occluded by obstacles" as explored;
     - randomly sample points in the unexplored region and pick the **farthest valid** one, g = argmax‖c_k − p_t‖;
     - if no unexplored point remains, use a spiral r(θ) = r_0 + εθ (Eq. 6);
     - when the region saturates, expand the radius by a factor (e.g. 1.5) (p5).
   - τ_det, τ_innov, T_lost, r_max, r_0 and ε are **not given** in this PDF (observed).
4. **Control network (p5, Fig. 3).**
   - Fused features plus F^state (guidance point g^cam, v^cam, a^cam) are arranged in a V×H spatial grid.
   - Each cell has a default initial end state S_{v,h}^init (position, orientation, prediction radius).
   - A light CNN predicts an offset ΔS = {Δp, Δv, Δa} and a scalar cost score s_{v,h}.
   - End state = init + offset. The **lowest-score cell** is selected.
   - A 5th-order polynomial per axis joins the start and end states, matching p/v/a at both ends (Eq. 7).
   - Commands are u_t = {f(t), ḟ(t), f̈(t)}, i.e. position, velocity and acceleration references "for low-level flight control" (Eq. 8, p5).
   - Horizon T is not given numerically.
5. **Training (p5–6):**
   - L = λ_t L_traj + λ_s L_score (Eq. 9);
   - L_traj = λ1 L_track + λ2 L_goal + λ3 L_smooth + λ4 L_safe (Eq. 10);
   - L_score = SmoothL1(l_{v,h}, L_traj), so the score is trained to **predict the trajectory loss** (Eq. 17, p6).
   - Depth branch, mask encoder and control network are trained end-to-end through the differentiable trajectory.
   - Data: >100k samples; synthetic targets are projected into target-free RGB-D captures (40% >10 m, 50% 5–10 m, 10% <5 m; 90% visible, the rest mildly occluded); metre-level noise is added to the guidance point (p6).

### Key equations
- Eq. 1 (p4): f*(t) = argmin_f L_traj(f(t); g_t(m_t)). "High confidence in the belief state m_t steers g_t toward tracking, while uncertainty directs it to exploration."
- Eqs. 2–3 (p4): central-box median depth back-projection, g^cam = [(u_c−c_x)z̄/f_x, (v_c−c_y)z̄/f_y, z̄]^T.
- Eqs. 4–5 (p4): CV-model KF with A = [[I3, ΔtI3],[0, I3]] and H = [I3 0].
- Eq. 6 (p5): spiral search.
- Eqs. 7–8 (p5): quintic polynomial and derived commands.
- Eq. 11 (p6): L_track = λ_a L_occ + λ_f L_fov, evaluated only on the **mid-segment (1/3–2/3 of the horizon)**.
- Eq. 12 (p6): L_occ = Σ_i Σ_j ReLU(d_voxel − d_ij), with SDF samples d_ij along the ray from trajectory point p_i to the target p_g. This is a differentiable LOS-occlusion penalty.
- Eq. 13 (p6): L_fov = Σ_i ReLU(|ψ_i−ψ_g|−ψ_fov) · ReLU(|φ_i−φ_g|−φ_fov) [UNCERTAIN: the operator between the two ReLU terms is not legible on the page image; they appear juxtaposed (a product?)]. ψ and φ are the yaw/pitch of the UAV **velocity vector** vs the target direction.
- Eq. 14 (p6): L_goal (length error plus a lateral-deviation term, weighted by ω_p, which grows as the UAV nears the goal).
- Eq. 15 (p6): L_smooth = Σ_k s_k^T R_s s_k.
- Eq. 16 (p6): L_safe = ∫_0^T exp(−(d(p(t))−d_0)/β) dt.
- Eq. 17 (p6): score regression.

### Occlusion, re-acquisition, identity, first acquisition
- **Occlusion handling:**
  - Online: score threshold and KF innovation gating; KF prediction while temporarily invisible; T_lost→exploration (p4–5).
  - Offline: occlusion-aware *training* through the SDF LOS loss L_occ and the FOV loss L_fov (p6). At inference, LOS awareness exists only implicitly, through the learned cost score.
- **Re-acquisition:** implicit. When m_t > τ_det again, the policy returns to the visible mode. It is measured by long-range SR = "fraction of flight time with uninterrupted, collision-free target tracking" (p7), plus qualitative "rapid recovery" (Fig. 7, p8). No re-acquisition latency metric.
- **Identity:** a fixed template plus the correlation response. Distractors are present (p7), but there is no identity metric or re-ID module. Observed: the template is never updated, so appearance change is a risk.
- **First acquisition:** **explicit.** Exploration mode is active "at the beginning of a search task when no target has been detected" (p5). It is evaluated with a target initially >100 m away (p7, Table 4).

### Online inputs vs ground truth
- **Online:** RGB-D, UAV velocity/acceleration, template, pose (world-frame KF and explored-region bookkeeping; Fast-LIO in the real system). No GT target position or masks; the coarse mask comes from the target branch.
- **Training:** a scene SDF/point cloud for L_occ and L_safe, GT synthetic target positions for projection and labels, and target-branch training under the SiamFC++ protocol (p5–6). [UNCERTAIN: how runtime "occluded by obstacles" is decided when marking explored FOV regions. Presumably from depth, but this is not stated (p5).]

### Timing / compute / model size
- Cycle time 8.6 ms per step (Table 2, p7; also the search CT in Table 4). Baselines: Yopov2 7.0 ms, Elastic 26.4 ms, Visibility 60.5 ms. Measured on an i9-13980HX + RTX 4090 (p6).
- Training: 50 epochs, batch 1024, about 8 h (p6).
- The real system runs onboard on a Jetson NX (p8); the onboard rate is not given.
- Parameter count is not given (AlexNet + ResNet-18 + light CNN).

### Experiments
- **Short-range** (open environments, 100–150 m flights, 40 randomised trajectories; success = target remains visible and obstacle clearance ≥ 0.2 m):
  - Ours 1.00/1.00/0.93/0.88 at 3/4/5/6 m/s target speed;
  - Yopov2 1.00/0.90/0.90/0.80;
  - mapping-based Visibility [30] and Elastic [15] drop to ≤0.15 at 5–6 m/s (Table 1, p7).
- **FOV heatmaps:** UAST concentrated (Fig. 6, p8).
- **Long-range** (cluttered pillars/walls, 1–1.5 km flights, 10 trajectories): SR 0.99 (3 m/s) and 0.89 (5 m/s) vs Yopov2 0.67/0.36 (Table 2, p7).
- **Safety:** minimum UAV–obstacle distance 0.63 / 0.13 m at 3 / 5 m/s (Table 3, p7).
- **Active search** vs RACER and FALCON: search time 54.49 s vs 176.62 / 146.67 s, with similar path length (Table 4, p7).
- **Ablation at 5 m/s** (short SR / long SR):
  - without search: 0.88 / 0.52;
  - without tracking loss: 0.85 / 0.82;
  - without data construction: 0.80 / 0.76;
  - without guidance point: 0.70 / 0.33;
  - full: 0.93 / 0.89.
  - Center Dis 0.57 m (full) vs 0.82 without the tracking loss (Table 5, p8).
- **Real world:** underground parking lot and forest, qualitative only (p8, Fig. 8).
- **Metrics:** short/long SR (defined above), search time / distance / speed, cycle time, Ave/Min obstacle distance, Center Dis (average point-to-centre distance in the FoV).

### Limitations
- **Stated (p8):** "relies on forward-facing RGB-D sensing, which restricts safe retreat during close interactions". Future work: multi-view perception.
- **Observed:**
  - Thresholds and horizon are missing from this PDF.
  - The innovation is normalised by R only.
  - The KF covariance is used only for gating; it does not drive search.
  - Exploration is a heuristic (farthest unexplored point / spiral). It ignores target-motion belief except through the last estimate.
  - The fixed template has no re-ID.
  - Few evaluation trajectories (40 / 10).
  - Training needs privileged SDF maps.
  - Real-world results are qualitative only.

### Code / weights
- `https://github.com/qinliangql/UAST` (p1). Weights are not mentioned.

### MOTAR relevance (highest of the five)
- Closest problem match: a quadrotor with forward RGB-D, LiDAR-based odometry in the real system, dense pillar clutter, target loss and recovery, and explicit first-acquisition search.
- **Reusable in MOTAR without touching the frozen policy:**
  1. Central-box median depth back-projection (Eqs. 2–3) as the 3D measurement for the MOTAR KF.
  2. Innovation gating to reject outlier detections. MOTAR should use the full S, not R (observed).
  3. The **guidance-point substitution pattern**: feed the KF-predicted or search-generated point to the controller in place of the detection. If the frozen 898-D actor observes a target-relative position, this is a no-retrain "goal rewrite" lever.
  4. FOV-swept explored-region bookkeeping plus farthest-unexplored / spiral fallback as a search goal generator.
  5. L_occ/L_fov as a **template for offline labels** of LOS/FOV loss.
- **Conflicts:**
  - The control network *replaces* the policy, and its outputs are polynomial p/v/a references, not velocity/yaw-rate from a frozen actor.
  - Its learned score needs SDF-supervised training.
  - Template correlation (AlexNet) is weaker than MOTAR's detector assumptions [observed].

### BOIA overlap check
- (a) Forecast of future detection probability: **Partly.** The learned score s_{v,h} regresses L_traj, which includes an SDF-based **LOS-occlusion** term and an **FOV** term over the mid-horizon (Eqs. 11–13, 17, p5–6). So candidate scoring does carry a learned visibility cost, but:
  - it is a deterministic geometric cost, not a probability;
  - it is not conditioned on belief covariance or detector reliability;
  - the target position is a single guidance point.
  The belief m_t only steers the guidance point (Eq. 1, p4).
- (b) Finite feasible candidates scored: **Partly (strong structural overlap).** A finite V×H grid of end-state candidates, each joined by a quintic polynomial (p/v/a continuous), scored by a learned cost; the arg-min is executed (p5, Fig. 3, Eq. 7). The score is a learned surrogate of track+goal+smooth+safety loss, **not** capture progress + loss probability + covariance growth. There are no explicit dynamics limits beyond polynomial continuity.
- (c) Calibrated-advantage arbitration over a frozen policy: **No.** UAST is the sole policy. Mode switching is rule-based (τ_det, τ_innov, T_lost; p4–5), with no advantage estimate, calibration, or hysteresis described.
- (d) Safe fallback + terminal-first: **No / partly.** Safety exists only as a training loss (L_safe, Eq. 16) and the 0.2 m clearance in the success criterion (p7). There is no runtime safety fallback and no terminal handling.

---

## 3. TrackVLA — "TrackVLA: Embodied Visual Tracking in the Wild"

- **Authors / venue as printed:** Wang, Zhang, Li, Liu, Li, Wu, Zhong, Yu, Zhang, Wang (PKU, Galbot, et al.). arXiv:2505.23189v1 [cs.RO], 29 May 2025 (p1). The venue is not printed on the paper. OA-VAT cites it as "CoRL 2025" (OA-VAT p6), which is not verified here.
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/TrackVLA__2505.23189v1.pdf`. 25 pp. sha256 prefix `c27d8f435b32a831`. Title metadata matches.
- **Pages read:** extracted text of p1–p25 (the main paper p1–9 in full, references, appendix A–H). Page image check: p7 (Tables 1–3). Figure-only pages p16, p19, p22–25 were reviewed through their captions.
- **Read status: FULL_READ** at text level (METHOD_REVIEWED was required).

### Problem and assumptions
- Task: embodied visual tracking (EVT). Given a natural-language description of the target and an egocentric RGB frame sequence, output the next action a_T ∈ {v, ω} (linear and angular velocity). Success = maintaining a 1–3 m following distance while facing the target (p3).
- Sensors: monocular RGB only, at 224×224 model input. Real camera: RealSense D455 RGB at 640×480, HFOV 90° (p14, p20). No depth or LiDAR in the model. On the real robot, LiDAR is used by a **separate** elastic-band obstacle avoider (p21).
- Platform: Habitat 3.0 simulation (EVT-Bench) and a Unitree GO2 **quadruped** (ground robot) in the real system (p5, p20).
- Targets: humanoid avatars (100 generated, SMPL-X; walking speed 1.0–1.5 m/s; ORCA collision avoidance) (p5). Gym-UnrealCV also includes unseen animals (p17).
- Map: none.

### Pipeline
1. **Encoding (p4).** EVA-CLIP (frozen) gives N = 256 patches × C = 1408. Grid pooling gives 64 tokens for the current frame (fine) and 4 tokens per frame for history (coarse). A sliding window of k = 32 frames. A 2-layer MLP projector maps into the LLM space (Eq. 1).
2. **LLM (Vicuna-7B) forward (p3–4).** Visual tokens, instruction tokens and a special `[Track]` token. In tracking mode there is a single autoregressive step, and the hidden state E_T^pred conditions the action model (p5).
3. **Anchor-based diffusion action head (p4–5, Fig. 3):**
   - M = 40 trajectory anchors from K-means over training trajectories, each with N_w = 10 waypoints (x, y, θ) (Table 5, p15);
   - anchors are noised and passed to a DiT, giving denoised trajectories plus classification scores ŝ_i (Eq. 2);
   - 2 DDIM steps at inference; the top-1 score trajectory is executed (p5, p14).
4. **Training:** L_track = Σ_i [s_i MSE(τ̂_i, τ_gt) + λ BCE(ŝ_i, s_i)], where the anchor nearest τ_gt is positive (Eq. 3, p5). L = L_track + α L_text (p5). Co-training uses 855K tracking + 855K VQA/recognition samples (1:1 was best, Fig. 6 p8).
5. **Low-level control (real):** trajectory waypoints go to the robot, which runs pure pursuit with its pose for v/ω closed-loop control, plus LiDAR elastic-band avoidance (p21). In simulation, the agent is commanded with linear/angular velocities (p15).
- **No explicit search/track/recovery state machine.** Recognition and planning are implicit in the VLA. Identity comes from the language description plus the history window.

### Key equations
- Eq. 1 (p4): grid pooling to 64 or 4 tokens.
- Eq. 2 (p4): {ŝ_i, τ̂_i} = A_θ({τ̃_i}, E_T^pred).
- Eq. 3 (p5): the anchor-classification plus regression loss.

### Occlusion, re-acquisition, identity, first acquisition
- Occlusion: no explicit module. It is handled implicitly through the 32-frame history (removing history drops DT SR 57.6→29.9; Table 10, p19) and through training data.
- Re-acquisition: not defined. TR (tracking rate) = fraction of steps successfully tracking (p15).
- Identity: the language description; the DT (fine-grained description, distractors) and AT ("first person you see", identical distractors) splits (p6); the recognition test on SYNTH-PEDES pairs gives ACC 80.7% at 10 FPS (Table 3, p7).
- First acquisition: the episode starts with the agent near the target, facing it within ±30° (p14–15). No search.

### Online inputs vs ground truth
- **Online:** RGB plus the language instruction only. No GT position or masks.
- **Training:** expert trajectories τ_gt from the simulator (p6, Sec. 4.1); simulator collision/rendering. The VLM supplies avatar descriptions (p5).

### Timing / compute / model size
- Above 7B parameters (Vicuna-7B + EVA-CLIP; OA-VAT lists ">7B", OA-VAT p6).
- 10 FPS inference on an RTX 4090 on a remote server (p1, p7, p21). Action head: DiT-Base 89M at 13 ms; DiT-Small 13M at 8 ms (Table 4, p8).
- Training: 24×H100 for about 15 h = 360 GPU-h, one epoch, lr 2e-5, batch 196 (p14).
- Observed: a 7B fp16 model does not fit an 8 GB GPU without heavy quantisation.

### Experiments
- **Gym-UnrealCV (zero-shot):**
  - EL = average steps over 100 episodes; SR = fraction of full 500-step episodes in which the target stays in a 90°, 750 cm fan; failure after >50 consecutive steps outside (p16).
  - Single Target 500/1.00, Distractor 474/0.91, Unseen Objects 500/1.00 (Table 1, p7).
- **EVT-Bench** (SR = ends oriented toward the target at 1–3 m; TR = S/L steps tracked; CR = fraction of episodes ending in collision **with the target humanoid**, p15), SR/TR/CR:
  - STT 85.1/78.6/1.65;
  - DT 57.6/63.2/5.80;
  - AT 50.2/63.7/17.1 (Table 2, p7).
  - Baselines: IBVS, PoliFormer, EVT, Uni-NaVid.
- **Real world** (quadruped vs a DJI Flip drone, 10 trials each), Easy/Medium/Hard: 100/90/70% vs 100/70/50% (Table 13, p21).

### Limitations
- **Stated (p9):** egocentric observation only, with a narrow FOV (typically 90°); uses only a waypoint controller and lacks a flexible local motion controller.
- **Observed:**
  - Ground robot with 2D actions.
  - Offboard 4090.
  - The collision metric ignores static obstacles.
  - SR is judged only at episode end.
  - No explicit uncertainty or occlusion reasoning.
  - Training code/checkpoints were reported unavailable by OA-VAT (OA-VAT p6).

### Code / weights
- Project page `https://pku-epic.github.io/TrackVLA-web` (p1). "We will make TrackVLA and EVT-Bench publicly available" (p2).

### MOTAR relevance
- Low for direct reuse: ground robot, 2D (v, ω), 7B scale, RGB-only, and it replaces the policy.
- Useful as benchmark/metric design: separate distractor and ambiguity splits; TR (per-step tracked fraction) vs SR (terminal); the history-window ablation.
- Its anchor-scoring head is a *learned imitation* analogue of a finite candidate set.

### BOIA overlap check
- (a) Forecast of future detection probability: **No.** Nothing explicit (p3–5).
- (b) Finite candidates scored: **Partly.** M = 40 fixed K-means anchors, refined and scored, with the top-1 executed (p4–5, p14). The score is an imitation "nearest-to-expert" classifier (Eq. 3), not capture progress, loss probability or covariance. Feasibility is implicit in the data.
- (c) Arbitration over a frozen policy: **No.**
- (d) Safe fallback: **Partly (deployment only).** A separate LiDAR elastic-band obstacle-avoidance layer runs on the robot (p21). It has no terminal-first logic.

---

## 4. TrackVLA++ — "Unleashing Reasoning and Memory Capabilities in VLA Models for Embodied Visual Tracking"

- **Authors / venue as printed:** Liu, Qi, Zhang, Li, Wang, Wu, Ye, Zhang, Chen, Zhong, Zhang, Wang (PKU, Galbot, USTC, BAAI, et al.). arXiv:2510.07134v1 [cs.RO], 8 Oct 2025 (p1). Venue not printed.
- **Path:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/TrackVLApp__2510.07134v1.pdf`. 9 pp. sha256 prefix `7f7de4b53d6d7503`. Title metadata matches.
- **Pages read:** text of p1–p9, all of it. Page images checked: p3–p7 (Fig. 2, Eqs. 1–10, Tables I–IV, Fig. 5).
- **Read status: FULL_READ.**

### Problem and assumptions
- Task: EVT with a language description. The model predicts a continuous waypoint trajectory W_T = {w_i}, w_i = (x, y, θ) in the egocentric frame, from N cameras (single front view or four views). Success = keeping a predefined following distance D (p3).
- Sensors: RGB only (one or four cameras). Real system: Unitree GO2 with four SG3S11AFxK cameras, Jetson AGX Orin onboard, and a LiDAR-L1 visible in Fig. 3; inference runs on a remote RTX 4090 server (p5–6). The LiDAR's role is not described [observed].
- Simulator: Habitat 3.0 / EVT-Bench, with Gym-UnrealCV zero-shot (p5). Ground platform. No map.

### Pipeline
1. **Encoding (p3).** SigLIP + DINOv2 dual encoders, grid pooling (64 fine tokens for the current frame, 4 coarse tokens for history), a sliding window k = 32, and an MLP projector (Eq. 1). Built on NavFoM; the LLM is Qwen2-7B (p3).
2. **Polar-CoT (p4).**
   - The LLM emits one reasoning token E_T^CoT (Eq. 2) that encodes the target's quantised (angle, distance) sector.
   - The annulus 0.6–5.0 m is split into 60 angular × 30 distance bins (p5).
   - An extra `<invalid>` token means occluded or outside the FOV (p4).
3. **Target Identification Memory (TIM) (p4):**
   - M_T = (1−w_T) M_{T−1} + w_T f_{T−1} (Eq. 3);
   - f_{T−1} is the fine-grained feature at the sector predicted by the CoT token;
   - w_T = C_{T−1} / (C̄_{T−2} + C_{T−1}), where C̄ is the historical mean confidence (Eq. 4);
   - C_{T−1} = 1 − H(softmax(P))/log K, the normalised entropy of the CoT logits (Eq. 5);
   - `<invalid>` forces C = 0, which **freezes memory** until a confident re-detection;
   - initialised from the first valid feature (p4);
   - TIM = 4 tokens (p5).
4. **Action (p4).** A second LLM pass with [E^M, E^V, E^L, E^CoT] gives the action token; an MLP head outputs **8 waypoints** (Eqs. 6–7; p5).
5. **Loss (p4):** L = L_traj + 0.2 L_reason + 0.5 L_text (Eq. 8). L_traj is the MSE over waypoints (Eq. 9); L_reason is the NLL of the CoT token (Eq. 10).
6. **Low-level control:** waypoints. The robot-side controller is not described in this paper (observed).
- **State machine:** none explicit. The only "mode" is the `<invalid>` token's effect on the memory gate.

### Occlusion, re-acquisition, identity, first acquisition
- Occlusion: explicitly predicted as `<invalid>` (p4), with memory frozen during absence.
- Re-acquisition: "until the target is confidently re-detected" (p4). There is no separate metric. The real-world "Obstacle" task tests re-identification after occlusion: 86% vs 72% for TrackVLA (Fig. 5 bar chart, p7).
- Identity: TIM plus language. The recognition benchmark gives ACC 87.5% at 4.8 FPS (Table III, p6). DT-split gains in the ablation: Polar-CoT +6.0 SR, TIM +2.8 SR (Table IV, p7).
- First acquisition: not addressed.

### Online inputs vs ground truth
- **Online:** RGB (1–4 views) plus language. No GT.
- **Training:** Polar-CoT labels come from the simulator's **GT relative angle and distance**. `<invalid>` labels come from **simulator semantic masks** (target mask < 2,500 px means occluded or too far) (p5). Expert trajectories W_gt come from the simulator (p5). These are training only.

### Timing / compute / model size
- About 7B+ parameters (Qwen2-7B + SigLIP + DINOv2).
- 4.8 FPS; 5.2 FPS without Polar-CoT (Table III, p6). Remote RTX 4090 (p6).
- Training: 8×H100 for about 1 day = 192 GPU-h, on 1M tracking + 1M QA samples (p5).

### Experiments
- EVT-Bench, same metrics as TrackVLA:
  - single view: STT 86.0/81.0/2.10, DT 66.5/68.8/4.71, AT 51.2/63.4/15.9;
  - four views: STT 90.9/82.7/1.50, DT 74.0/73.7/3.51, AT 55.9/63.8/15.1 (Table I, p5).
- Gym-UnrealCV, front view: 500/1.00, 484/0.92, 500/1.00 (Table II, p6).
- Real world: Obstacle / Winding Path / Distractor success 86 / 79 / 94% vs TrackVLA 72 / 72 / 77% (Fig. 5, p7; the bar chart is the only source of these numbers).

### Limitations
- **Stated:** no limitations section in the 9 pages (observed).
- **Observed:**
  - The `<invalid>` supervision relies on simulator masks with an arbitrary pixel threshold.
  - Polar bins only span 0.6–5 m.
  - Ground robot.
  - 7B scale at about 5 FPS.
  - Real-world numbers come only from a bar chart.
  - Increasing TIM from 4 to 16 tokens gives no gain (Table IV, p7).

### Code / weights
- Project page `https://pku-epic.github.io/TrackVLA-plus-plus-Web/` (p1). No code statement found.

### MOTAR relevance
- Not deployable on an 8 GB GPU.
- Transferable ideas:
  1. An explicit **visibility/occlusion class** (`<invalid>`) supervised from simulator masks, as a cheap label for training a "target visible?" head. MOTAR already has simulator ground truth.
  2. An **entropy-based confidence that gates memory/appearance updates**, with a freeze on invalid. This parallels OA-VAT's confidence→R and could gate MOTAR's KF and appearance updates.

### BOIA overlap check
- (a) Forecast of future detection probability: **Partly (weak).** `<invalid>` plus an entropy confidence predict **current** target presence/visibility (p4). That is not a forecast over future candidates, and it is not FOV/LOS-decomposed.
- (b) Finite candidates scored: **No.** An MLP regresses a single 8-waypoint trajectory (Eq. 7, p4–5).
- (c) Arbitration: **No.** The confidence gate controls memory updates, not policy selection (Eqs. 3–5).
- (d) Safe fallback / terminal-first: **No.**

---

## 5. DAT / GC-VAT — "Open-World Drone Active Tracking with Goal-Centered Rewards"

- **Authors / venue as printed:** Sun, Hu, Zhang, Tian, Xie, Wang, Xie, Lin, Yu, Tan (SCUT et al.; the same group as OA-VAT). "39th Conference on Neural Information Processing Systems (NeurIPS 2025)"; arXiv:2412.00744v2 [cs.RO], 22 Oct 2025 (p1).
- **Path:** `/home/fair/Downloads/Open-World Drone Active Tracking with Goal-Centered Rewards.pdf`. 36 pp. sha256 prefix `a54dc19d3e832df0`. Title metadata matches.
- **Pages read:**
  - main paper p1–p10 in full (page images p5–p8 checked: Eqs. 1–7, Alg. 1, Tables 2–4);
  - references p11–p15 skimmed;
  - appendix prose p16–p29 read (A related work, B benchmark incl. Tables 8–9, C reward proof and implementation incl. Tables 10–11, D baselines, E experiments, F limitation p29);
  - per-scene appendix Tables 12–17 (p23–p26) **skimmed only** from garbled extraction, numbers not verified;
  - NeurIPS checklist p30–p36 skimmed.
- **Read status: METHOD_REVIEWED** (main paper full read plus appendix prose; per-scene tables not verified).

### Problem and assumptions
- Task: drone visual active tracking of a **ground** target ("air-to-ground"), keeping the target at the image centre (p2–3).
- Observation: a single 84×84 RGB image per step as the MDP state (p3, p4). p2 mentions "visual and motion sensor data", but the MDP state is defined as the image only (p4) [observed inconsistency]. The benchmark *offers* camera, LiDAR (RPLIDAR A2), GPS, IMU, accelerometer and gyro (Table 8, p19).
- FOV: a parameter (cameraFov, Table 9). A tilted gimbal: pitch randomised in [0.6, 1.38] rad and altitude in [13, 22] m during training; tests use 22 m and 1.37 rad (p22, p27).
- Action: **discrete 7 actions** (forward, backward, left, right, turn left, turn right, stop) (p4). The benchmark also supports continuous velocity actions (p3). Drone translation speed 40 m/s and rotation 2 rad/s in training/test settings (p27) [observed: deliberately fast relative to targets].
- Simulator: Webots with full drone dynamics (DJI Matrice 100 with a 3-axis gimbal) and SUMO-driven vehicles. 24 scenes = 6 maps × 4 weathers. A digital-twin tool generates scenes from OpenStreetMap (p3–4, p18–19). Real system: DJI Mini 3 Pro (p10).
- Targets: automobile, motorbike, pedestrian, wheeled robot, legged robot; 24 targets (p4, Fig. 9 p18). Up to 40 vehicles at ≤20 m/s, ±25 m/s² (p27).
- Map: none at inference.

### Pipeline (GC-VAT)
- End-to-end RL: CNN (C8×8-16S4, C4×4-32S2) → FC200 → FC100 → GRU256, with an actor FC7 and a critic FC1 (Fig. 11, p21).
- Trained with PPO (γ = 0.9, λ_GAE = 0.95, entropy 0.01, clip 0.2; p24) and 35 parallel envs, for 9.2–21.3M total steps per scene. Webots runs at 500 Hz; the policy updates every 4 steps (125 Hz). Episodes are ≤1500 steps, terminated after >100 consecutive steps of loss, a collision, or a crash (p27, Table 10 p22).
- **Curriculum-Based Training (Alg. 1, p6):**
  - phase 1: straight-line target, no obstacles;
  - switch when the mean reward ≥ η;
  - phase 2: complex scenes with occlusion.
  - Transition steps are in Table 11 (p22).
- **Domain randomisation (p7, p22):** initial relative angle [−π, π], horizontal/vertical offsets, gimbal pitch.
- There is no detection, identity, KF or state machine. Everything is implicit in the recurrent policy.

### Key equations
- Eq. 1 (p5): ϕ(P_g, C_g) = |P_g − C_g| / |E_g(P_g, C_g) − C_g|. This is the deviation of the target's ground point from the ground projection of the image centre, normalised by the distance to the projected image boundary (a trapezoid) along the same ray.
- Eq. 2 (p5): the reward monotonicity principle.
- Eq. 3 (p5): r_gc = tanh(α(1−ϕ)³) if P_g ∈ I_clip, else 0; α = 4, λ_clip = 0.7.
- Eqs. 4–5 (p6): corner-ray/ground-plane intersections for the projected image footprint. The assumption is a flat ground plane z = h ("road surfaces are smooth", p6).
- Proposition 1 / Eqs. 6–7 (p7), proof in C.1 (p21): Euclidean ground distance is misaligned with image-plane deviation when the camera is not horizontal-forward.
- Metrics (p7):
  - CR = Σ_{t=1}^{E_l} r_gc;
  - TSR = (1/E_ml) Σ_{t=1}^{E_l} r_dt × 100%, where r_dt = 1 if the target is in the image (sparse reward Eq. 14, p23) and E_ml is the maximum episode length.
- PPO objectives: Eqs. 15–19 (p24).

### Occlusion, re-acquisition, identity, first acquisition
- Occlusion: only through training exposure (trees, tunnels; the village map "requir[es] the agent to predict the target's movement when it is fully obscured", p3) and the curriculum's phase 2 (p6–7). No explicit mechanism.
- Re-acquisition: not defined. Episodes terminate after 100 consecutive lost steps (p27).
- Identity: none explicit. The distractor test (a similar-looking vehicle near the target) gives TSR 0.91 vs 0.94 baseline (Table 5, p9); an unseen class (bus) is also tested (Table 6, p9).
- First acquisition: none. The target is initialised at the camera centre (p3, p27).

### Online inputs vs ground truth
- **Online:** 84×84 RGB only (p4).
- **Training:** the reward uses **privileged GT target pose and camera HTM** (T_cw, T_tw, carMidGlobalPos, carDronePosOri, Table 9, p19). The authors say: "During training in the simulator, the drone agent has access to additional information (e.g., the precise location of the target). However, during testing and real-world deployment, such privileged knowledge is not available" (p22).

### Timing / compute / model size
- Small CNN-GRU; the parameter count is not stated.
- Real deployment: laptop with an RTX 3050 + Intel i5, connected through the DJI Mobile SDK, at >30 FPS (p10).
- Training GPU and wall-clock time are not stated in the pages I read (only step counts, p22, p27).

### Experiments
- Settings: within-scene, cross-scene (unseen scenes, same weather), cross-domain (same scene, other weather). 40 episodes = 4 initial relative angles × 10 (p8).
- Context (D-VAT → GC-VAT, averages):
  - within-scene TSR 0.19 → 0.72 (CR 35 → 242);
  - cross-scene TSR 0.19 → 0.57;
  - cross-domain TSR 0.19 → 0.67 (p8, Tables 2–3).
- The baselines AOT and D-VAT were re-implemented with discrete actions and modified parameters (p25–27). Both essentially fail under the tilted view (p8).
- Ablations (farmland, Table 4, p8): the D-VAT reward gives TSR 0.06; without CBT 0.23; removing angle randomisation hurts most.
- Robustness: wind (forward/lateral 2.5–7.5 m/s, yaw-rate disturbance 0.05–0.15 rad/s) costs < 0.06 TSR (p9, p29). Rain costs < 0.07 TSR (Table 6, p9).
- Real images: CAR 0.795 / 0.833 / 0.802 on VOT / DTB70 / UAVDT (Table 7, p9).
- Real drone: zero-shot TSR 88.4% and CAR 81.3% (p10).
- Note the large ± spreads, about ±0.3 TSR (Tables 2–3).

### Limitations
- **Stated (App. F, p29):** "deploying the algorithm in truly open environments remains highly challenging … numerous similar interfering objects and the high complexity of real-world conditions … significant gap compared to simulated environments."
- **Observed:**
  - Discrete actions.
  - 40 m/s tracker speed.
  - The target starts centred.
  - No search, re-ID or uncertainty.
  - The reward assumes flat ground.
  - High variance.
  - "Within scene: we train the model on all scenes and evaluate it on the original scene" (p8) is ambiguous given the per-scene training steps in Table 10.

### Code / weights
- `https://github.com/SHWplus/DAT_Benchmark` (p1). The checklist says data and code are provided (p32).

### MOTAR relevance
- Low for the method: it trains a new RL policy, while MOTAR's policy is frozen; it uses discrete actions and a downward-tilted camera on ground targets.
- Useful items:
  1. TSR as a per-step in-view fraction with a lost-streak termination (p7, p27). This is a clean metric template.
  2. The CAR protocol for real-video sanity checks (p9–10).
  3. The explicit "privileged info only for reward" convention (p22), which matches MOTAR's GT-free-online rule.
  4. Eq. 1's footprint-normalised deviation, if MOTAR ever scores image-centring with a tilted camera.

### BOIA overlap check
- (a) Forecast: **No.**
- (b) Candidate scoring: **No.** It uses a discrete 7-action policy (p4).
- (c) Arbitration: **No.**
- (d) Safe fallback / terminal: **No.** Collision only ends the episode (p27).

---

## Cross-paper summary table

| Paper | Task | Sensors (model input) | Identity mechanism | State machine | Planner / policy | Control interface | GT at inference? | Compute (as printed) | Code |
|---|---|---|---|---|---|---|---|---|---|
| OA-VAT (arXiv 2604.21453; "CVPR 2026" in own table) | Instance-level VAT with occlusion recovery; UnrealCV, DAT, Tello | RGB + 1 reference image | DINOv3 multi-view prototype + cosine match (η_s = 0.5) + EMA (β = 0.8); ORTrack tracker | Rule-based: detect (b = ∅) → track → planner when c_t < η_c / no z_t for "many" frames (p4–5) | Confidence-aware image-plane KF (Eq. 9) + diffusion planner conditioned on image + KF-predicted bbox, trained on A* labels (Planning-20k) | Velocity [v_f, v_l, v_v, ω_y]; Tello velocity mode | No (GT pos + occupancy map only for training labels) | 35 FPS RTX 3090; 584M params; 15 h on one 3090 | github.com/SHWplus/OA-VAT |
| UAST (CVPR, open-access version) | Unified search + tracking, UAV, clutter | RGB-D + v, a + template (+ pose for world frame; Fast-LIO in real) | AlexNet Siamese template correlation (template fixed) | Rule-based 3 modes: visible (m_t > τ_det) / temporarily invisible (KF + innovation gate τ_innov) / lost (T_lost, or no detection at start) → exploration | Rule-based guidance point + control net scoring a V×H grid of quintic end-state candidates (learned cost incl. LOS/FOV) | p/v/a references from quintic polynomial | No (SDF + GT targets only in training) | 8.6 ms/step (i9 + RTX 4090); Jetson NX onboard; 8 h training | github.com/qinliangql/UAST |
| TrackVLA (arXiv 2505.23189) | Language-specified EVT, ground robot | Egocentric RGB (224², 90° HFOV real) + text | Implicit in VLA; language + 32-frame history; VQA co-training | None explicit | Vicuna-7B VLA + anchor diffusion head (40 anchors, top-1 score) | Waypoints → pure pursuit v/ω; LiDAR elastic band on robot | No (expert trajectories for training) | >7B; 10 FPS RTX 4090 (remote); 360 H100-h | Project page (release promised) |
| TrackVLA++ (arXiv 2510.07134) | Language EVT, 1 or 4 cameras, ground robot | RGB 1–4 views + text | TIM memory gated by Polar-CoT entropy confidence; frozen on `<invalid>` | None explicit (`<invalid>` token only gates memory) | Qwen2-7B VLA; Polar-CoT token → MLP 8-waypoint head | Waypoints (robot controller not described) | No (GT angle/distance + sim masks for training labels) | 4.8 FPS remote RTX 4090; 192 H100-h | Project page |
| DAT / GC-VAT (NeurIPS 2025) | Air-to-ground drone VAT benchmark + RL method | 84×84 RGB only | None (implicit; distractor test only) | None | CNN–GRU PPO policy; goal-centred reward + curriculum | Discrete 7 actions (benchmark also continuous velocity) | No (privileged pose only in reward) | >30 FPS on laptop RTX 3050; params not stated | github.com/SHWplus/DAT_Benchmark |

### BOIA overlap at a glance
| | (a) detection-prob. forecast | (b) finite feasible scored candidates | (c) calibrated-advantage arbitration over frozen policy | (d) safe fallback + terminal-first |
|---|---|---|---|---|
| OA-VAT | No (current-confidence → R only, p4) | No (single diffusion sample, p5) | Partly-weak (rule-based switch, no advantage/hysteresis, p4–5) | No |
| UAST | Partly (learned cost includes SDF-LOS + FOV terms, p6; not a probability; not belief/detector-conditioned) | **Partly-strong** (V×H quintic end-state grid, arg-min of learned cost, p5) | No (rule-based modes; UAST is the policy) | No (training-loss safety only, Eq. 16) |
| TrackVLA | No | Partly (40 anchors + imitation score, p4–5) | No | Partly (deployment LiDAR elastic band, p21) |
| TrackVLA++ | Partly-weak (current visibility `<invalid>` + entropy confidence, p4) | No | No | No |
| DAT/GC-VAT | No | No | No | No |

Net reading (observed): none of these five papers combines a forecast of detection probability under belief with risk/covariance-aware scoring of a finite candidate set, and none arbitrates against a frozen base policy with calibrated advantage and hysteresis. UAST is the nearest prior art for (b), and partly for (a) through its learned visibility-cost score. It must be cited and differentiated. The differentiators are probabilistic vs geometric-cost scoring, belief and detector-reliability conditioning, and the frozen-policy arbitration layer.
