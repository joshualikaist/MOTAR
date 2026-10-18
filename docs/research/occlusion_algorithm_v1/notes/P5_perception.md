# P5 — Perception: occlusion-robust tracking and perception-aware planning

## NOT_AVAILABLE (not in this batch, not described here)
- "Rethinking Occlusion Modeling for UAV Tracking" (OCTrack): NOT_AVAILABLE
- "You Only Plan Once" (YOPO): NOT_AVAILABLE

Conventions: "p.N" means the physical 1-based PDF page. Eq. numbers are the paper's own. All three PDFs were read page by page as rendered images with the Read tool, and pdftotext was used as a cross-check. Anything marked [UNCERTAIN] is a transcription or interpretation I am not sure about. Anything marked [MOTAR inference] is my own reasoning and does not come from the paper.

---

## Paper 1: ORTrack, "Learning Occlusion-Robust Vision Transformers for Real-Time UAV Tracking"

- **Authors / venue / year:** You Wu, Xucheng Wang, Xiangyang Yang, Mengyuan Liu, Dan Zeng, Hengzhou Ye, Shuiwang Li (Guilin Univ. of Technology, Fudan, Sun Yat-sen). arXiv:2504.09228v1 [cs.CV], 12 Apr 2025 (p.1 margin). This PDF does not state a venue.
- **Path:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/ORTrack__2504.09228v1.pdf
- **Identity / version note:** This is arXiv **v1**, 11 pages. It is a **DIFFERENT version from the user's copy**, so page numbers, table numbers and figures (FPS values in particular) may not match the user's version. Treat all numbers below as v1 numbers.
- **Pages read:** 1–11, all of them, including the references on pp.9–11.
- **Read status:** FULL_READ. This covers more than the requested METHOD_REVIEWED+. The supplementary material mentioned on p.7 is not in this PDF.
- **Code:** https://github.com/wuyou3474/ORTrack (abstract, p.1).

### Problem
Single-stream ViT trackers for UAVs lack strategies for occlusions caused by buildings, trees and similar obstacles. They also need real-time efficiency on constrained platforms (p.1 abstract; p.2).

### Assumptions
- Standard single-object tracking: a template Z is given at initialisation, and a search region X is searched in later frames (p.3 §3.1).
- Random masking of the template is assumed to "approximately simulate" target occlusion. The paper states this as a hope ("Hopefully, this random masking approximately simulates target occlusions", p.1 abstract). It does not validate it against real occluder geometry.

### Pipeline (p.3 §3.1, Fig. 2 on p.4)
1. **Teacher (ORTrack).** The ViT backbone B_T takes [Z, X] and also [Z', X], where Z' = m(Z) is a masked template. An MSE loss enforces invariance of the template tokens (ORR, Occlusion-Robust Representations). Masking is applied only to the template and only during training. Inference uses the unmasked [Z, X], so ORR adds no inference cost (p.5).
2. **Student (ORTrack-D).** The student has the same ViT layer structure but fewer layers. The teacher is frozen, and the student is trained with the adaptive feature-based distillation loss AFKD (p.3, p.5).
3. **Head.** Corner-style Conv-BN-ReLU head outputs:
   - a local offset o,
   - a normalised box size s,
   - a classification score map p ∈ [0,1]^{Hx/P×Wx/P}.

   The target position is argmax p (p.5 §3.4).

### Key equations
- **Uniform masking m_U (p.3–4, unnumbered).** m_ij ~ U(0,1). b_ij = 1 if m_ij ∈ TopK(m, K), where K = ⌊(1−σ)H_zW_z⌉ [as printed; UNCERTAIN whether a /b² factor is implied]. Then m_U(Z) = Z ⊙ (b ⊗ 1_{b×b}). Patches are non-overlapping b×b, and σ is the masking ratio.
- **Spatial Cox masking m_C, Eq. (1), p.4.**
  - Γ ~ Poisson with mean ς.
  - Intensity λ(x,y) = Γ·e^{−(x²+y²)} / ∫_B e^{−(x²+y²)} dx dy. This is a bell shape over B, the central rect region of size H_z/b × W_z/b of the template.
  - Points Ξ are drawn by inhomogeneous-Poisson **thinning** (p.4). b'_ij = 1 if (i,j) ∈ Ξ, and m_C(Z) = Z ⊙ (b' ⊗ 1).
  - With ς = ⌊(1−σ)H_zW_z⌉, the expected masking ratio equals that of m_U. The random Γ also makes the masking ratio itself random, which gives "more diverse pattern of occlusion" (p.4).
  - Stated rationale (p.3): the target sits near the template centre, so masking should hit it with higher probability than uniform masking does.
  - [UNCERTAIN] As written, Z ⊙ (b'⊗1) *keeps* the patches where b'=1. That reads as centre-concentrated points being kept, which seems to contradict the stated intent of masking the target with higher probability. The prose and the formula are ambiguous about which way it goes. Check the code before reusing this.
- **ORR loss, Eq. (2), p.5:** L_orr = ‖t^L_{1:K_z}(Z,X;B_T) − t^L_{1:K_z}(Z',X;B_T)‖². This is the MSE between L-th-layer template tokens with and without masking.
- **AFKD, Eq. (3), p.5:** L_afkd = (α + β(L_iou − mean(L_iou)))·‖t^L_{1:K}(Z,X;B_T) − t^L_{1:K}(Z,X;B_S)‖². The distillation weight is linear in how far the GIoU loss deviates from its average, which serves as the difficulty measure.
- **Prediction loss, Eq. (4), p.5:** L_pred = L_cls (weighted focal) + λ_iou·L_iou + λ_L1·L_L1, with λ_iou=2 and λ_L1=5.
  - Teacher loss: L_T = L_pred + γ·L_orr, with γ = 2.0×10⁻⁴.
  - Student loss: L_S = L_pred + L_afkd.

### Online inputs vs GT
- Online: RGB frames plus the initial template only. There is no masking at inference (p.5).
- GT: boxes are used in training only, for the losses and for the GIoU-based difficulty.

### Timing / compute / hardware
- All experiments ran on an i9-10850K, 16 GB RAM and an NVIDIA TitanX (p.5).
- Table 1 (p.6), average over 4 benchmarks:

  | Model | GPU FPS | CPU FPS | FLOPs | Params |
  |---|---|---|---|---|
  | ORTrack-DeiT | 226.4 | 55.4 | 2.4 GMac | 7.9 M |
  | ORTrack-D-DeiT | 292.3 | 64.7 | 1.5 GMac | 5.3 M |

- Fig. 1 (p.1) reports 236 FPS for ORTrack-DeiT and about 313 FPS for ORTrack-D-DeiT on UAVDT. These differ from Table 3 (p.7, UAVDT): 218.4 FPS for DeiT and 298.7 for D-DeiT. The paper does not explain the difference [observed].
- Training (p.6):
  - backbones ViT-tiny, Eva-tiny and DeiT-tiny;
  - search 256², template 128²;
  - data: GOT-10k, LaSOT, COCO, TrackingNet;
  - batch 32, AdamW (wd 1e-4, lr 4e-5);
  - 300 epochs × 60k pairs, lr ÷10 at epoch 240.

### Experiments (numbers with page and context)
- Benchmarks: DTB70, UAVDT, VisDrone2018, UAV123. Compared against 26 trackers (p.5).
- Table 1 (p.6), ORTrack-DeiT:
  - average Prec 85.6 / Succ 65.0, the best average among lightweight trackers;
  - UAVDT 83.4 / 60.1;
  - ORTrack-D-DeiT average 83.7 / 63.7.
- Table 2 (p.7), VisDrone2018: ORTrack-DeiT Prec 88.6, Succ 66.8, 206.2 FPS. This is ahead of AQATrack (87.2, 53.4 FPS) and HIPTrack (86.7, 31.3 FPS).
- **Partial-occlusion subset of VisDrone2018** (Fig. 3, p.7):
  - ORTrack-DeiT precision 0.850, second to AQATrack at 0.852;
  - the same model without ORR (ORTrack-DeiT*) gets 0.781, so ORR gives +6.9 points.
- Ablation, Table 3 (p.7), UAVDT:
  - ORR raises Prec by +3.3 (ViT), +2.7 (Eva) and +4.8 (DeiT), and Succ by +2.6, +2.1 and +3.1;
  - AFKD gives more than 30% GPU speed-up (+36% for DeiT), with small accuracy loss.
- Masking operators, Table 4 (p.7), VisDrone2018 Prec/Succ:

  | Operator | Prec / Succ |
  |---|---|
  | none | 81.6 / 62.2 |
  | m_U | 86.7 / 65.4 |
  | **m_C** | **88.6 / 66.8** |
  | SAM | 86.8 / 65.6 |
  | AdaAutoMix | 84.3 / 63.8 |
  | CutMix | 85.7 / 64.2 |

- AFKD vs plain KD, Table 5 (p.8): LaSOT AUC 54.6 vs 54.0, TrackingNet AUC 73.7 vs 73.1.
- ORR added to ARTrack, GRM and DropTrack (Table 6, p.8) gives at least 1.2-point gains in both precision and success rate on UAVDT and VisDrone2018.
- Fig. 5 (p.8) shows attention and feature maps under 0/10/30/70% masking. The maps are qualitatively more consistent with ORR.

### Limitations
- **Stated:** there is no limitations section. The conclusion (p.8) makes no stated limits.
- **Observed:**
  1. Occlusion is simulated as random patch masking of the *template*, not as real occluder geometry. The search frame is never masked in training.
  2. The tracker gives no visibility or occlusion-state estimate and no calibrated confidence. The score map p is trained with focal loss and is not calibrated.
  3. Full-occlusion and re-detection behaviour is not evaluated beyond the "partial occlusion" attribute.
  4. FPS values are inconsistent across Fig. 1 and Tables 1 and 3.

### MOTAR relevance / conflicts
- **Relevance:**
  1. ORTrack is evidence that partial-occlusion robustness of a learned detector is trainable and measurable. Fig. 3 shows a gap of about 7 points between the with-ORR and without-ORR models on the partial-occlusion subset.
  2. A centre-weighted random mask (Eq. 1) is a cheap generator of partial-occlusion samples, usable for calibrating or stress-testing MOTAR's simulated detector.
  3. The model fits an 8 GB GPU easily (7.9 M params, 2.4 GMac, p.6), so it could be swapped in as a realistic image-based tracker [MOTAR inference].
- **Conflicts:**
  1. MOTAR's occluders are dense *vertical bars*. They produce strip-shaped, structured occlusion, including centre-only occlusion, which is not the bell-shaped blob of Eq. (1).
  2. ORTrack is a tracker that needs an initial template, while MOTAR's pipeline uses a per-frame detector.
  3. ORTrack does not say how often the target is missed outright. It reports precision and success, not detection probability.

### Novelty overlap vs BOIA
| BOIA element | Overlap | Evidence |
|---|---|---|
| (a) belief-conditioned p_det forecast (FOV+LOS+reliability) | **no** | Perception model only, with no forecasting or planning. It only supplies evidence that reliability degrades under occlusion (Fig. 3, p.7). |
| (b) finite dyn-feasible candidates scored by capture/loss/covariance | **no** | No planning. |
| (c) arbitration over frozen RL policy, threshold + hysteresis | **no** | Not addressed. |
| (d) safe fallback + terminal-first | **no** | Not addressed. |

---

## Paper 2: Perception-Aware Time-Optimal Planning for Quadrotor Waypoint Flight

- **Authors / venue / year:** Chao Qin, Jiaxu Xing, Rudolf Reiter, Angel Romero, Yifan Lin, Hugh H.-T. Liu, Davide Scaramuzza (UTIAS and UZH RPG). arXiv:2603.04305v1 [cs.RO], 4 Mar 2026 (p.1 margin).
  - The page header reads "IEEE Journal of LaTeX Class Files, February 2026". That is a template header, not a stated venue.
- **Path:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/PATimeOptimal__2603.04305v1.pdf
- **Identity:** v1, 21 pages, the same as the user's copy (per the brief).
- **Pages read:** 1–21, all of them, including the references and Appendices A–C on pp.20–21.
- **Read status:** FULL_READ.
- **Code:** I found no code URL. p.17 says "Please visit our website for detailed footages", but no URL appears in the text I read.

### Problem
Time-optimal quadrotor trajectories for racing or waypoint flight ignore visual-perception quality, so they can fail in closed loop with vision-based state estimation. The paper plans minimum-time trajectories with full dynamics, rotor limits, drag, FOV constraints, convex gate geometry and perception objectives (p.1 abstract; p.2 contributions).

### Assumptions
- Landmarks (gates) have known 3D geometry and global positions. Their features (corners) are observable, and pose comes from PnP on at least 4 non-collinear features (p.6 §IV).
- Bearing noise is isotropic: c_x=c_y, f_x=f_y, σ_u=σ_v. This simplification is required for Proposition 1 (p.7).
- Visibility is **FOV-only**: azimuth, elevation and minimum depth (Eq. 11, Eqs. 30–33). The visibility formulation contains no line-of-sight or occlusion term [observed].
- The real-world runs use motion-capture state estimation, with uncertainty evaluated offline (p.16).

### Pipeline
1. **Problem definitions.**
   - TOGT problem, Eq. 18 (p.5): min T subject to dynamics, initial and terminal state, state and input bounds, and gate constraints h_G ≤ 0.
   - Perception-aware TOGT, Eq. 19 (p.6): min T + ∫ L_PA dt, adding the visibility constraints of Eq. 12.
   - L_PA = L_LA + L_FOV + L_PUM + L_MR (Eq. 20).
2. **Three-step solver** (p.9–10, Fig. 9):
   - Step 1: polynomial TOGT on waypoints and times, Eq. 51.
   - Step 2: time refinement with M_p pieces per segment, Eq. 52, giving an optimality gap under 5% (p.10).
   - Step 3: multi-stage multiple shooting, Eq. 53, using an RK4 integrator with variable Δt_i, IPOPT, and CasADi.
3. **MPCTC tracking controller** (p.11, Eqs. 54–60). Tracking error is split into a progress component e^l (Eq. 58) and a contouring component e^c (Eq. 59), weighted separately. It is implemented with acados, HPIPM and real-time iteration (RTI).

### Key equations
- **State and dynamics.**
  - State, Eq. (1), p.3: x = [p, q, v, ω, f]. The individual rotor thrusts f ∈ R⁴ are part of the state.
  - Collective thrust and torques, Eq. (2), p.4: f_T = [0, 0, Σf_i], and τ is built from arm length l and torque constant c_τ.
  - Equations of motion, Eq. (3), p.4: v̇ = (1/m)R f_T + g − R D Rᵀ v (linear drag D), and ω̇ = J⁻¹(τ − ω×Jω). The input is the thrust rate r_T = ḟ.
- **Constraints (p.4).**
  - Rotor limits: |ω| ≤ ω_max, f_min ≤ f ≤ f_max, r_Tmin ≤ r_T ≤ r_Tmax.
  - Waypoint balls, Eqs. (4)–(5).
  - Polyhedral gates A_i p ≤ b_i, Eqs. (6)–(7).
- **FOV visibility (p.4–5).**
  - Eq. (8): p^c = R_bcᵀ(Rᵀ(q)(p^w − p) − t_cb).
  - Eq. (9): α = atan2(X, Z). Eq. (10): β = atan2(Y, √(X²+Z²)).
  - Eq. (11): |α| ≤ α_max, |β| ≤ β_max, Z > Z_min.
  - Eq. (12): soft version with slacks S. Eq. (13): L_FOV = w_FOV(1ᵀS + ½‖S‖²).
- **Look-ahead and motion terms (p.5).**
  - Eq. (14): optical axis z_c = R_k R_bc e_z.
  - Eq. (15): L_LA = −w_LA·exp(−λ_LA·arccos⁴⟨b, z_c⟩), where b is the direction to p(t+t_LA).
  - Eqs. (16)–(17): jerk penalty L_MR = w_MR‖j‖².
- **Camera model and bearing noise.**
  - Eqs. (21)–(27), p.6–7: fisheye equidistant model and bearing vector.
  - Eq. (28), p.7: Σ_ρ ≈ diag(σ_u²/f_x², σ_v²/f_y², (σ_u c_x/f_x² + σ_v c_y/f_y²)²). The proof is in Appendix A, Eqs. 61–68 (p.20).
- **Fisher information (p.7).**
  - Eq. (29): I_FIM = Σ_i Σ_j v_ij J_ijᵀ Σ_ρ J_ij.
  - **Eqs. (30)–(33), smooth visibility:** v̂ = v_α·v_β·v_Z, with
    - v_α = ½ + ½ tanh(λ_v(α_max − |α|)),
    - v_β = ½ + ½ tanh(λ_v(β_max − |β|)),
    - v_Z = ½ + ½ tanh(λ_v(Z_min − Z)).

    v_Z is transcribed as printed. [UNCERTAIN: the sign looks inverted relative to Eq. 11's Z > Z_min, so this is likely a typo.]
  - Eqs. (34)–(36): bearing Jacobian.
  - **Proposition 1, Eqs. (37)–(39):** with isotropic Σ_ρ, I_FIM(p) = Σ v_ij A_ijᵀ Σ_ρ A_ij, where A = (1/d)(I − ρρᵀ). This depends only on position, not on rotation. Proof in Appendix B, Eqs. 69–70 (p.20).
  - Eq. (40): L_PUM = −log det(I_FIM).
- **Single-landmark range uncertainty (p.8).**
  - Eq. (41): covariance fusion Σ̂⁻¹ = Σ_k Σ_k⁻¹.
  - Eq. (42): range from apparent size, d = f(L/l)cos φ.
  - **Proposition 2, Eq. (44):** σ_d = |√2 d²/(L cos φ f_x)|·σ_u. Proof in Appendix C, Eqs. 71–76 (p.21). It assumes independent Gaussian corner noise, giving σ_l = √2 σ_u (Eq. 75).
  - **Eq. (45):** σ_d'² = σ_d² + σ_u/(d⁴+ε). This is a correction term that grows as distance shrinks, because a close landmark may be **only partially visible**.
  - Eq. (46): Σ_ℓ = diag(σ_d'²,·,·). Eq. (47): centroid FIM.
  - **Eq. (48):** L'_PMU = −log det(I'_FIM + Σ v_i Σ_ℓi⁻¹). The algorithm is given as Algorithm 1 (p.8).
- **Tracking errors (p.11):** e^l = (e·t)t (Eq. 58) and e^c = (I − ttᵀ)e (Eq. 59). The MPC cost is Eq. (60).

### Online inputs vs GT
- **Planning is offline.** It uses the known landmark and gate map and the full model.
- The online controller (MPCTC) tracks the precomputed trajectory from the state estimate.
- Simulation closed loop: RotorS with an EKF that fuses simulated IMU with PnP poses, using the localisation module from [55] (p.15).
- Real world: motion-capture state, with uncertainty computed offline from the executed trajectory (p.16). So the real-world closed loop does **not** run on vision.

### Timing / compute / hardware
- Planning time is **seconds to minutes**. Table III (p.12), Split-S track:

  | Planner | Duration (s) | Compute (s) |
  |---|---|---|
  | TOGT-Ours | 13.33 | 68.63 |
  | TOWP-Ours | 14.22 | 79.37 |
  | CPC | 14.25 | 2631.0 |
  | Poly | 16.75 | 6.13 |
  | Fast-Fly | 14.25 | 256.48 |
  | PA-TOGT PUM | 14.20 | 331.11 |
  | PA-TOGT LA-FOV-PUM | 15.65 | 459.02 |

- Solver settings, Table II (p.12): M_p = 5, Δt̂ = 2 ms, tolerance 1e-5, at most 5000 iterations.
- I did not find an MPC rate or solve time in the text [not stated].
- Hardware (p.16, Fig. 18): NVIDIA Jetson Orin NX 16GB, IMX219 fisheye, Betaflight low-level rate control.
- Quadrotor parameters, Table I (p.12):
  - RPG: m 0.7 kg, f ∈ [0, 8.5] N, c_τ 0.033, ω_max [10, 10, 6] rad/s;
  - EXP: m 1.0 kg, c_τ 0.013.
- Camera, Table IV (p.13): α_max 128.1°, β_max 72.2°, Z_min 0.3 m, σ_u/v 10 px.
- Perception weights, Table V (p.13): w_jerk 1e-7, w_LA 0.005, λ_LA 3.0, w_FOV 0.05, w_PUM 0.0003, λ_v 10.

### Experiments (numbers with page and context)
- **TOGT vs TOWP.**
  - TOGT lap time is 3.84 s vs 4.14 s for TOWP, about 7.2% shorter. Total duration is 6.3% shorter, and the path is 12.4 m shorter (p.12).
  - TOGT is about 4× faster to compute than Fast-Fly and more than 30× faster than CPC (p.13).
- **Uncertainty evaluation protocol (p.13).** The trajectory is sampled every 0.05 s. When the uncertainty exceeds 2 m, the error is accumulated by +0.1 m per sample.
  - Pure PUM gives an analytical average of 0.2418 m and FOV-PUM 0.2758 m.
  - The PnP minimum average is 0.4271 m with std 0.6520 m (p.13).
  - PUM gives a median of 2 visible gates, vs 1 for LA and FOV (p.13, Fig. 14).
- **Cost of PUM in time:** only +0.72 s for TOWP and +0.87 s for TOGT (p.15).
- **FOV weight sweep** (Fig. 16, p.15): increasing w_FOV monotonically lowers the target-lost percentage but lengthens the duration. w_FOV = 0.01 already changes orientation behaviour substantially.
- **Closed-loop simulation**, Table VI (p.15), 20 runs each:

  | Method | Position uncertainty (m) | Tracking RMSE (m) | Success |
  |---|---|---|---|
  | TOWP | 0.3590 | 0.4192 | 55% |
  | LA | 0.3541 | 0.4612 | 67% |
  | FOV | 0.4634 | 0.3577 | 71% |
  | **PUM** | **0.2228** | **0.3239** | **100%** |

- **Real world.**
  - MPCTC has 0.07 m average and below 0.23 m maximum tracking error at up to 9.8 m/s. Standard MPC degrades as the thrust limit rises from 70% to 90% (p.16, Fig. 19).
  - Aggressive rotations cause severe motion blur (p.17).
  - FOV gives the best gate visibility: the shortest no-gate-visible interval is 12.1%, and FOV-PUM is 7.7%.
  - PnP-valid percentage: FOV 67.6%, FOV-PUM 65.8%, PUM 46.4%, LA-PUM 45.0%, None 37.3%, LA 35.9% (p.17, Fig. 22 on p.18).

### Limitations
- **Stated (p.17):**
  1. Only position uncertainty is modelled. Yaw drift is ignored.
  2. The cascaded control structure trades completion time against tracking error and needs tuning.
  3. Real-world gaps (p.17): missing key features and gate-corner detection noise cause "unanticipated large biases", so PUM's theoretical optimum is not reached. The cost-function objectives (LA, PUM) lose authority on dense, tight tracks.
- **Observed:**
  1. Planning is offline (minutes), not reactive.
  2. There is no obstacle line-of-sight or occlusion modelling.
  3. Landmarks are static and known. There is no moving target.
  4. The real-world loop uses mocap, not vision.
  5. Detection is assumed to succeed whenever a landmark is inside the FOV. There is no detection-probability model.

### MOTAR relevance / conflicts
- **Relevance:**
  1. The smooth FOV-visibility product (Eqs. 30–33) is a ready template for the FOV factor of a p_det model.
  2. The apparent-size range model (Eqs. 42–45) gives explicit range and angle (cos φ) dependence of measurement quality. It includes a close-range partial-visibility correction.
  3. The FIM/CRLB view (Eqs. 29, 40, 48) supports scoring candidates by covariance growth.
  4. The MPCTC split into progress and contouring errors is useful if MOTAR tracks candidate references [MOTAR inference].
- **Conflicts:**
  1. The visibility here concerns *ego-localisation landmarks*, not a moving target.
  2. Planning is offline and far too slow for online use.
  3. FOV-only visibility ignores exactly the bar line-of-sight occlusions that MOTAR cares about.
  4. The full rotor-level model is heavier than what MOTAR's frozen RL policy interface needs [MOTAR inference].

### Novelty overlap vs BOIA
| BOIA element | Overlap | Evidence |
|---|---|---|
| (a) p_det forecast (FOV+LOS+reliability) | **partly** | FOV visibility in smooth form (Eqs. 30–33, p.7). Range/angle-dependent measurement noise (Eqs. 44–45, p.8). No LOS term and no detector-reliability term. Not belief-conditioned on a moving target. |
| (b) finite dyn-feasible candidates scored by progress/loss/covariance | **no** (partly in spirit) | A single continuous NLP (Eq. 53, p.10) trades time against −log det FIM (Eq. 48) and a target-lost FOV penalty (Eq. 13). There is no finite candidate set. |
| (c) arbitration over frozen RL policy | **no** | RL is discussed only as an alternative paradigm, and as a possible consumer of expert demonstrations (p.17–18). |
| (d) safe fallback, terminal-first | **no** | Not addressed. |

---

## Paper 3: Perception-aware Planning for Quadrotor Flight in Unknown and Feature-limited Environments

- **Authors / venue / year:** Chenxin Yu, Zihong Lu, Jie Mei, Boyu Zhou (HIT Shenzhen, SUSTech, Differential Robotics). arXiv:2503.15273v2 [cs.RO], 30 Jul 2025 (p.1 margin). This PDF does not state a venue.
- **Path:** /home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/PAUnknownFeatureLimited__2503.15273v2.pdf
- **Identity:** v2, 8 pages, the same as the user's copy (per the brief).
- **Pages read:** 1–8, all of them.
- **Read status:** FULL_READ.
- **Code:** https://github.com/Robotics-STAR-Lab/LA-Planner (footnote, p.1).

### Problem
Navigate to a goal in **unknown**, feature-sparse or dark environments using VIO. The planner must balance *localizability* (enough tracked features) against *exploration* (seeing unknown space for safety and progress) without heavy per-feature visibility evaluation (p.1–2).

Failure is defined as any of: a collision, failure to localise (excessive estimation error), or a timeout (p.2 §III).

### Assumptions
- No prior map. Only the onboard camera and IMU are used (p.2 §III).
- The platform carries an RGB-D camera, and state estimation is feature-based VIO (p.2 §IV).
- The quadrotor is differentially flat, so position and yaw are planned separately (p.3).
- VIO tolerates a maximum duration t_f of visual degradation by integrating the IMU (p.5).

### Pipeline (Fig. 2, p.3)
1. **Perception.** Obstacle map, feature map and VIO.
2. **Adaptive local target selection (§V-A).**
   - A **viewpoint transition graph** of v = (p, ψ) nodes contains three kinds of node:
     - the current state v₀;
     - frontier-based viewpoints, which see unknown space and enough features;
     - feature-based viewpoints, which bridge two feature clusters.
   - Dijkstra runs to each frontier viewpoint. The planner then selects by a reward that trades time against goal-directed exploration gain.
3. **Position trajectory (§V-B).** Kinodynamic A* in known space, then B-spline optimisation (Eq. 10).
4. **Yaw trajectory (§V-C).** Three stages:
   - a graph search over yaw samples with visibility and co-visibility constraints (Eqs. 11–12);
   - a localizable corridor of per-node yaw bounds (l_i, u_i) (Eq. 15);
   - MINCO yaw optimisation within the corridor, maximising frontier visibility (Eqs. 13–14).

### Key equations
- **Frontier viewpoint sampling (p.3).**
  - Eq. (1): cylindrical samples around the frontier-cluster centroid, p = FC̄_i + r_k[cos θ_j, sin θ_j, 0]ᵀ + [0, 0, z_m]ᵀ.
  - Eq. (2): Σ_{l∈L} 𝕀(l ∈ FOV(v)) > V_thr (feature visibility).
  - Eq. (3): Σ_{c∈FC_i} 𝕀(c ∈ FOV(v)) > FC_thr (frontier visibility).
- **Feature-based viewpoints, Eq. (4), p.3:** p ∈ R_i ∩ R_j, where R_i is a range annulus [d_min, d_max] around cluster centroid o_i, with o_i and o_j both in FOV(v).
- **Graph edges and selection (p.4).**
  - Edges are v₀→v^fea, v^fea↔v^fea and v^fea→v^fro.
  - Eq. (5): an edge exists only if |Ω_a ∩ Ω_b| > C_thr, i.e. enough co-visible features.
  - Eq. (6): transition cost C = max(‖p_a − p_b‖/v_max, ‖ψ_a − ψ_b‖/ψ̇_max).
  - Eq. (7): t_sg = t_sl + t_lg.
  - Eq. (8): G_nav = w·ℓ, where w is the cosine between the goal direction and the viewpoint yaw, and ℓ is the number of visible frontier cells.
  - **Eq. (9):** R = ω_p(−t_sg) + ω_n·G_nav. The planner takes the argmax over the finite viewpoint set.
- **Position optimisation, Eq. (10), p.4:** min f_s + w_t t_p + λ_c f_c + λ_d(f_v + f_a) over B-spline control points and Δt.
- **Yaw graph search (p.5).**
  - Eq. (11): sampling interval t_ψ = t_p(⌊t_p/t_f⌋ + 1)⁻¹. This bounds the sample interval by the VIO tolerance.
  - Each node N(p_i, a_i, ψ_ij) gets a 6-DoF pose from differential flatness.
  - Nodes are rejected if |𝕃_ij| ≤ V_thr (visibility constraint).
  - Edges are rejected if |𝕃 ∩ 𝕃'| ≤ C_thr (co-visibility constraint), or if the yaw-rate constraint is violated.
  - Eq. (12): c_s = Σ(Δψ)²[1 + exp(−μ_e c_e)], where c_e = μ_f·|newly visible frontier cells| + μ_g·𝕀(goal ∈ FOV).
- **Yaw optimisation (p.5).**
  - Eq. (13): diffeomorphic map of the bounded ψ to unconstrained q.
  - Eq. (14): J_ψ = Σ[J_s + λ_d J_d − λ_e J_e]. J_e = Σ_c v(c, s(ψ)) uses APACE's [9] differentiable visibility, which is "higher the closer c is to FOV centre".
  - **Line-of-sight filter (p.5):** a frontier cell is excluded from J_e if the "Raycast from c_j to N_i is blocked by obstacles", or if it was already observed.
- **Localizable corridor, Eq. (15), p.6:** ψ_i ∈ (l_i, u_i), and every yaw in that interval keeps every critical feature f ∈ 𝕃_i^cri in the FOV.
  - Critical features are chosen by co-visibility score μ(f, N_{i−1}, N_i) = v(f, T_{i−1})·v(f, T_i). The top C_thr are taken from each adjacent pair.
  - The bounds come from an incremental bidirectional yaw search.

### Online inputs vs GT
- **Online:** RGB-D, IMU, VIO (ORB-SLAM3 RGB-D in the real world), and incrementally built obstacle, feature and frontier maps.
- **GT:** OptiTrack is used for evaluation only, to quantify drift (p.7).

### Timing / compute / hardware
- Algorithm runtime is the maximum time of a single trajectory generation, from Table I (p.7):
  - Ours: 45.21, 23.12 and 62.33 ms in scenes a, b and c;
  - baselines: 54–773 ms.
- Simulation parameters (p.6): v_max = 1.5 m/s, a_max = 1.5 m/s², t_f = 0.35 s. 100 runs per scene.
- Real platform (p.7): Intel NUC i7-1260P with 32 GB RAM (CPU only), forward-looking RealSense D455, ORB-SLAM3 in RGB-D mode.

### Experiments (numbers with page and context)
Table I (p.7) gives success / navigation time / runtime:

| Scene | Ours | Zhang | APACE-R | APACE-E |
|---|---|---|---|---|
| (a) | **100/100**, 42.41 s, 45.21 ms | 23/100, 38.26 s, 102.60 ms | 0/100, —, 408.33 ms | 42/100, 62.87 s, 391.04 ms |
| (b) | **100/100**, 15.76 s, 23.12 ms | 0/100, —, 54.17 ms | 49/100, 18.69 s, 301.78 ms | 100/100, 30.80 s, 201.44 ms |
| (c) long corridor (sharp turns, dead-ends, stairs) | **92/100**, 110.95 s, 62.33 ms | 0/100, —, 81.41 ms | 0/100, —, 772.66 ms | 0/100, —, 472.90 ms |

- A run counts as **localization lost** if visible features stay below threshold for longer than t_f (Fig. 6 caption, p.7).
- **Ablation** (Fig. 7, p.7): without the viewpoint transition graph, the UAV is trapped in a dead-end because the co-visibility constraint cannot be met. With the graph it escapes.
- **Real world** (p.8), target 12 m away:
  - scenario 1 (dark, with a dead-end): 27.66 m path, goal error 0.79 m, VIO RMSE 0.53 m;
  - scenario 2 (only some obstacles emit light): 19.21 m, 0.60 m, 0.37 m.

### Limitations
- **Stated:** there is no limitations section. The conclusion (p.8) claims efficiency and feasibility only.
- **Observed:**
  1. Localizability is a hard count threshold (V_thr, C_thr) rather than a probabilistic or covariance measure.
  2. The world is static. There is no moving target.
  3. Speeds are low (v_max 1.5 m/s).
  4. The 8 failures in scene (c) are not analysed in the text.
  5. [UNCERTAIN] It is not explicit whether FOV(·) in Eqs. 2–5 includes occlusion raycasting for features. Raycast blocking is stated explicitly only for frontier cells (p.5).

### MOTAR relevance / conflicts
- **Relevance:**
  1. It is a finite, graph-based candidate generator with time-feasible edge costs (Eq. 6) and a scalar reward (Eq. 9). That is structurally close to BOIA's candidate scoring.
  2. It combines an explicit line-of-sight raycast with FOV checks (p.5).
  3. A two-frame co-visibility product (μ = v·v, p.6) works as a persistence-of-observation measure.
  4. The "lost if below threshold for more than t_f" definition (p.5, Fig. 6 on p.7) is a clean operational loss event.
  5. The sensors (forward RGB-D) match MOTAR.
- **Conflicts:**
  1. Visibility targets are static map features and frontier cells, not a small moving target.
  2. There is no detector-reliability model.
  3. Edges use only kinematic bounds (v_max, ψ̇_max), not full dynamics.
  4. There is no learned policy to arbitrate over.

### Novelty overlap vs BOIA
| BOIA element | Overlap | Evidence |
|---|---|---|
| (a) p_det forecast (FOV+LOS+reliability) | **partly** | FOV counts (Eqs. 2–5, p.3–4) plus raycast LOS filtering (p.5) plus a centre-weighted differentiable visibility v(c,s) (p.5). All are deterministic and apply to static features or frontiers. There is no belief over a moving target and no detector reliability. |
| (b) finite dyn-feasible candidates scored by progress/loss/cov | **partly** | A finite viewpoint set scored by R = ω_p(−t_sg) + ω_n G_nav (Eq. 9, p.4). Feasibility is via v_max/ψ̇_max (Eq. 6). Loss risk enters only as hard thresholds (Eq. 5) and t_f (p.5). There is no capture or covariance term. |
| (c) arbitration over frozen RL policy | **no** | Not addressed. |
| (d) safe fallback, terminal-first | **no** | No explicit fallback. The graph lets the planner back out of dead-ends (Fig. 7, p.7), which is the closest analogue. |

---

## How to model detector reliability p_det under partial occlusion in MOTAR

Target form: p_det = f_FOV(α, β, Z) · f_LOS(visible fraction ν, occlusion pattern) · f_range/angle(d, φ, apparent pixel size) · f_motion(ω). Calibrate it against the simulated detector's actual outputs. Each idea below is tied to a page.

1. **Smooth FOV gate** (PATimeOptimal, Eqs. 30–33, p.7; λ_v = 10 in Table V, p.13).
   - Use v_α·v_β·v_Z with tanh edges instead of a hard FOV test. The sharpness λ_v can be fitted to the detector's edge-of-image miss rate.
   - It is differentiable and avoids threshold chattering, which helps BOIA's hysteresis [MOTAR inference].
   - Note the v_Z sign issue flagged above.
2. **Range and angle through apparent size** (PATimeOptimal, Eqs. 42–44, p.8; Appendix C, p.21).
   - Apparent size is l = f·L·cos φ / d. Use the *expected pixel extent* l, not raw range, as the regressor: p_det is likely a sigmoid in l·ν, i.e. the visible pixel count [MOTAR inference].
   - Measurement noise grows with d²/(L cos φ) (Eq. 44). This can set the measurement covariance that BOIA's covariance-growth term uses.
3. **Close-range partial-visibility correction** (PATimeOptimal, Eq. 45, p.8).
   - The paper adds σ_u/(d⁴+ε) because close landmarks are only partly visible. For MOTAR, the analogue is a separate p_det drop when the target is partially out of frame at short range, distinct from bar occlusion.
4. **Motion-blur factor** (PATimeOptimal, p.17; FPV images in Fig. 21, p.18).
   - Aggressive rotations cause "severe motion blur", and gate-corner detection noise causes large biases. Add a body-rate or angular-velocity term to p_det if the renderer or detector shows such an effect [MOTAR inference on applicability; the rendered sim may have no blur].
5. **LOS by raycast** (PAUnknown, p.5).
   - The paper excludes a cell if a raycast to the viewpoint is blocked. For MOTAR's bars, use *multiple* rays across the target's extent (centre plus edges) to estimate the visible fraction ν. With a single centre ray, "centre-only" occlusion would be misread as full occlusion, and a target with only its centre visible would be misread as fully visible [MOTAR inference built on p.5].
6. **Centre-weighted visibility** (PAUnknown, v(c,s) "higher the closer c is to the FOV centre", p.5, citing APACE).
   - Weight p_det by image-plane position, since detectors and crops often degrade near borders.
7. **Structured occlusion masks to calibrate p_det(ν)** (ORTrack, Eq. 1, p.4; Table 4, p.7; Fig. 5 at 0/10/30/70% masking, p.8).
   - ORTrack generates random-ratio, centre-concentrated masks. For MOTAR, replace the bell-shaped Cox intensity with *bar-shaped strip masks* at controlled ν and positions (centre-only, edge, full).
   - Sweep these offline through the actual simulated detector to get an empirical p_det(ν, pattern) table [MOTAR inference adapting p.4].
   - Table 4 shows that the masking operator design changes robustness substantially (81.6→88.6 Prec), so the occlusion *pattern*, not only the ratio, matters.
8. **Attribute-binned evaluation** (ORTrack, Fig. 3 partial-occlusion subset, p.7).
   - Report detector precision per occlusion bin. The roughly 7-point gap between the with-ORR and without-ORR models shows how much a detector's training regime can shift p_det under occlusion. So **calibrate against the exact detector MOTAR uses**; do not reuse literature curves.
9. **Raw confidence as calibration input** (ORTrack score map p ∈ [0,1], p.5).
   - The score is focal-loss trained and not calibrated. If MOTAR's simulated detector emits scores, fit a calibration map (e.g. isotonic) on held-out rendered episodes [MOTAR inference; no calibration in the paper].
10. **Loss event and persistence** (PAUnknown: lost if visible count stays below threshold for more than t_f = 0.35 s, p.5–6 and Fig. 6 on p.7; co-visibility product μ = v(f, T_{i−1})·v(f, T_i), p.6).
    - Define BOIA's "loss" as the probability of no detection over a window longer than t_f. Approximate it as ∏(1 − p_det,k) over the window (independence assumption, MOTAR inference). Compare with consecutive-frame products as in μ.
11. **Uncertainty growth when unseen** (PATimeOptimal: +0.1 m per sample when uncertainty is unreliable, p.13; information fusion Eq. 41, p.8).
    - Use additive covariance inflation during non-detection, and information-additive updates (Σ⁻¹ sum) on detection. Weight by p_det for the expected-covariance forecast [MOTAR inference].

---

## Cross-paper table

| | ORTrack (2504.09228v1) | PA Time-Optimal (2603.04305v1) | PA Unknown/Feature-limited (2503.15273v2) |
|---|---|---|---|
| Read status | FULL_READ, pp.1–11 (**different version from user's copy**) | FULL_READ, pp.1–21 | FULL_READ, pp.1–8 |
| Domain | UAV visual single-object tracking (ViT) | Offline time-optimal plus perception-aware trajectory optimisation, plus an MPC tracker | Online perception-aware navigation in unknown, feature-poor environments |
| Occlusion handling | Cox-process template masking with an MSE invariance loss (Eqs. 1–2, p.4–5) | None (FOV only, Eq. 11, p.4) | Raycast LOS for frontier cells (p.5) |
| Visibility / reliability model | Implicit (robust features); uncalibrated score map | Smooth FOV product (Eqs. 30–33); range noise ∝ d² (Eq. 44); close-range correction (Eq. 45) | Count thresholds (Eqs. 2, 3, 5); differentiable centre-weighted v (p.5) |
| Uncertainty metric | None | −log det FIM (Eqs. 40, 48) | Feature-count co-visibility |
| Dynamics | n/a | Full rotor-level model, thrust and thrust-rate limits, drag (Eqs. 1–3, p.3–4) | Flat outputs; B-spline plus MINCO yaw; v/a limits |
| Candidates | n/a | Single NLP solution | Finite viewpoint graph plus Dijkstra; reward Eq. 9 |
| Online? / speed | 206–292 GPU FPS, 55–65 CPU FPS (TitanX/i9) | No: planning takes 68–459 s (Table III); MPCTC via acados | Yes: 23–62 ms per replan (NUC CPU) |
| Real-world | Benchmarks only | Orin NX, IMX219 fisheye, mocap state; 9.8 m/s | NUC, D455, ORB-SLAM3; 1.5 m/s sim params |
| Code | github.com/wuyou3474/ORTrack (p.1) | not given | github.com/Robotics-STAR-Lab/LA-Planner (p.1) |
| BOIA (a) p_det forecast | no | partly (FOV + range, no LOS) | partly (FOV + LOS, no reliability) |
| BOIA (b) finite candidates | no | no | partly (time + nav gain, no capture/cov) |
| BOIA (c) RL arbitration | no | no (RL only discussed, p.17–18) | no |
| BOIA (d) fallback / terminal | no | no | no (dead-end escape only, Fig. 7) |
| Main use for MOTAR | Occlusion-mask generator for calibrating p_det(ν); evidence that the detector's training regime changes how much occlusion hurts | FOV factor, range/angle noise and FIM-based covariance-growth terms | LOS raycast, co-visibility persistence and the t_f loss definition; finite candidate scoring template |
