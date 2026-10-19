# P6 — Four additional original papers (outside the original 25)

Reader: Codex, 2026-10-19. Status describes reading, never reproduction or code verification.
Physical PDF pages are numbered from 1. All four supplied PDFs were read from beginning to end,
including experiments, conclusions, references and any appendix (none of these four files contains
a separate appendix). The originals were opened read-only. Local text extraction and selected page
renders were used; no model, repository clone, simulator or GPU was run.

The comparison target is BOIA §07: (a) belief-conditioned future detection forecast; (b) finite
short-horizon dynamics rollouts including Frozen Policy F; (c) calibrated-gain arbitration over that
frozen policy, hysteresis/minimum hold and immediate safety override; (d) stopping/fallback and
terminal-first handling; (e) mission-level reobservation with collision as competing event and final
capture safeguards. A `Partial` match does not establish the complete conjunction or novelty.

MOTAR remains forward RGB-D + 4×72 LiDAR at 12 m, Frozen Policy F/898-D at 0.1 s, CV tracker,
70–205 vertical bars and RTX 3070 8 GB. Publication results below are contextual source facts, not
matched MOTAR comparisons. Paper repository URLs are references only: **code audit not done**.

## 1. EGO-Swarm — A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments

- **Authors / printed version:** Xin Zhou, Jiangchao Zhu, Hongyu Zhou, Chao Xu, Fei Gao;
  arXiv:2011.04183v3 [cs.RO], 5 May 2021 (p.1). This supplied preprint does not print a proceedings venue.
- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/EGO-Swarm A Fully Autonomous and Decentralized Quadrotor.pdf`
- **SHA-256:** `3fe008fa358dfff54c0d5ea632a7f6f3baa74339f7f8fe766383fc9343743ff5`; **pages:** 7; **read:** physical pp.1–7; **status:** `FULL_READ`.

### Problem, assumptions and pipeline

The problem is decentralized, asynchronous goal navigation for multiple quadrotors in an unknown
cluttered environment, not pursuit of a designated target (pp.1–2). EGO-Swarm extends EGO-Planner:
B-spline local optimization → implicit topologically distinct alternatives → reciprocal trajectory
collision penalties → lowest-cost trajectory execution (pp.2–3). Other agents broadcast their
planned trajectories; local depth mapping stores static obstacles (pp.3–5). Onboard camera/IMU/VIO
and wide-angle depth observations support local mapping and relative drift correction (pp.4–5).
The sensor's numeric FOV is **NOT_SPECIFIED in this supplied PDF**; the 7.5 m horizon in p.5 is a
distance, not a time duration. Goal points, rather than a target-motion model, define the mission.

- p.2 Eq.(1): `min_Q J_EGO = Σ_r λ_r J_r`, with smoothness, collision, dynamic-feasibility and
  terminal-progress terms. Eq.(3) is a soft barrier, not a certified hard safety constraint.
- p.3 Eq.(4): `d_ij = (Q_i − p_ij)·v_ij`; obstacle-surface anchor `p` and outward direction `v`.
  Opposite `{p_new, −v}` constructions produce alternative local minima; choose the lowest-cost
  optimized trajectory. Definition 1's UVD relation uses collision-free line segments between
  equal trajectory parameters; it does not promise exhaustive topology search (p.3).
- p.3 Eq.(5), p.4 Eq.(6): integrated swarm-collision soft penalty, with anisotropic clearance for
  downwash, augments `J_EGO`. Received time-stamped trajectories are evaluated at shared time.
- p.4 Eqs.(8)–(9): projection of a predicted agent trust region into the depth image and centroid
  of matching 3-D depth points estimate relative localization drift. These equations rely on
  small tracking errors and adequate inter-agent visibility; the paper expressly says the
  region can contain unrelated objects, so centroid correctness is not guaranteed (p.4).

### Inputs, decisions, termination and cost

Online inputs are own odometry, depth/gray images, occupancy map and other agents' communicated
trajectories, including their predicted positions; these are not evaluator-only labels in this
multiagent system (pp.4–5). No training stage or privileged training labels are involved. Mapping
removes detected agents' pixels because moving agents are handled separately (p.4 §IV.C); that
separation assumes reliable association and cannot justify blindly carving MOTAR target-like bars.
Replan every second, on a predicted/new collision, or near trajectory end (p.5). Newly received
trajectories also trigger collision checks and replanning. There is no frozen-RL arbitration,
explicit braking-to-known-free safe-hold procedure, target reacquisition deadline or terminal-first
capture contract described here.

Simulation uses i7-9700KF, 0.1 m map resolution, 7.5 m local planning horizon (p.5). In p.5 Fig.9,
front-end alternative generation is 0.017 ms for 2 candidates versus 7.63 ms for 5 Fast-Planner
candidates; these are **front-end-only** times, not full planning or universally comparable costs.
Ten-drone cluttered simulation reports total local planning 0.49–5.00 ms across its obstacle-density
conditions (p.6 Table II), with drone radius 0.2 m and velocity limit 2 m/s. Real indoor/outdoor
three-drone demonstrations use 1.5 m/s (p.6). No learned parameter count/VRAM applies.

### Limitations and MOTAR overlap

**Author-stated:** fewer candidates can lower the chance of reaching the global optimum (p.5);
relative drift correction needs precise trajectory tracking/seeing nearby agents and its centroid
assumption is not guaranteed (p.4). Communication mitigation reduces collision probability rather
than making latency disappear (p.5). **Auditor:** soft penalties and demonstrations do not prove
collision freedom with MOTAR's sparse LiDAR or narrow bar gaps. The paper tracks witnessed agents
for collision avoidance/drift correction; no designated moving-target pursuit/recovery experiment
or capture objective is supplied (entire pp.1–7).

| BOIA part | Match | Original evidence |
|---|---|---|
| (a) | No | No target belief/detection-probability forecast (pp.2–4) |
| (b) | Partial | Finite distinct optimized trajectories and cost selection; no Policy F rollout (p.3) |
| (c) | No | Local planner itself supplies commands, no learned-policy arbitration (pp.3–5) |
| (d) | Partial | Collision-triggered replanning, but no stated stopping-feasible fallback/terminal-first contract (p.5) |
| (e) | No | Goal navigation/collision metrics; no competing-risk target recovery (pp.5–6) |

Code: <https://github.com/ZJU-FAST-Lab/ego-planner-swarm> (pp.2,6); paper:
<https://arxiv.org/abs/2011.04183v3>. Weights: not applicable. Decision: **REFERENCE_ONLY**.

## 2. PPTracker — Tracking UAV Swarms with Prior Prompt

- **Authors / printed venue:** Haolin Qin, Tianhao Li, Tingfa Xu, Jingxuan Xu, Yuqiang Fang,
  Jianan Li; CVPR Workshop Open Access version, printed proceedings pp.6594–6601 (physical pp.1–8).
  A numeric publication year is **NOT_PRINTED on the title page**; do not infer it from file naming.
- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/PPTracker  Tracking UAV Swarms with Prior Prompt.pdf`
- **SHA-256:** `dbdc38f4231e09215e7344c4ac6511a0947741c6601df732b34d7748200f9fdb`; **pages:** 8; **read:** physical pp.1–8; **status:** `FULL_READ`.

### Problem, assumptions, pipeline and equations

This is passive image-based multi-object tracking of UAV swarms by an anti-UAV sensor, not active
flight by a pursuer. The benchmark uses infrared video, initial first-frame target boxes and dense
similar-looking flying objects (p.5 §4.1). There is no map, metric depth/range, camera FOV value,
3-D vehicle dynamics or control horizon. The online initialization contract is material: **first-frame
GT annotations initialize the prior prompt**, while subsequent frames use prior tracking results
(p.3 §3.2 and Fig.1). It would be false to describe all GT use as training-only.

Historical boxes → positional/query tokens; image patches → key/value tokens → attention-guided
prior prompt → residual image/feature refinement → YOLOv11 detector → BoT-SORT with Kalman motion,
camera-motion compensation and appearance/IoU association (pp.3–5). The spelling PPTracker and
PPTrack both appear in the source; retain the title's PPTracker.

- p.4 Eq.(1): `P = Softmax(QKᵀ / sqrt(h)) V`, where `h` is the printed scaling factor. Although
  the prose calls `P` a spatial probability matrix, multiplication by `V` makes it an attention
  output; it is **not a calibrated identity/detection probability** (auditor observation).
- p.4 Eq.(2): `I_re = P + I_base`. The paper does not specify enough tensor-shape details to
  reproduce every prompt-to-image operation from this equation alone (auditor).
- p.5 Eq.(3): `MOTA = 1 − Σ_t(FN_t + FP_t + IDSW_t)/Σ_t GT_t`. This metric combines identity
  switches with false detections/misses; a MOTA increase alone cannot isolate identity continuity.

High-confidence detections use geometry plus cosine-appearance matching; low-confidence ones use
IoU only; sparse optical flow/affine camera-motion compensation corrects motion prediction (p.5).
Numeric confidence/association/lost-track expiry thresholds are **NOT_SPECIFIED** here. There is
no BOIA intervention, action termination or 5-second tracker-clock preservation guarantee.

### Compute, results and limits

YOLOv11-n at 640×640, COCO initialization; dual RTX3090 training, 50 epochs, batch128 (p.6).
Inference FPS, parameter count, peak VRAM and shared-simulator resource cost are **NOT_RECORDED**
in this paper. It reports Table1 MOTA 25.29/60.09 for YOLOv11+ByteTrack/BoT-SORT and 31.60/67.90
with prompts (p.6). **Author-internal inconsistency:** adjacent prose says 2%/3% improvements;
Table1's literal differences are 6.31/7.81 percentage points. Use the table values with this caveat,
not the unsupported prose percentages. Standalone IDSW, IDF1, HOTA, occlusion-duration recovery
curves and uncertainty calibration are not reported; qualitative examples do not fill those gaps.

**Author-stated:** persistent identity challenges under prolonged occlusion/out-of-view remain
(p.7 conclusion). **Auditor:** initializing with GT identities is a different problem from MOTAR's
sensor-only acquisition. Feeding a tracker prediction back as a detection prior can reinforce a
wrong association; it must not turn `tracker.active` or LiDAR-updated age into verified identity.
Use it to motivate appearance/identity diagnostics and stress tests, not as evidence that current
MOTAR has identity-correct tracking. Any adoption needs a separately authorized measured-image
interface and timestamp/expiry tests; no production change is proposed for G4.

| BOIA part | Match | Original evidence |
|---|---|---|
| (a) | Partial, adjacent only | Kalman/appearance history, no future FOV×LOS detection forecast (pp.3–5) |
| (b) | No | Detector/association only, no UAV action rollouts (pp.3–5) |
| (c) | No | No active controller or policy arbitration (entire paper) |
| (d) | No | No UAV fallback/terminal-first logic (entire paper) |
| (e) | Partial, metric caution only | IDSW in MOTA; no collision-competing recovery/capture evaluation (pp.5–7) |

Own code/weights URL: **NOT_PROVIDED in the supplied paper**. Referenced YOLO/BoT-SORT are not
therefore an audited PPTracker release. Decision: **ADD_TO_CORE** (identity-contract caution).

## 3. PRIMER — Perception-Aware Robust Learning-based Multiagent Trajectory Planner

- **Authors / printed version:** Kota Kondo, Claudius T. Tewari, Andrea Tagliabue, Jesus Tordesillas,
  Parker C. Lusk, Mason B. Peterson, Jonathan P. How; arXiv:2406.10060v4 [cs.RO], 13 May 2025
  (p.1). No proceedings venue is printed in this supplied version.
- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/PRIMER Perception-Aware Robust Learning-based.pdf`
- **SHA-256:** `9f60cd5d5874073d66abd1255ecfa4e750e99f3aaee53e7d09a3c07517e2c16e`; **pages:** 7; **read:** physical pp.1–7; **status:** `FULL_READ`.

### Problem, assumptions and pipeline

The task is decentralized asynchronous multiagent goal navigation while keeping other agents and
obstacles inside a limited sensor FOV. It is not a designated target close-approach task. Own state,
predicted obstacle trajectories and received other agents' trajectories enter the planner (p.3 Fig.2,
p.4 Fig.3). Agents detect/track obstacles and share predictions (p.4). A camera's numeric FOV,
measurement-error model, tracking covariance and metric sensing range are not specified here.
A differentially flat quadrotor with position/yaw outputs and derivatives is assumed (p.2); the
trajectory time is optimized in PARM*, not a fixed BOIA-like horizon. Map-building details and
unknown-space safety contracts are not provided in the seven-page paper.

PARM adapts PANTHER to multiple obstacles and uses RMADER deconfliction. PARM* jointly optimizes
position/yaw and total time with less conservatism. PARM* demonstrations → DAgger → PRIMER
LSTM(variable number of obstacle/agent trajectories) + own state + four 1024-unit fully connected
layers → position/yaw trajectory predictions → Check/Delay Check → commit or reject (pp.2–4).

- **p.2 Eq.(1), image verified:** `−α_FOV Σ_i ∫_0^T [inFOV(obstacle_i)]³ dt`. The cube is **inside
  the time integral**. This is a field-of-view reward; no explicit obstacle-ray LOS test, probabilistic
  occlusion belief or calibrated future detection probability appears in this formula. A dynamic
  object being in the FOV is not proof it is geometrically unoccluded (auditor).
- **p.3 Eq.(2):** `L_total = L_pos + α L_yaw`, with α=70. These are imitation errors against
  PARM* trajectories, not calibrated intervention advantages.
- **p.2 Fig.1:** optimization → Check → publish tentative trajectory → repeated Delay Check →
  commit if still conflict-free. A detected conflict discards the new trajectory and retains the
  previously committed one. p.3 §II.B explicitly applies the RMADER deconfliction wrapper to
  PRIMER's learned trajectories. The paper's guarantee is under that trajectory-sharing/checking
  architecture, not arbitrary sensed-world uncertainty.

### Expert/student switching and BOIA(c) novelty

**Confirmed:** there is runtime acceptance/rejection of **learned** trajectories with reuse of a
prior committed trajectory on conflict (pp.2–3). Therefore a broad claim that "a learned planner
with safety checking and fallback" is new is **NOT_NOVEL_ENOUGH**. This is direct prior art and
must be added to core discussion, including BOIA(d).

**Not described:** PARM* is an expert demonstrator for **training**, not an online expert policy
invoked whenever PRIMER loses confidence (p.3 §II.C). The paper does not compare a fixed RL policy
and an alternative by calibrated expected benefit, does not specify gain thresholds/hysteresis or
minimum holds, and does not use target-belief-conditioned detection forecasts. Thus narrow BOIA(c)
is **Partial**, not a confirmed identical method. Absence in this one paper does not establish
novelty against the wider literature. No original-copy finding invalidates §07's formulas, so this
is novelty narrowing, **not by itself G3_REVIEW_REQUIRED**.

### Inputs, cost, decisions and limits

No semantic target GT input is required by the stated network interface; offline optimizer
trajectories and simulated obstacle/agent trajectories supply training demonstrations (pp.3–4).
The paper does not fully specify how accurately online obstacle trajectories must be measured;
GT-free MOTAR compatibility therefore remains **NOT_TESTED**, not assumed. No expiry/identity
clock, terminal-first simulator contract or maximum braking/stopping feasibility is specified.
PARM/PARM* compare one or six trajectories and choose lowest cost; TableIV also labels PRIMER
six trajectories (pp.4–5). The learned output ranking details cannot be inferred beyond this.

p.4 TableIII, single agent/single trefoil obstacle: PARM* 4495.8 ms vs PRIMER 0.8008 ms, mean
cost403 vs431, both100% success and0% obstacle/dynamic failures. This is the context for the
advertised5614× speedup; it is not the full multiagent control cost. p.5 TableIV multiagent results:
PRIMER140 ms (one agent/two obstacles),109 ms (three agents/two obstacles), both100% success;
three-agent translational dynamic-constraint violation0.3%. Report the nonzero violation rather
than promoting all network trajectories to feasible. Simulation hardware i9-9900K3.60GHz and62.6GiB
RAM (p.4); no inference GPU/VRAM/learned parameter count is printed. Limits2m/s,10m/s²,30m/s³,
box-edge0.5m and radius3m position-exchange scenario are source-specific (p.4).

**Author-stated:** optimization can be so slow that constraints become outdated and Check/Delay
Check reject trajectories; larger swarms and hardware flight are future work (pp.4–5).
**Auditor:** FOV-only perception is not LOS recovery; externally supplied predicted obstacle paths
and shared agents differ from MOTAR's sparse online sensing. Simulation results do not verify real
tracking/noise robustness or a collision guarantee in MOTAR. p.4 prose initially says "two agents"
where TableIV and subsequent text use **three**; retain that source inconsistency.

| BOIA part | Match | Original evidence |
|---|---|---|
| (a) | Partial, adjacent only | Future obstacle trajectories/FOV cost, not target belief×LOS×reliability (pp.2–3) |
| (b) | Partial | Multi-trajectory generation/cost selection, no frozen RL baseline candidate (pp.4–5) |
| (c) | Partial | Runtime checking/rejection of learned proposals; not calibrated-gain frozen-RL arbitration (pp.2–3) |
| (d) | Partial, important | Retain prior committed trajectory on conflict; no stopping-feasibility/terminal-first contract (pp.2–3) |
| (e) | No | Goal/collision/FOV metrics; no competing-risk target reacquisition or capture safeguards (pp.4–5) |

Paper: <https://arxiv.org/abs/2406.10060v4>; video <https://youtu.be/OmUxFmTrJOg> (p.1).
Own code/weights URL: **NOT_PROVIDED in this PDF**. Decision: **ADD_TO_CORE**.

## 4. Similarity-Guided Layer-Adaptive Vision Transformer for UAV Tracking (SGLATrack)

- **Authors / printed version:** Chaocan Xue, Bineng Zhong, Qihua Liang, Yaozong Zheng, Ning Li,
  Yuanliang Xue, Shuxiang Song; arXiv:2503.06625v1 [cs.CV], 9 March2025 (p.1). No accepted
  proceedings venue is printed in this supplied version.
- **Original:** `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/논문/Similarity-Guided Layer-Adaptive Vision Transformer for UAV Tracking.pdf`
- **SHA-256:** `ad303ff4fd5c043c1d979efa1bced6a09069869fbdbf0b214c51064df372758b`; **pages:** 11; **read:** physical pp.1–11; **status:** `FULL_READ`.

### Identity and scope

This is **not** *Layer-Guided UAV Tracking: Enhancing Efficiency and Occlusion Robustness*,
arXiv:2602.13636. The supplied PDF prints different title, identifier and authors (p.1).
The primary [arXiv record](https://arxiv.org/abs/2602.13636) names Yang Zhou, Derui Ding, Ran Sun,
Ying Sun and Haohua Zhang and the LGTrack method; its v1 is14February2026. That bibliographic
check is sufficient to distinguish papers, **not a full read of LGTrack**. Do not merge their
occlusion claims, GGCA modules, speeds or implementation details.

### Problem, assumptions, pipeline and equations

SGLATrack accelerates passive single-object visual tracking with lightweight one-stream ViTs.
Template RGB image128×128 and search RGB image256×256 are inputs (p.5); no depth/LiDAR/map, metric
target model, FOV requirement, planning horizon or UAV actuator/controller is specified. Tracking
uses an initialized target template/search crop. It neither solves arbitrary first acquisition nor
provides a closed-loop target-pursuit/recovery controller.

Patch embedding → six initial tiny-ViT layers → selection MLP → retain one subsequent layer,
disable the rest → center-score/size/offset head → box (pp.3–5). The deployed saturated depth
`l*=6` is **fixed**; only the later retained layer index is per-sample adaptive (pp.5,7). It would
be wrong to say the final method recomputes layer-by-layer saturation for every frame.

- p.3 Eqs.(1)–(3): concatenate template/search tokens, transformer composition and cosine feature
  similarity. No probabilistic scene/belief model is formed.
- p.4 Eq.(4): choose the subsequent layer maximizing similarity to saturated features;
  Eq.(5): sigmoid of 3-layerMLP gives selection scores, highest retained; Eqs.(6)–(7) use mean
  absolute deviation from a one-hot maximum-similarity label to supervise that selection.
- p.5 Eq.(8): `L_cls + 2L_iou + 5L_L1 + 0.2L_sim`. The classification score map and layer-selection
  scores are **not calibrated p_det or identity confidence**. No reliability calibration,
  abstention threshold or target-lost/reacquisition state machine is reported.

Train on annotated LaSOT/COCO/TrackingNet/GOT-10k pairs (p.5), not online segmentation GT.
Inference uses template/search images, learned weights and a positional window; p.5 inconsistently
calls the latter Hamming/Hanning. First target template initialization is assumed by the SOT
protocol; the paper does not validate MOTAR sensor-only instance acquisition.

### Compute, experiments and limitations

p.7 Table3 reports DeiT* Params7.98→5.81, FLOPs2.39→1.68 (table does not print their units; do not
infer memory usage from that alone); FPS175.5→224.7 with layer adaptation. Table1p.6 shows UAV123
AUC66.9%, precision84.9%,224.7GPUFPS/74.8CPUFPS; p.7 gives i7-9700KF and the printed "GTX2080Ti"
GPU. This hardware label is quoted, not silently corrected. Training uses four80GB A100s,300epochs
with60k pairs/epoch (p.5); it is not an RTX3070 training feasibility result. JetsonTX2 **4GB**
33FPS deployment is reported without TensorRT (p.8), not simultaneous IsaacGym use.

Metrics are SOT overlap-success AUC and center-error precision on six aerial-video benchmarks;
no controlled long-LOS-occlusion recovery probability, identity-switch rate or collision outcome
is reported (pp.5–8). **Author-discussed:** adaptive saturation can cost speed; fixed depth plus
selection balances speed/accuracy and preserves some performance drops (p.7). **Auditor:** there
is no dedicated limitations section, camera detector stress calibration or concurrent N128GPU
profile. The small network and TX2 result make an offline stress-detector candidate plausible,
but **8GB shared IsaacGym/N128 feasibility remains NOT_TESTED**: batch size, activation/workspace,
CUDA context and renderer contention must be measured under a separately authorized gate.

| BOIA part | Match | Original evidence |
|---|---|---|
| (a) | No | Selection/classification scores are not future belief-conditioned detection probabilities (pp.4–5) |
| (b) | No | Layer selection is not UAV trajectory/action candidate scoring (pp.3–5) |
| (c) | No | No frozen controller arbitration or safety switch (entire paper) |
| (d) | No | No vehicle stopping/fallback/terminal-first contract (entire paper) |
| (e) | No | Offline SOT metrics, no competing collision/reacquisition/capture outcome (pp.5–8) |

Code and models: <https://github.com/GXNU-ZhongLab/SGLATrack> (p.1); paper:
<https://arxiv.org/abs/2503.06625v1>. Code audit not done. Decision: **REFERENCE_ONLY**.

## Four-paper decision table

| Supplied original | Read | BOIA relevance | Decision | Direct qualification needed before adoption |
|---|---|---|---|---|
| EGO-Swarm | FULL_READ pp.1–7 | (b) alternative paths, weak(d) replan | REFERENCE_ONLY | Sparse measured-space safety and sensor association |
| PPTracker | FULL_READ pp.1–8 | Identity/GT-init/metric contract | ADD_TO_CORE | Sensor-only initialization, identity clocks and separate ID metrics |
| PRIMER | FULL_READ pp.1–7 | (c)/(d) learned trajectory rejection/prior fallback | ADD_TO_CORE | Narrow novelty wording; measured obstacle inputs and stopping contract |
| SGLATrack | FULL_READ pp.1–11 | Realistic image-tracker stress reference | REFERENCE_ONLY | Calibration/identity and shared8GB N128memory/latency |

**EGO-Swarm — three-line rationale.**
1. Adds finite alternative trajectories and reciprocal navigation, already adjacent to BOIA(b).
2. Other-agent avoidance/drift tracking is not a designated target recovery/capture objective.
3. Retain as navigation/topology reference; do not claim a direct BOIA intervention competitor.

**PPTracker — three-line rationale.**
1. Explicit first-frame GT initialization exposes an online identity assumption absent in MOTAR.
2. MOTA and qualitative identity examples cannot turn active/recency into verified target identity.
3. Add the identity-contract warning to core design and future evaluation; no tracker port in G4.

**PRIMER — three-line rationale.**
1. Runtime checking/rejecting a learned plan and reusing the previous trajectory is established prior art.
2. PARM* training expertise is not evidence of online expert/learner calibrated-gain arbitration.
3. Narrow BOIA novelty; generic learned-policy-plus-fallback is NOT_NOVEL_ENOUGH, not a formula stop.

**SGLATrack — three-line rationale.**
1. Tiny-ViT layer selection is an efficient passive tracker, distinct from LGTrack arXiv2602.13636.
2. Its scores and demonstrations do not calibrate detection or prove long-occlusion closed-loop recovery.
3. Retain as a future image-stress candidate; shared RTX3070/IsaacGym feasibility remains NOT_TESTED.
