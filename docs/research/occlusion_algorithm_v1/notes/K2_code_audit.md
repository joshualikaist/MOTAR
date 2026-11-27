# K2 — Source-code audit of UAV tracking/planning repositories

Audit date: 2026-10-19. Auditor: read-only static inspection. Nothing from any repository was executed, built, installed or sourced.

**Method.** Each repo was cloned with `env -u LD_LIBRARY_PATH git clone --depth 1 --filter=blob:limit=1m --no-checkout <url> <dir>` into `scratchpad/occ_research/code_clones/<name>/`. It was then inspected only with `rev-parse`, `log -1`, `ls-tree -r --long` and `git show HEAD:<path>` on text files.

Deviations from that method, all read-only:
- **Lazy blob fetches.** `ls-tree --long` lazily fetched the blobs over 1 MB to get their sizes (weights and GIFs). Those blobs were never opened.
- **One ref listing.** `git ls-remote --heads --tags` was run once on NavRL to list its branches.
- **GitHub API.** The GitHub REST API listing of `Zhefan-Xu` repos and that user's profile README were fetched to look for a NavRL++ release.
- **NavRL local copy.** The local copy was read directly from disk.

Line numbers below are from `git show HEAD:<path> | cat -n` (or from the local file for NavRL). The highest audit level claimed anywhere is FUNCTIONS_READ.

MOTAR target (for portability verdicts): Python 3.8, PyTorch, Isaac Gym / Aerial Gym, no ROS. A frozen RL policy outputs body (vx, vy, vz, yaw-rate) every 0.1 s. Sensors are forward RGB-D plus LiDAR. The scene has dense vertical bars.

---

## 1. OA-VAT (Instance-level Visual Active Tracking with Occlusion-Aware Planning, CVPR 2026)

| Item | Value |
|---|---|
| URL | https://github.com/SHWplus/OA-VAT (default branch `main`) |
| HEAD | `cea1f6b72a9ae297bd4c3d0e8243aeee81d4d28c`, 2026-04-24 17:04:04 +0800 |
| Size | 767 files, 69.4 MB (15 blobs >1 MB: GIFs, a PNG, Pose2ID keypoint txt) |
| License | **No top-level LICENSE file.** `README.md:300-302` says "Distributed under the MIT License. See `LICENSE`", but that file is missing. Vendored components carry their own licences: `ORTrack/LICENSE` MIT (You Wu 2025); `Pose2ID/LICENSE` MIT; `gym-unrealcv/LICENSE` Apache-2.0; `gym-unrealcv/diffusion_policy-main/LICENSE` MIT (Columbia); `dinov3-main/LICENSE.md` is the **DINOv3 License** (Meta's custom licence, not OSI). Implication: treat the OA-VAT glue code as "MIT claimed, file absent" (ask the authors or reimplement from scratch). The DINOv3 weights/code carry Meta's terms. Ideas are free to reimplement. |
| Languages / build | Python 3.10, conda `environment.yml` (env name `follow`). No ROS. |
| Dependencies | torch 2.2.2 / torchvision 0.17.2 (cu121), ultralytics 8.3.223 (YOLOE-11L-seg), diffusers 0.25.0, transformers, gym 0.10.9, gym-unrealcv 2.0.2 (UnrealCV / Unreal Engine binaries), djitellopy 2.5.0 (DJI Tello real drone), pykalman, hydra/dill (Diffusion Policy). Separate envs are recommended for IPG/Pose2ID (torch 2.0.1, xformers) and for Diffusion Policy (`robodiff`). |

### Method → code map (functions read)
- **Instance prototype / feature matching**
  - `extract_target_feature_dinov3.py:114 main` builds the reference feature set offline from user images. For persons, it can synthesise extra poses with Pose2ID/IPG (`:64 _run_ipg_inference`).
  - `dinov3_feature_extractor.py:17 DINOv3FeatureExtractor` resizes to 518 (`:65`). The default model is `dinov3_vith16plus` (`config.py:49`); the README examples use `dinov3_vitb16`.
  - `tracking/reference_store.py:7-81 ReferenceFeatureStore`: the prototype is the L2-normalised mean of the reference features (`:14-32`). Its online EMA update is `fused = alpha*old + (1-alpha)*cur` (`:65-81`); the CLI default is `--enhance_alpha 0.8` (`main.py:118`).
  - `tracking/detection.py:14-71 match_candidates_with_reference` takes YOLOE candidate masks/boxes, computes DINOv3 features per masked crop, and takes the max cosine similarity against the reference set, then the argmax over candidates.
  - `:74-157 find_instance_by_features` accepts the best match if `sim > match_similarity_thresh` (README uses 0.4, CLI default 0.75).
  - Minor bug at `detection.py:53`: `min(tx2, y2)` should be `ty2`.
- **Short-term tracker**
  - ORTrack (DeiT-tiny ViT, template 128 / search 256; `config.py:70-77`) is driven from `tracking/ortrack_core.py:55 track_object_with_ortrack`.
  - The track counts as lost when `confidence < ortrack_lost_thresh` (`:164`, default 0.6) for `ortrack_max_lost_frames` (default 3) frames (`:90-91, 253`).
- **Confidence-aware Kalman filter.** The filter runs in **image space only** (bbox x, y, a, h plus velocities; no 3-D state).
  - `kalman_filter.py:22-400` holds a DeepSORT-style constant-velocity KF.
  - It also has `project_with_confidence` (`:161-203`) and `update_with_confidence` (`:278-301`), with logistic noise scale `max_noise/(1+exp(k(c-center)))+min_noise`. **These two methods are not called** by the tracking loop: grep finds no call site in `main.py`, `ortrack_core.py`, `workers.py` or `eval_*.py`.
  - What actually runs (`ortrack_core.py:180-233`) has two branches:
    - **Low-confidence frames:** a pykalman filter (`tracking/kalman_utils.py:5-28 init_pkf_bbox`) is updated with `R = I*conf_to_noise(conf, k=20, center=0.5)` (`kalman_utils.py:31-45`). The smoothed bbox is written back and the internal KF is synced.
    - **Confident frames:** a standard KF update.
  - After the loss threshold, the KF is re-initiated with a velocity from a linear fit over the last 8 centres (`:261-265`, `kalman_utils.py:48-65`). It is then predicted for up to `kf_max_predict_frames` (README 8; default 15) (`:279-301`).
- **Occlusion handling** (`ortrack_core.py:253-444`):
  1. In predict mode the drone is **stopped** (`send_velocity(0,0,0,0)` once, `:302-308`). The KF only predicts the bbox.
  2. A parallel `LostReacquireWorker` (`tracking/workers.py:210`) runs YOLOE + DINOv3 re-ID on frames every `reacq_interval` (`:311-355`).
  3. After `max_predict_frames`, the "DP rescue" runs if `--dp_enable` is set (`:397-444`).
- **Diffusion planner**
  - `dp/dp_rescue.py:7-288 dp_reposition_and_try_reacquire` and `dp/dp_inference.py`.
  - **Inputs** (`dp_inference.py:139-175` and `diffusion_policy/config/task/unreal_bbox_image_abs.yaml:3-11`): RGB `[1, To=1, 3, 240, 360]` in [0, 1], plus the KF-predicted bbox normalised xywh `[1, 1, 4]`.
  - **Model** (`config/train_unreal_bbox_image_workspace.yaml:12-58`): `DiffusionUnetImagePolicy`, ResNet-18 encoder (no pretraining), 1-D U-Net with `down_dims [512,1024,2048]`, DDPM with 100 train steps and squaredcos schedule, `horizon 8`, `n_obs_steps 1`, `n_action_steps 8`.
  - **Inference steps** are overridden to **16** (`dp_inference.py:84`).
  - **Output:** 8 × 3 relative positions (dx, dy, dz). They are differenced into per-step deltas (`dp_rescue.py:112-137`).
  - **Velocity mapping** (`dp_inference.py:178-206 dp_action_to_tello_vel`): `yaw_rate = atan2(dy,dx)/dt` (×30 if dy<0), `forward = |dxy|/dt`, `vz = dz/dt`, clipped to Tello RC units.
  - **Execution:** receding horizon, executing the first `dp_action_exec_steps` (`:268-282`), with re-ID after each step (`:250-266`).
- **DP training labels.** These are A* expert trajectories on a privileged occupancy grid. They were collected in UnrealCV from deliberately occluded initial pairs (`gym-unrealcv/tools/collect.py:119 astar`, `:246/277 find_occluded_initial_pair_*`, `:661 compute_vis_ratio_by_mask`) and converted to zarr by `tools/convert_unreal_to_replay.py:131-246`.
- **Controller.** `control/drone_control.py:6-117 compute_drone_action_while_tracking` uses PID on the image-centre x-error → yaw-rate and on the bbox-area error → forward speed. **vz is disabled** (`:58`). `control/pid.py:4`.
- **Sim bridge.** `DRONE/unreal_sim_controller.py:34 send_velocity` maps to the UnrealCV action `[vx, vy, vz, vyaw]` (integers clipped to ±max_speed).

### Sensors, target source, outputs, rates
- **Sensor:** monocular RGB only (README runs at 320×240 or 160×120; DP takes 240×360). No depth and no LiDAR.
- **Target state at inference:** fully detector-based, from the YOLOE text-prompt detector, DINOv3 re-ID and the ORTrack tracker. The ground-truth bbox is used only in DP data collection.
- **Outputs:** velocity / yaw-rate RC commands (vx=0, vy=forward, vz, yaw_rate). `ctrl_dt` defaults to 0.1 s. `dp_dt` defaults to 1 s (`main.py:164`).

### Weights / datasets
**No weights are in the repo.**
- **ORTrack:** links to Google Drive / Baidu (`ORTrack/README.md:4, 76`).
- **YOLOE-11L-seg:** Ultralytics hub (AGPL-3.0 for Ultralytics code and weights; this matters for redistribution).
- **DINOv3:** Meta gated download under the DINOv3 License.
- **DP checkpoint:** not released. The default path points to the authors' local machine (`main.py:161`).
- **Datasets:** none bundled.

### Compute
- **Stated latency:** none found in the files read. The code only logs timings (`detection.py:115, 134`).
- **Estimate for an RTX 3070 8 GB beside Isaac Gym:**
  - ORTrack DeiT-tiny: tiny.
  - YOLOE-11L-seg: ~1–1.5 GB at inference.
  - DINOv3 ViT-H+: ~840 M params, ~1.7 GB fp16 plus activations at 518 px.
  - DP U-Net at 2048 width: ~250 M params, 16 denoising steps per plan.
- **Feasibility:** with ViT-H+ the stack is **marginal or infeasible** next to Isaac Gym rendering. With ViT-B/16 plus YOLOE-S it is plausible at low rate (a few Hz). It is not batched across envs, so it is unusable inside vectorised RL rollouts.

### Portability to MOTAR
| Component | Verdict | Reason |
|---|---|---|
| Instance prototype + EMA re-ID gate | REUSE_IDEA_ONLY | Needs YOLOE + DINOv3 (heavy, licence-encumbered). In MOTAR the target identity is known in sim. The useful idea is "mean-of-references prototype + EMA update + cosine gate" for a re-acquisition check. |
| Confidence-aware KF (logistic R(conf)) | REIMPLEMENT_IN_PYTHON | About 20 lines. The mapping `R = R0 * max_noise/(1+exp(k(c-c0)))` can be applied to a **3-D** target KF fed by MOTAR's detector confidence or visibility ratio. The OA-VAT KF itself is image-space bbox only, and its confidence methods are unused. |
| Occlusion policy (stop → KF predict → parallel re-ID → DP rescue) | REUSE_IDEA_ONLY | It stops the drone during prediction, which conflicts with MOTAR keeping pursuit. The step-limited receding-horizon "rescue" loop is a reusable pattern. |
| Diffusion planner | NOT_APPLICABLE (as code) / REUSE_IDEA_ONLY | RGB+bbox → 8-step xyz trained on UnrealCV A* demos. No weights; Tello-specific action mapping; needs a robodiff env (Python 3.9+, hydra). An occlusion-breaking expert from a privileged occupancy grid plus line-of-sight is an idea MOTAR can replicate in Isaac Gym. |
| PID image-servo controller | REUSE_IDEA_ONLY | Trivial; MOTAR has its RL policy. |

**Audit level:** FUNCTIONS_READ. Functions read:
- `kalman_filter.KalmanFilter` (all methods)
- `tracking/kalman_utils` (all)
- `tracking/reference_store` (all)
- `tracking/detection` (both functions)
- `tracking/ortrack_core.track_object_with_ortrack`: lines 55-310 and 395-445 read; the rest grepped
- `dp/dp_inference` (all)
- `dp/dp_rescue` (all)
- `control/drone_control` (all)
- `config.py:1-90`
- `DRONE/unreal_sim_controller.send_velocity`
- DP task/workspace YAMLs
- `convert_unreal_to_replay.py`: grepped
- `collect.py`: function list only
- `workers.py`, `ORTrack/lib`, `dinov3-main`, `Pose2ID`: structure only

---

## 2. YOPO (You Only Plan Once, RA-L 2024)

> **Primary-source caveat:** the YOPO paper PDF is NOT available to this project. For YOPO, **the code below is the only primary source**. Any statement about "the paper" is the README's wording, not verified against the publication.
>
> The clone's default branch is **`YOPO-Simple`**. The README calls it a 2025-06 Python/PyTorch refactor with a new CUDA simulator. It says the paper-consistent version is the **`main`** branch, and a MINCO variant is on **`YOPO-MINCO`** (`README.md:46-52`). Neither of those branches was audited. Numbers below describe `YOPO-Simple` and may differ from the published method.

| Item | Value |
|---|---|
| URL | https://github.com/TJU-Aerial-Robotics/YOPO (branch `YOPO-Simple`) |
| HEAD | `bd8a7f91953cfef9f1aa9a48d7ff0ab7bceee132`, 2026-08-14 18:44:07 +0800 |
| Size | 514 files, 97.8 MB (11 blobs >1 MB: `YOPO/saved/YOPO_1/epoch50.pth` 45.3 MB, GIFs, `uav.dae`, `tree.ply`, STEP) |
| License | `LICENSE` MIT (© 2024 TJU-Aerial-Robotics). Permissive; direct port allowed with the copyright notice. The controller is derived from HKUST Fast-Planner (`Controller/src/readme.md:35`); its sub-package licences were not checked. |
| Languages / build | Python 3.8 (conda), C++/CUDA for the simulator, ROS1 catkin (Ubuntu 20.04, Noetic implied), Jetson Orin/Xavier NX. |
| Dependencies | `YOPO/requirements.txt`: torch 2.4.1+cu118, torchvision 0.19.1, numpy 1.22.3, scipy 1.10.1, open3d 0.19.0, opencv 4.11, ruamel-yaml, tensorboard, rospkg/catkin_pkg. Optional TensorRT via torch2trt. Simulator: custom CUDA ray-cast depth/LiDAR renderer (`Simulator/src/src/sensor_simulator.cu`) with random forest/Perlin/maze maps. Controller: SO(3) attitude controller with disturbance observer (`Controller/src/so3_control/src/NetworkControl.cpp`, `HGDO.h`). |

### Method → code map (functions read)
- **Primitive / anchor generation:** `YOPO/policy/primitive.py:30-84 LatticePrimitive.__init__`
  - It builds `horizon_num × vertical_num = 5 × 3 = 15` anchors (`config/traj_opt.yaml:21-28`).
  - Anchor yaw angles are `alpha_j = -Δ(H-1)/2 + jΔ` with `Δ = 90°/5 = 18°`, giving {-36, -18, 0, 18, 36}°. Pitch uses `Δ = 60°/3 = 20°`, giving {-20, 0, 20}°.
  - The radius is `radio_range = 5 m` and the endpoint is `[cosβ cosα r, cosβ sinα r, sinβ r]` (`:63-77`). Each anchor carries the rotation `R_bp = R_ZYX(α, -β, 0)`.
  - Per-anchor search cones are ±15° yaw and ±15° pitch (`:83-84`, from `*_anchor_fov: 30`).
  - Speed scaling (`:8-11`): `vel_max = ratio*6`, `acc_max = ratio²*6`, `segment_time = (2*5/6)/ratio` s, with `ratio = velocity/vel_max_train`.
  - Grid ↔ lattice ordering is reversed (`:104-105`).
- **Network:** `policy/yopo_network.py:14-55 YopoNetwork`
  - **Image backbone** (`policy/models/backbone.py:8-16`): ResNet-18 with a **1-channel** conv1 and no avgpool. A 1×1 `output_layer` gives 64 channels. Input is **depth `[B, 1, 96, 160]`**, normalised by `min(d, 20 m)/20` and inpainted (`test_yopo_ros.py:145-165`). The camera renders 160×90 with fx=80 (90° HFOV) (`Simulator/src/config/config.yaml:58-66`) and is resized to 160×96. Stride 32 gives a **3 × 5 feature grid**, one cell per anchor.
  - **State vector:** 9 = [v_xyz, a_xyz, goal_dir_xyz] in the body frame. Velocity and acceleration are normalised by vel_max/acc_max, and the goal by `max(|g|, 10 m)` (`state_transform.py:130-137`). The state is rotated into each anchor's frame and broadcast as `[B, 9, 3, 5]` (`state_transform.py:87-110`), then concatenated channel-wise with the 64-ch image feature.
  - **Head** (`models/head.py:4-16`): 1×1 conv 73→256→256→10.
- **Output decoding:** `yopo_network.py:38-39` gives `endstate = tanh(out[:, :9])` and `score = softplus(out[:, 9])`. `state_transform.py:20-59 pred_to_endstate` decodes:
  - Δyaw = o0 · 15°, Δpitch = o1 · 15°, radius = (o2+1) · 5 m, giving the endpoint at angle anchor+Δ and range 0–10 m.
  - End velocity = o3:6 · vel_max and end acceleration = o6:9 · acc_max, both in the anchor frame and rotated by R_bp to the body frame.
  - The CPU version is `:61-84`.
- **Trajectory:** a single quintic per axis from (p0, v0, a0) to the predicted (p1, v1, a1) over `segment_time` (`policy/poly_solver.py:4-37 Poly5Solver`).
- **Cost / loss terms** (`loss/loss_function.py:10-111 YOPOLoss`). Training back-propagates trajectory cost directly; there are no expert labels.
  - **Smoothness:** ∫jerk² via QP matrices `R_J`, plus ∫acc² `R_A` (`:35-73`, `loss/smoothness_loss.py:11-31`). Weights are `ws/v⁵` and `wa/v³` (`:75-89`; ws=10, wa=1).
  - **Safety:** an ESDF is built offline from the dataset point clouds with a 0.2 m voxel (`loss/safety_loss.py:191-239`). It is sampled at 30 time points via `grid_sample` (`:38-103`). The cost is `exp(-(d - d0)/r)` with d0=1.2 m and r=0.6 m (`:105-106`), time-averaged; weight wc=1.
  - **Guidance:** the projection of the trajectory onto the goal direction, `smoothL1(|g|, along) + 0.5·perp` (`loss/guidance_loss.py:52-78`); weight wg=0.15.
  - **Score supervision** (`policy/yopo_trainer.py:142-176`): `score_label = detached total cost` per anchor, with `smooth_l1(score, label)`. The score is therefore a **predicted cost; lower is better.**
- **Selection / controller**
  - `test_yopo_ros.py:243-256 process_output` picks `argmin(score)` over the 15 anchors.
  - `:145-206 callback_depth` plans at depth rate (simulator 33 Hz, `config.yaml:7`).
  - `:208-241 control_pub` samples the polynomial every `ctrl_dt = 0.02 s` (50 Hz). It publishes `quadrotor_msgs/PositionCommand` (p, v, a, yaw, yaw_dot) to the SO(3) attitude controller with disturbance observer, which produces attitude and thrust.
  - Yaw comes from `poly_solver.py:65-89 calculate_yaw` (blend of velocity and goal direction, rate-limited to 0.5π rad/s).
  - `plan_from_reference` chooses between odometry and the reference state as the plan origin (`:397`).

### Sensors, target source, outputs, rates
- **Inputs:** forward depth `1×96×160` (90°×60° FOV) plus odometry (v, a) plus a goal point (RViz click). The LiDAR is simulated (16×360) but is not used by the network.
- **Target state:** none; this is navigation to a goal.
- **Outputs:** quintic endpoint (p, v, a) → PositionCommand at 50 Hz → attitude/thrust.

### Weights / datasets
- **Weights:** `YOPO/saved/YOPO_1/epoch50.pth`, 45.3 MB, in-repo (trained at 6 m/s). The README says more are on GitHub Releases (not verified). Covered by the repo MIT licence.
- **Datasets:** none shipped. The dataset is generated by `Simulator/src/src/dataset_generator.cpp` (10 maps × 10 000 images by default, `config.yaml:79-80`).

### Compute
- **Stated:** training takes <1 h for 100k samples / 50 epochs on an RTX 3080. Inference takes 1 ms (ResNet-14) to 5 ms (ResNet-18) on Orin NX with TensorRT (`README.md:167, 182`).
- **Estimate for an RTX 3070:** ResNet-18 on a 96×160 depth image is under 1 ms and under 200 MB. It can be batched over envs and is **easily feasible** next to Isaac Gym.
- **Training cost:** the ESDF maps live on the GPU, and their memory scales with map volume at 0.2 m voxels.

### Portability to MOTAR
| Component | Verdict | Reason |
|---|---|---|
| Lattice primitive generator (`LatticePrimitive`) | **DIRECT_PORT_POSSIBLE** | Pure torch/scipy geometry with no ROS. It only needs `cfg` values replaced. MIT licence. |
| Quintic boundary-value solver (`Poly5Solver`, `qp_generation`) | DIRECT_PORT_POSSIBLE | numpy/torch closed form. |
| ESDF safety cost (`SafetyLoss.cost_function`, grid_sample query) | REIMPLEMENT_IN_PYTHON | The cost form ports directly, but the ESDF here is built offline from PLY files. MOTAR would build it from Isaac Gym asset geometry (bars are analytic cylinders/boxes, so the distance is closed-form). |
| Network + guidance-learning training | REIMPLEMENT_IN_PYTHON / REUSE_IDEA_ONLY | Trainable in Python 3.8 + torch, but needs a depth dataset from MOTAR scenes. The pretrained weights are tied to a 90°×60° FOV, 0–20 m depth, 6 m/s and forest/Perlin maps, and are unlikely to transfer to dense vertical bars without retraining. |
| ROS node / SO(3) controller / CUDA simulator | NOT_APPLICABLE | MOTAR has no ROS, and Isaac Gym supplies dynamics and rendering. |

**Can the YOPO-style primitive candidate set be used as a finite candidate generator feeding velocity / yaw-rate commands without its network?** Yes, with adaptation. The steps:
1. **Build the candidates.** Instantiate the 15 (or a denser, e.g. 9×3) anchor endpoints `p_k = R(α_k, β_k)·[r, 0, 0]` from `primitive.py:63-77`, optionally with a few radii. Set each end velocity to `v_des·dir_k` and the end acceleration to 0 (no network offsets).
2. **Solve each candidate.** Use the quintic from the current (v, a) with `poly_solver.Poly5Solver` and T = 2r/v.
3. **Score each candidate.** Use MOTAR's own cost: an exp-barrier ESDF term like `safety_loss.py:105` on 30 samples, a target line-of-sight / visibility term, and a tracking-distance term like YOPO-Tracker's `tracking_loss.py:12-27`.
4. **Pick the winner.** Take the argmin, which mirrors `process_output`.
5. **Convert to a command.** Read the polynomial's velocity at t ≈ 0.1 s (or average it over [0, 0.1]) and rotate it to the body frame to get (vx, vy, vz). Compute yaw-rate as in `calculate_yaw` or YOPO-Tracker's target-pointing yaw.

Caveats:
- The default 18° yaw spacing and ±36° span are coarse for dense bars.
- Anchors are tied to the camera FOV, so there are no lateral or backward candidates.
- A 5 m radius at 6 m/s is mismatched to MOTAR speeds and bar spacing; use r ≈ 1.5–3 m.
- The candidates are open-loop, so per-candidate evaluation has to use a map or point cloud (Isaac Gym geometry or a LiDAR-built ESDF), not the network.
- In MOTAR the frozen RL policy already issues (vx, vy, vz, yaw-rate). This generator fits as a **shield, fallback, or action-candidate set** that the policy output is projected onto or compared against.

**Audit level:** FUNCTIONS_READ. Functions read:
- `config/traj_opt.yaml`, `config/config.py`
- `primitive.LatticeParam/LatticePrimitive` (all)
- `state_transform.StateTransform` (all methods)
- `yopo_network.YopoNetwork` (all)
- `models/backbone.ResNet18/ResNet14`, `models/head.YopoHead`
- `poly_solver` (all)
- `loss/loss_function.YOPOLoss` (all)
- `loss/safety_loss.SafetyLoss` (all)
- `loss/guidance_loss` (all)
- `loss/smoothness_loss` (all)
- `yopo_trainer.forward_and_compute_loss` (lines 142-176); the rest grepped
- `test_yopo_ros.py`: grepped and key callbacks read
- `Simulator/src/config/config.yaml:1-80`
- `NetworkControl.cpp`: grepped only

---

## 3. YOPO-Tracker (YOPOv2-Tracker, IEEE T-RO 2026 per README)

| Item | Value |
|---|---|
| URL | https://github.com/TJU-Aerial-Robotics/YOPO-Tracker (branch `main`) |
| HEAD | `d313f8610195dc986caceff7a3616f63d7e6f31f`, 2026-10-15 23:55:23 +0800 |
| Size | 477 files, 90.8 MB (9 blobs >1 MB: `YOPO/saved/YOPO_1/epoch50.pth` 45.3 MB, GIFs, `uav.dae`, STEP, `hardware_setup.pdf`) |
| License | `LICENSE` MIT (© 2024 TJU-Aerial-Robotics). The vendored `UnityConnector/src/ROS-TCP-Endpoint/LICENSE` is Apache-2.0 (Unity). The Unity simulator binary is a GitHub Release zip; its licence was not checked. |
| Languages / build | Python **3.12** (must match ROS 2 **Jazzy**, `YOPO/requirements.txt:1-2`), C++ ROS 2 colcon controller, Ubuntu 24.04, Jetson Orin NX. |
| Dependencies | Unpinned torch / torchvision / opencv / scipy / open3d / ruamel-yaml / tensorboard (`requirements.txt`). Prebuilt Unity "YOPO-Tracker-Simulator" (Release `v0.1.0-alpha`) via ROS-TCP-Endpoint. Optional COCO train2017 for detection pretraining (`traj_opt.yaml:29`). torchvision ImageNet ResNet-18 init (`backbone.py:9-20`). |

### Method → code map (functions read)
- **Primitives:** same lattice as YOPO (5×3, 90°/60° FOV, ±15° cones, r=5 m; `traj_opt.yaml:37-44`, `policy/primitive.py:8-11`).
- **Network:** `policy/yopo_network.py:13-66`
  - **Input:** **RGB-D `[B, 4, 192, 320]`** (RGB/255 plus depth/max_dis), built in `test_yopo_ros.py:205-244 parse_rgbd`.
  - **Backbone** (`models/backbone.py:24-37`): ResNet-18 with a 4-ch conv1. RGB filters come from ImageNet and the depth filter is their mean. An extra `AvgPool2d(2)` after layer1 (`models/resnet.py:188, 242`) makes the total stride 64, giving a **3×5 grid** (`config.py:19`, downsample = 320/5 = 64).
  - **State vector:** 6 = [v_xyz, a_xyz] in the body frame (**no goal**), broadcast per anchor (`state_transform.py:137-152`).
  - **Head:** 1×1 conv 70→256→256→15.
- **Output decoding** (`yopo_network.py:38-51`):
  - Endstate: tanh(9), decoded as in YOPO.
  - **Target (v, u):** `sigmoid·2-0.5` (a YOLOv5-style cell offset).
  - **Target depth:** `softplus`.
  - Trajectory cost: `softplus`.
  - **Objectness:** `sigmoid`.
  - Trajectory time: `sigmoid·max_traj_time` (`state_transform.py:93-99`).
- **Image → 3-D target** (`state_transform.py:101-123 pred_to_target(_cpu)`): `(v, u) = (offset + grid)·64`, then pinhole back-projection with `f = W/(2·tan(45°))` and principal point at the image centre: `x = d, y = -d(u-cx)/f, z = -d(v-cy)/f` (body frame, NWU). Detection loss: `loss/detection_loss.py:14-32`.
- **Target filter ("EKF"):** `policy/target_ekf.py:5-132 EKF`. It is in fact a **linear** constant-velocity KF.
  - State [p, v] (6), position measurement, process noise via acceleration Q = diag(16, 16, 8), R = 0.02 I.
  - Euclidean innovation gate `distance_threshold + σ_p + 6·blind_time`, capped at 3·thr (`:72-81`).
  - Dead reckoning up to `max_blind_time = 0.5 s` (`:59-65, 118-121`).
  - Re-initialisation inside a reachable set `v_max(10 m/s)·lost_time + gate_max` (`:111-117`).
  - m-of-n (4/6) confirmation (`:122-126`).
  - It runs at the image rate (`ekf_dt = 1/30`, `test_yopo_ros.py:59, 593`) with `ekf_thresh = 2.0`.
- **Cost / loss terms** (`loss/loss_function.py:11-146`, weights in `traj_opt.yaml:11-18`):
  - **Smooth** (ws=30/v⁵) and **Accel** (wa=0.3/v³), in normalised time.
  - **Safety:** ESDF `exp(-(d-1.2)/0.6)`, wc=1.8 (`loss/safety_loss.py:32-49`).
  - **Target clearance:** 10× the barrier while within `track_dist` (`safety_loss.py:43-47`).
  - **Track** (wt=0.12): smooth-L1 of the endpoint xy to the point `track_dist = 3 m` short of the target (`loss/tracking_loss.py:12-27`).
  - **Height** (wh=0.5): match target z-0.3, or hold altitude if no target (`:30-41`).
  - **Guide** (wg=0.01): `|10 m - traj length|` when no target (`loss/guidance_loss.py:11-22`).
  - **Time** (wtime=0.4·T) and **Feasible** (wf=30).
  - **Per-anchor masking** (`policy/yopo_trainer.py:29-49, 246-326`): Track applies only on "positive" anchors (cells containing the target, `get_detection_mask` `:354-379`). Guide applies only on negatives.
  - **Cost head label:** Smooth + Accel + Safety (`:305-306`).
  - **Objectness:** BCE vs the positive mask (`:313-314`).
  - **Modality dropout / COCO mixing:** `policy/modality.py`.
- **Training target source:** **ground truth**. Unity `labels.csv` world target positions are transformed to the body frame (`policy/yopo_dataset.py:94-105`). NaN means no target.
- **Selection** (`test_yopo_ros.py:456-484 NMS`):
  - Anchors with `obj_score > conf_thresh = 0.9` are gated by the EKF, then the lowest predicted cost among them is chosen.
  - If none pass, it falls back to the lowest-cost no-target anchor.
- **Occlusion / loss handling** (`test_yopo_ros.py:335-386, 246-271, 283-318`):
  - The EKF coasts the target for ≤0.5 s.
  - The last tracking trajectory keeps executing until 30% of its duration, then no-target planning (guidance: fly forward) takes over (`:363-376`).
  - Within 3 m of the last known target with no confirmed track, it plans a braking trajectory, then **spins yaw at 3.14 rad/s** to search (`:246-271, 387-394`).
  - There is a hover mode when caught up and the target is still.
  - **There is no line-of-sight or visibility cost in training.** The network never receives the EKF state, so target memory during occlusion is purely the heuristic above.
- **Controller:** the polynomial is sampled at 50 Hz (`ctrl_dt = 0.02`, `:58, 398-426`) → PositionCommand (p, v, a, yaw, yaw_dot) → ROS 2 SO(3) attitude controller with disturbance observers (`Controller/src/so3_control/include/so3_control/ADO.h`, `HGDO.h`) → attitude/thrust. Yaw points at the EKF target via `calculate_yaw(..., max_yaw_rate=1)` (`:387-396`).

### Sensors, target source, outputs, rates
- **Inputs:** forward RGB-D 4×192×320 (90° HFOV) and odometry. **No LiDAR.**
- **Target at inference:** the **network's own detection head**, from the RGB-D image only, filtered by the KF. No external detector and no ground truth.
- **Outputs:** quintic endpoint plus duration → PositionCommand at 50 Hz → attitude/thrust. Planning runs at the image rate (30 Hz).

### Weights / datasets
- **Weights:** `YOPO/saved/YOPO_1/epoch50.pth`, 45.3 MB, in-repo (trained at 6 m/s, tracks persons at 0–6 m/s). MIT.
- **Datasets:** none shipped. They are collected in the Unity sim (`README.md:136-156`). COCO is optional (COCO licence: CC-BY 4.0 annotations, Flickr image terms).

### Compute
- **Stated:** TensorRT recommended (legacy note, `README.md:193-218`). No latency figure is given for this repo.
- **Estimate for an RTX 3070:** ResNet-18 at 4×192×320 runs in ~1–2 ms and under 300 MB. **Feasible** and batchable beside Isaac Gym.

### Portability to MOTAR
| Component | Verdict | Reason |
|---|---|---|
| Lattice + quintic + per-anchor cost evaluation | DIRECT_PORT_POSSIBLE | Same as YOPO. `TrackingLoss` and `HeightLoss` are directly reusable cost terms for a candidate scorer. |
| Target KF with growing gate, blind-time limit, reachable-set re-init, m-of-n confirm (`target_ekf.py`) | **DIRECT_PORT_POSSIBLE** | Pure numpy and MIT. A good fit for MOTAR's 3-D target estimate under bar occlusion; replace dt with 0.1 s and retune Q/R. |
| Image→3-D back-projection | DIRECT_PORT_POSSIBLE | A pinhole formula. MOTAR needs its own intrinsics (Aerial Gym camera config). |
| Search behaviour (brake + yaw spin near last-known position) | REIMPLEMENT_IN_PYTHON | A simple state machine. It is a candidate fallback, but it ignores visibility; MOTAR should prefer a view-restoring candidate. |
| Network (joint detect + plan) and its weights | REUSE_IDEA_ONLY | Trained on Unity person scenes with a 192×320 RGB-D camera; no MOTAR transfer. Retraining needs a Unity-like labelled dataset plus ESDF. Its training has no occlusion-aware term. |
| ROS 2 node / controller / Unity bridge | NOT_APPLICABLE | No ROS in MOTAR. |

**Audit level:** FUNCTIONS_READ. Functions read:
- `config/traj_opt.yaml`, `config/config.py`
- `yopo_network` (all)
- `models/backbone` (all)
- `models/resnet._forward_impl/__init__` layers (grepped)
- `state_transform.pred_to_target(_cpu)`, `endstate_to_pred`, `prepare_input`, `pred_to_traj_time`
- `target_ekf.EKF` (all)
- `modality` (all)
- `loss/tracking_loss` (all), `loss/detection_loss` (all), `loss/guidance_loss` (all), `loss/loss_function` (all)
- `loss/safety_loss.forward`: grepped
- `yopo_trainer.TRAJ_LOSS_MASKS` and `forward_and_compute_loss` (lines 246-326)
- `yopo_dataset`: grepped
- `test_yopo_ros.update_search/target_uv/update_hover/callback_images/desired_yaw/NMS`, plus the settings block
- `primitive.py:1-40`: grepped, identical scaling to YOPO

---

## 4. NavRL (RA-L 2025) and the NavRL++ check

| Item | Value |
|---|---|
| URL | https://github.com/Zhefan-Xu/NavRL (branch `main`); local copy `/home/fair/workspaces/aerial_gym_ws/reference/NavRL` |
| HEAD | `3725bcc2e7c1be4ecf1455d922299ae85042603a`, 2025-07-03 16:47:18 -0700. **The local copy is the same commit**: it is a git repo with origin = Zhefan-Xu/NavRL, `git status --porcelain` is empty, `git diff --stat HEAD` is empty, and it has 2 933 tracked files, the same as the fresh clone. |
| Size | 2 933 files, 305.4 MB (45 blobs >1 MB: Gazebo meshes/textures, PCD maps, detector weights). Most files are vendored `isaac-training/third_party` (torchrl, warp, OmniDrones-style code). |
| Remote branches | `main` (3725bcc), `ros2-foxy` (bd8759bd). No tags. |
| NavRL++ | **No NavRL++ code release found.** Searched: (a) the NavRL repo tree for `plus`, `++`, `navrl2`, `v2` paths (only unrelated vendored `smacv2.py` / `conv2d*`); (b) its remote branches; (c) the GitHub API listing of all public Zhefan-Xu repos sorted by update (NavRL, Intent-MPC, LV-DOT, onboard_detector, CERLAB-UAV-Autonomy, etc.; no NavRL successor); (d) the profile README (no mention). |
| License | `LICENSE` MIT (© 2025 Zhefan Xu). Vendored third-party code has its own licences (not enumerated). Isaac Sim itself is under the NVIDIA Omniverse licence. |
| Languages / build | Python 3.10. Training needs **Isaac Sim 2023.1.0-hotfix.1** (exact version, `README.md:69`) plus torchrl/tensordict (vendored). Deployment has ROS1 Noetic and ROS2 Humble packages (`ros1/`, `ros2/`) with C++ safe-action, map_manager, onboard_detector (YOLO + LiDAR dynamic-obstacle detector), Gazebo `uav_simulator`. `quick-demos/` are plain-Python demos. |

### Method → code map (local files, functions read)
- **Observation** (`isaac-training/training/scripts/env.py:294-308 _set_specs`, `:431-530 _compute_state_and_obs`):
  - **LiDAR:** `[1, 36, 4]` as `range - hit_distance` (range 4 m, hres 10° → 36 beams, 4 vertical beams over -10°..20°) (`:437-442`; `cfg/drone.yaml:4-8`).
  - **State (8):** unit goal direction expressed in the goal-aligned frame (3), horizontal goal distance (1), vertical goal offset (1), velocity in the goal frame (3) (`:459-477`).
  - **Dynamic obstacles:** `[1, 5, 10]` for the 5 nearest by 2-D distance. Each has relative position (normalised), distances, velocity in the goal frame, and width/height category (`:480-530`).
  - **Direction:** `[1, 3]`, used for the goal-frame transform.
- **Network** (`scripts/ppo.py:13-92 PPO`):
  - LiDAR CNN: Conv(4,[5,3]) → Conv(16, stride[2,1]) → Conv(16, stride[2,2]) → Linear 128 + LayerNorm (`:21-27`).
  - Dynamic-obstacle MLP 50→128→64 (`:30-33`).
  - Concatenation [128 + 8 + 64] → MLP 256-256 (`:36-41`).
  - Beta actor (`:45-51`) and linear critic (`:54-56`).
  - PPO with GAE(0.99, 0.95) and Huber critic loss (`:60-61`).
- **Action space:** 3-D **velocity** in the goal frame, Beta-sampled in [0, 1], then `2·a·L - L` with L = 2 m/s (`ppo.py:89`, `cfg/ppo.yaml:9`), rotated to the world frame (`:90`). There is **no yaw command**: yaw is handled by a separate PID in deployment (`ros2/navigation_runner/scripts/navigation.py:83`).
- **Reward** (`env.py:549-577`): `vel·goal_dir + 1 + log(LiDAR clearance) + log(dynamic-obstacle clearance) - 0.1·smooth - 8·height`; collision when the LiDAR max exceeds range - 0.3 (`:570-571`). Sim dt = 0.016 (`cfg/sim.yaml:2`).
- **Safety shield** (deployment only): `ros2/navigation_runner/include/navigation_runner/safeAction.cpp:53 getORCAPlane`, `:139-182 getSafeAction`. It is an ORCA-style velocity-obstacle projection with time horizon 1.0 s and safety distance 0.05 m (`:28-50`). It covers dynamic obstacles (inflated as stacked spheres) and LiDAR points, solved by a 3-D LP (`solver.h`). It is called from `navigation.py:225-262 get_safe_action` with the RL velocity as the preferred velocity.
- **Target/goal state source:** the goal is a known waypoint, not a tracked target. Dynamic obstacles at deployment come from `onboard_detector` (YOLO + LiDAR); in training they come from simulator ground truth.

### Sensors, outputs, rates
- **Inputs:** 3-D LiDAR (36×4 ray-cast), odometry, goal, and detected dynamic obstacles.
- **Output:** world-frame velocity command. The control timer is about 0.05 s in deployment (PID dt = 0.05 at `navigation.py:83`); this was not exhaustively verified.

### Weights / datasets
- `navrl_checkpoint.pt` (723 925 B) in `quick-demos/ckpts/`, `ros1/` and `ros2/navigation_runner/scripts/ckpts/`.
- Detector weights `weight_AP05:0.253207_280-epoch.pth` (1.16 MB) and `shufflenetv2.pth` (0.70 MB).
- All covered by the repo MIT licence; the provenance of the YOLO weights was not checked.

### Compute
- **Stated:** none read.
- **Policy:** about 0.2 M params, negligible.
- **Training:** needs Isaac Sim 2023.1, which is not Isaac Gym. Running it alongside MOTAR's Isaac Gym on 8 GB is infeasible, but **inference is trivial** on CPU or GPU.

### Portability to MOTAR
| Component | Verdict | Reason |
|---|---|---|
| Goal-frame observation (state 8, LiDAR range-minus-hit) | REIMPLEMENT_IN_PYTHON | Straightforward tensors. Aerial Gym already provides LiDAR ray-casting. |
| Network / PPO | REIMPLEMENT_IN_PYTHON | Depends on torchrl/tensordict TensorDict modules; the architecture itself is small. MOTAR already has a frozen policy. |
| Pretrained checkpoint | NOT_APPLICABLE | Observation layout and sim differ (Isaac Sim, 36×4 LiDAR); outputs a goal-frame velocity with no yaw. |
| ORCA safety shield | REIMPLEMENT_IN_PYTHON | The C++ ROS service is not portable as-is. The ORCA half-plane construction plus a small LP/QP (or a sampled candidate check) can be rewritten in numpy or torch to filter MOTAR's (vx, vy, vz) every 0.1 s. Bars are static cylinders, so this is a static-obstacle VO. |

**Audit level:** FUNCTIONS_READ. Functions read:
- `ppo.PPO.__init__/__call__`, `train` (head)
- `env._set_specs`, `_compute_state_and_obs` (lines 431-530, grepped and read)
- `env._compute_reward_and_done` and the reward lines 549-577
- `cfg/*.yaml`
- `safeAction.cpp`: grepped (`initParam`, `getORCAPlane` signature, `getSafeAction` loop)
- `navigation.get_safe_action`
- Remote clone: structure only (identical commit)

---

## 5. EGO-Planner (ZJU-FAST-Lab) — structure-level

| Item | Value |
|---|---|
| URL | https://github.com/ZJU-FAST-Lab/ego-planner (branch `master`) |
| HEAD | `bfda51284c8c1b476043255a8145ef925a3778a5`, 2025-03-08 17:23:46 +0800 |
| Size | 566 files, 24.1 MB (5 blobs >1 MB: GIFs, a hummingbird mesh) |
| License | **GPL-3.0** (`LICENSE`). Copyleft: any port or derivative that is distributed must be GPL-3.0. Reimplement from the paper's equations rather than translating the code if MOTAR is to stay non-GPL. |
| Languages / build | C++14, ROS1 catkin (Ubuntu 16.04/18.04/20.04, `README.md:11, 54`), Eigen, Armadillo (simulator), PCL. Bundled `uav_simulator` (so3 quadrotor sim, mockamap, local_sensing; GPU depth render optional). The README points to ego-planner-swarm for ROS2. No CUDA requirement for the planner. |

### Method → code map
- **Front end / initialisation:** `src/planner/bspline_opt/src/bspline_optimizer.cpp:37 initControlPoints` finds colliding segments of the initial B-spline against the inflated occupancy grid (`grid_map_->getInflateOccupancy`, `:60`). It runs A* through them (`src/planner/path_searching/src/dyn_a_star.cpp`) to create `{base_point p, direction v}` pairs per control point.
- **Collision term:** `:342-381 calcDistanceCostRebound`.
  - For each control point Q_i and pair j: `d = (Q_i - p_ij)·v_ij` and `err = sf - d` with `sf = dist0`.
  - The cost is 0 if err < 0, `err³` if err < sf, else `3sf·err² - 3sf²·err + sf³`, with the matching gradient.
  - No ESDF is used. The "rebound" check (`check_collision_and_rebound`, called at `:351-354`) adds new {p, v} pairs during optimisation.
- **B-spline optimisation:**
  - `:1044-1065 combineCostRebound` minimises `λ1·smoothness + λ2·distance + λ3·feasibility` (`:1060`) with L-BFGS (`lbfgs.hpp`; call at `:903`). It doubles λ2 on failure (`:951`).
  - Refinement (`:856`, `:1067-1089`) swaps the distance term for fitness `λ4` (`calcFitnessCost` `:383`), doubling λ4 on failure (`:1032`).
  - Smoothness and feasibility: `:413`, `:454`.
  - Uniform B-spline: `uniform_bspline.cpp`.
  - Parameters are read in `setParam` (`:8-15`).
- **Orchestration:** `plan_manage/src/ego_replan_fsm.cpp` (FSM), `planner_manager.cpp`, `traj_server.cpp` (samples the B-spline to PositionCommand). Map: `plan_env/src/grid_map.cpp`, `raycast.cpp`.
- **Sensors / outputs:** depth image or point cloud → inflated occupancy grid. Output is a B-spline sampled into a PositionCommand (p, v, a, yaw) for the so3 controller.
- **Target state:** none; this is goal navigation.
- **Weights / datasets:** none (optimisation-based).
- **Compute:** CPU only, roughly 1 ms per replan per the paper's claims (not verified in code). Trivial on any GPU budget.

### Portability to MOTAR
| Component | Verdict | Reason |
|---|---|---|
| Rebound collision penalty `{p, v}` formulation | REUSE_IDEA_ONLY (GPL) | The idea and equation can be reimplemented in torch for candidate refinement. Do not translate the GPL code. |
| B-spline + L-BFGS optimiser | REUSE_IDEA_ONLY | C++/ROS. A Python reimplementation is possible, but per-step optimisation at 0.1 s across many envs is costly. A finite candidate set (YOPO-style) is simpler for MOTAR. |
| FSM / ROS stack | NOT_APPLICABLE | ROS1. |

**Audit level:** STRUCTURE_MAPPED. Beyond structure, these were read: function signatures in `bspline_optimizer.cpp`, `calcDistanceCostRebound` (`:342-381`) and `initControlPoints` (`:37-60`) in full, and grepped lines for the λ weights and L-BFGS calls.

---

## Summary table

| Repo | Commit (date) | License | Stack | Key function map | Target-state source (inference) | Weights | Portability verdict |
|---|---|---|---|---|---|---|---|
| OA-VAT | cea1f6b (2026-04-24) | MIT claimed in README but **no LICENSE file**. Vendored: ORTrack MIT, gym-unrealcv Apache-2.0, Diffusion Policy MIT, **DINOv3 License**; YOLOE AGPL. | Py3.10, torch 2.2.2/cu121, ultralytics, diffusers, UnrealCV, Tello. No ROS. | `tracking/reference_store.py:7` prototype+EMA; `tracking/detection.py:14,74` DINOv3 cosine match; `kalman_filter.py:161,278` conf-aware KF (unused), `kalman_utils.py:31` + `ortrack_core.py:180-233` (used); `ortrack_core.py:253-444` occlusion FSM; `dp/dp_inference.py:139,178`, `dp/dp_rescue.py:7` diffusion RHC (RGB 3×240×360 + bbox → 8×xyz, 16 steps) | Detector + re-ID + tracker (RGB only, image-space KF) | None released (ORTrack via Drive; DP checkpoint private) | KF confidence mapping: REIMPLEMENT. Rest: REUSE_IDEA_ONLY. DP code: NOT_APPLICABLE. |
| YOPO (`YOPO-Simple` branch; **paper PDF unavailable, code is the only source**) | bd8a7f9 (2026-08-14) | MIT | Py3.8, torch 2.4.1/cu118, ROS1 catkin, custom CUDA sim | `policy/primitive.py:30-84` 5×3 lattice; `yopo_network.py:30-40`, `backbone.py:8` ResNet-18 on depth 1×96×160 + state 9; `state_transform.py:20-59` decode; `loss/*` smooth/ESDF-exp/guidance; `yopo_trainer.py:174` score = cost; `test_yopo_ros.py:243` argmin, `:208` 50 Hz PositionCommand | N/A (goal navigation) | `epoch50.pth` 45.3 MB in repo (MIT) | Lattice + quintic: **DIRECT_PORT_POSSIBLE**; usable as a network-free candidate generator → (vx, vy, vz, yaw-rate) with a custom cost. Net: REIMPLEMENT/retrain. ROS: N/A. |
| YOPO-Tracker | d313f86 (2026-10-15) | MIT (+ Apache-2.0 ROS-TCP-Endpoint) | Py3.12, ROS2 Jazzy, unpinned torch, Unity sim (release zip) | `yopo_network.py:29-51` RGB-D 4×192×320 + state 6 → 15 ch (endstate, target vud, cost, obj, time); `state_transform.py:101-123` pinhole back-projection; `target_ekf.py:5-132` gated CV-KF; `loss/tracking_loss.py:12-41`; `yopo_trainer.py:40-49` anchor masks; `test_yopo_ros.py:456` NMS, `:246` brake+yaw-spin search | Network's own detection head + KF (training uses GT `labels.csv`) | `epoch50.pth` 45.3 MB in repo (MIT) | KF + back-projection + lattice/costs: **DIRECT_PORT_POSSIBLE**. Search FSM: REIMPLEMENT. Net: REUSE_IDEA_ONLY. ROS/Unity: N/A. |
| NavRL | 3725bcc (2025-07-03); local copy identical. **No NavRL++ release found.** | MIT | Py3.10, Isaac Sim 2023.1.0-hf1, torchrl; ROS1 Noetic / ROS2 Humble deploy | `env.py:431-530` obs (LiDAR 1×36×4, state 8, dyn-obs 5×10); `ppo.py:21-56` CNN+MLP Beta-PPO; `ppo.py:89` velocity ±2 m/s; `env.py:549-577` reward; `safeAction.cpp:53,139` ORCA shield | Known goal; dynamic obstacles from onboard_detector (GT in training) | `navrl_checkpoint.pt` 0.72 MB + detector weights in repo | Obs/net/ORCA shield: REIMPLEMENT_IN_PYTHON. Checkpoint: NOT_APPLICABLE. |
| EGO-Planner | bfda512 (2025-03-08) | **GPL-3.0** | C++14, ROS1 catkin, Eigen, Armadillo | `bspline_optimizer.cpp:37` {p, v} init; `:342-381` rebound collision cost; `:1044-1065` λ1·smooth+λ2·dist+λ3·feas with L-BFGS (`:903`) | N/A (goal navigation) | None (optimisation) | REUSE_IDEA_ONLY (GPL; do not translate code). ROS: N/A. |

Audit levels reached:
- OA-VAT: FUNCTIONS_READ
- YOPO: FUNCTIONS_READ
- YOPO-Tracker: FUNCTIONS_READ
- NavRL: FUNCTIONS_READ
- EGO-Planner: STRUCTURE_MAPPED (plus the collision cost read)

No repository is claimed CODE_VERIFIED: nothing was run.
