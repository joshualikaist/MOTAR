# K1 — Source-code audit of published UAV tracking / visibility-planning repos

Date: 2026-10-19. Auditor mode: read-only static inspection.

**Method.** Each repo was cloned with
`env -u LD_LIBRARY_PATH git clone --depth 1 --filter=blob:limit=1m --no-checkout <url> <dir>`
into `scratchpad/occ_research/code_clones/<name>/`. I inspected only with `git rev-parse`, `git log -1`,
`git ls-tree -r` and `git show HEAD:<path>`, plus `git rev-list --objects --missing=print` and
`git cat-file --batch-check` to get blob sizes without fetching. Blobs over 1 MB were not fetched;
lazy-fetch attempts from `ls-tree --long` failed with a local libp11/libffi error, so nothing large was
downloaded. No file from any repo was executed, built, installed or sourced. Line numbers come from
`nl -ba` on the `git show` output. Nothing was run, so nothing is marked CODE_VERIFIED.

"Total size" below is the sum of the blobs that were fetched (≤1 MB each), plus a count of the
larger blobs that were not fetched.

**MOTAR reference stack (portability target).** Python 3.8 + PyTorch + Isaac Gym / Aerial Gym, no ROS.
A frozen RL policy commands (vx, vy, vz, yaw-rate) every 0.1 s. Sensors are a forward RGB-D camera and a
LiDAR. The scenes have dense vertical bars. GPU is an RTX 3070 8 GB. Any planner from these repos could
only sit above the policy as a goal/waypoint/yaw-reference generator, a reward/cost term, or a recovery
supervisor. None of them can replace the policy.

Portability classes used: REUSE_IDEA_ONLY / REIMPLEMENT_IN_PYTHON / DIRECT_PORT_POSSIBLE / NOT_APPLICABLE.

---

## 1. Eva-Tracker (ICRA 2026, arXiv 2602.12549)

| Field | Value |
|---|---|
| URL | https://github.com/Yue-0/Eva-Tracker |
| HEAD | `636e26eb8094e94a1da9344cc875a417aa3de76f` |
| Commit date | 2026-03-05 22:11:14 +0800 |
| Default branch | `main` |
| Files / size | 65 files; 1.37 MB fetched + 7 blobs >1 MB not fetched (`bag/benchmark.bag`, 2 GIFs, 2 `.dae` meshes, `src/tracker/models/MonoLoco.onnx`, `src/tracker/models/YOLO11n-Pose.onnx`) |
| Audit level | **FUNCTIONS_READ** — read in full: `src/tracker/src/planner.cpp`, `src/tracker/src/planner/esdf.cpp` + `esdf.hpp`, `src/tracker/src/planner/optim.cpp`, `src/tracker/src/planner/plan.cpp` + `plan.hpp`, `src/tracker/src/planner/map.cpp`, `src/tracker/src/prediction.cpp`, `src/tracker/include/predictor/predictor.hpp`, `src/tracker/src/perception/tracker.cpp`. Read in part: `src/tracker/src/perception.cpp` (1–140), `src/simulator/src/simulator.cpp` (100–190). Mapped by grep only: `bezier.hpp`, `minco.hpp`, `yolo.cpp`, `monoloco.cpp`. |

**License.** `LICENSE` is **GNU AGPL-3.0**, a strong copyleft licence that also covers network use.
Copying any code into the MOTAR research repo would make the derived work AGPL-3.0 if it is
distributed or served. Reimplementing the ideas from the paper in our own code is fine. Treat the
code as reference only and do not paste from it.

**Stack.** C++14 with catkin (ROS1 Noetic, Ubuntu 20.04 per README). Uses Eigen, PCL and OpenMP
(`src/tracker/CMakeLists.txt:7,24`). The real-world perception path additionally needs CUDA, OpenCV
and TensorRT (`CMakeLists.txt:26-27`; `onnx2trt.py`; YOLO11n-Pose + MonoLoco ONNX models). The planner
has no external solver: it uses its own L-BFGS with Lewis–Overton line search. The real platform in
the README is a Livox Mid-360, a RealSense camera, a Jetson Orin NX, PX4/MAVROS and FasterLIO.

**Method map (path:line).**
- *Target prediction*: `src/tracker/include/predictor/predictor.hpp:7` (`Predictor`). It fits a Bézier
  curve of degree 2m to the last m+1 observed positions, minimising a squared-derivative energy under
  interpolation constraints. The KKT matrix is pre-inverted in the constructor (`:16-66`), and
  `predict()` (`:69-84`) is a single mat-vec. The ROS node is `src/tracker/src/prediction.cpp:10`
  (`/target/poses` → `/target/bezier`, `/target/predict`). Bézier evaluation is clamped to
  [−duration, duration] (`bezier.hpp:58`), so extrapolation saturates.
- *FoV-ESDF (visibility model)*: `src/tracker/src/planner/esdf.cpp:31` (`ESDF::ESDF(alpha, beta, distance, r)`).
  It builds, **once, offline and in the body frame**, an exact Euclidean distance transform (Felzenszwalb
  1-D lower envelope, `transform()` `:211`; separable 3-pass `build()` `:109`) of the camera frustum
  (half-angles tan(α/2), tan(β/2), depth D). This is the source of "ESDF-update-free".
  `ESDF::value()` (`:168`) does trilinear interpolation with an analytic gradient. `argmax()` (`:92`)
  gives the depth with the best observation and sets the desired tracking distance
  (`planner.cpp:77-81`).
- *Robot-collision ESDF (RC-ESDF)*: `esdf.cpp:63`, the same machinery applied to an inflated robot box.
- *Occlusion / safety cost*: `src/tracker/src/planner/optim.cpp:345-379`. For every obstacle point of
  the local cloud, transformed into the body frame, it sums `fov->value()` (Jo, occlusion) and
  `robot->value()` (Jp, collision) and applies a quadratic penalty with chain-rule gradients to
  position, yaw and time. This means **any obstacle point inside the frustum is penalised**, not only
  points on the line of sight.
- *Observation / angle costs*: `optim.cpp:310-325`. `fov->max() - fov->value(R·(target - p))` keeps the
  predicted target deep inside the frustum. `optim.cpp:327-343` is the yaw-alignment term 1−cos(ψ − atan2).
- *Dynamic limits*: velocity, acceleration and yaw-rate hinge penalties (`optim.cpp:381-453`). The time
  regulariser is at `:457-461`.
- *Optimiser*: `Optimizer::optimize()` `optim.cpp:27` (L-BFGS), `Optimizer::f()` `:260`,
  `setup()` `:187`. The trajectory is 4-D MINCO (x, y, z, yaw) (`include/planner/minco.hpp:85`
  `template<int dim> class Minco`; `Minco<4>` in `planner.cpp:73`).
- *Initial path ("candidate-correction")*: `src/tracker/src/planner/plan.cpp:17` (`PathPlanner::plan`).
  For each predicted target point it places an observation point at distance D on the line from the
  tracker. If the line-of-sight is blocked (`visible()` `:7`, a ray march on the inflated grid), it
  rotates the point about the target in ±1° steps up to 90° (`plan.hpp:13`, RA = π/2) until both
  target→point and point→tracker are visible (`feasible()` `plan.hpp:27-33`).
- *Map*: `src/tracker/src/planner/map.cpp:66` (`Map::update`). This is a rolling boolean occupancy grid
  built from each LiDAR scan with a cubic inflation. The target box is removed from the cloud using the
  `/target/odom` twist fields, which carry the target half-size (`map.cpp:95-98`). There is **no ESDF of
  the world**. The filtered point list is passed straight into the cost.
- *Reacquisition / lost logic*: `src/tracker/src/perception.cpp:101-124` sets `lost` when the detector
  returns nothing. `:75-78` freezes the Bézier prediction while lost. `Tracker::filter()`
  `src/tracker/src/perception/tracker.cpp:37-46` then publishes the position predicted by the frozen
  curve, and `Tracker::push(obs, refind)` `:60-71` resets the observation buffer on re-detection. The
  planner has **no separate search mode**: reacquisition comes implicitly from planning visibility
  toward the extrapolated target. The visibility "hold" check in the control loop is commented out
  (`planner.cpp:299-305`).
- *State machine*: minimal. `/triger` starts and `/back_trigger` stops (`planner.cpp:174-183`). The
  real-world RC-channel logic is in `scripts/control.py`.

**I/O.** Inputs:
- `/tracker/odom` (`nav_msgs/Odometry`; x, y, z, yaw)
- `/tracker/lidar` (`sensor_msgs/PointCloud2`)
- `/target/odom` (`nav_msgs/Odometry`; position plus size in twist)
- `/target/bezier` (`nav_msgs/Path` of 2m+1 control points)

Output: `/tracker/cmd` (`quadrotor_msgs/PositionCommand`: pos, vel, acc, jerk, yaw, yaw_dot) at
`tracker/hz` = 50 Hz (`planner.cpp:32,269`), plus `/tracker/plan` (Path sampled every 0.1 s).

**Target-state source.** In simulation the simulator publishes ground-truth target positions on
`/target/poses` at `tracker/fps` = 10 Hz (`simulator.cpp:159-172`). In the section read, the simulator
does not gate those positions by occlusion. In the real world the source is YOLO11n-Pose keypoints, a
depth lookup and MonoLoco orientation, run with TensorRT (`perception.cpp:96-122`).

**Key parameters.**

| Parameter | Value | Location |
|---|---|---|
| m (number of observations) | 2 | `src/tracker/launch/tracking.launch:25` |
| k (samples per piece) | 8 | `:26` |
| D (FoV depth) | 3.36 m | `:27` |
| τ (piece / observation interval) | 0.5 s | `:28` |
| β / α (vertical / horizontal FoV) | 42.5° / 69.4° | `:29-30` |
| γ (time regulariser) | 1e5 | `:31` |
| λ_p, λ_o, λ_v, λ_a | 1e5, 1, 1e4, 1e4 | `:32-35` |
| δθ (candidate rotation step) | 1° | `:36` |
| v_xy / a_xy limits | 2.0 m/s / 20 m/s² | `:13-14` |
| v_z / a_z limits | 0.1 / 0.1 | `:11-12` |
| yaw-rate / yaw-accel limits | 1.5 / 15 | `:15-16` |
| map | 40 m × 1.5 m height, 0.05 m resolution | `:19-22` |
| L-BFGS memory / max iterations | 64 / 64 | `planner.cpp:105-107` (defaults) |

**Compute and assets.** The README states no runtime. The planner is CPU-only with no solver
licence. The real-world detector needs TensorRT and ONNX models; those blobs are in the repo but over
1 MB and were not fetched. The bundled `bag/benchmark.bag` was also not fetched. Simulation needs no
weights.

**Portability to MOTAR.**

| Component | Class | Reason |
|---|---|---|
| Body-frame FoV-ESDF (precomputed frustum distance field + trilinear gradient) | REIMPLEMENT_IN_PYTHON | About 60 lines of numpy/scipy (`distance_transform_edt`) built once. Querying a batch of LiDAR/depth points with `grid_sample` on GPU is trivial. It suits the forward RGB-D camera and needs no world ESDF. Best single idea for a per-step "obstacles inside view frustum" occlusion feature or cost. |
| Obstacle-in-frustum occlusion penalty | REIMPLEMENT_IN_PYTHON | Torch-vectorisable over envs × points. As a reward or diagnostic it fits the dense-bar scenes. Note that it penalises all in-frustum points, not just line-of-sight points. |
| Candidate-correction observation point (rotate around target until LoS clear) | REIMPLEMENT_IN_PYTHON | Cheap ray-march over a voxel grid built from depth/LiDAR. Can produce a "visible viewpoint" goal fed to the frozen policy as its goal or relative-target input. |
| Bézier minimum-energy predictor with pre-inverted KKT | DIRECT_PORT_POSSIBLE (math only; rewrite, do not copy, because of AGPL) | A fixed small matrix; one matmul per step in torch. |
| Lost → freeze prediction and extrapolate; refind → reset buffer | REUSE_IDEA_ONLY | Trivial logic. MOTAR already has its own memory and belief handling. |
| 4-D MINCO + L-BFGS trajectory optimiser | NOT_APPLICABLE | The frozen policy outputs velocity and yaw-rate at 10 Hz. A full trajectory optimiser per env per step is too costly in Isaac Gym and could only act as an external reference generator. |
| YOLO/MonoLoco TensorRT perception | NOT_APPLICABLE | MOTAR uses simulator target state or its own perception. |

---

## 2. Elastic-Tracker (ICRA 2022, arXiv 2109.07111)

| Field | Value |
|---|---|
| URL | https://github.com/ZJU-FAST-Lab/Elastic-Tracker |
| HEAD | `0a302a2a9cc8b733e74941fdd4af3fb449447bea` |
| Commit date | 2023-06-16 11:23:52 +0800 |
| Default branch | `main` |
| Files / size | 196 files; 2.24 MB fetched + 7 blobs >1 MB not fetched (GIFs, prebuilt `odom_visualization` binaries, meshes) |
| Audit level | **FUNCTIONS_READ** — `src/planning/planning/src/planning_nodelet.cpp` `plan_timer_callback` (149–429) and `init` (744–782); `src/planning/planning/include/env/env.hpp` (1–800: ray check, SFC, visible-path A*, visible regions); `src/planning/planning/include/prediction/prediction.hpp` (full); `src/planning/traj_opt/src/traj_opt.cc` (ctor 255, landing `generate_traj` 380, `addTimeCost` 493, `grad_cost_p_tracking` 620, `grad_cost_visibility` 700); `src/detection/target_ekf/src/target_ekf_sim_node.cpp` (full). Mapping node mapped by grep only. |

**License.** `LICENSE` is **GNU GPL-3.0**, copyleft. Code copied into a distributed derivative must be
GPL-3.0. Reimplementing the ideas is fine. Two bundled parts carry permissive licences:
`src/planning/DecompROS/LICENSE` is BSD-3-Clause, and `traj_opt/include/traj_opt/minco.hpp` has an MIT
header (Zhepei Wang). Those two files could be copied with their notices kept.

**Stack.** C++14 catkin nodelets on ROS1 (the README names no distro; `.zsh` setup). Uses Eigen3
(`traj_opt/CMakeLists.txt:8`), DecompROS (ellipsoid decomposition), MINCO and LBFGS
(`lbfgs_raw.hpp`, header-only). The simulator's `local_sensing` uses CUDA depth rendering, with
`ENABLE_CUDA true` in `src/uav_simulator/local_sensing/CMakeLists.txt:6`, which the README says to
change. No commercial solver.

**Method map (path:line).**
- *Target prediction*: `prediction.hpp:57` (`Predict::predict`). This is A* over constant-acceleration
  motion primitives in 2-D (inputs ∈ {−3, 0, 3}² m/s², `:84-85`), collision-checked against the
  occupancy map, scored by ρ_a‖a‖ with a heuristic toward p + v·T. It has a 0.1 s budget (`:60,95-98`)
  and returns positions at `tracking_dt` intervals over `tracking_dur`.
- *Visibility model*: `env.hpp:94/128` (`checkRayValid`, 3-D DDA voxel traversal on the inflated
  occupancy grid). `visible_pair()` `env.hpp:646` sweeps a circle of radius `tracking_dist` around each
  predicted target position to find the largest angular sector [t_l, t_r] with clear line of sight.
  The result is a **fan-shaped visible region** (centre direction plus half-angle θ).
  `generate_visible_regions()` is at `:690`.
- *Visible path search*: `findVisiblePath()` `env.hpp:410` (per target) and `:513` (chained). It is
  6-connected A* that stops when within `tolerance_d` of the `tracking_dist` ring **and** has line of
  sight (`:414-416`).
- *Safe corridor*: `generateSFC()` `env.hpp:279`. It runs DecompROS `EllipsoidDecomp3D` on path segments
  with a 2 m local box (`planning_nodelet.cpp:347`), then shrinks/inflates polytopes (`:332-362`).
  `getPointCloudAroundLine()` is at `:172`.
- *Trajectory optimisation*: MINCO quintic (jerk) with spatio-temporal deformation over polytope
  vertices: `TrajOpt::generate_traj` (tracking variant `traj_opt.cc:325`, landing `:380`) and
  `objectiveFunc` `:167`.
  - The tracking-distance cost `grad_cost_p_tracking()` `:620` is an annulus [d−tol, d+tol] in xy plus
    a band in z.
  - The **visibility cost** `grad_cost_visibility()` `:700` is the angular constraint
    ⟨p−c, v−c⟩/(‖·‖‖·‖) ≥ cos(θ − θ_clear), keeping the drone inside each fan.
  - Both are applied in `addTimeCost()` `:493-558` with time-decaying weights ρ = 2^(−3i/M) over the
    first 4/5 of the prediction.
- *Yaw*: not optimised. It is set to atan2 toward the target at publish time
  (`planning_nodelet.cpp:397-404`).
- *State machine / fallbacks*: `plan_timer_callback` `planning_nodelet.cpp:149`. It contains:
  - force-hover until the drone is slow (`:193-195`);
  - hover-when-at-distance (`:218-231`);
  - replan from the previous trajectory state (`:269-279`);
  - REPLAN SUCCESS / FAILED-HOVER / EMERGENCY STOP / EXECUTE-LAST-TRAJ (`:392-427`);
  - a landing mode.
  There is **no explicit target-loss search**. Loss is handled only by the EKF coasting for up to 2 s
  and resetting after 5 s (`target_ekf_sim_node.cpp:24-31,94-97`).
- *Map*: `src/mapping` builds a probabilistic occupancy grid from a depth image synchronised with
  odometry (`mapping_nodelet.cpp:21-24,79`): 0.15 m resolution, 20×20×5 m local map, inflate 1
  (`launch/mapping.launch:29-33`), log-odds ray casting (`:22-27`).

**I/O.** Planner inputs:
- `gridmap_inflate` (`quadrotor_msgs/OccMap3d`)
- `odom` (`nav_msgs/Odometry`)
- `target` (`nav_msgs/Odometry` from the target EKF)
- `triger`, `land_triger` (`PoseStamped`)

Output: `trajectory` (`quadrotor_msgs/PolyTraj`, order-5 piecewise polynomial plus a single yaw,
`planning_nodelet.cpp:81-106`) at `plan_hz` = 20 Hz. `traj_server` converts it to `position_cmd`.
The mapping node takes a 640×480 depth image (`config/camera.yaml`).

**Target-state source.** In simulation the fake target publishes ground truth on `/target/odom`. It
goes through `target_ekf_sim_node` (constant-velocity EKF at 20 Hz). The FoV gate `check_fov` defaults
to false (`target_ekf_sim_node.cpp:20,78-91`), and there is **no occlusion gating**. On hardware the
source is YOLO bounding boxes into `target_ekf_node`.

**Key parameters** (`src/planning/planning/launch/planning.launch`).

| Parameter | Value | Line |
|---|---|---|
| plan_hz | 20 | 21 |
| K | 8 | 23 |
| vmax / amax | 3.0 / 6.0 | 24-25 |
| ρT / ρP / ρV / ρA | 100 / 1e4 / 1e3 / 1e3 | 26-29 |
| theta_clearance | 0.8 rad | 32 |
| clearance_d | 0.2 | 33 |
| tolerance_d | 0.3 | 34 |
| tracking_dist | 2.5 m | 35 |
| tracking_dur | 3.0 s | 36 |
| tracking_dt | 0.2 s | 37 |
| prediction vmax / ρ_a | 4.0 / 1.0 | 40-41 |
| rhosVisibility / rhoTracking (simulation1) | 1e4 / 1e3 | `simulation1.launch:20-21` |

**Compute and assets.** The README states no runtime. In code the search budget is 0.2 s
(`env.hpp:52`) and the prediction budget 0.1 s. CPU-only with no licensed solver. No learned weights.

**Portability to MOTAR.**

| Component | Class | Reason |
|---|---|---|
| Fan-shaped visible region (circle sweep around predicted target with LoS checks) + angular visibility constraint | REIMPLEMENT_IN_PYTHON | A cheap, batched torch ray-march on a voxel grid from LiDAR/depth. It yields a "visible bearing sector" that can serve as a geometric reacquisition/approach goal or as a reward term (cos-angle hinge). Fits dense vertical bars well, since the sweep is 2-D in xy. |
| Motion-primitive A* target predictor | REUSE_IDEA_ONLY | Sequential A* per env does not vectorise. For bars, constant-velocity or map-constrained rollouts in torch are a better match. |
| Visible-path A* + DecompROS SFC + MINCO optimisation | NOT_APPLICABLE | Optimisation-based full-trajectory planning is incompatible with a frozen 10 Hz velocity policy and too heavy per env. Useful only as an offline expert or teacher. |
| Annulus tracking-distance cost | DIRECT_PORT_POSSIBLE (formula; few lines) | Can be used as a reward term or a goal-distance band. |
| Hover / emergency / execute-last fallback ladder | REUSE_IDEA_ONLY | Supervisor design pattern. |

---

## 3. UAST — Unified Active Search and Tracking

| Field | Value |
|---|---|
| URL | https://github.com/qinliangql/UAST |
| HEAD | `1fcf5e0d34fc2cb2a0b7cf66d83374f39adb61bf` |
| Commit date | 2026-09-03 20:32:09 +0800 |
| Default branch | `main` |
| Files / size | 496 files (mostly the `Controller/` so3 simulator); 3.37 MB fetched + 4 blobs >1 MB not fetched: `UAST/saved/UAST_track/epoch50.pth`, `UAST/saved/YOPO_nav/epoch50.pth`, `Simulator/src/pointcloud/tree.ply`, `uav.dae` |
| Audit level | **FUNCTIONS_READ** — `UAST/test_uast_track.py`: `UASTNet.__init__` 208, `initialize_kalman_filter` 413, `kalman_predict` 448, `kalman_update` 472, `search_for_target` 493, `callback_set_goal` 586, `process_odom` 643, `callback_depth` 749, `callback_rgb` 894, `control_pub` 989, `main` 1144. Also `UAST/policy/uast_network.py` (full), `UAST/loss/track_loss.py` (full), `UAST/policy/uast_dataset.py` `__getitem__` 302, `UAST/policy/poly_solver.py` `calculate_yaw_track` 133. Configs read. Other files mapped by def/class grep. |

**License.** `LICENSE` is **MIT** (© 2024 TJU-Aerial-Robotics). Code may be copied into the research
repo, modified and redistributed if the copyright and licence notice are kept. This is the only fully
permissive tracking repo in the set.

**Stack.** Python 3.8.20 + PyTorch 2.4.1 / CUDA 11.8 (README). `UAST/requirements.txt` pins
opencv-python 4.11, scipy 1.10.1, numpy 1.22.3, open3d 0.19, ruamel-yaml, tensorboard, and lists
rospkg/catkin_pkg. ROS1 is used for I/O; the README says Ubuntu 20.04 and Jetson Orin/Xavier NX.
Optional TensorRT via `torch2trt`. The simulator and controller are separate catkin workspaces, and the
simulator has a CUDA sensor sim (`sensor_simulator_cuda_multi`).

**Method map (path:line).**
- *Network (policy)*: `UAST/policy/uast_network.py:60` `UASTNetworkTrack`. A depth backbone (YOPO
  ResNet, 64-ch) and a target-mask backbone (8-ch) are concatenated with the 9-D observation
  (v_c, a_c, goal_c in camera frame). `YopoHead` scores a 3×5 lattice of motion primitives (`forward` `:77-88`).
  The output per primitive is a tanh end-state (p, v, a) plus a softplus score. Lattice:
  `policy/primitive.py:28` `LatticePrimitive`.
- *Trajectory*: the selected primitive becomes per-axis quintic polynomials
  (`policy/poly_solver.py:4` `Poly5Solver`; built in `test_uast_track.py:867-872`). Its duration is
  `traj_time = 2·radio_range / velocity`.
- *Visibility / occlusion model (training loss)*: `UAST/loss/track_loss.py:62` `calculate_track_cost`.
  It samples the middle third of each trajectory and calls `check_line_of_sight()` `:114`, which places
  10 samples on the drone→target segment, queries a precomputed world SDF via `F.grid_sample`
  (`get_distance_cost` `:141`), and applies `relu(voxel_size − d)` as a **differentiable occlusion
  degree**. A vertical-FoV pitch hinge is added (`:86-107`). Total = 4·occlusion + pitch, ×5 (`:110-112`).
  The SDF comes from dataset `.ply` maps (`get_sdf_from_ply` `:270`, scipy `distance_transform_edt`).
  This is training-time only; the network is "mapping-free" at test time.
- *Target observation / visibility at test*: `callback_depth` `test_uast_track.py:749-829`. A synthetic
  quadcopter point cloud is placed at the **ground-truth** target position (from `target_topic`),
  projected into the depth image and z-buffered against the real depth (`:806-819`). Visible pixels form
  the target mask, and the mask goes to the network (`:836-854`).
- *Target position estimate*: `callback_rgb` `:929-962` back-projects the visible mask pixels with
  their depths into world coordinates and sets `noisy_goal`. A Kalman filter exists (`:413-491`), but
  its calls are commented out (`:320, 927, 959`).
- *Reacquisition / search state machine*: if fewer than 8 valid visible pixels persist for more than
  2.0 s, the node enters `search_mode` and resets a 2-D map (`:966-969`). `search_for_target()` `:493`
  picks one of the 10 farthest unexplored 2-D cells within 25 m, or falls back to 45° spiral headings
  (`:545-554`). The 2-D map is 200×200 cells at 0.5 m (`:327-331`). The node exits search on
  re-detection (`:960-962`). Search goals are re-chosen when within 5 m (`:507`).
- *Yaw*: `calculate_yaw_track()` `policy/poly_solver.py:133`. It blends direction toward the goal
  (weight 10) with velocity (0.1) and rate-limits to 0.3π rad/s.
- *Training*: the losses are in `UAST/loss/` (`loss_function.py:12` `UASTLoss`; safety, smoothness,
  guidance, track) and the dataset in `policy/uast_dataset.py:185`. **No training script is in the
  repo**, and the dataset (`dataset_path: "../dataset"`) is not included.

**I/O.** ROS inputs: `/drone1/sim/odom` (target ground truth, `Odometry`), `/drone2/sim/odom`,
`/drone2/depth_image` (32FC1), `/drone2/rgb_image`. Output: `/drone2/so3_control/pos_cmd`
(`PositionCommand`: pos, vel, acc, yaw, yaw_dot) at `ctrl_dt` = 0.02 s (50 Hz, `test_uast_track.py:234`).
Network tensors:
- depth `[1,1,96,160]`, normalised by 20 m;
- mask `[1,1,24,40]`;
- obs 9-D normalised and expanded to `[B,9,3,5]`;
- output endstate `[B,9,3,5]` and score `[B,3,5]`.

**Target-state source.** Ground-truth target odometry is the anchor of the mask rendering. Visibility
is real, because it is depth z-buffered against obstacles. The 3-D estimate comes from the depth of
visible pixels. In other words it is a GT-seeded simulated detector, not a learned detector.

**Key parameters.**

| Parameter | Value | Location |
|---|---|---|
| velocity | 6.0 m/s (target config: 5.0) | `UAST/config/traj_opt.yaml:2` (`traj_opt_target.yaml:2`) |
| loss weights wg / ws / wc / wt | 0.1 / 0.0015 / 0.5 / 0.4 | `traj_opt.yaml:9-12` |
| image / mask size | 96×160 / 24×40 | `:16-19` |
| lattice | 5×3 | `:22-23` |
| camera FoV | 90°×60° | `:24-25` |
| radio_range (horizon 2·r) | 5.0 m | `:28` |
| safety d0 / r | 1.2 / 0.6 | `:32-33` |
| camera intrinsics | fx = fy = 80, 160×90, max depth 20 m | `:57-64` |
| loss-side FoV | `fov_angle` = 40° (unused); mid-trajectory ratio [0.33, 0.66]; SDF voxel 0.2 m | `track_loss.py:28-36` |
| search interval / radius | 2.0 s / 25 m | `test_uast_track.py:338-339` |

**Compute and weights.** The node prints timing but the repo states no figure. Weights
`UAST/saved/UAST_track/epoch50.pth` exist in the repo but are over 1 MB and were not fetched, so their
size and validity are unverified. **Training data is not available, and there is no training script.**

**Static-reading observation (not verified).** In training, the mask is resized with
`cv2.resize(mask, (mask_width, mask_height))`, giving a 24×40 array (`uast_dataset.py:375`). At
inference, `test_uast_track.py:836` calls `cv2.resize(depth_mask, (self.mask_height, self.mask_width))`,
which gives 40×24, and then reshapes it to `[1,1,24,40]` (`:837`). This looks like a train/inference
mask-layout mismatch. Confirm before relying on the released weights.

**Portability to MOTAR.**

| Component | Class | Reason |
|---|---|---|
| Differentiable LoS-occlusion via SDF `grid_sample` along drone→target segment (`track_loss.py:114-172`) | DIRECT_PORT_POSSIBLE (MIT) | Already batched torch. It can become a GPU reward or diagnostic term over Isaac Gym envs using an SDF of the bar layout, which is known in simulation. This is the closest drop-in occlusion metric for MOTAR. |
| GT-seeded, depth-z-buffered target visibility mask | DIRECT_PORT_POSSIBLE (MIT) | MOTAR's RGB-D camera plus known target pose gives the same "visible-pixel" signal. It can be vectorised in torch over envs and is a realistic visibility oracle. |
| Unexplored-cell / spiral search when unseen > 2 s | REIMPLEMENT_IN_PYTHON | A simple supervisor that would emit a goal or bearing to the frozen policy during reacquisition. It is 2-D, which suits vertical bars. |
| Goal/velocity-blended rate-limited yaw | REIMPLEMENT_IN_PYTHON | Maps directly to MOTAR's yaw-rate command. |
| UASTNetworkTrack + primitive lattice + Poly5 | NOT_APPLICABLE as a replacement (frozen policy); REUSE_IDEA_ONLY as a teacher | It is a different action space (position-trajectory primitives at 50 Hz). The weights are trained on their maps and camera (90°×60°, 160×96), and no dataset or training script is available. |

---

## 4. FASTER (T-RO 2021 / IROS 2019) — navigation planner, not a tracker

| Field | Value |
|---|---|
| URL | https://github.com/mit-acl/faster |
| HEAD | `f431c97115b96ea08b85ef7796a53183e08441dc` |
| Commit date | 2025-03-10 18:32:39 -0400 |
| Default branch | `master` |
| Files / size | 598 files (553 under `thirdparty/DecompROS`, `thirdparty/jps3d`); 6.92 MB fetched + 7 blobs >1 MB not fetched (GIFs, DecompROS test bags/PNG) |
| Audit level | **FUNCTIONS_READ (limited)** — read: `Faster::replan` `faster/src/faster.cpp:296-595`, `Faster::yaw` `:650`, `Faster::getDesiredYaw` `:666`, `SolverGurobi::setObjective` `faster/src/solverGurobi.cpp:86`, `setPolytopesConstraints` `:180-250`; `faster/param/faster.yaml` (full). Everything else STRUCTURE_MAPPED. |

**License.** `LICENSE` is **BSD-3-Clause** (© 2020 Jesus Tordesillas, MIT ACL). Code may be copied
with the notice kept, without using the authors' names for endorsement. Bundled `thirdparty/DecompROS`
and `thirdparty/jps3d` are also BSD-3-Clause. **Gurobi is a commercial solver** (free academic
licence), so "copying" the method still needs Gurobi or a different MIQP solver.

**Stack.** C++ catkin on ROS1 Kinetic/Melodic (Ubuntu 16.04/18.04 per README). Gurobi 8.1/9.0/9.1
(`faster/FindGUROBI.cmake`, `solverGurobi.cpp`). Eigen, PCL, JPS3D, DecompROS, and snapstack_msgs (via
`faster/install/faster.rosinstall`). The external `global_mapper_ros` handles mapping. Gazebo is used
for simulation.

**Method map (path:line).**
- *Replanning loop*: `Faster::replan` `faster.cpp:296`. Steps:
  1. JPS from point A to the projected goal G (`:361`).
  2. Intersect with the sphere of radius Ra (`:373-382`).
  3. Convex decomposition in **free + unknown** space (whole trajectory, `:398`).
  4. Gurobi solve (`:418`).
  5. Find the first collision with unknown space M (`:452`).
  6. A **safe "rescue" trajectory** from R inside **free-known** space only (`:474-537`).
  7. Append the committed plan (`:548`).
- *Optimisation*: `SolverGurobi::setObjective` `solverGurobi.cpp:86` minimises Σ‖jerk‖² over N
  intervals. `setPolytopesConstraints` `:180` is a **MIQP**: binary variables assign each interval to
  one polytope (`:220-246`). Box limits on v/a/j are at `:390-404`. The time step is found by factor
  search (`findDT` `:494`, `genNewTraj` `:426`).
- *Map*: `occup_grid` / `unknown_grid` `PointCloud2` (`faster_ros.cpp:149-152`), synchronised in
  `mapCB` `:480`.
- *Yaw*: `getDesiredYaw` `faster.cpp:666` looks toward the next M point and is saturated by `w_max`
  (`:650-664`).
- *State machine*: `DroneStatus` YAWING / TRAVELING / GOAL_SEEN / GOAL_REACHED (`faster.cpp:671-693`,
  `changeDroneStatus` `:929`).
- No target prediction, no visibility or occlusion model, no reacquisition: **this is not a tracking
  planner.**

**I/O.** Inputs: `state` (`snapstack_msgs/State`), `term_goal` (`PoseStamped`), `mode`,
`occup_grid`, `unknown_grid` (`PointCloud2`). Output: `goal` (`snapstack_msgs/Goal`; pos, vel, accel,
jerk, yaw, dyaw) at `dc` = 0.01 s (100 Hz, `faster.yaml:5`), plus the whole/safe trajectory paths.

**Target-state source.** Not applicable (navigation to a clicked goal).

**Key parameters** (`faster/param/faster.yaml`).

| Parameter | Value | Line |
|---|---|---|
| dc | 0.01 | 5 |
| drone_radius | 0.42 | 7 |
| N_whole / N_safe | 6 / 6 | 11-12 |
| Ra | 4.0 m | 14 |
| w_max | 4.0 rad/s | 15 |
| z_max | 2 | 19 |
| inflation_jps | 0.47 | 20 |
| v / a / j max | 5 / 5 / 8 | 23-25 |
| gamma factors | 20 | 28-33 |
| max_poly_whole / max_poly_safe | 3 | 38-39 |
| dist_max_vertexes | 1.5 | 40 |
| gurobi_threads | 0 | 41 |

**Compute and assets.** The README states no runtime. A Gurobi licence is needed. No weights.

**Portability to MOTAR.**

| Component | Class | Reason |
|---|---|---|
| Free-known vs unknown split with a guaranteed "safe-in-known-space" backup | REUSE_IDEA_ONLY | The idea of a supervisor that always keeps a known-free fallback while the policy chases a target through partially observed bars is relevant. The implementation is not. |
| JPS + convex decomposition + Gurobi MIQP | NOT_APPLICABLE | Needs a commercial solver, is ROS-bound and sequential, and its position/jerk trajectory output is incompatible with the frozen velocity policy. |
| Rate-saturated look-ahead yaw | REUSE_IDEA_ONLY | Trivial. |

---

## 5. PANTHER (Perception-Aware Trajectory Planner in Dynamic Environments, arXiv 2103.06372)

**Clone outcome.** The clone **succeeded**. The earlier NOT_FOUND report was not reproduced;
`https://github.com/mit-acl/panther` resolves.

| Field | Value |
|---|---|
| URL | https://github.com/mit-acl/panther |
| HEAD | `50be45b30282d694e3b890dfef920090432920a1` |
| Commit date | 2022-11-01 12:13:15 -0400 |
| Default branch | `master` |
| Files / size | 513 tree entries; 12.18 MB fetched + 16 blobs >1 MB not fetched (GIFs, `panther/include/exprtk.hpp`, mesh textures, DecompROS bags) |
| Submodules | 6, not fetched: catkin_simple, separator, behavior_selector (GitLab), snapstack_msgs (GitLab), minvo, panther_extra (`.gitmodules`) |
| Git LFS | `.gitattributes` puts `*.casadi` in Git LFS. The committed `.casadi` files are LFS pointers; e.g. `visibility.casadi` points to a 12.56 MB object. The compiled optimisation problems are therefore not in the git objects. |
| Audit level | **FUNCTIONS_READ (limited)** — read: perception-cost section of `panther/matlab/main.m` (224–372); `Panther::sampleFeaturePosVel` `panther/src/panther.cpp:578`; obstacle-focus heuristic inside `Panther::replan` (`:1035-1070`); `panther/param/panther.yaml`. Everything else STRUCTURE_MAPPED. |

**License.** `LICENSE` is **BSD-3-Clause** (© 2020 Jesus Tordesillas, MIT ACL). Permissive: copy with
the notice kept. Its dependencies have other terms: CasADi (LGPL), IPOPT (EPL), **HSL MA27** (free
personal/academic licence, not redistributable), and CGAL 4.14 (GPL/LGPL components). Check these if
redistributing anything built on them.

**Stack.** C++ catkin on ROS1 Noetic / Ubuntu 20.04 (README). Uses CasADi + IPOPT (+ MA27), CGAL,
Boost Graph, Eigen, PCL and DecompROS. MATLAB is needed only to regenerate the problem
(`panther/matlab/main.m`). Gurobi is optional (`src/examples/test_gurobi.cpp`). Simulation uses Gazebo
(`panther_gazebo`).

**Method map (path:line).**
- *What is tracked*: the "feature" kept in view is the **dynamic obstacle with the highest heuristic
  collision probability**, not a mission target. The heuristic is in `panther.cpp:1035-1070`
  (`probMultivariateNormalDist` along the straight line A→G). Samples come from
  `Panther::sampleFeaturePosVel` `panther.cpp:578`. With no obstacle, the code falls back to the
  terminal goal (`:616-626`).
- *Obstacle prediction*: `panther/src/tracker_predictor.cpp`. `cloud_cb` `:182` does PCL clustering and
  Hungarian-style track association (`getCostRowColum` `:148`, `addNewTrack` `:155`).
  `generatePredictedPwpForTrack` `:710` fits a polynomial with mean and variance via the CasADi
  `get_mean_variance_pred_*.casadi` functions. Parameters are in `panther/param/predictor.yaml`.
- *Visibility / FoV cost*: `panther/matlab/main.m:285-320`. The feature is transformed into the camera
  frame using the differential-flatness attitude from acceleration + yaw (`:249-283`). A smooth FoV
  membership is computed as `sigmoid(γ·(cos∠(c_P, z) − cos(θ/2)))` (`:293-294`), and the cost is
  `−isInFOV / (offset + ε·‖ṡ‖²)` (projection-velocity weighting, `:320`), integrated by Simpson's rule
  (`:347`).
  **No static-obstacle occlusion term**: the model is FoV-cone membership only.
- *Total cost*: `main.m:368-372` combines pos/yaw smoothness, c_fov·fov_cost and final pos/yaw.
- *Optimiser*: `SolverIpopt::optimize` `panther/src/solver_ipopt.cpp:422`. It loads `op.casadi`,
  `op_fixed_pos.casadi`, `fit_yaw.casadi` and `visibility.casadi` (`:85-89`, all LFS). Position and
  yaw are clamped uniform B-splines (`MyClampedUniformSpline.m`; `bspline_utils.cpp`). The yaw initial
  guess comes from a graph search over sampled yaws (`yaw_guess_generator.cpp:100`). The position guess
  comes from `OctopusSearch` (`octopus_search.cpp`, A*-like with separating-plane LPs). Safety uses
  convex hulls of obstacle trajectories (`convexHullsOfCurves` `panther.cpp:564`).
- *State machine*: `changeDroneStatus` `panther.cpp:1369`, `isReplanningNeeded` `:860`,
  `replan` `:906`.

**I/O.** Inputs: `state` (`snapstack_msgs/State`), `term_goal` (`PoseStamped`), and obstacle
trajectories `panther_msgs/DynTraj`. The trajectories arrive on `/trajs`, which is ground truth when
`perfect_prediction` is true (the default in `launch/simulation.launch:5`), or on `trajs_predicted`
from `tracker_predictor` (`panther_ros.cpp:239-252`). The point cloud feeds the tracker_predictor.
Output: `goal` (`snapstack_msgs/Goal`) at `dc` = 0.01 s, plus the own trajectory on `/trajs`.

**Target-state source.** In default simulation, ground-truth obstacle trajectories
(`perfect_prediction:=true`). Otherwise, a point-cloud cluster tracker.

**Key parameters** (`panther/param/panther.yaml`).

| Parameter | Value | Line |
|---|---|---|
| dc | 0.01 | 6 |
| drone_radius | 0.1 | 8 |
| angle_deg_focus_front | 90 | 25 |
| Ra | 4.5 m | 32 |
| v_max | 2.6 | 35 |
| a_max | 15.5 | 36 |
| j_max | 50 | 37 |
| ydot_max | π | 38 |
| FoV | 70°×70°, depth 5 m | 40-42 |
| c_fov | 40 | 51 |
| c_final_pos | 25 | 52 |
| gamma (discretisation step) | 0.1 s | 89 |
| solver runtime bounds | 0.005–0.350 s | 95-96 |

**Compute and assets.** The yaml bounds the runtime at 0.35 s, but the README states no runtime
figure. Needs IPOPT + MA27 (HSL licence) and the LFS `.casadi` files. No learned weights.

**Portability to MOTAR.**

| Component | Class | Reason |
|---|---|---|
| Smooth sigmoid FoV-membership with flatness-derived camera attitude | REIMPLEMENT_IN_PYTHON | A few lines of torch. A differentiable "target in FoV" score can be a reward term. MOTAR's yaw-rate control makes the yaw-dependent part directly relevant. |
| Image-plane velocity (‖ṡ‖²) weighting | REUSE_IDEA_ONLY | Useful for reducing motion blur and keeping the track, but secondary. |
| Collision-probability-based "which object to watch" selection | REUSE_IDEA_ONLY | Relevant only if MOTAR adds dynamic obstacles. |
| CasADi/IPOPT B-spline joint pos-yaw optimisation, Octopus search, tracker_predictor | NOT_APPLICABLE | Licensed solver and LFS-generated problem; ROS-bound; heavy; incompatible with the frozen velocity policy. |

---

## 6. BPOD — "Probabilistic Visibility-Aware Trajectory Planning for Target Tracking in Cluttered Environments" (arXiv 2306.06363)

- PDF checked: `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/downloaded/BPOD__2306.06363v2.pdf`
  (v2, 27 Aug 2024; Gao, Wu, Su, Zhou, Ma, Liu, Liu).
- `env -u LD_LIBRARY_PATH pdftotext <file> - | grep -i github` returned **0 matches**. A broader grep
  over the 1,026 extracted lines for `http`, `www.`, `source code`, `open source`, `video` and
  `supplementary` also found **no URL of any kind**.
- **Result: no code link in paper.** There is nothing to audit. The audit level is N/A, and the
  portability verdict rests only on the paper's text (REUSE_IDEA_ONLY).

---

## Summary table

| Repo | Commit (date) | License | Stack | Key function map | Target-state source | Portability verdict |
|---|---|---|---|---|---|---|
| Eva-Tracker | `636e26e` (2026-03-05) | AGPL-3.0 (strong copyleft; do not copy code) | C++14, ROS1 Noetic, Eigen/PCL/OpenMP; own L-BFGS; TensorRT+YOLO11-Pose+MonoLoco for real flight | FoV-ESDF `esdf.cpp:31,109,168`; occlusion/safety cost `optim.cpp:345-379`; observation cost `optim.cpp:310-325`; L-BFGS `optim.cpp:27`; candidate-correction `plan.cpp:17`; Bézier predictor `predictor.hpp:69`; lost/refind `perception.cpp:101-124`, `tracker.cpp:37-71` | Sim: GT `/target/poses` (10 Hz, no occlusion gate in the code read); real: YOLO pose + depth | FoV-ESDF + in-frustum occlusion cost + candidate viewpoint: REIMPLEMENT_IN_PYTHON (clean-room); MINCO optimiser: NOT_APPLICABLE |
| Elastic-Tracker | `0a302a2` (2023-06-16) | GPL-3.0 (copyleft; bundled DecompROS BSD-3, minco.hpp MIT) | C++14, ROS1 nodelets, Eigen, DecompROS, MINCO/LBFGS; CUDA depth sim | Visible fan `env.hpp:646,690`; LoS DDA `env.hpp:94`; visible A* `env.hpp:410,513`; SFC `env.hpp:279`; visibility cost `traj_opt.cc:700`; tracking annulus `traj_opt.cc:620`; time costs `traj_opt.cc:493`; predictor `prediction.hpp:57`; FSM `planning_nodelet.cpp:149` | Sim: GT target odom → CV-EKF (FoV check off by default, no occlusion gate); real: YOLO → EKF | Visible-fan + angular visibility hinge: REIMPLEMENT_IN_PYTHON; annulus cost: DIRECT_PORT_POSSIBLE (formula); A*/SFC/MINCO: NOT_APPLICABLE |
| UAST | `1fcf5e0` (2026-09-03) | MIT (copy OK with notice) | Python 3.8, PyTorch 2.4.1/CUDA 11.8, OpenCV, open3d, scipy; ROS1 I/O; optional TensorRT | Network `uast_network.py:60`; LoS-occlusion loss `track_loss.py:62,114,141`; GT-seeded z-buffer mask `test_uast_track.py:749-829`; target back-projection `:929-962`; search FSM `:493,966-969`; yaw `poly_solver.py:133` | GT target odom → rendered quadcopter points z-buffered against real depth → visible-pixel back-projection | SDF LoS-occlusion term + z-buffer visibility mask: DIRECT_PORT_POSSIBLE; search supervisor and yaw: REIMPLEMENT_IN_PYTHON; network: NOT_APPLICABLE as replacement (weights unfetched, no dataset or training script, possible mask-layout mismatch) |
| FASTER | `f431c97` (2025-03-10) | BSD-3-Clause (needs commercial Gurobi) | C++, ROS1 Kinetic/Melodic, Gurobi MIQP, JPS3D, DecompROS | `Faster::replan` `faster.cpp:296`; MIQP `solverGurobi.cpp:86,180`; yaw `faster.cpp:666` | N/A (navigation to a goal; no target) | Known-free safe-backup idea: REUSE_IDEA_ONLY; rest NOT_APPLICABLE |
| PANTHER | `50be45b` (2022-11-01) — clone OK (earlier NOT_FOUND not reproduced) | BSD-3-Clause (deps: CasADi LGPL, IPOPT EPL, HSL MA27, CGAL GPL/LGPL; `.casadi` in LFS) | C++, ROS1 Noetic, CasADi+IPOPT, CGAL, MATLAB codegen; 6 submodules | FoV sigmoid cost `main.m:285-320,368`; feature choice `panther.cpp:578,1035-1070`; IPOPT `solver_ipopt.cpp:422`; predictor `tracker_predictor.cpp:182,710` | Dynamic-obstacle trajectories: GT `/trajs` in default sim (`perfect_prediction:=true`) or point-cloud cluster tracker | Sigmoid FoV-membership term: REIMPLEMENT_IN_PYTHON; no occlusion model; optimiser NOT_APPLICABLE |
| BPOD (2306.06363) | — | — | — | no code link in paper (0 GitHub hits, no URLs in PDF text) | — | REUSE_IDEA_ONLY (paper only) |

## Cross-repo notes for MOTAR

1. **No repo gives a target-occlusion signal that is independent of ground truth.** Eva and Elastic
   simulators feed ground-truth target state with no occlusion gating (in the code read). UAST gates
   ground truth through a depth z-buffer, which is the most realistic of the three and is MIT-licensed.
2. **Occlusion formulations are ranked here by fit to MOTAR** (GPU, batched, frozen policy). This
   ranking is the auditor's judgement:
   1. UAST SDF line-of-sight sampling (`track_loss.py:114`).
   2. Eva body-frame FoV-ESDF penalising obstacles inside the frustum (`esdf.cpp` + `optim.cpp:345`).
   3. Elastic fan-shaped visible sector around the target (`env.hpp:646`).
   4. PANTHER FoV sigmoid (no occlusion).
3. **Licence gate.** Code may be copied only from UAST (MIT) and FASTER/PANTHER (BSD-3). Eva (AGPL-3.0)
   and Elastic (GPL-3.0) must be clean-room reimplemented from their papers.
4. **Reacquisition.** Only UAST has an explicit search mode (2 s timeout, then frontier/spiral goals).
   Eva relies on prediction-freeze plus visibility-aware replanning. Elastic and PANTHER have none
   beyond estimator coasting.
