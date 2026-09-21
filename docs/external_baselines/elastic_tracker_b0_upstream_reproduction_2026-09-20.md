# B0 — Elastic Tracker upstream reproduction — 2026-09-20

**Verdict: `UPSTREAM_REPRODUCTION_FAIL`.**

The upstream baseline could not be built or run in this environment. The cause is environmental and
externally resolvable — **not** a defect in Elastic Tracker, and not evidence about its behaviour.
No performance quantity was recorded, and none could have been: nothing executed.

```text
B0 = UPSTREAM_REPRODUCTION_FAIL   (build blocked; see §3)
B1 = NOT AUTHORISED               (gate requires B0 PASS or PARTIAL)
B2..B5 = NOT AUTHORISED
```

Per the lifecycle gate, **B1 was not started.** The contract-diff matrix is not written here.

## 1. Pinned upstream

| | |
|---|---|
| Repository | `https://github.com/ZJU-FAST-Lab/Elastic-Tracker` (official ZJU-FAST-Lab) |
| Commit | `0a302a2a9cc8b733e74941fdd4af3fb449447bea` |
| Branch | `main` |
| Upstream commit date | 2023-06-16T11:23:52+08:00 |
| Clone timestamp (UTC) | 2026-09-19T15:28:27Z |
| Tree size | 49 MB |
| **License** | **GNU GPL v3** |
| Paper | Elastic Tracker, arXiv:2109.07111, ICRA 2022 (v2 read 2026-03-02) |
| Local path | `/home/fair/workspaces/external_baselines/elastic-tracker-upstream` (outside the MOTAR repo) |

No fork was used. Nothing was vendored into MOTAR or MOTAR-public.

## 2. Licence review — this decides the integration mode

> Elastic Tracker is GPL-3.0 while MOTAR is BSD-3-Clause. MOTAR therefore treats Elastic Tracker as
> a pinned external dependency and does not vendor it, avoiding redistribution/licensing ambiguity.

This is a redistribution-hygiene decision, not a claim about what co-location would legally do. In
particular it does **not** assert that placing the two side by side would relicense independent
MOTAR files. The licence review supports exactly one mode:

```text
EXTERNAL_DEPENDENCY_ONLY
```

Elastic Tracker must stay in a separate workspace, outside the MOTAR tree and outside the public
snapshot. This was the preferred mode already; the licence makes it the required one.

## 3. Why the build is blocked

Elastic Tracker is a ROS 1 / catkin project. Its documented flow is `catkin_make` followed by four
`roslaunch` commands.

| blocker | detail | resolvable by |
|---|---|---|
| **ROS 1 absent** | `/opt/ros` does not exist; no `roscore`, `catkin_make`, `rosversion` on PATH | installing `ros-noetic-*` |
| **No root** | `sudo -n true` → "a password is required"; apt cannot install ROS | the user |
| **Disk headroom** | 3.5 GB free against a ~3.0 GB `ros-noetic-desktop-full` install, on a filesystem already at 97 % | freeing space first |

No workaround was attempted. Installing a system-wide ROS distribution is a machine-level change
that was not authorised, and the disk margin would be under 500 MB even if it were.

## 4. Environment recorded

| | |
|---|---|
| OS | Ubuntu 20.04.6 LTS |
| Kernel | 5.15.0-139-generic |
| ROS | **not installed** (Noetic is the matching distro for 20.04) |
| Compiler | gcc 9.4.0 |
| CUDA toolkit | `/usr/local/cuda-11.7` present; `nvcc` not on PATH |
| GPU | RTX 3070 (compute capability **8.6**) |
| MOTAR environment | untouched — no conda env, worktree or PATH was modified for this task |

## 5. Anticipated upstream patches — NOT confirmed by building

Because nothing was compiled, these are **read from source, not observed**. They are recorded so
that a future attempt does not rediscover them, and they must be re-derived from an actual build
before being treated as fact.

| # | Issue | Evidence | Classification |
|---|---|---|---|
| P-1 | CUDA arch mismatch | `src/uav_simulator/local_sensing/CMakeLists.txt` sets `ENABLE_CUDA true` with `-gencode arch=compute_61,code=sm_61`. This machine is sm_86. The README explicitly says "remember to change the CUDA option". | `UPSTREAM_REPRODUCTION_PATCH` (anticipated) |
| P-2 | Missing ROS deps | `vikit_ros` and `svo_msgs` are declared as dependencies but are **not** vendored under `src/`. Every other non-standard dependency is vendored (`catkin_simple`, `DecompROS`, `quadrotor_msgs`, `pose_utils`, `traj_opt`, `mapping`, `object_detection_msgs`). | external dependency to obtain (anticipated) |

No patch was written or applied. Upstream is byte-identical to the pinned commit.

## 6. What could be established without building

Source reading was possible and was done, because §8 of the task makes the target-semantics audit a
precondition for B1 regardless of build state.

### 6.1 The public simulation does **not** perform visual target tracking

This is the single most important finding of B0, and it changes how any future comparison must be
designed.

`simulation1.launch` wires the tracker's target input like this:

```xml
<node pkg="target_ekf" name="target_ekf_sim_node" type="target_ekf_sim_node">
  <rosparam command="load" file="$(find target_ekf)/config/camera.yaml" />
  <param name="pitch_thr" value="37"/>
  <remap from="~yolo" to="/target/odom"/>        <!-- the TARGET'S OWN ODOMETRY -->
  <remap from="~target_odom" to="/target_ekf_odom"/>
</node>
```

The input named `yolo` — which in the real system carries detections — is remapped to
**`/target/odom`, the simulated target's ground-truth odometry**. `target_ekf_sim_node` subscribes
to it as `nav_msgs::Odometry` and feeds the target's true position and orientation straight into the
EKF.

There are two separate nodes, and the distinction is the whole point:

| | `target_ekf_node` (real) | `target_ekf_sim_node` (public simulation) |
|---|---|---|
| Input type | `object_detection_msgs::BoundingBoxes` | `nav_msgs::Odometry` |
| Range estimate | from bbox height: `depth = 0.7 / height * fy_` | not estimated — ground truth |
| Perception in the loop | yes (a detector must supply boxes) | **none** |

**And the field-of-view gate is off by default.** `check_fov_` is initialised `false`
(`target_ekf_sim_node.cpp:20`) and `simulation1.launch` never sets `check_fov` — it sets only
`pitch_thr`. So in the documented flow the planner receives the target's true pose continuously,
with no FOV test, no occlusion test and no detection dropout.

When `check_fov` *is* enabled it is still a geometric frustum test on the ground-truth position
(camera 640×480, fx 346.74, fy 349.13, plus a depth gate of 0.1–5.0 m) — not a perception model.

**Stated plainly, as the task requires:**

```text
PUBLIC_SIM_TARGET_INPUT = PRIVILEGED_GROUND_TRUTH_ODOMETRY
```

The public Elastic Tracker simulation feeds privileged oracle target state to the planner, so it is
**not an equivalent visual-perception benchmark to MOTAR**. This interpretation stands unless
contrary upstream evidence is found. It must not be described as visual target tracking. The
"visibility guarantee" in its objective is about *planning to keep the target visible*, not about
*sensing* whether it is.

### 6.2 Consequence for any future comparison

A comparison in which MOTAR consumes sensor-only evidence with measured perception error while
Elastic Tracker consumes ground-truth target pose is not a fair contest, in either direction. This
asymmetry would have to be declared as a first-class unmatched dimension, alongside the four already
expected (action space, vehicle model, target sensing, simulator), or addressed by degrading the
target stream — which would itself be a declared modification of the baseline.

This does **not** downgrade the classification on its own. It is recorded so that B1, if it is ever
authorised, starts from it.

### 6.3 Target motion in the public simulation

`fake_target.launch` starts `mockamap` as the global map and runs the target as a **second
so3_quadrotor** with its own controller, driven to goals with `use_global_map=true`. The target is
therefore obstacle-aware and autonomously collision-free — structurally similar to MOTAR's TM-E2 —
but it does not evade. The paper's own conclusion names escaping targets as future work.

## 7. What was deliberately not done

* No performance quantity was recorded. B0 is a software-reproduction gate; no success rate,
  collision rate or comparison of any kind appears in this document.
* No adapter was written. No MOTAR code was involved at any point.
* No upstream source was modified.
* No B1 contract-diff matrix was produced — the gate forbids it while B0 is FAIL.
* No MOTAR GPU evaluation and no PPO training was started.

## 8. Environment recovery — 2026-09-21

### 8.1 Disk headroom resolved

| | |
|---|---|
| Before | 3.1 GB free (98 %) |
| After | **6.6 GB free (95 %)** |

Reclaimed 3.5 GB from two sources only, both verifiable as safe:

* **1.16 GB — this session's own artifacts**: a clone of MOTAR-public (already pushed; the remote
  holds everything) and a public release-candidate snapshot (regenerable by
  `tools/build_public_release_candidate.py`).
* **2.33 GB — conda packages referenced by no environment**, as determined by `conda clean
  --packages` itself, not by inspection.

**Nothing belonging to MOTAR or to the user was touched**: `datasets/` (34 GB), `results/` (3.1 GB,
which holds the frozen raw evaluation), `aerial_gym/` (3.2 GB), `detector_runs/`,
`tensorboard_archive/`, the prior release candidate, the Cursor editor state (1.6 GB and growing)
and other sessions' `/tmp` directories were all left alone.

The `aerialgym` environment was verified intact after the conda operation: torch 2.4.1+cu121 with
CUDA available, numpy 1.24.4, matplotlib 3.7.5, and 32 MOTAR tests green.

6.6 GB free against a ~3 GB install leaves roughly 3.6 GB headroom, rather than the few hundred
megabytes that would have remained before.

### 8.2 Dependency audit against the pinned upstream

Resolved from source; the earlier note that `vikit_ros`/`svo_msgs` are "missing dependencies" needs
qualifying.

| finding | evidence |
|---|---|
| `local_sensing` **is** on the documented demo path | `simulation1.launch` → `mapping.launch` → `uav_simulator.launch`, which starts `local_sensing_node`'s `pcl_render_node` (`sensing_horizon 5.0`, matching the EKF's 0.1–5.0 m depth gate) |
| `vikit_ros` / `svo_msgs` are **probably not build requirements** | They appear only in `local_sensing/package.xml` as build/run depends. **Neither CMake branch `find_package`s them** — the CUDA branch requires `roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs dynamic_reconfigure`, the non-CUDA branch a subset. They look like stale declarations. `ANTICIPATED_NOT_CONFIRMED` — a real build decides this. |
| `cmake_utils` is **not** a dependency | Referenced only by `src/uav_simulator/uav_utils/CMakeLists.txt`, which has **no `package.xml`**, so catkin never builds it. The reference is inert. |
| **Armadillo is genuinely required** | `pose_utils` and `odom_visualization` need it, and `odom_visualization` is started by both `simulation1.launch` and `fake_target.launch`. |
| Qt is required | `decomp_ros_utils` builds an rviz plugin. |

### 8.3 Commands for the user to run

`sudo -n true` reports that a password is required. **I did not attempt to bypass it, and these were
not executed.** They are listed exactly as they should be run.

```bash
# 1. ROS Noetic (Ubuntu 20.04's matching distro)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-dynamic-reconfigure ros-noetic-nodelet ros-noetic-tf \
    libarmadillo-dev
```

`ros-noetic-desktop` is chosen over `desktop-full`: it carries rviz and the generic robot libraries
without Gazebo and the full perception stack, which this demo does not use, and saves roughly a
gigabyte.

### 8.4 Build sequence once ROS exists — documented build first

```bash
source /opt/ros/noetic/setup.bash
cd /home/fair/workspaces/external_baselines/elastic-tracker-upstream
catkin_make                       # upstream's documented command, UNMODIFIED
```

**The CUDA configuration is not touched until this build has actually been run.** The `sm_61` →
`sm_86` question stays `ANTICIPATED_NOT_CONFIRMED` until a compiler genuinely fails on the
architecture. If it does fail, and the fix matches the README's own instruction to "change the CUDA
option", it is recorded as an **environment/build reproduction patch — not an algorithm
modification**, because it changes which GPU the renderer targets and nothing about the planner.

Then the documented launch sequence, in four terminals:

```bash
source devel/setup.bash           # README says setup.zsh; this shell is bash
roslaunch mapping rviz_sim.launch
roslaunch planning fake_target.launch
roslaunch planning simulation1.launch
./sh_utils/pub_triger.sh
```

### 8.5 Status

B0 remains **`UPSTREAM_REPRODUCTION_FAIL`**. Disk is no longer a blocker; the remaining blocker is
solely that ROS 1 cannot be installed without the user's password. Nothing else in this document
changes.

## 9. Classification after B0

```text
Elastic Tracker = PARTIALLY_MATCHABLE   (unchanged)
```

The classification is **not** raised — replay feasibility was never demonstrated, and B0 did not
run. It is **not** lowered to `BLOCKED` either: nothing about the baseline itself was shown to be
fatally incompatible. What failed is this machine's ability to build a ROS 1 project, which is not a
property of Elastic Tracker.

The oracle-target finding in §6.1 is a serious design constraint for any future comparison and is
carried forward as such, but it is a matter for B1 to classify, and B1 is not authorised.
