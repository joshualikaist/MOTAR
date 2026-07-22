# MOTAR

**MOTAR: Moving Object Tracking and Reinforcement-Learning-Based Approach for UAV Navigation in Random Obstacle Fields**

This repository is a research fork of the [Aerial Gym Simulator](https://github.com/ntnu-arl/aerial_gym_simulator).
It studies raw-sensor-based UAV target localization, tracking, and approach in random obstacle
environments. A temporal camera–LiDAR fusion model must infer the moving target state under
occlusion; the deployed policy is never given ground-truth target position. The experimental
environment follows the setup of
[NavRL](https://github.com/Zhefan-Xu/NavRL), reimplemented on top of Aerial Gym.
The current method is specified in `PERCEPTION_TRANSFORMER_PLAN.md`; the staged roadmap is in
`RESEARCH_PLAN.md` and `ROADMAP.md`.

Research work lives on the `research/navrl-env` branch; `main` tracks upstream Aerial Gym.

---

## Getting Started (MOTAR)

This section is a self-contained guide: from a fresh machine to watching a drone fly
in the simulator. It targets the Phase 1 task `navrl_task` (static obstacles + a
stationary goal, LiDAR-based navigation).

**The arena (`navrl_bars_env`)** is a controlled environment: an otherwise-empty
24×24×3 m space with **density-controlled static vertical bars** (`NAVRL_NUM_BARS`,
default 48; random footprint 0.4–0.8 m per side, height fixed 2 m). Bars are scattered
per episode by **NavRL-style uniform random sampling with rejection** (min 1.5 m
center-to-center; after 128 failed attempts the threshold relaxes ×0.8 so high-density
fields don't stall). The drone flies **in 2D at a fixed 1 m altitude** (vertical
velocity command is zeroed), spawns at the left edge (x≈0), and must cross the whole
bar field to a goal placed on the far side at x=k — the epoch-proportional curriculum
pushes k from ~7 m out to the far wall (~24 m). See `navrl_run2038_random_layout.png`
in the workspace root for a top-down view.

### 1. Prerequisites

| Component | Tested version |
|-----------|----------------|
| OS | Ubuntu 20.04 |
| GPU | NVIDIA, ≥ 8 GB VRAM (developed on an RTX 3070 8 GB) + recent driver (CUDA 12.x) |
| [Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym) | 1.0rc4, unpacked to `~/isaacgym` |
| Conda | Miniconda / Anaconda |

### 2. Get the code + set up the environment

The workspace is **two repos side by side**: this one (`aerial_gym_simulator`) plus the
`urdfpy` it depends on. Clone both under one `src/` folder.

**⚡ Fastest path — one script.** After §1 (conda installed + Isaac Gym unpacked to
`~/isaacgym`):

```bash
mkdir -p ~/workspaces/aerial_gym_ws/src && cd ~/workspaces/aerial_gym_ws/src
git clone -b research/navrl-env git@github.com:joshualikaist/MOTAR.git aerial_gym_simulator
cd aerial_gym_simulator && ./bootstrap_second_machine.sh
```

`bootstrap_second_machine.sh` clones `urdfpy`, creates the `aerialgym` conda env, and
pip-installs Isaac Gym + this repo + rl_games + warp + urdfpy, then runs a smoke test.
(Override the Isaac Gym path with `ISAACGYM_PATH=/path ./bootstrap_second_machine.sh`.)

**Manual — exactly what the script does**, if you prefer step by step:

```bash
# (a) Get the code: this repo + the urdfpy fork it needs (unmodified upstream mmatl/urdfpy)
mkdir -p ~/workspaces/aerial_gym_ws/src && cd ~/workspaces/aerial_gym_ws/src
git clone -b research/navrl-env git@github.com:joshualikaist/MOTAR.git aerial_gym_simulator
git clone https://github.com/mmatl/urdfpy.git

# (b) Create and activate a Python 3.8 conda env
conda create -n aerialgym python=3.8 -y
conda activate aerialgym

# (c) Install Isaac Gym Preview 4 (downloaded from NVIDIA, unpacked to ~/isaacgym)
cd ~/isaacgym/python && pip install -e .

# (d) Install this repo (editable) + its deps + the urdfpy fork
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator
pip install -e .
pip install rl-games==1.6.5 warp-lang==1.0.0
pip install -e ../urdfpy
```

**Two gotchas that will bite you (already handled on the dev machine):**

1. **Import order** — in any custom script, import `isaacgym` (or `aerial_gym`)
   **before** `torch`, or Isaac Gym raises `ImportError: PyTorch was imported before
   isaacgym`.
2. **`~/.local` numpy shadowing** — if a `pip install --user` numpy (e.g. from AirSim)
   sits in `~/.local`, it overrides the conda env's numpy and breaks the warp sensor
   path (`np.int` error). The `aerialgym` env fixes this by exporting
   `PYTHONNOUSERSITE=1` on activation (see
   `$CONDA_PREFIX/etc/conda/activate.d/`). If you build the env yourself, set that
   variable, or `pip uninstall --user numpy`.

Quick check that everything imports and the sim builds:

```bash
conda activate aerialgym
python -c "import aerial_gym; from aerial_gym.registry.task_registry import task_registry; import torch; print('ok')"
```

### 3. Watch the environment (no policy)

Opens an Isaac Gym window with 16 LiDAR-equipped drones in the static-obstacle field,
driven by a constant "toward the goal" command — a quick way to see the scene:

```bash
conda activate aerialgym
cd aerial_gym/examples
python navrl_task_example.py            # a viewer window opens
```

### 4. Train the navigation policy (PPO, rl_games)

**The exact command to start a training run.** From the repository root
(`~/workspaces/aerial_gym_ws/src/aerial_gym_simulator`):

```bash
conda activate aerialgym
cd aerial_gym/rl_training/rl_games

# Short wrapper (recommended): headless, warp + PYTHONNOUSERSITE + tee log all built in.
# 256 parallel drones (default build = 150-bar ceiling, 48 active) ≈ 6.1 GB — fits 8 GB VRAM.
./train_navrl.sh                       # == NUM_ENVS=256 ./train_navrl.sh

# ...which wraps this underlying call (ppo_navrl_cnn.yaml = NavRL-style LiDAR CNN, recommended):
python runner.py --file ppo_navrl_cnn.yaml --task navrl_task \
    --num_envs 256 --headless True --use_warp True --train
```

This launches the full **6000-epoch** run (≈ 2 h at ~1.1 s/epoch on an RTX 3070). It
trains the latest task code — the LiDAR CNN plus the `terminate_on_capture` reward, i.e.
the drone flies through the bars to a random goal and the episode ends the moment it
touches the 0.5 m capture sphere. The console prints a per-epoch `NavRL progress` box;
the number to watch is **`captured (success)`** climbing toward 1.0.

`train_navrl.sh` already tees the run to `train_session_logs/train_<date>.log`. To tee the
raw command yourself instead:

```bash
python runner.py --file ppo_navrl_cnn.yaml --task navrl_task \
    --num_envs 256 --headless True --use_warp True --train 2>&1 | tee "train_$(date +%y%m%d_%H%M).log"
```

Two network configs exist, identical except for the feature extractor — train both to
compare: `ppo_navrl_cnn.yaml` (conv stack over the 36×4 scan, preserves which directions
hold obstacles; see `navrl_network.py`) and `ppo_navrl.yaml` (flat-MLP baseline that
flattens the scan). Both run 6000 epochs.

- Each run writes to `runs/ppo_<date>_navrl/`. Checkpoints land in `nn/gen_ppo.pth`
  (latest) plus periodic `last_gen_ppo_ep_<N>_rew_<R>.pth` snapshots.
- **Phase 1 target (M1):** `navrl/captured_rate` ≥ 0.9 while the goal-distance
  curriculum (`curriculum max (m)`) has expanded toward its far-wall ceiling (k_max → 24 m). Check these in
  the console box or in TensorBoard (next section).
- **Status — Phase 1 ✅ met (2026-07-15):** `captured 0.95 / crash 0.05` with **learned yaw control**
  (the drone steers its heading along travel so its 0.28 m body — not its 0.40 m diagonal — leads through
  gaps; action is now 4-D = 3-D velocity + yaw-rate). Beats NavRL's 0.81. Full history in `WORKLOG.md`.
- **Phase 2 (obstacle-density sweep) ✅ swept (2026-07-16, random placement, seed 1):** bar count via
  env vars, e.g. `NAVRL_MAX_BARS=150 NAVRL_NUM_BARS=150 NUM_ENVS=256 ./train_navrl.sh`. Densities are
  quoted over the ~478 m² placement band; the NavRL-density anchor (~22/100 m²) is **110 bars**.
  Results 25/50/75/110/150 bars → captured **0.97 / 0.97 / 0.94 / 0.93 / 0.66** — flat until the
  NavRL anchor, then a cliff toward the placement-jamming limit (~148 bars at 1.5 m spacing).
  Caveat: the 25/50 points were trained on the 4 GB machine before its minibatch was aligned to 4096.
- **Phase 3 (NavRL++-Target) — sensor-to-track path integrated, training pending:** RGB-D appearance
  detection and raw LiDAR range association now produce obstacle/target tracks and covariance. A NavRL++-style
  17-token Transformer then reasons over 2 seconds of static-obstacle, dynamic-obstacle, robot, and
  target history. Ground-truth target state and semantic masks/IDs are forbidden actor inputs. The
  old analytic semantic path remains only as a baseline. The new perception API has no GT/semantic
  arguments and the 17-token Transformer passed a 64-env end-to-end smoke test; the bootstrap RGB-D
  head still needs detector dataset training and held-out validation before a full PPO run. Start the
  new path with `NAVRL_VISION=1 NAVRL_PERCEPTION=1 ./train_navrl.sh` only after that gate. See
  `PERCEPTION_TRANSFORMER_PLAN.md` and `PHASE3_PLAN.md`.

For the first full perception-policy run, do not overlap two blind linear 6000-epoch ramps. Use
`aerial_gym/rl_training/rl_games/train_navrl_perception_staged.sh`: epochs 0–4000 expand goal
distance at 25 bars, then density is promoted 25→110 by measured capture rate through epoch 10000.
During the density stage, 25% of resets replay 5–10 m goals to prevent close-range skill forgetting.
Target motion and sensor perturbations stay off until the static detector/tracker/policy gate passes.

#### Monitoring training — TensorBoard + console metrics

rl_games writes TensorBoard logs automatically to `runs/<run_name>/summaries/`
(no flag needed — every `--train` run logs there). **Open a second terminal** while
training runs and point TensorBoard at the parent `runs/` folder (install once with
`pip install tensorboard`):

```bash
conda activate aerialgym          # tensorboard is installed in this env
cd ~/workspaces/aerial_gym_ws/src/aerial_gym_simulator    # the repository root
tensorboard --logdir aerial_gym/rl_training/rl_games/runs --port 6006
# then open http://localhost:6006 in a browser
```

Pointing `--logdir` at `runs/` (not a single run) makes every run show up as its own set
of curves, so you can compare the CNN and flat-MLP runs side by side. The plots refresh
live as training writes new epochs — no need to restart TensorBoard.

What to watch in TensorBoard (rl_games scalar names):

| Scalar | Meaning | Healthy sign |
|--------|---------|--------------|
| `rewards/step` (or `rewards/iter`) | mean episode reward | rising, then plateaus |
| `episode_lengths/step` | mean episode length | with `terminate_on_capture` (default), *dropping* below the 300-step cap is good — episodes end early when the drone captures the goal (a flat 300 means it never captures) |
| `losses/a_loss` | PPO actor loss | small, no explosion |
| `losses/c_loss` | critic (value) loss | decreasing / stable |
| `losses/entropy` | policy entropy | decreasing slowly (too fast = premature collapse) |
| `info/kl` | approx. KL between updates | small and stable (adaptive lr keeps ~0.008–0.016) |
| `info/lr` | current learning rate | adapts; persistent floor/ceiling = check kl |

**The task-specific navigation metrics** appear in the per-epoch console box AND as
`navrl/*` TensorBoard scalars:

| Console box line | TensorBoard scalar | Meaning / goal |
|------------------|--------------------|----------------|
| `captured (success)` | `navrl/captured_rate` | episodes ended by touching the 0.5 m capture radius — interception success, the primary signal, ↑ toward 1.0 |
| `crash` | `navrl/crash_rate` | ended by collision / height bound, ↓ |
| `timeout (no capture)` | `navrl/timeout_rate` | ran all 300 steps without capturing, ↓ |
| `closest, no crash (m)` | `navrl/closest_nocrash_m` (+ `navrl/closest_min_m`) | mean closest approach over NON-crash episodes (crashes die far and only inflate it), plus the best (min) approach; ↓ toward < 0.5 |
| `curriculum max (m)` | `navrl/curriculum_goal_dist_max_m` | current goal-x upper bound k_max; ramps epoch-proportionally 7 → 24 m over ~3000 epochs; overlapping it, the lower bound k_min rises 5 → 20 m over epochs 2000–5000 (`navrl/curriculum_goal_dist_min_m`) |

The same stats are also summarized to the console every ~2048 finished episodes as
`NavRL progress |` lines — handy with `... --train 2>&1 | tee train.log`.

### 5. Watch a trained policy (viewer)

The native interactive application runs the real Isaac Gym simulation rather than the website's
Three.js illustration. Double-click `launch_navrl_3d.sh`, or launch it once from the IDE. Its setup
window selects a checkpoint, target speed, and drone speed. Density is intentionally not a UI
control: each generalized trial samples it independently. A manual sensor-demo mode needs no
checkpoint. In the 3-D window: `,`/`.` changes target speed, `-`/`=` drone speed, `G` toggles real
LiDAR rays, `N` starts a new trial, and `M` switches policy/manual
control (`I/K/J/L`, `U/O`). The red target wireframe is a human-only debug overlay and is never an
actor input. The launcher inspects checkpoint structure before creating the simulator: new 574-D
Transformer checkpoints use the perception path, while archived 305-D `scan_cnn` checkpoints are
clearly labeled as a legacy semantic baseline and replayed through their exact compatible network.
Policy playback is a fixed 10-trial generalized protocol: every trial regenerates the obstacle
layout/count (25–110 bars), collision-free random drone XY/yaw, and random target XY. The target
uses a nonzero mixed trajectory (0.75 m/s default), and the session reports captured/crash/timeout
counts before exiting.

```bash
conda activate aerialgym
cd aerial_gym/rl_training/rl_games
# Short wrapper (recommended): viewer, 16 envs.
./play_navrl.sh runs/<your_run>/nn/gen_ppo.pth

# ...which wraps this. IMPORTANT: --file must match the network the checkpoint was trained
# with, or the state_dict load fails (CNN weights vs flat-MLP). Use ppo_navrl_cnn.yaml for CNN runs.
python runner.py --file ppo_navrl_cnn.yaml --task navrl_task \
    --num_envs 16 --headless False --play \
    --checkpoint runs/<your_run>/nn/gen_ppo.pth
```

`--headless False` opens the viewer so you can judge the policy by eye. Use a small
`--num_envs` (e.g. 16) for a readable window.

### 6. Evaluate (metrics only, no window)

Same as play but headless; read the `NavRL progress` lines from the output:

```bash
PLAY_GAMES_NUM=8000 python runner.py --file ppo_navrl_cnn.yaml --task navrl_task \
    --num_envs 512 --headless True --play \
    --checkpoint runs/<your_run>/nn/gen_ppo.pth 2>&1 | grep "NavRL progress"
```

### Where things live

| Path | What |
|------|------|
| `aerial_gym/task/navrl_task/` | the Phase 1 navigation task (obs / reward / done) |
| `aerial_gym/config/task_config/navrl_task_config.py` | task settings (goal placement, reward weights, flight altitude, episode length) |
| `aerial_gym/config/env_config/navrl_bars_env.py` | the arena: empty 24×24×3 m + density-controlled bars (`NAVRL_NUM_BARS`), random-rejection placement |
| `aerial_gym/config/asset_config/env_object_config.py` (`bar_asset_params`) | bar asset selection / placement band |
| `resources/models/environment_assets/bars/` | the variable-size bar URDF pool (regenerate: `python tools/generate_bar_assets.py`) |
| `aerial_gym/env_manager/asset_manager.py` | obstacle placement: random-rejection (navrl default) + legacy jittered-grid (`obstacle_placement_mode`) |
| `aerial_gym/config/robot_config/navrl_quad_config.py` | the LiDAR-equipped quad (spawns at 1 m) |
| `aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py` | NavRL-matched 36×4 yaw-only LiDAR |
| `aerial_gym/rl_training/rl_games/ppo_navrl_cnn.yaml` | PPO config, NavRL-style LiDAR CNN (recommended) |
| `aerial_gym/rl_training/rl_games/ppo_navrl.yaml` | PPO config, flat-MLP baseline |
| `aerial_gym/rl_training/rl_games/navrl_network.py` | the LiDAR CNN feature extractor (rl_games custom network) |
| `PERCEPTION_TRANSFORMER_PLAN.md` | authoritative NavRL++-Target detector/tracker, token, PF, and ablation design |
| `PHASE3_PLAN.md` | current execution gates for target detection/tracking/navigation |
| `WORKLOG.md` | chronological log of what changed and why |
| `RESEARCH_PLAN.md` (repo root) | staged research roadmap |

---

The upstream Aerial Gym Simulator documentation follows.

---

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) [![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

# Aerial Gym Simulator

Welcome to the [Aerial Gym Simulator](https://www.github.com/ntnu-arl/aerial_gym_simulator) repository. Please refer to our [documentation](https://ntnu-arl.github.io/aerial_gym_simulator/) for detailed information on how to get started with the simulator, and how to use it for your research.

The Aerial Gym Simulator is a high-fidelity physics-based simulator for training Micro Aerial Vehicle (MAV) platforms such as multirotors to learn to fly and navigate cluttered environments using learning-based methods. The environments are built upon the underlying [NVIDIA Isaac Gym](https://developer.nvidia.com/isaac-gym) simulator. We offer aerial robot models for standard planar quadrotor platforms, as well as fully-actuated platforms and multirotors with arbitrary configurations. These configurations are supported with low-level and high-level geometric controllers that reside on the GPU and provide parallelization for the simultaneous control of thousands of multirotors.

This is the *second release* of the simulator and includes a variety of new features and improvements. Task definition and environment configuration allow for fine-grained customization of all the environment entities without having to deal with large monolithic environment files. A custom rendering framework allows obtaining depth, and segmentation images at high speeds and can be used to simulate custom sensors such as LiDARs with varying properties. The simulator is open-source and is released under the [BSD-3-Clause License](https://opensource.org/licenses/BSD-3-Clause).


Aerial Gym Simulator allows you to train state-based control policies in under a minute:


And train vision-based navigation policies in under an hour:


Equipped with GPU-accelerated and customizable ray-casting based LiDAR and Camera sensors with depth and segmentation capabilities:




## Features

- **Modular and Extendable Design** allowing users to easily create custom environments, robots, sensors, tasks, and controllers, and changing parameters programmatically on-the-go by modifying the [Simulation Components](https://ntnu-arl.github.io/aerial_gym_simulator/4_simulation_components).
- **Rewritten from the Ground-Up** to offer very high control over each of the simulation components and capability to extensively [customize](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization) the simulator to your needs.
- **High-Fidelity Physics Engine** leveraging [NVIDIA Isaac Gym](https://developer.nvidia.com/isaac-gym/download), which provides a high-fidelity physics engine for simulating multirotor platforms, with the possibility of adding support for custom physics engine backends and rendering pipelines.
- **Parallelized Geometric Controllers** that reside on the GPU and provide parallelization for the [simultaneous control of (hundreds of) thousands of multirotor](https://ntnu-arl.github.io/aerial_gym_simulator/3_robots_and_controllers/#controllers) vehicles.
- **Custom Rendering Framework** (based on [NVIDIA Warp](https://nvidia.github.io/warp/)) used to design [custom sensors](https://ntnu-arl.github.io/aerial_gym_simulator/8_sensors_and_rendering/#warp-sensors) and perform parallelized kernel-based operations.
- **Modular and Extendable** allowing users to easily create [custom environments](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization/#custom-environments), [robots](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization/#custom-robots), [sensors](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization/#custom-sensors), [tasks](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization/#custom-tasks), and [controllers](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization/#custom-controllers).
- **RL-based control and navigation policies** of your choice can be added for robot learning tasks. [Includes scripts to get started with training your own robots.](https://ntnu-arl.github.io/aerial_gym_simulator/6_rl_training).


> [!IMPORTANT] 
> Support for [**Isaac Lab**](https://isaac-sim.github.io/IsaacLab/) and [**Isaac Sim**](https://developer.nvidia.com/isaac/sim) is currently under development. We anticipate releasing this feature in the near future.


Please refer to the paper detailing the previous version of our simulator to get insights into the motivation and the design principles involved in creating the Aerial Gym Simulator: [https://arxiv.org/abs/2305.16510](https://arxiv.org/abs/2305.16510) (link will be updated to reflect the newer version soon!).

## Why Aerial Gym Simulator?

The Aerial Gym Simulator is designed to simulate thousands of MAVs simultaneously and comes equipped with both low and high-level controllers that are used on real-world systems. In addition, the new customized ray-casting allows for superfast rendering of the environment for tasks using depth and segmentation from the environment.

The optimized code in this newer version allows training for motor-command policies for robot control in under a minute and vision-based navigation policies in under an hour. Extensive examples are provided to allow users to get started with training their own policies for their custom robots quickly.


## Citing
When referencing the Aerial Gym Simulator in your research, please cite the following paper

```bibtex
@ARTICLE{kulkarni2025aerial,
  author={Kulkarni, Mihir and Rehberg, Welf and Alexis, Kostas},
  journal={IEEE Robotics and Automation Letters}, 
  title={Aerial Gym Simulator: A Framework for Highly Parallelized Simulation of Aerial Robots}, 
  year={2025},
  volume={10},
  number={4},
  pages={4093-4100},
  keywords={Robots;Robot sensing systems;Rendering (computer graphics);Physics;Engines;Navigation;Training;Motors;Planning;Autonomous aerial vehicles;Aerial Systems: perception and autonomy;machine learning for robot control;reinforcement learning},
  doi={10.1109/LRA.2025.3548507}}
```

If you use the reinforcement learning policy provided alongside this simulator for navigation tasks, please cite the following paper:

```bibtex
@INPROCEEDINGS{kulkarni2024@dceRL,
  author={Kulkarni, Mihir and Alexis, Kostas},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={Reinforcement Learning for Collision-free Flight Exploiting Deep Collision Encoding}, 
  year={2024},
  volume={},
  number={},
  pages={15781-15788},
  keywords={Image coding;Navigation;Supervised learning;Noise;Robot sensing systems;Encoding;Odometry},
  doi={10.1109/ICRA57147.2024.10610287}}

```

## Quick Links
For your convenience, here are some quick links to the most important sections of the documentation:

- [Installation](https://ntnu-arl.github.io/aerial_gym_simulator/2_getting_started/#installation)
- [Robots and Controllers](https://ntnu-arl.github.io/aerial_gym_simulator/3_robots_and_controllers)
- [Sensors and Rendering Capabilities](https://ntnu-arl.github.io/aerial_gym_simulator/8_sensors_and_rendering)
- [RL Training](https://ntnu-arl.github.io/aerial_gym_simulator/6_rl_training)
- [Simulation Components](https://ntnu-arl.github.io/aerial_gym_simulator/4_simulation_components)
- [Customization](https://ntnu-arl.github.io/aerial_gym_simulator/5_customization)
- [FAQs and Troubleshooting](https://ntnu-arl.github.io/aerial_gym_simulator/7_FAQ_and_troubleshooting)



## Contact

Mihir Kulkarni  &nbsp;&nbsp;&nbsp; [Email](mailto:mihirk284@gmail.com) &nbsp; [GitHub](https://github.com/mihirk284) &nbsp; [LinkedIn](https://www.linkedin.com/in/mihir-kulkarni-6070b6135/) &nbsp; [X (formerly Twitter)](https://twitter.com/mihirk284)

Welf Rehberg &nbsp;&nbsp;&nbsp;&nbsp; [Email](mailto:welf.rehberg@ntnu.no) &nbsp; [GitHub](https://github.com/Zwoelf12) &nbsp; [LinkedIn](https://www.linkedin.com/in/welfrehberg/)

Theodor J. L. Forgaard &nbsp;&nbsp;&nbsp; [Email](mailto:tjforgaa@stud.ntnu.no) &nbsp; [GitHb](https://github.com/tforgaard) &nbsp; [LinkedIn](https://www.linkedin.com/in/theodor-johannes-line-forgaard-665b5311a/)

Kostas Alexis &nbsp;&nbsp;&nbsp;&nbsp; [Email](mailto:konstantinos.alexis@ntnu.no) &nbsp;  [GitHub](https://github.com/kostas-alexis) &nbsp; 
 [LinkedIn](https://www.linkedin.com/in/kostas-alexis-67713918/) &nbsp; [X (formerly Twitter)](https://twitter.com/arlteam)

This work is done at the [Autonomous Robots Lab](https://www.autonomousrobotslab.com), [Norwegian University of Science and Technology (NTNU)](https://www.ntnu.no). For more information, visit our [Website](https://www.autonomousrobotslab.com/).


## Acknowledgements
This material was supported by RESNAV (AFOSR Award No. FA8655-21-1-7033) and SPEAR (Horizon Europe Grant Agreement No. 101119774).

This repository utilizes some of the code and helper scripts from [https://github.com/leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym) and [IsaacGymEnvs](https://github.com/isaac-sim/IsaacGymEnvs).



## FAQs and Troubleshooting 

Please refer to our [website](https://ntnu-arl.github.io/aerial_gym_simulator/7_FAQ_and_troubleshooting/) or to the [Issues](https://github.com/ntnu-arl/aerial_gym_simulator/issues) section in the GitHub repository for more information.
