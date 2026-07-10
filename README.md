# MOTAR

**MOTAR: Moving Object Tracking and Reinforcement-Learning-Based Approach for UAV Navigation in Random Obstacle Fields**

This repository is a research fork of the [Aerial Gym Simulator](https://github.com/ntnu-arl/aerial_gym_simulator).
It studies reinforcement-learning-based UAV navigation and moving-target approach in random
obstacle environments, analyzing performance under varying obstacle density and target speed,
together with dynamic object detection. The experimental environment follows the setup of
[NavRL](https://github.com/Zhefan-Xu/NavRL), reimplemented on top of Aerial Gym.
The staged research roadmap is maintained in `RESEARCH_PLAN.md` in the local workspace root.

Research work lives on the `research/navrl-env` branch; `main` tracks upstream Aerial Gym.

---

## Getting Started (MOTAR)

This section is a self-contained guide: from a fresh machine to watching a drone fly
in the simulator. It targets the Phase 1 task `navrl_task` (static obstacles + a
stationary goal, LiDAR-based navigation).

**The Phase 1 arena (`navrl_bars_env`)** is a controlled smoke-test environment: an
otherwise-empty 10×10×3 m space with **16 static vertical bars** (random footprint
0.4–0.8 m per side, height fixed 2 m). Bars are placed on a per-episode jittered 4×4
grid that guarantees ≥ 1.3 m center-to-center distance (≥ 0.5 m clear gap even between
the two largest bars). The drone flies **in 2D at a fixed 1 m altitude** (vertical
velocity command is zeroed) and must weave through the bars to a goal sampled 2.5–5 m
away (goal-distance curriculum expands this as the reach rate improves). See
`navrl_bars_env_layout.png` in the workspace root for a top-down picture.

### 1. Prerequisites

| Component | Tested version |
|-----------|----------------|
| OS | Ubuntu 20.04 |
| GPU | NVIDIA, ≥ 8 GB VRAM (developed on an RTX 3070 8 GB) + recent driver (CUDA 12.x) |
| [Isaac Gym Preview 4](https://developer.nvidia.com/isaac-gym) | 1.0rc4, unpacked to `~/isaacgym` |
| Conda | Miniconda / Anaconda |

### 2. One-time environment setup

```bash
# (a) Create and activate a Python 3.8 conda env
conda create -n aerialgym python=3.8 -y
conda activate aerialgym

# (b) Install Isaac Gym Preview 4 (download from NVIDIA, then:)
cd ~/isaacgym/python && pip install -e .

# (c) Install this repo (editable) + its deps
cd <path>/aerial_gym_simulator
pip install -e .
pip install rl-games==1.6.5 warp-lang==1.0.0

# (d) Install the urdfpy fork used by the simulator
pip install -e <path>/urdfpy
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

```bash
conda activate aerialgym
cd aerial_gym/rl_training/rl_games

# headless training, 512 parallel drones (fits 8 GB VRAM; drop to 256 if you OOM)
python runner.py --file ppo_navrl.yaml --task navrl_task \
    --num_envs 512 --headless True --train
```

- Checkpoints: `runs/ppo_<date>_navrl/nn/gen_ppo.pth` (saved periodically), plus
  `last_gen_ppo_ep_<N>_rew_<R>.pth` snapshots.

#### Monitoring training — TensorBoard + console metrics

rl_games writes TensorBoard logs automatically to `runs/<run_name>/summaries/`
(no flag needed). To view them (install once with `pip install tensorboard`):

```bash
tensorboard --logdir aerial_gym/rl_training/rl_games/runs
# then open http://localhost:6006
```

What to watch in TensorBoard (rl_games scalar names):

| Scalar | Meaning | Healthy sign |
|--------|---------|--------------|
| `rewards/step` (or `rewards/iter`) | mean episode reward | rising, then plateaus |
| `episode_lengths/step` | mean episode length | rising toward the 150-step cap (fewer crashes) |
| `losses/a_loss` | PPO actor loss | small, no explosion |
| `losses/c_loss` | critic (value) loss | decreasing / stable |
| `losses/entropy` | policy entropy | decreasing slowly (too fast = premature collapse) |
| `info/kl` | approx. KL between updates | small and stable (adaptive lr keeps ~0.008–0.016) |
| `info/lr` | current learning rate | adapts; persistent floor/ceiling = check kl |

**The task-specific navigation metrics are printed to the console** (not TensorBoard)
every ~2048 finished episodes, as `NavRL progress |` lines:

| Metric | Meaning | Goal |
|--------|---------|------|
| `ever_reached` | fraction of episodes that touched the 1 m success radius at least once | ↑ toward 1.0 — the primary success signal; the curriculum expands the goal distance when this exceeds 0.6 |
| `success@timeout` | still within 1 m of the goal when the episode times out | ↑ |
| `crash` | episodes ended by collision / height bound | ↓ |
| `timeout` | episodes that ran the full 150 steps without reaching | ↓ as reaching improves |
| `mean_closest_approach` | mean over episodes of the closest distance to the goal | ↓ toward < 1 m |

Tip: capture them to a file while training,
`... --train 2>&1 | tee train.log`, then `grep "NavRL progress" train.log`.

### 5. Watch a trained policy (viewer)

```bash
conda activate aerialgym
cd aerial_gym/rl_training/rl_games
python runner.py --file ppo_navrl.yaml --task navrl_task \
    --num_envs 16 --headless False --play \
    --checkpoint runs/<your_run>/nn/gen_ppo.pth
```

`--headless False` opens the viewer so you can judge the policy by eye. Use a small
`--num_envs` (e.g. 16) for a readable window.

### 6. Evaluate (metrics only, no window)

Same as play but headless; read the `NavRL progress` lines from the output:

```bash
PLAY_GAMES_NUM=8000 python runner.py --file ppo_navrl.yaml --task navrl_task \
    --num_envs 512 --headless True --play \
    --checkpoint runs/<your_run>/nn/gen_ppo.pth 2>&1 | grep "NavRL progress"
```

### Where things live

| Path | What |
|------|------|
| `aerial_gym/task/navrl_task/` | the Phase 1 navigation task (obs / reward / done) |
| `aerial_gym/config/task_config/navrl_task_config.py` | task settings (goal placement, reward weights, flight altitude, episode length) |
| `aerial_gym/config/env_config/navrl_bars_env.py` | the Phase 1 arena: empty 10×10×3 m + 16 bars, grid spacing guarantee |
| `aerial_gym/config/asset_config/env_object_config.py` (`bar_asset_params`) | bar asset selection / placement band |
| `resources/models/environment_assets/bars/` | the variable-size bar URDF pool (regenerate: `python tools/generate_bar_assets.py`) |
| `aerial_gym/env_manager/asset_manager.py` | jittered-grid obstacle placement (`min_obstacle_xy_spacing`) |
| `aerial_gym/config/robot_config/navrl_quad_config.py` | the LiDAR-equipped quad (spawns at 1 m) |
| `aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py` | NavRL-matched 36×4 yaw-only LiDAR |
| `aerial_gym/rl_training/rl_games/ppo_navrl.yaml` | PPO hyperparameters |
| `WORKLOG.md` | chronological log of what changed and why |
| `RESEARCH_PLAN.md` (workspace root) | staged research roadmap |

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
