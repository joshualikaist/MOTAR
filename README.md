# MOTAR

**Moving-Object Tracking And RL for UAV navigation in random obstacle fields**

> A quadrotor chases a **moving target** through a **cluttered field of obstacles** using
> only its **onboard sensors** — LiDAR and a camera. It is never told where the target is.

MOTAR is a research fork of the [Aerial Gym Simulator](https://github.com/ntnu-arl/aerial_gym_simulator).
The obstacle field and LiDAR follow [NavRL](https://github.com/Zhefan-Xu/NavRL),
reimplemented on Aerial Gym and trained with PPO (rl_games). A temporal camera–LiDAR model
infers the target's state under occlusion, and the **deployed policy never receives
ground-truth target position** — only what the sensors can see.

Research lives on the `research/navrl-env` branch; `main` tracks upstream Aerial Gym.

## Why it's a research problem

- **Sensor-only.** The actor sees LiDAR + a learned camera detector — never a ground-truth
  target position, bearing, or mask. Those are allowed only for the critic and the reward
  during training (an *asymmetric* actor–critic).
- **Scaling map.** How far does interception hold as the field gets **denser** and the
  target moves **faster**? MOTAR sweeps obstacle density × target speed to find where it breaks.
- **Occlusion.** The target hides behind obstacles, so the policy tracks it through gaps in
  what it can see — using a short memory of past detections, not a single frame.

## Quick start

Full prerequisites (Ubuntu, NVIDIA GPU ≥ 8 GB, Isaac Gym Preview 4, conda), the manual
install, and the two environment gotchas are in **[OPERATIONS.md](OPERATIONS.md)**.
The short path:

```bash
# 1. Install — clones deps, builds the `aerialgym` conda env, runs a smoke test
mkdir -p ~/workspaces/aerial_gym_ws/src && cd ~/workspaces/aerial_gym_ws/src
git clone -b research/navrl-env git@github.com:joshualikaist/MOTAR.git aerial_gym_simulator
cd aerial_gym_simulator && ./bootstrap_second_machine.sh
conda activate aerialgym

# 2. See the arena — a viewer with drones flying toward the goal (no policy yet)
cd aerial_gym/examples && python navrl_task_example.py

# 3. Train the navigation policy (PPO) — headless; console output is also saved
cd ../rl_training/rl_games && ./train_navrl.sh
#    watch the console box: `captured (success)` should climb toward 1.0
#    from another terminal: ./watch_navrl_training.sh

# 4. Watch / evaluate a trained checkpoint
./play_navrl.sh runs/<your_run>/nn/gen_ppo.pth              # viewer, 16 drones
./launch_navrl_3d.sh runs/<your_run>/nn/last_gen_ppo_ep_XXXX.pth   # live 3-D viewer
```

Env counts, the 4 GB-VRAM preset, and the training metrics / TensorBoard scalars to watch
are documented in **[OPERATIONS.md](OPERATIONS.md)** and **[WORKLOG.md](WORKLOG.md)**.

> **Current method** — a learned RGB-D + LiDAR perception front-end feeding a NavRL++-style
> Transformer policy — is specified in **[RESEARCH_PLAN.md](RESEARCH_PLAN.md)**.
> Run it with `NAVRL_VISION=1 NAVRL_PERCEPTION=1 ./train_navrl.sh` (staged curriculum:
> `./train_navrl_perception_staged.sh`), after the detector-validation gate in that plan.
>
> **Current controlled result (closed 2026-08-05)** — the frozen ep24000 policy exposed an early
> high-speed bar-contact bottleneck, not an episode-horizon shortage. Complete-stop clearance/TTC
> governors cut crash but raised timeout to 16.59–23.57%, so they were rejected. A sensor-only
> minimum-intervention `riskcap` instead limits horizontal speed to 2.0 m/s only inside 3 m command-
> corridor clearance, releases it by 5 m, and never forces a stop. On unseen seed45 it moved the
> frozen policy from 70.03/27.87/2.10% to 78.20/17.80/4.00% capture/crash/timeout. Exactly 1,000
> adaptation epochs then reached **81.94/15.67/2.39%**, adding +3.75 pp capture over source+riskcap.
> The trained winner also improved capture and crash at all seed46 fixed speeds 0.3/0.9/1.5 m/s.
> Its checkpoint SHA-256 is `f70221393660…`; full confidence intervals and artifact contracts are in
> [results/navrl_v2_riskcap_postadapt/summary.md](results/navrl_v2_riskcap_postadapt/summary.md).
> No training or evaluation is active. Do not extend fixed-density PPO or retune riskcap post hoc;
> the next gated stage is learned-detector/perception robustness.

## Status

Sensor-only interception works end to end: the actor sees **only** a 72x4 LiDAR at 12 m plus a
forward sensor-derived target track (898-D structured observation, 17-token Transformer,
asymmetric 906-D critic with ground truth confined to training). The completed density recovery
used the **analytic detector mode** to isolate navigation/control. It is not evidence that
the final learned RGB-D detector gate has been passed; that perception stage remains a separate
research requirement.

**Current v2 limit result** (40×40×3 m, moving target 0.3–1.5 m/s, seed 42, deterministic deployment):

| bars | density/100 m² | episodes | capture | crash | timeout |
|---:|---:|---:|---:|---:|---:|
| 130 | 8.12 | 2,049 | 84.77% | 12.74% | 2.49% |
| 160 | 10.00 | 2,050 | 79.66% | 16.88% | 3.46% |
| 190 | 11.88 | 2,049 | 73.99% | 22.65% | 3.37% |
| **205** | **12.81** | **2,050** | **72.44%** | **25.07%** | **2.49%** |
| 220 | 13.75 | 2,050 | 68.49% | 29.76% | 1.76% |

At 205 bars the same checkpoint scores 67.35% under stochastic action sampling, a significant
5.09 pp gap from deterministic deployment. Ten complete curriculum holds, 20.1M samples at 205,
healthy PPO diagnostics, and 99.83% random-pair geometric connectivity show that more unchanged
epochs are not justified. The remaining failures are mostly bar contacts accumulated on long and
fast trajectories.

The frozen causal checks separate reproducibility from symmetry. Seed 43 scores **72.77%** at 205
bars versus seed 42's 72.44% (+0.33 pp; replication PASS). Original and mirror-conjugate policies
score 70.97% and 70.17% over 4,096 episodes/arm, and initial negative/positive-y target bearings
score 71.17%/70.97%, so no material outcome-side asymmetry was detected. The controller itself is
not reflection-equivariant: exact reflected-observation pairs have lateral action MAE **1.235** and
**73.08%** sign mismatch. This is a learned one-direction route preference, currently outcome-neutral
in the symmetric arena but a robustness concern. Fixed-speed evaluation shows a material
**-5.91 pp** capture drop from 0.3 to 1.5 m/s, accompanied by **+7.86 pp** absolute bar contact.
The matched forgetting evaluation found improvement, not degradation: ep24000 beats ep19100 by
**+4.65 pp** on U[0.3,1.5] and **+3.25 pp** at fixed 1.5 m/s. Subsequent risk-ordering and speed-
governor experiments are now complete; the non-stopping `riskcap` result above is the selected
navigation/control candidate. Full diagnostic outputs:
[causal 1–3](results/navrl_v2_ep24000_causal_1to3/summary.md),
[fixed speed](results/navrl_v2_ep24000_fixed_speed/summary.md), and
[riskcap final](results/navrl_v2_riskcap_postadapt/summary.md).

A left/right chirality defect in the observation pipeline was found and fixed on 2026-07-29 --
the perception bin-to-bearing table was the mirror image of the sensor's ray generator, so every
obstacle token had been emitted on the wrong side of the drone. Physically adjudicated: token
association with real bars went from 13.9% to 94.8%. Held-out density curve improved by **+11.1 pp
on average**, and the density curriculum reached **85 bars (17.8 bars/100 m^2)** where it had
previously stalled at 65.

The obstacle-token bottleneck (8 slots representing only ~3 unique bars) was traced to
suppression-window duplication and addressed by a `cluster_sector` selector, which raised unique
bars per step from 3.0 to 4.6.

**Headline result — density x target-speed map** (held-out, 2049 episodes/cell, deterministic,
`cluster_sector` checkpoint, sensor-only):

| bars | density/100 m^2 | capture @0.0 m/s | @0.5 | @1.0 | @1.5 |
|---|---|---|---|---|---|
| 25 | 5.2 | 0.962 | 0.970 | 0.965 | 0.949 |
| 50 | 10.5 | 0.918 | 0.930 | 0.922 | 0.904 |
| 65 | 13.6 | 0.881 | 0.879 | 0.861 | 0.837 |
| **85** | **17.8** | 0.736 | 0.753 | 0.718 | 0.671 |
| 110 | 23.0 | 0.497 | 0.506 | 0.484 | 0.437 |
| 130 | 27.2 | 0.318 | 0.296 | 0.281 | 0.259 |
| 150 | 31.4 | 0.194 | 0.192 | 0.190 | 0.159 |

85 bars is the trained maximum; rows below it measure generalization, not a ceiling of the method.
Across the full grid, raising density 5.2 -> 31.4 bars/100 m^2 costs **78 pp** of capture, while
raising target speed 0 -> 1.5 m/s costs only **4.2 pp** -- the pursuer (v_max 2.5 m/s) is fast
enough that target speed is not a binding difficulty axis in this regime; obstacle density is.
This is a historical pre-heading-continuity figure (full CSV:
`results/density_speed_map_cluster_sector.csv`, interactive version: [status dashboard, "Map"
tab](docs/status/)). It remains useful as a frozen 85-bar baseline, but must not be presented as the
current v2 40×40 result. The completed v2 result is the separate table above. Full detail is in
**[WORKLOG.md](WORKLOG.md)** (newest entry last).

## Repo map

| Path | What |
|------|------|
| `aerial_gym/task/navrl_task/` | the interception task — observations, reward, termination |
| `aerial_gym/task/navrl_task/navrl_perception.py` | learned camera–LiDAR detector + tracker (current method) |
| `aerial_gym/config/env_config/navrl_bars_env.py` | the arena — current v2 40×40×3 m full-width `navrl_band` field; legacy v1 was 24×24 m |
| `aerial_gym/config/…/navrl_lidar_config.py`, `navrl_quad_config.py` | the LiDAR-equipped quadrotor |
| `aerial_gym/rl_training/rl_games/` | PPO configs, custom networks, and the `*_navrl*.sh` run wrappers |
| `tools/`, `tests/` | bar-asset generation, geometry audits, smoke tests |

## Docs

| File | For |
|------|-----|
| **[WORKLOG.md](WORKLOG.md)** | what changed and why — **start here** |
| [RESEARCH_PLAN.md](RESEARCH_PLAN.md) | research questions, method spec, staged plan P0-P7 |
| [OPERATIONS.md](OPERATIONS.md) | install, GPU tiers/VRAM, second machine, transferring results |

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
