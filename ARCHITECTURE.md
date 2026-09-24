# Architecture

How the MOTAR system fits together, and where each part lives. The purpose and research questions
are in [`PROJECT.md`](PROJECT.md); the rules for changing anything are in [`AGENTS.md`](AGENTS.md).

![MOTAR code map: the closed simulation loop, the inputs that define a scenario, and the evidence layer](docs/assets/paper/final/arch-motar-code-map.svg)

## The closed loop

One RL step is 0.1 s: ten physics steps of 0.01 s in Isaac Gym PhysX.

```text
obstacle environment ── bars 70–205 in a 40 × 40 × 3 m arena; moving target
        │
        ▼
sensors ─────────────── forward camera (target) · LiDAR 72 × 4 beams @ 12 m (obstacles)
        │
        ▼
perception ──────────── target detection, tracking, 2 s history          [+ optional P9 measured-error injection]
        │                                                                  [+ optional D8 mesh rendering treatment]
        ▼
temporal state ──────── 17 tokens: [CLS], static scan, 5 obstacle, 5 robot, 5 target-track steps
        │
        ▼
policy observation ──── actor 898-D · critic 906-D (critic adds 8 ground-truth values)
        │
        ▼
frozen PPO policy ───── Transformer actor-critic (rl_games) → vx, vy, vz, yaw rate
        │
        ▼
safety filter ───────── LiDAR speed cap (riskcap for policy F); scales speed, keeps direction
        │
        ▼
low-level control ───── Lee velocity controller → quadrotor dynamics → back to the environment
```

**Observation contract.** The actor never receives ground-truth target information: no semantic id or
mask, bearing, range, `target_position` or ground-truth visibility. Ground truth is used only for
detector supervision, rewards, termination, the critic and evaluation metrics. Observation widths
belong to lineages (156 → 305 → 1265 → 898) that must never be mixed; a checkpoint only loads into
its own width.

**Task contract.** A capture (close approach) is a pursuer–target distance below `success_radius =
0.5 m` and ends the episode. A crash is a bar contact, leaving the arena, or hitting the floor or
ceiling. A timeout is 600 steps (60 s) with neither. These values are generated from the research
source into [`docs/status/research_task_contract.json`](docs/status/research_task_contract.json),
which the browser preview also reads.

**Frozen policies.** **Policy F** is checkpoint ep25000 + riskcap (SHA-256 `f70221393660…`), trained
on the historical target lineage (arm H); the perception-cost, safety-geometry and target-motion
results use it. **Policy R** is a different frozen checkpoint, ref5in D1 ep1900 (`197ea269…`), used
only by D8b. No result pools the two. The ep25000 training contract differs from HEAD defaults in 14 settings; it is
reproducible only through its recorded launcher environment
([audit](docs/audits/frozen_policy_training_contract_2026-09-17.md)).

## Components and source files

| Component | Role | Source |
| --- | --- | --- |
| Obstacle environment | Arena, bar placement, density curriculum | `aerial_gym/config/env_config/navrl_bars_env.py`, `aerial_gym/task/navrl_task/navrl_curriculum.py` |
| Target-motion generator | H legacy steering, E0 static, E1 constant velocity, E2 obstacle-aware local steering | `aerial_gym/task/navrl_task/target_motion.py` |
| Physical target (gated) | 6-DoF target with optional global route; its routing gates FAILed | `aerial_gym/task/navrl_task/physical_target.py`, `target_route_planner.py` |
| Camera and detector | Forward camera; target pixels occlusion-tested; built-in or learned detector | `aerial_gym/task/navrl_task/navrl_detector.py` |
| LiDAR | 72 × 4 beams at 12 m for policy F (source defaults are 36 × 4 at 4 m) | `aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py` |
| Perception and temporal state | Fusion, tracking, uncertainty, 2 s history, token assembly | `aerial_gym/task/navrl_task/navrl_perception.py` |
| Measured-error injection (P9) | Replays the P8 error model in simulation (opt-in) | `aerial_gym/task/navrl_task/navrl_empirical_error.py` |
| Rendering treatment (D8) | Mesh-derived target mask and depth (opt-in) | `aerial_gym/task/navrl_task/navrl_dynamic_mesh_treatment.py` |
| Task | Observation assembly, reward, termination, metrics | `aerial_gym/task/navrl_task/navrl_task.py`, `aerial_gym/config/task_config/navrl_task_config.py` |
| Policy | Transformer actor-critic, PPO runner | `aerial_gym/rl_training/rl_games/navrl_transformer_network.py`, `runner.py` |
| Safety filter | Speed governor modes: riskcap, stopcap, arc-clearance (`dwa_arc`) and others | `aerial_gym/task/navrl_task/speed_governor.py` |
| Low-level control | Lee velocity controller with the NavRL yaw-rate limit | `aerial_gym/control/controllers/velocity_control.py`, `aerial_gym/config/controller_config/lee_controller_config_navrl.py` |
| Vehicle | Quadrotor robot configurations; ref5in configs are provenance-frozen | `aerial_gym/config/robot_config/navrl_*_quad_config.py` |
| Real-image perception (offline) | Detector, temporal selector, streaming API. Not connected to the policy | `tools/eval_perception_temporal.py`, `docs/specs/perception_streaming_v1.md` |
| Independent renderer | Static graphics prototype with its own measurement contract | `tools/renderer_validation/` |
| Browser preview | WebGL arena explaining the task; ground-truth, not PPO | `docs/status/arena*.js`, `viewer.js` |

Two target planners are easy to confuse. The research target E2 is a **local** receding-horizon
executor with **no A\***. The browser preview uses a global route. The full per-lineage answer is in
[`docs/target_motion_algorithm_2026-09-17.md`](docs/target_motion_algorithm_2026-09-17.md).

## Evidence layer

```text
preregistration (docs/) ─► fail-closed launcher ─► receipt + raw counts (results/<run>/)
                                                         │
          tests/ ◄── README · site · figures ◄── canonical registries (docs/*.json)
```

- **Launchers** check the arena, sensors, action contract, robot hash, detector hash, seeds and a
  clean source tree before every cell. Examples:
  `aerial_gym/rl_training/rl_games/eval_navrl_v2_density_sweep.sh` and `tools/run_navrl_filter_grid.py`.
- **Receipts** record source commit, runtime fingerprint and file hashes. `results/MANIFEST.json`
  indexes every result directory without interpreting it.
- **Registries** are the machine-readable claims:
  - `docs/quantitative_positioning_registry.json`: headline numbers.
  - `docs/research_status_registry.json`: component lifecycle.
  - `docs/status_manifest.json`: Track D states.
- **Published numbers** in the README, site and figures are generated from, or tested against, those
  registries. `tools/build_paper_figures.py` builds the figures;
  `tests/test_canonical_docs.py` and the site tests bind the text.

## Repository structure

| Path | Contains | State | May be modified? |
| --- | --- | --- | --- |
| `aerial_gym/` | Simulator (upstream Aerial Gym) and the MOTAR task: environment, sensors, perception, policy runner, safety filter, controller | Active research code; several files are provenance-bound | Only for a preregistered change, never while a run is active. Frozen files (e.g. `navrl_ref5in_quad_config.py`) never |
| `aerial_gym/rl_training/rl_games/` | PPO runner, networks, training and evaluation launchers (`*.sh`) | Launchers are historical records of what ran | Do not edit a launcher that produced a result |
| `resources/` | Robot and environment assets (URDF, meshes) | Active; launchers hash the robot assets | No, unless a preregistered asset change |
| `configs/` | Perception model configurations (detector, temporal models) | Provenance for the P-track results | No |
| `artifacts/` | Detector weights with receipts | Provenance | No |
| `tools/` | Public tools (doctor, renderer, documentation and figure builders) and experiment launchers and analysers | Public tools active; experiment scripts historical | Public tools yes; experiment scripts only by adding, never by rewriting |
| `tests/` | Unit, contract, evidence and documentation guards (Python and Node) | Active | Yes, but never weaken a guard to make a change pass |
| `results/` | One folder per experiment: receipts, raw counts, summaries, READMEs, VOID runs | **Immutable evidence** | Never. Add new folders only |
| `docs/` | Canonical docs (`EVIDENCE`, `REPRODUCIBILITY`, `OPERATIONS`, `HISTORY`), preregistrations, dated result records, registries | Canonical docs active; preregistrations immutable; dated records historical | Canonical docs yes; preregistrations and dated records no |
| `docs/status/` | Public site: `index.html` (project page), `evidence.html` (full evidence), `archive-2026-09-13.html` (hash-pinned), browser preview scripts | Active; the preview is frozen (`GT_BROWSER_EPISODE_V1`) | Pages yes; preview semantics no |
| `docs/assets/paper/final/` | Current figure family, generated | Active | Only through `tools/build_paper_figures.py` |
| `docs/assets/` (other) | Dated figure packages | Historical, hash-pinned | No |
| `docs/archive/` | Superseded plans, notes and snapshots | Historical | No, except the archive index |
| `.github/workflows/` | CPU public validation | Active | Yes |

Root files: `README.md`, `PROJECT.md`, `ARCHITECTURE.md`, `AGENTS.md` and `DESIGN_SYSTEM.md` are
canonical. `WORKLOG.md` (chronology), `VERIFICATION.md` (verification ledger), `OPERATIONS.md`
(historical handbook), `RESEARCH_PLAN.md` (original charter) and `CRASH_TUNING_LOG.md` (frozen July
notes) are kept in place because tests, receipts or source comments cite them.
