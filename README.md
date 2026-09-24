# MOTAR

**MOTAR: Moving Object Tracking And Rendezvous**<br>
*Reinforcement Learning for UAV Tracking and Close Approach in Random Obstacle Fields*

A simulation study of how measured perception uncertainty, safety geometry, target motion and the
observation contract change the behaviour of frozen UAV policies.

![MOTAR overview: the simulated closed loop (environment, perception, temporal state, frozen policy, safety filter and control) above the measured evidence chain](docs/assets/paper/final/fig1-motar-overview.svg)

**Main finding:** target motion was not a monotonic difficulty ladder. Against the target motion the
policy was trained on, a static target scored **−3.58 pp** capture and an obstacle-aware target
**+1.33 pp**.
*Simulation only; no real-flight or state-of-the-art claim.*

## Why MOTAR?

Published UAV work usually isolates one piece of the problem — navigation, tracking, pursuit,
perception or safety — and reports a number inside it. MOTAR follows one simulated task end to end:
it measures perception error on real footage, injects it into a frozen policy, varies the safety
filter's geometry, the target's motion and the way the target is rendered, and records what each
change costs. Each headline comparison changes one factor; its preregistration, record and recorded
deviations are linked from [`docs/EVIDENCE.md`](docs/EVIDENCE.md). Negative, inconclusive and
withdrawn results are kept beside the positive ones.

## Key Results

| Question | Result | Status and limit |
| --- | --- | --- |
| Target motion vs the training target | static **−3.58 pp** · obstacle-aware **+1.33 pp** capture | Not a monotonic ladder. Policy F, 3 evaluation seeds; 95.42 % of static-target timeouts never acquired the target (association only) |
| Measured perception error → policy | **−4.57 pp** capture, 95 % CI [−6.32, −2.82] | Policy F; episode-level interval from two evaluation seeds at one density; one injector |
| Readaptation under that error | **+0.73 pp**, 95 % CI [−1.04, +2.50] | **INCONCLUSIVE**: no net benefit established; 3 training seeds |
| Safety-filter geometry | **−1.4903 pp** crash; seed-level 95 % CI [−2.42, −0.57] | Policy F, 3 evaluation seeds; lower in 15/15 seed × density cells, which are not independent replicates. Configured contrast; not a safety guarantee |
| Observation contract | **−48.967 pp** capture | **MATERIAL_LOSS**; causality **NOT_TESTED**; policy R (a different checkpoint), 3 evaluation seeds |
| Perception range error on real footage | **6.2 %** median of 9 block medians | One flight (9 blocks of 15 s; 3,107 frames, not independent samples); apparent-size proxy |

pp = percentage points. Two frozen checkpoints carry these results: policy F (ep25000 + riskcap) and,
for the observation contract only, policy R (ref5in D1 ep1900). No external result is compared
numerically: Class A matched comparisons = 0.

## System

![MOTAR code map: scenario inputs, the closed loop from obstacle environment to vehicle dynamics, and the evidence layer](docs/assets/paper/final/arch-motar-code-map.svg)

A forward camera and a LiDAR feed perception, which keeps a short track history. A Transformer PPO
policy, frozen for every evaluation, turns 17 observation tokens into a velocity command. A LiDAR
speed filter caps that command before a Lee velocity controller flies the quadrotor. Ground-truth
target state never enters the policy's observation. The scenario inputs (target motion, rendering
contract, measured-error injection) are the factors the experiments vary. Details and source paths:
[`ARCHITECTURE.md`](ARCHITECTURE.md).

## Repository Structure

| Path | Contents |
| --- | --- |
| `aerial_gym/` | Simulator and the MOTAR task: environment, sensors, perception, policy, safety filter, control |
| `tools/` | Public tools (environment doctor, renderer, figure and document builders) and experiment scripts |
| `tests/` | Contract, evidence and documentation guards |
| `results/` | One immutable folder per experiment: receipts, raw counts, summaries, VOID runs |
| `docs/` | Evidence, reproducibility, operations, history, preregistrations; the site in `docs/status/` |

The historical research code in `aerial_gym/` and the experiment launchers are provenance, not
recommended entry points. Start from the quick start below.

## Quick Start

The supported public path is CPU-only: Linux x86-64, Python 3.8, Git, and a new virtual environment.
It needs no GPU and no Isaac Gym, and runs no policy.

```bash
git clone https://github.com/joshualikaist/MOTAR-public.git MOTAR && cd MOTAR
# create and activate the isolated CPU environment: docs/renderer_cpu_quickstart_2026-09-12.md
export PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=''
python -B tools/motar_doctor.py --profile renderer-cpu
python -B tools/run_renderer_validation.py --device cpu --seed 0 --num-scenes 1 \
  --width 160 --height 120 --frames 2 --output /tmp/motar-smoke-new
python -B -m unittest discover -s tests -p 'test_public*.py'
```

The smoke test renders generic boxes and debug buffers. It is not an experiment verdict. The
recorded results are verified from their receipts rather than re-run, as described in
[`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md). The historical simulator environment is
separate: it needs a GPU and Isaac Gym Preview 4, which NVIDIA distributes and which cannot be
redistributed here.

## Research Evidence

Every result, its interval, its record and its limit are on one page:
[**`docs/EVIDENCE.md`**](docs/EVIDENCE.md). The public site has a project page and a full evidence
page ([`docs/status/`](docs/status/index.html)). Figures by track as of 2026-09-12 are in the
[results overview](docs/results_overview_2026-09-12.md).

## Documentation

- [`PROJECT.md`](PROJECT.md): purpose, questions, scope and phase.
- [`ARCHITECTURE.md`](ARCHITECTURE.md): how the system connects and which directories may change.
- [`docs/EVIDENCE.md`](docs/EVIDENCE.md): what is established, and what is not.
- [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md): environments, data availability, verification.
- [`docs/OPERATIONS.md`](docs/OPERATIONS.md): everyday commands and run rules.
- [`docs/HISTORY.md`](docs/HISTORY.md): milestones and the negative record.
- [`AGENTS.md`](AGENTS.md) and [`DESIGN_SYSTEM.md`](DESIGN_SYSTEM.md): rules for contributors, the site
  and the figures.

## Limitations

> MOTAR is **simulation-only** research on moving-target rendezvous (tracking and close approach in
> a simulated obstacle field). It makes **no real-flight validation claim**, no sim-to-real claim and
> no state-of-the-art claim, and it provides no deployment instructions. The historical experiments
> were framed as interception; their metric names (`capture`, `crash`) keep their original meaning,
> and "rendezvous" does not reinterpret them. Each effect holds only inside its recorded contract:
> checkpoint, arena, densities and seeds. Readaptation is INCONCLUSIVE, the D8b cause is NOT_TESTED,
> and persistent target identity and metric range are BLOCKED by missing ground truth.

## Citation

Cite the software with [`CITATION.cff`](CITATION.cff). No MOTAR paper has been published, and no
author identifier is invented. Cite the upstream software and datasets used by an experiment
separately.

## License

Original code is offered under the repository's [BSD-3-Clause notice](LICENSE), subject to the
retained upstream notices. This does not cover every data file or dependency; see the
[third-party inventory](THIRD_PARTY_LICENSES.md). ETH ds5 images are not redistributed and must be
obtained separately under CC BY-NC-SA 4.0
([external data](docs/external_data/ETH_DS5.md)).

MOTAR derives from [Aerial Gym Simulator](https://github.com/ntnu-arl/aerial_gym_simulator) and
acknowledges NavRL, rl_games, NVIDIA Warp, urdfpy, trimesh and the cited dataset authors.
Acknowledgement does not imply endorsement.
