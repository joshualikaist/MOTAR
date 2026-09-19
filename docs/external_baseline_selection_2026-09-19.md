# External matched-baseline selection — 2026-09-19

Class A matched external comparisons are currently **0**. This document decides which Class B
candidate could become the first one, and what that would cost.

**Ranking criterion, stated before the audit:** highest benchmark-contract compatibility at the
minimum algorithmic rewriting. Candidates are **not** ranked by published performance, and not by
how favourable a comparison would look. A candidate that would be easy to beat is not thereby a
good baseline.

**Nothing was executed.** No external system was installed, built, trained or run. No GPU workload
was started. Every row is a documentation-level judgement from the primary sources recorded in
[the quantitative ledger](literature_quantitative_ledger_2026-09-18.json).

## MOTAR's side of the contract

| | |
|---|---|
| Simulator | Isaac Gym (Preview 4) |
| Arena | 40 × 40 × 3 m, randomized vertical bars, densities 70/115/160/205 |
| Agents | **one** pursuer, one target |
| Observation | sensor-only: camera-derived target evidence + 72 × 4 LiDAR @ 12 m; 898-dim actor |
| Action | velocity + yaw-rate command, max 2.5 m/s, yaw rate 3.0 rad/s |
| Target | pursuer-independent; TM-E0/E1/E2 motion contracts |
| Success | `success_radius = 0.5 m`, swept; capture ends the episode |
| Timeout | 600 steps × 0.1 s = 60 s |

## Audit

| Candidate | Public code | License | Checkpoint | Single-UAV | Moving target | Random obstacles | Sensor compat | Action compat | Target traj portable | Termination portable | Rewrite needed | **Class** |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| [NavRL](https://github.com/Zhefan-Xu/NavRL) | yes, official | `NEEDS_CONFIRMATION` | `NOT_VERIFIED` | yes | **no** — goal navigation | yes (350 static + dynamic) | close: RGB-D + state | **close**: velocity command | goal stream only | success = reach goal, portable | **retraining required** for a moving goal | **PARTIALLY_MATCHABLE** |
| [OPEN](https://github.com/thu-uav/Multi-UAV-pursuit-evasion) | yes, official | **MIT** | `NOT_VERIFIED` | **no** — 3 pursuers | yes, scripted evader | yes, 4–5 cylinders | **none** — privileged relative state + LOS bit, no camera | CTBR | yes | capture radius, portable | removing multi-agent coordination and privileged state removes the method | **ARCHITECTURAL_ONLY** |
| [Fast-Tracker](https://github.com/ZJU-FAST-Lab/Fast-tracker) | yes, official | `NEEDS_CONFIRMATION` | n/a — classical | yes | yes | yes, 20 × 20 × 3 m / 140 obstacles | depth + **marker-based** target | trajectory, not velocity | yes, accepts a target stream | distance-based, portable | remove the oracle future trajectory; replace marker detection | **PARTIALLY_MATCHABLE** |
| [Elastic Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) | yes, official | `NEEDS_CONFIRMATION` | n/a — classical | yes | yes | yes, released map 42 × 40 × 5 m / 120 obstacles | depth + separate target camera | trajectory, not velocity | yes, accepts a target position stream | distance-based, portable | **no training**; replace broadcast target with a replayed trajectory | **PARTIALLY_MATCHABLE** |
| [YOPO](https://github.com/TJU-Aerial-Robotics/YOPO) | yes, official | `NEEDS_CONFIRMATION` | pretrained weights stated | yes | **no** | yes, Flightmare forests | depth only | motion-primitive selection | n/a | n/a | different simulator stack and a primitive-library action space | **ARCHITECTURAL_ONLY** |
| YOPOv2-Tracker | **no** — placeholder repo | n/a | n/a | yes | yes | yes | depth 160 × 96 | primitive selection | — | — | — | **BLOCKED** |

No candidate reaches `MATCHABLE`. Every one requires at least one contract dimension to be declared
unmatched, which is why the future protocol below is written as `PARTIALLY_MATCHED` by construction.

## Verified role hypotheses

The expected roles held up against the sources, with one correction.

* **NavRL** — closest PPO obstacle-navigation structure. **Confirmed**, and it is the closest on
  simulator family, action space and sensor family simultaneously.
* **OPEN** — closest pursuit semantics. **Confirmed but unusable as a baseline**: its pursuers read
  privileged relative state with an LOS mask and no camera exists anywhere in the paper, and the
  method is three cooperating pursuers. Reducing it to one sensor-only pursuer leaves the name and
  discards the method.
* **Fast-Tracker** — closest single-UAV target tracking. **Confirmed**, with the caveat that its
  simulation benchmark hands the planner the target's ground-truth future trajectory.
* **Elastic Tracker** — closest tracking + occlusion-aware planning. **Confirmed**, and it is the
  only candidate whose objective already contains a visibility term.
* **YOPO** — closest learned obstacle-aware trajectory generation. **Confirmed**, but its paper is
  paywalled with no preprint, so its own numbers are `NOT_EXTRACTED`.

## Primary candidate

```text
PRIMARY_EXTERNAL_BASELINE_CANDIDATE = Elastic Tracker
SECONDARY_ARCHITECTURAL_REFERENCE   = NavRL
```

**Why Elastic Tracker, on the stated criterion.**

1. **Task compatibility is the highest of the set.** It is a single UAV tracking a moving target
   through clutter while reasoning about occlusion — the same sentence as MOTAR's task. NavRL, YOPO
   and MAD do not track a target at all.
2. **No training is required.** It is a classical planner, so a port does not need a GPU run, does
   not need reward design, and cannot be accused of an unfair training budget. This is the single
   largest cost difference in the table.
3. **Its released arena is already close.** 42 × 40 × 5 m with 120 obstacles against MOTAR's
   40 × 40 × 3 m bar field. Obstacle layouts are configuration data, not algorithm.
4. **Its target is already obstacle-aware.** The released configuration drives the target as a
   second quadrotor that avoids obstacles using a global map — structurally the same idea as TM-E2,
   which makes replaying MOTAR's target trajectory a substitution rather than a redesign.
5. **It addresses a question MOTAR cannot currently answer about itself:** whether the frozen
   learned policy and a classical visibility-aware planner exhibit different success, collision and
   failure profiles under partially matched scenarios. C1–C7 contain no such comparison.

**Why not NavRL as primary.** It is the closest *implementation*, but it does not track a target.
Making it pursue requires feeding a moving goal to a policy trained on static goals — which is either
unfair (out of distribution) or requires retraining it, and retraining an external baseline is both
GPU work and a fairness argument this repository would rather not have to make. It stays as the
secondary architectural reference for navigation, collision and safety behaviour.

**Why not OPEN**, despite having the closest task words: its input contract is privileged state with
no camera, and its method is multi-agent. Both are load-bearing. Porting it fairly is not possible;
porting it unfairly would be a comparison of MOTAR against a crippled version of someone else's
system, which is less defensible than having no baseline at all.

## What each comparison would and would not mean

Following the rule that no system belongs on a universal leaderboard:

| Pairing | Meaningful for | **Not** meaningful for |
|---|---|---|
| MOTAR vs Elastic Tracker | target reacquisition, tracking success under occlusion, close-approach behaviour, visibility handling | RL-policy robustness, learning efficiency, perception-error propagation |
| MOTAR vs NavRL | obstacle navigation, collision rate, safety-filter behaviour under matched density | target tracking, reacquisition, anything target-motion specific |
| MOTAR vs Fast-Tracker | classical prediction and reacquisition under clutter | visibility-aware planning (Elastic Tracker is the better comparator there) |

## Licensing

Only OPEN states a license (MIT) in its repository, and OPEN is the candidate ruled out on method
grounds. The remaining four are `NEEDS_CONFIRMATION` and must be resolved **before** any port, not
after. This repository already tracks third-party licensing as `NEEDS_CONFIRMATION` rather than
assuming, and that convention applies here.
