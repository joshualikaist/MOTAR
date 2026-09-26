# MOTAR — project definition

**MOTAR: Moving Object Tracking And Rendezvous.**
Reinforcement Learning for UAV Tracking and Close Approach in Random Obstacle Fields.

This page says what the project is for and what it is trying to establish. What has been established
is in [`docs/EVIDENCE.md`](docs/EVIDENCE.md); how the system works is in
[`ARCHITECTURE.md`](ARCHITECTURE.md); what happened when is in [`docs/HISTORY.md`](docs/HISTORY.md).

## Purpose

A simulated UAV has to find, follow and closely approach a moving object while flying through a
random field of vertical bars. It sees the world only through its own sensors: a forward camera and a
LiDAR. Published work usually studies one piece of this problem in isolation (navigation, tracking,
pursuit, perception or safety) and reports a number inside that piece.

MOTAR measures the **research evidence sequence (not a causal chain)** on frozen policies:

```text
measured perception error → its cost to the policy → whether retraining recovers it
  → the safety geometry that executes the policy's command
  → the target motion the policy is evaluated against
  → the observation contract that renders all of it
```

It keeps the links that failed. The aim is a bounded, reproducible account of which factors move
close-approach performance in simulation and by how much. It is not a new state-of-the-art system.

## Research problem

The pursuer's policy is trained once and then **frozen**. MOTAR asks how its behaviour changes when
the assumptions it was trained under change: what perception reports, how the safety filter measures
clearance, how the target moves, and how the target is rendered. Each question is answered by a
matched, preregistered comparison in which exactly one factor changes.

## Research questions

| # | Question | Where it is answered |
| --- | --- | --- |
| Q1 | How large is perception error on real UAV imagery, measured rather than assumed? | ETH ds5 range proxy (E3-S); detector and association error models (P6–P8) |
| Q2 | What does that measured error cost a frozen policy, and does retraining recover it? | P9 injection, P10 readaptation |
| Q3 | Does the geometry the safety filter uses to measure clearance change collisions? | Arc-clearance vs riskcap, 15 seed × density cells |
| Q4 | Is target motion a difficulty ladder for a frozen policy? | H / E0 / E1 / E2: an independent bias-corrected replication (7 fresh seeds, 229,376 primary episodes) of the original 3-seed campaign |
| Q5 | How sensitive is a frozen policy to how the target is rendered? | D8b observation-contract contrast |
| Q6 | Which of these axes do published systems study, and can any be compared directly? | Positioning ledger; Class A matched comparisons = 0 |

## Current contributions

Each contribution is bounded by what its evidence cannot carry. The numbers are in
[`docs/EVIDENCE.md`](docs/EVIDENCE.md).

1. **Perception uncertainty is measured, not assumed.** An image-size range proxy on real footage.
   One flight only; the attitude decomposition is BLOCKED.
2. **Measured perception error is propagated into the policy.** The cost is clear. A net benefit from
   readaptation is **not** established (INCONCLUSIVE); that negative finding is part of the contribution.
3. **Temporal perception is tested explicitly.** A temporal selector is chosen on validation, and a
   learned detector is non-inferior in the loop. Persistent identity is not established.
4. **Safety-filter geometry is separated from the speed law.** Measuring clearance along the turning
   arc lowers the crash rate of policy F relative to riskcap in all 15 seed × density cells; the
   seed-level interval (three evaluation seeds) also excludes zero. This is a configured contrast, not
   a safety guarantee.
5. **Target-motion generalization of frozen policy F is measured and replicated.** Target motion was
   not a monotonic difficulty ladder: the static target produced the lowest capture rate and the
   obstacle-aware target the highest. All three preregistered capture contrasts met the directional
   replication criterion in an independent seven-seed bias-corrected replication. The obstacle-aware
   gain was not uniform at high density. In the original campaign the first-acquisition record is
   consistent with a visibility mechanism (association only).
6. **The observation contract can dominate.** Rendering the target differently produced the largest
   loss measured anywhere (MATERIAL_LOSS). Its cause is NOT_TESTED, and it used a different frozen
   checkpoint, policy R, from the policy F of contributions 2, 4 and 5.
7. **The record is kept whole.** Result directories are indexed with provenance links and hashes, and
   preregistrations, receipts where a directory has one, and negative, withdrawn and VOID results are
   kept beside the positive ones.

## Target readers

- **Researchers** in UAV navigation, pursuit and perception who want a bounded account of what
  moves close-approach performance, and the conditions each number holds under.
- **Reviewers** who need to trace any number to its preregistration and receipt.
- **Engineers and agents** extending the code. They start from [`AGENTS.md`](AGENTS.md) and
  [`ARCHITECTURE.md`](ARCHITECTURE.md).

## Success criteria

The project succeeds when each research question has an answer that meets all of these:

- It was fixed by a preregistration committed before execution, or it is labelled post hoc.
- It is produced by a fail-closed launcher, with a receipt that records source, runtime and hashes.
- It is reported with its unit of replication and interval, and without significance claims the design
  cannot support (three seeds cannot reach conventional significance).
- It is bound by test to the number shown in the README, site and figures.
- Its negative, inconclusive or withdrawn outcome is retained where it applies.

A result that fails these standards is recorded as VOID, NOT_RECORDED or NOT_TESTED. It is not reworded.

## Scope

In scope:

- Simulation in Aerial Gym on Isaac Gym Preview 4, with a quadrotor, a 40 × 40 × 3 m arena and
  70–205 random bars.
- Sensor-only pursuit: ground-truth target state never enters the actor observation.
- Frozen-policy evaluation of perception, safety geometry, target motion and observation contracts.
- Offline measurement of detector, association and range error on public real-footage datasets.
- An independent static renderer and its measurement contract.
- A browser visualisation of the arena. It is an explanation only, not evidence.

Out of scope:

- Real-flight experiments, hardware validation, sim-to-real transfer and deployment.
- State-of-the-art or cross-paper performance claims. No external number is ever subtracted from a
  MOTAR number.
- Operational interception or engagement. The historical metric names `interception` and `capture`
  keep their meanings. The public term *moving-target rendezvous* (close approach) does not reinterpret
  them.
- Reactive or learned evaders (TM-E3, TM-E4): planned, not implemented.

## Current phase

**Evidence consolidation and paper writing (2026-09-26).** The measurement programme behind
contributions 1–6 is complete, including the target-motion replication. No GPU evaluation or PPO
training is authorised.

| Track | State |
| --- | --- |
| Paper | Spine, outline and claim/evidence matrix of 2026-09-19; figures 1–6 generated from records |
| External matched baseline | Elastic Tracker selected. Upstream reproduction gate B0 FAILed on the environment; the remaining blocker is `sudo` to install ROS. B1–B5 are not authorised |
| Task diagnostics | TD-T1 / TD-T2 recorders exist; their density sweep has not run |
| Perception | D8c read-only audit COMPLETED ([`docs/d8c_perception_path_audit_2026-09-24.md`](docs/d8c_perception_path_audit_2026-09-24.md)). Mesh-shaded rendering is not the final policy observation for the current contract, so D8b stays sensitivity evidence (causality NOT_TESTED) and no retraining is required for the current claim. D9 (shortcut remeasurement) is PLANNED |
| Target behaviour | The independent bias-corrected seven-seed replication of H / E0 / E1 / E2 is complete and replicated all three contrasts ([`results/target_motion_replication_7seed/README.md`](results/target_motion_replication_7seed/README.md)); it is closed at seeds 4104–4110. TM-E3 / TM-E4 are PLANNED |

Any new experiment requires, in this order: a question that one of the tables above leaves open, a
preregistration, and explicit approval from the user. Rules for agents are in [`AGENTS.md`](AGENTS.md).

## Relationship to earlier plans

The original charter, [`RESEARCH_PLAN.md`](RESEARCH_PLAN.md), set out density × target-speed
questions and interception hypotheses H1–H5 in July 2026. It is kept unchanged because receipts cite
its §8.8 as a preregistered plan. This page supersedes it as the statement of current intent.
