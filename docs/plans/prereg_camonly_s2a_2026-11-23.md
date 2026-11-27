# Preregistration: CAM-S2a — camera-only from scratch with the vehicle-matched rules that exist tonight (training only)

Written 2026-11-23 ~22:45 KST, before any CAM-S2a run. Branch `camonly/dev`. Family `camera-only`. User decision
D-2026-11-23-3. Rule sheet: `~/motar_tmp/cams2_design_20261123/RULES.md` (r3), built from the measured vehicle facts
(`~/motar_tmp/cams1_20261123/vehicle_facts/README.md`) and two audits (sim/training and runtime, Claude Opus 5.5).

## Question and scope

Can a camera-only policy, trained from random initialisation against a quadrotor-shaped target with the measured
vehicle values, a delayed command, and a safe capture definition, learn the approach task? Training only; evaluation
needs its own preregistration. Scope approved by the user on 2026-11-23 ("내일 아침에 실행되는거 봐야된다"): one
development smoke (repeatable until one completes), then ONE run of seed 1, at most 12,000 epochs and 17 h.

## What is in CAM-S2a (and what is deferred to CAM-S2b)

| Rule | CAM-S2a setting | Source |
| --- | --- | --- |
| Start / sensors | random initialisation; camera only (no_lidar) from epoch 0 | D-2026-11-23-1 |
| Target | the quadrotor mesh (D8 v3 asset, `NAVRL_DYNAMIC_MESH_TREATMENT=mesh_flat`), not the red sphere; its rendered mask is the clean detection geometry; v3 is 6-8 % smaller and 0.12 m tall versus the real ~0.17 m (limit) | user rule; R1/R2 |
| Detection resolution | 624 x 468 (the real 4:3 profile D) | R2/R25 |
| Detector error | P9 injector (detection-loss placeholder; size bins now in comparable pixels) | R2 interim |
| Camera | 66.7 x 54 -> 66.7 x 52.6 deg (IMX378 full 4:3), mount 0.08 m forward, 0.03 m up | R4, R23 |
| Depth | OAK-D model f*B 29.88 px m, sigma_d 0.125 px, MinZ 0.315 m, MaxZ 20 m, holes 0.02; blob_near removed | R5 |
| Vehicle | per-motor limit at T/W 3.5 (measured hover 0.287), thrust +-15 %, imbalance +-5 %, tau 0.02-0.04 s | R8 |
| Controller | Lee gain randomisation +-20 % (`NAVRL_GAIN_RAND`) | R9/R18 interim |
| Latency | action delay k ~ U{1, 2} control steps per episode (100-200 ms); recorded as NAVRL_ACTION_DELAY_STEPS_MAX=2 | R7 |
| Capture | centre distance <= 1.0 m AND closing speed <= 1.0 m/s AND target detected; centre distance < 0.40 m (swept) = collision with the target | R3 |
| Altitude | 1.5 m (both drones) | R16 (user) |
| Rewards / rules | CAM-S1's: collision -40, static-safety range 4 m, blind-motion penalty 0.5, blind cap 1.0 m/s outside +-33.35 deg, governor FREE 2.5 m/s | R13/R14 |
| Curriculum | time-based: 70 bars for 1,000 epochs, then +15 every 1,000 (gates 0.001) | R15 |
| Checkpoints | every 250 epochs | — |

Deferred to CAM-S2b (need code or tomorrow's data): body-fixed camera with levelling (R17), bumpy ground with the
runtime's ground separation (R6), forest trees and 80x48 obstacle render (R19/R20), PX4 velocity-response lag (R18),
ego-state noise (R22), measured invalid-depth model (R5), collision box 0.30 x 0.30 x 0.17 (R21), altitude
randomisation (R24).

## Stop rule and labels

At epoch 3,000, if the episode-weighted training-rollout capture of the last 200 epochs is below 0.05, the run stops
(STOPPED_NO_LEARNING). Labels: TRAINED, STOPPED_NO_LEARNING, WALL_CAP, TRAINING_FAIL, VOID (host faults rose),
ABORTED_BY_USER. Recorded: epoch, density stage per epoch, last-100 capture/crash/timeout, `[camsafe]` counters,
CAM-S2a contact/capture counters, s/epoch, checkpoint SHA-256 of the last and every 1,000th epoch. Training-rollout
rates are not evaluation results.

## Not claimed

No evaluation, no comparison, no flight readiness (the runtime still needs the ROS 1 port, the gateway timing bound
from the contract, and the drone detector). One seed. The capture rule and the D8 target make this a new task; no
number compares with earlier lineages.

## Review

No independent pre-run review: the user asked for the run tonight. The design rules were audited (sim/training and
runtime audits, reward review r1, design review r1 of CAM-S1); the new code (`navrl_cams2.py`) has CPU tests
(`tests/test_cams2.py`). An independent review of this run's code runs in parallel with the run; a finding that
invalidates it stops the run.
