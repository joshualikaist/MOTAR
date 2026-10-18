# MOTAR sensor contract

Authority: the user's official sensor correction, 2026-10-18. Applies to current and subsequent
research until another explicit user instruction changes it. This corrects an assistant's
unsupported interpretation; the user did not request a change to a camera-only research goal.

## Required sensing and information boundary

| Component | Required role |
| --- | --- |
| RGB-D Camera | Target detection, bearing/range measurements and tracking information |
| LiDAR | Obstacle detection, range measurements and target-association assistance |
| Perception / Tracking | Fusion of camera and LiDAR measurements |
| Policy | Tracking, avoidance and approach from fused sensor observations |

The baseline remains **Frozen Policy F, 898-D actor, RGB-D + LiDAR**. Keep LiDAR functional;
removal, disabling, zeroing or a camera-only policy conversion is outside the current requirements.
An individual camera-validity flag, LiDAR-only update or modality ablation in a historical record
does not define a different current system contract.

RGB-D depth measurements are permitted online inputs. Their simulated rendering still needs
qualification for geometry, validity, noise, latency and calibration; the modality decision does
not establish hardware realism. Simulator target state, GT visibility, semantic identity/masks and
evaluation provenance must not be injected as privileged inputs to online tracking or control.
Independent evaluation may use geometry and semantic provenance. Preserve existing uses for
detector supervision, rewards, termination and the critic, outside the actor/online information
boundary. Do not substitute oracle range/depth for the declared RGB-D or LiDAR measurement model.

## S-L1 scope

S-L1 improves the correctness and reliability of the existing fusion system. The original
sensor correction approved only S-L1A. The user's subsequent acceptance at `aed6aff` recognizes
S-L1A/B/C/D v1 within their preregistered scopes and authorizes the separate C-v2 investigation:

| Package | Technical objective | Current production scope |
| --- | --- | --- |
| S-L1A | Obstacle depth and target-like carve-out correction | v1 accepted within its recorded qualification scope |
| S-L1B | Camera–LiDAR pinhole / SE(3) calibration | v1 accepted; further carve-retention production changes require approval |
| S-L1C | LiDAR false association, tracker identity and covariance | v1 accepted; separate opt-in v2 authorized under its own gates |
| S-L1D | Silhouette occlusion and sensor measurement validity | v1 accepted on C-v1; qualification is not inherited by C-v2 |

The [v2 correction plan](plans/correction_lineages_plan_v2.md) and
[dated sensor amendment](plans/sensor_contract_amendment_2026-10-18.md) remain historical records
of the original approval boundary. They are not rewritten. The subsequent
[C-v2 preregistration](plans/s_l1c_v2.md) and [measured evidence](evidence/s_l1c_v2_2026-10-18/RESULTS.md)
record the newly authorized work and its narrower qualification limits. Neither the sensor
correction nor these scope updates authorize PPO, Recovery Planner, actor-schema changes or
main merge. RGB-D + LiDAR remains the official contract throughout.

## Provisional Recovery hypotheses under fusion

All three remain **PROVISIONAL — NOT PREREGISTERED**. These are design hypotheses, not results:

- **H-R1 — Target Tracking Continuity:** when fused sensor observations are temporarily unstable,
  combining target-state prediction and visibility-aware control may improve valid tracking
  continuity and timely reobservation over the existing policy or prediction-only methods.
  Candidate measures include valid-track duration, reobservation within 1–2 s, identity retention
  and FOV exits. Report conditional redetection separately from end-to-end Recovery success.
- **H-R2 — Safe Target Approach:** control based on fused observations, estimation uncertainty and
  collision risk may improve collision-free approach and navigation safety over Frozen Policy F.
  Candidate measures include collision-free capture, crash, timeout, safe abort and approach time.
  Reducing crashes by merely stopping or avoiding approach is not sufficient evidence of improvement.
- **H-R3 — Contribution of Active Visibility Control:** with the same RGB-D + LiDAR sensing
  capabilities, permitted information access and predictor, active viewpoint selection may improve
  tracking and reobservation over prediction-only or yaw-only controls. Match the pre-intervention
  state and sensor history; subsequent measurements may differ as viewpoint changes. Measure
  valid reobservation, delay, FOV retention, movement cost and collision risk.

OCC-P1/P2 RQ1/RQ2/RQ3 are distinct questions. Their existing data may inform problem selection and
experimental design, but are not confirmatory test data for new H-R algorithm hypotheses. This
contract neither preregisters the hypotheses nor changes any recorded qualification outcome.

## Traceability and publication

Preserve historical raw, manifests, checkpoints, preregistrations, user WIP and stash. Record
validated authorized work in local commits under the
[Git Traceability Protocol](research_git_traceability_protocol.md).

The user's existing conditional permission to push the research branch remains subject to its
previously approved history/security checks. This document does not broaden the branch scope or
declare those checks complete. Main merge remains prohibited. Local commits alone are not visible
on GitHub; report whether any authorized push actually occurred.
