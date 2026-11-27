# Recovery hypothesis candidates — PROVISIONAL — NOT PREREGISTERED

User clarification supplied during EQ-0 on 2026-10-17. This file records candidate research
questions, not a frozen preregistration, implementation approval, verified result or final choice
of endpoints. EQ-0 audits whether an environment can test them; it does not test their effects.
OCC-P1/P2 RQ1/RQ2/RQ3 are different questions. Existing OCC-P2 may motivate exploratory design,
but must not be reused as independent confirmatory data for these new algorithm hypotheses.

## H-R1 — Target Tracking Continuity

Under short, repeated losses of target observation, combining target-state prediction and
visibility-aware control may improve valid tracking continuity and time-bounded reobservation
relative to an existing policy or prediction-only method.

Candidate outcomes: valid tracking duration, reobservation success within 1–2 s, identity
preservation and FOV-exit rate. Conditional redetection after geometric restoration is distinct
from end-to-end Recovery success.

EQ-0 implications: separate actual camera obstruction from partial/centre-only LOS, FOV/scale
loss and camera/fused measurement absence. Confirm identity, timestamps, accepted-correction
age, covariance and historical-feature availability. Current clean redetection and LiDAR-assisted
memory are insufficient evidence of realistic continuity; an active flag alone is not a valid-ID
metric. No effect size, margin, primary endpoint or positive verdict is assigned.

## H-R2 — Safe Target Approach

Under unstable target observation, control that accounts for state-estimation uncertainty and
collision risk may improve collision-free target-approach success and navigation safety relative
to Frozen Policy F.

Candidate outcomes: collision-free capture, crash, timeout, safe abort and approach time.
Reducing crashes merely by stopping or avoiding the task does not establish improvement.

EQ-0 implications: qualify pursuer/target dynamics, collision support, reachable obstacle passages
and stopping/control semantics. Preserve and separately label all raw contacts, including
same-step capture/contact; define safe abort without treating it as capture. Existing capture
priority and riskcap do not qualify collision-free completion. Safety and task completion must
be assessed jointly under preregistered rules, not by selecting the favorable metric afterward.

## H-R3 — Contribution of Active Visibility Control

With the same target-state predictor and sensor information, actively choosing a viewpoint
favorable to reobservation may improve tracking and reobservation over prediction-only or
yaw-only methods.

Candidate outcomes: valid reobservation rate, latency, FOV retention, movement cost and collision
risk. Use matched baselines sharing the information and pre-intervention state to identify the
incremental contribution of active control.

EQ-0 implications: declare the same sensing capabilities, predictor/belief implementation,
uncertainty inputs, action limits, timing and common acquired pre-loss prefix for all arms.
Future observations may legitimately diverge as a consequence of active movement; do not force
post-intervention cached observations to be identical and thereby remove the intended mechanism.
No arm may receive GT target pose, GT visibility, known simulator layout or synthetic depth
unavailable to its declared real sensors through an online scoring/supervision path. GT may
independently evaluate outcomes/visibility. Capture simulator/controller/tracker/history/RNG
state or validate deterministic prefix replay; the current OCC traces alone do not provide a
complete branch-and-replay snapshot. Matched comparison qualification is NOT_TESTED.

## Sensor contract: actual Frozen F versus a camera-only candidate

Actual OCC-P2 F uses LiDAR plus synthetic RGB-D: scan 72×4, camera depth, fused static obstacles,
camera target estimate and LiDAR range association. Its 898-D actor includes a 288-D fused scan
and 480-D obstacle history, as well as ego/target history. A camera-only supervisor attached to
this unchanged actor would still be part of a LiDAR-assisted system.

The long-term camera-only direction remains a **candidate**, not the current qualified contract.
Removing/zeroing LiDAR while keeping 898 numbers does not preserve the trained observation
distribution or establish a matched baseline. A real RGB-only system also needs a declared
source of metric range/scale/map/ego motion; synthetic depth cannot silently supply it. Camera-only,
RGB+LiDAR and real RGB-D studies require separately stated scopes and approval. No retraining,
new observation schema, modality ablation or Recovery implementation is authorized by this file.

Before a formal H-R3 comparison, fix the modality contract and give all arms that same information.
A comparison across different modalities is a whole-system contrast, not the isolated extra effect
of viewpoint control. H-R2's comparison against Frozen F must disclose any information mismatch.

## Status and next decision

H-R1: PROVISIONAL — NOT PREREGISTERED — EFFECT NOT_TESTED.

H-R2: PROVISIONAL — NOT PREREGISTERED — EFFECT NOT_TESTED.

H-R3: PROVISIONAL — NOT PREREGISTERED — EFFECT NOT_TESTED.

After EQ-0 review, the user will finalize hypothesis wording, metrics, scope and experimental
conditions. EQ-1 remains a proposal. Existing data changes, Recovery Scenario/Belief/Planner,
PPO, GPU execution, main merge and push remain unapproved.
