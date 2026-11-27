# Correction lineages v2 — preserve RGB-D + LiDAR fusion

Status: current scope record, 2026-10-18. Parent source
`183a333bd01c0124a95ae379dba0a8509aebc999`, branch `codex/mt-p1`.
Read the [official sensor contract](../SENSOR_CONTRACT.md) and
[sensor amendment](sensor_contract_amendment_2026-10-18.md) first.

This additive revision replaces the unsupported modality roadmap in
[v1](correction_lineages_plan_v1.md); v1 and all historical evidence remain unchanged. It does not
reopen sensor selection: **RGB-D Camera + LiDAR** is required. No LiDAR-free camera lineage,
single-RGB versus RGB-D decision, or new camera-only policy is scheduled.

## S-L1 package boundary

The user confirms S-L1A production approval and preserves the following division. Only the first
row is approved; the combined v1 proposal must not be treated as one authorized implementation.

| Package | Scope | Boundary / validation direction |
| --- | --- | --- |
| **S-L1A — approved** | Correct obstacle depth and target-like carve-out | Retain RGB-D and LiDAR; independently verify obstacle depth and foreground obstacle retention; demonstrate unchanged switch-off behavior |
| S-L1B — planned | Camera–LiDAR pinhole / SE(3) calibration | Separate implementation approval; projection and extrinsic fixtures |
| S-L1C — planned | LiDAR false association, tracker identity and covariance | Separate implementation approval; provenance, identity and covariance fixtures |
| S-L1D — planned | Silhouette occlusion and sensor measurement validity | Separate implementation approval; centre LOS versus full silhouette and valid-measurement fixtures |

For S-L1A, the v1 depth/carve-out rows are technical starting points, not a mandate to implement
every option listed there. Freeze the precise diff and independent CPU gates for that package.
Do not absorb B/C/D calibration, tracker, covariance or actor-visibility changes into A. Preserve
the existing 898-D actor, checkpoint and baseline sensor behavior behind the opt-in lineage;
do not disable a sensor or change the tracker's existing 5 s expiry contract.

Required evidence for implementation is an exact diff, independent depth/foreground-clearance
tests, regression checks and baseline non-interference. E-L0's evaluation-only labels and existing
EQ-1 fixtures can support qualification without becoming online sensor inputs. A new GPU check
needs its own applicable execution approval and committed preregistration; this documentation
unit launches none. New-lineage evaluation with Frozen F measures environment sensitivity and
robustness; it does not establish a Recovery algorithm benefit or authorize PPO retraining.

## Other lineages and order

E-L0 remains the evaluation instrument. S-L1 sensor corrections precede the proposed P-L1 plant
and G-L1 governor corrections because later safety measurements depend on sensor correctness.
P-L1 and G-L1 remain separate proposals from v1, with no implementation approval in this correction.
No new physics, dynamics, governor, Recovery Planner or historical outcome is changed here.

Future sensor work continues within RGB-D + LiDAR fusion. Hardware realism, range/depth provenance,
latency, identity and online GT isolation remain qualification questions; choosing this contract
does not turn an unresolved or failed measurement into PASS.
