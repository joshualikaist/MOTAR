# Q04 preregistration — DRAFT (not a preregistration)

Status: **DRAFT, DESIGN_ONLY, NOT APPROVED**. It is written at a non-preregistration path on
purpose. Only a completed copy committed under `docs/plans/prereg_*.md` binds anything, and that copy
becomes immutable once committed. GPU NOT_RUN, G5 HOLD. The design rationale is in
[Q04_READINESS.md](Q04_READINESS.md).

## Q04-D0 — CONTINUE-only shadow data collection (the first GPU request; still to be approved)

| Field | Draft value | To fix before commit |
| --- | --- | --- |
| Question | Under Frozen F, how often, when and with what evidence do previously-acquired targets get lost, and what do the five shadow triggers propose? | — |
| Not asked | Any effect of reacquisition; any capture-rate claim for BOIA | — |
| Arms | F0 executes; F0/H1/H2/H3/BOIA are logged in shadow only (owner F, no action) | — |
| Policy | Frozen F `f7022139…`, RMS `642fc9f7…` | source SHA |
| Detector lineage | `FROZEN_F_ORACLE_PROXY_RENDER` only (error-injected and image-based detectors are separate runs) | — |
| Scenes | hidden initial view + moving target; 70 and 160 bars | target-motion profile and speed |
| Seeds × episodes | indicative 8 seeds × 2 densities × 20 episodes | the final number, with justification |
| Processes | N1 fresh processes (avoids replay coupling) | wall cap |
| Observer | a new, versioned read-only tap calling `ShadowRecorder.observe`; no production edit | the tap's path and its tests |
| Recorded online | `Q02A-RECORD-3`, the 19 features, the five shadow decisions, Q03 map validity and candidate summary, F action, host runtime | the packet subsample rule |
| Evaluator labels | target pose, line of sight to camera and LiDAR, FOV, mesh occlusion, detector TP/FP, identity correctness, outcome (joined by record ID) | label code and its tests |
| Primary descriptive endpoints | rate of eligible episodes; loss onsets per eligible episode; trigger counts and latency per shadow policy against evaluator labels; collision-free capture under F0 | — |
| Stop conditions | source, configuration or checkpoint mismatch; disk below 3 GB; output cap; VRAM ceiling; wall cap; observer exception | the cap values |
| Integrity | raw manifest with SHA-256; numerics profile recorded; one S0 replay control | — |

## Q04-D1 / Q04-L / Q04-C — BLOCKED

These stages need:
- an executor;
- L3 qualification of candidates;
- for Q04-C, a learned or abstaining BOIA rule.

When these are unblocked, the preregistration will fix the following:
- the branch-point sampling rule;
- the number of pairs per (seed, branch time, candidate) and the keyed exogenous jitter;
- δ_risk and C_switch;
- the model-selection procedure (Q04_READINESS §D);
- held-out seeds;
- the primary contrast and margins;
- Holm correction over the four contrasts;
- cluster-bootstrap and sign-flip analyses at the seed level;
- the common-prefix eligibility rule (Q04_READINESS §C).
