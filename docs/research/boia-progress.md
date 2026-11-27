# BOIA research progress — October 2026

This development snapshot is dated 22 October 2026 and is separate from MOTAR's historical
scientific results. The machine-readable progress record binds the visible status to local
source commits and SHA-256 receipts. The latest reviewed source snapshot is local research
commit fde21cdf969a7df380aa1c4deddc1824cb524506. This document publishes no BOIA research
code or raw data.

## Research question and proposed architecture

BOIA studies moving-target interception (close approach) in unknown clutter: when should
a vehicle continue Frozen F tracking and when should it propose active reacquisition after
loss of an already acquired target? First acquisition is out of scope. The RGB-D Camera +
LiDAR contract, Frozen F checkpoint and 898-D actor are unchanged.

The proposed path is Frozen PPO tracking → target observation loss → sensor evidence and
target belief → local obstacle representation and candidate reacquisition planning →
intervention timing → CONTINUE or REACQUIRE PROPOSAL. Current validation is shadow-only;
utility remains ABSTAIN and proposals do not command the vehicle. See the
[website schematic](../status/index.html#boia).

## Milestones and current gate

| Milestone | Development status | Evidence boundary |
| --- | --- | --- |
| Q01-V2 | Implemented; runtime qualified | Bounded N1 native/OFF/ON exact parity. Evaluator mechanics, not algorithm efficacy. |
| Q02-A v2.6 | Implemented; CPU tested; independently reviewed | PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS; residual P2 H1/H2. |
| Q02-B | Implemented and CPU tested; independent review FAIL; HOLD | Phase 6 review found 0 P0, 2 P1 and 9 P2; both P1 probes were reproduced and remain unfixed. |
| Q03 | Local map and L1/L2 planner CPU tested; independent review FAIL; HOLD | The planner shares P1-2; L3/L4 are NOT_VALIDATED and no executor exists. |
| Q04 | Design only; D0 HOLD | Requires the Q02-B/Q03 fixes, trigger reporting, preregistration and separate GPU approval. |

The six focused package suites record 112 tests OK: Q02-A 49, Q02-B 29, Q03 map 12,
Q03 planner 15, benchmark metrics 2, and non-interference 5. This does not override the
independent stage-gate failure. The latest research PUBLIC_CPU receipt reports 3,177 tests
run, zero failures, 10 skips and four errors from known Warp DetectReduction tests;
aggregate status is FAIL.

### Independent review findings

- P1-1: a NOT_TESTABLE decision advances loss clocks as if it were a miss. This can bypass
  the two-miss debounce and minimum dwell and alter shadow loss endpoints.
- P1-2: planner L2 does not check stopping/cancel displacement from the current velocity;
  a candidate can pass its screen while the current-speed braking segment intersects an obstacle.
- Both probes were reproduced. Neither P1 was fixed in this snapshot. The review did not
  invalidate recorded synthetic benchmark values, but it requires HOLD.
- Recorded P2 findings include ambiguous trigger-met telemetry, past/future map query limits,
  update-time geometry error, no-return carving and Q04 statistical planning gaps.

Q03's map step records 12 CPU tests and the planner records 15. L3/L4 remain NOT_VALIDATED,
there is no trajectory executor, and no physical free-volume certificate exists. Q04/D0
has not run.

## What Q02-A establishes

The source at 3fb9050a318e1ac6c7b27be42589a0d3f76c2f04 separates detection, footprint geometry,
upstream tracker association and identity uncertainty. Its review at
95196b5eb6948401be7e43419ef20a98f8b3baf8 reports
PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS, no P0/P1 and residual P2 H1/H2.
The v2.6 repair record reports 49/49 focused CPU tests passing. Its earlier research
PUBLIC_CPU receipt reported 3,114 run, zero failures, four existing DetectReduction
errors: aggregate FAIL. The later package-wide PUBLIC_CPU receipt is reported separately above.

The reviewed profile is declared SPHERE, no distractors, matching runtime configuration,
with ORACLE_TRUE_POSE_RASTER detector provenance. That simulator detector is not a
real-image detector. FOOTPRINT_CONSISTENT with CONSISTENT_NOT_CONFIRMED is necessary
geometric evidence, never same-target proof or a sufficient acquisition gate.

Actual occlusion accuracy is NOT VALIDATED. Persistent identity is NOT CONFIRMED.
Synthetic adversarial corner false support and conservative rejection remain. H1 concerns
tie/rival rules; H2 concerns fitting seeds and convergence. Some residuals are search/rule
artefacts, so they must not all be described as information limits. Runtime geometry
attestation has a further follow-up note H4.

No capture improvement, calibrated perception probability, online flight safety, real-time
guarantee or real-flight performance is established. G5 HOLD; further GPU work requires
separate approval. Physical free-space and safe-control blockers are not resolved by
evidence-contract CPU tests.

## Source map and corrections

Full source commits, content hashes and the sanitized read-only Beads observation are in
the [progress record](../status/data/boia-progress.json).

| Record path in the research snapshot | Source commit | Role |
| --- | --- | --- |
| Q01-V2 report and result under docs/evidence/boia_shadow_q01_v2_run_2026-10-21 | 8d286cd | Completed Q01 parity result |
| Q02A_SPEC.md | a5fee45 | Original design; historical v1 details |
| Q02-A v2.5 repair/review | 5fc64a3 / a59a7bb | Prior repair followed by HOLD_P0_P1_REMAINS |
| Q02-A v2.6 repair/review | 3fb9050 / 95196b5 | Final review with residual identity limitations |
| Q02-B/Q03 Opus review and handoff | fde21cd | Phase 6 independent review; two P1 findings; HOLD |

The original Q02-A SPEC/HANDOFF and initial CPU receipt must not supply v2.6 counts or
timing claims. Historical records remain unchanged. The final Q02-A review takes
precedence over the repair's inaccurate residual-failure attribution. The Phase 6 Q02-B/Q03
review takes precedence over earlier WIP notes that described the review as pending.

## Availability and next gates

The site's Code link continues to a separate public release. The source identifiers above
refer to local research history and are not GitHub links. This website update publishes no
BOIA implementation, checkpoint or raw output. The browser GT preview remains an
explanatory visualization, not a PPO or BOIA rollout.

Next: fix Q02-B P1-1 and Q03 planner P1-2, address trigger-reason reporting before Q04-D0,
and obtain a delta review. Q04-D0 remains on hold pending a preregistration and separate GPU
approval. L3/L4, any executor, capture benefit, online intervention and physical safety
remain unvalidated.
