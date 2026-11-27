# Independent review: Q02-B / Q03 CPU shadow package (Opus 5.5, 2026-10-22)

**Review setup.**
- Reviewer: Claude Opus 5.5, in a fresh context (Agent `model="opus"`), read-only.
- Scope: `95196b5..031e77c` plus `tests/test_q02b_q03_noninterference.py`, which was uncommitted at the time.
- Probes: copied unchanged to [probes/](probes/) from `/tmp/claude-1000/opus_review_q03/`.
- The reviewer ran all six package test scripts, and all were OK.

**Verdict: FAIL as a stage gate.** There is no P0, and there are two P1s. The committed benchmark numbers are not
invalidated: no benchmark scenario produces NOT_TESTABLE, and P1-2 needs a moving vehicle, which no benchmark scene
has. Under the package stop rule ("a new P0/P1 → stop"), the package is **HOLD**. Neither P1 has been fixed;
fixing them needs user approval.

The orchestrating session reproduced both P1 probes at `031e77c`.

## P1-1: a flagged NOT_TESTABLE detection counts as a miss

**Location.** In `motar_eval/q02/history.py` `loss_state` (:194 onwards), `informative` looks only at camera input
problems. A flagged NOT_TESTABLE decision is not usable, so it reaches the miss branch. This contradicts the contract
"neither a hit nor a miss" (INTERFACES §2).

**Effect.** Unverified decisions advance the following, which lets H1 propose at the first ordinary miss without the
2-miss debounce or the minimum dwell:
- loss onset;
- `loss_duration_s`;
- `recent_loss_events`;
- `time_since_last_detection`.

**Reproduction (`p1b_h1_after_unverified.py`).**

| Script | k = 14 | Next decision |
| --- | --- | --- |
| 4 SEEN, 10 UNVERIFIED, then OCC | `PROPOSE_REACQUIRE_SHADOW`, loss_duration 1.1, onset 4 | — |
| Control: 14 SEEN, then OCC | `CONTINUE_F LOSS_DEBOUNCE_PENDING` | `MIN_DWELL_NOT_MET` at k = 15 |

**Where it matters.** Far targets give NOT_TESTABLE (Q02-A limit H2). This would corrupt the loss endpoints of D0.

**Proposed fix.** Treat `identity_unverified` decisions like non-informative ones in `loss_state`, and add a
regression test.

## P1-2: L2 ignores the current velocity

**Location.** `reacq_planner.py:174-193`.

**The gap.**
- The cancel phase has only a time budget. The displacement during cancellation is never checked against the map.
- The stopping screen uses the peak speed of a start from rest.
- ROTATE has no stopping screen at all.

**Reproduction (`p2_velocity_cancel.py`).** The vehicle moves at +2 m/s toward a bar whose surface is 1.3 m ahead.
- ROTATE, ADVANCE and RETREAT give L1 PASS and L2 PASS, and a plan exists.
- The braking distance at the current speed is 1.02 m, and that segment is BLOCKED.

**Why P1.** It is the same leniency class as the fix in 82424bc, and it changes which H1/H3 proposals exist in
shadow data.

**Proposed fix.** Screen the braking and cancel displacement from the current speed along the current velocity, for
every candidate including ROTATE, or report L2 FAIL whenever the speed is not zero. Add a regression test.

## P2 findings (recorded, not fixed)

1. **L1 passes through CONFLICT.** L1 fails only on OCCUPIED (`reacq_planner.py:182`). L2 still fails. The
   INTERFACES §4 wording ("occupancy or conflict") does not match the code.
2. **Trigger reasons are ambiguous.** The trigger label is appended whether or not the trigger condition held
   (`shadow.py:214`). "Met but gated" cannot be told from "not met". Fix this before D0, whose endpoints include
   trigger distributions.
3. **One bad decision clock refuses the rest of the episode.** After a single bad clock or index, every later
   decision is INPUT_INVALID. This fails closed, but it is a data-loss hazard for D0.
4. **Map evidence uses the decision-time stamp and pose.** Validation accepts capture up to 0.1 s old, so placement
   error is about 0.25 m at 2.5 m/s. This was not probed.
5. **LiDAR no-return carves to 12.0 m with no one-cell backoff.** FREE reaches 12.21 m, which is quantisation-level.
6. **A past-time query loses older occupancy.** Evidence newer than the query makes the cell UNKNOWN. The pipeline
   cannot reach this, because it always queries at the latest stamp.
7. **Q04_READINESS §C has two statistical gaps.**
   - Time-to-event endpoints conditioned on successes are survivor-selected. They should use all eligible episodes
     with right-censoring (KM/RMST).
   - With 8 seeds, the minimum exact sign-flip p is 0.0078, so seed count needs explicit planning under Holm.
   - D0 "READY_FOR_PREREG" should be conditional on P1-1 and P2-2.
8. **Continuity is checked only between consecutive active tracker means.** Reacquisition is not checked against
   the pre-loss track. This is an identity limit.
9. **`trigger_count` counts every PROPOSE decision.** Repeated proposals while in REACQUIRE_PLANNED are included,
   for example 16 in S2 H1. It should not be read as 16 separate triggers.

## Checked and found correct (reviewer)

- **UNKNOWN is never FREE** in any of these:
  - `segment_status`;
  - line of sight;
  - `clearance`;
  - L2;
  - the stopping screen.
- **Evidence ageing:** free 1 s, occupied 5 s; conflict is kept as CONFLICT.
- **Sensor semantics:** camera no-return stays within the 10 m far plane; near-clipped returns carve nothing.
- **Frames:** re-anchoring and rotation conventions are consistent.
- **No GT inflow:** inputs and imports are firewalled.
- **Identity gates:** usable detections need flag + FOOTPRINT_CONSISTENT + CONSISTENT_NOT_CONFIRMED + NIS. Nothing
  says "confirmed".
- **Planner levels:** L3/L4 are constant NOT_VALIDATED, and there is no executor.
- **Shadow records:** they cannot carry an action, and BOIA cannot propose.
- **Benchmark:** metrics are per policy with the declared-label semantics.
- **Q01 replay:** read-only.

**Not checked:**
- the internals of evidence, geometry and validation (reviewed separately for v2.6);
- the superseded `localmap.py` and `candidates.py`;
- simulator parity;
- the untracked receipts and the handoff document.
