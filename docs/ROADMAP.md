# Roadmap

Long-term direction, milestones and success conditions. Actionable work is tracked in Beads, not
here. Each milestone names its Beads epic: run `bd show <epic>`, or `bd list --label <label>` for its
issues. Finished milestones keep one line of outcome and the record behind it.

Research questions and scope are defined in [`PROJECT.md`](../PROJECT.md). The current state is in
[`PROJECT_STATE.md`](PROJECT_STATE.md), and decisions are in [`DECISIONS.md`](DECISIONS.md).

## Direction

Measure what perception, safety and target-motion conditions do to a frozen RGB-D + LiDAR pursuit
policy in simulation, and qualify corrections one lineage at a time:
- the environment first;
- then sensor fusion;
- then plant and governor;
- then active visibility.

Every claim is bound to a preregistration and a machine-readable record. There is no real-flight
claim and no state-of-the-art claim.

## Completed milestones

| Milestone | Outcome | Record |
| --- | --- | --- |
| MOTAR measurement programme (Q1–Q6) | Measurement complete as of 2026-09-26 | [`PROJECT.md`](../PROJECT.md), [`EVIDENCE.md`](EVIDENCE.md) |
| Occlusion study OCC-G0/P1/P2 | RQ1/RQ3 supported; RQ2 inconclusive (equivalence not tested) | [`evidence/occ_p2_confirmatory_2026-10-17/REPORT.md`](evidence/occ_p2_confirmatory_2026-10-17/REPORT.md) |
| Environment qualification EQ-0 → EQ-1D → E-L0 | Complete; the environment verdict is LIMITED_VALIDITY | [`evidence/eq1d_qualification_2026-10-18/REPORT.md`](evidence/eq1d_qualification_2026-10-18/REPORT.md) |
| S-L1 sensor-fusion corrections, v1 | A/B/C/D v1 GO; C-v2 scoped GO | [`evidence/s_l1c_v2_2026-10-18/QUALIFICATION.md`](evidence/s_l1c_v2_2026-10-18/QUALIFICATION.md) |

## Active and future milestones

### M1 — KROC2027 study (epic `motar-d43`, label `kroc2027`)

- **Goal:** report the KROC2027 hypotheses on Policy F.
- **Dates:** Simulation Freeze 2026-10-28, internal paper 2026-11-13, submission 2026-11-26.
- **Success condition:** a submitted paper whose numbers all come from committed records.
- **Status:** H-A supported; H-A2/I_A and H-B inconclusive
  ([record](evidence/mt_precision_review_2026-10-16/README.md)).
- **Precondition:** the Phase 1 exception in [`AGENTS.md`](../AGENTS.md) ends at the freeze or at the
  merge, whichever comes first.

### M2 — H-R3 active visibility recovery (epic `motar-6nn`, label `h-r3`)

- **Goal:** test whether active translation to a re-observation viewpoint adds identity-correct
  re-observation beyond prediction-only and yaw-only control.
- **Success condition:** a preregistered, seed-clustered confirmation with Holm correction, run only
  with explicit approval.
- **Status:** PROVISIONAL.
  - The chain so far is P01 exploratory → P02 NO-GO (integrity) → ISO-Q02 mechanics GO →
    P03-Q01 GO for qualification only.
  - Attribution is INCONCLUSIVE, and confirmation is on HOLD.
  - [Record](evidence/h_r3_p03_preparation_2026-10-19/REPORT.md).
- **Preconditions, in order:**
  1. the estimand decision (`motar-6nn.1`);
  2. the control redesign (`motar-6nn.2`);
  3. the optional pilot (`motar-6nn.3`).

### M3 — S-L1 follow-ups (epic `motar-kr2`, label `s-l1`)

- **Goal:** close the open sensor-fusion limits:
  - target-return carve retention;
  - camera centre-range bias;
  - D-on-C-v2 stress;
  - the C-v2 limits;
  - the remaining EQ-1D FAILs.
- **Success condition:** each item preregistered and qualified, or kept as a disclosed limit.
- **Design:** [`plans/post_s_l1c_v2_validation_design.md`](plans/post_s_l1c_v2_validation_design.md).

### M4 — Plant and governor lineages P-L1 → G-L1 (epic `motar-tbx`, label `lineage`)

- **Goal:** characterise the motor ZOH and the plant, then add a stopping-safe governor.
- **Success condition:** a qualified lineage with a preregistration. Until then there is no
  physical-safety claim.
- **Order:** after S-L1 ([`plans/correction_lineages_plan_v2.md`](plans/correction_lineages_plan_v2.md)).

### M5 — Repository and documentation upkeep (epic `motar-lp5`, labels `repo`, `docs`)

- **Goal:**
  - publish and merge the work branch, with approval;
  - bring the canonical docs up to the October work;
  - qualify the public target-motion claims for TF32 exposure.

### M6 — Legacy backlog (epic `motar-viu`, label `legacy`)

- **Contents:** pre-October carried items:
  - the paper checklist;
  - the TF32-off rerun;
  - H-R1/H-R2;
  - the OCC-P2 recovery package;
  - TD-T1/TD-T2, D9, TM-E3/E4, Elastic Tracker B0 and E3-P.
- **Each item** is to be kept, dropped or scheduled, with a recorded reason.

## Checking for forgotten work

1. **Epics:** `bd list --type epic` should show one epic per milestone above.
2. **Stale items:** `bd doctor --check=conventions` reports orphans and stale issues.
3. **Plans without issues:** compare `docs/plans/` and the newest `docs/evidence/*/NEXT_*.md` with
   `bd list`. Any plan step with no issue and no evidence is forgotten work; file it with `bd create`.
