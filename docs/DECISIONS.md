# Decisions

Important architecture, experiment and process decisions, with their reasons. Each entry gives:
- the date and the decision;
- the alternatives;
- the reason, the impact and the reversal condition;
- the sources.

A decision that replaces an earlier one marks the earlier one **superseded** and keeps it. Nothing is
deleted. Routine choices are not recorded here; they live in the [`WORKLOG.md`](../WORKLOG.md) entries
and the Beads issues.

This file was seeded on 2026-10-19 from existing records. Each entry cites where the decision was
first recorded, and that source stays authoritative.

## D-2026-10-19-2 — Beads for TODOs, Claude-Mem for reference memory

- **Decision:** Beads (`bd`) is the only tracker for actionable work: status, priority, dependencies
  and blockers. Claude-Mem is searchable memory of past sessions, and it never outranks code, tests
  or records.
  - Long-term direction lives in [`ROADMAP.md`](ROADMAP.md) and the current state in
    [`PROJECT_STATE.md`](PROJECT_STATE.md).
  - `WORKLOG.md` stays the append-only research log.
- **Alternatives:** Markdown TODO lists; the evidence folders' `NEXT_*.md` files as the only plan.
- **Reason:** one source of truth for open work, queryable by any session.
- **Impact:**
  - Open items were migrated as issues `motar-*`.
  - `NEXT_*.md` files remain records of what was planned at the time; they are not live TODO lists.
- **Reversal:** a user decision.
- **Sources:** the user's 2026-10-19 setup instruction; [`OPERATIONS.md`](OPERATIONS.md) §Task tracking.

## D-2026-10-19-1 — External backup declined (BACKUP_DECLINED_BY_USER)

- **Decision:** no external raw backup, Git bundle or transfer to another machine. Raw data stays in
  the local layout.
- **Alternatives:** an external SSD, a server or Git bundles. The user has no such destination.
- **Reason:** the user's decision.
- **Impact:**
  - Backups are not proposed again and are not used as a precondition for Git cleanup.
  - **This does not authorise deleting anything.** Raw data, checkpoints, Policy F, the WIP and stash,
    and the symlinks are all preserved.
- **Reversal:** the user names a destination.
- **Sources:** the user's 2026-10-19 instruction. It supersedes the "external backup NOT_ESTABLISHED,
  destination pending" notes in WORKLOG and in
  [`evidence/mt_precision_review_2026-10-16/README.md`](evidence/mt_precision_review_2026-10-16/README.md).

## D-2026-10-19-0 — H-R3 P03-Q01: GO for qualification only

- **Decision:** the mechanics qualify. Pure-viewpoint attribution is INCONCLUSIVE, and no automatic
  pilot follows.
- **Reason:** B3−B2/B4/B5 imbalance exceeded the margin.
- **Impact:** the user must choose the estimand next (`motar-6nn.1`).
- **Source:** [`evidence/h_r3_p03_preparation_2026-10-19/REPORT.md`](evidence/h_r3_p03_preparation_2026-10-19/REPORT.md).

## D-2026-10-18-4 — H-R3 P02 integrity failure: confirmation NO-GO and HOLD

- **Source:** [`evidence/h_r3_p02_2026-10-18/decision.json`](evidence/h_r3_p02_2026-10-18/decision.json).

## D-2026-10-18-3 — H-R3 P01 is exploratory only

- **Decision:** stop before any confirmatory run. The independent review returned
  ACCEPT_WITH_CORRECTIONS.
- **Source:** [`evidence/h_r3_pilot_2026-10-18/REVIEW.md`](evidence/h_r3_pilot_2026-10-18/REVIEW.md).

## D-2026-10-18-2 — S-L1 accepted; C-v2 scoped GO

- **Decision:**
  - S-L1A/B/C/D v1 are accepted.
  - C-v2 gets a GO limited to its preregistered scope.
  - D stays disabled on C-v2.
- **Sources:** [`SENSOR_CONTRACT.md`](SENSOR_CONTRACT.md);
  [`evidence/s_l1c_v2_2026-10-18/QUALIFICATION.md`](evidence/s_l1c_v2_2026-10-18/QUALIFICATION.md).

## D-2026-10-18-1 — The environment is LIMITED_VALIDITY; correction lineage order is fixed

- **Order:** E-L0 → S-L1 → P-L1 → G-L1.
- **Source:** [`evidence/eq1d_qualification_2026-10-18/REPORT.md`](evidence/eq1d_qualification_2026-10-18/REPORT.md).

## D-2026-10-18-0 — The sensor contract is RGB-D camera + LiDAR fusion

- **Impact:** camera-only and LiDAR-free goals are **superseded** (withdrawn). Where an older note
  proposes a LiDAR-free camera lineage, this decision wins: for example §5 of
  [`evidence/eq1d_qualification_2026-10-18/LINEAGE_DIFFS.md`](evidence/eq1d_qualification_2026-10-18/LINEAGE_DIFFS.md).
- **Also:** H-R1/H-R2/H-R3 are PROVISIONAL and not preregistered.
- **Sources:** [`SENSOR_CONTRACT.md`](SENSOR_CONTRACT.md);
  [`plans/sensor_contract_amendment_2026-10-18.md`](plans/sensor_contract_amendment_2026-10-18.md).

## D-2026-10-17-1 — Git traceability protocol

- **Decision:** verified work is committed locally. Push and merge each need separate approval.
- **Source:** [`research_git_traceability_protocol.md`](research_git_traceability_protocol.md).

## D-2026-10-17-0 — The 30 GPU-h budget is lifted

- **Decision:** design is driven by measurement quality. This **supersedes** the budget cap in the
  2026-10-14 plan.
- **Source:** WORKLOG, 2026-10-17 entries.

## D-2026-10-16-0 — The E1/E2 collision cause is TF32 in `torch.cdist`; `direct_l2_v1` is adopted

- **Impact:** historical moving cells are TF32-exposed. Qualifying the public claims is tracked as
  `motar-lp5.3`.
- **Sources:** [`plans/kroc2027_moving_target_cause_plan.md`](plans/kroc2027_moving_target_cause_plan.md);
  [`audits/moving_target_tf32_readonly_2026-10-16.md`](audits/moving_target_tf32_readonly_2026-10-16.md).

## D-2026-10-14-0 — KROC2027 plan v2: Policy F only, no new PPO

- **Decision:** Policy F (ep25000 riskcap) is the only policy, with no new PPO training. The capture
  metric `capture_0p5m` is primary.
- **Source:** [`AGENTS.md`](../AGENTS.md), the temporary KROC2027 exception.

## Earlier decisions

Decisions before October 2026 are recorded in:
- [`HISTORY.md`](HISTORY.md);
- [`discipline_review_2026-08-22.md`](discipline_review_2026-08-22.md) (standing prohibitions);
- [`cleanup/cleanup_record_2026-09-24.md`](cleanup/cleanup_record_2026-09-24.md).

## D-2026-10-19-3 — Occlusion-algorithm v1: G1-R and G4 (CPU-only U1–U7) approved

- **Authority:** user instruction 2026-10-19 (original-PDF G1-R and CPU-only G4 attachment).
- **Approved:** original 25+4 papers, preserved v1/v2 manifests and factual reading corrections;
  then CPU-only opt-in U1–U7, each locally committed. The design-delta gate precedes implementation:
  G3_REVIEW_REQUIRED permits only unaffected U1–U3. Proposed changes to07–09 remain unapplied.
- **Not approved:** G5 simulator integration/raw calibration, G6+ GPU/simulator/pilot, production
  changes, PPO, checkpoint/actor replacement, remote push or main merge.
- **Disk amendment:** the user's subsequent instruction explicitly supersedes the attachment's8GB
  floor with **3,000,000,000 bytes**. Initial startup stopped at7,975,260,160bytes; the user removed
  files and requested resume. Resume check measured9,547,649,024bytes. No agent data deletion or
  backup/bundle action was taken. Preserve PDFs, historical raw, symlinks, main WIP and stash.
- **Boundary:** G4 unit tests do not qualify performance, safety, calibration or recovery. G5 and
  above need separate user approval. Session quota deadline is unknown; conservative wrap-up17:38KST.
