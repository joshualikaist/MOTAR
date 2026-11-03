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
    and the symlinks are all preserved. (WIP/stash clause superseded by D-2026-10-31-0.)
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
- **Source:** the temporary KROC2027 Phase 1 exception, formerly in [`AGENTS.md`](../AGENTS.md)
  (added in `01c5b19`). It expired at the 2026-10-28 freeze and was removed on 2026-10-31; its text
  is in `git show 5f5a768:AGENTS.md`.

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
  backup/bundle action was taken. Preserve PDFs, historical raw, symlinks, main WIP and stash. (WIP/stash clause superseded by
  D-2026-10-31-0.)
- **Boundary:** G4 unit tests do not qualify performance, safety, calibration or recovery. G5 and
  above need separate user approval. Session quota deadline is unknown; conservative wrap-up17:38KST.

### D-2026-10-19-3 amendment — U4 target miss versus hard failure

The user explicitly confirmed that a braking-prior error above10% is **LIMITED_VALIDITY**;
when it remains within25%, subsequent CPU units may proceed. Retain the τ_v=.4 prior, record
both criteria and do not claim10% agreement. Above25% still fails the model-form gate. This
clarifies G4 execution only; no raw calibration, simulator or GPU authority is added.

The usage-limit interruption left G1-R f103721 and U1 dc2b6d3 committed; U2 had no source files.
Resume20:51KST Oct19, actual quota deadline UNKNOWN, conservative wrap-up00:51KST Oct20. Disk
stop remains3GB. All still-authorized CPU work resumes without a new GPU/push/merge permission.


## D-2026-10-21-0 — G4-R4 controlled Git state and independent nominal safety

- **Authority:** user G4-R4 instruction, received2026-10-20UTC/2026-10-21KST work completion.
- **Research entry point:** [BOIA Master Plan](research/occlusion_algorithm_v1/BOIA_RESEARCH_MASTER_PLAN.md)
  for current direction; it preserves old evidence/specs instead of overwriting them.
- **Git:** source8689efcd preserved; verified research/P1 changes committed separately from Git
  administration/master docs. Main WIP3/stash/Raw/results/checkpoints/symlinks are protected. (WIP/stash clause superseded by
  D-2026-10-31-0.)
  Only169verified nonRawCPython caches receive anchored local info/exclude rules; no Raw hiding.
- **Safety:** utility failure must not erase a valid current backup. Independent env/episode/decision/
  candidate/version/time/model/provenance and complete-prefix/backup bindings are now required,
  including F. Booleans only veto; all qualification is nominalCPU, not physical certification.
- **Execution:** R0/R1+docs only. R2 onward, online/GPU/Isaac/learning/push/merge remain unapproved;
  G5HOLD. Current instruction supersedes earlier PART C no-auto-local-commit only for this scope.
- **Evidence:** [R1 report](evidence/boia_g4r4_2026-10-20/R1_REPORT.md),
  [Git audit](evidence/boia_g4r4_2026-10-20/GIT_HYGIENE_REPORT.md).

## D-2026-10-31-0 — Cleanup bans removed from AGENTS.md and the traceability protocol

- **Decision:** the user removed the rules against deleting a worktree as session cleanup, against
  resetting, stashing or incorporating another person's WIP, and against deleting, resetting or
  hiding work to produce a clean status. Worktree and branch lifecycle rules are in
  [`AGENTS.md`](../AGENTS.md) ("Git rules"). Push, merge into `main`, path contracts and staging by
  name are unchanged.
- **Alternatives:** keep the rules and approve each cleanup as a one-off exception, as PLAN_v4 did.
- **Supersedes:** the WIP/stash preservation clauses of D-2026-10-19-1, D-2026-10-19-3 and
  D-2026-10-21-0. The main WIP is kept as `refs/archive/2026-10-31/main-wip` and the stash as
  `refs/archive/2026-10-31/stash-20261015` (local refs, not pushed).
- **Also recorded:** the user's 2026-10-16 instruction to carry recommended steps through and report
  once at the end ("이제부터는 너가 자동으로 나머지 과정까지 다 한다음에 나 불러주면 좋겠어": from now
  on, do the remaining steps yourself and call me when done). `AGENTS.md` states it under "What
  needs the user, and what does not"; reserved decisions still need the user.
- **Reversal:** the user restores the rules.
- **Sources:** the user's instructions of 2026-10-31 ("agents.md파일에 그런 내용을 삭제해", "2번 규칙도
  삭제해도 될거같아"); [`WORKLOG.md`](../WORKLOG.md) 2026-10-31.

## D-2026-10-31-1 — Experiment records are connected by an index, not reduced by deletion

- **Decision:** the user chose to keep every committed Markdown record in place and add
  [`EXPERIMENTS.md`](EXPERIMENTS.md), which puts every experiment record in its scope (result,
  evidence and research folders, plans, specs, audits, and top-level preregistrations and result
  notes) in exactly one family. `tests/test_experiment_index.py` checks that each record in that
  scope is in exactly one family and that a record first committed on or after 2026-10-31 is named
  literally.
  `AGENTS.md` ("Recording work") limits future growth: one evidence folder per unit with numbered
  review and repair rounds inside it, and no separate handoff files.
- **Why:**
  - Of 574 tracked Markdown files on 2026-10-31, 277 were immutable evidence, preregistrations or
    path-bound files, and 13 were canonical documents.
  - Tests, code or machine-readable records cite 247 of the other 284 by path.
  - At most 37 more could have been moved or deleted without breaking a test, code or record
    binding. Some of those are still cited by immutable Markdown, such as the B1-R draft, which
    `plans/prereg_b1r_classical_baseline_v1.md` binds by SHA-256.
  - Tracked Markdown files grew from 381 to 574 in October (the base is `2da5b12`, the last
    commit before 2026-10-01 KST).
- **Alternatives:** also move the at most 37 unbound files to `docs/archive/` (a separate plan,
  review and approval); delete records (rejected, because it breaks receipts and tests).
- **Reversal:** the user asks to remove the index or the growth rules.
- **Sources:**
  - the user's question of 2026-10-31 ("md파일 같은게 엄청나게 지금 많잖아. 줄일 필요가 있어 …
    비슷한 실험같은 경우는 연결성 있게 확인해보는게 중요하지 않겠어?": there are far too many
    Markdown files, they need reducing, and similar experiments should be checked together), and
    their choice of the recommended option;
  - [`WORKLOG.md`](../WORKLOG.md) 2026-10-31, the experiment-index entry;
  - the inventory `~/motar_tmp/docs_consolidation_20261031/inventory.csv` (SHA-256
    `412a6df6bd6a3ac66ea598c26e6cff62d72a3c06b22af81329a9152242749a5d`), made by `inventory.py`
    (`6d3d1f23cebf17a72dff84b3e6a5b6eebf8c6340dcbb654e40b98bbf86dc97e6`).

## D-2026-11-04-0 — C-Planner charter: hybrid track, review scope, approvals, no BIOS update

- **Decision:** the user approved the C-Planner roadmap of 2026-11-03 and its decisions D1–D8, fixed in
  [`plans/cplanner_charter_v1.md`](plans/cplanner_charter_v1.md):
  - D1: the KRoC 2027 manuscript does not include C-Planner results.
  - D2: hybrid track. A predictive planner (C-OFF) runs with the static-bar certificate as a logged
    label; the certificate filters execution (C-FILTER) only after the G-alpha gate.
  - D3: independent reviews use the charter's scope (cooperative caller, trust model A of
    2026-10-29). In-process adversarial Python and false adapter declarations are recorded
    limitations, never P0-blocking.
  - D4: development on branch `cplanner/dev` with local commits; no push, no merge.
  - D5: CPU work of milestones M1–M3 proceeds through its gates without intermediate approvals; each
    verdict is reported; a FAIL stops at the stop rule.
  - D6: a DEV GPU envelope (eval-only, Frozen F unchanged, no PPO, at most 16 environments and 2 GPU-h
    in total, results labelled DEV, a committed preregistration per run).
  - D7: eval-only instrumentation (`collision_free_capture`, per-step control logs), opt-in and OFF by
    default, with F and B-PP re-measured for the matched comparison.
  - D8: the user declined the BIOS update ("호스트 복구 안해도 될거같은데?"). It is replaced by pinning
    C-Planner processes off cores 2, 3 and 6, at most 12 workers, voiding a run on any kernel fault
    record, and duplicate determinism checks (charter §7).
- **Why:** 0 of 12 C-Planner components are connected to MOTAR data; the largest unknowns (certifiable
  rate on real 72x4 scans, compute, competitiveness with B-PP and F) are cheapest to remove in order.
  The P0 review loop (v1–v7, P0-R, r1) repeatedly stopped on in-process adversarial findings outside
  the chosen trust model. A first matched comparison cannot meet the KRoC dates (2026-11-13,
  2026-11-26).
- **Alternatives:** certificate-centric Track A first (no closed-loop number for 6–11+ weeks); a
  practical predictive planner without the certificate (no method candidate left to test); BIOS update
  before any run (declined).
- **Reversal:** the user changes the track, the review scope or the approvals; a G-alpha result below
  20 % pauses the certificate as a filter by the charter's own rule.
- **Sources:** the user's messages of 2026-11-03 ("이거 관련해서, 다음 계획을 어떤거를 세워야되고 …")
  and 2026-11-04 ("오케이 다른건 괜찮은데, 호스트 복구 안해도 될거같은데? … 다 진행해. D세션");
  `~/motar_tmp/cplanner_roadmap_20261103/ROADMAP_PROPOSAL.md`.
