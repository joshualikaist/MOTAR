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

- **Superseded on 2026-11-04** by [D-2026-11-04-1](#d-2026-11-04-1--new-ppo-training-allowed-again-gpu-runs-approved-for-control-gain-randomization):
  new PPO training is allowed again. The text below is kept as recorded.
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

## D-2026-11-04-1 — New PPO training allowed again; GPU runs approved for control gain randomization

- **Decision:**
  - The ban on new PPO training (D-2026-10-14-0) is lifted.
  - The user approves the GPU runs of the control-gain-randomization unit (GR), both evaluation and
    training.
  - Frozen Policy F (ep25000 riskcap) is not changed. Any newly trained checkpoint is a new policy,
    compared with F and never substituted for it.
- **Scope and conditions:**
  - Every GPU run still has a preregistration committed before launch, a fail-closed runner (worktree
    `PYTHONPATH` guard, per-process caps, kernel fault record before and after) and an independent
    review before any positive label.
  - Gain randomization is an opt-in switch with default OFF. Before measuring it ON, OFF must be shown
    to reproduce the native run.
  - Push and merge still need separate approval.
- **Alternatives:** keep F as the only policy and test robustness by evaluation only. That is the first
  step of GR (GR-0) in any case. Training follows only if the measurement motivates it.
- **Reason:** every Policy F result so far comes from one airframe (`navrl_quad`) with fixed controller
  gains (`randomize_params = False`, midpoint gains). The 2026-08 adaptation to the 1.2 kg ref5in
  airframe failed its gates (`policy-r-ref5in` in [`EXPERIMENTS.md`](EXPERIMENTS.md)). The user wants
  to know whether the method holds on other airframes and controller tunings.
- **Impact:** AGENTS.md's research rules already say that the ban holds "unless the user reverses
  that decision"; this entry is that reversal. GR work happens on branch `gainrand/dev` (worktree
  `src/MOTAR-gainrand`).
- **Reversal:** a user decision.
- **Sources:** the user's message of 2026-11-04 ("새 ppo 금지 없애도 되고, gpu 승인은 마음껏 내가 해줄게.
  교정하려는 작업 하면서 병행으로 먼저 control gain randomization 수행 가능할까?": the new-PPO ban can be
  removed, GPU approvals will be given freely, and the user asks whether control gain randomization can
  run first, in parallel with the C-Planner V1 repair). The same message's earlier part chose to repair
  the C-Planner V1 direction gap rather than accept it as tolerance (recorded on `cplanner/dev`).

## D-2026-11-04-3 — Camera-only comparison study: LiDAR may be removed in an opt-in research condition

- **Decision:**
  - A study may compare the current RGB-D camera + LiDAR system with the camera alone. It measures what difference
    appears when the LiDAR is not used.
  - For this comparison only, the LiDAR may be removed in an opt-in condition. The condition defaults to OFF and is
    selected only through the study's own entry point.
  - Frozen Policy F and the official RGB-D + LiDAR sensor contract (`docs/SENSOR_CONTRACT.md`) are unchanged.
  - The user accepted the plan's recommendations K1-K4 and K6. For K5 they judged a native-resolution camera arm
    hard. The plan separates camera quality from field of view by removing the LiDAR by bearing instead.
- **Scope and conditions:**
  - Work happens on branch `camonly/dev` (worktree `src/MOTAR-camonly`).
  - The CPU switch and its leak tests (C0) may proceed.
  - Every GPU stage (C1 identity, C2 zero-shot conditions, C3 training, C4 matched evaluation) needs a
    preregistration committed before launch and the user's approval of that stage.
  - The pinned `navrl_task.py` and `navrl_perception.py` stay byte-identical.
  - Push and merge need separate approval.
- **Alternatives:**
  - Keep the LiDAR in every option and study only depth fidelity. That was the first draft, which the user
    corrected.
  - Add a native-resolution camera arm. The user judged it hard: it changes the observation statistics and GPU
    memory.
- **Reason:** the user wants to know what the LiDAR contributes, and whether a camera-only system can work.
- **Impact:**
  - AGENTS.md and the sensor contract forbid removing the LiDAR "without a new explicit user instruction". This
    entry records that instruction and its scope.
  - D-2026-11-04-2 (the C-Planner V1 repair) is recorded on `cplanner/dev`.
- **Reversal:** a user decision.
- **Sources:** the user's messages of 2026-11-04:
  - "기존의 우리 방식은 카메라와 라이다를 둘 다 쓰는건데, 라이다를 쓰지 않았을때 어떠한 차이가 나타나는지를 보고
    싶어서 그런거야 … 방향성 수정 부탁할게": the existing method uses both camera and LiDAR; they want to see what
    difference appears without the LiDAR, and asked for the direction to be corrected.
  - "오케이 일단 진행해. 그렇게. 해상도 문제를 따로 분리하기는 어려울 것 같은데. 너도 한번 생각해봐": proceed as
    proposed; separating the resolution problem seems hard; think about it too.

## D-2026-11-05-1 — Concurrent CPU work alongside GPU runs: a definition replaces the blanket ban

- **Decision:** the one-line rule "Do not start heavy CPU or disk work alongside an evaluation" is replaced by
  AGENTS.md "Concurrent work":
  - **Always forbidden while a run is active** (these routes are certain):
    - writing the run's worktree, the main checkout or the run's output;
    - CUDA use by another job;
    - an uncapped job;
    - the offlined cores.
  - **CPU compute during PPO training:** allowed on disjoint cores, recorded.
  - **CPU compute during an evaluation:** allowed only after the load-invariance test T-LOAD reads LOAD_INVARIANT
    (DISJOINT_ONLY: disjoint cores only; otherwise still forbidden).
- **Reason:** an evaluation can change under another process's load only through a shared resource or a
  timing dependence inside it.
  - Code reading (2026-11-05) found no clock-dependent logic in the evaluation path. The two
    `time.perf_counter()` sites are timing telemetry.
  - It cannot exclude thread-timing effects (PhysX CPU workers, `num_threads = 10`).
  - The evidence is mixed:
    - against: one loaded evaluation differed in 2026-09 (A8 `S_off` 22.35 % vs 21.47 % twice; mechanism not
      identified; that run was also the first cell after three training runs);
    - for: three loaded cells on 2026-11-04 were byte-identical to idle twins
      (`gainrand/dev` `352b919`, `load_overlap/`).
  - T-LOAD settles it at a stronger load than either.
- **Alternatives:**
  - keep the blanket ban: it costs idle CPU time during every run;
  - allow everything now: one unexplained difference stands against it.
- **Impact:**
  - AGENTS.md "Concurrent work" and "What needs the user" (the PPO wording now cites D-2026-11-04-1).
  - docs/OPERATIONS.md sections 1 and 7.
  - The same rule text is copied to `gainrand/dev`.
- **Reversal:** a user decision, or a T-LOAD reading other than LOAD_INVARIANT (then the evaluation clause stays
  closed).
- **Outcome and narrowing (2026-11-05, T-LOAD review r1 MAJOR-1):** T-LOAD read LOAD_INVARIANT; review r1 was
  PASS_WITH_LIMITS.
  - The evaluation clause applies to the per-episode outcome files of Isaac Gym evaluation cells.
  - These keep an idle host: wall-clock timing results (latency gates, benchmarks, durations), real-time SITL runs
    and CPU-side decision paths T-LOAD did not cover. The reason is that disjoint cores slowed cells by 15-27 %.
- **Sources:** the user's messages of 2026-11-05:
  - "gpu 작업이랑 근데 cpu작업이랑 같이 하면 무슨 문제가 있어? … 그건 프로세스상 문제 될 일이 없잖아. 자 그 부분은
    확실하게 정의해야돼": what is wrong with running CPU work alongside GPU work; process-wise there should be no
    problem; define it precisely.
  - "동시 실행 정의 난 괜찮아. 반영해도 돼. 비었는데 굳이 안쓸 이유가 있나?": the definition is fine, apply it;
    why leave an idle CPU unused.
  - "AGENTS.md가 아직 예전 PPO 금지(D-2026-10-14-0)를 현행으로 적고 있어 문구 수정이 필요합니다. -> 수정해.": fix the
    stale PPO wording.


## D-2026-11-05-SESSION — All-agent five-hour closeout and account-neutral handoff

- **Decision:** Codex, Claude Code, Cursor and delegated workers must proactively close out a five-hour session without another reminder. Begin wrap-up at4h, finish handoff and stop by4h50; an earlier visible quota/user deadline wins. Restart/compaction/worktree switches never reset the allowance.
- **Reason:** the user wants to switch accounts and immediately resume the actual implementation from disk instead of reconstructing a lost conversation.
- **Implementation:** shared AGENTS session policy, corrected session-handoff skill and Cursor always-apply pointers across the five active worktrees; tools/agent_session.py records the outside-repository clock and immutable handoff. Known Git paths, verified/incomplete work, owned process details, validations, pending approvals and one next command are preserved in WORKLOG/Beads and the account-neutral handoff.
- **Scope/limits:** this is a cooperative agent procedure and on-demand clock, not an application timer/guaranteed quota API. Existing long-job/source-freeze/approval rules remain. No auto-push, auto-revert, job kill or gate promotion. Locked historical worktrees and evidence/checkpoints are untouched.
- **Alternatives:** Codex-only memory, implicit chat timestamps or a forced process-kill timer; none reliably preserves a transferable, truthful project state.
- **Reversal:** explicit user schedule change.
- **Source:** user2026-11-05: "코덱스만의 규칙은 아니고, 전체 에이전트들한테 규칙을 세우고 싶은데, 5시간 세션 끝날때 쯤 알아서 정리해서 끝내줘. 그래야 내가 다른 계정으로 바로 구현하고 있었던거 이어서 하지."
