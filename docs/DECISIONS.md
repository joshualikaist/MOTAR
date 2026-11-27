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

## D-2026-11-05-2 — Dev08 skip/late tolerance fixed at 0.15 s

- **Decision:** the user selects `skip_max_s = 0.15` seconds (150 ms) for the dev08 observed-prefix diagnostic.
  The subsequent scoped execution approval must carry this value for both `CLOCK_SKIP` and `RUNNER_LATE`,
  fixed before the smoke and retained across native/candidate members of the proposed unit.
- **Basis:** explicit user choice after the runtime-stack handoff presented the timing-only CPU measurements
  and Fable's analysis. The benchmark rule's recorded conclusion that 0.10 s suffices after the one-thread fix
  remains as measured; this decision selects 0.15 s without changing that result.
- **Scope and conditions:** this records the tolerance decision only. Amendment4's one-compute-thread setting
  remains in force. Review of `821706d` remains pending and `motar-5md` remains open. CLI entry activation,
  the two-episode smoke and the main measurement still require a separate scoped approval; measurement is
  NOT_RUN and live assumptions are NOT_VERIFIED.
- **Alternatives:** the measured rule's 0.10 s; Fable recommended 0.15 s from the earlier dev07 conditions.
- **Reversal:** a new explicit user decision, recorded before execution; committed preregistrations and
  measurements retain their original bytes.
- **Sources:** the user's 2026-11-05 instruction, "skip_max_s : 0.15s 로 지정 부탁할게";
  [amendment4](plans/prereg_sitl_dev08_candidate_gains_v1_amendment4_2026-11-05.md);
  [Fable analysis](evidence/sitl_2026-11-05/dev08/runtime_stack/ANALYSIS_skip_bound_fable.md).

## D-2026-11-05-3 — Complete dev08 through main measurement in a new five-hour session

- **Decision:** the user authorizes the essential CPU fixes, guarded CLI, source review, two native smoke
  episodes and forty main members of the existing dev08 observed-prefix-v1 schedule, then analysis, independent
  result review and local records. This supersedes the earlier pending smoke-only execution request; retain the
  original combined schedule and stop before main if a smoke fails. No additional permission question is needed
  inside this approved unit. Failed attempts remain preserved; no outcome-dependent rerun/substitution follows.
- **Fixed conditions:** Frozen Policy F, the twenty scenes and admitted target prefixes, native/candidate gain
  maps and pair order remain unchanged. `skip_max_s=0.15` and one compute thread remain fixed. Amendment5 is
  written/reviewed/committed before launch; the execution approval binds its final clean source commit.
- **Research preference:** prioritize informative experiments after the core execution/recording fixes; carry
  explicitly documented minor coverage limits forward. Reports explain the problem, its effect, the proposed
  repair and the remaining work rather than only reporting issue IDs or PASS labels.
- **Session:** the user explicitly starts a new five-hour work window at2026-11-05 13:45:22 UTC. Wrap-up17:45:22,
  finish-by18:35:22 UTC. This is a user-defined work window, not a claim about hidden provider quota.
- **Scope limits:** development diagnostic only; no GPU, later formal gate, flight, push or main merge.
- **Sources:** user2026-11-05: "안전한 연구보다는 모험심 많은 연구를 지향하고 싶어"; "오케이. 본 측정까지 하고
  나한테 말해. 얼마나 걸려?"; "지금부터 새 5시간 작업 세션으로 진행";
  [amendment5](plans/prereg_sitl_dev08_candidate_gains_v1_amendment5_2026-11-05.md).

## D-2026-11-06-0 — Diagnose existing failures and repair formal-validation preparation

- **Decision:** the user requests priority diagnosis of the existing four dev08 failure/counterpart logs,
  a report/judgment on control response, justified repairs and environment preparation toward formal validation.
  This authorizes the CPU audit, additive model/readiness implementation, tests/reviews and local records.
- **Implementation scope:** preserve Frozen F, old raw/preregistrations and all historical launchers/assets.
  Prepare an opt-in external model with the training collision box, keeping native OFF byte-identical.
  Do not choose gains from the old bundled contrast or infer EKF causality from terminal snapshots.
- **Boundary:** no new GPU measurement, full-horizon input regeneration, live diagnostic or formal stage is
  silently included in the consumed D-2026-11-05-3 approval. The new follow-up plan makes the geometry-only
  intervention and S3a numerical controls concrete; each new execution unit needs its scoped reviewed approval.
  S3a remains NOT_EQUIVALENT and complete state0..600 input coverage remains blocked.
- **Clock:** continuation retains the known13:45:22UTC start/18:35:22UTC finish-by. An early closed helper record
  is preserved; separate continuation metadata uses the identical dates, with no new time allowance.
- **Source:** user2026-11-06KST: "공식 검증 들어갈 정도까지의 환경 세팅좀 마무리하고, 제어 응답에서 어떤 이상이
  있었는지 보고 및 판단해서 수정해줘."
- **Record:** [AGQ preparation](evidence/sitl_2026-11-05/dev08/agq/README.md),
  [next-unit proposal](plans/sitl_agq_followup_v1_2026-11-06.md).

## D-2026-11-06-1 — Real vehicle without LiDAR: camera-only lineage on the OAK-D Pro, FOV-keeping reward

- **Decision (user, 2026-11-06).**
  - The real vehicle carries no LiDAR. Its sensors are a Luxonis OAK-D Pro: a stereo depth pair (HFOV 80° / VFOV
    55°, baseline 7.5 cm) and an autofocus IMX378 RGB camera (HFOV 66° / VFOV 54°).
  - The simulated camera follows the RGB camera's field of view ("use this as the reference").
  - Because the camera is the only sensor, the reward for keeping things in the camera's view is raised.
  - Motor specifications are to be randomized around a nominal small-motor model.
  - The real vehicle weighs about 0.5 kg.
- **Implementation.**
  - A new camera-only policy lineage, CAM-K1, continues from Frozen Policy F with `no_lidar`, a 66° × 54° camera,
    `yaw_align_weight` 0.6 and `visibility_bonus` 0.03 (prereg `docs/plans/prereg_camonly_k1_train_v1_2026-11-06.md`).
  - Policy F stays frozen as the reference.
  - The 0.5 kg vehicle, motor randomization, the stereo depth model (minimum depth, z² noise, holes) and separate
    RGB/depth renders come in later, separately preregistered units.
- **Boundary.**
  - This changes the target sensor set of the real-vehicle line. The RGB-D + LiDAR contract of D-2026-10-18-0 and
    AGENTS.md still governs Policy F's records and comparisons. `SENSOR_CONTRACT.md` and the AGENTS.md research-rule
    text are not edited here; that revision is open.
  - Each GPU run still needs its approval. The CAM-K1 training was approved on 2026-11-07, conditional on its
    review.
- **Source:** user 2026-11-06 KST:
  - "라이다를 못쓰게 되서 ppo학습 리워드 튜닝부터 다시 해야될 것 같은데";
  - "카메라 모델은 OAK-D Pro 이고, 모터 스펙은 작은 모터야. 기본 스펙으로 해야될거같은데";
  - "카메라 밖에 센서가 없으니까 FOV 리워드 값을 조금 더 많이 줘야 될거야. 실기 무게는 500g정도".
- **Record:** plan draft `~/motar_tmp/camonly_plan_20261106/PLAN.md` (Claude Fable 5.1);
  [CAM-K1 evidence](evidence/camonly_2026-11-04/camk1/README.md).

## D-2026-11-08-1 — The real vehicle: 1.1 kg, 5-inch props, T-Motor Velox 2550KV (replaces "about 0.5 kg")

- **Decision (user, 2026-11-08 about 16:10 KST, relaying their senior).**
  - "드론은 무게 1.1kg, 프로펠러 5x4.3x3 // 모터 velox 2550kV. 이정도로 코덱스 주면 어느정도는 맞출 수 있을거에요".
  - The user added: "이에 맞춰서 데이터 다시 수집하면 될거같아".
- **What it changes:**
  - D-2026-11-06-1's "about 0.5 kg" is superseded. Its camera (OAK-D Pro) and no-LiDAR parts stand.
  - CAM-K2 v1 (`62e2699`, T/W 2.25 cap for a 0.5 kg placeholder) is WITHDRAWN_BEFORE_RUN.
  - CAM-K2 v2 (`docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md`) models the real vehicle's thrust-to-weight on
    navrl_quad at run time, by raising the per-motor limit to T/W 4.0. The robot lineage is unchanged.
- **T/W 4.0 is INFERRED.** LIGPOWER's V2306 V2 2400KV with T5143S on 4S gives 1,084 g at 100 %, so
  4 × 1.084 / 1.1 = 3.94. 2550KV adds RPM, and bench thrust overstates thrust in flight.
- **Open, for the user:** the battery cell count (4S assumed), the wheelbase (about 225 mm assumed), whether 1.1 kg
  includes the battery, and any thrust-stand data.

## D-2026-11-13-1 — Camera-only real system for outdoor flight; sky fix required, no GO while obstacles may be missed

- **Decision (user, 2026-11-13).**
  - "MOTAR는 OAK-D 카메라만 사용하는 Camera-only 시스템입니다. LiDAR(Mid-360 포함)는 없으며, LIO도 사용하지 않습니다.
    추가 센서 장착을 전제로 설계하지 마세요."
  - "하늘 오류는 실외 비행 전 반드시 해결해야 합니다." No outdoor flight GO while a dangerous obstacle could be missed
    or ground/height estimation is uncertain.
  - Keep the 40 × 40 m box, the 3 m virtual ceiling, the geofence and the wind/braking terms. Height and ground come
    from the camera plus the flight controller's estimate only, refusing fail-closed.
  - Order: code → synthetic tests → real OAK-D data → SITL. Do not change the agreed camera-only architecture.
- **Scope.** The real-flight runtime (`tools/camera_*`, `ros2/motar_camera_policy/`). It does not change the
  repository's research sensor contract for Policy F (AGENTS.md *Research rules*).

## D-2026-11-15-1 — Stop synthetic patching of the outdoor profile; next are real OAK-D Pro recordings

- **Decision (user, 2026-11-15, option A):** "A로 진행해줘. oakd-pro 로 카메라 진행해주면 될 것 같아." Fix the clearly
  fixable items once, then stop synthetic patching and validate on real OAK-D Pro recordings.
- **Why:** three independent reviews (r1-r3) failed the near-obstacle criterion. A single-frame depth-0 pixel has no
  range, so every synthetic sky rule trades phantom walls against misses; only real data can set the thresholds.
- **State:** outdoor GO not given; residuals are listed in WORKLOG 2026-11-18 and `OUTDOOR_REVIEW_r3.md`.

## D-2026-11-18-1 — Real-system facts from the TA: VIO instead of GNSS, Orin NX, PX4 1.14, forest site

- **Source (TA answers relayed by the user, 2026-11-18).** Recorded as given; items marked unconfirmed need a reading
  from the vehicle.
  - **Camera:** "아마 OV9782일건데, 정확한건 드론 켜서 데이터를 읽어보긴 해야합니다." OV9782 is the colour option of the
    OAK-D Pro W (Luxonis: 150/127/79.5° D/H/V, 1280 × 800, global shutter); the standard OAK-D Pro colour sensor is
    the IMX378. **Unconfirmed.** If confirmed, the adapter's 12 MP IMX378 pipeline does not apply.
  - **Mount (nominal, not calibrated):** 8 cm forward and 3 cm up from the vehicle centre, facing straight ahead.
  - **Companion:** Jetson Orin NX already mounted, JetPack version unknown, software ROS 1 (ROS 2 via Docker is
    acceptable to the TA), DepthAI installable, USB3 available.
  - **Flight controller:** PX4 about 1.14, exact version unknown. The TA can reflash; MOTAR asks for exactly v1.14.3
    (the px4_msgs binding and SITL version).
  - **Localization:** no GNSS in the target environments (indoor, forest). An onboard camera + IMU VIO is already set
    up. Which camera it uses is not yet known.
  - **Vehicle:** 250 class (replaces the 225 mm wheelbase assumption of D-2026-11-08-1); flight time about 10 min.
  - **Site and safety:** a forest-like area beside the playground in front of the mechanical engineering building,
    on campus. In the TA's experience no flight report is needed on campus. A licensed person holds the RC. Wind is
    not considered for sheltered sites. A manual log flight is not possible right away.
  - **Recording:** OAK-D data are recorded on the drone's Orin and copied off.
- **What it changes:**
  - The outdoor geofence and estimator checks run on the VIO local frame, not GNSS.
  - The site is the outdoor profile's hardest case (trunks, thin branches).
- **Open:**
  - which camera the VIO uses (one DepthAI host connection per device);
  - the Orin-PX4 link (MAVROS?) and a free port for uXRCE-DDS;
  - the exact camera, JetPack and PX4 versions;
  - existing ULogs;
  - the target object;
  - the ground-station link;
  - whether the site lies in a prohibited zone. Since September 2026 an unapproved flight there is a criminal
    offence, so check the official map before the first flight.

## D-2026-11-18-2 — Follow-up facts from the user: the VIO shares the OAK-D, no earlier logs, target is another drone

- **Source (user, 2026-11-18, after D-2026-11-18-1).** This entry supersedes the matching "Open" items of
  D-2026-11-18-1; the rest of that entry stands.
  - **Camera:** "카메라는 일단 oakd라니까". Proceed with the OAK-D as given. The lens variant is not a blocker: the
    field recorder reads the sensor at recording time and the converter picks the preset.
  - **VIO camera:** "vio도 동일하다고". The onboard VIO uses the same OAK-D.
  - **PX4:** the firmware can be set to exactly v1.14.3.
  - **Earlier flight logs:** none exist. The first ULog comes from the next flight.
  - **Target:** another drone.
- **What it changes:**
  - A DepthAI device accepts one host connection, so the VIO is stopped while the field recorder runs.
  - In flight the policy cannot open the device itself. It must take RGB and depth from the streams of the VIO-side
    camera driver. Which streams that driver publishes is not yet known.
  - The policy's target detector is the trained red-colour rule. An unmarked drone is not detected by it; a red
    marker on the target or a separate detector integration is needed before any tracking trial.
- **Still open (asked of the TA):** the ROS topic list with the VIO running and a free PX4 port for uXRCE-DDS, the
  in-flight monitoring link, and a ULog from the next flight.

## D-2026-11-23-1 — Camera-only policy from scratch (CAM-S1); the flight of 2026-11-24 is manual with recording

- **Source (user, 2026-11-23).** "빨간 공만 찾게 하지 말라고 했잖아. 상대 드론을 동일한걸로 만들어서 인식하는거라고
  했잖아." / "학습 시켜야된다고 하면 첨부터 다시 해야지." / "지금 학습량을 조금 줄여서 하는게 답이지 않을까? ...
  처음부터 학습 시작해." Then, answering the questions: the 898-D layout kept; curriculum gates lowered with a stop
  rule; about 12,000 epochs, evaluation the next afternoon; the 2026-11-24 flight manual with camera and flight-log
  recording. The user also asked for an independent check and for a mathematical check of the reward by Fable.
- **Decision.**
  - A camera-only policy is trained from random initialisation (CAM-S1,
    `docs/plans/prereg_camonly_s1_scratch_2026-11-23.md`), not continued from F or CAM-K2, because a continuation
    inherits a LiDAR-trained policy's weights and habits.
  - Recognising the other drone is a perception task (an image detector), not a policy input: the policy receives a
    track. In simulation the object stays the analytic proxy; the P9 error injector is used, and recorded as a
    detection-loss placeholder (design review r1 P1-3), not as drone recognition.
  - No autonomous flight on 2026-11-24. The flight runtime cannot load a CAM-S1 checkpoint, has no blind cap and still
    uses the red-colour detector (design review r1 P0-F).
- **Supersedes:** nothing. D-2026-11-04-3 (camera-only study) and the official RGB-D + LiDAR contract stand.

## D-2026-11-23-2 — Stop CAM-S1 run 1; retrain only after the full vehicle information

- **Source (user, 2026-11-23 ~20:03).** "오늘 밤 학습하는거 어차피 오늘 확인 못하는거 이러한 정보들 다시 다 받은 다음에
  했으면 하는데?" and "원래 하던 학습 꺼."
- **Why.** The vehicle-side session measured the real camera the same evening: OAK-D-PRO-FF (colour IMX378, not the
  Pro W / OV9782 that D-2026-11-18-1 recorded as "probably"); the VIO-side driver streams 400p depth 640 x 400
  (77.6 x 53.3 deg, right-mono frame, not aligned to colour) and colour 1280 x 720 (63.9 x 38.7 deg); indoor depth is
  invalid in 30-39 % of pixels per frame with 12-13 % flicker and a 0.315 m minimum range. The training assumed one
  66 x 54 deg camera for colour and depth, 800P depth (f*B 66.19 px m versus the real 29.86), 2 % iid holes and the
  `blob_near` rule that turns dense invalid depth into a 0.4 m surface. PX4 is v1.14.3 with EKF2 on VIO only (height
  from VIO). Copy: `~/motar_tmp/cams1_20261123/vehicle_facts/README.md`.
- **Decision.** CAM-S1 run 1 stopped at epoch 849 (ABORTED_BY_USER, no result). The next camera-only training is
  designed after the outdoor depth recording, the stereo and colour stream settings, the hover thrust (ULog) and the
  latency budget are known.
- **Supersedes:** the run scope of `prereg_camonly_s1_scratch_2026-11-23.md` (its single run is used up); D-2026-11-18-1's
  camera-sensor guess.

## D-2026-11-23-3 — CAM-S2a tonight; simulated target is the same quadrotor; capture rule; runtime on ROS 1

- **Source (user, 2026-11-23 night).** "시뮬레이션 할때도 빨간공이 아니라 같은 기체로 실험해야되는거 까먹지 말고";
  approval of the target, detection, capture and latency rules; "비행 높이는 약 1.5m"; "A안으로 비행 프로그램 수정해야지";
  "학습 코드를 지금 얼른 짰어야지. 내일 아침에 실행되는거 봐야된다니까".
- **Decision.**
  - The simulated target is the quadrotor (supersedes D-2026-11-23-1's "the object stays the analytic proxy").
  - Capture = centre distance <= 1.0 m, closing speed <= 1.0 m/s, target detected; < 0.40 m is a collision.
  - Train CAM-S2a tonight with the rules that exist (`docs/plans/prereg_camonly_s2a_2026-11-23.md`); the rest
    (body-fixed camera, ground, forest, PX4 lag, ego noise, measured invalid depth) follows in CAM-S2b.
  - The flight runtime is ported to ROS 1 (rospy) + MAVROS (option A).
- **Supersedes:** D-2026-11-23-1's sentence on the simulated object.

