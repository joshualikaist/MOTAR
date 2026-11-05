# AGENTS.md — rules for coding and research agents

This is the canonical instruction file for every agent working in this repository (Claude Code,
Codex, Cursor). `CLAUDE.md` imports it. When a rule here conflicts with an older note elsewhere
(root `OPERATIONS.md`, `VERIFICATION.md`, `CLAUDE.md`, a skill or an earlier exception), this file
wins; tell the user about the conflict. The user's current instructions and the decisions in
[`docs/DECISIONS.md`](docs/DECISIONS.md) are not such notes: they stand unless superseded there.
Rules last reviewed against current practice on 2026-10-31.
Expired exceptions are removed here and remain in git history.

## Before changing anything, read

1. [`docs/PROJECT_STATE.md`](docs/PROJECT_STATE.md): where the work stands now, and on which branch.
   Check its *Last verified* date; newer state is at the bottom of [`WORKLOG.md`](WORKLOG.md).
2. [`README.md`](README.md) and [`PROJECT.md`](PROJECT.md): what MOTAR is, its intent and scope.
3. [`ARCHITECTURE.md`](ARCHITECTURE.md): how the system connects, and which directories may change.
4. [`docs/EVIDENCE.md`](docs/EVIDENCE.md): what is established, with its limits.
5. [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md): which experiments belong together, the file to
   read first for each, and which can be compared.
6. [`docs/DECISIONS.md`](docs/DECISIONS.md): the user's standing decisions and their reasons.
7. [`DESIGN_SYSTEM.md`](DESIGN_SYSTEM.md): if the site or any figure is involved.

For operations and commands see [`docs/OPERATIONS.md`](docs/OPERATIONS.md); for history,
[`docs/HISTORY.md`](docs/HISTORY.md) and the bottom of [`WORKLOG.md`](WORKLOG.md).

## What needs the user, and what does not

The user reserves these decisions. Ask each time. Never infer approval from an earlier approval or
from a recommended next step.

- Any GPU run, training or evaluation. New PPO training is allowed again since D-2026-11-04-1
  (which superseded the ban D-2026-10-14-0); each run still needs the user's approval and a committed
  preregistration.
- Entering the next gate of a staged plan (for example G4 → G5, or R2 → R3).
- Changing Frozen Policy F or a checkpoint. Committed preregistrations, existing results, raw data
  and recorded failure evidence are never edited, even with approval (see "Research rules" and
  "Path contracts"); a correction or amendment goes in a new, dated file.
- `git push`, including `bd dolt push` and the VS Code Sync and Publish Branch buttons, and any
  merge into `main`.

Approvals are single-use and name their scope. An approval may name several runs of one unit;
anything it does not name needs a new approval.

Inside an approved unit, carry the recommended steps through to the end without stopping for
confirmation: preflight, CPU runs, analysis, records and the local commit. Report once at the end
with the results and the open decisions (the user's preference since 2026-10-16; D-2026-10-31-0).
Stop early only for a reserved decision, a failing check that cannot be resolved inside the scope,
or a host fault.

## Research rules

- **Official sensor contract: RGB-D Camera + LiDAR, for current and subsequent research.**
  Preserve Frozen Policy F's 898-D actor and sensor fusion. Do not remove, disable or zero LiDAR,
  or pursue a camera-only conversion without a new explicit user instruction. See the current
  [`sensor contract`](docs/SENSOR_CONTRACT.md) and its linked correction of historical roadmap text.
  Evaluation-only GT/semantic provenance must not become online tracking or control inputs.
- **Frozen Policy F (ep25000 riskcap) is the reference policy and stays frozen**: no actor or
  checkpoint replaces it without the user's approval. New PPO training is allowed again
  (D-2026-11-04-1, superseding D-2026-10-14-0); a newly trained checkpoint is a new policy, compared
  with F and never substituted for it. Classical baselines and planners are comparisons. A
  correction or observer is an opt-in layer that defaults to OFF. Before measuring it ON, show that
  OFF reproduces the native run.
- **Never auto-start PPO training or GPU evaluation.** Every GPU run needs the user's explicit approval
  for that run and a preregistration committed before execution.
- **Never silently change a frozen experiment.** Do not edit rewards, the controller, target-motion
  definitions, termination values, observation schemas, checkpoints or safety-filter settings to
  change a recorded outcome.
- **Never rewrite a negative result.** FAIL, INCONCLUSIVE, MATERIAL_LOSS, WITHDRAWN, VOID and ABORTED
  stay as recorded, in their original folders. A rerun writes to a new folder.
- **Never promote NOT_TESTED, PLANNED or BLOCKED to PASS** or to any positive wording.
- **Never fabricate a missing metric.** If a quantity was not recorded, write `NOT_RECORDED`. Do not
  substitute a proxy.
- **Never change a preregistration after measurement.** Fix the analysis, thresholds and stop rules
  before execution, CPU gates included. Amendments go in a new, dated file that says what changed and
  why.
- **Gates report the verdict their preregistration fixes.**
  - Use the label the preregistration or plan defines (for example GO, CONDITIONAL, NO-GO, HOLD or
    PASS), with its scope. Never upgrade it afterwards, and never relabel a historical verdict.
  - An audit check is PASS, FAIL, NOT_TESTED or LIMITED_VALIDITY, each with its measured scope.
  - Until the independent review has finished, report a gate as CONDITIONAL (or the
    preregistration's equivalent) at most.
- **Independent review before GO.** A fresh-context reviewer audits the actual files and diff
  read-only and writes only to its own audit directory.
  - Give it the code and the gate criteria, not your conclusion.
  - In Claude Code, reviewers run on Opus 5.5 (Agent `model: "opus"`).
  - Record the reviewer and the model in the report.
  - Fix the findings and review again, or report them as open.
- **Checks must be non-vacuous.** A safety or rejection check passes only if it also accepts the
  valid cases. Pair every negative test with a positive control and assert a non-zero count. A
  guard that passes by rejecting every valid input is a FAIL.
- **Host faults void a run.** If the kernel's machine-check or crash count rose during a run, its
  results are not evidence. Record the attempt as aborted, with its kernel log, and rerun it only
  after the host is fixed (see "Known pitfalls").
- **Standing prohibitions** ([`docs/discipline_review_2026-08-22.md`](docs/discipline_review_2026-08-22.md)):
  - Do not extend fixed-density PPO training to paper over a failure. This reopens only when a
    non-density bottleneck is identified and the remedy is preregistered.
  - Riskcap post-hoc tuning is allowed only as a preregistered A/B on clean re-measurement.
- **Observation contract:** ground-truth target information (semantic id or mask, bearing, range,
  `target_position`, ground-truth visibility) must never enter the actor observation. It may be used
  for detector supervision, rewards, termination, the critic and evaluation metrics only. Source
  comments refer to this as the "CLAUDE.md observation contract".
- **Numbers travel with their source.** Any number written into README, the site, a figure or
  `docs/EVIDENCE.md` must come from, and be tested against, a machine-readable record. Never type a
  new number by hand.
- **Wording limits that apply everywhere:**
  - The arc filter is the "implemented arc-clearance speed filter", not a DWA planner or a safety
    guarantee.
  - The target-motion mechanism is "consistent with" a visibility mechanism, not causal.
  - D8b causality is `NOT_TESTED`.
  - There is no real-flight claim and no state-of-the-art claim.
  - Class A matched external comparisons = 0.

## Git rules

- No force-push, no `reset --hard`, no rebase of published history, no history rewrite.
- **Show the diff, review it, stage only explicitly named approved files, and make a local commit
  when approved work is complete and verified.**
  - No second per-commit approval is needed inside the approved scope.
  - Separate independent research units into separate commits.
  - The user's 2026-10-17 [Git Commit & Research Traceability Protocol](docs/research_git_traceability_protocol.md)
    sets the sequence. It supersedes older per-commit approval language in `CLAUDE.md`, root
    `OPERATIONS.md` and skills, whose frozen text stays intact.
- **Push and merge into `main` need separate explicit approval.**
  - A branch-specific push authorization applies only within its stated scope, and only after its
    prerequisites are verified against the intended HEAD.
  - Local commits are not visible on GitHub until an authorized push succeeds.
- **Branches.** `main` is the publishing branch: GitHub Pages serves `/docs` from it. Research
  commits currently land on `codex/mt-p1` (worktree `src/MOTAR-mt-p1`) and reach `main` only by an
  approved merge. Record the branch and base SHA before editing. The frozen `CLAUDE.md` and root
  `OPERATIONS.md` §0 still name `main` as the working branch; `tests/test_branch_policy.py` pins
  that text. This conflict is known. Do not stop on root `OPERATIONS.md` §0's session-start
  `## main...origin/main` check; confirm the branch that `docs/PROJECT_STATE.md` names instead, and
  do not report the conflict again.
- Several agent sessions may work in this repository at once, so an uncommitted change may belong to
  another session. Stage only your own approved files, by name. Never use `git add .` or
  `git add -A`.
- **Worktrees.** Use a short-lived worktree for a unit when it helps. When the unit ends:
  1. Check for unique content: commits that are in no other branch, and untracked or ignored files.
  2. Check for receipts that record the worktree's absolute path.
  3. If you created the worktree for this unit and steps 1–2 find nothing, remove it. If its path
     is bound, lock it with `git worktree lock --reason "..."` instead. Removing any other worktree,
     or one with unique content, follows "Plans for broad or destructive changes".

  If a branch's unique commits are worth keeping but the branch is not, make an archive ref
  (`git update-ref refs/archive/<date>/<name> <sha> ""`) before deleting the branch. VS Code does not
  list these refs and `git push --tags` does not push them.

  On the maintainer's workstation, four worktrees are locked because receipts cite their paths:
  `braking_route_v3`, `route_recovery_forensics`, `two_envelope_recovery_integration` and
  `kroc2027-phase2`. Never unlock or remove one without a decision about those receipts.
  `src/MOTAR-mt-p1` is not locked, but it holds the current research branch, about 200 tracked
  files cite its absolute path, and its two untracked `results/` symlinks are protected (WORKLOG
  2026-10-21, G4-R7; D-2026-10-21-0). Do not remove it without the user.
- **Keep Source Control at zero.**
  - On the maintainer's workstation, raw results live untracked in the main checkout's `results/`,
    and worktrees link to them.
  - A new raw directory gets one dated line in the main checkout's display exclude
    `.git/motar_main_display_exclude` (set with `git config --worktree core.excludesFile`), not in
    the shared `.git/info/exclude`. A linked worktree with its own untracked links has its own
    `.git/worktrees/<name>/motar_display_exclude` (`MOTAR-mt-p1`, `kroc2027-phase2`).
  - Do not leave decks or documents untracked in the main checkout. Commit them on the work branch,
    or keep them outside the repository.
  - Never run `git clean -X` or `git clean -fdX`.
- **Read-only audits** may create and locally commit approved new reports, analysis evidence and
  appended WORKLOG entries. Existing source and raw remain unchanged. Large raw and checkpoints stay
  outside Git; commit compact records, paths, manifests and SHA-256 bindings instead.
- Preserve provenance. Do not move, rename or byte-edit a path listed under "Path contracts" below.

## Path contracts — never move, rename or change these bytes

| Path | Why |
| --- | --- |
| `results/**` | Immutable evidence; receipts bind files by path and SHA-256 |
| `docs/prereg*.md`, `docs/*preregistration*.md`, `docs/plans/prereg_*.md`, `results/**/PREREGISTRATION.md` | Preregistrations, cited by receipts |
| `docs/reference_platform_proposal_2026-08.md` | Cited by the provenance-frozen `navrl_ref5in_quad_config.py`; one changed byte makes `eval_navrl_v2_density_sweep.sh` exit 2 (`tests/test_navrl_ref5in_provenance_freeze.py`) |
| `docs/prereg_2026-08-13_detector_coupling.md` | Stub whose path five sources cite; the original is in `docs/archive/` |
| `CRASH_TUNING_LOG.md` | Frozen 2026-08-05; three source files cite it |
| `VERIFICATION.md`, `OPERATIONS.md`, `RESEARCH_PLAN.md`, `CLAUDE.md` | Parsed by tests or cited by receipts (for example, `RESEARCH_PLAN.md` §8.8) |
| `docs/status/archive-2026-09-13.html`, `docs/assets/paper/manifest.json`, `docs/assets/paper/motar-paper-block-diagrams.zip` | SHA-256 pinned by tests |
| Dated figure packages in `docs/assets/paper/*-2026-09-1*/` | Hash-checked against their own manifests |
| `docs/status/arena*.js`, `viewer.js`, `research_task_contract.json` | Frozen browser preview `GT_BROWSER_EPISODE_V1` |

The full list, with the checks that enforce it, is in
[`docs/cleanup/preserved_contract_paths.md`](docs/cleanup/preserved_contract_paths.md).

## While a GPU run or a guarded runner is active

- **Freeze the tree.**
  - Commit the approved source and preregistration before launch.
  - During the run, **do not edit or commit any tracked file, including WORKLOG, docs or tests**.
  - Do not create runtime files under `aerial_gym/`, `tools/` or `resources/robots/`. Use only the
    run's designated output or scratch.
  - After the process has ended and its results are validated, record the results in a separate
    local commit.
- The same freeze applies to a CPU runner with a clean-tree guard: one edit, even to WORKLOG, can
  void it.
- **Concurrent work** (D-2026-11-05-1):
  - Always forbidden while a run is active:
    - writing the run's worktree or the main checkout (the freeze above), or the run's output or scratch;
    - any CUDA use by another job (start CPU jobs with `CUDA_VISIBLE_DEVICES=""`);
    - a job without a memory cap;
    - the offlined cores (cpu4/5).
  - Always allowed: reading, the web, and writing outside the repositories (`~/motar_tmp`, scratch).
  - CPU compute during PPO training: allowed on cores disjoint from the runner (6,7,12,13 when the
    runner is pinned to 0-3,8-11,14-23), `nice -n 10`, under the conditions above. Record its window
    in the WORKLOG.
  - CPU compute during an evaluation:
    - allowed only once the load-invariance test T-LOAD (`docs/plans/prereg_tload_v1.md`) reads
      LOAD_INVARIANT, then under the same conditions, with its window recorded;
    - after DISJOINT_ONLY, allowed on the disjoint cores only;
    - otherwise forbidden.
  - T-LOAD read LOAD_INVARIANT on 2026-11-05 (review r1 PASS_WITH_LIMITS). It covers the per-episode outcome
    files of Isaac Gym evaluation cells only. Disjoint cores still slowed the cells by 15-27 %, so these keep an
    idle host:
    - any evaluation whose result includes wall-clock timing (latency gates, benchmarks, `duration_s`,
      `perf_counter` telemetry);
    - real-time SITL acceptance runs. Development runs may run alongside if their metrics are computed in
      simulator time and the real-time factor is recorded (and is 1.0);
    - evaluation paths with CPU-side decision logic that T-LOAD did not cover (classical baselines, CPU replays,
      the target route planner).
  - Why: on 2026-09-05 one loaded evaluation differed from two idle runs (S_off 22.35 % vs 21.47 %,
    mechanism not identified). On 2026-11-04 three loaded cells were byte-identical to idle twins.
- A run started from a worktree must set `PYTHONPATH` to that worktree and fail closed unless
  `aerial_gym` resolves inside it. Otherwise the editable install imports the main checkout's code
  while the worktree is the one hashed.
- **Bound every long run.**
  - Set per-process caps and use `timeout -k`.
  - Compare the kernel log's crash count before and after.
  - `multiprocessing.Pool` waits forever when a worker dies.

## Work and evidence outside the repository

- `/tmp` is cleared at reboot; a reset on 2026-10-28 deleted evidence that lived only there.
  - Keep work directories and evidence that must survive in a persistent directory outside every
    repository. On this workstation that is `~/motar_tmp/<unit>_<date>/`, listed in
    `~/motar_tmp/INDEX.md`.
  - If a script must write to `/tmp`, mirror it after each milestone with
    `~/motar_tmp/sync_tmp_mirror.sh`.
- Bind external evidence in the committed record by path and SHA-256.
- An external backup was declined (D-2026-10-19-1). Never make a backup or bundle a precondition
  for a step.

## Recording work

Every piece of work ends with a `WORKLOG.md` entry.
- Newest entries go at the bottom, separated from the previous entry by one blank line.
- Write the entry before the local commit, so that code and record land in one commit.

Use this form:

- `## YYYY-MM-DD — <one-line headline>`
- The measured numbers (a table when there are two or more cells), the decision taken and the next
  concrete step, and the run folders, checkpoints and `results/` paths needed to re-derive them.
- Refuted hypotheses, stated as refuted: "hypothesis X rejected: measured Y".

Never cut the WORKLOG entry to save time; it is what makes the next session cheap.

Keep experiment records connected and few (D-2026-10-31-1):
- **Connect every new record.**
  - In the same commit, add each new preregistration, plan, evidence folder or result folder to
    its family in [`docs/EXPERIMENTS.md`](docs/EXPERIMENTS.md), by name.
  - Or add a family with every field: *Period*, *Question*, *Policy*, *Status*, *Read first*,
    *Lineage*, *Compare with* and *Members*.
  - `tests/test_experiment_index.py` fails when a record in its scope is in no family, in two
    families, or, if first committed on or after 2026-10-31, is matched only by a wildcard.
  - The test cannot check *Lineage* order or *Status* wording. Update them by hand, and update
    *Status* whenever a family's verdict or GO/HOLD state changes.
- **One evidence folder per unit.**
  - A unit's evidence goes in one folder, `docs/evidence/<unit>_<first date>/`, with a subfolder
    per gate if needed.
  - The independent reviewer still writes to its own audit directory. The committed copy of each
    review or repair round goes into the unit's folder as a numbered file (`REVIEW_r1.md`,
    `REPAIR_r1.md`, …), not into a new folder. Q02-A's seven repair rounds and eight reviews took
    15 folders.
- **No separate handoff files.**
  - A handoff goes in the WORKLOG entry and Beads, not in a tracked `HANDOFF.md`. This also applies
    to the output of a handoff skill.
  - During a GPU freeze, write the handoff outside the repository and tell the user where.
- **Prefer extending a file to creating one.** Before adding a Markdown file, check whether the
  family's report or plan can carry the text.
- **Committed records stay where they are.**
  - Path-contract files never move.
  - Other records are not moved or merged to tidy up without a reviewed plan and the user's
    approval (*Plans for broad or destructive changes*). The index connects them in place.

Every completion report ends with **Git Traceability**: Branch, Base SHA, New Commit SHA, Changed
Files, Tests, Working Tree, Remote Push YES/NO, Main Merge YES/NO, and Raw/Checkpoint Integrity.
If a commit cannot be made, explain why and list every remaining uncommitted path. Link GitHub
commits/branches only after a successful push.

## Session deadline and handoff

Standing user instruction (2026-11-05): **every agent** (Codex, Claude Code, Cursor and delegated workers) must
close out before a five-hour session ends so another account can resume from disk. This applies without a
separate reminder. A client restart, compaction or reconnect does not create a new five-hour allowance.

- **At session start:** read the latest WORKLOG/Beads and `~/motar_tmp/agent_sessions/latest_handoff.json` if present.
  Check its project/worktree before using it; records are continuation evidence, not new execution approval.
  Record the root session clock with `python3 tools/agent_session.py start --agent <agent>`; if a real quota/user
  deadline is visible, supply `--deadline <ISO8601-with-timezone>`. Use `--started-at` for a known earlier start.
  The earlier of that deadline and five hours from the known start wins. Unknown quota timing stays explicitly
  unknown; do not invent a fresh allowance from a long-lived chat's creation time or a reconnect.
- **Default closeout schedule:** start +4h (or deadline −60min) enters WRAP_UP: no new research/implementation unit,
  training/evaluation, expensive review, or major edit. Finish the in-flight authorized unit only if validation,
  records and a local commit fit. By deadline −10min, finish the handoff and stop session work; retain this slack
  for an early quota cutoff. When a reliable remaining-usage signal indicates imminent exhaustion, close out
  earlier. The wall clock does not guarantee the provider will permit five hours of use.
- **Check time:** `python3 tools/agent_session.py status` at every turn/work-unit boundary, before a long tool call,
  and at least every ten minutes while active. Before a new unit use
  `python3 tools/agent_session.py check --minutes <measured-work> --verification-minutes <review-test-record-time>`.
  A successful clock check grants no GPU, gate, commit, push or merge approval. Budget from measured durations
  including review/tests/recording, not GPU time alone. Workers inherit the root deadline, never reset it. Pass the root worktree/clock/deadline when delegating;
  workers check that root clock with `--worktree <root-worktree>` and never close it on the parent's behalf.
- **Switching worktrees is not a new session:** keep checking the original root worktree's clock with
  `--worktree <original-root-worktree>`; record every changed worktree/branch/HEAD in the handoff lists. The
  helper's automatic Git snapshot is of the clock's root worktree only.
- **Make handoff durable throughout:** record a checkpoint after each completed unit, and before a long run, in
  WORKLOG and Beads when the tree is editable. During a GPU/guarded-run freeze, write only outside repositories
  (for example `~/motar_tmp/agent_sessions`), and give the next owner the path. Do not depend on chat memory,
  provider account history, `.codex`/`.claude` caches, or files under `/tmp` alone. Never store credentials.
- **Before stopping:** commit verified authorized changes locally, with named files and Git Traceability; do not
  auto-push/merge, automatically revert work, delete evidence, or mark unfinished work complete. Inventory owned
  processes with PID/owner, command, log/output, limits, state, existing continue authorization and safe next action.
  Preserve approved ongoing jobs with an explicit process handoff; do not kill another agent's jobs or a training
  run merely to make a clean-looking ending. Preserve source freeze and report incomplete paths/tests honestly.
- **Account-neutral closeout:** write a JSON summary outside the repository, with `completed`, `remaining`,
  `running_processes`, `pending_approvals`, `validation` (lists) and exactly one concrete `next_action` (string:
  file/command plus required gate). Run `python3 tools/agent_session.py close --handoff <summary.json>`; it captures
  worktree/branch/base/HEAD/dirty paths and publishes an immutable session handoff plus the shared latest pointer.
  Reflect this in WORKLOG/Beads when editable. No tracked HANDOFF.md or second task tracker is created.
- **Next account:** read that handoff, WORKLOG and Beads; verify branch/HEAD/process state and pending approvals,
  then resume the single recorded action. `start` reuses an existing clock. `--new-window` is used only after the
  previous session closed and a genuinely new account/window is established; otherwise retain the earlier deadline.
  An explicit user change to the schedule overrides it. A closed session is not permission to reopen a frozen gate.

The helper saves clocks outside Git trees, serializes per-worktree updates and shares records across provider
accounts. It does **not** wake an idle model or close an application by itself. The agent performs the time checks
and wrap-up while working; no guaranteed background timer or hidden quota access is claimed. Follow the project
session-handoff skill under these rules, including the source-freeze, evidence-preservation and authorization rules.

## Plans for broad or destructive changes

Before a cleanup, deletion or migration beyond your own clean unit worktree and files, or any
other change that is hard to undo:
1. Write the plan with exact commands.
2. Have it reviewed independently.
3. Run it only after the user approves.

The script records the state before and after, refuses to overwrite its records, and can resume
after a crash. Example: the 2026-10-31 repository cleanup (`WORKLOG.md`).

## Commands

```bash
python -B tools/run_tests.py                            # PUBLIC_CPU: must end with 0 failures, 0 errors
python -B tools/run_tests.py --profile GPU_REQUIRED     # CUDA-only tests; reports SKIPPED_NO_CUDA without a GPU
python -B tools/check_public_docs.py
python tools/build_paper_figures.py --check
node tests/test_status_site.js && node tests/test_status_interception_episode.js
python -B tools/motar_doctor.py --profile simulator      # read-only environment inventory
journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"   # host crash count, before and after long runs
```

Always use `PYTHONNOUSERSITE=1` with the `aerialgym` conda environment: a user-site numpy otherwise
shadows the pinned one. More commands are in [`docs/OPERATIONS.md`](docs/OPERATIONS.md).

## Known pitfalls

- **Host faults (since 2026-10-28 17:33).** Under all-core load, the maintainer's workstation logs CPU
  machine-check errors (corrected, bank 0, mostly on one P-core) and random segfaults.
  - Until the user reports the host fixed, check the crash count before and after every long run.
  - Prefer few workers.
  - Report the state instead of silently rerunning.
- **TF32 in distance checks.** rl_games enables TF32 in every training and evaluation process.
  - `torch.cdist` uses a matmul when either point set has more than 25 rows. On the RTX 3070
    (torch 2.4.1) that erred by about 2 m with a batch of at least 2 and a bar count divisible by 4,
    such as 160 bars. The pattern came from one kernel scan and is not a safe-density whitelist:
    other densities also showed nonzero error
    ([audit](docs/audits/moving_target_tf32_readonly_2026-10-16.md)).
  - When a GPU result disagrees with a CPU replay, or a failure appears at one density only, suspect
    TF32 first.
  - Use exact difference norms for clearance checks.
- **PUBLIC_CPU runs all tests in one process.**
  - A helper that stubs `aerial_gym` in `sys.modules` or copies `NAVRL_*` variables into
    `os.environ` leaks into every later test.
  - Run production code inside `tools/audits/eq1_common.production(env)`.
  - Confirm a new test file with the full suite, not only in isolation.
- Evaluate a density-curriculum run with `last_gen_ppo_ep_XXXX.pth`. `gen_ppo.pth` is the
  best-reward (low-density) policy and reads as about 15 % at high density.
- Warm-start with `--checkpoint <path> --max_epochs <N>`.
- HEAD defaults do not reproduce the ep25000 training contract: 14 settings differ and are env-var
  opt-in. `tools/audits/check_eval_condition_contract.py` refuses a mismatched evaluation.
- `NAVRL_MAX_VELOCITY` is both the action scale and an observation normaliser. Changing it breaks
  Policy F's checkpoint contract, so it is a retraining axis, not a free knob.
- Results from the GTX 1650 Ti evaluation host (a separate machine, `GPU4GB=1`) are never pooled with
  RTX 3070 results.
- The RTX 3070 has 8 GB; 128 environments is the evaluation default.
- RTX 50-series GPUs are incompatible with Isaac Gym Preview 4.
- **Git and shell on the maintainer's workstation.**
  - git 2.25 prints no `locked` line in `git worktree list --porcelain`; test for
    `.git/worktrees/<name>/locked` instead.
  - Scripts that compare sorted lists set `LC_ALL=C`, because the default locale sorts differently.
  - The stash stack is shared by every worktree, so prefer a WIP commit on your own branch.

## Working style

- Talk to the user in Korean. Public documentation is in English. `VERIFICATION.md`, the root
  `OPERATIONS.md` and much of `WORKLOG.md` are historical Korean records; quote them rather than
  rewrite them.
- Delegate heavy reading (many files, `runs/**/*.csv`, long logs) to subagents and bring back short
  structured reports. Never load `nn/*.pth` or long logs into the main context.
- Prefer the project skills `.claude/skills/session-handoff/` and `.claude/skills/navrl/`. Use only
  the navrl skill's read-only diagnosis and delegation steps: it is a training loop, and the
  `VERIFICATION.md` it treats as execution authority is not an approval.

## Task tracking, memory and state documents

- **Beads (`bd`) holds actionable work.**
  - At session start, read the `bd prime` hook output and `bd ready`.
  - Claim an issue before working on it.
  - Close it only when its acceptance criteria are met and the evidence exists.
  - File new work as an issue under the matching epic.
  - Commands are in [`docs/OPERATIONS.md`](docs/OPERATIONS.md) §9.
- **[`docs/PROJECT_STATE.md`](docs/PROJECT_STATE.md) is the short current-state index.** Update it only
  when the state really changes, and keep it near 120 lines.
- **[`docs/ROADMAP.md`](docs/ROADMAP.md) holds direction.** Update it only when a milestone or the
  direction changes.
- **[`docs/DECISIONS.md`](docs/DECISIONS.md) holds decisions.** Add important decisions there, and mark
  replaced ones superseded instead of deleting them.
- **[`WORKLOG.md`](WORKLOG.md) stays the append-only research log.**
- **Claude-Mem and the user's auto-memory are reference memory.** Code, tests and committed records
  win over them.
- **Precedence over the managed Beads block below.** The rules of this file and the user's
  instructions take precedence over that generated block, which says so itself. In particular:
  - keep using the user's auto-memory files as they are;
  - no `git pull --rebase`, because rebase is forbidden here;
  - no `git push` and no `bd dolt push` without explicit approval.
- **Do not edit inside the `BEGIN/END BEADS INTEGRATION` markers by hand.** `bd setup claude` manages
  them.

<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:1105d646 -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/core-concepts/sync-concepts.md for details and anti-patterns.

## Agent Context Profiles

The managed Beads block is task-tracking guidance, not permission to override repository, user, or orchestrator instructions.

- **Conservative (default)**: Use `bd` for task tracking. Do not run git commits, git pushes, or Dolt remote sync unless explicitly asked. At handoff, report changed files, validation, and suggested next commands.
- **Minimal**: Keep tool instruction files as pointers to `bd prime`; use the same conservative git policy unless active instructions say otherwise.
- **Team-maintainer**: Only when the repository explicitly opts in, agents may close beads, run quality gates, commit, and push as part of session close. A current "do not commit" or "do not push" instruction still wins.

## Session Completion

This protocol applies when ending a Beads implementation workflow. It is subordinate to explicit user, repository, and orchestrator instructions.

1. **File issues for remaining work** - Create beads for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **Handle git/sync by active profile**:
   ```bash
   # Conservative/minimal/default: report status and proposed commands; wait for approval.
   git status

   # Team-maintainer opt-in only, unless current instructions forbid it:
   git pull --rebase
   git push
   git status
   ```
5. **Hand off** - Summarize changes, validation, issue status, and any blocked sync/commit/push step

**Critical rules:**
- Explicit user or orchestrator instructions override this Beads block.
- Do not commit or push without clear authority from the active profile or the current user request.
- If a required sync or push is blocked, stop and report the exact command and error.
<!-- END BEADS INTEGRATION -->
