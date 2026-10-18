# AGENTS.md — rules for coding and research agents

This is the canonical instruction file for every agent working in this repository (Claude Code,
Codex, Cursor). `CLAUDE.md` imports it. When a rule here conflicts with an older note elsewhere, this
file wins; tell the user about the conflict.

## Before changing anything, read

1. [`README.md`](README.md): what MOTAR is.
2. [`PROJECT.md`](PROJECT.md): current intent, scope and phase.
3. [`ARCHITECTURE.md`](ARCHITECTURE.md): how the system connects, and which directories may change.
4. [`docs/EVIDENCE.md`](docs/EVIDENCE.md): what is established, with its limits.
5. [`DESIGN_SYSTEM.md`](DESIGN_SYSTEM.md): if the site or any figure is involved.

For operations and commands see [`docs/OPERATIONS.md`](docs/OPERATIONS.md); for history,
[`docs/HISTORY.md`](docs/HISTORY.md) and the bottom of [`WORKLOG.md`](WORKLOG.md).

## Research rules

- **Official sensor contract: RGB-D Camera + LiDAR, for current and subsequent research.**
  Preserve Frozen Policy F's 898-D actor and sensor fusion. Do not remove, disable or zero LiDAR,
  or pursue a camera-only conversion without a new explicit user instruction. See the current
  [`sensor contract`](docs/SENSOR_CONTRACT.md) and its linked correction of historical roadmap text.
  Evaluation-only GT/semantic provenance must not become online tracking or control inputs.
- **Never auto-start PPO training or GPU evaluation.** Every GPU run needs the user's explicit approval
  for that run and a preregistration committed before execution. No GPU work is authorised at present.
- **Never silently change a frozen experiment.** Do not edit rewards, the controller, target-motion
  definitions, termination values, observation schemas, checkpoints or safety-filter settings to
  change a recorded outcome.
- **Never rewrite a negative result.** FAIL, INCONCLUSIVE, MATERIAL_LOSS, WITHDRAWN and VOID stay as
  recorded, in their original folders.
- **Never promote NOT_TESTED, PLANNED or BLOCKED to PASS** or to any positive wording.
- **Never fabricate a missing metric.** If a quantity was not recorded, write `NOT_RECORDED`. Do not
  substitute a proxy.
- **Never change a preregistration after measurement.** Amendments go in a new, dated file that says
  what changed and why.
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
  when approved work is complete and verified.** A second per-commit approval is not required for
  that authorized scope. Separate independent research units into separate commits.
- The user's **2026-10-17 [Git Commit & Research Traceability Protocol](docs/research_git_traceability_protocol.md)**
  applies to all agents and supersedes older per-local-commit approval language in `CLAUDE.md`,
  root `OPERATIONS.md`, earlier exceptions and skills. Those frozen historical files stay intact.
  **Remote push and main merge still require separate explicit approval.** Local commits are not
  visible through GitHub until an authorized push actually succeeds.
- Honor explicit branch-specific conditional push authorization within its stated scope, after
  verifying its history/security prerequisites against the intended HEAD. The sensor-contract
  correction preserves that authorization; it does not certify that those checks have passed.
- `main` is the only long-lived branch (root `OPERATIONS.md` §0). Short-lived agent worktrees or review
  branches merge back to `main` only with approval; do not merge or delete a worktree as automatic
  session cleanup.
- Several agent sessions may work in this repository at once. An uncommitted change may belong to
  someone else, so stage only your own approved files. Never use `git add .` or `git add -A`.
  Do not reset, stash, or incorporate another person's WIP; if it conflicts, stop and report.
- Read-only audits may create and locally commit approved new reports, analysis evidence and
  appended WORKLOG entries. Existing source and raw remain unchanged. Large raw/checkpoints stay
  outside Git; commit compact records, paths, manifests and SHA-256 bindings instead.
- Preserve provenance. Do not move, rename or byte-edit a path listed under "Path contracts" below.

### Temporary exception: KROC2027 Phase 1 (ends at merge to `main` or the 2026-10-28 Simulation Freeze, whichever is first)

- Approved by the user on 2026-10-14 (KROC2027 plan v2, package P1).
- Scope: branch `kroc2027-phase1` in the short-lived worktree `.codex_worktrees/kroc2027-phase1`.
  New paths: `motar_eval/**`, `aerial_gym/task/navrl_task/kroc_eval_adapter.py`,
  `tools/kroc2027_eval.py`, `tests/test_kroc2027_*.py`, `docs/plans/prereg_kroc2027_A_v1.md`,
  `docs/plans/kroc2027_*.json`. Existing paths: `aerial_gym/task/navrl_task/navrl_task.py`,
  `aerial_gym/task/navrl_task/navrl_perception.py`, `docs/OPERATIONS.md`, `WORKLOG.md` (append only).
- Inside that scope an agent may edit, run CPU tests and make local commits without showing each
  diff first. For those local commits only, this exception takes precedence over "Show the diff"
  above and over the general commit rule in `CLAUDE.md` ("커밋/푸시 전 사용자가 diff 검토·승인한다").
- Each local commit appends its file list and CPU test result to `WORKLOG.md`. The user reviews the
  whole branch diff before any merge or push.
- Still needs explicit approval: push, merge into `main`, every GPU run, any path outside the scope.
  Path contracts still apply: a preregistration is immutable once committed.
- Still forbidden: force-push, history rewrite, `reset --hard`, deleting or staging others' files.
- A GPU run started from a worktree must set `PYTHONPATH` to that worktree and fail closed unless
  `aerial_gym` resolves inside it.

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

## While a GPU run is active

- Commit the approved source and preregistration before launch. During the run, **do not edit or
  commit any tracked file, including WORKLOG, docs or tests**. Do not create runtime files under
  `aerial_gym/`, `tools/` or `resources/robots/`; use only the run's designated output or scratch.
  Keep the tracked tree and source HEAD frozen. After the process has ended and its results are
  validated, record the results in a separate local commit.
- Do not start heavy CPU or disk work; concurrent load has changed evaluation results.

## Recording work

Every piece of work ends with a `WORKLOG.md` entry. Newest entries go at the bottom, and the entry is
written before the local commit, so that code and record land in one commit. Use this form:

- `## YYYY-MM-DD — <one-line headline>`
- The measured numbers (a table when there are two or more cells), the decision taken and the next
  concrete step, and the run folders, checkpoints and `results/` paths needed to re-derive them.
- Refuted hypotheses, stated as refuted: "hypothesis X rejected: measured Y".

Never cut the WORKLOG entry to save time; it is what makes the next session cheap.

Every completion report ends with **Git Traceability**: Branch, Base SHA, New Commit SHA, Changed
Files, Tests, Working Tree, Remote Push YES/NO, Main Merge YES/NO, and Raw/Checkpoint Integrity.
If a commit cannot be made, explain why and list every remaining uncommitted path; do not clean
it up by deleting or resetting work. Link GitHub commits/branches only after a successful push.

## Session deadline and handoff

- Record the known session start/deadline and check the time after each research unit. For a
  five-hour session, reserve approximately the last hour for verification, commits and handoff.
  If the actual quota deadline is unavailable, say so and use a conservative wrap-up point; do
  not claim access to a hidden quota timer.
- Before the wrap-up point, finish the approved unit that fits. Stop starting long work that
  cannot leave time for validation and a local commit. Never bypass a frozen GPU tree to do so.
- Before the limit, record completed units/SHAs, unresolved issues, remaining files and tests,
  any owned process still running (PID, log, output, cap and safe next action), and exactly where
  the next session resumes. Commit verified authorized work; explicitly label incomplete work.
- Follow `.claude/skills/session-handoff/SKILL.md` with this precedence: no push without explicit
  authorization, no deletion/reset of WIP, and no tracked edits during a GPU run. If a process
  cannot finish before the deadline, report its actual state rather than claiming completion.

## Commands

```bash
python -B tools/run_tests.py                            # PUBLIC_CPU: must end with 0 failures, 0 errors
python -B tools/run_tests.py --profile GPU_REQUIRED     # CUDA-only tests; reports SKIPPED_NO_CUDA without a GPU
python -B tools/check_public_docs.py
python tools/build_paper_figures.py --check
node tests/test_status_site.js && node tests/test_status_interception_episode.js
python -B tools/motar_doctor.py --profile simulator      # read-only environment inventory
```

Always use `PYTHONNOUSERSITE=1` with the `aerialgym` conda environment: a user-site numpy otherwise
shadows the pinned one. More commands are in [`docs/OPERATIONS.md`](docs/OPERATIONS.md).

## Known pitfalls

- Evaluate a density-curriculum run with `last_gen_ppo_ep_XXXX.pth`. `gen_ppo.pth` is the
  best-reward (low-density) policy and reads as about 15 % at high density.
- Warm-start with `--checkpoint <path> --max_epochs <N>`.
- HEAD defaults do not reproduce the ep25000 training contract: 14 settings differ and are env-var
  opt-in. `tools/audits/check_eval_condition_contract.py` refuses a mismatched evaluation.
- `NAVRL_MAX_VELOCITY` is both the action scale and an observation normaliser. Changing it breaks the
  frozen checkpoint's contract, so it is a retraining axis, not a free knob.
- Results from the GTX 1650 Ti evaluation host (a separate machine, `GPU4GB=1`) are never pooled with
  RTX 3070 results.
- The RTX 3070 has 8 GB; 128 environments is the evaluation default.
- RTX 50-series GPUs are incompatible with Isaac Gym Preview 4.

## Working style

- Delegate heavy reading (many files, `runs/**/*.csv`, long logs) to subagents and bring back short
  structured reports. Never load `nn/*.pth` or long logs into the main context.
- Prefer existing project skills: `.claude/skills/navrl/` (research loop, reads `VERIFICATION.md`
  first, never auto-starts training) and `.claude/skills/session-handoff/`.
  The former `.cursor/skills/research-status/` skill is archived in `docs/archive/agents/` because it
  auto-committed and pushed.
- Public documentation is in English. `VERIFICATION.md`, the root `OPERATIONS.md` and much of
  `WORKLOG.md` are historical Korean records; quote them rather than rewrite them.



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
