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
- **Show the diff and wait for the user's approval before any commit or push.** Do not commit
  autonomously.
- `main` is the only long-lived branch (root `OPERATIONS.md` §0). Short-lived agent worktrees or review
  branches merge back to `main` and are deleted.
- Several agent sessions may work in this repository at once. An uncommitted change may belong to
  someone else, so stage only your own files.
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

## While a GPU run is active

- Do not create, edit or commit anything under `aerial_gym/`, `tools/` or `resources/robots/`. The
  launchers compare those trees before every cell. Work only in `docs/`, `tests/`, `WORKLOG.md` or a
  scratch directory.
- Do not start heavy CPU or disk work; concurrent load has changed evaluation results.

## Recording work

Every piece of work ends with a `WORKLOG.md` entry. Newest entries go at the bottom, and the entry is
written before asking for diff review, so that code and record land in one commit. Use this form:

- `## YYYY-MM-DD — <one-line headline>`
- The measured numbers (a table when there are two or more cells), the decision taken and the next
  concrete step, and the run folders, checkpoints and `results/` paths needed to re-derive them.
- Refuted hypotheses, stated as refuted: "hypothesis X rejected: measured Y".

Never cut the WORKLOG entry to save time; it is what makes the next session cheap.

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
