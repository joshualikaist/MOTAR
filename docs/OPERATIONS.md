# Operations

How to work in this repository today. For a first reproduction from a fresh clone, start with
[`REPRODUCIBILITY.md`](REPRODUCIBILITY.md). The root [`OPERATIONS.md`](../OPERATIONS.md) is the
historical Korean operations handbook: its launch commands record what was run and when, and they
are **not** approvals to run again.

## 1. Authority — read before running anything heavy

- **No GPU evaluation and no PPO training is authorised.** Every GPU run so far had an explicit,
  single-use approval; the last one was the H/E0/E1/E2 evaluation of 2026-09-18. A new run needs the
  user's explicit approval **and** a preregistration committed before execution.
- Frozen experiments stay frozen. Do not change rewards, the controller, target definitions,
  termination values, observation schemas or checkpoints to "fix" a result.
- `python tools/check_research_authority.py --json` checks the frozen authority record
  ([`research_authority_2026-08-26.json`](research_authority_2026-08-26.json)). It needs two evidence
  files that are not tracked in git, so it passes only on the original workstation. The record also
  predates the 2026-09-14 → 09-18 boundary changes, which are recorded in WORKLOG.

## 2. Branch and site rules

- `main` is the only long-lived branch; it is both the working and the publishing branch. GitHub Pages
  serves `/docs` from `main`, so the public site is `docs/status/`. Short-lived branches (agent
  worktrees under `.codex_worktrees/`, or a review branch the user asks for) are merged back into
  `main` and then deleted. The rule and its history are in root [`OPERATIONS.md`](../OPERATIONS.md) §0,
  and `tests/test_branch_policy.py` keeps the documents consistent.
- Never force-push, `reset --hard`, or rewrite published history. Show the diff before any commit or
  push.
- Any tool that writes a GPU-derived number to `runs/` or `results/` records the execution stack with
  `tools/runtime_fingerprint.py` (root `OPERATIONS.md` §0.1). Perception streaming reproduces only in
  `detector_runs/venv`.
- The public snapshot repository `MOTAR-public` is synced from this one by a separate release step.

## 3. Environments

| Profile | Use | Setup |
| --- | --- | --- |
| CPU renderer | Public quick start, renderer and documentation tests | [`renderer_cpu_quickstart_2026-09-12.md`](renderer_cpu_quickstart_2026-09-12.md) + `requirements-renderer-cpu.txt` in a new venv |
| Docs / CI | Citation and schema checks | `requirements-public-validation.txt` in its own venv |
| Historical simulator | Everything that imports `aerial_gym` or needs a GPU | conda env `aerialgym` (Python 3.8), Isaac Gym Preview 4, `PYTHONNOUSERSITE=1` always |

Check any environment without changing it:

```bash
python -B tools/motar_doctor.py --profile renderer-cpu    # or --profile simulator
```

## 4. Everyday checks (CPU only)

Run from the repository root. None of these start a simulator or touch a GPU.

```bash
# Whole Python suite, default profile PUBLIC_CPU: hides the GPU and excludes the listed CUDA-only tests.
# It must end with 0 failures and 0 errors.
python -B tools/run_tests.py
# The CUDA-only tests (GPU_REQUIRED), and everything at once (FULL_RESEARCH, which refuses without a GPU)
python -B tools/run_tests.py --profile GPU_REQUIRED
python -B tools/run_tests.py --profile FULL_RESEARCH

# Declared public surfaces: README, links, status manifests, title and subtitle
python -B tools/check_public_docs.py            # add --schema inside the docs/CI environment

# Site and browser-preview contracts (Node 20 in CI)
node tests/test_status_site.js
node tests/test_public_status_manifest.js
node tests/test_status_interception_episode.js
node tools/validate_gt_browser_tracking.js --episode-mode interception --seeds 2 --duration 20
```

Generated documents and figures must match their sources; each builder has a `--check` mode that
changes nothing:

```bash
python tools/build_paper_figures.py --check              # docs/assets/paper/final/ (needs matplotlib)
python tools/build_quantitative_positioning_figure.py --check
python tools/build_literature_positioning_doc.py --check
python tools/build_target_motion_result_doc.py --check
python tools/build_visibility_reacquisition_audit.py --check
python tools/build_research_task_contract.py --check     # browser termination contract
```

To inspect the result index without rewriting the tracked `results/MANIFEST.json`:

```bash
python tools/build_result_manifest.py --output /tmp/motar-manifest.json --print-issues
```

## 5. Previewing the site

```bash
cd docs && python -m http.server 8000
# open http://localhost:8000/status/   (project page)  and  /status/evidence.html
```

The interactive arena on the project page is a browser ground-truth preview. It is not PPO and not
PhysX, and nothing on the page depends on it. Its episode semantics are frozen as
`GT_BROWSER_EPISODE_V1` ([freeze record](status/gt_browser_episode_v1_freeze_2026-09-18.md)); do not
change the standoff, thresholds, horizon or route resolution without a new preregistration.

## 6. Figures

All paper figures come from one generator, and every number in them is read from a canonical record:

```bash
python tools/build_paper_figures.py          # rebuild docs/assets/paper/final/
python tools/build_paper_figures.py --check  # byte-exact with the recorded matplotlib, values-only otherwise
```

The visual rules are in [`DESIGN_SYSTEM.md`](../DESIGN_SYSTEM.md). Dated figure packages under
`docs/assets/paper/` (overview, renderer characterization, renderer contract) are hash-pinned
historical records; do not regenerate them in place.

## 7. When a GPU run is authorised

These rules exist because breaking them has voided runs before (see `results/VOID_*`):

- While training or evaluation runs, do **not** edit, add or commit anything under `aerial_gym/`,
  `tools/` or `resources/robots/`. The launchers compare those trees before every cell. Edit only
  `docs/`, `tests/`, `WORKLOG.md` or a scratch directory.
- Do not run heavy CPU or disk work alongside an evaluation; concurrent load has changed results.
- Evaluate a frozen policy only through its recorded contract; `tools/audits/check_eval_condition_contract.py`
  refuses a mismatched condition. HEAD defaults do **not** reproduce the ep25000 training contract of policy F.
- TensorBoard for this repository: `tensorboard --logdir aerial_gym/rl_training/rl_games/runs --port 6009`.
- Record the run in [`WORKLOG.md`](../WORKLOG.md) with its numbers and paths before asking for review.

## 8. Housekeeping

- Safe to delete at any time: `__pycache__/` directories outside `results/`, and example images
  written by `aerial_gym/examples/`.
- **Never** run `git clean -X` or `git clean -fdX`. Ignored paths include `runs/`, checkpoint and source
  snapshots bound by receipts, a registered worktree and `aerial_gym/lms_rl_trial/`.
- Everything under `results/` is evidence, including logs and VOID folders. Do not delete it.
