# Preserved contract paths

Paths whose location or bytes are part of a research, test or provenance contract. They were not
moved, renamed or byte-edited in the 2026-09-24 reorganisation. Treat each as `KEEP_IMMUTABLE`
unless its row says otherwise. The categories below also appear in [`AGENTS.md`](../../AGENTS.md).

## Evidence and preregistrations

| Path | Contract | Enforced by |
| --- | --- | --- |
| `results/**` (every result folder, including `VOID_*`, logs, READMEs and summaries) | Receipts bind files by path and SHA-256. `results/MANIFEST.json` indexes 207 result directories with provenance links and file hashes (build of 2026-09-24) | `tools/build_result_manifest.py`, per-result receipts, result tests |
| Every preregistration (`docs/prereg_*`, `docs/preregistration_*`, `docs/*_preregistration_*`, `docs/plans/prereg_*`, `results/**/PREREGISTRATION.md`) | Fixed before measurement; cited by receipts | Receipts, `docs/research_authority_2026-08-26.json` |
| `docs/results/*` | Generated result documents | `tools/build_target_motion_result_doc.py --check`, `tools/build_visibility_reacquisition_audit.py --check` |
| `docs/*registry*.json`, `docs/status_manifest.json`, `docs/literature_quantitative_ledger_2026-09-18.json` | Machine-readable claims | `tools/check_public_docs.py`, site and figure tests |
| `results/renderer_characterization_r{1b,2b,3b,5b}_2026-09-14/PREREGISTRATION.md` | Byte-identical stubs, hash-pinned | Renderer follow-up tests |

## Path-bound documents

| Path | Why it cannot move |
| --- | --- |
| `docs/reference_platform_proposal_2026-08.md` | Cited by the provenance-frozen `navrl_ref5in_quad_config.py`; a single changed byte makes `eval_navrl_v2_density_sweep.sh` exit 2 (`tests/test_navrl_ref5in_provenance_freeze.py`) |
| `docs/prereg_2026-08-13_detector_coupling.md` | Stub; five sources cite this path. The original is in `docs/archive/` |
| `docs/archive/prereg_2026-08-14_detector_coupling_binbias.md`, `development_directions_2026-08.md`, `sim_vs_hardware_gap_2026-08.md`, `readme_9732d12_2026-09-12.md` | Archived files cited by source code or tests |
| `CRASH_TUNING_LOG.md` | Frozen 2026-08-05; three source files cite it |
| `VERIFICATION.md` | Its section before `## 역사 기록:` is parsed by `tests/test_repository_claims.py` and `tools/check_public_docs.py`; a status banner was added above it |
| `OPERATIONS.md` | §0 and §1 are parsed by `tests/test_branch_policy.py`; completed-run wording by `tests/test_repository_claims.py`; a status banner was added above it |
| `RESEARCH_PLAN.md` | §8.8 is cited as a preregistered plan by receipts and `tools/summarize_navrl_v2_causal_1to3.py` |
| `CLAUDE.md` | Its commit-rule line is read by `tests/test_branch_policy.py`; code comments cite its observation contract (now in `AGENTS.md`) |
| `WORKLOG.md` | Cited by 24 result files and one test; append-only chronology |
| `docs/plans/confirmation_phase_plan_2026-09-06.md`, `docs/plans/eth_ds5_e3_2026-09-10.md` | Parsed by `tests/test_repository_claims.py`; cited by code and results |
| `docs/plans/diagnostic_paper_plan_2026-09-05.md` | Cited by three frozen preregistrations (A1, A3, A4) |
| `docs/target_motion_algorithm_2026-09-17.md` | Its algorithm text is bound to code by `tests/test_target_algorithm_documentation.py` |
| `docs/results_overview_2026-09-12.md` | Must hold nine figures (`tests/test_public_docs_consistency.py`); linked from the README by `tests/test_status_site.js` |
| `docs/RESEARCH_EVIDENCE_INDEX.md` | Cited as evidence by `docs/repository_status_2026-09-14.json` |
| `docs/renderer_urdf_loader_v1_contract.md` | Its hash is recorded by URDF smoke runs; any edit breaks those pins |

## Site and figures

| Path | Contract |
| --- | --- |
| `docs/status/archive-2026-09-13.html` | SHA-256 pinned by `tests/test_research_overview.py`; also depends on `style.css` |
| `docs/assets/paper/manifest.json`, `docs/assets/paper/motar-paper-block-diagrams.zip` and the nine `*-block-diagram.*` files | SHA-256 pinned through the pinned manifest |
| `docs/assets/paper/overview-2026-09-13/`, `renderer-contract-v1-2026-09-14/`, `docs/assets/presentation/` | Hash-checked against their own manifests |
| `docs/assets/paper/renderer-characterization-2026-09-13/` | Must be byte-identical to commit `32ee105` (`tests/test_renderer_followup_evidence.py`) |
| `docs/status/arena.js`, `arena_motion.js`, `arena_route.js`, `arena_demo_planner.js`, `viewer.js`, `research_task_contract.json` | Frozen browser preview `GT_BROWSER_EPISODE_V1`; the viewer DOM and script order are pinned by `tests/test_research_overview.py`; the termination values by `tools/build_research_task_contract.py --check` |
| `docs/status/style.css` | Test-pinned rules; styles the archive page and the preview |
| `docs/status/status.json`, `docs/status/data/*.json` | Read by `tests/test_status_site.js` |

## Code and tools

| Path | Contract |
| --- | --- |
| `aerial_gym/config/robot_config/navrl_ref5in_quad_config.py` | Provenance-frozen; its SHA is checked by the evaluation launcher |
| `tools/probe_navrl_physical_target_braking.py`, `tools/verify_navrl_physical_target_braking.py` | Their SHA-256 is checked at runtime by `navrl_task.py` |
| `tools/benchmark_renderer_characterization.py`, `tools/render_renderer_contract_figures.py` | Their bytes are compared to recorded manifests in CI tests |
| `tools/render_paper_blocks.py`, `tools/render_rc_figures.py` | Their outputs are pinned; do not re-run in place |
| `tools/create_navrl_source_bundle.py` | Copied into every training source receipt |
| `tools/test_navrl_p3_math.py`, `tools/test_navrl_p3_stage0.py` | Cited by path from frozen simulator code |
| Everything under `aerial_gym/`, `tools/` and `resources/robots/` while a GPU run is active | The launchers compare these trees before every cell |

## Ignored but bound (never `git clean -X`)

| Path | Why |
| --- | --- |
| `aerial_gym/rl_training/rl_games/runs/` | 1.5 GB of checkpoints and logs bound by more than a thousand receipts; TensorBoard (port 6009) reads it |
| `results/**/checkpoint_snapshot.pth`, `source_snapshot/`, `detector_snapshot.pth`, raw `*.log` | Bound by receipts |
| `aerial_gym/rl_training/rl_games/train_session_logs/`, `train_source_receipts/`, repository-root `train_source_receipts/` | Cited by the authority record and training records |
| `aerial_gym/rl_training/rl_games/checkpoints_saved/` | Cited by `results/baseline_speed_axis_peak986.csv` |
| `.codex_worktrees/route_recovery_forensics` | A registered git worktree; 85 absolute paths in nine receipts |
| `aerial_gym/lms_rl_trial/` | A separate historical project cited by WORKLOG and the parameter catalogue |
| `docs/status/status.legacy.json` | Still merged by `tools/update_status_snapshot.py` |
