# Archive

Superseded plans, handoffs, reviews and notes. Nothing here is current guidance: current intent is
[`PROJECT.md`](../../PROJECT.md) and current evidence is [`docs/EVIDENCE.md`](../EVIDENCE.md).
Nothing here was deleted or edited, except that relative links in moved files were updated so they
still resolve. Git history (`git log --follow <path>`) traces every move.

## Layout

| Folder | Contents |
| --- | --- |
| (top level) | The 2026-08-20 consolidation: planning, review, presentation, handoff and early preregistration documents |
| [`agents/`](agents/) | The retired Cursor `research-status` skill |
| [`execution/`](execution/) | Expired run instructions |
| [`perception/`](perception/) | Superseded perception planning |
| [`plans/`](plans/) | Superseded paper and project plans |
| [`platform/`](platform/) | Dormant hardware-platform procurement notes (ref5in, 2026-08-23) |
| [`renderer/`](renderer/) | Superseded renderer and appearance plans |

## Moved on 2026-09-24

`WORKLOG.md` and some older notes mention these by their old path as plain text; use this table to
find them.

| Old path | New path | Why archived |
| --- | --- | --- |
| `docs/paper_evidence_outline_2026-09-18.md` | [`plans/paper_evidence_outline_2026-09-18.md`](plans/paper_evidence_outline_2026-09-18.md) | Superseded by `paper_outline_2026-09-19.md` and the paper spine |
| `docs/execution_plan_2026-08-23_detection_range_stage1.md` | [`execution/execution_plan_2026-08-23_detection_range_stage1.md`](execution/execution_plan_2026-08-23_detection_range_stage1.md) | Run instruction for a stage that ended `RANGE_INCONCLUSIVE_AT_THIS_BUDGET` |
| `docs/plans/perception_next_plan_2026-09-09.md` | [`perception/perception_next_plan_2026-09-09.md`](perception/perception_next_plan_2026-09-09.md) | Superseded by the P10 and S4 results |
| `docs/plans/appearance_overhaul_2026-09-11.md` | [`renderer/appearance_overhaul_2026-09-11.md`](renderer/appearance_overhaul_2026-09-11.md) | The renderer track froze at Contract v1 on 2026-09-14 |
| `docs/navrl_frame_selection_2026-08-23.md` | [`platform/navrl_frame_selection_2026-08-23.md`](platform/navrl_frame_selection_2026-08-23.md) | Hardware track dormant (no airframe assembled) |
| `docs/navrl_ref5in_bom_direction_2026-08-23.md` | [`platform/navrl_ref5in_bom_direction_2026-08-23.md`](platform/navrl_ref5in_bom_direction_2026-08-23.md) | Hardware track dormant |
| `docs/navrl_ref5in_carrier_screen_2026-08-23.md` | [`platform/navrl_ref5in_carrier_screen_2026-08-23.md`](platform/navrl_ref5in_carrier_screen_2026-08-23.md) | Hardware track dormant |
| `docs/navrl_ref5in_component_screen_2026-08-23.md` | [`platform/navrl_ref5in_component_screen_2026-08-23.md`](platform/navrl_ref5in_component_screen_2026-08-23.md) | Hardware track dormant |
| `docs/navrl_ref5in_vendor_request_2026-08-23.md` | [`platform/navrl_ref5in_vendor_request_2026-08-23.md`](platform/navrl_ref5in_vendor_request_2026-08-23.md) | Hardware track dormant |
| `.cursor/skills/research-status/` | [`agents/cursor-research-status/`](agents/cursor-research-status/SKILL.md) | Edited a removed `app.js` and auto-committed and pushed to a retired branch; conflicted with `AGENTS.md` |
| `CLAUDE.md` (contents) | `git show 7fb547d:CLAUDE.md` | Replaced by `AGENTS.md`; `CLAUDE.md` is now a pointer |
| `docs/README.md` (contents) | `git show 7fb547d:docs/README.md` | Replaced by a compact documentation map |
| `tools/test_navrl_p3_smoke.py` | [`tools/archive/test_navrl_p3_smoke.py`](../../tools/archive/test_navrl_p3_smoke.py) | Asserts the retired 156-D observation lineage; cannot pass against the 898-D contract |
| `tools/test_navrl_p3_stage1.py` | [`tools/archive/test_navrl_p3_stage1.py`](../../tools/archive/test_navrl_p3_stage1.py) | Asserts the retired 1265/1273-D lineage. Its sibling `tools/test_navrl_p3_stage0.py` stays, because frozen simulator code cites it |
| `tools/audit_navrl_v2_placer_latency.py` | [`tools/archive/audit_navrl_v2_placer_latency.py`](../../tools/archive/audit_navrl_v2_placer_latency.py) | Left out of the 2026-08-27 density freeze; its output was never committed |

The dormant hardware contracts `docs/navrl_ref5in_payload_packaging_contract_2026-08-23.md` and
`docs/navrl_ref5in_thrust_stand_protocol_2026-08-23.md` stay in `docs/`: they are contracts, and
tests or receipts may cite their paths.

## Archived, but still cited by source code — do not move

| File | Cited by |
| --- | --- |
| `prereg_2026-08-13_detector_coupling.md` | Five sources cite the *un-archived* path; a stub at `docs/prereg_2026-08-13_detector_coupling.md` keeps them working (restored 2026-09-05) |
| `prereg_2026-08-14_detector_coupling_binbias.md` | `eval_navrl_v2_detector_coupling_binbias.sh` |
| `development_directions_2026-08.md` | `env_object_config.py` |
| `sim_vs_hardware_gap_2026-08.md` | `tools/generate_platform_spec.py` |
| `reference_platform_proposal_2026-08.md` | Original of the byte-frozen `docs/reference_platform_proposal_2026-08.md` |
| `readme_9732d12_2026-09-12.md` | `tests/test_public_docs_consistency.py` (the pre-release README) |

## Contents of the 2026-08-20 consolidation

| File | Contents |
| --- | --- |
| `RESEARCH_PLAN_v2_history.md` | Former `RESEARCH_PLAN.md` §8.1–8.22 (v2 205 bars, TTC, riskcap) |
| `ref5in_audit_and_next_steps_2026-08-13.md` | ref5in audit summary (absorbed into `VERIFICATION.md`) |
| `codex_review_2026-08-10.md`, `codex_review_2026-08-12.md` | Independent review records |
| `review_brief_2026-08-10.md`, `review_brief_2026-08-12_*.md` | Verification briefs |
| `CLAUDE_PPT_REVIEW_REQUEST_VERIFICATION5B_2026-08-13.md` | Presentation review request |
| `NEXT_WEEK_HANDOFF_2026-08-10.md`, `development_directions_2026-08.md` | Old execution handoffs |
| `sim_vs_hardware_gap_2026-08.md` | Simulation versus hardware gap |
| `midterm_summary_2026-08.md` | Mid-term summary. **It contains outdated riskcap numbers; do not cite it** |
| `presentation_followup_2026-08-14.md` | Presentation follow-up note |

The earlier index listed `GENSPARK_PPT_BRIEF_*` files; those are not in this repository.

## Historical documents kept in place

Many dated documents under `docs/` are historical, but their paths are cited by source code, tests,
receipts or preregistrations, so they stay where they are. The full classification of every Markdown
file is in [`docs/cleanup/inventory_2026-09-24.json`](../cleanup/inventory_2026-09-24.json).
Examples:

- The large root documents `VERIFICATION.md`, `OPERATIONS.md`, `RESEARCH_PLAN.md`, `CRASH_TUNING_LOG.md`
  and `WORKLOG.md`.
- Dated plans in `docs/plans/`.
- Handoffs such as `docs/HANDOVER_2026-09-06.md`.
- The superseded evidence index `docs/RESEARCH_EVIDENCE_INDEX.md` and the Tracks A–D results
  overview `docs/results_overview_2026-09-12.md`.
