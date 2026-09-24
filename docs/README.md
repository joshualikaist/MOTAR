# Documentation

## Where each question is answered

| Question | Canonical file |
| --- | --- |
| What is MOTAR? | [`README.md`](../README.md) |
| What are we researching? | [`PROJECT.md`](../PROJECT.md) |
| How does the system work? | [`ARCHITECTURE.md`](../ARCHITECTURE.md) |
| How should agents modify it? | [`AGENTS.md`](../AGENTS.md) |
| What results are established? | [`docs/EVIDENCE.md`](EVIDENCE.md) |
| How do I reproduce it? | [`docs/REPRODUCIBILITY.md`](REPRODUCIBILITY.md) |
| How do I operate it? | [`docs/OPERATIONS.md`](OPERATIONS.md) |
| What happened historically? | [`docs/HISTORY.md`](HISTORY.md) |
| How should the site and figures look? | [`DESIGN_SYSTEM.md`](../DESIGN_SYSTEM.md) |

Each file has one job. If two files seem to disagree, the one named in this table is right. Report
the conflict rather than working around it.

## Start here

- [Project](../PROJECT.md): purpose, questions, scope, phase.
- [Architecture](../ARCHITECTURE.md): the closed loop and the repository layout.
- [Evidence](EVIDENCE.md): the results, their intervals and their limits.
- [Reproducibility](REPRODUCIBILITY.md): what runs without a GPU, and which data cannot be shipped.
- [Operations](OPERATIONS.md): everyday commands, authority and run rules.
- [Project page](status/index.html) and its [full evidence page](status/evidence.html).

## Research evidence

| Track | Result record | Preregistration |
| --- | --- | --- |
| Perception: measured range error | [E3-S](../results/eth_ds5_e3s_2026-09-10/README.md) | [E3-S](../results/eth_ds5_e3s_2026-09-10/PREREGISTRATION.md) |
| Perception → policy, readaptation | [P10 replication](../results/perception_p10_seed_replication_2026-09-10/README.md) | [P10](preregistration_p10_seed_replication_2026-09-10.md) |
| Safety geometry | [independent verification](../results/independent_verification_2026-09-07/README.md) | pre-specified [grid specs](specs/) under the [confirmation-phase plan](plans/confirmation_phase_plan_2026-09-06.md) |
| Target motion | [H/E0/E1/E2 result](results/target_motion_generalization_2026-09-19.md) · [visibility audit](results/target_motion_visibility_reacquisition_audit_2026-09-19.md) | [TM E0/E1/E2](prereg_2026-09-17_target_motion_complexity_e0_e1_e2.md) |
| Observation contract | [D8b](../results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md) | [D8b](preregistration_dynamic_mesh_policy_sensitivity_d8b_2026-09-13.md) |
| Renderer | [Contract v1 freeze](renderer_track_v1_freeze_2026-09-14.md) | [renderer follow-up](preregistration_renderer_followup_2026-09-14.md) |
| Published work | [relation to published systems](relation_to_published_systems_2026-09-16.md) · [quantitative positioning](literature_quantitative_positioning_2026-09-18.md) | — |
| External baseline | [Elastic Tracker B0](external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md) · [selection](external_baseline_selection_2026-09-19.md) | [draft, not executed](preregistration_external_matched_baseline_draft_2026-09-19.md) |

Machine-readable status:
- [`quantitative_positioning_registry.json`](quantitative_positioning_registry.json): headline numbers.
- [`research_status_registry.json`](research_status_registry.json): component lifecycle.
- [`status_manifest.json`](status_manifest.json): Track D.
- [`results/MANIFEST.json`](../results/MANIFEST.json): every result directory.

Paper working documents:
- [spine](paper_evidence_spine_2026-09-19.md)
- [outline](paper_outline_2026-09-19.md)
- [claim/evidence matrix](paper_claim_evidence_matrix_2026-09-19.md)
- [figure plan](paper_figure_plan_2026-09-19.md)

## Historical archive

- [Archive index](archive/README.md): superseded plans, handoffs, reviews and agent notes, with an
  old-to-new path table.
- Preregistrations stay in `docs/` under their original names (`prereg_*`, `preregistration_*`);
  they are immutable, and receipts cite their paths.
- [`WORKLOG.md`](../WORKLOG.md): the full chronology. [`VERIFICATION.md`](../VERIFICATION.md) is
  the verification ledger (Korean).
- Maintenance record: [`cleanup/cleanup_record_2026-09-24.md`](cleanup/cleanup_record_2026-09-24.md) (the
  2026-09-24 reorganisation and claim audit), [`cleanup/new_file_budget.md`](cleanup/new_file_budget.md),
  [`cleanup/inventory_2026-09-24.json`](cleanup/inventory_2026-09-24.json) (every Markdown file classified)
  and [`cleanup/preserved_contract_paths.md`](cleanup/preserved_contract_paths.md) (paths that must never move).
