# H-R3 P02 evidence index

**NO-GO for confirmatory readiness.** GPU12/12 completed; analysis integrityFAIL. H-R3 remains
PROVISIONAL. 246loss triggers/203matched;56geometric candidates,14failed primary prefixes.
No subset is salvaged as a causal effect. No additional GPU or seed is launched.

- [Original result report](REPORT.md), [machine decision](decision.json), [original FAIL analysis](analysis.json).
- [Independent Fable5.1 focused review](FABLE_REVIEW.md), [scope/receipt](review_receipt.json),
  [response and corrections](REVIEW_RESPONSE.md), [review source checks](review_source_checks.json).
- [Read-only failure localization](main_failure_audit.md) and [reproduction script](main_failure_audit.py).
- [Next validation plan — not executed](NEXT_VALIDATION_PLAN.md), [confirmation HOLD](CONFIRMATION_HOLD.json).
- [CPU96 focused tests](cpu_validation.json), [preservation](preservation.json),
  [main run](run_report.json), [main raw manifest](RAW_MANIFEST.json), [review manifest](review_RAW_MANIFEST.json).
- [Passing N16 smoke](smoke_analysis.json); [failed earlier smoke](smoke01_failed/ASSESSMENT.md).
- [Actual changed-file list](CHANGED_FILES.txt), [preliminary history/publication audit](publication_history_audit.json).

Runtime changes are only new opt-in P02 evaluation modules: RNG/reset wrappers, B4/B5 arms,
detector-spatial scorer, launcher/supervisor, analysis and planning. Production aerial_gym,
URDF/controllers/detectors/adapters, PolicyF actor898, RGB-D+LiDAR, P01, historical raw, main WIP3
and stash are unchanged. Six focused P02 test modules and new plans/evidence accompany them.
Raw/checkpoint payload stays outside Git; symlinks/merge do not track it. External backup remains
NOT_ESTABLISHED. No main merge/push, force/history rewrite, PPO or Planner work.
