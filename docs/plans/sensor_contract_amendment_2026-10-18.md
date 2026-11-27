# Official sensor-contract correction — 2026-10-18

**Authority:** the user's explicit correction that MOTAR's current and subsequent system is
**RGB-D Camera + LiDAR**. The user did not direct LiDAR removal or a camera-only transition.
Earlier assistant assumptions that such a transition was a final goal are withdrawn. This is a
requirements/roadmap correction, not an amendment to any historical experiment's design or results.

Reviewed parent: `183a333bd01c0124a95ae379dba0a8509aebc999`, `codex/mt-p1`.
The [current sensor contract](../SENSOR_CONTRACT.md) and
[v2 correction plan](correction_lineages_plan_v2.md) now govern future work. Their scope is based
on this user instruction, not an inferred approval in the older plans.

## Explicit supersession inventory

Locations below refer to the parent above. Preserve their bytes and Git history. Candidate
comparisons remain historical discussions; they no longer imply a pending user decision, an
authorized conversion, or a final camera-only goal.

| Original record / location | Historical content and current correction |
| --- | --- |
| [EQ-0 provisional hypotheses](../evidence/eq0_environment_audit_2026-10-17/PROVISIONAL_HYPOTHESES.md), sensor section, lines 60–76 | Withdraw the assumed long-term camera-only direction and modality choice. Preserve the provisional hypothesis discussion and actual Frozen F sensor analysis. Apply the fusion restatement in the current contract. |
| [EQ-0 audit](../evidence/eq0_environment_audit_2026-10-17/EQ0_ENVIRONMENT_AUDIT.md), lines 139–142 | Withdraw camera-only as an unresolved user goal. Continue qualifying RGB-D depth, fusion and hardware realism within the official multimodal system. |
| [EQ-1 validation plan](../evidence/eq0_environment_audit_2026-10-17/EQ1_VALIDATION_PLAN.md), lines 10–15 | Sensor modality is fixed to RGB-D + LiDAR. Ideal-versus-measured depth, camera stabilization and 3D flight remain distinct qualification questions; they do not require reopening modality selection. |
| [EQ-0 issue register](../evidence/eq0_environment_audit_2026-10-17/ISSUE_REGISTER.md), lines 93–96 | Withdraw the future camera-only premise. Preserve the need to qualify range/depth provenance and prevent unauthorized online GT. |
| [EQ-1 sensor candidates](../evidence/eq1_validation_2026-10-18/SENSOR_CANDIDATES.md), title and lines 23–58 | Retain the assistant's RGB/RGB-D/fusion comparison as history. Withdraw its role as a pending modality decision, a camera-only recommendation, or a step toward removing LiDAR. |
| [EQ-1D lineage differences](../evidence/eq1d_qualification_2026-10-18/LINEAGE_DIFFS.md), section 5, lines 65–70 | The proposed LiDAR-free perception/navigation lineage is unrequested and is not pursued. Other calibration/measurement findings remain. |
| [EQ-1D report](../evidence/eq1d_qualification_2026-10-18/REPORT.md), line 160 | The trailing camera-based roadmap does not authorize a modality change; withdraw any LiDAR-free interpretation. |
| [Correction plan v1](correction_lineages_plan_v1.md), lines 56–57 | Explicitly withdraw preparation of a LiDAR-free lineage and the single-RGB versus RGB-D choice. v2 retains the fusion-system corrections and separates A/B/C/D authorization. |
| [WORKLOG](../../WORKLOG.md), lines 20632 and 20686 | Supersede the assumed long-term camera-only candidate and later camera-lineage order with the appended correction. Leave all previous entries intact. |

## Terms that remain valid

Do not mechanically replace every occurrence of camera-only or LiDAR-only:

- The camera-only validity flag in the immutable
  [task-diagnostics preregistration](../preregistration_task_diagnostics_t1_2026-09-14.md), lines
  35–37, is a measurement channel beside fused validity, not a robot sensor configuration.
- LiDAR-only continuation in the [EQ-1 report](../evidence/eq1_validation_2026-10-18/REPORT.md),
  lines 156–157, describes a measurement-source diagnostic within the fusion system.
- [RESEARCH_PLAN.md](../../RESEARCH_PLAN.md) historical ablations, monocular range-proxy work and
  the camera-only pixel-mask description in
  [parameters.json](../status/data/parameters.json) do not establish a current modality goal.
- EQ-0's distinction between synthetic metric RGB-D, RGB-only inference and actual hardware
  remains technically relevant. RGB-D sensor depth is an allowed measurement; unqualified oracle
  depth or target GT is not an allowed online substitute.

## Research and approval consequences

S-L1 aims to improve **existing Camera + LiDAR fusion accuracy and reliability**. This user message
confirms only S-L1A production approval (obstacle depth and target-like carve-out). S-L1B pinhole/SE(3),
S-L1C false association/identity/covariance and S-L1D silhouette/validity remain planned and require
separate approval. No production file is modified in this documentation unit.

H-R1, H-R2 and H-R3 are restated in the current contract under RGB-D + LiDAR and remain
**PROVISIONAL — NOT PREREGISTERED**. H-R3 comparisons must share RGB-D + LiDAR sensing capabilities,
permitted information access and predictor, with matched pre-intervention state and sensor history.
Post-intervention measurements may diverge as control changes viewpoint. OCC-P1/P2 data remain
exploratory/design support for these new algorithm questions, not reused confirmatory tests.
The existing OCC RQ1/RQ2/RQ3 are not H-R1/2/3.

No historical OCC/EQ/E-L0 numerical outcome, source SHA, raw manifest or preregistration is amended.
Existing failed calibration/association fixtures and limited hardware validity remain recorded.
Later EQ-1D task-level firewall/RMS PASS and E-L0 instrumentation PASS retain their tested scope;
this correction neither revokes them nor extends them to untested hardware or future algorithms.

The user's conditional research-branch push permission is preserved, subject to its previously
approved history/security checks. The older WORKLOG readiness entry is not fresh clearance of
this HEAD or its full unpublished history. No push is performed by this documentation task, and
main merge remains prohibited. No new GPU run, PPO training or Recovery Planner is performed.

## Validation and preservation

The [validation record](sensor_contract_amendment_2026-10-18.validation.json) binds the parent,
protected source/evidence/raw/checkpoint checks and the documentation verification results.
Historical files in the supersession table must remain byte-identical to the parent; WORKLOG
must retain its complete original prefix. Current AGENTS/PROJECT/ARCHITECTURE entry points link
the new authoritative documents so agents do not inherit the superseded roadmap.

Raw/manifests are preserved separately from these compact Git records. Existing untracked raw
symlinks do not become tracked through a merge. External backup has not been established by this
task; same-disk preservation and SHA verification are not an external backup.
