# G9 additive candidate review r2

Date: 2026-11-05 KST. Reviewer: Codex child `/root/g9_resume_audit`; exact model identifier unavailable. This is a source audit, not an Opus GR-1b results review.

Reviewed: `/home/fair/motar_tmp/g9_resume_20261105/sitl_g9_lee_steps_v2.py` (manifest SHA-256 `0ee6b1f50172448f6be06b106ce51a578a38f3afe1c73732cfc050bd57244741`), `candidate.diff`, `candidate_manifest.json`, and original G9 preregistration `MOTAR-camonly/docs/plans/prereg_sitl_reference_g8g9_v1.md`.

**Reading: CONDITIONAL preparation; not ready for execution until guard issue 1 is addressed.** This review launched no tests, compute jobs, or CUDA use and made no repository changes.

## Remaining findings

1. **MEDIUM — host-fault collection does not fail closed.** Candidate lines 68-69 ignore `journalctl` return code and stderr. If journal retrieval fails, empty stdout becomes fault count zero; line 233 can then emit `RECORDED_REVIEW_REQUIRED` without kernel evidence. Use `subprocess.run(..., check=True)` with current explicit byte decoding; propagate the diagnostic failure. A wrapper can supply equivalent checked fault evidence, but it should not silently treat collection failure as zero.

2. **MEDIUM integration condition — INVALID output exits successfully.** Lines 233-238 write an `INVALID` result but return success. A queue that relies only on process return code may advance. Require the wrapper to parse `validity` before advancing, or preserve the result and exit nonzero for `INVALID`. Contact/clearance should not become an acceptance gate absent a dated preregistered rule.

3. **LOW — timestamp/derived-frame sampling remains a historical limitation.** Candidate lines 166-177 sample world velocity after `em.simulate()` while body/vehicle orientation and body angular velocity were updated before the physics step (`base_multirotor.py:287-305`, `env_manager.py:392-401`). The x/y step segments have nominal constant yaw, so their transform error is small; do not claim exact instantaneous post-step body/yaw response. This is inherited from the original runner. Preserve metric formulas for the repair, and document the sampling convention if using G9 as a precise PX4 latency reference.

## Resolved findings / scope checks

- Lines 31-40 pin the 40x40x3 m legacy, no-distractor, single-bar scene before config imports and reject conflicting inherited values. Lines 128-130 verify one actual asset.
- Lines 133-144 write root-state views, submit to PhysX and refit Warp. Derived controller state is refreshed before control at the first physics step.
- Lines 218-230 record actual count, initial/final/bar pose, all-env contact peak, whole-run altitude extrema and tilt, finite sample flag and center clearance. These diagnostics do not imply collision-free flight or a passed gate.
- The name `RECORDED_REVIEW_REQUIRED` explicitly defers interpretation. Original G9 preregistration fixes reported-only response metrics and no success threshold; this candidate does not upgrade them to a scientific PASS.
- Original command schedule, physics/controller selection, altitude PI, response metric formulas and aggregation remain unchanged in the diff.
- The original historical script remains preserved; candidate is proposed as a new `tools/sitl_g9_lee_steps_v2.py`. A dated preregistration correction must record the actual previous asset-count uncertainty and amended single-bar layout before measurement.
- `OUT.open("x")` prevents overwrite even if another writer creates the destination after the initial exists check. A fresh exclusive output directory is still needed to avoid duplicate GPU work before that eventual refusal.

## Execution still pending

GR-1b completion/validation/analysis and its required independent results review; CPU prefix/contract smoke; dated preregistration amendment and experiment-index linkage; local commit and clean approved SHA; checked launch fault evidence; GPU first-run smoke/attempt under the approved scope. Those checks are not claimed by this source-only review.
