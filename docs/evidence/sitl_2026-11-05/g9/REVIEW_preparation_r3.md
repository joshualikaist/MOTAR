# G9 external preparation review r3

Date: 2026-11-05 KST. Reviewer: Codex child `/root/g9_resume_audit`; exact underlying model identifier not exposed. Source-only review, not Opus GR-1b results review or GPU approval.

Verified actual candidate bytes with `sha256sum`: `a41a4e1aadacabc023f37ae9b1efddbf01b212b18eeff559b53c35e87c2441a1`, matching `candidate_manifest.json`.

**Preparation reading: PASS_WITH_LIMITS for externally staged source only.** Both r2 preparation blockers are repaired: line 68 propagates failed journal retrieval using `check=True`; lines 238-241 preserve an INVALID result and then exit unsuccessfully. The candidate explicitly records the retained legacy sampling-alignment limitation. The one-bar scene guards, PhysX submission/Warp refit, all-env diagnostics and exclusive output remain present.

No additional source blocker was identified in this limited follow-up. Contact, clearance, finite samples and `RECORDED_REVIEW_REQUIRED` remain diagnostics requiring interpretation, not a collision-free claim or scientific gate PASS.

Execution is NOT_RUN. Pending before execution: GR-1b completion/validation/analysis and required independent results review; relevant CPU smoke; dated preregistration correction and index linkage; installation as an additive script preserving the original; clean local approved commit; fresh exclusive output and launch guards. No tests, CUDA or experimental compute jobs were launched by this reviewer. Repository status was clean at `camonly/dev` HEAD `5fd8d7d`; only this external review file was written.
