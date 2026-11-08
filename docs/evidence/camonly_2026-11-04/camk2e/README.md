# CAM-K2-E v1 — matched evaluation of CAM-K2 v2 (2026-11-08)

**Status:** preregistered; design review r1 AMEND_FIRST (in `../camk2/REVIEW_v2_design_r1.md`) and round 2
AMEND_FIRST ([`REVIEW_design_r2.md`](REVIEW_design_r2.md)) and round 3 PASS_WITH_LIMITS
([`REVIEW_design_r3.md`](REVIEW_design_r3.md)), every finding repaired before the commit. Approved by the
user on 2026-11-08 at about 23:30 KST. **NOT_RUN**. Read the
[preregistration](../../../plans/prereg_camonly_k2_eval_v1_2026-11-08.md) first.

**Found before launch** (CPU, exploratory; not CAM-K2-E data):
- [`DEPTHPROBE_REPORT_codex.md`](DEPTHPROBE_REPORT_codex.md) and [`CONTRACT_AUDIT_codex.md`](CONTRACT_AUDIT_codex.md)
  are by a Codex agent. Their raw files are in `~/motar_tmp/camk2_depthprobe_20261108/`.
- [`REVIEW_depthprobe_r1.md`](REVIEW_depthprobe_r1.md) is the independent review (Claude Opus 5.5): PASS_WITH_LIMITS
  for both reports.
- **What they found:** v2's MinZ of 0.4 m and its invalid → no-return convention erase face-on bars in about the last
  0.26 m of approach.
- **What is not shown:** that this changes the crash rate (NOT_TESTED).

**Records:**
- [`controls.json`](controls.json): the five controls on CAM-K1-E's data, CONTROLS_OK.
- Code: `tools/camk2e_run.py`, `tools/camk2e_analyse.py`, `aerial_gym/rl_training/rl_games/eval_navrl_camk2e_arms.sh`,
  `tests/test_camk2e.py` (27 tests).
- Raw outputs will be in `~/motar_tmp/camk2e_20261108/raw/`.
