# CAM-K1-E v1 — matched evaluation of the CAM-K1 camera-only lineage (2026-11-08)

**Status:** preregistered, design review r1 AMEND_FIRST with every finding repaired before the commit, approved by the
user (2026-11-08 12:07 KST), **NOT_RUN**. Read the [preregistration](../../../plans/prereg_camonly_k1_eval_v1_2026-11-08.md)
first.

- [`REVIEW_design_r1.md`](REVIEW_design_r1.md): the design review (Claude Opus 5.5, fresh context) of the
  preregistration, launcher copy, runner, analysis and tests. Its probes stay in
  `~/motar_tmp/camk1e_20261108/review_design_r1/`.
- Code: `tools/camk1e_run.py`, `tools/camk1e_analyse.py`, `aerial_gym/rl_training/rl_games/eval_navrl_camk1e_arms.sh`,
  `tests/test_camk1e.py` (18 tests, CPU).
- Raw outputs will be in `~/motar_tmp/camk1e_20261108/raw/`.
