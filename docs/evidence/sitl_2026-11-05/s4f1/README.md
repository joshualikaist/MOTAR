# S4F-1 v1 — SITL 2×2 of target appearance × ground on S4's ground arm (2026-11-08)

**Status:** preregistered, design reviews r1 AMEND_FIRST and r2 PASS_WITH_LIMITS with every finding repaired,
approved by the user (2026-11-08 12:07 KST), **NOT_RUN**. Read the
[preregistration](../../../plans/prereg_sitl_s4f1_v1_2026-11-08.md) first.

- [`REVIEW_design_r1.md`](REVIEW_design_r1.md) and [`REVIEW_design_r2.md`](REVIEW_design_r2.md): the design reviews
  (Claude Opus 5.5, fresh context). Their probes stay in `~/motar_tmp/s4f1_20261108/review_design_r{1,2}/`.
- Code: `tools/sitl_s4f1.py`, `tests/test_sitl_s4f1.py` (54 tests, CPU).
- Outside the repository, bound by SHA-256 in the prereg: `~/motar_tmp/s4f1_20261108/component_smoke.py` and the
  chain `run_s4f1.sh`. The implementer's notes are in `IMPLEMENTATION.md` there.
