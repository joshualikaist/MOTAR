# Camera-only study evidence (`camera-only` family)

The plan is [`docs/plans/camonly_study_plan_v1.md`](../../plans/camonly_study_plan_v1.md), with its "After review r1"
section, under decision D-2026-11-04-3.

## `c0/`: the opt-in LiDAR-ablation switch (CPU gate)

| File | What |
| --- | --- |
| `gate_01_c0_gate.json` | C0 gate run on `4d03f35`: C0_CONDITIONAL_PASS (10 tests, 5 mutants caught, PUBLIC_CPU 0 new failures against the base, pinned files unchanged, host faults 0) |
| `REVIEW_r1.md` | Independent review r1 (Claude Opus 5.5, fresh context): **FAIL** on B1, a vacuous association-leak check; medium M1-M3, low L1-L9 |
| `gate_02_c0_gate.json` | C0 gate run on the repair commit `842fcf5`: C0_CONDITIONAL_PASS (14 tests, 7 mutants caught, PUBLIC_CPU 3,248 = 3,234 + 14 with 0 new failures, pinned files unchanged, host faults 0) |
| `REVIEW_r2.md` | Independent review r2 (Claude Opus 5.5, fresh context): **PASS**. C0 is PASS. Open: M-r2-1 (obs_only governor not pinned by the tests) and M-r2-2 (a wrong disclosure in the plan), to fix before the C2 preregistration; low L-r2-1 to L-r2-5 |

Raw logs are in `~/motar_tmp/camonly_c0_20261104/`: `gate_01/`, the base suite log `public_cpu_base_02.log`, and
`review/audit/` (the reviewer's probes).
