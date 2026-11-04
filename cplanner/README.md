# C-Planner research code (CPU prototype, not production)

Charter: [`docs/plans/cplanner_charter_v1.md`](../docs/plans/cplanner_charter_v1.md). Nothing here is imported by
`aerial_gym/`, Frozen Policy F or the PUBLIC_CPU suite (`tests/test_*.py`); the scripts in `cplanner/tests/` are run
by `cplanner/tools/run_gate.py` or by hand, never by `tools/run_tests.py`. The modules use bare names (`core`,
`registry`, ...), so never import them into a shared test process.

| Path | Content | Origin |
| --- | --- | --- |
| `src/` | P0-R r1 certifier: exact-ndarray boundary, static-bar certificate, veto-only evidence map, camera label layer | byte-identical to `~/motar_tmp/p0r_r1_20261103/src/` |
| `tests/` | independent harness and suites (`harness.py` shares no code with `src/`) | r1 copies; `noninterference_v2.py` new for R2' |
| `snapshot/` | v3-v7 independent geometry oracle and catalog | r1 copy |
| `motar/replay.py`, `motar/adapter.py` | M2: CPU replay of the B1-R 72x4 LiDAR with MOTAR's own Warp kernel; normalised scan -> P0-R packet with explicit beam status | new |
| `tests/v1_hitbit.py`, `tests/galpha_replay.py` | M2 gates V1 (status mapping vs an independent ray caster) and G-alpha (certifiability on replay) | new |
| `tools/run_gate.py` | R2' gate driver (fail-closed, host-fault voiding, determinism duplicates) | new |
| `tools/run_m2.py`, `tools/run_m3.py` | M2 and M3 part 1 drivers (same helpers; bar-pool preflight; M3 builds the native library in its output) | new |
| `native/coverage_b.c`, `native/build.py`, `src/coverage_native.py` | M3: exact C port of `_coverage_B` (opt-in config `MOTAR-72x4-RUNTIME-NATIVE`; never build the library inside the repository) | new |
| `tests/m3_native_equivalence.py`, `tests/gbeta_timing.py` | M3 gates E1 (whole-result identity with the verbatim Python) and G-beta (compute on replay) | new |
| `planner/c_planner.py`, `tests/planner_unit.py` | M3 predictive planner skeleton (C-OFF / C-FILTER) and its development checks (not a gate); default configuration `MOTAR-72x4-RUNTIME-DIR` since 2026-11-04 (the repaired one; the unrepaired `MOTAR-72x4-RUNTIME` only by name) | new, default changed |
| `tests/native_binding.py` | SHA-256 binding of the native library (M3 review I3): `coverage_native.load()` refuses any library whose SHA-256 is not the qualified `afbd3e68…` or a hash declared in `CPLANNER_COVERAGE_SO_SHA256`; rebuild, tampered, missing and declared cases | new |
| `src/registry.py` (`direction_error_m`), `src/core.py` (eps > 0 only) | direction-bound repair after the M2 V1 FAIL: configs `MOTAR-72x4-RUNTIME-DIR` / `-DIR-NATIVE` (eps 1 mm); every existing config keeps eps 0 | changed |
| `tests/check_direction_bound.py`, `tests/v1v2_kernelray.py`, `tools/run_dirbound.py` | direction-bound gates D1 (most-permissive-direction fuzz), D2 (V1-v2 along the kernel's own float32 rays) and their driver; `tests/galpha_replay.py` reads `CPLANNER_GALPHA_CONFIG` | new |
| `tests/v1v2a_kernelray.py`, `tools/run_dirbound_a1.py` | direction-bound amendment 1: D2-A1 classifies disagreements as permissive / conservative (conservative accepted only when grazing) with rule controls; driver checks byte-identical carry-over of the certifier | new |
| `tests/check_direction_bound_full.py`, `tools/run_dirbound_d1f.py` | gate D1-F: full-bound direction adversary (81 rays per beam within 0.99 eps, heights near 0 / 3 m, candidates within 1.5 mm of R); repaired must give 0 FALSE_CERT, eps = 0 must be caught (review I1) | new |
| `tools/run_focused.py` | r1 single-command runner, kept for provenance; its empty-check behaviour is superseded by `run_gate.py` | r1 copy |
| `docs/` | R0 architecture, r1 contract and amendments, historical R2 rules, research plan, terminal-approach spec | r1 copies, historical |

Run a suite by hand (CPU only, pinned off the faulty cores):

```bash
cd cplanner/tests
CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 PYTHONHASHSEED=0 OMP_NUM_THREADS=1 \
  taskset -c 0-3,8-11,14-23 /home/fair/miniconda3/envs/aerialgym/bin/python -B check_r1_core.py /path/new.json
```
