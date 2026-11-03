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
| `tools/run_focused.py` | r1 single-command runner, kept for provenance; its empty-check behaviour is superseded by `run_gate.py` | r1 copy |
| `docs/` | R0 architecture, r1 contract and amendments, historical R2 rules, research plan, terminal-approach spec | r1 copies, historical |

Run a suite by hand (CPU only, pinned off the faulty cores):

```bash
cd cplanner/tests
CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 PYTHONHASHSEED=0 OMP_NUM_THREADS=1 \
  taskset -c 0-3,8-11,14-23 /home/fair/miniconda3/envs/aerialgym/bin/python -B check_r1_core.py /path/new.json
```
