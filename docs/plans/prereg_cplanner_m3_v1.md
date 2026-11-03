# Preregistration: C-Planner M3 part 1 — exact native coverage port (E1, E2) and compute gate G-beta

Fixed 2026-11-04 before the gate measurements. Charter: [`cplanner_charter_v1.md`](cplanner_charter_v1.md)
(milestone M3, gate G-beta in §3; host mitigation §7). Decision D5 lets M1-M3 CPU work proceed through its gates;
each verdict is reported.

**Entry condition.** R2' (`prereg_cplanner_r2prime_v1.md`) verdict CONDITIONAL, the independent review R3' with no
in-scope P0-blocking finding, and the M2 run (`prereg_cplanner_m2_v1.md`) finished without STOP_FALSE_CERT (the
charter stops everything on a false certificate). M3 part 1 does not otherwise depend on the G-alpha reading: the
certificate is computed as a label even when the filter arm is paused. It runs after M2 on an otherwise idle host.

**Scope of part 1.** Only the coverage implementation and its compute. The planner skeleton
(`cplanner/planner/c_planner.py`), the simulator arm and the DEV smoke (the rest of M3) are not gated here.

## Design decision (replaces the roadmap's per-packet exclusion grid)

The roadmap named a per-packet exclusion grid "with equivalence to `_coverage_B`". A fixed grid uses a different
cell decomposition than the verbatim branch and bound, so it would need its own soundness argument and could only be
approximately equal. M3 part 1 instead ports `_coverage_B` exactly to C:

- `cplanner/native/coverage_b.c`: same depth-first traversal, push order and node counting; IEEE-754 double
  arithmetic in the Python evaluation order; Python's two-argument `max` / `min` / `sorted` tie rules; CPython
  3.8's `math.hypot` (`vector_norm`, scaled compensated sum). Fixed-size stack (64 entries; the traversal needs at
  most 26), overflow refuses.
- `cplanner/native/build.py`: `gcc -O2 -fPIC -shared -std=c99 -ffp-contract=off -fno-fast-math -mfpmath=sse
  -fno-strict-aliasing -Wall -Werror`, no `-march`; records the gcc version and the SHA-256 of source and library.
- `cplanner/src/coverage_native.py`: ctypes wrapper with the result dictionary of `_coverage_B`; a missing library
  refuses (`NATIVE_COVERAGE_UNAVAILABLE`), never a silent fallback.
- `cplanner/src/registry.py`: new field `coverage`, default `VERBATIM_PY` (every existing configuration unchanged),
  and one new configuration `MOTAR-72x4-RUNTIME-NATIVE` (= `MOTAR-72x4-RUNTIME` with `coverage="NATIVE_EXACT"`).
- `cplanner/src/core.py`: `_certify_one` selects the implementation by that field; an unknown name refuses
  (`COVERAGE_IMPL_NOT_QUALIFIED`). Nothing else on the certify path changes; `coverage_b.py` is untouched.

The previous session's candidates (`coverage_fast.py`: indexed traversal and the Pareto-minimal shape subset) are
superseded and kept outside the repository (`~/motar_tmp/cplanner_m3_20261104/superseded/`, sha256 `bd9b4810…`
and `60a85dc9…` for `m3_equivalence.py`). The exclusion grid stays an option if G-beta fails.

## Driver

`cplanner/tools/run_m3.py` from a frozen `git archive` copy of the committing revision (`cplanner/`, `motar_eval/`,
the Warp LiDAR kernel file). It builds the library inside the output directory, then runs E1 twice, E2, and G-beta
twice, last. CPU only, `taskset -c 0-3,8-11,14-23`, at most 12 workers, kernel fault record and boot id compared
before and after every command (a change voids the run), bound inputs hashed before and after, bar-pool preflight as
in M2. Label at most CONDITIONAL until an independent review.

## E1 — native equivalence (`cplanner/tests/m3_native_equivalence.py`)

Arguments: 2,000 synthetic scenes, 12 workers, 3 episodes per cell from the six V1 cells (`A/F_SHADOW` at 70, 115,
160, 205 bars and `C-PP/B-PP` at 160, 205 bars, seed 4301), sampling seed 20,261,106. Every comparison is of whole
results (JSON of the result dictionary with sorted keys, or the bit pattern of a float):

- **H:** 200,000 `hypot` pairs (specials incl. signed zeros, subnormals, inf, nan; cancellation residues), 50,000 each
  of `point_segment` and `segment_box_distance`, 100,000 `crosses` (strict and not; zero-length, axis-parallel,
  |d| < 1e-15 and lattice inputs);
- **T:** threshold probes at the code's own constants: the strict-crossing margin 1e-9 on all four box edges and
  across the centre, the parallel test |d| < 1e-15, and the surface rule `<= R + 1e-9` at exact distances;
- **S:** synthetic scenes (harness ray caster; `P0R-72x1` and `MOTAR-72x4` alternating; sensor height uniform in
  [0.3, 2.7] m; 0-8 catalog bars; a target sphere), 16 segments each incl. zero-length ones, at the qualified
  setting (R 0.45 m, resolution 0.002 m, budget 100,000), with budget 60, with resolution 0.004 m, plus 2 segments
  without free rays or surfaces and 4 segments without surfaces;
- **R:** CPU-replayed B1-R packets through a `MOTAR-72x4-RUNTIME` core, the 16 G-alpha candidates at every decision
  step;
- **C:** on the same packets, a `MOTAR-72x4-RUNTIME` core and a `MOTAR-72x4-RUNTIME-NATIVE` core must return
  identical step reports and identical certify results (whole dictionaries except `config`).

**PASS** iff the library loads, all parts complete, H, T, S, R and C have zero mismatches, and the positive controls
hold: T has strict crossings true and false and surface hits and misses; S shows each of
ALL_COLLIDING_CENTER_CELLS_EXCLUDED, OBSERVED_SURFACE_INTERSECTION, UNRESOLVED_CENTER_CELL, NODE_BUDGET_UNKNOWN and
NO_VALID_RAYS; R has certified and refused segments; C has 0 < certified < segments. The two runs must agree after
removing timing keys.

## E2 — R2' suites on the changed tree

The seven deterministic R2' suites (`check_static`, `check_r1_core`, `attacks_p0r`, `scenarios_p0r`,
`map_invariants`, `check_r1_temporal`, `regress_v7`) each write their verdict key true and exit 0 on the M3 tree.
This shows the registry field and the dispatch leave every existing configuration's behaviour intact.

## G-beta — compute (`cplanner/tests/gbeta_timing.py`)

Sample: 3 episodes per cell from the 28 G-alpha cells (`A/F_SHADOW` and `C-PP/B-PP` at 160 and 205 bars, seeds
4301-4307), sampling seed 20,261,107. One process pinned to cpu 8 (a P-core thread inside the allowed set; the run
fails if the pinning does not hold), on `MOTAR-72x4-RUNTIME-NATIVE`. Every frame is replayed (not timed); timed with
`time.perf_counter`: adapter (`AD.packet`), `core.step`, and at every decision step (as in G-alpha: even frame,
accepted, target centre > 0.9 m) `core.certify` of the first 8 and of all 16 G-alpha candidates.
total_K = adapter + step + certify_K of the same frame.

Reading (charter §3: K = 8-16 candidates, single core, p95 <= 50 ms):
- **PASS_K16:** p95(total_16) <= 50 ms;
- **PASS_K8_ONLY:** otherwise, p95(total_8) <= 50 ms (the planner must then use at most 8 candidate segments per
  certify call);
- **FAIL:** otherwise; the evaluation scale is reduced by a recorded decision before any pilot (charter);
- **FAIL_INCOMPLETE:** an episode did not finish, nothing was timed, or the pinning failed;
- **UNSTABLE:** the two runs give different readings (not a PASS).
Reported, not gated: p50 / p99 / max, the components, certified counts, and the skeleton planner's `act()` time
in OFF and FILTER modes on the same scans (belief = recorded target position and `t_vel`, a timing input only).

## Limitations fixed in advance

- The identity claim is empirical over the tested inputs plus a line-by-line port; it is bound to this build
  (gcc 9.4.0, x86-64 SSE2, glibc `sqrt`, Python 3.8.20). Another compiler, flags or Python version needs E1 again.
- C code is a new trust base: its inputs are the core's own validated arrays (free rays, hit points, segments
  after `exact_paths`), never caller objects; memory safety rests on the fixed stack and on review.
- Timing is for this host (i7-13700K, BIOS 0806, NOT_REMEDIATED, cores 2, 3, 6 excluded); `core.step` remains the
  verbatim Python per-beam loop and is not optimised here.
- Replayed trajectories are F's and B-PP's, not C's; the candidate set is G-alpha's, not the planner's polylines.

## Disclosed development runs (not evidence)

`~/motar_tmp/cplanner_m3_20261104/dev/`, all under concurrent load (R2' with 12 workers and a review):

| Run | Result |
| --- | --- |
| E1 dev, 40 scenes + 1 replay episode (205 bars), 2 workers | all identical; native 0.33 ms (S) / 0.50 ms (R) per segment vs verbatim 38.6 / 69.3 ms |
| E1 dev with part C, 1 episode (B-PP 160 bars) | all identical; C: 704 segments, 404 certified |
| mutants: strict margin 1e-10; libm `hypot`; swapped push order; surface `<` | first version missed the margin mutant (part T added); final version kills all four: T 2,400 mismatches; H 61,778 hypot mismatches; S 22 / R 7 / C 1; T 2 surface mismatches |
| G-beta dev, 1 episode (205 bars), cpu 22 under load | p95 total_16 49.3 ms, total_8 42.4 ms; step p50 29.9 ms (14 ms in an unloaded earlier run) |
| driver smoke (reduced sizes) | CONDITIONAL; duplicates identical |
| planner dev checks (`cplanner/tests/planner_unit.py`) | 19/19 pass (not a gate) |
