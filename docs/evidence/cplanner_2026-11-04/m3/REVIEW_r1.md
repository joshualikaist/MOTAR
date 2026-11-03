# Independent review: C-Planner M3 part 1 (exact native coverage port E1, E2 suites, compute gate G-beta)

- Reviewer: Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only; written only under `review/audit/`.
- Date: 2026-11-04 (review 07:20-07:50 KST; the run finished 07:19 KST).
- Inputs judged: frozen code copy `frozen/` (commit `265b244`), `frozen/docs/plans/prereg_cplanner_m3_v1.md`,
  `frozen/docs/plans/cplanner_charter_v1.md`, `review/run_copy/` bound by `review/SHA256_MANIFEST.json`,
  `review/run_driver.log`, the B1-R raw traces (read-only). No author summary, WORKLOG or repository evidence folder
  was used.
- Host fault record (`journalctl -k -b 0 --no-pager | grep -cE "Machine check|segfault|general protection"`):
  **0 before the probes (07:20 and 07:28 KST), 0 after the probes (07:45 KST)**; boot id `407ae422-…` unchanged
  (`fault_count_before_probes.txt`, `fault_count_after_probes.txt`).
- Write disclosure: everything is under `review/audit/`, except one file list I wrote by mistake into the session
  scratchpad (`/tmp/claude-1000/.../scratchpad/frozen_files.txt`) and deleted at once; nothing was written to the
  frozen copy, the run copy, the repository, the B1-R traces or `~/.cache/warp` (Warp cache redirected to
  `audit/warp_cache`). The manifest still verifies after the probes.

## Verdict: **PASS**

E1 PASS and E2 PASS stand, and the G-beta reading **PASS_K16** (p95 total_16 = 20.70 ms, duplicate 20.63 ms,
limit 50 ms) stands as computed under the preregistered method. No blocking finding. Three important findings
(none changes this gate) concern how far the G-beta reading generalises, E1's blind spots for future
re-qualification, and the absence of a runtime identity check of the native library.

## Findings

### Blocking

None. I found no input the core can produce on which `coverage_b.c` and `_coverage_B` differ, no way for the
dispatch to fall back silently or to alter an existing configuration, and nothing in the G-beta method that could
move the reading across the 50 ms limit.

### Important (non-blocking)

**I1. The G-beta reading is a best-case-core, idle-host number; it does not carry over to a parallel evaluation.**
`tests/gbeta_timing.py:39,59` pins to cpu 8 as preregistered (`prereg_cplanner_m3_v1.md:81`). On this host cpu 8/9
is the Turbo-Boost-Max favoured P-core (`cpuinfo_max_freq` 5.4 GHz vs 5.3 GHz for the other P-cores and 4.2 GHz for
the E-cores cpu 16-23, which are half of the allowed set `0-3,8-11,14-23`); its SMT sibling cpu 9 was not reserved
(host loadavg ~1.4 during G-beta, `records.json`). The preregistration's own disclosed dev run on cpu 22 (an E-core)
under load gave p95 total_16 = 49.3 ms (`prereg_cplanner_m3_v1.md:116`), i.e. at the limit. The charter motivates
G-beta by the evaluation's CPU-h budget (`cplanner_charter_v1.md:52-55`), which will run many workers, many of them
on E-cores or busy SMT siblings. The reading is valid as preregistered ("single core", "timing is for this host");
it should be quoted as "favoured P-core, idle host", and any scale decision should use a measurement on an E-core
under the planned worker count.

**I2. E1 cannot detect wrong ports at three sites: the depth limit, the prune margin and the covered-box
evaluation order (and `part_t`'s docstring claims a prune probe that does not exist).** Mutation analysis (probe P3,
26 one-site mutants of `coverage_b.c` compiled with the exact build flags, scored with E1's own `part_h(200000)` and
`part_t()` plus the S comparison on 800 E1 scenes = 43,200 whole-result jobs, and my extras):
- killed by E1's parts: push order (M17), shape order (M14), budget `>=` (M03), global budget (M04), root-box
  evaluation order (M09), centre evaluation order (M10), strict-midpoint evaluation order (M12, by T), libm `hypot`
  (M13: 61,778 hypot mismatches, the number the preregistration reports), `hypot` without compensation (M21),
  surface `<` (M15), strict margin 1e-10 (M16), `lo >= hi` (M23), non-strict crossing in the distance (M25),
  dropped endpoint gaps (M19);
- **not killed by E1's parts**: `dep >= 23` (M01: 0 mismatches in H, T and all 43,200 E1-style S jobs; 504
  mismatches only on my long segments), prune `> reach` and `> reach + 1e-8` (M05, M06), covered box
  `cx - (hx - ex)` (M11); plus tie or practically equivalent mutants (M02 split tie `>`, M07 `>=`, M08 `<`,
  M18 `|d| < 1e-14`, M20, M22, M24).
- Cause: E1's segments are <= 3 m (`m3_native_equivalence.py:210`) and G-alpha's 1-2 m, so the depth limit
  (`coverage_b.py:66`, `coverage_b.c:148`) never binds: in my cache 0 of 998 UNRESOLVED results in E1-style scenes
  came from the depth limit, against 473 of 562 for segments of 8-45 m. The prune margin (`coverage_b.py:59`,
  `coverage_b.c:135`) is a 1e-9 band that random inputs essentially never hit, and `part_t` has no prune probe
  although its docstring says "and the prune rule through small segments" (`m3_native_equivalence.py:131`).
- Why this is not blocking: the actual port is identical at all these sites by line-by-line comparison (same
  literal `reach + 1e-9`, `dep >= 24`, `<= resolution`, `hx > ex && hy > ey`, same operand order), and the unmodified
  source gives 0 mismatches on my 4,000 arena-scale / long-segment jobs including the 473 depth-limit outcomes
  (M00, probe P3). A segment of >= ~8 m inside the arena is an input the core accepts, so this coverage matters for
  the re-qualification the preregistration requires for any other build (`prereg_cplanner_m3_v1.md:99-100`) and for
  any edit of `coverage_b.c`. Recommended: long segments (10-55 m) in S, and a constructed prune-margin probe.

**I3. No runtime identity check of the loaded library.** `src/coverage_native.py:14-38` loads whatever library is
at `CPLANNER_COVERAGE_SO` or the git-ignored default `cplanner/native/libcoverage_b.so`, with no SHA-256, version or
self-test check. In this run the binding is sound: the driver builds into the output directory and passes the path
(`tools/run_m3.py:88-94`), hashes it before and after (`run_m3.py:97-99,117-120`), both E1 runs and both G-beta runs
record that path, `build.json` / `inputs_*.json` / the manifest agree on `afbd3e68…`, and my rebuild with the same
flags from the frozen source is byte-identical. But the planner and the later simulator arm would load an
unqualified or stale build silently (a mismatching signature would even be memory-unsafe), while E1's qualification
is "bound to this build". A missing library or one without the symbols does fail closed (probe P5:
`NATIVE_COVERAGE_UNAVAILABLE`, category SAFE_REJECT; the verbatim configuration unaffected). Recommended: refuse in
`load()` unless the library's SHA-256 (or an exported source hash) equals the qualified value.

### Minor

- **m1.** cert16 is timed right after cert8 on the same packet (`gbeta_timing.py:109-113`), so its caches are
  warm. Measured effect (probe P6, paired, same process): cold/warm ratio p50 1.009, p95 1.049; immaterial against
  the 29 ms margin. There is no result caching in `certify`/`veto_near` (read).
- **m2.** The M3 driver binds inputs with `run_m2.inputs()` (`run_m2.py:47-59`), which omits the external
  references that E2's `check_static` and `regress_v7` read (`run_gate.py:34-46`: v3-v7 `certify_b.py`,
  `regress_boundary.json`, `MOTAR-mt-p1/motar_eval/occlusion_trace.py`, the `bars_h3` URDFs) and the Warp kernel
  binary cache `~/.cache/warp/1.0.0` used by the replay (`logs/e1.stdout.log`). I re-hashed the 54 external entries
  of the R2' `inputs_before.json`: all unchanged now.
- **m3.** `tests/check_static.py:79` lists the certify-path modules whose imports are checked for the camera/label
  layer; the new certify-path module `coverage_native.py` is not in the list (its imports are ctypes, os, pathlib,
  numpy: benign).
- **m4.** The preregistration says the traversal needs at most 26 stack entries (`prereg_cplanner_m3_v1.md:24`);
  the bound is 25 (after expanding a node of depth d <= 23 the stack holds d + 2). Irrelevant against 64; the
  refusal at `coverage_b.c:154` is unreachable.
- **m5.** T's parallel probes compare booleans only, and the |d| < 1e-15 threshold does not change any outcome on
  them (M18 survives); it is not a discriminating control. Harmless: the threshold is practically unobservable.
- **m6.** The G-beta sample includes 2 decision steps with incomplete packets (32 fast refusals) and 46 % of
  candidate segments are map-vetoed before coverage (`gbeta.json` `reasons16`); both are real core behaviour on
  this candidate set, not method errors (see L2).
- **m7.** `run_m3.py:140-146`: the run label CONDITIONAL does not depend on the G-beta reading or its stability
  (an UNSTABLE or FAIL reading would still give CONDITIONAL); this matches the preregistration (the reading is
  reported, `gbeta_reading`), but readers must take the reading from `gbeta_reading`, not from `verdict`.

### Limitations (disclosed or inherent; not findings against the run)

- **L1.** Identity is bound to gcc 9.4.0 without `-march`, glibc `sqrt`, CPython 3.8.20 (CPython 3.10 changed the
  `math.hypot` algorithm; the verbatim Python would change, the C port would not). Disclosed.
- **L2.** Replayed trajectories are F's and B-PP's; candidates are G-alpha's 16 straight 1-2 m segments. Planner
  polylines that are map-vetoed less often or are longer would cost more per certify. The native per-call worst case
  is not bounded by the gate (budget 100,000 nodes per shape x 40 shapes); observed max cert16 26.8 ms.
- **L3.** ulp-level evaluation-order changes inside the covered box (M11) are invisible to any whole-result test
  with these margins; identity at that site rests on review.

## Answers to the questions

**Q1 Port fidelity and safety: yes.** Line by line (`coverage_b.py:10-73` vs `coverage_b.c:19-166`):
- `max`/`min` of two floats keep the first argument unless the second is strictly greater/smaller
  (`py_max2`/`py_min2`, :19-20), as CPython's `min_max`; `sorted` of two floats swaps only on `w < u` (:69), as
  `count_run`; the three-argument `max(x0 - p[0], 0, p[0] - x1)` is reproduced as two sequential `py_max2` with the
  same order (:92-96; the int 0 only changes the type, `hypot` converts).
- `point_segment`: `if den` vs `den != 0.0` (NaN truthy in both; unreachable anyway), clamp order identical (:50).
- `crosses`: same per-axis order, `fabs(d) < 1e-15`, `lo > hi`, midpoint and four strict comparisons (:56-80).
- `segment_box_distance`: same six candidates in the same order, sequential strict `min` (:82-99).
- `_coverage_B`: surfaces first (nodes 0), then `not free`, shapes in order, per-shape `nodes` and global `total`,
  budget test after counting, prune, covered test only if `hx > ex and hy > ey`, first covering ray, resolution or
  `dep >= 24`, then push left, right (right popped first) (:103-166). Result dictionaries match key for key
  (`coverage_native.py:65-72`); the core uses only `certified`, `reason`, `nodes` (`core.py:301-306`).
- `math.hypot`: the C function is CPython 3.8's `vector_norm` for n = 2 (scaled, `csum` from 1.0, compensated
  `frac`, one `sqrt`). Bit identity holds on E1's 200,000 pairs and on my 400,000 further pairs (near-equal pairs,
  10^-323..10^308 sweeps, arena-scale residues; probe P1).
- Floating point: the build has `-ffp-contract=off -fno-fast-math -mfpmath=sse`, no `-march`; the disassembly of the
  run's library has no FMA or x87 instruction (SSE2 `mulsd/addsd/subsd/divsd/maxsd/minsd/sqrtsd` plus `sqrt@plt`
  for the errno path); CPython itself is a generic x86-64 build (`CFLAGS` `-O3` without `-march`, statically
  linked `python3.8` with no FMA instruction in its disassembly). All arithmetic on the Python side
  is one IEEE operation per Python operation, so no fusion exists there either.
- Inputs the core can pass (`core.py:285-302`, `registry.py:93-118`): finite Python floats a, b within the arena
  for the native configuration; free rays and hits as finite `np.float64` tuples from the validated packet; the fixed
  40-shape catalog; R 0.45, resolution 0.002, budget 100,000 (int). All of these are exact under the float64
  conversion of the wrapper.
- Memory safety: stack 64 x 4 doubles, bound 25 (m4); array lengths are taken from the reshaped float64 copies that
  the wrapper keeps alive during the call (`coverage_native.py:45-63`); the ctypes signatures match the C
  prototypes (16 arguments, `long` = 64-bit `c_long`); out buffers sized 2 and 4. A missing or symbol-less library
  refuses (probe P5); a wrong library is I3.
- Dispatch (`core.py:298-302`, diff `a5b7022..265b244`): the only certify-path change is the lookup by the new
  `coverage` field, default `VERBATIM_PY` (`registry.py:96`); an unknown name refuses; the native wrapper refuses
  when the library is absent; exceptions become `INTERNAL_ERROR_*` refusals. No silent fallback, and no existing
  configuration changes (only `_cfg` constructs `Config`; probe P5 shows the verbatim configuration certifies
  normally with the library absent).

**Q2 E1: non-vacuous, PASS supported; detects most but not all wrong ports.** Every comparison is of whole
results (JSON with sorted keys, float `repr` is bit-exact) or float bits. The run's `e1.json`: library loaded (the
built library's path), all parts completed (2,000 scenes, 0 skipped; 18 episodes), H 200,000 / 50,000 / 50,000 /
100,000 with 0 mismatches; T 80,000 strict probes (14,400 true / 25,600 false with strict), 16,800 parallel, 12
surface (8 hit / 4 miss, exact distances at `R + 1e-9` and its neighbours); S 108,000 segments, 0 mismatches, all five
reasons present (NODE_BUDGET only under budget 60); R 11,488 segments at 718 decision steps, 0 mismatches,
certified and refused present; C 1,471 step reports and 11,488 certify results identical, 5,925 certified
(0 < 5,925 < 11,488). The duplicate is identical after removing timing keys (checked). A wrong library would show as
NATIVE_COVERAGE_UNAVAILABLE mismatches in S/R/C. Detection power: see I2 (mutants). My replication of R and C on
nine of E1's own 18 sampled episodes (items 0-2: F 70 bars; 9-11: F 205 bars; 15-17: B-PP 205 bars; probe P4):
7,872 segments, 0 R mismatches, 0 C step/certify mismatches, 3,712 certified; my extension of C to random-start
segments up to 8 m (2,008 segments, 268 certified, 655 UNRESOLVED): 0 mismatches.

**Q3 E2: yes.** All seven suites wrote their verdict key `true` and exited 0 (`records.json`, outputs match the
manifest); `check_r1_core` and `check_r1_temporal` record the frozen `src/` as the source under test; `check_static`
finds `coverage_b.py`'s AST identical to v3-v7. Together with the two-hunk diff of `core.py` and the default field in
`registry.py`, this supports "existing configurations unchanged". Gap m3 (static import list) is cosmetic.

**Q4 G-beta: method as preregistered; reading stands.** Timed with `perf_counter`: `AD.packet` + `core.step`
contiguous (`gbeta_timing.py:94-98`), the scan replay and the planners' `act()` outside the timed spans, cert8 and
cert16 on a `MOTAR-72x4-RUNTIME-NATIVE` core (:109-113), total_K = adapter + step + certK of the same frame (:116-118),
i.e. the charter's "step + certify" plus the adapter, nothing less. Decision steps as in G-alpha (even frame, accepted,
target > 0.9 m). Sample: 28 cells x 3 episodes = 84 items, seed 20,261,107, 84 episodes finished, 6,591 frames, 3,152
decision steps. Pinning: `affinity [8]` recorded in both runs (the timed work is on the pinned main thread). The p95
is nearest-rank (`xs[int(0.95 n)]`, :48). Readings: run 1 p95 total_16 20.70 ms / total_8 17.81 ms; run 2 20.63 /
17.70 ms; both PASS_K16; non-timing fields identical. Inflation/deflation: warm caches immaterial (m1); excluded
frames are the preregistered non-decision frames (cheaper: step only); the host can only inflate (shared SMT sibling,
background load); the material caveat is generalisation (I1).

**Q5 Driver and provenance: sound, with m2.** Library built inside the output directory (`run/native/`), build
record with gcc 9.4.0, flags, source SHA `d559bce4…` (= frozen `coverage_b.c`) and library SHA `afbd3e68…`
(= `inputs_before/after`, = manifest, = my rebuild); `CPLANNER_COVERAGE_SO` injected through the patched
`run_gate.env` used by `run_one`; host record and boot id compared before and after each of the 11 commands (fault
count 0 throughout, boot id unchanged, start/end records consistent); duplicates identical; 174 bound inputs
unchanged before/after and still unchanged now; preflight 30 cells `bars_h3`; fail-closed `complete` condition
requires every element. The frozen copy is blob-identical to commit `265b244` for all 110 files (git hash-object) and
modules resolve to the frozen copy (`motar_eval`, `core`, `coverage_native`). `m3_summary.json` agrees with
`records.json` and the outputs.

**Q6 Other.** I1 (favoured core, idle host), I3 (runtime library identity), L2 (candidate set and map veto share),
and that `core.step` (the Python per-beam loop, p50 13.9 ms) is about three quarters of total_16 (p50 18.7 ms), so
the native port removed the coverage cost (native 0.30 ms vs verbatim 46.9 ms per replay segment, `e1.json`) rather than the step
cost.

## What I verified and how

1. Manifest: all 42 files of `run_copy/` hash to `SHA256_MANIFEST.json`, no extra or missing file; `run_copy/` is
   identical to `run/` (diff -r).
2. Frozen copy vs commit: every file (110) blob-identical to `265b244` (`git hash-object` vs `git rev-parse
   265b244:path`, read-only); diff `a5b7022..265b244` of `core.py` and `registry.py` read in full.
3. Code read in full: `coverage_b.py`, `coverage_b.c`, `build.py`, `coverage_native.py`, `core.py`, `registry.py`,
   `boundary.py`, `m3_native_equivalence.py`, `gbeta_timing.py`, `run_m3.py`, `run_gate.py`, `run_m2.py` (relevant
   parts), `adapter.py`, `replay.py`, `galpha_replay.py` (candidates), `evidence_map.veto_near`, the planner's core use.
4. Rebuild of the library in `audit/` with the exact flags: SHA-256 identical to the run's (`afbd3e68…`);
   `objdump -d` instruction census (no FMA, no x87).
5. Run outputs: `records.json`, `m3_summary.json`, `e1.json`, `gbeta.json`, `dup/*`, `build.json`, `inputs_*`,
   `host_start.json`, `preflight.json`, E2 outputs and logs; duplicate agreement recomputed; 174 bound inputs
   re-hashed now; 54 R2' external references re-hashed now.
6. Probes (CPU only, `taskset -c 0-3,8-11,14-23`, `CUDA_VISIBLE_DEVICES=`, `PYTHONNOUSERSITE=1`, `python -B` with
   `sys.dont_write_bytecode`, at most 4 processes, Warp kernel cache redirected into `audit/warp_cache`; no
   `__pycache__` appeared in the frozen copy):
   - P1 `probes/p1_primitives.py` -> `p1_primitives.json`: 400,000 hypot, 560,000 point_segment, 180,000
     segment_box_distance (on the exact traversal boxes of arena-scale and long segments), 1,440,000 crosses
     (edges +- 0, 1e-12, 1e-9, 2e-9): 0 mismatches.
   - P2 `probes/p2_cache.py` -> `cache_s_*.json`: verbatim results for E1 scenes 0-799 (5 settings, 43,200 jobs)
     and 200 of them translated by +20-35 m with 4 extra 8-45 m segments each (4,000 jobs).
   - P3 `probes/p3_mutants.py` -> `p3_mutants_{A,B,C}.json`, sources and libraries in `mutants/`: 26 mutants (I2).
     M00 (unmodified source) 0 mismatches on all 47,200 jobs, H and T.
   - P4 `probes/p4_replay.py` -> `p4_replay.json`, `p4_replay_9.json`, `p4_replay_15.json`: E1's `part_r`
     replicated on E1's own sampled episodes (items 0-2, 9-11, 15-17): 7,872 segments, 0 mismatches in R and C;
     plus 2,008 reviewer segments (random start within 6 m, length 0-8 m) through both cores: 0 mismatches.
   - P5 `probes/p5_dispatch.py` -> `p5_dispatch.jsonl`: run library / missing path / libm.
   - P6 `probes/p6_warm_cert.py` -> `p6_warm_cert.json`: paired cold/warm cert16 (relative only, shared host).
