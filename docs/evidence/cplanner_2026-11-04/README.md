# C-Planner evidence, from 2026-11-04

One evidence folder for the `c-planner` family ([`docs/EXPERIMENTS.md`](../../EXPERIMENTS.md)), one subfolder per
gate. Charter: [`docs/plans/cplanner_charter_v1.md`](../../plans/cplanner_charter_v1.md). Raw outputs stay outside
the repository in `~/motar_tmp/` and are bound here by path and SHA-256 (`SHA256_MANIFEST.json` in each subfolder).
Committed copies of independent reviews are numbered `REVIEW_r<n>.md` inside the gate's subfolder.

## M1 — R2' qualification run of the P0-R r1 certifier (`r2prime/`)

- Preregistration: [`docs/plans/prereg_cplanner_r2prime_v1.md`](../../plans/prereg_cplanner_r2prime_v1.md).
- Code: frozen `git archive` copy of `b407039` (`cplanner/` = byte-identical r1 source), run by
  `cplanner/tools/run_gate.py`, 2026-11-04 01:17 – 06:07 KST, CPU only, `taskset -c 0-3,8-11,14-23`, 12 workers.
- Raw: `~/motar_tmp/cplanner_r2p_20261104/r2prime/` (63 files); frozen tree `~/motar_tmp/cplanner_r2p_20261104/frozen/`.
  Both are listed with SHA-256 in [`r2prime/SHA256_MANIFEST.json`](r2prime/SHA256_MANIFEST.json).
- Driver verdict ([`r2prime/gate_summary.json`](r2prime/gate_summary.json)): **CONDITIONAL**. Rules 1-2, 3/8, 4,
  5A, 5B, 5C, 6/8 and 7 true; rule 9 (timing) REPORTED; all nine determinism comparisons true; 19 of 19 commands;
  inputs unchanged; host fault count 0 before and after every command, boot id unchanged.

| Suite (rule) | Result (from the suite's own JSON in the raw directory) |
| --- | --- |
| `check_static` (1-2), `check_r1_core` (4) | pass; core 7 tests, 0 failures, 0 baseline false certificates |
| `attacks_p0r` (4) | SAFETY 106/106, CONTRACT 10/10, CONTROL 1/1 |
| `scenarios_p0r` (6/8) | 114 segments, 0 failures, FALSE_CERT 0; open scene certified 200/200 on both layouts |
| `map_invariants` (7) | I1-I7 0 violations; 60 sequences, 1,800 steps, 115,831,788 cell checks |
| `check_r1_temporal` (5C) | focused checks pass (STATIC at packet 21, CONFLICT at 23) |
| `regress_v7` (3/8) | inputs, flags and reasons identical to v7; 0 false certifications |
| `ni_A` (5A) | 20,000 scenes, 160,000 segments: camera and track+camera outputs identical, track gain 0, false cert 0, exceptions 0 |
| `ni_B` (5B) | 2,000 sequences, 112,000 segments: track gain 0, track loss 1,833 (all MAP_VETO_CONFLICT), false cert 0 in all four runs; non-vacuity thresholds met |
| duplicates (11) | the seven deterministic suites rerun identical; 5A seeds 0-1999 and 5B seeds 0-199 rerun with equal per-seed digests |
| `timing_p0r` (9) | reported only: verbatim certify of 8 segments p95 612 ms on MOTAR-72x4 (the reason M3 ports the coverage search) |

### R3' independent review: **PASS** (`r2prime/REVIEW_r1.md`)

- Reviewer: Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only copies of the frozen tree and of the run
  output (`~/motar_tmp/cplanner_r3p_20261104/`), writing only to its `audit/` directory. Two phases: code, tests and
  driver while the run was in progress (`r2prime/REVIEW_r1_phase1.md`), then the outputs and re-execution probes.
  The request, phase plan, copy manifests and all 68 audit files are bound in
  [`r2prime/REVIEW_r1_MANIFEST.json`](r2prime/REVIEW_r1_MANIFEST.json).
- Verdict: no in-scope P0-blocking finding under charter §5, so **R2' PASS** (its preregistered label) and **M1
  complete**. Scope: charter §4 assumptions only; not a safety, real-flight, dynamic-object or real-time claim.
- Findings: P0-blocking none; important none; minor M1-M8; limitations L1-L5; observation O1 (see the report).
  - M8 (new in phase 2): cpu 4 and 5 were taken offline by hand at 01:46 KST while 5A ran (a hotplug, not a fault
    record; rule 10 holds). The driver does not compare the online CPU list, so the run output does not show it.
    Every command was pinned off cpu 4-7, and the duplicates and the reviewer's re-runs agree.
  - L3: step plus certify of 8 segments has p95 464-625 ms on the verbatim coverage search, far from the 50 ms
    G-beta budget (M3).
  - L5: certified segments often pass close to non-catalog or dynamic objects (outside scope, never counted safe).
- The reviewer's own checks include an independent ray caster and oracle fuzz (0 FALSE_CERT in 3,825 certificates;
  it catches three unsound mutants), recomputed determinism (5A 2,000/2,000 and 5B 200/200 digests) and 84 seeds
  re-run outside the duplicate range (84/84 digests equal).

## M2 — MOTAR 72x4 status mapping V1 and certifiability on replay G-alpha (`m2/`)

- Preregistration: [`docs/plans/prereg_cplanner_m2_v1.md`](../../plans/prereg_cplanner_m2_v1.md). Entry condition met
  (R2' CONDITIONAL, R3' no in-scope P0-blocking finding).
- Code: frozen `git archive` copy of `a5b7022`, run by `cplanner/tools/run_m2.py`, 2026-11-04 06:22 – 06:36 KST,
  CPU only, `taskset -c 0-3,8-11,14-23`, 12 workers. Raw: `~/motar_tmp/cplanner_m2_20261104/run/` (19 files),
  bound with the frozen tree in [`m2/SHA256_MANIFEST.json`](m2/SHA256_MANIFEST.json).
- Driver verdict ([`m2/m2_summary.json`](m2/m2_summary.json)): **FAIL**, because V1 failed. Bar-pool preflight
  pass; both commands ran twice with identical results; inputs unchanged; host fault count 0, boot id unchanged.

### V1: **FAIL** ([`m2/v1.json`](m2/v1.json))

2,400 frames, 691,200 beams (HIT 291,027, NO_HIT 400,119, NEAR_CLIPPED 54, target-sphere hits 4,936). Range error
max 2.48e-4 m (tolerance 5e-4 m), beam-direction difference max 1.41e-7 rad, 0 adapter status mismatches, 0 declared
ranges beyond the oracle distance, 3 band cases, positive controls present. One violation, so rule 1 fails:

- `v_nohit_but_oracle_hit`: cell `A/F_SHADOW/E2_obstacle_aware__205bars__seed4301`, trace index 10752 (env 0,
  episode 1, frame 28), beam 269 (row 3 at -10 deg elevation, column 53). The kernel returned no hit; the float64
  oracle hits bar 59 at 6.0991 m.

Post-hoc diagnosis, not gate evidence ([`m2/v1_violation_diagnosis.json`](m2/v1_violation_diagnosis.json), script
`m2/v1_violation_diagnosis.py`): the ray clips the bottom edge of bar 59. It enters the side face 2.2e-8 m above the
bar's bottom (z = 0) and leaves through the bottom face; the chord inside the box is 1.3e-7 m. This first
diagnosis attributed the miss to the float32 mesh. *Corrected by the independent review (I1, below):* the kernel's
own float32 ray passes 3.3e-8 m below the edge and misses; the certifier's float64 manifest direction, 9.7e-8 rad
away, clips the corner. Cast exactly along the kernel's rays, all 691,200 beams agree. This is exact boundary
grazing in the sense of charter §5. The
certifier cuts the free interval of a downward beam at z = 0 (`core._extent`), here at 6.09912 m, so the free
segment reaches 6.6e-8 m into the bar footprint. The V1 rule exempts only a 0.1 mm range band at 0.2 m and 12 m, not
grazing, so V1 stays FAIL. Any amendment would be a new dated preregistration and is the user's decision.

### G-alpha reading: **FILTER_ARM_RETAINED**, FALSE_CERT 0 ([`m2/galpha.json`](m2/galpha.json))

168 episodes, 12,636 frames, 6,089 decision steps, 97,424 candidate segments; 49,999 certified, 0 FALSE_CERT,
0 exceptions. Certifiable rate 4,969 / 6,089 = 0.816 (threshold 0.60):

| Group (arm : bars) | Certifiable rate |
| --- | --- |
| F_SHADOW : 160 | 0.885 |
| F_SHADOW : 205 | 0.868 |
| B-PP : 160 | 0.758 |
| B-PP : 205 | 0.823 |

This reading comes from a run whose verdict is FAIL (V1). It was computed as preregistered, but it does not by
itself clear the FILTER arm. Reported, not gated: verbatim `certify` of 16 segments p50 597 ms, p95 1,379 ms (12
concurrent workers).

### Independent review: **FAIL**, confirming the recorded FAIL (`m2/REVIEW_r1.md`)

- Reviewer: Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only copies (`~/motar_tmp/cplanner_m2_20261104/review/`),
  writing only to its `audit/` directory; request, manifests and the 23 audit files are bound in
  [`m2/REVIEW_r1_MANIFEST.json`](m2/REVIEW_r1_MANIFEST.json). Host fault count 0 before, between and after its probes.
- Verdict: FAIL, because V1 failed. It adds no new blocking reason. V1 and G-alpha are both computed as preregistered;
  the reviewer's own loop over all 168 episodes reproduces every G-alpha count (closest certified segment to a bar
  0.4556 m) and its separate oracle reproduces the V1 violation.
- B1 (blocking, the recorded FAIL): the V1 violation above.
- I1 (important): the cause is the beam-direction difference between the kernel's float32 rays and the certifier's
  float64 manifest, not the replay or the adapter. Along the kernel's exact rays there are 0 violations; production box
  faces and the 95 parked boxes do not change it. V1 has no band at bar edges, so a rerun of the same design is
  expected to fail at a similar rate, about 1 per 7e5 beams. G-alpha's 3.6e6 replayed beams probably held a few such
  beams; its FALSE_CERT, measured against ground truth, is 0.
- I2 (important, Q6): the certifier's input is more permissive than the scene for such a beam: here the free ray
  reaches 1.27e-7 m into bar 59's footprint. For every beam of this kind the intrusion is at most about 2e-5 m at 12 m.
  A shifted-bar argument then keeps every certified segment at least about 0.45 - 3e-5 m from the bar. It has three
  stated exceptions that the reviewer could not exclude: a second near-tangent free ray on the same bar in the same
  packet, intrusions from opposite faces, and a HIT beam within about 2 deg of grazing. Charter §4 assumes true
  declared statuses; this is a violation at the 1e-7 rad level, inside V1's scope.
- Minor: M1 the replay's box triangulation differs from production's `trimesh.creation.box` on 3 of 6 faces,
  undisclosed (re-cast with production faces: 0 status differences); M2 G-alpha's terminal zone uses the 2-D distance
  (3-D gives the same 6,089 decision steps); M3 the driver does not hash `frozen/docs`, site-packages or the Warp
  cache; M4 the V1 oracle shares the trace reader and `sensor_yaw` with the replay; M5 the 3 band cases are only counted.
- Limitations: CPU and GPU float32 can differ by ulps, so the GPU status of this beam is unknown; certifiable rate
  0.42 at 0.9-2 m from the target; verbatim certify far from the G-beta budget (resolved by M3's native port).

## M3 part 1 — exact native coverage port E1, R2' suites E2, compute G-beta (`m3/`)

- Preregistration: [`docs/plans/prereg_cplanner_m3_v1.md`](../../plans/prereg_cplanner_m3_v1.md). Entry condition
  met: R2' CONDITIONAL, R3' with no in-scope P0-blocking finding, M2 finished without STOP_FALSE_CERT.
- Code: frozen `git archive` copy of `265b244`, run by `cplanner/tools/run_m3.py`, 2026-11-04 06:37 – 07:19 KST,
  CPU only, `taskset -c 0-3,8-11,14-23`, 12 workers for E1, one process pinned to cpu 8 for G-beta, otherwise
  idle host. Library built in the output directory (gcc 9.4.0, fixed flags; source sha256 `d559bce4…`, library
  `afbd3e68…`, [`m3/native_build.json`](m3/native_build.json)). Raw: `~/motar_tmp/cplanner_m3_20261104/run/`
  (42 files), bound with the frozen tree in [`m3/SHA256_MANIFEST.json`](m3/SHA256_MANIFEST.json).
- Driver verdict ([`m3/m3_summary.json`](m3/m3_summary.json)): **CONDITIONAL**. 11/11 commands; E1 PASS twice with
  identical results; E2 seven suites PASS; G-beta PASS_K16 twice; inputs unchanged; host fault count 0, boot id
  unchanged. After the independent review below: **M3 part 1 PASS**.

### E1 native equivalence: **PASS** ([`m3/e1.json`](m3/e1.json))

| Part | Compared | Mismatches |
| --- | --- | --- |
| H primitives | 200,000 `hypot`, 50,000 `point_segment`, 50,000 `segment_box_distance`, 100,000 `crosses` | 0 |
| T thresholds | 80,000 strict-margin probes (crossings true and false), 16,800 parallel, 12 surface (8 hit, 4 miss) | 0 |
| S synthetic | 2,000 scenes, 108,000 segments at the qualified, budget-60 and 0.004 m settings plus no-free / no-surface cases | 0 |
| R replay | 18 episodes, 718 decision steps, 11,488 segments | 0 |
| C core differential | 1,471 steps, 11,488 segments, 5,925 certified (verbatim core vs native core) | 0 |

Mean time per segment, verbatim vs native: 33.6 vs 0.26 ms (S), 46.9 vs 0.30 ms (R).

### E2: the seven R2' suites on the changed tree, all **PASS** ([`m3/records.json`](m3/records.json))

`check_static`, `check_r1_core`, `attacks_p0r`, `scenarios_p0r`, `map_invariants`, `check_r1_temporal` and
`regress_v7` each wrote their verdict key true and exited 0.

### G-beta reading: **PASS_K16** ([`m3/gbeta.json`](m3/gbeta.json))

84 episodes (3 per cell from the 28 G-alpha cells), 6,591 frames, 3,152 decision steps, one process on cpu 8,
`MOTAR-72x4-RUNTIME-NATIVE`:

| Component (ms) | p50 | p95 | p99 | max |
| --- | --- | --- | --- | --- |
| adapter | 0.033 | 0.048 | 0.058 | 0.088 |
| `core.step` | 13.9 | 15.8 | 17.4 | 39.3 |
| certify, 8 segments | 2.2 | 3.6 | 4.0 | 5.6 |
| certify, 16 segments | 4.6 | 6.9 | 7.4 | 26.8 |
| total, K = 8 | 16.2 | 17.8 | 18.7 | 40.0 |
| total, K = 16 | 18.7 | **20.7** | 21.5 | 40.8 |

Limit 50 ms (charter §3). The duplicate run read PASS_K16 with p95 total_16 20.6 ms. Scope of the reading: one
process on cpu 8, the host's fastest P-core, on an idle host; it is not the per-decision cost inside a parallel
evaluation (the preregistration's development run on an E-core under load read 49.3 ms). `core.step` (verbatim Python)
is now the larger share. Reported only: the skeleton planner's `act()` p95 19.5 ms (OFF) and 23.4 ms (FILTER) on
the same scans, with the recorded target state as a stand-in belief (a timing input, not evidence).

### Independent review: **PASS** (`m3/REVIEW_r1.md`)

- Reviewer: Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only copies (`~/motar_tmp/cplanner_m3_20261104/review/`),
  writing only to its `audit/` directory (one stray scratch file outside it was deleted at once and is disclosed in
  the report). Request, manifests and the 92 audit files are bound in
  [`m3/REVIEW_r1_MANIFEST.json`](m3/REVIEW_r1_MANIFEST.json). Host fault count 0 before and after its probes.
- Verdict: E1 PASS and E2 PASS stand; the G-beta reading PASS_K16 stands as computed. No blocking finding: no input
  the core can produce on which the C port and `_coverage_B` differ was found, the dispatch never falls back
  silently, and no existing configuration changes.
- Important findings, none of which changes this gate:
  - I1: G-beta ran on the fastest P-core of an idle host; quote it with that scope (above).
  - I2: E1 cannot detect three mutant classes (a different depth limit, a changed prune margin, a reordered
    covered-box calculation); the port is identical there line by line, and the reviewer's 4,000 extra jobs at arena
    scale (473 depth-limit outcomes) gave 0 mismatches. E1 must be strengthened before it qualifies another build or
    an edit of the C file.
  - I3: `coverage_native.load()` loads whatever library is at the path, with no SHA-256 or version check; the M3
    driver binds it, later planner or simulator use must.
- Minor m1-m7 and limitations L1-L3 (build and Python version bound; replay trajectories and G-alpha candidates; last-
  bit covered-box arithmetic rests on review) are in the report.
- The reviewer's probes: a byte-identical rebuild with no FMA or x87 instructions, about 2.6 million primitive
  comparisons on other distributions, 26 mutants, E1 parts R and C replicated on 9 episodes plus 2,008 random-start
  segments: 0 mismatches for the unmodified source.

## Direction-bound repair after the M2 V1 FAIL (`dirbound/`)

- Decision D-2026-11-04-2 (repair, not tolerance). Preregistration:
  [`docs/plans/prereg_cplanner_dirbound_v1.md`](../../plans/prereg_cplanner_dirbound_v1.md).
- Run: frozen copy of `8e2b77d`, 2026-11-04 11:25 – 11:48 KST, 12 workers, host fault count 0 throughout, online CPU
  list unchanged. Raw: `~/motar_tmp/cplanner_dirbound_20261104/run/` (48 files), bound with the frozen tree in
  [`dirbound/SHA256_MANIFEST.json`](dirbound/SHA256_MANIFEST.json).
- Driver verdict ([`dirbound/dirbound_summary.json`](dirbound/dirbound_summary.json)): **FAIL**, because D2 failed.

| Gate | Result |
| --- | --- |
| D0, eps = 0 | the seven R2' suites PASS |
| D1 ([`dirbound.json`](dirbound/dirbound.json)), 2,000 scenes, 32,000 segments per part | A (bound respected): 0 FALSE_CERT, 21,636 certified; B (100x the bound): 46 FALSE_CERT; D exact scans: 0 FALSE_CERT, 21,663 (eps = 0) vs 21,636 (eps) certified; E native identical |
| **D2** ([`v1v2.json`](dirbound/v1v2.json)), 1,728,000 beams | **FAIL** (K1, K3, range tolerance): 3 kernel-ray HIT beams without an oracle hit and 1 HIT 24.26 m short of the oracle, all in one frame; 5 manifest disagreements (2 at 4.7e-7 m depth); K2 held (max lateral 2.96e-6 m) |
| D3 ([`galpha_dir.json`](dirbound/galpha_dir.json)) | FILTER_ARM_RETAINED, certifiable rate 0.8162 (M2 0.8161), 0 FALSE_CERT |

Post-hoc diagnosis, not gate evidence ([`dirbound/v1v2_failure_frame14571.txt`](dirbound/v1v2_failure_frame14571.txt)):
at trace index 14571 of the 205-bar cell, the four beams of column 54 hit the vertical corner edge of bar 167. Each hit
point lies within 1e-6 m of the bar's surface. The float32 ray touches the corner and the float64 ray passes just
outside it. That is grazing in the conservative direction: a declared obstacle where the layout is free.
Amendment 1 ([`docs/plans/prereg_cplanner_dirbound_v1_amendment1_2026-11-04.md`](../../plans/prereg_cplanner_dirbound_v1_amendment1_2026-11-04.md),
the one repair round) classifies disagreements by direction. It accepts conservative ones only when grazing (within
1e-5 m of a surface) and adds rule controls. It reruns D2 as D2-A1 on a fresh sample; D0, D1 and D3 carry over by
byte identity.
