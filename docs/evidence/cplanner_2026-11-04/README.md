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
bar's bottom (z = 0) and leaves through the bottom face; the chord inside the box is 1.3e-7 m. The float32 Warp mesh
misses it and the float64 box hits it. This is the exact boundary grazing that charter §5 lists as out of scope. The
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
