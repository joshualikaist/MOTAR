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
