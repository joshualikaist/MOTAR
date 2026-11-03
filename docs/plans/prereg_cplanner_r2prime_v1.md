# Preregistration: C-Planner P0-R r1 full qualification gate R2' (fixed 2026-11-04 before measurement)

Charter: [`cplanner_charter_v1.md`](cplanner_charter_v1.md) (milestone M1). Code under test: `cplanner/src/` at the
commit that adds this file; it is byte-identical to the r1 repair unit
`~/motar_tmp/p0r_r1_20261103/src/` (independent repair review `REPAIR_READY_WITH_LIMITATIONS / NOT_QUALIFIED`,
`audit/REVIEW_r1.md`). The historical rules `cplanner/docs/R2_FIXED_RULES.md` (2026-10-29) stay unchanged; this
file says what R2' keeps, changes and adds, and why.

## Execution

- Driver `cplanner/tools/run_gate.py`, run from a frozen `git archive` copy of `cplanner/` at that commit, in
  `~/motar_tmp/cplanner_r2p_20261104/frozen/`, output `~/motar_tmp/cplanner_r2p_20261104/r2prime/` (must not
  exist). The copied historical `tests/run_r2.sh` is not in the tree and is never used.
- CPU only (`CUDA_VISIBLE_DEVICES=`), `PYTHONNOUSERSITE=1`, `PYTHONHASHSEED=0`, one numerical-library thread per
  process, interpreter `/home/fair/miniconda3/envs/aerialgym/bin/python`, `taskset -c 0-3,8-11,14-23`, 12 workers
  for the randomized suites, `timeout -k` on every command (charter §7).
- Inputs bound before and after by SHA-256: the frozen tree, the v3-v7 `certify_b.py` references, v7 run 8
  `regress_boundary.json`, `motar_eval/occlusion_trace.py`, the 40 `bars_h3` URDFs, the two B1-R traces the
  regression reads, and the interpreter.

## Rules

| Rule | Suite | Pass condition |
| --- | --- | --- |
| 1-2 | `check_static.py` | unchanged: coverage AST identical to v3-v7, catalog equals the URDFs, static camera separation |
| 3/8 | `regress_v7.py` | unchanged: 4,000 nominal + 960 real-layout trials, 0 FALSE_CERT, flag and reason identical to v7 run 8, certified counts equal v7 |
| 4 | `attacks_p0r.py`, `check_r1_core.py` | unchanged attacks; plus the r1 regressions for R3 F1-F5 with 0 internal exceptions |
| 5A | `noninterference_v2.py A`, 20,000 scenes | changed: sensor height uniform in [0.05, 2.95] m (R3 F6: every suite had z = 1.0); each packet cast once and fed to all runs; certificate JSON identical across 7 camera variants; track+camera equals track; track gains 0; FALSE_CERT 0 (base and track runs, oracle at the actual height); 0 internal exceptions; positive controls: VALID > 0 for both layouts and truthful camera labels > 0 |
| 5B | `noninterference_v2.py B`, 2,000 sequences of 30 packets | new (R3 F6: the historical 2-packet track check was vacuous): moving target with a fresh track, catalog bars, relocating non-catalog pole; runs base / camera / track / track+camera; track gains 0; camera identity; FALSE_CERT 0 in all runs; static cells only with a track; **non-vacuity**: sequences with STATIC >= 25 %, with CONFLICT >= 10 %, with a track-caused certificate loss >= 5 %, VALID base certificates > 0 per layout, camera labels > 0 |
| 5C | `check_r1_temporal.py` | the r1 deterministic temporal witness (STATIC at packet 21, CONFLICT at 23, loss and retained controls) |
| 6/8 | `scenarios_p0r.py` | unchanged |
| 7 | `map_invariants.py` (60 sequences) | unchanged, now at full size (r1 ran 2) |
| 9 | `timing_p0r.py` | reported only, run last and alone; no real-time claim |
| 10 | host | kernel fault record and boot id unchanged across every command; else the run is VOID |
| 11 | determinism | deterministic suites run twice and agree after removing `wall_s` and `ms_*`; 5A reruns seeds 0-1999 and 5B seeds 0-199, every per-seed digest equal to the first run |
| 12 | driver | every gated command writes its JSON with an explicit boolean verdict and exits 0; inputs unchanged |

Seeds: 5A scene s uses `random.Random(1_000_000 + s)`, s = 0..19,999; 5B sequence s uses
`random.Random(2_000_000 + s)`, s = 0..1,999. Calibration dry runs used seeds 900,000+ only (below).

## Verdict

All of rules 1-8, 5A-5C and 10-12 pass, and rule 9 is reported: **CONDITIONAL**. Anything else: **FAIL**, or
**VOID_HOST_FAULT** under rule 10. PASS needs the independent review R3' (Opus 5.5, fresh context, read-only copy,
writes only to its audit directory) with no new P0-blocking finding inside charter §5. A FAIL is recorded as is;
a repair is a new dated unit.

## Disclosed pre-gate work (not evidence)

- Dry runs on calibration seeds, outside any result folder, `~/motar_tmp/cplanner_r2p_20261104/dryrun/`:
  5B 40 sequences (seed0 900,000): all checks true; STATIC in 35/40, CONFLICT 7/40, track loss 6/40 sequences,
  FALSE_CERT 0. 5A 80 scenes: all checks true, FALSE_CERT 0. The 5B non-vacuity fractions above were written
  before these dry runs and were not changed after them.
- Driver smoke test at reduced size and a negative control (a missing verdict key gives FAIL, exit 1),
  `~/motar_tmp/cplanner_r2p_20261104/smoke/`.
- Host at preregistration: boot 2026-11-02, all 24 CPUs online, BIOS 0806, microcode 0x12f, 0 kernel fault records
  in the current boot; not remediated (charter §7).
