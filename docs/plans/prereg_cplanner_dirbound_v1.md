# Preregistration: C-Planner direction-bound repair (after the M2 V1 FAIL)

Fixed 2026-11-04 before the gate measurements. Charter: [`cplanner_charter_v1.md`](cplanner_charter_v1.md).
Decision: D-2026-11-04-2 in [`docs/DECISIONS.md`](../DECISIONS.md), "repair, not tolerance". CPU only; no GPU.

**Entry condition.** M2 ([`prereg_cplanner_m2_v1.md`](prereg_cplanner_m2_v1.md)) is recorded FAIL. V1 found one
NO_HIT beam in 691,200 that the float64 oracle along the certifier's manifest direction sees hitting a bar's bottom
edge. Its independent review ([`../evidence/cplanner_2026-11-04/m2/REVIEW_r1.md`](../evidence/cplanner_2026-11-04/m2/REVIEW_r1.md))
traced the cause to the ~1e-7 rad difference between the kernel's float32 rays and the float64 manifest (I1). It found
0 violations along the kernel's own rays. It also showed that for such beams the certifier's input is more permissive
than the scene, with three cases its argument could not exclude (I2). The M2 FAIL stays recorded; nothing here
changes it.

## Design

The certifier gets a declared bound `eps` on beam-direction error and becomes sound under it by construction.

- **Model.** For every beam and every range t <= 12 m, the point the sensor actually probed lies within `eps` of the
  point o + t d that the certifier assumes: o is the declared origin and d the manifest direction. The bound covers
  direction error, origin rounding and the float32 geometry of the sensor's scene.
- **Certifier change (`cplanner/src/`, opt-in per configuration; `coverage_b.py` and the native port untouched):**
  - `registry.py`: a new field `direction_error_m`, default 0.0, which keeps every existing configuration
    unchanged. Two new configurations, `MOTAR-72x4-RUNTIME-DIR` and `MOTAR-72x4-RUNTIME-DIR-NATIVE`, are
    `MOTAR-72x4-RUNTIME` and its native variant with `direction_error_m = 1e-3` m. A registry assertion requires every
    bar half-size to exceed 10 eps.
  - `core._certify_one`: with eps > 0 the coverage proof receives the bar shapes shrunk by eps per half-size and the
    radius R + 2 eps.
  - `core._extent`: with eps > 0 the qualified bar-height interval is shrunk to (0 + eps, 3 - eps), both for the
    sensor-height eligibility and for clipping free rays.
- **Soundness argument (R2' argument applied to a shrunk scene).** Assume the bound holds, and assume the declared
  statuses and ranges are true along the probed rays (D2 measures both).
  - Every point the proof uses as free evidence then lies within eps of a probed point. That probed point is outside
    every bar footprint, at a height inside (0, 3) m.
  - Take a bar whose footprint comes within R of a candidate segment. Its footprint shrunk by eps per side comes within
    R + sqrt(2) eps <= R + 2 eps, so the proof must exclude its centre cell.
  - Excluding the cell needs a free point strictly inside the cell's common core. For the shrunk shapes that core is
    the true core shrunk by eps, so the probed point within eps would lie inside the true core, which is inside the
    bar's footprint. That contradicts the probed point being free.
  - A true hit within R of the segment appears within R + eps and is refused by the surface rule.
  - Hence, under the bound, no certified segment comes within R of a bar.
- **Choice of eps = 1 mm, fixed here.**
  - The M2 data show a direction difference of at most 1.41e-7 rad (1.7 um at 12 m) and the M2 review measured the
    kernel ray 3.3e-8 m from the edge. Float32 coordinates up to 40 m round by at most about 2e-6 m.
  - D2 requires the measured lateral offset to stay below eps / 10.
  - Development cost on exact scans: 450 -> 446 certified segments of 640 (below).
- **Other files.**
  - `tests/galpha_replay.py` reads an optional `CPLANNER_GALPHA_CONFIG`, limited to the four MOTAR configurations;
    its default is M2's.
  - New: `tests/check_direction_bound.py`, `tests/v1v2_kernelray.py` and the driver `tools/run_dirbound.py`.

## Driver

`cplanner/tools/run_dirbound.py`, run from a frozen `git archive` copy of the committing revision (`cplanner/`,
`motar_eval/`, the Warp LiDAR kernel).

- It builds the native library inside the output directory and passes `CPLANNER_COVERAGE_SO`, plus
  `CPLANNER_GALPHA_CONFIG = MOTAR-72x4-RUNTIME-DIR-NATIVE`.
- Host rules: CPU only, `taskset -c 0-3,8-11,14-23`, at most 12 workers. The kernel fault record and boot id are
  compared before and after every command; a change voids the run. The online CPU list is also recorded before and
  after every command and any change is reported (R3' minor M8).
- It uses M2's bar-pool preflight and input binding (frozen tree, kernel, trace reader, B1-R trace files, Python).
- Label at most CONDITIONAL until an independent review.

## Gates

- **D0 (eps = 0 unchanged):** the seven deterministic R2' suites (`check_static`, `check_r1_core`, `attacks_p0r`,
  `scenarios_p0r`, `map_invariants`, `check_r1_temporal`, `regress_v7`) each write their verdict key true and exit 0
  on this tree.
- **D1 (`check_direction_bound.py`, 2,000 scenes, sampling seed 20,261,108, twice):** synthetic MOTAR 72x4 scenes.
  - Each scene has 1-10 catalogue bars 1-7 m from a vehicle at a height uniform in [0.3, 2.7] m. It has 16 candidate
    segments: 8 random, and 8 aimed to pass a bar at R +- 5 cm.
  - The scan declares, for every beam, the most permissive result within a cone of half-angle alpha: the largest
    nearest-hit distance over the manifest direction and 16 cone directions, cast by the harness ray caster.
  - PASS iff all of these hold:
    - A: `MOTAR-72x4-RUNTIME-DIR` at alpha = 0.9 eps / 12 m has 0 FALSE_CERT, certified > 0 and every step accepted;
    - B: the same bound violated a hundredfold (`-DIR-NATIVE`, alpha x 100) yields FALSE_CERT > 0, a power control;
    - D: exact scans give 0 FALSE_CERT and certified > 0 for both `MOTAR-72x4-RUNTIME-NATIVE` and `-DIR-NATIVE`;
    - E: part A's packets through `-DIR-NATIVE` give results identical to the verbatim ones (whole dictionaries
      except `config`).
  - Reported only: C, `MOTAR-72x4-RUNTIME-NATIVE` (eps = 0) at alpha; the certified counts of D (the cost of eps).
- **D2 (`v1v2_kernelray.py`, 1,000 frames per V1 cell, sampling seed 20,261,109, the six M2 V1 cells, twice):**
  replayed scans as in M2.
  - The kernel ray of every beam is computed with the kernel's own float32 Warp operations (normalize, quat_rotate,
    normalize). The float64 oracle casts along it from the float32 origin against the recorded float64 layout.
  - PASS iff all of these hold:
    - K1: along the kernel ray, 0 non-finite values; every status disagreement of V1's rule-1 kinds is at most
      `DELTA_GEOM` = 1e-5 m deep inside its box (or the target sphere); 0 HIT range errors above 0.5 mm; and 0 adapter
      HIT ranges beyond the oracle distance;
    - K2: max over beams of 12 m x sin(angle between kernel ray and manifest) plus the origin rounding <= eps / 10;
    - K3: every disagreement along the manifest (the M2 V1 kind) is at most eps deep;
    - the maximum HIT range error is at most 0.5 mm;
    - the adapter status mapping is exact on every beam;
    - positive controls: HIT > 0, NO_HIT > 0 and target-sphere hits > 0.
- **D3 (`galpha_replay.py` on `MOTAR-72x4-RUNTIME-DIR-NATIVE`, M2's sample: 6 episodes per cell, seed 20,261,105,
  28 cells, twice):**
  - Any FALSE_CERT gives **STOP_FALSE_CERT**.
  - Otherwise the reading follows charter §3: certifiable rate >= 60 % FILTER_ARM_RETAINED, 20-60 %
    MONITOR_ONLY_PILOT, < 20 % FILTER_PAUSED.
  - The M2 rate (0.816 on `MOTAR-72x4-RUNTIME`) is reported beside it as the cost of the repair.

**Verdict.** Every gate passes, the duplicates agree and the inputs are unchanged: **CONDITIONAL**. Anything else:
**FAIL**, **STOP_FALSE_CERT** or **VOID_HOST_FAULT**. After an independent review with no blocking finding:
- the repair is **PASS**;
- V1-v2 replaces V1 for `MOTAR-72x4-RUNTIME-DIR` (V1 itself stays FAIL for `MOTAR-72x4-RUNTIME`);
- the D3 reading decides the FILTER arm under the charter.

## Limitations fixed in advance

- The bound is checked on CPU replays. The delivered GPU scans were never recorded, and CPU and GPU float32 results
  can differ in the last bits (M2 review L1). The repair relies on eps being 10 times the measured bound, and on
  ulp-level differences staying well below it; that is argued, not measured.
- D1's most permissive declaration samples 17 directions per beam, not the whole cone. The power control shows the
  sampling detects a hundredfold violation.
- `DELTA_GEOM` = 1e-5 m and `eps` = 1e-3 m are fixed here, before the gate run.
- The scope of charter §4 is unchanged: static, upright, full-height catalogue bars, exact pose, cooperative caller.

## Disclosed development runs (not evidence)

These ran in `~/motar_tmp/cplanner_dirbound_20261104/` on the uncommitted working tree:

| Run | Result |
| --- | --- |
| D1 dev, 40 scenes, 4 workers | A 0 FALSE_CERT, 446 certified of 640; B 2 FALSE_CERT; C 0; D 450 (eps = 0) and 446 (eps) certified, 0 FALSE_CERT; E identical |
| D2 dev, 2 cells x 60 frames | 0 kernel-ray and 0 manifest disagreements; max lateral 1.9e-6 m |
| D2 on the M2 violation frame (205 bars seed 4301, trace index 10752) | manifest disagreement 1 (beam 269, depth 1.7e-8 m), kernel-ray disagreement 0; kernel-ray HIT range error max 1.7e-6 m |
| driver smoke, reduced sizes | CONDITIONAL; duplicates identical; G-alpha on `-DIR-NATIVE` 0 FALSE_CERT |
| the first D1 dev run | refused 23 of 24 steps because the test built headers with the wrong episode id (test bug, fixed) |
