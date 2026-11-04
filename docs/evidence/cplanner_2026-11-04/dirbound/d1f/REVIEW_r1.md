# Independent review of C-Planner gate D1-F (full-bound direction adversary)

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`). Fresh context, read-only. I wrote only to `review_d1f/audit/`.
- **Date:** 2026-11-04, 14:09-14:45 KST.
- **Object:** the frozen copy `frozen_d1f/` of commit `37762ab`, its preregistration
  `frozen_d1f/docs/plans/prereg_cplanner_dirbound_d1f_v1.md`, the repair preregistration, charter §4-§5, and the run
  (`run_copy/`, `SHA256_MANIFEST.json`, `run_driver.log`).
- **Sources not used:** the author's summaries, WORKLOG, the repository evidence folders and the earlier review.

## Verdict: **PASS**

D1-F stands as computed and is valid eps-scale evidence for the direction-bound repair.

**Reproduction.** I re-ran all 2,000 scenes in my own process layout and got every gated statistic exactly. The ground
truth (caster, clearance, aimed targets) agrees with independent implementations to about 1e-14 m.

**Power at the eps scale.** The adversary is within the bound at 0.99 eps. On the gate's own scans it detects:
- a height-interval shrink of 0.98 eps (instead of eps);
- a radius growth of 0.75 mm (instead of 2 eps).

The eps = 0 control fails at ordinary heights through **lateral** offsets. Its 155 false certificates at eligible
heights are exactly the false certificates of a certifier that has only the height part of the repair.

**The repaired configuration.**
- It certifies 4,435 segments with 0 FALSE_CERT.
- Its refusals of the control's false certificates come from the repair's own mechanisms (unresolved centre cell,
  surface rule, height ineligibility), not from unrelated reasons.
- Two stronger adversaries beyond the gate's family also give 0 FALSE_CERT:
  - a 793-ray family per beam at 0.999 eps that adds longitudinal offsets;
  - candidate segments that do not start at the sensor.
- Provenance is complete and consistent.

There is no blocking finding. Four minor findings limit what the PASS shows and should be disclosed with the result.
The most important is m1: D1-F does not exercise the shape-shrink part of the repair.

## Findings

### Blocking (P0, charter §5): none

### Major: none

### Minor (non-blocking)

**m1. D1-F does not test the shape shrink. Its lateral power comes from the radius growth.**
- Code: `core.py:306-308`. Evidence: `out/p1_analysis.json` totals, `out/p3_merged.json` and `out/p5.json`.
- Test: I built a core equal to `-DIR-NATIVE` except that it does not shrink the bar half-sizes (M_S0: height shrink
  eps, radius R + 2 eps, full shapes).
- Result on the gate's 2,000 scans:
  - 0 FALSE_CERT and 4,434 certified, against 4,435 for `-DIR-NATIVE`. It would pass every D1-F check.
  - With or without the shape shrink, the detection counts at each radius growth are the same: 99 vs 101, 43 vs 43,
    8 vs 8, 0 vs 0.
- On my off-sensor candidates (P5), M_S0 also certifies nothing and has 0 FALSE_CERT.
- So in this scene family the radius growth alone gives the lateral protection. The shrink step of the soundness
  argument (prereg_cplanner_dirbound_v1.md, "Soundness argument") rests on the argument and on code review:
  - I confirmed in the code that the shrink is applied (`core.py:306-308`);
  - my `MutCore(eps, eps, 2 eps)` reproduces `-DIR-NATIVE` dictionary for dictionary (`out/p0_sanity.json`).
- D1-F would not detect a defect confined to the shape shrink. This does not hide a defect of the repair as
  committed. It does limit what D1-F shows, so it is worth one sentence in the result record.

**m2. Most of the control's 2,880 false certificates are vertical and sit at heights the repaired configuration refuses
anyway. The lateral evidence is 155 segments.**
- 2,725 of 2,880 ZERO false certificates (95 %) are in the bands z in [0.00001, 0.001] and [2.999, 2.99999] m:
  - the repaired configuration refuses them by sensor-height ineligibility (`core.py:214-218`; NO_VALID_RAYS 2,611,
    surface rule 114);
  - a certifier with only the lateral parts of the repair reproduces 2,611 of them (M_L, a subset of ZERO).
- At eligible heights, the eps = 0 control has 155 false certificates in 128 scenes (75 at 0.3-2.7 m). These are the
  lateral detections:
  - they are identical as a set to those of the height-only certifier M_H;
  - in all 75 at mid heights, a beam's free segment enters a bar footprint by 0.81-1.02 mm (`out/p4_forensic.json`);
  - in 74 of the 75 the declaring ray is a pure lateral origin offset (±a u).
- The headline count 2,880 therefore overstates the lateral power. Per-band reporting would be clearer (table below).
- Candidates anchored at the sensor are a weak lateral probe. With off-sensor candidates (P5), the same scans give
  403 lateral false certificates for the control (203 at mid heights) in 1,635 eligible scenes, against 155 in the
  gate.

**m3. The family has no offsets along the beam direction. The bound admits them.**
- Code: `check_direction_bound_full.py:70-76, 89-94`.
- The family offsets e0 and e12 all lie in the plane perpendicular to d. The model allows any probed point within eps
  of o + t d, so the bound also admits:
  - an origin shifted along d;
  - equivalently, a HIT range that overshoots the true range. Amendment 1 (D2-A1) tolerates overshoots of up to
    0.5 mm.
- The preregistration's limitation ("81 rays per beam, not the whole tube") covers this only generically.
- Probe P2 (`out/p2_shard*.json`): 600 gate scenes; 793 rays per beam (16-point ring plus longitudinal and mixed
  offsets); a = 0.999 eps; measured maximum offset 0.999 mm.
  - `-DIR-NATIVE`: 0 FALSE_CERT, 1,321 certified;
  - eps = 0 control: 843 false certificates (827 with the gate's family on the same scenes).
- The repaired certifier holds under this stronger in-bound adversary too.

**m4. The non-vacuity check is coarse.**
- Code: `check_direction_bound_full.py:286`.
- The repaired configuration never certifies an aimed candidate, which is expected: every aimed candidate is within
  R + 0.3 mm, inside its R + 2 eps margin. It also certifies nothing in the two ineligible bands, by design.
- Its 4,435 certificates all come from random segments. The smallest margin is 1.9 mm above R; 12 are within 5 mm
  and 30 within 1 cm.
- So the 0 FALSE_CERT on aimed candidates is structural. The discriminating content is that the repaired
  configuration refuses the 155 segments the control falsely certifies at eligible heights, for in-mechanism reasons:
  - UNRESOLVED_CENTER_CELL 117;
  - OBSERVED_SURFACE_INTERSECTION 38.
- That content holds. `> 0 certified` alone would not show it.

### Informational

- **i1.** `R = 0.45` is hard-coded in the gate (`check_direction_bound_full.py:44`) and not read from the registry.
  It equals `plan_radius` (`registry.py:94`) for every configuration used, so FALSE_CERT is defined correctly.
- **i2.** The input binding does not include the preregistration documents. `run_m2.inputs`
  (`run_m2.py:47-59`) hashes the `cplanner/` tree, the kernel, the trace files and the interpreter only. I checked
  the three `docs/plans` files against the commit blobs of `37762ab` (below).
- **i3.** The native build runs outside the per-command host check and without an explicit `taskset`
  (`run_dirbound_d1f.py:62-63`). Its CPU affinity is inherited from however the driver was launched, which is not
  recorded. The fault count was 0 throughout, so nothing follows from it.
- **i4.** "Most permissive = largest nearest-hit distance" (`check_direction_bound_full.py:100-106`) is not a total
  order in general. A shorter range moves the surface point. Here the candidates start at the sensor, so the farthest
  hit is also the farthest from the segment, and the choice is appropriate. P5 (off-sensor candidates, the same
  declaration) gave no counterexample. Each beam's declaration is chosen independently, which is stronger than a
  physical error.
- **i5.** The offset check samples t in {0, 3, 6, 12} (`:95-96`). The offset e0 + t (w − d) is affine in t, so its
  norm is convex and the endpoints bound [0, 12]: the check is exact. I measured the same 9.900000000116e-4 m
  independently.

## Answers to the questions

**Q1. Adversary and declaration: yes.**
- The family is the preregistered one (`:70-76, 87-96`):
  - e0 and e12 range over 0 and ±a in the directions u, v, (u+v)/√2 and (u−v)/√2;
  - a = 0.99 eps (`:46`); u is the horizontal normal of d and v = d × u;
  - this gives 81 rays: origin offsets (e0 = e12), tilts (e0 = 0) and pivots (e0 = −e12), in eight directions of the
    normal plane.
- Maximum offset: 0.99 mm, both the gate's value and my independent measurement.
- The declaration (`:100-106`):
  - NO_HIT with range 12 if any ray has no hit within 12 m;
  - otherwise HIT at the raw largest distance;
  - NEAR_CLIPPED below 0.2 m, which never occurs because bars are at least 0.3 m away.
- The declaration matches the core's validation rules (`core.py:137-142`). All 8,000 steps were accepted.
- The gate's manifest `MAN` is bitwise equal to `registry.directions("MOTAR-72x4")`. Its world rotation is the core's
  (`core.py:151`).
- Gaps: no offsets along d (m3), and the shape shrink is not exercised (m1).

**Q2. Ground truth: independent and correct.**
- The gate uses no code from `src/`, only the status constants, eps and the catalogue (by design).
- Caster (`:54-67`). I compared it with an independent face-enumeration caster:
  - every declaring ray and every exact ray of all 2,000 scenes: 0 mismatches, largest range difference 0.0;
  - all 81 × 288 family rays of 40 scenes (933,120 rays): 0 mismatches.
- Segment-rectangle distance (`:110-132`). I compared it with an independent edge-pair distance:
  - largest difference 4.1e-15 m over 31,673 segments;
  - 0 disagreements on the test clearance ≤ R.
- FALSE_CERT (`:227`) is "certified and clearance ≤ R" with R = plan_radius. The ≤ makes touching count as a
  violation, which is conservative.
- Aimed candidates (`:161-181`):
  - for all 23,673, the clearance to the target bar is R − delta within 6.8e-15 m;
  - delta lies in [−0.2999, 1.4998] mm;
  - 5,974 pass another bar even closer, and the gate correctly uses the minimum;
  - 20,724 have clearance ≤ R.

**Q3. Discrimination: yes. The PASS is not an artefact (with the limits in m1, m2 and m4).**
- The eps = 0 control has 2,880 false certificates, 75 of them at 0.3-2.7 m. These are genuine in-bound
  counterexamples: the declarations come from rays within 0.99 mm of the manifest ray, and the clearances are
  confirmed independently.
- The ordinary-height detections are lateral (m2, P4).
- The repaired configuration certifies 4,435 segments at the three eligible height bands (2,183 / 1,052 / 1,200) with
  0 FALSE_CERT. Its refusal reasons are in-mechanism (table below).
- Mutation scan (P1, P3), on the gate's own scans. Columns: FALSE_CERT in total / at eligible heights / at 0.3-2.7 m.

| Certifier (height shrink, shape shrink, radius growth) | Total | Eligible | Mid |
| --- | --- | --- | --- |
| ZERO (0, 0, 0) | 2,880 | 155 | 75 |
| M_H (eps, 0, 0) | 155 | 155 | 75 |
| M_R0 (eps, eps, 0) | 156 | 156 | 76 |
| (eps, 0, 0.10 mm) | 137 | 137 | 66 |
| (eps, 0, 0.25 mm); with shape shrink | 101; 99 | 101; 99 | 48; 47 |
| (eps, 0, 0.50 mm); with shape shrink | 43; 43 | 43; 43 | 22; 22 |
| (eps, 0, 0.75 mm); with shape shrink | 8; 8 | 8; 8 | 3; 3 |
| (eps, 0, 0.90 mm); (eps, 0, 0.99 mm) | 0; 0 | 0; 0 | 0; 0 |
| M_S0 (eps, 0, 2 eps) | **0** | 0 | 0 |
| M_RS2 (eps, eps, √2 eps), the argued minimum | 0 | 0 | 0 |
| **DIR (eps, eps, 2 eps)** | **0** | 0 | 0 |
| M_L (0, eps, 2 eps) | 2,611 | 0 | 0 |
| M_HALF (eps/2, eps/2, eps) | 1,396 | 0 | 0 |
| M_09 (0.9, 0.9, 1.8) eps | 318 | 0 | 0 |
| (0.95 eps, eps, 2 eps) | 128 | 0 | 0 |
| (0.98 eps, eps, 2 eps) | 11 | 0 | 0 |

- Per height band, D1-F scans:

| Band | Scenes | ZERO false | DIR certified (exact scan) | DIR reasons on ZERO's false certificates |
| --- | --- | --- | --- | --- |
| mid 0.3-2.7 | 800 | 75 | 2,183 (2,182) | UNRESOLVED 62, SURFACE 13 |
| top eligible 2.95-2.999 | 385 | 32 | 1,052 (1,048) | UNRESOLVED 18, SURFACE 14 |
| top ineligible 2.999-2.99999 | 171 | 1,370 | 0 (0) | NO_VALID_RAYS 1,311, SURFACE 59 |
| low eligible 0.001-0.05 | 450 | 48 | 1,200 (1,200) | UNRESOLVED 37, SURFACE 11 |
| low ineligible 0.00001-0.001 | 194 | 1,355 | 0 (0) | NO_VALID_RAYS 1,300, SURFACE 55 |

**Q4. Driver and provenance: consistent. The outputs support the summary.**
- **Frozen copy.** 117 of the 118 files in `frozen_d1f/` are byte-identical to the blobs of commit `37762ab`. The
  118th, `FROZEN_COMMIT`, is added by the freeze.
  - Method: `git hash-object` against `git ls-tree` with `GIT_OPTIONAL_LOCKS=0`, reading the object store only; I ran
    no command in the worktree.
- **Carry-over.** `src/`, `native/` and `motar/` (12 files) are identical between `frozen/` and `frozen_d1f/`
  (`diff -r`). `git diff 8e2b77d 37762ab` is empty for these paths. `meta.json` records `carry_over_identical: true`,
  0 differences.
- **Order of events.** Commit `37762ab` (preregistration and gate) is dated 14:02:19. The run started at 14:02:27.
  The only dev output (`dev/d1f_dev01.json`, seed 20,261,199, 120 scenes) matches the disclosed table.
- **Library.** I rebuilt it from the frozen `build.py` into `audit/native/`: sha `afbd3e68…`, byte-identical to the
  run's library.
  - The run's library is bound in inputs before and after.
  - The gate records it as the loaded library.
  - A missing library in a worker would have failed `EXACT_nonvacuous_certified`.
  - `VERB_identical` held on 100 scenes (1,579 segments).
- **Host.** Before and after both commands: fault count 0, the same boot id, online CPUs `0-3,6-23` (`records.json`,
  `host_start.json`, `d1f_summary.json`).
- **Duplicates and determinism.** Both runs are identical after removing `wall_s` (I checked with the driver's
  `strip_timing` rule). My third run, with 3 shards instead of 24 chunks on 6 workers, reproduces every gated
  statistic exactly.
- **Input binding.** `inputs_before.json` equals `inputs_after.json` (181 entries), and both still equal the files
  today. `SHA256_MANIFEST.json`: all 16 entries match, and `run_copy/` is identical to `run_d1f/`.
- **Parameters.** 2,000 scenes, seed 20,261,112, 6 workers, configurations and checks are exactly as preregistered.
- **Minor gaps:** i2 and i3.

**Q5. Anything else:** m3 and P2 (longitudinal offsets and a denser family), P5 (off-sensor candidates), i1, i4, i5.
No in-scope defect of the certifier was found.

## What I verified and how

Probes ran on CPU only:
- pinning and environment: `taskset -c 0-3,8-11,14-23`, `CUDA_VISIBLE_DEVICES=`, `PYTHONNOUSERSITE=1`,
  `python -B`;
- at most 3 compute processes at a time;
- kernel fault record: 0 before (14:09) and 0 after (14:44), with the same boot id (`kernel_faults_*.txt`);
- the frozen copies, the run copy and the bound inputs were unchanged afterwards.

The probes load the frozen gate module and the frozen `src/` read-only (`probe_lib.py`). They use my rebuilt
library.

| Probe | What it checks | Result (file) |
| --- | --- | --- |
| P0 `p0_sanity.py` | vectorised scan bitwise equal to `gate.scan`; offsets; `MAN` against the registry; `MutCore` against `-DIR-NATIVE` and `-NATIVE` | all equal on 12 scenes (`out/p0_sanity.json`) |
| P1 `p1_full.py` + `p1_analyze.py` | all 2,000 scenes: reproduction; independent caster and clearance; aimed targets; per band; refusal reasons; 7 mutations | exact reproduction; 0 ground-truth mismatches (`out/p1_analysis.json`) |
| P3 `p3_thresholds.py` | detection thresholds for radius growth, shape shrink and height shrink | table above (`out/p3_merged.json`) |
| P4 `p4_forensic.py` | mechanism and validity of the 75 mid-height control false certificates | lateral, 0.81-1.02 mm entry, declaring rays ≤ 0.99 mm (`out/p4_forensic.json`) |
| P2 `p2_strong.py` | 793-ray family at 0.999 eps with longitudinal offsets, 600 scenes | DIR 0 FALSE_CERT, 1,321 certified (`out/p2_shard*.json`) |
| P5 `p5_offsensor.py` | aimed candidates not anchored at the sensor, 1,635 eligible scenes, 26,160 segments | DIR 0 certified, 0 false; M_S0 0 / 0; ZERO 403 false (`out/p5.json`) |

Code read in full:
- `check_direction_bound_full.py`, `run_dirbound_d1f.py`, `run_gate.py`, `run_m2.py`;
- `src/core.py`, `registry.py`, `coverage_b.py`, `coverage_native.py`, `evidence_map.py`, `boundary.py`;
- `native/coverage_b.c` and `native/build.py`.

The evidence map cannot veto in a single-step scene: the occupancy episode starts in the packet's own step
(`evidence_map.py:169`). The arena check cannot trigger either: poses are 8-32 m and segments at most 4 m. So neither
can cause a refusal for an unrelated reason.
