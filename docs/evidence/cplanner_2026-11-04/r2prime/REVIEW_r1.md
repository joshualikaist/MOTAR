# R3' independent review of the C-Planner P0-R r1 certifier and its R2' qualification run

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, 2026-11-04 KST.
- **Scope:** read-only review of `review_copy/` (commit `b407039`) and `r2prime_copy/` (frozen run output).
- **What I wrote:** only files under `audit/`.
- **Phases:** phase 1 reviewed code, tests, driver and records (`REVIEW_phase1.md`); phase 2 reviewed the run outputs
  and added re-execution probes. Phase 1 did not read the live run directory.
- **Probes:** CPU only, `taskset -c 0-3,8-11,14-23`, `CUDA_VISIBLE_DEVICES=`, `PYTHONNOUSERSITE=1`, one BLAS thread,
  `python -B`. At most 2 processes in phase 1 and 4 in phase 2.
- **Host:** the kernel journal showed 0 machine-check, segfault, general-protection or hardware-error lines over the
  whole run window and across all my probes.

## Verdict: **PASS** — no in-scope P0-blocking finding (charter §5)

**Q1. Soundness.**
- I found no truthful packet and segment for which the certifier certifies while a catalog bar is within 0.45 m.
  Section 1 gives a proof sketch under charter §4.
- My own ray caster and oracle fuzz (`p2`) found 0 FALSE_CERT among 3,825 certificates (2,000 scenes, 16,000
  segments).
  - It covered all five configs, including the MOTAR runtime arena config.
  - Heights were at, near and outside the (0, 3) m boundaries.
  - Low blockers sat in front of the 0° row, and segments were arbitrary, grazing, or behind bars.
- The same fuzz catches three known-unsound mutants:

  | Mutant | Change | FALSE_CERT |
  | --- | --- | --- |
  | M1 | no elevated-row clipping | 154 in 600 scenes |
  | M0 | no height check (F1) | 25 in 300 |
  | M2 | free interval extends past the hit | 8 in 300 |

**Q2. Binding and monotonicity.**
- Every refusal clears the current packet.
- `for_step` must be the packet's exact int.
- The camera reaches only the label layer, after the commit.
- A track can only add STATIC/CONFLICT vetoes.
- No path to a stale certificate, a usable packet after a refusal, a camera effect, or a track gain exists in the
  code, and none appeared in my sequences (`p3`).

**Q3. Earlier findings F1-F6.** All stand closed. F1-F5 are closed in the code. F6 is closed by 5B, 5C and 5A's
random height, and phase 2 confirms it on the outputs.

**Q4. Test adequacy and provenance.**
- Every rule's PASS is supported by its JSON, with non-vacuous counts.
  - Duplicates agree, which I recomputed myself.
  - Inputs are unchanged.
  - The frozen tree is `b407039` file for file.
- I re-executed 72 non-duplicated 5A seeds and 12 non-duplicated 5B seeds from the review copy. All 84 per-seed
  certificate digests equal the run's.
- The suites detect the relevant defect classes:
  - 5A finds 12 FALSE_CERT on mutant M1 in 1,000 of its own scenes.
  - 5B finds 82 track gains on track-lift mutant M4 in 100 of its own sequences.

**Q5. Other in-scope issues.** None that a PASS would hide.

**Remaining findings.** All are minor or out of scope. One is new in phase 2: CPUs 4-5 were taken offline during the
run (M8 below).

Reading of the gate: R2' satisfied its preregistered CONDITIONAL criteria, and with this review there is no new
in-scope P0-blocking finding. The PASS is conditional on the declared assumptions (charter §4) and on the limitations
below. It is not a safety, real-flight, dynamic-object or real-time claim.

---

## 0. Provenance

**Review copy (`review_copy/`).**
- `SHA256_MANIFEST_review_copy.json`: 37/37 files verify.
- Byte-identical to `git show b407039:<path>` for every file, and the commit has no file outside the manifest.
- `src/` is byte-identical to the r1 unit `~/motar_tmp/p0r_r1_20261103/src`.
- Verified by `p1` with my own parse and regex:
  - `coverage_b`'s four functions are AST-identical to the v3-v7 `certify_b.py` files.
  - The catalog equals the 40 `bars_h3` URDF boxes (3.0 m tall, rpy 0).
  - The manifest directions are exact.

**Run copy (`r2prime_copy/`).**
- `SHA256_MANIFEST_r2prime_copy.json` (sha256 `525608784a3c…`): 63/63 files verify, with no extra or missing files.
- All 19 `records.json` `output_sha256` values match their files.

**Frozen tree.** `inputs_before.json` lists 35 frozen-tree files: the 34 files of `review_copy/cplanner/` with
identical SHA-256, plus `FROZEN_COMMIT` = `b407039 b407039347…`. No other file is present, so none of the later
untracked or committed M2/M3 files were in the tree.

**Inputs.**
- `inputs_before == inputs_after` (89 entries).
- The 54 external entries are unchanged, and they still match the files on disk today: v3-v7 references, v7 run 8 JSON,
  `occlusion_trace.py`, 40 URDFs, the B1-R trace files, and the interpreter.
- `check_r1_core.source` and `check_r1_temporal.source_dir` point at `frozen/cplanner/src`. The source hashes that
  `check_r1_temporal` recorded equal the review copy's.

## 1. Q1 — soundness (scope 1-2)

### Argument (`src/core.py`, `src/coverage_b.py`)

Setting: a complete packet (`core.py:144`), truthful statuses, an exact pose, and 0 < z < 3 (`core.py:211`). Each
free segment (`:157-160`) is the projection of a 3-D ray piece t ∈ [a, b] with these properties:

- a = 0 or min_range (`:150`).
- b = min(f1, hi), where f1 = r - 1e-7 (HIT) or 12 (NO_HIT) (`:151, :158`).
- Every point of the piece is at a height in [0, 3] (`:215-217`).
- Truthfully, no object lies on it.

If any of these points fell in the open footprint of a full-height catalog bar, the 3-D point would lie inside the
bar. The ray would then have hit the bar earlier, which contradicts the declared status.

`_coverage_B` (`coverage_b.py:41-73`) excludes a centre cell only when a free segment passes strictly (1e-9) through
[c ± (h - e)]. That box lies inside the footprint of every bar centred in the cell. The search:

- starts from all centres within R + h,
- prunes only cells farther than R + 1e-9,
- refuses on depth, resolution or node budget.

The surface test (`:45`), the map veto, the arena check and the range bound (`core.py:262-294`) can only remove
certificates.

Regimes in Q1:

- **Elevated rows.** All four rows share the same azimuths. For full-height bars, an elevated row's clipped free
  projection is a subset of the 0° row's. Elevated rows add evidence only past non-bar objects, and only inside the
  0-3 m band.
- **Blind zone.** Assured configs start free space at 0 and need NEAR_CLIPPED to make the packet incomplete, which
  both casters do. The NOASSURE config starts at 0.2 m.
- **No-hit fills.** A NO_HIT range must equal 12.0 exactly (`:137`).
- **Map.** Veto only (`:292-294`).
- **Arena.** Both endpoints must lie R inside the arena, and the segment is convex (`:288-291`).
- **Incomplete packets.** They add no free rays and are refused (`:157`, `:260`).

### Evidence

| Probe | Size | Certified | FALSE_CERT | Notes |
| --- | --- | --- | --- | --- |
| `p2` own caster/oracle, actual code | 2,000 scenes / 16,000 segments | 3,825 | **0** | 238 certified with a bar at 0.45-0.55 m; 39 where the elevated rows were decisive; 0 certified outside 0 < z < 3; 0 exceptions |
| `p2` on mutant M1 (no clipping) | 600 / 4,800 | 1,483 | 154 | positive control |
| `p2` on mutant M0 (no height check) | 300 / 2,400 | 686 | 25 | all at heights outside (0, 3) |
| `p2` on mutant M2 (HIT free + 0.3 m) | 300 / 2,400 | 621 | 8 | positive control |
| R2' 5A (run output) | 20,000 / 160,000 | 105,509 | **0** | VALID 94,494; OUT_OF_SCOPE 11,015; 35,033 justified refusals |
| R2' 5B (run output, 4 runs) | 2,000 × 30 packets / 112,000 per run | base 76,257 VALID | **0** in all runs | |
| R2' regress (run output) | 4,000 + 960 trials × 2 modes | 1,897 / 456 / 382 per mode | 0 | flags and reasons identical to v7 run 8; certified counts equal v7 (I recounted them from v7 `regress_boundary.json`) |
| R2' scenarios / attacks / 5C (run output) | 114 segments / 117 cases / 6 cases | | 0 | all pass |

## 2. Q2 — temporal binding and monotonicity (scope 3-4)

Code:

- Every refusal path calls `_refuse`, which clears `_current` (`core.py:99-124`). Exceptions also clear it
  (`:94-97`).
- The commit happens after all validation (`:185-187`).
- `certify` captures the packet once (`:249`) and requires an exact int `for_step` (`:258`). Every result is bound to
  the captured packet (`:234, :250, :283`).
- Steps strictly increase (`:116`). `_last_step` advances only on acceptance (`:186`).
- The camera is used only in `TargetLabelLayer.update`, after the commit, in its own `try` (`:188-197`). No
  certify-path method references the labels, the camera or the track (`p1`).
- The track changes only `EvidenceMap.static` (`evidence_map.py:88-91, 116-129`), so the track core's veto masks are
  a superset of the base core's (`:167-172`).
- A malformed camera or track is ignored and cannot refuse a step (`boundary.py:21-34`).

`p3` part 1: after an accepted step, all 16 refusal kinds clear the packet for both the old and the new step. A
refused step does not advance the counter. Every non-int or wrong `for_step` is refused. `map_states` has no effect on
certify output.

`p3` part 2 used 24 hostile sequences:
- Setup: a moving and yawing vehicle; 12 % incomplete and 5 % refused packets; hostile and truthful cameras; hostile
  and truthful tracks; a fresh core per packet for comparison.
- Camera diffs 0, with 2,564 labelled returns.
- Track gains 0.
- History vs fresh core: 0 gains and 267 losses.
- Wrong step bindings 0.
- FALSE_CERT 0.

R2' run outputs:
- 5A: 0 camera diffs over 7 variants and 20,000 scenes; track+camera equals track; 0 track gains.
- 5B: 0 camera diffs; 0 track gains against 1,833 track losses, all `MAP_VETO_CONFLICT`.
- 5C: all six cases pass.

## 3. Q3 — earlier findings F1-F6 (all stand closed)

| # | State | Code | Output evidence (R2') |
| --- | --- | --- | --- |
| F1 | closed | `core.py:211-212` strict 0 < z < 3 for every layout | `check_r1_core.json`: 7 tests, 16 height cases, 6 valid, 0 bar-top false certificates; `p2`: 0 certificates outside the interval; mutant M0 is detected |
| F2 | closed for numpy/warnings hooks | `core.py:90, 227, 306`; `_result(packet=cur)` | `check_r1_core`: 2 callback positive controls, 0 internal exceptions; `p5`: 0 hook calls inside, settings restored, identical under `seterr(raise)` |
| F3 | closed (synchronous) | `boundary.py:31, 48` | `check_r1_core` copy-corruption test passes |
| F4 | closed as documented | `R1_INTERFACE_AMENDMENT_20261103.md:7-14` | — |
| F5 | closed | `core.py:274-275` | `check_r1_core` late-certificate test passes |
| F6 | closed | 5B, 5C, 5A random z | See below |

F6 output evidence:
- 5B, over 2,000 sequences: STATIC in 1,673 (83.7 %, threshold 25 %), CONFLICT in 408 (20.4 %, threshold 10 %),
  track loss in 292 (14.6 %, threshold 5 %).
- The 200-sequence duplicate: 166, 45 and 32 respectively.
- 5C: STATIC at packet 21 and CONFLICT at packet 23 in all six cases.
- 5A: z ~ U[0.05, 2.95], and the suite is sensitive to clipping defects (`p4`).

## 4. Q4 — do the outputs support every rule's PASS?

| Rule | Preregistered condition | Output (main run; duplicate where relevant) | Supported |
| --- | --- | --- | --- |
| 1-2 | AST, catalog, static camera separation | `check_static.json`: all four booleans true; my re-run is identical | yes |
| 3/8 | 4,000 + 960, 0 FALSE_CERT, flag and reason = v7, counts = v7 | 5 checks true; 4000/4000 and 480/480 identical; counts 1,897 / 456 / 382 equal v7; reference SHA `181636e3…` = bound input | yes |
| 4 | attacks + r1 regressions, 0 internal exceptions | attacks 117 rows: SAFETY 106/106, CONTRACT 10/10, CONTROL 1/1 (1 big-endian int8 row skipped by design); R4 3,000 calls, 0 escaped, 0 logged. `check_r1_core` 7/7 tests, 0 internal exceptions; my re-run is identical | yes |
| 5A | 20,000 scenes, U[0.05, 2.95], 7 cameras identical, track+camera = track, 0 track gains, 0 FALSE_CERT, 0 exceptions, positive controls | 8/8 checks; 20,000 scenes, 160,000 segments; `camera_diff` {}; FALSE_CERT 0; VALID 47,264 / 47,230; 115,045 truthful labels; the 4 modes about 5,000 each | yes (5A's own track check is vacuous by design, minor M4) |
| 5B | 2,000 × 30, monotone, camera identity, FALSE_CERT 0 in all runs, static only with a track, non-vacuity thresholds, positive controls | 11/11 checks; numbers in section 3; `baseline_static_cells_total` 0; VALID 38,321 / 37,936; 47,151 labels | yes |
| 5C | deterministic witness | 6 cases, all 10 checks each; my re-run is identical | yes |
| 6/8 | scenarios unchanged | 114 segments, 0 failures, 0 FALSE_CERT, open scenes 200/200 per layout, moving-target 24/24 | yes |
| 7 | 60 sequences, 0 violations | 60 sequences, 1,800 steps, 1.16e8 cell checks, 0 violations. My instrumented copy (`p7`) reproduces the output exactly and shows I1 was exercised (210 STATIC cells in 56 of 60 sequences) but I2 only barely (1 CONFLICT cell in 1 sequence) | yes (I2 evidence is thin, minor M3) |
| 9 | timing reported | REPORTED, last command. K = 8: step + certify p95 464 ms (72x1) and 625 ms (72x4); with a 50 ms deadline, 4,514/4,800 and 4,553/4,800 return DEADLINE_UNKNOWN | reported only |
| 10 | kernel fault record and boot id unchanged | boot id `407ae422…` at start and end; every record has `host_valid`, faults 0→0; journal clean | yes, but see M8 |
| 11 | duplicates agree | I recomputed: the 7 deterministic suites are equal after `strip_timing`; 5A 2,000/2,000 and 5B 200/200 per-seed digests equal; all `digest_of_digests` recompute | yes |
| 12 | explicit boolean, exit 0, inputs unchanged | 18 PASS + 1 REPORTED, all exit 0, all under timeout (5A 11,081 s of 21,600 s); `inputs_unchanged` true | yes |

**Independent re-execution** (`p6`, 4 processes, from `review_copy/`):

| Suite | Seeds | Digests equal to the run | Stats |
| --- | --- | --- | --- |
| 5A | 72 seeds outside the duplicate prefix (2,160-19,999, including 19,999) | 72/72 | 0 FALSE_CERT, 0 camera diffs |
| 5B | 12 seeds (362-1,999) | 12/12 | 0 FALSE_CERT, 0 track gains, 15 losses |

This adds reproduction evidence for the 90 % of 5A and 5B that the gate did not duplicate. `p8`: my re-runs of
`check_static`, `check_r1_core` and `check_r1_temporal` equal the run's JSON, apart from the source-path fields.

**Test sensitivity** (phase 1, `p4`, the suites' own generators and oracle):
- 5A on M1: 12 FALSE_CERT in 1,000 scenes.
- 5B on M4 (hits near a fresh track not recorded as occupancy): 82 track gains in 100 sequences.

## 5. Q5

Nothing in scope that a PASS would hide.

Context for readers of the run:
- 5A certifies 11,015 segments as OUT_OF_SCOPE: a non-catalog pole or the target, between beams, lies within R.
- 5B has about 2,500 such segments.
- This is the declared `non_catalog_objects / dynamic_objects: UNVERIFIED` scope, reported and never counted as safe.
- It means the certificate frequently certifies segments that pass near small or dynamic objects. Any planner use must
  keep the separate dynamic-object and terminal handling (charter §1, `TERMINAL_INTERCEPTION_SPEC.md`).

---

## Findings by severity

**P0-blocking (in scope):** none.

**Important:** none.

**Minor.** Phase-1 status is given for each; all still stand except where noted.
- **M1 — stands; no effect on this run.** `run_gate.py:145-150` accepts any explicit `True` and checks no counts.
  `check_r1_core`, `attacks_p0r` and `map_invariants` could pass with nothing run. I checked every count in the outputs
  (section 4), and all are the full planned sizes.
- **M2 — stands; no effect on this run.** The driver records but does not verify the frozen commit. numpy, the stdlib
  and `motar_eval/__init__.py` are not hashed, and the numpy version is not recorded (`run_gate.py:76-89, 169-172`).
  I verified the frozen tree against `b407039` file for file (section 0).
- **M3 — narrowed by measurement.** `map_invariants.py` records no non-vacuity count for I1 or I2.
  - `p7` shows that in the gate's 60 sequences I1 (STATIC permanence) was exercised: 210 cells in 56 sequences.
  - I2 (CONFLICT permanence) was exercised on only 1 cell in 1 sequence.
  - CONFLICT persistence is also covered by 5C (final state CONFLICT) and 5B (1,833 `MAP_VETO_CONFLICT` losses).
    Map behaviour can only remove certificates.
- **M4 — stands, confirmed in the outputs.** 5A's `STEPS_A = 2` is below N_STATIC = 3 (`noninterference_v2.py:40`).
  5A shows track loss 0 over 160,000 segments, so its track checks see no track effect. 5B carries that evidence.
- **M5 — stands.** The gate never certifies with `MOTAR-72x4-RUNTIME` (arena, no assurance, runtime health). Its
  randomized suites use only sensor-anchored segments, and 5B uses only truthful tracks and cameras. My `p2` and `p3`
  covered these cases with 0 violations. The config differences are monotone restrictions.
- **M6 — stands.** `run_gate.py:1` and `noninterference_v2.py:1` cite the nonexistent
  `docs/plans/cplanner_r2prime_rules_v1.md`. The rules are in `prereg_cplanner_r2prime_v1.md`.
- **M7 — stands.** The preregistration's statement that the 5B fractions were fixed before the dry runs cannot be
  checked from git: one commit, `b407039`, holds both. I did not read the dry-run directory, which is outside the
  allowed scope.
- **M8 — new in phase 2: a host-state change during the run that the gate does not record.**
  - What happened: `host_start.json` shows CPUs `0-23` online. `gate_summary.host_end` shows `0-3,6-23`, with 4-5
    offline. The journal shows `sudo tee /sys/devices/system/cpu/cpu{4,5}/online` from an interactive terminal
    (pts/0) at 01:46:35 and 01:46:49 KST, followed by `smpboot: CPU 4/5 is now offline`. This was during command 8
    (5A, 01:29-04:34).
  - It is a deliberate hotplug of the known-faulty cores, not a kernel fault record. So rule 10 as preregistered (fault
    record and boot id) is satisfied, and the run is not VOID.
  - The driver's `host_valid` (`run_gate.py:134`) does not compare the online CPU list, and the records hold no
    per-command CPU state, so the run outputs do not disclose the change.
  - Every command was pinned to `0-3,8-11,14-23`, which excludes cpu4-7.
  - Effect on the evidence: none found. The 5A duplicate (run later) matches seeds 0-1,999. My re-run of 72 further 5A
    seeds matches. The deterministic suites match.
  - Recommendation: record and compare the online list per command, and record operator host actions in the run notes.

**Out-of-scope limitations (never P0-blocking under charter §5)**
- **L1.** Other process-global hooks still run caller code synchronously inside the core: `gc.callbacks` was called
  4.4 M times in `p5`, and `sys.setprofile` or audit hooks would be the same. This is in-process adversarial Python.
- **L2.** Statuses, health, pose and the yaw-only LiDAR attachment are trusted declarations. The 0-3 m `bars_h3`
  prior is a registry constant; binding it to the run's bar pool is a P1 prerequisite (charter §4).
- **L3.** No real-time guarantee. `deadline_s` is decision expiry only. Rule 9 timing is far above the charter's
  G-beta 50 ms target (p95 464-625 ms for K = 8), which is a matter for M3 (G-beta), not R3'.
- **L4.** Exact grazing of the 0 m and 3 m planes and of footprint boundaries (the 1e-9 / 1e-7 margins) is not covered.
- **L5.** Certificates make no claim about non-catalog or dynamic objects. OUT_OF_SCOPE certificates are frequent
  (section 5).
- **O1 (observation).** A Python call-level error (`TypeError` before the method body runs) leaves the previous packet
  current. The packet stays bound to its own step, and `certify` for any other step is refused. This is not a refused
  step and not a binding defect.

## What I verified and how

| Probe (under `audit/`) | What it checks | Result |
| --- | --- | --- |
| `p1_static_checks.py` | AST identity, URDF catalog, manifest, certify-path references | all identical / none |
| `p2_fuzz.py` | own-oracle soundness fuzz, plus mutants M0, M1, M2 | 0 FALSE_CERT on the actual code; mutants detected |
| `p3_temporal.py` | refusal clearing, binding, camera, track and history monotonicity | all hold |
| `p4_suite_sensitivity.py` | 5A on M1, 5B on M4 | both detected |
| `p5_hooks.py` | F2 closure and the `gc.callbacks` limitation | closed; limitation recorded |
| `p6_rerun_digests.py` (seeds in `p6_seeds.txt`) | re-execution of 72 + 12 non-duplicated seeds | 84/84 digests equal |
| `p7_map_invariants_counts.py` | instrumented rule-7 copy | identical output; I1/I2 coverage counts |
| `p8_compare.py` | re-runs of 3 deterministic suites vs the run | identical |

Phase-2 output checks: manifests and output hashes, the frozen tree against the commit, inputs before and after, host
records and the journal, records (argv, exit codes, timeouts, order), the gate summary, and every suite JSON's checks
and counts. I recomputed the determinism comparisons and the `digest_of_digests` values myself (section 4).

Mutant source copies (`audit/mut_M0`, `mut_M1`, `mut_M2`, `mut_M4`) are positive controls only. Each differs from
`review_copy/cplanner/src` by one line, or two lines for M4.
