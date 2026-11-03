# R3' independent review — phase 1 (code, tests, gate driver, design records)

Reviewer: **Claude Opus 5.5** (`claude-opus-5-5`), fresh context, 2026-11-04 KST, phase 1 of `REVIEW_PHASES.md`.
Read-only on `review_copy/`. Everything I wrote is in this `audit/` directory: probe scripts `p1`-`p5`, their outputs,
and four mutant source copies `mut_M0/ mut_M1/ mut_M2/ mut_M4/`, used only as positive controls. I did not open the
live run directory `cplanner_r2p_20261104/r2prime/` and did not touch its processes. Probes ran on CPU only, with
`taskset -c 0-3,8-11,14-23`, `CUDA_VISIBLE_DEVICES=`, `PYTHONNOUSERSITE=1`, one BLAS thread,
`/home/fair/miniconda3/envs/aerialgym/bin/python -B`, and never more than 2 processes at once. The host kernel
journal showed 0 machine-check, segfault or hardware-error lines before and after my probes; cpu4-5 were offline
throughout.

## Preliminary in-scope P0-blocking finding: **NO**

This covers phase 1 only: code, tests, driver and records. The run outputs have not been reviewed yet.

- **Q1 (soundness).** I found no truthful packet and segment that certifies with a catalog bar within 0.45 m, and I
  give a proof sketch (section 1) that none exists under charter §4.
  - My own caster/oracle fuzz (`p2`) ran 2,000 scenes and 16,000 segments, using all five registered configs.
    Heights were taken at the (0, 3) boundaries, near them and outside them. Scenes had low blockers in front of the
    0° row, and segments were arbitrary, grazing or behind bars.
  - Result: 3,825 certified, **0 FALSE_CERT**. 238 certificates had a bar at 0.45-0.55 m, and 39 certificates
    depended on the elevated rows' clipped free evidence.
  - The same fuzz finds the known-unsound mutant M1 (154 FALSE_CERT in 600 scenes), M0 (25 in 300) and M2 (8 in 300).
- **Q2 (binding and monotonicity).** No path was found.
  - Every refusal reason clears the current packet.
  - The camera reaches only the label layer, after the commit.
  - A track can only add STATIC and CONFLICT vetoes, so track vetoes are a superset of base vetoes.
  - Measured in my sequences (`p3`): camera diffs 0, track gains 0, history gains over a fresh core 0
    (history losses 267, so this check is not empty), and 0 certificates bound to a different step.
- **Q3 (F1-F6).** F1-F5 are closed in this code. F6 is closed in design: the 5B and 5C designs are non-vacuous, and
  5A now varies height. The output side of F6 is checked in phase 2.
- **Q4/Q5.** I found no vacuous test that would hide a scope 1-4 defect.
  - 5A detects the elevated-row clipping mutant: 12 FALSE_CERT in 1,000 of its own scenes.
  - 5B detects a track-lift mutant: 82 track gains in 100 of its own sequences.
  - Remaining gaps are minor coverage and provenance limits, listed below. None can produce a false PASS of rules
    1-4 given the code as reviewed.

---

## 0. What I reviewed (provenance)

- `SHA256_MANIFEST_review_copy.json`: all 37 files verify (`sha256sum -c`, 0 mismatches).
- The review copy is exactly commit `b407039`. `git show b407039:<path>` on all 37 files gives 0 differences, and
  `git ls-tree b407039 cplanner/` lists no file that is missing from the manifest.
- `cplanner/src/*.py` and the r1 test files are byte-identical to `~/motar_tmp/p0r_r1_20261103/{src,tests}`, as the
  preregistration states.
- `p1_static_checks.out` (own parse and regex, not `check_static.py`):
  - `coverage_b.py`: its four functions are AST-identical to all five v3-v7 `certify_b.py` references. The module
    contains nothing else but `import math`.
  - `registry.BAR_HALF_SIZES` equals the 40 `bars_h3` URDF boxes exactly. Every box is 3.0 m tall, with origin and
    rpy `0 0 0`.
  - The manifest directions equal an independent formula (max abs difference 0.0).
  - The MOTAR 0° row has `dz == 0.0` exactly, so its free rays are never clipped.
- For phase 2: the live worktree `src/MOTAR-cplanner` now has untracked files: `cplanner/native/`,
  `cplanner/planner/`, `cplanner/src/coverage_fast.py`, `cplanner/src/coverage_native.py` and
  `cplanner/tests/m3_equivalence.py`. None of them is in `b407039`. Phase 2 must confirm that none is in the frozen
  tree that the gate hashed and ran.

## 1. Q1 — soundness (scope 1-2)

### 1.1 Argument (code: `src/core.py`, `src/coverage_b.py`)

Take a complete packet (`core.py:144`): every beam validated as HIT or NO_HIT. Statuses are declared truthfully, the
pose is exact, and the sensor height satisfies 0 < z < 3 (`core.py:211`; otherwise `hi = 0`, so there is no free
segment). Each free segment (`core.py:157-160`) is the horizontal projection of a 3-D ray piece {o + t d : t in [a, b]}
with these properties:

- a = 0 (assured) or min_range (`:150`). These are monotone choices.
- b = min(f1, hi) (`:158`), where f1 = r - 1e-7 for a HIT and 12 = max_range for a NO_HIT (`:151`). The truthful
  nearest object along d is at r (HIT) or beyond 12 (NO_HIT), so nothing occupies o + t d for t <= b.
- For t in [0, hi] the height z + t·dz lies in [0, 3]. This follows from `:215-217`. Rotation leaves dz unchanged,
  and `p1` confirms the 0° row has dz exactly 0.

Now take a catalog bar B = [cx±hx] × [cy±hy] × [0, 3]. If any projected free point lay in the open footprint of B, the
3-D point at that t would be inside B. The ray would then have hit B at some t' <= t <= b, which contradicts the
truthful status. Grazing exactly on the boundary is out of scope.

`_coverage_B` (`coverage_b.py:41-73`) marks a centre cell as excluded only if a free segment passes strictly (by
1e-9) through the box [c ± (h - e)]. That box lies inside the open footprint of every bar whose centre is in the cell.
The search starts from every centre that can bring the bar within R (the initial box is R + h wide), prunes only
cells whose union of footprints is more than R + 1e-9 away, and refuses at the resolution or depth limit. A
certificate therefore implies that no catalog-shape bar lies within R + 1e-9 of the segment. Two further layers only
remove certificates: the surface test (every HIT within R + 1e-9 vetoes, `coverage_b.py:45`), and the map, arena and
range-bound checks (`core.py:262-294`).

The same argument covers each regime in the question:

- **Elevated rows.** All four MOTAR rows share the 72 azimuths. For a scene of full-height bars, the clipped free
  projection of an elevated row is a subset of the 0° row's. Elevated rows add evidence only past non-bar objects,
  for example a target sphere, and only inside the 0-3 m band, where a bar would have been hit.
- **Blind zone.** With assurance off, the free segment starts at 0.2 m. With assurance on, it starts at 0. The
  harness and my caster report NEAR_CLIPPED for t < 0.2, which makes the packet incomplete, as the assurance
  declaration requires.
- **No-hit fill.** A NO_HIT counts only if its range equals the fill exactly (`:137`). Its free extent is max_range,
  clipped.
- **Map veto-only.** `veto_near` returns only a veto state. Nothing in `_certify_one` grants on its result (`:292-294`).
- **Arena.** Both endpoints must lie at least R inside the arena (`:288-291`). Since the segment is convex, the swept
  disc stays inside.
- **Incomplete packet.** It gives no free evidence (`:157`) and is refused before coverage (`:260`).

### 1.2 Fuzz of the actual certifier (`p2_fuzz.py`, `p2_real_0_2000.json`)

`p2_fuzz.py` has its own caster (slab boxes with a z range, spheres, z-limited cylinders), its own beam formula and an
exact segment-rectangle distance. It shares no code with `tests/harness.py` or `snapshot/indep.py`. Its scene families:

- five configs: `SYNTH-72x1`, `-NOASSURE`, `SYNTH-72x4`, `SYNTH-72x4-RUNTIME`, `MOTAR-72x4-RUNTIME` (arena,
  runtime health);
- heights: 40 % from {1e-9, 1e-6, 1e-3, 0.01, 0.05, 0.2, 2.8, 2.95, 2.99, 2.999, 2.999999, 3 - 1e-9}, 50 % uniform,
  10 % outside {-0.3, -1e-9, 0, 3, 3 + 1e-9, 3.05, 3.5};
- 0-10 bars, 25 % of them at 6-11.5 m in the beam gaps;
- in 60 % of scenes, 1-4 low spheres or short boxes at the sensor height, or thin poles, placed 0.4-2.5 m away;
- 8 segments per scene: sensor-anchored, arbitrary world segments, segments parallel to a bar side at 0.2-0.6 m, and
  segments behind bars.

| Run | Scenes / segments | Certified | FALSE_CERT | Certified with bar at 0.45-0.55 m | Elevated rows decisive | Certified outside 0 < z < 3 | Exceptions |
| --- | --- | --- | --- | --- | --- | --- | --- |
| actual code | 2,000 / 16,000 | 3,825 | **0** | 238 | 39 | 0 | 0 |

Per config (segments / certified / FALSE_CERT): 72x1 3200/735/0, 72x1-NOASSURE 3200/725/0, 72x4 3200/772/0,
72x4-RUNTIME 3200/810/0, MOTAR-72x4-RUNTIME 3200/783/0.

"Elevated rows decisive" means the 72x4 certificate is not obtained from its 0° row alone. I tested this by feeding
the 0° row to the 72x1 core with the same statuses.

### 1.3 Positive controls (the fuzz detects known-unsound code)

| Mutant (copy under `audit/mut_*`) | Change | Result |
| --- | --- | --- |
| M1 | elevated rows not clipped (`core.py:213` `if True:`) | 600 scenes: **154 FALSE_CERT** (`p2_mut_M1_0_600.json`) |
| M0 | no height eligibility (`core.py:211` `if False:`, the historical F1 class) | 300 scenes: **25 FALSE_CERT**, all at heights outside (0, 3); 94 certificates outside the interval (`p2_mut_M0_0_300.json`) |
| M2 | HIT free interval extends 0.3 m past the hit | 300 scenes: **8 FALSE_CERT** (`p2_mut_M2_0_300.json`) |

## 2. Q2 — temporal binding and monotonicity (scope 3-4)

Code reading:

- Every refusal in `_step` goes through `_refuse`, which sets `_current = None` (`core.py:99-124`). Any exception in
  `step` also clears it (`:94-97`).
- The packet is committed only after all validation and planning (`:185-187`).
- `certify` captures `cur` once (`:249`). It requires `type(for_step) is int` and equality with the packet step
  (`:258`). It binds every result, including exception results, to the captured packet (`:234`, `:250`, `:283`).
- Steps must strictly increase (`:116`). `_last_step` advances only on acceptance (`:186`).
- The camera is copied and used only by `TargetLabelLayer.update`, after the commit, inside its own `try`
  (`:188-197`). No certify-path method references `_labels`, the camera or the track (`p1`: `core_method_refs`).
  `map_states` reads labels but is not called by `certify` (p3 shows it has no side effect on certify output).
- The track affects only `EvidenceMap.static`, through `seps` (`evidence_map.py:88-91, 116-129`), and `static`
  only leads to CONFLICT. `occ_first/occ_last/free_since/vis/pending` do not depend on the track. So the track
  core's veto masks (`veto_near`, `:167-172`) are a superset of the base core's, and the coverage inputs are
  identical. Monotone by construction.
- A malformed camera or track is ignored and cannot refuse the step: `exact_copy` returns None and has no raising
  path on any ndarray (`boundary.py:21-34`).

Probe `p3_temporal.py` (`p3_temporal_0_24.json`):

- **Part 1.** After an accepted step 5, each of 16 refusal kinds clears the packet, and `certify` gives
  `NO_CURRENT_PACKET` for both step 5 and step 6. The 16 kinds cover four malformed arrays, env and episode
  mismatch, negative step, health 3, noise 2, replay, an older step, NaN pose, yaw 7, z 2e4, a timestamp off by
  1e-6, and a NaN timestamp.
  - A refused step 7 does not advance the counter: a later step 6 is accepted once, and repeating it is refused.
  - `for_step` values 4, 6, 5.0, np.int64(5), True, None and "5" are all refused.
- **Part 2.** 24 sequences × 25 packets on both layouts.
  - Setup: a moving and yawing vehicle at random or near-boundary heights, bars, a relocating pole and a moving
    target sphere. 12 % of packets were incomplete and 5 % refused. Cameras were hostile (non-finite, ±1e308,
    5e-324, float32, list, synced with extreme sigma) or truthful. Tracks were truthful or hostile (negative or
    1e308 radius, stale, future, inactive, NaN flag, wrong position, float32).
  - Runs: base, camera, track, track+camera, and a fresh core per packet.
  - Results: 3,000 segments. Certified: base, camera, track and track+camera 1,454 each; fresh 1,721.
  - Camera diffs 0, with 2,564 labelled returns. Track+camera vs track diffs 0. Track gains 0.
  - History vs fresh: 0 gains and 267 losses, so the map acted as veto-only and the check was not empty.
  - Wrong step binding 0. FALSE_CERT 0 in all five runs. Exceptions 0.
  - Limitation of my probe: it produced no track loss (only 29 STATIC cells, no CONFLICT). The non-vacuous track
    evidence is 5B/5C's job; section 4 measures 5B's sensitivity.
- **Call-level `TypeError`.** A missing argument or an unknown keyword raises in the caller before the body runs,
  so the previous packet stays current. `certify(paths, 5)` still certifies with `valid_for_step = 5`. This is not a
  refused step, and the result stays bound to its own step, so it is not a binding defect. Recorded as observation O1.

## 3. Q3 — earlier findings F1-F6

| # | Earlier finding | State in `b407039` | Evidence |
| --- | --- | --- | --- |
| F1 (P0) | planar rays were free evidence at any height | **closed** | `core.py:211-212` strict 0 < z < 3 for every layout, before the planar branch. Fuzz: 0 certificates outside the interval; mutant M0 is caught (section 1.3). `check_r1_core.py:38-70` tests 8 heights × 2 layouts and the bar-top case. |
| F2 | caller hooks ran inside step/certify/map_states; `valid_for_step` read live state; `map_states` could raise | **closed (synchronous numpy/warnings hooks)** | `core.py:90, 227, 306` errstate + catch_warnings in explicit-signature bodies; `_result(packet=cur)`; `map_states` try + validation. `p5_hooks.out`: 0 numpy and 0 warnings callback calls inside, settings restored, outputs identical under `seterr(raise)`. Other process-global hooks (`gc.callbacks`: 4.4 M calls inside) still run caller code; out of scope under charter §5 (limitation L1). |
| F3 | copy not revalidated | **closed (synchronous)** | `boundary.py:31, 48` revalidate type, dtype, native order and shape of the owned copy. Asynchronous mutation is out of scope. |
| F4 | docs: track (6,), "every output", missing test names | **closed as documented** | `R1_INTERFACE_AMENDMENT_20261103.md:7-14`. Historical texts are kept on purpose. |
| F5 | late positive certificate after the deadline | **closed** | `core.py:274-275` replaces a late result with DEADLINE_UNKNOWN. Not preemptive (amendment line 18). |
| F6 | 2-packet track check vacuous; every suite at z = 1.0 | **closed in design; outputs in phase 2** | 5B: 30-packet sequences with a fresh track crossing N_STATIC/T_STATIC, a relocating pole and preregistered non-vacuity fractions. 5C: deterministic witness. 5A: z ~ U[0.05, 2.95] with random pose and yaw. 5A's own track check is still structurally vacuous (minor M4). |

## 4. Q4 — test adequacy, driver and input binding (code side)

### What each gated suite checks (read from the code)

- `check_static.py`: AST identity, catalog = snapshot = URDF, manifest vs harness, and a static name check on the
  certify-path methods and imports. `pass` is the AND of four booleans.
- `check_r1_core.py`: 7 unittest cases (F1-F5 regressions). `pass = wasSuccessful() and internal_exceptions == 0`.
  This would also pass with 0 tests, so phase 2 must read `tests == 7`.
- `attacks_p0r.py`: `pass = all(row passes)`. Variants that cannot be built are recorded as passing "skipped" rows
  (big-endian on int8).
- `scenarios_p0r.py`: fixed expected classes, camera identity for 3 variants, and the open-scene family >= 90 %.
- `map_invariants.py`: `pass = (violations == 0)`. No non-vacuity counter for I1/I2 (STATIC/CONFLICT seen); minor M3.
- `check_r1_temporal.py`: 10 checks per case × 6 cases, with explicit positive controls.
- `regress_v7.py`: input identity to v7, zero FALSE_CERT, and flags and reasons identical to v7 run 8.
- `noninterference_v2.py`:
  - **5A:** 20,000 scenes. Base, 7 cameras, track, track+camera. The JSON must be identical. FALSE_CERT 0 on base and
    track. Positive controls: VALID > 0 per layout and truthful labels > 0.
  - **5B:** 2,000 × 30 packets. Same checks, plus `static_rule_needs_track` and the three preregistered non-vacuity
    fractions (STATIC 25 %, CONFLICT 10 %, track loss 5 % of N).
  - Each packet is cast once and fed to every run (`noninterference_v2.py:164-174, 326-337`).
  - The oracle receives the actual height (`:201-202, 361`).
  - Thresholds and seeds match the preregistration (`:38-47`).

### Sensitivity probe (`p4_suite_sensitivity.py`)

This probe uses the suites' own generators and oracle. Only the base run (5A) or base+track (5B) runs on a mutant.

- 5A on M1 (no elevated-row clipping), seeds 0-999: **12 FALSE_CERT** out of 5,302 certified. In terms of scenes:
  at least 5 of 1,000. So 5A's random height can detect this class at gate size (about 100 scenes expected in 20,000,
  and about 10 in the 2,000-seed duplicate). This answers the F6 concern that the heights hid geometry defects. 5A
  cannot detect F1 itself, because z stays inside the interval; `check_r1_core` covers that.
- 5B on M4, seeds 0-99 (`p4_B_mut_M4_0_100.json`). The mutation: hits within 0.3 m beyond a fresh track's radius
  are not recorded as occupancy, which is the v6/v7 "tracker lifts a return" family, inserted at
  `evidence_map.py:111`. Result: **82 track gains** (and 115 losses) in 5,600 segments. 5B's `track_never_adds_certificate`
  would therefore fail on this defect class, so its monotonicity check is sensitive and not just non-vacuous on losses.

### Driver (`tools/run_gate.py`)

Fail-closed points:

- A rule passes only if `data[key] is True` and the exit code is 0 (`:148-150`).
- A host change means VOID_HOST_FAULT and the run stops (`:151-153, 185-187`).
- A changed input means FAIL (`:188-190, 217`).
- Every deterministic duplicate must equal its first run after `strip_timing`, which drops only `wall_s` and `ms_*`
  keys (`:115-120, 194-196`). Any extra nondeterminism fails; it cannot pass.
- Randomized duplicates: each dup per-seed digest must equal the first run's (`:197-202`). Since the dup must also
  PASS with all of its scenes completed, every seed in 0-1999 / 0-199 is compared. Per-seed results are independent
  of worker chunking: there are fresh cores and an RNG per seed, and no numpy randomness.
- The command count must equal 19, and timing must be REPORTED (`:210-218`).

What the driver does **not** do (minor M1-M2; phase-2 checks cover them):

- It checks no counts. An empty or short suite run is caught only because each suite's boolean includes its own
  completion check (5A/5B `all_*_completed`, regress input identity). `map_invariants`, `attacks` and
  `check_r1_core` have no such check.
- It does not tie the frozen tree to `b407039`. It only records `FROZEN_COMMIT` if that file exists.
- It binds the interpreter binary but not numpy, the stdlib or `motar_eval/__init__.py`, which `regress_v7.py:114`
  imports. That file is 1,040 bytes of docstring only. `meta.json` records no numpy version.

## 5. Q5 — anything else in scope a PASS would hide

Nothing in scope that would hide a defect. Coverage notes, all minor because the configs differ only by monotone
restrictions:

- No gated suite certifies with `MOTAR-72x4-RUNTIME`, the arena config a P1 adapter would use (no assurance, runtime
  health, arena). `SYNTH-72x4-RUNTIME` certifies only in attack R3b, in an open scene. My `p2` covered both
  configs: 0 FALSE_CERT in 6,400 segments.
- The randomized suites certify only sensor-anchored segments. Arbitrary world segments occur only in a few
  deterministic scenes (S11, U1, 5C). `p2` covered them.
- 5B uses only truthful tracks and truthful-noisy cameras. Hostile cameras appear only in 5A's 2-packet runs. `p3`
  covered hostile ones.

---

## Findings by severity (phase 1)

**P0-blocking (in scope):** none.

**Important:** none.

**Minor**

- M1. `run_gate.py:145-150` accepts any explicit `True`; it checks no counts. `check_r1_core` (0 tests would pass),
  `attacks_p0r` (`all([])`) and `map_invariants` (fixed 60 sequences, not checked) depend on the fixed plan.
  Phase 2 checks the counts.
- M2. Provenance: the driver records but does not check the frozen commit. Numpy, the stdlib and
  `motar_eval/__init__.py` are not hashed, and the numpy version is not recorded (`run_gate.py:76-89, 169-172`).
- M3. `map_invariants.py:264-267`: I1/I2 (STATIC/CONFLICT permanence) have no non-vacuity record. `static_seen` and
  `conflict_seen` are not output, so rule 7 cannot show that those invariants were exercised. Map behaviour only
  removes certificates, so this does not reach scope 1.
- M4. `noninterference_v2.py:40`: `STEPS_A = 2` < N_STATIC = 3. 5A's "track gains 0" and "track+camera = track"
  therefore cannot see any track effect (the F6 pattern, kept). 5B carries the non-vacuous evidence.
- M5. Coverage gaps (section 5): the MOTAR runtime arena config, non-anchored segments, and hostile tracks/cameras in
  multi-packet runs are not in the gate. All are covered by my probes with 0 violations.
- M6. Docs: `run_gate.py:1` and `noninterference_v2.py:1` cite `docs/plans/cplanner_r2prime_rules_v1.md`. That file
  does not exist; the rules are in `docs/plans/prereg_cplanner_r2prime_v1.md`.
- M7. The preregistration says the 5B fractions "were written before these dry runs". Only one commit (`b407039`,
  01:17 KST) contains both the rules and the dry-run disclosure, so this claim is not verifiable from git.

**Out-of-scope limitations (never P0-blocking under charter §5)**

- L1. Caller code still runs synchronously inside the core through other process-global hooks: `gc.callbacks`
  fired 4.4 M times inside step/certify/map_states in `p5`. `sys.setprofile` and audit hooks would behave the same
  way. This is in-process adversarial Python.
- L2. Declarations are trusted (statuses, health, pose, yaw-only LiDAR attachment). The 0-3 m catalog prior is a
  registry constant; binding it to the run's bar pool is a P1 prerequisite (charter §4).
- L3. Real time: `deadline_s` is decision expiry only. A camera's label-layer work happens in `step`, not in
  certify's timed window, so it could change a DEADLINE_UNKNOWN result only through GC or host timing effects.
- L4. Exact grazing of the 0 m and 3 m planes and of footprint boundaries (the 1e-9 / 1e-7 margins).
- O1 (observation). A Python call-level error (`TypeError`) leaves the previous packet current, still bound to its
  own step (section 2). This is not a refused step. A planner that catches it and asks for the new step gets
  `PACKET_NOT_CURRENT_FOR_STEP`.

## Phase-2 output checks I will make

1. `r2prime_copy/` manifest. Map `inputs_before.json` entries for the frozen tree file-for-file onto the review-copy
   manifest (`b407039`): no extra files (in particular no `coverage_fast.py`, `coverage_native.py`, `native/`,
   `planner/`, `m3_equivalence.py`) and no missing ones. Check `inputs_before == inputs_after`, `FROZEN_COMMIT`,
   the external and URDF hashes, and `P0R_SRC` and the `source` fields pointing at `frozen/src`.
2. Host. `host_start` and `host_end` and every record: `host_valid`, unchanged fault count and boot id, online CPU
   list. Also compare against the journal for the run window.
3. `records.json`. 19 commands in the planned order. argv as preregistered: N 20000/2000, dup 2000/200, workers 12,
   seed0 0. Exit 0. Elapsed time below the timeout. Timing ran last.
4. `gate_summary.json`. Verdict CONDITIONAL, all rules true, all determinism entries true, `duplicate_runs_pass`,
   `inputs_unchanged`.
5. Per-suite counts:
   - `check_r1_core`: tests 7, 0 failures and errors, `internal_exceptions` 0, height cases 16 / valid 6,
     baseline false certificates 0.
   - `attacks`: row count and SAFETY/CONTRACT/CONTROL totals; R4 escaped 0.
   - `scenarios`: segment count, failures 0, FALSE_CERT 0, open-scene >= 90 % per layout.
   - `map_invariants`: 60 sequences, 1,800 steps, cell checks, 0 violations.
   - `check_r1_temporal`: 6 cases, first STATIC 21, first CONFLICT 23, losses and retained controls.
   - `regress_v7`: 4,000 + 960 trials and the v7 reference SHA.
6. 5A / 5B JSON. Every check true.
   - 5A: `scenes == 20000`, segments 160,000, `camera_diff` empty, `trackcam_diff` 0, `track_gain` 0,
     `false_cert` 0, exceptions 0, VALID per layout, labelled returns. Class distribution per layout (how many
     JUSTIFIED_REFUSAL near bars, so FALSE_CERT 0 is not empty), plus `mode_counts`.
   - 5B: non-vacuity counts against 500 / 200 / 100 sequences, `track_loss_reasons`, FALSE_CERT 0 in all four runs,
     `baseline_static_cells_total == 0`, VALID per layout, labels.
   - Both: 20,000 / 2,000 digests. Recompute `digest_of_digests`.
7. Duplicates. Recompute the per-seed digest equality myself (0-1999, 0-199) and the deterministic-suite equality
   after `strip_timing`.
8. Independent re-execution, at most 4 processes. Rerun about 40 5A seeds and about 10 5B seeds from the review copy,
   spread over the range including the last seeds, and compare per-seed digests with the run's. Also rerun
   `check_static`, `check_r1_core` and `check_r1_temporal` and compare with the run's JSON.
9. Logs. Stderr is free of tracebacks and worker failures. Timing is reported only.
10. Optional. Dry-run timestamps against the commit time, for M7.

## Probe index (all under `audit/`)

| Script | Output | Purpose |
| --- | --- | --- |
| `p1_static_checks.py` | `p1_static_checks.out` | AST identity, URDF catalog, manifest, certify-path references |
| `p2_fuzz.py` | `p2_real_0_2000.json/.out`, `p2_mut_M{0,1,2}_*.json/.out` | own-oracle single-packet soundness fuzz and positive controls |
| `p3_temporal.py` | `p3_temporal_0_24.json/.out` | refusal clearing, binding, camera/track/history monotonicity in sequences |
| `p4_suite_sensitivity.py` | `p4_A_mut_M1_0_1000.json/.out`, `p4_B_mut_M4_0_100.json/.out` | 5A/5B detection of unsound mutants |
| `p5_hooks.py` | `p5_hooks.out` | F2 hooks closure; `gc.callbacks` limitation |

Reproduce, from `audit/`:

```
PY="env CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 taskset -c 0-3,8-11,14-23 /home/fair/miniconda3/envs/aerialgym/bin/python -B"
$PY p1_static_checks.py
$PY p2_fuzz.py new.json 0 2000
P0R_SRC=$PWD/mut_M1 $PY p2_fuzz.py new.json 0 600
$PY p3_temporal.py new.json 0 24
P0R_SRC=$PWD/mut_M1 $PY p4_suite_sensitivity.py A new.json 0 1000
$PY p5_hooks.py
```
