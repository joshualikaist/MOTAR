# P0-R Gate R2: rules fixed BEFORE the R2 run (2026-10-29)

Code under test: `src/` as frozen by `results/r2/r2_input_sha256.txt` (written by `tests/run_r2.sh` before any
suite runs). Driver: `tests/run_r2.sh`. CPU only (`CUDA_VISIBLE_DEVICES=`), `PYTHONNOUSERSITE=1`, conda `aerialgym`.

Independent geometry: `snapshot/c_review/indep.py` is the v3-v7 independent oracle, byte-identical except that its
`CATALOG_JSON` path points at `snapshot/geometry_results.json` (same sha256 e10991b2...). The 3-D test ray caster
and clearance oracle in `tests/harness.py` share no code with `src/`.

Outcome classes for every certificate in a scene suite (oracle with ground truth, offline only):
- **VALID** - certified, and every catalog bar is more than R = 0.45 m from the segment.
- **FALSE_CERT** - certified, and a catalog bar is within R (plan clearance <= 0). Must be 0 everywhere.
- **OUT_OF_SCOPE** - certified, catalog bars clear, but a non-catalog or dynamic object is within R. Reported, never
  counted as safe.
- **JUSTIFIED_REFUSAL** - refused, and the oracle shows an object within R or the scene withholds the evidence the
  rule needs (incomplete packet, occlusion).
- **DEADLOCK** - refused although every object is more than R away (functional cost, reported).
- **TERMINAL_UNCERTIFIED** - the segment ends within the capture distance (0.5 m) of the target, or needs
  centre distance < R + 0.20 to it; refused by construction.

## Gate rules

1. **AST / catalog.** `coverage_b`'s four functions are AST-identical to v3 (run 4), v4, v5, v6 and v7 `certify_b.py`;
   `registry.BAR_HALF_SIZES` equals the snapshot catalog and every `bars_h3` URDF box (read-only), and all 40 URDFs
   have rpy 0.
2. **Static non-interference.** Neither `core.CertifierCore._certify` nor any function of `evidence_map.py` refers
   to the label layer or a camera; `evidence_map.py` and `coverage_b.py` import neither `labels` nor `core`.
3. **Regression (4,000 nominal + 960 real-layout trials, v7 RNG sequences).** For SYNTH-72x1 and
   SYNTH-72x1-NOASSURE: 0 FALSE_CERT, and certified flag AND reason identical to v7 run 8 in its two matching modes
   (criterion_synthetic_assurance, no_nearest_hit_assurance) on every trial.
4. **Attacks (`tests/attacks_p0r.py`).** Every SAFETY and CONTRACT case passes; `core.exceptions` is empty after
   the suite except where a case expects an entry.
5. **Camera non-interference (`tests/noninterference.py`, 20,000 random scenes, 72x1 and 72x4).** Per scene the same
   packet sequence runs with no camera and with each camera variant (truthful noisy with range sigma up to 0.7 m,
   biased, wrong extrinsic, false positive on a bar, random finite, non-finite, malformed array): the JSON of every
   `certify` result is identical to the no-camera run (0 differences). With a track: 0 segments certified with the
   track and refused without it. 0 FALSE_CERT. OUT_OF_SCOPE and DEADLOCK reported.
6. **Research scenarios (`tests/scenarios_p0r.py`).** Each fixed scene has a fixed expected class per segment; all
   match. Includes the v7 F1 geometry (pole surface 0.540 m from the target centre, camera range error -1 sigma)
   and the v6 H3 geometry.
7. **Map invariants (`tests/map_invariants.py`).** 0 violations: no FREE without free evidence; no static cell ever
   leaves STATIC/CONFLICT; an expired label never yields FREE; an incomplete packet adds no free evidence; time alone
   changes no state; the moving-target trail resolves only after K_FREE free packets.
8. **Normal function.** The regression's certified counts equal v7's (rule 3); in the scenario suite every segment
   the scene declares VALID is certified; the 72x4 open-scene family certifies at least 90 % of plan-feasible short
   segments. No gate may pass by refusing everything.
9. **Timing (`tests/timing_p0r.py`).** Reported only: p50/p95/p99/max of `step` and of `certify` for K = 1, 8, 32,
   72x1 and 72x4. No real-time PASS is claimed.

Rules 1-8 give at most CONDITIONAL. PASS needs an independent review (Opus 5.5, fresh context, read-only copy, writes
only to the new audit directory, no permission bypass) with no new P0-blocking finding. The stop rule of v7 applies:
a new structural defect of the listed kinds stops the cycle; no automatic further version.

## Pre-gate changes (from dry runs outside `results/`, made before the gate run; disclosed)

- `harness.classify`: a refused segment with a non-target object within R is JUSTIFIED_REFUSAL before it is
  TERMINAL_UNCERTIFIED (the obstacle justifies the refusal on its own).
- Scenario expectations corrected from the oracle geometry, not from the certificate: S04 path 2 (ends 0.2 m from the
  hidden bar) J; S07 path 3 is clear (V) and path 4 added (J or X: a 0.03 m pole between 5 deg beams is a declared
  non-catalog limitation); S09 path 2 J and a terminal path added; S12 a second, shorter between-beam pole added
  (X or J); S16 the 1.3 m path is clear (0.7 m > 0.65 m from the target centre, V) and a 1.4 m terminal path added.
- `attacks_p0r.py`: the big-endian variant is skipped for 1-byte dtypes (no distinct byte order); the flat-shape
  variant gives 1-D headers the shape (1, 5); S1/S2 put beam 0 through cell centres; U1 uses a segment next to the
  hidden bar.
- `map_invariants.py` I3 counts complete packets after the test-side latest HIT evidence of the cell (it first used
  the last step the state was occupied, which is not the rule).
- `core.py`: each requested segment is certified in its own error scope (`_certify_one`), so an internal error yields
  one refusal per segment, in order.
