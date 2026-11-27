# OCC-P1: natural post-acquisition occlusion by obstacle density — pilot (draft)

Status: **DRAFT. No GPU run is authorised.** The machine-readable contract is
[`occlusion_pilot_v1.json`](occlusion_pilot_v1.json); where the two differ, the JSON governs. The
runner refuses while the JSON says `DRAFT`, so approval means changing that status in a committed
version.

## Questions (exploratory; no direction is assumed)

- RQ1: does the rate of geometric occlusion after first acquisition change with obstacle density?
- RQ2: does the duration of individual occlusions change with density?
- RQ3: do static and moving targets differ in occurrence, duration and reacquisition?

Two seeds cannot support a confirmatory density effect or a significance claim. The pilot decides
whether natural occlusion is measurable at 0.5/1/2/3 s and whether 5 s needs a separate stress test.

## Design

- Pursuer: Policy F (SHA-256 `f7022139…`), unchanged; actor 898-D input, tracker expiry and capture
  contract unchanged.
- Static group: A1 (E0_static, visible start). Moving group: A3 (E2_direct_l2_v1, visible start).
- Densities 70/115/160/205 bars; seeds 4201 and 4202; 128 environments × 4 quota episodes; 16 cells,
  8,192 primary episodes; initial view, distance (10 m) and timeout (600 actions) as in Experiment A.
- The seeds are reruns of existing A1/A3 cells, so G0 can compare the first four episodes per
  environment with the historical runs.
- Telemetry: `NAVRL_KROC2027_OCCLUSION_TRACE=1` (compact binary states and flags, per-reset layouts),
  `NAVRL_KROC2027_RECOVERY_EVENTS=1` with `NAVRL_KROC2027_RECOVERY_TRACE=compact` (R-P1a intervals and
  mesh probe, no per-frame JSON).

## Indicators (never merged)

`fused_loss` (a measurement loss, not an occlusion), `center_occlusion_in_view` (offline 3D centre
ray blocked while the centre is in the frustum and range), `geometric_full_in_view` (centre and all
eight silhouette-rim rays blocked; a geometric proxy), `out_of_view`, and `rendered_full_in_view`
(zero rendered target pixels while in view; only with the probe and verified zero-latency render
delivery, otherwise `NOT_RECORDED`). Offline line of sight uses the recorded boxes, which for the
bars_h3 pool are exact (upright boxes, collision equal to visual). It is not the camera image until
renderer parity passes.

## Analysis

- Intervals count after the first fused acquisition. An interval open when the episode ends is
  censored with the task outcome as its end cause; it is never a short interval.
- Per indicator and group: share of acquired episodes with an interval, rate per second of
  post-acquisition observation, observed duration quantiles, Kaplan-Meier survival at
  0.5/1/2/3 s (5 s secondary) with the caveat that capture and crash need not be independent of the
  remaining occlusion, and Aalen-Johansen cumulative incidence of each end cause (restored, capture,
  crash, timeout). Reacquisition after a restored centre occlusion: Kaplan-Meier time to the next
  fused measurement.
- Uncertainty: per-seed values and an episode-cluster bootstrap (each episode resets its own scene).
- Whole-system effect: the pilot itself. Geometry-only effect: `tools/occlusion_analysis.py sweep`
  over the pilot's recorded layouts with three scripted families (static pursuit, static crossing,
  moving pursuit at 1.5 m/s; collisions ignored), labelled `GEOMETRY_ONLY_KINEMATIC_BASELINE`.
  Straight pursuit of a static target cannot re-occlude once the sight line is clear (a later sight
  line lies on the earlier one), so any static post-acquisition occlusion in the pilot comes from
  detours or lateral motion. If the two effects cannot be separated, conclusions stay whole-system.

## Separate PASS conditions

- **G0 non-interference:** in every cell, the first four episodes of every environment equal the
  historical cell exactly (ledger fields in the JSON and every recorder field present in both). The
  first mismatch stops the run; all data are kept and the differing fields reported. The historical
  runs had telemetry off and used other commits with the same simulation paths, so a mismatch alone
  does not identify its cause.
- **Renderer parity:** probe-versus-offline centre-ray agreement ≥ 99.5 %; zero rendered pixels while
  geometrically clear in view ≤ 1 %; rendered pixels under geometric full occlusion ≤ 1 %. Without the
  probe or zero-latency delivery this is `NOT_TESTED`.

## Budget and preservation

Expected 0.4–0.6 GPU hours, cap 1.0 h (a running cell is killed at the cap and kept). Expected output
0.15–0.25 GB; the runner refuses below 2.0 GB free and without `--external-backup-confirmed`. Before
launch it requires: approved commit equal to HEAD, a clean tracked tree, this plan committed and not
draft, pinned launcher and checkpoint, one RTX 3070, imports inside the tree, a fresh output root and
every historical cell matching its RAW_MANIFEST. Existing raw results, Policy F, main, the user's WIP
and the stash are not touched.

## Not in scope

GPU execution until separately approved, PPO, R-P1b scene injection, R-P1c, the Recovery Planner,
5 s stress tests, push.
