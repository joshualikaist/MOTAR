# OCC-G0 and OCC-P1: natural post-acquisition occlusion by obstacle density (approved contract, v2)

Status: **approved by the user on 2026-10-17** for G0 and, if G0 passes, the seven-seed pilot.
The machine-readable contract is [`occlusion_pilot_v2.json`](occlusion_pilot_v2.json); where the two
differ, the JSON governs. It supersedes the two-seed draft `occlusion_pilot_v1.*`, which stays as the
record of the draft. Written before any G0 or pilot cell ran.

What changed from the draft: the 30 GPU-hour budget no longer shapes the design; G0 is an own-commit
ON/OFF comparison rather than a comparison with historical cells only; the pilot uses seeds
4201–4207 (56 cells, 28,672 primary episodes); the time limits are wall-clock safety caps with
overrun logging, not budget caps; the run stops below 8 GB free disk; the external backup is recorded
as not completed but is not a blocking condition for this run.

## Questions (exploratory; no direction assumed)

- RQ1: does the rate of geometric occlusion after first acquisition change with obstacle density?
- RQ2: does the duration of individual occlusions change with density?
- RQ3: do static and moving targets differ in occurrence, duration and reacquisition?

Seven seeds do not make this confirmatory. Hypotheses and criteria for a confirmatory OCC-P2 with
fresh seeds are fixed only after this pilot is analysed, in its own preregistration.

## Design

- Pursuer: Policy F (SHA-256 `f7022139…`), unchanged; actor 898-D input, tracker expiry (age > 5 s)
  and the capture contract unchanged. No PPO.
- Static group: A1 (E0_static, visible start). Moving group: A3 (E2_direct_l2_v1, visible start;
  `direct_l2_v1` backend, full_v2 packets, stop on a motion violation).
- Densities 70/115/160/205 bars; 128 environments × 4 quota episodes; initial view and distance
  (10 m) and the 600-action timeout as in Experiment A.
- Telemetry: `NAVRL_KROC2027_OCCLUSION_TRACE=1` (compact per-frame states and flags, per-reset
  layouts), `NAVRL_KROC2027_RECOVERY_EVENTS=1` with `NAVRL_KROC2027_RECOVERY_TRACE=compact` (R-P1a
  intervals and the Warp mesh probe; no per-frame JSON).

### OCC-G0 (seed 4201; 8 pairs, 16 cells)

Each (group, density) runs twice on the same commit. OFF: recorder, trajectory digest and actor
observation dump, no telemetry. ON: the same plus the telemetry variables. Three separate verdicts:

1. **Non-interference** (all 8 pairs): `result.json` equal except the run nonce; trajectory digest
   SHA-256 and step count equal with steps > 0; ledger `rows_sha256` equal; every actor-observation
   row sampled by both runs bitwise equal (`observation_gate` from the unmodified replication tool);
   recorder episode records equal field by field; and ON equals the historical A1/A3 cell on the
   first four episodes per environment. Identical trajectories, actions, observations and outcomes
   from the same seed also show the RNG stream was not consumed. A failure here blocks OCC-P1; the
   differing fields are reported, not explained away.
2. **Renderer parity** (ON cells): Warp mesh probe versus offline centre ray agree on ≥ 99.5 % of
   probe frames; zero rendered target pixels while geometrically clear and in view ≤ 1 %; rendered
   pixels under the geometric full-occlusion proxy ≤ 1 %. Needs probe frames and verified
   zero-latency render delivery; otherwise `NOT_TESTED` with the reason, and `rendered_full_in_view`
   stays `NOT_RECORDED` in every later analysis.
3. **Scene confirmation**: frames that are visible (in view, clear, rendered pixels > 0), partial
   (centre blocked, rim not all blocked or pixels > 0) and full (geometric proxy; rendered zero pixels
   when the probe ran) each occur at least once in the ON cells, with episode keys listed.

Camera capture time is not recorded. When render alignment is not verified, every camera-versus-
geometry comparison is `UNKNOWN`; a delayed frame is never treated as the current geometry.

### OCC-P1 (seeds 4201–4207; 56 cells)

Runs only after a G0 report at the same commit with non-interference PASS. Same cells as G0's ON
configuration for every seed.

## Indicators (never merged)

`fused_loss` (a measurement loss, not an occlusion), `center_occlusion_in_view` (offline 3D centre ray
blocked while the centre is in the frustum and range), `geometric_full_in_view` (centre and all eight
rim rays blocked; a geometric proxy), `out_of_view`, `rendered_full_in_view` (zero rendered pixels in
view; only with the probe and verified alignment). The recorded boxes are exact for the bars_h3 pool
(upright, unrotated, collision equal to visual).

## Analysis

Intervals count after the first fused acquisition; an interval open at the end is censored with the
task outcome as its cause and is never a short interval. Per indicator, group and density: share of
acquired episodes with an interval; rate per second of post-acquisition observation; observed
duration quantiles; Kaplan–Meier at 0.5/1/2/3 s (5 s secondary, with the risk-set size), stated with
the caveat that capture and crash need not be independent of the remaining occlusion; Aalen–Johansen
cumulative incidence by end cause; reacquisition time after a restored centre occlusion; exposure time
per density. Uncertainty: per-seed values and an episode-cluster bootstrap. The geometry-only sweep
(three scripted kinematic families over the recorded layouts, collisions ignored) is reported apart
from the whole-system pilot; where the two cannot be separated, conclusions stay whole-system. If
early episode ends leave too little exposure to observe 5 s occlusions, that is reported as a limit
of the condition, not answered by more cells.

## Safety and preservation

Wall-clock safety caps: G0 2.0 h (expected 0.35–0.6), P1 5.0 h (expected 1.3–1.8); at the cap the
running cell is killed and kept. A cell slower than 2 × 180 s is logged with its ratio and state. The
runner refuses to start and stops before the next cell below 8 GB free. Before launch: approved commit
equals HEAD, clean tracked tree, this plan committed, pinned launcher and checkpoint, one RTX 3070,
imports inside the tree, fresh output roots, every historical cell matching its RAW_MANIFEST. Policy F
and the 583 raw files are re-hashed after each run. New run ids; nothing existing is overwritten; the
user's WIP and stash are not touched. GPU seconds and output bytes are recorded per cell.

## Not in scope

OCC-P2, the Recovery Planner, PPO, R-P1b scene injection, push.
