# Evaluation contract design: collision-free capture, crash, timeout, safe abort (EQ-1C C6)

Status: **design, not implemented, not measured.** Historical outcome rules (capture / crash /
timeout, capture winning over a same-step contact) stay unchanged for every historical result. This
contract is for future runs and new lineages only; it never relabels OCC-P1/P2.

## Current rules (as executed, source `5ff285a`)

- Capture: the swept segment of the pursuer-target relative position over the 0.1 s step comes
  within `success_radius = 0.5 m`. **Capture wins over a contact in the same step**
  (`crashed_out & ~captured`).
- Crash: contact, below/above the altitude band, out of bounds (`[0, 40] ± 1.0 m`), or an invalid
  target. The cause is aggregated in the legacy `result.json` only, not per episode.
- Timeout: 600 actions.
- Safe abort: does not exist (no abort action or state); `safe_abort` is NOT_RECORDED in every OCC-P2
  recovery summary.
- Population: the episode ledger's quota (first 4 completed episodes per environment). The legacy
  window differs in membership in 112/112 cells and in outcome counts in 96/112 (EQ-1C C5).

## Proposed outcome partition (per quota episode, mutually exclusive)

| Outcome | Definition | Evidence required |
| --- | --- | --- |
| `capture_clean` | capture event and no contact at any physics substep of the episode up to and including the capture step | inner-step contact flags (E-L0 / EQ-1D telemetry) |
| `capture_with_contact` | capture event and a contact in the same step or earlier without termination | as above; today such episodes are plain captures |
| `crash:<cause>` | first terminal non-capture event, with the cause (`contact`, `below`, `above`, `oob`, `target_invalid`) stored per episode | per-episode cause field in the ledger |
| `safe_abort` | the controller declares abort (an explicit channel), then the vehicle holds horizontal speed < 0.1 m/s with conservative body clearance ≥ 0.45 m and no contact for ≥ 1.0 s before the timeout | an abort channel (absent in Policy F) and 10 Hz states |
| `timeout_at_rest` | timeout with the last 1 s at rest and clearance ≥ 0.45 m (a passive safe end; the only abort-like label measurable for Policy F) | 10 Hz states and layouts (available now) |
| `timeout_moving` | any other timeout | — |

Precedence inside one step: contact before capture for the `clean` label (the historical label is
kept in its own column). Out-of-bounds counts as a crash even when it coincides with capture.

## Safety measures reported alongside (not outcomes)

- Stopping margin `D − (v·t_r + v²/2a)` per frame along the velocity (EQ-1B B2b, conservative body),
  with the brake value declared. Reported for normal flight and the last second before a crash.
- Minimum conservative body clearance per episode; tilt and speed exceedance counts (B2).
- Contact chronology (substep of first contact, speed at contact) from E-L0 telemetry.

## Population and joins

- One population: quota episodes, joined by `(seed, cell, env, episode)`; batch tails excluded and
  counted. Every metric (outcomes, occlusion, governor samples, crash causes) uses the same join, and
  a run fails closed when a metric's key set differs from the quota set.
- Legacy windows are kept only for comparison with historical aggregates and are labelled as such.

## What can be measured now versus later

| Item | From existing OCC-P2 records | Needs new telemetry |
| --- | --- | --- |
| `timeout_at_rest` / `timeout_moving` | yes (5 quota timeouts) | — |
| Stopping margin, clearance, tilt/speed | yes (B2) | — |
| `capture_clean` versus `capture_with_contact` | no | inner-step contact flags |
| Per-episode crash cause | no | ledger field |
| `safe_abort` | no | an abort channel (a new controller lineage) |
