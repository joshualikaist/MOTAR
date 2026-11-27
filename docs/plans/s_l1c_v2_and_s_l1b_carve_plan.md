# Next-step plans: S-L1C v2 (off-row recall) and the S-L1B carve retention — PLAN ONLY

Status: **planning record, not a preregistration.** No production code is written or changed for
either item. Each needs its own preregistration, independent oracle, CPU gates, GPU canary and
review before any result is claimed. Written 2026-10-18, after SL1D-C02.

## 1. S-L1C v2: genuine off-row sphere returns refused below 2.295 m

### Problem

The `s_l1c_v1` shape test rejects a matched LiDAR return when the vertically adjacent row has the
same horizontal range within 0.10 m. A 0.20 m proxy sphere whose centre lies between two rows,
which are 10° apart, reaches both rows up to 0.20/sin 5° = 2.295 m. Its two horizontal ranges then
differ by only 2–3 cm, so a genuine return is refused. These results come from the CPU oracle, at
bearing 0° with elevations −8.5° to +17.5° in 0.5° steps (S-L1C review addendum):

| Centre range | Genuine-hit elevations | Refused by v1 |
| --- | --- | --- |
| 1.16 m | 53 | 47 |
| 1.50 m | 53 | 23 |
| 2.00 m | 53 | 8 |
| 2.28 m | 53 | 2 |
| 2.30 m | 50 | 0 |

In evaluation-only closed-loop data, about 2–3 % of steps fall in this band. The bar-only false
association that v1 removed (21/21 → 0/21) must not return.

### Candidate geometry-aware rules

All candidates use only the LiDAR scan, the track's own prediction and declared constants. None
uses simulator GT or semantic ids.

| ID | Rule | Expected strength | Risk |
| --- | --- | --- | --- |
| V2-A (hypothesis test) | For each eligible row, compare the measured range with two predictions: the predicted proxy sphere (analytic ray–sphere range along that row's beam from the predicted centre) and a vertical surface (equal horizontal range). Accept when the sphere residual is smaller by a declared margin. Rows the sphere cannot reach keep the v1 test. | Recovers the off-row band in simulation, where LiDAR is exact to 1e−4 m | The hypotheses differ by only centimetres at 1.2–2.3 m. Under realistic LiDAR noise this is not separable, so a declared noise model must bound the margin. |
| V2-B (azimuth support) | Add a horizontal-extent test. A 0.4–0.8 m bar returns in neighbouring azimuth columns at the same horizontal range, where the 0.20 m sphere cannot beyond its angular radius. Apply only where the sphere's angular radius is below the 5° column spacing. | Independent of vertical geometry | Inconclusive at short range, where the sphere also spans columns (< 4.6 m). |
| V2-C (camera confirmation) | In the band, do not reject. Mark the association range-only/unconfirmed and require a camera re-confirmation within a declared window before it can refresh the fused expiry. | Keeps recall; identity stays camera-borne | Changes the fused-recency contract. That needs an explicit new contract, with the 5 s baseline preserved. |
| V2-0 (baseline) | v1 unchanged, used as the matched comparison | — | — |

### Evaluation plan (matched comparison against v1)

- **CPU oracle sweep.** Range 1.16–3.0 m × elevation −8.5…+17.5° × bearing {0, 20, 45, 90}°, with
  centre offsets of 0 and ±0.15 m. Measure genuine-hit recall. Over the same sweep, measure
  false association for 105+ tall-bar negatives (1.5–11.5 m) and the 7 EQ-1D D2-N2 negatives.
  Mandatory cases: 1.16, 1.5, 2.0, 2.28 and 2.3 m at off-row elevations ±5°.
- **GPU canary.** Run sensor fixtures with off-row targets at 1.2–2.3 m (elevation offsets from
  flight altitude) and bar-only negatives at the same ranges, plus a closed-loop regression:
  - off = EL0-R1, s_l1a/b/c equal to their qualified runs, and E-L0 P1 = P2;
  - closed-loop update-source counts reported against v1 (no performance claim).
- **Gates (proposed).**
  - Bar-only false association no worse than v1: 0/21 D2-N2, and on the sweep ≤ v1's 5/105, all
    inside the declared > 11.35 m residual band.
  - Genuine recall in the 1.152–2.295 m off-row band strictly improves on v1, with no recall loss
    elsewhere.
  - Range-only covariance (C-COV), identity clock (C-PROV) and the GT firewall are unchanged.
  - If one metric improves while another core check fails, the result is NO-GO.

## 2. S-L1B carve: 84/102 target returns kept as obstacles

### Problem

The `s_l1b_v1` carve window starts at s_L = anchor_L − (0.20 − 0.15). anchor_L is the minimum
body range of the camera candidate pixels. No pixel ray passes exactly through the sphere's
nearest point, so the anchor is biased high by 0.41–24.3 mm (median 1.89 mm; S-L1B review). With
zero tolerance below s_L, 78 of 96 supported target-centre returns fall below s_L and are kept.
S-L1A carved more of them only because of its frame offset of about 0.05 m, and that same offset
let S-L1A erase bar returns in that band.

### Candidate corrections

All corrections are derived from sensor geometry, not tuned.

| ID | Rule | Safety argument | Risk |
| --- | --- | --- | --- |
| CB-1 (analytic sampling bound) | Lower the window by ε(anchor_L, pixel pitch, r_camera): the largest possible overestimate of the nearest-surface distance when the nearest point lies between pixel rays. | ε is a geometric bound, not a fitted value, so a foreground obstacle closer than s_L − ε is still never carved. | Bars within ε (≤ ~2.5 cm) in front of the proxy surface become carvable, and must be shown absent or reported. |
| CB-2 (per-beam sphere range) | Estimate the target centre from the camera (the anchor direction plus r_camera). Carve a LiDAR beam only when its return is within a declared range tolerance of the analytic proxy-sphere range along that beam. | Each beam is checked against its own expected range instead of one global lower bound, so foreground bars at any other range are kept. | Accuracy of the centre estimate; the A7 camera centre-range bias would need to be fixed or bounded first. |
| CB-0 (baseline) | s_l1b_v1 unchanged | — | — |

### Validation plan

- **CPU.** Run the S14, A_FOREGROUND and B_TIGHT fixtures. Add new tight fixtures with bars
  0–0.1 m in front of the proxy surface at 2–8 m and bearings 0/±20°, and beam-centred and
  off-centre targets. Measure target-return retention and foreground bar carving against an
  independent oracle.
- **GPU.** Run a sensor canary on the 43 S-L1B scenes plus the new tight scenes, and a closed-loop
  regression: lower lineages bitwise equal to their qualified runs, E-L0 P1 = P2, and target
  retention and ON behaviour reported without claims.
- **Gates (proposed).**
  - No foreground bar return carved and no bar false clear; unchanged from B-CARVE.
  - No unsupported carve; carve masks equal the oracle.
  - Target-return retention strictly below v1's 84/102, with a matched comparison on identical
    inputs.
  - Any carved bar return is NO-GO, regardless of retention gains.

## Not authorised by this plan

- implementation of V2-A/B/C or CB-1/2;
- P-L1, G-L1, PPO and the Recovery Planner;
- H-R confirmation experiments.
