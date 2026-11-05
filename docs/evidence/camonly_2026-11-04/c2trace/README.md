# G3: which bar does camera-only Policy F hit? (2026-11-05)

- **Preregistration:** [`docs/plans/prereg_camonly_c2trace_v1.md`](../../../plans/prereg_camonly_c2trace_v1.md)
  (committed in `90c00b4` together with the runner and the analysis tool).
- **Run:** `tools/camonly_trace_run.py` on `camonly/dev` `c4bc416`, 2026-11-05 09:16:13-09:48:43 KST.
  - 12 cells: full / no_lidar_out_fov / no_lidar × 70/115/160/205 bars, seed 4501.
  - Host faults 0 → 0.
  - Raw output: `results/camonly_c2trace_v1/` in the main checkout, untracked.
  - Log: `~/motar_tmp/gpu_queue_20261105/g3_r2.log`.
  - The first queue attempt stopped at G8 before G3 started, so G3 ran once.
- **Gate:** **NONINTERFERENCE_PASS**. Every traced cell's episode ledgers are byte-identical to the C2 / GR-0
  reference cells, so the trace recording did not change F's run.
- **Analysis:** `tools/camonly_c2trace_analyse.py`, CPU, 09:53 KST. It ran on cores 6,7,12,13 at nice 10 during the
  GR-1b GPU evaluation.
  - Output: [`c2trace_analysis.json`](c2trace_analysis.json) (SHA-256 `decd64ea…`).
  - Reading **BLIND_SIDE_SUPPORTED**.
- **Independent review r1** (Opus 5.5, read-only): **PASS_WITH_LIMITS**, no BLOCKER or MAJOR finding.
  - Every number reproduces exactly from the raw traces.
  - Copy: [`REVIEW_r1.md`](REVIEW_r1.md). The reviewer's scripts and outputs are in [`review_r1/`](review_r1/).

## Result (from `c2trace_analysis.json`)

The columns:
- **out_last:** the contact bar, the bar nearest the vehicle at the last recorded frame (about 0.1 s before contact),
  is outside the camera FOV (|bearing| > 43.5°).
- **never_in_1s:** that bar was outside the FOV in all of the last 11 frames.
- **travel_out:** moving frames whose velocity bearing is outside the FOV.

| Condition | Crash episodes | Valid (≤ 0.6 m) | out_last | never_in_1s | travel_out | Contact bar on the left |
| --- | --- | --- | --- | --- | --- | --- |
| A, camera + LiDAR (full) | 475 | 97.3 % | 48.8 % | 33.7 % | 52.1 % | 63.6 % |
| B_out, out-of-FOV LiDAR removed | 5138 | 99.8 % | 72.1 % | 56.5 % | 70.2 % | 93.3 % |
| C, camera only (no_lidar) | 5109 | 99.7 % | 72.9 % | 56.7 % | 70.7 % | 93.4 % |

Contrasts in out_last (percentile bootstrap over crash episodes within condition, 10,000 resamples, seed 20261105):
- **C − A:** +24.0 pp, 95 % CI [19.3, 28.7]. With out_last(C) ≥ 60 %, this gives the reading.
- **B_out − A:** +23.2 pp, CI [18.6, 27.9] (reported, not gated).

## Per-density counts (prereg "reported, not gated"; review F1)

The analysis JSON omits these. They come from the reviewer's independent recomputation
(`review_r1/recompute_v2.py` → `review_r1/recompute_out_v2.json`), whose pooled values equal the JSON exactly.

| Density | A crash / out_last | B_out crash / out_last | C crash / out_last | Δ C−A |
| --- | --- | --- | --- | --- |
| 70 | 38 / 42.1 % | 775 / 72.6 % | 672 / 74.7 % | 32.6 pp |
| 115 | 66 / 56.1 % | 974 / 72.3 % | 1159 / 74.1 % | 18.1 pp |
| 160 | 145 / 44.1 % | 1405 / 72.2 % | 1490 / 72.3 % | 28.2 pp |
| 205 | 226 / 50.9 % | 1984 / 71.6 % | 1788 / 71.8 % | 20.9 pp |

## How to read it

- **Consistent with a blind-side mechanism.** Camera-only F's crashes hit bars outside the camera's field of view
  more often than full-sensor F's crashes, and almost all of them on the left.
  - F flies diagonally forward-left (`../mechanism/`).
  - Removing only the out-of-FOV LiDAR (B_out) reproduces C almost exactly.
  - This is "consistent with", not a causal test.
- **never_in_1s is the more specific evidence** (review F12). out_last tracks travel_out in each condition (A 48.8
  against 52.1 %, C 72.9 against 70.7 %). never_in_1s, the bar unseen for the whole last second, rises from 33.7 to
  56.7 %.
- **Absolute levels are not the camera's literal view** (review F4). Bearings are measured from the vehicle centre,
  not the camera eye 0.1 m forward. From the eye, with body yaw (`review_r1/supplement_out.json`):
  - out_last is 62.7 % for A, 84.2 % for B_out and 84.0 % for C;
  - C − A is +21.3 pp, CI [16.7, 25.8].
  - The reading is unchanged.
- **Robustness (review, `review_r1/recompute_out_v2.json`):**
  - quota-only episodes: +26.0 pp, CI [20.7, 31.3];
  - a two-stage cell bootstrap: CI [17.7, 31.3];
  - Δ C−A is between 18.1 and 32.6 pp in every density.

## Limits

- **Coverage:** one seed (4501), the E2 target, Frozen Policy F zero-shot. B_out and C are out of F's training
  distribution (C2's OOD note).
- **Contact bar:** the nearest bar about 0.1 s before contact, by surface distance. 13 / 9 / 13 crashes fail the
  0.6 m check and stay in the counts (review F6).
- **Analysis record:** the JSON records neither the tool's SHA nor the run's reading (review F11). The tool is
  unchanged since `90c00b4`.
- **Open review notes:** F2, F3, F5, F7, F8 and F9 are in `REVIEW_r1.md`. None changes the reading.
