# Preregistration: camera-only G3 — which bar does camera-only Policy F hit? (v1, 2026-11-05)

Branch `camonly/dev`. Runner `tools/camonly_trace_run.py`; analysis `tools/camonly_c2trace_analyse.py`. Both are fixed
in this commit, before the run. The user approved the GPU session on 2026-11-05 ("이거 하고 gpu 세션 바로 시작하자");
its queue (Beads `motar-g9a`) names this run as G3.

## 1. Question and claim

- **Background:** the CPU mechanism analysis (`docs/evidence/camonly_2026-11-04/mechanism/`) found two things.
  - Policy F's full-LiDAR flight is diagonal forward-left: travel is outside the camera FOV in 51.8 % of moving
    frames.
  - Without the out-of-FOV LiDAR, F stops reversing laterally (2.07 % → 0.15 % of steps).
  - It left the contact bar's bearing in C2's crashes NOT_MEASURED.
- **Claim under test (blind side):** camera-only crashes (C) are contacts with bars outside the camera FOV, more than
  F's crashes with the full LiDAR (A) on the same cells.
- **What would make it false:** C's contact bars are no more often outside the FOV than A's, for example frontal
  contacts from camera misses.

## 2. Design

**Cells:**
- conditions `full` (A), `no_lidar_out_fov` (B_out) and `no_lidar` (C);
- 70/115/160/205 bars, seed 4501, quota 16;
- E2, A3 and Policy F, as in C2;
- 12 cells, order density then condition.

**Addition to C2's cell path:** `NAVRL_KROC2027_OCCLUSION_TRACE=1`, the recorder's compact binary telemetry:
- both UAVs' states per consumed frame, and every reset's obstacle layout;
- it reads detached tensors only (`kroc_eval_adapter.py`).

**Noninterference gate (runner):** each traced cell's `episodes.jsonl` must be byte-identical to its untraced
reference, and its outcome counts equal.

| Condition | Untraced reference |
| --- | --- |
| A | GR-0 S00 cell |
| B_out, C | C2 cell |

- **Reading:** NONINTERFERENCE_PASS, or TRACE_INTERFERES. Under TRACE_INTERFERES the analysis still runs, but it
  describes these traced runs only.
- **Prior status:** the B1-R traces recorded `gpu_trajectory_noninterference: NOT_TESTED`; this gate is the test.

## 3. Analysis (fixed here; `tools/camonly_c2trace_analyse.py`)

- **Geometry:** the yaw-only vehicle frame (x = camera axis). The FOV is |bearing| ≤ 43.5°. Bars are axis-aligned
  boxes spanning the vehicle's height, as in `mechanism/f_trace_geometry.py`.
- **Crash episodes:** `episodes.jsonl` outcome `crash`, every traced episode.
- **Contact bar:** the bar nearest (surface distance) at the last recorded frame, about 0.1 s before contact.

| Measure | Definition |
| --- | --- |
| out_last | contact bar outside the FOV at the last frame |
| never_in_1s | contact bar outside the FOV in each of the last 11 frames |
| left | contact bar on the left |
| travel_out | share of moving frames whose velocity bearing is outside the FOV |

- **Validity gate:** in each condition, at least 90 % of crash episodes have a last-frame surface distance ≤ 0.6 m;
  otherwise MEASUREMENT_INVALID.
- **Contrast:** Δ = out_last(C) − out_last(A), in pp. 95 % percentile bootstrap over crash episodes, resampled within
  each condition, 10,000 resamples, seed 20261105.
- **Reading** (first match wins):

  | Reading | Rule |
  | --- | --- |
  | MEASUREMENT_INVALID | validity gate fails |
  | **BLIND_SIDE_SUPPORTED** | Δ ≥ 15 pp, CI lower > 0, and out_last(C) ≥ 60 % |
  | **BLIND_SIDE_NOT_SUPPORTED** | CI upper < 15 pp, or out_last(C) < 50 % |
  | INCONCLUSIVE | otherwise |

- **Why the thresholds:**
  - If contact bearings were uniform, about 76 % would fall outside the FOV, because the camera covers 17 of the 72
    bins.
  - F's own full-LiDAR crashes, on another seed, were outside it in 51.3 %.
  - A rise of 15 pp moves from F's level toward the uniform level. 60 % is a clear majority outside the camera's view.
- **Reported, not gated:**
  - B_out − A;
  - never_in_1s, left and travel_out per condition;
  - per-density counts.

## 4. Limits fixed in advance

- **Scope:** one seed (4501), four densities, zero-shot Policy F, K1 and K2.
- **Contact bar:** the nearest bar 0.1 s before contact is a proxy. The validity gate bounds it.
- **Dependence:** episodes from one cell share a layout seed. The bootstrap treats crash episodes as independent, so
  the CI is narrower than a cluster CI.
- **Result labels:** they stay as computed until an independent results review.

## 5. Cost

12 cells at about 2–3 min each, so about 30–40 min of GPU. Budget cap 1.5 h, outer `timeout -k` 2 h. Traces take
about 30 MB per cell.
