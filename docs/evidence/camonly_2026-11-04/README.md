# Camera-only study evidence (`camera-only` family)

The plan is [`docs/plans/camonly_study_plan_v1.md`](../../plans/camonly_study_plan_v1.md), with its "After review r1"
section, under decision D-2026-11-04-3.

## `c0/`: the opt-in LiDAR-ablation switch (CPU gate)

| File | What |
| --- | --- |
| `gate_01_c0_gate.json` | C0 gate run on `4d03f35`: C0_CONDITIONAL_PASS (10 tests, 5 mutants caught, PUBLIC_CPU 0 new failures against the base, pinned files unchanged, host faults 0) |
| `REVIEW_r1.md` | Independent review r1 (Claude Opus 5.5, fresh context): **FAIL** on B1, a vacuous association-leak check; medium M1-M3, low L1-L9 |
| `gate_02_c0_gate.json` | C0 gate run on the repair commit `842fcf5`: C0_CONDITIONAL_PASS (14 tests, 7 mutants caught, PUBLIC_CPU 3,248 = 3,234 + 14 with 0 new failures, pinned files unchanged, host faults 0) |
| `REVIEW_r2.md` | Independent review r2 (Claude Opus 5.5, fresh context): **PASS**. C0 is PASS. Open: M-r2-1 (obs_only governor not pinned by the tests) and M-r2-2 (a wrong disclosure in the plan), to fix before the C2 preregistration; low L-r2-1 to L-r2-5 |

Raw logs are in `~/motar_tmp/camonly_c0_20261104/`: `gate_01/`, the base suite log `public_cpu_base_02.log`, and
`review/audit/` (the reviewer's probes).

## `c1c2/`: GPU stages C1 (identity) and C2 (zero-shot conditions)

Preregistration: [`docs/plans/prereg_camonly_c1c2_v1.md`](../../plans/prereg_camonly_c1c2_v1.md).

| File | What |
| --- | --- |
| `c1_report_1908fd8.json` | C1 on `1908fd8`: C1_IDENTITY_PASS (`full` equals GR-0's cell exactly; `no_lidar` differs). Superseded as C2's gate by the C1 rerun on amendment 1's commit |
| `REVIEW_design_r1.md` | Pre-measurement design review (Claude Opus 5.5): AMEND_FIRST, no blocking finding; answered by amendment 1 |
| `smoke_02.json` | The smoke test after amendment 1 (`smoke_c1c2.py`, current version): 14 of 14 checks, with distinct per-condition effects, contrast means checked independently, and five new known-bad cases |
| `smoke_01.json` (made by the first version of `smoke_c1c2.py`, in `1908fd8`) | CPU smoke test of `tools/camonly_run.py` and `tools/camonly_c2_analyse.py` before launch (simulator stubbed, real GR-0 cells copied): 9 of 9 checks pass, covering C1 pass, switch-without-effect fail, missing-receipt fail, C2 needing C1, wrong-condition stop, the complete and ordered C2, the analysis known-good case and two binding failures |


## `sensor/`: camera against exact range at Policy F's own poses (CPU, exploratory)

The plan's "Sensor-level check" (section "K5 revised"): resolution is not changed in C2, so this check measures the
camera input instead. Not preregistered, not a gate, and not independently reviewed. No policy is run and no GPU is
used. The plan also asks for the counts by bar position between rays; this run did not compute that split
(NOT_RECORDED).

| File | What |
| --- | --- |
| `sensor_check.py` | The check. It renders MOTAR's 40 x 24 obstacle camera (kernel copied byte for byte from `navrl_detector.py`, upsampled to 160 x 90 as `render_raw_rgbd`) and the 72 x 4 LiDAR (`cplanner/motar/replay.py` at `bf1ebc7`, MOTAR's LiDAR kernel) on the CPU, at 1200 recorded frames of the B1-R `A/F_SHADOW` traces (70/115/160/205 bars, seed 4301, 300 frames each). Each of the 17 in-FOV bins' fused camera value (`navrl_perception.py:1910-1921`) is compared with an exact horizontal ray against the bar boxes (tolerance 0.3 m) |
| `obstacle_kernel_extracted.py` | The obstacle-camera kernel extracted from `navrl_detector.py`; its SHA-256 is recorded in the JSON |
| `sensor_check_01.json` | The result, with the SHA-256 of every source file it loaded (they match `camonly/dev` at `5522be0` and `cplanner/dev` at `5944d43`) |
| `sensor_check_01.out` | Its stdout (Warp's CPU-only banners and the summary) |

Validation of the exact-range caster: the LiDAR's horizontal row agrees on 41837 of 41837 bins (largest error 0.0002 m).

Per bin, by the exact range on the bin's bearing (pooled over the four densities; `pooled_by_truth_band`):

| Exact range | Bins | Phantom | Miss | Beyond the camera | Phantom, sampled bearing | Miss, sampled bearing |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0-2 m | 2188 | 0.2 % | 9.1 % | 0.0 % | 2.4 % | 5.2 % |
| 2-4 m | 2266 | 2.1 % | 23.1 % | 0.0 % | 6.6 % | 11.3 % |
| 4-6 m | 1970 | 7.3 % | 36.4 % | 0.0 % | 11.3 % | 15.5 % |
| 6-8 m | 1554 | 10.2 % | 47.9 % | 0.0 % | 17.2 % | 18.4 % |
| 8-10 m | 1346 | 17.8 % | 42.1 % | 0.0 % | 22.6 % | 11.1 % |
| 10-12 m | 1103 | 28.6 % | 0.0 % | 68.4 % | 12.2 % | 6.2 % |
| 12 or more m | 9973 | 21.8 % | 0.0 % | 0.0 % | 8.2 % | 4.6 % |

- *Phantom*: the camera reports something nearer than any bar on that bearing. *Miss*: a bar within the camera's
  10 m is reported farther or not at all. *Beyond the camera*: the bar is at 10-12 m, inside the LiDAR's range only.
- *Sampled bearing*: the same classification against the bearing the camera column actually samples. The linear
  angle-to-column map in perception differs from the pinhole render by up to 3.41 degrees (`camera_column_bearing`;
  review r2 estimated about 3.4 degrees),
  so the bin-bearing columns include that offset and the sampled-bearing columns isolate the render itself
  (40 x 24 rays and bilinear upsampling).
- Scope: one seed per density, F_SHADOW poses only, horizontal truth rays only. It describes the camera value F
  receives in place of the in-FOV LiDAR; it does not measure what the policy does with it.
