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
| `c1_report_5522be0.json` | C1 rerun on amendment 1's commit `5522be0` (`c1_r2`): C1_IDENTITY_PASS, the gate for C2 |
| `c2_run_report.json` | C2 run report: COMPLETE, 60 of 60 cells OK, host faults 0 at start, after every cell and at the end (19:57:26 to 22:30:58) |
| `c2_analysis.json` | Output of `tools/camonly_c2_analyse.py` (SHA-256 `f9d709ff…`, the bytes at `5522be0`) on GR-0 `gr0_main`, `c1_r2` and `c2`: reading REPORTED, all 18 bindings true |
| `SHA256_MANIFEST_c1r2_c2.json` | SHA-256 of the 498 raw files of `results/camonly_c1c2_v1/{c1_r2,c2}` (main checkout, untracked) |
| `REVIEW_results_r1.md` | Independent results review r1 (Claude Opus 5.5, fresh context, read-only): **PASS_WITH_LIMITS**; every binding and number re-derived from raw; one MAJOR (M-1, a protocol deviation) and six MINOR. Its scratch scripts are in `~/motar_tmp/camonly_c1c2_20261104/review_results/scratch/`. Its header gives the end time as about 23:10; the review finished at about 22:44 |
| `sensitivity_author.json` | The author's own check of M-1 and m-2: each label with block (115, 4501) dropped, and how many of the 12 leave-one-block-out labels are unchanged |

### C2: reading REPORTED (independent review PASS_WITH_LIMITS)

Zero-shot, Frozen Policy F, simulation, E2 moving target, A3, 70-205 bars, seeds 4501-4503, quota 16; 12 blocks.
No adoption decision is taken. Conditions without out-of-FOV LiDAR are out of distribution (OOD) for F.

| Condition | Capture % | Crash % | Governor intervention rate | Mean clearance (m) | Mean executed speed (m/s) | Bar contacts / 100 | Floor / 100 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `full` (GR-0 S00 cells) | 95.54 | 4.46 | 0.28 | 7.99 | 2.95 | 4.38 | 0.10 |
| `no_lidar_in_fov` | 96.23 | 3.76 | 0.28 | 8.03 | 2.95 | 3.64 | 0.13 |
| `no_lidar_in_fov_obs_only` | 95.70 | 4.30 | 0.28 | 7.98 | 2.94 | 4.14 | 0.12 |
| `no_lidar_out_fov` (OOD) | 56.28 | 43.71 | 0.85 | 1.66 | 2.17 | 44.07 | 0.06 |
| `no_lidar` (OOD) | 55.59 | 44.40 | 0.85 | 1.67 | 2.17 | 44.76 | 0.13 |
| `no_lidar_obs_only` (OOD) | 60.64 | 39.36 | 0.41 | 6.91 | 2.81 | 39.84 | 0.07 |

Pooled capture and crash are over the quota; governor telemetry and crash causes are over the aggregate window of
`result.json` (reported, not gated).

| Contrast | What it changes | Capture, pp [95 % bootstrap CI] | Crash, pp [95 % bootstrap CI] | Labels (capture / crash) | Block (115, 4501) dropped | Leave-one-out unchanged (capture / crash) |
| --- | --- | --- | --- | --- | --- | --- |
| `total` (OOD) | C − A: all LiDAR removed | -39.95 [-45.68, -34.13] | +39.95 [+33.95, +45.52] | LOWER / HIGHER | LOWER / HIGHER | 12 / 12 of 12 |
| `out_fov` (OOD) | B_out − A: out-of-FOV LiDAR removed | -39.26 [-44.59, -33.74] | +39.25 [+33.65, +44.71] | LOWER / HIGHER | LOWER / HIGHER | 12 / 12 of 12 |
| `in_fov` | B_in − A: in-FOV LiDAR removed | +0.69 [+0.35, +1.06] | -0.69 [-1.07, -0.33] | SMALL_HIGHER / SMALL_LOWER | SMALL_HIGHER / SMALL_LOWER | 12 / 12 of 12 |
| `interaction` (OOD) | C − B_in − B_out + A | -1.38 [-2.51, -0.27] | +1.39 [+0.28, +2.52] | SMALL_LOWER / SMALL_HIGHER | UNRESOLVED / UNRESOLVED | 5 / 4 of 12 |
| `obs_in` | B_in_gL − A: in-FOV LiDAR removed from perception only | +0.16 [-0.12, +0.46] | -0.16 [-0.46, +0.11] | NEGLIGIBLE / NEGLIGIBLE | NEGLIGIBLE / NEGLIGIBLE | 12 / 12 of 12 |
| `gov_in` | B_in − B_in_gL: and from the governor | +0.53 [+0.15, +0.96] | -0.53 [-0.96, -0.16] | SMALL_HIGHER / SMALL_LOWER | UNRESOLVED / UNRESOLVED | 8 / 9 of 12 |
| `obs_all` (OOD) | C_gL − A: all LiDAR removed from perception only | -34.90 [-40.66, -29.00] | +34.90 [+28.88, +40.62] | LOWER / HIGHER | LOWER / HIGHER | 12 / 12 of 12 |
| `gov_all` (OOD) | C − C_gL: camera governor with K2 in place of the raw-LiDAR governor, perception camera-only | -5.05 [-5.83, -4.16] | +5.05 [+4.15, +5.83] | LOWER / HIGHER | LOWER / HIGHER | 12 / 12 of 12 |

The t-intervals and per-block values are in `c2_analysis.json`. The identities total = obs_all + gov_all and
in_fov = obs_in + gov_in hold exactly.

**Protocol deviation (review M-1).**
- The author ran the CPU sensor-level check (`sensor/`) from about 20:35:48 (the review's estimate from file times)
  to 20:37:46, while C2 was running.
  AGENTS.md forbids heavy CPU work alongside an evaluation.
- It overlapped two cells, `no_lidar` 115/4501 (20:33:46-20:36:37) and `no_lidar_obs_only` 115/4501
  (20:36:37-20:39:44). The second took 186 s, the slowest of its condition (others 104-161 s).
- It was not a host fault, and the tree freeze held.
- With that block dropped, every label is unchanged except `interaction` and `gov_in`, which become UNRESOLVED.
  Those two were already fragile (m-2).
- An idle-host rerun of the two cells, compared byte for byte, would test whether the load changed them. It is a
  GPU run and needs the user's approval; until then the deviation stands as disclosed.
- **Condition A inherits a larger deviation, found afterwards.** GR-0, whose S00 cells are condition A, also ran
  alongside the author's CPU jobs, on the GPU job's own cores. 8 of the 12 S00 cells overlapped:
  - 70 bars: seeds 4501 and 4502;
  - 115 bars: seeds 4502 and 4503;
  - 160 bars: seeds 4501, 4502 and 4503;
  - 205 bars: seed 4501.

  The record is on `gainrand/dev` (`352b919`, `docs/evidence/gain_randomization_2026-11-04/load_overlap/`; also
  `~/motar_tmp/load_overlap_20261104/overlap.json`, SHA-256 `80eed1ff…`). The 205/4501 cell ran under load, and
  C1 `full`, run idle twice, is identical to it. The other seven have no idle twin.
- Every C2 contrast involving A (all but `gov_in` and `gov_all`) therefore depends on cells that ran under load.

**Limits from the review.**
- **m-1:** `gov_all` includes the K2 rule (C has it, C_gL does not). It measures "the camera governor with K2 in
  place of the raw-LiDAR governor", not a governor-agnostic value of the LiDAR.
- **m-2:** the SMALL labels of `interaction` and `gov_in` are fragile.
  - Their t-interval ends lie 0.06-0.07 pp from 0, and leave-one-out changes them in several blocks (table).
  - With a Bonferroni correction over 8 contrasts, their adjusted p is 0.33 and 0.25.
  - No conclusion rests on them.
- **m-3:** the plan (`camonly_study_plan_v1.md`, reported items) also listed "crashes with the nearest bar inside
  vs outside the FOV" and "time to capture". The preregistration's reported list did not carry them and neither
  was computed: NOT_REPORTED.
- **m-4:** amendment 1's runner and analysis changes reached the GPU 6 minutes after their commit, checked by the
  author's smoke test only. This review read them at `5522be0` and found no defect.
- **m-5:** the analysis output is bound here by SHA-256 (`f145389c…`), with the analysis script (`f9d709ff…`).
- **m-6:** the magnitudes are specific to this grid. The mean of `total` capture over the three seeds is
  −24.54 pp at 70 bars and −51.45 pp at 205 bars (from `blocks_pp` in `c2_analysis.json`). Per-density values are
  descriptive, not preregistered.

**May be said** (each with "zero-shot, Frozen Policy F, simulation, this grid"; OOD contrasts also with "out of
distribution"):
- `total`, `out_fov` and `obs_all` as in the table. `total` and `out_fov` include K2.
- `gov_all`: "replacing the raw-LiDAR governor by the camera governor with K2, under camera-only perception".
- `in_fov`: "a detected change below the band". It includes:
  - the camera's misses and range;
  - the in-FOV association;
  - the governor's immunity to phantoms;
  - the column offset.

  It is not camera quality.
- `obs_in`: "no change larger than ±3 pp (capture) or ±2 pp (crash)".
- The loss without out-of-FOV LiDAR is "consistent with" F reading the K1 fill as free space. This is not tested.

**May not be said:**
- That the LiDAR or the camera is "necessary", "unnecessary" or "sufficient".
- That removing the in-FOV LiDAR improves F, or anything that rests on `gov_in` or `interaction`.
- "No effect", except for `obs_in`'s band statement.
- Any share of the LiDAR's value split between perception and governor. The fused-perception x camera-governor cell
  was not run.
- Causal mechanisms.
- Anything about a camera-only policy trained for its sensor, a real camera, real flight, or other grids.
- Capture and crash as two independent pieces of evidence (timeouts are 0.004-0.012 %).


## `sensor/`: camera against exact range at Policy F's own poses (CPU, exploratory)

The plan's "Sensor-level check" (section "K5 revised"): resolution is not changed in C2, so this check measures the
camera input instead. Not preregistered, not a gate, and not independently reviewed. No policy is run and no GPU is
used. The plan also asks for the counts by bar position between rays; this run did not compute that split
(NOT_RECORDED).
It ran while C2 was on the GPU, against AGENTS.md; see C2's protocol deviation (review M-1) above.

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


## `mechanism/`: why F crashes without the out-of-FOV LiDAR (CPU, exploratory)

Requested by the user on 2026-11-05 ("explain numerically why the crash rate rises so much with the camera only").
Post hoc, not preregistered, not a gate, not independently reviewed. CPU only (numpy on recorded JSON and traces);
no policy, no rendering, no GPU. It ran with the GPU idle.

| File | What |
| --- | --- |
| `c2_mechanism_telemetry.py` | Reads the 72 cells of C2 (condition A = GR-0 S00 cells): outcomes and crash durations on the quota episodes, the policy-action and speed-governor telemetry of `result.json` (all samples), and the paired first episodes |
| `telemetry_02.json` | Its output, with the SHA-256 of all 216 input files. Version 01 is not kept: it read crash time from `completion_step`, which is a global step counter (the duration only for each env's first episode) |
| `f_trace_geometry.py` | Reads Policy F's own full-LiDAR flights (B1-R `A/F_SHADOW` traces, seed 4301, 70/115/160/205 bars; MOTAR's `motar_eval/occlusion_trace.py` reader, which checks both SHA-256 values) and measures the travel direction and the nearby bars against the camera FOV, in the yaw-only vehicle frame |
| `trace_geometry_02.json` | Its output, with input SHA-256 (version 01 lacked two fields; every shared field is identical) |

Facts used below:
- **FOV coverage.** The camera covers 17 of the 72 LiDAR bins (87°). In conditions without the out-of-FOV LiDAR, the
  other 55 bins read "no return" (K1).
- **Policy F's command.** F runs in vision mode, so its command is a vehicle-frame velocity (x forward along the
  camera axis, y left). Each axis is limited to 2.5 m/s, so the fastest horizontal direction is 45° off the axis
  (3.54 m/s). The camera's half-FOV is 43.5°.
- **Reversals.** "Lateral reversals" counts consecutive steps whose y commands have opposite signs, both
  |y| > 0.1 (`navrl_task.py` action diagnostics).

Table 1. Behaviour (result.json, all samples of the 12 cells; vision mode: vehicle-frame command, y = left)

| Condition | Crash (quota) | Lateral reversals per step | Rightward commands (y < −0.1) | \|y\| ≥ 0.8 | Mean y | Mean \|yaw\| | Speed (m/s) | Governor caps | Speed at contact (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| A `full` | 4.46 % | 2.07 % | 3.4 % | 78.7 % | +0.809 | 0.197 | 2.52 | 28.3 % | 1.27 |
| B_in `no_lidar_in_fov` | 3.76 % | 2.13 % | 3.5 % | 78.3 % | +0.805 | 0.194 | 2.52 | 27.6 % | 1.30 |
| B_in_gL `no_lidar_in_fov_obs_only` | 4.30 % | 2.18 % | 3.6 % | 78.2 % | +0.804 | 0.195 | 2.52 | 28.4 % | 1.30 |
| B_out `no_lidar_out_fov` | 43.71 % | 0.14 % | 0.3 % | 89.7 % | +0.914 | 0.146 | 1.87 | 85.3 % | 0.84 |
| C `no_lidar` | 44.40 % | 0.15 % | 0.4 % | 89.8 % | +0.914 | 0.145 | 1.86 | 85.0 % | 0.84 |
| C_gL `no_lidar_obs_only` | 39.36 % | 0.21 % | 0.6 % | 84.4 % | +0.882 | 0.188 | 2.39 | 41.2 % | 0.81 |

Table 2. Paired first episodes (env_episode_index 0; same initial state in A and X, checked by `initial_state_hash`)

| X | Pairs | Hash mismatches | A capture → X crash | A crash → X capture | Both crash | New crash time p10 / p50 / p90 (s) |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| B_in | 1536 | 0 | 27 (1.8 %) | 35 (2.3 %) | 32 (2.1 %) | 1.3 / 4.2 / 6.8 |
| B_in_gL | 1536 | 0 | 25 (1.6 %) | 39 (2.5 %) | 28 (1.8 %) | 1.2 / 4.3 / 8.8 |
| B_out | 1536 | 0 | 573 (37.3 %) | 20 (1.3 %) | 47 (3.1 %) | 1.3 / 3.3 / 7.8 |
| C | 1536 | 0 | 592 (38.5 %) | 20 (1.3 %) | 47 (3.1 %) | 1.3 / 3.2 / 7.9 |
| C_gL | 1536 | 0 | 527 (34.3 %) | 25 (1.6 %) | 42 (2.7 %) | 1.4 / 2.6 / 5.0 |

Table 3. When crashes happen (quota episodes; duration = kroc2027 `actions` x 0.1 s)

| Condition | Crashes | Median crash time (s) | Episodes crashing within 2 s | within 5 s |
| --- | ---: | ---: | ---: | ---: |
| A | 1095 | 4.1 | 1.40 % | 2.94 % |
| B_in | 925 | 3.8 | 1.23 % | 2.60 % |
| B_in_gL | 1056 | 3.9 | 1.38 % | 2.97 % |
| B_out | 10741 | 3.3 | 11.71 % | 31.65 % |
| C | 10912 | 3.3 | 12.24 % | 32.16 % |
| C_gL | 9672 | 2.7 | 11.86 % | 35.31 % |

Table 4. Policy F's own flights with the full LiDAR (B1-R `A/F_SHADOW`, seed 4301) against the camera FOV (±43.5°), with C2's crash change by density

| Bars | Frames | Travel outside FOV | Travel to the left | Mean \|travel bearing\| | Speed (m/s) | Nearest bar ≤ 2 m outside FOV | Imminent bars (TTC < 1 s) outside FOV | Episodes with a blind-imminent frame | F crashes: contact bar outside FOV / on the left | C2 crash C − A |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| 70 | 26368 | 59.5 % | 98.0 % | 44.8° | 2.80 | 74.3 % | 50.8 % | 47.9 % | 9: 55.6 % / 55.6 % | +24.53 pp |
| 115 | 34432 | 53.3 % | 97.3 % | 43.4° | 2.72 | 75.0 % | 52.7 % | 67.3 % | 20: 70.0 % / 80.0 % | +37.24 pp |
| 160 | 48768 | 49.6 % | 97.1 % | 42.7° | 2.64 | 73.8 % | 51.8 % | 84.5 % | 56: 41.1 % / 64.3 % | +46.57 pp |
| 205 | 37376 | 47.8 % | 97.0 % | 42.2° | 2.53 | 74.3 % | 52.0 % | 86.9 % | 73: 53.4 % / 76.7 % | +51.45 pp |
| pooled | 146944 | 51.8 % | 97.3 % | 43.1° | 2.66 | 74.3 % | 52.0 % | 73.5 % | 158: 51.3 % / 71.5 % | +39.95 pp |

Travel-bearing histogram (pooled moving frames, 15° bins, share): -30..-15: 0.6 %, -15..0: 1.3 %, 0..15: 2.8 %, 15..30: 7.7 %, 30..45: 43.5 %, 45..60: 36.8 %, 60..75: 5.3 %, 75..90: 0.7 %

Definitions for Table 4:
- *Imminent*: a bar within 2 m surface distance, closing, with time to contact under 1 s.
- *Blind-imminent frame*: there are imminent bars and all of them are outside the FOV.
- *Contact bar*: the bar nearest at the last recorded frame of a crash episode, about 0.1 s before contact.

**What the numbers show** (descriptive, this grid):
1. **Where F flies.** With the full LiDAR, F flies diagonally forward-left: 97.3 % of moving frames go left, the mean
   travel bearing is 43.1° and 80.3 % of frames fall between 30° and 60°. In 51.8 % of moving frames the travel
   direction is outside the camera FOV. Its commands are diagonal too (A, all samples: mean |x| 0.96 and |y| 0.85 of
   the limit, in `telemetry_02.json`), and its mean speed, 2.66 m/s, is above the 2.5 m/s per-axis limit.
2. **Where the threats are.** Of the bars that would be hit within 1 s, 52.0 % are outside the FOV. 73.5 % of F's
   episodes contain at least one frame in which every such bar is outside the FOV. This share rises with density,
   from 47.9 % at 70 bars to 86.9 % at 205.
3. **What changes without the out-of-FOV LiDAR.**
   - F's lateral reversals fall from 2.07 % to 0.15 % of steps (C), and its rightward commands fall from 3.4 % to
     0.4 %. It keeps the left diagonal: |y| ≥ 0.8 in 89.8 % of samples, against 78.7 % in A.
   - Removing only the in-FOV LiDAR leaves this pattern as it was (B_in: 2.13 %, 3.5 %, 78.3 %).
4. **Outcome.**
   - From identical initial states, 592 of 1536 first episodes (38.5 %) that A captured crash in C, while 20 (1.3 %)
     go the other way.
   - The new crashes come early: median 3.2 s, p10 1.3 s. 12.2 % of C's quota episodes crash within 2 s,
     against 1.4 % in A.
5. **The governor does not make up for it.**
   - Under K2, 85.0 % of steps are capped, and speed falls from 2.52 to 1.86 m/s. C still crashes in 44.40 % of
     episodes, at 0.84 m/s at contact.
   - With the raw full LiDAR in the governor (C_gL), crashes are 39.36 %. The governor scales speed and keeps the
     direction (riskcap, `direction_preserved`); it does not turn.
6. **Density.** The crash change C − A grows with density, from +24.53 pp at 70 bars to +51.45 pp at 205. The share of
   F's episodes with a blind-imminent frame grows in the same way.

**May be said** (this grid, Frozen Policy F, zero-shot, K1 and K2):
- Policy F's own full-LiDAR flight puts the travel direction outside the camera FOV about half the time. About half
  of its imminent bars are outside the FOV.
- Without the out-of-FOV LiDAR, F almost stops reversing laterally, and it keeps its diagonal. Most C crashes come
  from scenes that A captured.
- These facts are consistent with this explanation: the crashes come from bars beside the diagonal path that only
  the out-of-FOV LiDAR saw, and F does not react to them.

**May not be said:**
- That this explanation is the measured cause. The contact bar's bearing in C2's crashes is NOT_MEASURED, because
  C2 recorded no per-step traces. The geometry in Table 4 is F's full-LiDAR flight on another seed, 4301.
- That the diagonal comes from the per-axis limit. This is consistent with the speed above 2.5 m/s, but untested.
- Anything about a policy trained without the LiDAR (C3/C4). Such a policy could learn to keep its travel direction
  inside the FOV, at a lower top speed (2.5 m/s forward instead of 3.54 m/s on the diagonal). That is a prediction,
  not a result.

**Tests that would settle the cause** (GPU; not run, for the user's decision; the user asked on 2026-11-05 to run GPU
work later):
1. A per-step trace of C and B_out on a subset of C2's cells, with the occlusion trace recorder: the contact bar's
   bearing and whether it was ever inside the FOV.
2. A zero-shot rotation control: give the camera-only F a camera aimed along its travel direction, or a wider FOV.
   If the cause is FOV geometry, the crash rate falls toward A.

## `c2trace/`: G3, which bar camera-only F hits (GPU trace and CPU analysis, preregistered)

- 12 traced reruns of C2 cells on `c4bc416` (2026-11-05). Gate NONINTERFERENCE_PASS. Reading **BLIND_SIDE_SUPPORTED**.
- Independent review r1 (Opus 5.5): PASS_WITH_LIMITS.
- The numbers, the per-density counts (review F1), the camera-eye variant (F4) and the limits are in
  [`c2trace/README.md`](c2trace/README.md).

## `camt1/`: CAM-T1, where CAM-K2 v3 crashes and its speed-cap / delay envelope (GPU, preregistered)

- 64 cells on `b3b9f0b` (`camonly/t1`, 2026-11-12): identity IDENTITY_PASS, trace (Stage A) NONINTERFERENCE_PASS,
  control CONTROL_PASS, envelope (Stage B) COMPLETE; Stage A **DEFINITION_SENSITIVE** (no single retraining lever);
  Stage B d1 (one-step delay) **WORSE** in both seeds, both speed caps SEEDS_DISAGREE.
- Independent results review r1 (Opus 5.5): ACCEPT_WITH_LIMITS.
- The numbers, the wording limits and the file list are in
  [`camt1/RESULTS_2026-11-12.md`](camt1/RESULTS_2026-11-12.md).
