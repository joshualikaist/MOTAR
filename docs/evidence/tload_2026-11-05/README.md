# T-LOAD evidence (2026-11-05): concurrent CPU load does not change these evaluation cells

- **Preregistration:** [`docs/plans/prereg_tload_v1.md`](../../plans/prereg_tload_v1.md).
- **Run:** commit `812f6b1`, runner `tools/tload_run.py` under `taskset -c 0-3,8-11,14-23`, 02:05-02:50 KST.
- **Host fault count:** 0 before and 0 after.
- **Reading: LOAD_INVARIANT.** Every load arm equals L0a in every cell (`arm_state`: L1, L2 and L3 EQUAL).
- **Independent review r1** (Claude Opus 5.5, fresh context, `REVIEW_r1.md`, recomputation `verify_r1.py`):
  **PASS_WITH_LIMITS**. It recomputed every hash and load figure from the raw files and found no blocker.
  - MAJOR-1: the rule went further than this evidence. Answered by narrowing AGENTS.md and D-2026-11-05-1: timing
    results, real-time SITL and untested CPU-side decision paths keep an idle host.
  - The MINOR findings are answered below.
- **Runner wrapper:** the outer `timeout -k 120 9000` (2.5 h) is recorded in `tload_run.out` (MINOR-6).

| File | What |
| --- | --- |
| `tload_report.json` | The runner's report: preflight, every run's digests, equality, load CPU and the reading |
| `SHA256_MANIFEST_tload_v1.json` | SHA-256 of all 121 raw files under `results/tload_v1/` (main checkout) |
| `tload_run.out` | The runner's stdout (`~/motar_tmp/tload_20261105/tload.log`) |
| `REVIEW_r1.md`, `verify_r1.py` | The independent results review and its own recomputation script |

| Run | Cell | Arm | Load cores | Wall (s) | Wall vs L0a | Equal to L0a | `episodes.jsonl` SHA-256 (first 12) |
| ---: | --- | --- | ---: | ---: | ---: | --- | --- |
| 1 | `full 205/4501` | L0a | — | 176 | +0 % | (reference) | `8899856375b1` |
| 2 | `full 205/4501` | L1 | 3.99 | 203 | +15 % | yes | `8899856375b1` |
| 3 | `full 205/4501` | L2 | 16.84 | 275 | +57 % | yes | `8899856375b1` |
| 4 | `full 205/4501` | L3 | 1.00 | 166 | -5 % | yes | `8899856375b1` |
| 5 | `full 205/4501` | L0b | — | 168 | -4 % | yes | `8899856375b1` |
| 6 | `no_lidar 205/4501` | L0a | — | 172 | +0 % | (reference) | `4a161f9eb0f0` |
| 7 | `no_lidar 205/4501` | L1 | 4.00 | 213 | +24 % | yes | `4a161f9eb0f0` |
| 8 | `no_lidar 205/4501` | L2 | 16.86 | 296 | +72 % | yes | `4a161f9eb0f0` |
| 9 | `no_lidar 205/4501` | L3 | 1.00 | 169 | -1 % | yes | `4a161f9eb0f0` |
| 10 | `no_lidar 205/4501` | L0b | — | 169 | -1 % | yes | `4a161f9eb0f0` |
| 11 | `full 70/4501` | L0a | — | 106 | +0 % | (reference) | `11e7ec818586` |
| 12 | `full 70/4501` | L1 | 4.00 | 135 | +27 % | yes | `11e7ec818586` |
| 13 | `full 70/4501` | L2 | 16.72 | 169 | +59 % | yes | `11e7ec818586` |
| 14 | `full 70/4501` | L3 | 1.00 | 107 | +0 % | yes | `11e7ec818586` |
| 15 | `full 70/4501` | L0b | — | 107 | +0 % | yes | `11e7ec818586` |

Equality covers `episodes.jsonl`, `kroc2027_episodes.jsonl` and `gr_run.identity_compare` `result_equal`.

**Checks in the preregistration:**
- **Gate references:** L0a of `full` 205/4501 equals C1 and C1_r2 `full`. L0a of `no_lidar` 205/4501 equals C1,
  C1_r2 and C2 `no_lidar`. So this commit reproduces the recorded idle runs.
- **Manipulation check:**
  - Every load ran from before its cell started until it ended, and held its cores: L1 3.99-4.00, L2 16.72-16.86,
    L3 1.00. The floors were 3.0, 12.0 and 0.8.
  - L2 lengthened the cells by 57-72 %, so the load reached the runner.
- **Wall times:** the first cell's L0a ran about 4 % slower than its L0b, a warm-up effect. Read its wall-time
  ratios with that in mind (MINOR-5).
- **Reported, not gated:** the GR-0 S00 cells of 2026-11-04 ran under load, and both equal L0a here. The load
  figures are from `gainrand/dev` `352b919` `docs/evidence/gain_randomization_2026-11-04/load_overlap/overlap_table.md`
  (MINOR-3).
  - 205/4501 ran under a one-process same-core probe for 164 of 170 s.
  - 70/4501 ran under the C-Planner D2-A1 gate, a heavy same-core job with 2 workers and a driver, for 95 s of its
    run.

**May be said:**
- On this host and commit, CPU load did not change the per-episode outcome files of these three cells.
  - This holds up to 4 disjoint cores plus a 1 GB disk stressor, about 16.8 core-equivalents on the runner's own
    18 cores, and the real Warp sensor-check job on 1 core (MINOR-4).
- The 2026-11-04 GR-0 cells 205/4501 and 70/4501, run under load, are confirmed unaffected.
- Under D-2026-11-05-1 as narrowed after review, CPU compute is allowed during Isaac Gym evaluation cells whose
  result is their outcome files, under AGENTS.md's conditions.

**May not be said:**
- That load can never change an evaluation. The test covers three cells, one seed, these loads and this host. A
  timing effect rare enough to change 1 run in 50 could pass 15 runs.
- That the remaining overlapped cells of 2026-11-04 are identical (MINOR-2).
  - The 352b919 record lists 51 overlapped evaluation cells: identity_02 1, GR-0 23, GR-1 evaluation 25, C2 2.
  - Byte-level twins now cover 4 of them: identity_02 `off`, GR-0 S00 205/4501, GR-0 S40 205/4501 and GR-0 S00
    70/4501. 47 remain without one.
  - The idle rerun (queue G2) is optional.
- Anything about wall-clock timing, memory-bandwidth load (not tested) or CPU-side decision paths (MAJOR-1,
  MINOR-8).
- Anything about the 2026-09-05 difference (another pipeline). Its cause stays unidentified.
