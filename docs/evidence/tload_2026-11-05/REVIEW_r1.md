# T-LOAD results review r1 (2026-11-05)

- **Reviewer:** independent fresh-context subagent, Claude Opus 5.5 (`claude-opus-5-5`), read-only. It wrote only to
  `~/motar_tmp/tload_20261105/review/`.
- **Inputs:** `docs/plans/prereg_tload_v1.md`, `tools/tload_run.py`, `tools/camonly_run.py` and `tools/gr_run.py`
  (tree `src/MOTAR-camonly`, HEAD `26a4a57`, clean); AGENTS.md "Concurrent work"; DECISIONS D-2026-11-05-1;
  `docs/evidence/tload_2026-11-05/*`; raw `results/tload_v1/` (15 runs); the references from prereg section 4;
  load logs in `~/motar_tmp/tload_20261105/load/`; `gainrand/dev` `352b919` `load_overlap/` for the GR-0 overlap
  numbers.
- **Own artefacts:** `verify_r1.py` is a stdlib-only re-derivation and does not import the runner.
  `verify_r1_out.json` is its output. `_overlap_352b919.json` is a copy of the committed overlap record.
  - All compute ran with `CUDA_VISIBLE_DEVICES=""`, under `taskset -c 6,7,12,13 nice -n 10`.

## Verdict: PASS_WITH_LIMITS

The reading **LOAD_INVARIANT is correct and not vacuous.** I recomputed it from the raw files and got the same
result.
- No BLOCKER.
- One MAJOR. It concerns how far the *consequence* in AGENTS.md / D-2026-11-05-1 reaches, not the reading.
- Eight MINOR, about wording, sourcing and process.

## 1. Runner vs preregistration

| Item | Prereg | Runner | OK |
| --- | --- | --- | --- |
| Cells and order | full 205/4501, no_lidar 205/4501, full 70/4501 | `CELLS`, outer loop | yes |
| Arms and order | L0a, L1, L2, L3, L0b in each cell, 15 runs | `ARMS`, inner loop, `NN_<arm>` folders 01-15 | yes |
| L1 | `stress-ng --cpu 4 --hdd 1 --hdd-bytes 1G`, temp under the scratch, cores 6,7,12,13 | same, `--temp-path` = the load scratch | yes |
| L2 | `stress-ng --cpu 18`, runner cores, nice 0 | same, no nice | yes |
| L3 | sensor_check.py copy, looped, nice 10, cores 6,7,12,13 | same. The copy's SHA `7adf2bf3…` equals `812f6b1:docs/evidence/camonly_2026-11-04/sensor/sensor_check.py` | yes |
| Hygiene | `CUDA_VISIBLE_DEVICES=""`, AS cap 4/24 GB, own process group, starts 5 s before | `env`, `RLIMIT_AS`, `setsid`, `sleep(5)` | yes |
| Manipulation check | alive before and at the end; group CPU/wall ≥ 3.0 / 12.0 / 0.8 | `alive_before_cell`, `alive_at_stop`, `group_cpu_s / sampled_wall_s` | yes |
| Equality | both jsonl files byte-identical + `identity_compare.result_equal` | `equal()`: SHA-256 of both files (None → unequal) and `G.identity_compare` (exception → unequal) | yes |
| Precedence | VOID/INCOMPLETE > REF_MISMATCH > NONDET_IDLE > LOAD_NOT_APPLIED > SENSITIVE > DISJOINT_ONLY > INVARIANT | `reading()` in that order. The smoke test covers all 9 outcomes | yes |
| Host | fault count at start and after every cell; a rise voids | `G.check_cell` compares fault_count, boot_id and online; `HOST…` → VOID_HOST_FAULT | yes |
| Affinity | runner under `taskset -c 0-3,8-11,14-23` | preflight compares `sched_getaffinity`; `preflight_problems` is `[]` | yes |
| `timeout -k` | outer bound | not recorded anywhere (MINOR-6) | unverifiable |

**Could the reading be vacuous? No.**
- **Distinct runs.** All 15 cells have distinct `evaluation_nonce` values and distinct inodes. No file is a symlink.
  - Every receipt names commit `812f6b1`. The references were made on commits `1908fd8`, `5522be0` and `c56be07`.
  - `dry_run` is false, and every cell has an Isaac Gym `cell.log` ("Using GPU PhysX"). So `fake_run_cell` copies
    were not used.
- **Never a file compared with itself.** Every `eq_L0a` compares two different run folders.
- **Missing files compare unequal.** Exceptions do too.
- **The comparator detects differences:**
  - L0a `full` 205 ≠ L0a `no_lidar` 205 in both files and in result;
  - `full` 205 ≠ `full` 70;
  - between twins, the only raw `result.json` difference is `condition.evaluation_nonce`. The `checkpoint` field is
    the F SHA in both, so the comparison strips nothing else.
- **The loads ran:**
  - stress-ng reports "successful run completed" in every L1 and L2 log;
  - the L1 hdd stressor did 458,752 / 458,752 / 294,912 bogo-ops;
  - the group-CPU measure agrees with stress-ng's own usr+sys, for example run 02: 829.36 s against 818.25 + 11.59 s,
    and run 03: 4716.37 s against 4717.08 s;
  - L3 produced 13 complete `sc_*.json` files, byte-identical to each other (md5 `74d92102…`). Warp logs "CUDA
    devices not available", so the load did not touch the GPU;
  - every load started before its cell and was alive after the cell finished. Load `t0` is earlier than receipt
    `started_at`, and `t0 + sampled_wall_s` is later than `finished_at`, in all 9 load runs.
- **Per-episode files are very sensitive.** They hold float32 `min_relative_distance_m`, `completion_step`,
  `completion_rank` (cross-env ordering), initial-state hashes and so on. A timing-induced change would show.

## 2. Recomputed reading (`verify_r1_out.json`)

- **Hashes.** `episodes.jsonl` / `kroc2027_episodes.jsonl` are identical within each cell, across all 5 arms:
  - full 205: `8899856375b1…` / `3eb439456a0e…`;
  - no_lidar 205: `4a161f9eb0f0…` / `adc872d7015c…`;
  - full 70: `11e7ec818586…` / `9f96ecf1edbb…`.

  All equal the prereg section 4 values in full length.
- **Equality.** Each L1, L2, L3 and L0b equals L0a: files equal, `result_equal` (nonce and checkpoint removed) and
  `equal`, in all 3 cells.
- **Gates.** L0a 205 `full` equals C1 and C1_r2 `full`. L0a `no_lidar` equals C1, C1_r2 and C2 `no_lidar`. The
  reported GR-0 S00 references for 205/4501 and 70/4501 are equal too.
- **Reading.**
  - gate_ok, idle_ok;
  - L1, L2 and L3 EQUAL;
  - manipulation checks all pass: L1 3.99 / 4.00 / 4.00, L2 16.84 / 16.86 / 16.72, L3 1.00 / 1.00 / 1.00.
  - **→ LOAD_INVARIANT**, the same as the report.
- **Records.**
  - The committed `tload_report.json` is byte-identical to the raw copy.
  - The manifest's 121 entries match the 121 files on disk: 0 mismatches, none missing, none extra.
  - `tload_run.out` is byte-identical to `~/motar_tmp/tload_20261105/tload.log`.

## 3. Receipts and host

- **All 15 receipts:**
  - `status` OK, `git_commit` `812f6b1…`, `git_dirty_files` 0, `checkpoint_sha256` `f7022139…`;
  - `NAVRL_SENSOR_ABLATION` equals the cell condition, and so does the ablation receipt's `condition`;
  - module SHA `ea9ccacb…`; LiDAR 72×4; no `NAVRL_GAIN*`; backend `direct_l2_v1`; quota 16; bars and seed match;
  - `result_sha256` matches `result.json`;
  - 2,048 quota rows. Outcomes per cell: 1867/181/0, 804/1244/0 and 2020/28/0, the same in every arm.
- **Host:**
  - fault count 0 at start, 0 after every cell and 0 at the end; boot id `407ae422…` unchanged; online `0-3,6-23`;
  - at review time the journal of the same boot still counts 0.
- **Tree hygiene:**
  - No load wrote outside its scratch. Nothing appeared in `~/.cache/warp` and no new `__pycache__` appeared in the
    cplanner tree during 02:05-02:51.
  - `motar_eval/__pycache__` in the camonly tree was created at 02:05:47, during the idle L0a. So the evaluation
    process wrote it, not a load. It is git-ignored, the tree is clean, and the file has no effect on outcomes.

## 4. Evidence README

- **Numbers.** Every number in the run table matches the report after rounding:
  - wall times;
  - "Wall vs L0a": +15.4, +56.6, −5.3, −4.4, +24.2, +72.3, −1.3, −1.5, +27.0, +58.9, +0.4 and +0.4 %;
  - load cores;
  - the 12-character SHA prefixes;
  - the time window 02:05-02:50, the host faults 0/0 and the 121 files.
- **GR-0 overlap figures.** "164 of 170 s", "95 s" and "2 workers and a driver" are correct against
  `gainrand/dev 352b919` `load_overlap/overlap_table.md`, but the README does not cite that record (MINOR-3).
- **"May be said" 1 and 2 are supported.** Wording points:
  - "confirmed unaffected" rests on byte equality with an idle rerun on another commit and launcher. That is direct
    evidence. "byte-identical to idle reruns" would be the more exact wording.
  - "17 of the runner's own 18 cores" stands for 16.72-16.86 core-equivalents of CPU time (MINOR-4).
- **"May be said" 3 ("is now allowed") was written before this review** (MINOR-1).
- **"May not be said" is appropriate.** The count "48" is off by one against the record (MINOR-2).

## Findings

### MAJOR

**MAJOR-1: the permission reaches further than the evidence.** This does not block the reading.
- **What the evidence covers.** T-LOAD shows outcome-file invariance for one deterministic GPU-PhysX Isaac Gym
  pipeline: the KROC2027/camonly launcher, Policy F, three cells and one seed.
- **What the rule permits.** AGENTS.md "Concurrent work" and D-2026-11-05-1 permit CPU compute during *any*
  evaluation, under "While a GPU run or a guarded runner is active".
- **The run itself shows that "disjoint" cores are not isolated for performance.** L1, on cores 6,7,12,13, slowed the
  cells by 15-27 %.
  - So every wall-clock quantity is load-sensitive by construction: latency gates and benchmarks (for example
    C-Planner M3 G-beta), renderer benchmarks, receipt `duration_s`, and `perf_counter` telemetry
    (`navrl_task.py:5259`, `target_route_planner.py:1138`, `motar_eval/boia_shadow.py`,
    `motar_eval/q02/pipeline.py`).
  - Evaluation paths with CPU-side logic were not tested: classical baselines, CPU replays, the route planner
    (`NAVRL_TARGET_ROUTE_MODE` was off).
- **Fix (no rerun needed).** A dated clarification to D-2026-11-05-1 and AGENTS.md:
  - the permission covers outcome-determinism of GPU-pipeline evaluation cells;
  - timing, latency and throughput measurements, and evaluation paths with wall-clock-dependent logic, are
    excluded unless separately checked.

### MINOR

**MINOR-1: the consequence was asserted before the review.**
- "CPU compute during evaluations is now allowed" appears in:
  - the README "May be said";
  - the EXPERIMENTS *Status*;
  - the WORKLOG entry;
  - the commit message of `26a4a57`.
- AGENTS.md requires CONDITIONAL at most until the independent review finishes. The README does say that the label
  awaits review. I found no evidence that the permission was used before review.
- "The G2 idle rerun is optional" is likewise not stated in the prereg or the decision. The earlier record said the
  rerun needs the user. It should be framed as the user's decision.

**MINOR-2: "the other 48 overlapped cells" does not match the record.**
- `352b919` `overlap.json` lists 51 overlapped evaluation cells: GR-0 main 23, GR-1 eval 25, C2 2 and identity_02 1.
- Three were already twin-confirmed before T-LOAD: identity_02 `off`, GR-0 S00 205/4501 and S40 205/4501.
- T-LOAD adds S00 70/4501, so 47 remain unconfirmed.
- The prereg's "50" is also inexact. The number should be computed from the record.

**MINOR-3: the GR-0 overlap figures lack a source.** The README's 164/170 s, 95 s and "2 workers + driver" are right,
but it does not cite their record (`gainrand/dev 352b919`, `docs/evidence/gain_randomization_2026-11-04/load_overlap/`).

**MINOR-4: "17 of the runner's own 18 cores".** The measurement is 16.72-16.86 core-equivalents consumed by the load.

**MINOR-5: the first cell's "Wall vs L0a" column carries a warm-up bias.**
- L0a of the first cell took 175.6 s. Its L0b took 167.9 s, and the historical idle runs took 168-169 s. This is
  the first cell of the session, which compiled `motar_eval` bytecode.
- So that cell's L1 +15 % and L3 −5 % are partly artefact. Reported only, not gated.

**MINOR-6: the outer `timeout -k` is not recorded.** The prereg names it, but neither the report nor the log records
it. The affinity is verified by the preflight.

**MINOR-7: no independent design review before launch.** The WORKLOG of `812f6b1` discloses this. This audit found no
design defect after the fact.

**MINOR-8: load coverage, recorded as a limit.**
- L3 is light (1.00 core). The main evidence for the configuration that is actually permitted (disjoint cores) is L1:
  4 cores plus disk at nice 0, which is stronger than the permitted nice 10.
- No memory-bandwidth or LLC-thrashing load (`--vm`, `--stream`) was tested. L2's +57-72 % slowdown dominates the
  timing perturbation.
- The rare-event limit (12 loaded runs) is stated correctly in the prereg and the README.

## Questions answered

1. **Design.** The runner implements the preregistered design, readout and precedence. No vacuity route was found.
2. **Recomputation.** It reproduces LOAD_INVARIANT exactly, with all hashes and manipulation numbers as reported.
3. **Receipts and host.** All 15 receipts are clean, and the host fault record is 0 throughout.
4. **README.** Its numbers are exact. It overstates mildly in three places: the pre-review "now allowed", "48", and
   "17 of 18".
5. **Blockers.** Nothing blocks LOAD_INVARIANT as the reading. Before the permission is relied on outside the tested
   pipeline, it should be narrowed (MAJOR-1).
