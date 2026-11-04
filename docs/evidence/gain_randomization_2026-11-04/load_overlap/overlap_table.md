# CPU jobs that ran alongside GPU simulation cells, 2026-11-04 (KST)

Read-only forensic reconstruction. All times are KST (UTC+9). Machine-readable twin of this file: `overlap.json`. Scripts in this folder.

## Method

- **gpu_cell_times**: start = host_after.time - wall_s; end = host_after.time (per-cell records). GR-1 training: started/finished of each arm.
- **job_start**: timestamp of the assistant tool_use block that launched the job (UTC -> KST).
- **job_end**: foreground: timestamp of its tool_result; run_in_background or auto-backgrounded: main = queue-operation enqueue of the completion notification, subagent = attachment record of the notification; detached '&' launches: output-file mtimes in ~/motar_tmp. Phase splits from output/log mtimes.
- **overlap**: job [start,end) intersects cell [start,end). Precision about 1 s; tool_result timestamps include under 1 s of harness overhead.
- **classes**: heavy = multi-process, >= 60 s, suite, gate, install or clone; moderate = single-process compute 3-60 s; minor = 1.5-3 s script/test/build runs and borderline disk commands (counted separately). Ignored: reads, edits, git status/log/diff/commit (no hooks installed), polls and sleep loops.
- **gpu_job_cpus**: every GPU runner was pinned to taskset 0-3,8-11,14-23 (cpu4/5 offline). 6,7,12,13 = two whole P-cores disjoint from that set (i7-13700K, siblings 6-7 and 12-13).

## Summary per GPU unit

| unit | cells | overlapped (any) | overlapped (heavy+moderate) | overlapped (heavy) | with a job on the GPU job's CPUs | only jobs on disjoint cores |
|---|---|---|---|---|---|---|
| GR-0 identity_01 | 4 | 0 | 0 | 0 | 0 | 0 |
| GR-0 identity_02 | 4 | 1 | 1 | 0 | 1 | 0 |
| GR-0 main | 36 | 23 | 22 | 22 | 22 | 0 |
| GR-1 train/rand40 | 1 | 1 | 1 | 1 | 1 | 0 |
| GR-1 train/ctrl | 1 | 1 | 1 | 1 | 1 | 0 |
| GR-1 repro | 2 | 0 | 0 | 0 | 0 | 0 |
| GR-1 eval/rand40 | 36 | 9 | 7 | 5 | 0 | 8 |
| GR-1 eval/ctrl | 36 | 16 | 16 | 7 | 0 | 16 |
| CamOnly C1/c1 | 2 | 0 | 0 | 0 | 0 | 0 |
| CamOnly C1_r2/c1_r2 | 2 | 0 | 0 | 0 | 0 | 0 |
| CamOnly C2 | 60 | 2 | 2 | 0 | 0 | 2 |

GR-1 training counts each arm as one cell (training runs have only started/finished).

### GR-0: overlapping job types (job [cores, processes, class]: cells overlapped)

- dirbound review probe P4 per-step G-alpha diff B-PP:205 [0-3,8-11,14-23, 1p, heavy]: 8
- dirbound review probe P5 eps=0 old-vs-new core [0-3,8-11,14-23, 2p, heavy]: 7
- C-Planner D2-A1 gate (run_dirbound_a1.py) [0-3,8-11,14-23, 2p, heavy]: 4
- dirbound review probe P1 native [0-3,8-11,14-23, 2p, heavy]: 2
- dirbound review probe P1 verbatim [0-3,8-11,14-23, 2p, heavy]: 2
- C-Planner V1-v2 dev check on failing frames [3, 1p, minor]: 1
- C-Planner V1-v2 kernel-ray dev run (Warp CPU) [3,14, 1p, moderate]: 1
- frozen copy for D2-A1 (git archive | tar) [unpinned, 1p, minor]: 1
- dirbound review probe P3 D1 clearance [0-3,8-11,14-23, 1p, minor]: 1
- dirbound review probe P1 timing [0-3,8-11,14-23, 1p, minor]: 1
- dirbound review probe P2 kernel rays [0-3,8-11,14-23, 1p, moderate]: 1
- dirbound review probe P4b (2 parts) [0-3,8-11,14-23, 2p, heavy]: 1
- dirbound review final integrity check [unpinned, 1p, minor]: 1
- commit + unittest + check_public_docs (cplanner) [unpinned, 1p, minor]: 1

### GR-1 training: overlapping job types (job [cores, processes, class]: cells overlapped)

- C-Planner full-bound adversary dev run [14-19, 6p, heavy]: 1
- commit D1-F + unittest + frozen copy [unpinned, 1p, minor]: 1
- C-Planner D1-F gate (run_dirbound_d1f.py) [0-3,8-11,14-23, 6p, heavy]: 1
- freeze D1-F run (cp -a + SHA-256 manifest) [unpinned, 1p, minor]: 1
- D1-F review: build native library (compile) [0-3,8-11,14-23, 1p, minor]: 1
- D1-F review probe P0 sanity [0-3,8-11,14-23, 1p, moderate]: 1
- D1-F review probe P1 smoke [0-3,8-11,14-23, 1p, moderate]: 1
- D1-F review probe P1 (3 shards) [0-3,8-11,14-23, 3p, heavy]: 1
- D1-F review P1 analysis [0-3,8-11,14-23, 1p, minor]: 1
- D1-F review probe P3 thresholds (3 shards) [0-3,8-11,14-23, 3p, heavy]: 1
- D1-F review probes P2 strong (2 shards) + P5 off-sensor (1) [0-3,8-11,14-23, 3p, heavy]: 1
- GR-1 design review: GR-0 variability/power simulation [0-3,8-11,14-23, 1p, heavy]: 1
- D1-F review probe P4 forensic [0-3,8-11,14-23, 1p, moderate]: 1
- camera-only feasibility: Isaac Gym create_sim on CPU [3, 1p, minor]: 1
- camera-only feasibility: Warp CPU depth-camera kernel [3, 1p, minor]: 1
- record D1-F review PASS + test + commit [unpinned, 1p, minor]: 1
- GR-1 design review: gr1_analyse.py on 4 synthetic inputs [0-3,8-11,14-23, 1p, moderate]: 1
- GR-1 amendment: synthetic known-bad/good analysis controls [3,14, 1p, moderate]: 1
- GR-1 amendment: synthetic controls rerun [3,14, 1p, moderate]: 1
- SITL plan: vehicle-spec consistency check [unpinned, 1p, minor]: 1
- SITL setup: PX4 v1.14.3 git clone --recursive (network + disk) [unpinned, 1p, heavy]: 1
- SITL setup: conda create px4 env + pip [unpinned, 1p, heavy]: 1
- SITL setup: pip install PX4 Python deps [unpinned, 1p, heavy]: 1

### GR-1 repro+eval: overlapping job types (job [cores, processes, class]: cells overlapped)

- PUBLIC_CPU baseline then candidate suites (camonly) [6,7,12,13, 1p, heavy]: 4
- camera-only C0 gate_01 [6,7,12,13, 1p, heavy]: 4
- camera-only C0 gate_02 [6,7,12,13, 1p, heavy]: 3
- C0 review r2: M1 governor probe [6,7,12,13, 1p, moderate]: 3
- camera-only C0 tests [6,7,12,13, 1p, minor]: 2
- PUBLIC_CPU suite (camonly) [6,7,12,13, 1p, heavy]: 2
- camera-only C0 repair: tests + source mutants [6,7,12,13, 1p, moderate]: 2
- grep -oE over the 20 MB main transcript [unpinned, 1p, minor]: 1
- C-Planner hardening: native binding (rebuild), static check, planner unit [6,7,12,13, 1p, moderate]: 1
- C-Planner hardening: planner unit on old default [6,7,12,13, 1p, minor]: 1
- CPU perception test + governor load [6,7,12,13, 1p, minor]: 1
- camera-only C0 tests [6,7,12,13, 1p, moderate]: 1
- camera-only C0 tests + mutant known-bad controls [6,7,12,13, 1p, moderate]: 1
- camera-only C0 tests + mutants [6,7,12,13, 1p, moderate]: 1
- camera-only C0 tests incl. entry point [6,7,12,13, 1p, minor]: 1
- camera-only quick checks [6,7,12,13 + unpinned, 1p, minor]: 1
- C0 review r1: association probe [6,7,12,13, 1p, minor]: 1
- C0 review r1: PATH/ninja probe (torch import per PYTHON) [6,7,12,13, 1p, moderate]: 1
- C0 review r1: C0 tests + mutants harness [6,7,12,13, 1p, moderate]: 1
- camera-only C0 repair tests [6,7,12,13, 1p, minor]: 1
- camera-only C0 repair tests [6,7,12,13, 1p, moderate]: 1
- camera-only C0 repair: two new mutants [6,7,12,13, 1p, moderate]: 1
- C0 review r2: tests + mutant harness [6,7,12,13, 1p, moderate]: 1
- C0 review r2: B1 association probe [6,7,12,13, 1p, moderate]: 1
- C0 review r2: M2 obs-only probe [6,7,12,13, 1p, minor]: 1
- C0 review r2: relative-PYTHON launcher failure demo [6,7,12,13, 1p, moderate]: 1
- C0 review r2: extra mutants vs C0 suite [6,7,12,13, 1p, moderate]: 1
- C0 review r2: entry-point/refusal mutants [6,7,12,13, 1p, moderate]: 1

### Camera-only C1/C2: overlapping job types (job [cores, processes, class]: cells overlapped)

- camera-only sensor check, 20-frame trial [6,7,12,13, 1p, moderate]: 1
- camera-only sensor check, 20-frame rerun [6,7,12,13, 1p, moderate]: 1
- camera-only sensor check, full [6,7,12,13, 1p, moderate]: 1

## Heaviest overlaps (process-seconds inside GPU cells)

| rank | job | session | start-end | cores | procs | nice | cells | process-s in cells |
|---|---|---|---|---|---|---|---|---|
| 1 | C-Planner D1-F gate (run_dirbound_d1f.py) | main session | 14:02:25-14:08:11 | 0-3,8-11,14-23 | 6 | 5 | 1 | 2076 |
| 2 | dirbound review probe P5 eps=0 old-vs-new core | C-Planner direction-bound repair independent review (Opus) | 12:21:45-12:35:49 | 0-3,8-11,14-23 | 2 | 0 | 7 | 1688 |
| 3 | dirbound review probe P4 per-step G-alpha diff B-PP:205 | C-Planner direction-bound repair independent review (Opus) | 12:42:53-13:02:55 | 0-3,8-11,14-23 | 1 | 0 | 8 | 1202 |
| 4 | D1-F review probes P2 strong (2 shards) + P5 off-sensor (1) | C-Planner D1-F independent review (Opus) | 14:37:10-14:43:13 | 0-3,8-11,14-23 | 3 | 0 | 1 | 1089 |
| 5 | C-Planner D2-A1 gate (run_dirbound_a1.py) | main session | 11:58:18-12:05:08 | 0-3,8-11,14-23 | 2 | 5 | 4 | 820 |
| 6 | D1-F review probe P1 (3 shards) | C-Planner D1-F independent review (Opus) | 14:19:36-14:23:20 | 0-3,8-11,14-23 | 3 | 0 | 1 | 672 |
| 7 | D1-F review probe P3 thresholds (3 shards) | C-Planner D1-F independent review (Opus) | 14:28:16-14:31:53 | 0-3,8-11,14-23 | 3 | 0 | 1 | 651 |
| 8 | PUBLIC_CPU baseline then candidate suites (camonly) | main session | 17:33:46-17:42:18 | 6,7,12,13 | 1 | 10 | 4 | 511 |
| 9 | SITL setup: PX4 v1.14.3 git clone --recursive (network + disk) | main session | 15:45:57-15:54:18 | unpinned | 1 | 0 | 1 | 501 |
| 10 | camera-only C0 gate_02 | main session | 18:11:46-18:16:14 | 6,7,12,13 | 1 | 10 | 3 | 268 |
| 11 | dirbound review probe P4b (2 parts) | C-Planner direction-bound repair independent review (Opus) | 13:03:20-13:05:32 | 0-3,8-11,14-23 | 2 | 0 | 1 | 264 |
| 12 | camera-only C0 gate_01 | main session | 17:43:42-17:48:02 | 6,7,12,13 | 1 | 10 | 4 | 260 |
| 13 | PUBLIC_CPU suite (camonly) | main session | 17:28:15-17:32:31 | 6,7,12,13 | 1 | 10 | 2 | 256 |
| 14 | SITL setup: pip install PX4 Python deps | main session | 15:47:04-15:50:34 | unpinned | 1 | 0 | 1 | 210 |
| 15 | dirbound review probe P1 native | C-Planner direction-bound repair independent review (Opus) | 12:38:59-12:40:34 | 0-3,8-11,14-23 | 2 | 0 | 2 | 190 |

## Byte-identical twin cells and the load during each run

Twins share byte-identical `episodes.jsonl` and `kroc2027_episodes.jsonl` (result.json differs only by run metadata; its hash is shown). identity_01's four cells also match each other, but only as empty files from failed cells, so they are excluded.

### episodes.jsonl 560097f46a21ff87, kroc2027_episodes.jsonl a8ffb6f9671b6b9e

| cell | wall s | result.json sha256 | CPU load during the cell |
|---|---|---|---|
| GR-0 identity_02 ref 160b s4501 11:54:37-11:55:18 | 40.96 | 167e8a8c60b2 | no CPU job (only reads, edits and polls) |
| GR-0 identity_02 off 160b s4501 11:55:18-11:55:59 | 41.26 | 9038b0308750 | C-Planner V1-v2 dev check on failing frames [3, 1p] 1 s; C-Planner V1-v2 kernel-ray dev run (Warp CPU) [3,14, 1p] 5 s |
| GR-0 identity_02 s00 160b s4501 11:55:59-11:56:40 | 40.56 | a9a9f668d9db | no CPU job (only reads, edits and polls) |

### episodes.jsonl 8899856375b19c6d, kroc2027_episodes.jsonl 3eb439456a0e7055

| cell | wall s | result.json sha256 | CPU load during the cell |
|---|---|---|---|
| GR-0 main S00 205b s4501 13:00:11-13:03:01 | 170.33 | 40ddbfc76cf4 | dirbound review probe P4 per-step G-alpha diff B-PP:205 [0-3,8-11,14-23, 1p] 164 s |
| CamOnly C1/c1 full 205b s4501 19:27:10-19:29:59 | 168.89 | 5697d81be0ce | no CPU job (only reads, edits and polls) |
| CamOnly C1_r2/c1_r2 full 205b s4501 19:51:46-19:54:36 | 169.2 | 5cdf8ca93c00 | no CPU job (only reads, edits and polls) |

### episodes.jsonl f97cffc6480c7785, kroc2027_episodes.jsonl 51ae628c57fbb69e

| cell | wall s | result.json sha256 | CPU load during the cell |
|---|---|---|---|
| GR-0 main S40 205b s4501 13:05:55-13:08:32 | 156.67 | f7e429f01e39 | dirbound review final integrity check [unpinned, 1p] 3 s; commit + unittest + check_public_docs (cplanner) [unpinned, 1p] 2 s |
| GR-1 repro S40 205b s4501 16:07:16-16:09:48 | 151.37 | 37ddffed9a39 | no CPU job (only reads, edits and polls) |
| GR-1 repro S40_gr1launcher 205b s4501 16:09:48-16:12:19 | 151.29 | 18b954871423 | no CPU job (only reads, edits and polls) |

### episodes.jsonl 4a161f9eb0f0792f, kroc2027_episodes.jsonl adc872d7015cded0

| cell | wall s | result.json sha256 | CPU load during the cell |
|---|---|---|---|
| CamOnly C1/c1 no_lidar 205b s4501 19:29:59-19:32:50 | 171.85 | 2c8e4ff7f086 | no CPU job (only reads, edits and polls) |
| CamOnly C1_r2/c1_r2 no_lidar 205b s4501 19:54:36-19:57:26 | 170.34 | b4cac92cfda4 | no CPU job (only reads, edits and polls) |
| CamOnly C2 no_lidar 205b s4501 21:53:35-21:56:23 | 168.13 | ab4e57cf73fa | no CPU job (only reads, edits and polls) |

## CPU jobs that overlapped at least one GPU cell

| # | start-end (KST) | wall s | session | job | command (short) | cores | procs | nice | class | end from | overlapped cells (cell start-end, overlap s) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 11:55:18-11:55:19 | 2 | main session | C-Planner V1-v2 dev check on failing frames — 1.7 s | `python heredoc v1v2 check` | 3 | 1 | 0 | minor | tool_result | GR-0 identity_02 off 160b s4501 (11:55:18-11:55:59, 1) |
| 2 | 11:55:47-11:55:52 | 5 | main session | C-Planner V1-v2 kernel-ray dev run (Warp CPU) | `v1v2a_kernelray.py 30 frames x2 cells` | 3,14 | 1 | 10 | moderate | tool_result | GR-0 identity_02 off 160b s4501 (11:55:18-11:55:59, 5) |
| 3 | 11:58:11-11:58:13 | 2 | main session | frozen copy for D2-A1 (git archive \| tar) — 1.9 s disk | `git archive HEAD ... \| tar -x` | unpinned | 1 | 0 | minor | tool_result | GR-0 main S00 70b s4501 (11:58:04-11:59:53, 2) |
| 4 | 11:58:18-12:05:08 | 409 | main session | C-Planner D2-A1 gate (run_dirbound_a1.py) — 3 sequential phases, each 2 worker processes + driver: a1 11:58:20-12:00:36, dup_a1 12:00:38-12:02:53, a1_d2 12:02:54-12:05:08 (run_a1/logs mtimes) | `run_dirbound_a1.py: a1, dup_a1, a1_d2 (v1v2a_kernelray.py, 1000 frames/cell)` | 0-3,8-11,14-23 | 2 | 5 | heavy | queue notification | GR-0 main S00 70b s4501 (11:58:04-11:59:53, 95); GR-0 main S20 70b s4501 (11:59:53-12:01:42, 109); GR-0 main S40 70b s4501 (12:01:42-12:03:35, 113); GR-0 main S00 70b s4502 (12:03:35-12:05:25, 93) |
| 5 | 12:20:59-12:21:01 | 2 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P3 D1 clearance — 1.5 s | `p3_d1_clearance.py` | 0-3,8-11,14-23 | 1 | 0 | minor | tool_result | GR-0 main S40 115b s4501 (12:19:33-12:21:50, 2) |
| 6 | 12:21:45-12:35:49 | 844 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P5 eps=0 old-vs-new core — both processes ended 12:35:49 | `p5_eps0_diff.py x2 (old,new) in parallel, 60 scenes` | 0-3,8-11,14-23 | 2 | 0 | heavy | auto-background notification + out/p5_eps0_*.summary.json mtimes | GR-0 main S40 115b s4501 (12:19:33-12:21:50, 5); GR-0 main S00 115b s4502 (12:21:50-12:24:47, 177); GR-0 main S20 115b s4502 (12:24:47-12:26:57, 130); GR-0 main S40 115b s4502 (12:26:57-12:29:25, 148); GR-0 main S00 115b s4503 (12:29:25-12:31:31, 126); GR-0 main S20 115b s4503 (12:31:31-12:34:35, 184); GR-0 main S40 115b s4503 (12:34:35-12:36:43, 74) |
| 7 | 12:37:56-12:37:59 | 3 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P1 timing — 2.8 s | `p1_full_eps.py 10 scenes native` | 0-3,8-11,14-23 | 1 | 0 | minor | tool_result | GR-0 main S00 160b s4501 (12:36:43-12:39:23, 3) |
| 8 | 12:38:59-12:40:34 | 95 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P1 native | `p1_full_eps.py x2 seeds, 400 scenes, parallel` | 0-3,8-11,14-23 | 2 | 0 | heavy | tool_result + out/p1_native_*.json mtimes (12:40:30, 12:40:34) | GR-0 main S00 160b s4501 (12:36:43-12:39:23, 24); GR-0 main S20 160b s4501 (12:39:23-12:41:59, 71) |
| 9 | 12:40:59-12:42:22 | 83 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P1 verbatim | `p1_full_eps.py x2 seeds, 60 scenes verbatim, parallel` | 0-3,8-11,14-23 | 2 | 0 | heavy | attachment notification + out/p1_verbatim_*.json mtimes (12:42:20, 12:42:22) | GR-0 main S20 160b s4501 (12:39:23-12:41:59, 60); GR-0 main S40 160b s4501 (12:41:59-12:44:32, 23) |
| 10 | 12:42:36-12:42:42 | 6 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P2 kernel rays | `p2_kernel_rays.py 15` | 0-3,8-11,14-23 | 1 | 0 | moderate | tool_result | GR-0 main S40 160b s4501 (12:41:59-12:44:32, 6) |
| 11 | 12:42:53-13:02:55 | 1201 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P4 per-step G-alpha diff B-PP:205 — 20 min single process | `p4_galpha_diff.py (timeout 1200)` | 0-3,8-11,14-23 | 1 | 0 | heavy | attachment notification (status failed = killed by timeout 1200 s) | GR-0 main S40 160b s4501 (12:41:59-12:44:32, 99); GR-0 main S00 160b s4502 (12:44:32-12:47:07, 155); GR-0 main S20 160b s4502 (12:47:07-12:49:27, 140); GR-0 main S40 160b s4502 (12:49:27-12:52:10, 163); GR-0 main S00 160b s4503 (12:52:10-12:54:34, 144); GR-0 main S20 160b s4503 (12:54:34-12:57:05, 151); GR-0 main S40 160b s4503 (12:57:05-13:00:11, 186); GR-0 main S00 205b s4501 (13:00:11-13:03:01, 164) |
| 12 | 13:03:20-13:05:32 | 132 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review probe P4b (2 parts) — detached (&) | `p4b_galpha_diff.py part 0,1 in parallel` | 0-3,8-11,14-23 | 2 | 0 | heavy | out/p4b_BPP205_part*.json mtimes (13:04:44, 13:05:32) | GR-0 main S20 205b s4501 (13:03:01-13:05:55, 132) |
| 13 | 13:06:10-13:06:13 | 3 | sub: C-Planner direction-bound repair independent review (Opus) | dirbound review final integrity check — 3 s light | `journalctl count, sha256 manifest re-check, find` | unpinned | 1 | 0 | minor | tool_result | GR-0 main S40 205b s4501 (13:05:55-13:08:32, 3) |
| 14 | 13:07:40-13:07:42 | 2 | main session | commit + unittest + check_public_docs (cplanner) — 2.1 s | `python -m unittest, check_public_docs.py, git commit` | unpinned | 1 | 0 | minor | tool_result | GR-0 main S40 205b s4501 (13:05:55-13:08:32, 2) |
| 15 | 14:00:25-14:00:50 | 25 | main session | C-Planner full-bound adversary dev run — 6 workers on 6 CPUs inside the GPU job's set | `check_direction_bound_full.py 120 scenes, 6 workers` | 14-19 | 6 | 10 | heavy | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 25) |
| 16 | 14:02:16-14:02:19 | 3 | main session | commit D1-F + unittest + frozen copy — 3.0 s | `-m unittest, git archive \| tar` | unpinned | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 3) |
| 17 | 14:02:25-14:08:11 | 345 | main session | C-Planner D1-F gate (run_dirbound_d1f.py) — native build 14:02:27; d1f 14:02:27-14:05:18; dup_d1f 14:05:18-14:08:11; 6 worker processes each (run_d1f/logs mtimes) | `native build.py, then check_direction_bound_full.py 2000 scenes x2 (d1f, dup_d1f)` | 0-3,8-11,14-23 | 6 | 5 | heavy | queue notification | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 346) |
| 18 | 14:08:28-14:08:35 | 7 | main session | freeze D1-F run (cp -a + SHA-256 manifest) — 7 s, small files | `cp -a run_d1f; sha256 of 16 files` | unpinned | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 7) |
| 19 | 14:16:34-14:16:36 | 2 | sub: C-Planner D1-F independent review (Opus) | D1-F review: build native library (compile) — 1.7 s compile | `native/build.py` | 0-3,8-11,14-23 | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 2) |
| 20 | 14:18:35-14:18:41 | 6 | sub: C-Planner D1-F independent review (Opus) | D1-F review probe P0 sanity | `p0_sanity.py 12 scenes` | 0-3,8-11,14-23 | 1 | 0 | moderate | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 6) |
| 21 | 14:19:25-14:19:29 | 5 | sub: C-Planner D1-F independent review (Opus) | D1-F review probe P1 smoke | `p1_full.py 12 scenes` | 0-3,8-11,14-23 | 1 | 0 | moderate | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 4) |
| 22 | 14:19:36-14:23:20 | 224 | sub: C-Planner D1-F independent review (Opus) | D1-F review probe P1 (3 shards) — detached (&) | `p1_full.py shards 0,1,2 detached` | 0-3,8-11,14-23 | 3 | 0 | heavy | out/p1_shard*.npz mtimes (14:23:19) | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 224) |
| 23 | 14:25:41-14:25:42 | 1 | sub: C-Planner D1-F independent review (Opus) | D1-F review P1 analysis — 1 s | `p1_analyze.py` | 0-3,8-11,14-23 | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 1) |
| 24 | 14:28:16-14:31:53 | 216 | sub: C-Planner D1-F independent review (Opus) | D1-F review probe P3 thresholds (3 shards) — detached (&) | `p3_thresholds.py shards 0,1,2 detached` | 0-3,8-11,14-23 | 3 | 0 | heavy | out/p3_shard*.json mtimes (14:31:34, 14:31:52, 14:31:53) | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 217) |
| 25 | 14:37:10-14:43:13 | 363 | sub: C-Planner D1-F independent review (Opus) | D1-F review probes P2 strong (2 shards) + P5 off-sensor (1) — detached (&) | `p2_strong.py x2 + p5_offsensor.py detached` | 0-3,8-11,14-23 | 3 | 0 | heavy | out/p5.json 14:42:14, p2_shard0 14:43:04, p2_shard1 14:43:13 mtimes | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 363) |
| 26 | 14:43:32-14:46:39 | 187 | sub: GR-1 pre-measurement design review (Opus) | GR-1 design review: GR-0 variability/power simulation — 3 min single process | `gr0_variability_power.py` | 0-3,8-11,14-23 | 1 | 0 | heavy | auto-background notification + gr0_variability_power.json mtime 14:46:39 | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 187) |
| 27 | 14:43:37-14:43:52 | 15 | sub: C-Planner D1-F independent review (Opus) | D1-F review probe P4 forensic | `p4_forensic.py + numpy one-liner` | 0-3,8-11,14-23 | 1 | 0 | moderate | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 15) |
| 28 | 14:47:57-14:48:00 | 3 | main session | camera-only feasibility: Isaac Gym create_sim on CPU — 3.0 s | `python -c isaacgym create_sim(-1,-1)` | 3 | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 3) |
| 29 | 14:48:44-14:48:47 | 3 | main session | camera-only feasibility: Warp CPU depth-camera kernel — 2.7 s | `cam_cpu_probe.py` | 3 | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 3) |
| 30 | 14:49:14-14:49:16 | 2 | main session | record D1-F review PASS + test + commit — 2.1 s | `python tests, git commit` | unpinned | 1 | 0 | minor | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 2) |
| 31 | 14:49:44-14:49:50 | 6 | sub: GR-1 pre-measurement design review (Opus) | GR-1 design review: gr1_analyse.py on 4 synthetic inputs | `gr1_analyse.py x4 sequential` | 0-3,8-11,14-23 | 1 | 0 | moderate | tool_result | GR-1 train/rand40 train 205b s1 (13:44:00-14:53:47, 6) |
| 32 | 15:09:38-15:09:52 | 14 | main session | GR-1 amendment: synthetic known-bad/good analysis controls | `test_gr1_analyse_synthetic.py` | 3,14 | 1 | 10 | moderate | tool_result | GR-1 train/ctrl train 205b s1 (14:55:03-15:59:37, 14) |
| 33 | 15:10:11-15:10:25 | 14 | main session | GR-1 amendment: synthetic controls rerun | `test_gr1_analyse_synthetic.py` | 3,14 | 1 | 10 | moderate | tool_result | GR-1 train/ctrl train 205b s1 (14:55:03-15:59:37, 14) |
| 34 | 15:43:34-15:43:36 | 2 | main session | SITL plan: vehicle-spec consistency check — 2.2 s | `python3 tools/check_vehicle_spec.py` | unpinned | 1 | 0 | minor | tool_result | GR-1 train/ctrl train 205b s1 (14:55:03-15:59:37, 2) |
| 35 | 15:45:57-15:54:18 | 500 | main session | SITL setup: PX4 v1.14.3 git clone --recursive (network + disk) — 8 min; submodule checkouts write many files | `git clone --recursive --depth 1 PX4-Autopilot` | unpinned | 1 | 0 | heavy | queue notification | GR-1 train/ctrl train 205b s1 (14:55:03-15:59:37, 501) |
| 36 | 15:46:16-15:46:48 | 31 | main session | SITL setup: conda create px4 env + pip — disk + CPU | `conda create -p envs/px4 python=3.8; pip install -r` | unpinned | 1 | 0 | heavy | tool_result | GR-1 train/ctrl train 205b s1 (14:55:03-15:59:37, 32) |
| 37 | 15:47:04-15:50:34 | 210 | main session | SITL setup: pip install PX4 Python deps — 3.5 min disk + CPU | `pip install -r requirements_px4 mavsdk` | unpinned | 1 | 0 | heavy | tool_result | GR-1 train/ctrl train 205b s1 (14:55:03-15:59:37, 210) |
| 38 | 16:14:10-16:14:20 | 10 | main session | grep -oE over the 20 MB main transcript — 10 s single-core grep (borderline light) | `grep -oE '.{300}(...)' session.jsonl` | unpinned | 1 | 0 | minor | tool_result | GR-1 eval/rand40 S00 70b s4501 (16:12:44-16:14:40, 10) |
| 39 | 16:48:32-16:48:35 | 4 | main session | C-Planner hardening: native binding (rebuild), static check, planner unit — sequential | `check_native_binding / check_static / planner_unit.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/rand40 S40 115b s4503 (16:47:44-16:49:58, 3) |
| 40 | 16:48:59-16:49:02 | 3 | main session | C-Planner hardening: planner unit on old default — 2.9 s | `git archive \| tar; planner_unit.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/rand40 S40 115b s4503 (16:47:44-16:49:58, 3) |
| 41 | 16:51:21-16:51:22 | 2 | main session | CPU perception test + governor load — 1.9 s | `test_navrl_latency_compensate.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/rand40 S00 160b s4501 (16:49:58-16:52:24, 1) |
| 42 | 17:24:55-17:24:58 | 3 | main session | camera-only C0 tests | `tests/test_navrl_sensor_ablation.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/rand40 S00 205b s4502 (17:24:19-17:27:07, 3) |
| 43 | 17:25:15-17:25:16 | 2 | main session | camera-only C0 tests — 1.6 s | `tests/test_navrl_sensor_ablation.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/rand40 S00 205b s4502 (17:24:19-17:27:07, 1) |
| 44 | 17:25:35-17:25:37 | 2 | main session | camera-only C0 tests — 1.7 s | `tests/test_navrl_sensor_ablation.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/rand40 S00 205b s4502 (17:24:19-17:27:07, 2) |
| 45 | 17:26:25-17:26:32 | 7 | main session | camera-only C0 tests + mutant known-bad controls — sequential | `test_navrl_sensor_ablation.py; mutants` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/rand40 S00 205b s4502 (17:24:19-17:27:07, 7) |
| 46 | 17:26:55-17:27:01 | 6 | main session | camera-only C0 tests + mutants — sequential | `test_navrl_sensor_ablation.py; mutants` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/rand40 S00 205b s4502 (17:24:19-17:27:07, 6) |
| 47 | 17:28:04-17:28:07 | 3 | main session | camera-only C0 tests incl. entry point — 2.6 s | `test_navrl_sensor_ablation.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/rand40 S20 205b s4502 (17:27:07-17:30:01, 3) |
| 48 | 17:28:15-17:32:31 | 255 | main session | PUBLIC_CPU suite (camonly) | `tools/run_tests.py` | 6,7,12,13 | 1 | 10 | heavy | queue notification + public_cpu_01.log mtime 17:32:31 | GR-1 eval/rand40 S20 205b s4502 (17:27:07-17:30:01, 106); GR-1 eval/rand40 S40 205b s4502 (17:30:01-17:32:55, 150) |
| 49 | 17:33:46-17:42:18 | 511 | main session | PUBLIC_CPU baseline then candidate suites (camonly) — base 17:33:48-17:38:02, candidate 17:38:02-17:42:17 | `tools/run_tests.py x2 sequential` | 6,7,12,13 | 1 | 10 | heavy | queue notification; base log 17:38:02, cand log 17:42:17 mtimes | GR-1 eval/rand40 S00 205b s4503 (17:32:55-17:35:42, 116); GR-1 eval/rand40 S20 205b s4503 (17:35:42-17:39:13, 211); GR-1 eval/rand40 S40 205b s4503 (17:39:13-17:42:05, 172); GR-1 eval/ctrl S00 70b s4501 (17:42:06-17:43:53, 12) |
| 50 | 17:43:03-17:43:05 | 2 | main session | camera-only quick checks — 2.2 s | `test_navrl_sensor_ablation.py (6,7,12,13); test_experiment_index, test_canonical_docs, check_public_docs (unpinned)` | 6,7,12,13 + unpinned | 1 | 0 | minor | tool_result | GR-1 eval/ctrl S00 70b s4501 (17:42:06-17:43:53, 2) |
| 51 | 17:43:42-17:48:02 | 260 | main session | camera-only C0 gate_01 — gate pins its children to 6,7,12,13 nice 10 | `camonly_c0_gate.py: tests, mutants, run_tests.py (sequential)` | 6,7,12,13 | 1 | 10 | heavy | queue notification + gate_01.out mtime 17:48:02 | GR-1 eval/ctrl S00 70b s4501 (17:42:06-17:43:53, 11); GR-1 eval/ctrl S20 70b s4501 (17:43:53-17:45:46, 113); GR-1 eval/ctrl S40 70b s4501 (17:45:46-17:47:46, 120); GR-1 eval/ctrl S00 70b s4502 (17:47:46-17:49:37, 16) |
| 52 | 17:59:35-17:59:38 | 3 | sub: camera-only C0 independent review r1 (Opus) | C0 review r1: association probe — 2.7 s | `probe_assoc.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/ctrl S00 115b s4501 (17:58:59-18:01:58, 3) |
| 53 | 18:00:22-18:00:26 | 4 | sub: camera-only C0 independent review r1 (Opus) | C0 review r1: PATH/ninja probe (torch import per PYTHON) | `probe_path.sh` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S00 115b s4501 (17:58:59-18:01:58, 4) |
| 54 | 18:02:03-18:02:12 | 9 | sub: camera-only C0 independent review r1 (Opus) | C0 review r1: C0 tests + mutants harness | `test_navrl_sensor_ablation.py; camonly_c0_mutants.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S20 115b s4501 (18:01:58-18:04:53, 9) |
| 55 | 18:09:36-18:09:39 | 3 | main session | camera-only C0 repair tests — 2.8 s | `tests (heredoc)` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/ctrl S20 115b s4502 (18:09:28-18:11:34, 3) |
| 56 | 18:09:56-18:10:00 | 3 | main session | camera-only C0 repair tests | `tests (heredoc)` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S20 115b s4502 (18:09:28-18:11:34, 4) |
| 57 | 18:10:14-18:10:27 | 13 | main session | camera-only C0 repair: two new mutants | `camonly_c0_mutants.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S20 115b s4502 (18:09:28-18:11:34, 13) |
| 58 | 18:11:46-18:16:14 | 267 | main session | camera-only C0 gate_02 | `camonly_c0_gate.py: tests, mutants, run_tests.py (sequential)` | 6,7,12,13 | 1 | 10 | heavy | queue notification + gate_02.out mtime 18:16:14 | GR-1 eval/ctrl S40 115b s4502 (18:11:34-18:13:42, 116); GR-1 eval/ctrl S00 115b s4503 (18:13:42-18:15:57, 135); GR-1 eval/ctrl S20 115b s4503 (18:15:57-18:18:01, 17) |
| 59 | 18:23:20-18:23:37 | 16 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: tests + mutant harness | `test_navrl_sensor_ablation.py; camonly_c0_mutants.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S20 160b s4501 (18:23:05-18:25:58, 17) |
| 60 | 18:24:25-18:24:32 | 6 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: B1 association probe | `probe_b1_assoc.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S20 160b s4501 (18:23:05-18:25:58, 7) |
| 61 | 18:25:37-18:25:41 | 4 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: M1 governor probe | `probe_m1_governor.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S20 160b s4501 (18:23:05-18:25:58, 4) |
| 62 | 18:26:09-18:26:13 | 5 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: M1 governor probe | `probe_m1_governor.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S40 160b s4501 (18:25:58-18:28:36, 4) |
| 63 | 18:26:27-18:26:31 | 4 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: M1 governor probe | `probe_m1_governor.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S40 160b s4501 (18:25:58-18:28:36, 4) |
| 64 | 18:27:02-18:27:04 | 2 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: M2 obs-only probe — 1.9 s | `probe_m2_obsonly.py` | 6,7,12,13 | 1 | 10 | minor | tool_result | GR-1 eval/ctrl S40 160b s4501 (18:25:58-18:28:36, 2) |
| 65 | 18:27:54-18:27:58 | 3 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: relative-PYTHON launcher failure demo | `bash play_navrl.sh (fails fast)` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S40 160b s4501 (18:25:58-18:28:36, 4) |
| 66 | 18:28:54-18:29:15 | 21 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: extra mutants vs C0 suite | `probe_extra_mutants.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S00 160b s4502 (18:28:36-18:31:06, 21) |
| 67 | 18:33:32-18:33:35 | 4 | sub: camera-only C0 independent review r2 (Opus) | C0 review r2: entry-point/refusal mutants | `probe_entry_mutants.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S40 160b s4502 (18:33:30-18:35:52, 3) |
| 68 | 18:39:03-18:39:22 | 19 | main session | camera-only C0 repair: tests + source mutants | `tests; mutants` | 6,7,12,13 | 1 | 10 | moderate | tool_result | GR-1 eval/ctrl S00 160b s4503 (18:35:52-18:39:15, 12); GR-1 eval/ctrl S20 160b s4503 (18:39:15-18:41:44, 7) |
| 69 | 20:36:05-20:36:11 | 5 | main session | camera-only sensor check, 20-frame trial | `sensor_check.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | C2 no_lidar 115b s4501 (20:33:46-20:36:37, 6) |
| 70 | 20:36:48-20:36:52 | 4 | main session | camera-only sensor check, 20-frame rerun | `sensor_check.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | C2 no_lidar_obs_only 115b s4501 (20:36:37-20:39:44, 4) |
| 71 | 20:37:15-20:37:46 | 32 | main session | camera-only sensor check, full — 31 s | `sensor_check.py` | 6,7,12,13 | 1 | 10 | moderate | tool_result | C2 no_lidar_obs_only 115b s4501 (20:36:37-20:39:44, 31) |

## CPU jobs checked that did not overlap any cell

| start-end | session | job | cores | procs | why no overlap |
|---|---|---|---|---|---|
| 11:54:24-11:54:28 | main session | record identity_01 + unittest + commit (gainrand) | unpinned | 1 | ran in a gap between GPU cells |
| 13:32:01-13:36:26 | main session | PUBLIC_CPU suite (gainrand) | 0-3,8-11 | 1 | ran in a gap between GPU cells |
| 13:32:54-13:32:57 | sub: GR-0 independent review (Opus) | GR-0 review cell validity audit | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 13:34:21-13:34:24 | sub: GR-0 independent review (Opus) | GR-0 review test file run | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 13:35:22-13:35:31 | sub: GR-0 independent review (Opus) | GR-0 review controller CPU probe | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 13:35:59-13:36:17 | sub: GR-0 independent review (Opus) | GR-0 review uniformity check | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 13:37:27-13:37:34 | main session | restore navrl_task.py + unittest (gainrand) | unpinned | 1 | ran in a gap between GPU cells |
| 13:37:30-13:37:34 | sub: GR-0 independent review (Opus) | GR-0 review recompute analysis | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 13:37:42-13:42:00 | main session | PUBLIC_CPU suite rerun (gainrand) | 0-3,8-11 | 1 | ran in a gap between GPU cells |
| 13:42:25-13:42:28 | main session | GR-1 prereg commit + unittest | unpinned | 1 | ran in a gap between GPU cells |
| 19:19:23-19:19:29 | sub: GR-1 results independent review (Opus) | GR-1 results review: cell validity probe | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 19:20:26-19:20:33 | sub: GR-1 results independent review (Opus) | GR-1 results review: recompute GR-1 quantities | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 19:21:23-19:21:26 | sub: GR-1 results independent review (Opus) | GR-1 results review: checkpoint inspection (torch CPU) | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 19:22:45-19:22:53 | sub: GR-1 results independent review (Opus) | GR-1 results review: find / -newermt receipt.json | unpinned | 1 | ran in a gap between GPU cells |
| 19:24:54-19:24:57 | sub: GR-1 results independent review (Opus) | GR-1 results review: runner smoke (fixed, known-bad) | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 19:25:17-19:25:21 | sub: GR-1 results independent review (Opus) | GR-1 results review: rerun committed analysis | 0-3,8-11,14-23 | 1 | ran in a gap between GPU cells |
| 19:25:51-19:26:05 | main session | camera-only C1/C2 runner + analysis smoke | 6,7,12,13 | 1 | ran in a gap between GPU cells |
| 19:26:44-19:26:47 | main session | camera-only stage + tests | unpinned | 1 | ran in a gap between GPU cells |
| 19:33:11-19:37:28 | main session | PUBLIC_CPU suite on C1/C2 commit (camonly) | 6,7,12,13 | 1 | ran in a gap between GPU cells |
| 19:38:46-19:38:56 | sub: camera-only C1/C2 design review (Opus) | C1/C2 design review: analysis probe | 6,7,12,13 | 1 | ran in a gap between GPU cells |
| 19:40:50-19:46:29 | sub: camera-only C1/C2 design review (Opus) | C1/C2 design review: git grep loop over 277 names | unpinned | 1 | ran in a gap between GPU cells |
| 19:50:12-19:50:28 | main session | camera-only C1/C2 smoke (extended) | 6,7,12,13 | 1 | ran in a gap between GPU cells |

## Per-cell load (cells with any overlapping job)

process-s = overlap seconds x processes. same = on the GPU job's CPUs (0-3,8-11,14-23 or a subset); disjoint = 6,7,12,13; unpinned = any online CPU.

| cell | wall s | jobs | process-s same | disjoint | unpinned | mean extra procs |
|---|---|---|---|---|---|---|
| GR-0 identity_02 off 160b s4501 11:55:18-11:55:59 | 41.26 | 2 | 6 | 0 | 0 | 0.146 |
| GR-0 main S00 70b s4501 11:58:04-11:59:53 | 109.34 | 2 | 190 | 0 | 2 | 1.761 |
| GR-0 main S20 70b s4501 11:59:53-12:01:42 | 109.11 | 1 | 218 | 0 | 0 | 2.0 |
| GR-0 main S40 70b s4501 12:01:42-12:03:35 | 112.84 | 1 | 226 | 0 | 0 | 2.0 |
| GR-0 main S00 70b s4502 12:03:35-12:05:25 | 109.76 | 1 | 186 | 0 | 0 | 1.691 |
| GR-0 main S40 115b s4501 12:19:33-12:21:50 | 137.09 | 2 | 12 | 0 | 0 | 0.088 |
| GR-0 main S00 115b s4502 12:21:50-12:24:47 | 176.86 | 1 | 354 | 0 | 0 | 2.0 |
| GR-0 main S20 115b s4502 12:24:47-12:26:57 | 129.23 | 1 | 260 | 0 | 0 | 2.0 |
| GR-0 main S40 115b s4502 12:26:57-12:29:25 | 148.75 | 1 | 296 | 0 | 0 | 2.0 |
| GR-0 main S00 115b s4503 12:29:25-12:31:31 | 125.28 | 1 | 252 | 0 | 0 | 2.0 |
| GR-0 main S20 115b s4503 12:31:31-12:34:35 | 184.65 | 1 | 368 | 0 | 0 | 2.0 |
| GR-0 main S40 115b s4503 12:34:35-12:36:43 | 127.33 | 1 | 148 | 0 | 0 | 1.156 |
| GR-0 main S00 160b s4501 12:36:43-12:39:23 | 160.23 | 2 | 51 | 0 | 0 | 0.319 |
| GR-0 main S20 160b s4501 12:39:23-12:41:59 | 156.32 | 2 | 262 | 0 | 0 | 1.679 |
| GR-0 main S40 160b s4501 12:41:59-12:44:32 | 153.33 | 3 | 151 | 0 | 0 | 0.987 |
| GR-0 main S00 160b s4502 12:44:32-12:47:07 | 154.45 | 1 | 155 | 0 | 0 | 1.0 |
| GR-0 main S20 160b s4502 12:47:07-12:49:27 | 139.76 | 1 | 140 | 0 | 0 | 1.0 |
| GR-0 main S40 160b s4502 12:49:27-12:52:10 | 163.83 | 1 | 163 | 0 | 0 | 1.0 |
| GR-0 main S00 160b s4503 12:52:10-12:54:34 | 143.67 | 1 | 144 | 0 | 0 | 1.0 |
| GR-0 main S20 160b s4503 12:54:34-12:57:05 | 150.59 | 1 | 151 | 0 | 0 | 1.0 |
| GR-0 main S40 160b s4503 12:57:05-13:00:11 | 186.4 | 1 | 186 | 0 | 0 | 1.0 |
| GR-0 main S00 205b s4501 13:00:11-13:03:01 | 170.33 | 1 | 164 | 0 | 0 | 0.965 |
| GR-0 main S20 205b s4501 13:03:01-13:05:55 | 173.68 | 1 | 264 | 0 | 0 | 1.517 |
| GR-0 main S40 205b s4501 13:05:55-13:08:32 | 156.67 | 2 | 0 | 0 | 5 | 0.032 |
| GR-1 train/rand40 train 205b s1 13:44:00-14:53:47 | 4186.42 | 17 | 4865 | 0 | 12 | 1.165 |
| GR-1 train/ctrl train 205b s1 14:55:03-15:59:37 | 3873.66 | 6 | 28 | 0 | 745 | 0.2 |
| GR-1 eval/rand40 S00 70b s4501 16:12:44-16:14:40 | 116.08 | 1 | 0 | 0 | 10 | 0.086 |
| GR-1 eval/rand40 S40 115b s4503 16:47:44-16:49:58 | 133.91 | 2 | 0 | 6 | 0 | 0.045 |
| GR-1 eval/rand40 S00 160b s4501 16:49:58-16:52:24 | 146.13 | 1 | 0 | 1 | 0 | 0.007 |
| GR-1 eval/rand40 S00 205b s4502 17:24:19-17:27:07 | 168.33 | 5 | 0 | 19 | 0 | 0.113 |
| GR-1 eval/rand40 S20 205b s4502 17:27:07-17:30:01 | 173.75 | 2 | 0 | 109 | 0 | 0.626 |
| GR-1 eval/rand40 S40 205b s4502 17:30:01-17:32:55 | 174.35 | 1 | 0 | 150 | 0 | 0.862 |
| GR-1 eval/rand40 S00 205b s4503 17:32:55-17:35:42 | 166.99 | 1 | 0 | 116 | 0 | 0.695 |
| GR-1 eval/rand40 S20 205b s4503 17:35:42-17:39:13 | 210.83 | 1 | 0 | 211 | 0 | 1.0 |
| GR-1 eval/rand40 S40 205b s4503 17:39:13-17:42:05 | 172.35 | 1 | 0 | 172 | 0 | 1.0 |
| GR-1 eval/ctrl S00 70b s4501 17:42:06-17:43:53 | 107.72 | 3 | 0 | 25 | 0 | 0.234 |
| GR-1 eval/ctrl S20 70b s4501 17:43:53-17:45:46 | 112.9 | 1 | 0 | 113 | 0 | 1.0 |
| GR-1 eval/ctrl S40 70b s4501 17:45:46-17:47:46 | 119.52 | 1 | 0 | 120 | 0 | 1.0 |
| GR-1 eval/ctrl S00 70b s4502 17:47:46-17:49:37 | 111.04 | 1 | 0 | 16 | 0 | 0.144 |
| GR-1 eval/ctrl S00 115b s4501 17:58:59-18:01:58 | 178.45 | 2 | 0 | 7 | 0 | 0.039 |
| GR-1 eval/ctrl S20 115b s4501 18:01:58-18:04:53 | 175.44 | 1 | 0 | 9 | 0 | 0.051 |
| GR-1 eval/ctrl S20 115b s4502 18:09:28-18:11:34 | 125.94 | 3 | 0 | 20 | 0 | 0.159 |
| GR-1 eval/ctrl S40 115b s4502 18:11:34-18:13:42 | 127.83 | 1 | 0 | 116 | 0 | 0.906 |
| GR-1 eval/ctrl S00 115b s4503 18:13:42-18:15:57 | 135.6 | 1 | 0 | 135 | 0 | 1.0 |
| GR-1 eval/ctrl S20 115b s4503 18:15:57-18:18:01 | 123.41 | 1 | 0 | 17 | 0 | 0.137 |
| GR-1 eval/ctrl S20 160b s4501 18:23:05-18:25:58 | 172.72 | 3 | 0 | 28 | 0 | 0.162 |
| GR-1 eval/ctrl S40 160b s4501 18:25:58-18:28:36 | 157.5 | 4 | 0 | 14 | 0 | 0.089 |
| GR-1 eval/ctrl S00 160b s4502 18:28:36-18:31:06 | 150.83 | 1 | 0 | 21 | 0 | 0.14 |
| GR-1 eval/ctrl S40 160b s4502 18:33:30-18:35:52 | 141.37 | 1 | 0 | 3 | 0 | 0.021 |
| GR-1 eval/ctrl S00 160b s4503 18:35:52-18:39:15 | 203.29 | 1 | 0 | 12 | 0 | 0.059 |
| GR-1 eval/ctrl S20 160b s4503 18:39:15-18:41:44 | 148.72 | 1 | 0 | 7 | 0 | 0.047 |
| CamOnly C2 no_lidar 115b s4501 20:33:46-20:36:37 | 171.55 | 1 | 0 | 6 | 0 | 0.035 |
| CamOnly C2 no_lidar_obs_only 115b s4501 20:36:37-20:39:44 | 186.47 | 2 | 0 | 35 | 0 | 0.187 |

## Not determined / caveats

- Per-process CPU utilisation and disk I/O were not logged; load is expressed as process count x time, with processes assumed busy for their whole lifetime (polling loops excluded).
- Worker counts inside drivers come from their source constants (D2-A1 WORKERS=2, D1-F WORKERS=6, the full-bound dev run's argument 6) and from launch commands; per-worker thread use (numpy/Warp) is unknown. Subagent probes set OMP/MKL/OPENBLAS threads to 1 in most D1-F review launches; others did not.
- End times of subagent background jobs come from notification attachments, which can lag the real exit by up to one tool call; they were cross-checked against output-file mtimes where those exist.
- git grep (the C1/C2 design-review loop) uses several threads by default; it ran between C1 and C1_r2, so it does not affect any cell.
- Load not launched by these sessions is not covered: desktop apps (Cursor, Xorg), the claude-mem worker and chroma-mcp, the Claude CLI itself, other users or cron jobs. A haiku-model claude-mem process and chroma-mcp were running when this was written.
- GR-1 training has no per-epoch timestamps here, so overlap with training is per arm, not per epoch.
- identity_01 cells (11:51:02-11:52:01) failed (no result.json); they are listed but no CPU job overlapped them.
- Light commands (under about 2 s, reads, edits, polls, git commits without hooks) were excluded by rule; 1.5-3 s script runs are listed as 'minor'.
- All 20 background jobs launched before the window have completion records; the last (C-Planner direction-bound repair gate) ended 11:48:55, before the first GPU cell (11:51:02). No detached process from earlier in the day was found running into the window.
- Cell start/end are truncated to whole seconds; overlaps under about 1 s at cell boundaries (for example the 1.7 s V1-v2 check at 11:55:18 against identity_02 ref/off) are uncertain.
- The twin comparison is a hash of episodes.jsonl and kroc2027_episodes.jsonl per cell (read-only, one nice'd core). result.json files are never byte-identical across runs because they carry run metadata.
