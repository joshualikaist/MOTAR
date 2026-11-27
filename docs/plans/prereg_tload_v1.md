# Preregistration: T-LOAD — does concurrent CPU work change a GPU evaluation? (v1, 2026-11-05)

Branch `camonly/dev`. Runner `tools/tload_run.py`. Decision D-2026-11-05-1 (AGENTS.md "Concurrent work") makes its
reading the condition for CPU work during evaluations.

**User approval (2026-11-05):**
- "동시 실행 정의 난 괜찮아. 반영해도 돼." — the definition is accepted.
- "이거 하고 gpu 세션 바로 시작하자." — after the rule edits, start the GPU session. The deferred queue
  (Beads `motar-g9a`) puts T-LOAD first.

## 1. Question and claim

- **Question:** can concurrent CPU or disk load from another process change the per-episode outcome files of a
  MOTAR evaluation cell?
- **Background:**
  - Code reading found no clock-dependent logic in the evaluation path. The two `time.perf_counter()` sites are
    timing telemetry.
  - It cannot exclude thread-timing effects, such as PhysX's CPU workers (`num_threads = 10`) or torch and numpy
    CPU ops.
  - One loaded evaluation differed in 2026-09 (A8 `S_off`); three loaded cells on 2026-11-04 did not
    (`gainrand/dev` `352b919`).
- **What would make "load cannot change results" false:** any loaded run whose `episodes.jsonl` or
  `kroc2027_episodes.jsonl` differs from the idle run of the same cell, commit and launcher.

## 2. Design

- **Cells:** `tools/camonly_run.run_cell`, unchanged. This is the camera-only launcher copy
  (SHA `1eadf8fb…`, pinned), the ablation entry point, GR-0's grid condition (E2, A3, quota 16, 128 envs) and
  Policy F `f7022139…`.

  | Cell | Condition | Bars | Seed |
  | --- | --- | ---: | ---: |
  | 1 | `full` | 205 | 4501 |
  | 2 | `no_lidar` | 205 | 4501 |
  | 3 | `full` | 70 | 4501 |

- **Arms**, in this order within each cell (cells in the order above; 15 runs):

  | Arm | Load | Cores |
  | --- | --- | --- |
  | L0a | none | — |
  | L1 | `stress-ng --cpu 4 --hdd 1 --hdd-bytes 1G` (CPU and disk; temp files under the load scratch) | 6,7,12,13 |
  | L2 | `stress-ng --cpu 18` (the worst case: the runner's own cores, nice 0) | 0-3,8-11,14-23 |
  | L3 | the camera-only sensor check (`docs/evidence/camonly_2026-11-04/sensor/sensor_check.py`, a copy in the load scratch), looped, `nice -n 10`: the real job behind review M-1 | 6,7,12,13 |
  | L0b | none | — |

- **Load hygiene:**
  - Every load runs with `CUDA_VISIBLE_DEVICES=""` and an address-space cap: 4 GB for stress-ng, 24 GB for L3.
  - Each load has its own process group. It starts 5 s before the cell and stops after it.
  - Loads write only under `--load-scratch`, outside every repository.
- **Runner:** started under `taskset -c 0-3,8-11,14-23` (preflight checks the affinity), with `timeout -k`.
- **Host:** the journal's machine-check and segfault count at start and after every cell. A rise voids the run
  (`gr_run.run_cell`).

## 3. Manipulation check (the load must be present)

For each load run:
- the load process was alive before the cell started and when it ended;
- its process group's CPU time divided by its wall time is at least the floor below.

| Arm | Floor (cores) |
| --- | ---: |
| L1 | 3.0 |
| L2 | 12.0 |
| L3 | 0.8 |

A load arm that fails this in any cell has no reading (LOAD_NOT_APPLIED). Cell wall times are reported for every run.

## 4. References

The historical references are 2026-11-04 runs of the same cells, with these SHA-256 values for `episodes.jsonl` and
`kroc2027_episodes.jsonl`:

| Cell | Reference runs | Role | `episodes.jsonl` | `kroc2027_episodes.jsonl` |
| --- | --- | --- | --- | --- |
| `full` 205/4501 | C1 `full` and C1_r2 `full` (both idle) | gate | `8899856375b19c6d1865e0f807827e479077325045bf82f1c44ab3113f77e6ad` | `3eb439456a0e7055994705db994523bdc0ead956effe8c2242ad35cfb99c0777` |
| `full` 205/4501 | GR-0 S00 (ran under a one-process same-core job) | reported | (same) | (same) |
| `no_lidar` 205/4501 | C1, C1_r2 and C2 `no_lidar` (all idle) | gate | `4a161f9eb0f0792f14dc3de6e1432a5786a099c661fc461bf69d8f4fd0119a3b` | `adc872d7015cded00f7017836ca31942c556c0138f981d6716b8f40b07841dd0` |
| `full` 70/4501 | GR-0 S00 (ran under a heavy same-core job, 95 s of overlap) | reported | `11e7ec818586df7b6f6859c2dacdc0a167882650e3e8e5a4e22f5dc316389543` | `9f96ecf1edbbc6bfdf03264a587189c6331834429797b3d03fe7ab0fabb37d73` |

## 5. Readout and reading (fixed before execution)

**Equality** of two runs means all three hold:
- `episodes.jsonl` is byte-identical;
- `kroc2027_episodes.jsonl` is byte-identical;
- `gr_run.identity_compare` reports `result_equal`, which compares `result.json` without the run nonce and the
  checkpoint path.

**Reading**, in this order of precedence:

| Reading | Condition |
| --- | --- |
| VOID_HOST_FAULT / INCOMPLETE | a run is not OK |
| REFERENCE_MISMATCH | L0a of a 205-bar cell is not equal to every gate reference: this commit does not reproduce the recorded idle runs, so no load reading |
| NONDETERMINISTIC_IDLE | L0b ≠ L0a in some cell: the pipeline is not deterministic even idle |
| LOAD_NOT_APPLIED | some load arm failed its manipulation check |
| LOAD_SENSITIVE | L1 or L3 ≠ L0a in some cell |
| DISJOINT_ONLY | only L2 ≠ L0a |
| LOAD_INVARIANT | every arm = L0a in every cell |

**Reported, not gated:**
- whether L0a equals the GR-0 references, which ran under load on 2026-11-04;
- the cell wall times;
- the load CPU.

**Consequences (D-2026-11-05-1):**

| Reading | Consequence |
| --- | --- |
| LOAD_INVARIANT | CPU compute during evaluations is allowed under AGENTS.md's conditions |
| DISJOINT_ONLY | allowed on the disjoint cores only |
| LOAD_SENSITIVE or NONDETERMINISTIC_IDLE | it stays forbidden; the 50 overlapped cells of 2026-11-04 must be rerun idle (queue G2) |

## 6. Limits fixed in advance

- **Scope:** three cells and one seed, Policy F, this host (i7-13700K with cpu4/5 offline, RTX 3070), this commit.
  A pass shows invariance for these loads on these cells. It is not a proof for every cell.
- **Power:** "load changes results" has no known-bad control, so power comes from the worst-case arm L2 and the
  manipulation check.
  - L2 is heavier than any load recorded on 2026-09-05 or 2026-11-04.
  - The rare-event concern applies: a timing effect that changes 1 run in 50 can pass 15 runs. Section 4 and the
    2026-11-04 twins add history, not certainty.
- **Disk:** only L1's 1 GB stress-ng file load tests the disk.

## 7. Cost

About 15 runs at 2–5 min each, so about 40–60 min of GPU. Budget cap 2 h, outer `timeout -k` 2.5 h.
