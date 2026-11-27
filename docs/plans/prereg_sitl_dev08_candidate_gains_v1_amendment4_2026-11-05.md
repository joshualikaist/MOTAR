# Dev08 amendment4 — runner compute threads and the timing basis for skip_max_s, 2026-11-05

Parents: [`amendment2`](prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md) and
[`amendment3`](prereg_sitl_dev08_candidate_gains_v1_amendment3_2026-11-05.md) (both immutable). This file changes
one runtime setting and records the measurements behind the open skip_max_s decision. It changes nothing of
amendment1's estimand, cohort, order or analysis. **No execution is authorized.** No PX4, Gazebo, SITL or GPU
process was started.

## 1. Why: the skip/late bound needed a measured basis

The user asked for the bound to be chosen from the numbers, not as a midpoint. An independent analysis
(Claude Fable 5.1; `docs/evidence/sitl_2026-11-05/dev08/runtime_stack/ANALYSIS_skip_bound_fable.md`) of the 23
dev07 step logs found the following.
- A rejection at bound τ happens only when the runner's compute exceeds about 100 + 1000·τ ms: 141 ms at 0.05 s,
  241 ms at 0.15 s. The PX4 stream side was clean.
- dev07 ran alongside a GPU evaluation, so its spike rate (about 1 checkpoint in 260 above 60 ms) may not carry
  over.
- The analysis recommended 0.15 s with stated limits. It also proposed a timing-only CPU measurement under a
  rule fixed beforehand (`/home/fair/motar_tmp/dev08_skip_bound_20261105/timing_benchmark_plan.md`):
  - maximum compute below 100 ms: 0.10 s suffices;
  - any compute above 150 ms: fix the runner first and do not widen the bound;
  - otherwise keep 0.15 s.

## 2. Measurements (timing only; no simulator, no experiment outcome)

All runs used the real bridge and Policy F on the CPU, the twenty fixed scenes, the warm-up, 10 Hz pacing, an
inbox producer thread and a /proc snapshot thread, on cores 6,7,12,13 at nice 10. Other work ran on the E-cores
16–23 at the same time. It shares the L3 cache, but not the hyperthreads.

| Run | Threads | n | median | p99 | max | > 60 ms | > 150 ms |
| --- | --- | --- | --- | --- | --- | --- | --- |
| planned benchmark (`timing_benchmark.json`) | 4 (amendment2) | 5000 | 5.64 ms | 87.3 ms | 351.8 ms | 117 | 11 |
| GC attribution, 205-bar scenes first (`timing_diag_gc.json`) | 4 | 3000 | 5.76 ms | 93.5 ms | 264.5 ms | 108 | 7 |
| A/B (`timing_diag_threads_omp4.json`) | 4 | 1500 | 5.78 ms | 92.0 ms | 160.4 ms | 42 | 1 |
| A/B (`timing_diag_threads_omp1.json`) | 1 | 1500 | 4.62 ms | 8.2 ms | 11.8 ms | 0 | 0 |
| planned benchmark after the fix (`timing_benchmark_threads1.json`) | 1 | 5000 | 4.51 ms | 8.0 ms | 13.8 ms | 0 | 0 |

The rule's verdict on the planned benchmark was "fix the runner first". The attribution runs showed two things.
- Hypothesis "Python garbage collection causes the spikes" rejected: none of 108 spikes contained a collection
  of 20 ms or more.
- Hypothesis "the spikes grow over the run" rejected: with the scene order reversed they stayed spread out.
- The spikes came from four compute threads (OMP/MKL 4, as amendment2 proposed) on two physical cores, competing
  with the runner's own Python threads. With one thread they disappeared.

## 3. Change: one compute thread

The unit runs with `OMP_NUM_THREADS=1` and `MKL_NUM_THREADS=1`, replacing amendment2's "OMP/MKL threads 4".
- `tools/sitl_dev08_attempt.environment_problems` refuses any other value.
- `execute()` calls `torch.set_num_threads(1)` after loading the bridge and refuses if torch reports another count.

## 4. The skip_max_s decision after the fix

The planned rule applied to the measurement after the fix gives **0.10 s suffices**: maximum compute 13.8 ms, below 100 ms, with none above 60 ms in 5000 iterations. Under the analysis's mechanism (rejection when compute exceeds about 100 + 1000·τ ms), a 0.10 s bound rejects only a compute above about 191 ms; the measured maximum is 14 times below that. The decision stays the user's and is
fixed in the approval record (`skip_max_s`) before the smoke. The measurement bounds runner-side compute only. The
smoke must still confirm the PX4 stream timing and the receiver thread (amendment2 live assumptions 4, 6 and 7).

## 5. Record notes (review r6 m6-c, n6-h)

- Review r6 found that two external files bound by `validation_amendment3.json` had been overwritten after that
  commit: `/home/fair/motar_tmp/dev08_runtime_stack_20261105/step_mutants/run_mutants.py` and `results.json`. They
  were overwritten at 20:29–20:30 by a later mutant run that added the M19, M21, M27 and M28 cases.
  - Their bound hashes no longer verify. The original contents are not recoverable.
  - r5 had verified the binding when it was made.
  - The new contents are bound in `validation_amendment4.json`.
  - From now on, new runs write to new directories.
- The evidence README's "the following commit" for the r5 fix is corrected: the fix and amendment3 are in the
  same commit, `c4974f4`.
