# dev08 `skip_max_s` — independent timing-tolerance analysis

Author: independent analyst, Claude Fable 5.1 (Agent model fable). Date: 2026-11-05. Read-only on all inputs; no
simulator, GPU or git action. Time-box about 30 min.

Scripts and outputs (all in this directory):
- `analyse_skip_bound.py` -> `analyse_skip_bound.out`, `analysis_out.json` (data summary, observed-exceedance
  Bayes/Clopper-Pearson tables, single-exponential tail model, control cost).
- `spike_mixture.py` -> `spike_mixture.out`, `spike_mixture_out.json` (bulk + spike tail model, spike-rate
  sensitivity, location of every lateness > 30 ms).

Inputs read: 23 dev07 step logs (`/home/fair/motar_tmp/sitl_s3b_20261105/dev07_*/ep*_steps.npy`), the prior
recomputation (`dev08_runtime_stack_20261105/dev07_lateness.{py,out}`, `dev07_records.json`), the two s3b
reports, `g7/analysis.json` + `prereg_actionhold_g7_v1.md`, `tools/sitl_dev08_link.py` (TelemetryGate.checkpoint),
amendment2 section "Decision for the approval" and amendment3, and `run_dev07.sh`.

## 0. What the gate actually tests (from the code)

At checkpoint k the new gate takes the **first** position sample with `t >= t_k` (CLOCK_SKIP if it is more than
`skip_max_s` after t_k: a stream gap) and then requires the **newest** sample to be within `skip_max_s` of t_k
(RUNNER_LATE: the runner arrived late). The historical runner used the newest sample, so the historical
"lateness" column is exactly the RUNNER_LATE quantity; the ordinary 0-24 ms residue bounds the CLOCK_SKIP quantity
from above. One number serves both checks. A failure aborts the 42-member unit (amendment1; never a censor).

Mechanism check ("mechanism pairs" in `analyse_skip_bound.out`): with the old runner,
`L[k+1] ~ L[k] + C[k] + o - 0.100 s` whenever positive (C = compute at k, o ~ 5 ms overhead), else the ordinary
0-24 ms residue. Predicted vs observed: 218.4 ms compute -> 0.138 vs 0.136 s; 128.5 ms -> 0.048 vs 0.040 s;
122.7 ms -> 0.027 vs 0.024 s. So **a rejection at tolerance tau is a compute spike larger than about
100 + 1000*tau - L_k - 5 ms**, i.e. ~141 ms at tau = 0.05, 191 ms at 0.10, 241 ms at 0.15, 291 ms at 0.20.
The lateness tail is the compute tail shifted by 100 ms.

## 1. Data summary

| Quantity | Value |
| --- | --- |
| Episodes / checkpoints | 23 / 1318 (20 main: mean 43.6 steps, range 18-75; smokes 55, 35, 356) |
| Real-time factor (reports) | 0.999-1.002 |
| Lateness, all 1318 | p50 4 ms, p90 20, p95 24, p99 24, p99.9 39; max 136 ms |
| Lateness value counts (ms) | 0: 496, 4: 413, 8: 27, 12: 54, 16: 61, 20: 184, 24: 80, 36: 1, 40: 1, 136: 1 |
| Ordinary lateness (no spike carry-over) | 1315 checkpoints, all in {0, 4, ..., 24} ms on the 4 ms grid; max 24 ms |
| Non-ordinary lateness (3 of 1318) | 136 ms (dev07_70/ep02 k28, after 218.4 ms compute at k27); 36 ms (same ep, k29, the reused sample); 40 ms (dev07_70/ep05 k4, after 128.5 ms compute at k3) |
| Compute ms, k >= 1 (n = 1295) | p50 6.2, p90 10.3, p95 16.5, p99 31.0, p99.9 127; max 218.4 |
| Compute exceedances, k >= 1 | > 20 ms: 42; > 40: 8; > 60: 5 (78.7, 95.0, 122.7, 128.5, 218.4); > 150: 1 |
| Compute at k = 0 (first call) | 48-60 ms in 5 of 23 episodes, 6-8 ms otherwise (the new warm-up removes this) |
| Exceedances of lateness over tau | tau = 0.03: 3; 0.04: 2; 0.05-0.136: 1; >= 0.14: 0 |

Tail shape: the compute distribution is a narrow bulk (median 6 ms, 97 % under 20 ms) plus a thin bulk tail
(20-45 ms, 37 points) plus **five isolated spikes of 79-218 ms** at a rate of 3.9e-3 per checkpoint
(Clopper-Pearson 95 % 1.3e-3 to 9.0e-3), about one per 260 checkpoints or one per 6 episodes. Only the largest
spike (218 ms) produced a lateness above 0.05 s. A single exponential fitted over 20 ms (mean excess 18 ms)
under-weights the spikes; a GPD fit returns shape 1.06 (no finite mean) and is not usable. The honest tail
description is the two-component one; its spike-size law rests on five points, its exceedance over 0.05 s on
one point.

Conditions of the historical data (from `run_dev07.sh`): the dev07 runs were a development run **alongside the
GR-1b GPU evaluation**, on disjoint cores (`taskset -c 6,7,12,13`, nice 10), without warm-up and without the idle
check. The new unit requires an idle host (no GPU run, cores >= 90 % idle) and warms up. Whether those spikes were
contention (memory bandwidth, L3, interrupts shared with the GPU job and with PX4/Gazebo) or runner-internal
(allocator, GC, thread wake-up) cannot be told from these logs; the new conditions plausibly reduce but do not
bound the spike rate.

## 2. Abort probability per tolerance

Definitions. p = per-checkpoint rejection probability. Episode rejection for a 44-checkpoint episode
= 1 - (1 - p)^44. Unit completion for N = 42 x 44 = 1848 checkpoints ("typ"); N = 3360 for 80-step episodes
("hi"). Independence of checkpoints and episodes is assumed throughout (a loaded host would cluster spikes and
make things worse than independent; the idle check is the only guard against that).

Three estimators, from least to most model-dependent:
- **A, observed exceedance**: x exceedances of lateness over tau in n = 1318; Jeffreys posterior
  Beta(x + 1/2, n - x + 1/2); unit completion = E[(1 - p)^N]; Clopper-Pearson 95 % on p propagated to the unit.
- **B, single-exponential compute tail** over 20 ms (optimistic: it treats the spikes as the tail of the bulk).
- **C, spike-mixture**: spike rate lambda = 5/1295 with its CP95, spike size exponential above 60 ms (mean excess
  68.6 ms), rejection when the spike exceeds the threshold of section 0 (pessimistic if the idle host removes
  spikes; the historical rate is applied unchanged).

| tau (s) | x/1318 | p CP95 | ep. reject (A, point) | unit OK: A Bayes (typ / hi) | A CP95 (typ) | B exp (typ) | C mixture (typ) [lambda CP95] | C if spike rate halved / divided by 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.03 | 3 | 4.7e-4 to 6.6e-3 | 9.5 % | 0.05 / 0.01 | 0.00-0.42 | 0.72 | 0.05 [0.00-0.39] | 0.23 / 0.75 |
| 0.05 | 1 | 1.9e-5 to 4.2e-3 | 3.3 % | 0.27 / 0.15 | 0.00-0.97 | 0.90 | 0.11 [0.01-0.49] | 0.33 / 0.80 |
| 0.08 | 1 | same | 3.3 % | 0.27 / 0.15 | 0.00-0.97 | 0.98 | 0.24 [0.04-0.63] | 0.49 / 0.87 |
| 0.10 | 1 | same | 3.3 % | 0.27 / 0.15 | 0.00-0.97 | 0.99 | 0.35 [0.09-0.71] | 0.59 / 0.90 |
| 0.12 | 1 | same | 3.3 % | 0.27 / 0.15 | 0.00-0.97 | 1.00 | 0.45 [0.16-0.77] | 0.67 / 0.92 |
| 0.15 | 0 | 0 to 2.8e-3 | 0 % | 0.65 / 0.53 | 0.01-1.00 | 1.00 | 0.60 [0.30-0.85] | 0.78 / 0.95 |
| 0.20 | 0 | same | 0 % | 0.65 / 0.53 | 0.01-1.00 | 1.00 | 0.78 [0.56-0.92] | 0.88 / 0.98 |
| 0.30 | 0 | same | 0 % | 0.65 / 0.53 | 0.01-1.00 | 1.00 | 0.94 [0.88-0.98] | 0.97 / 0.99 |
| 0.50 | 0 | same | 0 % | 0.65 / 0.53 | 0.01-1.00 | 1.00 | 1.00 | 1.00 / 1.00 |

Episode-level view (23 episodes, no model): 1/23 rejected for tau in [0.05, 0.136) -> CP95 for the per-episode
rate 0.1-22 %, unit completion Bayes 0.21, CP95 0.00-0.96; 0/23 for tau >= 0.14 -> rate CP95 0-14.8 %, unit
completion Bayes 0.60, CP95 0.001-1.00. (Amendment2's "1 in 20 -> 0.88 abort" is the plug-in point estimate of
this same row.)

Reading the table honestly:
- The observed data distinguish only three regions: tau < 0.04 (2-3 events), 0.04 <= tau <= 0.136 (one event,
  the 218 ms spike) and tau > 0.136 (none). Within each region the data say nothing about the choice; only a tail
  model does, and the tail model rests on five spikes (sizes) and one exceedance.
- For the code default 0.05 s the central estimates of unit completion are 0.11-0.27 (0.90 only under the
  optimistic model B). The one historical unit of this size would have aborted.
- No value up to 0.30 s reaches 0.9 completion under the historical spike rate (C); 0.15 reaches 0.9 only if
  the new conditions cut the spike rate by about ten, 0.20 if they cut it by about three (spike rate needed for
  0.9 at tau = 0.15 is 8e-4 per checkpoint, about one spike per 1250 checkpoints, against 1 per 260 observed).
- The stream side is well bounded: zero of 1318 ordinary residues above 24 ms (CP95 upper bound on the rate of
  a > 24 ms stream gap: 2.8e-3), so CLOCK_SKIP at any tau >= 0.05 is not the risk; RUNNER_LATE is.

## 3. What a late checkpoint costs the control

For lateness L the command is computed from the state at ~t_k and applied at t_k + L; meanwhile command k - 1
keeps acting. The vehicle travels v*L under the stale command (the drone does not go uncontrolled).

| tau (s) | travel at 2.5 m/s | at 3.6 m/s | fraction of the 0.1 s step | whole checkpoints effectively missed |
| --- | --- | --- | --- | --- |
| 0.03 | 0.075 m | 0.11 m | 0.3 | 0 |
| 0.05 | 0.125 m | 0.18 m | 0.5 | 0 |
| 0.08 | 0.20 m | 0.29 m | 0.8 | 0 |
| 0.10 | 0.25 m | 0.36 m | 1.0 | 1 |
| 0.12 | 0.30 m | 0.43 m | 1.2 | 1 |
| 0.15 | 0.375 m | 0.54 m | 1.5 | 1 |
| 0.20 | 0.50 m | 0.72 m | 2.0 | 2 |
| 0.30 | 0.75 m | 1.08 m | 3.0 | 3 |

Delay study G7 (training simulator, policy F, seed 4501, densities 70 and 205, 2048 episodes per cell):
- d = 1 (every command delayed one full step, 100 ms, sustained): crash +1.5 pp, CI95 +0.5 to +2.5, read
  **TOLERANT** (criterion delta <= +2 and CI upper <= +4).
- k = 2 (each action held two steps; the action is 0-100 ms stale, no fresh observation every other step):
  +2.0 pp, CI +1.0 to +3.0, **INCONCLUSIVE** (just over the TOLERANT line).
- k = 4 (held 0-300 ms): +10.5 pp, CI +9.2 to +11.7, **HARMFUL** (+17.4 pp at 205 bars).
- Stated limits: one seed; d = 1 is the only latency tested; hold is harsher than a trained chunk policy; all in
  the training simulator, not SITL.

Relation to a single late checkpoint:
- L <= 0.10 s: one command once delayed by at most what G7 applied to every command, with a measured sustained
  cost of +1.5 pp. A single event is far below that. **Minor by evidence.**
- 0.10 < L < 0.20 s: one checkpoint is effectively lost (command k - 1 runs 1-2 steps, command k acts briefly,
  k + 1 is 0-0.1 s late and the loop is back on schedule by k + 2). One-off equivalent of the k = 2 hold, whose
  sustained cost was +2.0 pp (INCONCLUSIVE). No delay > 100 ms was tested, so this is **plausibly minor, not
  evidenced**. Travel under the stale command 0.25-0.72 m, comparable to the episodes' recorded minimum obstacle
  distances (0.30-0.48 m in the dev07 reports).
- L >= 0.20 s: two or more checkpoints lost, entering the regime of the k = 4 hold (HARMFUL when sustained), with
  0.5-1.1 m travelled on a 0.2-0.3 s stale command. **No longer a minor timing deviation.** 0.20 s is the
  defensible ceiling; 0.30 and 0.50 should not be approved as "timing tolerances".

## 4. Recommendation

**Recommend `skip_max_s = 0.15 s`** as the single fixed value, with its limits stated below. Reasons:
1. It admits the only spike-class event seen in 1318 checkpoints (0.136 s) with 14 ms to spare; every tau <= 0.136
   would have aborted the historical unit on that one event, and the data cannot distinguish 0.05 from 0.12 at all
   (same single exceedance). The code default 0.05 s carries a central unit-completion estimate of 0.11-0.27.
2. It stays inside the "one checkpoint lost" regime (1.5 steps; 0.375-0.54 m of travel on a stale command), one
   step below the hold-4 regime that G7 found HARMFUL. 0.20 s buys a modest completion gain (0.60 -> 0.78 under
   the pessimistic model, nothing under the optimistic one) at the price of admitting two-step losses.
3. The stream side (CLOCK_SKIP) is unaffected by the choice: ordinary lateness never exceeded 24 ms.

Limits of this recommendation (state them in the approval record):
- Central unit-completion estimates at 0.15 s: 0.60 (historical spike rate, model C), 0.65 (observed exceedances,
  Jeffreys), ~1.0 (single-exponential tail); 90 % model-bootstrap / lambda-CP95 range 0.30-1.00. **The data do not
  support a claim of >= 0.9 completion at any value <= 0.20 s.** The dominant unknown is the runner's compute-spike
  rate under the new conditions (warm-up, idle host, no concurrent GPU job), which these logs cannot measure
  because all 23 episodes ran next to a GPU evaluation.
- All estimates assume independent spikes; clustering (a busy host) makes them worse. The per-member idle check is
  the guard.
- The spike-size law (exponential, mean excess 69 ms) is fitted to five points; the exceedance over 0.05 s to one.

Minimal additional evidence that would sharpen the choice without touching the experiment's outcomes (timing-only,
CPU-only, about 10-15 min, no simulator needed for the dominant mechanism): run the dev08 runner's per-checkpoint
compute path (observe -> policy -> governor -> serialize) after its warm-up, pinned to cores 6,7,12,13 on the idle
host with the same thread settings, paced at 10 Hz against recorded observations, for >= 5000 iterations (~2.7x
the unit), recording per-iteration wall time; pre-register the rule: max < 100 ms -> the spike mechanism is absent
under the new conditions and 0.10 s (one step, evidence-backed by G7 d = 1) is sufficient; any iteration > 150 ms
-> there is a runner defect to find (allocator, thread pool, GC) rather than a bound to widen; otherwise keep 0.15 s.
Caveat: it does not exercise the MAVLink receiver thread; a receiver stall shows up as CLOCK_SKIP/RUNNER_LATE too,
but the 0.136 s event was in the compute column, so the compute replay targets the observed mechanism. If the
schedule allows a smoke-class SITL timing run instead (no outcomes recorded, gate in record-only mode, ~1800
checkpoints), it measures both.

Rule shape (would need its own amendment; noted, not proposed for this approval): the two checks guard different
failures and have different data. CLOCK_SKIP (stream gap) is tightly bounded by 1318 residues <= 24 ms and could
keep 0.05 s; RUNNER_LATE (runner behind) is the spike risk and is where the tolerance matters. A per-checkpoint
RUNNER_LATE tolerance plus a per-episode cap (for example: lateness <= 0.15 s always; at most one checkpoint per
episode above 0.05 s, every such event recorded with k, L and compute) would turn a rare single spike from a
unit abort into a recorded, bounded deviation while still stopping an episode whose runner is repeatedly behind.
Since lateness is a harness event independent of the arm, both arms would still share it. Under amendment1's
"never a censor" rule this is not available now; the single-number rule with 0.15 s is the best fixed choice
within the present protocol.
