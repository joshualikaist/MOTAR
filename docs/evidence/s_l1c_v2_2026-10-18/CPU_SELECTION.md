# S-L1C v2 independent CPU selection

Plan `51a9015`; oracle/comparison source `396ca61`. Raw:
`../aerial_gym_simulator/results/s_l1c_v2_cpu/SL1CV2-CPU01`.
The [summary](cpu_selection_summary.json), [distance/elevation table](cpu_distance_elevation.csv)
and [raw manifest copy](CPU_RAW_MANIFEST.json) bind the complete ledger and unabridged strata.

All 105,408 preregistered matched cases and 1,152 angular-error diagnostics completed, 27.2s CPU.
These are deterministic constructed cases, not independent population samples.

| Candidate | New correct primary acceptances | False accepts, all matrix noise/prediction cases | Selection |
| --- | ---: | ---: | --- |
| V2-0 | 0 | 135 | Frozen baseline |
| V2-A | 3,318 | 135 | **CPU_GO — selected** |
| V2-B | 0 | 135 | NO-GO: no primary recall improvement |
| V2-C | 3,426 | 18,054 | NO-GO: false association/range gates fail; clock contract ineligible |

The primary near-range clear-sphere recall at zero noise is 2,860/4,383 (65.252%) for v1 and
3,966/4,383 (90.486%) for A. At ±0.1mm it is 5,720/8,766 versus 7,932/8,766, respectively.
No paired true acceptance is lost; no matched case adds a false acceptance. The inherited 135
false accepts remain false and are not relabelled. The seven original EQ-1D negatives stay zero
at primary bounds. Newly correct primary associations have mean centre error 3.213e-6m.

Noise is a material limit: at ±1cm near-range recall is 65.617% versus 65.880%; at ±3cm both are
68.560%, because the hypothesis advantage is insufficient and no recovery is added. Existing v1
decisions themselves depend on perturbed measurements; these rates do not establish noise realism.
The full abstention/error/diagnostic counts remain in the machine-readable records.

This is candidate selection only. No runtime, GPU, covariance/clock, old-mode regression or E-L0
qualification is implied. Next implement A under a separate `s_l1c_v2` flag, retaining A/B and
v1 clocks/covariance, while keeping S-L1D disabled on that new lineage.
