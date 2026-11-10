# Independent CAM-K2-E3 results review r1 — 2026-11-10

**Verdict: PASS for the integrity, fixed analysis and scoped reporting of this completed simulation unit.** This is not a positive reduction-gate verdict. Gate D remains **NO_DETECTED_CHANGE** for both seed pairs; R and T each remain **SEEDS_DISAGREE**.

Reviewer: fresh-context Codex worker `/root/e3_result_review`, independent of the execution/result-record author. Model identified in inherited task context: **gpt-6.1-sol**; a backend build identifier is not exposed and is not asserted. Reviewed worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, execution/base HEAD `aaca40e8dc6a3492cedd0b692427cbc796661015`. Only this external audit directory was written. No GPU/simulation execution, source edit, raw edit, checkpoint load into a model, commit, push or merge was performed by the reviewer.

## Findings by severity

- **Blocking / high / medium: none found.** The direct raw-ledger recomputation reproduces all primary numbers and fixed labels.
- **LOW-1, RESOLVED, documentation command:** the first reviewed result record's reproduction command omitted a time limit, address-space cap, CPU affinity, priority and `OPENBLAS_NUM_THREADS`. The final command supplies all five; this repair was verified. This did not affect the completed measurements or their analysis. The final prose verification and its exact SHA-256 are recorded in `prose_review.json`.
- **Audit implementation correction, not an experiment finding:** the first probe searched committed source preimages only among `.py`, `.sh` and `.urdf` names, omitting extensionless `tools/realveh_python`. Its one failure was the audit's incomplete file-discovery filter. The wrapper was already present, SHA-pinned and byte-correct. The filter was corrected, and the independent probe was run again. Exact first outputs remain `probe_r0.log` and `recomputed_probe_r0.json`; execution evidence was untouched.

## Independent method and evidence

`probe.py` uses Python's standard library and imports no project analysis or runner modules. It reads `episodes.jsonl` directly, reconstructs quota membership from `env_episode_index < 16`, verifies exactly 16 quota outcomes for each of 128 environments, contiguous per-environment episode indices, unique episode ids and exclusive outcome flags, and rebuilds quota and legacy counts. Each new cell contributes 2,048 quota episodes; the 53 new cells contribute 108,544. Legacy pooled-export totals are kept separate.

The script independently implements the preregistered density-stratified bootstrap (10,000 draws, `random.Random(20261108)`, fixed metric/contrast stream order, percentile order statistics) and stratified t calculation (8 df, preregistered t constant `2.306004`; 4 df for the six-block inherited controls). It reconstructs all eight contrasts for crash, capture and timeout, their block values and both intervals, fixed gate labels, C2 descriptive labels, pooled and per-density rates, all ten controls, and the D = R + T identity at every block. Every compared numerical leaf agrees to less than 1e-10.

The corrected run records **3,726 assertions/comparison leaves, zero failures**. This count includes individual numeric leaves, not 3,726 statistically independent experiments. The run is in `recomputed.json`; `SUMMARY.json` is the compact record. The source and all read inputs are bound in `input_manifest.json`. The separate artifact manifest binds the review files, including the archived initial probe outputs.

The integrity audit covers:

- Five identity cells and all 48 evaluation cells in the fixed density/seed/arm order, complete grids, run/commit/clean-tree receipts and matching source/checkpoint pins; each cell's quota counts and result/ledger hashes; 15 hashed receipt-source preimages at the execution commit, the frozen launcher and validity-module source pins.
- All four checkpoint files hashed without model deserialization; their TRAINED train reports and sidecar bindings; treatment, rule, robot, camera, no-LiDAR, worktree import origins and matched module seeds in snapshots/receipts. Current source bytes agree with frozen commit bytes on the inspected path.
- Vehicle/depth/rule install and runtime telemetry, motor-limit live readbacks and counters, gravity/mass/cap consistency, MinZ/noise/holes settings, rule ON/OFF keys, summary counts and zero sentinel mismatch. These establish the recorded synthetic implementation, not fidelity to a real sensor.
- Exact normalized result and full episode-ledger equality for both v2 identity/reference pairs; a rule-ON negative control that differs; an exact rule-ON repeat; a distinct v3 smoke; and identity/eval full-ledger and normalized-result reproducibility for V2Rs1 and V3s1. Every required between-arm ledger distinction is non-vacuous.
- The complete original CAM-K1-E manifest (**322 files**) and CAM-K2-E manifest (**572 files**) hash correctly, including all reused v2 and claim-scale control cells. The new manifest covers **490 files, 404,581,600 bytes**, with no mismatch or omitted raw file. The three copied result reports are byte-identical to their external originals.
- Rate projection recomputes to **8,810.429834260081 s**, below 12,600 s; actual identity and eval elapsed times satisfy their caps. Execution records and chain log consistently show host fault counts 0 → 0 with unchanged boot id. Review kernel counts are also 0 → 0.
- Legacy-window crash causes, sample-weighted speed-governor summaries and per-cell first-50-call validity summaries independently agree with the original analysis. These do not become quota/contact-event measurements.

## Primary result retained

Crash differences in percentage points; one training-seed pair per contrast, over the 12 registered evaluation blocks.

| Contrast | Mean | Bootstrap 95% | t 95% | Fixed label |
| --- | ---: | --- | --- | --- |
| D_s1 | -1.50146484375 | [-2.0263671875, -0.9765625] | [-2.2556860313571003, -0.7472436561428996] | NO_DETECTED_CHANGE |
| D_s2 | -1.1474609375 | [-1.7618815104166667, -0.54931640625] | [-2.033312205588523, -0.261609669411477] | NO_DETECTED_CHANGE |
| R_s1 | -1.81884765625 | [-2.2745768229166665, -1.416015625] | [-2.444641443466101, -1.1930538690338988] | NO_DETECTED_CHANGE |
| R_s2 | -2.6407877604166665 | [-3.0232747395833335, -2.2501627604166665] | [-3.2024461650317555, -2.0791293558015775] | SMALL_REDUCTION |
| T_s1 | +0.3173828125 | [-0.19938151041666666, +0.8138020833333334] | [-0.44415713490256037, +1.0789227599025604] | NEGLIGIBLE |
| T_s2 | +1.4933268229166667 | [+0.7364908854166666, +2.2420247395833335] | [+0.40334511204319656, +2.583308533790137] | NO_DETECTED_CHANGE |

Both D intervals exclude zero toward lower crash rates, but both point estimates miss the preregistered 2 pp reduction floor. Therefore `NO_DETECTED_CHANGE` is the required gate label. It must not be paraphrased as zero statistical change, equivalence, proof of no benefit, or SMALL_REDUCTION. Both T point estimates are positive; a registered added reduction from v3 retraining over V2R is not established. R's SMALL_REDUCTION occurs for seed 2 only. All quota timeout outcomes are zero. No pooled training-recipe interval or causal decomposition is justified.

## Scope and validity limits

This PASS covers the recorded synthetic grid, immutable raw bindings and fixed calculations. It does not qualify real OAK-D invalid masks, real images, real flight, generalization beyond the four densities, or the causal source of crashes. T also contains train/eval input mismatch and training-run variability. The intervals describe evaluation blocks conditional on each checkpoint pair, not variation across an independently sampled population of training seeds.

Only two of 24 reused v2 reference cells were replayed for identity; the remainder rely on their committed manifests and common deterministic path. A retrospective audit can corroborate recorded preflight/clean-tree/import/host evidence, but cannot independently reconstruct continuous host idleness between all logged events or the originating chat beyond the preserved approval record. The approval record names this exact 53-cell unit, V2R arms and execution commit; original chat authorization is not re-created by this review.

The rule's numeric telemetry covers its first 50 calls, not a full-run contact-aligned trace. **Contact-time hit-token share, same-time camera visibility and full-run contact clearance are NOT_RECORDED by this unit's rule telemetry.** Crash-cause counters and governor summaries are legacy-window aggregates; they do not supply those missing aligned metrics. The source inspection confirms the rule acts on the delivered depth-model invalid mask and camera ray geometry and introduces no target-ground-truth argument on that wrapper path; this is not a new audit of every historical actor-input path.

## Review execution and traceability

CPU jobs ran with `CUDA_VISIBLE_DEVICES=''`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, affinity CPUs 6,7, `nice -n 10`, `ulimit -v 8388608` (8 GiB address space), and `timeout -k 10 600`. No job was left running. Shared root session clock was checked without resetting or closing it; inherited deadline 2026-11-10T05:06:00Z, finish-by 04:56:00Z.

**Git Traceability:** Branch `camonly/dev`; Base SHA `aaca40e8dc6a3492cedd0b692427cbc796661015`; New Commit SHA **NONE (external read-only review; parent owns the result-record commit)**; Changed Files **only `~/motar_tmp/camk2v3e_20261110/review_results_r1/`**; Tests **independent probe PASS, 3,726 assertion/comparison leaves**; Working Tree **author's new result/index/WORKLOG records present, not reviewer edits**; Remote Push **NO**; Main Merge **NO**; Raw/Checkpoint Integrity **all audited manifests and checkpoint pins match, no raw/checkpoint writes**.
