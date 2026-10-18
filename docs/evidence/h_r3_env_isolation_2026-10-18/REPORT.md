# Environment Isolation Qualification — ISO-Q02

**GO — Mechanics Qualified**, limited to the registered seed4602/N128/moving160-bar/40-step profile. H-R3 remains **PROVISIONAL / CONFIRMATION HOLD**. No P03 or confirmation run occurred.

## 1. Cause and intervention

`aerial_gym/env_manager/warp_env_manager.py::WarpEnv.reset_idx` lines44–48 rewrites every environment's vertex buffer from current actor roots; lines52–53 refit only reset environments. Unselected static actor quaternion representations can normalize after physics. Recomputing their mesh coordinates changes shared scene storage even when that environment did not reset; its BVH is not refit. This source pattern was a hypothesis until controlled CPU and actual GPU reproduction.

Independent CPU geometry and actual Warp BVH rays reproduce the mechanism. In real N128, only env127 is forced at step2, baseline has no natural reset then. The other127 roots/quaternions/actions are equal at onset, but legacy changes154,832 vertex rows and126 environments' RGB/depth/LiDAR. Actor observations diverge at2, actions/physical states at3. The selected-only evaluator intervention removes the mutation and every observed downstream difference. This identifies the mechanism in this controlled diagnostic. Old P02 lacks per-modality/vertex snapshots: **the cause of all14 historical prefix failures is not retrospectively proven**.

New `tools/h_r3_env_isolation.py::WarpResetIsolation.reset_idx` updates the original selected contiguous slices in place with the exact production transform and refits the corresponding meshes. Mapping/topology/storage lifetime are checked. It does not globally rewrite then restore unselected values. Original production and P02 RNG/kinematic/indexed-root-write paths are unchanged; root writes and refit indices were recorded. No additional runtime full-root setter was observed. This qualifies the audited static-obstacle mesh contract, not arbitrary moving mesh assets.

## 2. Independent CPU geometry/rays

11 geometry tests PASS: independent NumPy float64 SE(3), triangle intersection, actual Warp CPU mesh/refit/rays, pose/quaternion changes, unrelated reset preservation, variable actor/vertex mapping, order/batch/all resets and fail-closed topology/storage checks. Tiny quaternion normalization: legacy untouched vertex max change9.5367431640625e-7m; actual ray change4.76837158203125e-7m. Large pose negative control produces stale-BVH false miss; fixed untouched scene/rays stay exact and selected scene is updated correctly. Across1000 random SE(3) points float32 versus independent float64 max error6.343943792508355e-7m. Declared oracle tolerances were not changed. See CPU_GEOMETRY.md and cpu_report.json.

24 initial focused tests PASS (13.164s); amendment2 runner suite4/4 PASS (0.013s), giving25 distinct focused tests. Existing simulator/P02 source byte preservation is tested. Tests were not represented as a full repository regression.

## 3. Actual N128 sensor/state comparisons

Initial plus all40 steps are compared bitwise; SHA checks supplement full numeric snapshots, not quantized summaries. Mesh handles are compared against each process's own mapping, not across address spaces.

| Comparison | Environments | RGB | Depth | LiDAR | Roots/tracker/actor/actions/vertices |
|---|---:|---|---|---|---|
| Legacy normal vs force127 |127 untouched|FAIL at2|FAIL at2|FAIL at2|vertex/observation onset2; physical/action onset3|
| Selected normal vs force127 |127 untouched|PASS exact|PASS exact|PASS exact|PASS exact all41 frames|
| Selected evaluator ON vs OFF |128|PASS exact|PASS exact|PASS exact|PASS exact all41 frames|
| Legacy vs selected forced env127 at2 |1 reset|PASS exact|PASS exact|PASS exact|PASS exact reset semantics|

Legacy onset maximum absolute numeric differences: RGB3.787875175476074e-5 (raw rendered values), depth8.594989776611328e-4m, LiDAR1.8024444580078125e-4m, vertices1.52587890625e-5m. All three sensor modalities differ in126 of127 untouched environments; the remaining environment is retained. Step2 natural resets=[] in all five processes; forced IDs=[127] only in treatments. Initial states/sensors all128 match across all five processes. No events or environments were excluded to obtain PASS beyond the preregistered reset target127 in the isolation comparison. See analysis.json for exact fields and first mismatch chronology.

## 4. Limits and safeguards

Qualification is only registered seed/profile/horizon, not all seeds, densities, dynamic mesh models, noisy sensors or long-run numerical determinism. No proof that every other possible shared state is absent; measured all-root/derived-cache/tracker/history/actor/sensor invariance covers this profile. GT/geometry/provenance snapshots remain evaluator-only; online RGB-D+LiDAR and FrozenF actor898 inputs are unchanged. Evaluator noninterference and source inspection PASS; this does not establish full sensor realism or Recovery efficacy. P01/P02 exploratory/failed evidence is immutable and excluded from confirmation.

## 5. Resources, failures and preservation

ISO-Q01 source `d5c91cec21c7e35917409b612957ecfb30f74a84`: legacy_normal completed40 steps then supervisor rejected valid nested completion paths. It stopped before second launch. Raw50 members/manifest retained; no mechanics outcome inferred. Amendment2 fixes only bookkeeping and uses fresh ISO-Q02.

ISO-Q02 source `9dabd3c3771ba7f381b20e7fa67ed8820052c50d`: five processes completed,282.6441 process seconds (0.0785h),283.9696 supervisor seconds. Including ISO-Q01:334.6642s (0.0930h). Peak whole-device usage5,380/8,192MiB. ISO-Q02 output about2.073GB; first attempt0.415GB. Free disk18.151GB before first run,15.661GB after final run, never below8GB guard. Caps600s/process/3600s total; source and WORKLOG frozen throughout each live run; no OOM or time-cap stop.

ISO-Q02 RAW_MANIFEST SHA256 `2484270805728b8d160b37fdab2b3a9ad3d461457770bee27e0113e0e19ce5f5`,242 members; ISO-Q01 `59861483db04b3bd4af012fa58abd217d475bfd6246cbbcbb0f0d0f240bb907d`,50 members. Both independently rehashed PASS. Raw root `../aerial_gym_simulator/results/h_r3_env_isolation_v1/`; committed copies of manifests and compact evidence only. Neither symlink nor merge commits raw. External backup remains NOT_ESTABLISHED; no data deleted.

Preservation PASS:6006 baseline tracked paths (WORKLOG only append),31 original raw manifests/5950 members, main HEAD cb644bbe, three main WIP files, stash, protected untracked results and both PolicyF checkpoint copies unchanged. FrozenF SHA256 `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`. No main merge/push, force-push, production change, retraining or Planner.

## Reproduction and traceability

Base `e6814a675a7b0d645d24e36b48bbeb2d6681f706`; preregistration commit c61d696, CPU/source d5c91ce, amended GPU source9dabd3c. To audit completed raw without GPU: `PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/h_r3_env_isolation_analysis.py --run ../aerial_gym_simulator/results/h_r3_env_isolation_v1/ISO-Q02 --out /tmp/isolation_analysis_fresh.json`. Output must not exist. Original launch command at source9dabd3c: `PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B tools/h_r3_env_isolation_run.py run --approved-commit 9dabd3c3771ba7f381b20e7fa67ed8820052c50d --run-id ISO-Q02`; this deliberately refuses reuse of existing raw. See preregistration/amendments for a separately approved future run, not automatic replay.

Actual file list: new geometry wrapper, diagnostic launcher, bounded supervisor, analysis and three focused test modules; new preregistration/amendments/evidence; WORKLOG append. Independent Fable5.1 review and publication receipts are recorded separately after this result commit.
