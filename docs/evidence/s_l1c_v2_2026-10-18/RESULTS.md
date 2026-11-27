# S-L1C v2 — matched near-range LiDAR qualification

**Status: GPU_QUALIFIED_PENDING_REVIEW.** Independent Fable 5.1 read-only review is required before final GO. This record does not establish hardware realism or Recovery performance.

## Source and immutable records

- Accepted base: `aed6affbd2567feee79e658fbe5f07510b039f36`. Preregistration `51a9015`; independent CPU oracle `396ca61`; CPU result `483e0c2`.
- GPU source: `e45794e876b15f71b5f38c65702fefadc368f93a`; runtime implementation `5b57021`. Amendments `d66acd6` (seven exact negative replays) and `0c2bddc` (actor firewall) preceded measurement.
- CPU run `SL1CV2-CPU01`, raw `../aerial_gym_simulator/results/s_l1c_v2_cpu/SL1CV2-CPU01`; GPU run `SL1CV2-C01`, raw `../aerial_gym_simulator/results/s_l1c_v2/SL1CV2-C01`.
- GPU [manifest](GPU_RAW_MANIFEST.json): SHA-256 `fce6b844146a6e2869f6e619bb7c57aef8d049c662855d9a05c3034c0cb7443b`, 284 files. [Run report](GPU_RUN_REPORT.json): SHA-256 `b902491953e23c35d2e1dd7e2b5c4883eec9f8eab334719e37d6cf91fd642c77`.
- CPU manifest SHA-256 `14d55c094b804e9c6de3c7edd16455c1e2b6a1d8b2977df89afa314154ac3f95`. [Independent GPU analysis](s_l1c_v2_analysis.json) SHA-256 `15149c888624a9c460d8e209e1ada7ea5f5441bb2ea7ce2acd4c28ce1ab630a7`.
- Frozen Policy F SHA-256 `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`; actor width 898, RMS fixed, no PPO.

## Candidate selection and implementation

[CPU selection](CPU_SELECTION.md) compares all four candidates on 105,408 matched cases plus 1,152 angular-error diagnostics. V2-A alone passes all selection gates. It adds 3,318 correct primary acceptances, loses no existing true acceptance and adds no false acceptance. V2-B adds no primary gain; V2-C produces 18,054 false acceptances and lacks an eligible identity/expiry contract. V2-0/A/B each retain the same 135 inherited false acceptances across the full noise matrix. These remain known failures, not target hits.

V2-A preserves every C-v1 accepted row/range. On a C-v1 rejection, at least two vertical returns must fit a 0.20m sphere better than an equal-horizontal-range surface, with the preregistered residual/noise margins. Ambiguous evidence abstains. Online inputs are ranges, ray table, camera-valid flag and own track prediction/active/gate; no semantic label, target GT or E-L0 provenance enters the decision.

Production scope: new `aerial_gym/task/navrl_task/navrl_lidar_association_v2.py::range_only_lidar_association_v2` and `_median`; `navrl_perception.py::__init__` / `_associate_lidar_target` opt-in dispatch; `navrl_sensor_lineage.py::LINEAGES` / `level` map the new mode to A+B+C level 3. No v1 covariance/clock function, actor, dynamics, detector or checkpoint changed. Use `NAVRL_SENSOR_LINEAGE=s_l1c_v2`; default OFF and every old mode remain separate. S-L1D is disabled on this lineage.

## GPU recall and abstention

144 unique rendered positives × three radial predictions = 432 matched probes per lineage. Recall denominator is independent sphere first-hit opportunity in the selected azimuth column; no-hit scenes are also reported. Repeated predictions/bearings are deterministic fixture cases, not independent population samples.

| Centre range | Opportunities / probes | v1 true recall | v2 true recall | v1 / v2 rejected probes |
| --- | ---: | ---: | ---: | ---: |
| 1.16m | 72/72 | 24/72 (33.33%) | 48/72 (66.67%) | 48 / 24 |
| 1.5m | 72/72 | 36/72 (50.00%) | 72/72 (100.00%) | 36 / 0 |
| 2m | 72/72 | 36/72 (50.00%) | 72/72 (100.00%) | 36 / 0 |
| 2.28m | 72/72 | 44/72 (61.11%) | 72/72 (100.00%) | 28 / 0 |
| 2.3m | 36/72 | 36/36 (100.00%) | 36/36 (100.00%) | 36 / 36 |
| 3m | 36/72 | 36/36 (100.00%) | 36/36 (100.00%) | 36 / 36 |

Primary <2.295m: **140/288 (48.61%) → 264/288 (91.67%)**. Paired true losses 0; new false accepts 0. At 2.30/3m, half the probes have no sphere hit in the selected column; conditional recall 100% does not imply 100% availability.

| Target elevation | v1 / opportunities | v2 / opportunities |
| --- | ---: | ---: |
| -5° | 0/48 | 44/48 |
| 0° | 68/72 | 68/72 |
| 5° | 0/48 | 44/48 |
| 10° | 68/72 | 68/72 |
| 15° | 8/48 | 44/48 |
| 20° | 68/72 | 68/72 |

The [full GPU distance/elevation table](gpu_distance_elevation.csv), [CPU table](cpu_distance_elevation.csv) and raw ledgers retain strata. GPU positive v2 decisions: 212 unchanged v1 acceptances + 124 geometry recoveries + 96 abstentions (24 below conclusive range, 72 insufficient support); v1 rejects 220. The 24 remaining 1.16m failures include inward-biased predictions below the unchanged 1.15175m conclusive threshold. No threshold was relaxed.

## Negatives, range, covariance and identity

- Per lineage: seven original EQ-1D scan replays × three predictions = 21 probes, false association 0/21; twelve new tall/narrow bar scenes × three = 36 probes, 0/36; twelve new foreground scenes × three = 36 probes, 0/36. The new narrow bars include width 0.02m. No additional false association in any GPU pair. Seven replays are not new rendered scenes.
- V2 negative decisions: 47 abstentions (2 range, 45 support) and 46 explicit unexpected-near-surface rejections. Semantic labels are used only to score the selected first hit. No accepted obstacle is relabelled as a target.
- Newly correct range: 124 updates, MAE 1.6191727e-06m, maximum 1.29137261e-05m; gate ≤0.02m. All-correct centre-range MAE v1 0.00636634317m / v2 0.00401745693m. **Inherited maximum 0.168693343m remains unchanged**; this task does not fix the old accepted-return centre approximation or camera bias.
- Independent first-hit oracle: maximum beam error 2.77428001e-05m, 0 geometry mismatches at the preregistered 1e-4m tolerance and 0 production/reference decision mismatches.
- Range-only covariance, v2 336 accepted covariance probes: radial posterior/prior 0.0059220886–0.0068188369; smallest orthogonal ratio 0.999600047 ≥0.99. Directional covariance code is unchanged; this gate does not calibrate real sensor uncertainty.
- Saved clock traces: identity age rises 0.1…1.0s across ten measurement-loss steps; LiDAR never resets it. Fused age resets on accepted range only as in v1. After 50 full misses the track remains active, the 51st expires it; camera reacquisition resets identity age. This is preservation of the existing fused 5s contract, not a new identity-confirmation guarantee.

Closed-loop update-source traces (0 prediction, 1 camera, 2 LiDAR; 4,808 observations include initialization/reset staging and are not 4,808 independent trials):

| Config | Mode | Prediction | Camera | LiDAR | Max identity age (s) |
| --- | --- | ---: | ---: | ---: | ---: |
| static | v1 | 501 | 3886 | 421 | 2.30 |
| static | v2 | 501 | 3886 | 421 | 2.30 |
| moving | v1 | 895 | 3551 | 362 | 4.80 |
| moving | v2 | 827 | 3550 | 431 | 4.20 |

These ON behavior differences are frozen-checkpoint robustness observations. No tracking-performance or Recovery improvement is inferred from extra association counts.

## Regression, firewall, resource and preservation gates

- 23/23 GPU processes completed. Ten historical OFF/A/B/C/D comparisons (static/moving) match all fourteen keys for all 600 steps; two new v2 E-L0 P1/P2 comparisons also match. D API shape/non-mutation is CPU-tested; D validity accuracy on C-v2 remains NOT_TESTED.
- Sensor GT/semantic replay preserves association, measurement, tracker state/covariance and identity/fused ages. Actor firewall has five nonvacuous branches (scheduled 50/150/300 and two reset boundaries), GT all/permutation/subset replays and five measurement-positive controls. Actor898/RMS/import origin and E-L0 provenance checks PASS.
- CPU: independent reference 10/10, runtime 7/7, canary design 5/5 PASS. PUBLIC_CPU at committed runtime source: 2,587 run, 0 failures/errors, 10 skipped, 5 excluded. The earlier dirty-HEAD guard failure was resolved by committing source; no test was weakened. The extra D API case and five canary tests ran separately afterward.
- GPU process wall total 863.892s (0.2400h); runner wall 865.556s; peak total device usage 4670MiB. Free disk 18.911GB before / 18.808GB after; output 30891408 bytes. Per-process 900s / overall 7200s, 8GB disk and OOM guards respected. No tracked file/WORKLOG/source HEAD edit during execution.
- [Preservation](preservation.json): all 15 prior manifests and 4,085 raw members verified, all historical source/evidence unchanged outside the explicitly approved runtime/contract/log paths; Policy F both worktrees, main HEAD, user WIP three hashes, stash and append-only WORKLOG prefix preserved. New CPU two files and GPU 284 files rehashed. No external backup established by this task; originals remain on the same disk. No deletion or automatic raw staging.

## Limits and next scope

Only the preregistered precision-limited simulator scope is qualified pending review. CPU ±1cm improves recall only slightly; ±3cm adds no recoveries because shape evidence is ambiguous. Structured bounded-noise probes are not a calibrated hardware noise distribution. Missing returns, spherical-radius mismatch and larger prediction errors remain outside this qualification. Existing far-range false associations, B target-return retention, camera centre-range bias and D belief-error realism are not solved.

Only [validation designs](../../plans/post_s_l1c_v2_validation_design.md) are prepared for those next items; no B/D production changes or new GPU runs are included.

## Reproduction (CPU analysis only)

```bash
python -B tools/s_l1c_v2_analysis.py --run ../aerial_gym_simulator/results/s_l1c_v2/SL1CV2-C01 --out /tmp/cv2-analysis-fresh.json
python -B tools/s_l1c_v2_report.py --run ../aerial_gym_simulator/results/s_l1c_v2/SL1CV2-C01 --analysis /tmp/cv2-analysis-fresh.json --cpu-manifest docs/evidence/s_l1c_v2_2026-10-18/CPU_RAW_MANIFEST.json --out-dir /tmp/cv2-tables-fresh
```

Use fresh output names; originals are not overwritten. GPU reexecution would require a new run ID/preregistration and authorization, not reuse of the immutable raw folder.

## 2026-10-18 review disclosure addendum

This append-only addendum addresses the independent [Fable review](FABLE_REVIEW.md); the original result section and preregistered gates above are unchanged. Final disposition: [QUALIFICATION.md](QUALIFICATION.md).

### Angular prediction error — diagnostic, not qualified

The headline GPU 91.67% is for angularly exact predictions with radial offsets only. The preregistered CPU ±1° bearing/elevation error set is substantially harder:

| Candidate | True hits / opportunities | Recall | Abstentions / total probes | New max centre error |
| --- | ---: | ---: | ---: | ---: |
| V2-0 | 312/864 | 36.11% | 0/1152 | — (no new accepts) |
| V2-A | 504/864 | 58.33% | 648/1152 | 0.0051587279m |

V1 has 840 explicit rejections; V2-A has 648 abstentions. Both have zero false accepts and zero paired true loss in this diagnostic. The 192 additional correct associations have maximum centre error 5.159mm. These figures are read from the committed CPU aggregate; angular-error robustness is **diagnostic, not qualified**, and cannot inherit the GPU angularly exact headline. Production assumes the fixed 0.1mm simulator precision bound; offline higher-noise trials increase that bound in the hypothesis test. They do not establish that fixed-parameter production is hardware-noise robust.

### Negative coverage and reconstructed reason labels

Negative coverage is vertical box faces and the named EQ-1D fixtures. Ground/top edges, non-vertical surfaces and adjacent rows hitting different objects remain NOT_TESTED. CPU uses gate 0.696m; GPU covariance-prior probes also exercise the production adaptive gate up to 1m, with no negative accepts. This does not qualify all adaptive-gate scene distributions.

The `decision_counts` in gpu_metrics.json are **independent-reference reconstructions from saved scans/predictions**, with valid decisions checked against production. Production reason-code telemetry is NOT_RECORDED. The actual saved production rejected counts are 96/432 positive and 93/93 negative probes for v2; the 47-versus-46 negative abstention/veto split is the reference diagnosis, not an exported runtime reason code.

### Post-review closed-loop robustness disclosure

This descriptive recount is new after review and is not a preregistered selection/GO gate. Source and raw are unchanged. [Diagnostic JSON](closed_loop_diagnostic.json) binds every ledger/frame hash.

| Config | Mode | Completed capture | Crash | E-L0 correct LiDAR / false |
| --- | --- | ---: | ---: | ---: |
| static | v1 | 116 | 5 | 419 / 0 |
| static | v2 | 116 | 5 | 419 / 0 |
| moving | v1 | 96 | 4 | 362 / 0 |
| moving | v2 | 89 | 5 | 431 / 0 |

Moving trajectories diverge from frame 105 (review finding); fixed 600-step runs produce different completed-episode counts. These counts are not a matched completed cohort, and no performance improvement is established. Open episode outcomes are not imputed. E-L0 records 4,800 action-aligned frames per arm; the 4,808 update-source observations above include initialization staging, explaining static 419 versus 421 LiDAR updates. Semantic provenance scoring is evaluator-only and does not newly establish independent geometric truth for every closed-loop frame.
