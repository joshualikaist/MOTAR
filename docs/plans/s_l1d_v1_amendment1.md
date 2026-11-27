# S-L1D v1 amendment 1: GPU fixtures for undetected partial, centre-only and scale loss

Recorded after SL1D-C01 and before any SL1D-C02 process. The original
[plan](s_l1d_v1.md) and [JSON](s_l1d_v1.json) bytes are unchanged, as are the SL1D-C01 results
(`aab7379`) and the INCOMPLETE record (`bd65d55`). The [amendment JSON](s_l1d_v1_amendment1.json)
governs this extension.

**Why.** In SL1D-C01, no GPU fixture produced an undetected PARTIAL, CENTRE_ONLY or SCALE_LOSS
state. All 12 CENTRE_ONLY and PARTIAL passes were detected, and there was no SCALE_LOSS fixture.
The user's requirement to compare those classes against real GPU RGB-D was therefore unmet, and
S-L1D stayed INCOMPLETE.

**What changes.** Only fixtures and evaluation are added:

- 13 sensor scenes;
- the sensor-only canary `SL1D-C02`;
- C02-specific realisation gates.

The following stay as they were:

- the S-L1D production algorithm (`aerial_gym/` and `motar_eval/` at `76f0f4c`);
- the online classification rules, precedence, τ and the v1 evaluator-to-online mapping;
- every v1 gate;
- S-L1A, S-L1B and S-L1C.

## Scenes (designed with the independent float64 oracle)

[`tools/audits/s_l1d_c02_scene_search.py`](../../tools/audits/s_l1d_c02_scene_search.py)
re-verifies each scene. The GPU bar half extents are randomised per slot, so an occluder is
specified by its edge (`edge_left_y` / `edge_right_y`). A scene is accepted only if its oracle
class, `alone_px`, `visible_px` and centre-robustness stay stable under all of these variations:

- hx 0.20/0.30/0.40 × hy 0.20/0.40;
- ±4 mm edge shifts;
- ±1.5 mm target shifts.

| Scenes | Geometry | Designed evaluator class | Designed online class |
| --- | --- | --- | --- |
| C02_SC0_1, _2 | clear, 19.8 and 20.0 m, support 0, ≥ 18 mm boundary margin | SCALE_LOSS | PRED_SCALE_LOSS |
| C02_SC1_1, _2 | clear, 15 and 18 m, support 1 | VISIBLE (1 px) | PRED_SCALE_LOSS |
| C02_SC1_FULL | 18 m, support 1, behind a bar | FULL (alone 1) | PRED_SCALE_LOSS |
| C02_PA_1…4 | 12/13 m, 2-pixel support, one pixel covered, centre open | PARTIAL (2/1) | OCCLUSION_PARTIAL_EVIDENCE |
| C02_CO_1…4 | same targets, the other side covered, centre blocked | CENTRE_ONLY (2/1) | OCCLUSION_CENTRE_EVIDENCE |

These design expectations are not gates. The gates use the GPU evaluator's actual classes. A scene
whose rendered class differs from the design is reported, never excluded.

## What is recorded

For each scene and pass, the following are recorded:

- the GPU E-L0 class and the oracle class;
- `alone_px` and rendered `visible_px`;
- the camera detection flag;
- delivered depth over the predicted support;
- the primed belief and covariance (1e−6·I);
- the production online class, support and occluded count;
- the independent oracle online class recomputed on the delivered GPU depth;
- whether the two agree.

The level, roll15 and pitch15 passes must be bitwise identical, and only the level scene pass
counts as an independent case.

## Gates and verdict

| Gate | Requirement |
| --- | --- |
| C02-UNIQUE | Only the level scene pass counts; attitude duplicates are verified identical. |
| C02-ORACLE | Production support equals E-L0 `alone_px` and the oracle, within the boundary allowance. The production online class equals the oracle online class on the delivered depth. |
| C02-MAPPING | Every undetected unique case meets the v1 mapping. An undetected case with alone < min must be PRED_SCALE_LOSS. No scene is excluded. |
| C02-SCALE | At least 1 undetected case with alone 0 or 1, online PRED_SCALE_LOSS, oracle-agreeing. |
| C02-PARTIAL | At least 1 undetected GPU-PARTIAL case with alone ≥ min, mapping-consistent, oracle-agreeing. |
| C02-CENTRE | At least 1 undetected GPU-CENTRE_ONLY case with alone ≥ min, centre-robust, online OCCLUSION_CENTRE_EVIDENCE, oracle-agreeing. |
| C02-SENSOR-NONINTERFERENCE | `sensors_s_l1c` and `sensors_s_l1d` deliver identical renders and detection. |
| C02-INTEGRITY | Production equals `76f0f4c`; checkpoint, origins, frozen tree, manifest and finite values are valid. |

**Verdict.**

- **GO:** all three states are realised and every C02 gate passes, with SL1D-C01's 8/8 v1 gates
  retained.
- **FAIL:** a required state is realised but the online result violates the mapping or the oracle.
- **INCOMPLETE:** a required state is not realised on GPU, or independent evidence is missing.

A GO does not claim real-robot sensor accuracy or Recovery performance.

The CPU-test omission found after SL1D-C01 (`bd65d55`) stays a separate review record. C02 adds
the GPU evidence for the same classes; it does not rewrite that history.
