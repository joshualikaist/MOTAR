# Sensor direction: single RGB versus RGB-D candidates (comparison only)

Status: **analysis, no implementation.** Policy F's RGB-D + LiDAR environment stays the **Frozen
Legacy Baseline** unchanged. No camera-only environment was built, and no input was removed from
Policy F; per the user's direction neither happens before a separate approval.

## What Policy F actually consumes

The 898-D actor input is built only by `NavRLPerceptionModule.observe` (EQ-1C C1):

| Block | Width | Source in the current contract |
| --- | ---: | --- |
| Robot history | 5 × 10 = 50 | ego state (ideal odometry), previous action |
| Target token history | 5 × 16 = 80 | camera detection (bearing/elevation, mean-depth range), LiDAR range association, Kalman covariance |
| Obstacle tokens | 5 × 8 × 12 = 480 | clusters extracted from the fused scan |
| Fused static scan | 72 × 4 = 288 | 360° LiDAR, with camera depth minima fused into the in-FOV bins |

So 768 of 898 inputs depend on metric range from the 360° LiDAR or the synthetic depth camera, and
the riskcap governor reads LiDAR clearance along the command direction. Any candidate without the
LiDAR changes the meaning of those inputs. Zeroing or deleting them would be out of distribution
for Policy F, and the user's direction excludes it.

## Candidate comparison

Camera numbers use the current 160 × 90, 87° HFOV camera (`fx = 84.30`) and the 0.30 m target.
The target spans about 25.3 / d pixels: 2.5 px at 10 m, 1.3 px at 20 m. EQ-1A A5 finds possible
scale loss from 12.75 m.

| Criterion | Single RGB | RGB-D (no LiDAR) | Current RGB-D + LiDAR (baseline) |
| --- | --- | --- | --- |
| Target bearing / elevation | yes (A7: ≤ 0.10 px on analytic frames) | yes | yes |
| Target range | not observed. Needs a size prior (`d = fx·D/s`: ±0.5 px on a 2.5 px blob is about ±20 % at 10 m, none beyond the scale band), bearing-only filtering with observer manoeuvre, or learned monocular depth (domain gap) | measured inside the depth sensor's range; a real stereo/ToF error grows with range and needs a declared noise model | measured (camera depth, LiDAR); conventions currently biased (A4, A7) |
| Obstacle geometry inside the FOV | no metric range (monocular estimate or motion parallax only) | metric | metric (fused, with the A1/A2 geometry errors) |
| Obstacles outside the 87° FOV | none | none, unless a memory/occupancy map is built from past frames | 360° LiDAR |
| Occlusion reasoning (occluder in front of target) | appearance only | depth ordering available | depth ordering and LiDAR |
| Identity | appearance only | appearance + depth segmentation | same as RGB-D; LiDAR association is range-only (A6, T3) |
| Policy F checkpoint compatibility | **no**: 768 range-dependent inputs lose their source | **no**: the 288-D 360° scan and tokens lose their source outside the FOV | yes (frozen) |
| New training needed | yes (new schema and perception) | yes (new schema; or a new map front-end and retraining) | no |
| Governor / safety logic | must be redesigned (no clearance outside the FOV) | must be redesigned for in-FOV-only clearance or memory | current riskcap (B5 FAIL as a guarantee) |
| Research cost (relative) | highest: range source plus error model, perception, schema, training, qualification, real appearance data for realism | medium: depth noise model, map or FOV-limited schema, training, qualification | lowest: fix conventions in an opt-in lineage (S-L1) and re-evaluate |

## Assessment

- The baseline stays as it is. Its historical results are whole-system results of the RGB-D +
  LiDAR contract with the EQ-1 caveats.
- If the research question needs camera-only sensing, **RGB-D is the lower-risk first step**. It keeps
  metric target and obstacle range inside the FOV, so the remaining difference from the baseline is
  mostly coverage outside the FOV. That difference can be measured with a trained ablation rather
  than by deleting inputs from Policy F.
- **Single RGB** is justified only when the claim needs it. Its range-information source and error
  model must be declared before training. Without range, capture approach, braking and occlusion
  reasoning all rest on priors.
- Both candidates require new training (not approved now), a sensor noise and latency model, an
  EQ-1A-style geometric qualification of their own sensor code, and an evaluation contract
  ([EVALUATION_CONTRACT.md](EVALUATION_CONTRACT.md)) shared with the baseline.
- A useful intermediate that needs no new training: the `S-L1` calibrated-fusion lineage of the
  current contract ([REPORT.md](REPORT.md)). It separates "geometry errors of the baseline" from
  "sensor modality" before any camera-only work.
