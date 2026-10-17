# S-L1D v1: occlusion and measurement validity (preregistration)

Status: preregistered after the recorded S-L1C GO decision (`ffbbaf3`) and before any S-L1D runtime code
or measurement. [`s_l1d_v1.json`](s_l1d_v1.json) is the machine-readable contract
and governs if the two disagree. No gate may be relaxed to reach a pass.

## Problem

EQ-1A A5 and EQ-1D D2-O1 found that the legacy indicators confuse physical occlusion with sensor
loss. The legacy indicators are the frustum test, the offline centre ray, the 8-ray rim proxy and
the rendered pixel count. There were 3 disagreements out of 23 rendered scenes:

- S08: a 24 m range exit reads as an FOV exit.
- S12: a partial occlusion with the centre still visible reads as visible.
- S14: a rim gap reads as fully occluded while 16 pixels remain detectable.

E-L0 already provides evaluator truth (GT classes RANGE_EXIT, FOV_EXIT, SCALE_LOSS, FULL,
CENTRE_ONLY, PARTIAL and VISIBLE). Nothing online can yet tell, from the sensors alone, why the
target is not being measured.

## Correction (cumulative `s_l1d_v1`, diagnostic only)

S-L1D adds an online validity state that a real robot could compute.

**Inputs it may use:**

- delivered depth;
- the detector flag;
- the track's own predicted centre;
- S-L1C's update source and identity age;
- declared constants: the S-L1B intrinsics and extrinsic, target radius 0.15 m, detector range
  20 m, `min_target_pixels`, and a depth tolerance floor of 0.05 m.

**Inputs it never reads:** evaluator classes, the renderer mask, semantic ids or the simulator
target pose.

**Effect on the actor and tracker:** none. The 898-D actor input and the tracker are unchanged,
so closed-loop s_l1d_v1 must be bitwise equal to s_l1c_v1.

**Per-step online classes.** The classes and their rules are in the JSON:

- NO_TRACK
- DETECTED
- PRED_RANGE_EXIT
- PRED_FOV_EXIT
- PRED_SCALE_LOSS
- OCCLUSION_FULL_EVIDENCE
- OCCLUSION_CENTRE_EVIDENCE
- OCCLUSION_PARTIAL_EVIDENCE
- UNEXPLAINED_LOSS

The predicted support is the set of pixel-centre rays that hit the 0.15 m sphere around the
predicted centre (the E-L0 target-alone rule applied to the prediction). Occlusion evidence means
delivered depth sits more than τ nearer than the predicted sphere surface, where
τ = 0.05 m + 2·σ_los. σ_los is the track's own line-of-sight range uncertainty, so τ is
sensor-inferable and not tuned. If the support is below `min_target_pixels`, the class is
PRED_SCALE_LOSS regardless of occlusion: the detector cannot detect fewer pixels.

**Two further layers.**

- **Measurement layer:** CAMERA, LIDAR_RANGE_ONLY or FUSED_LOSS.
- **Identity layer:** IDENTITY_CONFIRMED (identity age ≤ 0.5 s) or IDENTITY_UNCONFIRMED, plus
  HISTORY_ONLY and EXPIRED. The 5 s fused expiry is unchanged.

**Reacquisition.** An identity-confirmed reacquisition is a camera measurement after an
unconfirmed identity. A fused-valid return is a LiDAR range-only measurement after a fused loss.
The two are counted separately.

## Gates

| Gate | Requirement |
| --- | --- |
| D-ORACLE | Production predicted support equals the float64 oracle on CPU. With the true centre primed, it equals the E-L0 `alone_px` on every GPU fixture pass. The only allowed difference is boundary-ambiguous pixels, whose closest approach is within 1e−6 m of the radius. |
| D-FIXTURES | With the true centre primed (covariance 1e−6·I) and the camera not detecting, the online class must match the evaluator class. The required mapping is below. Agreement must be 100% on the preregistered fixtures, on CPU and GPU. Occluders within τ are reported, not gated. |
| D-SEPARATION | An AST/data-flow test shows the online function reads no evaluator classes, renderer mask, semantics or target pose. A closed-loop confusion matrix of online state against evaluator class is reported. |
| D-REACQ | Identity-confirmed reacquisitions and fused-valid returns are counted separately. The GT distance at each reacquisition is reported. |
| D-REGRESSION | On GPU, off equals EL0-R1, s_l1c equals SL1C-C01, and s_l1d equals s_l1c bitwise. On CPU, the lower lineages are bitwise equal to the S-L1C source. |
| D-EL0 | P1 and P2 are identical. |
| D-FIREWALL | The actor is 898-D and no GT reaches it. |
| D-INTEGRITY | As in S-L1A, S-L1B and S-L1C. |

The D-FIXTURES mapping from evaluator class to online class:

| Evaluator class | Required online class |
| --- | --- |
| RANGE_EXIT, FOV_EXIT, SCALE_LOSS | the PRED_ class of the same name |
| FULL, CENTRE_ONLY, PARTIAL with alone_px < min | PRED_SCALE_LOSS |
| FULL (alone_px ≥ min) | OCCLUSION_FULL_EVIDENCE |
| CENTRE_ONLY, PARTIAL (alone_px ≥ min) | a partial-type class; the evaluator's exact centre verdict is required where the exact centre ray and the nearest-pixel ray agree (centre-robust) |

Two further checks hold for every fixture:

- A target-alone pass never produces occlusion evidence.
- Zero support never yields OCCLUSION_FULL_EVIDENCE.

**QUALIFIED** requires every gate.

## Inherited limitations

- s_l1c_v1 refuses genuine LiDAR sphere returns below 1.152 m. For centres between rows it also
  refuses them up to 2.295 m (S-L1C review addendum), so LIDAR_RANGE_ONLY is under-counted there.
- s_l1b_v1's carve keeps about 82% of target-centre returns as obstacles.
- GPU attitude passes are identical yaw-only inputs, so unique counts are a third of the totals.

## Declared limits

- The online state is only as good as the track prediction. In closed loop, a wrong prediction
  produces wrong inferences. That rate is reported against the evaluator, not assumed.
- Occluders closer than τ to the target surface are indistinguishable from the target in depth.
- A depth image cannot see full occlusion by an object behind which the target lies beyond the
  10 m obstacle far plane. Such cases become UNEXPLAINED_LOSS, not FULL.

## Execution

The canary is `SL1D-C01`.

| Process set | Content |
| --- | --- |
| Sensor processes | 2 (s_l1c, s_l1d), 43 envs, with primed-centre probes |
| Closed-loop processes | 8: static/moving × {off, s_l1c, s_l1d P1, s_l1d P2} |

The limits match S-L1B and S-L1C. A separate result commit, an independent review and a push of
`codex/mt-p1` only follow.
