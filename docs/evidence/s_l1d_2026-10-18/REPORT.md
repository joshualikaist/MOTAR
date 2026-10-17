# S-L1D GPU qualification SL1D-C01 — 2026-10-18

**Verdict: `S-L1D_QUALIFIED`.** All eight verdict gates of the [preregistration](../../plans/s_l1d_v1.md)
pass: D-ORACLE, D-FIXTURES, D-SEPARATION, D-REACQ, D-REGRESSION, D-EL0, D-FIREWALL and
D-INTEGRITY.

`NAVRL_SENSOR_LINEAGE=s_l1d_v1` is cumulative over S-L1A, S-L1B and S-L1C. It adds a
sensor-inferable online validity state as evaluator-facing diagnostics. The state is computed from
three things only: delivered depth, the camera detection flag, and the track's own posterior and
covariance. It does not change the actor input or the tracker: the s_l1d closed loop is bitwise
equal to s_l1c. Evaluator truth remains the E-L0 classes.

Numbers come from [`s_l1d_analysis.json`](s_l1d_analysis.json). Re-running the analysis on the raw
output reproduces that file byte for byte, and the analysis imports no production code.

## Run

| Item | Value |
| --- | --- |
| Source commit (fixed during the run) | `76f0f4cc08ceecf9a05e628ea82d956123d29524` |
| Processes | 10 of 10 COMPLETED, every return code 0, no retry |
| Wall time | 432.0 s in total; each process 10.2–57.3 s |
| Peak GPU memory | 4,670 MiB, under launch caps of 7,277–7,283 MiB |
| Disk | 19.0 GB free after the run; output 18.56 MB |
| Raw | `../aerial_gym_simulator/results/s_l1d_v1/SL1D-C01`, outside Git. Manifest SHA-256 `2b5fd37e…`, 131 members, 0 mismatches |
| Policy F | `f7022139…` before and after |

## Fixture agreement (D-ORACLE, D-FIXTURES)

The sensor fixtures are the 43 S-L1B scenes. Each scene pass and each target-alone pass is probed
with the track primed at the true centre, which is fixture input only.

- **Support (D-ORACLE).** The online predicted support equals both the E-L0 `alone_px` and the
  independent float64 oracle in every probed pass: 0 mismatches of each kind, so the
  boundary-ambiguity allowance was never needed.
- **Gated cases (D-FIXTURES).** There are 30, with **30/30 agreement**:

  | Evaluator class | Scenes | Cases | Online class required and produced |
  | --- | --- | --- | --- |
  | RANGE_EXIT | S08 | 6 | PRED_RANGE_EXIT |
  | FOV_EXIT | S09, S10 | 12 | PRED_FOV_EXIT |
  | FULL | S11, S16, S17, S19 | 12 | OCCLUSION_FULL_EVIDENCE |

- **Other fixture properties.**
  - No occluder lay within τ, so nothing was set aside as "reported only".
  - Target-alone passes produced no occlusion evidence: 0 cases.
  - Zero support never produced FULL evidence: 0 cases.
- **Where the evidence comes from.**
  - On GPU, every CENTRE_ONLY (9) and PARTIAL (3) fixture pass was *detected* by the camera, so
    the mapping did not constrain them there.
  - The online classes OCCLUSION_CENTRE_EVIDENCE and OCCLUSION_PARTIAL_EVIDENCE, and the
    scale-loss precedence, are evidenced by the CPU oracle fixtures (`tests/test_s_l1d_reference.py`
    and `tests/test_s_l1d_runtime.py`).
  - The three attitude passes are identical yaw-only inputs, so the 30 gated cases are 10 unique
    measurements.
- **The EQ-1D D2-O1 disagreements, under the new separation.**
  - **S08:** reads PRED_RANGE_EXIT, which agrees with RANGE_EXIT; the legacy indicator read FOV.
  - **S14 rim gap:** E-L0 classifies it CENTRE_ONLY, and the camera detects it, so there is no false FULL.
  - **S12 partial with the centre visible:** E-L0 classifies it PARTIAL, and the camera detects it.

## Closed loop: online state against evaluator truth (reported, not gated)

These are evaluator-only traces from N8, 600-step runs (P2; P1 is identical).

| Evaluator class (static) | Online states |
| --- | --- |
| VISIBLE (3,760) | DETECTED 3,744 · UNEXPLAINED_LOSS 12 · PRED_SCALE_LOSS 3 · PRED_FOV_EXIT 1 |
| FULL (158) | OCCLUSION_FULL_EVIDENCE 128 · OCCLUSION_CENTRE_EVIDENCE 22 · OCCLUSION_PARTIAL_EVIDENCE 4 · PRED_FOV_EXIT 4 |
| FOV_EXIT (741) | PRED_FOV_EXIT 491 · UNEXPLAINED_LOSS 244 · PRED_SCALE_LOSS 6 |
| CENTRE_ONLY (63) | DETECTED 56 · OCCLUSION_CENTRE_EVIDENCE 4 · OCCLUSION_PARTIAL_EVIDENCE 2 · OCCLUSION_FULL_EVIDENCE 1 |

In the moving configuration, FULL (437) splits into FULL evidence 300, CENTRE 70, PARTIAL 32,
UNEXPLAINED 30 and FOV 5. FOV_EXIT (810) splits into PRED_FOV_EXIT 545 and UNEXPLAINED 251, plus
a few others.

In closed loop the track prediction drifts, so the online state is only as good as that prediction.
That is why some FOV exits read UNEXPLAINED_LOSS and some full occlusions read partial.

| Reacquisition (D-REACQ) | static | moving |
| --- | --- | --- |
| Identity-confirmed reacquisitions (camera after an identity age > 0.5 s) | 38 | 76 |
| GT camera-measurement error at reacquisition, median / max | 0.069 / 0.188 m | 0.079 / 0.176 m |
| Fused-valid returns (LiDAR range-only after a fused loss) | 73 | 72 |

## Regression and E-L0 (D-REGRESSION, D-EL0)

Each comparison below is bitwise identical over 14 digest keys and 600/600 steps, at static 4501
and moving 4502:

| Comparison | Result |
| --- | --- |
| off vs EL0-R1 P2 | identical |
| s_l1c vs SL1C-C01 s_l1c_p2 | identical |
| s_l1d P2 vs s_l1c in the same run (diagnostic only) | identical |
| s_l1d P1 vs P2 | identical |

On CPU, the off, s_l1a, s_l1b and s_l1c `observe` outputs are bitwise equal to `4d5911d`. The
actor input is 898-D in all eight closed-loop processes.

## Integrity and preservation

[`preservation_after.json`](preservation_after.json) records PASS:

- runtime/test files: 2,085, of which exactly 34 S-L1A/B/C/D paths are allowed to change;
- historical evidence files: 101;
- raw manifests: 10, with 3,479 members;
- Policy F: both copies unchanged;
- main checkout: `cb644bb`, its three WIP files and stash `23ddb33` unchanged.

The SL1A-C01, SL1B-C01 and SL1C-C01 manifests still verify (138, 138 and 176).

## Limits

- **Diagnostic only.** S-L1D does not change Policy F's inputs. Using the online state for control,
  for example a Recovery planner, is not part of this unit.
- **Occlusion classes on GPU.** OCCLUSION_CENTRE_EVIDENCE and OCCLUSION_PARTIAL_EVIDENCE on
  undetected fixtures are CPU-only evidence. The GPU gated set is 10 unique cases.
- **Depth tolerance.** Occluders within τ, or beyond the 10 m obstacle far plane, cannot be
  evidenced by depth.
- **Inherited limitations:**
  - S-L1C refuses LiDAR association below 1.152 m, and for centres between rows up to 2.295 m;
  - the S-L1B carve keeps about 82% of target-centre returns;
  - the camera A7 centre-range bias is unchanged.
- **Population rates.** Fixture counts are constructed. Closed-loop rates come from single-seed
  N=8 runs and support no claim.
