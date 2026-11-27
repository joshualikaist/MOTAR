# S-L1B GPU qualification SL1B-C01 — 2026-10-18

**Verdict: `S-L1B_QUALIFIED`.** All ten verdict gates of the [preregistration](../../plans/s_l1b_v1.md)
pass: B-POINT, B-A1A2, B-ENVELOPE, B-VERTICAL, B-CARVE, B-DEPTH, B-REGRESSION, B-EL0, B-FIREWALL
and B-INTEGRITY. [Amendment 1](../../plans/s_l1b_v1_amendment1.md) applies. B-RESOURCE is a
characterisation only.

`NAVRL_SENSOR_LINEAGE=s_l1b_v1` is cumulative over S-L1A. It makes four changes:

- camera depth is reprojected through pinhole intrinsics and the SE(3) camera→LiDAR extrinsic;
- the result is binned per LiDAR elevation;
- it is fused into every bin as a clamped conservative envelope;
- the S-L1A carve-out is moved into the common frame.

Numbers come from [`s_l1b_analysis.json`](s_l1b_analysis.json). Re-running the analysis on the raw
output reproduces that file byte for byte, and the analysis imports no production code.

## Run

| Item | Value |
| --- | --- |
| Source commit (fixed during the run) | `a1f97e93af57f97b9778e395ffa8d38f7365e989` |
| Processes | 11 of 11 COMPLETED, every return code 0, no retry |
| Wall time | 432.1 s in total: sensors 10.2–10.3 s each, closed loop 46.7–52.1 s each |
| Peak GPU memory | 4,162 MiB (sensors) and 4,668 MiB (closed loop), under launch caps of 7,277–7,279 MiB |
| Disk | 19.1 GB free after the run; output 21.96 MB |
| Raw | `../aerial_gym_simulator/results/s_l1b_v1/SL1B-C01`, outside Git. `RAW_MANIFEST.json` SHA-256 `11aab182…`, 138 members, 0 mismatches |
| Policy F | `f7022139…` before and after |

## Calibration (B-POINT, B-A1A2, B-ENVELOPE, B-VERTICAL)

| Measure | Result |
| --- | --- |
| Per-pixel LiDAR-frame point versus the analytic surface point | 137,862 robust non-grazing obstacle pixels: max range error **6.9e−6 m**, max azimuth/elevation error **1.7e−7 rad**; 0 above 1e−4 m or 1e−5 rad. 1,770 grazing pixels were excluded as preregistered |
| Strict and clamped bins versus the float64 reprojection of the delivered depth | 148,608 boundary-robust bins, **0 mismatches**, max 1.8e−6 m. No non-robust bins and no validity mismatches |
| Fused scan versus min(LiDAR after the oracle carve, oracle envelope) | 74,304 bins, 0 mismatches, max 1.2e−6 m |
| Bearing scenes at ±5/±20/±40° and B_TWO (21 scene passes) | S-L1B azimuth bins equal the analytic bins in every pass (0 missing, 0 extra). Legacy misses 1–2 bins in 18 of 21 passes and adds a wrong bin in B_TWO |
| Vertical | Middle rows are identical in strict and clamped form. The clamped edge rows are ≤ strict. The 2-D envelope contains every valid point |

The CPU fixtures add the EQ-1A counterexamples, each measured against exact S-L1B:

- **A2 front plane:** legacy errs by −0.0998 m on axis.
- **A1 20° bar:** legacy fuses the bar into the wrong bins.
- **F4 rotated mount:** pitch 10°, yaw 5°, t = (0.20, 0.05, −0.04).

## Carve-out in the common frame (B-CARVE)

| Mode | Bar returns carved (843 bar beams) | Bar false-clear | Target returns kept as obstacle (102) |
| --- | --- | --- | --- |
| legacy | 156 | 12 | 6 |
| S-L1A | 0 | 0 | 18 |
| **S-L1B** | **0** | **0** | **84** |

In S14 and A_FOREGROUND, under every attitude, the foreground bar beams (24 and 20) are carved
24/24 and 20/20 by legacy and **0 by S-L1B**. The S-L1B masks match the independent oracle with 0
mismatches, and no unsupported evidence produced a carve.

The CPU fixture B_TIGHT is retained under S-L1B. It places a bar return 0.03 m in front of the
LiDAR proxy surface, which S-L1A's camera-frame window still carves (amendment 1 records this).

The trade-off is that S-L1B keeps more of the target as an obstacle: 84 of 102 target beams,
against 18 for S-L1A. In these beam-centred fixtures, the centre return D − 0.20 lies 1–8 mm below
s_L, because the camera anchor overestimates by its pixel-sampling margin. Zero tolerance below
s_L therefore keeps that return. This is the preregistered conservative choice, and it was not
tuned after the smoke run. S-L1A carved those returns only because its anchor was in the camera
frame, roughly 0.05 m short. The same offset is what lets S-L1A erase bar returns in that band.

## Depth and regression (B-DEPTH, B-REGRESSION, B-EL0)

- **Depth.** s_l1b depth is bitwise equal to s_l1a depth in every pass. A-DEPTH holds on
  3,710,988 robust non-target pixels: max 4.1e−5 m, 0 pixels above 1e−4 m, 0 false-clear and 0
  false-positive.
- **Closed-loop regression.** Each comparison below is bitwise identical over 14 digest keys and
  600/600 steps, at static 4501 and moving 4502:

  | Comparison | Result |
  | --- | --- |
  | off vs EL0-R1 P2 | identical |
  | s_l1a vs SL1A-C01 on_p2 | identical |
  | s_l1b E-L0 P1 vs P2 | identical |

  The actor input is 898-D in all eight closed-loop processes.

## ON behaviour change (distribution only, no claim)

| Config | Captures (off / S-L1A / S-L1B) | Crashes (off / S-L1A / S-L1B) |
| --- | --- | --- |
| static | 123 / 117 / 120 | 1 / 2 / 2 |
| moving | 93 / 95 / 89 | 9 / 5 / 3 |

These are single-seed N=8 runs. Frozen F under a changed input distribution measures robustness,
not improvement.

## Not gated (reported)

- **Per-beam offset.** The envelope minus the LiDAR beam truth over 807 in-FOV obstacle beams
  ranges from −4.13 to +0.0015 m, and 15 of those beams lie within 1e−4 m. The envelope is a
  conservative bin minimum, so the EQ-1D D2-F1 beam-exact definition is still not met by design.
- **Fusion cost.** Fusing 43 envs takes 17.2 ms median for S-L1B against 4.4 ms for S-L1A
  (+12.8 ms). Peak allocated CUDA memory in the sensor process is 97.4 MB for S-L1B against 84.6 MB
  for S-L1A and 82.8 MB for legacy.

## Integrity and preservation

[`preservation_after.json`](preservation_after.json) records PASS:

- runtime/test files: 2,071, of which exactly 20 S-L1A/S-L1B paths are allowed to change;
- historical evidence files: 101;
- raw manifests: 10, with 3,479 members;
- Policy F: both copies unchanged;
- main checkout: `cb644bb`, its three WIP files and stash `23ddb33` unchanged.

The SL1A-C01 manifest still verifies, 138/138. The tracked tree and HEAD stayed frozen during the
run.

## Known FAILs remaining

- A4 and A7 centre-range radius biases (assigned to S-L1C).
- Identity and range-only covariance (S-L1C).
- Silhouette and measurement validity (S-L1D).
- Target renderer residual.
- Dynamics items from EQ-1.

The envelope is not a beam metric. Fixture counts are constructed and are not population rates.
