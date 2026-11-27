# S-L1A GPU qualification SL1A-C01 — 2026-10-18

**Verdict: `S-L1A_QUALIFIED`.** All six verdict gates of the [preregistration](../../plans/s_l1a_v1.md)
pass: A-DEPTH, A-BOUNDARY, A-CARVE, A-OFF, A-EL0 and A-INTEGRITY. Amendment 1 applies. A-RESOURCE
is a characterisation only.

S-L1A is an opt-in sensor lineage (`NAVRL_SENSOR_LINEAGE=s_l1a_v1`); its default is OFF. It
qualifies two environment corrections and nothing more:

- **A.** Obstacle depth is ray-cast on the actual 160×90 camera pixel rays.
- **B.** The target-like LiDAR carve-out keeps every nearer real obstacle return.

It is not an algorithm improvement. The Known FAILs listed below remain open.

Numbers come from [`s_l1a_analysis.json`](s_l1a_analysis.json). Re-running
`tools/s_l1a_analysis.py` on the raw output reproduces that file byte for byte. Compact run
records are in [`run_records/`](run_records/).

## Run

| Item | Value |
| --- | --- |
| Source commit (fixed during the run) | `139184bc0d2a7aa855fe8ab5a2a7f547702defa5` |
| Processes | 11 of 11 COMPLETED, every return code 0, no retry |
| Wall time | 421.4 s total: sensors 10.2 s each, closed loop 46.7–52.1 s each |
| Peak GPU memory | 4,164 MiB for sensors, 4,662 MiB for closed loop; launch caps 7,008–7,026 MiB (measured free memory minus 512) |
| Disk | 19.2 GB free before and after; output 20.06 MB |
| Raw | `../aerial_gym_simulator/results/s_l1a_v1/SL1A-C01`, outside Git; `RAW_MANIFEST.json` SHA-256 `cee40f01…`; 138 members, 0 mismatches |
| Policy F | `f7022139…` both before and after the run |

## A — pixel-ray obstacle depth (A-DEPTH, A-BOUNDARY)

The fixtures are the 36 EQ-1D D2 scenes plus the four preregistered `A_*` scenes, each under
three attitudes, with a scene pass and a target-alone pass. Errors are measured against the
independent float64 oracle on robust non-target pixels: 3,106,404 pixels, of which 115,176 are
obstacle hits; 798 non-robust edge pixels are excluded.

| Mode | Max error (m) | Mean (m) | p99 (m) | Pixels > 1e−4 m | False-clear | No-hit false-positive |
| --- | --- | --- | --- | --- | --- | --- |
| legacy (40×24 bilinear) | 7.30 | 0.0292 | 0.866 | 134,280 | 23,394 | 18,978 |
| S-L1A depth only | 4.1e−5 | 3.4e−8 | 1.1e−6 | 0 | 0 | 0 |
| S-L1A full | 4.1e−5 | 3.4e−8 | 1.1e−6 | 0 | 0 | 0 |

Each of the 12 `A_*` scene rows passes on its own. Under every attitude, S-L1A has a maximum error
of at most 3.4e−6 m and zero false-clear and zero false-positive pixels.

| Scene | Legacy false-clear / false-positive (each attitude) | Legacy max error (m) |
| --- | --- | --- |
| `A_EDGE` | 180 / 562 | 4.65 |
| `A_TWO_DEPTH` | 553 / 408 | 7.12 |
| `A_THIN` | 136 / 188 | 1.70 |
| `A_FOREGROUND` | 742 / 682 | 6.94 |

Paired inputs:

- RGB, LiDAR, the native 40×24 buffer and the target mask are bitwise identical to legacy.
- Composed depth changes in 135,264 of 3,110,400 pixels, with a mean absolute change of 0.030 m
  and a maximum of 7.82 m.
- The target-pixel residual is reported separately (Known FAIL, legacy target renderer): at most
  3.3e−4 m over 3,198 robust target pixels. The all-pixel maximum, including non-robust edges, is
  5.5e−3 m.

## B — conservative carve-out (A-CARVE)

LiDAR first-hit labels come from an independent oracle and are confirmed by semantics:
693 bar beams and 102 target beams.

| Carve | Bar returns removed | Bar false-clear | Target returns kept as obstacle |
| --- | --- | --- | --- |
| none (no camera evidence) | 0 | 0 | 102 / 102 |
| legacy (with S-L1A depth) | 156 | 12 | 6 / 102 |
| S-L1A fixed | **0** | **0** | 18 / 102 |

The S14 and `A_FOREGROUND` foreground bars are checked under every attitude:

- The legacy carve removes 24 of 24 S14 bar beams and 20 of 20 `A_FOREGROUND` bar beams, with 4
  false-clear beams in `A_FOREGROUND`.
- S-L1A removes none of them.

The fixed carve is also checked against independent masks:

- Fixed carve masks match the independent mask oracle exactly: 0 mismatches.
- No unsupported evidence authorizes removal: 0 uncertain carves.
- In the legacy-depth sensor mode the legacy carve's bar false-clear count is 48. This is reported
  as a distribution measure only.

The trade-off is that S-L1A keeps 18 instead of 6 target returns as obstacles. That outcome is
conservative and preregistered.

## OFF non-interference (A-OFF) and E-L0 (A-EL0)

The comparison uses N=8 environments, 600 steps and 160 bars, over 14 digest keys: obs, action,
reward, done, captured, crashed, pursuer, target, resets, rgb, depth, lidar, semantic and tracker.

| Config (seed) | unset vs OFF | OFF vs historical EL0-R1 P2 | ON E-L0 P1 vs P2 |
| --- | --- | --- | --- |
| static (4501) | identical 600/600 | identical | identical |
| moving `E2_direct_l2_v1` (4502) | identical 600/600 | identical | identical |

The step-log SHA-256 of the static OFF run equals the historical value `229f0c17…`. In the default
OFF path, Policy F's 898-D input, its actions, trajectories, outcomes and the sensor/tracker
streams are therefore unchanged.

## ON behaviour change (distribution only, no claim)

| Config | Captures OFF → ON | Crashes OFF → ON | Max position delta (m) | Max action delta |
| --- | --- | --- | --- | --- |
| static | 123 → 117 | 1 → 2 | 43.7 | 1.95 |
| moving | 93 → 95 | 9 → 5 | 40.6 | 1.98 |

These are single-seed N=8 runs. ON changes Policy F's sensor input distribution, so these counts
measure robustness under a new environment. They are not evidence of improvement or regression.

## A-RESOURCE (characterisation)

| Mode | Render median (ms) | Render p95 (ms) | Allocated vs legacy |
| --- | --- | --- | --- |
| legacy | 0.292 | 0.431 | — |
| S-L1A full | 0.318 (+0.026) | 0.338 | +2.06 MB |

Each mode was timed with 10 warmups and 50 CUDA-synchronized repeats in one benchmark scene. This
is not a performance claim.

## Integrity and preservation

[`preservation_after.json`](preservation_after.json) records PASS:

- runtime/test files: 2,064, of which exactly 13 S-L1A paths are allowed to change;
- historical evidence files: 101;
- raw manifests: 10, with 3,479 members;
- Policy F: both copies unchanged;
- main checkout: HEAD `cb644bb`, its three WIP files and stash `23ddb33` unchanged.

The tracked tree and HEAD stayed frozen during the run.

One integrity bug was found before any GPU work. Official preflight at `d7bdca6` crashed on a
relative `__file__`. It was fixed in `139184b`, a one-line change with a regression test, before
launch.

## Known FAILs (not addressed by S-L1A)

- S-L1B: pinhole bearing extraction and SE(3) camera pose.
- S-L1C: target identity and range-only covariance.
- S-L1D: silhouette and validity.
- The camera–LiDAR origin and target-radius mismatch.
- Target renderer float32 residual.
- Dynamics items from EQ-1.

The conservative inequality is not a universal guarantee of foreground or identity. Fixture
counts are constructed and are not population rates. Recovery hypotheses H-R1/H-R2/H-R3 remain
provisional and are not preregistered. No PPO, planner, motor or governor change was made.
