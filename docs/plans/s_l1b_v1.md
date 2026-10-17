# S-L1B v1 — Camera–LiDAR pinhole / SE(3) calibration (preregistration)

Status: preregistered before any S-L1B runtime code or measurement. Base `2d65f1d` on `codex/mt-p1`.
Machine-readable contract: [`s_l1b_v1.json`](s_l1b_v1.json); it governs where the two differ.
Authority: the user's 2026-10-18 unattended S-L1B/C/D approval. Every step is gated, and no
criterion may be relaxed to reach a pass.

## Scope

`NAVRL_SENSOR_LINEAGE=s_l1b_v1` is cumulative. It keeps every S-L1A correction and adds one
change: camera depth enters the LiDAR scan through calibrated geometry. Unset and `off` select the
legacy path. `s_l1a_v1` keeps the qualified S-L1A path byte for byte. Any other value fails
closed. The RGB-D Camera + LiDAR contract, the 898-D actor input, Policy F and its RMS stay fixed.
The following are excluded and stay unchanged: tracker, covariance, LiDAR association, camera
target measurement, obstacle-token construction, detector rendering, E-L0, PPO, planner, motor
and governor.

The legacy fusion has three faults, all reproduced by EQ-1A:

- **A1:** it maps each LiDAR bearing to a camera column linearly in angle, which is 5.82 px or
  3.41° wrong at 20°.
- **A2:** it reuses the camera-origin range as a body-origin range, an on-axis error of −0.0998 m.
- **A3:** it copies one column minimum into all four LiDAR rows.

S-L1B replaces these steps as follows:

1. **Pinhole bearing.** Each pixel has its own unit ray, r̂ = normalize(1, −(u−cx)/fx, −(v−cy)/fy),
   with fx = W/(2·tan(hfov/2)) and cx = (W−1)/2. The same form gives the vertical terms.
2. **Common origin (SE(3)).** Each valid depth pixel becomes p = R·d·r̂ + t in the LiDAR frame,
   where d is the S-L1A slant range. The LiDAR frame is the yaw-only vehicle frame with its origin
   at the vehicle origin. The nominal extrinsic is t = (0.10, 0, 0.03) m and R = I. The code
   accepts any R and t, and CPU fixtures exercise non-identity values.
3. **LiDAR coordinates.** For each point, ρ = |p|, az = atan2(y, x) and el = atan2(z, √(x²+y²)).
   The azimuth bin is j = round((π − az)/Δ) mod H, matching the decreasing production bearing table.
   The elevation row is e = round((20° − el)/10°), so row 0 is +20°.
4. **Per-elevation geometry and conservative envelope.** These are two separate outputs:
   - The strict bins keep only points that fall inside a LiDAR row band.
   - The actor-facing envelope clamps points outside the vertical coverage into the edge rows.

   Each bin value is the minimum ρ over its valid points. The 2-D envelope, the minimum over rows
   that the token selector reads, therefore stays conservative.
5. **Fusion.** Every bin is fused as min(LiDAR after carve, clamped camera envelope). The old
   horizontal-FOV column gate is removed.
6. **Carve-out in the common frame.** S-L1A compared a camera-origin anchor with body-origin LiDAR
   ranges. S-L1B fixes that mismatch:
   - anchor_L is the minimum body range of the valid candidate pixels;
   - the predicted LiDAR proxy surface is s_L = anchor_L − (0.20 − 0.15);
   - the carve window is [s_L, s_L + 0.55), with no tolerance below s_L;
   - the angular limit is min(15°, asin(0.20/(s_L + 0.20))).

   S-L1A's fresh-camera, minimum-count and refusal rules, and its camera blank mask, are unchanged.

### Range and radius conventions (explicit)

| Quantity | Convention |
| --- | --- |
| Camera obstacle depth | Slant range from the camera origin along r̂. Values ≥ 10 m − 1e−3 mean no return. |
| Camera target depth | Existing renderer, far plane 20 m (unchanged). |
| LiDAR | Slant range from the vehicle origin. No return is 12 m; returns below 0.2 m arrive as 0 m (unchanged). |
| Fused bin without camera points | Contributes no information (12 m). |
| Target radius | Camera visual sphere 0.15 m. LiDAR proxy sphere 0.20 m (`navrl_lidar_config.target_radius`). Perception's `target_radius` is 0.15 m. |
| Centre-range conventions | Camera mean depth + 0.15 (A7 bias) and LiDAR surface + 0.15 (A4, −0.05 m) are **unchanged Known FAILs**. They belong to S-L1C's measurement model. |

The 0.05 m proxy difference is a declared sensor-model constant. It is not a measured value and
not a tolerance.

## Independent oracle and fixtures

[`tools/audits/s_l1b_reference.py`](../../tools/audits/s_l1b_reference.py) is a NumPy float64
oracle. It imports no production code and reuses the committed S-L1A oracle for ray/box/sphere
first hits. The CPU fixtures are:

| Fixture | Content |
| --- | --- |
| F1 | Front plane at body x = 5 m, on and off axis. |
| F2 | 0.1 m bars at ±5°, ±20° and ±40°, at 3 m and 6 m. |
| F3 | An overhang above +25° and a low box below −15°. |
| F4 | A changed extrinsic: t = (0.20, 0.05, −0.04) m, mount pitch 10°, yaw 5°. |
| F5 | The S14, A_FOREGROUND and B_TIGHT carve cases. B_TIGHT places a bar return at 2.77 m, inside S-L1A's window but in front of the 2.80 m proxy surface. |
| F6 | Validity and no-return conventions. |

The GPU canary adds seven bar scenes at the same bearings to the 36 S-L1A sensor scenes. GPU bars
are 0.6–0.8 m wide, so B_TIGHT cannot be realised there and stays a CPU-only fixture.

The robustness rules are:

- A pixel is robust when all eight ±0.01 px neighbouring labels agree.
- A bin is boundary-robust when its oracle value does not change with the bin boundaries shifted
  by ±1e−5 rad.
- Grazing pixels, where incidence exceeds 80°, are excluded from the per-pixel metric gate and
  reported.

## Gates (fixed before measurement)

| Gate | Requirement |
| --- | --- |
| B-POINT | On robust, non-grazing obstacle pixels: per-pixel body range within 1e−4 m and azimuth/elevation within 1e−5 rad of the analytic surface point. Applies to CPU F1–F4 and to the GPU delivered depth. |
| B-A1A2 | The 20° counterexample and the front-plane on/off-axis cases meet those tolerances. Legacy's 3.41° and −0.0998 m errors are reported, not gated. |
| B-ENVELOPE | Strict and clamped bins equal the independent float64 reprojection of the same depth within 1e−4 m on every boundary-robust bin, with 0 mismatches. Applies to CPU and GPU. |
| B-VERTICAL | Strict rows contain only in-band points. Out-of-coverage points appear only in the clamped edge rows. The row minimum stays conservative. |
| B-CARVE | Under s_l1b_v1, the bar returns of S14, A_FOREGROUND and B_TIGHT are never carved and never falsely cleared. No unsupported carve occurs. Masks equal the oracle. Target retention is reported. |
| B-DEPTH | s_l1b depth is bitwise equal to s_l1a depth, and the A-DEPTH thresholds hold. |
| B-REGRESSION | CPU: unset/off and s_l1a_v1 are bitwise equal to source `139184b`. GPU: off equals EL0-R1 P2, and s_l1a_v1 equals SL1A-C01 on_p2, on all 14 digest keys for static 4501 and moving 4502. |
| B-EL0 | s_l1b_v1 runs with E-L0 P1 and P2 give identical digests. |
| B-FIREWALL | Helpers accept only delivered sensor arrays and declared calibration constants. The actor width is 898. |
| B-INTEGRITY | Checkpoint, RMS, import origins, frozen tree and raw manifest are valid, with no NaN/Inf. |
| B-RESOURCE | Characterisation only. |

**QUALIFIED** requires every gate except B-RESOURCE. GO to S-L1C additionally requires an
independent review with no unresolved blocker. The following are reported but not gated:

- the per-beam offset between envelope and LiDAR truth (one-sided);
- changes to the scan and tokens;
- closed-loop changes when ON (no performance claim);
- EQ-1D D2-F1 under its original beam-exact definition. An envelope is not a beam metric, so the
  old definition is reported rather than reinterpreted.

## Execution

The canary is `SL1B-C01`, written to `../aerial_gym_simulator/results/s_l1b_v1`. It uses the
S-L1A orchestrator pattern:

- watchdogs: 900 s per process, 7,200 s in total;
- a GPU cap measured at launch, with a 512 MiB reserve;
- disk at least 8 GB free, and output at most 2 GB;
- no retry, and a stop on any non-interference failure.

The runs are:

- three sensor processes (legacy, s_l1a, s_l1b), with 43 envs and seed 4401;
- eight closed-loop processes covering static/moving × {off, s_l1a, s_l1b P1, s_l1b P2}, with
  N 8, 600 steps and density 160, at seed 4501 (static) and 4502 (moving).

Source and preregistration are committed before launch, and tracked files stay frozen while it
runs. Results go into a separate commit. ON results with Frozen F measure environment
sensitivity, not an algorithm benefit.
