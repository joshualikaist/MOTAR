# CAM-K2 depth-model CPU diagnostic

**Exploratory result; independent review pending.** The synthetic same-frame probe reproduces removal of near bars by the configured MinZ + invalid/no-return convention. It does **not** reproduce a blanket loss of surfaces at 0.5–1.5 m: all four literal range-stripe controls in that interval remain tokens with the model on. This is faithful to the documented model, not an established unintended implementation regression. Real OAK-D validity and training-population causality remain NOT_TESTED.

## What ran

- Worktree source `ceab4a0d4a1df75098ec3b2e5b99cce76b910f8d`, immutable. Script `probe.py`, fixed `PROTOCOL.md`, raw `metrics.json`, `frames.npz`, `rows.csv`, source and artifact hashes `manifest.json`.
- 63 deterministic synthetic scenes: 54 axis-aligned boxes and seven range stripes plus two no-return controls. Four camera-inside boxes are retained but excluded from the 50-scene descriptive counts. Twelve arms and two fixed depth seeds; 1,512 scene-arm-seed observations are not independent population samples.
- Analytic normalized-ray/AABB rendering at 40×24, then the renderer’s bilinear 160×90 upsample. Production `navrl_depth_model.apply`, `NavRLPerceptionModule._fuse_static_and_extract_obstacles`, and `bar_probe.associate_surface_tokens_to_bars` execute inside `eq1_common.production()`; CAM-K2 FOV overrides applied by production `camtrain_entry.apply_overrides`.
- No target, ground, other bars, detector carving, flight dynamics, policy, GPU or checkpoint loading. Synthetic renderer uses the source-matching ray/slab equations; production Warp renderer execution is NOT_TESTED here. The direct obstacle method isolates the requested depth→column minimum→scan→token path; full observe-wrapper integration is NOT_TESTED by this probe.
- Runtime: CUDA hidden, CPU 6/7, nice 10, one torch/OMP/MKL thread, 8 GiB virtual-memory cap and 600-second timeout. Host fault count 0→0. All nine pinned production/core-source files unchanged.

## Distance semantics and stage order

`barprobe hit_dist` is vehicle-origin to nearest bar **centre** in the horizontal plane. It is not optical surface depth. In this diagnostic a front-facing bar at body-centre distance 0.54 m with width 0.30 m has optical front-surface z = 0.54 − 0.10 (camera offset) − 0.15 (bar half-width) = 0.29 m, below MinZ 0.40 m. Widths 0.50 and 0.80 m put that surface at 0.19 and 0.04 m. The 0.80 m example may already intersect the vehicle body, but the camera remains outside; it is not presented as a feasible pre-contact state.

The depth model receives **160×90**, after low-resolution obstacle rendering and bilinear interpolation, not the raw 40×24 grid. Near valid returns are converted to optical z using the final image intrinsics; z<0.4 becomes invalid. Invalid becomes 20 m, columns with minimum≥10 m become LiDAR no-return 12 m, and token selection drops those bins. Thus the first removal in these examples occurs at depth-model MinZ invalidation, upstream of column minima and token capacity.

## Fixed-grid descriptive results

The table uses seed 4921. Seed 4922 gives the same association and empty-token counts. These counts describe the constructed grid only; they do not estimate the observed 0.68→0.19 training-log change.

| Arm | Bar associated / valid scenes | Empty-token scenes | Nonempty but unassociated |
| --- | ---: | ---: | ---: |
| `off` | 50/50 | 0 | 0 |
| `on` | 26/50 | 19 | 5 |
| `minz_off` | 50/50 | 0 | 0 |
| `holes_off` | 26/50 | 19 | 5 |
| `noise_off` | 26/50 | 19 | 5 |
| `maxz_off_identity` | 26/50 | 19 | 5 |
| `cosine_off` | 29/50 | 17 | 4 |
| `invalid_encoding_bypass` | 50/50 | 0 | 0 |
| `near_clamp_diagnostic` | 50/50 | 0 | 0 |
| `deterministic_on` | 26/50 | 19 | 5 |
| `deterministic_minz_off` | 50/50 | 0 | 0 |
| `neutral_identity` | 50/50 | 0 | 0 |

The six positive/negative controls all pass: neutral settings are bitwise input-identical; both no-return frames produce no token; all seven stripe frames produce an OFF token; 0.30/0.39 m stripes are erased ON; 0.50/0.75/1.00/1.50 m stripes survive ON; the inert MaxZ arm equals ON.

## Representative stage traces

| Scene | Input minimum | ON output minimum | ON scan minimum | ON tokens | MinZ-off scan minimum |
| --- | ---: | ---: | ---: | ---: | ---: |
| #3 centre=0.54, width=0.3, bearing=0.0 | 0.290104 | 1.524950 | 12.000000 | 0 | 0.289810 |
| #21 centre=0.54, width=0.5, bearing=0.0 | 0.190068 | 20.000000 | 12.000000 | 0 | 0.190043 |
| #39 centre=0.54, width=0.8, bearing=0.0 | 0.040014 | 20.000000 | 12.000000 | 0 | 0.040012 |
| #9 centre=0.75, width=0.3, bearing=0.0 | 0.500179 | 0.499202 | 0.499250 | 3 | 0.499250 |
| #0 centre=0.5, width=0.3, bearing=0.0 | 0.250089 | 1.493785 | 6.149966 | 2 | 0.249908 |
| #57 range stripe=0.5 | 0.500000 | 0.498160 | 0.498659 | 1 | 0.498659 |
| #60 range stripe=1.5 | 1.500000 | 1.486035 | 1.486332 | 1 | 1.486332 |

Meters throughout. A scan minimum of 12 m is the no-return sentinel, not measured clearance.

In five of the 24 ON association losses, bilinear transitions between near-bar ranges and background 10 m survive after the true near core is invalidated. The column/ray sampling subsequently selects inflated edge ranges (3.73–6.15 m in these examples), and the production association test rejects them. This is a composition effect of the pre-existing bilinear depth interpolation with MinZ, separate from complete disappearance. It is not a measured 1.09 m replay of the training statistic.

## Intervention meaning and limitations

- `minz_off`, `holes_off`, `noise_off`, and deterministic controls call the production model with one named setting changed. Disabling noise changes when holes consume random draws; this is not assumed perfectly paired. The deterministic ON/MinZ-off contrast gives the same loss/recovery pattern without this coupling.
- `maxz_off_identity` keeps MaxZ20 m: all obstacle inputs are≤10 m, so the MaxZ branch is already inactive. It is an identity control, not evidence from an independently active MaxZ mechanism.
- `cosine_off` is an external diagnostic cached-cosine override, not valid stereo geometry. It recovers three grid associations but leaves most losses. Correct range↔z conversion is necessary; removing it is not a proposed fix.
- `invalid_encoding_bypass` restores privileged pre-model depths at invalid pixels. `near_clamp_diagnostic` replaces known sub-MinZ pixels with a 0.4 m optical-z surface. Both are diagnostic upper bounds only: a physical invalid stereo return does not reveal the true missing distance or even the cause of invalidity. Neither is an authorized production patch.
- Near-clamp retains associations in this artificial grid, but that alone is not a safety claim or a reason to retrain. A deployable validity/unknown-depth contract needs calibration and must avoid supplying simulator-only reasons or depths to the actor.
- MinZ/no-return was disclosed in the original source and preregistration. These measurements establish a synthetic failure mode of that choice; they do not establish that the implementation violates its specification or that all missing hit tokens have this cause.
- Original training diagnostics may compare terminal vehicle/bar positions to previously produced obstacle tokens. The directional governor clearance and centre distance are different quantities and need time alignment before inference. This same-frame synthetic probe does not reconstruct those missing frames.

## Next discriminating work

Before v2 evaluation, explicitly decide whether the disclosed optimistic invalid-depth convention is the intended treatment. If it is retained, v2 evaluation can only characterize that fixed convention. If it is changed, qualify an additive validity-preserving sensor contract (default OFF) with no-return, too-near and mixed-edge controls, then fix the new protocol and make v3 training a separate user decision. A new recorded sensor/command/contact-aligned run is needed to estimate how much of the actual training-population loss this mechanism explains. No GPU execution is authorized by this report.
