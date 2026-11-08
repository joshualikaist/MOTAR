# CAM-K2 depth-model source and existing-log contract audit

Date: 2026-11-08 KST. Author: delegated Codex agent `/root/depth_contract`, inherited GPT-family model; exact backend variant NOT_RECORDED. This is a read-only, post-hoc descriptive audit, not a training/evaluation result or independent review of the sibling probe.

## Finding

The rounded training-log differences in the brief reproduce. They justify a controlled depth-path probe but do not establish that valid 0.5–1.5 m camera depths disappear. Three historical labels need additive correction:

1. `hit_dist` is the terminal horizontal distance from the robot origin to the **nearest bar center**, not true clearance or camera depth. The contact probe chooses the nearest center; it does not identify a named physical-contact collider. `navrl_task.py:3653–3672` and `bar_probe.py` define this.
2. `hit_fov` is **240-degree token-sector inclusion**, not 66-degree camera visibility. `navrl_task.py:3678–3689` uses `token_effective_fov_deg`; both CAM-K2 receipts specify `NAVRL_OBSTACLE_FOV_DEG=240` with `cluster_sector`. Therefore the old statement that 96–98% of contact bars were in camera view is unsupported by this field. Actual camera visibility at contact is NOT_RECORDED in these aggregate diagnostics.
3. The bar probe compares **pre-action tokens with post-physics bar positions**, without time-aligning them. The governor's contact clearance is likewise the **preceding requested-direction corridor clearance**, not same-time nearest depth. `step` transforms the action/governor, advances physics, computes termination/probes, then renders/resets and builds the next observation (`navrl_task.py:5011–5067`, `5165–5182`, `6331–6333`; accumulation `6162–6195`). Comparing the two logged distances as estimation error is invalid.

## Recomputed rounded log summaries

Arithmetic means of the first or last five completed diagnostic windows, as in the historical note. These are post-hoc windows from training with exploration, not paired frames, held-out evaluation or inferential replication. `metrics.json` retains each source log's SHA, line numbers, component rows, window counts and approximate contact-weighted alternatives. Printed fractions are already rounded, so their weighted means do not recover raw counters exactly.

| Run/window | Hit token | Token-sector inclusion | Center distance (m) | Previous requested-corridor clearance (m) |
|---|---:|---:|---:|---:|
| K1s1 last5 | 0.6768 | 0.9758 | 0.540 | 0.352 |
| K1s2 last5 | 0.7222 | 0.9814 | 0.542 | 0.356 |
| K2v2s1 first5 | 0.1868 | 0.9774 | 0.540 | 1.092 |
| K2v2s1 last5 | 0.1668 | 0.9620 | 0.540 | 1.090 |
| K2v2s2 first5 | 0.1824 | 0.9778 | 0.540 | 0.938 |
| K2v2s2 last5 | 0.1916 | 0.9814 | 0.542 | 1.094 |

## Production path and pinned settings

- Both K1 and K2 use the camera-only lineage authorized by D-2026-11-06-1; D-2026-11-08-1 replaces the 0.5 kg vehicle assumption with 1.1 kg and leaves the no-LiDAR decision standing. Frozen F remains a separate reference. These decisions do not authorize this agent to launch any GPU run.
- K2 v2 launcher: `train_navrl_camk2v2.sh:92–140`; receipt is authoritative for live settings. HFOV/VFOV 66/54 degrees, `cluster_sector`, 8 token slots, H/V bins72/4, token-sector240 degrees, perception perturbations OFF. Default camera resolution160×90; obstacle render40×24, 10 m far; target/detection far20 m. Camera translation `[0.10,0,0.03]` m (`navrl_task_config.py:183–208`, `228–231`).
- Renderer normalizes the camera rays before `mesh_query_ray`, so returned `t` is Euclidean ray range (`navrl_detector.py:28–49`, `372–405`). Range-to-optical-axis-depth conversion by cosine is dimensionally correct. A low-resolution range interpolated across surfaces is not a physically measured high-resolution ray range; converting that mixed value later can create artifacts.
- **Bilinear**40×24→160×90 interpolation, `align_corners=False`, occurs before depth-model application (`navrl_detector.py:927–940`). This is not nearest-neighbor interpolation. An early message from this audit mistakenly called it nearest; the source-derived finding here corrects it.
- The renderer then paints exact visible target depth at target-colored pixels (`navrl_detector.py:948–974`). `realveh_entry.py:189–193` installs the depth wrapper before the LiDAR ablation wrapper. The depth model acts on the160×90 delivered depth (`navrl_depth_model.py:146–179`, `234–255`), then perception detects/carves target-colored pixels before column minima (`navrl_perception.py:1910–1922`, `2289–2317`). Camera no-return columns are mapped to LiDAR no return, 72 angular bins are sampled using the historical linear-angle-to-column map, then cluster-sector tokens are selected. No oracle mask enters the actor path.
- K2 settings: MinZ0.4 m, MaxZ20 m, sigma-disparity0.15 px, fB66.1875 px·m, holes0.02, own module seeds4921/4922. MinZ/MaxZ decisions use clean `z=range*cosine`, not noisy depth. Invalid is20 m; renderer no-hit10 m±0.001 passes unchanged. Noise magnitude at z1 m is0.0022663 m; random holes are independently sampled on high-resolution pixels. Disabling noise also changes subsequent hole RNG position unless diagnostic interventions hold draws fixed.

## Geometry and mechanism hypotheses

The40 committed `bars_h3` collision boxes have x side0.4123–0.7883 m, y side0.4029–0.7902 m, height3 m (`metrics.json` binds every URDF). For an axis-aligned directly-ahead bar with center0.54 m from the body and camera offset0.10 m, the near-face optical depth is `0.54 - 0.10 - x_half`, about0.046–0.234 m. This is a geometric example, not reconstructed crash geometry; actual yaw, approach, and stale observation differ. The MinZ0.4 rule therefore has a plausible opportunity to invalidate the real near surface well before contact. A literal0.5 m input-depth stripe tests a different regime and should also be included as a valid-depth control.

Discriminating hypotheses for the sibling probe:

1. **MinZ plus optimistic invalid encoding:** core near-surface pixels go to no return. This behavior was explicitly disclosed in the original depth-model docstring/preregistration, not an accidental implementation departure. It can still be an unsuitable observation contract for near-obstacle control. No-return conflates invalid/unknown with far free space.
2. **Bilinear edge mixture:** even when core depth is belowMinZ, interpolation between that surface and10 m background produces artificial farther edge depths aboveMinZ. Column minima/bin sampling may retain these and inflate represented range. Compare production-bilinear/no-bilinear frames, then MinZ OFF, while identifying core vs mixed pixels.
3. **Not intrinsically noise/holes/MaxZ:** small near-range noise and independent2%holes should not erase all pixels of a thick vertical bar; MaxZ20 should be inert for a0.5–1.5m input. Test one-at-a-time controls rather than assuming this.
4. **Historical fusion/selector geometry:** pinhole-vs-linear column indexing, broad240-degree token eligibility versus66-degree camera coverage, sparse72-bin sampling, clustering, target carve-out, and limited visibility can produce representation failure even with depth model OFF. Use no-target synthetic frames first, then target-mask positive controls if extending scope.
5. **Diagnostics and plant confounds:** post-physics matching, changing relative pose and motor/TW treatments can alter the aggregate contact sample. The two trainings are not paired same-scene treatments, and the first K2 windows already include continuation updates. A depth-path CPU result isolates representation behavior; it cannot attribute the whole training crash difference.

## Existing depth telemetry and limitations

The detailed invalid statistics cover only the first50 calls (6400 frames), not all contact events or training. Seed1/2 MinZ invalid fractions are0.0377495275/0.0374837362; hole fractions0.0200346491/0.0200121770; MaxZ fractions both0. The global pixel fraction can be small while contact-critical pixels are lost. Later lines are heartbeat call counts only. The training smoke accepts MinZ invalid fraction below0.5 and verifies finite noise/settings; it never asserts contact-bar token retention (`camk2v2_train.py:379–402`).

Restoring true below-MinZ simulator depth is a diagnostic ablation, not a deployable sensor correction. A prospective correction must decide how invalid depth is represented (unknown/occupied/temporal state), which sensor-valid signals may support it, and what that changes in the policy's training contract. Do not silently re-evaluate K2v2 with a repaired sensor and call it its original trained condition; keep original checkpoints/results immutable. No GPU/v3 execution is authorized by this report.

## Validation and artifacts

`extract_contract.py` used only Python stdlib, streamed four logs, paired first/last-five bar and governor contact counts exactly, parsed all40bar URDFs, and bound16 production/config/authority files. `write_report.py` verifies those source hashes stayed unchanged and records both actual run receipts. `supplement.json` records host fault counts0→0, CPU limits, and repository status; no tracked file was edited by this worker. Checkpoint tensors were not loaded. Source verification is an audit of this CPU unit, not independent validation of the synthetic probe. Use `metrics.json`, `supplement.json`, and `manifest.json` to rederive this report.
