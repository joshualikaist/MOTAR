# M1 — Online perception chain: function-level I/O contract for an occlusion-handling planner

Read-only audit, 2026-10-19. Repository root: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1`
(worktree, branch `codex/mt-p1`, HEAD `1d4696c`). All `path:line` citations are relative to that
root unless prefixed `../`. No file in either repository was edited. No simulator or GPU code was run.
The only code executed was a stdlib-only arithmetic check of the segmenter scores and intrinsics
(§2.2).

Labels used below:
- **[code]**: read directly from source at HEAD.
- **[evidence]**: quoted from a committed evidence record.
- **[audit]**: a new observation from this audit. I found no committed record of it, and it is not measured.

---

## 0. Operating point (Frozen Policy F evaluation contract)

Policy F `f7022139…` is evaluated under the OCC-P2 receipt
`../aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2/static/A1/E0_static__115bars__seed4301/receipt.json`
(the `env_snapshot_navrl` block). Its relevant values:

| Knob | Value | Effect on this chain |
| --- | --- | --- |
| `NAVRL_VISION=1`, `NAVRL_PERCEPTION=1` | on | detector + `NavRLPerceptionModule` path (`navrl_task.py:1413-1416`, `1446-1496`) |
| `NAVRL_LIDAR_HBEAMS/VBEAMS/RANGE` | 72 / 4 / 12 m | 288-beam scan, `STATIC_DIM = 288` |
| `NAVRL_MAX_OBSTACLES`, selector, token FOV | 8, `cluster_sector`, 240° | 480-D obstacle history |
| `NAVRL_PERCEPTION_PERTURB` | unset → **False** | `training=False` (`navrl_task.py:6477`): no RGB/depth noise, no dropout, no range error (`navrl_perception.py:1593-1598, 1640-1641, 1707-1710`) |
| `NAVRL_DETECTOR_CHECKPOINT` | empty | bootstrap 1×1 colour segmenter (`navrl_perception.py:1027`) |
| `NAVRL_DETECT_WIDTH/HEIGHT` | unset → camera resolution | **no** detect-resolution decoupling |
| `NAVRL_LIDAR_RANGE_ONLY_UPDATE`, `NAVRL_SENSOR_LINEAGE` | unset | legacy LiDAR association, legacy fusion |
| `NAVRL_SPEED_GOVERNOR` | `riskcap` | online command filter that reads the LiDAR plus `perception.last_target_like` (`navrl_task.py:5248-5292`) |
| `NAVRL_TARGET_DYNAMICS` | `bounded` (not `physical`) | target is the analytic sphere (oriented box only in `physical`) |

Policy F was also trained with perturbations off (`aerial_gym/rl_training/rl_games/train_navrl_v2_search.sh:353`).
The training contract is in `docs/audits/frozen_policy_training_contract_2026-09-17.md:64-77`.
**The whole Policy F chain is noise-free in both training and evaluation.** The configured noise
values (dropout 0.3, depth σ 0.02, RGB σ 0.015) are dormant.

---

## 1. Sensors

### 1.1 RGB-D camera

The camera is **not** an Aerial Gym camera sensor. `navrl_quad` sets `enable_camera = False` and
`enable_lidar = True` (`aerial_gym/config/robot_config/navrl_quad_config.py:42-46`). The RGB-D frame is
rendered by `NavRLTargetDetector`'s own Warp kernels (`navrl_detector.py:1-15`). **None of the files
under `aerial_gym/config/sensor_config/camera_config/`, including the D455 configs, is used by this
chain.** Its parameters are in `aerial_gym/config/task_config/navrl_task_config.py` `class vision`.

| Property | Value | Citation |
| --- | --- | --- |
| Resolution | 160×90 (env `NAVRL_CAMERA_WIDTH/HEIGHT`) | `navrl_task_config.py:183-184` |
| FOV | H 87°, V 58° | `navrl_task_config.py:171-172` |
| Intrinsics | `fx = W/(2 tan(hfov/2)) = 84.302`, `fy = H/(2 tan(vfov/2)) = 81.182`, `cx = 79.5`, `cy = 44.5`. Pixels are not square; there is no distortion. | `navrl_perception.py:758-761`; `navrl_detector.py:342-345` |
| Ray table | `[1, −(u−cx)/fx, −(v−cy)/fy]` normalised. Camera frame: +x forward, +y left, +z up; u grows right, v grows down. | `navrl_detector.py:368-380` |
| Extrinsic | translation (0.10, 0, 0.03) m in the **vehicle** frame, rotation = I | `navrl_task_config.py:208`; `navrl_detector.py:555-559`; `navrl_perception.py:784-786`; S-L1B `R = I`: `navrl_perception.py:791-794` |
| Mount attitude | Renderer origin = `drone_pos + R(q_veh)·offset`, orientation = `q_veh`. `q_veh` is `robot_vehicle_orientation`, which is **yaw-only** (`navrl_task.py:6354-6355, 6459-6463`). **Optical axis = vehicle (heading) +x, not body x.** This is an ideal level gimbal. A body-fixed camera would shift 21.8 px at 15° pitch [evidence: EQ-1D D2-Y1, `docs/evidence/eq1d_qualification_2026-10-18/REPORT.md`]. | `navrl_detector.py:820-831` |
| Target depth encoding | Euclidean **slant range** along the unit pixel ray (Warp `mesh_query_ray` `t` with normalised `rd`), not planar z-depth. Analytic sphere (r = 0.15 m) with exact mesh occlusion. Far plane `detector_max_range` = 20 m. | `navrl_detector.py:53-132`, `navrl_task_config.py:170, 206` |
| Obstacle depth (legacy, Policy F) | Native **40×24** ray-cast, far plane `camera_obstacle_max_range` = 10 m (no-hit = 10 m), **bilinearly upsampled** to 160×90, then the target pixels are pasted over it | `navrl_detector.py:28-49, 874-886, 927-938, 948-949, 969-974`; `navrl_task_config.py:229-231` |
| Obstacle depth (S-L1A+) | Native 160×90 obstacle ray-cast on the actual camera rays. The RGB shading still uses the 40×24 buffer. | `navrl_detector.py:452-458, 887-894, 948-949` |
| RGB | Background luminance `0.08 + 0.42·proximity`, tint (0.92, 1.00, 1.05). Target pixels are painted flat (0.88, 0.08, 0.045). Appearance knobs (hue, light, albedo, texture, blur) default to 0. | `navrl_detector.py:573-582, 940-947, 967-974`; `navrl_task_config.py:216-220` |
| Rate / latency | Rendered once per RL step (0.1 s, 10 Hz; physics dt 0.01 s × 10), inside `_process_obs_perception`. Capture step = delivery step (0-step latency) [evidence: EQ-1D D2-T1, D3-T1]. `detection_latency_s` = 0. | `navrl_task.py:6450-6465`; `navrl_task_config.py:280` |
| Noise | Policy F: **none**. Opt-in only: per-pixel RGB σ 0.015 and depth σ 0.02, only when `training=True`; Markov dropout; AR(1) range noise (`NAVRL_DETNOISE_*`); P9 empirical model; mount, FOV and pose perturbations. | `navrl_perception.py:1593-1598, 1336-1403, 951-952`; `navrl_detector.py:358-366, 822-831` |
| Filter's assumed camera R (not injected noise) | `σ_r = 0.04 + 0.012 r + 0.15/√px` (or the opt-in "stereo" r² term), `σ_lat = 0.03 + r/fx` | `navrl_perception.py:2222-2234` |

### 1.2 LiDAR

| Property | Value | Citation |
| --- | --- | --- |
| Beams | 72 azimuth × 4 elevation. Azimuth bin j bearing = `π − j·5°`, so **decreasing**: bin 0 = +180°, bin 71 = −175°. Rows i = 0..3 → elevation **+20°, +10°, 0°, −10°**. | `navrl_lidar_config.py:46-61`; `warp_lidar.py:68-89`; `navrl_perception.py:196-213` |
| Range | min 0.2 m, max 12 m (env). `>max` → `max`, `<min` → `−max`, then divided by max. | `navrl_lidar_config.py:64-72`; `warp_sensor.py:220-245` |
| Mount | Body origin, no offset, yaw-only attach | `navrl_lidar_config.py:75, 93-100`; `warp_sensor.py:184-205` |
| Target | Analytic sphere **r = 0.20 m** (camera uses 0.15 m), semantic id 50. Centre taken from `navrl_target_position`, which `_sync_target_to_sensor` mirrors from `task.target_position` every step. | `navrl_lidar_config.py:82-90`; `warp_sensor.py:147-151`; `navrl_task.py:1301, 7557-7563` |
| Rate / noise | Once per env step, rendered after physics (10 Hz). `enable_sensor_noise = False`: noise-free, float32 kernel residual ≤ 1.25e-4 m [evidence: EQ-1D D2-L1]. | `navrl_lidar_config.py:102-108`; `warp_sensor.py:213-218` |

**How `depth_range_pixels` is produced.** The kernel `draw_optimized_kernel_range_segmentation_target`
(`aerial_gym/sensors/warp/warp_kernels/warp_lidar_kernels.py:167-251`) works per ray:
1. A mesh ray query against the env mesh gives `dist` (line 203).
2. An analytic ray–sphere (or oriented-box) test against the GT target is run. The target overrides `dist` and writes semantic 50 if it is the nearer surface (lines 239-251).
3. `pixels` and `segmentation_pixels` are written (lines 250-251).

`WarpSensor.update` then:
- skips noise (disabled);
- applies range limits;
- normalises by `max_range` (`warp_sensor.py:213-245`).

The buffer `obs_dict["depth_range_pixels"]` has shape (N,1,4,72) and values in [−1,1]
(`robot_manager.py:229`). The task converts it with `_lidar_distance_m()`
(`navrl_task.py:6013-6017`): NaN/inf → 1, clamp [0,1], × 12 m → **(N,288) metres, row-major [row][bin]**.
Near returns below 0.2 m therefore read **0 m**.

---

## 2. Target detection

### 2.1 Decision rule (Policy F path, `_detect_rgbd`, `navrl_perception.py:1579-1760`)

1. `score = segmenter(rgb, depth, 20)` (line 1600). The bootstrap head is a 1×1 conv,
   `sigmoid(3R − 2G − 2B − 0.9)`, with depth weight 0 (`navrl_perception.py:613-633`, weights at 624-629).
2. `mask = (score ≥ 0.55) & (depth < 20)` (line 1601). The threshold is set at `navrl_task_config.py:274`.
3. `count = mask.sum`; `visible = count ≥ min_target_pixels = 2` (lines 1607-1608; `navrl_task_config.py:273`).
   The detector-side `camera_min_target_pixels = 1` is used only by the legacy `detect()`.
4. Opt-in only: dropout (training), Markov miss (DETNOISE) or P9 (lines 1640-1690).
5. `mask &= visible` (line 1691). Centroid `u, v` = **unweighted mean of mask pixel coordinates**;
   `surface_range` = mean mask depth (lines 1703-1706).
6. `confidence = mean(score over mask) × min(1, count/8)`, set to 0 if not visible (lines 1732-1734).
7. Back-projection (lines 1738-1749):
   - `ray = normalise([1, −(u−cx)/fx, −(v−cy)/fy])`;
   - `centre_range = surface_range + 0.15`;
   - `meas_vehicle = camera_offset + ray·centre_range`;
   - `bearing = atan2(meas_y, meas_x)` about the body origin.
8. The latency ring is a no-op at 0 latency (`navrl_perception.py:1221-1235`).
9. In `observe()`: `meas_world = drone_pos + R(q_veh)·meas_vehicle` (line 2214).

### 2.2 "Detects the target from appearance": claim versus implementation

**Structurally**, the claim holds. The module docstring (`navrl_perception.py:1-9`) says perception
receives only `rgb` and `depth`. No semantic id, mask or target pose is an argument of `observe()`
(`navrl_perception.py:2184-2199`; call site `navrl_task.py:6465-6480`). The renderer's `target_mask`
stays private (`navrl_detector.py:916-924`). This is verified at task level by EQ-1D D3-F1..F4 and
EQ-1A C1a/C1b [evidence].

**Informationally**, the claim does not hold: the detector is an oracle-grade silhouette.

- **How the RGB is made.** The renderer ray-casts the GT target sphere with exact mesh occlusion
  (`navrl_detector.py:110-132`). It paints exactly those pixels a unique flat red (`navrl_detector.py:969-974`).
  Every other pixel is a grey/blue-tinted luminance ≤ 0.50.
- **Bootstrap scores (stdlib arithmetic check, this audit).**
  - Target pixel: logit 1.49 → **score 0.816** (≥ 0.55).
  - Background: logit = −1.34·L − 0.9 → score **0.172–0.268** for L ∈ [0.08, 0.50].
  - So the "appearance" segmentation reproduces the renderer's GT target mask bit for bit. This is
    no learned or statistical inference.
- **Committed evidence confirms the identity:**
  - D2-S1: detector visibility equals analytic pixel count ≥ 2.
  - D2-N1: 9/9 bar-only scenes, no detection.
  - E-L0 E0-L3: TRUE_TARGET exactly where the detector reports, no FALSE_DETECTION.
  - The config itself says "the pixel classes are trivially separable" (`navrl_task_config.py:209-213`).
- **No segmentation image enters as input.** The RGB frame is, in effect, a one-hot encoding of the
  occlusion-aware GT mask (pixel-centre sampled at 160×90) plus exact float32 depth.
- **Opt-in decoupled detection (`NAVRL_DETECT_WIDTH > camera`) is a more direct oracle channel.**
  - Perception consumes `DETECT_CHANNEL` (`navrl_perception.py:1551-1577, 1615-1639`), which carries
    `count`, `u_sum`, `v_sum`, `depth_sum` and `bbox` reductions of the renderer's GT target mask
    (`navrl_detector.py:686-762`).
  - The segmenter is evaluated once per env, on the single flat paint value.
  - This passes the "no image, no semantic mask" wording (`navrl_detector.py:206-220`), but it is a
    GT-mask summary crossing into perception. Its validity rests on a fail-closed equivalence
    argument (`navrl_perception.py:1475-1549`).
  - Policy F does not use it.
- **Simulated detector realism versus oracle.**
  - Simulated (sensor-like) elements: FOV and 20 m range limits, pixel-centre sampling (scale loss
    from 12.75 m, certain from 23 m for an on-axis target [evidence: EQ-1A A5]), mesh occlusion,
    the 2-pixel threshold, and back-projection through declared intrinsics and extrinsic.
  - Oracle elements in the Policy F configuration: perfect precision and recall per visible pixel,
    exact depth, no false positives, no latency, no noise, no appearance variation, no distractors
    (`NAVRL_DISTRACTOR_COUNT` = 0, `navrl_detector.py:271-284`), and a perfect ideal gimbal.
  - The visual proxy (r = 0.15 sphere) also differs from the LiDAR proxy (r = 0.20).

### 2.3 Can GT reach the online detector or the actor?

| Path | Reaches perception/actor? | Citation |
| --- | --- | --- |
| `target_position` → `render_raw_rgbd` → rgb/depth pixels | Only as rendered pixels (the intended renderer role) | `navrl_task.py:6461-6463` |
| `target_position` → `navrl_target_position` → LiDAR kernel → ranges | Only as rendered ranges | `navrl_task.py:7557-7563` |
| `segmentation_pixels` (LiDAR semantic) | **No** in perception mode. It is read by the reward (`navrl_task.py:6311`), by action diagnostics (`5541`) and by the E-L0 evaluator. It enters the actor **only** in the legacy vision-oracle mode (`_process_obs_vision`, `navrl_task.py:6408-6409`), which Policy F does not use. | |
| `detector.target_mask/target_depth` | **No** in perception mode. Exported to `obs_dict` only in legacy vision mode (`navrl_task.py:6398-6400`). Read by E-L0 (`motar_eval/e_l0.py:166`). | |
| Critic GT extras | Written to `task_obs["states"][:, 898:906]` only (`navrl_task.py:6520-6540`) | |
| DETECT_CHANNEL GT-mask reductions | **Yes, into perception**, only when decoupled (opt-in) | §2.2 |

The task-level mutation test passed: 898-D rows and actions stayed bitwise equal under mutation of
target pose, LiDAR injection buffer, `segmentation_pixels`, obstacle positions and visibility flags
[evidence: EQ-1D D3-F1..F4; repeated for S-L1B, S-L1C, S-L1D and C-v2 (B-/C-/D-FIREWALL)].

---

## 3. Depth projection and centre estimate

- **Legacy (Policy F).**
  - Only the mask centroid is back-projected (§2.1, step 7).
  - Obstacle fusion maps LiDAR bearing θ to camera column with `u = round((hfov/2 − θ)/hfov·(W−1))`
    (`navrl_perception.py:1916-1919`). This is linear in angle, not pinhole: **A1**, 5.8 px / 3.41° at 20°.
  - It treats the camera origin as the LiDAR origin: **A2**, −0.0998 m on axis.
  - It copies one column minimum (over all 90 rows) to all 4 LiDAR rows (`navrl_perception.py:1913-1921`): **A3**.
- **S-L1B (opt-in).**
  - Every valid depth pixel is reprojected: `p = R·d·r̂ + t` into the LiDAR/body frame
    (`navrl_sensor_lineage.py:107-129`).
  - The points are binned per elevation into a clamped conservative envelope (`navrl_sensor_lineage.py:132-156`)
    and min-fused (`navrl_perception.py:1893-1908`).
  - Measured: max range error 6.9e-6 m [evidence: S-L1B B-POINT]. The envelope is not beam-exact:
    −4.13 … +0.0015 m versus beam truth.
- **Centre estimate and known biases.**
  - **A7, camera centre-range bias, OPEN in every lineage.** `centre = mean visible slant depth + 0.15`
    overestimates by +0.0445 … +0.0585 m (EQ-1A A7) and +0.0445 … +0.0508 m on GPU (EQ-1D D2-A1). The
    direction is fine (≤ 0.13 px). The cause is that the mean over the disc is greater than the
    nearest-point depth.
  - Plan only: `docs/plans/post_s_l1c_v2_validation_design.md` §"Camera centre-range bias".
  - **A4, LiDAR radius, −0.050 m, legacy.** The association adds the camera's 0.15 m to the 0.20 m
    proxy return (`navrl_perception.py:2092`). Fixed opt-in in S-L1C: centre = measured + 0.20, error
    ≤ 2.5e-5 m on beam-centred fixtures. An inherited max accepted-return centre bias of 0.1687 m
    remains [evidence: C-v2 RESULTS].
  - **[audit] Partial occlusion biases the centroid.** The centroid and mean depth cover only the
    *visible* part of the silhouette, so occlusion pulls the bearing toward the unoccluded side.
    This is not quantified in the searched evidence (NOT_RECORDED). Directly relevant to occlusion handling.
  - **[audit] The camera R is diagonal in world axes.** `measurement_var = [σ_r², σ_lat², σ_lat²]` is
    applied as `diag` in **world x/y/z** (`navrl_perception.py:2232-2235` → `correct` 713). It is
    never rotated into the camera ray frame, so the range variance always lands on world x whatever
    the bearing. EQ-1C T4 documents the same defect for LiDAR, but I found no record of it for the
    camera update. Status: OPEN, unmeasured.

---

## 4. Camera–LiDAR association and the target-like mask

### 4.1 LiDAR target association, `_associate_lidar_target` (`navrl_perception.py:2033-2135`)

**Legacy (Policy F):**
- Predicted centre: `rel = R⁻¹(state[:3] − drone_pos)`. Pick the azimuth bin nearest the predicted
  bearing, and take the minimum over its 4 rows (2035-2049).
- Gate: `clamp(0.35 + 2·√trace(P_pos), max = 1.0)` m (2050-2053), or the constant `NAVRL_LIDAR_ASSOC_GATE_M`.
- `valid = active & ~camera_visible & |measured − (‖rel‖ − 0.15)| < gate & in range` (2085-2091).
- The measurement is placed at the **predicted** bearing and height, with world-axis diagonal
  R = (0.08², 0.15², 0.20²). It resets `age` (2092-2131).
- Defects:
  - EQ-1C T4: tangential variance cut 1.0 → 0.022.
  - A6 / D2-N2: 7/7 target-absent bars accepted.
  - T3: the track stays alive on a bar indefinitely.
  - E-L0 moving run: 73/360 accepts were bars (count, not a rate).

**S-L1C v1 (opt-in, `s_l1c_v1`)** uses `range_only_lidar_association` (`navrl_sensor_lineage.py:212-266`).
- Only rows within `asin(0.20/d)` of the predicted elevation are eligible.
- A vertical-extension shape test rejects returns whose adjacent row has the same horizontal range
  within 0.10 m.
- Conclusive only if d ≥ 0.20/sin(10°) = 1.152 m.
- Centre = measured + 0.20 along the predicted unit vector.
- Range-only R = 0.08²·uuᵀ + 50²·(I − uuᵀ) (269-273).
- Fused age is reset unless `lidar_silent_correct` (`navrl_perception.py:2061-2084`).
- Fixtures: 0/21 negatives, 117/117 positives [evidence: S-L1C].
- Known band: off-row spheres between 1.152 and 2.295 m are refused [evidence: S-L1C REVIEW_ADDENDUM].
- Population false-association rate: NOT_TESTED.

**S-L1C v2 (opt-in, `s_l1c_v2`, level 3, D disabled)** uses `range_only_lidar_association_v2`
(`navrl_lidar_association_v2.py:21-76`). It adds sphere-versus-vertical-plane recoveries:
- requires ≥ 2 rows;
- sphere residual ≤ max(1 mm, 3·1e-4 m);
- plane residual minus sphere residual > max(1 mm, 4e-4 m).

Results and limits:
- Recall below 2.295 m: 140 → 264 of 288 for angularly exact predictions.
- With ±1° prediction error the recovery is diagnostic and unqualified: 36% → 58%.
- **It depends on the noise-free simulator LiDAR (0.1 mm bound).**
- 135 inherited CPU far-bar false acceptances remain [evidence: C-v2 QUALIFICATION/RESULTS].

### 4.2 Target-like LiDAR mask and the carve-out (`_fuse_static_and_extract_obstacles`, `navrl_perception.py:1835-2014`)

**Legacy:**
- `target_like = fused_visible & |Δbearing| < 15° & |scan − fused_surface| < 0.55 m` (1878-1888).
- The gate uses the *fused* bearing and range, so on LiDAR-only frames it uses the LiDAR's own
  quantised bearing. Constants are at 44-45.
- Carved beams are set to `lidar_max`, and the camera target pixels are blanked (1890, 1910-1912).
- Defect: S14, 2.8 m false clear on a real bar 0.53 m in front of the target [evidence: EQ-1D D2-F1].

**S-L1A:**
- `conservative_carve_masks` (`navrl_sensor_lineage.py:40-95`), authorised only by the current camera
  pixels (`confirmed = visible & camera_visible`, `navrl_perception.py:1849-1877`).
- The window starts at the minimum candidate-pixel depth and never erases nearer returns:
  0 bar returns removed [evidence: S-L1A A-CARVE].

**S-L1B:**
- `common_frame_carve_masks` (`navrl_sensor_lineage.py:159-203`) uses a body-frame anchor and the
  0.20 − 0.15 radius shift, with zero lower tolerance.
- **Open:** it keeps 84/102 (~82%) of target returns as obstacles, because the pixel-sampling anchor
  is biased high [evidence: S-L1B REVIEW_ADDENDUM item 1; plans CB-0/1/2].

**Provenance of `last_target_like` (N,4,72):**
- It is written at `navrl_perception.py:1889` from the masks above.
- It is sensor-derived: it depends on camera pixels, the fused bearing/range and the raw LiDAR, and
  never on `segmentation_pixels`.
- The rule that safety layers must use it instead of semantics is at `navrl_perception.py:1140-1145`.
- It is consumed online by the riskcap governor (`navrl_task.py:5284-5292`).

---

## 5. Tracker: `BatchedConstantVelocityTracker` (`navrl_perception.py:640-723`)

| Item | Contract |
| --- | --- |
| State | `state` (N,6) = [p_world(3), v_world(3)]; `cov` (N,6,6); `active` (N,) bool; `age` (N,) s. One track per env. |
| Motion model | Constant velocity, `F = [[I, dt·I], [0, I]]`, dt = 0.1 s. **Diagonal** Q: `q_pos = 0.025·dt² = 2.5e-4 m²`, `q_vel = 0.25·dt = 0.025 (m/s)²` per step, with no pos–vel cross terms (653-662). Only active tracks are predicted (671-677). |
| Initialisation | Camera-visible and inactive: `p = meas`, `v = 0`, `P_pos = diag(camera R)`, `P_vel = I`, `age = 0` (679-688). **The camera only** initialises a track; LiDAR needs `active` (2086). |
| Camera update | `step()` corrects visible, already-active envs with diagonal world-axis R and resets `age` (690, 696-723). Joseph-form covariance. **No innovation gate on camera measurements**; every detection is accepted. |
| LiDAR update | Second `correct()` in the same frame, without re-advancing time (§4.1) |
| Misses | Prediction only: `P ← FPFᵀ + Q` grows, `age += dt`. `active = False` once `age > memory_s = 5.0 s`, i.e. on the 51st miss (692-693; config `navrl_task_config.py:173`) [evidence: EQ-1C T2]. State and covariance are **not cleared** on expiry, only at `reset_idx` (664-668). Actor features are multiplied by `active` (`navrl_perception.py:2172`). |
| Identity | None. Single hypothesis. With distractors, the centroid of the union of target-coloured pixels is used. The S1 `ShadowAssociator` is a diagnostic-only counterfactual (`navrl_perception.py:1099-1128, 2236-2248`). S-L1C adds an evaluator-facing identity clock: `_s_l1c_identity_age` is reset only by camera updates; `_s_l1c_update_source` ∈ {0 prediction, 1 camera, 2 LiDAR}. It is not an actor input (1091-1096, 2274-2283). The fused `age` is still reset by LiDAR, so `active` ≠ valid identity. |
| Outputs exposed | `perception.tracker.{state,cov,active,age,memory_s,dt}`. Diagnostics `track_age` and `track_covariance` (`navrl_perception.py:2410-2411`) → `obs_dict["navrl_track_age"/"navrl_track_covariance"/"navrl_track_confidence"]` (`navrl_task.py:6504-6506`). |

---

## 6. Visibility classes

### 6.1 Online flags (sensor-derived, legal online)

| Flag | Definition | Citation |
| --- | --- | --- |
| `diagnostics["camera_visible"]` → `task._camera_visible_now` | count of segmenter pixels ≥ 2 (§2.1) | `navrl_perception.py:2395`; `navrl_task.py:1443-1445, 6483-6485` |
| `diagnostics["lidar_visible"]` | LiDAR association accepted (only when the camera is not visible) | `navrl_perception.py:2258-2273, 2396` |
| `diagnostics["visible"]` → `task._visible_now`, `perception.last_visible` | `camera | lidar` (the "fused" flag) | `navrl_perception.py:2286, 2343, 2394`; `navrl_task.py:1439, 6482` |
| LiDAR-only | `lidar_visible & ~camera_visible`, which is the same as `lidar_visible` by construction. EQ-1C C4: 42.7% of camera-lost frames; identity is not established. | |
| S-L1D online class (`s_l1d_v1` only) | Computed by `measurement_validity` (`navrl_sensor_lineage.py:283-332`; called at `navrl_perception.py:2016-2031, 2284-2285`). Inputs: delivered depth, the camera flag, and the track's own posterior and covariance in the vehicle frame. The predicted support is every pixel-centre ray hitting an r = 0.15 sphere at the belief. Occlusion evidence: `depth < t − τ`, with `τ = 0.05 + 2σ_los`. Precedence: NO_TRACK > DETECTED > PRED_RANGE_EXIT > PRED_FOV_EXIT > PRED_SCALE_LOSS (support < 2) > OCCLUSION_FULL / CENTRE / PARTIAL_EVIDENCE > UNEXPLAINED_LOSS. Exported as `s_l1d_online_class`, `s_l1d_support_px`, `s_l1d_occluded_px`, `s_l1d_centre_occluded`, `s_l1d_tau_m` (2418-2424). | |

The S-L1D class is **diagnostic only**. It was qualified on C-v1 with a *primed exact* belief
[evidence: S-L1D C01/C02]. In closed loop, belief drift matters: FOV_EXIT maps to UNEXPLAINED in
244/741 cases. It is disabled on C-v2 (`navrl_sensor_lineage.py:29-31`).

### 6.2 E-L0 classes: evaluator-only, never online (`motar_eval/e_l0.py`)

| Class | Computation (`classify`, `e_l0.py:97-110`) |
| --- | --- |
| NONE | Placeholder index 0; `classify` never emits it |
| RANGE_EXIT | `alone_px == 0` and `‖target − cam‖ − r ≥ 20` |
| FOV_EXIT | `alone_px == 0` and GT centre outside the pyramid (`centre_in_frustum`, 90-94) |
| SCALE_LOSS | `alone_px == 0` with the centre inside the FOV and in range (sub-pixel). This differs from S-L1D PRED_SCALE_LOSS (support < 2). E-L0 separately records `below_min_pixels`. |
| FULL | `alone_px > 0`, `visible_px == 0` |
| CENTRE_ONLY / PARTIAL | `0 < visible_px < alone_px`, with / without the GT centre ray blocked (`centre_ray_blocked`, mesh query, 139-151) |
| VISIBLE | `visible_px == alone_px > 0`. A 1-pixel target is VISIBLE but not detected. |

`alone_px` is computed from the detector's ray table and the **GT `task.target_position`**
(`e_l0.py:75-87, 154-178`). `visible_px` is the renderer's `target_mask`. Provenance uses
`segmentation_pixels` on the associated beam (`e_l0.py:121-136, 297`).

E-L0 installs a call-through wrapper over `perception._associate_lidar_target`
(`e_l0.py:246-256`): evaluation-only, outputs returned untouched.

**Allowed online: none of the E-L0 classes.** The online analogues are the S-L1D codes (§6.1).

---

## 7. The 898-D actor observation (must not change)

Built in `NavRLPerceptionModule.observe()` (`navrl_perception.py:2346-2387`) and written verbatim to
`task_obs["observations"]` (`navrl_task.py:6481`). Dimension guards are at `navrl_perception.py:455-463`
and `2388-2392`, and `navrl_task.py:1485-1489`. History is stride 5 steps (0.5 s), oldest slot first,
last slot = current step (`navrl_perception.py:2174-2182`, `navrl_task_config.py:266`).

| Slice | Dim | Content | Source |
| --- | --- | --- | --- |
| 0–287 | 288 | `static_state` = fused scan / 12, layout [row(4)][bin(72)]. LiDAR after the target-like carve, min-fused with camera depth (legacy column map, or the S-L1B envelope). | `navrl_perception.py:1847-1922` |
| 288–767 | 480 | `obstacle_history` 5 × 8 tokens × 12: pos/12 (3), vel 0 (3), 0.30/12, proximity, cov/144 (3), valid | `navrl_perception.py:1926-2013` |
| 768–817 | 50 | `robot_history` 5 × 10: vehicle-frame velocity / vmax (3), yaw-rate (1), previous action (4), z/(3·1.0) (1), 1 (1) | `navrl_perception.py:2332-2341` |
| 818–897 | 80 | `target_history` 5 × 16: vehicle-frame rel pos / 20 (3), rel vel / 2 (3), pos var / 400 (3, clamped), vel var / 4 (3), camera confidence, LiDAR confidence, age/5, 0.15/12; all × `active` | `_target_features`, `navrl_perception.py:2137-2172` |

Corridor, geofence and search segments are 0-D under Policy F (`navrl_perception.py:144-187, 2352-2386`).
Changing the actor schema is outside the authorised scope (`docs/SENSOR_CONTRACT.md`,
"Neither … authorize … actor-schema changes"; `AGENTS.md` observation contract). Enabling S-L1
changes the *values* of 768 of the 898 inputs; Policy F under S-L1 is an OOD robustness evaluation
(`docs/evidence/eq1d_qualification_2026-10-18/LINEAGE_DIFFS.md`).

---

## 8. GT firewall inventory (perception mode, `navrl_task.py`)

| Location | GT / semantic read | Role | Online? |
| --- | --- | --- | --- |
| `_process_obs_perception` 6461-6463 | `target_position`, `target_orientation` → `render_raw_rgbd` | renderer | renderer-only |
| `_sync_target_to_sensor` 7557-7563 | `target_position` → `navrl_target_position` | LiDAR renderer | renderer-only |
| `_process_obs_perception` 6520-6540 | `target_position`, `target_vel_w` → `states[:, 898:]` | asymmetric critic | train-time critic only |
| `compute_state_reward_and_terminations` 6019-6304 | `target_position` / `target_vel_w`, physical-target validity, capture swept segment | reward, termination | env |
| `add_static_safety_reward` 6306-6325 | `segmentation_pixels == 50` | reward | env |
| `step` 5075-5080, 5120-5127 | distance to goal, forensics | metrics | eval |
| `_record_action_diagnostics` 5495, 5541, 5571 | `target_position`, `segmentation_pixels` | diagnostics ("never enters observations…") | eval |
| `_record_distractor_lock_frame` 6542-6602, `_record_s1_shadow_frame` 6653-6697 | `target_position`, distractor `obstacle_position` | labelling | eval |
| `_record_episode_dump`, `_record_contact_geometry`, `_record_oob_exit`, `_record_bar_contact_probe`, overlays | GT target and bars | forensics | eval |
| `reset_idx`, `_sample_target_motion`, `_advance_target*`, `_plan_target_routes` | target state, `obstacle_position` | target simulation | world |
| `_process_obs_vision` 6385-6409 | `target_mask` / depth exports, `segmentation_pixels` **into the actor** | legacy VISION-ORACLE baseline only | not Policy F |
| `kroc_eval_adapter.py:393, 876-1017` | `target_position`, `target_vel_w` | OCC traces | eval (read-only hooks; `navrl_task.py:4945-6491`) |
| `motar_eval/e_l0.py:163, 166, 297` | `target_position`, `target_mask`, `segmentation_pixels` | E-L0 labels | eval ("GT … goes to files only", `e_l0.py:24`) |
| `navrl_detector.py:832, 686-762` | renderer copies `_targets`, `target_mask`, DETECT sums | renderer | renderer; decoupled DETECT_CHANNEL crosses into perception (opt-in) |

**Firewall verdict.** For Policy F (off) and S-L1A/B/C/D/C-v2, the actor and the online tracking
inputs contain no GT target state or semantics. This was verified bitwise at task level (EQ-1D D3;
B-/C-/D-FIREWALL; C-v2 actor firewall). The only online consumers outside the actor are:
- the governor, through `last_target_like`;
- the reward visibility bonus, through `_visible_now` (`navrl_task.py:6067-6069`).

Both are sensor-derived. The residual concerns are informational, not structural:
- the oracle-grade detector (§2.2);
- writable GT aliases next to legal tensors (`obs_dict["navrl_target_position"]`, `detector._targets`);
- DETECT_CHANNEL when decoupled.

---

## 9. Known defects and status

| ID (source) | Defect | Policy F (off) | Status |
| --- | --- | --- | --- |
| D2-R3 (EQ-1D) | 40×24 bilinear depth upsampling: 2.3% of pixels > 5 cm off, max 7.3 m; phantom obstacles at bar edges | present | **Opt-in fixed**, S-L1A (max 4.1e-5 m, 0 false-clear) |
| A1 (EQ-1A) | Linear column map, 5.8 px / 3.4° | present | **Opt-in fixed**, S-L1B |
| A2 (EQ-1A, D2-F1) | Camera range used as body range, −0.0998 m | present | **Opt-in fixed**, S-L1B |
| A3 | One column minimum copied to 4 rows | present | **Opt-in fixed** as a conservative envelope (S-L1B); not beam-exact (−4.13 m offsets) |
| D2-F1 S14 (EQ-1D) | Carve-out deletes a real foreground bar, 2.8 m false clear | present | **Opt-in fixed**, S-L1A/B (0 bar beams carved) |
| B-carve retention (S-L1B addendum) | ~82% of target-centre returns kept as obstacles | n/a | **OPEN** (CB-0/1/2 plan only) |
| A4 (EQ-1A) | LiDAR centre radius −0.050 m | present | **Opt-in fixed**, S-L1C (≤ 2.5e-5 m); inherited 0.169 m max accepted-return bias **OPEN** |
| A7 / D2-A1 | Camera centre range +0.045–0.058 m | present | **OPEN** in every lineage (design only) |
| A5 / D2-O1 | FULL/PARTIAL/CENTRE/scale label errors (S12, S14) | legacy indicator | **Fixed in the evaluator** (E-L0, 27/27); online analogue S-L1D is opt-in, diagnostic, primed-belief qualified |
| D2-O1 S08 (range versus FOV conflation) | Centre-frustum indicator reads range exit as FOV exit | legacy indicator | **Fixed** in E-L0 (RANGE_EXIT) and S-L1D (PRED_RANGE_EXIT); historical OCC records unchanged |
| A6 / D2-N2 / T3 | Target-absent bars accepted; track alive on bar | present (E-L0 moving 73/360) | **Opt-in fixed on fixtures**, S-L1C (0/21). Population rate NOT_TESTED. Recall band 1.152–2.295 m partly recovered by C-v2. 135 CPU far-bar false acceptances and the >11.35 m vertical-bar residual **OPEN**. |
| T4 (EQ-1C) | LiDAR update shrinks unobserved covariance | present | **Opt-in fixed** (`NAVRL_LIDAR_RANGE_ONLY_UPDATE=1` or S-L1C) |
| [audit] | Camera R diagonal in world axes | present | **OPEN**, unrecorded |
| [audit] | Visible-part centroid bias under partial occlusion | present | NOT_RECORDED |
| D2-R1 / D2-L1 | float32 kernel residuals 3.3e-4 / 1.25e-4 m | present | Open, below task tolerance |
| D2-Y1 | Ideal yaw-only camera (and LiDAR) mount | present | **OPEN** (synthetic idealisation; hardware NOT_TESTED) |
| D-on-C-v2 | S-L1D validity on the C-v2 belief distribution | n/a | NOT_TESTED; D disabled on C-v2 |

---

## 10. What an opt-in planner may read online

The precedent is the H-R3 arms (`motar_eval/h_r3_arms.py:1-7`; `tools/h_r3_p03_launcher.py:104-111, 141`).
They use own pose and velocity, `tracker.state[:, :3]`, `_lidar_distance_m()`, the ray table, and a
trigger on `_camera_visible_now` transitions.

### 10.1 Whitelist

Shapes are for N envs with Policy F's V = 4 and H = 72. **Read only. Do not mutate.**

| Attribute | Shape / units | Notes |
| --- | --- | --- |
| `task.obs_dict["robot_position"]`, `["robot_linvel"]` | (N,3) world m, m/s | Perfect odometry (an idealisation) |
| `task.obs_dict["robot_vehicle_orientation"]` / `["robot_orientation"]` | (N,4) xyzw | Yaw-only frame = camera/LiDAR frame |
| `task.obs_dict["robot_body_angvel"]` | (N,3) | |
| `task.obs_dict["depth_range_pixels"]` or `task._lidar_distance_m()` | (N,1,4,72) normalised / (N,288) m | Raw LiDAR. **Includes target returns.** Exclude them with the belief or `last_target_like`, never with semantics. |
| `task.obs_dict["navrl_raw_rgb"]`, `["navrl_raw_depth"]` | (N,3,90,160) [0,1]; (N,90,160) m slant | Delivered RGB-D. Depth saturates at 10 m off-target. |
| `perception.tracker.state` / `.cov` / `.active` / `.age` | (N,6) world; (N,6,6); (N,) bool; (N,) s | Belief. `memory_s = 5.0`, `dt = 0.1`. State is stale when inactive. |
| `task._camera_visible_now`, `task._visible_now`; `perception.last_visible`, `last_confidence` | (N,) | Online flags |
| `perception.last_target_like` | (N,4,72) bool | Sensor-derived target-return mask |
| `perception.last_scan_nearest` | (N,72) m | Fused, post-carve, row-minimum |
| `observe()` diagnostics: `visible`, `camera_visible`, `lidar_visible`, `confidence`, `camera_confidence`, `camera_measurement_world` (N,3), `target_pixels` (N,), `track_age`, `track_covariance` | | Reach consumers via the `after_perception` hook (`navrl_task.py:6486-6491`). `camera_measurement_world` is meaningless unless `camera_visible`. `target_pixels` is clamped to ≥ 1 (2221). |
| `obs_dict["navrl_track_confidence"/"navrl_track_age"/"navrl_track_covariance"]` | (N,), (N,), (N,6,6) | Copies |
| Calibration constants: `perception._lidar_angles` (72,), elevations [20, 10, 0, −10]°, `hfov`, `vfov`, `fx`, `fy`, `cx`, `cy`, `camera_offset` (1,3), `target_radius` 0.15, `max_camera_range` 20, `camera_obstacle_max_range` 10, `lidar_max_range` 12, `min_pixels` 2 | | Declared sensor model |
| S-L1C/C-v2 only: `perception._s_l1c_identity_age`, `_s_l1c_update_source` (N,); `last_s_l1c_association` dict | | Sensor-only, evaluator-facing. Legal information; not qualified as a control input. |
| S-L1D only: `perception.last_s_l1d_validity` {`code`, `support_px`, `occluded_px`, `centre_occluded`, `tau_m`, `in_frustum`} (N,) | | Same: legal information, diagnostic qualification only |
| S-L1B: `last_s_l1b_points` (rho/az/el/valid (N,90,160)), `last_s_l1b_envelope` / `_strict` (N,4,72) | | |
| Actor `obs` (N,898), the action, `task.prev_action`, `task._speed_governor_last` | | Own policy and filter state |
| `obs_dict["env_bounds_min/max"]` (N,3) | | Arena geometry. Not a sensor: declare it as a map prior if used. |

### 10.2 Must NOT read online

- `task.target_position`, `target_vel_w`, `target_orientation`.
- `obs_dict["navrl_target_position"/"navrl_target_orientation"]`: a writable GT alias.
- `obs_dict["segmentation_pixels"]`.
- `detector.target_mask`, `target_depth`, `_targets`, `_targets_wp`, `_target_orientations`.
- `detector.distractor_mask/depth`, `detector.detect_*`, `DETECT_CHANNEL`.
- `detector.obstacle_depth`: not GT, but not the delivered sensor.
- Legacy `obs_dict["target_camera_mask"/"target_camera_depth"/"obstacle_camera_depth"]`.
- `task_obs["states"]`.
- `obs_dict["obstacle_position"]` and any bar layout.
- `navrl_target_contact/invalid`; `captured_now`/`crashed_now` except as episode boundaries.
- All E-L0 outputs (`silhouette_from_task`, frame files, provenance).
- `recovery_visibility_probe`, KROC/forensics/ledger traces, `h_r3_p02_identity` identity labels.

Also, the planner must not enable decoupled detection, must not call `observe()` or
`render_raw_rgbd()` a second time, and must not mutate the tracker. Doing so would break the
single-consume channel, the RNG streams and bitwise parity.

---

## Summary table

| Component | Online input | Output | file:line | Validated by (evidence) | Open issue |
| --- | --- | --- | --- | --- | --- |
| RGB-D renderer | robot pose (yaw-only), GT target pose (render role), Warp mesh | rgb (N,3,90,160), depth (N,90,160) slant m | `navrl_detector.py:820-991` | EQ-1D D2-R1/S1/N1/Y1/T1; S-L1A A-DEPTH | Ideal gimbal; 40×24 upsampling (legacy); 10 m obstacle far plane; float32 |
| LiDAR renderer | robot pose, mesh, GT target sphere 0.20 | `depth_range_pixels` (N,1,4,72) | `warp_lidar_kernels.py:167-251`, `warp_sensor.py:184-245` | EQ-1D D2-L1 | Noise-free; radius differs from camera |
| Target detector | rgb, depth | visible, centroid, surface range, confidence, mask | `navrl_perception.py:1579-1760` | D2-S1, D2-N1, E-L0 E0-L3, EQ-1A A7 direction | Oracle-grade colour mask; A7 +5 cm; visible-part centroid bias |
| Camera → 3D | centroid, surface range, intrinsics, extrinsic, pose | `meas_world` (N,3) | `navrl_perception.py:1738-1749, 2214` | EQ-1A A7, D2-A1 | A7; world-axis R [audit] |
| Obstacle fusion (legacy / S-L1B) | LiDAR, depth, target pixels, fused bearing/range | `static_state` (N,288), tokens | `navrl_perception.py:1835-2014`; `navrl_sensor_lineage.py:107-203` | EQ-1D D2-F1; S-L1A A-CARVE; S-L1B B-POINT/ENVELOPE/CARVE | Legacy A1–A3, S14; B 82% target retention |
| LiDAR association (legacy / C / C-v2) | raw LiDAR, belief, camera flag, gate | lidar_visible, conf, KF correction | `navrl_perception.py:2033-2135`; `navrl_sensor_lineage.py:212-273`; `navrl_lidar_association_v2.py:21-76` | EQ-1C T3/T4; D2-N2; S-L1C C-NEG/C-POS; C-v2 GPU gates | Legacy false associations; population NOT_TESTED; noise-bound dependence |
| Tracker | camera/LiDAR measurements | state, cov, active, age | `navrl_perception.py:640-723` | EQ-1C T1/T2 | Heuristic Q, no camera gate, no identity, LiDAR resets age |
| Online validity (S-L1D) | depth, camera flag, belief | 9-class code | `navrl_sensor_lineage.py:283-332` | S-L1D C01/C02 (primed belief) | Belief-error robustness NOT_TESTED; disabled on C-v2 |
| Actor observation | all of the above | (N,898) | `navrl_perception.py:2346-2392` | EQ-1D D3; E-L0 E0-N1 | Frozen; S-L1 is OOD |
| E-L0 labeller | GT target, renderer mask, semantics | classes, provenance | `motar_eval/e_l0.py:75-178` | E-L0 E0-L1..L3 | Evaluation-only; pixel-centre, not sub-pixel |

## Top 8 risks for a belief-conditioned visibility planner on this chain

1. **The detector is an oracle silhouette.** The flat-red paint plus the colour rule reproduces the
   occlusion-aware GT mask exactly, with no noise, false positives or latency. Measured "visibility"
   and "reacquisition" are in effect GT pixel-centre visibility at ≥ 2 px. Gains may not transfer
   to a real detector. A planner must not be tuned against this idealisation without perturbation arms.
2. **The camera and LiDAR are ideal gimbals, and odometry is perfect** (D2-Y1). Viewpoint control is
   rewarded by an unrealistically stable view. Yaw is not held open-loop (D1-V1). A body-fixed camera
   would lose frames under pitch.
3. **`active` / fused-visible ≠ identity.**
   - Legacy LiDAR association hijacks tracks (7/7 bar negatives; 73/360 false in moving E-L0), and
     LiDAR resets `age`.
   - S-L1C fixes the fixtures, but the population rate is NOT_TESTED.
   - A planner conditioned on `track_age` or `visible` can pursue a bar. Use camera-only recency
     (`_s_l1c_identity_age`) where available.
4. **Belief covariance is not calibrated.**
   - Diagonal heuristic Q.
   - The camera R is axis-aligned in world frame [audit].
   - Legacy LiDAR shrinks unobserved variance (T4).
   - A7 adds +5 cm of radial bias; the partial-occlusion centroid bias is unmeasured.
   - The actor scales position variance by 1/400 with a clamp.
   - Expected-visibility or information-gain terms computed from `cov` will be mis-weighted.
5. **Online occlusion prediction (S-L1D) is weakly qualified.**
   - Only with primed exact beliefs, and diagnostic-only; disabled on C-v2.
   - In closed loop, drift turns FOV exits into UNEXPLAINED_LOSS (244/741).
   - Depth saturates at 10 m while the camera range is 20 m, so occluders beyond 10 m or within τ
     are invisible to depth evidence.
   - The 4-row, 5° LiDAR gives a sparse occluder map.
6. **The obstacle map near the target is wrong in mode-dependent ways.**
   - Legacy: false clears (2.8 m) and upsampling phantoms.
   - S-L1B: about 82% of target returns are kept as obstacles. LOS and clearance checks must exclude
     the target by belief (as H-R3 does), otherwise the target occludes itself.
   - Any carve or exclusion inherits A7.
7. **Contract and distribution coupling.**
   - The 898-D actor is frozen, and Policy F under any S-L1 lineage is out of distribution.
   - Lineage combinations are not jointly qualified (D × C-v2).
   - The riskcap governor post-filters every command, using `last_target_like`, so planner commands
     are not executed as issued.
   - The resolution-limited scale band (12.75–23 m at 160×90) makes far reacquisition fail for
     reasons other than occlusion.
8. **Implementation hazards.**
   - Process-wide switches are read at import and construction (HBEAMS, MAX_OBSTACLES, LINEAGE);
     see the test-pollution and worktree-import notes in the user's memory.
   - TF32 is on under rl_games (D3-S1). This affects batched matmul and cdist precision, per the
     user's TF32 memory note.
   - DETECT_CHANNEL is single-consume.
   - Writable GT aliases (`navrl_target_position`, `detector._targets`) sit beside legal tensors, so
     a careless read or write leaks GT or corrupts the renderer. Enforce a whitelist accessor and
     replay a D3-style mutation firewall test.
