# Runtime v1 (CAM-S2 branch of the camera flight runtime): review r1

- Reviewer: Claude Opus 5.5 (fresh context, read-only)
- Date: 2026-11-24
- Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b`, branch `camonly/s2b`, base `3a453e6`.
  The change is uncommitted:
  - `tools/camera_policy_runtime.py` (M, +306/-14)
  - `tools/camera_policy_contract.py` (M, +13)
  - `tools/camera_policy_contract_cams2.py` (new)
  - `tests/test_camera_runtime_cams2.py` (new)
- Nothing under `src/` was edited. Probes ran from scratch scripts on cores 6, 7, 12 and 13 only, CPU only.
- Line numbers below refer to the working-tree files at review time.

## Verdict: PASS_WITH_LIMITS

- **What is correct:**
  - The change implements the trained CAM-S2 rules on the runtime path: the previous-action rule (F1), the altitude, mount, yaw scale and blind cap (F2), the no-return depth encoding and the 0.35 s bound.
  - Given the same depth image, it reproduces the training observation. Parity max |Δobs| = 4.8e-7 for k = 1 and k = 2.
  - CAM-K2 v3 behaviour is unchanged: 329 camera-runtime tests pass and the CAM-K2 delay set is still (0,).
- **Why it is not flight-ready:**
  1. The runtime has no guard against a source image coarser than the trained camera. The vehicle side's documented decimation of 3 then turns obstacle bins into free space (P1-1).
  2. The latency on the Orin (and its fit to k̂) is unmeasured and unchecked (P1-2).
  3. "Exactly the trained observation from real data" holds for the code path, not for the sensor semantics. Training fed a 40×24 bilinear obstacle render. Neither the CPU parity nor the specified GPU check tests this (P1-3).
- No P0 was found on the path as specified (full-resolution 624×468 source).

## Test run (item 8)

**Command:** the 21 modules `test_camera_*`, `test_runtime_fingerprint`, `test_sitl_camera` and `test_oakd_camera`, run once with the prescribed environment and `taskset -c 6,7,12,13`. Log: `/home/fair/motar_tmp/runtime_v1_20261124/review_r1/test_run.log`.

**Result:** `Ran 329 tests in 171.966s — OK (expected failures=10)`, no skips.
- The CAM-S2 classes ran, because `HAVE_S2A` is true.
- The argparse "error:" lines in the log are expected-refusal tests.

**Parity report**, re-run separately for the numbers. The two delays gave identical figures:

| Delay | `max_abs_diff` | `visible_steps` | `k2_prev_diff` | `exact_slots` | `stale_refused` | `blind_capped_steps` | `minz_pixels` |
|---|---|---|---|---|---|---|---|
| k = 1 | 4.77e-7 | 10 | 0.79 | 165 | true | **0** | 48672 |
| k = 2 | 4.77e-7 | 10 | 0.79 | 165 | true | **0** | 48672 |

**Mutation probes** (monkeypatched inside the parity subprocess only; the repository was not touched):

| Mutant | Effect | Result |
|---|---|---|
| M1 | Delayed xy fed without the governor and without the blind cap | **killed** (prev assertion at step 1) |
| M2 | Delayed xy governed but **not blind-capped** | **survives** (parity passes) |

## Item-by-item

### 1. Previous-action channel

**Training rule** (verified in the code):
- `navrl_cams2.py:129-133` rolls the history and passes `delayed(hist, k)` = a_{t-k} to the original `transform_action_to_command`.
- That function clamps, scales and governs. `_apply_sensor_speed_governor` is wrapped by camsafe, so the blind cap comes after the governor (`navrl_camsafe.py:163-174`).
- It stores `_last_governed_action_xy` (`navrl_task.py:4966-4970`).
- `step()` then sets `prev_action = clamp(a_t)` and overwrites xy with the governed, capped, delayed command (`navrl_task.py:5051-5057`).
- **Reset:** the history is zeroed (`navrl_cams2.py:135-143`) and `prev_action` is zeroed (`navrl_task.py:4901`).

**Runtime** (`camera_policy_runtime.py:1493-1510`, `_cams2_command`):
- It rolls its own history of clamped actions and takes `delayed(history, k̂)`.
- It computes `cap_blind(_govern(clamp(xy)·v_max))/v_max`, clamped, with z/yaw from a_t. This is the same formula.
- `reset()` zeroes `prev` and `_action_history` (`:1337-1349`).
- The first-step-after-reset case is asserted by the integration test: prev xy = 0, z/yaw = a_0.

**k̂:**
- k̂ is explicit. There is no default in the CLI (`camera_policy_contract_cams2.py:340-354`).
- It is refused outside [max(DELAY_MIN, 1), DELAY_MAX] (`:187-190`), and re-checked at runtime (`camera_policy_runtime.py:1296-1298`).

**Limits:**
- **(a)** The fixed k̂ is only as right as the vehicle's real capture→actuation latency, and nothing measures or checks that. See P1-2.
- **(b)** In training, the governor and blind cap act on the frame at application time, with zero latency. In flight, the published command is governed on the capture frame and applied about k steps later. The prev channel therefore reports `gov_t(a_{t-k})`, which follows the trained formula, not what the vehicle actually executed (`gov_{t-k}(a_{t-k})`). See P2-4.
- **(c)** The blind-cap part of the channel is untested by parity: M2 survives (P2-1).

**`applied_zero_hold` for CAM-S2** (`:1357-1369`):
- It overwrites the newest history slot and zeroes the z/yaw of `prev`. This is consistent with the latency model.
- It is reachable only through the outdoor re-entry hold, and the outdoor profile is refused for CAM-S2.

### 2. Height, mount, yaw-rate, max velocity, blind cap

**Contract** (`camera_policy_contract_cams2.py:141-229`):
- Every value comes from the receipt env through the training modules' own parsers, loaded by path at `:121-128` and `:154-158`.
- The receipt's `config_after`, `delay_steps` and `overrides_after` are cross-checked (`:159-171`).

**Runtime** (`_setup_cams2`, `camera_policy_runtime.py:1277-1300`):
- It applies `navrl_cams2.apply_config`.
- It asserts that `flight_altitude`, `camera_translation`, `yaw_rate_max`, `max_velocity`, `blind_mps` and the delay pair equal the contract.
- `_check_cams2_build` (`:1302-1323`) asserts the 312×234 perception, no decoupling, `max_camera_range` = MaxZ, fx/fy, the camera offset, and that the normaliser is equal to the checkpoint's `running_mean_std`.
- The height feature uses `tc.flight_altitude` = 1.5 (`navrl_perception.py:2337`). The integration test checks 1.5/4.5.

**Cross-lineage leaks:**
- Dispatch is on `lineage`: `camera_policy_contract.py:228-231` and `:574-576`. A relabelled contract is fully validated by the other lineage's validator and fails there.
- CAM-K2's delay set stays (0,), and the test asserts it.
- The only CAM-K2 items a CAM-S2 contract inherits are the deployment floor/ceiling defaults, and that is intended.
- Leaks outside the runtime:
  - `camera_ros2_node.py:226` and `camera_companion.py:81` read `contract['seed']`, which fails with a KeyError on CAM-S2 (P3).
  - `oakd_camera.py:240-247` hard-codes the CAM-K2 66×54 coverage check (P3).

### 3. Depth preprocessing

`project_rgbd_min_valid` (`camera_policy_runtime.py:500-582`):
- **Valid samples:** finite, not flagged, MinZ ≤ z ≤ MaxZ and covered (`_source_classes` at `:236-266`; OAK 0 → `too_close` → invalid).
- **Moving into the trained camera:** the full attitude and the mount move each sample into the level trained camera (rotation plus translation, `:524-531`). It is splatted to the nearest trained pixel (`:544`), and MinZ/MaxZ are re-applied in the level camera (`:546`).
- **Per pixel:** min-z wins (lexsort, `:549-557`). A trained pixel without a valid sample is no-return, and so is every out-of-range sample: `_preprocess_cams2` sets them to `max_camera_range` = 20 (`:1456-1474`), as in training (`navrl_depth_model.py:154-164`). No blob_near; RGB is not blanked.
- **Principal point:** handled by reprojection. The test with the measured cy 246.7 lands the edge within 1 row; the plain 2×2 reduction is 5 or more rows off.
- **Tilt:** I probed the full-resolution source at roll/pitch (0,15), (20,0), (25,20) and (0,30) degrees. There were 0 interior holes and 0 empty columns in every case.
- **Sky:** the runtime encodes sky as 20 m and training as no-hit 10 m. Both become LiDAR no-return through `camera_no_return_to_lidar_range` (`navrl_perception.py:216-229`, the ≥ 10 m rule), and the parity confirms this.

**Not equivalent to the simulator:**
- **The training depth image:** training obstacle depth is a 40×24 ray grid, **bilinearly** upsampled to 312×234 and clamped at 10 m (`navrl_detector.py:927-938`, config `navrl_task_config.py:229-231`). The depth model's MinZ/holes act on that image. The perception then samples one image column per in-FOV bin (13 columns).
  - Real full-resolution min-valid depth gives exact, sharp bars.
  - Training gives bars smeared about ±1.67° at interpolated, longer ranges, with bars beyond about 7 m sometimes between rays. The bilinear MinZ edge artefacts are documented in `docs/evidence/camonly_2026-11-04/camk2e/DEPTHPROBE_REPORT_codex.md:17,54`.
- **Large invalid regions:** with the IR projector off, vehicle-side measurements show 30–39 % invalid depth on white walls. These become free space, which is the trained optimistic encoding.

This gap is P1-3.

### 4. Contract refusals and binding

**Refusals:**
- `runtime_env` must equal `flight_environment(receipt env)` exactly (`:296-300`).
- Training-only prefixes and keys are refused (`:66-67`, `:265-269`). This covers `NAVRL_P9_*`, `NAVRL_DEPTH_*` (including noise and the model switch), vehicle/gain randomisation, `NAVRL_DETNOISE_*`, dropout, RGB noise and the receipt path.
- Inherited process `NAVRL_*` variables that conflict are refused (`camera_policy_runtime.py:1210-1213`).
- Eval-only wrappers in the receipt (YAWCLIP, CAMS2DIAG, CONTACTCODE, blob_near, detector checkpoint) are refused (`:63-64`, `:147-149`).
- The removed keys either default to the same values (`navrl_task_config.py:278,427-428`) or are gated by `training=False` (`navrl_perception.py:1593-1596`). P9 must be removed because it is built regardless of `training` (`navrl_perception.py:955-982`). Note that training's target channel therefore carried P9 latency of 45–121 ms and the NO_LOCK/FALSE_LOCK states.

**SHA bindings:**
- Receipt SHA, checkpoint SHA listed next to the receipt, normaliser SHA, and the current source SHAs.
- Training-time pins for perception, ablation, depth model, cams2 and camsafe (`:73-77`, `:176-182`).
- I verified that `navrl_task_config.py`, `speed_governor.py`, `navrl_detector.py` and the rl_games player/network code are byte-identical to the training head `22ed16f` (`git diff --stat` is empty apart from two added `.sh` files). They are not pinned to training-time bytes by the contract (P3).

**Silent mismatch is possible (P2-3):**
- `_checkpoint_listed` (`:131-138`) accepts any SHA in `checkpoint_sha256_all.txt`, `checkpoint_sha256.txt` **or a bundle's `SHA256SUMS`**.
- The checkpoint's own `env_state` is never compared with the receipt. Only the schema/action-policy keys of `apply_schema_environment` are compared (`camera_policy_runtime.py:1252-1255`).
- The checkpoint carries `cfg_yaw_rate_max 3.0`, `cfg_max_velocity 2.5`, `cfg_detect_width/height 312/234`, `cfg_speed_governor_*`, `cfg_detector_*` and `cfg_lidar_max_range`.
- So a hand-made bundle listing an S2b checkpoint next to an S2a receipt would build with the wrong yaw scale. The normaliser check would not catch it either if the bundle's npz was exported from the same checkpoint.

### 5. Capture-age bound

- CAM-S2 gets (2+1)·0.1+0.05 = 0.35 s. The dispatch is at `camera_policy_contract.py:221-235`, and the supported set (0,1,2) at `camera_policy_contract_cams2.py:52`.
- `NAVRL_ACTION_DELAY_STEPS_MAX` must equal `DELAY_MAX` (`:184-186`).
- CAM-K2 stays at 0.15 s, and the test asserts both.
- The runtime refuses 0.36 s; the parity reports `stale_refused`.
- The bound is coherent with its definition: a command is not used after capture + 0.35 s (session `valid_until`).
- It does not by itself keep the effective delay inside the trained support (P1-2).

### 6. Near-constant slots

**What the 487 slots are:**
- Structural constants: no-LiDAR slots outside the camera sector (= 1), zero z/velocity obstacle fields, slot 817 (= 1) and the target offset-13 slots (= 0).
- Tiny-magnitude covariance features: obstacle offsets 8–10, values of order 1e-4.

**What a deviation means:**
- A deviation above 1 normalised unit in a structural slot can only come from a pipeline defect.
- Some tiny-magnitude slots go above 1 on legitimate data. For example, slot 297 (lateral covariance of obstacle 0) reaches about 2.3 units in an open scene with r = 12 m.

**Recommendation:** refusing on every violation would be wrong. Refuse on the structural slots and keep reporting the rest (P3). The current report-only handling (`:1325-1335`) is acceptable meanwhile.

**Height (P2-5):** the height slot is not in the set. Slot 816 has variance 1.98e-4, i.e. σ ≈ 6 cm at 1.5 m, so an altitude or estimator-origin error of 0.2 m is about 3σ.

### 7. Parity test

**It is non-vacuous for:**
- the depth path, with sub-samples farther or flagged so that min-valid must pick the trained sample;
- MinZ;
- sky no-hit;
- target detection;
- the delayed prev channel, with the governor active (M1 is killed);
- the k = 2 path;
- the timing refusal;
- the exact structural slots.

**It does not cover:**
- **(a)** The blind cap in the prev channel: `blind_capped_steps` = 0 and is never asserted, so M2 survives (P2-1).
- **(b)** The real training governor: the training side calls the runtime's own `_govern` (`test_camera_runtime_cams2.py:437`).
- **(c)** The renderer's 40×24 bilinear depth (P1-3).
- **(d)** Non-identity attitude/mount: the reprojection is the identity.
- **(e)** Resets mid-sequence.
- **(f)** The `alt_diff` negative control (`:447`) is pure arithmetic and exercises no code (P3).

**Is the specified GPU check sufficient?**
- **What it closes:** (b), and it ties the runtime to the real wrapped training code.
- **What it does not close:**
  - It does not require blind-capped or governor-limited steps. Rare blind events in 200 steps would leave M2 undetected again.
  - It does not say how to align the runtime's reset with episode ends inside the 200 steps.
  - Its 1e-5 tolerance ignores GPU TF32 versus CPU differences (the known rl_games TF32 hazard).
  - It uses identity attitude only.
  - By construction it cannot detect P1-3, because it feeds the simulator's own depth.

**Verdict on the GPU check:** necessary, not sufficient (P2-6).

## Findings

| ID | Class | Finding | Evidence | Fix |
|---|---|---|---|---|
| P1-1 | P1 (P0 if the vehicle's documented `source_decimation: 3` is carried over to CAM-S2) | No guard against a source coarser than the trained 312×234. In my probe (`project_rgbd_min_valid`, 3 m wall filling the view), a 208×156 source left 105 trained columns empty and **4 of 13 obstacle bins read no-return (free space)**. 624×468 and 312×234 gave 0 of 13. The vehicle-side YAML declares decimation 3 as its DECISION (`policy_contract_ours.yaml:68`), and the runtime accepts it silently. | `camera_policy_runtime.py:500-582`; the downscale docstring itself warns about holes (`:466-476`) | Refuse when source fx/fy are below the trained fx/fy (or below 2× for margin), or when an imaged trained column has no sample. Add a refusal test with a positive control. |
| P1-2 | P1 | Latency is unmeasured and unchecked against k̂. Full-resolution reprojection alone takes 67 ms per frame on one desktop core (my timing). The vehicle side measured about 185 ms for the CAM-K2 full-resolution projection on the Orin, plus 77 ms capture→arrival. That makes about 0.26–0.3 s capture→command, i.e. ≈ 3 steps, outside the trained support {1, 2} while still inside the 0.35 s bound. The runtime never compares the measured age with k̂. | `camera_policy_contract_cams2.py:81-89`, `camera_policy_runtime.py:1385-1395` | Measure the full CAM-S2 step on the Orin. Report `now − capture` per step against k̂·dt; refuse or flag when the implied delay leaves [DELAY_MIN, DELAY_MAX]. |
| P1-3 | P1 (claim/limit) | Training obstacle depth is a 40×24 render bilinearly upsampled and clamped at 10 m. Real min-valid depth is sharp, has finer angular resolution and different MinZ edge behaviour. Textureless invalid surfaces become free space. "Exactly the trained observation from real data" holds only for identical depth images. Neither the parity nor the GPU check can see this. | `navrl_detector.py:927-938`, `navrl_task_config.py:229-231`, `navrl_perception.py:1913-1921` | User decision: emulate the 40×24 point-sampled grid with bilinear upsampling on real depth, or accept the gap and measure it on recordings. State the limit in the contract/test docstrings. |
| P2-1 | P2 | The parity never exercises the blind cap; M2 (no cap on the delayed command) survives. | `test_camera_runtime_cams2.py:430,465`; not asserted at `:484-496` | Assert `blind_capped_steps > 0`, e.g. with a forced lateral action sequence. |
| P2-2 | P2 | The training-side governor in the parity is the runtime's replica. | `test_camera_runtime_cams2.py:437` | Covered only by the GPU check; keep that check mandatory. |
| P2-3 | P2 | The checkpoint ↔ receipt binding is only a SHA listing, which can be a hand-made bundle `SHA256SUMS`. The checkpoint `env_state` (`cfg_yaw_rate_max`, `cfg_max_velocity`, `cfg_detect_*`, `cfg_speed_governor_*`, `cfg_detector_*`) is not cross-checked. | `camera_policy_contract_cams2.py:79,131-138,191-195` | Compare these `env_state` fields with the receipt env in `_metadata` or `_check_cams2_build`. |
| P2-4 | P2 | Safety-layer latency: training applies the governor and blind cap at application time; flight applies them at capture time, k steps before. The prev channel follows the trained formula, not the executed command. | `navrl_task.py:4966`, `camera_policy_runtime.py:1503-1508` | Document in `PREVIOUS_ACTION_NOTE`; have the PHASE 7 filter re-apply the cap on the newest frame. |
| P2-5 | P2 | The height feature is tight (σ ≈ 6 cm) and assumes estimator z = height above floor. The `estimator_height` assertion is enforced only when a floor/ceiling filter is on. | `navrl_perception.py:2337`; tests build with `ground.enabled=False` | Require the assertion for CAM-S2 regardless, or report the height-slot deviation per step. |
| P2-6 | P2 | The GPU check spec does not require blind-cap/governor events, does not handle episode resets, does not account for TF32 tolerance, and uses identity attitude only. | `test_camera_runtime_cams2.py:13-27` | Amend the spec before preregistration. |
| P3-1 | P3 | Near-constant handling: split the structural slots (refuse) from the tiny-magnitude slots (report). Blanket refusal would trip on open scenes (slot 297 ≈ 2.3 units). | `camera_policy_runtime.py:1325-1335` | — |
| P3-2 | P3 | The `alt_diff` negative control is vacuous arithmetic. | `test_camera_runtime_cams2.py:447` | Build a runtime with altitude 1.0 or compare against an altered `tc`. |
| P3-3 | P3 | ROS2 node/companion read `contract['seed']` (KeyError on CAM-S2). `oakd_camera.require_training_fov` checks the CAM-K2 66×54 FOV. | `camera_ros2_node.py:226`, `camera_companion.py:81`, `oakd_camera.py:240` | Loud failures, not silent ones. |
| P3-4 | P3 | Training-time byte pins omit `navrl_task.py` (the receipt records it) and config/governor/detector (the receipt does not record them). They are currently identical to training head `22ed16f`; I verified this. | `camera_policy_contract_cams2.py:73-77` | — |

## CAM-K2 v3 unchanged

- The CAM-K2 path differs only by factoring the governor into `_govern` (`camera_policy_runtime.py:1476-1491`), with the same operations in the same order, and by the `published = governed` alias (`:1579-1581`).
- `cams2 = None` is a class attribute.
- The validation, delay-set and bound paths are unchanged for non-CAM-S2 lineages.
- All 329 tests pass, including `test_runtime_fingerprint` and the CAM-K2 runtime/contract suites.
