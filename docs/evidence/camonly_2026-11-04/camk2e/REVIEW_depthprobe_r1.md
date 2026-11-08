# Independent review r1: CAM-K2 depth probe r1 + contract audit r1

Reviewer: **Claude Opus 5.5 (fresh context)**, 2026-11-08. Read-only; wrote only this file. No GPU/CUDA, no Isaac Gym,
no checkpoint load. Source `MOTAR-camonly` at `ceab4a0d4a1df75098ec3b2e5b99cce76b910f8d`.
What I checked: the source (`navrl_depth_model.py`, `navrl_perception.py` 216-230 and 1835-2015 and 2184-2317,
`navrl_detector.py` 28-49, 363-405 and 927-940, `navrl_sensor_ablation.py`, `realveh_entry.py` 183-196,
`camtrain_entry.py`, `train_navrl_camk2v2.sh`, `navrl_task.py` 3650-3692 and 6160-6195, the v2 prereg), the
probe's `metrics.json` (re-tabulated per scene and arm, without re-running the probe), and the speedgov and
progress lines in the four training logs. Codex's `review_r1` already reproduced the geometry (max diff 1.2e-6 m) and
all 24 arm-seed runs, so I did not run the probe again.

## Verdicts

| Report | Verdict |
| --- | --- |
| `probe_r1` (synthetic same-frame probe) | **PASS_WITH_LIMITS** |
| `contract_r1` (source/log contract audit) | **PASS_WITH_LIMITS** |
| Combined claim "near-bar token loss at contact = MinZ 0.4 + invalid→no-return" | Supported for the **representation at the contact instant** (synthetic). **NOT supported** as a statement about crashes (F1). |

## Claim-by-claim

1. **Mechanism.** Confirmed in the per-scene data. The bar association is lost ON in exactly the scenes where
   most of the bar's pixels have z < 0.4 m (e.g. 14400/14400 sub-MinZ pixels). No loss happens in a scene without
   sub-MinZ pixels. At 30° bearing a slanted face keeps enough valid pixels (#5, #8, #26, #29, #50).
   `minz_off`, `deterministic_minz_off`, `invalid_encoding_bypass` and `near_clamp` all give 50/50. ON and
   deterministic ON both give 26/50, and both seeds give the same associations. The 0.39/0.41 m stripes bracket
   the threshold, and the 0.5-1.5 m stripes and all box scenes with min z ≥ 0.4 survive.
   Noise and holes are not the cause, **under the iid model only**: 2 % iid holes cannot empty a 90-row column, and
   real OAK holes are spatially correlated. MaxZ is inert by construction (input ≤ 10 m, MaxZ = max_camera_range = 20 m).
   Its "control" is the same call as ON, so it shows nothing experimentally (disclosed). The conclusion still
   holds from the source.
   Bilinear edge ranges are confirmed (3.73-6.15 m in #0, #1, #2, #7, #20), but **they also exist with the depth
   model OFF**: 16 of 50 OFF scenes have edge tokens of 3.9-6.5 m. MinZ removes the true core token and leaves only
   the edge one (F3).
2. **Contract corrections.** Confirmed in the source.
   - `hit_fov` = `|bearing| ≤ token_effective_fov/2` (240°), not camera visibility (`navrl_task.py:3676-3684`).
   - `hit_dist` = horizontal distance from the vehicle origin to the nearest bar **centre** (`:3658-3662`).
   - The governor clearance at contact is `_speed_governor_last["clearance_m"]` from the step's earlier command.
   - Tokens = `obstacle_history[:, -1]`, the pre-action observation.

   Comparing the governor clearance with hit_dist as an estimation error is invalid. One gap (F2).
3. **Geometry.** Confirmed: the `bars_h3` collision boxes are 0.4029-0.7902 m, so half-widths are 0.21-0.39 m.
   The camera translation is [0.10, 0, 0.03]. A face-on bar has z = 0.54 − 0.10 − half = 0.05-0.23 m < 0.4.
   More useful: a face-on bar vanishes below centre distance half + 0.50 m (0.61-0.79 m). If the vehicle's contact
   radius is r_v ≈ 0.54 − mean half (about 0.24 m, inferred), the blind approach is about 0.5 − r_v ≈ 0.26 m
   whatever the bar size. That is about 0.13 s at the executed 2.0 m/s, or 0.35 s at the logged contact speed
   0.74 m/s. Limit: 0.54 m is a mean over post-physics contacts, and the bars can be rotated rectangles.
4. **Faithfulness.** Good for bar-only frames.
   - The production `apply` and `_fuse_static_and_extract_obstacles` run inside `production(ENV)`, with the
     receipt's selector, FOV, bin, range and slot env.
   - The FOV comes from the production `apply_overrides`: fx = 123.19 = 160/(2 tan 33°), fy = 88.32.
   - The renderer conventions match: normalized rays, cx = (W−1)/2, no-hit = 10 m, bilinear with
     `align_corners=False`.
   - Bypassing `observe` is equivalent here. In production the depth wrapper (inner) changes only `depth` and the
     ablation (outer) only sets `lidar_m` to 12 m, which is the probe's input, so they commute. The ablation's
     `fuse` wrapper only records. With no target, the detection and carve-out mask is empty, so the zero
     `target_pixels` is faithful.

   Not exercised: a target in frame (an invalidated target is also undetected, because the mask needs
   depth < 20 m), latency/backfill carve paths, rotated or rectangular boxes, other bars, and the Warp renderer
   itself. The controls are non-vacuous, apart from the MaxZ identity arm.
5. **Next step.** See the recommendation below.

## Findings

- **F1 MAJOR (interpretation, both reports and the brief).** The 0.68→0.19 `hit_token` drop is mostly what MinZ
  does at contact distance by construction. It is not evidence that the depth model raised crashes. In the last
  progress window the crash fraction was K1 s1 0.728 (n = 2050) against K2v2 s1 0.710 and s2 0.712 (n ≈ 2051).
  Contacts per diagnostic window were 1491-1534 against 1455-1461 (log lines `navrl_task.py:9909/9989`).
  These are training windows: unpaired, exploratory, with the T/W 4.0 confound. The claim "the near-bar loss is
  caused by MinZ" should read "the token at the contact instant is removed by MinZ". The crash-level effect is
  NOT_TESTED, and the logs do not show it.
- **F2 MINOR (contract_r1).** Under `no_lidar` the governor's in-FOV scan is the camera-fused depth-model scan
  (`navrl_sensor_ablation.py:131-148`, `_ablation_fused_m`), so the MinZ blindness reaches the riskcap filter too.
  The prereg discloses this (l.445). The rise in contact-time clearance (K1 0.35-0.40 m → K2v2 1.09-1.25 m) is
  consistent with the governor seeing no return or edge ranges in the last ~0.26 m. That fits the same mechanism,
  but it is not an estimation error. The report should say so, not only that the comparison is invalid.
- **F3 MINOR (probe_r1).** The edge "inflated ranges" were present before the depth model: 16/50 OFF scenes
  carry 3.9-6.5 m phantom tokens. This is the known "EQ-1D upsampling phantoms" limit (prereg l.448), and the
  report does not cite it. The phantoms also affect F and K1. MinZ adds nothing to them, but it makes them the only
  token in 5 scenes.
- **F4 LOW (probe_r1).** 18 of the 54 boxes use width 0.30 m, below the smallest real bar (0.40 m).
  Axis-aligned square boxes only. The 26/50 count is a descriptive grid count (stated). For the realistic width
  0.5 m, 9 of 18 scenes lose the association.
- **F5 LOW.** "Holes are not the cause" holds only for iid holes. Real stereo invalidity is correlated: occlusion
  bands at depth edges, texture-less faces, and IR or sunlight washout.

## Recommendation

- **Is evaluating v2 as trained meaningful?** Yes, as a characterisation of the disclosed optimistic convention.
  The prereg discloses it in its limits (l.445) and lists it as open decision 3. Label it that way.
  - Do not use `hit_token` at contact as a perception metric across K1 and K2.
  - Add a contact-aligned near-zone diagnostic: the token and governor input in the last N steps before contact.
  - Never re-evaluate v2 with a repaired sensor under the v2 name.
- **Deployable invalid-depth contract for the OAK-D Pro** (0 = invalid; too-near cannot be told from a hole).
  These are candidates; each needs qualification.
  - **(a) Status quo, 0 → no return.** It matches v2 training. Risk: the policy and the governor go blind for about
    the last 0.26 m, and the real invalid regions are larger and correlated, so a bar can vanish at range too.
  - **(b) Blob rule.** An invalid connected region larger than k px reads as a surface at MinZ; an isolated pixel
    reads as no return. It is deployable, but:
    - k depends on the resolution, so define it on the native frame before downsampling;
    - sky, out-of-range, texture-less walls and sunlight produce large blobs, which become phantom walls at 0.4 m
      (over-braking or freezing);
    - in simulation the only large blobs come from MinZ, so a policy trained there learns that a blob always means
      near. The sim-to-real gap gets worse.

    To reduce the risk, require valid border pixels within about 1 m, or use the Pro's IR intensity (a too-near
    surface is bright or saturated).
  - **(c) Temporal hold in scan space.** When a bin's return below about 1 m becomes no return, hold it, carried
    with the ego-motion (odometry), for at most T s. It is physically motivated, but:
    - it needs good odometry and yaw compensation;
    - it leaves stale phantoms;
    - it gives nothing for a bar that enters the view already inside MinZ (a yaw sweep or lateral approach);
    - it is wrong for moving obstacles.

    Lowest-risk first step: apply (c), or (b) followed by a MinZ clamp, to the **governor input only**. That is an
    opt-in layer, default OFF, with an OFF = native check. It leaves the actor's observation unchanged, so it can be
    evaluated on the v2 checkpoints without retraining (with user approval). Changing the actor's input means v3.
  - **(d) Hardware.** MinZ = fB/d_max. 400P with extended disparity roughly halves MinZ to about 0.17-0.2 m
    (formula estimate; the unit's datasheet and calibration are NOT_VERIFIED here). It doubles σz.
- **Before any v3:** record an OAK-D bench approach to a bar from 1.5 m down to 0.1 m, plus sky, a texture-less
  wall and sunlight. Use it to calibrate k and the hold, and qualify the chosen contract in simulation with
  no-return, iid-hole, correlated-hole, too-near and mixed-edge controls.
