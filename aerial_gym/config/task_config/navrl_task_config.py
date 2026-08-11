import os
import sys


def _warn_bad_env(name, raw, default):
    # A malformed knob (e.g. NAVRL_K_FINAL=abc or a stray unit like "1.5m") used to be swallowed
    # silently and trained on the default — a nasty invisible confound. Make it loud instead.
    print(
        "[navrl config] WARNING: %s=%r could not be parsed; using default %r."
        % (name, raw, default),
        file=sys.stderr,
    )


def _env_int(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return int(default)
    try:
        return int(raw)
    except ValueError:
        _warn_bad_env(name, raw, default)
        return int(default)


def _env_float(name, default):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return float(default)
    try:
        return float(raw)
    except ValueError:
        _warn_bad_env(name, raw, default)
        return float(default)


def _env_bool(name, default=False):
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return bool(default)
    s = raw.strip().lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off"):
        return False
    # Tolerate numeric forms like "1.0"/"0.0" (a common footgun: NAVRL_VISION=1.0 used to be False).
    try:
        return float(s) != 0.0
    except ValueError:
        _warn_bad_env(name, raw, default)
        return bool(default)


def _perception_obs_dim():
    """Structured observation width, imported from navrl_perception (the single source of truth).

    Imported lazily inside the function: this config module is loaded by every NavRL task, while
    navrl_perception pulls in torch and is only meaningful in perception mode. Falls back to the
    36-beam / 5-token default so a non-perception run never fails on an unrelated import.
    """
    try:
        from aerial_gym.task.navrl_task.navrl_perception import STRUCTURED_OBS_DIM

        return int(STRUCTURED_OBS_DIM)
    except Exception:
        return 574


class task_config:
    """NavRL reimplementation: static bar field + capture task (Phases 1-3).

    Observation and reward follow NavRL (Xu et al., RA-L 2025), static branch, adapted for
    interception-style capture and a (Phase 3) moving target:
      state  = S_int (12: goal-frame direction/distance/velocity 8 + body-frame goal bearing 2
               + body-frame velocity 2) concatenated with the flattened 36x4 LiDAR scan -> 156.
      action = 4-D network output. x/y are vehicle-frame velocity commands with an independent
               per-axis +/- max_velocity limit, z is retained for checkpoint compatibility but
               is overwritten by the task-level altitude PI loop, and yaw commands yaw-rate.
      reward = range_rate + time_cost(alive<0) + static_safety - smooth - height - yaw_align
               - yaw_rate_damping + ego_progress, with a terminal +capture_bonus (episode ends
               on capture) and collision_penalty on a crash. With a static target (target_motion
               speed 0, the default) range_rate reduces exactly to NavRL's velocity-toward-goal.
    Dynamic obstacles are added in a later phase.
    """

    # NOTE: --seed only reaches the task in --train mode; the rl_games PLAYER drops it. For
    # reproducible evals use NAVRL_SEED=<n> ./play_navrl.sh ... instead.
    seed = _env_int("NAVRL_SEED", 42)
    # Default "base_sim" (8 GB main machine). A 4 GB secondary machine sets
    # AERIAL_GYM_SIM_NAME=base_sim_4gb (via GPU4GB=1 ./train_navrl.sh) to shrink the PhysX
    # GPU buffers so training fits in 4 GB VRAM. See base_sim_4gb_config.py.
    sim_name = os.environ.get("AERIAL_GYM_SIM_NAME", "base_sim")
    # Controlled arena: empty space + density-controlled static bars (no walls/panels).
    # See navrl_bars_env.py and class density below.
    env_name = "navrl_bars_env"
    robot_name = "navrl_quad"
    controller_name = "lee_velocity_control_navrl"  # NavRL-scoped: raises yaw-rate clamp pi/3 -> 2.5
    args = {}
    num_envs = 256
    use_warp = True
    headless = True
    device = "cuda:0"

    # LiDAR scan geometry -- must stay in sync with NavRLLidarConfig.
    # MUST match navrl_lidar_config.width/height and navrl_perception.HBEAMS/VBEAMS -- all three
    # read the same env vars. Doubling the horizontal beams halves the angular quantization that
    # dominates obstacle-token position error (see navrl_perception.HBEAMS).
    lidar_hbeams = _env_int("NAVRL_LIDAR_HBEAMS", 36)
    lidar_vbeams = _env_int("NAVRL_LIDAR_VBEAMS", 4)
    lidar_max_range = _env_float("NAVRL_LIDAR_RANGE", 4.0)  # must match NavRLLidarConfig.max_range

    # ------------------------------------------------------------------ Phase-3 vision pivot
    class vision:
        """NAVRL_VISION=1: the ACTOR perceives the target only through its sensors -- no ground-
        truth target position in the policy observation. Reward/termination/critic keep GT
        (privileged reward shaping + asymmetric critic are standard and do not leak into the
        deployed actor). Default OFF keeps the verified Phases 1-2 injected-goal task
        byte-compatible.

        Perception channels:
          1. semantic LiDAR: the 36x4 range image observes both environment geometry and the
             target; a second per-ray channel marks target returns.
          2. forward depth/segmentation CAMERA: a low-resolution full-scene depth image observes
             obstacles, while a higher-resolution target mask/depth image provides detection.
             Bars remove occluded target pixels, and bearing/elevation/range are calculated only
             from surviving pixels. The target pose is renderer-only and never enters the actor.
          3. a detector-side tracker (last-seen bearing + time-since-seen) -- the standard
             onboard filter memory (NavRL itself tracks dynamic obstacles onboard).
        """

        enable = _env_bool("NAVRL_VISION", False)
        # Read-only checkpoint compatibility for the 2026-07 semantic CNN runs. Those actors were
        # trained before the 40x24 obstacle-camera channel was added and therefore require the
        # original 17 + 144 + 144 = 305 observation exactly. Never use this for new training.
        legacy_actor_305 = _env_bool("NAVRL_LEGACY_VISION", False)

        # -- forward target camera (gimbal-level like the yaw-only LiDAR attach)
        detector_max_range = 20.0   # [m] detection range (a segmentation camera covers this)
        detector_hfov_deg = 87.0    # matches the D455-style forward depth camera
        detector_vfov_deg = 58.0
        tracker_memory_s = 5.0      # time_since_seen saturates at this many seconds
        camera_width = 160          # efficient target-only semantic render; aspect ratio 16:9
        camera_height = 90
        camera_target_radius = 0.15 # [m], matches the 0.30 m target-drone footprint
        camera_min_target_pixels = 1
        camera_translation = [0.10, 0.0, 0.03]  # vehicle frame, forward/left/up [m]
        # -- appearance domain shift (검증 2, WORKLOG 2026-08-12). The renderer paints a flat
        # pure-red target over a depth-shaded neutral background, so the pixel classes are
        # trivially separable; these knobs measure how much of the detector results survive when
        # that stops being true. All default 0 = bit-identical nominal render. Per-EPISODE draws
        # (resampled in detector.reset_idx) except the intrinsics error, which is per-run because
        # the ray table is built once.
        appearance_hue_deg = _env_float("NAVRL_APP_HUE_DEG", 0.0)        # target hue rotation, ±deg
        appearance_light_gain = _env_float("NAVRL_APP_LIGHT_GAIN", 0.0)  # global illumination, ±fraction
        appearance_albedo_jitter = _env_float("NAVRL_APP_ALBEDO_JITTER", 0.0)  # bar/background reflectance, ±fraction
        appearance_texture_std = _env_float("NAVRL_APP_TEXTURE_STD", 0.0)  # static per-pixel luminance noise std
        appearance_motion_blur = _env_float("NAVRL_APP_MOTION_BLUR", 0.0)  # EMA weight of the previous frame [0,1)
        # Calibration error, RENDERER-ONLY by construction: the perception module keeps its own
        # nominal intrinsics/extrinsics copy (navrl_perception.py), so perturbing the renderer
        # copy makes every downstream consumer back-project with the wrong model -- which is what
        # a real mis-calibration does. Perturbing both sides would cancel.
        camera_mount_rot_deg = _env_float("NAVRL_CAM_MOUNT_ROT_DEG", 0.0)   # mount rotation error, ±deg per env
        camera_mount_trans_m = _env_float("NAVRL_CAM_MOUNT_TRANS_M", 0.0)   # mount translation error, ±m per env
        camera_fov_scale_err = _env_float("NAVRL_CAM_FOV_SCALE_ERR", 0.0)   # FOV scale error, fraction, per run
        # Full-scene depth supplied to the actor. Kept lower-resolution than target detection to
        # preserve vectorized PPO throughput while still adding forward obstacle geometry.
        camera_obstacle_width = 40
        camera_obstacle_height = 24
        camera_obstacle_max_range = 10.0
        camera_obstacle_dim = camera_obstacle_width * camera_obstacle_height

        # -- reward add-ons (vision mode only)
        visibility_bonus = 0.02     # per-step bonus while the detector sees the target
        # The arena edge is an artificial task boundary, not physical geometry, so LiDAR cannot
        # see it. Keep the historical default while allowing checkpoint-only sensitivity sweeps
        # (no retraining) with NAVRL_OOB_MARGIN=1.0, 2.0, ...
        oob_margin = _env_float("NAVRL_OOB_MARGIN", 0.5)

        # -- cold-start visibility curriculum (vision mode only)
        # The target token is all-zero until the detector first ACQUIRES the target: the KF only
        # activates once the target enters FOV+range (navrl_perception.py). A from-scratch policy
        # whose goal starts OUTSIDE the camera FOV therefore never receives a bearing, cannot learn
        # to approach, and just drifts out of bounds (~100% crash, no gradient -- the exact failure
        # seen on the first perception run). Fix: constrain the goal's INITIAL bearing to the
        # detector FOV, then widen to the full arena over this many epochs. This shapes only the
        # episode initial condition (like the k-distance curriculum) -- it is NOT a GT leak into the
        # actor. Set NAVRL_FOV_CURRICULUM_EPOCHS=0 to disable (full-arena goals from step 0).
        fov_curriculum_epochs = _env_int("NAVRL_FOV_CURRICULUM_EPOCHS", 3000)
        spawn_yaw_max_deg = 30.0    # matches navrl_quad_config spawn yaw (+/-30 deg); FOV headroom

        # -- observation layout (actor)
        ego_dim = 9        # vel_vehicle(3) + yaw_rate(1) + prev_action(4) + height(1)
        detector_dim = 8   # visible, bearing sin/cos, elev, range | last bearing sin/cos, t_since
        privileged_dim = 8 # critic-only extras: rpos_unit_veh(3), dist, target_vel_veh(3), closing

    class perception:
        """Real sensor-to-track path used by the NavRL++-Target Transformer.

        Enable with ``NAVRL_VISION=1 NAVRL_PERCEPTION=1``. The legacy semantic prototype remains
        available only for baseline/checkpoint compatibility when NAVRL_PERCEPTION is unset.
        """

        enable = _env_bool("NAVRL_PERCEPTION", False)
        history_interval_s = 0.5
        # NOTE: history_steps and max_obstacles do NOT live here. They are ROBOT_HISTORY /
        # MAX_OBSTACLES in navrl_perception.py because they define STRUCTURED_OBS_DIM (and therefore
        # the network's token layout), so a single source of truth is mandatory. Duplicates used to
        # sit here and were read by nobody -- editing them looked like tuning but changed nothing.
        # Sweep the token capacity with NAVRL_MAX_OBSTACLES (requires a fresh policy: obs dim changes).
        lidar_max_range = _env_float("NAVRL_LIDAR_RANGE", 4.0)  # obstacle horizon; matches the LiDAR sensor
        min_target_pixels = _env_int("NAVRL_DETECTOR_MIN_PIXELS", 2)
        pixel_threshold = _env_float("NAVRL_DETECTOR_THRESHOLD", 0.55)
        detector_checkpoint = os.environ.get("NAVRL_DETECTOR_CHECKPOINT", "")
        # Perturbations are opt-in for the post-training stage, not silently applied to clean runs.
        enable_perturbations = _env_bool("NAVRL_PERCEPTION_PERTURB", False)
        detection_dropout_prob = _env_float("NAVRL_DETECTION_DROPOUT", 0.3)
        detection_latency_s = _env_float("NAVRL_DETECTION_LATENCY_S", 0.0)
        range_error_m = _env_float("NAVRL_RANGE_ERROR_M", 0.0)
        # Latency COMPENSATION (fix), deliberately separate knobs from the latency PERTURBATION
        # above: an eval arm sets the perturbation to model a slow pipeline and toggles these to
        # measure how much of the loss the perception-side fix recovers (WORKLOG 2026-08-05 R3).
        # P0: output-side constant-velocity forward predict by detection_latency_s.
        latency_compensate = _env_bool("NAVRL_LATENCY_COMPENSATE", False)
        # P1: a DELAYED camera detection no longer vetoes the fresh-LiDAR correction path.
        latency_lidar_backup = _env_bool("NAVRL_LATENCY_LIDAR_BACKUP", False)
        # P2: the obstacle map is edited with the delayed target bearing/range/pixels
        # (target_like carve-out + depth blanking in _fuse_static_and_extract_obstacles), so a
        # stale detection erases LiDAR returns where a real BAR stands. This is the channel that
        # P0/P1 did not touch and that tripled bar contacts under 0.1 s latency (WORKLOG
        # 2026-08-05). "predict" re-derives the edit location from the tracker's forward-predicted
        # target; "skip" simply stops editing the map while the detection is stale (the target
        # then survives as an obstacle, but no real bar is ever deleted).
        latency_obstacle_fix = os.environ.get("NAVRL_LATENCY_OBSTACLE_FIX", "off").strip().lower()
        # P3, DEFAULT ON: a delayed detection is a VEHICLE-frame measurement taken at t-tau, but
        # lifting it to world with the pose at t injects the drone's own motion over tau into
        # every KF correction -- 0.23 m of translation at the measured 2.33 m/s mean speed and
        # 0.41 m from yaw at 0.81 rad/s, both larger than the 0.15 m target lag P0 chased.
        # Buffering the pose alongside the detection and lifting with it is what a real pipeline
        # does (timestamped measurements + IMU/odometry), so the naive lift was not a "latency
        # model", it was an extra unmodelled error on top of one. It is default ON because it is
        # the CORRECT model, not a compensation: with it, 0.1 s latency costs 2.5 pp of capture
        # instead of 42.7 pp (WORKLOG 2026-08-06). Arithmetically a no-op at zero latency, so
        # clean results and every pre-2026-08-06 non-latency number are unaffected. Set
        # NAVRL_LATENCY_EGO_MOTION_FIX=0 only to reproduce the superseded R3 latency arms.
        latency_ego_motion_fix = _env_bool("NAVRL_LATENCY_EGO_MOTION_FIX", True)
        # The obstacle map is edited by two paths with different gates: the LiDAR target_like
        # carve-out uses fused visibility (camera OR LiDAR) while the depth blanking uses the
        # camera-only pixel mask. On any frame the camera misses but LiDAR still holds the track
        # -- i.e. 30% of frames under detection dropout -- the LiDAR half deletes the target and
        # the camera half reinstates it, leaving a phantom obstacle straight ahead that eats one
        # of the 8 obstacle tokens. Enabling this rebuilds the mask from the fused bearing/range
        # so both halves agree. Off until the A/B measures it (WORKLOG 2026-08-07).
        target_mask_backfill = _env_bool("NAVRL_TARGET_MASK_BACKFILL", False)
        # H2 probe (WORKLOG 2026-08-07): on a camera-missed frame the LiDAR association is the
        # only thing correcting the target track, and its bearing is quantised to one 360/HBEAMS
        # bin -- 5 deg, i.e. 0.44 m of lateral error at 5 m. Set NAVRL_LIDAR_TARGET_ASSOC=0 to
        # coast on the constant-velocity prediction instead and measure whether the coarse
        # correction is helping or hurting. Default on = current behaviour.
        lidar_target_assoc = _env_bool("NAVRL_LIDAR_TARGET_ASSOC", True)
        # H3 (WORKLOG 2026-08-07): the LiDAR target correction measures only a range along a
        # bearing the tracker itself predicted, yet it is applied as a full 3-D update, so the
        # lateral and vertical covariance shrink on information that was never observed. The
        # filter then reports ~0.09 m of lateral sigma while the true error reaches 3.27 m, and
        # the policy reads that covariance. Enabling this shapes R about the measurement ray so
        # only the range informs the filter. Off until the A/B measures it.
        lidar_range_only_update = _env_bool("NAVRL_LIDAR_RANGE_ONLY_UPDATE", False)
        # The LiDAR association gate is (0.35 + 2*pos_sigma) clamped at 1.0 m, i.e. it widens as
        # the track gets less certain -- so making the covariance honest (above) widens the
        # mis-association window with it, which measurably cancels that fix. Set a constant gate
        # in metres to decouple the two; 0 keeps the covariance-scaled behaviour.
        lidar_assoc_gate_m = _env_float("NAVRL_LIDAR_ASSOC_GATE_M", 0.0)
        # H4 (WORKLOG 2026-08-10): correct() resets time-since-seen and observe() ORs the
        # association into fused visibility, so the policy is told "visible, just seen" about a
        # measurement whose bearing is the filter's own prediction. This keeps the range
        # correction but silences those flags; if it matches H2's gain, the flags are the
        # channel and the state update itself is innocent.
        lidar_silent_correct = _env_bool("NAVRL_LIDAR_SILENT_CORRECT", False)
        rgb_noise_std = _env_float("NAVRL_RGB_NOISE_STD", 0.015)
        depth_noise_std = _env_float("NAVRL_DEPTH_NOISE_STD", 0.02)
        # [static VBEAMS*HBEAMS | obstacle history 5*MAX_OBSTACLES*12 | robot history 5x10 |
        #  target history 5x16]. The network converts these to 17 tokens.
        # DERIVED, never hard-coded: this used to be a literal 574, which silently contradicted
        # navrl_perception the moment the scan resolution or obstacle capacity was swept (the task
        # asserts the two agree at startup, so a stale literal aborts the run).
        observation_dim = _perception_obs_dim()

    # Observation:
    #   default      = S_int(12) + LiDAR range(144)                            = 156
    #   NAVRL_VISION = ego+detector(17) + LiDAR range+target(288)
    #                  + camera obstacle depth(40x24=960)                      = 1265
    #   critic states = actor obs + privileged extras(8)                       = 1273
    if vision.enable and perception.enable:
        internal_state_dim = perception.observation_dim
        observation_space_dim = perception.observation_dim
        state_space_dim = observation_space_dim + vision.privileged_dim
    elif vision.enable:
        internal_state_dim = vision.ego_dim + vision.detector_dim  # network state/scan split
        observation_space_dim = (
            internal_state_dim
            + 2 * lidar_hbeams * lidar_vbeams
            + (0 if vision.legacy_actor_305 else vision.camera_obstacle_dim)
        )
        state_space_dim = observation_space_dim + vision.privileged_dim
    else:
        internal_state_dim = 12  # (b) 8 nav dims + goal_bearing_body(2) + vel_body_xy(2)
        observation_space_dim = internal_state_dim + lidar_hbeams * lidar_vbeams
        state_space_dim = 0  # no asymmetric critic in the injected-goal task

    # Four network outputs are preserved by the frozen 898-D policy contract. In perception mode,
    # x/y command vehicle-frame velocity and yaw commands yaw-rate. The z output has no direct
    # actuator authority because altitude PI overwrites it, but the raw z request is retained in
    # the next observation's prev_action. It is therefore an indirect policy-state channel, not a
    # dead dimension. Removing or replacing it requires a fresh-policy observation/action ablation.
    action_space_dim = 4

    episode_len_steps = _env_int("NAVRL_EPISODE_LEN_STEPS", 300)  # RL steps (300 for far goals;
    # ~150 steps straight / ~225 weaving at 2 m/s, so 300 keeps timeout from being the failure mode)

    # Per-axis command limit, not a vector-norm limit. The attainable horizontal norm is
    # sqrt(2)*max_velocity (3.54 m/s in v2), which is why the riskcap free-speed setting uses that
    # value. Renaming the environment variable would break old launch recipes, so document the
    # actual semantics here and in the dashboard.
    max_velocity = _env_float("NAVRL_MAX_VELOCITY", 2.0)  # [m/s per commanded axis]

    # Vertical authority of the task-level altitude hold, DELIBERATELY independent of max_velocity.
    # It used to reuse max_velocity for both the vz clamp and the PI anti-windup bound, which made a
    # pursuer-speed sweep confound two effects: a 0.75 m/s sweep point lost ~70% of its altitude-hold
    # authority, so "slower pursuer crashes less" was inseparable from "slower pursuer cannot hold
    # altitude". Keep this fixed across speed sweeps so only the horizontal speed varies.
    alt_hold_vmax = _env_float("NAVRL_ALT_HOLD_VMAX", 2.5)  # [m/s] vz clamp for the altitude hold

    # (b) Learned yaw control. yaw_rate_max MUST equal lee_controller_config_navrl.max_yaw_rate so
    # action[:, 3] in [-1, 1] maps linearly onto the controller's yaw-rate clamp (no dead band).
    yaw_rate_max = _env_float("NAVRL_YAW_RATE_MAX", 2.5)  # [rad/s] max euler yaw-rate for action[:,3];
    #   raise (e.g. 3.0) to let a FASTER drone turn its heading fast enough to weave between bars --
    #   at 2.0 m/s the weave already demands ~2.4 rad/s, so yaw-rate (not tilt/thrust) is the binding
    #   maneuverability limit. MUST stay equal to lee_controller_config_navrl.max_yaw_rate (same env var).
    yaw_align_speed_ref = 1.0   # [m/s] speed at which the crab-alignment penalty reaches full weight

    # 2D navigation: the drone flies at this fixed altitude and tracks the goal in XY only.
    # The drone spawns here (navrl_quad init z-ratio) and the task zeroes the vertical velocity
    # command; the goal is placed (and, in Phase 3, moves) at this same altitude.
    flight_altitude = 1.0  # [m]

    # Goal placement: "cross the bar field" -- the drone spawns at the left edge (x~0) and the
    # goal is placed on the far side at x=k, so every episode traverses the whole bar field.
    # k grows epoch-proportionally: near goals first, then progressively deeper crossings.
    class curriculum:
        # The drone spawns at x~0 and the goal is placed at x=k on the far side, so every episode
        # must traverse the bars (band x in ~[3.1, 23.0]). k grows with training (epoch-proportional).
        #   goal x ~ U[k_min, k_max(t)],  goal y ~ U[wall_margin, arena_y - wall_margin]
        #   k_max(t) = k_start + (k_final - k_start) * min(1, epoch / k_warmup_epochs)
        # k_final / k_min_final are env-overridable so a SENSOR-ONLY run can keep goals inside the
        # detector's reliable range (vision.detector_max_range = 20 m). Deep goals (24 m) at high
        # density made the sensor-only policy over-cautious and time out; NAVRL_K_FINAL=16
        # NAVRL_K_MIN_FINAL=10 keeps every goal perceivable. Defaults reproduce the LiDAR task.
        k_min = 5.0              # [m] initial nearest goal x (shallow: just inside the bar band, ~3.1 m)
        k_min_final = _env_float("NAVRL_K_MIN_FINAL", 20.0)  # [m] LATE nearest goal x. k_min ramps
        #   k_min -> k_min_final linearly over [k_min_ramp_start_epochs, +k_min_ramp_epochs],
        #   narrowing the goal window to deep crossings. Set k_min_final = k_min to disable.
        k_min_ramp_start_epochs = _env_int("NAVRL_K_MIN_RAMP_START", 2000)
        # epoch k_min STARTS rising. Independent of the k_max ramp --
        #   it overlaps it (k_max keeps ramping to k_warmup_epochs=3000 while k_min already climbs).
        k_min_ramp_epochs = _env_int("NAVRL_K_MIN_RAMP_EPOCHS", 3000)
        # epochs to ramp k_min -> k_min_final => default hits max at 2000+3000 = 5000
        k_start = 7.0            # [m] initial k_max (first bar rows)
        k_final = _env_float("NAVRL_K_FINAL", 24.0)  # [m] final k_max (far wall; clamped to arena-margin)
        k_warmup_epochs = _env_int("NAVRL_K_WARMUP", 3000)  # epochs to ramp k_start->k_final, then plateau
        ppo_horizon = 32         # rl_games horizon_length (MUST match ppo_navrl_cnn.yaml) -> steps/epoch
        wall_margin = 0.5        # [m] keep drone/goal this far from the y walls

        # --- Competence-gated distance mode (NAVRL_K_COMPETENCE=1) ---------------------------------
        # The k_max/k_min ramps above are EPOCH-proportional: the goal deepens on a clock regardless
        # of skill. Measured failure -- at capture ~1% the window still auto-ramped 5-8 m -> 15-24 m,
        # every episode then ended in a crash, and the value function collapsed (no capture gradient
        # survives at a depth the policy cannot reach). Lowering the slope only DELAYS this: a time
        # ramp still eventually outruns a plateaued policy, and the right slope differs per run. This
        # mode instead advances the window ONLY when measured capture clears k_comp_threshold over
        # k_comp_check finished episodes -- exactly how the density curriculum self-paces -- and just
        # PAUSES (logging "held") when the policy stalls. Off => the epoch ramp above (LiDAR default).
        use_competence = _env_bool("NAVRL_K_COMPETENCE", False)
        k_comp_threshold = _env_float("NAVRL_K_THRESHOLD", 0.6)  # capture rate needed to deepen the goal
        k_comp_step = _env_float("NAVRL_K_STEP", 2.0)            # [m] deepen k_max by this per promotion
        k_comp_check = _env_int("NAVRL_K_CHECK", 2048)           # finished episodes per competence check

    # Phase 2 density sweep: obstacle size stays fixed; only the active bar count changes.
    # NAVRL_MAX_BARS controls the build-time ceiling in env_object_config.py.
    class density:
        # Density (clutter) curriculum: start sparse, add promote_step bars each time the capture
        # rate over check_after_episodes clears success_threshold, until n_final. Orthogonal to the
        # goal-DISTANCE curriculum above (that one ramps by epoch; this one ramps by capture).
        # All knobs are env-overridable because the defaults were tuned for the GT-injected LiDAR
        # task; the sensor-only (vision) task caps lower at high density and masters sparse density
        # fast, so it wants a SHORTER warmup and a LOWER threshold (see the vision launch recipe).
        use_density_curriculum = _env_bool("NAVRL_DENSITY_CURRICULUM", False)
        num_bars_active = _env_int("NAVRL_NUM_BARS", 48)
        n_start = _env_int("NAVRL_DENSITY_START", 25)
        n_final = _env_int("NAVRL_DENSITY_FINAL", 150)
        success_threshold = _env_float("NAVRL_DENSITY_THRESHOLD", 0.8)
        # Optional per-density threshold ramp: require MORE capture to promote out of the easy,
        # sparse end (build a solid foundation before compounding difficulty) and LESS to promote
        # out of the hard, dense end (a flat threshold can strand the curriculum forever if the
        # achievable capture ceiling keeps dropping with density -- this is exactly the failure
        # mode behind the observed v1 100-bar density-ceiling plateau). Both default to the flat
        # success_threshold above, so unset envs reproduce the old constant-threshold behavior
        # exactly. Interpolated linearly over [n_start, n_final] by current bar count.
        success_threshold_start = _env_float("NAVRL_DENSITY_THRESHOLD_START", success_threshold)
        success_threshold_end = _env_float("NAVRL_DENSITY_THRESHOLD_END", success_threshold)
        # Explicit per-density gate, e.g. "70:0.82,85:0.77,100:0.72,115:0.70". Overrides the
        # linear ramp above when set. The achievable ceiling is NOT linear in density -- at 70
        # bars it was measured at 0.843 while a straight line to the dense end would have demanded
        # far more than that at every level in between -- so the schedule exists to encode the
        # measured shape instead of a two-point guess. Step semantics: the highest knot at or
        # below the active bar count.
        success_threshold_schedule = os.environ.get(
            "NAVRL_DENSITY_THRESHOLD_SCHEDULE", ""
        ).strip()
        promote_step = _env_int("NAVRL_DENSITY_STEP", 15)
        warmup_epochs = _env_int("NAVRL_DENSITY_WARMUP", 2500)
        check_after_episodes = _env_int("NAVRL_DENSITY_CHECK_EPS", 2048)
        # Minimum epochs to DWELL at a density before promotion is allowed, even when the capture
        # gate already passes. Without it the curriculum can chain promotions the moment each
        # evidence window fills, so the policy is pushed to the next difficulty while still
        # improving at the current one -- the reward curve then only ever measures rising
        # difficulty. Dwelling lets each level converge before it is replaced. 0 = old behavior.
        min_epochs_per_density = _env_int("NAVRL_DENSITY_MIN_EPOCHS", 0)
        # Optional broad-slice guard. The ordinary aggregate gate can be dominated by short/slow
        # trials, so record speed, initial-distance and motion-pattern marginals on the exact same
        # evidence window. Enforcement remains opt-in until a fixed-density baseline establishes a
        # defensible floor; diagnostics are always emitted.
        use_stratified_gate = _env_bool("NAVRL_DENSITY_STRATIFIED_GATE", False)
        stratified_floor = _env_float("NAVRL_DENSITY_STRATIFIED_FLOOR", 0.55)
        stratified_min_episodes = _env_int("NAVRL_DENSITY_STRATIFIED_MIN_EPS", 512)
        # Legacy left-to-right spawn only: keep a fraction of short/medium crossings in a batch.
        # General-spawn training already samples its explicit radial range and does not use this
        # k-window mix. Zero preserves old runs.
        easy_goal_mix_prob = _env_float("NAVRL_DENSITY_EASY_GOAL_MIX", 0.0)
        easy_goal_min = _env_float("NAVRL_DENSITY_EASY_GOAL_MIN", 5.0)
        easy_goal_max = _env_float("NAVRL_DENSITY_EASY_GOAL_MAX", 10.0)

    # Phase 3 moving target (RQ2). The target is a VIRTUAL point (task-side coordinates, no
    # actor/mesh -> VRAM neutral; the LiDAR never sees it). speed_final = 0 (default) keeps the
    # target STATIC and the whole task byte-compatible with Phases 1-2.
    #   train:  NAVRL_TARGET_SPEED_FINAL=1.5 ./train_navrl.sh      (curriculum 0 -> 1.5 m/s)
    #   eval:   NAVRL_TARGET_SPEED=1.0 NAVRL_TARGET_PATTERN=cv ./play_navrl.sh <ckpt>  (exact cell)
    class target_motion:
        # Per-episode speed ~ U[speed_min, v_max(epoch)]; speed_min=0 keeps static/slow episodes
        # in-distribution for the default curriculum.
        # v_max(epoch) = speed_final * clamp((epoch - ramp_start) / ramp_epochs, 0, 1).
        # The epoch proxy is num_task_steps / ppo_horizon (checkpoint-persisted, resume-safe).
        speed_final = _env_float("NAVRL_TARGET_SPEED_FINAL", 0.0)  # [m/s] curriculum ceiling; 0 = static
        # Optional positive floor for generalized moving-target training. Zero preserves every
        # existing static-target experiment unless a launch recipe explicitly enables it.
        speed_min = _env_float("NAVRL_TARGET_SPEED_MIN", 0.0)      # [m/s]
        speed_ramp_start_epochs = 0
        # Epochs to reach speed_final. The v1 ramp (3000) existed so the policy could learn
        # interception against a STATIC target first. In v2 the target must be FOUND before it can
        # be intercepted, so target speed is no longer the dominant early-difficulty term, and a
        # time-based ramp only entangles the speed axis with the capture-paced density curriculum
        # (both indexed by num_task_steps) -- a variable-control defect. Set to 1 to hold the full
        # U[speed_min, speed_final] distribution from epoch 0. Default 3000 preserves v1 runs.
        speed_ramp_epochs = _env_int("NAVRL_TARGET_SPEED_RAMP_EPOCHS", 3000)
        # Evaluation override: force the EXACT per-episode speed (heatmap cells). < 0 disables.
        speed_fixed = _env_float("NAVRL_TARGET_SPEED", -1.0)      # [m/s]
        # Trajectory pattern: "cv" (constant velocity, reflected at the wall margins),
        # "waypoint" (random waypoints), "circle" (eval-only held-out pattern),
        # "mixed" (cv/waypoint 50:50 per episode -- the training default).
        pattern = os.environ.get("NAVRL_TARGET_PATTERN", "mixed").strip().lower()
        waypoint_reach_m = 0.5   # [m] resample the waypoint when the target gets this close
        circle_radius = 2.5      # [m] circle pattern radius around the spawn point

    # Success / termination.
    # NavRL's own reach_goal condition is distance < 0.5 m (env.py:583) — an interception-
    # grade capture radius (two ~0.3 m quads with centers 0.5 m apart nearly touch).
    # Interception semantics (deliberate divergence from NavRL, whose navigation env never
    # terminates on reach): touching the capture radius ENDS the episode as a success. The
    # capture test is a swept-segment test in the target-relative frame, so a fast fly-through
    # (closing speed up to 4 m/s = 0.4 m/step) cannot tunnel between samples.
    success_radius = 0.5  # [m]
    # Keep the goal at least this far (XY) from the nearest bar so the capture sphere is
    # actually flyable; also used to push a MOVING target back out of bar clearance.
    goal_min_bar_clearance = 1.0  # [m]
    lower_height_bound = 0.1  # [m] crash if below
    upper_height_bound = 4.0  # [m] crash if above (NavRL uses 4)

    # Ego-motion progress discount. Under a static target this has the standard PBRS algebra and
    # gamma must match PPO. Under a moving target the implementation deliberately re-anchors both
    # distances to target_(t+1) to avoid crediting exogenous target motion; that makes it a
    # heuristic rather than formally policy-invariant PBRS. Do not claim the PBRS theorem for v2.
    progress_gamma = 0.99

    # Reward weights. NavRL's static branch (env.py) is:
    #   r = 1*reward_vel + 1(alive) + 1*r_safety_static - 0.1*penalty_smooth - 8*penalty_height
    #
    # NavRL keeps a constant +1 "alive" survival bonus (present in code, absent from paper
    # Eqn. 7) so that flying stays net-positive even when the safety/height terms go negative
    # near obstacles — the classic antidote to a "suicidal agent" that would crash early to
    # stop accumulating negative reward. It is safe there because NavRL's ONLY terminations are
    # bad (collision / out-of-bounds); reaching the goal is never a termination (env.py:583-587
    # feeds reach_goal to stats only). Once we terminate on capture, a positive per-step alive
    # bonus flips from protective to harmful: ending the episode early forfeits the remaining
    # steps of (alive + safety + vel) reward for a small capture bonus, so the agent learns to
    # loiter just outside the capture radius instead of entering it (observed as ~10% capture).
    # Fix (Option A): replace the survival bonus with a small per-step time cost so that reaching
    # the goal quickly is strictly optimal, and raise the terminal capture bonus to cover the
    # forfeited future reward.
    #
    # (Removed dead ends, kept in git history + CRASH_TUNING_LOG.md: the B/C/D near-obstacle
    #  clearance penalty — three null results, crash is geometric not reward-driven — and the
    #  C1 finish-funnel, superseded by ego-progress shaping + capture_bonus + learned yaw.)
    reward_parameters = {
        # Phase 3: applied to the RELATIVE velocity toward the target (range-rate,
        # (v_drone - v_target) . dir). With a static target this is EXACTLY NavRL's
        # velocity-toward-goal term.
        "vel_weight": 1.0,
        "alive_weight": -0.05,  # time cost per step (was +1 survival bonus; see note above)
        # A(crash): raised 1.0 -> 1.5 so obstacle clearance is valued more relative to the
        # velocity-toward-goal reward (the drone was shaving bars while rushing to far goals).
        # The log-distance gradient is unchanged, only its weight.
        "safety_static_weight": 1.5,
        "smooth_weight": 0.1,
        "height_weight": 8.0,
        "height_margin": 0.2,  # NavRL's +/-0.2 m tolerance band
        # A: ego-motion progress reward weight -- dense per-step "got closer" gradient,
        # re-anchored to the target's CURRENT position so only the drone's own motion is credited
        # (reward = w*(||prev_pos - target|| - progress_gamma*||pos - target||)). Set 0.0 to disable.
        "progress_weight": 1.0,
        # B3: raised -10 -> -20 so a no-capture episode (~ -0.05*300 = -15 once B1 removes the
        # open-space safety income) stays strictly better than crashing -- the suicide guard that
        # the removed +1 alive bonus used to provide. NavRL leaves this commented out entirely.
        "collision_penalty": -20.0,
        # Terminal bonus when the capture radius is touched (episode ends as a success). Sized to
        # outweigh the future reward given up by ending the episode early.
        "capture_bonus": 30.0,
        # (b) Learned-yaw shaping. Dense, speed-gated crab PENALTY (<=0 so it cannot create standing
        # income / re-open the loiter optimum): punishes moving crab-wise so the drone leads with its
        # 0.28 m face, not its 0.40 m diagonal. Plus a tiny yaw-rate^2 damping. Set both 0.0 to disable.
        "yaw_align_weight": 0.3,
        "yaw_rate_smooth_weight": 0.02,
    }
