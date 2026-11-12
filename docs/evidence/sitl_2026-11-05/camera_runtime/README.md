# Camera-only runtime preparation, 2026-11-11

Scope: source preparation and CPU verification for the user's real vehicle without
LiDAR. This is a new opt-in RGB-D path. The historical S4/F1/R1 experiments and
Frozen Policy F's full-sensor path retain their original contracts and evidence.
No Gazebo, GPU, hardware, arming, training, commit, push or merge was performed.

The complete external evidence directory is
`/home/fair/motar_tmp/camera_runtime_20261111/`. `VALIDATION.json` in this folder
records the final public CPU result and its source bindings. The independent
review is **PASS_WITH_LIMITS, CPU source preparation only**; required repairs
remaining zero. Its original negative findings and failed tests are preserved.

## Prepared path

- `tools/sitl_s4_gazebo.py`: explicit `sensor_mode='rgbd'` startup requires fresh
  RGB and depth after initial target application, with no LiDAR payload or stamp.
  The default remains `rgbd_lidar` for the historical reference path.
- `tools/sitl_s4_model.py`: the RGB-D model contains no LiDAR sensor, and the
  separate `tools/sitl_s4/motar_rgbd_bridge.cc` plugin accepts an omitted LiDAR
  element. The historical S4 source is byte-identical to HEAD and its old pin;
  full mode loads `libmotar_s4_bridge.so`, RGB-D loads `libmotar_rgbd_bridge.so`.
  Zero dimensions and the `-1`
  stamp describe an absent modality; no sensed LiDAR data is fabricated.
- `tools/camera_policy_contract.py`: selects either actual trained CAM-K2-v3 seed
  explicitly, fixes checkpoint/report/sidecar SHA256, recorded environment, and
  runtime sources. Neither seed is called a measured winner. Changing bound
  source invalidates a generated contract.
- `tools/camera_policy_runtime.py`: simulator-free CPU production perception and
  actual policy restore, 898-D observations, no_lidar ablation and its trained
  camera-derived governor. Internal 288 scan slots contain the trained no-return
  representation. They are schema storage, not a physical sensor requirement.
- Incoming calibrated, aligned optical-z depth is converted to ray range. A
  same-frame invalid mask uses the extracted `navrl_depth_validity.apply_mask`;
  real inputs never receive simulated holes/noise. Camera projection uses both
  trained FOV axes (66 by 54 degrees), measured mount/rotation, and estimator
  attitude; tilted measured points use a depth buffer and leave holes invalid.
- `tools/sitl_camera.py` and `tools/camera_estimator_link.py`: only attributed
  PX4 LOCAL_POSITION_NED, ATTITUDE and HEARTBEAT; atomic snapshots with all
  receipt times. No HIL/target truth is consumed. Position/attitude receipt age
  is at most 0.15 seconds, heartbeat at most 2 seconds, and all four capture
  stamps have at most 0.025 seconds skew. Boot/sensor clock translation must be
  explicitly measured. Failure invalidates policy history; reset restarts its
  history phase and is tested across seven-frame sequences.
- `tools/oakd_camera.py`: optional DepthAI 2.29.0.0 source, IMX378 full-sensor
  12MP/ISP 1:4 (1014 by 760), measured calibration lens position/K/D, and explicit
  calibrated-baseline conversion/alignment. Actual calibrated ray coverage is
  checked before streaming. It synchronizes, rectifies and records RGB-D only.
  Startup SDK calls are unbounded and must finish before active control.
- `tools/camera_companion.py`: explicitly composes measured capture and the
  attributed estimator with the runtime, keeping host monotonic time distinct
  from simulation time. The sender defaults to proposed commands only; it does
  not arm, change modes, or invent a clock/calibration.

## Verification and preserved repairs

The external `review_r1/` contains source snapshots, test logs, independent
positive/negative probes and SHA bindings. Compact review copies are kept here:
`REVIEW_r1.md` (initial AMEND_FIRST), `REVIEW_r2.md` (receipt/skew repairs),
`REVIEW_r3.md` (OAK baseline/FOV fixes), `REVIEW_r4.md` (composition/reset), and
`REVIEW_r5.md` (separate plugin/isolation repairs). `SOURCE_SHA256_v2.json` binds
the final reviewed source and tests; the earlier snapshot is retained.

Independent tests cover actual checkpoint inference for both seeds, three
composed proposed ticks per seed, seven-frame reset bitwise reproducibility,
stale/reused/future/incoherent inputs, mode loss after inference, no publication
after failures, quaternion frame conversion, real fragmented TCP RGB-D startup,
legacy full-sensor positives, explicit depth-mask parity, and fake OAK graph
and messages. Fake interfaces and synthetic images are not physical evidence.

Required findings repaired: stale position/attitude with a fresh heartbeat;
attitude omitted from the capture skew check; OAK spec instead of calibration
baseline; 1080p crop losing the trained vertical FOV; reset mutating inference
tensors outside inference mode; standalone history phase retained across reset.
The first full PUBLIC_CPU run had 4010 tests, 2 failures, 0 errors: the new explicit
importer was absent from the isolation whitelist, and editing the shared plugin
broke the historical source pin. A behavior-tested fourth entry was registered;
the original source was restored exactly and RGB-D moved to its own plugin.
That failed log/result are preserved at `public_cpu_v1.log` and
`PUBLIC_CPU_RESULT_v1.json`. The final rerun passed **4012 tests, 0 failures,
0 errors, 10 skipped, 5 GPU tests excluded**, with host regex fault count 0 to 0.
All reviewed source and test hashes match the full-suite baseline and completion.
See `VALIDATION.json` and external `PUBLIC_CPU_RESULT_v2.json`; elapsed wall time
is test telemetry, never an account-quota estimate.
No existing checkpoint, raw result, preregistration or failure record was edited.

`artifacts_v2/` outside Git contains two SHA-bound policy contracts, a synthetic
RGB-D/estimator NPZ, seven-frame CLI replay outputs per seed (zero sends), generated
RGB-D SDF/airframe, and its own validation manifest. The new plugin compiled in
`plugin_build_v2/` and was never loaded. The original pinned plugin is unchanged.
The earlier `artifacts_v1/` and `plugin_build/` remain as historical development
outputs; their old-source contracts must not be used for the repaired tree.

Metadata-only checks on this workstation:

```bash
PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' /home/fair/miniconda3/envs/aerialgym/bin/python -B \
  tools/camera_companion.py --check /home/fair/motar_tmp/camera_runtime_20261111/artifacts_v2/camk2v3_seed1.json
```

The runtime CLI can replay the saved synthetic NPZ to a **new** output path;
this restores the real policy but never opens a camera, simulator or flight link.

## What remains before a real experiment

This record does not qualify physical deployment or claim target capture,
10Hz latency, sensor equivalence, flight, or safe operation. The inference target
detector is the trained bootstrap red appearance rule, not an arbitrary-object
detector. Ground returns, real invalid-depth patterns and body-camera projection
still require measured qualification. S3a-R exact-input equivalence stays HOLD.

DepthAI is not installed in the tested environment. No actual device ID, measured
mount/rotation, aligned-depth coordinate evidence, or PX4-to-camera clock binding
was supplied. These are required rather than replaced with synthetic values.
The companion computer, flight-link transport and final deployment environment
are also unconfirmed. Contracts currently name the original local checkpoint
copies; moving to another device needs an explicit hash-preserving asset binding.
Two bounded SDK installation attempts in an isolated environment timed out while
downloading the wheel, including a retry against official PyPI. They are recorded
in `SDK_INSTALL_ATTEMPTS.json`; no real SDK API or device test is claimed.

The next execution unit must first bind a committed source/execution amendment,
new smoke/output paths, actual model/plugin/runtime configuration, explicit clock
measurement, per-frame recording, a stop response and fresh positive smoke. It
must use this RGB-D entry and CAM checkpoint, not the historical Frozen F launcher.
Simulator/GPU/hardware execution and the previously forbidden commit require
scoped authorization; no run follows automatically from this preparation.

The separate user correction changes session policy to observed usage. Root and
worktree AGENTS, handoff skill and helper now preserve attributed usage across
reconnects, mark unavailable usage `USAGE_UNKNOWN`, and never infer quota from
elapsed time. The 20 percent closeout and 1/30 stop reserves are policy thresholds,
not observed remaining quota. Historical wall-clock handoffs remain immutable.
Known main-work estimates must finish before the 20 percent reserve; main work
plus verification/records must also preserve the final 1/30 reserve.

Git Traceability: branch `camonly/dev`, base/HEAD
`eefb832e1c4fc6de8e597e990aa9cd58f2228079`; new commit NONE, push NO, merge NO.
Previous uncommitted readiness changes were preserved. This work is intentionally
uncommitted under the standing user instruction.
