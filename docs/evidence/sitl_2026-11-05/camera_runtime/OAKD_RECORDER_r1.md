# Unit C r1 — OAK-D field recorder, converter, tests, Korean field protocol (Beads motar-783.21)

Date 2026-11-18. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, base HEAD
`7efbbd97b1626b473bedc0ede6afbf1e6fa5125f`. Git state unchanged by this unit (no add / commit / stash / checkout).
CPU only; no ROS, SITL, GPU, device or vehicle. Usage: USAGE_UNKNOWN (no provider-visible signal in this worker).
Not independently reviewed. No flight qualification, no outdoor GO, no real-flight claim.

## Files (new unless marked)

| File | Why | SHA-256 |
| --- | --- | --- |
| `tools/oakd_field_recorder.py` | single-file Jetson recorder (`--probe`, `--record`), stdlib + numpy + depthai only, Python 3.8 syntax | `fe4d892c843ae88ef7bf07829584f08e2d21608dfb3c5561ade0efb82f153d09` |
| `tools/oakd_recording_convert.py` | PC converter raw capture -> `motar-oakd-outdoor-recording-v1` | `3c9b3d6b883a5c8d382adfcbd0be9a9ac50f13f800a97b523182536b050572a7` |
| `tests/test_oakd_field_recorder.py` | 11 fake-SDK tests (selection, USB, timestamps, output dir, bytes, projector, busy/no device, probe, imports) | `de0707707a9679ae6e7672be1779b132cf795f4d50209af110d506a8882c9c68` |
| `tests/test_oakd_recording_convert.py` | 11 tests (replay load, crop K vs contract, point projection, depth no-bleed, depth model, presets/FOV, up vector, IMU refusals, time/hash/manifest, dry run/CLI, colour decode) | `85a9b644ac50d3d2a5ac241dbee23161f2d5f36359f3816ab8850a84c97ef71e` |
| `ros2/motar_camera_policy/FIELD_RECORDING_KO.md` | Korean step-by-step field protocol for the TA | `7538bc0d8d82956361b8cee9a5d36b8caa8f839c0bd473d9d26f3651aca56ea5` |
| `ros2/motar_camera_policy/README.md` (CHANGED, +12 lines appended at the end of "Recording the user's OAK-D outdoors...") | doc hook naming both tools and the protocol; earlier text untouched (pinned prefix tests pass) | `e0fe3b822bdf6d4dd8f9ff2aeb3814fae4c39e9b92b5b865826b29eea1953831` |

## Design decisions (and rejected alternatives)

1. **Model-agnostic profile from the detected CAM_A sensor.** `getCameraSensorNames()` decides: OV9782 (Pro W) ->
   ColorCamera THE_800_P, ISP 1280x800, no focus control; IMX378 (Pro) -> THE_12_MP + ISP 1/4 = 1014x760 with manual
   focus at the calibration lens position (the existing `oakd_camera.build_pipeline` approach). Rejected THE_1080_P
   for IMX378: a 16:9 crop of the 4:3 sensor that drops the VFOV below the trained 54 deg. Any other colour sensor,
   missing CAM_A, missing stereo pair or a stereo sensor outside {OV9282, OV9782} -> named refusal. Stereo is
   identical to the adapter (800P, HIGH_DENSITY, LR check, align CAM_A, calibrated translations, setOutputSize =
   colour size, extended or subpixel).
2. **Colour stored as the ISP frame (YUV420p/NV12), not RGB — deviation from the brief.** The ISP output is YUV;
   `getCvFrame` needs cv2, which the Jetson script must not import, and a numpy YUV->RGB reimplementation would not be
   bit-identical to DepthAI's. Verified offline with depthai 2.29.0.0: `ImgFrame.getFrame()` for YUV420p/NV12 is
   uint8 [H*3/2, W] and `getCvFrame()` == `cv2.cvtColor(that, COLOR_YUV2BGR_IYUV / _NV12)` exactly; the converter
   applies that call and reorders to RGB (test `test_colour_decoding_matches_depthai_get_cv_frame_formula`). Side
   benefit: half the bytes of RGB. A padded / strided frame is refused (`UNEXPECTED_ISP_LAYOUT`), not guessed.
3. **Bytes per frame (synthetic outdoor-like frame, recorder's exact encoding, np.savez_compressed):**

   | size | noise 0 DN | noise 2 DN | noise 4 DN |
   | --- | --- | --- | --- |
   | 1280x800 (OV9782) | 0.91 MB | 1.56 MB | 1.71 MB |
   | 1014x760 (IMX378) | 0.70 MB | 1.18 MB | 1.30 MB |

   (log `logs/smoke_real_depthai.log`; real-ImgFrame smoke first frame 1.54 MB, `logs/smoke_real_imgframe.log`.)
   Raw RGB uint8 + depth in npz measured 2.8 MB (noise 2) and 3.1 MB (noise 4) at 1280x800, above the 2.5 MB target;
   a row-delta filter brought RGB to 2.3 MB — both rejected in favour of YUV storage. The recorder prints the estimate
   and the first real frame size, and refuses `INSUFFICIENT_DISK` below 1.5x the estimate. Real outdoor frames are
   NOT_MEASURED.
4. **Aligned-depth coordinate model is NOT established -> converter flag is REQUIRED, no default.** Luxonis RGB-D page
   (https://docs.luxonis.com/software/perception/rgb-d): "By default, the depth image is undistorted during the
   rectification-undistortion process, but the color image is not. This can lead to misalignment" — i.e. the
   CAM_A-aligned depth is most likely in the UNDISTORTED CAM_A pinhole image, whereas the brief (and
   `oakd_camera.validate_config`'s `distorted_rgb`) assume it shares the distorted colour image's coordinates. Rectifying
   depth "with the same map" would mis-register it on a strong wide-FOV lens (synthetic Pro W fixture: up to >5 px
   difference at the crop corners, test `test_depth_model_choice_is_required_and_changes_the_depth_map`).
   `--depth-coordinate-model undistorted_pinhole` (pure pinhole map K_raw -> K_out) or `distorted_rgb` (same map as
   RGB); the choice and `HYPOTHESIS` status go into `recording.json.device.conversion`. The v2.29 RGB-D example uses
   `setMeshSource(CALIBRATION)` on the colour camera to undistort it; ColorCamera in 2.29 python exposes no
   `setMeshSource`/`setCalibrationAlpha`, so on-device undistortion was rejected (untestable, changes K unknowably).
5. **Pro W `trained_crop` preset.** K_out: fx = W/(2 tan 33 deg), fy = H/(2 tan 27 deg), cx = (W-1)/2, cy = (H-1)/2
   — verified against `camera_policy_contract.py` source (`width, height, hfov, vfov = 160, 90, 66.0, 54.0`,
   `cx = (width - 1) / 2.0`, `fx = width / (2 tan(hfov/2))`) and pinned by a test that greps those lines. Output
   640x360 = 4x the trained grid: trained pixel (i, j) is exactly the 4x4 block centred at (4i+1.5, 4j+1.5) (ray
   identity tested to 1e-12), edges subtend exactly 66.0 / 54.0 deg, pixels non-square like the trained camera.
   `oakd_camera.Rectifier.require_training_fov` accepts this K and refuses a 1 % narrower one. Sizes k x (160x90),
   k = 1..8 allowed (480x270 works). Rejected: square-pixel crops (cannot hit both 66 and 54 exactly), 160x90 directly
   (throws away resolution before the runtime's own projection). Every output pixel must be covered by the raw image
   or `FOV_SHORTFALL` (override `--allow-fov-shortfall`, recorded).
6. **IMX378 `adapter` preset** reuses `oakd_camera.Rectifier` maps exactly (array-equal test) with K_out = raw K at
   1014x760 and requires `require_training_fov` (else `FOV_SHORTFALL`).
7. **Depth resampling** is `cv2.INTER_NEAREST` with zero border (adapter convention); tests check no output value is
   outside the source value set, exact equality to round-to-nearest gathering on >99 % of pixels (all except
   exact .5 ties), every in-coverage zero comes from a source zero, with non-zero counts of zeros and valid pixels.
   Coverage = colour source AND depth source inside the raw image.
8. **IMU frame convention.** `CalibrationHandler.getImuToCameraExtrinsics(CAM_A)` docstring in depthai 2.29.0.0
   (read offline): "relative rotation and translation from the IMU to Camera" -> p_cam = R p_imu + T (T in cm, as
   `getCameraExtrinsics` documents). For acceleration only R applies (https://docs.luxonis.com/hardware/platform/sensors/imu/:
   "for vectors such as acceleration and angular velocity, the rotation part of the transform is used"). Camera frame
   = OpenCV optical / Luxonis RDF (x right, y down, z forward). Still accelerometer = +g along UP (Android/BNO08x
   convention, https://developer.android.com/reference/android/hardware/SensorEvent). up_optical = R a/|a|.
   Hazard: Luxonis forum threads report raw v2 IMU axes in vendor (possibly left-handed) frames and missing extrinsics
   on older boards (https://discuss.luxonis.com/d/5467-confusion-about-imu-coordinate-frame,
   https://discuss.luxonis.com/d/340-getimutocameraextrinsics-error). Guard: every frame's IMU roll/pitch is compared
   with nominal (roll 0, capture `--pitch-deg`); deviation > 5 deg -> `IMU_NOMINAL_PITCH_MISMATCH` (override
   `--no-imu-nominal-check`, recorded). Also `IMU_NOT_STILL` (|a| off g by > 0.8 m/s^2 or per-axis std > 0.5),
   `BAD_IMU_EXTRINSICS` (non-rotation). No extrinsic -> `roll_pitch_deg = (0, nominal)` and the device block says so;
   neither -> `NO_GRAVITY_SOURCE`. Tests: level -> (0,-1,0); 10 deg nose-down -> (0,-cos10,-sin10), pitch 10 via the
   replay's `roll_pitch_from_up`, with identity and a non-symmetric IMU rotation.
9. **VIO shares the OAK-D (user fact 2026-11-18).** `open_device` distinguishes `DEVICE_BUSY` (device connected per
   `getAllConnectedDevices()` or "in use/already/busy" in the error) from `NO_DEVICE`; both messages tell the operator
   to stop the VIO and restart it afterwards. The Korean protocol makes stopping/restarting the VIO numbered steps
   with checklist rows; `--probe` is optional. No ROS/VIO integration added.
10. **Fail-closed / safety.** Argument checks (frames 1..100, output must not exist, heights, pitch, projector 0..1)
    run before the device is opened. Non-increasing colour device timestamps stop with
    `NON_INCREASING_DEVICE_TIMESTAMP`; frames already saved are kept and capture.json is written with status
    INCOMPLETE + stop_error; the converter refuses INCOMPLETE captures. Dot projector set only when --projector > 0
    (and refused `NO_IR_PROJECTOR` when the device lists no IR driver); flood LED set to 0 whenever IR drivers exist.
    IMU: ACCELEROMETER_RAW 100 Hz; ROTATION_VECTOR added only for a BNO08x IMU. Accel samples per frame:
    (max(prev frame t, t-0.5 s), t].

## Verification

- Kernel fault count (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`): before 1
  (known QGroundControl user-space segfault 14:59:46), after both full runs 1 (no change).
- Focused: `test_oakd_field_recorder` 11 OK, `test_oakd_recording_convert` 11 OK, with `test_oakd_camera`: 33 OK.
- README pins: `test_camera_ros2_package` 8 OK; `test_camera_runtime_outdoor_r1.OutdoorRecords` + `test_camera_runtime_unit_b` 32 OK.
- Mutation checks (`logs/mutation_checks.log`): R^T instead of R, linear depth resampling, depth-model ignored, cx = W/2
  -> each makes its test fail (1 failing test each); unmutated 0.
- Offline smoke with the real depthai 2.29.0.0 module (`logs/smoke_real_depthai.log`): pipelines for OV9782 and
  IMX378 in both stereo modes built (8 nodes each); IMX214 refused UNSUPPORTED_COLOUR_SENSOR; `--probe` and `--record`
  exit 2 with `NO_DEVICE` (no output folder created).
- Real `dai.ImgFrame` smoke (`smoke_real_imgframe.py`, `logs/smoke_real_imgframe.log`): record() with the real SDK
  pipeline and real ImgFrame messages from a fake device -> COMPLETE, 3 frames, `sha256sum -c` OK, converter dry run
  PASS (trained_crop 640x360, coverage 1.0). The folder is `smoke/SMOKE_FAKE_DEVICE_NOT_DATA_capture`: written by the
  recorder, so its `source` reads `oakd_field_recorder` although the device is fake — never convert it to a
  non-dry-run folder or treat it as data.
- Python 3.8.10 `py_compile` of the recorder: OK (`logs/py38_compile.log`; pyc written outside the worktree; one
  stray `tools/__pycache__/*.cpython-38.pyc` from the first compile was deleted, gitignored anyway).
- `tools/check_public_docs.py`: PASS (exit 0, errors []).
- Full PUBLIC_CPU (`tools/run_tests.py`, cores 0-3,6-23, CUDA hidden): baseline 4259 OK / skipped 10 / expected
  failures 10 -> run 1 `Ran 4281 tests in 917.5 s, OK (skipped=10, expected failures=10)` (`logs/full_suite_r1.log`;
  the recorder's calibration-summary code was tidied while it ran) and run 2 on the final tree
  `Ran 4281 tests in 916.0 s, OK (skipped=10, expected failures=10)`, EXIT 0 (`logs/full_suite_r2.log`; file hashes
  re-checked unchanged after the run, `logs/sha256_before_run2.txt`). +22 tests = the two new files.

## NOT_TESTED

- Any real OAK-D (Pro W or Pro), the Jetson Orin NX, JetPack / Python 3.8 runtime, the aarch64 depthai 2.29.0.0 wheel.
- The real EEPROM calibration content (distortion model, 14 coefficients, IMU extrinsics present or not, their sign
  convention), `getConnectedCameraFeatures`/`getIrDrivers`/`getConnectedIMU` return shapes on hardware.
- Whether the device's CAM_A-aligned depth is in the distorted or undistorted colour image (converter flag is a
  hypothesis until a capture with a known target checks it).
- Real ISP frame layout at 1014x760 (stride/padding; refused if padded) and StereoDepth output width 1014 (not a
  multiple of 16; inherited from the adapter, unverified on hardware).
- Real bytes/frame, frame rate and save time on the Orin; BNO086 accelerometer rate and bias; ROTATION_VECTOR output.
- `DEVICE_BUSY` detection against a real VIO process (only the fake path is tested).
- Sync node behaviour on hardware (10 ms threshold), warm-up length for auto exposure.

## Open items

- Choose the depth coordinate model after the first real capture (board edge vs RGB edge check); consider a
  converter diagnostic for it.
- The TA must confirm the colour sensor via capture.json (the recorder logs it); a non-OV9782/IMX378 unit is refused.
- Independent review (Opus) of this unit not run.
- Records (WORKLOG, DECISIONS, Beads close, commit) are the caller's.
