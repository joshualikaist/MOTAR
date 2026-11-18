# Review r1: OAK-D field recorder, capture converter and Korean field protocol (unit C, uncommitted)

- Reviewer: Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only on the worktree. Wrote only to this audit directory.
- Target: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `7efbbd9`. The change is uncommitted.
- Files reviewed, by SHA-256. These match the implementer's `logs/sha256_before_run2.txt`, so the implementer's full-suite run covered these exact bytes:
  - `tools/oakd_field_recorder.py` fe4d892c…
  - `tools/oakd_recording_convert.py` 3c9b3d6b…
  - `tests/test_oakd_field_recorder.py` de070770…
  - `tests/test_oakd_recording_convert.py` 85a9b644…
  - `ros2/motar_camera_policy/FIELD_RECORDING_KO.md` 7538bc0d…
  - `ros2/motar_camera_policy/README.md` e0fe3b82…
- Host: boot `407ae422…`, kernel MCE count 0, cpu4-5 offline. Every run was pinned to cores 0-3 and 6-23.

## Verdict: FAIL (repairable; no P0)

The code path works.
- Geometry, gravity convention, integrity, refusals and the end-to-end path to the replay all pass independent checks.
- On a synthetic ground-plane scene, the ground fit lands within 0.15 mm of the tape height when the depth model matches.

The verdict is FAIL because three findings must be fixed before the field session:
- P1-1: per-frame exposure, ISO and white-balance state is not recorded.
- P1-2: the protocol's disk budget does not hold on the Orin or on the PC.
- P1-3: the protocol installs depthai into the drone's user site-packages.

Counts: P0 0, P1 3, P2 6, P3 10.

| Criterion | Result | Evidence |
| --- | --- | --- |
| C1 Recorder safety and refusals | PASS (hardware NOT_TESTED) | Every listed refusal exists, is named and has a positive control (recorder `:149-224, 391-409, 433-442, 621-622`; tests `:330-401, 479-501`). The only imports are stdlib and numpy, plus a lazy `depthai` at `:753`. There is no ROS, PX4 or MAVLink code. The flood LED is set to 0 and the dot projector is set only when `--projector > 0` (`:657-660`). `/usr/bin/python3` 3.8.10 `py_compile` passes. All 11 recorder tests pass under Python 3.8.20 with numpy 1.23 (depthai venv). |
| C2 Data sufficiency and integrity | LIMITED_VALIDITY (P1-1 open) | The capture holds everything the converter needs. The manifest is correct (`sha256sum -c` rc 0). Interrupted, ENOSPC, truncated and missing captures are all refused. `getCvFrame` matches the converter's cv2 call bit for bit for both layouts at 3 sizes. That was measured on constructed frames; real device frames (stride) are NOT_TESTED. Exposure, ISO and AWB are not recorded (P1-1). |
| C3 Converter geometry | PASS (device depth model NOT_TESTED) | The crop K matches the contract convention exactly: 66.000000000 x 54.000000000 deg edge to edge, and block centres at `4i+1.5`. Map direction is correct for the 14-coefficient rational and Fisheye models (max 0.03 px). Unsupported models are refused. Depth uses nearest sampling only. Coverage equals an independent computation. Point projections land exactly for both depth models (63/63). |
| C4 Gravity | PASS | The round trip accelerometer → `up_optical` → replay roll and pitch, over 500 random R, roll and pitch, errs by at most 1.4e-14 deg. Positive pitch is nose down. The guards and the fallback behave as documented. See P2-2 and P2-3 for limits. |
| C5 End to end | PASS (synthetic) | Recorder (fake SDK) → converter (both models) → `load_recording` → `frame_inputs` → `replay_recording` with an outdoor contract (no checkpoint needed). With the matching model: 4/4 frames accepted, ground error vs tape 0.1 mm. With the mismatched model: 4/4 refused. |
| C6 Tests | PASS_WITH_LIMITS | 41/41 targeted tests OK. 27/30 mutations killed; the 3 survivors are listed under P2-5. Modules are loaded under private names, with no `os.environ` writes and no stubs. The full suite was not rerun because its bytes are identical to the implementer's 4281-test OK run. |
| C7 Korean protocol | FAIL | Commands, flags, file names and printed lines match the code, and the safety steps are present. The disk estimate fails: P1-2. The install step is a hazard: P1-3. |
| C8 Scope and wording | PASS | Only the 6 listed files changed. The README pinned-prefix test passes. There are no edits to results, preregistrations, checkpoints, navrl_task, WORKLOG or settings. There is no flight, GO or real-flight claim; the protocol ends with "not a flight-readiness test". |

## Findings

### P0
None found.

### P1 — must fix before the field session

**P1-1 Per-frame camera 3A state is not recorded.** At `tools/oakd_field_recorder.py:679-695`, `frame_arrays` stores no exposure time, ISO, colour temperature or lens position.
- DepthAI 2.29 exposes all four per frame: `ImgFrame.getExposureTime / getSensitivity / getColorTemperature / getLensPosition`. All four were verified present (`logs/p1_depthai_api.log`).
- The main threshold this session is meant to calibrate is the sky rule `luma_min = 0.5` (`camera_policy_contract.py:72`), and luma is a direct function of auto-exposure.
- Failure scenario: the TA records S01 in bright sun and in shade. The sky luma differs, and nothing in the capture tells you whether the cause is the scene or the AE operating point. The deployed pipeline (VIO-shared) may also run different AE limits. A luma threshold fitted to these frames cannot be transferred or bounded.
- The data cannot be recovered after the session. The fix is a few lines.
- The stereo `initialConfig` and the colour 3A modes should be recorded with it (see P3-2).

**P1-2 The protocol's disk budget is wrong for both machines** (`FIELD_RECORDING_KO.md:56-57, 126-127, 182, 189-193, 224`). Computed in `logs/p6_sizes.log`:
- The plan is 72 recordings (24 in table 1, 48 in table 2), or 1,440 frames.
- At the recorder's own synthetic estimate of 1.558 MB/frame, that is 2.24 GB. At the protocol's 2.5 MB switch point it is 3.60 GB. With incompressible data it is 5.17 GB.
- **Orin.** Step 10.3 makes the tar in `~` on the same disk, which doubles the footprint: 4.49 GB at the estimate and 7.2 GB at 2.5 MB/frame. The protocol requires only "3 GB 이상".
- The protocol's own switch rule is inconsistent. Any first frame between 2.08 MB and 2.5 MB keeps 20 frames for table 2 and exceeds 3 GB. The recorder refuses each recording with `INSUFFICIENT_DISK`, so no data is corrupted, but the session stops partway.
- The tar runs after step 10.1 restarts the VIO, so it can fill the drone computer's disk while the VIO is running.
- **PC.** About 4.7 GB was quoted; `df` now shows 4.4 GB. The tar (2.24 GB), its extracted copy (2.24 GB) and the converted recordings (1.00 GB at the synthetic estimate) total 5.49 GB, which does not fit. Without keeping the tar it is 3.25 GB, which fits only at the synthetic estimate. At 2.5 MB/frame it is 4.60 GB, borderline. Realistic outdoor texture and sensor noise compress worse than the synthetic frame.
- Fix: budget from the measured first frame, require free space ≥ 2 x the total or stream the tar to the USB stick, and give PC extraction instructions that do not keep a second copy.

**P1-3 The install step can change the drone's software environment** (`FIELD_RECORDING_KO.md:40-45`). The step is `python3 -m pip install --user depthai==2.29.0.0 numpy`.
- It installs into the user site that the drone's own Python programs import, and the recorder refuses any other depthai version.
- Failure scenario: the VIO is a Python depthai application (for example an SDK pinned to another depthai 2.x) running as the same user. The install silently replaces its depthai, and the VIO fails or changes behaviour when it is restarted at step 10.1. Nothing in the protocol checks that the restarted VIO works.
- Which depthai the VIO uses is NOT_VERIFIED. depthai 2.29.0.0 has no numpy pin (wheel METADATA), so numpy is upgraded only if it is absent.
- Fix: a dedicated venv (`python3 -m venv ~/oakd_rec_venv`), and a post-restart VIO check.

### P2 — should fix or document

**P2-1 Projector and flood setter results are ignored** (recorder `:657-660`). `setIrLaserDotProjectorIntensity` and `setIrFloodLightIntensity` return bool. `capture.json` records the requested intensity (`:642-643`) whether or not the call succeeded, so a failed set mislabels every frame's projector state.

**P2-2 There is no record-time IMU check, and the converter has no explicit override for the gravity source.**
- The recorder never warns when `accel_count == 0` (`:676-695`). The first-frame log prints only MB.
- The converter raises `IMU_NO_SAMPLES` whenever an extrinsic exists (`oakd_recording_convert.py:248-252, 387-388`). It offers no way to use the recorded nominal pitch instead.
- A placeholder or identity IMU extrinsic in the EEPROM fails the nominal check on every capture. The only bypass, `--no-imu-nominal-check`, then uses that wrong R.
- Failure scenario: the IMU delivers nothing (firmware or IMU node issue). The TA sees COMPLETE, and every capture is unconvertible on the PC until the code changes. The data itself is retained.

**P2-3 The nominal IMU check is optional.** `--pitch-deg` is optional (recorder `:173, 736`), and without it the converter writes `NOT_CHECKED` and trusts the unverified IMU-axis and extrinsic convention (`:392, 410-411`). The protocol always passes it, but the tool does not require it, and the docstring calls that convention unestablished.

**P2-4 The depth-coordinate model is still a hypothesis, and the protocol does not name a deciding scene.**
- For a wide rational lens, the two models' depth sources differ by a median 17 and a maximum 85 raw px over the crop (`logs/p2b_points.log`).
- The end-to-end probe shows a flat-ground capture decides it. The matching model gives 4/4 accepted at 0.1 mm. The wrong model gives 4/4 refused (`ground_not_planar` or `ground_not_lowest_surface`).
- Recommend writing into the README or protocol that S02 (and the S10 board edges) decide the model before any threshold analysis.

**P2-5 Three mutations survive the new tests** (`logs/mutations.log`):
- R10: the recorder depth-shape check is removed (`:427`). The converter's `FRAME_LAYOUT` still catches it later.
- C6: coverage ignores the depth source (convert `:197-198`). That only matters under shortfall with `undistorted_pinhole`.
- C10: the roll limit of the nominal check is dropped (convert `:393`).

**P2-6 `DEVICE_BUSY` is inferred from "any device listed"** (recorder `:399-407`). A device that is listed but cannot be opened for another reason (missing udev rule, boot failure) is reported as busy with the stop-the-VIO hint. Protocol table row `:206` then sends the TA to stop a VIO that is already stopped.

### P3 — nits and limits

1. **IMX378 path.** Depth is aligned at width 1014 (`:89, 263`), the same as the existing `oakd_camera.py:34`. From recollection of the Luxonis docs, RGB-aligned depth wants a width that is a multiple of 16. Separately, real ISP frames may carry a stride. Both are NOT_TESTED. Both fail closed (`UNEXPECTED_*_LAYOUT`) rather than corrupt.
2. **Stereo settings are not dumped.** The HIGH_DENSITY values (confidence 245, median 5x5, LR threshold 10, depth unit mm, read from the real SDK) and the colour 3A modes are only implied by the pinned SDK version and preset name. Recommend dumping `stereo.initialConfig.get()`.
3. **`sequence_num` is stored only inside each npz.** `capture.json` does not have it, and drops from the non-blocking `maxSize=4` queue are not counted. That is harmless for stills.
4. **A mid-run converter refusal leaves a partial output folder** (frames without `recording.json`). The rerun then hits `OUTPUT_EXISTS`. The docstring is honest about this.
5. **Fake-SDK captures are labelled as real.** They get `source: oakd_field_recorder` and become `REAL_OAKD_RECORDING` in the replay summary. Keep smoke captures clearly named, as the implementer did.
6. **Some errors print a traceback instead of a named code.** ENOSPC and other unexpected errors do this, although `capture.json` does record `INCOMPLETE/UNEXPECTED`. The KO error table omits `BAD_ARGUMENT`, `NO_CAM_A`, `UNSUPPORTED_STEREO_SENSOR` and `UNEXPECTED_ISP/DEPTH_LAYOUT`.
7. **Edge blending in RGB.** Bilinear RGB within coverage blends with the black border at the outermost source pixel: 573 px, in the shortfall case only. There are 0 such pixels in the full-coverage crop.
8. **One test name overclaims.** `test_colour_decoding_matches_depthai_get_cv_frame_formula` does not use depthai. The reviewer's real-SDK probe supplies that evidence.
9. **KO readability.** `scp`, `systemctl stop <서비스 이름>` (the TA may not know the service name) and `tar` are unexplained. The pitch sign from a phone level app is easy to invert. An inverted sign is caught later as `IMU_NOMINAL_PITCH_MISMATCH`, but that costs the capture.
10. **Nominal tolerance.** The nominal tolerance of 5 deg is about the same size as the bias the accelerometer norm gate admits (0.8 m/s² perpendicular ≈ 4.66 deg). Expect some false `IMU_NOMINAL_PITCH_MISMATCH` refusals.

## Commands run (all outputs under `logs/`)

```
# targeted tests (aerialgym env, worktree, PUBLIC_CPU env prefix, taskset 0-3,6-23)
python -B -m unittest test_oakd_field_recorder test_oakd_recording_convert test_oakd_camera test_camera_ros2_package -v
  -> Ran 41 tests in 3.513s  OK                                   logs/targeted_tests.log
/usr/bin/python3 -m py_compile scratch/py38/oakd_field_recorder.py (Python 3.8.10) -> PY_COMPILE_OK
/home/fair/venvs/oakd-depthai-2.29/bin/python -B -m unittest discover -s tests -p test_oakd_field_recorder.py (3.8.20)
  -> Ran 11 tests  OK                                             logs/recorder_tests_py38.log
probes/p1_depthai_api.py (real depthai 2.29.0.0, no device)       logs/p1_depthai_api.log
  -> getCvFrame == cv2 IYUV/NV12 decode of isp_layout(getFrame()) identical True for 1280x800, 1014x760, 640x400 (6/6)
  -> all Device/CalibrationHandler/MessageGroup/ImgFrame methods used are present; both profiles build and serialize
probes/p2_geometry.py, p2b_points.py                              logs/p2_geometry.log, logs/p2b_points.log
  -> crop K exact; maps <= 0.03 px; 63/63 point depths and RGB dots at predicted pixel for both models;
     nearest depth = independent nearest (except .5 ties); coverage = independent; unsupported models refused
     (p2 "FAIL point depths within 2 px" lines are a flaw in that probe's single-pixel design under 2:1
      nearest downsampling, superseded by p2b's 7x7 patches)
probes/p3_gravity.py                                              logs/p3_gravity.log
  -> roundtrip max err 1.42e-14 deg; R^T or sign flip -> >= 80 deg deviation (caught by nominal check)
probes/p4_end_to_end.py e2e/                                      logs/p4_end_to_end.log
  -> matched model accepted 4/4, ground err vs tape 0.1 mm; mismatched model refused 4/4
probes/p5_partial.py partial/                                     logs/p5_partial.log
  -> Ctrl+C / ENOSPC -> INCOMPLETE -> INCOMPLETE_CAPTURE; missing/truncated json -> CAPTURE_UNREADABLE;
     truncated frame -> FRAME_HASH_MISMATCH; missing frame -> CAPTURE_UNREADABLE; sha256sum -c rc 0
probes/mutate.py scratch/mut (scratch copy of tools/ + the 2 tests)  logs/mutations.log
  -> unmutated OK; 27/30 killed; survived R10, C6, C10
probes/p6_sizes.py                                                logs/p6_sizes.log
  -> 72 recordings, 1,440 frames; 2.24 / 3.60 / 5.17 GB raw; PC need 5.49 GB with tar kept (does not fit 4.7 GB)
```

Not run, with the reason:
- The full suite: bytes are identical to the implementer's 4281-test OK log.
- Hardware, Jetson, aarch64 wheel and real calibration: out of scope, NOT_TESTED.
