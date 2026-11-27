# Unit C repair r1: response to review_r1/REVIEW_r1.md (FAIL; P0 0, P1 3, P2 6, P3 10)

Date 2026-11-18. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD
`7efbbd9`. The change is still uncommitted and git state was not touched. CPU only: no device, ROS, SITL or GPU, and
no package installs. Nothing was written into `review_r1/`. Usage: USAGE_UNKNOWN.

R = `tools/oakd_field_recorder.py`, C = `tools/oakd_recording_convert.py`, TR / TC = their test files,
KO = `ros2/motar_camera_policy/FIELD_RECORDING_KO.md`.

## Findings → changes → tests

| Finding | Change (file:line) | Test |
| --- | --- | --- |
| **P1-1** per-frame 3A not recorded | R:258-259 AE on and AWB AUTO set explicitly via `initialControl`. R:521 `colour_3a()` reads `getExposureTime / getSensitivity / getColorTemperature / getLensPosition / getLensPositionRaw` (-1 = not read). These are saved into each npz as `exposure_time_us, sensitivity_iso, colour_temperature_k, lens_position, lens_position_raw, auto_exposure_configured, auto_white_balance_configured`, and into each `capture.json` frame entry. R:567 and R:840 add a min/max `colour_3a_summary`. The first-frame log line prints exposure, ISO and colour temperature. C:386 and C:472 copy the 3A keys into each output npz (extra keys; the replay reads only its own). C:522 adds `recording.json.device.colour_3a` (summary + per frame). | TR:570 (fake SDK: per-frame values, summary, explicit AE/AWB calls, missing getters → -1 / NOT_RECORDED). TC:411 (copied through; `load_recording` + `frame_inputs` still accept the frames). Real offline `dai.ImgFrame`: `logs/repair_r1_smoke_real_imgframe.log` shows the getters callable and their values stored (0 offline; a device sets them). |
| **P1-2** disk budget | KO rewritten around a minimum set: 9.1 (KO:173) is 11 scenes × 2 projector states = 22 recordings, in priority order, about 1 h. 9.2 (KO:192) is an optional set of 48 recordings. The KO:210 table derives the frames per recording from the printed first-frame size X: X ≤ 1.8 MB → 20 frames; 1.8 < X ≤ 3.6 MB → 10 frames; X > 3.6 MB → stop. This bounds the minimum set at ≤ 0.8 GB and the optional set at ≤ 1.8 GB. Orin free space must be ≥ 1.5 GB for the minimum set, ≥ 3.5 GB for both. No tar on the Orin (KO:228): the folder is copied directly to a USB drive with `cp -r` + `sync`, after the VIO restart and check. PC needs ≈ 1.2 GB for the minimum set and ≈ 3.9 GB for both, against ~4.4 GB free, so the full set needs an external drive or a staged transfer (KO:222-226). C:413 and C:495 estimate the output size from the first converted frame and print it (`output_mb_estimate`, also with `--dry-run`, C:610). A real run refuses `INSUFFICIENT_DISK` below 1.2 × estimate + 10 MB, before anything is written. | TC:493 (dry-run estimate > 0; `INSUFFICIENT_DISK` with nothing written). Numbers in "Disk budget" below. |
| **P1-3** install into the drone's user site | KO:35-95: 2.1 records the current depthai / numpy / pip state without changing it (an error is fine). 2.2 `PIP_USER=0 python3 -m pip install --target ~/oakd_rec_pkgs depthai==2.29.0.0 numpy`. **Why `--target`:** it needs only pip; a venv needs `python3-venv` from apt, which is often missing on Jetson images. 2.3 `alias oakrec='PYTHONNOUSERSITE=1 PYTHONPATH=$HOME/oakd_rec_pkgs python3 $HOME/oakd_field_recorder.py'`; every recorder command in KO uses `oakrec`. 2.4 checks the isolated depthai is 2.29.0.0 from `oakd_rec_pkgs` and the system `import depthai` is unchanged. If pip is missing: stop, no apt. 11.2-11.3 restart the VIO and confirm it works as usual. 11.7 removes the install with `rm -rf ~/oakd_rec_pkgs`. Checklist rows 2, 3, 7, 14, 15, 18. R records `host.depthai_file` in `capture.json`. | Doc review; `depthai_file` in `logs/repair_r1_smoke_real_imgframe.log`. NOT_TESTED on a Jetson. |
| P2-1 setter results ignored | R:773-779 records `ir_flood_off_applied` and `ir_dot_projector_applied` from the bool returns. A false flood return is logged and recorded. A false dot return stops with `PROJECTOR_NOT_APPLIED` (`capture.json` INCOMPLETE, no frames). Projector 0 → `NOT_SET (boot default off)`. | TR:601 (dot false → refusal; projector 0 control records; flood false recorded) |
| P2-2 IMU checks | R:418 adds the IMU node only when `getConnectedIMU()` names one. R:812 refuses `IMU_NO_SAMPLES` when the first frame has none; later empty frames are counted (`frames_without_imu_samples`). C:381 adds `--gravity-source auto|imu|nominal`: `imu` without an extrinsic → `NO_IMU_EXTRINSICS` (C:447); `nominal` ignores the IMU, so a dead IMU or placeholder extrinsic leaves the capture convertible. The choice is recorded in `device.conversion.gravity`. | TR:618 (silent IMU → refusal; IMU NONE → no IMU node, COMPLETE; control). TC:429 (nominal vs imu, NO_IMU_EXTRINSICS, dead IMU → IMU_NO_SAMPLES under auto but convertible with nominal) |
| P2-3 `--pitch-deg` optional | R:182: required for `--record` (`BAD_ARGUMENT`; main lists it as missing). C:450 refuses `NO_NOMINAL_PITCH` when the IMU is used and the capture has no nominal, unless `--no-imu-nominal-check`. | TR:645, TC:429 |
| P2-4 no deciding scene named | KO:173-190: M01/M02 (flat ground, h 1.0 m, pitch 0 and +10) come first and are named as the depth-coordinate-model decision. The README hook names them too, citing the review's synthetic result. | README pin tests (below) |
| P2-5 surviving mutants R10, C6, C10 | New tests: TR:636 (device returns depth at the wrong size → `UNEXPECTED_DEPTH_LAYOUT`); TC:475 (barrel lens: colour sources all inside, pinhole depth sources outside for the top/bottom rows → coverage False there); TC:465 (roll 10° vs nominal 0 → mismatch; roll 3° control) | `logs/repair_r1_mutations.log`: 13/13 killed, including R10, C6 and C10 |
| P2-6 DEVICE_BUSY over-inferred | R:433 `classify_open_failure` uses the error text and each listed `DeviceInfo`'s XLink `state` / `status`. `X_LINK_INSUFFICIENT_PERMISSIONS` or "permission" → `DEVICE_PERMISSION` (udev). `X_LINK_DEVICE_ALREADY_IN_USE` / `ALREADY_OPEN`, a device already BOOTED, or "in use / already / busy" → `DEVICE_BUSY`. Listed otherwise → `DEVICE_OPEN_FAILED`. Nothing listed → `NO_DEVICE`. Every listed-device message names both the VIO and the udev rule / replug. The KO error table (KO:253) has one row per code. | TR:503 (6 cases); real `dai.DeviceInfo` with settable state/status classified BOOTED → BUSY and INSUFFICIENT_PERMISSIONS → PERMISSION (`logs/repair_r1_smoke_real_imgframe.log`) |
| P3-2 stereo config not dumped | R:304 records `initialConfig` confidence 245, median `KERNEL_5x5`, LR threshold 10, bilateral 0, depth unit MILLIMETER and max disparity (190 extended / 760 subpixel, real SDK) in `pipeline.stereo.initial_config`. Colour 3A modes are in `pipeline.colour`. | `logs/repair_r1_smoke_real_depthai.log` |
| P3-3 sequence numbers | `sequence_num` added to each `capture.json` frame entry; `sequence_gaps` counted (R:818). | TR:570 |
| P3-4 partial output blocks rerun | C:496-550 writes `<output>.partial`, renames it to `<output>` on success (C:546) and removes this call's own `.partial` on refusal (C:550). An existing `.partial` → `OUTPUT_EXISTS`. | TC:493 (refusal after frame 1 leaves nothing; rerun succeeds; existing `.partial` refused) |
| P3-6 traceback instead of a code | R:851 records `stop_error.code = OS_ERROR` for OSError; R:915 `main` prints `refused: OS_ERROR` with exit 2. KO error table adds BAD_ARGUMENT, NO_CAM_A, UNSUPPORTED_STEREO_SENSOR, UNEXPECTED_*_LAYOUT, OS_ERROR, DEVICE_PERMISSION, DEVICE_OPEN_FAILED, PROJECTOR_NOT_APPLIED and IMU_NO_SAMPLES. | TR:645 (ENOSPC on the first frame write → OS_ERROR, `capture.json` INCOMPLETE/OS_ERROR) |
| P3-9 KO readability | tar and scp removed. alias, PYTHONPATH, `cp -r`, `sync` and `sha256sum -c` each defined in one line. If the systemctl service name is unknown, ask the student. KO:121 adds a pitch-sign self-check with the phone app. | doc |
| Orchestrator: site name | Every "놀이터" replaced (0 left). The site is "기계동 앞 운동장 옆 숲"; open-sky scenes are at the 운동장; a plain light wall such as the outer wall of 기계동 may replace boards, with the distance taped (KO:163-169). | `grep -c 놀이터` = 0 |

## P3 left open, with reasons

1. **P3-1** IMX378 aligned-depth width 1014 (not a multiple of 16) and real ISP stride. Inherited from `oakd_camera.py`; needs hardware; both fail closed (`UNEXPECTED_*_LAYOUT`).
2. **P3-5** fake-SDK captures are labelled `oakd_field_recorder`. The smoke folders are named `SMOKE_FAKE_DEVICE_NOT_DATA_*`; no code change.
3. **P3-7** bilinear RGB blends with the black border at the outermost source pixel. This happens only under `--allow-fov-shortfall`; the full-coverage crop has 0 such pixels.
4. **P3-8** the test named `test_colour_decoding_matches_depthai_get_cv_frame_formula` does not itself use depthai. It is not renamed; the real-SDK equality evidence is in the reviewer's `p1_depthai_api.log` and my r0 offline check.
5. **P3-10** the nominal tolerance (5°) is about the size of the bias admitted by the accelerometer-norm gate (≈ 4.7°), so some false `IMU_NOMINAL_PITCH_MISMATCH` refusals are possible. The mitigations are `--max-imu-nominal-deviation-deg` and the new `--gravity-source nominal`. The thresholds are unchanged until real data exists.

## Disk budget: measured numbers

Synthetic outdoor-like frames, encoded exactly as the recorder and converter do (`repair_r1/measure_sizes_r1.py`, `logs/repair_r1_sizes.log`). Real frames are NOT_MEASURED.

| sensor noise | raw capture MB/frame (1280x800) | converted MB/frame (640x360) |
| --- | --- | --- |
| 0 DN | 0.915 | 0.318 |
| 2 DN | 1.559 | 0.695 |
| 4 DN | 1.713 | 0.756 |

The real-ImgFrame smoke frame (uniform random YUV, incompressible) measured 1.543 MB raw and 0.69 MB converted.

| plan | frames | raw at 1.559 MB | raw at the rule bound (X ≤ 1.8 MB × 20, or ≤ 3.6 MB × 10) | PC copy + converted |
| --- | --- | --- | --- | --- |
| minimum set, 22 recordings × 20 | 440 | 0.69 GB | 0.79 GB | 0.99 GB measured; ≈ 1.2 GB at the bound |
| optional set, 48 × 20 | 960 | 1.50 GB | 1.73 GB | 2.16 GB measured |
| both | 1400 | 2.18 GB | 2.52 GB | 3.16 GB measured; ≈ 3.9 GB at the bound |

Free space today: PC `/` 4.4 GB (`df -h`). The KO requirements follow from these numbers:
- Orin ≥ 1.5 GB for the minimum set: 0.79 GB, plus the recorder's own 1.5 × per-recording check, plus margin.
- Orin ≥ 3.5 GB for both sets: 2.52 GB plus margin.
- USB drive ≥ 1 GB (minimum set) or ≥ 3 GB (both).
- PC: the converted output is ≤ 0.5 × raw (measured 0.44-0.45).

## Verification

- Kernel fault count: 1 before the repair (the known QGroundControl user-space segfault), 1 after the full suite.
- Focused tests: TR 16 OK, TC 16 OK; with `test_oakd_camera`, 43 OK. Recorder tests under the depthai venv (Python 3.8.20, numpy 1.23): 16 OK.
- README pins: `test_camera_ros2_package` + `test_camera_runtime_outdoor_r1.OutdoorRecords` + `test_camera_runtime_unit_b`: 40 OK.
- Mutation checks: 13/13 killed; unmutated 32 OK (`logs/repair_r1_mutations.log`, script `repair_r1/mutate_r1.py`, run on a scratch copy that was then deleted).
- `/usr/bin/python3` 3.8.10 `py_compile` of the recorder: OK. The pyc was written outside the worktree; no stray pyc is left.
- Offline real depthai 2.29.0.0 (`logs/repair_r1_smoke_real_depthai.log`):
  - Both sensors × both stereo modes × with/without IMU build: 8 / 6 nodes.
  - IMX214 is refused.
  - `--probe` / `--record` exit 2 with `NO_DEVICE` and create no folder.
  - `--record` without `--pitch-deg` exits 2 with `BAD_ARGUMENT`.
- Real-ImgFrame smoke (`logs/repair_r1_smoke_real_imgframe.log`): COMPLETE, 3 frames, `sha256sum -c` OK, converter dry run PASS. The folder `repair_r1/SMOKE_FAKE_DEVICE_NOT_DATA_capture_r1` came from a fake device and is not data.
- `tools/check_public_docs.py`: exit 0.
- Full PUBLIC_CPU suite on the final tree (same command; baseline 4281): `Ran 4291 tests in 910.6 s, OK (skipped=10,
  expected failures=10)`, EXIT 0 (`logs/repair_r1_full_suite.log`). +10 = 5 new recorder tests + 5 new converter tests.
  File hashes were re-checked as unchanged after the run (`logs/repair_r1_sha256_before_full_suite.txt`).

## Changed files (SHA-256 after repair r1)

| File | SHA-256 |
| --- | --- |
| tools/oakd_field_recorder.py | 286a5196afb5fe3eea60d5f1d11adf1c70ed760c56f650b2e91f81b21fd0c6e4 |
| tools/oakd_recording_convert.py | b9147e32c5ae32d474090e5d2bcea728e0afe5d93e2b341df5849a21e9bd198f |
| tests/test_oakd_field_recorder.py | 3e6eaa0037b7a312355fa8269428d8d1a5a62a0edfe01f7781e8d360778c66a4 |
| tests/test_oakd_recording_convert.py | 39d23e0e499df5e82968bc79a471ff952a6ef58d6aa5283589ec3355aa54e3e1 |
| ros2/motar_camera_policy/FIELD_RECORDING_KO.md | 80b3dc8e0d6c0e48a0ac32d559334d200d68a026842acc0edb06ee76bef5f5c3 |
| ros2/motar_camera_policy/README.md | a38e8bb1db146b9a0dabe57caf5e0f4e5e2d049ee4d62cea0c25c219b7e2bf66 |

## NOT_TESTED / open

- Real OAK-D, Jetson, aarch64 wheel and `pip --target` on JetPack; real 3A values; real IMU axes and extrinsics; the depth coordinate model (decided by M01/M02); real bytes per frame; `DEVICE_BUSY` / `DEVICE_PERMISSION` against a real VIO or a missing udev rule.
- The VIO's own depthai version and environment are NOT_VERIFIED. KO 2.4 and 11.3 check them in the field.
- P3-1, P3-5, P3-7, P3-8 and P3-10 above.
- A second independent review was not run (the caller's decision).
