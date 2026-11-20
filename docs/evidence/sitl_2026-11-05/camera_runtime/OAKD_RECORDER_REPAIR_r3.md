# Unit C preflight r4 and repair r3 (2026-11-21): checks against the real DepthAI before the first device run

Worker: the orchestrating Claude Code session (Claude Fable 5.2), directly. Trigger: the user's request of 2026-11-21 to
judge once more, before the TA runs the recorder, whether anything is wrong, small errors included. CPU only, no
device, no GPU. Base `a76c5cc`. Independent review r4, r4b and r4c (Claude Opus 5.5, fresh context, read-only):
**PASS_WITH_LIMITS**, no P0 / P1 / P2 open after r4c; its findings and what was done with them are in the last table.

## Method

Reviews r1-r3 judged the code against a fake SDK. This round used four sources they did not have:

1. **The real SDK, no camera** (`preflight_r4/real_sdk_check.py`, DepthAI 2.29.0.0, Python 3.8.20): every `Device`
   and `CalibrationHandler` method the recorder calls, the enum names it compares, the real pipeline build for every
   colour sensor, stereo mode and IMU option, real `ImgFrame` / `MessageGroup` / `IMUData` objects passed through the
   recorder's own functions, the calibration getters with the recorder's exact argument lists, and the no-device
   refusal.
2. **Luxonis primary sources** (`preflight_r4/forum_*.json`, `depth_video_synced.py` at tag v2.29.0.0, the SDK
   docstrings, PyPI metadata).
3. **A stand-in device with real SDK objects** (`preflight_r4/real_sdk_hybrid.py`): `record()` end to end with the
   real `depthai` module, real message and calibration classes and a real `Pipeline`; only `dai.Device` is replaced.
   Its captures are named `SMOKE_FAKE_DEVICE_NOT_DATA_*` and are never data.
4. **A literal run of the protocol** (`preflight_r4/protocol_blocks.sh`, `protocol_literal_run.log`): every `bash`
   block of `FIELD_RECORDING_KO.md`, extracted by script, run with Ubuntu 20.04's own `/usr/bin/python3`
   (3.8.10, pip 20.0.2: the versions JetPack 5 ships) in an empty HOME.

The host library accepted every call before the repair (0 failed checks). What was wrong were rules that only the
device firmware enforces, which the host library builds without complaint.

## Findings and changes

| # | Severity | Finding | Evidence | Change | Test |
| --- | --- | --- | --- | --- | --- |
| F1 | P1 | The colour-depth `Sync` node ran at 10 ms with the default attempts −1. The node then sends nothing until a pair is that close: a constant offset above 10 ms between the colour and the stereo timestamps gives `FRAME_TIMEOUT` on every recording. | SDK docstring of `Sync.setSyncAttempts` ("if syncAttempts = -1 (default) then the node will only send a message if successfully synchronized"); DepthAI's own `examples/Sync/depth_video_synced.py` uses 50 ms; Luxonis staff, forum 5060: "11ms is way too low", 80 ms solved it. The offset of this camera is NOT_MEASURED. | `SYNC_THRESHOLD_MS = 60` (above half the 10 FPS frame period, so a partner always exists; 50 in the first repair, raised after review r4 F18). The measured depth-minus-colour time is in each frame entry, in the first-frame line and in the final status line. | `test_sync_threshold_is_above_half_the_frame_period`; the real pipeline reads back 60.0 ms |
| F2 | P1 (IMX378 only) | The IMX378 profile asked for a depth aligned to a 1014-pixel-wide colour image. The device refuses an aligned depth whose width is not a multiple of 16. | Forum 5933, device log: "[StereoDepth(3)] [error] Disparity/depth width must be multiple of 16, but RGB camera width is 1352. Set output size explicitly using 'setOutputSize(width, height)'." 1014 = 16 × 63.375. | ISP scale 4/13 → 1248 × 936 (16 × 78, full 4:3 sensor; the SDK's `getIspSize` returns exactly this). A build-time guard refuses any profile whose width is not a multiple of 16. The converter's `auto` preset uses the trained crop for an IMX378 capture that is not 1014 × 760. | `test_profile_sizes_fit_the_device_depth_width_rule`, `test_auto_preset_follows_the_capture_size_and_sensor_name` |
| F3 | P2 | The warm-up (2 s) was counted from the pipeline start, which itself takes seconds, so the first saved frames could be taken while the auto exposure of the colour and mono cameras was still moving. | Code reading. The convergence time on the device is NOT_MEASURED. | The warm-up counts from the first received frame, default 3 s; the number of discarded frames and the exposure / ISO at its start and end are recorded and printed. | `test_warmup_counts_from_the_first_received_frame` (a start-up of 3 s does not shorten the warm-up) |
| F4 | P2 | An SDK error during a recording (for example the USB link dropping, `X_LINK_ERROR`) ended in a Python traceback: no code, no instruction for the operator. | Code reading: `main` caught `RecorderError` and `OSError` only. | `UNEXPECTED` (exit 2) with the instruction to replug and record into a new folder, and `INTERRUPTED` (Ctrl+C, exit 130). `capture.json` is written INCOMPLETE as before. | `test_unexpected_sdk_error_is_named_and_keeps_the_capture` |
| F5 | P2 | `numpy` was installed unpinned. On Python 3.9 or newer pip would fetch numpy 2, which was never run with this SDK build. | PyPI: numpy 1.24.4 has aarch64 wheels for Python 3.8-3.11; latest numpy needs Python ≥ 3.12. | `numpy==1.24.4` in the protocol and in the recorder's install hint; the protocol stops if Python is not 3.8-3.11. | `test_install_hint_never_points_at_the_drones_python` |
| F6 | P2 | A colour socket reported as `OV9282` was refused. The firmware reports some OV9782 modules under that name. | Forum 6070 (`getCameraSensorNames` output with three of four OV9782 modules listed as OV9282, grey images). | `OV9282` on CAM_A records with the 1280 × 800 pipeline and `WARNING COLOUR_SENSOR_NAME`; sensor names are stripped and upper-cased. | `test_colour_socket_reported_as_ov9282_records_with_a_warning` |
| F7 | P2 | The protocol went straight to the outdoor hour with software that has never met the device. | — | Required desk test (step 4): `--probe` and two 3-frame recordings indoors, sent to the student, who answers before the outdoor session. | literal run of the three commands |
| F8 | P3 | Protocol details: no word on power for an hour of recording; the VIO process check matched three names only; `<USB이름>` in a command is a shell redirection if pasted; no path for an SSH-only Orin; `DEVICE_PERMISSION` had no fix; error table lacked the new codes. | Reading as the operator; literal run. | Battery row, wider process pattern and Docker note, `USB=` variable, `scp` note, the Luxonis udev rule, new table rows. | — (text) |
| F9 | OPEN | `tools/oakd_camera.build_pipeline`, the flight adapter, has F1 and F2 (1014 × 760 aligned depth, 10 ms). | `tools/oakd_camera.py` lines 34, 165, 179, 183-184. | Not changed in this unit: its 1014 × 760 size is bound to the adapter's rectifier and FOV check. Filed in Beads. | — |

## Checked and found correct (no change)

| Check | Result |
| --- | --- |
| SDK surface (15 `Device` methods, 9 `CalibrationHandler` methods, enum and XLink names, `DeviceInfo` fields) | all present in 2.29.0.0 |
| Real pipeline build, 3 colour sensors × 2 stereo modes × IMU options | builds and serialises; HIGH_DENSITY reads back confidence 245, median 5×5, max disparity 190 (extended) |
| Real `ImgFrame` YUV420p / NV12 / RAW16 through `isp_layout` and `depth_layout` | shapes and dtypes as the recorder expects |
| Calibration getters with the recorder's argument lists on a real `CalibrationHandler` | no signature error; an empty calibration is recorded as an error string, not a crash |
| Stand-in device run, protocol's own packages (Python 3.8.10, numpy 1.24.4) | 3 / 3 sensor cases COMPLETE; converter dry run accepts all three under both depth models |
| aarch64 wheels | `depthai-2.29.0.0` and `numpy-1.24.4` for cp38 and cp310, `manylinux2014_aarch64`; the DepthAI wheel bundles libusb |
| `pip 20.0.2 --target` with `PIP_USER=0` | installs both wheels; nothing under `~/.local` |
| Protocol commands, literally | alias, quoting and flags parse; without a camera every recording command ends in `NO_DEVICE`; copy and `sha256sum -c` blocks work |

An offline install kit (the four aarch64 wheels, SHA-256 equal to PyPI, 53 MB, with a Korean note) is in
`/home/fair/motar_tmp/oakd_recorder_20261118/offline_kit/`, for an Orin without internet. The no-index form of the
install command and the `python3 -m zipfile -e` fallback were run with pip 20.0.2 on x86_64 stand-in wheels.

## Numbers

| Quantity | Value |
| --- | --- |
| Recorder tests / converter tests | 29 / 18 OK (were 19 / 16) |
| Mutants of the new behaviour caught (author) | 42 / 44 (`preflight_r4/mutation_results_final.json`); survivors: M31 (the explicit warm-up drain is redundant with the drain on every empty poll) and M44 (`isp_padded_frames` in the final status line is not asserted) |
| Fast refusal right after the SDK import, protocol's own install (Python 3.8.10, pip 20.0.2) | committed `a76c5cc`: aborted with a C++ trace and exit 134 in 3 of 5 runs (`preflight_r4/fast_refusal_abort_a76c5cc.log`); repaired: 7 of 7 one line and exit 2 |
| PUBLIC_CPU, final tree | see the WORKLOG entry (`preflight_r4/public_cpu3.log`) |
| Synthetic bytes per frame, IMX378 at 1248 × 936 | 1.77 MB (OV9782 at 1280 × 800: 1.56 MB) |
| IMX378 intrinsics at 1248 × 936, SDK scaling vs exact 4/13 | 0.07 % in focal length (ISP rounds 935.4 → 936 rows) |

## Still NOT_TESTED (needs the device)

A real OAK-D and the Orin; whether the Sync node delivers at 50 ms and what the colour-depth offset is; the auto
exposure settling time; the IMX378 path at 1248 × 936 on the device; the sensor names this unit reports; IMU model,
rate and axes; projector power on the Orin's USB port; real bytes per frame; `DEVICE_BUSY` against the running VIO;
the depth-coordinate model. The desk test (protocol step 4) is the first measurement of all of these.

## Independent review r4 / r4b / r4c (Claude Opus 5.5) and what was done

Reports: `review_r4/REVIEW_r4.md`, `REVIEW_r4b.md`, `REVIEW_r4c.md` (committed copies `OAKD_RECORDER_REVIEW_r4.md`,
`_r4b.md`, `_r4c.md`). Verdict of each round: PASS_WITH_LIMITS.

| Finding | Severity | What it was | Done |
| --- | --- | --- | --- |
| r4 F1 | P2 | A refusal printed within milliseconds of `import depthai` (existing folder, bad `--frames`, missing argument or parent folder) was followed by a C++ abort trace, "core dumped" and exit 134: the SDK's resource-loading thread was still starting. Present since the first version of the recorder. | Arguments are checked before the import; the script leaves through `os._exit` after flushing. Reproduced before and after. |
| r4 F2 | P2 (inference) | The host IMU queue held 50 messages and was drained once per frame; on a slow host a frame's accelerometer window could be empty. | Queue 2000, drained on every empty poll, and every sample of the capture is kept in `imu.npz` (hashed, in the manifest). The converter does not use `imu.npz` yet. |
| r4 F3 | P3 | DepthAI 2.29 prints "HIGH_DENSITY is deprecated" on every run. | Suppressed around that one call. |
| r4 F4, r4b N1 | P3, P2 | `getFrame()` drops excess bytes silently, so a padded colour buffer would be stored garbled as COMPLETE; refusing it instead (first fix) would repeat on every frame. | The frame is kept with the whole buffer (`isp_raw`) and the frame-buffer layout (`isp_fb`), the capture is flagged (`WARNING ISP_PADDED`, `isp_padded_frames`), and the converter refuses it as `ISP_PADDED` until the layout is decoded. |
| r4 F5-F11, F19, r4b N3-N5 | P3 | Protocol and message details (quoted USB variable, stale disk figure, rerun of the desk test, udev file name, pip cache, probe JSON mixed with SDK log lines, table rows). | Fixed; `--probe --output FILE`; `--no-cache-dir`; `99-oakd-recorder.rules`. |
| r4 F13-F15, F17, F18, F20 | P3 | Depth-frame exposure not recorded; IMU rate label; `--gravity-source auto` on a capture without IMU; README wording; threshold exactly at half the period; a long line. | Fixed. |
| r4 F16, r4b residue | P3 | Surviving mutants; the protocol's commands were pinned by no test. | `test_protocol_commands_parse_and_match_the_pinned_versions` parses every `oakrec` command with the real parser and pins the install line, the alias, the table codes and four literal lines. |
| r4b N2 | P3 | `imu.npz` was written before `capture.json`. | `capture.json` first. |
| r4 F12 | P3, OPEN | The SDK scales the IMX378 intrinsics for 1248 x 936 by 936/3040 instead of 4/13 (+0.064 %, cx −0.4 px). | Not changed: recoverable from the recorded EEPROM dump on the PC; to be done if the camera is an IMX378. |
| r4b N6 | P3, OPEN | `os._exit` does not flush C stdio (inference, not observed). | Not changed. |
| r4c R1 | P3, OPEN | A colour buffer SMALLER than expected stops as `UNEXPECTED` with replug advice, although it would repeat. No bad data is stored; the desk test shows it. | Not changed (found after the last full test run). |
| r4c R2 | P3, OPEN | After a disk error on `imu.npz` the capture says COMPLETE and has no manifest. | Not changed. |
| r4c R3 | P3, OPEN | Six reviewer mutants survive (idle drain inside `record`, depth-exposure fixture, status-line field, two table rows). | Not changed. |

Side effect outside the work folders: the reviewer's and the author's reproductions of r4 F1 made apport write
`/var/crash/_usr_bin_python3.8.1000.crash` (6.3 MB). It was not deleted (deletions need the user).
