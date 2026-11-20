# Review r4 — OAK-D field recorder, converter and Korean field protocol (preflight r4 diff)

- Reviewer: independent fresh-context reviewer (Claude Code subagent), model **Claude Opus 5.5** (`claude-opus-5-5`).
- Date: 2026-11-21. Mode: read-only on the repository; all probes, logs and copies in this directory.
- Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `a76c5cc8574efc8ef377805c18c3d0a7649bea99`,
  six modified files (the subject of this review). Worktree status was identical before and after the review.

## Scope and file hashes (SHA-256 of the files as reviewed)

| File | SHA-256 |
| --- | --- |
| `tools/oakd_field_recorder.py` | `973cf7131f217e9e24d7f8e0516fb8e55d9ff0ca979489f4066b2428705247de` |
| `tools/oakd_recording_convert.py` | `f3d7f781eed56e8f758183c04536a156138b73c80aff7280c1a0c09d3563227b` |
| `ros2/motar_camera_policy/FIELD_RECORDING_KO.md` | `2bd235de340cbf744e9f1022dbaf97ae270091ef385702179d62ed3c460a526a` |
| `tests/test_oakd_field_recorder.py` | `c95d31153719243337a886b5d70f5444039c46a48e7cf7003f5d6f3497d7f0f8` |
| `tests/test_oakd_recording_convert.py` | `c888c7af4c8597939f08df3fd950ae41626e4aafcdd3fbb9c4476c748420c5ec` |
| `ros2/motar_camera_policy/README.md` | `4305fdd9a6b88914384be90ced88397c725b73f548214300fe28979f8ad2d511` |
| `tools/oakd_outdoor_replay.py` (read by the converter, unchanged) | `0119b6f7b7b5164b7b9d8f3f669a936a1144fa4cccd0dbe77f2c83e720a17939` |
| `preflight_r4/diff_vs_a76c5cc.diff` (= live `git diff`, same hash) | `fd9f0bc1c0c8beb700f7f97b215971ace2fe9fd2354fa85f7ea3d34292e336c8` |

The author's preflight material and earlier review reports were read as claims; every conclusion below rests on the code,
the real SDK, the DepthAI v2.29.0 sources or Luxonis primary documents, and is marked where it is an inference.

## What I ran (all CPU, `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `taskset -c 6,7,12,13`, `timeout`)

1. Repository tests, as specified: `test_oakd_field_recorder test_oakd_recording_convert` → **41 tests OK** (`tests_worktree.log`).
2. Real DepthAI 2.29.0.0 (x86_64 venv, no camera): API existence, enum names, docstrings, default values, ISP sizes,
   `ImgFrame.getFrame` behaviour with excess data, `MessageGroup` indexing (`probes/sdk_probe1.*`, `probes/sdk_probe2.*`).
3. The recorder run **as `__main__`** (the way the TA runs it) with the real SDK objects and only `dai.Device` replaced,
   OV9782 and IMX378 profiles (`probes/main_hybrid.py`, `probes/main_hybrid_*.log`); the two captures it wrote
   (`probes/R4_SMOKE_NOT_DATA_main_*`, synthetic, never data) converted with the PC converter in `--dry-run`, both depth
   coordinate models (`probes/convert_hybrid.log`).
4. Protocol §2.1–2.4 and the desk test literally, with Ubuntu 20.04's `/usr/bin/python3` 3.8.10 + pip 20.0.2 in a throwaway
   `HOME` (`env -i`), plus deliberately wrong commands (`probes/install_test.sh`, `probes/install_test.log`). The throwaway
   HOME (124 MB) was deleted afterwards.
5. Import-then-exit abort characterisation (`probes/abort_*.log`).
6. Per-frame encode time and size (`probes/encode_time.*`).
7. 14 own mutants on copies of `tools/` and the two test files (`probes/run_mutants_r4.py`, `mutation_r4.log`,
   `mutation_results_r4.json`); copies deleted afterwards.
8. Primary sources fetched into `src/`: depthai-core v2.29.0 `src/opencv/ImgFrame.cpp`, `src/utility/Logging.cpp`,
   `src/device/CalibrationHandler.cpp`, `src/pipeline/node/{Sync,StereoDepth,ColorCamera}.cpp`; depthai-python v2.29.0.0
   `ImgFrameBindings.cpp`; Luxonis docs pages (ColorCamera, StereoDepth, IMU node, BMI270, BNO08x, Sync) and the
   **Luxonis ISP-scaling spreadsheet** linked from the ColorCamera page (CSV export of every sheet,
   `src/isp_scaling_gid*.csv`).

## Verdict per criterion

| Criterion | Verdict | Measured scope |
| --- | --- | --- |
| C1 first-run correctness | **LIMITED_VALIDITY** | Every DepthAI call exists in 2.29.0.0 and is used with its documented semantics (real SDK, x86). Firmware-side constraints are supported by primary sources where they exist (below). The device, the Jetson aarch64 wheel and the firmware are NOT_TESTED. |
| C2 no unrecoverable loss | **LIMITED_VALIDITY** | Thresholds, ground fit, registration (full EEPROM dump), colour exposure and gravity are recorded or reconstructable. Two gaps: per-frame IMU samples can be starved by the 50-message IMU queue (F2, inference), and the ISP-layout guard cannot detect padding (F4). |
| C3 protocol literal correctness | **LIMITED_VALIDITY** | All commands as written parse and run on Python 3.8/pip 20.0.2 (install, alias, verification lines, probe, record, copy, `sha256sum -c`). What the TA sees differs from the text in three places (F1 abort trace, F3 DeprecationWarning, F10 probe file), plus small command hazards (F5, F7, F8). Safety statements hold in the code. |
| C4 converter accepts / refuses | **PASS** | Fake-SDK tests plus real-SDK-object captures (OV9782 1280x800, IMX378 1248x936) convert with both depth models; incomplete, hash-mismatch and layout cases refuse with named codes (existing tests). Minor: F15. |
| C5 tests non-vacuous | **LIMITED_VALIDITY** | 41 tests pass; 10 of my 14 mutants are caught; 4 survivors (two of them diff behaviour: R03, R12) and the protocol text is not pinned by any test (F16). No earlier test was weakened. |
| C6 regressions / inconsistencies | **PASS** | No regression found; docstrings, README and protocol agree with the code on sizes, constants and defaults. Two wording nits (F14, F17). |

### C1 evidence in detail

- **IMX378 at ISP 4/13 → 1248 x 936** (real SDK `getIspSize()` = (1248, 936)). Luxonis' ISP-scaling spreadsheet, sheet
  "4056 x 3040" (`src/isp_scaling_gid312835569.csv`): `0.3076923077,4,13,...,1248,936,TRUE,,TRUE,TRUE,,TRUE,,TRUE` —
  ISP supported, horizontal multiple of 32, vertical multiple of 8, *ISP output support* TRUE, *Depth alignment* TRUE.
  The former 1/4 profile: `0.25,1,4,...,1014,760,TRUE,,FALSE,TRUE,,TRUE,,FALSE` — depth alignment FALSE. Together with
  the firmware message quoted in forum 5933 ("Disparity/depth width must be multiple of 16"), this **confirms the r4
  change from primary sources**. Stated in the sheet's own note: "Depth output has to be divisible by 16. Alternative is
  to use stereo.setOutputSize()".
- **OV9782 / OV9282-on-CAM_A at THE_800_P → 1280 x 800**, unscaled; 1280 is a multiple of 16. Behaviour of a ColorCamera on
  a socket the firmware reports as OV9282 (forum 6070: grey output) is NOT_ESTABLISHED; if its ISP type is not YUV420p/NV12
  the recorder refuses UNEXPECTED_ISP_LAYOUT (named).
- **Sync**: `setSyncThreshold(timedelta)` and default 10 ms / attempts −1 confirmed in the real SDK; the docstring says
  "-1 (default) then the node will only send a message if successfully synchronized". 50 ms is the value of the
  `depth_video_synced` example and of the forum-5933 script (`0.5 / FPS`). Edge: 50 ms is exactly half the frame period,
  see F18 (P3).
- **Depth alignment**: `setDepthAlign(CAM_A)`, `setLeftRightCheck(True)` (required), `setOutputSize`, the two
  `Use...SpecTranslation(False)` setters exist. Requested output aspect equals the colour aspect for both profiles, so the
  "keep aspect ratio with cropping" behaviour documented for `setOutputSize` does not crop. The 2.29 `rgb_depth_aligned`
  example undistorts RGB on the device (`setMeshSource(CALIBRATION)`) to match the aligned depth, which supports — but does
  not prove — the converter's `undistorted_pinhole` hypothesis; the recorder keeps everything needed to decide later.
- **IMU**: BMI270 stable accelerometer points 25/50/100/200/250 Hz (Luxonis BMI270 page) — 100 Hz is valid. BNO08x
  accelerometer points 15/31/62/125/250/500 Hz, "requested rate rounds up" — 100 Hz becomes 125 Hz (F14); rotation vector
  up to 400 Hz. Probe/record only enable `ROTATION_VECTOR` when the IMU name starts with `BNO08`.
- **IR**: `setIrLaserDotProjectorIntensity(float, mask=-1) -> bool` and `setIrFloodLightIntensity` exist; docstring: "0 or
  negative to turn off", "The emitter is turned off by default". The recorder sets the flood to 0, the dot projector only
  when `--projector > 0`, and refuses when the dot setter returns false.
- **USB / boot**: `dai.Device()` then `startPipeline(pipeline)` is the pattern of the 2.29 `rgb_depth_aligned` example.
- **Stereo preset**: `HIGH_DENSITY` exists in 2.29 but is deprecated ("will be removed ... replaced with DEFAULT"); it
  still works and matches `tools/oakd_camera.build_pipeline`, but prints a warning (F3).
- **Wheels**: PyPI 2.29.0.0 has `cp38`/`cp39`/`cp310`/`cp311` `manylinux_2_17_aarch64.manylinux2014_aarch64` wheels, no
  `Requires-Dist`; pip 20.0.2 understands the `manylinux2014` tag (verified with the x86 twin of the same tag scheme).

**What the first real run (the desk test) must show** — listed under NOT_TESTED at the end.

## Findings

| # | Sev | Where | Finding |
| --- | --- | --- | --- |
| F1 | P2 | `tools/oakd_field_recorder.py:953-994` (import at 961, argument checks at 755/968) | Fast refusals after `import depthai` abort the interpreter with a C++ stack trace and exit 134 |
| F2 | P2 | `tools/oakd_field_recorder.py:821`, `855-858` | IMU host queue of 50 messages can be shorter than the age of the frame taken from the 4-deep RGB-D queue: frames lose their accelerometer window (inference; BNO08x most exposed) |
| F3 | P3 | `tools/oakd_field_recorder.py:306`; `FIELD_RECORDING_KO.md:194` | Every recording prints a `DeprecationWarning: HIGH_DENSITY is deprecated` that the protocol does not mention |
| F4 | P3 | `tools/oakd_field_recorder.py:515-523`, `865` | `isp_layout` size check is vacuous for YUV420p/NV12: `getFrame()` always returns exactly H*3/2 x W and silently drops excess data, so stride/plane padding would be saved as garbled colour with status COMPLETE |
| F5 | P3 | `FIELD_RECORDING_KO.md:290-291` | `USB=/media/$USER/USB이름` breaks for a volume label with a space (e.g. `NO NAME`, `USB DISK`); quote it |
| F6 | P3 | `FIELD_RECORDING_KO.md:274` | "PC 의 빈 공간(약 4.4 GB)" is stale: `df` shows 3.3–3.4 GB free on this PC today |
| F7 | P3 | `FIELD_RECORDING_KO.md:146-150`, `165-166` | Re-running the desk test after the student sends a fixed recorder hits OUTPUT_EXISTS (fixed folder names); no instruction to rename |
| F8 | P3 | `FIELD_RECORDING_KO.md:111-113` | `sudo tee /etc/udev/rules.d/80-movidius.rules` overwrites an existing rule of that name (not "adds one file") |
| F9 | P3 | `FIELD_RECORDING_KO.md:71`, `27`, `304` | pip also writes ~31 MB to `~/.cache/pip`; `rm -rf ~/oakd_rec_pkgs` does not remove it |
| F10 | P3 | `FIELD_RECORDING_KO.md:144`, `153`; recorder `:966` | DepthAI's logger writes to **stdout** (`Logging.cpp`: `stdout_color_sink_mt`), so any SDK warning during `--probe` lands inside `oakd_probe.json` and is not seen on screen |
| F11 | P3 | `FIELD_RECORDING_KO.md:321` | Remedy for UNSUPPORTED_DEPTHAI_VERSION ("alias 를 다시 입력") does not cover a failed or wrong install |
| F12 | P3 | `tools/oakd_field_recorder.py:400`; converter `:335` | `intrinsics_at_output` for 1248x936 comes from `getCameraIntrinsics(..., keepAspectRatio=True)`, which scales by 936/3040 (0.30789) instead of the ISP's 4/13 (0.30769) and shifts cx by −0.4 px; larger error if the EEPROM stores the colour calibration at a non-4:3 size |
| F13 | P3 | `tools/oakd_field_recorder.py:862` | Mono (stereo) exposure state is not recorded; only the colour 3A |
| F14 | P3 | `tools/oakd_field_recorder.py:33`, `366` | IMU `rate_hz` 100 is the requested rate; a BNO08x runs at 125 Hz |
| F15 | P3 | `tools/oakd_recording_convert.py:444-448` | `--gravity-source auto` ignores `pipeline.imu.present`; a no-IMU capture with a stored extrinsic refuses IMU_NO_SAMPLES instead of falling back to nominal |
| F16 | P3 | `tests/` | Mutant survivors R02, R03, R04, R12, and no test pins the protocol commands against `parser()` or the numpy pin against `NUMPY_PIN` |
| F17 | P3 | `ros2/motar_camera_policy/README.md:561-562` | "expected to fail on a device for the same two reasons": the 1014-px width is established (spreadsheet), the 10 ms sync failure is an inference |
| F18 | P3 | `tools/oakd_field_recorder.py:107` | Sync threshold exactly half the frame period: non-hardware-synced IMX378 pairs at ~50 ms offset pass or fail on timestamp jitter |
| F19 | P3 | `FIELD_RECORDING_KO.md:123-126`, `315` | No fallback when the TA does not know the VIO service name (r1 text said "ask the student"); USB_SPEED has no fallback if the drone has no USB3 path (desk test reveals it) |
| F20 | P3 | `ros2/motar_camera_policy/README.md:543` | Reflowed line of ~170 characters in the Field-tools paragraph (cosmetic) |

### F1 (P2) — fast refusals abort with a C++ stack trace

Evidence (`probes/install_test.log`, `abort_variants.log`, `abort_output_exists.log`, `abort_window.log`): with the
protocol's own install (pip 20.0.2 `--target`, depthai 2.29.0.0 + numpy 1.24.4, Python 3.8.10) and the `oakrec` alias,
`--frames 101`, a missing `--pitch-deg`, a missing `~/oakd_rec` (no `mkdir -p`) and an existing output folder each print
the correct `oakd_field_recorder refused: CODE: ...` line and then

```
terminate called after throwing an instance of 'std::system_error'
Stack trace (most recent call last) in thread ...:
#7 ... depthai.cpython-38-...so ... dai::getLazyTarXzFunction ...
#6 ... cmrc::embedded_filesystem::open ...
...
Aborted (core dumped)        -> exit 134
```

Reproduced 5/5 and 3/3. Minimal reproduction: `python3 -c "import numpy, depthai"` aborts (exit 134); `import depthai` alone
exits 0; any sleep ≥ 0.05 s after import exits 0. The process exits while DepthAI's lazy firmware-decompression thread is
still starting. On this PC apport wrote a 6.3 MB `/var/crash/_usr_bin_python3.8.1000.crash`. Refusals that need device
search time (NO_DEVICE) exit cleanly with 2. Impact: OUTPUT_EXISTS (re-used folder name, desk-test rerun F7), BAD_ARGUMENT
and FRAMES_OUT_OF_RANGE are the most likely operator errors in the field; the TA sees 15+ lines of C++ trace and "core
dumped" under the refusal line, which the protocol does not describe and which contradicts the r4 claim "an SDK error is
named, never a bare traceback". No data is lost (no folder exists yet on these paths). Observed on the x86_64 wheel; the
aarch64 wheel is NOT_TESTED but ships the same lazy-resource code.

Fix (both verified on x86, `abort_fix_check.log`): (a) run `check_record_args` and the missing-argument check in `main`
**before** `import depthai` (no import, no thread); and (b) end `main` with `sys.stdout.flush(); sys.stderr.flush();
os._exit(code)` on every return path after the import (`import numpy, depthai, os, sys; os._exit(2)` exits 2, 3/3).
Add a protocol line: "a refusal line may be followed by C++ lines; only the refusal line matters".

### F2 (P2, inference, LIMITED_VALIDITY) — IMU samples can be starved per frame

The RGB-D host queue is `maxSize=4, blocking=False` (`:820`), the IMU queue `maxSize=50, blocking=False` (`:821`), and the
IMU queue is drained only once per frame (`:855`). While a frame is compressed and hashed, newer groups arrive; the next
`tryGet()` returns the oldest of the four retained groups, i.e. a frame about 0.3–0.4 s old once the per-frame host time
exceeds ~0.3 s. Measured encode+SHA-256 time: 0.15–0.17 s per frame on an i7-13700K core (`encode_time.log`); an Orin NX
core is slower (NOT_TESTED, a factor 2–4 is plausible). The IMU queue keeps the newest 50 messages: about 0.5 s at 100 Hz
on a BMI270 (marginal but non-empty overlap with the frame's window `[t−0.5, t]`), but on a BNO08x with accelerometer
(125 Hz after rounding) and rotation vector (100 Hz) in separate reports, 50 messages cover roughly 0.2 s, which can lie
entirely after `t`. Such frames are saved with `accel_count = 0`, logged as `WARNING IMU_NO_SAMPLES`, and the converter's
IMU gravity refuses the whole capture (`IMU_NO_SAMPLES`), leaving only the phone-app pitch with roll assumed 0. Whether
the device packs one report per message is not established; this finding is an inference from the queue sizes.
Fix: IMU queue `maxSize` 1000 (or more), drain it inside `_next_group`'s polling loop, and additionally keep every drained
sample (e.g. `imu_all.npy` per capture, listed in the manifest) so the still-camera gravity can be averaged over the
whole capture. Report `accel_count` per frame on screen for the first frame already exists; add a capture-level count.

### F3 (P3) — DeprecationWarning on every recording

`probes/main_hybrid_*.log`: running the recorder as `__main__` prints
`oakd_field_recorder.py:306: DeprecationWarning: HIGH_DENSITY is deprecated, will be removed in future releases and replaced with DEFAULT.`
plus the source line, before `estimated ... MB/frame`. Protocol §6.1 says the first output is `estimated ...`. Fix:
`warnings.filterwarnings('ignore', message='HIGH_DENSITY is deprecated')` around the call (keeping the preset, which
matches the flight adapter), or mention the two lines in §4/§6.

### F4 (P3) — ISP layout guard cannot see padding

depthai-python 2.29 `ImgFrameBindings.cpp` `getFrame`: for YUV420p/NV12 the shape is `{H*3/2, W}`; if the data is larger
"`// logger::warn("ImgFrame has excess data ...")`" is commented out and a view of the first bytes is returned. Confirmed
with the real SDK: a frame with 4096 extra bytes returns shape (1404, 1248) (`sdk_probe2.log`). So
`arr.size != width*height*3//2` in `isp_layout` can never fire for these types; stride or plane-offset padding would be
recorded as garbled colour under status COMPLETE. Risk is low for the chosen sizes (spreadsheet: "ISP output support" TRUE
for 1248x936; 1280x800 unscaled), hence P3. Fix: record `len(colour.getData())` per frame and refuse/flag when it differs
from `W*H*3/2`; optionally record `colour.getRaw().fb` stride and plane offsets.

### F5–F11, F19 (P3) — protocol details

- F5: write `USB="/media/$USER/USB이름"` (quotes) and say the name may contain spaces.
- F6: replace "약 4.4 GB" by "`df -h` 로 확인" or the measured value at sending time.
- F7: §4 step 6: "다시 시험할 때는 폴더 이름 끝에 `_2` 를 붙입니다".
- F8: use a new file name (`/etc/udev/rules.d/99-oakd-recorder.rules`) or check `ls /etc/udev/rules.d/` first.
- F9: add `--no-cache-dir` to the pip command (and to the offline-kit line), or add `rm -rf ~/.cache/pip` to §11.7.
- F10: write the probe JSON to a file argument (`--probe-output`) instead of stdout, or note that log lines may appear in
  the file.
- F11: "2.4 의 두 줄 결과를 학생에게 보내기" for UNSUPPORTED_DEPTHAI_VERSION.
- F19: "서비스 이름을 모르면 학생에게 물어봐 주세요" (restore); USB_SPEED: "책상 시험에서 나오면 학생에게 연락 (학생이 `--allow-usb2` 여부를 정함)".

### F12–F18 (P3) — data / documentation details

- F12 (`CalibrationHandler.cpp:204-279`, else-branch for a 4:3 calibration): `scale = resizeHeight / height`, then
  `cx -= (width*scale − resizeWidth)/2`. For 4056x3040 → 1248x936 that is fx·0.307895 (ISP: 0.307692, +0.064 %) and
  cx −0.4 px; the author's real-SDK smoke shows fx 561.97 for 1825.2·4/13 = 561.6. Sub-pixel at the edges, recoverable
  because `calibration_eeprom` is recorded; the converter should derive K from the EEPROM size and the exact ISP scale
  (or record `keepAspectRatio=False` intrinsics as well). If the EEPROM stores CAM_A at 1920x1080, the SDK takes the
  16:9 branch and the error is several percent — the probe output shows the stored size.
- F13: also save `colour_3a(depth)` (exposure/ISO that the depth frame may carry from the mono cameras) as
  `depth_exposure_time_us` etc.; harmless if −1/0, useful to explain missing depth in bright sky.
- F14: record `rate_hz_requested: 100` and, for BNO08x, `rate_hz_effective: 125` (Luxonis IMU docs: "rounds up").
- F15: in `convert`, treat `meta['pipeline']['imu']['present'] is False` like "no extrinsic" for `auto`.
- F16: mutant survivors (`mutation_results_r4.json`): R02 IMU not drained during warm-up; R03 probe plan drops
  `isp_scale` (diff behaviour); R04 even-height guard removed (diff behaviour, no profile exercises it); R12
  `depth_minus_colour_ms.min` reported as the max (diff behaviour; the fixture's offset is constant, so min == max). Add a
  fixture with varying depth-minus-colour offsets, assert `recording_plan['isp_scale']`, and a test that extracts every
  `oakrec ...` command from `FIELD_RECORDING_KO.md` and parses it with `R.parser()` (and checks `numpy==NUMPY_PIN`).
- F17: "fails for the depth width (Luxonis ISP-scaling sheet: 1014x760 depth alignment FALSE) and may deliver no
  frames with the 10 ms threshold".
- F18: 55–60 ms keeps pairing unique at 10 FPS (period 100 ms) and removes the jitter edge; record the choice.

## Safety statements (C3) — checked in the code

- No vehicle path: the recorder imports only stdlib, numpy and depthai; no socket, serial, subprocess, ROS or MAVLink
  (`grep`); `vehicle_commands_sent: False` is a constant, consistent with the code.
- Emitters: flood set to 0 when an IR driver is listed; dot projector only for `--projector > 0`; false return refuses.
  The device resets on close, so nothing stays on after the run (SDK docstring: off by default).
- VIO restore and check: §4.5, §11.2–11.3 and checklist 17–18.
- Install isolation: `PIP_USER=0 ... --target ~/oakd_rec_pkgs`; nothing landed in `~/.local` in the literal run; the
  plain `python3 -c "import depthai"` still failed afterwards (environment unchanged). Exceptions: pip cache (F9) and the
  conditional udev rule (F8).

## Remaining NOT_TESTED until a real device is used (what the desk test must show)

1. The aarch64 wheels on the actual JetPack (glibc, Python version, pip version), and whether F1's abort also occurs there.
2. Device acceptance of the full pipeline: 12MP + ISP 4/13 + aligned depth 1248x936 (IMX378), or 800P colour + aligned
   1280x800 (OV9782); device memory; both stereo modes. Must show: `status COMPLETE`, 3 frames, `depth valid` > 0 % on a
   textured indoor scene.
3. Sync node delivery: `depth_minus_colour_ms` within ±50 ms; `sequence_gaps` small; no FRAME_TIMEOUT.
4. ISP frame layout: the colour of the desk-test frames decodes correctly on the PC (F4 would show as garbled chroma).
5. Real sensor names (OV9782 vs OV9282-on-CAM_A grey case vs IMX378) and the stored calibration sizes (F12).
6. IMU model (BMI270 or BNO08x), `accelerometer samples` per frame > 0 on every frame (F2), and the presence/validity
   of the IMU-to-CAM_A extrinsic.
7. IR projector at 0.5 on the drone's USB power (X_LINK_ERROR risk); flood/dot setter return values.
8. USB speed on the drone's port; VIO stop/restart behaviour on the actual system (service, Docker).
9. Encode time per frame on the Orin (recording duration claims in §4 and §6; F2 margin).
10. Which depth coordinate model the device uses (M01/M02 decide it on the PC).
11. Converter FOV coverage of the trained 66x54-deg crop for a real IMX378 (spec VFOV ~55°: thin margin; FOV_SHORTFALL is
    recoverable with `--allow-fov-shortfall`).

## Side effects outside this directory

- `/var/crash/_usr_bin_python3.8.1000.crash` (6.3 MB, 2026-11-21 02:08) was written by apport during my F1 reproduction.
  I did not delete it (my write scope is this directory); it is safe to remove.
- Throwaway install HOME and mutant copies were created inside this directory and deleted.

## Overall verdict

**PASS_WITH_LIMITS.** No P0 or P1 finding. The r4 changes that matter for the first run (IMX378 ISP 4/13 for a
16-multiple aligned depth, 50 ms Sync threshold, warm-up from the first frame, named UNEXPECTED/INTERRUPTED exits) are
correct against the real SDK and, for the ISP/alignment constraint, against Luxonis' own ISP-scaling table; the required
desk test is the right gate for everything that only the device can show. Before sending, fix F1 (abort trace on the most
likely operator errors) and F2 (enlarge and drain the IMU queue, keep all samples); the P3 items are cheap protocol and
record improvements. Hardware, Jetson, firmware and the real calibration remain NOT_TESTED; nothing here is flight
qualification.
