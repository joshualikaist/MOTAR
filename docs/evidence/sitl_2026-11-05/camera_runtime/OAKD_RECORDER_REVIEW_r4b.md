# Review r4b — audit of the fixes written after review r4

- Reviewer: independent fresh-context reviewer (Claude Code subagent), model **Claude Opus 5.5** (`claude-opus-5-5`).
- Date: 2026-11-21. Read-only on the repository; everything below lives in `review_r4/r4b/` (probes, logs, deltas).
- Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, HEAD `a76c5cc`, same six modified files.
  The live `git diff` hashes to `e18316ca31f8da40af9cd2fbfcca08f3bfe84132c7d55de3d680a5810960b66b`, identical to
  `preflight_r4/diff_final_vs_a76c5cc.diff`; it was unchanged at the end of this audit.

## Files audited (SHA-256)

| File | SHA-256 |
| --- | --- |
| `tools/oakd_field_recorder.py` | `565eaeee2e3e57c714d8adf856b4be69648a655dd71bb76f467caab0709a4986` |
| `tools/oakd_recording_convert.py` | `f7fd4227bfafa11fd6f301d3e645a9f2cdf05b1587ce4f8a094e552ed1164453` |
| `ros2/motar_camera_policy/FIELD_RECORDING_KO.md` | `0baa54403f18bfc74611ed90749c7ce8e13c4fd178242e9d1fe83e99550f14f2` |
| `tests/test_oakd_field_recorder.py` | `5ca688dcaadb87725078750bc789a8a73a63f7eef88877f329e0160a2c383730` |
| `tests/test_oakd_recording_convert.py` | `0016be48803da075c97dea7bc443f3b1e94ff2cda265022aa1a0cdba3ec5c501` |
| `ros2/motar_camera_policy/README.md` | `417415b087f0c2678934b3df95b88f23813f7deb257ee6cafba13954f5b3afca` |

Method for isolating the new part: I rebuilt the r4 versions (HEAD + `diff_vs_a76c5cc.diff`; the rebuilt recorder and
protocol reproduce the r4 hashes `973cf713…` and `2bd235de…`) and diffed them against the worktree:
`r4b/delta_tools.diff`, `r4b/delta_docs.diff`, `r4b/delta_tests.diff`.

## What I ran (CPU only; `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1` for tests; `taskset -c 6,7,12,13`; `timeout`)

1. Repository tests: `test_oakd_field_recorder test_oakd_recording_convert` → **47 tests OK** (`r4b/tests_worktree_r4b.log`).
2. **F1 reproduction with the protocol's own install** on `/usr/bin/python3` 3.8.10 / pip 20.0.2, throwaway `HOME`,
   `env -i`, the new pip line literally (`--no-cache-dir`), the `oakrec` alias (`r4b/f1_repro.sh`, `r4b/f1_repro2.sh`,
   logs alongside). The install folder (93 MB) was deleted afterwards.
3. The recorder run as `__main__` with real DepthAI 2.29.0.0 objects and only `dai.Device` replaced, OV9782 and IMX378
   (`r4b/main_hybrid_*.log`), then `sha256sum -c MANIFEST.sha256` in each capture, then the converter in `--dry-run`
   with `--gravity-source auto` and `nominal` (`r4b/convert_r4b.log`).
4. Real-SDK check of `ImgFrame.getData()` against `getFrame()` and of `getRaw().fb` (`r4b/getdata_probe.*`).
5. 21 own mutants on copies of `tools/`, the two test files and the protocol (`r4b/run_mutants_r4b.py`,
   `r4b/mutation_r4b.log`, `r4b/mutation_results_r4b.json`); copies deleted afterwards.

## 1. Status of the r4 findings (checked in the code)

| # | r4 sev | Status | Evidence |
| --- | --- | --- | --- |
| F1 | P2 | **FIXED** | `main` now runs the missing-argument check and `check_record_args` before `import depthai` (recorder `:1021-1027`); `__main__` leaves through `_leave` = flush stdout/stderr + `os._exit` (`:1069-1077`). Reproduced with the protocol install: OUTPUT_EXISTS, FRAMES_OUT_OF_RANGE, missing `--pitch-deg`, missing parent folder ×3 each → the single refusal line and **exit 2**, no C++ trace (`f1_repro2.log`). Paths that do import: NO_DEVICE for `--record` and for `--probe --output` → exit 2, clean; a fast refusal after the import (stub `depthai` reporting 2.24.0.0 → UNSUPPORTED_DEPTHAI_VERSION) → exit 2, clean (`f1_repro.log`). No new apport report appeared in `/var/crash`. The missing-parent message now names `mkdir -p`. |
| F2 | P2 | **FIXED** (recording side) | IMU host queue `IMU_QUEUE_SIZE = 2000` (`:119`, `:852-853`); `drain()` runs on every empty poll in all three `_next_group` calls (`:869`, `:880`, `:887`) and after each received frame; every sample is also kept in `imu.npz`, which is hashed in `capture.json.imu_all_samples` and listed in `MANIFEST.sha256`. With real SDK objects 531 samples were kept for 3 frames + warm-up, and `sha256sum -c` reported `imu.npz: OK`. The converter does not use `imu.npz` yet (frames with empty windows still need `--gravity-source nominal`), but no sample is lost any more. Starvation itself remains an inference that only the device can confirm. |
| F3 | P3 | **FIXED** | `warnings.catch_warnings()` + `simplefilter('ignore', DeprecationWarning)` around the preset (`:320-322`). The real-SDK `__main__` run no longer prints the warning (`main_hybrid_*.log`, first line is now `estimated ...`). |
| F4 | P3 | **FIXED, by refusing** | `isp_bytes_delivered = getData().size` must equal `W*H*3/2`, else UNEXPECTED_ISP_LAYOUT (`:902-907`). Real SDK: `getData()` returns the full buffer (1 767 168 bytes for a 1248x936 frame with 14 976 extra bytes) while `getFrame()` returns (1404, 1248), so the check sees what `getFrame` hides (`getdata_probe.log`). The refusal policy introduces a new risk, see N1. |
| F5 | P3 | **FIXED** | `USB="/media/$USER/USB이름"` with "따옴표는 그대로" (`FIELD_RECORDING_KO.md:296`). |
| F6 | P3 | **FIXED** | The PC figure is replaced by "`df -h` 로 확인 (2026-11-21 기준 약 3.4 GB)"; matches `df` today (3.3–3.4 GB). |
| F7 | P3 | **FIXED** | §4 step 6: append `_2` on a repeated desk test, otherwise OUTPUT_EXISTS. |
| F8 | P3 | **FIXED** | New file `/etc/udev/rules.d/99-oakd-recorder.rules`, reversible by deleting it. The recorder's own messages still name `80-movidius.rules`, see N3. |
| F9 | P3 | **FIXED** | `--no-cache-dir` in the protocol and in `INSTALL_HINT`; the literal install left no `~/.cache/pip` (`f1_repro.log`: "cache dir exists: no"). |
| F10 | P3 | **FIXED** | `--probe --output FILE` writes the JSON to the file and prints `probe written to FILE` (`:1035-1043`); the protocol uses it (`:146`, `:155`). |
| F11 | P3 | **FIXED** | Table row now ends "그래도 나오면 2.4 의 두 줄 결과를 학생에게 보내기"; new rows for `oakrec: command not found` and `ModuleNotFoundError`. |
| F12 | P3 | **NOT_FIXED** | `calibration_summary` and the converter's `capture_calibration` are unchanged; I found no recorded decision to decline it. The EEPROM dump is recorded, so it stays recoverable on the PC. |
| F13 | P3 | **FIXED** | `depth_exposure_time_us` / `depth_sensitivity_iso` from `colour_3a(depth)` per frame in `capture.json` (`:908`, `:938-939`). Not pinned by a test (mutant N13 survives). |
| F14 | P3 | **FIXED** | `pipeline.imu.rate_hz_is` says the rate is the requested one and that a BNO08x rounds it (up to 125 Hz). |
| F15 | P3 | **FIXED** | `imu_recorded = pipeline.imu.present is not False`; `auto` → nominal without an IMU (converter `:448-450`); tested both ways (mutant N11 caught). |
| F16 | P3 | **PARTLY** | R03 (probe `isp_scale`), R04 (odd height) and R12 (min/max) are now pinned, and a new protocol test pins the five `oakrec` commands, the pip line, the alias and the error-table codes. Still unpinned: R02-like behaviour (N04, N14: draining while waiting inside `record`), N13, and the protocol's non-`oakrec` lines (N15 `mkdir -p`, N16 quoting, N17 udev file). See §4. |
| F17 | P3 | **FIXED** | README now says the 1014 x 760 aligned depth is "listed as unsupported" in Luxonis' ISP-scaling table and the 10 ms threshold "may deliver no frames". |
| F18 | P3 | **FIXED** | `SYNC_THRESHOLD_MS = 60` (`:118`); test asserts 50 < 60 < 100. |
| F19 | P3 | **FIXED** | "(서비스 이름을 모르면 학생에게 물어봐 주세요)" restored; USB_SPEED row adds "바꿀 수 없으면 학생에게 연락". |
| F20 | P3 | **FIXED** (the flagged line) | README line 543 is reflowed. Line 550 (165 characters, "Each frame also carries ...") predates this diff. |

## 2. What the fixes broke or introduced

### Exit through `os._exit` (`_leave`, recorder `:1069-1077`)

- **Output**: `_leave` flushes Python's `sys.stdout`/`sys.stderr` first. The final status line, the refusal line and
  `probe written to` all appeared in every run above, including when stdout was a pipe.
- **Files**: every file is written in a `with` block (closed before `main` returns); after `MANIFEST.sha256`, `record`
  also calls `os.sync()` (`:991-994`). Nothing written by the recorder is left buffered when `os._exit` runs.
- **Device**: `record` and `probe` close the device in their own `finally` blocks before `main` returns, so skipping
  interpreter teardown does not skip `device.close()`.
- **Residual (inference, P3, not observed)**: C-level stdio buffers are not flushed by `os._exit`. DepthAI's spdlog sink
  writes to C `stdout`. If a DepthAI log line is still in a C buffer when stdout is a pipe or a file, it could be lost.
  The protocol runs in a terminal (line-buffered), so this only matters for redirected output.
- **`argparse` errors** still exit through `SystemExit` before any import; this path is clean (exit 2).

### `imu.npz`

- It is written whenever recording reached the queues (`accel_all` is set after `startPipeline` and the emitter setup).
  That includes captures without an IMU, where it holds an empty `(0, 4)` array (tested). An early refusal such as
  PROJECTOR_NOT_APPLIED leaves no `imu.npz` and no `imu_all_samples` key, and `capture.json` and the manifest agree in
  both cases.
- `MANIFEST.sha256` lists frames, then `imu.npz`, then `capture.json`. The protocol's `sha256sum -c` checks it (real-SDK
  captures: all lines `OK`). §11's `grep '"status"'` is unaffected.
- The converter ignores the file: it neither reads it nor checks its hash. Conversion of the new captures succeeded
  (`convert_r4b.log`: auto → imu, nominal → nominal_pitch, FOV PASS).
- **New ordering hazard (N2)**: `imu.npz` is now written *before* `capture.json` in the `finally` block (`:974-986`).
  An `OSError` while writing `imu.npz` (disk full) skips `capture.json` and the manifest. The folder then has frames but
  no status. The converter still refuses it with a named code (CAPTURE_UNREADABLE), but r4 always left an INCOMPLETE
  `capture.json`.

### IMU queue of 2000 and the idle drain

- `tryGetAll()` runs about every 5 ms while waiting. It is cheap, and it keeps the device queue far from its limit
  (~9 s at 225 msg/s even without draining).
- The per-frame `accel` list grows only between frames and is trimmed at each frame. `accel_all` grows to a few
  thousand tuples for 20 frames. Neither is a memory concern.
- The order of operations is unchanged: drain after the group arrives, then compute the window. No correctness change.

### Delivered-byte check on the colour frame

- It is correct against the real SDK (`getData` = full buffer; `getFrame` silently truncates).
- It turns any excess byte into a hard refusal of every frame, see N1.
- The depth frame has no equivalent check. Its width (1280 or 1248) is a multiple of 16, so the risk is low; noted
  only.

### Probe `--output`

- The file is written after the device is closed. Mode `'w'` overwrites the file, which the help text states.
- A refusal leaves no file. Before, the shell's `>` left an empty file. The desk-test `tar` and the §11 `cp` then miss
  `oakd_probe.json`, and §4 already says to drop missing names. §11.4's `cp` does not say this, which is minor.

### Sync 60 ms

- At 10 FPS a partner within 50 ms always exists, so delivery is guaranteed whatever the colour–stereo phase.
- With non-synced sensors (IMX378), the node may pair with a frame 50–60 ms away instead of the nearer one, depending on
  arrival order. That is harmless for still scenes, and `depth_minus_colour_s` records it per frame.
- For the hardware-synced OV9782 + OV9282 (OAK-D Pro W), the offset is about 0.

### `--gravity-source auto`

- It is correct.
- `present` is missing in old captures: then it defaults to True, which is the old behaviour.
- `present: False` with an extrinsic gives nominal.
- An explicit `--gravity-source imu` on a no-IMU capture still refuses IMU_NO_SAMPLES, named.

### New findings

| # | Sev | file:line | Finding and fix |
| --- | --- | --- | --- |
| N1 | P2 | `tools/oakd_field_recorder.py:902-907`; `FIELD_RECORDING_KO.md:336` | **Any excess byte in the colour buffer now refuses every frame**, and the protocol's advice for UNEXPECTED_ISP_LAYOUT ("케이블 다시 꽂고 새 폴더로 다시, 반복되면 메모") makes the TA retry a deterministic failure. Whether RVC2 ISP buffers for 1280x800 or 1248x936 carry trailing or plane padding is NOT_ESTABLISHED (936 is a multiple of 8 but not of 16). Before this change, trailing padding was harmless and plane padding garbled the chroma silently. Now both stop the desk test until a new recorder is sent. Because the desk test comes first, this costs a round trip, not the outdoor session. **Fix**: on a mismatch do not refuse. Store the full `getData()` buffer plus `getRaw().fb` (`stride`, `p1Offset`, `p2Offset`, `p3Offset`, `width`, `height`; verified present in 2.29, `getdata_probe.log`) in the npz. Log `WARNING ISP_PADDED` once and decode on the PC from the offsets. At minimum, change the table row for UNEXPECTED_ISP_LAYOUT to "두 번 나오면 멈추고 오류 문장을 학생에게 보내기". |
| N2 | P3 | `tools/oakd_field_recorder.py:974-986` | `imu.npz` is written before `capture.json`, so a write error on `imu.npz` loses the INCOMPLETE status file. **Fix**: write `imu.npz` inside its own `try`, record a failure in `meta['imu_all_samples'] = {'error': ...}`, then always write `capture.json` and the manifest. Also assert the on-disk `capture.json` has `imu_all_samples` (mutant N21 survives because the tests read the in-memory `meta`). |
| N3 | P3 | `tools/oakd_field_recorder.py:484`, `:522` | `UDEV_HINT` and the DEVICE_PERMISSION message still name `/etc/udev/rules.d/80-movidius.rules`, while the protocol now installs `99-oakd-recorder.rules`. **Fix**: name both, or point to "FIELD_RECORDING_KO.md 2.5 step 2". |
| N4 | P3 | `FIELD_RECORDING_KO.md` §12 | `argparse` errors (`usage: ... error: argument --frames: invalid int value`, `unrecognized arguments`) are not in the error table. The other unlabelled warnings (`WARNING: setIrFloodLightIntensity(0) returned false`, `WARNING: frame N has no accelerometer sample`) are not either. **Fix**: one row "`usage:` / `error:` → 명령을 6번 예시와 글자 그대로 비교해서 다시 입력". |
| N5 | P3 | `FIELD_RECORDING_KO.md:297` | When `--probe` refused, `~/oakd_probe.json` does not exist and the `cp` line prints an error for it (the rest is copied). **Fix**: "없는 파일이라는 오류는 무시" or drop the file from the `cp` line when absent. |
| N6 | P3 | `tools/oakd_field_recorder.py:1069-1077` | `os._exit` does not flush C stdio; a DepthAI log line still buffered while stdout is a pipe or file could be lost (inference, not observed). **Fix**: `ctypes.CDLL(None).fflush(None)` before `os._exit`, guarded by `try`. |

## 3. Protocol walk-through as the operator (changed parts and the table)

- §2.2 pip line runs literally on Python 3.8.10 / pip 20.0.2 (exit 0, no `~/.cache/pip`, nothing in `~/.local`).
- §2.3 alias and §2.4 verification lines unchanged and working.
- §2.5.2 udev: new file name, `sudo tee` + `udevadm` lines unchanged; correct, but see N3.
- §3: VIO stop text restores "ask the student".
- §4 desk test: `oakrec --probe --output ~/oakd_probe.json` prints only `probe written to ...` (fake SDK) or the
  refusal line (no camera here); the two 3-frame commands parse and pass all argument checks (test + my runs).
  Step 3 now tells the TA that the refusal line is what counts. Step 6 covers repeats with `_2`.
- §6 first output line: with the warning suppressed, `estimated ... MB/frame` is indeed the first line on an OV9782 /
  IMX378 device (an OV9282-on-CAM_A device prints `WARNING COLOUR_SENSOR_NAME` first, which the table explains).
- §10 PC disk text corrected. §11.4 quoted USB variable; `sha256sum -c` passes with `imu.npz`.
- Error table versus printable codes: every `RecorderError` code in the source plus OS_ERROR, UNEXPECTED and
  INTERRUPTED has a row; this is now enforced by `test_protocol_commands_parse_and_match_the_pinned_versions`
  (mutant N18 caught). Gaps are N1 (wrong advice for a deterministic UNEXPECTED_ISP_LAYOUT) and N4.
- Safety statements still hold. There is still no vehicle command path (imports are stdlib, `warnings`, numpy and
  depthai; enforced by the import test). Emitter handling is unchanged. The VIO is restarted and checked in §4.5 and
  §11.2–11.3. Installs stay in `~/oakd_rec_pkgs`; the udev file is now a separate, removable file.

## 4. Tests

- 47 tests OK. 14 of my 21 mutants are caught (`mutation_results_r4b.json`):

| Mutant | Caught |
| --- | --- |
| N01 argument checks after the import | yes |
| N02 `_leave` via `sys.exit` | yes |
| N03 IMU queue back to 50 | yes |
| N04 frame loop does not drain while waiting | **no** |
| N05 whole-capture samples not kept | yes |
| N06 `imu.npz` missing from the manifest | yes |
| N07 delivered-byte check removed | yes |
| N08 delivered bytes from `getFrame` | yes |
| N09 `os.sync` removed | yes |
| N10 deprecation filter removed | yes |
| N11 converter auto ignores `imu.present` | yes |
| N12 probe JSON also on stdout | yes |
| N13 depth exposure read from the colour frame | **no** (fixture `DepthMsg` inherits the colour getters) |
| N14 first warm-up wait without drain | **no** |
| N15 protocol drops `mkdir -p ~/oakd_rec` | **no** (the test creates `oakd_rec` itself) |
| N16 protocol USB variable unquoted | **no** |
| N17 protocol udev file back to 80-movidius | **no** |
| N18 protocol INTERRUPTED row removed | yes |
| N19 protocol probe back to `> file` | yes |
| N20 protocol desk test `--frames 300` | yes |
| N21 `imu_all_samples` missing from the on-disk `capture.json` | **no** |

- **`test_protocol_commands_parse_and_match_the_pinned_versions` pins what it claims**, which is narrower than "the
  protocol":
  - exactly five `oakrec` command lines (after joining `\` continuations), each parsed by the real `parser()` and,
    for `--record`, accepted by `check_record_args`, with projector 0 or 0.5;
  - the probe writing to `oakd_probe.json`;
  - the exact pip line (`--no-cache-dir`, `SDK_VERSION`, `NUMPY_PIN`), also inside `INSTALL_HINT`;
  - the exact alias;
  - every `RecorderError` code plus OS_ERROR / UNEXPECTED / INTERRUPTED in backticks somewhere in the protocol.

  It does not check that the codes are in the §12 table rather than anywhere else, that `~/oakd_rec` is created by the
  protocol, or any non-`oakrec` shell line. **Fix**: extract the protocol's non-`oakrec` lines and assert
  `mkdir -p ~/oakd_rec` precedes the first `~/oakd_rec/` command. Assert the quoted `USB="..."` form and the
  `99-oakd-recorder.rules` name. Restrict the code check to the §12 table.
- No earlier test was weakened. Every changed assertion follows a deliberate behaviour change:
  - manifest 4→5 and 2→3 lines;
  - sync 50→60 with a stronger min/max fixture;
  - the install-hint string.
- A useful extra test (not required): give `DepthMsg` its own exposure getters, so N13 is caught. Also assert that
  `record` passes `drain` to `_next_group` (N04/N14), e.g. with an IMU queue that only yields while the RGB-D queue is
  empty.

## Remaining NOT_TESTED (unchanged from r4, plus the new points)

Everything only the device can show stays NOT_TESTED:

- the aarch64 wheel and the F1 behaviour on the Jetson;
- device acceptance of both pipelines;
- Sync delivery;
- the real ISP buffer size (now decisive because of N1);
- sensor names and calibration sizes;
- the IMU model and per-frame sample counts;
- projector power;
- USB speed;
- VIO stop/restart;
- Orin encode time;
- the depth coordinate model;
- IMX378 FOV coverage in the converter.

## Overall verdict (sending the recorder file and the protocol to the operator for the desk test)

**PASS_WITH_LIMITS.**

- Both P2 findings of r4 are fixed: F1 was reproduced as fixed with the protocol's own install, and F2 is fixed on the
  recording side.
- 18 of the 20 findings are fixed, one is partly fixed (F16) and one is not fixed (F12, recoverable).
- No P0 or P1 was introduced.
- The one new P2 (N1) cannot cost the outdoor session, because the mandatory desk test comes first. It can cost a
  round trip if the device's ISP buffer carries any excess byte. Storing the raw buffer with its plane offsets instead
  of refusing, or at least correcting the table row, is recommended before or together with sending.

Hardware, the Jetson, the firmware and the real calibration remain NOT_TESTED; nothing here is flight qualification.
