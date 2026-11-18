# Review r3: unit C repair r2 (N1, N4 and P3s from review r2)

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only on the worktree. Wrote only under `review_r3/`.
- **Target:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `7efbbd9`. The change is uncommitted; `git status` is unchanged by this review.
- **Files reviewed:** pinned in `logs/sha256_reviewed.txt`. They match REPAIR_r2.md, and `sha256sum -c` was still OK at the end of the review:
  - `tools/oakd_field_recorder.py` 5a68317f
  - `tools/oakd_recording_convert.py` d083d60a
  - `tests/test_oakd_field_recorder.py` 502e292b
  - `tests/test_oakd_recording_convert.py` 0a6d5153
  - `FIELD_RECORDING_KO.md` f00e68a4
  - `README.md` a38e8bb1 (unchanged since r2)
- **Host:**
  - The kernel fault count is 1, the same known baseline as before.
  - cpu4-5 are offline; all runs used cores 0-3 and 6-23.
  - The orchestrator's full PUBLIC_CPU run (`logs/repair_r2_public_cpu.log`) was running concurrently and had not finished. I did not use it as evidence. None of my checks depends on timing.

## Verdict: PASS_WITH_LIMITS

The CPU-level checks pass. The real OAK-D, the Orin, the aarch64 `pip --target` install and the real IMU, projector and 3A behaviour remain NOT_TESTED. I have not verified the full-suite result on this tree; its log was still in progress.

**Open findings:** P0 0, P1 0, P2 0, P3 9.

## Requested checks

### 1. N1 (P1): a silent IMU — CLOSED

**Recorder** (`R:817-828`):
- A reported IMU that delivers zero samples no longer refuses. Every frame is saved and counted in `meta['frames_without_imu_samples']`, which is written to `capture.json`.
- The first such frame logs `WARNING IMU_NO_SAMPLES …`, naming `--gravity-source nominal`. Later frames log a short warning.
- The final status line carries `frames_without_imu_samples` (`R:922`).

**Probe** (`logs/q5_silent_imu.log`; fake silent-IMU device, synthetic ground plane):
- Recording: COMPLETE, 4 frames, `frames_without_imu_samples` 4 in memory and on disk, accelerometer counts [0, 0, 0, 0], one `WARNING IMU_NO_SAMPLES` plus 3 per-frame warnings.
- Converter with `auto` → **REFUSED** `IMU_NO_SAMPLES`. The message names `--gravity-source nominal`.
- Converter with `imu` → **REFUSED** `IMU_NO_SAMPLES`.
- Converter with `nominal` → OK. The record holds `nominal_pitch` / `requested nominal`. The replay accepts 4/4 frames with a ground error of 0.15 mm against the tape height.
- Controls: a working IMU converts under `auto` (shown as `requested auto`) and under `imu`.
- Through `main()`: exit 0, the status line shows `"frames_without_imu_samples": 3`, and the warning is printed.

**Protocol:** step 7.2 (KO:151-153) and the error-table row (KO:275) say the same thing. It is a warning, not an error; recording continues; measure the pitch carefully and write down that the warning appeared.

**Does warning-only hide anything?** No silent acceptance was found:
- A frame without samples makes `auto` and `imu` refuse the whole capture with a named code. Only the explicit `nominal` choice converts it, and that choice is recorded.
- An IMU that delivers on only some frames gives mixed counts. `frames_without_imu_samples` is then > 0 and visible, and `auto` refuses at the first empty frame.
- Fewer samples per frame (an intermittent fake gave counts [50, 10, 5, 5]) still passes the norm and stillness gates and converts. That is correct, because those frames have real samples.
- Remaining limit (P3 N9 below): for a partly silent capture, `nominal` replaces the IMU on every frame, and the IMU-versus-nominal pitch cross-check is lost for the whole capture.

### 2. N4 (P2): pip hints — CLOSED

Both depthai refusals use `INSTALL_HINT` (`R:196-198, 205, 909-910`). It names the isolated `--target ~/oakd_rec_pkgs` install and the `oakrec` alias, and says "do NOT pip install into the drone's own Python".

`grep "pip install"` over the recorder, the converter, the README and the KO finds only:
- the docstring line describing `--target`;
- `INSTALL_HINT`;
- KO 2.2, which is the `--target` command itself.

Nothing tells the TA to install into the drone's own Python.

### 3. Other r2 P3s

| Finding | Status | Evidence |
| --- | --- | --- |
| N3 `requested` lost | **CLOSED** | `requested_gravity` is kept in both gravity records (`C:444, 507, 513`). The probe shows `auto` / `imu` / `nominal` as given. |
| N5 "all COMPLETE" | **CLOSED** | KO 7.3, 11.1 and checklist row 13: one COMPLETE folder per planned recording; INCOMPLETE folders are kept, not deleted. |
| N7 gap counter untested | **CLOSED** | `test_sequence_gaps_are_counted`; my mutant N7a is killed. |
| N8 `PIP_USER=0` unexplained | **CLOSED** | KO:62 adds a one-line explanation. |

### 4. Regressions

None found.
- Targeted tests (`test_oakd_field_recorder`, `test_oakd_recording_convert`, `test_oakd_camera`, `test_camera_ros2_package`): **54 OK**.
- Recorder tests under Python 3.8.20 (depthai venv): **OK**.
- `/usr/bin/python3` 3.8.10 `py_compile`: **OK**.
- Synthetic end-to-end rerun (`logs/q2_end_to_end.log`) gives the same results as r2:
  - matched depth model: 4/4 accepted, ground error 0.15 mm and 0.10 mm;
  - mismatched model: 4/4 refused;
  - the replay CLI exits 0;
  - 3A values pass through to the output;
  - the dry-run estimate is 2.37 MB.
- My own mutations on a scratch copy (`logs/mutations_r3.log`): **9/9 killed**.
  - N1: count not incremented, warning loses its code, status line drops the count, refuse again on the first silent frame, converter accepts zero-sample frames.
  - N4: plain pip hint in the version message, and in the import message.
  - N3: `requested` set to the resolved source.
  - N7: gaps never counted.

## Open findings (P3 only)

- **N2. `.partial` ownership race.** Two simultaneous conversions to the same name can adopt or delete each other's folder (r2 simulation). Unchanged.
- **N6. Frames rule set once from M01's first frame.** In the worst case, later recordings can exceed the stated Orin and USB budgets. The recorder still refuses each recording with `INSUFFICIENT_DISK`, and `sha256sum -c` catches a short copy.
- **N9 (new). Partly silent IMU loses the cross-check.** With some frames silent, `nominal` is the only way to convert, and it drops the IMU-versus-nominal pitch check for every frame. A pitch-sign or reading error by the TA then goes unchecked in that capture. Suggestion: a per-frame mode that uses the IMU where it has samples and still checks those frames against the nominal pitch, or at least record which frames had IMU data in `recording.json`.
- **N10 (new, nit). KO:155 says INCOMPLETE folders are "PC 에서 자동으로 걸러집니다".** The converter refuses them, one folder at a time, with `INCOMPLETE_CAPTURE`. That is close enough, but "변환 프로그램이 거부합니다" would be exact.
- **r1 P3-1.** IMX378 aligned-depth width 1014 and ISP stride: hardware, fails closed.
- **r1 P3-5.** Fake-SDK captures are labelled `oakd_field_recorder`; only the naming convention guards this.
- **r1 P3-7.** RGB blends with the black border, under `--allow-fov-shortfall` only.
- **r1 P3-8.** The test name overclaims a depthai check.
- **r1 P3-10.** The 5° nominal tolerance is close to the accelerometer bias the norm gate admits.

## Commands (logs in `review_r3/logs/`)

```
targeted: CUDA_VISIBLE_DEVICES="" OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 PYTHONPATH=$PWD:$PWD/tests taskset -c 0-3,6-23
  python -B -m unittest test_oakd_field_recorder test_oakd_recording_convert test_oakd_camera test_camera_ros2_package -v
  -> Ran 54 tests OK                                            targeted_tests.log
depthai venv (3.8.20): unittest discover test_oakd_field_recorder.py -> OK      recorder_tests_py38.log
/usr/bin/python3 -m py_compile (scratch copy) -> PY38_COMPILE_OK               py38_compile.log
probes/q5_silent_imu.py -> silent/intermittent/working IMU x auto/imu/nominal + main() status line   q5_silent_imu.log
probes/q2_end_to_end.py -> end-to-end rerun (same results as r2)               q2_end_to_end.log
probes/mutate_r3.py on scratch/mut -> 9/9 killed, unmutated OK                  mutations_r3.log
sha256sum -c logs/sha256_reviewed.txt at the end -> 6/6 OK
```
