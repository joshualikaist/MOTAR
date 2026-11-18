# Review r2: unit C repair r1 (OAK-D field recorder, converter, Korean protocol)

**Reviewer.** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only on the worktree. I wrote only under `review_r2/`.

**Target.** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `7efbbd9`. The change is still uncommitted; `git status` is unchanged by this review.

**Files reviewed.** I did not take the repair report's claims on trust; every closure below was rechecked against the code.

| File | SHA-256 (prefix) |
| --- | --- |
| `tools/oakd_field_recorder.py` | 286a5196 |
| `tools/oakd_recording_convert.py` | b9147e32 |
| `tests/test_oakd_field_recorder.py` | 3e6eaa00 |
| `tests/test_oakd_recording_convert.py` | 39d23e0e |
| `FIELD_RECORDING_KO.md` | 80b3dc8e |
| `README.md` | a38e8bb1 |

All six are identical to `logs/repair_r1_sha256_before_full_suite.txt`. The implementer's full-suite log (`Ran 4291 tests … OK (skipped=10, expected failures=10)`) therefore covers these exact bytes, and I did not rerun the full suite.

**Host.** Boot `407ae422…`. Kernel fault count 1, the same as the implementer's before/after count (a known user-space segfault). cpu4-5 are offline; every run used cores 0-3 and 6-23.

## Verdict: FAIL (one new P1 introduced by the repair; easy to fix)

All 3 r1 P1s are CLOSED at the CPU level. Of the r1 P2s, 5 are CLOSED and 1 is PARTIAL.

The repair also made a dead or silent IMU fatal at record time (`IMU_NO_SAMPLES` on the first frame). There is no bypass and no protocol fallback. With that hardware fault, the field session would yield no data at all. The converter's new `--gravity-source nominal` could have converted those captures, and the recorder's own message wrongly says it still can.

**Open findings: P0 0, P1 1, P2 1, P3 11.**

## 1. r1 findings re-verified

| r1 finding | Status | Evidence (my own check) |
| --- | --- | --- |
| P1-1 per-frame 3A not recorded | **CLOSED** (real values NOT_TESTED) | See "P1-1 detail" below. |
| P1-2 disk budget | **CLOSED** with residual P3 (N6) | See section 3, "Disk arithmetic". |
| P1-3 install into the drone's user site | **CLOSED** (Jetson NOT_TESTED) | See section 3, "Commands, flags and the alias". The new P2 N4 is related. |
| P2-1 setter results ignored | **CLOSED** | `R:767-779`. Dot projector returning false → `PROJECTOR_NOT_APPLIED`, INCOMPLETE, 0 frames. Control: dot true → COMPLETE, 3 frames. Flood returning false is recorded as `ir_flood_off_applied: False` (`logs/q3_semantics.log`). Mutants M2 and M3 killed. |
| P2-2 IMU checks / converter override | **PARTIAL** | Converter side CLOSED; recorder side over-corrected into N1. See "P2-2 detail" below. |
| P2-3 `--pitch-deg` optional | **CLOSED** | Required (`R:182, 902`). `--record` without it → `BAD_ARGUMENT` and no folder, both in tests and via the KO alias. `NO_NOMINAL_PITCH` refusal; control `--no-imu-nominal-check` → OK. Mutant M9 killed. |
| P2-4 deciding scene not named | **CLOSED** | KO:173-190 puts M01/M02 (flat ground, pitch 0 and +10) first and names them as the depth-model decision; the README says the same. My rerun end-to-end probe still separates the models. Matched model: 4/4 accepted, ground error vs tape 0.15 mm and 0.10 mm. Mismatched model: 4/4 refused (`logs/q2_end_to_end.log`). |
| P2-5 three surviving mutants | **CLOSED** | R10, C6 and C10 are all killed now, by `test_record_refuses_depth_not_aligned_to_the_colour_size`, `test_coverage_includes_the_depth_source` and `test_roll_limit_of_the_nominal_check` (`logs/mutations_r1_survivors.log`). |
| P2-6 `DEVICE_BUSY` over-inferred | **CLOSED** (real VIO NOT_TESTED) | See "P2-6 detail" below. |

**P1-1 detail.**
- AE ON and AWB AUTO are now set on `initialControl` (`R:258-259`).
- Per-frame exposure, ISO, colour temperature, lens position and raw lens position go into each npz, each `capture.json` frame entry and a summary (`R:521-573, 805-832`).
- The converter copies them into every output npz and into `recording.json.device.colour_3a` (`C:386-406, 472-474, 522-525`).
- My end-to-end probe shows the values from the fake SDK (800-830 µs, ISO 100-103, 5200-5203 K) arriving in the converted npz keys and the recording summary. `load_recording`, `frame_inputs` and `replay_recording` still run, and the replay CLI exits 0.
- Real SDK: all five getters exist (`logs/q1_real_sdk.log`); a fresh offline frame reads 0, so real values are NOT_TESTED.
- Mutants M6, M7, M8 and C1 are killed.

**P2-2 detail.**
- **Converter side, CLOSED.** `--gravity-source auto|imu|nominal`:
  - `auto` with the IMU disagreeing gives `IMU_NOMINAL_PITCH_MISMATCH`; it does **not** fall back to nominal.
  - `nominal` with the IMU disagreeing is accepted, and the record says the IMU was not used.
  - `imu` without an extrinsic gives `NO_IMU_EXTRINSICS`; `auto` without an extrinsic uses nominal and says so.
  - With a dead IMU, `auto` gives `IMU_NO_SAMPLES` and `nominal` gives OK.
  - Mutants C2, C3 and C8 are killed.
- **Recorder side, over-corrected.** A silent IMU now raises `IMU_NO_SAMPLES` on the first frame and saves 0 frames, with no bypass. This is new finding N1.

**P2-6 detail.**
- `classify_open_failure` uses error text plus XLink state and status.
- Every enum name it compares against exists in the real depthai 2.29 (`XLinkDeviceState`: `X_LINK_BOOTED`, `X_LINK_FLASH_BOOTED`; `XLinkError_t`: `X_LINK_INSUFFICIENT_PERMISSIONS`, `X_LINK_DEVICE_ALREADY_IN_USE`, `X_LINK_ALREADY_OPEN`), so none of the checks is vacuous.
- On real `dai.DeviceInfo` objects:
  - BOOTED → `DEVICE_BUSY`
  - INSUFFICIENT_PERMISSIONS → `DEVICE_PERMISSION`
  - the permission error text → `DEVICE_PERMISSION`
  - UNBOOTED with "Failed to boot" → `DEVICE_OPEN_FAILED`
  - BOOTLOADER → `DEVICE_OPEN_FAILED`
  - nothing listed → `NO_DEVICE`
- All 6 cases match the expected code. Mutants M4 and M5 are killed.

| r1 P3 | Status |
| --- | --- |
| P3-1 IMX378 width 1014 / ISP stride | open (hardware; fails closed) |
| P3-2 stereo config not dumped | **fixed**: real SDK records confidence 245, `KERNEL_5x5`, LR threshold 10, bilateral 0, MILLIMETER, max disparity 190 |
| P3-3 sequence numbers | **fixed**: `sequence_num` per frame plus `sequence_gaps`. The gap counter is untested (N7). |
| P3-4 partial output blocks rerun | **fixed**: `.partial` plus rename; a mid-run refusal leaves nothing; an existing `.partial` is refused and left untouched. A race remains (N2). |
| P3-5 fake-SDK captures labelled real | open (naming convention only) |
| P3-6 traceback instead of a code | **fixed**: `OS_ERROR` in `capture.json` and in `main`; mutant M11 killed |
| P3-7 RGB border blend under shortfall | open |
| P3-8 test name overclaims depthai | open |
| P3-9 KO readability | **fixed**: `tar` and `scp` removed, terms defined, pitch-sign self-check added |
| P3-10 nominal tolerance vs accelerometer bias | open; `--gravity-source nominal` mitigates it |

## 2. New findings from the repair

### P1

**N1. A silent IMU blocks the entire field session** (`R:810-814`, KO:150, KO:270).
- If `getConnectedIMU()` names an IMU but the first frame has 0 accelerometer samples, the recorder raises `IMU_NO_SAMPLES` before saving anything. `capture.json` is INCOMPLETE with 0 frames (`logs/q3_semantics.log`: "silent IMU -> REFUSED IMU_NO_SAMPLES INCOMPLETE/IMU_NO_SAMPLES frames 0").
- No recorder flag bypasses it. The flag list in the log has no `--allow-no-imu`.
- The KO only says "한 번 더 시도, 또 나오면 메모".
- **Failure scenario:** the IMU firmware does not stream (a known class of OAK IMU issue) or the IMU node stalls. Every recording at the 기계동 site refuses, and the TA returns with no data.
- The capture would have been usable: `--pitch-deg` is now mandatory and `--gravity-source nominal` converts IMU-less captures.
- The refusal message "(the PC can still use --gravity-source nominal)" is false, because no frame is saved.
- The behaviour is also inconsistent: a device reporting no IMU (`'NONE'`) records normally (control: COMPLETE, 3 frames).
- **Fix:** make the zero-sample case a loud warning with `imu_samples: NONE` recorded (or add an explicit `--allow-no-imu` and give the KO that action), and correct the message.

### P2

**N4. The recorder's version and import refusal tells the TA to run a plain `pip install depthai==2.29.0.0`** (`R:196`, `R:897`).
- That is exactly the install into the drone's own Python environment that P1-3 removed.
- KO:267 says to re-enter the alias instead, but a TA reading the terminal sees the pip hint first.
- **Scenario:** the TA forgets the alias in a new terminal, `python3` imports the VIO's depthai (or none), the recorder prints "pip install depthai==2.29.0.0", and the TA runs it.
- **Fix:** the message should name the KO 2.2 `--target` command and the `oakrec` alias.

### P3

**N2. `.partial` ownership race** (`C:456, 535-539, 548-550`).
- The existence check happens before the first frame is converted. `partial` is assigned before `mkdir(parents=True)`, and `mkdir` of `<partial>/frames` succeeds inside a folder another process has just created.
- Simulated (`check_output_space` patched to create `<out>.partial` with a sentinel):
  - On success, the foreign folder and its sentinel are renamed into the output.
  - On a later refusal, the foreign folder is deleted by `rmtree`.
- This needs two concurrent conversions to the same name inside a sub-second window.
- **Fix:** `partial.mkdir()` without `parents`/`exist_ok` as the atomic ownership claim, and set the cleanup flag only after it succeeds.

**N3. The gravity record loses the user's choice** (`C:500-511`). `requested` is always `imu` after `auto` resolves to imu. The nominal branch has no `requested` key, so "auto because there is no extrinsic" and "explicit nominal" differ only by a method-string suffix.

**N5. KO 11.1 expects every `capture.json` to read COMPLETE**, but every refused-then-retried recording (`IMU_NO_SAMPLES`, `PROJECTOR_NOT_APPLIED`, `FRAME_TIMEOUT` …) leaves an INCOMPLETE folder, and KO 6.3 says to retry under a new name. The TA may think the session failed, or delete folders. Say that INCOMPLETE folders from retries are expected and are copied as they are.

**N6. The frames rule is fixed once, from M01's first frame.** If M01 is small (X ≤ 1.8 MB → 20 frames) and later forest or projector scenes compress worse, the minimum set can reach 22 × 20 × 3.59 MB = 1.58 GB. That exceeds the stated Orin 1.5 GB and the 1 GB USB stick. Both sets can reach 5.0 GB, over the stated 3.5 GB. The recorder refuses each recording with `INSUFFICIENT_DISK`, and `sha256sum -c` catches a short USB copy, so nothing is corrupted silently, but the session can stop early.

**N7. The sequence-gap counter is untested.** Mutant M12 (`sequence_gaps` never incremented) survives.

**N8. KO 2.2 does not explain `PIP_USER=0`.** It is harmless, but it is the one undefined term left in the install line.

## 3. Korean protocol, followed as the TA would

**Order.** The summary at KO:12-13 and section 11 run: stop VIO (3) → record (6, 9) → 11.1 status check → 11.2 restart VIO → 11.3 check VIO works → 11.4 copy with `cp -r` + `sync` → 11.5 `sha256sum -c` → 11.6 hand over, keeping `~/oakd_rec` until the student confirms → 11.7 `rm -rf ~/oakd_rec_pkgs`. There is no tar on the Orin. The order is correct.

**Safety.**
- Props off, no arming and no vehicle commands (KO:17-18).
- The drone's install is left unchanged (KO:21).
- The VIO is stopped and later restarted and checked.
- No flight-readiness claim (KO:297: "비행 가능 여부를 판단하는 시험이 아닙니다"). The README keeps "NOT_TESTED" and "never in flight".

**Site.** "기계동 앞 운동장 옆 숲" appears 2 times; "놀이터" appears 0 times.

**Commands, flags and the alias** (`logs/q4_ko_commands_usrbin.log`). I ran the exact KO text with `HOME` set to a scratch folder, `/usr/bin/python3` 3.8.10, and a stand-in for `~/oakd_rec_pkgs`. The stand-in contained the real cp38 depthai 2.29.0.0 and numpy 1.23.
- KO 2.4 line 1 printed `2.29.0.0 …/oakd_rec_pkgs/depthai…so`.
- KO 2.4 line 2 printed the system state unchanged.
- `oakrec --probe` → `NO_DEVICE`, exit 2, empty JSON.
- `oakrec --record …` without `--pitch-deg` → `BAD_ARGUMENT`.
- The full KO 6 command → `NO_DEVICE`, exit 2, no folder created.
- `$PYTHONPATH` was empty after every `oakrec` call: the `VAR=… cmd` form in the alias does not leak into the shell, even from `~/.bashrc`.
- Every error code in the KO table is raised by the recorder, and every recorder code is in the table.
- Converter flags `--gravity-source`, `--dry-run` and `--depth-coordinate-model` exist; the dry run prints `output_mb_estimate`.

**ABI and shadowing.**
- PyPI has `depthai-2.29.0.0-cp38-cp38-manylinux_2_17_aarch64.manylinux2014_aarch64.whl`, and `requires_dist` is null (no numpy pin). Pip 20.0.2 (Ubuntu 20.04) understands manylinux2014.
- depthai's pybind11 numpy use is resolved at runtime, so whatever numpy `--target` installs (1.24.4 for 3.8) is acceptable.
- If a different `python3` is first on `PATH` (I hit the conda 3.11 here), the cp38 `.so` is skipped, `import depthai` becomes a namespace package and the recorder refuses with `UNSUPPORTED_DEPTHAI_VERSION (found None)`. It fails closed, but its pip hint is N4.
- The Jetson aarch64 install itself is NOT_TESTED.

**Disk arithmetic.** Measured synthetic sizes:

| Frame | 2 DN noise | Incompressible bound |
| --- | --- | --- |
| Raw, per frame | 1.558 MB | 3.59 MB |
| Converted, per frame | 0.695 MB | 1.15 MB |

| Set | Rule bound (KO §10) | Synthetic estimate | Worst case (first frame small, later frames incompressible) | Orin requirement | Fits? |
| --- | --- | --- | --- | --- | --- |
| Minimum, 22 recordings | 0.79 GB | 0.69 GB | 1.58 GB | 1.5 GB | Rule and estimate: yes. Worst case: no, and the recorder refuses safely (N6). |
| Both, 70 recordings | 2.52 GB | 2.18 GB | 5.0 GB | 3.5 GB | Rule and estimate: yes. Worst case: no (N6). |

PC side, against `df` 4.4 GB free:

| Set | Copy + converted output | Fits? |
| --- | --- | --- |
| Minimum | 1.0–2.1 GB | Fits |
| Both | 3.15–6.6 GB | Does not fit safely; the KO says so and prescribes an external drive or a staged transfer |

The converter refuses `INSUFFICIENT_DISK` before writing (control: enough disk → OK).

**Readability.** Pip, alias, `PYTHONPATH`, udev, `cp -r`, `sync` and `sha256sum` are all defined. The pitch-sign self-check was added. The remaining gaps are N5 and N8. Using the alias in every new terminal is in the checklist.

## 4. Commands and results (logs in `review_r2/logs/`)

```
targeted (aerialgym env, worktree, CUDA_VISIBLE_DEVICES="" OMP/MKL=1 PYTHONNOUSERSITE=1 PYTHONPATH=$PWD:$PWD/tests taskset 0-3,6-23)
  unittest test_oakd_field_recorder test_oakd_recording_convert test_oakd_camera test_camera_ros2_package test_camera_runtime_unit_b
  -> Ran 79 tests in 7.930s OK                                         targeted_tests.log
recorder tests under the depthai venv (Python 3.8.20, numpy 1.23) -> Ran 16 tests OK         recorder_tests_py38.log
/usr/bin/python3 3.8.10 py_compile (scratch copy) -> PY38_COMPILE_OK                           py38_compile.log
probes/q1_real_sdk.py (real depthai 2.29, no device) -> getters/setters present, enum names present, 6/6 classify, both profiles
  build+serialize, no-IMU pipeline 6 nodes                                                      q1_real_sdk.log
probes/q2_end_to_end.py -> 3A flows through; matched model accepted 4/4 (0.15 / 0.10 mm), mismatched refused 4/4;
  replay CLI exit 0; dry-run estimate 2.37 MB                                                   q2_end_to_end.log
probes/q3_semantics.py -> gravity-source table, new refusals + controls, .partial ownership incl. race   q3_semantics.log
KO alias with fake HOME + /usr/bin/python3 3.8                                                  q4_ko_commands_usrbin.log
  (q4_ko_commands.log = same with conda 3.11 first on PATH: fails closed)
probes/mutate_r2.py on scratch copy -> 20/21 killed; survived M12 (sequence_gaps)               mutations_r2.log
r1 survivors R10, C6, C10 re-run -> 3/3 killed                                                  mutations_r1_survivors.log
full suite: not rerun; the 6 file hashes equal repair_r1_sha256_before_full_suite.txt (4291 OK)
```

## 5. NOT_TESTED

- Real OAK-D, Jetson, aarch64 `pip --target`, and real 3A, IMU and projector behaviour.
- Whether explicit AE ON / AWB AUTO behave identically to the firmware defaults on the device. They are the defaults by API contract, both are recorded, and the deployed `oakd_camera` adapter leaves both at default.
- `DEVICE_BUSY` against a running VIO.
- Real bytes per frame.
- Which depthai the VIO uses.
