# Review r4c — audit of the fixes for review r4b findings N1–N5

- Reviewer: independent fresh-context reviewer (Claude Code subagent), model **Claude Opus 5.5** (`claude-opus-5-5`).
- Date: 2026-11-21. Read-only on the repository; probes and logs in `review_r4/r4c/`.
- Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, HEAD `a76c5cc`. The live `git diff` hashes to
  `0a81b865b4eb3609b341bb26e7e1184507a2e7c49ba52ac91fd62ba3a46c4b34`, identical to
  `preflight_r4/diff_final2_vs_a76c5cc.diff`, and was unchanged at the end of this audit.
- The r4c delta: I rebuilt the r4b files (HEAD + `diff_final_vs_a76c5cc.diff`; the rebuilt recorder reproduces the r4b
  hash `565eaeee…`) and diffed them against the worktree (`r4c/delta.diff`, 275 lines). That delta touches the
  recorder, the converter, the protocol and both test files; the README is unchanged since r4b.

| File | SHA-256 |
| --- | --- |
| `tools/oakd_field_recorder.py` | `e8287ff3e7a0872bec2c43b807a2eaf78aabe29dd43a6ffad005812a6496c3bb` |
| `tools/oakd_recording_convert.py` | `4e42d20ea95cd1e65bd41326d2e431c4756ab5161344365d85b3acf5ec780770` |
| `ros2/motar_camera_policy/FIELD_RECORDING_KO.md` | `af0b675321b99558ad95e973ec76b444ea74d6adddd62671edf15cb6090cc0a2` |
| `tests/test_oakd_field_recorder.py` | `61e7737ed63d7ca3a3414ede536cd30311a201133f8afc82a277c0bbc40b87b9` |
| `tests/test_oakd_recording_convert.py` | `1535625c358778e6ee37584a06b8ca2712e30dfa8777caeebaf86e38fa15500e` |
| `ros2/motar_camera_policy/README.md` (unchanged since r4b) | `417415b087f0c2678934b3df95b88f23813f7deb257ee6cafba13954f5b3afca` |

## What I ran

All runs used `CUDA_VISIBLE_DEVICES=""`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1`,
`taskset -c 6,7,12,13` and `timeout`.

1. Repository tests `test_oakd_field_recorder test_oakd_recording_convert`: **47 tests OK** (`r4c/tests_worktree_r4c.log`).
2. The recorder run as `__main__` with **real DepthAI 2.29.0.0 message objects** (only `dai.Device` replaced), OV9782
   and IMX378, with the colour buffer exact (`PAD=0`), **14 976 bytes too large** (`PAD=14976`, the size an 8-row plane
   padding of 1248x936 would add) and **1 248 bytes too small** (`PAD=-1248`) (`r4c/main_hybrid_pad.py`,
   `r4c/hybrid_pad.log`). I then inspected the npz, `capture.json` and `MANIFEST.sha256`, and ran the converter in
   `--dry-run` (`r4c/npz_check.log`, `r4c/convert_r4c.log`).
3. A disk error (half the bytes written, then `ENOSPC`) injected at each of the three final writes:
   `capture.json`, `imu.npz` and `MANIFEST.sha256` (`r4c/n2_disk_errors.py`, `r4c/n2_disk_errors.log`).
4. 18 mutants on copies (`r4c/run_mutants_r4c.py`, `r4c/mutation_r4c.log`, `r4c/mutation_results_r4c.json`). The copies
   were deleted afterwards.

## Status per finding

| # | Status | Evidence |
| --- | --- | --- |
| N1 (P2) | **FIXED** (oversized buffer) | A larger buffer is no longer refused. The recorder saves `isp_raw` (the whole `getData()` buffer) and `isp_fb` (`stride, p1Offset, p2Offset, p3Offset, width, height, bytesPP` from `getRaw().fb`, −1 if unreadable) next to `isp` (`:928-942`). Real SDK, IMX378 `PAD=14976`: `isp_raw` (1 767 168,) uint8, `isp_fb` [1248, 0, 0, 0, 1248, 936, 1] (host-made frame, so offsets 0 — device values NOT_TESTED). Flagged in four places: one `WARNING ISP_PADDED` log line, `capture.json.isp_padded_frames` = 3, `frames[i].isp_bytes_delivered`, and `"isp_padded_frames": 3` in the final status line. `sha256sum -c`: OK. Converter: `refused: ISP_PADDED: 3 colour frames were delivered with padding ...` (named, before any frame is read); the unpadded twin converts. A padded capture can be analysed as normal data only by reading the npz `isp` array directly, bypassing the converter; the replay reads only converter output. The **smaller** buffer case is not handled well, see R1. |
| N2 (P3) | **FIXED** | `capture.json` is now written before `imu.npz`, then `MANIFEST.sha256` (`:1005-1024`). See "Disk error at each write" below. |
| N3 (P3) | **FIXED** | `UDEV_HINT` and the DEVICE_PERMISSION message name "USB vendor 03e7 (FIELD_RECORDING_KO.md 2.5)"; no `80-movidius` left in the recorder (now pinned by the protocol test; mutant C18 caught). |
| N4 (P3) | **FIXED** | §12 rows for `usage:` / `error:`, other `WARNING:` lines and `WARNING ISP_PADDED`. The UNEXPECTED_ISP_LAYOUT row now says "두 번 나오면 멈추고 오류 문장을 학생에게 보내 주기". The last two rows are not pinned by tests (C16, C17 survive). |
| N5 (P3) | **FIXED** | §11.4 note: a missing `~/oakd_probe.json` gives one error line and the rest is copied (`FIELD_RECORDING_KO.md:301`). |
| F16 remainder | **PARTLY → mostly FIXED** | The protocol test now restricts the code check to the §12 table and also requires the three WARNING rows. It pins `mkdir -p ~/oakd_rec`, the quoted `USB="..."`, `"$USB"/`, the `99-oakd-recorder.rules` file and the probe line, and asserts there is no `80-movidius` in the recorder. My r4b survivors N15, N16, N17 and N21 are now caught (C04–C07). Still unpinned: draining while waiting inside `record` (C01, C03), the depth-frame exposure source (C02), `isp_padded_frames` in the final status line (C12), and the two table rows above (C16, C17). |
| N6 (P3) | **OPEN (recorded)** | C-stdio flush before `os._exit` unchanged, as stated by the coordinator. It is an inference and was not observed. |
| F12 (P3) | **OPEN (recorded)** | IMX378 `intrinsics_at_output` from the SDK's aspect-ratio heuristic is unchanged, to be handled on the PC. The EEPROM dump is recorded, so it is recoverable. |

All other r4 findings remain as in r4b (FIXED).

### Disk error at each of the three final writes (N2)

| Failing write | Files left | `capture.json` | Consequence |
| --- | --- | --- | --- |
| `capture.json` | frames, truncated `capture.json` | unreadable | Process prints OS_ERROR (exit 2). Converter: CAPTURE_UNREADABLE (named). Unavoidable without an atomic write. |
| `imu.npz` | frames, `capture.json`, partial `imu.npz`, **no manifest** | COMPLETE | The status record survives (the r4b goal). The frames are intact and hash-checked by the converter, which ignores `imu.npz`, so the data stay usable. However, `capture.json` claims a hash for a partial `imu.npz`, the TA saw OS_ERROR while §11.1's `grep` shows COMPLETE, and §11.5's `sha256sum -c` finds no manifest for that folder. |
| `MANIFEST.sha256` | everything, partial manifest | COMPLETE | `sha256sum -c` reports the truncated lines; data usable. |

The `INSUFFICIENT_DISK` precheck makes all three unlikely. None of them can make unusable data look usable, so this
leaves only the P3 polish R2.

## New findings

| # | Sev | file:line | Finding | Fix |
| --- | --- | --- | --- | --- |
| R1 | P3 | `tools/oakd_field_recorder.py:928-934`; `FIELD_RECORDING_KO.md:341` | A colour buffer **smaller** than H*3/2*W makes `colour.getFrame()` raise inside the SDK (`ImgFrame doesn't have enough data ... required 1536000, actual 1534752`). It is reported as `UNEXPECTED: RuntimeError ...` with the USB-drop hint ("unplug and replug ... record again"), and the UNEXPECTED row has no "stop after two" rule. The capture is INCOMPLETE with 0 frames, so no bad data is stored. But the failure is deterministic and the advice sends the TA into repeated replug/retry. Reproduced with real SDK objects for both profiles. The desk test catches it first. | Before `getFrame()`, `if delivered < expected: raise RecorderError('UNEXPECTED_ISP_LAYOUT', 'colour frame carries %d bytes, needs %d')`, which the table already routes to "stop after two". Optionally add "두 번 반복되면 멈추고 보내기" to the UNEXPECTED row too. |
| R2 | P3 | `tools/oakd_field_recorder.py:1005-1024` | After an `imu.npz` write error, `capture.json` is COMPLETE, references `imu.npz` by hash, and the manifest is missing. | Write `imu.npz` inside `try/except OSError`, set `meta['imu_all_samples']['error']` on failure, and keep writing the manifest. Since `capture.json` is written first, re-write it, or write `imu.npz` before it inside the `try` with the error recorded. |
| R3 | P3 | `tests/test_oakd_field_recorder.py` | Survivors C01/C03 (drain while waiting not passed in `record`), C02 (`DepthMsg` inherits the colour exposure getters), C12 (`isp_padded_frames` missing from the status line), C16/C17 (two new table rows). | Give `DepthMsg` distinct exposure values. Assert the status line's `isp_padded_frames`. Add an IMU-queue fixture that yields only while the RGB-D queue is empty. Add the two table phrases to the protocol test. |

The delta broke nothing I could find:

- 47 tests pass.
- Exact-buffer captures are byte-for-byte unaffected (no `isp_raw` key; `PAD=0` runs COMPLETE, manifest OK, converter
  PASS).
- The padded path roughly doubles a frame's size (`isp` + `isp_raw`). The §10 table, which goes by the first frame,
  then lowers `--frames` or stops the session — but ISP_PADDED appears at the desk test first, and the student decides.

## Mutants (18; 12 caught)

C04–C11, C13–C15 and C18 are caught.

The 6 survivors:

| Mutant | Comment |
| --- | --- |
| C01 frame loop without idle drain | inherited from r4b N04 |
| C02 depth exposure from the colour frame | inherited from r4b N13 |
| C03 first warm-up wait without drain | inherited from r4b N14 |
| C12 status line without `isp_padded_frames` | new |
| C16 `usage:` / `error:` row removed | new |
| C17 UNEXPECTED_ISP_LAYOUT row back to "retry, memo" | new |

All six are documentation or diagnostic pins. None of them guards a data-integrity path. The integrity paths are all
caught:

- padded frames counted (C08);
- `isp_raw` from the delivered buffer (C10);
- `isp_fb` field order (C14);
- converter refusal (C09);
- write order (C13);
- `imu_all_samples` in the on-disk `capture.json` (C07).

## Findings that remain open

| # | Sev | State |
| --- | --- | --- |
| F12 | P3 | Open by decision: IMX378 intrinsics scaling, to be handled on the PC. |
| N6 | P3 | Open by decision: C-stdio flush before `os._exit`, to be handled if observed. |
| R1 | P3 | New, open. |
| R2 | P3 | New, open. |
| R3 | P3 | New, open (test pins). |

No P0, P1 or P2 is open. Everything that only the device can show stays NOT_TESTED, as listed in r4/r4b:

- the aarch64 wheel;
- pipeline acceptance;
- Sync delivery;
- the real ISP buffer size and `fb` values (now flagged instead of refused);
- sensor names and calibration size;
- IMU model and sample counts;
- projector power;
- USB speed;
- VIO stop/restart;
- Orin encode time;
- the depth coordinate model;
- IMX378 FOV coverage.

## Overall verdict (sending the recorder file and the protocol to the operator for the desk test)

**PASS_WITH_LIMITS.**

- N1–N5 are fixed.
- An oversized colour buffer is now kept whole with its layout, flagged in the log, `capture.json` and the status line,
  and refused by name by the converter, so it cannot pass as normal data through the conversion path.
- The remaining items are P3: R1 (an undersized buffer is misnamed UNEXPECTED with retry advice; no bad data stored;
  caught at the desk test), R2, R3, plus F12 and N6, which are open by decision.

Hardware, the Jetson, the firmware and the real calibration remain NOT_TESTED; nothing here is flight qualification.
