# Unit C repair r2 (2026-11-18): fixes for re-review r2 (N1, N4 and cheap P3s)

Worker: the orchestrating Claude Code session (Fable 5.2), directly; scope limited to the findings of
`review_r2/REVIEW_r2.md`. CPU only, no device. No git state change.

| Finding | Change | Test |
| --- | --- | --- |
| **N1 (P1)** a reported but silent IMU stopped the recording with 0 frames | `tools/oakd_field_recorder.py` record loop: no refusal; the frame is saved, counted in `frames_without_imu_samples`, announced once with `WARNING IMU_NO_SAMPLES` (says the PC must use `--gravity-source nominal`), and the count is in the final status line. Docstring and the Korean protocol (step 7.2, error table) say it is a warning and that recording continues | `test_imu_absent_silent_and_present` (silent IMU → COMPLETE, 3 frames, one warning; IMU-less and working-IMU controls), `test_final_status_line_reports_frames_without_imu_samples`; converter side already covered by `test_gravity_source_choice` (dead IMU: auto refused, nominal converts) |
| **N4 (P2)** version/import errors told the operator to `pip install depthai` | `INSTALL_HINT`: isolated `--target ~/oakd_rec_pkgs` install + `oakrec` alias, "do NOT pip install into the drone's own Python"; used by both messages | `test_install_hint_never_points_at_the_drones_python` (wrong version, accepted version control, import failure through `main`) |
| N3 (P3) the record lost whether gravity was `auto` or explicit | `tools/oakd_recording_convert.py`: `requested` keeps the argument as given, in both gravity records | `test_gravity_source_choice` (nominal, imu, auto→imu, auto→nominal) |
| N5 (P3) protocol required every capture.json COMPLETE | protocol steps 7.3, 11.1 and checklist row 13: one COMPLETE folder per planned recording; INCOMPLETE folders are kept, not deleted | — (text) |
| N7 (P3) sequence-gap counter untested | — | `test_sequence_gaps_are_counted` (gap of 3, control 0) |
| N8 (P3) `PIP_USER=0` unexplained | protocol 2.2: one-line explanation | — (text) |
| — | converter `IMU_NO_SAMPLES` message names `--gravity-source nominal` | existing code-only assertion |

Mutation checks (scratch copy `repair_r2/mut/`, results `repair_r2/mutation_results.json`): 8/8 killed
(refuse again; status line drops the count; gap counter zero; plain pip hint; plain import hint; no count; requested =
resolved; warning without the nominal hint). Baseline OK.

Verification: recorder tests 19 OK, converter tests 16 OK, README-pin tests 8 OK, `check_public_docs` PASS,
Python 3.8.10 compile OK; with the real DepthAI 2.29 and no device `--record` exits with `NO_DEVICE` and creates no
folder; with no depthai `--probe` prints the isolated-install hint. Full PUBLIC_CPU: see
`logs/repair_r2_public_cpu.log`.

Still open: N2 (cleanup race with two simultaneous conversions to one name), N6 (frames rule set once from the first
frame; the recorder still refuses per recording with INSUFFICIENT_DISK), r1 P3-1/-5/-7/-8/-10. NOT_TESTED: a real
OAK-D, the Orin, the aarch64 wheel, `pip --target` on JetPack, real calibration/IMU/projector behaviour, real
bytes per frame, `DEVICE_BUSY` against the running VIO.

Changed files (SHA-256):
    5a68317f456ecea548d4623bebb479bf5796108576a9e890f303d585eb4065fb  tools/oakd_field_recorder.py
    d083d60ae9ed9202314fc9b52288a366cd49245c4d4b137b6cd66a6a173cf281  tools/oakd_recording_convert.py
    502e292b553dbe3ef57c3492c3b6bf81420ffcd29cae1df05ab38502103b694a  tests/test_oakd_field_recorder.py
    0a6d5153ea223b37a1d2938eadd4e9f5052f1d3bd7b6feabb82af712d8b39eac  tests/test_oakd_recording_convert.py
    f00e68a4d1fd76ec6bd3d7d6829ba55fa42b58aac01ea0aed3ea78229e8d6de1  ros2/motar_camera_policy/FIELD_RECORDING_KO.md
    a38e8bb1db146b9a0dabe57caf5e0f4e5e2d049ee4d62cea0c25c219b7e2bf66  ros2/motar_camera_policy/README.md
