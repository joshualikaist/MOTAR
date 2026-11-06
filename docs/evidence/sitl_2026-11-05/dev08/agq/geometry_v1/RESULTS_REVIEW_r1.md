# Results review r1: AGQ geometry-v1 attempt01 and S3a-P1 gpu_run01 (2026-11-06)

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, independent results reviewer. Read-only on every
  Git worktree and raw/run directory. Writes went only to this directory.
- **Pinned bytes:** worktree `MOTAR-camonly`, commit `f286e29e4d0bd90842f896411dbbac00bfa2349c`. Code and
  preregistrations were read with `git show f286e29:<path>`. `tools/` is unchanged in the working tree. The only
  working-tree changes are draft records under `docs/` (reviewed for wording in section 4).
- **Compute:** CPU only. `CUDA_VISIBLE_DEVICES=""`, `nice -n 10`, cores 16–19, `ulimit -v 8 GiB`, `timeout -k 20`.
  No simulator, no GPU, no network. Kernel fault count was 0 at review time, on the same boot
  (`407ae422-184d-4dbf-a89b-1fd5d05a51df`) as the runs. cpu4/5 are offline.
- **Probes (in this directory):**
  - `probe_agq.py` → `probe_agq_out.json`. My own regex parser of `contacts.log`, pyulog rates and heights, and
    checkpoint facts. It is written from the prereg text and does not import the unit's code.
  - `probe_s3a.py` → `probe_s3a_out.json`. Recomputes every comparison from `result.npz` and the pinned G8 inputs.
  - `probe_traj.py` → `probe_traj_out.json`. Descriptive comparison of the new members with the dev08 members.
  - `reading_rerun.json` and `analyzer_rerun_stdout.log`. The fixed analyzer (`--analyze`, px4 Python) re-run into
    this directory. Its output is **byte-identical** to `analysis_v1/reading.json`.

## Verdicts

| Unit | Verdict | BLOCKER | MAJOR | MINOR | LOW |
| --- | --- | ---: | ---: | ---: | ---: |
| AGQ geometry-v1 attempt01 | **PASS_WITH_LIMITS** | 0 | 0 | 2 (A-m1, A-m2) | 4 (A-l1–A-l4) |
| S3a-P1 gpu_run01 | **PASS_WITH_LIMITS** | 0 | 0 | 1 (S-m1) | 2 (S-l1, S-l2) |

For both units the execution, the integrity checks and the fixed reading are correct and reproduce independently.
The limits are the preregistered ones: one boot per member, and one GPU/process. The findings concern draft report
wording and two envelope claims that no record binds. None changes a reading.

## 1. Authorization and envelope

| Check | AGQ geometry-v1 | S3a-P1 |
| --- | --- | --- |
| Approval scope = code `SCOPE` | **PASS.** Exact string equality with `SCOPE` in `tools/sitl_agq_geometry_diag.py:43` and `tools/sitl_agq_geometry_launch.py:28`; `skip_max_s` 0.15 (float) | **PASS.** Exact equality with `tools/sitl_s3a_policy_precision.py:28` |
| Approval commit | **PASS.** `f286e29…` | **PASS.** `f286e29…` |
| Approval file bound | **PASS.** sha256 `cdc7f242…` = `attempt01/attempt.json` `scope.approval.approval_file` | **PASS.** sha256 `94f17608…` = `gpu_run01/attempt.json` |
| Ran at the clean commit | **PASS.** `attempt.json` git: head f286e29, dirty "". `run01` head_after f286e29, dirty_after "". All 34 source hashes in `attempt.json` equal the f286e29 blobs; prereg sha `29a96683…` matches; Policy F sha `f7022139…` matches | **PASS.** `attempt.json` and `result.json` git: head f286e29, dirty "". `aerial_gym_path` is inside the worktree. `cpu_only_v4/tool.sha256` = the f286e29 blob (`94f32e51…`) |
| Kernel faults | **PASS.** 0 → 0 (launcher before/after; attempt start/end); 0 now on the same boot | **PASS.** 0 → 0, `fault_check_error` null |
| Envelope | **PASS.** Command `timeout -s TERM -k 120 1200 nice -n 10 taskset -c 6,7,12,13 …` (`run01/execution_process_record.json`). RLIMIT_AS 17179869184 read from the timeout PID's `/proc/<pid>/limits`. Env `CUDA_VISIBLE_DEVICES=""`, PYTHONNOUSERSITE=1, OMP/MKL=1, PYTHONPATH=worktree. Runner self-record: affinity [6,7,12,13], nice 10 | **LIMITED_VALIDITY.** Recorded by the tool: affinity [6,7,12,13], nice 10, `CUDA_VISIBLE_DEVICES=0`, OMP/MKL=1, PYTHONNOUSERSITE=1. The `timeout -k 30 900` wrapper and the absence of other CUDA jobs are NOT_RECORDED in any file I found (S-m1) |
| Single use | **PASS.** One `run01/`, one `attempt01/`. The launcher refuses an existing attempt directory (`sitl_agq_geometry_launch.py:69`, `:95`) | **PASS.** One `gpu_run01/`. `mkdir(exist_ok=False)` (`sitl_s3a_policy_precision.py:404`) |
| Ordering / concurrency | **PASS.** S3a ran 11:54:08–11:54:13 KST and AGQ 11:54:42–11:57:56 KST, so they did not overlap. Per-member idle checks before each member were ≥ 0.9388 on cores 6/7/12/13 | (same) |

A-l1 / S-l2 (LOW): both approval files carry the identical `recorded_at` `2026-11-06T02:40:03.856760+00:00`, so the
session wrote them; I cannot see the user's message. Authorization is verified to the recorded file, not to the
conversation.

## 2. Integrity (AGQ)

- **Ledger:** **PASS.** 4 lines. The chain root is sha256(`attempt.json`); every `prev_sha256` and `seq` holds;
  `final.json` `last_record_sha256` `b3b3b3f6…` closes it with `recorded` 4. `final.json` status COMPLETE, reason
  null, faults 0 → 0, elapsed 188.5 s (< 850 s budget).
- **Bound files:** **PASS.** For every member, 10 of 10 bound files still match their ledger SHA-256:
  - `logs/contacts.log`, `recording/contacts.jsonl`, `session.json`, `checkpoints.jsonl` and the single ULog;
  - the five `recording/` files.
- **Runtime binding:** **PASS.**
  - The 62 core runtime files are identical across the four members and unchanged on disk today.
  - `model_sdf` = the package SDF (native `f9188e90…` = installed SDF; training-box `1b658c0b…`).
  - `diff` of the two SDFs shows exactly the four `rotor_N_collision` blocks removed; `model.config` is identical.
- **Positive control:** **PASS** in all four members. 250/250 messages with a vehicle–asphalt contact, 0 gaps;
  last message 13.652–13.656 s against PX4 13.704–13.708 s, so fresh within 0.5 s.
- **Contact logs:** **PASS.** 10241, 10251, 10988 and 10520 messages, 0 non-message lines, 0 gaps and 0
  non-increasing steps anywhere in the log, no partial tail. Each log starts before gate start − 1 s and ends after
  the final checkpoint + 0.5 s.
- **Per-member status:** **PASS.** All four members are VALID:
  - observation checks equal rows (46/46/76/57), and commands equal sends (45/45/75/56);
  - no record, recorder or timing problems;
  - RTF 1.0010–1.0022; compute max 5.48–10.66 ms.
- **Evaluability:** **PASS.** All four members are evaluable under prereg section 6.

## 3. Readings recomputed from raw files

Times are seconds after each member's gate start. Rates are from `vehicle_angular_velocity_groundtruth` (multi_id
0); the ULog span covers [−35.3 s, final + ≥ 0.98 s], with no gap > 50 ms (max 12 ms) and no dropouts in any member.

| Member | Outcome (final s) | First bar contact (part, bar, interval) | Rate > 5 rad/s windows | Peak rate after start | Z0.8 | Err0.5 | Takeoff rise − anchor |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0 env5 native/native | capture (4.520) | none | none | 3.52 at 4.868 | none | none | +0.016 s, ok |
| 1 env5 native/training-box | capture (4.520) | none | none | 2.62 at 0.236 | none | none | +0.020 s, ok |
| 2 env2 candidate/training-box | capture (7.504) | base, 169, [2.768, 2.772]; later base, 109, [8.476, 8.492] (after the final) | [2.776, 3.224] | 12.26 at 2.796 | 3.800 | none | +0.016 s, ok |
| 3 env2 candidate/native | capture (5.616) | rotor_0, 169, [2.764, 2.784] | none | 4.31 at 3.068 | none | none | +0.028 s, ok |

Every value equals the analyzer's `reading.json`. The outcomes follow from the checkpoint rows: the last row's
decision is `capture` at dist 0.360, 0.391, 0.373 and 0.338 m, and the previous rows are ≥ 0.513 m.

**Pair readings: PASS.** Both follow prereg section 6 exactly.

- **env5 (N = 0, X = 1): INCONCLUSIVE_NOT_REPRODUCED.**
  - Not NOT_EVALUABLE: N's rates cover [0, 4.520].
  - Rule 2: there is no rotor–bar contact, so no t_c and no reproduction.
  - N has no rate onset and no Z0.8, so it is not DISTURBANCE_WITHOUT_CONTACT.
- **env2 (N = 3, X = 2): INCONCLUSIVE_NOT_REPRODUCED.**
  - Not NOT_EVALUABLE: N's rates cover [0, 5.616].
  - t_c = 2.764 (rotor_0, bar 169). N has no rate window, so there is no onset in [t_c, t_c + 0.5] and no
    reproduction.
  - N has no onset and no Z0.8, so it is not DISTURBANCE_WITHOUT_CONTACT. Rules 3–7 are not reached.
- **Clock:** all four takeoff checks fall inside [−0.05, +0.3] s, so both pairs read SUPPORTED, as reported.

## 3b. S3a-P1 recomputed from `result.npz` and the pinned inputs

- **Pins:** **PASS.** The four G8 npz and the two receipts hash to the code's pins. `result.npz` sha matches
  `result.json`. The (call, env) grid is complete, each pair once; the recorded actions are finite.
- **CPU actions:** **PASS.** Bitwise identical to `cpu_only_v4/result.npz` for both densities.
- **Comparisons:** **PASS.** My recomputation equals every reported maximum, argmax and count above 1e-4:

| Density | CPU vs rec | GPU_ON vs rec | GPU_OFF vs rec | GPU_OFF vs CPU | GPU_ON vs CPU | GPU_ON vs GPU_OFF | Shifted control |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 70 | 1.7202e-3 [79,119,1] (8453) | 0, bitwise equal | 1.7198e-3 | 2.3544e-6 [22,58,0] (0) | 1.7202e-3 | 1.7198e-3 (8451) | 1.9316 |
| 205 | 1.7297e-3 [28,34,1] (8468) | 0, bitwise equal | 1.7296e-3 | 1.4901e-6 [51,62,1] (0) | 1.7297e-3 | 1.7296e-3 (8467) | 1.9329 |

- **Reading: PASS.**
  - Controls: the positive control is < 1e-2 and the shifted control ≥ 1e-2 for both densities.
  - All arrays are finite. GPU_ON vs recorded is 0 ≤ 1e-5, so the replay is faithful.
  - GPU_OFF vs CPU ≤ 1e-4 for both densities, so both read TF32_EXPLAINS_POLICY_RESIDUAL (prereg §4 rule 4). Both
    agree, so the reading is TF32_EXPLAINS_POLICY_RESIDUAL, as reported. Faults are 0 → 0, so not VOID.
- **Switch read-back: PASS.** After construction and in ON: matmul and cuDNN TF32 True, precision `high`. In OFF:
  False/False, `highest`. `cudnn.benchmark` True throughout.

## 4. Report wording (draft records in the working tree, not yet committed)

The fixed labels are reported with their preregistered names and limits, and nothing is upgraded. Both draft
READMEs keep "RESULTS_REVIEW_PENDING". These items should change before commit.

- **A-m1 (MINOR), `docs/evidence/sitl_2026-11-05/dev08/agq/geometry_v1/README.md:79–120`.** The two
  INCONCLUSIVE_NOT_REPRODUCED readings arise differently, and the draft does not say so.
  - **env5:** native member 0 captured at 4.520 s. That is before dev08 member 33's first rotor-overlap proxy at
    5.728 s (`DIAGNOSIS.md`). The path was > 0.5 m from member 33's from checkpoint 28, and member 33 was still
    3.35 m from the target at checkpoint 45. The env5 encounter was **never reached**, so this reading says nothing
    about the rotor-contact hypothesis there.
  - **env2:** native member 3 did reach the historical encounter:
    - same bar 169; contact at 2.764 s against dev08 27's proxy at 2.736 s;
    - truth path within 0.084–0.117 m of dev08 27 at checkpoints 24–28;
    - but a shallower pass. The box clearance to bar 169 at checkpoint 28 (2.80 s) was 0.311 m, against 0.199 m for
      dev08 27 and 0.185 m for training-box member 2.
  - Suggest stating both facts, so that "not reproduced" is not read as evidence against the rotor-contact
    explanation, and so that the env2 contrast between members is read as confounded by approach depth.
- **A-m2 (MINOR), `geometry_v1/README.md:113`.** "had no rate excursion" should say "no rate above 5 rad/s (peak
  4.31 rad/s at 3.068 s, 0.30 s after the rotor contact)". The threshold reading is correct, but the unqualified
  phrase overstates it.
- **S-m1 (MINOR), `docs/evidence/sitl_2026-11-05/dev08/agq/s3a_p1/README.md:59–61`.** Two statements are not bound
  to any machine-readable record I could find:
  - "`timeout -k 30 900`". `gpu_run01_launch_at.txt` holds only two timestamps, and `result.json` records affinity,
    nice and environment but not the parent process.
  - "No other CUDA job ran; the only other GPU client was the desktop's remote-access program". No
    nvidia-smi/process capture is stored.

  Bind a record, or mark both NOT_RECORDED (AGENTS.md: numbers and claims travel with their source).
- **A-l2 (LOW), `geometry_v1/README.md:73`, `s3a_p1/README.md:57`.** "The user recorded the approval" should read
  "the approval was recorded (by the session) at 11:40 KST".
- **A-l3 (LOW), `geometry_v1/README.md:77`.** "the host was otherwise idle" holds with its measured scope:
  - per-member idle ≥ 0.9388 before and ≥ 0.98 after, on cores 6/7/12/13;
  - the open check's first sample read core 6 at 0.87 and passed on its second sample (`idle_check` tries = 3,
    inherited from dev08);
  - other cores were not measured.

  Suggest stating that scope.
- **A-l4 (LOW), `geometry_v1/README.md:108`.** "along the same horizontal axis (x)": the offsets are x
  +0.0308/+0.0308/+0.0251/+0.0293 m and y +0.0076/+0.0076/−0.0018/−0.0121 m. "Mainly along x" is more accurate.
- **S-l1 (LOW), `s3a_p1/README.md:81`.** "explained by GPU TF32 arithmetic" is the label's meaning and is
  acceptable. Two precisions to add:
  - The factor bundles matmul TF32, cuDNN TF32 and the `float32_matmul_precision` setting, with the cuDNN
    benchmark algorithm choice inside it (prereg §3).
  - "Explained" means within S3a's 1e-4. GPU_OFF still differs from the CPU by up to 2.35e-6, which exceeds the CPU
    batch effect of ≤ 8.6e-7.
- **Not overstated:**
  - "no geometry effect, no rotor-contact causality, no gain ranking, no S3b acceptance and no flight readiness" and
    "the historical dev08 failures are not explained or refuted";
  - the S3a "what does not follow" list;
  - the EXPERIMENTS.md entries.

## 5. Descriptive: new members against dev08 (not a gate)

| Comparison | k0 \|Δp_true\| | k0 max \|Δaction\| | First k with \|Δp\| > 0.1 / 0.3 / 0.5 m | Ends |
| --- | ---: | ---: | --- | --- |
| **0 vs dev08 33** (same scene/gains/model) | 0.0319 m | 0.1105 | 7 / 25 / 28 (2.80 s) | capture 4.52 vs crash (altitude) 7.00 |
| **3 vs dev08 27** (same scene/gains/model) | 0.0321 m | 0.0294 | 5 / 32 / 33 (3.32 s) | capture 5.616 vs crash (bar) 8.32 |
| 1 vs dev08 33 (other model) | 0.0335 m | 0.1205 | 20 / 26 / 29 | capture 4.52 vs crash 7.00 |
| 2 vs dev08 27 (other model) | 0.0264 m | 0.1083 | 31 / 33 / 34 | capture 7.504 vs crash 8.32 |
| Within pair 0 vs 1 | 0.0070 m | 0.0100 | 9 / 42 / never (max 0.383 m) | both capture 4.52 |
| Within pair 3 vs 2 | 0.0171 m | 0.0789 | 6 / 33 / 35 | capture 5.616 vs 7.504 |
| dev08 gain pair 33 vs 32 (reference) | 0.0115 m | 0.0881 | 7 / 13 / 32 | crash 7.00 vs capture 13.0 |
| dev08 gain pair 27 vs 26 (reference) | 0.0420 m | 0.0187 | 4 / 7 / 13 | crash 8.32 vs crash 7.40 |

- Target positions are identical at every common checkpoint (max difference 0 m), and checkpoint elapsed times
  agree within 0.024 s. The divergence is in the vehicle, not in the replayed inputs.
- All four new members start 0.025–0.031 m from dev08 in x, in the same direction, and 0.002–0.012 m in y. New
  gate-start sim times are 35.780–35.788 s, against 35.516 (33) and 35.700 (27).
- **Implication.**
  - With the same scene, gains and collision model, member 0 left dev08 33's path (> 0.5 m) at checkpoint 28. That
    is earlier than dev08's own gain pair 33/32 separated (checkpoint 32). For member 3 against 27 it was
    checkpoint 33.
  - So whatever differs between these boots (start offset, boot timing or the added contact subscriber; not
    separated) moves the path about as much as the historical gain change did.
  - Within-pair geometry differences in a single boot each are therefore not attributable to geometry. This is
    consistent with the prereg's limit "boot and noise repeatability are unverified".
  - The env5 path no longer reaches the historical encounter at all. The env2 path reaches it at a shallower
    clearance (A-m1).
  - A boot-to-boot and recording-on/off repeatability measurement of a fixed member is a prerequisite for any
    further geometry comparison. That is not preregistered and not approved.
