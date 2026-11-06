# AGQ geometry-v1 — recording and four-member launcher, CPU preparation, 2026-11-06

**Status:** CPU implementation and independent reviews only. **Measurement NOT_RUN; live behaviour NOT_VERIFIED.**
No PX4, Gazebo, SITL or GPU process was started. Execution needs the user's scoped approval bound to the clean
commit that adds this folder.

Read the [preregistration](../../../../../plans/prereg_sitl_agq_geometry_v1_2026-11-06.md) first. It fixes the
members, the recording, the stop rules and the pair reading. The parent is the
[follow-up proposal](../../../../../plans/sitl_agq_followup_v1_2026-11-06.md).

## What was built

- `tools/sitl_agq_geometry_diag.py` contains three parts.
  - **Recording**, injected into the unchanged dev08 `run_episode`:
    - the runner's MAVLink Inbox;
    - timed `*_send` calls;
    - checkpoint snapshots and the truth origin;
    - an owned Gazebo contact subscriber, `stdbuf -oL gz topic -u -e /gazebo/default/physics/contacts`, with a
      pre-takeoff floor-contact positive control and a 4 ms continuity audit.
  - **The guarded unit:** four fixed members, the stop rules, and the envelope guard on `--execute`.
  - **The fixed reading:** `--analyze`.
- `tools/sitl_agq_geometry_launch.py` launches the unit once:
  `timeout -s TERM -k 120 1200`, cores 6/7/12/13, `nice -n 10`, a 16 GiB address-space cap, CUDA hidden and one
  compute thread.
- `tests/test_sitl_agq_geometry_diag.py` runs end to end with the real owned session, fake simulator processes, the
  dev08 fake PX4 and a fake contact subscriber.

## CPU checks (external unit `~/motar_tmp/agq_geometry_diag_20261106/`)

| Check | Result | Record |
| --- | --- | --- |
| `stdbuf -oL` with a C++ `std::cout << "\n"` writer killed by SIGTERM | 0 of 50 lines kept without it, 50 of 50 with it | `stdbuf_check/result.txt` |
| Dry plan and binding with the real PX4 tree, four members | 60 identical runtime files; only `model_sdf` differs | `dry_binding_v1/out/dry_binding.json` |
| Mutant kill check v4 (39 mutants, r3 bytes) | all killed, control passes | `mutants_v4/results.json` |
| Targeted mutants v5 (10, r4 bytes) and v6 (4, after the r4 repairs) | v5: 2 survivors (`box_checkpoints_only`, `nonfinite_clearance_kept`), each then pinned by a test; v6: all 4 killed, including both | `mutants_v5/`, `mutants_v6/` |
| Focused tests and PUBLIC_CPU | see [`validation.json`](validation.json) | `logs/`, `public_cpu_v2/` |

Numbers in this table are from the bound records in [`validation.json`](validation.json).
- The final post-r5 edit (two reported reference values) was not mutation-tested.
- The dry binding ran the plan code before review.
- The later `member_deps` changes add the stdbuf files to the binding and the per-member file hashes to the record.

## Reviews (Claude Opus 5.5, fresh context each round)

- [r1](SOURCE_REVIEW_r1.md): FAIL, 4 MAJOR. The pair reading was favourable by construction in three ways. The clock
  check anchored on a post-crash touchdown. `analyze()` read invalidated members. Two tests depended on the cores
  they ran on.
- [r2](SOURCE_REVIEW_r2.md): FAIL, 2 MAJOR.
  - The numpy-typed ULog series broke the clock check and the JSON output.
  - The box member's window was not required to be covered by recorded data.
- [r3](SOURCE_REVIEW_r3.md): FAIL, 1 MAJOR. The exposure reference depended on where the contact fell between 10 Hz
  checkpoints.
- [r4](SOURCE_REVIEW_r4.md): PASS_WITH_LIMITS for both this unit and S3a-P1. It found two false preregistration
  sentences, which were corrected before commit:
  - the reference could still be enlarged by a stop at the contact;
  - the floor-impact sentence.
- [r5](SOURCE_REVIEW_r5.md): PASS_WITH_LIMITS, no BLOCKER or MAJOR. MINOR R5-m1: the exposure reference errs
  towards INCONCLUSIVE when N slows down or turns away just before the contact. This, and LOW R5-l1 (the truth
  position may be up to 30 ms old), were disclosed in the preregistration's §6.5 and §8 after r5. The two candidate
  reference values were added to the output, and a test was renamed (R5-l3).
  - These post-r5 edits were covered by the focused tests and the final PUBLIC_CPU run, but were not independently
    re-reviewed.
  - The reviewers' surviving mutants are listed in r4 and r5.
  - Open LOWs remain as stated there.

Every finding and its repair is listed in the review files. The repaired rules are in the preregistration's
section 6.
