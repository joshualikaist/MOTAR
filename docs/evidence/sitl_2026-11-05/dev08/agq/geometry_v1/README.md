# AGQ geometry-v1 — recording and four-member launcher, CPU preparation, 2026-11-06

**Status (2026-11-06, after attempt01):** the single approved attempt ran at `f286e29` and completed.
**Both pair readings are INCONCLUSIVE_NOT_REPRODUCED**: no native-model member reproduced a rotor contact followed by
an angular-rate excursion. Results review r1: PASS_WITH_LIMITS. See [Result: attempt01](#result-attempt01-2026-11-06) below.
The preparation record follows unchanged: before attempt01, CPU implementation and independent reviews only,
with no PX4, Gazebo, SITL or GPU process started.

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

## Result: attempt01 (2026-11-06)

- **Approval.** The approval file was written at 11:40 KST. It names the code's SCOPE ("AGQ geometry-v1: 4 fixed
  development members…"), commit `f286e29` and `skip_max_s` 0.15.
  - The user wrote it by running a snippet the session provided. That one process also wrote the S3a-P1
    approval, so both files share one `recorded_at`.
  - The session's own two attempts to write the files were refused by the auto-mode classifier.
  - The independent review verified authorization from these files only (results review r1, LOW A-l1).
- **Execution.** `tools/sitl_agq_geometry_launch.py --launch` ran once, 11:54:42–11:57:56 KST, exit code 0.
  - The unit read COMPLETE: 4 of 4 members recorded, all VALID, with no record, recorder or timing problem.
  - Kernel faults were 0 → 0. The tree stayed clean at `f286e29`.
  - The runner's idle checks on its cores 6/7/12/13 passed before and after every member. No other work ran.
- **Records.**
  - [`execution_record.json`](execution_record.json) holds the bindings, the member events and the descriptive
    comparisons below. `~/motar_tmp/agq_geometry_diag_20261106/make_results.py` wrote it from the raw files,
    run with px4 Python for the ULog peaks.
  - [`reading_attempt01.json`](reading_attempt01.json) is a byte-identical copy of the fixed analyzer's output
    (`--analyze`, px4 Python, `analysis_v1/reading.json`).
  - The raw attempt stays outside Git in `~/motar_tmp/agq_geometry_diag_20261106/attempt01/`.

| Scene (pair) | Fixed reading | Native member: first rotor–bar contact, rate onsets | Clock order |
| --- | --- | --- | --- |
| 205/env5 (0 native, 1 training-box) | INCONCLUSIVE_NOT_REPRODUCED | none, none | SUPPORTED |
| 205/env2 (3 native, 2 training-box) | INCONCLUSIVE_NOT_REPRODUCED | rotor_0 on bar 169 at 2.764 s, none | SUPPORTED |

| Member | Outcome (end, s) | First bar contact (part, bar, s, max force) | Rate > 5 rad/s | Peak rate, rad/s (s) | z < 0.8 m | estimate error > 0.5 m |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 0 env5 native gains, native model | capture (4.520) | none | none | 3.50 (4.448) | none | none |
| 1 env5 native gains, training-box model | capture (4.520) | none | none | 2.62 (0.236) | none | none |
| 2 env2 candidate gains, training-box model | capture (7.504) | base, 169, 2.768, 46.66 N | 2.776 | 12.26 (2.796) | 3.800 | none |
| 3 env2 candidate gains, native model | capture (5.616) | rotor, 169, 2.764, 4.19 N | none | 4.31 (3.068) | none | none |

- Times are seconds after each member's gate start.
- A rate is the largest absolute body rate on any axis in the ULog over the gate window.
- Under the preregistration, a NOT_REPRODUCED pair runs no replacement member and gets no further pair reading.

**The two NOT_REPRODUCED readings mean different things** (descriptive, from `execution_record.json`):
- **env5: the historical encounter was never reached.**
  - Member 0 captured at 4.520 s.
  - In dev08 the same member (`33_main_205bars_env5_native`) first overlapped a bar (168, footprint proxy) at
    5.728 s, according to the committed [dev08 diagnosis](../diagnosis.json).
  - The reading says nothing for or against the rotor-contact explanation of the dev08 env5 failure.
- **env2: the encounter was reached, but the pass was shallower.**
  - Member 3 touched bar 169 with rotor 0 at 2.764 s. dev08 `27_main_205bars_env2_candidate` first overlapped
    bar 169 at 2.736 s.
  - Its box_hit clearance to bar 169 at checkpoint 28 was 0.311 m, against 0.199 m for dev08 27 and 0.185 m for
    training-box member 2. Checkpoint 28 is the closest point of all three paths.
  - Its peak body rate stayed below 5 rad/s: 4.31 rad/s at 3.068 s.
  - A light rotor touch on a shallower pass without an excursion does not test the dev08 failure.
  - The env2 contrast between the two models confounds the geometry with how close each pass was. Member 2's
    base box hit at 0.185 m (46.66 N); a rate above 5 rad/s followed 8 ms later, and the height fell below 0.8 m
    at 3.800 s. It still captured.

**These members did not repeat their historical dev08 counterparts**, which had the same scene inputs, gains and
native collision model (descriptive):
- Member 0 against dev08 33:
  - 0.0319 m apart at the gate start;
  - first actions up to 0.1105 apart in one dimension;
  - more than 0.5 m apart from checkpoint 28.
  - dev08 crashed (altitude); member 0 captured.
- Member 3 against dev08 27: 0.0321 m apart at the gate start and more than 0.5 m apart from checkpoint 33. dev08
  crashed (bar); member 3 captured.
- All four members start 0.025–0.031 m from the dev08 starts, mainly along x, with identical target positions.
  Within this attempt the paired members start 0.007 m (env5) and 0.017 m (env2) apart.
- The cause is not identified. Candidates, not separated:
  - boot or takeoff variation;
  - start timing;
  - the added recording, since the contact subscriber makes Gazebo build contact messages.
- Results review r1 notes that dev08's own gain pair (33 against 32) separated by 0.5 m only at checkpoint 32. What
  differs between boots therefore moves the path about as much as that gain change did.

**What does not follow:**
- no geometry effect;
- no rotor-contact causality, for or against;
- no gain ranking;
- no S3b acceptance;
- no flight readiness.

The historical dev08 failures are neither explained nor refuted. Before any further geometry comparison, the open
question is the repeatability of one fixed member: boot to boot, and with contact recording on and off. It is not
preregistered and not approved.

## Results review r1 (Claude Opus 5.5, fresh context)

[RESULTS_REVIEW_r1.md](RESULTS_REVIEW_r1.md) is a copy of `~/motar_tmp/agq_geometry_diag_20261106/results_review_r1/REVIEW.md`.
The verdict for this unit is **PASS_WITH_LIMITS**, with no BLOCKER or MAJOR.
- **Checks.**
  - Authorization, envelope, ledger chain and file bindings: PASS.
  - The 250/250 floor-contact control and the contact-log continuity in every member: PASS.
  - Re-running the fixed analyzer gave byte-identical output, and an independent parser recomputed every value.
- **Findings, all repaired in this section before commit:**
  - MINOR A-m1: the two NOT_REPRODUCED readings mean different things;
  - MINOR A-m2: member 3's peak rate;
  - LOW A-l2: approval wording;
  - LOW A-l3: idle-check scope;
  - LOW A-l4: "mainly along x".
- LOW A-l1 is answered under *Approval* above.
- The clearances and peak rates the review computed are recomputed by `make_results.py` into
  `execution_record.json`. Member 0's peak is 3.50 rad/s there, against 3.52 rad/s at 4.868 s in the review, which is
  after the 4.520 s end of the gate window used here.
