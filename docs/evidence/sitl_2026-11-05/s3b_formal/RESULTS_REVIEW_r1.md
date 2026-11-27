# S3b formal v1 attempt01: results review r1

Reviewer: fresh-context Claude Code subagent, Opus 5.5. Read-only. Probes are in `probes/`, with my own analyzer (the runner's was not imported).

**Verdict: PASS_WITH_LIMITS.** The reading `FEASIBLE_IN_SITL` follows from the raw files under the preregistered rule. There are no blockers. One MAJOR item, a disclosure, must go into README and WORKLOG.

## 1. Authorization and envelope: PASS
- The runner that ran is the committed `HEAD:tools/sitl_s3b_formal.py`. Its SHA-256 `90662692…` equals `attempt.json` and is identical at c22a91b and 0d44972.
- The working-copy `remove_dataman` diff came after the run and was not reviewed, as instructed. Both `attempt.json.git.dirty` and `final.git_after.dirty` are `""`.
- `SCOPE` equals the approval scope and `attempt.scope`. The approval file's SHA equals `approval_sha256`, and approval.commit = HEAD = 0d44972.
- **Approval-note claim verified.** All 33 pinned sources match the git blobs at both commits. `git diff c22a91b..0d44972` touches none of them: only S3c/S4 files, WORKLOG and EXPERIMENTS. The runner references no S3c/S4 module, and no S4 plugin is in the runtime binding.
- The external `replay.py` SHA and its tree HEAD match. The 24 inputs match their pins on disk; the F pin is present.
- Environment: CUDA hidden, OMP/MKL 1, affinity 6,7,12,13, nice 10.
- Kernel faults 0→0, the per-row count is 0 throughout, and the journal reads 0 now.
- `final.json`: COMPLETE, 162/162, 2.00 h. Only `attempt01` exists.

## 2. Ledger integrity: PASS
- My schedule, built from the zero-action ledgers (122 and 124 eligible scenes; A, then B with the first 30, then C with the first 10), equals `attempt.schedule` and the ledger rows, in order with index 0..161.
- `isaac_outcome` equals the Isaac ledger in every row.
- I recomputed validity (outcome, the 10 parameter read-backs within 1e-6, late ≤ 0.15 s): 0 mismatches and 162/162 VALID. The largest lateness is 0.024 s.
- Extra read-backs are constant: YAWRATE_P 0.11, YAWRATE_I 0.055, HGT_REF 2, XY_VEL_MAX 3.6.
- The binding core (60 file SHAs, world excluded) is identical in all 162 rows. There are 152 distinct worlds, which is expected because C repeats A.
- Host idle was `ok` in every row (minimum 0.901). Two rows needed a second sample.

## 3. Independent recompute: PASS (every value matches reading.json exactly)

| Block | n valid | SITL cap/crash | Isaac cap/crash | Δcrash pp | Agree |
|---|---|---|---|---|---|
| A | 122 | 122/0 | 121/1 | −0.82 | 121 |
| B | 30 | 27/3 | 26/4 | −3.33 | 25 |
| C | 10 | 10/0 | 10/0 | 0.00 | 10 |

- Block A: Clopper-Pearson upper bound 2.978 %. The worst case equals the observed result, since there are 0 INVALID.
- Block C: 10/10 pairs repeat their block-A outcome.
- The timing and EKF maxima match, including B ekf_xy 1.1226 m and A 0.5365 m.
- Label: COMPLETE, 122 ≥ 110 valid, the tree clean, faults unchanged, and Δ −0.82 ≤ +5. The reading is FEASIBLE_IN_SITL.

## 4. Physical plausibility: PASS, with one finding
Episodes checked: A env 0, 66 and 75, plus all B crashes and disagreements.
- **World file:** the bars match the Isaac layouts within 5e-5 m. The quaternions are identity, so the bars are axis-aligned and the yaw-only world is valid.
- **Capture:** my swept distance reproduces `min_dist_m` exactly. Every capture first drops below 0.5 m on the last step; A env 75 reaches 0.472.
- **Crash geometry:** my box-clearance recompute equals column 20 (max diff 0). B env 10 and env 14 cross 0.14 m on their final step (0.127 and 0.131).
- **Altitude and timing:** the median true z is 0.93-1.02 m. The relative lateness series reproduces `late_max`, and the step spacing is 0.08-0.12 s.

**MAJOR-1 (disclosure, does not change the label): contacts are probably uncounted.**
- In `navrl_quad.sdf` the rotor collision cylinders (radius 0.128 at ±0.13, ±0.13) reach about 0.31 m from the centre. MOTAR's crash rule uses a 0.14 m footprint.
- Five episodes show disturbances that look like contact:
  - true z dips to 0.49 m (A env 82) and 0.275 m (B env 8);
  - EKF horizontal speed reaches 4.7-9.4 m/s, above the 3.6 m/s cap (A env 50, B env 8, B env 18);
  - each happens within about 3 steps of a bar clearance of 0.18-0.29 m.
- **B env 18's "altitude" crash is a fall**:
  - at k = 31 the clearance is 0.182 m;
  - z then drops from 1.03 to 0.06 m while the speed rises to 9 m/s;
  - the EKF error peaks at 1.12 m;
  - its `px4.log` records "Attitude failure (roll)" and "Failsafe".
- The direction of the bias is conservative for the label: extra contact can only add SITL crashes. If both block-A cases counted as crashes, Δ would be +0.82 pp, still below 5. This is inferred, because no contact sensor was used. The prereg already lists "contact cross-check not implemented"; the README does not carry it.

## 5. execution_record.json: PASS
- `reading_attempt01.json` is a byte copy (`cmp`).
- The raw SHAs, `final`, inputs, head, approval, F, scope and all reading blocks equal the raw files.
- `per_scene` (162 rows) matches the ledger and steps.npy with 0 differences.
- The transitions match (A: 121 c→c and 1 crash→cap; B: 24 c→c, 3 crash→cap, 1 crash→crash, 2 cap→crash; C: 10).
- The capture-step medians are 38.5 for A and 45 for B.

## 6. Other findings
- **MINOR-1.** The README names the dev07 overlap for block A (9 scenes) but not for block B (10 scenes, which the prereg names).
- **MINOR-2.** `per_scene.min_clearance_m` is the distance from the vehicle centre to a box. The crash threshold is 0.14 m, not 0. The README must define the field.
- **MINOR-3.** `px4.log` for B env 10, 14 and 16 (16 was a capture) also shows "Attitude failure (roll)". These are probably after termination, in the post-episode land mode, but the logs have no timestamps. Say so or leave them unclaimed.
- **LOW-1.** `binding_core` compares file SHAs only. I checked the environment (only the per-episode `GAZEBO_LOG_PATH` differs) and the plugin count (40) by hand.
- **LOW-2.** Block C repeats only 70-bar captures, so it says nothing about whether crash outcomes repeat.

## Must be disclosed in README and WORKLOG
- The EKF error reaches 1.12 m in B, during the env 18 fall, and 0.54 m in A.
- MAJOR-1: uncounted contacts and the 0.31 m vs 0.14 m geometry.
- The dev07 overlap, A 9 and B 10.
- The target is replayed and does not react (rates, not pairs). MOTAR's own sensors are rendered from truth (no S4). G9 is not gain-matched.
- The S3a NOT_EQUIVALENT entry was the user's decision.
- The approval commit: the user approved S3b at c22a91b, the run was at 0d44972, and the S3b sources are verified identical.
- One seed, first episodes only. No claim at 205 bars or about real flight.
