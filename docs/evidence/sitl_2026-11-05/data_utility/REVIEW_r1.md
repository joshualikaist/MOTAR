# Independent review r1: existing-data integrity, coverage and utility audit r1

- Date: 2026-11-10. Reviewer: Claude Code subagent, model Opus 5.5 (`claude-opus-5-5`), fresh context, read-only.
- Under review: `/home/fair/motar_tmp/sitl_next_20261110/data_audit_r1/` (PROTOCOL.md, audit_data.py,
  enrich_metrics.py, metrics.json, ORIGINAL_MANIFEST.json, MANIFEST.json, REPORT.md) and the untracked in-tree copy
  `docs/evidence/sitl_2026-11-05/data_utility/` in MOTAR-camonly (`camonly/dev`, HEAD `504eb4e`).
- Also used: the second worker's independent recomputation in this directory (`independent_audit.py`, `metrics.json`,
  `INPUT_MANIFEST.json`, `run3.log` "DONE failures 0 inputs 5346"), which had no written conclusion.
- Resources: CPU only, `CUDA_VISIBLE_DEVICES=""`, single-thread math, `taskset -c 8,9`, `nice -n 10 ionice -c3`,
  timeout 600 s. No GPU, simulator or network. My spot script and output are only in the session scratchpad
  (`spot.py`, `spot.out`). The only file written here is this review. No `*env*.txt` file under gainrand was read.

## 1. Number tracing and recomputation

**Three independent sources agree on every overlapping number. No disagreement was found.**

| Quantity (REPORT.md) | Author metrics.json | 2nd worker metrics.json | My recompute from raw |
|---|---|---|---|
| Manifests K1E/K2E/E3/S4F1 files, bytes, missing/mismatch | 322/572/490/2021; 306,571,245 / 491,429,712 / 404,581,600 / 581,012,414; 0/0 | identical | n/a (2nd worker rehashed) |
| ORIGINAL_MANIFEST 5,329 bindings | 5,329 | all 5,329 re-hashed: 0 missing, 0 mismatched | — |
| CAM cells / all rows / quota rows | 36/96,263/73,728; 60/156,813/122,880; 48/124,687/98,304 | identical | identical |
| CAM per-arm outcome counts (13 arms) | e.g. F66 13,244/11,331/1 timeout | identical (34 numeric comparisons, 0 failures) | identical |
| First-16 quota rule | asserted 16 per env × 128 | checked by ledger order, 0 failures | checked by `env_episode_index` sort: 0 of 144 cells differ |
| K2s1 = K2s2 = 3,647 crashes | per-cell counts in metrics | same | 12/12 cell pairs differ per cell, so the equal totals are a coincidence of the pooled sum and not duplicated files |
| SITL groups (S3b A/B/C, S3c, S4, F1 4 cells), VALID, invalid rows | as in REPORT tables | identical | identical; S4 invalid = idx34/env37 ground capture LATE 0.160 s, idx57/env20 raw timeout LATE 0.152 s |
| Step / sensor rows | 6,887 / 3,642 / 12,445 / 19,722; widths 27 and 16 | identical; k-column and steps↔sensor PX4-time join verified | identical; header × size consistent, 0 bad |
| ULogs | 462, 1 per episode; 2 truncated F1 tails | 462; same 2 partial records (offsets 9,215,866 / 9,215,879; 169→131, 154→118 B; +1.000 / +1.008 s after terminal); decompressed SHA matches recorded | 1 ULog per episode in all 462 |
| S4 ground vs F1 lit_rendered | n 39, 17 same, κ −0.00350877, 10 crash→capture, 8 capture→crash | identical | identical (chance agreement 17.0769) |
| S3b A vs C | 10/10 | — | 10/10 (all captures) |
| F1 ep004 / ep139 identity | env1 emissive_rendered / env37 lit_rendered | — | confirmed via `index` field |
| ULog rates (~100/50/125/50/5/10 Hz) | `representative_ulog` topics in all 4 families | — | read from metrics, consistent |
| dev08/AGQ 2 smoke + 40 main; 4 geometry members; floor contact | `dev08_AGQ` | — | metrics show asphalt_plane contact rows; labels match `geometry_v1/execution_record.json` |

Values that I checked in their original files because they are **not in metrics.json**. All are correct:
- `navrl_quad_s4.sdf` gives 20 Hz, HFOV 1.51843645 rad and 160×90. Its current SHA `d8bab518…` equals the
  `model_sdf_sha256` pinned in `s4f1/attempt01.json`.
- The world file gives physics at 250 Hz: step 0.004 s, `real_time_update_rate` 250.
- `kroc2027_summary.json` gives 66°×54°, translation [0.1, 0, 0.03] m and range 20 m.
- The KROC rows give `rl_dt_s` 0.1 and `initial.stream` UNPAIRED_STREAM, matching the 2nd worker's 377,763 rows.
- `realveh_receipt.json` gives `NAVRL_DEPTH_MINZ_M` 0.4.
- `review_summary_v3e_r1.json` gives 3,726 leaves and manifests of 322 + 572 = 894 files and 490 files.
- The historical "Attitude was not logged" prose is at `s3c_s4/README.md:200`.
- The F1 κ≈0 finding and the "neither primary reached after Holm" label are in `s4f1/README.md`.

## 2. Wording

- The report is descriptive. It makes no causal, real-flight or state-of-the-art claim.
- NOT_TESTED stays NOT_TESTED for real-sensor transfer. NOT_RECORDED is used for absent streams, with no proxy
  substituted.
- Historical labels are preserved: NOT_EQUIVALENT, INCONCLUSIVE_NOT_REPRODUCED and the F1 Holm result.
- It discloses the target-distance vs pursuer-clearance distinction and the UNPAIRED_STREAM limit.
- The recommendation is supported by the evidence:
  - Outcome integrity and coverage pass, so recollecting the outcomes is not justified.
  - Per-scene outcomes do not repeat (κ≈0). The 2nd worker's `early_trajectory_examples` show the S4 and F1
    initial truth states already differ at step 0 (about 1.8 cm in z) and that the actions differ from k=0.
  - The streams needed for a mechanism (frames, contacts, transport) are NOT_RECORDED, while attitude and controller
    history are recoverable from the ULogs.

## 3. Copy and manifest identity

- `README.md` = `REPORT.md`, SHA `893d9204…`.
- `audit_r1.json` = `metrics.json`, SHA `372c5af1…`.
- `AUDIT_MANIFEST_r1.json` = `MANIFEST.json`, byte-identical, SHA `223c4d78…`.
- All 9 entries in MANIFEST match on SHA and bytes. I checked them, and so did the 2nd worker's `author_artifacts`
  check.
- `provenance_r1.json` hashes match the externals.
- The folder is untracked. The worktree HEAD is `504eb4e`, as recorded.

## Findings

No BLOCKER and no MAJOR findings.

**MINOR-1. Some README numbers have no machine-readable source in the audit record** (AGENTS.md "Numbers travel with
their source").
- Affected: SDF 20 Hz / 1.51843645 rad / 160×90 / 250 Hz; CAM 66°×54°, translation, 20 m, `rl_dt_s` 0.1,
  MinZ 0.4; E3 review 3,726 / 894.
- These values are correct. The model SDF is not even bound in ORIGINAL_MANIFEST; only its hash, through
  `s4f1/attempt01.json`.
- Fix: in an r1a addendum, extract these values into a JSON with source path, key and SHA. Alternatively, cite path
  and key inline. Add a small check that REPORT numbers appear in the JSON.

**MINOR-2. The S4/F1 flat-trace description omits sentinel and non-finite values found by the 2nd worker.**
- `sensor_steps` `target_t` = −1 at k=0: 55/60 S4 and 144/160 F1 episodes.
- `steps` clearance = +inf: 2 F1 rows, in crash episodes.
- `vehicle_local_position` repeats a native timestamp 22 times.
- Why it matters: the report says these traces "can recompute sampled ages", and the sentinels would corrupt naive
  age statistics.
- Fix: add one line naming the sentinels and excluding them from any age or clearance summary.

**MINOR-3. In-tree record completeness before commit.**
- The README says "Independent review is PENDING".
- The in-tree names (`audit_r1.json`, `AUDIT_MANIFEST_r1.json`, `README.md`) differ from the names inside the
  manifest, and no file maps them.
- The scripts and the 1 MB ORIGINAL_MANIFEST exist only externally, bound only by hash.
- Fix:
  - Record this review as `REVIEW_r1.md` in the folder (AGENTS.md "One evidence folder per unit").
  - Update the review status, with reviewer and model.
  - Add the name mapping to `provenance_r1.json`.
  - Copy, or explicitly declare external, `audit_data.py`, `enrich_metrics.py` and `ORIGINAL_MANIFEST.json`.

**LOW-1. Undisclosed earlier run.** `run.log` (DONE bindings 4,996) predates the final `audit_data.py`
(mtime 09:33). That script version is not preserved. `enrich_metrics.py` is also not among PROTOCOL's listed
outputs. Fix: one sentence stating that the first run was superseded by a harness change and that no threshold or
label changed.

**LOW-2. "E3 review … checkpoint bindings" cites the wrong file.** The JSON summary has no checkpoint field. The
support is in `RESULTS_REVIEW_v3e_r1.md` lines 23–24. Fix: cite the Markdown file.

**LOW-3. The S3b A→C 10/10 agreement is uninformative about crash repeatability.** Block A was 122/122 capture. Fix:
say "all 10 C repeats repeated A's capture; A had no crashes".

**LOW-4. The "bounded 12-boot diagnostic" size is not derived from any data.** Fix: label it as a proposal requiring
its own preregistration, power rationale and the user's GPU/simulator authorization.

Overall: integrity, coverage and outcome numbers are fully reproduced by two independent recomputations and my own
spot checks. The remaining items are traceability and disclosure. Fix MINOR-1 to MINOR-3 when committing, or carry
them as open.

VERDICT: PASS_WITH_LIMITS
