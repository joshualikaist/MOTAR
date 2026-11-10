# Data-utility audit r1: addendum r1a, after independent review r1 (2026-11-10)

`README.md` is a byte copy of the audit's `REPORT.md`; `AUDIT_MANIFEST_r1.json` binds it by SHA-256, so it is not
edited. Its line "Independent review is **PENDING**" is superseded by this addendum.

- **Review:** Claude Opus 5.5, fresh context, **PASS_WITH_LIMITS** ([`REVIEW_r1.md`](REVIEW_r1.md)).
  - It found no BLOCKER and no MAJOR finding.
  - Every overlapping number agrees across three sources: the author's `audit_r1.json`, a second independent
    recomputation by a Codex worker (5,346 inputs, 0 failures; `~/motar_tmp/sitl_next_20261110/data_audit_review_r1/`),
    and the reviewer's own recount from raw data.
- **MINOR-1, numbers not in `audit_r1.json`:** `sources_addendum_r1a.json` lists each one with its source path, key
  and SHA-256. These are:
  - the S4/F1 sensors (20 Hz, 160 × 90, 1.51843645 rad);
  - the physics rate (250 Hz);
  - the CAM camera (66 × 54°, translation [0.1, 0, 0.03] m, 20 m);
  - `rl_dt_s` 0.1;
  - depth MinZ 0.4 m;
  - the E3 review's 3,726 leaves;
  - 894 = 322 + 572 manifest entries.
- **MINOR-2, sentinels in the SITL flat traces:**
  - `sensor_steps` `target_t` = −1 at k = 0 in 55 / 60 S4 and 144 / 160 F1 episodes;
  - `steps` clearance = +inf in 2 F1 rows, both in crash episodes;
  - `vehicle_local_position` repeats one native timestamp 22 times.

  Any age or clearance summary computed from these traces must exclude them.
- **MINOR-3, record layout:**
  - The scripts (`audit_data.py`, `enrich_metrics.py`) and `PROTOCOL.md` are copied here.
  - `ORIGINAL_MANIFEST.json` (1 MB) is declared external and bound by SHA-256 in `provenance_r1.json`.
  - `provenance_r1.json` maps the file names here to the names inside the manifest.
- **LOW-1, an earlier run:** a first run (`run.log`, 4,996 bindings) used an earlier `audit_data.py`. A harness change
  superseded it before the final run. That version is not preserved. No threshold or reading depends on it.
- **LOW-2, a citation:** the E3 checkpoint bindings are in
  `docs/evidence/camonly_2026-11-04/camk2e/RESULTS_REVIEW_v3e_r1.md`, not in `review_summary_v3e_r1.json`.
- **LOW-3:** the S3b C-group 10 / 10 repeat is trivially all captures. It shows nothing about repeatability.
- **The repeatability diagnostic** that this audit recommends needs its own preregistration and the user's
  authorization. That is S4R-1, `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md`.
