# Existing-data integrity and utility audit, r1

Fixed before computation on 2026-11-10. Post hoc descriptive audit only. No thresholds are fitted, no gate is relabeled, and no causal or sensor-transfer claim is tested. Existing preregistrations define primary rows, exclusions and labels.

Scope: CAM-K1-E, CAM-K2-E, CAM-K2-E3; SITL S3b formal, S3c, S4, S4F1; dev08/AGQ where needed to determine reconstruction and repeatability limitations. Read originals, compact records and manifests; inventory raw schemas and counts. Reuse the recent independent E3 checksum audit explicitly, while directly checking pertinent originals. Preserve all historical raw, negative outcomes, invalid/late records and exclusions. Missing fields are NOT_RECORDED only after checking available schemas, logs, ledgers and ULog inventory; reconstruction possibilities are distinguished from directly recorded quantities.

Report integrity (missing/mismatched files, schema/count checks, duplicate identities), coverage (cells, quota ledger rows, validity and lateness), fixed-grid outcome utility, causal-diagnosis limitations and real-sensor-transfer limitations separately. No aggregate quality score. Same-scene outcome repeatability is distinct from valid unpaired/fixed-grid rate estimation and randomized condition comparisons. Report exact denominators only from actual records.

Outputs: REPORT.md, metrics.json, audit_data.py, MANIFEST.json. External writes only. Source HEAD 504eb4e139af2edff254af88a8481a75be4ee0a1, camonly/dev. No simulator/GPU. CPU tasks: CUDA_VISIBLE_DEVICES empty, PYTHONNOUSERSITE=1, OMP/MKL/OPENBLAS=1; cores6,7, nice10, address-space cap8GiB, timeout600s, kernel crash counts before/after. Root clock start00:06Z, wrap04:06Z, finish04:56Z, deadline05:06Z; never reset/close by worker.
