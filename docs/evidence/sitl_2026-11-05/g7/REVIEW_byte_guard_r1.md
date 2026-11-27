# G7 raw-ledger identity guard — independent source review r1

Reviewer: Codex sub-agent `/root/g7_guard_review`; model: GPT-6 (Codex; exact serving variant not exposed to this reviewer). Fresh delegated context, author conclusions not supplied.

Scope: read-only source audit of `camonly/dev`, HEAD `3f6f7e540117df25b40dd8ad1cb1f02e67c4cf4c`, with only pending `tools/actionhold_run.py` and untracked `tests/test_actionhold_identity.py`. Read AGENTS.md, actual diff, preregistration, full changed files and relevant underlying runner/hook/analysis paths. No tests, simulator, GPU, or Beads actions performed. This is no scientific-results claim.

Verdict: **PASS_WITH_LIMITS for the proposed byte-identity guard**, subject to the author's required test execution and committed-source preflight before any approved run. No blocking defect found in the pending diff.

- `episodes_byte_equal` compares actual `read_bytes()` values and returns false on OSError. The new conjunct at runner lines 105–110 requires parsed episodes equality, parsed comparable-result equality, AND raw episodes byte equality. A semantic match with byte differences therefore sets HOOK_IDENTITY_FAIL; the immediate break precedes all six treatment cells. The seven-cell PLAN matches the preregistered order.
- The identity comparison is reached only after ordinary cell checks, the native/full ablation receipt checks and hook sidecar checks pass. Sidecar requires patch applied, exact k/d config and a nonzero calls count; treatment cells additionally require held or delayed-zero activity. A failed sidecar stops the loop as INCOMPLETE, and host fault becomes VOID_HOST_FAULT. COMPLETE requires all seven entries.
- The new tests contain a concrete nonempty, byte-equal positive control; a semantically equal but byte-different negative (including an explicit parsed equality assertion); and rejection for both missing, reference-only and cell-only ledgers. They exercise the helper without launching the runner. Existing hook tests supply positive manipulation controls. Test execution is NOT_TESTED by this reviewer.

Non-blocking existing failure-reporting limitation: `G.identity_compare` runs before the new helper and opens the reference result/ledger without catching OSError/JSON errors (`tools/gr_run.py:206`). A missing/unreadable GR-0 reference therefore aborts with an exception before the new helper and before `g7_run_report.json` is written; the helper-level missing-ledger test does not establish graceful end-to-end HOOK_IDENTITY_FAIL recording. This still prevents treatment launches and cannot yield COMPLETE. Similarly, a JSON sidecar of the wrong top-level type would raise on `.get` rather than produce an INCOMPLETE report. These are pre-existing reporting limitations, not acceptance bypasses introduced by this diff.

Reviewed SHA-256:

- tools/actionhold_run.py: `3ba5c379072bf820de6aa7d40a9a90f526367976a53336466d30be449ed239dc`
- tests/test_actionhold_identity.py: `0f6f0a919742f3eebee6e5be4e7cd5e0778bf63c6415386d9bead7011e427619`
- docs/plans/prereg_actionhold_g7_v1.md: `3e5122a66dac2626889e810cc19c93566841d4a24379d04649242d22e91fee86`

Git Traceability: Branch camonly/dev; base/HEAD as above; new commit NONE; changed repository files by reviewer NONE; tests NOT_RUN; initial working tree has exactly the two stated pending paths; remote push NO; main merge NO; raw/checkpoint integrity UNCHANGED (not opened or modified). Only this external review report was created.
