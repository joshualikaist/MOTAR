# CAM-S3-E review r2 — fix diff 8fbf950..e73c573 (camonly/s2b), read-only, fresh context

Reviewer: Claude Opus 5.5 (`claude-opus-5-5`) subagent, 2026-11-27. Text saved by the main session from the reviewer's
final message (summary of every finding; probes in this folder: probe_r2.py, probe_ledger.py, tests_r2.log,
check_r2.json).

**Verdict: GO_WITH_FIXES. No open P1.** Every r1 finding (P2-1..P2-4, P3-1..P3-10) fixed and confirmed by probes and
mutation checks (dropping the capture condition, rewriting SENSOR_MODEL_DEPENDENT, forcing Wilson BELOW_20 each make the
controls fail). Tests 167 OK with -B; 51 OK without -B (P3-5 fixed). Check mode exit 0, preflight_problems empty, only
"no approval file". Scope SHA-256 63fb7f7ef236da3f842cb5b9d0cafa3d4ffab245e9e4acdb6346c9281488408a recomputed. Launchers
pinned (cams3e 94dc5ffe, stage-1 51f949d0); original prereg byte-unchanged; no simulation code changed. Full suite
9 F / 8 E all inside the CAM-S3 baseline. User approval ("오케이 너가 하자고 하는대로 한번 해보자." then "승인", 22:40,
before e73c573 at 22:49) covers the scope; the ledger clause only tightens and needs no re-approval.

## New findings
- **P2-1** ledger bypassed by `--out` outside the unit folder (camref_eval_run.py:1415-1416; camref_stage3_run.py:175-191;
  amendment 71-72). Fix: refuse a stage unless out_root resolves under UNIT_DIR without symlinks at the expected depth;
  test. If launching at e73c573: use --out under ~/motar_tmp/cams3e_20261127, record in WORKLOG, fix before any rerun.
- **P3-1** intervention "running shares" do not cover the cell: camsafe prints at calls 32, 320, then every 3,200 (an
  eval cell's last line is calls=320, ~1/10 of the cell); speedgov counters reset after every print (last window only).
  Reword amendment/docstring; name the window in the output. Unlabelled, no decision depends on them.
- **P3-2** a killed cell with only cell.log counts 0 s (stage3_run.py:172); fallback to mtime-ctime or charge
  CELL_TIMEOUT_S.
- **P3-3** an unreadable report is double-counted (conservative). Ledger scans all of UNIT_DIR incl. reviewer scratch.
- **P3-4** a non-dict report/record raises AttributeError in the ledger (fails closed, opaquely).
- **P3-5** the rerun rule is procedural, not enforced in code.
- **P3-6** check mode does not print the ledger / budget_h.
- **P3-7** pointer wording: IMPROVES + CRASH_DOWN_CAPTURE_DOWN reads SENSOR_MODEL_DEPENDENT though both cut crashes;
  row 64 "one" means "at least one".
- **P3-8** intervention_table and the bootstrap RNG stream still include incomplete-block cells (readings correct).

Amendment and code agree apart from P2-1 and P3-1.
