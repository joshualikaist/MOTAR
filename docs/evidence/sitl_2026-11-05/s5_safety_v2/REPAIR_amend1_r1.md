# REPAIR_smoke_r1 — S5 v2 smoke01 and S4R-1 v1 smoke01_contact_on_env0 (2026-11-10)

Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, base HEAD `fd753f2`.
CPU only: no PX4, Gazebo, GPU, `--smoke` or `--launch`; raw smoke folders read, never written. Nothing staged or
committed. Awaiting an independent review.

Changed: `tools/sitl_s5_safety.py`, `tests/test_sitl_s5_safety.py`, `tools/sitl_s4r1.py`, `tests/test_sitl_s4r1.py`.
New: `docs/plans/prereg_sitl_s5_safety_v2_amendment1_2026-11-10.md`,
`docs/plans/prereg_sitl_s4r1_v1_amendment1_2026-11-10.md`. The committed preregistrations are untouched.

## Finding 1: `TypeError: Object of type bytearray is not JSON serializable` (both tools)

- **Evidence:** S5 `smoke01/final.json` ABORTED with that reason, 0 of 3 scenarios, no `ep000/trace.json`; S4R-1
  `episodes.jsonl` `RECORDING_WRITE: Object of type bytearray is not JSON serializable`.
- **Diagnosis:** S5 `json_safe` and S4R-1 `jsonable` converted `bytes` only. Every class of pymavlink 2.4.50's
  `dialects.v20.all` was packed and decoded once: no regular `to_dict()` field is a bytearray. The robust-parsing
  pseudo-messages `BAD_DATA` and `UNKNOWN_<msgid>` that `recv_match` returns carry `data` as a bytearray. Failing
  writes: S5 `json.dumps(trace)` in `run_scenario`'s `finally` (v2 l.977, an exception that aborted the unit); S4R-1
  `dump_tap` events (v1 l.518). Which pseudo-message arrived is NOT_RECORDED.
- **Change:** bytes, bytearray and memoryview become `{"bytes_hex": hex}` (lossless). S5 also maps numpy values and
  writes the trace through `trace_text` (`json.dumps(default=json_default)`), with a tagged-repr fallback. S4R-1
  `jsonable` falls back to a tagged repr.
- **Tests:** real pymavlink objects (HEARTBEAT, STATUSTEXT with a non-UTF-8 byte, PARAM_VALUE,
  FILE_TRANSFER_PROTOCOL uint8[251], BAD_DATA, UNKNOWN_54321), decoded from wire bytes by a robust-parsing receiver.
  They pass through S5 `handle_message`, `instrument_link`, `trace_text` and the fake-PX4 live path (N0 stays VALID;
  the ledger recompute agrees), and through S4R-1 `ConnProxy`, `MavProxy` and `finish_episode`/`dump_tap`. They
  round-trip byte for byte. Negative control: raw BAD_DATA `to_dict()` raises the smoke's TypeError.

## Finding 1b: smoke exit status 0 on ABORTED (both tools)

- **Evidence:** `chain.log`: `S5_smoke rc=0`, `S4R1_smoke rc=0`.
- **Change:** `--smoke` and `--launch` exit 0 only on COMPLETE (and, for `--smoke`, when the attempt's smoke gate
  passes). They exit 3 when not COMPLETE and 4 when a COMPLETE smoke is gate-refused. A refusal is still
  `SystemExit("refused: ...")`, exit 1. `__main__` is `sys.exit(main())`.
- **Tests:** the rule itself, then `main` through the real `launch()` with host checks and runner replaced:
  ABORTED → 3, gate-refused → 4, COMPLETE → 0, refused → SystemExit.

## Finding 2: S4R-1 `CONTACT_AUDIT: RECORDER_LOSS` (4 gaps, 6 missing steps) and `RECORDER_ORDER` (3)

- **Diagnosis (read-only, `ep000/logs/contacts.log`):**
  - The log holds 10,723 lines, all parsed, with 0 other lines and no partial tail.
  - All 10,723 simulator times are distinct, and the sorted times are all exactly 4 ms apart: **0 missing**.
  - The window [35.184, 42.684] s holds 1,876 messages. **3 arrive late**, at 35.980-36.000 s, at most **2 steps** late.
  - File order: `976, 988, 984, 980, 992, 996, 36.004, 36.000, 36.008` ms.
  - AGQ compares each message with the previous arrival, so one reordering counts as both a "gap" and a
    "non-increasing" time.
  - Our parser is not the cause: complete lines in file order. The publish rate is not the cause either: every
    interval is 4 ms.
  - The cause is the subscriber, `gz topic -e`, which delivered out of order. Whether gzserver published out of
    order or the transport reordered cannot be separated from this log.
- **Change:** an amended rule, set after seeing the smoke (disclosed). `audit_contacts` keeps AGQ's audit (coverage,
  events, counts) and replaces only its two arrival-order problems, which stay recorded. `arrival_continuity` checks
  the sorted distinct times in the same window. It allows zero missing, duplicate or off-grid steps, and at most
  2 steps (AGQ's existing `INTERVAL_JOIN_NS`) and 1 % late arrivals. The floor-contact positive control and coverage
  are unchanged. Under this rule smoke01's log has no problems.
- **Tests:**
  - Positive: a clean log and the smoke pattern, which reproduces AGQ's exact 4/6/3 messages. The boundaries pass:
    2 steps late, and 4 late arrivals of 426.
  - Negative, each failing: a missing step (also when reordered), a duplicate, an off-grid time, 3 steps late,
    5 late arrivals of 426, coverage short, an empty log and a missing log.
  - Through `finish_episode`, with the real AGQ module.
  - On the real smoke01 log, read-only: 10,723 messages, 1,876 in the window, 0 missing, 3 late, max 2.

## Finding 3: spent smoke folders

- **Change:**
  - New smoke folders: S5 `smoke02`, S4R-1 `smoke02_contact_on_env0`. `attempt01` was never created, so it stays.
  - The attempt gates read the new folders.
  - New SCOPEs say `amendment1` and the new folder; S4R-1's also carries the contact clause. Both are verbatim in the
    amendments.
  - `AMENDMENT` is a bound source and is recorded in `attempt.json`. S5 integrity also requires it.
  - The old approvals (old SCOPE, `ede0a14`) are refused.
- **Tests:**
  - Each new SCOPE is quoted verbatim in its amendment.
  - The old SCOPE is derived exactly, and is found in the S4R-1 prereg and in both amendments.
  - The folders are checked, the old-SCOPE approval is refused, and an S5 smoke without the amendment is refused.

## Results

| Suite (aerialgym, focused command) | Before (HEAD tests) | Now |
| --- | --- | --- |
| `test_sitl_s5_safety.py` | 47 | 54 run, OK, 0 skipped |
| `test_sitl_s4r1.py` | 34 | 42 run, OK, 0 skipped |

`test_experiment_index.py` passes now with the amendments untracked. When committed, they likely need adding to the
SITL family in `docs/EXPERIMENTS.md`, which is outside this repair's edit scope. Full PUBLIC_CPU was not run.

## Open

- An independent review, a commit with a WORKLOG entry and the EXPERIMENTS.md entries, and PUBLIC_CPU.
- A new user approval naming the new SCOPEs and HEAD.
- Not addressed: the S4R-1 `floor_control` rule, which stays strict, so a reorder inside its 250-message window would
  fail closed.
- Not addressed: `--analyze` on a missing attempt directory still exits with a traceback (rc 1).
