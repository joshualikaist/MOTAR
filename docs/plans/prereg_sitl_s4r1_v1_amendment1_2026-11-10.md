# S4R-1 v1, amendment 1: lossless bytes, a stated contact-log arrival-order tolerance, a gateable exit status and a new smoke folder (2026-11-10; NOT_RUN)

- **Amends:** `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md` (committed at `ede0a14`, immutable). That
  preregistration stays in force in every point this file does not change.
- **Date:** 2026-11-10, written after the instrumentation smoke `smoke01_contact_on_env0` ABORTED and before any rerun.
- **Status:** NOT_RUN. Source and tests repaired on CPU only; no PX4, Gazebo, GPU, `--smoke` or `--launch` was run for
  this amendment. The contact-continuity tolerance in §3.2 was **set after seeing the smoke's contact log** (disclosed
  here); it was chosen before any attempt01 boot exists. Needs an independent review, a commit, PUBLIC_CPU and a new
  scoped approval naming the new `SCOPE` and HEAD before any live run.
- **Revised** (still uncommitted, before any run) after the independent review `REVIEW_amend1_r1` (Claude Opus 5.5,
  PASS_WITH_LIMITS for this amendment): §3.2's tolerance wording corrected (MINOR-1), tagged-repr values counted and
  warned (MINOR-2, §3.1), a v1-approval refusal test (L1), the "1,876" code comment (L2).

## 1. Reason: smoke01_contact_on_env0 ABORTED

Recorded in `~/motar_tmp/s4r1_20261110/smoke01_contact_on_env0/` (`final.json`, `episodes.jsonl`; raw evidence,
unchanged) and `~/motar_tmp/sitl_runs_20261110/s4r1_smoke.log`, at HEAD `ede0a14`:

```
{"status": "ABORTED", "reason": "first episode invalid (HARNESS_INVALID): the harness does not work", ...}
recording_problems:
  CONTACT_AUDIT: RECORDER_LOSS: 4 simulator-time gaps in the reading window (4 gaps, 6 missing steps in the log)
  CONTACT_AUDIT: RECORDER_ORDER: 3 non-increasing simulator times in the reading window
  RECORDING_WRITE: Object of type bytearray is not JSON serializable
```

The one member (env 0, contact ON, 61 control steps, 48.6 s wall, Isaac reference outcome capture, LATE at most
0.084 s) read HARNESS_INVALID. `chain.log` has `S4R1_smoke rc=0`: the tool exited 0 on ABORTED. `--launch` refused
(instrumentation smoke invalid, `rc=1`); `--analyze` crashed on the missing `attempt01/` (`rc=1`). No result exists.

## 2. Diagnosis

### 2.1 bytearray (`RECORDING_WRITE`)

v1's `jsonable` (v1 line 88) converted `bytes` only. `ConnProxy.recv_match` records `m.to_dict()` for every message,
and `dump_tap` then failed at `events.jsonl.gz`. Measured against the installed pymavlink 2.4.50
(`pymavlink.dialects.v20.all`, reached by the aerialgym env through `~/motar_tmp/sitl_plan_20261104/pylink`): no
regular message field of `to_dict()` is a bytearray (every class of the dialect packed and decoded once); the
robust-parsing pseudo-messages that `recv_match` also returns, `BAD_DATA` and `UNKNOWN_<msgid>`, carry `data` as a
bytearray. Which one arrived is NOT_RECORDED, because the event file was never written.

### 2.2 Contact audit (`RECORDER_LOSS`, `RECORDER_ORDER`): reordered arrivals, no loss

Read-only on `ep000/logs/contacts.log` (13,682,495 bytes, 10,723 lines, `steps.npy` control span 36.184-42.184 s,
reading window [35.184, 42.684] s):

| Measure | Value |
| --- | --- |
| lines / parsed Contacts messages / other lines / partial tail | 10,723 / 10,723 / 0 / no |
| distinct simulator times / span | 10,723 / 0.372-43.260 s = exactly (span / 4 ms) + 1 |
| sorted distinct times: intervals other than 4 ms | **0** (no missing, duplicate or off-grid step anywhere) |
| messages in the reading window | 1,876 |
| late arrivals (time below the largest already received) | 3, all in the window, at 35.980-36.000 s |
| largest lateness | 2 steps (8 ms simulator time) |
| AGQ's arrival-order counts | 4 "gaps" (6 "missing steps") and 3 "non-increasing" |

The file order around 35.98 s is `...976, 988, 984, 980, 992, 996, 36.004, 36.000, 36.008...` ms: every one of the
"6 missing steps" is present a few lines later. AGQ's `contact_audit` compares each message with the previous
*arrival*, so one reordering is counted twice, once as a gap and once as a non-increasing time.

- **Not our parser:** each line is one complete `ShortDebugString` message (0 parse failures, no partial lines, no
  multi-message blocks); `_lines` reads complete lines in file order; the subscriber is
  `stdbuf -oL gz topic -u -e /gazebo/default/physics/contacts`, one process writing line-buffered to one file, so the
  file order is the subscriber's delivery order.
- **Not the expected rate:** the 4 ms step (`empty.world` max_step_size 0.004) is exactly the publish interval (every
  distinct interval is 4 ms).
- **The subscriber's behaviour:** Gazebo transport delivered three messages out of order to `gz topic -e` during the
  last 0.2 s of the hover before control started (whether gzserver published them out of order or the transport's
  asynchronous callbacks reordered them is not separable from this log; both are outside our code). It is inherent to
  the subscriber, so the rule is amended (§3.2) rather than the code "fixed".

## 3. Changes (line numbers are in the amended files)

### 3.1 Recording, exit status, folders

| File:line | Change | Why |
| --- | --- | --- |
| `tools/sitl_s4r1.py:51` | `AMENDMENT` = this file's path; added to `OWN_SOURCES` (`:88`), so its bytes are bound by the launch receipt; `attempt.json` records `'amendment'` (`:1172`) | the amendment is part of the executed protocol |
| `tools/sitl_s4r1.py:54` | `SMOKE_DIR` = `~/motar_tmp/s4r1_20261110/smoke02_contact_on_env0` (was `smoke01_contact_on_env0`); `ATTEMPT_DIR` stays `attempt01`, never created | smoke01 is spent evidence; the attempt gate (`instrumentation_smoke_problems(SMOKE_DIR)`, `S3C.smoke_problems(SMOKE_DIR)`) reads the new folder |
| `tools/sitl_s4r1.py:69-87` | `CONTACT_LATE_MAX_STEPS`, `CONTACT_LATE_MAX_FRACTION`, `CONTACT_SCOPE_CLAUSE`, `SCOPE_V1_SMOKE`/`SCOPE_SMOKE`; `SCOPE` changed (verbatim in §4) | it names the new smoke folder, the amendment and the continuity tolerance |
| `tools/sitl_s4r1.py:107-126` | `jsonable` maps `bytes`, `bytearray` and `memoryview` to `{'bytes_hex': '<hex>'}` (the tag v1 used for `bytes`); a type json cannot encode becomes `{'unserializable_type', 'repr'}` instead of failing the whole recording | lossless hex; the Python container type is not kept |
| `tools/sitl_s4r1.py:129-142, 254, 616-626, 628-668, 938-962, 1577` | review r1 MINOR-2: `tagged_types` counts every `{'unserializable_type', 'repr'}` value in what `write_json` and `dump_tap` write (events, packets, steps, summary, timeline, contacts) and in the loaded ULog extraction; `finish_episode` records `unserializable_values` (`count`, `types`; occurrences, so a value copied into two files counts twice) and, when the count is nonzero, `recording_warnings` (`UNSERIALIZABLE_VALUES n (types): kept as tagged repr`); `--analyze` lists the warnings per boot | the tagged fallback no longer fails open silently; it is a recorded warning, not a validity problem (the value's repr is kept) |
| `tools/sitl_s4r1.py:1133-1182, 1184-1199` | `launch` returns `finished_exit_code(out_dir, smoke)`: 0 only when `final.json` is COMPLETE (and, for `--smoke`, `instrumentation_smoke_problems` of the new smoke is empty, the gate attempt01 applies), 3 when not COMPLETE or absent, 4 for a COMPLETE smoke that the gate refuses (problems printed) | smoke01 exited 0 on ABORTED |
| `tools/sitl_s4r1.py:1593-1620` | `main(argv=None)` returns the status; `__main__` is `sys.exit(main())`. A refusal stays `SystemExit('refused: ...')`, exit status 1 | an outer chain can gate on `rc != 0` |
| docstring `:14-21` | amendment summary | — |

### 3.2 Contact-log continuity rule (replaces the continuity part of prereg §4 "Contacts" and §5 "ON: ... missing steps or non-increasing times in the window")

| File:line | Change |
| --- | --- |
| `tools/sitl_s4r1.py:387-448` | `arrival_continuity(times_ns, start, final)`: over AGQ's unchanged reading window [first control step - 1.0 s, last + 0.5 s], **loss is judged on the sorted distinct simulator times**; duplicates, off-grid intervals and late arrivals are counted apart |
| `tools/sitl_s4r1.py:456-472` | `audit_contacts(api, path, start, final)`: runs AGQ's `contact_audit` unchanged (coverage, floor and bar events, all its counts), removes only its two arrival-order continuity problems (`RECORDER_LOSS: ... simulator-time gaps in the reading window ...`, `RECORDER_ORDER: ...`) and adds `arrival_continuity`'s; the removed AGQ strings and the new counts stay in `repeatability/contacts.json` (`agq_continuity_problems_replaced`, `continuity_amendment1`) |
| `tools/sitl_s4r1.py:927` | `finish_episode` calls `audit_contacts` instead of `tap.contact_api.contact_audit` |

The member is recording-invalid (`CONTACT_AUDIT: ...`, as before) on any of:

- `RECORDER_LOSS`: at least one missing simulator step among the sorted distinct times in the window (**zero
  tolerance, unchanged in intent**: a step that never arrived is still loss, whatever the order);
- `RECORDER_DUPLICATE`: a repeated simulator time in the window (zero tolerance; new, stricter);
- `RECORDER_GRID`: an interval between sorted distinct times that is not a whole number of 4 ms steps within AGQ's
  1 µs tolerance (zero tolerance; new, stricter);
- `RECORDER_ORDER`: an arrival in the window more than **2 steps** late (more than 8 ms + 1 µs below the largest time
  already received), or more than **1 %** of the window's messages arriving late;
- unchanged from AGQ: `RECORDER_LOSS: contact log missing`, `RECORDER_LOSS: no contact messages`,
  `RECORDER_COVERAGE` (first message after start - 1.0 s, last before final + 0.5 s), `no control steps`, and an audit
  exception;
- unchanged and still a validity requirement: the pre-take-off floor-contact positive control (`floor_control`:
  250 contiguous messages, at least 125 with a vehicle-floor contact, current within 0.5 s). Its own gap rule is not
  amended; a reordering inside its 250-message window would still fail the boot (fail-closed; in smoke01 it passed:
  250 of 250 floor messages, 0 gaps, 12.916-13.912 s).

**Tolerance and its justification (set after seeing smoke01).** smoke01 measured 3 late arrivals of 1,876 window
messages (0.16 %), at most 2 steps late, and 0 missing, duplicate or off-grid steps. The 2-step bound is **set at the
observed maximum (no margin)**: it equals the smoke's largest lateness, so a later 3-step reordering fails closed. Its
value coincides with AGQ's pre-existing contact-interval join width (`INTERVAL_JOIN_NS = 2 * STEP_NS`), but that
coincidence does not make a late message harmless to the descriptive contact intervals. AGQ's `contact_audit` builds
each bar/other contact interval in **arrival order** (it compares each message with the previous *arrival* of the same
pair, `_last_ns`), so a message at most 2 steps late can still open a new interval in the descriptive `bar_events` /
`other_events` (for example the arrival order 988, 984, 980, then 992 ms leaves 12 ms, 3 steps, between 980 and 992),
and a late first contact can leave an interval whose start is after its end, or an onset one step late. These
intervals are descriptive only: `bar_events` and `other_events` are written to `repeatability/contacts.json` and are
not read by the analysis, any validity rule or the reading. The 1 % bound is about 6 times the smoke's rate (at most
18 late arrivals in a ~1,876-message window). The contact data are the OFF/ON treatment and a positive control only,
not a primary reading, so ordering within 8 ms of simulator time does not touch any analysed quantity; the counts of
late arrivals and their lateness are reported descriptively for every ON boot.

## 4. SCOPE

The execution approval must quote the new `SCOPE` exactly. v1's `SCOPE` (quoted in the preregistration's §8):

```
S4R-1 v1: one fixed instrumentation smoke smoke01_contact_on_env0 (env0, contact subscriber ON, not a result, 600s outer, a short boot) then attempt01: 12 fresh Gazebo/PX4 boots, G8 r2 first two eligible 70-bar scenes env0/env1, 3 repeats per scene with world-contact subscriber OFF/ON, fixed alternating order; S4F1 lit_rendered ground arm, Frozen Policy F, navrl_quad_s4, body-fixed 87deg RGB/depth/gpu_ray, pinned historical bridge, explicit new Gazebo seed4501; MAVLink recv_match arrivals and sends recorded (wait_heartbeat NOT_RECORDED), all delivered raw frames and actor observations recorded; no retries, stop on first operational/recording invalid member, 180s per boot, 1800s outer; owned-session RSS cap 8GiB, no RLIMIT_AS; DISPLAY :1 with a readable inherited XAUTHORITY; 3GiB maximum output, 4GiB launch free and 1GiB minimum free disk; descriptive only: first recorded divergence per stage and within/between-condition spread, no gate
```

`SCOPE` under this amendment (a test checks that it is quoted here verbatim, that v1's is quoted in both files, and
that they differ only by "amendment1", the smoke folder and the contact-log clause):

```
S4R-1 v1 amendment1: one fixed instrumentation smoke smoke02_contact_on_env0 (env0, contact subscriber ON, not a result, 600s outer, a short boot) then attempt01: 12 fresh Gazebo/PX4 boots, G8 r2 first two eligible 70-bar scenes env0/env1, 3 repeats per scene with world-contact subscriber OFF/ON, fixed alternating order; S4F1 lit_rendered ground arm, Frozen Policy F, navrl_quad_s4, body-fixed 87deg RGB/depth/gpu_ray, pinned historical bridge, explicit new Gazebo seed4501; MAVLink recv_match arrivals and sends recorded (wait_heartbeat NOT_RECORDED), all delivered raw frames and actor observations recorded; contact log: no missing, duplicate or off-grid simulator step in the reading window, arrivals at most 2 steps late and at most 1% late; no retries, stop on first operational/recording invalid member, 180s per boot, 1800s outer; owned-session RSS cap 8GiB, no RLIMIT_AS; DISPLAY :1 with a readable inherited XAUTHORITY; 3GiB maximum output, 4GiB launch free and 1GiB minimum free disk; descriptive only: first recorded divergence per stage and within/between-condition spread, no gate
```

The existing `execution_approval.json` names v1's `SCOPE` and HEAD `ede0a14`; it is refused. The launch commands in
prereg §8 are unchanged (`launch_command` is the same).

## 5. Tests added or changed (`tests/test_sitl_s4r1.py`)

- `real_mavlink()` / `real_messages()`: real pymavlink objects decoded by a robust-parsing receiver from wire bytes
  (HEARTBEAT, STATUSTEXT with a non-UTF-8 byte, PARAM_VALUE, FILE_TRANSFER_PROTOCOL uint8[251], BAD_DATA,
  UNKNOWN_54321), imported from the dialect module with `sys.path`/`sys.modules` restored; skipped only where pymavlink
  is absent. `build_member` can feed them through the real `ConnProxy` and `MavProxy`.
- `test_real_pymavlink_messages_recorded_losslessly_through_finish_episode`: negative control (raw BAD_DATA
  `to_dict()` raises smoke01's `TypeError`); a member with the six messages and bytearray/memoryview sends has no
  recording problem, `member_problems` is empty, and `events.jsonl.gz` gives back every `to_dict()` and send argument
  exactly (two `bytes_hex` fields).
- `test_jsonable_bytes_like_and_tagged_fallback`.
- `test_unserializable_values_counted_and_warned_through_finish_episode` (review r1 MINOR-2): `tagged_types` on nested
  values (a set and an opaque object counted, bytes and plain values not); a member whose received message carries
  both records `unserializable_values` count 2 and one `recording_warnings` entry, keeps `recording_problems` empty,
  and the count matches the written `events.jsonl.gz`; positive control: a clean member records count 0 and no
  warning.
- `test_v1_approval_is_refused_under_amendment1` (review r1 L1): an approval with v1's SCOPE and commit `ede0a14` is
  refused by `SF.approval_problems` under the new SCOPE (scope and commit); the new SCOPE at HEAD is accepted; the
  recorded `execution_approval.json`, if present, carries v1's SCOPE and is refused (read-only).
- `test_arrival_continuity_positive_and_negative_controls`: clean log and the smoke01 pattern pass (3 late, 2 steps);
  one missing step (also when reordered), a duplicate, an off-grid time, an arrival 3 steps late, and 5 late of 426
  (> 1 %) each fail with their problem; 2 steps late and 4 of 426 pass (boundaries); a step missing outside the
  window is descriptive only.
- `test_audit_contacts_replaces_only_agq_continuity_and_keeps_coverage`: the synthetic smoke01 pattern reproduces AGQ's
  exact smoke01 messages (4 gaps, 6 missing, 3 non-increasing), which are replaced and kept in the summary; a real
  missing step, a short log (`RECORDER_COVERAGE`), an empty log and a missing log still fail.
- `test_finish_episode_contact_audit_amended_positive_and_loss`: through `finish_episode` with the real AGQ module.
- `test_smoke01_contact_log_reads_as_reordered_not_lost` (skipped where the raw smoke is absent): the numbers in §2.2,
  read-only.
- `test_exit_status_from_final_record`, `test_main_returns_launch_exit_status`: COMPLETE valid smoke 0, ABORTED 3, no
  final 3, COMPLETE smoke refused by the gate 4, attempt by status; `main(['--smoke'])` ABORTED 3, `main(['--launch'])`
  COMPLETE 0 through the real `launch()` with its host checks and `S3C.launch_unit` replaced.
- `test_prereg_quotes_scope_and_names_xauthority`: binds v1's SCOPE to the immutable prereg and the new SCOPE to this
  amendment (see §4), `AMENDMENT` in `OWN_SOURCES`, the smoke02/attempt01 folders.

## 6. What did not change

The question, the 12-boot schedule and order, scenes, policy, vehicle, cameras, bridge, seed 4501, the recording
contract other than the bytes encoding, the floor-contact positive control, AGQ's window and coverage rules, every
other validity and stop rule, the time, memory and disk bounds, the launch commands, the analysis, the reporting
tolerances and the wording limits. `sitl_agq_geometry_diag.py` is not edited. The recorded smoke01 stays ABORTED in its
folder; it was read, never rewritten. Everything not listed in §3 keeps the v1 preregistration's wording and force.
