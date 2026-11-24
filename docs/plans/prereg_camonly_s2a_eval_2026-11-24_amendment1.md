# CAM-S2a-E v1 amendment 1 (2026-11-24): the cams2 target contacts need a crash-cause code before the result export

**State:** amendment, committed before any further GPU cell. Drafted by a Fable 5.1 subagent (stopped by its usage limit
after the draft and its tests were complete); placed, reviewed and committed under Claude Opus 5.5 at the user's request
("오퍼스로 낮춰서 진행하자", 2026-11-24). Amends
`docs/plans/prereg_camonly_s2a_eval_2026-11-24.md` (committed at `ab03ab0`), which is not edited. Written after the identity
stage of 2026-11-24 read IDENTITY_FAIL and before any further GPU cell. Branch `camonly/dev`.

## What failed

The identity stage (`~/motar_tmp/cams2a_eval_20261124/raw/identity_01/`, commit `ab03ab0`, host faults 1 → 1, kept as recorded)
ran its five cells to the end of their episodes and every cell FAILed at the result export:

```
RuntimeError: NavRL bulk eval speed crash-cause mismatch: 286 != 372
  (navrl_task.py:9048, _validate_eval_outcome_strata, called from _export_bulk_eval_result at :9127)
```

No `result.json` was written (`check_cell`: "missing or unreadable output"), so every cell is FAIL and the stage
IDENTITY_FAIL. The cells did run: 188–229 s each, `[yawclip]` on `YC12k_negative` shows a whole-cell clipped share of 0.5788
(1,280 calls), the ledger and the recorder rows were written. The fault count, boot id and online CPU list were unchanged.

## Why

`navrl_cams2` turns the task's swept test at the 0.40 m contact radius into a collision after the task's own method has
run (navrl_cams2.py:147-151). The task fills `_crash_cause_code` with −1 and then 0..3 for its own crashes
(navrl_task.py:6137-6141); a converted target contact is therefore a crash with code −1. The bulk-eval strata tallies
count only codes 0..3 (navrl_task.py:8514-8520, 8539-8545), and `_validate_eval_outcome_strata` requires the per-stratum
cause sum to equal the crash count (:9045-9050): 286 coded crashes against 372 crashes, the 86 missing ones being the target
contacts of that cell. Training never exported a bulk-eval result, so the defect could not show there; the review of the
trained unit (F6) had noted that the converted contacts keep code −1 and that "any evaluation that uses those recorders must
be corrected", and the preregistration corrected the recorder but not the task's own validator. The GPU smoke that would
have caught this did not exist: the preregistration had no smoke stage.

The identity cells also logged a second checkpoint-contract line outside the preregistered allowlist:
`NavRL CONFIG MISMATCH | NAVRL_P9_SEED: checkpoint trained with 4952.000, running with 45013.000` (navrl_task.py:4498).
It is the per-cell P9 module seed (10 × seed + 3) that the preregistration sets by design, matched across arms. Had the
cells exported, this line would have FAILed them in `receipt_problems`. The preregistration foresaw exactly this ("an
unexpected line there is IDENTITY_FAIL and an amendment must extend the list").

## What changes

1. **A new eval-only runtime wrapper, `aerial_gym/task/navrl_task/navrl_cams2_contactcode.py`** (opt-in,
   `NAVRL_CAMS2_CONTACTCODE=1`), installed by `tools/realveh_entry.py` **after `navrl_camsafe` and before `navrl_cams2`**, so
   that it sits inside cams2's wrapper and runs on the task's own result, where `captured_now` is the swept test at the contact
   radius: exactly the set cams2 converts one layer further out. Per step it (a) exposes that mask as
   `self._cams2_contact_now`; (b) sets `_crash_cause_code[mask] = 0` ("contact", the only code the strata have for a contact);
   (c) when the task's crash diagnostics are on (always in bulk eval, navrl_task.py:1900-1902) adds the same envs to the
   descriptive tallies `_diag["contact"]`, `_diag_steps["contact"]` and `_diag_x_sum`, so that `result.json`'s top-level
   `crash_causes.bar_contact` and the per-stratum causes agree and both read "contact includes target contacts (0.40 m swept)".
   It refuses to install before camsafe, after cams2, or twice. A `[cams2contact]` JSON line every 320 steps carries the
   running counts. `navrl_cams2.py` and `navrl_task.py` stay byte-identical (the runner checks the hashed modules against the
   training receipt).
   **Passivity, every reader checked:** `_crash_cause_code` is read by the strata tallies and their validator
   (navrl_task.py:8514, 8539, 9048), the joint speed telemetry (:5110), the episode forensics (:5135) and the recorder
   (navrl_cams2diag.py); the `_diag*` tallies by the result export (:9132-9134, :9685-9694, :9741), the OOB forensics'
   consistency check on `_diag["oob"]` only (:9188, untouched) and the crash-diagnostic log line inside `_log_progress` (:10012-10021). None of these writes
   rewards, terminations, observations or commands. The code is set after the task has resolved rewards and terminations
   and before cams2 converts the contacts; cams2 reads `captured_now`, not the code. The CPU test shows rewards,
   terminations and flags unchanged by the fix and the validator's arithmetic (coded crashes = crashes) holding with it and
   failing without it, on float32 tensors over the real `navrl_cams2` wrapper.
2. **The recorder `navrl_cams2diag` classifies `target_contact_swept` from the exposed mask**, not from code −1 (which no
   longer occurs), and refuses to install without the contact code. Code −1 on a crash, should it ever appear, reads
   `uncoded`.
3. **The fix is mandatory in every cell, recorder on or off**, `S12k_nodiag` included: the runner exports
   `NAVRL_CAMS2_CONTACTCODE=1` for every arm; the launcher refuses a cell without it; `receipt_problems` requires the install
   record (`contact_code` 0) at the committed module's SHA-256 and a `[cams2contact]` telemetry line; the key joins
   `SNAPSHOT_EXPECTED` with the value `1`; the module joins the committed-file list and the diff allowlist; the realveh entry
   hashes it when on.
4. **MISMATCH allowlist:** `CONFIG MISMATCH | NAVRL_P9_SEED:` is added (the per-cell module seed). The other module seeds
   (`NAVRL_VEH_RAND_SEED`, `NAVRL_DEPTH_SEED`, `NAVRL_CAMS2_DELAY_SEED`, `NAVRL_GAIN_RAND_SEED`) did not produce a line in
   identity_01 and stay outside the list; the smoke confirms.
5. **A mandatory GPU smoke before the identity stage** (`tools/cams2a_eval_run.py smoke`): one `S12k_smoke` cell (ep12000,
   205 bars, seed 4501, **quota 1**, the recorder and the contact code on, cap 0.5 h), run through every changed runner path
   (launcher, entry install order, cell checks, receipt checks). SMOKE_OK needs the cell OK, a `result.json` and at least one
   recorder row; it writes `smoke_report.json` and is **never counted** in any rate projection, identity comparison or
   result. The identity and eval stages refuse to start unless exactly one SMOKE_OK report on the same commit exists in the
   same parent folder. The lesson recorded here: every changed runner path is smoked on the GPU before the preregistration
   that depends on it is committed; a CPU test suite cannot exercise the task's own export validators.
6. **Output layout:** `raw/identity_01/` stays as recorded (IDENTITY_FAIL). The rerun writes under a new parent,
   `~/motar_tmp/cams2a_eval_20261124/raw_a1/{smoke,identity_01,eval_01}`, because the eval stage globs
   `out_root.parent/*/identity_report.json` and needs exactly one; the identity stage now also refuses a parent that already
   holds an identity report.
7. **Analysis:** only the label of the copied `result.json` `crash_causes` changes
   (`crash_causes_result_json_contact_includes_target_contacts_0p40m_swept`); the controls were regenerated with the amended
   script and are identical to r3 (`~/motar_tmp/cams2a_eval_20261124/controls_camk1e_9blocks_a1.json`,
   `analysis_script_sha256 = 7b54007b14072e6bcd380b1863bc7e5ed092f06ee77147ca9d9b655f34209c85`; the committed
   `tools/cams2a_eval_analyse.py` must have this SHA). The launcher copy changes (one required key), so its SHA is re-pinned
   in the runner at the commit (`LAUNCHER_SHA` is a placeholder until then and refuses).

## Why the gates, labels and thresholds are unchanged

- The fix changes a diagnostic code and descriptive tallies after rewards, terminations and the cams2 conversion are
  settled; the outcomes the ledger records (capture / crash / timeout per episode), the rates, the blocks, the intervals,
  the gate rules L and Y, the minimum binding share, the bands and the controls are untouched. The identity stage's
  determinism and passivity controls (`S12k_repeat`, `S12k_nodiag`) and the CPU passivity test cover the fix: with the fix in
  both arms of each identity comparison, any effect of it on an outcome would also show as a changed result between the
  recorder-off and recorder-on cells only if the recorder had one; the fix's own passivity rests on the reader inventory
  above and on the CPU test, which checks the validator arithmetic and the flags on the fixed path (a direct with/without
comparison of rewards and terminations is not part of the test; review a1 P3-2).
- The P9 seed line is a consequence of the preregistered per-cell module seeds, not a change of the environment.
- The smoke adds one short GPU cell (quota 1, roughly 2–3 min) and is excluded from every number.
- The evaluated environment, arms, densities, seeds, quota, stages A/B/C, caps and the rate gate are as preregistered. One
  observation from identity_01 is recorded for the time estimate only: the five cells took 188–229 s each (3.1–3.8 min),
  faster than the preregistration's 5.5–7.5 min estimate; the rate gate, not the estimate, governs.

## Authorization

The GPU rerun (smoke, identity, eval under `raw_a1/`) needs a new approval file naming the amended `SCOPE` (regenerated with
`python tools/cams2a_eval_run.py scope`; its text now names the amendment) and HEAD. The user's original approval words
("평가 fable 5.2로 진행해줘", 2026-11-24) are recorded in the preregistration; this amendment does not extend their scope on
its own.

## Tests (CPU, isolation, cores 6,7,12,13, no CUDA, 2026-11-24)

`~/motar_tmp/cams2a_eval_20261124/draft_a1/tests_a1.log`: 58 OK, none skipped on this host — `test_cams2_contactcode` 8 (new:
settings, install order with positive control, the validator's arithmetic failing without the fix (2 coded of 4 crashes, the
identity_01 failure mode) and holding with it (4 of 4) on float32 tensors over the real cams2 wrapper, the recorder classifying
from the mask and refusing without the fix, the recorder-off path, telemetry, the entry hook and the install order in the
entry source), `test_cams2diag` 11 (fixtures install the fix; `classify` from the mask), `test_yawclip` 11 (order test
includes the fix), `test_cams2a_eval` 28 (every cell's contact-code record and telemetry required, the exact identity_01 P9
line accepted and a non-listed seed line refused, smoke constants and the quota parameter). The full PUBLIC_CPU suite must be
run after the files are placed in the tree; then this amendment and the package are committed; then the GPU smoke, the identity
stage and the eval run on that same commit (the runner requires the commit and binds every stage to HEAD; review a1 P2-2).
SMOKE_OK additionally requires the fix to have acted: >= 1 recorder row of class `target_contact_swept`, >= 1 capture row
and a `[cams2contact]` running count > 0 (review a1 P2-1). If the smoke reads SMOKE_FAIL or VOID_HOST_FAULT, the identity
stage is not started; the smoke folder is kept as recorded and any repair needs a further dated amendment.

## Review a1

Claude Opus 5.5 (fresh context, read-only), `~/motar_tmp/cams2a_eval_20261124/review_a1/REVIEW_a1.md`: **GO_WITH_FIXES**,
no P0/P1. Q2 (other export-time validators the cams2 conversion could trip): none found on the whole export path.
P2-1 (smoke must show the fix acting) and P2-2 (order: commit, then smoke/identity/eval on that commit) fixed above;
P2-3 (keep the identity_01 approval record) done outside the repository (`execution_approval_ab03ab0.json` with its
SHA-256); P2-4 (WORKLOG entry) in the same commit; P3-1 and P3-2 wording fixed; P3-5 smoke report quota fixed. Open,
descriptive only: P3-3 (the smoke covers one arm; the others are first exercised in identity / stage B), P3-4 (result.json
contact counts now include target contacts at the top level and in `speed_governor.contact.mean_step`, while
`speed_governor.contact.count` and the bar-probe count stay bar-only), P3-6 (`raw_a1/` is a convention, `--out` unchecked).
