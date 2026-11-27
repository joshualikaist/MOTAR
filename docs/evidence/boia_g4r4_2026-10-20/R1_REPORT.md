# G4-R4 R1: independent current safety evidence, nominal CPU scope

Date2026-10-21KST; request/evidence directory2026-10-20UTC. Original HEAD
8689efcd066d64e7b49cecf011e3bd8675380cdb is preserved, never amended.
The user's R4 instruction now authorizes scoped local commit of the previously
uncommitted R3 research module and this P1 correction. No GPU/Isaac/learning,
production/C02/PolicyF/controller/governor edit, online connection, push or merge.

## Defect, counterexample and actual correction

Old `decision.py::_current_safe` required `option.benefit.key==case.key` despite
safety validity being logically separate. Identical candidate safety flag with
current benefit yielded SAFETY_OVERRIDE; changing only the utility key to an old
decision yielded NO_CERTIFIED_BACKUP. Missing utility also caused AttributeError.
Simply removing the key check would leave boolean-only, stale safety authorization.

The new `safety.py::validate_safety` consumes only SafetyContext, current key,
candidate ID, wait and SafetyEvidence. It never reads utility, paired losses,
benefit key or camera-hypothesis labels. `decision.py::_option_safe`, `_current_safe`
and `_baseline_safe` use it. Boolean `safety_valid`/`baseline_safe` flags only veto;
they cannot manufacture evidence or grant permission. Baseline F receives the same
strict proof validation before KEEP_F, utility ABSTAIN, endgame or WAIT.

| Independent safety record | Required checks |
| --- | --- |
| Env/episode/decision/candidate/wait | Exact current binding; nonempty IDs, finite nonnegative wait, integer current decision. Duplicate candidate/wait IDs refuse. |
| Sensor/boundary/model versions | Exact record/context version and capture-time matches; no substitution of utility domain. |
| Clock/freshness/expiry | Both measured times≤issuance≤NOW≤expiry; finite nonnegative clocks; measured age≤context max_age. |
| Prefix/backup | Explicit true prefix and complete-backup declarations; nonempty independent references. Horizon from issuance≥NOW−issuance+wait+decision_interval. |
| Qualification/provenance | NOMINAL_CPU_MODEL_ONLY plus supported EXACT_SYNTHETIC or NOMINAL_CPU_ORACLE_DECLARATION with matching model/ref. Never physical qualification. |

The prefix declaration must cover the **entire F waiting interval plus candidate
execution prefix**, with complete backup. Existing cumulative wait-interval booleans
remain additional vetoes. Prediction expiry permits evaluation NOW, not queued
future execution; every future option remains advisory and must be recalculated.
No runtime sensor adapter, C02 converter or proof generator was added.

Safety precedence: current independent backup may yield SAFETY_OVERRIDE even with
utility missing, stale, unsupported or unreadable. If F and alternatives lack
current safety evidence, return NO_CERTIFIED_BACKUP with no selected ID. Utility
ABSTAIN cannot authorize unqualified F. Safe F plus unsupported utility may ABSTAIN
and retain the independently supported F ID. Expired decision IDs remain rejected;
state mismatch never returns an unsupported F merely to satisfy an old test.

All records remain immutable/pure CPU identifiers, not actuator commands.
Physical qualification is false. References/statuses are supplied declarations,
not independent verification of real geometry, uncertainty, plant or sensors.

## Tests and negative control

| Required case | Current result |
| --- | --- |
| Fresh safety + stale utility | SAFETY_OVERRIDE |
| Stale safety + current utility | NO_CERTIFIED_BACKUP |
| Candidate/episode/decision mismatch | Refused |
| Missing safety despite true boolean | Refused, no implicit proof |
| Insufficient F proof + valid alternative | SAFETY_OVERRIDE |
| Insufficient F + no qualified alternative | NO_CERTIFIED_BACKUP |
| Utility missing / ABSTAIN with unsafe F | Valid backup only, otherwise no selected ID |
| WAIT/INTERVENE ordinary behavior | Preserved under explicit synthetic proof |
| Candidate permutation | Identical outcomes/selected stable IDs |
| Env state and RNG isolation | PASS; no input/global mutation |
| Version/time/model/provenance/full-prefix and stale F endgame | Refused when unsupported |

New20methods cover those12mandatory families plus extra bindings, age, prefix,
backup, provenance, duplicate/unsafe candidates and AST record isolation. Archived
exact old selector run against the same new assertions: **20run,14failures,2errors**
(two genuine missing-utility AttributeErrors), not a missing-schema/import failure.
The additive new records are ignored by that old selector; the negative replay
replaces only the selector callable, never assertions. Its static-helper check
examines the current helper source and is explicitly not a pre-fix static proof.
See pre_r1_negative.json and run_pre_r1_negative.py.

**Final focused50/50 PASS**,0failures/0errors,.015s. **Full BOIA252/252 PASS**,
0failures/0errors,15.123s. This is the old232methods plus20new methods. All30 prior
research test-method ASTs/assertions are unchanged. Only helper construction and
advance-key fixtures migrated to explicit synthetic safety evidence/context; original
test bytes/SHA archived, not falsely called byte-identical. No thresholds relaxed.
See fixture_migration.json and [R1_validation.json](R1_validation.json) for exact
commands, environment, source/test SHA and exit status.

Independent read-only agent independently passed50tests and adversarial poison-
utility, expiry-at-NOW, expired-F-endgame and invalid-sibling probes. No blocking
defect found within typed nominal CPU scope; see [review](R1_INDEPENDENT_REVIEW.md).
Full public suite **NOT_RERUN**; last C02 receipt remains2941tests/4knownerrors
aggregateFAIL (`motar-6nn.5`), not a result for current R1 source.

## Historical source and evidence preservation

All R3 evidence bytes remain unchanged, including its UNCOMMITTED/as-of-R3 labels.
Its source/test binding is resolved through six archived pre-R1 files. Its WORKLOG
hash reconstructs from Git8689efcd:WORKLOG plus R3_WORKLOG_APPEND.txt. All20 old
manifest entries match via historical_R3_bindings.json. R4 is a new source lineage,
not a retroactive replacement of old source/test receipts. The original R3 runner
uses the older schema; do not run it against R1 and reinterpret incompatibility as
historical failure. Source-delta review is R1_REVIEW_DIFF.patch; new module/test
files are independently read as part of explicit staging review.

C02's15source/13test hashes, main WIP3, stash, PolicyF checkpoint and protected
symlinks remain separately checked. Full Raw payload rehash NOT_RUN; no Raw replay,
calibration, writes or deletion. Initial ownership is in GIT_HYGIENE_REPORT.md.

## Verdict and next gate

P1 coupling corrected **CPU_VERIFIED under the declared nominal contract**.
B03/B04, actual sensor evidence/SE(3)/time calibration, physical C02/plant/controller,
FrozenF continuation, real mission-loss calibration, full deadline and episode
collision-free capture improvement remain unqualified. G5 HOLD. A true boolean or
nonempty evidence reference does not establish physical safety.

The [Master Plan](../../research/occlusion_algorithm_v1/BOIA_RESEARCH_MASTER_PLAN.md)
is the current research entry point; old proposals/reports remain historical.
Next technical unit is the R2 measured-evidence contract design (`motar-3p3.19`),
with B03/B04 and continuation requirements, under new user approval. Do not
automatically implement it, connect online or launch GPU experiments.
