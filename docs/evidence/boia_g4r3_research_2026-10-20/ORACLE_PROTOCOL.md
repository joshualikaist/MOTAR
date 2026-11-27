# PART C independent arithmetic/decision oracle protocol

Source base: local C02 commit8689efcd066d64e7b49cecf011e3bd8675380cdb.
CPU research only; outputs are candidate identifiers, not actuator commands.
These expectations are fixed before root's final verification of the new module.
Synthetic exactness is an explicit finite-world assumption, not measured sensor
calibration, Frozen F continuation accuracy or capture performance.

## Numerical expectations

1. For two scenes `(L_F,L_g)=(10,9),(1,0)`, paired robust gain is1. The unrelated
   `min(L_F)-max(L_g)=-8` is not the same estimand. A third supported scene(0,2)
   makes the paired worst-case gain−2 and must prevent utility intervention.
2. Exact, equally weighted discrete paths justify MC allowance0 ONLY inside the
   declared exhaustive synthetic law. For sampled paths, require the stated
   bounded/iid assumptions and simultaneous tested-option count; unsupported
   statistical/calibration/continuation evidence must refuse, never become0.
3. Deduct MC,set coverage,calibration,F-continuation mismatch and switching cost
   separately. A gross benefit.4 and switching cost.5 give net−.1. Threshold.1
   cannot accept this option. No actual capture probability is implied.
4. A now-option with supported benefit.4 and safe future option.9 must WAIT/KEEP_F
   rather than copy an immediate-only.4>.1 intervention rule. This holds only
   when the entire F waiting prefix is declared safe and evidence remains valid.
5. A currently observed synthetic target with supported now benefit and no
   supported later opportunity may trigger proactive INTERVENE; a pure actual-loss
   trigger retains F while no loss occurs. This tests a rule difference, not an
   event rate or a mission improvement.
6. Waiting intervals[unsafe,safe] cannot admit the later option. Endpoints do not
   certify the intervening prefix. If current opportunity is empty, report EMPTY;
   do not bridge a later reappearance into a current closing window. A finite grid
   makes no continuous-time guarantee.

## Eighteen required case families

| Case | Expected independent property |
| --- | --- |
| Identical measurements/different worlds | Same permitted packet cannot reveal offline world identity. |
| Foreground/target return | Retain competing explanations and measured first hits; do not carve free space. |
| Incorrect association | Wrong/unsupported identity cannot justify a utility intervention. |
| Missing evidence | Safe F may receive ABSTAIN, not an invented forecast. |
| Unsupported calibration | Missing/domain-incompatible provenance refuses utility. |
| Positive benefit | Exact supported net benefit above threshold can select a safe option. |
| Negative benefit | No utility intervention despite an otherwise safe candidate. |
| Switching reversal | A positive gross gain can become a negative net gain. |
| Early opportunity | Proactive choice can differ from an actual-loss-only rule. |
| Late opportunity | Supported wait choice can differ from immediate-only thresholding. |
| Unsafe wait | A later gain cannot justify an unsafe waiting prefix. |
| Safe F/unsafe alternative | Do not execute the utility winner if it fails safety. |
| Unsafe F/viable backup | SAFETY_OVERRIDE precedes utility/evidence/endgame/hysteresis. |
| No backup | NO_CERTIFIED_BACKUP is an emergency, never safe-stop success. |
| Endgame | Safe F retained when the declared endgame guard blocks unnecessary utility switching. |
| Candidate permutation | Deterministic stable IDs, not array order, decide ties. |
| Env reset isolation | Clear only the supplied env state; other env output/history unchanged. |
| GT firewall | No simulator/task handles, privileged semantic identity or GT target pose accepted online. |

Add local-invalid option controls, matching scenario/path/decision keys, missing
error allowances, over-small comparison families, numerical finite checks,
repeated calls, and positive-vs-negative confidence-bound controls where applicable.
The implementation itself cannot serve as the arithmetic answer generator.

## Verification and interpretation

Use pinned CPU Python3.8 with PYTHONNOUSERSITE=1, CUDA_VISIBLE_DEVICES='', one
thread and python-B. Run focused research tests, then the full BOIA-pattern CPU
suite on the frozen module to catch default-path regressions. The final C02
PUBLIC_CPU result remains attached to its committed source before PART C; do not
retroactively claim a whole-public-suite run for newly added research tests.
Additional broad testing is needed only if focused failures/source coupling reveal
an unresolved concern. Run source/schema/import firewall checks and preservation.

All positive action-selection examples must be labelled EXACT_SYNTHETIC. The
test-only HELD_OUT_DECLARATION example checks conditional MC arithmetic without
claiming an actual data artifact. A CALIBRATED string alone
does not verify an artifact; actual detector/plant/continuation calibration is
NOT_TESTED. Runtime observations measure only these finite kernels, not trajectory
forecasting, sensing, transfer, C02 or full planner deadlines. No GPU/Isaac/learning,
Raw calibration, online activation, staging, commit, push or merge follows.
