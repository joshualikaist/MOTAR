# G4-R3 PART C: isolated CPU research module

Source base: `8689efcd066d64e7b49cecf011e3bd8675380cdb` (C02 already committed).
This document describes new, uncommitted `motar_eval/boia_research/` and
`tests/test_boia_research.py`. Existing BOIA, production and historical tests are
unchanged by this unit. This is **CPU_RESEARCH_ONLY / physical_qualified=False**.
No simulator, detector, tracker, policy, GPU, checkpoint, raw arrays or online
caller is used. Outputs contain candidate IDs, never executable actions.

## Scope and declaration boundary

The implementation evaluates supplied finite surrogate loss tables. It does not
predict loss from sensors, estimate actual capture probability, execute Frozen F
in closed loop, or validate a safety/calibration artifact. Holding the current F
action for a horizon is not assumed equivalent to the actual Frozen F policy.
Actual sensor calibration, F-continuation error, finite-world coverage and plant
safety remain unavailable. Real operation without these artifacts must abstain
from utility switching; B03/B04 and G5 remain blocked.

All positive exact fixtures use an explicitly enumerated, finite synthetic model
with equally weighted atoms. Exactness applies only to that constructed law.
`EXACT_SYNTHETIC`, `HELD_OUT_DECLARATION`, reference, domain, frame alignment,
model exhaustiveness and safety flags are caller declarations. Their truth is not
verified here. A tag or a nonempty reference is not empirical calibration. The MC
test uses a test-only declaration, not real held-out data. No fabricated capture
rates, measured probability intervals or actual continuation errors are reported.

## API and units

All records are frozen dataclasses, Python3.8-compatible, with immutable tuples.
Callers provide records conforming to the declared schema; this is not a parser
for arbitrary Python objects. Numerical invalidity is refused locally where
possible. There are no simulator/task/evaluator handles or semantic GT fields.

| API / record | Contract |
| --- | --- |
| `MeasuredEvidence` | Key `(env_id, episode_id, decision_id)`; first-hit and camera-surface **common-ray range intervals in metres**, not raw camera-Z, horizontal LiDAR range or target-centre GT. Capture/arrival/current time and max age in **seconds** in one clock. Common frame/ray support explicitly declared; no extrinsic or depth conversion is performed here. |
| `ProbabilityInterval` / `validate_probability(interval, domain)` | Supplied closed interval in `[0,1]` plus provenance. Checks ordering, finite numbers, supported reference/domain; returns validity, not a fitted or proved CI. May represent detector/association support; missing interval is not replaced with a heuristic. |
| `build_hypotheses(measurement, domain)` | Returns nonexclusive labels, preserved first-hit intervals, domain and reasons. Never deletes a measured hit or produces a free-space certificate. |
| `LossTable` | Same env/episode/decision, scenario IDs and path IDs must match exactly across F and an option. Rows are scenarios; columns are equally weighted exact atoms or declared independent sampled paths. Values are finite **surrogate loss units**. |
| `paired_bound(id, wait_s, F, option, contract, allowances)` | Returns candidate ID, wait seconds, paired bound, scenario means, error components, domain and validity. Does not generate any losses. |
| `Option` | ID, wait seconds, current supplied safety-valid declaration, utility validity and paired-bound result. ID+wait pair must be unique. |
| `ResearchCase` | Per-env key, baseline ID/safety, options, evidence, strictly increasing finite wait grid starting at0, safety of **each whole wait interval**, switching policy, optional declared backup ID, explicit terminal/endgame/acquired flags. |
| `opportunity_window(case)` | Returns current contiguous window, all supported times, finite-grid closing/censoring flags and preferred ID/time. No continuous-time certificate. |
| `initial_state(env, episode, F_id)` / `decide(case, state)` | Explicit immutable per-env history; decision IDs only. No RNG or hidden mutable global state. A live episode mismatch requires explicit reset. Terminal resets only supplied env history before evidence/loss access. |

Every public result has `scope=CPU_RESEARCH_ONLY` and `physical_qualified=False`.
Domain agreement is required among common evidence, utility and bound results.
The loss-table domain is supplied by its bound contract, not independently inferred.
Decision IDs are nonnegative, nonboolean integers. Explicit state records the last
accepted nonterminal ID. A duplicate/stale ID returns no selected ID and preserves
state; current declarations cannot be reused as a fresh proof. The module has no
scheduler or wall clock, and an initial state still needs caller episode management.

## C1/C2: supported explanations and missing information

1. A first hit overlapping a camera-surface range retains both `TARGET_SURFACE`
   and `FOREGROUND_OCCLUDER`: equal ranges do not reveal physical identity.
2. A hit strictly nearer than the surface interval retains `FOREGROUND_OCCLUDER`.
   A contradictory farther hit adds `FALSE_ASSOCIATION` and refuses utility.
3. An association interval with lower endpoint below1 retains
   `FALSE_ASSOCIATION`. Utility must cover this alternative too; a low association
   score is not silently treated as a confirmed target ID.
4. Missing/invalid ranges, unsupported calibration, absent common-ray/frame support,
   stale/inconsistent times, or nonexhaustive hypothesis declaration produce
   `UNKNOWN_OR_UNOBSERVED` and utility refusal. Missing intervals are not invented.
5. Even a valid nonexclusive set is conditional on the supplied model family.
   It does not solve the actual sparse-ray/self-return identifiability problem.

## C3: paired arithmetic and distinct error terms

For every scenario `b` and matched path `m`, calculate
`d[b,m] = L_F[b,m] - L_option[b,m]`, then

```
raw = min_b mean_m d[b,m]
A_low = raw - eps_MC - eps_set - b_cal - b_F_continuation - c_switch
```

All five allowances are explicitly finite, nonnegative and present. Missing
allowances refuse the option instead of defaulting to0. The continuation scope
must be declared covered. These losses/allowances share one loss scale. Switching
cost is not substituted for calibration or F-continuation uncertainty.

`EXACT_SYNTHETIC` requires enumerated equally weighted atoms, exhaustive scenario
declaration, nonempty justification and zero first-four error terms. A switching
cost may be positive. This is an exact finite surrogate, not a statistical CI.

For the sampled declaration mode, require independent paths, bounded paired
differences `|d| <= D`, `alpha in (0,1)`, and exhaustive worlds or separately
declared coverage with an explicit remainder. With `O` tested ID/wait options,
`B` scenarios and `M` paths, the supplied allowance must satisfy

```
eps_MC >= D * sqrt(2 * log(2 * O * B / alpha) / M).
```

This is a simultaneous Hoeffding/union-bound check conditional on the declarations.
Forecast time knots within a path are not extra iid samples. Selection rejects an
option formed for fewer choices than the entire supplied option family, including
tested wait choices. It does not estimate independence, coverage or calibration.
Nonexhaustive finite scenarios without a declared coverage argument are refused.

NaN/Inf, unrepresentable huge integers, extreme finite sum overflow, missing
allowances, domain mismatch and mismatched IDs refuse an option. A valid sibling
remains eligible. Shared evidence invalidates this env's utility, not other envs.

## C4: opportunity timing changes the decision

An option is supported only if current evidence, matching bound and utility domains,
whole-interval waiting-prefix safety, option safety and positive net bound above
`tau_on` all hold. Prefix safety at `t_j` is the conjunction of every declared
interval `[t_i,t_(i+1)]` for `i<j`; a safe endpoint cannot erase an earlier false.

`supported_times_s` contains every supported grid knot. `current_window_s` is only
the consecutive supported prefix beginning at0. If the current set is empty, it
remains empty even when a later positive set appears. Later reappearance does not
bridge a gap. `closing` only reports that this current finite-grid prefix ends
before the final knot; it does not itself influence cost or prove mission benefit.
`horizon_censored` means the current prefix reaches the grid boundary, not that a
continuous window or eventual loss time has been certified.

The actual timing mechanism selects the largest supported net bound across
ID/wait options. Ties select earliest wait, then lexicographically smallest stable
ID. A preferred immediate option can produce `INTERVENE`. A preferred later option
produces `KEEP_F / WAIT_FOR_LATER_OPPORTUNITY` now, with an advisory future ID/time;
no future action is queued. Re-evaluation and current evidence are needed later.

Two explicit comparisons against independent rules under identical synthetic
loss/safety/info demonstrate a decision difference, not better capture:

| Exact surrogate | Independent rule | This module |
| --- | --- | --- |
| Observed now; net gains now3, later−1/−2 | Actual-loss-only reactive rule keeps F while currently observed | INTERVENE now; current window `{0}`, closing flag true. |
| Net gains now1, at0.2s3, at0.4s−1; entire wait safe | Immediate-only threshold accepts1 now | KEEP_F now, advisory wait0.2s. |

## C5: precedence, outcomes and state

1. Terminal is a no-action reset result; loss/evidence inputs are not accessed and
   the last decision ID is cleared. Otherwise an invalid/duplicate/stale ID returns
   `NO_CERTIFIED_BACKUP` with no selection/state advance: this denotes an invalid
   **current proof**, not proof of unavoidable physical collision.
2. For a fresh case, unsafe/undeclared F triggers safety handling before utility or endgame. Only a
   unique, immediate, same-case supplied safety declaration can name a backup.
   A retained ID from an old state is insufficient. Conflicting duplicate IDs fail.
3. With safe F, missing common evidence or available safe alternatives without any
   supported comparison yield `ABSTAIN`. This is not an unsafe execution override.
4. Safe F with no safety-admissible alternative yields `KEEP_F / NO_SAFE_ALTERNATIVE`.
   A valid nonpositive comparison yields `KEEP_F / NONPOSITIVE_SUPPORTED_BENEFIT`.
5. Declared endgame/no-acquisition guard retains safe F. Endgame is caller-supplied;
   actual capture is not detected here. Safety override still has precedence.
6. Positive supported timing may select `INTERVENE`; a future optimum keeps F.
7. Hysteresis is last. The implemented scope is **current owner-to-F retention**.
   A currently safe, utility-supported, nonnegative owner can be retained for the
   minimum hold/off-count rules. Unsafe, unsupported or negative owners are
   released immediately. This is not full multi-owner switching optimal control.

`SAFETY_OVERRIDE` selects a declared candidate ID, not a physical guarantee.
`NO_CERTIFIED_BACKUP` has no selected ID and never denotes successful safe stop.
Exactly the five requested outcomes are represented. All new histories are returned
explicitly; inputs and other envs are unchanged.

## Independent CPU verification

Command (pinned Python3.8, GPU hidden):

```
PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' /home/fair/miniconda3/envs/aerialgym/bin/python -B -m unittest discover -s tests -p test_boia_research.py -v
```

Local focused run: **30 tests PASS, 0 failures/errors, 0.008s**. Root performs final
frozen-source regression separately. The earlier `tests.test_boia_research` module
command failed discovery because this repository's tests directory is not a Python
package; the corrected discovery command above runs the actual tests.

Tests01–18 cover every user-required fixture family in order. Tests19–30 add exact
context/scenario/path pairing, distinct-world paired regret, missing error budgets,
simultaneous sampled-option accounting and coverage refusal, numeric overflow
isolation, domain binding, hysteresis validity, RNG/immutability, probability/time/
grid/flag validation, unsupported-vs-negative distinction, duplicate backup IDs and
stale-decision refusal without advancing another env or the current dwell counter.
Expected numbers are scalar arithmetic, not old BOIA output:

- Worlds `(F,A)=(10,9),(1,0)` have paired means `(1,1)` and robust gain1. The
  unrelated `min(F)-max(A)` is−8. Replacing the second option loss with3 gives
  paired gains `(1,−2)`, so the robust bound is−2.
- Switching cost0.2 makes gain0.8; cost1.2 makes gain−0.2 and prevents switching.
- In the sampled arithmetic fixture `O=6,B=2,M=100,D=1,alpha=.05`; the explicit
  bound is `sqrt(2*log(480)/100)`. A99%-sized allowance fails. A one-option bound
  cannot be reused when two alternatives are selected between.
- False earlier wait interval followed by true interval rejects the later gain5;
  supported now gain1 remains available. One invalid/nonnumeric/wrong-domain option
  cannot erase a supported sibling gain2. Conflicting duplicate backup IDs fail.

The AST/schema tests permit only internal imports and math/dataclasses/typing in
the new package, reject privileged field/access names, and verify no command/action
output. Hidden oracle-world labels live only in tests; identical measured packets
return identical hypotheses. These are focused schema checks, not a universal
information-flow proof. The package never accepts existing BOIA/task objects.

## Cost, limitations and next gate

For `O` ID/wait options, `B` worlds and `M` matched paths, paired-bound evaluation
is `O(B*M)` work/storage per option. Current selector uses duplicate counting and
grid scans, so it is `O(O^2 + O*J + O*log(O))`, not linear. Evidence construction is
constant-sized. Costs exclude trajectory generation, perception, transfer, plant
safety, calibration and Frozen F simulation. Tiny test runtime is not an online
deadline measurement; no GPU latency or closed-loop feasibility is established.

Results establish arithmetic and declared-condition selection behavior only.
Actual capture benefit, safety, calibration, uncertainty-set coverage, identity
correctness, actual F continuation and novelty remain **NOT_TESTED / UNVERIFIED**.
The paired regret formula is prior art; finite timing is a candidate mechanism to
test, not a first-in-literature claim. No PART C auto-commit or online connection,
no G5 release, and no new GPU/Isaac/learning experiment follows this CPU unit.
