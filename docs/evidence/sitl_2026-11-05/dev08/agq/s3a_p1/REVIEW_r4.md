# Source review r4: AGQ geometry diagnostic (unit A) and S3a-P1 policy precision (unit B)

| Unit | Verdict | Open after r4 |
| --- | --- | --- |
| **A**, AGQ geometry-v1 | **PASS_WITH_LIMITS** | BLOCKER 0, MAJOR 0, MINOR 2 (R4-m1, R4-m2), LOW 6 |
| **B**, S3a-P1 policy precision | **PASS_WITH_LIMITS** | BLOCKER 0, MAJOR 0, MINOR 0, LOW 6 in the changed parts (residuals N11(d), N14 module-path check, N13; new B4-l1 to B4-l3), plus r1–r3 LOWs not re-scored |

**Fix before commit.** Both of unit A's MINORs are false sentences in the preregistration, and the preregistration
cannot be edited once it is committed:
- §6.5 claims the exposure reference does not depend on where `t_c` falls between checkpoints (R4-m1);
- §8 describes the floor-impact onset wrongly (R4-m2).

**The round-3 MAJOR is fixed.** R3-M1 is resolved: r3's band case and its paired control now give the same reading
at both phases.

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context. This was an independent, read-only audit (round 4).
- **Date:** 2026-11-06, 03:32–03:47 KST.
- **Object:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD `0abb766`. The 7
  files under review are untracked.
- **What I wrote:** only under `source_review_r4/`.
  - Nothing went into any worktree, and I changed no Git state. I ran only `git status`/`rev-parse`, and
    `git_state()` ran inside the unit-B tests.
  - Every probe and test ran with `TMPDIR=source_review_r4/tmp/`, which is empty at the end.
- **No simulator and no GPU.** I started no gzserver, gz, px4, Gazebo or SITL process. `CUDA_VISIBLE_DEVICES=""` was
  set for every process, and unit B's probe had CUDA hidden whenever torch was imported.

## Hash check (`sha256sum -c BYTES_UNDER_REVIEW.sha256`)

| File | SHA-256 | Start 03:32 | End 03:46 (after writing this report) |
| --- | --- | --- | --- |
| tools/sitl_agq_geometry_diag.py | `80b42034…` | OK | OK |
| tools/sitl_agq_geometry_launch.py | `189e05ef…` | OK | OK |
| tests/test_sitl_agq_geometry_diag.py | `5674227d…` | OK | OK |
| docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md | `5e74dfc2…` | OK | OK |
| tools/sitl_s3a_policy_precision.py | `6544f5a4…` | OK | OK |
| tests/test_sitl_s3a_policy_precision.py | `14369281…` | OK | OK |
| docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md | `9ef9dd83…` | OK | OK |

**Changes in the worktree during the review.** These were not mine, and none of them touches the reviewed bytes:
- `docs/evidence/sitl_2026-11-05/dev08/agq/geometry_v1/README.md` was created at 03:43:39;
- `docs/evidence/sitl_2026-11-05/dev08/agq/s3a_p1/README.md` was created at 03:43:47.

Neither is reviewed here.

**A stale bytecode file.** `tools/__pycache__/sitl_agq_geometry_diag.cpython-38.pyc` (03:27:08) existed before I
started.
- It was written 2 s after the diag tool's last edit (03:27:06), so some process imported the current tool without
  `-B`.
- My runs did not change it: its mtime was the same at the end.
- It is gitignored, but it is a runtime file under `tools/`. Delete it before the source freeze (AGENTS.md).

---

# Unit A: AGQ geometry diagnostic

## Round-3 findings

| ID | r3 severity | r4 status | Evidence |
| --- | --- | --- | --- |
| **R3-M1**: the exposure reference is a pre-contact checkpoint | MAJOR | **Resolved.** Residual: R4-m1, MINOR | `_path_at` (diag:692-699) interpolates N linearly at `t_c` (diag:767-768). On r3's R7 case with the current bytes (`r4_exposure_probes.json` P1), N's reference is 0.25 m, exactly its clearance at contact, for both δ = 0.09 s and δ = 0.01 s: X at 0.33 m reads INCONCLUSIVE_NOT_EXPOSED both times (r3 got CONSISTENT at δ 0.09). In the paired positive control, X at 0.24 m reads CONSISTENT_WITH_ROTOR_CONTACT both times. The new test `test_exposure_does_not_depend_on_sampling_phase` (test:547-564) pins 0.31 m against 0.39 m. The author's mutant `exposure_last_checkpoint` and my mutants `path_at_next_checkpoint` and `path_at_nearest` are killed. **This holds only for uniform motion between the two checkpoints around `t_c`; see R4-m1.** |
| R3-m1: empty reference window | MINOR | **Resolved** | The reference no longer has a lower window bound. `final_sim_time_s` is the last checkpoint's `sim_s` (`sitl_s3b_dev08_prefix.py:253`), and reproduction requires `t_c ≤ onset ≤ fin_n`. So whenever the exposure step is reached, `t_c` lies within N's path, and the reference is always defined (the only exception is a contact before N's first checkpoint). |
| R3-m2: non-finite `clearance_m` | MINOR | **Resolved in code; untested (LOW)** | `_finite_or_none` (diag:670, :674-675). Probe `nonfinite_probe_r4.log`: inf, −inf, NaN and None give `null`, finite values pass, and `json.dumps(allow_nan=False)` succeeds. The author's mutant `nonfinite_clearance_kept` **survives** (`mutants_v5/run.log`), so no test covers the fix. |
| R3-m3: a floor-impact onset just before N's final checkpoint | MINOR | **Not resolved; the new prereg text is false** (R4-m2) | The code is unchanged (diag:735-744). Prereg §8 :287-288 now describes the case, incorrectly. |
| R3-m4: CONTACT_PERSISTS keyed on the first window | MINOR | **Resolved** | Prereg :208-210 now says "the first rate window of X that overlaps it", which equals diag:780. |
| R3-m5: unpinned boundaries | MINOR | **Partially resolved (LOW residual R4-l2)** | Now killed: exposure phase, X window widened, crash time ignored, in-progress window 0.2→0.3, ULog unbound (the fixture writes `fake.ulg`, test:688, :750-753), envelope gate. The launcher's refusal of an existing `attempt01` is tested (test:919-921). Survivors: see R4-l2. |
| R3-m6: wording | MINOR | **Resolved** | The ledger fields are listed (:77-79). The clock check's resolution is stated (:221-222). The budget names the post-member checks (:265-266). The onset filter sits under item 2, whose bullets include the not-reproduced branch (acceptable). |
| R3-m7: the test subprocess wrote bytecode | MINOR | **Resolved** | `sys.executable, "-B", "-c"` (test:899). My test run created no `.pyc`, and the existing one kept its mtime. |

## The author's mutant run v5 (a claim to check)

`mutants_v5/run.log`, complete (my `cat` at about 03:44):
- the control passes, and 10 of the 12 mutants are killed;
- **2 survive: `box_checkpoints_only` and `nonfinite_clearance_kept`**;
- the run's own summary line is `ALL_KILLED_AND_CONTROL_PASSES False`.

So the claim "all targeted repairs are pinned" does not hold:
- evaluating X only at its checkpoints, instead of interpolating every 5 ms, passes the tests;
- so does keeping a non-finite clearance.

Neither survivor makes a reading wrong by itself:
- checkpoint-only sampling can only *raise* X's minimum, which is conservative;
- the non-finite fix is correct, as the probe shows.

They are test gaps (R4-l2).

## Adversarial check of the changed code (`_path_at`, `_path_samples`, exposure)

| Case | Result (`r4_exposure_probes.json` P3/P4) |
| --- | --- |
| `t_c` exactly on an inner checkpoint | That checkpoint's position: the segment before it, with w = 1. Correct. |
| `t_c` at the first or last checkpoint | p0 and p_last. Correct. |
| `t_c` before the first or after the last checkpoint (±1e-12) | `None`, so INCONCLUSIVE_NOT_EXPOSED. Conservative. It cannot be reached under reproduction (see R3-m1). |
| A single-checkpoint path, even at its own time | `None`. Not reachable in practice. |
| Duplicate checkpoint times | `w = 0`, the first one. No division by zero. |
| Sample grid | 201 samples for a full ±0.5 s window, spaced 5 ms. Clipped to the path ends: 141 samples at `t_c = 0.2`, 161 at `t_c = 12.7`. An empty path or a window outside the path gives `[]`, so NOT_EXPOSED. |
| Units | m and s throughout. `bar_clearance` is unchanged; r3 verified it against the runner to 2e-16 m. |
| X's window | `[t_c−0.5, t_c+0.5]`, independent of the 1.5 s rate window. Correct per prereg §6.5. |
| Direction of the remaining sampling errors on X | Grid spacing: X's minimum is overestimated by at most v·2.5 ms ≈ 3 mm at 1.1 m/s. Chords cut the corners of a curved closest approach. Both make X look *farther*, so both are conservative. |
| Favourable by construction? | No, with one conditional exception for N: R4-m1. |
| Valid case rejected? | Only the floating-point edge case in R4-l1. |

## New findings, unit A

### MINOR

**R4-m1. The interpolated reference still depends on the contact's phase when N's velocity changes at the
contact. Prereg §6.5 says it does not.**

Where:
- `tools/sitl_agq_geometry_diag.py:767` (comment "no sampling-phase bias");
- prereg :201-202 ("The reference therefore does not depend on where `t_c` falls between N's 10 Hz
  checkpoints").

The mechanism:
- The reference is the chord between N's checkpoint before `t_c` and its checkpoint after `t_c`. The second of these
  comes after the contact, by up to 0.1 s.
- If N's approach stops at the contact (an inelastic stop), the chord at `t_c` exceeds N's true clearance by
  v·δ₁·δ₂/Δ. Here δ₁ and δ₂ are the times from the two checkpoints to `t_c`, and Δ is their spacing. At most this
  is v·Δ/4, about 0.028 m at 1.1 m/s.
- If N bounces back, the excess doubles.
- This excess favours the hypothesis: X reads exposed more easily.

Probe P2 (N touches at 0.25 m; X holds 0.27 m, outside N's rotor reach at contact):

| N after the contact | δ₁ = 0.01 s | δ₁ = 0.05 s | δ₁ = 0.09 s |
| --- | --- | --- | --- |
| stops (e = 0) | ref 0.2599 → NOT_EXPOSED | ref **0.2775 → CONSISTENT_WITH_ROTOR_CONTACT** | ref 0.2599 → NOT_EXPOSED |
| reverses (e = 1) | ref 0.2698 → NOT_EXPOSED | ref **0.305 → CONSISTENT** | ref 0.2698 → NOT_EXPOSED |

The predicted values match to 1e-15.

How plausible is this?
- The rotor collisions have no `<bounce>`, and their contact has `max_vel 0` (`navrl_quad.sdf`), so an inelastic
  stop at the contact point is physically plausible.
- On the other hand, the only old trace (env5 native, proxy `t_c`) kept approaching at about 1.1 m/s through the
  bracketing interval (r3 `r7_exposure_band.json`: 0.2755 → 0.1674 m over 0.096 s). There the chord is accurate.
- Whether N decelerates at a real rotor contact is NOT_VERIFIED.
- The residual is about a quarter of R3-M1's size (≤ 0.028 m against ≤ 0.11 m), and it needs X to stop in a narrow
  band just outside N's contact clearance. That is why it is MINOR, not MAJOR.

Fix, before the preregistration is committed:
1. **Code (preferred).** Take the reference as the *smaller* of two values:
   - the interpolated value;
   - the value extrapolated from N's last two checkpoints at or before `t_c`, which uses pre-contact data only.

   For uniform motion the two agree. For a stop or a bounce, the extrapolation is exact. Add the P2 pair as a test.
   Using the 50 Hz ULog truth would also work, and would cut the bound to about 0.006 m.
2. **Text (at minimum).** Scope §6.5's sentence: the reference is phase-independent when N moves uniformly between
   the two checkpoints. A change of N's velocity at the contact can raise it by up to v·Δ/4 (about 0.03 m at 1.1 m/s),
   which favours exposure. Fix the code comment at diag:767 to match.

**R4-m2. Prereg §8's new floor-impact sentence is false.**

Where:
- prereg :287-288: "It reads DISTURBANCE_WITHOUT_CONTACT only when no earlier onset or Z0.8 precedes it, and Z0.8
  normally does";
- diag:740-744.

What the code does:
- It reads the *earliest disturbance that has no contact in the 0.5 s before it*.
- An earlier onset or Z0.8 that has a contact before it does not stop the later floor-impact onset from giving
  DISTURBANCE_WITHOUT_CONTACT.
- An earlier free Z0.8 gives DISTURBANCE_WITHOUT_CONTACT as well.

So the sentence's mitigation never applies.

Probe `r4_floor_onset_probe.json`:
- rotor contact at 5.73; a rate window at 6.3, which is 0.57 s later, so the contact is not reproduced, and it has a
  second contact at 6.2–6.25 before it; a floor onset at 6.95; final at 7.0. This reads
  **DISTURBANCE_WITHOUT_CONTACT, onset 6.95**;
- the same with Z0.8 at 6.4, also contact-preceded: **DISTURBANCE_WITHOUT_CONTACT, onset 6.95**;
- the control, without the 6.95 onset: INCONCLUSIVE_NOT_REPRODUCED.

The bias runs against the rotor hypothesis, but the text misdescribes the fixed rule.

Fix: replace the sentence with one that is true. For example: "A rate onset from a floor impact shortly before N's
final checkpoint counts like any other. If it is the earliest disturbance with no contact in the 0.5 s before it,
the pair reads DISTURBANCE_WITHOUT_CONTACT, even after a contact-preceded excursion." Alternatively, implement r3's
remedy for R3-m3 and state it.

### LOW

| ID | Location | Finding | Fix |
| --- | --- | --- | --- |
| R4-l1 | diag:706-712, :771 | **Floating-point accumulation in the 5 ms grid.** `t += step` puts the sample meant for `t_c` at `t_c + 1.07e-14`. Combined with "no tolerance", a box member whose path is *identical* to N's reads INCONCLUSIVE_NOT_EXPOSED when N's clearance has a strict minimum exactly at `t_c` and `t_c` coincides with a checkpoint. The box member's minimum exceeds the reference by 1.18e-14 m (probe P4, `identical_V`). Without a checkpoint at `t_c`, identical paths read CONSISTENT (P4, `identical_chord`). This is practically unreachable. | Generate the grid as `lo + i*step`, and add `t_c` and X's own checkpoint times inside the window to the samples. |
| R4-l2 | test file | **Survivors among targeted mutants** (`mutants_r4_results.json`, Reading class; `mutants_v5/run.log`): `box_checkpoints_only` (author's), `nonfinite_clearance_kept` (author's), `in_progress_inclusive` (`w[0] <= t_c`; still alive from r3), `exposure_step_50ms`, `box_window_narrow_0p2` (the X exposure window is pinned against widening and one-sidedness only), `box_after_1p0` (the 1.5 s rate window is unpinned from below). `samples_no_end_clamp` is equivalent in practice. The launcher's `run01` refusal (`RUN.mkdir(exist_ok=False)`, launch:93) is still untested. | Add paired cases: a tangential pass whose minimum lies between X's checkpoints; an inf `clearance_m` row through `analyze` with `json.dumps(allow_nan=False)`; a native rate window starting exactly at `t_c`; an X approach at `t_c ± 0.3`; a box disturbance at `t_c + 1.2`. |
| R4-l3 | prereg :201 | "X's every 5 ms over its window" can be read as item 4's "X's window" (`[t_c−0.5, min(t_c+1.5, final)]`). The code samples `[t_c−0.5, t_c+0.5]`, as item 5's first sentence says. | Write "over `[t_c − 0.5, t_c + 0.5]`". |
| R4-l4 | `mutants_v5` | The repair was presented as fully pinned, but the run's own summary is `ALL_KILLED_AND_CONTROL_PASSES False`. | Report the two survivors, or close them (R4-l2). |
| R4-l5 | `tools/__pycache__/` | Stale `sitl_agq_geometry_diag.cpython-38.pyc` (03:27:08), from an import of the current bytes without `-B`. Not from the reviewed tests. | Delete it before the freeze. |
| R4-l6 | diag:770 | `box_clearance_to_bar_m` and `native_clearance_to_bar_m` would be `inf` if every sample lay outside the bar's height range, and `--analyze` would then crash in `json.dump(allow_nan=False)`. This cannot be reached here: X must stay within 0.3 m of N until `t_c − 0.2`, N touched bar b, and the bars span z 0–3 m. | Optional: map through `_finite_or_none` as well. |

## Tests (unit A)

- `python -B tests/test_sitl_agq_geometry_diag.py -v`: **25/25 OK in 63.6 s**, rc 0, max RSS 53 MB
  (`test_agq.log`, cores 16–17).
- **Portability and leaks:**
  - the Reading tests are pure;
  - the Unit fixture writes under `TMPDIR` and cleans up (`tmp/` is empty at the end);
  - the Launcher test runs read-only `git rev-parse`/`status` on the real worktree;
  - its envelope subprocess uses `-B`;
  - nothing new appeared in the worktree from my runs (`find -newer .marker_before` shows only the two evidence
    READMEs written by another session at 03:43), and the existing `.pyc` keeps its mtime.
- **Not run:** the tests without `-B`. The test's `load()` uses `exec_module`, which writes bytecode unless `-B` or
  `PYTHONDONTWRITEBYTECODE` is set. The documented commands set it.

---

# Unit B: S3a-P1 policy precision

## Round-3 findings

| ID | r3 severity | r4 status | Evidence |
| --- | --- | --- | --- |
| **N12**: `density_readings` carries labels under non-cause readings | MINOR | **Resolved** | `result["density_readings"]` is set only if `reading ∈ CAUSE_READINGS` (the three cause readings and MIXED; tool:50, :280-282), otherwise `None`. On VOID it moves to `density_readings_if_valid` and becomes `None` (tool:413-415). The rule is checked inside the real `measure`, with forced headline readings (`s3a_probe_r4.json` `null_rule`): the four cause readings carry the per-density dict, and GPU_REPLAY_NOT_FAITHFUL, CONTROL_FAILED, NONFINITE_OUTPUT and CPU_ONLY give `None`. Tests: test:234 (VOID moves it) and test:319 (NOT_FAITHFUL gives null). Prereg :68-71 matches exactly. `cpu_only_v3/result.json` has `density_readings: null` (v2 had `{"70": null, "205": null}`). |
| N11(a): `1e999` in the approval | LOW | **Resolved** | `_strict_json` passes a `parse_float` that rejects non-finite values (tool:332-341). Probe: `1e999`, `-1e999`, `1E400`, `NaN`, `±Infinity` and nested `[{"y": 1e999}]` are all refused with rc 2, before any directory exists. Positive controls `0.15`, `1e308` and a 400-digit integer are accepted. Test at test:186-189. |
| N11(b): `CUDA_VISIBLE_DEVICES=-1` | LOW | **Resolved** (new LOW B4-l1) | `re.fullmatch(r"\d+", …)` (tool:374). Probe: `-1`, `""`, unset, `0,1`, `" 0"`, `"0 "` and `GPU-1234` are refused before `mkdir`. `0`, `1` and `00` are accepted. |
| N11(c): pins checked before `mkdir` | LOW | **Partially resolved** (B4-l2) | The G8 input pins are checked before `mkdir` (tool:376-379; test:194-196). Policy F's pin is still checked only inside `measure` (tool:248), after `mkdir`. |
| N11(d): post-run `git_state()` not caught | LOW | **Open** | tool:408. If `git` fails after the measurement, neither `result.json` nor `aborted.json` is written. |
| N11(e): reading of a signal kill | LOW | **Resolved** | Prereg :117-118: ABORTED, and the GPU conditions count as NOT_RUN. |
| N14: guard timing, scope and PYTHONPATH | LOW | **Mostly resolved** | **Timing:** the guard now runs right after each density's CPU player and before any forward pass (tool:258-259). In `guard_probe_r4.log`, a foreign top-level `aerial_gym` raises after **0** forward passes (r3: 2). The control passes after 2 passes, with the path inside the worktree. **PYTHONPATH:** now in the §5 command (prereg :106). **Open (LOW):** the `loaded_module_files` paths are still not checked to lie inside ROOT. |
| N13: editorial | LOW | **Open** | Prereg :66 still reads "This applies when either density is unfaithful: For either density, …". Items 4 and 5 still say "for both densities". New in r4: :68 ("Otherwise each density …") follows item 3 without a blank line, so CommonMark renders it as a lazy continuation inside item 3. |
| l1–l7, N3, N5, N6, N8–N10, m3/m5 residuals | LOW | Unchanged by this repair | Not re-scored. |

## Adversarial check of the changed code

- **Null rules.** They are keyed only on the headline reading, as §4 states. MIXED can never include
  GPU_REPLAY_NOT_FAITHFUL, because that label is global, so `density_readings` under MIXED holds only cause labels.
  On VOID, `density_readings_if_valid` is null whenever the valid reading would have been null. That is consistent.
- **Strict JSON.**
  - An overflowing *integer* literal is a Python int and serialises, so it is harmless.
  - Invalid UTF-8 raises `UnicodeDecodeError`, a `ValueError`, which gives REFUSED.
  - Deep nesting raises `RecursionError` in `json.loads`, before `mkdir`, so no directory is spent.
  - A non-string `commit` is reported as "missing commit" (r3's N13 note).
- **CUDA check.** It is a strict improvement. It still accepts non-ASCII digits and indices of absent GPUs (B4-l1).
- **Valid case rejected?** None found in the changed refusals. The GPU-UUID form `GPU-…` is refused. That form is
  valid CUDA, but the preregistration requires an index (:110), so this matches.

## New findings, unit B

| ID | Severity | Location | Finding | Fix |
| --- | --- | --- | --- | --- |
| B4-l1 | LOW | tool:374 | `\d` is Unicode-aware. `CUDA_VISIBLE_DEVICES="٣"` (Arabic-Indic three) passes the check, and the attempt directory is created (probe). An index with no GPU (for example `7`) also passes, and the attempt is then spent in `aborted.json`. Both need deliberate misuse. | `re.fullmatch(r"[0-9]+", …)`. Optionally check `torch.cuda.device_count()` in a child process before `mkdir`. |
| B4-l2 | LOW | tool:248; prereg :109-111 with §2 :32 | §5 says the run refuses "before anything is created" if "an input differs from its pin", and §2 lists Policy F among the pinned inputs. F's SHA-256 is checked only inside `measure`, after `mkdir`. A changed F still cannot be misreported, because it gives `aborted.json`, but the attempt is spent and the sentence overstates the check. | Hash `B.F_CKPT`'s path before `mkdir`, or write "a G8 input differs from its pin (Policy F's pin is checked at the start of the measurement)". |
| B4-l3 | LOW | test:164-206 | The refusal tests call the real `verify_inputs()` on the host's G8 files and assert only rc 2, not the reason. On a host without G8, every case would refuse for the inputs reason, so the CUDA, `1e999` and NaN cases would pass without testing their own refusal. On this host each case refuses for its own reason (probe). | Patch `verify_inputs` to `{}` in that test, as the accept-path test does, and assert the problem text. |

## Preregistration against the code (changed parts)

| Prereg | Code | Match |
| --- | --- | --- |
| :68-71 `density_readings` only for the four readings; null otherwise; moved on VOID | tool:50, :280-282, :413-415 | Exact |
| :106 command with `CUDA_VISIBLE_DEVICES=0` and `PYTHONPATH=<worktree>` | the CUDA check accepts `0` | Exact |
| :107-108 string keys; strict JSON | `approval_problems` and `_strict_json` | Exact |
| :109-114 refusals before anything is created | tool:369-391 | True, except Policy F's pin (B4-l2) |
| :115-116 guard right after the first CPU player, before any GPU work | tool:258-259 | Exact. It also runs for the second density. |
| :117-118 signal kill reads ABORTED | a reading rule only | Consistent |
| :101 `cpu_only_v3` from "the bytes committed with this file" | `cpu_only_v3/tool.sha256` = `6544f5a4…`, the reviewed bytes. Tool mtime 03:29:39 < outputs 03:32:03. The npz `83c26c21…` is byte-identical to v1 and v2. Every common JSON field equals v2's except `density_readings` (null against `{"70": null, "205": null}`). | True, provided exactly these bytes are committed |

## Tests (unit B)

- `python -B tests/test_sitl_s3a_policy_precision.py -v`: **13/13 OK in 28.3 s**, rc 0, max RSS 0.73 GB
  (`test_s3a.log`, cores 18–19).
- The children all use `-B`, and nothing new appeared in the worktree.
- The tests run `git rev-parse`/`status` and `journalctl -k` read-only (`test_refusals`).
- **Portability:** the refusal tests use the host's G8 inputs (B4-l3). The two stored-input tests are skipped when
  G8 is absent.

---

## Commands run

All Python ran under `CUDA_VISIBLE_DEVICES="" PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
PYTHONDONTWRITEBYTECODE=1 [TMPDIR=source_review_r4/tmp] timeout -k 10 900 nice -n 10 taskset -c <16,17 | 18,19 |
16-19> bash -c 'ulimit -v 12582912; /home/fair/miniconda3/envs/aerialgym/bin/python -B …'`.

1. `sha256sum -c BYTES_UNDER_REVIEW.sha256` at the start and the end. `git rev-parse HEAD` and `git status --short`.
   `find <worktree> -newer .marker_before`. `stat` of the existing `.pyc`.
2. The tests: `tests/test_sitl_agq_geometry_diag.py -v` (cores 16–17) and `tests/test_sitl_s3a_policy_precision.py -v`
   (cores 18–19), under `/usr/bin/time -v`. Output in `test_agq.log` and `test_s3a.log`.
3. `scripts/r4_exposure_probes.py` (`c7eac579…`) → `r4_exposure_probes.json` (`35051b32…`):
   - P1 r3's R7 band case with its paired control;
   - P2 phase dependence under an inelastic stop or a bounce;
   - P3 interpolation boundaries;
   - P4 identical paths.
4. `scripts/r4_mutants.py` (`e9dc44c7…`) → `mutants_r4_results.json` (`cb6795f2…`) and `mutants_r4.log`: 11 mutants
   of the reading code against the Reading test class. 6 are killed and 5 survive (`samples_no_end_clamp` is
   equivalent).
5. `scripts/r4_floor_onset_probe.py` (`3adeb472…`) → `r4_floor_onset_probe.json` (`7e200ab8…`).
6. A one-liner on `member_events` with non-finite clearances → `nonfinite_probe_r4.log`.
7. `scripts/guard_probe_r4.py` (`1dda0cc1…`; r3's probe pointed at r3's fake package), cases `top_outside` and
   `control` → `guard_probe_r4.log`.
8. `scripts/s3a_probe_r4.py` (`30290280…`) → `s3a_probe_r4.json` (`03ab7b90…`), with run directories under
   `s3a_probe_runs/`:
   - the CUDA values;
   - strict-JSON cases with positive controls;
   - the null rule inside the real `measure`, with forward passes stubbed and the reading forced.
9. Read-only:
   - r3's REVIEW files for both units, and r3's `r7_exposure_band.py`/`.json`;
   - `mutants_v5/run_mutants.py` and `run.log`;
   - `cpu_only_v1/v2/v3`: hashes, mtimes and a field comparison;
   - `navrl_quad.sdf` (the collision surfaces);
   - `sitl_s3b_dev08_prefix.py:253`;
   - the launcher.

## Limits

- No live Gazebo, PX4 or GPU run. These are NOT_VERIFIED:
  - N's real motion through a rotor contact, which decides whether R4-m1 matters;
  - the real checkpoint phases;
  - every GPU reading of unit B.
- R4-m1's magnitudes come from synthetic straight-line trajectories with an idealised stop or bounce. They are a
  bound from reasoning, not a measured misclassification rate.
- My mutants ran against the Reading class only. The author's v5 run covered the full test file.
- The full PUBLIC_CPU suite (`tools/run_tests.py`) was not run.
- The two evidence READMEs created during the review, and `docs/EXPERIMENTS.md` (modified), were not reviewed.
- Unit B's residual LOWs from r1–r3 (l1–l7, N3, N5, N6, N8–N10) were not re-scored beyond the rows above.
